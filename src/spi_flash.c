#include "spi_flash.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(_WIN32)
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#else
#include <io.h>
#endif

/* Winbond W25Q512JV default: 512 Mbit (64 MB). */
#define SPI_FLASH_CHIP_COUNT       2u
#define SPI_FLASH_DEFAULT_CHIP_MB  64u
#define SPI_FLASH_UNIT_MB          32u
#define SPI_FLASH_MAX_CHIP_MB      256u
#define SPI_FLASH_DEFAULT_DIR      "flash"
#define SPI_FLASH1_DEFAULT         "flash/spi-flash1.bin"
#define SPI_FLASH2_DEFAULT         "flash/spi-flash2.bin"

typedef struct {
    int enabled;
    int use_default_path;
    char *path;
    uint32_t size_mb;
    FILE *fp;
#if !defined(_WIN32)
    uint8_t *map;
    size_t map_bytes;
#endif
} spi_flash_chip_t;

typedef struct {
    int valid;
    uint64_t next_off;
} spi_flash_seq_t;

static spi_flash_chip_t spi_flash_chips[SPI_FLASH_CHIP_COUNT] = {
    { .size_mb = SPI_FLASH_DEFAULT_CHIP_MB },
    { .size_mb = SPI_FLASH_DEFAULT_CHIP_MB },
};

static spi_flash_seq_t spi_flash_seq[SPI_FLASH_CHIP_COUNT];
static int spi_flash_logged;
static int spi_flash_dir_created;
static uint32_t spi_flash_fflush_pending;
static uint32_t spi_flash_msync_pending[SPI_FLASH_CHIP_COUNT];

static uint64_t spi_flash_chip_bytes(const spi_flash_chip_t *chip) {
    return (uint64_t)chip->size_mb * 1024u * 1024u;
}

static uint32_t spi_flash_normalize_size_mb(uint32_t size_mb) {
    if (size_mb < SPI_FLASH_UNIT_MB) {
        size_mb = SPI_FLASH_UNIT_MB;
    }
    if (size_mb % SPI_FLASH_UNIT_MB != 0u) {
        size_mb = ((size_mb + SPI_FLASH_UNIT_MB - 1u) / SPI_FLASH_UNIT_MB) *
                  SPI_FLASH_UNIT_MB;
    }
    if (size_mb > SPI_FLASH_MAX_CHIP_MB) {
        size_mb = SPI_FLASH_MAX_CHIP_MB;
    }
    return size_mb;
}

static void spi_flash_ensure_default_dir(void) {
#if !defined(_WIN32)
    if (!spi_flash_dir_created++) {
        mkdir(SPI_FLASH_DEFAULT_DIR, 0755);
    }
#endif
}

static void spi_flash_resolve_path(unsigned chip) {
    if (chip >= SPI_FLASH_CHIP_COUNT) {
        return;
    }
    spi_flash_chip_t *c = &spi_flash_chips[chip];
    if (!c->enabled || c->path != NULL) {
        return;
    }
    if (!c->use_default_path) {
        return;
    }
    spi_flash_ensure_default_dir();
    c->path = strdup(chip == 0u ? SPI_FLASH1_DEFAULT : SPI_FLASH2_DEFAULT);
}

static int spi_flash_ensure_file_size(spi_flash_chip_t *c) {
    if (c->fp == NULL) {
        return 0;
    }
    size_t sz = (size_t)c->size_mb * 1024u * 1024u;
#if !defined(_WIN32)
    int fd = fileno(c->fp);
    if (fd < 0) {
        return 0;
    }
    struct stat st;
    if (fstat(fd, &st) != 0) {
        return 0;
    }
    if ((size_t)st.st_size < sz && ftruncate(fd, (off_t)sz) != 0) {
        return 0;
    }
#else
    if (_chsize(_fileno(c->fp), (__int64)sz) != 0) {
        return 0;
    }
#endif
    return 1;
}

#if !defined(_WIN32)
static int spi_flash_map_chip(unsigned chip) {
    if (chip >= SPI_FLASH_CHIP_COUNT) {
        return 0;
    }
    spi_flash_chip_t *c = &spi_flash_chips[chip];
    if (c->map != NULL) {
        return 1;
    }
    if (c->fp == NULL) {
        return 0;
    }
    if (!spi_flash_ensure_file_size(c)) {
        return 0;
    }
    int fd = fileno(c->fp);
    if (fd < 0) {
        return 0;
    }
    size_t sz = (size_t)c->size_mb * 1024u * 1024u;
    void *p = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        return 0;
    }
    c->map = (uint8_t *)p;
    c->map_bytes = sz;
    static int map_logged;
    if (!map_logged++) {
        fprintf(stderr, "[SPI] flash mmap enabled for fast backing I/O\n");
    }
    return 1;
}
#endif

static FILE *spi_flash_open_chip(unsigned chip) {
    if (chip >= SPI_FLASH_CHIP_COUNT) {
        return NULL;
    }
    spi_flash_chip_t *c = &spi_flash_chips[chip];
    if (!c->enabled) {
        return NULL;
    }
    spi_flash_resolve_path(chip);
    if (c->path == NULL || c->path[0] == '\0') {
        return NULL;
    }
    if (c->fp != NULL) {
        return c->fp;
    }
    FILE *fp = fopen(c->path, "r+b");
    if (fp == NULL) {
        fp = fopen(c->path, "w+b");
    }
    if (fp == NULL) {
        fprintf(stderr, "[SPI] flash %u: cannot open %s: %s\n",
                chip + 1u, c->path, strerror(errno));
        return NULL;
    }
    c->fp = fp;
    (void)spi_flash_ensure_file_size(c);
#if !defined(_WIN32)
    (void)spi_flash_map_chip(chip);
#endif
    if (!spi_flash_logged++) {
        fprintf(stderr, "[SPI] SPI flash backing enabled\n");
    }
    fprintf(stderr, "[SPI]   spi-flash%u: %s (%uMB)\n",
            chip + 1u, c->path, c->size_mb);
    return fp;
}

void spi_flash_configure(unsigned chip, const char *path) {
    if (chip >= SPI_FLASH_CHIP_COUNT) {
        return;
    }
    spi_flash_chip_t *c = &spi_flash_chips[chip];
    c->enabled = 1;
    if (c->path != NULL) {
        free(c->path);
        c->path = NULL;
    }
    if (path != NULL && path[0] != '\0') {
        c->path = strdup(path);
        c->use_default_path = 0;
    } else {
        c->use_default_path = 1;
    }
}

void spi_flash_set_size(unsigned chip, uint32_t size_mb) {
    if (chip >= SPI_FLASH_CHIP_COUNT || size_mb == 0u) {
        return;
    }
    spi_flash_chips[chip].size_mb = spi_flash_normalize_size_mb(size_mb);
}

static int spi_flash_bounds_ok(unsigned chip, uint64_t offset, size_t len) {
    if (chip >= SPI_FLASH_CHIP_COUNT || len == 0u) {
        return 0;
    }
    if (!spi_flash_chips[chip].enabled) {
        return 0;
    }
    uint64_t chip_bytes = spi_flash_chip_bytes(&spi_flash_chips[chip]);
    if (offset >= chip_bytes) {
        return 0;
    }
    if (offset + (uint64_t)len > chip_bytes) {
        return 0;
    }
    return 1;
}

ssize_t spi_flash_read(unsigned chip, uint64_t offset, void *buf, size_t len) {
    if (buf == NULL) {
        return -1;
    }
    if (len == 0u) {
        return 0;
    }
    if (!spi_flash_bounds_ok(chip, offset, len)) {
        return -1;
    }
    if (spi_flash_open_chip(chip) == NULL) {
        return -1;
    }

    spi_flash_chip_t *c = &spi_flash_chips[chip];
    spi_flash_seq_t *seq = &spi_flash_seq[chip];

#if !defined(_WIN32)
    if (c->map != NULL && offset + len <= c->map_bytes) {
        memcpy(buf, c->map + offset, len);
        seq->valid = 1;
        seq->next_off = offset + len;
        return (ssize_t)len;
    }
#endif

    if (c->fp != NULL) {
        if (!(seq->valid && seq->next_off == offset)) {
            if (fseeko(c->fp, (off_t)offset, SEEK_SET) != 0) {
                return -1;
            }
        }
        size_t n = fread(buf, 1, len, c->fp);
        if (n < len) {
            memset((uint8_t *)buf + n, 0, len - n);
        }
        seq->valid = 1;
        seq->next_off = offset + len;
        return (ssize_t)len;
    }

    return -1;
}

ssize_t spi_flash_write(unsigned chip, uint64_t offset, const void *buf, size_t len) {
    if (buf == NULL) {
        return -1;
    }
    if (len == 0u) {
        return 0;
    }
    if (!spi_flash_bounds_ok(chip, offset, len)) {
        return -1;
    }
    if (spi_flash_open_chip(chip) == NULL) {
        return -1;
    }

    spi_flash_chip_t *c = &spi_flash_chips[chip];
    spi_flash_seq_t *seq = &spi_flash_seq[chip];

#if !defined(_WIN32)
    if (c->map != NULL && offset + len <= c->map_bytes) {
        memcpy(c->map + offset, buf, len);
        seq->valid = 1;
        seq->next_off = offset + len;
        if (++spi_flash_msync_pending[chip] >= 4096u) {
            size_t sync_len = 4096u * 512u;
            uint64_t sync_off = offset + len;
            if (sync_off > sync_len) {
                sync_off -= sync_len;
            } else {
                sync_off = 0;
            }
            if (sync_off + sync_len > c->map_bytes) {
                sync_len = c->map_bytes - (size_t)sync_off;
            }
            if (sync_len > 0u) {
                (void)msync(c->map + sync_off, sync_len, MS_ASYNC);
            }
            spi_flash_msync_pending[chip] = 0u;
        }
        return (ssize_t)len;
    }
#endif

    if (c->fp != NULL) {
        if (!(seq->valid && seq->next_off == offset)) {
            if (fseeko(c->fp, (off_t)offset, SEEK_SET) != 0) {
                fprintf(stderr,
                        "[SPI] flash write seek failed chip=%u off=%llu\n",
                        chip, (unsigned long long)offset);
                return -1;
            }
        }
        if (fwrite(buf, 1, len, c->fp) != len) {
            fprintf(stderr, "[SPI] flash write short chip=%u off=%llu len=%zu\n",
                    chip, (unsigned long long)offset, len);
            return -1;
        }
        seq->valid = 1;
        seq->next_off = offset + len;
        if (++spi_flash_fflush_pending >= 256u) {
            fflush(c->fp);
            spi_flash_fflush_pending = 0u;
        }
        return (ssize_t)len;
    }

    return -1;
}

void spi_flash_close(void) {
    for (unsigned chip = 0; chip < SPI_FLASH_CHIP_COUNT; chip++) {
        spi_flash_chip_t *c = &spi_flash_chips[chip];
#if !defined(_WIN32)
        if (c->map != NULL) {
            (void)msync(c->map, c->map_bytes, MS_SYNC);
            munmap(c->map, c->map_bytes);
            c->map = NULL;
            c->map_bytes = 0;
        }
#endif
        if (c->fp != NULL) {
            fflush(c->fp);
            fclose(c->fp);
            c->fp = NULL;
        }
        free(c->path);
        c->path = NULL;
        c->enabled = 0;
        c->use_default_path = 0;
        c->size_mb = SPI_FLASH_DEFAULT_CHIP_MB;
        spi_flash_seq[chip].valid = 0;
        spi_flash_msync_pending[chip] = 0u;
    }
    spi_flash_fflush_pending = 0u;
    spi_flash_logged = 0;
    spi_flash_dir_created = 0;
}
