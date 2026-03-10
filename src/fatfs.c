/*
 * Minimal FAT16 Filesystem Driver
 *
 * Operates on raw byte arrays. Parses the BPB, reads/writes the FAT,
 * and manipulates root directory entries and file data clusters.
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "fatfs.h"

/* ========================================================================
 * Helpers
 * ======================================================================== */

static uint16_t read16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write16(uint8_t *p, uint16_t val) {
    p[0] = val & 0xFF;
    p[1] = (val >> 8) & 0xFF;
}

/* Convert "FILENAME.EXT" to FAT 8.3 format "FILENAMEEXT" (space-padded) */
static void name_to_fat83(const char *name, char *fat_name) {
    memset(fat_name, ' ', 11);

    /* Find the dot */
    const char *dot = strrchr(name, '.');
    int name_len = dot ? (int)(dot - name) : (int)strlen(name);
    if (name_len > 8) name_len = 8;

    for (int i = 0; i < name_len; i++) {
        fat_name[i] = toupper((unsigned char)name[i]);
    }

    if (dot) {
        const char *ext = dot + 1;
        int ext_len = (int)strlen(ext);
        if (ext_len > 3) ext_len = 3;
        for (int i = 0; i < ext_len; i++) {
            fat_name[8 + i] = toupper((unsigned char)ext[i]);
        }
    }
}

/* Convert FAT 8.3 "FILENAMEEXT" to "FILENAME.EXT" */
static void fat83_to_name(const char *fat_name, char *name) {
    int pos = 0;

    /* Copy name part (trim trailing spaces) */
    int name_end = 8;
    while (name_end > 0 && fat_name[name_end - 1] == ' ') name_end--;
    for (int i = 0; i < name_end; i++) {
        name[pos++] = fat_name[i];
    }

    /* Copy extension (trim trailing spaces) */
    int ext_end = 3;
    while (ext_end > 0 && fat_name[8 + ext_end - 1] == ' ') ext_end--;
    if (ext_end > 0) {
        name[pos++] = '.';
        for (int i = 0; i < ext_end; i++) {
            name[pos++] = fat_name[8 + i];
        }
    }

    name[pos] = '\0';
}

/* Get cluster offset in media */
static uint32_t cluster_offset(fat16_fs_t *fs, uint16_t cluster) {
    return fs->data_offset + (uint32_t)(cluster - 2) * fs->cluster_size;
}

/* Read FAT entry for a cluster */
static uint16_t fat_read(fat16_fs_t *fs, uint16_t cluster) {
    uint32_t off = fs->fat_offset + (uint32_t)cluster * 2;
    if (off + 1 >= fs->media_size) return 0xFFFF;
    return read16(&fs->media[off]);
}

/* Write FAT entry for a cluster (updates both FATs) */
static void fat_write(fat16_fs_t *fs, uint16_t cluster, uint16_t value) {
    uint32_t off = fs->fat_offset + (uint32_t)cluster * 2;
    if (off + 1 >= fs->media_size) return;
    write16(&fs->media[off], value);

    /* Mirror to second FAT */
    if (fs->num_fats > 1) {
        uint32_t off2 = off + (uint32_t)fs->sectors_per_fat * fs->bytes_per_sector;
        if (off2 + 1 < fs->media_size) {
            write16(&fs->media[off2], value);
        }
    }
}

/* Allocate a free cluster. Returns cluster number or 0 on failure. */
static uint16_t fat_alloc(fat16_fs_t *fs) {
    for (uint16_t c = 2; c < fs->total_clusters + 2; c++) {
        if (fat_read(fs, c) == 0x0000) {
            fat_write(fs, c, 0xFFFF); /* Mark as end-of-chain */
            return c;
        }
    }
    return 0;
}

/* Free a cluster chain starting at given cluster */
static void fat_free_chain(fat16_fs_t *fs, uint16_t cluster) {
    while (cluster >= 2 && cluster < 0xFFF8) {
        uint16_t next = fat_read(fs, cluster);
        fat_write(fs, cluster, 0x0000);
        cluster = next;
    }
}

/* Find a root directory entry by 8.3 name. Returns pointer or NULL. */
static fat16_dirent_t *find_dirent(fat16_fs_t *fs, const char *fat_name) {
    uint8_t *root = &fs->media[fs->root_dir_offset];
    for (int i = 0; i < fs->root_entry_count; i++) {
        fat16_dirent_t *de = (fat16_dirent_t *)(root + i * 32);
        if ((uint8_t)de->name[0] == 0x00) break;  /* End of entries */
        if ((uint8_t)de->name[0] == 0xE5) continue; /* Deleted */
        if (de->attr == FAT16_ATTR_LFN) continue;   /* LFN entry */
        if (memcmp(de->name, fat_name, 11) == 0) {
            return de;
        }
    }
    return NULL;
}

/* Find a free root directory slot. Returns pointer or NULL. */
static fat16_dirent_t *find_free_dirent(fat16_fs_t *fs) {
    uint8_t *root = &fs->media[fs->root_dir_offset];
    for (int i = 0; i < fs->root_entry_count; i++) {
        fat16_dirent_t *de = (fat16_dirent_t *)(root + i * 32);
        if ((uint8_t)de->name[0] == 0x00 || (uint8_t)de->name[0] == 0xE5) {
            return de;
        }
    }
    return NULL;
}

/* ========================================================================
 * Public API
 * ======================================================================== */

int fat16_mount(fat16_fs_t *fs, uint8_t *media, size_t media_size) {
    memset(fs, 0, sizeof(*fs));
    fs->media = media;
    fs->media_size = media_size;

    if (media_size < 512) return -1;

    /* Check boot signature */
    if (media[510] != 0x55 || media[511] != 0xAA) return -1;

    /* Parse BPB */
    fs->bytes_per_sector = read16(&media[11]);
    fs->sectors_per_cluster = media[13];
    fs->reserved_sectors = read16(&media[14]);
    fs->num_fats = media[16];
    fs->root_entry_count = read16(&media[17]);
    fs->total_sectors = read16(&media[19]);
    if (fs->total_sectors == 0) {
        fs->total_sectors = (uint16_t)read32(&media[32]); /* Large sector count */
    }
    fs->sectors_per_fat = read16(&media[22]);

    /* Validate */
    if (fs->bytes_per_sector != 512) return -1;
    if (fs->sectors_per_cluster == 0) return -1;
    if (fs->num_fats == 0) return -1;
    if (fs->sectors_per_fat == 0) return -1;

    /* Compute offsets */
    fs->fat_offset = (uint32_t)fs->reserved_sectors * fs->bytes_per_sector;
    fs->root_dir_sectors = ((uint32_t)fs->root_entry_count * 32 + fs->bytes_per_sector - 1) /
                           fs->bytes_per_sector;
    fs->root_dir_offset = fs->fat_offset +
                          (uint32_t)fs->num_fats * fs->sectors_per_fat * fs->bytes_per_sector;
    fs->data_offset = fs->root_dir_offset + fs->root_dir_sectors * fs->bytes_per_sector;
    fs->cluster_size = (uint32_t)fs->sectors_per_cluster * fs->bytes_per_sector;

    uint32_t data_sectors = fs->total_sectors - (fs->reserved_sectors +
                            fs->num_fats * fs->sectors_per_fat + fs->root_dir_sectors);
    fs->total_clusters = data_sectors / fs->sectors_per_cluster;

    /* FAT16 has 4085-65524 clusters */
    if (fs->total_clusters < 4085 || fs->total_clusters > 65524) {
        /* Could be FAT12 — allow smaller filesystems too */
        if (fs->total_clusters == 0) return -1;
    }

    return 0;
}

int fat16_list_root(fat16_fs_t *fs, fat16_fileinfo_t *files, int max_files) {
    int count = 0;
    uint8_t *root = &fs->media[fs->root_dir_offset];

    for (int i = 0; i < fs->root_entry_count && count < max_files; i++) {
        fat16_dirent_t *de = (fat16_dirent_t *)(root + i * 32);
        if ((uint8_t)de->name[0] == 0x00) break;
        if ((uint8_t)de->name[0] == 0xE5) continue;
        if (de->attr == FAT16_ATTR_LFN) continue;
        if (de->attr & FAT16_ATTR_VOLUME) continue;

        fat83_to_name(de->name, files[count].name);
        files[count].attr = de->attr;
        files[count].size = de->file_size;
        files[count].cluster = de->cluster_lo;
        count++;
    }

    return count;
}

int fat16_read_file(fat16_fs_t *fs, const char *name, uint8_t *buf, size_t buf_size) {
    char fat_name[11];
    name_to_fat83(name, fat_name);

    fat16_dirent_t *de = find_dirent(fs, fat_name);
    if (!de) return -1;

    uint32_t file_size = de->file_size;
    if (file_size > buf_size) file_size = (uint32_t)buf_size;

    uint16_t cluster = de->cluster_lo;
    uint32_t remaining = file_size;
    uint32_t pos = 0;

    while (remaining > 0 && cluster >= 2 && cluster < 0xFFF8) {
        uint32_t off = cluster_offset(fs, cluster);
        uint32_t chunk = fs->cluster_size;
        if (chunk > remaining) chunk = remaining;

        if (off + chunk > fs->media_size) break;
        memcpy(&buf[pos], &fs->media[off], chunk);

        pos += chunk;
        remaining -= chunk;
        cluster = fat_read(fs, cluster);
    }

    return (int)pos;
}

int fat16_write_file(fat16_fs_t *fs, const char *name, const uint8_t *data, size_t size) {
    char fat_name[11];
    name_to_fat83(name, fat_name);

    /* Check if file exists — if so, delete it first */
    fat16_dirent_t *existing = find_dirent(fs, fat_name);
    if (existing) {
        fat_free_chain(fs, existing->cluster_lo);
        existing->name[0] = (char)0xE5;  /* Mark deleted */
    }

    /* Find free directory entry */
    fat16_dirent_t *de = find_free_dirent(fs);
    if (!de) return -1;

    /* Allocate clusters */
    uint32_t clusters_needed = (size + fs->cluster_size - 1) / fs->cluster_size;
    if (clusters_needed == 0) clusters_needed = 1; /* At least one cluster for empty files */

    uint16_t first_cluster = 0;
    uint16_t prev_cluster = 0;

    for (uint32_t i = 0; i < clusters_needed; i++) {
        uint16_t c = fat_alloc(fs);
        if (c == 0) {
            /* Out of space — free what we allocated */
            if (first_cluster) fat_free_chain(fs, first_cluster);
            return -1;
        }
        if (i == 0) {
            first_cluster = c;
        } else {
            fat_write(fs, prev_cluster, c);
        }
        prev_cluster = c;
    }

    /* Write data to clusters */
    uint16_t cluster = first_cluster;
    uint32_t remaining = (uint32_t)size;
    uint32_t pos = 0;

    while (remaining > 0 && cluster >= 2 && cluster < 0xFFF8) {
        uint32_t off = cluster_offset(fs, cluster);
        uint32_t chunk = fs->cluster_size;
        if (chunk > remaining) chunk = remaining;

        if (off + chunk <= fs->media_size) {
            memcpy(&fs->media[off], &data[pos], chunk);
            /* Zero-fill remainder of last cluster */
            if (chunk < fs->cluster_size) {
                memset(&fs->media[off + chunk], 0, fs->cluster_size - chunk);
            }
        }

        pos += chunk;
        remaining -= chunk;
        cluster = fat_read(fs, cluster);
    }

    /* Create directory entry */
    memcpy(de->name, fat_name, 11);
    de->attr = FAT16_ATTR_ARCHIVE;
    de->reserved = 0;
    de->ctime_tenths = 0;
    de->ctime = 0;
    de->cdate = 0;
    de->adate = 0;
    de->cluster_hi = 0;
    de->mtime = 0;
    de->mdate = 0;
    de->cluster_lo = first_cluster;
    de->file_size = (uint32_t)size;

    return 0;
}

int fat16_delete_file(fat16_fs_t *fs, const char *name) {
    char fat_name[11];
    name_to_fat83(name, fat_name);

    fat16_dirent_t *de = find_dirent(fs, fat_name);
    if (!de) return -1;

    fat_free_chain(fs, de->cluster_lo);
    de->name[0] = (char)0xE5;

    return 0;
}

int fat16_stat(fat16_fs_t *fs, const char *name, fat16_fileinfo_t *info) {
    char fat_name[11];
    name_to_fat83(name, fat_name);

    fat16_dirent_t *de = find_dirent(fs, fat_name);
    if (!de) return -1;

    fat83_to_name(de->name, info->name);
    info->attr = de->attr;
    info->size = de->file_size;
    info->cluster = de->cluster_lo;

    return 0;
}

uint32_t fat16_free_space(fat16_fs_t *fs) {
    uint32_t free_clusters = 0;
    for (uint16_t c = 2; c < fs->total_clusters + 2; c++) {
        if (fat_read(fs, c) == 0x0000) {
            free_clusters++;
        }
    }
    return free_clusters * fs->cluster_size;
}
