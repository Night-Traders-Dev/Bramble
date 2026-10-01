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

static void write32(uint8_t *p, uint32_t val) {
    p[0] = val & 0xFF;
    p[1] = (val >> 8) & 0xFF;
    p[2] = (val >> 16) & 0xFF;
    p[3] = (val >> 24) & 0xFF;
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

/* Number of root directory entries that are actually backed by the media.
 * root_entry_count is guest-controlled (up to 65535) and the root directory is
 * validated to fit in mount(), but this keeps every accessor independently
 * bounded rather than relying on that. */
static uint32_t fat16_root_entries(const fat16_fs_t *fs) {
    if (!fs->geometry_valid) return 0;
    uint64_t avail = (uint64_t)fs->media_size - fs->root_dir_offset;
    uint64_t n = avail / 32;
    if (n > fs->root_entry_count) n = fs->root_entry_count;
    return (uint32_t)n;
}

/* Byte offset of a cluster's first sector, or 0 if the cluster is out of range.
 * The multiplication is done in 64 bits: cluster_size can be up to 130560 and
 * the cluster number is read straight out of an on-disk directory entry, so the
 * 32-bit product could wrap and defeat the caller's bounds check. */
static uint32_t cluster_offset(fat16_fs_t *fs, uint16_t cluster) {
    if (cluster < 2) return 0;
    if ((uint32_t)(cluster - 2) >= fs->total_clusters) return 0;
    uint64_t off = (uint64_t)fs->data_offset +
                   (uint64_t)(cluster - 2) * (uint64_t)fs->cluster_size;
    if (off > (uint64_t)fs->media_size) return 0;
    return (uint32_t)off;
}

/* Read FAT entry for a cluster (supports FAT12 and FAT16) */
static uint16_t fat_read(fat16_fs_t *fs, uint16_t cluster) {
    if (fs->is_fat12) {
        /* FAT12: 1.5 bytes per entry */
        uint32_t off = fs->fat_offset + (uint32_t)cluster * 3 / 2;
        if (off + 1 >= fs->media_size) return 0xFFF;
        uint16_t val = read16(&fs->media[off]);
        if (cluster & 1) {
            return val >> 4;        /* Odd cluster: high 12 bits */
        } else {
            return val & 0x0FFF;    /* Even cluster: low 12 bits */
        }
    }
    uint32_t off = fs->fat_offset + (uint32_t)cluster * 2;
    if (off + 1 >= fs->media_size) return 0xFFFF;
    return read16(&fs->media[off]);
}

/* End-of-chain marker for current FAT type */
static uint16_t fat_eoc(fat16_fs_t *fs) {
    return fs->is_fat12 ? 0xFF8 : 0xFFF8;
}

/* Check if a FAT entry marks end of chain */
static int fat_is_eoc(fat16_fs_t *fs, uint16_t val) {
    return fs->is_fat12 ? (val >= 0xFF8) : (val >= 0xFFF8);
}

/* Write FAT entry for a cluster (updates both FATs, supports FAT12/16) */
static void fat_write(fat16_fs_t *fs, uint16_t cluster, uint16_t value) {
    if (fs->is_fat12) {
        uint32_t off = fs->fat_offset + (uint32_t)cluster * 3 / 2;
        if (off + 1 >= fs->media_size) return;
        uint16_t cur = read16(&fs->media[off]);
        if (cluster & 1) {
            cur = (cur & 0x000F) | (value << 4);
        } else {
            cur = (cur & 0xF000) | (value & 0x0FFF);
        }
        fs->media[off] = cur & 0xFF;
        fs->media[off + 1] = (cur >> 8) & 0xFF;
        /* Second FAT copy */
        if (fs->num_fats > 1) {
            uint32_t off2 = off + (uint32_t)fs->sectors_per_fat * fs->bytes_per_sector;
            if (off2 + 1 < fs->media_size) {
                fs->media[off2] = cur & 0xFF;
                fs->media[off2 + 1] = (cur >> 8) & 0xFF;
            }
        }
        return;
    }
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
            fat_write(fs, c, fat_eoc(fs)); /* Mark as end-of-chain */
            return c;
        }
    }
    return 0;
}

/* Free a cluster chain starting at given cluster */
static void fat_free_chain(fat16_fs_t *fs, uint16_t cluster) {
    while (cluster >= 2 && !fat_is_eoc(fs, cluster)) {
        uint16_t next = fat_read(fs, cluster);
        fat_write(fs, cluster, 0x0000);
        cluster = next;
    }
}

/* Find a root directory entry by 8.3 name. Returns pointer or NULL. */
static fat16_dirent_t *find_dirent(fat16_fs_t *fs, const char *fat_name) {
    uint8_t *root = &fs->media[fs->root_dir_offset];
    uint32_t max = fat16_root_entries(fs);
    for (uint32_t i = 0; i < max; i++) {
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
    uint32_t max = fat16_root_entries(fs);
    for (uint32_t i = 0; i < max; i++) {
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
    /* 16-bit field first; 0 means "use the 32-bit field at 0x20". Keeping this
     * 32-bit is what allows volumes above 32 MiB to mount at all. */
    fs->total_sectors = read16(&media[19]);
    if (fs->total_sectors == 0) {
        fs->total_sectors = read32(&media[32]);
    }
    fs->sectors_per_fat = read16(&media[22]);

    /* Validate the fields we must have before doing arithmetic with them. */
    if (fs->bytes_per_sector != 512) return -1;
    if (fs->sectors_per_cluster == 0) return -1;
    if (fs->num_fats == 0) return -1;
    if (fs->sectors_per_fat == 0) return -1;
    if (fs->total_sectors == 0) return -1;

    /* Compute offsets. Every product is done in 64 bits: the BPB is fully
     * guest-controlled, and the 32-bit sums wrap (num_fats*sectors_per_fat*
     * bytes_per_sector alone can reach ~4 GiB). */
    uint64_t bps    = fs->bytes_per_sector;
    uint64_t fat_sz = (uint64_t)fs->sectors_per_fat * bps;

    fs->fat_offset    = (uint32_t)((uint64_t)fs->reserved_sectors * bps);
    fs->root_dir_sectors = ((uint64_t)fs->root_entry_count * 32 + bps - 1) / bps;
    fs->root_dir_offset = fs->fat_offset + (uint32_t)((uint64_t)fs->num_fats * fat_sz);
    fs->data_offset    = fs->root_dir_offset + (uint32_t)(fs->root_dir_sectors * bps);
    fs->cluster_size   = (uint32_t)((uint64_t)fs->sectors_per_cluster * bps);

    uint64_t meta_sectors = (uint64_t)fs->reserved_sectors +
                            (uint64_t)fs->num_fats * fs->sectors_per_fat +
                            fs->root_dir_sectors;
    /* O11: the declared metadata must not exceed the declared volume. Without
     * this, a crafted reserved_sectors/sectors_per_fat drives root_dir_offset
     * megabytes past the media and every dirent access is an OOB read or
     * write. */
    if (meta_sectors > fs->total_sectors) return -1;
    uint64_t data_sectors = fs->total_sectors - meta_sectors;
    fs->total_clusters = (uint32_t)(data_sectors / fs->sectors_per_cluster);

    /* Determine FAT type by cluster count (Microsoft spec) */
    if (fs->total_clusters < 4085) {
        fs->is_fat12 = 1;  /* FAT12 */
    } else if (fs->total_clusters <= 65524) {
        fs->is_fat12 = 0;  /* FAT16 */
    } else {
        return -1;  /* FAT32 not supported */
    }
    if (fs->total_clusters == 0) return -1;

    /* O11/O13: the whole derived layout, including the FAT copies and the
     * entire data area, must lie inside the media. Each check is written as a
     * subtraction so it cannot itself overflow. */
    if ((uint64_t)fs->fat_offset > (uint64_t)media_size) return -1;
    if ((uint64_t)fs->num_fats * fat_sz > (uint64_t)media_size - fs->fat_offset) return -1;
    if ((uint64_t)fs->root_dir_offset > (uint64_t)media_size) return -1;
    if (fs->root_dir_sectors * bps > (uint64_t)media_size - fs->root_dir_offset) return -1;
    if ((uint64_t)fs->data_offset > (uint64_t)media_size) return -1;
    if ((uint64_t)fs->total_clusters * fs->cluster_size >
        (uint64_t)media_size - fs->data_offset) return -1;

    fs->geometry_valid = 1;
    return 0;
}

int fat16_list_root(fat16_fs_t *fs, fat16_fileinfo_t *files, int max_files) {
    int count = 0;
    if (!fs->geometry_valid) return 0;
    uint8_t *root = &fs->media[fs->root_dir_offset];
    uint32_t entries = fat16_root_entries(fs);

    for (uint32_t i = 0; i < entries && count < max_files; i++) {
        fat16_dirent_t *de = (fat16_dirent_t *)(root + i * 32);
        if ((uint8_t)de->name[0] == 0x00) break;
        if ((uint8_t)de->name[0] == 0xE5) continue;
        if (de->attr == FAT16_ATTR_LFN) continue;
        if (de->attr & FAT16_ATTR_VOLUME) continue;

        fat83_to_name(de->name, files[count].name);
        files[count].attr = de->attr;
        files[count].size = read32((const uint8_t *)&de->file_size);
        files[count].cluster = read16((const uint8_t *)&de->cluster_lo);
        count++;
    }

    return count;
}

int fat16_read_file(fat16_fs_t *fs, const char *name, uint8_t *buf, size_t buf_size) {
    char fat_name[11];
    name_to_fat83(name, fat_name);

    fat16_dirent_t *de = find_dirent(fs, fat_name);
    if (!de) return -1;

    uint32_t file_size = read32((const uint8_t *)&de->file_size);
    if (file_size > buf_size) file_size = (uint32_t)buf_size;
    if (de->cluster_lo == 0) return -1;   /* empty or unreachable chain */

    uint16_t cluster = de->cluster_lo;
    uint32_t remaining = file_size;
    uint32_t pos = 0;

    while (remaining > 0 && cluster >= 2 && !fat_is_eoc(fs, cluster)) {
        uint32_t off = cluster_offset(fs, cluster);
        uint32_t chunk = fs->cluster_size;
        if (chunk > remaining) chunk = remaining;

        if ((uint64_t)off + chunk > (uint64_t)fs->media_size) break;
        if ((uint64_t)pos + chunk > (uint64_t)buf_size) break;
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
    uint32_t clusters_needed = (uint32_t)((size + fs->cluster_size - 1) / fs->cluster_size);
    if (clusters_needed == 0) clusters_needed = 1; /* At least one cluster for empty files */
    if (clusters_needed > fs->total_clusters) return -1;  /* cannot fit in the volume */

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
    int short_write = 0;

    while (remaining > 0) {
        if (cluster < 2 || fat_is_eoc(fs, cluster)) { short_write = 1; break; }

        uint32_t off = cluster_offset(fs, cluster);
        uint32_t chunk = fs->cluster_size;
        if (chunk > remaining) chunk = remaining;

        /* cluster_offset() returns 0 for an out-of-range cluster, and the media
         * bound is checked in 64 bits. Previously a chain running off the end
         * of the media was skipped silently while the dirent still advertised
         * the full file_size, so callers saw success and the tail of the data
         * was simply gone. */
        if (off == 0 && fs->data_offset != 0) { short_write = 1; break; }
        if ((uint64_t)off + fs->cluster_size > (uint64_t)fs->media_size) { short_write = 1; break; }

        memcpy(&fs->media[off], &data[pos], chunk);
        /* Zero-fill remainder of last cluster */
        if (chunk < fs->cluster_size) {
            memset(&fs->media[off + chunk], 0, fs->cluster_size - chunk);
        }

        pos += chunk;
        remaining -= chunk;
        cluster = fat_read(fs, cluster);
    }

    if (short_write) {
        /* Roll back so no truncated file is left behind. */
        fat_free_chain(fs, first_cluster);
        return -1;
    }

    /* Create directory entry. Written via write16/write32 so the on-disk
     * little-endian layout is explicit and the packed struct is not written
     * through as a native-endian object. */
    memcpy(de->name, fat_name, 11);
    de->attr = FAT16_ATTR_ARCHIVE;
    de->reserved = 0;
    de->ctime_tenths = 0;
    write16((uint8_t *)&de->ctime, 0);
    write16((uint8_t *)&de->cdate, 0);
    write16((uint8_t *)&de->adate, 0);
    write16((uint8_t *)&de->cluster_hi, 0);
    write16((uint8_t *)&de->mtime, 0);
    write16((uint8_t *)&de->mdate, 0);
    write16((uint8_t *)&de->cluster_lo, first_cluster);
    write32((uint8_t *)&de->file_size, (uint32_t)size);

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
    info->size = read32((const uint8_t *)&de->file_size);
    info->cluster = read16((const uint8_t *)&de->cluster_lo);

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
