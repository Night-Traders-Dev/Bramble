#ifndef FATFS_H
#define FATFS_H

#include <stdint.h>
#include <stddef.h>

/* ========================================================================
 * Minimal FAT16 Filesystem Driver
 *
 * Operates directly on a raw byte array (e.g., cpu.flash[] at a given
 * offset). Supports reading and writing files, directory listing, and
 * file creation. Designed for the FAT16 filesystems created by
 * CircuitPython and MicroPython on RP2040 flash.
 *
 * This is NOT a general-purpose FAT driver. It handles:
 *   - FAT16 with 512-byte sectors
 *   - 8.3 short filenames (no LFN support)
 *   - Root directory (fixed size)
 *   - Single-level directories (no subdirectory traversal)
 *   - Read and write file data
 * ======================================================================== */

#define FAT16_SECTOR_SIZE   512
#define FAT16_MAX_NAME      11      /* 8.3 format: 8 name + 3 ext */
#define FAT16_MAX_FILES     128     /* Max files in directory listing */
#define FAT16_ATTR_READONLY 0x01
#define FAT16_ATTR_HIDDEN   0x02
#define FAT16_ATTR_SYSTEM   0x04
#define FAT16_ATTR_VOLUME   0x08
#define FAT16_ATTR_DIR      0x10
#define FAT16_ATTR_ARCHIVE  0x20
#define FAT16_ATTR_LFN      0x0F

/* Directory entry (32 bytes, matching on-disk format) */
typedef struct __attribute__((packed)) {
    char     name[11];      /* 8.3 filename (space-padded) */
    uint8_t  attr;          /* File attributes */
    uint8_t  reserved;
    uint8_t  ctime_tenths;
    uint16_t ctime;
    uint16_t cdate;
    uint16_t adate;
    uint16_t cluster_hi;    /* Always 0 for FAT16 */
    uint16_t mtime;
    uint16_t mdate;
    uint16_t cluster_lo;    /* Starting cluster */
    uint32_t file_size;
} fat16_dirent_t;

/* Filesystem context */
typedef struct {
    uint8_t *media;         /* Pointer to raw media bytes */
    size_t media_size;      /* Total media size */

    /* BPB (BIOS Parameter Block) parsed values */
    uint16_t bytes_per_sector;
    uint8_t  sectors_per_cluster;
    uint16_t reserved_sectors;
    uint8_t  num_fats;
    uint16_t root_entry_count;
    uint16_t total_sectors;
    uint16_t sectors_per_fat;

    /* Computed offsets (byte offsets into media) */
    uint32_t fat_offset;        /* Start of first FAT */
    uint32_t root_dir_offset;   /* Start of root directory */
    uint32_t data_offset;       /* Start of data area (cluster 2) */
    uint32_t root_dir_sectors;  /* Sectors used by root directory */
    uint32_t cluster_size;      /* Bytes per cluster */
    uint32_t total_clusters;    /* Total data clusters */
    int is_fat12;               /* 1 if FAT12, 0 if FAT16 */
} fat16_fs_t;

/* File info for directory listing */
typedef struct {
    char name[13];          /* "FILENAME.EXT" null-terminated */
    uint8_t attr;
    uint32_t size;
    uint16_t cluster;
} fat16_fileinfo_t;

/* Mount a FAT16 filesystem from raw media.
 * Returns 0 on success, -1 if not a valid FAT16 volume. */
int fat16_mount(fat16_fs_t *fs, uint8_t *media, size_t media_size);

/* List files in root directory.
 * Returns number of files found, up to max_files. */
int fat16_list_root(fat16_fs_t *fs, fat16_fileinfo_t *files, int max_files);

/* Read a file by name. Returns bytes read, or -1 if not found.
 * buf must be large enough for the file (check fileinfo.size first). */
int fat16_read_file(fat16_fs_t *fs, const char *name, uint8_t *buf, size_t buf_size);

/* Write/create a file. Returns 0 on success, -1 on error.
 * Overwrites existing file or creates new one. */
int fat16_write_file(fat16_fs_t *fs, const char *name, const uint8_t *data, size_t size);

/* Delete a file by name. Returns 0 on success, -1 if not found. */
int fat16_delete_file(fat16_fs_t *fs, const char *name);

/* Get file info by name. Returns 0 on success, -1 if not found. */
int fat16_stat(fat16_fs_t *fs, const char *name, fat16_fileinfo_t *info);

/* Get free space in bytes */
uint32_t fat16_free_space(fat16_fs_t *fs);

#endif /* FATFS_H */
