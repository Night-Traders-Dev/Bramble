/*
 * FUSE Filesystem Mount for Flash Storage
 *
 * Uses libfuse3 to expose the FAT16 filesystem from emulated flash
 * as a host-accessible directory. Runs in a background thread so the
 * emulator continues execution while files are browsable/editable.
 *
 * All file operations go through the FAT16 module (fatfs.c) which
 * operates directly on the flash memory array — changes are immediate.
 *
 * Thread safety: a mutex serializes FUSE operations against emulator
 * flash writes (ROM flash_range_erase/program). Both sides lock
 * fuse_flash_mutex before touching the flash array.
 */

#ifdef ENABLE_FUSE

#define FUSE_USE_VERSION 31
#include <fuse3/fuse.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>
#include "fuse_mount.h"
#include "fatfs.h"
#include "storage.h"

static fat16_fs_t fuse_fs;
static struct fuse *fuse_instance = NULL;
static pthread_t fuse_thread;
static int fuse_running = 0;
static char fuse_mountpoint[256];
static uint32_t fuse_fs_offset = 0;  /* Offset of filesystem region in flash */
static uint32_t fuse_fs_size = 0;

/* Mutex protecting flash array access between FUSE thread and emulator */
pthread_mutex_t fuse_flash_mutex = PTHREAD_MUTEX_INITIALIZER;

void fuse_set_flash_offset(uint32_t offset) {
    fuse_fs_offset = offset;
}

/* Sync the filesystem region to disk after modifications */
static void fuse_persist_sync(void) {
    if (fuse_fs_size > 0) {
        flash_persist_sync(fuse_fs_offset, fuse_fs_size);
    }
}

/* ========================================================================
 * FUSE operations
 * ======================================================================== */

static int bramble_getattr(const char *path, struct stat *stbuf,
                           struct fuse_file_info *fi) {
    (void)fi;
    memset(stbuf, 0, sizeof(struct stat));

    if (strcmp(path, "/") == 0) {
        stbuf->st_mode = S_IFDIR | 0755;
        stbuf->st_nlink = 2;
        return 0;
    }

    /* Skip leading slash */
    const char *name = path + 1;
    fat16_fileinfo_t info;

    pthread_mutex_lock(&fuse_flash_mutex);
    int rc = fat16_stat(&fuse_fs, name, &info);
    pthread_mutex_unlock(&fuse_flash_mutex);

    if (rc < 0) return -ENOENT;

    if (info.attr & FAT16_ATTR_DIR) {
        stbuf->st_mode = S_IFDIR | 0755;
        stbuf->st_nlink = 2;
    } else {
        stbuf->st_mode = S_IFREG | 0644;
        stbuf->st_nlink = 1;
        stbuf->st_size = info.size;
    }

    return 0;
}

static int bramble_readdir(const char *path, void *buf, fuse_fill_dir_t filler,
                           off_t offset, struct fuse_file_info *fi,
                           enum fuse_readdir_flags flags) {
    (void)offset;
    (void)fi;
    (void)flags;

    if (strcmp(path, "/") != 0) return -ENOENT;

    filler(buf, ".", NULL, 0, 0);
    filler(buf, "..", NULL, 0, 0);

    fat16_fileinfo_t files[FAT16_MAX_FILES];

    pthread_mutex_lock(&fuse_flash_mutex);
    int count = fat16_list_root(&fuse_fs, files, FAT16_MAX_FILES);
    pthread_mutex_unlock(&fuse_flash_mutex);

    for (int i = 0; i < count; i++) {
        filler(buf, files[i].name, NULL, 0, 0);
    }

    return 0;
}

static int bramble_open(const char *path, struct fuse_file_info *fi) {
    const char *name = path + 1;
    fat16_fileinfo_t info;

    pthread_mutex_lock(&fuse_flash_mutex);
    int rc = fat16_stat(&fuse_fs, name, &info);
    pthread_mutex_unlock(&fuse_flash_mutex);

    (void)fi;
    return rc < 0 ? -ENOENT : 0;
}

static int bramble_read(const char *path, char *buf, size_t size, off_t offset,
                        struct fuse_file_info *fi) {
    (void)fi;
    const char *name = path + 1;

    pthread_mutex_lock(&fuse_flash_mutex);

    fat16_fileinfo_t info;
    if (fat16_stat(&fuse_fs, name, &info) < 0) {
        pthread_mutex_unlock(&fuse_flash_mutex);
        return -ENOENT;
    }

    if ((uint32_t)offset >= info.size) {
        pthread_mutex_unlock(&fuse_flash_mutex);
        return 0;
    }

    /* Read entire file into temp buffer, then copy requested range */
    uint8_t *tmp = malloc(info.size);
    if (!tmp) {
        pthread_mutex_unlock(&fuse_flash_mutex);
        return -ENOMEM;
    }

    int n = fat16_read_file(&fuse_fs, name, tmp, info.size);
    pthread_mutex_unlock(&fuse_flash_mutex);

    if (n < 0) {
        free(tmp);
        return -EIO;
    }

    uint32_t avail = info.size - (uint32_t)offset;
    if (size > avail) size = avail;
    memcpy(buf, tmp + offset, size);

    free(tmp);
    return (int)size;
}

static int bramble_write(const char *path, const char *buf, size_t size,
                         off_t offset, struct fuse_file_info *fi) {
    (void)fi;
    const char *name = path + 1;

    pthread_mutex_lock(&fuse_flash_mutex);

    fat16_fileinfo_t info;
    uint32_t old_size = 0;
    uint8_t *tmp = NULL;

    if (fat16_stat(&fuse_fs, name, &info) == 0) {
        old_size = info.size;
    }

    /* Read existing file content */
    uint32_t new_size = (uint32_t)(offset + size);
    if (new_size < old_size) new_size = old_size;

    tmp = calloc(1, new_size);
    if (!tmp) {
        pthread_mutex_unlock(&fuse_flash_mutex);
        return -ENOMEM;
    }

    if (old_size > 0) {
        fat16_read_file(&fuse_fs, name, tmp, old_size);
    }

    /* Apply write */
    memcpy(tmp + offset, buf, size);

    /* Write back */
    int rc = fat16_write_file(&fuse_fs, name, tmp, new_size);
    if (rc >= 0) {
        fuse_persist_sync();
    }
    pthread_mutex_unlock(&fuse_flash_mutex);

    free(tmp);
    return rc < 0 ? -EIO : (int)size;
}

static int bramble_create(const char *path, mode_t mode, struct fuse_file_info *fi) {
    (void)mode;
    (void)fi;
    const char *name = path + 1;

    pthread_mutex_lock(&fuse_flash_mutex);
    uint8_t empty = 0;
    int rc = fat16_write_file(&fuse_fs, name, &empty, 0);
    if (rc >= 0) {
        fuse_persist_sync();
    }
    pthread_mutex_unlock(&fuse_flash_mutex);

    return rc < 0 ? -ENOSPC : 0;
}

static int bramble_unlink(const char *path) {
    const char *name = path + 1;

    pthread_mutex_lock(&fuse_flash_mutex);
    int rc = fat16_delete_file(&fuse_fs, name);
    if (rc >= 0) {
        fuse_persist_sync();
    }
    pthread_mutex_unlock(&fuse_flash_mutex);

    return rc < 0 ? -ENOENT : 0;
}

static int bramble_truncate(const char *path, off_t size,
                            struct fuse_file_info *fi) {
    (void)fi;
    const char *name = path + 1;

    pthread_mutex_lock(&fuse_flash_mutex);

    if (size == 0) {
        uint8_t empty = 0;
        int rc = fat16_write_file(&fuse_fs, name, &empty, 0);
        if (rc >= 0) {
            fuse_persist_sync();
        }
        pthread_mutex_unlock(&fuse_flash_mutex);
        return rc < 0 ? -EIO : 0;
    }

    fat16_fileinfo_t info;
    if (fat16_stat(&fuse_fs, name, &info) < 0) {
        pthread_mutex_unlock(&fuse_flash_mutex);
        return -ENOENT;
    }

    uint8_t *tmp = calloc(1, (size_t)size);
    if (!tmp) {
        pthread_mutex_unlock(&fuse_flash_mutex);
        return -ENOMEM;
    }

    uint32_t copy_size = info.size < (uint32_t)size ? info.size : (uint32_t)size;
    if (copy_size > 0) {
        fat16_read_file(&fuse_fs, name, tmp, copy_size);
    }

    int rc = fat16_write_file(&fuse_fs, name, tmp, (size_t)size);
    if (rc >= 0) {
        fuse_persist_sync();
    }
    pthread_mutex_unlock(&fuse_flash_mutex);

    free(tmp);
    return rc < 0 ? -EIO : 0;
}

static const struct fuse_operations bramble_ops = {
    .getattr  = bramble_getattr,
    .readdir  = bramble_readdir,
    .open     = bramble_open,
    .read     = bramble_read,
    .write    = bramble_write,
    .create   = bramble_create,
    .unlink   = bramble_unlink,
    .truncate = bramble_truncate,
};

/* ========================================================================
 * Thread management
 * ======================================================================== */

static void *fuse_thread_fn(void *arg) {
    (void)arg;
    /* fuse_loop runs until unmount */
    if (fuse_instance) {
        fuse_loop(fuse_instance);
    }
    fuse_running = 0;
    return NULL;
}

int fuse_mount_start(uint8_t *flash_data, size_t flash_size, const char *mount_point) {
    /* Try to mount the FAT16 filesystem */
    pthread_mutex_lock(&fuse_flash_mutex);
    int rc = fat16_mount(&fuse_fs, flash_data, flash_size);
    pthread_mutex_unlock(&fuse_flash_mutex);

    if (rc < 0) {
        fprintf(stderr, "[FUSE] No valid FAT16 filesystem found in flash region\n");
        return -1;
    }

    /* Store filesystem offset for persistence sync */
    fuse_fs_size = (uint32_t)flash_size;

    strncpy(fuse_mountpoint, mount_point, sizeof(fuse_mountpoint) - 1);

    /* Create mount point directory if it doesn't exist */
    mkdir(mount_point, 0755);

    /* Set up FUSE args */
    const char *fuse_argv[] = { "bramble", "-f", mount_point };
    struct fuse_args args = FUSE_ARGS_INIT(3, (char **)fuse_argv);

    fuse_instance = fuse_new(&args, &bramble_ops, sizeof(bramble_ops), NULL);
    if (!fuse_instance) {
        fprintf(stderr, "[FUSE] Failed to create FUSE instance\n");
        fuse_opt_free_args(&args);
        return -1;
    }

    if (fuse_mount(fuse_instance, mount_point) < 0) {
        fprintf(stderr, "[FUSE] Failed to mount at %s\n", mount_point);
        fuse_destroy(fuse_instance);
        fuse_instance = NULL;
        fuse_opt_free_args(&args);
        return -1;
    }

    fuse_running = 1;
    if (pthread_create(&fuse_thread, NULL, fuse_thread_fn, NULL) != 0) {
        fprintf(stderr, "[FUSE] Failed to start FUSE thread\n");
        fuse_unmount(fuse_instance);
        fuse_destroy(fuse_instance);
        fuse_instance = NULL;
        fuse_running = 0;
        fuse_opt_free_args(&args);
        return -1;
    }

    fprintf(stderr, "[FUSE] Mounted flash filesystem at %s\n", mount_point);
    fuse_opt_free_args(&args);
    return 0;
}

void fuse_mount_stop(void) {
    if (!fuse_instance) return;

    fuse_unmount(fuse_instance);

    if (fuse_running) {
        pthread_join(fuse_thread, NULL);
    }

    fuse_destroy(fuse_instance);
    fuse_instance = NULL;
    fuse_running = 0;

    fprintf(stderr, "[FUSE] Unmounted %s\n", fuse_mountpoint);
}

int fuse_mount_active(void) {
    return fuse_running;
}

#else /* !ENABLE_FUSE */

/* Stub implementations when FUSE is not available */
#include <stdio.h>
#include <pthread.h>
#include "fuse_mount.h"

/* Mutex still defined so ROM flash intercept can reference it */
pthread_mutex_t fuse_flash_mutex = PTHREAD_MUTEX_INITIALIZER;

void fuse_set_flash_offset(uint32_t offset) { (void)offset; }

int fuse_mount_start(uint8_t *flash_data, size_t flash_size, const char *mount_point) {
    (void)flash_data;
    (void)flash_size;
    fprintf(stderr, "[FUSE] Not available (build with -DENABLE_FUSE=ON)\n");
    fprintf(stderr, "[FUSE] Alternative: mount flash file with:\n");
    fprintf(stderr, "       sudo mount -o loop,offset=1048576 <flash.bin> %s\n", mount_point);
    return -1;
}

void fuse_mount_stop(void) {}
int fuse_mount_active(void) { return 0; }

#endif /* ENABLE_FUSE */
