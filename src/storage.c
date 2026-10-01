/*
 * Flash Write-Through Persistence
 *
 * Keeps the flash file always in sync with emulator memory by writing
 * affected sectors immediately after each flash_range_erase/program.
 * This enables external tools to mount and inspect the filesystem
 * while the emulator is running.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "storage.h"
#include "emulator.h"

static char *persist_path = NULL;
static FILE *persist_fp = NULL;

void flash_persist_set_path(const char *path) {
    /* Close any handle still pointing at the previous path, otherwise the next
     * sync writes to the old file while persist_path names the new one. */
    if (persist_fp) {
        fclose(persist_fp);
        persist_fp = NULL;
    }
    if (persist_path) {
        free(persist_path);
        persist_path = NULL;
    }
    if (path) {
        persist_path = strdup(path);
    }
}

int flash_persist_open(void) {
    if (!persist_path) return 0;

    /* Already open (set_path closes the previous handle first). */
    if (persist_fp) return 0;

    /* Try r+b first (existing file). Only fall back to creating a new file
     * when it genuinely does not exist: "w+b" truncates, so any other failure
     * (EMFILE, EACCES, a stale NFS handle) used to silently destroy the user's
     * entire persisted flash image. */
    persist_fp = fopen(persist_path, "r+b");
    if (!persist_fp) {
        int saved_errno = errno;
        if (saved_errno != ENOENT) {
            fprintf(stderr, "[Storage] Cannot open existing flash file %s: %s\n",
                    persist_path, strerror(saved_errno));
            return -1;
        }
        persist_fp = fopen(persist_path, "w+b");
        if (!persist_fp) {
            fprintf(stderr, "[Storage] Failed to open flash file: %s\n", persist_path);
            return -1;
        }
        /* New file: write full flash image */
        fwrite(cpu.flash, 1, FLASH_SIZE, persist_fp);
        fflush(persist_fp);
        fprintf(stderr, "[Storage] Created flash file: %s\n", persist_path);
    } else {
        /* Size the image to exactly FLASH_SIZE. Without this a file left over
         * from a larger build keeps a stale tail that a host `mount -o loop`
         * would then expose as old filesystem data. */
        if (fseek(persist_fp, 0, SEEK_END) == 0) {
            long sz = ftell(persist_fp);
            if (sz > (long)FLASH_SIZE) {
                if (ftruncate(fileno(persist_fp), FLASH_SIZE) != 0) {
                    fprintf(stderr, "[Storage] Failed to size flash file: %s\n",
                            strerror(errno));
                }
            }
        }
        fseek(persist_fp, 0, SEEK_SET);
    }

    return 0;
}

void flash_persist_sync(uint32_t offset, uint32_t len) {
    if (!persist_fp) return;
    /* offset+len wraps if both are guest-controlled 32-bit values. */
    if (offset > FLASH_SIZE || len > FLASH_SIZE - offset) return;

    fseek(persist_fp, (long)offset, SEEK_SET);
    fwrite(&cpu.flash[offset], 1, len, persist_fp);
    fflush(persist_fp);
}

void flash_persist_save_all(void) {
    if (!persist_fp) {
        /* No open file — try to create one for final save */
        if (!persist_path) return;
        persist_fp = fopen(persist_path, "wb");
        if (!persist_fp) {
            fprintf(stderr, "[Storage] Failed to save flash: %s\n", persist_path);
            return;
        }
        fwrite(cpu.flash, 1, FLASH_SIZE, persist_fp);
        fclose(persist_fp);
        persist_fp = NULL;
        fprintf(stderr, "[Flash] Saved to %s\n", persist_path);
        return;
    }

    /* Rewrite entire file */
    fseek(persist_fp, 0, SEEK_SET);
    fwrite(cpu.flash, 1, FLASH_SIZE, persist_fp);
    fflush(persist_fp);
    /* fflush only reaches the page cache; without fsync a power loss can lose
     * the whole image, and this file is the only copy. */
    fsync(fileno(persist_fp));
    fprintf(stderr, "[Flash] Saved to %s\n", persist_path);
}

void flash_persist_close(void) {
    if (persist_fp) {
        fclose(persist_fp);
        persist_fp = NULL;
    }
    if (persist_path) {
        free(persist_path);
        persist_path = NULL;
    }
}
