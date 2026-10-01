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
#include "storage.h"
#include "emulator.h"

static char *persist_path = NULL;
static FILE *persist_fp = NULL;

void flash_persist_set_path(const char *path) {
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

    /* Try r+b first (existing file), fall back to w+b (create new) */
    persist_fp = fopen(persist_path, "r+b");
    if (!persist_fp) {
        persist_fp = fopen(persist_path, "w+b");
        if (!persist_fp) {
            fprintf(stderr, "[Storage] Failed to open flash file: %s\n", persist_path);
            return -1;
        }
        /* New file: write full flash image */
        fwrite(cpu.flash, 1, FLASH_SIZE, persist_fp);
        fflush(persist_fp);
        fprintf(stderr, "[Storage] Created flash file: %s\n", persist_path);
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
