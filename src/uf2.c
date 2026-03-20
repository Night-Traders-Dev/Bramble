#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "emulator.h"

/* UF2 Block Structure (512 bytes) */
typedef struct {
    uint32_t magic_start0;
    uint32_t magic_start1;
    uint32_t flags;
    uint32_t target_addr;
    uint32_t payload_size;
    uint32_t block_no;
    uint32_t num_blocks;
    uint32_t file_size;    /* or familyID when flags & 0x2000 */
    uint8_t  data[476];
    uint32_t magic_end;
} __attribute__((packed)) uf2_block_t;

#define UF2_MAGIC_START0 0x0A324655  /* "UF2\n" */
#define UF2_MAGIC_START1 0x9E5D5157
#define UF2_MAGIC_END    0x0AB16F30  /* End marker */

/* Detected architecture from last firmware load (shared with elf.c) */
int detected_arch = FW_ARCH_UNKNOWN;

int loader_detected_arch(void) {
    return detected_arch;
}

static int uf2_block_flash_offset(const uf2_block_t *block, uint32_t *offset_out) {
    if (block->payload_size > sizeof(block->data)) {
        return 0;
    }
    if (block->target_addr < FLASH_BASE) {
        return 0;
    }

    uint32_t offset = block->target_addr - FLASH_BASE;
    if (offset > FLASH_SIZE) {
        return 0;
    }
    if (block->payload_size > FLASH_SIZE - offset) {
        return 0;
    }

    *offset_out = offset;
    return 1;
}

static const char *family_id_name(uint32_t id) {
    switch (id) {
    case UF2_FAMILY_RP2040:     return "RP2040 (Cortex-M0+)";
    case UF2_FAMILY_RP2350_ARM: return "RP2350 (Cortex-M33)";
    case UF2_FAMILY_RP2350_RV:  return "RP2350 (Hazard3 RISC-V)";
    default:                    return "Unknown";
    }
}

int load_uf2(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror("[LOADER] Failed to open UF2");
        return 0;
    }

    uf2_block_t block;
    int blocks_loaded = 0;
    int blocks_total = 0;
    uint32_t family_id = 0;
    int family_detected = 0;

    fprintf(stderr, "[LOADER] Starting UF2 load from: %s\n", filename);

    while (fread(&block, 1, 512, f) == 512) {
        blocks_total++;

        /* DEBUG: Print block info */
        fprintf(stderr, "[LOADER] Block %d: magic0=0x%08X magic1=0x%08X target=0x%08X size=%u\n",
               blocks_total, block.magic_start0, block.magic_start1,
               block.target_addr, block.payload_size);

        if (block.payload_size > 0) {
            uint32_t w0, w1, w2, w3;
            memcpy(&w0, &block.data[0], 4);
            memcpy(&w1, &block.data[4], 4);
            memcpy(&w2, &block.data[8], 4);
            memcpy(&w3, &block.data[12], 4);
            fprintf(stderr, "[LOADER] First 16 bytes of payload: %08X %08X %08X %08X\n",
                   w0, w1, w2, w3);
        }

        /* Validate ALL UF2 magic numbers */
        if (block.magic_start0 != UF2_MAGIC_START0 ||
            block.magic_start1 != UF2_MAGIC_START1 ||
            block.magic_end != UF2_MAGIC_END) {
            fprintf(stderr, "[LOADER] WARNING: Block %d has invalid magic numbers\n", blocks_total);
            fprintf(stderr, "[LOADER] Expected: 0x%08X 0x%08X 0x%08X\n",
                   UF2_MAGIC_START0, UF2_MAGIC_START1, UF2_MAGIC_END);
            fprintf(stderr, "[LOADER] Got:      0x%08X 0x%08X 0x%08X\n",
                   block.magic_start0, block.magic_start1, block.magic_end);
            continue;
        }

        /* Detect family ID from first valid block */
        if (!family_detected && (block.flags & UF2_FLAG_FAMILY_PRESENT)) {
            family_id = block.file_size;
            family_detected = 1;
            fprintf(stderr, "[LOADER] UF2 family ID: 0x%08X (%s)\n",
                    family_id, family_id_name(family_id));

            switch (family_id) {
            case UF2_FAMILY_RP2040:     detected_arch = FW_ARCH_ARM_M0P; break;
            case UF2_FAMILY_RP2350_ARM: detected_arch = FW_ARCH_ARM_M33; break;
            case UF2_FAMILY_RP2350_RV:  detected_arch = FW_ARCH_RV32;    break;
            default:                    detected_arch = FW_ARCH_UNKNOWN;  break;
            }
        }

        /* Bounds check before writing */
        uint32_t offset;
        if (!uf2_block_flash_offset(&block, &offset)) {
            fprintf(stderr,
                    "[LOADER] WARNING: Block %d target 0x%08X size %u out of Flash bounds\n",
                    blocks_total, block.target_addr, block.payload_size);
            continue;
        }

        /* Write payload to Flash */
        memcpy(&cpu.flash[offset], block.data, block.payload_size);
        blocks_loaded++;

        /* DEBUG: Verify what we wrote */
        if (block.payload_size >= 4) {
            uint32_t verify;
            memcpy(&verify, &cpu.flash[offset], 4);
            fprintf(stderr, "[LOADER] Wrote to flash[0x%08X] = 0x%08X\n", offset, verify);
        }
    }

    fclose(f);
    fprintf(stderr, "[LOADER] Load complete: %d/%d valid blocks processed\n", blocks_loaded, blocks_total);

    if (!family_detected)
        fprintf(stderr, "[LOADER] No family ID found — assuming RP2040\n");

    /* DEBUG: Show first words of flash */
    uint32_t flash0, flash4;
    memcpy(&flash0, &cpu.flash[0], 4);
    memcpy(&flash4, &cpu.flash[4], 4);
    fprintf(stderr, "[LOADER] Flash[0x00000000] = 0x%08X\n", flash0);
    fprintf(stderr, "[LOADER] Flash[0x00000004] = 0x%08X\n", flash4);
    return (blocks_loaded > 0);
}
