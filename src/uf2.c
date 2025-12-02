#include <stdio.h>
#include <string.h>
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
    uint32_t file_size;    /* or familyID */
    uint8_t  data[476];
    uint32_t magic_end;
} __attribute__((packed)) uf2_block_t;

#define UF2_MAGIC_START0 0x0A324655  /* "UF2\n" */
#define UF2_MAGIC_START1 0x9E5D5157
#define UF2_MAGIC_END    0x0AB16F30  /* End marker */

int load_uf2(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror("[LOADER] Failed to open UF2");
        return 0;
    }

    uf2_block_t block;
    int blocks_loaded = 0;
    int blocks_total = 0;

    while (fread(&block, 1, 512, f) == 512) {
        blocks_total++;

        /* Validate UF2 magic numbers */
        if (block.magic_start0 != UF2_MAGIC_START0 || 
            block.magic_end != UF2_MAGIC_END) {
            printf("[LOADER] WARNING: Block %d has invalid magic numbers\n", blocks_total);
            continue;
        }

        /* Bounds check before writing */
        if (block.target_addr < FLASH_BASE || 
            block.target_addr + block.payload_size > FLASH_BASE + FLASH_SIZE) {
            printf("[LOADER] WARNING: Block %d target 0x%08X out of Flash bounds\n", 
                   blocks_total, block.target_addr);
            continue;
        }

        /* Write payload to Flash */
        uint32_t offset = block.target_addr - FLASH_BASE;
        memcpy(&cpu.flash[offset], block.data, block.payload_size);
        blocks_loaded++;
    }

    fclose(f);
    printf("[LOADER] Loaded %d/%d blocks into Flash\n", blocks_loaded, blocks_total);
    return (blocks_loaded > 0);
}
