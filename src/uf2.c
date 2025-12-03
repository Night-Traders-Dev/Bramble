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
    uint32_t file_size;    /* or familyID */
    uint8_t  data[476];
    uint32_t magic_end;
} __attribute__((packed)) uf2_block_t;

#define UF2_MAGIC_START0 0x0A324655  /* "UF2
" */
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

    printf("[LOADER] Starting UF2 load from: %s", filename);

    while (fread(&block, 1, 512, f) == 512) {
        blocks_total++;

        /* DEBUG: Print block info */
        printf("[LOADER] Block %d: magic0=0x%08X magic1=0x%08X target=0x%08X size=%u",
               blocks_total, block.magic_start0, block.magic_start1, 
               block.target_addr, block.payload_size);
        
        if (block.payload_size > 0) {
            printf("[LOADER] First 16 bytes of payload: %08X %08X %08X %08X",
                   *(uint32_t*)&block.data[0],
                   *(uint32_t*)&block.data[4],
                   *(uint32_t*)&block.data[8],
                   *(uint32_t*)&block.data[12]);
        }

        /* Validate ALL UF2 magic numbers */
        if (block.magic_start0 != UF2_MAGIC_START0 ||
            block.magic_start1 != UF2_MAGIC_START1 ||
            block.magic_end != UF2_MAGIC_END) {
            printf("[LOADER] WARNING: Block %d has invalid magic numbers", blocks_total);
            printf("[LOADER] Expected: 0x%08X 0x%08X 0x%08X",
                   UF2_MAGIC_START0, UF2_MAGIC_START1, UF2_MAGIC_END);
            printf("[LOADER] Got:      0x%08X 0x%08X 0x%08X",
                   block.magic_start0, block.magic_start1, block.magic_end);
            continue;
        }

        /* Bounds check before writing */
        if (block.target_addr < FLASH_BASE ||
            block.target_addr + block.payload_size > FLASH_BASE + FLASH_SIZE) {
            printf("[LOADER] WARNING: Block %d target 0x%08X out of Flash bounds",
                   blocks_total, block.target_addr);
            continue;
        }

        /* Write payload to Flash */
        uint32_t offset = block.target_addr - FLASH_BASE;
        memcpy(&cpu.flash[offset], block.data, block.payload_size);
        blocks_loaded++;

        /* DEBUG: Verify what we wrote */
        if (block.payload_size >= 4) {
            printf("[LOADER] Wrote to flash[0x%08X] = 0x%08X", 
                   offset, *(uint32_t*)&cpu.flash[offset]);
        }
    }

    fclose(f);
    printf("[LOADER] Load complete: %d/%d valid blocks processed", blocks_loaded, blocks_total);
    
    /* DEBUG: Show first words of flash */
    printf("[LOADER] Flash[0x00000000] = 0x%08X", *(uint32_t*)&cpu.flash[0]);
    printf("[LOADER] Flash[0x00000004] = 0x%08X", *(uint32_t*)&cpu.flash[4]);
    
    return (blocks_loaded > 0);
}
