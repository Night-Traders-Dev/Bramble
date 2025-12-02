#include <stdio.h>
#include <string.h>
#include "emulator.h"

// --- UF2 File Structure ---
// Standard UF2 block is 512 bytes
typedef struct {
    uint32_t magicStart0;
    uint32_t magicStart1;
    uint32_t flags;
    uint32_t targetAddr;
    uint32_t payloadSize;
    uint32_t blockNo;
    uint32_t numBlocks;
    uint32_t fileSize; // or familyID
    uint8_t  data[476];
    uint32_t magicEnd;
} __attribute__((packed)) uf2_block_t;

int load_uf2(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror("Failed to open UF2");
        return 0;
    }

    uf2_block_t block;
    int blocks_loaded = 0;

    while (fread(&block, 1, 512, f) == 512) {
        // Check UF2 Magic Numbers
        if (block.magicStart0 == 0x0A324655 && block.magicEnd == 0x0AB16F30) {
            // Ensure target is within Flash bounds
            if (block.targetAddr >= FLASH_BASE && block.targetAddr < FLASH_BASE + FLASH_SIZE) {
                uint32_t offset = block.targetAddr - FLASH_BASE;
                // Copy payload (usually 256 bytes) into emulated Flash
                memcpy(&cpu.flash[offset], block.data, block.payloadSize);
                blocks_loaded++;
            }
        }
    }
    
    fclose(f);
    printf("[Loader] Loaded %d blocks into Flash.
", blocks_loaded);
    return 1;
}
