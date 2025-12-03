#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "emulator.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <firmware.uf2>", argv[0]);
        return EXIT_FAILURE;
    }

    printf("=== Bramble RP2040 Emulator ===\n\n");

    /* Initialize CPU state */
    cpu_init();
    
    /* Clear flash to erased state (0xFF) and RAM to 0 */
    memset(cpu.flash, 0xFF, FLASH_SIZE);
    memset(cpu.ram, 0, RAM_SIZE);

    printf("[Boot] Loading UF2 firmware...\n");
    if (!load_uf2(argv[1])) {
        fprintf(stderr, "[Boot] FATAL: Failed to load UF2\n");
        return EXIT_FAILURE;
    }

    printf("[Boot] Initializing RP2040...\n");

    /* Read vector table from START OF FLASH (0x10000000) */
    uint32_t vector_table = FLASH_BASE;  /* ✅ FIXED: No +0x100 offset */
    uint32_t initial_sp = mem_read32(vector_table);
    uint32_t reset_vector = mem_read32(vector_table + 4);

    /* Validate SP (should be top of RAM) */
    if (initial_sp != RAM_TOP) {
        fprintf(stderr, "[Boot] WARNING: Stack Pointer not at RAM_TOP: 0x%08X\n", initial_sp);
        /* Optional: Force it to RAM_TOP if firmware has wrong SP */
        /* initial_sp = RAM_TOP; */
    }

    /* Validate reset vector (should be in flash range) */
    if (reset_vector < FLASH_BASE || reset_vector >= FLASH_BASE + FLASH_SIZE) {
        fprintf(stderr, "[Boot] FATAL: Invalid Reset Vector: 0x%08X\n", reset_vector);
        return EXIT_FAILURE;
    }

    /* Initialize CPU registers */
    cpu.r[13] = initial_sp;           /* Set stack pointer */
    cpu.r[15] = reset_vector & ~1;    /* Set PC (clear thumb bit for emulator) */

    printf("[Boot] SP = 0x%08X\n", cpu.r[13]);
    printf("[Boot] PC = 0x%08X\n", cpu.r[15]);
    printf("[Boot] Starting execution...\n");

    /* Run until halt */
    while (!cpu_is_halted()) {
        cpu_step();
    }

    printf("[Boot] Execution complete. Total steps: %u\n", cpu.step_count);
    return EXIT_SUCCESS;
}
