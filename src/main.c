#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "emulator.h"

#define MAX_STEPS 100000  /* Increased from 10000 */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <firmware.uf2>\n", argv[0]);
        return EXIT_FAILURE;
    }

    printf("=== Bramble RP2040 Emulator ===\n\n");

    /* Initialize CPU */
    cpu_init();
    memset(cpu.flash, 0xFF, FLASH_SIZE);
    memset(cpu.ram, 0, RAM_SIZE);

    /* Load Firmware */
    printf("[Boot] Loading UF2 firmware...\n");
    if (!load_uf2(argv[1])) {
        fprintf(stderr, "[Boot] FATAL: Failed to load UF2\n");
        return EXIT_FAILURE;
    }

    /* Boot Sequence */
    printf("[Boot] Initializing RP2040...\n");
    
    uint32_t vector_table = FLASH_BASE + 0x100;
    
    uint32_t initial_sp = mem_read32(vector_table);
    uint32_t reset_vector = mem_read32(vector_table + 4);

    if (initial_sp < RAM_BASE || initial_sp >= RAM_BASE + RAM_SIZE) {
        fprintf(stderr, "[Boot] WARNING: Invalid Stack Pointer: 0x%08X\n", initial_sp);
        initial_sp = RAM_BASE + RAM_SIZE;
    }
    if (reset_vector < FLASH_BASE || reset_vector >= FLASH_BASE + FLASH_SIZE) {
        fprintf(stderr, "[Boot] FATAL: Invalid Reset Vector: 0x%08X\n", reset_vector);
        return EXIT_FAILURE;
    }

    cpu.r[13] = initial_sp;
    cpu.r[15] = reset_vector & ~1;

    printf("[Boot] SP = 0x%08X\n", cpu.r[13]);
    printf("[Boot] PC = 0x%08X\n", cpu.r[15]);
    printf("[Boot] Starting execution...\n\n");

    /* Execution Loop */
    for (int step = 0; step < MAX_STEPS; step++) {
        cpu_step();

        if (cpu_is_halted()) {
            printf("\n[CPU] Halted at step %d\n", step);
            break;
        }
    }

    printf("\n[Boot] Execution complete. Total steps: %u\n", cpu.step_count);
    return EXIT_SUCCESS;
}
