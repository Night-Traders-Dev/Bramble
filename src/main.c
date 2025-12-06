#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "emulator.h"
#include "gpio.h"
#include "timer.h"
#include "nvic.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s [-debug] [-asm] <firmware.uf2>\n", argv[0]);
        fprintf(stderr, "  -debug    Enable verbose CPU step output\n");
        fprintf(stderr, "  -asm      Enable instruction-level tracing (POP, BX, etc.)\n");
        return EXIT_FAILURE;
    }

    /* Parse command line arguments */
    int debug_mode = 0;
    int asm_mode = 0;
    char *firmware_path = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-debug") == 0 || strcmp(argv[i], "--debug") == 0) {
            debug_mode = 1;
        } else if (strcmp(argv[i], "-asm") == 0 || strcmp(argv[i], "--asm") == 0) {
            asm_mode = 1;
        } else {
            firmware_path = argv[i];
        }
    }

    if (!firmware_path) {
        fprintf(stderr, "Error: No firmware file specified\n");
        fprintf(stderr, "Usage: %s [-debug] [-asm] <firmware.uf2>\n", argv[0]);
        return EXIT_FAILURE;
    }

    printf("=== Bramble RP2040 Emulator ===\n\n");

    /* Initialize CPU state */
    cpu_init();

    /* Set debug modes (independent flags) */
    cpu.debug_enabled = debug_mode;
    cpu.debug_asm = asm_mode;

    /* Initialize peripherals */
    gpio_init();
    timer_init();
    nvic_init();

    /* Clear flash to erased state (0xFF) and RAM to 0 */
    memset(cpu.flash, 0xFF, FLASH_SIZE);
    memset(cpu.ram, 0, RAM_SIZE);

    printf("[Boot] Loading UF2 firmware...\n");
    if (!load_uf2(firmware_path)) {
        fprintf(stderr, "[Boot] FATAL: Failed to load UF2\n");
        return EXIT_FAILURE;
    }

    printf("[Boot] Initializing RP2040...\n");

    /* Read vector table from START OF FLASH (0x10000000) */
    uint32_t vector_table = FLASH_BASE;
    uint32_t initial_sp = mem_read32(vector_table);
    uint32_t reset_vector = mem_read32(vector_table + 4);

    /* Validate SP (should be top of RAM) */
    if (initial_sp != RAM_TOP) {
        fprintf(stderr, "[Boot] WARNING: Stack Pointer not at RAM_TOP: 0x%08X\n", initial_sp);
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
    
    if (debug_mode) {
        printf("[Boot] Debug mode enabled (CPU step output)\n");
    }
    if (asm_mode) {
        printf("[Boot] Assembly mode enabled (instruction tracing)\n");
    }
    
    printf("[Boot] Starting execution...\n");

    /* Run until halt */
    while (!cpu_is_halted()) {
        cpu_step();
    }

    printf("[Boot] Execution complete. Total steps: %u\n", cpu.step_count);
    return EXIT_SUCCESS;
}
