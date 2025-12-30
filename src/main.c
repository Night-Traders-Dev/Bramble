/* Bramble RP2040 Emulator - Unified Main (Single & Dual-Core)
 *
 * This main.c handles both single-core and dual-core emulation modes.
 * Detects hardware based on emulator.h definitions.
 *
 * Features:
 *   - Automatic single/dual-core detection
 *   - Vector table at FLASH_BASE (0x10000000) with 0x100 offset to code
 *   - Unified peripheral initialization
 *   - Debug mode support for both cores
 *   - Status monitoring and statistics
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "emulator.h"
#include "gpio.h"
#include "timer.h"
#include "nvic.h"

/* Dual-core declarations (if available) */
/* Dual-core declarations (if available) */
#ifdef DUAL_CORE_ENABLED
  #include "emulator_dual.h"
  int any_core_running(void);
#endif


/* ============================================================================
 * Main Entry Point
 * ============================================================================ */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <firmware.uf2> [options]\n", argv[0]);
        fprintf(stderr, "\nOptions:\n");
        fprintf(stderr, "  -debug     Enable debug output (single-core) or Core 0 (dual-core)\n");
        fprintf(stderr, "  -debug1    Enable debug output for Core 1 (dual-core only)\n");
        fprintf(stderr, "  -asm       Show assembly instruction tracing\n");
        fprintf(stderr, "  -status    Print periodic status updates\n");
        return EXIT_FAILURE;
    }

    /* Parse command line arguments */
    char *firmware_path = argv[1];
    int debug_mode = 0;
    int debug1_mode = 0;
    int asm_mode = 0;
    int show_status = 0;

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug_mode = 1;
        } else if (strcmp(argv[i], "-debug1") == 0) {
            debug1_mode = 1;
        } else if (strcmp(argv[i], "-asm") == 0) {
            asm_mode = 1;
        } else if (strcmp(argv[i], "-status") == 0) {
            show_status = 1;
        }
    }

    /* ========================================================================
     * Initialization Phase
     * ======================================================================== */

#ifdef DUAL_CORE_ENABLED
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║       Bramble RP2040 Emulator - Dual-Core Mode           ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");

    printf("[Init] Initializing dual-core RP2040 emulator...\n");
    dual_core_init();
#else
    printf("=== Bramble RP2040 Emulator (Single-Core) ===\n\n");

    printf("[Init] Initializing single-core RP2040 emulator...\n");
    cpu_init();
    gpio_init();
    timer_init();
    nvic_init();

    /* Clear flash to erased state (0xFF) and RAM to 0 */
    memset(cpu.flash, 0xFF, FLASH_SIZE);
    memset(cpu.ram, 0, RAM_SIZE);
#endif

    printf("[Init] Loading firmware: %s\n", firmware_path);

    /* ========================================================================
     * Firmware Loading
     * ======================================================================== */

    if (!load_uf2(firmware_path)) {
        fprintf(stderr, "[Error] FATAL: Failed to load UF2 firmware\n");
        return EXIT_FAILURE;
    }

    printf("[Init] Firmware loaded successfully\n");

    /* ========================================================================
     * Boot Configuration
     * ======================================================================== */

#ifdef DUAL_CORE_ENABLED
    /* Dual-core: dual_core_init() already handles vector table loading */
    if (debug_mode) {
        cpu_set_debug_core(CORE0, 1);
        printf("[Init] Debug output enabled for Core 0\n");
    }
    if (debug1_mode) {
        cpu_set_debug_core(CORE1, 1);
        printf("[Init] Debug output enabled for Core 1\n");
    }

    printf("[Boot] Starting Core 0 from flash...\n");
    cpu_reset_core(CORE0);
    printf("[Boot] Core 0 SP = 0x%08X\n", cores[CORE0].r[13]);
    printf("[Boot] Core 0 PC = 0x%08X\n", cores[CORE0].r[15]);
    printf("[Boot] Core 1 held in reset (waiting for Core 0 to start)\n");
    printf("\n");

#else
    /* Single-core: Manually load vector table */
    printf("[Boot] Initializing RP2040...\n");

    /* Read vector table from FLASH_BASE + 0x100 (after boot2)
     * The actual vector table is at FLASH_BASE, but code starts at +0x100
     * We read from where the code will place the vector info
     */
    uint32_t vector_table = FLASH_BASE + 0x100;
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

    if ((reset_vector & 0x1) == 0) {
        fprintf(stderr, "[Boot] ERROR: Reset vector 0x%08X missing Thumb bit\n", reset_vector);
        return EXIT_FAILURE;
    }

    /* Initialize CPU registers */
    cpu.r[13] = initial_sp;           /* Set stack pointer */
    cpu.r[15] = reset_vector & ~1;    /* Set PC (clear thumb bit for emulator) */

    printf("[Boot] SP = 0x%08X\n", cpu.r[13]);
    printf("[Boot] PC = 0x%08X\n", cpu.r[15]);

    if (debug_mode) {
        cpu.debug_enabled = 1;
        printf("[Boot] Debug mode enabled (CPU step output)\n");
    }

    if (asm_mode) {
        cpu.debug_asm = 1;
        printf("[Boot] Assembly mode enabled (instruction tracing)\n");
    }

    cpu_reset_from_flash();
#endif

    /* ========================================================================
     * Execution Phase
     * ======================================================================== */

    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf("Executing...\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

#ifdef DUAL_CORE_ENABLED
    /* Dual-core execution loop */
    uint32_t instruction_count = 0;
    uint32_t step_count = 0;

    while (any_core_running()) {
        dual_core_step();
        instruction_count += 2;
        step_count++;

        if (show_status && (step_count % 1000 == 0)) {
            printf("[Status] Step %u (Inst %u)\n", step_count, instruction_count);
            printf(" Core 0: PC=0x%08X SP=0x%08X %s\n",
                   cores[CORE0].r[15], cores[CORE0].r[13],
                   cores[CORE0].is_halted ? "(halted)" : "(running)");
            printf(" Core 1: PC=0x%08X SP=0x%08X %s\n",
                   cores[CORE1].r[15], cores[CORE1].r[13],
                   cores[CORE1].is_halted ? "(halted)" : "(running)");
            printf(" FIFO0: %u messages, FIFO1: %u messages\n",
                   fifo[CORE0].count, fifo[CORE1].count);
            printf("\n");
        }

        /* Safety limit: prevent infinite loops */
        if (instruction_count > 10000000) {
            printf("[Warning] Instruction limit reached (10M)\n");
            break;
        }
    }

    /* ========================================================================
     * Completion Phase (Dual-Core)
     * ======================================================================== */

    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf("Execution Complete\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    dual_core_status();

    printf("═══════════════════════════════════════════════════════════\n");
    printf("Statistics:\n");
    printf(" Total Instructions: %u\n", instruction_count);
    printf(" Total Steps: %u\n", step_count);
    printf(" Core 0 Steps: %u\n", cores[CORE0].step_count);
    printf(" Core 1 Steps: %u\n", cores[CORE1].step_count);
    printf("═══════════════════════════════════════════════════════════\n");

#else
    /* Single-core execution loop */
    printf("[Boot] Starting execution...\n");

    while (!cpu_is_halted()) {
        cpu_step();
    }

    /* ========================================================================
     * Completion Phase (Single-Core)
     * ======================================================================== */

    printf("[Boot] CPU halted at PC=0x%08X\n", cpu.r[15]);
    printf("[Boot] Execution complete. Total steps: %u\n", cpu.step_count);
#endif

    return EXIT_SUCCESS;
}