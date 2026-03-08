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
 *   - Non-blocking stdin polling for UART Rx (-stdin flag)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "emulator.h"
#include "gpio.h"
#include "timer.h"
#include "nvic.h"
#include "clocks.h"
#include "adc.h"
#include "rom.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "pwm.h"
#include "dma.h"
#include "pio.h"


int any_core_running(void);

/* ============================================================================
 * UART Stdin Polling
 *
 * When enabled with -stdin, polls stdin for input and pushes bytes into
 * UART0's RX FIFO. Uses non-blocking I/O to avoid stalling the emulator.
 * ============================================================================ */

static int stdin_enabled = 0;
static int stdin_nonblock_set = 0;

static void uart_stdin_init(void) {
    /* Set stdin to non-blocking mode */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags != -1) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        stdin_nonblock_set = 1;
    }
}

static void uart_stdin_cleanup(void) {
    if (stdin_nonblock_set) {
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags != -1) {
            fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
        }
    }
}

/* Poll stdin and push any available bytes into UART0 RX FIFO.
 * Called periodically from the main execution loop. */
static void uart_stdin_poll(void) {
    struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };
    if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        uint8_t buf[16];
        ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n > 0) {
            for (ssize_t i = 0; i < n; i++) {
                uart_rx_push(0, buf[i]);
            }
        }
    }
}

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
        fprintf(stderr, "  -stdin     Enable stdin polling for UART0 Rx input\n");
        return EXIT_FAILURE;
    }

    /* Parse command line arguments */
    char *firmware_path = argv[1];
    int debug_mode = 0;
    int debug1_mode = 0;
    int show_status = 0;

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug_mode = 1;
        } else if (strcmp(argv[i], "-debug1") == 0) {
            debug1_mode = 1;
        } else if (strcmp(argv[i], "-asm") == 0) {
            /* Reserved for future use */
        } else if (strcmp(argv[i], "-status") == 0) {
            show_status = 1;
        } else if (strcmp(argv[i], "-stdin") == 0) {
            stdin_enabled = 1;
        }
    }

    /* ========================================================================
     * Initialization Phase
     * ======================================================================== */

    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║       Bramble RP2040 Emulator - Dual-Core Mode           ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");


    printf("[Init] Loading firmware: %s\n", firmware_path);

    /* ========================================================================
     * Firmware Loading (auto-detect UF2 or ELF format)
     * ======================================================================== */

    /* Initialize CPU and peripherals */
    cpu_init();
    rom_init();
    uart_init();
    spi_init();
    i2c_init();
    pwm_init();
    dma_init();
    pio_init();
    clocks_init();
    adc_init();

    int loaded = 0;
    size_t path_len = strlen(firmware_path);

    /* Try ELF if filename ends with .elf */
    if (path_len > 4 && strcmp(firmware_path + path_len - 4, ".elf") == 0) {
        loaded = load_elf(firmware_path);
    } else {
        loaded = load_uf2(firmware_path);
    }

    if (!loaded) {
        fprintf(stderr, "[Error] FATAL: Failed to load firmware\n");
        return EXIT_FAILURE;
    }

    printf("[Init] Firmware loaded successfully\n");

    printf("[Init] Initializing dual-core RP2040 emulator...\n");
    dual_core_init();

    /* ========================================================================
     * Boot Configuration
     * ======================================================================== */

    /* Dual-core: dual_core_init() already handles vector table loading */
    if (debug_mode) {
        cpu_set_debug_core(CORE0, 1);
        printf("[Init] Debug output enabled for Core 0\n");
    }
    if (debug1_mode) {
        cpu_set_debug_core(CORE1, 1);
        printf("[Init] Debug output enabled for Core 1\n");
    }

    if (stdin_enabled) {
        uart_stdin_init();
        printf("[Init] Stdin polling enabled for UART0 Rx\n");
    }

    printf("[Boot] Starting Core 0 from flash...\n");
    cpu_reset_core(CORE0);
    printf("[Boot] Core 0 SP = 0x%08X\n", cores[CORE0].r[13]);
    printf("[Boot] Core 0 PC = 0x%08X\n", cores[CORE0].r[15]);
    cpu_reset_core(CORE1);
    printf("[Boot] Core 1 SP = 0x%08X\n", cores[CORE1].r[13]);
    printf("[Boot] Core 1 PC = 0x%08X\n", cores[CORE1].r[15]);
    printf("\n");


    /* ========================================================================
     * Execution Phase
     * ======================================================================== */

    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf("Executing...\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    /* Dual-core execution loop */
    uint32_t instruction_count = 0;
    uint32_t step_count = 0;

    while (any_core_running()) {

        dual_core_step();
        instruction_count += (!cores[CORE0].is_halted) + (!cores[CORE1].is_halted);
        step_count++;

        /* Poll stdin for UART Rx data every 1024 steps */
        if (stdin_enabled && (step_count & 0x3FF) == 0) {
            uart_stdin_poll();
        }

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

    if (stdin_enabled) {
        uart_stdin_cleanup();
    }

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


    return EXIT_SUCCESS;
}
