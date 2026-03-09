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
 *   - GDB remote debugging support (-gdb flag)
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
#include "gdb.h"
#include "usb.h"
#include "rtc.h"


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

/* Poll stdin and push any available bytes into UART0 RX FIFO and USB CDC.
 * Called periodically from the main execution loop. */
static void uart_stdin_poll(void) {
    struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN };
    if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        uint8_t buf[16];
        ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n > 0) {
            for (ssize_t i = 0; i < n; i++) {
                uint8_t ch = buf[i];
                /* Translate LF→CR: serial devices expect CR as line ending */
                if (ch == '\n') ch = '\r';
                uart_rx_push(0, ch);
                usb_cdc_rx_push(ch);
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
        fprintf(stderr, "  -gdb [port] Start GDB server (default port: %d)\n", GDB_DEFAULT_PORT);
        fprintf(stderr, "  -clock <MHz> Set CPU clock frequency (default: 1, real: 125)\n");
        fprintf(stderr, "  -no-boot2  Skip boot2 even if detected in firmware\n");
        fprintf(stderr, "  -debug-mem Log unmapped peripheral accesses\n");
        fprintf(stderr, "  -flash <path> Persistent flash storage (2MB file)\n");
        return EXIT_FAILURE;
    }

    /* Parse command line arguments */
    char *firmware_path = argv[1];
    int debug_mode = 0;
    int debug1_mode = 0;
    int show_status = 0;
    int gdb_enabled = 0;
    int gdb_port = GDB_DEFAULT_PORT;
    int no_boot2 = 0;
    char *flash_path = NULL;

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
        } else if (strcmp(argv[i], "-gdb") == 0) {
            gdb_enabled = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                gdb_port = atoi(argv[++i]);
            }
        } else if (strcmp(argv[i], "-clock") == 0) {
            if (i + 1 < argc) {
                uint32_t mhz = (uint32_t)atoi(argv[++i]);
                timing_set_clock_mhz(mhz);
            }
        } else if (strcmp(argv[i], "-no-boot2") == 0) {
            no_boot2 = 1;
        } else if (strcmp(argv[i], "-debug-mem") == 0) {
            mem_debug_unmapped = 1;
        } else if (strcmp(argv[i], "-flash") == 0) {
            if (i + 1 < argc) {
                flash_path = argv[++i];
            }
        }
    }

    /* ========================================================================
     * Initialization Phase
     * ======================================================================== */

    fprintf(stderr,"\n╔════════════════════════════════════════════════════════════╗\n");
    fprintf(stderr,"║       Bramble RP2040 Emulator - Dual-Core Mode           ║\n");
    fprintf(stderr,"╚════════════════════════════════════════════════════════════╝\n\n");


    fprintf(stderr,"[Init] Loading firmware: %s\n", firmware_path);

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
    usb_init();
    rtc_init();

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

    fprintf(stderr,"[Init] Firmware loaded successfully\n");

    /* Flash persistence: restore non-firmware sectors from flash file */
    if (flash_path) {
        /* Determine which 4KB sectors contain firmware data */
        uint8_t fw_sectors[FLASH_SIZE / 4096];
        for (int i = 0; i < (int)(FLASH_SIZE / 4096); i++) {
            fw_sectors[i] = 0;
            for (int j = 0; j < 4096; j++) {
                if (cpu.flash[i * 4096 + j] != 0xFF) {
                    fw_sectors[i] = 1;
                    break;
                }
            }
        }
        /* Load flash file and restore non-firmware sectors */
        FILE *ff = fopen(flash_path, "rb");
        if (ff) {
            uint8_t sector[4096];
            for (int i = 0; i < (int)(FLASH_SIZE / 4096); i++) {
                size_t n = fread(sector, 1, 4096, ff);
                if (n < 4096) break;
                if (!fw_sectors[i]) {
                    memcpy(&cpu.flash[i * 4096], sector, 4096);
                }
            }
            fclose(ff);
            fprintf(stderr,"[Init] Flash file loaded: %s (non-firmware sectors restored)\n", flash_path);
        }
    }

    /* Detect boot2 in firmware */
    if (!no_boot2 && cpu_has_boot2()) {
        cpu_set_boot2(1);
        fprintf(stderr,"[Init] Boot2 detected in firmware (first 256 bytes)\n");
    }

    fprintf(stderr,"[Init] Initializing dual-core RP2040 emulator...\n");
    dual_core_init();

    /* ========================================================================
     * Boot Configuration
     * ======================================================================== */

    /* Dual-core: dual_core_init() already handles vector table loading */
    if (debug_mode) {
        cpu_set_debug_core(CORE0, 1);
        fprintf(stderr,"[Init] Debug output enabled for Core 0\n");
    }
    if (debug1_mode) {
        cpu_set_debug_core(CORE1, 1);
        fprintf(stderr,"[Init] Debug output enabled for Core 1\n");
    }

    if (timing_config.cycles_per_us != 1) {
        fprintf(stderr,"[Init] Clock: %u MHz (%u cycles/µs)\n",
               timing_config.cycles_per_us, timing_config.cycles_per_us);
    }

    if (stdin_enabled) {
        uart_stdin_init();
        fprintf(stderr,"[Init] Stdin polling enabled for UART0 Rx\n");
    }

    fprintf(stderr,"[Boot] Starting Core 0 from flash...\n");
    cpu_reset_core(CORE0);
    fprintf(stderr,"[Boot] Core 0 SP = 0x%08X\n", cores[CORE0].r[13]);
    fprintf(stderr,"[Boot] Core 0 PC = 0x%08X\n", cores[CORE0].r[15]);
    cpu_reset_core(CORE1);
    fprintf(stderr,"[Boot] Core 1 SP = 0x%08X\n", cores[CORE1].r[13]);
    fprintf(stderr,"[Boot] Core 1 PC = 0x%08X\n", cores[CORE1].r[15]);
    fprintf(stderr,"\n");

    /* GDB server initialization */
    if (gdb_enabled) {
        if (gdb_init(gdb_port) < 0) {
            fprintf(stderr, "[Error] Failed to start GDB server\n");
            return EXIT_FAILURE;
        }
        /* Initial stop: let GDB inspect state before execution */
        gdb_handle();
    }

    /* ========================================================================
     * Execution Phase
     * ======================================================================== */

    fprintf(stderr,"\n");
    fprintf(stderr,"═══════════════════════════════════════════════════════════\n");
    fprintf(stderr,"Executing...\n");
    fprintf(stderr,"═══════════════════════════════════════════════════════════\n\n");

    /* Dual-core execution loop */
    uint32_t instruction_count = 0;
    uint32_t step_count = 0;

    while (any_core_running()) {

        /* GDB: check for breakpoint or single-step before executing */
        if (gdb_enabled && gdb.active && gdb_should_stop(cores[CORE0].r[15])) {
            int result = gdb_handle();
            if (result < 0) {
                gdb_enabled = 0;  /* Detached or killed */
            }
        }

        dual_core_step();
        pio_step();
        usb_step();
        instruction_count += (!cores[CORE0].is_halted) + (!cores[CORE1].is_halted);
        step_count++;

        /* Poll stdin for UART Rx data every 1024 steps */
        if (stdin_enabled && (step_count & 0x3FF) == 0) {
            uart_stdin_poll();
            /* Also push stdin bytes to USB CDC if enumerated */
            /* (uart_stdin_poll already handles UART; USB CDC uses same stdin) */
        }

        if (show_status && (step_count % 1000 == 0)) {
            fprintf(stderr,"[Status] Step %u (Inst %u)\n", step_count, instruction_count);
            fprintf(stderr," Core 0: PC=0x%08X SP=0x%08X %s\n",
                   cores[CORE0].r[15], cores[CORE0].r[13],
                   cores[CORE0].is_halted ? "(halted)" : "(running)");
            fprintf(stderr," Core 1: PC=0x%08X SP=0x%08X %s\n",
                   cores[CORE1].r[15], cores[CORE1].r[13],
                   cores[CORE1].is_halted ? "(halted)" : "(running)");
            fprintf(stderr," FIFO0: %u messages, FIFO1: %u messages\n",
                   fifo[CORE0].count, fifo[CORE1].count);
            fprintf(stderr,"\n");
        }

        /* Watchdog reboot: reset all cores and re-start from flash */
        if (watchdog_reboot_pending) {
            fprintf(stderr,"[Watchdog] Reboot triggered\n");
            watchdog_reboot_pending = 0;
            clocks_state.wdog_ctrl &= ~(1u << 31);  /* Clear trigger bit */
            dual_core_init();
            nvic_init();
            timer_init();
            rom_init();
            if (no_boot2 || !cpu_has_boot2()) {
                cpu_reset_core(CORE0);
            }
            instruction_count = 0;
            step_count = 0;
            continue;
        }

        /* Safety limit: prevent infinite loops (disabled in interactive/GDB mode) */
        if (!gdb_enabled && !stdin_enabled && instruction_count > 1000000000) {
            fprintf(stderr,"[Warning] Instruction limit reached (1B)\n");
            break;
        }
    }

    /* ========================================================================
     * Completion Phase (Dual-Core)
     * ======================================================================== */

    if (gdb_enabled) {
        gdb_cleanup();
    }

    if (stdin_enabled) {
        uart_stdin_cleanup();
    }

    /* Save flash to file if persistence enabled */
    if (flash_path) {
        FILE *ff = fopen(flash_path, "wb");
        if (ff) {
            fwrite(cpu.flash, 1, FLASH_SIZE, ff);
            fclose(ff);
            fprintf(stderr,"[Flash] Saved to %s\n", flash_path);
        } else {
            fprintf(stderr, "[Flash] Failed to save: %s\n", flash_path);
        }
    }

    fprintf(stderr,"\n");
    fprintf(stderr,"═══════════════════════════════════════════════════════════\n");
    fprintf(stderr,"Execution Complete\n");
    fprintf(stderr,"═══════════════════════════════════════════════════════════\n\n");

    dual_core_status();

    fprintf(stderr,"═══════════════════════════════════════════════════════════\n");
    fprintf(stderr,"Statistics:\n");
    fprintf(stderr," Total Instructions: %u\n", instruction_count);
    fprintf(stderr," Total Steps: %u\n", step_count);
    fprintf(stderr," Core 0 Steps: %u\n", cores[CORE0].step_count);
    fprintf(stderr," Core 1 Steps: %u\n", cores[CORE1].step_count);
    fprintf(stderr,"═══════════════════════════════════════════════════════════\n");


    return EXIT_SUCCESS;
}
