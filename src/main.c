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
#include "netbridge.h"
#include "wire.h"
#include "storage.h"
#include "sdcard.h"
#include "emmc.h"
#include "fuse_mount.h"
#include "corepool.h"
#include "cyw43.h"


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
        fprintf(stderr, "  -cores <N|auto> Active cores per instance (1, 2, or auto; default: 2)\n");
        fprintf(stderr, "  -no-boot2  Skip boot2 even if detected in firmware\n");
        fprintf(stderr, "  -debug-mem Log unmapped peripheral accesses\n");
        fprintf(stderr, "  -flash <path> Persistent flash storage (2MB file)\n");
        fprintf(stderr, "  -mount <dir>  Mount flash FAT filesystem via FUSE (requires -flash)\n");
        fprintf(stderr, "\nStorage:\n");
        fprintf(stderr, "  -sdcard <path>              Attach SD card image to SPI1\n");
        fprintf(stderr, "  -sdcard-spi <0|1>           SPI bus for SD card (default: 1)\n");
        fprintf(stderr, "  -sdcard-size <MB>           SD card size in MB (default: 64)\n");
        fprintf(stderr, "  -emmc <path>                Attach eMMC image to SPI0\n");
        fprintf(stderr, "  -emmc-spi <0|1>             SPI bus for eMMC (default: 0)\n");
        fprintf(stderr, "  -emmc-size <MB>             eMMC size in MB (default: 128)\n");
        fprintf(stderr, "\nNetworking:\n");
        fprintf(stderr, "  -net-uart0 <port>           Bridge UART0 to TCP server on port\n");
        fprintf(stderr, "  -net-uart1 <port>           Bridge UART1 to TCP server on port\n");
        fprintf(stderr, "  -net-uart0-connect <h:p>    Connect UART0 to remote host:port\n");
        fprintf(stderr, "  -net-uart1-connect <h:p>    Connect UART1 to remote host:port\n");
        fprintf(stderr, "\nMulti-Device Wiring:\n");
        fprintf(stderr, "  -wire-uart0 <path>          Wire UART0 to peer via Unix socket\n");
        fprintf(stderr, "  -wire-uart1 <path>          Wire UART1 to peer via Unix socket\n");
        fprintf(stderr, "  -wire-gpio <path>           Wire GPIO pins to peer via Unix socket\n");
        fprintf(stderr, "\nWiFi (Pico W):\n");
        fprintf(stderr, "  -wifi                       Enable CYW43 WiFi chip emulation\n");
        fprintf(stderr, "  -tap <ifname>               Bridge WiFi to TAP interface (implies -wifi)\n");
        fprintf(stderr, "\nPerformance:\n");
        fprintf(stderr, "  -jit        Enable JIT basic block compilation for hot loops\n");
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
    char *mount_path = NULL;
    char *sdcard_path = NULL;
    int sdcard_spi = 1;
    size_t sdcard_size = SD_DEFAULT_SIZE;
    char *emmc_path = NULL;
    int emmc_spi = 0;
    size_t emmc_size = EMMC_DEFAULT_SIZE;
    char *tap_name = NULL;
    int jit_mode = 0;
    int threaded_mode = 0;   /* Use pthread-per-core execution */
    int cores_auto = 0;     /* -cores auto requested */
    static sdcard_t sdcard;
    static emmc_t emmc_dev;

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
            usb_cdc_stdout_enabled = 1;
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
        } else if (strcmp(argv[i], "-cores") == 0) {
            if (i + 1 < argc) {
                i++;
                if (strcmp(argv[i], "auto") == 0) {
                    cores_auto = 1;
                    threaded_mode = 1;
                } else {
                    int n = atoi(argv[i]);
                    if (n >= 1 && n <= MAX_CORES) {
                        num_active_cores = n;
                    } else {
                        fprintf(stderr, "[Error] -cores must be 1, 2, or auto\n");
                        return EXIT_FAILURE;
                    }
                    threaded_mode = 1;
                }
            }
        } else if (strcmp(argv[i], "-no-boot2") == 0) {
            no_boot2 = 1;
        } else if (strcmp(argv[i], "-debug-mem") == 0) {
            mem_debug_unmapped = 1;
        } else if (strcmp(argv[i], "-flash") == 0) {
            if (i + 1 < argc) {
                flash_path = argv[++i];
            }
        } else if (strcmp(argv[i], "-mount") == 0) {
            if (i + 1 < argc) {
                mount_path = argv[++i];
            }
        } else if (strcmp(argv[i], "-sdcard") == 0) {
            if (i + 1 < argc) {
                sdcard_path = argv[++i];
            }
        } else if (strcmp(argv[i], "-sdcard-spi") == 0) {
            if (i + 1 < argc) {
                sdcard_spi = atoi(argv[++i]);
            }
        } else if (strcmp(argv[i], "-sdcard-size") == 0) {
            if (i + 1 < argc) {
                sdcard_size = (size_t)atoi(argv[++i]) * 1024 * 1024;
            }
        } else if (strcmp(argv[i], "-emmc") == 0) {
            if (i + 1 < argc) {
                emmc_path = argv[++i];
            }
        } else if (strcmp(argv[i], "-emmc-spi") == 0) {
            if (i + 1 < argc) {
                emmc_spi = atoi(argv[++i]);
            }
        } else if (strcmp(argv[i], "-emmc-size") == 0) {
            if (i + 1 < argc) {
                emmc_size = (size_t)atoi(argv[++i]) * 1024 * 1024;
            }
        } else if (strcmp(argv[i], "-net-uart0") == 0) {
            if (i + 1 < argc) {
                net_bridge.uart[0].mode = NET_MODE_LISTEN;
                net_bridge.uart[0].port = atoi(argv[++i]);
            }
        } else if (strcmp(argv[i], "-net-uart1") == 0) {
            if (i + 1 < argc) {
                net_bridge.uart[1].mode = NET_MODE_LISTEN;
                net_bridge.uart[1].port = atoi(argv[++i]);
            }
        } else if (strcmp(argv[i], "-net-uart0-connect") == 0) {
            if (i + 1 < argc) {
                char *arg = argv[++i];
                char *colon = strrchr(arg, ':');
                if (colon) {
                    *colon = '\0';
                    strncpy(net_bridge.uart[0].host, arg, sizeof(net_bridge.uart[0].host) - 1);
                    net_bridge.uart[0].remote_port = atoi(colon + 1);
                    net_bridge.uart[0].mode = NET_MODE_CONNECT;
                    *colon = ':';
                }
            }
        } else if (strcmp(argv[i], "-net-uart1-connect") == 0) {
            if (i + 1 < argc) {
                char *arg = argv[++i];
                char *colon = strrchr(arg, ':');
                if (colon) {
                    *colon = '\0';
                    strncpy(net_bridge.uart[1].host, arg, sizeof(net_bridge.uart[1].host) - 1);
                    net_bridge.uart[1].remote_port = atoi(colon + 1);
                    net_bridge.uart[1].mode = NET_MODE_CONNECT;
                    *colon = ':';
                }
            }
        } else if (strcmp(argv[i], "-wire-uart0") == 0) {
            if (i + 1 < argc) {
                wire_add_link(argv[++i], WIRE_MSG_UART_DATA, 0);
            }
        } else if (strcmp(argv[i], "-wire-uart1") == 0) {
            if (i + 1 < argc) {
                wire_add_link(argv[++i], WIRE_MSG_UART_DATA, 1);
            }
        } else if (strcmp(argv[i], "-wire-gpio") == 0) {
            if (i + 1 < argc) {
                wire_add_link(argv[++i], WIRE_MSG_GPIO_PIN, 0);
            }
        } else if (strcmp(argv[i], "-wifi") == 0) {
            cyw43.enabled = 1;
        } else if (strcmp(argv[i], "-tap") == 0) {
            if (i + 1 < argc) {
                tap_name = argv[++i];
                cyw43.enabled = 1;  /* -tap implies -wifi */
            }
        } else if (strcmp(argv[i], "-jit") == 0) {
            jit_mode = 1;
        }
    }

    /* ========================================================================
     * Initialization Phase
     * ======================================================================== */

    /* Initialize core pool (detect host CPUs, set up threading) */
    corepool_init();

    if (cores_auto) {
        num_active_cores = corepool_query_cores();
        fprintf(stderr, "[CorePool] Auto-detected: %d core(s) for this instance\n",
                num_active_cores);
    }

    fprintf(stderr,"\n╔════════════════════════════════════════════════════════════╗\n");
    fprintf(stderr,"║       Bramble RP2040 Emulator - %s Mode%s          ║\n",
            num_active_cores == 1 ? "Single-Core" : "Dual-Core",
            threaded_mode ? " (threaded)" : "          ");
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

    /* Initialize CYW43 WiFi emulation if enabled */
    if (cyw43.enabled) {
        cyw43_init();
        fprintf(stderr, "[WiFi] CYW43439 emulation enabled\n");
        if (tap_name) {
            if (cyw43_tap_open(tap_name) < 0) {
                fprintf(stderr, "[WiFi] Failed to open TAP interface '%s'\n", tap_name);
            }
        }
    }

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
        /* Enable write-through persistence */
        flash_persist_set_path(flash_path);
        flash_persist_open();
    }

    /* FUSE mount: expose flash filesystem as host directory */
    if (mount_path) {
        if (!flash_path) {
            fprintf(stderr, "[Error] -mount requires -flash <path>\n");
            return EXIT_FAILURE;
        }
        /* CircuitPython filesystem starts at 1MB offset in flash */
        uint32_t fs_offset = 0x100000;
        uint32_t fs_size = FLASH_SIZE - fs_offset;
        fuse_set_flash_offset(fs_offset);
        fuse_mount_start(&cpu.flash[fs_offset], fs_size, mount_path);
    }

    /* Detect boot2 in firmware */
    if (!no_boot2 && cpu_has_boot2()) {
        cpu_set_boot2(1);
        fprintf(stderr,"[Init] Boot2 detected in firmware (first 256 bytes)\n");
    }

    fprintf(stderr,"[Init] Initializing dual-core RP2040 emulator...\n");
    dual_core_init();

    /* JIT basic block compilation */
    if (jit_mode) {
        jit_init();
        fprintf(stderr,"[Init] JIT basic block compilation enabled\n");
    }

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

    /* Network bridge initialization */
    if (net_bridge_init() < 0) {
        fprintf(stderr, "[Error] Failed to initialize network bridge\n");
        return EXIT_FAILURE;
    }

    /* Wire protocol initialization */
    if (wire_init() < 0) {
        fprintf(stderr, "[Error] Failed to initialize wire protocol\n");
        return EXIT_FAILURE;
    }

    /* SD card initialization */
    if (sdcard_path) {
        if (sdcard_init(&sdcard, sdcard_path, sdcard_size) < 0) {
            fprintf(stderr, "[Error] Failed to initialize SD card\n");
            return EXIT_FAILURE;
        }
        spi_attach_device(sdcard_spi, sdcard_spi_xfer, sdcard_spi_cs, &sdcard);
        fprintf(stderr, "[Init] SD card (%zu MB) on SPI%d: %s\n",
                sdcard_size / (1024 * 1024), sdcard_spi, sdcard_path);
    }

    /* eMMC initialization */
    if (emmc_path) {
        if (emmc_init(&emmc_dev, emmc_path, emmc_size) < 0) {
            fprintf(stderr, "[Error] Failed to initialize eMMC\n");
            return EXIT_FAILURE;
        }
        spi_attach_device(emmc_spi, emmc_spi_xfer, emmc_spi_cs, &emmc_dev);
        fprintf(stderr, "[Init] eMMC (%zu MB) on SPI%d: %s\n",
                emmc_size / (1024 * 1024), emmc_spi, emmc_path);
    }

    fprintf(stderr,"[Boot] Starting Core 0 from flash...\n");
    cpu_reset_core(CORE0);
    fprintf(stderr,"[Boot] Core 0 SP = 0x%08X\n", cores[CORE0].r[13]);
    fprintf(stderr,"[Boot] Core 0 PC = 0x%08X\n", cores[CORE0].r[15]);
    if (num_active_cores > 1) {
        cpu_reset_core(CORE1);
        fprintf(stderr,"[Boot] Core 1 SP = 0x%08X\n", cores[CORE1].r[13]);
        fprintf(stderr,"[Boot] Core 1 PC = 0x%08X\n", cores[CORE1].r[15]);
    }
    fprintf(stderr,"\n");

    /* Register with core pool for multi-instance coordination */
    corepool_register(num_active_cores);

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

    /* Execution loop */
    uint32_t instruction_count = 0;
    uint32_t step_count = 0;

    if (threaded_mode && !gdb_enabled) {
        /* ====== Threaded execution: one host pthread per emulated core ====== */
        fprintf(stderr, "[CorePool] Host CPUs: %d, emulated cores: %d\n",
                corepool.host_cpus, num_active_cores);

        corepool_start_threads();

        /* Main thread handles I/O polling and watchdog */
        while (any_core_running() && corepool.running) {
            usleep(1000);  /* 1ms polling interval */

            /* Poll stdin */
            if (stdin_enabled) {
                corepool_lock();
                uart_stdin_poll();
                corepool_unlock();
            }

            /* Poll network, wire, and WiFi TAP */
            corepool_lock();
            net_bridge_poll();
            wire_poll();
            cyw43_tap_poll();
            corepool_unlock();

            /* Periodic storage flush */
            step_count++;
            if ((step_count & 0x3FF) == 0) {
                if (sdcard_path) sdcard_flush(&sdcard);
                if (emmc_path) emmc_flush(&emmc_dev);
            }

            /* Watchdog reboot */
            if (watchdog_reboot_pending) {
                corepool_stop_threads();
                fprintf(stderr,"[Watchdog] Reboot triggered\n");
                watchdog_reboot_pending = 0;
                clocks_state.wdog_ctrl &= ~(1u << 31);
                dual_core_init();
                nvic_init();
                timer_init();
                rom_init();
                if (no_boot2 || !cpu_has_boot2()) {
                    cpu_reset_core(CORE0);
                }
                corepool_start_threads();
            }

            /* Status reporting */
            if (show_status && (step_count % 1000 == 0)) {
                fprintf(stderr,"[Status] Core 0: PC=0x%08X %s%s  Core 1: PC=0x%08X %s%s\n",
                       cores[CORE0].r[15],
                       cores[CORE0].is_halted ? "halted" : "run",
                       cores[CORE0].is_wfi ? "/wfi" : "",
                       cores[CORE1].r[15],
                       cores[CORE1].is_halted ? "halted" : "run",
                       cores[CORE1].is_wfi ? "/wfi" : "");
            }

            /* Safety limit */
            if (!stdin_enabled && step_count > 1000000) {
                /* Approximate: each poll ~= 1000 instructions */
                instruction_count = cores[CORE0].step_count + cores[CORE1].step_count;
                if (instruction_count > 1000000000) {
                    fprintf(stderr,"[Warning] Instruction limit reached (1B)\n");
                    break;
                }
            }
        }

        corepool_stop_threads();
        instruction_count = cores[CORE0].step_count + cores[CORE1].step_count;

    } else {
        /* ====== Cooperative execution: original single-threaded round-robin ====== */
        while (any_core_running()) {

            /* GDB: check for breakpoint/watchpoint on both cores */
            if (gdb_enabled && gdb.active) {
                int should_stop = 0;
                for (int gc = 0; gc < num_active_cores; gc++) {
                    if (!cores[gc].is_halted &&
                        gdb_should_stop(cores[gc].r[15], gc)) {
                        gdb.stop_core = gc;
                        should_stop = 1;
                        break;
                    }
                }
                if (should_stop) {
                    int result = gdb_handle();
                    if (result < 0) {
                        gdb_enabled = 0;  /* Detached or killed */
                    }
                }
            }

            dual_core_step();
            pio_step();
            usb_step();
            instruction_count += (!cores[CORE0].is_halted && !cores[CORE0].is_wfi) +
                                (!cores[CORE1].is_halted && !cores[CORE1].is_wfi);
            step_count++;

            /* Poll stdin for UART Rx data every 1024 steps */
            if (stdin_enabled && (step_count & 0x3FF) == 0) {
                uart_stdin_poll();
            }

            /* Poll network bridges, wire links, and WiFi TAP every 1024 steps */
            if ((step_count & 0x3FF) == 0) {
                net_bridge_poll();
                wire_poll();
                cyw43_tap_poll();
            }

            /* Flush dirty storage devices every ~1M steps */
            if ((step_count & 0xFFFFF) == 0) {
                if (sdcard_path) sdcard_flush(&sdcard);
                if (emmc_path) emmc_flush(&emmc_dev);
            }

            if (show_status && (step_count % 1000 == 0)) {
                fprintf(stderr,"[Status] Step %u (Inst %u)\n", step_count, instruction_count);
                fprintf(stderr," Core 0: PC=0x%08X SP=0x%08X %s%s\n",
                       cores[CORE0].r[15], cores[CORE0].r[13],
                       cores[CORE0].is_halted ? "(halted)" : "(running)",
                       cores[CORE0].is_wfi ? " [WFI]" : "");
                if (num_active_cores > 1) {
                    fprintf(stderr," Core 1: PC=0x%08X SP=0x%08X %s%s\n",
                           cores[CORE1].r[15], cores[CORE1].r[13],
                           cores[CORE1].is_halted ? "(halted)" : "(running)",
                           cores[CORE1].is_wfi ? " [WFI]" : "");
                }
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

    net_bridge_cleanup();
    wire_cleanup();
    cyw43_tap_close();

    /* Unmount FUSE filesystem */
    if (mount_path) {
        fuse_mount_stop();
    }

    /* Flush and cleanup storage devices */
    if (sdcard_path) {
        sdcard_cleanup(&sdcard);
    }
    if (emmc_path) {
        emmc_cleanup(&emmc_dev);
    }

    /* Save flash and close persistence */
    if (flash_path) {
        flash_persist_save_all();
        flash_persist_close();
    }

    /* Core pool cleanup */
    corepool_cleanup();

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
    icache_report_stats();
    if (jit_mode) {
        jit_report_stats();
    }
    fprintf(stderr,"═══════════════════════════════════════════════════════════\n");


    return EXIT_SUCCESS;
}
