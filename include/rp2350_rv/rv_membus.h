/*
 * RP2350 Memory Bus for RISC-V Hazard3
 *
 * Thin layer over the shared membus that handles RP2350-specific differences:
 *   - 520KB SRAM (vs 264KB on RP2040)
 *   - CLINT registers in SIO space
 *   - RP2350-specific peripherals (TICKS, POWMAN, QMI, OTP, TIMER1, etc.)
 *   - Hart 1 launch mailbox in SIO space
 *   - RP2350 SIO (CPUID, GPIO_HI for pins 32-47)
 *
 * Most peripherals share the same addresses as RP2040, so reads/writes
 * to UART, SPI, I2C, GPIO, etc. route through the existing membus.
 */

#ifndef RV_MEMBUS_H
#define RV_MEMBUS_H

#include <stdint.h>
#include "rp2350_rv/rv_clint.h"
#include "rp2350_rv/rv_cpu.h"
#include "rp2350_rv/rp2350_periph.h"
#include "tmds.h"

/* RP2350 SRAM: 520KB (10 banks of 64KB + 8KB scratch) */
#define RV_SRAM_SIZE    (520 * 1024)

#define RV_SIO_CPUID              0x00

/* SIO inter-processor FIFO, per core. RP2350 datasheet section 3.1.5: 0x54
 * writes this core's TX FIFO, 0x58 reads its RX FIFO. This is the only
 * documented channel for launching core 1 (section 5.3).
 *
 * SIO +0x1C0..0x1CC is NOT a hart-1 mailbox -- it is the TMDS encoder
 * (TMDS_CTRL, TMDS_WDATA, TMDS_PEEK_SINGLE, TMDS_POP_SINGLE). The emulator
 * previously claimed those four registers as a boot mailbox, which meant the
 * real TMDS block was unreachable and RISC-V firmware's multicore_launch_core1()
 * wrote its entry point into the pixel encoder. */
#define RV_SIO_FIFO_WR            0x054
#define RV_SIO_FIFO_RD            0x058

typedef struct {
    /* RP2350 SRAM (520KB, separate from RP2040 RAM) */
    uint8_t sram[RV_SRAM_SIZE];

    /* CLINT interrupt controller */
    rv_clint_state_t clint;

    /* RP2350-specific peripherals */
    rp2350_periph_state_t periph;

    /* Pointer to flash (shared with main emulator) */
    uint8_t *flash;
    uint32_t flash_size;

    /* ROM (32KB for RP2350) */
    uint8_t rom[32 * 1024];
    uint32_t rom_size;

    /* Hart 1 launch, driven by the RP2350 datasheet section 5.3 protocol:
     * core 0 pushes {0, 0, 1, vector_table, sp, entry} to core 1 through the
     * SIO inter-processor FIFO. These are the words collected so far. */
    uint32_t hart1_words[6];
    int      hart1_count;
    uint32_t hart1_entry;
    uint32_t hart1_sp;
    uint32_t hart1_arg;
    int hart1_launch_pending;

    /* RP2350 SIO extra state */
    uint32_t gpio_hi_out;     /* GPIO pins 32-47 output */
    uint32_t gpio_hi_oe;      /* GPIO pins 32-47 output enable */
    uint32_t gpio_hi_in;      /* GPIO pins 32-47 input */

    /* Architecture mode flag (for SIO CPUID) */
    int is_riscv;
} rv_membus_state_t;

/* Initialize RP2350 memory bus */
void rv_membus_init(rv_membus_state_t *bus, uint8_t *flash, uint32_t flash_size, uint32_t cycles_per_us);

/* Memory access */
uint32_t rv_mem_read32(rv_membus_state_t *bus, uint32_t addr);
void rv_mem_write32(rv_membus_state_t *bus, uint32_t addr, uint32_t val);
uint16_t rv_mem_read16(rv_membus_state_t *bus, uint32_t addr);
void rv_mem_write16(rv_membus_state_t *bus, uint32_t addr, uint16_t val);
uint8_t rv_mem_read8(rv_membus_state_t *bus, uint32_t addr);
void rv_mem_write8(rv_membus_state_t *bus, uint32_t addr, uint8_t val);

/* Check if hart 1 launch was requested (called from main loop) */
int rv_membus_check_hart1_launch(rv_membus_state_t *bus, uint32_t *entry, uint32_t *sp, uint32_t *arg);

#endif /* RV_MEMBUS_H */
