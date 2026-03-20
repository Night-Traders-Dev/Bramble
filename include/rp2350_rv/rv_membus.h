/*
 * RP2350 Memory Bus for RISC-V Hazard3
 *
 * Thin layer over the shared membus that handles RP2350-specific differences:
 *   - 520KB SRAM (vs 264KB on RP2040)
 *   - CLINT registers in SIO space
 *   - RP2350 SRAM layout (10 banks x 64KB + 8KB scratch)
 *   - 4MB default flash (vs 2MB)
 *   - TICKS peripheral (0x40108000)
 *
 * Most peripherals share the same addresses as RP2040, so reads/writes
 * to UART, SPI, I2C, GPIO, etc. route through the existing membus.
 */

#ifndef RV_MEMBUS_H
#define RV_MEMBUS_H

#include <stdint.h>
#include "rp2350_rv/rv_clint.h"
#include "rp2350_rv/rv_cpu.h"

/* RP2350 SRAM: 520KB (10 banks of 64KB + 8KB scratch) */
#define RV_SRAM_SIZE    (520 * 1024)

typedef struct {
    /* RP2350 SRAM (520KB, separate from RP2040 RAM) */
    uint8_t sram[RV_SRAM_SIZE];

    /* CLINT interrupt controller */
    rv_clint_state_t clint;

    /* Pointer to flash (shared with main emulator) */
    uint8_t *flash;
    uint32_t flash_size;

    /* ROM (32KB for RP2350) */
    uint8_t rom[32 * 1024];
    uint32_t rom_size;
} rv_membus_state_t;

/* Initialize RP2350 memory bus */
void rv_membus_init(rv_membus_state_t *bus, uint8_t *flash, uint32_t flash_size, uint32_t cycles_per_us);

/* Memory access (used by rv_cpu_step via function pointers or direct calls) */
uint32_t rv_mem_read32(rv_membus_state_t *bus, uint32_t addr);
void rv_mem_write32(rv_membus_state_t *bus, uint32_t addr, uint32_t val);
uint16_t rv_mem_read16(rv_membus_state_t *bus, uint32_t addr);
void rv_mem_write16(rv_membus_state_t *bus, uint32_t addr, uint16_t val);
uint8_t rv_mem_read8(rv_membus_state_t *bus, uint32_t addr);
void rv_mem_write8(rv_membus_state_t *bus, uint32_t addr, uint8_t val);

#endif /* RV_MEMBUS_H */
