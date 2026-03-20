/*
 * RP2350 RISC-V Bootrom
 *
 * Minimal bootrom for Hazard3 RISC-V cores.
 * Sets up initial stack pointer and provides a small ROM with
 * jump-to-flash trampoline and basic function stubs.
 */

#ifndef RV_BOOTROM_H
#define RV_BOOTROM_H

#include <stdint.h>

/* Populate the RP2350 ROM buffer with a minimal RISC-V bootrom.
 * Sets up:
 *   - Reset vector at 0x00000000 (jump to flash)
 *   - Stack pointer initialization
 *   - Basic ROM function table stubs
 * Returns the entry PC for the CPU (usually 0x00000000). */
uint32_t rv_bootrom_init(uint8_t *rom, uint32_t rom_size,
                         uint32_t flash_base, uint32_t sram_end);

#endif /* RV_BOOTROM_H */
