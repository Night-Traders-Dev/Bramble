/*
 * RP2350 Memory Bus for RISC-V Hazard3
 *
 * Routes memory accesses for the RISC-V execution path.
 * RP2350-specific regions (520KB SRAM, CLINT, 32KB ROM) are handled here.
 * Shared peripherals (UART, SPI, I2C, GPIO, etc.) fall through to the
 * existing RP2040 membus functions since addresses are identical.
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/rv_membus.h"
#include "rp2350_rv/rp2350_memmap.h"
#include "emulator.h"

/* ========================================================================
 * Initialization
 * ======================================================================== */

void rv_membus_init(rv_membus_state_t *bus, uint8_t *flash, uint32_t flash_size,
                    uint32_t cycles_per_us) {
    memset(bus->sram, 0, RV_SRAM_SIZE);
    memset(bus->rom, 0, sizeof(bus->rom));
    bus->flash = flash;
    bus->flash_size = flash_size;
    bus->rom_size = 32 * 1024;  /* 32KB ROM for RP2350 */
    rv_clint_init(&bus->clint, cycles_per_us);
}

/* ========================================================================
 * 32-bit Access
 * ======================================================================== */

uint32_t rv_mem_read32(rv_membus_state_t *bus, uint32_t addr) {
    uint32_t val;

    /* SRAM: 0x20000000 - 0x20082000 (520KB) */
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        uint32_t off = addr - RP2350_SRAM_BASE;
        memcpy(&val, &bus->sram[off], 4);
        return val;
    }

    /* SRAM alias: 0x21000000 → 0x20000000 */
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        uint32_t off = addr - RP2350_SRAM_ALIAS_BASE;
        memcpy(&val, &bus->sram[off], 4);
        return val;
    }

    /* ROM: 0x00000000 - 0x00007FFF (32KB) */
    if (addr < bus->rom_size) {
        memcpy(&val, &bus->rom[addr], 4);
        return val;
    }

    /* Flash: 0x10000000+ */
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) {
        uint32_t off = addr - RP2350_FLASH_BASE;
        memcpy(&val, &bus->flash[off], 4);
        return val;
    }

    /* XIP aliases */
    if (addr >= RP2350_XIP_NOALLOC_BASE && addr < RP2350_XIP_NOALLOC_BASE + bus->flash_size) {
        uint32_t off = addr - RP2350_XIP_NOALLOC_BASE;
        memcpy(&val, &bus->flash[off], 4);
        return val;
    }
    if (addr >= RP2350_XIP_NOCACHE_BASE && addr < RP2350_XIP_NOCACHE_BASE + bus->flash_size) {
        uint32_t off = addr - RP2350_XIP_NOCACHE_BASE;
        memcpy(&val, &bus->flash[off], 4);
        return val;
    }
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC && addr < RP2350_XIP_NOCACHE_NOALLOC + bus->flash_size) {
        uint32_t off = addr - RP2350_XIP_NOCACHE_NOALLOC;
        memcpy(&val, &bus->flash[off], 4);
        return val;
    }

    /* CLINT registers (in SIO space) */
    if (rv_clint_match(addr)) {
        return rv_clint_read(&bus->clint, addr - RV_CLINT_BASE);
    }

    /* Fall through to shared RP2040 peripheral bus for everything else.
     * This handles UART, SPI, I2C, GPIO, Timer, PWM, DMA, PIO, USB, etc.
     * since their base addresses are the same on RP2350. */
    return mem_read32(addr);
}

void rv_mem_write32(rv_membus_state_t *bus, uint32_t addr, uint32_t val) {
    /* SRAM */
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        uint32_t off = addr - RP2350_SRAM_BASE;
        memcpy(&bus->sram[off], &val, 4);
        return;
    }

    /* SRAM alias */
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        uint32_t off = addr - RP2350_SRAM_ALIAS_BASE;
        memcpy(&bus->sram[off], &val, 4);
        return;
    }

    /* ROM is read-only */
    if (addr < bus->rom_size)
        return;

    /* Flash is read-only at runtime */
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size)
        return;
    if (addr >= RP2350_XIP_NOALLOC_BASE && addr < RP2350_XIP_NOALLOC_BASE + bus->flash_size)
        return;
    if (addr >= RP2350_XIP_NOCACHE_BASE && addr < RP2350_XIP_NOCACHE_BASE + bus->flash_size)
        return;
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC && addr < RP2350_XIP_NOCACHE_NOALLOC + bus->flash_size)
        return;

    /* CLINT registers */
    if (rv_clint_match(addr)) {
        rv_clint_write(&bus->clint, addr - RV_CLINT_BASE, val);
        return;
    }

    /* Fall through to shared peripheral bus */
    mem_write32(addr, val);
}

/* ========================================================================
 * 16-bit Access
 * ======================================================================== */

uint16_t rv_mem_read16(rv_membus_state_t *bus, uint32_t addr) {
    uint16_t val;

    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        memcpy(&val, &bus->sram[addr - RP2350_SRAM_BASE], 2);
        return val;
    }
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        memcpy(&val, &bus->sram[addr - RP2350_SRAM_ALIAS_BASE], 2);
        return val;
    }
    if (addr < bus->rom_size) {
        memcpy(&val, &bus->rom[addr], 2);
        return val;
    }
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_FLASH_BASE], 2);
        return val;
    }
    if (addr >= RP2350_XIP_NOALLOC_BASE && addr < RP2350_XIP_NOALLOC_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOALLOC_BASE], 2);
        return val;
    }
    if (addr >= RP2350_XIP_NOCACHE_BASE && addr < RP2350_XIP_NOCACHE_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOCACHE_BASE], 2);
        return val;
    }
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC && addr < RP2350_XIP_NOCACHE_NOALLOC + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOCACHE_NOALLOC], 2);
        return val;
    }

    return mem_read16(addr);
}

void rv_mem_write16(rv_membus_state_t *bus, uint32_t addr, uint16_t val) {
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        memcpy(&bus->sram[addr - RP2350_SRAM_BASE], &val, 2);
        return;
    }
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        memcpy(&bus->sram[addr - RP2350_SRAM_ALIAS_BASE], &val, 2);
        return;
    }
    if (addr < bus->rom_size) return;
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) return;

    mem_write16(addr, val);
}

/* ========================================================================
 * 8-bit Access
 * ======================================================================== */

uint8_t rv_mem_read8(rv_membus_state_t *bus, uint32_t addr) {
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END)
        return bus->sram[addr - RP2350_SRAM_BASE];
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE)
        return bus->sram[addr - RP2350_SRAM_ALIAS_BASE];
    if (addr < bus->rom_size)
        return bus->rom[addr];
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size)
        return bus->flash[addr - RP2350_FLASH_BASE];
    if (addr >= RP2350_XIP_NOALLOC_BASE && addr < RP2350_XIP_NOALLOC_BASE + bus->flash_size)
        return bus->flash[addr - RP2350_XIP_NOALLOC_BASE];
    if (addr >= RP2350_XIP_NOCACHE_BASE && addr < RP2350_XIP_NOCACHE_BASE + bus->flash_size)
        return bus->flash[addr - RP2350_XIP_NOCACHE_BASE];
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC && addr < RP2350_XIP_NOCACHE_NOALLOC + bus->flash_size)
        return bus->flash[addr - RP2350_XIP_NOCACHE_NOALLOC];

    return mem_read8(addr);
}

void rv_mem_write8(rv_membus_state_t *bus, uint32_t addr, uint8_t val) {
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        bus->sram[addr - RP2350_SRAM_BASE] = val;
        return;
    }
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        bus->sram[addr - RP2350_SRAM_ALIAS_BASE] = val;
        return;
    }
    if (addr < bus->rom_size) return;
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) return;

    mem_write8(addr, val);
}
