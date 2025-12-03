#include <stdio.h>
#include <string.h>
#include "emulator.h"

void mem_write32(uint32_t addr, uint32_t val) {
    /* Writes to XIP flash are ignored: in real hardware this is external QSPI flash. */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        memcpy(&cpu.ram[offset], &val, 4);
        return;
    }

    if (addr == UART0_DR) {
        putchar((char)(val & 0xFF));
        fflush(stdout);
        return;
    }

    /* Stub out other peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;

    /* Any other region is currently treated as unmapped/no-op. */
}

uint32_t mem_read32(uint32_t addr) {
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        uint32_t val;
        memcpy(&val, &cpu.ram[offset], 4);
        return val;
    }

    if (addr == UART0_FR) {
        /* PL011-style UARTFR reset: TXFE=1 (bit 7), RXFE=1 (bit 4), others 0. */
        return 0x00000090;
    }

    /* Stub peripheral reads: return 0 for now. */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000)   return 0x00000000;
    if (addr >= 0x40000000   && addr < 0x50000000)          return 0x00000000;

    /* Unmapped address space -> 0 (or you could model a bus fault in the CPU core). */
    return 0;
}

uint16_t mem_read16(uint32_t addr) {
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint16_t val;
        memcpy(&val, &cpu.flash[offset], 2);
        return val;
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        uint16_t val;
        memcpy(&val, &cpu.ram[offset], 2);
        return val;
    }

    /* No 16-bit peripheral emulation yet. */
    return 0;
}


uint8_t mem_read8(uint32_t addr) {
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return cpu.flash[addr - FLASH_BASE];
    }
    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        return cpu.ram[addr - RAM_BASE];
    }
    return 0xFF;  /* Unmapped reads return 0xFF */
}

void mem_write8(uint32_t addr, uint8_t val) {
    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        cpu.ram[addr - RAM_BASE] = val;
    }
}
