#include <stdio.h>
#include <string.h>
#include "emulator.h"

/* SIO (Single-cycle I/O) Base */
#define SIO_BASE   0xD0000000

void mem_write32(uint32_t addr, uint32_t val) {
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
    
    if (addr >= 0x40000000 && addr < 0x50000000) return;
    if (addr >= SIO_BASE && addr < SIO_BASE + 0x1000) return;
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) return;
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
    
    if (addr == UART0_FR) return 0x00000000;
    if (addr >= SIO_BASE && addr < SIO_BASE + 0x1000) return 0x00000000;
    if (addr >= 0x40000000 && addr < 0x50000000) return 0x00000000;

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

    return 0;
}
