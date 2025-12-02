#include <stdio.h>
#include <string.h>
#include "emulator.h"

// Standard RP2040 UART0 Data Register
#define UART0_BASE 0x40034000
#define UART0_DR   (UART0_BASE + 0x000)

void mem_write32(uint32_t addr, uint32_t val) {
    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        // Write to RAM (Little Endian)
        uint32_t offset = addr - RAM_BASE;
        memcpy(&cpu.ram[offset], &val, 4);
    } 
    else if (addr == UART0_DR) {
        // Intercept UART output -> Print to host console
        putchar((char)val);
        fflush(stdout);
    }
    // Ignore writes to Flash (Read-Only) or unmapped regions
}

uint32_t mem_read32(uint32_t addr) {
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val; 
    } 
    else if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        uint32_t val;
        memcpy(&val, &cpu.ram[offset], 4);
        return val;
    }
    return 0; // Default return for unmapped memory
}
