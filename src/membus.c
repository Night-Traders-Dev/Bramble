#include <stdio.h>
#include <string.h>
#include "emulator.h"

void mem_write32(uint32_t addr, uint32_t val) {
    /* Bounds check */
    if (addr + 4 > MEM_END) {
        printf("[MEMBUS] WARNING: Write out of bounds at 0x%08X\n", addr);
        return;
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        memcpy(&cpu.ram[offset], &val, 4);
    }
    else if (addr == UART0_DR) {
        /* UART Data Register - Echo to stdout */
        putchar((char)(val & 0xFF));
        fflush(stdout);
    }
    else if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        /* Flash is read-only in bootloader context */
        printf("[MEMBUS] WARNING: Attempted write to Flash at 0x%08X\n", addr);
    }
    /* Silently ignore other peripheral writes */
}

uint32_t mem_read32(uint32_t addr) {
    /* Bounds check */
    if (addr + 4 > MEM_END) {
        printf("[MEMBUS] WARNING: Read out of bounds at 0x%08X\n", addr);
        return 0;
    }

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

    /* Return 0 for unmapped memory */
    return 0;
}

uint16_t mem_read16(uint32_t addr) {
    /* For Thumb instruction fetch */
    if (addr + 2 > MEM_END) {
        printf("[MEMBUS] WARNING: Read16 out of bounds at 0x%08X\n", addr);
        return 0;
    }

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint16_t val;
        memcpy(&val, &cpu.flash[offset], 2);
        return val;
    }
    else if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        uint16_t val;
        memcpy(&val, &cpu.ram[offset], 2);
        return val;
    }

    return 0;
}
