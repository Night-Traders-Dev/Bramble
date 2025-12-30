#include <stdio.h>
#include <string.h>
#include "emulator.h"
#include "gpio.h"
#include "timer.h"
#include "nvic.h"

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

    /* NVIC registers (0xE000E000 - 0xE000EFFF) */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        nvic_write_register(addr, val);
        return;
    }

    /* Timer registers */
    if (addr >= TIMER_BASE && addr < TIMER_BASE + 0x50) {
        timer_write32(addr, val);
        printf("[MEMBUS] Timer read at 0x%08X\n", addr);  /* ADD THIS DEBUG LINE */
        return;
    }

    /* GPIO registers - UPDATED to include all PADSBANK0 alias regions */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||  /* FIXED: Extended range */
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
//        printf("[GPIO_WRITE] addr=0x%08X val=0x%08X\n", addr, val);
        gpio_write32(addr, val);
        return;
    }

    /* Stub out other peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;

    /* Any other region is currently treated as unmapped/no-op. */
}

void mem_write16(uint32_t addr, uint16_t val) {
    /* Writes to XIP flash are ignored: in real hardware this is external QSPI flash. */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t offset = addr - RAM_BASE;
        memcpy(&cpu.ram[offset], &val, 2);
        return;
    }

    /* NVIC registers - align to 32-bit boundary for subword access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t current = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        uint32_t mask = 0xFFFF << (offset * 8);
        uint32_t new_val = (current & ~mask) | ((uint32_t)val << (offset * 8));
        nvic_write_register(addr32, new_val);
        return;
    }

    /* GPIO - UPDATED to include all PADSBANK0 alias regions */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||  /* FIXED: Extended range */
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        gpio_write32(addr & ~0x3, val);  /* Align to 32-bit boundary */
        return;
    }

    /* Stub out peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;

    /* Any other region is currently treated as unmapped/no-op. */
}

void mem_write8(uint32_t addr, uint8_t val) {
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;  /* Flash writes ignored */
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        cpu.ram[addr - RAM_BASE] = val;
        return;
    }

    /* NVIC registers - align to 32-bit boundary for byte access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t current = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        uint32_t mask = 0xFF << (offset * 8);
        uint32_t new_val = (current & ~mask) | ((uint32_t)val << (offset * 8));
        nvic_write_register(addr32, new_val);
        return;
    }

    /* GPIO - UPDATED to include all PADSBANK0 alias regions */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||  /* FIXED: Extended range */
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        gpio_write32(addr & ~0x3, val);  /* Align to 32-bit boundary */
        return;
    }

    /* Stub out peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;
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

    /* NVIC registers (0xE000E000 - 0xE000EFFF) */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        return nvic_read_register(addr);
    }

    if (addr >= TIMER_BASE && addr < TIMER_BASE + 0x50) {
        return timer_read32(addr);
    }

    /* GPIO registers - UPDATED to include all PADSBANK0 alias regions */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||  /* FIXED: Extended range */
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        return gpio_read32(addr);
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

    /* NVIC registers - align to 32-bit boundary for subword access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t val32 = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        return (uint16_t)((val32 >> (offset * 8)) & 0xFFFF);
    }

    /* GPIO - UPDATED to include all PADSBANK0 alias regions */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||  /* FIXED: Extended range */
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        uint32_t val32 = gpio_read32(addr & ~0x3);
        return (uint16_t)(val32 & 0xFFFF);
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

    /* NVIC registers - align to 32-bit boundary for byte access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t val32 = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        return (uint8_t)((val32 >> (offset * 8)) & 0xFF);
    }

    /* GPIO - UPDATED to include all PADSBANK0 alias regions */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||  /* FIXED: Extended range */
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        uint32_t val32 = gpio_read32(addr & ~0x3);
        uint8_t byte_offset = addr & 0x3;
        return (uint8_t)((val32 >> (byte_offset * 8)) & 0xFF);
    }

    return 0xFF;  /* Unmapped reads return 0xFF */
}

void mem_write32_dual(int core_id, uint32_t addr, uint32_t val) {
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        printf("[MEM] WARNING: Attempt to write to flash at 0x%08X (read-only)\n", addr);
        return;
    }
    
    if (core_id == CORE0 && addr >= CORE0_RAM_START && addr < CORE0_RAM_END) {
        uint32_t offset = addr - CORE0_RAM_START;
        memcpy(&cores[CORE0].ram[offset], &val, 4);
        return;
    }
    
    if (core_id == CORE1 && addr >= CORE1_RAM_START && addr < CORE1_RAM_END) {
        uint32_t offset = addr - CORE1_RAM_START;
        memcpy(&cores[CORE1].ram[offset], &val, 4);
        return;
    }
    
    if (addr >= SHARED_RAM_BASE && addr < SHARED_RAM_BASE + SHARED_RAM_SIZE) {
        uint32_t offset = (addr - SHARED_RAM_BASE) / 4;
        if (offset < (SHARED_RAM_SIZE / 4)) {
            shared_ram[offset] = val;
        }
        return;
    }
}

uint16_t mem_read16_dual(int core_id, uint32_t addr) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    if (addr & 2) {
        return (word >> 16) & 0xFFFF;
    } else {
        return word & 0xFFFF;
    }
}

void mem_write16_dual(int core_id, uint32_t addr, uint16_t val) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    if (addr & 2) {
        word = (word & 0xFFFF) | ((uint32_t)val << 16);
    } else {
        word = (word & 0xFFFF0000) | val;
    }
    mem_write32_dual(core_id, addr & ~3, word);
}

uint8_t mem_read8_dual(int core_id, uint32_t addr) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    return (word >> ((addr & 3) * 8)) & 0xFF;
}

void mem_write8_dual(int core_id, uint32_t addr, uint8_t val) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    uint32_t shift = (addr & 3) * 8;
    word = (word & ~(0xFF << shift)) | ((uint32_t)val << shift);
    mem_write32_dual(core_id, addr & ~3, word);
}

uint32_t mem_read32_dual(int core_id, uint32_t addr) {
    /* Flash is shared across all cores */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint32_t val = 0;
        memcpy(&val, &cores[0].flash[offset], 4);
        return val;
    }
    
    /* Per-core RAM regions */
    if (core_id == CORE0 && addr >= CORE0_RAM_START && addr < CORE0_RAM_END) {
        uint32_t offset = addr - CORE0_RAM_START;
        uint32_t val = 0;
        memcpy(&val, &cores[CORE0].ram[offset], 4);
        return val;
    }
    
    if (core_id == CORE1 && addr >= CORE1_RAM_START && addr < CORE1_RAM_END) {
        uint32_t offset = addr - CORE1_RAM_START;
        uint32_t val = 0;
        memcpy(&val, &cores[CORE1].ram[offset], 4);
        return val;
    }
    
    /* Shared RAM (accessible by both cores) */
    if (addr >= SHARED_RAM_BASE && addr < SHARED_RAM_BASE + SHARED_RAM_SIZE) {
        uint32_t offset = (addr - SHARED_RAM_BASE) / 4;
        if (offset < (SHARED_RAM_SIZE / 4)) {
            return shared_ram[offset];
        }
    }
    
    return 0;  /* Out of bounds */
}
