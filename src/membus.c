#include <stdio.h>
#include <string.h>
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

/* ========================================================================
 * XIP Cache Control State (0x14000000)
 *
 * Stub implementation: cache is always "ready" and "empty".
 * Real RP2040 has 16KB XIP cache that caches flash reads.
 * ======================================================================== */

#define XIP_CTRL_OFFSET    0x00  /* Cache enable (bit 3=POWER_DOWN, bit 1=ERR_BADWRITE, bit 0=EN) */
#define XIP_FLUSH_OFFSET   0x04  /* Write 1 to flush (strobe, self-clearing) */
#define XIP_STAT_OFFSET    0x08  /* bit 2=FIFO_EMPTY, bit 1=FLUSH_READY */
#define XIP_CTR_HIT_OFFSET 0x0C  /* Cache hit counter */
#define XIP_CTR_ACC_OFFSET 0x10  /* Cache access counter */
#define XIP_STREAM_ADDR    0x14  /* Stream address */
#define XIP_STREAM_CTR     0x18  /* Stream counter */
#define XIP_STREAM_FIFO    0x1C  /* Stream FIFO (read-only) */

static uint32_t xip_ctrl_reg = 0x00000003;  /* EN=1, ERR_BADWRITE=1 (RP2040 default) */
static uint32_t xip_ctr_hit = 0;
static uint32_t xip_ctr_acc = 0;
static uint32_t xip_stream_addr = 0;
static uint32_t xip_stream_ctr = 0;

/* XIP SRAM: 16KB cache memory usable as SRAM */
static uint8_t xip_sram[XIP_SRAM_SIZE];

static uint32_t xip_ctrl_read(uint32_t offset) {
    switch (offset) {
    case XIP_CTRL_OFFSET:    return xip_ctrl_reg;
    case XIP_FLUSH_OFFSET:   return 0;  /* Always reads as 0 (strobe register) */
    case XIP_STAT_OFFSET:    return (1u << 2) | (1u << 1);  /* FIFO_EMPTY=1, FLUSH_READY=1 */
    case XIP_CTR_HIT_OFFSET: return xip_ctr_hit;
    case XIP_CTR_ACC_OFFSET: return xip_ctr_acc;
    case XIP_STREAM_ADDR:    return xip_stream_addr;
    case XIP_STREAM_CTR:     return xip_stream_ctr;
    case XIP_STREAM_FIFO:    return 0;  /* No stream data */
    default: return 0;
    }
}

static void xip_ctrl_write(uint32_t offset, uint32_t val) {
    switch (offset) {
    case XIP_CTRL_OFFSET:    xip_ctrl_reg = val & 0x0B; break;  /* EN, ERR_BADWRITE, POWER_DOWN */
    case XIP_FLUSH_OFFSET:   break;  /* Strobe: accept and ignore (no actual cache) */
    case XIP_CTR_HIT_OFFSET: xip_ctr_hit = val; break;
    case XIP_CTR_ACC_OFFSET: xip_ctr_acc = val; break;
    case XIP_STREAM_ADDR:    xip_stream_addr = val; break;
    case XIP_STREAM_CTR:     xip_stream_ctr = val; break;
    default: break;
    }
}

/* ========================================================================
 * SRAM Alias Translation
 *
 * RP2040 mirrors SRAM at 0x21000000 (same as 0x20000000).
 * Translate alias addresses to canonical SRAM addresses.
 * ======================================================================== */

static inline uint32_t sram_alias_translate(uint32_t addr) {
    if (addr >= SRAM_ALIAS_BASE && addr < SRAM_ALIAS_BASE + RAM_SIZE) {
        return addr - SRAM_ALIAS_BASE + RAM_BASE;
    }
    return addr;
}

/* ========================================================================
 * Active RAM pointer for zero-copy dual-core context switching
 *
 * In single-core mode: points to cpu.ram (full 264KB)
 * In dual-core mode: points to cores[active_core].ram (per-core 132KB)
 * ======================================================================== */

static uint8_t *active_ram = NULL;
static uint32_t active_ram_base = RAM_BASE;
static uint32_t active_ram_size = RAM_SIZE;

void mem_set_ram_ptr(uint8_t *ram, uint32_t base, uint32_t size) {
    active_ram = ram;
    active_ram_base = base;
    active_ram_size = size;
}

/* Lazy init: ensure active_ram is set before first use */
static inline uint8_t *get_ram(void) {
    if (!active_ram) active_ram = cpu.ram;
    return active_ram;
}

/* ========================================================================
 * Clock-Domain Peripheral Address Check
 *
 * RP2040 peripherals have atomic register aliases at +0x1000/+0x2000/+0x3000.
 * Each 4KB peripheral occupies a 16KB block. We check the 16KB-aligned base.
 * ======================================================================== */

static int is_clocks_addr(uint32_t addr) {
    uint32_t base = addr & ~0x3FFF; /* 16KB-aligned base */
    return (base == RESETS_BASE  || base == CLOCKS_BASE ||
            base == XOSC_BASE   || base == PLL_SYS_BASE ||
            base == PLL_USB_BASE || base == WATCHDOG_BASE ||
            base == PSM_BASE);
}

static int is_adc_addr(uint32_t addr) {
    return (addr & ~0x3FFF) == ADC_BASE;
}

/* ========================================================================
 * Single-Core Memory Access Functions
 * ======================================================================== */

void mem_write32(uint32_t addr, uint32_t val) {
    /* SRAM alias translation (0x21xxxxxx -> 0x20xxxxxx) */
    addr = sram_alias_translate(addr);

    /* Writes to XIP flash (and uncached aliases) are ignored */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;
    }
    if (addr >= XIP_NOALLOC_BASE && addr < XIP_NOALLOC_BASE + FLASH_SIZE) return;
    if (addr >= XIP_NOCACHE_BASE && addr < XIP_NOCACHE_BASE + FLASH_SIZE) return;
    if (addr >= XIP_NOCACHE_NOALLOC && addr < XIP_NOCACHE_NOALLOC + FLASH_SIZE) return;

    /* XIP cache control registers */
    if (addr >= XIP_CTRL_BASE && addr < XIP_CTRL_BASE + 0x20) {
        xip_ctrl_write(addr - XIP_CTRL_BASE, val);
        return;
    }

    /* XIP SRAM (cache as SRAM) */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        uint32_t offset = addr - XIP_SRAM_BASE;
        memcpy(&xip_sram[offset], &val, 4);
        return;
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        uint32_t offset = addr - active_ram_base;
        memcpy(&get_ram()[offset], &val, 4);
        return;
    }

    /* UART registers (including atomic aliases) */
    {
        int uart_num = uart_match(addr);
        if (uart_num >= 0) {
            uint32_t alias = addr & 0x3000;
            uint32_t offset = (addr & 0xFFF);
            if (alias == 0x0000) {
                uart_write32(uart_num, offset, val);
            } else if (alias == 0x2000) {  /* SET */
                uint32_t cur = uart_read32(uart_num, offset);
                uart_write32(uart_num, offset, cur | val);
            } else if (alias == 0x3000) {  /* CLR */
                uint32_t cur = uart_read32(uart_num, offset);
                uart_write32(uart_num, offset, cur & ~val);
            } else {  /* XOR 0x1000 */
                uint32_t cur = uart_read32(uart_num, offset);
                uart_write32(uart_num, offset, cur ^ val);
            }
            return;
        }
    }

    /* NVIC registers (0xE000E000 - 0xE000EFFF) */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        nvic_write_register(addr, val);
        return;
    }

    /* Timer registers */
    if (addr >= TIMER_BASE && addr < TIMER_BASE + 0x50) {
        timer_write32(addr, val);
        return;
    }

    /* GPIO registers - UPDATED to include all PADSBANK0 alias regions */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        gpio_write32(addr, val);
        return;
    }

    /* Clock-domain peripherals (Resets, Clocks, XOSC, PLLs, Watchdog) */
    if (is_clocks_addr(addr)) {
        clocks_write32(addr, val);
        return;
    }

    /* ADC */
    if (is_adc_addr(addr)) {
        adc_write32(addr, val);
        return;
    }

    /* SPI peripherals */
    {
        int spi_num = spi_match(addr);
        if (spi_num >= 0) {
            uint32_t alias = addr & 0x3000;
            uint32_t off = addr & 0xFFF;
            if (alias == 0x0000) {
                spi_write32(spi_num, off, val);
            } else if (alias == 0x2000) {
                spi_write32(spi_num, off, spi_read32(spi_num, off) | val);
            } else if (alias == 0x3000) {
                spi_write32(spi_num, off, spi_read32(spi_num, off) & ~val);
            } else {
                spi_write32(spi_num, off, spi_read32(spi_num, off) ^ val);
            }
            return;
        }
    }

    /* I2C peripherals */
    {
        int i2c_num = i2c_match(addr);
        if (i2c_num >= 0) {
            uint32_t alias = addr & 0x3000;
            uint32_t off = addr & 0xFFF;
            if (alias == 0x0000) {
                i2c_write32(i2c_num, off, val);
            } else if (alias == 0x2000) {
                i2c_write32(i2c_num, off, i2c_read32(i2c_num, off) | val);
            } else if (alias == 0x3000) {
                i2c_write32(i2c_num, off, i2c_read32(i2c_num, off) & ~val);
            } else {
                i2c_write32(i2c_num, off, i2c_read32(i2c_num, off) ^ val);
            }
            return;
        }
    }

    /* PWM */
    if (pwm_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t off = addr & 0xFFF;
        if (alias == 0x0000) {
            pwm_write32(off, val);
        } else if (alias == 0x2000) {
            pwm_write32(off, pwm_read32(off) | val);
        } else if (alias == 0x3000) {
            pwm_write32(off, pwm_read32(off) & ~val);
        } else {
            pwm_write32(off, pwm_read32(off) ^ val);
        }
        return;
    }

    /* DMA controller */
    if (dma_match(addr)) {
        uint32_t alias = addr & 0x3000;
        uint32_t off = addr & 0xFFF;
        if (alias == 0x0000) {
            dma_write32(off, val);
        } else if (alias == 0x2000) {
            dma_write32(off, dma_read32(off) | val);
        } else if (alias == 0x3000) {
            dma_write32(off, dma_read32(off) & ~val);
        } else {
            dma_write32(off, dma_read32(off) ^ val);
        }
        return;
    }

    /* PIO */
    {
        int pio_num = pio_match(addr);
        if (pio_num >= 0) {
            uint32_t alias = addr & 0x3000;
            uint32_t off = addr & 0xFFF;
            if (alias == 0x0000) {
                pio_write32(pio_num, off, val);
            } else if (alias == 0x2000) {
                pio_write32(pio_num, off, pio_read32(pio_num, off) | val);
            } else if (alias == 0x3000) {
                pio_write32(pio_num, off, pio_read32(pio_num, off) & ~val);
            } else {
                pio_write32(pio_num, off, pio_read32(pio_num, off) ^ val);
            }
            return;
        }
    }

    /* USB controller - accept writes silently (no emulation) */
    if ((addr >= USBCTRL_DPRAM_BASE && addr < USBCTRL_DPRAM_BASE + 0x1000) ||
        (addr >= USBCTRL_REGS_BASE && addr < USBCTRL_REGS_BASE + 0x1000)) {
        return;
    }

    /* Stub out other peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;

    /* Any other region is currently treated as unmapped/no-op. */
}

void mem_write16(uint32_t addr, uint16_t val) {
    addr = sram_alias_translate(addr);

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        uint32_t offset = addr - XIP_SRAM_BASE;
        memcpy(&xip_sram[offset], &val, 2);
        return;
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        uint32_t offset = addr - active_ram_base;
        memcpy(&get_ram()[offset], &val, 2);
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

    /* GPIO */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        gpio_write32(addr & ~0x3, val);
        return;
    }

    /* Stub out peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;
}

void mem_write8(uint32_t addr, uint8_t val) {
    addr = sram_alias_translate(addr);

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;  /* Flash writes ignored */
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        xip_sram[addr - XIP_SRAM_BASE] = val;
        return;
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        get_ram()[addr - active_ram_base] = val;
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

    /* GPIO */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        gpio_write32(addr & ~0x3, val);
        return;
    }

    /* Stub out peripheral writes for now. */
    if (addr >= 0x40000000 && addr < 0x50000000) return;   /* APB/AHB peripherals */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000) return;
}

uint32_t mem_read32(uint32_t addr) {
    /* SRAM alias translation */
    addr = sram_alias_translate(addr);

    /* ROM (0x00000000 - 0x00000FFF) */
    if (addr < ROM_SIZE) {
        return rom_read32(addr);
    }

    /* XIP flash (and uncached aliases read from same backing store) */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }
    if (addr >= XIP_NOALLOC_BASE && addr < XIP_NOALLOC_BASE + FLASH_SIZE) {
        uint32_t offset = addr - XIP_NOALLOC_BASE;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }
    if (addr >= XIP_NOCACHE_BASE && addr < XIP_NOCACHE_BASE + FLASH_SIZE) {
        uint32_t offset = addr - XIP_NOCACHE_BASE;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }
    if (addr >= XIP_NOCACHE_NOALLOC && addr < XIP_NOCACHE_NOALLOC + FLASH_SIZE) {
        uint32_t offset = addr - XIP_NOCACHE_NOALLOC;
        uint32_t val;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }

    /* XIP cache control registers */
    if (addr >= XIP_CTRL_BASE && addr < XIP_CTRL_BASE + 0x20) {
        return xip_ctrl_read(addr - XIP_CTRL_BASE);
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        uint32_t offset = addr - XIP_SRAM_BASE;
        uint32_t val;
        memcpy(&val, &xip_sram[offset], 4);
        return val;
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        uint32_t offset = addr - active_ram_base;
        uint32_t val;
        memcpy(&val, &get_ram()[offset], 4);
        return val;
    }

    /* UART registers (including atomic aliases) */
    {
        int uart_num = uart_match(addr);
        if (uart_num >= 0) {
            uint32_t offset = addr & 0xFFF;
            return uart_read32(uart_num, offset);
        }
    }

    /* NVIC registers (0xE000E000 - 0xE000EFFF) */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        return nvic_read_register(addr);
    }

    if (addr >= TIMER_BASE && addr < TIMER_BASE + 0x50) {
        return timer_read32(addr);
    }

    /* GPIO registers */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        return gpio_read32(addr);
    }

    /* Clock-domain peripherals (Resets, Clocks, XOSC, PLLs, Watchdog) */
    if (is_clocks_addr(addr)) {
        return clocks_read32(addr);
    }

    /* ADC */
    if (is_adc_addr(addr)) {
        return adc_read32(addr);
    }

    /* SPI peripherals */
    {
        int spi_num = spi_match(addr);
        if (spi_num >= 0) {
            return spi_read32(spi_num, addr & 0xFFF);
        }
    }

    /* I2C peripherals */
    {
        int i2c_num = i2c_match(addr);
        if (i2c_num >= 0) {
            return i2c_read32(i2c_num, addr & 0xFFF);
        }
    }

    /* PWM */
    if (pwm_match(addr)) {
        return pwm_read32(addr & 0xFFF);
    }

    /* DMA controller */
    if (dma_match(addr)) {
        return dma_read32(addr & 0xFFF);
    }

    /* PIO */
    {
        int pio_num = pio_match(addr);
        if (pio_num >= 0) {
            return pio_read32(pio_num, addr & 0xFFF);
        }
    }

    /* USB controller stub - return "disconnected" state.
     * SIE_STATUS (0x50110050): all zeros = no VBUS, not connected.
     * SDK's stdio_usb_init() times out after 500ms and falls back to UART. */
    if ((addr >= USBCTRL_DPRAM_BASE && addr < USBCTRL_DPRAM_BASE + 0x1000) ||
        (addr >= USBCTRL_REGS_BASE && addr < USBCTRL_REGS_BASE + 0x1000)) {
        return 0;
    }

    /* Stub peripheral reads: return 0 for now. */
    if (addr >= SIO_BASE     && addr < SIO_BASE + 0x1000)   return 0x00000000;
    if (addr >= 0x40000000   && addr < 0x50000000)          return 0x00000000;

    /* Unmapped address space -> 0 */
    return 0;
}

uint16_t mem_read16(uint32_t addr) {
    addr = sram_alias_translate(addr);

    /* ROM */
    if (addr < ROM_SIZE) {
        return rom_read16(addr);
    }

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint16_t val;
        memcpy(&val, &cpu.flash[offset], 2);
        return val;
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        uint32_t offset = addr - XIP_SRAM_BASE;
        uint16_t val;
        memcpy(&val, &xip_sram[offset], 2);
        return val;
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        uint32_t offset = addr - active_ram_base;
        uint16_t val;
        memcpy(&val, &get_ram()[offset], 2);
        return val;
    }

    /* NVIC registers - align to 32-bit boundary for subword access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t val32 = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        return (uint16_t)((val32 >> (offset * 8)) & 0xFFFF);
    }

    /* GPIO */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        uint32_t val32 = gpio_read32(addr & ~0x3);
        return (uint16_t)(val32 & 0xFFFF);
    }

    /* No 16-bit peripheral emulation yet. */
    return 0;
}

uint8_t mem_read8(uint32_t addr) {
    addr = sram_alias_translate(addr);

    /* ROM */
    if (addr < ROM_SIZE) {
        return rom_read8(addr);
    }

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return cpu.flash[addr - FLASH_BASE];
    }

    /* XIP SRAM */
    if (addr >= XIP_SRAM_BASE && addr < XIP_SRAM_BASE + XIP_SRAM_SIZE) {
        return xip_sram[addr - XIP_SRAM_BASE];
    }

    if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
        return get_ram()[addr - active_ram_base];
    }

    /* NVIC registers - align to 32-bit boundary for byte access */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t val32 = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        return (uint8_t)((val32 >> (offset * 8)) & 0xFF);
    }

    /* GPIO */
    if ((addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) ||
        (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) ||
        (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100)) {
        uint32_t val32 = gpio_read32(addr & ~0x3);
        uint8_t byte_offset = addr & 0x3;
        return (uint8_t)((val32 >> (byte_offset * 8)) & 0xFF);
    }

    return 0xFF;  /* Unmapped reads return 0xFF */
}

/* ========================================================================
 * Dual-Core Memory Access Functions
 * ======================================================================== */

void mem_write32_dual(int core_id, uint32_t addr, uint32_t val) {
    addr = sram_alias_translate(addr);

    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        return;  /* Flash writes ignored */
    }

    /* Shared RAM checked FIRST to resolve overlap with Core 1 per-core range */
    if (addr >= SHARED_RAM_BASE && addr < SHARED_RAM_BASE + SHARED_RAM_SIZE) {
        uint32_t offset = (addr - SHARED_RAM_BASE) / 4;
        if (offset < (SHARED_RAM_SIZE / 4)) {
            shared_ram[offset] = val;
        }
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

    /* Route peripheral writes through single-core memory bus */
    mem_write32(addr, val);
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
    addr = sram_alias_translate(addr);

    /* ROM */
    if (addr < ROM_SIZE) {
        return rom_read32(addr);
    }

    /* Flash is shared across all cores (stored in cpu.flash) */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint32_t val = 0;
        memcpy(&val, &cpu.flash[offset], 4);
        return val;
    }

    /* Shared RAM checked FIRST to resolve overlap with Core 1 per-core range */
    if (addr >= SHARED_RAM_BASE && addr < SHARED_RAM_BASE + SHARED_RAM_SIZE) {
        uint32_t offset = (addr - SHARED_RAM_BASE) / 4;
        if (offset < (SHARED_RAM_SIZE / 4)) {
            return shared_ram[offset];
        }
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

    /* Route peripheral reads through single-core memory bus */
    return mem_read32(addr);
}
