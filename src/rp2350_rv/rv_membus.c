/*
 * RP2350 Memory Bus for RISC-V Hazard3
 *
 * Routes memory accesses for the RISC-V execution path.
 * RP2350-specific regions (520KB SRAM, CLINT, RP2350 peripherals, SIO)
 * are handled here. Shared peripherals fall through to the RP2040 membus.
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/rv_membus.h"
#include "rp2350_rv/rp2350_memmap.h"
#include "emulator.h"

#define RV_SHARED_RP2040_SYSCFG_BASE     0x40004000u
#define RV_SHARED_RP2040_CLOCKS_BASE     0x40008000u
#define RV_SHARED_RP2040_PSM_BASE        0x40010000u
#define RV_SHARED_RP2040_RESETS_BASE     0x4000C000u
#define RV_SHARED_RP2040_IO_BANK0_BASE   0x40014000u
#define RV_SHARED_RP2040_IO_QSPI_BASE    0x40018000u
#define RV_SHARED_RP2040_PADS_BANK0_BASE 0x4001C000u
#define RV_SHARED_RP2040_PADS_QSPI_BASE  0x40020000u
#define RV_SHARED_RP2040_XOSC_BASE       0x40024000u
#define RV_SHARED_RP2040_PLL_SYS_BASE    0x40028000u
#define RV_SHARED_RP2040_PLL_USB_BASE    0x4002C000u
#define RV_SHARED_RP2040_BUSCTRL_BASE    0x40030000u
#define RV_SHARED_RP2040_UART0_BASE      0x40034000u
#define RV_SHARED_RP2040_UART1_BASE      0x40038000u
#define RV_SHARED_RP2040_SPI0_BASE       0x4003C000u
#define RV_SHARED_RP2040_SPI1_BASE       0x40040000u
#define RV_SHARED_RP2040_I2C0_BASE       0x40044000u
#define RV_SHARED_RP2040_I2C1_BASE       0x40048000u
#define RV_SHARED_RP2040_ADC_BASE        0x4004C000u
#define RV_SHARED_RP2040_PWM_BASE        0x40050000u
#define RV_SHARED_RP2040_TIMER0_BASE     0x40054000u
#define RV_SHARED_RP2040_WATCHDOG_BASE   0x40058000u
#define RV_SHARED_RP2040_ROSC_BASE       0x40060000u
#define RV_SHARED_RP2040_TBMAN_BASE      0x4006C000u

static uint32_t rv_translate_shared_addr(uint32_t addr) {
    uint32_t block = addr & ~0x3FFFu;
    uint32_t tail = addr & 0x3FFFu;

    switch (block) {
    case RP2350_SYSCFG_BASE:      return RV_SHARED_RP2040_SYSCFG_BASE | tail;
    case RP2350_CLOCKS_BASE:      return RV_SHARED_RP2040_CLOCKS_BASE | tail;
    case RP2350_PSM_BASE:         return RV_SHARED_RP2040_PSM_BASE | tail;
    case RP2350_RESETS_BASE:      return RV_SHARED_RP2040_RESETS_BASE | tail;
    case RP2350_IO_BANK0_BASE:    return RV_SHARED_RP2040_IO_BANK0_BASE | tail;
    case RP2350_IO_QSPI_BASE:     return RV_SHARED_RP2040_IO_QSPI_BASE | tail;
    case RP2350_PADS_BANK0_BASE:  return RV_SHARED_RP2040_PADS_BANK0_BASE | tail;
    case RP2350_PADS_QSPI_BASE:   return RV_SHARED_RP2040_PADS_QSPI_BASE | tail;
    case RP2350_XOSC_BASE:        return RV_SHARED_RP2040_XOSC_BASE | tail;
    case RP2350_PLL_SYS_BASE:     return RV_SHARED_RP2040_PLL_SYS_BASE | tail;
    case RP2350_PLL_USB_BASE:     return RV_SHARED_RP2040_PLL_USB_BASE | tail;
    case RP2350_BUSCTRL_BASE:     return RV_SHARED_RP2040_BUSCTRL_BASE | tail;
    case RP2350_UART0_BASE:       return RV_SHARED_RP2040_UART0_BASE | tail;
    case RP2350_UART1_BASE:       return RV_SHARED_RP2040_UART1_BASE | tail;
    case RP2350_SPI0_BASE:        return RV_SHARED_RP2040_SPI0_BASE | tail;
    case RP2350_SPI1_BASE:        return RV_SHARED_RP2040_SPI1_BASE | tail;
    case RP2350_I2C0_BASE:        return RV_SHARED_RP2040_I2C0_BASE | tail;
    case RP2350_I2C1_BASE:        return RV_SHARED_RP2040_I2C1_BASE | tail;
    case RP2350_ADC_BASE:         return RV_SHARED_RP2040_ADC_BASE | tail;
    case RP2350_PWM_BASE:         return RV_SHARED_RP2040_PWM_BASE | tail;
    case RP2350_TIMER0_BASE:      return RV_SHARED_RP2040_TIMER0_BASE | tail;
    case RP2350_WATCHDOG_BASE:    return RV_SHARED_RP2040_WATCHDOG_BASE | tail;
    case RP2350_ROSC_BASE:        return RV_SHARED_RP2040_ROSC_BASE | tail;
    case RP2350_TBMAN_BASE:       return RV_SHARED_RP2040_TBMAN_BASE | tail;
    default:
        return addr;
    }
}

/* ========================================================================
 * Initialization
 * ======================================================================== */

void rv_membus_init(rv_membus_state_t *bus, uint8_t *flash, uint32_t flash_size,
                    uint32_t cycles_per_us) {
    memset(bus->sram, 0, RV_SRAM_SIZE);
    memset(bus->rom, 0, sizeof(bus->rom));
    bus->flash = flash;
    bus->flash_size = flash_size;
    bus->rom_size = 32 * 1024;
    bus->is_riscv = 1;
    bus->hart1_launch_pending = 0;
    bus->hart1_count = 0;
    bus->gpio_hi_in = 0x3E;  /* CS high + data pulled up (same as RP2040 QSPI) */
    rv_clint_init(&bus->clint, cycles_per_us);
    rp2350_periph_init(&bus->periph, 0);
}

/* ========================================================================
 * Hart 1 Launch Check
 * ======================================================================== */

int rv_membus_check_hart1_launch(rv_membus_state_t *bus, uint32_t *entry,
                                  uint32_t *sp, uint32_t *arg) {
    if (!bus->hart1_launch_pending) return 0;
    bus->hart1_launch_pending = 0;
    *entry = bus->hart1_entry;
    *sp = bus->hart1_sp;
    *arg = bus->hart1_arg;
    return 1;
}

/* ========================================================================
 * RP2350 SIO Handler (0xD0000000)
 * Different from RP2040 SIO: CPUID returns a hart-dependent value, and
 * GPIO_HI (pins 32-47 plus the QSPI/USB IOs) is interleaved with the low bank
 * at +0x14/+0x1c/+0x24/+0x2c and +0x34..+0x4c.
 *
 * Note SIO +0x100..+0x17c is SPINLOCK0..31 on both chips and is deliberately
 * NOT handled here -- it falls through to the shared membus, which owns the
 * spinlock state. The machine timer lives at +0x1a0..+0x1bc (see rv_clint.h).
 * ======================================================================== */

static uint32_t rv_sio_read(rv_membus_state_t *bus, uint32_t offset) {
    switch (offset) {
    case RV_SIO_CPUID:
        /* RP2350 CPUID (datasheet 3.1.2) returns the *hart id*: 0 when read by
         * core 0, 1 when read by core 1. A fixed 0x2 is not just a different
         * value -- hart 1's boot sequence branches on this register to decide
         * whether it is the secondary core, so a constant makes every hart
         * identify as core 0.
         *
         * The bus cannot see which hart issued the access from its arguments,
         * but the CPU engine already publishes the current hart for exactly
         * this reason (RISCV_SOFTIRQ and MTIMECMP are per-hart at one address).
         */
        return (uint32_t)(rv_clint_current_hart() & 1);

    /* GPIO low (pins 0-31) — fall through to RP2040 */
    case 0x04: /* GPIO_IN */
    case 0x08: /* GPIO_HI_IN */
    case 0x10: /* GPIO_OUT */
    case 0x20: /* GPIO_OE */
        break;  /* Will fall through */

    /* GPIO high (pins 32-47) — RP2350-specific */
    /* GPIO_HI_OUT/OE and their SET/CLR/XOR aliases live interleaved with the
     * low bank on RP2350 (datasheet Table 17). */
    case 0x014: return bus->gpio_hi_out;      /* GPIO_HI_OUT */
    case 0x01c: return bus->gpio_hi_out;      /* GPIO_HI_OUT_SET reads as the value */
    case 0x024: return bus->gpio_hi_out;      /* GPIO_HI_OUT_CLR */
    case 0x02c: return bus->gpio_hi_out;      /* GPIO_HI_OUT_XOR */
    case 0x034: return bus->gpio_hi_oe;       /* GPIO_HI_OE */
    case 0x03c: return bus->gpio_hi_oe;       /* GPIO_HI_OE_SET */
    case 0x044: return bus->gpio_hi_oe;       /* GPIO_HI_OE_CLR */
    case 0x04c: return bus->gpio_hi_oe;       /* GPIO_HI_OE_XOR */

    default: break;
    }
    /* Fall through for standard SIO registers */
    return 0xDEAD0000 | offset;  /* Marker for unhandled — will be overridden by fallthrough */
}

static int rv_sio_write(rv_membus_state_t *bus, uint32_t offset, uint32_t val) {
    switch (offset) {
    /* GPIO high bank (pins 32-47, QSPI and USB IO). Interleaved with the low
     * bank per datasheet Table 17 -- the previous table began at 0x30, one
     * register too high, so a firmware write to GPIO_OE (0x30) set the HIGH
     * bank's output latch instead of enabling outputs. */
    case 0x014: bus->gpio_hi_out = val & 0xFFFF;  return 1; /* GPIO_HI_OUT */
    case 0x01c: bus->gpio_hi_out |= (val & 0xFFFF); return 1; /* GPIO_HI_OUT_SET */
    case 0x024: bus->gpio_hi_out &= ~(val & 0xFFFF); return 1; /* GPIO_HI_OUT_CLR */
    case 0x02c: bus->gpio_hi_out ^= (val & 0xFFFF); return 1; /* GPIO_HI_OUT_XOR */
    case 0x034: bus->gpio_hi_oe = val & 0xFFFF;   return 1; /* GPIO_HI_OE */
    case 0x03c: bus->gpio_hi_oe |= (val & 0xFFFF); return 1; /* GPIO_HI_OE_SET */
    case 0x044: bus->gpio_hi_oe &= ~(val & 0xFFFF); return 1; /* GPIO_HI_OE_CLR */
    case 0x04c: bus->gpio_hi_oe ^= (val & 0xFFFF); return 1; /* GPIO_HI_OE_XOR */

    /* Hart 1 launch: the datasheet section 5.3 protocol. Core 0 pushes six
     * words to core 1 over the SIO FIFO:
     *
     *     { 0, 0, 1, vector_table, sp, entry }
     *
     * so the launch is recognised by that shape rather than by a magic
     * register write. This used to be claimed at 0x1C0-0x1CC, which the
     * datasheet says is the TMDS encoder. */
    case RV_SIO_FIFO_WR: {
        /* The write still has to reach core 1's FIFO, so let the shared SIO
         * model handle it and only observe the value here. */
        if (bus->hart1_count < 6) {
            bus->hart1_words[bus->hart1_count++] = val;
            if (bus->hart1_count == 6) {
                uint32_t *w = bus->hart1_words;
                /* Shape check: {0, 0, 1, ...}. The real protocol also requires
                 * core 1 to echo each word, which the shared FIFO model does
                 * not track, so require the three constant words at least. */
                if (w[0] == 0 && w[1] == 0 && w[2] == 1) {
                    bus->hart1_arg    = w[3];   /* vector_table / VTOR */
                    bus->hart1_sp     = w[4];
                    bus->hart1_entry  = w[5] & ~1u;  /* drop the Thumb bit */
                    bus->hart1_launch_pending = 1;
                    fprintf(stderr,
                            "[RV-SIO] Hart 1 launch: entry=0x%08X SP=0x%08X vtor=0x%08X\n",
                            bus->hart1_entry, bus->hart1_sp, bus->hart1_arg);
                } else {
                    fprintf(stderr, "[RV-SIO] FIFO sequence did not match the "
                                    "core-1 launch protocol; hart 1 stays halted\n");
                }
                bus->hart1_count = 0;
            }
        }
        return 0;   /* fall through so the shared FIFO model also stores it */
    }

    default: break;
    }
    return 0;  /* Not handled — fall through to RP2040 SIO */
}

/* ========================================================================
 * 32-bit Access
 * ======================================================================== */

uint32_t rv_mem_read32(rv_membus_state_t *bus, uint32_t addr) {
    uint32_t val;

    /* SRAM: 0x20000000 - 0x20082000 (520KB) */
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        memcpy(&val, &bus->sram[addr - RP2350_SRAM_BASE], 4);
        return val;
    }

    /* SRAM alias: 0x21000000 */
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        memcpy(&val, &bus->sram[addr - RP2350_SRAM_ALIAS_BASE], 4);
        return val;
    }

    /* ROM: 0x00000000 - 0x00007FFF (32KB) */
    if (addr < bus->rom_size) {
        memcpy(&val, &bus->rom[addr], 4);
        return val;
    }

    /* Flash: 0x10000000+ */
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_FLASH_BASE], 4);
        return val;
    }

    /* XIP aliases */
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC_BASE && addr < RP2350_XIP_NOCACHE_NOALLOC_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOCACHE_NOALLOC_BASE], 4);
        return val;
    }

    /* CLINT registers (in SIO space) */
    if (rv_clint_match(addr))
        return rv_clint_read(&bus->clint, addr - RV_CLINT_BASE);

    /* RP2350-specific peripherals */
    if (rp2350_periph_match(addr))
        return rp2350_periph_read32(&bus->periph, addr);

    /* SIO: handle RP2350-specific registers, fall through for standard */
    if (addr >= RP2350_SIO_BASE && addr < RP2350_SIO_BASE + 0x200) {
        uint32_t offset = addr - RP2350_SIO_BASE;
        uint32_t sio_val = rv_sio_read(bus, offset);
        /* CPUID and the GPIO_HI aliases are RV-specific, so answer them here.
         *
         * The FIFO offsets must NOT be listed: they are ordinary SIO
         * registers owned by the shared model, and returning early would make
         * a read of FIFO_RD answer with a marker value instead of the actual
         * received word. Firmware polling its mailbox would then spin. The
         * launch protocol only needs to *observe* FIFO writes, which
         * rv_sio_write() does without consuming them. */
        if (offset == RV_SIO_CPUID || offset == 0x30 || offset == 0x38 ||
            offset == 0x40)
            return sio_val;
        /* Otherwise fall through to RP2040 SIO */
    }

    /* Fall through to shared RP2040 peripheral bus. The flag tells the
     * shared matchers these are already-translated RP2040 addresses. */
    membus_rv_delegate = 1;
    uint32_t v = mem_read32(rv_translate_shared_addr(addr));
    membus_rv_delegate = 0;
    return v;
}

void rv_mem_write32(rv_membus_state_t *bus, uint32_t addr, uint32_t val) {
    /* SRAM */
    if (addr >= RP2350_SRAM_BASE && addr < RP2350_SRAM_END) {
        memcpy(&bus->sram[addr - RP2350_SRAM_BASE], &val, 4);
        return;
    }
    if (addr >= RP2350_SRAM_ALIAS_BASE && addr < RP2350_SRAM_ALIAS_BASE + RV_SRAM_SIZE) {
        memcpy(&bus->sram[addr - RP2350_SRAM_ALIAS_BASE], &val, 4);
        return;
    }

    /* ROM and flash are read-only */
    if (addr < bus->rom_size) return;
    if (addr >= RP2350_FLASH_BASE && addr < RP2350_FLASH_BASE + bus->flash_size) return;
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC_BASE && addr < RP2350_XIP_NOCACHE_NOALLOC_BASE + bus->flash_size) return;

    /* CLINT */
    if (rv_clint_match(addr)) {
        rv_clint_write(&bus->clint, addr - RV_CLINT_BASE, val);
        return;
    }

    /* RP2350-specific peripherals */
    if (rp2350_periph_match(addr)) {
        rp2350_periph_write32(&bus->periph, addr, val);
        return;
    }

    /* SIO: try RP2350-specific first */
    if (addr >= RP2350_SIO_BASE && addr < RP2350_SIO_BASE + 0x200) {
        uint32_t offset = addr - RP2350_SIO_BASE;
        if (rv_sio_write(bus, offset, val))
            return;
        /* Fall through to RP2040 SIO */
    }

    /* Fall through to shared peripheral bus */
    membus_rv_delegate = 1;
    mem_write32(rv_translate_shared_addr(addr), val);
    membus_rv_delegate = 0;
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
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC_BASE && addr < RP2350_XIP_NOCACHE_NOALLOC_BASE + bus->flash_size) {
        memcpy(&val, &bus->flash[addr - RP2350_XIP_NOCACHE_NOALLOC_BASE], 2);
        return val;
    }
    membus_rv_delegate = 1;
    uint16_t v16 = mem_read16(rv_translate_shared_addr(addr));
    membus_rv_delegate = 0;
    return v16;
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
    membus_rv_delegate = 1;
    mem_write16(rv_translate_shared_addr(addr), val);
    membus_rv_delegate = 0;
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
    if (addr >= RP2350_XIP_NOCACHE_NOALLOC_BASE && addr < RP2350_XIP_NOCACHE_NOALLOC_BASE + bus->flash_size)
        return bus->flash[addr - RP2350_XIP_NOCACHE_NOALLOC_BASE];
    /* RP2350 peripherals byte access */
    if (rp2350_periph_match(addr))
        return rp2350_periph_read8(&bus->periph, addr);
    membus_rv_delegate = 1;
    uint8_t v8 = mem_read8(rv_translate_shared_addr(addr));
    membus_rv_delegate = 0;
    return v8;
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
    /* RP2350 peripherals byte access */
    if (rp2350_periph_match(addr)) {
        rp2350_periph_write8(&bus->periph, addr, val);
        return;
    }
    membus_rv_delegate = 1;
    mem_write8(rv_translate_shared_addr(addr), val);
    membus_rv_delegate = 0;
}
