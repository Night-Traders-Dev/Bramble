#include <stdio.h>
#include <string.h>
#include "clocks.h"
#include "emulator.h"

/* Global clock-domain peripheral state */
clocks_state_t clocks_state;

/* Initialize all clock-domain peripherals */
void clocks_init(void) {
    clocks_reset();
}

/* Reset to power-on defaults */
void clocks_reset(void) {
    memset(&clocks_state, 0, sizeof(clocks_state_t));

    /* RP2040 boots with all peripherals held in reset */
    clocks_state.reset = RESETS_ALL_MASK;

    /* Clock dividers default to 1:0 (no division) */
    for (int i = 0; i < NUM_CLOCK_GENERATORS; i++) {
        clocks_state.clk_div[i] = (1u << 8); /* Integer part = 1 */
    }

    /* XOSC defaults */
    clocks_state.xosc_startup = 0x00C4; /* Default startup delay */

    /* PLL defaults - powered down at reset */
    clocks_state.pll_sys.pwr = 0x0000002D; /* PD=1, VCOPD=1, POSTDIVPD=1 */
    clocks_state.pll_usb.pwr = 0x0000002D;
    clocks_state.pll_sys.fbdiv = 0;
    clocks_state.pll_usb.fbdiv = 0;

    /* Watchdog tick disabled at reset */
    clocks_state.wdog_tick = 0;

    /* PSM defaults: everything powered on, nothing watchdog-selected */
    clocks_state.psm_frce_on = 0;
    clocks_state.psm_frce_off = 0;
    clocks_state.psm_wdsel = 0;
}

/* ========================================================================
 * RP2040 Atomic Register Aliases
 *
 * Each peripheral's 4KB register space is mirrored 4 times in a 16KB block:
 *   +0x0000: Normal read/write
 *   +0x1000: XOR (write XORs with current value)
 *   +0x2000: SET (write ORs bits into current value)
 *   +0x3000: CLR (write clears bits from current value)
 *
 * This function applies the alias operation to a register value.
 * ======================================================================== */
static uint32_t apply_alias_write(uint32_t current, uint32_t val, uint32_t alias) {
    switch (alias) {
        case 0: return val;              /* Normal write */
        case 1: return current ^ val;    /* XOR */
        case 2: return current | val;    /* SET */
        case 3: return current & ~val;   /* CLR */
        default: return val;
    }
}

/* ========================================================================
 * Resets Peripheral
 * ======================================================================== */

static uint32_t resets_read(uint32_t addr) {
    uint32_t offset = addr & 0xFFF;
    switch (offset) {
        case 0x00: /* RESET */
            return clocks_state.reset;
        case 0x04: /* WDSEL */
            return clocks_state.wdsel;
        case 0x08: /* RESET_DONE */
            /* A peripheral is "done" when NOT held in reset */
            return (~clocks_state.reset) & RESETS_ALL_MASK;
        default:
            return 0;
    }
}

static void resets_write(uint32_t addr, uint32_t val, uint32_t alias) {
    uint32_t offset = addr & 0xFFF;
    switch (offset) {
        case 0x00: /* RESET */
            clocks_state.reset = apply_alias_write(
                clocks_state.reset, val, alias) & RESETS_ALL_MASK;
            break;
        case 0x04: /* WDSEL */
            clocks_state.wdsel = apply_alias_write(
                clocks_state.wdsel, val, alias) & RESETS_ALL_MASK;
            break;
        default:
            break;
    }
}

/* ========================================================================
 * Clocks Peripheral
 * ======================================================================== */

static uint32_t clocks_domain_read(uint32_t addr) {
    uint32_t offset = addr & 0xFFF;

    /* Clock generator registers: 10 generators, stride 0x0C each */
    if (offset < NUM_CLOCK_GENERATORS * 0x0C) {
        uint32_t gen = offset / 0x0C;
        uint32_t reg = offset % 0x0C;
        switch (reg) {
            case CLK_CTRL_OFFSET:
                return clocks_state.clk_ctrl[gen];
            case CLK_DIV_OFFSET:
                return clocks_state.clk_div[gen];
            case CLK_SELECTED_OFFSET:
                if (gen == CLK_REF) {
                    return 1u << (clocks_state.clk_ctrl[gen] & 0x3u);
                }
                if (gen == CLK_SYS) {
                    return 1u << (clocks_state.clk_ctrl[gen] & 0x1u);
                }
                /* Non-glitchless clocks are hardwired to selected=1. */
                return 0x1;
            default:
                return 0;
        }
    }

    /* Additional clocks registers */
    switch (offset) {
        case 0x78: /* CLK_SYS_RESUS_CTRL */
            return 0;
        case 0x7C: /* CLK_SYS_RESUS_STATUS */
            return 0; /* No resuscitation */
        case 0x98: /* FC0_STATUS */
            return (1u << 4); /* DONE=1 */
        case 0x9C: /* FC0_RESULT */
            return (125000u << 5); /* 125 MHz as KHz in [29:5] */
        default:
            return 0;
    }
}

static void clocks_domain_write(uint32_t addr, uint32_t val, uint32_t alias) {
    uint32_t offset = addr & 0xFFF;

    /* Clock generator registers */
    if (offset < NUM_CLOCK_GENERATORS * 0x0C) {
        uint32_t gen = offset / 0x0C;
        uint32_t reg = offset % 0x0C;
        switch (reg) {
            case CLK_CTRL_OFFSET:
                clocks_state.clk_ctrl[gen] = apply_alias_write(
                    clocks_state.clk_ctrl[gen], val, alias);
                break;
            case CLK_DIV_OFFSET:
                clocks_state.clk_div[gen] = apply_alias_write(
                    clocks_state.clk_div[gen], val, alias);
                break;
            default:
                break;
        }
    }
    /* Other writes silently accepted */
}

/* ========================================================================
 * XOSC
 * ======================================================================== */

static uint32_t xosc_read(uint32_t addr) {
    uint32_t offset = addr & 0xFFF;
    switch (offset) {
        case 0x00: /* CTRL */
            return clocks_state.xosc_ctrl;
        case 0x04: /* STATUS */
            /* Always report STABLE and ENABLED */
            return XOSC_STATUS_STABLE | XOSC_STATUS_ENABLED;
        case 0x0C: /* STARTUP */
            return clocks_state.xosc_startup;
        case 0x1C: /* COUNT */
            return clocks_state.xosc_count;
        default:
            return 0;
    }
}

static void xosc_write(uint32_t addr, uint32_t val, uint32_t alias) {
    uint32_t offset = addr & 0xFFF;
    switch (offset) {
        case 0x00: /* CTRL */
            clocks_state.xosc_ctrl = apply_alias_write(
                clocks_state.xosc_ctrl, val, alias);
            break;
        case 0x0C: /* STARTUP */
            clocks_state.xosc_startup = apply_alias_write(
                clocks_state.xosc_startup, val, alias);
            break;
        case 0x1C: /* COUNT */
            clocks_state.xosc_count = apply_alias_write(
                clocks_state.xosc_count, val, alias);
            break;
        default:
            break;
    }
}

/* ========================================================================
 * PLL (shared between PLL_SYS and PLL_USB)
 * ======================================================================== */

static uint32_t pll_read(pll_state_t *pll, uint32_t offset) {
    switch (offset) {
        case PLL_CS_OFFSET:
            /* Always report LOCK=1 */
            return pll->cs | PLL_CS_LOCK;
        case PLL_PWR_OFFSET:
            return pll->pwr;
        case PLL_FBDIV_INT_OFFSET:
            return pll->fbdiv;
        case PLL_PRIM_OFFSET:
            return pll->prim;
        default:
            return 0;
    }
}

static void pll_write(pll_state_t *pll, uint32_t offset, uint32_t val,
                       uint32_t alias) {
    switch (offset) {
        case PLL_CS_OFFSET:
            pll->cs = apply_alias_write(pll->cs, val, alias);
            break;
        case PLL_PWR_OFFSET:
            pll->pwr = apply_alias_write(pll->pwr, val, alias);
            break;
        case PLL_FBDIV_INT_OFFSET:
            pll->fbdiv = apply_alias_write(pll->fbdiv, val, alias);
            break;
        case PLL_PRIM_OFFSET:
            pll->prim = apply_alias_write(pll->prim, val, alias);
            break;
        default:
            break;
    }
}

/* ========================================================================
 * Watchdog
 * ======================================================================== */

static uint32_t watchdog_read(uint32_t addr) {
    uint32_t offset = addr & 0xFFF;
    switch (offset) {
        case 0x00: /* CTRL */
            return clocks_state.wdog_ctrl;
        case 0x04: /* LOAD - write-only */
            return 0;
        case 0x08: /* REASON */
            return 0; /* Clean boot - no watchdog reset */
        case 0x2C: /* TICK */
            /* Return stored value with RUNNING bit set if ENABLE is set */
            if (clocks_state.wdog_tick & WATCHDOG_TICK_ENABLE) {
                return clocks_state.wdog_tick | WATCHDOG_TICK_RUNNING;
            }
            return clocks_state.wdog_tick;
        default:
            /* Scratch registers at offset 0x0C - 0x28 */
            if (offset >= 0x0C && offset <= 0x28 && (offset & 0x3) == 0) {
                uint32_t idx = (offset - 0x0C) / 4;
                if (idx < WATCHDOG_NUM_SCRATCH)
                    return clocks_state.wdog_scratch[idx];
            }
            return 0;
    }
}

static void watchdog_write(uint32_t addr, uint32_t val, uint32_t alias) {
    uint32_t offset = addr & 0xFFF;
    switch (offset) {
        case 0x00: /* CTRL */
            clocks_state.wdog_ctrl = apply_alias_write(
                clocks_state.wdog_ctrl, val, alias);
            break;
        case 0x04: /* LOAD */
            clocks_state.wdog_load = val; /* Reload value, always direct write */
            break;
        case 0x2C: /* TICK */
            clocks_state.wdog_tick = apply_alias_write(
                clocks_state.wdog_tick, val, alias);
            break;
        default:
            /* Scratch registers */
            if (offset >= 0x0C && offset <= 0x28 && (offset & 0x3) == 0) {
                uint32_t idx = (offset - 0x0C) / 4;
                if (idx < WATCHDOG_NUM_SCRATCH) {
                    clocks_state.wdog_scratch[idx] = apply_alias_write(
                        clocks_state.wdog_scratch[idx], val, alias);
                }
            }
            break;
    }
}

/* ========================================================================
 * PSM (Power State Machine)
 * ======================================================================== */

static uint32_t psm_read(uint32_t addr) {
    uint32_t offset = addr & 0xFFF;

    switch (offset) {
        case PSM_FRCE_ON_OFFSET:
            return clocks_state.psm_frce_on;
        case PSM_FRCE_OFF_OFFSET:
            return clocks_state.psm_frce_off;
        case PSM_WDSEL_OFFSET:
            return clocks_state.psm_wdsel;
        case PSM_DONE_OFFSET:
            return (~clocks_state.psm_frce_off) & PSM_ALL_MASK;
        default:
            return 0;
    }
}

static void psm_write(uint32_t addr, uint32_t val, uint32_t alias) {
    uint32_t offset = addr & 0xFFF;
    uint32_t prev_proc1 = clocks_state.psm_frce_off & PSM_FRCE_OFF_PROC1_BITS;

    switch (offset) {
        case PSM_FRCE_ON_OFFSET:
            clocks_state.psm_frce_on = apply_alias_write(
                clocks_state.psm_frce_on, val, alias) & PSM_ALL_MASK;
            break;
        case PSM_FRCE_OFF_OFFSET:
            clocks_state.psm_frce_off = apply_alias_write(
                clocks_state.psm_frce_off, val, alias) & PSM_ALL_MASK;
            if ((clocks_state.psm_frce_off & PSM_FRCE_OFF_PROC1_BITS) != prev_proc1) {
                sio_set_core1_reset((clocks_state.psm_frce_off & PSM_FRCE_OFF_PROC1_BITS) != 0);
            }
            break;
        case PSM_WDSEL_OFFSET:
            clocks_state.psm_wdsel = apply_alias_write(
                clocks_state.psm_wdsel, val, alias) & PSM_ALL_MASK;
            break;
        default:
            break;
    }
}

/* ========================================================================
 * Top-level dispatch (called from membus.c)
 * ======================================================================== */

uint32_t clocks_read32(uint32_t addr) {
    /* Reads always return the canonical register value regardless of alias */
    uint32_t base_aligned = addr & ~0x3FFF;
    uint32_t reg_offset = addr & 0xFFF;
    (void)reg_offset; /* Used implicitly by sub-readers via addr masking */

    /* Map to canonical address for reading */
    uint32_t canonical = base_aligned | (addr & 0xFFF);

    if (base_aligned == RESETS_BASE)
        return resets_read(canonical);
    if (base_aligned == CLOCKS_BASE)
        return clocks_domain_read(canonical);
    if (base_aligned == XOSC_BASE)
        return xosc_read(canonical);
    if (base_aligned == PLL_SYS_BASE)
        return pll_read(&clocks_state.pll_sys, canonical & 0xFFF);
    if (base_aligned == PLL_USB_BASE)
        return pll_read(&clocks_state.pll_usb, canonical & 0xFFF);
    if (base_aligned == WATCHDOG_BASE)
        return watchdog_read(canonical);
    if (base_aligned == PSM_BASE)
        return psm_read(canonical);

    return 0;
}

void clocks_write32(uint32_t addr, uint32_t val) {
    uint32_t base_aligned = addr & ~0x3FFF;
    uint32_t alias = (addr >> 12) & 0x3;
    uint32_t canonical = base_aligned | (addr & 0xFFF);

    if (base_aligned == RESETS_BASE)
        resets_write(canonical, val, alias);
    else if (base_aligned == CLOCKS_BASE)
        clocks_domain_write(canonical, val, alias);
    else if (base_aligned == XOSC_BASE)
        xosc_write(canonical, val, alias);
    else if (base_aligned == PLL_SYS_BASE)
        pll_write(&clocks_state.pll_sys, canonical & 0xFFF, val, alias);
    else if (base_aligned == PLL_USB_BASE)
        pll_write(&clocks_state.pll_usb, canonical & 0xFFF, val, alias);
    else if (base_aligned == WATCHDOG_BASE)
        watchdog_write(canonical, val, alias);
    else if (base_aligned == PSM_BASE)
        psm_write(canonical, val, alias);
}
