#include <stdio.h>
#include <string.h>
#include "clocks.h"
#include "emulator.h"

/* Global clock-domain peripheral state */
clocks_state_t clocks_state;

/* Watchdog reboot flag - checked by main loop */
int watchdog_reboot_pending = 0;

static inline uint32_t resets_all_mask(void) {
    return membus_rp2350_mode ? RP2350_RESETS_ALL_MASK : RP2040_RESETS_ALL_MASK;
}

static inline uint32_t psm_all_mask(void) {
    return membus_rp2350_mode ? RP2350_PSM_ALL_MASK : RP2040_PSM_ALL_MASK;
}

static inline uint32_t psm_proc1_mask(void) {
    return membus_rp2350_mode ? RP2350_PSM_FRCE_OFF_PROC1_BITS : RP2040_PSM_FRCE_OFF_PROC1_BITS;
}

/* Initialize all clock-domain peripherals */
/* RP2350 implements 8 clock generators (GPOUT0-3, REF, SYS, PERI, USB);
 * RP2040 implements 10, adding ADC and RTC. Firmware that reads DIV/SELECTED
 * for a generator the chip does not have must not see aliasing register data
 * from a neighbouring one. */
static uint32_t num_clock_generators(void) {
    return membus_rp2350_mode ? 8u : (uint32_t)NUM_CLOCK_GENERATORS;
}

void clocks_init(void) {
    clocks_reset();
}

/* Reset to power-on defaults */
void clocks_reset(void) {
    /* SCRATCH0-7 "persist through soft reset of the chip" (datasheet, WATCHDOG
     * register list). A blanket memset wiped them, so firmware using scratch to
     * carry state across a self-inflicted reset lost it. Preserve them, and the
     * reason register, across a soft reset. */
    uint32_t scratch[WATCHDOG_NUM_SCRATCH];
    uint32_t reason;
    memcpy(scratch, clocks_state.wdog_scratch, sizeof(scratch));
    reason = clocks_state.wdog_reason;

    memset(&clocks_state, 0, sizeof(clocks_state_t));

    memcpy(clocks_state.wdog_scratch, scratch, sizeof(scratch));
    clocks_state.wdog_reason = reason;

    /* RP2040 boots with all peripherals held in reset */
    clocks_state.reset = resets_all_mask();

    /* Clock dividers default to 1.0 (no division), but the register format
     * differs by chip: RP2040 CLK_DIV is 8.8 fixed point (reset 0x00000100)
     * while RP2350 is 16.16 (reset 0x00010000). Resetting an RP2350 divider to
     * 1<<8 gives an integer part of 0, i.e. divide by zero. */
    for (int i = 0; i < NUM_CLOCK_GENERATORS; i++) {
        clocks_state.clk_div[i] = membus_rp2350_mode ? 0x00010000u : 0x00000100u;
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
            return (~clocks_state.reset) & resets_all_mask();
        default:
            return 0;
    }
}

static void resets_write(uint32_t addr, uint32_t val, uint32_t alias) {
    uint32_t offset = addr & 0xFFF;
    switch (offset) {
        case 0x00: /* RESET */
            clocks_state.reset = apply_alias_write(
                clocks_state.reset, val, alias) & resets_all_mask();
            break;
        case 0x04: /* WDSEL */
            clocks_state.wdsel = apply_alias_write(
                clocks_state.wdsel, val, alias) & resets_all_mask();
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

    /* Clock generator registers, stride 0x0C each. RP2350 implements 8
     * generators, RP2040 10, so the addressable range differs by chip. */
    if (offset < num_clock_generators() * 0x0C) {
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

    /* Additional clocks registers.
     *
     * D3: the RP2350 map shifts everything from RESUS_CTRL up by four bytes --
     * 0x84 RESUS_CTRL, 0x88 RESUS_STATUS, 0x8C FC0_REF_KHZ ... 0xA0 FC0_SRC,
     * 0xA4 FC0_STATUS, 0xA8 FC0_RESULT. Decoding the RP2040 offsets on RP2350
     * meant FC0_STATUS read back 0 forever, so frequency_count_khz()'s
     * "wait for FC0_STATUS.DONE" loop never completed. */
    uint32_t fc0 = offset;
    if (membus_rp2350_mode && offset >= 0x84)
        fc0 = offset + 4; /* RP2350 shifts FC0 (and RESUS) up by one word */

    switch (fc0) {
        case 0x78: /* CLK_SYS_RESUS_CTRL (RP2040) */
            return 0;
        case 0x7C: /* CLK_SYS_RESUS_STATUS (RP2040) */
            return 0; /* No resuscitation */
        case 0x80: /* FC0_REF_KHZ */
        case 0x84: /* FC0_MIN_KHZ */
        case 0x88: /* FC0_MAX_KHZ */
        case 0x8C: /* FC0_DELAY */
        case 0x90: /* FC0_INTERVAL */
        case 0x94: /* FC0_SRC */
            return 0;
        case 0x98: /* FC0_STATUS */
            return (1u << 4); /* DONE=1 */
        case 0x9C: { /* FC0_RESULT */
            /*
             * Compute system frequency from PLL_SYS config:
             * freq_khz = (XOSC_KHZ * FBDIV) / (REFDIV * POSTDIV1 * POSTDIV2)
             * XOSC = 12MHz, REFDIV = CS[5:0], FBDIV = FBDIV_INT,
             * POSTDIV1 = PRIM[18:16], POSTDIV2 = PRIM[14:12]
             */
            uint32_t fbdiv = clocks_state.pll_sys.fbdiv;
            uint32_t refdiv = clocks_state.pll_sys.cs & 0x3F;
            uint32_t postdiv1 = (clocks_state.pll_sys.prim >> 16) & 0x07;
            uint32_t postdiv2 = (clocks_state.pll_sys.prim >> 12) & 0x07;
            if (refdiv == 0) refdiv = 1;
            if (postdiv1 == 0) postdiv1 = 1;
            if (postdiv2 == 0) postdiv2 = 1;
            uint32_t freq_khz;
            if (fbdiv > 0) {
                freq_khz = (12000u * fbdiv) / (refdiv * postdiv1 * postdiv2);
            } else {
                freq_khz = 125000u; /* Default 125MHz if PLL not configured */
            }
            return (freq_khz << 5); /* Result format: KHz in bits [29:5] */
        }
        default:
            return 0;
    }
}

static void clocks_domain_write(uint32_t addr, uint32_t val, uint32_t alias) {
    uint32_t offset = addr & 0xFFF;

    /* Clock generator registers (see num_clock_generators()). */
    if (offset < num_clock_generators() * 0x0C) {
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
        case 0x08: /* REASON
             * Datasheet: "Logs the reason for the last reset. Both bits are zero
             * for the case of a hardware reset." This previously returned a
             * hardcoded 0, so a reboot caused by the watchdog was
             * indistinguishable from a power-on reset and firmware could not
             * tell that its own timeout had fired. */
            return clocks_state.wdog_reason;
        case 0x2C: /* TICK -- RP2040 only.
             * RP2350's register list ends at SCRATCH7 (0x028); there is no TICK,
             * so reading 0x2C there must return 0 rather than a synthesised
             * value. */
            if (membus_rp2350_mode)
                return 0;
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
        case 0x00: /* CTRL */ {
            uint32_t new_ctrl = apply_alias_write(
                clocks_state.wdog_ctrl, val, alias);
            clocks_state.wdog_ctrl = new_ctrl;
            /* Bit 31 = TRIGGER: request system reboot */
            if (new_ctrl & (1u << 31)) {
                watchdog_reboot_pending = 1;
                /* Record why the next boot will see. */
                clocks_state.wdog_reason = WATCHDOG_REASON_WDOG;
            }
            break;
        }
        case 0x04: /* LOAD */
            clocks_state.wdog_load = val; /* Reload value, always direct write */
            break;
        case 0x2C: /* TICK -- RP2040 only, see the read side. */
            if (membus_rp2350_mode)
                break;
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
    uint32_t mask = psm_all_mask();

    switch (offset) {
        case PSM_FRCE_ON_OFFSET:
            return clocks_state.psm_frce_on;
        case PSM_FRCE_OFF_OFFSET:
            return clocks_state.psm_frce_off;
        case PSM_WDSEL_OFFSET:
            return clocks_state.psm_wdsel;
        case PSM_DONE_OFFSET:
            return (~clocks_state.psm_frce_off) & mask;
        default:
            return 0;
    }
}

static void psm_write(uint32_t addr, uint32_t val, uint32_t alias) {
    uint32_t offset = addr & 0xFFF;
    uint32_t mask = psm_all_mask();
    uint32_t proc1_mask = psm_proc1_mask();
    uint32_t prev_proc1 = clocks_state.psm_frce_off & proc1_mask;

    switch (offset) {
        case PSM_FRCE_ON_OFFSET:
            clocks_state.psm_frce_on = apply_alias_write(
                clocks_state.psm_frce_on, val, alias) & mask;
            break;
        case PSM_FRCE_OFF_OFFSET:
            clocks_state.psm_frce_off = apply_alias_write(
                clocks_state.psm_frce_off, val, alias) & mask;
            if ((clocks_state.psm_frce_off & proc1_mask) != prev_proc1) {
                sio_set_core1_reset((clocks_state.psm_frce_off & proc1_mask) != 0);
            }
            break;
        case PSM_WDSEL_OFFSET:
            clocks_state.psm_wdsel = apply_alias_write(
                clocks_state.psm_wdsel, val, alias) & mask;
            break;
        default:
            break;
    }
}

/* ========================================================================
 * ROSC (Ring Oscillator) - 0x40060000
 * ======================================================================== */

/* ROSC register identity. RP2040 and RP2350 do not agree on the offsets:
 *
 *            RP2040   RP2350
 *   CTRL       0x00     0x00
 *   FREQA      0x04     0x04
 *   FREQB      0x08     0x08
 *   COUNT      0x20     0x0C
 *   DIV        0x10     0x14
 *   PHASE      0x14     0x18
 *   STATUS     0x18     0x1C
 *   RANDOMBIT  0x1C     0x20
 *
 * This is not a monotone shift -- COUNT moves *down* while RANDOMBIT moves up
 * past where COUNT used to be -- so it has to be mapped per register, not by
 * adding a constant. */
/* Prefixed ROSC_R_ to avoid the ROSC_* address macros in clocks.h. */
enum { ROSC_R_CTRL, ROSC_R_FREQA, ROSC_R_FREQB, ROSC_R_COUNT,
       ROSC_R_DIV, ROSC_R_PHASE, ROSC_R_STATUS, ROSC_R_RANDOMBIT };

static int rosc_reg(uint32_t offset) {
    if (!membus_rp2350_mode) {
        switch (offset) {
        case 0x00: return ROSC_R_CTRL;
        case 0x04: return ROSC_R_FREQA;
        case 0x08: return ROSC_R_FREQB;
        case 0x10: return ROSC_R_DIV;
        case 0x14: return ROSC_R_PHASE;
        case 0x18: return ROSC_R_STATUS;
        case 0x1C: return ROSC_R_RANDOMBIT;
        case 0x20: return ROSC_R_COUNT;
        default: return -1;
        }
    }
    switch (offset) {
    case 0x00: return ROSC_R_CTRL;
    case 0x04: return ROSC_R_FREQA;
    case 0x08: return ROSC_R_FREQB;
    case 0x0C: return ROSC_R_COUNT;
    case 0x14: return ROSC_R_DIV;
    case 0x18: return ROSC_R_PHASE;
    case 0x1C: return ROSC_R_STATUS;
    case 0x20: return ROSC_R_RANDOMBIT;
    default: return -1;
    }
}

static uint32_t rosc_read(uint32_t addr) {
    switch (rosc_reg(addr & 0xFFF)) {
    case ROSC_R_CTRL:  return clocks_state.rosc_ctrl;
    case ROSC_R_FREQA: return clocks_state.rosc_freqa;
    case ROSC_R_FREQB: return clocks_state.rosc_freqb;
    case ROSC_R_COUNT: return clocks_state.rosc_count;
    case ROSC_R_DIV:   return clocks_state.rosc_div;
    case ROSC_R_PHASE: return clocks_state.rosc_phase;
    case ROSC_R_STATUS: {
        /* STATUS (datasheet 12.17): bit 31 STABLE, bit 21 FREQ_RUNNING,
         * bit 20 DIV_RUNNING, bit 2 DIV_FAIL, bit 1 BADWRITE, bit 0 ENABLED.
         *
         * This previously reported bit 24 as ENABLED (it is BADWRITE) and
         * hard-wired bit 12 -- DIV_RUNNING -- to 1 regardless of state, so
         * firmware that waited for DIV_RUNNING before trusting DIV, and that
         * gated clock output on ENABLED, saw constants. */
        uint32_t enabled = ((clocks_state.rosc_ctrl >> 12) & 0xFFF) == 0xFAB;
        uint32_t div_running = (clocks_state.rosc_div & 0xFF) != 0;
        uint32_t freq_running = ((clocks_state.rosc_freqa >> 12) & 0xFFF) != 0;
        return (1u << 31) | (freq_running << 21) | (div_running << 20) | enabled;
    }
    case ROSC_R_RANDOMBIT: {
        uint32_t s = clocks_state.rosc_random_state;
        if (s == 0) s = 0xDEADBEEF;
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        clocks_state.rosc_random_state = s;
        return s & 1;
    }
    default: return 0;
    }
}

static void rosc_write(uint32_t addr, uint32_t val, uint32_t alias) {
    switch (rosc_reg(addr & 0xFFF)) {
    case ROSC_R_CTRL:
        clocks_state.rosc_ctrl = apply_alias_write(clocks_state.rosc_ctrl, val, alias);
        break;
    case ROSC_R_FREQA:
        clocks_state.rosc_freqa = apply_alias_write(clocks_state.rosc_freqa, val, alias);
        break;
    case ROSC_R_FREQB:
        clocks_state.rosc_freqb = apply_alias_write(clocks_state.rosc_freqb, val, alias);
        break;
    case ROSC_R_DIV:
        clocks_state.rosc_div = apply_alias_write(clocks_state.rosc_div, val, alias);
        break;
    case ROSC_R_PHASE:
        clocks_state.rosc_phase = apply_alias_write(clocks_state.rosc_phase, val, alias);
        break;
    case ROSC_R_COUNT:
        clocks_state.rosc_count = apply_alias_write(clocks_state.rosc_count, val, alias);
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

    /* RP2040 bases (and shared bases) */
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
    if (base_aligned == ROSC_BASE)
        return rosc_read(canonical);

    /* RP2350-only bases (only checked when in RP2350 mode to avoid collisions) */
    if (membus_rp2350_mode) {
        if (base_aligned == RP2350_RESETS_BASE)
            return resets_read(canonical);
        if (base_aligned == RP2350_CLOCKS_BASE)
            return clocks_domain_read(canonical);
        if (base_aligned == RP2350_XOSC_BASE)
            return xosc_read(canonical);
        if (base_aligned == RP2350_PLL_SYS_BASE)
            return pll_read(&clocks_state.pll_sys, canonical & 0xFFF);
        if (base_aligned == RP2350_PLL_USB_BASE)
            return pll_read(&clocks_state.pll_usb, canonical & 0xFFF);
        if (base_aligned == RP2350_WATCHDOG_BASE)
            return watchdog_read(canonical);
        if (base_aligned == RP2350_PSM_BASE)
            return psm_read(canonical);
        if (base_aligned == RP2350_ROSC_BASE)
            return rosc_read(canonical);
    }

    return 0;
}

void clocks_write32(uint32_t addr, uint32_t val) {
    uint32_t base_aligned = addr & ~0x3FFF;
    uint32_t alias = (addr >> 12) & 0x3;
    uint32_t canonical = base_aligned | (addr & 0xFFF);

    /* RP2040 bases (and shared bases) */
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
    else if (base_aligned == ROSC_BASE)
        rosc_write(canonical, val, alias);
    /* RP2350-only bases */
    else if (membus_rp2350_mode && base_aligned == RP2350_RESETS_BASE)
        resets_write(canonical, val, alias);
    else if (membus_rp2350_mode && base_aligned == RP2350_CLOCKS_BASE)
        clocks_domain_write(canonical, val, alias);
    else if (membus_rp2350_mode && base_aligned == RP2350_XOSC_BASE)
        xosc_write(canonical, val, alias);
    else if (membus_rp2350_mode && base_aligned == RP2350_PLL_SYS_BASE)
        pll_write(&clocks_state.pll_sys, canonical & 0xFFF, val, alias);
    else if (membus_rp2350_mode && base_aligned == RP2350_PLL_USB_BASE)
        pll_write(&clocks_state.pll_usb, canonical & 0xFFF, val, alias);
    else if (membus_rp2350_mode && base_aligned == RP2350_WATCHDOG_BASE)
        watchdog_write(canonical, val, alias);
    else if (membus_rp2350_mode && base_aligned == RP2350_PSM_BASE)
        psm_write(canonical, val, alias);
    else if (membus_rp2350_mode && base_aligned == RP2350_ROSC_BASE)
        rosc_write(canonical, val, alias);
}
