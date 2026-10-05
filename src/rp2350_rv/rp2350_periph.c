/*
 * RP2350-Specific Peripheral Emulation
 *
 * Implements TICKS, POWMAN, QMI, OTP, BOOTRAM, TIMER1, GLITCH,
 * CORESIGHT, and ACCESSCTRL peripherals for RP2350 emulation.
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/rp2350_periph.h"
#include "nvic.h"
#include "rp2350_rv/rp2350_memmap.h"

/* ========================================================================
 * Initialization
 * ======================================================================== */

void rp2350_periph_init(rp2350_periph_state_t *state, int for_arm) {
    tmds_init(&state->tmds);
    /* ARM: bx lr (0x4770) in Thumb. RISC-V: jalr x0, 0(ra) (0x00008067) */
    static const uint32_t bootram_stub_arm = 0x00004770u;
    static const uint32_t bootram_stub_rv  = 0x00008067u;
    const uint32_t bootram_xip_reentry_stub = for_arm ? bootram_stub_arm : bootram_stub_rv;

    memset(state, 0, sizeof(*state));

    /* TICKS: enable proc0/proc1/timer0 by default (1 tick per cycle) */
    state->ticks.ctrl[0] = 1;  /* PROC0 enabled */
    state->ticks.ctrl[1] = 1;  /* PROC1 enabled */
    state->ticks.ctrl[2] = 1;  /* TIMER0 enabled */
    state->ticks.ctrl[3] = 1;  /* TIMER1 enabled */
    state->ticks.ctrl[4] = 1;  /* WATCHDOG enabled */

    /* POWMAN: default power state (all domains on) */
    state->powman.state = 0x0000000F;  /* All domains powered */
    state->powman.vreg_ctrl = 0x000000B1;  /* Default VREG (1.1V, enabled) */
    state->powman.bod_ctrl = 0x00000091;   /* Default BOD (enabled) */

    /* QMI: default flash read command (03h, standard SPI) */
    state->qmi.m0_rcmd = 0x03000000;
    state->qmi.direct_csr = 0x01;  /* EN=1 */

    /* OTP: unprogrammed (all 0xFFFF) */
    memset(state->otp.data, 0xFF, sizeof(state->otp.data));

    /* BOOTRAM: cleared */
    memset(state->bootram, 0, BOOTRAM_SIZE);
    state->bootram_bootlock_stat = 0xFF;  /* All bootlocks start unclaimed. */
    /* The RP2350 SDK later copies BOOTRAM_BASE into a RAM buffer and calls it
     * to "re-enter XIP". In the emulator XIP stays accessible, so a simple
     * return stub is sufficient and avoids jumping into zero-filled RAM. */
    memcpy(state->bootram, &bootram_xip_reentry_stub, sizeof(bootram_xip_reentry_stub));

    /* Timer1: starts at 0 */
    state->timer1.time_us = 0;

    /* ACCESSCTRL: default all-access */
    memset(state->accessctrl_regs, 0xFF, sizeof(state->accessctrl_regs));
}

/* ========================================================================
 * Address Matching
 * ======================================================================== */

int rp2350_periph_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000u;  /* Strip atomic aliases */

    /* TICKS */
    if (base >= RP2350_TICKS_BASE && base < RP2350_TICKS_BASE + 0x100) return 1;
    /* POWMAN */
    if (base >= RP2350_POWMAN_BASE && base < RP2350_POWMAN_BASE + 0x100) return 1;
    /* QMI */
    if (base >= RP2350_QMI_BASE && base < RP2350_QMI_BASE + 0x100) return 1;
    /* OTP controller */
    if (base >= RP2350_OTP_BASE && base < RP2350_OTP_BASE + 0x100) return 1;
    /* OTP data */
    if (addr >= RP2350_OTP_DATA_BASE && addr < RP2350_OTP_DATA_BASE + OTP_NUM_ROWS * 4) return 1;
    /* BOOTRAM scratch plus adjacent bootrom-owned registers */
    if (addr >= RP2350_BOOTRAM_BASE && addr < RP2350_BOOTRAM_BASE + BOOTRAM_REGS_END) return 1;
    /* TIMER1 */
    if (base >= RP2350_TIMER1_BASE && base < RP2350_TIMER1_BASE + 0x100) return 1;
    /* GLITCH */
    if (base >= RP2350_GLITCH_BASE && base < RP2350_GLITCH_BASE + 0x20) return 1;
    /* CORESIGHT */
    if (base >= RP2350_CORESIGHT_BASE && base < RP2350_CORESIGHT_BASE + 0x40) return 1;
    /* ACCESSCTRL */
    if (base >= RP2350_ACCESSCTRL_BASE && base < RP2350_ACCESSCTRL_BASE + 0x100) return 1;
    /* TIMER0 at RP2350 address (moved from 0x40054000 to 0x400B0000) */
    if (base >= RP2350_TIMER0_BASE && base < RP2350_TIMER0_BASE + 0x100) return 1;

    return 0;
}

/* ========================================================================
 * TICKS (0x40108000)
 *
 * RP2350 datasheet Table 649 gives *three* registers per generator at a 12-byte
 * stride, not two at 8:
 *
 *   0x000 PROC0_CTRL    0x004 PROC0_CYCLES    0x008 PROC0_COUNT
 *   0x00c PROC1_CTRL    0x010 PROC1_CYCLES    0x014 PROC1_COUNT
 *   0x018 TIMER0_CTRL   0x01c TIMER0_CYCLES   0x020 TIMER0_COUNT
 *   0x024 TIMER1_CTRL   0x028 TIMER1_CYCLES   0x02c TIMER1_COUNT
 *   0x030 WATCHDOG_CTRL 0x034 WATCHDOG_CYCLES 0x038 WATCHDOG_COUNT
 *   0x03c RISCV_CTRL    0x040 RISCV_CYCLES    0x044 RISCV_COUNT
 *
 * The old decoder assumed an 8-byte stride, so every generator past the first
 * aliased onto the wrong registers: TIMER1_CTRL at 0x024 decoded as
 * 0x24/8 = generator 4, register 4 -- i.e. the CYCLES of the *watchdog*.
 * Firmware that selected TIMER1's tick source, or read the RISC-V platform
 * timer through TICKS, got another peripheral's register.
 *
 * CTRL is read/write, CYCLES and COUNT are read-only.
 * ======================================================================== */

#define TICKS_STRIDE   12u
#define TICKS_REG_CTRL   0u
#define TICKS_REG_CYCLES 4u
#define TICKS_REG_COUNT  8u

static uint32_t ticks_read(rp2350_ticks_state_t *t, uint32_t offset) {
    uint32_t gen = offset / TICKS_STRIDE;
    uint32_t reg = offset % TICKS_STRIDE;
    if (gen >= RP2350_TICKS_NUM_GENERATORS) return 0;
    switch (reg) {
    case TICKS_REG_CTRL:   return t->ctrl[gen];
    case TICKS_REG_CYCLES: return t->cycles[gen];
    case TICKS_REG_COUNT:  return t->count[gen];
    default: return 0;
    }
}

static void ticks_write(rp2350_ticks_state_t *t, uint32_t offset, uint32_t val) {
    uint32_t gen = offset / TICKS_STRIDE;
    uint32_t reg = offset % TICKS_STRIDE;
    if (gen >= RP2350_TICKS_NUM_GENERATORS) return;
    if (reg == TICKS_REG_CTRL)
        t->ctrl[gen] = val;
    /* CYCLES and COUNT are read-only. */
}

/* Advance the COUNT latches of every enabled generator. */
void rp2350_ticks_tick(rp2350_ticks_state_t *t, uint32_t ticks) {
    if (ticks == 0) return;
    for (uint32_t g = 0; g < RP2350_TICKS_NUM_GENERATORS; g++) {
        if (t->ctrl[g] != 0)
            t->count[g] += ticks;
    }
}

/* ========================================================================
 * POWMAN (0x40100000)
 * ======================================================================== */

static uint32_t powman_read(rp2350_powman_state_t *p, uint32_t offset) {
    switch (offset) {
    case 0x00: return p->vreg_ctrl;
    case 0x04: return p->vreg_ctrl | 0x00001000;  /* VREG_STATUS: ROK=1 */
    case 0x08: return p->bod_ctrl;
    case 0x0C: return p->bod_ctrl | 0x00001000;   /* BOD_STATUS: OK=1 */
    case 0x10: return p->state;
    case 0x50: return (uint32_t)p->timer;
    case 0x54: return p->timer_hi;
    case 0x60: return p->inte;
    case 0x64: return p->intf;
    case 0x68: return p->ints;
    default:
        if (offset / 4 < 32) return p->regs[offset / 4];
        return 0;
    }
}

static void powman_write(rp2350_powman_state_t *p, uint32_t offset, uint32_t val) {
    switch (offset) {
    case 0x00: p->vreg_ctrl = val; break;
    case 0x08: p->bod_ctrl = val; break;
    case 0x60: p->inte = val; break;
    case 0x64: p->intf = val; break;
    default:
        if (offset / 4 < 32) p->regs[offset / 4] = val;
        break;
    }
}

/* ========================================================================
 * QMI (0x400D0000)
 * ======================================================================== */

static uint32_t qmi_read(rp2350_qmi_state_t *q, uint32_t offset) {
    switch (offset) {
    case 0x00: return q->direct_csr | 0x00040000;  /* BUSY=0, EN=1 */
    case 0x04: return q->direct_tx;
    case 0x08: return q->direct_rx;
    case 0x0C: return q->m0_timing;
    case 0x10: return q->m0_rfmt;
    case 0x14: return q->m0_rcmd;
    case 0x18: return q->m0_wfmt;
    case 0x1C: return q->m0_wcmd;
    case 0x20: return q->m1_timing;
    case 0x24: return q->m1_rfmt;
    case 0x28: return q->m1_rcmd;
    case 0x2C: return q->m1_wfmt;
    case 0x30: return q->m1_wcmd;
    default:
        if (offset >= 0x34 && offset < 0x54)
            return q->atrans[(offset - 0x34) / 4];
        return 0;
    }
}

static void qmi_write(rp2350_qmi_state_t *q, uint32_t offset, uint32_t val) {
    switch (offset) {
    case 0x00: q->direct_csr = val; break;
    case 0x04: q->direct_tx = val; break;
    case 0x0C: q->m0_timing = val; break;
    case 0x10: q->m0_rfmt = val; break;
    case 0x14: q->m0_rcmd = val; break;
    case 0x18: q->m0_wfmt = val; break;
    case 0x1C: q->m0_wcmd = val; break;
    case 0x20: q->m1_timing = val; break;
    case 0x24: q->m1_rfmt = val; break;
    case 0x28: q->m1_rcmd = val; break;
    case 0x2C: q->m1_wfmt = val; break;
    case 0x30: q->m1_wcmd = val; break;
    default:
        if (offset >= 0x34 && offset < 0x54)
            q->atrans[(offset - 0x34) / 4] = val;
        break;
    }
}

/* ========================================================================
 * OTP (0x40120000 control, 0x40130000 data readout)
 * ======================================================================== */

static uint32_t otp_read(rp2350_otp_state_t *o, uint32_t addr) {
    if (addr >= RP2350_OTP_DATA_BASE) {
        /* Data readout: each row is 32-bit aligned, returns 16-bit data in low halfword */
        uint32_t row = (addr - RP2350_OTP_DATA_BASE) / 4;
        if (row < OTP_NUM_ROWS) return o->data[row];
        return 0;
    }
    /* Controller registers */
    uint32_t offset = (addr - RP2350_OTP_BASE) & 0xFFF;
    if (offset / 4 < 32) return o->ctrl_regs[offset / 4];
    return 0;
}

static void otp_write(rp2350_otp_state_t *o, uint32_t addr, uint32_t val) {
    if (addr >= RP2350_OTP_DATA_BASE) return;  /* Data is read-only */
    uint32_t offset = (addr - RP2350_OTP_BASE) & 0xFFF;
    if (offset / 4 < 32) o->ctrl_regs[offset / 4] = val;
}

/* ========================================================================
 * BOOTRAM register bank (0x400E0000)
 * ======================================================================== */

static uint32_t bootram_read(rp2350_periph_state_t *state, uint32_t addr) {
    uint32_t offset = addr - RP2350_BOOTRAM_BASE;

    if (offset < BOOTRAM_SIZE) {
        uint32_t val = 0;
        memcpy(&val, &state->bootram[offset], 4);
        return val;
    }

    switch (offset) {
    case BOOTRAM_WRITE_ONCE_OFFSET:
        return state->bootram_write_once[0];
    case BOOTRAM_WRITE_ONCE_OFFSET + 4:
        return state->bootram_write_once[1];
    case BOOTRAM_BOOTLOCK_STAT_OFFSET:
        return state->bootram_bootlock_stat;
    default:
        if (offset >= BOOTRAM_BOOTLOCK0_OFFSET &&
            offset < BOOTRAM_BOOTLOCK0_OFFSET + BOOTRAM_BOOTLOCK_COUNT * 4) {
            uint32_t lock_num = (offset - BOOTRAM_BOOTLOCK0_OFFSET) / 4;
            uint32_t bit = 1u << lock_num;
            if (state->bootram_bootlock_stat & bit) {
                state->bootram_bootlock_stat &= ~bit;
                return bit;
            }
            return 0;
        }
        return 0;
    }
}

static void bootram_write(rp2350_periph_state_t *state, uint32_t addr, uint32_t val) {
    uint32_t offset = addr - RP2350_BOOTRAM_BASE;

    if (offset < BOOTRAM_SIZE) {
        memcpy(&state->bootram[offset], &val, 4);
        return;
    }

    switch (offset) {
    case BOOTRAM_WRITE_ONCE_OFFSET:
        state->bootram_write_once[0] |= val;
        break;
    case BOOTRAM_WRITE_ONCE_OFFSET + 4:
        state->bootram_write_once[1] |= val;
        break;
    case BOOTRAM_BOOTLOCK_STAT_OFFSET:
        state->bootram_bootlock_stat = val & 0xFF;
        break;
    default:
        if (offset >= BOOTRAM_BOOTLOCK0_OFFSET &&
            offset < BOOTRAM_BOOTLOCK0_OFFSET + BOOTRAM_BOOTLOCK_COUNT * 4) {
            uint32_t lock_num = (offset - BOOTRAM_BOOTLOCK0_OFFSET) / 4;
            state->bootram_bootlock_stat |= (1u << lock_num);
        }
        break;
    }
}

/* ========================================================================
 * TIMER1 (0x400B8000) — same register layout as RP2040 timer
 * ======================================================================== */

static uint32_t timer1_read(rp2350_timer1_state_t *t, uint32_t offset) {
    switch (offset) {
    case 0x08: return (uint32_t)(t->time_us >> 32);  /* TIMEHR (latched on TIMELR read) */
    case 0x0C:
        t->latched_high = (uint32_t)(t->time_us >> 32);
        return (uint32_t)t->time_us;  /* TIMELR */
    case 0x10: return t->alarm[0];
    case 0x14: return t->alarm[1];
    case 0x18: return t->alarm[2];
    case 0x1C: return t->alarm[3];
    case 0x20: return t->armed;
    case 0x24: return (uint32_t)(t->time_us >> 32);  /* TIMERAWH */
    case 0x28: return (uint32_t)t->time_us;           /* TIMERAWL */
    case 0x30: return t->paused;
    /* RP2350 adds LOCKED (0x34) and SOURCE (0x38) ahead of the interrupt
     * registers (datasheet: "RP2350 added two new registers"), so INTR..INTS
     * sit at 0x3c..0x48 here rather than the RP2040 0x34..0x40. Using the
     * RP2040 offsets made an INTE write land on INTS and be dropped. */
    case 0x3C: return t->intr;
    case 0x40: return t->inte;
    case 0x44: return t->intf;
    case 0x48: return (t->intr | t->intf) & t->inte;  /* INTS */
    default: return 0;
    }
}

static void timer1_write(rp2350_timer1_state_t *t, uint32_t offset, uint32_t val) {
    switch (offset) {
    case 0x00: t->time_us = (t->time_us & 0xFFFFFFFF) | ((uint64_t)val << 32); break;
    case 0x04: t->time_us = (t->time_us & 0xFFFFFFFF00000000ULL) | val; break;
    case 0x10: t->alarm[0] = val; t->armed |= 1; break;
    case 0x14: t->alarm[1] = val; t->armed |= 2; break;
    case 0x18: t->alarm[2] = val; t->armed |= 4; break;
    case 0x1C: t->alarm[3] = val; t->armed |= 8; break;
    case 0x20: t->armed &= ~val; break;  /* W1C */
    case 0x30: t->paused = val & 1; break;
    case 0x34: break;                     /* LOCKED (RO) */
    case 0x38: break;                     /* SOURCE */
    case 0x3C: t->intr &= ~val; break;    /* W1C */
    case 0x40: t->inte = val & 0xF; break;
    case 0x44: t->intf = val & 0xF; break;
    default: break;
    }
}

void rp2350_timer1_tick(rp2350_periph_state_t *state, uint32_t us) {
    rp2350_timer1_state_t *t = &state->timer1;
    if (t->paused || us == 0) return;
    t->time_us += us;
    /* Check alarms */
    uint32_t time_lo = (uint32_t)t->time_us;
    for (int i = 0; i < 4; i++) {
        if ((t->armed & (1u << i)) && (int32_t)(time_lo - t->alarm[i]) >= 0) {
            t->intr |= (1u << i);
            t->armed &= ~(1u << i);
            /* Raise the matching TIMER1 interrupt. These were never signalled
             * before, so any firmware using timer1 hung forever waiting for a
             * callback. RP2350 vectors are 4..7 (TIMER1_IRQ_0..3); nvic_
             * irq_number maps the RP2040 TIMER_IRQ_0..3 onto them. */
            t->fired |= (1u << i);
        }
    }
    if (t->fired) {
        uint32_t mask = t->fired & t->inte;
        if (mask) {
            uint32_t line = (uint32_t)__builtin_ctz(mask);
            nvic_signal_irq(IRQ_TIMER_IRQ_0 + line);
        }
        t->fired = 0;
    }
}

/* ========================================================================
 * Unified Read/Write Dispatch
 * ======================================================================== */

uint32_t rp2350_periph_read32(rp2350_periph_state_t *state, uint32_t addr) {
    uint32_t base = addr & ~0x3000u;

    /* TICKS. The register offset must come from the full address, not from
     * `base`: `base` is 16KB-aligned, so subtracting it threw the offset away
     * and every TICKS register -- TIMER1_CTRL, WATCHDOG_COUNT, all of them --
     * resolved to generator 0's CTRL. */
    if (addr >= RP2350_TICKS_BASE && addr < RP2350_TICKS_BASE + 0x100)
        return ticks_read(&state->ticks, addr - RP2350_TICKS_BASE);

    /* POWMAN */
    if (base >= RP2350_POWMAN_BASE && base < RP2350_POWMAN_BASE + 0x100)
        return powman_read(&state->powman, base - RP2350_POWMAN_BASE);

    /* QMI */
    if (base >= RP2350_QMI_BASE && base < RP2350_QMI_BASE + 0x100)
        return qmi_read(&state->qmi, base - RP2350_QMI_BASE);

    /* OTP controller + data */
    if (base >= RP2350_OTP_BASE && base < RP2350_OTP_BASE + 0x100)
        return otp_read(&state->otp, base);
    if (addr >= RP2350_OTP_DATA_BASE && addr < RP2350_OTP_DATA_BASE + OTP_NUM_ROWS * 4)
        return otp_read(&state->otp, addr);

    /* BOOTRAM scratch plus adjacent bootrom registers */
    if (addr >= RP2350_BOOTRAM_BASE && addr < RP2350_BOOTRAM_BASE + BOOTRAM_REGS_END)
        return bootram_read(state, addr);

    /* TIMER1 */
    if (base >= RP2350_TIMER1_BASE && base < RP2350_TIMER1_BASE + 0x100)
        return timer1_read(&state->timer1, base - RP2350_TIMER1_BASE);

    /* TIMER0 at RP2350 address — redirect to RP2040 timer via offset translation */
    if (base >= RP2350_TIMER0_BASE && base < RP2350_TIMER0_BASE + 0x100) {
        extern uint32_t timer_read32(uint32_t addr);
        return timer_read32(0x40054000 + (base - RP2350_TIMER0_BASE));
    }

    /* GLITCH */
    if (base >= RP2350_GLITCH_BASE && base < RP2350_GLITCH_BASE + 0x20) {
        uint32_t idx = (base - RP2350_GLITCH_BASE) / 4;
        return (idx < 8) ? state->glitch_regs[idx] : 0;
    }

    /* CORESIGHT */
    if (base >= RP2350_CORESIGHT_BASE && base < RP2350_CORESIGHT_BASE + 0x40) {
        uint32_t idx = (base - RP2350_CORESIGHT_BASE) / 4;
        return (idx < 16) ? state->coresight_regs[idx] : 0;
    }

    /* ACCESSCTRL */
    if (base >= 0x40160000 && base < 0x40160000 + 0x100) {
        uint32_t idx = (base - 0x40160000) / 4;
        return (idx < 64) ? state->accessctrl_regs[idx] : 0;
    }

    return 0;
}

void rp2350_periph_write32(rp2350_periph_state_t *state, uint32_t addr, uint32_t val) {
    uint32_t base = addr & ~0x3000u;

    /* TICKS -- see the note on the read side about not using `base`. */
    if (addr >= RP2350_TICKS_BASE && addr < RP2350_TICKS_BASE + 0x100) {
        ticks_write(&state->ticks, addr - RP2350_TICKS_BASE, val);
        return;
    }

    /* POWMAN */
    if (base >= RP2350_POWMAN_BASE && base < RP2350_POWMAN_BASE + 0x100) {
        powman_write(&state->powman, base - RP2350_POWMAN_BASE, val);
        return;
    }

    /* QMI */
    if (base >= RP2350_QMI_BASE && base < RP2350_QMI_BASE + 0x100) {
        qmi_write(&state->qmi, base - RP2350_QMI_BASE, val);
        return;
    }

    /* OTP */
    if (base >= RP2350_OTP_BASE && base < RP2350_OTP_BASE + 0x100) {
        otp_write(&state->otp, base, val);
        return;
    }

    /* BOOTRAM scratch plus adjacent bootrom registers */
    if (addr >= RP2350_BOOTRAM_BASE && addr < RP2350_BOOTRAM_BASE + BOOTRAM_REGS_END) {
        bootram_write(state, addr, val);
        return;
    }

    /* TIMER1 */
    if (base >= RP2350_TIMER1_BASE && base < RP2350_TIMER1_BASE + 0x100) {
        timer1_write(&state->timer1, base - RP2350_TIMER1_BASE, val);
        return;
    }

    /* TIMER0 at RP2350 address — redirect */
    if (base >= RP2350_TIMER0_BASE && base < RP2350_TIMER0_BASE + 0x100) {
        extern void timer_write32(uint32_t addr, uint32_t val);
        timer_write32(0x40054000 + (base - RP2350_TIMER0_BASE), val);
        return;
    }

    /* GLITCH */
    if (base >= RP2350_GLITCH_BASE && base < RP2350_GLITCH_BASE + 0x20) {
        uint32_t idx = (base - RP2350_GLITCH_BASE) / 4;
        if (idx < 8) state->glitch_regs[idx] = val;
        return;
    }

    /* CORESIGHT */
    if (base >= RP2350_CORESIGHT_BASE && base < RP2350_CORESIGHT_BASE + 0x40) {
        uint32_t idx = (base - RP2350_CORESIGHT_BASE) / 4;
        if (idx < 16) state->coresight_regs[idx] = val;
        return;
    }

    /* ACCESSCTRL */
    if (base >= 0x40160000 && base < 0x40160000 + 0x100) {
        uint32_t idx = (base - 0x40160000) / 4;
        if (idx < 64) state->accessctrl_regs[idx] = val;
        return;
    }
}

uint8_t rp2350_periph_read8(rp2350_periph_state_t *state, uint32_t addr) {
    /* BOOTRAM byte access */
    if (addr >= RP2350_BOOTRAM_BASE && addr < RP2350_BOOTRAM_BASE + BOOTRAM_SIZE)
        return state->bootram[addr - RP2350_BOOTRAM_BASE];
    /* Fall back to 32-bit read */
    uint32_t aligned = addr & ~3u;
    uint32_t val = rp2350_periph_read32(state, aligned);
    return (uint8_t)(val >> ((addr & 3) * 8));
}

void rp2350_periph_write8(rp2350_periph_state_t *state, uint32_t addr, uint8_t val) {
    /* BOOTRAM byte access */
    if (addr >= RP2350_BOOTRAM_BASE && addr < RP2350_BOOTRAM_BASE + BOOTRAM_SIZE) {
        state->bootram[addr - RP2350_BOOTRAM_BASE] = val;
        return;
    }
    /* Other peripherals: read-modify-write */
    uint32_t aligned = addr & ~3u;
    uint32_t word = rp2350_periph_read32(state, aligned);
    uint32_t shift = (addr & 3) * 8;
    word = (word & ~(0xFFu << shift)) | ((uint32_t)val << shift);
    rp2350_periph_write32(state, aligned, word);
}
