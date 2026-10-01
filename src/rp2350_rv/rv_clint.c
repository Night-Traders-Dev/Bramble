/*
 * RISC-V CLINT (Core Local Interruptor) for RP2350 Hazard3
 *
 * Provides machine timer, software interrupts, and external interrupt
 * aggregation for dual Hazard3 RISC-V harts.
 */

#include <string.h>
#include <stdio.h>
#include "rp2350_rv/rv_clint.h"

/* ========================================================================
 * Initialization
 * ======================================================================== */

void rv_clint_init(rv_clint_state_t *clint, uint32_t cycles_per_us) {
    memset(clint, 0, sizeof(*clint));
    /* Default: timer compare at max so no immediate interrupt */
    clint->mtimecmp[0] = UINT64_MAX;
    clint->mtimecmp[1] = UINT64_MAX;
    clint->cycles_per_us = cycles_per_us > 0 ? cycles_per_us : 1;
    clint->tick_us = 1;
}

/* ========================================================================
 * Timer Tick
 * ======================================================================== */

void rv_clint_tick(rv_clint_state_t *clint, uint32_t cycles) {
    clint->cycle_accum += cycles;
    if (clint->cycle_accum >= clint->cycles_per_us) {
        uint32_t us = clint->cycle_accum / clint->cycles_per_us;
        clint->mtime += us;
        clint->cycle_accum %= clint->cycles_per_us;
    }
}

/* ========================================================================
 * Address Matching
 * ======================================================================== */

int rv_clint_match(uint32_t addr) {
    return addr >= RV_CLINT_BASE && addr < RV_CLINT_BASE + RV_CLINT_SIZE;
}

/* ========================================================================
 * Register Access
 * ======================================================================== */

/* hart currently accessing the shared CLINT registers */
static int cur_hart = 0;

void rv_clint_set_current_hart(int hart_id) {
    cur_hart = (hart_id < 0 || hart_id > 1) ? 0 : hart_id;
}

int rv_clint_current_hart(void) {
    return cur_hart;
}

uint32_t rv_clint_read(rv_clint_state_t *clint, uint32_t offset) {
    int h = cur_hart ? 1 : 0;
    switch (offset) {
    case RV_CLINT_MTIME_LO:     return (uint32_t)clint->mtime;
    case RV_CLINT_MTIME_HI:     return (uint32_t)(clint->mtime >> 32);
    case RV_CLINT_MTIME_CTRL:   return clint->mtime_ctrl;
    /* RISCV_SOFTIRQ reports this hart's software-interrupt pending bit. */
    case RV_CLINT_MSIP:         return clint->msip[h] & 1;
    /* MTIMECMP is per-hart at the same address. */
    case RV_CLINT_MTIMECMP0_LO: return (uint32_t)clint->mtimecmp[h];
    case RV_CLINT_MTIMECMP0_HI: return (uint32_t)(clint->mtimecmp[h] >> 32);
    default: return 0;
    }
}

void rv_clint_write(rv_clint_state_t *clint, uint32_t offset, uint32_t val) {
    switch (offset) {
    case RV_CLINT_MTIME_LO:
        clint->mtime = (clint->mtime & 0xFFFFFFFF00000000ULL) | val;
        break;
    case RV_CLINT_MTIME_HI:
        clint->mtime = (clint->mtime & 0xFFFFFFFF) | ((uint64_t)val << 32);
        break;
    case RV_CLINT_MTIME_CTRL:
        clint->mtime_ctrl = val;
        break;
    case RV_CLINT_MTIMECMP0_LO:
        clint->mtimecmp[cur_hart ? 1 : 0] =
            (clint->mtimecmp[cur_hart ? 1 : 0] & 0xFFFFFFFF00000000ULL) | val;
        break;
    case RV_CLINT_MTIMECMP0_HI:
        clint->mtimecmp[cur_hart ? 1 : 0] =
            (clint->mtimecmp[cur_hart ? 1 : 0] & 0xFFFFFFFF) | ((uint64_t)val << 32);
        break;
    case RV_CLINT_MSIP:
        clint->msip[cur_hart ? 1 : 0] = val & 1;
        break;
    default:
        break;
    }
}

/* ========================================================================
 * External Interrupt Management
 * ======================================================================== */

void rv_clint_set_ext_pending(rv_clint_state_t *clint, uint32_t irq_num) {
    if (irq_num < RV_NUM_EXT_IRQS)
        clint->ext_pending |= (1ULL << irq_num);
}

void rv_clint_clear_ext_pending(rv_clint_state_t *clint, uint32_t irq_num) {
    if (irq_num < RV_NUM_EXT_IRQS)
        clint->ext_pending &= ~(1ULL << irq_num);
}

/* ========================================================================
 * Interrupt Delivery
 *
 * Checks all interrupt sources and delivers the highest-priority pending
 * interrupt to the hart if mstatus.MIE is set and the source is enabled.
 * ======================================================================== */

int rv_clint_check_interrupts(rv_clint_state_t *clint, rv_cpu_state_t *hart) {
    uint32_t mstatus = hart->csr[CSR_MSTATUS];
    uint32_t mie_csr = hart->csr[CSR_MIE];
    uint32_t mip = 0;
    int hart_id = hart->hart_id;

    /* Compute mip from hardware state */

    /* Timer interrupt: mtime >= mtimecmp */
    if (hart_id < 2 && clint->mtime >= clint->mtimecmp[hart_id])
        mip |= MIP_MTIP;

    /* Software interrupt */
    if (hart_id < 2 && clint->msip[hart_id])
        mip |= MIP_MSIP;

    /* External interrupt: any enabled external IRQ pending for this hart */
    if (hart_id < 2) {
        uint64_t active = clint->ext_pending & clint->ext_enable[hart_id];
        if (active)
            mip |= MIP_MEIP;
    }

    /* Update mip CSR so firmware can read it */
    hart->csr[CSR_MIP] = mip;

    /* WFI ignores the global interrupt enable. Per datasheet 3.8.1.23: "wfi
     * ignores the global interrupt enable, MSTATUS.MIE. It respects all other
     * interrupt controls... If MIP.MEIP is 1, MIE.MEIE is 1, and MSTATUS.MIE is
     * 0, a wfi instruction falls through immediately without pausing." The
     * MIE test used to come first, so a hart that had cleared MSTATUS.MIE --
     * the canonical idle-loop idiom -- could never wake and hung permanently. */
    if (hart->is_wfi && (mip & mie_csr)) {
        hart->is_wfi = 0;
        /* Fall through; delivery is still gated on MIE below. */
    }

    /* Only then does the global enable gate delivery. */
    if (!(mstatus & MSTATUS_MIE))
        return 0;

    /* Determine which interrupt to deliver (priority: MEI > MSI > MTI) */
    uint32_t deliverable = mip & mie_csr;
    if (!deliverable)
        return 0;

    /* Deliver highest-priority interrupt */
    if (deliverable & MIP_MEIP) {
        rv_trap_enter(hart, MCAUSE_MEI, 0);
        return 1;
    }
    if (deliverable & MIP_MSIP) {
        rv_trap_enter(hart, MCAUSE_MSI, 0);
        return 1;
    }
    if (deliverable & MIP_MTIP) {
        rv_trap_enter(hart, MCAUSE_MTI, 0);
        return 1;
    }

    return 0;
}
