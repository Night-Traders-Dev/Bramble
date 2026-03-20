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

uint32_t rv_clint_read(rv_clint_state_t *clint, uint32_t offset) {
    switch (offset) {
    case RV_CLINT_MTIME_LO:     return (uint32_t)clint->mtime;
    case RV_CLINT_MTIME_HI:     return (uint32_t)(clint->mtime >> 32);
    case RV_CLINT_MTIMECMP0_LO: return (uint32_t)clint->mtimecmp[0];
    case RV_CLINT_MTIMECMP0_HI: return (uint32_t)(clint->mtimecmp[0] >> 32);
    case RV_CLINT_MTIMECMP1_LO: return (uint32_t)clint->mtimecmp[1];
    case RV_CLINT_MTIMECMP1_HI: return (uint32_t)(clint->mtimecmp[1] >> 32);
    case RV_CLINT_MSIP0:        return clint->msip[0] & 1;
    case RV_CLINT_MSIP1:        return clint->msip[1] & 1;
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
    case RV_CLINT_MTIMECMP0_LO:
        clint->mtimecmp[0] = (clint->mtimecmp[0] & 0xFFFFFFFF00000000ULL) | val;
        break;
    case RV_CLINT_MTIMECMP0_HI:
        clint->mtimecmp[0] = (clint->mtimecmp[0] & 0xFFFFFFFF) | ((uint64_t)val << 32);
        break;
    case RV_CLINT_MTIMECMP1_LO:
        clint->mtimecmp[1] = (clint->mtimecmp[1] & 0xFFFFFFFF00000000ULL) | val;
        break;
    case RV_CLINT_MTIMECMP1_HI:
        clint->mtimecmp[1] = (clint->mtimecmp[1] & 0xFFFFFFFF) | ((uint64_t)val << 32);
        break;
    case RV_CLINT_MSIP0:
        clint->msip[0] = val & 1;
        break;
    case RV_CLINT_MSIP1:
        clint->msip[1] = val & 1;
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

    /* Check if interrupts are globally enabled */
    if (!(mstatus & MSTATUS_MIE))
        return 0;

    /* WFI wake: any pending+enabled interrupt wakes the hart */
    if (hart->is_wfi && (mip & mie_csr)) {
        hart->is_wfi = 0;
        /* Fall through to deliver if MIE is set */
    }

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
