/*
 * RISC-V CLINT (Core Local Interruptor) for RP2350 Hazard3
 *
 * Provides:
 *   - Machine Timer (mtime / mtimecmp) → MTIP interrupt
 *   - Machine Software Interrupt (MSIP) → MSIP interrupt
 *   - External Interrupt aggregation → MEIP interrupt
 *
 * Memory-mapped at SIO space (0xD0000100):
 *   0xD0000100: MTIME_LO      (64-bit free-running timer, low word)
 *   0xD0000104: MTIME_HI
 *   0xD0000108: MTIMECMP0_LO  (hart 0 compare, low word)
 *   0xD000010C: MTIMECMP0_HI
 *   0xD0000110: MTIMECMP1_LO  (hart 1 compare, low word)
 *   0xD0000114: MTIMECMP1_HI
 *   0xD0000120: MSIP0          (hart 0 software interrupt, bit 0)
 *   0xD0000124: MSIP1          (hart 1 software interrupt, bit 0)
 */

#ifndef RV_CLINT_H
#define RV_CLINT_H

#include <stdint.h>
#include "rp2350_rv/rv_cpu.h"

/* CLINT register offsets from base (0xD0000100) */
#define RV_CLINT_BASE       0xD0000100
#define RV_CLINT_SIZE       0x30

#define RV_CLINT_MTIME_LO      0x00
#define RV_CLINT_MTIME_HI      0x04
#define RV_CLINT_MTIMECMP0_LO  0x08
#define RV_CLINT_MTIMECMP0_HI  0x0C
#define RV_CLINT_MTIMECMP1_LO  0x10
#define RV_CLINT_MTIMECMP1_HI  0x14
#define RV_CLINT_MSIP0         0x20
#define RV_CLINT_MSIP1         0x24

/* mip CSR bit positions */
#define MIP_MSIP    (1u << 3)   /* Machine Software Interrupt Pending */
#define MIP_MTIP    (1u << 7)   /* Machine Timer Interrupt Pending */
#define MIP_MEIP    (1u << 11)  /* Machine External Interrupt Pending */

/* mie CSR bit positions (same layout) */
#define MIE_MSIE    (1u << 3)
#define MIE_MTIE    (1u << 7)
#define MIE_MEIE    (1u << 11)

/* mcause interrupt values */
#define MCAUSE_MSI  (0x80000003u)   /* Machine software interrupt */
#define MCAUSE_MTI  (0x80000007u)   /* Machine timer interrupt */
#define MCAUSE_MEI  (0x8000000Bu)   /* Machine external interrupt */

/* Number of external interrupt sources (RP2350 has 52) */
#define RV_NUM_EXT_IRQS  52

typedef struct {
    /* 64-bit free-running timer */
    uint64_t mtime;

    /* Per-hart timer compare (2 harts) */
    uint64_t mtimecmp[2];

    /* Per-hart software interrupt pending (bit 0 only) */
    uint32_t msip[2];

    /* External interrupt pending/enable (52 IRQ lines) */
    uint64_t ext_pending;   /* One bit per IRQ source */
    uint64_t ext_enable[2]; /* Per-hart enable mask */
    uint64_t ext_priority[RV_NUM_EXT_IRQS]; /* 0 = highest */

    /* Timer tick rate (microseconds per mtime increment) */
    uint32_t tick_us;       /* Default: 1 (1 mtime tick per us) */

    /* Cycle accumulator for mtime updates */
    uint32_t cycles_per_us;
    uint32_t cycle_accum;
} rv_clint_state_t;

/* Initialize CLINT */
void rv_clint_init(rv_clint_state_t *clint, uint32_t cycles_per_us);

/* Advance mtime by elapsed cycles */
void rv_clint_tick(rv_clint_state_t *clint, uint32_t cycles);

/* Register read/write */
uint32_t rv_clint_read(rv_clint_state_t *clint, uint32_t offset);
void rv_clint_write(rv_clint_state_t *clint, uint32_t offset, uint32_t val);

/* Check if address is in CLINT range */
int rv_clint_match(uint32_t addr);

/* Signal an external interrupt (from peripheral) */
void rv_clint_set_ext_pending(rv_clint_state_t *clint, uint32_t irq_num);
void rv_clint_clear_ext_pending(rv_clint_state_t *clint, uint32_t irq_num);

/* Check and deliver pending interrupts to a hart.
 * Returns 1 if an interrupt was delivered, 0 otherwise. */
int rv_clint_check_interrupts(rv_clint_state_t *clint, rv_cpu_state_t *hart);

#endif /* RV_CLINT_H */
