/*
 * Hazard3 RISC-V CPU Engine for RP2350 Emulation
 *
 * RV32IMAC core:
 *   I  - Base integer (37 instructions)
 *   M  - Multiply/divide (8 instructions)
 *   A  - Atomics (11 instructions)
 *   C  - Compressed 16-bit (36 instructions)
 *   Zicsr    - CSR access (6 instructions)
 *   Zifencei - Instruction fence
 *
 * Phase 1: RV32I base integer only
 */

#ifndef RV_CPU_H
#define RV_CPU_H

#include <stdint.h>

/* ========================================================================
 * RISC-V CPU State
 * ======================================================================== */

/* CSR addresses (machine mode) */
#define CSR_MSTATUS     0x300
#define CSR_MISA        0x301
#define CSR_MIE         0x304
#define CSR_MTVEC       0x305
#define CSR_MSCRATCH    0x340
#define CSR_MEPC        0x341
#define CSR_MCAUSE      0x342
#define CSR_MTVAL       0x343
#define CSR_MIP         0x344
#define CSR_MCYCLE      0xB00
#define CSR_MINSTRET    0xB02
#define CSR_MCYCLEH     0xB80
#define CSR_MINSTRETH   0xB82
#define CSR_MVENDORID   0xF11
#define CSR_MARCHID     0xF12
#define CSR_MIMPID      0xF13
#define CSR_MHARTID     0xF14

/* mstatus bits */
#define MSTATUS_MIE     (1u << 3)   /* Machine interrupt enable */
#define MSTATUS_MPIE    (1u << 7)   /* Previous MIE */
#define MSTATUS_MPP     (3u << 11)  /* Previous privilege mode */

/* mcause values */
#define MCAUSE_INSTR_MISALIGNED     0
#define MCAUSE_INSTR_ACCESS_FAULT   1
#define MCAUSE_ILLEGAL_INSTR        2
#define MCAUSE_BREAKPOINT           3
#define MCAUSE_LOAD_MISALIGNED      4
#define MCAUSE_LOAD_ACCESS_FAULT    5
#define MCAUSE_STORE_MISALIGNED     6
#define MCAUSE_STORE_ACCESS_FAULT   7
#define MCAUSE_ECALL_M              11
#define MCAUSE_INTERRUPT_BIT        (1u << 31)

/* RV32I instruction field extraction */
#define RV_OPCODE(instr)    ((instr) & 0x7F)
#define RV_RD(instr)        (((instr) >> 7) & 0x1F)
#define RV_FUNCT3(instr)    (((instr) >> 12) & 0x7)
#define RV_RS1(instr)       (((instr) >> 15) & 0x1F)
#define RV_RS2(instr)       (((instr) >> 20) & 0x1F)
#define RV_FUNCT7(instr)    (((instr) >> 25) & 0x7F)

/* Immediate extraction helpers */
static inline int32_t rv_imm_i(uint32_t instr) {
    return (int32_t)instr >> 20;  /* Sign-extended [31:20] */
}

static inline int32_t rv_imm_s(uint32_t instr) {
    return (int32_t)(((instr >> 20) & 0xFE0) | ((instr >> 7) & 0x1F)) << 20 >> 20;
}

static inline int32_t rv_imm_b(uint32_t instr) {
    uint32_t imm = ((instr >> 7) & 0x1E)     /* [4:1]  */
                 | ((instr >> 20) & 0x7E0)    /* [10:5] */
                 | ((instr << 4) & 0x800)     /* [11]   */
                 | ((instr >> 19) & 0x1000);  /* [12]   */
    return (int32_t)(imm << 19) >> 19;        /* Sign-extend from bit 12 */
}

static inline int32_t rv_imm_u(uint32_t instr) {
    return (int32_t)(instr & 0xFFFFF000);     /* [31:12] << 12 */
}

static inline int32_t rv_imm_j(uint32_t instr) {
    uint32_t imm = ((instr >> 20) & 0x7FE)    /* [10:1]  */
                 | ((instr >> 9) & 0x800)      /* [11]    */
                 | (instr & 0xFF000)           /* [19:12] */
                 | ((instr >> 11) & 0x100000); /* [20]    */
    return (int32_t)(imm << 11) >> 11;         /* Sign-extend from bit 20 */
}

/* Number of CSRs to track */
#define RV_CSR_COUNT 4096

typedef struct {
    /* General-purpose registers (x0 is hardwired to 0) */
    uint32_t x[32];

    /* Program counter */
    uint32_t pc;

    /* CSRs (indexed by 12-bit address) */
    uint32_t csr[RV_CSR_COUNT];

    /* Performance counters (64-bit) */
    uint64_t cycle_count;
    uint64_t instret_count;

    /* Atomic LR/SC reservation */
    uint32_t lr_reservation;    /* Reserved address (set by LR.W) */
    int lr_valid;               /* 1 if reservation is active */

    /* Trap state */
    int in_trap;            /* Currently handling a trap */

    /* Core identity */
    int hart_id;            /* 0 or 1 */

    /* Execution state */
    int is_halted;
    int is_wfi;
    uint32_t step_count;

    /* Debug */
    int debug_enabled;
} rv_cpu_state_t;

/* ========================================================================
 * API
 * ======================================================================== */

/* Initialize the RISC-V CPU */
void rv_cpu_init(rv_cpu_state_t *cpu, int hart_id);

/* Reset from flash vector */
void rv_cpu_reset(rv_cpu_state_t *cpu, uint32_t entry_pc);

/* Execute one instruction. Returns 0 on success, -1 on halt. */
int rv_cpu_step(rv_cpu_state_t *cpu);

/* Trap entry/return */
void rv_trap_enter(rv_cpu_state_t *cpu, uint32_t cause, uint32_t tval);
void rv_trap_return(rv_cpu_state_t *cpu);

/* CSR access */
uint32_t rv_csr_read(rv_cpu_state_t *cpu, uint16_t addr);
void rv_csr_write(rv_cpu_state_t *cpu, uint16_t addr, uint32_t val);

/* Check if CPU is halted */
int rv_cpu_is_halted(rv_cpu_state_t *cpu);

#endif /* RV_CPU_H */
