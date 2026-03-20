/*
 * Hazard3 RISC-V CPU Engine — RV32I Base Integer
 *
 * Phase 1: Complete RV32I (37 instructions)
 *   - R-type: ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
 *   - I-type: ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
 *   - I-type loads: LB, LH, LW, LBU, LHU
 *   - S-type stores: SB, SH, SW
 *   - B-type branches: BEQ, BNE, BLT, BGE, BLTU, BGEU
 *   - U-type: LUI, AUIPC
 *   - J-type: JAL, JALR
 *   - System: ECALL, EBREAK, FENCE
 *   - CSR: CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI
 */

#include <stdio.h>
#include <string.h>
#include "rp2350_rv/rv_cpu.h"
#include "emulator.h"

/* ========================================================================
 * RV32I Opcode Map (bits [6:0])
 * ======================================================================== */

#define OP_LUI      0x37
#define OP_AUIPC    0x17
#define OP_JAL      0x6F
#define OP_JALR     0x67
#define OP_BRANCH   0x63
#define OP_LOAD     0x03
#define OP_STORE    0x23
#define OP_IMM      0x13
#define OP_REG      0x33
#define OP_FENCE    0x0F
#define OP_SYSTEM   0x73

/* ========================================================================
 * Initialization
 * ======================================================================== */

void rv_cpu_init(rv_cpu_state_t *cpu, int hart_id) {
    memset(cpu, 0, sizeof(*cpu));
    cpu->hart_id = hart_id;
    cpu->is_halted = 1;  /* Start halted until reset */

    /* Initialize misa: RV32IMAC */
    cpu->csr[CSR_MISA] = (1u << 30)   /* MXL=1 (32-bit) */
                        | (1u << 0)    /* A - Atomics */
                        | (1u << 2)    /* C - Compressed */
                        | (1u << 8)    /* I - Base integer */
                        | (1u << 12);  /* M - Multiply/divide */

    /* Hazard3 vendor/arch/impl IDs */
    cpu->csr[CSR_MVENDORID] = 0;       /* Non-commercial */
    cpu->csr[CSR_MARCHID]   = 0;
    cpu->csr[CSR_MIMPID]    = 0;
    cpu->csr[CSR_MHARTID]   = (uint32_t)hart_id;

    /* mstatus: machine mode, interrupts disabled */
    cpu->csr[CSR_MSTATUS] = MSTATUS_MPP;  /* MPP = M-mode */
}

void rv_cpu_reset(rv_cpu_state_t *cpu, uint32_t entry_pc) {
    cpu->pc = entry_pc;
    cpu->is_halted = 0;
    cpu->is_wfi = 0;
    cpu->step_count = 0;
    cpu->cycle_count = 0;
    cpu->instret_count = 0;
    memset(cpu->x, 0, sizeof(cpu->x));

    if (cpu->debug_enabled)
        fprintf(stderr, "[RV-CORE%d] Reset to PC=0x%08X\n", cpu->hart_id, entry_pc);
}

int rv_cpu_is_halted(rv_cpu_state_t *cpu) {
    return cpu->is_halted;
}

/* ========================================================================
 * CSR Access
 * ======================================================================== */

uint32_t rv_csr_read(rv_cpu_state_t *cpu, uint16_t addr) {
    switch (addr) {
    case CSR_MCYCLE:    return (uint32_t)cpu->cycle_count;
    case CSR_MCYCLEH:   return (uint32_t)(cpu->cycle_count >> 32);
    case CSR_MINSTRET:  return (uint32_t)cpu->instret_count;
    case CSR_MINSTRETH: return (uint32_t)(cpu->instret_count >> 32);
    case CSR_MHARTID:   return (uint32_t)cpu->hart_id;
    default:
        if (addr < RV_CSR_COUNT)
            return cpu->csr[addr];
        return 0;
    }
}

void rv_csr_write(rv_cpu_state_t *cpu, uint16_t addr, uint32_t val) {
    switch (addr) {
    case CSR_MCYCLE:    cpu->cycle_count = (cpu->cycle_count & 0xFFFFFFFF00000000ULL) | val; break;
    case CSR_MCYCLEH:   cpu->cycle_count = (cpu->cycle_count & 0xFFFFFFFF) | ((uint64_t)val << 32); break;
    case CSR_MINSTRET:  cpu->instret_count = (cpu->instret_count & 0xFFFFFFFF00000000ULL) | val; break;
    case CSR_MINSTRETH: cpu->instret_count = (cpu->instret_count & 0xFFFFFFFF) | ((uint64_t)val << 32); break;
    case CSR_MHARTID:   break;  /* Read-only */
    case CSR_MISA:      break;  /* Read-only */
    case CSR_MVENDORID: break;  /* Read-only */
    case CSR_MARCHID:   break;  /* Read-only */
    case CSR_MIMPID:    break;  /* Read-only */
    default:
        if (addr < RV_CSR_COUNT)
            cpu->csr[addr] = val;
        break;
    }
}

/* ========================================================================
 * Trap Handling
 * ======================================================================== */

void rv_trap_enter(rv_cpu_state_t *cpu, uint32_t cause, uint32_t tval) {
    /* Save current state */
    cpu->csr[CSR_MEPC] = cpu->pc;
    cpu->csr[CSR_MCAUSE] = cause;
    cpu->csr[CSR_MTVAL] = tval;

    /* Save and clear MIE */
    uint32_t mstatus = cpu->csr[CSR_MSTATUS];
    if (mstatus & MSTATUS_MIE)
        mstatus |= MSTATUS_MPIE;
    else
        mstatus &= ~MSTATUS_MPIE;
    mstatus &= ~MSTATUS_MIE;        /* Disable interrupts */
    mstatus = (mstatus & ~MSTATUS_MPP) | MSTATUS_MPP;  /* MPP = M-mode */
    cpu->csr[CSR_MSTATUS] = mstatus;

    /* Jump to trap handler */
    uint32_t mtvec = cpu->csr[CSR_MTVEC];
    uint32_t mode = mtvec & 3;
    uint32_t base = mtvec & ~3u;

    if (mode == 1 && (cause & MCAUSE_INTERRUPT_BIT)) {
        /* Vectored mode for interrupts */
        cpu->pc = base + (cause & ~MCAUSE_INTERRUPT_BIT) * 4;
    } else {
        /* Direct mode (or exceptions in vectored mode) */
        cpu->pc = base;
    }

    cpu->in_trap = 1;

    if (cpu->debug_enabled)
        fprintf(stderr, "[RV-CORE%d] TRAP: cause=0x%08X mepc=0x%08X -> handler=0x%08X\n",
                cpu->hart_id, cause, cpu->csr[CSR_MEPC], cpu->pc);
}

void rv_trap_return(rv_cpu_state_t *cpu) {
    /* Restore state from mepc */
    cpu->pc = cpu->csr[CSR_MEPC];

    /* Restore MIE from MPIE */
    uint32_t mstatus = cpu->csr[CSR_MSTATUS];
    if (mstatus & MSTATUS_MPIE)
        mstatus |= MSTATUS_MIE;
    else
        mstatus &= ~MSTATUS_MIE;
    mstatus |= MSTATUS_MPIE;   /* Set MPIE */
    cpu->csr[CSR_MSTATUS] = mstatus;

    cpu->in_trap = 0;

    if (cpu->debug_enabled)
        fprintf(stderr, "[RV-CORE%d] MRET -> PC=0x%08X\n", cpu->hart_id, cpu->pc);
}

/* ========================================================================
 * Write-back helper (x0 is hardwired to zero)
 * ======================================================================== */

static inline void rv_write_rd(rv_cpu_state_t *cpu, uint32_t rd, uint32_t val) {
    if (rd != 0) cpu->x[rd] = val;
}

/* ========================================================================
 * RV32I Instruction Execution
 * ======================================================================== */

int rv_cpu_step(rv_cpu_state_t *cpu) {
    if (cpu->is_halted) return -1;

    uint32_t pc = cpu->pc;

    /* Fetch instruction (supports both 32-bit and 16-bit compressed) */
    uint32_t instr;
    uint16_t lo = mem_read16(pc);

    int is_compressed = ((lo & 3) != 3);
    if (is_compressed) {
        /* ============================================================
         * RV32C Compressed Instructions (16-bit)
         *
         * Quadrant 0: bits[1:0] = 00
         * Quadrant 1: bits[1:0] = 01
         * Quadrant 2: bits[1:0] = 10
         * ============================================================ */
        uint16_t ci = lo;
        uint32_t quad = ci & 3;
        uint32_t funct3c = (ci >> 13) & 7;

        if (cpu->debug_enabled)
            fprintf(stderr, "[RV-CORE%d] PC=0x%08X C-instr=0x%04X quad=%u f3=%u\n",
                    cpu->hart_id, pc, ci, quad, funct3c);

        /* Compressed register mapping: r' = x(8 + r') for 3-bit reg fields */
        #define CRD_P(ci)   (8 + (((ci) >> 2) & 7))
        #define CRS2_P(ci)  (8 + (((ci) >> 2) & 7))
        #define CRS1_P(ci)  (8 + (((ci) >> 7) & 7))

        /* Full 5-bit register fields */
        #define CRD(ci)     (((ci) >> 7) & 0x1F)
        #define CRS2(ci)    (((ci) >> 2) & 0x1F)

        switch (quad) {
        case 0: /* Quadrant 0 */
            switch (funct3c) {
            case 0: { /* C.ADDI4SPN: addi rd', x2, nzuimm */
                uint32_t nzuimm = ((ci >> 1) & 0x3C0) | ((ci >> 7) & 0x30)
                                | ((ci >> 2) & 0x8) | ((ci >> 4) & 0x4);
                if (nzuimm == 0) goto c_illegal;
                rv_write_rd(cpu, CRD_P(ci), cpu->x[2] + nzuimm);
                break;
            }
            case 2: { /* C.LW: lw rd', offset(rs1') */
                uint32_t off = ((ci >> 7) & 0x38) | ((ci >> 4) & 0x4) | ((ci << 1) & 0x40);
                uint32_t addr = cpu->x[CRS1_P(ci)] + off;
                if (addr & 3) { rv_trap_enter(cpu, MCAUSE_LOAD_MISALIGNED, addr); return 0; }
                rv_write_rd(cpu, CRD_P(ci), mem_read32(addr));
                break;
            }
            case 6: { /* C.SW: sw rs2', offset(rs1') */
                uint32_t off = ((ci >> 7) & 0x38) | ((ci >> 4) & 0x4) | ((ci << 1) & 0x40);
                uint32_t addr = cpu->x[CRS1_P(ci)] + off;
                if (addr & 3) { rv_trap_enter(cpu, MCAUSE_STORE_MISALIGNED, addr); return 0; }
                mem_write32(addr, cpu->x[CRD_P(ci)]);
                break;
            }
            default: goto c_illegal;
            }
            break;

        case 1: /* Quadrant 1 */
            switch (funct3c) {
            case 0: { /* C.ADDI / C.NOP */
                uint32_t rd = CRD(ci);
                int32_t imm = (int32_t)(((ci >> 7) & 0x20) | ((ci >> 2) & 0x1F));
                if (imm & 0x20) imm |= (int32_t)0xFFFFFFC0; /* sign-extend 6-bit */
                if (rd != 0) cpu->x[rd] = cpu->x[rd] + (uint32_t)imm;
                break;
            }
            case 1: { /* C.JAL (RV32 only): jal x1, offset */
                int32_t off = (int32_t)(
                    ((ci >> 1) & 0x800) | ((ci >> 7) & 0x10) | ((ci >> 1) & 0x300)
                    | ((ci << 2) & 0x400) | ((ci >> 1) & 0x40) | ((ci << 1) & 0x80)
                    | ((ci >> 2) & 0xE) | ((ci << 3) & 0x20));
                if (off & 0x800) off |= (int32_t)0xFFFFF000;
                cpu->x[1] = pc + 2;
                cpu->pc = pc + (uint32_t)off;
                cpu->x[0] = 0;
                cpu->step_count++; cpu->cycle_count++; cpu->instret_count++;
                return 0;
            }
            case 2: { /* C.LI: addi rd, x0, imm */
                uint32_t rd = CRD(ci);
                int32_t imm = (int32_t)(((ci >> 7) & 0x20) | ((ci >> 2) & 0x1F));
                if (imm & 0x20) imm |= (int32_t)0xFFFFFFC0;
                rv_write_rd(cpu, rd, (uint32_t)imm);
                break;
            }
            case 3: { /* C.ADDI16SP / C.LUI */
                uint32_t rd = CRD(ci);
                if (rd == 2) {
                    /* C.ADDI16SP: addi x2, x2, nzimm*16 */
                    int32_t imm = (int32_t)(
                        ((ci >> 3) & 0x200) | ((ci >> 2) & 0x10)
                        | ((ci << 1) & 0x40) | ((ci << 4) & 0x180)
                        | ((ci << 3) & 0x20));
                    if (imm & 0x200) imm |= (int32_t)0xFFFFFC00;
                    if (imm == 0) goto c_illegal;
                    cpu->x[2] = cpu->x[2] + (uint32_t)imm;
                } else if (rd != 0) {
                    /* C.LUI: lui rd, nzimm */
                    int32_t imm = (int32_t)(((ci >> 7) & 0x20) | ((ci >> 2) & 0x1F));
                    if (imm & 0x20) imm |= (int32_t)0xFFFFFFC0;
                    if (imm == 0) goto c_illegal;
                    rv_write_rd(cpu, rd, (uint32_t)(imm << 12));
                }
                break;
            }
            case 4: { /* Misc ALU: C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND */
                uint32_t funct2 = (ci >> 10) & 3;
                uint32_t rd = CRS1_P(ci);
                uint32_t shamt = ((ci >> 7) & 0x20) | ((ci >> 2) & 0x1F);
                switch (funct2) {
                case 0: /* C.SRLI */
                    cpu->x[rd] = cpu->x[rd] >> (shamt & 0x1F);
                    break;
                case 1: /* C.SRAI */
                    cpu->x[rd] = (uint32_t)((int32_t)cpu->x[rd] >> (shamt & 0x1F));
                    break;
                case 2: { /* C.ANDI */
                    int32_t imm = (int32_t)(((ci >> 7) & 0x20) | ((ci >> 2) & 0x1F));
                    if (imm & 0x20) imm |= (int32_t)0xFFFFFFC0;
                    cpu->x[rd] = cpu->x[rd] & (uint32_t)imm;
                    break;
                }
                case 3: { /* C.SUB/XOR/OR/AND */
                    uint32_t rs2 = CRD_P(ci);
                    uint32_t funct1 = (ci >> 12) & 1;
                    uint32_t funct2b = (ci >> 5) & 3;
                    if (funct1 == 0) {
                        switch (funct2b) {
                        case 0: cpu->x[rd] = cpu->x[rd] - cpu->x[rs2]; break; /* C.SUB */
                        case 1: cpu->x[rd] = cpu->x[rd] ^ cpu->x[rs2]; break; /* C.XOR */
                        case 2: cpu->x[rd] = cpu->x[rd] | cpu->x[rs2]; break; /* C.OR */
                        case 3: cpu->x[rd] = cpu->x[rd] & cpu->x[rs2]; break; /* C.AND */
                        }
                    } else {
                        goto c_illegal;
                    }
                    break;
                }
                }
                break;
            }
            case 5: { /* C.J: jal x0, offset */
                int32_t off = (int32_t)(
                    ((ci >> 1) & 0x800) | ((ci >> 7) & 0x10) | ((ci >> 1) & 0x300)
                    | ((ci << 2) & 0x400) | ((ci >> 1) & 0x40) | ((ci << 1) & 0x80)
                    | ((ci >> 2) & 0xE) | ((ci << 3) & 0x20));
                if (off & 0x800) off |= (int32_t)0xFFFFF000;
                cpu->pc = pc + (uint32_t)off;
                cpu->x[0] = 0;
                cpu->step_count++; cpu->cycle_count++; cpu->instret_count++;
                return 0;
            }
            case 6: { /* C.BEQZ: beq rs1', x0, offset */
                int32_t off = (int32_t)(
                    ((ci >> 4) & 0x100) | ((ci >> 7) & 0x18)
                    | ((ci << 1) & 0x0C0) | ((ci >> 2) & 0x6)
                    | ((ci << 3) & 0x20));
                if (off & 0x100) off |= (int32_t)0xFFFFFE00;
                if (cpu->x[CRS1_P(ci)] == 0) {
                    cpu->pc = pc + (uint32_t)off;
                    cpu->x[0] = 0;
                    cpu->step_count++; cpu->cycle_count++; cpu->instret_count++;
                    return 0;
                }
                break;
            }
            case 7: { /* C.BNEZ: bne rs1', x0, offset */
                int32_t off = (int32_t)(
                    ((ci >> 4) & 0x100) | ((ci >> 7) & 0x18)
                    | ((ci << 1) & 0x0C0) | ((ci >> 2) & 0x6)
                    | ((ci << 3) & 0x20));
                if (off & 0x100) off |= (int32_t)0xFFFFFE00;
                if (cpu->x[CRS1_P(ci)] != 0) {
                    cpu->pc = pc + (uint32_t)off;
                    cpu->x[0] = 0;
                    cpu->step_count++; cpu->cycle_count++; cpu->instret_count++;
                    return 0;
                }
                break;
            }
            default: goto c_illegal;
            }
            break;

        case 2: /* Quadrant 2 */
            switch (funct3c) {
            case 0: { /* C.SLLI */
                uint32_t rd = CRD(ci);
                uint32_t shamt = ((ci >> 7) & 0x20) | ((ci >> 2) & 0x1F);
                if (rd != 0) cpu->x[rd] = cpu->x[rd] << (shamt & 0x1F);
                break;
            }
            case 2: { /* C.LWSP: lw rd, offset(x2) */
                uint32_t rd = CRD(ci);
                if (rd == 0) goto c_illegal;
                uint32_t off = ((ci >> 7) & 0x20) | ((ci >> 2) & 0x1C) | ((ci << 4) & 0xC0);
                uint32_t addr = cpu->x[2] + off;
                if (addr & 3) { rv_trap_enter(cpu, MCAUSE_LOAD_MISALIGNED, addr); return 0; }
                cpu->x[rd] = mem_read32(addr);
                break;
            }
            case 4: { /* C.MV / C.ADD / C.JR / C.JALR / C.EBREAK */
                uint32_t rd = CRD(ci);
                uint32_t rs2 = CRS2(ci);
                int bit12 = (ci >> 12) & 1;
                if (bit12 == 0) {
                    if (rs2 == 0) {
                        /* C.JR: jalr x0, rs1, 0 */
                        if (rd == 0) goto c_illegal;
                        cpu->pc = cpu->x[rd] & ~1u;
                        cpu->x[0] = 0;
                        cpu->step_count++; cpu->cycle_count++; cpu->instret_count++;
                        return 0;
                    } else {
                        /* C.MV: add rd, x0, rs2 */
                        rv_write_rd(cpu, rd, cpu->x[rs2]);
                    }
                } else {
                    if (rs2 == 0 && rd == 0) {
                        /* C.EBREAK */
                        rv_trap_enter(cpu, MCAUSE_BREAKPOINT, pc);
                        return 0;
                    } else if (rs2 == 0) {
                        /* C.JALR: jalr x1, rs1, 0 */
                        uint32_t target = cpu->x[rd] & ~1u;
                        cpu->x[1] = pc + 2;
                        cpu->pc = target;
                        cpu->x[0] = 0;
                        cpu->step_count++; cpu->cycle_count++; cpu->instret_count++;
                        return 0;
                    } else {
                        /* C.ADD: add rd, rd, rs2 */
                        if (rd != 0) cpu->x[rd] = cpu->x[rd] + cpu->x[rs2];
                    }
                }
                break;
            }
            case 6: { /* C.SWSP: sw rs2, offset(x2) */
                uint32_t off = ((ci >> 7) & 0x3C) | ((ci >> 1) & 0xC0);
                uint32_t addr = cpu->x[2] + off;
                if (addr & 3) { rv_trap_enter(cpu, MCAUSE_STORE_MISALIGNED, addr); return 0; }
                mem_write32(addr, cpu->x[CRS2(ci)]);
                break;
            }
            default: goto c_illegal;
            }
            break;

        default:
        c_illegal:
            if (cpu->debug_enabled)
                fprintf(stderr, "[RV-CORE%d] ILLEGAL C-INSTRUCTION at PC=0x%08X: 0x%04X\n",
                        cpu->hart_id, pc, ci);
            rv_trap_enter(cpu, MCAUSE_ILLEGAL_INSTR, ci);
            return 0;
        }

        #undef CRD_P
        #undef CRS2_P
        #undef CRS1_P
        #undef CRD
        #undef CRS2

        /* Compressed instructions advance PC by 2 */
        cpu->pc = pc + 2;
        cpu->x[0] = 0;
        cpu->step_count++;
        cpu->cycle_count++;
        cpu->instret_count++;
        return 0;
    }

    /* 32-bit instruction */
    uint16_t hi = mem_read16(pc + 2);
    instr = (uint32_t)lo | ((uint32_t)hi << 16);

    uint32_t opcode = RV_OPCODE(instr);
    uint32_t rd     = RV_RD(instr);
    uint32_t funct3 = RV_FUNCT3(instr);
    uint32_t rs1    = RV_RS1(instr);
    uint32_t rs2    = RV_RS2(instr);
    uint32_t funct7 = RV_FUNCT7(instr);

    uint32_t next_pc = pc + 4;

    if (cpu->debug_enabled) {
        fprintf(stderr, "[RV-CORE%d] PC=0x%08X instr=0x%08X op=0x%02X rd=%u rs1=%u rs2=%u f3=%u f7=0x%02X\n",
                cpu->hart_id, pc, instr, opcode, rd, rs1, rs2, funct3, funct7);
    }

    switch (opcode) {

    /* ================================================================
     * LUI: rd = imm_u
     * ================================================================ */
    case OP_LUI:
        rv_write_rd(cpu, rd, (uint32_t)rv_imm_u(instr));
        break;

    /* ================================================================
     * AUIPC: rd = pc + imm_u
     * ================================================================ */
    case OP_AUIPC:
        rv_write_rd(cpu, rd, pc + (uint32_t)rv_imm_u(instr));
        break;

    /* ================================================================
     * JAL: rd = pc + 4; pc = pc + imm_j
     * ================================================================ */
    case OP_JAL:
        rv_write_rd(cpu, rd, pc + 4);
        next_pc = pc + (uint32_t)rv_imm_j(instr);
        if (next_pc & 1) {
            rv_trap_enter(cpu, MCAUSE_INSTR_MISALIGNED, next_pc);
            return 0;
        }
        break;

    /* ================================================================
     * JALR: rd = pc + 4; pc = (rs1 + imm_i) & ~1
     * ================================================================ */
    case OP_JALR:
        if (funct3 != 0) goto illegal;
        {
            uint32_t target = (cpu->x[rs1] + (uint32_t)rv_imm_i(instr)) & ~1u;
            rv_write_rd(cpu, rd, pc + 4);
            next_pc = target;
        }
        break;

    /* ================================================================
     * Branches: BEQ, BNE, BLT, BGE, BLTU, BGEU
     * ================================================================ */
    case OP_BRANCH: {
        int32_t a = (int32_t)cpu->x[rs1];
        int32_t b = (int32_t)cpu->x[rs2];
        uint32_t ua = cpu->x[rs1];
        uint32_t ub = cpu->x[rs2];
        int taken = 0;

        switch (funct3) {
        case 0: taken = (ua == ub); break;  /* BEQ */
        case 1: taken = (ua != ub); break;  /* BNE */
        case 4: taken = (a < b);    break;  /* BLT */
        case 5: taken = (a >= b);   break;  /* BGE */
        case 6: taken = (ua < ub);  break;  /* BLTU */
        case 7: taken = (ua >= ub); break;  /* BGEU */
        default: goto illegal;
        }

        if (taken) {
            next_pc = pc + (uint32_t)rv_imm_b(instr);
            if (next_pc & 1) {
                rv_trap_enter(cpu, MCAUSE_INSTR_MISALIGNED, next_pc);
                return 0;
            }
        }
        break;
    }

    /* ================================================================
     * Loads: LB, LH, LW, LBU, LHU
     * ================================================================ */
    case OP_LOAD: {
        uint32_t addr = cpu->x[rs1] + (uint32_t)rv_imm_i(instr);
        uint32_t val;

        switch (funct3) {
        case 0: /* LB */
            val = (uint32_t)(int32_t)(int8_t)mem_read8(addr);
            break;
        case 1: /* LH */
            if (addr & 1) { rv_trap_enter(cpu, MCAUSE_LOAD_MISALIGNED, addr); return 0; }
            val = (uint32_t)(int32_t)(int16_t)mem_read16(addr);
            break;
        case 2: /* LW */
            if (addr & 3) { rv_trap_enter(cpu, MCAUSE_LOAD_MISALIGNED, addr); return 0; }
            val = mem_read32(addr);
            break;
        case 4: /* LBU */
            val = (uint32_t)mem_read8(addr);
            break;
        case 5: /* LHU */
            if (addr & 1) { rv_trap_enter(cpu, MCAUSE_LOAD_MISALIGNED, addr); return 0; }
            val = (uint32_t)mem_read16(addr);
            break;
        default: goto illegal;
        }

        rv_write_rd(cpu, rd, val);
        break;
    }

    /* ================================================================
     * Stores: SB, SH, SW
     * ================================================================ */
    case OP_STORE: {
        uint32_t addr = cpu->x[rs1] + (uint32_t)rv_imm_s(instr);
        uint32_t val = cpu->x[rs2];

        switch (funct3) {
        case 0: /* SB */
            mem_write8(addr, (uint8_t)val);
            break;
        case 1: /* SH */
            if (addr & 1) { rv_trap_enter(cpu, MCAUSE_STORE_MISALIGNED, addr); return 0; }
            mem_write16(addr, (uint16_t)val);
            break;
        case 2: /* SW */
            if (addr & 3) { rv_trap_enter(cpu, MCAUSE_STORE_MISALIGNED, addr); return 0; }
            mem_write32(addr, val);
            break;
        default: goto illegal;
        }
        break;
    }

    /* ================================================================
     * ALU immediate: ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
     * ================================================================ */
    case OP_IMM: {
        int32_t imm = rv_imm_i(instr);
        uint32_t src = cpu->x[rs1];
        uint32_t result;

        switch (funct3) {
        case 0: result = src + (uint32_t)imm; break;                    /* ADDI */
        case 2: result = ((int32_t)src < imm) ? 1 : 0; break;          /* SLTI */
        case 3: result = (src < (uint32_t)imm) ? 1 : 0; break;         /* SLTIU */
        case 4: result = src ^ (uint32_t)imm; break;                    /* XORI */
        case 6: result = src | (uint32_t)imm; break;                    /* ORI */
        case 7: result = src & (uint32_t)imm; break;                    /* ANDI */
        case 1: /* SLLI */
            if (funct7 != 0) goto illegal;
            result = src << (rs2 & 0x1F);
            break;
        case 5: /* SRLI / SRAI */
            if (funct7 == 0x00)
                result = src >> (rs2 & 0x1F);               /* SRLI */
            else if (funct7 == 0x20)
                result = (uint32_t)((int32_t)src >> (rs2 & 0x1F));  /* SRAI */
            else goto illegal;
            break;
        default: goto illegal;
        }

        rv_write_rd(cpu, rd, result);
        break;
    }

    /* ================================================================
     * ALU register: ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
     * (Also M extension: MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU)
     * ================================================================ */
    case OP_REG: {
        uint32_t a = cpu->x[rs1];
        uint32_t b = cpu->x[rs2];
        uint32_t result;

        if (funct7 == 0x01) {
            /* M extension (Phase 2) */
            switch (funct3) {
            case 0: result = a * b; break;                              /* MUL */
            case 1: result = (uint32_t)((int64_t)(int32_t)a * (int64_t)(int32_t)b >> 32); break; /* MULH */
            case 2: result = (uint32_t)((int64_t)(int32_t)a * (uint64_t)b >> 32); break;  /* MULHSU */
            case 3: result = (uint32_t)((uint64_t)a * (uint64_t)b >> 32); break;          /* MULHU */
            case 4: /* DIV */
                if (b == 0) result = 0xFFFFFFFF;
                else if ((int32_t)a == INT32_MIN && (int32_t)b == -1) result = a;
                else result = (uint32_t)((int32_t)a / (int32_t)b);
                break;
            case 5: /* DIVU */
                result = (b == 0) ? 0xFFFFFFFF : a / b;
                break;
            case 6: /* REM */
                if (b == 0) result = a;
                else if ((int32_t)a == INT32_MIN && (int32_t)b == -1) result = 0;
                else result = (uint32_t)((int32_t)a % (int32_t)b);
                break;
            case 7: /* REMU */
                result = (b == 0) ? a : a % b;
                break;
            default: goto illegal;
            }
        } else if (funct7 == 0x00) {
            switch (funct3) {
            case 0: result = a + b; break;                              /* ADD */
            case 1: result = a << (b & 0x1F); break;                   /* SLL */
            case 2: result = ((int32_t)a < (int32_t)b) ? 1 : 0; break; /* SLT */
            case 3: result = (a < b) ? 1 : 0; break;                   /* SLTU */
            case 4: result = a ^ b; break;                              /* XOR */
            case 5: result = a >> (b & 0x1F); break;                    /* SRL */
            case 6: result = a | b; break;                              /* OR */
            case 7: result = a & b; break;                              /* AND */
            default: goto illegal;
            }
        } else if (funct7 == 0x20) {
            switch (funct3) {
            case 0: result = a - b; break;                              /* SUB */
            case 5: result = (uint32_t)((int32_t)a >> (b & 0x1F)); break; /* SRA */
            default: goto illegal;
            }
        } else {
            goto illegal;
        }

        rv_write_rd(cpu, rd, result);
        break;
    }

    /* ================================================================
     * FENCE: memory ordering (NOP in emulator)
     * ================================================================ */
    case OP_FENCE:
        /* No reordering in emulator — treat as NOP */
        break;

    /* ================================================================
     * SYSTEM: ECALL, EBREAK, MRET, WFI, CSR instructions
     * ================================================================ */
    case OP_SYSTEM:
        if (funct3 == 0) {
            /* PRIV instructions encoded in imm_i */
            uint32_t funct12 = (instr >> 20) & 0xFFF;
            switch (funct12) {
            case 0x000: /* ECALL */
                rv_trap_enter(cpu, MCAUSE_ECALL_M, 0);
                return 0;
            case 0x001: /* EBREAK */
                rv_trap_enter(cpu, MCAUSE_BREAKPOINT, pc);
                return 0;
            case 0x302: /* MRET */
                rv_trap_return(cpu);
                return 0;
            case 0x105: /* WFI */
                cpu->is_wfi = 1;
                break;
            default:
                goto illegal;
            }
        } else {
            /* CSR instructions */
            uint16_t csr_addr = (uint16_t)((instr >> 20) & 0xFFF);
            uint32_t old_val = rv_csr_read(cpu, csr_addr);
            uint32_t new_val;

            switch (funct3) {
            case 1: /* CSRRW */
                new_val = cpu->x[rs1];
                rv_csr_write(cpu, csr_addr, new_val);
                rv_write_rd(cpu, rd, old_val);
                break;
            case 2: /* CSRRS */
                if (rs1 != 0)
                    rv_csr_write(cpu, csr_addr, old_val | cpu->x[rs1]);
                rv_write_rd(cpu, rd, old_val);
                break;
            case 3: /* CSRRC */
                if (rs1 != 0)
                    rv_csr_write(cpu, csr_addr, old_val & ~cpu->x[rs1]);
                rv_write_rd(cpu, rd, old_val);
                break;
            case 5: /* CSRRWI */
                rv_csr_write(cpu, csr_addr, rs1);  /* rs1 field is zimm */
                rv_write_rd(cpu, rd, old_val);
                break;
            case 6: /* CSRRSI */
                if (rs1 != 0)
                    rv_csr_write(cpu, csr_addr, old_val | rs1);
                rv_write_rd(cpu, rd, old_val);
                break;
            case 7: /* CSRRCI */
                if (rs1 != 0)
                    rv_csr_write(cpu, csr_addr, old_val & ~rs1);
                rv_write_rd(cpu, rd, old_val);
                break;
            default:
                goto illegal;
            }
        }
        break;

    default:
        goto illegal;
    }

    /* Advance PC and counters */
    cpu->pc = next_pc;
    cpu->x[0] = 0;  /* x0 always zero */
    cpu->step_count++;
    cpu->cycle_count++;
    cpu->instret_count++;
    return 0;

illegal:
    if (cpu->debug_enabled)
        fprintf(stderr, "[RV-CORE%d] ILLEGAL INSTRUCTION at PC=0x%08X: 0x%08X\n",
                cpu->hart_id, pc, instr);
    rv_trap_enter(cpu, MCAUSE_ILLEGAL_INSTR, instr);
    return 0;
}
