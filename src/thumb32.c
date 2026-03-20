/*
 * Thumb-2 32-bit instruction handler for ARMv7-M compatibility
 *
 * Handles instruction groups:
 *   0xE8xx/0xE9xx: Load/Store Multiple T2 (LDMIA/STMIA/LDMDB/STMDB)
 *   0xEAxx/0xEBxx: Data Processing (shifted register)
 *   0xF0xx-0xF7xx: Data Processing (modified/plain immediate) + wide branches
 *   0xF8xx-0xFFxx: Load/Store single (all widths and modes)
 *
 * Also handles BL T1, MSR, MRS, DSB/DMB/ISB (previously in cpu.c).
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "emulator.h"
#include "instructions.h"
#include "nvic.h"
#include "thumb32.h"
#include "devtools.h"

/* External globals from cpu.c */
extern int pc_updated;

/* Flag bits in XPSR */
#define FLAG_N 0x80000000u
#define FLAG_Z 0x40000000u
#define FLAG_C 0x20000000u
#define FLAG_V 0x10000000u

/* Evaluate ARM condition code against current XPSR flags */
static int t32_check_cond(uint8_t cond) {
    int N = (cpu.xpsr & FLAG_N) != 0;
    int Z = (cpu.xpsr & FLAG_Z) != 0;
    int C = (cpu.xpsr & FLAG_C) != 0;
    int V = (cpu.xpsr & FLAG_V) != 0;
    switch (cond & 0xF) {
    case 0x0: return  Z;
    case 0x1: return !Z;
    case 0x2: return  C;
    case 0x3: return !C;
    case 0x4: return  N;
    case 0x5: return !N;
    case 0x6: return  V;
    case 0x7: return !V;
    case 0x8: return  C && !Z;
    case 0x9: return !C ||  Z;
    case 0xA: return  N == V;
    case 0xB: return  N != V;
    case 0xC: return !Z && (N == V);
    case 0xD: return  Z || (N != V);
    case 0xE: return 1;  /* AL */
    default:  return 1;
    }
}

/* ========================================================================
 * Helper: Thumb-2 modified 12-bit immediate expansion
 * ======================================================================== */
static uint32_t thumb32_expand_imm(uint32_t imm12) {
    uint32_t b = imm12 & 0xFF;
    switch (imm12 >> 8) {
    case 0: return b;
    case 1: return (b << 16) | b;
    case 2: return (b << 24) | (b << 8);
    case 3: return (b << 24) | (b << 16) | (b << 8) | b;
    default: {
        /* Rotation: bit[7] is always 1 in the unrotated value */
        uint32_t val = 0x80 | (imm12 & 0x7F);
        uint32_t rot = (imm12 >> 7) & 0x1F;
        return (rot == 0) ? val : ((val >> rot) | (val << (32 - rot)));
    }
    }
}

/* ========================================================================
 * Helper: Barrel shifter with carry out
 * type: 0=LSL, 1=LSR, 2=ASR, 3=ROR
 * ======================================================================== */
static uint32_t t32_shift_c(uint32_t val, int type, int n, int *carry_out) {
    if (n == 0) {
        *carry_out = (cpu.xpsr & FLAG_C) ? 1 : 0;
        return val;
    }
    switch (type) {
    case 0: /* LSL */
        if (n >= 32) { *carry_out = (n == 32) ? (val & 1) : 0; return 0; }
        *carry_out = (val >> (32 - n)) & 1;
        return val << n;
    case 1: /* LSR */
        if (n > 32) { *carry_out = 0; return 0; }
        if (n == 32) { *carry_out = val >> 31; return 0; }
        *carry_out = (val >> (n - 1)) & 1;
        return val >> n;
    case 2: /* ASR */
        if (n >= 32) {
            *carry_out = val >> 31;
            return (val & 0x80000000u) ? 0xFFFFFFFFu : 0;
        }
        *carry_out = (val >> (n - 1)) & 1;
        return (uint32_t)((int32_t)val >> n);
    case 3: /* ROR */
        n &= 31;
        if (n == 0) { *carry_out = val >> 31; return val; }
        *carry_out = (val >> (n - 1)) & 1;
        return (val >> n) | (val << (32 - n));
    }
    *carry_out = 0;
    return val;
}

/* ========================================================================
 * Helper: Data processing ALU op (shared by shifted-reg and mod-imm)
 * op: AND=0,BIC=1,ORR=2,ORN=3,EOR=4,ADD=8,ADC=10,SBC=11,SUB=13,RSB=14
 * ======================================================================== */
static void t32_dp_exec(int op, int S, int Rn, int Rd, uint32_t imm32, int shift_c) {
    uint32_t rn  = cpu.r[Rn];
    uint32_t res = 0;
    int      wr  = 1;  /* write result to Rd */

    switch (op) {
    case 0x0: /* AND / TST (Rd=15,S) */
        res = rn & imm32;
        if (Rd == 15 && S) wr = 0;
        if (S) { update_nz_flags(res); if (shift_c) cpu.xpsr |= FLAG_C; else cpu.xpsr &= ~FLAG_C; }
        break;
    case 0x1: /* BIC */
        res = rn & ~imm32;
        if (S) { update_nz_flags(res); if (shift_c) cpu.xpsr |= FLAG_C; else cpu.xpsr &= ~FLAG_C; }
        break;
    case 0x2: /* ORR / MOV (Rn=15) */
        res = (Rn == 15) ? imm32 : (rn | imm32);
        if (S) { update_nz_flags(res); if (shift_c) cpu.xpsr |= FLAG_C; else cpu.xpsr &= ~FLAG_C; }
        break;
    case 0x3: /* ORN / MVN (Rn=15) */
        res = (Rn == 15) ? ~imm32 : (rn | ~imm32);
        if (S) { update_nz_flags(res); if (shift_c) cpu.xpsr |= FLAG_C; else cpu.xpsr &= ~FLAG_C; }
        break;
    case 0x4: /* EOR / TEQ (Rd=15,S) */
        res = rn ^ imm32;
        if (Rd == 15 && S) wr = 0;
        if (S) { update_nz_flags(res); if (shift_c) cpu.xpsr |= FLAG_C; else cpu.xpsr &= ~FLAG_C; }
        break;
    case 0x8: { /* ADD / CMN (Rd=15,S) */
        res = rn + imm32;
        if (Rd == 15 && S) wr = 0;
        if (S) update_add_flags(rn, imm32, res);
        break;
    }
    case 0xA: { /* ADC */
        uint32_t c = (cpu.xpsr & FLAG_C) ? 1u : 0u;
        uint64_t r64 = (uint64_t)rn + imm32 + c;
        res = (uint32_t)r64;
        if (S) {
            update_add_flags(rn, imm32 + c, res);
            cpu.xpsr &= ~FLAG_C;
            if (r64 > 0xFFFFFFFFULL) cpu.xpsr |= FLAG_C;
        }
        break;
    }
    case 0xB: { /* SBC */
        uint32_t c = (cpu.xpsr & FLAG_C) ? 1u : 0u;
        res = rn - imm32 - (1u - c);
        if (S) update_sub_flags(rn, imm32, res);
        break;
    }
    case 0xD: /* SUB / CMP (Rd=15,S) */
        res = rn - imm32;
        if (Rd == 15 && S) wr = 0;
        if (S) update_sub_flags(rn, imm32, res);
        break;
    case 0xE: /* RSB */
        res = imm32 - rn;
        if (S) update_sub_flags(imm32, rn, res);
        break;
    default:
        if (cpu.debug_enabled)
            fprintf(stderr, "[T32] Unknown dp op=0x%X @ PC=0x%08X\n", op, cpu.r[15]);
        cpu_exception_entry(EXC_HARDFAULT);
        return;
    }
    if (wr && Rd < 16) {
        if (Rd == 15) { cpu.r[15] = res & ~1u; pc_updated = 1; }
        else           { cpu.r[Rd] = res; }
    }
}

/* ========================================================================
 * BL T1 (already handled in cpu.c but also handled here for completeness)
 * upper: 1111 0 S imm10
 * lower: 1 1 J1 1 J2 imm11
 * ======================================================================== */
static void t32_bl(uint32_t pc, uint16_t upper, uint16_t lower) {
    uint32_t S     = (upper >> 10) & 1;
    uint32_t imm10 = upper & 0x3FF;
    uint32_t J1    = (lower >> 13) & 1;
    uint32_t J2    = (lower >> 11) & 1;
    uint32_t imm11 = lower & 0x7FF;
    uint32_t I1    = !(J1 ^ S) & 1;
    uint32_t I2    = !(J2 ^ S) & 1;
    int32_t  offset = (int32_t)((S << 24) | (I1 << 23) | (I2 << 22) | (imm10 << 12) | (imm11 << 1));
    if (S) offset |= (int32_t)0xFF000000;  /* sign-extend from bit 24 */
    cpu.r[14] = (pc + 4) | 1u;
    uint32_t target = (uint32_t)((int32_t)(pc + 4) + offset);
    cpu.r[15] = target;
    pc_updated = 1;
    if (__builtin_expect(callgraph_enabled, 0))
        callgraph_record_call(pc, target);
}

/* ========================================================================
 * Load/Store Multiple T2
 * upper: 1110 1000 W L Rn(4)   (IA: bit8=0)
 *        1110 1001 W L Rn(4)   (DB: bit8=1)
 * lower: 0 P M 0 reglist(12)   (bit15=P=PC, bit14=M=LR)
 * ======================================================================== */
static void t32_ldst_multiple(uint32_t pc, uint16_t upper, uint16_t lower) {
    (void)pc;
    int is_db = (upper >> 8) & 1;
    int L     = (upper >> 7) & 1;
    int W     = (upper >> 5) & 1;
    int Rn    = upper & 0xF;

    uint32_t reglist = lower;   /* bit15=PC, bit14=LR, bits[13:0] = R13..R0 */
    int      cnt     = __builtin_popcount(reglist & 0xFFFF);
    uint32_t addr    = cpu.r[Rn];

    if (is_db) addr -= (uint32_t)(cnt * 4);
    uint32_t base_end = addr + (uint32_t)(cnt * 4);

    for (int i = 0; i <= 15; i++) {
        if (reglist & (1u << i)) {
            if (L) {
                uint32_t val = mem_read32(addr);
                if (i == 15) { cpu.r[15] = val & ~1u; pc_updated = 1; }
                else          { cpu.r[i] = val; }
            } else {
                uint32_t val = (i == 15) ? (pc + 4) : cpu.r[i];
                mem_write32(addr, val);
            }
            addr += 4;
        }
    }

    if (W && !(L && (reglist & (1u << Rn)))) {
        /* Writeback: IA → updated addr, DB → original - count*4 */
        cpu.r[Rn] = is_db ? (cpu.r[Rn] - (uint32_t)(cnt * 4)) : base_end;
    }
}

/* ========================================================================
 * Load/Store Double
 * upper: 1110 1101 W L Rn   (load/store with imm8, P/U from lower)
 * upper pattern: (upper & 0xFE50) == 0xE840 → STRD, 0xE850 → LDRD
 * lower: Rt(4) Rt2(4) P U W imm8
 * Actually: upper = 1110 1 0 P U 1 W 0 L Rn
 * ======================================================================== */
static void t32_ldrd_strd(uint32_t pc, uint16_t upper, uint16_t lower) {
    (void)pc;
    int P   = (upper >> 8) & 1;
    int U   = (upper >> 7) & 1;
    int W   = (upper >> 5) & 1;
    int L   = (upper >> 4) & 1;
    int Rn  = upper & 0xF;
    int Rt  = (lower >> 12) & 0xF;
    int Rt2 = (lower >> 8) & 0xF;
    int imm8 = (lower & 0xFF) << 2;

    uint32_t base = cpu.r[Rn];
    uint32_t offset_addr = U ? (base + imm8) : (base - imm8);
    uint32_t addr = P ? offset_addr : base;

    if (L) {
        cpu.r[Rt]  = mem_read32(addr);
        cpu.r[Rt2] = mem_read32(addr + 4);
    } else {
        mem_write32(addr,     cpu.r[Rt]);
        mem_write32(addr + 4, cpu.r[Rt2]);
    }
    if (W) cpu.r[Rn] = offset_addr;
}

/* ========================================================================
 * Table Branch
 * upper: 1110 1000 1101 Rn   → TBB/TBH
 * lower: 1111 0000 H Rm
 * ======================================================================== */
static void t32_tbb_tbh(uint32_t pc, uint16_t upper, uint16_t lower) {
    int Rn = upper & 0xF;
    int H  = (lower >> 4) & 1;
    int Rm = lower & 0xF;
    uint32_t base  = (Rn == 15) ? (pc + 4) : cpu.r[Rn];
    uint32_t index = cpu.r[Rm];
    uint32_t addr  = base + (H ? index * 2 : index);
    uint32_t offset;
    if (H) offset = mem_read16(addr) * 2u;
    else   offset = mem_read8(addr) * 2u;
    cpu.r[15] = (pc + 4) + offset;
    pc_updated = 1;
}

/* ========================================================================
 * Data Processing: shifted register
 * upper: 1110 1010 op(4) S Rn(4)
 *        1110 1011 op(4) S Rn(4)   (op has bit3 set)
 * lower: 0 imm3(3) Rd(4) imm2(2) type(2) Rm(4)
 * ======================================================================== */
static void t32_dp_shifted_reg(uint32_t pc, uint16_t upper, uint16_t lower) {
    (void)pc;
    int      op    = (upper >> 5) & 0xF;
    int      S     = (upper >> 4) & 1;
    int      Rn    = upper & 0xF;
    int      imm3  = (lower >> 12) & 7;
    int      Rd    = (lower >> 8) & 0xF;
    int      imm2  = (lower >> 6) & 3;
    int      type  = (lower >> 4) & 3;
    int      Rm    = lower & 0xF;
    int      n     = (imm3 << 2) | imm2;
    int      carry = 0;
    uint32_t shifted = t32_shift_c(cpu.r[Rm], type, n, &carry);
    t32_dp_exec(op, S, Rn, Rd, shifted, carry);
}

/* ========================================================================
 * Data Processing: modified immediate
 * upper: 1111 0 i op(4) S Rn(4)
 * lower: 0 imm3(3) Rd(4) imm8(8)
 * ======================================================================== */
static void t32_dp_mod_imm(uint32_t pc, uint16_t upper, uint16_t lower) {
    (void)pc;
    uint32_t i     = (upper >> 10) & 1;
    int      op    = (upper >> 5) & 0xF;
    int      S     = (upper >> 4) & 1;
    int      Rn    = upper & 0xF;
    uint32_t imm3  = (lower >> 12) & 7;
    int      Rd    = (lower >> 8) & 0xF;
    uint32_t imm8  = lower & 0xFF;
    uint32_t imm12 = (i << 11) | (imm3 << 8) | imm8;
    uint32_t imm32 = thumb32_expand_imm(imm12);
    /* carry from expansion: MSB of result for rotation case */
    int carry = (imm32 >> 31) & 1;
    t32_dp_exec(op, S, Rn, Rd, imm32, carry);
}

/* ========================================================================
 * Wide branches: Bcc.W T3 and B.W T4
 * Bcc.W T3: upper = 1111 0 S cond(4) imm6, lower = 1 0 J1 0 J2 imm11
 * B.W T4:   upper = 1111 0 S imm10,        lower = 1 0 J1 1 J2 imm11
 * ======================================================================== */
static void t32_branch(uint32_t pc, uint16_t upper, uint16_t lower) {
    uint32_t S    = (upper >> 10) & 1;
    uint32_t J1   = (lower >> 13) & 1;
    uint32_t J2   = (lower >> 11) & 1;
    uint32_t imm11 = lower & 0x7FF;

    int is_bw = (lower >> 12) & 1;  /* bit12=1 → B.W T4, bit12=0 → Bcc.W T3 */

    if (is_bw) {
        /* B.W T4: same offset decode as BL */
        uint32_t imm10 = upper & 0x3FF;
        uint32_t I1    = (!(J1 ^ S)) & 1u;
        uint32_t I2    = (!(J2 ^ S)) & 1u;
        int32_t offset = (int32_t)((S << 24) | (I1 << 23) | (I2 << 22) | (imm10 << 12) | (imm11 << 1));
        if (S) offset |= (int32_t)0xFF000000;
        cpu.r[15] = (uint32_t)((int32_t)(pc + 4) + offset);
        pc_updated = 1;
    } else {
        /* Bcc.W T3: offset from {S,J2,J1,imm6,imm11,0} */
        uint32_t imm6  = upper & 0x3F;
        uint8_t  cond  = (upper >> 6) & 0xF;
        int32_t offset = (int32_t)((S << 20) | (J2 << 19) | (J1 << 18) | (imm6 << 12) | (imm11 << 1));
        if (S) offset |= (int32_t)0xFFE00000;  /* sign extend from bit 20 */
        if (t32_check_cond(cond)) {
            cpu.r[15] = (uint32_t)((int32_t)(pc + 4) + offset);
            pc_updated = 1;
        }
        /* If condition fails, fall through (pc += 4 done by caller) */
    }
}

/* ========================================================================
 * Data Processing: plain binary immediate (MOVW, MOVT, ADDW, SUBW)
 * upper: 1111 0 i 1 op(4) 0 imm4
 * lower: 0 imm3(3) Rd(4) imm8(8)
 * ======================================================================== */
static void t32_dp_plain_imm(uint32_t pc, uint16_t upper, uint16_t lower) {
    (void)pc;
    uint32_t i    = (upper >> 10) & 1;
    uint32_t op   = (upper >> 4) & 0x1F;   /* bits[8:4] of upper */
    uint32_t imm4 = upper & 0xF;
    uint32_t imm3 = (lower >> 12) & 7;
    int      Rd   = (lower >> 8) & 0xF;
    uint32_t imm8 = lower & 0xFF;
    uint32_t Rn   = imm4;  /* For ADD/SUB plain, Rn is in upper[3:0] */

    /* MOVW T3: op[4:0] = 00100, i.e., upper bits[8:4] = 00100 = 0x04 */
    /* upper & 0x01F0 isolates bits[8:4]: */
    uint32_t op5 = (upper >> 4) & 0x1F;  /* bits[8:4] */

    if (op5 == 0x04) {
        /* MOVW T3: imm16 = imm4:i:imm3:imm8 */
        uint32_t imm16 = (imm4 << 12) | (i << 11) | (imm3 << 8) | imm8;
        cpu.r[Rd] = imm16;
        return;
    }
    if (op5 == 0x0C) {
        /* MOVT T1: top halfword = imm4:i:imm3:imm8 */
        uint32_t imm16 = (imm4 << 12) | (i << 11) | (imm3 << 8) | imm8;
        cpu.r[Rd] = (cpu.r[Rd] & 0x0000FFFF) | (imm16 << 16);
        return;
    }

    /* ADDW T4: op5 = 0x00 → ADD with imm12 (zero-extended) */
    uint32_t imm12 = (i << 11) | (imm3 << 8) | imm8;
    if (op5 == 0x00) {
        /* ADD Rd, Rn, #imm12 */
        if (Rn == 15) {
            /* ADR T3 */
            uint32_t base = (pc + 4) & ~3u;
            cpu.r[Rd] = base + imm12;
        } else {
            cpu.r[Rd] = cpu.r[Rn] + imm12;
        }
        return;
    }
    if (op5 == 0x0A) {
        /* SUB Rd, Rn, #imm12 (SUBW T4) */
        if (Rn == 15) {
            /* ADR T2 (subtract) */
            uint32_t base = (pc + 4) & ~3u;
            cpu.r[Rd] = base - imm12;
        } else {
            cpu.r[Rd] = cpu.r[Rn] - imm12;
        }
        return;
    }

    /* Unhandled plain-binary-imm op */
    if (cpu.debug_enabled)
        fprintf(stderr, "[T32] Unhandled plain-imm op5=0x%02X upper=0x%04X @ PC=0x%08X\n",
                op5, upper, pc);
    cpu_exception_entry(EXC_HARDFAULT);
    (void)op;
}

/* ========================================================================
 * Miscellaneous 32-bit: MSR, MRS, DSB/DMB/ISB, UDIV, SDIV, MLA, MLS, etc.
 * ======================================================================== */
static int t32_misc(uint32_t pc, uint16_t upper, uint16_t lower) {
    /* MSR T1: upper=0xF380|Rn, lower=0x88xx */
    if ((upper & 0xFFF0) == 0xF380 && (lower & 0xFF00) == 0x8800) {
        uint8_t rn   = upper & 0xF;
        uint8_t sysm = lower & 0xFF;
        instr_msr_32(rn, sysm);
        return 1;
    }
    /* MRS T1: upper=0xF3EF, lower=0x8Rss */
    if ((upper & 0xFFFF) == 0xF3EF && (lower & 0xF000) == 0x8000) {
        uint8_t rd   = (lower >> 8) & 0xF;
        uint8_t sysm = lower & 0xFF;
        instr_mrs_32(rd, sysm);
        return 1;
    }
    /* DSB/DMB/ISB: upper=0xF3BF, lower=0x8Fxx */
    if ((upper & 0xFFFF) == 0xF3BF && (lower & 0xFF00) == 0x8F00) {
        /* Memory barriers are NOPs in emulator */
        return 1;
    }
    /* SDIV T1: upper = 1111 1011 1001 Rn, lower = 1111 Rd 1111 Rm */
    if ((upper & 0xFFF0) == 0xFB90 && (lower & 0xF0F0) == 0xF0F0) {
        int Rd = (lower >> 8) & 0xF;
        int Rn = upper & 0xF;
        int Rm = lower & 0xF;
        int32_t divisor = (int32_t)cpu.r[Rm];
        if (divisor == 0) cpu.r[Rd] = 0;
        else cpu.r[Rd] = (uint32_t)((int32_t)cpu.r[Rn] / divisor);
        return 1;
    }
    /* UDIV T1: upper = 1111 1011 1011 Rn, lower = 1111 Rd 1111 Rm */
    if ((upper & 0xFFF0) == 0xFBB0 && (lower & 0xF0F0) == 0xF0F0) {
        int Rd = (lower >> 8) & 0xF;
        int Rn = upper & 0xF;
        int Rm = lower & 0xF;
        uint32_t divisor = cpu.r[Rm];
        cpu.r[Rd] = (divisor == 0) ? 0 : (cpu.r[Rn] / divisor);
        return 1;
    }
    /* MUL T2: upper = 1111 1011 0000 Rn, lower = 1111 Rd 0000 Rm */
    if ((upper & 0xFFF0) == 0xFB00 && (lower & 0xF0F0) == 0xF000) {
        int Rd = (lower >> 8) & 0xF;
        int Rn = upper & 0xF;
        int Rm = lower & 0xF;
        cpu.r[Rd] = cpu.r[Rn] * cpu.r[Rm];
        return 1;
    }
    /* MLA T1: upper = 1111 1011 0000 Rn, lower = Ra Rd 0000 Rm (Ra != 1111) */
    if ((upper & 0xFFF0) == 0xFB00 && (lower & 0x00F0) == 0x0000) {
        int Rd = (lower >> 8) & 0xF;
        int Ra = (lower >> 12) & 0xF;
        int Rn = upper & 0xF;
        int Rm = lower & 0xF;
        if (Ra != 15) { cpu.r[Rd] = cpu.r[Rn] * cpu.r[Rm] + cpu.r[Ra]; return 1; }
    }
    /* MLS T1: upper = 1111 1011 0000 Rn, lower = Ra Rd 0001 Rm */
    if ((upper & 0xFFF0) == 0xFB00 && (lower & 0x00F0) == 0x0010) {
        int Rd = (lower >> 8) & 0xF;
        int Ra = (lower >> 12) & 0xF;
        int Rn = upper & 0xF;
        int Rm = lower & 0xF;
        cpu.r[Rd] = cpu.r[Ra] - cpu.r[Rn] * cpu.r[Rm];
        return 1;
    }
    /* SMULL T1: upper = 1111 1011 1000 Rn, lower = RdLo RdHi 0000 Rm */
    if ((upper & 0xFFF0) == 0xFB80 && (lower & 0x00F0) == 0x0000) {
        int RdLo = (lower >> 12) & 0xF;
        int RdHi = (lower >> 8) & 0xF;
        int Rn   = upper & 0xF;
        int Rm   = lower & 0xF;
        int64_t result = (int64_t)(int32_t)cpu.r[Rn] * (int64_t)(int32_t)cpu.r[Rm];
        cpu.r[RdLo] = (uint32_t)result;
        cpu.r[RdHi] = (uint32_t)(result >> 32);
        return 1;
    }
    /* UMULL T1: upper = 1111 1011 1010 Rn, lower = RdLo RdHi 0000 Rm */
    if ((upper & 0xFFF0) == 0xFBA0 && (lower & 0x00F0) == 0x0000) {
        int RdLo = (lower >> 12) & 0xF;
        int RdHi = (lower >> 8) & 0xF;
        int Rn   = upper & 0xF;
        int Rm   = lower & 0xF;
        uint64_t result = (uint64_t)cpu.r[Rn] * (uint64_t)cpu.r[Rm];
        cpu.r[RdLo] = (uint32_t)result;
        cpu.r[RdHi] = (uint32_t)(result >> 32);
        return 1;
    }
    /* UMLAL T1: upper = 1111 1011 1110 Rn, lower = RdLo RdHi 0000 Rm */
    if ((upper & 0xFFF0) == 0xFBE0 && (lower & 0x00F0) == 0x0000) {
        int RdLo = (lower >> 12) & 0xF;
        int RdHi = (lower >> 8) & 0xF;
        int Rn   = upper & 0xF;
        int Rm   = lower & 0xF;
        uint64_t acc    = ((uint64_t)cpu.r[RdHi] << 32) | cpu.r[RdLo];
        uint64_t result = acc + (uint64_t)cpu.r[Rn] * (uint64_t)cpu.r[Rm];
        cpu.r[RdLo] = (uint32_t)result;
        cpu.r[RdHi] = (uint32_t)(result >> 32);
        return 1;
    }
    /* SMLAL T1: upper = 1111 1011 1100 Rn, lower = RdLo RdHi 0000 Rm */
    if ((upper & 0xFFF0) == 0xFBC0 && (lower & 0x00F0) == 0x0000) {
        int RdLo = (lower >> 12) & 0xF;
        int RdHi = (lower >> 8) & 0xF;
        int Rn   = upper & 0xF;
        int Rm   = lower & 0xF;
        int64_t  acc    = (int64_t)(((uint64_t)cpu.r[RdHi] << 32) | cpu.r[RdLo]);
        int64_t  result = acc + (int64_t)(int32_t)cpu.r[Rn] * (int64_t)(int32_t)cpu.r[Rm];
        cpu.r[RdLo] = (uint32_t)result;
        cpu.r[RdHi] = (uint32_t)((uint64_t)result >> 32);
        return 1;
    }
    /* CLZ T1: upper = 1111 1010 1011 Rm, lower = 1111 Rd 1000 Rm */
    if ((upper & 0xFFF0) == 0xFAB0 && (lower & 0xF0FF) == 0xF080) {
        int Rd = (lower >> 8) & 0xF;
        int Rm = lower & 0xF;
        cpu.r[Rd] = cpu.r[Rm] ? __builtin_clz(cpu.r[Rm]) : 32;
        return 1;
    }
    /* RBIT T1: upper = 1111 1010 1001 Rm, lower = 1111 Rd 1010 Rm */
    if ((upper & 0xFFF0) == 0xFA90 && (lower & 0xF0FF) == 0xF0A0) {
        int Rd = (lower >> 8) & 0xF;
        int Rm = lower & 0xF;
        uint32_t v = cpu.r[Rm], r = 0;
        for (int b = 0; b < 32; b++) { r = (r << 1) | (v & 1); v >>= 1; }
        cpu.r[Rd] = r;
        return 1;
    }
    /* UBFX T1: upper = 1111 0011 1100 Rn, lower = 0 imm3 Rd imm2 widthm1 */
    if ((upper & 0xFFF0) == 0xF3C0 && (lower & 0x8000) == 0x0000) {
        int Rd      = (lower >> 8) & 0xF;
        int Rn      = upper & 0xF;
        int lsbit   = ((lower >> 12) & 7) << 2 | ((lower >> 6) & 3);
        int widthm1 = lower & 0x1F;
        uint32_t mask = (widthm1 == 31) ? 0xFFFFFFFF : ((1u << (widthm1 + 1)) - 1);
        cpu.r[Rd] = (cpu.r[Rn] >> lsbit) & mask;
        return 1;
    }
    /* SBFX T1: upper = 1111 0011 0100 Rn */
    if ((upper & 0xFFF0) == 0xF340 && (lower & 0x8000) == 0x0000) {
        int Rd      = (lower >> 8) & 0xF;
        int Rn      = upper & 0xF;
        int lsbit   = ((lower >> 12) & 7) << 2 | ((lower >> 6) & 3);
        int widthm1 = lower & 0x1F;
        uint32_t extracted = (cpu.r[Rn] >> lsbit) & ((widthm1 == 31) ? 0xFFFFFFFF : ((1u << (widthm1+1))-1));
        /* Sign-extend from bit widthm1 */
        if (extracted >> widthm1) extracted |= ~((1u << (widthm1 + 1)) - 1);
        cpu.r[Rd] = extracted;
        return 1;
    }
    /* BFI T1 / BFC T1: upper = 1111 0011 0110 Rn, lower = 0 imm3 Rd imm2 0 msbit */
    if ((upper & 0xFFF0) == 0xF360 && (lower & 0x8020) == 0x0000) {
        int Rd     = (lower >> 8) & 0xF;
        int Rn     = upper & 0xF;
        int lsbit  = ((lower >> 12) & 7) << 2 | ((lower >> 6) & 3);
        int msbit  = lower & 0x1F;
        if (msbit >= lsbit) {
            uint32_t width = msbit - lsbit + 1;
            uint32_t mask  = ((1u << width) - 1) << lsbit;
            uint32_t src   = (Rn == 15) ? 0 : cpu.r[Rn];
            cpu.r[Rd] = (cpu.r[Rd] & ~mask) | ((src << lsbit) & mask);
        }
        return 1;
    }
    /* SXTB T2: upper = 1111 1010 0100 1111, lower = 1111 Rd 10RR Rm */
    if ((upper & 0xFFFF) == 0xFA4F && (lower & 0xF0C0) == 0xF080) {
        int Rd  = (lower >> 8) & 0xF;
        int Rm  = lower & 0xF;
        int rot = ((lower >> 4) & 3) * 8;
        uint32_t val = cpu.r[Rm];
        if (rot) val = (val >> rot) | (val << (32 - rot));
        cpu.r[Rd] = (uint32_t)(int32_t)(int8_t)(val & 0xFF);
        return 1;
    }
    /* SXTH T2: upper = 1111 1010 0000 1111 */
    if ((upper & 0xFFFF) == 0xFA0F && (lower & 0xF0C0) == 0xF080) {
        int Rd  = (lower >> 8) & 0xF;
        int Rm  = lower & 0xF;
        int rot = ((lower >> 4) & 3) * 8;
        uint32_t val = cpu.r[Rm];
        if (rot) val = (val >> rot) | (val << (32 - rot));
        cpu.r[Rd] = (uint32_t)(int32_t)(int16_t)(val & 0xFFFF);
        return 1;
    }
    /* UXTB T2: upper = 1111 1010 0101 1111 */
    if ((upper & 0xFFFF) == 0xFA5F && (lower & 0xF0C0) == 0xF080) {
        int Rd  = (lower >> 8) & 0xF;
        int Rm  = lower & 0xF;
        int rot = ((lower >> 4) & 3) * 8;
        uint32_t val = cpu.r[Rm];
        if (rot) val = (val >> rot) | (val << (32 - rot));
        cpu.r[Rd] = val & 0xFF;
        return 1;
    }
    /* UXTH T2: upper = 1111 1010 0001 1111 */
    if ((upper & 0xFFFF) == 0xFA1F && (lower & 0xF0C0) == 0xF080) {
        int Rd  = (lower >> 8) & 0xF;
        int Rm  = lower & 0xF;
        int rot = ((lower >> 4) & 3) * 8;
        uint32_t val = cpu.r[Rm];
        if (rot) val = (val >> rot) | (val << (32 - rot));
        cpu.r[Rd] = val & 0xFFFF;
        return 1;
    }
    /* REV T2: upper = 1111 1010 1001 Rm, lower = 1111 Rd 1000 Rm */
    if ((upper & 0xFFF0) == 0xFA90 && (lower & 0xF0FF) == 0xF080) {
        int Rd = (lower >> 8) & 0xF;
        int Rm = lower & 0xF;
        uint32_t v = cpu.r[Rm];
        cpu.r[Rd] = ((v&0xFF)<<24)|((v&0xFF00)<<8)|((v&0xFF0000)>>8)|((v>>24)&0xFF);
        return 1;
    }
    /* BLX register is 16-bit (handled elsewhere) */
    /* NOP/IT/YIELD etc. 32-bit hints: upper = 0xF3AF, lower = 0x8000..80xx */
    if ((upper & 0xFFFF) == 0xF3AF && (lower & 0xFF00) == 0x8000) {
        return 1;  /* NOP, YIELD, WFE, WFI, SEV hints */
    }

    (void)pc;
    return 0;
}

/* ========================================================================
 * Load/Store single (32-bit): T2/T3/T4 encodings
 * upper pattern analysis:
 *   bit[8]=1 (T3): unsigned imm12 offset
 *     0xF8C0|Rn: STR.W   0xF8D0|Rn: LDR.W
 *     0xF880|Rn: STRB.W  0xF890|Rn: LDRB.W
 *     0xF8A0|Rn: STRH.W  0xF8B0|Rn: LDRH.W (also LDR.W T3 with Rn=15 = PC-relative)
 *     0xF990|Rn: LDRSB.W 0xF9B0|Rn: LDRSH.W
 *   bit[8]=0 (T4/T2): signed imm8 pre/post or register offset
 * ======================================================================== */
static void t32_ldst_single(uint32_t pc, uint16_t upper, uint16_t lower) {
    int Rn   = upper & 0xF;
    int size = (upper >> 5) & 3;   /* 00=byte, 01=hword, 10=word */
    int sign = (upper >> 8) & 1;   /* for F9xx: signed extension */
    int L    = (upper >> 4) & 1;   /* 1=load, 0=store */
    int Rt   = (lower >> 12) & 0xF;

    /* Distinguish T3 (12-bit unsigned imm, upper[7]=1 for same-size group,
     * more precisely: for 0xF8xx upper[8]=1 means T3, for 0xF9xx always T1/T3) */
    /* Simplification: use upper[11:8] to determine form */
    int upper_hi = (upper >> 4) & 0xF;  /* bits[11:8] */

    if (upper_hi & 0x8) {
        /* T3: 12-bit unsigned offset. lower = Rt(4):imm12(12) */
        int imm12 = lower & 0xFFF;
        uint32_t base = (Rn == 15) ? ((pc + 4) & ~3u) : cpu.r[Rn];
        uint32_t addr;
        if (Rn == 15) addr = base + imm12;  /* LDR PC-relative always adds */
        else addr = base + (uint32_t)imm12;

        /* Decode by upper[7:4] */
        int bits74 = (upper >> 4) & 0xF;
        /* And sign extension bit from upper[8] or upper group: */
        /* For F8xx: no sign extension (unsigned) */
        /* For F9xx: sign extension */
        int is_signed = ((upper >> 8) & 0xF) == 9;  /* upper[11:8]=1001 = 0xF9 group */

        switch (bits74) {
        case 0x8: /* STRB */
            mem_write8(base + imm12, cpu.r[Rt] & 0xFF); break;
        case 0x9: /* LDRB */
            cpu.r[Rt] = is_signed ? (uint32_t)(int32_t)(int8_t)mem_read8(base + imm12)
                                  : (uint32_t)mem_read8(base + imm12);
            break;
        case 0xA: /* STRH */
            mem_write16(base + imm12, cpu.r[Rt] & 0xFFFF); break;
        case 0xB: /* LDRH */
            cpu.r[Rt] = is_signed ? (uint32_t)(int32_t)(int16_t)mem_read16(base + imm12)
                                  : (uint32_t)mem_read16(base + imm12);
            break;
        case 0xC: /* STR.W */
            mem_write32(base + imm12, cpu.r[Rt]); break;
        case 0xD: /* LDR.W */
            cpu.r[Rt] = mem_read32(base + imm12);
            if (Rt == 15) pc_updated = 1;
            break;
        default:
            goto unhandled_ldst;
        }
        return;
    }

    /* T4/T2: 8-bit signed offset with P/U/W, or register offset */
    if (lower & 0x0800) {
        /* T4: lower = Rt(4) : 1 : P : U : W : imm8 */
        int P    = (lower >> 10) & 1;
        int U    = (lower >> 9) & 1;
        int W    = (lower >> 8) & 1;
        int imm8 = lower & 0xFF;
        uint32_t base = cpu.r[Rn];
        int32_t  off  = U ? (int32_t)imm8 : -(int32_t)imm8;
        uint32_t offset_addr = (uint32_t)((int32_t)base + off);
        uint32_t addr = P ? offset_addr : base;

        int bits74 = (upper >> 4) & 0xF;
        int is_signed = ((upper >> 8) & 0xF) == 9;
        switch (bits74) {
        case 0x8: case 0x0:
            if (L) cpu.r[Rt] = mem_read8(addr); else mem_write8(addr, cpu.r[Rt]&0xFF); break;
        case 0x9: case 0x1:
            cpu.r[Rt] = is_signed ? (uint32_t)(int32_t)(int8_t)mem_read8(addr)
                                  : (uint32_t)mem_read8(addr); break;
        case 0xA: case 0x2:
            if (L) cpu.r[Rt] = mem_read16(addr); else mem_write16(addr, cpu.r[Rt]&0xFFFF); break;
        case 0xB: case 0x3:
            cpu.r[Rt] = is_signed ? (uint32_t)(int32_t)(int16_t)mem_read16(addr)
                                  : (uint32_t)mem_read16(addr); break;
        case 0xC: case 0x4:
            if (L) cpu.r[Rt] = mem_read32(addr); else mem_write32(addr, cpu.r[Rt]); break;
        case 0xD: case 0x5:
            if (L) { cpu.r[Rt] = mem_read32(addr); if (Rt==15) pc_updated=1; }
            else   { mem_write32(addr, cpu.r[Rt]); }
            break;
        default: goto unhandled_ldst;
        }
        if (W) cpu.r[Rn] = offset_addr;
        return;
    }

    /* T2: register offset. lower = Rt(4) : 000000 : imm2(2) : Rm(4) */
    {
        int imm2 = (lower >> 4) & 3;
        int Rm   = lower & 0xF;
        uint32_t offset = cpu.r[Rm] << imm2;
        uint32_t addr   = cpu.r[Rn] + offset;
        int bits74 = (upper >> 4) & 0xF;
        int is_signed = ((upper >> 8) & 0xF) == 9;

        switch (bits74) {
        case 0x8: case 0x0:
            if (L) cpu.r[Rt] = mem_read8(addr); else mem_write8(addr, cpu.r[Rt]&0xFF); break;
        case 0x9: case 0x1:
            cpu.r[Rt] = is_signed ? (uint32_t)(int32_t)(int8_t)mem_read8(addr)
                                  : (uint32_t)mem_read8(addr); break;
        case 0xA: case 0x2:
            if (L) cpu.r[Rt] = mem_read16(addr); else mem_write16(addr, cpu.r[Rt]&0xFFFF); break;
        case 0xB: case 0x3:
            cpu.r[Rt] = is_signed ? (uint32_t)(int32_t)(int16_t)mem_read16(addr)
                                  : (uint32_t)mem_read16(addr); break;
        case 0xC: case 0x4:
            if (L) cpu.r[Rt] = mem_read32(addr); else mem_write32(addr, cpu.r[Rt]); break;
        case 0xD: case 0x5:
            if (L) { cpu.r[Rt] = mem_read32(addr); if (Rt==15) pc_updated=1; }
            else   { mem_write32(addr, cpu.r[Rt]); }
            break;
        case 0x7: /* LDRSH.W T2 (signed halfword, register offset): upper[15:8]=0xF9, bits74=7 */
            /* 0xF97x Rn: LDRSH.W Rt, [Rn, Rm{, LSL #imm2}] */
            cpu.r[Rt] = (uint32_t)(int32_t)(int16_t)mem_read16(addr);
            break;
        default: goto unhandled_ldst;
        }
        return;
    }

unhandled_ldst:
    if (cpu.debug_enabled)
        fprintf(stderr, "[T32] Unhandled ldst upper=0x%04X lower=0x%04X @ PC=0x%08X\n",
                upper, lower, pc);
    cpu_exception_entry(EXC_HARDFAULT);
    (void)sign; (void)size; (void)L;
}

/* ========================================================================
 * Main 32-bit Thumb-2 dispatcher
 * Called with current pc (address of upper halfword), upper and lower halfwords.
 * Returns 1 if instruction was handled, 0 if unknown (caller triggers HardFault).
 * ======================================================================== */
int thumb32_step(uint32_t pc, uint16_t upper, uint16_t lower) {
    /* Update PC to skip this 32-bit instruction (caller may override) */
    cpu.r[15] = pc + 4;
    pc_updated = 1;

    uint8_t top5 = upper >> 11;  /* bits[15:11] of upper */

    /* ------------------------------------------------------------------ */
    /* Group 11101: E8xx-EFxx                                              */
    /* ------------------------------------------------------------------ */
    if (top5 == 0x1D) {
        uint8_t bits_10_9 = (upper >> 9) & 3;
        if (bits_10_9 == 0) {
            /* Load/Store Multiple or Double */
            /* Distinguish: LDRD/STRD (bit[6]=1 in this subgroup) vs LDM/STM */
            /* For LDRD/STRD T1: upper[6]=1 (but let's use upper bits more carefully) */
            /* upper[9:8]: 00=STMIA/LDMIA T2, 01=LDM/STM again, 10=? */
            /* Simpler: upper[8]=is_DB, upper[7]=L */
            /* Check for LDRD/STRD: upper[6]=1 and upper[7:5] pattern */
            if (upper & 0x0040) {
                /* LDRD/STRD: upper[6]=1 */
                t32_ldrd_strd(pc, upper, lower);
            } else {
                /* LDM/STM T2 */
                /* Check for TBB/TBH: upper = 0xE8DF or 0xE89F ... actually */
                /* TBB/TBH: upper[15:4] = 1110 1000 1101 = 0xE8D, lower[15:4]=0xF00? */
                if ((upper & 0xFFF0) == 0xE8D0 && (lower & 0xFFE0) == 0xF000) {
                    t32_tbb_tbh(pc, upper, lower);
                } else {
                    t32_ldst_multiple(pc, upper, lower);
                }
            }
            return 1;
        }
        if (bits_10_9 == 1) {
            /* LDRD/STRD with different pre/post-index forms (0xE9xx) */
            t32_ldrd_strd(pc, upper, lower);
            return 1;
        }
        /* bits_10_9 == 2 or 3: Data processing shifted register */
        t32_dp_shifted_reg(pc, upper, lower);
        return 1;
    }

    /* ------------------------------------------------------------------ */
    /* Group 11110: F0xx-F7xx                                              */
    /* ------------------------------------------------------------------ */
    if (top5 == 0x1E) {
        /* Check MSR/MRS/barriers first: they have lower bit[15]=1 and would
         * otherwise be misidentified as branches (e.g. MSR F380 8808). */
        if (t32_misc(pc, upper, lower)) return 1;
        /* BL T1: lower bits[15,14,12] = 1,1,1 */
        if ((lower & 0xD000) == 0xD000) {
            t32_bl(pc, upper, lower);
            return 1;
        }
        /* Branch (wide): lower bit[15]=1 */
        if (lower & 0x8000) {
            t32_branch(pc, upper, lower);
            return 1;
        }
        /* Data processing (lower bit[15]=0) */
        if (upper & 0x0200) {
            /* Plain binary immediate (bit9=1): MOVW, MOVT, ADDW, SUBW */
            t32_dp_plain_imm(pc, upper, lower);
        } else {
            /* Modified immediate (bit9=0) */
            t32_dp_mod_imm(pc, upper, lower);
        }
        return 1;
    }

    /* ------------------------------------------------------------------ */
    /* Group 11111: F8xx-FFxx                                              */
    /* ------------------------------------------------------------------ */
    if (top5 == 0x1F) {
        /* Check misc 32-bit first (SDIV, UDIV, MUL, CLZ etc.) */
        if (t32_misc(pc, upper, lower)) return 1;
        /* Load/Store single */
        t32_ldst_single(pc, upper, lower);
        return 1;
    }

    return 0;  /* Unknown */
}
