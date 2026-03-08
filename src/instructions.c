#include <stdio.h>
#include <stdint.h>
#include "emulator.h"
#include "instructions.h"
#include "nvic.h"

/* pc_updated flag: instruction handlers set this when they modify cpu.r[15] */
extern int pc_updated;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Flag bit positions in XPSR
#define FLAG_N 0x80000000  // Negative (bit 31)
#define FLAG_Z 0x40000000  // Zero (bit 30)
#define FLAG_C 0x20000000  // Carry (bit 29)
#define FLAG_V 0x10000000  // Overflow (bit 28)

void update_nz_flags(uint32_t result) {
    cpu.xpsr &= ~(FLAG_N | FLAG_Z);
    if (result == 0) cpu.xpsr |= FLAG_Z;
    if (result & 0x80000000) cpu.xpsr |= FLAG_N;
}

void update_add_flags(uint32_t op1, uint32_t op2, uint32_t result) {
    cpu.xpsr &= ~(FLAG_N | FLAG_Z | FLAG_C | FLAG_V);

    // N and Z flags
    if (result == 0) cpu.xpsr |= FLAG_Z;
    if (result & 0x80000000) cpu.xpsr |= FLAG_N;

    // Carry flag (unsigned overflow)
    uint64_t result64 = (uint64_t)op1 + (uint64_t)op2;
    if (result64 > 0xFFFFFFFF) cpu.xpsr |= FLAG_C;

    // Overflow flag (signed overflow)
    if (((op1 ^ result) & (op2 ^ result)) & 0x80000000) cpu.xpsr |= FLAG_V;
}

void update_sub_flags(uint32_t op1, uint32_t op2, uint32_t result) {
    cpu.xpsr &= ~(FLAG_N | FLAG_Z | FLAG_C | FLAG_V);

    // N and Z flags
    if (result == 0) cpu.xpsr |= FLAG_Z;
    if (result & 0x80000000) cpu.xpsr |= FLAG_N;

    // Carry flag (NOT borrow - set if op1 >= op2)
    if (op1 >= op2) cpu.xpsr |= FLAG_C;

    // Overflow flag (signed overflow)
    if (((op1 ^ op2) & (op1 ^ result)) & 0x80000000) cpu.xpsr |= FLAG_V;
}

uint32_t sign_extend_8(uint8_t value) {
    return (value & 0x80) ? (value | 0xFFFFFF00) : value;
}

uint32_t sign_extend_16(uint16_t value) {
    return (value & 0x8000) ? (value | 0xFFFF0000) : value;
}

// ============================================================================
// FOUNDATIONAL INSTRUCTIONS
// ============================================================================

/* ============ ALU Instructions ============ */

void instr_adds_imm3(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x07;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    uint32_t op1 = cpu.r[reg_src];
    uint32_t result = op1 + imm;
    cpu.r[reg_dst] = result;
    update_add_flags(op1, imm, result);
}

void instr_adds_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    uint32_t op1 = cpu.r[reg];
    uint32_t result = op1 + imm;
    cpu.r[reg] = result;
    update_add_flags(op1, imm, result);
}

void instr_subs_imm3(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x07;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    uint32_t op1 = cpu.r[reg_src];
    uint32_t result = op1 - imm;
    cpu.r[reg_dst] = result;
    update_sub_flags(op1, imm, result);
}

void instr_subs_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    uint32_t op1 = cpu.r[reg];
    uint32_t result = op1 - imm;
    cpu.r[reg] = result;
    update_sub_flags(op1, imm, result);
}

void instr_movs_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    cpu.r[reg] = imm;
    update_nz_flags(imm);
}

void instr_mov_reg(uint16_t instr) {
    uint8_t rd = ((instr >> 4) & 0x8) | (instr & 0x7);
    uint8_t rm = (instr >> 3) & 0xF;

    uint32_t value = cpu.r[rm];

    if (rd == 15) {
        cpu.r[15] = value & ~1;
        pc_updated = 1;
    } else {
        cpu.r[rd] = value;
    }
}

void instr_adds_reg_reg(uint16_t instr) {
    uint8_t rm = (instr >> 6) & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rd = instr & 0x07;
    uint32_t op1 = cpu.r[rn];
    uint32_t op2 = cpu.r[rm];
    uint32_t result = op1 + op2;
    cpu.r[rd] = result;
    update_add_flags(op1, op2, result);
}

void instr_add_reg_high(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x0F;
    uint8_t reg_dst = ((instr >> 4) & 0x08) | (instr & 0x07);
    uint32_t result = cpu.r[reg_dst] + cpu.r[reg_src];

    if (reg_dst == 15) {
        cpu.r[15] = result & ~1u;
        pc_updated = 1;
    } else {
        cpu.r[reg_dst] = result;
    }
}

void instr_sub_reg_reg(uint16_t instr) {
    uint8_t rm = (instr >> 6) & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rd = instr & 0x07;
    uint32_t op1 = cpu.r[rn];
    uint32_t op2 = cpu.r[rm];
    uint32_t result = op1 - op2;
    cpu.r[rd] = result;
    update_sub_flags(op1, op2, result);
}

void instr_cmp_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    uint32_t op1 = cpu.r[reg];
    uint32_t result = op1 - imm;
    update_sub_flags(op1, imm, result);
}

void instr_cmp_reg_reg(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x0F;
    uint8_t reg_dst = ((instr >> 4) & 0x08) | (instr & 0x07);
    uint32_t op1 = cpu.r[reg_dst];
    uint32_t op2 = cpu.r[reg_src];
    uint32_t result = op1 - op2;
    update_sub_flags(op1, op2, result);
}

/* ============ Memory Instructions ============ */

void instr_ldr_imm5(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + (imm5 << 2);
    cpu.r[rd] = mem_read32(addr);
}

void instr_str_imm5(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + (imm5 << 2);
    mem_write32(addr, cpu.r[rd]);
}

void instr_ldr_pc_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    uint32_t addr = (cpu.r[15] & ~3) + (imm * 4) + 4;
    cpu.r[reg] = mem_read32(addr);
}

void instr_ldr_sp_imm8(uint16_t instr) {
    uint8_t rd = (instr >> 8) & 0x07;
    uint8_t imm8 = instr & 0xFF;
    uint32_t addr = cpu.r[13] + (imm8 << 2);
    cpu.r[rd] = mem_read32(addr);
}

void instr_str_sp_imm8(uint16_t instr) {
    uint8_t rd = (instr >> 8) & 0x07;
    uint8_t imm8 = instr & 0xFF;
    uint32_t addr = cpu.r[13] + (imm8 << 2);
    mem_write32(addr, cpu.r[rd]);
}

void instr_str_reg_offset(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    mem_write32(addr, cpu.r[rd]);
}

void instr_ldr_reg_offset(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    cpu.r[rd] = mem_read32(addr);
}

void instr_stmia(uint16_t instr) {
    uint8_t reg_base = (instr >> 8) & 0x07;
    uint8_t rlist = instr & 0xFF;
    for (int i = 0; i < 8; i++) {
        if (rlist & (1 << i)) {
            mem_write32(cpu.r[reg_base], cpu.r[i]);
            cpu.r[reg_base] += 4;
        }
    }
}

void instr_ldmia(uint16_t instr) {
    uint8_t reg_base = (instr >> 8) & 0x07;
    uint8_t rlist = instr & 0xFF;
    for (int i = 0; i < 8; i++) {
        if (rlist & (1 << i)) {
            cpu.r[i] = mem_read32(cpu.r[reg_base]);
            cpu.r[reg_base] += 4;
        }
    }
}

void instr_push(uint16_t instr) {
    uint8_t reglist = instr & 0xFF;
    uint8_t M = (instr >> 8) & 0x1;

    uint32_t sp = cpu.r[13];

    if (M) {
        sp -= 4;
        mem_write32(sp, cpu.r[14]);
    }

    for (int i = 7; i >= 0; i--) {
        if (reglist & (1 << i)) {
            sp -= 4;
            mem_write32(sp, cpu.r[i]);
        }
    }

    cpu.r[13] = sp;
}

void instr_pop(uint16_t instr) {
    uint8_t reglist = instr & 0xFF;
    uint8_t P = (instr >> 8) & 0x1;

    uint32_t sp = cpu.r[13];

    if (cpu.debug_asm) {
        printf("[POP] SP=0x%08X reglist=0x%02X P=%d current_irq=%d\n",
               sp, reglist, P, cpu.current_irq);
    }

    for (int i = 0; i < 8; i++) {
        if (reglist & (1 << i)) {
            uint32_t val = mem_read32(sp);
            if (cpu.debug_asm) {
                printf("[POP]   R%d @ 0x%08X = 0x%08X\n", i, sp, val);
            }
            cpu.r[i] = val;
            sp += 4;
        }
    }

    if (P) {
        uint32_t pc_val = mem_read32(sp);
        if (cpu.debug_asm) {
            printf("[POP]   PC @ 0x%08X = 0x%08X (magic check: 0x%08X)\n",
                   sp, pc_val, pc_val & 0xFFFFFFF0);
        }

        /* Check if PC is a magic exception return value */
        if ((pc_val & 0xFFFFFFF0) == 0xFFFFFFF0) {
            if (cpu.debug_asm) {
                printf("[POP] *** MAGIC VALUE DETECTED - EXCEPTION RETURN ***\n");
            }
            cpu_exception_return(pc_val);
            pc_updated = 1;
            return;
        }

        cpu.r[15] = pc_val & ~1;
        sp += 4;
        pc_updated = 1;
    }

    cpu.r[13] = sp;
}


/* ============ Branch Instructions ============ */

void instr_b_uncond(uint16_t instr) {
    int16_t offset = instr & 0x07FF;
    if (offset & 0x0400) {
        offset |= 0xF800;
    }
    int32_t signed_offset = (int32_t)offset;
    signed_offset <<= 1;
    cpu.r[15] += 4 + signed_offset;
    pc_updated = 1;
}

void instr_bcond(uint16_t instr) {
    uint8_t cond = (instr >> 8) & 0x0F;
    int8_t offset = instr & 0xFF;
    int32_t signed_offset = (int32_t)(offset << 24) >> 23;

    int take_branch = 0;
    switch (cond) {
        case 0x0: take_branch = (cpu.xpsr & FLAG_Z) != 0; break;
        case 0x1: take_branch = (cpu.xpsr & FLAG_Z) == 0; break;
        case 0x2: take_branch = (cpu.xpsr & FLAG_C) != 0; break;
        case 0x3: take_branch = (cpu.xpsr & FLAG_C) == 0; break;
        case 0x4: take_branch = (cpu.xpsr & FLAG_N) != 0; break;
        case 0x5: take_branch = (cpu.xpsr & FLAG_N) == 0; break;
        case 0x6: take_branch = (cpu.xpsr & FLAG_V) != 0; break;
        case 0x7: take_branch = (cpu.xpsr & FLAG_V) == 0; break;
        case 0x8: take_branch = ((cpu.xpsr & FLAG_C) != 0) && ((cpu.xpsr & FLAG_Z) == 0); break;
        case 0x9: take_branch = ((cpu.xpsr & FLAG_C) == 0) || ((cpu.xpsr & FLAG_Z) != 0); break;
        case 0xA: take_branch = ((cpu.xpsr & FLAG_N) != 0) == ((cpu.xpsr & FLAG_V) != 0); break;
        case 0xB: take_branch = ((cpu.xpsr & FLAG_N) != 0) != ((cpu.xpsr & FLAG_V) != 0); break;
        case 0xC: take_branch = ((cpu.xpsr & FLAG_Z) == 0) &&
                     (((cpu.xpsr & FLAG_N) != 0) == ((cpu.xpsr & FLAG_V) != 0)); break;
        case 0xD: take_branch = ((cpu.xpsr & FLAG_Z) != 0) ||
                     (((cpu.xpsr & FLAG_N) != 0) != ((cpu.xpsr & FLAG_V) != 0)); break;
        default: break;
    }

    if (take_branch) {
        cpu.r[15] += 4 + signed_offset;
    } else {
        cpu.r[15] += 2;
    }
    pc_updated = 1;
}


void instr_bl(uint16_t instr) {
    uint16_t instr2 = mem_read16(cpu.r[15] + 2);

    int32_t S = (instr >> 10) & 0x1;
    int32_t J1 = (instr2 >> 13) & 0x1;
    int32_t J2 = (instr2 >> 11) & 0x1;
    int32_t I1 = !(J1 ^ S);
    int32_t I2 = !(J2 ^ S);

    uint32_t imm10 = instr & 0x3FF;
    uint32_t imm11 = instr2 & 0x7FF;

    int32_t offset = (S << 24) | (I1 << 23) | (I2 << 22) |
                     (imm10 << 12) | (imm11 << 1);

    if (offset & 0x01000000) {
        offset |= 0xFE000000;
    }

    uint32_t pc = cpu.r[15];
    uint32_t target = pc + 4 + offset;

    cpu.r[14] = (pc + 4) | 1;
    cpu.r[15] = target & ~1;
    pc_updated = 1;
}


void instr_bl_32(uint16_t upper, uint16_t lower) {
    uint32_t S    = (upper >> 10) & 1;
    uint32_t imm10 =  upper        & 0x03FF;
    uint32_t J1   = (lower >> 13) & 1;
    uint32_t J2   = (lower >> 11) & 1;
    uint32_t imm11 =  lower        & 0x07FF;

    uint32_t I1 = ~(J1 ^ S) & 1;
    uint32_t I2 = ~(J2 ^ S) & 1;

    int32_t offset =
        (S    << 24) |
        (I1   << 23) |
        (I2   << 22) |
        (imm10 << 12) |
        (imm11 << 1);

    if (offset & (1u << 24)) offset |= 0xFE000000;

    uint32_t pc = cpu.r[15];

    cpu.r[14] = (pc + 4) | 1u;
    cpu.r[15] = (pc + 4 + offset) & ~1u;
    pc_updated = 1;
}


void instr_bx(uint16_t instr) {
    uint8_t rm = (instr >> 3) & 0x0F;
    uint32_t target = cpu.r[rm];

    if (cpu.debug_asm) {
        printf("[BX] R%d: target=0x%08X (magic check: 0x%08X)\n",
               rm, target, target & 0xFFFFFFF0);
    }

    /* Check for Exception Return (Magic values like 0xFFFFFFF9) */
    if ((target & 0xFFFFFFF0) == 0xFFFFFFF0) {
        if (cpu.debug_asm) {
            printf("[BX] *** EXCEPTION RETURN DETECTED ***\n");
        }
        cpu_exception_return(target);
        pc_updated = 1;
        return;
    }

    cpu.r[15] = target & ~1u;
    pc_updated = 1;
}


void instr_blx(uint16_t instr) {
    uint8_t rm = (instr >> 3) & 0x0F;
    uint32_t target = cpu.r[rm];

    cpu.r[14] = (cpu.r[15] + 2) | 1;
    cpu.r[15] = target & ~1;
    pc_updated = 1;
}

// ============================================================================
// ESSENTIAL INSTRUCTIONS
// ============================================================================

/* ============ Byte Operations ============ */

void instr_ldrb_imm5(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + imm5;
    cpu.r[rd] = mem_read8(addr);
}

void instr_ldrb_reg_offset(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    cpu.r[rd] = mem_read8(addr);
}

void instr_ldrsb_reg_offset(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    uint8_t value = mem_read8(addr);
    cpu.r[rd] = sign_extend_8(value);
}

void instr_strb_imm5(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + imm5;
    mem_write8(addr, cpu.r[rd] & 0xFF);
}

void instr_strb_reg_offset(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    mem_write8(addr, cpu.r[rd] & 0xFF);
}

/* ============ Halfword Operations ============ */

void instr_ldrh_imm5(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + (imm5 << 1);
    cpu.r[rd] = mem_read16(addr);
}

void instr_ldrh_reg_offset(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    cpu.r[rd] = mem_read16(addr);
}

void instr_ldrsh_reg_offset(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    uint16_t value = mem_read16(addr);
    cpu.r[rd] = sign_extend_16(value);
}

void instr_strh_imm5(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + (imm5 << 1);
    mem_write16(addr, cpu.r[rd] & 0xFFFF);
}

void instr_strh_reg_offset(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    mem_write16(addr, cpu.r[rd] & 0xFFFF);
}

/* ============ Bitwise Instructions ============ */

void instr_tst_reg_reg(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    uint32_t result = cpu.r[reg_dst] & cpu.r[reg_src];
    update_nz_flags(result);
}

void instr_bitwise_and(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] &= cpu.r[reg_src];
    update_nz_flags(cpu.r[reg_dst]);
}

void instr_bitwise_eor(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] ^= cpu.r[reg_src];
    update_nz_flags(cpu.r[reg_dst]);
}

void instr_bitwise_orr(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] |= cpu.r[reg_src];
    update_nz_flags(cpu.r[reg_dst]);
}

void instr_bitwise_bic(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] &= ~cpu.r[reg_src];
    update_nz_flags(cpu.r[reg_dst]);
}

void instr_bitwise_mvn(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] = ~cpu.r[reg_src];
    update_nz_flags(cpu.r[reg_dst]);
}

/* ============ Shift Instructions ============ */

void instr_shift_logical_left(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x1F;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;

    if (imm == 0) {
        cpu.r[reg_dst] = cpu.r[reg_src];
    } else {
        if (cpu.r[reg_src] & (1 << (32 - imm))) {
            cpu.xpsr |= FLAG_C;
        } else {
            cpu.xpsr &= ~FLAG_C;
        }
        cpu.r[reg_dst] = cpu.r[reg_src] << imm;
    }
    update_nz_flags(cpu.r[reg_dst]);
}

void instr_shift_logical_right(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x1F;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;

    if (imm == 0) {
        cpu.r[reg_dst] = 0;
        if (cpu.r[reg_src] & 0x80000000) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
    } else {
        if (cpu.r[reg_src] & (1 << (imm - 1))) {
            cpu.xpsr |= FLAG_C;
        } else {
            cpu.xpsr &= ~FLAG_C;
        }
        cpu.r[reg_dst] = cpu.r[reg_src] >> imm;
    }
    update_nz_flags(cpu.r[reg_dst]);
}

void instr_shift_arithmetic_right(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x1F;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;

    if (imm == 0) {
        if (cpu.r[reg_src] & 0x80000000) {
            cpu.xpsr |= FLAG_C;
            cpu.r[reg_dst] = 0xFFFFFFFF;
        } else {
            cpu.xpsr &= ~FLAG_C;
            cpu.r[reg_dst] = 0;
        }
    } else {
        if (cpu.r[reg_src] & (1u << (imm - 1))) {
            cpu.xpsr |= FLAG_C;
        } else {
            cpu.xpsr &= ~FLAG_C;
        }
        cpu.r[reg_dst] = ((int32_t)cpu.r[reg_src]) >> imm;
    }
    update_nz_flags(cpu.r[reg_dst]);
}

void instr_lsls_reg(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rs = (instr >> 3) & 0x07;
    uint8_t shift = cpu.r[rs] & 0xFF;

    if (shift == 0) {
        /* No change */
    } else if (shift < 32) {
        if (cpu.r[rd] & (1 << (32 - shift))) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] <<= shift;
        update_nz_flags(cpu.r[rd]);
    } else if (shift == 32) {
        if (cpu.r[rd] & 1) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] = 0;
        update_nz_flags(0);
    } else {
        cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] = 0;
        update_nz_flags(0);
    }
}

void instr_lsrs_reg(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rs = (instr >> 3) & 0x07;
    uint8_t shift = cpu.r[rs] & 0xFF;

    if (shift == 0) {
        /* No change */
    } else if (shift < 32) {
        if (cpu.r[rd] & (1 << (shift - 1))) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] >>= shift;
        update_nz_flags(cpu.r[rd]);
    } else if (shift == 32) {
        if (cpu.r[rd] & 0x80000000) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] = 0;
        update_nz_flags(0);
    } else {
        cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] = 0;
        update_nz_flags(0);
    }
}

void instr_asrs_reg(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rs = (instr >> 3) & 0x07;
    uint8_t shift = cpu.r[rs] & 0xFF;

    if (shift == 0) {
        /* No change */
    } else if (shift < 32) {
        if (cpu.r[rd] & (1 << (shift - 1))) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] = ((int32_t)cpu.r[rd]) >> shift;
        update_nz_flags(cpu.r[rd]);
    } else {
        if (cpu.r[rd] & 0x80000000) {
            cpu.xpsr |= FLAG_C;
            cpu.r[rd] = 0xFFFFFFFF;
        } else {
            cpu.xpsr &= ~FLAG_C;
            cpu.r[rd] = 0;
        }
        update_nz_flags(cpu.r[rd]);
    }
}

void instr_rors_reg(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rs = (instr >> 3) & 0x07;
    uint8_t shift_full = cpu.r[rs] & 0xFF;

    if (shift_full == 0) {
        /* No change */
    } else {
        uint8_t shift = shift_full & 0x1F;
        uint32_t result;
        if (shift == 0) {
            result = cpu.r[rd];
        } else {
            result = (cpu.r[rd] >> shift) | (cpu.r[rd] << (32 - shift));
        }
        if (result & 0x80000000) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] = result;
        update_nz_flags(result);
    }
}

/* ============ Multiplication & Carry Arithmetic ============ */

void instr_muls(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;

    cpu.r[rd] = cpu.r[rd] * cpu.r[rm];
    update_nz_flags(cpu.r[rd]);
}

void instr_adcs(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t carry = (cpu.xpsr & FLAG_C) ? 1 : 0;
    uint32_t op1 = cpu.r[rd];
    uint32_t op2 = cpu.r[rm];
    uint64_t result64 = (uint64_t)op1 + (uint64_t)op2 + carry;
    uint32_t result = (uint32_t)result64;
    cpu.r[rd] = result;

    cpu.xpsr &= ~(FLAG_N | FLAG_Z | FLAG_C | FLAG_V);
    if (result == 0) cpu.xpsr |= FLAG_Z;
    if (result & 0x80000000) cpu.xpsr |= FLAG_N;
    if (result64 > 0xFFFFFFFF) cpu.xpsr |= FLAG_C;
    if (((op1 ^ result) & (op2 ^ result)) & 0x80000000) cpu.xpsr |= FLAG_V;
}

void instr_sbcs(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t carry = (cpu.xpsr & FLAG_C) ? 1 : 0;
    uint32_t op1 = cpu.r[rd];
    uint32_t op2 = cpu.r[rm];
    uint32_t result = op1 - op2 - (1 - carry);
    cpu.r[rd] = result;

    cpu.xpsr &= ~(FLAG_N | FLAG_Z | FLAG_C | FLAG_V);
    if (result == 0) cpu.xpsr |= FLAG_Z;
    if (result & 0x80000000) cpu.xpsr |= FLAG_N;
    /* C flag: no borrow occurred */
    uint64_t uresult = (uint64_t)op1 - (uint64_t)op2 - (uint64_t)(1 - carry);
    if ((uresult >> 32) == 0) cpu.xpsr |= FLAG_C;
    if (((op1 ^ op2) & (op1 ^ result)) & 0x80000000) cpu.xpsr |= FLAG_V;
}

void instr_rsbs(uint16_t instr) {
    /* RSBS Rd, Rm, #0 (NEG) */
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t op2 = cpu.r[rm];
    uint32_t result = 0 - op2;
    cpu.r[rd] = result;
    update_sub_flags(0, op2, result);
}

// ============================================================================
// IMPORTANT INSTRUCTIONS
// ============================================================================

void instr_cmn_reg(uint16_t instr) {
    uint8_t rn = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t op1 = cpu.r[rn];
    uint32_t op2 = cpu.r[rm];
    uint32_t result = op1 + op2;
    update_add_flags(op1, op2, result);
}

void instr_teq_reg(uint16_t instr) {
    uint8_t rn = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t result = cpu.r[rn] ^ cpu.r[rm];
    update_nz_flags(result);
}

void instr_svc(uint16_t instr) {
    uint8_t imm = instr & 0xFF;
    if (cpu.debug_enabled) {
        printf("[CPU] SVC #%d at 0x%08X\n", imm, cpu.r[15]);
    }
    (void)imm;
    /* Advance PC past SVC so return address is next instruction */
    cpu.r[15] += 2;
    /* Trigger SVCall exception (vector 11) */
    cpu_exception_entry(EXC_SVCALL);
    pc_updated = 1;
}

void instr_add_high_reg(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x0F;
    uint8_t reg_dst = ((instr >> 4) & 0x08) | (instr & 0x07);
    uint32_t result = cpu.r[reg_dst] + cpu.r[reg_src];

    if (reg_dst == 15) {
        cpu.r[15] = result & ~1u;
        pc_updated = 1;
    } else {
        cpu.r[reg_dst] = result;
    }
}

void instr_cmp_high_reg(uint16_t instr) {
    instr_cmp_reg_reg(instr);
}

void instr_mov_high_reg(uint16_t instr) {
    instr_mov_reg(instr);
}

void instr_msr(uint32_t instr) {
    (void)instr;
    if (cpu.debug_enabled) {
        printf("[INSTR] MSR instruction at 0x%08X (not implemented)\n", cpu.r[15]);
    }
}

void instr_mrs(uint32_t instr) {
    uint8_t rd = (instr >> 8) & 0x0F;
    cpu.r[rd] = cpu.xpsr;
    if (cpu.debug_enabled) {
        printf("[INSTR] MRS R%u = 0x%08X (XPSR)\n", rd, cpu.r[rd]);
    }
}

// ============================================================================
// OPTIONAL INSTRUCTIONS
// ============================================================================

void instr_it(uint16_t instr) {
    (void)instr;
    printf("[CPU] IT instruction at 0x%08X (limited support)\n", cpu.r[15]);
}

void instr_dsb(uint32_t instr) { (void)instr; }
void instr_dmb(uint32_t instr) { (void)instr; }
void instr_isb(uint32_t instr) { (void)instr; }

void instr_wfi(uint16_t instr) {
    (void)instr;
    if (cpu.debug_enabled) {
        printf("[CPU] WFI - Will check for interrupt on next cycle\n");
    }
}

void instr_wfe(uint16_t instr) {
    (void)instr;
}

void instr_sev(uint16_t instr) {
    (void)instr;
}

void instr_yield(uint16_t instr) {
    (void)instr;
}

void instr_sxtb(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    cpu.r[rd] = sign_extend_8(cpu.r[rm] & 0xFF);
}

void instr_sxth(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    cpu.r[rd] = sign_extend_16(cpu.r[rm] & 0xFFFF);
}

void instr_uxtb(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    cpu.r[rd] = cpu.r[rm] & 0xFF;
}

void instr_uxth(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    cpu.r[rd] = cpu.r[rm] & 0xFFFF;
}

void instr_rev(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t value = cpu.r[rm];
    cpu.r[rd] = ((value & 0xFF) << 24) | ((value & 0xFF00) << 8) |
                ((value & 0xFF0000) >> 8) | ((value >> 24) & 0xFF);
}

void instr_rev16(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t value = cpu.r[rm];
    cpu.r[rd] = ((value & 0x00FF00FF) << 8) | ((value & 0xFF00FF00) >> 8);
}

void instr_revsh(uint16_t instr) {
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint16_t value = cpu.r[rm] & 0xFFFF;
    uint16_t reversed = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8);
    cpu.r[rd] = sign_extend_16(reversed);
}

void instr_add_sp_imm7(uint16_t instr) {
    uint8_t imm7 = instr & 0x7F;
    cpu.r[13] += (imm7 << 2);
}

void instr_sub_sp_imm7(uint16_t instr) {
    uint8_t imm7 = instr & 0x7F;
    cpu.r[13] -= (imm7 << 2);
}

void instr_adr(uint16_t instr) {
    uint8_t rd = (instr >> 8) & 0x07;
    uint8_t imm8 = instr & 0xFF;
    uint32_t pc_aligned = (cpu.r[15] + 4) & ~3;
    cpu.r[rd] = pc_aligned + (imm8 << 2);
}

void instr_add_pc_imm8(uint16_t instr) {
    instr_adr(instr);
}

void instr_cpsid(uint16_t instr) {
    (void)instr;
    cpu.primask = 1;
    if (cpu.debug_asm) {
        printf("[CPSID] Interrupts disabled (PRIMASK=1) at 0x%08X\n", cpu.r[15]);
    }
}

void instr_cpsie(uint16_t instr) {
    (void)instr;
    cpu.primask = 0;
    if (cpu.debug_asm) {
        printf("[CPSIE] Interrupts enabled (PRIMASK=0) at 0x%08X\n", cpu.r[15]);
    }
}

/* ============ Special Instructions ============ */

void instr_bkpt(uint16_t instr) {
    uint8_t imm = instr & 0xFF;
    printf("[CPU] BKPT #%d at 0x%08X\n", imm, cpu.r[15]);
    printf("[CPU] Program halted at breakpoint\n");
    printf("Register State:");

    for (int i = 0; i < 13; i++) {
        printf("  R%-2d=0x%08X  ", i, cpu.r[i]);
        if ((i + 1) % 4 == 0) printf("\n");
    }
    printf("  R13(SP)=0x%08X  R14(LR)=0x%08X  R15(PC)=0x%08X\n",
           cpu.r[13], cpu.r[14], cpu.r[15]);
    printf("  XPSR=0x%08X\n", cpu.xpsr);

    cpu.r[15] = 0xFFFFFFFF;
    pc_updated = 1;
}

void instr_nop(uint16_t instr) {
    (void)instr;
}

void instr_udf(uint16_t instr) {
    (void)instr;
    printf("[CPU] UDF (Undefined Instruction) at 0x%08X\n", cpu.r[15]);
    cpu.r[15] = 0xFFFFFFFF;
    pc_updated = 1;
}

void instr_unimplemented(uint16_t instr) {
    printf("[CPU] 0x%08X: UNIMPLEMENTED 0x%04X\n", cpu.r[15], instr);
    cpu.r[15] = 0xFFFFFFFF;
    pc_updated = 1;
}
