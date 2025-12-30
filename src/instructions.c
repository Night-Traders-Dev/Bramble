#include <stdio.h>
#include <stdint.h>
#include "emulator.h"
#include "instructions.h"
#include "nvic.h"

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
    cpu.xpsr = 0;

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
    cpu.xpsr = 0;

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
    // MOV with high register support: 0100 0110 D:1 m:4 d:3
    uint8_t rd = ((instr >> 4) & 0x8) | (instr & 0x7);  // D:Rd[2:0]
    uint8_t rm = (instr >> 3) & 0xF;

    uint32_t value = cpu.r[rm];

    if (rd == 15) {
        // Branch if writing to PC
        cpu.r[15] = value & ~1;  // Clear Thumb bit
    } else {
        cpu.r[rd] = value;
    }
    // MOV does NOT update flags
}

void instr_add_reg_reg(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x0F;
    uint8_t reg_dst = ((instr >> 4) & 0x08) | (instr & 0x07);
    uint32_t op1 = cpu.r[reg_dst];
    uint32_t op2 = cpu.r[reg_src];
    uint32_t result = op1 + op2;
    cpu.r[reg_dst] = result;
    // High register ADD does not update flags
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
    uint8_t reg_dst = ((instr >> 4) & 0x08) | (instr & 0x07);  // Support high regs
    uint32_t op1 = cpu.r[reg_dst];
    uint32_t op2 = cpu.r[reg_src];
    uint32_t result = op1 - op2;
    update_sub_flags(op1, op2, result);
}

/* ============ Memory Instructions ============ */

void instr_ldr_imm5(uint16_t instr) {
    // LDR Rd, [Rn, #imm5] - Encoding: 0110 1 iiiii nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + (imm5 << 2);  // Scaled by 4
    cpu.r[rd] = mem_read32(addr);
}

void instr_str_imm5(uint16_t instr) {
    // STR Rd, [Rn, #imm5] - Encoding: 0110 0 iiiii nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + (imm5 << 2);  // Scaled by 4
    mem_write32(addr, cpu.r[rd]);
}

void instr_ldr_pc_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    uint32_t addr = (cpu.r[15] & ~3) + (imm * 4) + 4;
    cpu.r[reg] = mem_read32(addr);
    //printf("ldr_pc 0x%08X\n", cpu.r[reg]);
}

void instr_ldr_sp_imm8(uint16_t instr) {
    // LDR Rd, [SP, #imm8] - Encoding: 1001 1 ddd iiii iiii
    uint8_t rd = (instr >> 8) & 0x07;
    uint8_t imm8 = instr & 0xFF;
    uint32_t addr = cpu.r[13] + (imm8 << 2);  // SP is R13, scaled by 4
    cpu.r[rd] = mem_read32(addr);
}

void instr_str_sp_imm8(uint16_t instr) {
    // STR Rd, [SP, #imm8] - Encoding: 1001 0 ddd iiii iiii
    uint8_t rd = (instr >> 8) & 0x07;
    uint8_t imm8 = instr & 0xFF;
    uint32_t addr = cpu.r[13] + (imm8 << 2);  // SP is R13, scaled by 4
    mem_write32(addr, cpu.r[rd]);
}

void instr_str_reg_offset(uint16_t instr) {
    uint8_t reg_src = instr & 0x07;
    uint8_t reg_dst = (instr >> 3) & 0x07;
    uint8_t imm = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[reg_dst] + (imm * 4);
    mem_write32(addr, cpu.r[reg_src]);
}

void instr_ldr_reg_offset(uint16_t instr) {
    uint8_t reg_dst = instr & 0x07;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t imm = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[reg_src] + (imm * 4);
    cpu.r[reg_dst] = mem_read32(addr);
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
    uint8_t M = (instr >> 8) & 0x1;  /* bit 8: store LR? */

    uint32_t sp = cpu.r[13];

    if (M) {
        sp -= 4;
        mem_write32(sp, cpu.r[14]);  /* Push LR */
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
    uint8_t P = (instr >> 8) & 0x1;  /* bit 8: load PC? */

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
            sp += 4;
            cpu.r[13] = sp;
            return;  /* PC already updated */
        }
        
        cpu.r[15] = pc_val & ~1;  /* Clear thumb bit */
        sp += 4;
    }

    cpu.r[13] = sp;
}


/* ============ Branch Instructions ============ */

void instr_b_uncond(uint16_t instr) {
    /* B encoding: 11100 offset[10:0] */
    int16_t offset = instr & 0x07FF;  /* Extract 11-bit offset */
    /* Sign extend from 11 bits */
    if (offset & 0x0400) {  /* If bit 10 set */
        offset |= 0xF800;   /* Sign extend to 16 bits */
    }
    /* Sign extend to 32 bits */
    int32_t signed_offset = (int32_t)offset;
    /* Multiply by 2 (Thumb instructions are 2-byte aligned) */
    signed_offset <<= 1;
    /* Branch: PC = PC + 4 + offset */
    cpu.r[15] += 4 + signed_offset;  // ← FIXED: Added +4
}

void instr_bcond(uint16_t instr) {
    uint8_t cond = (instr >> 8) & 0x0F;
    int8_t offset = instr & 0xFF;
    int32_t signed_offset = (int32_t)(offset << 24) >> 23;

    int take_branch = 0;
    switch (cond) {
        case 0x0: /* EQ - Z set */
            take_branch = (cpu.xpsr & FLAG_Z) != 0;
            break;
        case 0x1: /* NE - Z clear */
            take_branch = (cpu.xpsr & FLAG_Z) == 0;
            break;
        case 0x2: /* CS/HS - C set */
            take_branch = (cpu.xpsr & FLAG_C) != 0;
            break;
        case 0x3: /* CC/LO - C clear */
            take_branch = (cpu.xpsr & FLAG_C) == 0;
            break;
        case 0x4: /* MI - N set */
            take_branch = (cpu.xpsr & FLAG_N) != 0;
            break;
        case 0x5: /* PL - N clear */
            take_branch = (cpu.xpsr & FLAG_N) == 0;
            break;
        case 0x6: /* VS - V set */
            take_branch = (cpu.xpsr & FLAG_V) != 0;
            break;
        case 0x7: /* VC - V clear */
            take_branch = (cpu.xpsr & FLAG_V) == 0;
            break;
        case 0x8: /* HI - C set and Z clear */
            take_branch = ((cpu.xpsr & FLAG_C) != 0) && ((cpu.xpsr & FLAG_Z) == 0);
            break;
        case 0x9: /* LS - C clear or Z set */
            take_branch = ((cpu.xpsr & FLAG_C) == 0) || ((cpu.xpsr & FLAG_Z) != 0);
            break;
        case 0xA: /* GE - N == V */
            take_branch = ((cpu.xpsr & FLAG_N) != 0) == ((cpu.xpsr & FLAG_V) != 0);
            break;
        case 0xB: /* LT - N != V */
            take_branch = ((cpu.xpsr & FLAG_N) != 0) != ((cpu.xpsr & FLAG_V) != 0);
            break;
        case 0xC: /* GT - Z clear and N == V */
            take_branch = ((cpu.xpsr & FLAG_Z) == 0) && 
                         (((cpu.xpsr & FLAG_N) != 0) == ((cpu.xpsr & FLAG_V) != 0));
            break;
        case 0xD: /* LE - Z set or N != V */
            take_branch = ((cpu.xpsr & FLAG_Z) != 0) || 
                         (((cpu.xpsr & FLAG_N) != 0) != ((cpu.xpsr & FLAG_V) != 0));
            break;
        default:
            break;
    }

    if (take_branch) {
        cpu.r[15] += 4 + signed_offset;
    } else {
        cpu.r[15] += 2;
    }
}


void instr_bl(uint16_t instr) {
    /* Read second halfword */
    uint16_t instr2 = mem_read16(cpu.r[15] + 2);
    
    /* Extract signed 22-bit offset */
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

    /* LR = address of next instruction */
    cpu.r[14] = (pc + 4) | 1;  /* Set thumb bit */

    cpu.r[15] = target & ~1;
}


// upper = first halfword at PC
// lower = second halfword at PC+2
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

    // sign extend from bit 24
    if (offset & (1u << 24)) offset |= 0xFE000000;

    uint32_t pc = cpu.r[15];

    cpu.r[14] = (pc + 4) | 1u;
    cpu.r[15] = (pc + 4 + offset) & ~1u; // keep PC aligned, clear Thumb bit storage
}




/**
 * BX Rm - Branch with Exchange (and optional exception return)
 * 
 * Standard behavior: Branch to address in Rm
 * Special behavior: If LR contains magic value (0xFFFFFFF9), perform exception return
 * 
 * Exception Return (Cortex-M0+):
 *   Magic LR values indicate return from ISR:
 *   - 0xFFFFFFF9: Return to Thread mode (main interrupt return path)
 *   - 0xFFFFFFF1: Return to Handler mode (nested exception)
 * 
 * When BX LR with magic value:
 *   1. Call cpu_exception_return() to restore context frame
 *   2. Pop register state from stack
 *   3. Clear active exception bits
 *   4. PC restored to point of interrupt
 * 
 * This is the standard ARM Cortex-M method for ISR return.
 */
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
        /* This is an exception return - call dedicated handler */
        cpu_exception_return(target);
        return;  /* PC already updated by cpu_exception_return() */
    }

    /* Standard BX behavior: Branch to target (clear Thumb bit) */
    if (cpu.debug_asm) {
        printf("[BX] Standard: jumping to 0x%08X\n", target);
    }
    cpu.r[15] = target & ~1u;
    
}


void instr_blx(uint16_t instr) {
    // BLX Rm - Branch with Link and Exchange
    uint8_t rm = (instr >> 3) & 0x0F;
    uint32_t target = cpu.r[rm];
    
    // Save return address in LR with Thumb bit
    cpu.r[14] = (cpu.r[15] + 2) | 1;
    
    // Branch to target (clear Thumb bit)
    cpu.r[15] = target & ~1;
}

// ============================================================================
// ESSENTIAL INSTRUCTIONS
// ============================================================================

/* ============ Byte Operations ============ */

void instr_ldrb_imm5(uint16_t instr) {
    // LDRB Rd, [Rn, #imm5] - Encoding: 0111 1 iiiii nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + imm5;  // NOT scaled for byte access
    cpu.r[rd] = mem_read8(addr);
}

void instr_ldrb_reg_offset(uint16_t instr) {
    // LDRB Rd, [Rn, Rm] - Encoding: 0101 110 mmm nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    cpu.r[rd] = mem_read8(addr);
}

void instr_ldrsb_reg_offset(uint16_t instr) {
    // LDRSB Rd, [Rn, Rm] - Encoding: 0101 011 mmm nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    uint8_t value = mem_read8(addr);
    cpu.r[rd] = sign_extend_8(value);  // Sign extend
}

void instr_strb_imm5(uint16_t instr) {
    // STRB Rd, [Rn, #imm5] - Encoding: 0111 0 iiiii nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + imm5;  // NOT scaled for byte access
    mem_write8(addr, cpu.r[rd] & 0xFF);
}

void instr_strb_reg_offset(uint16_t instr) {
    // STRB Rd, [Rn, Rm] - Encoding: 0101 010 mmm nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    mem_write8(addr, cpu.r[rd] & 0xFF);
}

/* ============ Halfword Operations ============ */

void instr_ldrh_imm5(uint16_t instr) {
    // LDRH Rd, [Rn, #imm5] - Encoding: 1000 1 iiiii nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + (imm5 << 1);  // Scaled by 2
    cpu.r[rd] = mem_read16(addr);
}

void instr_ldrh_reg_offset(uint16_t instr) {
    // LDRH Rd, [Rn, Rm] - Encoding: 0101 101 mmm nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    cpu.r[rd] = mem_read16(addr);
}

void instr_ldrsh_reg_offset(uint16_t instr) {
    // LDRSH Rd, [Rn, Rm] - Encoding: 0101 111 mmm nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t rm = (instr >> 6) & 0x07;
    uint32_t addr = cpu.r[rn] + cpu.r[rm];
    uint16_t value = mem_read16(addr);
    cpu.r[rd] = sign_extend_16(value);  // Sign extend
}

void instr_strh_imm5(uint16_t instr) {
    // STRH Rd, [Rn, #imm5] - Encoding: 1000 0 iiiii nnn ddd
    uint8_t rd = instr & 0x07;
    uint8_t rn = (instr >> 3) & 0x07;
    uint8_t imm5 = (instr >> 6) & 0x1F;
    uint32_t addr = cpu.r[rn] + (imm5 << 1);  // Scaled by 2
    mem_write16(addr, cpu.r[rd] & 0xFFFF);
}

void instr_strh_reg_offset(uint16_t instr) {
    // STRH Rd, [Rn, Rm] - Encoding: 0101 001 mmm nnn ddd
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
        // Update carry before shift
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
        // Carry = bit 31
        if (cpu.r[reg_src] & 0x80000000) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
    } else {
        // Update carry before shift
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
        imm = 32;
    }
    
    // Update carry before shift
    if (cpu.r[reg_src] & (1 << (imm - 1))) {
        cpu.xpsr |= FLAG_C;
    } else {
        cpu.xpsr &= ~FLAG_C;
    }
    
    cpu.r[reg_dst] = ((int32_t)cpu.r[reg_src]) >> imm;
    update_nz_flags(cpu.r[reg_dst]);
}

void instr_lsls_reg(uint16_t instr) {
    // LSLS Rd, Rs - Encoding: 0100 0000 10 sss ddd
    uint8_t rd = instr & 0x07;
    uint8_t rs = (instr >> 3) & 0x07;
    uint8_t shift = cpu.r[rs] & 0xFF;
    
    if (shift == 0) {
        // No change, no flag update
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
    // LSRS Rd, Rs - Encoding: 0100 0000 11 sss ddd
    uint8_t rd = instr & 0x07;
    uint8_t rs = (instr >> 3) & 0x07;
    uint8_t shift = cpu.r[rs] & 0xFF;
    
    if (shift == 0) {
        // No change
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
    // ASRS Rd, Rs - Encoding: 0100 0001 00 sss ddd
    uint8_t rd = instr & 0x07;
    uint8_t rs = (instr >> 3) & 0x07;
    uint8_t shift = cpu.r[rs] & 0xFF;
    
    if (shift == 0) {
        // No change
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
    // RORS Rd, Rs - Encoding: 0100 0001 11 sss ddd
    uint8_t rd = instr & 0x07;
    uint8_t rs = (instr >> 3) & 0x07;
    uint8_t shift = cpu.r[rs] & 0x1F;  // Only use bottom 5 bits
    
    if (shift == 0) {
        // No change
    } else {
        uint32_t result = (cpu.r[rd] >> shift) | (cpu.r[rd] << (32 - shift));
        if (result & 0x80000000) cpu.xpsr |= FLAG_C;
        else cpu.xpsr &= ~FLAG_C;
        cpu.r[rd] = result;
        update_nz_flags(result);
    }
}

/* ============ Multiplication ============ */

void instr_muls(uint16_t instr) {
    // MULS Rd, Rm - Encoding: 0100 0011 01 mmm ddd
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    
    cpu.r[rd] = cpu.r[rd] * cpu.r[rm];
    update_nz_flags(cpu.r[rd]);
    // C and V flags are unpredictable after MUL
}

// ============================================================================
// IMPORTANT INSTRUCTIONS
// ============================================================================

void instr_cmn_reg(uint16_t instr) {
    // CMN Rn, Rm - Compare Negative (adds for flags)
    // Encoding: 0100 0010 11 mmm nnn
    uint8_t rn = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t op1 = cpu.r[rn];
    uint32_t op2 = cpu.r[rm];
    uint32_t result = op1 + op2;
    update_add_flags(op1, op2, result);
}

void instr_teq_reg(uint16_t instr) {
    // TEQ Rn, Rm - Test Equivalence (XOR for flags)
    // Encoding: 1110 1010 1001 nnnn 0000 dddd 0000 mmmm (32-bit)
    // Simplified 16-bit version for compatibility
    uint8_t rn = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t result = cpu.r[rn] ^ cpu.r[rm];
    update_nz_flags(result);
}

void instr_svc(uint16_t instr) {
    // SVC #imm8 - Supervisor Call
    uint8_t imm = instr & 0xFF;
    printf("[CPU] SVC #%d at 0x%08X\n", imm, cpu.r[15]);
    // In a real implementation, this would trigger an exception
    // For now, just log it
}

void instr_add_high_reg(uint16_t instr) {
    // ADD with high registers (already handled by instr_add_reg_reg)
    instr_add_reg_reg(instr);
}

void instr_cmp_high_reg(uint16_t instr) {
    // CMP with high registers (already handled by instr_cmp_reg_reg)
    instr_cmp_reg_reg(instr);
}

void instr_mov_high_reg(uint16_t instr) {
    // MOV with high registers (already handled by instr_mov_reg)
    instr_mov_reg(instr);
}

void instr_msr(uint32_t instr) {
    (void)instr; /* Mark unused */
    if (cpu.debug_enabled) {
        printf("[INSTR] MSR instruction at 0x%08X (not implemented)\n", cpu.r[15]);
    }
    // MSR - Move to Special Register (32-bit instruction)
    // Encoding: 1111 0011 1000 nnnn 1000 rrrr xxxx xxxx
    // Limited implementation for ARMv6-M
}

void instr_mrs(uint32_t instr) {
    // MRS - Move from Special Register (32-bit instruction)
    // Encoding: 1111 0011 1110 1111 1000 dddd xxxx xxxx
    uint8_t rd = (instr >> 8) & 0x0F;
    cpu.r[rd] = cpu.xpsr;  // Read XPSR
    if (cpu.debug_enabled) {
        printf("[INSTR] MRS R%u = 0x%08X (XPSR)\n", rd, cpu.r[rd]);
    }
}

// ============================================================================
// OPTIONAL INSTRUCTIONS
// ============================================================================

void instr_it(uint16_t instr) {
    (void)instr; /* Mark unused */
    // IT - If-Then (conditional execution block)
    // Encoding: 1011 1111 cccc mask
    // ARMv6-M has limited IT support
    printf("[CPU] IT instruction at 0x%08X (limited support)\n", cpu.r[15]);
}

void instr_dsb(uint32_t instr) {
    // DSB - Data Synchronization Barrier
    (void)instr;
    // In emulator, this is a no-op
}

void instr_dmb(uint32_t instr) {
    // DMB - Data Memory Barrier
    (void)instr;
    // In emulator, this is a no-op
}

void instr_isb(uint32_t instr) {
    // ISB - Instruction Synchronization Barrier
    (void)instr;
    // In emulator, this is a no-op
}

void instr_wfi(uint16_t instr) {
    /* Wait For Interrupt
     * 
     * In real hardware: CPU enters low-power state, wakes only on interrupt
     * In emulator: We simulate by always advancing PC
     * The cpu_step() function checks for interrupts at the start
     * If interrupt pending, it will enter exception handler
     * If no interrupt, WFI just advances like a normal instruction
     */
    (void)instr;
    
    if (cpu.debug_enabled) {
        printf("[CPU] WFI - Will check for interrupt on next cycle\n");
    }
    
    /* Simply advance PC - cpu_step() will handle interrupt on next cycle */
    /* This is the correct behavior: WFI doesn't block indefinitely */
    cpu.r[15] += 2;
}

void instr_wfe(uint16_t instr) {
    // WFE - Wait For Event
    (void)instr;
    printf("[CPU] WFE at 0x%08X (no-op in emulator)\n", cpu.r[15]);
}

void instr_sev(uint16_t instr) {
    // SEV - Send Event
    (void)instr;
    printf("[CPU] SEV at 0x%08X (no-op in emulator)\n", cpu.r[15]);
}

void instr_yield(uint16_t instr) {
    // YIELD - Hint instruction
    (void)instr;
    // No-op in emulator
}

void instr_sxtb(uint16_t instr) {
    // SXTB Rd, Rm - Sign Extend Byte
    // Encoding: 1011 0010 01 mmm ddd
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    cpu.r[rd] = sign_extend_8(cpu.r[rm] & 0xFF);
}

void instr_sxth(uint16_t instr) {
    // SXTH Rd, Rm - Sign Extend Halfword
    // Encoding: 1011 0010 00 mmm ddd
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    cpu.r[rd] = sign_extend_16(cpu.r[rm] & 0xFFFF);
}

void instr_uxtb(uint16_t instr) {
    // UXTB Rd, Rm - Zero Extend Byte
    // Encoding: 1011 0010 11 mmm ddd
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    cpu.r[rd] = cpu.r[rm] & 0xFF;
}

void instr_uxth(uint16_t instr) {
    // UXTH Rd, Rm - Zero Extend Halfword
    // Encoding: 1011 0010 10 mmm ddd
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    cpu.r[rd] = cpu.r[rm] & 0xFFFF;
}

void instr_rev(uint16_t instr) {
    // REV Rd, Rm - Reverse bytes
    // Encoding: 1011 1010 00 mmm ddd
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t value = cpu.r[rm];
    cpu.r[rd] = ((value & 0xFF) << 24) | ((value & 0xFF00) << 8) |
                ((value & 0xFF0000) >> 8) | ((value >> 24) & 0xFF);
}

void instr_rev16(uint16_t instr) {
    // REV16 Rd, Rm - Reverse bytes in each halfword
    // Encoding: 1011 1010 01 mmm ddd
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint32_t value = cpu.r[rm];
    cpu.r[rd] = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8) |
                ((value & 0xFF0000) << 8) | ((value >> 24) << 24);
}

void instr_revsh(uint16_t instr) {
    // REVSH Rd, Rm - Reverse Signed Halfword
    // Encoding: 1011 1010 11 mmm ddd
    uint8_t rd = instr & 0x07;
    uint8_t rm = (instr >> 3) & 0x07;
    uint16_t value = cpu.r[rm] & 0xFFFF;
    uint16_t reversed = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8);
    cpu.r[rd] = sign_extend_16(reversed);
}

void instr_add_sp_imm7(uint16_t instr) {
    // ADD SP, #imm7 - Encoding: 1011 0000 0 iiiiiii
    uint8_t imm7 = instr & 0x7F;
    cpu.r[13] += (imm7 << 2);  // Scaled by 4
}

void instr_sub_sp_imm7(uint16_t instr) {
    // SUB SP, #imm7 - Encoding: 1011 0000 1 iiiiiii
    uint8_t imm7 = instr & 0x7F;
    cpu.r[13] -= (imm7 << 2);  // Scaled by 4
}

void instr_adr(uint16_t instr) {
    // ADR Rd, label - Form PC-relative address
    // Encoding: 1010 0 ddd iiii iiii
    uint8_t rd = (instr >> 8) & 0x07;
    uint8_t imm8 = instr & 0xFF;
    uint32_t pc_aligned = (cpu.r[15] + 4) & ~3;
    cpu.r[rd] = pc_aligned + (imm8 << 2);
}

void instr_add_pc_imm8(uint16_t instr) {
    // ADD Rd, PC, #imm8 - Same as ADR
    instr_adr(instr);
}

void instr_cpsid(uint16_t instr) {
    // CPSID - Disable interrupts
    (void)instr;
    if (cpu.debug_asm) {
        printf("[CPSID] Disable interrupts at 0x%08X (no-op in emulator)\n", cpu.r[15]);
    }
}

void instr_cpsie(uint16_t instr) {
    // CPSIE - Enable interrupts
    (void)instr;
    if (cpu.debug_asm) {
        printf("[CPSIE] Enable interrupts at 0x%08X (no-op in emulator)\n", cpu.r[15]);
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

    /* Halt execution */
    cpu.r[15] = 0xFFFFFFFF;
}

void instr_nop(uint16_t instr) {
    (void)instr;  /* Suppress unused parameter warning */
    /* Do nothing - PC will increment normally */
}

void instr_udf(uint16_t instr) {
    (void)instr;  /* Suppress unused parameter warning */
    printf("[CPU] UDF (Undefined Instruction) at 0x%08X\n", cpu.r[15]);
    /* Halt by setting PC out of bounds */
    cpu.r[15] = 0xFFFFFFFF;
}

void instr_unimplemented(uint16_t instr) {
    printf("[CPU] 0x%08X: UNIMPLEMENTED 0x%04X\n", cpu.r[15], instr);
    /* Halt */
    cpu.r[15] = 0xFFFFFFFF;
}
