#include <stdio.h>
#include <stdint.h>
#include "emulator.h"
#include "instructions.h"

/* ============ ALU Instructions ============ */

void instr_adds_imm3(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x07;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] = cpu.r[reg_src] + imm;
}

void instr_subs_imm3(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x07;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] = cpu.r[reg_src] - imm;
}

void instr_movs_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    cpu.r[reg] = imm;
}

void instr_add_reg_reg(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x0F;
    uint8_t reg_dst = ((instr >> 4) & 0x08) | (instr & 0x07);
    cpu.r[reg_dst] += cpu.r[reg_src];
}

void instr_sub_reg_reg(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] -= cpu.r[reg_src];
}

void instr_cmp_reg_reg(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x0F;
    uint8_t reg_dst = instr & 0x07;
    uint32_t result = cpu.r[reg_dst] - cpu.r[reg_src];
    cpu.xpsr = 0;
    if (result == 0) cpu.xpsr |= 0x60000000;      /* Z flag */
    if ((int32_t)result < 0) cpu.xpsr |= 0x80000000;  /* N flag */
}

/* ============ Bitwise Instructions ============ */

void instr_bitwise_and(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] &= cpu.r[reg_src];
}

void instr_bitwise_eor(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] ^= cpu.r[reg_src];
}

void instr_bitwise_orr(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] |= cpu.r[reg_src];
}

void instr_bitwise_bic(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] &= ~cpu.r[reg_src];
}

void instr_bitwise_mvn(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] = ~cpu.r[reg_src];
}

/* ============ Shift Instructions ============ */

void instr_shift_logical_left(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x1F;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    if (imm == 0) {
        cpu.r[reg_dst] = cpu.r[reg_src];
    } else {
        cpu.r[reg_dst] = cpu.r[reg_src] << imm;
    }
}

void instr_shift_logical_right(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x1F;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    if (imm == 0) {
        cpu.r[reg_dst] = 0;
    } else {
        cpu.r[reg_dst] = cpu.r[reg_src] >> imm;
    }
}

void instr_shift_arithmetic_right(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x1F;
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    if (imm == 0) {
        cpu.r[reg_dst] = ((int32_t)cpu.r[reg_src]) >> 1;
    } else {
        cpu.r[reg_dst] = ((int32_t)cpu.r[reg_src]) >> imm;
    }
}

/* ============ Memory Instructions ============ */

void instr_ldr_pc_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    uint32_t addr = (cpu.r[15] & ~3) + (imm * 4) + 4;
    cpu.r[reg] = mem_read32(addr);
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
    uint8_t rlist = instr & 0xFF;
    uint8_t m = (instr >> 8) & 0x01;  /* M bit: R14 for PUSH */

    for (int i = 0; i < 8; i++) {
        if (rlist & (1 << i)) {
            cpu.r[13] -= 4;
            mem_write32(cpu.r[13], cpu.r[i]);
        }
    }

    /* M=1 means push LR (R14) */
    if (m) {
        cpu.r[13] -= 4;
        mem_write32(cpu.r[13], cpu.r[14]);
    }
}

void instr_pop(uint16_t instr) {
    uint8_t rlist = instr & 0xFF;
    uint8_t p = (instr >> 8) & 0x01;  /* P bit: R15 for POP */

    for (int i = 0; i < 8; i++) {
        if (rlist & (1 << i)) {
            cpu.r[i] = mem_read32(cpu.r[13]);
            cpu.r[13] += 4;
        }
    }

    /* P=1 means pop PC (R15) */
    if (p) {
        cpu.r[15] = mem_read32(cpu.r[13]);
        cpu.r[13] += 4;
        /* Don't increment PC further - POP PC is a branch */
        return;
    }
}


/* ============ Branch Instructions ============ */

void instr_bcond(uint16_t instr) {
    uint8_t cond = (instr >> 8) & 0x0F;
    int8_t offset = instr & 0xFF;
    
    /* Sign extend from 8 bits: shift left 24, arithmetic shift right 23 */
    int32_t signed_offset = (int32_t)(offset << 24) >> 23;
    
    int take_branch = 0;
    switch (cond) {
        case 0x0: take_branch = (cpu.xpsr & 0x60000000) != 0; break; /* EQ (Z=1) */
        case 0x1: take_branch = (cpu.xpsr & 0x60000000) == 0; break; /* NE (Z=0) */
        case 0x2: take_branch = (cpu.xpsr & 0x20000000) != 0; break; /* CS (C=1) */
        case 0x3: take_branch = (cpu.xpsr & 0x20000000) == 0; break; /* CC (C=0) */
        case 0x4: take_branch = (cpu.xpsr & 0x80000000) != 0; break; /* MI (N=1) */
        case 0x5: take_branch = (cpu.xpsr & 0x80000000) == 0; break; /* PL (N=0) */
        case 0x6: take_branch = (cpu.xpsr & 0x10000000) != 0; break; /* VS (V=1) */
        case 0x7: take_branch = (cpu.xpsr & 0x10000000) == 0; break; /* VC (V=0) */
        default: take_branch = 0; break;
    }
    
    if (take_branch) {
        cpu.r[15] += signed_offset;
    }
    /* If branch not taken, PC increments normally in cpu_step */
}

void instr_bl(uint16_t instr) {
    /* BL is a 32-bit instruction encoded as two 16-bit halfwords */
    uint16_t instr2 = mem_read16(cpu.r[15] + 2);
    
    /* Extract offset from both halfwords */
    /* First halfword: 11110 sign Offset[10:0] */
    /* Second halfword: 11111 J1 J2 Offset[9:0] */
    int32_t offset = 0;
    offset |= (instr & 0x07FF) << 12;      /* Upper 11 bits of offset */
    offset |= (instr2 & 0x07FF) << 1;      /* Lower 11 bits of offset */
    
    /* Sign extend from bit 22 */
    if (offset & 0x00400000) {
        offset |= 0xFF800000;
    }
    
    /* Calculate next instruction address and target */
    uint32_t next_pc = cpu.r[15] + 4;  /* Skip both halfwords */
    uint32_t target = next_pc + offset;
    
    /* Save return address in LR with Thumb bit set */
    cpu.r[14] = next_pc | 1;
    
    /* Set PC to target (clear Thumb bit for internal use) */
    cpu.r[15] = target & ~1;
}

void instr_bx(uint16_t instr) {
    uint8_t reg = (instr >> 3) & 0x0F;
    uint32_t target = cpu.r[reg];
    
    /* BX: branch and exchange */
    /* Bit 0 of target indicates Thumb mode (should be 1 for M0+) */
    cpu.r[15] = target & ~1;  /* Clear Thumb bit for internal PC */
}

/* ============ Special Instructions ============ */

void instr_bkpt(uint16_t instr) {
    uint8_t imm = instr & 0xFF;
    printf("[CPU] BKPT #%d at 0x%08X", imm, cpu.r[15]);
    printf("[CPU] Program halted at breakpoint");
    printf("Register State:");

    for (int i = 0; i < 13; i++) {
        printf("  R%-2d=0x%08X  ", i, cpu.r[i]);
        if ((i + 1) % 4 == 0) printf("");
    }
    printf("  R13(SP)=0x%08X  R14(LR)=0x%08X  R15(PC)=0x%08X",
           cpu.r[13], cpu.r[14], cpu.r[15]);
    printf("  XPSR=0x%08X", cpu.xpsr);
    
    /* Halt execution */
    cpu.r[15] = 0xFFFFFFFF;
}

void instr_nop(uint16_t instr) {
    (void)instr;  /* Suppress unused parameter warning */
    /* Do nothing - PC will increment normally */
}

void instr_udf(uint16_t instr) {
    (void)instr;  /* Suppress unused parameter warning */
    printf("[CPU] UDF (Undefined Instruction) at 0x%08X", cpu.r[15]);
    /* Halt by setting PC out of bounds */
    cpu.r[15] = 0xFFFFFFFF;
}

void instr_unimplemented(uint16_t instr) {
    printf("[CPU] 0x%08X: UNIMPLEMENTED 0x%04X", cpu.r[15], instr);
    /* Halt */
    cpu.r[15] = 0xFFFFFFFF;
}
