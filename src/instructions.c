#include <stdio.h>
#include <stdint.h>
#include "emulator.h"
#include "instructions.h"

void instr_adds_imm3(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x07;  // 3-bit immediate
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] = cpu.r[reg_src] + imm;
}

void instr_subs_imm3(uint16_t instr) {
    uint8_t imm = (instr >> 6) & 0x07;  // 3-bit immediate
    uint8_t reg_src = (instr >> 3) & 0x07;
    uint8_t reg_dst = instr & 0x07;
    cpu.r[reg_dst] = cpu.r[reg_src] - imm;
}


void instr_movs_imm8(uint16_t instr) {
    uint8_t reg = (instr >> 8) & 0x07;
    uint8_t imm = instr & 0xFF;
    cpu.r[reg] = imm;
}

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

void instr_push(uint16_t instr) {
    uint8_t rlist = instr & 0xFF;
    for (int i = 0; i < 8; i++) {
        if (rlist & (1 << i)) {
            cpu.r[13] -= 4;
            mem_write32(cpu.r[13], cpu.r[i]);
        }
    }
}

void instr_pop(uint16_t instr) {
    uint8_t rlist = instr & 0xFF;
    for (int i = 0; i < 8; i++) {
        if (rlist & (1 << i)) {
            cpu.r[i] = mem_read32(cpu.r[13]);
            cpu.r[13] += 4;
        }
    }
}

void instr_bkpt(uint16_t instr) {
    uint8_t imm = instr & 0xFF;
    printf("[CPU] BKPT #%d at 0x%08X\n", imm, cpu.r[15]);
    printf("[CPU] Program halted at breakpoint\n");
    printf("\nRegister State:\n");
    for (int i = 0; i < 13; i++) {
        printf("  R%-2d=0x%08X  ", i, cpu.r[i]);
        if (i % 4 == 3) printf("\n");
    }
    printf("  R13(SP)=0x%08X  R14(LR)=0x%08X  R15(PC)=0x%08X\n", cpu.r[13], cpu.r[14], cpu.r[15]);
    printf("  XPSR=0x%08X\n", cpu.xpsr);
    /* Halt execution: do not increment PC or step further */
    cpu.r[15] = 0xFFFFFFFF; // Set PC out of range to stop
}

void instr_cmp_reg_reg(uint16_t instr) {
    uint8_t reg_src = (instr >> 3) & 0x0F;
    uint8_t reg_dst = instr & 0x07;
    uint32_t result = cpu.r[reg_dst] - cpu.r[reg_src];
    cpu.xpsr = 0;
    if (result == 0) cpu.xpsr |= 0x60000000; // Z flag
    if ((int32_t)result < 0) cpu.xpsr |= 0x80000000; // N flag
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

void instr_bcond(uint16_t instr) {
    uint8_t cond = (instr >> 8) & 0x0F;
    int8_t offset = instr & 0xFF;
    offset = (offset << 24) >> 23; // Sign extend
    int take_branch = 0;
    switch (cond) {
        case 0x0: take_branch = (cpu.xpsr & 0x60000000) != 0; break; // EQ
        case 0x1: take_branch = (cpu.xpsr & 0x60000000) == 0; break; // NE
        case 0x2: take_branch = (cpu.xpsr & 0x20000000) != 0; break; // CS
        case 0x3: take_branch = (cpu.xpsr & 0x20000000) == 0; break; // CC
        case 0x4: take_branch = (cpu.xpsr & 0x80000000) != 0; break; // MI
        case 0x5: take_branch = (cpu.xpsr & 0x80000000) == 0; break; // PL
        case 0x6: take_branch = (cpu.xpsr & 0x10000000) != 0; break; // VS
        case 0x7: take_branch = (cpu.xpsr & 0x10000000) == 0; break; // VC
        default: take_branch = 0; break;
    }
    if (take_branch) {
        cpu.r[15] += offset;
    }
}

void instr_bl(uint16_t instr) {
    uint16_t instr2 = mem_read16(cpu.r[15] + 2);
    uint32_t offset = ((instr & 0x07FF) << 12) | ((instr2 & 0x07FF) << 1);
    if (offset & 0x00800000) offset |= 0xFF000000; // Sign extend
    uint32_t target = (cpu.r[15] + 4) + offset;
    cpu.r[14] = (cpu.r[15] + 4) | 1; // LR
    cpu.r[15] = target;
}

void instr_bx(uint16_t instr) {
    uint8_t reg = (instr >> 3) & 0x0F;
    uint32_t target = cpu.r[reg];
    cpu.r[15] = target & ~1;
}

void instr_unimplemented(uint16_t instr) {
    printf("[CPU] 0x%08X: UNIMPLEMENTED 0x%04X\n", cpu.r[15], instr);
}

void instr_nop(uint16_t instr) {
    // NOP: do nothing
}

void instr_udf(uint16_t instr) {
    printf("[CPU] UDF instruction encountered at 0x%08X\n", cpu.r[15]);
    // Infinite loop or halt
    cpu.r[15] = 0xFFFFFFFF;
}

/* Priority 1: Shift instructions */

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

/* Priority 1: Bitwise instructions */

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
