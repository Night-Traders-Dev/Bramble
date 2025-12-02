#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "emulator.h"

cpu_state_t cpu = {0};

void cpu_init(void) {
    memset(cpu.r, 0, sizeof(cpu.r));
    cpu.xpsr = 0;
    cpu.step_count = 0;
}

int cpu_is_halted(void) {
    return 0;
}

void cpu_step(void) {
    if (cpu.r[15] >= FLASH_BASE + FLASH_SIZE) {
        printf("[CPU] ERROR: PC out of bounds (0x%08X)\n", cpu.r[15]);
        return;
    }

    uint16_t instr = mem_read16(cpu.r[15]);
    
    if (instr == 0x0000) {
        if (cpu.step_count % 10000 == 0) {
            printf("[CPU] 0x%08X: NOP (step %u)\n", cpu.r[15], cpu.step_count);
        }
        cpu.r[15] += 2;
        cpu.step_count++;
        return;
    }

    /* MOVS Rd, #imm8 (0010 0Rdii iiiiii) */
    if ((instr & 0xF800) == 0x2000) {
        uint8_t reg = (instr >> 8) & 0x07;
        uint8_t imm = instr & 0xFF;
        cpu.r[reg] = imm;
    }
    
    /* LDR Rd, [PC, #imm8] (0100 1Rdii iiiiii) */
    else if ((instr & 0xF800) == 0x4800) {
        uint8_t reg = (instr >> 8) & 0x07;
        uint8_t imm = instr & 0xFF;
        uint32_t addr = (cpu.r[15] & ~3) + (imm * 4) + 4;
        cpu.r[reg] = mem_read32(addr);
    }
    
    /* STR Rd, [Rn, #imm5] (0110 0iiii nnRdRd) */
    else if ((instr & 0xF800) == 0x6000) {
        uint8_t reg_src = (instr >> 0) & 0x07;
        uint8_t reg_dst = (instr >> 3) & 0x07;
        uint8_t imm = (instr >> 6) & 0x1F;
        uint32_t addr = cpu.r[reg_dst] + (imm * 4);
        mem_write32(addr, cpu.r[reg_src]);
    }
    
    /* STRB Rd, [Rn, #imm5] (0111 0iiii nnRdRd) */
    else if ((instr & 0xF800) == 0x7000) {
        uint8_t reg_src = instr & 0x07;
        uint8_t reg_dst = (instr >> 3) & 0x07;
        uint8_t imm = (instr >> 6) & 0x1F;
        uint32_t addr = cpu.r[reg_dst] + imm;
        uint8_t val = cpu.r[reg_src] & 0xFF;
        mem_write32(addr, val);
    }
    
    /* ADD Rd, Rn (0100 01dd nnnnddd) */
    else if ((instr & 0xFFC0) == 0x4400) {
        uint8_t reg_src = (instr >> 3) & 0x0F;
        uint8_t reg_dst = ((instr >> 4) & 0x08) | (instr & 0x07);
        cpu.r[reg_dst] += cpu.r[reg_src];
    }
    
    /* SUB Rd, Rn (0001 10nn nnRdRd) */
    else if ((instr & 0xFE00) == 0x1A00) {
        uint8_t reg_src = (instr >> 3) & 0x07;
        uint8_t reg_dst = instr & 0x07;
        cpu.r[reg_dst] -= cpu.r[reg_src];
    }
    
    /* SUBS Rd, #imm8 (0011 1Rdd iiiiiiii) */
    else if ((instr & 0xF800) == 0x3800) {
        uint8_t reg = (instr >> 8) & 0x07;
        uint8_t imm = instr & 0xFF;
        cpu.r[reg] -= imm;
    }
    
    /* ADDS Rd, #imm8 (0011 0Rdd iiiiiiii) */
    else if ((instr & 0xF800) == 0x3000) {
        uint8_t reg = (instr >> 8) & 0x07;
        uint8_t imm = instr & 0xFF;
        cpu.r[reg] += imm;
    }
    
    /* PUSH {Rlist} (1011 0100 Rlist) */
    else if ((instr & 0xFF00) == 0xB400) {
        uint8_t rlist = instr & 0xFF;
        for (int i = 0; i < 8; i++) {
            if (rlist & (1 << i)) {
                cpu.r[13] -= 4;
                mem_write32(cpu.r[13], cpu.r[i]);
            }
        }
    }
    
    /* POP {Rlist} (1011 1100 Rlist) */
    else if ((instr & 0xFF00) == 0xBC00) {
        uint8_t rlist = instr & 0xFF;
        for (int i = 0; i < 8; i++) {
            if (rlist & (1 << i)) {
                cpu.r[i] = mem_read32(cpu.r[13]);
                cpu.r[13] += 4;
            }
        }
    }
    
    /* BKPT #imm8 (1011 1110 iiii iiii) - BREAKPOINT INSTRUCTION */
    else if ((instr & 0xFF00) == 0xBE00) {
        return;  /* Halt execution */
    }
    
    /* CMP Rd, Rn (0100 0010 00nn nddd) */
    else if ((instr & 0xFFC0) == 0x4280) {
        uint8_t reg_src = (instr >> 3) & 0x0F;
        uint8_t reg_dst = instr & 0x07;
        uint32_t result = cpu.r[reg_dst] - cpu.r[reg_src];
        cpu.xpsr = 0;
        if (result == 0) cpu.xpsr |= 0x60000000;
        if ((int32_t)result < 0) cpu.xpsr |= 0x80000000;
    }
    
    /* STMIA Rn!, {Rlist} (1100 0nnn Rlist) - Store Multiple */
    else if ((instr & 0xF800) == 0xC000) {
        uint8_t reg_base = (instr >> 8) & 0x07;
        uint8_t rlist = instr & 0xFF;
        for (int i = 0; i < 8; i++) {
            if (rlist & (1 << i)) {
                mem_write32(cpu.r[reg_base], cpu.r[i]);
                cpu.r[reg_base] += 4;
            }
        }
    }
    
    /* LDMIA Rn!, {Rlist} (1100 1nnn Rlist) - Load Multiple */
    else if ((instr & 0xF800) == 0xC800) {
        uint8_t reg_base = (instr >> 8) & 0x07;
        uint8_t rlist = instr & 0xFF;
        for (int i = 0; i < 8; i++) {
            if (rlist & (1 << i)) {
                cpu.r[i] = mem_read32(cpu.r[reg_base]);
                cpu.r[reg_base] += 4;
            }
        }
    }
    
    /* BEQ/BNE/BCS/BCC/BMI/BPL/BVS/BVC (1101 cccc iiii iiii) */
    else if ((instr & 0xF000) == 0xD000) {
        uint8_t cond = (instr >> 8) & 0x0F;
        int8_t offset = instr & 0xFF;
        offset = (offset << 24) >> 23;
        
        int branch_taken = 0;
        switch (cond) {
            case 0x0: branch_taken = (cpu.xpsr & 0x60000000) != 0; break;
            case 0x1: branch_taken = (cpu.xpsr & 0x60000000) == 0; break;
            case 0x2: branch_taken = (cpu.xpsr & 0x20000000) != 0; break;
            case 0x3: branch_taken = (cpu.xpsr & 0x20000000) == 0; break;
            case 0x4: branch_taken = (cpu.xpsr & 0x80000000) != 0; break;
            case 0x5: branch_taken = (cpu.xpsr & 0x80000000) == 0; break;
            case 0x6: branch_taken = (cpu.xpsr & 0x10000000) != 0; break;
            case 0x7: branch_taken = (cpu.xpsr & 0x10000000) == 0; break;
        }
        
        if (branch_taken) {
            cpu.r[15] += offset;
            cpu.step_count++;
            return;
        }
    }
    
    /* BL target (1111 0sss ssss ssss 1111 1sss ssss ssss) */
    else if ((instr & 0xF800) == 0xF000) {
        uint16_t instr2 = mem_read16(cpu.r[15] + 2);
        if ((instr2 & 0xF800) == 0xF800) {
            uint32_t offset = ((instr & 0x07FF) << 12) | ((instr2 & 0x07FF) << 1);
            if (offset & 0x00800000) offset |= 0xFF000000;
            uint32_t target = (cpu.r[15] + 4) + offset;
            cpu.r[14] = (cpu.r[15] + 4) | 1;
            cpu.r[15] = target;
            cpu.step_count++;
            return;
        }
    }
    
    /* BX Rn (0100 0111 0nnn 000) */
    else if ((instr & 0xFF87) == 0x4700) {
        uint8_t reg = (instr >> 3) & 0x0F;
        uint32_t target = cpu.r[reg];
        cpu.r[15] = target & ~1;
        cpu.step_count++;
        return;
    }
    
    else {
        /* Skip unimplemented instructions silently */
    }

    cpu.r[15] += 2;
    cpu.step_count++;
}
