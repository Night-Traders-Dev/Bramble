#include <stdio.h>
#include <stdint.h>
#include "emulator.h"

cpu_state_t cpu = {0};

void cpu_init(void) {
    memset(cpu.r, 0, sizeof(cpu.r));
    cpu.xpsr = 0;
    cpu.step_count = 0;
}

int cpu_is_halted(void) {
    /* Check for infinite loop (PC not advancing) or WFI instruction */
    return 0; /* Not implemented yet */
}

void cpu_step(void) {
    if (cpu.r[15] >= FLASH_BASE + FLASH_SIZE) {
        printf("[CPU] ERROR: PC out of bounds (0x%08X)\n", cpu.r[15]);
        return;
    }

    /* FETCH: Read 16-bit Thumb instruction */
    uint16_t instr = mem_read16(cpu.r[15]);
    
    if (instr == 0x0000) {
        printf("[CPU] 0x%08X: Encountered halt (NOP-like)\n", cpu.r[15]);
        return;
    }

    /* DECODE & EXECUTE */
    
    /* MOVS Rd, #imm8 (0010 0Rdii iiiiii) */
    if ((instr & 0xF800) == 0x2000) {
        uint8_t reg = (instr >> 8) & 0x07;
        uint8_t imm = instr & 0xFF;
        cpu.r[reg] = imm;
        printf("[CPU] 0x%08X: MOVS R%d, #0x%02X\n", cpu.r[15], reg, imm);
    }
    /* LDR Rd, [PC, #imm8] (0100 1Rdii iiiiii) */
    else if ((instr & 0xF800) == 0x4800) {
        uint8_t reg = (instr >> 8) & 0x07;
        uint8_t imm = instr & 0xFF;
        uint32_t addr = (cpu.r[15] & ~3) + (imm * 4) + 4;
        cpu.r[reg] = mem_read32(addr);
        printf("[CPU] 0x%08X: LDR R%d, [PC, #0x%02X] = 0x%08X\n", 
               cpu.r[15], reg, imm, cpu.r[reg]);
    }
    /* STR Rd, [Rn, #imm5] (0110 0iiii nnRdRd) */
    else if ((instr & 0xF800) == 0x6000) {
        uint8_t reg_src = (instr >> 0) & 0x07;
        uint8_t reg_dst = (instr >> 3) & 0x07;
        uint8_t imm = (instr >> 6) & 0x1F;
        uint32_t addr = cpu.r[reg_dst] + (imm * 4);
        mem_write32(addr, cpu.r[reg_src]);
        printf("[CPU] 0x%08X: STR R%d, [R%d, #0x%02X]\n", 
               cpu.r[15], reg_src, reg_dst, imm);
    }
    /* ADD Rd, Rn (0100 01dd nnnnddd) */
    else if ((instr & 0xFFC0) == 0x4400) {
        uint8_t reg_src = (instr >> 3) & 0x0F;
        uint8_t reg_dst = ((instr >> 4) & 0x08) | (instr & 0x07);
        cpu.r[reg_dst] += cpu.r[reg_src];
        printf("[CPU] 0x%08X: ADD R%d, R%d\n", cpu.r[15], reg_dst, reg_src);
    }
    else {
        printf("[CPU] 0x%08X: UNIMPLEMENTED 0x%04X\n", cpu.r[15], instr);
    }

    /* UPDATE PC */
    cpu.r[15] += 2;
    cpu.step_count++;
}
