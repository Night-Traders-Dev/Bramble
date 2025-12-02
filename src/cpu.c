#include <stdio.h>
#include "emulator.h"

// Define the global CPU state here
cpu_state_t cpu;

void cpu_step() {
    // 1. FETCH
    // ARM Cortex-M0+ instructions are mostly 16-bit (Thumb mode)
    // In a real emulator, handle alignment/endianness carefully.
    uint16_t instr = (uint16_t)mem_read32(cpu.r[15]); 
    
    // 2. DECODE & EXECUTE (Stub)
    
    // Example: MOVS Rd, #imm8 (0010 0Rdii iiiiii) -> Opcode 0x2000
    if ((instr & 0xF800) == 0x2000) { 
        uint8_t reg = (instr >> 8) & 0x07;
        uint8_t imm = instr & 0xFF;
        cpu.r[reg] = imm;
        printf("[CPU] 0x%08X: MOVS R%d, #%d
", cpu.r[15], reg, imm);
    }
    // Example: LDR Rd, [PC, #imm8] (0100 1Rdii iiiiii) -> Opcode 0x4800
    else if ((instr & 0xF800) == 0x4800) {
        uint8_t reg = (instr >> 8) & 0x07;
        uint8_t imm = instr & 0xFF;
        // PC is 4-byte aligned + offset * 4
        uint32_t addr = (cpu.r[15] & ~3) + (imm * 4) + 4;
        cpu.r[reg] = mem_read32(addr);
        printf("[CPU] 0x%08X: LDR R%d, [PC, #%d] (Addr: 0x%08X)
", cpu.r[15], reg, imm, addr);
    }
    else {
        // printf("[CPU] 0x%08X: Unknown Instruction 0x%04X
", cpu.r[15], instr);
    }

    // 3. UPDATE PC
    // Advance by 2 bytes (standard Thumb instruction size)
    cpu.r[15] += 2; 
}
