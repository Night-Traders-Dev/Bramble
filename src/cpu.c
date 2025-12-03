#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "emulator.h"
#include "instructions.h"

cpu_state_t cpu = {0};

void cpu_init(void) {
    memset(cpu.r, 0, sizeof(cpu.r));
    cpu.xpsr = 0;
    cpu.step_count = 0;
}

/* Treat PC out of the flash code region or sentinel as halted */
int cpu_is_halted(void) {
    uint32_t pc = cpu.r[15];

    if (pc == 0xFFFFFFFF) {
        return 1;
    }

    /* For now, execute only from FLASH; later you can allow RAM as well */
    if (pc < FLASH_BASE || pc >= FLASH_BASE + FLASH_SIZE) {
        return 1;
    }

    return 0;
}

void cpu_step(void) {
    uint32_t pc = cpu.r[15];

    /* Stop if PC already out of range */
    if (pc < FLASH_BASE || pc >= FLASH_BASE + FLASH_SIZE) {
        printf("[CPU] ERROR: PC out of bounds (0x%08X)\n", pc);
        cpu.r[15] = 0xFFFFFFFF;  /* Mark halted */
        return;
    }

    uint16_t instr = mem_read16(pc);

    cpu.step_count++;
    if (cpu.step_count < 50) {
        printf("[CPU] Step %2u: PC=0x%08X instr=0x%04X\n", cpu.step_count, pc, instr);
    }

    /* Treat all-zero halfword as NOP */
    if (instr == 0x0000) {
        cpu.r[15] = pc + 2;
        return;
    }

    /* -------- Control-flow instructions (handle PC themselves) -------- */

    if ((instr & 0xFF00) == 0xBE00) {          /* BKPT */
        instr_bkpt(instr);
        return;
    } else if ((instr & 0xF000) == 0xD000) {   /* B{cond} */
        instr_bcond(instr);
        return;
    } else if ((instr & 0xF800) == 0xF000) {   /* BL (32-bit) */
        instr_bl(instr);
        return;
    } else if ((instr & 0xFF87) == 0x4700) {   /* BX */
        instr_bx(instr);
        return;
    } else if (instr == 0xE7FD) {              /* UDF */
        instr_udf(instr);
        return;
    }

    /* -------- Normal 16-bit instructions (PC auto-increments) -------- */

    /* ADD/SUB with immediate (including SP-relative) */
    if ((instr & 0xFF00) == 0xAF00) {          /* ADD SP, SP, imm8 */
        uint8_t imm = (instr & 0x7F) * 4;
        cpu.r[13] += imm;
    } else if ((instr & 0xFF00) == 0xAE00) {   /* SUB SP, SP, imm8 */
        uint8_t imm = (instr & 0x7F) * 4;
        cpu.r[13] -= imm;
    } else if ((instr & 0xF800) == 0x2000) {   /* MOVS imm8 */
        instr_movs_imm8(instr);
    } else if ((instr & 0xFE00) == 0x1C00) {   /* ADDS imm3 */
        instr_adds_imm3(instr);
    } else if ((instr & 0xFE00) == 0x1E00) {   /* SUBS imm3 */
        instr_subs_imm3(instr);
    } else if ((instr & 0xF800) == 0x7800) {   /* LDRB (reg offset) */
        instr_ldrb_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x4800) {   /* LDR (PC-relative, imm8) */
        instr_ldr_pc_imm8(instr);
    } else if ((instr & 0xF800) == 0x6000) {   /* STR (reg offset) */
        instr_str_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x6800) {   /* LDR (reg offset) */
        instr_ldr_reg_offset(instr);
    } else if ((instr & 0xFFC0) == 0x4400) {   /* ADD (register) */
        instr_add_reg_reg(instr);
    } else if ((instr & 0xFE00) == 0x1A00) {   /* SUB (register) */
        instr_sub_reg_reg(instr);
    } else if ((instr & 0xFE00) == 0xB400) {   /* PUSH (includes R14) */
        instr_push(instr);
    } else if ((instr & 0xFE00) == 0xBC00) {   /* POP (includes R15) */
        instr_pop(instr);
        /* If P bit (bit 8) set, POP loaded PC - don't increment */
        if ((instr & 0x0100) == 0) {
            cpu.r[15] = pc + 2;
        }
        return;  /* POP modifies PC directly */
    } else if ((instr & 0xF800) == 0x2800) {   /* CMP imm8 */
        instr_cmp_imm8(instr);
    } else if ((instr & 0xF800) == 0x7800) {   /* LDRB (reg offset) */
        instr_ldrb_reg_offset(instr);
    } else if ((instr & 0xFFC0) == 0x4280) {   /* CMP (register) */
        instr_cmp_reg_reg(instr);
    } else if ((instr & 0xF800) == 0xC000) {   /* STMIA */
        instr_stmia(instr);
    } else if ((instr & 0xF800) == 0xC800) {   /* LDMIA */
        instr_ldmia(instr);
    } else if ((instr & 0xF800) == 0x1800) {   /* LSRS (imm) */
        instr_shift_logical_right(instr);
    } else if ((instr & 0xF800) == 0x0000) {   /* LSLS (imm) */
        instr_shift_logical_left(instr);
    } else if ((instr & 0xF800) == 0x1000) {   /* ASRS (imm) */
        instr_shift_arithmetic_right(instr);
    } else if ((instr & 0xFFC0) == 0x4000) {   /* AND (register) */
        instr_bitwise_and(instr);
    } else if ((instr & 0xFFC0) == 0x4040) {   /* EOR (register) */
        instr_bitwise_eor(instr);
    } else if ((instr & 0xFFC0) == 0x4300) {   /* ORR (register) */
        instr_bitwise_orr(instr);
    } else if ((instr & 0xFFC0) == 0x4380) {   /* BIC (register) */
        instr_bitwise_bic(instr);
    } else if ((instr & 0xFFC0) == 0x43C0) {   /* MVN (register) */
        instr_bitwise_mvn(instr);
    } else {
        /* Unknown / unimplemented instruction: log and halt */
        instr_unimplemented(instr);
        return;
    }

    /* Advance to next 16-bit instruction */
    cpu.r[15] = pc + 2;
}
