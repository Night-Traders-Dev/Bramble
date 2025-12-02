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
        if (cpu.step_count % 50000 == 0) {
            printf("[CPU] NOP at step %u\n", cpu.step_count);
        }
        cpu.r[15] += 2;
        cpu.step_count++;
        return;
    }

    /* Decode and execute instruction */
    if ((instr & 0xF800) == 0x2000) {
        instr_movs_imm8(instr);
    } else if ((instr & 0xF800) == 0x4800) {
        instr_ldr_pc_imm8(instr);
    } else if ((instr & 0xF800) == 0x6000) {
        instr_str_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x6800) {
        instr_ldr_reg_offset(instr);
    } else if ((instr & 0xFFC0) == 0x4400) {
        instr_add_reg_reg(instr);
    } else if ((instr & 0xFE00) == 0x1A00) {
        instr_sub_reg_reg(instr);
    } else if ((instr & 0xFF00) == 0xB400) {
        instr_push(instr);
    } else if ((instr & 0xFF00) == 0xBC00) {
        instr_pop(instr);
    } else if ((instr & 0xFF00) == 0xBE00) {
        instr_bkpt(instr);
    } else if ((instr & 0xFFC0) == 0x4280) {
        instr_cmp_reg_reg(instr);
    } else if ((instr & 0xF800) == 0xC000) {
        instr_stmia(instr);
    } else if ((instr & 0xF800) == 0xC800) {
        instr_ldmia(instr);
    } else if ((instr & 0xF000) == 0xD000) {
        instr_bcond(instr);
    } else if ((instr & 0xF800) == 0xF000) {
        instr_bl(instr);
    } else if ((instr & 0xFF87) == 0x4700) {
        instr_bx(instr);
    } else if ((instr & 0xF800) == 0xE000) {
        instr_unimplemented(instr);
    } else if (instr == 0xE7FD) {
        instr_udf(instr);
    } else if ((instr & 0xF800) == 0x1800) {
        instr_shift_logical_right(instr);
    } else if ((instr & 0xF800) == 0x0000) {
        instr_shift_logical_left(instr);
    } else if ((instr & 0xF800) == 0x1000) {
        instr_shift_arithmetic_right(instr);
    } else if ((instr & 0xFFC0) == 0x4000) {
        instr_bitwise_and(instr);
    } else if ((instr & 0xFFC0) == 0x4040) {
        instr_bitwise_eor(instr);
    } else if ((instr & 0xFFC0) == 0x4300) {
        instr_bitwise_orr(instr);
    } else if ((instr & 0xFFC0) == 0x43C0) {
        instr_bitwise_bic(instr);
    } else if ((instr & 0xFFC0) == 0x43C0) {
        instr_bitwise_mvn(instr);
    } else {
        return;
//        printf("[CPU] 0x%08X: UNIMPLEMENTED 0x%04X\n", cpu.r[15], instr);
    }

    cpu.r[15] += 2;
    cpu.step_count++;
}
