#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "emulator.h"
#include "instructions.h"


cpu_state_t cpu = {0};
int pc_updated = 0;

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
    if (cpu.step_count < 200) {
        printf("[CPU] Step %2u: PC=0x%08X instr=0x%04X\n", cpu.step_count, pc, instr);
    }

    /* Treat all-zero halfword as NOP */
    if (instr == 0x0000) {
        cpu.r[15] = pc + 2;
        return;
    }

    // ========================================================================
    // FOUNDATIONAL INSTRUCTIONS
    // ========================================================================

    /* -------- 32-bit instructions (must check first) -------- */
    
    if ((instr & 0xF800) == 0xF000) {   /* BL (32-bit) */
        instr_bl(instr);
        return;
    }
    
    /* -------- Control-flow instructions (handle PC themselves) -------- */
    
    if ((instr & 0xFF00) == 0xBE00) {   /* BKPT */
        instr_bkpt(instr);
        return;
    } else if ((instr & 0xFF87) == 0x4700) {   /* BX Rm */
        instr_bx(instr);
        return;
    } else if ((instr & 0xFF87) == 0x4780) {   /* BLX Rm */
        instr_blx(instr);
        return;
    } else if ((instr & 0xF800) == 0xE000) {   /* B (unconditional) */
        instr_b_uncond(instr);
        return;
    } else if ((instr & 0xF000) == 0xD000) {   /* B{cond} */
        instr_bcond(instr);
        return;
    } else if ((instr & 0xFE00) == 0xBC00) {   /* POP (includes R15) */
        instr_pop(instr);
        /* If P bit (bit 8) set, POP loaded PC - don't increment */
        if ((instr & 0x0100) != 0) {
            return;  /* PC was loaded, don't increment */
        }
        cpu.r[15] = pc + 2;
        return;
    } else if (instr == 0xE7FD) {              /* UDF */
        instr_udf(instr);
        return;
    }
    
    /* -------- Data Movement -------- */
    
    if ((instr & 0xF800) == 0x2000) {          /* MOVS Rd, #imm8 */
        instr_movs_imm8(instr);
    } else if ((instr & 0xFF00) == 0x4600) {   /* MOV Rd, Rm (high regs) */
        instr_mov_reg(instr);
    }
    
    /* -------- Arithmetic - Immediate -------- */
    
    else if ((instr & 0xFE00) == 0x1C00) {     /* ADDS Rd, Rn, #imm3 */
        instr_adds_imm3(instr);
    } else if ((instr & 0xF800) == 0x3000) {   /* ADDS Rd, #imm8 */
        instr_adds_imm8(instr);
    } else if ((instr & 0xFE00) == 0x1E00) {   /* SUBS Rd, Rn, #imm3 */
        instr_subs_imm3(instr);
    } else if ((instr & 0xF800) == 0x3800) {   /* SUBS Rd, #imm8 */
        instr_subs_imm8(instr);
    }
    
    /* -------- Arithmetic - Register -------- */
    
    else if ((instr & 0xFE00) == 0x1800) {     /* ADDS Rd, Rn, Rm */
        instr_add_reg_reg(instr);
    } else if ((instr & 0xFF00) == 0x4400) {   /* ADD Rd, Rm (high regs) */
        instr_add_reg_reg(instr);
    } else if ((instr & 0xFE00) == 0x1A00) {   /* SUBS Rd, Rn, Rm */
        instr_sub_reg_reg(instr);
    }
    
    /* -------- Comparison -------- */
    
    else if ((instr & 0xF800) == 0x2800) {     /* CMP Rn, #imm8 */
        instr_cmp_imm8(instr);
    } else if ((instr & 0xFF00) == 0x4500) {   /* CMP Rn, Rm (high regs) */
        instr_cmp_reg_reg(instr);
    } else if ((instr & 0xFFC0) == 0x4280) {   /* CMP Rn, Rm (low regs) */
        instr_cmp_reg_reg(instr);
    }
    
    /* -------- Load/Store - Word -------- */
    
    else if ((instr & 0xF800) == 0x6800) {     /* LDR Rd, [Rn, #imm5] */
        instr_ldr_imm5(instr);
    } else if ((instr & 0xF800) == 0x5800) {   /* LDR Rd, [Rn, Rm] */
        instr_ldr_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x4800) {   /* LDR Rd, [PC, #imm8] */
        instr_ldr_pc_imm8(instr);
    } else if ((instr & 0xF800) == 0x9800) {   /* LDR Rd, [SP, #imm8] */
        instr_ldr_sp_imm8(instr);
    } else if ((instr & 0xF800) == 0x6000) {   /* STR Rd, [Rn, #imm5] */
        instr_str_imm5(instr);
    } else if ((instr & 0xF800) == 0x5000) {   /* STR Rd, [Rn, Rm] */
        instr_str_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x9000) {   /* STR Rd, [SP, #imm8] */
        instr_str_sp_imm8(instr);
    }
    
    /* -------- Stack Operations -------- */
    
    else if ((instr & 0xFE00) == 0xB400) {     /* PUSH {reglist, lr} */
        instr_push(instr);
    }
    
    /* POP is handled above in control-flow section */
    
    // ========================================================================
    // ESSENTIAL INSTRUCTIONS
    // ========================================================================
    
    /* -------- Load/Store - Byte -------- */
    
    else if ((instr & 0xF800) == 0x7800) {     /* LDRB Rd, [Rn, #imm5] */
        instr_ldrb_imm5(instr);
    } else if ((instr & 0xFE00) == 0x5C00) {   /* LDRB Rd, [Rn, Rm] */
        instr_ldrb_reg_offset(instr);
    } else if ((instr & 0xFE00) == 0x5600) {   /* LDRSB Rd, [Rn, Rm] */
        instr_ldrsb_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x7000) {   /* STRB Rd, [Rn, #imm5] */
        instr_strb_imm5(instr);
    } else if ((instr & 0xFE00) == 0x5400) {   /* STRB Rd, [Rn, Rm] */
        instr_strb_reg_offset(instr);
    }
    
    /* -------- Load/Store - Halfword -------- */
    
    else if ((instr & 0xF800) == 0x8800) {     /* LDRH Rd, [Rn, #imm5] */
        instr_ldrh_imm5(instr);
    } else if ((instr & 0xFE00) == 0x5A00) {   /* LDRH Rd, [Rn, Rm] */
        instr_ldrh_reg_offset(instr);
    } else if ((instr & 0xFE00) == 0x5E00) {   /* LDRSH Rd, [Rn, Rm] */
        instr_ldrsh_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x8000) {   /* STRH Rd, [Rn, #imm5] */
        instr_strh_imm5(instr);
    } else if ((instr & 0xFE00) == 0x5200) {   /* STRH Rd, [Rn, Rm] */
        instr_strh_reg_offset(instr);
    }
    
    /* -------- Shift Operations - Immediate -------- */
    
    else if ((instr & 0xF800) == 0x0000) {     /* LSLS Rd, Rm, #imm5 */
        if (instr != 0x0000) {  /* Not NOP */
            instr_shift_logical_left(instr);
        }
    } else if ((instr & 0xF800) == 0x0800) {   /* LSRS Rd, Rm, #imm5 */
        instr_shift_logical_right(instr);
    } else if ((instr & 0xF800) == 0x1000) {   /* ASRS Rd, Rm, #imm5 */
        instr_shift_arithmetic_right(instr);
    }
    
    /* -------- Shift Operations - Register -------- */
    
    else if ((instr & 0xFFC0) == 0x4080) {     /* LSLS Rd, Rs */
        instr_lsls_reg(instr);
    } else if ((instr & 0xFFC0) == 0x40C0) {   /* LSRS Rd, Rs */
        instr_lsrs_reg(instr);
    } else if ((instr & 0xFFC0) == 0x4100) {   /* ASRS Rd, Rs */
        instr_asrs_reg(instr);
    } else if ((instr & 0xFFC0) == 0x41C0) {   /* RORS Rd, Rs */
        instr_rors_reg(instr);
    }
    
    /* -------- Logical Operations -------- */
    
    else if ((instr & 0xFFC0) == 0x4000) {     /* ANDS Rd, Rm */
        instr_bitwise_and(instr);
    } else if ((instr & 0xFFC0) == 0x4040) {   /* EORS Rd, Rm */
        instr_bitwise_eor(instr);
    } else if ((instr & 0xFFC0) == 0x4300) {   /* ORRS Rd, Rm */
        instr_bitwise_orr(instr);
    } else if ((instr & 0xFFC0) == 0x4380) {   /* BICS Rd, Rm */
        instr_bitwise_bic(instr);
    } else if ((instr & 0xFFC0) == 0x43C0) {   /* MVNS Rd, Rm */
        instr_bitwise_mvn(instr);
    }
    
    /* -------- Multiplication -------- */
    
    else if ((instr & 0xFFC0) == 0x4340) {     /* MULS Rd, Rm */
        instr_muls(instr);
    }
    
    /* -------- Multiple Load/Store -------- */
    
    else if ((instr & 0xF800) == 0xC000) {     /* STMIA Rn!, {reglist} */
        instr_stmia(instr);
    } else if ((instr & 0xF800) == 0xC800) {   /* LDMIA Rn!, {reglist} */
        instr_ldmia(instr);
    }
    
    // ========================================================================
    // IMPORTANT INSTRUCTIONS
    // ========================================================================
    
    /* -------- Special Comparison -------- */
    
    else if ((instr & 0xFFC0) == 0x42C0) {     /* CMN Rn, Rm */
        instr_cmn_reg(instr);
    } else if ((instr & 0xFFC0) == 0x4200) {   /* TST Rn, Rm */
        instr_tst_reg_reg(instr);
    }
    // Note: TEQ is not in standard Thumb, removed from dispatch
    
    /* -------- System Operations -------- */
    
    else if ((instr & 0xFF00) == 0xDF00) {     /* SVC #imm8 */
        instr_svc(instr);
    }
    
    // ========================================================================
    // OPTIONAL INSTRUCTIONS
    // ========================================================================
    
    /* -------- Hints and Special -------- */
    
    else if (instr == 0xBF00) {                /* NOP */
        instr_nop(instr);
    } else if ((instr & 0xFF00) == 0xBF00) {   /* IT and hints */
        uint8_t firstcond = (instr >> 4) & 0xF;
        if (firstcond == 0x0) {
            instr_nop(instr);  /* NOP */
        } else if (firstcond == 0x1) {
            instr_yield(instr);  /* YIELD */
        } else if (firstcond == 0x2) {
            instr_wfe(instr);  /* WFE */
        } else if (firstcond == 0x3) {
            instr_wfi(instr);  /* WFI */
        } else if (firstcond == 0x4) {
            instr_sev(instr);  /* SEV */
        } else {
            instr_it(instr);  /* IT */
        }
    }
    
    /* -------- Sign/Zero Extend -------- */
    
    else if ((instr & 0xFFC0) == 0xB240) {     /* SXTB Rd, Rm */
        instr_sxtb(instr);
    } else if ((instr & 0xFFC0) == 0xB200) {   /* SXTH Rd, Rm */
        instr_sxth(instr);
    } else if ((instr & 0xFFC0) == 0xB2C0) {   /* UXTB Rd, Rm */
        instr_uxtb(instr);
    } else if ((instr & 0xFFC0) == 0xB280) {   /* UXTH Rd, Rm */
        instr_uxth(instr);
    }
    
    /* -------- Byte Reverse -------- */
    
    else if ((instr & 0xFFC0) == 0xBA00) {     /* REV Rd, Rm */
        instr_rev(instr);
    } else if ((instr & 0xFFC0) == 0xBA40) {   /* REV16 Rd, Rm */
        instr_rev16(instr);
    } else if ((instr & 0xFFC0) == 0xBAC0) {   /* REVSH Rd, Rm */
        instr_revsh(instr);
    }
    
    /* -------- Stack Pointer Adjustment -------- */
    
    else if ((instr & 0xFF80) == 0xB000) {     /* ADD SP, #imm7 */
        instr_add_sp_imm7(instr);
    } else if ((instr & 0xFF80) == 0xB080) {   /* SUB SP, #imm7 */
        instr_sub_sp_imm7(instr);
    }
    
    /* -------- Address Generation -------- */
    
    else if ((instr & 0xF800) == 0xA000) {     /* ADR Rd, label */
        instr_adr(instr);
    } else if ((instr & 0xF800) == 0xA800) {   /* ADD Rd, SP, #imm8 */
        // Use existing logic or create wrapper
        uint8_t rd = (instr >> 8) & 0x07;
        uint8_t imm8 = instr & 0xFF;
        cpu.r[rd] = cpu.r[13] + (imm8 << 2);
    }
    
    /* -------- Interrupt Control -------- */
    
    else if ((instr & 0xFFEF) == 0xB662) {     /* CPSID i */
        instr_cpsid(instr);
    } else if ((instr & 0xFFEF) == 0xB660) {   /* CPSIE i */
        instr_cpsie(instr);
    }
    
    // ========================================================================
    // UNKNOWN/UNIMPLEMENTED
    // ========================================================================
    
    else {
        /* Unknown / unimplemented instruction: log and halt */
        instr_unimplemented(instr);
        return;
    }

    /* Advance to next 16-bit instruction (if not already handled) */
    cpu.r[15] = pc + 2;
}
