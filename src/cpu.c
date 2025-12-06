#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "emulator.h"
#include "instructions.h"
#include "timer.h"
#include "nvic.h"

cpu_state_t cpu = {0};
int pc_updated = 0;

void cpu_init(void) {
    memset(cpu.r, 0, sizeof(cpu.r));
    cpu.xpsr = 0;
    cpu.step_count = 0;
    cpu.debug_enabled = 0; /* Default: debug off */
    cpu.current_irq = 0xFFFFFFFF; /* Initialize as "no IRQ active" */
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

/* CPU exception entry - saves processor context and jumps to ISR */
void cpu_exception_entry(uint32_t vector_num) {
    /* Vector number 16+ maps to IRQ 0+ */
    uint32_t vector_offset = vector_num * 4;
    uint32_t vector_table_base = 0x10000000; /* Start of flash */
    uint32_t handler_addr = mem_read32(vector_table_base + vector_offset);
    
    if (cpu.debug_enabled) {
        printf("[CPU] Exception %u: PC=0x%08X -> Handler=0x%08X\n", 
               vector_num, cpu.r[15], handler_addr);
    }

    /* Mark as active in NVIC and CPU state */
    nvic_state.active_exceptions |= (1 << vector_num);
    cpu.current_irq = vector_num;

    /* Set IABR bit for external interrupts (Vector >= 16) */
    if (vector_num >= 16) {
        nvic_state.iabr |= (1 << (vector_num - 16));
    }

    /* Save context to stack (Cortex-M0+ automatic stacking frame)
     * Pushes: R0, R1, R2, R3, R12, LR, PC, xPSR
     * Stack frame is 32 bytes
     */
    uint32_t sp = cpu.r[13]; /* Stack pointer */
    sp -= 4; mem_write32(sp, cpu.r[0]);   /* Save R0 */
    sp -= 4; mem_write32(sp, cpu.r[1]);   /* Save R1 */
    sp -= 4; mem_write32(sp, cpu.r[2]);   /* Save R2 */
    sp -= 4; mem_write32(sp, cpu.r[3]);   /* Save R3 */
    sp -= 4; mem_write32(sp, cpu.r[12]);  /* Save R12 */
    sp -= 4; mem_write32(sp, cpu.r[14]);  /* Save LR (return address) */
    sp -= 4; mem_write32(sp, cpu.r[15]);  /* Save PC (instruction address) */
    sp -= 4; mem_write32(sp, cpu.xpsr);   /* Save xPSR (flags) */
    cpu.r[13] = sp; /* Update SP to new position */

    /* Set up for ISR execution:
     * - PC = handler address
     * - LR = special return value (0xFFFFFFF9 for Thumb mode return)
     * - R0-R3 preserved (already on stack)
     */
    cpu.r[15] = handler_addr & ~1; /* Clear Thumb bit if needed, set PC to handler */
    cpu.r[14] = 0xFFFFFFF9;        /* Special return address (return to thread mode) */
}

/**
 * Handle exception return from ISR via BX LR with magic LR values.
 * 
 * Magic LR values in ARM Cortex-M0+:
 *   - 0xFFFFFFF9: Return to Thread mode (normal ISR return)
 *   - 0xFFFFFFF1: Return to Handler mode (nested exception)
 *   - 0xFFFFFFFD: Return with FPU context (Cortex-M4/M7, not M0+)
 * 
 * For Cortex-M0+ (no FPU), typically only 0xFFFFFFF9 is used.
 * 
 * When returning from ISR:
 *   1. Pop the 8-register stacking frame from stack
 *   2. Restore PC, xPSR, R0-R3, R12, LR, SP
 *   3. Clear active exception bit
 *   4. Clear IABR bit for this IRQ
 */
void cpu_exception_return(uint32_t lr_value) {
    uint32_t return_mode = lr_value & 0x0F;
    
    if (cpu.debug_enabled) {
        printf("[CPU] Exception return: LR=0x%08X mode=%u\n", lr_value, return_mode);
    }
    
    /* Only support 0xFFFFFFF9 (return to thread mode) for Cortex-M0+ */
    if (return_mode == 0x9) {
        /* Pop the 8-register context frame from stack
         * Stack layout (from top to bottom, as pushed):
         *   [SP-0]  xPSR
         *   [SP-4]  PC
         *   [SP-8]  LR
         *   [SP-12] R12
         *   [SP-16] R3
         *   [SP-20] R2
         *   [SP-24] R1
         *   [SP-28] R0
         */
        uint32_t sp = cpu.r[13];
        
        /* Pop in same order as push (LIFO) */
        uint32_t xpsr = mem_read32(sp);      sp += 4;
        uint32_t pc   = mem_read32(sp);      sp += 4;
        uint32_t lr   = mem_read32(sp);      sp += 4;
        uint32_t r12  = mem_read32(sp);      sp += 4;
        uint32_t r3   = mem_read32(sp);      sp += 4;
        uint32_t r2   = mem_read32(sp);      sp += 4;
        uint32_t r1   = mem_read32(sp);      sp += 4;
        uint32_t r0   = mem_read32(sp);      sp += 4;
        
        /* Restore registers */
        cpu.r[0]  = r0;
        cpu.r[1]  = r1;
        cpu.r[2]  = r2;
        cpu.r[3]  = r3;
        cpu.r[12] = r12;
        cpu.r[13] = sp;           /* Updated SP */
        cpu.r[14] = lr;           /* Restore LR */
        cpu.r[15] = pc & ~1;      /* Restore PC, clear Thumb bit */
        cpu.xpsr  = xpsr;         /* Restore condition flags */
        
        if (cpu.debug_enabled) {
            printf("[CPU] Frame restored: PC=0x%08X SP=0x%08X XPSR=0x%08X\n",
                   cpu.r[15], cpu.r[13], cpu.xpsr);
        }
        
        /* Clear active exception tracking */
        if (cpu.current_irq != 0xFFFFFFFF) {
            uint32_t vector_num = cpu.current_irq;
            
            /* Clear IABR bit for external interrupts (Vector >= 16) */
            if (vector_num >= 16) {
                nvic_state.iabr &= ~(1 << (vector_num - 16));
            }
            
            /* Clear active_exceptions bit */
            nvic_state.active_exceptions &= ~(1 << vector_num);
            
            if (cpu.debug_enabled) {
                printf("[CPU] Cleared active exception (vector %u), IABR=0x%X\n",
                       vector_num, nvic_state.iabr);
            }
            
            cpu.current_irq = 0xFFFFFFFF;  /* Reset current IRQ tracker */
        }
    }
}

void cpu_step(void) {
    uint32_t pc = cpu.r[15];
    
    /* Stop if PC already out of range */
    if (pc < FLASH_BASE || pc >= FLASH_BASE + FLASH_SIZE) {
        printf("[CPU] ERROR: PC out of bounds (0x%08X)\n", pc);
        cpu.r[15] = 0xFFFFFFFF; /* Mark halted */
        return;
    }

    uint16_t instr = mem_read16(pc);
    cpu.step_count++;

    /* Only print debug output if debug mode is enabled */
    if (cpu.debug_enabled) {
        printf("[CPU] Step %3u: PC=0x%08X instr=0x%04X\n", cpu.step_count, pc, instr);
    }

    timer_tick(1);

    /* Check for pending interrupts BEFORE executing instruction */
    uint32_t pending_irq = nvic_get_pending_irq();
    if (pending_irq != 0xFFFFFFFF) {
        /* Convert IRQ number to vector number (IRQ 0 = vector 16) */
        uint32_t vector_num = pending_irq + 16;
        
        if (cpu.debug_enabled) {
            printf("[CPU] *** INTERRUPT: IRQ %u detected ***\n", pending_irq);
        }

        /* Clear the pending bit */
        nvic_clear_pending(pending_irq);
        
        /* Enter the exception handler (this sets Active bits) */
        cpu_exception_entry(vector_num);
        
        /* Update timer and return without executing regular instruction */
        timer_tick(1);
        return;
    }

    /* Treat all-zero halfword as NOP */
    if (instr == 0x0000) {
        cpu.r[15] = pc + 2;
        timer_tick(1);
        return;
    }

    // ========================================================================
    // FOUNDATIONAL INSTRUCTIONS
    // ========================================================================

    /* -------- 32-bit instructions (must check first) -------- */
    if ((instr & 0xF800) == 0xF000) { /* BL (32-bit) */
        instr_bl(instr);
        timer_tick(1);
        return;
    }

    /* -------- Control-flow instructions (handle PC themselves) -------- */
    if ((instr & 0xFF00) == 0xBE00) { /* BKPT */
        instr_bkpt(instr);
        timer_tick(1);
        return;
    } else if ((instr & 0xFF87) == 0x4700) { /* BX Rm */
        instr_bx(instr);
        timer_tick(1);
        return;
    } else if ((instr & 0xFF87) == 0x4780) { /* BLX Rm */
        instr_blx(instr);
        timer_tick(1);
        return;
    } else if ((instr & 0xF800) == 0xE000) { /* B (unconditional) */
        instr_b_uncond(instr);
        timer_tick(1);
        return;
    } else if ((instr & 0xF000) == 0xD000) { /* B{cond} */
        instr_bcond(instr);
        timer_tick(1);
        return;
    } else if ((instr & 0xFE00) == 0xBC00) { /* POP (includes R15) */
        instr_pop(instr);
        /* If P bit (bit 8) set, POP loaded PC - don't increment */
        if ((instr & 0x0100) != 0) { 
            timer_tick(1);
            return; /* PC was loaded, don't increment */
        }
    } else if (instr == 0xE7FD) { /* UDF */
        instr_udf(instr);
        timer_tick(1);
        return;
    }

    /* -------- Data Movement -------- */
    if ((instr & 0xF800) == 0x2000) { /* MOVS Rd, #imm8 */
        instr_movs_imm8(instr);
    } else if ((instr & 0xFF00) == 0x4600) { /* MOV Rd, Rm (high regs) */
        instr_mov_reg(instr);
    }

    /* -------- Arithmetic - Immediate -------- */
    else if ((instr & 0xFE00) == 0x1C00) { /* ADDS Rd, Rn, #imm3 */
        instr_adds_imm3(instr);
    } else if ((instr & 0xF800) == 0x3000) { /* ADDS Rd, #imm8 */
        instr_adds_imm8(instr);
    } else if ((instr & 0xFE00) == 0x1E00) { /* SUBS Rd, Rn, #imm3 */
        instr_subs_imm3(instr);
    } else if ((instr & 0xF800) == 0x3800) { /* SUBS Rd, #imm8 */
        instr_subs_imm8(instr);
    }

    /* -------- Arithmetic - Register -------- */
    else if ((instr & 0xFE00) == 0x1800) { /* ADDS Rd, Rn, Rm */
        instr_add_reg_reg(instr);
    } else if ((instr & 0xFF00) == 0x4400) { /* ADD Rd, Rm (high regs) */
        instr_add_reg_reg(instr);
    } else if ((instr & 0xFE00) == 0x1A00) { /* SUBS Rd, Rn, Rm */
        instr_sub_reg_reg(instr);
    }

    /* -------- Comparison -------- */
    else if ((instr & 0xF800) == 0x2800) { /* CMP Rn, #imm8 */
        instr_cmp_imm8(instr);
    } else if ((instr & 0xFF00) == 0x4500) { /* CMP Rn, Rm (high regs) */
        instr_cmp_reg_reg(instr);
    } else if ((instr & 0xFFC0) == 0x4280) { /* CMP Rn, Rm (low regs) */
        instr_cmp_reg_reg(instr);
    }

    /* -------- Load/Store - Word -------- */
    else if ((instr & 0xF800) == 0x6800) { /* LDR Rd, [Rn, #imm5] */
        instr_ldr_imm5(instr);
    } else if ((instr & 0xF800) == 0x5800) { /* LDR Rd, [Rn, Rm] */
        instr_ldr_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x4800) { /* LDR Rd, [PC, #imm8] */
        instr_ldr_pc_imm8(instr);
    } else if ((instr & 0xF800) == 0x9800) { /* LDR Rd, [SP, #imm8] */
        instr_ldr_sp_imm8(instr);
    } else if ((instr & 0xF800) == 0x6000) { /* STR Rd, [Rn, #imm5] */
        instr_str_imm5(instr);
    } else if ((instr & 0xF800) == 0x5000) { /* STR Rd, [Rn, Rm] */
        instr_str_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x9000) { /* STR Rd, [SP, #imm8] */
        instr_str_sp_imm8(instr);
    }

    /* -------- Stack Operations -------- */
    else if ((instr & 0xFE00) == 0xB400) { /* PUSH {reglist, lr} */
        instr_push(instr);
    }
    /* POP is handled above in control-flow section */
    
    // ========================================================================
    // ESSENTIAL INSTRUCTIONS
    // ========================================================================

    /* -------- Load/Store - Byte -------- */
    else if ((instr & 0xF800) == 0x7800) { /* LDRB Rd, [Rn, #imm5] */
        instr_ldrb_imm5(instr);
    } else if ((instr & 0xFE00) == 0x5C00) { /* LDRB Rd, [Rn, Rm] */
        instr_ldrb_reg_offset(instr);
    } else if ((instr & 0xFE00) == 0x5600) { /* LDRSB Rd, [Rn, Rm] */
        instr_ldrsb_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x7000) { /* STRB Rd, [Rn, #imm5] */
        instr_strb_imm5(instr);
    } else if ((instr & 0xFE00) == 0x5400) { /* STRB Rd, [Rn, Rm] */
        instr_strb_reg_offset(instr);
    }

    /* -------- Load/Store - Halfword -------- */
    else if ((instr & 0xF800) == 0x8800) { /* LDRH Rd, [Rn, #imm5] */
        instr_ldrh_imm5(instr);
    } else if ((instr & 0xFE00) == 0x5A00) { /* LDRH Rd, [Rn, Rm] */
        instr_ldrh_reg_offset(instr);
    } else if ((instr & 0xFE00) == 0x5E00) { /* LDRSH Rd, [Rn, Rm] */
        instr_ldrsh_reg_offset(instr);
    } else if ((instr & 0xF800) == 0x8000) { /* STRH Rd, [Rn, #imm5] */
        instr_strh_imm5(instr);
    } else if ((instr & 0xFE00) == 0x5200) { /* STRH Rd, [Rn, Rm] */
        instr_strh_reg_offset(instr);
    }

    /* -------- Shift Operations - Immediate -------- */
    else if ((instr & 0xF800) == 0x0000) { /* LSLS Rd, Rm, #imm5 */
        if (instr != 0x0000) { /* Not NOP */
            instr_shift_logical_left(instr);
        }
    } else if ((instr & 0xF800) == 0x0800) { /* LSRS Rd, Rm, #imm5 */
        instr_shift_logical_right(instr);
    } else if ((instr & 0xF800) == 0x1000) { /* ASRS Rd, Rm, #imm5 */
        instr_shift_arithmetic_right(instr);
    }

    /* -------- Shift Operations - Register -------- */
    else if ((instr & 0xFFC0) == 0x4080) { /* LSLS Rd, Rs */
        instr_lsls_reg(instr);
    } else if ((instr & 0xFFC0) == 0x40C0) { /* LSRS Rd, Rs */
        instr_lsrs_reg(instr);
    } else if ((instr & 0xFFC0) == 0x4100) { /* ASRS Rd, Rs */
        instr_asrs_reg(instr);
    } else if ((instr & 0xFFC0) == 0x41C0) { /* RORS Rd, Rs */
        instr_rors_reg(instr);
    }

    /* -------- Logical Operations -------- */
    else if ((instr & 0xFFC0) == 0x4000) { /* ANDS Rd, Rm */
        instr_bitwise_and(instr);
    } else if ((instr & 0xFFC0) == 0x4040) { /* EORS Rd, Rm */
        instr_bitwise_eor(instr);
    } else if ((instr & 0xFFC0) == 0x4300) { /* ORRS Rd, Rm */
        instr_bitwise_orr(instr);
    } else if ((instr & 0xFFC0) == 0x4380) { /* BICS Rd, Rm */
        instr_bitwise_bic(instr);
    } else if ((instr & 0xFFC0) == 0x43C0) { /* MVNS Rd, Rm */
        instr_bitwise_mvn(instr);
    }

    /* -------- Multiplication -------- */
    else if ((instr & 0xFFC0) == 0x4340) { /* MULS Rd, Rm */
        instr_muls(instr);
    }

    /* -------- Multiple Load/Store -------- */
    else if ((instr & 0xF800) == 0xC000) { /* STMIA Rn!, {reglist} */
        instr_stmia(instr);
    } else if ((instr & 0xF800) == 0xC800) { /* LDMIA Rn!, {reglist} */
        instr_ldmia(instr);
    }

    // ========================================================================
    // IMPORTANT INSTRUCTIONS
    // ========================================================================

    /* -------- Special Comparison -------- */
    else if ((instr & 0xFFC0) == 0x42C0) { /* CMN Rn, Rm */
        instr_cmn_reg(instr);
    } else if ((instr & 0xFFC0) == 0x4200) { /* TST Rn, Rm */
        instr_tst_reg_reg(instr);
    }

    /* -------- System Operations -------- */
    else if ((instr & 0xFF00) == 0xDF00) { /* SVC #imm8 */
        instr_svc(instr);
    }

    // ========================================================================
    // OPTIONAL INSTRUCTIONS
    // ========================================================================

    /* -------- Hints and Special -------- */
    else if (instr == 0xBF00) { /* NOP */
        instr_nop(instr);
    } else if ((instr & 0xFF00) == 0xBF00) { /* IT and hints */
        uint8_t firstcond = (instr >> 4) & 0xF;
        if (firstcond == 0x0) {
            instr_nop(instr); /* NOP */
        } else if (firstcond == 0x1) {
            instr_yield(instr); /* YIELD */
        } else if (firstcond == 0x2) {
            instr_wfe(instr); /* WFE */
        } else if (firstcond == 0x3) {
            instr_wfi(instr); /* WFI */
        } else if (firstcond == 0x4) {
            instr_sev(instr); /* SEV */
        } else {
            instr_it(instr); /* IT */
        }
    }

    /* -------- Sign/Zero Extend -------- */
    else if ((instr & 0xFFC0) == 0xB240) { /* SXTB Rd, Rm */
        instr_sxtb(instr);
    } else if ((instr & 0xFFC0) == 0xB200) { /* SXTH Rd, Rm */
        instr_sxth(instr);
    } else if ((instr & 0xFFC0) == 0xB2C0) { /* UXTB Rd, Rm */
        instr_uxtb(instr);
    } else if ((instr & 0xFFC0) == 0xB280) { /* UXTH Rd, Rm */
        instr_uxth(instr);
    }

    /* -------- Byte Reverse -------- */
    else if ((instr & 0xFFC0) == 0xBA00) { /* REV Rd, Rm */
        instr_rev(instr);
    } else if ((instr & 0xFFC0) == 0xBA40) { /* REV16 Rd, Rm */
        instr_rev16(instr);
    } else if ((instr & 0xFFC0) == 0xBAC0) { /* REVSH Rd, Rm */
        instr_revsh(instr);
    }

    /* -------- Stack Pointer Adjustment -------- */
    else if ((instr & 0xFF80) == 0xB000) { /* ADD SP, #imm7 */
        instr_add_sp_imm7(instr);
    } else if ((instr & 0xFF80) == 0xB080) { /* SUB SP, #imm7 */
        instr_sub_sp_imm7(instr);
    }

    /* -------- Address Generation -------- */
    else if ((instr & 0xF800) == 0xA000) { /* ADR Rd, label */
        instr_adr(instr);
    } else if ((instr & 0xF800) == 0xA800) { /* ADD Rd, SP, #imm8 */
        /* Use existing logic or create wrapper */
        uint8_t rd = (instr >> 8) & 0x07;
        uint8_t imm8 = instr & 0xFF;
        cpu.r[rd] = cpu.r[13] + (imm8 << 2);
    }

    /* -------- Interrupt Control -------- */
    else if ((instr & 0xFFEF) == 0xB662) { /* CPSID i */
        instr_cpsid(instr);
    } else if ((instr & 0xFFEF) == 0xB660) { /* CPSIE i */
        instr_cpsie(instr);
    }

    // ========================================================================
    // UNKNOWN/UNIMPLEMENTED
    // ========================================================================
    else {
        /* Unknown / unimplemented instruction: log and halt */
        instr_unimplemented(instr);
        timer_tick(1);
        return;
    }

    /* Advance to next 16-bit instruction (if not already handled) */
    cpu.r[15] = pc + 2;
}
