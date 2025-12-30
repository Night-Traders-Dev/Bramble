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
    cpu.xpsr = 0x01000000;  // Thumb bit
    cpu.step_count = 0;
    cpu.debug_enabled = 0; /* Default: debug off */
    cpu.current_irq = 0xFFFFFFFF; /* Initialize as "no IRQ active" */
    cpu.vtor = 0x10000100; /* RP2040 vector table after 256-byte boot2 */
}

/**
 * Reset CPU from flash after boot2 has executed.
 * 
 * On RP2040, the boot sequence is:
 *   1. Bootrom validates and executes boot2 (first 256 bytes at 0x10000000)
 *   2. Boot2 configures XIP flash and sets VTOR to 0x10000100
 *   3. Boot2 loads SP from [0x10000100] and PC from [0x10000104]
 *   4. Boot2 jumps to user application reset handler
 * 
 * This function simulates steps 2-4, skipping boot2 execution.
 */
void cpu_reset_from_flash(void) {
    /* RP2040 vector table is at offset 0x100 (after 256-byte boot2) */
    cpu.vtor = FLASH_BASE + 0x100;
    
    /* Load initial stack pointer from vector table offset 0 */
    uint32_t initial_sp = mem_read32(cpu.vtor + 0x00);
    
    /* Load reset vector from vector table offset 4 */
    uint32_t reset_vector = mem_read32(cpu.vtor + 0x04);
    
    /* Validate stack pointer is in valid RAM range */
    if (initial_sp < RAM_BASE || initial_sp > RAM_BASE + RAM_SIZE) {
        printf("[Boot] ERROR: Invalid SP 0x%08X (not in RAM 0x%08X-0x%08X)\n",
               initial_sp, RAM_BASE, RAM_BASE + RAM_SIZE);
        cpu.r[15] = 0xFFFFFFFF; /* Mark as halted */
        return;
    }
    
    /* Validate reset vector has Thumb bit set (LSB=1) */
    if ((reset_vector & 0x1) == 0) {
        printf("[Boot] ERROR: Invalid reset vector 0x%08X (Thumb bit not set)\n", 
               reset_vector);
        cpu.r[15] = 0xFFFFFFFF; /* Mark as halted */
        return;
    }
    
    /* Initialize CPU state */
    cpu.r[13] = initial_sp;              /* Set stack pointer (R13/SP) */
    cpu.r[15] = reset_vector & ~1u;      /* Set PC, clear Thumb bit */
    cpu.r[14] = 0xFFFFFFFF;              /* Set LR to invalid address */
    cpu.xpsr  = 0x01000000;              /* Set Thumb bit in xPSR */
    
    printf("[Boot] Reset complete:\n");
    printf("[Boot]   VTOR = 0x%08X\n", cpu.vtor);
    printf("[Boot]   SP   = 0x%08X\n", cpu.r[13]);
    printf("[Boot]   PC   = 0x%08X\n", cpu.r[15]);
    printf("[Boot]   xPSR = 0x%08X\n", cpu.xpsr);
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
    
    /* Use VTOR (Vector Table Offset Register) to find vector table base */
    uint32_t handler_addr = mem_read32(cpu.vtor + vector_offset);

    if (cpu.debug_enabled) {
        printf("[CPU] Exception %u: PC=0x%08X VTOR=0x%08X -> Handler=0x%08X\n", 
               vector_num, cpu.r[15], cpu.vtor, handler_addr);
    }

    /* Mark as active in NVIC and CPU state */
    if (vector_num < 32) {
        nvic_state.active_exceptions |= (1u << vector_num);
    }
    cpu.current_irq = vector_num;

    /* Set IABR bit for external interrupts (Vector >= 16) */
    if (vector_num >= 16 && (vector_num - 16) < 32) {
        nvic_state.iabr |= (1u << (vector_num - 16));
    }

    /* Save context to stack (Cortex-M0+ automatic stacking frame)
     * Hardware stacks (in this order, growing downward):
     *   xPSR, PC, LR, R12, R3, R2, R1, R0
     * Final SP points to R0 (lowest address of frame)
     */
    uint32_t sp = cpu.r[13]; /* Stack pointer */
    sp -= 4; mem_write32(sp, cpu.xpsr);   /* Save xPSR */
    sp -= 4; mem_write32(sp, cpu.r[15]);  /* Save PC (return address) */
    sp -= 4; mem_write32(sp, cpu.r[14]);  /* Save LR */
    sp -= 4; mem_write32(sp, cpu.r[12]);  /* Save R12 */
    sp -= 4; mem_write32(sp, cpu.r[3]);   /* Save R3 */
    sp -= 4; mem_write32(sp, cpu.r[2]);   /* Save R2 */
    sp -= 4; mem_write32(sp, cpu.r[1]);   /* Save R1 */
    sp -= 4; mem_write32(sp, cpu.r[0]);   /* Save R0 */
    cpu.r[13] = sp; /* Update SP to new position (pointing to R0) */

    if (cpu.debug_enabled) {
        printf("[CPU] Context saved, SP now=0x%08X\n", sp);
    }

    /* Set up for ISR execution:
     * - PC = handler address
     * - LR = special return value (0xFFFFFFF9 for return to thread mode)
     * - Thumb bit in handler_addr is ignored (cleared)
     */
    cpu.r[15] = handler_addr & ~1u; /* Set PC to handler, clear Thumb bit */
    cpu.r[14] = 0xFFFFFFF9;         /* EXC_RETURN: return to thread mode with MSP */
}

/**
 * Handle exception return from ISR via BX LR with magic LR values.
 * 
 * Magic EXC_RETURN values in ARM Cortex-M0+:
 *   - 0xFFFFFFF1: Return to Handler mode (nested exception)
 *   - 0xFFFFFFF9: Return to Thread mode using MSP
 *   - 0xFFFFFFFD: Return to Thread mode using PSP
 * 
 * For Cortex-M0+, typically only 0xFFFFFFF9 is used (no PSP in simple cases).
 * 
 * When returning from ISR:
 *   1. Pop the 8-register stacking frame from stack
 *   2. Restore R0-R3, R12, LR, PC, xPSR
 *   3. Clear active exception bit
 *   4. Clear IABR bit for this IRQ
 */
void cpu_exception_return(uint32_t lr_value) {
    uint32_t return_mode = lr_value & 0x0F;

    if (cpu.debug_enabled) {
        printf("[CPU] >>> EXCEPTION RETURN START: LR=0x%08X mode=0x%X SP=0x%08X\n", 
               lr_value, return_mode, cpu.r[13]);
    }

    /* Support 0xFFFFFFF9 (return to thread mode using MSP) */
    if (return_mode == 0x9 || return_mode == 0x1) {
        /* Pop the 8-register context frame from stack
         * Stack layout (SP points to R0):
         *   [SP+0]  R0
         *   [SP+4]  R1
         *   [SP+8]  R2
         *   [SP+12] R3
         *   [SP+16] R12
         *   [SP+20] LR
         *   [SP+24] PC
         *   [SP+28] xPSR
         */
        uint32_t sp = cpu.r[13];

        if (cpu.debug_enabled) {
            printf("[CPU]   Popping frame from SP=0x%08X\n", sp);
        }

        /* Pop in reverse order of push */
        uint32_t r0   = mem_read32(sp);      sp += 4;
        uint32_t r1   = mem_read32(sp);      sp += 4;
        uint32_t r2   = mem_read32(sp);      sp += 4;
        uint32_t r3   = mem_read32(sp);      sp += 4;
        uint32_t r12  = mem_read32(sp);      sp += 4;
        uint32_t lr   = mem_read32(sp);      sp += 4;
        uint32_t pc   = mem_read32(sp);      sp += 4;
        uint32_t xpsr = mem_read32(sp);      sp += 4;

        if (cpu.debug_enabled) {
            printf("[CPU]   Popped: R0=0x%08X R1=0x%08X R2=0x%08X R3=0x%08X\n",
                   r0, r1, r2, r3);
            printf("[CPU]   Popped: R12=0x%08X LR=0x%08X PC=0x%08X xPSR=0x%08X\n",
                   r12, lr, pc, xpsr);
        }

        /* Restore registers */
        cpu.r[0]  = r0;
        cpu.r[1]  = r1;
        cpu.r[2]  = r2;
        cpu.r[3]  = r3;
        cpu.r[12] = r12;
        cpu.r[13] = sp;           /* Updated SP */
        cpu.r[14] = lr;           /* Restore LR */
        cpu.r[15] = pc & ~1u;     /* Restore PC, clear Thumb bit */
        cpu.xpsr  = xpsr;         /* Restore condition flags */

        if (cpu.debug_enabled) {
            printf("[CPU]   RESTORED: PC now=0x%08X SP now=0x%08X\n",
                   cpu.r[15], cpu.r[13]);
        }

        /* Clear active exception tracking */
        if (cpu.current_irq != 0xFFFFFFFF) {
            uint32_t vector_num = cpu.current_irq;

            /* Clear IABR bit for external interrupts (Vector >= 16) */
            if (vector_num >= 16 && (vector_num - 16) < 32) {
                nvic_state.iabr &= ~(1u << (vector_num - 16));
            }

            /* Clear active_exceptions bit */
            if (vector_num < 32) {
                nvic_state.active_exceptions &= ~(1u << vector_num);
            }

            if (cpu.debug_enabled) {
                printf("[CPU]   Cleared active exception (vector %u), IABR=0x%X\n",
                       vector_num, nvic_state.iabr);
            }

            cpu.current_irq = 0xFFFFFFFF;  /* Reset current IRQ tracker */
        }

        if (cpu.debug_enabled) {
            printf("[CPU] <<< EXCEPTION RETURN COMPLETE\n");
        }
    } else {
        printf("[CPU] ERROR: Unsupported EXC_RETURN mode 0x%X (expected 0x9 or 0x1)\n", 
               return_mode);
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
