/*
 * RP2040 Emulator CPU Engine (Unified Single & Dual-Core)
 *
 * Consolidated implementation providing:
 * - Single-core CPU execution (cpu_step)
 * - Dual-core CPU execution (cpu_step_core, dual_core_step)
 * - Exception handling for both cores
 * - Unified instruction dispatch
 * - Memory access with core context awareness
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "emulator.h"
#include "instructions.h"
#include "timer.h"
#include "nvic.h"

/* ========================================================================
 * Single-Core Global State
 * ======================================================================== */

cpu_state_t cpu = {0};
int pc_updated = 0;

/* ========================================================================
 * Single-Core Initialization & Reset
 * ======================================================================== */

void cpu_init(void) {
    memset(cpu.r, 0, sizeof(cpu.r));
    cpu.xpsr = 0x01000000; /* Thumb bit */
    cpu.step_count = 0;
    cpu.debug_enabled = 0; /* Default: debug off */
    cpu.current_irq = 0xFFFFFFFF; /* Initialize as "no IRQ active" */
    cpu.vtor = 0x10000100; /* RP2040 vector table after 256-byte boot2 */
}

/**
 * Reset CPU from flash after boot2 has executed.
 *
 * On RP2040, the boot sequence is:
 * 1. Bootrom validates and executes boot2 (first 256 bytes at 0x10000000)
 * 2. Boot2 configures XIP flash and sets VTOR to 0x10000100
 * 3. Boot2 loads SP from [0x10000100] and PC from [0x10000104]
 * 4. Boot2 jumps to user application reset handler
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
    cpu.r[13] = initial_sp; /* Set stack pointer (R13/SP) */
    cpu.r[15] = reset_vector & ~1u; /* Set PC, clear Thumb bit */
    cpu.r[14] = 0xFFFFFFFF; /* Set LR to invalid address */
    cpu.xpsr = 0x01000000; /* Set Thumb bit in xPSR */

    printf("[Boot] Reset complete:\n");
    printf("[Boot] VTOR = 0x%08X\n", cpu.vtor);
    printf("[Boot] SP = 0x%08X\n", cpu.r[13]);
    printf("[Boot] PC = 0x%08X\n", cpu.r[15]);
    printf("[Boot] xPSR = 0x%08X\n", cpu.xpsr);
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

/* ========================================================================
 * Single-Core Exception Handling
 * ======================================================================== */

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
     * xPSR, PC, LR, R12, R3, R2, R1, R0
     * Final SP points to R0 (lowest address of frame)
     */
    uint32_t sp = cpu.r[13]; /* Stack pointer */

    sp -= 4; mem_write32(sp, cpu.xpsr); /* Save xPSR */
    sp -= 4; mem_write32(sp, cpu.r[15]); /* Save PC (return address) */
    sp -= 4; mem_write32(sp, cpu.r[14]); /* Save LR */
    sp -= 4; mem_write32(sp, cpu.r[12]); /* Save R12 */
    sp -= 4; mem_write32(sp, cpu.r[3]); /* Save R3 */
    sp -= 4; mem_write32(sp, cpu.r[2]); /* Save R2 */
    sp -= 4; mem_write32(sp, cpu.r[1]); /* Save R1 */
    sp -= 4; mem_write32(sp, cpu.r[0]); /* Save R0 */

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
    cpu.r[14] = 0xFFFFFFF9; /* EXC_RETURN: return to thread mode with MSP */
}

/**
 * Handle exception return from ISR via BX LR with magic LR values.
 *
 * Magic EXC_RETURN values in ARM Cortex-M0+:
 * - 0xFFFFFFF1: Return to Handler mode (nested exception)
 * - 0xFFFFFFF9: Return to Thread mode using MSP
 * - 0xFFFFFFFD: Return to Thread mode using PSP
 *
 * For Cortex-M0+, typically only 0xFFFFFFF9 is used (no PSP in simple cases).
 *
 * When returning from ISR:
 * 1. Pop the 8-register stacking frame from stack
 * 2. Restore R0-R3, R12, LR, PC, xPSR
 * 3. Clear active exception bit
 * 4. Clear IABR bit for this IRQ
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
         * [SP+0] R0
         * [SP+4] R1
         * [SP+8] R2
         * [SP+12] R3
         * [SP+16] R12
         * [SP+20] LR
         * [SP+24] PC
         * [SP+28] xPSR
         */
        uint32_t sp = cpu.r[13];

        if (cpu.debug_enabled) {
            printf("[CPU] Popping frame from SP=0x%08X\n", sp);
        }

        /* Pop in reverse order of push */
        uint32_t r0 = mem_read32(sp); sp += 4;
        uint32_t r1 = mem_read32(sp); sp += 4;
        uint32_t r2 = mem_read32(sp); sp += 4;
        uint32_t r3 = mem_read32(sp); sp += 4;
        uint32_t r12 = mem_read32(sp); sp += 4;
        uint32_t lr = mem_read32(sp); sp += 4;
        uint32_t pc = mem_read32(sp); sp += 4;
        uint32_t xpsr = mem_read32(sp); sp += 4;

        if (cpu.debug_enabled) {
            printf("[CPU] Popped: R0=0x%08X R1=0x%08X R2=0x%08X R3=0x%08X\n",
                   r0, r1, r2, r3);
            printf("[CPU] Popped: R12=0x%08X LR=0x%08X PC=0x%08X xPSR=0x%08X\n",
                   r12, lr, pc, xpsr);
        }

        /* Restore registers */
        cpu.r[0] = r0;
        cpu.r[1] = r1;
        cpu.r[2] = r2;
        cpu.r[3] = r3;
        cpu.r[12] = r12;
        cpu.r[13] = sp; /* Updated SP */
        cpu.r[14] = lr; /* Restore LR */
        cpu.r[15] = pc & ~1u; /* Restore PC, clear Thumb bit */
        cpu.xpsr = xpsr; /* Restore condition flags */

        if (cpu.debug_enabled) {
            printf("[CPU] RESTORED: PC now=0x%08X SP now=0x%08X\n",
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
                printf("[CPU] Cleared active exception (vector %u), IABR=0x%X\n",
                       vector_num, nvic_state.iabr);
            }

            cpu.current_irq = 0xFFFFFFFF; /* Reset current IRQ tracker */
        }

        if (cpu.debug_enabled) {
            printf("[CPU] <<< EXCEPTION RETURN COMPLETE\n");
        }

    } else {
        printf("[CPU] ERROR: Unsupported EXC_RETURN mode 0x%X (expected 0x9 or 0x1)\n",
               return_mode);
    }
}

/* ========================================================================
 * Single-Core CPU Execution (Main Loop)
 * ======================================================================== */

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
    if ((instr & 0xF800) == 0xF000) { /* FIXED: Fetch both halfwords for 32-bit instruction */
        uint16_t instr2 = mem_read16(pc + 2); /* Check if this is BL or BLX (0xD000 or 0x9000 pattern) */
        if ((instr2 & 0xD000) == 0xD000) { 
            instr_bl_32(instr, instr2); /* Pass BOTH halfwords */ 
            timer_tick(1); 
            return; 
        } else {
            instr_bl(instr);

        }

    /* -------- Control-flow instructions (handle PC themselves) -------- */
    } else if ((instr & 0xFF00) == 0xBE00) { /* BKPT */
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

    /* -------- Data Movement -------- */
    } else if ((instr & 0xF800) == 0x2000) { /* MOVS Rd, #imm8 */
        instr_movs_imm8(instr);

    } else if ((instr & 0xFF00) == 0x4600) { /* MOV Rd, Rm (high regs) */
        instr_mov_reg(instr);

    /* -------- Arithmetic - Immediate -------- */
    } else if ((instr & 0xFE00) == 0x1C00) { /* ADDS Rd, Rn, #imm3 */
        instr_adds_imm3(instr);

    } else if ((instr & 0xF800) == 0x3000) { /* ADDS Rd, #imm8 */
        instr_adds_imm8(instr);

    } else if ((instr & 0xFE00) == 0x1E00) { /* SUBS Rd, Rn, #imm3 */
        instr_subs_imm3(instr);

    } else if ((instr & 0xF800) == 0x3800) { /* SUBS Rd, #imm8 */
        instr_subs_imm8(instr);

    /* -------- Arithmetic - Register -------- */
    } else if ((instr & 0xFE00) == 0x1800) { /* ADDS Rd, Rn, Rm */
        instr_add_reg_reg(instr);

    } else if ((instr & 0xFF00) == 0x4400) { /* ADD Rd, Rm (high regs) */
        instr_add_reg_reg(instr);

    } else if ((instr & 0xFE00) == 0x1A00) { /* SUBS Rd, Rn, Rm */
        instr_sub_reg_reg(instr);

    /* -------- Comparison -------- */
    } else if ((instr & 0xF800) == 0x2800) { /* CMP Rn, #imm8 */
        instr_cmp_imm8(instr);

    } else if ((instr & 0xFF00) == 0x4500) { /* CMP Rn, Rm (high regs) */
        instr_cmp_reg_reg(instr);

    } else if ((instr & 0xFFC0) == 0x4280) { /* CMP Rn, Rm (low regs) */
        instr_cmp_reg_reg(instr);

    /* -------- Load/Store - Word -------- */
    } else if ((instr & 0xF800) == 0x6800) { /* LDR Rd, [Rn, #imm5] */
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

    /* -------- Stack Operations -------- */
    } else if ((instr & 0xFE00) == 0xB400) { /* PUSH {reglist, lr} */
        instr_push(instr);

    /* POP is handled above in control-flow section */

    // ========================================================================
    // ESSENTIAL INSTRUCTIONS
    // ========================================================================

    /* -------- Load/Store - Byte -------- */
    } else if ((instr & 0xF800) == 0x7800) { /* LDRB Rd, [Rn, #imm5] */
        instr_ldrb_imm5(instr);

    } else if ((instr & 0xFE00) == 0x5C00) { /* LDRB Rd, [Rn, Rm] */
        instr_ldrb_reg_offset(instr);

    } else if ((instr & 0xFE00) == 0x5600) { /* LDRSB Rd, [Rn, Rm] */
        instr_ldrsb_reg_offset(instr);

    } else if ((instr & 0xF800) == 0x7000) { /* STRB Rd, [Rn, #imm5] */
        instr_strb_imm5(instr);

    } else if ((instr & 0xFE00) == 0x5400) { /* STRB Rd, [Rn, Rm] */
        instr_strb_reg_offset(instr);

    /* -------- Load/Store - Halfword -------- */
    } else if ((instr & 0xF800) == 0x8800) { /* LDRH Rd, [Rn, #imm5] */
        instr_ldrh_imm5(instr);

    } else if ((instr & 0xFE00) == 0x5A00) { /* LDRH Rd, [Rn, Rm] */
        instr_ldrh_reg_offset(instr);

    } else if ((instr & 0xFE00) == 0x5E00) { /* LDRSH Rd, [Rn, Rm] */
        instr_ldrsh_reg_offset(instr);

    } else if ((instr & 0xF800) == 0x8000) { /* STRH Rd, [Rn, #imm5] */
        instr_strh_imm5(instr);

    } else if ((instr & 0xFE00) == 0x5200) { /* STRH Rd, [Rn, Rm] */
        instr_strh_reg_offset(instr);

    /* -------- Shift Operations - Immediate -------- */
    } else if ((instr & 0xF800) == 0x0000) { /* LSLS Rd, Rm, #imm5 */
        if (instr != 0x0000) { /* Not NOP */
            instr_shift_logical_left(instr);
        }

    } else if ((instr & 0xF800) == 0x0800) { /* LSRS Rd, Rm, #imm5 */
        instr_shift_logical_right(instr);

    } else if ((instr & 0xF800) == 0x1000) { /* ASRS Rd, Rm, #imm5 */
        instr_shift_arithmetic_right(instr);

    /* -------- Shift Operations - Register -------- */
    } else if ((instr & 0xFFC0) == 0x4080) { /* LSLS Rd, Rs */
        instr_lsls_reg(instr);

    } else if ((instr & 0xFFC0) == 0x40C0) { /* LSRS Rd, Rs */
        instr_lsrs_reg(instr);

    } else if ((instr & 0xFFC0) == 0x4100) { /* ASRS Rd, Rs */
        instr_asrs_reg(instr);

    } else if ((instr & 0xFFC0) == 0x41C0) { /* RORS Rd, Rs */
        instr_rors_reg(instr);

    /* -------- Logical Operations -------- */
    } else if ((instr & 0xFFC0) == 0x4000) { /* ANDS Rd, Rm */
        instr_bitwise_and(instr);

    } else if ((instr & 0xFFC0) == 0x4040) { /* EORS Rd, Rm */
        instr_bitwise_eor(instr);

    } else if ((instr & 0xFFC0) == 0x4300) { /* ORRS Rd, Rm */
        instr_bitwise_orr(instr);

    } else if ((instr & 0xFFC0) == 0x4380) { /* BICS Rd, Rm */
        instr_bitwise_bic(instr);

    } else if ((instr & 0xFFC0) == 0x43C0) { /* MVNS Rd, Rm */
        instr_bitwise_mvn(instr);

    /* -------- Multiplication -------- */
    } else if ((instr & 0xFFC0) == 0x4340) { /* MULS Rd, Rm */
        instr_muls(instr);

    /* -------- Multiple Load/Store -------- */
    } else if ((instr & 0xF800) == 0xC000) { /* STMIA Rn!, {reglist} */
        instr_stmia(instr);

    } else if ((instr & 0xF800) == 0xC800) { /* LDMIA Rn!, {reglist} */
        instr_ldmia(instr);

    // ========================================================================
    // IMPORTANT INSTRUCTIONS
    // ========================================================================

    /* -------- Special Comparison -------- */
    } else if ((instr & 0xFFC0) == 0x42C0) { /* CMN Rn, Rm */
        instr_cmn_reg(instr);

    } else if ((instr & 0xFFC0) == 0x4200) { /* TST Rn, Rm */
        instr_tst_reg_reg(instr);

    /* -------- System Operations -------- */
    } else if ((instr & 0xFF00) == 0xDF00) { /* SVC #imm8 */
        instr_svc(instr);

    // ========================================================================
    // OPTIONAL INSTRUCTIONS
    // ========================================================================

    /* -------- Hints and Special -------- */
    } else if (instr == 0xBF00) { /* NOP */
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

    /* -------- Sign/Zero Extend -------- */
    } else if ((instr & 0xFFC0) == 0xB240) { /* SXTB Rd, Rm */
        instr_sxtb(instr);

    } else if ((instr & 0xFFC0) == 0xB200) { /* SXTH Rd, Rm */
        instr_sxth(instr);

    } else if ((instr & 0xFFC0) == 0xB2C0) { /* UXTB Rd, Rm */
        instr_uxtb(instr);

    } else if ((instr & 0xFFC0) == 0xB280) { /* UXTH Rd, Rm */
        instr_uxth(instr);

    /* -------- Byte Reverse -------- */
    } else if ((instr & 0xFFC0) == 0xBA00) { /* REV Rd, Rm */
        instr_rev(instr);

    } else if ((instr & 0xFFC0) == 0xBA40) { /* REV16 Rd, Rm */
        instr_rev16(instr);

    } else if ((instr & 0xFFC0) == 0xBAC0) { /* REVSH Rd, Rm */
        instr_revsh(instr);

    /* -------- Stack Pointer Adjustment -------- */
    } else if ((instr & 0xFF80) == 0xB000) { /* ADD SP, #imm7 */
        instr_add_sp_imm7(instr);

    } else if ((instr & 0xFF80) == 0xB080) { /* SUB SP, #imm7 */
        instr_sub_sp_imm7(instr);

    /* -------- Address Generation -------- */
    } else if ((instr & 0xF800) == 0xA000) { /* ADR Rd, label */
        instr_adr(instr);

    } else if ((instr & 0xF800) == 0xA800) { /* ADD Rd, SP, #imm8 */
        /* Use existing logic or create wrapper */
        uint8_t rd = (instr >> 8) & 0x07;
        uint8_t imm8 = instr & 0xFF;
        cpu.r[rd] = cpu.r[13] + (imm8 << 2);

    /* -------- Interrupt Control -------- */
    } else if ((instr & 0xFFEF) == 0xB662) { /* CPSID i */
        instr_cpsid(instr);

    } else if ((instr & 0xFFEF) == 0xB660) { /* CPSIE i */
        instr_cpsie(instr);

    // ========================================================================
    // UNKNOWN/UNIMPLEMENTED
    // ========================================================================

    } else {
        /* Unknown / unimplemented instruction: log and halt */
        instr_unimplemented(instr);
        timer_tick(1);
        return;
    }

    /* Advance to next 16-bit instruction (if not already handled) */
    cpu.r[15] = pc + 2;
}

/* ========================================================================
 * Dual-Core Initialization
 * ======================================================================== */

cpu_state_dual_t cores[NUM_CORES] = {0};
uint32_t shared_ram[SHARED_RAM_SIZE / 4] = {0};
uint32_t spinlocks[SPINLOCK_SIZE] = {0};
multicore_fifo_t fifo[NUM_CORES] = {0};

static int active_core = CORE0;

int get_active_core(void) {
    return active_core;
}

void set_active_core(int core_id) {
    if (core_id < NUM_CORES) {
        active_core = core_id;
    }
}

void dual_core_init(void) {
    /* Initialize core structures */
    for (int i = 0; i < NUM_CORES; i++) {
        memset(&cores[i], 0, sizeof(cpu_state_dual_t));
        memset(cores[i].ram, 0, CORE_RAM_SIZE);

        cores[i].core_id = i;
        cores[i].is_halted = (i == CORE1) ? 1 : 0;
        cores[i].xpsr = 0x01000000;
        cores[i].vtor = 0x10000100;
        cores[i].current_irq = 0xFFFFFFFF;

        printf("[CORE%d] Initialized (halted: %d)\n", i, cores[i].is_halted);
    }

    /* ✅ COPY FIRMWARE FROM SINGLE-CORE TO DUAL-CORE */
    /* This ensures dual_core_step() can access the loaded firmware */
    memcpy(cores[CORE0].flash, cpu.flash, FLASH_SIZE);
    memcpy(cores[CORE0].ram, cpu.ram, CORE_RAM_SIZE);

    printf("[Boot] Firmware copied to CORE0 (flash: %u bytes, ram: %u bytes)\n",
           FLASH_SIZE, CORE_RAM_SIZE);

    /* ✅ READ VECTOR TABLE FROM FLASH (same as single-core) */
    /* This was missing in the original implementation */
    uint32_t vector_table = FLASH_BASE + 0x100;
    uint32_t initial_sp = mem_read32(vector_table);
    uint32_t reset_vector = mem_read32(vector_table + 4);

    /* Set Core 0 registers from vector table */
    cores[CORE0].r[13] = initial_sp;           /* SP (R13) */
    cores[CORE0].r[15] = reset_vector & ~1;    /* PC (R15), clear Thumb bit */

    if (initial_sp != 0 || reset_vector != 0) {
        printf("[Boot] Vector table loaded: SP=0x%08X, PC=0x%08X\n",
               initial_sp, reset_vector & ~1);
    }

    /* Core 1 stays in reset (no vector table init needed) */

    /* Initialize FIFO channels */
    for (int i = 0; i < NUM_CORES; i++) {
        fifo[i].count = 0;
        fifo[i].read_ptr = 0;
        fifo[i].write_ptr = 0;
    }
}

/* ========================================================================
 * Dual-Core CPU Execution
 * ======================================================================== */



/* 
 * Helper: Context-switch a specific core into the single-core engine
 * to execute one instruction using the full 'instructions.c' decoder.
 */
static void cpu_step_core_via_single(int core_id) {
    if (core_id >= NUM_CORES) return;
    if (cores[core_id].is_halted) return;

    // 1. Activate the core (for shared peripheral logic)
    set_active_core(core_id);

    // 2. Backup the global single-core state (so we don't corrupt it)
    cpu_state_t saved_cpu = cpu;

    // 3. Load the target core's state into the global 'cpu' structure
    memset(cpu.r, 0, sizeof(cpu.r));
    memcpy(cpu.r, cores[core_id].r, sizeof(cpu.r));
    cpu.xpsr         = cores[core_id].xpsr;
    cpu.vtor         = cores[core_id].vtor;
    cpu.step_count   = cores[core_id].step_count;
    cpu.debug_enabled= cores[core_id].debug_enabled;
    cpu.current_irq  = cores[core_id].current_irq;
    
    // 4. Map memory contexts
    //    - Flash is shared (copy from Core 0 storage)
    //    - RAM is per-core
    memcpy(cpu.flash, cores[0].flash, FLASH_SIZE);
    memcpy(cpu.ram,   cores[core_id].ram, CORE_RAM_SIZE);

    // 5. Execute ONE instruction using the single-core engine
    //    This handles LDR, CMP, B, BL, PUSH/POP, etc. correctly.
    cpu_step();

    // 6. Save the updated state back to the target core
    memcpy(cores[core_id].r, cpu.r, sizeof(cpu.r));
    cores[core_id].xpsr         = cpu.xpsr;
    cores[core_id].vtor         = cpu.vtor;
    cores[core_id].step_count   = cpu.step_count;
    cores[core_id].current_irq  = cpu.current_irq;
    
    //    Check if the instruction halted the CPU (PC=0xFFFFFFFF)
    if (cpu.r[15] == 0xFFFFFFFF) {
        cores[core_id].is_halted = 1;
    }

    // 7. Write back RAM (in case the instruction was STR or PUSH)
    memcpy(cores[core_id].ram, cpu.ram, CORE_RAM_SIZE);

    // 8. Restore the original global single-core state
    cpu = saved_cpu;
}

/* 
 * Unified Dual-Core Stepper
 */
void cpu_step_core(int core_id) {
    #define CORE1_ENTRY_OFFSET 0x2C  /* Example - check your binary */
    uint32_t core1_entry = mem_read32(FLASH_BASE + CORE1_ENTRY_OFFSET);

    if ((core1_entry & 0x1) || (core1_entry >= FLASH_BASE)) {
        cores[CORE1].is_halted = 0;
        cores[CORE1].r[15] = core1_entry & ~1;
        printf("[Boot] Core 1 entry: 0x%08X\n", core1_entry & ~1);
    }
    cpu_step_core_via_single(core_id);
}


void dual_core_step(void) {
    static int current = 0;

    for (int i = 0; i < NUM_CORES; i++) {
        cpu_step_core(current);
        current = (current + 1) % NUM_CORES;
    }
}

int cpu_is_halted_core(int core_id) {
    if (core_id >= NUM_CORES) return 1;
    return cores[core_id].is_halted;
}

void cpu_reset_core(int core_id) {
    if (core_id >= NUM_CORES) return;

    cpu_state_dual_t *cpu = &cores[core_id];
    set_active_core(core_id);

    memset(cpu->r, 0, sizeof(cpu->r));
    cpu->xpsr = 0x01000000;
    cpu->step_count = 0;
    cpu->is_halted = (core_id == CORE1) ? 1 : 0;
    cpu->vtor = 0x10000100;

    if (core_id == CORE0) {
        uint32_t vector_table = FLASH_BASE + 0x100;
        uint32_t initial_sp = mem_read32(vector_table);
        uint32_t reset_vector = mem_read32(vector_table + 4);

        cpu->r[13] = initial_sp;
        cpu->r[15] = reset_vector & ~1;

        if (cpu->debug_enabled) {
            printf("[CORE%d] Reset to PC=0x%08X SP=0x%08X\n",
                   core_id, cpu->r[15], cpu->r[13]);
        }
    }
}

/* ========================================================================
 * Dual-Core Exception Handling
 * ======================================================================== */

void cpu_exception_entry_dual(int core_id, uint32_t vector_num) {
    if (core_id >= NUM_CORES) return;

    cpu_state_dual_t *cpu = &cores[core_id];
    set_active_core(core_id);

    uint32_t vector_offset = vector_num * 4;
    uint32_t handler_addr = mem_read32_dual(core_id, cpu->vtor + vector_offset);

    if (cpu->debug_enabled) {
        printf("[CORE%d] Exception %u -> Handler 0x%08X\n",
               core_id, vector_num, handler_addr);
    }

    uint32_t sp = cpu->r[13];

    sp -= 4; mem_write32_dual(core_id, sp, cpu->xpsr);
    sp -= 4; mem_write32_dual(core_id, sp, cpu->r[15]);
    sp -= 4; mem_write32_dual(core_id, sp, cpu->r[14]);
    sp -= 4; mem_write32_dual(core_id, sp, cpu->r[12]);
    sp -= 4; mem_write32_dual(core_id, sp, cpu->r[3]);
    sp -= 4; mem_write32_dual(core_id, sp, cpu->r[2]);
    sp -= 4; mem_write32_dual(core_id, sp, cpu->r[1]);
    sp -= 4; mem_write32_dual(core_id, sp, cpu->r[0]);

    cpu->r[13] = sp;
    cpu->r[15] = handler_addr & ~1u;
    cpu->r[14] = 0xFFFFFFF9;
    cpu->in_handler_mode = 1;
}

void cpu_exception_return_dual(int core_id, uint32_t lr_value) {
    (void)lr_value; /* Mark as intentionally unused */

    if (core_id >= NUM_CORES) return;

    cpu_state_dual_t *cpu = &cores[core_id];
    set_active_core(core_id);

    uint32_t sp = cpu->r[13];

    uint32_t r0 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t r1 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t r2 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t r3 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t r12 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t lr = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t pc = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t xpsr = mem_read32_dual(core_id, sp); sp += 4;

    cpu->r[0] = r0;
    cpu->r[1] = r1;
    cpu->r[2] = r2;
    cpu->r[3] = r3;
    cpu->r[12] = r12;
    cpu->r[13] = sp;
    cpu->r[14] = lr;
    cpu->r[15] = pc & ~1u;
    cpu->xpsr = xpsr;
    cpu->in_handler_mode = 0;

    if (cpu->debug_enabled) {
        printf("[CORE%d] Exception return to PC=0x%08X\n", core_id, cpu->r[15]);
    }
}

int any_core_running(void) {
    for (int i = 0; i < NUM_CORES; i++) {
        if (!cores[i].is_halted) {
            return 1;
        }
    }
    return 0;
}

/* Dual-core status reporting */
void dual_core_status(void) {
    printf("[DUAL-CORE STATUS]\n");
    for (int i = 0; i < NUM_CORES; i++) {
        printf("[CORE%d] Status: %s\n", i, cores[i].is_halted ? "HALTED" : "RUNNING");
        printf("[CORE%d] PC=0x%08X SP=0x%08X\n", i, cores[i].r[15], cores[i].r[13]);
        printf("[CORE%d] Step count: %u\n", i, cores[i].step_count);
    }
}

void cpu_set_debug_core(int core_id, int enabled) {
    if (core_id >= NUM_CORES) return;
    cores[core_id].debug_enabled = enabled;
}

/* ========================================================================
 * SIO (Single-Cycle I/O) Operations
 * ======================================================================== */

uint32_t sio_get_core_id(void) {
    return get_active_core();
}

void sio_set_core1_reset(int assert_reset) {
    if (assert_reset) {
        cores[CORE1].is_halted = 1;
    } else {
        cores[CORE1].is_halted = 0;
    }
}

void sio_set_core1_stall(int stall) {
    (void)stall; /* Not fully implemented */
}

/* ========================================================================
 * Spinlock Operations
 * ======================================================================== */

uint32_t spinlock_acquire(uint32_t lock_num) {
    if (lock_num >= SPINLOCK_SIZE) return 0;

    while ((spinlocks[lock_num] & SPINLOCK_LOCKED) == 0) {
        spinlocks[lock_num] = SPINLOCK_VALID | SPINLOCK_LOCKED;
    }

    return spinlocks[lock_num];
}

void spinlock_release(uint32_t lock_num) {
    if (lock_num >= SPINLOCK_SIZE) return;
    spinlocks[lock_num] = 0;
}

/* ========================================================================
 * Multicore FIFO Operations
 * ======================================================================== */

int fifo_is_empty(int core_id) {
    if (core_id >= NUM_CORES) return 1;
    return fifo[core_id].count == 0;
}

int fifo_is_full(int core_id) {
    if (core_id >= NUM_CORES) return 1;
    return fifo[core_id].count >= FIFO_DEPTH;
}

uint32_t fifo_pop(int core_id) {
    if (core_id >= NUM_CORES) return 0;

    /* Wait for data to be available */
    while (fifo[core_id].count == 0) {
        /* Spin-wait */
    }

    uint32_t val = fifo[core_id].messages[fifo[core_id].read_ptr];
    fifo[core_id].read_ptr = (fifo[core_id].read_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count--;

    return val;
}

void fifo_push(int core_id, uint32_t val) {
    if (core_id >= NUM_CORES) return;

    /* Wait for space */
    while (fifo[core_id].count >= FIFO_DEPTH) {
        /* Spin-wait */
    }

    fifo[core_id].messages[fifo[core_id].write_ptr] = val;
    fifo[core_id].write_ptr = (fifo[core_id].write_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count++;
}

int fifo_try_pop(int core_id, uint32_t *val) {
    if (core_id >= NUM_CORES) return 0;

    if (fifo[core_id].count == 0) {
        return 0; /* No data available */
    }

    *val = fifo[core_id].messages[fifo[core_id].read_ptr];
    fifo[core_id].read_ptr = (fifo[core_id].read_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count--;

    return 1; /* Success */
}

int fifo_try_push(int core_id, uint32_t val) {
    if (core_id >= NUM_CORES) return 0;

    if (fifo[core_id].count >= FIFO_DEPTH) {
        return 0; /* FIFO full */
    }

    fifo[core_id].messages[fifo[core_id].write_ptr] = val;
    fifo[core_id].write_ptr = (fifo[core_id].write_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count++;

    return 1; /* Success */
}
