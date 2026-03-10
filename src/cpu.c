/*
 * RP2040 Emulator CPU Engine (Unified Single & Dual-Core)
 *
 * Consolidated implementation providing:
 * - Single-core CPU execution (cpu_step) with O(1) dispatch table
 * - Dual-core CPU execution with zero-copy context switching
 * - Exception handling for both cores
 * - PRIMASK-aware interrupt delivery
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "emulator.h"
#include "instructions.h"
#include "timer.h"
#include "nvic.h"
#include "rom.h"
#include "rtc.h"
#include "corepool.h"

/* ========================================================================
 * Single-Core Global State
 * ======================================================================== */

cpu_state_t cpu = {0};
int pc_updated = 0;

/* ========================================================================
 * Cycle-Accurate Timing
 * ======================================================================== */

timing_config_t timing_config = {
    .cycles_per_us = 1,         /* Default: 1 cycle = 1 µs (fast-forward) */
    .cycle_accumulator = 0,
};

void timing_set_clock_mhz(uint32_t mhz) {
    if (mhz == 0) mhz = 1;
    timing_config.cycles_per_us = mhz;
    timing_config.cycle_accumulator = 0;
}

/*
 * ARMv6-M (Cortex-M0+) instruction cycle costs.
 * Reference: ARM DDI 0484C, Cortex-M0+ Technical Reference Manual, Table 3-1.
 *
 * Most data processing: 1 cycle
 * Branch (taken): 2 cycles (pipeline refill)
 * Branch (not taken): 1 cycle
 * LDR/STR: 2 cycles
 * LDR Rd, [PC, #]: 2 cycles
 * LDR/STR register-offset: 2 cycles
 * PUSH/POP: 1 + N (N = register count)
 * LDMIA/STMIA: 1 + N
 * BX/BLX: 3 cycles
 * BL (32-bit): 4 cycles
 * MUL: 1 cycle (M0+ has single-cycle multiplier)
 * MSR/MRS: 4 cycles
 * DSB/DMB/ISB: 3 cycles
 */
uint32_t timing_instruction_cycles(uint16_t instr, int branch_taken) {
    uint8_t top8 = instr >> 8;

    /* Shifts (top8 0x00-0x17): LSL/LSR/ASR imm5 — 1 cycle */
    if (top8 <= 0x17) return 1;

    /* ADD/SUB reg/imm3 (top8 0x18-0x1F) — 1 cycle */
    if (top8 <= 0x1F) return 1;

    /* MOVS/CMP/ADDS/SUBS imm8 (top8 0x20-0x3F) — 1 cycle */
    if (top8 <= 0x3F) return 1;

    /* ALU operations (0x40-0x43): 1 cycle */
    if (top8 <= 0x43) return 1;

    /* Special data processing (0x44-0x46): ADD/CMP/MOV high regs: 1 cycle */
    if (top8 <= 0x46) return 1;

    /* BX/BLX register (0x47): 3 cycles */
    if (top8 == 0x47) return 3;

    /* LDR Rd, [PC, #imm8] (0x48-0x4F): 2 cycles */
    if (top8 <= 0x4F) return 2;

    /* Load/store register offset (0x50-0x5F): 2 cycles */
    if (top8 <= 0x5F) return 2;

    /* STR/LDR imm5 word (0x60-0x6F): 2 cycles */
    if (top8 <= 0x6F) return 2;

    /* STRB/LDRB imm5 (0x70-0x7F): 2 cycles */
    if (top8 <= 0x7F) return 2;

    /* STRH/LDRH imm5 (0x80-0x8F): 2 cycles */
    if (top8 <= 0x8F) return 2;

    /* STR/LDR SP-relative (0x90-0x9F): 2 cycles */
    if (top8 <= 0x9F) return 2;

    /* ADR (0xA0-0xA7): 1 cycle */
    if (top8 >= 0xA0 && top8 <= 0xA7) return 1;

    /* ADD Rd, SP, #imm8 (0xA8-0xAF): 1 cycle */
    if (top8 >= 0xA8 && top8 <= 0xAF) return 1;

    /* Miscellaneous 16-bit (0xB0-0xBF) */
    if (top8 == 0xB0) return 1;  /* ADD/SUB SP */
    if (top8 == 0xB2) return 1;  /* SXTH/SXTB/UXTH/UXTB */
    if (top8 == 0xB4 || top8 == 0xB5) {
        /* PUSH: 1 + N (count set bits in reglist + LR bit) */
        int n = __builtin_popcount(instr & 0xFF) + ((instr >> 8) & 1);
        return 1 + (uint32_t)n;
    }
    if (top8 == 0xB6) return 1;  /* CPS */
    if (top8 == 0xBA) return 1;  /* REV/REV16/REVSH */
    if (top8 == 0xBC || top8 == 0xBD) {
        /* POP: 1 + N (count set bits in reglist + PC bit) */
        int n = __builtin_popcount(instr & 0xFF) + ((instr >> 8) & 1);
        /* If popping PC, extra pipeline refill cycle */
        if (instr & 0x100) n++;
        return 1 + (uint32_t)n;
    }
    if (top8 == 0xBE) return 1;  /* BKPT */
    if (top8 == 0xBF) return 1;  /* NOP/hints */

    /* STMIA/LDMIA (0xC0-0xCF): 1 + N */
    if (top8 >= 0xC0 && top8 <= 0xCF) {
        int n = __builtin_popcount(instr & 0xFF);
        return 1 + (uint32_t)n;
    }

    /* Conditional branch (0xD0-0xDD): 1 if not taken, 2 if taken */
    if (top8 >= 0xD0 && top8 <= 0xDD) return branch_taken ? 2 : 1;

    /* UDF (0xDE): 1 cycle */
    if (top8 == 0xDE) return 1;

    /* SVC (0xDF): triggers exception entry, counted separately */
    if (top8 == 0xDF) return 1;

    /* Unconditional branch (0xE0-0xE7): 2 cycles (always taken) */
    if (top8 >= 0xE0 && top8 <= 0xE7) return 2;

    return 1;  /* Default */
}

uint32_t timing_instruction_cycles_32(uint16_t upper, uint16_t lower) {
    /* BL: 4 cycles */
    if ((upper & 0xF800) == 0xF000 && (lower & 0xD000) == 0xD000)
        return 4;

    /* MSR: 4 cycles */
    if ((upper & 0xFFF0) == 0xF380 && (lower & 0xFF00) == 0x8800)
        return 4;

    /* MRS: 4 cycles */
    if ((upper & 0xFFFF) == 0xF3EF && (lower & 0xF000) == 0x8000)
        return 4;

    /* DSB/DMB/ISB: 3 cycles */
    if ((upper & 0xFFFF) == 0xF3BF && (lower & 0xFF00) == 0x8F00)
        return 3;

    return 2;  /* Default for unknown 32-bit */
}

/* ========================================================================
 * Instruction Dispatch Table
 *
 * 256-entry table indexed by bits [15:8] of 16-bit Thumb instruction.
 * Secondary dispatchers handle entries where multiple instructions
 * share the same top byte.
 * ======================================================================== */

typedef void (*thumb_handler_t)(uint16_t);
static thumb_handler_t dispatch_table[256];
static int dispatch_initialized = 0;

/* Secondary dispatchers for ALU block (0x40-0x43) */
static void dispatch_alu_40(uint16_t instr) {
    switch ((instr >> 6) & 0x3) {
        case 0: instr_bitwise_and(instr); break;
        case 1: instr_bitwise_eor(instr); break;
        case 2: instr_lsls_reg(instr); break;
        case 3: instr_lsrs_reg(instr); break;
    }
}

static void dispatch_alu_41(uint16_t instr) {
    switch ((instr >> 6) & 0x3) {
        case 0: instr_asrs_reg(instr); break;
        case 1: instr_adcs(instr); break;
        case 2: instr_sbcs(instr); break;
        case 3: instr_rors_reg(instr); break;
    }
}

static void dispatch_alu_42(uint16_t instr) {
    switch ((instr >> 6) & 0x3) {
        case 0: instr_tst_reg_reg(instr); break;
        case 1: instr_rsbs(instr); break;
        case 2: instr_cmp_alu(instr); break;
        case 3: instr_cmn_reg(instr); break;
    }
}

static void dispatch_alu_43(uint16_t instr) {
    switch ((instr >> 6) & 0x3) {
        case 0: instr_bitwise_orr(instr); break;
        case 1: instr_muls(instr); break;
        case 2: instr_bitwise_bic(instr); break;
        case 3: instr_bitwise_mvn(instr); break;
    }
}

/* Special data processing / branch exchange (0x44-0x47) */
static void dispatch_special_47(uint16_t instr) {
    if (instr & 0x80) {
        instr_blx(instr);
    } else {
        instr_bx(instr);
    }
}

/* Misc 16-bit: ADD/SUB SP (0xB0) */
static void dispatch_sp_b0(uint16_t instr) {
    if (instr & 0x80) {
        instr_sub_sp_imm7(instr);
    } else {
        instr_add_sp_imm7(instr);
    }
}

/* Sign/zero extend (0xB2) */
static void dispatch_extend_b2(uint16_t instr) {
    switch ((instr >> 6) & 0x3) {
        case 0: instr_sxth(instr); break;
        case 1: instr_sxtb(instr); break;
        case 2: instr_uxth(instr); break;
        case 3: instr_uxtb(instr); break;
    }
}

/* CPS instructions (0xB6) */
static void dispatch_cps_b6(uint16_t instr) {
    if (instr & 0x10) {
        instr_cpsid(instr);
    } else {
        instr_cpsie(instr);
    }
}

/* REV instructions (0xBA) */
static void dispatch_rev_ba(uint16_t instr) {
    switch ((instr >> 6) & 0x3) {
        case 0: instr_rev(instr); break;
        case 1: instr_rev16(instr); break;
        case 3: instr_revsh(instr); break;
        default: instr_unimplemented(instr); break;
    }
}

/* Hints (0xBF) */
static void dispatch_hints_bf(uint16_t instr) {
    uint8_t op = (instr >> 4) & 0xF;
    switch (op) {
        case 0x0: instr_nop(instr); break;
        case 0x1: instr_yield(instr); break;
        case 0x2: instr_wfe(instr); break;
        case 0x3: instr_wfi(instr); break;
        case 0x4: instr_sev(instr); break;
        default:  instr_it(instr); break;
    }
}

/* ADD Rd, SP, #imm8 (0xA8-0xAF) */
static void dispatch_add_sp_rd(uint16_t instr) {
    uint8_t rd = (instr >> 8) & 0x07;
    uint8_t imm8 = instr & 0xFF;
    cpu.r[rd] = cpu.r[13] + (imm8 << 2);
}

static void init_dispatch_table(void) {
    /* Default all to unimplemented */
    for (int i = 0; i < 256; i++) {
        dispatch_table[i] = instr_unimplemented;
    }

    /* LSLS Rd, Rm, #imm5: 000 00xxx */
    for (int i = 0x00; i <= 0x07; i++) dispatch_table[i] = instr_shift_logical_left;

    /* LSRS Rd, Rm, #imm5: 000 01xxx */
    for (int i = 0x08; i <= 0x0F; i++) dispatch_table[i] = instr_shift_logical_right;

    /* ASRS Rd, Rm, #imm5: 000 10xxx */
    for (int i = 0x10; i <= 0x17; i++) dispatch_table[i] = instr_shift_arithmetic_right;

    /* ADDS Rd, Rn, Rm: 000 110xx */
    dispatch_table[0x18] = instr_adds_reg_reg;
    dispatch_table[0x19] = instr_adds_reg_reg;

    /* SUBS Rd, Rn, Rm: 000 111xx (actually 0x1A-0x1B) */
    dispatch_table[0x1A] = instr_sub_reg_reg;
    dispatch_table[0x1B] = instr_sub_reg_reg;

    /* ADDS Rd, Rn, #imm3: 000 11100-11101 -> 0x1C-0x1D */
    dispatch_table[0x1C] = instr_adds_imm3;
    dispatch_table[0x1D] = instr_adds_imm3;

    /* SUBS Rd, Rn, #imm3: 0x1E-0x1F */
    dispatch_table[0x1E] = instr_subs_imm3;
    dispatch_table[0x1F] = instr_subs_imm3;

    /* MOVS Rd, #imm8: 001 00xxx */
    for (int i = 0x20; i <= 0x27; i++) dispatch_table[i] = instr_movs_imm8;

    /* CMP Rn, #imm8: 001 01xxx */
    for (int i = 0x28; i <= 0x2F; i++) dispatch_table[i] = instr_cmp_imm8;

    /* ADDS Rd, #imm8: 001 10xxx */
    for (int i = 0x30; i <= 0x37; i++) dispatch_table[i] = instr_adds_imm8;

    /* SUBS Rd, #imm8: 001 11xxx */
    for (int i = 0x38; i <= 0x3F; i++) dispatch_table[i] = instr_subs_imm8;

    /* ALU operations: 010000xx */
    dispatch_table[0x40] = dispatch_alu_40;
    dispatch_table[0x41] = dispatch_alu_41;
    dispatch_table[0x42] = dispatch_alu_42;
    dispatch_table[0x43] = dispatch_alu_43;

    /* Special data / branch exchange: 010001xx */
    dispatch_table[0x44] = instr_add_reg_high;
    dispatch_table[0x45] = instr_cmp_reg_reg;
    dispatch_table[0x46] = instr_mov_high_reg;
    dispatch_table[0x47] = dispatch_special_47;

    /* LDR Rd, [PC, #imm8]: 01001xxx */
    for (int i = 0x48; i <= 0x4F; i++) dispatch_table[i] = instr_ldr_pc_imm8;

    /* Register-offset load/store: 0101 xxx */
    dispatch_table[0x50] = instr_str_reg_offset;
    dispatch_table[0x51] = instr_str_reg_offset;
    dispatch_table[0x52] = instr_strh_reg_offset;
    dispatch_table[0x53] = instr_strh_reg_offset;
    dispatch_table[0x54] = instr_strb_reg_offset;
    dispatch_table[0x55] = instr_strb_reg_offset;
    dispatch_table[0x56] = instr_ldrsb_reg_offset;
    dispatch_table[0x57] = instr_ldrsb_reg_offset;
    dispatch_table[0x58] = instr_ldr_reg_offset;
    dispatch_table[0x59] = instr_ldr_reg_offset;
    dispatch_table[0x5A] = instr_ldrh_reg_offset;
    dispatch_table[0x5B] = instr_ldrh_reg_offset;
    dispatch_table[0x5C] = instr_ldrb_reg_offset;
    dispatch_table[0x5D] = instr_ldrb_reg_offset;
    dispatch_table[0x5E] = instr_ldrsh_reg_offset;
    dispatch_table[0x5F] = instr_ldrsh_reg_offset;

    /* STR Rd, [Rn, #imm5]: 0110 0xxx */
    for (int i = 0x60; i <= 0x67; i++) dispatch_table[i] = instr_str_imm5;

    /* LDR Rd, [Rn, #imm5]: 0110 1xxx */
    for (int i = 0x68; i <= 0x6F; i++) dispatch_table[i] = instr_ldr_imm5;

    /* STRB Rd, [Rn, #imm5]: 0111 0xxx */
    for (int i = 0x70; i <= 0x77; i++) dispatch_table[i] = instr_strb_imm5;

    /* LDRB Rd, [Rn, #imm5]: 0111 1xxx */
    for (int i = 0x78; i <= 0x7F; i++) dispatch_table[i] = instr_ldrb_imm5;

    /* STRH Rd, [Rn, #imm5]: 1000 0xxx */
    for (int i = 0x80; i <= 0x87; i++) dispatch_table[i] = instr_strh_imm5;

    /* LDRH Rd, [Rn, #imm5]: 1000 1xxx */
    for (int i = 0x88; i <= 0x8F; i++) dispatch_table[i] = instr_ldrh_imm5;

    /* STR Rd, [SP, #imm8]: 1001 0xxx */
    for (int i = 0x90; i <= 0x97; i++) dispatch_table[i] = instr_str_sp_imm8;

    /* LDR Rd, [SP, #imm8]: 1001 1xxx */
    for (int i = 0x98; i <= 0x9F; i++) dispatch_table[i] = instr_ldr_sp_imm8;

    /* ADR Rd, label: 1010 0xxx */
    for (int i = 0xA0; i <= 0xA7; i++) dispatch_table[i] = instr_adr;

    /* ADD Rd, SP, #imm8: 1010 1xxx */
    for (int i = 0xA8; i <= 0xAF; i++) dispatch_table[i] = dispatch_add_sp_rd;

    /* Miscellaneous 16-bit instructions: 1011 xxxx */
    dispatch_table[0xB0] = dispatch_sp_b0;
    /* 0xB1: CBZ (not supported on M0+) */
    dispatch_table[0xB2] = dispatch_extend_b2;
    /* 0xB3: CBZ (not supported on M0+) */
    dispatch_table[0xB4] = instr_push;
    dispatch_table[0xB5] = instr_push;
    dispatch_table[0xB6] = dispatch_cps_b6;
    dispatch_table[0xBA] = dispatch_rev_ba;
    dispatch_table[0xBC] = instr_pop;
    dispatch_table[0xBD] = instr_pop;
    dispatch_table[0xBE] = instr_bkpt;
    dispatch_table[0xBF] = dispatch_hints_bf;

    /* STMIA Rn!, {reglist}: 1100 0xxx */
    for (int i = 0xC0; i <= 0xC7; i++) dispatch_table[i] = instr_stmia;

    /* LDMIA Rn!, {reglist}: 1100 1xxx */
    for (int i = 0xC8; i <= 0xCF; i++) dispatch_table[i] = instr_ldmia;

    /* B{cond}: 1101 cccc (conditions 0-13) */
    for (int i = 0xD0; i <= 0xDD; i++) dispatch_table[i] = instr_bcond;
    /* 0xDE: UDF (permanently undefined) */
    dispatch_table[0xDE] = instr_udf;
    /* 0xDF: SVC */
    dispatch_table[0xDF] = instr_svc;

    /* B (unconditional): 1110 0xxx */
    for (int i = 0xE0; i <= 0xE7; i++) dispatch_table[i] = instr_b_uncond;

    /* 32-bit instruction prefixes (0xE8-0xEF, 0xF0-0xFF) are handled
     * separately in cpu_step before the dispatch table lookup */
}

/* ========================================================================
 * Single-Core Initialization & Reset
 * ======================================================================== */

void cpu_init(void) {
    memset(cpu.r, 0, sizeof(cpu.r));
    cpu.xpsr = 0x01000000; /* Thumb bit */
    cpu.step_count = 0;
    cpu.debug_enabled = 0;
    cpu.current_irq = 0xFFFFFFFF;
    cpu.primask = 0;
    cpu.control = 0;
    cpu.vtor = 0x10000100;

    /* Initialize memory bus to use cpu.ram */
    mem_set_ram_ptr(cpu.ram, RAM_BASE, RAM_SIZE);

    /* Initialize dispatch table */
    if (!dispatch_initialized) {
        init_dispatch_table();
        dispatch_initialized = 1;
    }
}

void cpu_reset_from_flash(void) {
    cpu.vtor = FLASH_BASE + 0x100;

    uint32_t initial_sp = mem_read32(cpu.vtor + 0x00);
    uint32_t reset_vector = mem_read32(cpu.vtor + 0x04);

    if (initial_sp < RAM_BASE || initial_sp > RAM_BASE + RAM_SIZE) {
        fprintf(stderr, "[Boot] ERROR: Invalid SP 0x%08X (not in RAM 0x%08X-0x%08X)\n",
               initial_sp, RAM_BASE, RAM_BASE + RAM_SIZE);
        cpu.r[15] = 0xFFFFFFFF;
        return;
    }

    if ((reset_vector & 0x1) == 0) {
        fprintf(stderr, "[Boot] ERROR: Invalid reset vector 0x%08X (Thumb bit not set)\n",
               reset_vector);
        cpu.r[15] = 0xFFFFFFFF;
        return;
    }

    cpu.r[13] = initial_sp;
    cpu.r[15] = reset_vector & ~1u;
    cpu.r[14] = 0xFFFFFFFF;
    cpu.xpsr = 0x01000000;

    fprintf(stderr, "[Boot] Reset complete:\n");
    fprintf(stderr, "[Boot] VTOR = 0x%08X\n", cpu.vtor);
    fprintf(stderr, "[Boot] SP = 0x%08X\n", cpu.r[13]);
    fprintf(stderr, "[Boot] PC = 0x%08X\n", cpu.r[15]);
    fprintf(stderr, "[Boot] xPSR = 0x%08X\n", cpu.xpsr);
}

/* Allow execution from both flash and RAM */
int cpu_is_halted(void) {
    uint32_t pc = cpu.r[15];

    if (pc == 0xFFFFFFFF) {
        return 1;
    }

    /* Execute from ROM */
    if (pc < ROM_SIZE) {
        return 0;
    }

    /* Execute from flash */
    if (pc >= FLASH_BASE && pc < FLASH_BASE + FLASH_SIZE) {
        return 0;
    }

    /* Execute from RAM */
    if (pc >= RAM_BASE && pc < RAM_TOP) {
        return 0;
    }

    return 1;
}

/* ========================================================================
 * Single-Core Exception Handling
 * ======================================================================== */

/* Exception nesting stack: tracks active exception chain for proper return */
#define MAX_EXCEPTION_DEPTH 8
static uint32_t exception_stack[MAX_EXCEPTION_DEPTH];
static int exception_depth = 0;

void cpu_exception_entry(uint32_t vector_num) {
    uint32_t vector_offset = vector_num * 4;
    uint32_t handler_addr = mem_read32(cpu.vtor + vector_offset);

    if (cpu.debug_enabled) {
        printf("[CPU] Exception %u: PC=0x%08X VTOR=0x%08X -> Handler=0x%08X (depth %d)\n",
               vector_num, cpu.r[15], cpu.vtor, handler_addr, exception_depth);
    }

    if (vector_num < 32) {
        nvic_state.active_exceptions |= (1u << vector_num);
    }

    /* Push previous exception onto nesting stack */
    if (exception_depth < MAX_EXCEPTION_DEPTH) {
        exception_stack[exception_depth] = cpu.current_irq;
        exception_depth++;
    }

    cpu.current_irq = vector_num;

    if (vector_num >= 16 && (vector_num - 16) < 32) {
        nvic_state.iabr |= (1u << (vector_num - 16));
    }

    uint32_t sp = cpu.r[13];

    sp -= 4; mem_write32(sp, cpu.xpsr);
    sp -= 4; mem_write32(sp, cpu.r[15]);
    sp -= 4; mem_write32(sp, cpu.r[14]);
    sp -= 4; mem_write32(sp, cpu.r[12]);
    sp -= 4; mem_write32(sp, cpu.r[3]);
    sp -= 4; mem_write32(sp, cpu.r[2]);
    sp -= 4; mem_write32(sp, cpu.r[1]);
    sp -= 4; mem_write32(sp, cpu.r[0]);

    cpu.r[13] = sp;

    if (cpu.debug_enabled) {
        printf("[CPU] Context saved, SP now=0x%08X\n", sp);
    }

    cpu.r[15] = handler_addr & ~1u;
    cpu.r[14] = 0xFFFFFFF9;
}

void cpu_exception_return(uint32_t lr_value) {
    uint32_t return_mode = lr_value & 0x0F;

    if (cpu.debug_enabled) {
        printf("[CPU] >>> EXCEPTION RETURN START: LR=0x%08X mode=0x%X SP=0x%08X\n",
               lr_value, return_mode, cpu.r[13]);
    }

    if (return_mode == 0x9 || return_mode == 0x1 || return_mode == 0xD) {
        uint32_t sp = cpu.r[13];

        if (cpu.debug_enabled) {
            printf("[CPU] Popping frame from SP=0x%08X\n", sp);
        }

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

        cpu.r[0] = r0;
        cpu.r[1] = r1;
        cpu.r[2] = r2;
        cpu.r[3] = r3;
        cpu.r[12] = r12;
        cpu.r[13] = sp;
        cpu.r[14] = lr;
        cpu.r[15] = pc & ~1u;
        cpu.xpsr = xpsr;

        if (cpu.debug_enabled) {
            printf("[CPU] RESTORED: PC now=0x%08X SP now=0x%08X\n",
                   cpu.r[15], cpu.r[13]);
        }

        if (cpu.current_irq != 0xFFFFFFFF) {
            uint32_t vector_num = cpu.current_irq;

            if (vector_num >= 16 && (vector_num - 16) < 32) {
                nvic_state.iabr &= ~(1u << (vector_num - 16));
            }

            if (vector_num < 32) {
                nvic_state.active_exceptions &= ~(1u << vector_num);
            }

            /* Pop previous exception from nesting stack */
            if (exception_depth > 0) {
                exception_depth--;
                cpu.current_irq = exception_stack[exception_depth];
            } else {
                cpu.current_irq = 0xFFFFFFFF;
            }

            if (cpu.debug_enabled) {
                printf("[CPU] Cleared active exception (vector %u), IABR=0x%X, depth=%d\n",
                       vector_num, nvic_state.iabr, exception_depth);
            }
        }

        if (cpu.debug_enabled) {
            printf("[CPU] <<< EXCEPTION RETURN COMPLETE\n");
        }

    } else {
        if (cpu.debug_enabled)
            printf("[CPU] ERROR: Unsupported EXC_RETURN mode 0x%X (expected 0x9 or 0x1)\n",
                   return_mode);
    }
}

/* ========================================================================
 * Cycle-Aware Timer/SysTick Ticking
 *
 * Accumulates CPU cycles and converts to microseconds based on clock
 * frequency. At 1 cycles/µs (default), this is 1:1 like before.
 * At 125 cycles/µs (real RP2040), it takes 125 instructions to
 * advance the timer by 1 µs.
 * ======================================================================== */

static void timing_tick(uint32_t cycles) {
    timing_config.cycle_accumulator += cycles;
    uint32_t us = timing_config.cycle_accumulator / timing_config.cycles_per_us;
    if (us > 0) {
        timing_config.cycle_accumulator -= us * timing_config.cycles_per_us;
        timer_tick(us);
        rtc_tick(us);
    }
    /* SysTick counts in CPU cycles, not microseconds */
    systick_tick(cycles);
}

/* ========================================================================
 * Single-Core CPU Execution (Dispatch Table)
 * ======================================================================== */

void cpu_step(void) {
    uint32_t pc = cpu.r[15];

    /* Stop if PC already out of range */
    if (pc == 0xFFFFFFFF) {
        return;
    }

    /* Allow execution from ROM, flash, or RAM */
    if (!(pc < ROM_SIZE ||
          (pc >= FLASH_BASE && pc < FLASH_BASE + FLASH_SIZE) ||
          (pc >= RAM_BASE && pc < RAM_TOP))) {
        /* HardFault: PC out of executable range */
        uint32_t handler = mem_read32(cpu.vtor + EXC_HARDFAULT * 4);
        if (handler && handler != 0xFFFFFFFF) {
            if (cpu.debug_enabled) {
                printf("[CPU] HardFault: PC out of bounds (0x%08X) -> handler 0x%08X\n",
                       pc, handler);
            }
            cpu_exception_entry(EXC_HARDFAULT);
            return;
        }
        if (cpu.debug_enabled)
            printf("[CPU] ERROR: PC out of bounds (0x%08X), no HardFault handler\n", pc);
        cpu.r[15] = 0xFFFFFFFF;
        return;
    }

    /* ROM function interception (float/double/flash) */
    if (pc < ROM_SIZE && rom_intercept(pc)) {
        timing_tick(4);  /* ROM functions take ~4 cycles */
        return;
    }

    uint16_t instr = mem_read16(pc);
    cpu.step_count++;

    if (cpu.debug_enabled) {
        printf("[CPU] Step %3u: PC=0x%08X instr=0x%04X\n", cpu.step_count, pc, instr);
    }

    /* Check for pending interrupts (only if PRIMASK allows) */
    if (!cpu.primask) {
        /* Get active exception priority (0xFF if no active exception) */
        uint8_t active_pri = 0xFF;
        if (cpu.current_irq != 0xFFFFFFFF) {
            active_pri = nvic_get_exception_priority(cpu.current_irq);
        }

        /* Check external IRQs */
        uint32_t pending_irq = nvic_get_pending_irq();
        if (pending_irq != 0xFFFFFFFF) {
            uint8_t pending_pri = nvic_state.priority[pending_irq] & 0xC0;
            /* Only deliver if strictly higher priority (lower number) */
            if (pending_pri < active_pri) {
                if (cpu.debug_enabled) {
                    printf("[CPU] *** INTERRUPT: IRQ %u (pri=0x%02X) preempting (active_pri=0x%02X) ***\n",
                           pending_irq, pending_pri, active_pri);
                }
                nvic_clear_pending(pending_irq);
                cpu_exception_entry(pending_irq + 16);
                timing_tick(1);
                return;
            }
        }

        /* Check SysTick pending */
        if (systick_state.pending) {
            uint8_t systick_pri = nvic_get_exception_priority(EXC_SYSTICK);
            if (systick_pri < active_pri) {
                systick_state.pending = 0;
                cpu_exception_entry(EXC_SYSTICK);
                timing_tick(1);
                return;
            }
        }

        /* Check PendSV pending */
        if (nvic_state.pendsv_pending) {
            uint8_t pendsv_pri = nvic_get_exception_priority(EXC_PENDSV);
            if (pendsv_pri < active_pri) {
                nvic_state.pendsv_pending = 0;
                cpu_exception_entry(EXC_PENDSV);
                timing_tick(1);
                return;
            }
        }
    }

    /* NOP: all-zero halfword */
    if (instr == 0x0000) {
        cpu.r[15] = pc + 2;
        timing_tick(1);
        return;
    }

    /* 32-bit instructions: top 5 bits = 11101/11110/11111 */
    uint8_t top5 = instr >> 11;
    if (top5 >= 0x1D) {  /* 0xE800+ could be 32-bit */
        if ((instr & 0xF800) == 0xF000 || (instr & 0xF800) == 0xF800) {
            uint16_t instr2 = mem_read16(pc + 2);

            /* BL/BLX immediate */
            if ((instr & 0xF800) == 0xF000 && (instr2 & 0xD000) == 0xD000) {
                if (cpu.debug_enabled) {
                    printf("[CPU] BL32 upper=0x%04X lower=0x%04X @ PC=0x%08X\n",
                           instr, instr2, pc);
                }
                pc_updated = 0;
                instr_bl_32(instr, instr2);
                timing_tick(timing_instruction_cycles_32(instr, instr2));
                return;
            }

            /* MSR: upper = 0xF380 | Rn, lower = 0x88xx | SYSm */
            if ((instr & 0xFFF0) == 0xF380 && (instr2 & 0xFF00) == 0x8800) {
                uint8_t rn = instr & 0xF;
                uint8_t sysm = instr2 & 0xFF;
                instr_msr_32(rn, sysm);
                cpu.r[15] = pc + 4;
                timing_tick(timing_instruction_cycles_32(instr, instr2));
                return;
            }

            /* MRS: upper = 0xF3EF, lower = 0x8Rss (Rd in [11:8], SYSm in [7:0]) */
            if ((instr & 0xFFFF) == 0xF3EF && (instr2 & 0xF000) == 0x8000) {
                uint8_t rd = (instr2 >> 8) & 0xF;
                uint8_t sysm = instr2 & 0xFF;
                instr_mrs_32(rd, sysm);
                cpu.r[15] = pc + 4;
                timing_tick(timing_instruction_cycles_32(instr, instr2));
                return;
            }

            /* DSB/DMB/ISB: upper = 0xF3BF, lower = 0x8F4x/8F5x/8F6x */
            if ((instr & 0xFFFF) == 0xF3BF && (instr2 & 0xFF00) == 0x8F00) {
                /* Memory barriers are NOPs in our emulator */
                cpu.r[15] = pc + 4;
                timing_tick(timing_instruction_cycles_32(instr, instr2));
                return;
            }

            /* Unhandled 32-bit instruction -> HardFault */
            if (cpu.debug_enabled)
                printf("[CPU] Unhandled 32-bit Thumb instr: upper=0x%04X lower=0x%04X @ PC=0x%08X\n",
                       instr, instr2, pc);
            cpu_exception_entry(EXC_HARDFAULT);
            return;
        }
    }

    /* 16-bit instruction dispatch via table */
    pc_updated = 0;
    dispatch_table[instr >> 8](instr);

    if (!pc_updated) {
        cpu.r[15] = pc + 2;
    }

    /* Determine if a branch was taken (PC changed to something other than pc+2) */
    int branch_taken = (cpu.r[15] != pc + 2);
    timing_tick(timing_instruction_cycles(instr, branch_taken));
}

/* ========================================================================
 * Dual-Core Initialization
 * ======================================================================== */

cpu_state_dual_t cores[NUM_CORES] = {0};
uint32_t shared_ram[SHARED_RAM_SIZE / 4] = {0};
uint32_t spinlocks[SPINLOCK_SIZE] = {0};
multicore_fifo_t fifo[NUM_CORES] = {0};

int num_active_cores = MAX_CORES;  /* Runtime-configurable, default: all cores */
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

        cores[i].core_id = i;
        cores[i].is_halted = (i == CORE1) ? 1 : 0;
        cores[i].xpsr = 0x01000000;
        cores[i].vtor = 0x10000100;
        cores[i].current_irq = 0xFFFFFFFF;
        cores[i].primask = 0;

        fprintf(stderr, "[CORE%d] Initialized (halted: %d)\n", i, cores[i].is_halted);
    }

    fprintf(stderr, "[Boot] Firmware in shared flash (%u bytes), SRAM shared across both cores (%u bytes)\n",
           FLASH_SIZE, RAM_SIZE);

    /* Read vector table from flash */
    uint32_t vector_table = FLASH_BASE + 0x100;
    uint32_t initial_sp = mem_read32(vector_table);
    uint32_t reset_vector = mem_read32(vector_table + 4);

    cores[CORE0].r[13] = initial_sp;
    cores[CORE0].r[15] = reset_vector & ~1;

    if (initial_sp != 0 || reset_vector != 0) {
        fprintf(stderr, "[Boot] Vector table loaded: SP=0x%08X, PC=0x%08X\n",
               initial_sp, reset_vector & ~1);
    }

    /* Initialize FIFO channels */
    for (int i = 0; i < NUM_CORES; i++) {
        fifo[i].count = 0;
        fifo[i].read_ptr = 0;
        fifo[i].write_ptr = 0;
    }

    /* Initialize dispatch table if needed */
    if (!dispatch_initialized) {
        init_dispatch_table();
        dispatch_initialized = 1;
    }
}

/* ========================================================================
 * Dual-Core CPU Execution (Zero-Copy Context Switch)
 * ======================================================================== */

/*
 * Context-switch a specific core into the single-core engine.
 *
 * Register state is per-core, but RP2040 SRAM is shared across both cores.
 * Keep the active memory bus pointed at the full shared SRAM image while
 * swapping only the lightweight CPU register state.
 */
static void cpu_step_core_via_single(int core_id) {
    if (core_id >= NUM_CORES) return;
    if (cores[core_id].is_halted) return;

    /* 1. Save current register state */
    uint32_t saved_r[16];
    memcpy(saved_r, cpu.r, sizeof(saved_r));
    uint32_t saved_xpsr = cpu.xpsr;
    uint32_t saved_vtor = cpu.vtor;
    uint32_t saved_step = cpu.step_count;
    int saved_debug = cpu.debug_enabled;
    int saved_debug_asm = cpu.debug_asm;
    uint32_t saved_irq = cpu.current_irq;
    uint32_t saved_primask = cpu.primask;
    uint32_t saved_control = cpu.control;

    /* 2. Load core registers into global cpu */
    memcpy(cpu.r, cores[core_id].r, sizeof(cpu.r));
    cpu.xpsr         = cores[core_id].xpsr;
    cpu.vtor         = cores[core_id].vtor;
    cpu.step_count   = cores[core_id].step_count;
    cpu.debug_enabled= cores[core_id].debug_enabled;
    cpu.debug_asm    = cores[core_id].debug_asm;
    cpu.current_irq  = cores[core_id].current_irq;
    cpu.primask      = cores[core_id].primask;
    cpu.control      = cores[core_id].control;

    /* 3. Point the memory bus at shared SRAM for all cores */
    mem_set_ram_ptr(cpu.ram, RAM_BASE, RAM_SIZE);

    /* 4. Set active core for SIO routing */
    set_active_core(core_id);

    /* 5. Execute ONE instruction */
    cpu_step();

    /* 6. Save updated state back to core */
    memcpy(cores[core_id].r, cpu.r, sizeof(cpu.r));
    cores[core_id].xpsr         = cpu.xpsr;
    cores[core_id].vtor         = cpu.vtor;
    cores[core_id].step_count   = cpu.step_count;
    cores[core_id].current_irq  = cpu.current_irq;
    cores[core_id].primask      = cpu.primask;
    cores[core_id].control      = cpu.control;

    if (cpu.r[15] == 0xFFFFFFFF) {
        cores[core_id].is_halted = 1;
    }

    /* 7. Restore saved register state */
    memcpy(cpu.r, saved_r, sizeof(saved_r));
    cpu.xpsr = saved_xpsr;
    cpu.vtor = saved_vtor;
    cpu.step_count = saved_step;
    cpu.debug_enabled = saved_debug;
    cpu.debug_asm = saved_debug_asm;
    cpu.current_irq = saved_irq;
    cpu.primask = saved_primask;
    cpu.control = saved_control;

    /* 8. Restore memory bus to default */
    mem_set_ram_ptr(cpu.ram, RAM_BASE, RAM_SIZE);
}

void cpu_step_core(int core_id) {
    cpu_step_core_via_single(core_id);
}

void dual_core_step(void) {
    static int current = 0;

    for (int i = 0; i < num_active_cores; i++) {
        int c = current % num_active_cores;
        current = (current + 1) % num_active_cores;

        /* Skip cores sleeping in WFI/WFE — wake when interrupt pending */
        if (cores[c].is_wfi) {
            /* Check for pending interrupt that would wake this core */
            int saved_core = get_active_core();
            set_active_core(c);
            uint32_t pending = nvic_get_pending_irq();
            int wake = (pending != 0xFFFFFFFF) ||
                       systick_state.pending ||
                       nvic_state.pendsv_pending;
            set_active_core(saved_core);
            if (!wake) continue;
            cores[c].is_wfi = 0;  /* Wake up */
        }

        cpu_step_core(c);
    }
}

int cpu_is_halted_core(int core_id) {
    if (core_id >= NUM_CORES) return 1;
    return cores[core_id].is_halted;
}

/* ========================================================================
 * Boot2 Detection
 *
 * RP2040 flash layout: first 256 bytes (offset 0x000-0x0FF) are the
 * second-stage bootloader (boot2).  The application vector table lives
 * at offset 0x100.  If boot2 is present we start execution from
 * 0x10000000 so the real boot2 code configures XIP/SSI before jumping
 * to the application.  Otherwise we skip straight to the app.
 *
 * Detection: boot2 is present when the first flash word is non-trivial
 * (not 0x00000000 or 0xFFFFFFFF) and the vector table at +0x100 also
 * contains a plausible initial SP (in RAM range).
 * ======================================================================== */

static int boot2_detected = 0;

int cpu_has_boot2(void) {
    uint32_t first_word;
    memcpy(&first_word, &cpu.flash[0], 4);

    if (first_word == 0x00000000 || first_word == 0xFFFFFFFF) {
        return 0;
    }

    /* Also verify the vector table at +0x100 looks valid */
    uint32_t sp_word;
    memcpy(&sp_word, &cpu.flash[0x100], 4);
    if (sp_word < RAM_BASE || sp_word > RAM_BASE + RAM_SIZE) {
        return 0;
    }

    return 1;
}

void cpu_reset_core(int core_id) {
    if (core_id >= NUM_CORES) return;

    cpu_state_dual_t *c = &cores[core_id];
    set_active_core(core_id);

    memset(c->r, 0, sizeof(c->r));
    c->xpsr = 0x01000000;
    c->step_count = 0;
    c->is_halted = (core_id == CORE1) ? 1 : 0;
    c->vtor = 0x10000100;
    c->primask = 0;

    if (core_id == CORE0) {
        if (boot2_detected) {
            /* Boot2 present: start from flash base, boot2 sets up XIP
             * and eventually jumps to the application at +0x100.
             * Use top of SRAM as initial SP (boot2 sets its own). */
            c->r[13] = RAM_BASE + RAM_SIZE;
            c->r[15] = FLASH_BASE;
            fprintf(stderr, "[Boot2] Starting boot2 from 0x%08X\n", FLASH_BASE);
        } else {
            /* No boot2: read vector table directly from +0x100 */
            uint32_t vector_table = FLASH_BASE + 0x100;
            uint32_t initial_sp = mem_read32(vector_table);
            uint32_t reset_vector = mem_read32(vector_table + 4);

            c->r[13] = initial_sp;
            c->r[15] = reset_vector & ~1;
        }

        if (c->debug_enabled) {
            printf("[CORE%d] Reset to PC=0x%08X SP=0x%08X\n",
                   core_id, c->r[15], c->r[13]);
        }
    }
}

void cpu_set_boot2(int enable) {
    boot2_detected = enable;
}

/* ========================================================================
 * Dual-Core Exception Handling
 * ======================================================================== */

void cpu_exception_entry_dual(int core_id, uint32_t vector_num) {
    if (core_id >= NUM_CORES) return;

    cpu_state_dual_t *c = &cores[core_id];
    set_active_core(core_id);

    uint32_t vector_offset = vector_num * 4;
    uint32_t handler_addr = mem_read32_dual(core_id, c->vtor + vector_offset);

    if (c->debug_enabled) {
        printf("[CORE%d] Exception %u -> Handler 0x%08X\n",
               core_id, vector_num, handler_addr);
    }

    uint32_t sp = c->r[13];

    sp -= 4; mem_write32_dual(core_id, sp, c->xpsr);
    sp -= 4; mem_write32_dual(core_id, sp, c->r[15]);
    sp -= 4; mem_write32_dual(core_id, sp, c->r[14]);
    sp -= 4; mem_write32_dual(core_id, sp, c->r[12]);
    sp -= 4; mem_write32_dual(core_id, sp, c->r[3]);
    sp -= 4; mem_write32_dual(core_id, sp, c->r[2]);
    sp -= 4; mem_write32_dual(core_id, sp, c->r[1]);
    sp -= 4; mem_write32_dual(core_id, sp, c->r[0]);

    c->r[13] = sp;
    c->r[15] = handler_addr & ~1u;
    c->r[14] = 0xFFFFFFF9;
    c->in_handler_mode = 1;
}

void cpu_exception_return_dual(int core_id, uint32_t lr_value) {
    (void)lr_value;

    if (core_id >= NUM_CORES) return;

    cpu_state_dual_t *c = &cores[core_id];
    set_active_core(core_id);

    uint32_t sp = c->r[13];

    uint32_t r0 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t r1 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t r2 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t r3 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t r12 = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t lr = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t pc = mem_read32_dual(core_id, sp); sp += 4;
    uint32_t xpsr = mem_read32_dual(core_id, sp); sp += 4;

    c->r[0] = r0;
    c->r[1] = r1;
    c->r[2] = r2;
    c->r[3] = r3;
    c->r[12] = r12;
    c->r[13] = sp;
    c->r[14] = lr;
    c->r[15] = pc & ~1u;
    c->xpsr = xpsr;
    c->in_handler_mode = 0;

    if (c->debug_enabled) {
        printf("[CORE%d] Exception return to PC=0x%08X\n", core_id, c->r[15]);
    }
}

int any_core_running(void) {
    for (int i = 0; i < num_active_cores; i++) {
        if (!cores[i].is_halted) {
            return 1;  /* WFI cores still count as running */
        }
    }
    return 0;
}

void dual_core_status(void) {
    fprintf(stderr, "[DUAL-CORE STATUS]\n");
    for (int i = 0; i < NUM_CORES; i++) {
        fprintf(stderr, "[CORE%d] Status: %s\n", i, cores[i].is_halted ? "HALTED" : "RUNNING");
        fprintf(stderr, "[CORE%d] PC=0x%08X SP=0x%08X\n", i, cores[i].r[15], cores[i].r[13]);
        fprintf(stderr, "[CORE%d] Step count: %u\n", i, cores[i].step_count);
    }
}

void cpu_set_debug_core(int core_id, int enabled) {
    if (core_id >= NUM_CORES) return;
    cores[core_id].debug_enabled = enabled;
}

/* ========================================================================
 * SIO (Single-Cycle I/O) Operations
 * ======================================================================== */

typedef struct {
    int waiting_for_launch;
    uint32_t launch_words[6];
    uint32_t launch_count;
} core1_bootrom_state_t;

static core1_bootrom_state_t core1_bootrom = {0};

uint32_t sio_get_core_id(void) {
    return get_active_core();
}

void sio_set_core1_reset(int assert_reset) {
    if (assert_reset) {
        cores[CORE1].is_halted = 1;
        core1_bootrom.waiting_for_launch = 0;
        core1_bootrom.launch_count = 0;
        memset(&fifo[CORE1], 0, sizeof(fifo[CORE1]));
    } else {
        cores[CORE1].is_halted = 1;
        core1_bootrom.waiting_for_launch = 1;
        core1_bootrom.launch_count = 0;
        memset(&fifo[CORE1], 0, sizeof(fifo[CORE1]));
        fifo_try_push(CORE0, 0);
    }
}

void sio_set_core1_stall(int stall) {
    (void)stall;
}

int sio_core1_bootrom_handle_fifo_write(uint32_t val) {
    if (!core1_bootrom.waiting_for_launch) {
        return 0;
    }

    fifo_try_push(CORE0, val);

    if (core1_bootrom.launch_count < 6) {
        core1_bootrom.launch_words[core1_bootrom.launch_count++] = val;
    }

    if (core1_bootrom.launch_count == 6) {
        memset(cores[CORE1].r, 0, sizeof(cores[CORE1].r));
        cores[CORE1].vtor = core1_bootrom.launch_words[3];
        cores[CORE1].r[13] = core1_bootrom.launch_words[4];
        cores[CORE1].r[15] = core1_bootrom.launch_words[5] & ~1u;
        cores[CORE1].xpsr = 0x01000000;
        cores[CORE1].current_irq = 0xFFFFFFFF;
        cores[CORE1].primask = 0;
        cores[CORE1].control = 0;
        cores[CORE1].is_halted = 0;
        core1_bootrom.waiting_for_launch = 0;
        core1_bootrom.launch_count = 0;
        corepool_wake_cores();  /* Wake Core 1 thread */
    }

    return 1;
}

/* ========================================================================
 * Spinlock Operations
 * ======================================================================== */

uint32_t spinlock_acquire(uint32_t lock_num) {
    if (lock_num >= SPINLOCK_SIZE) return 0;

    if (spinlocks[lock_num] & SPINLOCK_LOCKED) {
        return 0;
    }

    spinlocks[lock_num] = SPINLOCK_VALID | SPINLOCK_LOCKED;
    return 1u << lock_num;
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

    if (fifo[core_id].count == 0) {
        if (cpu.debug_enabled)
            printf("[FIFO] WARNING: Pop on empty FIFO for core %d\n", core_id);
        return 0;
    }

    uint32_t val = fifo[core_id].messages[fifo[core_id].read_ptr];
    fifo[core_id].read_ptr = (fifo[core_id].read_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count--;

    return val;
}

void fifo_push(int core_id, uint32_t val) {
    if (core_id >= NUM_CORES) return;

    if (fifo[core_id].count >= FIFO_DEPTH) {
        if (cpu.debug_enabled)
            printf("[FIFO] WARNING: Push on full FIFO for core %d, dropping\n", core_id);
        return;
    }

    fifo[core_id].messages[fifo[core_id].write_ptr] = val;
    fifo[core_id].write_ptr = (fifo[core_id].write_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count++;
}

int fifo_try_pop(int core_id, uint32_t *val) {
    if (core_id >= NUM_CORES) return 0;

    if (fifo[core_id].count == 0) {
        return 0;
    }

    *val = fifo[core_id].messages[fifo[core_id].read_ptr];
    fifo[core_id].read_ptr = (fifo[core_id].read_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count--;

    return 1;
}

int fifo_try_push(int core_id, uint32_t val) {
    if (core_id >= NUM_CORES) return 0;

    if (fifo[core_id].count >= FIFO_DEPTH) {
        return 0;
    }

    fifo[core_id].messages[fifo[core_id].write_ptr] = val;
    fifo[core_id].write_ptr = (fifo[core_id].write_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count++;

    /* Signal SIO IRQ for the receiving core */
    nvic_signal_irq(core_id == CORE0 ? IRQ_SIO_IRQ_PROC0 : IRQ_SIO_IRQ_PROC1);

    return 1;
}
