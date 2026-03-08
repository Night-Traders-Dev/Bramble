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

/* ========================================================================
 * Single-Core Global State
 * ======================================================================== */

cpu_state_t cpu = {0};
int pc_updated = 0;

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
        case 2: instr_cmp_reg_reg(instr); break;
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
    dispatch_table[0x46] = instr_mov_reg;
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
        printf("[Boot] ERROR: Invalid SP 0x%08X (not in RAM 0x%08X-0x%08X)\n",
               initial_sp, RAM_BASE, RAM_BASE + RAM_SIZE);
        cpu.r[15] = 0xFFFFFFFF;
        return;
    }

    if ((reset_vector & 0x1) == 0) {
        printf("[Boot] ERROR: Invalid reset vector 0x%08X (Thumb bit not set)\n",
               reset_vector);
        cpu.r[15] = 0xFFFFFFFF;
        return;
    }

    cpu.r[13] = initial_sp;
    cpu.r[15] = reset_vector & ~1u;
    cpu.r[14] = 0xFFFFFFFF;
    cpu.xpsr = 0x01000000;

    printf("[Boot] Reset complete:\n");
    printf("[Boot] VTOR = 0x%08X\n", cpu.vtor);
    printf("[Boot] SP = 0x%08X\n", cpu.r[13]);
    printf("[Boot] PC = 0x%08X\n", cpu.r[15]);
    printf("[Boot] xPSR = 0x%08X\n", cpu.xpsr);
}

/* Allow execution from both flash and RAM */
int cpu_is_halted(void) {
    uint32_t pc = cpu.r[15];

    if (pc == 0xFFFFFFFF) {
        return 1;
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

void cpu_exception_entry(uint32_t vector_num) {
    uint32_t vector_offset = vector_num * 4;
    uint32_t handler_addr = mem_read32(cpu.vtor + vector_offset);

    if (cpu.debug_enabled) {
        printf("[CPU] Exception %u: PC=0x%08X VTOR=0x%08X -> Handler=0x%08X\n",
               vector_num, cpu.r[15], cpu.vtor, handler_addr);
    }

    if (vector_num < 32) {
        nvic_state.active_exceptions |= (1u << vector_num);
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

    if (return_mode == 0x9 || return_mode == 0x1) {
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

            if (cpu.debug_enabled) {
                printf("[CPU] Cleared active exception (vector %u), IABR=0x%X\n",
                       vector_num, nvic_state.iabr);
            }

            cpu.current_irq = 0xFFFFFFFF;
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
 * Single-Core CPU Execution (Dispatch Table)
 * ======================================================================== */

void cpu_step(void) {
    uint32_t pc = cpu.r[15];

    /* Stop if PC already out of range */
    if (pc == 0xFFFFFFFF) {
        return;
    }

    /* Allow execution from flash or RAM */
    if (!((pc >= FLASH_BASE && pc < FLASH_BASE + FLASH_SIZE) ||
          (pc >= RAM_BASE && pc < RAM_TOP))) {
        printf("[CPU] ERROR: PC out of bounds (0x%08X)\n", pc);
        cpu.r[15] = 0xFFFFFFFF;
        return;
    }

    uint16_t instr = mem_read16(pc);
    cpu.step_count++;

    if (cpu.debug_enabled) {
        printf("[CPU] Step %3u: PC=0x%08X instr=0x%04X\n", cpu.step_count, pc, instr);
    }

    timer_tick(1);

    /* Check for pending interrupts (only if PRIMASK allows) */
    if (!cpu.primask) {
        uint32_t pending_irq = nvic_get_pending_irq();

        if (pending_irq != 0xFFFFFFFF) {
            uint32_t vector_num = pending_irq + 16;

            if (cpu.debug_enabled) {
                printf("[CPU] *** INTERRUPT: IRQ %u detected ***\n", pending_irq);
            }

            nvic_clear_pending(pending_irq);
            cpu_exception_entry(vector_num);
            return;
        }
    }

    /* NOP: all-zero halfword */
    if (instr == 0x0000) {
        cpu.r[15] = pc + 2;
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
                /* pc_updated is set inside instr_bl_32 */
                return;
            }

            /* Unhandled 32-bit instruction */
            printf("[CPU] Unhandled 32-bit Thumb instr: upper=0x%04X lower=0x%04X @ PC=0x%08X\n",
                   instr, instr2, pc);
            cpu.r[15] = 0xFFFFFFFF;
            return;
        }
    }

    /* 16-bit instruction dispatch via table */
    pc_updated = 0;
    dispatch_table[instr >> 8](instr);

    if (!pc_updated) {
        cpu.r[15] = pc + 2;
    }
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

        cores[i].core_id = i;
        cores[i].is_halted = (i == CORE1) ? 1 : 0;
        cores[i].xpsr = 0x01000000;
        cores[i].vtor = 0x10000100;
        cores[i].current_irq = 0xFFFFFFFF;
        cores[i].primask = 0;

        printf("[CORE%d] Initialized (halted: %d)\n", i, cores[i].is_halted);
    }

    /* Copy RAM from single-core to core 0 (flash is shared via cpu.flash) */
    memcpy(cores[CORE0].ram, cpu.ram, CORE_RAM_SIZE);

    printf("[Boot] Firmware in shared flash (%u bytes), RAM copied to CORE0 (%u bytes)\n",
           FLASH_SIZE, CORE_RAM_SIZE);

    /* Read vector table from flash */
    uint32_t vector_table = FLASH_BASE + 0x100;
    uint32_t initial_sp = mem_read32(vector_table);
    uint32_t reset_vector = mem_read32(vector_table + 4);

    cores[CORE0].r[13] = initial_sp;
    cores[CORE0].r[15] = reset_vector & ~1;

    if (initial_sp != 0 || reset_vector != 0) {
        printf("[Boot] Vector table loaded: SP=0x%08X, PC=0x%08X\n",
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
 * Zero-copy approach: Instead of copying 132KB RAM, we just redirect
 * the memory bus pointer to the core's RAM array. Only lightweight
 * register state (64 bytes) is swapped.
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

    /* 2. Load core registers into global cpu */
    memcpy(cpu.r, cores[core_id].r, sizeof(cpu.r));
    cpu.xpsr         = cores[core_id].xpsr;
    cpu.vtor         = cores[core_id].vtor;
    cpu.step_count   = cores[core_id].step_count;
    cpu.debug_enabled= cores[core_id].debug_enabled;
    cpu.debug_asm    = cores[core_id].debug_asm;
    cpu.current_irq  = cores[core_id].current_irq;
    cpu.primask      = cores[core_id].primask;

    /* 3. Redirect memory bus to this core's RAM (zero-copy!) */
    uint32_t ram_base = (core_id == CORE0) ? CORE0_RAM_START : CORE1_RAM_START;
    mem_set_ram_ptr(cores[core_id].ram, ram_base, CORE_RAM_SIZE);

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

    /* 8. Restore memory bus to default */
    mem_set_ram_ptr(cpu.ram, RAM_BASE, RAM_SIZE);
}

void cpu_step_core(int core_id) {
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

    cpu_state_dual_t *c = &cores[core_id];
    set_active_core(core_id);

    memset(c->r, 0, sizeof(c->r));
    c->xpsr = 0x01000000;
    c->step_count = 0;
    c->is_halted = (core_id == CORE1) ? 1 : 0;
    c->vtor = 0x10000100;
    c->primask = 0;

    if (core_id == CORE0) {
        uint32_t vector_table = FLASH_BASE + 0x100;
        uint32_t initial_sp = mem_read32(vector_table);
        uint32_t reset_vector = mem_read32(vector_table + 4);

        c->r[13] = initial_sp;
        c->r[15] = reset_vector & ~1;

        if (c->debug_enabled) {
            printf("[CORE%d] Reset to PC=0x%08X SP=0x%08X\n",
                   core_id, c->r[15], c->r[13]);
        }
    }
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
    for (int i = 0; i < NUM_CORES; i++) {
        if (!cores[i].is_halted) {
            return 1;
        }
    }
    return 0;
}

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
    (void)stall;
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

    if (fifo[core_id].count == 0) {
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

    return 1;
}
