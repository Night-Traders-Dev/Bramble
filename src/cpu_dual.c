/*
 * Dual-Core CPU Execution Engine (FIXED)
 * 
 * Fixes:
 * - Use %u for uint32_t format specifiers (not %lu)
 * - Remove unused parameter warnings
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "emulator_dual.h"
#include "instructions.h"
#include "timer.h"
#include "nvic.h"

void cpu_step_core(int core_id) {
    if (core_id >= NUM_CORES) return;
    if (cores[core_id].is_halted) return;
    
    cpu_state_dual_t *cpu = &cores[core_id];
    set_active_core(core_id);
    
    uint32_t pc = cpu->r[15];
    
    if (pc < FLASH_BASE || pc >= FLASH_BASE + FLASH_SIZE) {
        printf("[CORE%d] PC out of bounds: 0x%08X\n", core_id, pc);
        cpu->is_halted = 1;
        return;
    }
    
    uint32_t flash_offset = pc - FLASH_BASE;
    uint16_t instr = (cores[0].flash[flash_offset] |
                      (cores[0].flash[flash_offset + 1] << 8));
    
    cpu->step_count++;
    
    if (cpu->debug_enabled) {
        printf("[CORE%d] Step %u: PC=0x%08X instr=0x%04X\n",
               core_id, cpu->step_count, pc, instr);
    }
    
    uint32_t pending_irq = nvic_get_pending_irq();
    if (pending_irq != 0xFFFFFFFF) {
        uint32_t vector_num = pending_irq + 16;
        
        if (cpu->debug_enabled) {
            printf("[CORE%d] INTERRUPT: IRQ %u\n", core_id, pending_irq);
        }
        
        nvic_clear_pending(pending_irq);
        cpu_exception_entry_dual(core_id, vector_num);
        timer_tick(1);
        return;
    }
    
    if (instr == 0x0000) {
        cpu->r[15] = pc + 2;
        timer_tick(1);
        return;
    }
    
    if ((instr & 0xFF00) == 0xBE00) {
        printf("[CORE%d] BKPT hit\n", core_id);
        cpu->is_halted = 1;
        timer_tick(1);
        return;
    }
    
    if ((instr & 0xFF87) == 0x4700) {
        uint8_t rm = (instr >> 3) & 0x0F;
        cpu->r[15] = (cpu->r[rm] & ~1u);
        timer_tick(1);
        return;
    }
    
    if ((instr & 0xF800) == 0x2000) {
        uint8_t rd = (instr >> 8) & 0x07;
        uint8_t imm8 = instr & 0xFF;
        cpu->r[rd] = imm8;
        cpu->r[15] = pc + 2;
        timer_tick(1);
        return;
    }
    
    if ((instr & 0xF800) == 0x6800) {
        uint8_t rd = instr & 0x07;
        uint8_t rn = (instr >> 3) & 0x07;
        uint8_t imm5 = ((instr >> 6) & 0x1F) << 2;
        
        uint32_t addr = cpu->r[rn] + imm5;
        cpu->r[rd] = mem_read32_dual(core_id, addr);
        cpu->r[15] = pc + 2;
        timer_tick(1);
        return;
    }
    
    if ((instr & 0xF800) == 0x6000) {
        uint8_t rd = instr & 0x07;
        uint8_t rn = (instr >> 3) & 0x07;
        uint8_t imm5 = ((instr >> 6) & 0x1F) << 2;
        
        uint32_t addr = cpu->r[rn] + imm5;
        mem_write32_dual(core_id, addr, cpu->r[rd]);
        cpu->r[15] = pc + 2;
        timer_tick(1);
        return;
    }
    
    if ((instr & 0xFF00) == 0x4600) {
        uint8_t rd = ((instr >> 4) & 0x08) | (instr & 0x07);
        uint8_t rm = ((instr >> 3) & 0x0F);
        cpu->r[rd] = cpu->r[rm];
        cpu->r[15] = pc + 2;
        timer_tick(1);
        return;
    }
    
    printf("[CORE%d] Unimplemented instruction: 0x%04X at 0x%08X\n",
           core_id, instr, pc);
    cpu->is_halted = 1;
    timer_tick(1);
}

void dual_core_step(void) {
    static int current = 0;
    
    for (int i = 0; i < NUM_CORES; i++) {
        cpu_step_core(current);
        current = (current + 1) % NUM_CORES;
    }
}

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
    (void)lr_value;  /* Mark as intentionally unused */
    
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
