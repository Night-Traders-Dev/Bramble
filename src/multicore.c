/*
 * Multicore Support - FIFO and Spinlock Implementation (FIXED)
 * 
 * Fixes:
 * - Correct shared_ram size (64 KB to match header)
 * - Fix format specifiers (%u instead of %lu for uint32_t)
 * - Proper memory initialization
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "emulator_dual.h"

/* ========================================================================
 * Global Dual-Core State
 * ======================================================================== */

cpu_state_dual_t cores[NUM_CORES] = {0};
uint32_t shared_ram[SHARED_RAM_SIZE / 4] = {0};  /* 64 KB shared RAM */
uint32_t spinlocks[SPINLOCK_SIZE] = {0};

/* Multicore FIFOs: Each core has an outgoing FIFO */
multicore_fifo_t fifo[NUM_CORES] = {0};

uint64_t timer_tick_counter = 0;

/* Track which core is currently executing (for mem_* functions) */
static int active_core = CORE0;

/* ========================================================================
 * Spinlock Implementation (RP2040 Hardware Accurate)
 * ======================================================================== */

uint32_t spinlock_acquire(uint32_t lock_num) {
    if (lock_num >= SPINLOCK_SIZE) {
        return 0;  /* Invalid lock number */
    }
    
    /* Keep trying until we acquire */
    while ((spinlocks[lock_num] & SPINLOCK_LOCKED) == 0) {
        spinlocks[lock_num] |= SPINLOCK_LOCKED;
    }
    
    return 1;  /* Acquired */
}

void spinlock_release(uint32_t lock_num) {
    if (lock_num < SPINLOCK_SIZE) {
        spinlocks[lock_num] &= ~SPINLOCK_LOCKED;
    }
}

/* ========================================================================
 * Multicore FIFO Implementation
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
    int other_core = (core_id == CORE0) ? CORE1 : CORE0;
    
    /* Wait until other core's FIFO has data */
    while (fifo[other_core].count == 0) {
        /* Spin - in real hardware this would WFI */
    }
    
    /* Read from other core's FIFO */
    uint32_t val = fifo[other_core].messages[fifo[other_core].read_ptr];
    fifo[other_core].read_ptr = (fifo[other_core].read_ptr + 1) % FIFO_DEPTH;
    fifo[other_core].count--;
    
    return val;
}

void fifo_push(int core_id, uint32_t val) {
    /* Wait until my FIFO has space */
    while (fifo[core_id].count >= FIFO_DEPTH) {
        /* Spin - in real hardware this would WFI */
    }
    
    /* Write to my FIFO */
    fifo[core_id].messages[fifo[core_id].write_ptr] = val;
    fifo[core_id].write_ptr = (fifo[core_id].write_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count++;
}

int fifo_try_pop(int core_id, uint32_t *val) {
    int other_core = (core_id == CORE0) ? CORE1 : CORE0;
    
    if (fifo[other_core].count == 0) {
        return 0;  /* FIFO empty */
    }
    
    *val = fifo[other_core].messages[fifo[other_core].read_ptr];
    fifo[other_core].read_ptr = (fifo[other_core].read_ptr + 1) % FIFO_DEPTH;
    fifo[other_core].count--;
    
    return 1;  /* Success */
}

int fifo_try_push(int core_id, uint32_t val) {
    if (fifo[core_id].count >= FIFO_DEPTH) {
        return 0;  /* FIFO full */
    }
    
    fifo[core_id].messages[fifo[core_id].write_ptr] = val;
    fifo[core_id].write_ptr = (fifo[core_id].write_ptr + 1) % FIFO_DEPTH;
    fifo[core_id].count++;
    
    return 1;  /* Success */
}

/* ========================================================================
 * Per-Core Memory Management
 * ======================================================================== */

int get_active_core(void) {
    return active_core;
}

void set_active_core(int core_id) {
    if (core_id < NUM_CORES) {
        active_core = core_id;
    }
}

uint32_t mem_read32_dual(int core_id, uint32_t addr) {
    /* Flash is shared across all cores */
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        uint32_t offset = addr - FLASH_BASE;
        uint32_t val = 0;
        memcpy(&val, &cores[0].flash[offset], 4);
        return val;
    }
    
    /* Per-core RAM regions */
    if (core_id == CORE0 && addr >= CORE0_RAM_START && addr < CORE0_RAM_END) {
        uint32_t offset = addr - CORE0_RAM_START;
        uint32_t val = 0;
        memcpy(&val, &cores[CORE0].ram[offset], 4);
        return val;
    }
    
    if (core_id == CORE1 && addr >= CORE1_RAM_START && addr < CORE1_RAM_END) {
        uint32_t offset = addr - CORE1_RAM_START;
        uint32_t val = 0;
        memcpy(&val, &cores[CORE1].ram[offset], 4);
        return val;
    }
    
    /* Shared RAM (accessible by both cores) */
    if (addr >= SHARED_RAM_BASE && addr < SHARED_RAM_BASE + SHARED_RAM_SIZE) {
        uint32_t offset = (addr - SHARED_RAM_BASE) / 4;
        if (offset < (SHARED_RAM_SIZE / 4)) {
            return shared_ram[offset];
        }
    }
    
    return 0;  /* Out of bounds */
}

void mem_write32_dual(int core_id, uint32_t addr, uint32_t val) {
    if (addr >= FLASH_BASE && addr < FLASH_BASE + FLASH_SIZE) {
        printf("[MEM] WARNING: Attempt to write to flash at 0x%08X (read-only)\n", addr);
        return;
    }
    
    if (core_id == CORE0 && addr >= CORE0_RAM_START && addr < CORE0_RAM_END) {
        uint32_t offset = addr - CORE0_RAM_START;
        memcpy(&cores[CORE0].ram[offset], &val, 4);
        return;
    }
    
    if (core_id == CORE1 && addr >= CORE1_RAM_START && addr < CORE1_RAM_END) {
        uint32_t offset = addr - CORE1_RAM_START;
        memcpy(&cores[CORE1].ram[offset], &val, 4);
        return;
    }
    
    if (addr >= SHARED_RAM_BASE && addr < SHARED_RAM_BASE + SHARED_RAM_SIZE) {
        uint32_t offset = (addr - SHARED_RAM_BASE) / 4;
        if (offset < (SHARED_RAM_SIZE / 4)) {
            shared_ram[offset] = val;
        }
        return;
    }
}

uint16_t mem_read16_dual(int core_id, uint32_t addr) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    if (addr & 2) {
        return (word >> 16) & 0xFFFF;
    } else {
        return word & 0xFFFF;
    }
}

void mem_write16_dual(int core_id, uint32_t addr, uint16_t val) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    if (addr & 2) {
        word = (word & 0xFFFF) | ((uint32_t)val << 16);
    } else {
        word = (word & 0xFFFF0000) | val;
    }
    mem_write32_dual(core_id, addr & ~3, word);
}

uint8_t mem_read8_dual(int core_id, uint32_t addr) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    return (word >> ((addr & 3) * 8)) & 0xFF;
}

void mem_write8_dual(int core_id, uint32_t addr, uint8_t val) {
    uint32_t word = mem_read32_dual(core_id, addr & ~3);
    uint32_t shift = (addr & 3) * 8;
    word = (word & ~(0xFF << shift)) | ((uint32_t)val << shift);
    mem_write32_dual(core_id, addr & ~3, word);
}

/* ========================================================================
 * SIO (Single-Cycle I/O) - Core Identification
 * ======================================================================== */

uint32_t sio_get_core_id(void) {
    return active_core;
}

void sio_set_core1_reset(int assert_reset) {
    cores[CORE1].is_halted = assert_reset ? 1 : 0;
    if (!assert_reset && cores[CORE1].is_halted) {
        printf("[CORE1] Released from reset\n");
    } else if (assert_reset) {
        printf("[CORE1] Held in reset\n");
    }
}

void sio_set_core1_stall(int stall) {
    if (stall) {
        printf("[CORE1] Stalled\n");
    } else {
        printf("[CORE1] Unstalled\n");
    }
}

/* ========================================================================
 * Dual-Core CPU Initialization
 * ======================================================================== */

void dual_core_init(void) {
// Initialize core structures
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


    uint32_t vector_table = FLASH_BASE + 0x100;
    uint32_t initial_sp = mem_read32(vector_table);
    uint32_t reset_vector = mem_read32(vector_table + 4);

    // Set Core 0 registers from vector table
    cores[CORE0].r[13] = initial_sp;           // SP (R13)
    cores[CORE0].r[15] = reset_vector & ~1;    // PC (R15), clear Thumb bit

    if (initial_sp != 0 || reset_vector != 0) {
        printf("[Boot] Vector table loaded: SP=0x%08X, PC=0x%08X\n", 
               initial_sp, reset_vector & ~1);
    }

    // Core 1 stays in reset (no vector table init needed)

    // Initialize FIFO channels
    for (int i = 0; i < NUM_CORES; i++) {
        fifo[i].count = 0;
        fifo[i].read_ptr = 0;
        fifo[i].write_ptr = 0;
    }

}

void cpu_reset_core(int core_id) {
    if (core_id >= NUM_CORES) return;
    
    uint32_t vtor = cores[core_id].vtor;
    uint32_t sp = mem_read32_dual(core_id, vtor + 0x00);
    uint32_t pc = mem_read32_dual(core_id, vtor + 0x04);
    
    cores[core_id].r[13] = sp;
    cores[core_id].r[15] = pc & ~1;
    cores[core_id].is_halted = 0;
    cores[core_id].step_count = 0;
    
    printf("[CORE%d] Reset: SP=0x%08X PC=0x%08X\n", core_id, sp, pc);
}

int cpu_is_halted_core(int core_id) {
    if (core_id >= NUM_CORES) return 1;
    return cores[core_id].is_halted;
}

void dual_core_status(void) {
    printf("\n=== Dual-Core Status ===\n");
    for (int i = 0; i < NUM_CORES; i++) {
        printf("Core %d:\n", i);
        printf("  Halted: %d\n", cores[i].is_halted);
        printf("  PC: 0x%08X\n", cores[i].r[15]);
        printf("  SP: 0x%08X\n", cores[i].r[13]);
        printf("  Step Count: %u\n", cores[i].step_count);
        printf("  FIFO Out: %u/%u\n", fifo[i].count, FIFO_DEPTH);
    }
    printf("\n");
}

void cpu_set_debug_core(int core_id, int enabled) {
    if (core_id < NUM_CORES) {
        cores[core_id].debug_enabled = enabled;
    }
}
