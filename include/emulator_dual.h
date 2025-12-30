/*
 * RP2040 Dual-Core Emulator Extension (emulator_dual.h)
 * 
 * This header extends the base emulator with:
 * - Dual independent CPU cores (Cortex-M0+ #0 and #1)
 * - Shared memory regions (Flash, SRAM)
 * - Per-core vector tables and interrupt handling
 * - Multicore FIFO for inter-processor communication
 * - Spinlocks for synchronization
 * - Per-core timer and NVIC state
 */

#ifndef EMULATOR_DUAL_H
#define EMULATOR_DUAL_H

#include <stdint.h>
#include <string.h>
#include "emulator.h"

/* ========================================================================
 * Dual-Core Configuration
 * ======================================================================== */

#define NUM_CORES 2
#define CORE0 0
#define CORE1 1

/* Per-core stack allocation (split RAM_SIZE between cores) */
#define CORE_RAM_SIZE (RAM_SIZE / 2)
#define CORE0_RAM_START 0x20000000
#define CORE0_RAM_END   (CORE0_RAM_START + CORE_RAM_SIZE)
#define CORE1_RAM_START CORE0_RAM_END
#define CORE1_RAM_END   (CORE1_RAM_START + CORE_RAM_SIZE)

/* Shared region in RAM for inter-processor communication */
#define SHARED_RAM_BASE 0x20040000
#define SHARED_RAM_SIZE (64 * 1024)  /* 64 KB shared RAM */

/* Multicore FIFO: Stores 32-bit messages between cores */
#define FIFO_DEPTH 32

/* Spinlock hardware addresses (RP2040) */
#define SPINLOCK_BASE 0xD0000100
#define SPINLOCK_SIZE 32  /* 32 hardware spinlocks available */

/* ========================================================================
 * Per-Core State (Extended cpu_state_t)
 * ======================================================================== */

typedef struct {
    /* Original CPU state from emulator.h */
    uint8_t  flash[FLASH_SIZE];  /* Shared across cores */
    uint8_t  ram[CORE_RAM_SIZE]; /* Per-core private RAM */
    uint32_t r[16];              /* R0-R15 (R13=SP, R14=LR, R15=PC) */
    uint32_t xpsr;               /* Application Program Status Register */
    uint32_t vtor;               /* Vector Table Offset Register */
    uint32_t step_count;
    int      debug_enabled;
    int      debug_asm;
    uint32_t current_irq;
    
    /* Dual-core extensions */
    int      core_id;            /* 0 or 1 */
    int      is_halted;          /* Execution state */
    uint32_t exception_sp;       /* Saved SP for exception handling */
    int      in_handler_mode;    /* True if currently in ISR */
} cpu_state_dual_t;

/* ========================================================================
 * Global Dual-Core State
 * ======================================================================== */

extern cpu_state_dual_t cores[NUM_CORES];
extern uint32_t shared_ram[SHARED_RAM_SIZE / 4];
extern uint32_t spinlocks[SPINLOCK_SIZE];

/* ========================================================================
 * Multicore FIFO (MCP: Multicore FIFO)
 * ======================================================================== */

typedef struct {
    uint32_t messages[FIFO_DEPTH];  /* Circular buffer */
    uint32_t write_ptr;
    uint32_t read_ptr;
    uint32_t count;
} multicore_fifo_t;

extern multicore_fifo_t fifo[NUM_CORES];  /* Per-direction FIFOs */

/* Hardware FIFO addresses (RP2040) */
#define FIFO0_WR  0xD0000050  /* Core 0 write, read from Core 1 */
#define FIFO0_RD  0xD0000058
#define FIFO1_WR  0xD0000054  /* Core 1 write, read from Core 0 */
#define FIFO1_RD  0xD000005C

/* ========================================================================
 * Multicore Synchronization Primitives
 * ======================================================================== */

/* Spinlock flags and hardware implementation */
#define SPINLOCK_LOCKED   0x00000001
#define SPINLOCK_VALID    0x80000000

/* Spinlock operations */
uint32_t spinlock_acquire(uint32_t lock_num);  /* Wait and acquire */
void     spinlock_release(uint32_t lock_num);  /* Release */

/* FIFO operations - per-core perspective */
int      fifo_is_empty(int core_id);
int      fifo_is_full(int core_id);
uint32_t fifo_pop(int core_id);                /* Blocking read from my FIFO */
void     fifo_push(int core_id, uint32_t val); /* Write to other core's FIFO */
int      fifo_try_pop(int core_id, uint32_t *val); /* Non-blocking */
int      fifo_try_push(int core_id, uint32_t val); /* Non-blocking */

/* ========================================================================
 * Per-Core CPU Functions
 * ======================================================================== */

/* Initialize both cores */
void dual_core_init(void);

/* Step execution on specific core */
void cpu_step_core(int core_id);

/* Reset core from flash */
void cpu_reset_core(int core_id);

/* Check if core is halted */
int cpu_is_halted_core(int core_id);

/* Run both cores with simple round-robin scheduling */
void dual_core_step(void);  /* Step core 0, then core 1 */

/* ========================================================================
 * Memory Access (Dual-Core Aware)
 * ======================================================================== */

/* Unified memory access for both cores */
uint32_t mem_read32_dual(int core_id, uint32_t addr);
void     mem_write32_dual(int core_id, uint32_t addr, uint32_t val);
uint16_t mem_read16_dual(int core_id, uint32_t addr);
void     mem_write16_dual(int core_id, uint32_t addr, uint16_t val);
uint8_t  mem_read8_dual(int core_id, uint32_t addr);
void     mem_write8_dual(int core_id, uint32_t addr, uint8_t val);

/* Get current active core for global mem_* functions */
int get_active_core(void);
void set_active_core(int core_id);

/* ========================================================================
 * Shared Hardware Peripherals (Synchronized)
 * ======================================================================== */

/* Timer (shared across cores, per-core can have alarms) */
extern uint64_t timer_tick_counter;

/* GPIO and UART (truly shared) */
/* No special handling needed - both cores see same output */

/* ========================================================================
 * Exception Handling (Per-Core)
 * ======================================================================== */

void cpu_exception_entry_dual(int core_id, uint32_t vector_num);
void cpu_exception_return_dual(int core_id, uint32_t lr_value);

/* ========================================================================
 * SIO (Single-Cycle I/O) - Core-Aware Registers
 * ======================================================================== */

/* Core ID register access (R/O from CPU perspective) */
uint32_t sio_get_core_id(void);

/* Core control - used to reset/enable core 1 */
void sio_set_core1_reset(int assert_reset);
void sio_set_core1_stall(int stall);

/* ========================================================================
 * Testing & Debugging (Dual-Core)
 * ======================================================================== */

/* Check synchronization status */
void dual_core_status(void);

/* Per-core debug output */
void cpu_set_debug_core(int core_id, int enabled);

#endif /* EMULATOR_DUAL_H */
