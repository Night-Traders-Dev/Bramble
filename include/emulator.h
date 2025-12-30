/*
 * RP2040 Emulator Header (Unified Single & Dual-Core)
 *
 * Consolidated header providing:
 * - Single-core CPU state and operations
 * - Dual-core extensions with shared memory and synchronization
 * - Unified memory access functions
 * - Peripheral interfaces (UART, GPIO, Timer, NVIC)
 * - Firmware loading (UF2 format)
 */

#ifndef EMULATOR_H
#define EMULATOR_H

#include <stdint.h>
#include <stdio.h>

/* ========================================================================
 * Build Configuration
 * ======================================================================== */

/* Select operating mode - define exactly ONE */
//#define SINGLE_CORE_ENABLED
#define DUAL_CORE_ENABLED

/* ========================================================================
 * Memory Layout (RP2040)
 * ======================================================================== */

#define FLASH_BASE      0x10000000
#define FLASH_SIZE      (2 * 1024 * 1024)    /* 2 MB XIP flash */
#define RAM_BASE        0x20000000
#define RAM_SIZE        (264 * 1024)         /* 264 KB on-chip SRAM */
#define RAM_TOP         (RAM_BASE + RAM_SIZE)

/* Emulator guard (not a hardware boundary) */
#define MEM_END         0x30000000

/* ========================================================================
 * Peripheral Base Addresses
 * ======================================================================== */

/* SIO (Single-cycle I/O) - RP2040 core registers */
#define SIO_BASE        0xD0000000

/* UART0 (PL011-style) */
#define UART0_BASE      0x40034000
#define UART0_DR        (UART0_BASE + 0x000)  /* Data Register */
#define UART0_FR        (UART0_BASE + 0x018)  /* Flag Register */

/* ========================================================================
 * Single-Core CPU State (Base)
 * ======================================================================== */

typedef struct {
    /* Memory */
    uint8_t flash[FLASH_SIZE];      /* Instruction & data flash */
    uint8_t ram[RAM_SIZE];          /* On-chip SRAM */

    /* Registers */
    uint32_t r[16];                 /* R0-R15: R13=SP, R14=LR, R15=PC */
    uint32_t xpsr;                  /* Application Program Status Register */
    uint32_t vtor;                  /* Vector Table Offset Register */

    /* State & Debugging */
    uint32_t step_count;            /* Instruction counter */
    int debug_enabled;              /* Verbose CPU step output (-debug) */
    int debug_asm;                  /* Instruction-level tracing (-asm) */
    uint32_t current_irq;           /* Active interrupt number */

} cpu_state_t;

extern cpu_state_t cpu;

/* ========================================================================
 * Dual-Core Configuration
 * ======================================================================== */

#define NUM_CORES               2
#define CORE0                   0
#define CORE1                   1

/* Per-core memory allocation (split shared RAM) */
#define CORE_RAM_SIZE           (RAM_SIZE / 2)
#define CORE0_RAM_START         0x20000000
#define CORE0_RAM_END           (CORE0_RAM_START + CORE_RAM_SIZE)
#define CORE1_RAM_START         CORE0_RAM_END
#define CORE1_RAM_END           (CORE1_RAM_START + CORE_RAM_SIZE)

/* Shared RAM for inter-processor communication */
#define SHARED_RAM_BASE         0x20040000
#define SHARED_RAM_SIZE         (64 * 1024)

/* Multicore FIFO configuration */
#define FIFO_DEPTH              32

/* Hardware spinlock configuration (RP2040) */
#define SPINLOCK_BASE           0xD0000100
#define SPINLOCK_SIZE           32
#define SPINLOCK_LOCKED         0x00000001
#define SPINLOCK_VALID          0x80000000

/* ========================================================================
 * Dual-Core CPU State (Extended)
 * ======================================================================== */

typedef struct {
    /* Original CPU state */
    uint8_t flash[FLASH_SIZE];      /* Shared flash across cores */
    uint8_t ram[CORE_RAM_SIZE];     /* Per-core private RAM */
    uint32_t r[16];                 /* R0-R15 */
    uint32_t xpsr;                  /* APSR */
    uint32_t vtor;                  /* Vector Table Offset */
    uint32_t step_count;            /* Instruction counter */
    int debug_enabled;
    int debug_asm;
    uint32_t current_irq;

    /* Dual-core extensions */
    int core_id;                    /* 0 or 1 */
    int is_halted;                  /* Execution state */
    uint32_t exception_sp;          /* Saved SP for exception handling */
    int in_handler_mode;            /* True if currently in ISR */

} cpu_state_dual_t;

extern cpu_state_dual_t cores[NUM_CORES];
extern uint32_t shared_ram[SHARED_RAM_SIZE / 4];
extern uint32_t spinlocks[SPINLOCK_SIZE];

/* ========================================================================
 * Multicore FIFO (MCP: Multicore Protocol)
 * ======================================================================== */

typedef struct {
    uint32_t messages[FIFO_DEPTH];  /* Circular buffer */
    uint32_t write_ptr;
    uint32_t read_ptr;
    uint32_t count;
} multicore_fifo_t;

extern multicore_fifo_t fifo[NUM_CORES];

/* Hardware FIFO addresses (RP2040) */
#define FIFO0_WR        0xD0000050  /* Core 0 write, Core 1 reads */
#define FIFO0_RD        0xD0000058
#define FIFO1_WR        0xD0000054  /* Core 1 write, Core 0 reads */
#define FIFO1_RD        0xD000005C

/* ========================================================================
 * Memory Access Functions (Single-Core)
 * ======================================================================== */

void mem_write32(uint32_t addr, uint32_t val);
uint32_t mem_read32(uint32_t addr);

void mem_write16(uint32_t addr, uint16_t val);
uint16_t mem_read16(uint32_t addr);

void mem_write8(uint32_t addr, uint8_t val);
uint8_t mem_read8(uint32_t addr);

/* ========================================================================
 * Memory Access Functions (Dual-Core)
 * ======================================================================== */

uint32_t mem_read32_dual(int core_id, uint32_t addr);
void mem_write32_dual(int core_id, uint32_t addr, uint32_t val);

uint16_t mem_read16_dual(int core_id, uint32_t addr);
void mem_write16_dual(int core_id, uint32_t addr, uint16_t val);

uint8_t mem_read8_dual(int core_id, uint32_t addr);
void mem_write8_dual(int core_id, uint32_t addr, uint8_t val);

/* Context-aware memory access */
int get_active_core(void);
void set_active_core(int core_id);

/* ========================================================================
 * Instruction Execution
 * ======================================================================== */

void instr_cmp_imm8(uint16_t instr);
void instr_ldrb_reg_offset(uint16_t instr);

/* ========================================================================
 * CPU Control Functions (Single-Core)
 * ======================================================================== */

void cpu_init(void);
void cpu_step(void);
int cpu_is_halted(void);
void cpu_reset_from_flash(void);

void cpu_exception_entry(uint32_t vector_num);
void cpu_exception_return(uint32_t lr_value);

/* ========================================================================
 * CPU Control Functions (Dual-Core)
 * ======================================================================== */

/* Initialization */
void dual_core_init(void);

/* Per-core execution */
void cpu_step_core(int core_id);
void cpu_reset_core(int core_id);
int cpu_is_halted_core(int core_id);

/* Synchronized execution */
void dual_core_step(void);

/* Exception handling */
void cpu_exception_entry_dual(int core_id, uint32_t vector_num);
void cpu_exception_return_dual(int core_id, uint32_t lr_value);

/* SIO (Single-Cycle I/O) operations */
uint32_t sio_get_core_id(void);
void sio_set_core1_reset(int assert_reset);
void sio_set_core1_stall(int stall);

/* Debugging */
void dual_core_status(void);
void cpu_set_debug_core(int core_id, int enabled);

/* ========================================================================
 * Spinlock Operations (Hardware Synchronization)
 * ======================================================================== */

uint32_t spinlock_acquire(uint32_t lock_num);
void spinlock_release(uint32_t lock_num);

/* ========================================================================
 * Multicore FIFO Operations
 * ======================================================================== */

int fifo_is_empty(int core_id);
int fifo_is_full(int core_id);

uint32_t fifo_pop(int core_id);          /* Blocking read */
void fifo_push(int core_id, uint32_t val);  /* Write to other core */

int fifo_try_pop(int core_id, uint32_t *val);     /* Non-blocking read */
int fifo_try_push(int core_id, uint32_t val);     /* Non-blocking write */

/* ========================================================================
 * Firmware Loading
 * ======================================================================== */

int load_uf2(const char *filename);

#endif /* EMULATOR_H */
