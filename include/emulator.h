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

/* SRAM alias (mirror at +0x01000000) */
#define SRAM_ALIAS_BASE 0x21000000

/* XIP aliases and cache control */
#define XIP_NOALLOC_BASE     0x11000000  /* XIP non-allocating (uncached read) */
#define XIP_NOCACHE_BASE     0x12000000  /* XIP cache bypass */
#define XIP_NOCACHE_NOALLOC  0x13000000  /* XIP bypass + non-allocating */
#define XIP_CTRL_BASE        0x14000000  /* XIP cache control registers */
#define XIP_SRAM_BASE        0x15000000  /* XIP cache as SRAM (16KB) */
#define XIP_SRAM_SIZE        (16 * 1024)
#define XIP_SSI_BASE         0x18000000  /* XIP SSI (flash serial interface) */

/* Emulator guard (not a hardware boundary) */
#define MEM_END         0x30000000

/* ========================================================================
 * Cycle-Accurate Timing
 *
 * Configurable clock frequency for accurate timer/SysTick behavior.
 * Default: 1 cycle per µs (fast-forward mode, backward compatible).
 * Real RP2040: 125 cycles per µs (125 MHz).
 * ======================================================================== */

typedef struct {
    uint32_t cycles_per_us;     /* Clock cycles per microsecond (1=fast, 125=real) */
    uint32_t cycle_accumulator; /* Accumulated cycles not yet converted to µs */
} timing_config_t;

extern timing_config_t timing_config;

/* Set clock frequency in MHz (e.g., 125 for real RP2040) */
void timing_set_clock_mhz(uint32_t mhz);

/* Get instruction cycle cost for a 16-bit Thumb instruction */
uint32_t timing_instruction_cycles(uint16_t instr, int branch_taken);

/* Get cycle cost for a 32-bit Thumb instruction */
uint32_t timing_instruction_cycles_32(uint16_t upper, uint16_t lower);

/* ========================================================================
 * Peripheral Base Addresses
 * ======================================================================== */

/* SIO (Single-cycle I/O) - RP2040 core registers */
#define SIO_BASE        0xD0000000

/* Peripherals - see individual headers (uart.h, spi.h, i2c.h, pwm.h) */

/* USB — see usb.h for full register definitions */

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
    uint32_t primask;               /* PRIMASK register (1=interrupts disabled) */
    uint32_t faultmask;             /* FAULTMASK register (1=all exceptions masked except NMI) */
    uint32_t control;               /* CONTROL register (SPSEL, nPRIV) */

} cpu_state_t;

extern cpu_state_t cpu;

/* ========================================================================
 * Dual-Core Configuration
 * ======================================================================== */

#define MAX_CORES               2       /* RP2040 hardware maximum */
#define NUM_CORES               2       /* Compile-time max (array sizing) */
#define CORE0                   0
#define CORE1                   1

/* Runtime-configurable active core count (default: MAX_CORES) */
extern int num_active_cores;

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

/* Exception nesting depth limit */
#define MAX_EXCEPTION_DEPTH     8

/* Hardware spinlock configuration (RP2040) */
#define SPINLOCK_BASE           0xD0000100
#define SPINLOCK_SIZE           32
#define SPINLOCK_LOCKED         0x00000001
#define SPINLOCK_VALID          0x80000000

/* ========================================================================
 * Dual-Core CPU State (Extended)
 * ======================================================================== */

typedef struct {
    /* Per-core private RAM (flash is shared via cpu.flash) */
    uint8_t ram[CORE_RAM_SIZE];     /* Per-core private RAM */
    uint32_t r[16];                 /* R0-R15 */
    uint32_t xpsr;                  /* APSR */
    uint32_t vtor;                  /* Vector Table Offset */
    uint32_t step_count;            /* Instruction counter */
    int debug_enabled;
    int debug_asm;
    uint32_t current_irq;
    uint32_t primask;               /* PRIMASK register (1=interrupts disabled) */
    uint32_t faultmask;             /* FAULTMASK register (1=all exceptions masked except NMI) */
    uint32_t control;               /* CONTROL register (SPSEL, nPRIV) */

    /* Exception nesting (per-core) */
    uint32_t exception_stack[MAX_EXCEPTION_DEPTH];
    int exception_depth;

    /* Dual-core extensions */
    int core_id;                    /* 0 or 1 */
    int is_halted;                  /* Execution state */
    int is_wfi;                     /* Core sleeping (WFI/WFE), skip until interrupt */
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
 * Memory Bus Configuration
 * ======================================================================== */

/* Set active RAM pointer for memory bus routing (eliminates per-step memcpy) */
void mem_set_ram_ptr(uint8_t *ram, uint32_t base, uint32_t size);

/* pc_updated flag: set by instruction handlers that modify PC */
extern int pc_updated;

/* Debug logging for unmapped peripheral accesses (-debug-mem flag) */
extern int mem_debug_unmapped;

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

/* Decoded instruction cache */
void icache_init(void);
void icache_enable(int enable);
int icache_is_enabled(void);
void icache_invalidate_addr(uint32_t addr);
void icache_invalidate_range(uint32_t addr, uint32_t size);
void icache_invalidate_all(void);
void icache_report_stats(void);

/* JIT basic block cache */
void jit_init(void);
void jit_enable(int enable);
void jit_invalidate_addr(uint32_t addr);
void jit_invalidate_range(uint32_t addr, uint32_t size);
void jit_invalidate_all(void);
void jit_report_stats(void);

/* ========================================================================
 * CPU Control Functions (Dual-Core)
 * ======================================================================== */

typedef struct {
    uint32_t r[16];
    uint32_t xpsr;
    uint32_t vtor;
    uint32_t step_count;
    int debug_enabled;
    int debug_asm;
    uint32_t current_irq;
    uint32_t primask;
    uint32_t faultmask;
    uint32_t control;
    int active_core;
} cpu_bind_context_t;

/* Initialization */
void dual_core_init(void);

/* Per-core execution */
int cpu_bind_core_context(int core_id, cpu_bind_context_t *ctx);
void cpu_unbind_core_context(int core_id, const cpu_bind_context_t *ctx);
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
int sio_core1_bootrom_handle_fifo_write(uint32_t val);

/* Boot2 detection */
int cpu_has_boot2(void);
void cpu_set_boot2(int enable);

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

/* UF2 family IDs */
#define UF2_FAMILY_RP2040       0xE48BFF56
#define UF2_FAMILY_RP2350_ARM   0xE48BFF59
#define UF2_FAMILY_RP2350_RV    0xE48BFF5A
#define UF2_FLAG_FAMILY_PRESENT 0x00002000

/* Detected firmware architecture (set by loaders) */
#define FW_ARCH_UNKNOWN  0
#define FW_ARCH_ARM_M0P  1  /* Cortex-M0+ (RP2040) */
#define FW_ARCH_ARM_M33  2  /* Cortex-M33 (RP2350 ARM) */
#define FW_ARCH_RV32     3  /* Hazard3 RISC-V (RP2350 RV) */

int load_uf2(const char *filename);
int load_elf(const char *filename);

/* Returns the architecture detected by the last load_uf2/load_elf call */
int loader_detected_arch(void);

#endif /* EMULATOR_H */
