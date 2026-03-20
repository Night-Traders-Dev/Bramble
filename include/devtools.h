/*
 * Developer Tools for Bramble RP2040 Emulator
 *
 * Provides:
 * - ARM semihosting (BKPT #0xAB interception)
 * - Code coverage bitmap (executed PC tracking)
 * - Hotspot profiling (top-N PCs by execution count)
 * - Instruction trace (PC + opcode stream to file)
 * - Exit code hook (read firmware return code from RAM)
 * - Timeout enforcement (wall-clock execution limit)
 */

#ifndef DEVTOOLS_H
#define DEVTOOLS_H

#include <stdint.h>
#include <stdio.h>
#include "emulator.h"

/* ========================================================================
 * ARM Semihosting (BKPT #0xAB)
 *
 * Standard ARM semihosting interface — used by test frameworks (Unity,
 * CppUTest) and newlib's --specs=rdimon.specs to route I/O to the host.
 *
 * Convention: BKPT #0xAB with R0=operation, R1=parameter block pointer.
 * ======================================================================== */

#define SEMIHOST_SYS_OPEN           0x01
#define SEMIHOST_SYS_CLOSE          0x02
#define SEMIHOST_SYS_WRITEC         0x03
#define SEMIHOST_SYS_WRITE0         0x04
#define SEMIHOST_SYS_WRITE          0x05
#define SEMIHOST_SYS_READ           0x06
#define SEMIHOST_SYS_READC          0x07
#define SEMIHOST_SYS_SEEK           0x0A
#define SEMIHOST_SYS_FLEN           0x0C
#define SEMIHOST_SYS_ERRNO          0x13
#define SEMIHOST_SYS_EXIT           0x18
#define SEMIHOST_SYS_EXIT_EXTENDED  0x20
#define SEMIHOST_SYS_ELAPSED        0x30
#define SEMIHOST_SYS_TICKFREQ       0x31

extern int semihosting_enabled;
extern int semihost_exit_requested;
extern int semihost_exit_code;

void semihosting_init(void);

/* Handle a BKPT #0xAB instruction.  Returns 1 if handled, 0 if not semihosting. */
int semihosting_handle(void);

/* ========================================================================
 * Code Coverage
 *
 * Bitmap tracking which 16-bit-aligned addresses have been executed.
 * Covers full flash range (2MB = 1M halfwords = 128KB bitmap).
 * ======================================================================== */

extern int coverage_enabled;

void coverage_init(void);
void coverage_cleanup(void);

/* Record a PC execution (called from cpu_step hot path) */
static inline void coverage_record(uint32_t pc);

/* Dump coverage data to file */
void coverage_dump(const char *path);

/* Print summary statistics */
void coverage_report(void);

/* ========================================================================
 * Hotspot Profiling
 *
 * Per-PC execution counter using a hash map.  Reports top-N addresses
 * by execution count on exit.
 * ======================================================================== */

extern int hotspots_enabled;
extern int hotspots_top_n;

void hotspots_init(void);
void hotspots_cleanup(void);

/* Record a PC execution (called from cpu_step hot path) */
static inline void hotspots_record(uint32_t pc);

/* Print top-N hotspots to stderr */
void hotspots_report(void);

/* ========================================================================
 * Instruction Trace
 *
 * Streams (PC, opcode, cycles) tuples to a binary file for offline
 * analysis.  Format: 4 bytes PC, 2 bytes opcode, 2 bytes cycles.
 * ======================================================================== */

extern int trace_enabled;
extern FILE *trace_file;

void trace_init(const char *path);
void trace_cleanup(void);

/* Record one instruction (called from cpu_step) */
static inline void trace_record(uint32_t pc, uint16_t opcode, uint16_t cycles);

/* ========================================================================
 * Exit Code Hook
 *
 * On halt, read a uint32 from a user-specified RAM address and use it
 * as the process exit code.  Enables CI test result reporting.
 * ======================================================================== */

extern int exit_code_enabled;
extern uint32_t exit_code_addr;

/* ========================================================================
 * Timeout Enforcement
 *
 * Wall-clock timeout in seconds.  If the emulator hasn't halted by then,
 * it is forcefully stopped and returns exit code 124 (like GNU timeout).
 * ======================================================================== */

extern int timeout_seconds;
extern volatile int timeout_expired;

void timeout_start(int seconds);
void timeout_cancel(void);

/* ========================================================================
 * SYSCFG Peripheral (0x40004000)
 *
 * System configuration: processor config, debug control.
 * Mostly read-only on real hardware.
 * ======================================================================== */

#define SYSCFG_BASE     0x40004000
#define SYSCFG_SIZE     0x20

int  syscfg_match(uint32_t addr);
uint32_t syscfg_read(uint32_t offset);
void syscfg_write(uint32_t offset, uint32_t val);

/* ========================================================================
 * TBMAN Peripheral (0x4006C000)
 *
 * Testbench Manager: indicates whether running on real hardware or
 * simulation.  Returns PLATFORM=1 (simulation) for emulator.
 * ======================================================================== */

#define TBMAN_BASE      0x4006C000
#define TBMAN_SIZE      0x08

int  tbman_match(uint32_t addr);
uint32_t tbman_read(uint32_t offset);
void tbman_write(uint32_t offset, uint32_t val);

/* ========================================================================
 * Inline Implementations (must be in header for hot-path inlining)
 * ======================================================================== */

/* Coverage bitmap: 1 bit per halfword in flash (2MB/2 = 1M bits = 128KB) */
#define COVERAGE_FLASH_BITS (FLASH_SIZE / 2)
#define COVERAGE_BITMAP_BYTES ((COVERAGE_FLASH_BITS + 7) / 8)

extern uint8_t *coverage_bitmap;    /* Flash coverage */
extern uint8_t *coverage_ram_bitmap; /* RAM coverage (RAM_SIZE/2 bits) */

static inline void coverage_record(uint32_t pc) {
    if (!coverage_enabled) return;
    if (pc >= FLASH_BASE && pc < FLASH_BASE + FLASH_SIZE) {
        uint32_t idx = (pc - FLASH_BASE) >> 1;
        coverage_bitmap[idx >> 3] |= (1u << (idx & 7));
    } else if (pc >= RAM_BASE && pc < RAM_BASE + RAM_SIZE) {
        uint32_t idx = (pc - RAM_BASE) >> 1;
        coverage_ram_bitmap[idx >> 3] |= (1u << (idx & 7));
    }
}

/* Hotspot hash map: direct-mapped by (PC >> 1) & mask */
#define HOTSPOT_MAP_SIZE    (1 << 18)  /* 256K entries */
#define HOTSPOT_MAP_MASK    (HOTSPOT_MAP_SIZE - 1)

typedef struct {
    uint32_t pc;
    uint32_t count;
} hotspot_entry_t;

extern hotspot_entry_t *hotspot_map;

static inline void hotspots_record(uint32_t pc) {
    if (!hotspots_enabled) return;
    uint32_t idx = (pc >> 1) & HOTSPOT_MAP_MASK;
    hotspot_entry_t *e = &hotspot_map[idx];
    if (e->pc == pc) {
        e->count++;
    } else if (e->count == 0) {
        e->pc = pc;
        e->count = 1;
    } else {
        /* Collision: simple frequency-based eviction */
        e->count++;
        e->pc = pc;
    }
}

static inline void trace_record(uint32_t pc, uint16_t opcode, uint16_t cycles) {
    if (!trace_enabled || !trace_file) return;
    uint8_t rec[8];
    rec[0] = pc & 0xFF; rec[1] = (pc >> 8) & 0xFF;
    rec[2] = (pc >> 16) & 0xFF; rec[3] = (pc >> 24) & 0xFF;
    rec[4] = opcode & 0xFF; rec[5] = (opcode >> 8) & 0xFF;
    rec[6] = cycles & 0xFF; rec[7] = (cycles >> 8) & 0xFF;
    fwrite(rec, 1, 8, trace_file);
}

/* ========================================================================
 * ELF Symbol Table Loading
 *
 * Parses .symtab + .strtab from an ELF32 file to resolve addresses to
 * function names.  Used by hotspots, trace, callgraph, and crash reports.
 * ======================================================================== */

extern int symbols_loaded;

int  symbols_load(const char *elf_path);
void symbols_cleanup(void);
const char *symbols_lookup(uint32_t addr, uint32_t *offset_out);

/* ========================================================================
 * Scripted I/O
 *
 * Feeds timestamped UART/GPIO input from a text file:
 *   100ms: uart0 "hello\n"
 *   200ms: gpio25 1
 * ======================================================================== */

extern int script_enabled;

int  script_init(const char *path);
void script_poll(uint32_t elapsed_us);
void script_cleanup(void);

/* ========================================================================
 * Expected Output Matching
 *
 * Captures stdout and compares against a golden file on exit.
 * Exit 0 on match, exit 1 on diff.
 * ======================================================================== */

extern int expect_enabled;
extern char *expect_path;
extern char *expect_capture_buf;
extern size_t expect_capture_len;
extern size_t expect_capture_cap;

void expect_init(const char *path);
void expect_append(const char *data, size_t len);
int  expect_check(void);
void expect_cleanup(void);

/* ========================================================================
 * Memory Watch Log
 *
 * Logs every read/write to a specified address range to stderr.
 * Lightweight alternative to GDB watchpoints.
 * ======================================================================== */

#define MAX_WATCH_REGIONS 8

typedef struct {
    uint32_t addr;
    uint32_t len;
    int active;
} watch_region_t;

extern watch_region_t watch_regions[MAX_WATCH_REGIONS];
extern int watch_count;

int  watch_add(uint32_t addr, uint32_t len);
void watch_check_write(uint32_t addr, uint32_t val, int width);
void watch_check_read(uint32_t addr, uint32_t val, int width);

/* ========================================================================
 * Call Graph
 *
 * Tracks BL/BLX calls and POP-PC/BX-LR returns to build a caller→callee
 * graph with call counts.  Outputs DOT format.
 * ======================================================================== */

extern int callgraph_enabled;

void callgraph_init(void);
void callgraph_record_call(uint32_t caller_pc, uint32_t target_pc);
void callgraph_dump(const char *path);
void callgraph_cleanup(void);

/* ========================================================================
 * Stack Watermark
 *
 * Tracks SP high-water mark (lowest SP value) per core.
 * Reports stack depth on exit, warns if SP enters danger zone.
 * ======================================================================== */

extern int stack_check_enabled;
extern uint32_t stack_watermark[2]; /* Per-core lowest SP seen */

static inline void stack_check_record(int core, uint32_t sp) {
    if (!stack_check_enabled) return;
    if (sp < stack_watermark[core] && sp >= RAM_BASE) {
        stack_watermark[core] = sp;
    }
}

void stack_check_report(void);

/* ========================================================================
 * IRQ Latency Profiling
 *
 * Measures cycles from nvic_set_pending() to handler entry for each IRQ.
 * ======================================================================== */

#define IRQ_LATENCY_MAX_IRQS 32

typedef struct {
    uint64_t total_cycles;
    uint32_t count;
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint64_t pend_cycle;  /* Cycle count when last pended */
} irq_latency_entry_t;

extern int irq_latency_enabled;
extern irq_latency_entry_t irq_latency[IRQ_LATENCY_MAX_IRQS];
extern uint64_t global_cycle_count;

void irq_latency_pend(uint32_t irq);
void irq_latency_enter(uint32_t irq);
void irq_latency_report(void);

/* ========================================================================
 * Bus Transaction Logging (UART/SPI/I2C)
 * ======================================================================== */

extern int log_uart_enabled;
extern int log_spi_enabled;
extern int log_i2c_enabled;

void bus_log_uart(int num, int is_tx, uint8_t byte);
void bus_log_spi(int num, int is_tx, uint8_t byte);
void bus_log_i2c(int num, int is_write, uint8_t addr, uint8_t byte);

/* ========================================================================
 * GPIO VCD Trace
 *
 * Records GPIO pin changes with cycle-accurate timestamps.
 * Output in Value Change Dump (VCD) format for GTKWave/PulseView.
 * ======================================================================== */

extern int gpio_trace_enabled;

void gpio_trace_init(const char *path);
void gpio_trace_record(uint8_t pin, uint8_t value);
void gpio_trace_cleanup(void);

/* ========================================================================
 * Fault Injection
 *
 * Simulate hardware faults at specified cycle counts:
 *   flash_bitflip:<cycle>:<addr>    Flip a bit in flash
 *   ram_corrupt:<cycle>:<addr>      Write garbage to RAM
 *   brownout:<cycle>                Trigger watchdog reboot
 * ======================================================================== */

#define MAX_FAULT_INJECTIONS 16

typedef enum {
    FAULT_FLASH_BITFLIP,
    FAULT_RAM_CORRUPT,
    FAULT_BROWNOUT,
} fault_type_t;

typedef struct {
    fault_type_t type;
    uint64_t trigger_cycle;
    uint32_t addr;
    int fired;
} fault_injection_t;

extern fault_injection_t fault_injections[MAX_FAULT_INJECTIONS];
extern int fault_count;

int  fault_add(const char *spec);
void fault_check(uint64_t cycle);

/* ========================================================================
 * Cycle Profiling
 *
 * Per-address cycle accounting — tracks total cycles consumed at each PC.
 * Combined with -symbols gives per-function timing.
 * ======================================================================== */

extern int profile_enabled;

void profile_init(void);
void profile_cleanup(void);

static inline void profile_record(uint32_t pc, uint32_t cycles);

void profile_dump(const char *path);
void profile_report(void);

/* Profile map: same structure as hotspots but tracking cycles */
typedef struct {
    uint32_t pc;
    uint32_t total_cycles;
    uint32_t count;
} profile_entry_t;

#define PROFILE_MAP_SIZE    (1 << 18)
#define PROFILE_MAP_MASK    (PROFILE_MAP_SIZE - 1)

extern profile_entry_t *profile_map;

static inline void profile_record(uint32_t pc, uint32_t cycles) {
    if (!profile_enabled) return;
    uint32_t idx = (pc >> 1) & PROFILE_MAP_MASK;
    profile_entry_t *e = &profile_map[idx];
    if (e->pc == pc) {
        e->total_cycles += cycles;
        e->count++;
    } else if (e->count == 0) {
        e->pc = pc;
        e->total_cycles = cycles;
        e->count = 1;
    } else {
        e->total_cycles += cycles;
        e->count++;
        e->pc = pc;
    }
}

/* ========================================================================
 * Memory Access Heatmap
 *
 * Tracks read/write frequency per 256-byte block of the address space.
 * Dumps as raw data for visualization.
 * ======================================================================== */

extern int mem_heatmap_enabled;

void mem_heatmap_init(void);
void mem_heatmap_cleanup(void);

static inline void mem_heatmap_record_read(uint32_t addr);
static inline void mem_heatmap_record_write(uint32_t addr);

void mem_heatmap_dump(const char *path);

/* Heatmap covers RAM region: RAM_SIZE / 256 = ~1K entries */
#define HEATMAP_BLOCK_SHIFT  8
#define HEATMAP_RAM_BLOCKS   (RAM_SIZE >> HEATMAP_BLOCK_SHIFT)

typedef struct {
    uint32_t reads;
    uint32_t writes;
} heatmap_entry_t;

extern heatmap_entry_t *heatmap_ram;

static inline void mem_heatmap_record_read(uint32_t addr) {
    if (!mem_heatmap_enabled) return;
    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t idx = (addr - RAM_BASE) >> HEATMAP_BLOCK_SHIFT;
        heatmap_ram[idx].reads++;
    }
}

static inline void mem_heatmap_record_write(uint32_t addr) {
    if (!mem_heatmap_enabled) return;
    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE) {
        uint32_t idx = (addr - RAM_BASE) >> HEATMAP_BLOCK_SHIFT;
        heatmap_ram[idx].writes++;
    }
}

#endif /* DEVTOOLS_H */
