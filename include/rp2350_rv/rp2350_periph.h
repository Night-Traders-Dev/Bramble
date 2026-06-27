/*
 * RP2350-Specific Peripheral Emulation
 *
 * Peripherals that are new or different on RP2350 vs RP2040:
 *   - TICKS (0x40108000): Tick generators for watchdog/timer clocking
 *   - POWMAN (0x40100000): Power manager (AON domain)
 *   - QMI (0x400D0000): QSPI memory interface (replaces XIP SSI)
 *   - OTP (0x40120000/0x40130000): One-time programmable memory + data readout
 *   - BOOTRAM (0x400E0000): 256-byte boot scratch RAM
 *   - GLITCH (0x40158000): Glitch detector
 *   - CORESIGHT (0x40140000): CoreSight trace peripherals
 *   - ACCESSCTRL (0x40160000): Peripheral access control
 *   - TIMER1 (0x400B8000): Second hardware timer (instance of RP2040 timer)
 *   - PIO2 (0x50400000): Third PIO block
 *
 * These are handled in the RV memory bus and membus fallthrough for RP2350 mode.
 */

#ifndef RP2350_PERIPH_H
#define RP2350_PERIPH_H

#include <stdint.h>

/* ========================================================================
 * TICKS Peripheral (0x40108000)
 * Provides tick generators for proc0, proc1, timer0, timer1, watchdog, etc.
 * Each generator has CTRL (enable + cycles) and CYCLES registers.
 * ======================================================================== */

#define RP2350_TICKS_NUM_GENERATORS  9
/* Generator indices: PROC0=0, PROC1=1, TIMER0=2, TIMER1=3, WATCHDOG=4,
 * RISCV=5, REFTICK=6, ADC=7, reserved=8 */

typedef struct {
    uint32_t ctrl[RP2350_TICKS_NUM_GENERATORS];   /* Enable + cycle count */
    uint32_t cycles[RP2350_TICKS_NUM_GENERATORS]; /* Running cycle counter */
} rp2350_ticks_state_t;

/* ========================================================================
 * POWMAN (0x40100000)
 * Power manager — controls voltage regulators, power domains, AON timer.
 * ======================================================================== */

typedef struct {
    uint32_t vreg_ctrl;        /* VREG control */
    uint32_t bod_ctrl;         /* Brown-out detector */
    uint32_t state;            /* Current power state */
    uint32_t timer;            /* AON timer (64-bit, low word) */
    uint32_t timer_hi;         /* AON timer high word */
    uint32_t inte;             /* Interrupt enable */
    uint32_t intf;             /* Interrupt force */
    uint32_t ints;             /* Interrupt status */
    uint32_t regs[32];         /* General register storage */
} rp2350_powman_state_t;

/* ========================================================================
 * QMI (0x400D0000)
 * QSPI Memory Interface — controls flash access (replaces XIP SSI on RP2350).
 * ======================================================================== */

typedef struct {
    uint32_t direct_csr;       /* Direct-mode control */
    uint32_t direct_tx;        /* Direct TX data */
    uint32_t direct_rx;        /* Direct RX data */
    uint32_t m0_timing;        /* Memory 0 timing */
    uint32_t m0_rfmt;          /* Memory 0 read format */
    uint32_t m0_rcmd;          /* Memory 0 read command */
    uint32_t m0_wfmt;          /* Memory 0 write format */
    uint32_t m0_wcmd;          /* Memory 0 write command */
    uint32_t m1_timing;        /* Memory 1 timing */
    uint32_t m1_rfmt;          /* Memory 1 read format */
    uint32_t m1_rcmd;          /* Memory 1 read command */
    uint32_t m1_wfmt;          /* Memory 1 write format */
    uint32_t m1_wcmd;          /* Memory 1 write command */
    uint32_t atrans[8];        /* Address translation */
} rp2350_qmi_state_t;

/* ========================================================================
 * OTP (0x40120000 control, 0x40130000 data)
 * One-Time Programmable memory — 8192 rows of 16-bit data.
 * ======================================================================== */

#define OTP_NUM_ROWS  8192

typedef struct {
    uint16_t data[OTP_NUM_ROWS];  /* OTP data (16-bit per row) */
    uint32_t ctrl_regs[32];       /* OTP controller registers */
} rp2350_otp_state_t;

/* ========================================================================
 * BOOTRAM (0x400E0000)
 * 1KB boot scratch RAM plus adjacent bootrom-owned registers.
 * ======================================================================== */

#define BOOTRAM_SIZE                 0x400
#define BOOTRAM_WRITE_ONCE_OFFSET    0x800
#define BOOTRAM_BOOTLOCK_STAT_OFFSET 0x808
#define BOOTRAM_BOOTLOCK0_OFFSET     0x80C
#define BOOTRAM_BOOTLOCK_COUNT       8
#define BOOTRAM_REGS_END             0x82C

/* ========================================================================
 * TIMER1 (0x400B8000)
 * Second hardware timer — same register layout as RP2040 timer.
 * ======================================================================== */

typedef struct {
    uint64_t time_us;
    uint32_t alarm[4];
    uint32_t armed;
    uint32_t intr;
    uint32_t inte;
    uint32_t intf;
    uint32_t paused;
    uint32_t latched_high;     /* For atomic 64-bit reads */
} rp2350_timer1_state_t;

/* ========================================================================
 * Unified RP2350 Peripheral State
 * ======================================================================== */

typedef struct {
    rp2350_ticks_state_t ticks;
    rp2350_powman_state_t powman;
    rp2350_qmi_state_t qmi;
    rp2350_otp_state_t otp;
    rp2350_timer1_state_t timer1;
    uint8_t bootram[BOOTRAM_SIZE];
    uint32_t bootram_write_once[2];
    uint32_t bootram_bootlock_stat;
    uint32_t glitch_regs[8];      /* Glitch detector */
    uint32_t coresight_regs[16];  /* CoreSight trace */
    uint32_t accessctrl_regs[64]; /* Access control */
} rp2350_periph_state_t;

/* ========================================================================
 * API
 * ======================================================================== */

void rp2350_periph_init(rp2350_periph_state_t *state, int for_arm);

/* Returns 1 if addr is handled, 0 otherwise */
int rp2350_periph_match(uint32_t addr);

uint32_t rp2350_periph_read32(rp2350_periph_state_t *state, uint32_t addr);
void rp2350_periph_write32(rp2350_periph_state_t *state, uint32_t addr, uint32_t val);

uint8_t rp2350_periph_read8(rp2350_periph_state_t *state, uint32_t addr);
void rp2350_periph_write8(rp2350_periph_state_t *state, uint32_t addr, uint8_t val);

/* Timer1 tick (called from CLINT tick) */
void rp2350_timer1_tick(rp2350_periph_state_t *state, uint32_t us);

#endif /* RP2350_PERIPH_H */
