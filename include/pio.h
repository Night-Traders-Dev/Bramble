/*
 * RP2040 PIO (Programmable I/O) Emulation
 *
 * Two PIO blocks (PIO0, PIO1), each with:
 * - 4 state machines (SM0-SM3)
 * - 32-word shared instruction memory
 * - Per-SM: CLKDIV, EXECCTRL, SHIFTCTRL, ADDR, INSTR, PINCTRL
 * - Per-SM: TX/RX FIFOs (4-deep each)
 * - Global: CTRL, FSTAT, FDEBUG, FLEVEL, IRQ, IRQ_FORCE
 *
 * This is a register-level implementation that allows SDK code to
 * configure PIO without crashing. Actual PIO instruction execution
 * is not emulated — FIFOs accept writes and return 0 on reads.
 */

#ifndef PIO_H
#define PIO_H

#include <stdint.h>

/* ========================================================================
 * PIO Base Addresses
 * ======================================================================== */

#define PIO0_BASE           0x50200000
#define PIO1_BASE           0x50300000
#define PIO_BLOCK_SIZE      0x1000
#define PIO_NUM_BLOCKS      2
#define PIO_NUM_SM           4
#define PIO_INSTR_MEM_SIZE  32

/* ========================================================================
 * Register Offsets (from PIO base)
 * ======================================================================== */

#define PIO_CTRL            0x000
#define PIO_FSTAT           0x004
#define PIO_FDEBUG          0x008
#define PIO_FLEVEL          0x00C
#define PIO_TXF0            0x010
#define PIO_TXF1            0x014
#define PIO_TXF2            0x018
#define PIO_TXF3            0x01C
#define PIO_RXF0            0x020
#define PIO_RXF1            0x024
#define PIO_RXF2            0x028
#define PIO_RXF3            0x02C
#define PIO_IRQ             0x030
#define PIO_IRQ_FORCE       0x034
#define PIO_INPUT_SYNC_BYPASS 0x038
#define PIO_DBG_PADOUT      0x03C
#define PIO_DBG_PADOE       0x040
#define PIO_DBG_CFGINFO     0x044
#define PIO_INSTR_MEM0      0x048  /* 32 words: 0x048-0x0C4 */

/* Per-SM register offsets (SM0 base = 0x0C8, stride = 0x18) */
#define PIO_SM0_CLKDIV      0x0C8
#define PIO_SM0_EXECCTRL     0x0CC
#define PIO_SM0_SHIFTCTRL    0x0D0
#define PIO_SM0_ADDR         0x0D4
#define PIO_SM0_INSTR        0x0D8
#define PIO_SM0_PINCTRL      0x0DC
#define PIO_SM_STRIDE        0x018  /* 6 registers * 4 bytes */

/* Interrupt registers */
#define PIO_INTR            0x128
#define PIO_IRQ0_INTE       0x12C
#define PIO_IRQ0_INTF       0x130
#define PIO_IRQ0_INTS       0x134
#define PIO_IRQ1_INTE       0x138
#define PIO_IRQ1_INTF       0x13C
#define PIO_IRQ1_INTS       0x140

/* ========================================================================
 * CTRL bits
 * ======================================================================== */

#define PIO_CTRL_SM_ENABLE_MASK     0x0F
#define PIO_CTRL_SM_RESTART_MASK    (0x0F << 4)
#define PIO_CTRL_CLKDIV_RESTART_MASK (0x0F << 8)

/* ========================================================================
 * FSTAT bits (per-SM FIFO status, 4 bits each)
 * ======================================================================== */

#define PIO_FSTAT_TXFULL_SHIFT  16
#define PIO_FSTAT_TXEMPTY_SHIFT 24
#define PIO_FSTAT_RXFULL_SHIFT  0
#define PIO_FSTAT_RXEMPTY_SHIFT 8

/* ========================================================================
 * Per-SM State
 * ======================================================================== */

typedef struct {
    uint32_t clkdiv;
    uint32_t execctrl;
    uint32_t shiftctrl;
    uint32_t addr;
    uint32_t instr;
    uint32_t pinctrl;
} pio_sm_t;

/* ========================================================================
 * Per-PIO Block State
 * ======================================================================== */

typedef struct {
    uint32_t ctrl;
    uint32_t fdebug;
    uint32_t irq;
    uint32_t irq_force;
    uint32_t input_sync_bypass;

    /* Instruction memory */
    uint32_t instr_mem[PIO_INSTR_MEM_SIZE];

    /* State machines */
    pio_sm_t sm[PIO_NUM_SM];

    /* Interrupt registers */
    uint32_t irq0_inte;
    uint32_t irq0_intf;
    uint32_t irq1_inte;
    uint32_t irq1_intf;
} pio_block_t;

extern pio_block_t pio_state[PIO_NUM_BLOCKS];

/* ========================================================================
 * API
 * ======================================================================== */

void     pio_init(void);
int      pio_match(uint32_t addr);  /* Returns 0/1 for PIO block, -1 if not */
uint32_t pio_read32(int pio_num, uint32_t offset);
void     pio_write32(int pio_num, uint32_t offset, uint32_t val);

#endif /* PIO_H */
