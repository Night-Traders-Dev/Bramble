/*
 * RP2040 PIO (Programmable I/O) Emulation
 *
 * Register-level implementation allowing SDK code to configure PIO
 * without crashing. State machines do not execute PIO instructions.
 *
 * - FSTAT reports all TX FIFOs empty, all RX FIFOs empty
 * - TX FIFO writes are accepted and discarded
 * - RX FIFO reads return 0
 * - Instruction memory is writable and readable
 * - Per-SM registers (CLKDIV, EXECCTRL, SHIFTCTRL, PINCTRL) are stored
 * - CTRL SM_ENABLE bits are tracked
 * - DBG_CFGINFO returns correct RP2040 values
 */

#include <string.h>
#include "pio.h"

pio_block_t pio_state[PIO_NUM_BLOCKS];

void pio_init(void) {
    memset(pio_state, 0, sizeof(pio_state));
    for (int b = 0; b < PIO_NUM_BLOCKS; b++) {
        for (int sm = 0; sm < PIO_NUM_SM; sm++) {
            /* Default EXECCTRL: WRAP_TOP=31, STATUS_SEL=0 */
            pio_state[b].sm[sm].execctrl = (31u << 12);  /* wrap_top=31 */
            /* Default SHIFTCTRL: PULL_THRESH=0, PUSH_THRESH=0, both autopush/pull off */
            pio_state[b].sm[sm].shiftctrl = (1u << 18) | (1u << 19);  /* IN/OUT shift right */
        }
    }
}

int pio_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;  /* Strip atomic alias bits */
    if (base >= PIO0_BASE && base < PIO0_BASE + PIO_BLOCK_SIZE)
        return 0;
    if (base >= PIO1_BASE && base < PIO1_BASE + PIO_BLOCK_SIZE)
        return 1;
    return -1;
}

uint32_t pio_read32(int pio_num, uint32_t offset) {
    pio_block_t *p = &pio_state[pio_num];

    switch (offset) {
    case PIO_CTRL:
        return p->ctrl;

    case PIO_FSTAT:
        /* All TX FIFOs empty, all RX FIFOs empty */
        return (0x0F << PIO_FSTAT_TXEMPTY_SHIFT) |
               (0x0F << PIO_FSTAT_RXEMPTY_SHIFT);

    case PIO_FDEBUG:
        return p->fdebug;

    case PIO_FLEVEL:
        /* All FIFOs at level 0 */
        return 0;

    /* TX FIFOs: write-only, reading returns 0 */
    case PIO_TXF0: case PIO_TXF1: case PIO_TXF2: case PIO_TXF3:
        return 0;

    /* RX FIFOs: no data, return 0 */
    case PIO_RXF0: case PIO_RXF1: case PIO_RXF2: case PIO_RXF3:
        return 0;

    case PIO_IRQ:
        return p->irq;

    case PIO_IRQ_FORCE:
        return p->irq_force;

    case PIO_INPUT_SYNC_BYPASS:
        return p->input_sync_bypass;

    case PIO_DBG_PADOUT:
        return 0;  /* No GPIO output */

    case PIO_DBG_PADOE:
        return 0;  /* No GPIO output enable */

    case PIO_DBG_CFGINFO:
        /* RP2040: 4 state machines, 32 instruction memory, 4-deep FIFOs */
        return (4 << 16) | (PIO_INSTR_MEM_SIZE << 8) | (PIO_NUM_SM << 0);

    /* Interrupt registers */
    case PIO_INTR:
        return 0;  /* No interrupts active */
    case PIO_IRQ0_INTE:
        return p->irq0_inte;
    case PIO_IRQ0_INTF:
        return p->irq0_intf;
    case PIO_IRQ0_INTS:
        return p->irq0_intf & p->irq0_inte;
    case PIO_IRQ1_INTE:
        return p->irq1_inte;
    case PIO_IRQ1_INTF:
        return p->irq1_intf;
    case PIO_IRQ1_INTS:
        return p->irq1_intf & p->irq1_inte;

    default:
        break;
    }

    /* Instruction memory: 0x048 - 0x0C4 (32 words) */
    if (offset >= PIO_INSTR_MEM0 && offset < PIO_INSTR_MEM0 + PIO_INSTR_MEM_SIZE * 4) {
        int idx = (offset - PIO_INSTR_MEM0) / 4;
        return p->instr_mem[idx];
    }

    /* Per-SM registers: SM0 at 0x0C8, stride 0x18, 4 SMs */
    if (offset >= PIO_SM0_CLKDIV && offset < PIO_SM0_CLKDIV + PIO_NUM_SM * PIO_SM_STRIDE) {
        int sm = (offset - PIO_SM0_CLKDIV) / PIO_SM_STRIDE;
        int reg = (offset - PIO_SM0_CLKDIV) % PIO_SM_STRIDE;
        pio_sm_t *s = &p->sm[sm];

        switch (reg) {
        case 0x00: return s->clkdiv;
        case 0x04: return s->execctrl;
        case 0x08: return s->shiftctrl;
        case 0x0C: return s->addr;
        case 0x10: return s->instr;
        case 0x14: return s->pinctrl;
        default: return 0;
        }
    }

    return 0;
}

void pio_write32(int pio_num, uint32_t offset, uint32_t val) {
    pio_block_t *p = &pio_state[pio_num];

    switch (offset) {
    case PIO_CTRL:
        /* SM_RESTART and CLKDIV_RESTART are self-clearing (strobe) */
        p->ctrl = val & PIO_CTRL_SM_ENABLE_MASK;
        break;

    case PIO_FSTAT:
        /* Read-only */
        break;

    case PIO_FDEBUG:
        /* Write-1-to-clear */
        p->fdebug &= ~val;
        break;

    /* TX FIFOs: accept and discard */
    case PIO_TXF0: case PIO_TXF1: case PIO_TXF2: case PIO_TXF3:
        break;

    case PIO_IRQ:
        /* Write-1-to-clear */
        p->irq &= ~(val & 0xFF);
        break;

    case PIO_IRQ_FORCE:
        p->irq_force = val & 0xFF;
        p->irq |= val & 0xFF;
        break;

    case PIO_INPUT_SYNC_BYPASS:
        p->input_sync_bypass = val;
        break;

    case PIO_IRQ0_INTE:
        p->irq0_inte = val & 0xFFF;
        break;
    case PIO_IRQ0_INTF:
        p->irq0_intf = val & 0xFFF;
        break;
    case PIO_IRQ1_INTE:
        p->irq1_inte = val & 0xFFF;
        break;
    case PIO_IRQ1_INTF:
        p->irq1_intf = val & 0xFFF;
        break;

    default:
        break;
    }

    /* Instruction memory */
    if (offset >= PIO_INSTR_MEM0 && offset < PIO_INSTR_MEM0 + PIO_INSTR_MEM_SIZE * 4) {
        int idx = (offset - PIO_INSTR_MEM0) / 4;
        p->instr_mem[idx] = val & 0xFFFF;  /* PIO instructions are 16-bit */
        return;
    }

    /* Per-SM registers */
    if (offset >= PIO_SM0_CLKDIV && offset < PIO_SM0_CLKDIV + PIO_NUM_SM * PIO_SM_STRIDE) {
        int sm = (offset - PIO_SM0_CLKDIV) / PIO_SM_STRIDE;
        int reg = (offset - PIO_SM0_CLKDIV) % PIO_SM_STRIDE;
        pio_sm_t *s = &p->sm[sm];

        switch (reg) {
        case 0x00: s->clkdiv = val; break;
        case 0x04: s->execctrl = val; break;
        case 0x08: s->shiftctrl = val; break;
        case 0x0C: /* ADDR is read-only */ break;
        case 0x10: s->instr = val & 0xFFFF; break;
        case 0x14: s->pinctrl = val; break;
        default: break;
        }
    }
}
