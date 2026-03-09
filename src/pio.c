/*
 * RP2040 PIO (Programmable I/O) Emulation
 *
 * Full PIO implementation with instruction execution:
 * - 9 PIO instructions: JMP, WAIT, IN, OUT, PUSH, PULL, MOV, IRQ, SET
 * - Per-SM: ISR, OSR, X, Y scratch registers, shift counters
 * - 4-deep TX/RX FIFOs per state machine
 * - EXECCTRL wrap control, SHIFTCTRL autopush/autopull
 * - GPIO pin interaction via gpio.h
 * - Instruction memory is writable and readable
 * - Per-SM registers (CLKDIV, EXECCTRL, SHIFTCTRL, PINCTRL) are stored
 * - CTRL SM_ENABLE bits control which SMs are running
 */

#include <string.h>
#include <stdio.h>
#include "pio.h"
#include "gpio.h"

pio_block_t pio_state[PIO_NUM_BLOCKS];

/* ========================================================================
 * FIFO Helpers
 * ======================================================================== */

static int fifo_push(pio_fifo_t *f, uint32_t val) {
    if (f->count >= PIO_FIFO_DEPTH) return 0;
    f->data[f->wr] = val;
    f->wr = (f->wr + 1) % PIO_FIFO_DEPTH;
    f->count++;
    return 1;
}

static int fifo_pop(pio_fifo_t *f, uint32_t *val) {
    if (f->count == 0) return 0;
    *val = f->data[f->rd];
    f->rd = (f->rd + 1) % PIO_FIFO_DEPTH;
    f->count--;
    return 1;
}

static int fifo_empty(pio_fifo_t *f) { return f->count == 0; }
static int fifo_full(pio_fifo_t *f)  { return f->count >= PIO_FIFO_DEPTH; }

/* ========================================================================
 * EXECCTRL / SHIFTCTRL field extraction helpers
 * ======================================================================== */

static uint8_t sm_wrap_bottom(pio_sm_t *s) {
    return (s->execctrl >> 7) & 0x1F;
}

static uint8_t sm_wrap_top(pio_sm_t *s) {
    return (s->execctrl >> 12) & 0x1F;
}

static int sm_out_shift_right(pio_sm_t *s) {
    return (s->shiftctrl >> 19) & 1;
}

static int sm_in_shift_right(pio_sm_t *s) {
    return (s->shiftctrl >> 18) & 1;
}

static int sm_autopull(pio_sm_t *s) {
    return (s->shiftctrl >> 17) & 1;
}

static int sm_autopush(pio_sm_t *s) {
    return (s->shiftctrl >> 16) & 1;
}

static uint8_t sm_pull_thresh(pio_sm_t *s) {
    uint8_t t = (s->shiftctrl >> 25) & 0x1F;
    return t == 0 ? 32 : t;
}

static uint8_t sm_push_thresh(pio_sm_t *s) {
    uint8_t t = (s->shiftctrl >> 20) & 0x1F;
    return t == 0 ? 32 : t;
}

/* PINCTRL field extraction */
static uint8_t sm_set_count(pio_sm_t *s) {
    return (s->pinctrl >> 26) & 0x07;
}

static uint8_t sm_set_base(pio_sm_t *s) {
    return (s->pinctrl >> 5) & 0x1F;
}

static uint8_t sm_out_count(pio_sm_t *s) {
    return (s->pinctrl >> 20) & 0x3F;
}

static uint8_t sm_out_base(pio_sm_t *s) {
    return (s->pinctrl >> 0) & 0x1F;
}

static uint8_t sm_in_base(pio_sm_t *s) {
    return (s->pinctrl >> 15) & 0x1F;
}

/* ========================================================================
 * GPIO Pin Helpers (read/write N pins from base)
 * ======================================================================== */

static uint32_t read_pins(uint8_t base, uint8_t count) {
    if (count == 0) return 0;
    uint32_t val = 0;
    for (int i = 0; i < count && i < 32; i++) {
        uint8_t pin = (base + i) % 30;
        if (gpio_get_pin(pin))
            val |= (1u << i);
    }
    return val;
}

static void write_pins(uint8_t base, uint8_t count, uint32_t val) {
    for (int i = 0; i < count && i < 32; i++) {
        uint8_t pin = (base + i) % 30;
        gpio_set_pin(pin, (val >> i) & 1);
    }
}

static void write_pindirs(uint8_t base, uint8_t count, uint32_t val) {
    for (int i = 0; i < count && i < 32; i++) {
        uint8_t pin = (base + i) % 30;
        gpio_set_direction(pin, (val >> i) & 1);
    }
}

/* ========================================================================
 * PIO SM PC Advance (with wrap)
 * ======================================================================== */

static void sm_advance_pc(pio_sm_t *s) {
    if (s->pc == sm_wrap_top(s))
        s->pc = sm_wrap_bottom(s);
    else
        s->pc = (s->pc + 1) & 0x1F;
}

/* ========================================================================
 * PIO Instruction Execution
 * ======================================================================== */

void pio_sm_exec(int pio_num, int sm_num, uint16_t instr) {
    pio_block_t *p = &pio_state[pio_num];
    pio_sm_t *s = &p->sm[sm_num];

    uint8_t opcode = (instr >> 13) & 0x07;
    /* delay/side-set in bits [12:8] — we ignore delay for now */
    uint8_t arg = instr & 0xFF;  /* Lower 8 bits */

    s->stalled = 0;

    switch (opcode) {

    /* ----------------------------------------------------------------
     * JMP: condition, address
     * [7:5] = condition, [4:0] = address
     * ---------------------------------------------------------------- */
    case PIO_OP_JMP: {
        uint8_t cond = (arg >> 5) & 0x07;
        uint8_t addr = arg & 0x1F;
        int take = 0;
        switch (cond) {
        case 0: take = 1; break;                    /* always */
        case 1: take = (s->x == 0); break;          /* !X (X is zero) */
        case 2: take = (s->x != 0); s->x--; break;  /* X-- (post-decrement, jump if nonzero before) */
        case 3: take = (s->y == 0); break;          /* !Y */
        case 4: take = (s->y != 0); s->y--; break;  /* Y-- */
        case 5: take = (s->x != s->y); break;       /* X!=Y */
        case 6: {
            /* PIN: jump on input pin */
            uint8_t jmp_pin = (s->execctrl >> 24) & 0x1F;
            take = gpio_get_pin(jmp_pin) ? 1 : 0;
            break;
        }
        case 7: take = (s->osr_count < sm_pull_thresh(s)); break; /* !OSRE (OSR not empty) */
        }
        if (take) {
            s->pc = addr;
        } else {
            sm_advance_pc(s);
        }
        return;  /* PC already set */
    }

    /* ----------------------------------------------------------------
     * WAIT: polarity, source, index
     * [7] = polarity, [6:5] = source, [4:0] = index
     * ---------------------------------------------------------------- */
    case PIO_OP_WAIT: {
        uint8_t polarity = (arg >> 7) & 1;
        uint8_t source = (arg >> 5) & 0x03;
        uint8_t index = arg & 0x1F;
        int condition_met = 0;

        switch (source) {
        case 0: /* GPIO (absolute pin number) */
            condition_met = (gpio_get_pin(index % 30) == polarity);
            break;
        case 1: /* PIN (relative to IN_BASE) */
            condition_met = (gpio_get_pin((sm_in_base(s) + index) % 30) == polarity);
            break;
        case 2: /* IRQ flag */
            condition_met = ((p->irq >> (index & 0x07)) & 1) == polarity;
            if (condition_met && polarity) {
                p->irq &= ~(1u << (index & 0x07));  /* Auto-clear on wait */
            }
            break;
        default:
            condition_met = 1;
            break;
        }

        if (!condition_met) {
            s->stalled = 1;
            return;  /* Don't advance PC */
        }
        break;
    }

    /* ----------------------------------------------------------------
     * IN: source, bit_count
     * [7:5] = source, [4:0] = bit_count (0 means 32)
     * ---------------------------------------------------------------- */
    case PIO_OP_IN: {
        uint8_t source = (arg >> 5) & 0x07;
        uint8_t bit_count = arg & 0x1F;
        if (bit_count == 0) bit_count = 32;

        uint32_t data = 0;
        switch (source) {
        case 0: data = read_pins(sm_in_base(s), bit_count); break;  /* PINS */
        case 1: data = s->x; break;           /* X */
        case 2: data = s->y; break;           /* Y */
        case 3: data = 0; break;              /* NULL (zeros) */
        case 6: data = s->isr; break;         /* ISR */
        case 7: data = s->osr; break;         /* OSR */
        default: data = 0; break;
        }

        /* Mask to bit_count bits */
        if (bit_count < 32) data &= (1u << bit_count) - 1;

        /* Shift into ISR */
        if (sm_in_shift_right(s)) {
            s->isr >>= bit_count;
            s->isr |= data << (32 - bit_count);
        } else {
            s->isr <<= bit_count;
            s->isr |= data;
        }
        s->isr_count += bit_count;
        if (s->isr_count > 32) s->isr_count = 32;

        /* Autopush check */
        if (sm_autopush(s) && s->isr_count >= sm_push_thresh(s)) {
            if (fifo_push(&s->rx_fifo, s->isr)) {
                s->isr = 0;
                s->isr_count = 0;
            }
        }
        break;
    }

    /* ----------------------------------------------------------------
     * OUT: destination, bit_count
     * [7:5] = destination, [4:0] = bit_count (0 means 32)
     * ---------------------------------------------------------------- */
    case PIO_OP_OUT: {
        uint8_t dest = (arg >> 5) & 0x07;
        uint8_t bit_count = arg & 0x1F;
        if (bit_count == 0) bit_count = 32;

        /* Autopull check: refill OSR if empty */
        if (sm_autopull(s) && s->osr_count >= sm_pull_thresh(s)) {
            uint32_t val;
            if (fifo_pop(&s->tx_fifo, &val)) {
                s->osr = val;
                s->osr_count = 0;
            } else {
                s->stalled = 1;
                return;
            }
        }

        /* Extract data from OSR */
        uint32_t data;
        if (sm_out_shift_right(s)) {
            data = s->osr;
            if (bit_count < 32) data &= (1u << bit_count) - 1;
            s->osr >>= bit_count;
        } else {
            data = s->osr >> (32 - bit_count);
            s->osr <<= bit_count;
        }
        s->osr_count += bit_count;

        /* Route data to destination */
        switch (dest) {
        case 0: write_pins(sm_out_base(s), sm_out_count(s), data); break;  /* PINS */
        case 1: s->x = data; break;
        case 2: s->y = data; break;
        case 3: /* NULL (discard) */ break;
        case 4: write_pindirs(sm_out_base(s), sm_out_count(s), data); break;  /* PINDIRS */
        case 5: /* PC */
            s->pc = data & 0x1F;
            return;
        case 6: s->isr = data; s->isr_count = bit_count; break;
        case 7: /* EXEC: execute data as instruction */
            pio_sm_exec(pio_num, sm_num, (uint16_t)data);
            return;
        }
        break;
    }

    /* ----------------------------------------------------------------
     * PUSH / PULL
     * [7] = 0 for PUSH, 1 for PULL
     * [6] = if_full/if_empty flag
     * [5] = block flag
     * ---------------------------------------------------------------- */
    case PIO_OP_PUSH_PULL: {
        int is_pull = (arg >> 7) & 1;
        int ife = (arg >> 6) & 1;   /* if_full (push) / if_empty (pull) */
        int block = (arg >> 5) & 1;

        if (is_pull) {
            /* PULL: TX FIFO → OSR */
            if (ife && s->osr_count < sm_pull_thresh(s)) {
                /* if_empty: only pull if OSR is empty (shift count reached threshold) */
                break;
            }
            uint32_t val;
            if (fifo_pop(&s->tx_fifo, &val)) {
                s->osr = val;
                s->osr_count = 0;
            } else if (block) {
                s->stalled = 1;
                return;
            } else {
                /* Non-blocking: copy X to OSR */
                s->osr = s->x;
                s->osr_count = 0;
            }
        } else {
            /* PUSH: ISR → RX FIFO */
            if (ife && s->isr_count < sm_push_thresh(s)) {
                /* if_full: only push if ISR is full (reached threshold) */
                break;
            }
            if (fifo_push(&s->rx_fifo, s->isr)) {
                s->isr = 0;
                s->isr_count = 0;
            } else if (block) {
                s->stalled = 1;
                return;
            }
            /* Non-blocking + full: data lost */
        }
        break;
    }

    /* ----------------------------------------------------------------
     * MOV: destination, op, source
     * [7:5] = destination, [4:3] = operation, [2:0] = source
     * ---------------------------------------------------------------- */
    case PIO_OP_MOV: {
        uint8_t dest = (arg >> 5) & 0x07;
        uint8_t op = (arg >> 3) & 0x03;
        uint8_t source = arg & 0x07;

        /* Read source */
        uint32_t val = 0;
        switch (source) {
        case 0: val = read_pins(sm_in_base(s), 32); break;  /* PINS */
        case 1: val = s->x; break;
        case 2: val = s->y; break;
        case 3: val = 0; break;              /* NULL */
        case 5: val = 0; break;  /* STATUS (FIFO level comparison, simplified) */
        case 6: val = s->isr; break;
        case 7: val = s->osr; break;
        default: val = 0; break;
        }

        /* Apply operation */
        switch (op) {
        case 0: /* None */ break;
        case 1: val = ~val; break;          /* Invert */
        case 2: {                           /* Bit-reverse */
            uint32_t r = 0;
            for (int i = 0; i < 32; i++)
                if (val & (1u << i)) r |= (1u << (31 - i));
            val = r;
            break;
        }
        default: break;
        }

        /* Write destination */
        switch (dest) {
        case 0: write_pins(sm_out_base(s), sm_out_count(s), val); break;  /* PINS */
        case 1: s->x = val; break;
        case 2: s->y = val; break;
        case 4: /* EXEC */
            pio_sm_exec(pio_num, sm_num, (uint16_t)val);
            return;
        case 5: s->pc = val & 0x1F; return;  /* PC */
        case 6: s->isr = val; break;
        case 7: s->osr = val; break;
        default: break;
        }
        break;
    }

    /* ----------------------------------------------------------------
     * IRQ: clear, wait, index
     * [7] = unused, [6] = clear, [5] = wait, [4:0] = index
     * ---------------------------------------------------------------- */
    case PIO_OP_IRQ: {
        int clr = (arg >> 6) & 1;
        int wait = (arg >> 5) & 1;
        uint8_t index = arg & 0x1F;

        /* Relative IRQ: bit 4 set means index = (irq_num + sm_num) % 4 */
        uint8_t irq_num = index & 0x07;
        if (index & 0x10) {
            irq_num = ((index & 0x03) + sm_num) & 0x03;
        }

        if (clr) {
            p->irq &= ~(1u << irq_num);
        } else {
            p->irq |= (1u << irq_num);
            if (wait) {
                /* Wait for IRQ to be cleared */
                if (p->irq & (1u << irq_num)) {
                    s->stalled = 1;
                    return;
                }
            }
        }
        break;
    }

    /* ----------------------------------------------------------------
     * SET: destination, data
     * [7:5] = destination, [4:0] = data (5-bit immediate)
     * ---------------------------------------------------------------- */
    case PIO_OP_SET: {
        uint8_t dest = (arg >> 5) & 0x07;
        uint32_t data = arg & 0x1F;

        switch (dest) {
        case 0: write_pins(sm_set_base(s), sm_set_count(s), data); break;    /* PINS */
        case 1: write_pindirs(sm_set_base(s), sm_set_count(s), data); break; /* PINDIRS */
        case 5: s->x = data; break;
        case 6: s->y = data; break;
        default: break;
        }
        break;
    }

    } /* switch opcode */

    /* Advance PC (unless already set by JMP/OUT PC/MOV PC) */
    sm_advance_pc(s);
}

/* ========================================================================
 * PIO Step: execute one cycle for all enabled SMs
 * ======================================================================== */

void pio_step(void) {
    for (int b = 0; b < PIO_NUM_BLOCKS; b++) {
        pio_block_t *p = &pio_state[b];
        for (int sm = 0; sm < PIO_NUM_SM; sm++) {
            if (!(p->ctrl & (1u << sm))) continue;  /* SM not enabled */
            pio_sm_t *s = &p->sm[sm];

            /* Check for forced execution (SM_INSTR write) — bypasses clock divider */
            if (s->exec_pending) {
                s->exec_pending = 0;
                pio_sm_exec(b, sm, s->instr & 0xFFFF);
                continue;
            }

            /* Clock division: CLKDIV register is INT[31:16].FRAC[15:8] */
            uint16_t clk_int = (s->clkdiv >> 16) & 0xFFFF;
            uint8_t clk_frac = (s->clkdiv >> 8) & 0xFF;

            /* INT=0 means 65536 per RP2040 spec; INT=1,FRAC=0 means every cycle */
            if (clk_int != 1 || clk_frac != 0) {
                uint32_t effective_int = (clk_int == 0) ? 65536u : clk_int;
                uint32_t divisor_fp = (effective_int << 8) | clk_frac;

                s->clk_frac_acc += 256;  /* Add 1.0 in 16.8 fixed-point */
                if (s->clk_frac_acc < divisor_fp) {
                    continue;  /* Not time to execute yet */
                }
                s->clk_frac_acc -= divisor_fp;
            }

            /* Skip if stalled */
            if (s->stalled) {
                /* Re-execute same instruction to re-check condition */
                uint16_t instr = p->instr_mem[s->pc] & 0xFFFF;
                pio_sm_exec(b, sm, instr);
                continue;
            }

            /* Fetch and execute from instruction memory */
            uint16_t instr = p->instr_mem[s->pc] & 0xFFFF;
            s->addr = s->pc;  /* Update addr register (visible to CPU) */
            pio_sm_exec(b, sm, instr);
        }
    }
}

/* ========================================================================
 * Init
 * ======================================================================== */

void pio_init(void) {
    memset(pio_state, 0, sizeof(pio_state));
    for (int b = 0; b < PIO_NUM_BLOCKS; b++) {
        for (int sm = 0; sm < PIO_NUM_SM; sm++) {
            /* Default CLKDIV: INT=1, FRAC=0 (run every cycle) */
            pio_state[b].sm[sm].clkdiv = (1u << 16);
            /* Default EXECCTRL: WRAP_TOP=31, STATUS_SEL=0 */
            pio_state[b].sm[sm].execctrl = (31u << 12);  /* wrap_top=31 */
            /* Default SHIFTCTRL: PULL_THRESH=0, PUSH_THRESH=0, both autopush/pull off */
            pio_state[b].sm[sm].shiftctrl = (1u << 18) | (1u << 19);  /* IN/OUT shift right */
        }
    }
}

/* ========================================================================
 * Address Matching
 * ======================================================================== */

int pio_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;  /* Strip atomic alias bits */
    if (base >= PIO0_BASE && base < PIO0_BASE + PIO_BLOCK_SIZE)
        return 0;
    if (base >= PIO1_BASE && base < PIO1_BASE + PIO_BLOCK_SIZE)
        return 1;
    return -1;
}

/* ========================================================================
 * Register Read
 * ======================================================================== */

uint32_t pio_read32(int pio_num, uint32_t offset) {
    pio_block_t *p = &pio_state[pio_num];

    switch (offset) {
    case PIO_CTRL:
        return p->ctrl;

    case PIO_FSTAT: {
        /* Build FSTAT from actual FIFO state */
        uint32_t fstat = 0;
        for (int sm = 0; sm < PIO_NUM_SM; sm++) {
            if (fifo_full(&p->sm[sm].tx_fifo))
                fstat |= (1u << (PIO_FSTAT_TXFULL_SHIFT + sm));
            if (fifo_empty(&p->sm[sm].tx_fifo))
                fstat |= (1u << (PIO_FSTAT_TXEMPTY_SHIFT + sm));
            if (fifo_full(&p->sm[sm].rx_fifo))
                fstat |= (1u << (PIO_FSTAT_RXFULL_SHIFT + sm));
            if (fifo_empty(&p->sm[sm].rx_fifo))
                fstat |= (1u << (PIO_FSTAT_RXEMPTY_SHIFT + sm));
        }
        return fstat;
    }

    case PIO_FDEBUG:
        return p->fdebug;

    case PIO_FLEVEL: {
        /* 4 bits per SM: [TX_level:RX_level], SM0 in lowest bits */
        uint32_t flevel = 0;
        for (int sm = 0; sm < PIO_NUM_SM; sm++) {
            uint32_t tx_lvl = p->sm[sm].tx_fifo.count & 0x0F;
            uint32_t rx_lvl = p->sm[sm].rx_fifo.count & 0x0F;
            flevel |= (tx_lvl << (sm * 8)) | (rx_lvl << (sm * 8 + 4));
        }
        return flevel;
    }

    /* TX FIFOs: write-only, reading returns 0 */
    case PIO_TXF0: case PIO_TXF1: case PIO_TXF2: case PIO_TXF3:
        return 0;

    /* RX FIFOs: pop from SM's RX FIFO */
    case PIO_RXF0: case PIO_RXF1: case PIO_RXF2: case PIO_RXF3: {
        int sm = (offset - PIO_RXF0) / 4;
        uint32_t val = 0;
        fifo_pop(&p->sm[sm].rx_fifo, &val);
        return val;
    }

    case PIO_IRQ:
        return p->irq;

    case PIO_IRQ_FORCE:
        return p->irq_force;

    case PIO_INPUT_SYNC_BYPASS:
        return p->input_sync_bypass;

    case PIO_DBG_PADOUT:
        return 0;

    case PIO_DBG_PADOE:
        return 0;

    case PIO_DBG_CFGINFO:
        return (PIO_FIFO_DEPTH << 16) | (PIO_INSTR_MEM_SIZE << 8) | (PIO_NUM_SM << 0);

    /* Interrupt registers */
    case PIO_INTR:
        return p->irq;  /* Raw interrupt flags */
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
        case 0x0C: return s->pc;  /* ADDR = current PC */
        case 0x10: return s->instr;
        case 0x14: return s->pinctrl;
        default: return 0;
        }
    }

    return 0;
}

/* ========================================================================
 * Register Write
 * ======================================================================== */

void pio_write32(int pio_num, uint32_t offset, uint32_t val) {
    pio_block_t *p = &pio_state[pio_num];

    switch (offset) {
    case PIO_CTRL: {
        /* SM_ENABLE bits [3:0] */
        p->ctrl = val & PIO_CTRL_SM_ENABLE_MASK;

        /* SM_RESTART [7:4]: reset SMs (strobe, self-clearing) */
        uint8_t restart = (val >> 4) & 0x0F;
        for (int sm = 0; sm < PIO_NUM_SM; sm++) {
            if (restart & (1u << sm)) {
                pio_sm_t *s = &p->sm[sm];
                s->pc = 0;
                s->stalled = 0;
                s->x = 0;
                s->y = 0;
                s->isr = 0;
                s->osr = 0;
                s->isr_count = 0;
                s->osr_count = 0;
                s->exec_pending = 0;
                s->clk_frac_acc = 0;
                memset(&s->tx_fifo, 0, sizeof(pio_fifo_t));
                memset(&s->rx_fifo, 0, sizeof(pio_fifo_t));
            }
        }

        /* CLKDIV_RESTART [11:8]: restart clock dividers (strobe, self-clearing) */
        uint8_t clkdiv_restart = (val >> 8) & 0x0F;
        for (int sm = 0; sm < PIO_NUM_SM; sm++) {
            if (clkdiv_restart & (1u << sm)) {
                p->sm[sm].clk_frac_acc = 0;
            }
        }
        break;
    }

    case PIO_FSTAT:
        break;  /* Read-only */

    case PIO_FDEBUG:
        p->fdebug &= ~val;  /* Write-1-to-clear */
        break;

    /* TX FIFOs: push into SM's TX FIFO */
    case PIO_TXF0: case PIO_TXF1: case PIO_TXF2: case PIO_TXF3: {
        int sm = (offset - PIO_TXF0) / 4;
        fifo_push(&p->sm[sm].tx_fifo, val);
        break;
    }

    case PIO_IRQ:
        p->irq &= ~(val & 0xFF);  /* Write-1-to-clear */
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
        p->instr_mem[idx] = val & 0xFFFF;
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
        case 0x10:
            /* Writing SM_INSTR triggers forced execution */
            s->instr = val & 0xFFFF;
            s->exec_pending = 1;
            break;
        case 0x14: s->pinctrl = val; break;
        default: break;
        }
    }
}
