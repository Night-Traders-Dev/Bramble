/*
 * RP2040 DMA Controller Emulation
 *
 * Implements 12 DMA channels with immediate (synchronous) transfers.
 * When a channel is triggered (CTRL_TRIG written with EN=1, or via alias
 * trigger registers), the transfer executes immediately within the write call.
 *
 * Supports:
 * - READ_ADDR / WRITE_ADDR with optional increment
 * - DATA_SIZE: byte, halfword, word
 * - TRANS_COUNT transfers per trigger
 * - CHAIN_TO: automatically triggers another channel on completion
 * - IRQ_QUIET: suppress interrupt on completion
 * - Global INTR/INTE/INTF/INTS interrupt registers
 * - All 4 alias register layouts per channel
 */

#include <string.h>
#include "dma.h"
#include "emulator.h"

dma_state_t dma_state;

/* ========================================================================
 * Initialization
 * ======================================================================== */

void dma_init(void) {
    memset(&dma_state, 0, sizeof(dma_state));
    /* Default CHAIN_TO = self (no chaining) for each channel */
    for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
        dma_state.ch[i].ctrl = (uint32_t)i << DMA_CTRL_CHAIN_TO_SHIFT;
    }
}

/* ========================================================================
 * Address Matching
 * ======================================================================== */

int dma_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;  /* Strip atomic alias bits */
    return (base >= DMA_BASE && base < DMA_BASE + DMA_BLOCK_SIZE) ? 1 : 0;
}

/* ========================================================================
 * DMA Transfer Engine
 *
 * Performs an immediate synchronous transfer for the given channel.
 * This is called when a trigger register is written.
 * ======================================================================== */

static void dma_do_transfer(int ch_idx) {
    dma_channel_t *c = &dma_state.ch[ch_idx];

    if (!(c->ctrl & DMA_CTRL_EN))
        return;

    uint32_t count = c->trans_count;
    if (count == 0)
        return;

    int data_size = (c->ctrl & DMA_CTRL_DATA_SIZE_MASK) >> DMA_CTRL_DATA_SIZE_SHIFT;
    int incr_read  = (c->ctrl & DMA_CTRL_INCR_READ)  ? 1 : 0;
    int incr_write = (c->ctrl & DMA_CTRL_INCR_WRITE) ? 1 : 0;

    uint32_t src = c->read_addr;
    uint32_t dst = c->write_addr;
    uint32_t step;

    switch (data_size) {
    case DMA_SIZE_BYTE:     step = 1; break;
    case DMA_SIZE_HALFWORD: step = 2; break;
    default:                step = 4; break;  /* DMA_SIZE_WORD */
    }

    for (uint32_t i = 0; i < count; i++) {
        switch (data_size) {
        case DMA_SIZE_BYTE: {
            uint8_t val = mem_read8(src);
            mem_write8(dst, val);
            break;
        }
        case DMA_SIZE_HALFWORD: {
            uint16_t val = mem_read16(src);
            mem_write16(dst, val);
            break;
        }
        default: {  /* WORD */
            uint32_t val = mem_read32(src);
            mem_write32(dst, val);
            break;
        }
        }

        if (incr_read)  src += step;
        if (incr_write) dst += step;
    }

    /* Update addresses to final positions */
    c->read_addr = src;
    c->write_addr = dst;
    c->trans_count = 0;  /* Transfer complete */

    /* Set interrupt if not IRQ_QUIET */
    if (!(c->ctrl & DMA_CTRL_IRQ_QUIET)) {
        dma_state.intr |= (1u << ch_idx);
    }

    /* Chain: if CHAIN_TO != self, trigger the chained channel */
    int chain_to = (c->ctrl & DMA_CTRL_CHAIN_TO_MASK) >> DMA_CTRL_CHAIN_TO_SHIFT;
    if (chain_to != ch_idx && chain_to < DMA_NUM_CHANNELS) {
        dma_do_transfer(chain_to);
    }
}

/* ========================================================================
 * Channel Register Access Helpers
 *
 * Each channel has 4 alias layouts (0x00-0x3F within the channel block).
 * All aliases access the same 4 fields (CTRL, READ_ADDR, WRITE_ADDR,
 * TRANS_COUNT) but in different orders. The last register in each alias
 * is the "trigger" register — writing it starts a transfer.
 *
 * Alias 0: READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL_TRIG
 * Alias 1: CTRL, READ_ADDR, WRITE_ADDR, TRANS_COUNT_TRIG
 * Alias 2: CTRL, TRANS_COUNT, READ_ADDR, WRITE_ADDR_TRIG
 * Alias 3: CTRL, WRITE_ADDR, TRANS_COUNT, READ_ADDR_TRIG
 * ======================================================================== */

/* Map alias register offset (0x00-0x3F) to field + is_trigger */
enum dma_field { F_CTRL, F_READ, F_WRITE, F_COUNT };

static void ch_field_write(int ch, enum dma_field f, uint32_t val, int trigger) {
    dma_channel_t *c = &dma_state.ch[ch];
    switch (f) {
    case F_CTRL:
        /* Preserve read-only bits; handle W1C for error flags */
        {
            uint32_t w1c = val & (DMA_CTRL_WRITE_ERROR | DMA_CTRL_READ_ERROR);
            c->ctrl &= ~w1c;  /* Clear error flags that are written as 1 */
            /* Write all other writable bits */
            uint32_t writable = ~(DMA_CTRL_BUSY | DMA_CTRL_AHB_ERROR |
                                  DMA_CTRL_WRITE_ERROR | DMA_CTRL_READ_ERROR);
            c->ctrl = (c->ctrl & ~writable) | (val & writable);
        }
        break;
    case F_READ:  c->read_addr = val; break;
    case F_WRITE: c->write_addr = val; break;
    case F_COUNT: c->trans_count = val; break;
    }
    if (trigger) {
        dma_do_transfer(ch);
    }
}

static uint32_t ch_field_read(int ch, enum dma_field f) {
    dma_channel_t *c = &dma_state.ch[ch];
    switch (f) {
    case F_CTRL:  return c->ctrl;
    case F_READ:  return c->read_addr;
    case F_WRITE: return c->write_addr;
    case F_COUNT: return c->trans_count;
    }
    return 0;
}

/* Alias layout tables: [alias][reg_within_alias] -> field */
static const enum dma_field alias_layout[4][4] = {
    /* Alias 0 */ { F_READ,  F_WRITE, F_COUNT, F_CTRL  },
    /* Alias 1 */ { F_CTRL,  F_READ,  F_WRITE, F_COUNT },
    /* Alias 2 */ { F_CTRL,  F_COUNT, F_READ,  F_WRITE },
    /* Alias 3 */ { F_CTRL,  F_WRITE, F_COUNT, F_READ  },
};

/* ========================================================================
 * Register Read
 * ======================================================================== */

uint32_t dma_read32(uint32_t offset) {
    /* Per-channel registers: 12 channels * 0x40 = 0x300 */
    if (offset < DMA_NUM_CHANNELS * DMA_CH_STRIDE) {
        int ch = offset / DMA_CH_STRIDE;
        int reg = offset % DMA_CH_STRIDE;
        int alias = reg / 0x10;         /* 0-3 */
        int field_idx = (reg % 0x10) / 4;  /* 0-3 */
        return ch_field_read(ch, alias_layout[alias][field_idx]);
    }

    /* Global registers */
    switch (offset) {
    case DMA_INTR:  return dma_state.intr;
    case DMA_INTE0: return dma_state.inte0;
    case DMA_INTF0: return dma_state.intf0;
    case DMA_INTS0: return (dma_state.intr | dma_state.intf0) & dma_state.inte0;
    case DMA_INTE1: return dma_state.inte1;
    case DMA_INTF1: return dma_state.intf1;
    case DMA_INTS1: return (dma_state.intr | dma_state.intf1) & dma_state.inte1;
    case DMA_TIMER0: return dma_state.timer[0];
    case DMA_TIMER1: return dma_state.timer[1];
    case DMA_TIMER2: return dma_state.timer[2];
    case DMA_TIMER3: return dma_state.timer[3];
    case DMA_MULTI_CHAN_TRIGGER: return 0;  /* Write-only */
    case DMA_SNIFF_CTRL: return dma_state.sniff_ctrl;
    case DMA_SNIFF_DATA: return dma_state.sniff_data;
    case DMA_FIFO_LEVELS: return 0;  /* All FIFOs empty */
    case DMA_CHAN_ABORT: return 0;    /* Write-only */
    case DMA_N_CHANNELS: return DMA_NUM_CHANNELS;
    default: return 0;
    }
}

/* ========================================================================
 * Register Write
 * ======================================================================== */

void dma_write32(uint32_t offset, uint32_t val) {
    /* Per-channel registers */
    if (offset < DMA_NUM_CHANNELS * DMA_CH_STRIDE) {
        int ch = offset / DMA_CH_STRIDE;
        int reg = offset % DMA_CH_STRIDE;
        int alias = reg / 0x10;
        int field_idx = (reg % 0x10) / 4;
        /* Last register in each alias block is the trigger */
        int is_trigger = (field_idx == 3);
        ch_field_write(ch, alias_layout[alias][field_idx], val, is_trigger);
        return;
    }

    /* Global registers */
    switch (offset) {
    case DMA_INTR:
        /* Write-1-to-clear */
        dma_state.intr &= ~val;
        break;
    case DMA_INTE0:
        dma_state.inte0 = val & ((1u << DMA_NUM_CHANNELS) - 1);
        break;
    case DMA_INTF0:
        dma_state.intf0 = val & ((1u << DMA_NUM_CHANNELS) - 1);
        break;
    case DMA_INTE1:
        dma_state.inte1 = val & ((1u << DMA_NUM_CHANNELS) - 1);
        break;
    case DMA_INTF1:
        dma_state.intf1 = val & ((1u << DMA_NUM_CHANNELS) - 1);
        break;
    case DMA_TIMER0: dma_state.timer[0] = val; break;
    case DMA_TIMER1: dma_state.timer[1] = val; break;
    case DMA_TIMER2: dma_state.timer[2] = val; break;
    case DMA_TIMER3: dma_state.timer[3] = val; break;
    case DMA_MULTI_CHAN_TRIGGER:
        /* Trigger all channels indicated by set bits */
        for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
            if (val & (1u << i)) {
                dma_do_transfer(i);
            }
        }
        break;
    case DMA_SNIFF_CTRL:
        dma_state.sniff_ctrl = val;
        break;
    case DMA_SNIFF_DATA:
        dma_state.sniff_data = val;
        break;
    case DMA_CHAN_ABORT:
        /* Abort channels - for our synchronous model, transfers are already complete.
         * Just clear trans_count for indicated channels. */
        for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
            if (val & (1u << i)) {
                dma_state.ch[i].trans_count = 0;
            }
        }
        break;
    default:
        break;
    }
}
