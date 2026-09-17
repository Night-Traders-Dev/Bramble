/*
 * RP2040 / RP2350 DMA Controller Emulation
 *
 * Implements DMA channels with immediate (synchronous) transfers.
 * When a channel is triggered (CTRL_TRIG written with EN=1, or via alias
 * trigger registers), the transfer executes immediately within the write call.
 *
 * RP2350 differences vs RP2040 (selected via membus_rp2350_mode):
 * - 16 channels (vs 12)
 * - CTRL_TRIG: INCR_WRITE, CHAIN_TO, BSWAP, IRQ_QUIET, BUSY bit positions
 * - Global regs MULTI_CHAN_TRIGGER / CHAN_ABORT / N_CHANNELS offsets
 */

#include <string.h>
#include "dma.h"
#include "emulator.h"
#include "nvic.h"

dma_state_t dma_state;

static int dma_n_channels(void) {
    return membus_rp2350_mode ? DMA_NUM_CHANNELS_RP2350 : DMA_NUM_CHANNELS_RP2040;
}

static uint32_t dma_ctrl_incr_write_bit(void) {
    return membus_rp2350_mode ? DMA_CTRL_INCR_WRITE_RP2350 : DMA_CTRL_INCR_WRITE;
}

static uint32_t dma_ctrl_bswap_bit(void) {
    return membus_rp2350_mode ? DMA_CTRL_BSWAP_RP2350 : DMA_CTRL_BSWAP;
}

static uint32_t dma_ctrl_irq_quiet_bit(void) {
    return membus_rp2350_mode ? DMA_CTRL_IRQ_QUIET_RP2350 : DMA_CTRL_IRQ_QUIET;
}

static uint32_t dma_ctrl_busy_bit(void) {
    return membus_rp2350_mode ? DMA_CTRL_BUSY_RP2350 : DMA_CTRL_BUSY;
}

static uint32_t dma_ctrl_chain_to_mask(void) {
    return membus_rp2350_mode ? DMA_CTRL_CHAIN_TO_MASK_RP2350 : DMA_CTRL_CHAIN_TO_MASK;
}

static int dma_ctrl_chain_to_shift(void) {
    return membus_rp2350_mode ? DMA_CTRL_CHAIN_TO_SHIFT_RP2350 : DMA_CTRL_CHAIN_TO_SHIFT;
}

static uint32_t dma_ctrl_ro_mask(void) {
    return dma_ctrl_busy_bit() | DMA_CTRL_AHB_ERROR |
           DMA_CTRL_WRITE_ERROR | DMA_CTRL_READ_ERROR;
}

/* ========================================================================
 * Initialization
 * ======================================================================== */

void dma_init(void) {
    memset(&dma_state, 0, sizeof(dma_state));
    /* Default CHAIN_TO = self (no chaining) for each channel */
    int shift = dma_ctrl_chain_to_shift();
    for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
        dma_state.ch[i].ctrl = (uint32_t)i << shift;
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
    int incr_write = (c->ctrl & dma_ctrl_incr_write_bit()) ? 1 : 0;
    int bswap      = (c->ctrl & dma_ctrl_bswap_bit()) ? 1 : 0;

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
            if (bswap) val = (uint16_t)((val >> 8) | (val << 8));
            mem_write16(dst, val);
            break;
        }
        default: {  /* WORD */
            uint32_t val = mem_read32(src);
            if (bswap) val = ((val >> 24) & 0xFF) | ((val >> 8) & 0xFF00) |
                             ((val << 8) & 0xFF0000) | ((val << 24) & 0xFF000000);
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
    if (!(c->ctrl & dma_ctrl_irq_quiet_bit())) {
        dma_state.intr |= (1u << ch_idx);
        /* Signal NVIC if enabled in INTE0 or INTE1 */
        if (dma_state.inte0 & (1u << ch_idx))
            nvic_signal_irq(IRQ_DMA_IRQ_0);
        if (dma_state.inte1 & (1u << ch_idx))
            nvic_signal_irq(IRQ_DMA_IRQ_1);
    }

    /* Chain: if CHAIN_TO != self, trigger the chained channel */
    int chain_to = (int)((c->ctrl & dma_ctrl_chain_to_mask()) >> dma_ctrl_chain_to_shift());
    if (chain_to != ch_idx && chain_to < dma_n_channels()) {
        dma_do_transfer(chain_to);
    }
}

/* ========================================================================
 * Channel Register Access Helpers
 * ======================================================================== */

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
            uint32_t writable = ~dma_ctrl_ro_mask();
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

/* RP2040 and RP2350 global maps overlap (e.g. 0x448 = N_CHANNELS vs TIMER2). */
static int dma_off_multi(void) {
    return membus_rp2350_mode ? DMA_MULTI_CHAN_TRIGGER_RP2350 : DMA_MULTI_CHAN_TRIGGER;
}
static int dma_off_abort(void) {
    return membus_rp2350_mode ? DMA_CHAN_ABORT_RP2350 : DMA_CHAN_ABORT;
}
static int dma_off_nchan(void) {
    return membus_rp2350_mode ? DMA_N_CHANNELS_RP2350 : DMA_N_CHANNELS;
}
static int dma_off_sniff_ctrl(void) {
    return membus_rp2350_mode ? DMA_SNIFF_CTRL_RP2350 : DMA_SNIFF_CTRL;
}
static int dma_off_sniff_data(void) {
    return membus_rp2350_mode ? DMA_SNIFF_DATA_RP2350 : DMA_SNIFF_DATA;
}
static int dma_off_fifo(void) {
    return membus_rp2350_mode ? DMA_FIFO_LEVELS_RP2350 : DMA_FIFO_LEVELS;
}
static int dma_off_timer0(void) {
    return membus_rp2350_mode ? 0x440 : DMA_TIMER0;
}

/* ========================================================================
 * Register Read
 * ======================================================================== */

uint32_t dma_read32(uint32_t offset) {
    int nchan = dma_n_channels();

    /* Per-channel registers */
    if (offset < (uint32_t)nchan * DMA_CH_STRIDE) {
        int ch = offset / DMA_CH_STRIDE;
        int reg = offset % DMA_CH_STRIDE;
        int alias = reg / 0x10;         /* 0-3 */
        int field_idx = (reg % 0x10) / 4;  /* 0-3 */
        return ch_field_read(ch, alias_layout[alias][field_idx]);
    }

    /* Global registers — interrupt block is identical on both chips */
    switch (offset) {
    case DMA_INTR:  return dma_state.intr;
    case DMA_INTE0: return dma_state.inte0;
    case DMA_INTF0: return dma_state.intf0;
    case DMA_INTS0: return (dma_state.intr | dma_state.intf0) & dma_state.inte0;
    case DMA_INTE1: return dma_state.inte1;
    case DMA_INTF1: return dma_state.intf1;
    case DMA_INTS1: return (dma_state.intr | dma_state.intf1) & dma_state.inte1;
    default:
        break;
    }

    {
        uint32_t t0 = (uint32_t)dma_off_timer0();
        if (offset >= t0 && offset < t0 + 16u)
            return dma_state.timer[(offset - t0) / 4u];
    }

    if (offset == (uint32_t)dma_off_multi())
        return 0;  /* Write-only */
    if (offset == (uint32_t)dma_off_sniff_ctrl())
        return dma_state.sniff_ctrl;
    if (offset == (uint32_t)dma_off_sniff_data())
        return dma_state.sniff_data;
    if (offset == (uint32_t)dma_off_fifo())
        return 0;  /* All FIFOs empty */
    if (offset == (uint32_t)dma_off_abort())
        return 0;  /* Write-only */
    if (offset == (uint32_t)dma_off_nchan())
        return (uint32_t)nchan;

    return 0;
}

/* ========================================================================
 * Register Write
 * ======================================================================== */

void dma_write32(uint32_t offset, uint32_t val) {
    int nchan = dma_n_channels();

    /* Per-channel registers */
    if (offset < (uint32_t)nchan * DMA_CH_STRIDE) {
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
        return;
    case DMA_INTE0:
        dma_state.inte0 = val & ((1u << nchan) - 1);
        return;
    case DMA_INTF0:
        dma_state.intf0 = val & ((1u << nchan) - 1);
        return;
    case DMA_INTE1:
        dma_state.inte1 = val & ((1u << nchan) - 1);
        return;
    case DMA_INTF1:
        dma_state.intf1 = val & ((1u << nchan) - 1);
        return;
    default:
        break;
    }

    {
        uint32_t t0 = (uint32_t)dma_off_timer0();
        if (offset >= t0 && offset < t0 + 16u) {
            dma_state.timer[(offset - t0) / 4u] = val;
            return;
        }
    }

    if (offset == (uint32_t)dma_off_multi()) {
        for (int i = 0; i < nchan; i++) {
            if (val & (1u << i))
                dma_do_transfer(i);
        }
        return;
    }
    if (offset == (uint32_t)dma_off_sniff_ctrl()) {
        dma_state.sniff_ctrl = val;
        return;
    }
    if (offset == (uint32_t)dma_off_sniff_data()) {
        dma_state.sniff_data = val;
        return;
    }
    if (offset == (uint32_t)dma_off_abort()) {
        /* Abort channels - for our synchronous model, transfers are already complete.
         * Just clear trans_count for indicated channels. */
        for (int i = 0; i < nchan; i++) {
            if (val & (1u << i))
                dma_state.ch[i].trans_count = 0;
        }
    }
}
