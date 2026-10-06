/*
 * RP2040 DW_apb_i2c Controller
 *
 * Two I2C instances with 16-deep RX FIFOs and device callback support.
 * Writes to DATA_CMD execute immediately through attached device callbacks.
 * Read commands (CMD bit set) query the device and push response to RX FIFO.
 *
 * When no device is attached at the target address, writes are acknowledged
 * and reads return 0xFF (matching open-drain pull-up behavior).
 */

#include <string.h>
#include "i2c.h"
#include "emulator.h"
#include "rp2350_rv/rp2350_memmap.h"
#include "nvic.h"

i2c_state_t i2c_state[2];

static void i2c_reset_instance(i2c_state_t *c) {
    /* Save device registrations across reset */
    i2c_device_entry_t saved_devices[I2C_MAX_DEVICES];
    int saved_count = c->device_count;
    memcpy(saved_devices, c->devices, sizeof(saved_devices));

    memset(c, 0, sizeof(*c));
    c->con         = 0x7F;
    c->tar         = 0x055;
    c->sar         = 0x055;
    c->ss_scl_hcnt = 0x0028;
    c->ss_scl_lcnt = 0x002F;
    c->fs_scl_hcnt = 0x0006;
    c->fs_scl_lcnt = 0x000D;
    c->sda_hold    = 1;
    c->fs_spklen   = 0x07;

    /* Restore device registrations */
    c->device_count = saved_count;
    memcpy(c->devices, saved_devices, sizeof(saved_devices));
}

void i2c_init(void) {
    for (int i = 0; i < 2; i++) {
        i2c_state[i].device_count = 0;
        i2c_reset_instance(&i2c_state[i]);
    }
}

int i2c_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;

    /* RP2350 moved both I2C controllers: I2C0 is at 0x40090000 and I2C1 at
     * 0x40098000, while the RP2040 constants in i2c.h are 0x40044000 and
     * 0x40048000. This function had no per-chip branch, so on RP2350 neither
     * controller was decoded at all -- accesses fell through as unmapped.
     * uart_match() and spi_match() both branch already; this now matches them. */
    if (membus_rp2350_mode && !membus_rv_delegate) {
        if (base >= RP2350_I2C0_BASE && base < RP2350_I2C0_BASE + I2C_BLOCK_SIZE)
            return 0;
        if (base >= RP2350_I2C1_BASE && base < RP2350_I2C1_BASE + I2C_BLOCK_SIZE)
            return 1;
        return -1;
    }

    if (base >= I2C0_BASE && base < I2C0_BASE + I2C_BLOCK_SIZE)
        return 0;
    if (base >= I2C1_BASE && base < I2C1_BASE + I2C_BLOCK_SIZE)
        return 1;
    return -1;
}

/* ========================================================================
 * RX FIFO helpers
 * ======================================================================== */

static void rx_push(i2c_state_t *c, uint8_t val) {
    if (c->rx_count >= I2C_FIFO_SIZE) {
        c->raw_intr_stat |= I2C_INT_RX_OVER;
        return;
    }
    c->rx_fifo[c->rx_head] = val;
    c->rx_head = (c->rx_head + 1) % I2C_FIFO_SIZE;
    c->rx_count++;
}

static uint8_t rx_pop(i2c_state_t *c) {
    if (c->rx_count == 0) {
        c->raw_intr_stat |= I2C_INT_RX_UNDER;
        return 0;
    }
    uint8_t val = c->rx_fifo[c->rx_tail];
    c->rx_tail = (c->rx_tail + 1) % I2C_FIFO_SIZE;
    c->rx_count--;
    return val;
}

/* ========================================================================
 * Device lookup
 * ======================================================================== */

static i2c_device_entry_t *find_device(i2c_state_t *c, uint8_t addr) {
    for (int i = 0; i < c->device_count; i++) {
        if (c->devices[i].addr == addr) {
            return &c->devices[i];
        }
    }
    return NULL;
}

/* ========================================================================
 * Interrupt update
 * ======================================================================== */

static void i2c_update_irq(int i2c_num) {
    i2c_state_t *c = &i2c_state[i2c_num];

    /* TX_EMPTY: TX is always instantly consumed, so always empty */
    c->raw_intr_stat |= I2C_INT_TX_EMPTY;

    /* RX_FULL: RX FIFO level > threshold */
    if (c->rx_count > (int)c->rx_tl) {
        c->raw_intr_stat |= I2C_INT_RX_FULL;
    } else {
        c->raw_intr_stat &= ~I2C_INT_RX_FULL;
    }

    if (c->raw_intr_stat & c->intr_mask) {
        nvic_signal_irq(i2c_num == 0 ? IRQ_I2C0_IRQ : IRQ_I2C1_IRQ);
    }
}

/* ========================================================================
 * Register access
 * ======================================================================== */

uint32_t i2c_read32(int i2c_num, uint32_t offset) {
    i2c_state_t *c = &i2c_state[i2c_num];

    switch (offset) {
    case I2C_CON:           return c->con;
    case I2C_TAR:           return c->tar;
    case I2C_SAR:           return c->sar;

    case I2C_DATA_CMD:
        /* Read pops from RX FIFO */
        {
            uint8_t val = rx_pop(c);
            i2c_update_irq(i2c_num);
            return val;
        }

    case I2C_SS_SCL_HCNT:   return c->ss_scl_hcnt;
    case I2C_SS_SCL_LCNT:   return c->ss_scl_lcnt;
    case I2C_FS_SCL_HCNT:   return c->fs_scl_hcnt;
    case I2C_FS_SCL_LCNT:   return c->fs_scl_lcnt;
    case I2C_INTR_STAT:     return c->raw_intr_stat & c->intr_mask;
    case I2C_INTR_MASK:     return c->intr_mask;
    case I2C_RAW_INTR_STAT: return c->raw_intr_stat;
    case I2C_RX_TL:         return c->rx_tl;
    case I2C_TX_TL:         return c->tx_tl;

    case I2C_CLR_INTR:
        { uint32_t v = c->raw_intr_stat; c->raw_intr_stat = 0; return v; }
    case I2C_CLR_RX_UNDER:
        { c->raw_intr_stat &= ~I2C_INT_RX_UNDER; return 0; }
    case I2C_CLR_RX_OVER:
        { c->raw_intr_stat &= ~I2C_INT_RX_OVER; return 0; }
    case I2C_CLR_TX_OVER:
        { c->raw_intr_stat &= ~I2C_INT_TX_OVER; return 0; }
    case I2C_CLR_TX_ABRT:
        { c->raw_intr_stat &= ~I2C_INT_TX_ABRT; return 0; }
    case I2C_CLR_STOP_DET:
        { c->raw_intr_stat &= ~I2C_INT_STOP_DET; return 0; }
    case I2C_CLR_START_DET:
        { c->raw_intr_stat &= ~I2C_INT_START_DET; return 0; }
    case I2C_CLR_RESTART_DET:
        return 0;

    case I2C_ENABLE:        return c->enable;

    case I2C_STATUS: {
        uint32_t sr = I2C_STATUS_TFE | I2C_STATUS_TFNF;  /* TX always ready */
        if (c->rx_count > 0)              sr |= I2C_STATUS_RFNE;
        if (c->rx_count >= I2C_FIFO_SIZE) sr |= I2C_STATUS_RFF;
        return sr;
    }

    case I2C_TXFLR:         return 0;  /* TX instantly consumed */
    case I2C_RXFLR:         return (uint32_t)c->rx_count;
    case I2C_SDA_HOLD:      return c->sda_hold;
    case I2C_TX_ABRT_SOURCE: return 0;
    case I2C_DMA_CR:        return c->dma_cr;
    case I2C_DMA_TDLR:      return c->dma_tdlr;
    case I2C_DMA_RDLR:      return c->dma_rdlr;
    case I2C_FS_SPKLEN:     return c->fs_spklen;

    /* DW_apb_i2c component registers */
    case I2C_COMP_PARAM_1:  return 0x00FFFF6E;
    case I2C_COMP_VERSION:  return 0x3230312A;
    case I2C_COMP_TYPE:     return 0x44570140;

    default:
        return 0;
    }
}

void i2c_write32(int i2c_num, uint32_t offset, uint32_t val) {
    i2c_state_t *c = &i2c_state[i2c_num];

    switch (offset) {
    case I2C_CON:           c->con = val & 0x7F; break;
    case I2C_TAR:           c->tar = val & 0x3FF; break;
    case I2C_SAR:           c->sar = val & 0x3FF; break;

    case I2C_DATA_CMD: {
        /* Execute I2C transaction immediately */
        uint8_t addr = (uint8_t)(c->tar & 0x7F);
        i2c_device_entry_t *dev = find_device(c, addr);

        if (val & I2C_DATA_CMD_RESTART) {
            if (dev && dev->start_fn) dev->start_fn(dev->ctx);
        }

        if (val & I2C_DATA_CMD_READ) {
            /* Read command: query device, push to RX FIFO */
            uint8_t data = 0xFF;  /* Default: pull-up */
            if (dev && dev->read_fn) {
                data = dev->read_fn(dev->ctx);
            }
            rx_push(c, data);
        } else {
            /* Write command: send byte to device */
            if (dev && dev->write_fn) {
                dev->write_fn(dev->ctx, (uint8_t)(val & 0xFF));
            }
        }

        if (val & I2C_DATA_CMD_STOP) {
            c->raw_intr_stat |= I2C_INT_STOP_DET;
            if (dev && dev->stop_fn) dev->stop_fn(dev->ctx);
        }
        break;
    }

    case I2C_SS_SCL_HCNT:   c->ss_scl_hcnt = val & 0xFFFF; break;
    case I2C_SS_SCL_LCNT:   c->ss_scl_lcnt = val & 0xFFFF; break;
    case I2C_FS_SCL_HCNT:   c->fs_scl_hcnt = val & 0xFFFF; break;
    case I2C_FS_SCL_LCNT:   c->fs_scl_lcnt = val & 0xFFFF; break;
    case I2C_INTR_MASK:     c->intr_mask = val & 0xFFF; break;
    case I2C_RX_TL:         c->rx_tl = val & 0xFF; break;
    case I2C_TX_TL:         c->tx_tl = val & 0xFF; break;
    case I2C_CLR_INTR:      c->raw_intr_stat = 0; break;
    case I2C_CLR_TX_ABRT:   c->raw_intr_stat &= ~I2C_INT_TX_ABRT; break;
    case I2C_ENABLE:        c->enable = val & 0x03; break;
    case I2C_SDA_HOLD:      c->sda_hold = val & 0xFFFF; break;
    case I2C_DMA_CR:        c->dma_cr = val & 0x03; break;
    case I2C_DMA_TDLR:      c->dma_tdlr = val & 0x1F; break;
    case I2C_DMA_RDLR:      c->dma_rdlr = val & 0x1F; break;
    case I2C_FS_SPKLEN:     c->fs_spklen = val & 0xFF; break;
    default: break;
    }

    i2c_update_irq(i2c_num);
}

int i2c_attach_device(int i2c_num, uint8_t addr,
                      i2c_device_write_fn write_fn,
                      i2c_device_read_fn read_fn,
                      i2c_device_event_fn start_fn,
                      i2c_device_event_fn stop_fn,
                      void *ctx) {
    if (i2c_num < 0 || i2c_num > 1) return -1;
    i2c_state_t *c = &i2c_state[i2c_num];
    if (c->device_count >= I2C_MAX_DEVICES) return -1;

    i2c_device_entry_t *e = &c->devices[c->device_count++];
    e->addr = addr;
    e->write_fn = write_fn;
    e->read_fn = read_fn;
    e->start_fn = start_fn;
    e->stop_fn = stop_fn;
    e->ctx = ctx;
    return 0;
}
