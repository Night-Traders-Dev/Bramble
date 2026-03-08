#include "i2c.h"

i2c_state_t i2c_state[2];

void i2c_init(void) {
    for (int i = 0; i < 2; i++) {
        i2c_state[i].con         = 0x7F;  /* Default: master, 7-bit, fast, restart, enabled */
        i2c_state[i].tar         = 0x055;
        i2c_state[i].sar         = 0x055;
        i2c_state[i].ss_scl_hcnt = 0x0028;
        i2c_state[i].ss_scl_lcnt = 0x002F;
        i2c_state[i].fs_scl_hcnt = 0x0006;
        i2c_state[i].fs_scl_lcnt = 0x000D;
        i2c_state[i].intr_mask   = 0;
        i2c_state[i].raw_intr_stat = 0;
        i2c_state[i].rx_tl       = 0;
        i2c_state[i].tx_tl       = 0;
        i2c_state[i].enable      = 0;
        i2c_state[i].sda_hold    = 1;
        i2c_state[i].dma_cr      = 0;
        i2c_state[i].dma_tdlr    = 0;
        i2c_state[i].dma_rdlr    = 0;
        i2c_state[i].fs_spklen   = 0x07;
    }
}

int i2c_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;
    if (base >= I2C0_BASE && base < I2C0_BASE + I2C_BLOCK_SIZE)
        return 0;
    if (base >= I2C1_BASE && base < I2C1_BASE + I2C_BLOCK_SIZE)
        return 1;
    return -1;
}

uint32_t i2c_read32(int i2c_num, uint32_t offset) {
    i2c_state_t *c = &i2c_state[i2c_num];

    switch (offset) {
    case I2C_CON:           return c->con;
    case I2C_TAR:           return c->tar;
    case I2C_SAR:           return c->sar;
    case I2C_DATA_CMD:      return 0;  /* No RX data */
    case I2C_SS_SCL_HCNT:   return c->ss_scl_hcnt;
    case I2C_SS_SCL_LCNT:   return c->ss_scl_lcnt;
    case I2C_FS_SCL_HCNT:   return c->fs_scl_hcnt;
    case I2C_FS_SCL_LCNT:   return c->fs_scl_lcnt;
    case I2C_INTR_STAT:     return c->raw_intr_stat & c->intr_mask;
    case I2C_INTR_MASK:     return c->intr_mask;
    case I2C_RAW_INTR_STAT: return c->raw_intr_stat;
    case I2C_RX_TL:         return c->rx_tl;
    case I2C_TX_TL:         return c->tx_tl;
    case I2C_CLR_INTR:      return 0;  /* Read clears all interrupts */
    case I2C_CLR_TX_ABRT:   return 0;
    case I2C_ENABLE:        return c->enable;

    case I2C_STATUS:
        /* TX FIFO empty + not full, RX empty, not busy */
        return I2C_STATUS_TFE | I2C_STATUS_TFNF;

    case I2C_TXFLR:         return 0;  /* TX FIFO empty */
    case I2C_RXFLR:         return 0;  /* RX FIFO empty */
    case I2C_SDA_HOLD:      return c->sda_hold;
    case I2C_TX_ABRT_SOURCE: return 0;
    case I2C_DMA_CR:        return c->dma_cr;
    case I2C_DMA_TDLR:      return c->dma_tdlr;
    case I2C_DMA_RDLR:      return c->dma_rdlr;
    case I2C_FS_SPKLEN:     return c->fs_spklen;
    case I2C_CLR_RESTART_DET: return 0;

    /* DW_apb_i2c component registers */
    case I2C_COMP_PARAM_1:  return 0x00FFFF6E;
    case I2C_COMP_VERSION:  return 0x3230312A;  /* "210*" */
    case I2C_COMP_TYPE:     return 0x44570140;  /* "DW\x01\x40" */

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
    case I2C_DATA_CMD:      break;  /* TX data silently discarded */
    case I2C_SS_SCL_HCNT:   c->ss_scl_hcnt = val & 0xFFFF; break;
    case I2C_SS_SCL_LCNT:   c->ss_scl_lcnt = val & 0xFFFF; break;
    case I2C_FS_SCL_HCNT:   c->fs_scl_hcnt = val & 0xFFFF; break;
    case I2C_FS_SCL_LCNT:   c->fs_scl_lcnt = val & 0xFFFF; break;
    case I2C_INTR_MASK:     c->intr_mask = val & 0xFFF; break;
    case I2C_RX_TL:         c->rx_tl = val & 0xFF; break;
    case I2C_TX_TL:         c->tx_tl = val & 0xFF; break;
    case I2C_CLR_INTR:      c->raw_intr_stat = 0; break;
    case I2C_CLR_TX_ABRT:   break;
    case I2C_ENABLE:        c->enable = val & 0x03; break;
    case I2C_SDA_HOLD:      c->sda_hold = val & 0xFFFF; break;
    case I2C_DMA_CR:        c->dma_cr = val & 0x03; break;
    case I2C_DMA_TDLR:      c->dma_tdlr = val & 0x1F; break;
    case I2C_DMA_RDLR:      c->dma_rdlr = val & 0x1F; break;
    case I2C_FS_SPKLEN:     c->fs_spklen = val & 0xFF; break;
    default: break;
    }
}
