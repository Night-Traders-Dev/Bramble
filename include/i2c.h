#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/* RP2040 has two DW_apb_i2c controllers */
#define I2C0_BASE       0x40044000
#define I2C1_BASE       0x40048000
#define I2C_BLOCK_SIZE  0x1000

/* DW_apb_i2c register offsets */
#define I2C_CON             0x000   /* Control register */
#define I2C_TAR             0x004   /* Target address */
#define I2C_SAR             0x008   /* Slave address */
#define I2C_DATA_CMD        0x010   /* Data buffer and command */
#define I2C_SS_SCL_HCNT     0x014   /* Standard speed SCL high count */
#define I2C_SS_SCL_LCNT     0x018   /* Standard speed SCL low count */
#define I2C_FS_SCL_HCNT     0x01C   /* Fast speed SCL high count */
#define I2C_FS_SCL_LCNT     0x020   /* Fast speed SCL low count */
#define I2C_INTR_STAT       0x02C   /* Interrupt status */
#define I2C_INTR_MASK       0x030   /* Interrupt mask */
#define I2C_RAW_INTR_STAT   0x034   /* Raw interrupt status */
#define I2C_RX_TL           0x038   /* RX FIFO threshold */
#define I2C_TX_TL           0x03C   /* TX FIFO threshold */
#define I2C_CLR_INTR        0x040   /* Clear combined interrupt */
#define I2C_CLR_TX_ABRT     0x054   /* Clear TX_ABRT interrupt */
#define I2C_ENABLE          0x06C   /* Enable register */
#define I2C_STATUS          0x070   /* Status register */
#define I2C_TXFLR           0x074   /* TX FIFO level */
#define I2C_RXFLR           0x078   /* RX FIFO level */
#define I2C_SDA_HOLD        0x07C   /* SDA hold time */
#define I2C_TX_ABRT_SOURCE  0x080   /* TX abort source */
#define I2C_DMA_CR          0x088   /* DMA control */
#define I2C_DMA_TDLR        0x08C   /* DMA TX data level */
#define I2C_DMA_RDLR        0x090   /* DMA RX data level */
#define I2C_FS_SPKLEN       0x0A0   /* Spike suppression limit */
#define I2C_CLR_RESTART_DET 0x0A8   /* Clear RESTART_DET interrupt */
#define I2C_COMP_PARAM_1    0x0F4   /* Component parameter */
#define I2C_COMP_VERSION    0x0F8   /* Component version */
#define I2C_COMP_TYPE       0x0FC   /* Component type */

/* Status register bits */
#define I2C_STATUS_ACTIVITY (1u << 0)   /* I2C activity */
#define I2C_STATUS_TFNF     (1u << 1)   /* TX FIFO not full */
#define I2C_STATUS_TFE      (1u << 2)   /* TX FIFO empty */
#define I2C_STATUS_RFNE     (1u << 3)   /* RX FIFO not empty */
#define I2C_STATUS_RFF      (1u << 4)   /* RX FIFO full */
#define I2C_STATUS_MST_ACT  (1u << 5)   /* Master activity */

/* Per-I2C state */
typedef struct {
    uint32_t con;           /* Control */
    uint32_t tar;           /* Target address */
    uint32_t sar;           /* Slave address */
    uint32_t ss_scl_hcnt;
    uint32_t ss_scl_lcnt;
    uint32_t fs_scl_hcnt;
    uint32_t fs_scl_lcnt;
    uint32_t intr_mask;
    uint32_t raw_intr_stat;
    uint32_t rx_tl;
    uint32_t tx_tl;
    uint32_t enable;
    uint32_t sda_hold;
    uint32_t dma_cr;
    uint32_t dma_tdlr;
    uint32_t dma_rdlr;
    uint32_t fs_spklen;
} i2c_state_t;

extern i2c_state_t i2c_state[2];

void i2c_init(void);
uint32_t i2c_read32(int i2c_num, uint32_t offset);
void i2c_write32(int i2c_num, uint32_t offset, uint32_t val);
int i2c_match(uint32_t addr);

#endif /* I2C_H */
