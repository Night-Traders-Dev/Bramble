#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/* RP2040 has two PL022 SPI controllers */
#define SPI0_BASE       0x4003C000
#define SPI1_BASE       0x40040000
#define SPI_BLOCK_SIZE  0x1000

/* PL022 register offsets */
#define SPI_SSPCR0      0x000   /* Control register 0 */
#define SPI_SSPCR1      0x004   /* Control register 1 */
#define SPI_SSPDR       0x008   /* Data register */
#define SPI_SSPSR       0x00C   /* Status register */
#define SPI_SSPCPSR     0x010   /* Clock prescale register */
#define SPI_SSPIMSC     0x014   /* Interrupt mask */
#define SPI_SSPRIS      0x018   /* Raw interrupt status */
#define SPI_SSPMIS      0x01C   /* Masked interrupt status */
#define SPI_SSPICR      0x020   /* Interrupt clear register */
#define SPI_SSPDMACR    0x024   /* DMA control register */
#define SPI_PERIPHID0   0xFE0
#define SPI_PERIPHID1   0xFE4
#define SPI_PERIPHID2   0xFE8
#define SPI_PERIPHID3   0xFEC

/* Status register bits */
#define SPI_SSPSR_TFE   (1u << 0)   /* TX FIFO empty */
#define SPI_SSPSR_TNF   (1u << 1)   /* TX FIFO not full */
#define SPI_SSPSR_RNE   (1u << 2)   /* RX FIFO not empty */
#define SPI_SSPSR_RFF   (1u << 3)   /* RX FIFO full */
#define SPI_SSPSR_BSY   (1u << 4)   /* SPI busy */

/* Per-SPI state */
typedef struct {
    uint32_t cr0;       /* Control register 0 (frame format, clock rate) */
    uint32_t cr1;       /* Control register 1 (enable, mode) */
    uint32_t cpsr;      /* Clock prescale */
    uint32_t imsc;      /* Interrupt mask */
    uint32_t ris;       /* Raw interrupt status */
    uint32_t dmacr;     /* DMA control */
} spi_state_t;

extern spi_state_t spi_state[2];

void spi_init(void);
uint32_t spi_read32(int spi_num, uint32_t offset);
void spi_write32(int spi_num, uint32_t offset, uint32_t val);
int spi_match(uint32_t addr);

#endif /* SPI_H */
