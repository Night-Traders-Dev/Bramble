#include "spi.h"

spi_state_t spi_state[2];

void spi_init(void) {
    for (int i = 0; i < 2; i++) {
        spi_state[i].cr0   = 0;
        spi_state[i].cr1   = 0;
        spi_state[i].cpsr  = 0;
        spi_state[i].imsc  = 0;
        spi_state[i].ris   = 0;
        spi_state[i].dmacr = 0;
    }
}

int spi_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;
    if (base >= SPI0_BASE && base < SPI0_BASE + SPI_BLOCK_SIZE)
        return 0;
    if (base >= SPI1_BASE && base < SPI1_BASE + SPI_BLOCK_SIZE)
        return 1;
    return -1;
}

uint32_t spi_read32(int spi_num, uint32_t offset) {
    spi_state_t *s = &spi_state[spi_num];

    switch (offset) {
    case SPI_SSPCR0:
        return s->cr0;

    case SPI_SSPCR1:
        return s->cr1;

    case SPI_SSPDR:
        /* No RX data - return 0 */
        return 0;

    case SPI_SSPSR:
        /* TX FIFO empty, TX not full, RX empty, not busy */
        return SPI_SSPSR_TFE | SPI_SSPSR_TNF;

    case SPI_SSPCPSR:
        return s->cpsr;

    case SPI_SSPIMSC:
        return s->imsc;

    case SPI_SSPRIS:
        return s->ris;

    case SPI_SSPMIS:
        return s->ris & s->imsc;

    case SPI_SSPDMACR:
        return s->dmacr;

    /* PL022 Peripheral ID */
    case SPI_PERIPHID0: return 0x22;
    case SPI_PERIPHID1: return 0x10;
    case SPI_PERIPHID2: return 0x04;
    case SPI_PERIPHID3: return 0x00;

    default:
        return 0;
    }
}

void spi_write32(int spi_num, uint32_t offset, uint32_t val) {
    spi_state_t *s = &spi_state[spi_num];

    switch (offset) {
    case SPI_SSPCR0:
        s->cr0 = val & 0xFFFF;
        break;

    case SPI_SSPCR1:
        s->cr1 = val & 0x0F;
        break;

    case SPI_SSPDR:
        /* TX write - data is silently discarded (no bus connected) */
        break;

    case SPI_SSPCPSR:
        s->cpsr = val & 0xFF;
        break;

    case SPI_SSPIMSC:
        s->imsc = val & 0x0F;
        break;

    case SPI_SSPICR:
        s->ris &= ~(val & 0x03);
        break;

    case SPI_SSPDMACR:
        s->dmacr = val & 0x03;
        break;

    default:
        break;
    }
}
