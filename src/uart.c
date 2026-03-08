#include <stdio.h>
#include "uart.h"

/* Two UART instances */
uart_state_t uart_state[2];

void uart_init(void) {
    for (int i = 0; i < 2; i++) {
        uart_state[i].dr    = 0;
        uart_state[i].rsr   = 0;
        uart_state[i].ibrd  = 0;
        uart_state[i].fbrd  = 0;
        uart_state[i].lcr_h = 0;
        uart_state[i].cr    = UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
        uart_state[i].ifls  = 0x12;  /* Default: 1/2 full */
        uart_state[i].imsc  = 0;
        uart_state[i].ris   = UART_INT_TX;  /* TX FIFO empty from start */
        uart_state[i].dmacr = 0;
        uart_state[i].enabled = 1;
    }
}

int uart_match(uint32_t addr) {
    /* Strip atomic alias bits to get base peripheral address */
    uint32_t base = addr & ~0x3000;
    if (base >= UART0_BASE && base < UART0_BASE + UART_BLOCK_SIZE)
        return 0;
    if (base >= UART1_BASE && base < UART1_BASE + UART_BLOCK_SIZE)
        return 1;
    return -1;
}

uint32_t uart_read32(int uart_num, uint32_t offset) {
    uart_state_t *u = &uart_state[uart_num];

    switch (offset) {
    case UART_DR:
        /* No RX data available - return 0 */
        return 0;

    case UART_RSR:
        return u->rsr;

    case UART_FR:
        /* TX FIFO always empty (instant TX), RX FIFO always empty (no input) */
        return UART_FR_TXFE | UART_FR_RXFE;

    case UART_IBRD:
        return u->ibrd;

    case UART_FBRD:
        return u->fbrd;

    case UART_LCR_H:
        return u->lcr_h;

    case UART_CR:
        return u->cr;

    case UART_IFLS:
        return u->ifls;

    case UART_IMSC:
        return u->imsc;

    case UART_RIS:
        return u->ris;

    case UART_MIS:
        return u->ris & u->imsc;

    case UART_DMACR:
        return u->dmacr;

    /* PL011 Peripheral/PrimeCell ID registers */
    case UART_PERIPHID0: return 0x11;
    case UART_PERIPHID1: return 0x10;
    case UART_PERIPHID2: return 0x34;
    case UART_PERIPHID3: return 0x00;
    case UART_PCELLID0:  return 0x0D;
    case UART_PCELLID1:  return 0xF0;
    case UART_PCELLID2:  return 0x05;
    case UART_PCELLID3:  return 0xB1;

    default:
        return 0;
    }
}

void uart_write32(int uart_num, uint32_t offset, uint32_t val) {
    uart_state_t *u = &uart_state[uart_num];

    switch (offset) {
    case UART_DR:
        u->dr = val;
        if (u->cr & UART_CR_TXE) {
            putchar((char)(val & 0xFF));
            fflush(stdout);
        }
        break;

    case UART_RSR:
        /* Write clears error flags */
        u->rsr = 0;
        break;

    case UART_IBRD:
        u->ibrd = val & 0xFFFF;
        break;

    case UART_FBRD:
        u->fbrd = val & 0x3F;
        break;

    case UART_LCR_H:
        u->lcr_h = val & 0xFF;
        break;

    case UART_CR:
        u->cr = val & 0xFFFF;
        u->enabled = (val & UART_CR_UARTEN) ? 1 : 0;
        break;

    case UART_IFLS:
        u->ifls = val & 0x3F;
        break;

    case UART_IMSC:
        u->imsc = val & 0x7FF;
        break;

    case UART_ICR:
        /* Write-1-to-clear interrupt bits */
        u->ris &= ~(val & 0x7FF);
        break;

    case UART_DMACR:
        u->dmacr = val & 0x07;
        break;

    default:
        break;
    }
}
