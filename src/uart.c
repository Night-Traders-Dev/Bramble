#include <stdio.h>
#include <string.h>
#include "uart.h"
#include "nvic.h"
#include "netbridge.h"
#include "wire.h"
#include "usb.h"

/* Two UART instances */
uart_state_t uart_state[2];

static void uart_reset_instance(uart_state_t *u) {
    memset(u, 0, sizeof(*u));
    u->ifls = 0x12;  /* Reset FIFO trigger levels */
    u->enabled = 0;
}

void uart_init(void) {
    for (int i = 0; i < 2; i++) {
        uart_reset_instance(&uart_state[i]);
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

/* ========================================================================
 * RX FIFO helpers
 * ======================================================================== */

/* Get the RX FIFO trigger level based on IFLS register bits [5:3] */
static uint32_t rx_trigger_level(uart_state_t *u) {
    switch ((u->ifls >> 3) & 0x7) {
    case 0: return 2;   /* 1/8 full  = 2 of 16 */
    case 1: return 4;   /* 1/4 full  = 4 of 16 */
    case 2: return 8;   /* 1/2 full  = 8 of 16 */
    case 3: return 12;  /* 3/4 full  = 12 of 16 */
    case 4: return 14;  /* 7/8 full  = 14 of 16 */
    default: return 8;
    }
}

/* Signal NVIC if any masked interrupt is active */
static void uart_check_irq(int uart_num) {
    uart_state_t *u = &uart_state[uart_num];
    if (u->ris & u->imsc) {
        nvic_signal_irq(uart_num == 0 ? IRQ_UART0_IRQ : IRQ_UART1_IRQ);
    }
}

/* Update RX interrupt status based on FIFO level */
static void uart_rx_update_irq(uart_state_t *u) {
    if (u->rx_count >= rx_trigger_level(u)) {
        u->ris |= UART_INT_RX;
    }
}

int uart_rx_push(int uart_num, uint8_t data) {
    if (uart_num < 0 || uart_num > 1) return 0;
    uart_state_t *u = &uart_state[uart_num];

    if (u->rx_count >= UART_RX_FIFO_SIZE)
        return 0;  /* FIFO full */

    u->rx_fifo[u->rx_head] = data;
    u->rx_head = (u->rx_head + 1) % UART_RX_FIFO_SIZE;
    u->rx_count++;

    uart_rx_update_irq(u);
    uart_check_irq(uart_num);
    return 1;
}

static uint8_t uart_rx_pop(uart_state_t *u) {
    if (u->rx_count == 0)
        return 0;

    uint8_t data = u->rx_fifo[u->rx_tail];
    u->rx_tail = (u->rx_tail + 1) % UART_RX_FIFO_SIZE;
    u->rx_count--;

    /* Clear RX interrupt if below trigger level */
    if (u->rx_count < rx_trigger_level(u)) {
        u->ris &= ~UART_INT_RX;
    }
    return data;
}

/* ========================================================================
 * Register Access
 * ======================================================================== */

uint32_t uart_read32(int uart_num, uint32_t offset) {
    uart_state_t *u = &uart_state[uart_num];

    switch (offset) {
    case UART_DR:
        /* Read pops from RX FIFO; DR[7:0] = data, DR[11:8] = error flags */
        if (u->rx_count > 0) {
            return (uint32_t)uart_rx_pop(u);
        }
        return 0;  /* No data available */

    case UART_RSR:
        return u->rsr;

    case UART_FR: {
        uint32_t fr = UART_FR_TXFE;  /* TX FIFO always empty (instant TX) */
        if (u->rx_count == 0)
            fr |= UART_FR_RXFE;
        if (u->rx_count >= UART_RX_FIFO_SIZE)
            fr |= UART_FR_RXFF;
        return fr;
    }

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
            uint8_t ch = (uint8_t)(val & 0xFF);
            if (net_bridge_uart_active(uart_num)) {
                net_bridge_uart_tx(uart_num, ch);
            } else if (wire_uart_active(uart_num)) {
                wire_send_uart(uart_num, ch);
            } else if (!usb_cdc_stdout_enabled) {
                putchar((char)ch);
                fflush(stdout);
            }
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
        if (u->enabled) {
            /* TX FIFO is always empty (instant TX), so assert TX interrupt */
            u->ris |= UART_INT_TX;
        }
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

    /* Check if any masked interrupt is now active */
    uart_check_irq(uart_num);
}
