#ifndef UART_H
#define UART_H

#include <stdint.h>

/* RP2040 has two PL011 UARTs */
#define UART0_BASE      0x40034000
#define UART1_BASE      0x40038000
#define UART_BLOCK_SIZE 0x1000

/* PL011 register offsets */
#define UART_DR         0x000   /* Data Register */
#define UART_RSR        0x004   /* Receive Status / Error Clear */
#define UART_FR         0x018   /* Flag Register */
#define UART_ILPR       0x020   /* IrDA Low-Power Counter (unused) */
#define UART_IBRD       0x024   /* Integer Baud Rate */
#define UART_FBRD       0x028   /* Fractional Baud Rate */
#define UART_LCR_H      0x02C   /* Line Control */
#define UART_CR         0x030   /* Control Register */
#define UART_IFLS       0x034   /* Interrupt FIFO Level Select */
#define UART_IMSC       0x038   /* Interrupt Mask Set/Clear */
#define UART_RIS        0x03C   /* Raw Interrupt Status */
#define UART_MIS        0x040   /* Masked Interrupt Status */
#define UART_ICR        0x044   /* Interrupt Clear Register */
#define UART_DMACR      0x048   /* DMA Control Register */
#define UART_PERIPHID0  0xFE0   /* Peripheral ID 0 */
#define UART_PERIPHID1  0xFE4
#define UART_PERIPHID2  0xFE8
#define UART_PERIPHID3  0xFEC
#define UART_PCELLID0   0xFF0   /* PrimeCell ID 0 */
#define UART_PCELLID1   0xFF4
#define UART_PCELLID2   0xFF8
#define UART_PCELLID3   0xFFC

/* Flag Register bits */
#define UART_FR_TXFE    (1u << 7)   /* TX FIFO empty */
#define UART_FR_RXFF    (1u << 6)   /* RX FIFO full */
#define UART_FR_TXFF    (1u << 5)   /* TX FIFO full */
#define UART_FR_RXFE    (1u << 4)   /* RX FIFO empty */
#define UART_FR_BUSY    (1u << 3)   /* UART busy */

/* Control Register bits */
#define UART_CR_UARTEN  (1u << 0)   /* UART enable */
#define UART_CR_TXE     (1u << 8)   /* TX enable */
#define UART_CR_RXE     (1u << 9)   /* RX enable */

/* Interrupt bits */
#define UART_INT_TX     (1u << 5)   /* TX interrupt */
#define UART_INT_RX     (1u << 4)   /* RX interrupt */

/* Per-UART state */
typedef struct {
    uint32_t dr;        /* Last written data (for debug) */
    uint32_t rsr;       /* Receive status */
    uint32_t ibrd;      /* Integer baud rate divisor */
    uint32_t fbrd;      /* Fractional baud rate divisor */
    uint32_t lcr_h;     /* Line control */
    uint32_t cr;        /* Control register */
    uint32_t ifls;      /* Interrupt FIFO level select */
    uint32_t imsc;      /* Interrupt mask */
    uint32_t ris;       /* Raw interrupt status */
    uint32_t dmacr;     /* DMA control */
    int enabled;        /* Derived from CR.UARTEN */
} uart_state_t;

extern uart_state_t uart_state[2];

/* Initialize both UARTs */
void uart_init(void);

/* Register access */
uint32_t uart_read32(int uart_num, uint32_t offset);
void uart_write32(int uart_num, uint32_t offset, uint32_t val);

/* Check if address is in UART space, returns 0 or 1 for uart_num, -1 if not */
int uart_match(uint32_t addr);

#endif /* UART_H */
