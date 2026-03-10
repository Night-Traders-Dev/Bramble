#ifndef W5500_H
#define W5500_H

#include <stdint.h>

/* ========================================================================
 * W5500 Ethernet Controller (SPI Device Plugin)
 *
 * Emulates the WIZnet W5500 hardwired TCP/IP Ethernet controller.
 * Communicates via SPI using variable-length data frames with a
 * 3-byte header (2 address + 1 control).
 *
 * Attaches to an SPI bus via spi_attach_device().
 *
 * SPI frame format:
 *   Byte 0-1: 16-bit offset address (big-endian)
 *   Byte 2:   Control byte - BSB[4:0] | RW | OM[1:0]
 *             RW: 0=read, 1=write
 *             OM: 00=VDM (variable), 01=FDM 1B, 10=FDM 2B, 11=FDM 4B
 *   Byte 3+:  Data phase (read returns register, write stores)
 *
 * Block Select Byte (BSB) mapping:
 *   0       = Common registers
 *   1,5,9,13,17,21,25,29  = Socket 0-7 registers
 *   2,6,10,14,18,22,26,30 = Socket 0-7 TX buffer
 *   3,7,11,15,19,23,27,31 = Socket 0-7 RX buffer
 *
 * Usage:
 *   w5500_t w5500;
 *   w5500_init(&w5500);
 *   spi_attach_device(spi_num, w5500_spi_xfer, w5500_spi_cs, &w5500);
 * ======================================================================== */

#define W5500_NUM_SOCKETS   8
#define W5500_TX_BUF_SIZE   2048    /* Per socket */
#define W5500_RX_BUF_SIZE   2048    /* Per socket */

/* Common register offsets */
#define W5500_MR        0x0000  /* Mode register */
#define W5500_GAR0      0x0001  /* Gateway address (4 bytes) */
#define W5500_SUBR0     0x0005  /* Subnet mask (4 bytes) */
#define W5500_SHAR0     0x0009  /* Source MAC address (6 bytes) */
#define W5500_SIPR0     0x000F  /* Source IP address (4 bytes) */
#define W5500_IR        0x0015  /* Interrupt register */
#define W5500_IMR       0x0016  /* Interrupt mask */
#define W5500_SIR       0x0017  /* Socket interrupt */
#define W5500_SIMR      0x0018  /* Socket interrupt mask */
#define W5500_RTR0      0x0019  /* Retry time (2 bytes) */
#define W5500_RCR       0x001B  /* Retry count */
#define W5500_PHYCFGR   0x002E  /* PHY configuration */
#define W5500_VERSIONR  0x0039  /* Chip version (read-only, 0x04) */

/* Common register block size */
#define W5500_COMMON_REG_SIZE   0x0040

/* Socket register offsets (within each socket register block) */
#define W5500_Sn_MR     0x0000  /* Socket mode */
#define W5500_Sn_CR     0x0001  /* Socket command */
#define W5500_Sn_IR     0x0002  /* Socket interrupt */
#define W5500_Sn_SR     0x0003  /* Socket status */
#define W5500_Sn_PORT0  0x0004  /* Source port (2 bytes) */
#define W5500_Sn_DHAR0  0x0006  /* Dest MAC (6 bytes) */
#define W5500_Sn_DIPR0  0x000C  /* Dest IP (4 bytes) */
#define W5500_Sn_DPORT0 0x0010  /* Dest port (2 bytes) */
#define W5500_Sn_MSSR0  0x0012  /* Max segment size (2 bytes) */
#define W5500_Sn_TOS    0x0015  /* TOS */
#define W5500_Sn_TTL    0x0016  /* TTL */
#define W5500_Sn_RXBUF_SIZE 0x001E  /* RX buffer size */
#define W5500_Sn_TXBUF_SIZE 0x001F  /* TX buffer size */
#define W5500_Sn_TX_FSR0    0x0020  /* TX free size (2 bytes) */
#define W5500_Sn_TX_RD0     0x0022  /* TX read pointer (2 bytes) */
#define W5500_Sn_TX_WR0     0x0024  /* TX write pointer (2 bytes) */
#define W5500_Sn_RX_RSR0    0x0026  /* RX received size (2 bytes) */
#define W5500_Sn_RX_RD0     0x0028  /* RX read pointer (2 bytes) */
#define W5500_Sn_RX_WR0     0x002A  /* RX write pointer (2 bytes) */
#define W5500_Sn_IMR    0x002C  /* Socket interrupt mask */

/* Socket register block size */
#define W5500_SOCKET_REG_SIZE   0x0030

/* Socket status values */
#define W5500_SOCK_CLOSED   0x00
#define W5500_SOCK_INIT     0x13
#define W5500_SOCK_LISTEN   0x14
#define W5500_SOCK_ESTABLISHED 0x17
#define W5500_SOCK_UDP      0x22
#define W5500_SOCK_MACRAW   0x42

/* Socket commands */
#define W5500_CMD_OPEN      0x01
#define W5500_CMD_LISTEN    0x02
#define W5500_CMD_CONNECT   0x04
#define W5500_CMD_CLOSE     0x08
#define W5500_CMD_SEND      0x20
#define W5500_CMD_RECV      0x40

/* Socket modes */
#define W5500_MR_TCP    0x01
#define W5500_MR_UDP    0x02
#define W5500_MR_MACRAW 0x04

/* PHYCFGR bits */
#define W5500_PHY_LINK  (1u << 0)   /* Link status */
#define W5500_PHY_SPD   (1u << 1)   /* Speed: 1=100Mbps */
#define W5500_PHY_DPX   (1u << 2)   /* Duplex: 1=full */
#define W5500_PHY_OPMDC (7u << 3)   /* Operation mode */
#define W5500_PHY_RST   (1u << 7)   /* Reset (active low) */

/* SPI frame phase */
typedef enum {
    W5500_PHASE_ADDR_HI,    /* Collecting address byte 1 */
    W5500_PHASE_ADDR_LO,    /* Collecting address byte 2 */
    W5500_PHASE_CONTROL,    /* Collecting control byte */
    W5500_PHASE_DATA,       /* Data transfer phase */
} w5500_phase_t;

/* Per-socket state */
typedef struct {
    uint8_t regs[W5500_SOCKET_REG_SIZE];
    uint8_t tx_buf[W5500_TX_BUF_SIZE];
    uint8_t rx_buf[W5500_RX_BUF_SIZE];
} w5500_socket_t;

/* W5500 device state */
typedef struct {
    /* Common registers */
    uint8_t common[W5500_COMMON_REG_SIZE];

    /* Socket state */
    w5500_socket_t sockets[W5500_NUM_SOCKETS];

    /* SPI frame state machine */
    w5500_phase_t phase;
    uint16_t addr;          /* Current offset address */
    uint8_t  bsb;           /* Block select byte (5 bits) */
    int      rw;            /* 0=read, 1=write */
    int      cs_active;     /* Chip select state */
} w5500_t;

/* Initialize W5500 device with default register values */
void w5500_init(w5500_t *dev);

/* SPI transfer callback (for spi_attach_device) */
uint8_t w5500_spi_xfer(void *ctx, uint8_t mosi);

/* SPI chip-select callback */
void w5500_spi_cs(void *ctx, int cs_active);

#endif /* W5500_H */
