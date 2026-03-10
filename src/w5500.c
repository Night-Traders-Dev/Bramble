/*
 * W5500 Ethernet Controller (SPI Device Plugin)
 *
 * Emulates the WIZnet W5500 hardwired TCP/IP Ethernet controller.
 * Processes SPI frames with a 3-byte header (2 address + 1 control)
 * followed by data bytes for read or write operations.
 *
 * Block Select Byte (BSB) routes access to common registers,
 * per-socket registers, or per-socket TX/RX buffers. The W5500
 * supports 8 independent sockets, each with 2KB TX and 2KB RX
 * buffer space.
 *
 * This is a register-level model for firmware development. No actual
 * network I/O is performed -- socket commands update status registers
 * to simulate expected state transitions.
 */

#include <string.h>
#include "w5500.h"

/* ========================================================================
 * BSB decoding helpers
 *
 * BSB encoding (5 bits):
 *   0        = common registers
 *   n*4 + 1  = socket n registers   (n = 0..7)
 *   n*4 + 2  = socket n TX buffer
 *   n*4 + 3  = socket n RX buffer
 * ======================================================================== */

/* Returns block type: 0=common, 1=socket reg, 2=TX buf, 3=RX buf */
static int bsb_type(uint8_t bsb) {
    if (bsb == 0) return 0;
    return ((bsb - 1) & 3) + 1;  /* 1=reg, 2=TX, 3=RX */
}

/* Returns socket number for non-common BSBs (0-7) */
static int bsb_socket(uint8_t bsb) {
    if (bsb == 0) return -1;
    return (bsb - 1) >> 2;
}

/* ========================================================================
 * Socket command processing
 * ======================================================================== */

static void w5500_process_socket_cmd(w5500_t *dev, int sock) {
    w5500_socket_t *s = &dev->sockets[sock];
    uint8_t cmd = s->regs[W5500_Sn_CR];
    uint8_t mode = s->regs[W5500_Sn_MR];

    if (cmd == 0) return;

    switch (cmd) {
    case W5500_CMD_OPEN:
        if ((mode & 0x0F) == W5500_MR_TCP)
            s->regs[W5500_Sn_SR] = W5500_SOCK_INIT;
        else if ((mode & 0x0F) == W5500_MR_UDP)
            s->regs[W5500_Sn_SR] = W5500_SOCK_UDP;
        else if ((mode & 0x0F) == W5500_MR_MACRAW)
            s->regs[W5500_Sn_SR] = W5500_SOCK_MACRAW;
        /* TX free = full buffer size */
        s->regs[W5500_Sn_TX_FSR0] = (W5500_TX_BUF_SIZE >> 8) & 0xFF;
        s->regs[W5500_Sn_TX_FSR0 + 1] = W5500_TX_BUF_SIZE & 0xFF;
        break;

    case W5500_CMD_LISTEN:
        if (s->regs[W5500_Sn_SR] == W5500_SOCK_INIT)
            s->regs[W5500_Sn_SR] = W5500_SOCK_LISTEN;
        break;

    case W5500_CMD_CONNECT:
        if (s->regs[W5500_Sn_SR] == W5500_SOCK_INIT)
            s->regs[W5500_Sn_SR] = W5500_SOCK_ESTABLISHED;
        break;

    case W5500_CMD_CLOSE:
        s->regs[W5500_Sn_SR] = W5500_SOCK_CLOSED;
        break;

    case W5500_CMD_SEND:
        /* Simulate send complete: advance TX read pointer to write pointer */
        s->regs[W5500_Sn_TX_RD0] = s->regs[W5500_Sn_TX_WR0];
        s->regs[W5500_Sn_TX_RD0 + 1] = s->regs[W5500_Sn_TX_WR0 + 1];
        /* Restore full TX free space */
        s->regs[W5500_Sn_TX_FSR0] = (W5500_TX_BUF_SIZE >> 8) & 0xFF;
        s->regs[W5500_Sn_TX_FSR0 + 1] = W5500_TX_BUF_SIZE & 0xFF;
        /* Set SEND_OK interrupt */
        s->regs[W5500_Sn_IR] |= 0x10;
        break;

    case W5500_CMD_RECV:
        /* Advance RX read pointer, clear received size */
        s->regs[W5500_Sn_RX_RSR0] = 0;
        s->regs[W5500_Sn_RX_RSR0 + 1] = 0;
        break;

    default:
        break;
    }

    /* Command register auto-clears after execution */
    s->regs[W5500_Sn_CR] = 0x00;
}

/* ========================================================================
 * Read/write a single byte from the appropriate block
 * ======================================================================== */

static uint8_t w5500_read_byte(w5500_t *dev, uint8_t bsb, uint16_t addr) {
    int type = bsb_type(bsb);
    int sock = bsb_socket(bsb);

    switch (type) {
    case 0: /* Common registers */
        if (addr < W5500_COMMON_REG_SIZE)
            return dev->common[addr];
        return 0x00;

    case 1: /* Socket registers */
        if (sock >= 0 && sock < W5500_NUM_SOCKETS &&
            addr < W5500_SOCKET_REG_SIZE)
            return dev->sockets[sock].regs[addr];
        return 0x00;

    case 2: /* Socket TX buffer */
        if (sock >= 0 && sock < W5500_NUM_SOCKETS)
            return dev->sockets[sock].tx_buf[addr % W5500_TX_BUF_SIZE];
        return 0x00;

    case 3: /* Socket RX buffer */
        if (sock >= 0 && sock < W5500_NUM_SOCKETS)
            return dev->sockets[sock].rx_buf[addr % W5500_RX_BUF_SIZE];
        return 0x00;
    }

    return 0x00;
}

static void w5500_write_byte(w5500_t *dev, uint8_t bsb, uint16_t addr,
                             uint8_t val) {
    int type = bsb_type(bsb);
    int sock = bsb_socket(bsb);

    switch (type) {
    case 0: /* Common registers */
        if (addr == W5500_VERSIONR) return;  /* Read-only */
        if (addr < W5500_COMMON_REG_SIZE)
            dev->common[addr] = val;
        break;

    case 1: /* Socket registers */
        if (sock >= 0 && sock < W5500_NUM_SOCKETS &&
            addr < W5500_SOCKET_REG_SIZE) {
            dev->sockets[sock].regs[addr] = val;
            /* Process command register writes */
            if (addr == W5500_Sn_CR)
                w5500_process_socket_cmd(dev, sock);
        }
        break;

    case 2: /* Socket TX buffer */
        if (sock >= 0 && sock < W5500_NUM_SOCKETS)
            dev->sockets[sock].tx_buf[addr % W5500_TX_BUF_SIZE] = val;
        break;

    case 3: /* Socket RX buffer */
        if (sock >= 0 && sock < W5500_NUM_SOCKETS)
            dev->sockets[sock].rx_buf[addr % W5500_RX_BUF_SIZE] = val;
        break;
    }
}

/* ========================================================================
 * Initialization
 * ======================================================================== */

void w5500_init(w5500_t *dev) {
    memset(dev, 0, sizeof(*dev));

    /* Version register (read-only) */
    dev->common[W5500_VERSIONR] = 0x04;

    /* Default MAC: 02:00:00:00:00:01 */
    dev->common[W5500_SHAR0]     = 0x02;
    dev->common[W5500_SHAR0 + 1] = 0x00;
    dev->common[W5500_SHAR0 + 2] = 0x00;
    dev->common[W5500_SHAR0 + 3] = 0x00;
    dev->common[W5500_SHAR0 + 4] = 0x00;
    dev->common[W5500_SHAR0 + 5] = 0x01;

    /* IP defaults to 0.0.0.0 (already zero from memset) */

    /* PHY config: link up, 100Mbps full-duplex, not in reset */
    dev->common[W5500_PHYCFGR] = W5500_PHY_RST | W5500_PHY_LINK |
                                  W5500_PHY_SPD | W5500_PHY_DPX;

    /* Default retry time: 200ms (0x07D0 = 2000 * 100us) */
    dev->common[W5500_RTR0]     = 0x07;
    dev->common[W5500_RTR0 + 1] = 0xD0;

    /* Default retry count */
    dev->common[W5500_RCR] = 0x08;

    /* Initialize each socket with default buffer sizes and TTL */
    for (int i = 0; i < W5500_NUM_SOCKETS; i++) {
        w5500_socket_t *s = &dev->sockets[i];
        s->regs[W5500_Sn_RXBUF_SIZE] = 2;  /* 2KB */
        s->regs[W5500_Sn_TXBUF_SIZE] = 2;  /* 2KB */
        s->regs[W5500_Sn_TTL] = 128;
        s->regs[W5500_Sn_SR] = W5500_SOCK_CLOSED;
        /* TX free = full buffer */
        s->regs[W5500_Sn_TX_FSR0] = (W5500_TX_BUF_SIZE >> 8) & 0xFF;
        s->regs[W5500_Sn_TX_FSR0 + 1] = W5500_TX_BUF_SIZE & 0xFF;
    }
}

/* ========================================================================
 * SPI interface callbacks
 * ======================================================================== */

uint8_t w5500_spi_xfer(void *ctx, uint8_t mosi) {
    w5500_t *dev = (w5500_t *)ctx;
    uint8_t miso = 0xFF;

    if (!dev->cs_active) return 0xFF;

    switch (dev->phase) {
    case W5500_PHASE_ADDR_HI:
        dev->addr = (uint16_t)mosi << 8;
        dev->phase = W5500_PHASE_ADDR_LO;
        break;

    case W5500_PHASE_ADDR_LO:
        dev->addr |= mosi;
        dev->phase = W5500_PHASE_CONTROL;
        break;

    case W5500_PHASE_CONTROL:
        dev->bsb = (mosi >> 3) & 0x1F;
        dev->rw  = (mosi >> 2) & 1;
        dev->phase = W5500_PHASE_DATA;
        break;

    case W5500_PHASE_DATA:
        if (dev->rw) {
            /* Write */
            w5500_write_byte(dev, dev->bsb, dev->addr, mosi);
        } else {
            /* Read */
            miso = w5500_read_byte(dev, dev->bsb, dev->addr);
        }
        dev->addr++;
        break;
    }

    return miso;
}

void w5500_spi_cs(void *ctx, int cs_active) {
    w5500_t *dev = (w5500_t *)ctx;
    dev->cs_active = cs_active;

    if (cs_active) {
        /* CS asserted: reset frame state machine */
        dev->phase = W5500_PHASE_ADDR_HI;
    }
}
