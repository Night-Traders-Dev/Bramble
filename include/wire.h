#ifndef WIRE_H
#define WIRE_H

#include <stddef.h>
#include <stdint.h>

/* ========================================================================
 * Bramble Wire Protocol - Inter-Instance Communication
 *
 * Connects multiple Bramble emulator instances for multi-device testing.
 * Each instance runs as a separate process, communicating via Unix domain
 * sockets or TCP.
 *
 * Wire types:
 *   UART: Cross-wires UART TX→RX between two instances
 *   GPIO: Mirrors GPIO pin state between instances
 *
 * Usage:
 *   Instance A: ./bramble fw_a.uf2 -wire-uart0 /tmp/bramble_uart0.sock
 *   Instance B: ./bramble fw_b.uf2 -wire-uart0 /tmp/bramble_uart0.sock
 *
 * The first instance to start creates the socket and listens.
 * The second instance connects. UART TX on either side is delivered
 * as UART RX on the other.
 * ======================================================================== */

#define WIRE_MAX_LINKS  4
#define WIRE_MAX_PAYLOAD 8
/* Large enough for a max-length Ethernet frame plus the 6-byte framing
 * header. At 256 the largest frame that could ever be delivered was
 * 250 bytes. */
#define WIRE_IO_BUFFER_SIZE (WIRE_ETH_MAX_FRAME + 16)
#define WIRE_ETH_MAX_FRAME 1522

/* Wire message types */
#define WIRE_MSG_UART_DATA  0x01  /* UART byte: payload = 1 byte */
#define WIRE_MSG_GPIO_PIN   0x02  /* GPIO pin change: pin(1) + value(1) */
#define WIRE_MSG_SPI_XFER   0x03  /* SPI byte exchange: tx(1), expects rx(1) */
#define WIRE_MSG_ETH_FRAME  0x04  /* Ethernet frame: 2-byte LE length + frame data */

/* Wire message header (4 bytes) */
typedef struct {
    uint8_t type;       /* WIRE_MSG_* */
    uint8_t channel;    /* UART num, SPI num, etc */
    uint8_t len;        /* Payload length */
    uint8_t reserved;
} wire_msg_t;

/* Wire link state */
typedef enum {
    WIRE_NONE,
    WIRE_LISTEN,    /* Waiting for peer to connect */
    WIRE_CONNECTED, /* Both peers connected */
} wire_state_t;

typedef struct {
    wire_state_t state;
    int listen_fd;
    int peer_fd;
    char path[256];     /* Socket path */
    uint8_t type;       /* What peripheral this wire connects (WIRE_MSG_*) */
    uint8_t channel;    /* Peripheral instance number */
    uint8_t rx_buf[WIRE_IO_BUFFER_SIZE];
    size_t rx_len;
    uint8_t tx_buf[WIRE_IO_BUFFER_SIZE];
    size_t tx_len;
} wire_link_t;

typedef struct {
    wire_link_t links[WIRE_MAX_LINKS];
    int link_count;
} wire_state_global_t;

extern wire_state_global_t wire_state;

/* Add a wire link (called during argument parsing) */
int wire_add_link(const char *path, uint8_t type, uint8_t channel);

/* Initialize all wire links (create/connect sockets) */
int wire_init(void);

/* Cleanup all sockets and remove files */
void wire_cleanup(void);

/* Poll for incoming data from peers (call from main loop) */
void wire_poll(void);

/* Send data to the peer on a specific wire */
void wire_send_uart(int uart_num, uint8_t byte);
void wire_send_gpio(uint8_t pin, uint8_t value);

/* Send an Ethernet frame to all ETH-type wire links.
 * Uses extended framing: header + 2-byte LE length + frame data. */
void wire_send_eth_frame(const uint8_t *frame, int len);

/* Returns 1 if UART has a wire link active */
int wire_uart_active(int uart_num);

/* Returns 1 if any ETH wire link is active */
int wire_eth_active(void);

#endif /* WIRE_H */
