#ifndef NETBRIDGE_H
#define NETBRIDGE_H

#include <stdint.h>

/* ========================================================================
 * Bramble Network Bridge
 *
 * Bridges UART peripherals to TCP sockets, enabling network communication
 * with the emulated RP2040. Supports both listen (server) and connect
 * (client) modes for each UART.
 *
 * Usage:
 *   -uart-console <port>     Listen on TCP (UART0); connect with nc/socat
 *   -net-uart0 <port>         Same as -uart-console
 *   -net-uart0-connect <host:port>  TCP client mode
 * ======================================================================== */

#define NET_BRIDGE_MAX_UART  2

/* Bridge mode */
typedef enum {
    NET_MODE_NONE,      /* No network bridge */
    NET_MODE_LISTEN,    /* TCP server: listen for connections */
    NET_MODE_CONNECT,   /* TCP client: connect to remote host */
} net_mode_t;

/* Per-UART bridge state */
typedef struct {
    net_mode_t mode;
    int listen_fd;      /* Server socket (listen mode) */
    int client_fd;      /* Connected client socket */
    int port;           /* TCP port (listen mode) */
    char host[256];     /* Remote host (connect mode) */
    int remote_port;    /* Remote port (connect mode) */
} net_uart_bridge_t;

/* Global bridge state */
typedef struct {
    net_uart_bridge_t uart[NET_BRIDGE_MAX_UART];
} net_bridge_t;

extern net_bridge_t net_bridge;

/* Initialize network bridge (call after argument parsing) */
int  net_bridge_init(void);

/* Cleanup sockets on exit */
void net_bridge_cleanup(void);

/* Poll for incoming connections and data (call from main loop) */
void net_bridge_poll(void);

/* Called by UART TX path to send a byte over the network */
void net_bridge_uart_tx(int uart_num, uint8_t byte);

/* Returns 1 if UART has a network bridge active */
int  net_bridge_uart_active(int uart_num);

/* Returns 1 if any UART has a TCP bridge configured */
int net_bridge_any_active(void);

/* When set, guest UART TX is also copied to host stderr */
extern int net_bridge_mirror_stdio;

#endif /* NETBRIDGE_H */
