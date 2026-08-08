#ifndef USB_CONSOLE_H
#define USB_CONSOLE_H

#include <stdint.h>

/*
 * Host-side USB CDC console bridge (-usb-console).
 *
 * TCP: listen on a port; one client at a time (nc localhost <port>).
 * PTY:  virtual serial (screen/cu on symlink).
 *
 * RX: usb_console_poll() reads host bytes and pushes them into the guest
 *     via usb_cdc_rx_push() (declared in usb.h).
 * TX: usb.c calls usb_console_tx() from the CDC bulk IN handler.
 */

/* Configure before usb_console_init(). port > 0 enables TCP; 0 leaves off. */
void usb_console_set_port(int port);

/* Enable PTY transport; optional symlink (NULL or "" = no symlink). */
void usb_console_set_pty(const char *symlink_path);

int  usb_console_init(void);
void usb_console_cleanup(void);

int  usb_console_active(void);

/* Accept clients, flush pending TX, read host input → usb_cdc_rx_push. */
void usb_console_poll(void);

/* Guest CDC IN payload → host TCP/PTY (buffers until client connects). */
void usb_console_tx(const uint8_t *data, int len);

/* ------------------------------------------------------------------ */
/* Names used in the local Bramble tree (drop-in aliases).             */
/* ------------------------------------------------------------------ */

#define usb_console_tcp_set_port   usb_console_set_port
#define usb_console_tcp_init       usb_console_init
#define usb_console_tcp_cleanup    usb_console_cleanup
#define usb_console_tcp_active     usb_console_active
#define usb_console_tcp_poll       usb_console_poll
#define usb_console_tcp_tx         usb_console_tx

static inline void usb_console_tcp_poll_rx(int force_rx) {
    (void)force_rx;
    usb_console_poll();
}

#endif /* USB_CONSOLE_H */
