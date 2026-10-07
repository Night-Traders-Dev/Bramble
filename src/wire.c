/*
 * Bramble Wire Protocol - Inter-Instance Communication
 *
 * Uses Unix domain sockets for low-latency IPC between emulator instances.
 * First instance to access a socket path creates it (server).
 * Second instance connects (client). Both are then peers.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "wire.h"
#include "uart.h"
#include "gpio.h"
#include "vnet.h"

wire_state_global_t wire_state;

static void set_nonblock(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

static void wire_reset_buffers(wire_link_t *l) {
    l->rx_len = 0;
    l->tx_len = 0;
}

static void wire_disconnect_peer(wire_link_t *l) {
    if (l->peer_fd >= 0) {
        close(l->peer_fd);
        l->peer_fd = -1;
    }
    wire_reset_buffers(l);
    l->state = (l->listen_fd >= 0) ? WIRE_LISTEN : WIRE_NONE;
}

static void wire_attach_peer(wire_link_t *l, int peer_fd) {
    set_nonblock(peer_fd);
    l->peer_fd = peer_fd;
    l->state = WIRE_CONNECTED;
    wire_reset_buffers(l);
}

int wire_add_link(const char *path, uint8_t type, uint8_t channel) {
    if (wire_state.link_count >= WIRE_MAX_LINKS) return -1;
    wire_link_t *l = &wire_state.links[wire_state.link_count++];
    l->state = WIRE_NONE;
    l->listen_fd = -1;
    l->peer_fd = -1;
    strncpy(l->path, path, sizeof(l->path) - 1);
    l->path[sizeof(l->path) - 1] = '\0';
    l->type = type;
    l->channel = channel;
    return 0;
}

static int try_connect(const char *path) {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    set_nonblock(fd);
    return fd;
}

static int create_listen(const char *path) {
    /* Remove stale socket file */
    unlink(path);

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    if (listen(fd, 1) < 0) {
        close(fd);
        unlink(path);
        return -1;
    }

    set_nonblock(fd);
    return fd;
}

int wire_init(void) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];

        /* Try to connect first (peer already listening?) */
        l->peer_fd = try_connect(l->path);
        if (l->peer_fd >= 0) {
            wire_attach_peer(l, l->peer_fd);
            fprintf(stderr, "[Wire] %s: connected to peer\n", l->path);
            continue;
        }

        /* No peer yet — create server socket */
        l->listen_fd = create_listen(l->path);
        if (l->listen_fd < 0) {
            fprintf(stderr, "[Wire] %s: failed to create socket: %s\n",
                    l->path, strerror(errno));
            return -1;
        }
        l->state = WIRE_LISTEN;
        fprintf(stderr, "[Wire] %s: waiting for peer\n", l->path);
    }
    return 0;
}

void wire_cleanup(void) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->peer_fd >= 0) {
            wire_disconnect_peer(l);
        }
        if (l->listen_fd >= 0) {
            close(l->listen_fd);
            l->listen_fd = -1;
            unlink(l->path);
        }
        l->state = WIRE_NONE;
    }
}

static void wire_handle_message(wire_link_t *l, wire_msg_t *msg, uint8_t *payload) {
    switch (msg->type) {
    case WIRE_MSG_UART_DATA:
        if (msg->len >= 1) {
            uart_rx_push(msg->channel, payload[0]);
        }
        break;

    case WIRE_MSG_GPIO_PIN:
        if (msg->len >= 2) {
            gpio_set_input_pin(payload[0], payload[1]);
        }
        break;

    case WIRE_MSG_ETH_FRAME:
        /* Extended framing: the msg->len field is unused for ETH frames.
         * The actual frame length was decoded from 2-byte LE prefix. */
        break;

    default:
        break;
    }
    (void)l;
}

static void wire_process_rx(wire_link_t *l) {
    while (l->rx_len >= sizeof(wire_msg_t)) {
        wire_msg_t msg;
        memcpy(&msg, l->rx_buf, sizeof(msg));

        /* Ethernet frames use extended framing */
        if (msg.type == WIRE_MSG_ETH_FRAME) {
            /* After the 4-byte header, expect 2-byte LE frame length + frame data */
            if (l->rx_len < (int)(sizeof(wire_msg_t) + 2)) return;

            uint16_t frame_len = (uint16_t)l->rx_buf[sizeof(wire_msg_t)] |
                                 ((uint16_t)l->rx_buf[sizeof(wire_msg_t) + 1] << 8);
            if (frame_len > WIRE_ETH_MAX_FRAME) {
                fprintf(stderr, "[Wire] %s: oversized ETH frame (%u)\n", l->path, frame_len);
                wire_disconnect_peer(l);
                return;
            }

            size_t total = sizeof(wire_msg_t) + 2 + frame_len;
            if ((size_t)l->rx_len < total) return;

            /* Deliver frame to vnet for distribution */
            const uint8_t *frame = l->rx_buf + sizeof(wire_msg_t) + 2;
            vnet_tx_frame(-1, frame, (int)frame_len);

            memmove(l->rx_buf, l->rx_buf + total, (size_t)l->rx_len - total);
            l->rx_len -= (int)total;
            continue;
        }

        /* Standard small-payload messages */
        if (msg.len > WIRE_MAX_PAYLOAD) {
            fprintf(stderr, "[Wire] %s: invalid payload length %u\n", l->path, msg.len);
            wire_disconnect_peer(l);
            return;
        }

        size_t frame_len = sizeof(wire_msg_t) + msg.len;
        if ((size_t)l->rx_len < frame_len) {
            return;
        }

        wire_handle_message(l, &msg, l->rx_buf + sizeof(wire_msg_t));
        memmove(l->rx_buf, l->rx_buf + frame_len, (size_t)l->rx_len - frame_len);
        l->rx_len -= (int)frame_len;
    }
}

static void wire_flush_tx(wire_link_t *l) {
    while (l->tx_len > 0) {
        ssize_t n = write(l->peer_fd, l->tx_buf, l->tx_len);
        if (n > 0) {
            if ((size_t)n < l->tx_len) {
                memmove(l->tx_buf, l->tx_buf + n, l->tx_len - (size_t)n);
            }
            l->tx_len -= (size_t)n;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;
        }

        fprintf(stderr, "[Wire] %s: peer disconnected\n", l->path);
        wire_disconnect_peer(l);
        return;
    }
}

void wire_poll(void) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];

        if (l->state == WIRE_NONE && l->listen_fd < 0) {
            int peer_fd = try_connect(l->path);
            if (peer_fd >= 0) {
                wire_attach_peer(l, peer_fd);
                fprintf(stderr, "[Wire] %s: reconnected to peer\n", l->path);
            }
        }

        /* Accept pending connections */
        if (l->state == WIRE_LISTEN && l->listen_fd >= 0) {
            int cfd = accept(l->listen_fd, NULL, NULL);
            if (cfd >= 0) {
                wire_attach_peer(l, cfd);
                fprintf(stderr, "[Wire] %s: peer connected\n", l->path);
            }
        }

        /* Read messages from peer */
        if (l->state == WIRE_CONNECTED && l->peer_fd >= 0) {
            struct pollfd pfd = {
                .fd = l->peer_fd,
                .events = POLLIN | (l->tx_len > 0 ? POLLOUT : 0)
            };
            if (poll(&pfd, 1, 0) <= 0) {
                continue;
            }

            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                fprintf(stderr, "[Wire] %s: peer disconnected\n", l->path);
                wire_disconnect_peer(l);
                continue;
            }

            if (pfd.revents & POLLOUT) {
                wire_flush_tx(l);
                if (l->state != WIRE_CONNECTED) {
                    continue;
                }
            }

            while (pfd.revents & POLLIN) {
                ssize_t n = read(l->peer_fd, l->rx_buf + l->rx_len,
                                 sizeof(l->rx_buf) - l->rx_len);
                if (n > 0) {
                    l->rx_len += (size_t)n;
                    wire_process_rx(l);
                    if (l->state != WIRE_CONNECTED) {
                        break;
                    }
                    if (l->rx_len == sizeof(l->rx_buf)) {
                        fprintf(stderr, "[Wire] %s: receive buffer overflow\n", l->path);
                        wire_disconnect_peer(l);
                        break;
                    }
                    pfd.revents = 0;
                    if (poll(&pfd, 1, 0) <= 0) {
                        break;
                    }
                    continue;
                }

                if (n == 0) {
                    fprintf(stderr, "[Wire] %s: peer disconnected\n", l->path);
                    wire_disconnect_peer(l);
                } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    fprintf(stderr, "[Wire] %s: read error: %s\n", l->path, strerror(errno));
                    wire_disconnect_peer(l);
                }
                break;
            }
        }
    }
}

static wire_link_t *find_link(uint8_t type, uint8_t channel) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->type == type && l->channel == channel && l->state == WIRE_CONNECTED) {
            return l;
        }
    }
    return NULL;
}

static void wire_send(wire_link_t *l, uint8_t type, uint8_t channel,
                       const uint8_t *data, uint8_t len) {
    if (!l || l->peer_fd < 0) return;
    if (len > WIRE_MAX_PAYLOAD) return;

    wire_msg_t msg = { .type = type, .channel = channel, .len = len, .reserved = 0 };
    uint8_t buf[sizeof(wire_msg_t) + WIRE_MAX_PAYLOAD];
    size_t frame_len = sizeof(msg) + len;

    wire_flush_tx(l);
    if (l->state != WIRE_CONNECTED) {
        return;
    }
    if (frame_len > sizeof(l->tx_buf) - l->tx_len) {
        fprintf(stderr, "[Wire] %s: transmit buffer full, dropping frame\n", l->path);
        return;
    }

    memcpy(buf, &msg, sizeof(msg));
    if (len > 0 && data) memcpy(buf + sizeof(msg), data, len);
    memcpy(l->tx_buf + l->tx_len, buf, frame_len);
    l->tx_len += frame_len;

    wire_flush_tx(l);
}

void wire_send_uart(int uart_num, uint8_t byte) {
    wire_link_t *l = find_link(WIRE_MSG_UART_DATA, (uint8_t)uart_num);
    if (l) wire_send(l, WIRE_MSG_UART_DATA, (uint8_t)uart_num, &byte, 1);
}

void wire_send_gpio(uint8_t pin, uint8_t value) {
    uint8_t payload[2] = { pin, value };
    /* Send to all GPIO wire links */
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->type == WIRE_MSG_GPIO_PIN && l->state == WIRE_CONNECTED) {
            wire_send(l, WIRE_MSG_GPIO_PIN, 0, payload, 2);
        }
    }
}

int wire_uart_active(int uart_num) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->type == WIRE_MSG_UART_DATA && l->channel == (uint8_t)uart_num) {
            return 1;
        }
    }
    return 0;
}

void wire_send_eth_frame(const uint8_t *frame, int len) {
    if (len <= 0 || len > WIRE_ETH_MAX_FRAME) return;

    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->type != WIRE_MSG_ETH_FRAME || l->state != WIRE_CONNECTED) continue;
        if (l->peer_fd < 0) continue;

        /* Extended framing: 4-byte header + 2-byte LE length + frame */
        wire_msg_t msg = { .type = WIRE_MSG_ETH_FRAME, .channel = 0, .len = 0, .reserved = 0 };
        uint8_t hdr[sizeof(wire_msg_t) + 2];
        memcpy(hdr, &msg, sizeof(msg));
        hdr[sizeof(msg)]     = (uint8_t)(len & 0xFF);
        hdr[sizeof(msg) + 1] = (uint8_t)((len >> 8) & 0xFF);

        /* Write header + frame (best-effort, non-blocking) */
        ssize_t n = write(l->peer_fd, hdr, sizeof(hdr));
        if (n == (ssize_t)sizeof(hdr)) {
            /* A short or failed body write desynchronises the peer's length
             * prefix, so treat it as a link error rather than dropping it.
             * EAGAIN here is benign -- the header already went out, so the
             * peer will time out on an incomplete frame. */
            ssize_t m = write(l->peer_fd, frame, (size_t)len);
            if (m != (ssize_t)len && errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "[Wire] %s: ETH body write error\n", l->path);
                wire_disconnect_peer(l);
            }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            fprintf(stderr, "[Wire] %s: ETH write error\n", l->path);
            wire_disconnect_peer(l);
        }
    }
}

int wire_eth_active(void) {
    for (int i = 0; i < wire_state.link_count; i++) {
        wire_link_t *l = &wire_state.links[i];
        if (l->type == WIRE_MSG_ETH_FRAME) {
            return 1;
        }
    }
    return 0;
}
