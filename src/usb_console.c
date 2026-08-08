/*
 * USB CDC host console bridge (TCP / PTY).
 *
 * Standalone translation unit for upstream Bramble: no MegaFlash guest
 * stubs, no XMODEM, no SPI. Depends only on usb.h for usb_cdc_rx_push().
 */

#include "usb_console.h"
#include "usb.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#if !defined(_WIN32)
#include <termios.h>
#include <util.h>
#endif

typedef enum {
    USB_CONSOLE_OFF = 0,
    USB_CONSOLE_TCP,
    USB_CONSOLE_PTY,
} usb_console_transport_t;

typedef struct {
    usb_console_transport_t transport;
    int listen_fd;
    int client_fd;
    int port;
    char pty_slave_name[128];
    char pty_symlink[256];
} usb_console_bridge_t;

#define USB_CONSOLE_TX_PENDING_MAX 4096u

static usb_console_bridge_t usb_console;
static uint8_t usb_console_tx_pending[USB_CONSOLE_TX_PENDING_MAX];
static size_t usb_console_tx_pending_len;

static void usb_console_set_nonblock(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

static void usb_console_set_nodelay(int fd) {
    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
}

static void usb_console_tx_flush_pending(void) {
    if (usb_console.client_fd < 0 || usb_console_tx_pending_len == 0u) {
        return;
    }

    size_t off = 0;
    while (off < usb_console_tx_pending_len) {
        ssize_t n = write(usb_console.client_fd,
                          usb_console_tx_pending + off,
                          usb_console_tx_pending_len - off);
        if (n > 0) {
            off += (size_t)n;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
        }
        if (usb_console.transport == USB_CONSOLE_PTY) {
            break;
        }
        close(usb_console.client_fd);
        usb_console.client_fd = -1;
        usb_console_tx_pending_len = 0;
        return;
    }

    if (off > 0u) {
        memmove(usb_console_tx_pending,
                usb_console_tx_pending + off,
                usb_console_tx_pending_len - off);
        usb_console_tx_pending_len -= off;
    }
}

void usb_console_set_port(int port) {
    usb_console.port = port;
    usb_console.transport = port > 0 ? USB_CONSOLE_TCP : USB_CONSOLE_OFF;
}

void usb_console_set_pty(const char *symlink_path) {
    usb_console.transport = USB_CONSOLE_PTY;
    usb_console.port = 0;
    usb_console.pty_symlink[0] = '\0';
    if (symlink_path != NULL && symlink_path[0] != '\0') {
        strncpy(usb_console.pty_symlink, symlink_path,
                sizeof(usb_console.pty_symlink) - 1u);
        usb_console.pty_symlink[sizeof(usb_console.pty_symlink) - 1u] = '\0';
    }
}

int usb_console_active(void) {
    return usb_console.transport != USB_CONSOLE_OFF;
}

#if !defined(_WIN32)
static int usb_console_pty_init(void) {
    int master = -1;
    int slave = -1;
    char slave_name[sizeof(usb_console.pty_slave_name)];

    if (openpty(&master, &slave, slave_name, NULL, NULL) != 0) {
        fprintf(stderr, "[USB] PTY console: openpty failed: %s\n", strerror(errno));
        return -1;
    }
#if defined(__linux__)
    if (grantpt(slave) != 0 || unlockpt(slave) != 0) {
        fprintf(stderr, "[USB] PTY console: grant/unlock failed: %s\n",
                strerror(errno));
        close(master);
        close(slave);
        return -1;
    }
#endif

    usb_console_set_nonblock(master);
    struct termios tio;
    if (tcgetattr(master, &tio) == 0) {
        cfmakeraw(&tio);
        tio.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
        (void)tcsetattr(master, TCSANOW, &tio);
    }

    usb_console.client_fd = master;
    strncpy(usb_console.pty_slave_name, slave_name,
            sizeof(usb_console.pty_slave_name) - 1u);
    usb_console.pty_slave_name[sizeof(usb_console.pty_slave_name) - 1u] = '\0';

    const char *open_path = usb_console.pty_slave_name;
    if (usb_console.pty_symlink[0] != '\0') {
        unlink(usb_console.pty_symlink);
        if (symlink(usb_console.pty_slave_name, usb_console.pty_symlink) != 0) {
            fprintf(stderr, "[USB] PTY console: symlink %s -> %s failed: %s\n",
                    usb_console.pty_symlink, usb_console.pty_slave_name,
                    strerror(errno));
        } else {
            open_path = usb_console.pty_symlink;
            fprintf(stderr, "[USB] CDC console symlink: %s -> %s\n",
                    usb_console.pty_symlink, usb_console.pty_slave_name);
        }
    }

    fprintf(stderr,
            "[USB] CDC console on serial port %s\n"
            "[USB]   attach: screen %s 115200   (or cu -l %s -s 115200)\n",
            usb_console.pty_slave_name, open_path, open_path);

    /* Do not keep slave open — macOS clients cannot read it otherwise. */
    close(slave);

    if (usb_console_tx_pending_len > 0u) {
        usb_console_tx(usb_console_tx_pending,
                       (int)usb_console_tx_pending_len);
        usb_console_tx_pending_len = 0;
    }
    return 0;
}
#endif /* !_WIN32 */

int usb_console_init(void) {
    usb_console.listen_fd = -1;
    usb_console.client_fd = -1;
    usb_console.pty_slave_name[0] = '\0';
    usb_console_tx_pending_len = 0;

    if (usb_console.transport == USB_CONSOLE_OFF) {
        return 0;
    }

#if !defined(_WIN32)
    if (usb_console.transport == USB_CONSOLE_PTY) {
        return usb_console_pty_init();
    }
#endif

    if (usb_console.transport != USB_CONSOLE_TCP) {
        fprintf(stderr, "[USB] PTY console is not supported on this platform\n");
        return -1;
    }

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        fprintf(stderr, "[USB] TCP console: socket failed: %s\n", strerror(errno));
        return -1;
    }

    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)usb_console.port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        listen(fd, 1) < 0) {
        fprintf(stderr, "[USB] TCP console: listen on %d failed: %s\n",
                usb_console.port, strerror(errno));
        close(fd);
        return -1;
    }

    usb_console_set_nonblock(fd);
    usb_console.listen_fd = fd;
    fprintf(stderr,
            "[USB] CDC console listening on TCP port %d (nc localhost %d)\n",
            usb_console.port, usb_console.port);
    return 0;
}

void usb_console_cleanup(void) {
    if (usb_console.client_fd >= 0) {
        close(usb_console.client_fd);
        usb_console.client_fd = -1;
    }
    if (usb_console.listen_fd >= 0) {
        close(usb_console.listen_fd);
        usb_console.listen_fd = -1;
    }
    if (usb_console.pty_symlink[0] != '\0') {
        unlink(usb_console.pty_symlink);
    }
    usb_console.transport = USB_CONSOLE_OFF;
    usb_console_tx_pending_len = 0;
}

void usb_console_tx(const uint8_t *data, int len) {
    if (len <= 0) {
        return;
    }

    if (usb_console.client_fd < 0) {
        for (int i = 0; i < len; i++) {
            if (usb_console_tx_pending_len < USB_CONSOLE_TX_PENDING_MAX) {
                usb_console_tx_pending[usb_console_tx_pending_len++] = data[i];
            }
        }
        return;
    }

    if (usb_console_tx_pending_len > 0u) {
        usb_console_tx_flush_pending();
    }

    ssize_t off = 0;
    while (off < len) {
        ssize_t n = write(usb_console.client_fd, data + off, (size_t)(len - off));
        if (n > 0) {
            off += n;
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            for (ssize_t i = off; i < len; i++) {
                if (usb_console_tx_pending_len < USB_CONSOLE_TX_PENDING_MAX) {
                    usb_console_tx_pending[usb_console_tx_pending_len++] =
                        data[(size_t)i];
                }
            }
            break;
        }
        if (usb_console.transport == USB_CONSOLE_PTY) {
            for (ssize_t i = off; i < len; i++) {
                if (usb_console_tx_pending_len < USB_CONSOLE_TX_PENDING_MAX) {
                    usb_console_tx_pending[usb_console_tx_pending_len++] =
                        data[(size_t)i];
                }
            }
            break;
        }
        fprintf(stderr, "[USB] CDC console write error, disconnecting\n");
        close(usb_console.client_fd);
        usb_console.client_fd = -1;
        break;
    }
}

void usb_console_poll(void) {
    if (usb_console.transport == USB_CONSOLE_OFF) {
        return;
    }

    usb_console_tx_flush_pending();

    if (usb_console.transport == USB_CONSOLE_TCP &&
        usb_console.listen_fd >= 0 && usb_console.client_fd < 0) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int cfd = accept(usb_console.listen_fd,
                         (struct sockaddr *)&client_addr, &addr_len);
        if (cfd >= 0) {
            usb_console_set_nonblock(cfd);
            usb_console_set_nodelay(cfd);
            usb_console.client_fd = cfd;
            fprintf(stderr, "[USB] CDC console client connected from %s:%d\n",
                    inet_ntoa(client_addr.sin_addr),
                    ntohs(client_addr.sin_port));
            if (usb_console_tx_pending_len > 0u) {
                usb_console_tx(usb_console_tx_pending,
                               (int)usb_console_tx_pending_len);
                usb_console_tx_pending_len = 0;
            }
        }
    }

    if (usb_console.client_fd < 0) {
        return;
    }

    struct pollfd pfd = { .fd = usb_console.client_fd, .events = POLLIN };
    if (poll(&pfd, 1, 0) <= 0 || !(pfd.revents & POLLIN)) {
        return;
    }

    uint8_t buf[4096];
    for (;;) {
        ssize_t n = read(usb_console.client_fd, buf, sizeof(buf));
        if (n > 0) {
            for (ssize_t j = 0; j < n; j++) {
                if (!usb_cdc_rx_push(buf[j])) {
                    return;
                }
            }
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
        }
        if (n == 0) {
            if (usb_console.transport == USB_CONSOLE_PTY) {
                return;
            }
            fprintf(stderr, "[USB] CDC console client disconnected\n");
            close(usb_console.client_fd);
            usb_console.client_fd = -1;
        }
        break;
    }
}
