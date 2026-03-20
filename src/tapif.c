/*
 * TAP Interface for CYW43 WiFi Bridge
 *
 * Linux TAP device creation and non-blocking I/O.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "tapif.h"

#ifdef __linux__
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/if_tun.h>

int tapif_open(const char *name) {
    int fd = open("/dev/net/tun", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[TAP] Failed to open /dev/net/tun: %s\n", strerror(errno));
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    if (name && name[0])
        strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);

    if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
        fprintf(stderr, "[TAP] TUNSETIFF failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Set non-blocking */
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    fprintf(stderr, "[TAP] Interface '%s' opened (fd=%d)\n", ifr.ifr_name, fd);
    fprintf(stderr, "[TAP] To configure the interface, run in another terminal:\n");
    fprintf(stderr, "[TAP]   sudo ip addr add 192.168.4.1/24 dev %s\n", ifr.ifr_name);
    fprintf(stderr, "[TAP]   sudo ip link set %s up\n", ifr.ifr_name);
    return fd;
}

#else /* !__linux__ */

int tapif_open(const char *name) {
    (void)name;
    fprintf(stderr, "[TAP] TAP interface only supported on Linux\n");
    return -1;
}

#endif /* __linux__ */

void tapif_close(int fd) {
    if (fd >= 0) close(fd);
}

int tapif_read(int fd, uint8_t *buf, int maxlen) {
    if (fd < 0) return -1;
    /* Clamp to Ethernet max frame size (prevent oversized frames) */
    if (maxlen > 1518) maxlen = 1518;
    ssize_t n = read(fd, buf, (size_t)maxlen);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    return (int)n;
}

int tapif_write(int fd, const uint8_t *buf, int len) {
    if (fd < 0) return -1;
    int total = 0;
    while (total < len) {
        ssize_t n = write(fd, buf + total, (size_t)(len - total));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* Non-blocking: return what we wrote so far */
                return total > 0 ? total : 0;
            }
            return -1;
        }
        total += (int)n;
    }
    return total;
}
