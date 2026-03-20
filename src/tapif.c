/*
 * TAP Interface for CYW43 WiFi Bridge
 *
 * Creates a Linux TAP virtual network interface and configures the host
 * side for bridging Ethernet frames between the emulated CYW43 and the
 * host network.
 *
 * On open: creates TAP, assigns 192.168.4.1/24, brings interface UP,
 * enables IP forwarding, and sets up NAT masquerade on the default route.
 * On close: removes the NAT rule and restores forwarding state.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "tapif.h"

#ifdef __linux__
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_tun.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* The virtual subnet used by the CYW43 DHCP server */
#define TAP_HOST_IP     "192.168.4.1"
#define TAP_SUBNET      "192.168.4.0/24"
#define TAP_NETMASK     "255.255.255.0"

/* State for cleanup */
static char tap_ifname[IFNAMSIZ];
static int  tap_forwarding_was_enabled = 0;
static char tap_outgoing_iface[IFNAMSIZ];

/* Run a shell command, return 0 on success */
static int run_cmd(const char *cmd) {
    int rc = system(cmd);
    return WIFEXITED(rc) ? WEXITSTATUS(rc) : -1;
}

/* Detect the default outgoing interface for NAT masquerade */
static int detect_outgoing_iface(char *out, size_t out_sz) {
    FILE *f = popen("ip route show default 2>/dev/null | awk '/default/{print $5; exit}'", "r");
    if (!f) return -1;
    if (fgets(out, (int)out_sz, f) == NULL) {
        pclose(f);
        return -1;
    }
    pclose(f);
    /* Strip trailing newline */
    size_t len = strlen(out);
    if (len > 0 && out[len - 1] == '\n') out[len - 1] = '\0';
    return (out[0] != '\0') ? 0 : -1;
}

/* Check current IP forwarding state */
static int get_ip_forwarding(void) {
    FILE *f = fopen("/proc/sys/net/ipv4/ip_forward", "r");
    if (!f) return 0;
    int val = 0;
    if (fscanf(f, "%d", &val) != 1) val = 0;
    fclose(f);
    return val;
}

/* Set IP forwarding state */
static void set_ip_forwarding(int enable) {
    FILE *f = fopen("/proc/sys/net/ipv4/ip_forward", "w");
    if (f) {
        fprintf(f, "%d\n", enable ? 1 : 0);
        fclose(f);
    }
}

/* Copy interface name safely (always null-terminated, no truncation warning) */
static void copy_ifname(char *dst, const char *src) {
    memset(dst, 0, IFNAMSIZ);
    size_t len = strlen(src);
    if (len >= IFNAMSIZ) len = IFNAMSIZ - 1;
    memcpy(dst, src, len);
}

/* Assign IP address and bring interface UP using ioctl (no shell commands) */
static int configure_interface(const char *ifname) {
    struct ifreq ifr;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;

    /* Set IP address */
    memset(&ifr, 0, sizeof(ifr));
    copy_ifname(ifr.ifr_name, ifname);
    struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
    addr->sin_family = AF_INET;
    inet_pton(AF_INET, TAP_HOST_IP, &addr->sin_addr);
    if (ioctl(sock, SIOCSIFADDR, &ifr) < 0) {
        fprintf(stderr, "[TAP] Failed to set IP address: %s\n", strerror(errno));
        close(sock);
        return -1;
    }

    /* Set netmask */
    inet_pton(AF_INET, TAP_NETMASK, &addr->sin_addr);
    if (ioctl(sock, SIOCSIFNETMASK, &ifr) < 0) {
        fprintf(stderr, "[TAP] Failed to set netmask: %s\n", strerror(errno));
        close(sock);
        return -1;
    }

    /* Bring interface UP */
    memset(&ifr, 0, sizeof(ifr));
    copy_ifname(ifr.ifr_name, ifname);
    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
        close(sock);
        return -1;
    }
    ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
    if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0) {
        fprintf(stderr, "[TAP] Failed to bring interface up: %s\n", strerror(errno));
        close(sock);
        return -1;
    }

    close(sock);
    return 0;
}

/* Set up NAT masquerade for internet access */
static int setup_nat(void) {
    /* Detect outgoing interface */
    if (detect_outgoing_iface(tap_outgoing_iface, sizeof(tap_outgoing_iface)) < 0) {
        fprintf(stderr, "[TAP] No default route found — NAT not configured\n");
        fprintf(stderr, "[TAP] Local subnet 192.168.4.0/24 will still work\n");
        return 0;  /* Not fatal — local communication still works */
    }

    /* Save and enable IP forwarding */
    tap_forwarding_was_enabled = get_ip_forwarding();
    if (!tap_forwarding_was_enabled) {
        set_ip_forwarding(1);
        fprintf(stderr, "[TAP] Enabled IP forwarding\n");
    }

    /* Add iptables masquerade rule */
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "iptables -t nat -A POSTROUTING -s %s -o %s -j MASQUERADE 2>/dev/null",
             TAP_SUBNET, tap_outgoing_iface);
    if (run_cmd(cmd) != 0) {
        /* Try nftables if iptables not available */
        snprintf(cmd, sizeof(cmd),
                 "nft add rule nat postrouting oifname \"%s\" ip saddr %s masquerade 2>/dev/null",
                 tap_outgoing_iface, TAP_SUBNET);
        if (run_cmd(cmd) != 0) {
            fprintf(stderr, "[TAP] NAT setup failed (no iptables or nft) — local only\n");
            return 0;
        }
    }

    fprintf(stderr, "[TAP] NAT: %s → %s (masquerade)\n", TAP_SUBNET, tap_outgoing_iface);
    return 0;
}

/* Remove NAT rule on cleanup */
static void teardown_nat(void) {
    if (tap_outgoing_iface[0] == '\0') return;

    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "iptables -t nat -D POSTROUTING -s %s -o %s -j MASQUERADE 2>/dev/null",
             TAP_SUBNET, tap_outgoing_iface);
    run_cmd(cmd);

    /* Restore IP forwarding to previous state */
    if (!tap_forwarding_was_enabled) {
        set_ip_forwarding(0);
    }
}

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

    /* Save interface name for cleanup */
    copy_ifname(tap_ifname, ifr.ifr_name);

    fprintf(stderr, "[TAP] Interface '%s' created (fd=%d)\n", ifr.ifr_name, fd);

    /* Configure host side: IP, bring UP, NAT */
    if (configure_interface(ifr.ifr_name) == 0) {
        fprintf(stderr, "[TAP] Configured %s as %s/%s\n", ifr.ifr_name, TAP_HOST_IP, TAP_NETMASK);
    } else {
        fprintf(stderr, "[TAP] Auto-configuration failed. Manual setup:\n");
        fprintf(stderr, "[TAP]   sudo ip addr add %s/24 dev %s\n", TAP_HOST_IP, ifr.ifr_name);
        fprintf(stderr, "[TAP]   sudo ip link set %s up\n", ifr.ifr_name);
    }

    setup_nat();

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
    if (fd >= 0) {
#ifdef __linux__
        teardown_nat();
        fprintf(stderr, "[TAP] Closed interface '%s'\n", tap_ifname);
#endif
        close(fd);
    }
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
                return total > 0 ? total : 0;
            }
            return -1;
        }
        total += (int)n;
    }
    return total;
}
