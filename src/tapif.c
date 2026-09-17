/*
 * Host network interface for CYW43 WiFi bridge.
 *
 * Linux: TAP (/dev/net/tun) carrying raw Ethernet frames + iptables/nft NAT.
 * macOS: utun (L3) with Ethernet↔IPv4 adaptation + optional pf NAT helper.
 *
 * Virtual subnet (matches CYW43 fake DHCP):
 *   Guest  192.168.4.2
 *   Host   192.168.4.1
 *
 * Usage: ./bramble firmware.uf2 -wifi -tap <ifname>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "tapif.h"

#define TAP_HOST_IP     "192.168.4.1"
#define TAP_PEER_IP     "192.168.4.2"
#define TAP_SUBNET      "192.168.4.0/24"
#define TAP_NETMASK     "255.255.255.0"

static int run_cmd(const char *cmd) {
    int rc = system(cmd);
    return WIFEXITED(rc) ? WEXITSTATUS(rc) : -1;
}

#ifdef __linux__
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_tun.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/wait.h>

static char tap_ifname[IFNAMSIZ];
static int  tap_forwarding_was_enabled = 0;
static char tap_outgoing_iface[IFNAMSIZ];

static int detect_outgoing_iface(char *out, size_t out_sz) {
    FILE *f = popen("ip route show default 2>/dev/null | awk '/default/{print $5; exit}'", "r");
    if (!f) return -1;
    if (fgets(out, (int)out_sz, f) == NULL) {
        pclose(f);
        return -1;
    }
    pclose(f);
    size_t len = strlen(out);
    if (len > 0 && out[len - 1] == '\n') out[len - 1] = '\0';
    return (out[0] != '\0') ? 0 : -1;
}

static int get_ip_forwarding(void) {
    FILE *f = fopen("/proc/sys/net/ipv4/ip_forward", "r");
    if (!f) return 0;
    int val = 0;
    if (fscanf(f, "%d", &val) != 1) val = 0;
    fclose(f);
    return val;
}

static void set_ip_forwarding(int enable) {
    FILE *f = fopen("/proc/sys/net/ipv4/ip_forward", "w");
    if (f) {
        fprintf(f, "%d\n", enable ? 1 : 0);
        fclose(f);
    }
}

static void copy_ifname(char *dst, const char *src) {
    memset(dst, 0, IFNAMSIZ);
    size_t len = strlen(src);
    if (len >= IFNAMSIZ) len = IFNAMSIZ - 1;
    memcpy(dst, src, len);
}

static int configure_interface(const char *ifname) {
    struct ifreq ifr;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;

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

    inet_pton(AF_INET, TAP_NETMASK, &addr->sin_addr);
    if (ioctl(sock, SIOCSIFNETMASK, &ifr) < 0) {
        fprintf(stderr, "[TAP] Failed to set netmask: %s\n", strerror(errno));
        close(sock);
        return -1;
    }

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

static int setup_nat(void) {
    if (detect_outgoing_iface(tap_outgoing_iface, sizeof(tap_outgoing_iface)) < 0) {
        fprintf(stderr, "[TAP] No default route found — NAT not configured\n");
        fprintf(stderr, "[TAP] Local subnet 192.168.4.0/24 will still work\n");
        return 0;
    }

    tap_forwarding_was_enabled = get_ip_forwarding();
    if (!tap_forwarding_was_enabled) {
        set_ip_forwarding(1);
        fprintf(stderr, "[TAP] Enabled IP forwarding\n");
    }

    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "iptables -t nat -A POSTROUTING -s %s -o %s -j MASQUERADE 2>/dev/null",
             TAP_SUBNET, tap_outgoing_iface);
    if (run_cmd(cmd) != 0) {
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

static void teardown_nat(void) {
    if (tap_outgoing_iface[0] == '\0') return;

    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "iptables -t nat -D POSTROUTING -s %s -o %s -j MASQUERADE 2>/dev/null",
             TAP_SUBNET, tap_outgoing_iface);
    run_cmd(cmd);

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

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    copy_ifname(tap_ifname, ifr.ifr_name);
    fprintf(stderr, "[TAP] Interface '%s' created (fd=%d)\n", ifr.ifr_name, fd);

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

void tapif_close(int fd) {
    if (fd >= 0) {
        teardown_nat();
        fprintf(stderr, "[TAP] Closed interface '%s'\n", tap_ifname);
        close(fd);
    }
}

int tapif_read(int fd, uint8_t *buf, int maxlen) {
    if (fd < 0) return -1;
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

void tapif_service(int fd) {
    (void)fd;
}

void tapif_tftp_transfer_done(void) {
}

#elif defined(__APPLE__)

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sys_domain.h>
#include <sys/kern_control.h>
#include <sys/wait.h>
#include <net/if.h>
#include <net/if_utun.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <time.h>

/* CYW43 guest default MAC / fake BSSID used as gateway L2 address */
static const uint8_t k_guest_mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
static const uint8_t k_gw_mac[6]    = {0x02, 0xCA, 0xFE, 0xBA, 0xBE, 0x01};
static const uint8_t k_host_ip[4]   = {192, 168, 4, 1};

static char utun_ifname[IFNAMSIZ];
static int darwin_active;
static int darwin_utun_rx_ok; /* 0 = skip utun read; UDP NAT still active */

/* RX ring: ARP replies + userspace UDP NAT replies (and optional utun). */
#define DARWIN_RXQ 16
static uint8_t darwin_rxq[DARWIN_RXQ][1518];
static int darwin_rxq_len[DARWIN_RXQ];
static int darwin_rxq_head, darwin_rxq_tail, darwin_rxq_count;

/* Userspace UDP NAT — pf/utun does not reliably forward guest→internet on macOS. */
#define DARWIN_UDP_FLOWS 32
typedef struct {
    int fd;
    uint32_t guest_ip;    /* network order */
    uint16_t guest_port;  /* host order */
    uint32_t remote_ip;   /* network order */
    uint16_t remote_port; /* host order */
    time_t last_used;
} darwin_udp_flow_t;
static darwin_udp_flow_t darwin_udp_flows[DARWIN_UDP_FLOWS];

/*
 * TFTP ACK proxy: guest Thumb emu is too slow to ACK before tftpd times out
 * (even one DATA→ACK round-trip can exceed the server timeout → Unknown TID).
 * Host ACKs the server immediately, buffers blocks, and paces delivery to the
 * guest one DATA/OACK at a time (guest ACK advances the queue; not forwarded).
 */
#define TFTP_PROXY_QMAX 256 /* buffered blocks awaiting guest */
#define TFTP_PROXY_WINDOW 32 /* max host-ACKed ahead of guest (keep tftpd alive) */
typedef struct {
    uint32_t remote_ip;   /* network order */
    uint16_t remote_port; /* host order (server TID) */
    uint16_t len;
    uint8_t payload[516];
} tftp_proxy_pkt_t;

static tftp_proxy_pkt_t tftp_proxy_q[TFTP_PROXY_QMAX];
static int tftp_proxy_q_head;
static int tftp_proxy_q_count;
static int tftp_proxy_active;
static int tftp_proxy_fd = -1;
static int tftp_proxy_guest_inflight; /* 1 = guest has unacked DATA/OACK */
static uint16_t tftp_proxy_last_guest_ack;
static uint32_t tftp_proxy_guest_ip;
static uint16_t tftp_proxy_guest_port;
static uint32_t tftp_proxy_remote_ip;
static unsigned tftp_proxy_host_acks;
static unsigned tftp_proxy_guest_acks;
static uint16_t tftp_proxy_last_enqueued; /* last DATA/OACK block buffered */
static int tftp_proxy_have_enqueued;
static uint16_t tftp_proxy_last_acked; /* last block host-ACKed (keepalive) */
static uint16_t tftp_proxy_server_tid; /* server ephemeral port */
static struct timespec tftp_proxy_last_rx;
static int tftp_proxy_last_rx_armed;
static int tftp_proxy_transfer_done;
static void tftp_proxy_reset(void) {
    tftp_proxy_q_head = 0;
    tftp_proxy_q_count = 0;
    tftp_proxy_active = 0;
    tftp_proxy_fd = -1;
    tftp_proxy_guest_inflight = 0;
    tftp_proxy_last_guest_ack = 0;
    tftp_proxy_guest_ip = 0;
    tftp_proxy_guest_port = 0;
    tftp_proxy_remote_ip = 0;
    tftp_proxy_host_acks = 0;
    tftp_proxy_guest_acks = 0;
    tftp_proxy_last_enqueued = 0;
    tftp_proxy_have_enqueued = 0;
    tftp_proxy_last_acked = 0;
    tftp_proxy_server_tid = 0;
    tftp_proxy_last_rx_armed = 0;
    tftp_proxy_transfer_done = 0;
}

/* True if blk is a retransmit of last (handles uint16 wrap; 0 after 65535
 * is the *next* block, not a retransmit of 65535). */
static int tftp_proxy_is_retransmit(uint16_t blk) {
    uint16_t behind;
    if (!tftp_proxy_have_enqueued)
        return 0;
    if (blk == tftp_proxy_last_enqueued)
        return 1;
    behind = (uint16_t)(tftp_proxy_last_enqueued - blk);
    return behind >= 1u && behind <= 32u;
}

static void darwin_queue_udp_reply(uint32_t guest_ip, uint16_t guest_port,
                                   uint32_t remote_ip, uint16_t remote_port,
                                   const uint8_t *payload, int payload_len);

static void tftp_proxy_host_ack(int fd, uint32_t remote_ip, uint16_t remote_port,
                                uint16_t block) {
    uint8_t ack[4];
    struct sockaddr_in dest;
    ack[0] = 0;
    ack[1] = 4; /* ACK */
    ack[2] = (uint8_t)(block >> 8);
    ack[3] = (uint8_t)(block & 0xFF);
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(remote_port);
    dest.sin_addr.s_addr = remote_ip;
    if (sendto(fd, ack, 4, 0, (struct sockaddr *)&dest, sizeof(dest)) < 0) {
        fprintf(stderr, "[TAP] TFTP host-ACK block %u failed: %s\n",
                (unsigned)block, strerror(errno));
    } else {
        tftp_proxy_host_acks++;
        tftp_proxy_last_acked = block;
        if (tftp_proxy_host_acks <= 8 || (tftp_proxy_host_acks % 64) == 0) {
            fprintf(stderr, "[TAP] TFTP host-ACK block %u → :%u (#%u)\n",
                    (unsigned)block, (unsigned)remote_port,
                    tftp_proxy_host_acks);
            fflush(stderr);
        }
    }
}

static void tftp_proxy_deliver_one(void) {
    tftp_proxy_pkt_t *p;
    if (tftp_proxy_guest_inflight || tftp_proxy_q_count <= 0)
        return;
    p = &tftp_proxy_q[tftp_proxy_q_head];
    darwin_queue_udp_reply(tftp_proxy_guest_ip, tftp_proxy_guest_port,
                           p->remote_ip, p->remote_port,
                           p->payload, (int)p->len);
    tftp_proxy_guest_inflight = 1;
    {
        uint16_t op = ((uint16_t)p->payload[0] << 8) | p->payload[1];
        uint16_t blk = ((uint16_t)p->payload[2] << 8) | p->payload[3];
        static int del_logged;
        if (del_logged < 12 || (blk % 64) == 1) {
            del_logged++;
            fprintf(stderr,
                    "[TAP] TFTP deliver to guest op=%u block %u (%u bytes) q=%d\n",
                    (unsigned)op, (unsigned)blk, (unsigned)p->len,
                    tftp_proxy_q_count);
            fflush(stderr);
        }
    }
    tftp_proxy_q_head = (tftp_proxy_q_head + 1) % TFTP_PROXY_QMAX;
    tftp_proxy_q_count--;
}

static int tftp_proxy_enqueue(uint32_t rip, uint16_t rport,
                              const uint8_t *payload, int len) {
    tftp_proxy_pkt_t *p;
    int tail;
    if (len < 4 || len > 516)
        return 0;
    if (tftp_proxy_q_count >= TFTP_PROXY_QMAX)
        return 0;
    tail = (tftp_proxy_q_head + tftp_proxy_q_count) % TFTP_PROXY_QMAX;
    p = &tftp_proxy_q[tail];
    p->remote_ip = rip;
    p->remote_port = rport;
    p->len = (uint16_t)len;
    memcpy(p->payload, payload, (size_t)len);
    tftp_proxy_q_count++;
    return 1;
}

/* Recv TFTP on the proxy flow: host-ACK, buffer, pace to guest. */
static void tftp_proxy_poll_flow(darwin_udp_flow_t *f) {
    uint8_t payload[1400];
    int got = 0;
    for (;;) {
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        ssize_t n;
        uint16_t op, blk, rport;
        uint32_t rip;

        /* Host-apply path never enqueues — skip the guest catch-up gate so a
         * full proxy queue cannot pause ACKs mid-download. */
        if (tftp_proxy_q_count >= TFTP_PROXY_WINDOW)
            break; /* don't host-ACK further until guest catches up */

        n = recvfrom(f->fd, payload, sizeof(payload), 0,
                     (struct sockaddr *)&from, &fromlen);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            close(f->fd);
            f->fd = -1;
            tftp_proxy_reset();
            break;
        }
        got = 1;
        rip = from.sin_addr.s_addr;
        rport = ntohs(from.sin_port);
        f->last_used = time(NULL);
        if (f->remote_port == 69u && rport != 69u)
            f->remote_port = rport;
        tftp_proxy_remote_ip = rip;
        tftp_proxy_server_tid = rport;
        clock_gettime(CLOCK_MONOTONIC, &tftp_proxy_last_rx);
        tftp_proxy_last_rx_armed = 1;

        if (n < 2)
            continue;
        op = ((uint16_t)payload[0] << 8) | payload[1];

        {
            static int tftp_rx_hex;
            if (tftp_rx_hex < 16 && op >= 1 && op <= 6) {
                int hi, hn = n < 48 ? (int)n : 48;
                tftp_rx_hex++;
                fprintf(stderr,
                        "[TAP] TFTP RX from :%u op=%u (%zd bytes) hex:",
                        (unsigned)rport, (unsigned)op, n);
                for (hi = 0; hi < hn; hi++)
                    fprintf(stderr, " %02x", payload[hi]);
                if (n > hn)
                    fprintf(stderr, " …");
                if (op == 5 && n > 4)
                    fprintf(stderr, " msg='%.*s'", (int)n - 4,
                            (const char *)payload + 4);
                fprintf(stderr, "\n");
                fflush(stderr);
            }
        }

        if (op == 3 || op == 6) {
            /* DATA or OACK */
            blk = (op == 6) ? 0u : (((uint16_t)payload[2] << 8) | payload[3]);
            if (n > 516)
                n = 516;

            /* Retransmit of last applied — ACK only (wrap-safe). */
            if (tftp_proxy_is_retransmit(blk)) {
                tftp_proxy_host_ack(f->fd, rip, rport, blk);
                continue;
            }

            /* Host-ACK then queue for guest CYW43 delivery. */
            tftp_proxy_host_ack(f->fd, rip, rport, blk);
            if (!tftp_proxy_enqueue(rip, rport, payload, (int)n)) {
                fprintf(stderr, "[TAP] TFTP proxy queue full — drop block %u\n",
                        (unsigned)blk);
                fflush(stderr);
            } else {
                tftp_proxy_last_enqueued = blk;
                tftp_proxy_have_enqueued = 1;
            }
            continue;
        }
        if (op == 5) {
            /* Server ERROR while proxying. If transfer already done, ignore;
             * otherwise log and stop keepalive (Unknown TID = session dead). */
            static int err_drop;
            if (tftp_proxy_transfer_done)
                continue;
            tftp_proxy_last_rx_armed = 0; /* stop keepalive spam */
            if (err_drop < 8) {
                err_drop++;
                fprintf(stderr,
                        "[TAP] TFTP drop ERROR from server while proxying "
                        "(%zd bytes)",
                        n);
                if (n > 4)
                    fprintf(stderr, " msg='%.*s'", (int)n - 4,
                            (const char *)payload + 4);
                fprintf(stderr, "\n");
                fflush(stderr);
            }
            continue;
        }
        /* Unexpected: pass through. */
        darwin_queue_udp_reply(f->guest_ip, f->guest_port, rip, rport,
                               payload, (int)n);
    }
    /* Keepalive: re-ACK last block if idle so tftpd does not give up while
     * host-apply is between packets or guest core0 has aborted. */
    if (!got && !tftp_proxy_transfer_done &&
        tftp_proxy_have_enqueued && tftp_proxy_last_rx_armed &&
        tftp_proxy_server_tid != 0u && f->fd >= 0) {
        struct timespec now;
        uint64_t idle_ms;
        if (clock_gettime(CLOCK_MONOTONIC, &now) == 0) {
            idle_ms = (uint64_t)(now.tv_sec - tftp_proxy_last_rx.tv_sec) * 1000ull;
            if (now.tv_nsec >= tftp_proxy_last_rx.tv_nsec)
                idle_ms += (uint64_t)(now.tv_nsec - tftp_proxy_last_rx.tv_nsec)
                           / 1000000ull;
            else {
                idle_ms -= 1000ull;
                idle_ms += (uint64_t)(now.tv_nsec + 1000000000L
                                     - tftp_proxy_last_rx.tv_nsec)
                           / 1000000ull;
            }
            if (idle_ms >= 2000ull) {
                static unsigned ka;
                tftp_proxy_host_ack(f->fd, tftp_proxy_remote_ip
                                            ? tftp_proxy_remote_ip
                                            : f->remote_ip,
                                    tftp_proxy_server_tid,
                                    tftp_proxy_last_acked
                                        ? tftp_proxy_last_acked
                                        : tftp_proxy_last_enqueued);
                clock_gettime(CLOCK_MONOTONIC, &tftp_proxy_last_rx);
                if (ka < 8u || (ka % 32u) == 0u) {
                    fprintf(stderr,
                            "[TAP] TFTP keepalive ACK block %u (idle %llums)\n",
                            (unsigned)tftp_proxy_last_acked,
                            (unsigned long long)idle_ms);
                    fflush(stderr);
                }
                ka++;
            }
        }
    }
    tftp_proxy_deliver_one();
}

static void queue_pending_eth(const uint8_t *frame, int len) {
    if (len <= 0 || len > 1518) return;
    if (darwin_rxq_count >= DARWIN_RXQ) {
        /* Drop oldest */
        darwin_rxq_head = (darwin_rxq_head + 1) % DARWIN_RXQ;
        darwin_rxq_count--;
    }
    memcpy(darwin_rxq[darwin_rxq_tail], frame, (size_t)len);
    darwin_rxq_len[darwin_rxq_tail] = len;
    darwin_rxq_tail = (darwin_rxq_tail + 1) % DARWIN_RXQ;
    darwin_rxq_count++;
}

static int dequeue_pending_eth(uint8_t *buf, int maxlen) {
    if (darwin_rxq_count <= 0) return 0;
    int n = darwin_rxq_len[darwin_rxq_head];
    if (n > maxlen) n = maxlen;
    memcpy(buf, darwin_rxq[darwin_rxq_head], (size_t)n);
    darwin_rxq_head = (darwin_rxq_head + 1) % DARWIN_RXQ;
    darwin_rxq_count--;
    return n;
}

static uint16_t ipv4_checksum(const uint8_t *hdr, int len) {
    uint32_t sum = 0;
    for (int i = 0; i < len; i += 2)
        sum += ((uint32_t)hdr[i] << 8) | hdr[i + 1];
    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)(~sum);
}

static void darwin_udp_nat_close_all(void) {
    for (int i = 0; i < DARWIN_UDP_FLOWS; i++) {
        if (darwin_udp_flows[i].fd >= 0) {
            close(darwin_udp_flows[i].fd);
            darwin_udp_flows[i].fd = -1;
        }
    }
    tftp_proxy_reset();
}

static darwin_udp_flow_t *darwin_udp_flow_find(uint32_t gip, uint16_t gport,
                                               uint32_t rip, uint16_t rport) {
    for (int i = 0; i < DARWIN_UDP_FLOWS; i++) {
        darwin_udp_flow_t *f = &darwin_udp_flows[i];
        if (f->fd < 0) continue;
        if (f->guest_ip == gip && f->guest_port == gport &&
            f->remote_ip == rip && f->remote_port == rport)
            return f;
    }
    /* TFTP TID: server replies from ephemeral port; reuse the :69 socket so
     * host source port stays the same (else server returns Unknown transfer ID). */
    if (rport != 69u) {
        for (int i = 0; i < DARWIN_UDP_FLOWS; i++) {
            darwin_udp_flow_t *f = &darwin_udp_flows[i];
            if (f->fd < 0) continue;
            if (f->guest_ip == gip && f->guest_port == gport &&
                f->remote_ip == rip && f->remote_port == 69u) {
                f->remote_port = rport;
                return f;
            }
        }
    }
    /* Guest still ACKs to :69 after we rebound the flow to the server TID. */
    if (rport == 69u && tftp_proxy_active && tftp_proxy_fd >= 0) {
        for (int i = 0; i < DARWIN_UDP_FLOWS; i++) {
            darwin_udp_flow_t *f = &darwin_udp_flows[i];
            if (f->fd != tftp_proxy_fd) continue;
            if (f->guest_ip == gip && f->guest_port == gport &&
                f->remote_ip == rip)
                return f;
        }
    }
    return NULL;
}

static darwin_udp_flow_t *darwin_udp_flow_alloc(void) {
    for (int i = 0; i < DARWIN_UDP_FLOWS; i++) {
        if (darwin_udp_flows[i].fd < 0)
            return &darwin_udp_flows[i];
    }
    /* Evict oldest — never steal the active TFTP proxy socket. */
    int oldest = -1;
    for (int i = 0; i < DARWIN_UDP_FLOWS; i++) {
        if (tftp_proxy_active && darwin_udp_flows[i].fd == tftp_proxy_fd)
            continue;
        if (oldest < 0 ||
            darwin_udp_flows[i].last_used < darwin_udp_flows[oldest].last_used)
            oldest = i;
    }
    if (oldest < 0)
        oldest = 0;
    close(darwin_udp_flows[oldest].fd);
    darwin_udp_flows[oldest].fd = -1;
    return &darwin_udp_flows[oldest];
}

/* Build Ethernet+IPv4+UDP reply toward guest and queue it. */
static void darwin_queue_udp_reply(uint32_t guest_ip, uint16_t guest_port,
                                   uint32_t remote_ip, uint16_t remote_port,
                                   const uint8_t *payload, int payload_len) {
    if (payload_len < 0 || payload_len > 1400) return;

    uint8_t frame[1518];
    memset(frame, 0, sizeof(frame));
    int off = 0;

    memcpy(frame + off, k_guest_mac, 6); off += 6;
    memcpy(frame + off, k_gw_mac, 6);    off += 6;
    frame[off++] = 0x08; frame[off++] = 0x00;

    int ip_off = off;
    frame[off++] = 0x45;
    frame[off++] = 0x00;
    int ip_len_off = off; off += 2;
    frame[off++] = 0x00; frame[off++] = 0x00; /* id */
    frame[off++] = 0x40; frame[off++] = 0x00; /* DF */
    frame[off++] = 64;
    frame[off++] = 17; /* UDP */
    int ip_csum_off = off; off += 2;
    memcpy(frame + off, &remote_ip, 4); off += 4;
    memcpy(frame + off, &guest_ip, 4);  off += 4;

    int udp_off = off;
    frame[off++] = (uint8_t)(remote_port >> 8);
    frame[off++] = (uint8_t)(remote_port & 0xFF);
    frame[off++] = (uint8_t)(guest_port >> 8);
    frame[off++] = (uint8_t)(guest_port & 0xFF);
    int udp_len_off = off; off += 2;
    frame[off++] = 0; frame[off++] = 0; /* UDP checksum = 0 (IPv4 OK) */
    memcpy(frame + off, payload, (size_t)payload_len);
    off += payload_len;

    int udp_total = off - udp_off;
    int ip_total = off - ip_off;
    frame[udp_len_off]     = (uint8_t)(udp_total >> 8);
    frame[udp_len_off + 1] = (uint8_t)(udp_total & 0xFF);
    frame[ip_len_off]      = (uint8_t)(ip_total >> 8);
    frame[ip_len_off + 1]  = (uint8_t)(ip_total & 0xFF);

    uint16_t csum = ipv4_checksum(frame + ip_off, 20);
    frame[ip_csum_off]     = (uint8_t)(csum >> 8);
    frame[ip_csum_off + 1] = (uint8_t)(csum & 0xFF);

    queue_pending_eth(frame, off);

    {
        static int rx_logged;
        if (rx_logged < 12) {
            rx_logged++;
            const uint8_t *r = (const uint8_t *)&remote_ip;
            fprintf(stderr,
                    "[TAP] UDP NAT ← %u.%u.%u.%u:%u (%d payload bytes)\n",
                    r[0], r[1], r[2], r[3], (unsigned)remote_port, payload_len);
            fflush(stderr);
        }
    }
}

/* Poll host UDP sockets; queue any replies as Ethernet frames. */
static void darwin_udp_nat_poll(void) {
    uint8_t payload[1400];
    for (int i = 0; i < DARWIN_UDP_FLOWS; i++) {
        darwin_udp_flow_t *f = &darwin_udp_flows[i];
        if (f->fd < 0) continue;
        if (tftp_proxy_active && f->fd == tftp_proxy_fd) {
            tftp_proxy_poll_flow(f);
            continue;
        }
        for (;;) {
            struct sockaddr_in from;
            socklen_t fromlen = sizeof(from);
            ssize_t n = recvfrom(f->fd, payload, sizeof(payload), 0,
                                 (struct sockaddr *)&from, &fromlen);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                close(f->fd);
                f->fd = -1;
                break;
            }
            uint32_t rip = from.sin_addr.s_addr;
            uint16_t rport = ntohs(from.sin_port);
            f->last_used = time(NULL);
            if (f->remote_port == 69u && rport != 69u)
                f->remote_port = rport;
            darwin_queue_udp_reply(f->guest_ip, f->guest_port,
                                   rip, rport, payload, (int)n);
        }
    }
}

/* Forward guest UDP via host socket. Returns 1 if handled. */
static int darwin_udp_nat_tx(const uint8_t *eth, int len) {
    if (len < 14 + 20 + 8) return 0;
    if (eth[12] != 0x08 || eth[13] != 0x00) return 0;

    const uint8_t *ip = eth + 14;
    if ((ip[0] >> 4) != 4) return 0;
    int ihl = (ip[0] & 0x0F) * 4;
    if (ihl < 20 || ip[9] != 17) return 0; /* not UDP */
    if (len < 14 + ihl + 8) return 0;

    const uint8_t *udp = ip + ihl;
    uint16_t sport = ((uint16_t)udp[0] << 8) | udp[1];
    uint16_t dport = ((uint16_t)udp[2] << 8) | udp[3];
    uint16_t ulen  = ((uint16_t)udp[4] << 8) | udp[5];
    int payload_len = (int)ulen - 8;
    if (payload_len < 0) return 0;
    if (14 + ihl + 8 + payload_len > len)
        payload_len = len - 14 - ihl - 8;

    uint32_t sip, dip;
    memcpy(&sip, ip + 12, 4);
    memcpy(&dip, ip + 16, 4);

    /* Leave packets aimed at the virtual gateway to utun (rare). */
    if (memcmp(&dip, k_host_ip, 4) == 0)
        return 0;

    darwin_udp_flow_t *f = darwin_udp_flow_find(sip, sport, dip, dport);
    if (!f) {
        f = darwin_udp_flow_alloc();
        int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s < 0) {
            fprintf(stderr, "[TAP] UDP NAT socket failed: %s\n", strerror(errno));
            return 1; /* consumed but failed */
        }
        int flags = fcntl(s, F_GETFL, 0);
        if (flags != -1)
            fcntl(s, F_SETFL, flags | O_NONBLOCK);
        {
            int rcv = 1 << 20; /* 1MB — large TFTP downloads need headroom */
            setsockopt(s, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));
        }
        f->fd = s;
        f->guest_ip = sip;
        f->guest_port = sport;
        f->remote_ip = dip;
        f->remote_port = dport;
    }
    f->last_used = time(NULL);

    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(dport);
    dest.sin_addr.s_addr = dip;

    const uint8_t *payload = udp + 8;
    int suppress_send = 0;
    if (payload_len >= 2) {
        uint16_t op = ((uint16_t)payload[0] << 8) | payload[1];
        if (op == 1 || op == 2) {
            /* New RRQ/WRQ — arm proxy on this flow. */
            tftp_proxy_reset();
            tftp_proxy_active = 1;
            tftp_proxy_fd = f->fd;
            tftp_proxy_guest_ip = sip;
            tftp_proxy_guest_port = sport;
            tftp_proxy_remote_ip = dip;
            fprintf(stderr, "[TAP] TFTP proxy armed (RRQ/WRQ → :%u)\n",
                    (unsigned)dport);
            fflush(stderr);
        } else if (op == 4 && payload_len >= 4 && tftp_proxy_active &&
                   f->fd == tftp_proxy_fd) {
            /* Guest ACK — already host-ACKed; advance guest queue only. */
            uint16_t blk = ((uint16_t)payload[2] << 8) | payload[3];
            tftp_proxy_last_guest_ack = blk;
            tftp_proxy_guest_acks++;
            tftp_proxy_guest_inflight = 0;
            suppress_send = 1;
            if (tftp_proxy_guest_acks <= 8 || (tftp_proxy_guest_acks % 64) == 0) {
                fprintf(stderr,
                        "[TAP] TFTP guest-ACK block %u (host already ACKed; "
                        "q=%d)\n",
                        (unsigned)blk, tftp_proxy_q_count);
                fflush(stderr);
            }
            tftp_proxy_deliver_one();
        } else if (op == 5 && tftp_proxy_active && f->fd == tftp_proxy_fd) {
            /* Guest ERROR (timeout/abort) would kill tftpd while host-apply
             * still owns the session — drop it. */
            static int guest_err;
            suppress_send = 1;
            if (guest_err < 8) {
                guest_err++;
                fprintf(stderr,
                        "[TAP] TFTP drop guest ERROR while host-proxying\n");
                fflush(stderr);
            }
        }
    }
    if (!suppress_send) {
        ssize_t n = sendto(f->fd, payload, (size_t)payload_len, 0,
                           (struct sockaddr *)&dest, sizeof(dest));
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            fprintf(stderr, "[TAP] UDP NAT sendto failed: %s\n", strerror(errno));
        } else {
            static int tx_logged;
            int is_tftp = (dport == 69u) ||
                          (payload_len >= 2 &&
                           (((uint16_t)payload[0] << 8) | payload[1]) <= 6);
            if (tx_logged < 24 || is_tftp) {
                if (tx_logged < 24)
                    tx_logged++;
                const uint8_t *d = (const uint8_t *)&dip;
                fprintf(stderr,
                        "[TAP] UDP NAT → %u.%u.%u.%u:%u (%d payload bytes)\n",
                        d[0], d[1], d[2], d[3], (unsigned)dport, payload_len);
                if (is_tftp && payload_len > 0) {
                    int i, hn = payload_len < 48 ? payload_len : 48;
                    fprintf(stderr, "[TAP] TFTP TX hex:");
                    for (i = 0; i < hn; i++)
                        fprintf(stderr, " %02x", payload[i]);
                    if (payload_len > hn)
                        fprintf(stderr, " …");
                    fprintf(stderr, "\n");
                }
                fflush(stderr);
            }
        }
    }
    return 1;
}

/* Answer ARP who-has 192.168.4.1 — return 1 if handled. */
static int darwin_handle_arp(const uint8_t *eth, int len) {
    if (len < 42) return 0;
    if (eth[12] != 0x08 || eth[13] != 0x06) return 0; /* not ARP */

    const uint8_t *arp = eth + 14;
    uint16_t op = ((uint16_t)arp[6] << 8) | arp[7];
    if (op != 1) return 0; /* not request */

    const uint8_t *tpa = arp + 24; /* target protocol address */
    if (memcmp(tpa, k_host_ip, 4) != 0) return 0;

    uint8_t reply[42];
    memset(reply, 0, sizeof(reply));
    /* Ethernet: dst=requester, src=gw */
    memcpy(reply, eth + 6, 6);
    memcpy(reply + 6, k_gw_mac, 6);
    reply[12] = 0x08;
    reply[13] = 0x06;
    /* ARP reply */
    reply[14] = 0x00; reply[15] = 0x01; /* HTYPE Ethernet */
    reply[16] = 0x08; reply[17] = 0x00; /* PTYPE IPv4 */
    reply[18] = 6;    reply[19] = 4;    /* HLEN PLEN */
    reply[20] = 0x00; reply[21] = 0x02; /* OPER reply */
    memcpy(reply + 22, k_gw_mac, 6);    /* SHA */
    memcpy(reply + 28, k_host_ip, 4);   /* SPA */
    memcpy(reply + 32, eth + 6, 6);     /* THA = requester MAC */
    memcpy(reply + 38, arp + 14, 4);    /* TPA = requester IP (SPA of req) */

    queue_pending_eth(reply, 42);
    return 1;
}

static int darwin_configure_utun(const char *ifname) {
    char cmd[256];
    /* Point-to-point: local 192.168.4.1, peer 192.168.4.2 */
    snprintf(cmd, sizeof(cmd),
             "ifconfig %s inet %s %s netmask %s up 2>/dev/null",
             ifname, TAP_HOST_IP, TAP_PEER_IP, TAP_NETMASK);
    if (run_cmd(cmd) != 0) {
        fprintf(stderr, "[TAP] ifconfig failed (need root?). Manual:\n");
        fprintf(stderr, "[TAP]   sudo ifconfig %s inet %s %s netmask %s up\n",
                ifname, TAP_HOST_IP, TAP_PEER_IP, TAP_NETMASK);
        return -1;
    }
    fprintf(stderr, "[TAP] Configured %s as %s (peer %s)\n",
            ifname, TAP_HOST_IP, TAP_PEER_IP);
    return 0;
}

static void darwin_try_pf_nat(void) {
    /* Optional: TCP/ICMP via kernel. UDP uses userspace NAT regardless. */
    const char *script = getenv("BRAMBLE_PF_NAT");
    char local[512];
    if (!script || !script[0]) {
        static const char *candidates[] = {
            "scripts/macos-cyw43-pf-nat.sh",
            "../Bramble/scripts/macos-cyw43-pf-nat.sh",
            NULL
        };
        for (int i = 0; candidates[i]; i++) {
            if (access(candidates[i], X_OK) == 0 || access(candidates[i], R_OK) == 0) {
                script = candidates[i];
                break;
            }
        }
    }
    if (script && script[0]) {
        snprintf(local, sizeof(local), "sh '%s' enable 2>/dev/null", script);
        if (run_cmd(local) == 0) {
            fprintf(stderr, "[TAP] pf NAT enabled via %s (TCP/ICMP; UDP is userspace)\n",
                    script);
            return;
        }
    }
    fprintf(stderr, "[TAP] pf NAT optional (UDP DNS/NTP/TFTP use userspace NAT)\n");
}

int tapif_open(const char *name) {
    (void)name; /* utun unit is assigned by the kernel */

    for (int i = 0; i < DARWIN_UDP_FLOWS; i++)
        darwin_udp_flows[i].fd = -1;
    darwin_rxq_head = darwin_rxq_tail = darwin_rxq_count = 0;

    int fd = socket(PF_SYSTEM, SOCK_DGRAM, SYSPROTO_CONTROL);
    if (fd < 0) {
        fprintf(stderr, "[TAP] utun socket failed: %s\n", strerror(errno));
        return -1;
    }

    struct ctl_info info;
    memset(&info, 0, sizeof(info));
    strncpy(info.ctl_name, UTUN_CONTROL_NAME, sizeof(info.ctl_name) - 1);
    if (ioctl(fd, CTLIOCGINFO, &info) < 0) {
        fprintf(stderr, "[TAP] CTLIOCGINFO failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    struct sockaddr_ctl addr;
    memset(&addr, 0, sizeof(addr));
    addr.sc_len = sizeof(addr);
    addr.sc_family = AF_SYSTEM;
    addr.ss_sysaddr = AF_SYS_CONTROL;
    addr.sc_id = info.ctl_id;
    addr.sc_unit = 0; /* kernel picks next utunN */

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[TAP] utun connect failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    char ifname[IFNAMSIZ];
    socklen_t ifname_len = sizeof(ifname);
    if (getsockopt(fd, SYSPROTO_CONTROL, UTUN_OPT_IFNAME, ifname, &ifname_len) < 0) {
        fprintf(stderr, "[TAP] UTUN_OPT_IFNAME failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1)
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    memset(utun_ifname, 0, sizeof(utun_ifname));
    strncpy(utun_ifname, ifname, sizeof(utun_ifname) - 1);
    darwin_active = 1;
    darwin_utun_rx_ok = 1;

    fprintf(stderr,
            "[TAP] macOS utun '%s' opened (fd=%d) — UDP userspace NAT + L3 bridge\n",
            utun_ifname, fd);

    darwin_configure_utun(utun_ifname);
    darwin_try_pf_nat();
    return fd;
}

void tapif_close(int fd) {
    if (fd >= 0) {
        if (darwin_active && utun_ifname[0]) {
            fprintf(stderr, "[TAP] Closed utun '%s'\n", utun_ifname);
        }
        darwin_udp_nat_close_all();
        darwin_rxq_count = 0;
        darwin_active = 0;
        utun_ifname[0] = '\0';
        close(fd);
    }
}

void tapif_service(int fd) {
    (void)fd;
    if (fd < 0) return;
    darwin_udp_nat_poll();
}

int tapif_read(int fd, uint8_t *buf, int maxlen) {
    if (fd < 0) return -1;

    darwin_udp_nat_poll();

    int q = dequeue_pending_eth(buf, maxlen);
    if (q > 0) return q;

    /* utun RX is optional (TCP/ICMP via pf). UDP NAT does not need it.
     * A hard utun read error must not kill the bridge — that dropped ARP/DNS. */
    if (!darwin_utun_rx_ok)
        return 0;

    uint8_t raw[4 + 1500];
    ssize_t n = read(fd, raw, sizeof(raw));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            return 0;
        static int utun_err_logged;
        if (utun_err_logged < 3) {
            utun_err_logged++;
            fprintf(stderr,
                    "[TAP] utun read: %s — continuing with UDP NAT only\n",
                    strerror(errno));
            fflush(stderr);
        }
        if (errno == EBADF)
            return -1;
        darwin_utun_rx_ok = 0;
        return 0;
    }
    if (n <= 4) return 0;

    uint32_t af;
    memcpy(&af, raw, 4);
    af = ntohl(af);
    if (af != AF_INET) return 0; /* ignore IPv6 for now */

    int ip_len = (int)n - 4;
    if (14 + ip_len > maxlen) return 0;
    if (ip_len < 20) return 0;

    /* Build Ethernet frame for guest */
    memcpy(buf, k_guest_mac, 6);
    memcpy(buf + 6, k_gw_mac, 6);
    buf[12] = 0x08;
    buf[13] = 0x00;
    memcpy(buf + 14, raw + 4, (size_t)ip_len);
    return 14 + ip_len;
}

int tapif_write(int fd, const uint8_t *buf, int len) {
    if (fd < 0) return -1;
    if (len < 14) return -1;

    if (darwin_handle_arp(buf, len)) {
        return len; /* consumed; reply queued for tapif_read */
    }

    if (buf[12] != 0x08 || buf[13] != 0x00) {
        return len; /* drop non-IPv4 quietly */
    }

    /* UDP (DNS/NTP/TFTP): userspace NAT — does not need pf */
    if (darwin_udp_nat_tx(buf, len))
        return len;

    /* Other IPv4 (TCP/ICMP): best-effort via utun + optional pf */
    int ip_len = len - 14;
    if (ip_len < 20) return -1;

    uint8_t raw[4 + 1500];
    if (ip_len > 1500) ip_len = 1500;
    uint32_t af = htonl(AF_INET);
    memcpy(raw, &af, 4);
    memcpy(raw + 4, buf + 14, (size_t)ip_len);

    int total = 0;
    int want = 4 + ip_len;
    while (total < want) {
        ssize_t n = write(fd, raw + total, (size_t)(want - total));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return total > 4 ? len : 0;
            }
            return -1;
        }
        total += (int)n;
    }
    return len;
}

#else /* !linux && !apple */

int tapif_open(const char *name) {
    (void)name;
    fprintf(stderr, "[TAP] Host network bridge not supported on this platform\n");
    return -1;
}

void tapif_close(int fd) {
    if (fd >= 0) close(fd);
}

int tapif_read(int fd, uint8_t *buf, int maxlen) {
    (void)fd; (void)buf; (void)maxlen;
    return -1;
}

int tapif_write(int fd, const uint8_t *buf, int len) {
    (void)fd; (void)buf; (void)len;
    return -1;
}

void tapif_service(int fd) {
    (void)fd;
}


#endif
