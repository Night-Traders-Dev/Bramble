#ifndef TAPIF_H
#define TAPIF_H

#include <stdint.h>

/*
 * Host virtual interface for CYW43 WLAN Ethernet frames.
 *
 * Linux: TAP (L2).  macOS: utun (L3) with Ethernet↔IPv4 adaptation.
 * Subnet: guest 192.168.4.2 / host 192.168.4.1 (matches CYW43 DHCP).
 *
 * Usage: ./bramble firmware.uf2 -wifi -tap <ifname>
 * macOS: UDP (DNS/NTP/TFTP) uses userspace NAT; pf optional for TCP.
 */

/* Open host interface. Returns fd or -1 on error. */
int tapif_open(const char *name);

/* Close host interface */
void tapif_close(int fd);

/* Read an Ethernet frame (non-blocking). Returns bytes, 0 if none, -1 on error. */
int tapif_read(int fd, uint8_t *buf, int maxlen);

/* Write an Ethernet frame. Returns bytes written or -1 on error. */
int tapif_write(int fd, const uint8_t *buf, int len);

/* Service UDP NAT / TFTP host-ACK without requiring a TAP Ethernet read.
 * Safe to call often; used when guest radio is asleep but TFTP still runs. */
void tapif_service(int fd);

#endif /* TAPIF_H */
