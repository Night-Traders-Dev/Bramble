#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include "cyw43.h"
#include "tapif.h"
#include "emulator.h"
#include "gpio.h"

/* CYW43439 chip ID */
#define CYW43439_CHIP_ID 0x00A9A6A7

/* WiFi GPIO pin assignments (Pico W) */
#define WL_CS         23
#define WL_CLK        24
#define WL_DIO        25
/* WL_HOST_WAKE = GPIO 24 (shared with CLK/DIO in different phases).
 * Active HIGH: CYW43 asserts this to tell RP2040 that data is available.
 * cyw43_ll.c checks gpio_get(24) != 0 before polling the interrupt register. */
#define WL_HOST_WAKE  24

/* Core wrapper full addresses (BASE + WRAPPER_REGISTER_OFFSET + register_offset) */
/* WLAN ARM CM3: 0x18003000 + 0x100000 = 0x18103000 */
#define CYW43_WLAN_RESETCTRL  0x18103800u  /* WLAN ARM + WRAPPER + AI_RESETCTRL (0x800) */
#define CYW43_WLAN_IOCTRL     0x18103408u  /* WLAN ARM + WRAPPER + AI_IOCTRL (0x408) */
/* SOCRAM: 0x18004000 + 0x100000 = 0x18104000 */
#define CYW43_SOCRAM_RESETCTRL 0x18104800u
#define CYW43_SOCRAM_IOCTRL    0x18104408u
#define CYW43_AIRC_RESET       0x01u

/* SPI_STATUS_REGISTER (0x0008) bits */
#define SPI_STATUS_F2_RX_READY      0x00000020u
#define SPI_STATUS_F2_PKT_AVAILABLE 0x00000100u
#define SPI_STATUS_F2_PKT_LEN_SHIFT 9

cyw43_state_t cyw43;

/* Default MAC address (locally administered) */
static const uint8_t default_mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};

/* ========================================================================
 * RX Queue Management
 * ======================================================================== */

static int rx_queue_count(void) {
    return (cyw43.rx_head - cyw43.rx_tail + CYW43_RX_QUEUE_SIZE) % CYW43_RX_QUEUE_SIZE;
}

/* Drive WL_HOST_WAKE (GPIO 24) HIGH when data is queued, LOW when empty.
 * cyw43_ll.c's sdpcm_poll_device checks gpio_get(WL_HOST_WAKE) == 1 before
 * reading the SPI interrupt register, so we must assert this whenever there
 * is a frame waiting in rx_queue. */
static void cyw43_update_irq(void) {
    /* GPIO 24 is WL_DIO (SPI data, output during TX) shared with WL_HOST_WAKE
     * (input when idle). The PIO program does "set pindirs, 0" after RX to
     * switch it back to input, but since we intercept the FIFO without running
     * instructions, we must do this ourselves so gpio_get(24) returns our IRQ
     * state rather than the stale PIO output direction. */
    gpio_set_direction(WL_HOST_WAKE, 0);  /* 0 = input */
    int val = rx_queue_count() > 0 ? 1 : 0;
    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] update_irq: GPIO24=%d (q=%d)\n", val, rx_queue_count());
    gpio_set_input_pin(WL_HOST_WAKE, val);
}

static int rx_queue_push(const uint8_t *data, int len) {
    if (rx_queue_count() >= CYW43_RX_QUEUE_SIZE - 1) return -1;
    if (len > CYW43_MAX_FRAME_SIZE) return -1;

    cyw43_rx_frame_t *f = &cyw43.rx_queue[cyw43.rx_head];
    memcpy(f->data, data, len);
    f->len = len;
    cyw43.rx_head = (cyw43.rx_head + 1) % CYW43_RX_QUEUE_SIZE;
    cyw43_update_irq();
    return 0;
}

static cyw43_rx_frame_t *rx_queue_peek(void) {
    if (rx_queue_count() == 0) return NULL;
    return &cyw43.rx_queue[cyw43.rx_tail];
}

static void rx_queue_pop(void) {
    if (rx_queue_count() > 0) {
        cyw43.rx_tail = (cyw43.rx_tail + 1) % CYW43_RX_QUEUE_SIZE;
        cyw43_update_irq();
    }
}

/* ========================================================================
 * SDPCM Frame Construction Helpers
 * ======================================================================== */

static void sdpcm_fill_header(uint8_t *buf, int total_size, uint8_t channel, uint8_t hdr_len) {
    sdpcm_header_t *h = (sdpcm_header_t *)buf;
    memset(h, 0, sizeof(*h));
    h->size = (uint16_t)total_size;
    h->size_com = ~h->size;
    h->sequence = cyw43.tx_seq++;
    h->channel_and_flags = channel;
    h->header_length = hdr_len;
    h->bus_data_credit = (uint8_t)(cyw43.last_fw_seq + 4);
}

/* Build an IOCTL response and queue it */
static void cyw43_queue_ioctl_response(uint32_t cmd, uint16_t ioctl_id,
                                        const uint8_t *payload, int payload_len,
                                        uint32_t status) {
    uint8_t frame[CYW43_MAX_FRAME_SIZE];
    int offset = 0;

    /* SDPCM header (12 bytes) - control channel, no pad */
    int total = 12 + 16 + payload_len;
    sdpcm_fill_header(frame, total, SDPCM_CONTROL_CHANNEL, 12);
    offset = 12;

    /* CDC header (16 bytes) */
    cdc_header_t *cdc = (cdc_header_t *)(frame + offset);
    cdc->cmd = cmd;
    cdc->len = (uint32_t)payload_len;
    cdc->flags = ((uint32_t)ioctl_id << CDC_ID_SHIFT) | CDC_REPLY;
    cdc->status = status;
    offset += 16;

    /* Payload */
    if (payload && payload_len > 0) {
        memcpy(frame + offset, payload, payload_len);
        offset += payload_len;
    }

    rx_queue_push(frame, offset);
}

/* Build an async event and queue it.
 *
 * Frame layout (offsets relative to BDC payload start = "payload"):
 *   payload[0..13]  : Ethernet header (14 bytes)
 *   payload[14..23] : bcmilcp header: subtype(2)+length(2)+pad(1)+OUI(3)+usr_subtype(2)
 *   payload[24..]   : event body = wl_event_msg_t / cyw43_async_event_t layout:
 *                     _0/version(2), flags(2), event_type(4), status(4), reason(4),
 *                     auth_type(4), datalen(4), addr(6), ifname(16), ifidx(1), bsscfgidx(1)
 *
 * SDK passes buf=payload+24 to cyw43_ll_parse_async_event, which does a memmove
 * (alignment fix) copying buf -> buf-2 (payload+22), then casts to cyw43_async_event_t:
 *   ev._0        = payload[22..23] = was at payload[24..25] = version (0x0002)
 *   ev.flags     = payload[24..25] = was at payload[26..27] (BE uint16)
 *   ev.event_type = payload[26..29] = was at payload[28..31] (BE uint32)
 *   ev.status    = payload[30..33] = was at payload[32..35] (BE uint32)
 */
static void cyw43_queue_event(uint32_t event_type, uint32_t status,
                               uint32_t reason, uint16_t flags) {
    uint8_t frame[256];
    int offset = 0;

    /* Reserve space for SDPCM (12) + pad (2) + BDC (4) = 18 bytes */
    offset = 18;

    /* Ethernet header (14 bytes) — payload[0..13] */
    memset(frame + offset, 0xFF, 6);                    /* dst: broadcast */
    memcpy(frame + offset + 6, cyw43.mac_addr, 6);      /* src: device MAC */
    frame[offset + 12] = 0x88;                           /* ethertype: 0x886C */
    frame[offset + 13] = 0x6C;
    offset += 14;

    /* bcmilcp header — payload[14..23] */
    frame[offset++] = 0x00; frame[offset++] = 0x01;     /* subtype = 1 (BE) */
    int len_offset = offset;
    frame[offset++] = 0x00; frame[offset++] = 0x00;     /* length placeholder */
    frame[offset++] = 0x00;                              /* pad byte — payload[18] */
    frame[offset++] = 0x00; frame[offset++] = 0x10; frame[offset++] = 0x18; /* OUI — payload[19-21] */
    frame[offset++] = 0x80; frame[offset++] = 0x02;     /* usr_subtype — payload[22-23] = ev._0 */

    /* Event body — payload[24..] */

    /* ev._0 = version = 0x0002 (BE uint16; SDK ignores this field) */
    frame[offset++] = 0x00;
    frame[offset++] = 0x02;

    /* ev.flags (BE uint16): bit 0 = link-up for EV_LINK — payload[26..27] */
    frame[offset++] = (flags >> 8) & 0xFF;
    frame[offset++] = flags & 0xFF;

    /* ev.event_type (BE u32) — payload[28..31] */
    frame[offset++] = (event_type >> 24) & 0xFF;
    frame[offset++] = (event_type >> 16) & 0xFF;
    frame[offset++] = (event_type >>  8) & 0xFF;
    frame[offset++] = (event_type >>  0) & 0xFF;

    /* ev.status (BE u32) — payload[32..35] */
    frame[offset++] = (status >> 24) & 0xFF;
    frame[offset++] = (status >> 16) & 0xFF;
    frame[offset++] = (status >>  8) & 0xFF;
    frame[offset++] = (status >>  0) & 0xFF;

    /* ev.reason (BE u32) — payload[36..39] */
    frame[offset++] = (reason >> 24) & 0xFF;
    frame[offset++] = (reason >> 16) & 0xFF;
    frame[offset++] = (reason >>  8) & 0xFF;
    frame[offset++] = (reason >>  0) & 0xFF;

    /* ev._1[0..3]: auth_type (BE u32) = 0 — payload[40..43] */
    frame[offset++] = 0; frame[offset++] = 0;
    frame[offset++] = 0; frame[offset++] = 0;

    /* ev._1[4..7]: datalen (BE u32) = 0 */
    frame[offset++] = 0; frame[offset++] = 0;
    frame[offset++] = 0; frame[offset++] = 0;

    /* ev._1[8..13]: addr (6 bytes) = device MAC */
    memcpy(frame + offset, cyw43.mac_addr, 6);
    offset += 6;

    /* ev._1[14..29]: ifname (16 bytes) = "wl0" */
    memset(frame + offset, 0, 16);
    frame[offset] = 'w'; frame[offset+1] = 'l'; frame[offset+2] = '0';
    offset += 16;

    /* ifidx + bsscfgidx (= ev.interface) */
    frame[offset++] = 0;  /* ifidx */
    frame[offset++] = 0;  /* bsscfgidx */

    /* Fill bcmilcp length field: total bytes following the length field */
    uint16_t ev_len = (uint16_t)(offset - len_offset - 2);
    frame[len_offset]     = (ev_len >> 8) & 0xFF;
    frame[len_offset + 1] = ev_len & 0xFF;

    /* Fill SDPCM + pad + BDC at the beginning */
    sdpcm_fill_header(frame, offset, SDPCM_EVENT_CHANNEL, 14);
    frame[12] = 0; frame[13] = 0;  /* 2-byte pad */

    bdc_header_t *bdc = (bdc_header_t *)(frame + 14);
    bdc->flags = 0x20;
    bdc->priority = 0;
    bdc->flags2 = 0;
    bdc->data_offset = 0;

    rx_queue_push(frame, offset);
}

/* Queue connection events (simulates successful WiFi join) */
static void cyw43_queue_connect_events(void) {
    cyw43_queue_event(CYW43_EV_AUTH, CYW43_STATUS_SUCCESS, 0, 0);
    cyw43_queue_event(CYW43_EV_LINK, CYW43_STATUS_SUCCESS, 0, 1);  /* flags=1: link-up */
    cyw43_queue_event(CYW43_EV_SET_SSID, CYW43_STATUS_SUCCESS, 0, 0);
    cyw43_queue_event(CYW43_EV_PSK_SUP, CYW43_SUP_KEYED, 0, 0);
    cyw43.wifi_state = CYW43_WIFI_CONNECTED;

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] Queued connection events for SSID '%s'\n",
                cyw43.connected_ssid);
}

/* Wrap an Ethernet frame from TAP in SDPCM + BDC and queue for firmware */
static void cyw43_queue_rx_data(const uint8_t *eth_frame, int eth_len) {
    uint8_t frame[CYW43_MAX_FRAME_SIZE];
    int total = 18 + eth_len;  /* SDPCM(12) + pad(2) + BDC(4) + ethernet */

    if (total > CYW43_MAX_FRAME_SIZE) return;

    sdpcm_fill_header(frame, total, SDPCM_DATA_CHANNEL, 14);

    /* 2-byte pad */
    frame[12] = 0; frame[13] = 0;

    /* BDC header */
    bdc_header_t *bdc = (bdc_header_t *)(frame + 14);
    bdc->flags = 0x20;
    bdc->priority = 0;
    bdc->flags2 = 0;
    bdc->data_offset = 0;

    /* Ethernet frame */
    memcpy(frame + 18, eth_frame, eth_len);

    rx_queue_push(frame, total);
}

/* ========================================================================
 * IOCTL Handler
 * ======================================================================== */

static void cyw43_handle_ioctl(const uint8_t *buf, int len) {
    if (len < 12 + 16) return;  /* SDPCM + CDC minimum */

    sdpcm_header_t *sdpcm = (sdpcm_header_t *)buf;
    cdc_header_t *cdc = (cdc_header_t *)(buf + 12);

    cyw43.last_fw_seq = sdpcm->sequence;

    uint32_t cmd = cdc->cmd;
    uint16_t ioctl_id = (uint16_t)(cdc->flags >> CDC_ID_SHIFT);
    int is_set = (cdc->flags & 0x02) != 0;
    const uint8_t *payload = buf + 28;  /* After SDPCM(12) + CDC(16) */
    int payload_len = len - 28;

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] IOCTL cmd=%d %s id=%d payload_len=%d\n",
                cmd, is_set ? "SET" : "GET", ioctl_id, payload_len);

    switch (cmd) {
    case WLC_GET_VAR: {
        /* payload is null-terminated iovar name */
        const char *varname = (const char *)payload;

        if (strcmp(varname, "cur_etheraddr") == 0) {
            cyw43_queue_ioctl_response(cmd, ioctl_id, cyw43.mac_addr, 6, 0);
            return;
        }
        if (strcmp(varname, "ver") == 0) {
            const char *ver = "wl0: Bramble CYW43 Emulator\n";
            cyw43_queue_ioctl_response(cmd, ioctl_id,
                                        (const uint8_t *)ver, (int)strlen(ver) + 1, 0);
            return;
        }
        /* Default: return zeros */
        uint8_t zeros[256];
        memset(zeros, 0, sizeof(zeros));
        int resp_len = payload_len > 0 ? payload_len : 4;
        if (resp_len > (int)sizeof(zeros)) resp_len = (int)sizeof(zeros);
        cyw43_queue_ioctl_response(cmd, ioctl_id, zeros, resp_len, 0);
        return;
    }

    case WLC_SET_SSID: {
        /* Payload: 4 bytes SSID length + SSID string (32 bytes) */
        if (payload_len >= 36) {
            uint32_t ssid_len = payload[0] | (payload[1] << 8) |
                                (payload[2] << 16) | (payload[3] << 24);
            if (ssid_len > CYW43_MAX_SSID_LEN) ssid_len = CYW43_MAX_SSID_LEN;
            memcpy(cyw43.connected_ssid, payload + 4, ssid_len);
            cyw43.connected_ssid[ssid_len] = '\0';

            if (cpu.debug_enabled)
                fprintf(stderr, "[CYW43] JOIN SSID: '%s'\n", cyw43.connected_ssid);
        }

        /* Respond success */
        cyw43_queue_ioctl_response(cmd, ioctl_id, NULL, 0, 0);

        /* Queue connection events */
        cyw43_queue_connect_events();
        return;
    }

    case WLC_GET_SSID: {
        /* Return current SSID: 4 bytes length + 32 bytes SSID */
        uint8_t resp[36];
        memset(resp, 0, sizeof(resp));
        uint32_t slen = (uint32_t)strlen(cyw43.connected_ssid);
        resp[0] = slen & 0xFF;
        resp[1] = (slen >> 8) & 0xFF;
        resp[2] = (slen >> 16) & 0xFF;
        resp[3] = (slen >> 24) & 0xFF;
        memcpy(resp + 4, cyw43.connected_ssid, slen);
        cyw43_queue_ioctl_response(cmd, ioctl_id, resp, 36, 0);
        return;
    }

    case WLC_GET_BSSID: {
        /* Return a fake BSSID */
        uint8_t bssid[6] = {0x02, 0xCA, 0xFE, 0xBA, 0xBE, 0x01};
        cyw43_queue_ioctl_response(cmd, ioctl_id, bssid, 6, 0);
        return;
    }

    default:
        /* All other IOCTLs: return success with echo payload for GET, empty for SET */
        if (is_set) {
            cyw43_queue_ioctl_response(cmd, ioctl_id, NULL, 0, 0);
        } else {
            /* GET: return payload echoed (or zeros) */
            uint8_t zeros[256];
            memset(zeros, 0, sizeof(zeros));
            int resp_len = payload_len > 0 ? payload_len : 4;
            if (resp_len > (int)sizeof(zeros)) resp_len = (int)sizeof(zeros);
            cyw43_queue_ioctl_response(cmd, ioctl_id, zeros, resp_len, 0);
        }
        return;
    }
}

/* ========================================================================
 * Fake DHCP Server (assigns 192.168.4.2 to firmware, no real network needed)
 * ======================================================================== */

/* Fixed IP addressing for the virtual WiFi network */
static const uint8_t dhcp_client_ip[4]  = {192, 168,   4, 2};
static const uint8_t dhcp_server_ip[4]  = {192, 168,   4, 1};
static const uint8_t dhcp_subnet[4]     = {255, 255, 255, 0};
static const uint8_t dhcp_lease_time[4] = {0, 0, 14, 16};  /* 3600 s */

/* Build and queue a DHCP OFFER (msg_type=2) or ACK (msg_type=5) reply. */
static void cyw43_send_dhcp_reply(const uint8_t *eth_req, int eth_len, uint8_t reply_type) {
    if (eth_len < 14 + 20 + 8 + 240) return;

    const uint8_t *ip_req  = eth_req + 14;
    int ip_hlen = (ip_req[0] & 0x0F) * 4;
    const uint8_t *dhcp    = ip_req + ip_hlen + 8;  /* skip IP+UDP */
    int dhcp_len           = eth_len - 14 - ip_hlen - 8;
    if (dhcp_len < 236) return;

    uint32_t xid = ((uint32_t)dhcp[4] << 24) | ((uint32_t)dhcp[5] << 16) |
                   ((uint32_t)dhcp[6] <<  8) |  (uint32_t)dhcp[7];
    const uint8_t *chaddr  = dhcp + 28;  /* client MAC (first 6 bytes) */

    uint8_t frame[512];
    memset(frame, 0, sizeof(frame));
    int off = 0;

    /* Ethernet header: broadcast dst, server MAC src */
    memset(frame + off, 0xFF, 6);               off += 6;
    memcpy(frame + off, cyw43.mac_addr, 6);     off += 6;
    frame[off++] = 0x08; frame[off++] = 0x00;   /* IPv4 */

    /* IPv4 header */
    int ip_off = off;
    frame[off++] = 0x45;                        /* v4, IHL=5 */
    frame[off++] = 0x00;                        /* DSCP/ECN */
    int ip_len_off = off; off += 2;             /* total length (filled later) */
    frame[off++] = 0x00; frame[off++] = 0x01;  /* ID */
    frame[off++] = 0x00; frame[off++] = 0x00;  /* flags/frag */
    frame[off++] = 0x80;                        /* TTL=128 */
    frame[off++] = 0x11;                        /* proto=UDP */
    int ip_csum_off = off; off += 2;            /* checksum (filled later) */
    memcpy(frame + off, dhcp_server_ip, 4);     off += 4;  /* src = server */
    frame[off++] = 255; frame[off++] = 255;
    frame[off++] = 255; frame[off++] = 255;    /* dst = broadcast */

    /* UDP header */
    int udp_off = off;
    frame[off++] = 0x00; frame[off++] = 67;    /* src port = 67 */
    frame[off++] = 0x00; frame[off++] = 68;    /* dst port = 68 */
    int udp_len_off = off; off += 2;            /* UDP length (filled later) */
    frame[off++] = 0x00; frame[off++] = 0x00;  /* checksum = 0 */

    /* DHCP message */
    int dhcp_off = off;
    frame[off++] = 0x02;                        /* op = BOOTREPLY */
    frame[off++] = 0x01; frame[off++] = 0x06;  /* htype=1, hlen=6 */
    frame[off++] = 0x00;                        /* hops */
    frame[off++] = (xid >> 24) & 0xFF;
    frame[off++] = (xid >> 16) & 0xFF;
    frame[off++] = (xid >>  8) & 0xFF;
    frame[off++] = (xid >>  0) & 0xFF;
    frame[off++] = 0x00; frame[off++] = 0x00;  /* secs */
    frame[off++] = 0x80; frame[off++] = 0x00;  /* flags: broadcast */
    frame[off++] = 0x00; frame[off++] = 0x00;
    frame[off++] = 0x00; frame[off++] = 0x00;  /* ciaddr = 0 */
    memcpy(frame + off, dhcp_client_ip, 4);     off += 4;  /* yiaddr */
    memcpy(frame + off, dhcp_server_ip, 4);     off += 4;  /* siaddr */
    frame[off++] = 0x00; frame[off++] = 0x00;
    frame[off++] = 0x00; frame[off++] = 0x00;  /* giaddr = 0 */
    memcpy(frame + off, chaddr, 16);            off += 16; /* chaddr */
    memset(frame + off, 0, 64);                 off += 64; /* sname */
    memset(frame + off, 0, 128);                off += 128;/* file */
    /* Magic cookie */
    frame[off++] = 0x63; frame[off++] = 0x82;
    frame[off++] = 0x53; frame[off++] = 0x63;
    /* Options */
    frame[off++] = 53; frame[off++] = 1; frame[off++] = reply_type;  /* msg type */
    frame[off++] = 54; frame[off++] = 4;
    memcpy(frame + off, dhcp_server_ip, 4);     off += 4;  /* server ID */
    frame[off++] = 51; frame[off++] = 4;
    memcpy(frame + off, dhcp_lease_time, 4);    off += 4;  /* lease time */
    frame[off++] =  1; frame[off++] = 4;
    memcpy(frame + off, dhcp_subnet, 4);        off += 4;  /* subnet mask */
    frame[off++] =  3; frame[off++] = 4;
    memcpy(frame + off, dhcp_server_ip, 4);     off += 4;  /* router */
    frame[off++] =  6; frame[off++] = 4;
    memcpy(frame + off, dhcp_server_ip, 4);     off += 4;  /* DNS */
    frame[off++] = 255;                                    /* end */

    /* Fill in lengths */
    int ip_total  = off - ip_off;
    int udp_total = off - udp_off;
    frame[ip_len_off]     = (ip_total  >> 8) & 0xFF;
    frame[ip_len_off + 1] =  ip_total        & 0xFF;
    frame[udp_len_off]    = (udp_total >> 8) & 0xFF;
    frame[udp_len_off + 1] = udp_total       & 0xFF;

    /* IPv4 header checksum */
    uint32_t csum = 0;
    for (int i = 0; i < 20; i += 2)
        csum += ((uint32_t)frame[ip_off + i] << 8) | frame[ip_off + i + 1];
    while (csum >> 16) csum = (csum & 0xFFFF) + (csum >> 16);
    csum = ~csum & 0xFFFF;
    frame[ip_csum_off]     = (csum >> 8) & 0xFF;
    frame[ip_csum_off + 1] =  csum       & 0xFF;

    (void)udp_off; (void)dhcp_off;

    cyw43_queue_rx_data(frame, off);

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] DHCP %s → offer %d.%d.%d.%d\n",
                reply_type == 2 ? "DISCOVER" : "REQUEST",
                dhcp_client_ip[0], dhcp_client_ip[1],
                dhcp_client_ip[2], dhcp_client_ip[3]);
}

/* Returns 1 if this was a DHCP packet that we handled, 0 otherwise. */
static int cyw43_handle_dhcp(const uint8_t *eth_frame, int eth_len) {
    if (eth_len < 14 + 20 + 8 + 240) return 0;
    /* IPv4 only */
    if (eth_frame[12] != 0x08 || eth_frame[13] != 0x00) return 0;

    const uint8_t *ip = eth_frame + 14;
    if ((ip[0] >> 4) != 4) return 0;
    int ip_hlen = (ip[0] & 0x0F) * 4;
    if (ip[9] != 17) return 0;          /* not UDP */

    const uint8_t *udp = ip + ip_hlen;
    uint16_t src_port  = ((uint16_t)udp[0] << 8) | udp[1];
    uint16_t dst_port  = ((uint16_t)udp[2] << 8) | udp[3];
    if (src_port != 68 || dst_port != 67) return 0;

    const uint8_t *dhcp = udp + 8;
    int dhcp_len = eth_len - 14 - ip_hlen - 8;
    if (dhcp_len < 240) return 0;
    /* Verify magic cookie */
    if (dhcp[236] != 0x63 || dhcp[237] != 0x82 ||
        dhcp[238] != 0x53 || dhcp[239] != 0x63) return 0;

    /* Scan options for message type (opt 53) */
    uint8_t msg_type = 0;
    for (int i = 240; i < dhcp_len; ) {
        uint8_t opt = dhcp[i++];
        if (opt == 255) break;
        if (opt == 0) continue;
        if (i >= dhcp_len) break;
        uint8_t olen = dhcp[i++];
        if (opt == 53 && olen == 1 && i < dhcp_len)
            msg_type = dhcp[i];
        i += olen;
    }

    if (msg_type == 1) {        /* DISCOVER → OFFER */
        cyw43_send_dhcp_reply(eth_frame, eth_len, 2);
        return 1;
    } else if (msg_type == 3) { /* REQUEST → ACK */
        cyw43_send_dhcp_reply(eth_frame, eth_len, 5);
        return 1;
    }
    return 0;
}

/* ========================================================================
 * WLAN TX Processing (firmware -> CYW43)
 * ======================================================================== */

static void cyw43_wlan_tx_word(uint32_t val) {
    if (cyw43.wlan_tx_offset + 4 <= CYW43_WLAN_TX_BUF_SIZE) {
        cyw43.wlan_tx_buf[cyw43.wlan_tx_offset++] = (val >>  0) & 0xFF;
        cyw43.wlan_tx_buf[cyw43.wlan_tx_offset++] = (val >>  8) & 0xFF;
        cyw43.wlan_tx_buf[cyw43.wlan_tx_offset++] = (val >> 16) & 0xFF;
        cyw43.wlan_tx_buf[cyw43.wlan_tx_offset++] = (val >> 24) & 0xFF;
    }
}

static void cyw43_wlan_tx_complete(void) {
    int len = cyw43.wlan_tx_offset;
    if (len < 12) {
        cyw43.wlan_tx_offset = 0;
        return;
    }

    sdpcm_header_t *sdpcm = (sdpcm_header_t *)cyw43.wlan_tx_buf;
    cyw43.last_fw_seq = sdpcm->sequence;

    uint8_t channel = sdpcm->channel_and_flags & 0x0F;

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] WLAN TX: size=%d channel=%d seq=%d\n",
                sdpcm->size, channel, sdpcm->sequence);

    switch (channel) {
    case SDPCM_CONTROL_CHANNEL:
        cyw43_handle_ioctl(cyw43.wlan_tx_buf, len);
        break;

    case SDPCM_DATA_CHANNEL: {
        /* Extract Ethernet frame: skip SDPCM(12) + pad(2) + BDC(4+extra) bytes */
        if (len > 18) {
            int eth_offset = 18;
            bdc_header_t *bdc = (bdc_header_t *)(cyw43.wlan_tx_buf + 14);
            eth_offset += bdc->data_offset * 4;

            int eth_len = len - eth_offset;
            if (eth_len > 0) {
                const uint8_t *eth = cyw43.wlan_tx_buf + eth_offset;
                /* Always try fake DHCP first (no real network needed) */
                if (!cyw43_handle_dhcp(eth, eth_len)) {
                    /* Not DHCP: forward to TAP interface if available */
                    if (cyw43.tap_fd >= 0) {
                        tapif_write(cyw43.tap_fd, eth, eth_len);
                        if (cpu.debug_enabled)
                            fprintf(stderr, "[CYW43] TAP TX: %d bytes\n", eth_len);
                    }
                }
            }
        }
        break;
    }

    default:
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] WLAN TX unknown channel %d\n", channel);
        break;
    }

    cyw43.wlan_tx_offset = 0;
}

/* Forward declarations for PIO gSPI state (defined in PIO section below) */
static int pio_init_swap_remaining;
static int pio_cmd_is_swap;

/* Forward declarations for backplane core wrapper state */
static uint8_t cyw43_wlan_resetctrl;
static uint8_t cyw43_wlan_ioctrl;
static uint8_t cyw43_socram_resetctrl;
static uint8_t cyw43_socram_ioctrl;

/* ========================================================================
 * Init / Reset
 * ======================================================================== */

void cyw43_init(void) {
    cyw43_reset();

    /* Add default fake APs for testing */
    cyw43_add_scan_result("BrambleNet", -45, 6, 3);
    cyw43_add_scan_result("PicoTestAP", -60, 1, 3);
    cyw43_add_scan_result("OpenNetwork", -70, 11, 0);
}

void cyw43_reset(void) {
    int saved_enabled = cyw43.enabled;
    int saved_tap_fd = cyw43.tap_fd;
    char saved_tap_name[32];
    memcpy(saved_tap_name, cyw43.tap_name, sizeof(saved_tap_name));

    memset(&cyw43, 0, sizeof(cyw43_state_t));

    cyw43.enabled = saved_enabled;
    cyw43.tap_fd = saved_tap_fd;
    memcpy(cyw43.tap_name, saved_tap_name, sizeof(cyw43.tap_name));

    memcpy(cyw43.mac_addr, default_mac, 6);
    strcpy(cyw43.country, "XX");
    cyw43.wifi_state = CYW43_WIFI_OFF;
    cyw43.chipclkcsr = CYW43_HT_AVAIL | CYW43_ALP_AVAIL;
    cyw43.pio_num = -1;
    cyw43.pio_sm = -1;
    pio_init_swap_remaining = 2;  /* First 2 commands use SWAP32 encoding */
    pio_cmd_is_swap = 0;

    /* Core wrappers start in reset (RESETCTRL=1 = AIRC_RESET) */
    cyw43_wlan_resetctrl = CYW43_AIRC_RESET;
    cyw43_wlan_ioctrl = 0;
    cyw43_socram_resetctrl = CYW43_AIRC_RESET;
    cyw43_socram_ioctrl = 0;
}

int cyw43_is_wifi_gpio(uint32_t gpio) {
    return cyw43.enabled && (gpio == WL_CS || gpio == WL_CLK || gpio == WL_DIO);
}

/* ========================================================================
 * TAP Bridge
 * ======================================================================== */

int cyw43_tap_open(const char *name) {
    cyw43.tap_fd = tapif_open(name);
    if (cyw43.tap_fd >= 0) {
        strncpy(cyw43.tap_name, name, sizeof(cyw43.tap_name) - 1);
        fprintf(stderr, "[CYW43] TAP bridge enabled on '%s'\n", name);
    }
    return cyw43.tap_fd;
}

void cyw43_tap_close(void) {
    if (cyw43.tap_fd >= 0) {
        tapif_close(cyw43.tap_fd);
        cyw43.tap_fd = -1;
    }
}

void cyw43_tap_poll(void) {
    if (cyw43.tap_fd < 0) return;
    if (cyw43.wifi_state != CYW43_WIFI_CONNECTED) return;

    /* Read Ethernet frames from TAP and queue for firmware */
    uint8_t eth_buf[1518];  /* Max Ethernet frame (tapif_read clamps to this) */
    int n = tapif_read(cyw43.tap_fd, eth_buf, (int)sizeof(eth_buf));
    if (n > 0) {
        cyw43_queue_rx_data(eth_buf, n);
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] TAP RX: %d bytes queued\n", n);
    } else if (n < 0) {
        /* Persistent error — close TAP to avoid spin-polling a dead fd */
        fprintf(stderr, "[CYW43] TAP read error, closing interface\n");
        tapif_close(cyw43.tap_fd);
        cyw43.tap_fd = -1;
    }
}

/* ========================================================================
 * Bus Register Access (Function 0)
 * ======================================================================== */

static uint32_t cyw43_bus_read(uint32_t addr) {
    switch (addr & 0xFF) {
    case 0x00: /* SPI_BUS_CONTROL */
        return cyw43.bus_ctrl;
    case 0x04: /* SPI_INTERRUPT_REGISTER - F2_PACKET_AVAILABLE=0x20 when queued */
    {
        uint32_t val = cyw43.bus_int;
        if (rx_queue_count() > 0)
            val |= CYW43_BUS_INT_F2_PKT_AVAIL;  /* = 0x20 = F2_PACKET_AVAILABLE */
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] INT_REG read: 0x%02X (q=%d)\n", val, rx_queue_count());
        return val;
    }
    case 0x08: /* SPI_STATUS_REGISTER - F2_RX_READY + packet info */
    {
        /* F2 (WLAN) is always ready in our emulation */
        uint32_t val = SPI_STATUS_F2_RX_READY;
        cyw43_rx_frame_t *f = rx_queue_peek();
        if (f && f->len > 0) {
            val |= SPI_STATUS_F2_PKT_AVAILABLE;
            val |= ((uint32_t)(f->len & 0x7FF)) << SPI_STATUS_F2_PKT_LEN_SHIFT;
        }
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] STATUS_REG read: 0x%08X (q=%d)\n", val, rx_queue_count());
        return val;
    }
    case 0x14: /* SPI_READ_TEST_REGISTER = FEEDBEAD */
        return 0xFEEDBEAD;
    case 0x18:
        return cyw43.bus_test_reg;
    default:
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Bus read unknown reg 0x%02X\n", addr & 0xFF);
        return 0;
    }
}

static void cyw43_bus_write(uint32_t addr, uint32_t val) {
    switch (addr & 0xFF) {
    case CYW43_REG_BUS_CTRL:
        cyw43.bus_ctrl = val;
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Bus ctrl = 0x%08X\n", val);
        break;
    case CYW43_REG_BUS_INTERRUPT:
        cyw43.bus_int &= ~val;  /* W1C */
        break;
    case CYW43_REG_BUS_INTMASK:
        cyw43.bus_intmask = val;
        break;
    case CYW43_REG_BUS_TEST:
        cyw43.bus_test_reg = val;
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Test reg = 0x%08X\n", val);
        break;
    default:
        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] Bus write unknown reg 0x%02X = 0x%08X\n",
                    addr & 0xFF, val);
        break;
    }
}

/* ========================================================================
 * Backplane Access (Function 1)
 * ======================================================================== */

static uint32_t cyw43_backplane_read(uint32_t addr) {
    /* CHIPCLKCSR - ALP/HT clock status (SDIO func1 direct register) */
    if (addr == CYW43_BP_CHIPCLKCSR) {
        return cyw43.chipclkcsr;
    }

    /* Window address register reads */
    if (addr == 0x1000A) return (cyw43.bp_window >> 8) & 0xFF;
    if (addr == 0x1000B) return (cyw43.bp_window >> 16) & 0xFF;
    if (addr == 0x1000C) return (cyw43.bp_window >> 24) & 0xFF;

    /* SDIO func1 direct registers (full 17-bit address, no windowing) */
    if (addr == 0x1001F) {
        /* SBSDIO_FUNC1_SLEEPCSR (KSO): always return KSO_SET | DEVICE_ON = 0x03 */
        return 0x03;
    }

    /* Windowed backplane access: reconstruct full address from window + offset */
    uint32_t full_addr = cyw43.bp_window | (addr & 0x7FFF);

    /* Chip ID at CHIPCOMMON offset 0 */
    if (full_addr == 0x18000000u)
        return CYW43439_CHIP_ID;

    /* Core wrapper registers (use full address comparison) */
    if (full_addr == CYW43_WLAN_RESETCTRL)  return cyw43_wlan_resetctrl;
    if (full_addr == CYW43_WLAN_IOCTRL)     return cyw43_wlan_ioctrl;
    if (full_addr == CYW43_SOCRAM_RESETCTRL) return cyw43_socram_resetctrl;
    if (full_addr == CYW43_SOCRAM_IOCTRL)    return cyw43_socram_ioctrl;

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] Backplane read addr=0x%05X (full=0x%08X) -> 0\n",
                addr, full_addr);
    return 0;
}

static void cyw43_backplane_write(uint32_t addr, uint32_t val) {
    /* Window address registers: 3 byte writes build the window */
    if (addr == 0x1000A) { /* SDIO_BACKPLANE_ADDRESS_LOW: bits [15:8] */
        cyw43.bp_window = (cyw43.bp_window & 0xFFFF00FFu) | ((val & 0xFF) << 8);
        return;
    }
    if (addr == 0x1000B) { /* SDIO_BACKPLANE_ADDRESS_MID: bits [23:16] */
        cyw43.bp_window = (cyw43.bp_window & 0xFF00FFFFu) | ((val & 0xFF) << 16);
        return;
    }
    if (addr == 0x1000C) { /* SDIO_BACKPLANE_ADDRESS_HIGH: bits [31:24] */
        cyw43.bp_window = (cyw43.bp_window & 0x00FFFFFFu) | ((val & 0xFF) << 24);
        return;
    }

    /* CHIPCLKCSR write (SDIO func1 direct register) */
    if (addr == CYW43_BP_CHIPCLKCSR) {
        cyw43.chipclkcsr = val | CYW43_HT_AVAIL | CYW43_ALP_AVAIL;
        return;
    }

    /* Windowed backplane access: reconstruct full address */
    uint32_t full_addr = cyw43.bp_window | (addr & 0x7FFF);

    /* Core wrapper registers (use full address comparison) */
    if (full_addr == CYW43_WLAN_RESETCTRL)  { cyw43_wlan_resetctrl = val & 0xFF; return; }
    if (full_addr == CYW43_WLAN_IOCTRL)     { cyw43_wlan_ioctrl = val & 0xFF; return; }
    if (full_addr == CYW43_SOCRAM_RESETCTRL) { cyw43_socram_resetctrl = val & 0xFF; return; }
    if (full_addr == CYW43_SOCRAM_IOCTRL)    { cyw43_socram_ioctrl = val & 0xFF; return; }

    if (cpu.debug_enabled)
        fprintf(stderr, "[CYW43] Backplane write addr=0x%05X (full=0x%08X) = 0x%08X\n",
                addr, full_addr, val);
}

/* ========================================================================
 * GPIO-Level gSPI (legacy, unused by Pico SDK PIO driver)
 * ======================================================================== */

static uint8_t dio_out_value = 0;
static int dio_out_bit = 0;

static void cyw43_clock_bit(int dio_in) {
    cyw43_spi_state_t *s = &cyw43.spi;

    if (!s->in_data_phase) {
        s->cmd_word = (s->cmd_word << 1) | (dio_in & 1);
        s->cmd_bits++;
        if (s->cmd_bits >= 32) {
            /* Would need to process command here */
            s->in_data_phase = 1;
        }
    } else if (s->is_write) {
        /* Write data phase */
    } else {
        /* Read data phase */
        if (dio_out_bit == 0) {
            if (s->resp_offset < s->resp_len)
                dio_out_value = s->resp_buf[s->resp_offset++];
            else
                dio_out_value = 0;
        }
        dio_out_bit++;
        if (dio_out_bit >= 8) dio_out_bit = 0;
    }
}

int cyw43_gpio_intercept(uint32_t gpio, uint32_t value) {
    if (!cyw43.enabled) return 0;

    switch (gpio) {
    case WL_CS:
        if (value == 0) {
            cyw43.spi.cs_active = 1;
            cyw43.spi.cmd_word = 0;
            cyw43.spi.cmd_bits = 0;
            cyw43.spi.in_data_phase = 0;
            cyw43.spi.data_offset = 0;
            dio_out_bit = 0;
        } else {
            cyw43.spi.cs_active = 0;
        }
        return 1;
    case WL_CLK:
        if (cyw43.spi.cs_active && value == 1) {
            int dio_in = 0;
            cyw43_clock_bit(dio_in);
        }
        return 1;
    case WL_DIO:
        return 1;
    default:
        return 0;
    }
}

uint32_t cyw43_gpio_read_dio(void) {
    if (!cyw43.enabled) return 0;
    cyw43_spi_state_t *s = &cyw43.spi;
    if (!s->cs_active || s->is_write || !s->in_data_phase) return 0;
    return (dio_out_value >> (7 - dio_out_bit)) & 1;
}

void cyw43_add_scan_result(const char *ssid, int rssi, int channel, int auth) {
    if (cyw43.scan_count >= CYW43_MAX_SCAN_RESULTS) return;

    cyw43_scan_result_t *r = &cyw43.scan_results[cyw43.scan_count];
    strncpy(r->ssid, ssid, CYW43_MAX_SSID_LEN);
    r->ssid[CYW43_MAX_SSID_LEN] = '\0';
    r->rssi = (int16_t)rssi;
    r->channel = (uint8_t)channel;
    r->auth_mode = (uint8_t)auth;

    uint32_t hash = 0x12345678;
    for (const char *p = ssid; *p; p++)
        hash = hash * 31 + *p;
    r->bssid[0] = 0x02;
    r->bssid[1] = (hash >> 24) & 0xFF;
    r->bssid[2] = (hash >> 16) & 0xFF;
    r->bssid[3] = (hash >>  8) & 0xFF;
    r->bssid[4] = (hash >>  0) & 0xFF;
    r->bssid[5] = cyw43.scan_count;

    cyw43.scan_count++;
}

/* ========================================================================
 * PIO-Level gSPI Protocol Handling
 *
 * The Pico SDK CYW43 driver uses PIO0 SM0 for gSPI communication.
 * TX FIFO writes = gSPI command/data words
 * RX FIFO reads  = gSPI response words
 * ======================================================================== */

static enum {
    PIO_CYW43_IDLE,
    PIO_CYW43_WRITE,
    PIO_CYW43_READ
} pio_cyw43_phase = PIO_CYW43_IDLE;

static uint32_t pio_cmd_function;
static uint32_t pio_cmd_address;
static uint32_t pio_cmd_size;
static uint32_t pio_words_remaining;

static uint32_t pio_resp_buf[512];
static int pio_resp_count;
static int pio_resp_idx;

/* Count of pre-DMA pio_sm_put writes to skip (bit count words, not gSPI data) */
static int pio_pre_dma_skip = 0;

/* Number of remaining commands that use SWAP32 encoding (first 2 at boot) */
static int pio_init_swap_remaining = 0;

/* Whether the current command uses SWAP32 encoding */
static int pio_cmd_is_swap = 0;

/* Reverse bytes within each 16-bit halfword (ARM REV16 / SWAP32) */
static inline uint32_t cyw43_rev16(uint32_t x) {
    return ((x & 0xFF00FF00u) >> 8) | ((x & 0x00FF00FFu) << 8);
}

/* Reverse all 4 bytes of a 32-bit word */
static inline uint32_t cyw43_bswap32(uint32_t x) {
    return ((x >> 24) & 0xFF) | ((x >> 8) & 0xFF00) |
           ((x << 8) & 0xFF0000) | ((x << 24) & 0xFF000000);
}

/* Decode a TXF word to the original make_cmd() value */
static inline uint32_t cyw43_decode_txf(uint32_t val, int is_swap) {
    /* Regular: TXF = bswap32(cmd)  → cmd = bswap32(TXF)
     * Swap:    TXF = bswap32(rev16(cmd)) → cmd = rev16(bswap32(TXF)) */
    uint32_t b = cyw43_bswap32(val);
    return is_swap ? cyw43_rev16(b) : b;
}

/* Encode a response value for pio_resp_buf so firmware sees 'expected' after DMA bswap */
static inline uint32_t cyw43_encode_resp(uint32_t expected, int is_swap) {
    /* Regular: firmware reads bswap32(X) as uint32_t → X = bswap32(expected)
     * Swap:    firmware reads SWAP32(bswap32(X))    → X = bswap32(rev16(expected)) */
    return is_swap ? cyw43_bswap32(cyw43_rev16(expected)) : cyw43_bswap32(expected);
}

/* Called when CYW43 PIO SM is restarted (before each transfer setup) */
void cyw43_pio_sm_restart(void) {
    pio_pre_dma_skip = 2;  /* SDK always does 2 pio_sm_put (X, Y) before DMA */
    pio_cyw43_phase = PIO_CYW43_IDLE;
}

void cyw43_pio_tx_write(uint32_t val) {
    if (!cyw43.enabled) return;

    /* Skip the pre-DMA bit-count words (from pio_sm_put for X/Y registers) */
    if (pio_pre_dma_skip > 0) {
        pio_pre_dma_skip--;
        return;
    }

    switch (pio_cyw43_phase) {
    case PIO_CYW43_IDLE: {
        /* Determine encoding: first 2 commands at boot use SWAP32 (rev16+bswap),
         * all subsequent commands use plain bswap32. */
        pio_cmd_is_swap = (pio_init_swap_remaining > 0);
        if (pio_cmd_is_swap)
            pio_init_swap_remaining--;

        uint32_t cmd = cyw43_decode_txf(val, pio_cmd_is_swap);

        int is_write     = (cmd >> 31) & 1;
        pio_cmd_function = (cmd >> 28) & 0x3;   /* 2-bit function field */
        pio_cmd_address  = (cmd >> 11) & 0x1FFFF;
        pio_cmd_size     = cmd & 0x7FF;          /* 11-bit size field */

        if (cpu.debug_enabled)
            fprintf(stderr, "[CYW43] PIO gSPI: %s func=%d addr=0x%05X size=%d%s (raw=0x%08X)\n",
                    is_write ? "WR" : "RD", pio_cmd_function,
                    pio_cmd_address, pio_cmd_size,
                    pio_cmd_is_swap ? " (swap)" : "", val);

        if (is_write) {
            pio_cyw43_phase = PIO_CYW43_WRITE;
            pio_words_remaining = (pio_cmd_size + 3) / 4;
            if (pio_words_remaining == 0) pio_words_remaining = 1;

            /* Reset WLAN TX buffer for accumulation */
            if (pio_cmd_function == CYW43_FUNC_WLAN)
                cyw43.wlan_tx_offset = 0;
        } else {
            /* Prepare read response */
            pio_resp_count = 0;
            pio_resp_idx = 0;

            int total_words = (pio_cmd_size + 3) / 4;
            if (total_words <= 0) total_words = 1;
            if (total_words > 512) total_words = 512;

            memset(pio_resp_buf, 0, total_words * 4);

            switch (pio_cmd_function) {
            case CYW43_FUNC_BUS: {
                uint32_t resp = cyw43_bus_read(pio_cmd_address);
                pio_resp_buf[0] = cyw43_encode_resp(resp, pio_cmd_is_swap);
                pio_resp_count = total_words;
                break;
            }
            case CYW43_FUNC_BACKPLANE: {
                /* Backplane reads have CYW43_BACKPLANE_READ_PAD_LEN_BYTES=16 of padding
                 * (for CYW43_USE_SPI). The firmware DMA reads (4 + pad + data) bytes,
                 * returning (rx_length/4 - tx_length/4) = (24/4 - 1) = 5 words from RXF.
                 * The actual response goes at index pad_words = 4. */
                int pad_words = 4; /* CYW43_BACKPLANE_READ_PAD_LEN_BYTES / 4 = 16/4 = 4 */
                int total_words_bp = total_words + pad_words;
                if (total_words_bp > 512) total_words_bp = 512;
                memset(pio_resp_buf, 0, total_words_bp * 4);
                uint32_t resp = cyw43_backplane_read(pio_cmd_address);
                pio_resp_buf[pad_words] = cyw43_encode_resp(resp, pio_cmd_is_swap);
                pio_resp_count = total_words_bp;
                break;
            }
            case CYW43_FUNC_WLAN: {
                /* Return queued RX frame.
                 * Each word in pio_resp_buf must be bswap32 of the LE frame word,
                 * because DMA bswap will reverse it back for the firmware. */
                cyw43_rx_frame_t *f = rx_queue_peek();
                if (f && f->len > 0) {
                    int copy_len = f->len;
                    if (copy_len > (int)pio_cmd_size) copy_len = (int)pio_cmd_size;
                    int words = (copy_len + 3) / 4;
                    if (words > 512) words = 512;
                    memset(pio_resp_buf, 0, words * 4);
                    /* Copy bytes then bswap32 each word so firmware gets correct byte order */
                    memcpy(pio_resp_buf, f->data, copy_len);
                    for (int j = 0; j < words; j++)
                        pio_resp_buf[j] = cyw43_bswap32(pio_resp_buf[j]);
                    pio_resp_count = total_words;
                    rx_queue_pop();

                    if (cpu.debug_enabled)
                        fprintf(stderr, "[CYW43] WLAN RX: delivering %d byte frame (ch=%d seq=%d)\n",
                                copy_len, f->data[4] & 0x0F, f->data[3]);
                } else {
                    /* No data - return empty SDPCM (size=0) */
                    pio_resp_count = total_words;
                }
                break;
            }
            }

            pio_cyw43_phase = PIO_CYW43_READ;
        }
        break;
    }

    case PIO_CYW43_WRITE: {
        /* Decode data word: same transform as the command word */
        uint32_t decoded = cyw43_decode_txf(val, pio_cmd_is_swap);
        switch (pio_cmd_function) {
        case CYW43_FUNC_BUS:
            cyw43_bus_write(pio_cmd_address, decoded);
            break;
        case CYW43_FUNC_BACKPLANE:
            cyw43_backplane_write(pio_cmd_address, decoded);
            break;
        case CYW43_FUNC_WLAN:
            /* decoded = bswap32(TXF) = LE word with correct frame bytes */
            cyw43_wlan_tx_word(decoded);
            break;
        }
        pio_cmd_address += 4;

        if (--pio_words_remaining <= 0) {
            if (pio_cmd_function == CYW43_FUNC_WLAN)
                cyw43_wlan_tx_complete();
            pio_cyw43_phase = PIO_CYW43_IDLE;
        }
        break;
    }

    case PIO_CYW43_READ:
        pio_cyw43_phase = PIO_CYW43_IDLE;
        cyw43_pio_tx_write(val);
        break;
    }
}

uint32_t cyw43_pio_rx_read(void) {
    if (!cyw43.enabled) return 0;

    if (pio_cyw43_phase == PIO_CYW43_READ && pio_resp_idx < pio_resp_count) {
        uint32_t val = pio_resp_buf[pio_resp_idx++];
        if (pio_resp_idx >= pio_resp_count) {
            pio_cyw43_phase = PIO_CYW43_IDLE;
        }
        return val;
    }

    return 0;
}

int cyw43_pio_rx_ready(void) {
    return (pio_cyw43_phase == PIO_CYW43_READ && pio_resp_idx < pio_resp_count);
}

int cyw43_pio_phase_is_idle(void) {
    return (pio_cyw43_phase == PIO_CYW43_IDLE);
}
