/*
 * CYW43439 WiFi/BT Chip Emulation for Pico W
 *
 * The Pico W communicates with the CYW43439 via GPIO bit-banged SPI:
 *   GPIO 23 = WL_CS (chip select, directly driven)
 *   GPIO 24 = WL_CLK (clock, directly driven)
 *   GPIO 25 = WL_DIO (bidirectional data)
 *   GPIO 24 = WL_IRQ (active-low interrupt to RP2040, active via PIO)
 *
 * The Pico SDK cyw43_driver uses PIO for SPI bit-bang. We intercept
 * at the gSPI protocol level, decoding 32-bit command words.
 *
 * gSPI Protocol:
 *   Command word: [31]=write, [30:28]=function, [27:17]=address, [16:0]=size
 *   Functions: 0=bus, 1=backplane, 2=wlan
 *
 * TAP Bridge (optional):
 *   -tap <ifname> bridges CYW43 WLAN data frames to a Linux TAP interface,
 *   enabling real internet access for emulated Pico W firmware.
 *
 * Usage:
 *   ./bramble firmware.uf2 -wifi              # Stub mode (no networking)
 *   ./bramble firmware.uf2 -wifi -tap tap0    # TAP bridge mode
 */

#ifndef CYW43_H
#define CYW43_H

#include <stdint.h>

/* ======================================================================== */
/* gSPI Functions                                                            */
/* ======================================================================== */

#define CYW43_FUNC_BUS       0   /* Bus/SPI control */
#define CYW43_FUNC_BACKPLANE  1   /* Chip backplane (registers, memory) */
#define CYW43_FUNC_WLAN       2   /* WLAN (802.11 frames) */

/* ======================================================================== */
/* Bus Registers (Function 0)                                                */
/* ======================================================================== */

#define CYW43_REG_BUS_CTRL        0x00
#define CYW43_REG_BUS_INTERRUPT   0x04
#define CYW43_REG_BUS_INTMASK     0x08
#define CYW43_REG_BUS_STATUS      0x0C
#define CYW43_REG_BUS_FEEDBEAD    0x14
#define CYW43_REG_BUS_TEST        0x18

/* Bus interrupt bits */
#define CYW43_BUS_INT_F2_PKT_AVAIL  0x20  /* Bit 5: F2 packet available */

/* ======================================================================== */
/* Backplane Registers (Function 1)                                          */
/* ======================================================================== */

#define CYW43_REG_BP_CHIPID       0x00
#define CYW43_REG_BP_WIN_ADDR     0x1000A
#define CYW43_BP_CHIPCLKCSR       0x1000E  /* Chip clock CSR */

/* CHIPCLKCSR bits */
#define CYW43_HT_AVAIL            0x80
#define CYW43_ALP_AVAIL           0x40

/* ======================================================================== */
/* SDPCM Protocol (Host <-> CYW43 WLAN data framing)                        */
/* ======================================================================== */

/* SDPCM channel types */
#define SDPCM_CONTROL_CHANNEL    0
#define SDPCM_EVENT_CHANNEL      1
#define SDPCM_DATA_CHANNEL       2

/* SDPCM header (12 bytes, little-endian) */
typedef struct __attribute__((packed)) {
    uint16_t size;              /* Total frame size */
    uint16_t size_com;          /* ~size & 0xFFFF (complement) */
    uint8_t  sequence;          /* Sequence number */
    uint8_t  channel_and_flags; /* Channel type (lower 4 bits) */
    uint8_t  next_length;       /* 0 */
    uint8_t  header_length;     /* 12 for control, 14 for data (with pad) */
    uint8_t  wireless_flow_ctrl;
    uint8_t  bus_data_credit;   /* TX credit for host */
    uint8_t  reserved[2];
} sdpcm_header_t;

/* BDC header (4 bytes, after SDPCM + optional 2-byte pad) */
typedef struct __attribute__((packed)) {
    uint8_t  flags;       /* 0x20 (BDC v2) */
    uint8_t  priority;    /* 0 */
    uint8_t  flags2;      /* Interface number (0=STA) */
    uint8_t  data_offset; /* Extra offset in 4-byte units */
} bdc_header_t;

/* CDC/IOCTL header (16 bytes, after SDPCM on control channel) */
typedef struct __attribute__((packed)) {
    uint32_t cmd;     /* WLC_* command number */
    uint32_t len;     /* Output length (lower 16) / input length (upper 16) */
    uint32_t flags;   /* (ioctl_id << 16) | kind | (iface << 12) */
    uint32_t status;  /* 0 for request, result code for response */
} cdc_header_t;

/* CDC flags */
#define CDC_GET      0x00
#define CDC_SET      0x02
#define CDC_REPLY    0x01
#define CDC_ID_SHIFT 16
#define CDC_IF_SHIFT 12

/* ======================================================================== */
/* WLC IOCTL Commands                                                        */
/* ======================================================================== */

#define WLC_UP              2
#define WLC_DOWN            3
#define WLC_SET_INFRA       20
#define WLC_SET_AUTH        22
#define WLC_GET_BSSID       23
#define WLC_GET_SSID        25
#define WLC_SET_SSID        26
#define WLC_SET_CHANNEL     30
#define WLC_DISASSOC        52
#define WLC_SET_PASSIVE_SCAN 49
#define WLC_SET_PM          86
#define WLC_SET_GMODE       110
#define WLC_SET_WSEC        134
#define WLC_SET_BAND        142
#define WLC_SET_WPA_AUTH    165
#define WLC_GET_VAR         262
#define WLC_SET_VAR         263
#define WLC_SET_WSEC_PMK    268

/* ======================================================================== */
/* Async Event Types (big-endian in wire format)                             */
/* ======================================================================== */

#define CYW43_EV_SET_SSID       0
#define CYW43_EV_AUTH            3
#define CYW43_EV_DEAUTH         5
#define CYW43_EV_DISASSOC        12
#define CYW43_EV_LINK            16
#define CYW43_EV_PSK_SUP         46
#define CYW43_EV_ESCAN_RESULT    69

/* Event status codes */
#define CYW43_STATUS_SUCCESS     0
#define CYW43_SUP_KEYED          6

/* Broadcom event ethertype */
#define BCMILCP_ETHER_TYPE       0x886C
#define BCMILCP_BCM_SUBTYPE_EVENT 0x8002

/* ======================================================================== */
/* WiFi Connection States                                                    */
/* ======================================================================== */

#define CYW43_WIFI_OFF        0
#define CYW43_WIFI_SCANNING   1
#define CYW43_WIFI_CONNECTING 2
#define CYW43_WIFI_CONNECTED  3

/* ======================================================================== */
/* Limits                                                                    */
/* ======================================================================== */

#define CYW43_MAX_SCAN_RESULTS  8
#define CYW43_MAX_SSID_LEN     32
#define CYW43_MAC_LEN           6
#define CYW43_MAX_FRAME_SIZE   2048
#define CYW43_RX_QUEUE_SIZE    32
#define CYW43_WLAN_TX_BUF_SIZE 2048

/* ======================================================================== */
/* Scan Result                                                               */
/* ======================================================================== */

typedef struct {
    char ssid[CYW43_MAX_SSID_LEN + 1];
    uint8_t bssid[CYW43_MAC_LEN];
    int16_t rssi;
    uint8_t channel;
    uint8_t auth_mode;    /* 0=open, 1=WEP, 2=WPA, 3=WPA2 */
} cyw43_scan_result_t;

/* ======================================================================== */
/* gSPI Transaction State                                                    */
/* ======================================================================== */

typedef struct {
    int cs_active;
    uint32_t cmd_word;
    int cmd_bits;
    int in_data_phase;
    int is_write;
    int function;
    uint32_t address;
    uint32_t size;
    uint32_t data_offset;

    uint8_t resp_buf[2048];
    uint32_t resp_len;
    uint32_t resp_offset;
} cyw43_spi_state_t;

/* ======================================================================== */
/* RX Frame Queue (device -> host)                                           */
/* ======================================================================== */

typedef struct {
    uint8_t data[CYW43_MAX_FRAME_SIZE];
    int len;
} cyw43_rx_frame_t;

/* ======================================================================== */
/* CYW43 Emulator State                                                      */
/* ======================================================================== */

typedef struct {
    int enabled;
    int initialized;

    /* gSPI state */
    cyw43_spi_state_t spi;

    /* Bus registers (function 0) */
    uint32_t bus_test_reg;
    uint32_t bus_ctrl;
    uint32_t bus_int;
    uint32_t bus_intmask;
    uint32_t bus_status;

    /* Backplane (function 1) */
    uint32_t bp_window;
    uint32_t chipclkcsr;

    /* WiFi state */
    int wifi_state;
    char connected_ssid[CYW43_MAX_SSID_LEN + 1];
    uint8_t mac_addr[CYW43_MAC_LEN];
    uint32_t ip_addr;

    /* Scan results */
    cyw43_scan_result_t scan_results[CYW43_MAX_SCAN_RESULTS];
    int scan_count;

    /* Country code */
    char country[4];

    /* TAP bridge */
    int tap_fd;
    char tap_name[32];

    /* WLAN TX accumulation buffer */
    uint8_t wlan_tx_buf[CYW43_WLAN_TX_BUF_SIZE];
    int wlan_tx_offset;

    /* RX frame queue (IOCTL responses, events, data from TAP) */
    cyw43_rx_frame_t rx_queue[CYW43_RX_QUEUE_SIZE];
    int rx_head;
    int rx_tail;

    /* SDPCM sequencing */
    uint8_t tx_seq;          /* Our TX sequence (for responses/events) */
    uint8_t last_fw_seq;     /* Last sequence received from firmware */
    uint16_t ioctl_id_next;  /* Next IOCTL ID for matching */

    /* Auto-detected PIO/SM for gSPI (set when sideset pin == WL_CLK) */
    int pio_num;             /* -1 = not detected yet */
    int pio_sm;
} cyw43_state_t;

extern cyw43_state_t cyw43;

/* Initialize CYW43 emulation */
void cyw43_init(void);

/* Reset CYW43 to power-on defaults */
void cyw43_reset(void);

/* GPIO intercept */
int cyw43_gpio_intercept(uint32_t gpio, uint32_t value);
uint32_t cyw43_gpio_read_dio(void);
int cyw43_is_wifi_gpio(uint32_t gpio);

/* Add a fake AP to scan results */
void cyw43_add_scan_result(const char *ssid, int rssi, int channel, int auth);

/* PIO-level intercept */
void cyw43_pio_tx_write(uint32_t val);
uint32_t cyw43_pio_rx_read(void);
int cyw43_pio_rx_ready(void);
void cyw43_pio_sm_restart(void);
int cyw43_pio_phase_is_idle(void);

/* TAP bridge */
int cyw43_tap_open(const char *name);
void cyw43_tap_close(void);
void cyw43_tap_poll(void);

#endif /* CYW43_H */
