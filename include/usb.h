#ifndef USB_H
#define USB_H

#include <stdint.h>

/* ========================================================================
 * RP2040 USB Controller with Host Enumeration Simulation
 *
 * Simulates a USB host that enumerates the device and bridges CDC
 * data to emulator stdout/stdin. Firmware sees a fully connected
 * USB CDC device.
 * ======================================================================== */

/* Base addresses */
#define USBCTRL_DPRAM_BASE      0x50100000  /* USB dual-port RAM (4KB) */
#define USBCTRL_DPRAM_SIZE      0x1000
#define USBCTRL_REGS_BASE       0x50110000  /* USB controller registers */
#define USBCTRL_REGS_SIZE       0x1000

/* Register offsets (from USBCTRL_REGS_BASE) */
#define USB_ADDR_ENDP           0x00
#define USB_MAIN_CTRL           0x40
#define USB_SOF_WR              0x44
#define USB_SOF_RD              0x48
#define USB_SIE_CTRL            0x4C
#define USB_SIE_STATUS          0x50
#define USB_INT_EP_CTRL         0x54
#define USB_BUFF_STATUS         0x58
#define USB_BUFF_CPU_SHOULD_HANDLE 0x5C
#define USB_EP_ABORT            0x60
#define USB_EP_ABORT_DONE       0x64
#define USB_EP_STALL_ARM        0x68
#define USB_NAK_POLL            0x6C
#define USB_EP_STATUS_STALL_NAK 0x70
#define USB_USB_MUXING          0x74
#define USB_USB_PWR             0x78
#define USB_USBPHY_DIRECT       0x7C
#define USB_USBPHY_DIRECT_OVERRIDE 0x80
#define USB_USBPHY_TRIM         0x84
#define USB_INTR                0x8C
#define USB_INTE                0x90
#define USB_INTF                0x94
#define USB_INTS                0x98

/* SIE_STATUS bits */
#define USB_SIE_VBUS_DETECTED   (1u << 0)
#define USB_SIE_SUSPENDED       (1u << 4)
#define USB_SIE_CONNECTED       (1u << 16)
#define USB_SIE_SETUP_REC       (1u << 17)
#define USB_SIE_TRANS_COMPLETE  (1u << 18)
#define USB_SIE_BUS_RESET       (1u << 19)
#define USB_SIE_ACK_REC         (1u << 30)

/* SIE_CTRL bits */
#define USB_SIE_CTRL_PULLUP_EN  (1u << 16)

/* MAIN_CTRL bits */
#define USB_MAIN_CTRL_EN        (1u << 0)

/* INTR bits */
#define USB_INTR_TRANS_COMPLETE (1u << 3)
#define USB_INTR_BUFF_STATUS    (1u << 4)
#define USB_INTR_BUS_RESET      (1u << 12)
#define USB_INTR_SETUP_REQ      (1u << 16)

/* Buffer control bits (BUFFER0, single-buffered mode)
 * RP2040 DPRAM EP buffer control register layout:
 *   [31:16] = BUFFER1 (for double-buffered EPs)
 *   [15:0]  = BUFFER0 (always used for single-buffered) */
#define USB_BUF_CTRL_FULL       (1u << 15)
#define USB_BUF_CTRL_LAST       (1u << 14)
#define USB_BUF_CTRL_DATA_PID   (1u << 13)
#define USB_BUF_CTRL_RESET      (1u << 12)
#define USB_BUF_CTRL_STALL      (1u << 11)
#define USB_BUF_CTRL_AVAILABLE  (1u << 10)
#define USB_BUF_CTRL_LEN_MASK   0x3FF

/* DPRAM layout */
#define USB_DPRAM_SETUP         0x000   /* Setup packet (8 bytes) */
#define USB_DPRAM_EP_CTRL       0x008   /* EP1+ control registers */
#define USB_DPRAM_BUF_CTRL      0x080   /* Buffer control registers */
#define USB_DPRAM_EP0_BUF       0x100   /* EP0 data buffer (64 bytes) */

/* Enumeration state machine */
typedef enum {
    USB_ENUM_DISABLED,
    USB_ENUM_WAIT_PULLUP,
    USB_ENUM_BUS_RESET,
    USB_ENUM_GET_DESC_8,
    USB_ENUM_SET_ADDRESS,
    USB_ENUM_GET_DESC_FULL,
    USB_ENUM_GET_CONFIG_SHORT,
    USB_ENUM_GET_CONFIG_FULL,
    USB_ENUM_SET_CONFIG,
    USB_ENUM_CDC_SET_LINE_CODING,
    USB_ENUM_CDC_SET_CTRL_LINE,
    USB_ENUM_ACTIVE,
} usb_enum_state_t;

/* Control transfer phase */
typedef enum {
    USB_CTRL_IDLE,
    USB_CTRL_SETUP_SENT,
    USB_CTRL_WAIT_DATA_IN,
    USB_CTRL_WAIT_STATUS_OUT,
    USB_CTRL_WAIT_DATA_OUT,
    USB_CTRL_WAIT_STATUS_IN,
    USB_CTRL_DONE,
} usb_ctrl_state_t;

/* State */
typedef struct {
    uint8_t  dpram[USBCTRL_DPRAM_SIZE]; /* Dual-port RAM */
    uint32_t main_ctrl;
    uint32_t sie_ctrl;
    uint32_t sie_status;
    uint32_t buff_status;
    uint32_t inte;
    uint32_t intf;
    uint32_t usb_muxing;
    uint32_t usb_pwr;
    uint32_t addr_endp;
    uint32_t int_ep_ctrl;
    uint32_t ep_abort;
    uint32_t ep_abort_done;
    uint32_t ep_stall_arm;
    uint32_t ep_status_stall_nak;

    /* Host enumeration simulation */
    usb_enum_state_t enum_state;
    usb_ctrl_state_t ctrl_state;
    int delay;                /* Steps to wait before next action */
    int config_total_len;     /* Total configuration descriptor length */
    int cdc_iface;            /* CDC interface number (for class requests) */

    /* CDC bulk endpoint tracking */
    int cdc_in_ep;            /* CDC bulk IN endpoint number (0 = not found) */
    int cdc_out_ep;           /* CDC bulk OUT endpoint number (0 = not found) */

    /* OUT data for control transfers */
    uint8_t out_data[64];
    int out_data_len;

    /* Multi-packet IN accumulation */
    uint8_t in_accum[256];   /* Accumulated IN data */
    int in_accum_len;        /* Bytes accumulated so far */
    int in_expected_len;     /* Total bytes expected (wLength from setup) */

    /* CDC RX FIFO (stdin → device OUT endpoint) */
    uint8_t cdc_rx_fifo[256];
    int cdc_rx_head;
    int cdc_rx_tail;
    int cdc_rx_count;
} usb_state_t;

extern usb_state_t usb_state;

/* Functions */
void     usb_init(void);
int      usb_match(uint32_t addr);  /* Returns 1 if addr in USB range */
uint32_t usb_read32(uint32_t addr);
void     usb_write32(uint32_t addr, uint32_t val);

/* Called from main loop to advance USB host simulation */
void     usb_step(void);

/* Push a byte into USB CDC OUT endpoint (for stdin bridging) */
void     usb_cdc_rx_push(uint8_t byte);

/* Set to 1 to allow USB CDC IN data to appear on stdout (set by -stdin flag) */
extern int usb_cdc_stdout_enabled;

#endif /* USB_H */
