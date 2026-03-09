#ifndef USB_H
#define USB_H

#include <stdint.h>

/* ========================================================================
 * RP2040 USB Controller
 *
 * Stub implementation: returns "disconnected" state so SDK falls back
 * to UART. Handles DPRAM (4KB) and controller registers.
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

/* State */
typedef struct {
    uint8_t  dpram[USBCTRL_DPRAM_SIZE]; /* Dual-port RAM */
    uint32_t main_ctrl;
    uint32_t sie_ctrl;
    uint32_t sie_status;
    uint32_t inte;
    uint32_t intf;
    uint32_t usb_muxing;
    uint32_t usb_pwr;
} usb_state_t;

extern usb_state_t usb_state;

/* Functions */
void     usb_init(void);
int      usb_match(uint32_t addr);  /* Returns 1 if addr in USB range */
uint32_t usb_read32(uint32_t addr);
void     usb_write32(uint32_t addr, uint32_t val);

#endif /* USB_H */
