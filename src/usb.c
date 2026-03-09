/*
 * RP2040 USB Controller Emulation (Stub)
 *
 * Returns "disconnected" state for all status registers.
 * DPRAM is backed by real memory so SDK initialization code can
 * write endpoint descriptors without crashing.
 * SDK's stdio_usb_init() times out (~500ms) and falls back to UART.
 */

#include <string.h>
#include "usb.h"

usb_state_t usb_state;

void usb_init(void) {
    memset(&usb_state, 0, sizeof(usb_state_t));
}

int usb_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;  /* Strip atomic alias bits */
    if (base >= USBCTRL_DPRAM_BASE && base < USBCTRL_DPRAM_BASE + USBCTRL_DPRAM_SIZE)
        return 1;
    if (base >= USBCTRL_REGS_BASE && base < USBCTRL_REGS_BASE + USBCTRL_REGS_SIZE)
        return 1;
    return 0;
}

uint32_t usb_read32(uint32_t addr) {
    uint32_t base = addr & ~0x3000;

    /* DPRAM reads */
    if (base >= USBCTRL_DPRAM_BASE && base < USBCTRL_DPRAM_BASE + USBCTRL_DPRAM_SIZE) {
        uint32_t off = base - USBCTRL_DPRAM_BASE;
        uint32_t val;
        memcpy(&val, &usb_state.dpram[off], 4);
        return val;
    }

    /* Controller registers */
    uint32_t offset = base - USBCTRL_REGS_BASE;
    switch (offset) {
    case USB_MAIN_CTRL:
        return usb_state.main_ctrl;
    case USB_SIE_CTRL:
        return usb_state.sie_ctrl;
    case USB_SIE_STATUS:
        /* All zeros = no VBUS detected, not connected */
        return 0;
    case USB_BUFF_STATUS:
        return 0;  /* No buffers pending */
    case USB_BUFF_CPU_SHOULD_HANDLE:
        return 0;
    case USB_EP_STATUS_STALL_NAK:
        return 0;
    case USB_USB_MUXING:
        return usb_state.usb_muxing;
    case USB_USB_PWR:
        return usb_state.usb_pwr;
    case USB_INTR:
        return 0;  /* No interrupts */
    case USB_INTE:
        return usb_state.inte;
    case USB_INTF:
        return usb_state.intf;
    case USB_INTS:
        return 0;  /* No active interrupts */
    case USB_SOF_RD:
        return 0;
    case USB_EP_ABORT_DONE:
        return 0xFFFFFFFF;  /* All aborts complete */
    case USB_ADDR_ENDP:
        return 0;
    default:
        return 0;
    }
}

void usb_write32(uint32_t addr, uint32_t val) {
    uint32_t base = addr & ~0x3000;
    uint32_t alias = (addr >> 12) & 0x3;

    /* DPRAM writes */
    if (base >= USBCTRL_DPRAM_BASE && base < USBCTRL_DPRAM_BASE + USBCTRL_DPRAM_SIZE) {
        uint32_t off = base - USBCTRL_DPRAM_BASE;
        memcpy(&usb_state.dpram[off], &val, 4);
        return;
    }

    /* Controller registers */
    uint32_t offset = base - USBCTRL_REGS_BASE;

    /* Apply atomic alias */
    #define ALIAS_APPLY(reg) do { \
        switch (alias) { \
            case 0: (reg) = val; break; \
            case 1: (reg) ^= val; break; \
            case 2: (reg) |= val; break; \
            case 3: (reg) &= ~val; break; \
        } \
    } while(0)

    switch (offset) {
    case USB_MAIN_CTRL:
        ALIAS_APPLY(usb_state.main_ctrl);
        break;
    case USB_SIE_CTRL:
        ALIAS_APPLY(usb_state.sie_ctrl);
        break;
    case USB_SIE_STATUS:
        /* W1C — but status is always 0, so no-op */
        break;
    case USB_BUFF_STATUS:
        /* W1C — nothing pending */
        break;
    case USB_USB_MUXING:
        ALIAS_APPLY(usb_state.usb_muxing);
        break;
    case USB_USB_PWR:
        ALIAS_APPLY(usb_state.usb_pwr);
        break;
    case USB_INTE:
        ALIAS_APPLY(usb_state.inte);
        break;
    case USB_INTF:
        ALIAS_APPLY(usb_state.intf);
        break;
    default:
        break;  /* Accept silently */
    }

    #undef ALIAS_APPLY
}
