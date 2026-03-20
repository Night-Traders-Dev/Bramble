/*
 * RP2040 USB Controller with Host Enumeration Simulation
 *
 * Simulates a USB host performing device enumeration:
 *   1. Bus reset
 *   2. GET_DEVICE_DESCRIPTOR (8 bytes)
 *   3. SET_ADDRESS
 *   4. GET_DEVICE_DESCRIPTOR (full)
 *   5. GET_CONFIGURATION_DESCRIPTOR
 *   6. SET_CONFIGURATION
 *   7. CDC SET_LINE_CODING + SET_CONTROL_LINE_STATE
 *
 * After enumeration, CDC bulk endpoints are bridged to stdout/stdin.
 */

#include <string.h>
#include <stdio.h>
#include "usb.h"
#include "nvic.h"
#include "devtools.h"

usb_state_t usb_state;
int usb_cdc_stdout_enabled = 0;

/* ========================================================================
 * DPRAM Helpers
 * ======================================================================== */

static uint32_t dpram_read32(uint32_t off) {
    uint32_t val;
    memcpy(&val, &usb_state.dpram[off], 4);
    return val;
}

static void dpram_write32(uint32_t off, uint32_t val) {
    memcpy(&usb_state.dpram[off], &val, 4);
}

/* ========================================================================
 * Interrupt Handling
 * ======================================================================== */

static uint32_t usb_compute_intr(void) {
    uint32_t intr = 0;
    if (usb_state.buff_status)
        intr |= USB_INTR_BUFF_STATUS;
    if (usb_state.sie_status & USB_SIE_BUS_RESET)
        intr |= USB_INTR_BUS_RESET;
    if (usb_state.sie_status & USB_SIE_SETUP_REC)
        intr |= USB_INTR_SETUP_REQ;
    if (usb_state.sie_status & USB_SIE_TRANS_COMPLETE)
        intr |= USB_INTR_TRANS_COMPLETE;
    return intr;
}

static void usb_fire_irq(void) {
    uint32_t intr = usb_compute_intr();
    uint32_t ints = (intr | usb_state.intf) & usb_state.inte;
    if (ints) {
        nvic_signal_irq(5);  /* USBCTRL_IRQ */
    }
}

/* ========================================================================
 * Setup Packet Injection
 * ======================================================================== */

static void usb_send_setup(uint8_t bmRequestType, uint8_t bRequest,
                           uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
    usb_state.dpram[0] = bmRequestType;
    usb_state.dpram[1] = bRequest;
    usb_state.dpram[2] = wValue & 0xFF;
    usb_state.dpram[3] = (wValue >> 8) & 0xFF;
    usb_state.dpram[4] = wIndex & 0xFF;
    usb_state.dpram[5] = (wIndex >> 8) & 0xFF;
    usb_state.dpram[6] = wLength & 0xFF;
    usb_state.dpram[7] = (wLength >> 8) & 0xFF;

    usb_state.sie_status |= USB_SIE_SETUP_REC;
    usb_state.ctrl_state = USB_CTRL_SETUP_SENT;
    usb_fire_irq();
}

/* Complete an EP0 IN transfer (device sent data to us) */
static void usb_complete_ep0_in(void) {
    uint32_t buf_ctrl = dpram_read32(USB_DPRAM_BUF_CTRL);  /* EP0 IN at 0x080 */
    buf_ctrl &= ~(USB_BUF_CTRL_AVAILABLE | USB_BUF_CTRL_FULL);
    dpram_write32(USB_DPRAM_BUF_CTRL, buf_ctrl);
    usb_state.buff_status |= (1u << 0);  /* EP0_IN */
    usb_fire_irq();
}

/* Complete an EP0 OUT transfer (we sent data to device) */
static void usb_complete_ep0_out(uint8_t *data, int len) {
    /* Write data to EP0 buffer */
    if (data && len > 0) {
        memcpy(&usb_state.dpram[USB_DPRAM_EP0_BUF], data, len);
    }
    uint32_t buf_ctrl = dpram_read32(USB_DPRAM_BUF_CTRL + 4);  /* EP0 OUT at 0x084 */
    buf_ctrl &= ~USB_BUF_CTRL_AVAILABLE;
    buf_ctrl |= USB_BUF_CTRL_FULL;
    buf_ctrl = (buf_ctrl & ~USB_BUF_CTRL_LEN_MASK) | (len & USB_BUF_CTRL_LEN_MASK);
    dpram_write32(USB_DPRAM_BUF_CTRL + 4, buf_ctrl);
    usb_state.buff_status |= (1u << 1);  /* EP0_OUT */
    usb_fire_irq();
}

/* ========================================================================
 * Control Transfer State Machine
 * ======================================================================== */

static void usb_ctrl_step(void) {
    uint32_t buf_ctrl;
    switch (usb_state.ctrl_state) {
    case USB_CTRL_IDLE:
    case USB_CTRL_DONE:
        break;

    case USB_CTRL_SETUP_SENT:
        /* Setup just sent — firmware needs to process it.
         * Check if SETUP_REC was cleared by firmware (W1C). */
        if (!(usb_state.sie_status & USB_SIE_SETUP_REC)) {
            /* Firmware cleared it, now determine transfer type from setup packet */
            uint8_t bmRequestType = usb_state.dpram[0];
            uint16_t wLength = usb_state.dpram[6] | (usb_state.dpram[7] << 8);

            if (wLength == 0) {
                /* No data phase — wait for status IN (ZLP) */
                usb_state.ctrl_state = USB_CTRL_WAIT_STATUS_IN;
            } else if (bmRequestType & 0x80) {
                /* IN transfer (device → host) — track expected length for multi-packet */
                usb_state.in_accum_len = 0;
                usb_state.in_expected_len = wLength;
                usb_state.ctrl_state = USB_CTRL_WAIT_DATA_IN;
            } else {
                /* OUT transfer (host → device) */
                usb_state.ctrl_state = USB_CTRL_WAIT_DATA_OUT;
            }
        }
        break;

    case USB_CTRL_WAIT_DATA_IN:
        buf_ctrl = dpram_read32(USB_DPRAM_BUF_CTRL);  /* EP0 IN */
        if ((buf_ctrl & USB_BUF_CTRL_AVAILABLE) && (buf_ctrl & USB_BUF_CTRL_FULL)) {
            /* Firmware has data ready — accumulate into in_accum buffer */
            int pkt_len = buf_ctrl & USB_BUF_CTRL_LEN_MASK;
            if (pkt_len > 0 && usb_state.in_accum_len + pkt_len <= (int)sizeof(usb_state.in_accum)) {
                memcpy(&usb_state.in_accum[usb_state.in_accum_len],
                       &usb_state.dpram[USB_DPRAM_EP0_BUF], pkt_len);
                usb_state.in_accum_len += pkt_len;
            }
            usb_complete_ep0_in();

            /* Check if transfer is complete:
             * - received all expected bytes, OR
             * - short packet (less than max packet size = 64), OR
             * - zero-length packet */
            if (usb_state.in_accum_len >= usb_state.in_expected_len ||
                pkt_len < 64 || pkt_len == 0) {
                /* Copy accumulated data back to EP0 buffer for descriptor parsing */
                int copy_len = usb_state.in_accum_len;
                if (copy_len > (int)sizeof(usb_state.dpram) - USB_DPRAM_EP0_BUF)
                    copy_len = (int)sizeof(usb_state.dpram) - USB_DPRAM_EP0_BUF;
                memcpy(&usb_state.dpram[USB_DPRAM_EP0_BUF],
                       usb_state.in_accum, copy_len);
                /* Update buf_ctrl with total length for parsers */
                uint32_t final_bc = dpram_read32(USB_DPRAM_BUF_CTRL);
                final_bc = (final_bc & ~USB_BUF_CTRL_LEN_MASK) |
                           (copy_len & USB_BUF_CTRL_LEN_MASK);
                dpram_write32(USB_DPRAM_BUF_CTRL, final_bc);
                usb_state.ctrl_state = USB_CTRL_WAIT_STATUS_OUT;
            }
            /* else: wait for next packet */
        }
        break;

    case USB_CTRL_WAIT_STATUS_OUT:
        buf_ctrl = dpram_read32(USB_DPRAM_BUF_CTRL + 4);  /* EP0 OUT */
        if (buf_ctrl & USB_BUF_CTRL_AVAILABLE) {
            /* Firmware ready for status ZLP */
            usb_complete_ep0_out(NULL, 0);
            usb_state.ctrl_state = USB_CTRL_DONE;
        }
        break;

    case USB_CTRL_WAIT_DATA_OUT:
        buf_ctrl = dpram_read32(USB_DPRAM_BUF_CTRL + 4);  /* EP0 OUT */
        if (buf_ctrl & USB_BUF_CTRL_AVAILABLE) {
            /* Firmware ready to receive data */
            usb_complete_ep0_out(usb_state.out_data, usb_state.out_data_len);
            usb_state.ctrl_state = USB_CTRL_WAIT_STATUS_IN;
        }
        break;

    case USB_CTRL_WAIT_STATUS_IN:
        buf_ctrl = dpram_read32(USB_DPRAM_BUF_CTRL);  /* EP0 IN */
        if (buf_ctrl & USB_BUF_CTRL_AVAILABLE) {
            /* Firmware sending status ZLP */
            usb_complete_ep0_in();
            usb_state.ctrl_state = USB_CTRL_DONE;
        }
        break;
    }
}

/* ========================================================================
 * Enumeration State Machine
 * ======================================================================== */

/* Parse configuration descriptor to find CDC bulk endpoints */
static void usb_parse_config_desc(void) {
    uint8_t *buf = &usb_state.dpram[USB_DPRAM_EP0_BUF];
    uint32_t buf_ctrl = dpram_read32(USB_DPRAM_BUF_CTRL);
    int total_len = buf_ctrl & USB_BUF_CTRL_LEN_MASK;
    if (total_len < 9) return;

    int pos = 0;
    int in_cdc_data = 0;

    while (pos + 1 < total_len) {
        int desc_len = buf[pos];
        int desc_type = buf[pos + 1];
        if (desc_len < 2 || pos + desc_len > total_len) break;

        if (desc_type == 4 && desc_len >= 9) {
            /* Interface descriptor */
            int iface_class = buf[pos + 5];
            int iface_subclass = buf[pos + 6];
            if (iface_class == 0x0A) {
                /* CDC Data class — endpoints are here */
                in_cdc_data = 1;
            } else {
                if (iface_class == 0x02 && iface_subclass == 0x02) {
                    /* CDC ACM Communication interface — class requests go here */
                    usb_state.cdc_iface = buf[pos + 2];
                }
                in_cdc_data = 0;
            }
        }

        if (desc_type == 5 && desc_len >= 7 && in_cdc_data) {
            /* Endpoint descriptor in CDC Data interface */
            int ep_addr = buf[pos + 2];
            int ep_attr = buf[pos + 3] & 0x03;
            if (ep_attr == 2) {  /* Bulk */
                if (ep_addr & 0x80) {
                    usb_state.cdc_in_ep = ep_addr & 0x0F;
                } else {
                    usb_state.cdc_out_ep = ep_addr & 0x0F;
                }
            }
        }

        pos += desc_len;
    }
}

static void usb_enum_step(void) {
    if (usb_state.ctrl_state != USB_CTRL_IDLE &&
        usb_state.ctrl_state != USB_CTRL_DONE)
        return;  /* Transfer in progress */

    if (usb_state.delay > 0) {
        usb_state.delay--;
        return;
    }

    switch (usb_state.enum_state) {
    case USB_ENUM_DISABLED:
        break;

    case USB_ENUM_WAIT_PULLUP:
        if (usb_state.sie_ctrl & USB_SIE_CTRL_PULLUP_EN) {
            usb_state.enum_state = USB_ENUM_BUS_RESET;
            usb_state.delay = 500;
        }
        break;

    case USB_ENUM_BUS_RESET:
        usb_state.sie_status |= USB_SIE_BUS_RESET | USB_SIE_CONNECTED;
        usb_fire_irq();
        usb_state.enum_state = USB_ENUM_GET_DESC_8;
        usb_state.delay = 500;
        usb_state.ctrl_state = USB_CTRL_IDLE;
        break;

    case USB_ENUM_GET_DESC_8:
        /* GET_DEVICE_DESCRIPTOR, first 8 bytes */
        usb_send_setup(0x80, 6, 0x0100, 0, 8);
        usb_state.enum_state = USB_ENUM_SET_ADDRESS;
        break;

    case USB_ENUM_SET_ADDRESS:
        if (usb_state.ctrl_state == USB_CTRL_DONE) {
            /* Assign address 1 */
            usb_send_setup(0x00, 5, 1, 0, 0);
            usb_state.enum_state = USB_ENUM_GET_DESC_FULL;
            usb_state.delay = 50;
        }
        break;

    case USB_ENUM_GET_DESC_FULL:
        if (usb_state.ctrl_state == USB_CTRL_DONE) {
            usb_state.ctrl_state = USB_CTRL_IDLE;
            /* GET_DEVICE_DESCRIPTOR, full 18 bytes */
            usb_send_setup(0x80, 6, 0x0100, 0, 18);
            usb_state.enum_state = USB_ENUM_GET_CONFIG_SHORT;
        }
        break;

    case USB_ENUM_GET_CONFIG_SHORT:
        if (usb_state.ctrl_state == USB_CTRL_DONE) {
            usb_state.ctrl_state = USB_CTRL_IDLE;
            /* GET_CONFIGURATION_DESCRIPTOR, first 9 bytes to get total length */
            usb_send_setup(0x80, 6, 0x0200, 0, 9);
            usb_state.enum_state = USB_ENUM_GET_CONFIG_FULL;
        }
        break;

    case USB_ENUM_GET_CONFIG_FULL:
        if (usb_state.ctrl_state == USB_CTRL_DONE) {
            /* Read wTotalLength from config descriptor */
            uint8_t *buf = &usb_state.dpram[USB_DPRAM_EP0_BUF];
            usb_state.config_total_len = buf[2] | (buf[3] << 8);
            if (usb_state.config_total_len > 64) usb_state.config_total_len = 64;
            usb_state.ctrl_state = USB_CTRL_IDLE;
            /* GET_CONFIGURATION_DESCRIPTOR, full length */
            usb_send_setup(0x80, 6, 0x0200, 0, usb_state.config_total_len);
            usb_state.enum_state = USB_ENUM_SET_CONFIG;
        }
        break;

    case USB_ENUM_SET_CONFIG:
        if (usb_state.ctrl_state == USB_CTRL_DONE) {
            /* Parse config descriptor for CDC endpoints */
            usb_parse_config_desc();
            /* If parsing didn't find CDC endpoints (truncated descriptor),
             * use standard CDC-ACM defaults: EP2 IN, EP2 OUT */
            if (usb_state.cdc_in_ep == 0) usb_state.cdc_in_ep = 2;
            if (usb_state.cdc_out_ep == 0) usb_state.cdc_out_ep = 2;
            usb_state.ctrl_state = USB_CTRL_IDLE;
            /* SET_CONFIGURATION 1 */
            usb_send_setup(0x00, 9, 1, 0, 0);
            /* Skip SET_LINE_CODING (causes hangs with some firmware),
             * go directly to SET_CONTROL_LINE_STATE for DTR/RTS.
             * Delay to let firmware complete USB stack init before DTR. */
            usb_state.enum_state = USB_ENUM_CDC_SET_CTRL_LINE;
            usb_state.delay = 500;
        }
        break;

    case USB_ENUM_CDC_SET_LINE_CODING:
        if (usb_state.ctrl_state == USB_CTRL_DONE) {
            usb_state.ctrl_state = USB_CTRL_IDLE;
            /* SET_LINE_CODING: 115200 baud, 8N1 */
            usb_state.out_data[0] = 0x00; usb_state.out_data[1] = 0xC2;
            usb_state.out_data[2] = 0x01; usb_state.out_data[3] = 0x00; /* 115200 LE */
            usb_state.out_data[4] = 0x00; /* 1 stop bit */
            usb_state.out_data[5] = 0x00; /* No parity */
            usb_state.out_data[6] = 0x08; /* 8 data bits */
            usb_state.out_data_len = 7;
            usb_send_setup(0x21, 0x20, 0, usb_state.cdc_iface, 7);
            usb_state.enum_state = USB_ENUM_CDC_SET_CTRL_LINE;
        }
        break;

    case USB_ENUM_CDC_SET_CTRL_LINE:
        if (usb_state.ctrl_state == USB_CTRL_DONE) {
            usb_state.ctrl_state = USB_CTRL_IDLE;
            /* SET_CONTROL_LINE_STATE: DTR=1, RTS=1 */
            usb_send_setup(0x21, 0x22, 0x0003, usb_state.cdc_iface, 0);
            usb_state.enum_state = USB_ENUM_ACTIVE;
        }
        break;

    case USB_ENUM_ACTIVE:
        if (usb_state.ctrl_state == USB_CTRL_DONE) {
            usb_state.ctrl_state = USB_CTRL_IDLE;
        }
        break;
    }
}

/* ========================================================================
 * CDC Data Path
 * ======================================================================== */

static void usb_handle_cdc(void) {
    if (usb_state.cdc_in_ep == 0) return;

    /* Check CDC bulk IN endpoint for data from device */
    int ep = usb_state.cdc_in_ep;
    uint32_t buf_ctrl_off = USB_DPRAM_BUF_CTRL + ep * 8;  /* IN */
    uint32_t buf_ctrl = dpram_read32(buf_ctrl_off);

    if ((buf_ctrl & USB_BUF_CTRL_FULL) && (buf_ctrl & USB_BUF_CTRL_AVAILABLE)) {
        int len = buf_ctrl & USB_BUF_CTRL_LEN_MASK;

        /* Get buffer address from EP control register */
        uint32_t ep_ctrl_off = USB_DPRAM_EP_CTRL + (ep - 1) * 8;  /* IN control */
        uint32_t ep_ctrl = dpram_read32(ep_ctrl_off);
        uint32_t buf_addr = (ep_ctrl & 0xFFFF) & ~0x3F;  /* bits [15:6] << 0, already aligned */
        /* Actually bits [15:6] are the address divided by 64 ... no, bits [15:6] ARE the address with lower 6 bits zero */
        /* RP2040: BUFFER_ADDRESS is bits [15:6], shift not needed since they store byte address with 64-byte alignment */
        buf_addr = ep_ctrl & 0xFFC0;  /* bits [15:6] */

        if (buf_addr > 0 && buf_addr + len <= USBCTRL_DPRAM_SIZE) {
            /* Output CDC data to stdout only when USB CDC stdio is primary
             * (i.e. -stdin mode). When UART stdio is also active, UART
             * already handles stdout and we must not duplicate the data. */
            if (usb_cdc_stdout_enabled) {
                fwrite(&usb_state.dpram[buf_addr], 1, len, stdout);
                fflush(stdout);
                if (__builtin_expect(expect_enabled, 0))
                    expect_append((const char *)&usb_state.dpram[buf_addr], len);
            }
        }

        /* Complete the transfer */
        buf_ctrl &= ~(USB_BUF_CTRL_AVAILABLE | USB_BUF_CTRL_FULL);
        dpram_write32(buf_ctrl_off, buf_ctrl);
        usb_state.buff_status |= (1u << (ep * 2));  /* EPn_IN */
        usb_fire_irq();
    }
}

/* Push a byte into the CDC OUT endpoint (for stdin input) */
int usb_cdc_rx_push(uint8_t byte) {
    if (usb_state.enum_state != USB_ENUM_ACTIVE) return 0;
    if (usb_state.cdc_out_ep == 0) return 0;

    /* Queue into CDC RX FIFO */
    if (usb_state.cdc_rx_count < (int)sizeof(usb_state.cdc_rx_fifo)) {
        usb_state.cdc_rx_fifo[usb_state.cdc_rx_head] = byte;
        usb_state.cdc_rx_head = (usb_state.cdc_rx_head + 1) % (int)sizeof(usb_state.cdc_rx_fifo);
        usb_state.cdc_rx_count++;
        return 1;
    }

    return 0;
}

int usb_cdc_stdio_active(void) {
    return usb_state.enum_state == USB_ENUM_ACTIVE &&
           usb_state.cdc_in_ep != 0 &&
           usb_state.cdc_out_ep != 0;
}

/* Drain CDC RX FIFO into the OUT endpoint when firmware has buffer available */
static void usb_cdc_rx_drain(void) {
    if (usb_state.cdc_rx_count == 0) return;
    if (usb_state.cdc_out_ep == 0) return;

    int ep = usb_state.cdc_out_ep;
    uint32_t buf_ctrl_off = USB_DPRAM_BUF_CTRL + ep * 8 + 4;  /* OUT */
    uint32_t buf_ctrl = dpram_read32(buf_ctrl_off);

    if (buf_ctrl & USB_BUF_CTRL_AVAILABLE) {
        /* Get buffer address and max packet size */
        uint32_t ep_ctrl_off = USB_DPRAM_EP_CTRL + (ep - 1) * 8 + 4;  /* OUT control */
        uint32_t ep_ctrl = dpram_read32(ep_ctrl_off);
        uint32_t buf_addr = ep_ctrl & 0xFFC0;
        int max_pkt = buf_ctrl & USB_BUF_CTRL_LEN_MASK;
        if (max_pkt == 0) max_pkt = 64;

        if (buf_addr > 0 && buf_addr < USBCTRL_DPRAM_SIZE) {
            /* Copy as many bytes as possible from FIFO into the buffer */
            int len = 0;
            while (len < max_pkt && usb_state.cdc_rx_count > 0 &&
                   buf_addr + len < USBCTRL_DPRAM_SIZE) {
                usb_state.dpram[buf_addr + len] = usb_state.cdc_rx_fifo[usb_state.cdc_rx_tail];
                usb_state.cdc_rx_tail = (usb_state.cdc_rx_tail + 1) % (int)sizeof(usb_state.cdc_rx_fifo);
                usb_state.cdc_rx_count--;
                len++;
            }

            buf_ctrl &= ~USB_BUF_CTRL_AVAILABLE;
            buf_ctrl |= USB_BUF_CTRL_FULL;
            buf_ctrl = (buf_ctrl & ~USB_BUF_CTRL_LEN_MASK) | len;
            dpram_write32(buf_ctrl_off, buf_ctrl);
            usb_state.buff_status |= (1u << (ep * 2 + 1));  /* EPn_OUT */
            usb_fire_irq();
        }
    }
}

/* ========================================================================
 * USB Step (called from main loop)
 * ======================================================================== */

void usb_step(void) {
    /* Only active when controller is enabled */
    if (!(usb_state.main_ctrl & USB_MAIN_CTRL_EN)) {
        return;
    }

    /* First time enabled: start enumeration */
    if (usb_state.enum_state == USB_ENUM_DISABLED) {
        usb_state.enum_state = USB_ENUM_WAIT_PULLUP;
        usb_state.sie_status |= USB_SIE_VBUS_DETECTED;
    }

    /* Advance control transfer */
    usb_ctrl_step();

    /* Advance enumeration */
    usb_enum_step();

    /* Handle CDC data when active */
    if (usb_state.enum_state == USB_ENUM_ACTIVE) {
        usb_handle_cdc();
        usb_cdc_rx_drain();
    }
}

/* ========================================================================
 * Init / Match / Read / Write
 * ======================================================================== */

void usb_init(void) {
    memset(&usb_state, 0, sizeof(usb_state_t));
    usb_state.ep_abort_done = 0xFFFFFFFF;
}

int usb_match(uint32_t addr) {
    uint32_t base = addr & ~0x3000;
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
    case USB_ADDR_ENDP:
        return usb_state.addr_endp;
    case USB_MAIN_CTRL:
        return usb_state.main_ctrl;
    case USB_SOF_RD:
        return 0;
    case USB_SIE_CTRL:
        return usb_state.sie_ctrl;
    case USB_SIE_STATUS:
        return usb_state.sie_status;
    case USB_INT_EP_CTRL:
        return usb_state.int_ep_ctrl;
    case USB_BUFF_STATUS:
        return usb_state.buff_status;
    case USB_BUFF_CPU_SHOULD_HANDLE:
        return usb_state.buff_status;
    case USB_EP_ABORT:
        return usb_state.ep_abort;
    case USB_EP_ABORT_DONE:
        return usb_state.ep_abort_done;
    case USB_EP_STALL_ARM:
        return usb_state.ep_stall_arm;
    case USB_EP_STATUS_STALL_NAK:
        return usb_state.ep_status_stall_nak;
    case USB_USB_MUXING:
        return usb_state.usb_muxing;
    case USB_USB_PWR:
        return usb_state.usb_pwr;
    case USB_INTR:
        return usb_compute_intr();
    case USB_INTE:
        return usb_state.inte;
    case USB_INTF:
        return usb_state.intf;
    case USB_INTS:
        return (usb_compute_intr() | usb_state.intf) & usb_state.inte;
    case USB_NAK_POLL:
        return 0;
    default:
        return 0;
    }
}

void usb_write32(uint32_t addr, uint32_t val) {
    uint32_t base = addr & ~0x3000;
    uint32_t alias = (addr >> 12) & 0x3;

    /* DPRAM writes (no alias for DPRAM) */
    if (base >= USBCTRL_DPRAM_BASE && base < USBCTRL_DPRAM_BASE + USBCTRL_DPRAM_SIZE) {
        uint32_t off = base - USBCTRL_DPRAM_BASE;
        if (alias == 0) {
            memcpy(&usb_state.dpram[off], &val, 4);
        } else {
            uint32_t cur;
            memcpy(&cur, &usb_state.dpram[off], 4);
            switch (alias) {
                case 1: cur ^= val; break;
                case 2: cur |= val; break;
                case 3: cur &= ~val; break;
            }
            memcpy(&usb_state.dpram[off], &cur, 4);
        }
        return;
    }

    /* Controller registers */
    uint32_t offset = base - USBCTRL_REGS_BASE;

    #define ALIAS_APPLY(reg) do { \
        switch (alias) { \
            case 0: (reg) = val; break; \
            case 1: (reg) ^= val; break; \
            case 2: (reg) |= val; break; \
            case 3: (reg) &= ~val; break; \
        } \
    } while(0)

    switch (offset) {
    case USB_ADDR_ENDP:
        ALIAS_APPLY(usb_state.addr_endp);
        break;
    case USB_MAIN_CTRL:
        ALIAS_APPLY(usb_state.main_ctrl);
        break;
    case USB_SIE_CTRL:
        ALIAS_APPLY(usb_state.sie_ctrl);
        break;
    case USB_SIE_STATUS:
        /* W1C for most bits */
        if (alias == 0 || alias == 3) {
            usb_state.sie_status &= ~val;
        } else {
            ALIAS_APPLY(usb_state.sie_status);
        }
        break;
    case USB_INT_EP_CTRL:
        ALIAS_APPLY(usb_state.int_ep_ctrl);
        break;
    case USB_BUFF_STATUS:
        /* W1C */
        if (alias == 0 || alias == 3) {
            usb_state.buff_status &= ~val;
        } else {
            ALIAS_APPLY(usb_state.buff_status);
        }
        break;
    case USB_EP_ABORT:
        ALIAS_APPLY(usb_state.ep_abort);
        break;
    case USB_EP_ABORT_DONE:
        /* W1C */
        if (alias == 0 || alias == 3) {
            usb_state.ep_abort_done &= ~val;
        } else {
            ALIAS_APPLY(usb_state.ep_abort_done);
        }
        break;
    case USB_EP_STALL_ARM:
        ALIAS_APPLY(usb_state.ep_stall_arm);
        break;
    case USB_EP_STATUS_STALL_NAK:
        /* W1C */
        if (alias == 0 || alias == 3) {
            usb_state.ep_status_stall_nak &= ~val;
        } else {
            ALIAS_APPLY(usb_state.ep_status_stall_nak);
        }
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
        break;
    }

    #undef ALIAS_APPLY
}
