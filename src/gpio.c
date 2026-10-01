#include <stdio.h>
#include <string.h>
#include "gpio.h"
#include "emulator.h"
#include "nvic.h"
#include "devtools.h"

/* Helper: trace GPIO changes via VCD when gpio_out is modified */
static inline void gpio_trace_changes(uint32_t old_val, uint32_t new_val) {
    if (__builtin_expect(!gpio_trace_enabled, 1)) return;
    uint32_t changed = old_val ^ new_val;
    while (changed) {
        int pin = __builtin_ctz(changed);
        gpio_trace_record((uint8_t)pin, (new_val >> pin) & 1);
        changed &= changed - 1;
    }
}

/* GPIO state */
gpio_state_t gpio_state;

/* Which chip's GPIO map is in effect *for the address we are being asked
 * about*. This is not simply membus_rp2350_mode: the Hazard3 membus rewrites
 * RP2350 GPIO bases back to their RP2040 equivalents (rv_translate_shared_addr)
 * before delegating, so on that path the shared bus is handed RP2040 addresses
 * and must decode them with the RP2040 layout. membus_rv_delegate is set for
 * exactly those accesses. */
static int gpio_rp2350_layout(void) {
    return membus_rp2350_mode && !membus_rv_delegate;
}

/* The emulated chip's GPIO blocks. RP2350 relocates IO_BANK0 and PADS_BANK0 and
 * those addresses hold entirely different peripherals on RP2040, so every match
 * and decode site asks which chip it is on rather than using one constant. */
uint32_t gpio_io_bank0_base(void) {
    return gpio_rp2350_layout() ? RP2350_IO_BANK0_BASE : IO_BANK0_BASE;
}

uint32_t gpio_pads_bank0_base(void) {
    return gpio_rp2350_layout() ? RP2350_PADS_BANK0_BASE : PADS_BANK0_BASE;
}

int gpio_num_user_pins(void) {
    return gpio_rp2350_layout() ? NUM_GPIO_PINS : NUM_GPIO_PINS_RP2040;
}

/* Offset of the interrupt-register window within IO_BANK0. RP2040: INTR0 at
 * +0x0F0, right after GPIO29_CTRL at +0x0EC. RP2350: 48 pins, then
 * PROC0_IRQSUMMARY at +0x200 and INTR0 at +0x230. */
static uint32_t gpio_irq_window(void) {
    return gpio_rp2350_layout() ? GPIO_IRQ_WINDOW_RP2350 : GPIO_IRQ_WINDOW_RP2040;
}

static uint32_t gpio_io_bank0_span(void) {
    return gpio_rp2350_layout() ? GPIO_IO_BANK0_SPAN_RP2350 : GPIO_IO_BANK0_SPAN_RP2040;
}

/* Interrupt banks needed for the emulated chip. */
static int gpio_irq_banks(void) {
    int b = (gpio_num_user_pins() + 7) / 8;
    return b > GPIO_IRQ_BANKS ? GPIO_IRQ_BANKS : b;
}

/* PADS_BANK0 covers the user pads plus SWCLK and SWD. */
static uint32_t gpio_pads_size(void) {
    return ((uint32_t)gpio_num_user_pins() + 2) * 4;
}

/* Which atomic-alias region of PADS_BANK0 an address is in (0 for a plain
 * access). RP2350 relocates the block to 0x40038000 and carries 48 pads plus
 * SWCLK/SWD (0xCC bytes); RP2040 has 30 pads plus SWCLK/SWD (0x84) at
 * 0x4001C000. A fixed 0x80 window at the RP2040 base left the entire RP2350 pad
 * block unmapped -- so every gpio_init()/gpio_set_function() pad write was
 * dropped on that chip -- and hid the SWD pad even on RP2040. */
static uint32_t gpio_pads_alias(uint32_t addr) {
    uint32_t pbase = gpio_pads_bank0_base();
    uint32_t psize = gpio_pads_size();
    if (addr >= pbase + REG_ALIAS_CLR_BITS && addr < pbase + REG_ALIAS_CLR_BITS + psize)
        return REG_ALIAS_CLR_BITS;
    if (addr >= pbase + REG_ALIAS_SET_BITS && addr < pbase + REG_ALIAS_SET_BITS + psize)
        return REG_ALIAS_SET_BITS;
    if (addr >= pbase + REG_ALIAS_XOR_BITS && addr < pbase + REG_ALIAS_XOR_BITS + psize)
        return REG_ALIAS_XOR_BITS;
    return REG_ALIAS_RW_BITS;
}

static int gpio_pads_contains(uint32_t addr) {
    uint32_t off = addr - gpio_pads_bank0_base() - gpio_pads_alias(addr);
    return off < gpio_pads_size();
}

/* Initialize GPIO subsystem */
void gpio_init(void) {
    gpio_reset();
}

/* Reset GPIO to power-on defaults */
void gpio_reset(void) {
    memset(&gpio_state, 0, sizeof(gpio_state_t));

    /* Datasheet reset values: GPIOx_CTRL.FUNCSEL resets to 0x1f (NULL), not SIO,
     * so pins must come out of reset de-selected. PADS reset is 0x56 for the user
     * bank (IE=1, OD=0, PUE=0, PDE=1, SCHMITT=1) and 0x96 for SWCLK/SWD, which
     * additionally have PUE=1. */
    int user = gpio_num_user_pins();
    for (int i = 0; i < NUM_GPIO_PINS; i++) {
        if (i < user) {
            gpio_state.pins[i].ctrl = GPIO_FUNC_NULL;
            gpio_state.pads[i] = 0x00000056;
        }
    }
    gpio_state.pads[user + 0] = 0x00000096;  /* SWCLK */
    gpio_state.pads[user + 1] = 0x00000096;  /* SWD */

    /* All pins start as inputs (OE=0) */
    gpio_state.gpio_oe = 0x00000000;
    gpio_state.gpio_out = 0x00000000;
    gpio_state.gpio_in = 0x00000000;
}

/* Recompute INTS and signal NVIC if any interrupt is active */
static void gpio_check_irq(void) {
    uint32_t any_active = 0;
    int banks = gpio_irq_banks();
    for (int i = 0; i < banks; i++) {
        gpio_state.proc0_ints[i] = (gpio_state.intr[i] | gpio_state.proc0_intf[i])
                                    & gpio_state.proc0_inte[i];
        any_active |= gpio_state.proc0_ints[i];
    }
    if (any_active) {
        nvic_signal_irq(IRQ_IO_IRQ_BANK0);
    }
}

/*
 * Detect GPIO edge/level events by comparing old and new pin values.
 * Sets INTR bits: per pin 4 bits = [edge_high, edge_low, level_high, level_low]
 * Level interrupts are continuously asserted while the pin is at that level.
 * Edge interrupts are latched (W1C) when a transition occurs.
 */
static void gpio_detect_events(uint32_t old_pins, uint32_t new_pins) {
    /* Recompute level interrupts from current pin state. The bank count follows
     * the emulated chip: a hardcoded 4 banks left RP2350's pins 32-47 unable to
     * set interrupt state at all. */
    int banks = gpio_irq_banks();
    for (int reg = 0; reg < banks; reg++) {
        uint32_t level_bits = 0;
        for (int bit = 0; bit < 8; bit++) {
            int pin = reg * 8 + bit;
            if (pin >= gpio_num_user_pins()) break;
            int val = (new_pins >> pin) & 1;
            uint32_t shift = bit * 4;
            /* Level low (bit 0): pin is 0 */
            if (!val) level_bits |= (GPIO_INTR_LEVEL_LOW << shift);
            /* Level high (bit 1): pin is 1 */
            if (val)  level_bits |= (GPIO_INTR_LEVEL_HIGH << shift);
        }
        /* Level bits are not latched — recompute every time.
         * Merge with existing edge bits (which are latched/W1C). */
        uint32_t edge_mask = 0;
        for (int bit = 0; bit < 8; bit++) {
            uint32_t shift = bit * 4;
            edge_mask |= ((uint32_t)(GPIO_INTR_EDGE_LOW | GPIO_INTR_EDGE_HIGH) << shift);
        }
        gpio_state.intr[reg] = (gpio_state.intr[reg] & edge_mask) | level_bits;
    }

    /* Detect edges from changed pins */
    uint32_t changed = old_pins ^ new_pins;
    if (changed) {
        for (int pin = 0; pin < gpio_num_user_pins(); pin++) {
            /* pins >= 32 live in the separate GPIO_HI bank and are not part of
             * this 32-bit word; shifting 1u by >= 32 is undefined. */
            if (pin >= 32) break;
            if (!(changed & (1u << pin))) continue;
            int reg = pin / 8;
            int bit = pin % 8;
            if (reg >= gpio_irq_banks()) break;
            uint32_t shift = bit * 4;
            int new_val = (new_pins >> pin) & 1;
            if (new_val) {
                /* Rising edge */
                gpio_state.intr[reg] |= (GPIO_INTR_EDGE_HIGH << shift);
            } else {
                /* Falling edge */
                gpio_state.intr[reg] |= (GPIO_INTR_EDGE_LOW << shift);
            }
        }
    }

    gpio_check_irq();
}

/* Compute effective pin values (what SIO_GPIO_IN would return) */
static uint32_t gpio_effective_pins(void) {
    return (gpio_state.gpio_out & gpio_state.gpio_oe) |
           (gpio_state.gpio_in & ~gpio_state.gpio_oe);
}

/* Read from GPIO register space */
uint32_t gpio_read32(uint32_t addr) {
    /* SIO GPIO registers (fast access) */
    if (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100) {
        switch (addr) {
            case SIO_GPIO_IN:
                /* Return current input values */
                /* For pins configured as outputs, return the output value */
                /* For inputs, return the gpio_in value */
                return (gpio_state.gpio_out & gpio_state.gpio_oe) |
                       (gpio_state.gpio_in & ~gpio_state.gpio_oe);

            case SIO_GPIO_HI_IN:
                /* QSPI GPIO input: 6 pins (SCLK=0, SS=1, SD0-3=2-5) */
                /* Default: CS(SS) high, data lines high (pulled up) */
                return 0x3E;  /* bits 1-5 set: SS + SD0-3 high */

            case SIO_GPIO_OUT:
                return gpio_state.gpio_out;

            case SIO_GPIO_OE:
                return gpio_state.gpio_oe;

            default:
                return 0x00000000;
        }
    }

    /* IO_BANK0 registers (per-pin configuration). The window must stop where the
     * per-pin registers stop. On RP2040 GPIO29_CTRL is last at +0x0EC and INTR0
     * begins at +0x0F0; claiming a fixed 0x200 span returned phantom GPIO30+
     * registers here and made the interrupt decoder below unreachable for plain
     * (non-alias) accesses. */
    uint32_t iobase_r = gpio_io_bank0_base();
    uint32_t pin_win_r = (uint32_t)gpio_num_user_pins() * 8;
    if (addr >= iobase_r && addr < iobase_r + pin_win_r) {
        uint32_t offset = addr - iobase_r;
        uint32_t pin = offset / 8;  /* Each pin has 8 bytes (STATUS + CTRL) */
        uint32_t reg = offset % 8;

        if (pin < (uint32_t)gpio_num_user_pins()) {
            if (reg == GPIO_STATUS_OFFSET) {
                return gpio_state.pins[pin].status;
            } else if (reg == GPIO_CTRL_OFFSET) {
                return gpio_state.pins[pin].ctrl;
            }
        }
    }

    /* IO_BANK0 interrupt registers (all aliases read the underlying register) */
    {
        uint32_t base_addr = addr;
        if (addr >= IO_BANK0_BASE + REG_ALIAS_CLR_BITS && addr < IO_BANK0_BASE + REG_ALIAS_CLR_BITS + 0x200)
            base_addr -= REG_ALIAS_CLR_BITS;
        else if (addr >= IO_BANK0_BASE + REG_ALIAS_SET_BITS && addr < IO_BANK0_BASE + REG_ALIAS_SET_BITS + 0x200)
            base_addr -= REG_ALIAS_SET_BITS;
        else if (addr >= IO_BANK0_BASE + REG_ALIAS_XOR_BITS && addr < IO_BANK0_BASE + REG_ALIAS_XOR_BITS + 0x200)
            base_addr -= REG_ALIAS_XOR_BITS;

        uint32_t iw_r = gpio_irq_window();
        if (base_addr >= iobase_r + iw_r && base_addr < iobase_r + gpio_io_bank0_span()) {
            uint32_t offset = (base_addr - (iobase_r + iw_r)) / 4;
            /* The window opens at INTR0 on both chips (RP2040 +0x0F0,
             * RP2350 +0x230), so no offset shift is needed. The four register
             * groups are each `banks` words wide: 4 banks on RP2040, 6 on
             * RP2350 (48 pins). A hardcoded 4 read the wrong registers on
             * RP2350 from PROC0_INTE3 onward. */
            int banks = (int)gpio_irq_banks();
            if (offset < (uint32_t)banks)                      return gpio_state.intr[offset];
            if (offset < (uint32_t)banks * 2)  return gpio_state.proc0_inte[offset - banks];
            if (offset < (uint32_t)banks * 3)  return gpio_state.proc0_intf[offset - banks * 2];
            if (offset < (uint32_t)banks * 4)  return gpio_state.proc0_ints[offset - banks * 3];
        }
    }

    /* PADS_BANK0 registers with alias support */
    if (gpio_pads_contains(addr)) {
        /* Strip alias offset to get base address */
        uint32_t alias_bits = gpio_pads_alias(addr);
        uint32_t base_addr = addr - alias_bits;

        uint32_t offset = (base_addr - gpio_pads_bank0_base()) / 4;
        
        if (offset > 0 && offset <= (uint32_t)gpio_num_user_pins() + 2) {
            return gpio_state.pads[offset - 1];
        }
        /* Voltage select and other pad registers */
        return 0x00000056;  /* Default pad config */
    }

    return 0x00000000;
}

/* Write to GPIO register space */
void gpio_write32(uint32_t addr, uint32_t val) {
    /* SIO GPIO registers (fast access with atomic operations) */
    if (addr >= SIO_BASE_GPIO && addr < SIO_BASE_GPIO + 0x100) {
        uint32_t old_pins = gpio_effective_pins();
        switch (addr) {
            case SIO_GPIO_OUT: {
                uint32_t old = gpio_state.gpio_out;
                gpio_state.gpio_out = val;
                gpio_trace_changes(old, val);
                break;
            }
            case SIO_GPIO_OUT_SET: {
                uint32_t old = gpio_state.gpio_out;
                gpio_state.gpio_out |= val;
                gpio_trace_changes(old, gpio_state.gpio_out);
                break;
            }
            case SIO_GPIO_OUT_CLR: {
                uint32_t old = gpio_state.gpio_out;
                gpio_state.gpio_out &= ~val;
                gpio_trace_changes(old, gpio_state.gpio_out);
                break;
            }
            case SIO_GPIO_OUT_XOR: {
                uint32_t old = gpio_state.gpio_out;
                gpio_state.gpio_out ^= val;
                gpio_trace_changes(old, gpio_state.gpio_out);
                break;
            }

            case SIO_GPIO_OE:
                gpio_state.gpio_oe = val;
                break;

            case SIO_GPIO_OE_SET:
                gpio_state.gpio_oe |= val;  /* Atomic set */
                break;

            case SIO_GPIO_OE_CLR:
                gpio_state.gpio_oe &= ~val;  /* Atomic clear */
                break;

            case SIO_GPIO_OE_XOR:
                gpio_state.gpio_oe ^= val;  /* Atomic toggle */
                break;

            /* GPIO_IN is read-only, writes ignored */
            case SIO_GPIO_IN:
                break;
        }
        /* Detect edge/level events from pin value changes */
        gpio_detect_events(old_pins, gpio_effective_pins());
        return;
    }

    /* IO_BANK0 registers (per-pin configuration) — see the read path for why
     * this window is per-pin-limited rather than a fixed 0x200. */
    uint32_t iobase_w = gpio_io_bank0_base();
    uint32_t pin_win_w = (uint32_t)gpio_num_user_pins() * 8;
    if (addr >= iobase_w && addr < iobase_w + pin_win_w) {
        uint32_t offset = addr - iobase_w;
        uint32_t pin = offset / 8;
        uint32_t reg = offset % 8;

        if (pin < (uint32_t)gpio_num_user_pins()) {
            if (reg == GPIO_STATUS_OFFSET) {
                /* STATUS register - mostly read-only, but some bits writable */
                /* For now, treat as mostly read-only */
                gpio_state.pins[pin].status = val;
            } else if (reg == GPIO_CTRL_OFFSET) {
                /* CTRL register - function select and other config */
                gpio_state.pins[pin].ctrl = val & 0x1F;  /* Only lower 5 bits for function */
            }
        }
        return;
    }

    /* IO_BANK0 interrupt registers with atomic alias support.
     * Aliases at IO_BANK0_BASE + 0x1000 (XOR), +0x2000 (SET), +0x3000 (CLR).
     * Firmware uses hw_set_bits/hw_clear_bits (SET/CLR aliases) to enable/disable
     * GPIO interrupts, so we must handle all four alias regions. */
    {
        uint32_t irq_alias = REG_ALIAS_RW_BITS;
        uint32_t base_addr = addr;
        if (addr >= IO_BANK0_BASE + REG_ALIAS_CLR_BITS &&
            addr <  IO_BANK0_BASE + REG_ALIAS_CLR_BITS + 0x200) {
            irq_alias = REG_ALIAS_CLR_BITS;
            base_addr -= REG_ALIAS_CLR_BITS;
        } else if (addr >= IO_BANK0_BASE + REG_ALIAS_SET_BITS &&
                   addr <  IO_BANK0_BASE + REG_ALIAS_SET_BITS + 0x200) {
            irq_alias = REG_ALIAS_SET_BITS;
            base_addr -= REG_ALIAS_SET_BITS;
        } else if (addr >= IO_BANK0_BASE + REG_ALIAS_XOR_BITS &&
                   addr <  IO_BANK0_BASE + REG_ALIAS_XOR_BITS + 0x200) {
            irq_alias = REG_ALIAS_XOR_BITS;
            base_addr -= REG_ALIAS_XOR_BITS;
        }

        uint32_t iw_w = gpio_irq_window();
        if (base_addr >= iobase_w + iw_w && base_addr < iobase_w + gpio_io_bank0_span()) {
            uint32_t offset = (base_addr - (iobase_w + iw_w)) / 4;
            uint32_t *reg_ptr = NULL;

            int banks = (int)gpio_irq_banks();
            if (offset < (uint32_t)banks) {
                /* INTR - write-1-to-clear regardless of alias */
                gpio_state.intr[offset] &= ~val;
            } else if (offset < (uint32_t)banks * 2) {
                reg_ptr = &gpio_state.proc0_inte[offset - banks];
            } else if (offset < (uint32_t)banks * 3) {
                reg_ptr = &gpio_state.proc0_intf[offset - banks * 2];
            }
            /* INTS is read-only */

            if (reg_ptr) {
                switch (irq_alias) {
                case REG_ALIAS_SET_BITS: *reg_ptr |= val;  break;
                case REG_ALIAS_CLR_BITS: *reg_ptr &= ~val; break;
                case REG_ALIAS_XOR_BITS: *reg_ptr ^= val;  break;
                default:                 *reg_ptr  = val;  break;
                }
            }
            gpio_check_irq();
            return;
        }
    }

    /* ===== CRITICAL FIX: PADS_BANK0 with Alias Support ===== */
    /* Handle all 4 alias regions: 0x0000, 0x1000, 0x2000, 0x3000 */
    if (gpio_pads_contains(addr)) {
        /* Determine which alias region we're in */
        uint32_t alias_offset = gpio_pads_alias(addr);  /* 0 = normal access */
        uint32_t base_addr = addr - alias_offset;

        uint32_t offset = (base_addr - gpio_pads_bank0_base()) / 4;
        
        if (offset > 0 && offset <= (uint32_t)gpio_num_user_pins() + 2) {
            uint32_t pin_idx = offset - 1;
            
            /* Apply atomic operation based on alias */
            switch (alias_offset) {
                case REG_ALIAS_RW_BITS:  /* 0x0000 - Normal write */
                    gpio_state.pads[pin_idx] = val;
                    break;
                    
                case REG_ALIAS_XOR_BITS:  /* 0x1000 - XOR */
                    gpio_state.pads[pin_idx] ^= val;
                    break;
                    
                case REG_ALIAS_SET_BITS:  /* 0x2000 - SET */
                    gpio_state.pads[pin_idx] |= val;  /* Set bits where val=1 */
                    break;
                    
                case REG_ALIAS_CLR_BITS:  /* 0x3000 - CLEAR */
                    gpio_state.pads[pin_idx] &= ~val;  /* Clear bits where val=1 */
                    break;
            }
        } else if (offset == 0) {
            /* Voltage select register - stub for now */
        }
        return;
    }
}

/* ========================================================================
 * GPIO_HI (pins 32-47, plus the QSPI/USB IOs)
 *
 * On RP2350 the high bank's output and output-enable registers are interleaved
 * with the low bank's in SIO space (datasheet Table 17):
 *   0x010 GPIO_OUT   0x014 GPIO_HI_OUT   0x018 GPIO_OUT_SET  0x01c GPIO_HI_OUT_SET
 *   0x020 GPIO_OUT_CLR 0x024 GPIO_HI_OUT_CLR 0x028 GPIO_OUT_XOR 0x02c GPIO_HI_OUT_XOR
 *   0x030 GPIO_OE    0x034 GPIO_HI_OE    0x038 GPIO_OE_SET   0x03c GPIO_HI_OE_SET
 *   0x040 GPIO_OE_CLR 0x044 GPIO_HI_OE_CLR 0x048 GPIO_OE_XOR  0x04c GPIO_HI_OE_XOR
 * so the high bank needs its own state rather than living in the 32-bit words.
 * ======================================================================== */

uint32_t gpio_hi_out = 0;
uint32_t gpio_hi_oe = 0;

void gpio_hi_write32(uint32_t offset, uint32_t val) {
    uint16_t v = (uint16_t)(val & 0xFFFF);
    switch (offset) {
    case SIO_GPIO_HI_OUT:       gpio_hi_out = v; return;
    case SIO_GPIO_HI_OUT_SET:  gpio_hi_out |= v; return;
    case SIO_GPIO_HI_OUT_CLR:  gpio_hi_out &= (uint16_t)~v; return;
    case SIO_GPIO_HI_OUT_XOR:  gpio_hi_out ^= v; return;
    case SIO_GPIO_HI_OE:       gpio_hi_oe = v; return;
    case SIO_GPIO_HI_OE_SET:   gpio_hi_oe |= v; return;
    case SIO_GPIO_HI_OE_CLR:   gpio_hi_oe &= (uint16_t)~v; return;
    case SIO_GPIO_HI_OE_XOR:   gpio_hi_oe ^= v; return;
    default: break;
    }
}

uint32_t gpio_hi_read32(uint32_t offset) {
    switch (offset) {
    case SIO_GPIO_HI_OUT:
    case SIO_GPIO_HI_OUT_SET:
    case SIO_GPIO_HI_OUT_CLR:
    case SIO_GPIO_HI_OUT_XOR:  return gpio_hi_out;
    case SIO_GPIO_HI_OE:
    case SIO_GPIO_HI_OE_SET:
    case SIO_GPIO_HI_OE_CLR:
    case SIO_GPIO_HI_OE_XOR:  return gpio_hi_oe;
    default: return 0;
    }
}

/* Is this SIO offset one of the RP2350 GPIO_HI registers? */
int gpio_hi_offset(uint32_t offset) {
    switch (offset) {
    case SIO_GPIO_HI_OUT: case SIO_GPIO_HI_OUT_SET:
    case SIO_GPIO_HI_OUT_CLR: case SIO_GPIO_HI_OUT_XOR:
    case SIO_GPIO_HI_OE: case SIO_GPIO_HI_OE_SET:
    case SIO_GPIO_HI_OE_CLR: case SIO_GPIO_HI_OE_XOR:
        return 1;
    default: return 0;
    }
}

/* Helper functions for GPIO pin operations */

void gpio_set_pin(uint8_t pin, uint8_t value) {
    if (pin >= NUM_GPIO_PINS) return;

    uint32_t old_pins = gpio_effective_pins();
    if (value) {
        gpio_state.gpio_out |= (1u << pin);
    } else {
        gpio_state.gpio_out &= ~(1u << pin);
    }
    gpio_detect_events(old_pins, gpio_effective_pins());
}

uint8_t gpio_get_pin(uint8_t pin) {
    if (pin >= NUM_GPIO_PINS) return 0;

    /* If pin is output, return output value */
    if (gpio_state.gpio_oe & (1 << pin)) {
        return (gpio_state.gpio_out >> pin) & 1;
    }
    /* Otherwise return input value */
    return (gpio_state.gpio_in >> pin) & 1;
}

void gpio_set_input_pin(uint8_t pin, uint8_t value) {
    if (pin >= NUM_GPIO_PINS) return;

    uint32_t old_pins = gpio_effective_pins();
    if (value) {
        gpio_state.gpio_in |= (1u << pin);
    } else {
        gpio_state.gpio_in &= ~(1u << pin);
    }
    gpio_detect_events(old_pins, gpio_effective_pins());
}

void gpio_set_direction(uint8_t pin, uint8_t output) {
    if (pin >= NUM_GPIO_PINS) return;

    if (output) {
        gpio_state.gpio_oe |= (1 << pin);
    } else {
        gpio_state.gpio_oe &= ~(1 << pin);
    }
}

void gpio_set_function(uint8_t pin, uint8_t func) {
    if (pin >= NUM_GPIO_PINS) return;

    gpio_state.pins[pin].ctrl = (gpio_state.pins[pin].ctrl & ~0x1F) | (func & 0x1F);
}
