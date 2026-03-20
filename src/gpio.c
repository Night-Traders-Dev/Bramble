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

/* Initialize GPIO subsystem */
void gpio_init(void) {
    gpio_reset();
}

/* Reset GPIO to power-on defaults */
void gpio_reset(void) {
    memset(&gpio_state, 0, sizeof(gpio_state_t));

    /* Default: all pins as inputs with SIO function */
    for (int i = 0; i < NUM_GPIO_PINS; i++) {
        gpio_state.pins[i].ctrl = GPIO_FUNC_SIO;  /* Default to SIO function */
        gpio_state.pads[i] = 0x00000056;  /* Default pad config: IE=1, OD=0, PUE=1, PDE=1 */
    }

    /* All pins start as inputs (OE=0) */
    gpio_state.gpio_oe = 0x00000000;
    gpio_state.gpio_out = 0x00000000;
    gpio_state.gpio_in = 0x00000000;
}

/* Recompute INTS and signal NVIC if any interrupt is active */
static void gpio_check_irq(void) {
    uint32_t any_active = 0;
    for (int i = 0; i < 4; i++) {
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
    /* Recompute level interrupts from current pin state */
    for (int reg = 0; reg < 4; reg++) {
        uint32_t level_bits = 0;
        for (int bit = 0; bit < 8; bit++) {
            int pin = reg * 8 + bit;
            if (pin >= NUM_GPIO_PINS) break;
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
            edge_mask |= ((GPIO_INTR_EDGE_LOW | GPIO_INTR_EDGE_HIGH) << shift);
        }
        gpio_state.intr[reg] = (gpio_state.intr[reg] & edge_mask) | level_bits;
    }

    /* Detect edges from changed pins */
    uint32_t changed = old_pins ^ new_pins;
    if (changed) {
        for (int pin = 0; pin < NUM_GPIO_PINS; pin++) {
            if (!(changed & (1u << pin))) continue;
            int reg = pin / 8;
            int bit = pin % 8;
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

    /* IO_BANK0 registers (per-pin configuration) */
    if (addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) {
        uint32_t offset = addr - IO_BANK0_BASE;
        uint32_t pin = offset / 8;  /* Each pin has 8 bytes (STATUS + CTRL) */
        uint32_t reg = offset % 8;

        if (pin < NUM_GPIO_PINS) {
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

        if (base_addr >= IO_BANK0_BASE + 0xF0 && base_addr < IO_BANK0_BASE + 0x180) {
            uint32_t offset = (base_addr - (IO_BANK0_BASE + 0xF0)) / 4;
            if (offset < 4)                  return gpio_state.intr[offset];
            else if (offset < 8)             return gpio_state.proc0_inte[offset - 4];
            else if (offset < 12)            return gpio_state.proc0_intf[offset - 8];
            else if (offset < 16)            return gpio_state.proc0_ints[offset - 12];
        }
    }

    /* PADS_BANK0 registers with alias support */
    if ((addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x80) ||
        (addr >= PADS_BANK0_BASE + REG_ALIAS_XOR_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_XOR_BITS + 0x80) ||
        (addr >= PADS_BANK0_BASE + REG_ALIAS_SET_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_SET_BITS + 0x80) ||
        (addr >= PADS_BANK0_BASE + REG_ALIAS_CLR_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_CLR_BITS + 0x80)) {
        /* Strip alias offset to get base address */
        uint32_t base_addr = addr;
        
        if (addr >= PADS_BANK0_BASE + REG_ALIAS_CLR_BITS) {
            base_addr = addr - REG_ALIAS_CLR_BITS;
        } else if (addr >= PADS_BANK0_BASE + REG_ALIAS_SET_BITS) {
            base_addr = addr - REG_ALIAS_SET_BITS;
        } else if (addr >= PADS_BANK0_BASE + REG_ALIAS_XOR_BITS) {
            base_addr = addr - REG_ALIAS_XOR_BITS;
        }

        uint32_t offset = (base_addr - PADS_BANK0_BASE) / 4;
        
        if (offset > 0 && offset <= NUM_GPIO_PINS) {
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

    /* IO_BANK0 registers (per-pin configuration) */
    if (addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) {
        uint32_t offset = addr - IO_BANK0_BASE;
        uint32_t pin = offset / 8;
        uint32_t reg = offset % 8;

        if (pin < NUM_GPIO_PINS) {
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

        if (base_addr >= IO_BANK0_BASE + 0xF0 && base_addr < IO_BANK0_BASE + 0x180) {
            uint32_t offset = (base_addr - (IO_BANK0_BASE + 0xF0)) / 4;
            uint32_t *reg_ptr = NULL;

            if (offset < 4) {
                /* INTR - W1C regardless of alias */
                gpio_state.intr[offset] &= ~val;
            } else if (offset >= 4 && offset < 8) {
                reg_ptr = &gpio_state.proc0_inte[offset - 4];
            } else if (offset >= 8 && offset < 12) {
                reg_ptr = &gpio_state.proc0_intf[offset - 8];
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
    if ((addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x80) ||
        (addr >= PADS_BANK0_BASE + REG_ALIAS_XOR_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_XOR_BITS + 0x80) ||
        (addr >= PADS_BANK0_BASE + REG_ALIAS_SET_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_SET_BITS + 0x80) ||
        (addr >= PADS_BANK0_BASE + REG_ALIAS_CLR_BITS && addr < PADS_BANK0_BASE + REG_ALIAS_CLR_BITS + 0x80)) {
        /* Determine which alias region we're in */
        uint32_t alias_offset = REG_ALIAS_RW_BITS;  /* Default to normal access */
        uint32_t base_addr = addr;
        
        if (addr >= PADS_BANK0_BASE + REG_ALIAS_CLR_BITS) {
            alias_offset = REG_ALIAS_CLR_BITS;  /* 0x3000 - CLEAR */
            base_addr = addr - REG_ALIAS_CLR_BITS;
        } else if (addr >= PADS_BANK0_BASE + REG_ALIAS_SET_BITS) {
            alias_offset = REG_ALIAS_SET_BITS;  /* 0x2000 - SET */
            base_addr = addr - REG_ALIAS_SET_BITS;
        } else if (addr >= PADS_BANK0_BASE + REG_ALIAS_XOR_BITS) {
            alias_offset = REG_ALIAS_XOR_BITS;  /* 0x1000 - XOR */
            base_addr = addr - REG_ALIAS_XOR_BITS;
        }

        uint32_t offset = (base_addr - PADS_BANK0_BASE) / 4;
        
        if (offset > 0 && offset <= NUM_GPIO_PINS) {
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
