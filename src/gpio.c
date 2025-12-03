#include <stdio.h>
#include <string.h>
#include "gpio.h"
#include "emulator.h"

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

    /* Interrupt registers */
    if (addr >= IO_BANK0_BASE + 0xF0 && addr < IO_BANK0_BASE + 0x180) {
        uint32_t offset = (addr - (IO_BANK0_BASE + 0xF0)) / 4;

        if (offset < 4) return gpio_state.intr[offset];
        else if (offset >= 4 && offset < 8) return gpio_state.proc0_inte[offset - 4];
        else if (offset >= 8 && offset < 12) return gpio_state.proc0_intf[offset - 8];
        else if (offset >= 12 && offset < 16) return gpio_state.proc0_ints[offset - 12];
    }

    /* PADS_BANK0 registers with alias support */
    if (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) {
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
        switch (addr) {
            case SIO_GPIO_OUT:
                gpio_state.gpio_out = val;
                break;

            case SIO_GPIO_OUT_SET:
                gpio_state.gpio_out |= val;  /* Atomic set */
                break;

            case SIO_GPIO_OUT_CLR:
                gpio_state.gpio_out &= ~val;  /* Atomic clear */
                break;

            case SIO_GPIO_OUT_XOR:
                gpio_state.gpio_out ^= val;  /* Atomic toggle */
                break;

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

    /* Interrupt registers */
    if (addr >= IO_BANK0_BASE + 0xF0 && addr < IO_BANK0_BASE + 0x180) {
        uint32_t offset = (addr - (IO_BANK0_BASE + 0xF0)) / 4;

        if (offset < 4) {
            /* INTR - write 1 to clear */
            gpio_state.intr[offset] &= ~val;
        }
        else if (offset >= 4 && offset < 8) {
            gpio_state.proc0_inte[offset - 4] = val;
        }
        else if (offset >= 8 && offset < 12) {
            gpio_state.proc0_intf[offset - 8] = val;
        }
        /* INTS is read-only */
        return;
    }

    /* ===== CRITICAL FIX: PADS_BANK0 with Alias Support ===== */
    /* Handle all 4 alias regions: 0x0000, 0x1000, 0x2000, 0x3000 */
    if (addr >= PADS_BANK0_BASE && addr < PADS_BANK0_BASE + 0x4000 + 0x80) {
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

    if (value) {
        gpio_state.gpio_out |= (1 << pin);
    } else {
        gpio_state.gpio_out &= ~(1 << pin);
    }
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
