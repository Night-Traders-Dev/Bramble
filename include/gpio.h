#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

/* GPIO Base Addresses (RP2040) */
#define IO_BANK0_BASE       0x40014000  /* GPIO 0-29 control */
#define PADS_BANK0_BASE     0x4001C000  /* GPIO pad controls */
#define SIO_BASE_GPIO       0xD0000000  /* SIO for direct GPIO access */

/* SIO GPIO Registers (fast GPIO access) */
#define SIO_GPIO_IN         (SIO_BASE_GPIO + 0x004)  /* GPIO input values */
#define SIO_GPIO_OUT        (SIO_BASE_GPIO + 0x010)  /* GPIO output values */
#define SIO_GPIO_OUT_SET    (SIO_BASE_GPIO + 0x014)  /* Atomic bit set */
#define SIO_GPIO_OUT_CLR    (SIO_BASE_GPIO + 0x018)  /* Atomic bit clear */
#define SIO_GPIO_OUT_XOR    (SIO_BASE_GPIO + 0x01C)  /* Atomic bit toggle */
#define SIO_GPIO_OE         (SIO_BASE_GPIO + 0x020)  /* Output enable */
#define SIO_GPIO_OE_SET     (SIO_BASE_GPIO + 0x024)  /* Atomic OE set */
#define SIO_GPIO_OE_CLR     (SIO_BASE_GPIO + 0x028)  /* Atomic OE clear */
#define SIO_GPIO_OE_XOR     (SIO_BASE_GPIO + 0x02C)  /* Atomic OE toggle */

/* IO_BANK0 Registers (per-pin configuration) */
#define GPIO_STATUS_OFFSET  0x000  /* GPIO status register */
#define GPIO_CTRL_OFFSET    0x004  /* GPIO control register */

/* Number of GPIO pins on RP2040 */
#define NUM_GPIO_PINS       30

/* GPIO Function Select Values */
#define GPIO_FUNC_XIP       0
#define GPIO_FUNC_SPI       1
#define GPIO_FUNC_UART      2
#define GPIO_FUNC_I2C       3
#define GPIO_FUNC_PWM       4
#define GPIO_FUNC_SIO       5  /* Software controlled I/O */
#define GPIO_FUNC_PIO0      6
#define GPIO_FUNC_PIO1      7
#define GPIO_FUNC_CLOCK     8
#define GPIO_FUNC_USB       9
#define GPIO_FUNC_NULL      0x1F

/* GPIO Interrupt Types */
#define GPIO_INTR_LEVEL_LOW   0x1
#define GPIO_INTR_LEVEL_HIGH  0x2
#define GPIO_INTR_EDGE_LOW    0x4
#define GPIO_INTR_EDGE_HIGH   0x8

/* GPIO State Structure */
typedef struct {
    /* SIO registers (fast access) */
    uint32_t gpio_in;        /* Current input values */
    uint32_t gpio_out;       /* Output values */
    uint32_t gpio_oe;        /* Output enable mask */
    
    /* Per-pin configuration (IO_BANK0) */
    struct {
        uint32_t status;     /* GPIO status register */
        uint32_t ctrl;       /* GPIO control register */
    } pins[NUM_GPIO_PINS];
    
    /* Interrupt registers */
    uint32_t intr[4];        /* Raw interrupt status (8 pins per register) */
    uint32_t proc0_inte[4];  /* Interrupt enable for processor 0 */
    uint32_t proc0_intf[4];  /* Interrupt force for processor 0 */
    uint32_t proc0_ints[4];  /* Interrupt status for processor 0 */
    
    /* Pad control registers */
    uint32_t pads[NUM_GPIO_PINS];
} gpio_state_t;

/* GPIO Functions */
void gpio_init(void);
void gpio_reset(void);

uint32_t gpio_read32(uint32_t addr);
void gpio_write32(uint32_t addr, uint32_t val);

/* GPIO pin operations */
void gpio_set_pin(uint8_t pin, uint8_t value);
uint8_t gpio_get_pin(uint8_t pin);
void gpio_set_direction(uint8_t pin, uint8_t output);
void gpio_set_function(uint8_t pin, uint8_t func);

/* External state */
extern gpio_state_t gpio_state;

#endif /* GPIO_H */
