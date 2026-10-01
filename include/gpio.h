#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

/* GPIO Base Addresses. RP2350 relocates both blocks, and the two maps overlap
 * with other peripherals, so these must never be used interchangeably. */
#define IO_BANK0_BASE          0x40014000  /* RP2040 IO_BANK0 */
#define PADS_BANK0_BASE        0x4001C000  /* RP2040 PADS_BANK0 */
#define RP2350_IO_BANK0_BASE   0x40028000  /* RP2350 IO_BANK0 */
#define RP2350_PADS_BANK0_BASE 0x40038000  /* RP2350 PADS_BANK0 */
#define SIO_BASE_GPIO          0xD0000000  /* SIO for direct GPIO access */

/* Active IO_BANK0 / PADS_BANK0 base for the emulated chip. */
uint32_t gpio_io_bank0_base(void);
uint32_t gpio_pads_bank0_base(void);

/* SIO GPIO register offsets. RP2350 interleaves the GPIO_HI bank with the low
 * bank (datasheet Table 17). */
#define SIO_GPIO_HI_OUT                0x14
#define SIO_GPIO_HI_OUT_SET            0x1C
#define SIO_GPIO_HI_OUT_CLR            0x24
#define SIO_GPIO_HI_OUT_XOR            0x2C
#define SIO_GPIO_HI_OE                 0x34
#define SIO_GPIO_HI_OE_SET             0x3C
#define SIO_GPIO_HI_OE_CLR             0x44
#define SIO_GPIO_HI_OE_XOR             0x4C

/* GPIO_HI bank (RP2350 pins 32-47 plus QSPI/USB IOs). */
extern uint32_t gpio_hi_out;
extern uint32_t gpio_hi_oe;
void gpio_hi_write32(uint32_t offset, uint32_t val);
uint32_t gpio_hi_read32(uint32_t offset);
int gpio_hi_offset(uint32_t offset);

/* Number of GPIO user pins on the emulated chip (30 on RP2040, 48 on RP2350). */
int gpio_num_user_pins(void);

/* Interrupt-register window and per-pin register window, per chip. */
#define GPIO_IO_BANK0_PIN_WINDOW_RP2040  0x200u  /* GPIOx_STATUS/CTRL up to +0x1FF */
#define GPIO_IO_BANK0_PIN_WINDOW_RP2350   0x200u
#define GPIO_IRQ_WINDOW_RP2040           0x0F0u  /* INTR0 .. DORMANT_WAKE_INTS */
#define GPIO_IRQ_WINDOW_RP2350           0x230u  /* IRQSUMMARY .. DORMANT_WAKE_INTS */
#define GPIO_IO_BANK0_SPAN_RP2040        0x200u
#define GPIO_IO_BANK0_SPAN_RP2350        0x320u  /* through DORMANT_WAKE_INTS */

/* Register Alias Offsets for Atomic Operations */
#define REG_ALIAS_RW_BITS   0x0000      /* Normal read/write */
#define REG_ALIAS_XOR_BITS  0x1000      /* Atomic XOR */
#define REG_ALIAS_SET_BITS  0x2000      /* Atomic SET (write 1s to set bits) */
#define REG_ALIAS_CLR_BITS  0x3000      /* Atomic CLEAR (write 1s to clear bits) */

/* SIO GPIO Registers (fast GPIO access) */
#define SIO_GPIO_IN         (SIO_BASE_GPIO + 0x004)  /* GPIO input values */
#define SIO_GPIO_HI_IN      (SIO_BASE_GPIO + 0x008)  /* QSPI GPIO input values (6 pins) */
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

/* Number of GPIO pins (48 on RP2350, 30 used on RP2040) */
#define NUM_GPIO_PINS       48
#define NUM_GPIO_PINS_RP2040 30
#define GPIO_IRQ_BANKS      6   /* INTR0..INTR5 (RP2350 has 48 pins) */

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

    /* Interrupt registers (6 regs: 8 pins per reg, 48 pins total) */
    uint32_t intr[6];        /* Raw interrupt status (8 pins per register) */
    uint32_t proc0_inte[6];  /* Interrupt enable for processor 0 */
    uint32_t proc0_intf[6];  /* Interrupt force for processor 0 */
    uint32_t proc0_ints[6];  /* Interrupt status for processor 0 */

    /* Pad control registers */
    uint32_t pads[NUM_GPIO_PINS + 2];  /* user pads, then SWCLK and SWD */
} gpio_state_t;

/* GPIO Functions */
void gpio_init(void);
void gpio_reset(void);

uint32_t gpio_read32(uint32_t addr);
void gpio_write32(uint32_t addr, uint32_t val);

/* GPIO pin operations */
void gpio_set_pin(uint8_t pin, uint8_t value);
uint8_t gpio_get_pin(uint8_t pin);
void gpio_set_input_pin(uint8_t pin, uint8_t value);
void gpio_set_direction(uint8_t pin, uint8_t output);
void gpio_set_function(uint8_t pin, uint8_t func);

/* External state */
extern gpio_state_t gpio_state;

#endif /* GPIO_H */
