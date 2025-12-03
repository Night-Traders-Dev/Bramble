# GPIO Peripheral Emulation

## Overview

Bramble now includes comprehensive GPIO (General Purpose Input/Output) emulation for the RP2040's 30 GPIO pins. This enables firmware to configure pins, read/write values, and manage pin functions just like on real hardware.

## Implementation Details

### Emulated Features

#### ✅ Fully Implemented

- **30 GPIO pins** (GPIO 0-29)
- **SIO Fast GPIO Access**
  - Direct read/write via `SIO_GPIO_IN`, `SIO_GPIO_OUT`, `SIO_GPIO_OE`
  - Atomic set/clear/toggle operations (SET, CLR, XOR registers)
  - Output enable control
- **IO_BANK0 Configuration**
  - Per-pin control registers (function select, drive strength, etc.)
  - Per-pin status registers
- **PADS_BANK0 Pad Control**
  - Pull-up/pull-down configuration
  - Input enable/output disable
  - Drive strength settings
- **Interrupt Registers**
  - Raw interrupt status (`INTR`)
  - Interrupt enable/force/status for processor 0
  - Edge and level detection (stubbed)

#### ⚠️ Partially Implemented

- **Interrupt Generation**: Registers are emulated but interrupts aren't generated to the CPU (requires NVIC)
- **Edge/Level Detection**: Interrupt types are defined but not actively monitored

#### ❌ Not Yet Implemented

- **Processor 1 Interrupts**: Only PROC0 interrupt registers are implemented
- **Dormant Wake**: Wake-from-dormant functionality
- **Input Synchronization**: Real RP2040 synchronizes inputs; emulator reads instantly

## Memory Map

### SIO GPIO Registers (Fast Access)

| Address | Name | Description |
|---------|------|-------------|
| `0xD0000004` | `SIO_GPIO_IN` | Read current GPIO input values |
| `0xD0000010` | `SIO_GPIO_OUT` | GPIO output values |
| `0xD0000014` | `SIO_GPIO_OUT_SET` | Atomic set bits (write 1 to set) |
| `0xD0000018` | `SIO_GPIO_OUT_CLR` | Atomic clear bits (write 1 to clear) |
| `0xD000001C` | `SIO_GPIO_OUT_XOR` | Atomic toggle bits (write 1 to toggle) |
| `0xD0000020` | `SIO_GPIO_OE` | Output enable mask |
| `0xD0000024` | `SIO_GPIO_OE_SET` | Atomic set output enable |
| `0xD0000028` | `SIO_GPIO_OE_CLR` | Atomic clear output enable |
| `0xD000002C` | `SIO_GPIO_OE_XOR` | Atomic toggle output enable |

### IO_BANK0 Registers (Per-Pin Configuration)

**Base Address**: `0x40014000`

**Per-pin layout** (each pin has 8 bytes):
- Offset `0x00`: `GPIO_STATUS` (read-only status)
- Offset `0x04`: `GPIO_CTRL` (function select, interrupt config)

**Example for GPIO 25** (LED on Pico):
- `0x400140C8`: GPIO25_STATUS
- `0x400140CC`: GPIO25_CTRL

### PADS_BANK0 Registers

**Base Address**: `0x4001C000`

Pad control for each GPIO pin (pull-ups, drive strength, slew rate).

## GPIO Function Select

| Value | Function | Description |
|-------|----------|-------------|
| `0` | XIP | Execute-in-place flash |
| `1` | SPI | SPI interface |
| `2` | UART | UART interface |
| `3` | I2C | I2C interface |
| `4` | PWM | PWM output |
| `5` | **SIO** | **Software-controlled I/O (default for GPIO)** |
| `6` | PIO0 | Programmable I/O block 0 |
| `7` | PIO1 | Programmable I/O block 1 |
| `8` | CLOCK | Clock output |
| `9` | USB | USB interface |
| `0x1F` | NULL | Disable pin |

**Note**: For basic GPIO control, use function `5` (SIO).

## Usage Examples

### Example 1: Blink LED on GPIO 25 (Pico Onboard LED)

```assembly
/* Configure GPIO 25 as SIO */
ldr r0, =0x400140CC      /* GPIO25_CTRL */
movs r1, #5              /* Function 5 = SIO */
str r1, [r0]

/* Enable output */
ldr r0, =0xD0000024      /* SIO_GPIO_OE_SET */
ldr r1, =(1 << 25)       /* Bit 25 */
str r1, [r0]

/* Turn LED on */
ldr r0, =0xD0000014      /* SIO_GPIO_OUT_SET */
ldr r1, =(1 << 25)
str r1, [r0]

/* Turn LED off */
ldr r0, =0xD0000018      /* SIO_GPIO_OUT_CLR */
ldr r1, =(1 << 25)
str r1, [r0]
```

### Example 2: Read Button on GPIO 14

```assembly
/* Configure GPIO 14 as input (SIO function, OE=0) */
ldr r0, =0x40014070      /* GPIO14_CTRL */
movs r1, #5              /* Function 5 = SIO */
str r1, [r0]

/* Ensure output is disabled (default, but explicit) */
ldr r0, =0xD0000028      /* SIO_GPIO_OE_CLR */
ldr r1, =(1 << 14)       /* Bit 14 */
str r1, [r0]

/* Read pin state */
ldr r0, =0xD0000004      /* SIO_GPIO_IN */
ldr r0, [r0]
ldr r1, =(1 << 14)
ands r0, r1              /* Check if bit 14 is set */
beq button_not_pressed
/* Button is pressed */
```

### Example 3: Atomic Toggle

```assembly
/* Toggle GPIO 25 without affecting other pins */
ldr r0, =0xD000001C      /* SIO_GPIO_OUT_XOR */
ldr r1, =(1 << 25)
str r1, [r0]             /* Atomically toggle bit 25 */
```

## Testing GPIO

### Build GPIO Test Firmware

```bash
cd test-firmware
chmod +x build.sh
./build.sh gpio
```

This creates `gpio_test.uf2` which:
1. Configures GPIO 25 as output
2. Toggles it 5 times (ON/OFF)
3. Reads back final state
4. Prints status to UART

### Run GPIO Test

```bash
./bramble gpio_test.uf2
```

**Expected Output:**
```
GPIO Test Starting...
GPIO 25 configured as output
LED ON
LED OFF
LED ON
LED OFF
LED ON
LED OFF
LED ON
LED OFF
LED ON
LED OFF
Final state: LED OFF
GPIO Test Complete!
```

## Internal State

The GPIO emulation maintains this state:

```c
typedef struct {
    uint32_t gpio_in;        /* Current input values */
    uint32_t gpio_out;       /* Output values */
    uint32_t gpio_oe;        /* Output enable mask */
    
    struct {
        uint32_t status;     /* Per-pin status */
        uint32_t ctrl;       /* Per-pin control */
    } pins[30];
    
    uint32_t intr[4];        /* Interrupt status */
    uint32_t proc0_inte[4];  /* Interrupt enable */
    uint32_t proc0_intf[4];  /* Interrupt force */
    uint32_t proc0_ints[4];  /* Interrupt status (computed) */
    
    uint32_t pads[30];       /* Pad configurations */
} gpio_state_t;
```

## Pin Behavior

### Output Pins (OE=1)
- Writing to `SIO_GPIO_OUT` sets pin value
- Reading `SIO_GPIO_IN` returns output value (loopback)
- Actual output value visible in `gpio_out`

### Input Pins (OE=0)
- Writing to `SIO_GPIO_OUT` has no effect on pin
- Reading `SIO_GPIO_IN` returns `gpio_in` value
- External simulation code can set `gpio_in` bits

### Bidirectional Pins
- Can be switched between input/output dynamically
- Common pattern: read switch, write LED on same pin

## Limitations

1. **No Real I/O**: GPIO doesn't connect to actual hardware (emulator only)
2. **Instant Response**: No propagation delay or glitch filtering
3. **No Electrical Properties**: Drive strength, slew rate are stored but not simulated
4. **Interrupt Stub**: Interrupt detection works but doesn't trigger CPU exceptions (needs NVIC)

## Future Enhancements

- [ ] **Virtual Peripherals**: Attach simulated LEDs, buttons, sensors to GPIO pins
- [ ] **Interrupt Generation**: Trigger CPU interrupts when enabled
- [ ] **Waveform Logging**: Record GPIO state changes for debugging
- [ ] **Pin Visualization**: GUI showing pin states in real-time
- [ ] **Input Injection**: Load pin state changes from file for testing

## API Functions

Internal C functions for GPIO control:

```c
void gpio_init(void);                      /* Initialize GPIO subsystem */
void gpio_reset(void);                     /* Reset to power-on defaults */

uint32_t gpio_read32(uint32_t addr);       /* Read GPIO register */
void gpio_write32(uint32_t addr, uint32_t val); /* Write GPIO register */

void gpio_set_pin(uint8_t pin, uint8_t value);    /* Set pin high/low */
uint8_t gpio_get_pin(uint8_t pin);                /* Read pin state */
void gpio_set_direction(uint8_t pin, uint8_t output); /* Set direction */
void gpio_set_function(uint8_t pin, uint8_t func);    /* Set function */
```

## Integration

GPIO is integrated into the memory bus (`membus.c`):
- Reads/writes to GPIO address ranges are routed to `gpio_read32()` / `gpio_write32()`
- 8-bit and 16-bit accesses are promoted to 32-bit operations
- GPIO is initialized at boot before firmware execution

## References

- [RP2040 Datasheet - Section 2.19 (GPIO)](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [Pico SDK GPIO Documentation](https://github.com/raspberrypi/pico-sdk)
- Bramble source: `src/gpio.c`, `include/gpio.h`

---

**Next Steps**: With GPIO working, the next logical addition is **Timer peripherals** for delays and scheduling, followed by **NVIC** for interrupt support.
