# Bramble RP2040 Emulator

A from-scratch ARM Cortex-M0+ emulator for the Raspberry Pi RP2040 microcontroller, capable of loading and executing UF2 and ELF firmware with accurate memory mapping and peripheral emulation.

## Current Status: v0.18.0

228 tests passing. Boots and runs Pico SDK firmware with full peripheral emulation.

### Coverage

| Area | Status | Details |
|------|--------|---------|
| CPU | 65+ instructions | Full Thumb-1 + BL/MSR/MRS/DSB/DMB/ISB, O(1) dispatch, NZCV flags |
| Dual-Core | Complete | Zero-copy context switching, per-core RAM, shared FIFO, spinlocks |
| Memory Map | ~90% | Flash + XIP aliases + XIP SRAM + SRAM + SRAM alias + ROM (4KB) |
| Boot | Complete | Vector table, boot2 auto-detect, ROM function table, SDK peripherals |
| Exceptions | ~70% | NVIC priority preemption, SysTick, PendSV, SVCall, PRIMASK |
| Timing | Cycle-accurate | Configurable clock (`-clock 125`), ARMv6-M instruction costs |
| Debugging | GDB RSP | Breakpoints, single-step, register/memory access (`-gdb`) |
| Tests | 218 | CTest integrated, 45+ categories |

### Peripherals

| Peripheral | Address | Emulation Level |
|------------|---------|-----------------|
| GPIO | `0x40014000` / `0xD0000000` | Full (30 pins, SIO, IO_BANK0, PADS, interrupts) |
| UART | `0x40034000` / `0x40038000` | Full (dual PL011, Tx+Rx, 16-deep FIFO, stdin polling) |
| SPI | `0x4003C000` / `0x40040000` | Registers (dual PL022, status, peripheral ID) |
| I2C | `0x40044000` / `0x40048000` | Registers (dual DW_apb_i2c, status, component ID) |
| Timer | `0x40054000` | Full (64-bit counter, 4 alarms, interrupts) |
| PWM | `0x40050000` | Full (8 slices, CSR/DIV/CTR/CC/TOP, interrupts) |
| ADC | `0x4004C000` | Full (5 channels, temp sensor, FIFO, round-robin) |
| DMA | `0x50000000` | Full (12 channels, chaining, 4 alias layouts) |
| PIO | `0x50200000` / `0x50300000` | Full (2 blocks, all 9 opcodes, FIFOs, clock divider) |
| SysTick | `0xE000E010` | Full (CSR/RVR/CVR/CALIB, TICKINT, COUNTFLAG) |
| NVIC | `0xE000E100` | Full (priority preemption, 4 levels, SCB_SHPR) |
| Resets | `0x4000C000` | Full (reset/unreset, RESET_DONE tracking) |
| Clocks | `0x40008000` | Full (10 generators, FC0, SELECTED) |
| XOSC/PLLs | `0x40024000` | Full (STATUS.STABLE, CS.LOCK) |
| Watchdog | `0x40058000` | Full (CTRL, TICK, SCRATCH[0-7]) |
| SIO | `0xD0000000` | Full (GPIO, FIFO, spinlocks, hardware divider) |
| ROM | `0x00000000` | Full (4KB, function table, Thumb code stubs) |
| USB | `0x50110000` | Stub (returns disconnected, SDK falls back to UART) |
| XIP Cache | `0x14000000` | Stub (always ready) + 16KB XIP SRAM |

All peripherals support RP2040 atomic register aliases (SET/CLR/XOR).

### Known Limitations

- **USB**: Stub only (returns disconnected state). SDK gracefully falls back to UART.
- **Cycle timing**: Default 1 MHz (fast-forward). Use `-clock 125` for real RP2040 timing.
- See [ROADMAP](docs/ROADMAP.md) for detailed status.

## Building and Running

### Prerequisites

- GCC cross-compiler: `arm-none-eabi-gcc`
- CMake 3.10+
- Python 3 (for UF2 conversion)
- Standard C library (host)

### Build the Emulator

```bash
cd Bramble
./build.sh
```

This builds the `bramble` executable in the project root.

### Configure Hardware Mode

Edit `include/emulator.h` to select single-core or dual-core:

```c
/* For SINGLE-CORE: */
// #define DUAL_CORE_ENABLED

/* For DUAL-CORE: */
#define DUAL_CORE_ENABLED
```

Then rebuild:

```bash
make clean && make
```

### Build Test Firmware

**Hello World** (prints "Hello from ASM!"):
```bash
cd test-firmware
chmod +x build.sh
./build.sh hello_world
```

**GPIO Test** (toggles LED on GPIO 25):
```bash
cd test-firmware
./build.sh gpio
```

**Timer Test** (measures elapsed time):
```bash
cd test-firmware
./build.sh timer
```

**Alarm Test** (tests timer alarms):
```bash
cd test-firmware
./build.sh alarm
```

**Interactive UART Prompt Test** (reads host stdin via `-stdin` and prints a greeting):
```bash
cd test-firmware
./build.sh name_prompt
```

**Build All Tests**:
```bash
cd test-firmware
./build.sh all
```

### Run

**UF2 Firmware:**

```bash
./bramble hello_world.uf2
./bramble gpio_test.uf2
./bramble timer_test.uf2
./bramble alarm_test.uf2
./bramble name_prompt.uf2 -stdin
printf 'Ada\n' | ./bramble name_prompt.uf2 -stdin
```

**ELF Firmware** (auto-detected by extension):

```bash
./bramble firmware.elf
```

### Run Tests

```bash
cd build
cmake .. && make bramble_tests
ctest --output-on-failure
```

### Debug Modes

Bramble now supports flexible debug output modes:

**Single-Core CPU Step Tracing** (verbose CPU and peripheral logging):
```bash
./bramble -debug timer_test.uf2
```

**Assembly Instruction Tracing** (detailed POP/BX/branch operations):
```bash
./bramble -asm alarm_test.uf2
```

**Combined Debug + Assembly Tracing:**
```bash
./bramble -debug -asm alarm_test.uf2
```

**No Debug Output:**
```bash
./bramble hello_world.uf2
```

**Dual-Core Specific:**
```bash
./bramble firmware.uf2 -debug           # Core 0 debug output
./bramble firmware.uf2 -debug -debug1   # Both cores debug
./bramble firmware.uf2 -status          # Periodic status updates
./bramble firmware.uf2 -debug -status   # Debug + status combined
./bramble firmware.uf2 -stdin           # Enable stdin → UART0 Rx
./bramble firmware.uf2 -gdb            # Start GDB server on port 3333
./bramble firmware.uf2 -gdb 4444       # GDB server on custom port
./bramble firmware.uf2 -clock 125      # Real RP2040 timing (125 MHz)
```

**GDB Remote Debugging:**
```bash
# Terminal 1: Start emulator with GDB server
./bramble firmware.uf2 -gdb

# Terminal 2: Connect GDB
arm-none-eabi-gdb firmware.elf -ex "target remote :3333"
```

Expected output:
```
╔════════════════════════════════════════════════════════════╗
║       Bramble RP2040 Emulator - Dual-Core Mode           ║
╚════════════════════════════════════════════════════════════╝

[Init] Initializing dual-core RP2040 emulator...
[Init] Loading firmware: littleOS.uf2
[Init] Firmware loaded successfully
[Boot] Starting Core 0 from flash...
[Boot] Core 0 SP = 0x20020000
[Boot] Core 0 PC = 0x10000104
[Boot] Core 1 held in reset (waiting for Core 0 to start)

═══════════════════════════════════════════════════════════
Executing...
═══════════════════════════════════════════════════════════
```

## Project Structure

```
Bramble/
├── src/
│   ├── main.c          # Unified entry point, boot, execution (single & dual)
│   ├── cpu.c           # Cortex-M0+ core: O(1) dispatch, dual-core, exceptions
│   ├── instructions.c  # 60+ Thumb instruction implementations
│   ├── membus.c        # Memory bus: pointer-based routing, peripheral stubs
│   ├── elf.c           # ELF32 ARM binary loader
│   ├── uf2.c           # UF2 file loader
│   ├── gpio.c          # GPIO peripheral emulation
│   ├── timer.c         # Hardware timer emulation
│   ├── nvic.c          # NVIC interrupt controller
│   ├── clocks.c        # Resets, Clocks, XOSC, PLLs, Watchdog
│   ├── adc.c           # ADC peripheral emulation
│   ├── rom.c           # ROM function table with Thumb code stubs
│   ├── uart.c          # Dual PL011 UART emulation
│   ├── spi.c           # Dual PL022 SPI emulation
│   ├── i2c.c           # Dual DW_apb_i2c emulation
│   ├── pwm.c           # 8-slice PWM emulation
│   ├── dma.c           # 12-channel DMA controller
│   ├── pio.c           # Dual PIO block emulation (full instruction execution)
│   └── gdb.c           # GDB remote serial protocol stub
├── include/
│   ├── emulator.h      # Core definitions, CPU state, memory layout
│   ├── instructions.h  # Instruction handler prototypes
│   ├── gpio.h          # GPIO register definitions
│   ├── timer.h         # Timer register definitions
│   ├── nvic.h          # NVIC register definitions
│   ├── clocks.h        # Clock-domain peripheral definitions
│   ├── adc.h           # ADC register definitions
│   ├── rom.h           # ROM layout and function codes
│   ├── uart.h          # PL011 UART register definitions
│   ├── spi.h           # PL022 SPI register definitions
│   ├── i2c.h           # DW_apb_i2c register definitions
│   ├── pwm.h           # PWM register definitions
│   ├── dma.h           # DMA controller register definitions
│   ├── pio.h           # PIO register definitions
│   └── gdb.h           # GDB RSP stub definitions
├── tests/
│   └── test_suite.c    # Unit test suite (218+ tests, verbose, CTest integrated)
├── test-firmware/
│   ├── hello_world.S   # Assembly UART test
│   ├── gpio_test.S     # Assembly GPIO test
│   ├── timer_test.S    # Assembly timer test
│   ├── alarm_test.S    # Assembly alarm test
│   ├── interrupt_test.S # Assembly interrupt test
│   ├── linker.ld       # Memory layout definition
│   ├── uf2conv.py      # UF2 conversion utility
│   └── build.sh        # Firmware build script
├── docs/
│   ├── GPIO.md         # GPIO peripheral documentation
│   └── NVIC_audit_report.md # NVIC audit findings and recommendations
├── CMakeLists.txt      # Build configuration
├── build.sh            # Top-level build script
├── CHANGELOG.md        # Version history and changes
├── UPDATES.md          # Detailed per-version update notes
└── README.md           # This file
```

## Hardware Timer ✨

### Features

- **64-bit Counter**: Microsecond-resolution time tracking
- **4 Independent Alarms**: ALARM0-3 with configurable trigger points
- **Interrupt Generation**: Sets INTR bits when alarms fire (NVIC integration pending)
- **Write-1-to-Clear**: Standard ARM interrupt acknowledgment
- **Pause/Resume**: Stop timer for debugging
- **Atomic Operations**: Armed register shows active alarms

### Registers

- **TIMER_TIMELR/TIMEHR** (0x4005400C/08): Read 64-bit counter
- **TIMER_TIMELW/TIMEHW** (0x40054004/00): Write 64-bit counter
- **TIMER_ALARM0-3** (0x40054010-1C): Set alarm compare values
- **TIMER_ARMED** (0x40054020): Shows which alarms are active
- **TIMER_INTR** (0x40054034): Raw interrupt status (W1C)
- **TIMER_INTE** (0x40054038): Interrupt enable mask
- **TIMER_INTS** (0x40054040): Masked interrupt status

### Quick Example

```assembly
/* Read current time */
ldr r0, =0x4005400C      /* TIMER_TIMELR */
ldr r1, [r0]             /* R1 = current time in microseconds */

/* Set alarm for 1000us in future */
ldr r2, =1000
add r1, r2               /* R1 = target time */
ldr r0, =0x40054010      /* TIMER_ALARM0 */
str r1, [r0]             /* Alarm armed automatically */

/* Wait for alarm (polling) */
poll:
    ldr r0, =0x40054034  /* TIMER_INTR */
    ldr r1, [r0]
    movs r2, #1
    tst r1, r2           /* Check bit 0 */
    beq poll

/* Clear interrupt */
movs r1, #1
ldr r0, =0x40054034      /* TIMER_INTR */
str r1, [r0]             /* Write 1 to clear */
```

## GPIO Peripheral

### Features

- **30 GPIO Pins** (GPIO 0-29, matching RP2040)
- **SIO Fast Access**: Direct read/write at 0xD0000000
- **Atomic Operations**: SET, CLR, XOR registers for thread-safe bit manipulation
- **Function Select**: All 10 GPIO functions (SIO, UART, SPI, I2C, PWM, PIO0/1, etc.)
- **Per-Pin Configuration**: Control and status registers via IO_BANK0
- **Pad Control**: Pull-up/down, drive strength via PADS_BANK0
- **Interrupt Support**: Registers implemented (NVIC integration pending)

### Quick Example

```assembly
/* Configure GPIO 25 as output (LED on Pico) */
ldr r0, =0x400140CC      /* GPIO25_CTRL */
movs r1, #5              /* Function 5 = SIO */
str r1, [r0]

/* Enable output */
ldr r0, =0xD0000024      /* SIO_GPIO_OE_SET */
ldr r1, =(1 << 25)       /* Bit 25 */
str r1, [r0]

/* Turn LED on */
ldr r0, =0xD0000014      /* SIO_GPIO_OUT_SET */
str r1, [r0]
```

See [docs/GPIO.md](docs/GPIO.md) for complete documentation.

## Dual-Core Support ✨

### Features

- **Independent Core Execution**: Both cores run independently with their own:
  - Program counters (PC)
  - Stack pointers (SP)
  - Register sets (R0-R12, LR)
  - Debug flags (-debug, -debug1)

- **Memory Sharing**:
  - **Flash (2 MB)**: Shared and execute-only
  - **Core 0 RAM (128 KB)**: 0x20000000 - 0x2001FFFF
  - **Core 1 RAM (128 KB)**: 0x20020000 - 0x2003FFFF
  - **Shared RAM (64 KB)**: 0x20040000 - 0x2004FFFF

- **Inter-Core Communication**:
  - **Dual FIFOs**: Core 0 ↔ Core 1 messaging
  - **Spinlocks**: Hardware-level synchronization (32 spinlocks)
  - **SIO Atomic Operations**: Thread-safe bit manipulation

### Memory Layout (Dual-Core)

```
Flash:            0x10000000 - 0x10200000  (2 MB, shared)
Core 0 RAM:       0x20000000 - 0x20020000  (128 KB, core-local)
Core 1 RAM:       0x20020000 - 0x20040000  (128 KB, core-local)
Shared RAM:       0x20040000 - 0x20050000  (64 KB, shared)
Total RAM:        320 KB usable, 264 KB available
```

### Usage Example

```c
// In C firmware code for dual-core operation:

// Core 0: Send message to Core 1
fifo_push(CORE0, 0x12345678);

// Core 1: Receive message
uint32_t msg = fifo_pop(CORE1);

// Both cores: Synchronized access to shared memory
spinlock_acquire(0);
shared_counter++;
spinlock_release(0);
```

### Building Dual-Core Firmware

Most firmware builds naturally for dual-core:

```bash
# In your firmware makefile:
make CORES=2  # Compiles with dual-core definitions
```

Then run with:
```bash
./bramble firmware.uf2 -status  # Show status for both cores
```

## Technical Implementation

### Memory Map

The emulator accurately models the RP2040 address space:
- **Flash (XIP)**: 0x10000000 - 0x101FFFFF (2MB executable, + aliases at 0x11/0x12/0x13)
- **XIP Cache Control**: 0x14000000 (CTRL, FLUSH, STAT, counters)
- **XIP SRAM**: 0x15000000 - 0x15003FFF (16KB cache as SRAM)
- **SRAM**: 0x20000000 - 0x20041FFF (264KB, + mirror alias at 0x21000000)
- **APB Peripherals**: 0x40000000 - 0x4FFFFFFF (UART, GPIO, timers, etc.)
- **SIO**: 0xD0000000 - 0xD0000FFF (Single-cycle I/O, GPIO fast access)

All accesses respect alignment requirements and return appropriate values for unimplemented regions.

### Peripheral Integration

Peripherals are integrated into the memory bus (`membus.c`):
- Reads/writes to peripheral address ranges are routed to peripheral modules
- GPIO: `0x40014000` (IO_BANK0), `0x4001C000` (PADS), `0xD0000000` (SIO)
- Timer: `0x40054000` (64-bit counter, 4 alarms, interrupts)
- UART: `0x40034000` / `0x40038000` (PL011 dual)
- SPI: `0x4003C000` / `0x40040000` (PL022 dual)
- I2C: `0x40044000` / `0x40048000` (DW_apb_i2c dual)
- PWM: `0x40050000` (8 slices)
- DMA: `0x50000000` (12 channels with chaining)
- PIO: `0x50200000` / `0x50300000` (2 blocks, 4 SMs each, full instruction execution)
- XIP Cache: `0x14000000` (CTRL, FLUSH, STAT, counters, stream)
- XIP SRAM: `0x15000000` (16KB cache as SRAM)
- ROM: `0x00000000` (4KB with function table and Thumb code)

### Timer Timing Model

The timer uses a cycle-accurate timing model with configurable clock frequency:
- **Default: 1 MHz** (1 cycle = 1 µs, fast-forward mode for speed)
- **Real RP2040: 125 MHz** (`-clock 125`, 125 cycles per microsecond)
- Each instruction costs 1-4+ CPU cycles based on the ARMv6-M instruction timing table
- A cycle accumulator converts CPU cycles to microseconds for the timer
- SysTick counts in raw CPU cycles (correct per ARM Cortex-M specification)

Alarms trigger when:
```c
if (timer_low_32bits >= alarm_value) {
    set_interrupt_bit();
    disarm_alarm();
}
```

### Instruction Dispatch

Instructions are dispatched via a 256-entry O(1) lookup table indexed by `instr >> 8`:

1. **32-bit instructions** (BL/BLX, MSR/MRS, DSB/DMB/ISB) detected by top-5-bit check and handled before table lookup
2. **16-bit instructions** dispatched via `dispatch_table[instr >> 8](instr)` in constant time
3. **Secondary dispatchers** handle entries where multiple instructions share the same top byte (ALU block 0x40-0x43, hints 0xBF, etc.)
4. **`pc_updated` flag**: Handlers that modify PC set this flag; `cpu_step()` auto-advances PC by 2 only if unset

### Flag Management

The emulator implements full APSR flag semantics:
- **N (Negative)**: Bit 31 of result
- **Z (Zero)**: Result equals zero
- **C (Carry)**: Unsigned overflow (for ADD) or NOT borrow (for SUB)
- **V (Overflow)**: Signed overflow (operands same sign, result different)

Helper functions `update_add_flags()` and `update_sub_flags()` ensure consistency across all arithmetic instructions.

### UF2 Loading

The loader validates:
- Magic numbers (0x0A324655, 0x9E5D5157)
- Target address in flash range
- Payload size (476 bytes standard)
- Family ID (0xE48BFF56 for RP2040)

Multi-block firmware images are supported with sequential loading.

### Dual-Core Architecture

**Core Synchronization**:
- Both cores execute independently in the main loop
- `dual_core_step()` advances both cores one instruction each
- `any_core_running()` checks if either core is still executing
- Shared state (memory, peripherals) is automatically synchronized

**FIFO Implementation**:
```c
typedef struct {
    uint32_t buffer[FIFO_DEPTH];  // 8 entries per FIFO
    uint16_t write_ptr;
    uint16_t read_ptr;
    uint16_t count;
} fifo_t;
```

**Spinlock Implementation**:
```c
void spinlock_acquire(uint8_t lock_id) {
    while (spinlocks[lock_id].locked) {
        // Spin until lock is released
    }
    spinlocks[lock_id].locked = 1;
}
```

## Performance

The emulator executes simple firmware at approximately:
- **Simple loop**: ~10,000 instructions/second (development build)
- **UART output**: Limited by `putchar()` overhead
- **GPIO operations**: Instant (no electrical simulation)
- **Timer operations**: Lightweight counter increment per cycle
- **Memory access**: Direct array indexing (no caching overhead)
- **Dual-core**: Both cores step together (no penalty)

The GPIO test executes 2M+ instructions in under 1 second. Performance is adequate for firmware debugging and testing. Optimization (JIT, caching) could achieve 100x improvement but isn't currently needed.

## Future Work

1. **GDB Enhancements**: Watchpoints, Core 1 debugging, conditional breakpoints
2. **USB Device Mode**: Full endpoint handling (stub returns disconnected; SDK falls back to UART)
3. **Performance**: JIT compilation for hot loops, instruction caching

## Contributing

The Bramble project is open for contributions! Areas that need help:

1. **Peripheral Emulation**: USB device mode
2. **Testing**: New test firmware, edge cases, performance benchmarks
3. **Debugging**: GDB watchpoints, Core 1 debugging
4. **Documentation**: Register descriptions, usage examples, architecture guides

Run `ctest` to verify changes don't break existing tests. See [CHANGELOG.md](CHANGELOG.md) for recent updates and [docs/](docs/) for detailed technical documentation.

## License

MIT License - See LICENSE file for details

## Contact & Support

For issues, questions, or contributions:
- Open an issue on GitHub
- Check existing documentation in [docs/](docs/)
- Review [CHANGELOG.md](CHANGELOG.md) for recent changes
- See [NVIC Audit Report](docs/NVIC_audit_report.md) for known limitations
