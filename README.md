# Bramble RP2040 Emulator

A from-scratch ARM Cortex-M0+ emulator for the Raspberry Pi RP2040 microcontroller, capable of loading and executing UF2 and ELF firmware with accurate memory mapping and peripheral emulation.

## Current Status: v0.35.0

276 tests passing. Boots and runs Pico SDK firmware including **MicroPython v1.27.0**, **CircuitPython 10.1.3**, and **littleOS**, with Pico W/CYW43 support, USB CDC REPL, flash write-through persistence, SD card and eMMC emulation, UART-to-TCP networking, multi-instance wiring, GDB watchpoints, host-threaded execution, a decoded instruction cache, optional JIT acceleration, automatic privilege escalation, ARM semihosting, code coverage, hotspot profiling, cycle profiling, call graphs, GPIO VCD traces, IRQ latency analysis, scripted I/O, golden file testing, and full CI integration.

### Coverage

| Area | Status | Details |
|------|--------|---------|
| CPU | 65+ instructions | Full Thumb-1 + BL/MSR/MRS/DSB/DMB/ISB, O(1) dispatch, NZCV flags |
| Dual-Core | Complete | Host-threaded (`-cores 2`), WFI sleep, shared FIFO, spinlocks, core pool, Core 1 auto-launch |
| Memory Map | ~97% | Flash + XIP aliases + XIP SRAM + SRAM + SRAM alias + ROM (16KB) + NVIC/SCB MMIO + XIP SSI/IO_QSPI/PADS_QSPI/BUSCTRL alias coverage |
| Boot | Complete | Vector table, boot2 auto-detect, ROM function table, ROM soft-float/double |
| Exceptions | ~95% | NVIC priority preemption, SysTick, PendSV, SVCall, HardFault, nested returns, `cpu_step()` IRQ delivery, double-fault lockup |
| Timing | Cycle-accurate | Configurable clock (`-clock 125`), ARMv6-M instruction costs |
| Debugging | GDB RSP | Breakpoints, watchpoints, conditional breakpoints, dual-core threads (`-gdb`) |
| Flash | Write-through | `-flash <path>` with immediate sync on every write |
| Storage | SD card + eMMC | SPI-attached file-backed block devices |
| WiFi | CYW43 (Pico W) | gSPI-over-PIO emulation with optional TAP bridge (`-wifi`, `-tap`) |
| Performance | ICache + JIT | 64K decoded cache by default, optional hot-block JIT (`-jit`) |
| Privilege | Auto-sudo | `-tap` and `-mount` auto-escalate with `BRAMBLE_ESCALATED` guard |
| Dev Tools | 18 tools | Semihosting, coverage, hotspots, profile, trace, callgraph, VCD, IRQ latency, stack check, bus logging, watch, expect, script, fault injection, heatmap, symbols, exit codes, timeouts |
| Tests | 276 | CTest integrated, 50+ categories |

### Peripherals

| Peripheral | Address | Emulation Level |
|------------|---------|-----------------|
| GPIO | `0x40014000` / `0xD0000000` | Full (30 pins, SIO, IO_BANK0, PADS, edge/level interrupts) |
| UART | `0x40034000` / `0x40038000` | Full (dual PL011, Tx+Rx, 16-deep FIFO, active-console stdin routing) |
| SPI | `0x4003C000` / `0x40040000` | Full (dual PL022, 8-deep TX/RX FIFOs, device callbacks) |
| I2C | `0x40044000` / `0x40048000` | Full (dual DW_apb_i2c, 16-deep RX FIFO, device callbacks) |
| Timer | `0x40054000` | Full (64-bit counter, 4 alarms, interrupts) |
| PWM | `0x40050000` | Full (8 slices, CSR/DIV/CTR/CC/TOP, interrupts) |
| ADC | `0x4004C000` | Full (5 channels, temp sensor, FIFO, round-robin) |
| DMA | `0x50000000` | Full (12 channels, chaining, 4 alias layouts) |
| PIO | `0x50200000` / `0x50300000` | Full (2 blocks, all 9 opcodes, FIFOs, clock divider) |
| SysTick | `0xE000E010` | Full (CSR/RVR/CVR/CALIB, TICKINT, COUNTFLAG) |
| NVIC | `0xE000E100` | Full (priority preemption, 4 levels, SCB_SHPR) |
| Resets | `0x4000C000` | Full (reset/unreset, RESET_DONE tracking) |
| Clocks | `0x40008000` | Full (10 generators, FC0 dynamic freq, SELECTED) |
| XOSC/PLLs | `0x40024000` | Full (STATUS.STABLE, CS.LOCK) |
| Watchdog | `0x40058000` | Full (CTRL, TICK, SCRATCH[0-7], reboot with full multicore state reset) |
| SIO | `0xD0000000` | Full (GPIO, FIFO, spinlocks, hardware divider, interpolators) |
| ROM | `0x00000000` | Full (16KB, function table, soft-float/double, flash write) |
| USB | `0x50110000` | Full (host enumeration, CDC data bridge, stdio_usb, multi-packet IN) |
| SYSINFO | `0x40000000` | Stub (CHIP_ID=RP2040-B2, PLATFORM=ASIC) |
| SYSCFG | `0x40004000` | Full (NMI mask, proc config, debug force, mem power-down) |
| TBMAN | `0x4006C000` | Full (PLATFORM=ASIC, testbench manager) |
| IO_QSPI | `0x40018000` | Stub (6 QSPI GPIO pins, STATUS/CTRL) |
| PADS_QSPI | `0x40020000` | Stub (QSPI pad electrical control) |
| ROSC | `0x40060000` | Full (STATUS, RANDOMBIT LFSR, CTRL enable) |
| RTC | `0x4005C000` | Full (LOAD strobe, calendar rollover, leap year, ticking) |
| XIP Cache | `0x14000000` | Stub (always ready) + 16KB XIP SRAM |
| CYW43 | Pico W via PIO/SPI | Functional (scan/connect path, WLAN framing, TAP bridge) |

### Storage Devices

| Device         | Interface          | Details                                                                |
|----------------|--------------------|------------------------------------------------------------------------|
| SD Card (SDHC) | SPI (default SPI1) | Full SPI-mode protocol, CSD v2.0, single/multi-block R/W, file-backed |
| eMMC           | SPI (default SPI0) | CMD1 init, EXT_CSD, sector addressing, file-backed                    |

Both devices attach via `spi_attach_device()` callbacks with periodic flush and flush-on-exit.

All peripherals support RP2040 atomic register aliases (SET/CLR/XOR).

### Known Limitations

- **Cycle timing**: Default 1 MHz (fast-forward). Use `-clock 125` for real RP2040 timing.
- **Fidelity tradeoffs**: DMA pacing, very high-speed PIO timing, and non-CDC USB device behavior are still functional models rather than fully cycle-perfect hardware.
- See [ROADMAP](docs/ROADMAP.md) for detailed status.

## Building and Running

### Prerequisites

- CMake 3.10+
- Standard C library (host)
- `arm-none-eabi-gcc` and Python 3 if you want to build the sample/test firmware

### Build the Emulator

```bash
./build.sh
```

This builds the `bramble` executable in the project root.

You can also build explicitly with CMake:

```bash
cmake -S . -B build
cmake --build build -j
```

### Choose Core Mode

Bramble builds with dual-core support enabled by default. Select the active cores at runtime:

```bash
./bramble firmware.uf2 -cores 1
./bramble firmware.uf2 -cores 2
./bramble firmware.uf2 -cores auto
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
ctest --test-dir build --output-on-failure
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
./bramble firmware.uf2 -stdin           # Route stdin to USB CDC when active, else UART0
./bramble firmware.uf2 -gdb            # Start GDB server on port 3333
./bramble firmware.uf2 -gdb 4444       # GDB server on custom port
./bramble firmware.uf2 -clock 125      # Real RP2040 timing (125 MHz)
./bramble firmware.uf2 -flash fs.bin   # Persistent flash storage
./bramble firmware.uf2 -debug-mem      # Log unmapped peripheral access
./bramble firmware.uf2 -jit            # Enable JIT for hot flash/ROM loops
./bramble firmware.uf2 -cores 2 -thread-quantum 128  # Tune threaded timeslice
```

**Networking (UART-to-TCP bridge):**

```bash
# Bridge UART0 to TCP port (connect with nc, minicom, etc.)
./bramble firmware.uf2 -net-uart0 9999 -stdin
# In another terminal: nc localhost 9999

# Connect UART0 to a remote host
./bramble firmware.uf2 -net-uart0-connect 192.168.1.10:9999
```

**Multi-Device Wiring (inter-instance communication):**

```bash
# Terminal 1: Instance A with UART0 wired via Unix socket
./bramble fw_sensor.uf2 -wire-uart0 /tmp/uart_link.sock -stdin

# Terminal 2: Instance B with UART0 wired to the same socket
./bramble fw_controller.uf2 -wire-uart0 /tmp/uart_link.sock -stdin

# UART TX on either side arrives as UART RX on the other
# GPIO pins can also be wired: -wire-gpio /tmp/gpio_link.sock
```

**WiFi (Pico W / CYW43):**

```bash
# Basic Pico W/CYW43 emulation
./bramble firmware.uf2 -wifi

# Bridge emulated WLAN frames to a host TAP interface
./bramble firmware.uf2 -wifi -tap tap0
```

**Storage Devices (SD Card / eMMC):**

```bash
# Attach a 32MB SD card image on SPI1 (default)
./bramble firmware.uf2 -sdcard sdcard.img -sdcard-size 32

# Attach SD card on SPI0 instead
./bramble firmware.uf2 -sdcard sdcard.img -sdcard-spi 0

# Attach a 64MB eMMC image on SPI0 (default)
./bramble firmware.uf2 -emmc emmc.img -emmc-size 64

# Attach eMMC on SPI1 instead
./bramble firmware.uf2 -emmc emmc.img -emmc-spi 1

# Combine with flash persistence and MicroPython
./bramble python/micropython.uf2 -stdin -clock 125 -flash mpy.bin -sdcard sd.img
```

**MicroPython REPL:**

```bash
./bramble python/micropython.uf2 -stdin -clock 125 -flash mpy.bin
```

Output:
```
MicroPython v1.27.0 on 2025-12-09; Raspberry Pi Pico with RP2040
Type "help()" for more information.
>>>
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
│   ├── usb.c           # USB controller with host enumeration + CDC bridge
│   ├── rtc.c           # RTC peripheral (ticking, calendar, leap year)
│   ├── gdb.c           # GDB remote serial protocol stub
│   ├── netbridge.c     # UART-to-TCP bridge
│   ├── wire.c          # Multi-instance Unix socket wiring
│   ├── storage.c       # Flash write-through persistence
│   ├── sdcard.c        # SD card SPI emulation (SDHC, file-backed)
│   ├── emmc.c          # eMMC SPI emulation (file-backed)
│   ├── fatfs.c         # FAT16 helpers for flash/FUSE
│   ├── fuse_mount.c    # Optional FUSE mount integration
│   ├── w5500.c         # W5500 Ethernet device model
│   ├── bme280.c        # BME280 sensor model
│   ├── corepool.c      # Host-threaded execution + core allocation
│   ├── cyw43.c         # CYW43 WiFi emulation
│   └── tapif.c         # TAP bridge for Pico W traffic
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
│   ├── usb.h           # USB controller register definitions
│   ├── rtc.h           # RTC register definitions
│   ├── gdb.h           # GDB RSP stub definitions
│   ├── netbridge.h     # UART network bridge definitions
│   ├── wire.h          # Multi-instance wire protocol definitions
│   ├── storage.h       # Flash write-through definitions
│   ├── sdcard.h        # SD card SPI definitions
│   ├── emmc.h          # eMMC SPI definitions
│   ├── fatfs.h         # FAT filesystem helpers
│   ├── fuse_mount.h    # FUSE mount definitions
│   ├── w5500.h         # W5500 device definitions
│   ├── bme280.h        # BME280 device definitions
│   ├── corepool.h      # Core pool definitions
│   ├── cyw43.h         # CYW43 WiFi definitions
│   └── tapif.h         # TAP bridge definitions
├── tests/
│   └── test_suite.c    # Unit test suite (274 tests, verbose, CTest integrated)
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
│   ├── NVIC_audit_report.md # NVIC audit findings and recommendations
│   └── ROADMAP.md      # Development roadmap and feature status
├── CMakeLists.txt      # Build configuration
├── build.sh            # Top-level build script
├── CHANGELOG.md        # Version history and changes
└── README.md           # This file
```

## Hardware Timer ✨

### Features

- **64-bit Counter**: Microsecond-resolution time tracking
- **4 Independent Alarms**: ALARM0-3 with configurable trigger points
- **Interrupt Generation**: Sets INTR bits when alarms fire and signals NVIC IRQ 0-3 when enabled
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
- **Interrupt Support**: Full edge/level detection with NVIC delivery (IRQ 13)

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
- ROM: `0x00000000` (16KB with function table and Thumb code)

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

The UF2 loader validates:
- Magic numbers (0x0A324655, 0x9E5D5157, 0x0AB16F30)
- Target address in flash range
- Payload size bounds (up to 476 bytes per block)
- Overflow-safe target calculations before writing to flash

Multi-block firmware images are supported with sequential loading, and malformed or out-of-range blocks are rejected without modifying flash.

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
uint32_t spinlock_acquire(uint32_t lock_id) {
    if (spinlocks[lock_id] & SPINLOCK_LOCKED) {
        return 0;
    }
    spinlocks[lock_id] = SPINLOCK_VALID | SPINLOCK_LOCKED;
    return 1u << lock_id;
}
```

## Performance

Bramble now ships with a 64K decoded instruction cache enabled by default and optional JIT basic-block compilation via `-jit`.

- **Instruction cache**: Avoids repeat decode/dispatch work for hot Thumb-1 paths.
- **JIT**: Compiles hot flash/ROM basic blocks and reports execution stats on exit.
- **Threaded execution**: `-cores 2` and `-cores auto` map emulated cores to host pthreads while preserving a shared-state lock.
- **I/O behavior**: Firmware output stays on stdout while emulator diagnostics stay on stderr, which keeps pipes and scripted runs predictable.

For benchmarking details, see `tests/benchmark.c`.

## Future Work

1. **Timing fidelity**: DMA pacing, high-speed PIO timing, and more USB edge cases.
2. **Device breadth**: More SPI/I2C device models and broader Pico W/WLAN coverage.
3. **Tooling**: More firmware examples, benchmarks, and workflow automation around regression testing.

## Contributing

The Bramble project is open for contributions! Areas that need help:

1. **Testing**: Firmware coverage, edge cases, and performance benchmarks.
2. **Device models**: Sensors, networking peripherals, and board-specific integrations.
3. **Documentation**: Register descriptions, usage examples, architecture guides

Run `ctest --test-dir build --output-on-failure` to verify changes don't break existing tests. See [CHANGELOG.md](CHANGELOG.md) for release history and [docs/](docs/) for detailed technical documentation.

## License

MIT License - See LICENSE file for details

## Contact & Support

For issues, questions, or contributions:
- Open an issue on GitHub
- Check existing documentation in [docs/](docs/)
- Review [CHANGELOG.md](CHANGELOG.md) for recent changes
- See [NVIC Audit Report](docs/NVIC_audit_report.md) for historical background on early NVIC issues
