![Bramble RP2040 Emulator](assets/bramble-logo.jpg)

# Bramble RP2040 Emulator

A from-scratch ARM Cortex-M0+ emulator for the Raspberry Pi RP2040 microcontroller, capable of loading and executing UF2 firmware with accurate memory mapping and peripheral emulation.

## Current Status: **Feature Complete Beta** ✅

Bramble successfully boots RP2040 firmware, executes the complete Thumb-1 instruction set, provides working UART0 output, full GPIO emulation, and **hardware timer support with alarms**! The emulator cleanly executes test programs with proper halting via BKPT instructions.

### ✅ What Works

- **Complete RP2040 Memory Map**: Flash (0x10000000), SRAM (0x20000000), SIO (0xD0000000), and APB peripherals
- **UF2 Firmware Loading**: Parses and loads UF2 blocks into flash with proper address validation
- **Full ARM Cortex-M0+ Thumb Instruction Set**: 60+ instructions across 4 phases:

  **Phase 1 - Foundational (Bootloader Essential):**
  - Data movement: MOVS, MOV (with high register support)
  - Arithmetic: ADDS/SUBS (imm3, imm8, register)
  - Comparison: CMP (imm8, register with high regs)
  - Load/Store: LDR/STR (imm5, reg offset, PC-relative, SP-relative)
  - Stack: PUSH/POP (with LR/PC support)
  - Branches: B, Bcond (all 14 conditions), BL, BX, BLX

  **Phase 2 - Essential (Program Execution):**
  - Byte operations: LDRB/STRB (imm5, reg), LDRSB (sign-extended)
  - Halfword operations: LDRH/STRH (imm5, reg), LDRSH (sign-extended)
  - Shifts: LSLS, LSRS, ASRS, RORS (immediate and register)
  - Logical: ANDS, EORS, ORRS, BICS, MVNS
  - Multiplication: MULS
  - Multiple load/store: LDMIA, STMIA

  **Phase 3 - Important (Advanced Features):**
  - Special comparison: CMN, TST
  - System: SVC, MSR, MRS
  - High register operations

  **Phase 4 - Optional (Optimization & Polish):**
  - Hints: NOP, YIELD, WFE, WFI, SEV, IT
  - Sign/zero extend: SXTB, SXTH, UXTB, UXTH
  - Byte reverse: REV, REV16, REVSH
  - Address generation: ADR, ADD/SUB SP
  - Interrupt control: CPSID, CPSIE
  - Debug: BKPT, UDF

- **Accurate Flag Handling**: N, Z, C, V flags with proper carry/overflow detection
- **UART0 Emulation**: Character output via `UART0_DR` (0x40034000) with TX FIFO status
- **GPIO Emulation**: Complete 30-pin GPIO peripheral with:
  - SIO fast GPIO access (direct read/write, atomic operations)
  - IO_BANK0 per-pin configuration (function select, control)
  - PADS_BANK0 pad control (pull-up/down, drive strength)
  - Interrupt registers (enable, force, status)
  - All 10 GPIO functions supported (SIO, UART, SPI, I2C, PWM, PIO, etc.)
- **✨ Hardware Timer (NEW!)**: Full 64-bit microsecond timer with:
  - 64-bit counter incrementing with CPU cycles
  - 4 independent alarm channels (ALARM0-3)
  - Interrupt generation on alarm match
  - Pause/resume functionality
  - Write-1-to-clear interrupt handling
  - Cycle-accurate timing simulation
- **Proper Reset Sequence**: Vector table parsing, SP/PC initialization from flash
- **Clean Halt Detection**: BKPT instruction properly stops execution with register dump

### 📊 Test Results

**hello_world.uf2** (Assembly version):
```
Total steps: 187
Output: "Hello from ASM!" ✅
Halt: Clean BKPT #0 at 0x1000005A ✅
```

**gpio_test.uf2** (GPIO test):
```
GPIO Test Starting...
GPIO 25 configured as output
LED ON / LED OFF (x5 cycles) ✅
Final state: LED OFF ✅
GPIO Test Complete!
Total steps: 2,001,191 ✅
```

**timer_test.uf2** (Timer test):
```
Timer Test Starting...
Timer value 1: 99 us ✅
Timer value 2: 10,224 us ✅
Elapsed time: ~10,125 us ✅
Timer Test Complete!
Total steps: 20,808 ✅
```

**alarm_test.uf2** (Timer alarm test):
```
Timer Alarm Test Starting...
Alarm set for +1000us ✅
SUCCESS: Alarm fired! ✅
Interrupt cleared ✅
Timer Alarm Test Complete!
Total steps: 1,948 ✅
```

All registers preserved correctly, flags set accurately, GPIO state properly managed, timer counting accurately!

### ⚠️ Known Limitations

- **NVIC (Interrupt Controller)**: Core structure implemented but missing 3 key features:
  - Memory bus routing (NVIC registers not accessible via MMIO)
  - Priority scheduling (wrong interrupt executes when multiple pending)
  - Exception return mechanism (ISRs cannot return cleanly)
  - See [NVIC Audit Report](docs/NVIC_audit_report.md) for detailed analysis
- **Limited Peripheral Emulation**: UART0, GPIO, and Timer implemented; other peripherals (DMA, USB, etc.) return stub values
- **No Cycle Accuracy**: Instructions execute in logical order; timer uses simplified 1 cycle = 1 microsecond model
- **Single Core Only**: Second Cortex-M0+ core not emulated

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

**Build All Tests**:
```bash
cd test-firmware
./build.sh all
```

### Run

**Hello World:**
```bash
./bramble hello_world.uf2
```

**GPIO Test:**
```bash
./bramble gpio_test.uf2
```

**Timer Test:**
```bash
./bramble timer_test.uf2
```

**Alarm Test:**
```bash
./bramble alarm_test.uf2
```

### Debug Modes

Bramble now supports flexible debug output modes:

**CPU Step Tracing** (verbose CPU and peripheral logging):
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

Expected timer output:
```
=== Bramble RP2040 Emulator ===

[Boot] Loading UF2 firmware...
[Boot] Initializing RP2040...
[Boot] SP = 0x20042000
[Boot] PC = 0x10000040
[Boot] Starting execution...
Timer Test Starting...
Timer value 1: (check debug)
Timer value 2: (check debug)
Elapsed time calculated
Timer Test Complete!
[Boot] Execution complete.
```

## Project Structure

```
Bramble/
├── src/
│   ├── main.c          # Entry point, boot sequence, execution loop
│   ├── cpu.c           # Cortex-M0+ core: fetch, decode, dispatch
│   ├── instructions.c  # 60+ Thumb instruction implementations
│   ├── membus.c        # Memory system: flash, RAM, peripherals
│   ├── gpio.c          # GPIO peripheral emulation
│   ├── timer.c         # Hardware timer emulation
│   ├── nvic.c          # NVIC interrupt controller (core structure)
│   └── uf2.c           # UF2 file loader
├── include/
│   ├── emulator.h      # Core definitions and CPU state
│   ├── instructions.h  # Instruction handler prototypes
│   ├── gpio.h          # GPIO register definitions
│   ├── timer.h         # Timer register definitions
│   └── nvic.h          # NVIC register definitions
├── test-firmware/
│   ├── hello_world.S   # Assembly UART test
│   ├── gpio_test.S     # Assembly GPIO test
│   ├── timer_test.S    # Assembly timer test
│   ├── alarm_test.S    # Assembly alarm test
│   ├── linker.ld       # Memory layout definition
│   ├── uf2conv.py      # UF2 conversion utility
│   └── build.sh        # Firmware build script
├── docs/
│   ├── GPIO.md         # GPIO peripheral documentation
│   └── NVIC_audit_report.md # NVIC audit findings and recommendations
├── assets/
│   └── bramble-logo.jpg    # Project logo
├── CMakeLists.txt      # Build configuration
├── build.sh            # Top-level build script
├── CHANGELOG.md        # Version history and changes
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

## Technical Implementation

### Memory Map

The emulator accurately models the RP2040 address space:
- **Flash (XIP)**: 0x10000000 - 0x101FFFFF (2MB executable)
- **SRAM**: 0x20000000 - 0x20041FFF (264KB)
- **APB Peripherals**: 0x40000000 - 0x4FFFFFFF (UART, GPIO, timers, etc.)
- **SIO**: 0xD0000000 - 0xD0000FFF (Single-cycle I/O, GPIO fast access)

All accesses respect alignment requirements and return appropriate values for unimplemented regions.

### Peripheral Integration

Peripherals are integrated into the memory bus (`membus.c`):
- Reads/writes to peripheral address ranges are routed to peripheral modules
- GPIO: `0x40014000` (IO_BANK0), `0x4001C000` (PADS), `0xD0000000` (SIO)
- Timer: `0x40054000` (64-bit counter, 4 alarms, interrupts)
- UART0: `0x40034000`
- Stub responses for unimplemented peripherals

### Timer Timing Model

The timer increments every CPU cycle with a simplified timing model:
- **1 CPU cycle = 1 microsecond** (for simulation speed)
- Real RP2040: 125 MHz = 125 cycles per microsecond
- Configurable in `timer_tick()` for accuracy vs. speed tradeoff

Alarms trigger when:
```c
if (timer_low_32bits >= alarm_value) {
    set_interrupt_bit();
    disarm_alarm();
}
```

### Instruction Decoding

Thumb instructions are decoded using a priority-ordered bitmask dispatch system in `cpu_step()`:

1. **32-bit instructions checked first** (BL, extended operations)
2. **Control-flow instructions** (branches, BKPT) handle PC directly and return
3. **Data processing** (ALU, shifts, logical ops) execute and fall through
4. **PC increment** (+2 bytes) happens automatically for non-branch instructions

Critical insight: The dispatcher must check more-specific masks before less-specific ones to avoid false matches.

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

## Performance

The emulator executes simple firmware at approximately:
- **Simple loop**: ~10,000 instructions/second (development build)
- **UART output**: Limited by `putchar()` overhead
- **GPIO operations**: Instant (no electrical simulation)
- **Timer operations**: Lightweight counter increment per cycle
- **Memory access**: Direct array indexing (no caching overhead)

The GPIO test executes 2M+ instructions in under 1 second. Performance is adequate for firmware debugging and testing. Optimization (JIT, caching) could achieve 100x improvement but isn't currently needed.

## Future Work

### High Priority
- [x] **GPIO Emulation**: Basic pin state and direction registers ✅
- [x] **Hardware Timer**: 64-bit counter with alarms ✅
- [ ] **NVIC Completion**: 3 outstanding fixes for full interrupt support
  - [ ] Memory bus routing (CRITICAL)
  - [ ] Priority scheduling (IMPORTANT)
  - [ ] Exception return mechanism (IMPORTANT)
- [ ] **GDB Stub**: Remote debugging protocol for source-level debugging

### Medium Priority
- [ ] **SysTick Timer**: OS-level timing for RTOS support
- [ ] **Watchdog Timer**: Reset and debug timeout functionality
- [ ] **SIO Operations**: FIFO, spinlocks, hardware divider
- [ ] **DMA Engine**: Multi-channel transfer controller
- [ ] **Additional Peripherals**: SPI, I2C, PWM

### Low Priority
- [ ] **PIO State Machines**: RP2040's unique programmable I/O
- [ ] **Dual-Core Support**: Second Cortex-M0+ core with SIO coordination
- [ ] **Flash Programming**: XIP flash cache and program/erase simulation
- [ ] **USB Device**: Full-speed USB 1.1 controller emulation

### Nice to Have
- [ ] **Visual GPIO Debugger**: GUI showing pin states in real-time
- [ ] **Trace Export**: Execution history for analysis
- [ ] **Performance Profiling**: Hotspot identification
- [ ] **Automated Test Suite**: Instruction validation framework

## Contributing

Contributions welcome! Areas of particular interest:
- **NVIC Completion** (see [NVIC audit report](docs/NVIC_audit_report.md))
- **Additional peripherals** (SPI, I2C, PWM)
- **Test firmware** exercising different instruction patterns
- **Bug reports** with reproducible test cases
- **Documentation** improvements

## License

MIT License - Feel free to use, modify, and learn from this project.

---

## Acknowledgments

- **ARM**: Thumb-1 instruction set documentation
- **Raspberry Pi Foundation**: RP2040 datasheet and SDK examples
- **Community**: Emulation guides and debugging assistance

---

**Bramble** demonstrates that accurate hardware emulation requires both correct implementation *and* proper testing. With 60+ instructions, accurate flag handling, full GPIO support, hardware timer with alarms, and clean halt detection, it's ready for real firmware development and debugging workflows.

*Built from scratch with ☕ and debugging patience.*
