# Bramble RP2040 Emulator

A from-scratch ARM Cortex-M0+ emulator for the Raspberry Pi RP2040 microcontroller, capable of loading and executing UF2 firmware with accurate memory mapping and peripheral emulation.

## Current Status: **Enhanced Beta** ✅

Bramble successfully boots RP2040 firmware, executes the complete Thumb-1 instruction set, provides working UART0 output, and **now includes full GPIO emulation**! The emulator cleanly executes test programs with proper halting via BKPT instructions.

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
- **✨ GPIO Emulation (NEW!)**: Complete 30-pin GPIO peripheral with:
  - SIO fast GPIO access (direct read/write, atomic operations)
  - IO_BANK0 per-pin configuration (function select, control)
  - PADS_BANK0 pad control (pull-up/down, drive strength)
  - Interrupt registers (enable, force, status)
  - All 10 GPIO functions supported (SIO, UART, SPI, I2C, PWM, PIO, etc.)
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
```

All registers preserved correctly, flags set accurately, GPIO state properly managed!

### ⚠️ Known Limitations

- **Limited Peripheral Emulation**: UART0 and GPIO are implemented; other peripherals (timers, DMA, etc.) return stub values
- **No Interrupt Support**: NVIC and interrupt handling are not yet implemented (GPIO interrupt registers exist but don't trigger CPU)
- **No Cycle Accuracy**: Instructions execute in logical order without timing simulation
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

Expected GPIO output:
```
=== Bramble RP2040 Emulator ===

[Boot] Loading UF2 firmware...
[Boot] Initializing RP2040...
[Boot] SP = 0x20042000
[Boot] PC = 0x10000040
[Boot] Starting execution...
GPIO Test Starting...
GPIO 25 configured as output
LED ON
LED OFF
...
GPIO Test Complete!
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
│   ├── gpio.c          # GPIO peripheral emulation (NEW!)
│   └── uf2.c           # UF2 file loader
├── include/
│   ├── emulator.h      # Core definitions and CPU state
│   ├── instructions.h  # Instruction handler prototypes
│   └── gpio.h          # GPIO register definitions (NEW!)
├── test-firmware/
│   ├── hello_world.S   # Assembly UART test
│   ├── gpio_test.S     # Assembly GPIO test (NEW!)
│   ├── linker.ld       # Memory layout definition
│   ├── uf2conv.py      # UF2 conversion utility
│   └── build.sh        # Firmware build script
├── docs/
│   └── GPIO.md         # GPIO peripheral documentation (NEW!)
├── CMakeLists.txt      # Build configuration
├── build.sh            # Top-level build script
└── README.md           # This file
```

## GPIO Peripheral ✨

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
- UART0: `0x40034000`
- Stub responses for unimplemented peripherals

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
- **Simple loop**: ~5,000 instructions/second (development build)
- **UART output**: Limited by `putchar()` overhead
- **GPIO operations**: Instant (no electrical simulation)
- **Memory access**: Direct array indexing (no caching overhead)

Performance is adequate for firmware debugging and testing. Optimization (JIT, caching) could achieve 100x improvement but isn't currently needed.

## Future Work

### High Priority
- [x] **GPIO Emulation**: Basic pin state and direction registers ✅
- [ ] **Timer Peripherals**: TIMER0-3 with alarm support
- [ ] **Interrupt Support**: NVIC, exception entry/exit, priority handling
- [ ] **GDB Stub**: Remote debugging protocol for source-level debugging

### Medium Priority
- [ ] **Cycle Counting**: Accurate timing for peripheral synchronization
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
- **Peripheral implementations** (timers, I2C, SPI, PWM)
- **Test firmware** exercising different instruction patterns
- **Bug reports** with reproducible test cases
- **Documentation** improvements

## License

MIT License - Feel free to use, modify, and learn from this project.

***

## Acknowledgments

- **ARM**: Thumb-1 instruction set documentation
- **Raspberry Pi Foundation**: RP2040 datasheet and SDK examples
- **Community**: Emulation guides and debugging assistance

***

**Bramble** demonstrates that accurate hardware emulation requires both correct implementation *and* proper testing. With 60+ instructions, accurate flag handling, full GPIO support, and clean halt detection, it's ready for real firmware development and debugging workflows.

*Built from scratch with ☕ and debugging patience.*
