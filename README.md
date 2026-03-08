# Bramble RP2040 Emulator

A from-scratch ARM Cortex-M0+ emulator for the Raspberry Pi RP2040 microcontroller, capable of loading and executing UF2 and ELF firmware with accurate memory mapping and peripheral emulation.

## Current Status: **v0.8.0 - Production Ready** ✅

Bramble successfully boots RP2040 firmware, executes the complete Thumb-1 instruction set with O(1) dispatch, provides working UART0 output, full GPIO emulation, hardware timer support with alarms, **SysTick timer**, **SDK boot peripherals** (Resets, Clocks, XOSC, PLLs, Watchdog), **ADC with temperature sensor**, SPI/I2C/PWM peripheral stubs, **NVIC priority preemption**, full **MSR/MRS** support, **RP2040 atomic register aliases** (SET/CLR/XOR), PRIMASK interrupt masking, SVC exceptions, RAM execution, and **zero-copy dual-core support**. Includes a **114-test verbose unit test suite** across 30+ categories.

### ✅ What Works

- **Complete RP2040 Memory Map**: Flash (0x10000000), SRAM (0x20000000), SIO (0xD0000000), and APB peripherals
- **UF2 & ELF Firmware Loading**: Parses UF2 blocks or ELF32 ARM binaries into flash/RAM with proper address validation
- **O(1) Instruction Dispatch**: 256-entry lookup table indexed by `instr >> 8` with secondary dispatchers for ALU/misc blocks
- **Full ARM Cortex-M0+ Thumb Instruction Set**: 65+ instructions across 4 phases:

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
  - Carry arithmetic: ADCS, SBCS, RSBS (negate)
  - Multiple load/store: LDMIA, STMIA

  **Phase 3 - Important (Advanced Features):**
  - Special comparison: CMN, TST
  - System: SVC (triggers SVCall exception)
  - 32-bit instructions: MSR/MRS (PRIMASK, CONTROL, xPSR, MSP), DSB/DMB/ISB
  - High register operations
  - PRIMASK support: CPSID/CPSIE control interrupt delivery

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
- **✨ SysTick Timer**: Full ARM SysTick implementation with:
  - SYST_CSR, SYST_RVR, SYST_CVR, SYST_CALIB registers
  - Counter decrements every CPU step, reloads from RVR on wrap
  - COUNTFLAG (bit 16) set on underflow, cleared on CSR read
  - TICKINT generates SysTick exception through NVIC priority system
- **✨ NVIC Priority Preemption**: Proper interrupt priority enforcement:
  - 4 priority levels via bits [7:6] (Cortex-M0+ compliant)
  - Pending IRQs only preempt if strictly higher priority than active exception
  - SCB_SHPR2/SHPR3 for SVCall, PendSV, SysTick priority configuration
  - PendSV set/clear via SCB_ICSR
- **✨ Hardware Timer**: Full 64-bit microsecond timer with:
  - 64-bit counter incrementing with CPU cycles
  - 4 independent alarm channels (ALARM0-3)
  - Interrupt generation on alarm match
  - Pause/resume functionality
  - Write-1-to-clear interrupt handling
  - Cycle-accurate timing simulation
- **✨ Dual-Core Support**: Full second Cortex-M0+ core emulation with:
  - Independent Core 0 and Core 1 state machines
  - Zero-copy context switching via pointer-based memory bus routing
  - Shared flash memory (2 MB, single copy)
  - Separate core-local RAM (132 KB each)
  - Shared RAM (64 KB for inter-core communication)
  - Dual FIFO channels for core-to-core messaging
  - Spinlock support for synchronization
  - SIO (Single-cycle I/O) for atomic operations
- **✨ SDK Boot Peripherals**: Complete peripheral stubs for SDK initialization:
  - Resets (0x4000C000): reset/unreset with RESET_DONE tracking
  - Clocks (0x40008000): 10 clock generators with CTRL/DIV/SELECTED
  - XOSC (0x40024000): STATUS.STABLE=1, STATUS.ENABLED=1
  - PLL_SYS/PLL_USB: CS.LOCK=1 (always locked)
  - Watchdog (0x40058000): CTRL, TICK, SCRATCH[0-7], REASON
  - PSM (Power State Machine): stub
  - RP2040 atomic register aliases: SET (+0x2000), CLR (+0x3000), XOR (+0x1000)
- **✨ ADC**: 5-channel ADC with temperature sensor:
  - CS, RESULT, FIFO, DIV registers
  - Channel 4 = internal temperature sensor (~27C default)
  - Configurable channel values for testing
- **✨ Expanded UART0**: Full register set (DR, FR, IBRD, FBRD, LCR_H, CR, IMSC, RIS, MIS)
- **Peripheral Stubs**: SPI0/SPI1 (PL022 idle status), I2C0/I2C1, PWM return sensible defaults so SDK firmware doesn't crash during init
- **RAM Execution**: PC accepted in RAM range (0x20000000-0x20042000) for flash programming routines and performance-critical code
- **✨ Unit Test Suite**: 114 tests across 30+ categories with verbose per-category reporting, integrated with CTest
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

- **USB CDC**: No USB device emulation - `stdio_usb_connected()` will not work (use UART stdio instead)
- **Limited Peripheral Emulation**: UART0, GPIO, Timer, SysTick, ADC, Resets, Clocks, XOSC, PLLs, Watchdog, SPI/I2C/PWM stubs; DMA, USB, PIO not emulated
- **No Cycle Accuracy**: Instructions execute in logical order; timer uses simplified 1 cycle = 1 microsecond model
- **UART Tx Only**: No receive (Rx) emulation
- See [ROADMAP](docs/ROADMAP.md) for full status and next phases

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
│   └── adc.c           # ADC peripheral emulation
├── include/
│   ├── emulator.h      # Core definitions, CPU state, memory layout
│   ├── instructions.h  # Instruction handler prototypes
│   ├── gpio.h          # GPIO register definitions
│   ├── timer.h         # Timer register definitions
│   ├── nvic.h          # NVIC register definitions
│   ├── clocks.h        # Clock-domain peripheral definitions
│   └── adc.h           # ADC register definitions
├── tests/
│   └── test_suite.c    # Unit test suite (114 tests, verbose, CTest integrated)
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

### High Priority (Phase 3 - Run littleOS)

1. **USB CDC Stub**: Allow `stdio_init_all()` to complete without hanging
2. **Flash Programming**: `flash_range_erase()` / `flash_range_program()` stubs for config persistence
3. **ROM Function Table**: Stub implementations for SDK utility functions (`rom_func_lookup`, `_memcpy4`)

### Medium Priority

1. **UART Rx**: Implement receive path for bidirectional serial communication
2. **Cycle-Accurate Timing**: Replace 1:1 cycle-to-microsecond model with configurable ratio
3. **Debugging Features**: Hardware breakpoints, watchpoints, GDB remote stub
4. **Full Peripheral Emulation**: DMA controller, PIO state machines

### Low Priority

1. **Performance**: JIT compilation for hot loops, instruction caching
2. **Visualization**: Web-based register display, memory map explorer
3. **SRAM Aliasing**: Map 0x21-0x25 ranges with XOR/SET/CLR semantics

## Contributing

The Bramble project is open for contributions! Areas that need help:

1. **Peripheral Emulation**: DMA, USB, PIO state machines, UART Rx
2. **Testing**: New test firmware, edge cases, performance benchmarks
3. **Debugging**: GDB remote stub, hardware breakpoints
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
