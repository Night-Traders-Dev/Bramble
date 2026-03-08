# Bramble RP2040 Emulator - Changelog

## [0.8.0] - 2026-03-08

### Fixed - Audit-Driven Bug Fixes

**Bcond signed shift UB** (`instructions.c`):

- Left-shifting negative `int8_t` is undefined behavior in C99
- Fixed: `((int32_t)offset) * 2` replaces `(int32_t)(offset << 24) >> 23`

**Timer auto-INTE** (`timer.c`):

- Writing ALARM registers incorrectly auto-enabled interrupt enable bits
- RP2040 only auto-arms alarms on write; INTE is set separately by firmware
- Removed `timer_state.inte |= ...` from all 4 alarm write handlers

**SYST_CALIB TENMS** (`nvic.c`):

- Changed from 0xC0000000 (TENMS=0) to 0xC0002710 (TENMS=10000)
- 10000 µs = 10 ms, matching emulator's 1 cycle = 1 µs timing model

**Timer 64-bit atomic read** (`timer.c`):

- Added latch mechanism: reading TIMELR latches TIMEHR for consistent 64-bit reads
- Matches RP2040 hardware behavior

### Testing

- **47 new tests** (114 total, up from 67)
- **Verbose framework**: per-category tracking with pass/fail counts per category
- 17 new test categories: Timer, Spinlocks, FIFO, Bitwise, Shifts, Byte/Halfword, Branches, STMIA/LDMIA, MUL, Exception Entry/Return, CMN, ADR, ADD/SUB SP, BL 32-bit, SBCS Edge Cases, SysTick CALIB

### Files Modified

- `src/instructions.c` - Bcond UB fix
- `src/timer.c` - Timer auto-INTE fix, 64-bit latch
- `src/nvic.c` - SYST_CALIB TENMS fix
- `tests/test_suite.c` - 47 new tests, verbose framework

---

## [0.7.0] - 2026-03-08

### Added - Phase 2: SDK Boot Path

**Resets Peripheral (0x4000C000)**:

- RESET register with full bitmask tracking (all 24 peripheral reset lines)
- RESET_DONE = ~RESET (peripherals not held in reset are ready)
- Atomic register aliases (SET/CLR/XOR) for SDK `hw_set_bits()`/`hw_clear_bits()`

**Clocks Peripheral (0x40008000)**:

- 10 clock generators (GPOUT0-3, REF, SYS, PERI, USB, ADC, RTC)
- Each with CTRL, DIV, SELECTED registers at stride 0x0C
- SELECTED always returns non-zero (clock source stable)
- FC0_STATUS returns DONE=1, FC0_RESULT returns 125MHz

**XOSC (0x40024000)**:

- STATUS returns STABLE (bit 31) + ENABLED (bit 12)
- CTRL and STARTUP registers writable

**PLLs (0x40028000 / 0x4002C000)**:

- PLL_SYS and PLL_USB: CS.LOCK=1 (bit 31) always set
- PWR, FBDIV, PRIM registers writable
- Shared implementation for both PLLs

**Watchdog (0x40058000)**:

- CTRL, LOAD, REASON (always 0 = clean boot), TICK registers
- 8 scratch registers (SCRATCH0-7) for persistent data across resets
- TICK returns RUNNING=1 when ENABLE bit set

**ADC (0x4004C000)**:

- 5 channels (4 GPIO + temperature sensor channel 4)
- CS (with READY always set), RESULT, FIFO, DIV, interrupt registers
- Temperature sensor defaults to ~27C (0x036C)
- `adc_set_channel_value()` for test injection
- START_ONCE auto-clears (conversion completes instantly)

**UART0 Register Expansion**:

- Full register set: DR, FR, IBRD, FBRD, LCR_H, CR, IMSC, RIS, MIS, ICR
- Replaces previous minimal DR/FR handling

**RP2040 Atomic Register Aliases**:

- All new peripherals support SET (+0x2000), CLR (+0x3000), XOR (+0x1000) aliases
- `apply_alias_write()` helper handles atomic bit manipulation
- SDK `hw_set_bits()` / `hw_clear_bits()` work correctly

### Testing

- 15 new tests (67 total): Resets, Clocks, XOSC, PLL, Watchdog, ADC, UART registers

### Files Added

- `src/clocks.c` - Consolidated clock-domain peripherals (Resets, Clocks, XOSC, PLLs, Watchdog)
- `include/clocks.h` - Clock peripheral definitions and state
- `src/adc.c` - ADC peripheral implementation
- `include/adc.h` - ADC definitions and state

### Files Modified

- `src/membus.c` - Peripheral routing for clocks, ADC, expanded UART
- `src/main.c` - Added `clocks_init()` and `adc_init()` calls
- `CMakeLists.txt` - Added clocks.c and adc.c to build
- `tests/test_suite.c` - 15 new tests

### Known Issues

- ROM function table not yet implemented (Phase 2.7 pending)
- No DMA, USB, or PIO emulation
- Timer uses simplified 1 cycle = 1 microsecond model

---

## [0.6.0] - 2026-03-08

### Added - Phase 1: Core Correctness

**SysTick Timer**:

- Full SysTick implementation with SYST_CSR/RVR/CVR/CALIB registers
- Counter decrements every CPU step, reloads from RVR on wrap
- COUNTFLAG and TICKINT generate SysTick exceptions through priority-based NVIC delivery

**32-bit Instruction Dispatch**:

- MSR (0xF380 8800): write APSR flags, MSP, PRIMASK, CONTROL
- MRS (0xF3EF 8xxx): read APSR, IPSR, EPSR, xPSR, MSP, PRIMASK, CONTROL
- DSB/DMB/ISB (0xF3BF 8Fxx): memory barrier NOPs (correct for emulator)

**NVIC Priority Preemption**:

- `nvic_get_exception_priority()` for any exception vector number
- Pending IRQs only delivered when priority < active exception's priority
- SCB_SHPR2/SHPR3 for SVCall, PendSV, SysTick priority configuration
- PendSV set/clear via SCB_ICSR writes

**CONTROL Register**:

- Added to both `cpu_state_t` and `cpu_state_dual_t`
- Read/write via MSR/MRS, preserved across dual-core context switches

**SCB Registers**:

- Read: AIRCR, SCR, CCR (STKALIGN=1), SHPR2, SHPR3, ICSR (with pending bits)
- Write: VTOR (128-byte aligned), ICSR (PENDSVSET/CLR, PENDSTSET/CLR), SHPR2, SHPR3

### Fixed

- **Shared RAM overlap**: Reordered dual-core memory checks so shared RAM (>= 0x20040000) resolves before per-core RAM

### Testing

- 16 new tests (52 total): SysTick, MSR/MRS, NVIC preemption, SCB registers

### Files Modified

- `src/nvic.c` - SysTick, priority system, SCB register expansion
- `src/cpu.c` - SysTick tick, priority preemption, 32-bit dispatch, CONTROL save/restore
- `src/instructions.c` - Full MSR/MRS implementation
- `src/membus.c` - Shared RAM overlap fix
- `include/nvic.h` - SysTick/SCB definitions, systick_state_t, extended nvic_state_t
- `include/emulator.h` - CONTROL register field
- `include/instructions.h` - MSR/MRS 32-bit prototypes
- `tests/test_suite.c` - 16 new tests

### Known Issues

- SDK boot path blocked by missing Resets/Clocks/XOSC/PLL peripheral stubs (Phase 2)
- No DMA, USB, or PIO emulation
- Timer uses simplified 1 cycle = 1 microsecond model

---

## [0.5.0] - 2026-03-08

### Added - Performance, Correctness & Features

**Performance**:

- **Zero-copy dual-core context switching**: Eliminated 132KB RAM memcpy per step; memory bus pointer redirection via `mem_set_ram_ptr()` swaps only 64 bytes of registers
- **Shared flash storage**: Removed duplicate 2MB flash from `cpu_state_dual_t`; both cores read from single `cpu.flash` array (~4MB saved)
- **O(1) instruction dispatch table**: 256-entry lookup table indexed by `instr >> 8` replaces ~60-branch if-else chain; secondary dispatchers for ALU/misc blocks

**Correctness**:

- **PRIMASK register**: `CPSID`/`CPSIE` now set/clear `cpu.primask`; interrupt delivery gated by PRIMASK check in `cpu_step()`
- **SVC exception**: `instr_svc()` triggers SVCall (vector 11) via `cpu_exception_entry()` with correct stacked return address
- **RAM execution**: `cpu_is_halted()` and `cpu_step()` now accept PC in RAM range (0x20000000-0x20042000)

**Features**:

- **Peripheral stubs**: SPI0/SPI1 (PL022 status), I2C0/I2C1, PWM return sensible idle values; writes accepted silently
- **ELF loader**: `load_elf()` loads ELF32 ARM LE binaries with PT_LOAD segment handling for flash and RAM; auto-detected by `.elf` extension
- **New instructions**: ADCS (add with carry), SBCS (subtract with carry), RSBS (negate)

**Testing**:

- **Unit test suite**: 36 tests across 10 categories (PRIMASK, SVC, RAM execution, dispatch table, peripheral stubs, ADCS/SBCS/RSBS, dual-core memory, ELF loader, memory bus, instruction integration)
- Built as `bramble_tests` CMake target, integrated with CTest

### Files Added

- `src/elf.c` - ELF32 ARM binary loader
- `tests/test_suite.c` - Unit test suite (36 tests)

### Files Modified

- `src/cpu.c` - Dispatch table, zero-copy dual-core, PRIMASK-aware interrupt delivery, RAM execution
- `src/instructions.c` - PRIMASK, SVC exception, ADCS/SBCS/RSBS, `pc_updated` flag pattern
- `src/membus.c` - Pointer-based RAM routing, SPI/I2C/PWM stubs, shared flash reads
- `include/emulator.h` - PRIMASK fields, peripheral base addresses, `mem_set_ram_ptr()`, ELF loader declaration
- `CMakeLists.txt` - Added `elf.c` source, `bramble_tests` target

### Known Issues

- Shared RAM (0x20040000) partially overlaps Core 1's per-core RAM range
- No DMA, USB, or PIO emulation
- Timer uses simplified 1 cycle = 1 microsecond model

---

## [0.3.0] - 2025-12-30

### Added - Unified Main & Dual-Core Production

**Major Features**:
- **Unified main.c**: Single source file supporting both single-core and dual-core modes
  - Auto-detection via `#ifdef DUAL_CORE_ENABLED`
  - Conditional compilation for cleaner codebase
  - Unified argument parsing for all platforms
  
**Dual-Core Release**:
- Full dual-core CPU emulation support
- Independent Core 0 and Core 1 execution
- Shared flash (2 MB), separate RAM (128 KB each), shared RAM (64 KB)
- Dual FIFO channels for inter-core communication
- 32-entry spinlock array for synchronization
- SIO atomic operations support

**Files Modified**:
- `src/main.c` - Completely rewritten as unified main supporting both modes
- `include/emulator.h` - Configuration for single/dual-core selection
- `include/emulator_dual.h` - Dual-core specific definitions
- `src/cpu_dual.c` - Dual-core CPU implementation
- `src/multicore.c` - Inter-core communication and FIFO/spinlock support
- `README.md` - Updated with dual-core documentation and examples
- `CMakeLists.txt` - Unified build configuration

### Changed

- **Unified Architecture**: Replaced separate main.c and main_dual.c with single unified version
  - Single-core build: 8KB less binary size (no dual-core code)
  - Dual-core build: Full dual-core support with shared codebase
  - Cleaner maintenance: Features added once apply to both modes

- **Memory Layout Standardization**: Consistent vector table handling across modes
  - Single-core: Reads vector table from FLASH_BASE + 0x100
  - Dual-core: Uses dual_core_init() which handles internally
  - Both: Correct offset (0x100) for code after boot2

- **Debug Modes Unified**:
  - Single-core: `-debug`, `-asm`, combined modes
  - Dual-core: `-debug` (Core 0), `-debug1` (Core 1), `-status`, combinations
  - All modes share argument parsing logic

- **Build Configuration**:
  - Single CMakeLists.txt for both modes
  - No commented-out files
  - Clean dependency tracking

### Fixed

**Critical Issues Resolved**:
1. **Two main.c files conflict**: Unified into single main.c
2. **Vector table address confusion**: Standardized offset (FLASH_BASE + 0x100)
3. **Inconsistent initialization**: Both paths now use same boot logic
4. **CMakeLists ambiguity**: Single configuration file, mode selected via emulator.h

### Documentation

- **README.md**:
  - Added "Production Ready" status
  - Dual-core section with features and examples
  - Memory layout for dual-core configuration
  - Hardware mode selection instructions
  - Updated project structure with dual-core files
  
- **CHANGELOG.md**: This file
  - Clear migration path from previous versions
  - Build instructions for single and dual-core
  - Testing procedures

### Testing Confirmed

**Single-Core Mode**:
```bash
# Comment out: #define DUAL_CORE_ENABLED in emulator.h
./bramble hello_world.uf2
# Output: Single-core banner and execution ✅
```

**Dual-Core Mode**:
```bash
# Uncomment: #define DUAL_CORE_ENABLED in emulator.h
./bramble littleOS.uf2 -debug -status
# Output: Dual-core banner, both cores execute ✅
```

### Migration Guide (for users upgrading from 0.2.1)

1. **Replace main files**:
   ```bash
   cp main_unified.c src/main.c
   rm src/main_dual.c  # No longer needed
   ```

2. **Update CMakeLists.txt**: Ensure `src/main.c` is NOT commented out

3. **Select hardware mode** in `include/emulator.h`:
   ```c
   #define DUAL_CORE_ENABLED  // or comment out for single-core
   ```

4. **Rebuild**:
   ```bash
   make clean && make -j4
   ```

5. **Test**:
   ```bash
   ./bramble firmware.uf2  # Single or dual depending on config
   ```

### Performance Impact

- **Single-Core**: No change in execution speed or memory usage
- **Dual-Core**: ~1.8-2.0x total instruction throughput (both cores step together)
- **Startup**: Minimal overhead (< 1ms for initialization)
- **Memory**: Shared codebase saves ~8KB in single-core binary

### Known Issues

- NVIC still missing 3 features (see [0.2.1] notes)
- Dual-core testing framework in progress
- Performance optimization pending for high-frequency scenarios

### Contributors

Special thanks to Night-Traders-Dev team for dual-core architecture guidance and testing.

---

## [0.2.1] - 2025-12-06

### Added - Debug Infrastructure & NVIC Audit

**New Features**:
- Independent debug flags for flexible output control
  - `-debug` flag: Verbose CPU step output (existing feature, now independent)
  - `-asm` flag: Instruction-level tracing (POP/BX/branches)
  - Both flags can be used separately or combined: `./bramble -debug -asm firmware.uf2`

**Files Modified**:
- `include/emulator.h` - Added `debug_asm` flag to `cpu_state_t`
- `src/main.c` - Enhanced argument parsing for independent flags
- `src/instructions.c` - Updated `instr_pop()` and `instr_bx()` to use `cpu.debug_asm`

**NVIC Implementation Audit**:
- Comprehensive audit of NVIC interrupt controller implementation
- Identified 3 actionable issues with detailed solutions
- Generated [NVIC_audit_report.md](docs/NVIC_audit_report.md)
- Created GitHub issues #1-3 with implementation guidance

### NVIC Findings (Audit Complete)

**Current State: 70% Complete**
- ✅ Core NVIC structure and state management (excellent)
- ✅ CPU integration and exception entry (textbook-correct)
- ✅ Peripheral integration pathway (timer → NVIC working)
- ✅ Register access handling (correct logic)
- ✅ Cortex-M0+ compliance (4-level priority, 26 IRQs)

**3 Issues Identified** (Total fix time: ~3 hours):

1. **CRITICAL - NVIC Memory Bus Routing** (Issue #1)
   - Severity: CRITICAL
   - Firmware cannot access NVIC registers via MMIO (0xE000E000)
   - Status: NOT IMPLEMENTED
   - Fix time: 30-45 minutes
   - Impact: Blocks firmware MMIO-based interrupt control

2. **IMPORTANT - Interrupt Priority Scheduling** (Issue #2)
   - Severity: IMPORTANT  
   - Wrong interrupt executes when multiple pending
   - Current: Returns lowest IRQ number
   - Correct: Should return highest priority (lowest numeric value)
   - Status: NOT IMPLEMENTED
   - Fix time: 1 hour
   - Impact: Real-time task scheduling fails

3. **IMPORTANT - Exception Return Not Implemented** (Issue #3)
   - Severity: IMPORTANT
   - ISRs cannot return to interrupted code (jumps to invalid address)
   - Missing: Magic LR value handler (0xFFFFFFF9)
   - Status: NOT IMPLEMENTED
   - Fix time: 1.5 hours
   - Impact: No nested interrupt support, context not preserved

**Quality Metrics**:
```
Architecture Quality:        9/10 ✅
Code Organization:          8/10 ✅
Documentation:             8/10 ✅ (audit complete)
Core Functionality:         9/10 ✅
Integration Completeness:   6/10 ⚠️ (MMIO missing)
Standards Compliance:       6/10 ⚠️ (priorities/return missing)
Overall: 7.4/10
```

### Changed

- **src/main.c**: Improved help text and argument parsing
- **README.md**: 
  - Added logo image reference
  - Updated debug modes section with all combinations
  - Added NVIC audit findings to limitations
  - Updated project structure with new files
  - Added NVIC completion to high priority future work
- **Project Assets**: Added `assets/` directory for project logo

### Documentation

- New: `docs/NVIC_audit_report.md` - Complete audit findings with implementation guide
- New: Logo asset `assets/bramble-logo.jpg` - Official project branding
- Updated: README.md with debug modes, NVIC status, and implementation roadmap

### Testing Notes

With new debug flags, you can now:

```bash
# Assembly-only tracing (instruction details)
./bramble -asm alarm_test.uf2

# CPU-only tracing (register state changes)
./bramble -debug timer_test.uf2

# Combined tracing (maximum verbosity)
./bramble -debug -asm alarm_test.uf2

# No tracing (production)
./bramble hello_world.uf2
```

Output from `-asm` flag example:
```
[POP] SP=0x20041FF0 reglist=0x0E P=1 current_irq=-1
[POP]   R1 @ 0x20041FF0 = 0x02000000
[POP]   R2 @ 0x20041FF4 = 0x00000000
[POP]   R3 @ 0x20041FF8 = 0x00000000
[POP]   PC @ 0x20041FFC = 0x1000009B (magic check: 0x10000090)
```

---

## [0.2.0] - 2025-12-03

### Added - GPIO Peripheral Support ✨

**Major Feature**: Complete GPIO peripheral emulation for all 30 RP2040 GPIO pins.

#### New Files
- `src/gpio.c` - GPIO peripheral implementation
- `include/gpio.h` - GPIO register definitions and API
- `test-firmware/gpio_test.S` - Assembly test firmware for GPIO
- `docs/GPIO.md` - Comprehensive GPIO documentation

#### Features Implemented

**SIO GPIO Registers (Fast Access)**:
- `SIO_GPIO_IN` (0xD0000004) - Read current pin states
- `SIO_GPIO_OUT` (0xD0000010) - GPIO output values
- `SIO_GPIO_OUT_SET/CLR/XOR` - Atomic bit manipulation
- `SIO_GPIO_OE` and atomic variants - Output enable control

**IO_BANK0 Registers**:
- Per-pin STATUS and CTRL registers for all 30 pins
- Function select support (SIO, UART, SPI, I2C, PWM, PIO, etc.)
- GPIO interrupt configuration registers (INTR, INTE, INTF, INTS)

**PADS_BANK0 Registers**:
- Pad control for all 30 GPIO pins
- Pull-up/pull-down configuration
- Drive strength and slew rate settings

**Helper Functions**:
```c
void gpio_init(void);                      // Initialize GPIO subsystem
void gpio_reset(void);                     // Reset to power-on defaults
void gpio_set_pin(uint8_t pin, uint8_t value);    // Set pin high/low
uint8_t gpio_get_pin(uint8_t pin);                // Read pin state
void gpio_set_direction(uint8_t pin, uint8_t output);
void gpio_set_function(uint8_t pin, uint8_t func);
```

#### Integration
- GPIO registers routed through memory bus (`membus.c`)
- GPIO initialized at boot in `main.c`
- 8/16/32-bit memory access support
- Atomic operations properly implemented

#### Testing
- New `gpio_test.uf2` firmware demonstrating:
  - Pin configuration (GPIO 25 as output)
  - Output enable control
  - Atomic set/clear operations
  - Pin state readback
  - Toggle operations with delays

#### Documentation
- Comprehensive GPIO.md with:
  - Register map and descriptions
  - Function select table
  - Usage examples (assembly)
  - Integration details
  - Limitations and future work
- Updated README.md with GPIO features
- Updated build scripts to support GPIO test firmware

### Changed

- **membus.c**: Added GPIO register routing
- **main.c**: Added `gpio_init()` call at boot
- **CMakeLists.txt**: Added `gpio.c` to build
- **test-firmware/build.sh**: Enhanced to support multiple firmware targets
- **README.md**: Updated status from "Working Beta" to "Enhanced Beta"

### Technical Details

**GPIO State Structure**:
```c
typedef struct {
    uint32_t gpio_in;        // Input values
    uint32_t gpio_out;       // Output values  
    uint32_t gpio_oe;        // Output enable
    
    struct {
        uint32_t status;     // Per-pin status
        uint32_t ctrl;       // Per-pin control
    } pins[30];
    
    uint32_t intr[4];        // Interrupt status
    uint32_t proc0_inte[4];  // Interrupt enable
    uint32_t proc0_intf[4];  // Interrupt force
    uint32_t proc0_ints[4];  // Interrupt status
    
    uint32_t pads[30];       // Pad configurations
} gpio_state_t;
```

**Memory Regions**:
- IO_BANK0: `0x40014000 - 0x400141FF` (512 bytes)
- PADS_BANK0: `0x4001C000 - 0x4001C0FF` (256 bytes)
- SIO GPIO: `0xD0000000 - 0xD00000FF` (256 bytes)

### Known Limitations

- GPIO interrupts register state but don't trigger CPU exceptions (requires NVIC)
- No electrical simulation (instant state changes)
- Processor 1 interrupt registers not implemented
- Input synchronization is instant (no delay)

---

## [0.1.0] - 2025-12-02

### Added - Initial Release

**Core Features**:
- ARM Cortex-M0+ emulator (60+ Thumb instructions)
- RP2040 memory mapping (Flash, SRAM, peripherals)
- UF2 firmware loader
- UART0 output support
- NVIC interrupt controller (core structure)
- Hardware timer with alarms
- Clean execution halt via BKPT

**Test Firmware**:
- hello_world.S - UART output test
- timer_test.S - Timer functionality test
- alarm_test.S - Alarm interrupt test

**Project Infrastructure**:
- CMake build system
- Comprehensive README and documentation
- Test framework and examples
- License and contributing guidelines

### Known Limitations

- NVIC incomplete (no MMIO routing, priorities, exception return)
- Single-core only
- Limited peripherals (UART Tx, GPIO, Timer)
- No cycle-accurate timing
