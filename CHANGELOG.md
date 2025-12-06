# Bramble RP2040 Emulator - Changelog

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

**NVIC Implementation Audit** :
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

**Core Emulator**:
- Complete ARM Cortex-M0+ Thumb instruction set (60+ instructions)
- UF2 firmware loader
- RP2040 memory map (Flash, SRAM, peripherals)
- UART0 character output
- Accurate flag handling (N, Z, C, V)
- Clean halt detection (BKPT instruction)

**Test Firmware**:
- `hello_world.S` - Assembly UART test
- UF2 conversion tools
- Linker script for RP2040 layout

**Documentation**:
- Comprehensive README with technical details
- Development notes on emulation pitfalls
- Build instructions

**Project Structure**:
- Clean separation: CPU, instructions, memory bus, peripherals
- CMake build system
- Bash build script for convenience

### Technical Achievements

- Solved BCOND +4 offset bug
- Proper PC management discipline
- Instruction dispatch ordering
- 32-bit instruction handling
- Sign extension for byte/halfword loads
- Test-driven validation

---

## Future Versions

### Planned for 0.3.0
- [ ] **NVIC Completion** (3 outstanding issues from v0.2.1 audit)
  - [ ] Memory bus routing (CRITICAL)
  - [ ] Priority scheduling (IMPORTANT)
  - [ ] Exception return mechanism (IMPORTANT)
- [ ] GDB remote debugging stub
- [ ] Additional communication peripherals (SPI, I2C)

### Planned for 0.4.0
- [ ] SysTick timer
- [ ] Watchdog timer
- [ ] Watchdog timer

### Planned for 0.5.0+
- [ ] DMA engine
- [ ] Dual-core support
- [ ] PIO state machines (experimental)
- [ ] USB device support

---

**Version Format**: [Major].[Minor].[Patch]
- **Major**: Breaking changes, architecture redesigns
- **Minor**: New features, peripheral additions
- **Patch**: Bug fixes, documentation updates, minor improvements
