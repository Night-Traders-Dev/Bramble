# Bramble RP2040 Emulator - Updates

## [0.11.0] - 2026-03-08

### UART Receive Path

1. **UART Rx FIFO** (`uart.c`, `uart.h`)
   - 16-deep PL011-standard circular buffer per UART instance
   - `uart_rx_push(uart_num, data)` API for external data injection
   - DR read pops from FIFO in order; returns 0 when empty
   - FR register: RXFE/RXFF flags reflect actual FIFO occupancy
   - RX interrupt (RIS bit 4) set when FIFO reaches trigger level (IFLS bits [5:3])
   - Interrupt auto-clears when DR reads drain FIFO below trigger
   - Both UART0 and UART1 have independent RX FIFOs

### Test Suite (165 tests, up from 157)

1. **8 new UART Rx tests**:
   - Push/pop via DR, FIFO empty flag, FIFO full flag (rejects on overflow)
   - FIFO ordering (3-byte sequence), RX interrupt at trigger level
   - Interrupt clear on read, UART1 independent Rx, masked interrupt (MIS)

---

## [0.10.0] - 2026-03-08

### Phase 3.5: DMA Controller

1. **DMA Controller** (`dma.c`, `dma.h`)
   - 12 channels at 0x50000000 with 0x40-byte stride per channel
   - Per-channel: READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL_TRIG
   - 4 alias layouts per channel (AL1-AL3 reorder fields; last register in each alias triggers transfer)
   - Immediate synchronous transfer engine using mem_read/write functions
   - DATA_SIZE: byte (1B), halfword (2B), word (4B)
   - INCR_READ / INCR_WRITE for address auto-increment
   - CHAIN_TO: triggers another channel on completion (default = self = no chain)
   - IRQ_QUIET: suppress interrupt flag on completion
   - Global INTR (W1C), INTE0/1, INTF0/1, INTS0/1 interrupt registers
   - MULTI_CHAN_TRIGGER: trigger multiple channels simultaneously
   - Pacing timers (TIMER0-3), SNIFF_CTRL/DATA, CHAN_ABORT, N_CHANNELS
   - Atomic register aliases (SET/CLR/XOR)

### Test Suite (157 tests, up from 145)

1. **12 new DMA Controller tests**:
   - N_CHANNELS readback, channel defaults (CHAIN_TO=self), register readback
   - Word transfer (4 words, verify dst), byte transfer (3 bytes)
   - No-incr-write (last value wins), interrupt on completion (INTR bit set + W1C)
   - IRQ_QUIET (no INTR bit), interrupt status (INTS0 computation)
   - Chain transfer (ch0 chains to ch1), MULTI_CHAN_TRIGGER
   - Atomic SET/CLR aliases on INTE0

---

## [0.9.0] - 2026-03-08

### Phase 2.7: ROM Function Table + Phase 3: Full Peripherals

1. **ROM Function Table** (`rom.c`, `rom.h`)
   - 4KB ROM at 0x00000000 with RP2040-compatible layout (magic, func/data table pointers)
   - Executable Thumb code stubs: `rom_table_lookup`, `memcpy`, `memset`, `popcount32`, `clz32`, `ctz32`, `reverse32`
   - Flash function no-op stubs: connect_internal_flash, flash_exit_xip, flash_range_erase/program, flash_flush_cache, flash_enter_cmd_xip
   - Integrated into membus (ROM reads) and cpu.c (execution from ROM addresses)

2. **USB Controller Stub** (`membus.c`)
   - DPRAM (0x50100000) and REGS (0x50110000) accept reads/writes without crashing
   - Reads return 0 (SIE_STATUS = disconnected); SDK falls back to UART

3. **UART Full Module** (`uart.c`, `uart.h`)
   - Extracted inline UART from membus.c into proper PL011 module
   - Both UART0 (0x40034000) and UART1 (0x40038000) with independent state
   - 12 registers: DR, RSR, IBRD, FBRD, LCR_H, CR, IFLS, IMSC, RIS, MIS, ICR, DMACR
   - TX interrupt status, ICR write-1-to-clear, PL011 peripheral ID
   - Atomic register aliases (SET/CLR/XOR)

4. **SPI Full Module** (`spi.c`, `spi.h`)
   - PL022 module with both SPI0 (0x4003C000) and SPI1 (0x40040000)
   - Register state: CR0, CR1, CPSR, IMSC, RIS, DMACR
   - Status register (TFE, TNF), PL022 peripheral ID
   - Atomic register aliases

5. **I2C Full Module** (`i2c.c`, `i2c.h`)
   - DW_apb_i2c module with both I2C0 (0x40044000) and I2C1 (0x40048000)
   - Full register set: CON, TAR, SAR, SCL timing, ENABLE, STATUS, interrupt, DMA
   - Component ID registers (COMP_PARAM_1, COMP_VERSION, COMP_TYPE)
   - Atomic register aliases

6. **PWM Full Module** (`pwm.c`, `pwm.h`)
   - 8 independent slices with CSR, DIV, CTR, CC, TOP registers
   - Global EN, INTR (W1C), INTE, INTF, INTS registers
   - Atomic register aliases

7. **CMP ALU fix** (`instructions.c`, `cpu.c`)
   - Split ALU-block CMP (3-bit register fields, 0x4280-0x42BF) from high-register CMP (4-bit fields, 0x4500-0x45FF)
   - New `instr_cmp_alu()` handler for correct register decoding

### Test Suite (145 tests, up from 114)

8. **31 new tests across 4 new categories**:
   - ROM Function Table (8): magic, table pointers, func entries, lookup, lookup not-found, popcount, clz, ctz
   - USB Controller Stub (3): regs read zero, dpram read zero, write no crash
   - Flash ROM Functions (1): flash function table entries
   - UART Peripheral (8): output, registers, baud readback, UART1 independent, CR readback, IMSC/ICR, atomic SET/CLR, periph ID
   - SPI Peripheral (4): status, CR0 readback, SPI1 independent, periph ID
   - I2C Peripheral (5): status, TAR readback, I2C1 independent, enable/disable, comp type
   - PWM Peripheral (4): slice defaults, readback, multiple slices, global enable

### Peripheral defines consolidated
- Moved UART, SPI, I2C, PWM base addresses from `emulator.h` to individual peripheral headers
- Peripheral modules are self-contained with their own header + source files

---

## [0.8.0] - 2026-03-08

### Audit & Bug Fixes

1. **Bcond signed shift undefined behavior** (`instructions.c`)
   - `(int32_t)(offset << 24) >> 23` where offset is negative `int8_t` is UB in C99
   - Fixed: `((int32_t)offset) * 2` — well-defined signed multiplication
   - Impact: conditional branches with negative offsets could produce wrong targets

2. **Timer auto-INTE removed** (`timer.c`)
   - Writing ALARM registers incorrectly auto-enabled INTE bits (`timer_state.inte |= 0x1`)
   - RP2040 datasheet: writing ALARM auto-arms, but INTE must be set separately by firmware
   - Fixed: removed INTE auto-enable from all 4 alarm write handlers

3. **SYST_CALIB TENMS field** (`nvic.c`)
   - Was `0xC0000000` (TENMS=0), SDK couldn't determine SysTick calibration
   - Fixed: `0xC0002710` (TENMS=10000) — matches emulator's 1 cycle = 1 µs model

4. **Timer 64-bit atomic read** (`timer.c`)
   - TIMELR and TIMEHR read independently, timer could tick between reads
   - Added `timer_latched_high`: reading TIMELR latches TIMEHR for consistent 64-bit value
   - Matches RP2040 hardware latch behavior

### Test Suite Expansion (114 tests, up from 67)

5. **Verbose test framework** (`tests/test_suite.c`)
   - Per-category tracking with BEGIN_CATEGORY/END_CATEGORY macros
   - Each test shows name and PASS/FAIL result on its own line
   - Category summaries: `-- Category: X/Y passed`
   - Final summary: `Results: 114/114 passed, 0 failed`

6. **47 new tests across 17 new categories**:
   - Timer (5): alarm arm, alarm fire/disarm, 64-bit latch, pause, intr clear
   - Spinlocks (4): acquire free, acquire locked, release, out of range
   - FIFO (4): push/pop, empty check, try pop empty, try push full
   - Bitwise (6): AND, EOR, ORR, BIC, MVN, TST flags
   - Shifts (6): LSR imm32, ASR imm32 (neg/pos), ROR reg, LSLS reg by 0/32
   - Byte/Halfword (7): SXTB, SXTH, UXTB, UXTH, REV, REV16, REVSH
   - Branches (3): bcond negative offset, B unconditional, bcond not taken
   - STMIA/LDMIA (1): round-trip store and load
   - MUL (2): multiply, multiply by zero
   - Exception Entry/Return (1): full stacking/unstacking round-trip
   - CMN (1): compare negative register
   - ADR (1): PC-relative address generation
   - ADD/SUB SP (2): SP immediate add/subtract
   - BL 32-bit (1): branch-with-link encoding
   - SBCS Edge Cases (2): carry flag with/without borrow
   - SysTick CALIB (1): TENMS=10000 verification

---

## [0.7.0] - 2026-03-08

### Phase 2: SDK Boot Path (Mostly Complete)

1. **Resets Peripheral** (`clocks.c`, `clocks.h`)
   - RESET register with full 24-bit peripheral mask
   - RESET_DONE = `~reset & RESETS_ALL_MASK` (anything not in reset is ready)
   - Atomic aliases (SET/CLR/XOR) at +0x1000/+0x2000/+0x3000 offsets
   - SDK `reset_block()` / `unreset_block_wait()` now functional

2. **Clocks Peripheral** (`clocks.c`, `clocks.h`)
   - 10 clock generators with CTRL, DIV, SELECTED registers
   - SELECTED always returns 0x1 (clock source stable)
   - FC0 frequency counter: STATUS.DONE=1, RESULT=125MHz
   - Power-on default: `clk_div[i] = 1 << 8` (divide by 1)

3. **XOSC + PLLs** (`clocks.c`, `clocks.h`)
   - XOSC STATUS returns STABLE | ENABLED immediately
   - PLL_SYS and PLL_USB: CS.LOCK always set (instant lock)
   - PWR/FBDIV/PRIM registers writable for SDK configuration flow

4. **Watchdog** (`clocks.c`, `clocks.h`)
   - REASON register returns 0 (clean boot, no watchdog/force reset)
   - TICK register: returns RUNNING=1 when ENABLE bit set
   - 8 scratch registers for persistent data across resets
   - SDK watchdog tick configuration for SysTick reference clock

5. **ADC** (`adc.c`, `adc.h`)
   - 5 channels: GPIO26-29 (channels 0-3) + temperature sensor (channel 4)
   - CS.READY always set; START_ONCE auto-clears (instant conversion)
   - RESULT returns `channel_values[ainsel]`
   - Temperature sensor defaults to 0x036C (~27°C)
   - `adc_set_channel_value()` API for test injection

6. **UART0 Expansion** (`membus.c`)
   - Full register set: DR, FR, IBRD, FBRD, LCR_H, CR, IMSC, RIS, MIS, ICR
   - FR returns TXE=1, RXFE=1 (transmit empty, receive empty)
   - CR defaults to 0x0300 (TXE | RXE enabled)

7. **Atomic Register Aliases** (`clocks.c`, `adc.c`)
   - All peripherals support XOR (+0x1000), SET (+0x2000), CLR (+0x3000)
   - `apply_alias_write()` helper: determines operation from address bits [13:12]
   - Enables SDK's `hw_set_bits()` / `hw_clear_bits()` / `hw_xor_bits()`

### Test Suite

8. **15 new tests** (67 total, up from 52) (`tests/test_suite.c`)
   - Resets: power-on state, release and done, atomic clear alias
   - Clocks: selected always set, ctrl write/read
   - XOSC: status stable
   - PLL: sys lock, usb lock
   - Watchdog: reason clean boot, scratch registers, tick enable
   - ADC: cs ready, temp sensor, set channel value
   - UART: register reads (FR, CR)

### Known Remaining Issues

- ROM function table not yet implemented (Phase 2.7)
- No DMA, USB, PIO emulation
- UART is Tx only (no Rx data path)
- Timer model is 1 cycle = 1 microsecond (not cycle-accurate)

---

## [0.6.0] - 2026-03-08

### Phase 1: Core Correctness (Complete)

1. **SysTick Timer** (`nvic.c`, `nvic.h`)
   - Full SysTick implementation: SYST_CSR, SYST_RVR, SYST_CVR, SYST_CALIB registers
   - `systick_tick()` called every CPU step; decrements CVR, sets COUNTFLAG on wrap
   - TICKINT (bit 1) pends SysTick exception for priority-based delivery
   - COUNTFLAG (bit 16) cleared on CSR read per ARM spec
   - SDK `sleep_ms()` / `sleep_us()` now unblocked

2. **MSR/MRS 32-bit Instructions** (`instructions.c`, `cpu.c`)
   - Full MSR: write APSR flags (bits [31:28]), MSP (R13), PRIMASK, CONTROL
   - Full MRS: read APSR, IPSR, EPSR, xPSR, MSP, PRIMASK, CONTROL
   - 32-bit dispatch in `cpu_step()`: MSR (0xF380), MRS (0xF3EF), DSB/DMB/ISB (0xF3BF)
   - Note: MOVW/MOVT/LDR.W/STR.W/B.W are NOT valid on M0+ (Thumb-2 only)

3. **NVIC Priority Preemption** (`nvic.c`, `cpu.c`)
   - `nvic_get_exception_priority()` returns effective priority for any vector number
   - Interrupt delivery compares pending priority vs. active exception priority
   - Only delivers if strictly higher priority (lower numeric value)
   - SCB_SHPR2/SHPR3 configure SVCall, PendSV, and SysTick priorities
   - 4 priority levels via bits [7:6] (Cortex-M0+ compliant)

4. **Fixed Shared RAM Overlap** (`membus.c`)
   - Reordered range checks in `mem_read32_dual()` and `mem_write32_dual()`
   - Shared RAM (>= 0x20040000) now checked before per-core RAM
   - Addresses 0x20040000-0x2004FFFF correctly resolve to shared RAM for both cores

5. **CONTROL Register** (`emulator.h`, `cpu.c`)
   - Added `uint32_t control` to both `cpu_state_t` and `cpu_state_dual_t`
   - Preserved across dual-core context switches
   - Read/write via MSR/MRS with SYSm 0x14

6. **SCB Register Expansion** (`nvic.c`)
   - Read: SCB_AIRCR, SCB_SCR, SCB_CCR (STKALIGN=1), SCB_SHPR2, SCB_SHPR3
   - Write: SCB_VTOR (128-byte aligned), SCB_ICSR (PENDSVSET/CLR, PENDSTSET/CLR)
   - ICSR read returns VECTPENDING, ISRPENDING, PENDSVSET, PENDSTSET

### Test Suite

7. **16 new tests** (52 total, up from 36) (`tests/test_suite.c`)
   - SysTick: registers, countdown, disabled-no-count
   - MSR/MRS: PRIMASK, xPSR, APSR flags, CONTROL, 32-bit dispatch (MSR, MRS, DSB)
   - NVIC preemption: blocked (lower can't preempt higher), allowed (higher preempts lower), priority lookup
   - SCB: SHPR register read/write, VTOR write
   - Updated `test_dual_core_shared_ram` to use SHARED_RAM_BASE directly (overlap fixed)

### Known Remaining Issues

- No DMA, USB, PIO emulation
- UART is Tx only (no Rx)
- Timer model is 1 cycle = 1 microsecond (not cycle-accurate)
- SDK boot path blocked by missing Resets/Clocks/XOSC/PLL stubs (Phase 2)

---

## [0.5.0] - 2026-03-08

### Performance

1. **Zero-copy dual-core context switching** (`cpu.c`, `membus.c`)
   - Eliminated 132KB RAM memcpy per dual-core step
   - `mem_set_ram_ptr()` redirects memory bus to per-core RAM arrays via pointer
   - `cpu_step_core_via_single()` swaps only 64 bytes of register state per context switch
   - ~2000x reduction in per-step memory bandwidth vs. previous RAM copy approach

2. **Shared flash across cores** (`emulator.h`, `cpu.c`)
   - Removed duplicate 2MB `flash[FLASH_SIZE]` from `cpu_state_dual_t`
   - Both cores read flash from the single `cpu.flash` array
   - Saves ~4MB of memory in dual-core mode

3. **O(1) instruction dispatch table** (`cpu.c`)
   - Replaced ~60-branch if-else chain with 256-entry lookup table indexed by `instr >> 8`
   - Secondary dispatchers handle ALU block (0x40-0x43), special/branch exchange (0x47), misc (0xB0-0xBF)
   - 32-bit instructions (BL/BLX) handled as special case before table lookup
   - `pc_updated` flag pattern: handlers set flag when modifying PC, `cpu_step()` auto-advances if unset

### Correctness

4. **PRIMASK support** (`instructions.c`, `cpu.c`, `emulator.h`)
   - Added `uint32_t primask` field to both `cpu_state_t` and `cpu_state_dual_t`
   - `instr_cpsid()` sets `cpu.primask = 1` (disable interrupts)
   - `instr_cpsie()` sets `cpu.primask = 0` (enable interrupts)
   - `cpu_step()` checks `!cpu.primask` before delivering pending interrupts
   - PRIMASK preserved across dual-core context switches

5. **SVC exception triggering** (`instructions.c`)
   - `instr_svc()` now advances PC by 2 (past SVC instruction) then calls `cpu_exception_entry(EXC_SVCALL)`
   - Sets `pc_updated = 1` to prevent double PC advance
   - Stacked return address points to instruction after SVC (correct for exception return)
   - Previously SVC was a no-op that only printed a message

6. **RAM execution** (`cpu.c`)
   - `cpu_is_halted()` now accepts PC in range `RAM_BASE` to `RAM_TOP` (0x20000000-0x20042000)
   - `cpu_step()` allows instruction fetch from RAM via existing `mem_read16()` routing
   - Enables running code copied to RAM (common for flash programming, performance-critical loops)

### Features

7. **Peripheral stubs: SPI, I2C, PWM** (`membus.c`, `emulator.h`)
   - SPI0 (0x4003C000) and SPI1 (0x40040000): SSPSR returns 0x03 (TFE=1, TNF=1 = idle/ready)
   - I2C0 (0x40044000) and I2C1 (0x40048000): all reads return 0 (idle)
   - PWM (0x40050000): all reads return 0
   - All peripheral writes in these ranges are silently accepted (no-op)
   - Prevents SDK firmware from crashing during peripheral initialization

8. **ELF loader** (`src/elf.c`, `emulator.h`, `CMakeLists.txt`)
   - New `load_elf()` function loads ELF32 ARM little-endian binaries
   - Validates magic, class (32-bit), endianness (little), architecture (ARM)
   - Loads PT_LOAD segments to flash (0x10000000) or RAM (0x20000000)
   - Handles .bss by zeroing memsz before loading filesz bytes
   - Auto-detected in main.c when filename ends with ".elf"

### New Instructions

9. **ADCS (Add with Carry)** (`instructions.c`)
   - `instr_adcs()`: Rd = Rd + Rm + C, updates all NZCV flags
   - Uses 64-bit intermediate for correct carry detection

10. **SBCS (Subtract with Carry)** (`instructions.c`)
    - `instr_sbcs()`: Rd = Rd - Rm - (1 - C), updates all NZCV flags
    - Carry flag computed via 64-bit unsigned subtraction

11. **RSBS (Reverse Subtract / Negate)** (`instructions.c`)
    - `instr_rsbs()`: Rd = 0 - Rm, updates flags via `update_sub_flags(0, Rm, result)`

### Test Suite

12. **Unit test framework** (`tests/test_suite.c`, `CMakeLists.txt`)
    - 36 tests across 10 categories covering all new features
    - Minimal C test framework with ASSERT_EQ/ASSERT_TRUE/PASS macros
    - Built as `bramble_tests` target, integrated with CTest
    - Run via `cd build && make bramble_tests && ctest`
    - Categories: PRIMASK, SVC, RAM execution, dispatch table, peripheral stubs,
      ADCS/SBCS/RSBS, dual-core memory, ELF loader, memory bus, instruction integration

### Known Remaining Issues (Resolved in v0.6.0)

All items below were fixed in v0.6.0:

- ~~Shared RAM (0x20040000) overlaps with Core 1's per-core RAM range~~ -> Reordered range checks
- No DMA, USB, PIO emulation
- UART is Tx only (no Rx)
- Timer model is 1 cycle = 1 microsecond (not cycle-accurate)

---

## [0.4.0] - 2026-03-08

### Bug Fixes

**Critical**:

1. **xPSR Thumb bit destroyed by flag updates** (`instructions.c`)
   - `update_add_flags()` and `update_sub_flags()` zeroed the entire xPSR register (`cpu.xpsr = 0`) before setting flags, which wiped the Thumb bit (bit 24) and any other state
   - Fixed to mask only the flag bits: `cpu.xpsr &= ~(FLAG_N | FLAG_Z | FLAG_C | FLAG_V)`
   - Impact: Exception entry/return and MRS reads would see corrupted xPSR

2. **LDR/STR register-offset used wrong encoding** (`instructions.c`)
   - `instr_str_reg_offset()` and `instr_ldr_reg_offset()` decoded bits [10:6] as an immediate and scaled by 4, but the 0x5000/0x5800 encoding has Rm in bits [8:6]
   - Fixed to read Rm register and compute `Rn + Rm` instead of `Rn + (imm * 4)`

3. **ADDS Rd,Rn,Rm never updated flags** (`instructions.c`, `cpu.c`)
   - `instr_add_reg_reg()` handled both the low-register ADDS (0x1800, flags required) and high-register ADD (0x4400, no flags). Neither path updated flags
   - Split into `instr_adds_reg_reg()` (with flags) and `instr_add_reg_high()` (without flags, with PC write support)

4. **POP exception return corrupted SP** (`instructions.c`)
   - When POP detected a magic EXC_RETURN value, it called `cpu_exception_return()` which restored SP, then the POP code overwrote SP with `sp += 4`
   - Fixed to return immediately after `cpu_exception_return()` without touching SP

5. **ASR #0 (encoding for ASR #32) caused undefined behavior** (`instructions.c`)
   - `((int32_t)value) >> 32` is undefined in C. Fixed to handle the `imm == 0` case explicitly: result is all-sign-bits, carry = bit 31

6. **Core 1 entry checked and reset on every instruction** (`cpu.c`)
   - `cpu_step_core()` read a hardcoded flash offset on every call and reset Core 1's PC, making Core 1 impossible to use normally
   - Removed the per-step Core 1 entry check entirely

7. **REV16 had overlapping byte positions** (`instructions.c`)
   - Byte 2 was shifted into byte 3's position while byte 3 also stayed in place
   - Fixed to `((value & 0x00FF00FF) << 8) | ((value & 0xFF00FF00) >> 8)`

8. **ROR register shift used wrong mask** (`instructions.c`)
   - Used `& 0x1F` (5 bits) instead of `& 0xFF` (8 bits) per ARM spec
   - Fixed to use bottom 8 bits, then modulo 32 for effective rotation

9. **FIFO blocking operations deadlocked** (`cpu.c`)
   - `fifo_pop()` and `fifo_push()` had `while` spin-loops that would hang forever in the single-threaded emulator
   - Replaced with non-blocking behavior: warn and return 0 / drop on overflow

10. **Spinlock acquire had inverted logic** (`cpu.c`)
    - Would spin forever if lock already held (deadlock in single-threaded emulator)
    - Fixed to match RP2040 hardware: return 0 if already locked, acquire and return nonzero if free

11. **Dual-core memory bus didn't route to peripherals** (`membus.c`)
    - `mem_read32_dual()` and `mem_write32_dual()` only handled flash, per-core RAM, and shared RAM
    - Added fallthrough to `mem_read32()` / `mem_write32()` for peripheral access (UART, GPIO, Timer, NVIC)

12. **Instruction count wrong when a core is halted** (`main.c`)
    - Always added 2 per step regardless of how many cores were running
    - Fixed to count only non-halted cores

**Minor**:

13. **WFI double-advanced PC** (`instructions.c`)
    - `instr_wfi()` manually did `cpu.r[15] += 2`, but `cpu_step()` also advances PC for non-branch instructions, causing WFI to skip an instruction
    - Removed manual PC advance from WFI

14. **SCB_VTOR returned wrong value** (`nvic.c`)
    - Hardcoded `0x10000000` instead of the actual VTOR value (`0x10000100`)
    - Fixed to return `cpu.vtor`

15. **UF2 loader had alignment-dependent pointer casts** (`uf2.c`)
    - `*(uint32_t*)&block.data[0]` can cause unaligned access faults on strict-alignment platforms
    - Replaced with `memcpy()` for safe unaligned reads

### Performance Improvements

1. **Eliminated 2MB flash copy per instruction in dual-core mode** (`cpu.c`)
   - `cpu_step_core_via_single()` previously copied 2MB flash + 132KB RAM into the global `cpu` struct on every instruction, for every core
   - Removed the flash copy entirely (flash is read-only, already in sync)
   - Still copies per-core RAM (132KB) which is necessary for correctness
   - **~15x reduction in memory bandwidth per step** (from ~4.2MB to ~264KB)

2. **Consolidated timer_tick() calls** (`cpu.c`)
   - `timer_tick(1)` was called 2-3 times per instruction (once at start, once at each return path)
   - Consolidated to single call at the top of `cpu_step()`

3. **Removed printf from hot paths** (`membus.c`, `timer.c`)
   - Removed unconditional `printf` on every timer write (`membus.c`)
   - Removed unconditional `printf` on every TIMELR read (`timer.c`)

### Known Remaining Issues (Resolved in v0.5.0)

All items below were fixed in v0.5.0:

- ~~Per-core RAM still copied (132KB) per dual-core step~~ -> Zero-copy pointer routing
- ~~Each `cpu_state_dual_t` embeds 2MB flash~~ -> Shared flash via `cpu.flash`
- ~~CPSID/CPSIE are no-ops (PRIMASK not tracked)~~ -> PRIMASK fully implemented
- ~~SVC doesn't trigger exception 11~~ -> SVCall exception works
- ~~Instruction dispatch is a linear if-else chain~~ -> O(1) dispatch table

---

## Version History

| Version | Date       | Highlights                                                                    |
|---------|------------|-------------------------------------------------------------------------------|
| 0.11.0  | 2026-03-08 | UART Rx FIFO (16-deep, trigger-level interrupts), 165 tests                    |
| 0.10.0  | 2026-03-08 | DMA controller (12 channels, chaining, immediate transfers), 157 tests         |
| 0.9.0   | 2026-03-08 | ROM function table, full UART/SPI/I2C/PWM peripherals, USB stub, 145 tests    |
| 0.8.0   | 2026-03-08 | Audit bug fixes (bcond UB, timer, SysTick), 114 tests with verbose framework  |
| 0.7.0   | 2026-03-08 | Phase 2: Resets, Clocks, XOSC/PLL, Watchdog, ADC, atomic aliases, 67 tests    |
| 0.6.0   | 2026-03-08 | Phase 1: SysTick, MSR/MRS, NVIC preemption, shared RAM fix, 52 tests          |
| 0.5.0   | 2026-03-08 | Zero-copy dual-core, dispatch table, PRIMASK, SVC, ELF loader, test suite     |
| 0.4.0   | 2026-03-08 | 15 bug fixes, 3 performance improvements, audit-driven                        |
| 0.3.0   | 2025-12-30 | Unified single/dual-core, FIFO, spinlocks                                     |
| 0.2.1   | 2025-12-06 | Debug flags, NVIC audit                                                       |
| 0.2.0   | 2025-12-03 | GPIO peripheral support                                                       |
| 0.1.0   | 2025-12-02 | Initial release: CPU, timer, UART, UF2 loader                                 |

## Git History Summary

```
0.11.0 (2026-03-08)  UART Rx FIFO (16-deep, trigger-level interrupts), 165 tests
0.10.0 (2026-03-08)  DMA controller (12 channels, chaining, immediate transfers), 157 tests
0.9.0  (2026-03-08)  ROM function table, full peripherals (UART/SPI/I2C/PWM), USB stub, 145 tests
0.8.0  (2026-03-08)  Audit bug fixes, 114-test verbose suite
0.7.0  (2026-03-08)  Phase 2: SDK boot path (Resets, Clocks, XOSC/PLL, Watchdog, ADC)
0.6.0  (2026-03-08)  Phase 1: Core correctness (SysTick, MSR/MRS, NVIC preemption)
0.5.0  (2026-03-08)  Performance, correctness, features, and test suite
0.4.0  (2026-03-08)  Audit-driven bug fixes and performance improvements
0.3.0  (2025-12-30)  Unified main, dual-core production release
0.2.1  (2025-12-06)  Debug infrastructure, NVIC audit
0.2.0  (2025-12-03)  GPIO peripheral implementation
0.1.0  (2025-12-02)  Initial release
pre    (2025-11-xx)  Early development: instruction set, memory bus, boot sequence
```

### Commit Milestones

- `8f538ed` Initial commit
- `57ecc80` GPIO peripheral header
- `dd60866` NVIC memory bus routing (Issue #1)
- `aa18ccb` Priority-aware interrupt scheduling (Issue #2)
- `454120d` Exception return in BX (Issue #3)
- `22c0167` Unified single/dual-core
- `b10f6a7` Dual-core bug fixes
- `9eeea24` Pre-audit state (v0.3.0)
