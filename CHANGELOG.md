# Bramble RP2040 Emulator - Changelog

## [0.17.0] - 2026-03-08

### Added - Boot2, SIO Hardware Divider, PIO Clock Division

**Boot2 Support**:

- Auto-detection of second-stage bootloader in firmware (first 256 bytes of flash)
- When detected, Core 0 starts execution at 0x10000000 (boot2 entry) instead of 0x10000100
- `-no-boot2` CLI flag to skip boot2 execution
- `cpu_has_boot2()` / `cpu_set_boot2()` API

**SIO Hardware Divider**:

- Per-core signed and unsigned 32-bit divider
- Registers: DIV_UDIVIDEND, DIV_UDIVISOR, DIV_SDIVIDEND, DIV_SDIVISOR
- QUOTIENT, REMAINDER, CSR (READY always 1, DIRTY tracking)
- Division-by-zero returns 0xFFFFFFFF quotient, dividend as remainder
- INT32_MIN / -1 wraps to 0x80000000 (matches RP2040 hardware, avoids C UB)

**XIP SSI Flash Interface (0x18000000)**:

- CTRLR0, CTRLR1, SSIENR, SER, BAUDR, TXFTLR, RXFTLR, SR, DR0 registers
- TX/RX FIFOs (16-deep each)
- Flash read commands (0x03) with 24-bit address decoding

**PIO Clock Division**:

- Per-SM fractional clock divider (16.8 fixed-point)
- CLKDIV register: bits[31:16]=INT, bits[15:8]=FRAC
- INT=0 treated as 65536, INT=1/FRAC=0 runs every cycle (no division)
- CLKDIV_RESTART strobe (CTRL bits[11:8]) resets accumulator
- Force-exec bypasses clock divider
- Default CLKDIV set to 1.0 at init

### Fixed

- **UART PL011 hardware reset state**: UART now starts disabled (CR=0, RIS=0). TX RIS bit auto-asserted when UART is enabled via CR write.
- **Signed divider INT32_MIN / -1**: Explicit check prevents C undefined behavior
- **Interrupt test ISR**: Fixed LR clobber by adding push/pop around bl calls
- **Test firmware UART enable**: All 5 assembly test firmwares now enable UART before writing
- Removed dead `instr_mov_reg` function and all TRACE probe variables

### Testing

- **6 new tests** (218 total, up from 212)
- PIO Clock Division category: default 1:1, divide-by-2, fractional 1.5, CLKDIV_RESTART, SM_RESTART clears accumulator, force-exec bypasses divider
- 3 existing UART tests updated for hardware reset state

### Files Modified

- `src/pio.c` - Clock division in `pio_step()`, default CLKDIV=1.0, CLKDIV_RESTART handler
- `include/pio.h` - `clk_frac_acc` widened to uint32_t
- `src/cpu.c` - Boot2 detection, removed TRACE probes
- `src/membus.c` - XIP SSI, SIO divider, INT32_MIN/-1 fix
- `src/uart.c` - PL011 hardware reset state (starts disabled)
- `src/main.c` - `-no-boot2` CLI flag
- `src/instructions.c` - Removed TRACE probes and dead code
- `include/emulator.h` - Boot2 API declarations
- `tests/test_suite.c` - 6 new PIO clkdiv tests, 3 updated UART tests

---

## [0.16.0] - 2026-03-08

### Added - PIO Instruction Execution

**Full PIO State Machine Instruction Engine**:

- All 9 PIO instructions implemented: JMP, WAIT, IN, OUT, PUSH, PULL, MOV, IRQ, SET
- JMP conditions: always, !X, X--, !Y, Y--, X!=Y, PIN, !OSRE
- WAIT sources: GPIO, PIN, IRQ (with stall on condition not met)
- IN/OUT: shift direction from SHIFTCTRL, autopush/autopull with configurable thresholds
- PUSH/PULL: blocking and non-blocking variants, if_full/if_empty conditions
- MOV: all source/dest combinations, invert and bit-reverse operations
- IRQ: set/clear with relative index (modulo SM number)
- SET: pins, pindirs, X, Y scratch registers

**PIO Runtime State**:

- Per-SM scratch registers (X, Y), ISR/OSR with shift counters
- 4-deep TX and RX FIFOs with proper push/pop/empty/full semantics
- PC wrapping via EXECCTRL wrap_bottom/wrap_top
- Force-exec via SM_INSTR register write
- SM_RESTART resets all runtime state (FIFOs, shift registers, PC)
- FSTAT/FLEVEL now reflect actual FIFO state (not hardcoded)
- `pio_step()` called from main loop, steps all enabled SMs

**GPIO Integration**:

- PINCTRL-based GPIO pin mapping (set_base/count, out_base/count, in_base)
- SET/OUT write to GPIO output pins, IN reads GPIO input pins
- PINDIRS support for SET and OUT destinations

### Testing

- **18 new tests** (212 total, up from 194)
- PIO Execution category: SET X/Y, MOV X→Y, MOV invert, JMP always/!X/X--, TX FIFO push/pull, RX FIFO push/read, PULL blocking stall, FSTAT reflects FIFO, wrap, OUT X, IN X push, IRQ set/clear, SM enable/step, SM restart clears state, FLEVEL reflects FIFO

### Files Modified

- `include/pio.h` - PIO instruction opcodes, FIFO struct, full SM runtime state, `pio_step()`/`pio_sm_exec()` API
- `src/pio.c` - Complete PIO instruction execution engine (~500 lines)
- `src/main.c` - `pio_step()` in main execution loop
- `tests/test_suite.c` - 18 new PIO execution tests

---

## [0.15.0] - 2026-03-08

### Added - Cycle-Accurate Timing

**Configurable Clock Frequency** (`-clock <MHz>`):

- Replaces fixed 1:1 cycle-to-microsecond model with configurable ratio
- Default: 1 MHz (backward compatible, 1 cycle = 1 µs)
- Real RP2040: 125 MHz (125 cycles per 1 µs timer tick)
- Cycle accumulator converts CPU cycles to microseconds for timer
- SysTick counts in CPU cycles (correct per ARM spec)

**ARMv6-M Instruction Timing Table**:

- Per-instruction cycle costs based on ARM Cortex-M0+ TRM (DDI 0484C)
- Data processing (ALU, shifts, moves): 1 cycle
- Load/store (all widths and modes): 2 cycles
- BX/BLX register: 3 cycles
- Conditional branch: 1 (not taken) or 2 (taken) cycles
- Unconditional branch: 2 cycles
- PUSH/POP: 1 + N cycles (N = register count, +1 for PC pipeline refill)
- STMIA/LDMIA: 1 + N cycles
- BL (32-bit): 4 cycles
- MSR/MRS: 4 cycles
- DSB/DMB/ISB: 3 cycles

### Testing

- **11 new tests** (194 total, up from 183)
- Cycle Timing category: default config, set_clock_mhz, ALU=1, load/store=2, branch taken/not taken, BX/BLX=3, PUSH/POP=1+N, BL/MSR/DSB 32-bit costs, 125MHz accumulator, backward compatibility, STMIA/LDMIA=1+N

### Files Modified

- `include/emulator.h` - `timing_config_t`, timing API declarations
- `src/cpu.c` - Instruction timing table, cycle accumulator, `timing_tick()` helper
- `src/main.c` - `-clock <MHz>` flag
- `tests/test_suite.c` - 11 new cycle timing tests

---

## [0.14.0] - 2026-03-08

### Added - GDB Remote Debugging

**GDB Remote Serial Protocol Stub** (`-gdb [port]`):

- TCP server implementing GDB RSP for interactive firmware debugging
- Default port 3333, configurable via `-gdb <port>`
- Register read/write: R0-R15 + xPSR (17 registers, little-endian hex)
- Memory read/write: all widths via `m`/`M` commands
- Software and hardware breakpoints: up to 16 concurrent breakpoints
- Single-step execution via `s` command
- Continue execution via `c` command, vCont support
- Ctrl-C interrupt to break running execution
- Detach (`D`) and kill (`k`) commands
- Thread and feature queries (qSupported, qAttached, qC, qfThreadInfo)
- Initial stop on connect: GDB can inspect state before execution starts
- 10M instruction limit disabled during GDB sessions
- Usage: `./bramble firmware.uf2 -gdb` then `arm-none-eabi-gdb -ex "target remote :3333"`

### Files Added

- `src/gdb.c`, `include/gdb.h` - GDB remote stub module

### Files Modified

- `src/main.c` - `-gdb` flag, GDB init/handle/cleanup in execution loop
- `CMakeLists.txt` - Added `src/gdb.c` to SOURCES

---

## [0.13.0] - 2026-03-08

### Added - SRAM Aliasing + XIP Cache Control

**SRAM Aliasing** (0x21000000):

- SRAM mirror at 0x21000000 maps to canonical SRAM at 0x20000000
- All access widths (32/16/8-bit), both single-core and dual-core paths

**XIP Cache Control** (0x14000000):

- CTRL register with RP2040 defaults, FLUSH (strobe), STAT (always ready)
- CTR_HIT/CTR_ACC performance counters, STREAM registers
- XIP SRAM at 0x15000000 (16KB cache as general SRAM)

**XIP Flash Aliases**:

- 0x11000000 (NOALLOC), 0x12000000 (NOCACHE), 0x13000000 (NOCACHE_NOALLOC)
- All read from the same backing flash storage; writes ignored

### Testing

- **9 new tests** (183 total, up from 174)
- SRAM Aliasing: write-read mirror, write-through, byte/halfword access
- XIP Cache Control: defaults, STAT, FLUSH strobe, counters, XIP SRAM, flash aliases

### Files Modified

- `include/emulator.h` - SRAM_ALIAS_BASE, XIP address defines
- `src/membus.c` - SRAM alias translation, XIP cache control, XIP SRAM, flash aliases
- `tests/test_suite.c` - 9 new tests

---

## [0.12.0] - 2026-03-08

### Added - PIO Peripheral + UART Stdin Polling

**PIO (Programmable I/O)** (0x50200000 / 0x50300000):

- Two independent PIO blocks (PIO0 + PIO1) with register-level emulation
- 4 state machines per block with CLKDIV, EXECCTRL, SHIFTCTRL, ADDR, INSTR, PINCTRL
- 32-word shared instruction memory per block (writable and readable, 16-bit masked)
- FSTAT reports all TX/RX FIFOs empty; TX writes accepted and discarded; RX reads return 0
- CTRL SM_ENABLE bits tracked (SM_RESTART and CLKDIV_RESTART are strobe/self-clearing)
- IRQ and IRQ_FORCE with write-1-to-clear semantics
- DBG_CFGINFO returns correct RP2040 values (4 SMs, 32 instr mem, 4-deep FIFOs)
- Interrupt registers: IRQ0/IRQ1 INTE, INTF, INTS
- Atomic register aliases (SET/CLR/XOR)

**UART Stdin Polling** (`-stdin` flag):

- Non-blocking stdin via `O_NONBLOCK` + `poll()` syscall
- Polls every 1024 steps and pushes bytes into UART0 RX FIFO via `uart_rx_push()`
- Cleanup restores blocking stdin mode on exit
- Usage: `./bramble firmware.uf2 -stdin`

### Testing

- **9 new tests** (174 total, up from 165)
- PIO Peripheral category: FSTAT fifos empty, instruction memory readback (with 16-bit mask), SM register readback (SM0 + SM2), CTRL enable, DBG_CFGINFO, PIO1 independence, IRQ write-1-to-clear, TX/RX FIFO stubs, atomic SET/CLR aliases

### Files Added

- `src/pio.c`, `include/pio.h` - PIO peripheral module

### Files Modified

- `src/membus.c` - PIO address routing with atomic aliases
- `src/main.c` - PIO init, `-stdin` flag, UART stdin polling infrastructure
- `CMakeLists.txt` - Added `src/pio.c`
- `tests/test_suite.c` - 9 new PIO tests

---

## [0.11.0] - 2026-03-08

### Added - UART Receive Path

**UART Rx FIFO**:

- 16-deep PL011-standard receive FIFO for both UART0 and UART1
- `uart_rx_push()` API for external data injection (test harness, stdin)
- DR read pops from RX FIFO (FIFO order preserved)
- FR register reflects actual FIFO state: RXFE (empty), RXFF (full)
- RX interrupt (RIS bit 4) triggers at configurable FIFO level (IFLS bits [5:3])
- RX interrupt auto-clears when FIFO drops below trigger level on DR reads
- MIS = RIS & IMSC for masked interrupt status

### Testing

- **8 new tests** (165 total, up from 157)
- UART Rx category: push/pop, FIFO empty flag, FIFO full flag, FIFO order, RX interrupt at trigger, interrupt clear on read, UART1 independent Rx, masked interrupt

### Files Modified

- `include/uart.h` - RX FIFO fields in `uart_state_t`, `uart_rx_push()` API, `UART_INT_RT` define
- `src/uart.c` - RX FIFO implementation, DR read from FIFO, FR flags, trigger-level interrupts
- `tests/test_suite.c` - 8 new UART Rx tests

---

## [0.10.0] - 2026-03-08

### Added - DMA Controller

**DMA Controller** (0x50000000):

- 12 independent channels with READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL_TRIG
- 4 alias register layouts per channel (AL1-AL3 reorder fields, last reg triggers)
- Immediate synchronous transfer engine: byte, halfword, and word sizes
- INCR_READ / INCR_WRITE address increment control
- CHAIN_TO: automatic channel chaining on completion
- IRQ_QUIET: suppress interrupt on completion
- Global registers: INTR (W1C), INTE0/1, INTF0/1, INTS0/1, MULTI_CHAN_TRIGGER
- Pacing timers (TIMER0-3), SNIFF_CTRL/DATA, CHAN_ABORT, N_CHANNELS
- Atomic register aliases (SET/CLR/XOR)

### Testing

- **12 new tests** (157 total, up from 145)
- DMA Controller category: N_CHANNELS, defaults, readback, word transfer, byte transfer, no-incr-write, interrupt on completion, IRQ_QUIET, interrupt status, chain transfer, multi-chan trigger, atomic SET/CLR

### Files Added

- `src/dma.c`, `include/dma.h` - DMA controller module

### Files Modified

- `src/membus.c` - DMA address routing with atomic aliases
- `src/main.c` - `dma_init()` call
- `CMakeLists.txt` - Added `src/dma.c`
- `tests/test_suite.c` - 12 new DMA tests
- `docs/ROADMAP.md` - Phase 3.5 marked complete

---

## [0.9.0] - 2026-03-08

### Added - ROM Function Table & Full Peripherals

**ROM Function Table (0x00000000)**:

- 4KB ROM with RP2040-compatible layout (magic at 0x10, func/data table pointers at 0x14-0x18)
- Executable Thumb code: `rom_table_lookup`, `memcpy`, `memset`, `popcount32`, `clz32`, `ctz32`, `reverse32`
- 14 function table entries including 6 flash function no-op stubs
- CPU execution from ROM addresses (0x00000000-0x00000FFF)

**USB Controller Stub**:

- DPRAM (0x50100000) and REGS (0x50110000) accept reads/writes
- Returns 0 (disconnected); SDK `stdio_usb_init()` times out and falls back to UART

**UART Module** (extracted from membus.c):

- Dual PL011 UARTs (UART0 + UART1) with independent state
- 12 registers with read-back: DR, RSR, IBRD, FBRD, LCR_H, CR, IFLS, IMSC, RIS, MIS, ICR, DMACR
- TX interrupt status, ICR write-1-to-clear, PL011 peripheral ID registers
- Atomic register aliases (SET/CLR/XOR)

**SPI Module** (replaced inline stub):

- Dual PL022 controllers (SPI0 + SPI1) with independent state
- Register state: CR0, CR1, CPSR, IMSC, RIS, DMACR
- Status register (TFE, TNF), PL022 peripheral ID, atomic aliases

**I2C Module** (replaced return-0 stub):

- Dual DW_apb_i2c controllers (I2C0 + I2C1) with independent state
- Full register set: CON, TAR, SAR, SCL timing, ENABLE, STATUS, interrupts, DMA
- Component ID registers (COMP_PARAM_1, COMP_VERSION, COMP_TYPE), atomic aliases

**PWM Module** (replaced return-0 stub):

- 8 independent slices with CSR, DIV, CTR, CC, TOP registers
- Global EN, INTR (W1C), INTE, INTF, INTS registers, atomic aliases

### Fixed

**CMP ALU register decoding** (`instructions.c`):

- ALU-block CMP (0x4280-0x42BF) uses 3-bit register fields
- High-register CMP (0x4500-0x45FF) uses 4-bit register fields
- Split into `instr_cmp_alu()` and `instr_cmp_reg_reg()` for correct decoding

### Testing

- **31 new tests** (145 total, up from 114)
- 7 new categories: ROM Function Table, USB Controller Stub, Flash ROM Functions, UART Peripheral, SPI Peripheral, I2C Peripheral, PWM Peripheral

### Files Added

- `src/rom.c`, `include/rom.h` - ROM function table
- `src/uart.c`, `include/uart.h` - UART peripheral module
- `src/spi.c`, `include/spi.h` - SPI peripheral module
- `src/i2c.c`, `include/i2c.h` - I2C peripheral module
- `src/pwm.c`, `include/pwm.h` - PWM peripheral module

### Files Modified

- `src/membus.c` - ROM reads, USB stub, peripheral routing to new modules
- `src/cpu.c` - ROM execution, CMP ALU dispatch fix
- `src/instructions.c` - `instr_cmp_alu()` added
- `src/main.c` - Init calls for all new peripherals
- `include/emulator.h` - Peripheral defines moved to individual headers
- `CMakeLists.txt` - Added 5 new source files
- `tests/test_suite.c` - 31 new tests, updated stale peripheral tests
- `docs/ROADMAP.md` - Phase 2.7 and Phase 3.1-3.4 marked complete

---

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
