# Bramble RP2040/RP2350 Emulator - Roadmap

## Current State: v0.37.0

| Category | Coverage | Notes |
|----------|----------|-------|
| RP2040 CPU | 65+ Thumb-1 | Full ARMv6-M instruction set + O(1) dispatch + JIT |
| RP2350 RV | RV32IMC | Hazard3: 80+ instructions (I+M+C), CSRs, traps, dual hart (A extension pending) |
| Memory Map | 100% | Flash + XIP aliases + XIP SRAM + SRAM + ROM (16KB) + all APB/AHB peripherals + SIO + NVIC/SCB + atomic aliases |
| Peripherals | 100% | All 30 RP2040 peripherals emulated including VREG/BOD/CHIP_RESET, SYSCFG, TBMAN |
| Storage | SD card + eMMC | SPI-attached SD (SDHC, CSD v2.0) and eMMC with file-backed images |
| Exceptions | 100% | All vectors, 3 EXC_RETURN modes, tail-chaining, late-arriving, priority preemption, nesting, lockup, PRIMASK + FAULTMASK |
| Boot | ~95% | Vector table + SDK boot peripherals + ROM function table + boot2 auto-detect + ROM soft-float/double |
| Firmware | MicroPython + CircuitPython + littleOS | Interactive MicroPython REPL, CircuitPython `code.py`, and littleOS shell |
| Networking | UART-to-TCP | Bridge UART to TCP server/client for remote serial access |
| Multi-Device | Wire protocol | Unix socket IPC for UART/GPIO between Bramble instances |
| Threading | Host-threaded | pthread-per-core, WFI sleep, dynamic core allocation, multi-instance pool |
| Privilege | Auto-sudo | `-tap` and `-mount` auto-escalate via sudo when needed |
| Dev Tools | 18 tools | Semihosting, coverage, hotspots, profile, trace, callgraph, VCD, IRQ latency, stack check, bus log, watch, expect, script, fault injection, heatmap, symbols, exit codes, timeouts |
| Validation | 276 tests | Loader hardening, core pool, wire transport, watchdog reset, console routing, memory-map aliases, exception-path, and multicore reboot coverage |

### Recent Changes (v0.37.0)

- **RP2350 RISC-V Hazard3 engine**: RV32I (37 instructions) + M extension (8 instructions) with CSRs, trap handling, dual hart. Multi-architecture directory structure (`src/rp2350_rv/`).
- **Complete ARMv6-M exception model**: Tail-chaining, late-arriving interrupts, FAULTMASK register.
- **VREG_AND_CHIP_RESET full registers**: Replaces stub with VREG/BOD/CHIP_RESET.

### Previous (v0.36.0)

- TAP auto-configuration, FAT auto-scan, FAT12 support, build.sh rewrite.
- **FAT12 support**: 12-bit packed entries, EOC 0xFF8. CircuitPython creates FAT12 (~980 clusters).
- **Flash 0xFF init**: `cpu.flash[]` starts erased matching real hardware.
- **Zero compiler warnings**: All `-Wall -Wextra -pedantic` warnings resolved.
- **build.sh rewritten**: Auto-detects FUSE, `--clean`/`--no-fuse`/`--release`/`--debug`/`--help`.
- 12 new developer tools (v0.35.0): symbols, callgraph, stack check, IRQ latency, bus logging, GPIO VCD, scripted I/O, expect, watch, fault injection, cycle profiling, memory heatmap.

---

## Phase 1: Core Correctness (Run simple SDK programs) -- COMPLETE

### 1.1 32-bit Thumb Instructions [COMPLETE]
~~Any unrecognized 32-bit instruction halts the CPU.~~

**Note**: Cortex-M0+ (ARMv6-M) only supports BL, MSR, MRS, and DSB/DMB/ISB as 32-bit
instructions. MOVW/MOVT, LDR.W/STR.W, and B.W are Thumb-2 (ARMv7-M) and are NOT valid
on M0+. The original roadmap incorrectly listed these.

- **MSR** (0xF380 8800) - write PRIMASK, CONTROL, APSR flags, MSP
- **MRS** (0xF3EF 8xxx) - read PRIMASK, CONTROL, xPSR, APSR, IPSR, EPSR, MSP
- **DSB/DMB/ISB** (0xF3BF 8Fxx) - memory barrier NOPs (correct for emulator)
- **BL** (0xF000 + 0xF800) - already implemented in v0.1.0

### 1.2 Fix Shared RAM Overlap [COMPLETE]
~~Core 1 RAM (0x20021000-0x20042000) shadows shared RAM (0x20040000+).~~
- Reordered checks in `mem_read32_dual()` and `mem_write32_dual()`: shared RAM range
  (>= 0x20040000) checked before per-core RAM, so addresses resolve correctly

### 1.3 SysTick Timer [COMPLETE]
~~SDK `sleep_ms()` / `sleep_us()` rely on SysTick. Without it, all delays hang.~~
- Registers: SYST_CSR, SYST_RVR, SYST_CVR, SYST_CALIB fully implemented
- COUNTFLAG (bit 16) set on wrap, cleared on CSR read
- TICKINT (bit 1) pends SysTick exception when counter reaches zero
- Priority-based delivery through normal NVIC path

### 1.4 NVIC Priority Preemption [COMPLETE]
~~Currently no check whether pending IRQ can preempt active exception.~~
- `nvic_get_exception_priority()` returns effective priority for any vector number
- Pending IRQ only delivered if its priority < active exception's priority
- SCB_SHPR2/SHPR3 configure SVCall, PendSV, and SysTick priorities
- 4 priority levels via bits [7:6] (M0+ compliant)

### 1.5 MSR/MRS Implementation [COMPLETE]
~~Currently stubs.~~
- PRIMASK: read/write via MSR/MRS
- CONTROL: read/write bits [1:0] (nPRIV, SPSEL)
- xPSR: APSR (flags in bits [31:28]), IPSR (vector in bits [5:0]), EPSR (T bit)
- MSP: read/write via SYSm 0x08

---

## Phase 2: SDK Boot Path (Run Pico SDK `hello_world`) -- MOSTLY COMPLETE

### 2.1 Resets Peripheral (0x4000C000) [COMPLETE]
~~SDK calls `reset_block()` / `unreset_block_wait()` during init for every peripheral.~~
- RESET register with full bitmask tracking
- RESET_DONE = ~RESET (peripherals not held in reset are ready)
- Atomic register aliases (SET/CLR/XOR) supported

### 2.2 Clocks Peripheral (0x40008000) [COMPLETE]
~~SDK configures clocks before anything else.~~
- 10 clock generators (GPOUT0-3, REF, SYS, PERI, USB, ADC, RTC)
- Each with CTRL, DIV, SELECTED registers
- SELECTED always returns non-zero (clock source stable)
- FC0_STATUS returns DONE=1, FC0_RESULT returns 125MHz
- Atomic register aliases supported

### 2.3 XOSC (0x40024000) + PLLs (0x40028000/0x4002C000) [COMPLETE]
~~SDK enables crystal oscillator, then configures PLL for 125MHz.~~
- XOSC STATUS: STABLE=1 (bit 31) + ENABLED=1 (bit 12)
- PLL_SYS and PLL_USB: CS.LOCK=1, PWR/FBDIV/PRIM writable
- Atomic register aliases supported

### 2.4 Watchdog (0x40058000) [COMPLETE]
~~SDK configures watchdog tick for SysTick reference clock.~~
- CTRL, LOAD, REASON (always 0 = clean boot), TICK registers
- 8 scratch registers (SCRATCH0-7) for persistent data
- TICK returns RUNNING=1 when ENABLE set
- **Reboot support** (v0.19.0): CTRL bit 31 (TRIGGER) resets entire emulator

### 2.5 ADC (0x4004C000) [COMPLETE - bonus]
- 5 channels (4 GPIO + temperature sensor)
- CS (with READY=1), RESULT, FIFO, DIV, interrupt registers
- Temperature sensor channel defaults to ~27C
- `adc_set_channel_value()` for test injection

### 2.6 RP2040 Atomic Register Aliases [COMPLETE - bonus]
- All new peripherals support SET (+0x2000), CLR (+0x3000), XOR (+0x1000) aliases
- SDK `hw_set_bits()` / `hw_clear_bits()` work correctly

### 2.7 ROM Function Table [COMPLETE]

~~SDK calls ROM utility functions (memcpy, popcount, etc.) via table at 0x00000018.~~

- 16KB ROM at 0x00000000 with RP2040-compatible layout (magic, pointers, function table)
- Thumb code: `rom_table_lookup`, `memcpy`, `memset`, `popcount32`, `clz32`, `ctz32`
- Flash function stubs (connect, exit_xip, flush, enter_xip)
- **Soft-float/double** (v0.19.0): ROM data tables ('SF'/'SD') with native C float/double interception
- **Flash write** (v0.19.0): flash_range_erase/flash_range_program execute natively via ROM interception
- USB controller stub (reads return 0, writes accepted silently)

---

## Phase 3: Peripheral Emulation (Run real applications)

### 3.1 UART Full (0x40034000 / 0x40038000) [COMPLETE]
~~Rx data register: return buffered input or empty flag~~
- Proper PL011 module with full register state (DR, RSR, IBRD, FBRD, LCR_H, CR, IFLS, IMSC, RIS, MIS, ICR)
- Both UART0 and UART1 with independent state
- TX interrupt status, ICR write-1-to-clear, PL011 peripheral ID
- Atomic register aliases (SET/CLR/XOR)
- **Rx FIFO** (v0.11.0): 16-deep receive FIFO, `uart_rx_push()` API, FIFO-level interrupts

### 3.2 SPI Full (0x4003C000 / 0x40040000) [COMPLETE]
~~Data register read/write with simple FIFO~~
- PL022 module with register state (CR0, CR1, CPSR, IMSC, RIS, DMACR)
- Both SPI0 and SPI1 with independent state
- 8-deep TX/RX FIFOs, full status register (TFE, TNF, RNE, RFF, BSY)
- Device callback interface for external hardware models
- TX/RX interrupt at half-full thresholds, PL022 peripheral ID
- Atomic register aliases

### 3.3 I2C Full (0x40044000 / 0x40048000) [COMPLETE]
~~Target address, data register, status~~
- DW_apb_i2c module with full register set (CON, TAR, SAR, SCL timing, ENABLE, etc.)
- Both I2C0 and I2C1 with independent state
- 16-deep RX FIFO, proper DATA_CMD read/write/stop/restart handling
- Device callback interface: multiple devices per bus (up to 8, addressed by 7-bit addr)
- RX_FULL/TX_EMPTY/STOP_DET interrupts, CLR_* registers
- Status register (TFE, TFNF, RFNE, RFF), component ID registers
- Atomic register aliases

### 3.4 PWM (0x40050000) [COMPLETE]
~~8 PWM slices, each with counter, CC, TOP~~
- 8 independent slices with CSR, DIV, CTR, CC, TOP registers
- Global EN, INTR (W1C), INTE, INTF, INTS registers
- Atomic register aliases

### 3.5 DMA (0x50000000) [COMPLETE]
~~12 channels with source, dest, count, control~~
- 12 independent channels with READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL_TRIG
- 4 alias register layouts per channel (AL1-AL3)
- Immediate synchronous transfers: byte, halfword, word
- INCR_READ / INCR_WRITE, CHAIN_TO, IRQ_QUIET
- Global INTR (W1C), INTE0/1, INTF0/1, INTS0/1, MULTI_CHAN_TRIGGER
- **IRQ delivery** (v0.19.0): DMA completion signals NVIC IRQ 11/12 when enabled
- Atomic register aliases (SET/CLR/XOR)

### 3.6 ADC (0x4004C000) [COMPLETE]

~~Return configurable analog values, FIFO support, temperature sensor channel~~

- CS, RESULT, FCS, FIFO, DIV, interrupt registers with atomic aliases
- 4-deep FIFO with EN, SHIFT (12->8 bit), overflow/underflow flags (W1C)
- Round-robin channel cycling (RROBIN mask in CS)
- START_ONCE triggers immediate conversion and FIFO push
- Temperature sensor channel 4 (~27C default)
- FCS.LEVEL, EMPTY, FULL computed from actual FIFO state

### 3.7 PIO (0x50200000 / 0x50300000) [COMPLETE]
~~State machine instruction execution~~
- Register-level emulation: CTRL, FSTAT, FDEBUG, FLEVEL, IRQ, per-SM registers
- 32-word instruction memory writable/readable per block
- Full PIO instruction execution: all 9 opcodes (JMP, WAIT, IN, OUT, PUSH, PULL, MOV, IRQ, SET)
- Per-SM runtime state: X/Y scratch, ISR/OSR with shift counters, 4-deep TX/RX FIFOs
- PC wrapping, autopush/autopull, blocking/non-blocking FIFO ops, force-exec
- FSTAT/FLEVEL reflect actual FIFO state, GPIO pin integration via PINCTRL
- DBG_CFGINFO, interrupt registers, atomic aliases
- Per-SM fractional clock divider (16.8 fixed-point, CLKDIV_RESTART strobe)

### 3.8 USB (0x50110000) [COMPLETE]

~~Device mode endpoint handling~~

- Proper USB module (usb.c/usb.h) with register-level emulation
- 4KB DPRAM backed by real memory (endpoint descriptors writable/readable)
- MAIN_CTRL, SIE_CTRL, USB_MUXING, USB_PWR writable with atomic aliases
- Full host enumeration simulation: bus reset → descriptor fetches → configuration
- CDC data bridge: bulk IN → stdout, stdin → bulk OUT
- Multi-packet IN accumulation for descriptors > 64 bytes
- DPRAM byte/halfword access (read-modify-write routing for TinyUSB memcpy)
- SIE_STATUS/BUFF_STATUS/INTR dynamically computed; W1C for status bits

### 3.9 RTC (0x4005C000) [COMPLETE]

- Full register-level emulation: CLKDIV_M1, SETUP_0/1, CTRL, IRQ registers
- CTRL.LOAD strobe copies SETUP into running time fields
- CTRL.ACTIVE reflects CTRL.ENABLE state
- RTC_1/RTC_0 return actual running time (year/month/day/dotw/hour/min/sec)
- Clock ticks seconds based on elapsed microseconds from timer
- Proper calendar rollover (seconds → minutes → hours → days → months → years)
- Leap year support
- Atomic register aliases supported

---

## Phase 4: Advanced Features (Full compatibility)

### 4.1 SRAM Aliasing [COMPLETE]
~~Map 0x21000000 to 0x20000000~~
- SRAM mirror at 0x21000000 translates to canonical 0x20000000
- All access widths (32/16/8-bit), single-core and dual-core paths
- Note: RP2040 SRAM alias is a simple mirror, not XOR/SET/CLR (those are peripheral-only)

### 4.2 XIP Cache Control (0x14000000) [COMPLETE]
~~Cache flush, invalidate registers~~
- CTRL, FLUSH (strobe), STAT (FIFO_EMPTY + FLUSH_READY), CTR_HIT/ACC, STREAM registers
- XIP SRAM at 0x15000000 (16KB cache memory usable as general SRAM)
- XIP flash aliases: 0x11 (NOALLOC), 0x12 (NOCACHE), 0x13 (NOCACHE_NOALLOC)

### 4.3 ROM Bootloader [COMPLETE]

~~Full boot2 simulation or skip-to-application shortcut~~

- Auto-detection of boot2 in first 256 bytes of flash
- Core 0 starts at 0x10000000 when boot2 present, 0x10000100 otherwise
- `-no-boot2` CLI flag to skip boot2 execution

### 4.4 Cycle-Accurate Timing [COMPLETE]
~~Configurable cycles-per-microsecond ratio~~
- `-clock <MHz>` flag: configurable clock frequency (default 1, real RP2040: 125)
- ARMv6-M instruction timing table based on Cortex-M0+ TRM (DDI 0484C)
- Cycle accumulator converts CPU cycles to microseconds for timer
- SysTick counts in raw CPU cycles (correct per ARM spec)
- Per-instruction costs: ALU=1, LDR/STR=2, BX=3, BL=4, PUSH/POP=1+N, branches=1-2

### 4.5 GDB Remote Stub [COMPLETE]
~~TCP server implementing GDB RSP~~
- TCP server on configurable port (default 3333) implementing full GDB RSP
- Register read/write (R0-R15 + xPSR), memory read/write
- 16 software/hardware breakpoints, single-step, continue, vCont
- Ctrl-C interrupt, detach, kill, thread queries
- Usage: `./bramble firmware.uf2 -gdb` then `target remote :3333`

### 4.6 SIO Interpolators (0xD0000080-0xD00000FF) [COMPLETE]

- 2 interpolators per core (INTERP0, INTERP1) with lane-based accumulators
- ACCUM0/1, BASE0/1/2, CTRL_LANE0/1 with shift/mask/sign-extend
- PEEK (read-only lane results), POP (read + add base to accumulator)
- FULL result = LANE0 + LANE1 + BASE2
- BASE_1AND0 combined write

### 4.7 HardFault + Exception Nesting [COMPLETE]

- HardFault (vector 3) on bad PC, undefined instructions, unimplemented opcodes
- Exception nesting stack (depth 8) replaces single current_irq tracking
- Nested exceptions restore previous exception on return

### 4.8 Peripheral IRQ Delivery [COMPLETE]

- DMA completion → NVIC IRQ 11/12 (DMA_IRQ_0/1) when INTE0/1 enabled
- SIO FIFO push → NVIC IRQ 15/16 (SIO_IRQ_PROC0/1) for receiving core
- Timer alarm IRQ already implemented (IRQ 0-3)

### 4.9 Debug Logging [COMPLETE]

- `-debug-mem` flag logs unmapped peripheral read/write access to stderr
- Helps diagnose firmware accessing unimplemented peripherals

---

## Phase 5: Remaining Gaps (MicroPython/CircuitPython target)

### 5.1 GPIO Edge/Level Interrupt Detection [COMPLETE]

- Automatic INTR bit setting when pin values change
- Level interrupts: continuously asserted while pin is at configured level
- Edge interrupts: latched on rising/falling transitions, cleared by W1C
- Triggers NVIC IRQ 13 (IO_IRQ_BANK0) through INTE/INTF/INTS chain
- `gpio_set_input_pin()` API for external input changes with event detection

### 5.2 PIO INTR from FIFO Status [COMPLETE]

- INTR register dynamically computed from hardware state:
  - Bits [11:8]: SM IRQ flags
  - Bits [7:4]: TX FIFO not full (per SM)
  - Bits [3:0]: RX FIFO not empty (per SM)
- INTS = (INTR | INTF) & INTE for both IRQ0 and IRQ1
- IRQ check after each pio_step() cycle

### 5.3 Dynamic Frequency Reporting [COMPLETE]

- FC0_RESULT computed from PLL_SYS registers: `(12MHz * FBDIV) / (REFDIV * POSTDIV1 * POSTDIV2)`
- `machine.freq()` returns actual configured frequency instead of hardcoded 125MHz
- Defaults to 125MHz when PLL not yet configured

### 5.4 USB Device Enumeration [COMPLETE]

- Full USB host simulation state machine:
  - Bus reset → GET_DEVICE_DESCRIPTOR → SET_ADDRESS → GET_CONFIGURATION_DESCRIPTOR
  - SET_CONFIGURATION → CDC SET_LINE_CODING → SET_CONTROL_LINE_STATE (DTR+RTS)
- SIE_STATUS properly reflects VBUS_DETECTED, CONNECTED, BUS_RESET, SETUP_REC
- BUFF_STATUS tracking with W1C, INTR/INTS dynamic computation
- CDC bulk endpoint detection via configuration descriptor parsing
- CDC data output: bulk IN endpoint data printed to stdout
- CDC data input: `usb_cdc_rx_push()` injects stdin bytes into bulk OUT endpoint when USB CDC is the active guest console
- `usb_step()` called from main loop, advances enumeration and CDC data transfer
- `stdio_usb_connected()` returns true after full enumeration

### 5.5 Flash Persistence [COMPLETE]

- `-flash <path>` option for persistent flash storage
- On startup: loads flash file, restores non-firmware sectors (preserves UF2 firmware, restores filesystem)
- On exit: saves full 2MB flash image to file
- Enables littlefs/FAT filesystem persistence across emulator runs
- Smart sector detection: compares 4KB sectors against erased state to identify firmware vs filesystem

### 5.6 SCB/NVIC Extensions [COMPLETE]

- CPUID register (0xE000ED00): returns Cortex-M0+ identifier (0x410CC601)
- NVIC_IABR (0xE000E300): interrupt active bit register read
- IPR registers extended to full range (IPR0-7, covering all 26 IRQs)
- All NVIC/peripheral debug printf output gated behind `-debug` flag

### 5.7 RTC Time Ticking [COMPLETE]

- CTRL.LOAD strobe copies SETUP_0/1 into running time fields
- Clock ticks seconds based on elapsed microseconds (via `rtc_tick()` in timing path)
- Full calendar rollover with leap year support
- RTC_1/RTC_0 return packed running time instead of raw setup values

### 5.8 Peripheral Stubs [COMPLETE]

- SYSINFO (0x40000000): CHIP_ID returns RP2040-B2, PLATFORM=ASIC, GITREF_RP2040
- IO_QSPI (0x40018000): 6 QSPI GPIO pins with STATUS/CTRL + interrupt registers
- PADS_QSPI (0x40020000): QSPI pad electrical control registers
- XIP SSI atomic aliases: SET/CLR/XOR at +0x2000/+0x3000/+0x1000 from 0x18000000

### 5.9 ROSC (0x40060000) [COMPLETE]

- Ring Oscillator with CTRL, STATUS, RANDOMBIT, FREQA/B, DIV, PHASE, COUNT registers
- STATUS: STABLE (bit 31) + ENABLED (bit 24) based on CTRL enable field (0xFAB)
- RANDOMBIT: xorshift LFSR pseudo-random bit generation
- Integrated into clocks module with full atomic register alias support

### 5.10 SIO QSPI GPIO Input [COMPLETE]

- SIO_GPIO_HI_IN (0xD0000008): QSPI GPIO input register (6 pins: SCLK, SS, SD0-3)
- Returns CS(SS) high + data lines pulled up (0x3E) for normal flash operation
- GPIO offset routing in sio_read32() to prevent interception before gpio_read32()

### 5.11 Nice to Have

- Dormant/sleep mode
- Double-precision ROM functions (currently stubs but untested)
- DMA pacing timers (cycle-based transfer throttling)

---

## Phase 6: Connectivity & Multi-Device (Community Features)

### 6.1 UART-to-TCP Network Bridge [COMPLETE]

- `-net-uart0 <port>` / `-net-uart1 <port>`: Bridge UART to TCP server socket
- `-net-uart0-connect <host:port>` / `-net-uart1-connect <host:port>`: TCP client mode
- Non-blocking I/O with TCP_NODELAY for low-latency byte-at-a-time transfer
- Automatic client accept/disconnect; reconnection in listen mode
- UART TX routed through bridge when active (instead of stdout)

### 6.2 SPI Device Callback Interface [COMPLETE]

- 8-deep TX/RX FIFOs (PL022 spec-compliant)
- `spi_attach_device(spi_num, xfer_fn, cs_fn, ctx)` API
- Full-duplex byte exchange: SSPDR write triggers device callback, response pushed to RX FIFO
- TX/RX interrupt generation at half-full thresholds
- Status register reflects actual FIFO state

### 6.3 I2C Device Callback Interface [COMPLETE]

- 16-deep RX FIFO (DW_apb_i2c spec-compliant)
- `i2c_attach_device(i2c_num, addr, write_fn, read_fn, start_fn, stop_fn, ctx)` API
- Multiple devices per bus (up to 8) addressed by 7-bit I2C address
- DATA_CMD read/write/stop/restart bits processed immediately
- Proper interrupt generation (RX_FULL, TX_EMPTY, STOP_DET) and CLR registers

### 6.4 Multi-Instance Wire Protocol [COMPLETE]

- `-wire-uart0 <path>` / `-wire-uart1 <path>`: Wire UART between Bramble instances
- `-wire-gpio <path>`: Wire GPIO pin state between instances
- Unix domain socket IPC; first instance creates, second connects
- UART TX on one instance delivered as UART RX on the other
- GPIO pin changes propagated between instances
- Partial `SOCK_STREAM` reads and writes are buffered correctly
- Wire message protocol: 4-byte header + payload (UART_DATA, GPIO_PIN, SPI_XFER)

### 6.5 Future: Device Plugins

- Community-contributed device models (sensors, displays, flash, Ethernet)
- SPI devices: W5500 (Ethernet), SD card, SPI flash, OLED displays
- I2C devices: BME280/BMP280, MPU6050, EEPROM, RTC modules
- Each device is a self-contained `.c` file implementing the callback interface

---

## Phase 7: Storage & Runtime Access

### 7.1 Flash Write-Through Persistence [COMPLETE]

- Every `flash_range_erase` and `flash_range_program` immediately syncs to disk file
- Enables external mounting of flash filesystem at runtime (no need to wait for emulator exit)
- New module: `storage.c` / `storage.h`

### 7.2 SD Card SPI Emulation [COMPLETE]

- Full SPI-mode protocol state machine for SDHC cards
- CSD v2.0 and CID register emulation
- Commands: CMD0/8/9/10/12/13/16/17/18/24/25/55/58, ACMD41
- Single-block and multi-block read/write with file-backed storage
- CLI: `-sdcard <path>`, `-sdcard-spi <0|1>`, `-sdcard-size <MB>`
- Attaches to SPI1 by default via `spi_attach_device()` callback
- New module: `sdcard.c` / `sdcard.h`

### 7.3 eMMC SPI Emulation [COMPLETE]

- CMD1 initialization, EXT_CSD via CMD8
- Sector addressing for large storage
- File-backed storage image
- CLI: `-emmc <path>`, `-emmc-spi <0|1>`, `-emmc-size <MB>`
- Attaches to SPI0 by default via `spi_attach_device()` callback
- New module: `emmc.c` / `emmc.h`

### 7.4 FUSE Mount [COMPLETE]

- Mount flash FAT16 filesystem as host directory via libfuse3
- Thread-safe: mutex serializes FUSE operations against emulator flash writes
- FUSE writes automatically sync to disk via flash persistence
- CLI: `-mount <dir>` (requires `-flash <path>`)
- Optional build: `cmake .. -DENABLE_FUSE=ON`
- Module: `fuse_mount.c` / `fuse_mount.h` + `fatfs.c` / `fatfs.h`

---

## Phase 8: Performance

### 8.1 Instruction Caching [COMPLETE]

- 64K-entry direct-mapped decoded instruction cache
- Stores pre-decoded handler pointers for 16-bit Thumb instructions
- Avoids `mem_read16()` + dispatch table lookup on cache hits
- Invalidated on RAM writes; flash/ROM entries never invalidated
- Hit rate statistics reported on exit

### 8.2 JIT Basic Block Compilation [COMPLETE]

- 4096-entry basic block cache for consecutive instruction sequences
- Blocks terminate at branches, 32-bit instructions, or 32-instruction limit
- Block execution skips per-instruction PC bounds check, interrupt check, and dispatch lookup
- Only compiles flash/ROM code (immutable, no invalidation needed)
- Blocks of 1 instruction fall back to normal dispatch (no overhead for single instructions)
- Invalidated alongside icache on RAM writes
- CLI: `-jit` flag enables block compilation
- Statistics: block compiles, executions, and accelerated instructions reported on exit

---

## Priority Matrix

| Task | Blocks SDK hello_world? | Effort | Impact | Status |
|------|------------------------|--------|--------|--------|
| ~~32-bit instructions (MSR/MRS/barriers)~~ | ~~YES~~ | ~~Medium~~ | ~~Critical~~ | DONE v0.6.0 |
| ~~Resets peripheral~~ | ~~YES~~ | ~~Small~~ | ~~Critical~~ | DONE v0.7.0 |
| ~~Clocks peripheral~~ | ~~YES~~ | ~~Small~~ | ~~Critical~~ | DONE v0.7.0 |
| ~~XOSC/PLL~~ | ~~YES~~ | ~~Small~~ | ~~Critical~~ | DONE v0.7.0 |
| ~~SysTick timer~~ | ~~YES (sleep_ms)~~ | ~~Medium~~ | ~~Critical~~ | DONE v0.6.0 |
| ~~Shared RAM fix~~ | ~~No~~ | ~~Small~~ | ~~High~~ | DONE v0.6.0 |
| ~~NVIC preemption~~ | ~~No~~ | ~~Medium~~ | ~~High~~ | DONE v0.6.0 |
| ~~MSR/MRS full impl~~ | ~~Maybe~~ | ~~Small~~ | ~~Medium~~ | DONE v0.6.0 |
| ~~Watchdog~~ | ~~Maybe~~ | ~~Small~~ | ~~Medium~~ | DONE v0.7.0 |
| ~~ADC~~ | ~~No~~ | ~~Small~~ | ~~Medium~~ | DONE v0.7.0 |
| ~~ROM function table~~ | ~~Some programs~~ | ~~Medium~~ | ~~Medium~~ | DONE v0.8.0 |
| ~~UART Rx~~ | ~~No~~ | ~~Medium~~ | ~~Medium~~ | DONE v0.11.0 |
| ~~DMA controller~~ | ~~Some programs~~ | ~~Medium~~ | ~~Medium~~ | DONE v0.10.0 |
