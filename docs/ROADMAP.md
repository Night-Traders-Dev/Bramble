# Bramble RP2040 Emulator - Roadmap to Full Pico Emulation

## Current State: v0.10.0

| Category | Coverage | Notes |
|----------|----------|-------|
| Instructions | ~75% | 65+ Thumb-1; 32-bit: BL, MSR, MRS, DSB/DMB/ISB |
| Memory Map | ~70% | Flash + SRAM + ROM (4KB); no SRAM aliases |
| Peripherals | ~75% | GPIO, Timer, NVIC+SysTick, UART, SPI, I2C, PWM, DMA, Resets, Clocks, XOSC, PLLs, Watchdog, ADC, USB stub; no PIO |
| Exceptions | ~70% | Entry/return, priority preemption, SysTick, PendSV |
| Boot | ~80% | Vector table + SDK boot peripherals + ROM function table |

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

- 4KB ROM at 0x00000000 with RP2040-compatible layout (magic, pointers, function table)
- Thumb code: `rom_table_lookup`, `memcpy`, `memset`, `popcount32`, `clz32`, `ctz32`
- Flash function no-op stubs (connect, exit_xip, erase, program, flush, enter_xip)
- USB controller stub (reads return 0, writes accepted silently)

---

## Phase 3: Peripheral Emulation (Run real applications)

### 3.1 UART Full (0x40034000 / 0x40038000) [COMPLETE]
~~Rx data register: return buffered input or empty flag~~
- Proper PL011 module with full register state (DR, RSR, IBRD, FBRD, LCR_H, CR, IFLS, IMSC, RIS, MIS, ICR)
- Both UART0 and UART1 with independent state
- TX interrupt status, ICR write-1-to-clear, PL011 peripheral ID
- Atomic register aliases (SET/CLR/XOR)

### 3.2 SPI Full (0x4003C000 / 0x40040000) [COMPLETE]
~~Data register read/write with simple FIFO~~
- PL022 module with register state (CR0, CR1, CPSR, IMSC, RIS, DMACR)
- Both SPI0 and SPI1 with independent state
- Status register (TFE, TNF), PL022 peripheral ID
- Atomic register aliases

### 3.3 I2C Full (0x40044000 / 0x40048000) [COMPLETE]
~~Target address, data register, status~~
- DW_apb_i2c module with full register set (CON, TAR, SAR, SCL timing, ENABLE, etc.)
- Both I2C0 and I2C1 with independent state
- Status register (TFE, TFNF), component ID registers
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
- Atomic register aliases (SET/CLR/XOR)

### 3.6 ADC (0x4004C000) [LOW]
- Return configurable analog values
- FIFO support
- Temperature sensor channel

### 3.7 PIO (0x50200000 / 0x50300000) [LOW]
- State machine instruction execution
- Most complex peripheral - defer unless needed

### 3.8 USB (0x50110000) [LOW]
- Device mode endpoint handling
- Very complex - defer unless needed

---

## Phase 4: Advanced Features (Full compatibility)

### 4.1 SRAM Aliasing
- Map 0x21000000-0x25FFFFFF to 0x20000000 with XOR/SET/CLR semantics

### 4.2 XIP Cache Control (0x14000000)
- Cache flush, invalidate registers

### 4.3 ROM Bootloader
- Full boot2 simulation or skip-to-application shortcut

### 4.4 Cycle-Accurate Timing
- Configurable cycles-per-microsecond ratio
- Instruction timing table

### 4.5 GDB Remote Stub
- TCP server implementing GDB RSP
- Hardware breakpoints, single-step, register read/write

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
| UART Rx | No | Medium | Medium | Phase 3 |
| ~~DMA controller~~ | ~~Some programs~~ | ~~Medium~~ | ~~Medium~~ | DONE v0.10.0 |
