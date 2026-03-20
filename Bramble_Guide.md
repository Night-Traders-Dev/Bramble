
# Part 1: Architecture Overview and Design Philosophy

## 1.1 Design Goals

Bramble is designed as a **development and debugging tool** for RP2040 firmware that:

- **Boots real firmware**: UF2 and ELF binaries produced by the Pico SDK, MicroPython, CircuitPython, and third-party projects run unmodified.
- **Emulates both cores**: True dual-core execution with per-core NVIC, SysTick, exception stacks, and inter-core FIFO — either cooperatively (round-robin) or with host pthreads.
- **Provides full peripheral coverage**: GPIO, Timer, UART, SPI, I2C, PWM, DMA, PIO, ADC, USB, RTC, and CYW43 WiFi — all register-accurate against the RP2040 datasheet.
- **Supports interactive debugging**: GDB remote serial protocol with breakpoints, watchpoints, conditional breakpoints, and dual-core thread support.
- **Enables multi-device systems**: UART-to-TCP bridging, Unix-domain-socket wiring between Bramble instances, SPI-attached SD card and eMMC emulation.

## 1.2 System Characteristics

| Feature | Details |
|---------|---------|
| **Target** | RP2040 (dual ARM Cortex-M0+, ARMv6-M) |
| **Language** | C99 with POSIX extensions |
| **Build** | CMake, links `-lm -lpthread` |
| **Instruction Set** | 65+ Thumb-1 instructions + BL, MSR, MRS, DSB/DMB/ISB |
| **Memory** | 2 MB XIP flash, 264 KB SRAM (per-core + shared), 16 KB ROM |
| **Peripherals** | 20+ modules (GPIO, Timer, NVIC, UART, SPI, I2C, PWM, DMA, PIO, ADC, USB, RTC, Clocks, XOSC, PLLs, ROSC, Watchdog, SIO, CYW43) |
| **Cores** | 1 or 2 emulated cores; cooperative or host-threaded execution |
| **Firmware Formats** | UF2, ELF32 ARM |
| **Debugging** | GDB RSP over TCP; 16 breakpoints, 16 watchpoints, conditional breakpoints |
| **Clock Model** | Configurable cycles-per-µs (`-clock <MHz>`); ARMv6-M instruction timing table |
| **Performance** | Instruction cache (64K entries) + optional JIT basic block compilation (`-jit`) |
| **Storage** | Flash persistence, SPI-attached SD card, eMMC, FUSE mount |
| **Networking** | UART-to-TCP bridge, Unix socket wire protocol |
| **Output** | Firmware UART/USB CDC output on stdout; all diagnostics on stderr |

## 1.3 Execution Model

Bramble operates in two execution modes:

**Cooperative mode** (default, or when GDB is active):
1. `dual_core_step()` executes one instruction on each active core in round-robin
2. `pio_step()` advances all enabled PIO state machines
3. `usb_step()` advances USB enumeration state machine
4. Peripherals (timer, SysTick) tick based on accumulated CPU cycles
5. Stdin, network, and wire polling occurs every 1024 steps

**Threaded mode** (`-cores N` or `-cores auto`):
1. One host pthread per emulated core, each running its own execution loop
2. A big lock (mutex) serializes access to shared state (peripherals, memory)
3. WFI/WFE instructions release the lock and sleep on a condition variable
4. `corepool_wake_cores()` signals sleeping cores when interrupts become pending
5. Main thread handles I/O polling, watchdog, and storage flush

## 1.4 Boot Sequence

1. CPU, ROM, and all peripherals are initialized
2. Firmware is loaded from UF2 or ELF file into flash at `0x10000000`
3. If `-flash <path>` specified, non-firmware flash sectors are restored from file
4. Boot2 auto-detection: if the first 256 bytes of flash contain valid boot2 code, Core 0 starts at `0x10000000`; otherwise it starts at `0x10000100` (application entry)
5. Vector table is read: SP from word 0, reset vector from word 1
6. Core 0 begins execution; Core 1 starts halted (launched by firmware via SIO FIFO protocol)


# Part 2: Building and Running

## 2.1 Build Requirements

- C99 compiler (GCC or Clang)
- CMake 3.10+
- POSIX system (Linux, macOS)
- pthreads (included on all POSIX systems)

## 2.2 Building

```bash
cmake -S . -B build
cmake --build build -j
```

**Build options:**

| Option | Description |
|--------|-------------|
| `-DCMAKE_BUILD_TYPE=Release` | Optimized build |
| `-DCMAKE_C_COMPILER=clang` | Use Clang instead of GCC |
| `-DENABLE_FUSE=ON` | Enable FUSE filesystem mount support (requires libfuse3) |

## 2.3 Running Tests

```bash
ctest --test-dir build --output-on-failure
```

The test suite contains 274 tests covering instruction execution, peripheral register behavior, memory access, memory-mapped alias routing, exception delivery/return, loader hardening, wire transport, and firmware boot sequences.

## 2.4 Running Firmware

```bash
# Basic execution
./bramble firmware.uf2

# With clock speed and interactive stdin
./bramble firmware.uf2 -clock 125 -stdin

# Dual-core threaded with flash persistence
./bramble firmware.uf2 -cores 2 -clock 125 -flash storage.bin

# GDB debugging
./bramble firmware.uf2 -gdb 3333
```


# Part 3: Command-Line Reference

## 3.1 Usage

```
bramble <firmware.uf2|firmware.elf> [options]
```

## 3.2 Core Options

| Flag | Arguments | Description |
|------|-----------|-------------|
| `-debug` | | Enable debug output for Core 0 (or single-core mode) |
| `-debug1` | | Enable debug output for Core 1 (dual-core only) |
| `-asm` | | Show assembly instruction tracing (reserved) |
| `-status` | | Print periodic status updates (PC, SP, FIFO state) |
| `-stdin` | | Route stdin to the active guest console (USB CDC when fully active, otherwise UART0) |
| `-gdb` | `[port]` | Start GDB RSP server (default port: 3333) |
| `-clock` | `<MHz>` | Set CPU clock frequency (default: 1, real RP2040: 125) |
| `-cores` | `<N\|auto>` | Active cores: 1, 2, or auto (queries core pool); enables threading |
| `-thread-quantum` | `<N>` | Guest instructions per threaded lock hold (default: 64, clamped to 1..4096) |
| `-no-boot2` | | Skip boot2 execution even if detected in firmware |
| `-debug-mem` | | Log unmapped peripheral read/write accesses to stderr |
| `-jit` | | Enable JIT basic block compilation for hot loops |

## 3.3 Developer Tools

| Flag | Arguments | Description |
|------|-----------|-------------|
| `-semihosting` | | Enable ARM semihosting via BKPT #0xAB (SYS_WRITE/EXIT/etc.) |
| `-coverage` | `<file>` | Write code coverage bitmap (flash + RAM PCs) on exit |
| `-hotspots` | `[N]` | Print top N PCs by execution count on exit (default: 20) |
| `-trace` | `<file>` | Write instruction trace (PC, opcode, cycles) to binary file |
| `-exit-code` | `<addr>` | Read uint32 from RAM address on halt as process exit code |
| `-timeout` | `<seconds>` | Kill emulator after N seconds (exit code 124) |
| `-symbols` | `<elf>` | Load ELF symbols for readable function names in reports |
| `-callgraph` | `<file>` | Write call graph in DOT format (Graphviz) |
| `-stack-check` | | Track per-core SP watermark, warn on near-overflow |
| `-irq-latency` | | Measure IRQ delivery latency in cycles (min/avg/max) |
| `-log-uart` | | Log every UART TX/RX byte to stderr |
| `-log-spi` | | Log every SPI MOSI/MISO byte to stderr |
| `-log-i2c` | | Log every I2C transaction to stderr |
| `-gpio-trace` | `<file>` | Record GPIO pin changes as VCD (GTKWave/PulseView) |
| `-script` | `<file>` | Feed timestamped UART/GPIO input from script file |
| `-expect` | `<file>` | Compare stdout against golden file (exit 0=match, 1=diff) |
| `-watch` | `<addr[:len]>` | Log reads/writes to address range (up to 8 regions) |
| `-inject-fault` | `<spec>` | Schedule faults: `flash_bitflip:cycle:addr`, `brownout:cycle` |
| `-profile` | `<file>` | Per-PC cycle profiling (CSV with function names if `-symbols`) |
| `-mem-heatmap` | `<file>` | Memory access heatmap per 256-byte block (CSV) |

## 3.4 Storage Options

| Flag | Arguments | Description |
|------|-----------|-------------|
| `-flash` | `<path>` | Persistent flash storage (2 MB file); saves/restores across runs |
| `-mount` | `<dir>` | Mount flash FAT filesystem as host directory via FUSE (requires `-flash`, may need sudo) |
| `-mount-offset` | `<hex>` | Flash offset of FAT region (default: `0x100000` for CircuitPython) |
| `-sdcard` | `<path>` | Attach SD card image file to SPI bus |
| `-sdcard-spi` | `<0\|1>` | SPI bus for SD card (default: 1) |
| `-sdcard-size` | `<MB>` | SD card size in MB (default: 64) |
| `-emmc` | `<path>` | Attach eMMC image file to SPI bus |
| `-emmc-spi` | `<0\|1>` | SPI bus for eMMC (default: 0) |
| `-emmc-size` | `<MB>` | eMMC size in MB (default: 128) |

## 3.5 Networking Options

| Flag | Arguments | Description |
|------|-----------|-------------|
| `-net-uart0` | `<port>` | Bridge UART0 TX/RX to TCP server socket on port |
| `-net-uart1` | `<port>` | Bridge UART1 TX/RX to TCP server socket on port |
| `-net-uart0-connect` | `<host:port>` | Connect UART0 to remote TCP host:port (client mode) |
| `-net-uart1-connect` | `<host:port>` | Connect UART1 to remote TCP host:port (client mode) |

## 3.6 Multi-Device Wiring Options

| Flag | Arguments | Description |
|------|-----------|-------------|
| `-wire-uart0` | `<path>` | Wire UART0 to peer Bramble instance via Unix domain socket |
| `-wire-uart1` | `<path>` | Wire UART1 to peer Bramble instance via Unix domain socket |
| `-wire-gpio` | `<path>` | Wire GPIO pin state to peer via Unix domain socket |

## 3.7 WiFi Options

| Flag | Arguments | Description |
|------|-----------|-------------|
| `-wifi` | | Enable CYW43439 WiFi chip emulation (Pico W) |
| `-tap` | `<ifname>` | Bridge CYW43 WLAN traffic to a host TAP interface (implies `-wifi`, sudo) |


# Part 4: Internal Architecture and Core Modules

## 4.1 Module Dependency Graph

```
main.c
  ├─ cpu.c / emulator.h        [CPU engine, dual-core context switching, O(1) dispatch]
  ├─ instructions.c             [65+ Thumb instruction handlers]
  ├─ membus.c                   [Unified memory bus with pointer-based RAM routing]
  ├─ gpio.c / gpio.h            [GPIO peripheral + SIO pin control]
  ├─ timer.c / timer.h          [64-bit microsecond timer + 4 alarms]
  ├─ nvic.c / nvic.h            [Per-core NVIC + SysTick + SCB]
  ├─ clocks.c / clocks.h        [Resets, Clocks, XOSC, PLLs, Watchdog, ROSC]
  ├─ adc.c / adc.h              [5-channel ADC + 4-deep FIFO + round-robin]
  ├─ uart.c / uart.h            [Dual PL011 UART + 16-deep Rx FIFO]
  ├─ spi.c / spi.h              [Dual PL022 SPI + 8-deep FIFOs + device callbacks]
  ├─ i2c.c / i2c.h              [Dual DW_apb_i2c + 16-deep RX FIFO + device callbacks]
  ├─ pwm.c / pwm.h              [8-slice PWM peripheral]
  ├─ dma.c / dma.h              [12-channel DMA + chaining + immediate transfers]
  ├─ pio.c / pio.h              [Dual PIO blocks + full 9-opcode instruction engine]
  ├─ usb.c / usb.h              [USB controller + host enumeration + CDC data bridge]
  ├─ rtc.c / rtc.h              [RTC with LOAD strobe + calendar rollover + leap year]
  ├─ rom.c / rom.h              [16KB ROM + soft-float/double + flash write]
  ├─ gdb.c / gdb.h              [GDB RSP server + watchpoints + conditional breakpoints]
  ├─ netbridge.c / netbridge.h  [UART-to-TCP bridge (server/client)]
  ├─ wire.c / wire.h            [Unix socket IPC for multi-instance wiring]
  ├─ storage.c / storage.h      [Flash write-through persistence]
  ├─ sdcard.c / sdcard.h        [SD card SPI-mode emulation]
  ├─ emmc.c / emmc.h            [eMMC SPI-mode emulation]
  ├─ cyw43.c / cyw43.h          [CYW43439 WiFi gSPI emulation]
  ├─ corepool.c / corepool.h    [Host-threaded execution + core pool registry]
  ├─ uf2.c / uf2.h              [UF2 firmware loader]
  └─ elf.c / elf.h              [ELF32 ARM binary loader]
```

## 4.2 CPU Engine (cpu.c / emulator.h)

**Responsibility**: Execute ARMv6-M Thumb instructions, manage dual-core state, handle exceptions, and coordinate timing.

**Core CPU State** (`cpu_state_t`):
```c
struct cpu_state_t {
    uint32_t r[16];          // R0-R12, SP(R13), LR(R14), PC(R15)
    uint32_t xpsr;           // Combined xPSR (APSR + IPSR + EPSR)
    uint32_t vtor;           // Vector Table Offset Register
    uint32_t primask;        // PRIMASK (bit 0 = interrupt disable)
    uint32_t control;        // CONTROL register (nPRIV, SPSEL)
    uint8_t  flash[2MB];     // XIP flash (shared, read-only after load)
    uint8_t  ram[264KB];     // SRAM (single-core mode)
    uint64_t step_count;     // Instructions executed
    int      debug_enabled;  // Debug output flag
    int      current_irq;    // Currently executing exception vector
};
```

**Per-Core State** (`cpu_state_dual_t`):
```c
struct cpu_state_dual_t {
    uint8_t  ram[132KB];     // Per-core RAM partition
    uint32_t r[16];          // Per-core register file
    uint32_t xpsr, vtor;     // Per-core status
    uint32_t primask, control;
    uint64_t step_count;
    int      core_id;
    int      is_halted;      // Core stopped
    int      is_wfi;         // Core sleeping (WFI/WFE)
    int      in_handler_mode;
    // Exception nesting
    uint32_t exception_stack[8];  // Stack of active exception vectors
    int      exception_depth;     // Current nesting depth
};
```

**Instruction Dispatch**:
- 256-entry lookup table indexed by instruction bits `[15:8]`
- Each entry is a function pointer to the appropriate handler
- 64K-entry decoded instruction cache (direct-mapped by PC) stores pre-decoded handler + raw instruction
- Cache invalidated on RAM writes; flash/ROM entries never invalidated

**Exception Handling**:
- Full ARMv6-M exception entry: pushes 8-word stack frame (R0-R3, R12, LR, PC, xPSR)
- Exception return via `EXC_RETURN` magic values (`0xFFFFFFF1`, `0xFFFFFFF9`, `0xFFFFFFFD`)
- Exception nesting stack (depth 8) replaces single `current_irq` tracking
- Priority-based preemption: pending IRQ delivered only if priority < active exception
- ARMv6-M double-fault lockup: HardFault during HardFault handler halts core (real M0+ behavior)
- HardFault (vector 3) triggered by bad PC, undefined instructions, BKPT

**Timing Model**:
- Configurable `cycles_per_us` via `-clock <MHz>` (default: 1, real: 125)
- ARMv6-M instruction timing per Cortex-M0+ TRM (DDI 0484C):

| Instruction Class | Cycles |
|-------------------|--------|
| ALU (ADD, SUB, MOV, CMP, AND, ORR, etc.) | 1 |
| LDR, STR (all variants) | 2 |
| BX, BLX | 3 |
| BL | 4 |
| PUSH, POP | 1 + N (N = register count) |
| Taken branch (B, Bcc) | 2 |
| Not-taken branch | 1 |

- Cycle accumulator converts CPU cycles to microseconds for timer peripheral
- SysTick counts in raw CPU cycles (per ARM spec)

## 4.3 Memory Bus (membus.c)

**Responsibility**: Route all memory accesses (read/write, 8/16/32-bit) to the correct backing store or peripheral.

**Memory Map**:

| Address Range | Size | Description |
|---------------|------|-------------|
| `0x00000000 - 0x00003FFF` | 16 KB | ROM (bootrom) |
| `0x10000000 - 0x101FFFFF` | 2 MB | XIP Flash |
| `0x11000000 - 0x111FFFFF` | 2 MB | XIP NOALLOC alias |
| `0x12000000 - 0x121FFFFF` | 2 MB | XIP NOCACHE alias |
| `0x13000000 - 0x131FFFFF` | 2 MB | XIP NOCACHE_NOALLOC alias |
| `0x14000000` | | XIP Cache Control |
| `0x15000000 - 0x15003FFF` | 16 KB | XIP SRAM |
| `0x18000000` | | XIP SSI (flash SPI controller) |
| `0x20000000 - 0x20041FFF` | 264 KB | SRAM (striped across 6 banks) |
| `0x21000000 - 0x21041FFF` | 264 KB | SRAM alias (mirror) |
| `0xD0000000` | | SIO (single-cycle I/O) |
| `0x40000000 - 0x4FFFFFFF` | | Peripheral registers |
| `0xE0000000` | | PPB (NVIC, SysTick, SCB) |

**Dual-Core RAM Layout**:

| Address Range | Size | Owner |
|---------------|------|-------|
| `0x20000000 - 0x2001FFFF` | 128 KB | Core 0 private RAM |
| `0x20020000 - 0x2003FFFF` | 128 KB | Core 1 private RAM |
| `0x20040000 - 0x20041FFF` | 8 KB | Shared RAM |

- `mem_set_ram_ptr()` redirects memory bus to the active core's RAM (zero-copy pointer swap)
- All peripheral registers support atomic aliases: Normal (`+0x0000`), XOR (`+0x1000`), SET (`+0x2000`), CLR (`+0x3000`)

## 4.4 Instruction Set (instructions.c)

**Responsibility**: Implement all 65+ ARMv6-M Thumb-1 instruction handlers plus supported 32-bit instructions.

**Implemented Instructions**:

| Category | Instructions |
|----------|-------------|
| **Data Processing** | ADD, ADC, SUB, SBC, RSB, MOV, MVN, MUL, AND, ORR, EOR, BIC, TST, CMP, CMN |
| **Shift/Rotate** | LSL, LSR, ASR, ROR |
| **Load/Store** | LDR, LDRH, LDRB, LDRSH, LDRSB, STR, STRH, STRB (immediate, register, SP-relative, PC-relative) |
| **Load/Store Multiple** | LDM, STM (with writeback) |
| **Stack** | PUSH, POP (including LR/PC) |
| **Branch** | B, Bcc (14 conditions), BX, BLX, BL (32-bit) |
| **System** | SVC, BKPT, NOP, SEV, WFI, WFE, YIELD, CPSIE, CPSID |
| **Misc** | SXTH, SXTB, UXTH, UXTB, REV, REV16, REVSH, ADR |
| **32-bit** | BL, MSR, MRS, DSB, DMB, ISB |

**ARMv6-M Note**: The Cortex-M0+ does **not** support Thumb-2 instructions (MOVW, MOVT, LDR.W, STR.W, B.W, IT, TBB/TBH). These are ARMv7-M only.

**Special Behaviors**:
- LDM with base register in register list: writeback NOT applied (loaded value wins, per ARMv6-M spec)
- POP PC with EXC_RETURN: SP updated BEFORE exception return processing
- BKPT triggers HardFault exception (vector 3) instead of halting emulator
- Unrecognized 32-bit instructions trigger HardFault


# Part 5: Peripheral Emulation

## 5.1 GPIO (gpio.c / gpio.h)

**Base Addresses**: IO_BANK0 at `0x40014000`, PADS_BANK0 at `0x4001C000`, SIO GPIO at `0xD0000000`

**SIO GPIO Registers** (at SIO base `0xD0000000`):

| Offset | Register | Description |
|--------|----------|-------------|
| `0x004` | GPIO_IN | GPIO input values (read-only) |
| `0x008` | GPIO_HI_IN | QSPI GPIO input (returns 0x3E) |
| `0x010` | GPIO_OUT | GPIO output values |
| `0x014` | GPIO_OUT_SET | Atomic set output bits |
| `0x018` | GPIO_OUT_CLR | Atomic clear output bits |
| `0x01C` | GPIO_OUT_XOR | Atomic toggle output bits |
| `0x020` | GPIO_OE | GPIO output enable |
| `0x024` | GPIO_OE_SET | Atomic set output enable bits |
| `0x028` | GPIO_OE_CLR | Atomic clear output enable bits |
| `0x02C` | GPIO_OE_XOR | Atomic toggle output enable bits |

**IO_BANK0**: 30 GPIO pins, each with STATUS and CTRL registers (stride 8 bytes). CTRL selects function (SIO, UART, SPI, I2C, PWM, PIO, etc.).

**Interrupt Detection**:
- Level interrupts: continuously asserted while pin matches configured level
- Edge interrupts: latched on rising/falling transitions, cleared by W1C write to INTR
- GPIO events trigger NVIC IRQ 13 (`IO_IRQ_BANK0`) through INTE/INTF/INTS chain
- `gpio_set_input_pin(pin, value)` API for external input injection with automatic event detection

## 5.2 Timer (timer.c / timer.h)

**Base Address**: `0x40054000`

**Registers**:

| Offset | Register | Description |
|--------|----------|-------------|
| `0x00` | TIMEHW | Write high word of 64-bit counter |
| `0x04` | TIMELW | Write low word of 64-bit counter |
| `0x08` | TIMEHR | Read high word (latched by TIMELR read) |
| `0x0C` | TIMELR | Read low word (latches TIMEHR for atomic 64-bit reads) |
| `0x10` | ALARM0 | Alarm 0 compare value (writing arms alarm) |
| `0x14` | ALARM1 | Alarm 1 compare value |
| `0x18` | ALARM2 | Alarm 2 compare value |
| `0x1C` | ALARM3 | Alarm 3 compare value |
| `0x20` | ARMED | Armed alarm bitmask (write 1 to disarm) |
| `0x24` | TIMERAWH | Raw high word (no latching) |
| `0x28` | TIMERAWL | Raw low word (no latching) |
| `0x30` | DBGPAUSE | Debug pause control (stub) |
| `0x34` | PAUSE | Pause/resume timer |
| `0x38` | INTR | Raw interrupt status (W1C) |
| `0x3C` | INTE | Interrupt enable |
| `0x40` | INTF | Interrupt force |
| `0x44` | INTS | Interrupt status: `(INTR | INTF) & INTE` |

**Behavior**:
- 64-bit microsecond counter (`time_us`) incremented by `timer_tick()` based on CPU cycle accumulator
- 4 alarm comparators: alarm fires when `(int32_t)(current_low - alarm_target) >= 0` (signed comparison handles 32-bit wrap)
- Writing an ALARM register automatically arms it
- Alarm fire: sets INTR bit, signals NVIC IRQ 0-3 (`TIMER_IRQ_0-3`), disarms alarm
- Atomic 64-bit reads: reading TIMELR latches the high word for subsequent TIMEHR read

## 5.3 NVIC and SysTick (nvic.c / nvic.h)

**Base Address**: `0xE000E000` (Private Peripheral Bus)

**NVIC Registers** (offsets from `0xE000E000`):

| Offset | Register | Description |
|--------|----------|-------------|
| `0x010` | SYST_CSR | SysTick Control and Status |
| `0x014` | SYST_RVR | SysTick Reload Value |
| `0x018` | SYST_CVR | SysTick Current Value |
| `0x01C` | SYST_CALIB | SysTick Calibration (returns `0xC0002710`) |
| `0x100` | NVIC_ISER | Interrupt Set Enable Register |
| `0x180` | NVIC_ICER | Interrupt Clear Enable Register |
| `0x200` | NVIC_ISPR | Interrupt Set Pending Register |
| `0x280` | NVIC_ICPR | Interrupt Clear Pending Register |
| `0x300` | NVIC_IABR | Interrupt Active Bit Register |
| `0x400` | NVIC_IPR0-7 | Interrupt Priority Registers (4 IRQs per word) |
| `0xD00` | SCB_CPUID | CPUID Base Register (returns `0x410CC601`) |
| `0xD04` | SCB_ICSR | Interrupt Control and State |
| `0xD08` | SCB_VTOR | Vector Table Offset Register |
| `0xD0C` | SCB_AIRCR | Application Interrupt and Reset Control |
| `0xD10` | SCB_SCR | System Control Register |
| `0xD14` | SCB_CCR | Configuration and Control Register |
| `0xD1C` | SCB_SHPR2 | System Handler Priority Register 2 (SVCall) |
| `0xD20` | SCB_SHPR3 | System Handler Priority Register 3 (PendSV, SysTick) |

**Per-Core Architecture**:
- `nvic_states[2]`: each core has independent enable, pending, priority, IABR, PendSV state
- `systick_states[2]`: each core has independent SysTick counter, reload, pending flag
- `nvic_signal_irq(irq)`: sets pending on **both** cores (shared interrupt line); each core's enable mask filters delivery independently

**SysTick Behavior**:
- Counts down in raw CPU cycles (not microseconds)
- COUNTFLAG (bit 16) set when counter reaches zero, cleared on CSR read
- When TICKINT (bit 1) set, reaching zero pends SysTick exception and wakes sleeping cores
- Reload occurs from RVR when counter reaches zero

**Priority Model**:
- 4 priority levels via bits `[7:6]` (Cortex-M0+ has 2 priority bits)
- Fixed priorities: Reset = -3, NMI = -2, HardFault = -1
- Configurable: SVCall (SHPR2), PendSV and SysTick (SHPR3), external IRQs (IPR0-7)

## 5.4 IRQ Assignment Table

| IRQ | Name | Source Peripheral |
|-----|------|-------------------|
| 0-3 | `TIMER_IRQ_0-3` | Timer alarm 0-3 |
| 4 | `PWM_IRQ_WRAP` | PWM counter wrap |
| 5 | `USBCTRL_IRQ` | USB controller |
| 6 | `XIP_IRQ` | XIP cache |
| 7-8 | `PIO0_IRQ_0/1` | PIO block 0 |
| 9-10 | `PIO1_IRQ_0/1` | PIO block 1 |
| 11-12 | `DMA_IRQ_0/1` | DMA controller |
| 13 | `IO_IRQ_BANK0` | GPIO bank 0 |
| 14 | `IO_IRQ_QSPI` | QSPI GPIO |
| 15-16 | `SIO_IRQ_PROC0/1` | Inter-core FIFO |
| 17 | `CLOCKS_IRQ` | Clock system |
| 18-19 | `SPI0/1_IRQ` | SPI controllers |
| 20-21 | `UART0/1_IRQ` | UART controllers |
| 22 | `ADC_IRQ_FIFO` | ADC FIFO |
| 23-24 | `I2C0/1_IRQ` | I2C controllers |
| 25 | `RTC_IRQ` | Real-time clock |

## 5.5 UART (uart.c / uart.h)

**Base Addresses**: UART0 at `0x40034000`, UART1 at `0x40038000`

PL011 UART peripheral with full register state:

| Offset | Register | Description |
|--------|----------|-------------|
| `0x000` | DR | Data Register (TX write / RX read) |
| `0x004` | RSR | Receive Status Register |
| `0x018` | FR | Flag Register (BUSY, RXFE, TXFF, RXFF, TXFE) |
| `0x024` | IBRD | Integer Baud Rate Divisor |
| `0x028` | FBRD | Fractional Baud Rate Divisor |
| `0x02C` | LCR_H | Line Control Register |
| `0x030` | CR | Control Register (UART enable, TX/RX enable) |
| `0x034` | IFLS | Interrupt FIFO Level Select |
| `0x038` | IMSC | Interrupt Mask Set/Clear |
| `0x03C` | RIS | Raw Interrupt Status |
| `0x040` | MIS | Masked Interrupt Status |
| `0x044` | ICR | Interrupt Clear Register (W1C) |

**Features**:
- 16-deep circular receive FIFO per UART
- `uart_rx_push(uart_num, byte)` API for external data injection
- RX interrupt triggers at IFLS-configured FIFO level, auto-clears when drained below threshold
- TX routing priority: net bridge → wire → stdout
- UART starts disabled per PL011 spec; TX RIS asserted when UART enabled via CR write
- Triggers NVIC IRQ 20/21 (`UART0/1_IRQ`)

## 5.6 SPI (spi.c / spi.h)

**Base Addresses**: SPI0 at `0x4003C000`, SPI1 at `0x40040000`

PL022 SPI peripheral:

| Offset | Register | Description |
|--------|----------|-------------|
| `0x000` | SSPCR0 | Control Register 0 |
| `0x004` | SSPCR1 | Control Register 1 |
| `0x008` | SSPDR | Data Register (TX write / RX read) |
| `0x00C` | SSPSR | Status Register (TFE, TNF, RNE, RFF, BSY) |
| `0x010` | SSPCPSR | Clock Prescale Register |
| `0x014` | SSPIMSC | Interrupt Mask Set/Clear |
| `0x018` | SSPRIS | Raw Interrupt Status |
| `0x01C` | SSPMIS | Masked Interrupt Status |
| `0x020` | SSPICR | Interrupt Clear Register |

**Features**:
- 8-deep TX and RX FIFOs per SPI controller
- Device callback interface: `spi_attach_device(spi_num, xfer_fn, cs_fn, ctx)`
- Writing SSPDR triggers full-duplex device callback; response pushed to RX FIFO
- TX/RX interrupts at half-full thresholds
- Triggers NVIC IRQ 18/19 (`SPI0/1_IRQ`)

## 5.7 I2C (i2c.c / i2c.h)

**Base Addresses**: I2C0 at `0x40044000`, I2C1 at `0x40048000`

DW_apb_i2c peripheral:

| Key Register | Description |
|-------------|-------------|
| IC_CON | Control (master/slave, speed, restart) |
| IC_TAR | Target address |
| IC_SAR | Slave address |
| IC_DATA_CMD | Data + command bits (bit 8=read, bit 9=stop, bit 10=restart) |
| IC_INTR_STAT | Interrupt status |
| IC_INTR_MASK | Interrupt mask |
| IC_RX_TL | RX FIFO threshold |
| IC_TX_TL | TX FIFO threshold |
| IC_ENABLE | Enable/disable |
| IC_STATUS | Bus status (TFE, TFNF, RFNE, RFF, ACTIVITY) |

**Features**:
- 16-deep RX FIFO
- Device callback interface: `i2c_attach_device(i2c_num, addr, write_fn, read_fn, start_fn, stop_fn, ctx)`
- Up to 8 devices per bus, addressed by 7-bit I2C address
- DATA_CMD read/write/stop/restart bits executed immediately through device callback
- RX_FULL, TX_EMPTY, STOP_DET interrupts
- Triggers NVIC IRQ 23/24 (`I2C0/1_IRQ`)

## 5.8 PWM (pwm.c / pwm.h)

**Base Address**: `0x40050000`

8 independent PWM slices, each with:

| Offset (per slice, stride `0x14`) | Register | Description |
|-----------------------------------|----------|-------------|
| `0x00` | CSR | Control and Status |
| `0x04` | DIV | Clock divider (8.4 fixed-point) |
| `0x08` | CTR | Counter value |
| `0x0C` | CC | Compare values (A in low 16, B in high 16) |
| `0x10` | TOP | Wrap value |

**Global Registers**:

| Offset | Register | Description |
|--------|----------|-------------|
| `0xA0` | EN | Global enable bitmask |
| `0xA4` | INTR | Raw interrupt status (W1C) |
| `0xA8` | INTE | Interrupt enable |
| `0xAC` | INTF | Interrupt force |
| `0xB0` | INTS | Interrupt status |

- Triggers NVIC IRQ 4 (`PWM_IRQ_WRAP`)

## 5.9 DMA (dma.c / dma.h)

**Base Address**: `0x50000000`

12 independent channels, each with 4 alias register layouts:

| Alias | Registers (stride `0x10`) | Trigger Register |
|-------|---------------------------|------------------|
| AL0 | READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL_TRIG | CTRL_TRIG |
| AL1 | CTRL, READ_ADDR, WRITE_ADDR, TRANS_COUNT_TRIG | TRANS_COUNT_TRIG |
| AL2 | CTRL, TRANS_COUNT, READ_ADDR, WRITE_ADDR_TRIG | WRITE_ADDR_TRIG |
| AL3 | CTRL, WRITE_ADDR, TRANS_COUNT, READ_ADDR_TRIG | READ_ADDR_TRIG |

**CTRL_TRIG Fields**:
- `DATA_SIZE`: Transfer width (0=byte, 1=halfword, 2=word)
- `INCR_READ`, `INCR_WRITE`: Auto-increment source/destination
- `CHAIN_TO`: Channel to trigger after completion (self = no chaining)
- `IRQ_QUIET`: Suppress IRQ on completion
- `EN`: Channel enable

**Global Registers**:

| Offset | Register | Description |
|--------|----------|-------------|
| `0x400` | INTR | Raw interrupt status (W1C) |
| `0x404` | INTE0 | Interrupt enable for DMA_IRQ_0 |
| `0x414` | INTE1 | Interrupt enable for DMA_IRQ_1 |
| `0x430` | MULTI_CHAN_TRIGGER | Trigger multiple channels simultaneously |

**Behavior**:
- Transfers execute **synchronously** (immediate) when triggered — not cycle-by-cycle
- Writing the trigger register in each alias initiates the transfer
- Completion signals NVIC IRQ 11/12 (`DMA_IRQ_0/1`) when INTE0/1 enabled

## 5.10 PIO (pio.c / pio.h)

**Base Addresses**: PIO0 at `0x50200000`, PIO1 at `0x50300000`

Each PIO block contains 4 state machines, 32-word instruction memory, and independent FIFOs.

**Block Registers**:

| Offset | Register | Description |
|--------|----------|-------------|
| `0x000` | CTRL | SM enable, restart, clock divider restart |
| `0x004` | FSTAT | FIFO status (TX full/empty, RX full/empty per SM) |
| `0x008` | FDEBUG | FIFO debug (stall, overflow, underflow flags) |
| `0x00C` | FLEVEL | FIFO levels (4 bits per TX/RX per SM) |
| `0x010-0x01C` | TXF0-3 | TX FIFO write ports |
| `0x020-0x02C` | RXF0-3 | RX FIFO read ports |
| `0x030` | IRQ | IRQ flags (8 bits) |
| `0x034` | IRQ_FORCE | Force IRQ flags |
| `0x048-0x0C4` | INSTR_MEM0-31 | 32-word instruction memory |

**Per-SM Registers** (stride `0x18`, starting at `0x0C8`):

| Offset | Register | Description |
|--------|----------|-------------|
| `+0x00` | CLKDIV | Clock divider (16.8 fixed-point) |
| `+0x04` | EXECCTRL | Execution control (wrap, side-set, status) |
| `+0x08` | SHIFTCTRL | Shift control (autopush/pull thresholds, direction) |
| `+0x0C` | ADDR | Current PC |
| `+0x10` | INSTR | Current/force-exec instruction |
| `+0x14` | PINCTRL | Pin mapping (IN/OUT/SET/SIDESET base + count) |

**PIO Instruction Set** (9 opcodes):

| Opcode | Bits [15:13] | Description |
|--------|-------------|-------------|
| JMP | `000` | Conditional jump (always, !X, X--, !Y, Y--, X!=Y, PIN, !OSRE) |
| WAIT | `001` | Wait for GPIO/PIN/IRQ condition |
| IN | `010` | Shift data into ISR from source |
| OUT | `011` | Shift data out of OSR to destination |
| PUSH | `100` | Push ISR to RX FIFO (blocking/non-blocking) |
| PULL | `100` | Pull TX FIFO to OSR (blocking/non-blocking) |
| MOV | `101` | Move/copy between registers |
| IRQ | `110` | Set/clear/wait IRQ flags |
| SET | `111` | Set pins/pindirs/X/Y to immediate value |

**Features**:
- 4-deep circular FIFO per TX/RX per SM
- Autopush/autopull based on shift count thresholds
- Fractional clock divider (16.8 fixed-point accumulator per SM)
- Force-exec: writing SM_INSTR executes instruction on next `pio_step()`
- FSTAT/FLEVEL reflect actual FIFO state
- GPIO pin integration via PINCTRL (IN/OUT/SET/SIDESET base and count)
- INTR dynamically computed: IRQ flags [11:8], TX not full [7:4], RX not empty [3:0]
- INTS = (INTR | INTF) & INTE; triggers NVIC IRQ 7-10 (`PIO0/1_IRQ_0/1`)

## 5.11 ADC (adc.c / adc.h)

**Base Address**: `0x4004C000`

| Offset | Register | Description |
|--------|----------|-------------|
| `0x00` | CS | Control and Status (AINSEL, START_ONCE, READY, RROBIN) |
| `0x04` | RESULT | Most recent conversion result |
| `0x08` | FCS | FIFO Control and Status (EN, SHIFT, LEVEL, EMPTY, FULL, OVER, UNDER) |
| `0x0C` | FIFO | FIFO read port |
| `0x10` | DIV | Clock divider |
| `0x14-0x20` | INTR/INTE/INTF/INTS | Interrupt registers |

**Features**:
- 5 channels: GPIO 26-29 (channels 0-3) + temperature sensor (channel 4, ~27°C default)
- 4-deep circular FIFO with optional 12→8 bit shift
- Round-robin mode: RROBIN mask in CS advances AINSEL after each conversion
- `adc_set_channel_value(channel, value)` for test injection
- Overflow/underflow flags (W1C) in FCS
- Triggers NVIC IRQ 22 (`ADC_IRQ_FIFO`)

## 5.12 USB (usb.c / usb.h)

**Base Addresses**: USBCTRL_DPRAM at `0x50100000` (4 KB), USBCTRL_REGS at `0x50110000`

**Key Registers** (offsets from `0x50110000`):

| Offset | Register | Description |
|--------|----------|-------------|
| `0x00` | ADDR_ENDP | Device address and endpoint |
| `0x40` | MAIN_CTRL | Main control (CONTROLLER_EN, HOST_NDEVICE) |
| `0x4C` | SIE_CTRL | SIE control (PULLUP_EN, SOF_EN, START_TRANS) |
| `0x50` | SIE_STATUS | SIE status (VBUS_DETECTED, CONNECTED, BUS_RESET, SETUP_REC) |
| `0x58` | BUFF_STATUS | Buffer status per endpoint (W1C) |
| `0x8C-0x98` | INTR/INTE/INTF/INTS | Interrupt registers |

**Host Enumeration Simulation**:
1. Waits for `PULLUP_EN` in SIE_CTRL
2. Simulates bus reset → `GET_DEVICE_DESCRIPTOR` → `SET_ADDRESS` → `GET_CONFIGURATION_DESCRIPTOR` → `SET_CONFIGURATION` → `SET_CONTROL_LINE_STATE` (DTR+RTS)
3. Each step paced by firmware response (writing buf_ctrl AVAILABLE)
4. After enumeration reaches ACTIVE state, CDC data bridge activates

**CDC Data Bridge**:
- Bulk IN endpoint data → stdout
- stdin → `usb_cdc_rx_push(byte)` → bulk OUT endpoint
- 256-byte circular RX FIFO with `usb_cdc_rx_drain()` for endpoint delivery
- Multi-packet IN accumulation for descriptors > 64 bytes (256-byte buffer)
- CDC interface detection from configuration descriptor parsing

**DPRAM Access**:
- 4 KB dual-port RAM for endpoint buffer descriptors and data
- Supports byte, halfword, and word access (read-modify-write routing for TinyUSB memcpy)
- buf_ctrl: AVAILABLE (bit 10), FULL (bit 15), LEN (bits [9:0])

## 5.13 RTC (rtc.c / rtc.h)

**Base Address**: `0x4005C000`

| Offset | Register | Description |
|--------|----------|-------------|
| `0x00` | CLKDIV_M1 | Clock divider minus 1 |
| `0x04` | SETUP_0 | Setup: year, month, day |
| `0x08` | SETUP_1 | Setup: day-of-week, hour, minute, second |
| `0x0C` | CTRL | Control (ENABLE, LOAD strobe, ACTIVE) |
| `0x10` | IRQ_SETUP_0 | IRQ setup 0 |
| `0x14` | IRQ_SETUP_1 | IRQ setup 1 |
| `0x18` | RTC_1 | Running time: year, month, day |
| `0x1C` | RTC_0 | Running time: day-of-week, hour, minute, second |
| `0x20-0x2C` | INTR/INTE/INTF/INTS | Interrupt registers |

**Behavior**:
- Writing CTRL with LOAD bit copies SETUP_0/1 into running time fields
- Clock ticks seconds based on elapsed microseconds (via `rtc_tick()` in timing path)
- Full calendar rollover: seconds → minutes → hours → days → months → years
- Leap year support (Gregorian calendar rules)
- Triggers NVIC IRQ 25 (`RTC_IRQ`)

## 5.14 Clock System (clocks.c / clocks.h)

Consolidated module handling all clock-domain peripherals:

**Resets** (`0x4000C000`):
- RESET register with full bitmask tracking
- RESET_DONE = ~RESET (peripherals not in reset are ready)
- Firmware calls `reset_block()` / `unreset_block_wait()` during initialization

**Clocks** (`0x40008000`):
- 10 clock generators (GPOUT0-3, REF, SYS, PERI, USB, ADC, RTC)
- Each with CTRL, DIV, SELECTED registers
- FC0_RESULT computed from PLL_SYS: `(12 MHz × FBDIV) / (REFDIV × POSTDIV1 × POSTDIV2)`

**XOSC** (`0x40024000`):
- Crystal oscillator; STATUS reports STABLE + ENABLED

**PLLs** (`0x40028000` / `0x4002C000`):
- PLL_SYS and PLL_USB with CS.LOCK, PWR, FBDIV, PRIM registers

**ROSC** (`0x40060000`):
- Ring oscillator with CTRL, STATUS, RANDOMBIT (xorshift LFSR), FREQA/B, DIV, PHASE

**Watchdog** (`0x40058000`):
- CTRL (TRIGGER bit 31 for reboot), LOAD, REASON (0 = clean boot), TICK, SCRATCH0-7
- SYSRESETREQ via AIRCR (`0x05FA0004`) triggers system reset through `watchdog_reboot_pending`

## 5.15 SIO (cpu.c / emulator.h)

**Base Address**: `0xD0000000`

Single-cycle I/O block, partially integrated into `cpu.c`:

| Feature | Offset | Description |
|---------|--------|-------------|
| GPIO | `0x000-0x02C` | GPIO input/output/enable (see Section 5.1) |
| FIFO | `0x050-0x058` | Inter-core FIFO (32 entries per direction) |
| Divider | `0x060-0x078` | Hardware divider (per-core signed/unsigned) |
| Interpolators | `0x080-0x0FF` | 2 interpolators per core (lane shift/mask/PEEK/POP) |
| CPUID | `0x000` | Returns current core ID (0 or 1) |

**Hardware Divider**:
- Per-core state; signed and unsigned division
- Division by zero returns `0xFFFFFFFF`
- `INT32_MIN / -1` wraps correctly

**Interpolators** (2 per core, INTERP0/INTERP1):
- ACCUM0/1, BASE0/1/2, CTRL_LANE0/1 with configurable shift, mask, sign-extend
- PEEK: read lane results without side effects
- POP: read result and add base to accumulator
- FULL result = LANE0 + LANE1 + BASE2

**Inter-Core FIFO**:
- 32-entry FIFO per direction
- `fifo_try_push()` signals NVIC IRQ 15/16 (`SIO_IRQ_PROC0/1`) for receiving core
- Used by Pico SDK for Core 1 launch protocol


# Part 6: ROM and Boot

## 6.1 ROM Layout (rom.c / rom.h)

**Base Address**: `0x00000000`, Size: 16 KB (`0x4000`)

The ROM implements the RP2040 bootrom layout:
- Magic bytes and version at base
- Function pointer table at `0x00000018`
- Thumb code stubs for utility functions

**ROM Functions**:

| Function | Description |
|----------|-------------|
| `rom_table_lookup` | Look up function by two-character code |
| `memcpy` / `memset` | Standard memory operations |
| `popcount32` / `clz32` / `ctz32` | Bit manipulation utilities |
| `flash_connect` / `flash_exit_xip` / `flash_flush` / `flash_enter_xip` | Flash access stubs |
| `flash_range_erase` | Erase flash sectors (fills with 0xFF) |
| `flash_range_program` | Program flash bytes |

**Soft-Float/Double Interception**:
- ROM data tables with codes `'SF'` (single-float) and `'SD'` (double-float)
- BX LR stubs at `0x0500-0x0567`
- Intercepted by PC check in `cpu_step()`: native C float/double operations execute instead of Thumb code
- Covers: add, sub, mul, div, sqrt, float↔int conversions, float↔double conversions, comparisons

**Flash Write Interception**:
- `flash_range_erase`: fills specified flash region with `0xFF`
- `flash_range_program`: copies bytes into flash
- Both trigger `flash_persist_sync()` for immediate write-through to disk

## 6.2 Boot2 Detection

Boot2 is the second-stage bootloader stored in the first 256 bytes of flash. Bramble auto-detects it by:
1. Checking first flash word for valid ARM vector (reasonable SP value)
2. Verifying vector table validity

When boot2 is present:
- Core 0 starts execution at `0x10000000` (boot2 entry)
- Boot2 configures XIP and jumps to application at `0x10000100`

When boot2 is absent or `-no-boot2` is specified:
- Core 0 starts directly at `0x10000100` (application entry)

## 6.3 Firmware Loading

**UF2 Format** (uf2.c):
- Parses UF2 blocks (512 bytes each, payload up to 476 bytes)
- Validates all magic numbers, payload bounds, and target range
- Rejects malformed or overflowed block writes without modifying flash
- Writes payload to flash at specified target addresses

**ELF Format** (elf.c):
- Parses ELF32 ARM headers
- Validates PT_LOAD segment bounds and requires `p_filesz <= p_memsz`
- Loads PROGBITS segments to their target addresses (flash or RAM)
- Returns 1 on success, 0 on failure


# Part 7: Debugging

## 7.1 GDB Remote Debugging (gdb.c / gdb.h)

Launch with `-gdb [port]` (default port: 3333).

```bash
# Terminal 1: Start emulator with GDB server
./bramble firmware.uf2 -gdb

# Terminal 2: Connect GDB
arm-none-eabi-gdb firmware.elf
(gdb) target remote :3333
(gdb) break main
(gdb) continue
```

**Supported GDB Commands**:

| Command | Description |
|---------|-------------|
| `g` / `G` | Read/write all registers (R0-R15 + xPSR, LE hex) |
| `p` / `P` | Read/write single register |
| `m` / `M` | Read/write memory |
| `c` | Continue execution |
| `s` | Single step |
| `Z0` / `z0` | Set/remove software breakpoint |
| `Z1` / `z1` | Set/remove hardware breakpoint |
| `Z2` / `z2` | Set/remove write watchpoint |
| `Z3` / `z3` | Set/remove read watchpoint |
| `Z4` / `z4` | Set/remove access watchpoint |
| `vCont` | Continue/step with thread selection |
| `Hg` / `Hc` | Set thread for subsequent operations |
| `qfThreadInfo` | List threads (returns `m1,2` for dual-core) |
| `Ctrl-C` | Interrupt execution |

**Capabilities**:
- 16 breakpoint slots, 16 watchpoint slots
- Dual-core thread support: Thread 1 = Core 0, Thread 2 = Core 1
- Stop replies include thread ID: `T05thread:N;`
- Watchpoint stop replies: `T05watch:addr;thread:N;` (write), `T05rwatch:addr;` (read), `T05awatch:addr;` (access)

**Conditional Breakpoints** (via `qRcmd` monitor commands):
```
(gdb) monitor cond 0 r0==0x1234    # Break at BP 0 only when R0 == 0x1234
(gdb) monitor uncond 0             # Remove condition from BP 0
```

## 7.2 Debug Output Flags

| Flag | Effect |
|------|--------|
| `-debug` | Enables `cpu.debug_enabled` for Core 0: logs exceptions, IRQ delivery, peripheral activity |
| `-debug1` | Enables debug for Core 1 |
| `-debug-mem` | Logs unmapped peripheral accesses (helps find unimplemented peripherals) |
| `-status` | Periodic status: PC, SP, FIFO counts, core state |

All diagnostic output goes to stderr. Only firmware UART/USB output appears on stdout.


# Part 8: Storage and Persistence

## 8.1 Flash Persistence

```bash
./bramble firmware.uf2 -flash storage.bin
```

- On startup: loads flash file, restores non-firmware sectors (filesystem data)
- Smart sector detection: 4 KB sectors compared to 0xFF to distinguish firmware from filesystem
- On exit: saves full 2 MB flash image
- Write-through: every `flash_range_erase` and `flash_range_program` immediately syncs to disk via `flash_persist_sync()`

## 8.2 SD Card

```bash
./bramble firmware.uf2 -sdcard sdcard.img -sdcard-size 64
```

SPI-mode SD card emulation:
- SDHC protocol with CSD v2.0, CID (product name "BRMSD")
- Commands: CMD0, CMD8, CMD9, CMD10, CMD12, CMD13, CMD16, CMD17, CMD18, CMD24, CMD25, CMD55, CMD58, ACMD41
- Single-block and multi-block read/write
- File-backed: loaded on init, flushed periodically (~1M steps) and on cleanup
- Attaches to SPI1 by default via `spi_attach_device()` callback

## 8.3 eMMC

```bash
./bramble firmware.uf2 -emmc emmc.img -emmc-size 128
```

SPI-mode eMMC emulation:
- CMD1 initialization (no ACMD41), CSD_STRUCTURE=3
- EXT_CSD via CMD8 (512 bytes)
- Sector addressing, product name "BRMMC"
- Default 128 MB, file-backed

## 8.4 FUSE Mount

```bash
./bramble firmware.uf2 -flash storage.bin -mount /tmp/pico-fs
```

Mounts the flash FAT16 filesystem as a host directory via libfuse3:
- Requires `-flash <path>` for the backing store
- Thread-safe: mutex serializes FUSE operations against emulator flash writes
- FUSE writes automatically sync to disk via flash persistence
- Build with `cmake .. -DENABLE_FUSE=ON`


# Part 9: Networking and Multi-Device

## 9.1 UART-to-TCP Bridge

```bash
# Server mode: listen for connections
./bramble firmware.uf2 -net-uart0 8888

# Client mode: connect to remote
./bramble firmware.uf2 -net-uart0-connect 192.168.1.100:8888
```

- Non-blocking I/O with `TCP_NODELAY` for low-latency byte-at-a-time transfer
- Automatic client accept/disconnect; reconnection in listen mode
- UART TX routed through bridge when active (instead of stdout)
- Both UART0 and UART1 supported independently

## 9.2 Wire Protocol (Multi-Instance IPC)

```bash
# Instance 1: wire UART0 to socket
./bramble firmware1.uf2 -wire-uart0 /tmp/uart-link

# Instance 2: wire UART0 to same socket (auto-negotiates peer connection)
./bramble firmware2.uf2 -wire-uart0 /tmp/uart-link
```

- Unix domain socket IPC between Bramble instances
- First instance creates socket (listen), second connects
- UART TX on one instance delivered as UART RX on the other
- GPIO pin changes propagated between instances
- Stream reads and writes handle partial I/O safely on `SOCK_STREAM`
- Wire message protocol: 4-byte header (type, channel, length, reserved) + payload
- Message types: `WIRE_MSG_UART_DATA`, `WIRE_MSG_GPIO_PIN`, `WIRE_MSG_SPI_XFER`

## 9.3 CYW43 WiFi Emulation

```bash
./bramble firmware.uf2 -wifi
```

CYW43439 WiFi chip emulation for Pico W firmware:
- gSPI protocol via PIO0 SM0 FIFO intercept
- Command word: `[31]=write, [30:28]=function, [27:17]=address, [16:0]=size`
- Three bus functions:
  - Function 0 (BUS): `FEEDBEAD` magic, test register
  - Function 1 (BACKPLANE): chip ID, register window
  - Function 2 (WLAN): WiFi data
- Configurable scan results: `cyw43_add_scan_result()` adds fake access points


# Part 10: Performance

## 10.1 Instruction Cache

64K-entry direct-mapped decoded instruction cache:
- Indexed by PC (lower 16 bits)
- Stores pre-decoded handler function pointer + raw 16-bit instruction
- Avoids `mem_read16()` + dispatch table lookup on cache hits
- Invalidated on RAM writes; flash/ROM entries never invalidated
- Hit rate statistics reported on exit

## 10.2 JIT Basic Block Compilation

```bash
./bramble firmware.uf2 -jit
```

16384-entry basic block cache:
- Compiles consecutive instruction sequences into blocks (max 64 instructions)
- Blocks terminate at branches, 32-bit instructions, or the 64-instruction limit
- Block execution skips per-instruction PC bounds check, interrupt check, and dispatch lookup
- Pre-computed per-instruction cycle costs stored in block (avoids timing LUT lookup)
- Only compiles flash/ROM code (immutable — no invalidation needed)
- Single-instruction blocks fall back to normal dispatch (zero overhead)
- ~1.50x speedup over instruction cache alone
- Statistics reported on exit: block compiles, executions, accelerated instructions

## 10.3 Host Threading

```bash
./bramble firmware.uf2 -cores 2
```

- One host pthread per emulated core
- Big lock (mutex) for shared state access
- WFI/WFE: releases lock, sleeps on condition variable (zero CPU usage while idle)
- `corepool_wake_cores()` signals sleeping cores on interrupt delivery
- Core pool: file-based registry at `/tmp/bramble-corepool.reg`
- `-cores auto`: queries pool for optimal allocation across multiple instances


# Part 11: Tested Firmware

## 11.1 MicroPython

```bash
./bramble micropython.uf2 -stdin -clock 125 -flash mpy.bin
```

MicroPython v1.27.0 boots to interactive REPL via USB CDC stdio. Requires boot2 support, USB enumeration, ROM 16KB, XIP SSI aliases.

## 11.2 CircuitPython

```bash
./bramble circuitpython.uf2 -stdin -clock 125
```

CircuitPython 10.1.3 boots and runs `code.py` via USB CDC stdio. First boot creates FAT filesystem (64 flash operations at offset `0x100000`). Requires ROSC, SIO_GPIO_HI_IN (QSPI), CDC DTR signaling.

## 11.3 littleOS

```bash
./bramble littleos.uf2 -stdin -clock 125 -flash los.bin
```

Full Pico SDK 2.x OS with interactive shell, SageLang interpreter, and dual-core supervisor. Boots to interactive shell. Requires per-core NVIC (timer IRQ filtered by per-core enable mask), per-core exception nesting, timer core-gating.

## 11.4 Test Firmware Suite

| Firmware | Purpose |
|----------|---------|
| `hello_world.uf2` | UART output verification |
| `gpio_test.uf2` | GPIO toggle and pin state |
| `timer_test.uf2` | Timer measurement accuracy |
| `alarm_test.uf2` | Timer alarm and interrupt delivery |
| `interrupt_test.uf2` | Exception handling and nesting |


# Part 12: Atomic Register Aliases

All RP2040 peripherals support atomic register access via address aliases:

| Alias | Offset | Operation | Example |
|-------|--------|-----------|---------|
| Normal | `+0x0000` | Direct read/write | `*(reg) = value` |
| XOR | `+0x1000` | Atomic XOR | `*(reg) ^= value` |
| SET | `+0x2000` | Atomic bit set | `*(reg) \|= value` |
| CLR | `+0x3000` | Atomic bit clear | `*(reg) &= ~value` |

These aliases work for all peripheral registers in the `0x40000000-0x50FFFFFF` range and the XIP SSI at `0x18000000`. The Pico SDK uses these extensively via `hw_set_bits()`, `hw_clear_bits()`, and `hw_xor_bits()`.

**Note**: The SRAM alias at `0x21000000` is a simple mirror of `0x20000000` — not an atomic alias. Atomic operations are peripheral-only.


# Part 13: Repository Structure

```
bramble/
├── CMakeLists.txt          # Build configuration
├── src/
│   ├── main.c              # Entry point, CLI parsing, execution loops
│   ├── cpu.c               # CPU engine, dual-core, exceptions, timing
│   ├── instructions.c      # 65+ Thumb instruction handlers
│   ├── membus.c            # Memory bus routing
│   ├── gpio.c              # GPIO peripheral
│   ├── timer.c             # 64-bit timer + alarms
│   ├── nvic.c              # Per-core NVIC + SysTick
│   ├── clocks.c            # Clocks, Resets, XOSC, PLLs, ROSC, Watchdog
│   ├── adc.c               # ADC peripheral
│   ├── uart.c              # Dual UART
│   ├── spi.c               # Dual SPI
│   ├── i2c.c               # Dual I2C
│   ├── pwm.c               # 8-slice PWM
│   ├── dma.c               # 12-channel DMA
│   ├── pio.c               # Dual PIO blocks
│   ├── usb.c               # USB controller
│   ├── rtc.c               # Real-time clock
│   ├── rom.c               # ROM emulation
│   ├── gdb.c               # GDB RSP server
│   ├── netbridge.c         # UART-to-TCP bridge
│   ├── wire.c              # Unix socket IPC
│   ├── storage.c           # Flash write-through
│   ├── sdcard.c            # SD card emulation
│   ├── emmc.c              # eMMC emulation
│   ├── cyw43.c             # CYW43 WiFi emulation
│   ├── corepool.c          # Host threading + core pool
│   ├── uf2.c               # UF2 loader
│   └── elf.c               # ELF loader
├── include/
│   ├── emulator.h          # Core structures, constants, memory map
│   ├── gpio.h              # GPIO register definitions
│   ├── timer.h             # Timer register definitions
│   ├── nvic.h              # NVIC/SysTick/SCB definitions, IRQ numbers
│   ├── clocks.h            # Clock peripheral definitions
│   ├── adc.h               # ADC register definitions
│   ├── uart.h              # UART register definitions
│   ├── spi.h               # SPI register definitions
│   ├── i2c.h               # I2C register definitions
│   ├── pwm.h               # PWM register definitions
│   ├── dma.h               # DMA register definitions
│   ├── pio.h               # PIO register definitions
│   ├── usb.h               # USB register definitions
│   ├── rtc.h               # RTC register definitions
│   ├── rom.h               # ROM definitions
│   ├── gdb.h               # GDB protocol definitions
│   ├── netbridge.h         # Network bridge definitions
│   ├── wire.h              # Wire protocol definitions
│   ├── storage.h           # Storage persistence definitions
│   ├── sdcard.h            # SD card definitions
│   ├── emmc.h              # eMMC definitions
│   ├── cyw43.h             # CYW43 definitions
│   ├── corepool.h          # Core pool definitions
│   ├── uf2.h               # UF2 format definitions
│   └── elf.h               # ELF format definitions
├── tests/                  # 274 automated tests
├── docs/
│   ├── GPIO.md             # GPIO behavior and register notes
│   ├── NVIC_audit_report.md # Historical NVIC audit (resolved issues)
│   └── ROADMAP.md          # Development roadmap and feature tracker
├── build/                  # Build output directory
└── README.md               # Top-level usage and project overview
```


# Part 14: Peripheral Stub Summary

Some peripherals have minimal implementations sufficient for firmware boot:

| Peripheral | Base Address | Behavior |
|------------|-------------|----------|
| SYSINFO | `0x40000000` | CHIP_ID returns RP2040-B2, PLATFORM=ASIC |
| IO_QSPI | `0x40018000` | 6 QSPI GPIO pins with STATUS/CTRL + interrupt registers |
| PADS_QSPI | `0x40020000` | QSPI pad electrical control |
| XIP Cache | `0x14000000` | Cache always reports ready/empty |
| XIP SSI | `0x18000000` | Flash read command (0x03) with TX/RX FIFOs |


# Part 15: Design Decisions and Trade-Offs

## 15.1 Correctness Over Performance

Bramble prioritizes correctness against the RP2040 datasheet over raw emulation speed:
- Per-core NVIC state faithfully models how shared interrupt lines are filtered by independent enable masks
- Signed alarm comparison handles 32-bit wrap correctly
- Exception nesting stack (not just a single `current_irq`) enables proper nested interrupt handling
- Double-fault lockup detection prevents infinite HardFault recursion

## 15.2 Synchronous DMA

DMA transfers execute immediately (synchronously) rather than cycle-by-cycle:
- Simpler implementation, deterministic behavior
- Firmware that depends on DMA transfer timing (pacing timers) is not supported
- Sufficient for all tested firmware (MicroPython, CircuitPython, littleOS)

## 15.3 Immediate PIO Execution

PIO state machines step once per main loop iteration:
- Clock dividers use 16.8 fixed-point accumulators for fractional timing
- Sufficient for UART, SPI, I2C PIO programs
- Very high-speed PIO programs may not match real-hardware timing

## 15.4 Global Peripheral State

All peripherals use global state (single emulated chip):
- Multi-chip systems require separate Bramble processes connected via wire protocol
- This keeps the code simple and matches the RP2040's single-chip architecture

## 15.5 Output Separation

- All firmware output (UART TX, USB CDC) goes to **stdout**
- All emulator diagnostics go to **stderr**
- This enables clean piping: `./bramble firmware.uf2 > output.txt` captures only firmware output


# Part 16: Versioning and Release History

Version source of truth: `CHANGELOG.md` (including the `Unreleased` section for work on `main`).

| Version | Key Features |
|---------|-------------|
| v0.36.0 | FUSE flash mutex fix, TAP partial write/MTU hardening, configurable mount offset, FUSE no longer needs sudo |
| v0.35.0 | Advanced devtools: symbols, callgraph, VCD, IRQ latency, stack check, bus logging, script I/O, expect, watch, fault injection, cycle profile, heatmap |
| v0.34.0 | Developer tools (semihosting, coverage, hotspots, trace, exit codes, timeouts), SYSCFG + TBMAN peripherals, JIT fixes |
| v0.33.0 | Auto-sudo for `-tap`/`-mount`, watchdog reboot resets full multicore state, SysTick reset on reboot |
| v0.31.0 | CYW43/Pico W support, TAP bridge, JIT basic-block compilation, benchmarked speedups |
| v0.30.0 | GDB watchpoints + conditional breakpoints, dual-core GDB threads, decoded instruction cache, double-fault lockup detection |
| v0.29.0 | Per-core NVIC + SysTick behavior, per-core exception nesting, timer core-gating, littleOS boots |
| v0.28.0 | Host-threaded execution (pthread-per-core), `-cores N\|auto`, core pool registry |
| v0.27.0 | Flash write-through persistence, SD card SPI, eMMC SPI, FAT16 module, FUSE mount |
| v0.26.0 | UART-to-TCP bridge, SPI FIFOs+device callbacks, I2C FIFO+device callbacks, wire protocol |
| v0.25.0 | CircuitPython support (ROSC, SIO_GPIO_HI_IN, CDC DTR fix) |
| v0.24.0 | MicroPython support (USB buf_ctrl fix, DPRAM byte access, ROM 16KB, XIP SSI) |
| v0.23.0 | All printf→stderr separation, diagnostics gated behind `-debug` |
| v0.22.0 | CPUID register, NVIC IABR + full IPR range, RTC time ticking |
| v0.21.0 | USB host enumeration, USB CDC data bridge, flash persistence |
| v0.20.0 | GPIO edge/level interrupts, PIO INTR from FIFO status, dynamic FC0_RESULT |
| v0.19.0 | ROM soft-float/double, flash write, HardFault, exception nesting, SIO interpolators |


# Part 17: External References

- RP2040 Datasheet: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
- ARM Cortex-M0+ Technical Reference Manual: https://developer.arm.com/documentation/ddi0484/
- ARMv6-M Architecture Reference Manual: https://developer.arm.com/documentation/ddi0419/
- Pico SDK Documentation: https://www.raspberrypi.com/documentation/pico-sdk/
- MicroPython: https://micropython.org/
- CircuitPython: https://circuitpython.org/
- PL011 UART Technical Reference: https://developer.arm.com/documentation/ddi0183/
- PL022 SPI Technical Reference: https://developer.arm.com/documentation/ddi0194/
- DesignWare I2C Databook (Synopsys DW_apb_i2c)
- CYW43439 Datasheet (Infineon/Cypress)
- GDB Remote Serial Protocol: https://sourceware.org/gdb/current/onlinedocs/gdb.html/Remote-Protocol.html
- UF2 Specification: https://github.com/microsoft/uf2
- CMake Documentation: https://cmake.org/cmake/help/latest/


# Part 18: Closing Notes

Bramble is designed as a practical development tool for the RP2040 ecosystem:
- add peripheral modules as new firmware requires them
- validate correctness with automated tests (276 and growing)
- maintain register-level fidelity against the RP2040 datasheet
- provide a debuggable environment that boots real firmware unmodified

The fastest path to broader firmware compatibility is identifying unimplemented peripheral accesses (via `-debug-mem`), adding the missing register handlers, and locking behavior with tests.
