# Bramble RP2040/RP2350 Emulator - Changelog

## [0.38.0] - 2026-03-20

### Added - RP2350 RISC-V Full Integration (CLINT, Memory Bus, Bootrom, Firmware Auto-Detect)

- **RISC-V CLINT interrupt controller** (`src/rp2350_rv/rv_clint.c`): Machine timer (mtime/mtimecmp, 64-bit), software interrupts (MSIP per hart), external interrupt aggregation. Memory-mapped at 0xD0000100 in SIO space. Supports vectored and direct mtvec modes.
- **RP2350 memory bus** (`src/rp2350_rv/rv_membus.c`): 520KB SRAM (0x20000000-0x20082000) with alias support, 32KB ROM, flash access, CLINT routing. Falls through to shared RP2040 peripheral bus for UART/SPI/I2C/GPIO/etc.
- **RP2350 RISC-V bootrom** (`src/rp2350_rv/rv_bootrom.c`): Generated RISC-V code in 32KB ROM — sets SP to top of SRAM (0x20082000), sets GP to SRAM base, jumps to flash entry (0x10000000). Trap handler at 0x0004 loops on unhandled traps.
- **Dual-hart execution**: Cooperative step loop for both harts. Hart 1 halted until firmware launches. WFI support with interrupt wake.
- **CLINT mtime ticking**: Converts CPU cycles to microseconds via configurable cycles_per_us, matching `-clock` setting.
- **CLINT interrupt delivery**: Checks mip/mie/mstatus.MIE each step. Priority: MEIP > MSIP > MTIP. WFI wake on any pending+enabled interrupt.
- **UF2 family ID detection**: Reads UF2 flags bit 13 (family present) and identifies RP2040 (0xE48BFF56), RP2350-ARM (0xE48BFF59), RP2350-RV (0xE48BFF5A).
- **ELF RISC-V support**: Accepts EM_RISCV (243) ELF32 binaries in addition to EM_ARM (40).
- **Firmware auto-detection**: When `-arch` not specified, auto-detects architecture from UF2 family ID or ELF machine type and switches execution mode automatically.
- **RV CPU memory routing**: All memory accesses in `rv_cpu_step()` now route through RP2350 memory bus (520KB SRAM, ROM, CLINT) instead of RP2040 global membus.
- **Banner after load**: Architecture banner displayed after firmware loading and auto-detection, so it always reflects the correct mode.

### Files Added

- `include/rp2350_rv/rv_clint.h` — CLINT state, interrupt constants, API
- `src/rp2350_rv/rv_clint.c` — CLINT timer, register access, interrupt delivery
- `include/rp2350_rv/rv_membus.h` — RP2350 memory bus state and API
- `src/rp2350_rv/rv_membus.c` — 520KB SRAM, ROM, CLINT, peripheral routing
- `include/rp2350_rv/rv_bootrom.h` — Bootrom generator API
- `src/rp2350_rv/rv_bootrom.c` — RISC-V bootrom code generation

### Tests

- 276 tests passing, zero warnings. RP2040 tests unaffected by RP2350 changes.

---

## [0.37.0] - 2026-03-20

### Added - RP2350 RISC-V Hazard3 Support (Phase 1+2), Complete Exception Model

- **RISC-V Hazard3 CPU engine** (`src/rp2350_rv/rv_cpu.c`): Full RV32I base integer instruction set (37 instructions) plus M extension (multiply/divide). Separate architecture directory with clean ISA abstraction.
- **RV32I instructions**: LUI, AUIPC, JAL, JALR, BEQ/BNE/BLT/BGE/BLTU/BGEU, LB/LH/LW/LBU/LHU, SB/SH/SW, ADDI/SLTI/SLTIU/XORI/ORI/ANDI/SLLI/SRLI/SRAI, ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR/AND, FENCE, ECALL, EBREAK.
- **RV32M extension**: MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU with correct edge cases (div-by-zero returns 0xFFFFFFFF, INT_MIN/-1 wraps).
- **RV32C compressed extension**: Full 16-bit compressed instruction set — C.LW, C.SW, C.LWSP, C.SWSP, C.ADDI, C.LI, C.LUI, C.ADDI16SP, C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND, C.J, C.JAL, C.JR, C.JALR, C.BEQZ, C.BNEZ, C.SLLI, C.MV, C.ADD, C.ADDI4SPN, C.EBREAK, C.NOP. All three quadrants (Q0/Q1/Q2) decoded.
- **RV32A atomic extension**: LR.W (load-reserved), SC.W (store-conditional), AMOSWAP.W, AMOADD.W, AMOAND.W, AMOOR.W, AMOXOR.W, AMOMIN.W, AMOMAX.W, AMOMINU.W, AMOMAXU.W. Reservation tracking with per-hart valid/address state.
- **`-arch` flag**: Runtime ISA selection — `-arch m0+` (default, RP2040 ARM) or `-arch rv32` (RP2350 Hazard3 RISC-V). Banner and execution loop adapt to selected architecture.
- **RP2350 memory map**: 520KB SRAM, up to 16MB flash, new peripheral addresses defined in `rp2350_memmap.h`.
- **RP2350 peripheral stubs**: TRNG (xorshift random), SHA-256, OTP (blank), HSTX, TICKS — wired into membus for read/write access.
- **RISC-V execution loop**: Single-hart cooperative loop in `main.c` with stdin polling, timeout, and semihosting support.
- **RISC-V CSR support**: CSRRW/CSRRS/CSRRC + immediate variants. Tracks mstatus, misa, mie, mip, mtvec, mepc, mcause, mtval, mscratch, mhartid, mcycle, minstret.
- **RISC-V trap handling**: Direct + vectored mtvec modes, MIE/MPIE/MPP save/restore, MRET instruction, alignment fault traps.
- **Exception tail-chaining**: On exception return, checks for pending higher-priority exceptions and skips unstack/restack (ARMv6-M optimization).
- **Late-arriving interrupts**: During exception stacking, detects higher-priority pending IRQ and switches handler without re-stacking.
- **FAULTMASK register**: MSR/MRS support for SYSm=0x13, blocks all exceptions except NMI, auto-clears on exception return. Context-switched across cores.
- **VREG_AND_CHIP_RESET full implementation**: Replaces stub with real register set (VREG EN/VSEL/ROK, BOD EN/VSEL, CHIP_RESET had_por/had_run/had_psm W1C flags).
- **Multi-architecture directory structure**: `src/rp2040/`, `src/rp2350_rv/`, `src/rp2350_arm/` (placeholder).

### Files Added

- `include/rp2350_rv/rv_cpu.h` — RISC-V CPU state, CSR definitions, instruction decode helpers
- `src/rp2350_rv/rv_cpu.c` — RV32I+M fetch/decode/execute engine, CSR access, trap handling
- `include/rp2350_arm/m33_cpu.h` — Cortex-M33 placeholder

### Tests

- 276 tests passing, zero warnings. RP2040 tests unaffected by new RV code.

---

## [0.36.0] - 2026-03-20

### Added - TAP Auto-Configuration, FAT Auto-Scan, and Warning-Free Build

- **TAP full host-side configuration**: `tapif_open()` now automatically assigns 192.168.4.1/24 to the TAP interface, brings it UP, enables IP forwarding, and sets up NAT masquerade via iptables/nft. On close, NAT rules are removed and forwarding is restored. No manual `ip addr add` needed.
- **FAT partition auto-scan**: `-mount` without `-mount-offset` now scans the entire `cpu.flash[]` (UF2 firmware + persisted data) for a valid FAT BPB (0x55AA signature + valid geometry). Automatically finds CircuitPython's FAT12 at 0x181000 or any other offset.
- **`-mount` works without `-flash`**: Volatile mount from UF2-loaded flash data (changes lost on exit). With `-flash`, changes persist across runs.

### Fixed

- **Flash initialized to 0xFF**: `cpu.flash[]` now starts erased (all 0xFF) matching real RP2040 hardware, fixing flash persistence sector detection.
- **FAT12 filesystem support**: FAT driver handles FAT12 (12-bit packed entries, EOC 0xFF8) in addition to FAT16.
- **FUSE flash mutex race**: ROM flash writes now lock `fuse_flash_mutex`.
- **TAP persistent read errors**: `cyw43_tap_poll()` closes TAP fd on error instead of spin-polling.
- **CYW43 PIO buffer overflow**: `pio_resp_buf` copy length clamped to buffer size.
- **All compiler warnings resolved**: Fixed unused variables (`addr` in thumb32, `raw_best_ms` in benchmark), unused parameters (`ifname` in setup_nat), uninitialized variables (`off` in devtools), `strncpy` truncation (replaced with `memcpy`+`memset`), suppressed GCC 15 false-positive `stringop-overflow` from system headers.

### Changed

- **`build.sh` rewritten**: Auto-detects FUSE, supports `--clean`/`--no-fuse`/`--release`/`--debug`/`--help`. No longer destructive (no `git pull` or `rm -rf`).
- **CMakeLists.txt updated**: Version 0.36.0, FUSE definitions applied to all targets, deduplicated source lists, build summary output.

### Tests

- 276 tests passing, zero compiler warnings.

---

## [0.35.0] - 2026-03-20

### Added - Advanced Developer Tools

- **ELF symbol loading** (`-symbols <elf>`): Parses .symtab/.strtab from ELF32 files for function name resolution in hotspots, profiles, callgraphs, crash reports, and memory watch logs.
- **Call graph** (`-callgraph <file>`): Tracks BL/BLX calls to build caller→callee graph with call counts. DOT format output for Graphviz visualization.
- **Stack watermark** (`-stack-check`): Per-core SP high-water mark tracking. Reports peak stack depth on exit with warnings if SP approaches RAM base.
- **IRQ latency profiling** (`-irq-latency`): Measures cycles from `nvic_signal_irq()` to exception handler entry. Reports min/avg/max per IRQ number.
- **Bus transaction logging** (`-log-uart`, `-log-spi`, `-log-i2c`): Logs every byte TX/RX with hex and printable representation to stderr.
- **GPIO VCD trace** (`-gpio-trace <file>`): Records all GPIO pin changes with cycle-accurate timestamps in Value Change Dump format for GTKWave/PulseView.
- **Scripted I/O** (`-script <file>`): Timestamped UART/GPIO input injection from text files (`100ms: uart0 "hello\n"`, `200ms: gpio25 1`).
- **Expected output matching** (`-expect <file>`): Captures stdout (UART + USB CDC) and compares against golden file on exit. Exit 0 on match, 1 on diff with byte-level diff location.
- **Memory watch log** (`-watch <addr[:len]>`): Logs every read/write to an address range to stderr with PC and symbol context. Up to 8 simultaneous regions.
- **Fault injection** (`-inject-fault <spec>`): Schedule hardware faults at cycle counts: `flash_bitflip:cycle:addr`, `ram_corrupt:cycle:addr`, `brownout:cycle`.
- **Cycle profiling** (`-profile <file>`): Per-PC cycle accounting with CSV output. Combined with `-symbols`, gives per-function timing breakdown.
- **Memory heatmap** (`-mem-heatmap <file>`): Tracks read/write frequency per 256-byte RAM block. CSV output for visualization.

### Files Modified

- `include/devtools.h` + `src/devtools.c` — All new tools and symbol table loader
- `src/thumb32.c` — Callgraph recording on BL instructions
- `src/nvic.c` — IRQ latency pend timestamp recording
- `src/uart.c` — UART TX bus logging and expect capture hooks
- `src/usb.c` — USB CDC expect capture hook
- `src/gpio.c` — GPIO VCD trace on SIO output writes
- `src/membus.c` — Memory watch and heatmap hooks on RAM read/write fast paths
- `src/cpu.c` — Profile, stack check, IRQ cycle counter in cpu_step hot path

### Tests

- 276 tests passing. No performance regression (all tools gated behind `__builtin_expect(flag, 0)`).

---

## [0.34.0] - 2026-03-20

### Added - Developer Tools, Missing Peripherals, and Performance Audit

- **SYSCFG peripheral** (0x40004000): NMI mask, proc config, debug force, memory power-down registers. Fully wired with atomic aliases.
- **TBMAN peripheral** (0x4006C000): Testbench Manager returns PLATFORM=ASIC (0x01) matching real RP2040 hardware.
- **ARM semihosting** (`-semihosting`): Intercepts BKPT #0xAB for SYS_WRITEC/WRITE0/WRITE/READC/EXIT/EXIT_EXTENDED. Enables test frameworks (Unity, CppUTest) and newlib rdimon specs.
- **Code coverage** (`-coverage <file>`): Bitmap tracking executed 16-bit-aligned PCs across flash and RAM. Dumps binary file with header on exit; reports summary statistics.
- **Hotspot profiling** (`-hotspots [N]`): 256K-entry hash map counting per-PC execution frequency. Reports top-N addresses by region (flash/rom/ram) on exit.
- **Instruction trace** (`-trace <file>`): Streams (PC, opcode, cycles) tuples as 8-byte binary records for offline analysis. Binary format with header.
- **Exit code hook** (`-exit-code <addr>`): Reads uint32 from a RAM address on halt and uses it as the process exit code. Enables CI test result reporting.
- **Timeout enforcement** (`-timeout <seconds>`): Wall-clock limit via SIGALRM. Returns exit code 124 (GNU timeout convention) when exceeded.

### Fixed

- **JIT timing undercount**: `timing_lut[]` returns 0 for dynamic-cost instructions (PUSH/POP/LDMIA/STMIA). Non-terminal JIT instructions were accumulating 0 cycles, starving SysTick and hanging littleOS benchmarks. Fix: fall back to `timing_instruction_cycles()` when LUT returns 0.
- **JIT invalidation O(16K) per RAM write**: `jit_invalidate_addr()` had an inverted early-return — it scanned all 16,384 blocks for every STR to RAM despite JIT only compiling flash/ROM code. This caused `memcpy` to hang. Fix: early-return for RAM addresses.
- **JIT CBZ/CBNZ not terminal**: CBZ (0xB1/0xB3) and CBNZ (0xB9/0xBB) were missing from the JIT terminal set, allowing blocks to execute past conditional branches.
- **JIT compile bounds check**: Used `&&` instead of `||` for ROM/flash range validation during block compilation.

### Performance

- JIT block max: 32→64 instructions; removed `__attribute__((packed))` from block struct.
- JIT terminal check: bitmap lookup replaces 9 sequential if-else branches.
- membus: RAM read/write promoted to first check (before flash/XIP). GDB watchpoint calls gated with `__builtin_expect(gdb.active, 0)`.
- Synthetic benchmark: 182→161 MIPS (JIT, regression from 64-insn blocks but real workloads benefit).
- littleOS JIT: was HUNG, now completes in 3447ms (matches no-JIT exactly).

### Files Added

- `include/devtools.h` + `src/devtools.c` — Developer tools (semihosting, coverage, hotspots, trace, SYSCFG, TBMAN)

### Tests

- 276 tests passing.

---

## [0.33.0] - 2026-03-20

### Added - Privilege Escalation, Watchdog Hardening, and Dual-Core Correctness

- Automatic privilege escalation: `-tap` and `-mount` flags now detect when root is needed, explain why, and re-exec via `sudo` with `BRAMBLE_ESCALATED` env guard to prevent loops. Falls back with a manual command hint on failure.
- Help text annotates privileged flags with `(sudo)` so users know before running.
- Watchdog reboot now fully resets multicore state: `num_active_cores` reset to 1, Core 1 bootrom launch state machine cleared, spinlocks and shared RAM zeroed, `active_core` reset to CORE0. Firmware must re-launch Core 1 through the standard FIFO protocol after reboot.
- `nvic_init()` now calls `systick_reset()` so both cores' SysTick counters are properly cleared on watchdog reboot and peripheral re-initialization.
- `dual_core_init()` resets the Core 1 bootrom state machine (`waiting_for_launch`, `launch_count`) preventing stale launch state from surviving across reboots.

### Fixed

- Watchdog reboot left `num_active_cores = 2` from Core 1 auto-launch, causing the emulator to step a halted Core 1 after reboot instead of letting firmware re-launch it.
- Watchdog reboot did not clear spinlocks, shared RAM, or the Core 1 bootrom handshake state, potentially causing firmware to see stale inter-core state.
- SysTick state survived watchdog reboot because `systick_reset()` was never called from `nvic_init()` or the peripheral reset path.
- Test `test_num_active_cores_default` was invalidated by `setup()` setting `num_active_cores = MAX_CORES` before the test ran.

### Tests

- 276 tests passing (was 274), including watchdog multicore reset correctness and updated core pool assertions.

---

## [Unreleased - pre-0.33.0] - 2026-03-20

### Changed - Maintenance, Hardening, and Documentation Consolidation

- Hardened the UF2 loader against oversized payloads and overflowed flash writes.
- Hardened ELF `PT_LOAD` handling with overflow-safe bounds checks and `p_filesz <= p_memsz` validation.
- Watchdog and `SYSRESETREQ` reboot paths now reinitialize the full runtime peripheral set instead of a partial reset.
- Core pool registry updates now hold a single lock window for read/modify/write operations.
- Wire transport now handles partial `SOCK_STREAM` reads and writes safely without desynchronizing frames.
- `-stdin` now stages host input until a guest console is ready, preserving littleOS interactive shells while still delivering early piped or interactive input to USB CDC firmware such as MicroPython.
- Consolidated the old `UPDATES.md` history into this file and refreshed `README.md`, `Bramble_Guide.md`, and `docs/`.
- Added end-to-end regression coverage for memory-mapped alias paths including flash XIP aliases, NVIC subword MMIO, XIP SSI, IO_QSPI, PADS_QSPI, and BUSCTRL.
- Added exception-path regression coverage for SVCall entry/return, IRQ delivery through `cpu_step()`, nested exception unwind, invalid-PC HardFault entry, and double-HardFault lockup.
- Fixed the remaining `PADS_QSPI` decode overlap so QSPI pad accesses no longer fall through the generic `PADS_BANK0` path.
- Tightened the CPU hot path with direct ROM/flash/RAM halfword fetches and earlier JIT block dispatch, and added a tunable `-thread-quantum` for host-threaded execution.

### Tests

- 274 tests passing, including loader hardening, watchdog reset, core pool, wire transport, USB CDC RX readiness, stdin-routing regressions, memory-map alias coverage, and exception-path coverage

---

## [0.31.0] - 2026-03-13

### Added - Pico W Connectivity and JIT Acceleration

- CYW43 / Pico W emulation with PIO-backed gSPI transport, WLAN framing, and fake scan/connect flows
- TAP bridge support for moving emulated WLAN Ethernet frames onto a host TAP interface
- JIT basic-block compilation for hot flash/ROM paths (`-jit`) with execution statistics on exit
- Benchmark coverage for instruction-cache and JIT performance comparisons

### Files Added

- `include/cyw43.h` + `src/cyw43.c` - CYW43 WiFi emulation
- `include/tapif.h` + `src/tapif.c` - TAP bridge support
- `tests/benchmark.c` - Performance benchmark harness

### Files Modified

- `src/main.c` - `-wifi`, `-tap`, and `-jit` CLI handling
- `src/pio.c` - CYW43 gSPI FIFO interception and auto-detection
- `src/cpu.c` - JIT block cache and runtime reporting
- `CMakeLists.txt` - Added WiFi, TAP, and benchmark targets

---

## [0.30.0] - 2026-03-10

### Added - Advanced Debugging and CPU Execution Caching

- GDB write/read/access watchpoints (`Z2`/`Z3`/`Z4`) and conditional breakpoint monitor commands
- Dual-core GDB thread awareness for register access, stepping, and stop replies
- 64K decoded instruction cache for hot Thumb-1 dispatch paths
- ARMv6-M lockup handling for HardFault escalation during HardFault

### Files Modified

- `src/gdb.c` + `include/gdb.h` - Watchpoints, conditional breakpoints, dual-core thread support
- `src/membus.c` - Read/write watchpoint hooks on memory accesses
- `src/cpu.c` - Decoded instruction cache and double-fault lockup handling

---

## [0.29.0] - 2026-03-10

### Added - Per-Core Interrupt Behavior and littleOS Compatibility

- Per-core NVIC and SysTick behavior needed for shared interrupt lines on RP2040 dual-core firmware
- Per-core exception nesting and timer IRQ core-gating fixes
- littleOS boot and interactive shell support

### Files Modified

- `src/nvic.c` - Per-core interrupt enable and delivery behavior
- `src/timer.c` - Timer alarm delivery filtered by the active core's NVIC state
- `src/cpu.c` - Per-core exception bookkeeping
- `README.md` - Added littleOS coverage notes

---

## [0.28.0] - 2026-03-09

### Added - Host-Threaded Execution & Dynamic Core Allocation

**pthread-per-core Execution Model:**

- Each emulated RP2040 core runs in its own host pthread for true parallelism
- Big lock (mutex) synchronizes shared peripheral state between core threads
- WFI/WFE instructions put host threads to sleep (condition variable, zero CPU usage)
- Interrupt delivery (NVIC pend, SysTick, PendSV) wakes sleeping threads via condvar signal
- SEV instruction wakes all sleeping cores
- Cooperative round-robin still available as fallback (default when no `-cores` flag)

**Runtime Core Configuration:**

- `-cores N` flag: 1 for single-core, 2 for dual-core (default: 2)
- `-cores auto`: queries core pool for optimal allocation based on host CPUs and running instances
- Threaded mode automatically enabled when `-cores` is specified

**Multi-Instance Core Pool:**

- File-based registry (`/tmp/bramble-corepool.reg`) tracks running bramble instances
- Each instance registers its PID and allocated core count
- `corepool_query_cores()` recommends cores based on host CPU count minus active allocations
- Stale entries (dead PIDs) automatically cleaned on each access
- Process-safe via `flock()` file locking

### Files Added

- `include/corepool.h` + `src/corepool.c` - Core pool manager (threading + multi-instance)

### Files Modified

- `include/emulator.h` - `MAX_CORES`, `num_active_cores`, `is_wfi` flag in `cpu_state_dual_t`
- `src/cpu.c` - WFI-aware `dual_core_step()`, `corepool_wake_cores()` on core1 launch
- `src/instructions.c` - WFI/WFE set `is_wfi` flag, SEV clears all
- `src/nvic.c` - `corepool_wake_cores()` on interrupt pend (NVIC, SysTick, PendSV)
- `src/main.c` - `-cores` flag, threaded execution loop, core pool init/register/cleanup
- `CMakeLists.txt` - Added corepool.c, pthread linkage
- `tests/test_suite.c` - 6 new tests (host CPU detection, pool register/unregister, query, WFI flag)

### Tests

- 255 tests passing (249 + 6 new core pool/threading tests)

---

## [0.27.0] - 2026-03-09

### Added - Storage Devices & Flash Write-Through

**Flash Write-Through Persistence:**

- Every `flash_range_erase` and `flash_range_program` immediately syncs to disk file
- Enables external mounting of flash filesystem at runtime (no need to wait for emulator exit)
- New module: `storage.c` / `storage.h`

**SD Card SPI Emulation:**

- Full SPI-mode protocol state machine for SDHC cards
- CSD v2.0 and CID register emulation
- Commands: CMD0/8/9/10/12/13/16/17/18/24/25/55/58, ACMD41
- Single-block and multi-block read/write
- File-backed storage image
- CLI: `-sdcard <path>`, `-sdcard-spi <0|1>`, `-sdcard-size <MB>`
- Defaults to SPI1 via `spi_attach_device()` callback
- New module: `sdcard.c` / `sdcard.h`

**eMMC SPI Emulation:**

- CMD1 initialization, EXT_CSD via CMD8
- Sector addressing for large storage
- File-backed storage image
- CLI: `-emmc <path>`, `-emmc-spi <0|1>`, `-emmc-size <MB>`
- Defaults to SPI0 via `spi_attach_device()` callback
- New module: `emmc.c` / `emmc.h`

**General:**

- Periodic flush every ~1M steps + flush on exit for both SD and eMMC
- Both storage devices attach via `spi_attach_device()` callbacks

### Files Added

- `include/storage.h` + `src/storage.c` - Flash write-through persistence
- `include/sdcard.h` + `src/sdcard.c` - SD card SPI emulation
- `include/emmc.h` + `src/emmc.c` - eMMC SPI emulation

### Files Modified

- `src/rom.c` - Flash write-through integration
- `src/main.c` - CLI flags for `-sdcard`, `-emmc`, init/cleanup/flush
- `CMakeLists.txt` - Added storage.c, sdcard.c, emmc.c

---

## [0.26.0] - 2026-03-09

### Added - Networking, Device Callbacks, Multi-Instance Wiring

**UART-to-TCP Network Bridge:**

- `-net-uart0 <port>` / `-net-uart1 <port>`: Bridge UART to TCP server socket
- `-net-uart0-connect <host:port>`: Connect UART to remote TCP host
- Non-blocking I/O with TCP_NODELAY for low-latency serial communication
- Automatic client accept/disconnect handling in server mode
- New module: `netbridge.c` / `netbridge.h`

**SPI Functional Upgrade:**

- 8-deep TX/RX FIFOs (PL022 spec-compliant)
- Device callback interface: `spi_attach_device(spi_num, xfer_fn, cs_fn, ctx)`
- Full-duplex byte exchange through attached device on SSPDR write
- TX/RX interrupt generation at half-full thresholds
- Status register reflects actual FIFO state (TFE, TNF, RNE, RFF, BSY)

**I2C Functional Upgrade:**

- 16-deep RX FIFO (DW_apb_i2c spec-compliant)
- Device callback interface: `i2c_attach_device(i2c_num, addr, write_fn, read_fn, start_fn, stop_fn, ctx)`
- Multiple devices per bus (up to 8), addressed by 7-bit I2C address
- DATA_CMD read/write/stop/restart bits processed immediately
- RX_FULL/TX_EMPTY/STOP_DET interrupt generation
- Proper CLR_* registers for individual interrupt clearing

**Multi-Instance Wire Protocol:**

- `-wire-uart0 <path>` / `-wire-uart1 <path>`: Wire UART between two Bramble instances
- `-wire-gpio <path>`: Wire GPIO pins between instances
- Unix domain socket IPC with auto server/client negotiation
- First instance creates socket, second connects; both become peers
- UART TX on one instance arrives as UART RX on the other
- GPIO pin changes propagated between instances
- New module: `wire.c` / `wire.h`

### Files Added

- `include/netbridge.h` + `src/netbridge.c` - UART-to-TCP bridge
- `include/wire.h` + `src/wire.c` - Multi-instance wire protocol

### Files Modified

- `include/spi.h` + `src/spi.c` - Rewritten with FIFOs and device callbacks
- `include/i2c.h` + `src/i2c.c` - Rewritten with RX FIFO and device callbacks
- `src/uart.c` - TX routing through net bridge, wire, or stdout
- `src/main.c` - CLI flags, init/cleanup/poll for net bridge and wire
- `CMakeLists.txt` - Added netbridge.c and wire.c

---

## [0.25.0] - 2026-03-09

### Added - CircuitPython Support

**CircuitPython 10.1.3 boots and runs code.py via USB CDC stdio.**

Command: `./bramble python/circuitpython.uf2 -stdin -clock 125`

**ROSC Peripheral (0x40060000)**:

- Ring Oscillator with CTRL, STATUS, RANDOMBIT, FREQA/B, DIV, PHASE, COUNT
- STATUS returns STABLE=1 (bit 31) + ENABLED when CTRL=0xFAB
- RANDOMBIT uses xorshift LFSR for pseudo-random bit generation
- Integrated into clocks module with atomic register aliases

**SIO QSPI GPIO Input**:

- SIO_GPIO_HI_IN (0xD0000008) returns QSPI pin state (CS high, data lines pulled up)
- GPIO offset routing in sio_read32() delegates to gpio module for GPIO-range offsets

**USB CDC Fixes**:

- CDC DTR signaling: SET_CONTROL_LINE_STATE (0x22) with DTR+RTS required for CircuitPython serial output
- CDC interface fix: Communication interface (class 0x02) preserved as cdc_iface, not overwritten by Data interface (class 0x0A)
- Skip SET_LINE_CODING OUT data phase (causes hangs with some firmware)
- CDC buf_ctrl requires both AVAILABLE and FULL bits for data reads

### Files Modified

- `include/clocks.h` - ROSC register defines and state fields
- `include/gpio.h` - SIO_GPIO_HI_IN define
- `src/clocks.c` - ROSC read/write handlers with LFSR RANDOMBIT
- `src/gpio.c` - SIO_GPIO_HI_IN case in gpio_read32
- `src/membus.c` - ROSC in is_clocks_addr(), GPIO offset routing in sio_read32()
- `src/usb.c` - CDC DTR, interface fix, skip SET_LINE_CODING, buf_ctrl fix

---

## [0.24.0] - 2026-03-09

### Added - MicroPython Support

**MicroPython v1.27.0 boots and runs REPL via USB CDC stdio.**

Command: `./bramble python/micropython.uf2 -stdin -clock 125 -flash mpy.bin`

**USB Fixes**:

- Fixed buffer control bit definitions: BUFFER0 (lower halfword) instead of BUFFER1 (upper halfword)
- Added DPRAM byte/halfword access routing (read-modify-write for TinyUSB memcpy)
- Multi-packet IN accumulation for descriptors > 64 bytes (EP0 max packet size)
- CDC endpoint defaults (EP2 IN/OUT) when config descriptor parsing truncated
- Skip optional CDC class setup (SET_LINE_CODING/SET_CONTROL_LINE_STATE)

**Peripheral Stubs**:

- SYSINFO (0x40000000): CHIP_ID returns RP2040-B2, PLATFORM=ASIC
- IO_QSPI (0x40018000): 6 QSPI GPIO pins with STATUS/CTRL + interrupt registers
- PADS_QSPI (0x40020000): QSPI pad electrical control registers
- XIP SSI atomic aliases: SET/CLR/XOR at +0x2000/+0x3000/+0x1000 from 0x18000000

**Other Fixes**:

- ROM expanded to 16KB (matching real RP2040 bootrom)
- POP PC with EXC_RETURN: SP updated before exception return processing (critical for nested exceptions)

### Files Modified

- `include/rom.h` - ROM_SIZE 0x1000→0x4000
- `include/usb.h` - Buffer control bit definitions (BUFFER0), multi-packet IN fields
- `src/membus.c` - USB DPRAM byte/halfword routing, SYSINFO/IO_QSPI/PADS_QSPI stubs, XIP SSI aliases
- `src/usb.c` - Multi-packet IN, CDC endpoint defaults, skip CDC class setup
- `src/instructions.c` - POP EXC_RETURN SP fix

---

## [0.23.0] - 2026-03-09

### Changed - Clean Output Separation + Robustness

**Stdout/Stderr Separation**:

- All emulator boot, init, loader, and status messages redirected to stderr
- Firmware UART/USB CDC output is now the only thing on stdout
- Enables clean piping: `./bramble firmware.uf2 -stdin > output.txt`
- Affected files: main.c, cpu.c, rom.c, uf2.c, elf.c, gdb.c

**Runtime Diagnostic Printf Gated Behind `-debug`**:

- Timer alarm/disarm/interrupt messages gated behind `cpu.debug_enabled`
- FIFO push/pop warnings gated behind `cpu.debug_enabled`
- IT instruction, UDF, BKPT, unimplemented instruction messages gated
- Eliminates all spurious stdout output during normal firmware execution

**BKPT and Unhandled 32-bit Instructions Now HardFault**:

- BKPT previously halted emulator (set PC=0xFFFFFFFF); now triggers HardFault exception
- Unhandled 32-bit Thumb instructions previously halted; now trigger HardFault
- Allows firmware HardFault handlers to catch these instead of crashing emulator

**Interactive Mode Instruction Limit**:

- 1B instruction safety limit now disabled when `-stdin` flag is active
- Enables long-running firmware (MicroPython REPL, interactive shells) to run indefinitely
- GDB mode already had this behavior; stdin mode now matches

### Files Modified

- `src/instructions.c` - BKPT→HardFault, gated printf for IT/UDF/BKPT/unimplemented
- `src/cpu.c` - Gated FIFO warnings, EXC_RETURN error, PC-out-of-bounds; boot messages→stderr
- `src/timer.c` - Gated all alarm/interrupt diagnostic printf
- `src/main.c` - All printf→fprintf(stderr), stdin disables instruction limit
- `src/rom.c`, `src/uf2.c`, `src/elf.c`, `src/gdb.c` - printf→fprintf(stderr)
- `tests/test_suite.c` - 237 tests (unchanged)

---

## [0.22.0] - 2026-03-09

### Added - SCB/NVIC Extensions + RTC Time Ticking

**CPUID Register**:
- `0xE000ED00` returns Cortex-M0+ identifier (0x410CC601)
- Implementer=ARM, Variant=0, Architecture=M0+, PartNo=0xC60, Revision=1

**NVIC IABR + Full IPR Range**:
- NVIC_IABR (0xE000E300) readable, tracks active interrupts
- IPR registers extended to IPR0-7 (covering all 26 RP2040 IRQs)

**RTC Time Ticking**:
- CTRL.LOAD strobe copies SETUP_0/1 into running time fields
- Clock ticks seconds via `rtc_tick()` in timing path (microsecond accumulator)
- Full calendar rollover: seconds→minutes→hours→days→months→years
- Leap year support
- RTC_1/RTC_0 return packed running time instead of raw setup values

**NVIC Debug Output Gating**:
- All NVIC printf output gated behind `cpu.debug_enabled`

### Files Modified

- `src/nvic.c` - CPUID, IABR, IPR0-7, debug printf gating
- `src/rtc.c` - Complete rewrite: LOAD strobe, ticking, calendar rollover
- `include/rtc.h` - Running time fields, tick_acc, rtc_tick() declaration
- `src/cpu.c` - rtc_tick() in timing_tick()
- `tests/test_suite.c` - 7 new tests (237 total)

---

## [0.21.0] - 2026-03-09

### Added - USB Device Enumeration + Flash Persistence

**USB Host Enumeration Simulation**:

- Full USB host state machine simulating device enumeration:
  - Bus reset → GET_DEVICE_DESCRIPTOR (8 bytes) → SET_ADDRESS
  - GET_DEVICE_DESCRIPTOR (full) → GET_CONFIGURATION_DESCRIPTOR (short + full)
  - SET_CONFIGURATION → CDC SET_LINE_CODING → SET_CONTROL_LINE_STATE (DTR+RTS)
- SIE_STATUS properly reflects VBUS_DETECTED, CONNECTED, BUS_RESET, SETUP_REC, TRANS_COMPLETE
- BUFF_STATUS tracking with W1C semantics, INTR/INTS dynamic computation
- Control transfer phase tracking (setup → data in/out → status in/out → done)
- Enumeration waits for PULLUP_EN (SIE_CTRL bit 16) before starting
- Paced by firmware EP0 buffer responses (polls buffer control for AVAILABLE/FULL)

**USB CDC Data Bridge**:

- Configuration descriptor parsing to find CDC bulk endpoints (class 0x0A, endpoint type 2)
- CDC bulk IN: firmware data printed to stdout
- CDC bulk OUT: `usb_cdc_rx_push()` injects stdin bytes into device
- `usb_step()` called from main loop advances enumeration and CDC transfer
- `stdio_usb_connected()` returns true after full enumeration

**Flash Persistence** (`-flash <path>`):

- On startup: loads flash file, restores non-firmware sectors (preserves UF2 firmware, restores filesystem)
- On exit: saves full 2MB flash image to file
- Smart sector detection: 4KB sectors compared against 0xFF to identify firmware vs filesystem
- Enables littlefs/FAT filesystem persistence across emulator runs

### Files Modified

- `include/usb.h` - Complete rewrite: enumeration enums, CDC fields, expanded state struct
- `src/usb.c` - Complete rewrite (~400 lines): enumeration engine, CDC bridge, control transfers
- `src/main.c` - `-flash` option, flash load/save, `usb_step()` in main loop, USB stdin bridging
- `docs/ROADMAP.md` - Sections 5.4 (USB) and 5.5 (Flash) marked COMPLETE

---

## [0.20.0] - 2026-03-09

### Added - GPIO Interrupts, PIO INTR, Dynamic Frequency

**GPIO Edge/Level Interrupt Detection**:

- Automatic INTR bit setting when pin values change via SIO writes or `gpio_set_input_pin()`
- Level interrupts: continuously recomputed from current pin state (not latched)
- Edge interrupts: latched on rising/falling transitions, cleared by W1C
- Triggers NVIC IRQ 13 (IO_IRQ_BANK0) through INTE/INTF/INTS chain
- `gpio_set_input_pin()` API for external input changes with event detection

**PIO INTR from FIFO Status**:

- INTR register dynamically computed from hardware state:
  - Bits [11:8]: SM IRQ flags
  - Bits [7:4]: TX FIFO not full (per SM)
  - Bits [3:0]: RX FIFO not empty (per SM)
- INTS = (INTR | INTF) & INTE for both IRQ0 and IRQ1
- IRQ check after each `pio_step()` cycle and after `pio_write32()`

**Dynamic Frequency Reporting**:

- FC0_RESULT computed from PLL_SYS registers: `(12MHz * FBDIV) / (REFDIV * POSTDIV1 * POSTDIV2)`
- `machine.freq()` returns actual configured frequency instead of hardcoded 125MHz
- Defaults to 125MHz when PLL not yet configured

### Files Modified

- `src/gpio.c` - Edge/level detection, `gpio_effective_pins()`, `gpio_detect_events()`, `gpio_set_input_pin()`
- `include/gpio.h` - `gpio_set_input_pin()` declaration
- `src/pio.c` - `pio_compute_intr()`, `pio_check_irq()`, dynamic INTR/INTS reads
- `src/clocks.c` - FC0_RESULT from PLL_SYS, FC0_REF_KHZ through FC0_SRC stubs
- `docs/ROADMAP.md` - Sections 5.1-5.3 marked COMPLETE

---

## [0.19.0] - 2026-03-08

### Added - ROM Soft-Float, HardFault, Exception Nesting, Peripheral IRQs

**ROM Soft-Float/Double**:

- BX LR stubs at 0x0500-0x0567 for 'SF'/'SD' ROM data tables
- Intercepted by PC check in `cpu_step()`, native C float/double operations
- Covers all standard operations: add, sub, mul, div, sqrt, trig, conversions

**ROM Flash Write**:

- `flash_range_erase()`: memset to 0xFF for specified sector range
- `flash_range_program()`: byte copy to flash for specified range
- Intercepted via ROM function table

**HardFault Exception**:

- Vector 3 triggered on bad PC, undefined instructions, unimplemented opcodes
- Replaces halt behavior with proper ARM exception entry

**Exception Nesting**:

- Stack of active exceptions (MAX_EXCEPTION_DEPTH=8) replaces single `current_irq`
- Nested exceptions restore previous exception on return
- EXC_RETURN 0xFFFFFFFD (thread mode PSP) accepted

**SIO Interpolators**:

- 2 per core at 0xD0000080-0xD00000FF
- ACCUM0/1, BASE0/1/2, CTRL_LANE0/1 with shift/mask/sign-extend
- PEEK/POP/FULL result computation, BASE_1AND0 combined write

**Peripheral IRQ Delivery**:

- All peripherals now signal NVIC: UART→IRQ20/21, GPIO→IRQ13, PIO→IRQ7-10, ADC→IRQ22,
  SPI→IRQ18/19, I2C→IRQ23/24, PWM→IRQ4, DMA→IRQ11/12, Timer→IRQ0-3, FIFO→IRQ15/16

**Watchdog Reboot**:

- CTRL bit 31 (TRIGGER) resets entire emulator (re-inits cores, NVIC, timer, ROM)
- AIRCR SYSRESETREQ (0x05FA0004) also triggers system reset

**RTC Stub** (0x4005C000):

- Register-level: CLKDIV_M1, SETUP_0/1, CTRL, IRQ registers
- CTRL.ACTIVE reflects CTRL.ENABLE, setup values returned as current time

**Debug Logging**:

- `-debug-mem` flag logs unmapped peripheral read/write access to stderr

### Files Added

- `src/rtc.c`, `include/rtc.h` - RTC peripheral stub

### Files Modified

- `src/rom.c` - Soft-float/double interception, flash write functions
- `src/cpu.c` - HardFault, exception nesting, EXC_RETURN PSP, ROM PC interception
- `src/membus.c` - SIO interpolators, AIRCR SYSRESETREQ
- `src/clocks.c` - Watchdog reboot trigger
- `src/gpio.c`, `src/uart.c`, `src/pio.c`, `src/adc.c`, `src/spi.c`, `src/i2c.c`, `src/pwm.c`, `src/dma.c` - NVIC IRQ signaling
- `src/main.c` - `-debug-mem` flag, RTC init
- `tests/test_suite.c` - 230 tests total

---

## [0.18.0] - 2026-03-08

### Added - ADC FIFO, USB Module, README Refactor

**ADC FIFO and Round-Robin**:

- 4-deep FIFO with circular buffer (push on conversion, pop on FIFO register read)
- FCS register: EN, SHIFT (12->8 bit), ERR, DREQ_EN, THRESH, LEVEL, EMPTY, FULL
- Overflow (sample dropped when full) and underflow (read from empty) sticky flags (W1C)
- Round-robin channel cycling via RROBIN mask in CS register
- START_ONCE triggers immediate conversion with FIFO push
- FIFO interrupt when level >= threshold

**USB Controller Module**:

- Extracted from inline membus stubs into proper `usb.c`/`usb.h` module
- 4KB DPRAM backed by real memory (endpoint descriptors writable/readable)
- Register-level handling: MAIN_CTRL, SIE_CTRL, USB_MUXING, USB_PWR, INTE, INTF
- SIE_STATUS returns 0 (disconnected); SDK falls back to UART
- Atomic register aliases (SET/CLR/XOR)

**README Refactor**:

- Replaced wall-of-text Current Status section with clean tables
- Coverage table (CPU, Memory Map, Peripherals, Boot, etc.)
- Peripheral table with addresses and emulation level
- Removed outdated Known Limitations (PIO clkdiv now done)

### Testing

- **10 new tests** (228 total, up from 218)
- ADC FIFO: push/pop, overflow, underflow, shift, W1C flags, round-robin, START_ONCE
- USB Controller: DPRAM readback, SIE_STATUS disconnected, MAIN_CTRL readback

### Files Modified

- `include/adc.h` - FCS bit defines, FIFO state in `adc_state_t`, `adc_do_conversion()` API
- `src/adc.c` - Complete rewrite: FIFO engine, round-robin, FCS status bits
- `include/usb.h` - New: USB register offsets, `usb_state_t`, API
- `src/usb.c` - New: USB module with DPRAM, register read/write, atomic aliases
- `src/membus.c` - Replaced inline USB stubs with `usb_match()`/`usb_read32()`/`usb_write32()`
- `src/main.c` - Added `usb_init()` call
- `include/emulator.h` - Removed USB base address defines (now in usb.h)
- `CMakeLists.txt` - Added usb.c to both SOURCES and LIB_SOURCES
- `tests/test_suite.c` - 10 new tests, usb.h include
- `README.md` - Refactored Current Status into tables

---

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

## [0.4.0] - 2026-03-08

### Fixed - Audit-Driven Correctness and Performance Bugs

- Preserved the xPSR Thumb bit when updating flags.
- Fixed register-offset `LDR`/`STR`, `ADDS Rd,Rn,Rm` flag handling, `POP` exception return, `ASR #0`, `REV16`, and register `ROR`.
- Removed Core 1 re-entry resets, FIFO deadlocks, inverted spinlock acquisition, and dual-core peripheral-routing gaps.
- Fixed instruction counting for halted cores, `WFI` double-PC advance, `SCB_VTOR` readback, and unsafe UF2 alignment-dependent pointer casts.
- Reduced dual-core per-step copying overhead and removed unconditional timer hot-path printf noise.

### Files Modified

- `src/instructions.c` - ALU, shift, exception return, and decode fixes
- `src/cpu.c` - Core stepping, FIFO/spinlock behavior, and performance cleanup
- `src/membus.c` - Dual-core peripheral routing and timer printf cleanup
- `src/uf2.c` - Safe unaligned field reads
- `src/nvic.c`, `src/main.c`, `src/timer.c` - Correctness and diagnostics fixes

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
