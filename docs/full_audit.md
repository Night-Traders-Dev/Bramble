# Bramble Full Audit — Correctness, Concurrency, Security, Hygiene

**Date:** 2026-09-30
**Audited commit:** `45ccf08` (findings)
**Status:** fixes landed across `8e34bf8`, `4eebffa`, `b4343d0`, `cb468e3`,
`c5276d6`, `9758e3a`, `8646097`. See "Resolution" on each item.
**Method:** empirical-first. AddressSanitizer + UndefinedBehaviorSanitizer over
the full test suite and every bundled firmware image; GCC `-fanalyzer` over all
47 translation units; targeted review of concurrency, host-facing and storage
paths; each critical claim reproduced before acting on it.
**Companion:** `datasheet_audit.md` covers spec conformance (RP2040/RP2350/
Hazard3 register and memory-map correctness). This document covers everything
else.

## Why this document exists

The previous internal audit (`audit_report.md`, June) was code-only. It
correctly found some issues but **systematically under-rated two classes**,
because it reasoned about the code rather than running it:

- it described the SD/eMMC block-address bug as a benign "reads from address 0
  instead of the intended block" aliasing quirk. It is actually a guest-
  triggerable **4 GiB out-of-bounds read *and write*** that segfaults.
- it never found `fatfs.c`, which parses a fully attacker-controlled FAT BPB
  and had three out-of-bounds primitives and **zero test coverage**.

Sanitizers found 8 undefined-behaviour sites and one stack overflow that no
amount of reading had surfaced.

---

## Fixed in `8e34bf8`

All reproduced first, then fixed, then re-verified clean under sanitizers on
two architectures.

### C1 — [CRITICAL] Guest firmware → 4 GiB host OOB write via the flash ROM stub

`src/rom.c:439-465`. The erase and program stubs guarded with
`offs + count <= FLASH_SIZE`, where `offs` and `count` are guest registers.
The addition is 32-bit and wraps:

```
offs = 0xFFFF0000  count = 0x00020000
offs + count = 0x00010000 <= FLASH_SIZE        guard PASSES
memset(&cpu.flash[0xFFFF0000], 0xFF, 0x20000)  writes 4 GiB past the array
```

Reachable by branching to PC `0x03B8` / `0x03BC` — precisely where the Pico
SDK's `flash_range_erase()` / `flash_range_program()` land. `rom_intercept()` is
consulted on *every* instruction step, so no special setup is needed. Program
sources bytes via `mem_read8(src + i)`, so the **content** is guest-controlled
too. This is a host-memory-write primitive from an untrusted firmware file.

Fixed: `offs <= FLASH_SIZE && count <= FLASH_SIZE - offs`.

### C2 — [CRITICAL] Same wrap reaches the persisted flash file

`src/storage.c:51` had the identical `offset + len > FLASH_SIZE` check, called
from both C1 sites, then `fwrite(&cpu.flash[offset], 1, len, persist_fp)` — a
~4 GiB host **read** written into the user's `-flash` file.

### C3 — [CRITICAL] SD card / eMMC: guest SPI argument wraps past all bounds checks

`src/sdcard.c`, `src/emmc.c` (8 sites). `arg * 512` for a 32-bit argument taken
straight off the SPI bus wraps, and then `addr + 512` wraps to 0 so the guard
passes:

```
arg = 0x007FFFFF -> addr = 0xFFFFFE00
addr + 512 = 0  >  size?  NO -> accepted
&sd->data[0xFFFFFE00]                  4 GiB OOB read  (CMD17/CMD18)
memcpy(&sd->data[0xFFFFFE00], ...)     4 GiB OOB write (CMD24/CMD25)
```

The existing audit called this an aliasing quirk; it is a 512-byte
guest-controlled OOB write. Fixed by doing the scaling and comparison in
64-bit, plus the same for the multi-block and write-address guards.

### C4 — [HIGH] GDB `M` packet: unbounded host stack over-read into guest RAM

`src/gdb.c:494`. The peer-supplied byte count indexed a fixed `pkt[4096]` with
no clamp, so a short packet declaring a large length read past the buffer and
wrote host stack contents into emulated RAM, where the same peer could read
them back with `m`. `handle_read_memory()` already clamped to 2048 — the write
path did not. Both now clamp to the bytes actually present.

Verified against the live binary: `M20000000,2600` (declares 9728 bytes, sends
one) previously returned `$OK` after copying ~9.7 KB of stack; now returns
`$E01` and the read-back is all zeroes.

### C5 — [HIGH] SIGPIPE kills the emulator on any host peer disconnect

No `SIGPIPE` handling anywhere except one `MSG_NOSIGNAL` in `w5500.c`. Verified
with a harness driving the **real** `gdb_send_raw()` loop: the first write after
a peer reset returns `ECONNRESET`, and the **second** raises `SIGPIPE` and
terminates the process — silently discarding all emulator state. Now ignored at
startup, after which the same harness reports `EPIPE` and survives.

### C6 — [HIGH] `get_sys_info` ROM stub: stack-buffer-overflow

`src/rom.c:511`. ASan reported `stack-buffer-overflow` and
`index 16 out of bounds for type 'uint32_t [16]'` when running
`littleos_pico2.uf2`: the table stored **17** words into `buf[16]` before
clamping. It also emitted `NONCE`, which datasheet §5.4.8.17 lists as *not
supported*, and reported `CPU_INFO` as `0x01000200` instead of `0`. Corrected
to 15 words, bounded by construction, matching the datasheet field list.

### C7 — [MEDIUM] Eight undefined-behaviour sites

All confirmed by UBSan, all fixed:

| Site | UB |
|---|---|
| `src/membus.c:1447,1502` | `0xFFFF << (offset*8)` / `0xFF << (bo*8)` overflow `int` |
| `include/dma.h:83` | `(1 << 31)` signed overflow |
| `include/nvic.h:103-104` | `(1 << 30)`, `(1 << 31)` |
| `src/instructions.c:337,612,674` | shift of a negative value; `1 << 31` |
| `src/gpio.c:81` | `(0xC << 28)` overflows `int` |
| `src/gpio.c:90` | `1u << pin` for pin ≥ 32 — **wraps onto the wrong pin** |
| `src/rp2350_rv/rv_cpu.c:471` | `C.LUI` shifts a negative `imm` |
| `include/rp2350_rv/rv_cpu.h:88` | `rv_imm_s()` sign-extends via signed `<< 20` |

The `gpio.c` one is the only one with a *correctness* consequence rather than
merely theoretical: `gpio_set_pin(40, 1)` wrote bit 8.

---

## Resolution summary

| Batch | Findings | State |
|---|---|---|
| 1 — memory safety | O11-O17, F13 | fixed |
| 2 — deadlock/concurrency | O1-O10 | fixed |
| 3 — interrupts | datasheet B1, C1, C2, C12, A2 | fixed |
| 4 — UART/SPI | O25, C5, C6, C7, O27, O29, O30 | fixed |
| 5 — timer/pio/clocks | B2, B3, C3, C11 | fixed |
| 6 — Hazard3 | B4, B5, B6, B7, B8 | fixed |
| 7 — hygiene/tests | O19, O22, O23 | fixed |

Still open, with reasons, are listed under [Still open](#still-open) at the end.

## Findings (kept as originally written, for provenance)

### Open findings

### Concurrency and thread safety

**O1 — [CRITICAL] `WFE` is modelled as `WFI`, and `SEV` does not latch — dual-core firmware deadlocks.**
`src/instructions.c:1094-1117`. `instr_wfe()` sets `is_wfi` exactly like WFI, and
both schedulers gate resumption on an **interrupt-only** predicate
(`corepool.c:319`, `cpu.c:1692`): pending **and enabled** IRQ, SysTick, or
PendSV. The event register, SIO FIFO-valid bits and spinlock state are not
consulted. `instr_sev()` clears `is_wfi` on every core rather than setting an
event latch, so an event arriving *before* the `WFE` is lost.

Verified with a hand-assembled firmware containing the pico-sdk `spin_lock()`
inner loop `SEV; WFE;` with no interrupts enabled: **2 instructions then frozen
for the full timeout**, versus the same program with the SEV replaced by a NOP.
Real impact: `multicore_fifo_pop_blocking()` is literally
`while (!fifo_rvalid()) __wfe();`, and `lock_core.h`'s unlock-with-wait uses the
same idiom. A correct fix is one predicate change: add a per-core event flag set
by SEV, spinlock release and FIFO push, and include it in the wake test.

**O2 — [HIGH] `spin_unlock` never wakes a waiter.** `src/cpu.c:1984` clears the
lock and calls nothing, so a peer parked in `spin_lock()`'s `__wfe()` never
resumes. Second independent deadlock path into O1.

**O3 — [HIGH] PIO and USB only advance from the core-0 thread.**
`src/corepool.c:367-383`. `pio_step()` / `usb_step()` are guarded by
`core_id == CORE0` inside the per-core quantum. If core 0 halts or executes WFI,
neither is ever called again no matter how much core 1 runs. TIMER/RTC were
explicitly handled for exactly this case (`corepool.c:350-356`); PIO/USB were
not. The cooperative scheduler calls both unconditionally.

**O4 — [HIGH] `pthread_create` failure leaves the emulator spinning forever.**
`src/corepool.c:404`. Failure prints one line, sets `thread_active = 0`, and
falls through — but `corepool.running` was already set to 1 and
`any_core_running()` only inspects `is_halted`, so the main loop spins with zero
guest progress and no diagnostic. Reachable under a low `pids.max`.

**O5 — [HIGH] Core-1 thread can outlive `pthread_mutex_destroy`.**
`src/corepool.c:421` publishes `thread_active[core_id] = 1` *after*
`pthread_create`, so `corepool_stop_threads()` can miss a thread created
concurrently during teardown; `corepool_cleanup()` then destroys the mutex that
thread is blocked on.

**O6 — [MEDIUM] `sdcard_flush()`/`emmc_flush()` run outside `emu_lock`.**
`src/main.c:1329`. A multi-hundred-MB `fwrite` of the backing image races with
guest SPI writes that hold only `emu_lock` — disjoint locks over the same buffer,
so the host image file can be torn.

**O7 — [MEDIUM] Emulated TIMER advances at up to 2× when both cores sleep.**
`src/corepool.c:339`. Both sleeping cores tick the timer for overlapping host
time; the cooperative scheduler restricts this to core 0. Timing-dependent
dual-core firmware drifts.

**O8 — [MEDIUM] Flash writes skip icache/JIT invalidation.** `src/rom.c:445`.
Every other flash-writing path invalidates both (`membus.c:1054`); the ROM stub
does not, so a guest that erases and re-programs the region it is executing from
runs stale decoded instructions. The Hazard3 audit found the same defect on the
RISC-V path.

**O9 — [MEDIUM] `-cores N` and `-cores auto` are silently discarded.**
`src/main.c:489` sets `num_active_cores`, then `dual_core_init()` at
`src/cpu.c:1459` immediately resets it to 1. The multi-core host-coordination
feature is inert.

**O10 — [MEDIUM] GPIO per-processor interrupt registers alias each other.**
`src/gpio.c:167-311`. The decoder maps the whole `0xF0..0x17F` window onto
`proc0_*` only, so `PROC1_INTE` reads/writes core 0's mask and `PROC0_INTS`
falls through to 0.

*Verified sound, so not re-audited:* every `core_thread_fn` exit path releases
`emu_lock`; no lock-order inversion exists; guest spinlock mutual exclusion holds
(test-and-set under the big lock, and the JIT re-dispatches through
`dispatch_table` so the side-effecting read is not bypassed); condvar clock IDs
are consistent; host socket I/O is non-blocking throughout; teardown ordering is
otherwise correct.

### Storage and filesystem

**O11 — [CRITICAL] `fatfs.c` validates no BPB-derived offset.** `src/fatfs.c:230`.
`fat16_mount()` checks `bytes_per_sector == 512` and a few non-zero divisors,
but never that the geometry it *derives* fits inside `media_size`.
`root_dir_offset` and `data_offset` are then indexed unguarded by
`find_dirent`, `find_free_dirent` and `fat16_list_root`. The BPB comes from the
untrusted firmware image, so a crafted `reserved_sectors` places the root
directory megabytes past `cpu.flash[]`, giving both OOB read and OOB write (the
dirent write in `fat16_write_file`). Compounding it, `root_entry_count` is
capped at 65535, so one `ls` walks 2 MiB at a guest-chosen offset. **`fatfs.c`
has no test coverage at all and is not mentioned in any prior audit.**

**O12 — [HIGH] `fat16_write_file` reports success after silently dropping data.**
`src/fatfs.c:345-377`. A cluster chain that runs off the end of the media is
skipped with no error path; the dirent then advertises the full `file_size` and
the function returns 0. Reproduced: an 8192-byte write reported success,
`stat` showed 8192, and 6144 bytes had vanished.

**O13 — [HIGH] FAT16 volumes above 32 MiB cannot be mounted.**
`src/fatfs.c:216` truncates the 32-bit `total_sectors` BPB field to 16 bits, so
a legitimate 64 MiB volume is rejected as "FAT32 not supported".

**O14 — [HIGH] FUSE `write` truncates a 64-bit offset to 32 bits.**
`src/fuse_mount.c:184`. `new_size` is a `uint32_t` while the subsequent
`memcpy` uses the full `off_t`, so a large offset produces a wild heap write.
Host-triggered (FUSE client is local), not guest-reachable.

**O15 — [HIGH] `ELF` loader bounds use the wrong constants.** `src/elf.c:219`.
`region_contains()` checks against `FLASH_SIZE` (2 MiB) and `RAM_SIZE` (264 KiB)
while `cpu.flash[]` is 16 MiB and RP2350 has 520 KiB SRAM. Any RP2350 ELF with a
segment above 2 MiB flash fails to load at all. Combined with the UF2 loader
accepting up to 16 MiB but the bus only mapping 2 MiB, the two loaders disagree
for the same firmware.

**O16 — [MEDIUM] Flash persistence is neither atomic nor crash-safe.**
`src/storage.c`. No temp-file-then-rename, no `fsync`; `fopen(..., "r+b")` falls
back to **truncating** `"w+b"` on any failure other than ENOENT, destroying an
existing image. `flash_persist_save_all` rewrites the whole 2 MiB in place.

**O17 — [MEDIUM] `-mount-offset` is never range-checked.** `src/main.c:516,925`.
`fs_size = FLASH_SIZE - fs_offset` underflows for `fs_offset > FLASH_SIZE`,
handing `fat16_mount` a pointer megabytes past `cpu.flash[]`.

### Build, portability, hygiene

**O18 — [HIGH] The build is not `-Werror` clean.** Confirmed with the project's
own CMake: 26 warnings, including the duplicate macro redefinition, three
`%X`-vs-`uint64_t` format mismatches (`cpu.c:1153`), four
tautological-compare warnings, and one unused function. `build.sh` pipes make
through `grep || true`, so these never fail the build.

**O19 — [MEDIUM] Eight duplicate RP2350 base macros.** `include/clocks.h:52-76`
re-declares eight macros that already exist in
`include/rp2350_rv/rp2350_memmap.h`. Only two warn — GCC suppresses
redefinition when the replacement list is *token-identical*, and exactly
`RP2350_WATCHDOG_BASE` and `RP2350_ROSC_BASE` differ in hex-letter case
(`0x400d8000` vs `0x400D8000`). The other six are silent, which is worse: a
future correction in one header silently loses to the other depending on include
order.

*(Correction: `datasheet_audit.md` attributed the warnings to include order.
That is wrong — it is token-case identity. The underlying duplication is the
same defect.)*

**O20 — [MEDIUM] Four unreachable instruction decoders shipped as working.**
`src/thumb32.c:366-435`. `VLDR`/`VSTR` double-precision and `VCVT` F32↔U32
arms have masks that disagree with their patterns
(`0xED900000 & 0xFF700000 = 0xED100000`). Exhaustive scan confirms **zero**
reachable encodings. Firmware using them silently falls through to
`instr_unimplemented`.

**O21 — [MEDIUM] Test coverage is 19.9 %.** 187 of 938 functions referenced by
`tests/test_suite.c`. Eleven files have zero references, including `fatfs.c`
(21 functions, parses attacker-controlled BPB), `gdb.c` (33), `cyw43.c` (39),
`tapif.c` (32) and `fuse_mount.c` (14). This is why the datasheet audit's real
bugs in the RV32 interrupt path, GPIO interrupt decode and SPI chip-select had
no failing test.

**O22 — [MEDIUM] Four tests assert nothing.** `tests/test_suite.c:373,797,3799`.
`test_peripheral_writes_no_crash` — which is exactly the test that would have
caught the SIO-GPIO-write bug fixed in `a06d497` — writes three peripheral
registers and never reads them back.

**O23 — [LOW] A test file is wired into nothing.** `tests/test_invariant_cyw43.c`
is not referenced by any build target, needs `libcheck` (not in CMakeLists), and
its single test asserts on a libc `strncpy` rather than any `cyw43_*` function —
it would pass if `cyw43.c` were empty.

**O24 — [LOW] Builds are not reproducible.** `-g` is hard-coded into
`CMAKE_C_FLAGS` so `--release` still ships debug info; no
`SOURCE_DATE_EPOCH`/`-ffile-prefix-map`; and the post-build step copies the
binary into the source tree, so building mutates the working tree.

### Correctness (non-datasheet)

**O25 — [HIGH] SPI chip-select callback is never invoked.**
`src/spi.c:240` stores `device.cs`; nothing calls it. `sdcard_spi_xfer`,
`emmc_spi_xfer` and `w5500_spi_xfer` all begin `if (!cs_active) return 0xFF`, so
**SPI-attached SD cards, eMMC and W5500 never respond to any command.**

**O26 — [HIGH] UART FIFOs are half depth.** Datasheet: 32 locations. Code: 16,
with IFLS trigger levels computed for 16 — so the SDK default `IFLS=0x2` (½ full)
fires at 8 bytes. `LCR_H.FEN` is ignored, so character-mode RX never interrupts
at one byte. Overrun never sets `RSR.OE`.

**O27 — [MEDIUM] The UART TX interrupt never re-asserts** after
`ICR = TXIC`, so the canonical IRQ-driven TX loop delivers one interrupt and
then deadlocks; and the interrupt line is edge- rather than level-triggered.

**O28 — [MEDIUM] Atomic SET/CLR/XOR aliases do the interposer RMW in software**
against the model instead of in the bus, so a SET-alias write to `UARTDR` or
`SSPDR` *consumes an RX byte*.

**O29 — [MEDIUM] `-net-uart0/1` binds `INADDR_ANY`,** unlike the GDB server
which binds loopback. Anyone routable to the host can read guest UART output and
inject bytes into its RX FIFO.

**O30 — [LOW] `-wire-eth` cannot carry a real Ethernet frame.**
`WIRE_IO_BUFFER_SIZE` is 256 but frames are up to 1522, so the largest
deliverable frame is 250 bytes; an oversized frame wedges the link until the peer
is dropped as a receive-buffer overflow.

---

## Method notes

- Sanitizer builds were made with `-fsanitize=address,undefined
  -fsanitize-recover=undefined`. The first attempt used
  `-fno-sanitize-recover`, which **halted on the first UB report** and hid the
  other seven; recovering mode is what makes the full picture visible. An early
  "all clean" reading was also caused by rsync preserving mtimes, so make skipped
  recompilation — `rm -rf CMakeCache.txt CMakeFiles` was needed before the
  numbers were trustworthy.
- Both fixes for C1/C2/C4/C5 were confirmed to *fail* against the pre-fix source
  before being accepted (the flash-ROM tests segfault with exit 139).
- `GCC -fanalyzer` produced no findings beyond the duplicate macros; the real
  value came from sanitizers. GCC's static analyzer does not model the
  integer-overflow patterns that dominate this codebase.
- The SIGPIPE finding was verified by driving the *real* `gdb_send_raw()` and
  `net_bridge_uart_tx()` code from a harness rather than a toy program: the
  first post-reset write returns `ECONNRESET` and only the second raises the
  signal, which is why an end-to-end emulator run did not reproduce it.
- Some sub-agent claims were **rejected on checking** — notably "JAL/JALR link
  register ignores instruction width" (JAL is a 32-bit-only encoding, so
  `pc + 4` is correct) and "the duplicate-macro warning depends on include
  order" (it is token-identity). Findings I could not reproduce are marked
  "needs verification" rather than reported as fact.

## Suggested order

1. **O1–O3** — guest-triggerable host OOB writes, then the WFE/spinlock deadlock
   family. One predicate change closes O1 and O2 together.
2. **O11** — `fatfs.c` BPB validation; the file is entirely unaudited and
   unaudited *and* untested.
3. **C4** — the GDB fix landed in `8e34bf8`; O6 and O14 are the remaining
   memory-safety items.
4. **O21/O22/O23** — coverage and assertion quality, so the next round of
   findings is actually caught.
5. **O18/O19/O20** — warning-clean build and the unreachable decoders.
---

## Still open

These were **not** fixed, and why.

### O2 — **fixed in `19956f2`**
Reading `SSPDR` with an empty RX FIFO pushed a dummy 0xFF through the device
model to keep SDK poll loops from spinning. Removed: a real PL022 returns 0 and
has no bus effect, and the dummy meant every plain register read -- including
the read-modify-write each SET/CLR/XOR alias performs -- sent a byte to whatever
was attached. `name_prompt.uf2` produces identical output and runs ~3% fewer
instructions, since it was doing phantom transfers in its poll loop.

### TICKS register decode — **fixed in 0.48.3**
`src/rp2350_rv/rp2350_periph.c`. Two independent defects:

1. The decoder passed `base - RP2350_TICKS_BASE` where `base` was 16KB-aligned,
   discarding the register offset entirely. Every register in the block --
   `TIMER1_CTRL`, `WATCHDOG_COUNT`, `RISCV_CTRL` -- resolved to generator 0.
2. The generator stride was 8 bytes; RP2350 datasheet Table 649 gives three
   registers (CTRL, CYCLES, COUNT) at a **12-byte** stride. Even with the offset
   fixed, `TIMER1_CTRL` at `0x024` resolves as generator 4 register 4 -- the
   watchdog's CYCLES.

Added the missing per-generator COUNT latch plus `rp2350_ticks_tick()`. The test
writes a distinct value to all six CTRL registers from Table 649 and reads them
back; it fails under either defect alone.

### Bus-fault exceptions — **not attempted, and reassessed**
Raising mcause 1/5/7 requires the bus to say *whether* an address is mapped, and
`mem_read32()` has no such answer: it falls through to the shared RP2040 bus,
which answers 0 for everything. Adding it means writing a `membus_is_mapped()`
that mirrors the entire peripheral decode — `mem_read32()` alone has 47 return
points — and any address misclassified as unmapped would start trapping and
breaking firmware, with no way to tell from the test suite that a region had
simply been forgotten.

A narrower variant was considered and rejected: fault only on *instruction
fetch* outside flash/ROM/SRAM. That is safe, but worth little, because an
unmapped fetch already reads as `0x0000`, which decodes as an illegal
instruction and traps — with the wrong cause, but it traps. Firmware that
distinguishes access fault from illegal instruction is rare, and that
correctness is not worth the decode-mirroring risk.

This needs either real hardware to check region classifications against, or a
refactor that makes the peripheral decode return "handled" explicitly instead of
inferring it from fallthrough.

### C10 — **fixed in `8b9502c`**
Peripheral IRQs now reach `mip.MEIP` via an NVIC → Xh3irq sink installed by
`main.c` for `ARCH_RV32`.

### O28 — atomic aliases do the interposer RMW in software
The datasheet puts the SET/CLR/XOR interposer in the *bus*; the emulator does
read-modify-write against the model, so a SET-alias write to `UARTDR` or
`SSPDR` consumes an RX byte. The correct fix is a bus-level alias transform
applied before the peripheral decode, which is a structural change to
`membus.c`'s dispatch. Not attempted — doing it partially risks breaking the
alias decode that firmware's `hw_set_bits()` depends on.

### D3-D5 — **fixed in `41e59ee`**
FC0 offsets, the 16.16 `CLK_DIV` reset value, the generator count, and the ROSC
map and `STATUS` decode are all per-chip now. ROSC needed mapping by register
identity rather than a shift, because RP2350 moves `COUNT` *down* to `0x0C` while
`RANDOMBIT` moves *up* to `0x20`.

### D6 — RP2350 PWM — **fixed in `fc9042f`**
`41e59ee` had already stopped RP2040's PWM shadowing RP2350's PLL_SYS and routed
`0x400A8000` to the PWM model. What remained was that the *body* was still
RP2040's, so on RP2350 every access to the global register block landed on a
slice register or on nothing.

Offsets from pico-sdk `src/rp2350/hardware_regs/include/hardware/regs/pwm.h`:

| | RP2040 | RP2350 |
|---|---|---|
| slices | 8 | 12 |
| slice region | 0x00-0x9F | 0x00-0xEF |
| EN | 0xA0 | 0xF0 |
| INTR | 0xA4 | 0xF4 |
| IRQ0_INTE / INTF / INTS | 0xA8 / 0xAC / 0xB0 | 0xF8 / 0xFC / 0x100 |
| IRQ1_INTE / INTF / INTS | -- | 0x104 / 0x108 / 0x10C |
| global width | 8 bits | 12 bits |

So slice 11 (CSR at 0xDC) did not exist; `PWM_EN` was masked to `0xFF` so slices
8-11 could never be enabled; the interrupt block sat 0x50 too low, so firmware
masking an interrupt was writing PWM compare values; and the second interrupt
output did not exist at all. All four are fixed, with the per-chip slice count,
offsets and widths selected at decode time. The per-slice registers are
identical on both chips and are unchanged.

Two tests cover it, one pinning RP2350 and one pinning RP2040 so the per-chip
selection cannot regress the original target.

### F1/F4 — remaining per-chip register differences
Closed. The PIO interrupt registers, the WATCHDOG map, and the DMA interrupt lines
were each per-chip variants of an existing model; all are now done — see below.

### VFP register file — **fixed in 0.48.7**
`src/thumb32.c`. Found by AddressSanitizer on riscv64.

- `VLDR`/`VSTR` `.64` indexed a 16-entry `vfp_d` by a decoded register number
  reaching D31, overwriting the adjacent `exclusive_monitor` global.
- The single and double views were separate arrays, so a 64-bit write was
  invisible to a 32-bit access of the overlapping register.
- D16-D31 were not aliased to D0-D15. The register file is 32 words; masking the
  double index to 4 bits is both the aliasing rule and what keeps it in bounds.

Register-count assertions now turn a wrong-sized file into a build failure.

### RP2350 TMDS encoder — **fixed for Arm reachability in 0.49.1**
`src/tmds.c`, `include/tmds.h`. The SIO range `0x1c0`-`0x1e4` was unmapped
(the fake hart-launch mailbox that used to occupy it was removed without a
replacement), so DVI firmware read the unhandled marker instead of symbols.

Implemented the register map, `CTRL` semantics, colour extraction with rotation
and `NBITS` masking, `PEEK`-versus-`POP` shifting, per-layer DC balance, both
packing formats, and the 8b/10b encoder. Four bugs were found while writing the
tests: the `NBITS` mask shifted by the wrong amount, the control symbol table had
`C1` as `0x2AB` instead of `0x0AB` (an invalid symbol) with lane 1's symbols
transposed, decoded symbols were being discarded by the SIO read fallthrough,
and the `DOUBLE_L*` lane index stepped by 4 instead of 8 (`PEEK` and `POP`
alternate every 4 bytes) so it indexed past the end of the symbol arrays. The
last of those was found only by AddressSanitizer on riscv64, which is a good
argument for running the sanitizer build routinely rather than on the ARM host.

**Open risk:** the datasheet does not specify the symbol patterns, so they come
from the DVI specification, and the data-symbol path has never been checked
against a reference decoder or a real DVI link. The control symbols, register
semantics, packing and rotation are tested; the encoded data symbols are not.

### TMDS reachability — **fixed in 0.49.1**
Adding TMDS in 0.49.0 put the decoder in the RV SIO path only, and the Arm SIO
decode window was hard-coded to `0x100` bytes in three places while RP2350's
block is `0x200`. So the encoder was reachable only from Hazard3, and the two
cores would not have shared register state even if both could decode it. The
decoder moved to the shared `membus.c` path, the state into
`rp2350_periph_state_t`, and the window is now `sio_span()`.

### RP2350 DMA interrupt lines — **fixed in 0.48.9**
`src/dma.c`, `include/dma.h`, `src/nvic.c`. RP2350 has four DMA interrupt lines
(`INTE0`-`INTE3` at `0x404`/`0x414`/`0x424`/`0x434`); RP2040 has two. Lines 2
and 3 were not decoded and were never signalled, so a channel could not raise
them.

The extra lines take internal IRQ numbers 26 and 27, past the end of the RP2040
range, because the RP2040 numbering has no free slot there (`IO_IRQ_BANK0` and
`IO_IRQ_QSPI` occupy 13 and 14). They map to the datasheet's `DMA_IRQ_2`/`3`
(12 and 13) and are rejected on RP2040 by `nvic_irq_valid()`.

### VMOV GPR<->VFP — **fixed in 0.48.8**
`src/thumb32.c`. Recorded as broken in 0.48.7; closed against ARM ARM A7.7.243
(encoding T1), which the fix was written against rather than inferred from the
existing code.

Four defects:

- Not dispatched at all. `VMOV`'s first halfword is `1110 1110 000 op Vn`, giving
  `top5 == 0x1D`, but the only `0xEE00` route to the VFP decoder was under
  `top5 == 0x1E`, which those encodings never satisfy.
- The opcode mask masked bit 20, pinning `op` -- the direction -- to 0, so
  `VMOV <Rt>, <Sn>` was unreachable. Now `0xFFE00F70` against `0xEE000A10`.
- The direction test examined bits 23:16 against a `0x17` threshold; those bits
  are `0000` plus `op`, so it was never true for a matching instruction.
- The register number's low bit came from bit 6, pinned to 0 by the encoding.
  Per `n = UInt(Vn:N)` it is bit 7, so `(Vn << 1) | N` and all 32 registers.

### RP2350 PIO interrupt registers — **fixed in 0.48.6**
`include/pio.h`. The interrupt block was placed as if RP2350 had no RX FIFO
PUTGET window. Real layout: PUTGET at `0x128`–`0x164`, `GPIOBASE` `0x168`,
`INTR` `0x16c`, then `IRQ0_INTE/INTF/INTS` at `0x170`/`0x174`/`0x178` and
`IRQ1_*` at `0x17c`/`0x180`/`0x184`. The old defines started at `0x128`, so every
driver write to an IRQ enable register landed in the PUTGET window and PIO IRQ0
and IRQ1 were unreachable on RP2350. Separately, both lines masked with `0xFFF`
though the chips have 4 and 8 state machines.

### RP2350 WATCHDOG map — **fixed in 0.48.4**
`src/clocks.c`. Three defects in one block:

- `REASON` returned a hardcoded 0. The datasheet specifies it logs the reason for
  the last reset, with both bits zero for a hardware reset. Stubbed to a
  constant, a watchdog reboot was indistinguishable from a power-on reset.
- `clocks_reset()` memset the whole state block, wiping SCRATCH0-7 even though the
  register list states they persist through a soft reset.
- `0x2c` was decoded as `TICK` on both chips. RP2350 has no TICK — its register
  list ends at SCRATCH7 (`0x028`) — so writes to a nonexistent register were
  silently accepted. Now selected per chip.

Two tests: one pins the RP2350 map, one pins RP2040 so `TICK` still works.

Follow-up in 0.48.5, from re-checking the parts 0.48.4 did not close:

- **The watchdog never fired.** `LOAD` was stored and never counted, so firmware
  that armed the watchdog to recover from a hang would spin forever. WATCHDOG has
  no enable bit -- writing `LOAD` arms it -- so the value sat unused. The
  countdown now advances on the microsecond tick and fires once at expiry.
- **`SYSRESETREQ` did not set `REASON`**, though the datasheet gives bit 1 for a
  software reset. Only the watchdog path recorded anything.
- **`clocks_init()` leaked a stale `REASON`**, because it delegated to
  `clocks_reset()`, which preserves the reason across a soft reset. Cold boot now
  clears it; a soft reset preserves it.

### O20 — VFP decoders — **fixed in this release; build is warning-free**
`src/thumb32.c`. The audit recorded "masks that disagree with their patterns".
Chasing that down found two separate defects, one of them a real mis-decode.

**1. VCVT float<->int was never decoded.** The two arms tested
`(insn >> 8) & 0xFF == 0x7A` (or `0x7E`) *and* `(insn & 0xFF00FF00) == 0xEE000000`.
Those can never both hold: `0xEE000000` has zero in bits 15:8. That was the
source of the two "and of mutually exclusive equal-tests" warnings, and it meant
the instructions were unreachable.

Root cause was one line of dispatch:
```c
if ((upper & 0xEF00) == 0xEE00 || (upper & 0xEF00) == 0xED00)
```
VCVT float<->int has first halfword `0xEB8x`-`0xEBDx`, which the guard excluded.
Added `|| == 0xEB00` and implemented all four forms per QEMU's `vfp.decode`:
`VCVT.F32.U32/.S32` and `VCVT.U32.F32/.S32`, with the correct interleaved
register fields (`Vd = {bit22, bits 12:9}`, `Vm = {bits 5:1, bit 0}`) and the
correct `s` bit -- bit 11, with bit 8 a fixed 0.

**2. The VFP load/store arms were hijacking ARM-core LDRD/STRD.** They matched
`0xED9x` / `0xED8x` / `0xED5x` / `0xED4x`. Those are *core* opcodes -- `0xED40` is
STRD (U=0), `0xED50` is LDRD (U=1), `0xED80`/`0xED90` the register forms. Two
problems: the mask bits could not all match (the
`-Wtautological-compare` warnings), and `0xED4x`/`0xED5x` *are* self-consistent,
so genuine core LDRD/STRD were intercepted and executed as float accesses --
moving bytes of the VFP register file instead of the pair of core registers the
instruction named. Removing them stops that.

**VFP load/store remain unimplemented.** An earlier revision of this note claimed
they were 16-bit T2 forms in `0xD8xx`/`0xD9xx` reachable from the 16-bit dispatch
table. **That was wrong**, and following it would have introduced a bug. The ARM
ARM's Table F3-2 maps the 16-bit `1101xx` group to *conditional branch and
Supervisor Call*, so `dispatch_table[0xD8]`/`[0xD9]` correctly handle `B<cond>`;
wiring VFP there would have broken branches.

The `1101 u:1 .0 l:1 rn:4 .... 1010 imm:8` pattern in QEMU's `vfp.decode` is
likewise the **A32** form, not Thumb -- that file is titled "AArch32 VFP
instruction descriptions (conditional insns)" and its header notes it covers
"anything matching A32". V8's `Assembler::IsVldrDRegisterImmediate` confirms it,
masking bits 27:24 (`15 * B24`) against `13 * B24` -- the A32 condition field.

The Thumb-2 M-profile VFP load/store encodings differ again, and I could not
confirm their exact layout, so no decoder was written. Firmware using them now
gets `instr_unimplemented` rather than silently executing a core LDRD as a float
load, which is the correct failure mode.

Also removed as a consequence: the now-unused `vfp_d[]` double-precision file,
`vfp_st_from_insn()`/`vfp_sm_from_insn()`, and a triplicated VDIV block.

**Hart-1 launch — fixed, and the audit was half right.**
The claim "hart-1 launch uses invented SIO registers, so
`multicore_launch_core1()` never launches" was correct *for the RISC-V path*
and wrong for Arm. Arm already implements the datasheet protocol in
`sio_core1_bootrom_handle_fifo_write()`; only the RV path used the invented
mailbox.

RP2350 datasheet §5.3 gives the mechanism: core 0 pushes six words to core 1
over the SIO inter-processor FIFO,

    { 0, 0, 1, vector_table, sp, entry }

with `vector_table` destined for VTOR. §3.1.5 puts the FIFO at SIO `+0x54`
(`FIFO_WR`) and `+0x58` (`FIFO_RD`). And §3.1.9's register list is explicit that
`0x1C0`-`0x1CC` is `TMDS_CTRL`, `TMDS_WDATA`, `TMDS_PEEK_SINGLE` and
`TMDS_POP_SINGLE` — so the "boot mailbox" was writing a hart-1 entry point into
the HDMI pixel encoder, and the TMDS block itself was unreachable.

The RV path now watches core 0's `FIFO_WR`, recognises the six-word shape, and
takes the entry point, stack pointer and vector table from it. The third word
goes to `mtvec`; the old code loaded it into `a0`, which only made sense as a
"boot arg" for the invented register.

One subtlety worth recording: `FIFO_WR`/`FIFO_RD` reads must keep falling
through to the shared SIO model. Answering them early from the RV path makes
`FIFO_RD` return a marker value, and firmware polling its mailbox spins -- which
is exactly what happened on the first attempt and showed up as the RV littleOS
image climbing from ~44M to ~41M instructions in the wrong direction.

**Core `LDRD`/`STRD` are implemented after all.** Two claims here were wrong and
are withdrawn. I had written that they had "no executor", and separately that
they "silently report handled without loading". Both came from greps that
matched the guard helpers (`t32_is_tt`, `t32_is_halfword_acqrel`, ...) but
missed `t32_ldrd_strd()`, which is the actual executor. The second claim came
from a probe whose `lower` halfword I had written as `0x1000`, which puts Rt and
Rt2 in the wrong fields -- so the emulator loaded the right words into the wrong
registers and I read that as a bug.

Against ARMv7-M ARM A6.7.49/A6.7.124, `LDRD`/`STRD` (immediate) T1 is
`1110 100 P U 1 W L Rn | Rt Rt2 imm8` with `index=(P==1)`, `add=(U==1)`,
`wback=(W==1)` and `imm32 = imm8:'00'`. `t32_ldrd_strd()` decodes exactly that,
including the `imm8 << 2` scaling and the index/add/writeback rules, so no
change was needed -- `test_thumb2_ldrd_strd_immediate` now pins it against the
ARM's field layout.

Result: `src/thumb32.c` builds clean, and the whole project compiles with zero
warnings under `-Wall -Wextra -pedantic`.

### O21 — test coverage — **partly fixed in `c0f3885`**
Originally measured at 19.9% (187 of 938 functions referenced), with eleven
source files having no test references at all. `spi_flash.c` and `fatfs.c` now
have coverage, chosen because both are reached with guest-controlled values, so
their validation paths are the security-relevant surface rather than incidental.

`spi_flash`: reads take a 64-bit offset and a length from the emulated device,
so the bounds check must not overflow. Pinned that `offset + len` is evaluated
in 64 bits (a near-`UINT64_MAX` offset is rejected, not wrapped), that a read
straddling the chip end is refused, and that the chip index, `NULL` buffer and
zero length are handled.

`fatfs`: the BPB is entirely guest-supplied. A minimal valid image is built and
one field corrupted at a time -- boot signature, short media,
`bytes_per_sector != 512`, and the three zero-value fields that would divide by
zero -- plus `reserved_sectors = 0xFFFF`, which before the 64-bit arithmetic fix
placed the root directory megabytes past the media. Also checks the 32-bit
`total_sectors` fallback, that `list_root` respects the caller's capacity, and
that a short read buffer is reported rather than silently truncated.

Also earlier in this work: three tests that asserted nothing were made to assert
real behaviour, one of which (`test_peripheral_writes_no_crash`) had passed
straight through the SIO GPIO write-drop bug.

`bme280` (`2c7a317`): sits behind the emulated I2C bus, and its
register-pointer protocol is what SDK drivers depend on. Pinned the chip id,
that reads auto-increment, that the soft-reset command is recognised, and that
the 20-bit raw values survive the msb/lsb/xlsb split. The protocol needs an
explicit `bme280_i2c_start()` between transactions; driving `bme280_i2c_write()`
directly writes *data* into the last-addressed register, a silent
wrong-register write, so the test now exercises it the way the I2C model does.

`netbridge` (`2c7a317`): `uart_num` comes from the emulated UART model and
indexes fixed arrays, so the bounds check guards an OOB write into
`net_bridge_tx_pending[]`. Pinned that out-of-range and negative indices are
dropped.

`cyw43` (`381c609`): the largest uncovered file, and it bit-bangs SPI over
`WL_CS`/`WL_CLK`/`WL_DIO`, so the state machine is drivable with no hardware.
Asserts that CS low selects and high deselects, that each rising clock edge
shifts exactly one command bit, that the data phase is not entered before the
32nd bit and is entered on it, and that the read phase does not advance
`resp_offset` when nothing is queued -- the guard against walking off
`resp_buf[]`.

One trap worth recording: the model is gated on `cyw43.enabled`, which `main.c`
sets only for `-wifi` or `-tap`. The test harness passes neither, so without
setting it the first version of this test passed while exercising nothing --
every intercept returned 0.

`gdb` (`8c06bce`): built the harness first, and it immediately turned up a
conformance bug. `gdb_recv_packet()` never validated the two hex checksum digits
after `#` -- it ACKed unconditionally, so a corrupted packet was accepted as
intact and the debugger and emulator then silently disagreed. The sum is now
compared, a mismatch is NAKed with `-`, and the buffer advances past the bad
packet so the next read starts on a boundary instead of rescanning it.

The tests drive the real `gdb_handle()` over a `socketpair` -- `gdb` is extern,
so `gdb.client_fd` can be pointed at one end, with no port binding, fork or
timing dependency. Worth recording the harness subtlety: `sv[1]` is
half-closed for writing so the emulator sees EOF, but its *read* side stays open,
so a blocking drain waits forever. The drain sets `O_NONBLOCK`.

The checksum test was confirmed load-bearing by bypassing the comparison: the
suite drops to 359/360, so it exercises the validation rather than passing either
way.

`fuse_mount`: `fuse_mount_start()` is the point where a guest-supplied image
becomes a host-visible filesystem, so its validation is the security-relevant
part. Covered against a FUSE-enabled build, which matters -- with
`-DENABLE_FUSE=OFF` the stub returns non-zero immediately and the test would
pass without exercising anything, the same trap as `cyw43.enabled` and
`bme280_i2c_start()`.

### O21 — closed: every source file has a test reference
All eleven previously-uncovered files now have coverage. `tapif` is the last,
tested against a real TAP interface (`ip tuntap add mode tap dev brtest0`),
gated on `BRAMBLE_TEST_TAP` so the suite still passes without root.

`fuse_mount`'s entry validation is covered and its data path needs a live
`/dev/fuse` mount, exercised manually via `-mount` rather than in the suite.

The coverage figure in the original report (19.9%) is superseded: 366 tests, up
from 319, and no source file is unreferenced.

One trap worth recording for anyone adding tests here: `PASS()` does **not**
return. An early `if (bad) PASS();` therefore falls through into whatever
follows. Several tests written during this work had that shape and could have
dereferenced NULL or used fd -1; all now `PASS(); return;`.

### O18-O24 — hygiene items, status at 0.48.0
Warnings: **zero** under `-Wall -Wextra -pedantic`, verified on a clean rebuild.
That closes the warning half of O18 and O19; the four unreachable VFP load/store
arms were removed rather than fixed, and the TMDS/PWM/LDRD defects above came
out of the same investigation. O20 (duplicate macros) is closed -- `clocks.h` now
includes `rp2350_memmap.h`. O22 is closed for the three tests that asserted
nothing. O23 is closed (`test_invariant_cyw43.c` deleted). O21 is the item above.
O24 (reproducible builds) and the 19.9%-coverage figure quoted in the original
report are both superseded by the numbers in this release.

### F1 — largely fixed in `e810fe4`, plus `f57db6b`
Now fixed: the instruction cache is invalidated after `flash_range_program` and
`flash_range_erase` (a firmware flash update used to write the bytes and then
jump into the stale decoded instructions); `misa` advertises X, so firmware can
see the Zb\* extensions this core implements; `mvendorid`/`marchid`/`mimpid`
report real values instead of an "unimplemented" zero; `MRET` clears `MPP`; and
`CSRRW` no longer reads the CSR when `rd == x0` -- which mattered because
reading `MEINEXT` clears the force bits it samples. `f57db6b` makes SIO `CPUID`
return the hart id.

Still open from F1: bus-fault exceptions are not raised (mcause 1/5/7), and
U-mode/PMP are absent -- so `misa.U` is deliberately *not* set, which is the one
deliberate divergence from the datasheet's `0x40901105`.

The Zcb quadrant-0 byte/halfword stores (`C.SB`, `C.SH`, `C.SBSP`, `C.SHSP`)
compute a base register and **no offset**, so they always write to base+0.
Attempted and reverted: the Zcb bit layout for these forms could not be
confirmed offline, and a wrong decode is worse than a known-bad one. Needs the
RISC-V unprivileged spec.

### Remaining datasheet-audit items
The Hazard3 work now covers: CLINT relocation, the Xh3irq CSRs and their array
semantics, peripheral interrupt delivery, mtval, WFI/MIE, Zcmp, icache
coherence on flash writes, the identity CSRs, `MRET`/`MPP`, `CSRRW` semantics
and hart-dependent `CPUID`. Still open: hart-1 launch uses invented SIO
registers (`0x1c0`-`0x1cc`, which on RP2350 are TMDS) instead of the FIFO
handshake, bus-fault exceptions, and U-mode/PMP.
