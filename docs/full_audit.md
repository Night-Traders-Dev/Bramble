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

### D6 — RP2350 PWM 12-slice body
`41e59ee` stopped RP2040's PWM at `0x40050000` shadowing RP2350's PLL_SYS, and
routed `0x400A8000` to the PWM model. The **register body** is still RP2040's
8-slice layout: RP2350's PWM is a different 12-slice block with two IRQ outputs,
so slice and channel registers decode wrongly. That needs the RP2350 §12.19 map,
which was not reachable when this was written.

### F1/F4 — remaining per-chip register differences
RP2350 PIO diverges past `0x124` (`IRQ0_INTE` at `0x170`, so PIO interrupts
cannot be enabled on RP2350), RP2350 DMA has 4 IRQ lines but 2 are implemented,
and RP2350 WATCHDOG has no TICK register. Each is a self-contained per-chip
variant of an existing model and should be done one block at a time with
firmware to test against, not in one pass.

### O20 — unreachable VFP decoders — **root cause found, feature gap not a mask typo**
`src/thumb32.c`. The audit recorded this as "masks that disagree with their
patterns". That is a real secondary defect, but it is not why the instructions
are unreachable, and fixing the masks alone changes nothing.

The actual cause is the dispatch. `thumb32_vfp_exec()` is called from exactly
one place, guarded by

```c
if ((upper & 0xEF00) == 0xEE00 || (upper & 0xEF00) == 0xED00)
```

but the encodings these arms implement have first halfwords elsewhere:

| Instruction | Encoding | First halfword | Reaches the VFP decoder? |
|---|---|---|---|
| `VLDR/VSTR s/d, [Rn, #imm]` | `D8xx`/`D9xx` | `0xD8xx`/`0xD9xx` | no |
| `VCVT.F32.U32 Sd, Sm` etc. | `EB80A4xx` | `0xEB80` | no |

These are **16-bit T2** encodings, not 32-bit Thumb-2, so they would need to be
reached from the 16-bit dispatcher, which never calls the VFP decoder at all.

Mask correction was attempted and reverted: with the masks fixed the arms still
are dead, so the change would have looked like a fix while changing no
behaviour. Doing this properly means adding 16-bit VFP dispatch to the Thumb
core -- a feature addition, not a bug fix -- and checking the 16-bit decoder
does not already claim `0xD8xx`-`0xD9xx`.

The compiler's `-Wtautological-compare` warning is left in place as the honest
signal that the code is dead.

### O21 — test coverage remains ~20%
This work added 9 regression tests (333 total, up from 324) and fixed three
tests that asserted nothing, but the gap is structural: eleven source files
still have no test references. Closing it needs a deliberate effort per file,
not opportunistic additions.

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
