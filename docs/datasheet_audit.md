# Bramble Datasheet Audit — RP2040 / RP2350 / RP2350-Hazard3

**Date:** 2026-09-30
**Version audited:** commit `45ccf08` (post-#16/#15 fixes)
**Ground truth:** official Raspberry Pi datasheets, text-extracted
(`rp2040.txt`, `rp2350.txt`; the Hazard3 core, CLINT, SIO and bootrom are all
specified inside the RP2350 datasheet).
**Supersedes:** `audit_report.md` (2026-06-28, v0.46.0) — that was an internal
code audit; this one is datasheet-referenced.

**Status note (v0.49.10):** this document is a point-in-time audit. Most of its
findings have since been fixed, several by the later register-decode pass, which
caught peripherals that were unreachable despite passing every test. For current
status, the outstanding queue and the items still blocked on hardware, read
`full_audit.md`.

## Verification status

Every **CRITICAL** and **HIGH** item below was independently re-checked against
the datasheet text and the source by the author, and the decisive line numbers
are quoted. Items marked *(second pass)* come from a parallel review sweep and
are reported as-reported; they have not been individually re-verified and are
ranked below the verified tier.

One reported **critical** was checked and **rejected** — see [R1](#r1-rejected-finding). It is recorded because it is the kind of claim that is easy to act on by mistake.

---

## Headline

The RP2040 (Cortex-M0+) path is in reasonable shape. The **RP2350** support is
substantially incomplete in a structural way rather than a detail way: the
emulator relocates the peripheral map for RP2350 but implements only parts of
it, and it carries the **RP2040 interrupt vector table on both chips**. The
**Hazard3** core is the weakest area — the interrupt/timer subsystem is
effectively non-functional, and several core CSRs sit at invented addresses.

### Severity summary

| Severity | Verified by author | Second pass |
|---|---|---|
| CRITICAL | 8 | 5 |
| HIGH | 12 | 11 |
| MEDIUM | 6 | 15 |
| LOW / INFO | 3 | 20 |

---

## A. Fixed during this audit

### A1 — [CRITICAL, BUG] Hazard3/UART/SPI fix had broken RP2350-ARM GPIO

Found *by* this audit: the fix for issue #16 (commit `e6fd508`) was itself
defective and was corrected in `45ccf08`.

`rv_translate_shared_addr()` rewrites RP2350 peripheral bases to their RP2040
equivalents before delegating to the shared bus. `e6fd508` addressed that by
making `uart_match()`/`spi_match()` accept **both** address spaces when
`membus_rp2350_mode` was set. The two chip maps overlap:

```
RP2040 UART1_BASE 0x40038000  ==  RP2350 PADS_BANK0_BASE 0x40038000
RP2040 SPI1_BASE  0x40040000  ==  RP2350 PADS_QSPI_BASE  0x40040000
```

So on the **Cortex-M33** path — which does *not* rewrite addresses — every
PADS_BANK0 access was claimed by UART1 and every PADS_QSPI access by SPI1.
Since `gpio_init()`/`gpio_set_function()` program pads first, RP2350-ARM GPIO
configuration was broken outright, and pad writes landed in `UART1CR` (bit 0 is
`UARTEN`), so they could spuriously enable or disable the UART.

Why it was not caught: the Hazard3 path rewrites bases *before* the shared bus,
so it was unaffected, and no bundled M33 firmware programs pad registers hard
enough to trip it.

**Fix:** the discriminator is the *caller*, not the address. `membus_rv_delegate`
is set for the duration of a shared-bus access that has already been rewritten
(`src/rp2350_rv/rv_membus.c`, all six accessors). Matchers keep their original
per-chip semantics. This also repaired the M33 path for `i2c_match()`,
`pwm_match()`, `is_adc_addr()`, `syscfg_match()`, `tbman_match()` and
`rtc_match()`, which compared against RP2040 bases only and so were unmapped on
RP2350-ARM for I2C, PWM, ADC, SYSCFG and TBMAN.

Lesson worth recording: **the RP2040 and RP2350 peripheral maps are not
disjoint, so address-space "accept both" is never a safe pattern here.**

### A2 — [CRITICAL, GAP, pre-existing] RP2350-ARM GPIO and PADS are entirely unmapped

Not caused by the above; independent and older. `include/gpio.h:7-8` hardcodes
`IO_BANK0_BASE 0x40014000` and `PADS_BANK0_BASE 0x4001C000` — the RP2040
values — and `gpio_bus_match()` (`src/membus.c`) has no RP2350 variant, so it
never claims RP2350's `0x40028000` / `0x40038000`. Confirmed empirically on
`littleos_pico2.uf2`:

```
[MEM] unmapped write32: 0x40038004     (RP2350 PADS_BANK0 + GPIO0)
[MEM] unmapped write32: 0x40038008     (RP2350 PADS_BANK0 + GPIO1)
[MEM] unmapped write32: 0x40039004     (+0x1000 XOR alias)
[MEM] unmapped write32: 0x4003B004     (+0x3000 CLR alias)
```

The Hazard3 path masks this because it rewrites the bases first. On
RP2350-ARM, **all** GPIO register access is inert. This is the same class as
issue #16, one peripheral over, and on the path that the earlier fix could not
see.

---

## B. CRITICAL findings (verified)

### B1 — All IRQ numbers are the RP2040 table on both chips

`include/nvic.h:49-74` hardcodes RP2040 vectors and the ~15 `nvic_signal_irq()`
call sites use them unconditionally, even with `membus_rp2350_mode = 1`.
`membus_rp2350_mode` changes only the IRQ *count* (`src/nvic.c:8-10`), never the
IRQ *numbers*. There is no RP2350 IRQ table anywhere in the repo.

Verified from the datasheets:

| IRQ | RP2040 (Table 80) | RP2350 (Table 95) | code raises |
|----:|---|---|---|
| UART0 | 20 | **33** | 20 |
| UART1 | 21 | **34** | 21 |
| SPI0 | 18 | **31** | 18 |
| SPI1 | 19 | **32** | 19 |
| IO_BANK0 | 13 | **21** | 13 |
| IO_QSPI | 14 | **23** | none |
| DMA_IRQ_0/1 | 11/12 | **10/11** | 11/12 |
| PIO0 | 7/8 | **15/16** | 7/8 |
| PIO1 | 9/10 | **17/18** | 9/10 |
| PIO2 | n/a | **19/20** | 9/10 (aliases PIO1) |
| PWM wrap | 4 | **8 and 9** | 4 (one of two) |
| ADC | 22 | **35** | 22 |
| I2C0/1 | 23/24 | **36/37** | 23/24 |
| CLOCKS | 17 | **30** | none |
| SIO FIFO/BELL | 15/16 | **25/26** | 15/16 |
| USBCTRL | 5 | **14** | 5 |
| TIMER1 | n/a | **4/5/6/7** | none |

Because `nvic_signal_irq()` latches *pending* bits, this is worse than
"interrupts never arrive": firmware's handler for vector 4 (`TIMER1_IRQ_0` on
RP2350) is invoked by a PWM wrap, GPIO bank0 raises vector 13 (`DMA_IRQ_3`), and
PIO2 reuses PIO1's vectors. `NUM_EXTERNAL_IRQS_RP2350` is 52, so the wrong
vectors are accepted silently. **Every peripheral interrupt is wrong on
RP2350.** This is the single highest-value defect in the codebase.

### B2 — RP2350 TIMER register map is shifted; interrupt enables are lost

Datasheet RP2350 §12.16: `0x34 LOCKED, 0x38 SOURCE, 0x3c INTR, 0x40 INTE,
0x44 INTF, 0x48 INTS` — RP2350 added two registers ahead of INTR. RP2040:
`0x34 INTR, 0x38 INTE, 0x3c INTF, 0x40 INTS`.

`include/timer.h:23-26` implements the RP2040 map for **both** chips. On
RP2350, `timer_hw->inte = 1` (offset 0x40) is decoded as `INTS` and dropped, so
**timer interrupts can never be enabled**. `LOCKED`/`SOURCE` reads return
INTR/INTE garbage.

### B3 — RP2350 TIMER1 never raises any interrupt

`src/rp2350_rv/rp2350_periph.c` sets TIMER1's `intr` and disarms, but
`nvic_signal_irq` appears nowhere under `src/rp2350_rv/`. Datasheet:
IRQs 4–7 = `TIMER1_IRQ_0..3`.

### B4 — Hazard3 CLINT is at the wrong address and shadows 12 SIO spinlocks

`include/rp2350_rv/rv_clint.h:27-28` places the CLINT at SIO `0xD0000100`,
size `0x30`, matched at `src/rp2350_rv/rv_membus.c:209,253` **before** the SIO
handler.

Datasheet Table 17: SIO `0x100`–`0x17c` is **SPINLOCK0…SPINLOCK31**. The
emulated CLINT therefore occupies SPINLOCK0–11. The real registers are
elsewhere and are **not implemented at all**:

```
0x1a0 RISCV_SOFTIRQ   0x1a4 MTIME_CTRL
0x1b0 MTIME  0x1b4 MTIMEH  0x1b8 MTIMECMP  0x1bc MTIMECMPH
```

Consequences: SDK `spin_lock_*` on SIO is broken (SPINLOCK0-11 read garbage;
SPINLOCK12-31 fall through unmapped so acquire loops exit without exclusivity),
and `SIO_IRQ_MTIMECMP` (IRQ 29) has no backing state.

### B5 — Hazard3 Xh3irq CSRs are at invented addresses

`include/rp2350_rv/rv_cpu.h:45-53` vs datasheet Table 367:

| emulator | actual CSR at that address |
|---|---|
| `0xBE0` MEIE0 | **MEIEA** (enable array) — right address, wrong semantics |
| `0xBE1` MEIE1 | **MEIPA** (pending array, read-only) |
| `0xBE2` MEIEA | **MEIFA** (force array) |
| `0xBE4` MEIFA | **MEINEXT** (next-IRQ) |
| `0xBE6` MEICONTEXT | not a CSR — must raise mcause 2 |
| `0xFE0/0xFE1/0xFE4` | not CSRs — must raise mcause 2 |

Missing: `0xBE3` MEIPRA (priority), `0xBE5` MEICONTEXT, `0xBF0` MSLEEP.
`rv_clint.h`'s `ext_priority[52]` is consequently dead. The datasheet's own
idiom `csrs 0xbe0, index | (mask << 16)` is latched as raw enable bits 0-31.
The SDK's RISC-V `hardware_irq` cannot enable, prioritise, dispatch or nest
anything.

### B6 — `mtval` is written with fault data; the datasheet says it is hardwired zero

`src/rp2350_rv/rv_cpu.c:244` writes `tval` into `CSR_MTVAL`, fed from ~13 trap
sites. Datasheet Table 367, offset `0x343`: *"Machine bad address or
instruction. **Hardwired to zero.**"* Firmware that trusts `mtval` to
distinguish fault causes mis-handles every trap.

### B7 — WFI never wakes when `mstatus.MIE` is clear → permanent hart hang

`src/rp2350_rv/rv_clint.c:143-150`:

```c
if (!(mstatus & MSTATUS_MIE))
    return 0;                 /* returns BEFORE the WFI wake check below */

/* WFI wake: any pending+enabled interrupt wakes the hart */
if (hart->is_wfi && (mip & mie_csr)) {
    hart->is_wfi = 0;
}
```

Datasheet §3.8.1.23: *"wfi **ignores** the global interrupt enable,
MSTATUS.MIE."* The canonical idle idiom (`csrci mstatus, 8` then `wfi`)
deadlocks the hart permanently even with `mtimecmp` expired. `main.c:1215`
then skips the hart forever. Two-line fix.

### B8 — Hazard3 Zcmp branch is unreachable; `cm.push`/`cm.pop` always trap

`src/rp2350_rv/rv_cpu.c:621` computes `op = (ci >> 8) & 0x7` = `inst[10:8]`,
but the branch is only reachable when `rd == 0` where `rd = inst[11:7] == 0`.
Since `inst[10:8] ⊂ inst[11:7]`, **`op` is always 0**, so `if (op == 6)` /
`else if (op == 7)` are dead and everything falls to `c_illegal`.

Verified: 31 encodings enter the branch, 0 reach `cm.push`/`cm.pop`, 31 become
illegal instructions. Zcmp is in the datasheet's *preferred* `-march`. (The
register-list logic at `:625`/`:638` is independently wrong — it is not the
Zcmp register set, and `rlist >= 4` skips the mandatory "push ra".)

---

## C. HIGH findings (verified)

| # | Finding | Evidence |
|---|---|---|
| **C1** | **GPIO interrupt registers are shadowed by the per-pin array** on RP2040. `NUM_GPIO_PINS` is 48 but RP2040 has 30 pins: GPIO29_CTRL ends at `0x0EC` and `INTR0` is at `0x0F0`, yet `gpio_write32` claims `IO_BANK0_BASE..+0x200`, maps `pin = offset/8`, and `return`s. INTR0-3, PROC0_INTE/INTF/INTS land on phantom `pins[30..37]`. Only the `+0x2000`/`+0x3000` aliases reach the real registers. | `src/gpio.c:261-277`, `:143-155`, `include/gpio.h:34`; datasheet RP2040 Table 283 |
| **C2** | **`gpio_acknowledge_irq()` can never clear an edge interrupt.** It is a *plain* store to `intr[bank]`, which C1 swallows. pico-sdk `gpio_acknowledge_irq()` therefore leaves edges latched → interrupt storm. | consequence of C1 |
| **C3** | **PIO `sm_set_base()` reads the wrong bit field.** `(pinctrl >> 5) & 0x1F` reads bits 9:5. Datasheet: `14:10 SIDESET_BASE`. Every `set pindirs, …` drives the wrong pin. (The same file uses `>>10` correctly at `pio.c:892`.) | `src/pio.c:124-126`; datasheet RP2040 `14:10` |
| **C4** | **SPI chip-select callback is never invoked.** `device.cs` is assigned at `spi.c:240` and called from nowhere. `sdcard_spi_xfer`, `emmc_spi_xfer`, `w5500_spi_xfer` all begin `if (!cs_active) return 0xFF;` — so **SPI-attached SD cards, eMMC and W5500 never respond to any command.** | `src/spi.c`; `grep -rn "device\.cs"` → 1 hit (the assignment) |
| **C5** | **UART FIFOs are half depth and IFLS trigger levels are half.** Datasheet: *"32 location deep"* for both TX and RX. Code: `UART_RX_FIFO_SIZE 16`, trigger levels 2/4/8/12/14. The SDK default `IFLS=0x2` (½ full = 16) fires at 8. | `include/uart.h:53`, `src/uart.c:56-65`; datasheet RP2040 §4.2.2.4 |
| **C6** | **`LCR_H.FEN` ignored; character-mode RX never interrupts.** Datasheet: with FIFOs disabled, *"the receive interrupt is asserted HIGH"* on 1 byte, cleared by one read. Code always compares against the IFLS level, so FEN=0 asserts at 8 bytes. | `src/uart.c:56-80`; datasheet §4.2.6.2 |
| **C7** | **RP2350 SIO `GPIO_HI_*` table is shifted by one register.** Code maps `0x30/34/38/3C` → `HI_OUT/SET/CLR/XOR`. Datasheet Table 17: `0x30`=GPIO_OE, `0x34`=GPIO_HI_OE, `0x38`=GPIO_OE_SET, `0x3c`=GPIO_HI_OE_SET … the HI output registers are at `0x14/0x1c/0x24/0x2c`. Writing `GPIO_OE` therefore sets the *high* bank's latch and leaves every low pin high-Z. | `src/rp2350_rv/rv_membus.c:126-152` |
| **C8** | **Hazard3 hart-1 launch uses invented SIO registers.** Code uses `0x1C0-0x1CC`. Datasheet: `0x1c0`=TMDS_CTRL, `0x1c4`=TMDS_WDATA, `0x1c8`/`0x1cc`=TMDS_PEEK/POP_SINGLE. Real launch is a FIFO handshake (`{0,0,1,vector_table,sp,entry}`), so `multicore_launch_core1()` never launches, and its FIFO traffic lands on the RP2040 SIO map where `0x54/0x58` are **UART1** registers. | `include/rp2350_rv/rv_membus.h:28-31`; datasheet §3.1.11, §5.3 |
| **C9** | **SIO `CPUID` returns 2 for both harts.** Datasheet: *"returns a value of 0 when read by core 0, and 1 when read by core 1"* — and core 1's boot depends on it. `rv_mem_read32` has no hart-ID parameter, so it structurally cannot. | `src/rp2350_rv/rv_membus.c:116`; datasheet §3.1.2 |
| **C10** | **No peripheral can raise `mip.MEIP`.** `rv_clint_set_ext_pending()` is called only from the test suite, never from emulator code, so `ext_pending` is permanently 0. **Every system IRQ is silently dropped for RISC-V firmware.** | `src/rp2350_rv/rv_clint.c:99-107` |
| **C11** | **`PIO2_BASE 0x50400000` is claimed unconditionally, but that is RP2040's `XIP_AUX_BASE`.** `pio_match()` has no RP2350 gate despite the "RP2350 only" comment. On RP2040, XIP_AUX accesses are captured by the PIO model. | `src/pio.c:613-622`, `include/pio.h:27`; datasheets RP2040/RP2350 AHB tables |
| **C12** | **RP2350 `IO_BANK0` interrupt registers are outside the modelled window.** They move to `+0x230` (INTR0) / `+0x248` (PROC0_INTE0) on RP2350 vs `+0x0F0` / `+0x100` on RP2040; `gpio_bus_match` only covers `+0x200`. Moot until A2 is fixed. | datasheet RP2350 Table 649 |

---

## D. MEDIUM (verified)

| # | Finding | Evidence |
|---|---|---|
| **D1** | **RP2350 has no RTC.** Datasheet: *"The RP2040 Real Time Clock (RTC) is not used in RP2350."* `rtc_match()` claims `0x4005C000` unconditionally (that address is unallocated on RP2350). Benign address-claim + dead state, but `CLK_RTC` should not exist on RP2350. | `include/rtc.h:7`; datasheet line 90400 |
| **D2** | **Duplicate `RP2350_WATCHDOG_BASE` / `RP2350_ROSC_BASE` defines** in `include/clocks.h:71,74` and `include/rp2350_rv/rp2350_memmap.h:80,83`. *Corrected in `docs/full_audit.md` §O19: the warning does not depend on include order — GCC suppresses redefinition when the replacement list is token-identical, and only these two differ in hex-letter case (`0x400d8000` vs `0x400D8000`). Six further duplicated macros are silent, which is the larger risk.* | reproduced `gcc`; visible in build logs |
| **D3** | **RP2040 FC0 offsets used on RP2350 → `frequency_count_khz()` spins forever.** RP2040 `FC0_STATUS 0x98`; RP2350 `0x84 RESUS_CTRL … FC0_STATUS 0xa4`. Code returns `DONE` only for 0x98. | `src/clocks.c:146-184`; datasheet RP2350 |
| **D4** | **RP2350 `CLK_DIV` is 16.16, not 8.8.** RP2040 reset `0x00000100`, RP2350 reset `0x00010000`. Code resets to `1u<<8` = **divide-by-zero on RP2350**. RP2350 also has 8 clock generators; code models 10 and misnames generator 9 `CLK_RTC` (it is ADC). | `include/clocks.h:23,35`, `src/clocks.c:37-39` |
| **D5** | **ROSC map shifted 4 bytes on RP2350, and `STATUS` bit decode is wrong on both chips.** Code returns `bit24` as ENABLED (it is `BADWRITE`), hard-wires bit12 `ENABLED` to 1, never sets `DIV_RUNNING`. | `src/clocks.c:407-433`; datasheet RP2350 §12.17 |
| **D6** | **RP2350 PWM is not implemented, and the RP2040 PWM base collides with RP2350 `PLL_SYS`.** `pwm_match` uses `0x40050000` = RP2350 PLL_SYS (claimed first by `is_clocks_addr`), while RP2350 PWM at `0x400A8000` is unmapped. RP2350 PWM is also 12 slices with a different map and two IRQ outputs. | `include/pwm.h:7`, `src/membus.c:1221/1669`; datasheet RP2350 §12.19 |

---

## E. Rejected finding

### R1 — "JAL/JALR link register ignores instruction width" — **NOT A BUG**

A second-pass review reported that `rv_cpu.c:745,757` write `rd = pc + 4`
unconditionally, claiming the spec requires `pc + len` and that a JAL at
`0x1000` should link to `0x1002`.

**This is incorrect.** JAL and JALR are 32-bit-only encodings; there is no
compressed JAL at that opcode. For a 4-byte instruction `pc + 4` *is* the next
instruction address and is exactly what the RISC-V spec requires. The
compressed `c.jal`/`c.jalr` forms are handled separately and correctly
(`rv_cpu.c:442`, `:614` use `pc + 2`). No change required.

Recorded because the claim is superficially convincing and cites a real
datasheet section; acting on it would have introduced a bug.

---

## F. Second-pass findings (not individually re-verified)

Reported by the parallel review sweep. Each cites specific datasheet lines and
code locations. Treat as a worklist to confirm before fixing.

### F1. RISC-V core *(high)*
- No bus-fault exceptions: load/store/AMO/fetch never raise mcause 5/7/1
  (`rv_cpu.c:799-1161`). Peripheral-probing loops spin instead of trapping.
- Instruction cache is never invalidated after `flash_range_program`/erase
  (`rv_bootrom.c:377-401`) → firmware patching flash executes stale words.
- Zcb quadrant-0 compressed loads/stores drop the offset (`rv_cpu.c:385`).
- `misa` missing X and U bits → `0x40001105` instead of `0x40901105`.
- `MVENDORID`/`MARCHID`/`MIMPID` all 0; should be `0x493` / `0x1b` / `0x86fc4e3f`.
- `mret` does not clear `MPP`; U-mode and PMP entirely absent.
- `mtvec` MODE[1] is not hardwired to 0; reset should be `0x00001fff`.
- `CSRRW` writes even when `rs1 == x0` (spec forbids).
- Bootrom magic is `'R','P'`; datasheet requires `'M','u',0x02` at `0x10`.
- Bootrom header fields at `0x14`/`0x16`/`0x18` are 16-bit, not 32-bit.
- `get_sys_info` claims `RV_SYS_INFO_NONCE`; datasheet says "not supported".
- TICKS block assumes 8 bytes/generator; datasheet says 12 (`RISCV_*` at
  `0x3c/0x40/0x44`). `mtime` is driven from a loop counter, not TICKS[5].
- `mtime` and `global_cycle_count` use different time bases, so VCD/`-script`
  timestamps drift from `mtime`.
- Missing `brev8`/`unzip`/`zip`; `packh` decoded with the wrong funct7/funct3;
  `c.mul` writes `x[rs1']` instead of `x[rd']`.
- `CSRRW`/`CSRRS`/`CSRRC` on unknown CSRs silently succeed instead of trapping.

*Verified-correct in the same sweep* (so these need not be re-audited): the
entire M extension including all div/rem edge cases; all compressed immediate
decoders (exhaustive over all 65 536 words); all 32-bit immediate extractors;
trap entry; vectored `mtvec`; interrupt priority `MEI > MSIP > MTIP`; the picobin
parser; the RP2350 APB base table and its `rv_translate_shared_addr` mapping.

### F2. GPIO *(medium/low)*
- Interpolator `PEEK`/`POP` never add `BASE0`/`BASE1` — every lane and full
  result is wrong (`membus.c:699-735`). Reproduces the datasheet's own worked
  example incorrectly.
- Interpolator sign-extension is UB when `MASK_MSB == 31`; on gcc/x86 the shift
  wraps and the result becomes all-ones.
- `ACCUM0_ADD`/`ACCUM1_ADD` read the raw accumulator instead of the masked lane.
- `1 << pin` UB for pins ≥ 31; `gpio_set_pin(40,1)` writes **bit 8**.
- `NUM_GPIO_PINS` 48 admits 18 phantom pins on RP2040; `NUM_GPIO_PINS_RP2040`
  is defined but never used.
- `GPIOx_CTRL` writes masked to `val & 0x1F`, destroying OUTOVER/OEOVER/INOVER/IRQOVER.
- PADS_BANK0 window stops at `+0x80`, so the SWD pad at `0x80` is unreachable;
  SWCLK reset is `0x56` but should be `0x96`.
- `FUNCSEL` reset is 5 (`SIO`); datasheet says `0x1f` (`NULL`) on both chips.
- `IO_QSPI` STATUS/INTR/INTS hard-wired to 0; no QSPI-bank GPIO interrupt.
- `PADS_QSPI` is a raw zeroed 8-word array with no field decoding or reset values.

### F3. UART/SPI *(medium)*
- Overrun never sets `RSR.OE` / `RIS.OERIS`; `IMSC.OEIM`/`ICR.OEIC` are dead.
- RX timeout interrupt (`RIS[6]`) never implemented.
- TX interrupt never re-asserts after `ICR=TXIC` → the canonical IRQ-driven
  UART loop delivers one interrupt then deadlocks.
- Interrupt line is edge-triggered, not level-triggered.
- Atomic SET/CLR/XOR aliases perform the interposer RMW in software against the
  model, so a SET-alias write to `UARTDR`/`SSPDR` *consumes an RX byte*.
  (Datasheet §2.1.2 puts this in a bus interposer — decoding is right,
  semantics are wrong.)
- `SSPCPSR[0]` must always read 0; `SSP_PERIPHID2` returns `0x04`, should be `0x34`.
- `SSPRIS` reset `0x0` should be `0x1` (firmware polling `TXRIS` before the first
  transfer spins forever); `UARTCR` reset `0x0` should be `0x301`.
- `CR0.DSS` (4–16 bit) ignored — always 8-bit frames, silently corrupting
  16-bit SPI devices.
- XIP SSI: `IDR`/`VERSION_ID`/`SPI_CTRLR0` reset values wrong, 8 registers
  missing, no TX FIFO (`TFE`/`TFNF`/`TXFLR` hardcoded).
- `RESETS` writes do not reset the peripheral models, so SDK `uart_reset()` is
  a no-op.
- `UARTILPR` unimplemented; RTS/CTS flow control unimplemented.

### F4. DMA/PIO/clocks *(medium)*
- RP2350 DMA has 4 IRQ lines; 2 implemented.
- `TIMELW` write-combine rule not implemented.
- PIO side-set and delay entirely unimplemented; `IN_BASE` uses `% 30` instead of
  modulo-32; OSR not saturated at 32; `MOV OSR` does not reset the counter.
- RP2350 PIO register map differs past `0x124` (`IRQ0_INTE` at `0x170`) → PIO
  interrupts cannot be enabled at all on RP2350.
- `IO_QSPI`, `CLOCKS`, `RTC`, `POWMAN` interrupts are never raised at all.
- DMA `READ_ERROR`/`WRITE_ERROR`/`AHB_ERROR` are never set.
- RP2350 WATCHDOG has no TICK register and a read-only `TIME[23:0]`; the code
  maps `0x2C` to TICK, so firmware writes to `TIME` pollute CTRL/TICK.
  `REASON` always reads 0, so `watchdog_enable_caused_reboot()` can never report
  a watchdog reset.
- `TIMER_DBGPAUSE` reset should be `0x3`.

---

## G. Suggested fix order

1. **B1** (IRQ table) — introduce per-chip tables and retrofit ~15 call sites.
   Everything else downstream is masked by it.
2. **B7** (WFI) — two lines, unblocks RISC-V idle loops.
3. **A2** (RP2350 IO_BANK0/PADS_BANK0 bases) — same shape as the #16 fix.
4. **B4 + B5 + C10** as one unit — move the timer to SIO `0x1a0-0x1bc`, put
   MEIEA/MEIPA/MEIFA/MEIPRA/MEINEXT/MEICONTEXT at their real addresses, and
   publish `ext_pending` from the peripheral models. Without these, RISC-V
   interrupts and the machine timer are unusable.
5. **B2 + B3 + D6 + F4(PIO map)** — the RP2350 blocks that stop Pico 2 firmware
   booting.
6. **C1/C2** (GPIO IRQ shadowing) — unblocks all GPIO interrupt firmware.
7. **C4** (SPI chip select) — unblocks SD/eMMC/W5500.
8. **C3** (PIO SIDESET_BASE) — one-line, high value.

## H. Note on the earlier audit

`audit_report.md` (2026-06-28) is a code-internal review and is now stale
(v0.46.0, commit `ca5bff3`). It does **not** reference the datasheets, so it
cannot have caught the mapping and register-layout class of defect that
dominates this report. Its CRITICAL finding C1 (SIO GPIO writes dropped) is
genuinely fixed — see `a06d497` in the git history.
