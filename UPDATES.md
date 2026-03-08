# Bramble RP2040 Emulator - Updates

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

### Known Remaining Issues

- Shared RAM (0x20040000) overlaps with Core 1's per-core RAM range (0x20021000-0x20042000);
  addresses 0x20040000-0x20041FFF resolve to Core 1's per-core RAM instead of shared RAM
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

| Version | Date       | Highlights                                                                |
|---------|------------|---------------------------------------------------------------------------|
| 0.5.0   | 2026-03-08 | Zero-copy dual-core, dispatch table, PRIMASK, SVC, ELF loader, test suite |
| 0.4.0   | 2026-03-08 | 15 bug fixes, 3 performance improvements, audit-driven                    |
| 0.3.0   | 2025-12-30 | Unified single/dual-core, FIFO, spinlocks |
| 0.2.1   | 2025-12-06 | Debug flags, NVIC audit |
| 0.2.0   | 2025-12-03 | GPIO peripheral support |
| 0.1.0   | 2025-12-02 | Initial release: CPU, timer, UART, UF2 loader |

## Git History Summary

```
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
