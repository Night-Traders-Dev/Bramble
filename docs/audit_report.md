# Bramble Emulator - Comprehensive Audit Report

**Date:** 2026-06-28  
**Auditor:** Automated codebase audit  
**Scope:** All source files in `src/`, `src/rp2350_rv/`, `src/rp2350_arm/`, and `include/`  
**Version audited:** v0.46.0 (commit ca5bff3)

---

## Summary

| Severity | Count |
|----------|-------|
| Critical | 9 |
| High | 20 |
| Medium | 30 |
| Low | 36 |
| **Total** | **95** |

### Categories

| Category | Count |
|----------|-------|
| BUG | 42 |
| CORRECTNESS | 27 |
| PERF | 14 |
| SECURITY | 12 |

---

## Critical Issues (9)

### C1. SIO GPIO writes silently dropped

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1126-1129`, `src/membus.c:928-963` |
| **Category** | BUG |
| **Severity** | Critical |

**Description:** `mem_write32` routes SIO addresses (`0xD0000000`-`0xD00000FF`) to `sio_write32()` before `gpio_write32()` can handle them. But `sio_write32()` has no cases for GPIO_OUT (`0x10`), GPIO_OUT_SET (`0x14`), GPIO_OUT_CLR (`0x18`), GPIO_OUT_XOR (`0x1C`), GPIO_OE (`0x20`), GPIO_OE_SET (`0x24`), GPIO_OE_CLR (`0x28`), GPIO_OE_XOR (`0x2C`) -- they all hit the `default: break;` and are silently lost. Meanwhile `sio_read32()` at line 977-986 **does** delegate GPIO reads to `gpio_read32()`. The write side is missing the same delegation.

**Impact:** All SIO GPIO output/oe operations via 32-bit writes do nothing. Firmware that toggles GPIO pins via `gpio_put()` (which writes SIO_GPIO_OUT_SET/CLR) will fail.

**Fix:** Add GPIO write delegation in `sio_write32()`, mirroring the read path in `sio_read32()`. Or reorder `mem_write32` to check `gpio_bus_match()` before the SIO range.

---

### C2. Non-aliased GPIO interrupt register reads/writes intercepted by pin handler

| Field | Value |
|-------|-------|
| **File** | `src/gpio.c:143-155` (read), `src/gpio.c:261-276` (write) |
| **Category** | BUG |
| **Severity** | Critical |

**Description:** Both `gpio_read32` and `gpio_write32` have a pin-configuration handler that matches the entire range `IO_BANK0_BASE` to `IO_BANK0_BASE + 0x200`:

```c
if (addr >= IO_BANK0_BASE && addr < IO_BANK0_BASE + 0x200) {
    uint32_t pin = offset / 8;  // For INTR0 at 0xF0: pin=30
    uint32_t reg = offset % 8;  // reg=0 = GPIO_STATUS_OFFSET
    if (pin < NUM_GPIO_PINS) {
        if (reg == GPIO_STATUS_OFFSET) return gpio_state.pins[pin].status; // WRONG!
```

For `IO_BANK0_BASE + 0xF0` (INTR0): `pin=30`, `reg=0=STATUS_OFFSET` -> returns `pins[30].status` instead of `intr[0]`. The interrupt register handler at lines 158-174 (read) and 283-325 (write) is **dead code** for non-aliased addresses because the pin handler returns first.

Aliased writes (XOR/SET/CLR at `+0x1000`/`+0x2000`/`+0x3000`) DO reach the interrupt handler since the pin handler only matches the base `0x200`-byte window.

**Impact:** Reading INTR/INTE/INTF/INTS via normal (non-aliased) access returns wrong values. Writing INTR (W1C clear) via normal access writes to pin status instead. The Pico SDK reads `io_bank0_hw->intr[i]` via the base address, so this breaks all GPIO interrupt handling.

**Fix:** In both read and write paths, check for interrupt register offsets (>= 0xF0) before the pin configuration handler, or restructure the pin handler to only match offsets < 0xF0.

---

### C3. Interpolator sign extension: undefined behavior when mask_msb == 31

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:719-720` |
| **Category** | BUG |
| **Severity** | Critical |

**Description:**

```c
if (sign_ext && (val & (1u << mask_msb))) {
    val |= ~((1u << (mask_msb + 1)) - 1);  // 1u << 32 is UB!
}
```

When `mask_msb` is 31, `1u << (mask_msb + 1)` = `1u << 32`, which is undefined behavior in C (shifting by the width of the type). This can crash, produce wrong results, or behave unpredictably depending on the compiler/architecture.

**Fix:** Handle `mask_msb == 31` as a special case: if sign_ext and bit 31 is set, set all upper bits (which is already all of them), so the value stays as-is.

---

### C4. GPIO `1 << pin` undefined behavior for pins >= 31

| Field | Value |
|-------|-------|
| **File** | `src/gpio.c:396`, `src/gpio.c:419`, `src/gpio.c:421` |
| **Category** | BUG |
| **Severity** | Critical |

**Description:**

```c
if (gpio_state.gpio_oe & (1 << pin)) {   // gpio_get_pin, line 396
    gpio_state.gpio_oe |= (1 << pin);     // gpio_set_direction, line 419
    gpio_state.gpio_oe &= ~(1 << pin);    // gpio_set_direction, line 421
```

`1` is `signed int`. For pin >= 31, `1 << 31` shifts into the sign bit (UB), and `1 << 32+` exceeds int width (UB). `NUM_GPIO_PINS` is 48, so these functions can be called with pin values 31-47.

**Fix:** Use `1u << pin` (or `1ULL << pin` for pins >= 32). Note that `gpio_state.gpio_oe` is `uint32_t`, so pins >= 32 can't be represented anyway -- the struct needs `uint64_t` fields for 48-pin support, or pin validation should cap at 31.

---

### C5. Stack buffer overflow in W5500 SEND command

| Field | Value |
|-------|-------|
| **File** | `src/w5500.c:184-190` |
| **Category** | SECURITY |
| **Severity** | Critical |

**Description:** `data_len = tx_wr - tx_rd` is a `uint16_t` subtraction. If the firmware sets `tx_wr` and `tx_rd` such that their difference exceeds `W5500_TX_BUF_SIZE` (2048) -- for example, `tx_wr = 0x0900, tx_rd = 0x0000` gives `data_len = 2304` -- the loop `for (uint16_t i = 0; i < data_len; i++) { send_buf[i] = ... }` writes beyond the 2048-byte `send_buf` stack buffer. This is a stack buffer overflow triggered by emulated firmware input. The subsequent `send()` at line 199 also reads beyond the buffer (buffer over-read).

**Fix:** Clamp `data_len` to `W5500_TX_BUF_SIZE`: `if (data_len > W5500_TX_BUF_SIZE) data_len = W5500_TX_BUF_SIZE;`

---

### C6. Wire RX buffer too small for Ethernet frames

| Field | Value |
|-------|-------|
| **File** | `src/wire.c:288-289` (with `wire.h:29` `WIRE_IO_BUFFER_SIZE = 256`) |
| **Category** | BUG |
| **Severity** | Critical |

**Description:** The `rx_buf` is only 256 bytes (`WIRE_IO_BUFFER_SIZE`). Ethernet frames can be up to `WIRE_ETH_MAX_FRAME` (1522) bytes. The total wire framing for an ETH frame is `sizeof(wire_msg_t) + 2 + frame_len = 4 + 2 + 1522 = 1528` bytes. The 256-byte buffer can never hold a complete large ETH frame. When the buffer fills at 256 bytes, the code at line 296-299 detects "receive buffer overflow" and disconnects the peer. This means any Ethernet frame larger than ~250 bytes will kill the wire connection. This effectively makes the ETH wire protocol non-functional for standard Ethernet frames.

**Fix:** Increase `WIRE_IO_BUFFER_SIZE` to at least `sizeof(wire_msg_t) + 2 + WIRE_ETH_MAX_FRAME + 4` (1528+), or use a separate larger buffer for ETH frame reception.

---

### C7. Integer overflow in ROM flash erase bounds check

| Field | Value |
|-------|-------|
| **File** | `src/rom.c:443`, `src/rom.c:456` |
| **Category** | SECURITY |
| **Severity** | Critical |

**Description:** `rom_intercept_flash()` checks `offs + count <= FLASH_SIZE` before writing to `cpu.flash[]`. If firmware passes `offs=0xFFFFFF00` and `count=0x200`, the addition wraps to `0x100`, which passes the check. The subsequent `memset(&cpu.flash[offs], 0xFF, count)` then writes far beyond the 16MB flash buffer, causing heap/stack corruption. The same overflow exists for `flash_range_program` at line 456.

**Fix:** Use safe bounds check: `if (offs < FLASH_SIZE && count <= FLASH_SIZE - offs)`.

---

### C8. RP2350 function table overlaps with stub addresses

| Field | Value |
|-------|-------|
| **File** | `src/rom.c:620-644` (`rom_patch_rp2350_arm`) |
| **Category** | BUG |
| **Severity** | Critical |

**Description:** The 32-bit RP2350 function table starts at 0x0740 with 16 entries of 8 bytes each (spanning 0x0740-0x07C7). The GS and RB stub `BX LR` instructions are written at 0x0750 and 0x0752 (lines 643-644), which is inside the table -- specifically at entry 2 (MEMSET at 0x0750). This corrupts the MEMSET entry's code field from `ROM_FUNC_MEMSET` to `0x47704770`. When firmware calls `rom_func_lookup('MS')`, the lookup function at 0x0700 reads the corrupted entry and fails to find memset. This breaks all RP2350 ARM firmware that relies on ROM memset.

**Fix:** Place the GS/RB stubs at an address outside the table range, e.g., 0x07C8/0x07CA, and update the table entries to point there.

---

### C9. Partial writes corrupt vnet peer framing

| Field | Value |
|-------|-------|
| **File** | `src/vnet.c:184-193` |
| **Category** | BUG |
| **Severity** | Critical |

**Description:** `vnet_peers_tx()` writes the 4-byte length header and frame payload in two separate `write()` calls on a non-blocking socket. If the header write returns 1-3 (partial write), the check `n == 4` fails, the frame write is skipped, but the partial header bytes are already sent. The peer receives a truncated length prefix, permanently desynchronizing the framing protocol. Similarly, the frame `write()` at line 186 can return a partial count (`> 0` but `< len`), and only `n > 0` is checked -- truncated frames are silently sent, corrupting the peer's receive stream.

**Fix:** Use `writev()` with two iovec elements (header + frame) for an atomic write, or buffer the complete frame (header + payload) into a single buffer and write that. Handle partial writes by buffering the remainder and retrying on the next poll cycle.

---

## High Issues (20)

### H1. Shift carry flag extraction uses signed shift (UB)

| Field | Value |
|-------|-------|
| **File** | `src/instructions.c:610`, `672`, `696`, `720` |
| **Category** | BUG |
| **Severity** | High |

**Description:** `1 << (32 - shift)` with signed `1` when shift=1 gives `1 << 31` = signed overflow (UB). Should use `1u`. Affects `instr_shift_logical_left`, `instr_lsls_reg`, `instr_lsrs_reg`, `instr_asrs_reg`.

**Fix:** Use `1u << (32 - shift)` and `1u << (shift - 1)` in all shift carry extraction code.

---

### H2. Float/double shift operations UB for shift >= 31

| Field | Value |
|-------|-------|
| **File** | `src/rom.c:319`, `335`, `340`, `341`, `393`, `402`, `409`, `416` |
| **Category** | BUG |
| **Severity** | High |

**Description:** Expressions like `(float)(1 << cpu.r[1])` and `(double)(1u << cpu.r[2])` are undefined behavior when the shift amount is >= 31 (for signed `int`) or >= 32 (for `unsigned`). A register value of 32 or higher causes UB, and the real RP2040 ROM handles shifts up to 63 with saturation.

**Fix:** Clamp the shift amount to valid range, or use 64-bit types: `(float)(1ULL << (cpu.r[1] & 63))`.

---

### H3. GPIO interrupt detection only processes 4 of 6 register banks

| Field | Value |
|-------|-------|
| **File** | `src/gpio.c:64`, `src/gpio.c:46` |
| **Category** | BUG |
| **Severity** | High |

**Description:**

```c
static void gpio_detect_events(...) {
    for (int reg = 0; reg < 4; reg++) {   // Should be 6 for 48 pins
```

```c
static void gpio_check_irq(void) {
    for (int i = 0; i < 4; i++) {         // Should be 6
```

`NUM_GPIO_PINS` is 48 (6 registers x 8 pins). The `gpio_state_t` struct correctly has `intr[6]`, `proc0_inte[6]`, etc. But the event detection and IRQ check loops only iterate 4 times. Pins 32-47 (registers 4-5) are never checked for interrupt conditions.

**Impact:** GPIO interrupts for pins 32-47 never fire. Relevant for RP2350 mode (48 pins).

**Fix:** Replace hardcoded `4` with `(NUM_GPIO_PINS + 7) / 8` or `6`.

---

### H4. No proc1 GPIO interrupt registers modeled

| Field | Value |
|-------|-------|
| **File** | `include/gpio.h:70-73`, `src/gpio.c:44-54` |
| **Category** | CORRECTNESS |
| **Severity** | High |

**Description:** The `gpio_state_t` struct only has `proc0_inte`, `proc0_intf`, `proc0_ints`. There are no `proc1_inte/intf/ints` arrays. On RP2040, each core has independent GPIO interrupt enable/force/status. The IO_BANK0 register map includes proc1 interrupt registers at offsets 0x130-0x15F.

`gpio_check_irq()` only computes `proc0_ints` and signals `IRQ_IO_IRQ_BANK0` based on proc0's enable mask. If core 1 has GPIO interrupts enabled but core 0 doesn't, no interrupt is signaled.

**Fix:** Add `proc1_inte[6]`, `proc1_intf[6]`, `proc1_ints[6]` to the struct. Compute proc1_ints in `gpio_check_irq()` and signal the interrupt if either core has active interrupts.

---

### H5. Dual-core RAM writes skip icache/jit invalidation

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1864-1868` |
| **Category** | BUG |
| **Severity** | High |

**Description:**

```c
if (addr >= active_ram_base && addr < active_ram_base + active_ram_size) {
    uint32_t offset = addr - active_ram_base;
    memcpy(&get_ram()[offset], &val, 4);
    return;   // Missing: icache_invalidate_addr(addr); jit_invalidate_addr(addr);
}
```

The single-core `mem_write32` calls `icache_invalidate_addr(addr)` and `jit_invalidate_addr(addr)` after RAM writes (line 1029-1030). The dual-core version doesn't. If one core writes code into RAM for the other core to execute, the instruction cache and JIT may serve stale code.

**Fix:** Add `icache_invalidate_addr(addr); jit_invalidate_addr(addr);` after the `memcpy` in `mem_write32_dual`.

---

### H6. SysTick fast-skip path missing `corepool_wake_cores()`

| Field | Value |
|-------|-------|
| **File** | `src/nvic.c:124-130` |
| **Category** | BUG |
| **Severity** | High |

**Description:**

```c
if (remaining > reload) {
    uint32_t full_periods = (remaining - 1) / reload;
    remaining -= full_periods * reload;
    if (st->csr & 2) {
        st->pending = 1;
        // MISSING: corepool_wake_cores();
    }
}
```

The non-fast-skip path at lines 117-120 calls `corepool_wake_cores()` when setting `pending`. The fast-skip path doesn't. If the core is in WFI and the fast-skip path is taken, the core won't wake up for the SysTick interrupt.

**Fix:** Add `corepool_wake_cores();` after `st->pending = 1;` in the fast-skip block.

---

### H7. Subword GPIO writes clobber full 32-bit register

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1441`, `src/membus.c:1496` |
| **Category** | BUG |
| **Severity** | High |

**Description:**

```c
// mem_write16:
if (gpio_bus_match(addr)) {
    gpio_write32(addr & ~0x3, val);   // val is uint16_t, zero-extended to uint32_t
    return;
}
// mem_write8:
if (gpio_bus_match(addr)) {
    gpio_write32(addr & ~0x3, val);   // val is uint8_t, zero-extended to uint32_t
    return;
}
```

A byte write of `0xFF` to address `0x40014001` calls `gpio_write32(0x40014000, 0xFF)`, writing `0x000000FF` to the full 32-bit register and clobbering the upper 3 bytes. The NVIC subword handler at lines 1418-1426 correctly does a read-modify-write with masking. The GPIO handler does not.

**Fix:** Implement read-modify-write with byte/halfword masking, similar to the NVIC handler.

---

### H8. `mem_read16` for GPIO always returns lower 16 bits

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1786-1788` |
| **Category** | BUG |
| **Severity** | High |

**Description:**

```c
if (gpio_bus_match(addr)) {
    uint32_t val32 = gpio_read32(addr & ~0x3);
    return (uint16_t)(val32 & 0xFFFF);   // Always lower 16 bits!
}
```

If `addr & 0x3 == 2`, the upper 16 bits should be returned: `(val32 >> 16) & 0xFFFF`. The code always returns the lower 16 bits regardless of offset. Compare with `mem_read8` at line 1838 which correctly handles the byte offset.

**Fix:** `return (uint16_t)((val32 >> ((addr & 0x2) * 8)) & 0xFFFF);`

---

### H9. NVIC IPR write stores full 8 bits instead of masking to 0xC0

| Field | Value |
|-------|-------|
| **File** | `src/nvic.c:434` |
| **Category** | BUG |
| **Severity** | High |

**Description:** `nvic_set_priority()` at line 219 correctly masks with `0xC0` (Cortex-M0+ implements only bits 7:6). But the IPR register write stores all 8 bits:

```c
ns->priority[irq_idx] = (val >> (i * 8)) & 0xFF;   // Should be & 0xC0
```

This causes:
1. IPR reads returning non-zero lower bits (unlike real hardware where bits 5:0 read as 0)
2. The `priorities_nondefault` flag being set incorrectly if lower bits are non-zero, forcing the slow path in `nvic_get_pending_irq()` unnecessarily

**Fix:** Change `& 0xFF` to `& 0xC0` on line 434.

---

### H10. Missed wakeup race for halted cores in corepool

| Field | Value |
|-------|-------|
| **File** | `src/corepool.c:285-300` |
| **Category** | BUG |
| **Severity** | High |

**Description:** When a core is halted, the thread unlocks the mutex (line 286), then re-acquires it (line 296) to wait on the condvar. Between the unlock and re-lock, another thread or the main thread could call `corepool_wake_cores()` (condvar broadcast). Since this thread hasn't entered `pthread_cond_timedwait` yet, it misses the signal entirely. The 1ms timeout mitigates this, but it causes up to 1ms of unnecessary delay on every core re-launch.

**Fix:** Keep the lock held between the halt check and the condvar wait. Restructure so the thread doesn't unlock/re-lock between detecting halt and sleeping.

---

### H11. Per-byte `fflush(stdout)` on UART TX

| Field | Value |
|-------|-------|
| **File** | `src/uart.c:195-196` |
| **Category** | PERF |
| **Severity** | High |

**Description:** Every byte written to the UART data register triggers `putchar()` followed by `fflush(stdout)`. For firmware that outputs significant data (e.g., boot logs, printf debugging), this causes a syscall per byte, severely degrading performance. A 1KB log output requires 1024 syscalls.

**Fix:** Remove the per-byte `fflush`. Either rely on default stdio buffering, or flush periodically (e.g., on newline or every N bytes).

---

### H12. Per-byte network writes in netbridge

| Field | Value |
|-------|-------|
| **File** | `src/netbridge.c:184-196` |
| **Category** | PERF |
| **Severity** | High |

**Description:** `net_bridge_uart_tx()` calls `write(b->client_fd, &byte, 1)` for every UART byte transmitted. This is one syscall per byte, and with `TCP_NODELAY` set, each byte is sent in a separate TCP segment. For a UART running at 115200 baud, this creates up to 115200 syscalls and TCP segments per second.

**Fix:** Buffer UART TX bytes and flush in batches (e.g., on a timer, when the buffer is full, or when no more bytes are pending).

---

### H13. ELF loader limited to 2MB flash / 264KB RAM

| Field | Value |
|-------|-------|
| **File** | `src/elf.c:219`, `244`, `265` |
| **Category** | CORRECTNESS |
| **Severity** | High |

**Description:** `region_contains()` uses `FLASH_SIZE` (2MB) and `RAM_SIZE` (264KB), but the emulator allocates `FLASH_SIZE_MAX` (16MB) for flash and the RP2350 has 520KB SRAM. ELF firmware for RP2350 with segments above 2MB flash or 264KB RAM will be silently rejected ("outside flash/RAM bounds, skipping"). The UF2 loader correctly uses `FLASH_SIZE_MAX`.

**Fix:** Use `FLASH_SIZE_MAX` for flash bounds and the appropriate SRAM size for the active architecture.

---

### H14. No GDB packet checksum validation

| Field | Value |
|-------|-------|
| **File** | `src/gdb.c:188-196` |
| **Category** | BUG |
| **Severity** | High |

**Description:** `gdb_recv_packet()` reads the checksum bytes after `#` but never validates them against the computed checksum of the packet data. It unconditionally sends `+` (acknowledgment). Per the GDB protocol, the receiver must compute `sum(data) mod 256` and compare with the received checksum. If they don't match, it should send `-` (NAK) and wait for retransmission. Corrupted packets are silently accepted, potentially causing debugger malfunction (wrong addresses, wrong register values).

**Fix:** After extracting data, read the 2 checksum hex chars, compute the checksum, compare, and only send `+` on match (send `-` on mismatch and re-read).

---

### H15. SIGPIPE not handled on GDB socket write

| Field | Value |
|-------|-------|
| **File** | `src/gdb.c:130`, `147-153` |
| **Category** | BUG |
| **Severity** | High |

**Description:** `gdb_send_raw()` uses `write()` to send data to the GDB client socket. If the client disconnects unexpectedly, `write()` generates SIGPIPE, which by default terminates the process. The emulator would crash without cleanup. The `write()` returning -1 with `errno=EPIPE` is the proper behavior, but SIGPIPE preempts it.

**Fix:** Use `send(gdb.client_fd, data+sent, len-sent, MSG_NOSIGNAL)` instead of `write()`, or ignore SIGPIPE globally with `signal(SIGPIPE, SIG_IGN)`.

---

### H16. No GDB support in RISC-V (RV32) execution path

| Field | Value |
|-------|-------|
| **File** | `src/main.c:1127-1249` |
| **Category** | BUG |
| **Severity** | High |

**Description:** The RV32 execution loop never calls `gdb_should_stop()` or `gdb_handle()`. GDB is initialized at line 1020-1027 before the architecture check, and RISC-V hart pointers are set up (lines 1144-1147). But the RV32 loop at lines 1183-1249 has no GDB breakpoint/step handling. Using `-gdb` with `-arch rv32` silently fails -- GDB connects but breakpoints never trigger and single-step doesn't work.

**Fix:** Add GDB stop checks inside the RV32 loop, similar to the cooperative ARM path (lines 1351-1367).

---

### H17. RV32 path missing watchdog reboot, fault injection, script I/O, and storage flush

| Field | Value |
|-------|-------|
| **File** | `src/main.c:1183-1249` |
| **Category** | BUG |
| **Severity** | High |

**Description:** The RV32 execution loop doesn't check `watchdog_reboot_pending`, doesn't call `fault_check()`, `script_poll()`, or `sdcard_flush()`/`emmc_flush()`. This means:
- Watchdog reboots are silently ignored in RV32 mode
- Fault injection (`-inject-fault`) doesn't work
- Scripted I/O (`-script`) doesn't work
- SD card/eMMC periodic flush doesn't happen, risking data loss on crash

**Fix:** Add these checks/calls to the RV32 loop, mirroring the cooperative ARM path.

---

### H18. Integer overflow in `flash_persist_sync` bounds check

| Field | Value |
|-------|-------|
| **File** | `src/storage.c:51` |
| **Category** | SECURITY |
| **Severity** | High |

**Description:** `if (offset + len > FLASH_SIZE) return;` -- if `offset = 0xFFFFF000` and `len = 0x2000`, `offset + len = 0x100001000` which truncates to `0x00001000` (on 32-bit), passing the check. The subsequent `fseek` and `fwrite` would access `cpu.flash` out of bounds.

**Fix:** Use `if (offset >= FLASH_SIZE || len > FLASH_SIZE - offset) return;`

---

### H19. USB DPRAM buffer overflow on unaligned writes

| Field | Value |
|-------|-------|
| **File** | `src/usb.c:596-597` |
| **Category** | SECURITY |
| **Severity** | High |

**Description:** `usb_write32()` computes `off = base - USBCTRL_DPRAM_BASE` and does `memcpy(&usb_state.dpram[off], &val, 4)`. The match function checks `base < USBCTRL_DPRAM_BASE + USBCTRL_DPRAM_SIZE` (0x1000), but `off` can be up to 0xFFC (if base = DPRAM_BASE + 0xFFC, `off = 0xFFC`, `off + 4 = 0x1000`, OK). But if the address is not 4-byte aligned (e.g., base = DPRAM_BASE + 0xFFD), `off = 0xFFD`, and `memcpy` writes 4 bytes at indices 0xFFD-0x1000, with index 0x1000 being one byte past the 4096-byte `dpram` array. This is a heap buffer overflow.

**Fix:** Check `off + 4 <= USBCTRL_DPRAM_SIZE` before the memcpy, or align `off` down and handle partial writes.

---

### H20. `sched_yield()` on every lock release in corepool

| Field | Value |
|-------|-------|
| **File** | `src/corepool.c:381` |
| **Category** | PERF |
| **Severity** | High |

**Description:** After each step quantum (default 64 instructions), the thread calls `sched_yield()`, forcing a OS context switch. At 125MHz emulation speed, this means millions of context switches per second. The yield is intended to give the other core thread a chance to acquire the lock, but the OS scheduler already handles this via mutex contention. The explicit yield adds unnecessary overhead.

**Fix:** Remove `sched_yield()` and rely on mutex contention scheduling, or use an adaptive spin before yielding.

---

## Medium Issues (30)

### M1. Timer TIMEHW/TIMELW writes not atomic

| Field | Value |
|-------|-------|
| **File** | `src/timer.c:131-141` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** Writing TIMEHW updates the high word immediately. If the timer ticks between the TIMEHW and TIMELW writes, the counter has an inconsistent value (new high word, old low word). On real RP2040, the write is latched and only applied when TIMELW is written.

**Fix:** Latch the high word on TIMEHW write and apply both words together on TIMELW write.

---

### M2. Timer `fire_alarm` unconditionally signals NVIC regardless of INTE

| Field | Value |
|-------|-------|
| **File** | `src/timer.c:39-43` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** `timer_fire_alarm` always calls `nvic_signal_irq()` even when the corresponding INTE bit is not set. On real RP2040, the NVIC line follows `(INTR | INTF) & INTE`. The unconditional signaling sets the pending bit even when the interrupt is disabled. Later enabling INTE could fire a stale interrupt.

**Fix:** Check `timer_state.inte & (1 << i)` before calling `nvic_signal_irq`.

---

### M3. XIP SSI atomic alias on DR0 FIFO register corrupts FIFO state

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1058-1074` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** Atomic alias operations (SET/CLR/XOR) do a read-modify-write on the register. For DR0 (offset 0x60), reading pops from the RX FIFO and writing pushes to the TX FIFO. A SET alias write to DR0 would: pop a value from RX FIFO, OR it with val, and push the result to TX FIFO -- corrupting both FIFOs.

**Fix:** Skip atomic alias handling for FIFO registers (DR0), or handle them as direct writes.

---

### M4. Timer INTR W1C via SET atomic alias clears all set bits

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1112-1121` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** For TIMER_INTR (W1C), the SET alias reads `cur`, ORs with `val`, writes `cur | val` -- this clears all currently-set interrupt bits plus the ones in `val`, not just the bits in `val`.

**Fix:** For W1C registers, atomic aliases should operate on the raw register state, not through the W1C write path. Or simply don't apply alias logic to W1C registers.

---

### M5. GPIO CTRL register only stores 5 bits, loses override fields

| Field | Value |
|-------|-------|
| **File** | `src/gpio.c:273` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** `gpio_state.pins[pin].ctrl = val & 0x1F;` only stores FUNCSEL (bits 4:0). RP2040 GPIO CTRL also has OUTOVER (13:12), OEOVER (17:16), INOVER (26:24), IRQOVER (30). Firmware using output/input overrides will not work.

**Fix:** Store the full 32-bit value: `gpio_state.pins[pin].ctrl = val;` (or at minimum mask `val & 0x7F1F1F1F` for valid bits).

---

### M6. RV membus subword access missing CLINT and SIO handling

| Field | Value |
|-------|-------|
| **File** | `src/rp2350_rv/rv_membus.c:276-352` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** The 16-bit and 8-bit access functions handle SRAM, ROM, flash, and RP2350 peripherals, but not CLINT or SIO. These fall through to `mem_write16`/`mem_read16` etc., which don't handle CLINT at all and stub SIO. Subword CLINT/SIO accesses in RV mode silently fail.

**Fix:** Add CLINT and SIO handling to the subword access functions, or do word-level read-modify-write for these regions.

---

### M7. RV membus doesn't handle all XIP aliases

| Field | Value |
|-------|-------|
| **File** | `src/rp2350_rv/rv_membus.c:175-230` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** `rv_mem_read32` handles `RP2350_FLASH_BASE` and `RP2350_XIP_NOCACHE_NOALLOC_BASE` using `bus->flash`. Other XIP aliases (e.g., `XIP_NOALLOC_BASE` at 0x11000000, `XIP_NOCACHE_BASE` at 0x12000000) fall through to `mem_read32`, which reads from `cpu.flash` -- a different backing store. If the flash images differ, reads return wrong data.

**Fix:** Handle all XIP alias ranges in the RV membus using `bus->flash`.

---

### M8. Interp lane result returns unmasked value when mask_msb < mask_lsb

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:708-724` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** If `mask_msb < mask_lsb`, the mask window is empty and the result should be 0. The code returns the raw shifted accumulator.

**Fix:** Return 0 when `mask_msb < mask_lsb`, or initialize `val = 0` before the if block.

---

### M9. Interp BASE_1AND0 read returns base0 instead of combined value

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:770` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** On RP2040, BASE_1AND0 read returns BASE1:BASE0 (upper 16 bits = BASE1, lower 16 bits = BASE0). The code returns only `base0`.

**Fix:** `return ((interp->base1 & 0xFFFF) << 16) | (interp->base0 & 0xFFFF);`

---

### M10. Peer RX buffer deadlock with large frames in vnet

| Field | Value |
|-------|-------|
| **File** | `src/vnet.c:379-381` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** `peer_process_rx()` requires the full `4 + frame_len` bytes to be in `rx_buf` before consuming a frame. If a peer sends a frame whose length prefix indicates a size near `VNET_MAX_FRAME` (1522), the total needed is 1526 bytes -- exactly the buffer size. If the peer sends data in small chunks and the buffer fills before the complete frame arrives, `space` becomes 0 and no more data can be read. The incomplete frame can never be consumed, causing a permanent deadlock.

**Fix:** After processing all complete frames, if `rx_len > 0` but no complete frame is available and the buffer is full, either grow the buffer, compact it, or reset the connection.

---

### M11. Wire-originated frames not forwarded to TAP

| Field | Value |
|-------|-------|
| **File** | `src/vnet.c:218-221` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** `vnet_tx_frame()` only forwards to TAP when `src_port >= 0`. But `wire.c:198` calls `vnet_tx_frame(-1, ...)` for Ethernet frames received over the wire protocol. The `src_port = -1` check prevents these frames from reaching the TAP interface.

**Fix:** Use a distinct sentinel (e.g., `src_port = -2` for wire origin) or forward to TAP for all `src_port != VNET_TAP_PORT`.

---

### M12. ETH frame write bypasses TX buffer in wire

| Field | Value |
|-------|-------|
| **File** | `src/wire.c:399-405` |
| **Category** | BUG |
| **Severity** | High |

**Description:** `wire_send_eth_frame()` writes directly to the socket with `write()`, bypassing the `tx_buf` buffering mechanism used by `wire_send()`. If there is pending data in `tx_buf`, the ETH frame data is interleaved out-of-order with the pending TX buffer data, corrupting the protocol. Additionally, the return value of the frame `write()` at line 401 is completely ignored -- partial writes and EAGAIN are silently dropped.

**Fix:** Route ETH frames through `tx_buf` like other messages, or ensure `tx_buf` is flushed before direct writes. Check the return value of the frame write and handle partial writes/EAGAIN.

---

### M13. Read errors silently ignored in netbridge

| Field | Value |
|-------|-------|
| **File** | `src/netbridge.c:168-179` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** In `net_bridge_poll()`, when `read()` returns -1 (error), only the `n > 0` and `n == 0` cases are handled. The `n < 0` case falls through with no error handling. If the socket has a transient error (not EAGAIN), the error is silently ignored and the socket is never closed, leading to a persistent error state that spins on every poll cycle.

**Fix:** Add an `else if (n < 0)` branch that checks `errno != EAGAIN && errno != EWOULDBLOCK` and closes the socket on real errors.

---

### M14. No POLLERR/POLLHUP detection in netbridge

| Field | Value |
|-------|-------|
| **File** | `src/netbridge.c:164-180` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** `net_bridge_poll()` only checks `POLLIN` on the poll result. It never checks for `POLLERR`, `POLLHUP`, or `POLLNVAL`. A peer disconnect with data still in the buffer might not be detected until a `read()` returns 0, and some error conditions (e.g., connection reset) might not trigger POLLIN at all, leaving the socket in a zombie state.

**Fix:** Check `pfd.revents & (POLLERR | POLLHUP | POLLNVAL)` and close the socket.

---

### M15. Premature ESTABLISHED status on non-blocking connect in W5500

| Field | Value |
|-------|-------|
| **File** | `src/w5500.c:157-170` |
| **Category** | CORRECTNESS |
| **Severity** | High |

**Description:** `CONNECT` sets `s->regs[W5500_Sn_SR] = W5500_SOCK_ESTABLISHED` at line 159 before the `connect()` call. The non-blocking `connect()` may return `EINPROGRESS` (connection still pending), but the status is already ESTABLISHED. The `w5500_poll()` function never checks for connect completion (no POLLOUT check). If the connection fails, the socket stays in ESTABLISHED forever, and the firmware believes it's connected to a non-existent peer.

**Fix:** Keep status as SOCK_INIT (or a connecting state) until `poll()` detects POLLOUT on the socket, then check `SO_ERROR` via `getsockopt()` and set ESTABLISHED or CLOSED accordingly.

---

### M16. UDP RX missing W5500 header format

| Field | Value |
|-------|-------|
| **File** | `src/w5500.c:450-462` |
| **Category** | CORRECTNESS |
| **Severity** | High |

**Description:** For UDP sockets, the W5500 hardware prepends an 8-byte header (4 bytes source IP + 2 bytes source port + 2 bytes data length) before each packet in the RX buffer. The code copies raw `recv()` data directly into `rx_buf` without this header. Firmware reading the RX buffer will misinterpret the first 8 bytes of data as the UDP source address header, corrupting all UDP receive operations.

**Fix:** For UDP, use `recvfrom()` to get the source address, and write the 8-byte W5500 UDP header (source IP, source port, data length) before the data into `rx_buf`.

---

### M17. Non-atomic save corrupts on crash in storage

| Field | Value |
|-------|-------|
| **File** | `src/storage.c:62-67`, `74-77` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** `flash_persist_save_all()` opens the file with "wb" (truncating) or rewrites in-place. If the process crashes during the write, the file is left in a truncated/corrupt state. This is especially dangerous for `flash_persist_save_all` which truncates first.

**Fix:** Write to a temporary file and atomically rename on success.

---

### M18. Integer overflow in SD card block address calculation

| Field | Value |
|-------|-------|
| **File** | `src/sdcard.c:282`, `294`, `309`, `324` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** `uint32_t addr = sd->sdhc ? (arg * SD_BLOCK_SIZE) : arg;` -- `arg` is `uint32_t`, `SD_BLOCK_SIZE` is 512. If `arg = 0x00800000`, `arg * 512 = 0x100000000` overflows to 0. The bounds check `addr + SD_BLOCK_SIZE > sd->size` passes (0 + 512 <= 64MB), and the card reads from address 0 instead of the intended block.

**Fix:** Check for overflow before multiplication: `if (arg > 0xFFFFFFFFu / SD_BLOCK_SIZE) { queue_r1(sd, R1_ADDR_ERR); break; }`

---

### M19. Integer overflow in eMMC block address calculation

| Field | Value |
|-------|-------|
| **File** | `src/emmc.c:276`, `287`, `300`, `314` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** Same as M18. `uint32_t addr = arg * EMMC_BLOCK_SIZE;` can overflow for large `arg` values.

**Fix:** Check for overflow before multiplication.

---

### M20. Flush truncates file (non-atomic) in SD card

| Field | Value |
|-------|-------|
| **File** | `src/sdcard.c:149-156` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** `sdcard_flush()` opens the backing file with "wb" (truncating). If the write fails or the process crashes, the image file is destroyed. For large cards (64MB+), this means writing the entire file each flush.

**Fix:** Write to a temporary file and rename atomically. Alternatively, use "r+b" with fseek for incremental updates.

---

### M21. Same non-atomic flush in eMMC

| Field | Value |
|-------|-------|
| **File** | `src/emmc.c:149-156` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** Same as M20. `emmc_flush()` opens with "wb" (truncating).

**Fix:** Write to temporary file and rename.

---

### M22. USB config descriptor capped at 64 bytes

| Field | Value |
|-------|-------|
| **File** | `src/usb.c:325` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** `if (usb_state.config_total_len > 64) usb_state.config_total_len = 64;` truncates the configuration descriptor to 64 bytes. A standard CDC-ACM configuration descriptor is typically 67-75+ bytes. Truncation means endpoint descriptors at the end are lost.

**Fix:** Increase the cap to at least 255 (USB max descriptor length). The `in_accum` buffer is 256 bytes, which can handle this.

---

### M23. Per-write `fflush` on CDC data in USB

| Field | Value |
|-------|-------|
| **File** | `src/usb.c:408-409` |
| **Category** | PERF |
| **Severity** | Medium |

**Description:** `fwrite(&usb_state.dpram[buf_addr], 1, len, stdout); fflush(stdout);` flushes stdout after every CDC bulk transfer. For high-throughput CDC data, this is a significant performance bottleneck.

**Fix:** Remove per-write fflush, or flush only on newline or when the CDC FIFO is empty.

---

### M24. Flash erase/program limited to 2MB on RP2350 in ROM

| Field | Value |
|-------|-------|
| **File** | `src/rom.c:443`, `456` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** Uses `FLASH_SIZE` (2MB) for bounds checking, but RP2350 has 4MB flash. Flash operations above 2MB are silently ignored. The flash buffer itself is `FLASH_SIZE_MAX` (16MB).

**Fix:** Use `FLASH_SIZE_MAX` (or an RP2350-specific 4MB constant) for bounds checks.

---

### M25. UART TX interrupt not maintained after ICR clear

| Field | Value |
|-------|-------|
| **File** | `src/uart.c:222-224` |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** When the UART is enabled (CR write), `UART_INT_TX` is set in RIS. Since the TX FIFO is always empty (instant TX), the TX interrupt should remain asserted whenever TX is enabled. However, if firmware writes to ICR to clear the TX interrupt, it stays cleared until the next CR write. Real PL011 hardware re-asserts the TX interrupt as long as the TX FIFO is below the trigger level.

**Fix:** Re-assert `UART_INT_TX` in RIS whenever FR is read or on any register write, as long as TXE is set and the TX FIFO is empty.

---

### M26. UART RX timeout interrupt not implemented

| Field | Value |
|-------|-------|
| **File** | `src/uart.c` (entire file) |
| **Category** | CORRECTNESS |
| **Severity** | Medium |

**Description:** The PL011 asserts `UART_INT_RT` (bit 6, receive timeout) when the RX FIFO contains data but no new data has arrived for ~32 bit periods. This is essential for firmware that reads variable-length lines (e.g., a shell prompt). Without it, the last few bytes of a transmission that don't fill to the RX trigger level will never trigger an interrupt, causing input loss.

**Fix:** Implement a timer-based RX timeout that asserts `UART_INT_RT` when `rx_count > 0` and no push has occurred for a configured period.

---

### M27. ABS-only UF2 files don't set detected architecture

| Field | Value |
|-------|-------|
| **File** | `src/uf2.c:110-115`, `145-146` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** When a UF2 file contains only `UF2_FAMILY_RP2350_ABS` blocks, `detected_arch` is never set (the case at line 110 just `break`s). The fallback message at line 146 says "assuming RP2040" but doesn't actually set `detected_arch` to `FW_ARCH_ARM_M0P`. So `loader_detected_arch()` returns `FW_ARCH_UNKNOWN`, and main.c's auto-detection won't switch architecture. RP2350 absolute UF2 files would run in M0+ mode.

**Fix:** Set `detected_arch = FW_ARCH_ARM_M0P` (or a better default) in the fallback path, or detect RP2350 from the payload content (e.g., picobin blocks).

---

### M28. Breakpoint check is O(N) per instruction in GDB

| Field | Value |
|-------|-------|
| **File** | `src/gdb.c:659-673` |
| **Category** | PERF |
| **Severity** | Medium |

**Description:** `gdb_should_stop()` iterates through all 16 breakpoint slots on every instruction execution. When GDB is active, this linear scan runs per-instruction, adding overhead proportional to the number of breakpoint slots (even empty ones are checked).

**Fix:** Use a hash set keyed by address, or maintain a sorted array with binary search, or early-exit when `bp_count == 0`.

---

### M29. Race condition in threaded mode status reporting

| Field | Value |
|-------|-------|
| **File** | `src/main.c:1313-1321` |
| **Category** | BUG |
| **Severity** | Medium |

**Description:** In threaded mode, the status reporting reads `cores[CORE0].r[15]`, `cores[CORE1].r[15]`, `is_halted`, and `is_wfi` without holding `corepool_lock`. The core threads could be modifying these values concurrently. This is a data race that could print garbled/inconsistent status.

**Fix:** Acquire `corepool_lock()` before reading core state for status, or use atomic reads.

---

### M30. Unconditional `fprintf(stderr)` on every RV trap

| Field | Value |
|-------|-------|
| **File** | `src/rp2350_rv/rv_cpu.c:247-248` |
| **Category** | PERF |
| **Severity** | Medium |

**Description:** `rv_trap_enter()` always calls `fprintf(stderr, ...)` regardless of debug mode. In interrupt-heavy workloads, this floods stderr and significantly impacts performance due to the I/O syscall per trap.

**Fix:** Gate the `fprintf` behind `cpu->debug_enabled` like other debug output in the file.

---

## Low Issues (36)

### L1. Linear if-else chain for address routing

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1018-1391` (write32), `1505-1738` (read32) |
| **Category** | PERF |
| **Severity** | Low |

**Description:** `mem_write32` has ~30 sequential address comparisons before reaching late peripherals. A timer access requires ~10 comparisons. A lookup table indexed by `addr >> 24` (256 entries) would give O(1) dispatch.

**Fix:** Use a dispatch table indexed by upper address bits.

---

### L2. `gpio_detect_events` loops all 48 pins instead of iterating set bits

| Field | Value |
|-------|-------|
| **File** | `src/gpio.c:89-103` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** Iterates all 48 pins even if only 1 changed. Using `__builtin_ctz` to iterate only set bits would be faster. Also, `1u << pin` for pin >= 32 is UB (see C4).

**Fix:** `while (changed) { int pin = __builtin_ctz(changed); ... changed &= changed - 1; }`

---

### L3. USB read `fprintf` on hot path

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1663-1667` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** Every USB read evaluates this branch:

```c
static int usb_read_hit = 0;
if (usb_read_hit < 5) {
    fprintf(stderr, "[MEM] USB read addr=0x%08X\n", addr);
    usb_read_hit++;
}
```

Should be behind `mem_debug_unmapped` or `cpu.debug_enabled`.

**Fix:** Guard with `if (mem_debug_unmapped && usb_read_hit < 5)` or remove entirely.

---

### L4. GPIO STATUS register writable when should be read-only

| Field | Value |
|-------|-------|
| **File** | `src/gpio.c:270` |
| **Category** | CORRECTNESS |
| **Severity** | Low |

**Description:** `gpio_state.pins[pin].status = val;` allows writes to GPIO STATUS. On RP2040, GPIO STATUS is read-only (shows current pin state).

**Fix:** Ignore writes to STATUS, or only allow writable bits per the datasheet.

---

### L5. PADS_BANK0 VOLTAGE_SELECT write is a no-op stub

| Field | Value |
|-------|-------|
| **File** | `src/gpio.c:371-373` |
| **Category** | CORRECTNESS |
| **Severity** | Low |

**Description:** Writes to VOLTAGE_SELECT are silently ignored. Reads return a default pad value (0x56) instead of the VOLTAGE_SELECT value.

**Fix:** Store and return a VOLTAGE_SELECT register value.

---

### L6. NVIC IPR read returns full 8 bits instead of bits 7:6 only

| Field | Value |
|-------|-------|
| **File** | `src/nvic.c:319` |
| **Category** | CORRECTNESS |
| **Severity** | Low |

**Description:** On Cortex-M0+, IPR bits 5:0 read as 0 (RAZ). The code returns the full stored byte.

**Fix:** Mask with 0xC0: `((uint32_t)(ns->priority[irq_idx] & 0xC0)) << (i * 8)`

---

### L7. `priorities_nondefault` flag incorrectly set by lower IPR bits

| Field | Value |
|-------|-------|
| **File** | `src/nvic.c:439-441` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** If IPR write stored value 0x01 (lower bits set, effective priority 0x00), this flag is set to 1, forcing the slow CTZ path in `nvic_get_pending_irq()` even though all effective priorities are 0.

**Fix:** Check `(ns->priority[i] & 0xC0) != 0` instead.

---

### L8. RV SIO read returns 0xDEAD0000 marker for unhandled offsets

| Field | Value |
|-------|-------|
| **File** | `src/rp2350_rv/rv_membus.c:139` |
| **Category** | CORRECTNESS |
| **Severity** | Low |

**Description:** For GPIO_IN (0x04), GPIO_OUT (0x10), GPIO_OE (0x20), etc., `rv_sio_read` returns `0xDEAD0000 | offset`. The caller ignores it for these offsets, but it's a debugging artifact.

**Fix:** Return 0 instead of the marker, or document that the return value is ignored for fall-through cases.

---

### L9. RP2350 TIMER0 routed to RP2040 timer stub

| Field | Value |
|-------|-------|
| **File** | `src/rp2350_rv/rv_membus.c:65` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `rv_translate_shared_addr` translates RP2350 TIMER0 to RP2040 TIMER0. The RP2040 timer has only 4 alarms. The RP2350 has TIMER0 and TIMER1 with potentially different features. Routing TIMER0 to the RP2040 timer stub is a simplification.

**Fix:** Implement RP2350-specific timer handling.

---

### L10. Dead code: SIO spinlock state bitmap check unreachable

| Field | Value |
|-------|-------|
| **File** | `src/membus.c:1609-1611` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** After the SIO handler at line 1599-1601 which handles `SIO_BASE` to `SIO_BASE + 0x100` (including 0x5C), this code is never reached:

```c
if (addr == SIO_BASE + 0x5C) {
    return sio_spinlock_state_bitmap();
}
```

**Fix:** Remove the dead code.

---

### L11. Port slot leak on unregister in vnet

| Field | Value |
|-------|-------|
| **File** | `src/vnet.c:151-156` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `vnet_unregister_port()` marks a port inactive but never decrements `port_count` or compacts the array. If ports are repeatedly registered and unregistered, `port_count` reaches `VNET_MAX_PORTS` (8) and no new ports can be registered.

**Fix:** Either compact the array by shifting subsequent ports down, or use a free-list approach.

---

### L12. Excessive poll() syscalls in vnet peer polling

| Field | Value |
|-------|-------|
| **File** | `src/vnet.c:377-410` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** `vnet_poll_peers()` calls `poll()` twice per peer per poll cycle. With 4 peers, that's 8 poll syscalls per main loop iteration.

**Fix:** Build a single `struct pollfd` array with all peer + listen + TAP fds and call `poll()` once per cycle.

---

### L13. No minimum frame length check on peer-received frames

| Field | Value |
|-------|-------|
| **File** | `src/vnet.c:337-344` |
| **Category** | SECURITY |
| **Severity** | Low |

**Description:** `peer_process_rx()` delivers frames to port RX callbacks without checking `frame_len >= 14` (minimum Ethernet header). A malicious peer could send a 0-length frame.

**Fix:** Add `if (frame_len < 14) { p->rx_len = 0; return; }` before delivering frames.

---

### L14. Unnecessary double memcpy in wire_send

| Field | Value |
|-------|-------|
| **File** | `src/wire.c:349-351` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** `wire_send()` first copies the message header and payload into a stack buffer `buf`, then copies `buf` into `tx_buf`. Two memcpys when one would suffice.

**Fix:** Construct the message directly in `tx_buf + tx_len`.

---

### L15. Poll syscalls in inner read loop

| Field | Value |
|-------|-------|
| **File** | `src/wire.c:302` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** Inside the `while (pfd.revents & POLLIN)` loop, `poll(&pfd, 1, 0)` is called each iteration. Since the socket is non-blocking, simply attempting `read()` and checking for EAGAIN would avoid the poll syscall.

**Fix:** Replace the poll-in-loop with a direct `read()` and break on EAGAIN/EWOULDBLOCK.

---

### L16. Blocking connect during init in netbridge

| Field | Value |
|-------|-------|
| **File** | `src/netbridge.c:86` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** `create_connect_socket()` calls `connect()` on a blocking socket (non-blocking is set at line 93, after connect). If the remote host is unreachable, `connect()` blocks for the TCP timeout duration (potentially 75+ seconds), freezing the emulator during init.

**Fix:** Set non-blocking before `connect()`, handle `EINPROGRESS`, and poll for completion.

---

### L17. Small read buffer in netbridge

| Field | Value |
|-------|-------|
| **File** | `src/netbridge.c:167` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** `uint8_t buf[64]` -- only 64 bytes read per poll. For high-throughput UART data, this requires many `read()` calls.

**Fix:** Increase buffer to at least 1024 bytes.

---

### L18. Accepted connection leaks old fd in W5500

| Field | Value |
|-------|-------|
| **File** | `src/w5500.c:416-419` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** When accepting a TCP connection, the new `cfd` is stored in `s->host_fd` without checking if `s->host_fd` is already in use. If a previous connection's fd was not properly closed, it is leaked.

**Fix:** Close `s->host_fd` before assigning the new `cfd` if `s->host_fd >= 0`.

---

### L19. Large stack buffers per poll cycle in W5500

| Field | Value |
|-------|-------|
| **File** | `src/w5500.c:187`, `450` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** `send_buf[W5500_TX_BUF_SIZE]` (2048 bytes) and `tmp[W5500_RX_BUF_SIZE]` (2048 bytes) are allocated on the stack for each socket in each poll cycle. With 8 sockets, up to 16KB of stack per poll.

**Fix:** Use static or per-device persistent buffers. Use `memcpy` with wrap handling instead of byte-by-byte copy.

---

### L20. RX RSR 16-bit overflow in W5500

| Field | Value |
|-------|-------|
| **File** | `src/w5500.c:460-462` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `rx_rsr = (uint16_t)(rx_rsr + (uint16_t)n)` can overflow if `rx_rsr` is near 65535.

**Fix:** Mask `rx_rsr` to `W5500_RX_BUF_SIZE` after addition.

---

### L21. `persist_fp` not closed when path changes in storage

| Field | Value |
|-------|-------|
| **File** | `src/storage.c:19-27` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `flash_persist_set_path()` frees the old path string but does not close `persist_fp`. The open file handle still references the old file.

**Fix:** Close `persist_fp` before freeing the old path.

---

### L22. Dead code: sdhc set twice in SD card

| Field | Value |
|-------|-------|
| **File** | `src/sdcard.c:110-111` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** Line 110 computes `sd->sdhc` based on card size, then line 111 unconditionally overwrites it to 1. The first assignment is dead code.

**Fix:** Remove line 110.

---

### L23. CSD underflow for small card sizes in SD card

| Field | Value |
|-------|-------|
| **File** | `src/sdcard.c:62` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `uint32_t c_size = (uint32_t)(card_size / (512 * 1024)) - 1;` underflows if `card_size < 512 * 1024`.

**Fix:** Add a minimum size check or clamp `c_size` to 0.

---

### L24. State not reset on CS deassert during multi-block operations in SD card

| Field | Value |
|-------|-------|
| **File** | `src/sdcard.c:467-472` |
| **Category** | CORRECTNESS |
| **Severity** | Low |

**Description:** `sdcard_spi_cs()` resets `cmd_pos`, `resp_len`, and `resp_pos` on CS deassert, but does not reset `state`. If CS is deasserted during a multi-block read/write, the state persists across CS cycles.

**Fix:** Reset `state` to `SD_STATE_READY` on CS deassert if it's `SEND_MULTI` or `RECV_MULTI`.

---

### L25. Multi-packet IN data silently dropped if too large in USB

| Field | Value |
|-------|-------|
| **File** | `src/usb.c:146-151` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `pkt_len` (up to 1023 bytes) is checked against `in_accum` (256 bytes). If `in_accum_len + pkt_len > 256`, the data is silently dropped but `usb_complete_ep0_in()` is still called, acknowledging the transfer.

**Fix:** Increase `in_accum` to 1024 bytes or handle the overflow by stalling the endpoint.

---

### L26. W1C register alias handling incorrect for XOR/OR in USB

| Field | Value |
|-------|-------|
| **File** | `src/usb.c:636-662` |
| **Category** | CORRECTNESS |
| **Severity** | Low |

**Description:** For W1C registers (SIE_STATUS, BUFF_STATUS, etc.), only alias 0 (direct) and alias 3 (clear) are treated as write-to-clear. Aliases 1 (XOR) and 2 (OR) fall through to `ALIAS_APPLY`, which can SET bits on a W1C register.

**Fix:** For W1C registers, only allow clearing (alias 0 and 3), and ignore XOR/OR aliases or treat them as no-ops.

---

### L27. TAP read clamps to 1518, truncating VLAN frames

| Field | Value |
|-------|-------|
| **File** | `src/tapif.c:248` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `if (maxlen > 1518) maxlen = 1518;` clamps reads to 1518 bytes. But `VNET_MAX_FRAME` is 1522 (with VLAN tag headroom). VLAN-tagged frames from the host TAP are truncated.

**Fix:** Change the clamp to 1522 or remove it.

---

### L28. Infinite loop on write returning 0 in tapif

| Field | Value |
|-------|-------|
| **File** | `src/tapif.c:260-269` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** In `tapif_write()`, if `write()` returns 0, `total` doesn't advance, and the `while (total < len)` loop repeats indefinitely.

**Fix:** Add `if (n == 0) break;` or treat n == 0 as an error.

---

### L29. Command injection via interface name in NAT setup

| Field | Value |
|-------|-------|
| **File** | `src/tapif.c:148-149`, `153-154` |
| **Category** | SECURITY |
| **Severity** | Low |

**Description:** `tap_outgoing_iface` is detected from `ip route` output and interpolated into `system()` commands for iptables/nftables. A maliciously crafted interface name could inject commands.

**Fix:** Use `execvp()` instead of `system()`, or validate the interface name (only alphanumeric + dash characters).

---

### L30. Unsafe `strcmp` on non-null-terminated payload in CYW43

| Field | Value |
|-------|-------|
| **File** | `src/cyw43.c:304` |
| **Category** | SECURITY |
| **Severity** | Low |

**Description:** `strcmp(varname, "cur_etheraddr")` where `varname` points to payload data that may not be null-terminated. `strcmp` reads beyond valid data.

**Fix:** Use `strncmp(varname, "cur_etheraddr", payload_len) == 0`.

---

### L31. Unaligned struct access through cast pointers in CYW43

| Field | Value |
|-------|-------|
| **File** | `src/cyw43.c:94`, `117`, `230`, `265`, `284`, `285`, `566`, `584` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** Multiple locations cast `uint8_t *` buffer pointers to structured types containing `uint16_t`/`uint32_t` fields without ensuring alignment. On ARM32, unaligned access through typed pointers can cause a bus fault.

**Fix:** Use `memcpy` to read/write struct fields from byte buffers, or ensure buffers are declared with appropriate alignment.

---

### L32. Race condition reading `is_halted` without lock in corepool

| Field | Value |
|-------|-------|
| **File** | `src/corepool.c:287` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** After `pthread_mutex_unlock`, `cores[core_id].is_halted` is read without holding the lock. Another thread could modify `is_halted` between the unlock and the read.

**Fix:** Read `is_halted` while still holding the lock, or use an atomic variable.

---

### L33. Failed thread creation not handled in corepool

| Field | Value |
|-------|-------|
| **File** | `src/corepool.c:391-403` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `corepool_start_threads()` sets `corepool.running = 1` before creating threads. If `pthread_create` fails for all cores, `running` remains 1 but no threads are active. The main loop would spin forever with no actual execution.

**Fix:** Check if any thread was successfully started; if none, set `corepool.running = 0` and report an error.

---

### L34. `fflush(stdout)` should be `fflush(stderr)` in GDB

| Field | Value |
|-------|-------|
| **File** | `src/gdb.c:635` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** In `gdb_init()`, all `fprintf` calls write to `stderr`, but the flush call is `fflush(stdout)`. This is a no-op.

**Fix:** Change to `fflush(stderr)` or remove the line.

---

### L35. `-clock 0` causes zero-speed emulation

| Field | Value |
|-------|-------|
| **File** | `src/main.c:466-469` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `atoi` returns 0 for invalid input (e.g., `-clock abc`). Setting `cycles_per_us = 0` would cause timer ticks to never advance. There's no validation that the clock value is > 0. (Note: `timing_set_clock_mhz` does clamp to 1, but the main.c code path may not use it.)

**Fix:** Validate `mhz > 0` before setting the clock, or clamp to minimum 1.

---

### L36. Resource leak on emmc_init failure

| Field | Value |
|-------|-------|
| **File** | `src/main.c:993-997` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** If `emmc_init()` fails, the function returns `EXIT_FAILURE` without cleaning up the already-initialized SD card (if `sdcard_path` was also given). The SD card file handle and resources are leaked.

**Fix:** Call `sdcard_cleanup(&sdcard)` before returning on emmc failure.

---

## Additional Low Issues

### L37. No GDB acknowledgment wait after sending packets

| Field | Value |
|-------|-------|
| **File** | `src/gdb.c:137-153` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `gdb_send_packet()` computes and appends a checksum, sends the packet, and returns immediately without waiting for `+`/`-` from the GDB client. If the client sends `-` (NAK), the emulator never retransmits.

**Fix:** After sending, read one byte. If `-`, retransmit. If `+`, return success.

---

### L38. No distinction between M0+ and M33 for ARM ELFs

| Field | Value |
|-------|-------|
| **File** | `src/elf.c:138-145` |
| **Category** | CORRECTNESS |
| **Severity** | Low |

**Description:** ARM ELF files are always detected as `FW_ARCH_ARM_M0P`. An ARM ELF compiled for Cortex-M33 (RP2350) would not trigger M33 auto-detection.

**Fix:** Use ELF processor flags (`e_flags`) or section attributes to distinguish M0+ from M33.

---

### L39. PIO `read_pins`/`write_pins` hardcodes `% 30` instead of `% NUM_GPIO_PINS`

| Field | Value |
|-------|-------|
| **File** | `src/pio.c:148`, `157`, `164` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** The modulo 30 wraps pin indices to RP2040's 30 pins, but RP2350 has 48 pins. PIO pin operations on pins 30-47 will wrap incorrectly in RP2350 mode.

**Fix:** Use `% NUM_GPIO_PINS` instead of `% 30`.

---

### L40. RV32 path missing PIO and USB stepping

| Field | Value |
|-------|-------|
| **File** | `src/main.c:1183-1249` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** The cooperative ARM path calls `pio_step()` and `usb_step()` every iteration, but the RV32 path doesn't. If RP2350 RISC-V firmware uses PIO or USB, these peripherals won't advance during RV32 emulation.

**Fix:** Add `pio_step()` and `usb_step()` calls to the RV32 loop.

---

### L41. `step_count` not reset after watchdog reboot in threaded mode

| Field | Value |
|-------|-------|
| **File** | `src/main.c:1305-1310`, `1333` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** After `reboot_from_watchdog()` in threaded mode, `step_count` is not reset to 0. The safety limit check at line 1333 could trigger prematurely after a reboot.

**Fix:** Reset `step_count = 0` after `reboot_from_watchdog()` in the threaded path.

---

### L42. Predictable `/tmp/bramble-corepool.reg` path

| Field | Value |
|-------|-------|
| **File** | `src/corepool.c:107-112` |
| **Category** | SECURITY |
| **Severity** | Low |

**Description:** The registry file defaults to `/tmp/bramble-corepool.reg`. On a multi-user system, an attacker could pre-create this file or a symlink at this path.

**Fix:** Use `mkstemp` or a per-user path under `$XDG_RUNTIME_DIR`, and use `O_NOFOLLOW` to prevent symlink attacks.

---

### L43. No payload_size alignment validation in UF2

| Field | Value |
|-------|-------|
| **File** | `src/uf2.c:32`, `131` |
| **Category** | SECURITY |
| **Severity** | Low |

**Description:** `uf2_block_flash_offset` checks `payload_size > sizeof(block.data)` (476), but doesn't validate that `payload_size` is 256 or a multiple of 256 as required by the UF2 spec.

**Fix:** Validate that `payload_size` is 256 (or a multiple of 256) and that `target_addr` is 256-aligned.

---

### L44. DHCP options scan can read out of bounds in CYW43

| Field | Value |
|-------|-------|
| **File** | `src/cyw43.c:525-533` |
| **Category** | SECURITY |
| **Severity** | Low |

**Description:** The options scan loop reads `dhcp[i]` and `dhcp[i+1]` (for `olen`). The check prevents reading `olen` past the end, but `i += olen` can skip past `dhcp_len` without further checks.

**Fix:** Add explicit bounds checks before each array access.

---

### L45. Backplane read response truncated for large reads in CYW43

| Field | Value |
|-------|-------|
| **File** | `src/cyw43.c:1066-1067` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** For backplane reads, `total_words_bp = total_words + pad_words` (up to 516), but `pio_resp_buf` is only 512 elements. The cap truncates the response.

**Fix:** Increase `pio_resp_buf` to 516+ elements.

---

### L46. TAP poll reads only one frame per cycle in CYW43

| Field | Value |
|-------|-------|
| **File** | `src/cyw43.c:688-705` |
| **Category** | PERF |
| **Severity** | Low |

**Description:** `cyw43_tap_poll()` reads only one frame per call. Under high network traffic, this creates backpressure.

**Fix:** Add a loop to drain multiple frames (up to 16) per poll cycle.

---

### L47. IP header length not validated in CYW43

| Field | Value |
|-------|-------|
| **File** | `src/cyw43.c:508` |
| **Category** | BUG |
| **Severity** | Low |

**Description:** `int ip_hlen = (ip[0] & 0x0F) * 4;` -- if IHL is 0, `ip_hlen = 0`, and subsequent calculations point to the wrong location. No validation that IHL >= 5.

**Fix:** Validate `ip_hlen >= 20 && ip_hlen <= 60`.

---

## Performance Bottlenecks Summary

| # | File | Issue | Impact |
|---|------|-------|--------|
| P1 | `uart.c:195-196` | Per-byte `fflush(stdout)` on UART TX | 1 syscall per byte |
| P2 | `usb.c:408-409` | Per-write `fflush(stdout)` on CDC data | 1 syscall per transfer |
| P3 | `corepool.c:381` | `sched_yield()` every 64 instructions | Millions of context switches/sec |
| P4 | `rv_cpu.c:247-248` | Unconditional `fprintf(stderr)` on every RV trap | I/O flood in interrupt-heavy workloads |
| P5 | `membus.c:1018-1738` | Linear if-else chain for address routing | ~30 comparisons per memory access |
| P6 | `gdb.c:659-673` | O(N) breakpoint scan per instruction | 16 slot linear scan per instruction in GDB mode |
| P7 | `netbridge.c:184-196` | Per-byte network writes | 1 syscall + 1 TCP segment per byte |
| P8 | `vnet.c:377-410` | Excessive `poll()` syscalls in peer polling | 2 per peer per cycle |
| P9 | `w5500.c:187,450` | Large stack allocations per poll cycle | Up to 16KB stack per poll |
| P10 | `gpio.c:89-103` | GPIO event detection loops all pins | 48 iterations per change |
| P11 | `wire.c:349-351` | Double memcpy in `wire_send` | 2x memcpy per message |
| P12 | `membus.c:1663-1667` | USB read `fprintf` on hot path | Branch evaluation per USB read |

---

## Top 5 Fixes by Impact

1. **C1 (SIO GPIO writes dropped)** -- Breaks all GPIO output; most firmware affected
2. **C5 (W5500 stack buffer overflow)** -- Security vulnerability, exploitable by firmware
3. **C7 (ROM flash erase overflow)** -- Security vulnerability, heap corruption
4. **C8 (RP2350 function table overlap)** -- Breaks all RP2350 ARM ROM functions
5. **H11 (Per-byte fflush on UART)** -- Severe performance degradation on all firmware

---

## Recommended Fix Priority

### Phase 1: Critical bugs affecting basic functionality
- C1: SIO GPIO writes
- C2: GPIO interrupt registers
- C4: `1 << pin` UB for pins >= 31
- C8: RP2350 function table overlap

### Phase 2: Security vulnerabilities
- C5: W5500 stack buffer overflow
- C7: ROM flash erase overflow
- H18: `flash_persist_sync` overflow
- H19: USB DPRAM buffer overflow

### Phase 3: High-priority correctness and performance
- H5: Dual-core icache/jit invalidation
- H7: Subword GPIO writes
- H11: Per-byte fflush on UART
- H16: RV32 GDB support
- H17: RV32 missing watchdog/fault/script/flush
- H20: sched_yield in corepool

### Phase 4: Medium issues
- M1-M9: Timer, GPIO, interpolator, RV membus fixes
- M10-M16: Networking protocol fixes
- M17-M24: Storage, USB, ROM, UART fixes
- M25-M30: Remaining medium issues

### Phase 5: Low issues and cleanup
- L1-L47: Performance optimizations, minor correctness, cleanup
