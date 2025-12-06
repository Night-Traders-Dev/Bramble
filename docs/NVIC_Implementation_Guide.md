# NVIC Implementation Guide

**Last Updated:** December 6, 2025  
**Status:** In Progress - 3 issues identified  
**Completeness:** 70% (Core working, MMIO and return path missing)

## Quick Reference

### What Works ✅

- NVIC state machine (enable/disable/pending)
- CPU interrupt detection and exception entry
- Timer integration pattern
- Register read/write logic (but not mapped in membus)
- Priority register storage (but not used for scheduling)
- Debug logging

### What's Missing ❌

1. **MMIO Routing** (Critical)
   - Firmware can't access NVIC via LDR/STR to 0xE000E000
   - Location: `src/membus.c`
   - Fix: ~15 lines
   - Time: 30 min

2. **Priority Scheduling** (Important)
   - Returns lowest IRQ number, not highest priority
   - Location: `src/nvic.c`, `nvic_get_pending_irq()`
   - Fix: ~30 lines
   - Time: 1 hour

3. **Exception Return** (Important)
   - ISR can't return to caller via BX LR magic value
   - Location: `src/cpu.c`, `instr_bx()`
   - Fix: ~50 lines
   - Time: 1.5 hours

---

## Issue #1: Add NVIC to Memory Bus

### File Location
`src/membus.c`

### Current Problem

```c
struct {
    uint32_t *iser = (uint32_t *)0xE000E100;  // NVIC_ISER
    *iser = 0x01;  // Enable IRQ 0
}
// Result: No-op. nvic_state.enable NOT updated.
```

### Solution

Add to `mem_write32()` function:

```c
void mem_write32(uint32_t addr, uint32_t val) {
    // ... existing cases (FLASH, RAM, GPIO, TIMER, UART) ...
    
    /* NVIC Memory-Mapped I/O (0xE000E000 - 0xE000EFFF) */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        nvic_write_register(addr, val);
        return;
    }
    
    // ... rest of function ...
}
```

Add to `mem_read32()` function:

```c
uint32_t mem_read32(uint32_t addr) {
    // ... existing cases ...
    
    /* NVIC Memory-Mapped I/O */
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        return nvic_read_register(addr);
    }
    
    // ... rest of function ...
}
```

Also handle 16-bit and 8-bit accesses:

```c
void mem_write16(uint32_t addr, uint16_t val) {
    // ... existing ...
    if (addr >= NVIC_BASE && addr < NVIC_BASE + 0x1000) {
        uint32_t addr32 = addr & ~0x3;
        uint32_t current = nvic_read_register(addr32);
        uint8_t offset = addr & 0x3;
        uint32_t mask = 0xFFFF << (offset * 8);
        uint32_t new_val = (current & ~mask) | ((uint32_t)val << (offset * 8));
        nvic_write_register(addr32, new_val);
        return;
    }
    // ...
}
```

### Required Header

Add to top of `membus.c`:
```c
#include "nvic.h"  /* For NVIC_BASE */
```

### Verification

```c
void test_nvic_mmio() {
    // Enable IRQ 0 via MMIO
    uint32_t *iser = (uint32_t *)0xE000E100;
    *iser = 0x01;
    
    // Verify internal state
    assert(nvic_state.enable & 0x01);
    
    // Read back
    uint32_t val = *iser;
    assert(val & 0x01);
}
```

---

## Issue #2: Implement Priority Scheduling

### File Location
`src/nvic.c`, function `nvic_get_pending_irq()`

### Current Problem

```
Scenario:
  IRQ 3: pending, priority=0xC0 (level 3, lowest)
  IRQ 8: pending, priority=0x00 (level 0, highest)

Current code: Returns IRQ 3 (lowest number)
Should return: IRQ 8 (lower priority value = higher priority)
```

### ARM Cortex-M0+ Priority System

```
Priority bits [7:6]  →  Numeric value  →  Importance
      00              →     0x00        →  ⭐⭐⭐⭐ Highest
      01              →     0x40        →  ⭐⭐⭐ High
      10              →     0x80        →  ⭐⭐ Low
      11              →     0xC0        →  ⭐ Lowest

Key: LOWER numeric = HIGHER priority (counter-intuitive but standard)
```

### Solution

Replace `nvic_get_pending_irq()` with:

```c
/**
 * Get the highest priority pending IRQ.
 * 
 * Returns the IRQ (0-25) with both:
 *   - Pending bit set (nvic_state.pending)
 *   - Enable bit set (nvic_state.enable)
 *   - LOWEST numeric priority value (bits [7:6])
 * 
 * Tiebreaker: Same priority → return lower IRQ number
 * Returns 0xFFFFFFFF if no pending IRQ
 */
uint32_t nvic_get_pending_irq(void) {
    uint32_t pending_and_enabled = nvic_state.pending & nvic_state.enable;
    
    if (pending_and_enabled == 0) {
        return 0xFFFFFFFF;  /* No pending interrupts */
    }
    
    uint32_t highest_priority_irq = 0xFFFFFFFF;
    uint8_t highest_priority_value = 0xFF;  /* Start with worst priority (0xFF > any valid) */
    
    /* Scan all IRQs to find the best one */
    for (uint32_t irq = 0; irq < NUM_EXTERNAL_IRQS; irq++) {
        if ((pending_and_enabled & (1 << irq))) {
            /* Extract priority (only bits [7:6] used in Cortex-M0+) */
            uint8_t prio = nvic_state.priority[irq] & 0xC0;
            
            /* Update if this IRQ has LOWER (better) priority value */
            if (prio < highest_priority_value) {
                highest_priority_value = prio;
                highest_priority_irq = irq;
            }
            /* Tiebreaker: same priority → prefer lower IRQ number */
            else if (prio == highest_priority_value) {
                if (irq < highest_priority_irq) {
                    highest_priority_irq = irq;
                }
            }
        }
    }
    
    if (cpu.debug_enabled && highest_priority_irq != 0xFFFFFFFF) {
        printf("[NVIC] Selected IRQ %u with priority 0x%02X\n",
               highest_priority_irq, highest_priority_value);
    }
    
    return highest_priority_irq;
}
```

### Test Cases

```c
void test_priority_scheduling() {
    /* Test 1: Higher IRQ number with better priority wins */
    nvic_enable_irq(3);
    nvic_enable_irq(8);
    nvic_set_priority(3, 0xC0);  /* Priority 3 (worst) */
    nvic_set_priority(8, 0x00);  /* Priority 0 (best) */
    nvic_signal_irq(3);
    nvic_signal_irq(8);
    assert(nvic_get_pending_irq() == 8);  /* Should pick 8 */
    
    /* Test 2: Same priority uses tiebreaker */
    nvic_clear_pending(3);
    nvic_clear_pending(8);
    nvic_set_priority(3, 0x40);
    nvic_set_priority(5, 0x40);  /* Same priority */
    nvic_signal_irq(3);
    nvic_signal_irq(5);
    assert(nvic_get_pending_irq() == 3);  /* Lower IRQ number wins */
    
    /* Test 3: Disabled IRQ ignored */
    nvic_clear_pending(3);
    nvic_clear_pending(5);
    nvic_disable_irq(5);
    nvic_signal_irq(5);
    nvic_signal_irq(3);
    assert(nvic_get_pending_irq() == 3);  /* IRQ 5 disabled, so skip it */
}
```

---

## Issue #3: Implement Exception Return

### File Locations
- `src/cpu.c`: Add `cpu_exception_return()` function
- `src/instructions.c`: Modify `instr_bx()` to detect magic return values

### Background: Magic Return Addresses

ARM uses special LR values to indicate how to return from exception:

```c
When entering exception:
    cpu.r[14] = 0xFFFFFFF9;  /* Magic: return to thread mode */
    cpu.r[15] = handler;      /* Jump to ISR */

When exiting exception:
    bx lr              /* If LR = 0xFFFFFFF9, special handling */
                       /* Should pop stack frame and restore context */
```

### Solution

**In `src/cpu.c`, add function:**

```c
/**
 * Handle exception return via BX LR with magic values.
 * 
 * Magic LR values (bits [3:0] define return mode):
 *   0xFFFFFFF1 = Return to Handler mode (nested interrupt)
 *   0xFFFFFFF9 = Return to Thread mode (normal)
 *   0xFFFFFFFD = Return to Thread mode with FPU context
 * 
 * For Cortex-M0+ (no FPU), only 0xFFFFFFF9 is used.
 */
void cpu_exception_return(uint32_t lr_value) {
    uint32_t return_mode = lr_value & 0x0F;
    
    if (cpu.debug_enabled) {
        printf("[CPU] Exception return: LR=0x%08X mode=%d\n", lr_value, return_mode);
    }
    
    if (return_mode == 0x9) {  /* 0xFFFFFFF9 = Return to Thread */
        /* Pop 8-register context frame from stack */
        uint32_t sp = cpu.r[13];
        
        uint32_t xpsr = mem_read32(sp);     sp += 4;
        uint32_t pc   = mem_read32(sp);     sp += 4;
        uint32_t lr   = mem_read32(sp);     sp += 4;
        uint32_t r12  = mem_read32(sp);     sp += 4;
        uint32_t r3   = mem_read32(sp);     sp += 4;
        uint32_t r2   = mem_read32(sp);     sp += 4;
        uint32_t r1   = mem_read32(sp);     sp += 4;
        uint32_t r0   = mem_read32(sp);     sp += 4;
        
        /* Restore registers */
        cpu.r[0]  = r0;
        cpu.r[1]  = r1;
        cpu.r[2]  = r2;
        cpu.r[3]  = r3;
        cpu.r[12] = r12;
        cpu.r[13] = sp;         /* Updated SP */
        cpu.r[14] = lr;         /* Restore LR */
        cpu.r[15] = pc & ~1;    /* Restore PC, clear Thumb bit */
        cpu.xpsr  = xpsr;       /* Restore condition flags */
        
        /* Clear active exception bit */
        if (cpu.current_irq != 0xFFFFFFFF) {
            uint32_t vector_num = cpu.current_irq + 16;
            nvic_state.active_exceptions &= ~(1 << vector_num);
            cpu.current_irq = 0xFFFFFFFF;
        }
        
        if (cpu.debug_enabled) {
            printf("[CPU] Exception return complete: PC=0x%08X SP=0x%08X\n",
                   cpu.r[15], cpu.r[13]);
        }
    }
}
```

**In `src/instructions.c`, modify `instr_bx()`:**

```c
void instr_bx(uint16_t instr) {
    uint32_t rm = (instr >> 3) & 0x0F;
    uint32_t target = cpu.r[rm];
    
    if (cpu.debug_enabled) {
        printf("[CPU] BX R%u (=0x%08X)\n", rm, target);
    }
    
    /* Check for exception return magic addresses */
    if ((target & 0xFFFFFFF0) == 0xFFFFFFF0) {
        cpu_exception_return(target);
        return;
    }
    
    /* Normal BX: branch to address in register */
    cpu.r[15] = target & ~1;  /* Set PC, clear Thumb bit */
}
```

### Verification

```c
void test_exception_return() {
    /* Setup ISR context on stack */
    uint32_t *sp = (uint32_t *)0x20000000;  /* Start of RAM */
    
    /* Push return frame (in reverse order of pop) */
    *sp++ = 0x12345678;  /* xPSR */
    *sp++ = 0x10000010;  /* PC - return address */
    *sp++ = 0x00000000;  /* LR */
    *sp++ = 0xDEADBEEF;  /* R12 */
    *sp++ = 0x03030303;  /* R3 */
    *sp++ = 0x02020202;  /* R2 */
    *sp++ = 0x01010101;  /* R1 */
    *sp++ = 0x00000000;  /* R0 */
    
    /* Set up CPU state */
    cpu.r[13] = (uint32_t)sp;  /* SP = end of frame */
    cpu.r[14] = 0xFFFFFFF9;    /* Magic return address */
    
    /* Execute BX LR */
    instr_bx(0x4770);  /* BX LR instruction */
    
    /* Verify restoration */
    assert(cpu.r[15] == 0x10000010);  /* PC restored */
    assert(cpu.r[0] == 0x00000000);   /* R0 restored */
    assert(cpu.r[1] == 0x01010101);   /* R1 restored */
    assert(cpu.r[12] == 0xDEADBEEF);  /* R12 restored */
}
```

---

## Debugging NVIC Issues

### Enable Debug Output

```c
cpu.debug_enabled = 1;  /* In main or test */
```

### Expected Debug Output

**When enabling IRQ via MMIO:**
```
[NVIC] Write ISER: 0x01, enabled mask now=0x01
```

**When interrupt fires:**
```
[TIMER] Alarm 0 FIRED at 1000 us
[NVIC] Signal IRQ 0
[NVIC] Set pending for IRQ 0 (pending mask=0x01)
[CPU] *** INTERRUPT: IRQ 0 detected ***
[CPU] Exception 16: PC=0x10000000 -> Handler=0x10000010
```

**When entering ISR:**
```
[CPU] Step   1: PC=0x10000010 instr=0xE7FE
```

**When exiting ISR (with fix #3):**
```
[CPU] BX R14 (=0xFFFFFFF9)
[CPU] Exception return: LR=0xFFFFFFF9 mode=9
[CPU] Exception return complete: PC=0x10000000 SP=0x20000100
```

---

## Testing Checklist

- [ ] **After Fix #1:** Firmware can enable/disable interrupts via ISER/ICER
- [ ] **After Fix #2:** High-priority interrupts interrupt low-priority ones
- [ ] **After Fix #3:** ISRs can return cleanly to interrupted code
- [ ] **Regression:** Timer interrupts still work
- [ ] **Regression:** Debug output still correct
- [ ] **Integration:** All three fixes work together

---

## References

### ARM Documentation
- ARM Cortex-M0+ Programmer's Manual (Section 4: Interrupt Model)
- ARMv6-M Architecture Reference Manual

### RP2040
- Raspberry Pi Pico Datasheet Chapter 2: System Architecture

### Related Code
- `include/nvic.h` - NVIC register definitions
- `src/timer.c` - Example of peripheral→NVIC integration
- `src/cpu.c` - CPU exception entry (already correct)

---

## Summary

The three fixes above are **isolated, testable changes** that complete the NVIC implementation:

1. **Fix #1:** 30 min, 15 lines - Unblock MMIO access
2. **Fix #2:** 1 hour, 30 lines - Implement correct priority logic
3. **Fix #3:** 1.5 hours, 50 lines - Enable ISR return

Total: ~3 hours, ~95 lines, production-ready interrupt system.
