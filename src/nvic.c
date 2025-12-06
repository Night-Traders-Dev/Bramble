#include <stdio.h>
#include <string.h>
#include "nvic.h"
#include "emulator.h"

/* Global NVIC state */
nvic_state_t nvic_state = {0};

/* Track last IRQ signal for duplicate detection */
static uint32_t last_irq_signal = 0xFFFFFFFF;
static uint32_t irq_signal_count = 0;

/* Initialize NVIC */
void nvic_init(void) {
    nvic_reset();
}

/* Reset NVIC to power-on defaults */
void nvic_reset(void) {
    memset(&nvic_state, 0, sizeof(nvic_state_t));
    nvic_state.enable = 0x0;              /* All IRQs disabled */
    nvic_state.pending = 0x0;             /* No pending interrupts */
    nvic_state.active_exceptions = 0x0;   /* No active exceptions */
    
    /* Initialize priority registers */
    for (int i = 0; i < NUM_EXTERNAL_IRQS; i++) {
        nvic_state.priority[i] = 0;
    }
    
    last_irq_signal = 0xFFFFFFFF;
    irq_signal_count = 0;
}

/* Enable an IRQ (set in ISER) */
void nvic_enable_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.enable |= (1 << irq);
        printf("[NVIC] Enabled IRQ %u (enable mask=0x%X)\n", irq, nvic_state.enable);
    }
}

/* Disable an IRQ (set in ICER) */
void nvic_disable_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.enable &= ~(1 << irq);
        printf("[NVIC] Disabled IRQ %u (enable mask=0x%X)\n", irq, nvic_state.enable);
    }
}

/* Mark an IRQ as pending (set in ISPR) */
void nvic_set_pending(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.pending |= (1 << irq);
        printf("[NVIC] Set pending for IRQ %u (pending mask=0x%X, enable mask=0x%X)\n", 
               irq, nvic_state.pending, nvic_state.enable);
    }
}

/* Clear pending bit (set in ICPR) */
void nvic_clear_pending(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.pending &= ~(1 << irq);
        printf("[NVIC] Cleared pending for IRQ %u (pending mask now=0x%X)\n", irq, nvic_state.pending);
    }
}

/* Set priority for an IRQ */
void nvic_set_priority(uint32_t irq, uint8_t priority) {
    if (irq < NUM_EXTERNAL_IRQS) {
        /* Cortex-M0+ uses only bits [7:6] of priority register */
        /* Allowed values: 0, 64 (0x40), 128 (0x80), 192 (0xC0) */
        nvic_state.priority[irq] = priority & 0xC0;
    }
}

/**
 * Get the highest priority pending IRQ.
 * 
 * Returns the IRQ (0-25) with:
 *   - Pending bit set (nvic_state.pending)
 *   - Enable bit set (nvic_state.enable)  
 *   - LOWEST numeric priority value (bits [7:6] of priority register)
 * 
 * In ARM Cortex-M0+: LOWER numeric priority value = HIGHER importance
 * Priority levels: 0x00 (level 0, highest), 0x40 (level 1), 0x80 (level 2), 0xC0 (level 3, lowest)
 * 
 * Tiebreaker: If multiple IRQs have same priority, returns the one with lower IRQ number
 * 
 * Returns 0xFFFFFFFF if no pending IRQ
 */
uint32_t nvic_get_pending_irq(void) {
    uint32_t pending_and_enabled = nvic_state.pending & nvic_state.enable;
    
    if (pending_and_enabled == 0) {
        /* No pending interrupts */
        return 0xFFFFFFFF;
    }
    
    uint32_t highest_priority_irq = 0xFFFFFFFF;
    uint8_t highest_priority_value = 0xFF;  /* Start with worst possible priority */
    
    /* Scan all IRQs to find the one with lowest (best) numeric priority value */
    for (uint32_t irq = 0; irq < NUM_EXTERNAL_IRQS; irq++) {
        if ((pending_and_enabled & (1 << irq))) {
            /* Extract priority (only bits [7:6] used in Cortex-M0+) */
            uint8_t prio = nvic_state.priority[irq] & 0xC0;
            
            /* Update if this IRQ has LOWER (better) numeric priority value */
            if (prio < highest_priority_value) {
                highest_priority_value = prio;
                highest_priority_irq = irq;
            }
            /* Tiebreaker: same priority -> prefer lower IRQ number */
            else if (prio == highest_priority_value) {
                if (irq < highest_priority_irq) {
                    highest_priority_irq = irq;
                }
            }
        }
    }
    
    return highest_priority_irq;
}

/* Read NVIC register */
uint32_t nvic_read_register(uint32_t addr) {
    switch (addr) {
        case NVIC_ISER:
            return nvic_state.enable;
            
        case NVIC_ICER:
            return nvic_state.enable;  /* ICER reads back what's enabled */
            
        case NVIC_ISPR:
            return nvic_state.pending;
            
        case NVIC_ICPR:
            return nvic_state.pending;  /* ICPR reads back what's pending */
            
        case NVIC_IPR:
        case NVIC_IPR + 4:
        case NVIC_IPR + 8:
        case NVIC_IPR + 12:
        case NVIC_IPR + 16:
        case NVIC_IPR + 20:
        case NVIC_IPR + 24:
            /* Priority registers - 8 bytes for 32 IRQs (4 bytes per register) */
            {
                uint32_t offset = (addr - NVIC_IPR) / 4;
                if (offset < 7) {
                    uint32_t result = 0;
                    for (int i = 0; i < 4; i++) {
                        uint32_t irq_idx = offset * 4 + i;
                        if (irq_idx < NUM_EXTERNAL_IRQS) {
                            result |= ((uint32_t)nvic_state.priority[irq_idx]) << (i * 8);
                        }
                    }
                    return result;
                }
            }
            return 0;
            
        case SCB_ICSR:
            /* Interrupt Control and State Register */
            {
                uint32_t pending_irq = nvic_get_pending_irq();
                uint32_t vect_pending = (pending_irq != 0xFFFFFFFF) ? (pending_irq + 16) : 0;
                uint32_t isrpending = (pending_irq != 0xFFFFFFFF) ? (1 << 22) : 0;
                return vect_pending | isrpending;
            }
            
        case SCB_VTOR:
            /* Vector Table Offset Register */
            return 0x10000000;  /* Start of flash */
            
        default:
            return 0;
    }
}

/* Write NVIC register */
void nvic_write_register(uint32_t addr, uint32_t val) {
    switch (addr) {
        case NVIC_ISER:
            /* Interrupt Set Enable Register - write 1 to enable */
            nvic_state.enable |= val;
            printf("[NVIC] Write ISER: 0x%X, enabled mask now=0x%X\n", val, nvic_state.enable);
            break;
            
        case NVIC_ICER:
            /* Interrupt Clear Enable Register - write 1 to disable */
            nvic_state.enable &= ~val;
            printf("[NVIC] Write ICER: 0x%X, enabled mask now=0x%X\n", val, nvic_state.enable);
            break;
            
        case NVIC_ISPR:
            /* Interrupt Set Pending Register - write 1 to set pending */
            nvic_state.pending |= val;
            printf("[NVIC] Write ISPR: 0x%X, pending mask now=0x%X\n", val, nvic_state.pending);
            break;
            
        case NVIC_ICPR:
            /* Interrupt Clear Pending Register - write 1 to clear pending */
            nvic_state.pending &= ~val;
            printf("[NVIC] Write ICPR: 0x%X, pending mask now=0x%X\n", val, nvic_state.pending);
            break;
            
        case NVIC_IPR:
        case NVIC_IPR + 4:
        case NVIC_IPR + 8:
        case NVIC_IPR + 12:
        case NVIC_IPR + 16:
        case NVIC_IPR + 20:
        case NVIC_IPR + 24:
            /* Priority registers */
            {
                uint32_t offset = (addr - NVIC_IPR) / 4;
                if (offset < 7) {
                    for (int i = 0; i < 4; i++) {
                        uint32_t irq_idx = offset * 4 + i;
                        if (irq_idx < NUM_EXTERNAL_IRQS) {
                            nvic_state.priority[irq_idx] = (val >> (i * 8)) & 0xFF;
                        }
                    }
                }
            }
            break;
            
        case SCB_VTOR:
            /* Vector Table Offset Register - read-only in our implementation */
            break;
            
        default:
            break;
    }
}

/* Called by peripherals (like timer) to signal that an interrupt occurred */
void nvic_signal_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        irq_signal_count++;
        
        printf("[NVIC] *** SIGNAL IRQ %u (count=%u, pending before=0x%X, enable=0x%X) ***\n", 
               irq, irq_signal_count, nvic_state.pending, nvic_state.enable);
        
        /* Check if this is a duplicate signal (same IRQ, still pending) */
        if (last_irq_signal == irq && (nvic_state.pending & (1 << irq))) {
            printf("[NVIC] WARNING: Duplicate signal for IRQ %u - still pending!\n", irq);
        }
        
        last_irq_signal = irq;
        
        /* Just mark it as pending - the CPU will check on next cycle */
        nvic_set_pending(irq);
    }
}
