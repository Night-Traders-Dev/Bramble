#include <stdio.h>
#include <string.h>
#include "nvic.h"
#include "emulator.h"

/* Global NVIC state */
nvic_state_t nvic_state = {0};

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
}

/* Enable an IRQ (set in ISER) */
void nvic_enable_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.enable |= (1 << irq);
        if (cpu.debug_enabled) {
            printf("[NVIC] Enabled IRQ %u\n", irq);
        }
    }
}

/* Disable an IRQ (set in ICER) */
void nvic_disable_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.enable &= ~(1 << irq);
        if (cpu.debug_enabled) {
            printf("[NVIC] Disabled IRQ %u\n", irq);
        }
    }
}

/* Mark an IRQ as pending (set in ISPR) */
void nvic_set_pending(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.pending |= (1 << irq);
        if (cpu.debug_enabled) {
            printf("[NVIC] Set pending for IRQ %u (pending mask=0x%X)\n", irq, nvic_state.pending);
        }
    }
}

/* Clear pending bit (set in ICPR) */
void nvic_clear_pending(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.pending &= ~(1 << irq);
        if (cpu.debug_enabled) {
            printf("[NVIC] Cleared pending for IRQ %u (pending mask=0x%X)\n", irq, nvic_state.pending);
        }
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

/* Get the highest priority pending IRQ
 * Returns the IRQ number (0-25) if pending and enabled
 * Returns 0xFFFFFFFF if no pending IRQ
 */
uint32_t nvic_get_pending_irq(void) {
    /* Check which IRQs are both pending AND enabled */
    uint32_t pending_and_enabled = nvic_state.pending & nvic_state.enable;
    
    if (pending_and_enabled == 0) {
        /* No pending IRQs */
        return 0xFFFFFFFF;
    }
    
    /* Find the highest priority (lowest IRQ number) pending IRQ */
    for (uint32_t irq = 0; irq < NUM_EXTERNAL_IRQS; irq++) {
        if (pending_and_enabled & (1 << irq)) {
            if (cpu.debug_enabled) {
                printf("[NVIC] Get pending IRQ: Found IRQ %u (pending=0x%X, enabled=0x%X)\n", 
                       irq, nvic_state.pending, nvic_state.enable);
            }
            return irq;
        }
    }
    
    return 0xFFFFFFFF;
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
            if (cpu.debug_enabled && val) {
                printf("[NVIC] Write ISER: 0x%X, enabled mask now=0x%X\n", val, nvic_state.enable);
            }
            break;
            
        case NVIC_ICER:
            /* Interrupt Clear Enable Register - write 1 to disable */
            nvic_state.enable &= ~val;
            if (cpu.debug_enabled && val) {
                printf("[NVIC] Write ICER: 0x%X\n", val);
            }
            break;
            
        case NVIC_ISPR:
            /* Interrupt Set Pending Register - write 1 to set pending */
            nvic_state.pending |= val;
            if (cpu.debug_enabled && val) {
                printf("[NVIC] Write ISPR: 0x%X, pending mask now=0x%X\n", val, nvic_state.pending);
            }
            break;
            
        case NVIC_ICPR:
            /* Interrupt Clear Pending Register - write 1 to clear pending */
            nvic_state.pending &= ~val;
            if (cpu.debug_enabled && val) {
                printf("[NVIC] Write ICPR: 0x%X, pending mask now=0x%X\n", val, nvic_state.pending);
            }
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
        if (cpu.debug_enabled) {
            printf("[NVIC] Signal IRQ %u\n", irq);
        }
        /* Just mark it as pending - the CPU will check on next cycle */
        nvic_set_pending(irq);
    }
}
