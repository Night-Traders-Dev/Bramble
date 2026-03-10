#include <stdio.h>
#include <string.h>
#include "nvic.h"
#include "emulator.h"
#include "corepool.h"

/* Per-core NVIC state (RP2040 has independent NVIC per core) */
nvic_state_t nvic_states[2] = {{0}};

/* Per-core SysTick state */
systick_state_t systick_states[2] = {{0}};

/* Track last IRQ signal for duplicate detection */
static uint32_t last_irq_signal = 0xFFFFFFFF;
static uint32_t irq_signal_count = 0;

/* Helper: get current core's NVIC state */
static inline nvic_state_t *nvic_cur(void) {
    return &nvic_states[get_active_core()];
}

/* Helper: get current core's SysTick state */
static inline systick_state_t *systick_cur(void) {
    return &systick_states[get_active_core()];
}

/* Initialize NVIC */
void nvic_init(void) {
    nvic_reset();
}

/* Reset NVIC to power-on defaults (both cores) */
void nvic_reset(void) {
    for (int c = 0; c < 2; c++) {
        memset(&nvic_states[c], 0, sizeof(nvic_state_t));
        nvic_states[c].enable = 0x0;
        nvic_states[c].pending = 0x0;
        nvic_states[c].active_exceptions = 0x0;
        nvic_states[c].shpr2 = 0;
        nvic_states[c].shpr3 = 0;
        nvic_states[c].pendsv_pending = 0;
        for (int i = 0; i < NUM_EXTERNAL_IRQS; i++) {
            nvic_states[c].priority[i] = 0;
        }
    }

    for (int c = 0; c < 2; c++) {
        memset(&systick_states[c], 0, sizeof(systick_state_t));
    }

    last_irq_signal = 0xFFFFFFFF;
    irq_signal_count = 0;
}

/* Initialize SysTick */
void systick_init(void) {
    systick_reset();
}

/* Reset SysTick to power-on defaults (both cores) */
void systick_reset(void) {
    for (int c = 0; c < 2; c++) {
        memset(&systick_states[c], 0, sizeof(systick_state_t));
    }
}

/* Tick SysTick timer - called once per CPU step for active core */
void systick_tick(uint32_t cycles) {
    systick_state_t *st = systick_cur();
    if (!(st->csr & 1)) return; /* Not enabled */

    for (uint32_t i = 0; i < cycles; i++) {
        if (st->cvr > 0) {
            st->cvr--;
        }
        if (st->cvr == 0) {
            /* Counter reached zero - set COUNTFLAG and reload */
            st->csr |= (1u << 16); /* COUNTFLAG */
            st->cvr = st->rvr & 0x00FFFFFF;
            /* If TICKINT (bit 1) is set, pend SysTick exception */
            if (st->csr & 2) {
                st->pending = 1;
                corepool_wake_cores();
            }
        }
    }
}

/**
 * Get effective priority of an exception vector number.
 * Uses active core's NVIC state.
 */
uint8_t nvic_get_exception_priority(uint32_t vector_num) {
    nvic_state_t *ns = nvic_cur();
    switch (vector_num) {
        case EXC_RESET:
        case EXC_NMI:
        case EXC_HARDFAULT:
            return 0; /* Fixed highest priority */
        case EXC_SVCALL:
            return (ns->shpr2 >> 24) & 0xC0;
        case EXC_PENDSV:
            return (ns->shpr3 >> 16) & 0xC0;
        case EXC_SYSTICK:
            return (ns->shpr3 >> 24) & 0xC0;
        default:
            if (vector_num >= 16 && (vector_num - 16) < NUM_EXTERNAL_IRQS) {
                return ns->priority[vector_num - 16] & 0xC0;
            }
            return 0xFF;
    }
}

/* Enable an IRQ on current core (set in ISER) */
void nvic_enable_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state_t *ns = nvic_cur();
        ns->enable |= (1 << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Core %d: Enabled IRQ %u (enable mask=0x%X)\n",
                   get_active_core(), irq, ns->enable);
    }
}

/* Disable an IRQ on current core (set in ICER) */
void nvic_disable_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state_t *ns = nvic_cur();
        ns->enable &= ~(1 << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Core %d: Disabled IRQ %u (enable mask=0x%X)\n",
                   get_active_core(), irq, ns->enable);
    }
}

/* Mark an IRQ as pending on current core (firmware ISPR write) */
void nvic_set_pending(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state_t *ns = nvic_cur();
        ns->pending |= (1 << irq);
        corepool_wake_cores();
        if (cpu.debug_enabled)
            printf("[NVIC] Core %d: Set pending IRQ %u (pending=0x%X, enable=0x%X)\n",
                   get_active_core(), irq, ns->pending, ns->enable);
    }
}

/* Clear pending bit on current core (set in ICPR) */
void nvic_clear_pending(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state_t *ns = nvic_cur();
        ns->pending &= ~(1 << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Core %d: Cleared pending IRQ %u (pending now=0x%X)\n",
                   get_active_core(), irq, ns->pending);
    }
}

/* Set priority for an IRQ on current core */
void nvic_set_priority(uint32_t irq, uint8_t priority) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_cur()->priority[irq] = priority & 0xC0;
    }
}

/**
 * Get the highest priority pending IRQ for the active core.
 */
uint32_t nvic_get_pending_irq(void) {
    nvic_state_t *ns = nvic_cur();
    uint32_t pending_and_enabled = ns->pending & ns->enable;

    if (pending_and_enabled == 0) {
        return 0xFFFFFFFF;
    }

    uint32_t highest_priority_irq = 0xFFFFFFFF;
    uint8_t highest_priority_value = 0xFF;

    for (uint32_t irq = 0; irq < NUM_EXTERNAL_IRQS; irq++) {
        if ((pending_and_enabled & (1 << irq))) {
            uint8_t prio = ns->priority[irq] & 0xC0;

            if (prio < highest_priority_value) {
                highest_priority_value = prio;
                highest_priority_irq = irq;
            } else if (prio == highest_priority_value) {
                if (irq < highest_priority_irq) {
                    highest_priority_irq = irq;
                }
            }
        }
    }

    return highest_priority_irq;
}

/* Read NVIC register (current core's view) */
uint32_t nvic_read_register(uint32_t addr) {
    nvic_state_t *ns = nvic_cur();
    systick_state_t *st = systick_cur();

    switch (addr) {
        /* SysTick registers */
        case SYST_CSR:
            {
                uint32_t val = st->csr;
                /* COUNTFLAG (bit 16) is cleared on read */
                st->csr &= ~(1u << 16);
                return val;
            }
        case SYST_RVR:
            return st->rvr & 0x00FFFFFF;
        case SYST_CVR:
            return st->cvr & 0x00FFFFFF;
        case SYST_CALIB:
            return 0xC0002710;

        /* NVIC registers */
        case NVIC_ISER:
            return ns->enable;

        case NVIC_ICER:
            return ns->enable;

        case NVIC_ISPR:
            return ns->pending;

        case NVIC_ICPR:
            return ns->pending;

        case NVIC_IABR:
            return ns->iabr;

        case NVIC_IPR:
        case NVIC_IPR + 4:
        case NVIC_IPR + 8:
        case NVIC_IPR + 12:
        case NVIC_IPR + 16:
        case NVIC_IPR + 20:
        case NVIC_IPR + 24:
        case NVIC_IPR + 28:
            {
                uint32_t offset = (addr - NVIC_IPR) / 4;
                if (offset < 8) {
                    uint32_t result = 0;
                    for (int i = 0; i < 4; i++) {
                        uint32_t irq_idx = offset * 4 + i;
                        if (irq_idx < NUM_EXTERNAL_IRQS) {
                            result |= ((uint32_t)ns->priority[irq_idx]) << (i * 8);
                        }
                    }
                    return result;
                }
            }
            return 0;

        /* SCB registers */
        case SCB_ICSR:
            {
                uint32_t pending_irq = nvic_get_pending_irq();
                uint32_t val = 0;
                if (pending_irq != 0xFFFFFFFF) {
                    val |= ((pending_irq + 16) << ICSR_VECTPENDING_SHIFT);
                    val |= ICSR_ISRPENDING;
                }
                if (st->pending)
                    val |= ICSR_PENDSTSET;
                if (ns->pendsv_pending)
                    val |= ICSR_PENDSVSET;
                return val;
            }

        case SCB_VTOR:
            return cpu.vtor;

        case SCB_BASE:  /* 0xE000ED00 - CPUID */
            return 0x410CC601;

        case SCB_AIRCR:
            return 0x05FA0000;

        case SCB_SCR:
            return 0;

        case SCB_CCR:
            return (1u << 9);

        case SCB_SHPR2:
            return ns->shpr2;

        case SCB_SHPR3:
            return ns->shpr3;

        default:
            return 0;
    }
}

/* Write NVIC register (current core's state) */
void nvic_write_register(uint32_t addr, uint32_t val) {
    nvic_state_t *ns = nvic_cur();
    systick_state_t *st = systick_cur();

    switch (addr) {
        /* SysTick registers */
        case SYST_CSR:
            st->csr = (st->csr & ~0x7) | (val & 0x7);
            if (cpu.debug_enabled) {
                printf("[SYSTICK] Core %d: CSR = 0x%08X (EN=%d TICKINT=%d)\n",
                       get_active_core(), st->csr, (int)(val & 1), (int)((val >> 1) & 1));
            }
            break;
        case SYST_RVR:
            st->rvr = val & 0x00FFFFFF;
            break;
        case SYST_CVR:
            st->cvr = 0;
            st->csr &= ~(1u << 16);
            break;

        /* NVIC registers */
        case NVIC_ISER:
            ns->enable |= val;
            if (cpu.debug_enabled)
                printf("[NVIC] Core %d: Write ISER: 0x%X, enabled mask now=0x%X\n",
                       get_active_core(), val, ns->enable);
            break;

        case NVIC_ICER:
            ns->enable &= ~val;
            if (cpu.debug_enabled)
                printf("[NVIC] Core %d: Write ICER: 0x%X, enabled mask now=0x%X\n",
                       get_active_core(), val, ns->enable);
            break;

        case NVIC_ISPR:
            ns->pending |= val;
            if (cpu.debug_enabled)
                printf("[NVIC] Core %d: Write ISPR: 0x%X, pending mask now=0x%X\n",
                       get_active_core(), val, ns->pending);
            break;

        case NVIC_ICPR:
            ns->pending &= ~val;
            if (cpu.debug_enabled)
                printf("[NVIC] Core %d: Write ICPR: 0x%X, pending mask now=0x%X\n",
                       get_active_core(), val, ns->pending);
            break;

        case NVIC_IPR:
        case NVIC_IPR + 4:
        case NVIC_IPR + 8:
        case NVIC_IPR + 12:
        case NVIC_IPR + 16:
        case NVIC_IPR + 20:
        case NVIC_IPR + 24:
        case NVIC_IPR + 28:
            {
                uint32_t offset = (addr - NVIC_IPR) / 4;
                if (offset < 8) {
                    for (int i = 0; i < 4; i++) {
                        uint32_t irq_idx = offset * 4 + i;
                        if (irq_idx < NUM_EXTERNAL_IRQS) {
                            ns->priority[irq_idx] = (val >> (i * 8)) & 0xFF;
                        }
                    }
                }
            }
            break;

        /* SCB registers */
        case SCB_ICSR:
            if (val & ICSR_PENDSVCLR)
                ns->pendsv_pending = 0;
            if (val & ICSR_PENDSTCLR)
                st->pending = 0;
            if (val & ICSR_PENDSVSET) {
                ns->pendsv_pending = 1;
                corepool_wake_cores();
            }
            if (val & ICSR_PENDSTSET) {
                st->pending = 1;
                corepool_wake_cores();
            }
            break;

        case SCB_VTOR:
            cpu.vtor = val & 0xFFFFFF80;
            break;

        case SCB_SHPR2:
            ns->shpr2 = val & 0xC0000000;
            break;

        case SCB_SHPR3:
            ns->shpr3 = val & 0xC0C00000;
            break;

        case SCB_AIRCR:
            if ((val >> 16) == 0x05FA) {
                if (val & (1u << 2)) {
                    extern int watchdog_reboot_pending;
                    watchdog_reboot_pending = 1;
                }
            }
            break;
        case SCB_SCR:
        case SCB_CCR:
            break;

        default:
            break;
    }
}

/* Called by peripherals to signal an interrupt on BOTH cores' NVICs.
 * On real RP2040, the interrupt line goes to both cores' NVICs.
 * Each core independently decides whether to handle based on its own enable mask. */
void nvic_signal_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        irq_signal_count++;

        if (cpu.debug_enabled) {
            printf("[NVIC] *** SIGNAL IRQ %u (count=%u) ***\n",
                   irq, irq_signal_count);
        }

        last_irq_signal = irq;

        /* Set pending on BOTH cores (shared interrupt line) */
        for (int c = 0; c < 2; c++) {
            nvic_states[c].pending |= (1 << irq);
        }
        corepool_wake_cores();
    }
}
