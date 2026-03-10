#include <stdio.h>
#include <string.h>
#include "nvic.h"
#include "emulator.h"
#include "corepool.h"

/* Global NVIC state */
nvic_state_t nvic_state = {0};

/* SysTick state */
systick_state_t systick_state = {0};

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
    nvic_state.enable = 0x0;
    nvic_state.pending = 0x0;
    nvic_state.active_exceptions = 0x0;
    nvic_state.shpr2 = 0;
    nvic_state.shpr3 = 0;
    nvic_state.pendsv_pending = 0;

    for (int i = 0; i < NUM_EXTERNAL_IRQS; i++) {
        nvic_state.priority[i] = 0;
    }

    last_irq_signal = 0xFFFFFFFF;
    irq_signal_count = 0;
}

/* Initialize SysTick */
void systick_init(void) {
    systick_reset();
}

/* Reset SysTick to power-on defaults */
void systick_reset(void) {
    memset(&systick_state, 0, sizeof(systick_state_t));
}

/* Tick SysTick timer - called once per CPU step */
void systick_tick(uint32_t cycles) {
    if (!(systick_state.csr & 1)) return; /* Not enabled */

    for (uint32_t i = 0; i < cycles; i++) {
        if (systick_state.cvr > 0) {
            systick_state.cvr--;
        }
        if (systick_state.cvr == 0) {
            /* Counter reached zero - set COUNTFLAG and reload */
            systick_state.csr |= (1u << 16); /* COUNTFLAG */
            systick_state.cvr = systick_state.rvr & 0x00FFFFFF;
            /* If TICKINT (bit 1) is set, pend SysTick exception */
            if (systick_state.csr & 2) {
                systick_state.pending = 1;
                corepool_wake_cores();
            }
        }
    }
}

/**
 * Get effective priority of an exception vector number.
 * Returns priority in bits [7:6] format (0x00=highest, 0xC0=lowest).
 * Fixed-priority exceptions return 0 (they cannot be preempted by
 * configurable-priority exceptions anyway).
 */
uint8_t nvic_get_exception_priority(uint32_t vector_num) {
    switch (vector_num) {
        case EXC_RESET:
        case EXC_NMI:
        case EXC_HARDFAULT:
            return 0; /* Fixed highest priority */
        case EXC_SVCALL:
            return (nvic_state.shpr2 >> 24) & 0xC0;
        case EXC_PENDSV:
            return (nvic_state.shpr3 >> 16) & 0xC0;
        case EXC_SYSTICK:
            return (nvic_state.shpr3 >> 24) & 0xC0;
        default:
            if (vector_num >= 16 && (vector_num - 16) < NUM_EXTERNAL_IRQS) {
                return nvic_state.priority[vector_num - 16] & 0xC0;
            }
            return 0xFF;
    }
}

/* Enable an IRQ (set in ISER) */
void nvic_enable_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.enable |= (1 << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Enabled IRQ %u (enable mask=0x%X)\n", irq, nvic_state.enable);
    }
}

/* Disable an IRQ (set in ICER) */
void nvic_disable_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.enable &= ~(1 << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Disabled IRQ %u (enable mask=0x%X)\n", irq, nvic_state.enable);
    }
}

/* Mark an IRQ as pending (set in ISPR) */
void nvic_set_pending(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.pending |= (1 << irq);
        corepool_wake_cores();  /* Wake any WFI-sleeping host threads */
        if (cpu.debug_enabled)
            printf("[NVIC] Set pending for IRQ %u (pending mask=0x%X, enable mask=0x%X)\n",
                   irq, nvic_state.pending, nvic_state.enable);
    }
}

/* Clear pending bit (set in ICPR) */
void nvic_clear_pending(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.pending &= ~(1 << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Cleared pending for IRQ %u (pending mask now=0x%X)\n", irq, nvic_state.pending);
    }
}

/* Set priority for an IRQ */
void nvic_set_priority(uint32_t irq, uint8_t priority) {
    if (irq < NUM_EXTERNAL_IRQS) {
        nvic_state.priority[irq] = priority & 0xC0;
    }
}

/**
 * Get the highest priority pending IRQ.
 * Returns 0xFFFFFFFF if no pending IRQ.
 */
uint32_t nvic_get_pending_irq(void) {
    uint32_t pending_and_enabled = nvic_state.pending & nvic_state.enable;

    if (pending_and_enabled == 0) {
        return 0xFFFFFFFF;
    }

    uint32_t highest_priority_irq = 0xFFFFFFFF;
    uint8_t highest_priority_value = 0xFF;

    for (uint32_t irq = 0; irq < NUM_EXTERNAL_IRQS; irq++) {
        if ((pending_and_enabled & (1 << irq))) {
            uint8_t prio = nvic_state.priority[irq] & 0xC0;

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

/* Read NVIC register */
uint32_t nvic_read_register(uint32_t addr) {
    switch (addr) {
        /* SysTick registers */
        case SYST_CSR:
            {
                uint32_t val = systick_state.csr;
                /* COUNTFLAG (bit 16) is cleared on read */
                systick_state.csr &= ~(1u << 16);
                return val;
            }
        case SYST_RVR:
            return systick_state.rvr & 0x00FFFFFF;
        case SYST_CVR:
            return systick_state.cvr & 0x00FFFFFF;
        case SYST_CALIB:
            /* NOREF=1 (bit 31: no external ref), SKEW=1 (bit 30),
             * TENMS=10000 (10ms at 1 cycle/µs emulator model) */
            return 0xC0002710;

        /* NVIC registers */
        case NVIC_ISER:
            return nvic_state.enable;

        case NVIC_ICER:
            return nvic_state.enable;

        case NVIC_ISPR:
            return nvic_state.pending;

        case NVIC_ICPR:
            return nvic_state.pending;

        case NVIC_IABR:
            return nvic_state.iabr;

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
                            result |= ((uint32_t)nvic_state.priority[irq_idx]) << (i * 8);
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
                if (systick_state.pending)
                    val |= ICSR_PENDSTSET;
                if (nvic_state.pendsv_pending)
                    val |= ICSR_PENDSVSET;
                return val;
            }

        case SCB_VTOR:
            return cpu.vtor;

        /* CPUID register: Cortex-M0+ identifier */
        case SCB_BASE:  /* 0xE000ED00 */
            /* Implementer=ARM(0x41), Variant=0, Architecture=0xC(M0+),
             * PartNo=0xC60(Cortex-M0+), Revision=1 */
            return 0x410CC601;

        case SCB_AIRCR:
            return 0x05FA0000; /* VECTKEY + default PRIGROUP */

        case SCB_SCR:
            return 0;

        case SCB_CCR:
            /* STKALIGN=1 (bit 9) - 8-byte stack alignment on exception */
            return (1u << 9);

        case SCB_SHPR2:
            return nvic_state.shpr2;

        case SCB_SHPR3:
            return nvic_state.shpr3;

        default:
            return 0;
    }
}

/* Write NVIC register */
void nvic_write_register(uint32_t addr, uint32_t val) {
    switch (addr) {
        /* SysTick registers */
        case SYST_CSR:
            /* Only bits [2:0] are writable: CLKSOURCE, TICKINT, ENABLE */
            systick_state.csr = (systick_state.csr & ~0x7) | (val & 0x7);
            if (cpu.debug_enabled) {
                printf("[SYSTICK] CSR = 0x%08X (EN=%d TICKINT=%d)\n",
                       systick_state.csr, (int)(val & 1), (int)((val >> 1) & 1));
            }
            break;
        case SYST_RVR:
            systick_state.rvr = val & 0x00FFFFFF;
            break;
        case SYST_CVR:
            /* Writing any value clears CVR and COUNTFLAG */
            systick_state.cvr = 0;
            systick_state.csr &= ~(1u << 16);
            break;

        /* NVIC registers */
        case NVIC_ISER:
            nvic_state.enable |= val;
            if (cpu.debug_enabled)
                printf("[NVIC] Write ISER: 0x%X, enabled mask now=0x%X\n", val, nvic_state.enable);
            break;

        case NVIC_ICER:
            nvic_state.enable &= ~val;
            if (cpu.debug_enabled)
                printf("[NVIC] Write ICER: 0x%X, enabled mask now=0x%X\n", val, nvic_state.enable);
            break;

        case NVIC_ISPR:
            nvic_state.pending |= val;
            if (cpu.debug_enabled)
                printf("[NVIC] Write ISPR: 0x%X, pending mask now=0x%X\n", val, nvic_state.pending);
            break;

        case NVIC_ICPR:
            nvic_state.pending &= ~val;
            if (cpu.debug_enabled)
                printf("[NVIC] Write ICPR: 0x%X, pending mask now=0x%X\n", val, nvic_state.pending);
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
                            nvic_state.priority[irq_idx] = (val >> (i * 8)) & 0xFF;
                        }
                    }
                }
            }
            break;

        /* SCB registers */
        case SCB_ICSR:
            /* Write-1-to-clear for PENDSVCLR and PENDSTCLR */
            if (val & ICSR_PENDSVCLR)
                nvic_state.pendsv_pending = 0;
            if (val & ICSR_PENDSTCLR)
                systick_state.pending = 0;
            /* Write-1-to-set for PENDSVSET and PENDSTSET */
            if (val & ICSR_PENDSVSET) {
                nvic_state.pendsv_pending = 1;
                corepool_wake_cores();
            }
            if (val & ICSR_PENDSTSET) {
                systick_state.pending = 1;
                corepool_wake_cores();
            }
            break;

        case SCB_VTOR:
            cpu.vtor = val & 0xFFFFFF80; /* 128-byte aligned */
            break;

        case SCB_SHPR2:
            /* Only bits [31:30] writable (SVCall priority) on M0+ */
            nvic_state.shpr2 = val & 0xC0000000;
            break;

        case SCB_SHPR3:
            /* Byte 3 (bits [31:24]) = SysTick priority, Byte 2 (bits [23:16]) = PendSV priority */
            nvic_state.shpr3 = val & 0xC0C00000;
            break;

        case SCB_AIRCR:
            /* VECTKEY must be 0x05FA for write to take effect */
            if ((val >> 16) == 0x05FA) {
                if (val & (1u << 2)) {
                    /* SYSRESETREQ: request system reset */
                    extern int watchdog_reboot_pending;
                    watchdog_reboot_pending = 1;
                }
            }
            break;
        case SCB_SCR:
        case SCB_CCR:
            /* Accept writes silently */
            break;

        default:
            break;
    }
}

/* Called by peripherals (like timer) to signal that an interrupt occurred */
void nvic_signal_irq(uint32_t irq) {
    if (irq < NUM_EXTERNAL_IRQS) {
        irq_signal_count++;

        if (cpu.debug_enabled) {
            printf("[NVIC] *** SIGNAL IRQ %u (count=%u, pending before=0x%X, enable=0x%X) ***\n",
                   irq, irq_signal_count, nvic_state.pending, nvic_state.enable);

            if (last_irq_signal == irq && (nvic_state.pending & (1 << irq))) {
                printf("[NVIC] WARNING: Duplicate signal for IRQ %u - still pending!\n", irq);
            }
        }

        last_irq_signal = irq;
        nvic_set_pending(irq);
    }
}
