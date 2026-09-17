#include <stdio.h>
#include <string.h>
#include "nvic.h"
#include "emulator.h"
#include "corepool.h"
#include "devtools.h"

uint32_t nvic_num_external_irqs(void) {
    return membus_rp2350_mode ? NUM_EXTERNAL_IRQS_RP2350 : NUM_EXTERNAL_IRQS;
}

static inline uint64_t nvic_valid_irq_mask(void) {
    uint32_t n = nvic_num_external_irqs();
    return (n >= 64) ? ~0ULL : ((1ULL << n) - 1);
}

static void nvic_recompute_priorities_flag(nvic_state_t *ns) {
    ns->priorities_nondefault = 0;
    uint32_t n = nvic_num_external_irqs();
    for (uint32_t i = 0; i < n; i++) {
        if (ns->priority[i] != 0) {
            ns->priorities_nondefault = 1;
            break;
        }
    }
}

/* CPUID value (M0+ default, M33 overlay changes this) */
uint32_t nvic_cpuid_value = 0x410CC601;  /* Cortex-M0+ default */

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
    systick_reset();
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
        for (int i = 0; i < NUM_EXTERNAL_IRQS_MAX; i++) {
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

/* Tick SysTick timer - called once per CPU step for active core.
 * O(1) arithmetic: handles multi-cycle steps without looping.
 *
 * The original per-cycle loop behavior:
 *   1. if cvr > 0: cvr--
 *   2. if cvr == 0: set COUNTFLAG, reload, pend interrupt
 *
 * This means a cycle that decrements cvr to 0 ALSO fires the event
 * in the same cycle, and the reload value is what the next cycle sees.
 */
void systick_tick(uint32_t cycles) {
    systick_state_t *st = systick_cur();
    if (!(st->csr & 1)) return; /* Not enabled */

    uint32_t reload = st->rvr & 0x00FFFFFF;
    uint32_t remaining = cycles;

    while (remaining > 0) {
        if (st->cvr == 0) {
            /* Counter already at zero from a previous tick — reload and fire.
             * This consumes one cycle (the cycle that "sees" zero and reloads). */
            st->csr |= (1u << 16); /* COUNTFLAG */
            st->cvr = reload;
            if (st->csr & 2) {
                st->pending = 1;
                corepool_wake_cores();
            }
            remaining--;
            if (reload == 0) return;
            continue;
        }

        if (remaining < st->cvr) {
            /* Won't reach zero this batch */
            st->cvr -= remaining;
            return;
        }

        /* Will reach zero: consume cvr cycles to hit 0 */
        remaining -= st->cvr;
        st->cvr = 0;

        /* The cycle that caused cvr to reach 0 also fires the event */
        st->csr |= (1u << 16); /* COUNTFLAG */
        st->cvr = reload;
        if (st->csr & 2) {
            st->pending = 1;
            corepool_wake_cores();
        }
        if (reload == 0) return;

        /* Fast-skip full periods if remaining > reload */
        if (remaining > reload) {
            uint32_t full_periods = (remaining - 1) / reload;
            remaining -= full_periods * reload;
            if (st->csr & 2) {
                st->pending = 1;
            }
        }
    }
}

void systick_tick_for_core(int core_id, uint32_t cycles) {
    if (core_id < 0 || core_id >= NUM_CORES) {
        return;
    }

    int saved_core = get_active_core();
    set_active_core(core_id);
    systick_tick(cycles);
    set_active_core(saved_core);
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
            if (vector_num >= 16 && nvic_irq_valid(vector_num - 16)) {
                return ns->priority[vector_num - 16] & 0xC0;
            }
            return 0xFF;
    }
}

/* Enable an IRQ on current core (set in ISER) */
void nvic_enable_irq(uint32_t irq) {
    if (nvic_irq_valid(irq)) {
        nvic_state_t *ns = nvic_cur();
        ns->enable |= (1ULL << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Core %d: Enabled IRQ %u (enable=0x%llX)\n",
                   get_active_core(), irq, (unsigned long long)ns->enable);
    }
}

/* Disable an IRQ on current core (set in ICER) */
void nvic_disable_irq(uint32_t irq) {
    if (nvic_irq_valid(irq)) {
        nvic_state_t *ns = nvic_cur();
        ns->enable &= ~(1ULL << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Core %d: Disabled IRQ %u (enable=0x%llX)\n",
                   get_active_core(), irq, (unsigned long long)ns->enable);
    }
}

/* Mark an IRQ as pending on current core (firmware ISPR write) */
void nvic_set_pending(uint32_t irq) {
    if (nvic_irq_valid(irq)) {
        nvic_state_t *ns = nvic_cur();
        ns->pending |= (1ULL << irq);
        corepool_wake_cores();
        if (irq >= 46) {
            static int user_irq_logged;
            if (!user_irq_logged) {
                fprintf(stderr, "[Init] NVIC pending user IRQ %u (RP2350 SPARE)\n", irq);
                user_irq_logged = 1;
            }
        }
        if (cpu.debug_enabled)
            printf("[NVIC] Core %d: Set pending IRQ %u (pending=0x%llX, enable=0x%llX)\n",
                   get_active_core(), irq,
                   (unsigned long long)ns->pending, (unsigned long long)ns->enable);
    }
}

/* Clear pending bit on current core (set in ICPR) */
void nvic_clear_pending(uint32_t irq) {
    if (nvic_irq_valid(irq)) {
        nvic_state_t *ns = nvic_cur();
        ns->pending &= ~(1ULL << irq);
        if (cpu.debug_enabled)
            printf("[NVIC] Core %d: Cleared pending IRQ %u (pending now=0x%llX)\n",
                   get_active_core(), irq, (unsigned long long)ns->pending);
    }
}

/* Set priority for an IRQ on current core */
void nvic_set_priority(uint32_t irq, uint8_t priority) {
    if (nvic_irq_valid(irq)) {
        nvic_state_t *ns = nvic_cur();
        ns->priority[irq] = priority & 0xC0;
        nvic_recompute_priorities_flag(ns);
    }
}

/**
 * Get the highest priority pending IRQ for the active core.
 * Optimized: fast-path uses CTZ when all priorities are default (0).
 */
uint32_t nvic_get_pending_irq(void) {
    nvic_state_t *ns = nvic_cur();
    uint64_t valid_mask = nvic_valid_irq_mask();
    uint64_t pending_and_enabled = ns->pending & ns->enable & valid_mask;

    if (pending_and_enabled == 0) {
        return 0xFFFFFFFF;
    }

    /* Fast path: if no custom priorities set, lowest IRQ number wins */
    if (ns->priorities_nondefault == 0) {
        return (uint32_t)__builtin_ctzll(pending_and_enabled);
    }

    /* Slow path: scan for highest priority (lowest value) */
    uint32_t highest_priority_irq = 0xFFFFFFFF;
    uint8_t highest_priority_value = 0xFF;
    uint64_t bits = pending_and_enabled;

    while (bits) {
        uint32_t irq = (uint32_t)__builtin_ctzll(bits);
        uint8_t prio = ns->priority[irq] & 0xC0;

        if (prio < highest_priority_value ||
            (prio == highest_priority_value && irq < highest_priority_irq)) {
            highest_priority_value = prio;
            highest_priority_irq = irq;
        }
        bits &= bits - 1;
    }

    return highest_priority_irq;
}

/* Read NVIC register (current core's view) */
uint32_t nvic_read_register(uint32_t addr) {
    nvic_state_t *ns = nvic_cur();
    systick_state_t *st = systick_cur();

    if (addr >= NVIC_IPR && addr < NVIC_IPR + NUM_EXTERNAL_IRQS_MAX) {
        uint32_t offset = (addr - NVIC_IPR) / 4;
        uint32_t result = 0;
        for (int i = 0; i < 4; i++) {
            uint32_t irq_idx = offset * 4 + i;
            if (nvic_irq_valid(irq_idx)) {
                result |= ((uint32_t)ns->priority[irq_idx]) << (i * 8);
            }
        }
        return result;
    }

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

        /* NVIC registers (ISER/ICER/ISPR/ICPR are banked: +0x100 = IRQ 0-31, +0x104 = IRQ 32-63) */
        case NVIC_ISER:
            return (uint32_t)ns->enable;
        case NVIC_ISER + 4:
            return (uint32_t)(ns->enable >> 32);

        case NVIC_ICER:
            return (uint32_t)ns->enable;
        case NVIC_ICER + 4:
            return (uint32_t)(ns->enable >> 32);

        case NVIC_ISPR:
            return (uint32_t)ns->pending;
        case NVIC_ISPR + 4:
            return (uint32_t)(ns->pending >> 32);

        case NVIC_ICPR:
            return (uint32_t)ns->pending;
        case NVIC_ICPR + 4:
            return (uint32_t)(ns->pending >> 32);

        case NVIC_IABR:
            return (uint32_t)ns->iabr;
        case NVIC_IABR + 4:
            return (uint32_t)(ns->iabr >> 32);

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
            return nvic_cpuid_value;

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

    if (addr >= NVIC_IPR && addr < NVIC_IPR + NUM_EXTERNAL_IRQS_MAX) {
        uint32_t offset = (addr - NVIC_IPR) / 4;
        for (int i = 0; i < 4; i++) {
            uint32_t irq_idx = offset * 4 + i;
            if (nvic_irq_valid(irq_idx)) {
                ns->priority[irq_idx] = (val >> (i * 8)) & 0xFF;
            }
        }
        nvic_recompute_priorities_flag(ns);
        return;
    }

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
            ns->enable |= (uint64_t)val;
            break;
        case NVIC_ISER + 4:
            if (val & 0xFFFF0000) {
                static int iser_hi_logged;
                if (!iser_hi_logged) {
                    fprintf(stderr, "[Init] NVIC ISER1=0x%08X (enables IRQ 32+)\n", val);
                    iser_hi_logged = 1;
                }
            }
            ns->enable |= ((uint64_t)val << 32);
            break;

        case NVIC_ICER:
            ns->enable &= ~((uint64_t)val);
            break;
        case NVIC_ICER + 4:
            ns->enable &= ~((uint64_t)val << 32);
            break;

        case NVIC_ISPR:
            ns->pending |= (uint64_t)val;
            corepool_wake_cores();
            break;
        case NVIC_ISPR + 4:
            if (val) {
                static int ispr_hi_logged;
                if (!ispr_hi_logged) {
                    fprintf(stderr, "[Init] NVIC ISPR1=0x%08X (pending IRQ 32+)\n", val);
                    ispr_hi_logged = 1;
                }
            }
            ns->pending |= ((uint64_t)val << 32);
            corepool_wake_cores();
            break;

        case NVIC_ICPR:
            ns->pending &= ~((uint64_t)val);
            break;
        case NVIC_ICPR + 4:
            ns->pending &= ~((uint64_t)val << 32);
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
    if (nvic_irq_valid(irq)) {
        irq_signal_count++;

        if (cpu.debug_enabled) {
            printf("[NVIC] *** SIGNAL IRQ %u (count=%u) ***\n",
                   irq, irq_signal_count);
        }

        last_irq_signal = irq;

        if (__builtin_expect(irq_latency_enabled, 0))
            irq_latency_pend(irq);

        for (int c = 0; c < 2; c++) {
            nvic_states[c].pending |= (1ULL << irq);
        }
        corepool_wake_cores();
    }
}
