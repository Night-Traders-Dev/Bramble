#include <stdio.h>
#include <string.h>

#include "timer.h"
#include "emulator.h"
#include "nvic.h"

/* Timer state */
timer_state_t timer_state;

/* Latch for atomic 64-bit timer reads: reading TIMELR latches the high word */
static uint32_t timer_latched_high = 0;

/* Initialize timer subsystem */
void timer_init(void) {
    timer_reset();
}

/* Reset timer to power-on defaults */
void timer_reset(void) {
    memset(&timer_state, 0, sizeof(timer_state_t));

    /* Timer starts at 0 microseconds */
    timer_state.time_us = 0;

    /* All alarms disabled */
    for (int i = 0; i < 4; i++) {
        timer_state.alarm[i] = 0;
    }

    timer_state.armed = 0x0;   /* No alarms armed */
    timer_state.intr = 0x0;    /* No interrupts pending */
    timer_state.inte = 0x0;    /* Interrupts disabled */
    timer_state.intf = 0x0;    /* No forced interrupts */
    timer_state.paused = 0;    /* Timer running */
}

/* Fire alarm i: set INTR, signal NVIC, disarm */
static void timer_fire_alarm(int i) {
    timer_state.intr |= (1 << i);
    nvic_signal_irq(IRQ_TIMER_IRQ_0 + i);
    timer_state.armed &= ~(1 << i);
}

void timer_tick(uint32_t cycles) {
    if (timer_state.paused) {
        return;
    }

    timer_state.time_us += cycles;
    uint32_t cur_low = (uint32_t)(timer_state.time_us & 0xFFFFFFFF);

    /* Check armed alarms: fire if target has been reached or passed.
     * Uses signed comparison to handle 32-bit wrap correctly.
     * This catches both exact matches AND alarms armed with past targets
     * (as on real RP2040, which fires on the next tick after arming). */
    for (int i = 0; i < 4; i++) {
        if (!(timer_state.armed & (1 << i))) {
            continue;
        }
        if ((int32_t)(cur_low - timer_state.alarm[i]) >= 0) {
            timer_fire_alarm(i);
        }
    }
}

/* Read from timer register space */
uint32_t timer_read32(uint32_t addr) {
    switch (addr) {
    case TIMER_TIMELR:
        /* Reading TIMELR latches the high word for atomic 64-bit reads.
         * Firmware reads TIMELR first, then TIMEHR to get consistent value. */
        timer_latched_high = (uint32_t)((timer_state.time_us >> 32) & 0xFFFFFFFF);
        return (uint32_t)(timer_state.time_us & 0xFFFFFFFF);

    case TIMER_TIMEHR:
        /* Returns latched value from last TIMELR read (atomic 64-bit pair) */
        return timer_latched_high;

    case TIMER_TIMERAWH:
        /* Raw read high word (same as TIMEHR for us) */
        return (uint32_t)((timer_state.time_us >> 32) & 0xFFFFFFFF);

    case TIMER_TIMERAWL:
        /* Raw read low word (same as TIMELR for us) */
        return (uint32_t)(timer_state.time_us & 0xFFFFFFFF);

    case TIMER_ALARM0:
        return timer_state.alarm[0];

    case TIMER_ALARM1:
        return timer_state.alarm[1];

    case TIMER_ALARM2:
        return timer_state.alarm[2];

    case TIMER_ALARM3:
        return timer_state.alarm[3];

    case TIMER_ARMED:
        return timer_state.armed;

    case TIMER_INTR:
        /* Raw interrupt status */
        return timer_state.intr;

    case TIMER_INTE:
        return timer_state.inte;

    case TIMER_INTF:
        return timer_state.intf;

    case TIMER_INTS:
        /* Interrupt status = (INTR | INTF) & INTE */
        return (timer_state.intr | timer_state.intf) & timer_state.inte;

    case TIMER_PAUSE:
        return timer_state.paused;

    case TIMER_DBGPAUSE:
        return 0x0; /* Debug pause (not implemented) */

    default:
        return 0x00000000;
    }
}

/* Write to timer register space */
void timer_write32(uint32_t addr, uint32_t val) {
    switch (addr) {
    case TIMER_TIMEHW:
        /* Write high word of 64-bit counter */
        timer_state.time_us = (timer_state.time_us & 0x00000000FFFFFFFF) |
                              ((uint64_t)val << 32);
        break;

    case TIMER_TIMELW:
        /* Write low word of 64-bit counter */
        timer_state.time_us = (timer_state.time_us & 0xFFFFFFFF00000000) |
                              (uint64_t)val;
        break;

    case TIMER_ALARM0:
        timer_state.alarm[0] = val;
        timer_state.armed |= 0x1;  /* RP2040: writing ALARM register arms it */
        break;

    case TIMER_ALARM1:
        timer_state.alarm[1] = val;
        timer_state.armed |= 0x2;
        break;

    case TIMER_ALARM2:
        timer_state.alarm[2] = val;
        timer_state.armed |= 0x4;
        break;

    case TIMER_ALARM3:
        timer_state.alarm[3] = val;
        timer_state.armed |= 0x8;
        break;

    case TIMER_ARMED:
        /* Writing to ARMED disarms the specified alarms */
        timer_state.armed &= ~val;
        if (val && cpu.debug_enabled) {
            fprintf(stderr, "[TIMER] Disarmed alarms: 0x%X\n", val);
        }
        break;

    case TIMER_INTR:
        /* Write 1 to clear interrupt (W1C - Write 1 to Clear)
         * This is how firmware clears the interrupt after handling it
         */
        timer_state.intr &= ~val;
        if (val && cpu.debug_enabled) {
            fprintf(stderr, "[TIMER] Cleared interrupt bits: 0x%X\n", val);
        }
        break;

    case TIMER_INTE:
        /* Interrupt enable register - controls which alarms generate interrupts */
        timer_state.inte = val & 0xF; /* Only 4 alarms */
        if (cpu.debug_enabled)
            fprintf(stderr, "[TIMER] Interrupt enable set to: 0x%X\n", val);
        /* Signal NVIC for any newly enabled interrupts */
        {
            uint32_t ints = (timer_state.intr | timer_state.intf) & timer_state.inte;
            for (int i = 0; i < 4; i++) {
                if (ints & (1 << i))
                    nvic_signal_irq(IRQ_TIMER_IRQ_0 + i);
            }
        }
        break;

    case TIMER_INTF:
        /* Interrupt force - separate from raw status (INTS = (INTR | INTF) & INTE) */
        timer_state.intf = val & 0xF;
        if (val && cpu.debug_enabled) {
            fprintf(stderr, "[TIMER] Forced interrupt bits: 0x%X\n", val);
        }
        /* Signal NVIC for any newly active interrupts */
        {
            uint32_t ints = (timer_state.intr | timer_state.intf) & timer_state.inte;
            for (int i = 0; i < 4; i++) {
                if (ints & (1 << i))
                    nvic_signal_irq(IRQ_TIMER_IRQ_0 + i);
            }
        }
        break;

    case TIMER_PAUSE:
        /* Pause/resume timer */
        timer_state.paused = val & 0x1;
        if (cpu.debug_enabled)
            fprintf(stderr, "[TIMER] %s\n", timer_state.paused ? "PAUSED" : "RESUMED");
        break;

    case TIMER_DBGPAUSE:
        /* Debug pause (not implemented) */
        break;

    /* TIMEHR, TIMELR, TIMERAWH, TIMERAWL are read-only */
    case TIMER_TIMEHR:
    case TIMER_TIMELR:
    case TIMER_TIMERAWH:
    case TIMER_TIMERAWL:
        /* Writes ignored */
        break;

    default:
        break;
    }
}
