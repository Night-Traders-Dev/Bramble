#include <stdio.h>
#include <string.h>
#include "timer.h"
#include "emulator.h"

/* Timer state */
timer_state_t timer_state;

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
    
    timer_state.armed = 0x0;     /* No alarms armed */
    timer_state.intr = 0x0;      /* No interrupts pending */
    timer_state.inte = 0x0;      /* Interrupts disabled */
    timer_state.paused = 0;      /* Timer running */
}

/* Update timer based on CPU cycles executed */
void timer_tick(uint32_t cycles) {
    if (timer_state.paused) {
        return;  /* Timer is paused, don't increment */
    }
    
    /* RP2040 runs at 125 MHz by default */
    /* 1 microsecond = 125 cycles at 125 MHz */
    /* For simplicity, we'll use 1 cycle = 1 microsecond (fast-forward simulation) */
    /* For accurate timing, divide cycles by 125 */
    
    /* Option 1: Fast simulation (1 cycle = 1 us) */
    timer_state.time_us += cycles;
    
    /* Option 2: Accurate timing (uncomment this, comment above) */
    // timer_state.time_us += (cycles / 125);  /* 125 cycles = 1 us at 125 MHz */
    
    /* Check if any armed alarms have triggered */
    for (int i = 0; i < 4; i++) {
        if (timer_state.armed & (1 << i)) {
            /* Compare lower 32 bits of timer with alarm value */
            uint32_t timer_low = (uint32_t)(timer_state.time_us & 0xFFFFFFFF);
            
            if (timer_low >= timer_state.alarm[i]) {
                /* Alarm triggered! */
                timer_state.intr |= (1 << i);     /* Set interrupt bit */
                timer_state.armed &= ~(1 << i);   /* Disarm alarm */
                
                /* TODO: Trigger IRQ to NVIC when implemented */
                /* For now, just log it */
                if (cpu.debug_enabled) {
                    printf("[TIMER] Alarm %d triggered at %llu us\n", 
                           i, timer_state.time_us);
                }
            }
        }
    }
}

/* Read from timer register space */
uint32_t timer_read32(uint32_t addr) {
    switch (addr) {
        case TIMER_TIMEHR:
            /* Read high word of 64-bit counter */
            return (uint32_t)((timer_state.time_us >> 32) & 0xFFFFFFFF);
            
        case TIMER_TIMELR:
            /* Read low word of 64-bit counter */
            return (uint32_t)(timer_state.time_us & 0xFFFFFFFF);
            
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
            return 0x0;  /* Force register (write-only) */
            
        case TIMER_INTS:
            /* Interrupt status = INTR & INTE */
            return timer_state.intr & timer_state.inte;
            
        case TIMER_PAUSE:
            return timer_state.paused;
            
        case TIMER_DBGPAUSE:
            return 0x0;  /* Debug pause (not implemented) */
            
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
            timer_state.armed |= 0x1;  /* Arm alarm 0 */
            if (cpu.debug_enabled) {
                printf("[TIMER] Alarm 0 set to %u us (armed)\n", val);
            }
            break;
            
        case TIMER_ALARM1:
            timer_state.alarm[1] = val;
            timer_state.armed |= 0x2;  /* Arm alarm 1 */
            if (cpu.debug_enabled) {
                printf("[TIMER] Alarm 1 set to %u us (armed)\n", val);
            }
            break;
            
        case TIMER_ALARM2:
            timer_state.alarm[2] = val;
            timer_state.armed |= 0x4;  /* Arm alarm 2 */
            if (cpu.debug_enabled) {
                printf("[TIMER] Alarm 2 set to %u us (armed)\n", val);
            }
            break;
            
        case TIMER_ALARM3:
            timer_state.alarm[3] = val;
            timer_state.armed |= 0x8;  /* Arm alarm 3 */
            if (cpu.debug_enabled) {
                printf("[TIMER] Alarm 3 set to %u us (armed)\n", val);
            }
            break;
            
        case TIMER_ARMED:
            /* Writing to ARMED disarms the specified alarms */
            timer_state.armed &= ~val;
            break;
            
        case TIMER_INTR:
            /* Write 1 to clear interrupt (W1C - Write 1 to Clear) */
            timer_state.intr &= ~val;
            if (cpu.debug_enabled && val) {
                printf("[TIMER] Cleared interrupt bits: 0x%X\n", val);
            }
            break;
            
        case TIMER_INTE:
            /* Interrupt enable */
            timer_state.inte = val & 0xF;  /* Only 4 alarms */
            break;
            
        case TIMER_INTF:
            /* Interrupt force - set interrupt bits */
            timer_state.intr |= (val & 0xF);
            break;
            
        case TIMER_PAUSE:
            /* Pause/resume timer */
            timer_state.paused = val & 0x1;
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
