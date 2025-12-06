#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "timer.h"
#include "emulator.h"
#include "nvic.h"

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

    timer_state.armed = 0x0;   /* No alarms armed */
    timer_state.intr = 0x0;    /* No interrupts pending */
    timer_state.inte = 0x0;    /* Interrupts disabled */
    timer_state.paused = 0;    /* Timer running */
}

/* =========================================================================
 * CRITICAL FIX: Update timer based on CPU cycles executed
 * 
 * KEY INSIGHT FROM RP2040 PDF (Ch 11, Page 210):
 * "An alarm is programmed by setting a hardware register with a 32-bit 
 *  number, and when the lower-order 32 bits of the timer match, 
 *  an interrupt is fired."
 * 
 * BUGS FIXED:
 * 1. Changed comparison from >= to == (must be EXACT MATCH)
 * 2. DISARM alarm after it fires (prevents re-firing on timer wrap)
 * 3. Firmware must re-arm alarm explicitly for periodic operation
 * ========================================================================= */
void timer_tick(uint32_t cycles) {
    if (timer_state.paused) {
        return; /* Timer is paused, don't increment */
    }

    /* RP2040 runs at 125 MHz by default
     * 1 microsecond = 125 cycles at 125 MHz
     * For simplicity in simulation, we use 1 cycle = 1 microsecond
     * (This is a fast-forward simulation, not cycle-accurate timing)
     */
    timer_state.time_us += cycles;

    /* Check if any ARMED alarms have triggered
     * Extract lower 32 bits of the 64-bit timer for alarm comparison
     */
    uint32_t timer_low = (uint32_t)(timer_state.time_us & 0xFFFFFFFF);

    for (int i = 0; i < 4; i++) {
        if (!(timer_state.armed & (1 << i))) {
            continue; /* This alarm is not armed */
        }

        /* CRITICAL: Alarm fires on EXACT MATCH of lower 32 bits */
        if (timer_low == timer_state.alarm[i]) {
            /* Alarm triggered! */
            timer_state.intr |= (1 << i); /* Set interrupt bit */

            /* Signal to NVIC - this sets the pending bit in NVIC */
            nvic_signal_irq(IRQ_TIMER_IRQ_0 + i);

            if (cpu.debug_enabled) {
                printf("[TIMER] Alarm %d FIRED at %