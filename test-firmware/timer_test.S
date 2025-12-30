/*
 * Timer Test - FIXED with Correct Memory Addresses
 * 
 * This test:
 * 1. Reads the timer low word (TIMER_TIMELR at 0x4005400C)
 * 2. Does some work (delay loop)
 * 3. Reads timer again
 * 4. Outputs via UART
 * 
 * Key fixes:
 * - Vector table at 0x10000100 (not 0x10000000)
 * - SP initialized to 0x20042000 (RAM_TOP)
 * - Correct timer register addresses
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* Timer Register Addresses (from RP2040 datasheet) */
.equ TIMER_BASE,        0x40054000
.equ TIMER_TIMELR,      0x4005400C    /* Read low word of timer */
.equ TIMER_TIMEHR,      0x40054008    /* Read high word of timer */
.equ UART0_DR,          0x40034000    /* UART0 data register */

/* RAM Addresses */
.equ RAM_TOP,           0x20042000    /* Initial SP (top of 264 KB RAM) */

/* Vector table (placed at flash 0x10000100 by linker) */
.section .vectors, "a", %progbits
.align 2
.global _vectors

_vectors:
    .word RAM_TOP                      /* SP = 0x20042000 */
    .word reset_handler + 1            /* Reset vector (Thumb bit) */
    .word 0xFFFFFFFF                   /* NMI */
    .word 0xFFFFFFFF                   /* HardFault */
    .rept 12
    .word 0xFFFFFFFF                   /* Remaining exceptions */
    .endr

/* Code section */
.section .text
.align 2

.thumb_func
.global reset_handler

reset_handler:
    /* Initialize SP using temp register (sp = R13 can't use = directly) */
    ldr r7, =RAM_TOP
    mov sp, r7
    
    /* Print startup message */
    ldr r0, =msg_start
    bl print_string

    /* Read timer low word */
    ldr r0, =TIMER_TIMELR
    ldr r1, [r0]        /* r1 = first timer reading */
    
    /* Print "Time 1 read" */
    ldr r0, =msg_time1
    bl print_string

    /* Do some work (delay loop) */
    ldr r2, =50000
delay_loop:
    subs r2, #1
    bne delay_loop

    /* Read timer again */
    ldr r0, =TIMER_TIMELR
    ldr r3, [r0]        /* r3 = second timer reading */
    
    /* Print "Time 2 read" */
    ldr r0, =msg_time2
    bl print_string

    /* Calculate elapsed time (r3 - r1) */
    subs r4, r3, r1

    /* Print elapsed */
    ldr r0, =msg_elapsed
    bl print_string

    /* Test complete */
    ldr r0, =msg_done
    bl print_string

    /* Halt */
    bkpt #0
    
    /* Infinite loop as backup */
    b . - 2

/* Print null-terminated string via UART0 */
.thumb_func
print_string:
    push {r1, r2, r3, lr}
    movs r1, r0                        /* r1 = string pointer */
    ldr r2, =UART0_DR                  /* r2 = UART0 data register */

print_loop:
    ldrb r3, [r1]                      /* Load next character */
    cmp r3, #0                         /* Check for null terminator */
    beq print_done
    
    str r3, [r2]                       /* Write to UART */
    adds r1, #1                        /* Next character */
    b print_loop

print_done:
    pop {r1, r2, r3, pc}

/* String data (in flash after code) */
.section .rodata
.align 2

msg_start:
    .asciz "Timer Test Starting\n"

msg_time1:
    .asciz "Timer value 1 read\n"

msg_time2:
    .asciz "Timer value 2 read\n"

msg_elapsed:
    .asciz "Elapsed time: (see debug output)\n"

msg_done:
    .asciz "Timer Test Complete!\n"

.align 2
