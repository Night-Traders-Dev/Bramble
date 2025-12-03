/*
 * Timer Test Firmware for Bramble RP2040 Emulator
 * Tests timer functionality by reading current time
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* Timer Register Addresses */
.equ TIMER_BASE,        0x40054000
.equ TIMER_TIMELR,      0x4005400C
.equ TIMER_TIMEHR,      0x40054008
.equ UART0_DR,          0x40034000

/* Vector table */
.section .vectors, "a", %progbits
.align 2
.global _start

_start:
    .word 0x20042000          /* Initial SP */
    .word reset_handler + 1   /* Reset vector */
    .rept 14
    .word 0xFFFFFFFF
    .endr

/* Code section */
.section .text
.align 2

.thumb_func
reset_handler:
    /* Print startup message */
    ldr r0, =msg_start
    bl print_string

    /* Read timer low word */
    ldr r0, =TIMER_TIMELR
    ldr r1, [r0]
    
    /* Print timer value */
    ldr r0, =msg_time1
    bl print_string
    
    /* Do some work (delay loop) */
    ldr r2, =10000
delay_loop:
    subs r2, #1
    bne delay_loop
    
    /* Read timer again */
    ldr r0, =TIMER_TIMELR
    ldr r3, [r0]
    
    /* Print second timer value */
    ldr r0, =msg_time2
    bl print_string
    
    /* Calculate elapsed time (r3 - r1) */
    subs r4, r3, r1
    
    ldr r0, =msg_elapsed
    bl print_string
    
    /* Test complete */
    ldr r0, =msg_done
    bl print_string
    
    bkpt #0

/* Print null-terminated string */
.thumb_func
print_string:
    push {r1, r2, r3, lr}
    movs r1, r0
    ldr r2, =UART0_DR
print_loop:
    ldrb r3, [r1]
    cmp r3, #0
    beq print_done
    str r3, [r2]
    adds r1, #1
    b print_loop
print_done:
    pop {r1, r2, r3, pc}

/* Data section */
.section .rodata
.align 2
msg_start:
    .asciz "Timer Test Starting...\n"
msg_time1:
    .asciz "Timer value 1: (check debug)\n"
msg_time2:
    .asciz "Timer value 2: (check debug)\n"
msg_elapsed:
    .asciz "Elapsed time calculated\n"
msg_done:
    .asciz "Timer Test Complete!\n"
.align 2
