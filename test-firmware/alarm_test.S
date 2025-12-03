/*
 * Timer Alarm Test for Bramble RP2040 Emulator
 * Tests timer alarm functionality
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* Timer Register Addresses */
.equ TIMER_BASE,        0x40054000
.equ TIMER_TIMELR,      0x4005400C
.equ TIMER_ALARM0,      0x40054010
.equ TIMER_INTR,        0x40054034
.equ TIMER_INTE,        0x40054038
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

    /* Read current timer value */
    ldr r0, =TIMER_TIMELR
    ldr r1, [r0]
    
    ldr r0, =msg_time_now
    bl print_string
    
    /* Set alarm for 1000 microseconds in future */
    ldr r2, =1000
    add r1, r2
    ldr r0, =TIMER_ALARM0
    str r1, [r0]
    
    ldr r0, =msg_alarm_set
    bl print_string
    
    /* Poll timer interrupt register until alarm fires */
    ldr r4, =0              /* Timeout counter */
poll_loop:
    ldr r0, =TIMER_INTR
    ldr r1, [r0]
    movs r2, #1             /* FIXED: Load mask into register */
    tst r1, r2              /* Check bit 0 for alarm 0 */
    bne alarm_fired
    
    /* Increment timeout counter */
    adds r4, #1
    ldr r2, =100000         /* Timeout after 100k iterations */
    cmp r4, r2
    blt poll_loop
    
    /* Timeout! */
    ldr r0, =msg_timeout
    bl print_string
    b test_done

alarm_fired:
    ldr r0, =msg_alarm_fired
    bl print_string
    
    /* Clear the interrupt */
    movs r1, #1
    ldr r0, =TIMER_INTR
    str r1, [r0]
    
    ldr r0, =msg_intr_cleared
    bl print_string

test_done:
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
    .asciz "Timer Alarm Test Starting...\n"
msg_time_now:
    .asciz "Current time read\n"
msg_alarm_set:
    .asciz "Alarm set for +1000us\n"
msg_alarm_fired:
    .asciz "SUCCESS: Alarm fired!\n"
msg_intr_cleared:
    .asciz "Interrupt cleared\n"
msg_timeout:
    .asciz "ERROR: Alarm timeout\n"
msg_done:
    .asciz "Timer Alarm Test Complete!\n"
.align 2
