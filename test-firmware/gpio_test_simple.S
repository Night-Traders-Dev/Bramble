/*
 * Simplified GPIO Test - Fixed version
 */

.syntax unified
.cpu cortex-m0plus
.thumb

.equ UART0_DR,          0x40034000
.equ GPIO25_CTRL,       0x400140CC
.equ SIO_GPIO_OE_SET,   0xD0000024
.equ SIO_GPIO_OUT_SET,  0xD0000014
.equ SIO_GPIO_OUT_CLR,  0xD0000018
.equ SIO_GPIO_IN,       0xD0000004
.equ LED_MASK,          (1 << 25)

.section .vectors, "a", %progbits
.align 2

_vectors:
    .word 0x20042000          /* SP */
    .word _start + 1          /* Reset */
    .rept 14
    .word 0xFFFFFFFF
    .endr

.section .text
.align 2
.global _start
.thumb_func

_start:
    /* Print: GPIO Test Starting... */
    ldr r0, =msg1
    bl print_str

    /* Configure GPIO 25 as SIO (function 5) */
    ldr r0, =GPIO25_CTRL
    movs r1, #5
    str r1, [r0]

    /* Enable GPIO 25 output */
    ldr r0, =SIO_GPIO_OE_SET
    ldr r1, =LED_MASK
    str r1, [r0]

    /* Print: Configured */
    ldr r0, =msg2
    bl print_str

    /* Toggle 3 times */
    movs r4, #3
toggle_loop:
    /* LED ON */
    ldr r0, =SIO_GPIO_OUT_SET
    ldr r1, =LED_MASK
    str r1, [r0]

    ldr r0, =msg_on
    bl print_str

    /* Delay */
    ldr r3, =50000
delay1:
    subs r3, #1
    bne delay1

    /* LED OFF */
    ldr r0, =SIO_GPIO_OUT_CLR
    ldr r1, =LED_MASK
    str r1, [r0]

    ldr r0, =msg_off
    bl print_str

    /* Delay */
    ldr r3, =50000
delay2:
    subs r3, #1
    bne delay2

    subs r4, #1
    bne toggle_loop

    /* Read GPIO state */
    ldr r0, =SIO_GPIO_IN
    ldr r0, [r0]
    ldr r1, =LED_MASK
    ands r0, r1
    bne led_on_final

led_off_final:
    ldr r0, =msg_off_final
    b print_final

led_on_final:
    ldr r0, =msg_on_final

print_final:
    bl print_str

    /* Print done */
    ldr r0, =msg_done
    bl print_str

    /* Halt */
    bkpt #0

/* Print string subroutine */
/* Input: r0 = pointer to null-terminated string */
.align 2
.thumb_func
print_str:
    push {r0, r1, r2, lr}
    movs r1, r0              /* String pointer */
    ldr r2, =UART0_DR        /* UART data register */
print_loop:
    ldrb r0, [r1]            /* Load byte */
    cmp r0, #0               /* Check for null */
    beq print_done
    str r0, [r2]             /* Write to UART */
    adds r1, #1              /* Next character */
    b print_loop
print_done:
    pop {r0, r1, r2, pc}     /* Return */

.section .rodata
.align 2
msg1:           .asciz "GPIO Test Starting...\n"
msg2:           .asciz "GPIO configured\n"
msg_on:         .asciz "LED ON\n"
msg_off:        .asciz "LED OFF\n"
msg_on_final:   .asciz "Final: ON\n"
msg_off_final:  .asciz "Final: OFF\n"
msg_done:       .asciz "Test Complete!\n"
.align 2
