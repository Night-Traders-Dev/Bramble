/*
 * Simplified GPIO Test - Inline version
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
    ldr r1, =UART0_DR
1:  ldrb r2, [r0]
    cmp r2, #0
    beq 2f
    str r2, [r1]
    adds r0, #1
    b 1b
2:

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
    ldr r1, =UART0_DR
1:  ldrb r2, [r0]
    cmp r2, #0
    beq 2f
    str r2, [r1]
    adds r0, #1
    b 1b
2:

    /* Toggle 3 times */
    movs r4, #3
toggle:
    /* LED ON */
    ldr r0, =SIO_GPIO_OUT_SET
    ldr r1, =LED_MASK
    str r1, [r0]

    ldr r0, =msg_on
    ldr r1, =UART0_DR
1:  ldrb r2, [r0]
    cmp r2, #0
    beq 2f
    str r2, [r1]
    adds r0, #1
    b 1b
2:

    /* Delay */
    ldr r3, =50000
1:  subs r3, #1
    bne 1b

    /* LED OFF */
    ldr r0, =SIO_GPIO_OUT_CLR
    ldr r1, =LED_MASK
    str r1, [r0]

    ldr r0, =msg_off
    ldr r1, =UART0_DR
1:  ldrb r2, [r0]
    cmp r2, #0
    beq 2f
    str r2, [r1]
    adds r0, #1
    b 1b
2:

    /* Delay */
    ldr r3, =50000
1:  subs r3, #1
    bne 1b

    subs r4, #1
    bne toggle

    /* Read GPIO state */
    ldr r0, =SIO_GPIO_IN
    ldr r0, [r0]
    ldr r1, =LED_MASK
    ands r0, r1
    bne final_on

final_off:
    ldr r0, =msg_off_final
    b final_print

final_on:
    ldr r0, =msg_on_final

final_print:
    ldr r1, =UART0_DR
1:  ldrb r2, [r0]
    cmp r2, #0
    beq 2f
    str r2, [r1]
    adds r0, #1
    b 1b
2:

    /* Print done */
    ldr r0, =msg_done
    ldr r1, =UART0_DR
1:  ldrb r2, [r0]
    cmp r2, #0
    beq done
    str r2, [r1]
    adds r0, #1
    b 1b

done:
    bkpt #0

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
