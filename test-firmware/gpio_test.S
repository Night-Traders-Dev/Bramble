/*
 * GPIO Test Firmware for Bramble RP2040 Emulator
 * Tests basic GPIO functionality:
 * - Configure GPIO 25 (LED pin on Pico) as output
 * - Toggle the pin several times
 * - Read back pin state
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* GPIO Register Addresses */
.equ SIO_BASE,          0xD0000000
.equ SIO_GPIO_OUT,      0xD0000010
.equ SIO_GPIO_OUT_SET,  0xD0000014
.equ SIO_GPIO_OUT_CLR,  0xD0000018
.equ SIO_GPIO_OUT_XOR,  0xD000001C
.equ SIO_GPIO_OE,       0xD0000020
.equ SIO_GPIO_OE_SET,   0xD0000024
.equ SIO_GPIO_IN,       0xD0000004

.equ IO_BANK0_BASE,     0x40014000
.equ GPIO25_CTRL,       0x400140CC  /* GPIO 25 control register (offset 0xCC) */

.equ UART0_DR,          0x40034000

.equ GPIO_FUNC_SIO,     5           /* Function 5 = SIO (software I/O) */

.equ LED_PIN,           25          /* GPIO 25 is the LED on Raspberry Pi Pico */
.equ LED_MASK,          (1 << LED_PIN)

/* Vector table */
.section .vectors, "a", %progbits
.align 2
.global _start

_start:
    .word 0x20042000          /* Initial SP (top of RAM) */
    .word reset_handler + 1   /* Reset vector (Thumb mode) */
    .rept 14
    .word 0xFFFFFFFF          /* Unused exception vectors */
    .endr

/* Entry point */
.section .text
.thumb_func
reset_handler:
    /* Print startup message */
    ldr r1, =msg_start
    bl print_string

    /* Configure GPIO 25 as SIO function */
    ldr r0, =GPIO25_CTRL
    movs r1, #GPIO_FUNC_SIO
    str r1, [r0]

    /* Set GPIO 25 as output */
    ldr r0, =SIO_GPIO_OE_SET
    ldr r1, =LED_MASK
    str r1, [r0]

    ldr r1, =msg_configured
    bl print_string

    /* Toggle LED 5 times */
    movs r4, #5              /* Loop counter */
toggle_loop:
    /* Turn LED ON */
    ldr r0, =SIO_GPIO_OUT_SET
    ldr r1, =LED_MASK
    str r1, [r0]
    
    ldr r1, =msg_on
    bl print_string

    /* Simple delay */
    ldr r2, =100000
delay_on:
    subs r2, #1
    bne delay_on

    /* Turn LED OFF */
    ldr r0, =SIO_GPIO_OUT_CLR
    ldr r1, =LED_MASK
    str r1, [r0]
    
    ldr r1, =msg_off
    bl print_string

    /* Simple delay */
    ldr r2, =100000
delay_off:
    subs r2, #1
    bne delay_off

    /* Decrement loop counter */
    subs r4, #1
    bne toggle_loop

    /* Read GPIO state */
    ldr r0, =SIO_GPIO_IN
    ldr r0, [r0]
    
    /* Check if bit 25 is set */
    ldr r1, =LED_MASK
    ands r0, r1
    beq led_is_off

led_is_on:
    ldr r1, =msg_final_on
    bl print_string
    b test_complete

led_is_off:
    ldr r1, =msg_final_off
    bl print_string

test_complete:
    ldr r1, =msg_done
    bl print_string

    /* Halt */
    bkpt #0
    b test_complete

/* Print null-terminated string */
/* r1 = pointer to string */
.thumb_func
print_string:
    ldr r2, =UART0_DR
print_loop:
    ldrb r3, [r1]
    cmp r3, #0
    beq print_done
    str r3, [r2]
    adds r1, #1
    b print_loop
print_done:
    bx lr

/* Data section */
.section .rodata
.align 2
msg_start:
    .asciz "GPIO Test Starting...\n"
msg_configured:
    .asciz "GPIO 25 configured as output\n"
msg_on:
    .asciz "LED ON\n"
msg_off:
    .asciz "LED OFF\n"
msg_final_on:
    .asciz "Final state: LED ON\n"
msg_final_off:
    .asciz "Final state: LED OFF\n"
msg_done:
    .asciz "GPIO Test Complete!\n"
