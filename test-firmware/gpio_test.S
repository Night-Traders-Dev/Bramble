/*
 * GPIO Test (Simple) - FIXED with Correct Memory Addresses
 * 
 * This test:
 * 1. Configures GPIO 25 (LED on Pico) as SIO (software I/O)
 * 2. Sets GPIO 25 as output
 * 3. Toggles the LED 3 times
 * 4. Reads back GPIO state
 * 5. Reports via UART
 * 
 * Key fixes:
 * - Vector table at 0x10000100 (not 0x10000000)
 * - SP = 0x20042000 (RAM_TOP)
 * - GPIO25_CTRL = 0x400140CC (IO_BANK0_BASE + 0xCC)
 * - SIO_GPIO_OE_SET = 0xD0000024
 * - SIO_GPIO_OUT_SET = 0xD0000014
 * - SIO_GPIO_OUT_CLR = 0xD0000018
 * - SIO_GPIO_IN = 0xD0000004
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* SIO (Single-cycle I/O) - Fast GPIO access */
.equ SIO_BASE,          0xD0000000
.equ SIO_GPIO_IN,       0xD0000004    /* GPIO input values (read-only) */
.equ SIO_GPIO_OUT,      0xD0000010    /* GPIO output values */
.equ SIO_GPIO_OUT_SET,  0xD0000014    /* Atomic SET (write 1s to set) */
.equ SIO_GPIO_OUT_CLR,  0xD0000018    /* Atomic CLEAR (write 1s to clear) */
.equ SIO_GPIO_OUT_XOR,  0xD000001C    /* Atomic XOR */
.equ SIO_GPIO_OE,       0xD0000020    /* Output enable */
.equ SIO_GPIO_OE_SET,   0xD0000024    /* Atomic OE SET */
.equ SIO_GPIO_OE_CLR,   0xD0000028    /* Atomic OE CLEAR */

/* IO_BANK0 - Per-pin configuration */
.equ IO_BANK0_BASE,     0x40014000
.equ GPIO25_CTRL,       0x400140CC    /* GPIO 25 control (offset 0xCC = 25*8 + 4) */

/* UART0 */
.equ UART0_DR,          0x40034000    /* UART0 data register */

/* RAM */
.equ RAM_TOP,           0x20042000    /* Initial SP */

/* GPIO Constants */
.equ GPIO_FUNC_SIO,     5             /* Function 5 = SIO (software I/O) */
.equ LED_PIN,           25            /* GPIO 25 = LED on Pico */
.equ LED_MASK,          (1 << 25)     /* Bit mask for GPIO 25 */

/* Vector table (placed at flash 0x10000100) */
.section .vectors, "a", %progbits
.align 2
.global _vectors

_vectors:
    .word RAM_TOP                      /* SP = 0x20042000 */
    .word reset_handler + 1            /* Reset vector (Thumb bit set) */
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

    /* Print startup */
    ldr r0, =msg_start
    bl print_string

    /* Configure GPIO 25 as SIO (function 5) */
    ldr r0, =GPIO25_CTRL
    movs r1, #GPIO_FUNC_SIO
    str r1, [r0]

    /* Set GPIO 25 as output */
    ldr r0, =SIO_GPIO_OE_SET
    ldr r1, =LED_MASK
    str r1, [r0]

    ldr r0, =msg_configured
    bl print_string

    /* Toggle LED 3 times */
    movs r4, #3
toggle_loop:
    /* LED ON */
    ldr r0, =SIO_GPIO_OUT_SET
    ldr r1, =LED_MASK
    str r1, [r0]
    
    ldr r0, =msg_on
    bl print_string

    /* Delay */
    ldr r2, =50000
delay_on:
    subs r2, #1
    bne delay_on

    /* LED OFF */
    ldr r0, =SIO_GPIO_OUT_CLR
    ldr r1, =LED_MASK
    str r1, [r0]
    
    ldr r0, =msg_off
    bl print_string

    /* Delay */
    ldr r2, =50000
delay_off:
    subs r2, #1
    bne delay_off

    /* Decrement loop counter */
    subs r4, #1
    bne toggle_loop

    /* Read GPIO input state */
    ldr r0, =SIO_GPIO_IN
    ldr r0, [r0]
    
    /* Check if GPIO 25 is set */
    ldr r1, =LED_MASK
    ands r0, r1
    beq led_is_off

led_is_on:
    ldr r0, =msg_final_on
    bl print_string
    b test_complete

led_is_off:
    ldr r0, =msg_final_off
    bl print_string

test_complete:
    ldr r0, =msg_done
    bl print_string

    /* Halt */
    bkpt #0
    b . - 2

/* Print string via UART0 */
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

/* String data */
.section .rodata
.align 2

msg_start:
    .asciz "GPIO Test Starting\n"

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

.align 2
