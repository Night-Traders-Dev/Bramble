/* hello_world.S - Complete RP2040 bare-metal hello world */

.section .vectors, "a", %progbits
.align 2
.global _start

/* Vector table at 0x10000000 */
_start:
    .word 0x20042000          /* Initial SP */
    .word reset_handler + 1   /* Reset vector (Thumb bit) */
    .word 0xFFFFFFFF          /* NMI - unused */
    .word 0xFFFFFFFF          /* HardFault - unused */
    /* Fill rest with 0xFFFFFFFF */
    .rept 12
    .word 0xFFFFFFFF
    .endr

/* Reset handler */
.section .text
.type reset_handler, %function
reset_handler:
    /* UART0 base */
    ldr r0, =0x40034000
    
    /* Print "Hello" */
    ldr r1, =message
    bl uart_print
    
    /* Halt */
1:  wfi
    b 1b

/* UART print function */
.type uart_print, %function
uart_print:
    push {lr}
1:  ldrb r2, [r1], #1
    cmp r2, #0
    beq 2f
    bl uart_putc
    b 1b
2:  pop {pc}

/* UART put char - waits for TXFE */
.type uart_putc, %function
uart_putc:
    push {lr}
    ldr r3, =0x40034018  /* UART0_FR */
1:  ldr r4, [r3]
    tst r4, #0x20        /* Check TXFF bit 5 */
    bne 1b               /* Wait if FIFO full */
    str r2, [r0]         /* Write to UART0_DR */
    pop {pc}

/* Message */
.section .rodata
message:
    .asciz "Hello from pure assembly!"

.size reset_handler, . - reset_handler
