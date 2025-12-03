/* hello_world.S - Fixed with explicit labels */

.section .vectors, "a", %progbits
.align 2
.global _start

_start:
    .word 0x20042000          /* Initial SP */
    .word reset_handler + 1   /* Reset vector */
    .rept 14
    .word 0xFFFFFFFF
    .endr

.section .text
.thumb_func
reset_handler:
    ldr r0, =message          /* Load message pointer */
    ldr r1, =0x40034000       /* UART0 base */
    ldr r2, =0x40034018       /* UART0_FR */

print_loop:
    ldrb r3, [r0]             /* Load byte */
    cmp r3, #0
    beq  print_done           /* If null, done */

wait_tx_ready:
    ldr r4, [r2]              /* Read UART0_FR */
    movs r5, #0x20
    tst r4, r5                /* Check TXFF bit 5 */
    bne  wait_tx_ready        /* Wait if FIFO full */

    str r3, [r1]              /* Write to UART0_DR */
    add r0, #1                /* Advance pointer */
    b    print_loop           /* Next character */

print_done:
    BKPT #0
    B print_done

.section .rodata
message:
    .ascii "Hello from ASM!"
    .byte 0                   /* Null terminator */
    .align 2
