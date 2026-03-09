/*
 * Interactive UART Name Prompt Test
 *
 * Demonstrates Bramble's stdin -> UART0 RX path:
 * 1. Prompt for a name
 * 2. Poll UART0 until input is available
 * 3. Echo typed characters while buffering them in RAM
 * 4. Print "Hello, <name>!"
 * 5. Halt with BKPT for clean test completion
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* UART0 */
.equ UART0_DR,          0x40034000
.equ UART0_FR,          0x40034018
.equ UART0_CR,          0x40034030
.equ UART_FR_RXFE,      0x10
.equ UART_CR_EN,        0x0301        /* UARTEN | TXE | RXE */

/* RAM
 * Use Core 0 private RAM top for stack safety in Bramble's dual-core mode.
 */
.equ RAM_TOP,           0x20021000
.equ NAME_BUF_SIZE,     32

/* Vector table */
.section .vectors, "a", %progbits
.align 2
.global _vectors

_vectors:
    .word RAM_TOP
    .word reset_handler + 1
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .rept 12
    .word 0xFFFFFFFF
    .endr

/* Code */
.section .text
.align 2

.thumb_func
.global reset_handler

reset_handler:
    ldr r7, =RAM_TOP
    mov sp, r7

    /* Enable UART0 */
    ldr r4, =UART0_CR
    ldr r5, =UART_CR_EN
    str r5, [r4]

    ldr r0, =prompt_msg
    bl print_string

    ldr r4, =name_buffer              /* r4 = buffer start */
    mov r6, r4                        /* r6 = current write pointer */
    movs r5, #0                       /* r5 = current length */

read_loop:
    bl uart_getc_wait

    movs r1, #13                      /* '\r' */
    cmp r0, r1
    beq finish_input

    movs r1, #10                      /* '\n' */
    cmp r0, r1
    beq finish_input

    movs r1, #8                       /* backspace */
    cmp r0, r1
    beq handle_backspace

    movs r1, #127                     /* delete */
    cmp r0, r1
    beq handle_backspace

    movs r1, #31                      /* keep room for terminator */
    cmp r5, r1
    bhs read_loop

    strb r0, [r6]
    adds r6, #1
    adds r5, #1
    b read_loop

handle_backspace:
    cmp r5, #0
    beq read_loop

    subs r5, #1
    subs r6, #1
    b read_loop

finish_input:
    movs r0, #0
    strb r0, [r6]

    movs r0, #10
    bl uart_putc

    ldr r0, =greeting_prefix
    bl print_string

    mov r0, r4
    bl print_string

    ldr r0, =greeting_suffix
    bl print_string

    bkpt #0
halt:
    b halt

.thumb_func
uart_getc_wait:
    push {r1, r2, r3, lr}
    ldr r1, =UART0_FR
    ldr r2, =UART0_DR

uart_wait_rx:
    ldr r0, [r1]
    movs r3, #UART_FR_RXFE
    tst r0, r3
    bne uart_wait_rx

    ldr r0, [r2]
    pop {r1, r2, r3, pc}

.thumb_func
uart_putc:
    push {r1, lr}
    ldr r1, =UART0_DR
    str r0, [r1]
    pop {r1, pc}

.thumb_func
print_string:
    push {r1, r2, r3, lr}
    mov r1, r0
    ldr r2, =UART0_DR

print_string_loop:
    ldrb r3, [r1]
    cmp r3, #0
    beq print_string_done

    str r3, [r2]
    adds r1, #1
    b print_string_loop

print_string_done:
    pop {r1, r2, r3, pc}

/* Read-only data */
.section .rodata
.align 2

prompt_msg:
    .asciz "What is your name? "

greeting_prefix:
    .asciz "Hello, "

greeting_suffix:
    .asciz "!\n"

.align 2

/* Writable data */
.section .bss
.align 2

name_buffer:
    .space NAME_BUF_SIZE
