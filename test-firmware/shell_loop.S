/*
 * Shell Test Firmware for Bramble RP2040 Emulator
 * Simple interactive shell in pure assembly
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* UART Register Addresses */
.equ UART0_DR,          0x40034000
.equ UART0_FR,          0x40034018

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
    /* Print welcome message */
    ldr r0, =msg_welcome
    bl print_string

main_loop:
    /* Print prompt */
    ldr r0, =msg_prompt
    bl print_string

    /* Read command line into buffer */
    ldr r4, =input_buffer    /* r4 = buffer pointer */
    movs r5, #0              /* r5 = buffer index */

read_loop:
    /* Wait for character */
    bl uart_getc
    movs r6, r0              /* r6 = received char */

    /* Check for newline/carriage return */
    cmp r6, #0x0D            /* Carriage return */
    beq process_command
    cmp r6, #0x0A            /* Line feed */
    beq process_command

    /* Check for backspace */
    cmp r6, #0x08            /* Backspace */
    beq handle_backspace
    cmp r6, #0x7F            /* DEL */
    beq handle_backspace

    /* Echo character */
    movs r0, r6
    bl uart_putc

    /* Store in buffer (max 63 chars) */
    cmp r5, #63
    bge read_loop            /* Buffer full, ignore */

    strb r6, [r4, r5]        /* Store character */
    adds r5, #1              /* Increment index */
    b read_loop

handle_backspace:
    cmp r5, #0               /* Check if buffer empty */
    beq read_loop            /* Nothing to delete */

    subs r5, #1              /* Decrement index */

    /* Echo backspace sequence: BS + Space + BS */
    movs r0, #0x08
    bl uart_putc
    movs r0, #0x20
    bl uart_putc
    movs r0, #0x08
    bl uart_putc

    b read_loop

process_command:
    /* Print newline */
    movs r0, #0x0A
    bl uart_putc

    /* Null-terminate buffer */
    movs r0, #0
    strb r0, [r4, r5]

    /* Check if empty command */
    cmp r5, #0
    beq main_loop            /* Empty, show prompt again */

    /* Check for 'help' command */
    ldr r0, =input_buffer
    ldr r1, =cmd_help
    bl strcmp
    cmp r0, #0
    beq cmd_help_handler

    /* Check for 'test' command */
    ldr r0, =input_buffer
    ldr r1, =cmd_test
    bl strcmp
    cmp r0, #0
    beq cmd_test_handler

    /* Check for 'exit' command */
    ldr r0, =input_buffer
    ldr r1, =cmd_exit
    bl strcmp
    cmp r0, #0
    beq cmd_exit_handler

    /* Unknown command - echo it back */
    ldr r0, =msg_echo
    bl print_string

    ldr r0, =input_buffer
    bl print_string

    movs r0, #0x0A
    bl uart_putc

    b main_loop

/* Command handlers */
.thumb_func
cmd_help_handler:
    ldr r0, =msg_help
    bl print_string
    b main_loop

.thumb_func
cmd_test_handler:
    ldr r0, =msg_test
    bl print_string
    b main_loop

.thumb_func
cmd_exit_handler:
    ldr r0, =msg_exit
    bl print_string
    bkpt #0

/* UART functions */
.thumb_func
uart_putc:
    push {r1, r2, lr}
    movs r1, r0              /* Save character */
    ldr r2, =UART0_DR
    str r1, [r2]             /* Write to UART */
    pop {r1, r2, pc}

.thumb_func
uart_getc:
    push {r1, r2, lr}
    ldr r1, =UART0_DR
wait_char:
    /* In Bramble, we'll simulate polling by reading directly */
    /* Real hardware would check UART0_FR bit 4 (RXFE) */
    ldr r0, [r1]             /* Read data register */

    /* Check if valid character (non-zero for simulation) */
    cmp r0, #0
    beq wait_char            /* Wait if no data */

    pop {r1, r2, pc}

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

/* Simple string compare - returns 0 if equal */
.thumb_func
strcmp:
    push {r2, r3, lr}
strcmp_loop:
    ldrb r2, [r0]            /* Load byte from str1 */
    ldrb r3, [r1]            /* Load byte from str2 */
    cmp r2, r3               /* Compare bytes */
    bne strcmp_diff          /* Different */
    cmp r2, #0               /* Check for null terminator */
    beq strcmp_equal         /* Both strings ended - equal */
    adds r0, #1
    adds r1, #1
    b strcmp_loop
strcmp_equal:
    movs r0, #0              /* Return 0 for equal */
    pop {r2, r3, pc}
strcmp_diff:
    movs r0, #1              /* Return non-zero for different */
    pop {r2, r3, pc}

/* Data section */
.section .rodata
.align 2
msg_welcome:
    .asciz "Bramble Shell v0.1\n\rType 'help' for commands\n\r\n\r"
msg_prompt:
    .asciz "> "
msg_echo:
    .asciz "You typed: "
msg_help:
    .asciz "Available commands:\n\r  help  - Show this message\n\r  test  - Run a test\n\r  exit  - Exit shell\n\r"
msg_test:
    .asciz "Test command executed!\n\r"
msg_exit:
    .asciz "Exiting shell...\n\r"

/* Command strings for matching */
cmd_help:
    .asciz "help"
cmd_test:
    .asciz "test"
cmd_exit:
    .asciz "exit"

.align 2

/* BSS section */
.section .bss
.align 2
input_buffer:
    .space 64                /* 64 byte input buffer */

.end
