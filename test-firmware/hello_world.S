/*
 * Hello World Test - Fixed memory addresses for Bramble
 * 
 * Memory Layout (RP2040):
 *   Vector table:   0x10000100 - 0x10000140 (after boot2)
 *   Flash code:     0x10000100 - 0x10200000
 *   RAM:            0x20000000 - 0x20042000 (264 KB)
 *   RAM_TOP:        0x20042000
 *   UART0:          0x40034000
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* Peripheral Addresses */
.equ UART0_DR,          0x40034000   /* UART Data Register */
.equ UART0_FR,          0x40034018   /* UART Flag Register */

.equ RAM_TOP,           0x20042000   /* Top of RAM (264 KB) */

/* Vector table (placed at flash offset 0x100 by linker script) */
.section .vectors, "a", %progbits
.align 2
.global _vectors

_vectors:
    .word RAM_TOP                      /* SP = top of RAM */
    .word reset_handler + 1            /* PC = reset handler (Thumb bit set) */
    .word 0xFFFFFFFF                   /* NMI handler */
    .word 0xFFFFFFFF                   /* HardFault handler */
    .rept 12
    .word 0xFFFFFFFF                   /* Remaining system exceptions */
    .endr

/* Code section */
.section .text
.align 2

.thumb_func
.global reset_handler

reset_handler:
    /* Initialize stack pointer using temp register (sp = R13 can't use = directly) */
    ldr r7, =RAM_TOP
    mov sp, r7
    
    /* Load message address */
    ldr r0, =message
    
    /* Load UART data register address */
    ldr r1, =UART0_DR
    
    /* Load UART flag register address */
    ldr r2, =UART0_FR

print_loop:
    /* Load next character */
    ldrb r3, [r0]
    
    /* Check for null terminator */
    cmp r3, #0
    beq done
    
    /* Write to UART */
    str r3, [r1]
    
    /* Advance to next character */
    adds r0, #1
    
    b print_loop

done:
    /* Halt with breakpoint */
    bkpt #0
    
    /* Infinite loop as backup */
    b done

/* String data (placed in flash after code) */
.section .rodata
.align 2

message:
    .asciz "Hello from Bramble RP2040 Emulator!\n"
    .align 2
