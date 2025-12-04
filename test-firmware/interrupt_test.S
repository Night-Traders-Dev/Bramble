/*
 * Timer Interrupt Test for Bramble RP2040 Emulator
 * Tests full interrupt flow: Timer → NVIC → CPU Exception Entry
 * 
 * Key fixes from RP2040 Assembly Language Programming (Smith):
 * 1. Clear pending interrupt BEFORE enabling in NVIC (p.211)
 * 2. Use NVIC_ICPR to clear, then NVIC_ISER to enable
 * 3. Vector table entry offset for IRQ 0 is 0x40 (vector 16)
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

/* NVIC Registers (from RP2040 datasheet) */
.equ PPBBASE,           0xE0000000
.equ NVIC_ICPR,         0xE000E180
.equ NVIC_ISER,         0xE000E100
.equ M0PLUS_VTOR,       0xE000ED08

/* Vector table */
.section .vectors, "a", %progbits
.align 2
.global _start

_start:
    .word 0x20042000          /* Initial SP */
    .word reset_handler + 1   /* Reset vector */
    .word 0xFFFFFFFF          /* NMI */
    .word 0xFFFFFFFF          /* HardFault */
    .rept 12
    .word 0xFFFFFFFF
    .endr
    .word timer_isr + 1       /* IRQ 0 - Timer (vector 16) */

/* Code section */
.section .text
.align 2

.thumb_func
reset_handler:
    /* Print startup message */
    ldr r0, =msg_start
    bl print_string

    /* CRITICAL: Clear any pending interrupt for IRQ 0 FIRST (before enabling) */
    /* This follows the SDK recommendation from the PDF (p.211) */
    ldr r0, =NVIC_ICPR
    movs r1, #1              /* Bit 0 = IRQ 0 */
    str r1, [r0]
    
    ldr r0, =msg_cleared
    bl print_string

    /* NOW enable IRQ 0 in NVIC */
    ldr r0, =NVIC_ISER
    movs r1, #1              /* Bit 0 = IRQ 0 */
    str r1, [r0]
    
    ldr r0, =msg_nvic_enabled
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
    
    /* Enable interrupts globally */
    cpsie i
    
    ldr r0, =msg_waiting
    bl print_string
    
    /* Wait for interrupt (this will be interrupted!) */
    wfi
    
    ldr r0, =msg_resumed
    bl print_string
    
    b test_done

/* Timer ISR - called when alarm fires */
.thumb_func
timer_isr:
    ldr r0, =msg_in_isr
    bl print_string
    
    /* Clear the interrupt */
    ldr r0, =TIMER_INTR
    movs r1, #1
    str r1, [r0]
    
    ldr r0, =msg_isr_done
    bl print_string
    
    /* Return from interrupt (special return) */
    bx lr

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
    .asciz "Timer Interrupt Test Starting...\n"
msg_cleared:
    .asciz "NVIC IRQ 0 pending cleared\n"
msg_nvic_enabled:
    .asciz "NVIC IRQ 0 enabled\n"
msg_time_now:
    .asciz "Current time read\n"
msg_alarm_set:
    .asciz "Alarm set for +1000us\n"
msg_waiting:
    .asciz "Waiting for interrupt (WFI)...\n"
msg_in_isr:
    .asciz ">>> TIMER ISR EXECUTING <<<\n"
msg_isr_done:
    .asciz "ISR complete, returning\n"
msg_resumed:
    .asciz "Resumed after interrupt\n"
msg_done:
    .asciz "Timer Interrupt Test Complete!\n"
.align 2
