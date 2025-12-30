/*
 * Interrupt Test - FIXED with Correct Memory Addresses
 * 
 * This test demonstrates full interrupt flow:
 * 1. Configure Timer alarm
 * 2. Enable NVIC interrupt
 * 3. Wait for interrupt (WFI)
 * 4. ISR executes
 * 5. Return to main code
 * 
 * Key fixes:
 * - Vector table at 0x10000100 (not 0x10000000)
 * - SP = 0x20042000 (RAM_TOP)
 * - NVIC_ICPR = 0xE000E280 (clear pending)
 * - NVIC_ISER = 0xE000E100 (set enable)
 * - Timer interrupts mapped to vector 16+ (IRQ 0 = vector 16)
 * - ISR address in vector table at offset 0x40 (16 * 4 = 64 = 0x40)
 */

.syntax unified
.cpu cortex-m0plus
.thumb

/* Timer */
.equ TIMER_BASE,        0x40054000
.equ TIMER_TIMELR,      0x4005400C
.equ TIMER_ALARM0,      0x40054010
.equ TIMER_INTR,        0x40054034
.equ TIMER_INTE,        0x40054038

/* NVIC (Nested Vectored Interrupt Controller) */
.equ NVIC_ICPR,         0xE000E280    /* Interrupt Clear Pending Register */
.equ NVIC_ISER,         0xE000E100    /* Interrupt Set Enable Register */

/* UART0 */
.equ UART0_DR,          0x40034000

/* RAM */
.equ RAM_TOP,           0x20042000

/* Vector table (0x10000100) */
.section .vectors, "a", %progbits
.align 2
.global _vectors

_vectors:
    .word RAM_TOP                      /* Vector 0: SP */
    .word reset_handler + 1            /* Vector 1: Reset */
    .word 0xFFFFFFFF                   /* Vector 2: NMI */
    .word 0xFFFFFFFF                   /* Vector 3: HardFault */
    .rept 12
    .word 0xFFFFFFFF                   /* Vectors 4-15: System exceptions */
    .endr
    .word timer_isr + 1                /* Vector 16: IRQ 0 (Timer alarm 0) */

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

    /* CRITICAL: Clear any pending interrupt for IRQ 0 FIRST */
    /* This prevents spurious interrupt from previous state */
    ldr r0, =NVIC_ICPR
    movs r1, #1                        /* Bit 0 = IRQ 0 */
    str r1, [r0]
    
    ldr r0, =msg_cleared
    bl print_string

    /* NOW enable IRQ 0 in NVIC */
    ldr r0, =NVIC_ISER
    movs r1, #1                        /* Bit 0 = IRQ 0 */
    str r1, [r0]
    
    ldr r0, =msg_enabled
    bl print_string

    /* Read current timer value */
    ldr r0, =TIMER_TIMELR
    ldr r1, [r0]
    
    ldr r0, =msg_time_read
    bl print_string
    
    /* Set alarm for 1000 us in future */
    ldr r2, =1000
    add r1, r2
    ldr r0, =TIMER_ALARM0
    str r1, [r0]
    
    ldr r0, =msg_alarm_set
    bl print_string
    
    /* Enable interrupts globally (CPSIE i) */
    cpsie i
    
    ldr r0, =msg_waiting
    bl print_string
    
    /* Wait for interrupt - ISR will execute when alarm fires */
    wfi
    
    /* Resumed after ISR returns */
    ldr r0, =msg_resumed
    bl print_string
    
    /* Done */
    ldr r0, =msg_done
    bl print_string
    
    bkpt #0
    b . - 2

/* Timer ISR - executed when alarm fires */
.thumb_func
.global timer_isr

timer_isr:
    /* This runs in handler mode with stacked context */
    
    ldr r0, =msg_in_isr
    bl print_string
    
    /* Clear the interrupt (write 1 to clear bit 0) */
    ldr r0, =TIMER_INTR
    movs r1, #1
    str r1, [r0]
    
    ldr r0, =msg_isr_cleared
    bl print_string
    
    /* Return from ISR - Cortex-M automatically unstacks context */
    /* The special return value in LR (0xFFFFFFF9) tells CPU to return to thread mode */
    bx lr

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
    .asciz "Timer Interrupt Test Starting\n"

msg_cleared:
    .asciz "NVIC IRQ 0 pending cleared\n"

msg_enabled:
    .asciz "NVIC IRQ 0 enabled\n"

msg_time_read:
    .asciz "Timer value read\n"

msg_alarm_set:
    .asciz "Alarm set for +1000us\n"

msg_waiting:
    .asciz "Waiting for interrupt (WFI)...\n"

msg_in_isr:
    .asciz ">>> TIMER ISR EXECUTING <<<\n"

msg_isr_cleared:
    .asciz "Interrupt cleared in ISR\n"

msg_resumed:
    .asciz "Resumed after ISR returned\n"

msg_done:
    .asciz "Timer Interrupt Test Complete!\n"

.align 2
