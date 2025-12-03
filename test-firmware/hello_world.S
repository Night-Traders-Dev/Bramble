/* hello_world.S - Simplified Cortex-M0+ */

.section .vectors, "a", %progbits
.align 2
.global _start

_start:
    .word 0x20042000          /* SP */
    .word reset_handler + 1   /* Reset */
    .rept 14
    .word 0xFFFFFFFF
    .endr

.section .text
.global reset_handler
.thumb_func

reset_handler:
    push {r4, lr}
    ldr r0, =message
    ldr r1, =0x40034000
    ldr r2, =0x40034018

loop:
    ldrb r3, [r0]
    cmp r3, #0
    beq done

wait:
    ldr r4, [r2]
    movs r5, #0x20
    tst r4, r5
    bne wait

    str r3, [r1]
    add r0, #1
    b loop

done:
    pop {r4, pc}

.section .rodata
message:
    .asciz "Hello from ASM!"
