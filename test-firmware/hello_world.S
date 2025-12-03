/* hello_world.S - Final working version */

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
    push {r4, lr}
    ldr r0, =message
    ldr r1, =0x40034000
    ldr r2, =0x40034018

1:  ldrb r3, [r0]
    cmp r3, #0
    beq 2f

wait:
    ldr r4, [r2]
    movs r5, #0x20
    tst r4, r5
    bne wait

    str r3, [r1]
    add r0, #1
    b 1b

2:  pop {r4, pc}
    bkpt #0      /* Clean halt */

.section .rodata
message:
    .ascii "Hello from ASM!"
    .byte 0      /* Explicit null terminator */
    .align 2

/* No .size directives needed */
