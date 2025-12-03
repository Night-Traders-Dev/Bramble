/* boot.S - Minimal RP2040 vector table */

.section .vectors, "a", %progbits
.align 2
.global _vectors

_vectors:
    .word 0x20042000          /* Initial SP (top of SRAM) */
    .word Reset_Handler + 1   /* Reset vector (Thumb bit set) */
    .word Reset_Handler + 1   /* NMI */
    .word Reset_Handler + 1   /* HardFault */
    /* Fill remaining vectors */
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF
    .word 0xFFFFFFFF

/* Reset handler */
.section .text
.global Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
    bl main          /* Call C main */
    b .              /* Loop forever */

.size Reset_Handler, . - Reset_Handler
