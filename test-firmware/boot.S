.syntax unified
.cpu cortex-m0plus
.thumb

.section .vectors, "ax"
.align 2

/* Vector table */
.word 0x20042000           /* Initial Stack Pointer (top of RAM) */
.word _start + 1           /* Reset vector (Thumb mode bit set) */

.section .text
.align 2
.thumb_func

.global _start
_start:
    /* Stack pointer already initialized by vector table */
    
    /* Call main function */
    movs r0, #0            /* Clear r0 */
    movs r1, #0            /* Clear r1 */
    movs r2, #0            /* Clear r2 */
    movs r3, #0            /* Clear r3 */
    
    /* Branch to main (PC-relative) */
    ldr  r4, =main
    bx   r4

halt:
    bkpt #0                /* Breakpoint for emulator */
    b    halt              /* Loop forever */

.size _start, . - _start
