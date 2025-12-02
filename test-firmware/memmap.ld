/* 
 * Simplified RP2040 Linker Script for Bramble
 * Minimal configuration for testing
 */

MEMORY
{
    FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 2M
    RAM(rwx)  : ORIGIN = 0x20000000, LENGTH = 256k
}

SECTIONS
{
    /* Boot2 + Vector Table */
    .vectors :
    {
        KEEP(*(.vectors))
    } > FLASH

    /* Code and read-only data */
    .text :
    {
        *(.text*)
        *(.rodata*)
    } > FLASH

    /* Initialized data (copy to RAM at startup) */
    .data :
    {
        __data_start__ = .;
        *(.data*)
        __data_end__ = .;
    } > RAM AT > FLASH

    __data_load_addr__ = LOADADDR(.data);

    /* Uninitialized data (zero at startup) */
    .bss :
    {
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        __bss_end__ = .;
    } > RAM

    /* Discard unused sections */
    /DISCARD/ :
    {
        *(.note.GNU-stack)
    }
}
