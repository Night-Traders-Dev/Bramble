/* Linker Script for Bramble RP2040 Emulator
 * 
 * This linker script ensures:
 * 1. Vector table placed at 0x10000100 (after boot2)
 * 2. Reset handler is immediately after vector table
 * 3. All sections properly aligned
 * 4. Memory regions defined correctly
 */

MEMORY
{
    /* Flash (XIP) - Code and read-only data */
    /* Boot2 occupies 0x10000000-0x100000FF (256 bytes) */
    /* User app starts at 0x10000100 */
    FLASH (rx) : ORIGIN = 0x10000100, LENGTH = 2M - 256
    
    /* RAM - Data and stack */
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 264K
}

SECTIONS
{
    /* Vector table (must be first in output) */
    .vectors :
    {
        KEEP(*(.vectors))
        . = ALIGN(4);
    } > FLASH

    /* Code and read-only data */
    .text :
    {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* Initialized data (in flash, copied to RAM at startup) */
    .data :
    {
        _data_start = .;
        *(.data*)
        _data_end = .;
        . = ALIGN(4);
    } > RAM AT > FLASH
    
    _data_lma = LOADADDR(.data);

    /* Uninitialized data (RAM only, zeroed at startup) */
    .bss :
    {
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        _bss_end = .;
        . = ALIGN(4);
    } > RAM

    /* Stack and heap (remainder of RAM) */
    .heap :
    {
        _heap_start = .;
        . = . + 4K;  /* Reserve 4K for heap */
        _heap_end = .;
    } > RAM

    /* Stack grows downward from end of RAM */
    _stack_top = ORIGIN(RAM) + LENGTH(RAM);

    /* Debug symbols */
    .debug_info 0 : { *(.debug_info) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_line 0 : { *(.debug_line) }
    .debug_loc 0 : { *(.debug_loc) }
}

/* Entry point */
ENTRY(reset_handler)

/* Symbols for startup code */
EXTERN(reset_handler)
