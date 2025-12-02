### memmap.ld (Linker script)

/* Memory layout for RP2040 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x10000100, LENGTH = 2048K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 264K
}

ENTRY(_start)

SECTIONS
{
    .vectors : {
        KEEP(*(.vectors))
    } > FLASH
    
    .text : {
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
    } > FLASH
    
    .data : {
        *(.data)
        *(.data*)
    } > RAM AT > FLASH
    
    .bss : {
        *(.bss)
        *(.bss*)
        *(COMMON)
    } > RAM
    
    /* Stack grows downward from end of RAM */
    PROVIDE(_stack_top = ORIGIN(RAM) + LENGTH(RAM));
}
