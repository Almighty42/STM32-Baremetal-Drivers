/* Define memory regions. */
MEMORY
{
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 512K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

/* Enforce that the program provide a vector table. */
EXTERN (g_pfnVectors)

/* Define the entry point of the final executable. */
ENTRY(Reset_Handler)

/* Define ELF sections. */
SECTIONS
{
    .text : {
        *(.vectors)     /* Vector table */
        *(.text*)       /* Program code */
        . = ALIGN(4);
        *(.rodata*)     /* Read-only data */
        . = ALIGN(4);
    } >rom

    . = ALIGN(4);
    _etext = .;

    /* ram, but not cleared on reset, eg boot/app comms */
    .noinit (NOLOAD) : {
        *(.noinit*)
    } >ram
    . = ALIGN(4);

    .data : {
        _sdata = .;
        *(.data*)       /* Read-write initialized data */
        *(.ramtext*)    /* "text" functions to run in ram */
        . = ALIGN(4);
        _edata = .;
    } >ram AT >rom
    _sidata = LOADADDR(.data);

    .bss : {
        _sbss = .;
        *(.bss*)        /* Read-write zero initialized data */
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >ram

    /DISCARD/ : { *(.eh_frame) }

    . = ALIGN(4);
    end = .;
    _estack = ORIGIN(ram) + LENGTH(ram);
}
