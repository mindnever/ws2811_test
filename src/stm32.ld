ENTRY(Reset_Handler)

/* Memory configuration for STM32F103C8 */

MEMORY
{
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
  FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 64K
/*  FLASH (rx) : ORIGIN = 0x8005000, LENGTH = 40K */
}

_end_stack = 0x20005000;

SECTIONS {
    . = ORIGIN(FLASH);

    vectors :
    {
        *(.vectors)
    } >FLASH

    .text :
    {
        *(.text)
        *(.rodata)
        *(.rodata*)
        _end_text = .;
    } >FLASH

    .data :
    {
        _start_data = .;
        *(.data)
        _end_data = .;
    } >RAM AT >FLASH

    . = ALIGN(4);

    .bss :
    {
        _start_bss = .;
        *(.bss)
        _end_bss = .;
    } >RAM

    . = ALIGN(4);

    __exidx_start = .;
    .ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >RAM
    __exidx_end = .;

    _start_stack = .;
}

_end = .;
