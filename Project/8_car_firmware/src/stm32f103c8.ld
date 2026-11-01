/* STM32F103 中容量：Flash @0x08000000，20 KB RAM @0x20000000。
 *   C8T6 (LQFP48) = 64K  -> 本文件
 *   RBT6 (LQFP64, Keil MCBSTM32 板上那颗) = 128K -> stm32f103rb.ld
 * 用 `make LDSCRIPT=stm32f103rb.ld` 切换。 */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text :
    {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* 栈从 _estack（RAM 顶端）向下生长，不需要单独划段 */
}
