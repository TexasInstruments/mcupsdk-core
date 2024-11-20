/* Stack and Heap Size Definitions */

__TI_STACK_SIZE = 16384;
__TI_HEAP_SIZE = 32768;

ENTRY(_vectors_sbl)

/* Stack Sizes */
__IRQ_STACK_SIZE = 4096;
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 256;
__ABORT_STACK_SIZE = 256;
__UNDEFINED_STACK_SIZE = 256;

MEMORY
{
    R5F_VECS : ORIGIN = 0x00000000 , LENGTH = 0x00000040
    R5F_TCMA : ORIGIN = 0x00000040 , LENGTH = 0x00007FC0
    R5F_TCMB0 : ORIGIN = 0x41010000 , LENGTH = 0x00008000
    MSRAM_VECS : ORIGIN = 0x70000000 , LENGTH = 0x100
    MSRAM_0  : ORIGIN = 0x70000100 , LENGTH = 0x50000 - 0x100
    MSRAM_1  : ORIGIN = 0x70050000 , LENGTH = 0x30000
    APPIMAGE   : ORIGIN = 0x82000000 , LENGTH = 0x800000 

}

SECTIONS
{
    .vectors : ALIGN(8) {} > MSRAM_VECS

    .text       : ALIGN(8) {} > MSRAM_0
    .text.hwi   : ALIGN(8) {} > MSRAM_0
    .text.cache : ALIGN(8) {} > MSRAM_0
    .text.mpu   : ALIGN(8) {} > MSRAM_0
    .text.boot  : ALIGN(8) {} > MSRAM_0
    .data       : ALIGN(8) {} > MSRAM_0
    .rodata     : ALIGN(8) {} > MSRAM_0

    .app (NOLOAD) : { KEEP(*(.app)) } > APPIMAGE
    
    .bss : {
        __bss_start__ = .;
        __BSS_START = .;
        *(.bss)
        *(.bss.*)
        . = ALIGN (8);
        __BSS_END = .;
        __bss_end__ = .;
        . = ALIGN (8);
    } > MSRAM_1

    .irqstack : ALIGN(16) {
        __IRQ_STACK_START = .;
        . = . + __IRQ_STACK_SIZE;
        __IRQ_STACK_END = .;
        . = ALIGN (8);
    } > MSRAM_1
    .fiqstack : ALIGN(16) {
        __FIQ_STACK_START = .;
        . = . + __FIQ_STACK_SIZE;
        __FIQ_STACK_END = .;
    } > MSRAM_1
    .svcstack : ALIGN(16) {
        __SVC_STACK_START = .;
        . = . + __SVC_STACK_SIZE;
        __SVC_STACK_END = .;
    } > MSRAM_1
    .abortstack : ALIGN(16) {
        __ABORT_STACK_START = .;
        . = . + __ABORT_STACK_SIZE;
        __ABORT_STACK_END = .;
    } > MSRAM_1
    .undefinedstack : ALIGN(16) {
        __UNDEFINED_STACK_START = .;
        . = . + __UNDEFINED_STACK_SIZE;
        __UNDEFINED_STACK_END = .;
    } > MSRAM_1

    .heap (NOLOAD) : {
        end = .;
        KEEP(*(.heap))
        . = . + __TI_HEAP_SIZE;
    } > MSRAM_1

    .stack (NOLOAD) : ALIGN(16) {
        __TI_STACK_BASE = .;
        KEEP(*(.stack))
        . = . + __TI_STACK_SIZE;
        __STACK_END = .;
    } > MSRAM_1


}





