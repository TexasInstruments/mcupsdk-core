__TI_STACK_SIZE = 8192;
__TI_HEAP_SIZE = 300000;
ENTRY(_vectors)
__IRQ_STACK_SIZE = 256;
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 4096;
__ABORT_STACK_SIZE = 256;
__UNDEFINED_STACK_SIZE = 256;
MEMORY
{
    R5F_VECS : ORIGIN = 0x00000000 , LENGTH = 0x00000040
    R5F_TCMA : ORIGIN = 0x00000040 , LENGTH = 0x00007FC0
    R5F_TCMB0 : ORIGIN = 0x41010000 , LENGTH = 0x00008000
    MSRAM : ORIGIN = 0x70080000 , LENGTH = 0x3000
    FLASH : ORIGIN = 0x60200000 , LENGTH = 0x100000
    DDR : ORIGIN = 0x80000000 , LENGTH = 0xFFF0000
    USER_SHM_MEM : ORIGIN = 0x701D0000, LENGTH = 0x00004000
    LOG_SHM_MEM : ORIGIN = 0x701D4000, LENGTH = 0x00004000
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0x701D8000, LENGTH = 0x00008000
}
SECTIONS
{
    .vectors : ALIGN (8) {} > R5F_VECS
    .text.hwi : ALIGN (8) {} > MSRAM
    .text.cache : ALIGN (8) {} > MSRAM
    .text.mpu : ALIGN (8) {} > MSRAM
    .text.boot : ALIGN (8) {} > MSRAM
    .text:abort : ALIGN (8) {} > MSRAM
    .text : ALIGN (8) {} > DDR
    .rodata : ALIGN (8) {} > DDR
    .data : ALIGN (8) {} > DDR
    .bss : {
        __bss_start__ = .;
        __BSS_START = .;
        *(.bss)
        *(.bss.*)
        . = ALIGN (8);
        __BSS_END = .;
        __bss_end__ = .;
        . = ALIGN (8);
    } > DDR
    .irqstack : ALIGN(16) {
        __IRQ_STACK_START = .;
        . = . + __IRQ_STACK_SIZE;
        __IRQ_STACK_END = .;
        . = ALIGN (8);
    } > DDR
    .fiqstack : ALIGN(16) {
        __FIQ_STACK_START = .;
        . = . + __FIQ_STACK_SIZE;
        __FIQ_STACK_END = .;
    } > DDR
    .svcstack : ALIGN(16) {
        __SVC_STACK_START = .;
        . = . + __SVC_STACK_SIZE;
        __SVC_STACK_END = .;
    } > DDR
    .abortstack : ALIGN(16) {
        __ABORT_STACK_START = .;
        . = . + __ABORT_STACK_SIZE;
        __ABORT_STACK_END = .;
    } > DDR
    .undefinedstack : ALIGN(16) {
        __UNDEFINED_STACK_START = .;
        . = . + __UNDEFINED_STACK_SIZE;
        __UNDEFINED_STACK_END = .;
    } > DDR
    .heap (NOLOAD) : {
        end = .;
        KEEP(*(.heap))
        . = . + __TI_HEAP_SIZE;
    } > DDR
    .stack (NOLOAD) : ALIGN(16) {
        __TI_STACK_BASE = .;
        KEEP(*(.stack))
        . = . + __TI_STACK_SIZE;
        __STACK_END = .;
    } > DDR
    .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    .bss.log_shared_mem (NOLOAD) : {} > LOG_SHM_MEM
    .bss.ipc_vring_mem (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM
    .enet_dma_mem (NOLOAD) : ALIGN(128) {
        *(.ENET_DMA_DESC_MEMPOOL)
        *(.ENET_DMA_RING_MEMPOOL)
        *(.ENET_DMA_PKT_MEMPOOL)
    } > DDR
    .bss:ENET_DMA_OBJ_MEM (NOLOAD) : ALIGN (128) {} > MSRAM
    .bss:ENET_DMA_PKT_INFO_MEMPOOL (NOLOAD) : ALIGN (128) {} > MSRAM
    .bss:ENET_ICSSG_OCMC_MEM (NOLOAD) : ALIGN (128) {} > MSRAM
}
