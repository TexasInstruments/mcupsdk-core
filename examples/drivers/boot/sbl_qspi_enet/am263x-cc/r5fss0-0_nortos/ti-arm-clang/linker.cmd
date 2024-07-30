
--stack_size=16384
--heap_size=32768
-e_vectors_sbl  /* for SBL make sure to set entry point to _vectors_sbl */

__IRQ_STACK_SIZE = 4096;
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 256;
__ABORT_STACK_SIZE = 256;
__UNDEFINED_STACK_SIZE = 256;

SECTIONS
{
    .sbl_init_code: palign(8), fill=0xabcdabcd
    {
       *(.vectors) /* IVT is put at the beginning of the section */
       . = align(8);
    } load=MSRAM_VECS, run=R5F_VECS
    .vectors:{} palign(8) > MSRAM_VECS
    GROUP {
        .text:   {} palign(8)
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)
        .text.boot: palign(8)
        .data:   {} palign(8)
        .rodata: {} palign(8)
    } > MSRAM_0
    .bss:    {} palign(8) > MSRAM_0
    RUN_START(__BSS_START)
    RUN_END(__BSS_END)
    .sysmem: {} palign(8) > MSRAM_0
    .stack:  {} palign(8) > MSRAM_0
    GROUP {
        .irqstack: {. = . + __IRQ_STACK_SIZE;} align(8)
        RUN_START(__IRQ_STACK_START)
        RUN_END(__IRQ_STACK_END)
        .fiqstack: {. = . + __FIQ_STACK_SIZE;} align(8)
        RUN_START(__FIQ_STACK_START)
        RUN_END(__FIQ_STACK_END)
        .svcstack: {. = . + __SVC_STACK_SIZE;} align(8)
        RUN_START(__SVC_STACK_START)
        RUN_END(__SVC_STACK_END)
        .abortstack: {. = . + __ABORT_STACK_SIZE;} align(8)
        RUN_START(__ABORT_STACK_START)
        RUN_END(__ABORT_STACK_END)
        .undefinedstack: {. = . + __UNDEFINED_STACK_SIZE;} align(8)
        RUN_START(__UNDEFINED_STACK_START)
        RUN_END(__UNDEFINED_STACK_END)
    } > MSRAM_0

    /* HSMRt image section */
    .rodata.hsmrt : {} palign(8) > MSRAM_HSMRT

    /* this is used only when Secure IPC is enabled */
    .bss.sipc_hsm_queue_mem   (NOLOAD) : {} > MAILBOX_HSM
    .bss.sipc_r5f_queue_mem   (NOLOAD) : {} > MAILBOX_R5F
}

MEMORY
{
    R5F_VECS :  ORIGIN = 0x00000000 , LENGTH = 0x00000100
    R5F_TCMA :  ORIGIN = 0x00000100 , LENGTH = 0x00007F00
    R5F_TCMB0:  ORIGIN = 0x41010000 , LENGTH = 0x00008000
    MSRAM_VECS: ORIGIN = 0x70002000 , LENGTH = 0x00000100
    MSRAM_0  :  ORIGIN = 0x70002100 , LENGTH = 0x60000
    /* 256 KB memory section HSMRt */
    MSRAM_HSMRT  :  ORIGIN = 0x70062100 , LENGTH = 0x40000
    MAILBOX_HSM:    ORIGIN = 0x44000000 , LENGTH = 0x000003CE
    MAILBOX_R5F:    ORIGIN = 0x44000400 , LENGTH = 0x000003CE
}
