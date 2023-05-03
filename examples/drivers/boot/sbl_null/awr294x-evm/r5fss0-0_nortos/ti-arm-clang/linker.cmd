
/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=0x2000
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=0x2000
-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */

/* This is the size of stack when R5 is in IRQ mode
 * In NORTOS,
 * - Here interrupt nesting is disabled as of now
 * - This is the stack used by ISRs registered as type IRQ
 * In FreeRTOS,
 * - Here interrupt nesting is enabled
 * - This is stack that is used initally when a IRQ is received
 * - But then the mode is switched to SVC mode and SVC stack is used for all user ISR callbacks
 * - Hence in FreeRTOS, IRQ stack size is less and SVC stack size is more
 */
__IRQ_STACK_SIZE = 4096;
/* This is the size of stack when R5 is in IRQ mode
 * - In both NORTOS and FreeRTOS nesting is disabled for FIQ
 */
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 256; /* This is the size of stack when R5 is in SVC mode */
__ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */
__UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */

SECTIONS
{
    .sbl_init_code: palign(8), fill=0xabcdabcd
    {
       *(.vectors) /* IVT is put at the beginning of the section */
       . = align(8);
       *(.text.boot)
       . = align(8);
    } load=MSS_L2_RSVD, run=R5F_VECS

    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > MSS_L2_RSVD
    .text.boot:{} palign(8) > MSS_L2_RSVD

    /* This has the R5F boot code until MPU is enabled,  this MUST be at a address < 0x80000000
     * i.e this cannot be placed in DDR
     */
    GROUP {
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)

        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
    } > MSS_L2

    /* This is rest of code. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .text:   {} palign(8)   /* This is where code resides */
        .rodata: {} palign(8)   /* This is where const's go */
    } > MSS_L2

    /* This is rest of initialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {

        .data:   {} palign(8)   /* This is where initialized globals and static go */
    } > MSS_L2


    /* This is rest of uninitialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
        .sysmem: {} palign(8)   /* This is where the malloc heap goes */
        .stack:  {} palign(8)   /* This is where the main() stack goes */
    } > MSS_L2

    /* This is where the stacks for different R5F modes go */
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
    } > MSS_L2

    /* HSMRt image section */
    .rodata.hsmrt : {} palign(8) > MSRAM_HSMRT

    /* this is used only when Secure IPC is enabled */
    .bss.sipc_hsm_queue_mem   (NOLOAD) : {} > MAILBOX_HSM
    .bss.sipc_r5f_queue_mem   (NOLOAD) : {} > MAILBOX_R5F

    /* any data buffer needed to be put in L3 can be assigned this section name */
    .bss.dss_l3 {} > DSS_L3

    /* General purpose user shared memory, used in some examples */
    .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
    .bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM
    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM
}

#define SBL_INIT_CODE_SIZE          640

MEMORY
{
    R5F_VECS  : ORIGIN = 0x00000000 , LENGTH = SBL_INIT_CODE_SIZE
    R5F_TCMA  : ORIGIN = SBL_INIT_CODE_SIZE , LENGTH = (0x00008000 - SBL_INIT_CODE_SIZE)
    R5F_TCMB  : ORIGIN = 0x00080000 , LENGTH = 0x00008000

    /* when using multi-core application's i.e more than one R5F active, make sure
     * this memory does not overlap with other R5F's
     */
    MSS_L2_RSVD : ORIGIN = 0x10200000 , LENGTH = SBL_INIT_CODE_SIZE
    MSS_L2      : ORIGIN = (0x10200000 + SBL_INIT_CODE_SIZE) , LENGTH = (0x20000 - SBL_INIT_CODE_SIZE)

    /* This is typically used to hold data IO buffers from accelerators like CSI, HWA, DSP */
    DSS_L3:   ORIGIN = 0x88000000, LENGTH = 0x00200000

    /* shared memories that are used by RTOS/NORTOS cores */
    /* On R5F,
     * - make sure there are MPU entries which maps below regions as non-cache
     */
    USER_SHM_MEM            : ORIGIN = 0x102E8000, LENGTH = 0x00004000
    LOG_SHM_MEM             : ORIGIN = 0x102EC000, LENGTH = 0x00004000
    /* MSS mailbox memory is used as shared memory, we dont use bottom 32*6 bytes, since its used as SW queue by ipc_notify */
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0xC5000000, LENGTH = 0x1F40

    MSRAM_HSMRT  :  ORIGIN = (0x10200000 + 0x20000) , LENGTH = 0x20000
    MAILBOX_HSM:    ORIGIN = 0x44000000 , LENGTH = 0x000003CE
    MAILBOX_R5F:    ORIGIN = 0x44000400 , LENGTH = 0x000003CE
}
