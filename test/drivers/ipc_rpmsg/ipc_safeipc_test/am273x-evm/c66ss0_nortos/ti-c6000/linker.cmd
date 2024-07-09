/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=16384
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=32768
--retain=_vectors

SECTIONS
{
    /* hard addresses forces vecs to be allocated there */
    .text:vectors: {. = align(1024); } > 0x00800000
    .text:      {} > DSS_L2
    .const:     {} > DSS_L2
    .cinit:     {} > DSS_L2
    .data:      {} > DSS_L2
    .stack:     {} > DSS_L2
    .switch:    {} > DSS_L2
    .cio:       {} > DSS_L2
    .sysmem:    {} > DSS_L2
    .fardata:   {} > DSS_L2
    .far:       {} > DSS_L2

    /* These should be grouped together to avoid STATIC_BASE relative relocation linker error */
    GROUP {
        .rodata:    {}
        .bss:       {}
        .neardata:  {}
    } > DSS_L2

    /* Sections needed for C++ projects */
    GROUP {
        .c6xabi.exidx:  {} palign(8)   /* Needed for C++ exception handling */
        .init_array:    {} palign(8)   /* Contains function pointers called before main */
        .fini_array:    {} palign(8)   /* Contains function pointers called after main */
    } > DSS_L2

    /* any data buffer needed to be put in L3 can be assigned this section name */
    .bss.dss_l3 {} > DSS_L3

    /* General purpose user shared memory, used in some examples */
    .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
    .bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM
    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM
    /* this is used only when Secure IPC is enabled */
    .bss.sipc_hsm_queue_mem   (NOLOAD) : {} > MAILBOX_HSM
    .bss.sipc_secure_host_queue_mem   (NOLOAD) : {} > MAILBOX_R5F
}

MEMORY
{
    DSS_L2:   ORIGIN = 0x800000, LENGTH = 0x60000
    DSS_L3:   ORIGIN = 0x88000000, LENGTH = 0x00390000

    /* shared memories that are used by RTOS/NORTOS cores */
    /* On C66,
     * - make sure these are which mapped as non-cache in MAR bits
     */
    USER_SHM_MEM            : ORIGIN = 0xC02E8000, LENGTH = 0x00004000
    LOG_SHM_MEM             : ORIGIN = 0xC02EC000, LENGTH = 0x00004000
    /* MSS mailbox memory is used as shared memory, we dont use bottom 32*6 bytes, since its used as SW queue by ipc_notify */
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0xC5000000, LENGTH = 0x1F40
    MAILBOX_HSM:    ORIGIN = 0x44000000 , LENGTH = 0x000003CE
    MAILBOX_R5F:    ORIGIN = 0x44000400 , LENGTH = 0x000003CE
}
