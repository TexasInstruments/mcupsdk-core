
/* make sure below retain is there in your linker command file, it keeps the vector table in the final binary */
--retain="*(.vectors)"
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


SECTIONS
{
    /* This has the M4F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > M4F_VECS
    .text:   {} palign(8) > M4F_IRAM     /* This is where code resides */

    .bss:    {} palign(8) > M4F_DRAM     /* This is where uninitialized globals go */
    RUN_START(__BSS_START)
    RUN_END(__BSS_END)

    .data:   {} palign(8) > M4F_DRAM     /* This is where initialized globals and static go */
    .rodata: {} palign(8) > M4F_DRAM     /* This is where const's go */
    .sysmem: {} palign(8) > M4F_IRAM     /* This is where the malloc heap goes */
    .stack:  {} palign(8) > M4F_IRAM     /* This is where the main() stack goes */

    GROUP {
        /* This is the resource table used by linux to know where the IPC "VRINGs" are located */
        .resource_table: {} palign(4096)
    } > DDR_0

    /* Sections needed for C++ projects */
    .ARM.exidx:     {} palign(8) > M4F_IRAM  /* Needed for C++ exception handling */
    .init_array:    {} palign(8) > M4F_IRAM  /* Contains function pointers called before main */
    .fini_array:    {} palign(8) > M4F_IRAM  /* Contains function pointers called after main */

    /* General purpose user shared memory */
    .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
    .bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM
    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) : {} > IPC_VRING_MEM
}

MEMORY
{
    M4F_VECS : ORIGIN = 0x00000000 , LENGTH = 0x00000200
    M4F_IRAM : ORIGIN = 0x00000200 , LENGTH = 0x0002FE00
    M4F_DRAM : ORIGIN = 0x00030000 , LENGTH = 0x00010000

    /* when using multi-core application's i.e more than one R5F/M4F active, make sure
     * this memory does not overlap with R5F's
     */
    /* Resource table must be placed at the start of DDR_0 when M4 core is early booting with Linux */
    DDR_0       : ORIGIN = 0xA4100000 , LENGTH = 0x1000

    USER_SHM_MEM    : ORIGIN = 0xA5000000, LENGTH = 0x80
    LOG_SHM_MEM     : ORIGIN = 0xA5000000 + 0x80, LENGTH = 0x00004000 - 0x80
    IPC_VRING_MEM   : ORIGIN = 0xA5004000, LENGTH = 0x0000C000
}
