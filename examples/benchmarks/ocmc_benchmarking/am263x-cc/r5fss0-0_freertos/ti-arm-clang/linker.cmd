
/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=65536
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=131072
-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */

/* This is the size of stack when R5 is in IRQ mode
 * In NORTOS,
 * - Here interrupt nesting is enabled
 * - This is the stack used by ISRs registered as type IRQ
 * In FreeRTOS,
 * - Here interrupt nesting is disabled
 * - This is stack that is used initally when a IRQ is received
 * - But then the mode is switched to SVC mode and SVC stack is used for all user ISR callbacks
 * - Hence in FreeRTOS, IRQ stack size is less and SVC stack size is more
 */
__IRQ_STACK_SIZE =16384;
/* This is the size of stack when R5 is in IRQ mode
 * - In both NORTOS and FreeRTOS nesting is disabled for FIQ
 */
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 16384; /* This is the size of stack when R5 is in SVC mode */
__ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */
__UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */


#define TASK_SIZE	0x8000
#define TEXT_SIZE	0x26000
#define BUF_SIZE	0x4000

#define MSRAM_START	0x70080000
#define XIP_MCU1_0_START 0x60100000

SECTIONS
{
    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > R5F_VECS

    /* This has the R5F boot code until MPU is enabled,  this MUST be at a address < 0x80000000
     * i.e this cannot be placed in DDR
     */
    GROUP {
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)
        .text.boot: palign(8)
        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
    } > MSRAM

    .TI.local   : {} >> R5F_TCMA | R5F_TCMB0 | MSRAM
    .TI.onchip  : {} >> MSRAM
    .TI.offchip : {} > FLASH

    /* This is rest of code. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .task_0:  {} palign(8)
        .task_1:  {} palign(8)
        .task_2:  {} palign(8)
        .task_3:  {} palign(8)
        .task_4:  {} palign(8)
        .task_5:  {} palign(8)
        .task_6:  {} palign(8)
        .task_7:  {} palign(8)
        .task_8:  {} palign(8)
        .task_9:  {} palign(8)
        .task_10: {} palign(8)
        .task_11: {} palign(8)
        .task_12: {} palign(8)
        .task_13: {} palign(8)
        .task_14: {} palign(8)
        .task_15: {} palign(8)
        .text:    {} palign(8)   /* This is where code resides */
        .rodata:  {} palign(8)   /* This is where const's go */
    } > MSRAM

    /* This is rest of initialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .buf_0:  {} palign(4)
        .buf_1:  {} palign(4)
        .buf_2:  {} palign(4)
        .buf_3:  {} palign(4)
        .buf_4:  {} palign(4)
        .buf_5:  {} palign(4)
        .buf_6:  {} palign(4)
        .buf_7:  {} palign(4)
        .buf_8:  {} palign(4)
        .buf_9:  {} palign(4)
        .buf_10: {} palign(4)
        .buf_11: {} palign(4)
        .buf_12: {} palign(4)
        .buf_13: {} palign(4)
        .buf_14: {} palign(4)
        .buf_15: {} palign(4)
        .buf_cpy:{} palign(4)
        .data:   {} palign(8)   /* This is where initialized globals and static go */
    } > MSRAM

    /* This is rest of uninitialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
        .sysmem: {} palign(8)   /* This is where the malloc heap goes */
        .stack:  {} palign(8)   /* This is where the main() stack goes */
    } > MSRAM_STACK

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
    } > MSRAM

    /* Sections needed for C++ projects */
    GROUP {
        .ARM.exidx:  {} palign(8)   /* Needed for C++ exception handling */
        .init_array: {} palign(8)   /* Contains function pointers called before main */
        .fini_array: {} palign(8)   /* Contains function pointers called after main */
    } > MSRAM

    /* General purpose user shared memory, used in some examples */
    .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
    .bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM
    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM
    /* General purpose non cacheable memory, used in some examples */
    .bss.nocache (NOLOAD) : {} > NON_CACHE_MEM
}

/*
NOTE: Below memory is reserved for DMSC usage
 - During Boot till security handoff is complete
   0x701E0000 - 0x701FFFFF (128KB)
 - After "Security Handoff" is complete (i.e at run time)
   0x701F4000 - 0x701FFFFF (48KB)

 Security handoff is complete when this message is sent to the DMSC,
   TISCI_MSG_SEC_HANDOVER

 This should be sent once all cores are loaded and all application
 specific firewall calls are setup.
*/

MEMORY
{
    R5F_VECS  : ORIGIN = 0x00000000 , LENGTH = 0x00000040
    R5F_TCMA  : ORIGIN = 0x00000040 , LENGTH = 0x00007FC0
    R5F_TCMB0 : ORIGIN = 0x00080000 , LENGTH = 0x00008000
    MSRAM_STACK : ORIGIN = 0x70000000 , LENGTH = 0x60000

    /* memory segment used to hold CPU specific non-cached data, MAKE to add a MPU entry to mark this as non-cached */
    NON_CACHE_MEM : ORIGIN = 0x70060000 , LENGTH = 0x8000

    MSRAM : ORIGIN = MSRAM_START, LENGTH = 0x150000

    FLASH : ORIGIN = XIP_MCU1_0_START, LENGTH = 0x150000

    /* shared memory segments */
    /* On R5F,
     * - make sure there is a MPU entry which maps below regions as non-cache
     */
    USER_SHM_MEM            : ORIGIN = 0x701D0000, LENGTH = 0x180
    LOG_SHM_MEM             : ORIGIN = 0x701D0000 + 0x180, LENGTH = 0x00004000 - 0x180
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0x701D4000, LENGTH = 0x0000C000
}


