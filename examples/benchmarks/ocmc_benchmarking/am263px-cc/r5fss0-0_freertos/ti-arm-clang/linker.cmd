
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
#define XIP_MCU1_0_START 0x60300000

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
        .text.main: palign(8)
        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
    } > MSRAM

    cio > MSRAM
    {
        -llibsysbm.a<trgmsg.c.obj> (.text)
    }

    .TI.local   : {} >> R5F_TCMA | R5F_TCMB0 | MSRAM | FLASH
    .TI.onchip  : {} >> MSRAM | FLASH
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
        .text:    {} palign(8)
        .rodata:  {} palign(8)
    } > FLASH

/*
    GROUP  :   {
        .task_0 : {} align(8)
        .task_1 : {} align(8)
        .task_2 : {} align(8)
        .task_3 : {} align(8)
        .task_4 : {} align(8)
        .task_5 : {} align(8)
        .task_6 : {} align(8)
        .task_7 : {} align(8)
        .task_8 : {} align(8)
        .task_9 : {} align(8)
        .task_10 : {} align(8)
        .task_11 : {} align(8)
        .task_12 : {} align(8)
        .task_13 : {} align(8)
        .task_14 : {} align(8)
        .task_15 : {} align(8)
        .text : {} align(8)
        .rodata : {} align(8)
    } > FLASH
*/

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
        .sysmem: {} palign(8)   /* This is where the malloc heap goes */
        .stack:  {} palign(8)   /* This is where the main() stack goes */
    } > MSRAM_STACK

    GROUP
    {
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        __llvm_prf_cnts: {} palign(8)
        __llvm_prf_bits: {} palign(8)
    }>MSRAM
    RUN_START(__BSS_START)
    RUN_END(__BSS_END)


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
}


MEMORY
{
    R5F_VECS   : ORIGIN = 0x0 , LENGTH = 0x40
    R5F_TCMA   : ORIGIN = 0x40 , LENGTH = 0x7FC0
    R5F_TCMB0   : ORIGIN = 0x80000 , LENGTH = 0x8000
    MSRAM_STACK   : ORIGIN = 0x70000000 , LENGTH = 0x60000
    NON_CACHE_MEM   : ORIGIN = 0x70060000 , LENGTH = 0x8000
    MSRAM   : ORIGIN = 0x70068000 , LENGTH = 0x160000
    FLASH   : ORIGIN = 0x60300000 , LENGTH = 0x150000

}


