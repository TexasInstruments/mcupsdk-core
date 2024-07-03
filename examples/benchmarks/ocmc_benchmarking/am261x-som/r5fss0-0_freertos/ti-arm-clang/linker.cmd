

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
 --heap_size=16384
-e_vectors  /* This is the entry of the application, _vector MUST be placed starting address 0x0 */

/* This is the size of stack when R5 is in IRQ mode
 * In NORTOS,
 * - Here interrupt nesting is enabled
 * - This is the stack used by ISRs registered as type IRQ
 * In FreeRTOS,
 * - Here interrupt nesting is enabled
 * - This is stack that is used initally when a IRQ is received
 * - But then the mode is switched to SVC mode and SVC stack is used for all user ISR callbacks
 * - Hence in FreeRTOS, IRQ stack size is less and SVC stack size is more
 */
__IRQ_STACK_SIZE = 1024;
/* This is the size of stack when R5 is in IRQ mode
 * - In both NORTOS and FreeRTOS nesting is disabled for FIQ
 */
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 1024; /* This is the size of stack when R5 is in SVC mode */
__ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */
__UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */



SECTIONS
{
    .vectors  : {
    } > R5F_VECS   , palign(8)


    GROUP  :   {
    .text.hwi : {
    } palign(8)
    .text.cache : {
    } palign(8)
    .text.mpu : {
    } palign(8)
    .text.boot : {
    } palign(8)
    .text.main : {
    } palign(8)
    } > MSRAM


    GROUP  :   {
    .task_0 : {
    } palign(8)
    .task_1 : {
    } palign(8)
    .task_2 : {
    } palign(8)
    .task_3 : {
    } palign(8)
    .task_4 : {
    } palign(8)
    .task_5 : {
    } palign(8)
    .task_6 : {
    } palign(8)
    .task_7 : {
    } palign(8)
    .task_8 : {
    } palign(8)
    .task_9 : {
    } palign(8)
    .task_10 : {
    } palign(8)
    .task_11 : {
    } palign(8)
    .task_12 : {
    } palign(8)
    .task_13 : {
    } palign(8)
    .task_14 : {
    } palign(8)
    .task_15 : {
    } palign(8)
    .text : {
    } palign(8)
    } > FLASH


    GROUP  :   {
    .rodata : {
    } palign(8)
    .data : {
    } palign(8)
    .buf_0 : {
    } palign(8)
    .buf_1 : {
    } palign(8)
    .buf_2 : {
    } palign(8)
    .buf_3 : {
    } palign(8)
    .buf_4 : {
    } palign(8)
    .buf_5 : {
    } palign(8)
    .buf_6 : {
    } palign(8)
    .buf_7 : {
    } palign(8)
    .buf_8 : {
    } palign(8)
    .buf_9 : {
    } palign(8)
    .buf_10 : {
    } palign(8)
    .buf_11 : {
    } palign(8)
    .buf_12 : {
    } palign(8)
    .buf_13 : {
    } palign(8)
    .buf_14 : {
    } palign(8)
    .buf_15 : {
    } palign(8)
    .buf_cpy : {
    } palign(8)
    } > MSRAM


    GROUP  :   {
    .sysmem : {
    } palign(8)
    .stack : {
    } palign(8)
    } > MSRAM_STACK


    GROUP  :   {
    .irqstack : {
        . = . + __IRQ_STACK_SIZE;
    } align(8)
    RUN_START(__IRQ_STACK_START)
    RUN_END(__IRQ_STACK_END)
    .fiqstack : {
        . = . + __FIQ_STACK_SIZE;
    } align(8)
    RUN_START(__FIQ_STACK_START)
    RUN_END(__FIQ_STACK_END)
    .svcstack : {
        . = . + __SVC_STACK_SIZE;
    } align(8)
    RUN_START(__SVC_STACK_START)
    RUN_END(__SVC_STACK_END)
    .abortstack : {
        . = . + __ABORT_STACK_SIZE;
    } align(8)
    RUN_START(__ABORT_STACK_START)
    RUN_END(__ABORT_STACK_END)
    .undefinedstack : {
        . = . + __UNDEFINED_STACK_SIZE;
    } align(8)
    RUN_START(__UNDEFINED_STACK_START)
    RUN_END(__UNDEFINED_STACK_END)
    } > MSRAM_STACK


    GROUP  :   {
    .ARM.exidx : {
    } palign(8)
    .init_array : {
    } palign(8)
    .fini_array : {
    } palign(8)
    } > MSRAM_STACK


    GROUP  :   {
    .bss : {
    } palign(8)
    __llvm_prf_cnts : {
    } palign(8)
    __llvm_prf_bits : {
    } palign(8)
    } > MSRAM
    RUN_START(__BSS_START)
    RUN_END(__BSS_END)

    .TI.local  : {
    } >> R5F_TCMA | R5F_TCMB | MSRAM | FLASH

    .TI.onchip  : {
    } >> MSRAM | FLASH   , palign(8)

    .TI.offchip  : {
    } > FLASH   , palign(8)


}


MEMORY
{
    R5F_VECS   : ORIGIN = 0x0 , LENGTH = 0x40
    R5F_TCMA   : ORIGIN = 0x40 , LENGTH = 0x7FC0
    R5F_TCMB   : ORIGIN = 0x80000 , LENGTH = 0x8000
    MSRAM_STACK   : ORIGIN = 0x70000000 , LENGTH = 0x10000
    MSRAM   : ORIGIN = 0x70040000 , LENGTH = 0xE1000
    FLASH   : ORIGIN = 0x60300000 , LENGTH = 0x200000
    FLASH_RESERVED   : ORIGIN = 0x60000000 , LENGTH = 0x300000

    /* For memory Regions not defined in this core but shared by other cores with the current core */


}
