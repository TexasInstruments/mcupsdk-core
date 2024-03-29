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
--retain=_InterruptVectorTable

#define DEFAULT_SECTION_ALIGNMENT   ( 1K )

#define L2_CODE_START               ( 0x800000 )
#define L2_CODE_SIZE                ( L2_RO_DATA_START - L2_CODE_START )
#define L2_RO_DATA_START            ( 0x00820000 )
#define L2_RO_DATA_SIZE             ( L2_RW_DATA_START - L2_RO_DATA_START )
#define L2_RW_DATA_START            ( 0x00824000 )
#define L2_RW_DATA_SIZE             ( 0x60000 - L2_CODE_SIZE - L2_RO_DATA_SIZE)

SECTIONS
{
    /* hard addresses forces vecs to be allocated there */
    .priv_code          palign( DEFAULT_SECTION_ALIGNMENT ),
                        LOAD_START( lnkKernelFuncStartAddr ),
                        LOAD_END( lnkKernelFuncEndAddr ) :
    {
        *(.interrupt_vectors)
        *(.KERNEL_FUNCTION)
    } > L2_CODE_START

    .unpriv_code        palign( DEFAULT_SECTION_ALIGNMENT ),
                        LOAD_START( lnkStartFlashAddress ) :
    {
        *(.text)
    } > L2_CODE_SRAM

    .unpriv_rodata      palign( DEFAULT_SECTION_ALIGNMENT ) :
    {
        *(.const)
        *(.switch)
    } > L2_READ_ONLY_SRAM

    .rodata             palign( DEFAULT_SECTION_ALIGNMENT )     : {} > L2_READ_ONLY_SRAM
    .cinit              LOAD_END( lnkEndFlashAddress )          : {} > L2_READ_ONLY_SRAM

    .KERNEL_DATA        palign( DEFAULT_SECTION_ALIGNMENT ),
                        LOAD_START( lnkKernelDataStartAddr ),
                        LOAD_END( lnkKernelDataEndAddr )        : {} > L2_RW_DATA_START

    GROUP               palign( DEFAULT_SECTION_ALIGNMENT ),
                        LOAD_START( lnkUnprivilegedDataStartAddr ),
                        LOAD_END( lnkUnprivilegedDataEndAddr )
    {
    .bss:
    .neardata:
    .cio:
    .data:
    .sysmem:
    .fardata:
    .far:
    } > L2_READ_WRITE_SRAM

    .stack              palign( DEFAULT_SECTION_ALIGNMENT ),
                        LOAD_START( lnkStacksStartAddr )        : {} > L2_READ_WRITE_SRAM


/* Additional sections settings     */

    /* These configuration settings are for the SafeRTOS demonstration
     * project. They are included as a demonstration of how task data can be
     * grouped together into known locations, therefore enabling MPU regions to
     * be defined. */


    __idle_hook_data__  palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkIdleHookDataStartAddr )           : {} > L2_READ_WRITE_SRAM

    __led_task_data__   palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkLEDFlashTestDataStartAddr )       : {} > L2_READ_WRITE_SRAM

    __notified_task_data__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkTaskNotifyDataStartAddr )         : {} > L2_READ_WRITE_SRAM

    __timer_demo_task_data__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkStartTimerTestData )              : {} > L2_READ_WRITE_SRAM

    __com_test_data__   palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkComTestDataStartAddr )            : {} > L2_READ_WRITE_SRAM

    __block_tim_data__  palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkBlockTimeTestDataStartAddr )      : {} > L2_READ_WRITE_SRAM

    __block_q_data__    palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkBlockQueueTestDataStartAddr )     : {} > L2_READ_WRITE_SRAM

    __dynamic_task_data__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkDynamicTestDataStartAddr )        : {} > L2_READ_WRITE_SRAM

    __poll_q_data__     palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkPollQTestDataStartAddr )          : {} > L2_READ_WRITE_SRAM

    __binary_semaphore_task_data__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkSemaphoreTestDataStartAddr )      : {} > L2_READ_WRITE_SRAM

    __maths_test_data__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkMathsTestDataStartAddr )          : {} > L2_READ_WRITE_SRAM

    __create_delete_data__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkCreateDeleteDemoDataStartAddr )   : {} > L2_READ_WRITE_SRAM

    __counting_semaphore_task_data__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkCountSemTestDataStartAddr )       : {} > L2_READ_WRITE_SRAM

    __rec_mutex_data__  palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkRecMutexDataStartAddr )           : {} > L2_READ_WRITE_SRAM

    __streambuffer_data__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkStreamBufferDataStartAddr )       : {} > L2_READ_WRITE_SRAM

	__streambuffer_data_common__ palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkStreamBufferDataCommonStartAddr ) : {} > L2_READ_WRITE_SRAM

	__evt_mplx_data__   palign( DEFAULT_SECTION_ALIGNMENT ),
                        START( lnkMplxDataStartAddr )               : {} > L2_READ_WRITE_SRAM

    /* Sections needed for C++ projects */
    GROUP {
        .c6xabi.exidx:  {} palign(8)   /* Needed for C++ exception handling */
        .init_array:    {} palign(8)   /* Contains function pointers called before main */
        .fini_array:    {} palign(8)   /* Contains function pointers called after main */
    } > L2_READ_WRITE_SRAM

    /* any data buffer needed to be put in L3 can be assigned this section name */
    .bss.dss_l3 {} > DSS_L3

    /* General purpose user shared memory, used in some examples */
    .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
    .bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM
    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM
}

MEMORY
{
    L2_CODE_SRAM (RIX)          :   o = L2_CODE_START,      l = L2_CODE_SIZE
    L2_READ_ONLY_SRAM (R)       :   o = L2_RO_DATA_START,   l = L2_RO_DATA_SIZE
    L2_READ_WRITE_SRAM (RW)     :   o = L2_RW_DATA_START,   l = L2_RW_DATA_SIZE

    DSS_L3:   ORIGIN = 0x88000000, LENGTH = 0x00200000

    /* shared memories that are used by RTOS/NORTOS cores */
    /* On C66,
     * - make sure these are which mapped as non-cache in MAR bits
     */
    USER_SHM_MEM            : ORIGIN = 0xC02E8000, LENGTH = 0x00004000
    LOG_SHM_MEM             : ORIGIN = 0xC02EC000, LENGTH = 0x00004000
    /* 1st 512 B of DSS mailbox memory and MSS mailbox memory is used for IPC with R4 and should not be used by application */
    /* MSS mailbox memory is used as shared memory, we dont use bottom 32*6 bytes, since its used as SW queue by ipc_notify */
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0xC5000200, LENGTH = 0x1D40
}
