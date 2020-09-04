# OSAL Migration Guide {#OSAL_MIGRATION_GUIDE}

This section describes the differences between Driver Porting Layer of MCU+ SDK and OSAL from the Processor SDK RTOS (PDK).
This can be used as migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.

## API changes

There are changes in functions names, structure names and macro names. The changes in function names are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
        <th> Change Description / Remarks
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Address Translate**</td></tr>
    <tr>
        <td>NONE
        <td>\ref KERNEL_DPL_ADDR_TRANSLATE_PAGE
        <td>New in MCU+ SDK
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Cache**</td></tr>
    <tr>
        <td>CacheP_wb
        <td>\ref CacheP_wb
        <td>NO CHANGE
    </tr>
    <tr>
        <td>CacheP_Inv
        <td>\ref CacheP_inv
        <td>API rename
    </tr>
    <tr>
        <td>CacheP_wbInv
        <td>\ref CacheP_wbInv
        <td>NO CHANGE
    </tr>
    <tr>
        <td>CacheP_fenceCpu2Dma, CacheP_fenceDma2Cpu
        <td>NONE
        <td>NOT APPLICABLE for current supported SOCs
    </tr>
    <tr>
        <td>NONE
        <td>\ref CacheP_enable, \ref CacheP_disable, \ref CacheP_getEnabled, \ref CacheP_wbAll, \ref CacheP_wbInvAll, \ref CacheP_init
        <td>New APIs to complete functionality
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Clock**</td></tr>
    <tr>
        <td>NONE
        <td>\ref KERNEL_DPL_CLOCK_PAGE
        <td>New in MCU+ SDK
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Cycle Profiler**</td></tr>
    <tr>
        <td> CycleprofilerP_init
        <td>\ref CycleCounterP_reset
        <td>API rename
    </tr>
    <tr>
        <td> CycleprofilerP_getTimeStamp
        <td>\ref CycleCounterP_getCount32
        <td>API rename
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Debug**</td></tr>
    <tr>
        <td> DebugP_assert
        <td>\ref DebugP_assert
        <td>NO CHANGE
    </tr>
    <tr>
        <td> DebugP_log0, DebugP_log1, DebugP_log2, DebugP_log3, DebugP_log4
        <td>\ref DebugP_log
        <td>Single API to replace multiple APIs
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Event Combiner**</td></tr>
    <tr>
        <td> EventCombinerP_disableEvent, EventCombinerP_enableEvent, EventCombinerP_clearEvent, EventCombinerP_dispatchPlug,
        EventCombinerP_dispatchUnplug, EventCombinerP_getHwi, EventCombinerP_getIntNum, EventCombinerP_SingleRegisterInt, EventCombinerP_GroupRegisterInt
        <td> Use APIs from \ref KERNEL_DPL_HWI_PAGE
        <td>Single \ref KERNEL_DPL_HWI_PAGE API in MCU+ SDK to setup HWI
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Event**</td></tr>
    <tr>
        <td>EventP_create
        <td>\ref EventP_construct
        <td>Static allocation friendly API
    </tr>
    <tr>
        <td>EventP_delete
        <td>\ref EventP_destruct
        <td>Static allocation friendly API
    </tr>
    <tr>
        <td>EventP_pend
        <td>\ref EventP_waitBits
        <td>API change to make it more generic vs SysBIOS specific
    </tr>
    <tr>
        <td>EventP_post
        <td>\ref EventP_setBits
        <td>API rename
    </tr>
    <tr>
        <td> EventP_getPostedEvents
        <td>\ref EventP_getBits
        <td>API rename
    </tr>
    <tr>
        <td>EventP_Params_init
        <td>NONE
        <td>NOT required anymore
    </tr>
    <tr>
        <td>NONE
        <td>\ref EventP_clearBits
        <td>New APIs to complete functionality
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**HWI**</td></tr>
    <tr>
        <td>HwiP_create
        <td>\ref HwiP_construct
        <td>Static allocation friendly API
    </tr>
    <tr>
        <td>HwiP_delete
        <td>\ref HwiP_destruct
        <td>Static allocation friendly API
    </tr>
    <tr>
        <td>HwiP_disable
        <td>\ref HwiP_disable
        <td>NO CHANGE
    </tr>
    <tr>
        <td>HwiP_disableInterrupt
        <td>\ref HwiP_disableInt
        <td>API rename
    </tr>
    <tr>
        <td>HwiP_enableInterrupt
        <td>\ref HwiP_enableInt
        <td>API rename
    </tr>
    <tr>
        <td>HwiP_Params_init
        <td>\ref HwiP_Params_init
        <td>NO CHANGE
    </tr>
    <tr>
        <td>HwiP_restore
        <td>\ref HwiP_restore
        <td>NO CHANGE
    </tr>
    <tr>
        <td>HwiP_post
        <td>\ref HwiP_post
        <td>NO CHANGE
    </tr>
    <tr>
        <td>HwiP_createDirect
        <td>NONE
        <td>NOT supported as of now in MCU+ SDK, use \ref HwiP_construct instead
    </tr>
    <tr>
        <td>HwiP_getHandle, HwiP_getEventId
        <td>NONE
        <td>NOT needed
    </tr>
    <tr>
        <td>NONE
        <td>\ref HwiP_init, \ref HwiP_inISR, \ref HwiP_enable, \ref HwiP_clearInt, \ref HwiP_restoreInt, \ref HwiP_setArgs
        <td>New APIs to complete functionality
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Memory**</td></tr>
    <tr>
        <td>MemoryP_ctrlAlloc, MemoryP_dataAlloc
        <td>\ref HeapP_alloc
        <td>Use \ref HeapP_construct to create "ctrl", "data" heaps as needed by application
    </tr>
    <tr>
        <td>MemoryP_ctrlFree, MemoryP_dataFree
        <td>\ref HeapP_free
        <td>Use \ref HeapP_construct to create "ctrl", "data" heaps as needed by application
    </tr>
    <tr>
        <td>MemoryP_getStats
        <td>\ref HeapP_getHeapStats
        <td>
    </tr>
    <tr>
        <td>NONE
        <td>\ref HeapP_construct, \ref HeapP_destruct, \ref HeapP_getFreeHeapSize, \ref HeapP_getMinimumEverFreeHeapSize
        <td>New APIs to complete functionality
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**MPU ARMv7**</td></tr>
    <tr>
        <td>NONE
        <td>\ref KERNEL_DPL_MPU_ARMV7_PAGE
        <td>New in MCU+ SDK
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**OSAL**</td></tr>
    <tr>
        <td>Osal_setHwAttrs, Osal_getHwAttrs, Osal_getStaticMemStatus, Osal_getCoreId
        <td>NONE
        <td>NOT needed anymore
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Queue**</td></tr>
    <tr>
        <td>Osal_Queue_construct, Osal_Queue_handle, Osal_Queue_empty, Osal_Queue_get, Osal_Queue_put
        <td>NONE
        <td>NOT supported in MCU+ SDK. Use direct OS APIs for queue or write your own implementation instead.
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**RegisterIntr**</td></tr>
    <tr>
        <td>Osal_RegisterInterrupt_initParams
        <td>\ref HwiP_Params_init
        <td>Single \ref KERNEL_DPL_HWI_PAGE API in MCU+ SDK to setup HWI
    </tr>
    <tr>
        <td>Osal_RegisterInterrupt, Osal_RegisterInterruptDirect
        <td>\ref HwiP_construct
        <td>Single \ref KERNEL_DPL_HWI_PAGE API in MCU+ SDK to setup HWI
    </tr>
    <tr>
        <td>Osal_DeleteInterrupt
        <td>\ref HwiP_disableInt
        <td>Single \ref KERNEL_DPL_HWI_PAGE API in MCU+ SDK to setup HWI
    </tr>
    <tr>
        <td>Osal_EnableInterrupt
        <td>\ref HwiP_enableInt
        <td>Single \ref KERNEL_DPL_HWI_PAGE API in MCU+ SDK to setup HWI
    </tr>
    <tr>
        <td>Osal_DisableInterrupt
        <td>\ref HwiP_disableInt
        <td>Single \ref KERNEL_DPL_HWI_PAGE API in MCU+ SDK to setup HWI
    </tr>
    <tr>
        <td>Osal_ClearInterrupt
        <td>\ref HwiP_clearInt
        <td>Single \ref KERNEL_DPL_HWI_PAGE API in MCU+ SDK to setup HWI
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Semaphore**</td></tr>
    <tr>
        <td>SemaphoreP_create
        <td>\ref SemaphoreP_constructMutex, \ref SemaphoreP_constructBinary, \ref SemaphoreP_constructCounting
        <td>Static allocation friendly API. Distinct API for mutex vs binary semaphore.
    </tr>
    <tr>
        <td>SemaphoreP_delete
        <td>\ref SemaphoreP_destruct
        <td>Static allocation friendly API
    </tr>
    <tr>
        <td>SemaphoreP_Params_init
        <td>NONE
        <td>NOT needed
    </tr>
    <tr>
        <td>SemaphoreP_pend
        <td>\ref SemaphoreP_pend
        <td>NO CHANGE
    </tr>
    <tr>
        <td>SemaphoreP_post
        <td>\ref SemaphoreP_post
        <td>NO CHANGE
    </tr>
    <tr>
        <td>SemaphoreP_postFromClock, SemaphoreP_postFromISR
        <td>\ref SemaphoreP_post
        <td>Single API for all variants
    </tr>
    <tr>
        <td>SemaphoreP_getCount
        <td>NONE
        <td>NOT needed
    </tr>
    <tr>
        <td>SemaphoreP_reset
        <td>NONE
        <td>Use \ref SemaphoreP_pend with timeout as 0 to reset sempahore.
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**SWI**</td></tr>
    <tr>
        <td>SwiP_create, SwiP_delete, SwiP_Params_init, SwiP_post
        <td>NONE
        <td>SysBIOS specific API, see \ref KERNEL_FREERTOS_PAGE for equivalent API in FreeRTOS.
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Task**</td></tr>
    <tr>
        <td> TaskP_create
        <td>\ref TaskP_construct
        <td>Static allocation friendly API
    </tr>
    <tr>
        <td>TaskP_delete
        <td>\ref TaskP_destruct
        <td>Static allocation friendly API
    </tr>
    <tr>
        <td>TaskP_Params_init
        <td>\ref TaskP_Params_init
        <td>NO CHANGE
    </tr>
    <tr>
        <td>TaskP_sleep
        <td>\ref ClockP_sleep
        <td>API rename
    </tr>
    <tr>
        <td>TaskP_sleepInMsecs
        <td>\ref ClockP_usleep
        <td>API rename
    </tr>
    <tr>
        <td>TaskP_yield
        <td>\ref TaskP_yield
        <td>NO CHANGE
    </tr>
    <tr>
        <td>TaskP_setPrio, TaskP_self, TaskP_selfmacro, TaskP_isTerminated, OS_start
        <td>NONE
        <td>NOT needed
    </tr>
    <tr>
        <td>NONE
        <td>\ref TaskP_exit
        <td>New APIs to complete functionality
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Timer**</td></tr>
    <tr>
        <td>TimerP_create
        <td>\ref TimerP_setup
        <td>Takes timer base address as input in MCU+ SDK and HWI register should be done from application.
    </tr>
    <tr>
        <td>TimerP_delete
        <td>NONE
        <td>NOT needed. Stop the timer when done.
    </tr>
    <tr>
        <td>TimerP_setPeriodMicroSecs
        <td>\ref TimerP_setup
        <td>Separate API not needed.
    </tr>
    <tr>
        <td>TimerP_Params_init
        <td>\ref TimerP_Params_init
        <td>NO CHANGE
    </tr>
    <tr>
        <td>TimerP_start
        <td>\ref TimerP_start
        <td>Takes timer base address as input in MCU+ SDK
    </tr>
    <tr>
        <td>TimerP_stop
        <td>\ref TimerP_stop
        <td>Takes timer base address as input in MCU+ SDK
    </tr>
    <tr>
        <td>TimerP_ClearInterrupt
        <td>\ref TimerP_clearOverflowInt
        <td>Takes timer base address as input in MCU+ SDK
    </tr>
    <tr>
        <td>TimerP_getTimeInUsecs
        <td>\ref ClockP_getTimeUsec
        <td>API rename
    </tr>
    <tr>
        <td>NONE
        <td>\ref TimerP_getCount, \ref TimerP_getReloadCount, \ref TimerP_isOverflowed
        <td>New APIs to complete functionality
    </tr>
</table>

## Important Notes

- All DPL APIs use the common return values from \ref KERNEL_DPL_SYSTEM for success and failure
- Some APIs take timeout as input, timeout is always specified in units of clock or OS ticks.
  - Use \ref SystemP_NO_WAIT when you dont want to block, i.e timeout of zero.
  - Use \ref SystemP_WAIT_FOREVER when one wants to block for ever, i.e wait forever.

## See Also

\ref KERNEL_DPL_PAGE