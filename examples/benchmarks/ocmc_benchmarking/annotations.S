@ #define ENABLE_SMART_PLACEMENT_ANNOTATION

#ifdef ENABLE_SMART_PLACEMENT_ANNOTATION

.global _DebugP_assert
 .sym_meta_info _DebugP_assert, "of_placement", "local", 0

.global _DebugP_assertNoLog
 .sym_meta_info _DebugP_assertNoLog, "of_placement", "local", 102214

.global vPortEnterCritical
 .sym_meta_info vPortEnterCritical, "of_placement", "local", 102227

.global vPortExitCritical
 .sym_meta_info vPortExitCritical, "of_placement", "local", 102227

.global ClockP_getTimeUsec
 .sym_meta_info ClockP_getTimeUsec, "of_placement", "local", 121832

.global ClockP_getTimerCount
 .sym_meta_info ClockP_getTimerCount, "of_placement", "local", 121832

.global TimerP_getCount
 .sym_meta_info TimerP_getCount, "of_placement", "local", 121832

.global HW_RD_REG32_RAW
 .sym_meta_info HW_RD_REG32_RAW, "of_placement", "local", 129890

.global HwiP_inISR
 .sym_meta_info HwiP_inISR, "of_placement", "local", 132214

.global HW_WR_REG32_RAW
 .sym_meta_info HW_WR_REG32_RAW, "of_placement", "local", 132703

.global HwiP_clearInt
 .sym_meta_info HwiP_clearInt, "of_placement", "local", 133318

.global HwiP_irq_handler_c
 .sym_meta_info HwiP_irq_handler_c, "of_placement", "local", 133320

.global HwiP_getIRQ
 .sym_meta_info HwiP_getIRQ, "of_placement", "local", 133320

.global HwiP_isPulse
 .sym_meta_info HwiP_isPulse, "of_placement", "local", 133320

.global HwiP_ackIRQ
 .sym_meta_info HwiP_ackIRQ, "of_placement", "local", 133320

.global xTaskGetSchedulerState
 .sym_meta_info xTaskGetSchedulerState, "of_placement", "local", 135137

.global xTaskIncrementTick
 .sym_meta_info xTaskIncrementTick, "of_placement", "local", 136134

.global TimerP_clearOverflowInt
 .sym_meta_info TimerP_clearOverflowInt, "of_placement", "local", 136147

.global ClockP_timerTickIsr
 .sym_meta_info ClockP_timerTickIsr, "of_placement", "local", 136148

.global ClockP_timerClearOverflowInt
 .sym_meta_info ClockP_timerClearOverflowInt, "of_placement", "local", 136148

.global vPortTimerTickHandler
 .sym_meta_info vPortTimerTickHandler, "of_placement", "local", 136148

.global vApplicationLoadHook
 .sym_meta_info vApplicationLoadHook, "of_placement", "local", 136593

.global prvCheckTasksWaitingTermination
 .sym_meta_info prvCheckTasksWaitingTermination, "of_placement", "local", 136593

.global vApplicationIdleHook
 .sym_meta_info vApplicationIdleHook, "of_placement", "local", 136593

.global uiPortGetRunTimeCounterValue
 .sym_meta_info uiPortGetRunTimeCounterValue, "of_placement", "local", 143489

.global vTaskSwitchContext
 .sym_meta_info vTaskSwitchContext, "of_placement", "local", 143533

.global uxListRemove
 .sym_meta_info uxListRemove, "of_placement", "local", 144187

.global SemaphoreP_pend
 .sym_meta_info SemaphoreP_pend, "of_placement", "local", 145271

.global xQueueSemaphoreTake
 .sym_meta_info xQueueSemaphoreTake, "of_placement", "local", 145272

.global SemaphoreP_post
 .sym_meta_info SemaphoreP_post, "of_placement", "local", 145287

.global xQueueGenericSend
 .sym_meta_info xQueueGenericSend, "of_placement", "local", 148112

.global prvCopyDataToQueue
 .sym_meta_info prvCopyDataToQueue, "of_placement", "local", 148113

.global get_rand
 .sym_meta_info get_rand, "of_placement", "local", 148230

.global QueueP_isEmpty
 .sym_meta_info QueueP_isEmpty, "of_placement", "local", 148230

.global TaskP_yield
 .sym_meta_info TaskP_yield, "of_placement", "local", 148230

.global vListInsertEnd
 .sym_meta_info vListInsertEnd, "of_placement", "local", 148842

.global CSL_REG32_RD_RAW
 .sym_meta_info CSL_REG32_RD_RAW, "of_placement", "local", 152564

.global UART_getIntrIdentityStatus
 .sym_meta_info UART_getIntrIdentityStatus, "of_placement", "local", 152574

.global Sciclient_secProxyThreadStatusReg
 .sym_meta_info Sciclient_secProxyThreadStatusReg, "of_placement", "local", 152591

.global QueueP_get
 .sym_meta_info QueueP_get, "of_placement", "local", 153230

.global QueueP_put
 .sym_meta_info QueueP_put, "of_placement", "local", 153230

.global vTaskSuspendAll
 .sym_meta_info vTaskSuspendAll, "of_placement", "local", 153478

.global xTaskResumeAll
 .sym_meta_info xTaskResumeAll, "of_placement", "local", 153478

.global prvAddCurrentTaskToDelayedList
 .sym_meta_info prvAddCurrentTaskToDelayedList, "of_placement", "local", 153539

.global prvUnlockQueue
 .sym_meta_info prvUnlockQueue, "of_placement", "local", 153539

.global vTaskPlaceOnEventList
 .sym_meta_info vTaskPlaceOnEventList, "of_placement", "local", 153540

.global vTaskInternalSetTimeOutState
 .sym_meta_info vTaskInternalSetTimeOutState, "of_placement", "local", 153540

.global xTaskCheckForTimeOut
 .sym_meta_info xTaskCheckForTimeOut, "of_placement", "local", 153540

.global prvIsQueueEmpty
 .sym_meta_info prvIsQueueEmpty, "of_placement", "local", 153540

.global vListInsert
 .sym_meta_info vListInsert, "of_placement", "local", 153540

.global xTaskRemoveFromEventList
 .sym_meta_info xTaskRemoveFromEventList, "of_placement", "local", 153556

.global _out_char
 .sym_meta_info _out_char, "of_placement", "local", 155288

.global UART_intrDisable
 .sym_meta_info UART_intrDisable, "of_placement", "local", 155401

.global putchar_
 .sym_meta_info putchar_, "of_placement", "local", 155402

.global DebugP_uartLogWriterPutChar
 .sym_meta_info DebugP_uartLogWriterPutChar, "of_placement", "local", 155402

.global UART_Transaction_init
 .sym_meta_info UART_Transaction_init, "of_placement", "local", 155402

.global xQueueGiveFromISR
 .sym_meta_info xQueueGiveFromISR, "of_placement", "local", 155402

.global vPortYeildFromISR
 .sym_meta_info vPortYeildFromISR, "of_placement", "local", 155402

.global vPortValidateInterruptPriority
 .sym_meta_info vPortValidateInterruptPriority, "of_placement", "local", 155402

.global UART_getHandle
 .sym_meta_info UART_getHandle, "of_placement", "local", 155402

.global UART_write
 .sym_meta_info UART_write, "of_placement", "local", 155402

.global UART_intrEnable
 .sym_meta_info UART_intrEnable, "of_placement", "local", 155402

.global UART_checkTransaction
 .sym_meta_info UART_checkTransaction, "of_placement", "local", 155402

.global UART_masterIsr
 .sym_meta_info UART_masterIsr, "of_placement", "local", 155402

.global UART_writeData
 .sym_meta_info UART_writeData, "of_placement", "local", 155402

.global UART_putChar
 .sym_meta_info UART_putChar, "of_placement", "local", 155402

.global UART_writeInterrupt
 .sym_meta_info UART_writeInterrupt, "of_placement", "local", 155402

.global TaskP_calcCounterDiff
 .sym_meta_info TaskP_calcCounterDiff, "of_placement", "local", 157438

.global vTaskGetInfo
 .sym_meta_info vTaskGetInfo, "of_placement", "local", 157482

.global HwiP_setPri
 .sym_meta_info HwiP_setPri, "of_placement", "local", 157716

.global HwiP_setVecAddr
 .sym_meta_info HwiP_setVecAddr, "of_placement", "local", 157716

.global xTaskGetCurrentTaskHandle
 .sym_meta_info xTaskGetCurrentTaskHandle, "of_placement", "local", 158000

.global xTaskPriorityDisinherit
 .sym_meta_info xTaskPriorityDisinherit, "of_placement", "local", 158113

.global xQueueGiveMutexRecursive
 .sym_meta_info xQueueGiveMutexRecursive, "of_placement", "local", 158115

.global xQueueTakeMutexRecursive
 .sym_meta_info xQueueTakeMutexRecursive, "of_placement", "local", 158115

.global _DebugP_logZone
 .sym_meta_info _DebugP_logZone, "of_placement", "local", 158116

.global pvTaskIncrementMutexHeldCount
 .sym_meta_info pvTaskIncrementMutexHeldCount, "of_placement", "local", 158116

.global vprintf_
 .sym_meta_info vprintf_, "of_placement", "local", 158116

.global _vsnprintf
 .sym_meta_info _vsnprintf, "of_placement", "local", 158116

.global _is_digit
 .sym_meta_info _is_digit, "of_placement", "local", 158122

.global _ntoa_format
 .sym_meta_info _ntoa_format, "of_placement", "local", 158122

.global _out_rev
 .sym_meta_info _out_rev, "of_placement", "local", 158122

.global _ntoa_long
 .sym_meta_info _ntoa_long, "of_placement", "local", 158122

.global vListInitialise
 .sym_meta_info vListInitialise, "of_placement", "local", 158147

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158171

.global CSL_secProxyGetDataAddr
 .sym_meta_info CSL_secProxyGetDataAddr, "of_placement", "local", 158180

.global TaskP_loadUpdateAll
 .sym_meta_info TaskP_loadUpdateAll, "of_placement", "local", 158186

.global xTaskGetIdleTaskHandle
 .sym_meta_info xTaskGetIdleTaskHandle, "of_placement", "local", 158186

.global HW_WR_FIELD32_RAW
 .sym_meta_info HW_WR_FIELD32_RAW, "of_placement", "local", 158188

.global vListInitialiseItem
 .sym_meta_info vListInitialiseItem, "of_placement", "local", 158190

.global UART_regConfigModeEnable
 .sym_meta_info UART_regConfigModeEnable, "of_placement", "local", 158201

.global Sciclient_secProxyReadThread32
 .sym_meta_info Sciclient_secProxyReadThread32, "of_placement", "local", 158204

.global xQueueGenericReset
 .sym_meta_info xQueueGenericReset, "of_placement", "local", 158208

.global xQueueGenericCreateStatic
 .sym_meta_info xQueueGenericCreateStatic, "of_placement", "local", 158208

.global vQueueAddToRegistry
 .sym_meta_info vQueueAddToRegistry, "of_placement", "local", 158208

.global prvInitialiseNewQueue
 .sym_meta_info prvInitialiseNewQueue, "of_placement", "local", 158208

.global xTaskCreateStatic
 .sym_meta_info xTaskCreateStatic, "of_placement", "local", 158210

.global prvInitialiseNewTask
 .sym_meta_info prvInitialiseNewTask, "of_placement", "local", 158210

.global prvAddNewTaskToReadyList
 .sym_meta_info prvAddNewTaskToReadyList, "of_placement", "local", 158210

.global pxPortInitialiseStack
 .sym_meta_info pxPortInitialiseStack, "of_placement", "local", 158210

.global SemaphoreP_constructBinary
 .sym_meta_info SemaphoreP_constructBinary, "of_placement", "local", 158212

.global TaskP_construct
 .sym_meta_info TaskP_construct, "of_placement", "local", 158213

.global TaskP_addToRegistry
 .sym_meta_info TaskP_addToRegistry, "of_placement", "local", 158213

.global QueueP_create
 .sym_meta_info QueueP_create, "of_placement", "local", 158214

.global Sciclient_secProxyVerifyThread
 .sym_meta_info Sciclient_secProxyVerifyThread, "of_placement", "local", 158214

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158216

.global AddrTranslateP_getLocalAddr
 .sym_meta_info AddrTranslateP_getLocalAddr, "of_placement", "local", 158218

.global multicore_wait
 .sym_meta_info multicore_wait, "of_placement", "local", 158220

.global HW_RD_FIELD32_RAW
 .sym_meta_info HW_RD_FIELD32_RAW, "of_placement", "local", 158220

.global CSL_secProxyGetMaxMsgSize
 .sym_meta_info CSL_secProxyGetMaxMsgSize, "of_placement", "local", 158221

.global Sciclient_service
 .sym_meta_info Sciclient_service, "of_placement", "local", 158222

.global Sciclient_getCurrentContext
 .sym_meta_info Sciclient_getCurrentContext, "of_placement", "local", 158222

.global Sciclient_sendMessage
 .sym_meta_info Sciclient_sendMessage, "of_placement", "local", 158222

.global Sciclient_waitForMessage
 .sym_meta_info Sciclient_waitForMessage, "of_placement", "local", 158222

.global Sciclient_recvMessage
 .sym_meta_info Sciclient_recvMessage, "of_placement", "local", 158222

.global Sciclient_secProxyReadThreadCount
 .sym_meta_info Sciclient_secProxyReadThreadCount, "of_placement", "local", 158222

.global Sciclient_secProxyWaitThread
 .sym_meta_info Sciclient_secProxyWaitThread, "of_placement", "local", 158222

.global Sciclient_secProxyFlush
 .sym_meta_info Sciclient_secProxyFlush, "of_placement", "local", 158222

.global UART_operatingModeSelect
 .sym_meta_info UART_operatingModeSelect, "of_placement", "local", 158222

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158226

.global SemaphoreP_constructMutex
 .sym_meta_info SemaphoreP_constructMutex, "of_placement", "local", 158227

.global xQueueCreateMutexStatic
 .sym_meta_info xQueueCreateMutexStatic, "of_placement", "local", 158227

.global prvInitialiseMutex
 .sym_meta_info prvInitialiseMutex, "of_placement", "local", 158227

.global UART_enhanFuncEnable
 .sym_meta_info UART_enhanFuncEnable, "of_placement", "local", 158227

.global UART_divisorLatchWrite
 .sym_meta_info UART_divisorLatchWrite, "of_placement", "local", 158227

.global DebugP_logZoneEnable
 .sym_meta_info DebugP_logZoneEnable, "of_placement", "local", 158228

.global HwiP_enableInt
 .sym_meta_info HwiP_enableInt, "of_placement", "local", 158228

.global HwiP_disableInt
 .sym_meta_info HwiP_disableInt, "of_placement", "local", 158228

.global HwiP_Params_init
 .sym_meta_info HwiP_Params_init, "of_placement", "local", 158228

.global HwiP_construct
 .sym_meta_info HwiP_construct, "of_placement", "local", 158228

.global HwiP_setAsFIQ
 .sym_meta_info HwiP_setAsFIQ, "of_placement", "local", 158228

.global HwiP_setAsPulse
 .sym_meta_info HwiP_setAsPulse, "of_placement", "local", 158228

.global vTaskDelete
 .sym_meta_info vTaskDelete, "of_placement", "local", 158228

.global prvDeleteTCB
 .sym_meta_info prvDeleteTCB, "of_placement", "local", 158228

.global Pinmux_config
 .sym_meta_info Pinmux_config, "of_placement", "local", 158228

.global Pinmux_lockMMR
 .sym_meta_info Pinmux_lockMMR, "of_placement", "local", 158228

.global Pinmux_unlockMMR
 .sym_meta_info Pinmux_unlockMMR, "of_placement", "local", 158228

.global Sciclient_getContext
 .sym_meta_info Sciclient_getContext, "of_placement", "local", 158228

.global UART_enhanFuncBitValRestore
 .sym_meta_info UART_enhanFuncBitValRestore, "of_placement", "local", 158228

.global MasterTask
 .sym_meta_info MasterTask, "of_placement", "local", 158229

.global ocmc_benchmarking_main
 .sym_meta_info ocmc_benchmarking_main, "of_placement", "local", 158229

.global ocmc_benchmarking_entry
 .sym_meta_info ocmc_benchmarking_entry, "of_placement", "local", 158229

.global SlaveTaskFxn_0
 .sym_meta_info SlaveTaskFxn_0, "of_placement", "local", 158229

.global SlaveTaskFxn_1
 .sym_meta_info SlaveTaskFxn_1, "of_placement", "local", 158229

.global SlaveTaskFxn_2
 .sym_meta_info SlaveTaskFxn_2, "of_placement", "local", 158229

.global SlaveTaskFxn_3
 .sym_meta_info SlaveTaskFxn_3, "of_placement", "local", 158229

.global SlaveTaskFxn_4
 .sym_meta_info SlaveTaskFxn_4, "of_placement", "local", 158229

.global SlaveTaskFxn_5
 .sym_meta_info SlaveTaskFxn_5, "of_placement", "local", 158229

.global SlaveTaskFxn_6
 .sym_meta_info SlaveTaskFxn_6, "of_placement", "local", 158229

.global SlaveTaskFxn_7
 .sym_meta_info SlaveTaskFxn_7, "of_placement", "local", 158229

.global SlaveTaskFxn_8
 .sym_meta_info SlaveTaskFxn_8, "of_placement", "local", 158229

.global SlaveTaskFxn_9
 .sym_meta_info SlaveTaskFxn_9, "of_placement", "local", 158229

.global SlaveTaskFxn_10
 .sym_meta_info SlaveTaskFxn_10, "of_placement", "local", 158229

.global SlaveTaskFxn_11
 .sym_meta_info SlaveTaskFxn_11, "of_placement", "local", 158229

.global SlaveTaskFxn_12
 .sym_meta_info SlaveTaskFxn_12, "of_placement", "local", 158229

.global SlaveTaskFxn_13
 .sym_meta_info SlaveTaskFxn_13, "of_placement", "local", 158229

.global SlaveTaskFxn_14
 .sym_meta_info SlaveTaskFxn_14, "of_placement", "local", 158229

.global SlaveTaskFxn_15
 .sym_meta_info SlaveTaskFxn_15, "of_placement", "local", 158229

.global freertos_main
 .sym_meta_info freertos_main, "of_placement", "local", 158229

.global main
 .sym_meta_info main, "of_placement", "local", 158229

.global Drivers_uartInit
 .sym_meta_info Drivers_uartInit, "of_placement", "local", 158229

.global System_init
 .sym_meta_info System_init, "of_placement", "local", 158229

.global Drivers_open
 .sym_meta_info Drivers_open, "of_placement", "local", 158229

.global Drivers_uartOpen
 .sym_meta_info Drivers_uartOpen, "of_placement", "local", 158229

.global Board_init
 .sym_meta_info Board_init, "of_placement", "local", 158229

.global Board_driversOpen
 .sym_meta_info Board_driversOpen, "of_placement", "local", 158229

.global Dpl_init
 .sym_meta_info Dpl_init, "of_placement", "local", 158229

.global Pinmux_init
 .sym_meta_info Pinmux_init, "of_placement", "local", 158229

.global Module_clockEnable
 .sym_meta_info Module_clockEnable, "of_placement", "local", 158229

.global Module_clockSetFrequency
 .sym_meta_info Module_clockSetFrequency, "of_placement", "local", 158229

.global PowerClock_init
 .sym_meta_info PowerClock_init, "of_placement", "local", 158229

.global DebugP_uartSetDrvIndex
 .sym_meta_info DebugP_uartSetDrvIndex, "of_placement", "local", 158229

.global HwiP_init
 .sym_meta_info HwiP_init, "of_placement", "local", 158229

.global HwiP_getIRQVecAddr
 .sym_meta_info HwiP_getIRQVecAddr, "of_placement", "local", 158229

.global HwiP_getFIQVecAddr
 .sym_meta_info HwiP_getFIQVecAddr, "of_placement", "local", 158229

.global HwiP_ackIRQ
 .sym_meta_info HwiP_ackIRQ, "of_placement", "local", 158229

.global HwiP_ackFIQ
 .sym_meta_info HwiP_ackFIQ, "of_placement", "local", 158229

.global ClockP_usecToTicks
 .sym_meta_info ClockP_usecToTicks, "of_placement", "local", 158229

.global ClockP_init
 .sym_meta_info ClockP_init, "of_placement", "local", 158229

.global TaskP_Params_init
 .sym_meta_info TaskP_Params_init, "of_placement", "local", 158229

.global vTaskStartScheduler
 .sym_meta_info vTaskStartScheduler, "of_placement", "local", 158229

.global xTaskGetTickCount
 .sym_meta_info xTaskGetTickCount, "of_placement", "local", 158229

.global vTaskPlaceOnEventListRestricted
 .sym_meta_info vTaskPlaceOnEventListRestricted, "of_placement", "local", 158229

.global prvInitialiseTaskLists
 .sym_meta_info prvInitialiseTaskLists, "of_placement", "local", 158229

.global prvIdleTask
 .sym_meta_info prvIdleTask, "of_placement", "local", 158229

.global xTimerCreateTimerTask
 .sym_meta_info xTimerCreateTimerTask, "of_placement", "local", 158229

.global prvTimerTask
 .sym_meta_info prvTimerTask, "of_placement", "local", 158229

.global prvGetNextExpireTime
 .sym_meta_info prvGetNextExpireTime, "of_placement", "local", 158229

.global prvProcessTimerOrBlockTask
 .sym_meta_info prvProcessTimerOrBlockTask, "of_placement", "local", 158229

.global prvSampleTimeNow
 .sym_meta_info prvSampleTimeNow, "of_placement", "local", 158229

.global prvCheckForValidListAndQueue
 .sym_meta_info prvCheckForValidListAndQueue, "of_placement", "local", 158229

.global vQueueWaitForMessageRestricted
 .sym_meta_info vQueueWaitForMessageRestricted, "of_placement", "local", 158229

.global xPortStartScheduler
 .sym_meta_info xPortStartScheduler, "of_placement", "local", 158229

.global vPortConfigTimerForRunTimeStats
 .sym_meta_info vPortConfigTimerForRunTimeStats, "of_placement", "local", 158229

.global vApplicationGetIdleTaskMemory
 .sym_meta_info vApplicationGetIdleTaskMemory, "of_placement", "local", 158229

.global vApplicationGetTimerTaskMemory
 .sym_meta_info vApplicationGetTimerTaskMemory, "of_placement", "local", 158229

.global TimerP_Params_init
 .sym_meta_info TimerP_Params_init, "of_placement", "local", 158229

.global TimerP_setup
 .sym_meta_info TimerP_setup, "of_placement", "local", 158229

.global TimerP_start
 .sym_meta_info TimerP_start, "of_placement", "local", 158229

.global TimerP_stop
 .sym_meta_info TimerP_stop, "of_placement", "local", 158229

.global TimerP_getReloadCount
 .sym_meta_info TimerP_getReloadCount, "of_placement", "local", 158229

.global Sciclient_init
 .sym_meta_info Sciclient_init, "of_placement", "local", 158229

.global CSL_REG32_FEXT_RAW
 .sym_meta_info CSL_REG32_FEXT_RAW, "of_placement", "local", 158229

.global Sciclient_getDevId
 .sym_meta_info Sciclient_getDevId, "of_placement", "local", 158229

.global SOC_moduleClockEnable
 .sym_meta_info SOC_moduleClockEnable, "of_placement", "local", 158229

.global SOC_moduleSetClockFrequency
 .sym_meta_info SOC_moduleSetClockFrequency, "of_placement", "local", 158229

.global SOC_controlModuleLockMMR
 .sym_meta_info SOC_controlModuleLockMMR, "of_placement", "local", 158229

.global SOC_controlModuleUnlockMMR
 .sym_meta_info SOC_controlModuleUnlockMMR, "of_placement", "local", 158229

.global UART_init
 .sym_meta_info UART_init, "of_placement", "local", 158229

.global UART_open
 .sym_meta_info UART_open, "of_placement", "local", 158229

.global UART_intr2Disable
 .sym_meta_info UART_intr2Disable, "of_placement", "local", 158229

.global UART_configInstance
 .sym_meta_info UART_configInstance, "of_placement", "local", 158229

.global UART_resetModule
 .sym_meta_info UART_resetModule, "of_placement", "local", 158229

.global UART_regConfModeRestore
 .sym_meta_info UART_regConfModeRestore, "of_placement", "local", 158229

.global UART_modemControlReset
 .sym_meta_info UART_modemControlReset, "of_placement", "local", 158229

.global UART_moduleReset
 .sym_meta_info UART_moduleReset, "of_placement", "local", 158229

.global UART_fifoConfig
 .sym_meta_info UART_fifoConfig, "of_placement", "local", 158229

.global UART_subConfigTCRTLRModeEn
 .sym_meta_info UART_subConfigTCRTLRModeEn, "of_placement", "local", 158229

.global UART_fifoRegisterWrite
 .sym_meta_info UART_fifoRegisterWrite, "of_placement", "local", 158229

.global UART_tcrTlrBitValRestore
 .sym_meta_info UART_tcrTlrBitValRestore, "of_placement", "local", 158229

.global UART_divisorValCompute
 .sym_meta_info UART_divisorValCompute, "of_placement", "local", 158229

.global UART_divideRoundCloset
 .sym_meta_info UART_divideRoundCloset, "of_placement", "local", 158229

.global UART_lineCharConfig
 .sym_meta_info UART_lineCharConfig, "of_placement", "local", 158229

.global UART_divisorLatchDisable
 .sym_meta_info UART_divisorLatchDisable, "of_placement", "local", 158229

.global UART_breakCtl
 .sym_meta_info UART_breakCtl, "of_placement", "local", 158229

.global UART_hardwareFlowCtrlOptSet
 .sym_meta_info UART_hardwareFlowCtrlOptSet, "of_placement", "local", 158229

.global UART_checkOpenParams
 .sym_meta_info UART_checkOpenParams, "of_placement", "local", 158229

.global Sciclient_pmGetModuleState
 .sym_meta_info Sciclient_pmGetModuleState, "of_placement", "local", 158229

.global Sciclient_pmModuleClkRequest
 .sym_meta_info Sciclient_pmModuleClkRequest, "of_placement", "local", 158229

.global Sciclient_pmModuleGetClkStatus
 .sym_meta_info Sciclient_pmModuleGetClkStatus, "of_placement", "local", 158229

.global Sciclient_pmSetModuleClkParent
 .sym_meta_info Sciclient_pmSetModuleClkParent, "of_placement", "local", 158229

.global Sciclient_pmGetModuleClkParent
 .sym_meta_info Sciclient_pmGetModuleClkParent, "of_placement", "local", 158229

.global Sciclient_pmGetModuleClkNumParent
 .sym_meta_info Sciclient_pmGetModuleClkNumParent, "of_placement", "local", 158229

.global Sciclient_pmSetModuleClkFreq
 .sym_meta_info Sciclient_pmSetModuleClkFreq, "of_placement", "local", 158229

.global Sciclient_pmQueryModuleClkFreq
 .sym_meta_info Sciclient_pmQueryModuleClkFreq, "of_placement", "local", 158229

.global ocmc_benchmarking_exit
 .sym_meta_info ocmc_benchmarking_exit, "of_placement", "local", 158230

.global CSL_REG64_RD_RAW
 .sym_meta_info CSL_REG64_RD_RAW, "of_placement", "local", 158230

.global CSL_REG64_WR_RAW
 .sym_meta_info CSL_REG64_WR_RAW, "of_placement", "local", 158230

.global CSL_REG64_FEXT_RAW
 .sym_meta_info CSL_REG64_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG64_FINS_RAW
 .sym_meta_info CSL_REG64_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_RAW
 .sym_meta_info CSL_REG32_RD_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158230

.global CSL_REG16_RD_RAW
 .sym_meta_info CSL_REG16_RD_RAW, "of_placement", "local", 158230

.global CSL_REG16_WR_RAW
 .sym_meta_info CSL_REG16_WR_RAW, "of_placement", "local", 158230

.global CSL_REG8_RD_RAW
 .sym_meta_info CSL_REG8_RD_RAW, "of_placement", "local", 158230

.global CSL_REG8_WR_RAW
 .sym_meta_info CSL_REG8_WR_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_OFF_RAW
 .sym_meta_info CSL_REG32_RD_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_OFF_RAW
 .sym_meta_info CSL_REG32_WR_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_FEXT_RAW
 .sym_meta_info CSL_REG32_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG16_FEXT_RAW
 .sym_meta_info CSL_REG16_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG8_FEXT_RAW
 .sym_meta_info CSL_REG8_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG32_FINS_RAW
 .sym_meta_info CSL_REG32_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG16_FINS_RAW
 .sym_meta_info CSL_REG16_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG8_FINS_RAW
 .sym_meta_info CSL_REG8_FINS_RAW, "of_placement", "local", 158230

.global HW_RD_REG32_RAW
 .sym_meta_info HW_RD_REG32_RAW, "of_placement", "local", 158230

.global HW_WR_REG32_RAW
 .sym_meta_info HW_WR_REG32_RAW, "of_placement", "local", 158230

.global HW_RD_REG16_RAW
 .sym_meta_info HW_RD_REG16_RAW, "of_placement", "local", 158230

.global HW_WR_REG16_RAW
 .sym_meta_info HW_WR_REG16_RAW, "of_placement", "local", 158230

.global HW_RD_REG8_RAW
 .sym_meta_info HW_RD_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_REG8_RAW
 .sym_meta_info HW_WR_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD32_RAW
 .sym_meta_info HW_WR_FIELD32_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD16_RAW
 .sym_meta_info HW_WR_FIELD16_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD8_RAW
 .sym_meta_info HW_WR_FIELD8_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD32_RAW
 .sym_meta_info HW_RD_FIELD32_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD16_RAW
 .sym_meta_info HW_RD_FIELD16_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD8_RAW
 .sym_meta_info HW_RD_FIELD8_RAW, "of_placement", "local", 158230

.global UART_Params_init
 .sym_meta_info UART_Params_init, "of_placement", "local", 158230

.global UART_Transaction_init
 .sym_meta_info UART_Transaction_init, "of_placement", "local", 158230

.global UART_putChar
 .sym_meta_info UART_putChar, "of_placement", "local", 158230

.global UART_getChar
 .sym_meta_info UART_getChar, "of_placement", "local", 158230

.global UART_intrEnable
 .sym_meta_info UART_intrEnable, "of_placement", "local", 158230

.global UART_intrDisable
 .sym_meta_info UART_intrDisable, "of_placement", "local", 158230

.global UART_intr2Enable
 .sym_meta_info UART_intr2Enable, "of_placement", "local", 158230

.global UART_intr2Disable
 .sym_meta_info UART_intr2Disable, "of_placement", "local", 158230

.global UART_getIntrIdentityStatus
 .sym_meta_info UART_getIntrIdentityStatus, "of_placement", "local", 158230

.global UART_getIntr2Status
 .sym_meta_info UART_getIntr2Status, "of_placement", "local", 158230

.global UART_checkCharsAvailInFifo
 .sym_meta_info UART_checkCharsAvailInFifo, "of_placement", "local", 158230

.global UART_readLineStatus
 .sym_meta_info UART_readLineStatus, "of_placement", "local", 158230

.global UART_getCharFifo
 .sym_meta_info UART_getCharFifo, "of_placement", "local", 158230

.global CSL_REG64_RD_RAW
 .sym_meta_info CSL_REG64_RD_RAW, "of_placement", "local", 158230

.global CSL_REG64_WR_RAW
 .sym_meta_info CSL_REG64_WR_RAW, "of_placement", "local", 158230

.global CSL_REG64_FEXT_RAW
 .sym_meta_info CSL_REG64_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG64_FINS_RAW
 .sym_meta_info CSL_REG64_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_RAW
 .sym_meta_info CSL_REG32_RD_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158230

.global CSL_REG16_RD_RAW
 .sym_meta_info CSL_REG16_RD_RAW, "of_placement", "local", 158230

.global CSL_REG16_WR_RAW
 .sym_meta_info CSL_REG16_WR_RAW, "of_placement", "local", 158230

.global CSL_REG8_RD_RAW
 .sym_meta_info CSL_REG8_RD_RAW, "of_placement", "local", 158230

.global CSL_REG8_WR_RAW
 .sym_meta_info CSL_REG8_WR_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_OFF_RAW
 .sym_meta_info CSL_REG32_RD_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_OFF_RAW
 .sym_meta_info CSL_REG32_WR_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_FEXT_RAW
 .sym_meta_info CSL_REG32_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG16_FEXT_RAW
 .sym_meta_info CSL_REG16_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG8_FEXT_RAW
 .sym_meta_info CSL_REG8_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG32_FINS_RAW
 .sym_meta_info CSL_REG32_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG16_FINS_RAW
 .sym_meta_info CSL_REG16_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG8_FINS_RAW
 .sym_meta_info CSL_REG8_FINS_RAW, "of_placement", "local", 158230

.global HW_RD_REG32_RAW
 .sym_meta_info HW_RD_REG32_RAW, "of_placement", "local", 158230

.global HW_WR_REG32_RAW
 .sym_meta_info HW_WR_REG32_RAW, "of_placement", "local", 158230

.global HW_RD_REG16_RAW
 .sym_meta_info HW_RD_REG16_RAW, "of_placement", "local", 158230

.global HW_WR_REG16_RAW
 .sym_meta_info HW_WR_REG16_RAW, "of_placement", "local", 158230

.global HW_RD_REG8_RAW
 .sym_meta_info HW_RD_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_REG8_RAW
 .sym_meta_info HW_WR_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD32_RAW
 .sym_meta_info HW_WR_FIELD32_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD16_RAW
 .sym_meta_info HW_WR_FIELD16_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD8_RAW
 .sym_meta_info HW_WR_FIELD8_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD32_RAW
 .sym_meta_info HW_RD_FIELD32_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD16_RAW
 .sym_meta_info HW_RD_FIELD16_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD8_RAW
 .sym_meta_info HW_RD_FIELD8_RAW, "of_placement", "local", 158230

.global UART_Params_init
 .sym_meta_info UART_Params_init, "of_placement", "local", 158230

.global UART_Transaction_init
 .sym_meta_info UART_Transaction_init, "of_placement", "local", 158230

.global UART_putChar
 .sym_meta_info UART_putChar, "of_placement", "local", 158230

.global UART_getChar
 .sym_meta_info UART_getChar, "of_placement", "local", 158230

.global UART_intrEnable
 .sym_meta_info UART_intrEnable, "of_placement", "local", 158230

.global UART_intrDisable
 .sym_meta_info UART_intrDisable, "of_placement", "local", 158230

.global UART_intr2Enable
 .sym_meta_info UART_intr2Enable, "of_placement", "local", 158230

.global UART_intr2Disable
 .sym_meta_info UART_intr2Disable, "of_placement", "local", 158230

.global UART_getIntrIdentityStatus
 .sym_meta_info UART_getIntrIdentityStatus, "of_placement", "local", 158230

.global UART_getIntr2Status
 .sym_meta_info UART_getIntr2Status, "of_placement", "local", 158230

.global UART_checkCharsAvailInFifo
 .sym_meta_info UART_checkCharsAvailInFifo, "of_placement", "local", 158230

.global UART_readLineStatus
 .sym_meta_info UART_readLineStatus, "of_placement", "local", 158230

.global UART_getCharFifo
 .sym_meta_info UART_getCharFifo, "of_placement", "local", 158230

.global System_deinit
 .sym_meta_info System_deinit, "of_placement", "local", 158230

.global CSL_REG64_RD_RAW
 .sym_meta_info CSL_REG64_RD_RAW, "of_placement", "local", 158230

.global CSL_REG64_WR_RAW
 .sym_meta_info CSL_REG64_WR_RAW, "of_placement", "local", 158230

.global CSL_REG64_FEXT_RAW
 .sym_meta_info CSL_REG64_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG64_FINS_RAW
 .sym_meta_info CSL_REG64_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_RAW
 .sym_meta_info CSL_REG32_RD_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158230

.global CSL_REG16_RD_RAW
 .sym_meta_info CSL_REG16_RD_RAW, "of_placement", "local", 158230

.global CSL_REG16_WR_RAW
 .sym_meta_info CSL_REG16_WR_RAW, "of_placement", "local", 158230

.global CSL_REG8_RD_RAW
 .sym_meta_info CSL_REG8_RD_RAW, "of_placement", "local", 158230

.global CSL_REG8_WR_RAW
 .sym_meta_info CSL_REG8_WR_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_OFF_RAW
 .sym_meta_info CSL_REG32_RD_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_OFF_RAW
 .sym_meta_info CSL_REG32_WR_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_FEXT_RAW
 .sym_meta_info CSL_REG32_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG16_FEXT_RAW
 .sym_meta_info CSL_REG16_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG8_FEXT_RAW
 .sym_meta_info CSL_REG8_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG32_FINS_RAW
 .sym_meta_info CSL_REG32_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG16_FINS_RAW
 .sym_meta_info CSL_REG16_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG8_FINS_RAW
 .sym_meta_info CSL_REG8_FINS_RAW, "of_placement", "local", 158230

.global HW_RD_REG32_RAW
 .sym_meta_info HW_RD_REG32_RAW, "of_placement", "local", 158230

.global HW_WR_REG32_RAW
 .sym_meta_info HW_WR_REG32_RAW, "of_placement", "local", 158230

.global HW_RD_REG16_RAW
 .sym_meta_info HW_RD_REG16_RAW, "of_placement", "local", 158230

.global HW_WR_REG16_RAW
 .sym_meta_info HW_WR_REG16_RAW, "of_placement", "local", 158230

.global HW_RD_REG8_RAW
 .sym_meta_info HW_RD_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_REG8_RAW
 .sym_meta_info HW_WR_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD32_RAW
 .sym_meta_info HW_WR_FIELD32_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD16_RAW
 .sym_meta_info HW_WR_FIELD16_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD8_RAW
 .sym_meta_info HW_WR_FIELD8_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD32_RAW
 .sym_meta_info HW_RD_FIELD32_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD16_RAW
 .sym_meta_info HW_RD_FIELD16_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD8_RAW
 .sym_meta_info HW_RD_FIELD8_RAW, "of_placement", "local", 158230

.global UART_Params_init
 .sym_meta_info UART_Params_init, "of_placement", "local", 158230

.global UART_Transaction_init
 .sym_meta_info UART_Transaction_init, "of_placement", "local", 158230

.global UART_putChar
 .sym_meta_info UART_putChar, "of_placement", "local", 158230

.global UART_getChar
 .sym_meta_info UART_getChar, "of_placement", "local", 158230

.global UART_intrEnable
 .sym_meta_info UART_intrEnable, "of_placement", "local", 158230

.global UART_intrDisable
 .sym_meta_info UART_intrDisable, "of_placement", "local", 158230

.global UART_intr2Enable
 .sym_meta_info UART_intr2Enable, "of_placement", "local", 158230

.global UART_intr2Disable
 .sym_meta_info UART_intr2Disable, "of_placement", "local", 158230

.global UART_getIntrIdentityStatus
 .sym_meta_info UART_getIntrIdentityStatus, "of_placement", "local", 158230

.global UART_getIntr2Status
 .sym_meta_info UART_getIntr2Status, "of_placement", "local", 158230

.global UART_checkCharsAvailInFifo
 .sym_meta_info UART_checkCharsAvailInFifo, "of_placement", "local", 158230

.global UART_readLineStatus
 .sym_meta_info UART_readLineStatus, "of_placement", "local", 158230

.global UART_getCharFifo
 .sym_meta_info UART_getCharFifo, "of_placement", "local", 158230

.global Drivers_close
 .sym_meta_info Drivers_close, "of_placement", "local", 158230

.global Drivers_uartClose
 .sym_meta_info Drivers_uartClose, "of_placement", "local", 158230

.global CSL_REG64_RD_RAW
 .sym_meta_info CSL_REG64_RD_RAW, "of_placement", "local", 158230

.global CSL_REG64_WR_RAW
 .sym_meta_info CSL_REG64_WR_RAW, "of_placement", "local", 158230

.global CSL_REG64_FEXT_RAW
 .sym_meta_info CSL_REG64_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG64_FINS_RAW
 .sym_meta_info CSL_REG64_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_RAW
 .sym_meta_info CSL_REG32_RD_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158230

.global CSL_REG16_RD_RAW
 .sym_meta_info CSL_REG16_RD_RAW, "of_placement", "local", 158230

.global CSL_REG16_WR_RAW
 .sym_meta_info CSL_REG16_WR_RAW, "of_placement", "local", 158230

.global CSL_REG8_RD_RAW
 .sym_meta_info CSL_REG8_RD_RAW, "of_placement", "local", 158230

.global CSL_REG8_WR_RAW
 .sym_meta_info CSL_REG8_WR_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_OFF_RAW
 .sym_meta_info CSL_REG32_RD_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_OFF_RAW
 .sym_meta_info CSL_REG32_WR_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_FEXT_RAW
 .sym_meta_info CSL_REG32_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG16_FEXT_RAW
 .sym_meta_info CSL_REG16_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG8_FEXT_RAW
 .sym_meta_info CSL_REG8_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG32_FINS_RAW
 .sym_meta_info CSL_REG32_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG16_FINS_RAW
 .sym_meta_info CSL_REG16_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG8_FINS_RAW
 .sym_meta_info CSL_REG8_FINS_RAW, "of_placement", "local", 158230

.global HW_RD_REG32_RAW
 .sym_meta_info HW_RD_REG32_RAW, "of_placement", "local", 158230

.global HW_WR_REG32_RAW
 .sym_meta_info HW_WR_REG32_RAW, "of_placement", "local", 158230

.global HW_RD_REG16_RAW
 .sym_meta_info HW_RD_REG16_RAW, "of_placement", "local", 158230

.global HW_WR_REG16_RAW
 .sym_meta_info HW_WR_REG16_RAW, "of_placement", "local", 158230

.global HW_RD_REG8_RAW
 .sym_meta_info HW_RD_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_REG8_RAW
 .sym_meta_info HW_WR_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD32_RAW
 .sym_meta_info HW_WR_FIELD32_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD16_RAW
 .sym_meta_info HW_WR_FIELD16_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD8_RAW
 .sym_meta_info HW_WR_FIELD8_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD32_RAW
 .sym_meta_info HW_RD_FIELD32_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD16_RAW
 .sym_meta_info HW_RD_FIELD16_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD8_RAW
 .sym_meta_info HW_RD_FIELD8_RAW, "of_placement", "local", 158230

.global UART_Params_init
 .sym_meta_info UART_Params_init, "of_placement", "local", 158230

.global UART_Transaction_init
 .sym_meta_info UART_Transaction_init, "of_placement", "local", 158230

.global UART_putChar
 .sym_meta_info UART_putChar, "of_placement", "local", 158230

.global UART_getChar
 .sym_meta_info UART_getChar, "of_placement", "local", 158230

.global UART_intrEnable
 .sym_meta_info UART_intrEnable, "of_placement", "local", 158230

.global UART_intrDisable
 .sym_meta_info UART_intrDisable, "of_placement", "local", 158230

.global UART_intr2Enable
 .sym_meta_info UART_intr2Enable, "of_placement", "local", 158230

.global UART_intr2Disable
 .sym_meta_info UART_intr2Disable, "of_placement", "local", 158230

.global UART_getIntrIdentityStatus
 .sym_meta_info UART_getIntrIdentityStatus, "of_placement", "local", 158230

.global UART_getIntr2Status
 .sym_meta_info UART_getIntr2Status, "of_placement", "local", 158230

.global UART_checkCharsAvailInFifo
 .sym_meta_info UART_checkCharsAvailInFifo, "of_placement", "local", 158230

.global UART_readLineStatus
 .sym_meta_info UART_readLineStatus, "of_placement", "local", 158230

.global UART_getCharFifo
 .sym_meta_info UART_getCharFifo, "of_placement", "local", 158230

.global Board_deinit
 .sym_meta_info Board_deinit, "of_placement", "local", 158230

.global CSL_REG64_RD_RAW
 .sym_meta_info CSL_REG64_RD_RAW, "of_placement", "local", 158230

.global CSL_REG64_WR_RAW
 .sym_meta_info CSL_REG64_WR_RAW, "of_placement", "local", 158230

.global CSL_REG64_FEXT_RAW
 .sym_meta_info CSL_REG64_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG64_FINS_RAW
 .sym_meta_info CSL_REG64_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_RAW
 .sym_meta_info CSL_REG32_RD_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158230

.global CSL_REG16_RD_RAW
 .sym_meta_info CSL_REG16_RD_RAW, "of_placement", "local", 158230

.global CSL_REG16_WR_RAW
 .sym_meta_info CSL_REG16_WR_RAW, "of_placement", "local", 158230

.global CSL_REG8_RD_RAW
 .sym_meta_info CSL_REG8_RD_RAW, "of_placement", "local", 158230

.global CSL_REG8_WR_RAW
 .sym_meta_info CSL_REG8_WR_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_OFF_RAW
 .sym_meta_info CSL_REG32_RD_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_OFF_RAW
 .sym_meta_info CSL_REG32_WR_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_FEXT_RAW
 .sym_meta_info CSL_REG32_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG16_FEXT_RAW
 .sym_meta_info CSL_REG16_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG8_FEXT_RAW
 .sym_meta_info CSL_REG8_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG32_FINS_RAW
 .sym_meta_info CSL_REG32_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG16_FINS_RAW
 .sym_meta_info CSL_REG16_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG8_FINS_RAW
 .sym_meta_info CSL_REG8_FINS_RAW, "of_placement", "local", 158230

.global HW_RD_REG32_RAW
 .sym_meta_info HW_RD_REG32_RAW, "of_placement", "local", 158230

.global HW_WR_REG32_RAW
 .sym_meta_info HW_WR_REG32_RAW, "of_placement", "local", 158230

.global HW_RD_REG16_RAW
 .sym_meta_info HW_RD_REG16_RAW, "of_placement", "local", 158230

.global HW_WR_REG16_RAW
 .sym_meta_info HW_WR_REG16_RAW, "of_placement", "local", 158230

.global HW_RD_REG8_RAW
 .sym_meta_info HW_RD_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_REG8_RAW
 .sym_meta_info HW_WR_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD32_RAW
 .sym_meta_info HW_WR_FIELD32_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD16_RAW
 .sym_meta_info HW_WR_FIELD16_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD8_RAW
 .sym_meta_info HW_WR_FIELD8_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD32_RAW
 .sym_meta_info HW_RD_FIELD32_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD16_RAW
 .sym_meta_info HW_RD_FIELD16_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD8_RAW
 .sym_meta_info HW_RD_FIELD8_RAW, "of_placement", "local", 158230

.global UART_Params_init
 .sym_meta_info UART_Params_init, "of_placement", "local", 158230

.global UART_Transaction_init
 .sym_meta_info UART_Transaction_init, "of_placement", "local", 158230

.global UART_putChar
 .sym_meta_info UART_putChar, "of_placement", "local", 158230

.global UART_getChar
 .sym_meta_info UART_getChar, "of_placement", "local", 158230

.global UART_intrEnable
 .sym_meta_info UART_intrEnable, "of_placement", "local", 158230

.global UART_intrDisable
 .sym_meta_info UART_intrDisable, "of_placement", "local", 158230

.global UART_intr2Enable
 .sym_meta_info UART_intr2Enable, "of_placement", "local", 158230

.global UART_intr2Disable
 .sym_meta_info UART_intr2Disable, "of_placement", "local", 158230

.global UART_getIntrIdentityStatus
 .sym_meta_info UART_getIntrIdentityStatus, "of_placement", "local", 158230

.global UART_getIntr2Status
 .sym_meta_info UART_getIntr2Status, "of_placement", "local", 158230

.global UART_checkCharsAvailInFifo
 .sym_meta_info UART_checkCharsAvailInFifo, "of_placement", "local", 158230

.global UART_readLineStatus
 .sym_meta_info UART_readLineStatus, "of_placement", "local", 158230

.global UART_getCharFifo
 .sym_meta_info UART_getCharFifo, "of_placement", "local", 158230

.global Board_driversClose
 .sym_meta_info Board_driversClose, "of_placement", "local", 158230

.global CSL_REG64_RD_RAW
 .sym_meta_info CSL_REG64_RD_RAW, "of_placement", "local", 158230

.global CSL_REG64_WR_RAW
 .sym_meta_info CSL_REG64_WR_RAW, "of_placement", "local", 158230

.global CSL_REG64_FEXT_RAW
 .sym_meta_info CSL_REG64_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG64_FINS_RAW
 .sym_meta_info CSL_REG64_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_RAW
 .sym_meta_info CSL_REG32_RD_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_RAW
 .sym_meta_info CSL_REG32_WR_RAW, "of_placement", "local", 158230

.global CSL_REG16_RD_RAW
 .sym_meta_info CSL_REG16_RD_RAW, "of_placement", "local", 158230

.global CSL_REG16_WR_RAW
 .sym_meta_info CSL_REG16_WR_RAW, "of_placement", "local", 158230

.global CSL_REG8_RD_RAW
 .sym_meta_info CSL_REG8_RD_RAW, "of_placement", "local", 158230

.global CSL_REG8_WR_RAW
 .sym_meta_info CSL_REG8_WR_RAW, "of_placement", "local", 158230

.global CSL_REG32_RD_OFF_RAW
 .sym_meta_info CSL_REG32_RD_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_WR_OFF_RAW
 .sym_meta_info CSL_REG32_WR_OFF_RAW, "of_placement", "local", 158230

.global CSL_REG32_FEXT_RAW
 .sym_meta_info CSL_REG32_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG16_FEXT_RAW
 .sym_meta_info CSL_REG16_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG8_FEXT_RAW
 .sym_meta_info CSL_REG8_FEXT_RAW, "of_placement", "local", 158230

.global CSL_REG32_FINS_RAW
 .sym_meta_info CSL_REG32_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG16_FINS_RAW
 .sym_meta_info CSL_REG16_FINS_RAW, "of_placement", "local", 158230

.global CSL_REG8_FINS_RAW
 .sym_meta_info CSL_REG8_FINS_RAW, "of_placement", "local", 158230

.global HW_RD_REG32_RAW
 .sym_meta_info HW_RD_REG32_RAW, "of_placement", "local", 158230

.global HW_WR_REG32_RAW
 .sym_meta_info HW_WR_REG32_RAW, "of_placement", "local", 158230

.global HW_RD_REG16_RAW
 .sym_meta_info HW_RD_REG16_RAW, "of_placement", "local", 158230

.global HW_WR_REG16_RAW
 .sym_meta_info HW_WR_REG16_RAW, "of_placement", "local", 158230

.global HW_RD_REG8_RAW
 .sym_meta_info HW_RD_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_REG8_RAW
 .sym_meta_info HW_WR_REG8_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD32_RAW
 .sym_meta_info HW_WR_FIELD32_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD16_RAW
 .sym_meta_info HW_WR_FIELD16_RAW, "of_placement", "local", 158230

.global HW_WR_FIELD8_RAW
 .sym_meta_info HW_WR_FIELD8_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD32_RAW
 .sym_meta_info HW_RD_FIELD32_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD16_RAW
 .sym_meta_info HW_RD_FIELD16_RAW, "of_placement", "local", 158230

.global HW_RD_FIELD8_RAW
 .sym_meta_info HW_RD_FIELD8_RAW, "of_placement", "local", 158230

.global UART_Params_init
 .sym_meta_info UART_Params_init, "of_placement", "local", 158230

.global UART_Transaction_init
 .sym_meta_info UART_Transaction_init, "of_placement", "local", 158230

.global UART_putChar
 .sym_meta_info UART_putChar, "of_placement", "local", 158230

.global UART_getChar
 .sym_meta_info UART_getChar, "of_placement", "local", 158230

.global UART_intrEnable
 .sym_meta_info UART_intrEnable, "of_placement", "local", 158230

.global UART_intrDisable
 .sym_meta_info UART_intrDisable, "of_placement", "local", 158230

.global UART_intr2Enable
 .sym_meta_info UART_intr2Enable, "of_placement", "local", 158230

.global UART_intr2Disable
 .sym_meta_info UART_intr2Disable, "of_placement", "local", 158230

.global UART_getIntrIdentityStatus
 .sym_meta_info UART_getIntrIdentityStatus, "of_placement", "local", 158230

.global UART_getIntr2Status
 .sym_meta_info UART_getIntr2Status, "of_placement", "local", 158230

.global UART_checkCharsAvailInFifo
 .sym_meta_info UART_checkCharsAvailInFifo, "of_placement", "local", 158230

.global UART_readLineStatus
 .sym_meta_info UART_readLineStatus, "of_placement", "local", 158230

.global UART_getCharFifo
 .sym_meta_info UART_getCharFifo, "of_placement", "local", 158230

.global __mpu_init
 .sym_meta_info __mpu_init, "of_placement", "local", 158230

 #endif