# FreeRTOS MPU usage guidelines {#KERNEL_FREERTOS_MPU_IMPORTANT_GUIDELINES_PAGE}

[TOC]

This section has additional useful information related to FreeRTOS MPU and its integration within MCU+ SDK.

Refer [FreeRTOS - Memory Protection Unit (MPU) Support](https://www.freertos.org/Security/04-FreeRTOS-MPU-memory-protection-unit) 
for official FreeRTOS documentation.

## Using and adding FreeRTOS MPU to your project
- You can start using FreeRTOS MPU referring to \ref EXAMPLES_KERNEL_FREERTOS_TASK_SWITCH_MPU at
    `${SDK_INSTALL_PATH}/examples/kernel/freertos/task_switch_mpu` 

- Also refer \ref EXAMPLES_KERNEL_DPL_DEMO at `examples/kernel/dpl/dpl_demo` for usage of DPL TaskP API to 
  create user mode tasks.

- Given below are some details to add FreeRTOS MPU to a project if you decide to start from scratch

- Below paths must be added to your project include path to use FreeRTOS Kernel API with MPU support

        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F_MPU
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/@VAR_SOC_NAME_LOWER/r5f_mpu

- To link to FreeRTOS library, add below path to you library path

        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib

- And link to below library for your SOC and CPU of choice

        freertos.{soc}.{cpu}-mpu.{compiler}.{profile}.lib

## Important tips for application writers

- For R5F with total of 16 configurable MPU regions,
  - The first 8 regions (Region 0 - Region 7) are static and can be setup via SysConfig
    - SysConfig generates DPL MpuP config and these are programmed one-time as part of startup code
  - Next region is reserved for task stack (Region 8)
    - Refer [`xTaskCreateRestrictedStatic`](https://www.freertos.org/Documentation/02-Kernel/04-API-references/13-FreeRTOS-MPU-specific/02-xTaskCreateRestrictedStatic)
      API documentation description for `puxStackBuffer` for details on various constraints related to stack
      size and alignment
  - Rest of the 7 regions (Region 9 - Region 15) are task specific and can be provided while creating the task

-  Various FreeRTOS kernel APIS like create/delete tasks/semaphore/events can be used only from privileged 
   tasks and are not available for user mode tasks
   - Refer **System Call restrictions** under "Changes in FreeRTOS version 10.6.0" and 
     "Changes in FreeRTOS version 10.5.0" in
     [FreeRTOS - Memory Protection Unit (MPU) Support - Upgrade Information](https://www.freertos.org/Security/04-FreeRTOS-MPU-memory-protection-unit#upgrade-information)
     for the list of APIs that are not accessible from user mode tasks
   - Hence following DPL APIs which are implemented using these restricted kernel APIs should as well be used
     only from a privileged task,
     - `TaskP_construct`
     - `TaskP_constructRestricted`
     - `TaskP_destruct`
     - `SemaphoreP_constructMutex`
     - `SemaphoreP_constructBinary`
     - `SemaphoreP_constructCounting`
     - `SemaphoreP_destruct`
     - `MailboxP_create`
     - `MailboxP_delete`
     - `EventP_construct`
     - `EventP_destruct`
     - `ClockP_construct`
     - `TaskP_loadGet`
     - `TaskP_loadGetTotalCpuLoad`
     - `TaskP_loadUpdateAll`
     - `TaskP_loadResetAll`
     - `HeapP_alloc`*
     - `HeapP_free`*
     - `HeapP_getHeapStats`*
     - `HeapP_destruct`
     
\note `HeapP_alloc`, `HeapP_free` & `HeapP_getHeapStats` is temporary. 
      This is expected to be updated in future releases such that `HeapP_construct` & `HeapP_destruct` can be 
      used only from a privileged task, whereas `HeapP_alloc`, `HeapP_free` & `HeapP_getHeapStats` will be
      accessible even for user mode tasks.

- RTI registers are read-only in user mode and can be modified only in privileged mode. 
  - Hence various SDK APIs which are impacted due to this performs temporary switch to privileged mode
    when invoked from user mode tasks. This is similar to how FreeRTOS kernel system calls are implemented for
    user mode tasks. Following are the list of SDK APIs,
    - `TimerP_setup`
    - `TimerP_start`
    - `TimerP_stop`
    - `TimerP_clearOverflowInt`

- Performance monitoring registers in Cortex-R5 will be accessible in user mode only if 
  [`c9, User Enable Register`](https://developer.arm.com/documentation/ddi0460/d/Events-and-Performance-Monitor/Performance-monitoring-registers/c9--User-Enable-Register?lang=en)
  `EN` bit is set. This needs to be set from privileged mode.
  - Hence SDK sets this as part of starting the scheduler before switching to the first task. This enables
    user mode tasks to access the PMU registers as well as use various DPL `CycleCounterP` APIs
    (\ref KERNEL_DPL_CYCLE_COUNTER)

- While creating the first task for an application that uses FreeRTOS MPU, make sure to create in privileged
  mode so as to be able to create other tasks, semaphores, etc.

- A privileged task is created by performing a bitwise OR of `portPRIVILEGE_BIT` with the task priority 
  parameter of `xTaskCreateStatic`
  - Refer [`xTaskCreateStatic`](https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/02-xTaskCreateStatic) 
    API documentation description for `uxPriority` parameter for more details

- A user mode task with task specific MPU regions is created using [`xTaskCreateRestrictedStatic`](https://www.freertos.org/Documentation/02-Kernel/04-API-references/13-FreeRTOS-MPU-specific/02-xTaskCreateRestrictedStatic)

- Refer [FreeeRTOS - Memory Protection Unit (MPU) Support - Creating Tasks](https://www.freertos.org/Security/04-FreeRTOS-MPU-memory-protection-unit#creating-tasks)
  for more details

- DPL API `TaskP_construct` can be used to create privileged tasks whereas `TaskP_constructRestricted` can be
  used to create user mode task with task specific MPU regions
  - `TaskP_constructRestricted` API uses DPL MpuP structure `MpuP_RegionAttrs` to define the region attributes
     for the task specific MPU regions, similar to how the static MPU regions are configured
