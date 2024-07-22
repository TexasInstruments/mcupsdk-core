# FreeRTOS usage guidelines {#KERNEL_FREERTOS_IMPORTANT_GUIDELINES_PAGE}

[TOC]

This section has additional useful information related to FreeRTOS and its integration within MCU+ SDK.
**It also compares some key points and config in FreeRTOS vs TI SysBIOS RTOS which was used in previous TI SOCs and SDKs.**

## FreeRTOS configuration

- In FreeRTOS, all applications need to provide a `FreeRTOSConfig.h` file which specifies the FreeRTOS kernel configuration, see
https://www.freertos.org/a00110.html
- All source code using FreeRTOS APIs, including FreeRTOS kernel itself MUST include “FreeRTOSConfig.h” before any other FreeRTOS .h files
- Configuration includes things like below
  - Task scheduling options, Timer tick resolution, Number of priorities, Debug and trace hooks
  - Enable / disable RTOS features, mainly task, mutex, recursive mutex, semaphore, etc
  - Default sizes for task stack, timer task, idle task etc
- To allow support of pre-built libraries, MCU+ SDK has a default predefined `FreeRTOSConfig.h` per SOC and per CPU type
- In general extreme fine tuning of FreeRTOS config is not needed and a predefined config per SOC and CPU type would meet almost all use-cases and applications.
- However users can modify this config if needed. When FreeRTOS config is changed, all pre-built libraries have to be recompiled for the changes to take effect (see \ref MAKEFILE_BUILD_PAGE)
- Also many config options are related to inclusion/exclusion of RTOS modules to save code/data size. However we can rely on compiler to optimize out functions that are not called by applications.
- The predefined config file per SOC and per CPU can be found at below path

        ${SDK_INSALL_PATH}/source/kernel/freertos/config/{soc}/{cpu}/FreeRTOSConfig.h

## Using and adding FreeRTOS to your project
- You can start using FreeRTOS using one of the many example projects in the SDK. The \ref EXAMPLES_EMPTY at
    `${SDK_INSTALL_PATH}/examples/empty` is a good starting point

- Given below are some details to add FreeRTOS to a project if you decide to start from scratch

\cond !SOC_AM62X
- Below paths must be added to your project include path to use FreeRTOS Kernel APIs- (below shows example for R5F)

        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/@VAR_SOC_NAME_LOWER/r5f
\endcond
\cond SOC_AM62X
- Below paths must be added to your project include path to use FreeRTOS Kernel APIs- (below shows example for M4F)

        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/@VAR_SOC_NAME_LOWER/m4f
\endcond
- To use FreeRTOS+POSIX, you need to add the below additional paths

        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-POSIX/include
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-POSIX/include/private
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include
        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable

- To link to FreeRTOS library, add below path to you library path

        ${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib

- And link to below library for your SOC and CPU of choice

        freertos.{soc}.{cpu}.{compiler}.{profile}.lib

- Before using FreeRTOS APIs in your application, you need to do below
  - Initialize cache, MPU, HW interrupt as needed.
  - Setup one timer to tick at a frequency defined in `FreeRTOSConfig.h`, typically 1 ms
  - Create at least one task
  - Finally call `vTaskStartScheduler` to start FreeRTOS.
  - This starts the FreeRTOS schedular and it jumps to the highest priority task previously created.
  - More tasks and OS resources like semaphores can be created from within this initial task.

- Once again refer to the many FreeRTOS examples to see the project settings and startup code.

## Summary of comparison with SysBIOS

This section has a quick summary of comparision between SysBIOS and FreeRTOS features.
Refer rest of the sections on this page for more details

### Key similarities between modules

SysBIOS Module          | FreeRTOS Module       | Additional Remarks
------------------------|-----------------------|------------------------------------
XDC based config        | FreeRTOSConfig.h      | `#define` based config, most code in C files for FreeRTOS
Static alloc            | Static alloc          | “construct” APIs in SysBIOS, “CreateStatic” APIs in FreeRTOS
Task                    | Task                  | Same scheduling policy as SysBIOS
Semaphore               | Semaphore, Mutex      | Same as SysBIOS. Binary, counting semaphore’s, recursive, priority inheritance mutex’s
Clock                   | Timer                 | Similar features, see details below
Event                   | Event                 | Similar features, see details below
Idle                    | Idle                  | Only one idle hook in FreeRTOS
Queue                   | List                  | Similar features, see details below
Mailbox                 | Queue                 | Similar features, see details below
Hwi                     | HwiP in DPL           | Similar features, see details below
Cache                   | CacheP in DPL         | Similar features, see details below
HeapMem                 | HeapP in DPL          | Similar features, see details below
MPU/MMU                 | MPU/MMU in DPL        | Similar features, see details below
Swi                     | “Pend” function call  | Similar features, see details below
Load                    | Run-time stats APIs   | Similar features, see details below
POSIX                   | FreeRTOS+POSIX        | Similar features, see details below

### Key differences between modules

<table>
<tr>
    <th>FreeRTOS difference vs SysBIOS
    <th>Alternative in FreeRTOS
</tr>
<tr>
    <td> When `configUSE_TIME_SLICING` is 1, FreeRTOS will time slice between tasks at the same priority
    <td> Keep this at 0 (**SDK default config**), if this is not desired
</tr>
<tr>
    <td> Timer callbacks are called in task context vs ISR context in SysBIOS
    <td> Keep the timer task to highest priority so that timer callback gets called immediately after all ISRs are done (**SDK default config**)
</tr>
<tr>
    <td> No SWI support in FreeRTOS
    <td> Use "pend" function call, where functions are executed in timer task context. Again keep timer task as highest priority (**SDK default config**), see `xTimerPendFunctionCall()` and `xTimerPendFunctionCallFromISR()`
</tr>
<tr>
    <td> No ability to create arbitrary heaps
    <td> SDK DPL provides APIs to create arbitrary heaps to match SysBIOS (see \ref KERNEL_DPL_HEAP_PAGE)
</tr>
<tr>
    <td> Only FreeRTOS APIs ending with `FromISR()` can be called from within ISRs
    <td> SDK DPL provides a function `HwiP_inISR()` to detect if code is in ISR. This can be used to keep the application functions generic. SDK DPL takes care of this by default
</tr>
<tr>
    <td> Number of task priorities is configurable (upto max 32) vs SysBIOS default of 16
    <td> **SDK default config** keeps number of task priorities to 16 in FreeRTOS
</tr>
</table>

## FreeRTOS Tasks

### Overview

- FreeRTOS has ability to create tasks with below parameters
  - Entry function
  - One `void *` entry function argument
  - Stack memory (when NULL, FreeRTOS uses the default heap to allocate the stack memory)
\cond SOC_AM64X
  - Stack size in units of “stack words”, i.e 32b or 4 bytes in case of R5F, M4F and 8 bytes in case of A53
\endcond
\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294x || SOC_AM261X
  - Stack size in units of “stack words”, i.e 32b or 4 bytes in case of R5F and M4F
\endcond
\cond SOC_AM62X
  - Stack size in units of ??stack words??, i.e 32b or 4 bytes for M4F
\endcond
  - Priority (0 is lowest, `configMAX_PRIORITY-1` is the highest)
    - **SDK default config is 16 priorities to match SysBIOS priorities**
- Static task object memory allocation is supported
  - SDK examples and DPL by default use static alloc APIs
- Scheduler policy is configurable
  - `configUSE_PREEMPTION`, **SDK default is 1 to match SysBIOS scheduler policy**
    - This enables pre-emptive priority based scheduling
  - `configUSE_TIME_SLICING`, **SDK default is 0, i.e disabled, to match SysBIOS scheduler policy**
    - When enabled, when two tasks are of the same priority, FreeRTOS will time-slice between the tasks at units of timer tick
    - **NOTE, this is unlike SysBIOS where unless `Task_yeild` is called scheduler will not switch to another task at same priority**

### Initial Tasks

- Two tasks are created inside FreeRTOS on startup, idle task and timer task
- Idle task is similar to any other task, only it runs at lowest priority.
  - User can configure a “hook” function to call inside of IDLE, e.g, WFI can be called here.
\cond SOC_AM64X
  - SDK default config is shown below, `vApplicationIdleHook` calls `wfi` instruction for R5F, M4F, A53.
\endcond
\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294x || SOC_AM261X
  - SDK default config is shown below, `vApplicationIdleHook` calls `wfi` instruction for R5F, M4F.
\endcond
\cond SOC_AM62X
  - SDK default config is shown below, `vApplicationIdleHook` calls `wfi` instruction for M4F.
\endcond
    \code
    /* in FreeRTOSConfig.h file */
    /* when 1, make sure to implement void vApplicationIdleHook(void) as the hook function */
    #define configUSE_IDLE_HOOK                     (1)
    /* in port.c file for R5F, M4F */
    void vApplicationIdleHook( void )
    {
        __asm__ volatile ("wfi");
    }
    \endcode

- Timer task (**SDK default config enables timer task to match SysBIOS**),
  - Timer callbacks and “deferred” interrupt handler functions are called in context of the timer task
  - **NOTE, unlike SysBIOS, SW timer callbacks are not called in ISR context**
  - “deferred”  handler functions, can be used as equivalent of SW interrupt of SysBIOS
  - **NOTE, unlike SysBIOS, there is no SWI module in FreeRTOS**
  - SDK config keeps the priority of this task to highest so that timer function will run immediately after a ISR
  - All of above can be controlled via FreeRTOS Config, SDK defaults are listed below

    \code
    #define configUSE_TIMERS               (1) /* enable timer task and SW timers */
    #define configTIMER_TASK_PRIORITY      (configMAX_PRIORITIES - 1) /* highest priority */
    #define configTIMER_TASK_STACK_DEPTH   (256) /* 1KB */
    \endcode

### Task load

- To measure task and CPU load, in FreeRTOS config, we need to set `configGENERATE_RUN_TIME_STATS` to `1` (**SDK default** is `1` when `configOPTIMIZE_FOR_LATENCY` is `0`)
- When load measurement is enabled, `portGET_RUN_TIME_COUNTER_VALUE()` is called to get a high resolution timer counter value.
- In MCU+ SDK porting layer, we implement `portGET_RUN_TIME_COUNTER_VALUE()` using \ref ClockP_getTimeUsec to return time stamp in units of usecs
- In FreeRTOS Kernel, the load measurement counter will overflow after 32b, i.e around 1 hr with usec resolution timer.
- To avoid this, in FreeRTOS DPL, a periodic load update API is called in IDLE task to accumulate the measured task load and take care of overflow condition.
- To get task load with overflow condition taken care of, it is recommended to create the task using \ref KERNEL_DPL_TASK_PAGE DPL APIs
- Total CPU load is calculated by measuring the time spent in IDLE task and then subtracting it from 100%, i.e CPU load = 100 - CPU idle load

### Important tips for application writers

- Task function should not return in FreeRTOS, instead it should call vTaskDelete(NULL) to destroy itself, e.g
    \code
    void myTaskMain(void *args)
    {
        /* ... do something ... */

        vTaskDelete(NULL);
        /* do not call “return” */
    }
    \endcode
- FreeRTOS schedular, by default task switch does not save/restore FPU (floating point unit) registers, tasks which need FPU need to call `portTASK_USES_FLOATING_POINT`
  once before using FPU operations. If in doubt always call this function
- ISR handler may or may not save FPU state depending on the CPU, see \ref FREERTOS_SUPPORTED_FEATURES to check if your CPU of interest supports FPU save/restore
- On task delete, FreeRTOS will free any memory allocated internally, if dynamically memory allocation mode was used.
  This memory free is done in “IDLE” task, so “IDLE” needs to get opportunity to run at some point.
- `vTaskStartScheduler()` starts FreeRTOS and the highest priority task is executed first.
  - Tasks, semaphores and OS objects can be created before `vTaskStartScheduler()` is called.
  - Recommend to create one task and then call `vTaskStartScheduler()` and then do everything from within that task including creating other tasks.
  - ** `vTaskStartScheduler()` is similar to `BIOS_start()`, i.e it never returns and switches to the created tasks or idle task**
- .stack is the stack used by code before FreeRTOS scheduler is started, i.e from `_c_int00` to `vTaskStartSchedular()`

## FreeRTOS Interrupts

### Overview

- Interrupt handler is not directly invoked by FreeRTOS Kernel, porting layer handles this
  - In theory, FreeRTOS can work without interrupts. To switch tasks, interrupts are not needed. However that’s not very useful.
- Porting layer does below
  - Setup (any) one timer to be configured at `configTICK_RATE_HZ`, typically 1ms
\cond SOC_AM64X
  - For R5F, A53, we use one of the SOC level general purpose DM timers.
\endcond
\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294x || SOC_AM261X
  - For R5F we use one of the SOC level general purpose DM timers.
\endcond
  - For M4F, we use the M4F internal SysTick timer.
  - Timer ISR is outside FreeRTOS kernel
  - When the timer ISR happens the porting layer calls `xTaskIncrementTick()` FreeRTOS API to maintain FreeRTOS timer tick state
- Porting layer also implements the common interrupt entry and exit logic
  - Esp before interrupt exit, porting layer needs to invoke a task switch if during ISR handling
    a semaphore was posted that needed a task switch on ISR exit
- Interrupt nesting is also taken care by this common interrupt handler
- Porting layer also implements functions to protect critical sections of FreeRTOS via below APIs
  - `portENTER_CRITICAL`, disable interrupt, track nesting of calls
  - `portEXIT_CRITICAL`, reenable interrupt, if nesting call depth is 0
  - `portSET_INTERRUPT_MASK_FROM_ISR`, interrupt disable and return old interrupt state
  - `portCLEAR_INTERRUPT_MASK_FROM_ISR`, restore interrupt state

### Nested interrupts and ISR stack

\cond !SOC_AM62X
- On R5F,
  - When a interrupt is triggered, the CPU switches to IRQ mode and uses IRQ stack.
  - IRQ interrupt are disabled by HW at this point.
  - In the ISR handler, some CPU state is saved to IRQ stack and mode is switched to SVC mode and therefore SVC stack
  - IRQs are then enabled, i.e nested interrupts are enabled
  - The user ISR code is executed
  - At this point more IRQs can occur to interrupt the user ISR code
  - After all IRQs are handled, IRQ is disabled
  - Now mode is switched back to IRQ mode
  - If a task switch was requested by any of the ISRs, tasks are switched and control returns from IRQ to the highest priority pending task
  - If a task switch was NOT requested, control returns from IRQ to the interrupted task
- On R5F,
  - Thus the user ISR code get called in context of SVC stack, so size of SVC stack MUST be kept large enough to handle worst case ISR requirement.
  - The IRQ stack size itself can be kept small since it only saves few bytes (8 bytes) of state for every nested IRQ invocation.
  - SDK default examples keep IRQ stack as 256 bytes and SVC stack size as 4096 bytes.
  - This stack size is specified in the linker command file, a sample snippet is shown below,
    \code
    __IRQ_STACK_SIZE = 256;
    /* This is the size of stack when R5 is in IRQ mode
    * - In both NORTOS and FreeRTOS nesting is disabled for FIQ
    */
    __FIQ_STACK_SIZE = 256;
    __SVC_STACK_SIZE = 4096; /* This is the size of stack when R5 is in SVC mode */
    __ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */
    __UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */
    \endcode
  - **Important NOTE: For R5F, if interrupt nesting is enabled, then FreeRTOSConfig.h should set**
    \code
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION (0)
    \endcode
    This is because of bug MCUSDK-1016 whereby enabling interrupt nesting will cause task scheduling to stop working
    if configUSE_PORT_OPTIMISED_TASK_SELECTION is set to 1
\endcond

- On M4F, nested interrupt work by default without any special handling.
  - M4F provides a `pendSV` exception which when triggered is invoked after all nested ISRs are handled.
  - The porting layer calls FreeRTOS task switch logic in the `pendSV` exception handler

\cond SOC_AM64X
- On A53,
  - The user ISR gets called in context of EL1 stack and is represented by the `.stack` section in the linker command file.
\endcond

### Interrupts outside of FreeRTOS

\cond !SOC_AM62X
- On R5F,
  - When FreeRTOS enter its critical section, it only disables IRQ but not FIQ
  - Hence, FreeRTOS API calls MUST NOT be done inside FIQ.
  - FIQs can be used for extreme low latency interrupt bypassing the OS completely.
  - **NOTE: this is same as the case with SysBIOS**
\endcond

- On M4F,
  - When FreeRTOS enter its critical section, all interrupt levels numerically below `configMAX_SYSCALL_INTERRUPT_PRIORITY` defined in FreeRTOSConfig.h are disabled.
  - So interrupts at numerical priority level above `configMAX_SYSCALL_INTERRUPT_PRIORITY` MUST not use FreeRTOS API calls.
  - SDK default keeps `configMAX_SYSCALL_INTERRUPT_PRIORITY` at `0xE0`, i.e all interrupt priority levels are disabled when FreeRTOS
    critical section is entered.

\cond SOC_AM64X
- On A53,
  - Only IRQ is supported, FIQ is not supported
\endcond

### Additional important tips for application writers

- Only FreeRTOS APIs that end with `FromISR` can be called from ISR context
    \code
    BaseType_t xHigherPriorityTaskWoken = 0;

    xSemaphoreGiveFromISR(pSemaphore->semHndl, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    \endcode

- The `FromISR` API returns a flag `xHigherPriorityTaskWoken` to indicate task switch should be requested,
  `portYIELD_FROM_ISR` requests the task switch. Actual task switch happens after all nested ISRs have executed.

- **This is different vs SysBIOS, where the same APIs can be used inside ISR and outside ISR.**

- The SDK DPL APIs takes care of this internally as below, e.g,
    \code
    SemaphoreP_post(...) {
        if( HwiP_inISR() ) {            
            BaseType_t xHigherPriorityTaskWoken = 0;
            xSemaphoreGiveFromISR(pSemaphore->semHndl, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        } else {
            xSemaphoreGive(pSemaphore->semHndl);
        }
    }
    \endcode

## FreeRTOS Semaphores and Mutex

### Overview

- FreeRTOS has text book implementation of binary semaphore, counting semaphore, mutex with priority inheritance
  and recursive mutex
- Timeouts can be specified when waiting on a semaphore/mutex, with timeout error code if semaphore/mutex is not available
- Timeout is specified in units of OS ticks. Use `pdMS_TO_TICKS()` to convert from msec to OS ticks

### Important tips for application writers

- Binary and counting semaphores should be used to signal from ISR to task and task to task
  - **SDK default config enables binary and counting semaphores**
- Mutex should be used for mutual exclusion of critical sections in applications.
  - **SDK default config enables mutex and recursive mutex's**
- Once again, only APIs that end with `FromISR` can be used from within ISR. These are non-blocking APIs.

## FreeRTOS Task Notification

### Overview

- Task notification is low overhead API to signal a task from ISR or a task from another task
- We see about 25% reduction in cycles needed for task switch using task notification vs using semaphores in ideal standalone conditions.
- This is a very freertos specific API and SDK drivers and DPL will NOT use this.

### Important tips for application writers

- In practical use-cases, typically cache effects due to cache miss on the larger application code/data, will have higher effect than
  overheads of task switch in the smaller OS code.
- So we recommend to NOT use this feature unless really the savings is critical to end application.
- Here one needs to know the task handle to signal, so its difficult to use this API from within a driver library since driver ISR for
  example does not know the task in which the driver API is called
- Thus use these APIs in applications after thorough analysis and profiling on benefit for the use-case.

## FreeRTOS Event Groups

### Overview

- An event group is a set of event bits. Event bits are used to indicate if an event has occurred or not. Event bits are often referred to as event flags.
- The number of bits (or flags) stored within an event group is 24 bits

### Important tips for application writers

- Unlike "Event" in SysBIOS where one could give a "AND" mask and "OR" mask to wait on, in FreeRTOS one can only give one "event mask" and then tell if one
  should use "AND" wait or "OR" wait on the single event mask
- When "set bits" is called from ISR, actual action of setting bits is done within the "Timer task" as a "deferred" function call.

## FreeRTOS Timers

### Overview

- Allows users to create one shot or periodic callbacks at time resolution of one OS tick, typically 1ms.
- **Similar to “Clock” module in SysBIOS**
- A timer task executes the callbacks, this timer task is like any other task, nothing special about it.
 - **SDK default is to set highest priority for this task so that after all ISRs this task will execute**
- A SW queue is used to post callbacks or functions to execute on timer expiry, the queue depth is config option.
  **SDK default is 16**

### Important tips for application writers

- Since a single task executes all the multiple callbacks, the callbacks are recommended to not block and spend too much time,
  even though in theory blocking calls are allowed.
- Use `xTimerPendFunctionCall()` to do “deferred ISR” handling , i.e do the most critical work in ISR and do the rest of work in a another
  function call which gets called right after all ISRs are done. **This feature can be used as alternative to SWI in SysBIOS**
- Note, do not use `FromISR` APIs in callbacks, use regular FreeRTOS APIs

## FreeRTOS Heaps

### Default heap

- FreeRTOS kernel has multiple heap implementations as defined in `portable/MemMang`
- A port should compile one of below `heap_{n}.c` files to pick the required heap implementation
  - `heap_1.c`
    - linear heap allocation, `free` not allowed. Typically used when true dynamic alloc is prohibited.
    - Obsolete now and not used anymore, since FreeRTOS natively support static alloc mode
  - `heap_2.c`
    - Linked list based heap but adjacent free blocks not merged
    - Obsolete now and not used anymore, `heap_4.c` is a better implementation
  - `heap_3.c` **SDK default**
    - Uses compiler provided, `malloc()` and `free()`, FreeRTOS only makes the calls task safe
    - Heap size specified via `--heap` compiler option and placed in `.heap` section in linker command file.
  - `heap_4.c`
    - Linked list based heap with adjacent free block merging
    - More or less similar to `malloc()` from compiler and SysBIOS heap implementation
    - Memory needs to be provided as a global static array, and placed appropriately in linker command file.
  - `heap_5.c`
    - Same as `heap_4.c`, + it allows memory to be specified as multiple memory blocks
      When memory in one block is full, it will alloc from the next block and so on.

### User defined heaps

- **Unlike SysBIOS, FreeRTOS does not support user defined heaps at arbitrary user defined memory locations**
- On TI SOCs with multiple levels of memory like L2, L3, DDR,
  it is very convenient to have ability to create multiple application specified heaps.
- `heap_4.c` almost does this, but it uses a global variable for heap memory base and size
- Therefore, SDK DPL implements heap APIs to match SysBIOS as below
  - SDK has adapted `heap_4.c` to take memory base and size as input parameters during heap create.
    This allows creation of arbitrary number of application heaps to match SysBIOS features
  - `HeapP.h` is the user API (\ref KERNEL_DPL_HEAP_PAGE)
  - `HeapP_freertos.c`, `HeapP_internal.c` is the implementation (Note, FreeRTOS kernel code is not modified)

### Important tips for application writers

- Use `--heap` in linker command file to specify heap size and place `.heap` section in appropriate memory
  in linker command file. This is used by FreeRTOS (`heap_3.c`) when it needs dynamic memory allocation.
- In general recommend to use FreeRTOS static alloc APIs for tasks, semaphores to keep the application deterministic.
  - SDK DPL uses FreeRTOS static alloc APIs
- Use SDK provided `HeapP.h` to create arbitrary application heaps as needed.

## FreeRTOS Additional Modules

### Queues

- Highly efficient SW queue implementation
- Lowest level primitive in FreeRTOS used internally by almost all modules like tasks, semaphores, mutex
- Task/ISR safe
- Blocking/non-blocking with timeouts
- Fixed queue depth and fixed queue element size at queue create
- Can be used by application users

### Queue Sets, Stream Buffers, Co-routines

- Queue Sets, more higher level than queues, built using queues, semaphores. FreeRTOS recommends to use these very carefully. https://www.freertos.org/Pend-on-multiple-rtos-objects.html
- Stream buffers, built using queues, semaphores
- Croutines, used when tasks are deemed too expensive to use, almost unlikely we will ever use these
- **SDK default config keeps these disabled**

## FreeRTOS Hook Functions

### Debug Hook Functions

- FreeRTOS provides "hook" function callbacks specified in FreeRTOS config.
- These callbacks are invoked by FreeRTOS at specific points or when certain conditions are met
- Stack overflow check, `configCHECK_FOR_STACK_OVERFLOW` ( **SDK default enabled** )
  - Enables stack overflow checks, two options to detect overflows,
    - Option 1: Check if stack pointer has gone beyond limit ( **SDK default** )
    - Option 2: Put a pattern at top of stack and if pattern is overwritten then call out stack overflow
    - **NOTE:** Depending on nature of stack overflow, this logic may or may not catch all stack overflows
- Malloc failed hook, `configUSE_MALLOC_FAILED_HOOK` ( **SDK default disabled** )
  - Called when `pvPortMalloc` fails inside FreeRTOS, not very useful, since all APIs return error on memory alloc failure.
- Assert, `configASSERT` ( **SDK default enabled** )
  - Called when unrecoverable assert condition happens inside FreeRTOS or porting layer
- SDK defines a new config flag `configOPTIMIZE_FOR_LATENCY`, this can be used to disable all debug hooks and some other configs in  FreeRTOS config to minimize FreeRTOS overheads due to debug hooks,
- Recommend to enable all hooks in `debug` mode and disable all hooks in `release` mode after applications are reasonably sure assert and overflows will not happen.
  - By default `configOPTIMIZE_FOR_LATENCY` is set to 1, i.e enabled

### Statistics Hook Functions

- FreeRTOS provides APIs to query and return time spent in each task and output the results in a nice formatted table
- The hook functions for this feature are enabled in **SDK default config** when `configOPTIMIZE_FOR_LATENCY` is 0.
- It uses same timer as the tick timer to measure task load.
- See also https://www.freertos.org/rtos-run-time-stats.html

### Trace Hooks Functions

- FreeRTOS provides more than 70+ hook marcos at key points in the kernel to log trace info
- Example,
  - `traceTASK_SWITCHED_IN`, called when a task is being switched into
  - `traceTASK_SWITCHED_OUT`, called when a task is begin switched to another task
- This is used by tools like `Tracealyzer` to log and then visualize task execution and lot more in a GUI tool, see https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_Trace/FreeRTOS_Plus_Trace.html
- **Not enabled in SDK default config**

## FreeRTOS POSIX

### Overview

- Implements a subset of the POSIX threading API, see https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_POSIX/index.html
- Support for below
  - pthread, pthread_cond, pthread_mutex
  - semaphore
  - mqueue
  - timer, clock
- **Features similar to SysBIOS POSIX**
- Available in SDK but optional to use this layer, SDK drivers and SDK DPL APIs does not use POSIX APIs

### Important tips for application writers

- Stack used by threads created with pthread is allocated using  dynamic memory alloc, so make sure heap size is sufficiently large
- mqueue, timer also uses dynamic alloc APIs
- Use this layer if application portability to other POSIX OS is important, else it is recommended to use direct FreeRTOS APIs or SDK DPL APIs




