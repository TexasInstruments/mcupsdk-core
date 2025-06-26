\cond THREADX

# ThreadX usage guide {#KERNEL_THREADX_GUIDE_PAGE}

[TOC]

This section has additional useful information related to ThreadX and its integration within MCU+ SDK.


## ThreadX configuration

- With ThreadX, applications can optionally alter the kernel configuration through the `tx_user.h` header file. In the SDK, the tx_user.h file is always included to prevent build issues with mismatched include files.
- Users can modify this file if needed but all pre-built libraries must be recompiled for the change to take effect. (see \ref MAKEFILE_BUILD_PAGE)
- The `tx_user.h` header files can be found in the following directory:

        ${SDK_INSALL_PATH}/source/kernel/threadx/ports/{target}/inc/tx_user.h

        
## Using and adding ThreadX to your project

You can start using ThreadX using one of the many example projects provided with the SDK. The examples located under `${SDK_INSTALL_PATH}/examples/kernel/threadx/` are a good starting point.
        
To add ThreadX to an empty projet it is necessary to configure the include paths, library and preprocessor defines accordingly. It is recommended to copy the configurations from an example project for the targeted SoC and EVM but we detail below the basic configurations for the R5F on an AM243X SoC.

The following include paths must be added to the project.
    
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/common/inc
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/ports/ti_arm_gcc_clang_cortex_r5/inc
    
Finally the proper library must be included, for the example platform named above the following library should be added.

    threadx.am243x.r5f.ti-arm-clang.debug.lib
    
Alongside the following linker library search path needs to be added.

    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/lib
    
    
## Enabling TraceX

The first step to enable TraceX for the ThreadX kernel is to modify the `tx_user.h` file to enable the trace instrumentation within the kernel. This can be done by uncommenting the following define:

```
/*
#define TX_ENABLE_EVENT_TRACE
*/
```

The SDK libraries must now be rebuilt for the change to take effect.

Following this trace can be enabled in the application. To do so first define a trace buffer area, for example:

```
UCHAR app_trace_buffer[65536];
```

Then at a convenient location, for example from within `tx_application_define` call `tx_trace_enable` to enable the collection of trace data.

```
    status = tx_trace_enable ( &app_trace_buffer, sizeof(app_trace_buffer), 30);
    DebugP_assertNoLog(status == TX_SUCCESS);
```

## Extracting trace data

To visualized trace data it must first be downloaded from the target. You can check the address of the trace buffer and see it's raw content using the Memory Browser as shown below:

  \imageStyle{tracex_buffer_memory_browser.png,width:30%}
  \image html tracex_buffer_memory_browser.png Application trace buffer in memory view.

To instruct the debugger to download the data right click in the memory browser and select "Save Memory...".

  \imageStyle{tracex_buffer_memory_browser_context.png,width:30%}
  \image html tracex_buffer_memory_browser_context.png Memory browser context menu.
  
The "Save Memory" dialog should appear. Select the directory and filename to use and make sure the "Binary" format is selected.
       
  \imageStyle{tracex_buffer_memory_export.png,width:30%}
  \image html tracex_buffer_memory_export.png Save Memory dialog.
  
In the next dialog enter the start address of the trace buffer as shown in the memory browser as well as its length in bytes then click save.

  \imageStyle{tracex_buffer_memory_export2.png,width:30%}
  \image html tracex_buffer_memory_export2.png Save Memory dialog.
  
Finally the raw trace buffer data can be visualized in the TraceX application. Make sure to select all files to open the binary file.

  \imageStyle{tracex_view.png,width:30%}
  \image html tracex_view.png Save Memory dialog.
  
Additional information on the use of TraceX can be found in the official Eclipse ThreadX documentation : https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/tracex/chapter1.md

## Running the Thread Metric benchmarks

The thread metric benchmarks are included in the ThreadX source distribution. The thread metric test suite consists on 8 individual tests that can be added to an existing ThreadX project. A good starting point is the hello world examples. To add the source for a test start by copying or including the following file in an existing project:

    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/tm_api.h
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/tm_porting_layer.h
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_example/tm_porting_layer_threadx.c
    
In addition add one of the following files, only one test can be compiled at any one time.

    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_tm_basic_test_example.c
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_tm_cooperative_scheduling_test_example.c
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_tm_interrupt_preemption_test_example.c
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_tm_interrupt_test_example.c
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_tm_memory_allocation_test_example.c
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_tm_message_processing_test_example.c
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_tm_preemptive_scheduling_test_example.c
    ${MCU_PLUS_SDK_PATH}/source/kernel/threadx/threadx_src/utility/benchmarks/thread_metric/threadx_tm_synchronization_test_example.c
    
The test can be run by calling tm_main().
    
```
    void tm_main();
    tm_main()
```
    
\endcond
