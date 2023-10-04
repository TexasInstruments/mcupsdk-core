# Smart Placement {#SMART_PLACEMENT}

[TOC]

## What is Smart Placement?
TI Sitara MCU have many different levels of memories with varying latency from CPU and peripherals, and while developing an application, it is required to place sections in different memories. Those sections whose destination memory is already known are placed in the targeted memory, however, other sections' destination memory may not be known. For example, in a control loop application, there would an ISRs/functions which would be there in the control path. Since these functions/ISRs are already known to be critical therefore, they are placed in the fastest memory to have maximum performance.

Smart placement is a name given to a process, using which functions and other linker-placed objects are distributed across different memories, while accounting for their criticality, directly from the source code.

## When is Smart Placement required?

If code is experiencing the following:
1. Lower then expectation for XIP code performance (only valid for devices that supports XiP)
2. Higher jitter numbers.
3. High Cache miss rate.
4. High CPU loading of ISR/Task.

Then Smart placement can help in diagnosing the above problem.

However this is very application specific, it is expected to yield a better performance if the code is well placed between memories. Smart Placement handles this problem with much ease of use.

## What not to expect from Smart Placement tool?

Depending on the usecase, since, underlying concept of smart placement is to move some function to a faster memory which might help in improving performance with improved ease of use. So in case where a frequently called function or critical function is cached most of the times, then performance improvements using smart placement might not be significant.

## More details on Smart Placement.

Following table list down memories in ascending order of access latency (not an exhaustive list of memories). Memory with minimum latency is the fastest memory.

\cond SOC_AM243X

Memory Name| Memory Start Address | Memory Size
 ----------|----------------------|---------------
 TCM-A     |0x0                   | 32KB (in split) or 64KB (in lockstep)
 TCM-B     |0x41010000            | 32KB (in split) or 64KB (in lockstep)
 MSRAM     |0x70000000            | 2MB
 FLASH     |0x60000000            | External Flash
\endcond

\cond SOC_AM263X || SOC_AM263PX

 Memory Name|Memory Start Address | Memory Size
 -----------|---------------------|---------------
 TCM-A      |0x0                  | 32KB (in split) or 64KB (in lockstep)
 TCM-B      |0x8000               | 32KB (in split) or 64KB (in lockstep)
 OCRAM      |0x70000000           | 2MB
 FLASH      |0x60000000           | External Flash
\endcond

From the above table, this device has various memories which can store both code and data. Each memory differs from each other in terms of size and access latency therefore the placement of code and data will determine the performance of code.

This performance penalty is high incase code or data has to be placed in external memory (or FLASH). In this case, it is beneficial to not place data or code in the external flash that is not critical. Now the criticality of the code can be determined by various factors such as number of times a particular code or data is being accessed or based on the design decision.

Suppose that for a given program, this criticality has been quantified for each code and data section, i.e. if a program has 2 functions viz. `foo` and `bar` then this criticality is quantified for `foo` to be 1 and for `bar` to be 2. What this tells is, during the placement of these functions, `foo` should be placed in a faster memory than `bar`. Therefore, this criticality has to be translated to placement priority. More critical a function is higher the priority it should have during the placement of that function/data section.

Normally any program has a lot of different functions and data members it accesses and using linker all of those functions and data sections are then distributed across different memories, however, to maximize the performance, it is required to smartly place critical code and data sections in faster memory on priority.

Smart Placement distribute functions across memories based on function's critically. Following diagram explains more.

\cond SOC_AM243X
\image html smart_placement_am243x_0.png
\endcond
\cond SOC_AM263X || SOC_AM263PX
\image html smart_placement_am263x_0.png
\endcond

In above diagram, F1 is a critical function which internally calls F2, F3 and F4. However, because F2, F3 and F4 are placed far away in memory, they cannot be called directly but rather trampolines has be inserted in order to call them. This further reduces the performance. Applying smart placement, not only placed F1 function in faster memory, but also its internally called function, which are also critical function in this case. This also removed trampolines.

Since AM243x/AM263X are Cortex-R5 based SOC, by design TCM-A and TCM-B are not cached and also they are single cycle access memories which makes them the fastest. Other memories, MSRAM/OCRAM are cacheable (however, it depends on MPU settings) but is slower than TCMs and then FLASH is external FLASH and is the slowest.

## Steps involved in applying smart placement.

In general, there are following steps involved in applying smart placement:
1. Identification of function criticality.
2. Priority Assignment.
3. Function Annotation.
4. Linker update.

### Identification of function criticality

There are many ways in which criticality of a function is identified.

#### 1. Manual identifiaction of Critical Functions.

Manually finding function criticality is one method.

For example, in a control loop application, certain functions and ISR are known to be critical. Similarly, in any application, there are some functions which can be identified as critical.

#### 2. Automatically Identifying Critical Function.

Other methods of determining criticality of a function is based on metric of function call frequency and cacheability i.e. if some function is such that it is frequently called. Such function can be treated a critical function.

The output of this step is run time information of how many times a function is executed. Lets say in an application of 100 functions, for each function a number should be associated that helps in determining frequent functions.

This can be determined using
1. <b>Post processing core trace data: </b>
 Here core tracing is employed to determine function execution if sufficient run time trace has been collected. This does not increase function overhead and also does not changes memory footprint. Core Tracing can be done by making use of some Aadvanced tools like Lauterbach Trace32 or Segger.tools. This is required for advanced uasecase.

2. <b>Using function profiling via instrumentation: </b>
TI-ARM-CLANG provides function profiling which can be leveraged to attain this information. However, this method increases the memory footprint and increases function call overhead. Latest TI-CGT-CLANG can be downloaded from https://www.ti.com/tool/download/ARM-CGT-CLANG/ and example instrumentation documentation can be found at https://software-dl.ti.com/codegen/docs/tiarmclang/rel3_2_0_LTS/compiler_manual/code_coverage/source_based_code_coverage.html?highlight=instrumentation.


Manual identification on top of automatic critical function identification is also possible.

## Priority Assignment

Once critical functions are identified, now it is required to put a priority function. For Manually identified functions, it is known that they are critical functions. So, we can start with assigning highest priority.

\note
Lower number corresponds to higer priority. `Priority` and `Priority Number` are both different. A highest `Priority` function would have lowest `Priority Number`.

For the functions which are identified via automated means, will have a function frequency count associated with them. This priority can be calculated roughly using `(max_function_frequency_number - current_frequency_number) + offset ` where
1. `max_function_frequency_number` is the frequency number of most frequently called function.
2. `current_frequency_number` is the call frequency of a function.
3. `offset` is a number that is one more than the max `prioirty number` that is assigned to manually identified functions, if any.

\note Here, function with lower `priority number` will be placed by the linker in the faster memory first.

### Example

For an application, suppose the following functions has been maunually identified as critical functions:
1. fm_1
2. fm_2
3. fm_3
4. fm_4
5. fm_5

Following list of function, which has been identified using any automatic method, in the descending order execution frequency, i.e. fa_1 has been called most number of times and similarly fa_5 has been called least number of times:

1. fa_1
2. fa_2
3. fa_3
4. fa_4
5. fa_5

Now, for all the above critical functions, which are identified either automatically or manually, priority number can be assigned like following:

Function Name | Priority Number
 -------------|----------------
 fm_1         | 0
 fm_2         | 0
 fm_3         | 0
 fm_4         | 0
 fm_5         | 0
 fa_1         | 1
 fa_2         | 2
 fa_3         | 3
 fa_4         | 4
 fa_5         | 5

Notice that all the critical functions that has been identified manually has been given same priority number of 0. `Priority number` 0 implies highest `priority`.

## Function Annotation

Compiler provides a feature where any function can be annotated which is essentially controlling its placement in memory.

**C/C++ Source-level function attributes:**

    __attribute__(({local,onchip,offchip}(priority)))

Here `local`, `onchip` and `offchip` are corresponding to TCMx, MSRAM/OCRAM, FLASH.

Example:

    void __attribute__((local(1))) func0(void) { .. } // Place in TCM with priority 1

    void __attribute__((local(2))) func1(void) { .. } // Place in TCM with priority 2

    void __attribute__((onchip)) func2(void) { .. } // Place in MSRAM/OCRAM with implied priority 1

The attributes can be added to a function definition or a function declaration (if that function is called/referenced in the same compilation unit).

**Assembly metainfo directives:**

Functions can also be annotated by adding an assembly metainfo directive in an assembly file that is compiled and linked with the project using the following format:

      .global <global function symbol>

      .sym_meta_info <global function symbol>, “of_placement”, {“local”,”onchip”,”offchip”}, <priority>

e.g.

    .global strcmp

    .sym_meta_info strcmp, “of_placement”, “local”, 1


Here in C, strcmp is the symbol name given to function strcmp.

This would allow users to avoid having to compile 3rd party source code.

## Linker update

Linker will aggregate all function input sections into designated output sections while sorting the placement of input sections based on the given priority. Following code can be added in the linker file to inorder to do that.



    SECTIONS

    {

        .TI.local   : {} >> R5F_TCMA | R5F_TCMB | MSRAM

        .TI.onchip  : {} >> MSRAM | FLASH

        .TI.offchip : {} > FLASH

    }


By default we should use section splitting as shown above between memory regions to get the full effect of function prioritization.

The above linker lines basically channeling all the functions that are annotated to be in `local` memory into TCM memory and if total size of the functions that are marked `local` is more than the size of R5F_TCMA then all the functions that could be placed in R5F_TCMA will be placed in R5F_TCMA and rest of functions will be moved in R5F_TCMB and even if it still fills R5F_TCMB then remaining function will be moved to MSRAM.

Similar treatment is for all the functions that are marked `onchip`, however, they should never be placed in any TCM otherwise it will be logically wrong.

Also all functions which are marked `offchip`, should be placed in external FLASH.

It should be noted that although annotating using assembler directive is convenient, one short-coming is that static functions cannot be annotated or if annotated, no effect of it is on its placement. TO solve this, C/C++ based annotation has to be used in the definition/declaration of that static function.

## Basic Smart Placement Example

\ref EXAMPLES_BASIC_SMART_PLACEMENT

This example provides a basic overview of applying smart placement and compares run time of functions with smart placement and without smart placement.

The Aim of this example is to:
1. Showcase the process of smart placement in simple terms.
2. How Smart Placement improves code performance?
3. How Cache Miss ratio is improved?

## Benchmarking Application

\ref BENCHMARK_SMART_PLACEMENT

This demo provides a means of measuring the performance of a realistic application where the text of the application is sitting in various memory locations and the data is sitting in On-Chip-Memory RAM (referred to as OCM, OCMC or OCMRAM).
