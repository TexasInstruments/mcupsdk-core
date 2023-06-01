# Smart Placement & Benchmarking {#SMART_PLACEMENT_AND_BENCHMARK}

[TOC]

## What is Smart Placement?
TI Sitara MCU have many different levels of memories with varying latency from CPU and peripherals, and while developing an application, it is required to place sections in different memories. Those sections whose destination memory is already known are placed in the targeted memory, however, other sections' destination memory may not be known. For example, in a control loop application, there would an ISRs/functions which would be there in the control path. Since these functions/ISRs are already known to be critical therefore, they are placed in the fastest memory to have maximum performance.

Smart placement is a name given to a process, using which functions and other linker-placed objects are distributed across different memories, while accounting for their criticality, directly from the source code.

## When is Smart Placement required?

If code is experiencing the following:
1. Lower then expectation for XIP code performance.
2. Higher jitter numbers.
3. High Cache miss rate.
4. High CPU loading of ISR/Task.

Then Smart placement can help in diagnosing the above problem.

On top of these, Smart Placement will always theoretically provide improvements to the code by:
1. Distributing code across different levels of memories to maximize run time performance.

## When Smart Placement won't work?

Depending on the usecase, since, underlying concept of smart placement is to move some function to a faster memory which might help in improving performance with improved ease of use. So in case where a frequently called function or critical function is cached most of the times, then performance improvements using smart placement might not be significant.

## More details on Smart Placement.

Following table list down memories in ascending order of access latency. Memory with minimum latency is the fastest memory.

\cond SOC_AM243X

Memory Name| Memory Start Address | Memory Size
 ----------|----------------------|---------------
 TCM-A     |0x0                   | 32KB (in split) or 64KB (in lockstep)
 TCM-B     |0x41010000            | 32KB (in split) or 64KB (in lockstep)
 MSRAM     |0x70000000            | 2MB
 FLASH     |0x60000000            | External Flash
\endcond

\cond SOC_AM263X

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
\cond SOC_AM263X
\image html smart_placement_am263x_0.png
\endcond

In above diagram, F1 is a critical function which internally calls F2, F3 and F4. However, because F2, F3 and F4 are placed far away in memory, they cannot be called directly but rather trampolines has be inserted in order to call them. This further reduces the performance. Applying smart placement, not only placed F1 function in faster memory, but also its internally called function, which are also critical function in this case. This also removed trampolines.

Since AM243x/AM263X are Cortex-R5 based SOC, by design TCM-A and TCM-B are not cached and also they are single cycle access memories which makes them the fastest. Other memories, MSRAM/OCRAM are cacheable (however, it depends on MPU settings) but is slower than TCMs and then FLASH is external FLASH and is the slowest.

Above devices has many cores. These cores can be configured to run in lockstep or in split mode and depending on the mode, the size of TCM available also varies.

## Steps involved in applying smart placement.

In general, there are 2 steps involved in applying smart placement and those are:
1. Identification of function criticality.
2. Priority Assignment.
3. Function Annotation.
4. Linker update.

### Identification of function criticality

There are many ways in which criticality of a function is identified. For example, in a control loop application, certain functions and ISR are known to be critical. Similarly, in any application, there are some functions which can be identified as critical.

However, there are other methods of determining criticality of a function. One such metric is function call frequency and cacheability i.e. if some function is such that it is frequently called but is not cached well. Such function can be treated a critical function. Other improvement which can be done on top of this to identify and place critical data as well.

#### How to identify?

Manually finding function criticality is one method.

Automatic function criticality is also possible. The output of this step is run time information of how many times a function is executed. Lets say in an application of 100 functions, for each function a number should be associated that helps in determining frequent functions.

This can be determined using
1. Post processing core trace data or
2. Using function profiling via instrumentation.

TI-ARM-CLANG provides function profiling which can be leveraged to attain this information. However, cache statistics is something that is not visible with this method and also it increases the memory footprint and increases function call overhead. Latest TI-CGT-CLANG can be downloaded from https://www.ti.com/tool/download/ARM-CGT-CLANG/ and example instrumentation documentation can be found at https://software-dl.ti.com/codegen/docs/tiarmclang/rel3_1_0_STS/compiler_manual/code_coverage/source_based_code_coverage.html?highlight=instrumentation.

Other method is core trace. Here core tracing is employed to determine function execution if sufficient run time trace has been collected. This does not increase function overhead and also does not changes memory footprint. Core Tracing can be done by making use of Lauterbach Trace32 or Segger tools.

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

## Function Annotation

Compiler provides a feature where any function can be annotated which is essentially controlling its placement in memory.

**C/C++ Source-level function attributes:**

    __attribute__(({local,onchip,offchip}(priority)))

Here `local`, `onchip` and `offchip` are corresponding to TCMx, MSRAM/OCRAM, FLASH.

Example:

    void func0(void) __attribute__((local(1))) { .. } // Place in TCM with priority 1

    void func1(void) __attribute__((local(2))) { .. } // Place in TCM with priority 2

    void func2(void) __attribute__((onchip))   { .. } // Place in MSRAM/OCRAM with implied priority 1

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

        .TI.local   : {} >> R5F_TCMA | R5F_TCMB | MSRAM;

        .TI.onchip  : {} >> MSRAM | FLASH

        .TI.offchip : {} > FLASH;

    }


By default we should use section splitting as shown above between memory regions to get the full effect of function prioritization.


It should be noted that although annotating using assembler directive is convenient, one short-coming is that static functions cannot be annotated or if annotated, no effect of it is on its placement. TO solve this, C/C++ based annotation has to be used in the definition/declaration of that static function.

## Benchmarking Application

### Supported Combinations {#SMART_PLACEMENT_COMBOS}

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang V3.1.0 STS
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/ocmc_benchmarking

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang V3.1.0 STS
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/ocmc_benchmarking

\endcond

### About


- This demo provides a means of measuring the performance of a realistic application where the text of the application is sitting in various memory locations and the data is sitting in On-Chip-Memory RAM (referred to as OCM, OCMC or OCMRAM).
- The application executes 10 different configurations of the same text varying by data vs. instruction cache intensity. Each test calls 16 separate functions 500 total times in random order.
- The most instruction intensive example achieves a instruction cache miss rate (ICM/sec) of ~3-4 million per second when run entirely from OCMRAM. This is a rate that we have similarly seen in real-world customer examples.
- More data intensive tests have more repetitive code, achieving much lower ICM rates.

| Application Output |	Description
---------------------|-------------
| Mem Cpy size    => 100 	|Size of the memcpy in bytes executed by each task
| Exec Time in usec => 2567 	|Amount of time in microseconds
| Iter            => 1 	|Number of times the test was run
| Task calls      => 500 	|Number of randomly ordered calls to the 16 tasks
| Inst Cache miss => 11421 	|Total instruction cache misses
| Inst Cache acc  => 650207 	|Total instruction cache accesses
| num switches    => 1469 	|Number of total context switches
| num instr exec  => 1029260 	|Total number of executed instructions
| ICM/sec         => 4449162 	|Instruction cache misses per second
| INST/sec        => 400958317 	|Instructions executed per second


In this example,`annotations.S` is an assembly file that contains annotations.

Assembly annotations is being used for easier implementation.

`annotations.S` has a macro named `ENABLE_SMART_PLACEMENT_ANNOTATION` which can be used to enable and disable annotations which will effectively enable and disable smart placement.

### Building benchmark application

To build this application, compiler ti-cgt-armllvm >= 3.1.0 is required. Please make sure that `CGT_TI_ARM_CLANG_PATH` variable in imports.mak points to correct path of the compiler. Once this basic path is setup then compiling this program can be achieved using make command.

### Running benchmark application

This particular application is used to show the effect of smart placement. To do this, we first remove `ENABLE_SMART_PLACEMENT_ANNOTATION` macro (which is in `annotations.S` file.) and compile it.

After running the application, console logs prints out in the above table format with different configuration like `Mem Cpy Size` i.e. for different values of `Mem Cpy Size`, `exec time` and `ICM/sec` (among with others) is printed. For example,


    OCMC benchmarking:: Board_init success
    Countdown...
    1/5
    2/5
    3/5
    4/5
    5/5
    Filling up the buffers
    Inst Cache Miss: 2
    Inst Cache Access: 22
    Data Cache Miss: 39


    master_task


    master_task -- start sending

    Mem Cpy Size    => 0
    Start Time in Usec => 244351
    Exec Time in Usec => 11867
    Iter            => 1
    Task Calls      => 500
    Inst Cache Miss => 87023
    Inst Cache Acc  => 856803
    Num Instr Exec  => 1893125
    ICM/sec         => 9
    ICM/sec         => 9
    INST/sec        => 197
    INST/sec        => 197

    Mem Cpy Size    => 100
    Start Time in Usec => 269072
    Exec Time in Usec => 12428
    Iter            => 1
    Task Calls      => 500
    Inst Cache Miss => 85720
    Inst Cache Acc  => 995992
    Num Instr Exec  => 2087348
    ICM/sec         => 8
    ICM/sec         => 8
    INST/sec        => 207
    INST/sec        => 207

    Mem Cpy Size    => 200
    Start Time in Usec => 298477
    Exec Time in Usec => 13130
    Iter            => 1
    Task Calls      => 500
    Inst Cache Miss => 86529
    Inst Cache Acc  => 1141634
    Num Instr Exec  => 2287056
    ICM/sec         => 8
    ICM/sec         => 8
    INST/sec        => 215
    INST/sec        => 215

    Mem Cpy Size    => 400
    Start Time in Usec => 325456
    Exec Time in Usec => 14606
    Iter            => 1
    Task Calls      => 500
    Inst Cache Miss => 88361
    Inst Cache Acc  => 1446200
    Num Instr Exec  => 2688209
    ICM/sec         => 7
    ICM/sec         => 7
    INST/sec        => 227
    INST/sec        => 227

    Mem Cpy Size    => 1000
    Start Time in Usec => 354810
    Exec Time in Usec => 18367
    Iter            => 1
    Task Calls      => 500
    Inst Cache Miss => 84032
    Inst Cache Acc  => 2342630
    Num Instr Exec  => 3879466
    ICM/sec         => 5
    ICM/sec         => 5
    INST/sec        => 260
    INST/sec        => 260

    Mem Cpy Size    => 1500
    Start Time in Usec => 388418
    Exec Time in Usec => 23678
    Iter            => 1
    Task Calls      => 500
    Inst Cache Miss => 86357
    Inst Cache Acc  => 3106798
    Num Instr Exec  => 4892362
    ICM/sec         => 4
    ICM/sec         => 4
    INST/sec        => 255
    INST/sec        => 255

    Mem Cpy Size    => 2000
    Start Time in Usec => 426769
    Exec Time in Usec => 28432
    Iter            => 1
    Task Calls      => 500
    Inst Cache Miss => 86051
    Inst Cache Acc  => 3855309
    Num Instr Exec  => 5887390
    ICM/sec         => 3
    ICM/sec         => 3
    INST/sec        => 255
    INST/sec        => 255

    Mem Cpy Size    => 2250
    Start Time in Usec => 471196
    Exec Time in Usec => 31236
    Iter            => 1
    Task Calls      => 500
    Inst Cache Miss => 88306
    Inst Cache Acc  => 4239124
    Num Instr Exec  => 6397039
    ICM/sec         => 3
    ICM/sec         => 3
    INST/sec        => 252
    INST/sec        => 252



So from the above, `mem Cpy Size` is changing, ICM is almost same but `Exec Time in Usec` is increasing which is thereby changing `ICM/sec`. Here to find the base line, `Exec Time in Usec` should be selected for the case where `ICM/sec` is 3 (or ~3Millions Instruction Cache Miss/Sec) and from the above data, it can be determined that when `mem Cpy Size` is 2000, `ICM/sec` was 3 and for this, it took 28432 us to execute the application. Therefore, 28432 us is our baseline number.


Now after this, `ENABLE_SMART_PLACEMENT_ANNOTATION` macro is defined again and example is compiled back.

running that would give similar logs and execution time against the `Mem Cpy Size` of 2000 is to be noted and has been measured out to be 25758 us. Following table shows the same:

Mem Cpy Size | INST/sec          | Exec Time in Usec without Smart Placement | Exec Time in Usec with Smart Placement
-------------|-------------------|-------------------------------------------|---------------------------------------
 0000         | 7333192           | 11867                                     | 9424
 0100         | 7002172           | 12428                                     | 10057
 0200         | 6627799           | 13130                                     | 10653
 0400         | 5958031           | 14606                                     | 12105
 1000         | 4509122           | 18367                                     | 15924
 1500         | 3647140           | 23678                                     | 21222
 2000         | 3026765           | 28432                                     | 25758
 2250         | 2827058           | 31236                                     | 28583


Therefore, with smart placement code was executed in 25758us and without smart placement same code executed in 28432us which corresponds to performance gain of 10% without changing of any code.



