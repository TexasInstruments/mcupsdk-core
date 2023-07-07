# Smart Placement & Benchmarking {#SMART_PLACEMENT_AND_BENCHMARK}

[TOC]

## What is Smart Placement?
TI Sitara MCU have many different levels of memories with varying latency from CPU and peripherals, and while developing an application, it is required to place sections in different memories. Those sections whose destination memory is already known are placed in the targeted memory, however, other sections' destination memory may not be known.

Smart placement is a name given to a process, using which functions and other linker-placed objects  are distributed across different memories.

Smart Placement distribute functions across memories based on function's call frequency. **A more frequently called function will be placed in faster memory**.

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

Since above devices are Cortex-R5 based SOC, by design TCM-A and TCM-B are not cached and also they are single cycle access memories which makes them the fastest. Other memories, MSRAM/OCRAM are cacheable (however, it depends on MPU settings) but is slower than TCMs and then FLASH is external FLASH and is the slowest.

Above devices has many cores. These cores can be configured to run in lockstep or in slip mode and depending on the mode, the size of TCM available also varies.


## Steps involved in applying smart placement.

In general there are 2 steps involved in applying smart placement and those are:
1. Identification of function frequency.
2. Function Annotation & priority Identification.

### Identification of function frequency

The output of this step is run time information of how many times a function is executed. Lets say in an application of 100 functions, for each function a number should be associated that helps in determining frequent functions.

This can be determined post pocessing core trace data or using function profiling via instrumentation.

### Function Annotation & priority Identification

Compiler provides a feature where any function can be annotated which is essentially controlling linker in placement of that function.

**C/C++ Source-level function attributes:**

    __attribute__(({local,onchip,offchip}(priority))) (Corresponding to TCMx, MSRAM/OCRAM, FLASH)



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

This would allow users to avoid having to compile other source code.

Here, priority is very important aspect as a function with lower priority number will be placed by the linker in the corresponding memory first and then other function with higher priority number will be placed. This priority can be calculated roughly using `(max_function_frequency_number - current_frequency_number)` where `max_function_frequency_number` is the frequency number of most frequently called function and `current_frequency_number` is the call frequency of current function that is being annotated.

**Linker aggregation:**


As previously described, the linker will aggregate all function input sections into designated output sections while sorting the placement of input sections based on the given priority. Following code can be added in the linker file to inorder to do that.



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



