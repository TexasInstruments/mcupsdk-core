# Memory Benchmark with Smart Placement {#BENCHMARK_SMART_PLACEMENT}

[TOC]

# Supported Combinations {#BENCHMARK_SMART_PLACEMENT_COMBOS}

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/ocmc_benchmarking

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/ocmc_benchmarking

\endcond

# Introduction


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

Please read more on smart placement at \ref SMART_PLACEMENT, before moving further ahead.

`annotations.S` has a macro named `ENABLE_SMART_PLACEMENT_ANNOTATION` which can be used to enable and disable annotations which will effectively enable and disable smart placement.

Here it will be shown how code performance (or CPI) is improved with smart placement.

# Building benchmark application

To build this application, compiler ti-cgt-armllvm >= 3.2.0 LTS or later is required. Application can be compiled using make command.

# Running benchmark application

This particular application is used to show the effect of smart placement. To do this, we first remove `ENABLE_SMART_PLACEMENT_ANNOTATION` macro (which is in `annotations.S` file.) and compile it. To know how `annotations.S` has been generated then please refer to \ref SW_INSTRUMENT_SMART_PLACEMENT

Flash this application using uart_uniflash. For this please go through \ref TOOLS_FLASH_UART_UNIFLASH

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



