# FreeRTOS MPU Task Switch Example {#EXAMPLES_KERNEL_FREERTOS_TASK_SWITCH_MPU}

[TOC]

# Introduction

This example shows usage of direct FreeRTOS APIs, i.e not via the DPL APIs.
It shows usage of user mode task create API, task notification APIs, semaphore APIs.
It also shows how to signal to FreeRTOS task from ISRs.

The example does below:-
- Creates two user mode tasks, ping and pong with task specific MPU regions
- Creates three semaphores
- Ping and pong tasks signal each other using semaphores and task notifications
- Creates 2 HW ISR and ping/pong task is signaled from the ISR
- Ping/Pong tasks increment a global ping/pong counter as well as a shared counter in between task switch

Following tables summaries relevant memory map used by the example along with the static MPU regions 
configurations as well as the task specific MPU regions configurations,

| Memory Section  | Origin     | Size    | Static MPU Region Mapping| Dynamic MPU Region Mapping      |
|-----------------|------------|---------|--------------------------|---------------------------------|
| R5F_VECS        | 0x00000000 | 64 B    | [1]ATCM                  | -                               |
| R5F_TCMA        | 0x00000040 | 31.9 KB | [1]ATCM                  | -                               |
| R5F_TCMB        | 0x00080000 | 32 KB   | [2]BTCM                  | -                               |
| FLASH           | 0x60100000 | 512 KB  | [6]FLASH                 | -                               |
| SBL             | 0x70000000 | 256 KB  | [3]OCRAM_BACKGROUND      | -                               |
| OCRAM           | 0x70040000 | 128 KB  | [4]OCRAM_UNPRIVILEGED    | -                               |
| PRIVILEGED_TEXT | 0x70060000 | 64 KB   | [5]OCRAM_PRIVILEGED_TEXT | -                               |
| PRIVILEGED_DATA | 0x70070000 | 56 KB   | [3]OCRAM_BACKGROUND      |  [8]Ping/Pong Task Stack Region |
| PING_TEXT       | 0x7007E000 | 2 KB    | [3]OCRAM_BACKGROUND      |  [9]Ping Task Region 0          |
| PING_DATA       | 0x7007E800 | 2 KB    | [3]OCRAM_BACKGROUND      | [10]Ping Task Region 1          |
| PONG_TEXT       | 0x7007F000 | 2 KB    | [3]OCRAM_BACKGROUND      |  [9]Pong Task Region 0          |
| PONG_DATA       | 0x7007F800 | 2 KB    | [3]OCRAM_BACKGROUND      | [10]Pong Task Region 1          |

| [idx]Static MPU Regions       | Base Address | Size   | Execute | Access Permissions        |
|-------------------------------|--------------|--------|---------|---------------------------|
| [0]BACKGROUND_REGION          | 0x00000000   | 2 GB   | No      | Supervisor RW, User RW    |
| [1]ATCM                       | 0x00000000   | 32 KB  | Yes     | Supervisor RW, User BLOCK |
| [2]BTCM                       | 0x00080000   | 32 KB  | Yes     | Supervisor RW, User BLOCK |
| [3]OCRAM_BACKGROUND           | 0x70000000   | 2 MB   | No      | Supervisor RW, User BLOCK |
| [4]OCRAM_UNPRIVILEGED         | 0x70040000   | 128 KB | Yes     | Supervisor RW, User RW    |
| [5]OCRAM_PRIVILEGED_TEXT      | 0x70060000   | 64 KB  | Yes     | Supervisor RD, User BLOCK |
| [6]FLASH                      | 0x60100000   | 512 KB | Yes     | Supervisor RD, User BLOCK |

| [idx]Ping/Pong Task MPU Regions | Base Address          | Size   | Execute | Access Permissions     |
|---------------------------------|-----------------------|--------|---------|------------------------|
|  [8]Stack Region                | -                     | -      | No      | Supervisor RW, User RW |
|  [9]PING_TEXT/PONG_TEXT         | 0x7007E000/0x7007F000 | *2 KB  | Yes     | Supervisor RD, User RD |
| [10]PING_DATA/PONG_DATA         | 0x7007E800/0x7007F800 | *2 KB  | No      | Supervisor RW, User RW |

Note:
- [0]BACKGROUND_REGION with User RW access is required for peripheral access
- [3]OCRAM_BACKGROUND maps entire OCRAM memory as Supervisor R/W only, restricting access by User
  - Other overlay regions to give only the required access to user 
- [8]Stack Region base address and size is dynamic
  - MPU entry is added using the the stack buffer address and size
- [9]PING_TEXT/PONG_TEXT and [10]PING_DATA/PONG_DATA size is based on actual usage
  - MPU entry is added using the linker script variables

The example source files are split to dedicated files to achieve following access restrictions,
- All text and data in `task_switch_mpu.c` is by default placed in privileged functions and privileged data 
  sections respectively with following in linker script:-
  \code
    .text.task_switch_mpu.privileged : {
        task_switch_mpu.obj(.text)
    } palign(8)
    } > PRIVILEGED_TEXT       
     
    .data.task_switch_mpu.privileged : {
        task_switch_mpu.obj(.data, .bss)
    } palign(8)
    } > PRIVILEGED_DATA 
  \endcode
  Hence user mode tasks won't have access to any of the text/data in `task_switch_mpu.c`.
  Ping/Pong task stacks are defined in `task_switch_mpu.c` and will be placed in privileged data section. 
  But as part of user task creation a dedicated MPU region will be configured for the task stack.  

- All text and data in `task_switch_mpu_ping.c` is by default placed in corresponding sections reserved for 
  the ping task with the following in linker script:-
  \code
    GROUP  :   {
    .text.task_switch_mpu.ping : {
        task_switch_mpu_ping.obj(.text)
    } palign(8)
    } > PING_TEXT  
    RUN_START(__TEXT_PING_START)
    RUN_END(__TEXT_PING_END)
 
    GROUP  :   {
    .data.task_switch_mpu.ping : {
        task_switch_mpu_ping.obj(.data, .bss)
    } palign(8)
    } > PING_DATA  
    RUN_START(__DATA_PING_START)
    RUN_END(__DATA_PING_END)
  \endcode
  The linker variables `__TEXT_PING_START`, `__TEXT_PING_END`, `__DATA_PING_START`, `__DATA_PING_END` will be
  used to create the MPU regions for the ping task. Hence other user mode tasks won't have access to any of 
  the text/data in `task_switch_mpu_ping.c`.  

- All text and data in `task_switch_mpu_pong.c` is by default placed in corresponding sections reserved for 
  the pong task with the following in linker script:-
  \code
    GROUP  :   {
    .text.task_switch_mpu.pong : {
        task_switch_mpu_pong.obj(.text)
    } palign(8)
    } > PONG_TEXT  
    RUN_START(__TEXT_PONG_START)
    RUN_END(__TEXT_PONG_END)
 
    GROUP  :   {
    .data.task_switch_mpu.pong : {
        task_switch_mpu_pong.obj(.data, .bss)
    } palign(8)
    } > PONG_DATA  
    RUN_START(__DATA_PONG_START)
    RUN_END(__DATA_PONG_END)
  \endcode
  The linker variables `__TEXT_PONG_START`, `__TEXT_PONG_END`, `__DATA_PONG_START`, `__DATA_PONG_END` will be
  used to create the MPU regions for the pong task. Hence other user mode tasks won't have access to any of 
  the text/data in `task_switch_mpu_pong.c`.  

- All text and data in `task_switch_mpu_common.c` is by default placed in generic text and data segments 
  in OCRAM which falls under OCRAM_UNPRIVILEGED MPU region configuration. These will be accessible by user 
  mode tasks unless explicitly specified by an attribute to place in a different dedicated section with 
  reduced access permissions.
  - Same is the case with SDK driver APIs and data.

- In summary,
  - Any text/data which is not required by user mode tasks is placed in `task_switch_mpu.c`
  - Any text/data which is required only by ping task is placed in `task_switch_mpu_ping.c`
  - Any text/data which is required only by pong task is placed in `task_switch_mpu_pong.c`
  - Any text/data which is required by both ping & pong tasks is placed in `task_switch_mpu_common.c`

# Supported Combinations

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos_mpu
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/task_switch_mpu

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref KERNEL_FREERTOS_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
FreeRTOS MPU Task Switch example ... start !!!

[FreeRTOS] ping task ... start !!!

Performing task switch using semaphores

pong count = 100000
ping count = 100000
common count = 200000

execution time for task switches = 1706467 us
number of task switches = 200000 
time per task switch (semaphore give/take) = 8532 ns

Performing task switch using direct-to-task notification

pong count = 100000
ping count = 100000
common count = 200000

execution time for task switches = 1509621 us
number of task switches = 200000 
time per task switch (direct-to-task notification give/take) = 7548 ns

Performing task switch using interrupts

pong count = 100000
ping count = 100000
common count = 200000

execution time for task - ISR - task - task switches = 1652114 us
number of ISRs = 200000 
time per task - ISR - task switch (semaphore give/take) = 8260 ns

[FreeRTOS] ping task ... done !!!

All tests have passed!!
\endcode