# DDR MEMTESTER {#EXAMPLES_DRIVERS_DDR_MEMTESTER}

[TOC]

\note This example demostrates performing the DDR Memory Testing in R5.

# Introduction

This example performs read and write tests on the DDR memory space with multiple patterns. If either write is not possible, or there is a mismatch of value after reading from the memory space, the test fails. Ensure DDR Init is performed such that the DDR memory space is initialized and can be accessed in the Application.

# Supported Combinations {#EXAMPLES_DRIVERS_DDR_MEMTESTER_COMBOS}

\cond SOC_AM243X

## AM243X-EVM
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/ddr/ddr_memtester/

\endcond

\cond SOC_AM64X

## AM64X
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/ddr/ddr_memtester/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_DDR_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code 
START DDR Memtester
Testing from start address: 0x80000000
Testing all patterns
Testing for pattern: 0x0
Pattern: 0x0 passed
Testing for pattern: 0xffffffffffffffff
Pattern: 0xffffffffffffffff passed
Testing for pattern: 0x5555555555555555
Pattern: 0x5555555555555555 passed
Testing for pattern: 0xaaaaaaaaaaaaaaaa
Pattern: 0xaaaaaaaaaaaaaaaa passed
Testing for pattern: 0x1111111111111111
Pattern: 0x1111111111111111 passed
Testing for pattern: 0x2222222222222222
Pattern: 0x2222222222222222 passed
Testing for pattern: 0x4444444444444444
Pattern: 0x4444444444444444 passed
Testing for pattern: 0x8888888888888888
Pattern: 0x8888888888888888 passed
Testing for pattern: 0x3333333333333333
Pattern: 0x3333333333333333 passed
Testing for pattern: 0x6666666666666666
Pattern: 0x6666666666666666 passed
Testing for pattern: 0x9999999999999999
Pattern: 0x9999999999999999 passed
Testing for pattern: 0xcccccccccccccccc
Pattern: 0xcccccccccccccccc passed
Testing for pattern: 0x7777777777777777
Pattern: 0x7777777777777777 passed
Testing for pattern: 0xbbbbbbbbbbbbbbbb
Pattern: 0xbbbbbbbbbbbbbbbb passed
Testing for pattern: 0xdddddddddddddddd
Pattern: 0xdddddddddddddddd passed
Testing for pattern: 0xeeeeeeeeeeeeeeee
Pattern: 0xeeeeeeeeeeeeeeee passed
Testing for pattern: 0x7a6c7258554e494c
Pattern: 0x7a6c7258554e494c passed
Testing from mid address: 0xc0000000
Testing all patterns
Testing for pattern: 0x0
Pattern: 0x0 passed
Testing for pattern: 0xffffffffffffffff
Pattern: 0xffffffffffffffff passed
Testing for pattern: 0x5555555555555555
Pattern: 0x5555555555555555 passed
Testing for pattern: 0xaaaaaaaaaaaaaaaa
Pattern: 0xaaaaaaaaaaaaaaaa passed
Testing for pattern: 0x1111111111111111
Pattern: 0x1111111111111111 passed
Testing for pattern: 0x2222222222222222
Pattern: 0x2222222222222222 passed
Testing for pattern: 0x4444444444444444
Pattern: 0x4444444444444444 passed
Testing for pattern: 0x8888888888888888
Pattern: 0x8888888888888888 passed
Testing for pattern: 0x3333333333333333
Pattern: 0x3333333333333333 passed
Testing for pattern: 0x6666666666666666
Pattern: 0x6666666666666666 passed
Testing for pattern: 0x9999999999999999
Pattern: 0x9999999999999999 passed
Testing for pattern: 0xcccccccccccccccc
Pattern: 0xcccccccccccccccc passed
Testing for pattern: 0x7777777777777777
Pattern: 0x7777777777777777 passed
Testing for pattern: 0xbbbbbbbbbbbbbbbb
Pattern: 0xbbbbbbbbbbbbbbbb passed
Testing for pattern: 0xdddddddddddddddd
Pattern: 0xdddddddddddddddd passed
Testing for pattern: 0xeeeeeeeeeeeeeeee
Pattern: 0xeeeeeeeeeeeeeeee passed
Testing for pattern: 0x7a6c7258554e494c
Pattern: 0x7a6c7258554e494c passed
Testing from end address: 0xffdfffff
Testing all patterns
Testing for pattern: 0x0
Pattern: 0x0 passed
Testing for pattern: 0xffffffffffffffff
Pattern: 0xffffffffffffffff passed
Testing for pattern: 0x5555555555555555
Pattern: 0x5555555555555555 passed
Testing for pattern: 0xaaaaaaaaaaaaaaaa
Pattern: 0xaaaaaaaaaaaaaaaa passed
Testing for pattern: 0x1111111111111111
Pattern: 0x1111111111111111 passed
Testing for pattern: 0x2222222222222222
Pattern: 0x2222222222222222 passed
Testing for pattern: 0x4444444444444444
Pattern: 0x4444444444444444 passed
Testing for pattern: 0x8888888888888888
Pattern: 0x8888888888888888 passed
Testing for pattern: 0x3333333333333333
Pattern: 0x3333333333333333 passed
Testing for pattern: 0x6666666666666666
Pattern: 0x6666666666666666 passed
Testing for pattern: 0x9999999999999999
Pattern: 0x9999999999999999 passed
Testing for pattern: 0xcccccccccccccccc
Pattern: 0xcccccccccccccccc passed
Testing for pattern: 0x7777777777777777
Pattern: 0x7777777777777777 passed
Testing for pattern: 0xbbbbbbbbbbbbbbbb
Pattern: 0xbbbbbbbbbbbbbbbb passed
Testing for pattern: 0xdddddddddddddddd
Pattern: 0xdddddddddddddddd passed
Testing for pattern: 0xeeeeeeeeeeeeeeee
Pattern: 0xeeeeeeeeeeeeeeee passed
Testing for pattern: 0x7a6c7258554e494c
Pattern: 0x7a6c7258554e494c passed
DDR Memtester Completed
\endcode
