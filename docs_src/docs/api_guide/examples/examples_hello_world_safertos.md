#  Hello World Safertos Project {#EXAMPLES_HELLO_WORLD_SAFERTOS}

[TOC]

# Introduction

This example just does driver and board initialization and prints the string, Hello World! on CCS console.

# Supported Combinations {#EXAMPLES_HELLO_WORLD_SAFERTOS_COMBOS}

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 safertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/hello_world_safertos

\endcond

# Steps to build the Example

You can use the makefiles to build please refer \ref MAKEFILE_BUILD_PAGE

- To build the hello world safertos example, do below,
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s -C examples/hello_world_safertos/awr294x-evm/c66ss0_safertos/ti-c6000 all PROFILE=release
    \endcode

- To clean  the safertos library, do below
    \code
    cd ${SDK_INSTALL_PATH}
    gmake -s -C examples/hello_world_safertos/awr294x-evm/c66ss0_safertos/ti-c6000 clean PROFILE=release
    \endcode

\note Make sure the safertos libs are built before trying to build this example \ref KERNEL_SAFERTOS_LIB_BUILD

# Steps to Run the Example

- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output on CCS console when the application is run,

\code
Hello World!
\endcode
