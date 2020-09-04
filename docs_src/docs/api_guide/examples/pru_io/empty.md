# Empty Project {#EXAMPLES_PRU_EMPTY}

[TOC]

## Introduction

This example acts as a getting started point for PRU firmware development in assembly. It demoes how to access local memory locations of PRU.

# Supported Combinations

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 ICSSG          | ICSSG0 - PRU0
 Toolchain      | pru-cgt
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/pru_io/empty/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project from the above mentioned Example folder path. After this you should see two linked files: `main.asm` and `linker.cmd`. The `main.asm` contains sample code for accessing Local PRU memories available: DMEM0 (64kB), DMEM1 (64kB) and Shared Ram (64kB).

- Build the PRU project.

    \note
    Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Writing PRU code

* You can modify this example to write your own firmware. `<sdk-install-dir/examples/pru_io/adc/>` contains some PRU projects which can act as reference. PRU sources are usually separated out in a `firmware/` subfolder in all these example project folders.
* You can change the core in project properties to develop firmware for other PRU cores.