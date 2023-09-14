# Driver Porting Layer (DPL) Low Latency Interrupt {#EXAMPLES_KERNEL_DPL_LOW_LATENCY_INTERRUPT}

[TOC]

# Introduction

This examples demonstrates the performance and features of optimized R5F IRQ handler in DPL of MCU_PLUS_SDK

- Part 1 demonstrates Interrupt Latency
- Part 2 demsontrates Nesting/Re-entrancy support and floating point (FPU) save/restore support

Also refer \ref PERFORMANCE_OPTIMIZATIONS_GUIDE


\imageStyle{am263_dpl_low_latency_interrupt_a.png,width:80%}
\image html am263_dpl_low_latency_interrupt_a.png "Block diagram and expected output"


\imageStyle{am263_dpl_low_latency_interrupt_b.png,width:80%}
\image html am263_dpl_low_latency_interrupt_b.png "Flow chart"


This example uses following assembly macros for IRQ handling. These are defined in the dpl_low_latency_interrupt.c file

- ISR_CALL_LEVEL_NONFLOAT_NONREENTRANT and ISR_CALL_PULSE_NONFLOAT_NONREENTRANT
 - Use this if nesting of another interrupt inside this IRQ handler is not required and R5F Floating Point Unit is not used inside user ISR code
- ISR_CALL_LEVEL_FLOAT_NONREENTRANT and ISR_CALL_PULSE_FLOAT_NONREENTRANT
 - Use this if nesting of another interrupt inside this IRQ handler is not required and R5F Floating Point Unit is used inside user ISR code
- ISR_CALL_LEVEL_NONFLOAT_REENTRANT and ISR_CALL_PULSE_NONFLOAT_REENTRANT
 - Use this if nesting of another interrupt inside this IRQ handler is required and R5F Floating Point Unit is not used inside user ISR code
- ISR_CALL_LEVEL_FLOAT_REENTRANT and ISR_CALL_PULSE_FLOAT_REENTRANT
 - Use this if nesting of another interrupt inside this IRQ handler is required and R5F Floating Point Unit is used inside user ISR code

Refer the page \ref CHAPTER_OPTIMIZATION_SECTION_1 for steps to register custom interrupt using above macros and expected latency (cycles) improvements.


Benchmarks:

- Refer table below for measurement of execution time of Interrupt handler (NORTOS, IRQ handlers)
- These measurements include IRQ entry handling latency + EPWM interrupt clear + IRQ exit handling latency + GPIO toggling overheads in background task



 Interrupt handler                                            | Measured execution time
 -------------------------------------------------------------|--------------------------
 1. Non Re-entrant, Without FPU context save                  | 310ns
 2. Non Re-entrant, With FPU context save                     | 426ns
 3. Re-entrant, Without FPU context save                      | 376ns
 4. Re-entrant, With FPU context save                         | 496ns


# Supported Combinations

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/dpl_low_latency_interrupt/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Establish the specified external hardware connections.
- Observe the logs printed on the console.

\imageStyle{am263_dpl_low_latency_interrupt_c.png,width:80%}
\image html am263_dpl_low_latency_interrupt_c.png "External connections required"

# See Also

\ref KERNEL_DPL_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
DPL Low latency interrupt example started...

Interrupt Latency test...

Configuring source of interrupts... Done.
Registering interrupt handler in R5F VIM... Done.
Configuring pin to toggle in background loop... Done.
Please observe pin (GPIO 43)
   If toggling         -> Background loop running
   If gap/no toggling  -> Background loop interrupted. Interrupt routine running
Waiting for few seconds... Done.
De-initializations... Done.

Nesting and float point context save test...

Configuring source of interrupts... Done.
Registering interrupt handlers in R5F VIM... Done.
Configuring pins to use in interrupt routine... Done.
Please observe pins (GPIO 44 and 45)
   If pin 44 high -> ISR 1 running
   If pin 45 high -> ISR 2 running
Waiting for few seconds...
De-initializations... Done.

All tests have passed!!
\endcode

\imageStyle{am263_dpl_low_latency_interrupt_d.png,width:80%}
\image html am263_dpl_low_latency_interrupt_d.png "Sample output"
