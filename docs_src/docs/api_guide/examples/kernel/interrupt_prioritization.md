# Driver Porting Layer (DPL) Interrupt Prioritization {#EXAMPLES_KERNEL_DPL_INTERRUPT_PRIORITIZATION}

[TOC]

# Introduction

This examples demonstrates the prioritization of interrupts in VIM through nesting of 3 Timer interrupts
of different priority.

The example does the below
- Sets up Timer's 0,1,2 with same tick period.
- Sets up Timer interrupt with Timer 0 being highest priority, TImer 2 lowest.
- Nesting of interrupts is visualised with the help of trace buffer which captures
  the start and end of each Timer ISR.
- Timer2 is started first followed by Timer1 and then Timer0.
- Timer2 Interrupts gets nested by Timer1 interrupt which in turn gets
  nested by Timer0 interrupt.
- The trace macros used are -
  - 0xA0 - Timer0 ISR start
  - 0xB0 - Timer0 ISR end
  - 0xA1 - Timer1 ISR start
  - 0xB1 - Timer1 ISR end
  - 0xA2 - Timer2 ISR start
  - 0xB2 - Timer2 ISR end

\note  This example uses SysCfg to configure priority of Timer interrupts. For configuring
 the priority of interrupts which are not supported by SysCfg, use the element "HwiP_Params.priority"
 in HwiP. Priority can be specified from a level of 0 to 15, where 0 denotes highest priority.

# Supported Combinations

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/interrupt_prioritization/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The trace gets printed and can be used to verify interrupt nesting and prioritization.

# See Also

\ref KERNEL_DPL_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[DPL] Interrupt prioritization Test Started...
[DPL] All timers stopped
[DPL] Printing Trace -
A2 - A1 - A0 - B0 - B1 - B2 - A2 - A1 - A0 - B0 - B1 - B2 - A2 - A1 - A0 - B0 - B1 - B2 - A2 - A1 - A0 - B0 - B1 - B2 -
[DPL] Interrupt prioritization Test Passed!
All tests have passed!!
\endcode
