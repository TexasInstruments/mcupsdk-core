# TMU Context Save and Restore {#EXAMPLES_DRIVERS_TMU_CONTEXTSAVE}

[TOC]

# Introduction

This example demonstrates the Context Save and Restore functionality of TMU. We make use of a software interrupt
that will get enable while the TMU operation is being executed. On receiving this high priority interrupt, the ISR gets invoked
which will use the TMU hardware. Due to the context save functionality of the TMU result registers, we can enable the use of TMU
in an ISR context while simultaneously used in main function.

When the interrupt is taken and the ISR starts running, the context save can be initiated by writing ‘1’ to CONTEXT_SAVE.SAVE bit.
TMU result registers are saved to CSAVE_<*> registers. Context save will happen only after all operations initiated before writing to
CONTEXT_SAVE.SAVE are complete. This is to ensure that context save happens at correct point. Even though TMU operations are multi-cycle,
the TMU operation will have completed by the time context save operation is initiated in ISR. So no additional measure is needed in ISR.
After saving context, the ISR can use the TMU without any restriction.

Here in this eaxmple, we calculate the sin of an input value using TMU operation and store it in a variable. After this an interrupt
get enabled, and within the ISR the TMU operations is called which will overwrite the values in the existing result registers, to avoid this
the context save functionality is initiated that store the values of the result registers in the corresponding context save registers. Before
exiting the ISR, the context restore functionality is enabled which will restore the values present in the context save registers to the
correponding result registers. In the main function, we store the value present in the result register in another variable and draw a
comparison between the two  values that got stored before interrupt and after interrupt.


\imageStyle{am263p_tmu_context_save.png,width:80%}
\image html am263p_tmu_context_save.png "Flow chart depicting example flow"


# Supported Combinations {#EXAMPLES_DRIVERS_TMU_CONTEXTSAVE_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/tmu/tmu_context_save/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- View the console log for results


# Sample Output

Shown below is a sample output when the application is run,

\code
TMU Context Save and Restore Test Started ...
TMU Context Save and Restore Example runs for 10 Secs
Value in result register before interrupts occur : -0.087851
Value in result register after interrupts occur : -0.087851

TMU Context Save and Restore Test Passed!!
All tests have passed!!
\endcode

