# SDL MCU  ESM {#EXAMPLES_SDL_MCU_ESM}

[TOC]

# Introduction

This example demonstrates High and Low priority events received from the MCU_ESM through the application-registered callback. It also shows routing and clearing of events to the MCU_SAFETY_ERRORn pin.

This example displays setting up ESM to

    * Enable/Disable input events to the ESM module for each ESM instance
    * Registration of application callback for ESM error events for each ESM instance
    * Designating input events as high priority or low priority for each ESM instance
    * Designating direct pin routing to MCU_SAFETY_ERRORn pin for external monitor communication


Use Cases
---------
Use Case | Description
---------|------------
UC-0     | Configuration of High Priority ESM
UC-1     | Configuration of Config Priority ESM
UC-2     | Configuration of Low Priority ESM


# Supported Combinations {#EXAMPLES_SDL_MCU_ESM_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/mcu_esm0/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
# Sample Output

Shown below is a sample output when the application is run,

\code
ESM Example Application

TIMER_ESM_init: Init MCU ESM complete

ESM example init complete
  Any clear of MCU_SAFETY_ERRORn pin will first wait 10 usecs
  Minimum Time Interval is 2000 usecs
ESM timer initialization complete

Starting Test Case 0

Use Case 0 completed: Input Event Trigger = Step completed successfully,
                       Event Handler Complete = Step completed successfully,
                       MCU_SAFETY_ERRORn Pin Clear = Step completed successfully

Starting Test Case 1

Use Case 1 completed: Input Event Trigger = Step completed successfully,
                       Event Handler Complete = Step completed successfully,
                       MCU_SAFETY_ERRORn Pin Clear = Step completed successfully

Starting Test Case 2

Use Case 2 completed: Input Event Trigger = Step completed successfully,
                       Event Handler Complete = Step completed successfully,
                       MCU_SAFETY_ERRORn Pin Clear = Step completed successfully


ESM Example Application summary
-------------------------------
Completed 3 Test Cases
Received 1 High Priority Interrupts
Received 1 Low Priority Interrupts
Received 1 Config Priority Interrupts

Test Case Event Log
------------------

Test Case 0: ESM Call back function called : grpChannel 0x2, index 0x1, intSrc 0x41
  ESM instance #1, ESM interrupt type = High Priority ESM event

Test Case 1: ESM Call back function called : grpChannel 0x1, index 0x0, intSrc 0x0
  ESM instance #1, ESM interrupt type = Config ESM event

Test Case 2: ESM Call back function called : grpChannel 0x1, index 0x3, intSrc 0x23
  ESM instance #1, ESM interrupt type = Low Priority ESM event

ESM Example Application: Complete
 All tests have passed!!
\endcode
