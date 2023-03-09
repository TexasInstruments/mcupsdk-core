# SDL POK Example {#EXAMPLES_SDL_POK}

[TOC]

This example demonstrates the usage of the POK modules. The example shows how to listen for POK error events using the ESM, and also forcing an error in order to demonstrate application notification of overvoltage and undervoltage events.

The example demonstrates:

    * Undervoltage event detection
    * Overvoltage event detection

Use Cases
---------
Use Case | Description
---------|------------
UC-0     | An under-voltage POK instance is configured to a threshold value that will force the undervoltage
           event.Error event is triggered and application is notified of the event.
UC-1     | An over-voltage POK instance is configured to a threshold value that will force the overvoltage event. Error event is triggered and application is notified of the event.

# Supported Combinations {#EXAMPLES_SDL_POK_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 ^              | r5ss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/pok/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/pok/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_POK_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code

POK Test Application

sdlEsmSetupForPOK init: Init MCU ESM complete


 POK ID = 1 , monitoring set to UV
Waiting for ESM to report the error
 Got the ESM Error Interrupt
Action taken
Safety software Example UC-1 pok for instance 1 PASSED


 POK ID = 11 , monitoring set to OV
Waiting for ESM to report the error
 Got the ESM Error Interrupt
Action taken
Safety software Example UC-2 pok for instance  11 PASSED
Test Name: POK EXAMPLE UC-1  PASSED
Test Name: POR EXAMPLE UC-2  PASSED

 All tests have passed.

\endcode