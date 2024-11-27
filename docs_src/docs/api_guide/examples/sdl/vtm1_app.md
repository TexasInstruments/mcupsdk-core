# SDL MCU VTM {#EXAMPLES_SDL_VTM}

[TOC]

# Introduction

This example demonstrates the usage of the VTM to detect overtemperature events and signal the events through the interrupt/ESM. The example shows how to configure the VTM for the various alerts and listen for VTM events using the interrupts/ESM. It also shows forcing an error in order to demonstrate application notification of overtemperature events.

The example demonstrates:

* Configuring the VTM thresholds
* Early alert event detection
* Critial threshold event detection
* Follow-up event notification when temperature has dropped to a safe level


Use Cases
---------
Use Case | Description
---------|------------
UC-0     | Configuration of VTM threshold to trigger Low and High Threshold Hot alert warning.
UC-1     | Configuration of VTM threshold to trigger Hot alert warning, then return thresholds back to normal to simulate "cooling down".
UC-2     | Configuration of VTM threshold to trigger ESM interrupt.

# Supported Combinations {#EXAMPLES_SDL_VTM_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss1-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/vtm/


# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_VTM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code

[Cortex_R5_0]
 VTM Example Application


 UC1 :
 Device Temperature: 70000mc
 Configured hot alert Temperature: 58000mc
 Hot Alert Temperature breached and received Interrupt : 133

 Device Temperature: 70000mc
 Configured cold alert Temperature: 72000mc
 Cold Alert Temperature breached and received Interrupt
 Temperature is below Configured Low Temperature and received Interrupt : 133

 UC2 :
 Device Temperature: 70000mc
 Configured Low threshold Temperature: 66000mc
 Low Threshold Temperature breached and received Interrupt: 134

 Device Temperature: 70000mc
 Configured hot threshold Temperature: 66000mc
 Exceeded Configured Temperature and received Interrupt : 133

 UC3 :
ESM_Test_init: Init MSS ESM complete

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x1, index 0xc, intSrc 0x2c

 Device Temperature: 70000mc
 Configured Low threshold Temperature: 74000mc
 Configured hot threshold Temperature: 66000mc
 ESM Interrupt Received
 All tests Passed

\endcode
