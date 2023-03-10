# SDL MCU VTM {#EXAMPLES_SDL_VTM}

[TOC]

# Introduction

This example demonstrates the usage of the VTM to detect overtemperature events and signal the events through the ESM. The example shows how to configure the VTM for the various alerts and listen for VTM events using the ESM. It also shows forcing an error in order to demonstrate application notification of overtemperature events.

The example demonstrates:

* Configuring the VTM thresholds
* Early alert event detection
* Critial threshold event detection
* Follow-up event notification when temperature has dropped to a safe level


Use Cases
---------
Use Case | Description
---------|------------
UC-0     | Configuration of VTM threshold to trigger early warning, then return thresholds back to normal to simulate "cooling down".
UC-1     | Configuration of VTM to trigger early warning, then alter the thresholds again to pass temperature critical threshold. Then, return thresholds back to normal to simulate "cooling down".


# Supported Combinations {#EXAMPLES_SDL_VTM_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
                  r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/vtm/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/vtm/

\endcond

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

VTM Example Application

VTM_ESM_init: Init MAIN ESM complete 

 VTM_ESM_init complete
Starting Use Case 0 
sensor id                       : 0 
adc_code                        : 348 
temp in milli degree celcius    : 53261 
vtmTriggerTh: Setting lt_thr0_val temp to 49261 millidegrees Celsius, and adc_code_lt_thr0 = 330
vtmTriggerTh: Setting gt_thr1_val temp to 51261 millidegrees Celsius, and adc_code_gt_thr1 = 340
vtmTriggerTh: Setting gt_thr2_val temp to 58261 millidegrees Celsius, and adc_code_gt_thr2 = 370
Finished VTM threshold setting
case 0 success

 Use Case 0 completed: Input Event Trigger = Step completed successfully, 

Starting Use Case 1 
sensor id                       : 0 
adc_code                        : 349 
temp in milli degree celcius    : 53493 
vtmTriggerTh: Setting lt_thr0_val temp to 49493 millidegrees Celsius, and adc_code_lt_thr0 = 332
vtmTriggerTh: Setting gt_thr1_val temp to 51493 millidegrees Celsius, and adc_code_gt_thr1 = 340
vtmTriggerTh: Setting gt_thr2_val temp to 58493 millidegrees Celsius, and adc_code_gt_thr2 = 370
Finished VTM threshold setting

 Use Case 1 completed: Input Event Trigger = Step completed successfully, 


ESM Example Application summary
-------------------------------
Completed 2 Test Cases
Received 1 High Priority Interrupts
Received 5 Low Priority Interrupts

Test Case Event Log
------------------

Test Case 0: ESM Call back function called : grpChannel 0x4, index 0x9, intSrc 0x89 
  ESM instance #2, ESM interrupt type = Low Priority ESM event

Test Case 0: ESM Call back function called : grpChannel 0x4, index 0x8, intSrc 0x88 
  ESM instance #2, ESM interrupt type = Low Priority ESM event

Test Case 1: ESM Call back function called : grpChannel 0x4, index 0x9, intSrc 0x89 
  ESM instance #2, ESM interrupt type = Low Priority ESM event

Test Case 1: ESM Call back function called : grpChannel 0x4, index 0xa, intSrc 0x8a 
  ESM instance #2, ESM interrupt type = High Priority ESM event

Test Case 1: ESM Call back function called : grpChannel 0x4, index 0x9, intSrc 0x89 
  ESM instance #2, ESM interrupt type = Low Priority ESM event

Test Case 1: ESM Call back function called : grpChannel 0x4, index 0x8, intSrc 0x88 
  ESM instance #2, ESM interrupt type = Low Priority ESM event

 VTM Example Application: Complete
 All Use cases have passed.  

\endcode
