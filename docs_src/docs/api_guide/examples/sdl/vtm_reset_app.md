# SDL MCU VTM RESET {#EXAMPLES_SDL_VTM_RESET}

[TOC]

# Introduction

This example demonstrates the usage of the VTM to detect Max temperature out of range event and trigger SoC reset. The example shows how to configure the VTM for the thresehold temperature and enabling the max temperature out of range event.

The example demonstrates the following, in sequence:

* Configuring the VTM max temperature threshold and enabling the event
* Waiting for rise in temperature, caused by idle running or external influence
* Max temperature out of range detection
* Triggering of SoC reset by VTM module alert
* Detection that thermal reset of SoC had occurred, after cooling down
* Follow-up disabling of max temp out of range event


Use Cases
---------
Use Case | Description
---------|------------
UC-0     | Configuration of VTM threshold to trigger SoC reset, then after cooldown, disable max temp out of range event.


# Supported Combinations {#EXAMPLES_SDL_VTM_RST_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/vtm_reset/

\endcond


# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- After SoC reset, cool down the device and run the program again

# See Also

\ref SDL_VTM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code

VTM Example Application

Starting Use Case 0

VTM_config: starting test with threshold change
sensor id                       : 0
temp in milli degree celcius    : 44754
sensor id                       : 0
temp in milli degree celcius    : 46796
sensor id                       : 0
temp in milli degree celcius    : 47247
sensor id                       : 0
temp in milli degree celcius    : 46796
sensor id                       : 0
temp in milli degree celcius    : 46796

VTM_triggerTh: Setting high temp to 48247 millidegrees Celsius
VTM_triggerTh: Setting low temp to 47297 millidegrees Celsius

Finished VTM threshold setting

Now waiting for thermal reset...
temp in milli degree celcius    : 47247
temp in milli degree celcius    : 46796
temp in milli degree celcius    : 47247
temp in milli degree celcius    : 46570
temp in milli degree celcius    : 47021
temp in milli degree celcius    : 47472
temp in milli degree celcius    : 47247
temp in milli degree celcius    : 47247

\endcode

Note that at this point in the execution, the high temperature threshold was achieved. This triggered an SoC reset (and in turn, a target disconnection if connected via CCS). After cooling down the SoC, if the example is run again, the following logs will be obtained.

\code

VTM Example Application

Reset source was thermal.
VTM Maximum Temperature Outrange Alert disabled.
VTM Reset Example Application completed successfully.

VTM Reset Example Application summary
-------------------------------
Completed 1 Test Case(s)

VTM Reset Example Application: Complete
All tests have passed.

\endcode