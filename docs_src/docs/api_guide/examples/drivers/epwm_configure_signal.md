# EPWM Configure Signal API usage {#EXAMPLES_DRIVERS_EPWM_CONFIGURE_SIGNAL}

[TOC]

# Example Description
 This example configures EPWM0, EPWM1, epEPWM2 to produce signal of desired
frequency and duty. It also configures phase between the configured
modules.

Signal of 10kHz with duty of 0.5 is configured on ePWMxA & ePWMxB
with ePWMxB inverted. Also, phase of 120 degree is configured between
EPWM0 to EPWM2 (EPWM2 to EPWM4 in case of AM261x-LP) signals.
 
During the test, monitor EPWM0, EPWM1, and/or EPWM2 (EPWM2, EPWM3 and/or
EPWM4 in case of AM261x-LP) outputs on an oscilloscope.

# External Connections

## On AM263x CC/ AM263Px CC with HSEC Dock
Probe the following on the HSEC pins
 - EPWM 0A/0B : 49 / 51
 - EPWM 1A/1B : 53 / 55
 - EPWM 2A/2B : 50 / 52
 
## On AM263x LP/ AM263Px LP
Probe the following on boosterpack
 - EPWM 0A/0B : J4 11 / J8 59
 - EPWM 1A/1B : J2 37 / J2 38
 - EPWM 2A/2B : J2 39 / J2 40
## On AM261x LP
Probe the following on boosterpack
 - EPWM 2A/2B : J2 40 / J8 39
 - EPWM 3A/3B : J2 38 / J2 37
 - EPWM 4A/4B : J2 36 / J2 35
# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_CONFIGURE_SIGNAL_COMBOS}

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_configure_signal/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_EPWM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EPWM Configure Signal Test Started ...
EPWM Configure Signal Example runs for 5 Secs 
EPWM Configure Signal Test Passed!!
All Tests have Passed!!

\endcode

\imageStyle{am263_epwm_configure_signal_output.png,width:75%}
\image html am263_epwm_configure_signal_output.png "EPWM Chopper Example Sample Capture"
