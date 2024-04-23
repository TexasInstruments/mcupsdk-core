# CMPSS Asynchronous trip {#EXAMPLES_DRIVERS_CMPSS_ASYNCHRONOUS_TRIP}

[TOC]

# Introduction


A CMPSS example that enables the CMPSS High comparator and feeds the
asynchronous output to GPIO and EPWM

# @VAR_BOARD_NAME
This example enables the CMPSSA0 COMPH comparator and feeds the asynchronous
CTRIPOUTH signal to the XBAROUT0 pin and CTRIPH to EPWM0B.

\imageStyle{am263_cmpss_asynchronous_trip.png,width:50%}
\image html am263_cmpss_asynchronous_trip.png "Block diagram"

CMPSS is configured to generate trip signals to trip the EPWM signals.
CMPIN1P is used to give positive input and internal DAC is configured
to provide the negative input. Internal DAC is configured to provide a
signal at VDD/2. An EPWM signal is generated at EPWM0B and is configured
to be tripped by CTRIPOUTH.

When a low input(VSS) is provided to CMPIN1P,
    - Trip signal(XBAROUT0) output is low
    - EPWM0B gives a PWM signal

When a high input(higher than VDD/2) is provided to CMPIN1P,
    - Trip signal(XBAROUT0) output turns high
    - EPWM0B gets tripped and outputs as high


# @VAR_LP_BOARD_NAME
This example enables the CMPSSA1 COMPH comparator and feeds the asynchronous
CTRIPOUTH signal to the XBAROUT10 pin and CTRIPH to EPWM0B.

CMPSS is configured to generate trip signals to trip the EPWM signals.
CMPIN1P is used to give positive input and internal DAC is configured
to provide the negative input. Internal DAC is configured to provide a
signal at VDD/2. An EPWM signal is generated at EPWM0B and is configured
to be tripped by CTRIPOUTH.

When a low input(VSS) is provided to CMPIN1P,
    - Trip signal(XBAROUT10) output is low
    - EPWM0B gives a PWM signal

When a high input(higher than VDD/2) is provided to CMPIN1P,
    - Trip signal(XBAROUT10) output turns high
    - EPWM0B gets tripped and outputs as high

# External Connections
 - Give input on CMPIN1P
 - Outputs can be observed on XBAROUT0 and EPWM0B using an oscilloscope

## AM263PX-CC E2 or AM263X-CC E2
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Feed analog input on HSEC Pin 15
- Capture and analyze waveforms on USER_LED1 and HSEC pin 51

## AM263PX-LP or AM263X-LP
When using AM263x-LP
- Feed analog input on J5/J6 Pin 66
- Capture and analyze waveforms on DS2 LED and boosterpack header J6/J8 pin 59

# Supported Combinations {#EXAMPLES_DRIVERS_CMPSS_ASYNCHRONOUS_TRIP_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/cmpss/cmpss_asynchronous_trip/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- When a low input is provided, Trip signal output is low and EPWM gives a PWM waveform
- When a high input(higher than VDD/2) is provided, Trip signal output turns high and EPWM gets tripped

# See Also

\ref DRIVERS_CMPSS_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[Cortex_R5_0] CMPSS asynchronous trip Test Started ...
CMPSS asynchronous trip Test Passed!!!All tests have Passed!!!CMPSS asynchronous trip Test Started ...
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CTRIP signal is asserted
CMPSS asynchronous trip Test Passed!!!All tests have Passed!!!
\endcode

