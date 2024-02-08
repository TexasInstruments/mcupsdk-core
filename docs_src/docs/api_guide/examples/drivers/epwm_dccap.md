# EPWM Digital counter compare capture {#EXAMPLES_DRIVERS_EPWM_DCCAP}

[TOC]

# Introduction

An EPWM example to detect occurrence of a trip event in a configured time window.
The window is configured by MIN and MAX values configured in MINMAX register set.

Purpose of this window is to detect the occurrence of such edge. If no such edge occurs, this module will generate a trip event as well as
interrupt configurable by user.

In this example EPWM1_A is used for Digital compare input on which logic is performed.
EPWM0_A is used as the gating signal to Min/max logic. (Routed to PWM XBar from GPIO)

If an edge is not detected on EPWM0 in the MINMAX window,
    - EPWM1 is tripped

\imageStyle{am263_epwm_dccap_flow.png, width:60%}
    \image html am263_epwm_dccap_example_flow.png "Flow Chart"

# External Connections

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Connect EPWM0_A(HSEC pin 49) to GPIO15(HSEC pin 81)
- Output can be observed from EPWM1_A(HSEC pin 51)
- CAPEVT TripOut can be observed from XBAROUT3(HSEC pin 75)


# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_DCCAP_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_dccap

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
EPWM Capture logic test

Iteration: 1
PWMXBAR_STATUS = 0x00000001
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 2
PWMXBAR_STATUS = 0x00000000
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 3
PWMXBAR_STATUS = 0x00000001
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 4
PWMXBAR_STATUS = 0x00000000
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 5
PWMXBAR_STATUS = 0x00000000
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 6
PWMXBAR_STATUS = 0x00000001
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 7
PWMXBAR_STATUS = 0x00000000
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 8
PWMXBAR_STATUS = 0x00000002
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 9
PWMXBAR_STATUS = 0x00000002
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 10
PWMXBAR_STATUS = 0x00000003
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 11
PWMXBAR_STATUS = 0x00000002
DCCAP = 209
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 12
PWMXBAR_STATUS = 0x00000000
DCCAP = 709
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 13
PWMXBAR_STATUS = 0x00000000
DCCAP = 709
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 14
PWMXBAR_STATUS = 0x00000000
DCCAP = 709
DCCAP Status = 1
DCCAPCTL = 0x2003

Iteration: 15
PWMXBAR_STATUS = 0x00000000
DCCAP = 709
DCCAP Status = 1
DCCAPCTL = 0x2003

All tests have passed!!
\endcode


\cond SOC_AM263X
\imageStyle{am263_epwm_dccap_example_test1.PNG, width:60%}
    \image html am263_epwm_dccap_example_test1.PNG "EPWM DCCAP Test 1"
\imageStyle{am263_epwm_dccap_example_test2.PNG, width:60%}
    \image html am263_epwm_dccap_example_test2.PNG "EPWM DCCAP Test 2"
\imageStyle{am263_epwm_dccap_example_test3.PNG, width:60%}
    \image html am263_epwm_dccap_example_test3.PNG "EPWM DCCAP Test 3"
\imageStyle{am263_epwm_dccap_example.PNG, width:60%}
    \image html am263_epwm_dccap_example.PNG "EPWM DCCAP"
\endcond