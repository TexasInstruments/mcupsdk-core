# SDFM ECAP Loop Back Example {#EXAMPLES_DRIVERS_SDFM_ECAP_LOOP_BACK}

[TOC]

# Introduction

\imageStyle{am263p_sdfm_ecap_loop_back_block_diagram.png,width:75%}
\image html am263p_sdfm_ecap_loop_back_block_diagram.png "Example Block diagram"

## Example Description
ECAP in its APWM mode can be used a Clock for the SDFM. This example demonstrates the configurations and usage of such internal loopback. In AM263Px, the loopback configurations can be done to leverage the following pairs.
\cond SOC_AM263PX
SDFM instance   |   Clock       | ECAP instances
----------------|---------------|-----------------
        0       |       0       |   4(default), 12
        0       |       1       |   5(default), 13
        0       |       2       |   6(default), 14
        0       |       3       |   7(default), 15
        1       |       0       |   7(default), 15
        1       |       1       |   6(default), 14
        1       |       2       |   5(default), 13
        1       |       3       |   4(default), 12
\endcond
\cond SOC_AM261X
SDFM instance   |   Clock       | ECAP instances
----------------|---------------|-----------------
        0       |       0       |   0(default), 7
        0       |       1       |   1(default), 6
        0       |       2       |   2(default), 5
        0       |       3       |   3(default), 4
        1       |       0       |   4(default), 3
        1       |       1       |   5(default), 2
        1       |       2       |   6(default), 1
        1       |       3       |   7(default), 0
\endcond

In this example, SDFM filters 2,3 are configured for the clock input to have from filter 1 and filter 1 is configured to take the loopback clock. The Data interrupts from the SDFM filter 2,3 invoke the ISR, where the CPU reads the data form FIFO of Filter 2 and Data result of Filter 3.
## Configurations
SDFM configuration is shown below:
-  SDFM used in this example - SDFM0
-  Input control mode selected - MODE0
-  Data filter settings
    - filters 2,3  modules enabled
    - Sinc1 filter selected
    - OSR = 256
    - All the filters are synchronized by using MFE (Main Filter enable bit)
    - Filter output represented in 16 bit format
- Interrupt module settings for SDFM filter
    - FIFO full interrupt from Filter 2 and Data ready from Filter 3 are enabled.

# External Connections
- Connect Sigma-Delta data streams to SDFM0_D1, SDFM0_D2
- Output xbar out 8 or SDFM0_CLK0 can be used for providing clock to external sdfm modulator.
## AM263PX-CC (E2) with 180 pin HSEC Dock
- SDFM0_CLK0 can be probed at HSEC Pin 72 for Clock
- OutputXbar8 can be probed at HSEC Pin 85 for Clock
- Provide Data Streams on SDFM0_D1 at HSEC 99   or, alternately, use EPWM0_A output for data stream input from HSEC 49
- Provide Data Streams on SDFM0_D2 at HSEC 103  or, alternately, use EPWM0_A output for data stream input from HSEC 49
## AM263PX-LP (E2) with 180 pin HSEC Dock
Not Supported

## Watch  Variables
 -   filter2Result - Output of filter 2
 -   filter3Result - Output of filter 3

# Supported Combinations {#EXAMPLES_DRIVERS_SDFM_ECAP_LOOP_BACK_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/sdfm/sdfm_ecap_loop_back/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- View the UART console logs for results

# See Also

\ref DRIVERS_SDFM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
SDFM filter sync CPU read Test Started ...
Max Samples read complete. Printing some of the values...
Filter out :
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256
Corresponding FIFO DATA
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256

Filter out :
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256
Corresponding FIFO DATA
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256

Filter out :
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256
Corresponding FIFO DATA
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256

Filter out :
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256
Corresponding FIFO DATA
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256

Filter out :
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256
Corresponding FIFO DATA
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256

Filter out :
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256
Corresponding FIFO DATA
 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256 256

All tests have passed!!

\endcode

