# ECAP EDMA Example {#EXAMPLES_DRIVERS_ECAP_EDMA}

[TOC]

# Introduction

 This example demostrates the ECAP triggerring EDMA for transfer of the ECAP timestamp registers to the TCM location without R5 intervention. ECAP can generate EDMA triggers for Data transfer in both CAPTURE mode and APWM mode
- In ECAP Capture Mode, the CAPEVTx [x= 1 to 4] can be used and
- In ECAP APWM Mode, The counter match to Compare or Period can be used.

## Example Description:
 An ECAP (APWM_ECAP) is configured in APWM mode to generate trigger at its counter matching period event to transfer timestamp data from "CAPTURE_ECAP2". 2 ECAPs (CAPTURE_ECAP1, CAPTURE_ECAP2) are configured to capture the APMW_ECAP PWM wave. CAPTURE_ECAP1 also triggers EDMA on one of its Capture events.

\imageStyle{am263_ecap_edma_example_block_diagram.png,width:50%}
\image html am263_ecap_edma_example_block_diagram.png "Example Block diagram"

### ECAP Configurations:
1. APWM_ECAP:
- APWM mode, 5KHz and 25% duty cycle
- DMA trigger at Counter = Period event for channel 0
- Output routed via Outputxbar13 (GPIO119)

2. CAPTURE_ECAP1:
- CAPTURE MODE, capturing Falling, Rising, Falling, Rising edges on its four events,
- counter resets at CAPEVT1,2,3,4. Thereby, recording
     - ON time at CAP1, CAP3,
     - OFF times at CAP2,CAP4,
- CAPEVT4 is the source for the DMA channel 1 trigger for the transfer of its timestamps in CAPx

3. CAPTURE_ECAP2:
- CAPTURE mode, Capturing Falling, Rising edges on its 2 events,
- counter resets on CAPEVT1,2. thereby recording,
     - ON time on CAP1
     - OFF time on CAP2
 A total of 256 transfers on Channel 0 and 128 transfers on Channel 1 are triggered, and the buffers "capTrigTimeStamps" and "pwmTrigTimeStamps" are compared.

# Watch Variables :
1. capTrigTimeStamps [ ]: array with CAPTURE_ECAP1 Captured timestamps, triggered by Capture mode.
2. pwmTrigTimeStamps [ ]: array with CAPTURE_ECAP2 Captured timestamps, triggered by APWM mode.

# External Connections

## AM263X-CC
No external connection is required.

## AM263X-LP
No external connection is required.

# Supported Combinations {#EXAMPLES_DRIVERS_ECAP_EDMA_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ecap/ecap_edma/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_ECAP_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ECAP triggered EDMA transfers Test Started ...
EDMA transfers complete.
ECAP triggered EDMA transfers Passed!!
All tests have passed!!
\endcode
