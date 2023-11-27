# CMPSS Digital Filter trip {#EXAMPLES_DRIVERS_CMPSS_DIGITAL_FILTER_TRIP}

[TOC]

# Introduction


A CMPSS example that enables the CMPSS High comparator and feeds the
digital filter output to GPIO/OUTPUTXBAR pin

# @VAR_BOARD_NAME
This example enables the CMPSSA1 COMPH comparator and feeds the digital filter CTRIPH signal to the OUTPUTXBAR1 pin.

\imageStyle{am263_cmpss_digital_filter_trip.png,width:50%}
\image html am263_cmpss_digital_filter_trip.png "Block diagram"

CMPIN1P is used to give positive input and internal DAC is configured
to provide the negative input. Internal DAC is configured to provide a
signal at VDD/2. A Digital Filter signal is generated and is configured
to be tripped by CTRIPOUTH.

When a low input(VSS) is provided to CMPIN1P,
    - Trip signal(OUTPUTXBAR0) output is low

When a high input(higher than VDD/2) is provided to CMPIN1P,
    - Trip signal(OUTPUTXBAR0) output turns high

External Connections \n
    - Give input on CMPIN1P (HSEC Pin 15)
    - Outputs can be observed on OUTPUTXBAR1 (HSEC Pin 81)


# @VAR_LP_BOARD_NAME
This example enables the CMPSSA1 COMPH comparator and feeds the digital filter CTRIPH signal to the OUTPUTXBAR1 pin.

CMPIN1P is used to give positive input and internal DAC is configured
to provide the negative input. Internal DAC is configured to provide a
signal at VDD/2. A Digital Filter signal is generated and is configured
to be tripped by CTRIPOUTH.

When a low input(VSS) is provided to CMPIN1P,
    - Trip signal(OUTPUTXBAR1) output is low

When a high input(higher than VDD/2) is provided to CMPIN1P,
    - Trip signal(OUTPUTXBAR1) output turns high

External Connections \n
    - Give input on CMPIN1P (HSEC Pin 15)
    - Outputs can be observed on OUTPUTXBAR1

# External Connections
 - Give input on CMPIN1P (HSEC Pin 15)
 - Outputs can be observed on XBAROUT1


# Supported Combinations {#EXAMPLES_DRIVERS_CMPSS_DIGITAL_FILTER_TRIP_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/cmpss/cmpss_digital_filter_trip/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- When a low input is provided, Trip signal output is low
- When a high input(higher than VDD/2) is provided, Trip signal output turns high

# See Also

\ref DRIVERS_CMPSS_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\imageStyle{am263_cmpss_digital_filter_trip_output.png,width:50%}
\image html am263_cmpss_digital_filter_trip_output.png "Output Waveform"

