# GPADC Temperature Read{#EXAMPLES_DRIVERS_GPADC_TEMPERATURE_SENSOR_READ}

[TOC]

# Introduction

This example configures a GPADC configuration for temperature sensors and triggers the  group channel conversion. Upon successful conversion temperature values for the sensors is displayed. Conversion happens based on the parameters provided through syscfg

# Supported Combinations {#EXAMPLES_DRIVERS_GPADC_TEMPERATURE_SENSOR_READ_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpadc/gpadc_temperature_sensor_read/

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_GPADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
Temperature read conversion successful
DSP Temp Sensor temp value 41
HWA Temp Sensor temp value 41
HSM Temp Sensor temp value 42
All tests have passed!!

\endcode
