# Temperature Sensor Data Capture {#EXAMPLES_AI_TEMP_SENSOR_DATA_CAPTURE}

[TOC]

# Introduction

This example demonstrates temperature sensor data capture for building datasets used in training machine learning models. It streams temperature readings to Edge AI Studio for dataset collection.

The following mode is supported:
- **Live Capture**: Streams temperature readings to build datasets for training custom models

# Supported Combinations {#EXAMPLES_AI_TEMP_SENSOR_DATA_CAPTURE_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/temp_sensor_data_capture/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/temp_sensor_data_capture/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/temp_sensor_data_capture/

\endcond

# Hardware Requirements

- TIDA-010997 Edge AI Sensor Boosterpack

# External Connections

- Mount TIDA-010997 Edge AI Sensor Boosterpack on BoosterPack Headers Site 1 (J1/J3 and J2/J4).

# Dependencies

- Edge AI Studio for data capture
- Compatible temperature sensor hardware

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Connect the sensor hardware as described in External Connections
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Connect Edge AI Studio to the device via UART for data capture

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
[TEMP] TIDA-010997 Temperature Sensor Data Capture Example
[TEMP] HDC3020 sensor initialized at 10Hz
[TEMP] DAP initialized. Waiting for Edge AI Studio...
\endcode
