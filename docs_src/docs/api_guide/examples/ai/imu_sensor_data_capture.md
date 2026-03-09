# IMU Sensor Data Capture {#EXAMPLES_AI_IMU_SENSOR_DATA_CAPTURE}

[TOC]

# Introduction

This example demonstrates real-time accelerometer-based activity classification using the BMI270 sensor on the TIDA-010997 booster pack. It showcases both data capture for dataset building and live inference preview capabilities, enabling seamless integration with Edge AI Studio.

The following modes are supported:
- **Live Capture (DATA_ACQUISITION)**: Streams raw 3-axis accelerometer readings to build datasets for training custom models
- **Live Preview (SENSOR_INFERENCE)**: Runs the trained classification model and streams inference results in real-time

# Supported Combinations {#EXAMPLES_AI_IMU_SENSOR_DATA_CAPTURE_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/imu_sensor_data_capture/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/imu_sensor_data_capture/

\endcond

# Hardware Requirements

- TIDA-010997 Edge AI Sensor Boosterpack

# External Connections

- Mount TIDA-010997 Edge AI Sensor Boosterpack on BoosterPack Headers Site 1 (J1/J3 and J2/J4).

# Application Overview

This example supports dual-mode operation controlled by Edge AI Studio:
1. **DATA_ACQUISITION mode**: Streams raw 3-axis accelerometer samples (6 bytes per sample: X, Y, Z as INT16)
2. **SENSOR_INFERENCE mode**: Collects frames of 128 samples, applies feature extraction (FFT, binning, log scaling), runs inference using the trained classification model, and outputs classification results

The inference classifies motion into two classes:
- Class 0: Jerk
- Class 1: Smooth

Test vectors are provided for validation and reference testing purposes.

# Dependencies

- Edge AI Studio (for live capture and preview modes)
- BMI270 sensor hardware (TIDA-010997 booster pack)

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Connect the TIDA-010997 booster pack with BMI270 sensor
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- For live capture or live preview mode, connect Edge AI Studio to the device via UART

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
==========================================================
[IMU] IMU Sensor Data Capture Example (Dual-Mode DAP)
==========================================================
[IMU] Feature extraction initialized
[IMU]   Variables: 3 (axes)
[IMU]   Frame size: 128 samples
[IMU]   Model input size: 384
[IMU]   Model output size: 2 classes
[IMU] Running test vector validation...
[IMU] Test vector validation PASSED!
[IMU] Model output: [45, -23], Golden: [45, -23]
[IMU] BMI270 sensor initialized (Accel: +/-8g, 100Hz)
[IMU] DAP initialized. Waiting for Edge AI Studio...
[IMU] Supported modes: DATA_ACQUISITION, SENSOR_INFERENCE
==========================================================
\endcode
