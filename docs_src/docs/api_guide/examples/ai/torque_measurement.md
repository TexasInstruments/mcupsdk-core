# Torque Measurement {#EXAMPLES_AI_TORQUE_MEASUREMENT}

[TOC]

# Introduction

Predicting rotor torque in motors presents a significant engineering challenge: direct measurement requires costly sensors that add weight, a critical concern for electric vehicles striving for efficiency. While traditional statistical methods for estimating rotor torque exist, they are limited by the need for domain expertise and motor-specific tuning.

This project leverages deep learning to efficiently predict rotor torque in Permanent Magnet Synchronous Motors (PMSM), enabling a substantial reduction in the hardware sensors required for motor instrumentation and control.

# Supported Combinations {#EXAMPLES_AI_TORQUE_MEASUREMENT_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/torque_measurement/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/torque_measurement/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/torque_measurement/

\endcond

# Dataset and Model Details

## Dataset

The dataset contains sensor measurements collected from a PMSM running on a controlled test bench. The measurements were collected by the LEA department at Paderborn University at 2 Hz sampling rate.

| Column Name | Description |
|-------------|-------------|
| `i_d` | Current d-component (active) |
| `i_q` | Current q-component (reactive) |
| `u_d` | Voltage d-component (active) |
| `u_q` | Voltage q-component (reactive) |
| `motor_speed` | Motor speed |
| `ambient` | Ambient temperature |
| `torque` | Torque induced by current (target) |

The model uses a subset of easily measurable inputs: Ambient temperature, Coolant temperature, Voltage components (u_d, u_q), and Phase current magnitude to predict torque.

## Model Architecture

This regression model `REGR_10k` contains approximately 10,000 parameters with three convolutional layers (each with BatchNorm and ReLU) followed by two linear layers.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (10): channels for multiple parameters
- H (128): samples of timeseries data
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

This model produces a 1D output representing the continuous value of torque.

# Feature Extraction Configuration

For this torque measurement system, no FFT or frequency-domain feature extraction is applied. The model uses normalization to convert floating point input into fixed point for efficient inference.

Configuration in `user_input_config.h`:
- **SKIP_NORMALIZE**: Converts floating point input into fixed point input for the AI model

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will predict torque values and compare against expected outputs

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Torque Measurement Example Started ...
Feature extraction mismatches 0
All tests have passed!!
Golden vectors matched: 3 not matched: 0
\endcode
