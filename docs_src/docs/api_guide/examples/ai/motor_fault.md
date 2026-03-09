# Motor Fault Detection {#EXAMPLES_AI_MOTOR_FAULT}

[TOC]

# Introduction

Vibration analysis is a key diagnostic tool for identifying bearing faults in machinery. When bearings deteriorate, their smooth motion becomes erratic, causing increased vibration levels. While basic time-domain measurements provide limited information, frequency analysis can pinpoint which specific component is failing, allowing engineers to anticipate critical failures and schedule repairs before catastrophic breakdowns occur.

This project demonstrates implementation of an AI-based motor fault detection system on AM26x microcontrollers. It showcases how to deploy machine learning models for real-time mechanical motor fault classification in embedded systems, helping prevent hazards through early detection of motor faults.

# Supported Combinations {#EXAMPLES_AI_MOTOR_FAULT_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/motor_fault/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/motor_fault/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/motor_fault/

\endcond

# Dataset and Model Details

## Dataset

TI has created a specialized motor fault dataset containing vibrational measurements. The dataset is divided into six classes representing different bearing conditions.

| Parameter | Value |
|-----------|-------|
| **Sensor** | 3-axis accelerometer (X, Y, Z axes) |
| **Sampling Rates** | 10 Hz, 20 Hz, 30 Hz, 40 Hz (variable) |
| **Channels** | 3 (Vibration X, Y, Z axes) |
| **Samples per File** | Variable: 24,000 - 144,000 samples |
| **Total Files** | 216 files (36 files in each of 6 classes) |

**Output Classes:**
- Class 0: Bearing Normal
- Class 1: Bearing Contaminated
- Class 2: Bearing Erosion
- Class 3: Bearing Flaking
- Class 4: Bearing No Lubrication
- Class 5: Bearing Localized Fault

## Model Architecture

This lightweight classification model contains approximately 1,000 parameters and follows a streamlined architecture consisting of four convolutional layers (each enhanced with BatchNorm and ReLU activation functions) followed by a single linear layer.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (3): channels which is 3 for 3-axis vibration data
- H (128): samples of timeseries vibrations which is 128 in this example
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

This model produces a 1D output representing the six possible classes. The position of the highest value in this output indicates the classification result.

# Feature Extraction Configuration

Feature extraction transforms raw vibration data into meaningful inputs for the AI model. The feature extraction pipeline uses FFT to identify frequencies, followed by binning and logarithmic scaling.

Configuration flags in `user_input_config.h`:
- **FE_FFT**: Performs Fast Fourier Transform, converting time-domain signal into frequency components
- **FE_DC_REM**: Removes the DC component from Fourier transform
- **FE_BIN**: Groups frequencies into bins (size 8), reducing dimensionality
- **FE_BIN_NORMALIZE**: Normalizes the bin values
- **FE_LOG**: Applies logarithmic scaling to compress dynamic range
- **FE_CONCAT**: Combines outputs from 8 frames for temporal context

The pipeline takes a 256-sample frame, computes FFT (129 outputs), removes DC (128), bins to 16 features, and concatenates 8 frames for 128 total features per channel.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will run inference on test vectors and classify bearing condition

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Motor Fault Detection Example Started ...
Feature extraction mismatches 0
All tests have passed!!
Golden vectors matched: 6 not matched: 0
\endcode
