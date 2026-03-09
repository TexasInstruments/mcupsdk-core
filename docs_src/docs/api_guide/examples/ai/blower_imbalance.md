# Blower Imbalance Detection {#EXAMPLES_AI_BLOWER_IMBALANCE}

[TOC]

# Introduction

Blower Imbalance detection is a classification problem to understand the correct working of fan blowers. The problem is to identify whether the fans of a running motor have any imbalance in rotation or not. This becomes important in real-world scenarios such as dust accumulation on fan blades, cooling systems, elevators, and anywhere there is a running motor. This example uses current readings instead of additional sensors to do the classification.

This project demonstrates implementation of an AI-based blower imbalance detection system on AM26x microcontrollers, helping prevent hazards through early detection of faults.

# Supported Combinations {#EXAMPLES_AI_BLOWER_IMBALANCE_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/blower_imbalance/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/blower_imbalance/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/blower_imbalance/

\endcond

# Dataset and Model Details

## Dataset

TI has created a specialized blower imbalance dataset containing AC current measurements (3-phase).

| Parameter | Value |
|-----------|-------|
| **Sensor** | Current Meter |
| **Channels** | 3 (AC Current - 3 phases) |
| **Sampling Rates** | 20 Hz, 30 Hz, 50 Hz, 60 Hz (variable) |
| **Samples per File** | Variable: 36,000 - 90,000 samples |
| **Total Files** | 123 files (69 normal, 54 fault) |

**Output Classes:**
- Class 0: Normal Operation
- Class 1: Blower Imbalance

## Model Architecture

This lightweight classification model `CLS_1k_NPU` contains approximately 1,000 parameters and follows a streamlined architecture consisting of four convolutional layers (each enhanced with BatchNorm and ReLU activation functions) followed by a single linear layer.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (3): channels which is 3 for 3-phase AC current
- H (128): samples of timeseries AC current which is 128 in this example
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

This model produces a 1D output representing the two possible classes - normal operation or blower imbalance condition.

# Feature Extraction Configuration

The feature extraction pipeline uses FFT-based frequency analysis:

- **FE_FFT**: Performs Fast Fourier Transform (256-point)
- **FE_DC_REM**: Removes DC component
- **FE_BIN**: Groups frequencies into 16 bins (bin size 8)
- **FE_BIN_NORMALIZE**: Normalizes bin values
- **FE_LOG**: Applies logarithmic scaling
- **FE_CONCAT**: Combines 8 frames for temporal context

The pipeline processes 256-sample frames per channel, producing 16 features per frame, concatenated across 8 frames for 128 total features per channel.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will classify current readings as normal or imbalanced

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Blower Imbalance Detection Example Started ...
Feature extraction mismatches 0
All tests have passed!!
Golden vectors matched: 2 not matched: 0
\endcode
