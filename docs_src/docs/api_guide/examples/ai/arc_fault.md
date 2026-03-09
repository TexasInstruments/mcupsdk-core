# Arc Fault Detection {#EXAMPLES_AI_ARC_FAULT}

[TOC]

# Introduction

An arc fault is an unintended electrical discharge that occurs when electrical current flows through the air between conductors or from a conductor to ground. Arc faults generate intense heat that can ignite surrounding materials and cause electrical fires. Unlike short circuits that typically trigger circuit breakers immediately, arc faults may draw current below the trip threshold while still producing dangerous heat. In DC systems like solar installations, arc faults are particularly concerning as they can sustain more easily than in AC systems.

This project demonstrates implementation of an AI-based arc fault detection system on AM26x microcontrollers. It showcases how to deploy machine learning models for real-time electrical arc fault classification in embedded systems, helping prevent electrical hazards through early detection.

# Supported Combinations {#EXAMPLES_AI_ARC_FAULT_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/arc_fault/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/arc_fault/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/arc_fault/

\endcond

# Dataset and Model Details

## Dataset

TI has created a specialized arc fault dataset containing DC current measurements under various voltage and current configurations.

| Parameter | Value |
|-----------|-------|
| **Sensor** | Current Meter |
| **Channels** | 1 (DC Current) |
| **Samples per File** | 250,000 |
| **Total Files** | 24 files (12 files in each of 2 classes) |
| **Voltage Configurations** | 312 V, 318 V, 607 V |
| **Current Configurations** | 3 A, 8 A, 8.5 A |

**Output Classes:**
- Class 0: Normal Operation
- Class 1: Arc Fault

## Model Architecture

This lightweight classification model `CLS_1k_NPU` contains approximately 1,000 parameters and follows a streamlined architecture consisting of four convolutional layers (each enhanced with BatchNorm and ReLU activation functions) followed by a single linear layer.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (1): channels which is 1 for single DC current
- H (256): samples of timeseries DC current which is 256 in this example
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

This model produces a 1D output representing the two possible classes - normal operation or arc fault condition - providing a straightforward binary classification decision.

# Feature Extraction Configuration

The feature extraction pipeline is configured for frequency analysis of DC current signals:

- **FE_WIN**: Applies windowing to reduce spectral leakage
- **FE_FFT**: Performs Fast Fourier Transform (1024-point)
- **FE_NORMALIZE**: Scales FFT output by frame size
- **FE_BIN**: Groups frequencies into 256 bins
- **FE_LOG**: Applies logarithmic scaling
- **FE_CONCAT**: Combines outputs from multiple frames

The pipeline takes 1024-sample frames, computes FFT, and bins to 256 features. Arc faults exhibit distinctive frequency signatures that differ from normal operation.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will classify current readings as normal or arc fault

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Arc Fault Detection Example Started ...
Feature extraction mismatches 0
All tests have passed!!
Golden vectors matched: 2 not matched: 0
\endcode
