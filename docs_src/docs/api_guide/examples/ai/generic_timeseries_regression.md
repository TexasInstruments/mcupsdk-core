# Generic Timeseries Regression {#EXAMPLES_AI_GENERIC_TIMESERIES_REGRESSION}

[TOC]

# Introduction

Generic Timeseries Regression is a hello world example for understanding usage of regression AI models on TI MCU. This example uses a simple synthetic regression dataset where the target variable y = 1.2 sin(x) + 3.2 cos(x), with x randomly generated in the range [0, 3]. This serves as a fundamental example for deploying regression models on AM26x microcontrollers.

# Supported Combinations {#EXAMPLES_AI_GENERIC_TIMESERIES_REGRESSION_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_regression/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_regression/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_regression/

\endcond

# Dataset and Model Details

## Dataset

A synthetic regression dataset where:
- x is randomly generated in the range [0, 3]
- Target variable y = 1.2 sin(x) + 3.2 cos(x)

| Column Name | Description |
|-------------|-------------|
| x | Randomly generated input in range [0,3] |
| y | Target: y = 1.2 sin(x) + 3.2 cos(x) |

## Model Architecture

This lightweight regression model contains approximately 1,800 parameters with one input batch normalization layer, 4 convolution layers, and 2 fully connected layers.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (1): channels which is 1 for signal
- H (10): samples of timeseries signals which is 10 in this example
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

This model produces a 1D output representing the predicted values of target variable y.

# Feature Extraction Configuration

For this regression task, a SimpleWindow transformation is used, which makes use of the previous frame_size datapoints for prediction of the current target value. A frame size of 10 is used in this example.

No FFT or frequency-domain feature extraction is applied - the model learns directly from raw signal values.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will run inference on test vectors and compare against expected outputs

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Generic Timeseries Regression Example Started ...
All tests have passed!!
Golden vectors matched: 3 not matched: 0
\endcode
