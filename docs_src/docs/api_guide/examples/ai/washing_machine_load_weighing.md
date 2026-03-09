# Washing Machine Load Weighing {#EXAMPLES_AI_WASHING_MACHINE_LOAD_WEIGHING}

[TOC]

# Introduction

Prediction of weight of clothes in a washing machine eliminates the need for a physical weight sensor, which is typically required for automatic water level adjustment. This reduces manufacturing costs and provides a reliable sensor-less solution.

This project utilizes machine learning models that take into account various features such as voltage, current, and speed of the washing machine motor to predict the weight of the clothes. This ensures that the water level can be adjusted accurately without relying on a physical weight sensor, which can be prone to mechanical failure.

# Supported Combinations {#EXAMPLES_AI_WASHING_MACHINE_LOAD_WEIGHING_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/washing_machine_load_weighing/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/washing_machine_load_weighing/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/washing_machine_load_weighing/

\endcond

# Dataset and Model Details

## Dataset

The dataset consists of 6 current, voltage, and speed-based features obtained from TI Lab. The dataset has 100g precision from 0g to 900g.

| Variable | Description |
|----------|-------------|
| Vd | Voltage component along d-axis |
| Id | Current component along d-axis |
| Vq | Voltage component along q-axis |
| Iq | Current component along q-axis |
| Iqref | Reference Current |
| Speed | Speed of the washing machine motor |

## Model Architecture

The model is a CNN-based architecture with approximately 13,000 parameters consisting of 3 convolution layers and 2 fully connected layers.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (6): channels which is 6 for the features
- H (512): samples of timeseries signals which is 512 in this example
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

This model produces a 1D output representing the weight of the clothes in the washing machine (in grams).

# Feature Extraction Configuration

For this regression task, a SimpleWindow transformation is used, which makes use of the previous 512 datapoints to predict the current instance of the target variable. No FFT or frequency-domain feature extraction is applied.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will predict load weight and compare against expected values

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Washing Machine Load Weighing Example Started ...
All tests have passed!!
Golden vectors matched: 3 not matched: 0
\endcode
