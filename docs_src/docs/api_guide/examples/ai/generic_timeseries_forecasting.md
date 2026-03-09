# Generic Timeseries Forecasting {#EXAMPLES_AI_GENERIC_TIMESERIES_FORECASTING}

[TOC]

# Introduction

Time series forecasting is the process of predicting future values based on historical observations of sequential data. It is widely used across industrial, appliance, and automotive domains from estimating energy consumption patterns to forecasting temperature variations in cooling systems and predicting motor operating conditions.

This project serves as an introduction to Timeseries Forecasting on AM26x microcontrollers. It uses a simple simulated thermostat dataset, demonstrating the complete end-to-end development chain: from training and exporting a model using TinyML ModelZoo to deploying and running inference on-device.

# Supported Combinations {#EXAMPLES_AI_GENERIC_TIMESERIES_FORECASTING_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_forecasting/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_forecasting/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_forecasting/

\endcond

# Dataset and Model Details

## Dataset

This example uses a simulated thermostat dataset that models room temperature controlled by an ON/OFF heater with hysteresis. The heater turns ON when temperature drops below 20C and turns OFF when it rises above 24C.

| Parameter | Value |
|-----------|-------|
| **Sensor** | Temperature Sensor (Simulated) |
| **Channels** | 1 (Temperature in C) |
| **Samples per File** | 1,000 timesteps |
| **Total Files** | 15 (10 train, 2 validation, 3 test) |
| **Temperature Range** | ~19.7C to ~24.1C |

## Model Architecture

The model used is `FCST_LSTM10`, a lightweight single-layer LSTM (Long Short-Term Memory) network with a hidden size of 10 and approximately 542 trainable parameters. LSTM networks are well-suited for time series forecasting because their gating mechanisms can learn long-term dependencies in sequential data.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size, restricted to 1
- C (1): channels, 1 for single temperature variable
- H (32): number of historical timesteps (frame size) used for prediction
- W (1): width, restricted to 1 for timeseries applications

## Output

The model produces an output tensor of shape (1, 2), representing the predicted temperature values for the next 2 timesteps (forecast horizon = 2). Unlike classification models that output discrete class labels, this forecasting model outputs continuous values representing future temperature.

# Feature Extraction Configuration

For this forecasting task, a SimpleWindow transform is used which segments the continuous temperature stream into fixed-length windows of 32 consecutive timesteps with a stride of 10%. Each window becomes one input sample, with the next 2 timesteps serving as the forecast target.

No FFT, binning, or log scaling is applied - the LSTM model learns directly from the raw temperature values.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will run inference on test vectors and display forecasted values

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Generic Timeseries Forecasting Example Started ...
All tests have passed!!
Golden vectors matched: 3 not matched: 0
\endcode
