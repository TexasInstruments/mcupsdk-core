# HVAC Indoor Temperature Forecasting {#EXAMPLES_AI_HVAC_INDOOR_TEMP_FORECAST}

[TOC]

# Introduction

Traditional HVAC controllers are reactive - they respond to temperature deviations only after they occur, leading to delayed corrections, unnecessary compressor cycling, and wasted energy. A smarter approach is to anticipate what the indoor temperature will be in the near future and act preemptively.

This project demonstrates on-device machine learning for HVAC control. A compact LSTM model running directly on the AM26x MCU predicts upcoming temperature trends from recent sensor readings, enabling smoother, more energy-efficient control decisions without relying on cloud connectivity.

# Supported Combinations {#EXAMPLES_AI_HVAC_INDOOR_TEMP_FORECAST_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/hvac_indoor_temp_forecast/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/hvac_indoor_temp_forecast/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/hvac_indoor_temp_forecast/

\endcond

# Dataset and Model Details

## Dataset

This example uses a synthetic HVAC dataset designed to emulate realistic indoor thermal dynamics. The dataset captures the interplay between compressor operation, outdoor conditions, and the resulting indoor temperature.

| Parameter | Value |
|-----------|-------|
| **Signals** | compressorFrequency, outdoorTemperature, indoorTemperature |
| **Input Variables** | 3 |
| **Prediction Target** | indoorTemperature (next timestep) |

The model learns to predict the indoor temperature at the next timestep given the past 5 readings of all three signals.

## Model Architecture

The model is `FCST_LSTM10`, a single-layer LSTM network with a hidden size of 10 and approximately 611 trainable parameters. LSTM gating mechanisms capture the temporal dependencies between compressor activity, outdoor conditions, and the resulting indoor temperature trajectory.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size, restricted to 1
- C (3): channels - compressorFrequency, outdoorTemperature, indoorTemperature
- H (5): number of historical timesteps (frame size) used for prediction
- W (1): width, restricted to 1 for timeseries applications

## Output

The model produces an output tensor of shape (1, 1, 1, 1), representing the predicted indoor temperature at the next timestep (forecast_horizon = 1). This is a continuous forecasting output that can be fed directly into a downstream control algorithm.

# Feature Extraction Configuration

A SimpleWindow transform segments the continuous stream into fixed-length windows of 5 consecutive timesteps across all 3 input channels. No FFT, binning, or log scaling is applied - the LSTM model learns directly from raw sensor values.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will forecast indoor temperature and compare against expected values

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
HVAC Indoor Temperature Forecasting Example Started ...
All tests have passed!!
Golden vectors matched: 3 not matched: 0
\endcode
