# PMSM Rotor Temperature Forecasting {#EXAMPLES_AI_FORECASTING_PMSM_ROTOR_TEMP}

[TOC]

# Introduction

Permanent Magnet Synchronous Motors (PMSMs) are widely used in electric vehicles, industrial automation, and wind turbines. However, they carry a critical vulnerability: the permanent magnets can suffer irreversible demagnetization if rotor temperatures exceed safe limits (typically above 150C). Placing temperature sensors directly on the rotor is expensive and mechanically fragile.

This project demonstrates an elegant solution using on-device machine learning. A lightweight LSTM model running on the AM26x MCU estimates rotor temperature from signals that are already available - phase currents, voltage components, ambient temperature, and coolant temperature. This enables preemptive thermal protection actions before damage occurs.

# Supported Combinations {#EXAMPLES_AI_FORECASTING_PMSM_ROTOR_TEMP_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/forecasting_pmsm_rotor_temp/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/forecasting_pmsm_rotor_temp/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/forecasting_pmsm_rotor_temp/

\endcond

# Dataset and Model Details

## Dataset

This example uses a dataset derived from real sensor measurements collected from a PMSM running on a controlled test bench. The original data was recorded by the LEA department at Paderborn University at 2 Hz sampling rate.

| Signal | Description |
|--------|-------------|
| `i_a` | Phase current magnitude |
| `u_d` | Voltage d-component (active) |
| `u_q` | Voltage q-component (reactive) |
| `ambient` | Ambient temperature |
| `coolant` | Coolant temperature |
| `pm` | Permanent magnet surface temperature (target) |

The model predicts the permanent magnet surface temperature at the next timestep given the past 3 readings of all six signals.

## Model Architecture

The model is `FCST_LSTM8` - a single-layer LSTM network with a hidden size of 8 and approximately 393 trainable parameters. The LSTM's gating mechanisms learn how current loading, voltage conditions, and cooling state interact over time to determine rotor temperature.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size, restricted to 1
- C (6): channels - i_a, u_d, u_q, ambient, coolant, pm
- H (3): number of historical timesteps (frame size) used for prediction
- W (1): width, restricted to 1 for timeseries applications

## Output

The model produces an output tensor of shape (1, 1), representing the predicted permanent magnet surface temperature at the next timestep. This continuous forecasting output can be fed directly into thermal protection or derating logic.

# Feature Extraction Configuration

A SimpleWindow transform segments the continuous stream into fixed-length windows of 3 consecutive timesteps across all 6 input channels. No FFT, binning, or log scaling is applied - the LSTM model learns directly from raw sensor values.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will forecast rotor temperature and compare against expected values

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
PMSM Rotor Temperature Forecasting Example Started ...
All tests have passed!!
Golden vectors matched: 3 not matched: 0
\endcode
