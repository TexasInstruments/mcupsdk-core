# Generic Timeseries Anomaly Detection {#EXAMPLES_AI_GENERIC_TIMESERIES_ANOMALYDETECTION}

[TOC]

# Introduction

Generic Timeseries Anomaly Detection is a hello world example for understanding usage of autoencoder-based AI models for anomaly detection on TI MCU. This project demonstrates implementation of an AI-based anomaly detection system on AM26x microcontrollers using a simple synthetic waveform dataset.

Unlike classification models, the autoencoder is trained only on normal data and learns to reconstruct normal patterns. At inference, the reconstruction error between the input and output is compared against a threshold - a high error indicates an anomaly.

# Supported Combinations {#EXAMPLES_AI_GENERIC_TIMESERIES_ANOMALYDETECTION_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_anomalydetection/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_anomalydetection/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_anomalydetection/

\endcond

# Dataset and Model Details

## Dataset

TI has created a synthetic waveform dataset based on a combined sinusoidal pattern. The normal signal follows y = 1.2 sin(2*pi*f*t) + 0.8 cos(2*pi*f*t) with a base frequency of 1.0 Hz.

| Parameter | Value |
|-----------|-------|
| **Sensor** | Signal readings |
| **Sampling Rate** | 100 Hz |
| **Channels** | 1 (signal magnitude) |
| **Samples per File** | 5,000 samples |
| **Total Files** | 92 files (60 Normal, 32 Anomaly) |

The dataset is divided into two classes:
- **Normal**: Standard sinusoidal pattern
- **Anomaly**: Includes four types of deviations - faster frequency, slower frequency, higher amplitude, and lower amplitude

**Important:** For anomaly detection, the model is trained only on normal data. Anomaly samples are used exclusively for testing.

## Model Architecture

This autoencoder model `AD_17k` contains approximately 17,000 parameters. The encoder compresses the input signal into a compact representation, and the decoder reconstructs the signal. During training, the model learns to reconstruct normal patterns with low error. When presented with an anomalous signal, the reconstruction error is significantly higher.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (1): channels which is 1 for signal
- H (100): samples of timeseries signal which is 100 in this example
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

Unlike classification models that output class probabilities, this model produces a reconstructed signal of the same shape as the input (1, 1, 100, 1). The reconstruction error (Mean Squared Error between input and output) is then compared against a threshold:

```
if reconstruction_error > threshold:
    -> ANOMALY
else:
    -> NORMAL
```

The threshold value is stored in `user_input_config.h` as `RECONSTRUCTION_ERROR_THRESHOLD`.

# Feature Extraction Configuration

For this anomaly detection task, the raw signal with downsampling and simple windowing approach produces the best results. The autoencoder learns to reconstruct the temporal shape of the raw waveform, so preserving the original signal structure is critical.

Key configuration parameters in `user_input_config.h`:
- **FE_FRAME_SIZE** (100): Number of samples per window
- **FE_NUM_FRAME_CONCAT** (1): No frame concatenation
- **FE_VARIABLES** (1): Single channel input
- **RECONSTRUCTION_ERROR_THRESHOLD**: Threshold for anomaly decision

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will run inference on test vectors, compute reconstruction error, and classify as Normal or Anomaly

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Generic Timeseries Anomaly Detection Example Started ...
Feature extraction mismatches 0
All tests have passed!!
Golden vectors matched: 2 not matched: 0
Reconstruction error is 0.008234, Threshold is 0.014500
The test sample is a Normal sample
\endcode
