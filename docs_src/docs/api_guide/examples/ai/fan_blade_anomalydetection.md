# Fan Blade Anomaly Detection {#EXAMPLES_AI_FAN_BLADE_ANOMALYDETECTION}

[TOC]

# Introduction

Fan blade faults such as imbalance, damage, and obstruction can lead to reduced efficiency, increased energy consumption, and potential system failure. Early detection of these anomalies through vibration analysis enables predictive maintenance, preventing costly downtime and extending equipment lifespan.

This project demonstrates implementation of an AI-based fan blade anomaly detection system on AM26x microcontrollers. It uses an autoencoder model trained only on normal vibration data to detect deviations that indicate potential faults - including fault types never seen during training.

# Supported Combinations {#EXAMPLES_AI_FAN_BLADE_ANOMALYDETECTION_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/fan_blade_anomalydetection/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/fan_blade_anomalydetection/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/fan_blade_anomalydetection/

\endcond

# Dataset and Model Details

## Dataset

TI has created a fan blade vibration dataset collected using a 3-axis ADXL355 accelerometer. The dataset captures both normal operation and various fault conditions.

| Parameter | Value |
|-----------|-------|
| **Sensor** | ADXL355 3-axis Accelerometer |
| **Sampling Rate** | 4 kHz |
| **Channels** | 3 (Vibration X, Y, Z axes) |
| **Samples per File** | ~20,000 samples (~5 seconds of data) |
| **Total Files** | 287 files (100 Normal, 187 Anomaly) |

**Anomaly Types:**
- Blade imbalance
- Blade damage
- Blade obstruction

**Important:** For anomaly detection, the model is trained only on normal data. All anomaly samples are used exclusively for testing.

## Model Architecture

This autoencoder model `AD_17k` contains approximately 17,000 parameters. The encoder compresses the input vibration features into a compact representation, and the decoder reconstructs the features. When presented with anomalous vibration data, the reconstruction error is significantly higher than for normal data.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (3): channels which is 3 for vibration X, Y, Z axes
- H (64): features per channel (16 frequency bins x 4 concatenated frames)
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

The model produces a reconstructed feature vector of the same shape as the input (1, 3, 64, 1). The reconstruction error (MSE between input and output) is compared against a threshold:

```
if reconstruction_error > threshold:
    -> ANOMALY
else:
    -> NORMAL
```

# Feature Extraction Configuration

The feature extraction pipeline uses FFT-based frequency analysis on 3-axis vibration data:

- **FE_FFT**: Performs Fast Fourier Transform (256-point)
- **FE_DC_REM**: Removes DC component
- **FE_BIN**: Groups frequencies into 16 bins
- **FE_BIN_NORMALIZE**: Normalizes bin values
- **FE_LOG**: Applies logarithmic scaling
- **FE_CONCAT**: Combines 4 frames for temporal context

The pipeline produces 16 features per frame x 4 frames = 64 features per channel. With 3 channels (X, Y, Z), the total input is 192 features.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application computes reconstruction error and classifies as Normal or Anomaly

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Fan Blade Anomaly Detection Example Started ...
Feature extraction mismatches 0
All tests have passed!!
Golden vectors matched: 2 not matched: 0
Reconstruction error is 0.002345, Threshold is 0.010000
The test sample is a Normal sample
\endcode
