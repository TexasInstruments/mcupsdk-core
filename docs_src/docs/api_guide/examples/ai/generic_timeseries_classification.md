# Generic Timeseries Classification {#EXAMPLES_AI_GENERIC_TIMESERIES_CLASSIFICATION}

[TOC]

# Introduction

Generic Timeseries Classification is a hello world example for understanding usage of AI models on TI MCU. This project demonstrates implementation of an AI-based waveform detection system on AM26x microcontrollers. It showcases how to deploy machine learning models for real-time waveform classification in embedded systems, serving as a hello world example. TI provides a complete development ecosystem with toolchains and SDKs that significantly streamline all stages of Edge AI solution development.

The example uses a simple classification dataset with three waveform classes: sine, sawtooth, and square waveforms.

# Supported Combinations {#EXAMPLES_AI_GENERIC_TIMESERIES_CLASSIFICATION_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_classification/

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_classification/

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos      | 
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/ai/generic_timeseries_classification/

\endcond

# Dataset and Model Details

## Dataset

TI has created a specialized waveform dataset containing signal measurements. The dataset is divided into three classes: sine waveform, sawtooth waveform and square waveform.

| Parameter | Value |
|-----------|-------|
| **Sensor** | Signal readings |
| **Sampling Rate** | 40 Hz |
| **Channels** | 1 (signal magnitude) |
| **Samples per File** | 25,600 samples |
| **Total Files** | 75 files (~25 files in each of 3 classes) |

## Model Architecture

This lightweight classification model `CLS_1k_NPU` contains approximately 1,000 parameters and follows a streamlined architecture consisting of four convolutional layers (each enhanced with BatchNorm and ReLU activation functions) followed by a single linear layer.

## Input Features

The model takes 4D input (N,C,H,W):
- N (1): batch size which is restricted to 1
- C (1): channels which is 1 for signal
- H (256): samples of timeseries signals which is 256 in this example
- W (1): width of samples is restricted to 1 for timeseries applications

## Output

This model produces a 1D output representing the three possible classes. The position of the highest value in this output indicates the classification result:
- Class 0: Sine waveform
- Class 1: Sawtooth waveform
- Class 2: Square waveform

# Feature Extraction Configuration

Feature extraction transforms raw data into meaningful inputs for the AI model. For this waveform detection system, applying FFT to identify frequencies, followed by binning and logarithmic scaling, produces superior results.

The feature extraction pipeline is configured in `user_input_config.h`:

- **FE_FFT**: Performs Fast Fourier Transform on the raw frame, calculating magnitude values from complex outputs
- **FE_DC_REM**: Removes the DC component from Fourier transform
- **FE_BIN**: Groups frequencies into bins, reducing dimensionality while preserving frequency distribution patterns
- **FE_LOG**: Applies logarithmic scaling to the binned frequency data
- **FE_CONCAT**: Combines scaled outputs from multiple data frames for temporal context

Key parameters:
- `FE_FRAME_SIZE`: 1024 samples per FFT frame
- `FE_FEATURE_SIZE_PER_FRAME`: 64 features per frame
- `FE_NUM_FRAME_CONCAT`: 8 frames concatenated
- `FE_FFT_BIN_SIZE`: 8 (bin width)
- `FE_NN_OUT_SIZE`: 3 (number of output classes)

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- The application will run inference on test vectors and display results in the console

# See Also

\ref EXAMPLES_AI

# Sample Output

\code
Generic Timeseries Classification Example Started ...
Feature extraction mismatches 0
All tests have passed!!
Golden vectors matched: 3 not matched: 0
\endcode
