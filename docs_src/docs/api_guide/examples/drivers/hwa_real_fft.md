# HWA REAL FFT {#EXAMPLES_DRIVERS_HWA_REAL_FFT}

[TOC]

# Introduction

This example performs HWA based FFT and IFFT operation on real input data.

The input to the FFT is a sinusoidal real (float) data. This is converted to
24-bit signed fixed point data which the HWA will operate on.
The HWA PaRAM sets are initialized with proper configuration for FFT and
IFFT operation.
The HWA is triggered to perform the FFT operation and the frequency domain
complex output is converted back to float just to show that user can tap
this data for further processing.
In the IFFT part, the frequency domain complex data is converted back to
fixed point and provided to HWA to perform the IFFT operation. Once the
conversion is completed, the data is converted back to float.
The output is then compared to the input data and SNR is calculated and
printed on the console.

The example also demonstrates multi-pass dynamic scaling FFT operation which
improves the SNR ratio compared to the single-pass conservative scaling FFT
operation.

# Supported Combinations {#EXAMPLES_DRIVERS_HWA_REAL_FFT_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/hwa/hwa_real_fft

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_HWA_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code

\endcode
