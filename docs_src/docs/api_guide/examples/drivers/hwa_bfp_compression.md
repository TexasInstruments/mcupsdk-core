# HWA BFP COMPRESSION {#EXAMPLES_DRIVERS_HWA_BFP_COMPRESSION}

[TOC]

# Introduction

This example performs HWA based BFP Compression and Decompression operation
along the chirp dimension on complex 1D FFT input data.

The input to the compression engine is a complex range-FFT data. This data is
arranged as [Chirps ] [Antennas ] [Samples ] dimensions.
The HWA PaRAM sets are initialized with proper configuration for Compression and
Decompression operation.
HWA is triggered to perform the Compression operation and Decompression operation.

The HWA decompressed output is then compared to the ideal decompressed data.

The example also demonstrates how BFP mantissa bitwidth can be configured depending
on the desired compression ratio.

# Supported Combinations {#EXAMPLES_DRIVERS_HWA_BFP_COMPRESSION_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/hwa/hwa_bfp_compression

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
------- HWA BFP compression/decompression tests -------

HWA Instance has been opened successfully


................................
Running test 0
NumSamples: 32, NumChirps: 16
CompRatio: 0.33, SamplesPerBlock: 8
Performing Asymmetric Compression
Achieved Compression ratio: 0.375, BFPMantissaBW: 6
HWA cofiguration done successfully
Compression and Decompression Done. Comparing Results..
Debug: Compression/decompression BFP output generated by HWA found correct


................................
Running test 1
NumSamples: 32, NumChirps: 16
CompRatio: 0.50, SamplesPerBlock: 8
Performing Asymmetric Compression
Achieved Compression ratio: 0.500, BFPMantissaBW: 8
HWA cofiguration done successfully
Compression and Decompression Done. Comparing Results..
Debug: Compression/decompression BFP output generated by HWA found correct


................................
Running test 2
NumSamples: 32, NumChirps: 16
CompRatio: 0.75, SamplesPerBlock: 8
Performing Asymmetric Compression
Achieved Compression ratio: 0.750, BFPMantissaBW: 12
HWA cofiguration done successfully
Compression and Decompression Done. Comparing Results..
Debug: Compression/decompression BFP output generated by HWA found correct


HWA BFP Compression/Decompression Test Completed!!
\endcode
