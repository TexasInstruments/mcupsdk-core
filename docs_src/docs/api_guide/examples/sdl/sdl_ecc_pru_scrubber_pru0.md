# SDL ECC PRU Scrubber PRU Firmware {#EXAMPLES_SDL_ECC_PRU_SCRUBBER_PRU0}

[TOC]

# Introduction

This example is the PRU firmware component of the Active PRU Scrubber solution
for AM263Px devices. It runs on ICSS_M0_PRU0 and works in conjunction with the
R5F host application project (\ref EXAMPLES_SDL_ECC_PRU_SCRUBBER_R5F0).

The PRU firmware:
- Reads a configurable memory range in 4-beat bursts (16 bytes at a time) to perform
  active memory scrubbing
- Monitors ECC error status registers for Single Error Correction (SEC) events
- Analyzes error patterns to determine the source of detected errors:
  - **Benign**: Adjacent burst beats contain the same data — PRU read triggered the
    correction (no action required)
  - **Critical**: Adjacent burst beats differ — another master wrote corrupted data
    (R5F intervention required)
- Notifies the R5F host of critical errors by setting `pruTriggerBit = 1` in the
  shared DMEM structure
- Clears `hostTriggerBit` on completion to signal the R5F host

The PRU firmware is written in PRU assembly (`main.asm`) and supports scrubbing
memory ranges up to approximately 7 MB (limited by the 16-bit iteration counter).

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | PRU completes all scrub iterations without detecting any ECC errors; clears `hostTriggerBit` to signal completion.
 UC-2     | SEC error detected during scrubbing; PRU analyzes adjacent beat values — if another master caused the error, PRU sets `pruTriggerBit = 1` to notify the R5F host.


# Supported Combinations {#EXAMPLES_SDL_ECC_PRU_SCRUBBER_PRU0_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | icss_m0_pru0
 Toolchain      | ti-pru-cgt
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_pru_scrubber_pru0/

\endcond

# Steps to Run the Example

\note This PRU firmware project must be built **before** the R5F host project
(\ref EXAMPLES_SDL_ECC_PRU_SCRUBBER_R5F0). After building, the generated firmware
binary must be embedded in `pru0_load_bin.h` in the R5F host project. The PRU firmware
is loaded and started at runtime by the R5F host application — it is not loaded
directly via CCS.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE).
- After building, proceed to build and run the R5F host application
  (\ref EXAMPLES_SDL_ECC_PRU_SCRUBBER_R5F0), which loads this firmware onto ICSS_M0_PRU0.

# See Also

\ref SDL_ECC_PAGE

# Sample Output

The PRU firmware does not produce direct UART output. Scrubbing activity and error
results are reported by the R5F host application (see \ref EXAMPLES_SDL_ECC_PRU_SCRUBBER_R5F0).

\cond SOC_AM263PX
\code
[R5F Host output when PRU firmware is running]

PRU Scrubber: Initializing R5F host application ...
Loading PRU firmware onto ICSS_M0_PRU0 ...
PRU firmware loaded successfully.
Starting memory scrub: startAddress=0x70000000, iterations=0x400
Scrub in progress ...
Scrub complete. hostTriggerBit cleared by PRU.
No critical ECC errors detected.

\endcode
\endcond

When a critical ECC error is detected by the PRU, the R5F host receives notification
via `pruTriggerBit` and can implement appropriate error handling (memory reset,
system restart, or safety state transition).
