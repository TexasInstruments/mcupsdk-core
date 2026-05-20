# SDL ECC PRU Scrubber R5F Host {#EXAMPLES_SDL_ECC_PRU_SCRUBBER_R5F0}

[TOC]

# Introduction

This example is the R5F host application component of the Active PRU Scrubber solution
for AM263Px devices. It runs on the R5FSS0-0 core with FreeRTOS and works in conjunction
with the PRU firmware project (\ref EXAMPLES_SDL_ECC_PRU_SCRUBBER_PRU0).

The R5F host application:
- Loads and starts the PRU firmware on ICSS_M0_PRU0
- Initializes the shared DMEM communication structure with scrub parameters (start address,
  number of iterations)
- Triggers memory scrubbing by setting `hostTriggerBit = 1`
- Polls for completion by monitoring `hostTriggerBit` returning to 0
- Handles critical ECC error notifications from the PRU via `pruTriggerBit`

The shared DMEM communication interface between R5F and PRU is:

```c
typedef struct {
    uint32_t             startAddress;     /* Memory address to start scrubbing   */
    uint16_t             numOfIterations;  /* Number of iterations to perform     */
    volatile uint8_t     hostTriggerBit;   /* Set by R5F to trigger PRU           */
    volatile uint8_t     pruTriggerBit;    /* Set by PRU to notify R5F of error   */
} dmemParameters;
```

The example demonstrates scrubbing a 256 KB region starting at address `0x70000000`:

```c
status = scrub_mem(0x70000000, 0x40000);
```

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | PRU scrubber completes all iterations with no ECC errors; `hostTriggerBit` returns to 0.
 UC-2     | PRU detects a critical SEC error (another master caused corruption); `pruTriggerBit` is set and R5F performs error handling.


# Supported Combinations {#EXAMPLES_SDL_ECC_PRU_SCRUBBER_R5F0_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_pru_scrubber_r5f0/

\endcond

# Steps to Run the Example

\note The PRU firmware project (\ref EXAMPLES_SDL_ECC_PRU_SCRUBBER_PRU0) must be built
first. Ensure the generated binary is embedded in `pru0_load_bin.h` before building
this R5F host project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE).
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE.

# See Also

\ref SDL_ECC_PAGE

# Sample Output

Shown below is a sample output when the application is run:

\cond SOC_AM263PX
\code
PRU Scrubber: Initializing R5F host application ...
Loading PRU firmware onto ICSS_M0_PRU0 ...
PRU firmware loaded successfully.
Starting memory scrub: startAddress=0x70000000, iterations=0x400
Scrub in progress ...
Scrub complete. hostTriggerBit cleared by PRU.
No critical ECC errors detected.

\endcode
\endcond

The R5F application monitors `pruTriggerBit` continuously. If the PRU sets it to indicate
a critical SEC error (error caused by another master rather than the PRU read itself),
the R5F host can perform appropriate error handling such as a memory reset or system restart.
