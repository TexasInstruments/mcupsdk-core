# Active PRU Scrubber Solution for AM263Px

## Overview
This readme documents a comprehensive memory scrubbing solution for AM263Px devices, consisting of two projects:

1. **R5F Host Application** (`pru_scrubber_am263px-cc_active_r5fss0-0_freertos_ti-arm-clang`)
   - Runs on the R5FSS0-0 core with FreeRTOS
   - Loads and controls the PRU firmware
   - Processes PRU notifications for error management

2. **PRU Firmware** (`pru_scrubber_am263px-cc_r5_active_icss_m0_pru0_fw_ti-pru-cgt`)
   - Runs on ICSS_M0_PRU0
   - Performs active memory scrubbing
   - Detects and analyzes ECC errors
   - Communicates findings back to the R5F core

This "active" scrubber implementation detects ECC errors and analyzes them to determine their source, 
enabling system responses based on error conditions.

## Features
- Performs continuous memory reads (scrubbing) over configurable memory ranges
- Detects Single Error Correction (SEC) events during memory scrubbing
- Analyzes error patterns to determine if errors were caused by PRU reads or by other masters
- Uses bi-directional communication between R5F and PRU
- Can trigger specific error handling in R5F based on error analysis
- Supports scrubbing memory ranges up to ~7MB
- Efficient implementation with 4-beat (16-byte) burst reads

## Communication Interface
The R5F and PRU communicate through a shared memory structure in PRU DMEM:

```c
typedef struct {
    uint32_t             startAddress;     // Memory address to start scrubbing
    uint16_t             numOfIterations;  // Number of iterations to perform
    volatile uint8_t     hostTriggerBit;   // Set by R5F to trigger PRU (1=start, 0=idle)
    volatile uint8_t     pruTriggerBit;    // Set by PRU to notify R5F of error analysis
} dmemParameters;
```

## How the Active Scrubber Works

### Normal Operation Flow:
1. R5F loads the PRU firmware and initializes the DMEM parameters
2. R5F sets the starting address, number of iterations, and sets hostTriggerBit=1
3. PRU begins reading memory in 4-beat bursts (16 bytes at a time)
4. After completing all iterations, PRU clears the hostTriggerBit and waits
5. R5F detects completion when hostTriggerBit returns to 0

### Error Detection and Analysis:
1. During scrubbing, PRU monitors ECC error status registers
2. When an SEC error occurs, PRU reads the error address from ECC_ERR_STAT2
3. PRU compares the error address with the current read address
4. PRU analyzes adjacent beat values to determine error source:
   - If adjacent beats have same data: PRU likely caused the SEC (benign)
   - If adjacent beats differ: Another master may have corrupted memory (critical)
5. For critical errors, PRU sets pruTriggerBit=1 to notify R5F
6. R5F can implement appropriate error handling (e.g., memory reset, system restart)

## Project Structure

### R5F Host Application:
- `main.c` - Entry point and FreeRTOS initialization
- `empty_example.c` - Main application logic for PRU control and error handling
- `pru0_load_bin.h` - PRU firmware binary in array format for loading

### PRU Firmware:
- `main.asm` - PRU assembly code implementing scrubbing and error detection logic
- `icss_constant_defines.inc` - PRU constant table definitions
- `linker.cmd` - Custom linker command file for PRU firmware

## Building and Running

### Build Process:
1. First build the PRU firmware project (`pru_scrubber_am263px-cc_r5_active_icss_m0_pru0_fw_ti-pru-cgt`)
2. Update the firmware binary in the R5F project's `pru0_load_bin.h` file (if not automated)
3. Build the R5F host application (`pru_scrubber_am263px-cc_active_r5fss0-0_freertos_ti-arm-clang`)
4. Connect the AM263Px-CC board to your computer
5. Load and run the R5F application, which will load the PRU firmware

### Example Usage:
The example demonstrates scrubbing a 256KB (0x40000) memory region starting at address 0x70000000:
```c
status = scrub_mem(0x70000000, 0x40000);
```

## Technical Notes

### Memory Scrubbing Considerations:
- The solution uses a read-only approach to avoid modifying memory content
- ECC errors are detected and analyzed to determine their source
- Maximum scrubbing range is ~7MB due to the 16-bit iteration counter limit
- Each scrub operation reads memory in 16-byte bursts

### Error Handling Strategy:
- PRU analyzes error patterns to differentiate between:
  - Benign errors: PRU reads triggered error correction (no action needed)
  - Critical errors: Another master wrote corrupted data (requires intervention)
- For critical errors, R5F can implement appropriate error handling strategies

Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/