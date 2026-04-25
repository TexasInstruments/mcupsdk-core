# EDMA RTI SRAM Scrub Demo for AM263Px-CC

## Overview
This example demonstrates how to use EDMA (Enhanced Direct Memory Access) and RTI (Real-Time Interrupt) 
to implement a SRAM scrubbing mechanism for detecting and mitigating Single-Bit Errors (SBE) in AM263Px
microcontrollers.

## Features
- Scrubs (reads) the first 3 SRAM banks (1.5 MB total) using EDMA transfers
- EDMA transfers are triggered periodically by RTI every 50 μs (configurable in SysConfig)
- Each EDMA transfer processes chunks of 4 x 64-bit beats (32 bytes)
- Detects and corrects single-bit ECC errors in SRAM
- Demonstrates how to handle SEC (Single Error Correction) interrupts
- Memory configuration is managed through SysConfig

## Hardware Requirements
- AM263Px-LP LaunchPad development kit
- USB cable for power and debug interface

## Software Requirements
- Code Composer Studio (CCS) v12.x or later
- TI ARM Compiler v20.x or later
- SysConfig v1.x or later

## Project Structure
- `main.c` - Entry point and system initialization
- `edma_rti_sram_scrub.c/.h` - Main application logic and interface definitions
- `ecc_esm_config.c` - ECC and ESM (Error Signaling Module) configuration
- `edma_config.c` - EDMA channel and trigger configuration
- `eccagg.c` - ECC Aggregator register access functions
- `esm.c` - Error Signaling Module initialization

## Memory Usage
- Instruction (text) and constant data are placed in the first 3 SRAM banks
- The rest of the data is placed in the 4th SRAM bank
- EDMA read buffer is located in TCM (Tightly Coupled Memory)
- Memory configuration is managed through SysConfig

## How It Works
1. The application configures the EDMA channels and RTI for periodic triggers
2. RTI triggers the EDMA to read chunks of SRAM every 50 μs
3. EDMA reads the SRAM data into a separate buffer in TCM
4. If a SEC (Single Error Correction) interrupt occurs:
   - The ECC fault address is compared with the current EDMA read chunk address range
   - If the fault address is within the current chunk, the EDMA buffer is compared with SRAM data
   - Data mismatch indicates the EDMA read contained an error that was corrected (scrub successful)
   - No data mismatch suggests the CPU may have read wrong data, requiring a device reset

## Building and Running the Example
1. Import the project into CCS
2. Build the project
3. Connect the AM263Px-CC Control Card to your PC
4. Load the program onto the target
5. Run the application

## Expected Output
When the application runs successfully, it outputs:
```
EDMA RTI triggered SRAM Scrub Test ...
Starting RTI for periodic trigger for EDMA ...
```
The application will continuously scrub SRAM in the background.

Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/