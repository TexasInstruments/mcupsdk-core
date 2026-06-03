# Active PRU Scrubber Solution for AM263x and AM263Px

## Overview
This PRU scrubbing code is a workaround for the i2499 errata issue. It implements Option 3 (SRAM Scrubbing via ICSS), as described in the i2499 errata issue.

This workaround is required for devices up to Silicon Revision 1.2 on AM263x and Silicon Revision 1.1 on AM263Px. Therefore, the workaround is no longer required starting from Silicon Revision 1.3 on AM263x and Silicon Revision 1.2 on AM263Px, where the errata issue has been addressed in hardware.

This consisting of three projects:

1. **R5F Host Application** (`sdl_ecc_pru_scrubber_r5f0`)
   - Runs on the R5FSS0-0 core with FreeRTOS
   - Initializes SDL ECC and ESM hardware modules
   - Runs ECC functional tests (1-bit error injection) before activating the scrubber
   - Loads and controls the PRU firmware
   - Processes ECC SEC errors via ESM interrupt handlers

2. **PRU Firmware** (`sdl_ecc_pru_scrubber_pru0`)
   - Runs on ICSS_M0_PRU0
   - Performs active memory scrubbing
   - Detects ECC errors and signals the R5F core
   - Communicates findings back to the R5F core via shared DMEM flags

3. System Project
   - This is mainly to automate the build process.
   - Building system project will first build the PRU project and then the generated binary in PRU project will be copied into R5F project, and then finally R5F project will get build along with PRU binary.

This "active" scrubber implementation uses the ESM (Error Signaling Module) to intercept
ECC SEC (Single Error Correction) events and determine whether the PRU scrubber triggered
the correction, enabling safe fault clearance vs. device reset.

## Features
- Performs memory reads (scrubbing) over configurable memory range
- Detects Single Error Correction (SEC) events via ESM low-priority interrupt
- Determines if the ECC event was triggered by the PRU scrubber or by another bus master
- Integrates SDL ECC module for memory initialization and error injection self-test
- Uses direct ECC Aggregator (ECCAGG) register access via serial bus vector interface
- Supports scrubbing for entire OCRAM, for example in the case of AM263x it is 2MB (MSS_L2_SLV0, MSS_L2_SLV1, MSS_L2_SLV2 and MSS_L2_SLV3), 
  and in the case of AM263Px it is 3MB (MSS_L2_SLV0, MSS_L2_SLV1, MSS_L2_SLV2, MSS_L2_SLV3, MSS_L2_SLV4 and MSS_L2_SLV5)
## Communication Interface
The R5F and PRU communicate through a shared memory structure in PRU DMEM:

```c
typedef struct {
    uint32_t             startAddress;     // Memory address to start scrubbing
    uint32_t             numOfBursts;      // Number of burst reads to perform in memory scrubbing
    volatile uint8_t     pruECCStatusReadBit; // Set by PRU while it is reading each burst, this is a handshake signal between PRU and R5F in the case of SEC error occurred
    volatile uint8_t     pruTriggerBit;       // Set by PRU to confirm it triggered the SEC (scrub successful)
} dmemParameters;
```

## How the Active Scrubber Works

### Normal Operation Flow:
1. R5F loads the PRU firmware and initializes the DMEM parameters
2. PRU reads memory in burst mode starting at `startAddress` for `numOfBursts` times, after scrubbing entire memory the PRU will start scrubbing again from 'startAddress' and this process will continue forever from PRU side.
3. If there is no SEC then PRU will just continue its scrubbing in background

### Error Detection and Resolution:
1. If there is a SEC error detection then R5F will first enter into SEC interrupt handler routine and there it will verify whether SEC is caused by PRU or R5F itself.
2. In `App_serviceSecInterrupt()` reads the ECC SEC status register across all SRAM banks
3. For each bank with a pending SEC event:
   - R5F checks `pruTriggerBit`; if set, the PRU read triggered the SEC and scrub is successful
     R5F clears the ECC fault (STATUS1 register + EOI) and clears `pruTriggerBit`
     `scrubSuccess = true` is set, signaling the main task
4. If the SEC event occurred but the PRU did not confirm (CPU may have read corrupted data),
   the R5F halts in an infinite loop (device reset required)

## Project Structure

### R5F Host Application:
- `main.c` - Entry point and FreeRTOS initialization
- `empty_example.c` - Main application: PRU control, ECC/ESM setup, SEC interrupt handler
- `ecc_esm_config.c` - ECC Aggregator SEC enable, per-bank status/fault-address/clear helpers
- `ecc_trigger.c` - SDL ECC initialization and 1-bit / 2-bit error injection tests
- `eccagg.c` - Low-level ECC Aggregator register read/write via the serial bus vector
- `esm.c` - ESM driver: reset, enable, SEC/DED interrupt routing and ISR registration
- `dpl_interface.c` - SDL DPL interface implementation (interrupt registration, address translation)
- `ecc_main.h` - ECC module shared header (SDL includes, test result codes)
- `edma_rti_sram_scrub.h` - SRAM scrub defines (bank count, bank size, chunk size) and public API
- `pru0_load_bin.h` - PRU firmware binary in array format for loading

### PRU Firmware:
- `main.asm` - PRU assembly code implementing scrubbing and ECC status signaling
- `icss_constant_defines.inc` - PRU constant table definitions
- `linker.cmd` - Custom linker command file for PRU firmware

## SRAM Bank Configuration
| Bank      | SDL RAM ID            | Base Address  | Size   |
|-----------|-----------------------|---------------|--------|
| Bank 0    | MSS_L2_SLV0           | 0x70000000    | 512 KB |
| Bank 1    | MSS_L2_SLV1           | 0x70080000    | 512 KB |
| Bank 2    | MSS_L2_SLV2           | 0x70100000    | 512 KB |
| Bank 3    | MSS_L2_SLV3           | 0x70180000    | 512 KB |
If device is AM263Px:
| Bank 4    | MSS_L2_SLV4           | 0x70200000    | 512 KB |
| Bank 5    | MSS_L2_SLV5           | 0x70270000    | 512 KB |

## Building and Running

### Build Process:
1. There are 3 projects in the PRU scrubber (R5F0, PRU and System)
2. If you build system project then
   -First it build the PRU firmware project (`sdl_ecc_pru_scrubber_pru0`)
   -It automatically update the firmware binary generated by PRU project into the R5F project `pru0_load_bin.h` file.
   -Build the R5F host application (`sdl_ecc_pru_scrubber_r5f0`)
3. Connect the AM26x board to your computer
4. Load and run the R5F application, which will initialize ECC/ESM and load the PRU firmware

### Example Usage:
The example demonstrates scrubbing a 2MB for AM263x and 3MB for AM263Px
#define  SDL_ECC_RAM_TEST_ADDR   0x70000000U
#if defined(SOC_AM263X)
    #define  SDL_ECC_RAM_NUM_BYTES   0x200000U
#elif defined(SOC_AM263PX)
    #define  SDL_ECC_RAM_NUM_BYTES   0x300000U
#else
#endif
```

## Technical Notes

### Memory Scrubbing Considerations:
- The solution uses a read-only approach to avoid modifying memory content
- Each scrub operation reads memory in bursts; PRU burst size determines bytes per iteration

### ECC Aggregator Access:
- The ECC Aggregator exposes per-endpoint registers through a serial bus vector mechanism
- `ECCAGG_readRegister()` writes the vector register with read bit + endpoint ID, polls done bit, then reads the result
- `ECCAGG_writeRegister()` writes the vector register with endpoint ID (read bit cleared), then writes the value
- Worst-case serial read latency is ~35ms; the implementation waits up to ~70ms

### ESM Interrupt Configuration:
- ECC SEC (interrupt 19) â†’ ESM low-priority ISR â†’ `App_serviceSecInterrupt()`
- ECC DED (interrupt 20) â†’ ESM high-priority ISR (no action currently implemented)
- ESM config-register-corruption ISR is also registered for safety

### Error Handling Strategy:
- PRU sets `pruTriggerBit` before R5F can service the SEC interrupt
- R5F confirms the PRU was responsible for the SEC event before clearing the fault
- If the PRU did not confirm (CPU read corrupted data before correction), R5F halts to prevent
  propagation of bad data â€” a device reset is required in this case

Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/