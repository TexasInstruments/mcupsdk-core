# ICSS EMAC Firmware Build Guide

This document describes how to build the ICSS Dual EMAC and ICSS Switch firmware using Code Composer Studio (CCS).

## Directory Structure

```
firmware/
|-- source/                          # Common source files for both firmwares
|   |-- micro_scheduler.asm          # Main control loop and initialization
|   |-- emac_MII_Rcv.asm             # Packet receive routines
|   |-- emac_MII_Xmt.asm             # Packet transmit routines
|   |-- emac_statistics.asm          # Statistics handling
|   |-- emac_ptp.asm                 # PTP (Precision Time Protocol) support
|   |-- emac_tts.asm                 # Time Triggered Send support
|   |-- switch_collision_task.asm    # Switch collision handling (ICSS Switch only)
|   |-- pru.cmd                      # Linker command file
|   +-- *.h                          # Header files
|
|-- icss_dual_emac/                  # ICSS Dual EMAC firmware
|   +-- mii/
|       |-- am263x-cc/               # AM263x ControlCard
|       |-- am263x-lp/               # AM263x LaunchPad
|       |-- am263px-cc/              # AM263Px ControlCard
|       +-- am263px-lp/              # AM263Px LaunchPad
|           |-- icss_m0_pru0_fw/     # PRU0 firmware project
|           +-- icss_m0_pru1_fw/     # PRU1 firmware project
|
+-- icss_switch/                     # ICSS Switch firmware
    +-- mii/
        |-- am263x-cc/
        |-- am263x-lp/
        |-- am263px-cc/
        +-- am263px-lp/
            |-- icss_m0_pru0_fw/
            +-- icss_m0_pru1_fw/
```

## Supported Boards

| Board         | Device   | Description          |
|---------------|----------|----------------------|
| am263x-cc     | AM263x   | ControlCard          |
| am263x-lp     | AM263x   | LaunchPad            |
| am263px-cc    | AM263Px  | ControlCard          |
| am263px-lp    | AM263Px  | LaunchPad            |

## Prerequisites

- Code Composer Studio (CCS) with PRU Code Generation Tools (ti-pru-cgt) version 2.3.3 or later
- MCU+ SDK installed

## Building the Firmware in CCS

### Step 1: Import the Project

1. Open Code Composer Studio
2. Go to **Project** > **Import CCS Projects...**
3. Click **Browse...** and navigate to the desired project location:
   - For **ICSS Dual EMAC**:
     ```
     <MCU_PLUS_SDK>/source/networking/icss_emac/firmware/icss_dual_emac/mii/<board>/icss_m0_pru<0|1>_fw/ti-pru-cgt/
     ```
   - For **ICSS Switch**:
     ```
     <MCU_PLUS_SDK>/source/networking/icss_emac/firmware/icss_switch/mii/<board>/icss_m0_pru<0|1>_fw/ti-pru-cgt/
     ```
4. Click **Finish** to import the project

### Step 2: Build the Project

1. Select the imported project in the **Project Explorer**
2. Choose the build configuration:
   - **Release**: Optimized build (-O3)
   - **Debug**: Debug build with symbols
3. Right-click the project and select **Build Project**

### Step 3: Locate Output Files

After a successful build, the following output files are generated:

- `PRU0_bin.h` or `PRU1_bin.h` - C header file with firmware binary array

The header files are automatically copied to:
- ICSS Dual EMAC: `firmware/icss_dual_emac/mii/`
- ICSS Switch: `firmware/icss_switch/mii/`

## Building Both PRU Cores

Each ICSS instance requires firmware for both PRU0 and PRU1. Make sure to build both projects:

1. `icss_m0_pru0_fw` - PRU0 firmware
2. `icss_m0_pru1_fw` - PRU1 firmware

## Notes

- The projects use the TI PRU Code Generation Tools (ti-pru-cgt) toolchain
- Source files are shared between all board variants and are linked (not copied) into each project
- The `MCU_PLUS_SDK_PATH` variable is automatically set based on your SDK installation
