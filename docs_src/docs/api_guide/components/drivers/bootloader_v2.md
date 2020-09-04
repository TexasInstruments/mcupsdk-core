# BOOTLOADER {#DRIVERS_BOOTLOADER_PAGE}

[TOC]

The Bootloader module provides APIs to write bootloader applications for various boot media like OSPI, UART, SOC memory etc.

## Features Supported

- QSPI Boot
- UART Boot
- API to parse multicore appimage
- Separate APIs to boot self and non-self cores

## R5 Dual Core Support

- With the configuration used in MCU SDK to generate the app images, the RBL brings the R5 Lock Step Mode.
- The SBL runs on R5 core in Lock Step Mode.
- The SBL will switch to Dual core or continue in lock step mode depending on whether App Image loaded by SBL contains the R5 Core 1 image or not.
- If the App Image to be loaded contains the R5 core 1 binary
    - The SBL will configure R5 to switch to Dual Core Mode.
    - At the end of SBL flow, The Application image of R5 will start running in the dual core mode.
- If the App Image doesnot contain the R5 core 1 binary
    - The SBL will not configure R5 to switch to Dual core Mode
    - At the end of SBL flow, The Application image of R5 will start running in the lock step mode.
- Refer the IPC example which configures the R5 in Dual Core mode. \ref EXAMPLES_DRIVERS_IPC_NOTIFY_ECHO

## Example Usage

Include the below file to access the APIs
\snippet Bootloader_sample.c include

Instance Open Example
\snippet Bootloader_sample.c open

Booting Cores Example
\snippet Bootloader_sample_v1.c bootcores_am273x

Instance Close Example
\snippet Bootloader_sample.c close

## API

\ref DRV_BOOTLOADER_MODULE