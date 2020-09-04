#  HSM Services Example {#EXAMPLES_HSM_SERVICES}

[TOC]

# Introduction

This example demonstrates the use of HSM services.

Two services are available.
1. UID is a unique identifier for each device. It can be extracted  using **get_uid service**.
2. HSM **get_version_service** serves the client with TIFS-MCU version information.

# Supported Combinations {#EXAMPLES_HSM_SERVICES_COMBOS}

\cond SOC_AM263X

 Parameter             | Value
 ----------------------|-----------
 CPU + OS              | r5fss0_0 nortos
 Toolchain             | ti-arm-clang
 Boards                | @VAR_BOARD_NAME_LOWER
 Example folder        | /examples/hsm_client/hsm_services/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

## Via SBL_qspi bootloader

### Build sbl_qspi

\code
cd ~/ti/mcu_plus_sdk
make -s -C examples/drivers/boot/sbl_qspi/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x
\endcode

### Build sbl_uart_uniflash
\code
cd ~/ti/mcu_plus_sdk
make -s -C examples/drivers/boot/sbl_uart_uniflash/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x
\endcode

#### Edit default_sbl_qspi.cfg to include the correct images

- Example config file.

```
  #-----------------------------------------------------------------------------#
  #                                                                             #
  #      DEFAULT CONFIGURATION FILE TO BE USED WITH THE FLASHWRITER SCRIPT      #
  #                                                                             #
  #-----------------------------------------------------------------------------#
  #
  # By default this config file,
  # - points to pre-built flash writer, bootloader for this EVM
  # - The application image points to relative path of the ipc echo application image for this EVM
  #   - Make sure this application is built before running this script
  # - You can customized this config file to point to your own bootloader and/or application images
  # - You can use --operation=flashverify if you just want to verify the flash contents and not flash the file.
  #

  # First point to sbl_uart_uniflash binary, which function's as a server to flash one or more files
  --flash-writer=sbl_prebuilt/am263x-cc/sbl_uart_uniflash.release.tiimage

  # Now send one or more files to flash or flashverify as needed. The order of sending files does not matter

  # When sending bootloader make sure to flash at offset 0x0. ROM expects bootloader at offset 0x0
  --file=sbl_prebuilt/am263x-cc/sbl_qspi.release.tiimage --operation=flash --flash-offset=0x0

  # When sending application image, make sure to flash at offset 0x80000 (default) or to whatever offset your bootloader is configured for
  --file=../../examples/drivers/hsmclient/hsm_services/am263x-cc/r5fss0-0_nortos/ti-arm-clang/hsm_services.release.appimage --operation=flash --flash-offset=0x80000
```

### Sample output

On successful boot, following message will be seen on terminal.

\imageStyle{hsmrt_ver.png,width:50%}
\image html hsmrt_ver.png "Services Sample Output"

### Set board for UART boot

See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/EVM_SETUP_PAGE.html#autotoc_md29)

### Load image

Power cycle and load image via xmodem (teraterm/minicom, baud rate 115200bps)

### Sample Output

On successful boot, following message will be seen on terminal. Please note that
the R5F application has not been side-loaded yet.
\imageStyle{hsmrt_ver_uart.PNG,width:50%}
\image html hsmrt_ver_uart.PNG "Sample Output"

