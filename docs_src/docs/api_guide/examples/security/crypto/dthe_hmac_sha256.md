# DTHE HMAC SHA 256 {#EXAMPLES_DRIVERS_DTHE_HMAC_SHA_256}

[TOC]

# Introduction
This example demonstrates HMAC SHA-256 algorithm running on SHA/MD5 module. This example explains the steps to build and run HMAC SHA-256.

\cond SOC_AM263X || SOC_AM263PX

 Parameter             | Value
 ----------------------|-----------
 CPU + OS              | r5fss0-0 nortos
 Toolchain             | ti-arm-clang
 Boards                | @VAR_BOARD_NAME_LOWER
 Example folder        | examples/security/crypto/dthe_sha/crypto_hmac_sha256/
 Supported Device Type | HS-FS

\endcond

\cond SOC_AM273X

 Parameter             | Value
 ----------------------|-----------
 CPU + OS              | r5fss0-0 nortos
 Toolchain             | ti-arm-clang
 Boards                | @VAR_BOARD_NAME_LOWER
 Example folder        | examples/security/crypto/dthe_sha/crypto_hmac_sha256/
 Supported Device Type | HS-SE, GP

\endcond


## Build the hmac sha 256 example
\code
$make -s -C examples/security/crypto/dthe_sha/crypto_hmac_sha256/<soc>-<board>/r5fss0-0_nortos/ti-arm-clang all DEVICE=<soc>
\endcode


# Steps to Run through ROM Boot flow

\note If the Device Type is HS-SE, an additional flag "DEVICE_TYPE=HS" is passed for building SBL. The flag results in generation of sbl image with suffix "hs.tiimage" which can only be used when device type is HS-SE.

## Via SBL_uart bootloader

### Build sbl_uart
\code
make -s -C examples/drivers/boot/sbl_uart/<soc>-<board>/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=<soc>
\endcode

### Set board for UART boot

See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/@VAR_SOC_NAME/latest/exports/docs/api_guide_@VAR_SOC_NAME_LOWER/EVM_SETUP_PAGE.html#autotoc_md29)

### Run UART_bootloader
\code
python uart_bootloader.py -p <COMxx> --bootloader=sbl_prebuilt/<soc>-<board>/sbl_uart.release.hs.tiimage --file=../../examples/security/crypto/dthe_sha/crypto_hmac_sha256/<soc>-<board>/r5fss0-0_nortos/ti-arm-clang/dthe_hmac_sha256.release.appimage
\endcode

### Sample output
On successful boot, R5 log at uart terminal, will have the following output.
UART Console:
\code
[CRYPTO] DTHE HMAC SHA-256 example started ...
[CRYPTO] DTHE HMAC SHA-256 example completed!!
All tests have passed!!

\endcode

## Via SBL_qspi bootloader

### Build sbl_qspi
\code
make -C examples/drivers/boot/sbl_qspi/<soc>-<board>/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=<soc>
\endcode

### Build sbl_uart_uniflash
\code
make -s -C examples/drivers/boot/sbl_uart_uniflash/<soc>-<board>/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=<soc>
\endcode


### Set UART boot mode
See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/@VAR_SOC_NAME/latest/exports/docs/api_guide_@VAR_SOC_NAME_LOWER/EVM_SETUP_PAGE.html#autotoc_md29)

### Edit default_sbl_qspi.cfg to include the correct images

### Run UART_uniflash
\code
python uart_uniflash.py -p <COMxx> --cfg=sbl_prebuilt/<soc>-<board>/default_sbl_qspi.cfg
\endcode

### Set QSPI boot mode
See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/@VAR_SOC_NAME/latest/exports/docs/api_guide_@VAR_SOC_NAME_LOWER/EVM_SETUP_PAGE.html#autotoc_md29)

### Sample output
On successful boot, R5 log at uart terminal, will have the following output.

UART Console:
\code
[CRYPTO] DTHE HMAC SHA-256 example started ...
[CRYPTO] DTHE HMAC SHA-256 example completed!!
All tests have passed!!

\endcode

# See Also

\ref DRIVERS_DTHE_PAGE