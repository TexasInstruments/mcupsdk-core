# DTHE PKA RSA Encryption and Decryption Test {#EXAMPLES_DRIVERS_DTHE_PKA_RSA_ENC_DEC}

[TOC]

# Introduction

This example demonstrates the DTHE PKA Encryption and Decryption operations.

# Supported Combinations {#EXAMPLES_DRIVERS_DTHE_PKA_RSA_ENC_DEC_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/security/crypto/dthe_pka/rsa_encryption_decryption/
\endcond

## Build the PKA RSA Encryption and Decryption example
\code
$make -s -C examples/security/crypto/dthe_pka/rsa_encryption_decryption/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x
\endcode


# Steps to Run through ROM Boot flow 

## Via SBL_uart bootloader

### Build sbl_uart
\code
make -s -C examples/drivers/boot/sbl_uart/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x
\endcode

### Set board for UART boot

See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/EVM_SETUP_PAGE.html#autotoc_md29)

### Run UART_bootloader
\code
python uart_bootloader.py -p <COMxx> --bootloader=sbl_prebuilt/am263x-cc/sbl_uart.release.hs.tiimage --file=../../examples/security/crypto/dthe_pka/rsa_encryption_decryption/am263x-cc/r5fss0-0_nortos/ti-arm-clang/crypto_rsa_encryption_decryption.release.appimage
\endcode

### Sample output
On successful boot, R5 log at uart terminal, will have the following output. 
UART Console:
\code

[PKA] RSA Encryption and Decryption started ...
[PKA] RSA Encryption and Decryption completed!!
All tests have passed!!

\endcode

## Via SBL_qspi bootloader

### Build sbl_qspi
\code
make -C examples/drivers/boot/sbl_qspi/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x
\endcode

### Build sbl_uart_uniflash
\code
make -s -C examples/drivers/boot/sbl_uart_uniflash/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x
\endcode


### Set UART boot mode
See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/EVM_SETUP_PAGE.html#autotoc_md29)

### Edit default_sbl_qspi.cfg to include the correct images

### Run UART_uniflash
\code
python uart_uniflash.py -p <COMxx> --cfg=sbl_prebuilt/am263x-cc/default_sbl_qspi.cfg
\endcode

### Set QSPI boot mode
See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/EVM_SETUP_PAGE.html#autotoc_md29)

### Sample output
On successful boot, R5 log at uart terminal, will have the following output.

UART Console:
\code

[PKA] RSA Encryption and Decryption started ...
[PKA] RSA Encryption and Decryption completed!!
All tests have passed!!

\endcode

# See Also

\ref DRIVERS_DTHE_PAGE