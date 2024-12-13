# Achieving Fast Secure Boot and Boot time calculator {#FAST_SECURE_BOOT}

## Achieving Fast Secure Boot

### Introduction

This document is intended for the users who want to achieve the least secure boot time for a signed (optionally encrypted) application through Flash interface

### Steps to build, flash, load and run secure applications

* Make sure **pyelftools**, **construct** and **cryptography** is installed from pip. This package is required for encrypting MCELF application images.

\code
    pip install pyelftools construct cryptography
\endcode

* Set **MCELF_MERGE_SEGMENTS_FLAG = false** and **MCELF_MAX_SEGMENT_SIZE = 65536(64KB)** in devconfig.mak file present in MCU+ SDK.

* Build the libraries for the specific SOC in MCU+ SDK .
\cond SOC_AM263PX
\code   
    make libs DEVICE=am263px
\endcode
\endcond
\cond SOC_AM263X
\code   
    make libs DEVICE=am263x
\endcode
\endcond

* Build the HSM runtime firmware for the HSSE device with the command below in TIFSMCU.
\cond SOC_AM263X
\code
    make -s -C hsm_firmware/am263x/hsse/hsm0-0_nortos/ti-arm-clang/ all DEVICE=am263x DEVICE_TYPE=HS
\endcode
\endcond
\cond SOC_AM263PX
\code
    make -s -C hsm_firmware/am263px/hsse/hsm0-0_nortos/ti-arm-clang/ all DEVICE=am263px DEVICE_TYPE=HS
\endcode
\endcond

* Build the fastboot SBL project which has various optimizations.
\cond SOC_AM263PX  
\code
    make -sj -c examples/drivers/boot/sbl_ospi_fastboot/am263px-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263px DEVICE_TYPE=HS
\endcode
\endcond
\cond SOC_AM263X
\code
    make -sj -c examples/drivers/boot/sbl_qspi_fastboot/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x DEVICE_TYPE=HS
\endcode
\endcond

\note
    The HSM runtime flash address can be configured via sysconfig tool. 
    Open the sysconfig gui for the fastboot SBL and change the **HSM Runtime Image Offset** option to the new value.
    The flash address must be the same in the flash writer config and the SBL sysconfig for correct SBL operation.
    \image html hsmrt_flash_address.png "HSM Runtime Flash Offset" width=50%

* Build the application with **DEVICE** and **DEVICE_TYPE=HS** options to make a secure MCELF signed application . The extension of this application is *.mcelf.hs .
\cond SOC_AM263PX
\code   
    make -sj -c examples/hello_world/am263px-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263px DEVICE_TYPE=HS
\endcode
\endcond
\cond SOC_AM263X
\code
    make -sj -c examples/hello_world/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x DEVICE_TYPE=HS
\endcode
\endcond

\note
    Make sure that application and SBL OCRAM usage is mutually exclusive **except** the vector table.
    This can be done by looking at the MAP files of both the SBL and Application. 
    In case of overlap between the two, move the application to a greater address using sysconfig memory configurator tool.

* Set the board to UART bootmode and update the flash contents. Checkout the steps mentioned in \ref GETTING_STARTED_FLASH . 
\cond SOC_AM263X  
    * Please use fast_sbl_qspi.cfg present in ${MCU_PLUS_SDK_PATH}/tools/boot/sbl_prebuilt/am263x-cc/fast_sbl_qspi.cfg . 
\endcond
\cond SOC_AM263PX  
    * Please use fast_sbl_ospi.cfg present in ${MCU_PLUS_SDK_PATH}/tools/boot/sbl_prebuilt/am263px-cc/fast_sbl_ospi.cfg . 
\endcond

\note
    HSM runtime firmware path must be provided in this fast sbl config in addition to sbl_qspi_fastboot, application and sbl_uart_uniflash binary paths .

\cond SOC_AM263X
* Set the board to QSPI bootmode and reset the device.

No logs are expected in sbl_qspi_fastboot since UART logging is disabled.
\endcond
\cond SOC_AM263PX
* Set the board to OSPI bootmode and reset the device.

No logs are expected in sbl_ospi_fastboot since UART logging is disabled.
\endcond

## Component wise secure boot time measurement

Secure Boot time can be measured by using GPIOs at various points of execution.

The points of interest for Secure Boot are:
1. HSMRT load request and acknowledgement
    * This can be measured with the Hsmclient_loadHSMRtFirmwareNonBlocking function and Hsmclient_mboxRxISRNonBlocking ISR present in **hsmclient_loadhsmrt.c** in the source/security/security_common folder.
2. Flash read call in board flash drivers.
    * This can be measured with the functions present in **flash_xxxx_xspi.c** in the source/board/flash/xspi/ folder.
3. HSM Authentication/Encryption request: 
    * This can be measured by toggling GPIOs in the TIFS streaming service ISRs.

## Boot time calculator

The boot time calculator can be used get an idea of the secure boot time values which can be achieved with fast boot sbls and tifs mcu. The numbers are as per SDK 10.00 version.

Checkout the secure boot time calculator tool <a href="../boottime_calculator/index.html">here</a> .
