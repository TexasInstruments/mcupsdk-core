# SBL OSPI Linux {#EXAMPLES_DRIVERS_SBL_OSPI_LINUX}

\note The load address of resource table for R5 and M4 cores must be consistant with the address in the Linux dts file.

[TOC]

# Introduction

\if SOC_AM65X
This is a bootloader example, which shows an example of booting Linux on A53 core and RTOS/NORTOS applications on R5 cores.

The SBL uses two appimages
- A Linux appimage containing the **Linux binaries (ATF, OPTEE, SPL)**.
- A muticore appimage containing the **RTOS/NORTOS applications for R5 cores**.

The bootloader does SOC initializations and parses the multicore appimage present at 0x100000, splits it into RPRCs for each core applicable. Each core is then initialized, RPRC image is loaded, entry points are set and the core is released from reset.

For booting Linux, SBL parses the Linux appimage present at 0x800000, splits it into individual linux binaries (ATF, OPTEE, SPL). SBL loads the Linux binaries, entry point is set to the start address of ATF and A53 core is released from reset.
\else
This is a bootloader example, which shows an example of booting Linux on A53 core and RTOS/NORTOS applications on R5 and M4 cores.

The SBL uses two appimages
- A Linux appimage containing the **Linux binaries (ATF, OPTEE, SPL)**.
- A muticore appimage containing the **RTOS/NORTOS applications for R5 and M4 cores**.

The bootloader does SOC initializations and parses the multicore appimage present at 0x80000, splits it into RPRCs for each core applicable. Each core is then initialized, RPRC image is loaded, entry points are set and the core is released from reset.

For booting Linux, SBL parses the Linux appimage present at 0x300000, splits it into individual linux binaries (ATF, OPTEE, SPL). SBL loads the Linux binaries, entry point is set to the start address of ATF and A53 core is released from reset.

\endif

# Supported Combinations

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_ospi_linux

# Steps to Run the Example

## Build the example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Create an SD-Card with Linux image

- For booting A53 with linux SD-Card with linux image needs to be created. Refer to **Processor SDK Linux** user guide on how to create SD-Card to boot Linux.

## Create Linux Appimage

\if SOC_AM65X
\note Change DEVICE_TYPE to HS in ${SDK_INSTALL_PATH}/devconfig/devconfig.mak and then generate Linux Appimage for HS device.
\else
\note Change DEVICE_TYPE to HS in ${SDK_INSTALL_PATH}/devconfig/devconfig.mak and then generate Linux Appimage for HS-SE device.
\endif

\note User needs to build A53 SPL images (ATF, OPTEE and uboot) and the prebuilt images
in linux SDK is signed and the Linux appimage gen tool requries unsigned binaries

\note Instructions to build A53 ATF, OPTEE, uboot can be found in the SDK Linux documentation at
        **Foundational Components » ARM Trusted Firmware-A**
        **Foundational Components » OP-TEE**
        **Foundational Components » U-Boot » User’s Guide » General Information » Build U-Boot**

- Create a Linux Appimage containing the **Linux binaries (ATF, OPTEE, SPL)**
- This can be done by running the makefile at {SDK_INSTALL_PATH}/tools/boot/linuxAppimageGen after setting the prebuilt images path in file `config.mak`
- Refer \ref LINUX_APPIMAGE_GEN_TOOL for more details

## Run the example

\if SOC_AM65X
- This example is the SBL which needs to be flashed on the EVM flash, along with sample application images for R5 CPUs and Linux Appimage.
\else
- This example is the SBL which needs to be flashed on the EVM flash, along with sample application images for R5, M4 CPUs and Linux Appimage.
\endif
\note Use **default_sbl_ospi_linux_hs.cfg** when flashing to HS devices
- There is a default flash config file as shown below which flashes this SBL and the IPC RPMsg Linux echo applications

        ${SDK_INSTALL_PATH}/examples/drivers/boot/sbl_ospi_linux/@VAR_BOARD_NAME_LOWER/{cpu}_{os}/default_sbl_ospi_linux.cfg

- Make sure IPC rpmsg linux echo application is built before running the flash script. (see \ref EXAMPLES_DRIVERS_IPC_RPMESSAGE_LINUX_ECHO)

\note For IPC rpmsg linux echo, the resource table entity must be placed at the beginning of remoteproc memory section as mentoined in Linux dts file.

- To flash to the EVM, refer to \ref GETTING_STARTED_FLASH . Only when giving the flash config file, point to the `default_sbl_ospi_linux.cfg` shown above.

- Example, assuming SDK is installed at `C:/ti/mcu_plus_sdk` and this example and IPC application is built using makefiles, and Linux Appimage is already created, in Windows,

        cd C:/ti/mcu_plus_sdk/tools/boot
        C:/ti/mcu_plus_sdk/tools/boot>python uart_uniflash.py -p COM13 --cfg=C:/ti/mcu_plus_sdk/examples/drivers/boot/sbl_ospi_linux/@VAR_BOARD_NAME_LOWER/r5fss0-0_nortos/default_sbl_ospi_linux.cfg

- Boot the EVM in OSPI boot mode with the SD card containing the Linux image in the EVM.

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

After flashing and booting the EVM, you will see below output on the UART console (Complete log is not shown)
\if SOC_AM65X

    [BOOTLOADER PROFILE] System_init                      :        445us
    [BOOTLOADER PROFILE] Drivers_open                     :        105us
    [BOOTLOADER PROFILE] Board_driversOpen                :      14457us
    [BOOTLOADER PROFILE] SYSFW init                       :     199687us
    [BOOTLOADER PROFILE] App_loadImages                   :     180472us
    [BOOTLOADER_PROFILE] SBL Total Time Taken             :     897993us

    Image loading done, switching to application ...
    Starting linux and RTOS/Baremetal applications

    U-Boot SPL 2021.01-g44a87e3ab8 (Mar 24 2022 - 02:48:34 +0000)
    SYSFW ABI: 3.1 (firmware rev 0x0016 '22.1.1--v2022.01 (Terrific Llam')
    Trying to boot from MMC2

    .
    .
    .
    .
    .
    .


    _____                    _____           _         _
    |  _  |___ ___ ___ ___   |  _  |___ ___  |_|___ ___| |_
    |     |  _| .'| . | . |  |   __|  _| . | | | -_|  _|  _|
    |__|__|_| |__,|_  |___|  |__|  |_| |___|_| |___|___|_|
                |___|                    |___|

    Arago Project am65xx-evm ttyS1

    Arago 2021.09 am65xx-evm ttyS1

    am65xx-evm login:
\else

    [BOOTLOADER PROFILE] SYSFW Load                       :      17592us
    [BOOTLOADER PROFILE] System_init                      :      19018us
    [BOOTLOADER PROFILE] Drivers_open                     :        141us
    [BOOTLOADER PROFILE] Board_driversOpen                :      21886us
    [BOOTLOADER PROFILE] App_loadImages                   :       3446us
    [BOOTLOADER_PROFILE] SBL Total Time Taken             :      65575us

    Image loading done, switching to application ...
    Starting linux and RTOS/Baremetal applications
    NOTICE:  BL31: v2.5(release):08.00.00.004-dirty
    NOTICE:  BL31: Built : 14:02:03, Aug  5 2021

    U-Boot SPL 2021.01-g53e79d0e89 (Aug 05 2021 - 14:03:40 +0000)
    SYSFW ABI: 3.1 (firmware rev 0x0015 '21.5.0--v2021.05 (Terrific Llam')
    Trying to boot from MMC2

    .
    .
    .
    .
    .
    .


    _____                    _____           _         _
    |  _  |___ ___ ___ ___   |  _  |___ ___  |_|___ ___| |_
    |     |  _| .'| . | . |  |   __|  _| . | | | -_|  _|  _|
    |__|__|_| |__,|_  |___|  |__|  |_| |___|_| |___|___|_|
                |___|                    |___|

    Arago Project http://arago-project.org am64xx-evm ttyS2

    Arago 2020.09 am64xx-evm ttyS2

    am64xx-evm login:
\endif
