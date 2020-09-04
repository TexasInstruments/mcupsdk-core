# SBL EMMC Linux {#EXAMPLES_DRIVERS_SBL_EMMC_LINUX}

\note SBL EMMC Linux boots from the boot partition 1 of eMMC and does not use a filesystem boot. The appimage is flashed to the offsets in eMMC as configured using syscfg.
\attention Care should be taken to ensure that the R5, M4 appimage and the Linux appimage flashed does NOT overwrite the Linux binaries in eMMC.
\note The load address of resource table for R5 and M4 cores must be consistant with the address in the Linux dts file.

[TOC]

# Introduction

This is a bootloader example, which shows an example of booting Linux on A53 core and RTOS/NORTOS applications on R5 and M4 cores from eMMC.

The SBL uses two appimages
- A Linux appimage containing the **Linux binaries (ATF, OPTEE, SPL)**.
- A muticore appimage containing the **RTOS/NORTOS applications for R5 and M4 cores**.

The bootloader does SOC initializations and parses the multicore appimage present at 0x800000 in eMMC boot partition 1, splits it into RPRCs for each core applicable. Each core is then initialized, RPRC image is loaded, entry points are set and the core is released from reset.

For booting Linux, SBL parses the Linux appimage present at 0xA00000 in eMMC boot partition, splits it into individual linux binaries (ATF, OPTEE, SPL). SBL loads the Linux binaries, entry point is set to the start address of ATF and A53 core is released from reset.


# Supported Combinations

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_emmc_linux

# Steps to Run the Example

## Build the example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Flash the eMMC with the default linux image

\note This needs to be the first step as later the tiboot3.bin at the starting of the bootpartition will be overwritten by `sbl_emmc_linux.tiimage` .

- For booting A53 with linux, eMMC needs to be flashed with the Linux image. Refer to **Processor SDK Linux** user guide on how to flash eMMC to boot Linux.

## Create Linux Appimage

\note Change DEVICE_TYPE to HS in ${SDK_INSTALL_PATH}/devconfig/devconfig.mak and then generate Linux Appimage for HS-SE device.

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

- This example is the SBL which needs to be flashed on the eMMC, along with sample application images for R5, M4 CPUs and Linux Appimage.
\note Use **default_sbl_emmc_linux_hs.cfg** when flashing to HS devices
- There is a default flash config file as shown below which flashes this SBL and the IPC RPMsg Linux echo applications

        ${SDK_INSTALL_PATH}/examples/drivers/boot/sbl_emmc_linux/@VAR_BOARD_NAME_LOWER/{cpu}_{os}/default_sbl_emmc_linux.cfg

- Make sure IPC rpmsg linux echo application is built before running the flash script. (see \ref EXAMPLES_DRIVERS_IPC_RPMESSAGE_LINUX_ECHO)

\note For IPC rpmsg linux echo, the resource table entity must be placed at the beginning of remoteproc memory section as mentoined in Linux dts file.

- To flash to the EVM, refer to \ref GETTING_STARTED_FLASH . Only when giving the flash config file, point to the `default_sbl_emmc_linux.cfg` shown above.

- Example, assuming SDK is installed at `C:/ti/mcu_plus_sdk` and this example and IPC application is built using makefiles, and Linux Appimage is already created, in Windows,

        cd C:/ti/mcu_plus_sdk/tools/boot
        C:/ti/mcu_plus_sdk/tools/boot>python uart_uniflash.py -p COM13 --cfg=C:/ti/mcu_plus_sdk/examples/drivers/boot/sbl_emmc_linux/@VAR_BOARD_NAME_LOWER/r5fss0-0_nortos/default_sbl_emmc_linux.cfg

- Boot the EVM in eMMC boot mode to boot Linux on A53 and RTOS/Baremetal application on R5 and M4 cores.

\note User might be required to set environmet variables from uBooot prompt to boot linux kernel from eMMC. Refer to **Processor SDK Linux** user guide for details.

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

After flashing and booting the EVM, you will see below output on the UART console (Complete log is not shown)

    [BOOTLOADER PROFILE] SYSFW Load                       :      17592us
    [BOOTLOADER PROFILE] System_init                      :      18177us
    [BOOTLOADER PROFILE] Drivers_open                     :     764669us
    [BOOTLOADER PROFILE] Board_driversOpen                :          0us
    [BOOTLOADER PROFILE] App_loadImages                   :     479465us
    [BOOTLOADER_PROFILE] SBL Total Time Taken             :    1431232us

    Image loading done, switching to application ...
    Starting linux and RTOS/Baremetal applications

    NOTICE:  BL31: v2.5(release):08.00.00.004-dirty
    NOTICE:  BL31: Built : 14:02:03, Aug  5 2021

    U-Boot SPL 2021.01-g53e79d0e89 (Aug 05 2021 - 14:03:40 +0000)
    SYSFW ABI: 3.1 (firmware rev 0x0015 '21.5.0--v2021.05 (Terrific Llam')
    Trying to boot from MMC1


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
