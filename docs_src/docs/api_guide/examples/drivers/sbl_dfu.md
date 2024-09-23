# SBL DFU {#EXAMPLES_DRIVERS_SBL_DFU}

[TOC]

# Introduction


**SBL_DFU** is a bootloader which boots the multicore application received via USB DFU. It is analogous to \ref EXAMPLES_DRIVERS_SBL_UART

- DFU which stands for (Device Firmware Upgrade) is a standard device class of \ref USB_DEVICE_DRIVER. This application uses DFU to transfer appimages from host pc to RAM. 

	- It uses DFU class from TinyUSB stack please refer to \ref USB_DEVICE_DRIVER for more information. 

- SBL DFU implements the **dfu task** which receives the appimage and put it into a dedicated buffer in RAM memory. Once the manifestation stage of DFU state machine is completed and DFU state machine goes to IDLE state, then the dfu task will be exited and booting process of the received image will be initiated. 

- It uses bootloader APIs to do the SOC initializations. Once the appimage is received, the SBL parses it and splits the appimage into RPRCs where each RPRC image corresponds to a distinct core. 
After that it initializes each core and load the corresponding RPRC image. This step sets the entry points and the core is then released from reset. 

- Refer to \htmllink{https://www.usb.org/sites/default/files/DFU_1.1.pdf, DFU_1.1.pdf} to know more about USB DFU class. 

\note This bootloader performs memory boot i.e it directly boots from the appimage received in RAM buffer. Due to this as of now the maximum boot image size supported with SBL DFU is **384KB**

\cond SOC_AM261X

# SBL DFU MULTICORE ELF {#EXAMPLES_DRIVERS_SBL_DFU_MCELF}

To parse and load an **mcelf** file via DFU bootloader, use the project **examples/drivers/boot/sbl_dfu_multicore_elf**

When an mcelf image is received, the SBL parses it, loads each segment to its respective core. Then the core is released from reset. For more information refer \ref BOOTFLOW_GUIDE

The steps to run the example is same irrespective of the image format.

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_DFU_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_dfu

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_dfu

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_dfu

\endcond

# Steps to Run the Example

Since this is a bootloader and is used as a SOC initialization binary, the example will be run every time you boot an application using this example. It is generally run from a boot media (OSPI Flash, SD Card or over UART) unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE )

# Example Usage

\cond SOC_AM261X 
The standard DFU-Utility is not compatible with the AM261x device. Instead, please use the dfu-util executable located in ${SDK_INSTALL_PATH}/tools/boot/usb_dfu_utility, depending on your operating system, to carry out DFU operations.
\endcond

- Make sure that \ref INSTALL_DFU_UTIL tool is installed on your host pc. In case of windows make sure that appropriate windows USB generic drivers are installed. 
- The application boot is a 4 step process. 

1. Put the device into DFU boot mode \ref EVM_SETUP_PAGE
2. Connect the USB cable to the HOST PC and observer that correct DFU device is detected. 
3. Run the following command which uses \ref INSTALL_DFU_UTIL to send the SBL DFU bootloader tiimage. 


        sudo dfu-util -i 0 -a 0 <path to mcu_plus_sdk>/tools/boot/sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_dfu.release.hs_fs.tiimage 

 \imageStyle{sbl_dfu_rom.png,width:50%}
 \image html sbl_dfu_rom.png "DFU ROM boot log"

    4 Run the following command which uses \ref INSTALL_DFU_UTIL tool to send the appimages. SBl DFU receives that and boots the application. 

        sudo dfu-util -i 0 -a 0 -D <path to appimage>

- Another option is to use the \ref USB_BOOTLOADER which abstracts the setps 3 and 4 mentioned above. 

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

\ref TOOLS_FLASH_DFU_UNIFLASH. 

\ref TOOLS_FLASH

\cond SOC_AM243X || SOC_AM64X
\ref GETTING_STARTED_FLASH_DFU
\endcond

\ref USB_BOOTLOADER 


