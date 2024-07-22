# Flashing Tools {#TOOLS_FLASH}

[TOC]

## Introduction

Flashing tools allow to flash binaries to the flash on a EVM.

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM243X || SOC_AM261X
- \ref TOOLS_TI_UNIFLASH_TOOL
\endcond
- \ref TOOLS_FLASH_UART_UNIFLASH
\cond SOC_AM64X || SOC_AM243X
- \ref TOOLS_FLASH_DFU_UNIFLASH
\endcond
- \ref TOOLS_FLASH_JTAG_UNIFLASH
\cond SOC_AWR294X
- \ref TOOLS_FLASH_ENET_UNIFLASH
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
- \ref TOOLS_FLASH_CAN_UNIFLASH
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM243X || SOC_AM261X
## TI Uniflash {#TOOLS_TI_UNIFLASH_TOOL}

\note Sitara MCU devices now supports TI Uniflash Tool for loading/flashing images into the target. Please refer \ref TI_UNIFLASH_TOOL for more details.

\cond SOC_AM243X
### JTAG Session
\attention It is recommended to do \ref EVM_SOC_INIT_NOBOOT_MODE and disconnect the R5 Core from CCS before flashing the image from UNIFLASH.

1.  Set the board in \ref BOOTMODE_NOBOOT mode and do a power cycle prior to loading.

2.  Choose a Program to Flash:
    - Click the "Browse" button to select the program you want to flash.
    - The default start address is automatically filled. UniFlash requires the full address since flash offsets are not supported.
    \imageStyle{load_jtag.png,width:70%}
    \image html load_jtag.png "Load Binary Image"
    - Do not modify the address field for XIP file type. The field is used by the flash loader to recognize XIP files.
3.  Flash Address Table:
    - The table below shows the flash addresses accepted by the ROM/SBL to load programs onto the target:
        | Program     | Start Address |
        |-------------|---------------|
        | SBL         | 0x60000000    |
        | Application | 0x60080000    |
    - Edit the application offset field only, if your sbl is configured with custom application offset
4.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.
\endcond

\cond SOC_AM263PX || SOC_AM261X
### JTAG Session
1.  Set the board in \ref BOOTMODE_NOBOOT mode and do a power cycle prior to loading.

2.  Choose a Program to Flash:
    - Click the "Browse" button to select the program you want to flash.
    - The default start address is automatically filled. UniFlash requires the full address since flash offsets are not supported.
    \imageStyle{load_jtag_2.png,width:70%}
    \image html load_jtag_2.png "Load Binary Image"
    - Do not modify the address field for XIP file type. The field is used by the flash loader to recognize XIP files.
3.  Flash Address Table:
    - The table below shows the flash addresses accepted by the ROM/SBL to load programs onto the target:
        | Program     | Start Address |
        |-------------|---------------|
        | SBL         | 0x60000000    |
        | Application | 0x60080000    |
    - Edit the application offset field only, if your sbl is configured with custom application offset
4.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.
\endcond

\cond SOC_AM263X
### JTAG Session
1.  Set the board in \ref BOOTMODE_NOBOOT mode and do a power cycle prior to loading.

2.  Choose a Program to Flash:
    - Click the "Browse" button to select the program you want to flash.
    - The default start address is automatically filled. UniFlash requires the full address since flash offsets are not supported.
    \imageStyle{load_jtag_1.png,width:70%}
    \image html load_jtag_1.png "Load Binary Image"
    - Do not modify the address field for XIP file type. The field is used by the flash loader to recognize XIP files.
3.  Flash Address Table:
    - The table below shows the flash addresses accepted by the ROM/SBL to load programs onto the target:
        | Program     | Start Address |
        |-------------|---------------|
        | SBL         | 0x60000000    |
        | Application | 0x60080000    |
    - Edit the application offset field only, if your sbl is configured with custom application offset
4.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.
\endcond

\cond SOC_AM243X
### Serial (UART) Session
1.  Set the board in \ref BOOTMODE_UART mode and do a power cycle.

2. Enter the appropriate COM Port and select the board type. Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
    \image html com_board.png "COM Port Selection"

3.  Choose a Program to Flash:
    - Click the "Browse" button to select the program you want to flash. Serial Uniflash session supports three binary image formats - SBL, application image, XIP application image. It is not necessary to have three images in order to carry out the operation.
    - The SBL and application image flash offsets are handled internally based on the device's SBL configuration.
    \imageStyle{uart_load.png,width:80%}
    \image html uart_load.png "Load Binary Image"
    - Change the application offset from the settings. This step is only needed if you are using sbl with custom application offset.
    \imageStyle{image_offset.png,width:80%}
    \image html image_offset.png "Custom Application Offset"

4.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.
\endcond

\cond SOC_AM263PX || SOC_AM261X
### Serial (UART) Session
1.  Set the board in \ref BOOTMODE_UART mode and do a power cycle.

2. Enter the appropriate COM Port and select the board type. Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
    \image html uart_com.png "COM Port Selection"

3.  Choose a Program to Flash:
    - Click the "Browse" button to select the program you want to flash. Serial Uniflash session supports three binary image formats - SBL, application image, XIP application image. It is not necessary to have three images in order to carry out the operation.
    - The SBL and application image flash offsets are handled internally based on the device's SBL configuration.
    \imageStyle{uart_load_2.png,width:80%}
    \image html uart_load_2.png "Load Binary Image"
    - Change the application offset from the settings. This step is only needed if you are using sbl with custom application offset.
    \imageStyle{image_offset.png,width:80%}
    \image html image_offset.png "Custom Application Offset"

4.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.
\endcond

\cond SOC_AM263X || SOC_AM273X
### Serial (UART) Session
1.  Set the board in \ref BOOTMODE_UART mode and do a power cycle.

2. Enter the appropriate COM Port and select the board type. Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
    \image html uart_com.png "COM Port Selection"

3.  Choose a Program to Flash:
    - Click the "Browse" button to select the program you want to flash. Serial Uniflash session supports three binary image formats - SBL, application image, XIP application image. It is not necessary to have three images in order to carry out the operation.
    - The SBL and application image flash offsets are handled internally based on the device's SBL configuration.
    \imageStyle{uart_load_1.png,width:80%}
    \image html uart_load_1.png "Load Binary Image"
    - Change the application offset from the settings. This step is only needed if you are using sbl with custom application offset.
    \imageStyle{image_offset.png,width:80%}
    \image html image_offset.png "Custom Application Offset"

4.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.
\endcond
\endcond

## UART Uniflash {#TOOLS_FLASH_UART_UNIFLASH}

UART is used as the transport or interface to send the file to flash to the EVM.

### Tool requirements on host PC

- The tool is implemented using python and needs python version 3.x
- The tool uses additional python packages as listed below.
  - pyserial for UART access on PC
  - xmodem for the file transfer protocol
  - tqdm for progress bar when the tool is run
- Refer to the page, \ref INSTALL_PYTHON3 , to install python and the required python packages on your PC.

### Important files and folders

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/tools/boot/</td></tr>
<tr>
    <td>uart_uniflash.py
    <td>Flashing tool
</tr>
<tr>
    <td>sbl_prebuilt/@VAR_BOARD_NAME_LOWER
    <td>Pre-built bootloader images and default flash configuration files for a supported EVM
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/drivers/boot/</td></tr>
<tr>
    <td>sbl_uart_uniflash
    <td>Flashing application that is run on the EVM to receive files to flash
</tr>
\cond SOC_AM64X || SOC_AM243X
<tr>
    <td>sbl_ospi
    <td>OSPI bootloader application that needs to be flashed at offset 0x0. When in OSPI boot mode, this bootloader application
    will boot the user application file for all the CPUs
</tr>
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
<tr>
    <td>sbl_qspi
    <td>QSPI bootloader application that needs to be flashed at offset 0x0. When in QSPI boot mode, this bootloader application
    will boot the user application file for all the CPUs
</tr>
<tr>
    <td>sbl_can
    <td>CAN bootloader application that needs to be flashed at offset 0x0. When in QSPI boot mode, this bootloader application will boot the user application file for all the CPUs
</tr>
<tr>
    <td>sbl_can_uniflash
    <td>CAN Uniflash application that needs to be flashed at offset 0x0. When in QSPI boot mode, this uniflash application will wait for CAN packets to flash the user application file via CAN using the can_uniflash python script. This application has capability to boot the application as well.
</tr>
<tr>
    <td>sbl_sd
    <td>SD bootloader application that needs to be flashed at offset 0x0. When in QSPI boot mode, this bootloader application will boot the user application file from SD card for all the CPUs
</tr>
\endcond
<tr>
    <td>sbl_null
    <td>SOC init bootloader application that can be used to init the SOC when working in CCS IDE environment.
</tr>
</table>

### Basic steps to flash files {#BASIC_STEPS_TO_FLASH_FILES}

\cond SOC_AM64X || SOC_AM243X

#### Getting ready to flash

- Make sure the flashing application (`sbl_uart_uniflash`), OSPI bootloader (`sbl_ospi`), and the user application (`*.appimage`) you want to flash is built for the EVM.
  - For every supported EVM pre-built flashing application and OSPI bootloader can be found below

        {SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}

  - The flashing application and OSPI bootloader source code can be found at below path

        {SDK_INSTALL_PATH}/examples/drivers/boot

  - If you have modified the flashing or bootloader applications, make sure to rebuild these applications and note the path to the `hs_fs.tiimage` files
    that are generated as part of the build.

  - To build your application follow the steps mentioned in \ref GETTING_STARTED_BUILD to build the application you want.
    Note the path to the `*.appimage.hs_fs` file that is generated as part of the build.

- Make sure you have installed python as mention in \ref INSTALL_PYTHON3

- Make sure you have identified the UART port on the EVM as mentioned in \ref EVM_SETUP_PAGE

#### Flash configuration file

- Create a flash configuration file, using the default flash configuration file present at below as reference

        ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_ospi.cfg

- In this config file, modify the paths to the flashing application and OSPI bootloader, in case you are not using the pre-built applications

        --flash-writer={path to flash application .tiimage}
        --file={path to OSPI bootloader .tiimage} --operation=flash --flash-offset=0x0

- Edit below line to point to the user application (`.appimage.hs_fs`) file

        --file={path to your application .appimage.hs_fs file} --operation=flash --flash-offset=0x80000

\cond SOC_AM243X || SOC_AM64X
- Edit below line to point to the user application XIP image (`.appimage_xip`) file. When not using XIP mode, this file input is optional.

        --file={path to your application .appimage_xip file} --operation=flash-xip
\endcond
\cond SOC_AM64X || SOC_AM243X

#### Flash configuration file for flashing to eMMC

- Create a flash configuration file, using the default flash configuration file present at below as reference

    - For `sbl_emmc`

            ${SDK_INSTALL_PATH}/examples/drivers/boot/sbl_emmc/@VAR_BOARD_NAME_LOWER/r5fss0-0_nortos/default_sbl_emmc.cfg
\cond SOC_AM64X
    - For `sbl_emmc_linux`

            ${SDK_INSTALL_PATH}/examples/drivers/boot/sbl_emmc_linux/am64x-evm/r5fss0-0_nortos/default_sbl_emmc_linux.cfg
\endcond
- The flashing application and the eMMC bootloader needs to be specified in this file as

        --flash-writer={path to flash application .tiimage}
        --file={path to eMMC bootloader .tiimage} --operation=flash-emmc --flash-offset=0x0

- The user application (`.appimage`) file needs to be specified in the configuration file as

        --file={path to your application .appimage file} --operation=flash-emmc --flash-offset=0x800000

\endcond
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X

#### Getting ready to flash

- Make sure the flashing application (`sbl_uart_uniflash`), QSPI bootloader (`sbl_qspi`), and the user application (`*.appimage`) you want to flash is built for the EVM.
  - For every supported EVM pre-built flashing application and QSPI bootloader can be found below

        {SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}

  - The flashing application and QSPI bootloader source code can be found at below path

        {SDK_INSTALL_PATH}/examples/drivers/boot

  - If you have modified the flashing or bootloader applications, make sure to rebuild these applications and note the path to the `.tiimage` files
    that are generated as part of the build.

  - To build your application follow the steps mentioned in \ref GETTING_STARTED_BUILD to build the application you want.
    Note the path to the `*.appimage` file that is generated as part of the build.

- Make sure you have installed python as mention in \ref INSTALL_PYTHON3

- Make sure you have identified the UART port on the EVM as mentioned in \ref EVM_SETUP_PAGE

#### Flash configuration file

- Create a flash configuration file, using the default flash configuration file present at below as reference

        ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_qspi.cfg

- In this config file, modify the paths to the flashing application and QSPI bootloader, in case you are not using the pre-built applications

        --flash-writer={path to flash application .tiimage}
        --file={path to QSPI bootloader .tiimage} --operation=flash --flash-offset=0x0

- Edit below line to point to the user application (`.appimage`) file

        --file={path to your application .appimage file} --operation=flash --flash-offset=0x80000

\endcond

#### Flashing the files

- Set EVM in \ref BOOTMODE_UART and power on the EVM

- Run below python command on the Windows command prompt (`cmd.exe`) or Linux bash shell to flash the files.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p {name of your UART com port} --cfg={path to your edited config file}

- At each step in the flashing your will see success or error messages, including progress as the file is being transferred.

\cond SOC_AM243X || SOC_AM64X

- If flashing is successful, power OFF the EVM, set the EVM to \ref BOOTMODE_OSPI and power ON the EVM to run the flashed application.
\endcond
\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- If flashing is successful, power OFF the EVM, set the EVM to \ref BOOTMODE_QSPI and power ON the EVM to run the flashed application.
\endcond

- If flashing is not successful, then check the error messages and take appropriate action (See \ref TOOLS_FLASH_ERROR_MESSAGES).

#### Flash tool options

- Type below to see all the possible options with the flashing tool and also see the default .cfg file for syntax and options possible in the config file

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py --help

### Flash tool error messages and solutions {#TOOLS_FLASH_ERROR_MESSAGES}

If the flashing fails, the error message will give a hint as to why it failed.
Some common error messages, reasons and potential solutions are listed below.

<table>
<tr>
    <th>Error
    <th>Possible Reason
    <th>Solution
</tr>
<tr>
    <td> Serial port not found or not accessible
    <td> Wrong UART port passed as argument or the UART port is open in some other terminal application.
    <td> Check the UART port, UART ports are named as `COM1`, `COM2`, and so on in Windows and as `/dev/ttyUSB0`, `/dev/ttyUSB1`, and so on in Linux.
         Also make sure to close all open UART terminals and try again.
</tr>
<tr>
    <td> No response or error response from EVM.
    <td> Either EVM is not in power-ON state or EVM is not setup in UART boot mode.
    <td> Use Ctrl-C to terminate the script if it is stuck. Check UART boot mode switch setting, check power to EVM, power-cycle EVM and try again
</tr>
<tr>
    <td> Flashing failed error message
    <td> This should not happen ideally on TI EVMs. On custom EVM this could happen if there is some issue in the flash driver on the EVM.
    <td> Power cycle EVM and try again. If the problem still does not go away, then likely
    the flash on the EVM has gone bad. Try other SOC initialization options and check the flash driver via CCS IDE debug.
</tr>
<tr>
    <td> Config file parsing error
    <td> Wrong config file passed or input files not found.
    <td> Check the message that is printed, and edit the config file to fix the parsing errors.
        Make sure to specify file paths with forward slash `/`, including in Windows.
        The default config file should not have any errors though.
</tr>
<tr>
    <td> Python not found error or python packages not not found error
    <td> Python or python packages needed for this script are not installed
    <td> Follow steps mentioned in \ref INSTALL_PYTHON3 to install python and related packages
</tr>
<tr>
    <td> Parsing config file error
    <td> SBL binaries are missing from the prebuilt folder
    <td> Build sbl using below command: \code gmake -s sbl DEVICE=am243x \endcode
</tr>
</table>

### Detailed sequence of steps that happen when flashing files

\note This section has more detailed sequence of steps that happen underneath the tools and on the EVM for reference.

The detailed sequence of steps that happen when flashing files is listed below, refer to the \ref EVM_SETUP_PAGE page to see how to setup the EVM in different boot modes that are needed for this sequence of steps.

- Set EVM in UART boot mode and power it on, the SOC ROM bootloader waits to receive a file using the UART+XMODEM protocol.
- PC sends the flashing application file (`sbl_uart_uniflash.release.tiimage`) via the flashing tool using UART+XMODEM protocol underneath.
- The ROM bootloader, boots the flashing application
- The flashing application now initializes the flash on the EVM and waits for additional commands using UART+XMODEM protocol
- The PC tool can now send one or more of below commands with the file data, one after the other, until it is done.
  - Flash a file at a given offset in the flash
  - Verify a previously flashed file at a given offset in the flash
  - Erase a region of flash memory
- The flashing application as such does not care what the file contains, it will simply flash it at the user specified location.
\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- However typically one needs to at least send the below files to flash
  - Send a QSPI flash bootloader application and flash it at offset 0x0 (`sbl_qspi.release.tiimage`). If the QSPI bootloader is
    already flashed previously then this step can be skipped.
  - Send your application image multi-core image and flash it at offset 0x80000 (`*.appimage`).
    The offset 0x80000 is the offset that is specified in the QSPI bootloader and when the EVM boots in QSPI mode, it
    will attempt to find a application at this location.
- After flashing is done, power OFF the EVM
- Set EVM in QSPI boot mode and power ON the EVM.
  - The ROM bootloader will now boot the QSPI bootloader by reading offset 0x0
  - And the QSPI bootloader will boot the application by reading from offset 0x80000.
- The initial flashing application and the subsequent commands to send and flash the QSPI bootloader and application files are all specified
  in a single configuration file which is provided as input to the tool.
\endcond
\cond SOC_AM243X || SOC_AM64X
- However typically one needs to at least send the below files to flash
  - Send a OSPI flash bootloader application and flash it at offset 0x0 (`sbl_ospi.release.hs_fs.tiimage`). If the OSPI bootloader is
    already flashed previously then this step can be skipped.
  - Send your application image multi-core image and flash it at offset 0x80000 (`*.appimage.hs_fs`).
    The offset 0x80000 is the offset that is specified in the OSPI bootloader and when the EVM boots in OSPI mode, it
    will attempt to find a application at this location.
- After flashing is done, power OFF the EVM
- Set EVM in OSPI boot mode and power ON the EVM.
  - The ROM bootloader will now boot the OSPI bootloader by reading offset 0x0
  - And the OSPI bootloader will boot the application by reading from offset 0x80000.
- The initial flashing application and the subsequent commands to send and flash the OSPI bootloader and application files are all specified
  in a single configuration file which is provided as input to the tool.
\endcond

### GUI for UART Uniflash (Experimental) {#TOOLS_UART_UNIFLASH_GUI}

UART Uniflash GUI is a GUI wrapper around the UART Uniflash tool already present (`uart_uniflash.py`). This is a strictly experimental feature with minimal testing from TI side. Can be used if GUI is more comfortable. Since most of the CLI tool is used underneath, it is the same functionality wise

#### Pre-Requisites for UART Uniflash GUI

This GUI is built on top of the python based UART Uniflash CLI tool already mentioned, and specifically based on the PyQt5 binding of the QT5 framework. So it is expected that python3 and other dependencies (xmodem, pyserial etc) are already installed. In addition to this, one also needs to install the PyQt5 python library for the GUI to work.

- In windows, you can install PyQt5 by doing below:

  `python -m pip install pyqt5`

- In Ubuntu (or other Debian based distros) you can install PyQt5 by doing below:

  `sudo apt install python3-pyqt5`

### Using the UART Uniflash GUI

The UART Uniflash GUI can be used to flash files arbitrarily into the device flash

\imageStyle{uniflash_gui_manual_config_sport.png,width:40%}
\image html uniflash_gui_manual_config_sport.png "UART Uniflash GUI : Serial Port Selection"

At the top there is a drop down to select the UART COM port which will be used for flashing. Please connect the target to the EVM before running the GUI so that the serial port we are interested will show up here.

- For choosing what to flash and how to flash there are two high level choices in UART Uniflash GUI:
  - Manual Config
  - From File

    \imageStyle{uniflash_gui_manual_config.png, width:50%}
    \imageStyle{uniflash_gui_file_config.png, width:50%}

<table style="border: 0 px">
    <tr>
        <td> \image html uniflash_gui_manual_config.png "UART Uniflash GUI : Manual Config based flashing" </td>
        <td> \image html uniflash_gui_file_config.png "UART Uniflash GUI : Config file based flashing" </td>
    </tr>
</table>

- **Manual Config** : Manual configuration of the files to be flashed. There will be drop down file browse options to select the various files you will need to flash/send to the target. It provides options / slots to select below:

  - **Flash writer binary** : This is the sbl_uart_uniflash binary. This needs to be send first for the ROM to receive and boot. Once this boots up you can send any number of files arbitrarily for flashing.

  - **Bootloader binary** : It is assumed that the eventual goal of the flashing process is to boot your application from the flash device. For this a bootloader capable of reading an image from flash device needs to be flashed at offset 0 (generally) of the flash. This would be the `sbl_ospi` or `sbl_qspi`. Although this is no different than flashing any other file to a particular offset, we have decided to keep it a separate option for better clarity. Although the offset is almost always 0, we have provided an offset edit box as well if there is any change whatsoever.

  - **Appimage binary** : You can select the application image to be flashed from this slot. SDK convention is to flash at a 512 KB offset (0x80000). This can be changed, but keep in mind that the bootloader booting this application should be aware of this offset as well. It is a configurable option in the Sysconfig of the bootloader.
\cond SOC_AM243X || SOC_AM64X
  - **Appimage XIP binary** : You can select the XIP component to your application from this slot. These files will be of the format (`*.appimage_xip`). These files already contain details as to where these need to be flashed, so no need to provide any offset in this case.
\endcond
  - **Custom data** : This slot can be used to flash any custom data file at an arbitrary offset. Don't forget to provide the offset

  - **Flash PHY tuning data** : In certain NOR SPI drivers, for tuning the PHY, known data needs to be present in the flash. This is a fixed size (usually 128 bytes) array written to the last block of the flash. This check box needs to be ticked if you're flashing for the first time to the target so that this known data can be written to the flash.

  \imageStyle{uniflash_gui_manual_config_flash_phy_tuning.png,width:40%}
  \image html uniflash_gui_manual_config_flash_phy_tuning.png "UART Uniflash GUI : Saving the manual configuration"

  - There is an option to save the settings you selected manually as a config file using the **Save CFG** button towards the bottom right of the Manual Config group.

\imageStyle{uniflash_gui_manual_config_save_cfg.png,width:40%}
\image html uniflash_gui_manual_config_save_cfg.png "UART Uniflash GUI : Saving the manual configuration"

- **From File** : Flash using a configuration file (`*.cfg` extension) specifying what needs to be flashed. This is a convenient option if the files to be flashed are pretty much the same for every try. In fact even in the manual config case, a cfg file is created internally and used for flashing. This `cfg` follows the same format used by the CLI script.

After selecting the config options and files, just press the **FLASH** button to start the flashing. A progress bar will show the flashing progress.

There is also a log area which will show detailed logs in addition to the pop up messages.

\imageStyle{uniflash_gui_manual_config_log_area.png,width:40%}
\image html uniflash_gui_manual_config_log_area.png "UART Uniflash GUI : LOGS"


### Important Notes and Common Pitfalls in UART Uniflash GUI

- Make sure that the target device is powered ON and is in UART boot mode before attempting to flash
- Make sure that the correct COM port is selected from the drop down. If the wrong port is selected, the flasher might take time to exit out as it will wait for the device to send the XMODEM control character.
- If the GUI seems to hang, re-check the selected COM port and make sure the EVM is **powered ON** and in **UART boot mode**
- The GUI is based on the `uart_uniflash.py` CLI script, and reuses a lot of objects and functions, so core functionality is the same among both.
- In manual config case, if a drop down is non blank it is assumed that the file provided there needs to be flashed. So it will be picked up and used when `FLASH` button is clicked. If this is not required, make sure to delete it and keep it blank, the drop down is editable. This is true for the config file drop down as well, but the mishap is more probable in manual config case.


\cond SOC_AM64X || SOC_AM243X

## USB DFU Uniflash{#TOOLS_FLASH_DFU_UNIFLASH}

- **usb_dfu_uniflash.py** is a tool which is used for flashing the images onto the flash memory using USB DFU protocol.
It uses \ref INSTALL_DFU_UTIL tool to underneath to send binaries via USB.
- This tool is used in conjuction with \ref EXAMPLES_DRIVERS_SBL_DFU_UNIFLASH which is a flash-writer application.

- refer \ref EXAMPLES_USB_DFU for more information on DFU.

### USB DFU bootflow using dfu based flash-writer.

- Following diagram explains boot flow using USB DFU and SBL OSPI.
- Its a three step process.
	1. Put the device into DFU BOOT mode refer \ref BOOTMODE_DFU. After this ROM Bootoader will accept a valid SBL image
	via USB and boot it. In this case we will boot \ref EXAMPLES_DRIVERS_SBL_DFU_UNIFLASH which is a flash-writer binary.
	2. Once flash-writer is booted a new USB DFU capable device will be enumerated. After this using **usb_dfu_uniflash.py** tool
	we will send **SBL_OSPI** or **SBL_QSPI** along with multicore appimage. Flash-writer SBL will flash the received files onto
	flash memory.
	3. Change the boot mode to \ref BOOTMODE_OSPI and power cycle the board. First **SBL_OSPI** or **SBL_QSPI** will be booted from flash
	and later it is responsible to boot the multicore appimages.


  \imageStyle{dfu_flash_bootflow.png,width:40%}
  \image html dfu_flash_bootflow.png


### Tool requirements on host PC

- The tool is implemented using python and needs python version 3.x
- Refer to the page, \ref INSTALL_PYTHON3 , to install python and the required python packages on your PC.
- It uses **dfu-util** tool to execute dfu commands from the host PC.
- Refer to the page \ref INSTALL_DFU_UTIL , to install **dfu-util** tool.

### Important files and folders

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/tools/boot/</td></tr>
<tr>
    <td>usb_dfu_uniflash.py
    <td>Flashing tool
</tr>
<tr>
    <td>sbl_prebuilt/@VAR_BOARD_NAME_LOWER
    <td>Pre-built bootloader images and default flash configuration files for a supported EVM
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/drivers/boot/</td></tr>
<tr>
    <td>sbl_dfu_uniflash
    <td>Flashing application that is run on the EVM to receive files to flash
</tr>
\cond SOC_AM64X || SOC_AM243X
<tr>
    <td>sbl_ospi
    <td>OSPI bootloader application that needs to be flashed at offset 0x0. When in OSPI boot mode, this bootloader application
    will boot the user application file for all the CPUs
</tr>
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
<tr>
    <td>sbl_qspi
    <td>QSPI bootloader application that needs to be flashed at offset 0x0. When in QSPI boot mode, this bootloader application
    will boot the user application file for all the CPUs
</tr>
<tr>
    <td>sbl_can
    <td>CAN bootloader application that needs to be flashed at offset 0x0. When in QSPI boot mode, this bootloader application will boot the user application file for all the CPUs
</tr>
\endcond
<tr>
    <td>sbl_null
    <td>SOC init bootloader application that can be used to init the SOC when working in CCS IDE environment.
</tr>
</table>

### Basic steps to flash files {#BASIC_STEPS_TO_FLASH_FILES_DFU}

 - The flashing steps are same as refer \ref BASIC_STEPS_TO_FLASH_FILES only difference is instead of using ~~uart_uniflash.py~~ use **usb_dfu_uniflash.py**.


#### Flashing the files

- Set EVM in \ref BOOTMODE_DFU and power on the EVM

- Run below python command on the Windows command prompt (`cmd.exe`) or Linux bash shell to flash the files.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python usb_dfu_uniflash.py --cfg={path to your edited config file}

- At each step in the flashing your will see success or error messages, including progress as the file is being transferred.

\cond SOC_AM243X || SOC_AM64X

- If flashing is successful, power OFF the EVM, set the EVM to \ref BOOTMODE_OSPI and power ON the EVM to run the flashed application.
\endcond
\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- If flashing is successful, power OFF the EVM, set the EVM to \ref BOOTMODE_QSPI and power ON the EVM to run the flashed application.
\endcond

- If flashing is not successful, then DFU device i.e EVM must have send error status in **GET_STATUS** phase of **SETUP** transfer. This error condition
will be detected by **dfu-util** tool and will be displayed on console.

#### Flash tool options

- Type below to see all the possible options with the flashing tool and also see the default .cfg file for syntax and options possible in the config file

        cd ${SDK_INSTALL_PATH}/tools/boot
        python usb_dfu_uniflash.py --help

### Detailed sequence of steps that happen when flashing files using usb_dfu_uniflash tool

\note This section has more detailed sequence of steps that happen underneath the tools and on the EVM for reference.

The detailed sequence of steps that happen when flashing files is listed below, refer to the \ref EVM_SETUP_PAGE page to see how to setup the EVM in different boot modes that are needed for this sequence of steps.

- Set EVM in DFU boot mode and power it on, the SOC ROM bootloader waits to receive a file using the USB2.0 DFU protocol.
- PC sends the flashing application file (`sbl_dfu_uniflash.release.hs_fs.tiimage`) via the flashing tool using USB2.0 DFU protocol underneath.
- The ROM bootloader, boots the flashing application
- The flashing application now initializes the flash on the EVM and waits for additional commands using USB2.0 DFU protocol
- The PC tool can now send one or more of below commands with the file data, one after the other, until it is done.
  - Flash a file at a given offset in the flash
  - Verify a previously flashed file at a given offset in the flash
  - Erase a region of flash memory
- The flashing application as such does not care what the file contains, it will simply flash it at the user specified location.
\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM261X
- However typically one needs to at least send the below files to flash
  - Send a QSPI flash bootloader application and flash it at offset 0x0 (`sbl_qspi.release.tiimage`). If the QSPI bootloader is
    already flashed previously then this step can be skipped.
  - Send your application image multi-core image and flash it at offset 0x80000 (`*.appimage`).
    The offset 0x80000 is the offset that is specified in the QSPI bootloader and when the EVM boots in QSPI mode, it
    will attempt to find a application at this location.
- After flashing is done, power OFF the EVM
- Set EVM in QSPI boot mode and power ON the EVM.
  - The ROM bootloader will now boot the QSPI bootloader by reading offset 0x0
  - And the QSPI bootloader will boot the application by reading from offset 0x80000.
- The initial flashing application and the subsequent commands to send and flash the QSPI bootloader and application files are all specified
  in a single configuration file which is provided as input to the tool.
\endcond
\cond SOC_AM243X || SOC_AM64X
- However typically one needs to at least send the below files to flash
  - Send a OSPI flash bootloader application and flash it at offset 0x0 (`sbl_ospi.release.hs_fs.tiimage`). If the OSPI bootloader is
    already flashed previously then this step can be skipped.
  - Send your application image multi-core image and flash it at offset 0x80000 (`*.appimage`).
    The offset 0x80000 is the offset that is specified in the OSPI bootloader and when the EVM boots in OSPI mode, it
    will attempt to find a application at this location.
- After flashing is done, power OFF the EVM
- Set EVM in OSPI boot mode and power ON the EVM.
  - The ROM bootloader will now boot the OSPI bootloader by reading offset 0x0
  - And the OSPI bootloader will boot the application by reading from offset 0x80000.
- The initial flashing application and the subsequent commands to send and flash the OSPI bootloader and application files are all specified
  in a single configuration file which is provided as input to the tool.
\endcond

- see also \ref EXAMPLES_DRIVERS_SBL_DFU_UNIFLASH

\endcond

## JTAG Uniflash {#TOOLS_FLASH_JTAG_UNIFLASH}

JTAG is used as the transport or interface to send the file to flash to the EVM.

### Important files and folders

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/drivers/boot/</td></tr>
<tr>
    <td>sbl_jtag_uniflash
    <td> Flash-writer example which uses JTAG to write files or erase flash
</tr>
</table>

### Basic steps to flash files

Refer the example \ref EXAMPLES_DRIVERS_SBL_JTAG_UNIFLASH

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AM261X
## CAN Uniflash {#TOOLS_FLASH_CAN_UNIFLASH}

CAN is used as the transport or interface to send the file to flash to the EVM.
### Important files and folders

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/drivers/boot/</td></tr>
<tr>
    <td>sbl_can_uniflash
    <td> Flash-writer example which uses CAN to write files
</tr>
</table>

### Basic steps to flash files

Refer the example \ref EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH
\endcond

\cond SOC_AM64X || SOC_AM243X

### See also

- \ref EXAMPLES_USB_DFU
\endcond

\cond SOC_AWR294X

## ENET Uniflash {#TOOLS_FLASH_ENET_UNIFLASH}

UDP over ethernet is used as the transport or interface to send the file to flash to the EVM.

### Tool requirements on host PC

- The tool is implemented using python and needs python version 3.x
- The tool uses additional python packages as listed below.
  - tqdm for progress bar when the tool is run
- Refer to the page, \ref INSTALL_PYTHON3 , to install python and the required python packages on your PC.

### Important files and folders

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/tools/boot/</td></tr>
<tr>
    <td>enet_uniflash.py
    <td>Flashing tool
</tr>
<tr>
    <td>sbl_prebuilt/@VAR_BOARD_NAME_LOWER
    <td>Pre-built bootloader images and default flash configuration files for a supported EVM
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/drivers/boot/</td></tr>
<tr>
    <td>sbl_qspi_enet
    <td>QSPI bootloader application and Flashing application that is run on the EVM to receive files to flash. To be flashed at offset 0x0. When in QSPI boot mode, this bootloader application
    will boot the user application file for all the CPUs
</tr>
</table>

### Basic steps to flash files over ethernet {#BASIC_STEPS_TO_FLASH_FILES_OVER_ENET}

#### Getting ready to flash

- Make sure the QSPI Ethernet bootloader (`sbl_qspi_enet`) has been flashed using the steps provided in \ref BASIC_STEPS_TO_FLASH_FILES.
- Make sure the user application (`*.appimage`) you want to flash over ethernet is built for the EVM.

#### Flash configuration file

- Create a flash configuration file, using the default flash configuration file present

        ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_qspi.cfg

- In this config file, modify the paths to the application to be flashed only.

   Remove or comment out the flash-writer path

        --flash-writer={path to flash application .tiimage}

   Remove or comment out the bootloader path

        --file={path to QSPI bootloader .tiimage} --operation=flash --flash-offset=0x0

   Edit below line to point to the user application (`.appimage`) file

        --file={path to your application .appimage file} --operation=flash --flash-offset=0xA0000

#### Flashing the files over ethernet

- Run below python command on the Windows command prompt (`cmd.exe`) or Linux bash shell to flash the files.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python enet_uniflash.py --cfg={path to your edited config file}

- When the python script starts, it will display the message "Starting Linkup ...".

  Based on the MACRO ENETSBL_TRANSFER_START_MODE set in sbl_enet.h of the sbl_qspi_enet bootloader, do either of the below when the above message is observed,
    - If in ENETSBL_TIMER_MODE, Press the reset button SW1 on the EVM.
    - If in ENETSBL_BUTTON_MODE, Press the reset button SW1 while holding down the user switch SW2.

- After flashing is successful, the flashed application code will be automatically run after the SBL completes. See \ref SBL_QSPI_ENET_OUTPUT_SAMPLE for a sample of the python script output after a successful flash.

#### Flash tool options

- Type below to see all the possible options with the flashing tool and also see the default .cfg file for syntax and options possible in the config file

        cd ${SDK_INSTALL_PATH}/tools/boot
        python enet_uniflash.py --help


\endcond