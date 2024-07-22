# TI Uniflash Tool {#TI_UNIFLASH_TOOL}

[TOC]

## Introduction

TI UniFlash Tool is a software utility designed for programming "On-chip" and "External flash" memory on TI microcontrollers and wireless connectivity devices. Sitara MCU devices now support TI Uniflash tool for loading or flashing images into the Target. Uniflash offers both graphical and command-line interfaces. This document provides a comprehensive guide on how to use UniFlash for flashing Sitara MCU devices.

## Download and Install Uniflash {#INSTALL_UNIFLASH}

\note The steps on this page need to be done once on a given host machine

- The TI Uniflash download home page is, https://www.ti.com/tool/UNIFLASH
- Download the latest version @VAR_UNIFLASH_VERSION
- Install at below path,
  - Windows, C:/ti
  - Linux, ${HOME}/ti
- Launch the Uniflash Application

## Launching a Session{#LAUNCHING_SESSION}

To perform flash operations on your device, you must initiate a flash session tailored to the specific device. Here are multiple ways to start a new session:

-  Auto Detect
    - When connecting a board, UniFlash automatically chooses the appropriate connection for you.
    Auto-detect uses JTAG serial to recognize the device. If you start the session using auto detect, JTAG connection is automatically selected for the session.
    \image html detected_device.png "Detected Devices"
-  Board or Device/Connection Combination
    - You can manually select your device by choosing **Sitara MCU** filter and selecting your device and connection type.
    - JTAG connection type is selected by default if you choose your device with on-chip type.
    \imageStyle{new_config.png,width:80%}
    \image html new_config.png "New JTAG Configuration"
    - Use the device name ending with (Serial) to initiate a serial (UART) flashing session. In this case, the serial connection is automatically selected.
    \imageStyle{serial_config.png,width:80%}
    \image html serial_config.png "New Serial Configuration"
    - Select the correct part number to use a different connection. If you select the device by board name, the default connection type will be automatically selected for you.
    \imageStyle{connection.png,width:80%}
    \image html connection.png "Connection Type"
-   Loading a Previously Saved Session
    - Loading a saved session restores your prior selections, including device, connection, and settings. If you haven't saved a session in Uniflash before, this tab will not be visible.
    \imageStyle{recent_session.png,width:80%}
    \image html recent_session.png "Recent Session Selection"
-   Selecting a CCXML File Created by CCS/Uniflash
    - This option is useful when you already have the configuration settings in the CCXML file.
    \image html existing_config.png "New CCXML Configuration"

After setting up one of the above session launch type, Click the Start button to launch the new session.

\cond SOC_AM273X
\note JTAG flashing support in AM273x was not tested due to XDS emulator failure while connecting R5 core 0 with \ref BOOTMODE_NOBOOT Boot Mode. It is recommended to use Serial flashing for AM273x.
\endcond

## Programming the Device{#PROGRAM_THE_DEVICE}

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
\note The default flasher included in the uniflash package is incompatible with the E2 revision of the board. Please refer to the \ref TI_UNIFLASH_TROUBLESHOOTING for instructions on how to make it work on the E2 board.
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

## Custom Flash Support{#CUSTOM_FLASH}
\cond SOC_AM263PX || SOC_AM243X || SOC_AM263X || SOC_AM261X
 - JTAG
     - Please use \ref TOOLS_UNIFLASH_CUSTOM_FLASHER guide to build the flasher app and input the path of the .out file in the tool. The custom flasher input field is found at Settings & Utilities page.
    \imageStyle{jtag_flasher_path.png,width:70%}
    \image html jtag_flasher_path.png "JTAG Custom Flash writer"
\endcond
 - Serial
     - Please build the existing \ref EXAMPLES_DRIVERS_SBL_UART_UNIFLASH application using your custom flash configuration. Then, input the file path for the .tiimage in the tool. The custom flasher input field is found at Settings & Utilities page.
    \imageStyle{uart_flasher_path.png,width:70%}
    \image html uart_flasher_path.png "UART Custom Flash writer"

## Existing Limitations{#LIMITATIONS}
  - If the flashing operation hangs in the serial session, the Cancel button will be unresponsive. To resolve the issue, close uniflash, relaunch it, and ensure that your board selection and flash configurations are correct.

## Troubleshooting {#TI_UNIFLASH_TROUBLESHOOTING}
  - If the default flashing algorithm that comes with Uniflash is not working for you, you can try using the one that comes with the SDK.
  - You can find the prebuilt flashers at the following location:
    \code
    <MCU_SDK_ROOT>\tools\flasher\prebuilt
    \endcode
  - To use the flasher packaged with the SDK, you can either enter the path in the custom flasher field or replace the Uniflash flasher directly from the below path:
    \code
    <uniflash_root>deskdb\content\TICloudAgent\win\ccs_base\sitara_mcu\flasher
    \endcode

## Supported File Formats{#SUPPORTED_FORMATS}

When loading a file to flash or RAM, UniFlash supports the following formats:

-   TI COFF
-   TI ELF
-   Intel Hex
-   Motorola S-Record
-   Tektronix Hex
-   TI-TXT
-   Binary (.appimage, .appimage.hs, .appimage.hs_fs, .tiimage)

When saving memory to a file, UniFlash supports the following formats:

-   TI COFF
-   Binary

## Memory Browser and Memory Export{#MEMORY_BROWSER}

The Memory View in UniFlash allows you to browse the target memory quickly. Please note that this view is read-only.

To export a range of memory, follow these steps:

-   Use the Memory View to select the desired memory range.
-   Click the "Export" button.
-   Choose either a binary (.BIN) file or a COFF (.out) file for export. The COFF format enables the export of multiple memory sections.

## Additional Information

This documentation provides an overview of the TI UniFlash Tool's key features and usage for Sitara MCU devices. Refer to the tool's official documentation for more detailed instructions and troubleshooting information. \htmllink{https://software-dl.ti.com/ccs/esd/uniflash/docs/v8_4/uniflash_quick_start_guide.html, TI Uniflash Quick Start Guide}