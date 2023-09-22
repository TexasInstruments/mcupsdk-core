# TI Uniflash Tool {#TI_UNIFLASH_TOOL}

## Introduction

TI UniFlash Tool is a software utility designed for programming "On-chip" and "External flash" memory on TI microcontrollers and wireless connectivity devices. Sitara MCU devices now support TI Uniflash tool for loading or flashing images into the Target. Uniflash offers both graphical and command-line interfaces. This document provides a comprehensive guide on how to use UniFlash for flashing Sitara MCU devices.

## Download and Install Uniflash

\note The steps on this page need to be done once on a given host machine

- The TI Uniflash download home page is, https://www.ti.com/tool/UNIFLASH
- At present, the TI Uniflash Tool is only supported by AM263P.
- Please contact TI representative to get the installer and try this tool.
- Install at below path,
  - Windows, C:/ti
  - Linux, ${HOME}/ti
- Launch the Uniflash Application

## Launching a Session

To perform flash operations on your device, you must initiate a flash session tailored to the specific device. Here are multiple ways to start a new session:

-  Auto Detect
    - When connecting a board, UniFlash automatically chooses the appropriate connection for you.
    Auto-detect uses JTAG serial to recognize the device. If you start the session using auto detect, JTAG connection is automatically selected for the session.
    \image html detected_device.png "Detected Devices"
-  Board or Device/Connection Combination
    - For some devices, the device type is not recognized by uniflash. In such cases, you can manually select your device by choosing **Sitara MCU** filter and selecting your device and connection type.
    - JTAG connection type is selected by default.
    \imageStyle{new_config.png,width:80%}
    \image html new_config.png "New JTAG Configuration"
    - Use the device name ending with (Serial) to initiate a serial (UART) flashing session. In this case, the serial connection is automatically selected.
    \imageStyle{serial_config.png,width:80%}
    \image html serial_config.png "New Serial Configuration"
-   Loading a Previously Saved Session
    - Loading a saved session restores your prior selections, including device, connection, and settings.
-   Selecting a CCXML File Created by CCS/Uniflash
    - This option is useful when you already have the configuration settings in the CCXML file.
    \image html existing_config.png "New CCXML Configuration"

After setting up one of the above session launch type, Click the Start button to launch the new session.

## Programming the Device

### JTAG Session

1.  Set the board in \ref BOOTMODE_NOBOOT mode and do a power cycle prior to loading.

2.  Choose a Program to Flash:
    - Click the "Browse" button to select the program you want to flash.
    - If you're loading a ELF File(.out), the address and informations are obtained by the flash loader and the program gets loaded into the target. The program will run once after it loads if the **Run Target After Program Load/Flash Operation** is enabled.
    \imageStyle{load_elf.png,width:90%}
    \image html load_elf.png "Load ELF"
    -   Check the "Binary" checkbox if you are loading a binary file. You must provide a start address for binary files. UniFlash currently does not support flash offsets, so the full address is required.
    \imageStyle{load_image.png,width:90%}
    \image html load_image.png "Load Binary Image"

3.  Flash Address Table:
    - The table below shows the flash addresses accepted by the ROM/SBL to load programs onto the target:
| Program     | Start Address |
|-------------|---------------|
| SBL         | 0x60000000    |
| Application | 0x60080000    |

4.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.

### Serial (UART) Session


1.  Set the board in \ref BOOTMODE_UART mode and do a power cycle.

2. Enter the appropriate COM Port. Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
    \image html uart_com.png "COM Port Selection"

3.  Choose a Program to Flash:
    - Click the "Browse" button to select the program you want to flash. Serial Uniflash session supports three binary image formats - SBL, application image, XIP application image. It is not necessary to have three images in order to carry out the operation.
    - The SBL and application image flash offsets are handled internally based on the device's SBL configuration.
    \imageStyle{uart_load.png,width:90%}
    \image html uart_load.png "Load Binary Image"

4.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.

## Existing Limitations

  - Loading Multiple Images is currently not Supported in JTAG flashing. If you load multiple flash images, the second will be appended to the end of first image.

## Supported File Formats

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

## Memory Browser and Memory Export

The Memory View in UniFlash allows you to browse the target memory quickly. Please note that this view is read-only.

To export a range of memory, follow these steps:

-   Use the Memory View to select the desired memory range.
-   Click the "Export" button.
-   Choose either a binary (.BIN) file or a COFF (.out) file for export. The COFF format enables the export of multiple memory sections.

## Additional Information

This documentation provides an overview of the TI UniFlash Tool's key features and usage for Sitara MCU devices. Refer to the tool's official documentation for more detailed instructions and troubleshooting information. \htmllink{https://software-dl.ti.com/ccs/esd/uniflash/docs/v8_4/uniflash_quick_start_guide.html, TI Uniflash Quick Start Guide}