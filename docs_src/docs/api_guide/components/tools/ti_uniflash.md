# TI Uniflash Tool {#TI_UNIFLASH_TOOL}

### Introduction

The TI UniFlash Tool is a software utility designed for programming on-chip and external flash memory on TI microcontrollers and wireless connectivity devices. Sitara MCU devices now supports TI Uniflash tool for loading or flashing images into the Target. Uniflash offers both graphical and command-line interfaces. This document provides a comprehensive guide on how to use UniFlash for flashing Sitara MCU devices.

## Download and Install Uniflash

\note The steps on this page need to be done once on a given host machine

- The TI Uniflash download home page is, https://www.ti.com/tool/UNIFLASH
- Download Uniflash @VAR_UNIFLASH_VERSION
- Install at below path,
  - Windows, C:/ti
  - Linux, ${HOME}/ti
- Launch the Uniflash

### Launching a Session

To perform flash operations on your device, you must initiate a flash session tailored to the specific device. Here are the multiple ways to start a new session:

-  Board or Device/Connection Combination
    - When connecting a board, UniFlash automatically chooses the appropriate connection for you.
    \image html detected_device.png "Detected Devices"
    - For some devices, the device type is not recognized by uniflash. In such cases, you can manually select your device by choosing **Sitara MCU** filter and selecting your device and connection type
    \image html new_config.png "New Configuration"
-   Loading a Previously Saved Session
    - Loading a saved session restores your prior selections, including device, connection, and settings.
-   Selecting a CCXML File Created by CCS
    - This option is useful when you already have the configuration settings in the CCXML file.
    \image html existing_config.png "New Configuration"

After setting up one of the above session launch type, Click the Start button to launch the new session.

### Programming the Device

\note Uniflash utilizes GEL files to initialize the target. Please put the board in \ref BOOTMODE_NOBOOT mode and power cycle the board prior to loading.

1.  Choose a Program to Flash:
    -   Click the "Browse" button to select the program you want to flash.
    -   If you're loading a ELF File(.out), the address and informations are obtained by the flash loader and the program gets loaded into the target. The program will run once after it loads if the **Run Target After Program Load/Flash Operation** is enabled.
    \imageStyle{load_elf.png,width:90%}
    \image html load_elf.png "Load ELF"
    -   Check the "Binary" checkbox if you are loading a binary file. You must provide a start address for binary files. UniFlash currently does not support flash offsets, so the full address is required.
    \imageStyle{load_image.png,width:90%}
    \image html load_image.png "Load Binary Image"
2.  Flash Address Table:
    - The table below shows the flash addresses accepted by the ROM/SBL to load programs onto the target:
| Program Type | Start Address |
| --- | --- |
| SBL | 0x60000000 |
| Application | 0x60080000 |

3.  Initiating Programming:
    - After clicking "Load Image," UniFlash starts the programming process, and the console displays a log of each operation. **[SUCCESS] Program Load completed successfully** will get printed in the console, if the program loads into the target successfully.

### Existing Limitations

  - Load Multiple Images was not currently Supported. If you load multiple flash images, the second will be appended to the first image’s end.
  - Verify Image button will not verify the image as the images get verified in the load operation itself.

### Supported File Formats

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

### Memory Browser and Memory Export

The Memory View in UniFlash allows you to browse the target memory quickly. Please note that this view is read-only.

To export a range of memory, follow these steps:

-   Use the Memory View to select the desired memory range.
-   Click the "Export" button.
-   Choose either a binary (.BIN) file or a COFF (.out) file for export. The COFF format enables the export of multiple memory sections.

### Additional Information

This documentation provides an overview of the TI UniFlash Tool's key features and usage for Sitara MCU devices. Refer to the tool's official documentation for more detailed instructions and troubleshooting information. , \htmllink{https://software-dl.ti.com/ccs/esd/uniflash/docs/v8_4/uniflash_quick_start_guide.html, TI Uniflash Quick Start Guide}