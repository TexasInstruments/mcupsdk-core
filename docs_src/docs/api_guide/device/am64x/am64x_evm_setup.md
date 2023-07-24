#  EVM Setup {#EVM_SETUP_PAGE}

[TOC]

\cond SOC_AM64X
\note Refer to EVM page for more details on the EVM, https://www.ti.com/tool/TMDS64GPEVM <br />
Refer to SK page for more details on the AM64X-SK, https://www.ti.com/tool/SK-AM64
\endcond
\cond SOC_NONE
\note Refer to EVM page for more details on the EVM, https://www.ti.com/tool/TMDS243GPEVM
\endcond

\attention This document contains the details for @VAR_BOARD_NAME and @VAR_SK_BOARD_NAME board setup under each section.
            User should select the right board setup in the sections.

## Cable Connections {#EVM_CABLES}
### AM64X-EVM
- The figure below shows some important cable connections, ports and switches.
  - Take note of the location of the "BOOTMODE" switch, this is used to
    switch between different boot modes like OSPI, UART, SD, NOBOOT mode

  \imageStyle{evm_overview.png,width:50%}
  \image html evm_overview.png "@VAR_BOARD_NAME"

### AM64X-SK
- The figure below shows some important cable connections, ports and switches.
  - Take note of the location of the "BOOTMODE" switch, this is used to
    switch between different boot modes like OSPI, UART, SD, NOBOOT mode

  \imageStyle{sk_overview.png,width:60%}
  \image html sk_overview.png "@VAR_SK_BOARD_NAME"

## Device Type Identification {#DEVICE_TYPE}
 - The MCU+SDK supports two device types: HS-FS (High Security - Field Securable) and GP (General Purpose). The HS-FS is the main device type and the support for GP device is deprecated. Please use \ref SOC_ID_PARSER to identify the device type.

 \note It is recommended to know the device type before loading or flashing the binary into the target.

## Setup UART Terminal {#CCS_UART_TERMINAL}
### AM64X-EVM
- Many examples use a standard UART terminal to log the output from the examples.
  You can use any UART terminal program for the same. Below steps show how to setup
  the UART terminal from CCS.

- First identify the UART port as enumerated on the host machine.

  - Make sure that the EVM and UART cable connected as shown in \ref EVM_CABLES

  - In windows, you can use the "Device Manager" to see the detected UART ports
    - Search "Device Manager" in Windows Search Box in the Windows taskbar.

  - If dont see any USB serial ports listed in "Device Manager" under "Ports (COM & LPT)",
    then make sure you have installed the UART to USB driver from FTDI, https://www.ftdichip.com/FTDrivers.htm.

    \imageStyle{ccs_uart_identify.png,width:30%}
    \image html ccs_uart_identify.png "Identify UART Port in Windows Device Manager"

- In CCS, goto "View > Terminal"

    \imageStyle{ccs_uart_00.png,width:20%}
    \image html ccs_uart_00.png "UART Terminal Menu"

- Open a new UART terminal

    \imageStyle{ccs_uart_01.png,width:50%}
    \image html ccs_uart_01.png "Open New UART Terminal"

- Select the UART port, keep other options to default, i.e 115200 baud rate - 8 data bits - No parity - 1 stop bit,

  - We use the 1st USB serial port, as seen in the device manager, for below in the SDK
    - Flashing application via UART
    - Booting application via UART
    - Console output for examples which run from R5F

  - We use the 2nd USB serial port, as seen in the device manager, as terminal output for examples which run from M4F

      \imageStyle{ccs_uart_02.png,width:25%}
      \image html ccs_uart_02.png "Connect to UART port"

  - In this screenshot this happens to be COM13 and COM14 however on your machine this could be different.
    One tip to make sure there is no mistake in identifying the UART port is to disconnect all other UART to USB devices other than this EVM before checking in device manager.

### AM64X-SK
- Many examples use a standard UART terminal to log the output from the examples.
  You can use any UART terminal program for the same. Below steps show how to setup
  the UART terminal from CCS.

- First identify the UART port as enumerated on the host machine.

  - Make sure that the AM64X-SK and UART cable connected as shown in \ref EVM_CABLES

  - In windows, you can use the "Device Manager" to see the detected UART ports
    - Search "Device Manager" in Windows Search Box in the Windows taskbar.

  - If dont see any USB serial ports listed in "Device Manager" under "Ports (COM & LPT)",
    then make sure you have installed the UART to USB driver from Silicon labs,https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers.

    \imageStyle{sk_ccs_uart_identify.png,width:60%}
    \image html sk_ccs_uart_identify.png "Identify UART Port in Windows Device Manager"

- In CCS, goto "View > Terminal"

    \imageStyle{ccs_uart_00.png,width:20%}
    \image html ccs_uart_00.png "UART Terminal Menu"

- Open a new UART terminal

    \imageStyle{ccs_uart_01.png,width:50%}
    \image html ccs_uart_01.png "Open New UART Terminal"

- Select the UART port, keep other options to default, i.e 115200 baud rate - 8 data bits - No parity - 1 stop bit,

  - We use the 2nd USB serial port, as seen in the device manager, for below in the SDK
    - Flashing application via UART
    - Booting application via UART
    - Console output for examples which run from R5F

  - We use the 1st USB serial port, as seen in the device manager, as terminal output for examples which run from M4F

      \imageStyle{sk_ccs_uart_02.png,width:25%}
      \image html sk_ccs_uart_02.png "Connect to UART port"

  - In this screenshot this happens to be COM5 and COM6 however on your machine this could be different.
    One tip to make sure there is no mistake in identifying the UART port is to disconnect all other UART to USB devices other than this AM64X-SK before checking in device manager.

## Flash SOC Initialization Binary {#EVM_FLASH_SOC_INIT}

### AM64X-EVM
\attention This is a recommended one time step that needs to be done before
           you can load and run programs via CCS

\attention If this step fails, maybe due to bad flash in EVM, then try one of the other SOC initialization steps
           mentioned at \ref EVM_SOC_INIT

\attention This step needs to be done once unless the OSPI flash has been erased
           or some other application has been flashed

- A quick recap of steps done so far that are needed for the flashing to work
  - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
  - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
  - Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES

- **POWER-OFF** the EVM

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{boot_pins_uart_mode.png,width:30%}
  \image html boot_pins_uart_mode.png "UART BOOT MODE"

- **POWER-ON** the EVM

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{uart_rom_boot.png,width:80%}
  \image html uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{ccs_uart_close.png,width:80%}
  \image html ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the EVM.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_null.cfg

  - Here COM<x> is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

        C:\ti\mcu_plus_sdk_am64x_09_00_00_00\tools\boot>python uart_uniflash.py -p COM13 --cfg=sbl_prebuilt\am64x-evm\default_sbl_null.cfg

        Parsing config file ...
        Parsing config file ... SUCCESS. Found 2 command(s) !

        Executing command 1 of 2 ...
        Found flash writer ... sending sbl prebuilt/am64x-evm/sbl_uart_uniflash.hs_fs.tiimage
        Sending sbl_prebuilt/am64x-evm/sbl_uart_uniflash.release.hs_fs.tiimage: 24% ||||   | 58653/244220 [00:07<00:17, 10706.64bytes/s],

- After all the flashing is done, you will see something like below

        Parsing config file ...
        Parsing config file ... SUCCESS. Found 2 command(s) !

        Executing command 1 of 2 ...
        Found flash writer ... sending sbl_prebuilt/am64x-evm/sbl_uart_uniflash.release.hs_fs.tiimage
        Sent flashwriter sbl_prebuilt/am64x-evm/sbl_uart_uniflash.release.hs_fs.tiimage of size 244220 bytes in 24.84s.

        Executing command 2 of 2
        Command arguments : -—file=sbl _prebuilt/am64x-evm/sbl null.release.hs_fs.tiimage --operation=flash --flash-offset=0x0
        Sent sbl_prebuilt/am64x-evm/sbl_null.release.hs_fs.tiimage of size 231036 bytes in 24.06s.

        [STATUS] SUCCESS

        All commands from config file are executed !

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

- **POWER-OFF** the EVM

- Switch the EVM boot mode to OSPI mode as shown below,

  \imageStyle{boot_pins_ospi_mode.png,width:30%}
  \image html boot_pins_ospi_mode.png "OSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the EVM

- You should see output like below on the UART terminal

        Starting NULL Bootloader ...

        DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
        DMSC Firmware revision 0x15
        DMSC ABI revision 3.1

        INFO: Bootloader_runCpu:151: CPU r5f1-0  is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU r5f1-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU m4f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:203: CPU r5f0-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:203: CPU r5f0-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runSelfCpu:215: All done, reseting self ...

- Congratulations now the EVM is setup for loading and running from CCS !!!
- You dont need to do these steps again unless you have flashed some other binary to the flash.
- Now you can build a example of interest (see \ref GETTING_STARTED_BUILD) and then run it (see \ref CCS_LAUNCH_PAGE)

\attention If SBL NULL is used for development, GEL files aren't required for R5FSS0-0 and A53SS0-0 they can be replaced by GELs of other R5/A53 cores. Steps are given below

- In the CCS target configuration CCXML file, under Advanced tab, select, `MAIN_Cortex_R5_0_0`.
- Under Cpu Properties, there should be an option **Initialization script** which should have a value `../../emulation/gel/AM64x/AM64_DDRSS/AM64_GP_EVM.gel`
- Click on browse and select the CPU reset gel, which should be under `<CCS_DIR>/ccs/ccs_base/emulation/gel/AM64x/CPU_reset.gel`
- And for A53SS0-0. In the CCS target configuration CCXML file, under Advanced tab, select, `ComputeCluster_A53_0`.
- Under Cpu Properties, there should be an option **Initialization script** which should have a value `../../emulation/gel/AM64x/AM64_DDRSS/AM64_GP_EVM.gel`
- This field needs to be cleared for `ComputeCluster_A53_0` when using **SBL NULL**
- Save the target configuration file.

### AM64X-SK
\attention This is a recommended one time step that needs to be done before
           you can load and run programs via CCS

\attention If this step fails, maybe due to bad flash in AM64X-SK, then try one of the other SOC initialization steps
           mentioned at \ref EVM_SOC_INIT

\attention This step needs to be done once unless the OSPI flash has been erased
           or some other application has been flashed

- A quick recap of steps done so far that are needed for the flashing to work
  - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
  - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
  - Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES

- **POWER-OFF** the AM64X-SK

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{sk_boot_pins_uart_mode.png,width:30%}
  \image html sk_boot_pins_uart_mode.png "UART BOOT MODE"

- **POWER-ON** the EVM

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{uart_rom_boot.png,width:80%}
  \image html uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{ccs_uart_close.png,width:80%}
  \image html ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the SK.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/@VAR_SK_BOARD_NAME_LOWER/default_sbl_null.cfg

  - Here COM<x> is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

        C:\ti\mcu_plus_sdk_am64x_09_00_00_00\tools\boot>python uart_uniflash.py -p COM13 --cfg=sbl_prebuilt\am64x-sk\default_sbl_null.cfg

        Parsing config file ...
        Parsing config file ... SUCCESS. Found 2 command(s) !

        Executing command 1 of 2 ...
        Found flash writer ... sending sbl prebuilt/am64x-sk/sbl_uart_uniflash.hs_fs.tiimage
        Sending sbl_prebuilt/am64x-sk/sbl_uart_uniflash.release.hs_fs.tiimage: 24% ||||   | 58653/244220 [00:07<00:17, 10706.64bytes/s],


- After all the flashing is done, you will see something like below

        Parsing config file ...
        Parsing config file ... SUCCESS. Found 2 command(s) !

        Executing command 1 of 2 ...
        Found flash writer ... sending sbl_prebuilt/am64x-sk/sbl_uart_uniflash.release.hs_fs.tiimage
        Sent flashwriter sbl_prebuilt/am64x-sk/sbl_uart_uniflash.release.hs_fs.tiimage of size 244220 bytes in 24.84s.

        Executing command 2 of 2
        Command arguments : -—file=sbl _prebuilt/am64x-sk/sbl null.release.hs_fs.tiimage --operation=flash --flash-offset=0x0
        Sent sbl_prebuilt/am64x-sk/sbl_null.release.hs_fs.tiimage of size 231036 bytes in 24.06s.

        [STATUS] SUCCESS

        All commands from config file are executed !

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

- **POWER-OFF** the AM64X-SK

- Switch the SK boot mode to OSPI mode as shown below,

  \imageStyle{sk_boot_pins_ospi_mode.png,width:30%}
  \image html sk_boot_pins_ospi_mode.png "OSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the AM64X-SK

- You should see output like below on the UART terminal

        Starting NULL Bootloader ...

        DMSC Firmware Version 22.1.1--v2022.01 (Terrific Llam
        DMSC Firmware revision 0x16
        DMSC ABI revision 3.1

        INFO: Bootloader_runCpu:150: CPU r5f1-0  is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU r5f1-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU m4f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU a530-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU a530-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:202: CPU r5f0-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:202: CPU r5f0-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runSelfCpu:219: All done, reseting self ...

- Congratulations now the AM64X-SK is setup for loading and running from CCS !!!
- You dont need to do these steps again unless you have flashed some other binary to the flash.
- Now you can build a example of interest (see \ref GETTING_STARTED_BUILD) and then run it (see \ref CCS_LAUNCH_PAGE)

\attention If SBL NULL is used for development, GEL files aren't required for R5FSS0-0 and A53SS0-0 they can be replaced by GELs of other R5/A53 cores. Steps are given below

- In the CCS target configuration CCXML file, under Advanced tab, select, `MAIN_Cortex_R5_0_0`.
- Under Cpu Properties, there should be an option **Initialization script** which should have a value `../../emulation/gel/AM64x/AM64_DDRSS/AM64_SK_EVM.gel`
- Click on browse and select the CPU reset gel, which should be under `<CCS_DIR>/ccs/ccs_base/emulation/gel/AM64x/CPU_reset.gel`
- And for A53SS0-0. In the CCS target configuration CCXML file, under Advanced tab, select, `ComputeCluster_A53_0`.
- Under Cpu Properties, there should be an option **Initialization script** which should have a value `../../emulation/gel/AM64x/AM64_DDRSS/AM64_SK_EVM.gel`
- This field needs to be cleared for `ComputeCluster_A53_0` when using **SBL NULL**
- Save the target configuration file.

## Additional Details

\note This section has more details on @VAR_BOARD_NAME. This is mainly for reference and can be skiped unless referred to by
other pages in this user guide.

### SOC Initialization {#EVM_SOC_INIT}

Before any program can be loaded and run on the EVM, the SOC needs to be initialized.
Below sections describes the various options available for SOC initialization.

It is recommended to use the method mentioned in \ref EVM_FLASH_SOC_INIT. However
in some cases, additional methods described below can be used esp when running on your own custom EVM
and flash driver is not yet available for that EVM.

#### SOC Initialization Using SD BOOT {#EVM_SOC_INIT_SD_BOOT_MODE}
##### AM64X-EVM
\note This step needs to be done once, unless the SD card is reformatted and/or the boot file is changed

- Format a micro-SD card to contain a FAT partition as the primary partition
- In windows,
  - Insert a micro-SD in a USB card reader, connect to the PC.
  - In "Windows Explorer", select the SD drive, right-click, select "Format"
    \imageStyle{format_sd_card_00.png,width:20%}
    \image html format_sd_card_00.png "Format SD Card"
  - Format as FAT filesystem
    \imageStyle{format_sd_card_01.png,width:20%}
    \image html format_sd_card_01.png "Format as FAT partition"
  - After formatting is done, you need to mark the partition as "ACTIVE".
    - There are various ways to do this in Windows, see online forums for various options
    - Here we use the GUI tool "Active Partition Manager" http://www.pcdisk.com/index.html
      - Download and install the tool
      - Launch the tool and select "Open Disk Image"
      - You will see all the disks present on the system, including the SD card FAT partition
      - Right click on the SD card FAT partition
      - Select "Change Attributes"
      - Enable the option "Mark Partition As Active", click "OK"
      - Close the application

- In Linux,
  - Refer to online forums to create FAT partition on a SD card and set it as "ACTIVE"
  - `gparted` is one GUI based tool that can be used to create partitions, format them and set them as "ACTIVE"

        sudo apt install gparted

- Copy the below file to the newly formatted FAT partition on the SD card and **rename** it as `tiboot3.bin`

        copy file to SD card => ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/@VAR_BOARD_NAME_LOWER/sbl_null.release.hs_fs.tiimage
        rename in SD card as => tiboot3.bin

- Eject the SD card

- **POWER-OFF** the EVM

- Set EVM to SD BOOT MODE by setting the switch setting as shown below

  \imageStyle{boot_pins_sd_mode.png,width:30%}
  \image html boot_pins_sd_mode.png "SD BOOT MODE"

- Insert the formated SD card in the SD card slot in the EVM

- Open a UART terminal as mentioned in \ref CCS_UART_TERMINAL

- **POWER-ON** the EVM

- You should see output like below on the UART terminal

        Starting NULL Bootloader ...

        DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
        DMSC Firmware revision 0x15
        DMSC ABI revision 3.1

        INFO: Bootloader_runCpu:151: CPU r5f1-0  is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU r5f1-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:151: CPU m4f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:203: CPU r5f0-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:203: CPU r5f0-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runSelfCpu:215: All done, reseting self ...

- Congratulations now the EVM is setup for loading and running from CCS !!!
- You dont need to do these steps again unless you have changed or re-formatted or deleted/changed the `tiboot3.bin` file.

##### AM64X-SK
\note This step needs to be done once, unless the SD card is reformatted and/or the boot file is changed

- Format a micro-SD card to contain a FAT partition as the primary partition
- In windows,
  - Insert a micro-SD in a USB card reader, connect to the PC.
  - In "Windows Explorer", select the SD drive, right-click, select "Format"
    \imageStyle{format_sd_card_00.png,width:20%}
    \image html format_sd_card_00.png "Format SD Card"
  - Format as FAT filesystem
    \imageStyle{format_sd_card_01.png,width:20%}
    \image html format_sd_card_01.png "Format as FAT partition"
  - After formatting is done, you need to mark the partition as "ACTIVE".
    - There are various ways to do this in Windows, see online forums for various options
    - Here we use the GUI tool "Active Partition Manager" http://www.pcdisk.com/index.html
      - Download and install the tool
      - Launch the tool and select "Open Disk Image"
      - You will see all the disks present on the system, including the SD card FAT partition
      - Right click on the SD card FAT partition
      - Select "Change Attributes"
      - Enable the option "Mark Partition As Active", click "OK"
      - Close the application

- In Linux,
  - Refer to online forums to create FAT partition on a SD card and set it as "ACTIVE"
  - `gparted` is one GUI based tool that can be used to create partitions, format them and set them as "ACTIVE"

        sudo apt install gparted

- Copy the below file to the newly formatted FAT partition on the SD card and **rename** it as `tiboot3.bin`

        copy file to SD card => ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/@VAR_SK_BOARD_NAME_LOWER/sbl_null.release.hs_fs.tiimage
        rename in SD card as => tiboot3.bin

- Eject the SD card

- **POWER-OFF** the AM64X-SK

- Set EVM to SD BOOT MODE by setting the switch setting as shown below

  \imageStyle{sk_boot_pins_sd_mode.png,width:30%}
  \image html sk_boot_pins_sd_mode.png "SD BOOT MODE"

- Insert the formated SD card in the SD card slot in the EVM

- Open a UART terminal as mentioned in \ref CCS_UART_TERMINAL

- **POWER-ON** the AM64X-SK

- You should see output like below on the UART terminal

        Starting NULL Bootloader ...

        DMSC Firmware Version 22.1.1--v2022.01 (Terrific Llam
        DMSC Firmware revision 0x16
        DMSC ABI revision 3.1

        INFO: Bootloader_runCpu:150: CPU r5f1-0  is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU r5f1-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU m4f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU a530-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU a530-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:202: CPU r5f0-0 is initialized to 800000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:202: CPU r5f0-1 is initialized to 800000000 Hz !!!
        INFO: Bootloader_runSelfCpu:219: All done, reseting self ...

- Congratulations now the EVM is setup for loading and running from CCS !!!
- You dont need to do these steps again unless you have changed or re-formatted or deleted/changed the `tiboot3.bin` file.

#### SOC Initialization Using CCS Scripting {#EVM_SOC_INIT_NOBOOT_MODE}

##### Set Environment Variable

\note This step needs to be done once and is needed for the
  SOC initialization script `load_dmsc_hsfs.js` to find certain initialization files within the SDK folder.
  This variable is not used otherwise in the build process. If you dont like adding variables in the environment, then
  you need to edit the file `${SDK_INSTALL_PATH}/tools/ccs_load/am64x/load_dmsc_hsfs.js`
  and specify the SDK path in the file itself.

- Add path to the SDK folder as a environment variable in the host machine.

- In windows, goto "Windows Task Bar Search" and search for "environment variables for your account"
        \imageStyle{ccs_setup_03.png,width:15%}
        \image html ccs_setup_03.png "Environment Variables For Your Account"

\cond SOC_AM64X
- Add a new variable named `MCU_PLUS_SDK_AM64X_PATH` and point it to the path where the SDK is installed
\endcond
\cond SOC_AM243X
- Add a new variable named `MCU_PLUS_SDK_AM243X_PATH` and point it to the path where the SDK is installed
\endcond

\imageStyle{ccs_setup_04.png,width:50%}
\image html ccs_setup_04.png "Add New Environment Variable For Your Account"

- In Linux, usually you need to add a line as below in the ${HOME}/.bashrc,

\cond SOC_AM64X
        export MCU_PLUS_SDK_AM64X_PATH=${HOME}/ti/mcu_plus_sdk_am64x_{sdk version}/
\endcond
\cond SOC_AM243X
        export MCU_PLUS_SDK_AM243X_PATH=${HOME}/ti/mcu_plus_sdk_am243x_{sdk version}/
\endcond

- If CCS is open, close and reopen CCS for the CCS to be able to see the updated environment variable

##### Run the SOC Initialization Script
###### AM64X-EVM
\attention This step needs to be done **every time** the EVM is power-cycled.

- **POWER-OFF** the EVM

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable

- Set EVM in DEV BOOT MODE mode as shown below
    \imageStyle{boot_pins_devboot_mode.png,width:30%}
    \image html boot_pins_devboot_mode.png "DEV BOOT MODE"

- **POWER-ON** the EVM

- Launch the target configuration created with \ref CCS_NEW_TARGET_CONFIG

    \imageStyle{ccs_launch_00.png,width:40%}
    \image html ccs_launch_00.png "Launch Target Configuration"

- You will see the @VAR_SOC_NAME target configuration in the "Debug" window as shown below

    \imageStyle{ccs_launch_01.png,width:40%}
    \image html ccs_launch_01.png "Target Configuration After Launch"

- Goto "CCS Toolbar > View > Scripting Console"

- Type the below command in the scripting console and press "enter", to load DMSC FW and initialize the SOC
  - In Windows, assuming the SDK is installed at `C:/ti/mcu_plus_sdk_{soc}_{sdk version}`

        loadJSFile "C:/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am64x/load_dmsc_hsfs.js" (HS-FS device)

    \imageStyle{ccs_load_dmsc_00.png,width:50%}
    \image html ccs_load_dmsc_00.png "Scripting Console"

- In Linux, run the same command, only the path would be a Linux path like `/home/{username}/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am64x/load_dmsc_hsfs.js`

- After successful execution of this script one would see logs as below

  - In the scripting console, this is log from the script itself.

        js:> LoadJSFile "C:/ti/mcu_plus_sdk/tools/ccs_load/am64x/load_dmsc_hsfs.js"
        Connecting to MCU Cortex_R5_0!
        Writing While(1) for R5F
        Running the board configuration initialization from R5!
        Happy Debugging

  - In the @VAR_SOC_NAME "CIO" console, this is log from the R5F.

        [MAIN_Cortex_R5_0_0]
        DMSC Firmware Version AA.B.C-vDDDD.EEE-FF-gggggg (HHH
        DMSC Firmware revision 0xNN
        DMSC ABI revision x.y

        [SCICLIENT] ABI check PASSED
        [SCICLIENT] Board Configuration with Debug enabled ...
        [SCICLIENT] Common Board Configuration PASSED
        [SCICLIENT] PM Board Configuration PASSED
        [SCICLIENT] RM Board Configuration PASSED
        [SCICLIENT] Security Board Configuration PASSED

        DMSC Firmware Version AA.B.C-vDDDD.EEE-FF-gggggg (HHH
        DMSC Firmware revision 0xNN
        DMSC ABI revision x.y

        All tests have passed!!

  - In the @VAR_SOC_NAME console, this is log from the GEL scripts.
    \imageStyle{ccs_load_dmsc_03.png,width:50%}
    \image html ccs_load_dmsc_03.png "Select @VAR_SOC_NAME GEL Console"

    \imageStyle{ccs_load_dmsc_04.png,width:50%}
    \image html ccs_load_dmsc_04.png "@VAR_SOC_NAME Console GEL Log"

- For success, all the three consoles should have no errors in their logs.

- If the script is run without providing power to the EVM or if the EVM BOOTMODE is
  not set to \ref BOOTMODE_NOBOOT then you will see errors in the console and/or unexpected behaviour and error messages.
  - **SOLUTION**: Power cycle EVM and repeat the steps.

###### AM64X-SK
\attention This step needs to be done **every time** the EVM is power-cycled.

- **POWER-OFF** the AM64X-SK

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable

- Set EVM in NOBOOT mode as shown below
  \imageStyle{sk_boot_pins_noboot_mode.png,width:30%}
  \image html sk_boot_pins_noboot_mode.png "NO BOOT MODE"

- **POWER-ON** the AM64X-SK

- Launch the target configuration created with \ref CCS_NEW_TARGET_CONFIG

    \imageStyle{ccs_launch_00.png,width:40%}
    \image html ccs_launch_00.png "Launch Target Configuration"

- You will see the @VAR_SOC_NAME target configuration in the "Debug" window as shown below

    \imageStyle{ccs_launch_01.png,width:40%}
    \image html ccs_launch_01.png "Target Configuration After Launch"

- Goto "CCS Toolbar > View > Scripting Console"

- Type the below command in the scripting console and press "enter", to load DMSC FW and initialize the SOC
  - In Windows, assuming the SDK is installed at `C:/ti/mcu_plus_sdk_{soc}_{sdk version}`

        loadJSFile "C:/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am64x/load_dmsc_hsfs.js"

    \imageStyle{ccs_load_dmsc_00.png,width:50%}
    \image html ccs_load_dmsc_00.png "Scripting Console"

- In Linux, run the same command, only the path would be a Linux path like `/home/{username}/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am64x/load_dmsc_hsfs.js`

- After successful execution of this script one would see logs as below

  - In the scripting console, this is log from the script itself.

        js:> LoadJSFile "C:/ti/mcu_plus_sdk/tools/ccs_load/am243x/load_dmsc_hsfs.js"
        Connecting to MCU Cortex_R5_0!
        Writing While(1) for R5F
        Running the board configuration initialization from R5!
        Happy Debugging

  - In the @VAR_SOC_NAME "CIO" console, this is log from the R5F.

        [MAIN_Cortex_R5_0_0]
        DMSC Firmware Version AA.B.C-vDDDD.EEE-FF-gggggg (HHH
        DMSC Firmware revision 0xNN
        DMSC ABI revision x.y

        [SCICLIENT] ABI check PASSED
        [SCICLIENT] Board Configuration with Debug enabled ...
        [SCICLIENT] Common Board Configuration PASSED
        [SCICLIENT] PM Board Configuration PASSED
        [SCICLIENT] RM Board Configuration PASSED
        [SCICLIENT] Security Board Configuration PASSED

        DMSC Firmware Version AA.B.C-vDDDD.EEE-FF-gggggg (HHH
        DMSC Firmware revision 0xNN
        DMSC ABI revision x.y

        All tests have passed!!

  - In the @VAR_SOC_NAME console, this is log from the GEL scripts.
    \imageStyle{ccs_load_dmsc_03.png,width:50%}
    \image html ccs_load_dmsc_03.png "Select @VAR_SOC_NAME GEL Console"

    \imageStyle{ccs_load_dmsc_04.png,width:50%}
    \image html ccs_load_dmsc_04.png "@VAR_SOC_NAME Console GEL Log"

- For success, all the three consoles should have no errors in their logs.

- If the script is run without providing power to the AM64X-SK or if the AM64X-SK BOOTMODE is
  not set to \ref BOOTMODE_NOBOOT then you will see errors in the console and/or unexpected behaviour and error messages.
  - **SOLUTION**: Power cycle AM64X-SK and repeat the steps.

##### DDR Initialization {#DDR_INIT}

- To initialize DDR, do the additional step below from CCS GEL menu.
- Launch CCS debug session as mentioned here, \ref CCS_LAUNCH
- Connect to `MAIN_Cortex_R5_0_0`

    \imageStyle{ccs_load_run_00.png,width:60%}
    \image html ccs_load_run_00.png "Connect to R5F0-0"

- Run the DDR initialization from GEL menu as shown below,

    \imageStyle{ddr_init.png,width:60%}
    \image html ddr_init.png "DDR initialization via GEL menu"

- After the DDR initialization is done you will see prints like below in the CCS "CIO" console,

        MAIN_Cortex_R5_0_0: GEL Output: --->>> DDR4 Initialization is in progress ... <<<---
        MAIN_Cortex_R5_0_0: GEL Output: --->>> ECC Disabled <<<---
        MAIN_Cortex_R5_0_0: GEL Output: --->>> DDR controller programming in progress.. <<<---
        MAIN_Cortex_R5_0_0: GEL Output: --->>> DDR controller programming completed... <<<---
        MAIN_Cortex_R5_0_0: GEL Output: --->>> DDR PI programming in progress.. <<<---
        MAIN_Cortex_R5_0_0: GEL Output: --->>> DDR PI programming completed... <<<---
        ...
        MAIN_Cortex_R5_0_0: GEL Output: --->>> DDR Initialization completed... <<<---
        MAIN_Cortex_R5_0_0: GEL Output: --->>> DDR4 Initialization is DONE! <<<---

- Now you can load programs to DDR and they can access DDR. Note, however for R5F and M4F to access DDR, there
  needs to be a MPU region setup to cover the DDR memory. By default, most SDK examples dont setup this MPU region.
  Also all code that executes before this MPU region is setup MUST NOT execute from DDR.

### BOOT MODE
#### UART BOOT MODE  {#BOOTMODE_UART}
##### AM64X-EVM

This mode is used to flash files to the EVM flash via UART. It can also be used to boot applications via UART.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1101 1100
    BOOTMODE [ 8 : 15 ] (SW3) = 1011 0000
    \endcode

  \imageStyle{boot_pins_uart_mode.png,width:30%}
  \image html boot_pins_uart_mode.png "UART BOOT MODE"

##### AM64X-SK

This mode is used to flash files to the AM64X-SK flash via UART. It can also be used to boot applications via UART.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1101 1100
    BOOTMODE [ 8 : 15 ] (SW3) = 1011 0000
    \endcode

  \imageStyle{sk_boot_pins_uart_mode.png,width:30%}
  \image html sk_boot_pins_uart_mode.png "UART BOOT MODE"

#### DFU BOOT MODE {#BOOTMODE_DFU}

##### AM64x-EVM

This mode is used to flash files to the EVM flash via USB2.0 DFU. It can also be used to boot applications via USB2.0 DFU.

    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1100 1010
    BOOTMODE [ 8 : 15 ] (SW3) = 0000 0000
    \endcode

  \imageStyle{bootpins-dfu-evm.jpg,width:30%}
  \image html bootpins-dfu-evm.jpg "DFU BOOT MODE"

#### OSPI BOOT MODE  {#BOOTMODE_OSPI}
##### AM64X-EVM
This mode is used to boot flashed applications via EVM flash like OSPI flash
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1100 1110
    BOOTMODE [ 8 : 15 ] (SW3) = 0100 0000
    \endcode

  \imageStyle{boot_pins_ospi_mode.png,width:30%}
  \image html boot_pins_ospi_mode.png "OSPI BOOT MODE"

##### AM64X-SK

This mode is used to boot flashed applications via AM64X-SK flash like OSPI flash
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1100 1110
    BOOTMODE [ 8 : 15 ] (SW3) = 0100 0000
    \endcode

  \imageStyle{sk_boot_pins_ospi_mode.png,width:30%}
  \image html sk_boot_pins_ospi_mode.png "OSPI BOOT MODE"


#### SD BOOT MODE  {#BOOTMODE_SD}
##### AM64X-EVM
This mode is used to boot applications via SD card on the EVM.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1100 0011
    BOOTMODE [ 8 : 15 ] (SW3) = 0110 1100
    \endcode

  \imageStyle{boot_pins_sd_mode.png,width:30%}
  \image html boot_pins_sd_mode.png "SD BOOT MODE"

##### AM64X-SK
This mode is used to boot applications via SD card on the AM64X-SK.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1100 0011
    BOOTMODE [ 8 : 15 ] (SW3) = 0110 1100
    \endcode

  \imageStyle{sk_boot_pins_sd_mode.png,width:30%}
  \image html sk_boot_pins_sd_mode.png "SD BOOT MODE"

#### DEV BOOT MODE {#BOOTMODE_NOBOOT}
##### AM64X-EVM
This mode is used in conjunction with the `load_dmsc_hsfs.js` script described here \ref EVM_SOC_INIT_NOBOOT_MODE,
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1101 1110
    BOOTMODE [ 8 : 15 ] (SW3) = 0000 0000
    \endcode

  \imageStyle{boot_pins_devboot_mode.png,width:30%}
  \image html boot_pins_devboot_mode.png "DEV BOOT MODE"

##### AM64X-SK
This mode is used in conjunction with the `load_dmsc.js` script described here \ref EVM_SOC_INIT_NOBOOT_MODE,
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1101 1111
    BOOTMODE [ 8 : 15 ] (SW3) = 0000 0000
    \endcode

  \imageStyle{sk_boot_pins_noboot_mode.png,width:30%}
  \image html sk_boot_pins_noboot_mode.png "NO BOOT MODE"

#### EMMC BOOT MODE  {#BOOTMODE_EMMC}
##### AM64X-EVM
This mode is used to boot applications via eMMC on the EVM.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1101 0010
    BOOTMODE [ 8 : 15 ] (SW3) = 0000 0000
    \endcode

  \imageStyle{boot_pins_emmc_mode.png,width:30%}
  \image html boot_pins_emmc_mode.png "EMMC BOOT MODE"

#### PCIE BOOT MODE  {#BOOTMODE_PCIE}
##### AM64X-EVM
This mode is used to boot applications via PCIe on the EVM.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1101 0110
    BOOTMODE [ 8 : 15 ] (SW3) = 0000 0000
    \endcode

  \imageStyle{boot_pins_pcie_mode.png,width:30%}
  \image html boot_pins_pcie_mode.png "PCIE BOOT MODE"

## Troubleshooting EVM issues {#TROUBLESHOOT_ISSUES}

 - JTAG connection fails on some EVMs with the following error. Need to connect the JTAG cable after board is powered on.

  \imageStyle{jtag_connection_error.PNG,width:30%}
  \image html jtag_connection_error.PNG "JTAG Connection Error Dialog"
