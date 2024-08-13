#  IDK Setup {#IDK_SETUP_PAGE}

[TOC]

\cond SOC_NONE
\note Refer to EVM page for more details on the EVM, https://www.ti.com/tool/TMDS243GPEVM
\endcond

\attention This document contains the details for @VAR_BOARD_NAME board setup under each section.
            User should select the right board setup in the sections.

## Cable Connections {#EVM_CABLES}
### AM65X-IDK
- The figure below shows some important cable connections, ports and switches.
  - Take note of the location of the "BOOTMODE" switch, this is used to
    switch between different boot modes like OSPI, UART, SD, NOBOOT mode

  \imageStyle{evm_overview.png,width:75%}
  \image html evm_overview.png "@VAR_BOARD_NAME"

## Setup UART Terminal {#CCS_UART_TERMINAL}
### AM65X-IDK
- Many examples use a standard UART terminal to log the output from the examples.
  You can use any UART terminal program for the same. Below steps show how to setup
  the UART terminal from CCS.

- First identify the UART port as enumerated on the host machine.

  - Make sure that the IDK and UART cable connected as shown in \ref EVM_CABLES

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

  - We use the 2nd USB serial port, as seen in the device manager, as terminal output for examples which run from M3F

      \imageStyle{ccs_uart_02.png,width:25%}
      \image html ccs_uart_02.png "Connect to UART port"

  - In this screenshot this happens to be COM13 and COM14 however on your machine this could be different.
    One tip to make sure there is no mistake in identifying the UART port is to disconnect all other UART to USB devices other than this IDK before checking in device manager.

## Flash SOC Initialization Binary {#EVM_FLASH_SOC_INIT}

\note We have mentioned the steps for GP device.If you have an HS device, please
refer to the migration guide \ref HS_MIGRATION_GUIDE, specifically \ref SBL_BOOT_HS for differences

### AM65X-IDK
\attention This is a recommended one time step that needs to be done before
           you can load and run programs via CCS

\attention If this step fails, maybe due to bad flash in IDK, then try one of the other SOC initialization steps
           mentioned at \ref EVM_SOC_INIT

\attention This step needs to be done once unless the OSPI flash has been erased
           or some other application has been flashed

- A quick recap of steps done so far that are needed for the flashing to work
  - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
  - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
  - Make sure you have the IDK power cable and UART cable connected as shown in \ref EVM_CABLES

- **POWER-OFF** the IDK

- Set boot mode to UART BOOTMODE as shown in below image

  \imageStyle{boot_pins_uart_mode.png,width:30%}
  \image html boot_pins_uart_mode.png "UART BOOT MODE"

- **POWER-ON** the IDK

- You should see character "C" getting printed on the UART terminal every 2-3 seconds as shown below

  \imageStyle{uart_rom_boot.png,width:80%}
  \image html uart_rom_boot.png "UART output in UART BOOT MODE"

- Close the UART terminal as shown below. This is important, else the UART script in next step wont be able to connect to the UART port.

  \imageStyle{ccs_uart_close.png,width:80%}
  \image html ccs_uart_close.png "Close UART terminal"

- Open a command prompt and run the below command to flash the SOC initialization binary to the IDK.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_null.cfg

  - Here COM<x> is the port name of the identified UART port in Windows.
  - On Linux,
    - The name for UART port is typically something like `/dev/ttyUSB0`
    - On some Linux systems, one needs to use `python3` to invoke python3.x, just `python` command may invoke python 2.x which will not work with the flashing script.

- When the flashing is in progress you will see something like below

  \imageStyle{flash_soc_init_in_progress.png,width:100%}
  \image html flash_soc_init_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

  \imageStyle{flash_soc_init_success.png,width:80%}
  \image html flash_soc_init_success.png "Flashing successful"

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

- **POWER-OFF** the IDK

- Switch the IDK boot mode to OSPI mode as shown below,

  \imageStyle{boot_pins_ospi_mode.png,width:30%}
  \image html boot_pins_ospi_mode.png "OSPI BOOT MODE"

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- **POWER-ON** the IDK

- You should see output like below on the UART terminal

        Starting NULL Bootloader ...

        DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
        DMSC Firmware revision 0x15
        DMSC ABI revision 3.1

        INFO: Bootloader_loadSelfCpu:207: CPU r5f0-0 is initialized to 400000000 Hz !!!
        INFO: Bootloader_loadSelfCpu:207: CPU r5f0-1 is initialized to 400000000 Hz !!!
        INFO: Bootloader_runSelfCpu:217: All done, reseting self ...

- Congratulations now the IDK is setup for loading and running from CCS !!!
- You dont need to do these steps again unless you have flashed some other binary to the flash.
- Now you can build a example of interest (see \ref GETTING_STARTED_BUILD) and then run it (see \ref CCS_LAUNCH_PAGE)

## Additional Details

\note This section has more details on @VAR_BOARD_NAME. This is mainly for reference and can be skiped unless referred to by
other pages in this user guide.

### SOC Initialization Using CCS Scripting {#EVM_SOC_INIT_NOBOOT_MODE}

\note We have mentioned the steps for GP device.

#### Set Environment Variable

\note This step needs to be done once and is needed for the
  SOC initialization script `load_dmsc.js`/`load_dmsc_hsfs.js` to find certain initialization files within the SDK folder.
  This variable is not used otherwise in the build process. If you dont like adding variables in the environment, then
  you need to edit the file `${SDK_INSTALL_PATH}/tools/ccs_load/am65x/load_dmsc.js`/`${SDK_INSTALL_PATH}/tools/ccs_load/am65x/load_dmsc_hsfs.js`
  and specify the SDK path in the file itself.

- Add path to the SDK folder as a environment variable in the host machine.

- In windows, goto "Windows Task Bar Search" and search for "environment variables for your account"
        \imageStyle{ccs_setup_03.png,width:15%}
        \image html ccs_setup_03.png "Environment Variables For Your Account"


- Add a new variable named `MCU_PLUS_SDK_AM65X_PATH` and point it to the path where the SDK is installed

\imageStyle{ccs_setup_04_am65x.png,width:50%}
\image html ccs_setup_04_am65x.png "Add New Environment Variable For Your Account"

- In Linux, usually you need to add a line as below in the ${HOME}/.bashrc,

        export MCU_PLUS_SDK_AM65X_PATH=${HOME}/ti/mcu_plus_sdk_am65x_{sdk version}/



- If CCS is open, close and reopen CCS for the CCS to be able to see the updated environment variable

##### Run the SOC Initialization Script {#EVM_SOC_INIT}
###### AM65X-IDK
\attention This step needs to be done **every time** the IDK is power-cycled.

- **POWER-OFF** the IDK

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable

- Set IDK in NOBOOT mode as shown below
  \imageStyle{boot_pins_noboot_mode.png,width:30%}
  \image html boot_pins_noboot_mode.png "NO BOOT MODE"

- **POWER-ON** the IDK

- Launch the target configuration created with \ref CCS_NEW_TARGET_CONFIG

    \imageStyle{ccs_launch_00.png,width:40%}
    \image html ccs_launch_00.png "Launch Target Configuration"

- You will see the @VAR_SOC_NAME target configuration in the "Debug" window as shown below

    \imageStyle{ccs_launch_01.png,width:40%}
    \image html ccs_launch_01.png "Target Configuration After Launch"

- Goto "CCS Toolbar > View > Scripting Console"

- Type the below command in the scripting console and press "enter", to load DMSC FW and initialize the SOC
  - In Windows, assuming the SDK is installed at `C:/ti/mcu_plus_sdk_{soc}_{sdk version}`

        loadJSFile "C:/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am65x/load_dmsc.js" (GP device)

    \imageStyle{ccs_load_dmsc_00.png,width:50%}
    \image html ccs_load_dmsc_00.png "Scripting Console"

- In Linux, run the same command, only the path would be a Linux path like `/home/{username}/ti/mcu_plus_sdk_{soc}_{sdk version}/tools/ccs_load/am65x/load_dmsc.js`
- After successful execution of this script one would see logs as below

  - In the scripting console, this is log from the script itself.
    \imageStyle{ccs_load_dmsc_01.png,width:50%}
    \image html ccs_load_dmsc_01.png "Scripting Console Log"

  - In the @VAR_SOC_NAME "CIO" console, this is log from the R5F.
    \imageStyle{ccs_load_dmsc_02.png,width:50%}
    \image html ccs_load_dmsc_02.png "@VAR_SOC_NAME CIO Console R5F Log"

  - In the @VAR_SOC_NAME console, this is log from the GEL scripts.
    \imageStyle{ccs_load_dmsc_03.png,width:50%}
    \image html ccs_load_dmsc_03.png "Select @VAR_SOC_NAME GEL Console"

    \imageStyle{ccs_load_dmsc_04.png,width:50%}
    \image html ccs_load_dmsc_04.png "@VAR_SOC_NAME Console GEL Log"

- For success, all the three consoles should have no errors in their logs.

- If the script is run without providing power to the IDK or if the IDK BOOTMODE is
  not set to \ref BOOTMODE_NOBOOT then you will see errors in the console and/or unexpected behaviour and error messages.
  - **SOLUTION**: Power cycle IDK and repeat the steps.

### BOOT MODE

#### NOBOOT MODE  {#BOOTMODE_NOBOOT}
##### AM65X-IDK
This mode is used in conjunction with the `load_dmsc.js` script described here \ref EVM_SOC_INIT_NOBOOT_MODE,
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 0010 0000
    BOOTMODE [ 8 : 15 ] (SW3) = 0000 0000
    BOOTMODE [0:1]      (SW4) = 00
    \endcode

  \imageStyle{boot_pins_noboot_mode.png,width:30%}
  \image html boot_pins_noboot_mode.png "NO BOOT MODE"

#### UART BOOT MODE  {#BOOTMODE_UART}
##### AM65X-IDK

This mode is used to flash files to the IDK flash via UART. It can also be used to boot applications via UART.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 0010 0000
    BOOTMODE [ 8 : 15 ] (SW3) = 0101 0000
    BOOTMODE [0:1]      (SW4) = 00
    \endcode

  \imageStyle{boot_pins_uart_mode.png,width:30%}
  \image html boot_pins_uart_mode.png "UART BOOT MODE"

#### OSPI BOOT MODE  {#BOOTMODE_OSPI}
##### AM65X-IDK
This mode is used to boot flashed applications via IDK flash like OSPI flash
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 1000 0000
    BOOTMODE [ 8 : 15 ] (SW3) = 1000 0000
    BOOTMODE [0:1]      (SW4) = 00
    \endcode

  \imageStyle{boot_pins_ospi_mode.png,width:30%}
  \image html boot_pins_ospi_mode.png "OSPI BOOT MODE"

#### SD BOOT MODE  {#BOOTMODE_SD}
##### AM65X-IDK
This mode is used to boot applications via SD card on the IDK.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 0010 0000
    BOOTMODE [ 8 : 15 ] (SW3) = 0110 0000
    BOOTMODE [0:1]      (SW4) = 00
    \endcode

  \imageStyle{boot_pins_sd_mode.png,width:30%}
  \image html boot_pins_sd_mode.png "SD BOOT MODE"

#### PCIE BOOT MODE  {#BOOTMODE_PCIE}
##### AM65X-IDK
This mode is used to boot applications via PCIe on the IDK.
    \code
    BOOTMODE [ 0 :  7 ] (SW2) = 0010 1000
    BOOTMODE [ 8 : 15 ] (SW3) = 1001 0000
    \endcode

  \imageStyle{boot_pins_pcie_mode.png,width:30%}
  \image html boot_pins_pcie_mode.png "PCIE BOOT MODE"

## Troubleshooting IDK issues {#TROUBLESHOOT_ISSUES}

 - JTAG connection fails on some IDKs with the following error. Need to connect the JTAG cable after board is powered on.

  \imageStyle{jtag_connection_error.PNG,width:30%}
  \image html jtag_connection_error.PNG "JTAG Connection Error Dialog"
