#  EVM Setup {#EVM_SETUP_PAGE}

[TOC]

## @VAR_SOC_NAME EVM

\cond SOC_NONE
\note Refer to EVM page for more details on the EVM, https://www.ti.com/product/AWR2243
\endcond

### Cable Connections {#EVM_CABLES}

- The figure below shows some important cable connections, ports and switches.
  - Take note of the location of the "BOOTMODE" switch, this is used to
    switch between different boot modes like OSPI, UART, SD, NOBOOT mode

  \imageStyle{evm_overview.png,width:50%}
  \image html evm_overview.png "@VAR_BOARD_NAME"

### Setup UART Terminal {#CCS_UART_TERMINAL}

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

  - We use the 2st USB serial port, as seen in the device manager, as terminal output for examples which run from M4F

      \imageStyle{ccs_uart_02.png,width:25%}
      \image html ccs_uart_02.png "Connect to UART port"

  - In this screenshot this happens to be COM13 and COM14 however on your machine this could be different.
    One tip to make sure there is no mistake in identifying the UART port is to disconnect all other UART to USB devices other than this EVM before checking in device manager.

### Flash SOC Initialization Binary {#EVM_FLASH_SOC_INIT}

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

  \imageStyle{uart_bootmode.png,width:30%}
  \image html uart_bootmode.png "UART BOOT MODE"
- **POWER-ON** the EVM

- You should see character "C" getting printed on the UART terminal every 1-2 seconds as shown below

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

  \imageStyle{flash_soc_init_in_progress.png,width:100%}
  \image html flash_soc_init_in_progress.png "Flash in progress"

- After all the flashing is done, you will see something like below

  \imageStyle{flash_soc_init_success.png,width:80%}
  \image html flash_soc_init_success.png "Flashing successful"

- The above step flashes the SBL QSPI and the SBL Multicore App images to the flash. The multicore app will have the valid appimage for all cores with just while(1) loop.

\note Due to current limitations with SBL NULL, connecting to C66x core using CCS does not work.
\note Hence SBL QSPI is used instead of SBL NULL. User can use SBL NULL if using only R5 cores.

- Use the below instructions to flash the SBL NULL binary.

        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p COM<x> --cfg=sbl_prebuilt/@VAR_BOARD_NAME_LOWER/default_sbl_null_no_appimage.cfg

- If flashing has failed, see \ref TOOLS_FLASH_ERROR_MESSAGES, and resolve the errors.

- If flashing is successful, do the next steps ...

- **POWER-OFF** the EVM

- Switch the EVM boot mode to QSPI mode as shown below,

  \imageStyle{qspi_bootmode.png,width:30%}
  \image html qspi_bootmode.png "QSPI BOOT MODE"

- **POWER-ON** the EVM

- Re-connect the UART terminal in CCS window as shown in \ref CCS_UART_TERMINAL

- Press the reset button on the board

- You should see output like below on the UART terminal

        Starting QSPI Bootloader ...
        INFO: Bootloader_runCpu:150: CPU r4 is initialized to 200000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU c66ss0 is initialized to 360000000 Hz !!!
        INFO: Bootloader_runCpu:150: CPU r5f0-1 is initialized to 300000000 Hz !!!
        [BOOTLOADER PROFILE] System_init                      :        434us
        [BOOTLOADER PROFILE] Drivers_open                     :         17us
        [BOOTLOADER PROFILE] Board_driversOpen                :       2721us
        [BOOTLOADER PROFILE] CPU load                         :     365527us
        [BOOTLOADER_PROFILE] SBL Total Time Taken             :    1392241us

        Image loading done, switching to application ...
        INFO: Bootloader_runSelfCpu:219: All done, reseting self ...

- Congratulations now the EVM is setup for loading and running from CCS !!!
- You dont need to do these steps again unless you have flashed some other binary to the flash.
- Now you can build a example of interest (see \ref GETTING_STARTED_BUILD) and then run it (see \ref CCS_LAUNCH_PAGE)

### Additional Details

\note This section has more details on @VAR_BOARD_NAME. This is mainly for reference and can be skiped unless referred to by
other pages in this user guide.

#### SOC Initialization {#EVM_SOC_INIT}

Before any program can be loaded and run on the EVM, the SOC needs to be initialized.
Below sections describes the various options available for SOC initialization.

It is recommended to use the method mentioned in \ref EVM_FLASH_SOC_INIT.

#### BOOT MODE

##### UART BOOT MODE  {#BOOTMODE_UART}

This mode is used to flash files to the EVM flash via UART. It can also be used to boot applications via UART.

    \code
    SOP0 - Short
    SOP2 - Short
    SOP1 - Open
    \endcode

  \imageStyle{uart_bootmode.png,width:30%}
  \image html uart_bootmode.png "UART BOOT MODE"

##### QSPI BOOT MODE  {#BOOTMODE_QSPI}

This mode is used to boot flashed applications via EVM flash like OSPI flash

    \code
    SOP0 - Short
    SOP2 - Open
    SOP1 - Open
    \endcode

  \imageStyle{qspi_bootmode.png,width:30%}
  \image html qspi_bootmode.png "QSPI BOOT MODE"


##### NOBOOT MODE  {#BOOTMODE_NOBOOT}

This mode is used with the Gel files without using the SBL.

    \code
    SOP0 - Short
    SOP2 - Open
    SOP1 - Short
    \endcode

  \imageStyle{no_bootmode.png,width:30%}
  \image html no_bootmode.png "NO BOOT MODE"
