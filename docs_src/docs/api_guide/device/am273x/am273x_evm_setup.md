#  EVM Setup {#EVM_SETUP_PAGE}

[TOC]

## @VAR_SOC_NAME EVM

\cond SOC_NONE
\note Refer to EVM page for more details on the EVM, https://www.ti.com/tool/TMDS273GPEVM
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

- UART for R5F console is only visible in Device Manager when USB XDS110 cable is connected to EVM

- UART for C66x console is only visible in Device Manager when UART to USB cable is connected to EVM

  - We use the 1st USB serial port, as seen in the device manager, when USB XDS110 cable is connected for below in the SDK
    - Flashing application via UART
    - Booting application via UART
    - Console output for examples which run from R5F

  - We use the 3rd USB serial port out of 4 ports emulated, when UART to USB cable is connected for below in the SDK
    - Console output for examples which run from C66x

  - In this screenshot this happens to be COM13 and COM14 however on your machine this could be different.

  - Tips for checking the right UART port number
     - disconnect all other UART to USB devices other than this EVM before checking in device manager.
     -  un-plug and re-plug the USB FTDI and XDS110 connection to the PC while the Device Manager is open and the COM ports are visible

#### BOOT MODE

##### UART BOOT MODE  {#BOOTMODE_UART}

This mode is used to flash files to the EVM flash via UART. It can also be used to boot applications via UART.

    SOP0 - Short
    SOP2 - Short
    SOP1 - Open

  \imageStyle{uart_bootmode.png,width:30%}
  \image html uart_bootmode.png "UART BOOT MODE"

##### QSPI BOOT MODE  {#BOOTMODE_QSPI}

This mode is used to boot flashed applications via EVM flash like OSPI flash

    SOP0 - Short
    SOP2 - Open
    SOP1 - Open

  \imageStyle{qspi_bootmode.png,width:30%}
  \image html qspi_bootmode.png "QSPI BOOT MODE"


##### NOBOOT MODE  {#BOOTMODE_NOBOOT}

This mode is used with the Gel files without using the SBL.

    SOP0 - Short
    SOP2 - Open
    SOP1 - Short

  \imageStyle{no_bootmode.png,width:30%}
  \image html no_bootmode.png "NO BOOT MODE"


