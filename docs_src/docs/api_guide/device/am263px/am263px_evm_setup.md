#  EVM Setup {#EVM_SETUP_PAGE}

[TOC]

## EVM Overview

### AM263PX-CC

- EVM (Part Number: TMDSCNCD263P)

  \imageStyle{evm_overview_2.png,width:50%}
  \image html evm_overview_2.png "@VAR_BOARD_NAME"

<!-- - Major Components and Features

  \imageStyle{evm_overview_3.png,width:80%}
  \image html evm_overview_3.png "@VAR_BOARD_NAME Components and Features" -->

- What is not included with EVM
  - USB Type-C AC/DC Supply, 5V/3A. Power options available from typical distributors
  - TMDSHSECDOCK - TI Control Card debug breakout

<!-- \note Refer to EVM page for more details, https://www.ti.com/tool/TMDSCNCD263 -->

<!-- ### AM263X-LP

  \imageStyle{am263x_lp_overview.png,width:50%}
  \image html am263x_lp_overview.png "@VAR_LP_BOARD_NAME"

- Major Components and Features

  \imageStyle{am263x_lp_overview1.png,width:80%}
  \image html am263x_lp_overview1.png "@VAR_LP_BOARD_NAME Components and Features"

\note Refer to EVM page for more details, https://www.ti.com/tool/LP-AM263 -->

## Cable Connections {#EVM_CABLES}

Important cable connections, ports and switches.

### AM263PX-CC

- System Power On/Off
  - 5V power through Type-C 5V/3A
  - HSEC 5V power is available only if Type-C is unavailable
  - No mechanical on/off switch on this EVM

  \imageStyle{evm_overview_4.png,width:50%}
  \image html evm_overview_4.png "@VAR_BOARD_NAME Power "

- JTAG Switch selection

  \imageStyle{evm_overview_5.png,width:20%}
  \image html evm_overview_5.png "@VAR_BOARD_NAME JTAG selection"

- Bootmode selection

  \imageStyle{evm_overview_6.png,width:50%}
  \image html evm_overview_6.png "@VAR_BOARD_NAME bootmode"

<!-- ### AM263X-LP
  \imageStyle{am263x_lp_power_supply.png,width:50%}
  \image html am263x_lp_power_supply.png "@VAR_LP_BOARD_NAME Power "

- Bootmode selection

  \imageStyle{am263x_lp_bootswitch.png,width:20%}
  \image html am263x_lp_bootswitch.png "@VAR_LP_BOARD_NAME bootmode" -->

## Setup UART Terminal {#CCS_UART_TERMINAL}

###AM263PX-CC
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

  - We use this serial port, as seen in the device manager, for below in the SDK
    - Flashing application via UART
    - Booting application via UART
    - Console output for examples

      \imageStyle{ccs_uart_02.png,width:25%}
      \image html ccs_uart_02.png "Connect to UART port"

  - In this screenshot this happens to be COM5/COM11 however on your machine this could be different.
    One tip to make sure there is no mistake in identifying the UART port is to disconnect all other UART to USB devices other than this EVM before checking in device manager.

<!-- ###AM263X-LP
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

  - We use this serial port, as seen in the device manager, for below in the SDK
    - Flashing application via UART
    - Booting application via UART
    - Console output for examples

      \imageStyle{ccs_uart_02.png,width:25%}
      \image html ccs_uart_02.png "Connect to UART port"

  - In this screenshot this happens to be COM5/COM11 however on your machine this could be different.
    One tip to make sure there is no mistake in identifying the UART port is to disconnect all other UART to USB devices other than this EVM before checking in device manager. -->

## Additional Details

### BOOT MODE

#### OSPI BOOT MODE  {#BOOTMODE_QSPI}

##### AM263PX-CC
This mode is used to boot flashed applications via EVM flash like OSPI flash
\code
BOOTMODE [ 1 : 4 ] (SW6) = 1100
\endcode

  \imageStyle{boot_pins_ospi_mode.jpg,width:30%}
  \image html boot_pins_ospi_mode.jpg "OSPI BOOT MODE"

<!-- ##### AM263X-LP
This mode is used to boot flashed applications via EVM flash like QSPI flash

  \imageStyle{am263x_lp_boot_pins_qspi_mode.png,width:30%}
  \image html am263x_lp_boot_pins_qspi_mode.png "QSPI BOOT MODE" -->

#### UART BOOT MODE  {#BOOTMODE_UART}

##### AM263PX-CC
This mode is used to flash files to the EVM flash via UART. It can also be used to boot applications via UART.

\code
BOOTMODE [ 1 : 4 ] (SW6) = 1000
\endcode

  \imageStyle{boot_pins_uart_mode.jpg,width:30%}
  \image html boot_pins_uart_mode.jpg "UART BOOT MODE"

<!-- ##### AM263X-LP
This mode is used to flash files to the EVM flash via UART. It can also be used to boot applications via UART.

  \imageStyle{am263x_lp_boot_pins_uart_mode.png,width:30%}
  \image html am263x_lp_boot_pins_uart_mode.png "UART BOOT MODE" -->

#### DEVBOOT MODE  {#BOOTMODE_NOBOOT}

##### AM263PX-CC
This mode is used in CCS.
    \code
    BOOTMODE [ 1 : 4 ] (SW6) = 1101
    \endcode

  \imageStyle{boot_pins_noboot_mode.jpg,width:30%}
  \image html boot_pins_noboot_mode.jpg "NO BOOT MODE"

<!-- ##### AM263X-LP
This mode is used in CCS.

  \imageStyle{am263x_lp_boot_pins_noboot_mode.png,width:30%}
  \image html am263x_lp_boot_pins_noboot_mode.png "NO BOOT MODE" -->
