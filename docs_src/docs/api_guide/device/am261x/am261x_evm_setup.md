#  EVM Setup {#EVM_SETUP_PAGE}

[TOC]

## EVM Overview

  \imageStyle{am261x_lp_overview.png,width:50%}
  \image html am261x_lp_overview.png "@VAR_LP_BOARD_NAME"

### AM261X-LP

## Cable Connections {#EVM_CABLES}

Important cable connections, ports and switches.

### AM261X-LP

- Bootmode selection

  \imageStyle{am261x_lp_bootswitch.png,width:20%}
  \image html am261x_lp_bootswitch.png "@VAR_LP_BOARD_NAME bootmode"

## Setup UART Terminal {#CCS_UART_TERMINAL}

###AM261X-LP
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

### BOOT MODE

#### OSPI BOOT MODE  {#BOOTMODE_OSPI}

##### AM261X-LP
This mode is used to boot flashed applications via EVM flash like OSPI flash

  \imageStyle{am261x_lp_boot_pins_ospi.png,width:30%}
  \image html am261x_lp_boot_pins_ospi.png "OSPI BOOT MODE"

#### UART BOOT MODE  {#BOOTMODE_UART}

##### AM261X-LP
This mode is used to flash files to the EVM flash via UART. It can also be used to boot applications via UART.

  \imageStyle{am261x_lp_boot_pins_uart_mode.png,width:30%}
  \image html am261x_lp_boot_pins_uart_mode.png "UART BOOT MODE"

#### DEVBOOT MODE  {#BOOTMODE_NOBOOT}

##### AM261X-LP
This mode is used in CCS.

  \imageStyle{am261x_lp_boot_pins_noboot_mode.png,width:30%}
  \image html am261x_lp_boot_pins_noboot_mode.png "NO BOOT MODE"

#### DFU BOOT MODE  {#BOOTMODE_DFU}

##### AM261X-LP
This mode is used with DFU bootloader and DFU Uniflash.

  \imageStyle{am261x_lp_boot_pins_dfu_mode.png,width:30%}
  \image html am261x_lp_boot_pins_dfu_mode.png "DFU BOOT MODE"
