#  EVM Setup {#EVM_SETUP_PAGE}

[TOC]

## AM62X-SK

\note Refer to EVM page for more details on the EVM, https://www.ti.com/tool/SK-AM62

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
  - We use the 1st USB serial port, as seen in the device manager, for uboot and Linux.
  - We use the 4th USB serial port, as seen in the device manager, as terminal output for examples which run from M4F

      \imageStyle{ccs_uart_02.png,width:25%}
      \image html ccs_uart_02.png "Connect to UART port"

  - In this screenshot this happens to be COM27 and COM30. However on your machine this could be different.
    One tip to make sure there is no mistake in identifying the UART port is to disconnect all other UART to USB devices other than this EVM before checking in device manager.

### SOC Initialization {#EVM_SOC_INIT}

Before any program can be loaded and run on the EVM, the SOC needs to be initialized.
Below sections describes the various options available for SOC initialization.

#### SOC Initialization Using SPL {#EVM_SOC_INIT_SPL}
\attention This is a recommended one time step that needs to be done before
           you can load and run programs via CCS

- Prepare a SD card with Linux image by following the \htmllink{https://software-dl.ti.com/processor-sdk-linux/esd/AM62X/latest/exports/docs/linux/Overview/Processor_SDK_Linux_create_SD_card.html, Processor SDK Linux - Create SD card} page.

- **POWER-OFF** the EVM

- Make sure below cables are connected as shown in \ref EVM_CABLES
  - Power cable
  - JTAG cable
  - UART cable

- Set EVM in SDCARD BOOT mode as shown below
  \imageStyle{boot_pins_sd_card_boot_mode.png,width:30%}
  \image html boot_pins_sd_card_boot_mode.png "SD CARD BOOT MODE"

- Insert the prepared SD card on the SD card slot.

- Setup UART terminals for Uboot/Linux and the M4 console as per \ref CCS_UART_TERMINAL section.

- **POWER-ON** the EVM

- Uboot and Linux should come-up on the UART terminal.

- While Linux is booting, the remoteproc should start M4 as shown below.
\code
[    6.124861] k3-m4-rproc 5000000.m4fss: assigned reserved memory node m4f-dma-memory@9cb00000
[    6.141637] k3-m4-rproc 5000000.m4fss: configured M4 for remoteproc mode
[    6.154843] k3-m4-rproc 5000000.m4fss: local reset is deasserted for device
[    6.170602] remoteproc remoteproc0: 5000000.m4fss is available
[    6.196447] remoteproc remoteproc0: powering up 5000000.m4fss
[    6.202553] remoteproc remoteproc0: Booting fw image am62-mcu-m4f0_0-fw, size 78960
[    6.242673]  remoteproc0#vdev0buffer: assigned reserved memory node m4f-dma-memory@9cb00000
[    6.251498]  remoteproc0#vdev0buffer: registered virtio0 (type 7)
[    6.257790] remoteproc remoteproc0: remote processor 5000000.m4fss is now up
\endcode

- Setting up the board for Linux boot requires to be done only once with the EVM. But after every power cycle of the board, we need to wait for the linux to come up before loading binaries to AM62x M4 through CCS.

### Troubleshooting EVM issues {#TROUBLESHOOT_ISSUES}

 - JTAG connection fails on some EVMs with the following error. Need to connect the JTAG cable after board is powered on.

  \imageStyle{jtag_connection_error.PNG,width:30%}
  \image html jtag_connection_error.PNG "JTAG Connection Error Dialog"
