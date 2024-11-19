# SBL QSPI ENET {#EXAMPLES_DRIVERS_SBL_QSPI_ENET_AM263X}

[TOC]

# Introduction

This bootloader performs SOC initializations and offers additional functionality, including: 
1. Receiving an application images via UDP over Ethernet.
2. Flashing the received application to QSPI Flash Memory.
3. Attempting to boot a multicore appimage present at 0x800000 location in the QSPI Flash. This offset is specified in the QSPI bootloader and when the EVM boots in QSPI mode, it will attempt to find a application at this location.

To flash a multicore appimage at this location, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES.

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath

The transfer of the user application image works in conjunction with the enet_uniflash.py python script mentioned in \ref TOOLS_FLASH.

If a multicore appimage is found at the location, the SBL parses it, splits it into RPRCs for each core applicable. Each core is then initialized, RPRC image is loaded, entry points are set and the core is released from reset. For more on bootflow/bootloaders, please refer \ref BOOTFLOW_GUIDE

Note that the SBL is transferred solely through UART, whereas the files that the SBL processes are transmitted over Ethernet.

\note To ensure the sbl_qspi_enet SBL remains intact, allocate a reserved area of 0x13C000 in MSRAM within the application image's linker script. This precaution prevents the application image from overwriting the SBL during Ethernet-based flashing.
\note MSRAM specifications:
1. Total size: 0x0200000
2. HSMRT module allocation: 0x40000
3. ENET libraries/components allocation approximately: 0x7C000
4. Scratch pad allocation: 0x8000 
\note The remaining 0x13C000 space is reserved for the application image.

# Supported Combinations

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_qspi_enet

## Application Flow

The below flow diagram shows the application flow for the reception of an application image via UDP over ethernet on boot of the sbl_qspi_enet SBL.

  \imageStyle{sbl_qspi_enet_flow_diagram.png,width:30%}
  \image html sbl_qspi_enet_flow_diagram.png Flow Path

This SBL application consumes more MSRAM memory (>0x20000) as compared to the sbl_uart_uniflash SBL due to the increased size of the ethernet driver library.

## Create a network between EVM and host PC

### Linux (Ubuntu 22.04):

The instructions provided below have been validated on Ubuntu(v22.04), but are also applicable to other operating systems. The scripts included in MCU+SDK are designed to be compatible with multiple host PC operating systems, ensuring seamless execution across different platforms.

1. Setup the static IP on the interface connected to the board using the following command:

The below steps are for the network interface corresponding to the ethernet connection between the PC and EVM.

Note that the IP addresses utilized in this configuration are also referenced in the Python script and EVM application. To avoid potential conflicts with existing devices, it is essential to update the IP addresses consistently across all relevant components.

- Connect an ethernet cable between the PC and the EVM(Here in this case AM263 Control Card, Port 1)

- This will reflect all the interfaces connected to the linux host PC.
\code
ifconfig
\endcode

- static-IP is hardcoded in **sbl_enet.h** as:
\code
/* EVM IP ADDRESS */
#define ENET_SOURCE_IP_ADDRESS          { 192U, 168U, 0U, 195U }
/* Host PC IP Address */
#define ENET_DESTINATION_IP_ADDRESS     { 192U, 168U, 0U, 136U }
\endcode
\code
sudo ifconfig <interface-name> <static-HOST-IP>
\endcode

- Due to the absence of ARP services (typially provided by LwIP) in this Uniflash application, manual configuration of an ARP entry for the EVM is required. This ensures that the PC does not discard incoming packets from the EVM due to Unknown Unicast address.
\code
sudo arp -i <interface-name> -s <EVM-static-IP> <EVM-MAC-address>
\endcode

EVM MAC Address can be found in the SBL UART console. In this case it is: 70:ff:76:1d:ec:f2

  \attention Make sure to set the value of the MACRO **ENET_HOST_PC_MAC_ADDRESS** in the file **sbl_enet.h** to the MAC address of the corresponding ethernet interface of Linux Host PC:
\code
/* Host PC MAC Address */
#define ENET_HOST_PC_MAC_ADDRESS        { 0xC0, 0x18, 0x03, 0xBD, 0xB2, 0x7B}
\endcode

# Steps to Run the Example

Since this is a bootloader, the example will be run every time you boot an application using this example. It is run from a QSPI boot media unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Refer to the page \ref BASIC_STEPS_TO_FLASH_FILES to flash the sbl_qspi_enet bootloader to the EVM in UART Boot Mode.
  Ensure that you should flash the sbl_qspi_enet bootloader, rather than the default sbl_qspi bootloader. Optionally, you can also flash the application image to the QSPI flash memory at offset 0x80000 as mentioned in \ref BASIC_STEPS_TO_FLASH_FILES. Alternatively, you can omit this step and transmit the application image over Ethernet using the procedure outlined below.
- Once the sbl_qspi_enet bootloader image is flashed on the EVM, switch to QSPI Boot Mode and refer to the page \ref BASIC_STEPS_TO_FLASH_FILES_OVER_ENET to send an application image over ethernet to the EVM.
# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output {#SBL_QSPI_ENET_OUTPUT_SAMPLE}

Below is an example log from the serial teminal, demonstrating a normal boot process in ENETSBL_TIMER_MODE without image transfer. As shown, the bootloader bypasses the transfer step and proceeds to boot the user application image:

\code

[ ENETSBL ] Starting Ethernet Transfer ...
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:23 Ver:01 <-> 'dp83869' : OK
PHY 0 is alive
[ ENETSBL ] initQs() txFreePktInfoQ initialized with 16 pkts
[ ENETSBL ] EVM MAC address: 70:ff:76:1d:ec:f2
[ ENETSBL ] PHY 0 is alive
[ ENETSBL ] Please wait for Linkup ...
Cpsw_handleLinkUp: Port 1: Link up: 100-Mbps Full-Duplex
[ ENETSBL ] Linkup Done!
[ ENETSBL TIMEOUT ] Skipping enet transfer.
Cpsw_handleLinkDown: Port 1: Link down

Starting QSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 80.000 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 30 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        154us
[BOOTLOADER PROFILE] Drivers_open                     :         30us
[BOOTLOADER PROFILE] Board_driversOpen                :         57us
[BOOTLOADER PROFILE] CPU load                         :    8483203us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :    8483448us

Image loading done, switching to application ...
INFO: Bootloader_runSelfCpu:217: All done, reseting self ...

Hello World!

\endcode
