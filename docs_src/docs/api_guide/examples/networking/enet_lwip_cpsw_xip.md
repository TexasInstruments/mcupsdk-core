# Enet Lwip CPSW XIP Example{#EXAMPLES_ENET_LWIP_CPSW_XIP}

[TOC]

# Introduction

\note lwIP features are made available as is from public lwIP project. SDK configuration may only enable and exercise a subset of these features.

This example is an Execute-In-Place (XIP) variant of the standard Enet Lwip CPSW example (\ref EXAMPLES_ENET_LWIP_CPSW). It demonstrates a TCP/UDP IP application using the LwIP networking stack coupled with ethernet driver (ENET), where the application code executes directly from flash memory instead of being copied to RAM.

## What is XIP (Execute-In-Place)?

XIP allows the processor to execute code directly from external flash memory (OSPI) without first copying it to internal RAM. This provides several benefits:
- **Reduced RAM Usage**: Code sections remain in flash, freeing up RAM for data and buffers
- **Faster Boot Time**: Less data to copy during boot
- **Larger Code Size Support**: Applications can be larger than available RAM

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism:
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with LwIP TCP/IP networking stack

The example does the following:
- Initializes the ethernet driver for the underlying HW
- Initializes the LwIP stack for TCP/UDP IP
- Configures the application with XIP support, placing `.text` and `.rodata` sections in flash
- Allows user to run and test basic networking features like DHCP, ping, iperf with TCP/UDP

\cond SOC_AM263PX
\note This example cannot be run using CCS boot mode. Flash the image using UART/QSPI boot mode via the uniflash script.
\endcond

\cond SOC_AM261X
\note This example cannot be run using CCS boot mode. Flash the image using UART/OSPI boot mode via the uniflash script.
\endcond

# Supported Combinations

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/lwip/enet_lwip_cpsw_xip

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/lwip/enet_lwip_cpsw_xip

\endcond

# Key Differences from Standard enet_lwip_cpsw Example

The XIP variant has the following key differences:

## Memory Configuration

- **Code Placement**: `.text` section is placed in FLASH region (starting at 0x60100000) instead of OCRAM
- **Read-Only Data**: `.rodata` section is also placed in FLASH
- **Additional MPU Region**: MPU Region 5 (0x60000000, 256MB) is configured to allow code execution from flash
- **Reduced Flash Size for Non-XIP**: FLASH region for non-XIP sections is reduced from 0x80000 to 0x40000 bytes
- **Additional Sections**: `.cfg.rodata` and `cio` sections are also configured for XIP

## Build Artifacts

The build process generates two binary files:
- **`<filename>.mcelf`**: Main bootable image containing critical code and data loaded to RAM
- **`<filename>.mcelf_xip`**: XIP image containing code and read-only data that executes from flash

## Flashing Procedure

Both images must be flashed:
- The `.mcelf` file is flashed at offset 0x81000 using `--operation=flash-sector-write`
- The `.mcelf_xip` file is flashed using `--operation=flash-mcelf-xip` which places sections at addresses specified within the image

## Performance Considerations

- **RAM Savings**: Significant reduction in RAM usage as code executes from flash
- **Execution Speed**: Flash execution may be slower than RAM execution, but OSPI performance is optimized
- **Time-Critical Code**: Interrupt handlers and critical functions remain in OCRAM for performance

# Packet Pool Configuration
To change packet pool configuration from syscfg, please refer to \ref PACKETPOOL_CONFIG_TOP

# Configuring Syscfg

Following Syscfg option allows flexibility to configure memory footprint based on the required use case like: Number of DMA descriptors and buffering.

- Supported Options with default configuration

<table>
<tr>
    <th>Feature
    <th>Section
    <th>Description
    <th>Remarks/Default Setting
</tr>

<tr>
    <td>Mdio Manual Mode Enable
    <td>TI Networking / Enet (CPSW) / MDIO Config
    <td>Flag to enable MDIO manual mode in example.\n Driver support for Manual mode is enabled, so this parameter configures manual mode in the example.
    <td>Default is true.\n If your silicon is affected with errata <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf" target="_blank">i2329— MDIO interface corruption</a>, then TI suggests to use MDIO_MANUAL_MODE as software workaround.
</tr>

\cond SOC_AM263PX
<tr>
    <td>Disable Mac Port1, Disable Mac Port2
    <td>TI Networking / Enet (CPSW) / MAC Port Config
    <td>Select which port to disable.
    <td>Default is Port2 enabled.
</tr>
\endcond

\cond SOC_AM261X
<tr>
    <td>Disable Mac Port1, Disable Mac Port2
    <td>TI Networking / Enet (CPSW) / MAC Port Config
    <td>Select which port to disable.
    <td>Default is Port2 enabled.
</tr>
\endcond

<tr>
    <td>Enable Packet Pool Allocation
    <td>TI Networking / Enet (CPSW) / Packet Pool Config
    <td>Flag to enable packet buffer memory allocation from enet utils library. It should be disabled to avoid utils memory wastage, in case application allots packet via other mechanism.
    <td>Default is true.\n If enabled, size of pkt pool size depends on 'Large Pool Packet Size', 'Large Pool Packet Count', 'Medium Pool Packet Size', 'Medium Pool Packet Count', 'Small Pool Packet Size' and 'Small Pool Packet Count'. EnetMem_allocEthPkt API uses this memory to allocate the DMA Ethernet packet.
</tr>

<tr>
    <td>Only Enable Packet Info Allocation
    <td>TI Networking / Enet (CPSW)
    <td>Flag to allocate only the DMA Packet Info structures, this does not include the buffer memory. This is useful when the buffer memory is internally allocated by the application. (Ex- Lwip pools)
    <td>Default is true.\n If enabled, "PktInfoMem Only Count" determines the number of additional DMA Packet Info structures allocated. EnetMem_allocEthPktInfoMem uses this memory to allocate empty DMA Packet Info structures.
</tr>

<tr>
    <td>Number of Tx Packet
    <td>TI Networking / Enet (CPSW) / DMA channel config
    <td>No of Tx packets required for DMA channel
    <td>Default is 16.\n For LwIP example, the Tx packet buffer memory is internally allocated in lwippools.h. Only the DMA Pkt Info structures are allocated via sysCfg, so this number should match the "PktInfoMem Only Count" described in the above item. To increase the Tx packet count, user needs to update the number correspondingly at "PktInfoMem Only Count" and lwippools.h and build the libs.
</tr>

<tr>
    <td>Number of Rx Packet
    <td>TI Networking / Enet (CPSW) / DMA channel config
    <td>No of Rx packets required for DMA channel
    <td>Default is 32.\n It contributes to the size of Pkt Mem Pool, DMA ring buffer and accessories size. Rx packet buffer memory is completely managed with application sysCfg, this is done by using Rx custom Pbuf in LwIP.
</tr>

<tr>
    <td>Netif instance
    <td>TI Networking / Enet (CPSW) / LWIP Interface config
    <td>No of netifs allocated by the example.\n Only one netif should be set to default when more than one netif is allocated.
</tr>
</table>

## XIP-Specific Configuration

The XIP example includes additional memory configuration in syscfg:

- **MPU Region for Flash**: CONFIG_MPU_REGION5 is configured for base address 0x60000000 with size 256MB (2^28) to enable code execution from flash
- **Flash Memory Region**: Configured for 0x40000 bytes at 0x60100000 (reduced from 0x80000 in standard example)
- **Code and Read-Only Data Sections**: `.text` and `.rodata` sections are placed in FLASH region instead of OCRAM
- **Additional Text Sections**: `.cfg.rodata` and `cio` sections are also placed in FLASH for XIP

# Iperf using LWIP

- Iperf is a tool for network performance measurement and tuning. It is a cross-platform tool that can produce standardized performance measurements for any network. Iperf has client and server functionality, and can create data streams to measure the throughput between the two ends in one or both directions.

- Iperf version to be used is version 2.0.9-win64(on windows) and version 2.+ on linux(Ubuntu 18.04 64bit).

- The lwip app has iperf enabled by default. To disable the iperf functionality, disable the macro (LWIP_LWIPERF_APP) in the application file lwipcfg.h under path (source/networking/enet/core/examples/lwip/enet_lwip_cpsw_xip).

- The data streams can be either Transmission Control Protocol (TCP) or User Datagram Protocol (UDP).

 - UDP: When used for testing UDP capacity, iperf allows the user to specify the datagram size and provides results for the datagram throughput and the packet loss.
 - TCP: When used for testing TCP capacity, iperf measures the throughput of the payload. Iperf uses 1024 × 1024 for mebibytes and 1000 × 1000 for megabytes.

## Important Iperf Arguments:

 Argument      | Meaning
 ---------------|-----------
 -s             | Run server
 -c             | Run Client [Ex: -c 192.168.1.102]
 -u             | UDP
 -b             | Bandwidth [Used in UDP, Ex: 100M M->Mbits]
 -i             | Output interval in Sec [Ex: -i1 1sec interval]
 -t             | Time in sec [Ex: -t60 60sec]
 -p             | Port number [Ex: -p 5555]
 -w             | Windows size [Ex: -w 1M M->Mbits]
 -d             | Bi-directional traffic
 -l             | Length [Ex: -l 1046 1046bytes]
 -V             | Used when IPv6 address is used instead of IPv4


# Steps to Run the Example

## To Configure Static IP
Modify code in file `lwipcfg.h` file as below to set USE_DHCP and USE_AUTOIP as '0'
\code
/* uncomment the next two lines for Static IP */
-//#define USE_DHCP     0
-//#define USE_AUTOIP  0
+#define USE_DHCP     0
+#define USE_AUTOIP  0
\endcode

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.

- Connect an Ethernet cable to the SOM from host PC to the appropriate RGMII port

## Create a network between EVM and host PC

- The EVM will get an IP address using DHCP, so make sure to connect the other end of the cable
to a network which has a DHCP server running.

- To get started, one can create a simple local network
  between the EVM and the host PC by using a home broadband/wifi router as shown below.
  Most such routers run a DHCP server

  \imageStyle{lwip_example_01.png,width:30%}
  \image html lwip_example_01.png Local network between PC and EVM

- To check the router connection with host PC, recommend to disconnect all other networking connections
  on the PC, sometimes you may need to disable firewall SW, and make sure the router is able
  to assign a IP address to your host PC

- After we run the example on the EVM (next step), the EVM will similarly be assigned a IP address, and then host
  can communicate with the EVM using the assigned IP address.

## Flash the example

\attention XIP applications require special flashing procedure. Both the main image and XIP image must be flashed.

- Refer to \ref GETTING_STARTED_FLASH for more details on flashing applications.

## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- After flashing, reset the EVM
- You will see logs in the UART terminal as shown in the next section
- Note the IP address seen in the log, this is what we will use to communicate with the EVM

## Sample output for CPSW XIP example

\code

==========================
      ENET LWIP App
==========================
EnetPhy_bindDriver:
PHY 0 is alive
Starting lwIP, local interface IP is dhcp-enabled
Host MAC address: 70:ff:76:1d:ec:f2
[LWIPIF_LWIP] Enet has been started successfully
[LWIPIF_LWIP] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 0.0.0.0
UDP server listening on port 5001
Cpsw_handleLinkUp:
MAC Port 1: link up
link_callback==UP
status_callback==UP, local interface IP is 192.168.1.100
      5. 34s : CPU load =   2.04 %


\endcode

## Communicate with the EVM using ethernet

- Firstly you can try to reach the EVM using ping as shown below, using a command shell on the host PC

        > ping 192.168.1.100

- Next you can run `iperf` tests as shown below. Below steps have been tried with a Linux Ubuntu 18.04 host PC running bash shell

 - Install `iperf` if not installed by doing below

        > sudo apt install iperf

 - Invoke iperf to test TCP bi-directional RX+TX connection as shown below
    \code
    > iperf -c 192.168.1.100 -i 5 -t 20 -d
    \endcode

## Measuring the throughput using Iperf:

- Once we get the IP after running the example, we can use following iperf command on windows to get the throughput.
	- iperf.exe -c 192.168.1.100 -r

# Performance

Refer this page for XIP performance benchmarking: \ref networking_xip_performance

## Troubleshooting issues

\cond SOC_AM261X
- For @VAR_LP_BOARD_NAME_LOWER, Software configures MDIO based on Board version(E1/E2) which is read from EEPROM. It expects
  on-board EEPROM to be pre-programmed for E2 EVMs. If EEPROM(0x51) is not programmed, the software considers
  EVM as "E1" version.
\endcond

\cond SOC_AM263PX
- To ensure compatibility with CPSW RGMII2 port on @VAR_BOARD_NAME_LOWER board, make sure that SW-15(switch) should be set to `LOW` for board versions(E2/A/B versions). This configuration ensures seamless out-of-box support and functionality with the CPSW RGMII2 port on the specified versions.
\endcond

- If you see a valid, non-zero MAC address but IP address is `0.0.0.0` then
   - Make sure you see `link_callback==UP` message, if not check the ethernet cable
   - Check the local network and check if the DHCP server is indeed running as expected
   - When using a home broadband/wifi router, its possible to check the clients connected to the DHCP server via a web
     browser. Check your router user manual for more details.

- If the application fails to boot or behaves unexpectedly:
   - Verify that both `.mcelf` and `.mcelf_xip` files are flashed correctly
   - Check that the OSPI flash is properly configured and functioning
   - Ensure the SBL supports MCELF XIP format (SDK 11.00 and later for AM263PX, AM261X)
   - Verify the flash configuration file paths match your build output

- Performance considerations:
   - XIP execution from flash may be slower than RAM execution for compute-intensive operations
   - Network throughput should be comparable to non-XIP version as packet processing remains in RAM
   - If performance is critical, consider moving time-critical functions to RAM sections

# See Also

\ref NETWORKING
\ref EXAMPLES_ENET_LWIP_CPSW