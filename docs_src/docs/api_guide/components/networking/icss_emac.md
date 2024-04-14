# ICSS-EMAC {#ICSS_EMAC}

[TOC]

## Introduction

The ICSS-EMAC (Industrial Communications Subsystem Ethernet Media Access Controller) driver provide APIs to transmit and receive packets with a firmware based Ethernet switch that has been implemented on PRUICSS cores.

ICSS-EMAC runs on the host processor, provides a well defined set of APIs to configure the firmware, send packets to the firmware and receive packet from the firmware. Industrial Communication Protocol specific firmware (running on PRUICSS cores) implements a 2 port ethernet switch supporting 802.1d at 100 Mbps.

\note ICSS-EMAC will be supported for use in Profinet Device, EtherNet/IP Adapter and PRP(Parallel Redundancy Protocol) examples only. \n ICSS firmware to support above protocols or EMAC (to add one or more Ethernet port) is not supported in this release.

## Features Supported

- Packet Receieve: Copying packet received from firmware and providing it to TCP/IP stack or upper layer protocols
- Packet Transmit: Providing packet from a TCP/IP stack or upper layer protocols to firmware
- Storm Prevention
- Host Statistics
- Learning/Forwarding Data Base
- Multicast/VLAN Filtering
- Interrupt handling for Rx, Tx and Link
- Task Creation for Rx, Tx and Link

## SysConfig Features {#ICSS_EMAC_SYSCONFIG_FEATURES}

@VAR_SYSCFG_USAGE_NOTE

- Selecting the PRU-ICSS instance
- Selecting port number
- Configuring PHY address for the ports
- Select queue threshold for separating real-time(RT) and non real-time(NRT) traffic
- Specify the size of buffer space
- Provide the task priorities for Rx, Tx and Link tasks

## Features not supported

- Time-triggered send (TTS)
- Firmware Learning mode
- Timer based Interrupt Pacing support

## Terms and Abbreviations

<table>
<tr>
    <th>Abbreviation
    <th>Expansion
</tr>
<tr>
    <td>PRU-ICSS</td>
    <td>Programmable Real-Time Unit Industrial Communication Subsystem</td>
</tr>
<tr>
    <td>EMAC</td>
    <td>Ethernet MAC</td>
</tr>
<tr>
    <td>Rx/Tx</td>
    <td>Packet Receive/Packet Transmit</td>
</tr>
<tr>
    <td>INTC</td>
    <td>Interrupt Controller</td>
</tr>
<tr>
    <td>DLR</td>
    <td>Device Level Ring (A redundancy protocol for EtherNet/IP)</td>
</tr>
<tr>
    <td>Host/CPU/Cortex</td>
    <td>Used interchangeably and refers to the Application Processor</td>
</tr>
</table>

## ICSS-EMAC Design

\subpage ICSS_EMAC_DESIGN explains the driver design in detail.

## Usage

### Enable ICSS-EMAC in SysConfig

- Enable ICSS-EMAC via SysConfig, by selecting `ICSS-EMAC` under `TI NETWORKING` in the left pane in SysConfig.
- Parameters mentioned in \ref ICSS_EMAC_SYSCONFIG_FEATURES should be configured as required.

### Update linker command file

- A shared memory region between CPU and PRU-ICSS needs to be allocated for queues. The size of this region can be configured using "Queue Buffer Size (KB)" option in SysConfig. The value should be configured according to the firmware which will be loaded on PRU-ICSS's cores. Based on the configured size, following code is generated in `ti_drivers_config.c` file. (This is an example, and exact code will vary based on the configured value in SysConfig).

  \code
  /* ICSS EMAC Packet Buffers */
  #define ICSS_EMAC_PKT_BUF_1_MEM_SIZE (65536)
  uint8_t gIcssEmacPktBufMem1[ICSS_EMAC_PKT_BUF_1_MEM_SIZE] __attribute__((aligned(128), section(".bss.  icss_emac_pktbuf_mem")));
  \endcode

- The linker command file should include this `.bss.icss_emac_pktbuf_mem` section and and map it to a memory region, as shown in below snippet. (Only relevant code sections which should be added are shown)

  \code
  MEMORY
  {
      /* shared memories that is used between ICCS and this core. MARK as cache+sharable */
      ICSS_PKT_BUF_MEM        : ORIGIN = 0x70000000, LENGTH = 0x00010000
  }
  SECTIONS
  {
      /* Packet buffer memory used by ICSS */
      .bss.icss_emac_pktbuf_mem (NOLOAD): {} > ICSS_PKT_BUF_MEM
  }
  \endcode

### Update MPU for the CPU

- The shared memory section (described in the above section) which was put in the linker command file needs to be mapped as `Cached+Sharable` for the CPU.

- This can be done via SysConfig, by adding additional MPU entries using the `MPU` module under `TI DRIVER PORTING LAYER (DPL)` in SysConfig. Following is an example:

  \image html icss_emac_mpu_config.PNG

### Including the header file

Include the below file to access the APIs
\snippet Icss_emac_sample.c icss_emac_include

### Initializing the Handle

- PRUICSS handle needs to be initialized before \ref ICSS_EMAC_open and passed using \ref ICSS_EMAC_Params.

- PRUICSS_IntcInitData should be passed based on the firmware. See \htmllink{@VAR_MCU_SDK_DOCS_PATH/DRIVERS_PRUICSS_PAGE.html#PRUICSS_INTC, PRUICSS Interrupt Controller} page for more details.

- ICSS-EMAC needs information about the memory map of firmware which will be used. User needs to provide the same information in the form of \ref ICSS_EMAC_FwStaticMmap and \ref ICSS_EMAC_FwDynamicMmap while filling the \ref ICSS_EMAC_Params structure. There are some other offsets related to VLAN filtering and multicast filtering, for which the structures \ref ICSS_EMAC_FwVlanFilterParams and \ref ICSS_EMAC_FwMulticastFilterParams should be passed if IOCTL for VLAN and multicast filtering are going to be called. For Profinet and EtherNet/IP firmware, these structures are defined in `${SDK_INSTALL_PATH}/source/industrial_comms/profinet_device/icss_fwhal/firmware/icss_emac_mmap.h` and `${SDK_INSTALL_PATH}/source/industrial_comms/ethernetip_adapter/icss_fwhal/firmware/icss_emac_mmap.h` respectively.

- Various callback functions can be set using \ref ICSS_EMAC_CallBackObject structure.

- Please see \ref ICSS_EMAC_Params for information on which members are mandatory. If mandatory options are not passed, DebugP_assert will fail in \ref ICSS_EMAC_open.

\snippet Icss_emac_sample.c icss_emac_open

### Sending a Packet

\snippet Icss_emac_sample.c icss_emac_tx

### Receiving a Packet

\ref ICSS_EMAC_rxPktGet is the API used for receving packets. Refer \ref ICSS_EMAC_DESIGN_DATA_PATH_RX for more details. Following is an example usage:

\snippet Icss_emac_sample.c icss_emac_rx

### IOCTL

IOCTL implementation is identical to the Unix/Linux based IOCTL calls. They provide the application a convenient method to access driver space parameters or modify them. Users are expected to familiarize themselves with the full list of IOCTL calls so that they can utilize all the features provided.

The API for IOCTL is \ref ICSS_EMAC_ioctl. An IOCTL call uses two parameters to find out which driver function to call internally.

1. `ioctlCommand` : Used to locate the module which should be called. Allowed values are \ref ICSS_EMAC_IOCTL_COMMANDS.
2. `ioctlParams` : Used to give module specific instructions. `ioctlParams` consists of
    - `command` : Indicates which specific command to execute. Allowed values for `ioctlParams.command` are \ref ICSS_EMAC_IOCTL_STORM_PREV_CTRL_COMMANDS, \ref ICSS_EMAC_IOCTL_LEARNING_CTRL_COMMANDS, \ref ICSS_EMAC_IOCTL_STATISTICS_COMMANDS, \ref ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_COMMANDS, \ref ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_COMMANDS.
    - `ioctlVal` : Sometimes the command may require specific input, this is used to provide that.

For example, to disable receive functionality on a port, following code is used.

\snippet Icss_emac_sample.c icss_emac_ioctl

## Dependencies

ICSS-EMAC is dependent on the Application/Transport layer for proper functioning. Following dependencies should be handled by the application layer (example) :

- MDIO Configuration : MDIO Initialization is not done by ICSS-EMAC.
- PHY Initialization/Configuration
- Learning Module Increment Counter Implementation : If learning is enabled, application needs to call IOCTL with \ref ICSS_EMAC_LEARN_CTRL_INC_COUNTER periodically.

## Debug Guide

\subpage ICSS_EMAC_DEBUG_GUIDE covers the most obvious use cases and debug scenarios encountered while using ICSS-EMAC.

## API

\ref NETWORKING_ICSS_EMAC_MODULE
