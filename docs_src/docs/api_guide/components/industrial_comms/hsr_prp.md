# HSR_PRP(High-availability seamless Redundancy/Parallel Redundancy Protocol) FWHAL {#HSR_PRP_FWHAL}

[TOC]

## Introduction

This software is designed for the TI SoCs with PRU-ICSS IP to enable customers add HSR/PRP(High-availability seamless Redundancy and Parallel Redundancy Protocol) Dual Attached Node support to their system. It implements HSR/PRP functionality and provides HSR/PRP FPGA like functionality integrated into TI SoCs.

HSR and PRP are Ethernet based communication technology commonly deployed in smart grid substation for low cost, easy to maintain and interoperable common network infrastructure with built-in redundancy. They exist as a LRE(Link Redundancy Entity) between the MAC and Network layer above providing a transparent view of the MAC below to the application. Every packet is duplicated by the HSR/PRP FWHAL on it's way out and only a single copy of the duplicates arriving is forwarded to Host.

\image html HSR_PRP_Software_Architecture.png "Software Architecture"

HSR/PRP firmware for PRU-ICSS is a black box product maintained by TI. HSR/PRP FWHAL(Firmware and Hardware Abstraction Layer) allows loading and running HSR/PRP firmware and acts as an interface with the firmware. Firmware is based on 200 MHz clock frequency for PRU-ICSS Core Clock and IEP Clock.

## Data Sheet
### Features Supported

<table>
<tr>
    <th>Feature
    <th>Description
    <th>Implementation
</tr>
<tr>
    <td>High-Availability Seamless (HSR as per clause 5 of IEC 62439)
    <td>Redundancy Operates as a DANH, Support for modes – H, T, U and N as per standard, Two ports as per standard(Port A & Port B), Modes can be changed at runtime
    <td>Yes
</tr>
<tr>
    <td>Parallel Redundancy Protocol (PRP as per clause 4 of IEC 62439)
    <td>Operates as DANP, Two ports as per standard(Port A & Port B)
    <td>Yes
</tr>
<tr>
    <td>Quality of Service (QoS)
    <td>Two priority receive queues on host port(6 KB), Four priority transmit queues on each physical port(3 KB).
    <td>Yes
</tr>
<tr>
    <td>Node Table
    <td>256 entries, Hash Table for faster lookup(Complexity : O(1)), Node forget time : 60s, Node Table statistics
    <td>Yes
</tr>
<tr>
    <td>Multicast Filtering
    <td>Support for Multicast Packet Filtering, Hash Table for faster lookup(Complexity : O(1))
    <td>Yes
</tr>
<tr>
    <td>VLAN Filtering
    <td>Support for VLAN Filtering for host port, Hash Table for faster lookup(Complexity : O(1))
    <td>Yes
</tr>
<tr>
    <td>Duplicate Discard Table
    <td>Duplicate discard on Port to Host path (PRP), Data integrity (CRC) check during port to port forwarding
    <td>Yes
</tr>
<tr>
    <td rowspan=2>Duplex <td>Full</td> <td>Yes</td>
</tr>
<tr>
    <td>Half</td> <td>No</td>
</tr>
<tr>
    <td rowspan=3>Speed</td> <td>Auto</td> <td>Yes</td>
</tr>
<tr>
    <td>10Mbps</td> <td>No</td>
</tr>
<tr>
    <td>100Mbps</td> <td>Yes</td>
</tr>
<tr>
    <td>Statistics
    <td>Supports all MIB statistics as per standard, Node Table statistics for debugging, Self-configuring
    <td>Yes
</tr>
<tr>
    <td>PTP/1588 – Time Synchronization
    <td>PTP, Supports P2P clock, PTP over 802.3 (Annex F), Transparent Clock supported, Ordinary Clock supported, Single and Two step clock supported, Peer delay Response is always sent as two-step
    <td>Yes
</tr>
<tr>
    <td>Port Buffering
    <td>1 ms buffering per port
    <td>Yes
</tr>
<tr>
    <td>Storm Prevention
    <td>Multicast and Broadcast storm prevention per port
    <td>Yes
</tr>
<tr>
    <td>Interrupt for Link loss detection
    <td>Link loss detection, Callback APIs to perform tasks related to change in network topology
    <td>Yes
</tr>
</table>

### Known Issues

<table>
<tr>
    <th> Record ID
    <th> Details
    <th> Workaround
</tr>
<tr>
    <td> PINDSW-5290
    <td> Frame drops due to LWIP pool reduction
    <td>
\cond SOC_AM64X
    Add following code in source/networking/lwip/lwip-config/am64x/lwippools.h file :
    LWIP_MALLOC_MEMPOOL(100, 256) LWIP_MALLOC_MEMPOOL(50, 512) LWIP_MALLOC_MEMPOOL(50, 1024) LWIP_MALLOC_MEMPOOL(50, 1792).
\endcond

\cond SOC_AM243X
    Add following code in source/networking/lwip/lwip-config/am243x/lwippools.h file :
    LWIP_MALLOC_MEMPOOL(100, 256) LWIP_MALLOC_MEMPOOL(50, 512) LWIP_MALLOC_MEMPOOL(50, 1024) LWIP_MALLOC_MEMPOOL(50, 1792).
\endcond
</tr>
<tr>
    <td> MCUSDK-4379
    <td> Low Tx side throughput seen when tested using iperf application
    <td>
\cond SOC_AM64X
    Replace mcu_plus_sdk\source\networking\lwip\lwip-config\am64x\lwipopts.h and mcu_plus_sdk\source\networking\lwip\lwip-config\am64x\lwippools.h from MCU PLUS SDK 8.2.0 release and rebuild lwip_freertos, lwip-contrib and icss_emac_lwip_if libraries.
\endcond

\cond SOC_AM243X
    Replace mcu_plus_sdk\source\networking\lwip\lwip-config\am243x\lwipopts.h and mcu_plus_sdk\source\networking\lwip\lwip-config\am243x\lwippools.h from MCU PLUS SDK 8.2.0 release and rebuild lwip_freertos, lwip-contrib and icss_emac_lwip_if libraries.
\endcond
</tr>
<tr>
    <th> MCUSDK-8234
    <th> HSR/PRP - PTP Device is unable to keep offset under 1000 ns
    <th> -
</tr>
<tr>
    <th> MCUSDK-8236
    <th> HSR/PRP is not functional in rgmii mode
    <th> -
</tr>
</table>

## Important Files and Directory Structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/industrial_comms</td></tr>
<tr>
    <td>hsr_prp_demo</td>
    <td>HSR/PRP Examples (based on pre-integrated stack)</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/industrial_comms/hsr_prp</td></tr>
<tr>
    <td>icss_fwhal/firmware</td>
    <td>Firmware for the PRU cores in PRU-ICSS. **Firmware Version : 2.20.31** </td>
</tr>
<tr>
    <td>icss_fwhal/lib/</td>
    <td>FWHAL library for HSR/PRP</td>
</tr>
<tr>
    <td>icss_fwhal/*.c</td>
    <td>FWHAL source files</td>
</tr>
<tr>
    <td>icss_fwhal/*.h</td>
    <td>FWHAL interface files</td>
</tr>
</table>

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
    <td>HSR</td>
    <td>High-availability seamless Redundancy Protocol (A redundancy protocol)</td>
</tr>
<tr>
    <td>DANH</td>
    <td>Dual Attached Node (HSR)</td>
</tr>
<tr>
    <td>PRP</td>
    <td>Parallel Redundancy Protocol (A redundancy protocol)</td>
</tr>
<tr>
    <td>DANP</td>
    <td>Dual Attached Node (PRP)</td>
</tr>
<tr>
    <td>LRE</td>
    <td>Link Redundancy Entity</td>
</tr>
<tr>
    <td>MII</td>
    <td>Media Independent Interface</td>
</tr>
<tr>
    <td>DL</td>
    <td>Datalink Layer</td>
</tr>
<tr>
    <td>PTP-1588</td>
    <td>Precision Time Protocol (IEEE time synchronization protocol)</td>
</tr>
</table>

## API Documentation

Please see \ref INDUSTRIAL_COMMS_HSR_PRP_FWHAL_MODULE for API documentation.

## Procedure to kick-off the PRP

- Initialize ICSS-EMAC to work as a switch
- Initialize the required tasks and interrupts
- Initialize the HSR/PRP FWHAL and PRU-ICSS INTC
- Load HSR/PRP firmware into PRUs of PRU-ICSS
- Start firmware
- Handle the events as needed.


## Integration with ICSS-EMAC

TI HSR/PRP solution uses the ICSS-EMAC as it's base switch layer. The PRU Firmware is customized for HSR/PRP functionalities, and is not same as a standard switch firmware. All the traffic to Host is handled by HSR/PRP LRE (Link redundancy entity). The NRT(non-real time) traffic is passed to the LwIP stack while the Real time traffic goes to the registered callback.

### Interface with ICSS-EMAC

HSR/PRP packets are standard ethernet frames and all protocol specific data is embedded in TCP/IP payload. On a standard ICSS-EMAC Switch driver, of the 4 queues, two queues are dedicated to each receiving port, that is packet from \ref ICSS_EMAC_QUEUE1, \ref ICSS_EMAC_QUEUE3 go to RT callback and packets from \ref ICSS_EMAC_QUEUE2, \ref ICSS_EMAC_QUEUE4 go to NRT callback. We need to set "RT/NRT Priority Separation Queue" to (QUEUE 2) in SysConfig for ICSS-EMAC.

The packets are forwarded to based on the priority of the packet which is decided by the queue number (refer to \ref ICSS_EMAC_DESIGN_QOS). The driver decides to forward high priority packets to the `rxRTCallBack` and the rest are forwarded to LwIP stack, done by `rxNRTCallBack`.

### Callbacks from ICSS EMAC
HSR/PRP FWHAL provides custom Rx and Tx functions to be used for HSR/PRP application. This needs to be configured while initializing ICSS EMAC driver. The custom Rx and Tx functions override the default Rx and Tx functions (\ref ICSS_EMAC_rxPktGet and \ref ICSS_EMAC_txPacket respectively) provided by ICSS-EMAC driver.

<table>
<tr>
    <th>Callback Name
    <th>Description
    <th>Function used for callback registration
</tr>
<tr>
    <td> Custom Rx
    <td> The callback handles the PRP trailer present it in the message and removes it for the upper layers, presenting a transparent interface.
    <td> \ref RedRxPktGet
</tr>
<tr>
    <td> Custom Tx
    <td> The callback inserts the redundancy tags in the frame, duplicates packets on both ports and attempts to send them out at the same time.
    <td> \ref RedTxPacket
</tr>
<tr>
    <td> Rx RT
    <td> This callback is used for processing high priority packets like PTP. It is implemented in the application itself as an example.
    <td> `hsrprp_processHighPrioFrames`
</tr>
<tr>
    <td> Rx NRT
    <td> This callback is used to send all the non real-time packets to lwIP stack.
    <td> `Lwip2Emac_serviceRx`
</tr>
</table>

## Interrupts
HSR/PRP firmware generates the following interrupts.

HSR/PRP implementation uses following interrupts mapped to Host Interrupt Controller. 8 Host Interrupts (Host Interrupts 2 through 9) are exported from the PRU_ICSSG internal INTC for signaling the device level interrupt controllers. PRU_EVTOUT0 to PRU_EVTOUT7 correspond to these eight interrupts in the following table. Please check \ref PRUICSS_INTC section for more details.

<table>
<tr>
    <th>Name
    <th>Host Interrupt
    <th>Description
</tr>
<tr>
    <td> Frame Receive
    <td> PRU_ICSS_EVTOUT0
    <td> Notifies host when firmware has stored a frame in host receive queue
</tr>
<tr>
    <td> Tx Callback Interrupt
    <td> PRU_ICSS_EVTOUT3
    <td> Raised when a PTP/1588 frame which requires Tx Timestamping is sent out
</tr>
<tr>
    <td> Link Status
    <td> PRU_ICSS_EVTOUT6
    <td> Interrupt is raised when the Link on MII0/MII1 port comes up or goes down
</tr>
</table>

### Tasks
The HSR/PRP FWHAL uses following tasks

- `RedLifeCheckTask` : Check for Link up, prepare a supervision frame and then send them periodically. This is done by waiting on a semaphore redLifeCheckSemaphore which is in turn posted by a Timer. The task initialization is done inside \ref RedLifeCheckTaskCreate.

- `RedNodetableRefresh` : Increments the `timeLastSeenX` values by 1 every 10 ms and delete a node table entry if it reaches `NODE_FORGET_TIME`. The task initialization is done inside \ref RedLifeCheckTaskCreate.

### Semaphores
The HSR/PRP FWHAL uses following semaphores

- redLifeCheckSemaphore : redLifeCheckSemaphore to gate the sending of supervision frames. A timer interrupt upon expiry posts the semaphore thus enabling periodic transmission of supervision frames.Semaphore created in \ref RedLifeCheckTaskCreate.

- nodesTableSemaphore : nodesTableSemaphore to gate the increments of `timeLastSeenX` values by 1 every 10 ms and delete a node table entry if it reaches `NODE_FORGET_TIME`. Semaphore created in \ref RedLifeCheckTaskCreate.

### Clocks
The HSR/PRP FWHAL uses following clocks

- redPruCheckTimer : This timer is used to periodically check and clear bits in Firmware implementation. ISR is `RedPruCheckTimerHandler`. Period is specified by `RED_PRU_CHECK_TIMER_PERIOD` (microseconds).

- redLifeCheckTimer : This timer is used to send supervision frames periodically. ISR is `RedLifeCheckTimerHandler` where it posts a semaphore redLifeCheckSemaphore to indicate to a waiting task to send supervision frames. Period is specified by `RED_LIFE_CHECK_TIMER_PERIOD` (microseconds).

## See also

- [HSR/PRP Demo](\ref EXAMPLES_INDUSTRIAL_COMMS_HSR_PRP_DEMOS)