# EtherNet/IP Adapter FWHAL {#ETHERNETIP_ADAPTER_FWHAL}

[TOC]

## Introduction

This software is designed for the TI SoCs with PRU-ICSS IP to enable customers add EtherNet/IP Adapter protocol support to their system. It implements EtherNet/IP + PTP + DLR functionality and provides EtherNet/IP ASIC like functionality integrated into TI SoCs.

\image html EtherNetIP_Adapter_Software_Architecture.PNG "Software Architecture"

EtherNet/IP firmware for PRU-ICSS is a black box product maintained by TI. EtherNet/IP Adapter FWHAL(Firmware and Hardware Abstraction Layer) allows loading and running EtherNet/IP firmware and acts as an interface with the firmware. FWHAL implements the key interface between EtherNet/IP Adapter firmware and EtherNet/IP Adapter stack.

## PRU-ICSS EtherNet/IP Adapter Firmware

### Features Supported

- Integrated two-port cut-through switch
    - 100 Mbps, 10Mbps
    - Full Duplex, Half Duplex
- Quality of Service (QoS)
    - Four priority receive queues on host port, each queue 6 KB in size
    - Four priority transmit queues on each physical port, each queue 3 KB in size
- DLR – Device Level Ring
    - 200us beacon interval
    - 400us beacon timeout
    - Self-Configuring
- PTP/1588 - Time Synchronization
    - E2E mode supported
    - CIP Sync capable
    - Transparent and Ordinary Clock
- 1 ms buffering per port
- 802.1d learning bridge for received source MAC addresses
    - 1024 addresses per port
    - APIs for port state configuration and flushing learning table upon change in network topology
    - Switch address learned table (FDB) is flushed in 2.4 us
- Multicast and Broadcast storm prevention per port
- Interrupt for Link loss detection
    - Callback APIs to perform tasks related to change in network topology
- Statistics
    - Media counters supported per port
    - Interface counters supported per port
- LLDP Support
- Firmware based on 200 MHz clock frequency for PRU-ICSS Core Clock and IEP Clock

### Known Issues

<table>
<tr>
    <th> Record ID
    <th> Details
    <th> Workaround
</tr>
<tr>
    <td> PINDSW-4982
    <td> SQE and Carrier Sense Errors counters not supported in EtherNet/IP
    <td> -
</tr>

## Important Files and Directory Structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/industrial_comms</td></tr>
<tr>
    <td>ethernetip_adapter_demo</td>
    <td>EtherNet/IP Adapter Examples (based on pre-integrated stack)</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/industrial_comms/ethernetip_adapter</td></tr>
<tr>
    <td>icss_fwhal/firmware</td>
    <td>Firmware for the PRU cores in PRU-ICSS. **Firmware Version : 5.2.9** </td>
</tr>
<tr>
    <td>icss_fwhal/lib/</td>
    <td>FWHAL library for EtherNet/IP Adapter</td>
</tr>
<tr>
    <td>icss_fwhal/*.c</td>
    <td>FWHAL source files</td>
</tr>
<tr>
    <td>icss_fwhal/*.h</td>
    <td>FWHAL interface files</td>
</tr>
<tr>
    <td>stack</td>
    <td>Stack header files and stack library</td>
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
    <td>DLR</td>
    <td>Device Level Ring (A redundancy protocol)</td>
</tr>
<tr>
    <td>PTP</td>
    <td>Precision Time Protocol</td>
</tr>
<tr>
    <td>OC</td>
    <td>Ordinary Clock</td>
</tr>
<tr>
    <td>TC</td>
    <td>Transparent Clock</td>
</tr>
<tr>
    <td>E2E</td>
    <td>End to End</td>
</tr>
</table>

## API Documentation

Please see \ref INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_FWHAL_MODULE for API documentation. It is recommended to use these FWHAL APIs in the stack adaptation files.

## Procedure to kick-off the EtherNet/IP Adapter

- Initialize ICSS-EMAC to work as a switch
- Initialize the required tasks and interrupts
- Initialize the EtherNet/IP FWHAL, PTP, DLR and PRU-ICSS INTC
- Load EtherNet/IP firmware into PRUs of PRU-ICSS
- Initialize the EtherNet/IP adapter stack
- Start firmware
- Handle the events as needed. The Event/ISR definitions are available in developer guide(provide a link here), these have respective callbacks that can be registered to custom stack APIs.

## Integration with ICSS-EMAC

Ethernet/IP Adapter examples uses the ICSS-EMAC as its base switch layer. The PRU Firmware provides switch functionality and additional PTP and DLR functionalities. The NRT(non-real time) traffic is handled by ICSS-EMAC, wherein the packets are forwarded either to the TCP stack or a custom callback (configurable in ICSS-EMAC).

### Interface with ICSS-EMAC

Ethernet/IP packets are standard ethernet frames and all protocol specific data is embedded in TCP/IP payload. Hence the standard switch model applies. Packets are segregated based on VLAN PCP field. The 8 priorities which map to 4 queues, with highest and next highest priorities going to Queue 0 and Queue 1 and so on. If PCP field does not exist, then frames go to Queue 3. All queues except the highest priority queue forward the frames to TCP/IP stack. Further if packet type does not match PTP or DLR then that frame goes to TCP/IP stack. The highest priority queues are used for PTP and DLR. These packets are directly forwarded to the registered callback in ICSS-EMAC (using rxRTCallBack).

## Interrupts
EtherNet/IP firmware generates the following interrupts.

8 Host Interrupts (Host Interrupts 2 through 9) are exported from the PRU_ICSSG internal INTC for signaling the device level interrupt controllers. PRU_EVTOUT0 to PRU_EVTOUT7 correspond to these eight interrupts in the following table. Please check \ref PRUICSS_INTC section for more details.

<table>
<tr>
    <th>Name
    <th>Host Interrupt
    <th>Description
</tr>
<tr>
    <td> Frame Receive
    <td> PRU_EVTOUT0
    <td> Notifies host when firmware has stored a frame in host receive queue
</tr>
<tr>
    <td> DLR Port 0 Interrupt
    <td> PRU_EVTOUT1
    <td> Raised when there is a state change in DLR on Port 0
</tr>
<tr>
    <td> DLR Port 1 Interrupt
    <td> PRU_EVTOUT2
    <td> Raised when there is a state change in DLR on Port 1
</tr>
<tr>
    <td> Tx Callback Interrupt
    <td> PRU_EVTOUT3
    <td> Raised when a PTP/1588 frame which requires Tx Timestamping is sent out
</tr>
<tr>
    <td> DLR Beacon Timeout Interrupt for Port 0
    <td> PRU_EVTOUT4
    <td> Raised when the beacon timeout timer on Port 0 expires
</tr>
<tr>
    <td> DLR Beacon Timeout Interrupt for Port 1
    <td> PRU_EVTOUT7
    <td> Raised when the beacon timeout timer on Port 1 expires
</tr>
<tr>
    <td> Link Change
    <td> PRU_EVTOUT6
    <td> Interrupt is raised when the link on Ethernet PHY comes up or goes down
</tr>
</table>

## Device Level Ring

Device Level Ring on EtherNet/IP firmware provides redundancy to the switch implementation. It is a beacon based implementation with support for a minimum beacon of 200us and timeout of 400us. More information is available in developer guide(provide a link here).

## IEEE PTP-1588 Implementation
PTP/1588 on EtherNet/IP provides time synchronization support. The implementation is driven by CIP Sync requirements which require End to End clock support over UDP (Annex D). EtherNet/IP adapter application supports both OC and TC implementations along with syntonization.

## Additional References {#ETHERNETIP_ADAPTER_ADDITIONAL_REFERENCES}

Please refer to below documents to understand more about EtherNet/IP Adapter on TI platforms and EtherNet/IP Adapter protocol specifications.

<table>
<tr>
    <th>Document
    <th>Description
</tr>
<tr>
    <td>[EtherNet/IP on TI's Sitara processors](https://www.ti.com/lit/pdf/spry249)
    <td>Application note by TI on the EtherNet/IP Adapter implementation on TI's Sitara Processors.
</tr>
</table>

## See also

- [EtherNet/IP Adapter Demos](\ref EXAMPLES_INDUSTRIAL_COMMS_ETHERNETIP_ADAPTER_DEMOS)
