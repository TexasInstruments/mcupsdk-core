# Ethernet Features(CPSW) {#NETWORKING_FEATURES}

[TOC]

## Overview
This page lists the software features and protocols supported by the MCU Plus SDK Ethernet (CPSW) drivers

## Feature Summary

| Feature | Description |
|---------|-------------|
|Basic L2 Switching|Ethernet packet switching with 10M/100M/1G Bandwidth with Multicast and VLAN Capability|
|L3 switching with LwIP Stack|TCP/IP and UDP protocol integration with LwIP|
|TSN support|gPTP protocol stack support for Time Synchronization|
|EST/TAS support|IEEE 802.1Qbv compliant traffic scheduling capabilities|
|CBS|IEEE 802.1Qav compliant traffic shaping feature supported in Enet-lld driver|
|IET|IEEE 802.3br compliant frame preemption feature supported in Enet-lld driver|
|Cut Through|IEEE 802.1DU compliant Packet forwarding without store and delay supported|

## Basic Layer 2 Switching

Common Platform Switch or CPSW supports Layer 2 Ethernet packet switching on 10M/100M/1G bandwidth in 
Half or Full Duplex mode (Half Duplex with 1G is not supported) on both of its external MAC ports. 
CPSW operates in two modes, namely: 
 - Switch Mode - packet forwarding is enabled and packets are forwarded to Host + Other MAC ports. This is the default configuration for CPSW.
 - MAC mode - packets are only given to the Host port and are not forwarded to other MAC Ports.

The switch dynamically builds and maintains a forwarding table to optimize 
traffic flow within the network using its Address Learning Engine submodule, which also gives the flexibility of auto-ageing the learnt entries.

- It is capable of handling IEEE 802.1Q compliant **Virtual LAN** tagged packets. 
- Multicast traffic handling is supported.
- This SDK includes out-of-the-box examples of MAC and PHY Loopback capability. For further details, refer to \ref EXAMPLES_ENET_CPSW_LOOPBACK
- SDK also includes a 'vanilla' L2 CPSW switch example. For further details, refer to \ref EXAMPLES_ENET_LAYER2_CPSW_SWITCH

For further driver details and IOCTLs, refer to \ref ENET_LLD

## Layer 3 switching with LwIP Stack

TCP/IP and UDP protocol stacks are provided by the open-source Lightweight IP (LwIP) stack. The SDK integrates all major L3 and above protocols such as TCP, UDP, 
ICMP, ARP, DHCP, and HTTP with IPv4. 
- CPSW is capable of packet routing based on IP addresses, both with and without VLAN. 
- Hardware is also capable of **TCP/UDP checksum offload**. UDP-Lite checksum offload is performed in driver software.

The SDK provides a set of default parameters (such as LwIP pbuf memory pool size) for LwIP integration for every SoC that can be configured per requirement. The SDK also provides numerous examples with the LwIP stack; the following table lists a few of them:

<table>
<tr><th>Example</th><th>Reference</th></tr>
<tr><td>TCP server</td><td>\ref EXAMPLES_ENET_LWIP_CPSW_TCPSERVER</td></tr>
<tr><td>TCP Client</td><td>\ref EXAMPLES_ENET_LWIP_CPSW_TCPCLIENT</td></tr>
<tr><td>UDP Client</td><td>\ref EXAMPLES_ENET_LWIP_CPSW_UDPCLIENT</td></tr>
<tr><td>UDP IGMP</td><td>\ref EXAMPLES_ENET_LWIP_CPSW_UDP_IGMP</td></tr>
<tr><td>Lwip Sockets</td><td>\ref EXAMPLES_ENET_LWIP_CPSW_SOCKET</td></tr>
<tr><td>Iperf testing</td><td>\ref EXAMPLES_ENET_LWIP_CPSW</td></tr>
<tr><td>HTTPS Server</td><td>\ref EXAMPLES_CPSW_LWIP_HTTPS</td></tr>
</table>

TLS support is enabled for examples such as HTTPS using the open-source **MbedTLS** library. This feature provides encryption, authentication, and data integrity protection for network traffic.
For further reading about MbedTLS, see https://mbed-tls.readthedocs.io/en/latest/ 

Note: Currently **IPv6 is not supported** by the driver.

For further details please check \ref NETWORKING_LWIP


## TSN support

SDK provides **general Precision Time Protocol(gPTP)** or **IEEE 802.1AS** stack enabled for time synchronization across networked devices. It establishes a common time reference with sub-microsecond accuracy, allowing coordinated operation of distributed systems. Accurate synchronization is critical for scheduled traffic, industrial automation, and time-sensitive networking applications. The protocol continuously compensates for clock drift and network delays to maintain timing precision.

- The **CPTS** submodule of CPSW is used for timestamping packets, and [Time Sync IOCTLs](\ref EnetTimeSync_Ioctl) provide a way to configure it.

The SDK gives full fledged gPTP example in

<table>
<tr><th>gPTP variant</th><th>Reference</th></tr>
<tr><td>Master mode</td><td>\ref EXAMPLES_ENET_CPSW_TSN_GPTP_TT</td></tr>
<tr><td>Slave mode</td><td>\ref EXAMPLES_ENET_CPSW_TSN_GPTP_TR</td></tr>
<tr><td>Bridge mode</td><td>\ref EXAMPLES_ENET_CPSW_TSN_GPTP_BRIDGE</td></tr>
<tr><td>Parallel to LwIP stack</td><td>\ref EXAMPLES_ENET_CPSW_TSN_LWIP_GPTP</td></tr>
</table>


For further details, read \ref ENET_CPSW_TSN_GPTP

## EST/TAS support

**Enhanced Scheduled Traffic (EST)**, also known as Time-Aware Shaper (TAS), provides IEEE 802.1Qbv compliant traffic scheduling capabilities. 
CPSW supports express traffic of 8 priority classes and generates a repeating open-close sequence for the gates of each priority class via the CPTS EST function generator. It can be configured to work with or without preemption.
This enables deterministic transmission windows for time-critical traffic. Gate control schedules ensure that high-priority frames are transmitted at precisely defined times. This feature is essential for real-time industrial and automotive networking applications.

- The SDK provides a menu-driven example with positive and negative test cases for configuring various gate sequences to demonstrate this feature. The example is described in detail here: \ref EXAMPLES_ENET_CPSW_EST

- EST related IOCTLS are described here \ref ENET_MOD_TAS API


## CBS 

**Credit-Based Shaper** is defined by IEEE 802.1Qav. Its primary purpose is to smooth out bursty traffic by distributing data transmissions evenly over time, thereby providing a deterministic upper bound on network latency and preventing high-priority streams from completely blocking lower-priority traffic. The CBS algorithm works by assigning a credit counter to each high-priority traffic queue. Packets can only be sent if the queue's credit balance is greater than or equal to zero.
When a high-priority queue has data waiting to be sent, but is blocked because a lower-priority packet is currently occupying the physical wire, it accumulates credit at a defined rate called the idleSlope. While the high-priority queue is actively transmitting its data, its credit balance decreases at a rapid rate called the sendSlope.

**This feature is implemented in the driver, but no example currently demonstrates this feature.** The list of IOCTLs that implement this is provided below:
- \ref ENET_HOSTPORT_IOCTL_SET_CREDIT_BASED_SHAPING
- \ref ENET_HOSTPORT_IOCTL_GET_CREDIT_BASED_SHAPING
- \ref ENET_MACPORT_IOCTL_SET_CREDIT_BASED_SHAPING
- \ref ENET_MACPORT_IOCTL_GET_CREDIT_BASED_SHAPING

## IET

**Interspersing Express Traffic (IET)** is specified in IEEE 802.3br. It reduces latency for critical time-sensitive data by allowing high-priority frames to temporarily interrupt (preempt) an ongoing, lower-priority frame mid-transmission. Once the urgent packet is sent, the interrupted frame automatically resumes without having to be entirely retransmitted, preventing large best-effort data packets from blocking critical control streams. This mechanism is particularly beneficial for deterministic and real-time communication systems. 

**This feature is implemented in the driver, but no example currently demonstrates this feature.** The list of IOCTLs that implement this is provided below:

- \ref ENET_MACPORT_IOCTL_IET_RELEASE_PREEMPT_TRAFFIC
- \ref ENET_MACPORT_IOCTL_IET_HOLD_PREEMPT_TRAFFIC
- \ref ENET_MACPORT_IOCTL_GET_QUEUE_PREEMPT_STATUS
- \ref ENET_MACPORT_IOCTL_SET_PREEMPT_QUEUE
- \ref ENET_MACPORT_IOCTL_GET_PREEMPT_MIN_FRAG_SIZE
- \ref ENET_MACPORT_IOCTL_SET_PREEMPT_MIN_FRAG_SIZE
- \ref ENET_MACPORT_IOCTL_GET_PREEMPT_VERIFY_STATUS
- \ref ENET_MACPORT_IOCTL_DISABLE_PREEMPT_VERIFICATION
- \ref ENET_MACPORT_IOCTL_ENABLE_PREEMPT_VERIFICATION
- \ref ENET_MACPORT_IOCTL_GET_PREEMPTION_ACTIVE_STATUS
- \ref ENET_MACPORT_IOCTL_GET_PREEMPTION_ENABLE_STATUS
- \ref ENET_MACPORT_IOCTL_DISABLE_PREEMPTION
- \ref ENET_MACPORT_IOCTL_ENABLE_PREEMPTION

\cond SOC_AM243X || SOC_AM64X
## Cut Through

Cut-Through Forwarding is defined by IEEE 802.1DU. Cut-through switching is a network packet forwarding method where a switch minimizes latency by transmitting a frame before it is completely received. Unlike traditional store-and-forward switching, which waits for the entire packet to arrive, a cut-through switch reads only the destination address and immediately starts routing the data. Any received packets with errors that are sent through cut-through from an Ethernet receive port to any Ethernet transmit port(s) will egress with at least one byte of the generated outgoing packet CRC inverted to indicate the error. This occurs because cut-through operations begin before the end of the packet when the receive port detects that the packet had an error.

In this SDK \ref EXAMPLES_ENET_LAYER2_CPSW_SWITCH uses the Cut-Through feature.

The list of IOCTLs that implement this is provided below:

- \ref ENET_MACPORT_IOCTL_SET_CUT_THRU_PARAMS
- \ref ENET_MACPORT_IOCTL_GET_CUT_THRU_PARAMS

\endcond
## See Also
\ref NETWORKING