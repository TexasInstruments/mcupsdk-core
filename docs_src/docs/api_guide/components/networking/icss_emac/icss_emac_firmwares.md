# Dual EMAC and Switch {#DUAL_EMAC_AND_SWITCH}

[TOC]

## Introduction

The ICSS FIRMWARES serves as example to implement various network functionalities. Package includes source release for DUAL EMAC and basic Switch firmwares.

## DUAL_EMAC

ICSS DUAL EMAC FIRMWARE is a single port Ethernet MAC (Media Access Control) i.e. Layer 2 of OSI Model. It implements a 2 port ethernet mac supporting 10/100 Mbps. DUAL EMAC FIRMWARE is standardized to IEEE 802.1 Ethernet Standards. Primary use case of the protocol is to demonstrate basic ethernet functionality via both PRU cores on 10/100 Mbit Ethernet cable. ICSS DUAL EMAC FIRMWARE can be used independently on two PRU’s to implement two independent MAC’s with two different MAC addresses and two different IP addresses. To provide an analogy, this is somewhat similar to a two port Ethernet PCIe NIC card on a PC.Ethernet interface in this case is available along with Host processor on a single SoC. Following are high level features:

<table>
<tr>
    <th>Requirements
    <th>Remarks
</tr>
<tr>
    <td>1 ms buffering per port
    <td>Supported
</tr>
<tr>
    <td>Host IRQ
    <td>Supported
</tr>
<tr>
    <td>Ethernet QoS
    <td>With 2 queues instead of 8. So, it is not a standard Ethernet QoS implementation.
</tr>
<tr>
    <td>Statistics
    <td>Supported
</tr>
<tr>
    <td>Storm Prevention
    <td>Supported
</tr>
<tr>
    <td>Promiscuous Mode
    <td>Experimental Feature which is to be tested extensively
</tr>
<tr>
    <td>TTS (Time Triggered Send)
    <td>Not Supported
</tr>
<tr>
    <td>Error Handling
    <td>Supported
</tr>
<tr>
    <td>Multicast filtering
    <td>Supported
</tr>
<tr>
    <td>VLAN filtering
    <td>Experimental Feature which is to be tested extensively
</tr>
</table>

## SWITCH

ICSS SWITCH FIRMWARE is a three port learning Ethernet switch i.e. Layer 2 of OSI Model. It implements a 2 port cut through ethernet switch supporting at 100 Mbps. SWITCH FIRMWARE is standardized to IEEE 802.1 Ethernet Standards. The primary use case of the protocol is to use Ethernet to automate applications which require short cut-through latency and low hardware costs. ICSS SWITCH FIRMWARE uses two PRU to implement three port Ethernet switch with one single MAC and IP address. To provide an analogy, this is somewhat standard network switch only here the network functionality is available to the host core within the single SOC. The following are the high level features it supports:

<table>
<tr>
    <th>Requirements
    <th>Remarks
</tr>
<tr>
    <td>Cut-Through
    <td>Supported
</tr>
<tr>
    <td>Store and Forward
    <td>Supported
</tr>
<tr>
    <td>1 ms buffering per port
    <td>Supported
</tr>
<tr>
    <td>Host IRQ
    <td>Supported
</tr>
<tr>
    <td>Ethernet QoS
    <td>With 4 queues instead of 8. So, it is not a standard Ethernet QoS implementation.
</tr>
<tr>
    <td>802.1 learning switch
    <td>Supported
</tr>
<tr>
    <td>Statistics
    <td>Supported
</tr>
<tr>
    <td>Queue-Contention Handling on each port
    <td>Supported
</tr>
<tr>
    <td>Three-Port Switch
    <td>Supported
</tr>
<tr>
    <td>Storm Prevention
    <td>Not Supported
</tr>
<tr>
    <td>Error Handling
    <td>Supported
</tr>
</table>
