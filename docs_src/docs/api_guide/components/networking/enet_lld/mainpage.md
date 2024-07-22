# Enet LLD Introduction {#enetlld_top}

[TOC]


- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Introduction {#enetlld_intro}

The Unified Ethernet Low-Level Driver (Enet LLD) is a driver that aims at
providing an unified interface for the different Ethernet peripherals found in
TI SoCs.

The Enet LLD is composed of the following layers: top-layer APIs, peripheral
layer, module layer and CSL.  The diagram below depicts the layers in the
Enet LLD.

![](EnetLLD_Diagram.png "Enet LLD Block Diagram")

[Back To Top](\ref index)



- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Application Programming Interface {#enetlld_api_overview}

The Enet LLD provides two sets of APIs: control and DMA.


## Control path API

The control API is an IOCTL-based interface which is used by applications to
control the Ethernet peripheral and its submodules.

For further details on the top-level control APIs, refer to the \ref ENET_MAIN_API.


## Data path (DMA) API

The DMA API is used by applications to perform data movement related operations,
such as opening and closing DMA channels, submitting and retrieving packets from
the underlying DMA controller.

For further details on the top-level control APIs, refer to the \ref ENET_DMA_API.


## Life cycle of an Enet LLD based application

The following diagram shows the usage of Enet LLD top-level APIs by local and
remote clients.

![](Enet_Lifecycle.png "Enet LLD life cycle")

[Back To Top](\ref index)


- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Enet Peripherals {#enetlld_enetpers}

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM261X

Enet LLD supports two families of Ethernet devices: CPSW and ICSSG.

\endcond

\cond SOC_AM273X || SOC_AWR294X

Enet LLD supports CPSW peripheral.

\endcond

## CPSW Peripheral {#enetper_cpsw}

Common Port SWitch (CPSW) subsystems is an IP providing networking functionality.
It provide Ethernet packet communication between the connected port(s) and
the System on Chip.  The total number of ports include host port which is an
internal port providing the packet streaming interface to the device internal
cores.  The external ports are MAC ports supporting Media Independent Interface
(MII) like MII, Gigabit Media Independent Interface (GMII), Reduced Media
Independent Interface (RMII), Reduced Gigabit Media Independent Interface
(RGMII), Serial Gigabit Media Independent Interface (SGMII) and Quad Serial
Gigabit Media Independent Interface (QSGMII).  The MII modes supported vary
based on device variant.

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM261X

**CPSW3G** is an integrated Ethernet switch IP with three-port: two MAC ports
and a CPPI DMA host port. The CPSW3G switch facilitates the
transfer of data between external ports along with internal traffic.

\endcond

\cond SOC_AM273X || SOC_AWR294X

**CPSW2G** is an integrated Ethernet switch IP with two-port: one MAC ports
and a CPPI DMA host port. The CPSW2G switch facilitates the
transfer of data between external ports along with internal traffic.

\endcond

\cond SOC_AM64X || SOC_AM243X

## ICSSG Peripheral {#enetper_icssg}

ICSSG supports two different modes: Dual-MAC and Switch.


### ICSSG Dual-MAC {#enetper_icssg_dualmac}

Each port in ICSSG Dual-MAC mode is abstracted in Enet LLD as different Enet
peripherals as each port is a different logical entity.

This is shown in the table below:

<table>
<tr>
  <th>ICSSG Instance
  <th>Slice
  <th>Enet Peripheral
  <th>Enet MAC Ports
<tr>
  <td rowspan="2">ICSSG0
  <td>Slice 0 (Port 1)
  <td>EnetType: \ref ENET_ICSSG_DUALMAC \n InstId: 0
  <td>\ref ENET_MAC_PORT_1
<tr>
  <td>Slice 1 (Port 2)
  <td>EnetType: \ref ENET_ICSSG_DUALMAC \n InstId: 1
  <td>\ref ENET_MAC_PORT_1
<tr>
  <td rowspan="2">ICSSG1
  <td>Slice 0 (Port 1)
  <td>EnetType: \ref ENET_ICSSG_DUALMAC \n InstId: 2
  <td>\ref ENET_MAC_PORT_1
<tr>
  <td>Slice 1 (Port 2)
  <td>EnetType: \ref ENET_ICSSG_DUALMAC \n InstId: 3
  <td>\ref ENET_MAC_PORT_1
</table>

It's worth clarifying that the first two columns ("ICSSG Instance" and "Slice")
are in ICSSG nomenclature, while the last two columns ("Enet Peripheral"
and "Enet MAC Ports") are in Enet LLD nomenclature.  The Enet MAC port numbers
in last column don't map one-to-one to the ICSSG port nomenclature, and
one should refer to this table to find out the peripheral type and instance
number to pass in \ref Enet_open().

In the Enet LLD hierarchy, each Enet peripheral can have one or more MAC
ports, starting at \ref ENET_MAC_PORT_1 id. Since Dual-MAC is implemented
as separate Enet peripherals, each peripheral will have only one MAC port
with id \ref ENET_MAC_PORT_1.

### ICSSG Switch Mode {#enetper_icssg_switch}

ICSSG Switch mode is abstracted in a different way, as the two ports are
logically tied to the same entity, the switch peripheral.

The peripheral and MAC port ids for Switch are shown below.

<table>
<tr>
  <th>ICSSG Instance
  <th>Slice
  <th>Enet Peripheral
  <th>Enet MAC Ports
<tr>
  <td>ICSSG0
  <td>Both slices
  <td>EnetType: \ref ENET_ICSSG_SWITCH \n InstId: 0
  <td>\ref ENET_MAC_PORT_1 \n \ref ENET_MAC_PORT_2
<tr>
  <td>ICSSG1
  <td>Both slices
  <td>EnetType: \ref ENET_ICSSG_SWITCH \n InstId: 1
  <td>\ref ENET_MAC_PORT_1 \n \ref ENET_MAC_PORT_2
</table>

\endcond

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Document Revision History {#enetlld_hist}

Revision | Date          | Author         | Description              | Status
---------|---------------|----------------|--------------------------|-------
0.1      | 17 Aug 2020   | Misael Lopez   | First version            | Draft
0.1      | 1 Sept 2021   | Prasad Jondhale| Updates for MCU+ SDK     | 0.1
