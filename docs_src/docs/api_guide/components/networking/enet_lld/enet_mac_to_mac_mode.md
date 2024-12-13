MAC2MAC support {#enet_mac2mac_top}
=====================

[TOC]

## Introduction {#mac2mac_intro}

A MAC is connected to the network using a PHY, but when the objective is to connect to another MAC in a controlled environment such as a PCB, the PHY is often redundant. 
In such a situation, a PHY-less approach can be used where the RGMII Rx, Tx, and Clock pins are usually connected directly using buffers. This is called a MAC2MAC connection.

  \imageStyle{Mac2MacMode.png,width:25%}
  \image html Mac2MacMode.png

## Enable MAC2MAC through Sysconfig {#mac2mac_syscfg_support}

\note MAC2MAC is not a comprehensively tested feature.

The following steps list the changes needed in Sysconfig to enable MAC2MAC mode.

- Configure the following option under Enet (CPSW) -> MAC Port Config -> MAC Port 1/2 Config
  + Set the "Link Speed Capability" to \ref ENET_SPEED_1GBIT.
  + Set the "Link Duplexity Capability" to \ref ENET_DUPLEX_FULL.
  + From Port PHY Configuration, set the "ETHPHY Device" to **NO-PHY**, this internally sets the PHY address to #ENETPHY_INVALID_PHYADDR indicating that it is a PHY-less connection and Sysconfig will not generate any code for ETHPHY. 

  \imageStyle{Mac2MacSyscfg.png,width:50%}
  \image html Mac2MacSyscfg.png

