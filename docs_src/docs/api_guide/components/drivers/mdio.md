# MDIO {#DRIVERS_MDIO_PAGE}

[TOC]

MDIO driver provides an interface to the MDIO management interface module which implements the 802.3 serial management interface to interrogate and control the Ethernet PHYs on the board.

## Features Supported

- Initialize MDIO clock
- Ethernet PHY register access (IEEE defined registers and extended registers)
- Read the link status of Ethernet PHY
- Enable link change Interrupt

## Features NOT Supported

NA

## Example Usage

Include the below file to access the APIs
\snippet Mdio_sample.c include

Initializing MDIO clock
\snippet Mdio_sample.c mdio_init_clock

Reading PHY link status for a PHY connected to MDIO
\snippet Mdio_sample.c mdio_link_status

Reading/writing a PHY register using MDIO
\snippet Mdio_sample.c mdio_phy_register_access

## API

\ref DRV_MDIO_MODULE
