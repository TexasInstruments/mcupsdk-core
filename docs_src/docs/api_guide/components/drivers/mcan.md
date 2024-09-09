# MCAN {#DRIVERS_MCAN_PAGE}

[TOC]

The Controller Area Network (CAN) is a serial communication protocol which efficiently supports distributed real-time control.

This module supports MCAN and CANFD.

MCAN is a base address type and is similar to LLD (Low Level Driver). LLD is designed to be independent
of other modules and the cross module dependencies are to be taken care in application.

CANFD wich is a handle based and is similar to HLD (High Level Driver). HLD is a heaviy abstracted 
driver which uses LLD driver underneath. It uses the DPL layer in SDK to fill the LLD dependencies. 
It has the advantage of ease of use and less Lines of code in application. Applications which find
the SDK DPL layer incompatible should use the LLD driver and customize based on requirement.
Others can go ahead with HLD.

- \subpage DRIVERS_MCAN_LLD_PAGE
- \subpage DRIVERS_CANFD_HLD_PAGE
