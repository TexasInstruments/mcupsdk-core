# QSPI {#DRIVERS_QSPI_PAGE}

[TOC]

The Quad Serial Peripheral Interface (QSPI) module is a kind of Serial Peripheral Interface (SPI) module
which allows single, dual or quad read access to external flash devices.
The QSPI module is used to transfer data, either in a memory mapped mode (for example a
processor wishing to execute code directly from external flash memory), or in an configuration mode where the
module is set-up to silently perform some requested operation, signaling its completion via interrupts or
status registers.

SDK has support for both High Level(HLD) and Low Level driver(LLD).
LLD is designed to be independent of other modules and the cross module
dependencies are to be taken care in application. HLD is a heaviy abstracted
driver which uses LLD driver underneath. It uses the DPL layer in SDK to
fill the LLD dependencies. It has the advantage of ease of use and less
Lines of code in application. Applications which find the SDK DPL layer
incompatible should use the LLD driver and customize based on requirement.
Others can go ahead with HLD.

- \subpage DRIVERS_QSPI_HLD_PAGE
- \subpage DRIVERS_QSPI_LLD_PAGE
