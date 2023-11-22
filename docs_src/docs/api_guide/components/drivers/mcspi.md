# MCSPI {#DRIVERS_MCSPI_PAGE}

[TOC]

The Multi Channel Serial Peripheral Interface (MCSPI) driver is a generic,
full-duplex driver that transmits and receives data on the SPI bus.
The SPI protocol defines the format of a data transfer over the SPI bus,
but it leaves flow control, data formatting, and handshaking mechanisms
to higher-level software layers.

SDK has support for both High Level(HLD) and Low Level driver(LLD).
LLD is designed to be independent of other modules and the cross module
dependencies are to be taken care in application. HLD is a heaviy abstracted
driver which uses LLD driver underneath. It uses the DPL layer in SDK to
fill the LLD dependencies. It has the advantage of ease of use and less
Lines of code in application. Applications which find the SDK DPL layer
incompatible should use the LLD driver and customize based on requirement.
Others can go ahead with HLD.

- \subpage DRIVERS_MCSPI_HLD_PAGE
- \subpage DRIVERS_MCSPI_LLD_PAGE

