# OSPI {#DRIVERS_OSPI_PAGE}

[TOC]

The Octal Serial Peripheral Interface (OSPI) module is a kind of Serial Peripheral Interface (SPI) module
which allows single, dual, quad or octal read and write access to external flash devices.
The OSPI module is used to transfer data, either in a memory mapped direct mode (for example a
processor wishing to execute code directly from external flash memory), or in an indirect mode where the
module is set-up to silently perform some requested operation, signaling its completion via interrupts or
status registers.

SDK has support for both High Level driver(HLD) and Low Level driver(LLD).
LLD is designed to be independent of other modules and the cross module
dependencies are to be taken care in application. HLD is a heaviy abstracted
driver which uses LLD underneath. It uses the DPL layer in SDK to
fill the LLD dependencies. It has the advantage of ease of use and less
Lines of code in application. Applications which find the SDK DPL layer
incompatible should use the LLD and customize based on requirement.
Others can go ahead with HLD.

- \subpage DRIVERS_OSPI_HLD_PAGE
- \subpage DRIVERS_OSPI_LLD_PAGE