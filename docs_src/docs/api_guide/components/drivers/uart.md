# UART {#DRIVERS_UART_PAGE}

[TOC]
UART is used to translate the data between the chip and a serial port.
The UART driver provides API to perform read and write to any of the UART peripherals on the board, with the multiple modes of operation.

SDK has support for both High Level(HLD) and Low Level driver(LLD).
LLD is designed to be independent of other modules and the cross module
dependencies are to be taken care in application. HLD is a heaviy abstracted
driver which uses LLD driver underneath. It uses the DPL layer in SDK to
fill the LLD dependencies. It has the advantage of ease of use and less
Lines of code in application. Applications which find the SDK DPL layer
incompatible should use the LLD driver and customize based on requirement.
Others can go ahead with HLD.

- \subpage DRIVERS_UART_HLD_PAGE
- \subpage DRIVERS_UART_LLD_PAGE
