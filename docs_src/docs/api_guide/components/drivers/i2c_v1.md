# I2C {#DRIVERS_I2C_PAGE}

[TOC]

I2C module provides an interface to any I2C bus compatible device
accessible via I2C serial bus. External components attached to the I2C bus
can serially transmit/receive data to/from the CPU through two wire interface.
I2C driver provides APIs to perform transmit/receive to any of the I2C peripheral

SDK has support for both High Level(HLD) and Low Level driver(LLD).
LLD is designed to be independent of other modules and the cross module
dependencies are to be taken care of in application. HLD is a heavily abstracted
driver which uses the LLD driver underneath. It uses the DPL layer in SDK to
fill the LLD dependencies. It has the advantage of ease of use and less
lines of code in application. Applications which find the SDK DPL layer
incompatible should use the LLD driver and customize based on requirement.
Others can go ahead with HLD.

- \subpage DRIVERS_I2C_HLD_PAGE
- \subpage DRIVERS_I2C_LLD_PAGE
