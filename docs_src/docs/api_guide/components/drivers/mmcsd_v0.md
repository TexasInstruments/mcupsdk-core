# MMCSD {#DRIVERS_MMCSD_PAGE}

[TOC]

MMCSD module provides an interface to any MMC and SD compatible memory device.
MMCSD driver provides APIs to perform Write and Read operations on the MMC and
SD devices connected to the module.

SDK has support for both High Level(HLD) and Low Level driver(LLD).
LLD is designed to be independent of other modules and the cross module
dependencies are to be taken care of in application. HLD is a heavily abstracted
driver which uses the LLD driver underneath. It uses the DPL layer in SDK to
fill the LLD dependencies. It has the advantage of ease of use and less
lines of code in application. Applications which find the SDK DPL layer
incompatible should use the LLD driver and customize based on requirement.
Others can go ahead with HLD.

- \subpage DRIVERS_MMCSD_V0_HLD_PAGE
- \subpage DRIVERS_MMCSD_V0_LLD_PAGE