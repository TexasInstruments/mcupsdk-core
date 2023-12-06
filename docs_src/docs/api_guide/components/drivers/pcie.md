# PCIE {#DRIVERS_PCIE_PAGE}

[TOC]
PCIE is a peripheral used for high speed data transfer between devices.
The PCIe driver provides API to perform initialization, configuration of End point (EP)and Root complex (RC) mode of operation, configuring and sending interrupts.

## Features Supported

\note PCIe makes use of 2 Ring Accelerators (Rings for IPC), one each for MSI and MSIx to route interrupts to the core.
\note The actual index for the Ring Accelerator, Global Event and Virtual Interrupt is depended on the Board Config for the soc and core.

- EP and RC operation
- Gen 1 and 2 operation speed
- x1 lane support
\if SOC_AM65X
- x2 lane support
\else
- Legacy interrupts
- MSI (Message Signalled Interrupt)
- MSIx (Message Signalled Interrupt Extended)
\endif

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock,MPU,RAT and others.
- PCIe operation mode - Root complex (RC) or End point (EP)
- PCIe instances
- Vendor ID, Device ID, Subsystem Vendor ID, Subsystem ID
- Class Code, Sub-Class Code, Programming Interface, Revision ID
- Operation speed (Gen1 or Gen2)
- Number of lanes (only x1 supported)
- Reference clock mode
  - Internal or external reference clock
  - Reference clock output enabled or disabled (for internal reference clock)
  - Optional spread spectrum clocking
- SRIS (Separate reference, independent spread) reference clock configuration
- Legacy interrupt pin to use
  - INTA-INTD or disabled
- Number of MSI vectors requested by the device (multiple message capable)
- Inbound Address Translation Unit (ATU) configuration
- Outbound Address Translation Unit (ATU) configuration

## Features NOT supported

- Bridge mode of operation
- Bus enumeration
\if SOC_AM65X
- x4 lane support
\else
- x2 and x4 lanes
\endif

## Usage Overview

### Initializing PCIe driver

#Pcie_init() must be called before any other PCIe APIs. This function iterates through the elements of the gPcieConfig[] array, initializing the driver. Please note that initializing of PCIe driver is taken care by the SysConfig generated code.

### Opening PCIe driver

After initializing the driver by calling #Pcie_init(), the application can open a PCIe instance by calling #Pcie_open().
Please note that the opening of PCIe driver is taken care by the SysConfig generated code.
This function takes and index into the gPcieConfig[] array, and the PCIe parameters data structure. The PCIe instance is specified by the index of the PCIe in the gPcieConfig[].
Calling #Pcie_open() second time with same index, previously passed to #Pcie_open() will result in an error.

### Configuration of the PCIe driver

In case of a root complex, configuration of the RC is complete after the call to #Pcie_open().

In case of an endpoint, only a minimum amount of configuration is performed by the PCIe driver as part of #Pcie_open(), and configuration needs to be finalized with a call to #Pcie_cfgEP().

The endpoint intially blocks configuration space accesses from the RC by responding with a Configuration Request Retry Status (CRS). This allows the EP to postpone enumeration by the RC until the EP had a chance to setup its configuration space. The EP can modify its own configuration space only when a PCIe reference clock is available.

For setups where the EP always has a reference clock available, an endpoint application can immediately follow-up with a call to #Pcie_cfgEP().
For setups where the EP can't rely on the reference clock being continously available, the endpoint must wait for the reference clock before calling #Pcie_cfgEP(). The endpoint can detect the presence of a Link via #Pcie_isLinkUp() as an indication of whether the reference clock is available.

After a call to #Pcie_cfgEP(), the EP responds to configuration space accesses.

## Important Usage Guidelines

- Care must be taken so that the ATU address regions does not overlap. The ATU configuration can be done using SysConfig.
- The zeroth Outbound ATU region for RC device is used for configuration space access of the remote EP. This configuration is performed implicitly as part of #Pcie_open().
- The usage of Inbound ATU regions and Outbound ATU regions differs between RC and EP:
  - A RC may have at most two inbound ATU regions that can optionally be used to remap addresses from PCIe space to local SoC memory space, but these are optional.
  - An EP may have up six inbound ATU regions (actually BARs) that can each map a single contiguous memory region from SoC memory space to PCIe space. The number of BARs depends on their properties, as a 64-bit BAR (that can be located anywhere in the 64-bit PCIe memory space) takes up to consecutive BARs. Only BARs 0, 2 and 4 may be configured as 64-bit BARs.
  - An RC may statically configure its outbound ATU regions, because it might statically partition its view of PCIe space (e.g. 64 KB configuration space, 64 KB I/O space, 64 MB memory space), and the RC assigns addresses within the PCIe space to EPs at its own discretion.
  - An EP typically needs to configure its outbound ATU regions dynamically, because the PCIe addresses it needs to access are determined by the RC at runtime.
- Address translation on these devices works by replacing a number of MSB bits when passing addresses from PCIe to the SoC and vice versa, while leaving a number of LSB bits unmodified. The number of MSB vs. LSB bits depends on the size of the memory region, e.g. a 64 KB region passes 16 LSB bits unmodified and replaces the 16 MSB bits.
  Based on this, memory region sizes need to be a power of two, and the addresses, both local and on the PCIe side, need to be aligned to the region's size.
  If you need to access some memory that isn't itself a power of two, or if the base address isn't aligned to the region's size, you need to
    - map a larger area that covers all of the memory of intereset, at the risk of mapping additional, unwanted regions as well
    - align the base address and size of your data structures to a power of two, e.g. by manually placing them in a linker script

## Example Usage

Include the below file to access the APIs
\snippet Pcie_sample.c include

Instance open Example
\snippet Pcie_sample.c open

Instance close Example
\snippet Pcie_sample.c close

Outbound ATU config Example
\snippet Pcie_sample.c obatu

Inbound ATU config Example
\snippet Pcie_sample.c ibatu

BAR Configuration for EP Example
\snippet Pcie_sample.c barCfgEP

## API

\ref DRV_PCIE_MODULE