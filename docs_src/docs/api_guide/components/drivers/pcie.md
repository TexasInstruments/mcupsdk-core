# PCIE {#DRIVERS_PCIE_PAGE}

[TOC]
PCIE is a peripheral used for high speed data transfer between devices.
The PCIe driver provides API to perform initialization, configuration of End point (EP)and Root complex (RC) mode of operation, configuring and sending interrupts.

## Features Supported

\note PCIe makes use of 2 Ring Accelerators (Rings for IPC), one each for MSI and MSIx to route interrupts to the core.
\note The actual index for the Ring Accelerator, Global Event and Virtual Interrupt is depended on the Board Config for the soc and core.

- EP and RC operation
- Gen 2 operation speed
- x1 lane support
- Legacy interrupts
- MSI (Message Signalled Interrupt)
- MSIx (Message Signalled Interrupt Extended)

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock,MPU,RAT and others.
- PCIe operation mode - Root complex (RC) or End point (EP)
- PCIe instances
- Operation speed (Gen1, Gen2 or Gen3)
- Number of lanes
- Inbound Address Translation Unit (ATU) configuration
- Outbound Address Translation Unit (ATU) configuration

## Features NOT supported

- Bridge mode of operation
- Bus enumeration
- x2 and x4 lanes

## Usage Overview

### Initializing PCIe driver

#Pcie_init() must be called before any other PCIe APIs. This function iterates through the elements of the gPcieConfig[] array, initializing the driver. Please note that initializing of PCIe driver is taken care by the SysConfig generated code.

### Opening PCIe driver

After initializing the driver by calling #Pcie_init(), the application can open a PCIe instance by calling #Pcie_open().
Please note that the opening of PCIe driver is taken care by the SysConfig generated code.
This function takes and index into the gPcieConfig[] array, and the PCIe parameters data structure. The PCIe instance is specified by the index of the PCIe in the gPcieConfig[].
Calling #Pcie_open() second time with same index, previously passed to #Pcie_open() will result in an error.

## Important Usage Guidelines

- Care must be taken so that the ATU address regions does not overlap. The ATU configuration can be done using SysConfig.
- The zeroth Outbound ATU region for RC device is used for configuration space access of the remote EP.
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