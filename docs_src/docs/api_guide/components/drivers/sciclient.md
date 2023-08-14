# SCICLIENT {#DRIVERS_SCICLIENT_PAGE}

[TOC]
\if SOC_AM65X
Device AM65x fall under the K3 SoC family and has a concept of centralized Power, Resource and Security management to allow mitigating the challenges of the traditional approach to system control. System Firmware (hereafter referred to as SYSFW) is the collective name for the security and device management firmware which offers these centralized services. In this concept a processing unit (for example an R5F) can request the SYSFW to control power, grant resources or provide secure services. This is done via a special messaging channel called a **secure proxy**. The messages are sent obeying a proprietary protocol called TISCI (TI System Controller Interface) protocol. For more information on TISCI protocol you can refer to the \htmllink{https://software-dl.ti.com/tisci/esd/latest/index.html, **TISCI Public Documentation**}.
\else
Devices AM64x and AM243x fall under the K3 SoC family and has a concept of centralized Power, Resource and Security management to allow mitigating the challenges of the traditional approach to system control. System Firmware (hereafter referred to as SYSFW) is the collective name for the security and device management firmware which offers these centralized services. In this concept a processing unit (for example an R5F) can request the SYSFW to control power, grant resources or provide secure services. This is done via a special messaging channel called a **secure proxy**. The messages are sent obeying a proprietary protocol called TISCI (TI System Controller Interface) protocol. For more information on TISCI protocol you can refer to the \htmllink{https://software-dl.ti.com/tisci/esd/latest/index.html, **TISCI Public Documentation**}.
\endif
Sciclient as a software block has multiple functional sub-blocks inside it, as shown in the below image:

\imageStyle{sciclient_block.png,width:60%}
\image html sciclient_block.png "Sciclient Sub-blocks"

More details on the APIs provided on these layers can be found in the API section, linked towards the end of this page.

Generally speaking, the Sciclient driver provides API to communicate with the SYSFW using the TISCI protocol. As mentioned above, this would be for system level tasks like resource allocation, peripheral power on/off, peripheral clock setting, secure services and so on. The sciclient will be part of the application code running on each core.

\imageStyle{sciclient_sysfw.png,width:60%}
\image html sciclient_sysfw.png "Typical Sciclient Operation"

The above image shows the operation for only one core, but the same thing happens for all the cores. SYSFW deals with all the requests coming from each of the cores.

Sciclient is mostly used by other drivers, like DMA, GPIO etc. Sciclient acts as an interface to the SYSFW for these drivers when they need say a resource like DMA channel, or configure an interrupt route. Below are the high level features supported by the driver:

## Features Supported

- APIs to load the SYSFW onto the DMSC core
- APIs to pass a specific board configuration to the SYSFW
- Abstracted APIs for Power and Resource Management
- APIs for Processor Boot including secure boot
- APIs for configuring firewalls

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- There are no user programmable features for Sciclient SysConfig. However, adding any module makes the PowerClock_init() initialize the module power and clock. This is indirectly done using Sciclient APIs.

## Features NOT Supported

NA

## Important Usage Guidelines

- Sciclient is mostly used by other peripheral drivers, and mostly not directly by an application. From an application point of view, major usage of Sciclient APIs would be to power on/off a module, set/get the clock of a module, etc.
\cond SOC_AM64X || SOC_AM243X
This is abstracted in the SOC specific layer.
\endcond

## Example Usage

Include the below file to access the APIs
\snippet Sciclient_sample.c include

**Module Power ON Example**
\snippet Sciclient_sample.c sciclient_module_power_on

\cond SOC_AM64X || SOC_AM243X
**Interrupt configuration Example**

Since interrupt router outputs are shared resources, we need Sciclient to configure interrupt routers for certain peripherals like GPIO.
Here is a snippet explaining this with an example of configuring a GPIO interrupt. For more details on the ideas used below, refer \htmllink{https://www.ti.com/lit/ug/spruim2c/spruim2c.pdf, AM64x/AM243x TRM} Chapter 9 on interrupts. The `INTRTR Integration` subsection explains the routing with a diagram.
\snippet Sciclient_sample.c sciclient_rm_irq_gpio
\endcond

## API

\ref DRV_SCICLIENT_MODULE
