# PRUICSS {#DRIVERS_PRUICSS_PAGE}

[TOC]

Programmable Real-Time Unit and Industrial Communication Subsystem(PRU-ICSS) driver provides a well-defined API layer which allows applications to use the PRU-ICSS. PRU-ICSS is firmware programmable and can take on various personalities like Industrial Communication Protocol Switch (for protocols like EtherCAT, Profinet, EtherNet/IP), Ethernet Switch, Ethernet MAC, Industrial Drives, etc.

## Features Supported

- PRU control features like enabling/disabling/resetting a Programmable Real-Time Units (PRU0 and PRU1)
- Loading the firmware in PRU cores
- Read/Write/Reset different memories inside PRU-ICSS
- PRU and Host Event management. It does mapping of sys_evt/channel/hosts in the INTC module. APIs to register interrupt handler for events, generate an event, wait for occurrence of an event, and clear an event.
- Basic configurations in registers like GPCFG, MII_RT_G_CFG, ICSSG_SA_MX
- IEP clock selection, IEP counter enable/disable, IEP counter increment configuration
- PRU Constant Table Configuration

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Option to select the PRU-ICSS instance
- Option to select the Core Clock Frequency
- Based on above parameters, the SysConfig generated code does following:
    - Call \ref PRUICSS_init in System_init, and \ref PRUICSS_deinit in System_deinit
    - Enable the Core Clock and set the selected frequency
    - Create macros like `CONFIG_PRU_ICSS0` using the name passed in SysConfig. This is used as an input to \ref PRUICSS_open API.

## PRUICSS Interrupt Controller {#PRUICSS_INTC}

The interrupt controller (INTC) module maps interrupts coming from different parts of the device (mapped to PRU-ICSS instance) to a reduced set of interrupt channels.

The interrupt controller has the following features:
- Capturing up to 64 Events (inputs)
    <!-- TODO: Update this
    - Upper 96 are external events
    - Lower 64 are internal events -->
- Supports up to 10 output interrupt channels
- Generation of 10 Host Interrupts
    <!-- TODO: Update this
    - 2 Host Interrupts shared between the PRUs (PRU0 and PRU1) and TX_PRUs (TX_PRU0 and TX_PRU1).
    - 2 Host Interrupts for the RTU PRUs (RTU_PRU0 and RTU_PRU1).
    - 8 Host Interrupts exported from the PRU_ICSSG internal INTC for signaling the device level interrupt controllers (pulse and level provided).
    - 8 Host Interrupts (event 12 through 19) for the Task Managers. -->
- Each event can be enabled and disabled.
- Each host event can be enabled and disabled.
<!-- - Hardware prioritization of events. -->

<!-- TODO: Update this
Following are some important points related to INTC configuration:

- Any of the 160 internal interrupts can be mapped to any of the 20 channels.
- Multiple interrupts can be mapped to a single channel.
- An interrupt should not be mapped to more than one channel.
- Any of the 20 channels can be mapped to any of the 20 host interrupts. It is recommended to map channel "x" to host interrupt "x", where x is from 0 to 19.
- A channel should not be mapped to more than one host interrupt
- For channels mapping to the same host interrupt, lower number channels have higher priority.
- For interrupts on same channel, priority is determined by the hardware interrupt number. The lower the interrupt number, the higher the priority.
- Host Interrupt 0 is connected to bit 30 in register 31 (R31) of PRU0 and PRU1 in parallel.
- Host Interrupt 1 is connected to bit 31 in register 31 (R31) for PRU0 and PRU1 in parallel.
- Host Interrupts 2 through 9 exported from PRU-ICSS and mapped to device level interrupt controllers.
- Host Interrupt 10 is connected to bit 30 in register 31 (R31) to both RTU_PRU0 and RTU_PRU1 in parallel.
- Host Interrupt 11 is connected to bit 31 in register 31 (R31) to both RTU_PRU0 and RTU_PRU1 in parallel.
- Host Interrupts 12 through 19 are connected to each of the 4 Task Managers.

For industrial communication protocol examples running on R5F, Host Interrupts 2 through 9 exported from PRU-ICSS are used for signalling an interrupt to R5F. As this mapping is programmable and varies from example to example, we have a `*_pruss_intc_mapping.h` file for different protocol examples which is used for passing \ref PRUICSS_IntcInitData structure while calling \ref PRUICSS_intcInit API. -->

Following is an example of one mapping from `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/icss_fwhal/tiesc_pruss_intc_mapping.h` used for EtherCAT SubDevice.

The following line maps `PRU_ARM_EVENT2` to `CHANNEL6`.

```c
{PRU_ARM_EVENT2,CHANNEL6, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
```

`CHANNEL6` is mapped to the fourth host interrupt mapped to device level interrupt controller(Host Interrupt 6 out of 20) through this line.

```c
{CHANNEL6, PRU_EVTOUT4}
```

In AM64x, `PRU_ICSSG0_PR1_HOST_INTR_PEND_0`-`PRU_ICSSG0_PR1_HOST_INTR_PEND_7` (8 host interrupts) are mapped to `R5FSS0_CORE0_INTR_IN_120`-`R5FSS0_CORE0_INTR_IN_127`. This values are for ICSSG0 events mapped to R5FSS0 CORE0. For details regarding interrupt numbers on other cores, please refer to "9.4 Interrupt Sources" section in Technical Reference Manual(TRM) of AM64x, or corresponding section in TRM of other SoCs. These interrupt numbers can change from SoC to SoC, so please consult TRM before making any modifications to the interrupt map.

For the example mentioned above, interrupt number 124 (`R5FSS0_CORE0_INTR_IN_124`) should be used for `intrNum` parameter for \ref PRUICSS_registerIrqHandler. \ref PRUICSS_registerIrqHandler creates Hwi instance using \ref HwiP_construct API with the `intrNum` passed,

This mapping alone determines which interrupt number on R5F will be associated with a particular interrupt from PRUICSS. For example, in the code shown above, where ``PRU_ARM_EVENT2` maps to `CHANNEL6`, and  `CHANNEL6` maps to `PRU_EVTOUT4` can be modified to following, and the interrupt number on R5F would still remain the same.

```c
{PRU_ARM_EVENT2,CHANNEL7, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
{CHANNEL7, PRU_EVTOUT4}
```

But the usefulness of channels is that channels allow us to map multiple PRU events  to a single channel and in turn to a single host interrupt. For example, take a look at the following mapping used for link interrupt.

```c
{MII_LINK0_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
{MII_LINK1_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
{CHANNEL1, PRU1}
```

This configuration maps both Port 0 and Port 1 link interrupts to a single channel and in turn to a single host interrupt for PRU1 (Host Interrupt 1 out of 20).

\ref PRUICSS_intcInit API should be used to configure the INTC module.

<!-- \note Please refer to section "6.4.7 PRU_ICSSG Local INTC" in Technical Reference Manual(TRM) of AM64x for more details on INTC module -->

## Example Usage

Include the below file to access the APIs
\snippet Pruicss_sample_m_v0.c pruicss_include

Instance Open Example
\snippet Pruicss_sample_m_v0.c pruicss_open

Sequence for loading a firmware on PRU and running the PRU core
\snippet Pruicss_sample_m_v0.c pruicss_run_firmware

## API

\ref DRV_PRUICSS_MODULE
