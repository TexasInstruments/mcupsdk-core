# PRUICSS {#DRIVERS_PRUICSS_PAGE}

[TOC]

Programmable Real-Time Unit and Industrial Communication Subsystem(PRU-ICSS) driver provides a well-defined API layer which allows applications to use the PRU-ICSS. PRU-ICSS is firmware programmable and can take on various personalities like Industrial Communication Protocol Switch (for protocols like EtherCAT, Profinet, EtherNet/IP), Ethernet Switch, Ethernet MAC, Industrial Drives, etc.

## Features Supported

- PRU control features like enabling/disabling/resetting a Programmable Real-Time Units (PRU0 and PRU1), Auxiliary Programmable Real-Time Units (RTU_PRU0 and RTU_PRU1) or Transmit Programmable Real-Time Units (TX_PRU0 and TX_PRU1) core
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
- Option to enable the IEP Sync Mode
- Option to select the IEP Clock Frequency if IEP Sync Mode is disabled
- Optional sub-module to select pin configurations
- Optional sub-module to select ICSS interrupt controller signals mapping
- Based on above parameters, the SysConfig generated code does following:
    - Call \ref PRUICSS_init in System_init, and \ref PRUICSS_deinit in System_deinit
    - Enable the Core Clock and set the selected frequency
    - Create macros like `CONFIG_PRU_ICSS1` using the name passed in SysConfig. This is used as an input to \ref PRUICSS_open API.
    - Set hardware register for enabling IEP Sync mode in System_init

## PRUICSS Interrupt Controller {#PRUICSS_INTC}

The interrupt controller (INTC) module maps interrupts coming from different parts of the device (mapped to PRU-ICSS instance) to a reduced set of interrupt channels.

The interrupt controller has the following features:
- Capturing up to 160 Events (inputs)
    - Upper 96 are external events
    - Lower 64 are internal events
- Supports up to 20 output interrupt channels
- Generation of 20 Host Interrupts
    - 2 Host Interrupts shared between the PRUs (PRU0 and PRU1) and TX_PRUs (TX_PRU0 and TX_PRU1).
    - 2 Host Interrupts for the RTU PRUs (RTU_PRU0 and RTU_PRU1).
    - 8 Host Interrupts exported from the PRU_ICSSG internal INTC for signaling the device level interrupt controllers (pulse and level provided).
    - 8 Host Interrupts (event 12 through 19) for the Task Managers.
- Each event can be enabled and disabled.
- Each host event can be enabled and disabled.
- Hardware prioritization of events.

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

For industrial communication protocol examples running on R5F, Host Interrupts 2 through 9 exported from PRU-ICSS are used for signalling an interrupt to R5F. As this mapping is programmable and varies from example to example, we have a `*_pruss_intc_mapping.h` file for different protocol examples which is used for passing \ref PRUICSS_IntcInitData structure while calling \ref PRUICSS_intcInit API.

### Sysconfig for INTC
You may use the sysconfig interface for PRU_ICSSG Local INTC mapping.
  \imageStyle{pruicss_intc_settings.png,width:60%}
  \image html pruicss_intc_settings.png " "

Sysconfig will create (see on right side generated code)
- \ref PRUICSS_IntcInitData structures for interrupt mapping named `icss0_intc_initdata`, `icss1_intc_initdata` for ICSS0, ICSS1 instances respectively in `ti_drivers_config.c`. This is for use in main app to initialize interrupt settings by passing these to \ref PRUICSS_intcInit. The previous interrupt settings are overwritten by this so please use it only once to initialize the interrupts.

### User Defined INTC Settings
Following is an example of one mapping from `${SDK_INSTALL_PATH}/source/industrial_comms/ethercat_slave/icss_fwhal/tiesc_pruss_intc_mapping.h` used for EtherCAT SubDevice.

The following line maps `PRU_ARM_EVENT2` to `CHANNEL6`.

```c
{PRU_ARM_EVENT2, CHANNEL6, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
```

`CHANNEL6` is mapped to the fourth host interrupt mapped to device level interrupt controller(Host Interrupt 6 out of 20) through this line.

```c
{CHANNEL6, PRU_EVTOUT4}
```

In AM64x, `PRU_ICSSG0_PR1_HOST_INTR_PEND_0`-`PRU_ICSSG0_PR1_HOST_INTR_PEND_7` (8 host interrupts) are mapped to `R5FSS0_CORE0_INTR_IN_120`-`R5FSS0_CORE0_INTR_IN_127`. This values are for ICSSG0 events mapped to R5FSS0 CORE0. For details regarding interrupt numbers on other cores, please refer to "9.4 Interrupt Sources" section in Technical Reference Manual(TRM) of AM64x, or corresponding section in TRM of other SoCs. These interrupt numbers can change from SoC to SoC, so please consult TRM before making any modifications to the interrupt map.

For the example mentioned above, interrupt number 124 (`R5FSS0_CORE0_INTR_IN_124`) should be used for `intrNum` parameter for \ref PRUICSS_registerIrqHandler. \ref PRUICSS_registerIrqHandler creates Hwi instance using \ref HwiP_construct API with the `intrNum` passed,

This mapping alone determines which interrupt number on R5F will be associated with a particular interrupt from PRUICSS. For example, in the code shown above, where ``PRU_ARM_EVENT2` maps to `CHANNEL6`, and  `CHANNEL6` maps to `PRU_EVTOUT4` can be modified to following, and the interrupt number on R5F would still remain the same.

```c
{PRU_ARM_EVENT2, CHANNEL7, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_EDGE},\
{CHANNEL7, PRU_EVTOUT4}
```

But the usefulness of channels is that channels allow us to map multiple PRU events  to a single channel and in turn to a single host interrupt. For example, take a look at the following mapping used for link interrupt.

```c
{MII_LINK0_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
{MII_LINK1_EVENT, CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
{CHANNEL1, PRU1}
```

This configuration maps both Port 0 and Port 1 link interrupts to a single channel and in turn to a single host interrupt for PRU1 (Host Interrupt 1 out of 20).

\note
Please refer to section "6.4.7 PRU_ICSSG Local INTC" in Technical Reference Manual(TRM) of AM64x for more details on INTC module

## Example Usage

Include the below file to access the APIs
\snippet Pruicss_sample_g_v0.c pruicss_include

Instance Open Example
\snippet Pruicss_sample_g_v0.c pruicss_open

Sequence for loading a firmware on PRU and running the PRU core
\snippet Pruicss_sample_g_v0.c pruicss_run_firmware

## PRUICSS ENET Driver

The ICSSG can be used as a generic Layer 2 Ethernet Switch or Dual Mac. The Ethernet Low Level Driver (Enet-LLD) APIs can be used to realize this networking capability using the ICSSG sub-system.

\ref enetlld_top

### ENET ICSSG Examples
\ref EXAMPLES_ENET_LAYER2_ICSSG  
\ref EXAMPLES_ENET_ICSSG_LOOPBACK  
\ref EXAMPLES_ENET_LWIP_ICSSG | \ref EXAMPLES_ENET_LWIP_ICSSG_TCPSERVER  
\ref EXAMPLES_ENET_VLAN_ICSSG  
\ref EXAMPLES_ENET_ICSSG_TAS  
\ref EXAMPLES_ENET_ICSSG_TSN_GPTP_TR | \ref EXAMPLES_ENET_ICSSG_TSN_GPTP_TT | \ref EXAMPLES_ENET_ICSSG_TSN_GPTP_BRIDGE | \ref EXAMPLES_ENET_ICSSG_TSN_LWIP_GPTP  

### Queue Usage
The ICSSG Queues used for packet content buffering are elaborated here:  
File : {Any of above ENET ICSSG examples} CCS Project > Generated Source > SysConfig > ti_enet_soc.c 

ICSSG_SWITCH_PORT_POOL_SIZE :
- Buffer for the purpose of Forwarding of Frames from Port A to Port B.
- Size: Configurable ( By default 6KB)
- Totally 8 such pools are allocated for each of the QoS levels.

ICSSG_SWITCH_HOST_POOL_SIZE:
- Buffer for the purpose of transmitting frames from the host to Port A or/and Port B.
- Size: Configurable ( By default 6KB for 1Gbps Support and 3KB for 100Mbps Support)
- Totally 'n' such pools are allocated for each of the "QoS  Level" configured via SysConfig for each of the ports.

ICSSG_SWITCH_HOST_QUEUE_SIZE:
- Buffer for the purpose of reception of frames from the Port A or/and Port B to the host.
- Size: Configurable ( By default 8KB for 1Gbps Support and 5KB for 100Mbps sSupport)
- Totally 2 such pools are allocated for each of the ports.

#### To reduce number of Pools

ICSSG_SWITCH_PORT_BUFFER_POOL_NUM:  The number of ICSSG Port buffer pools is by default defined as 8 (max) to provide a unique Pool for upto 8 QoS levels.

This number can be reduced as per the requirement of the user application, for example 'n' (n = 1 to 8). However, care must be taken to ensure all priorities (PCPs), in case of VLAN-tagging, are mapped to only those 0 to n-1 Queues.

This can be done by using the IOCTL: ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP 

For Example if setting ICSSG_SWITCH_PORT_BUFFER_POOL_NUM = 3, Then the available pools will only be 0 ,1, 2.
Hence, all the traffic must be directed to only these queues 0 to 2 by using the above IOCTL. Type of mapping is left to the user.

This PCP to Queue mapping can be done via the input argument of the IOCTL ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP.  
`uint32_t EnetPort_PriorityMap::priorityMap[ENET_PRI_NUM]` , where the array index corresponds to the PCP and the value holds the mapped Queue value 

<table>
<tr>
  <th>PCP(Index)
  <th>Queue Number = priorityMap[pcp]
<tr>
  <td>0
  <td>0
<tr>
  <td>1
  <td>0
<tr>
  <td>2
  <td>0
<tr>
  <td>3
  <td>1
<tr>
  <td>4
  <td>1
<tr>
  <td>5
  <td>2
<tr>
  <td>6
  <td>2
<tr>
  <td>7
  <td>2
</table>

Sample IOCTL usage: ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP

```c
/* Mapping the PCP to Queue*/   
EnetMacPort_SetEgressPriorityMapInArgs SetEgressPriorityMapInArgs;
EnetPort_PriorityMap PriorityMap;          
PriorityMap.priorityMap[0] = 0;
PriorityMap.priorityMap[1] = 0;
PriorityMap.priorityMap[2] = 0;
PriorityMap.priorityMap[3] = 1;
PriorityMap.priorityMap[4] = 1;
PriorityMap.priorityMap[5] = 2;
PriorityMap.priorityMap[6] = 2;
PriorityMap.priorityMap[7] = 2;

SetEgressPriorityMapInArgs.macPort = macPortList[i];
SetEgressPriorityMapInArgs.priorityMap = PriorityMap;
ENET_IOCTL_SET_IN_ARGS(&prms, &SetEgressPriorityMapInArgs);
ENET_IOCTL(hEnet, coreId, ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, status);
if (status != ENET_SOK)
{
    EnetAppUtils_print("EnetApp_enablePorts() failed to set PCP to Q map: %d\r\n", status);
}
```
## API

\ref DRV_PRUICSS_MODULE
