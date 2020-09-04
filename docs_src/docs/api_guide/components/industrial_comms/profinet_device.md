# Profinet Device FWHAL {#PROFINET_DEVICE_FWHAL}

[TOC]

## Introduction

This software is designed for the TI SoCs with PRU-ICSS IP to enable customers add Profinet Device protocol support to their system. It provides Profinet ASIC like functionality integrated into TI SoCs.

\image html Profinet_Device_Software_Architecture.PNG "Software Architecture"

Profinet firmware for PRU-ICSS is a black box product maintained by TI. Profinet Device FWHAL (Firmware and Hardware Abstraction Layer) allows loading and running Profinet firmware and acts as an interface with the firmware. FWHAL implements the key interface between Profinet Device firmware and Profinet Device firm stack. It provides stack interface for CPM/PPM management, Triple Buffer Management, MRP, DCP Filter, Multicast Filter, Phase management and PTCP modules.

## PRU-ICSS Profinet Device Firmware

### Features Supported

- Supports minimum cycle time of 250 us
- Integrated two-port cut-through switch, 100 Mb/s Full Duplex
    - Relative forwarder, computes the Forward FSO for RTC3 frames which have to be forwarded
- PROFINET Quality of Service (QoS)
    - Four priority receive queues on host port, each queue 6 KB in size
    - Four priority transmit queues on each physical port, each queue 3 KB in size
- Up to 8 Application Relations (ARs)
- 8 IOCRs
    - 8 Consumer Protocol Machines (CPM)
    - 8 Provider Protocol Machines (PPM)
    - Supports PROFINET IO data size from 40 to 1440 Bytes
- Data Hold Timer
- DCP Identify Filter
    - DCP Identify frame is given to host only if it is meant for it otherwise it is just forwarded.
    - Reduces the DCP Identify frames reaching host at a particular node at network startup
- One Step Time Synchronization (PTCP)
- 1 millisecond buffering per port
- 802.1d learning bridge for received source MAC addresses
- PNIO static routing and custom FDB for multicast addresses
- Interrupt Pacing
- Isochronous Mode
- Firmware based on 200 MHz clock frequency for PRU-ICSS Core Clock and IEP Clock

### Features Not Supported

- Media Redundancy Protocol (MRP)
    - Bump less transition of PROFINET connection to redundant path on ring break
    - Switch address learned table (FDB) is flushed in 2.4 micro second
- Profinet IRT MRPD support
- Profinet IRT High Performance Profile

## Important Files and Directory Structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/industrial_comms</td></tr>
<tr>
    <td>profinet_device_demo</td>
    <td>Profinet Device Examples (based on pre-integrated stack)</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/industrial_comms/profinet_device</td></tr>
<tr>
    <td>icss_fwhal/firmware</td>
    <td>Firmware for the PRU cores in PRU-ICSS. **Firmware Version : 0.15.1** </td>
</tr>
<tr>
    <td>icss_fwhal/lib/</td>
    <td>FWHAL library for Profinet Device</td>
</tr>
<tr>
    <td>icss_fwhal/*.c</td>
    <td>FWHAL source files</td>
</tr>
<tr>
    <td>icss_fwhal/*.h</td>
    <td>FWHAL interface files</td>
</tr>
<tr>
    <td>stack</td>
    <td>Stack header files and stack library</td>
</tr>
</table>

## Terms and Abbreviations

<table>
<tr>
    <th>Abbreviation
    <th>Expansion
</tr>
<tr>
    <td>PRU-ICSS</td>
    <td>Programmable Real-Time Unit Industrial Communication Subsystem</td>
</tr>
<tr>
    <td>CPM</td>
    <td>Consumer Protocol Machine</td>
</tr>
<tr>
    <td>PPM</td>
    <td>Producer Protocol Machine</td>
</tr><tr>
    <td>DHT</td>
    <td>Data Hold Timer</td>
</tr><tr>
    <td>AR</td>
    <td>Application Relation</td>
</tr><tr>
    <td>NRT</td>
    <td>Non Real Time</td>
</tr><tr>
    <td>LLDP</td>
    <td>Link Layer Discovery Protocol</td>
</tr><tr>
    <td>MRP</td>
    <td>Media Redundancy Protocol</td>
</tr><tr>
    <td>DCP</td>
    <td>Discovery and Basic Configuration Protocol</td>
</tr>
</table>

## API Documentation

Please see \ref INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE for API documentation. It is recommended to use these FWHAL APIs in the stack adaptation files.

## Procedure to kick-off the Profinet Device

- Initialize ICSS-EMAC to work as a switch
- Initialize the required tasks and interrupts
- Initialize the Profinet Driver, PTCP and Memory (CPM/PPM Buffers and lists) and PRU-ICSS INTC
- Load Profinet firmware into PRUs of PRU-ICSS
- Initialize the Profinet device stack
- Start firmware
- Handle the events as needed. The event/ISR definitions are availale in iRtcDrv.c, these have respective callbacks that can be registered to custom stack APIs.

## Integration with ICSS-EMAC

Profinet examples use the ICSS-EMAC as its base switch layer. The PRU Firmware is customized for Profinet functionalities. The NRT (non-real time) traffic is handled by ICSS-EMAC, wherein the packets are forwarded either to the TCP stack or a custom custom callback (configurable in ICSS-EMAC).

### Interface with ICSS-EMAC
In case of Profinet firmware, the queues are designed as shown in the diagram below. The highest priority queues are used for LLDP, MRP, DCP, RTA and PTCP. These packets are directly forwarded to the registered callback in ICSS-EMAC (using rxRTCallBack). And the Queues 2 and 3 are forwarded to TCP/IP. Profinet FWHAL and stack also uses the TX/RX APIs available in the ICSS-EMAC for transmission/reception of packets like PTCP, LLDP, etc. More info on ICSS-EMAC can be found [here](\ref ICSS_EMAC).

\image html Profinet_Device_Priority_Queue_Decision_Block.PNG

## OS Components
This section describes the OS entities used by Profinet Device FWHAL.

### Interrupts
Profinet Device firmware generates the following interrupts.

8 Host Interrupts (Host Interrupts 2 through 9) are exported from the PRU_ICSSG internal INTC for signaling the device level interrupt controllers. PRU_EVTOUT0 to PRU_EVTOUT7 corresponds to these eight interrupts in the following table. Please check \ref PRUICSS_INTC section for more details.

<table>
<tr>
    <th>Name
    <th>Host Interrupt
    <th>Description
</tr>
<tr>
    <td> Frame Receive
    <td> PRU_EVTOUT0
    <td> Notifies host when firmware has stored a frame in host receive queue
</tr>
<tr>
    <td> PPM Frame Transmit Completion
    <td> PRU_EVTOUT1
    <td> Raised when firmware has transmitted a PPM frame
</tr>
<tr>
    <td> CPM Frame Receive
    <td> PRU_EVTOUT2
    <td> On reception of CPM frame firmware raises this interrupt
</tr>
<tr>
    <td> DHT
    <td> PRU_EVTOUT3
    <td> Firmware notifies DHT event and PPM list toggle event to host through this interrupt
</tr>
<tr>
    <td> PTCP
    <td> PRU_EVTOUT4
    <td> Firmware notifies reception of RTSync frame
</tr>
<tr>
    <td> Link Change
    <td> PRU_EVTOUT6
    <td> Interrupt is raised when the link on Ethernet PHY comes up or goes down
</tr>
<tr>
    <td> ISOM
    <td> PRU_EVTOUT7
    <td> Interrupt is raised ISOM event occurs. Applicable for ISOM Interrupt mode only
</tr>
</table>

### Tasks

<table>
<tr>
    <th> Task Function Name
    <th> Description
</tr>
<tr>
    <td> PN_PTCP_task
    <td> PTCP delay measurement scheduling
</tr>
<tr>
    <td> PN_PTCP_syncMonitorTask
    <td> PTCP sync monitor scheduling
</tr>
<tr>
    <td> PN_IRT_legacyTask
    <td> Legacy startup mode scheduling
</tr>
<tr>
    <td> PN_MRP_CPMTask
    <td> MRP Task to control flush mode of ICSS
</tr>
<tr>
    <td> PN_tapWatchDog_task
    <td> Tap the ICSS WatchDog Timer
</tr>
</table>

### Semaphores

<table>
<tr>
    <th> Name
    <th> Description
</tr>
<tr>
    <td> ptcpStartSem
    <td> Indicates start of PTCP
</tr>
<tr>
    <td> ptcpTriggerMeasurementSem
    <td> Triggers delay measurement
</tr>
</table>

## CPM/PPM Management

Profinet FWHAL supports upto 8 PPMs and CPMs. These CPMs/PPMs are managed internally as two lists of type (\ref t_descList). In case of PPM, the descriptor buffer (pDescs of \ref t_descList) has two instances to maintain an \ref ACTIVE_LIST and \ref SHADOW_LIST of buffers. Any data update to the PPM descriptors happens only on the \ref SHADOW_LIST. This data is picked up by the PRU after a list toggle (\ref PN_togglePpmList), which is currently done in the \ref PN_insPpmList. In the PPM descritor list, at any time the descriptors are sorted by port and phase, which is done by an internal API (\ref PN_writeSortedList). CPM does not have the concept of Shadow and Active list. The API definitions can be found [here](\ref PN_CPM_PPM_MANAGEMENT).

### Initialization

The initialization of the Profinet system, lists and buffers are done in \ref PN_initDrv. All the OS related initializations are done in
\ref PN_initOs.

Taking a deeper look at \ref PN_initDrv, it initializes the following (in order of execution):
- Sets the Port 1 and Port 2 MACs in PRU memory
- Sets the Compensation value - \ref PN_setCompensationValue
- Calls \ref PN_initRtcDrv
    - Initializes and clears the CPM/PPM lists (`PN_initLists` initializes the \ref t_descList and \ref PN_clearList)
    - Sets the base clock - \ref PN_setBaseClock
    - Initializes the Profinet Interrupts (\ref PN_cpmIsrHandler, \ref PN_ppmIsrHandler and \ref PN_dhtIsrHandler)
- Initializes PTCP - \ref PN_PTCP_init
- Loads the static tables
- Writes the Profinet firmware to the PRU memory, and checks for correct version
- Initialize both ports in \ref FORWARDING state (for \ref PROFINET_DEVICE_MRP)

Once \ref PN_initDrv and \ref PN_initOs is done, the driver memory (descriptor buffers) have to be initialized. This is done by the API \ref PN_cfgRtcMem. This API initializes two things :
1. The driver memory blocks ppmBlock0, ppmBlock1 and cpmBlock (using `PN_initRtcBuffs`).
2. Internal structure storing the Profinet configuration (currPN of \ref PN_Handle). This structure stores the maximum size of CPM/PPM, number of ARs supported and the base structure for PPM/CPM (\ref t_rtcPacket).

### Establishing a connection
Whenever a connection has to be established, the stack should update the connection data in the internal driver lists. This can be done using the APIs \ref PN_insPpmList and \ref PN_insCpmList. For configuring the CPM/PPM lists, the stack has to first allocate a buffer (in the driver). This can be done using \ref PN_allocPkt. This initializes the packet input to the API, finds a free slot in the list (ppmPkts/cpmPkts of currPN of \ref PN_Handle) and allocates a particular block \ref t_ppmBlock or \ref t_cpmBlock to the buffer.

#### Establishing a PPM connection
Once the buffer is allocated, the stack can update the data in the structure (\ref t_rtcPacket) and call \ref PN_insPpmList to update the driver with the new data. This API sorts the \ref t_descList in terms of Port and Phase and requests the PRU to toggle the list (i.e., shadow to active list) using \ref PN_togglePpmList . When the PRU is ready, it toggles the list and sends out the new data (also fires the \ref PN_ppmIsrHandler).

\note All data updates are done to the \ref SHADOW_LIST only.

#### Establishing a CPM connection
Similar to establishing a connection in PPM, \ref PN_insCpmList is used to update the driver with a new connection. As the concept of Shadow lists is not present in CPM, the data is directly inserted to the list using \ref PN_writeCpmDesc. It also locks the PROC buffer (PROC buffer is explained in \ref PROFINET_DEVICE_TRIPLE_BUFFER_MANAGEMENT).

### Updating CPM/PPM data
Once the connection is established, the CPM and PPM data can be updated using the Triple Buffer Scheme. Read more about it [here](\ref PROFINET_DEVICE_TRIPLE_BUFFER_MANAGEMENT).

## Triple Buffer Management {#PROFINET_DEVICE_TRIPLE_BUFFER_MANAGEMENT}

The Triple Buffer scheme ensures that the stack or PRU always finds a buffer to update. In case of PPM, the stack is the Producer and PRU consumes this, and in case of CPM, the PRU is the producer and Stack consumes this. Each CPM/PPM descriptor has three buffers associated with it, these buffers are indexed by three pointers - next, last and proc (in \ref t_rtcPacket). There is also a validLast flag in the same structure that indicates there is an update in data (CPM or PPM, set correspondingly by PRU or stack).
- NEXT
    - Index to the next available buffer.
    - The stack/PRU writes to this buffer whenever there is new data.
- LAST
    - Index to the last updated buffer.
    - At any point of time, this buffer contains the latest data to be consumed (CPM) or produced (PPM).
- PROC
    - Index to the buffer being currently processed.
    - In case of CPM, this indicates the buffer which is currently being consumed by the stack. This buffer will be locked by the stack (using \ref PN_cpmBuffLock). This indicates to PRU that the PROC buffer cannot be written. The PRU ping pongs between NEXT and LAST buffer to update data.
    - In case of PPM, this indicates the buffer being send by the PRU. The stack updates the NEXT and LAST buffer in ping pong fashion until the PRU is ready to send updated data (by \ref PN_checkLastPPM)
- validLast
    - In case of CPM, the PRU sets this flag whenever there is updated data in the LAST buffer. The stack calls \ref PN_nextCpmRdy when it is ready to consume a new packet, and checks this flag for any new data.
    - In case of PPM, when the stack has updated data in the LAST buffer, this flag is set (\ref PN_relPpmBuff). When the PRU is ready to take a new buffer, \ref PN_checkLastPPM is called and the new PPM is produced by PRU.

The API definitions can be found [here](\ref PN_IRT_TRIPLE_BUFFER).

### PPM Triple buffer management

In case of PPM, the stack is the Producer, and PRU consumes it to send it out on the wire.

When the stack has a new PPM to be updated to send, it calls the API \ref PN_getPpmBuff, which returns the a free buffer in the triple buffer(NEXT). The stack then starts updating data in NEXT (marked by grey box in the following diagram).

\image html Profinet_Device_ppm_triplebuffer_1.png "Figure 1"

When the stack has completed updating data (marked by Green box in the following diagram to NEXT, it calls the API \ref PN_relPpmBuff. At this point, the LAST index and NEXT index is swapped. This means that the LAST data (previous) is discarded and we have a new LAST data (the validLast flag is set). If the stack wants to further update new data, the NEXT pointer is again used.

\image html Profinet_Device_ppm_triplebuffer_2.png "Figure 2"

Next, when the PRU is ready to update data (which is send on the wire), \ref PN_checkLastPPM is triggered. At this point, the PROC and LAST indexes are swapped. This enables PRU to send the LAST data, which is the latest available PPM data at any point of time. The validLast flag is cleared, which indicates the PRU that there is no update in the data. Afer this, the stack has NEXT and LAST buffers to update future data (in a ping pong fashion) as shown in Figure 1.

\image html Profinet_Device_ppm_triplebuffer_3.png "Figure 3"

### CPM Triple buffer management

In case of CPM, the PRU is the Producer, and the stack consumes it to use the data.

The stack always reads from the PROC buffer and also has it locked using \ref PN_cpmBuffLock. The PRU never writes into this buffer. Figure 4 shows that the PRU is writing in NEXT buffer, while PROC (Buffer 2) is locked by the stack.

\image html Profinet_Device_cpm_triplebuffer_1.png "Figure 4"

When the PRU has new data to be written, the NEXT and LAST indexes are swapped. This indicates that LAST is pointed to the latest available data (to be consumed by the stack). If the PRU has updated data to be written, it uses the NEXT buffer. The PRU keeps continuing in the loop i.e., switching the NEXT and LAST buffers in a ping pong fashion till the stack consumes the LAST data.

\image html Profinet_Device_cpm_triplebuffer_2.png "Figure 5"

When the stack is ready to consume new data, it calls the API \ref PN_getLastCpm. A swap of LAST and PROC indexes in done here. This indicates the stack consumes the latest data available at any point of time. The stack also should lock the buffer using \ref PN_cpmBuffLock. At this point PRU starts updating any new data in the NEXT buffer.

\image html Profinet_Device_cpm_triplebuffer_3.png "Figure 6"

## Media Redundancy Protocol (MRP) {#PROFINET_DEVICE_MRP}

A state machine is implemented by `PN_MRP_CPMTask` to implement MRP. If a break is detected in the topology, this state machine switches to a state where the PRU sends PPM on both ports. It is also made sure that the FDB for both the ports are cleared. Once all the CPMs learn the correct port number (port of \ref t_rtcPacket), the flush mode is deactivated. The API definitions are present [here](\ref PN_MRP).

\note To enable MRP, it is required to set the macro `MRP_SUPPORT` in `${SDK_INSTALL_PATH}/source/industrial_comms/profinet_device/RT_MRP/pnDrvConfig.h` file

### MRP State Machine

The MRP state machine has 5 states defined by \ref tMrpStates. The state machine flow chart is shown in Figure 7.

\image html Profinet_Device_mrp_state.png "Figure 7"

- \ref MRPREADY : At initialization the state machine is set to this. This is an idle loop that does nothing, and indicates all links are OK.
- \ref MRPENTER : Whenever a link break is detected. The stack calls \ref PN_enterFlushMode to switch the state machine to MRPENTER. In this state, the port(port of \ref t_rtcPacket) of all the CPM packets are set to 0(invalid state), and the PRU is indicated to switch to Flush mode.
- \ref MRPWAIT : This state is used to switch between \ref MRPCHECK and \ref MRPWAIT with a sleep.
- \ref MRPCHECK : This state checks if the port of all CPMs are learned, if yes it goes to \ref MRPEXIT, else sleep(going to \ref MRPWAIT).
- \ref MRPEXIT : Resets the state machine back to \ref MRPREADY.

## Profinet IRT Legacy Startup Support

The Profinet IRT Legacy startup feature is implemented using a state machine `PN_IRT_legacyTask`.

Before a connection is established(i.e., `RTC_CPM_STATUS_OFFSET` is not set to `RTC_CPM_RUN`), if legacy mode is enabled, the driver configures a PPM packet to be send in Green period. The API definitions are present [here](\ref PN_IRT_LEGACY).

\note To enable Legacy startup, it is required to set the macro `IRT_LEGACY_STARTUP_SUPPORT` in `${SDK_INSTALL_PATH}/source/industrial_comms/profinet_device/IRT/pnDrvConfig.h` file

### Profinet IRT Legacy Startup State Machine

The IRT Legacy Startup state machine has 4 states defined by \ref tLegStates. The state machine flow charts is shown in Figure 8.

\image html Profinet_Device_irt_legstate.png "Figure 8"

- \ref NOINIT : At initialization the state machine is set to \ref NOINIT. This state waits for the base driver to be ready, and then switches the state machine to \ref READY.
- \ref READY : This state is an idle loop, which waits for PPM packet to be initialized.
- \ref SENDPPM : When the PPM packet is ready, the application sets the state to \ref SENDPPM (by callback function `irtLegStateCall`. When this is done, the driver will send out the latest PPM data from `pLegPkt` (which is set by the callback `irtLegPktCall`). The driver calls the callback in \ref insPpmList if the input param to the API has legMode configured). The packet is send out in intervals, till the connection is established.
- \ref ABORT : Error state.

## Phase Management

Every cycle in Profinet IRT is split into a RED and GREEN period. The driver has APIs to configure these periods, which is mostly done during establishment of connection. GREEN period can be further classified into Green steady period and Yellow period. If a packet has to be transmitted during Yellow period, the driver has to check if the packet can be send out in the remaining time left. Otherwise it is send out in the next green period. The API definitions are present [here](\ref PN_PHASE_MANAGEMENT).

\image html Profinet_Device_phase_mgmnt.png

### Implementation

- The phases are mapped to profiles(which holds the information regarding RX start of green period and TX start of green period). There can be 5 profiles each for a port, which is configured by \ref PN_setProfile.
- The 16 phases in Profinet can be mapped to any of these Profiles using \ref PN_mapPhaseToProfile. All this configuration
should be done during the connection establishment.
-The enabling/disabling of Phase management is based on the RTC3 port state. Phase management is disabled if RTC3 port
state is \ref OFF, and enabled if it's \ref RUN or \ref UP.
- The stack also has to configure the Max Bridge delay(\ref PN_setMaxBridgeDelay), Max Line RX Delay(\ref PN_setMaxLineRxDelay), and
the Yellow period(\ref PN_setYellowPeriod). This information is available from the GSD file.
- The RED guard also should be set by the stack, which defines a range of valid Red frame IDs(\ref PN_setRedGuard).
- The API \ref PN_setRtc3PortStatus can be used by the stack to set the Port status to enable RTC3 on the port(valid states \ref RUN, \ref UP and \ref OFF).

## PTCP

The API definitions are present [here](\ref PN_PTCP).

### Implementation

PTCP is implemented in the PRU and PTCP driver. It mainly has two parts

#### SYNC handling

PTCP has an ISR \ref PN_PTCP_isrHandler which is triggered when a SYNC packet is received from the PRU. The ISR internally calls an important API \ref PN_PTCP_syncHandling which calculates the deltaT between the controller and device, and according initiates the clock adjustment process. There is also a filter implemented which makes sure the compensation is done only once the deltaT is stable. The driver also maintains a task `PN_PTCP_syncMonitorTask` to monitor if there is a sync timeout.

#### Delay Handling
Another task `PN_PTCP_task` does the handling of delays (cable delay, line delay etc.). This task sends out multiple delay requests in a particular interval, and processes the delay responses(\ref PN_PTCP_delayMeasurement). The calculated line delays (\ref deviceDelays_t) and port delays (\ref ptcpPortDelayVal_t) are stored in the internal structures.

### Integrating with the stack

The driver provides two main callbacks that can be used by the stack for certain even triggers. They are `ptcpSyncStatusCall` and `ptcpDelayUpdateCall` (which can be registered by calling the APIs \ref PN_PTCP_registerSyncStatusCall and \ref PN_PTCP_registerDelayUpdateCall). The  `ptcpDelayUpdateCall` is triggered whenever there is any update in the delays calculated. The `ptcpSyncStatusCall` is used by Sync to notify any update in the SYNC state. It also indates error events like \ref TAKEOVER_TIMEOUT and \ref SYNC_TIMEOUT.

## Isochronous Mode

The firmare supports generation of two ISOM events in a network cycle. \ref PN_ISOM can be used to
to configure the firmware.

### Implementation
The ISOM implementation supports configuration of two types of ISOM events: Interrupt mode and GPIO mode.

- Interrupt Mode : Interrupts will be generated at configured intervals. The details of the Hardware interrupt (Interrupt number and ISR) need to be configured in the ISOM structure in PN handle. Firmware uses SYNC1_OUT_EVENT to trigger the intterupt. By default this event is mapped to PRU_EVTOUT7.

- GPIO Mode : In this mode, the firware is configured to generate signals at specific intervals. The signals are available at the pin prX_edc_sync1_out. Current implementation supports only single pulse width configuration for the signals. Pulse width used in latest configuration is taken.

## DCP Filter

In a chain of devices, during startup the first device receives a storm of Ident Requests. As all these packets go to ARM, there is a possibility that the device might lose the Ident Req which is meant for the device. To avoid this situation, the DCP Filter can be used. This ensures that only the Ident Request packet with the configured station name is forwarded to ARM. A maximum of 8 characters is compared by the firmware(last 8 characters). If the length of station name is set to 0, this indicates the DCP Filter is not in use. The API definitions are present [here](\ref PN_DCP_FILTER).

## Multicast Filter Tables

The stack has to configure the Multicast filter table. This is done by the API \ref PN_setStaticFilterTable. There are two tables, \ref FORWARDING table and \ref BLOCKING table. The Multicast addresses can be configured to function in one of these states:
- \ref NO_RCV_NO_FWD
- \ref RCV_NO_FWD
- \ref NO_RCV_FWD
- \ref RCV_FWD

The API definitions are present [here](\ref PN_FILTER_TABLE).

## Watchdog Timer

Watchdog timer is used as a safety feature to monitor the application state and to turn off the PPM transmission after pre-defined interval if application is not responding. The watchdog will thereby protect the system from errors or faults by timeout or expiration. The expiration is used to initiate corrective action in order to keep the system in a safe state and restore normal operation based on configuration. Therefore, if the system is stable, the watchdog timer should be regularly reset or cleared to avoid timeout or expiration.

Application can use the API \ref PN_setWatchDogTimer (defined in iPNDrv.c) to set the timeout value. Application needs to enable the `WATCHDOG_SUPPORT` macro in `${SDK_INSTALL_PATH}/source/industrial_comms/profinet_device/IRT/pnDrvConfig.h` or `${SDK_INSTALL_PATH}/source/industrial_comms/profinet_device/RT_MRP/pnDrvConfig.h` file to use this feature. By default the PROFINET Device driver sets the timeout to 100ms (watchDogExpireDuration defined in iPndrv.h) using
`PN_setWatchDogTimer(pnHandle, watchDogExpireDuration)`. The PN_tapWatchDog_task task defined in iPnOs.c resets the watchdog timer periodically.

This timeout is system dependent and is the responsibility of user to set the trigger frequency and the timeout. If watchdog timer expires, the firmware will stop all active PPM connections.

Application can check the expiry state of watchdog timer using WD_STATUS register in ICSS IEP. Bit 0 of WD_STATUS is PD_WD_STAT. An example is shown below.
- Condition for Active or disabled: 0x0001 == HW_RD_REG16(emacBaseAddr->prussIepRegs + CSL_ICSSIEP_WD_STATUS_REG)
- Condition for Expired: 0x0000 == HW_RD_REG16(emacBaseAddr->prussIepRegs + CSL_ICSSIEP_WD_STATUS_REG)

The API definitions are present [here](\ref PN_WATCHDOG).

## Additional References {#PROFINET_DEVICE_ADDITIONAL_REFERENCES}

Please refer to below documents to understand more about Profinet Device on TI platforms and Profinet Device protocol specifications.

<table>
<tr>
    <th>Document
    <th>Description
</tr>
<tr>
    <td>[PROFINET on TI's Sitara processors](https://www.ti.com/lit/pdf/spry252)
    <td>Application note by TI on the Profinet implementation on TI's Sitara Processors.
</tr>
</table>

