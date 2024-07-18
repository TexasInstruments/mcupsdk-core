# ICSS TimeSync Design {#ICSS_TIMESYNC_DESIGN}

[TOC]

## Terms and Abbreviations

<table>
<tr>
    <th>Term/Abbreviation
    <th>Expansion
</tr>
<tr>
    <td>PRU-ICSS</td>
    <td>Programmable Real-Time Unit Industrial Communication Subsystem</td>
</tr>
<tr>
    <td>PTP</td>
    <td>Precision Time Protocol</td>
</tr>
<tr>
    <td>E2E</td>
    <td>End to End</td>
</tr>
<tr>
    <td>P2P</td>
    <td>Peer to Peer</td>
</tr>
<tr>
    <td>TC</td>
    <td>Transparent Clock</td>
</tr>
<tr>
    <td>OC</td>
    <td>Ordinary clock</td>
</tr>
<tr>
    <td>Time Source</td>
    <td>PTP Time Source</td>
</tr>
<tr>
    <td>Receiver</td>
    <td>PTP Receiver</td>
</tr>
<!-- <tr>
    <td>HSR</td>
    <td>High Speed Seamless Redundancy</td>
</tr>
<tr>
    <td>PRP</td>
    <td>Parallel Redundancy Protocol</td>
</tr> -->
<tr>
    <td>RCF</td>
    <td>Syntonization/(Frequency Compensation) factor</td>
</tr>
</table>

## Timer
ICSS TimeSync Driver uses the 64-bit free running IEP Timer inside PRU-ICSS running at 200 MHz as the base timer for all synchronization related activities. PRU-ICSS has the capability to timestamp entry and exit of a frame based on this timer.

## Sync Signal Generation

Sync signal is enabled in IEP with a sync pulse width that is relative to the sync signal generation interval. This sync is equivalent to the 1PPS output and should not be confused with PTP Sync frame.
For the sync signal generation CMP1 is programmed to a value ranging from 1ms to 1 second. Firmware checks this event and re-programs it after every hit to ensure that accurate sync pulses are generated.
The sync interval must not be configured such that the 1000000000 ns or 1 seconds is not an integral multiple of it. This will lead to sync signal generation which is not at the second boundary, this might impact synchronizing other devices using the sync output.

## Timestamping {#ICSS_TIMESYNC_DESIGN_TIMESTAMPING}

The timestamps are read from 64-bit free running IEP Timer inside PRU-ICSS running at 200 MHz. The timer has a resolution of 5 ns. The Rx and Tx timestamps are stored in PRU-ICSS Shared RAM. For Rx timestamps there is another mechanism where the timestamps are appended to the end of the frame being sent to host, this method isn't available for all protocols. In the driver, this mode is controlled by the user configurable variable `timestamp_from_shared_ram` (in structure \ref TimeSync_Config_t).

For Rx, both Start of Frame (SOF) and Start of Frame Delimiter (SFD) timestamping is possible using IEP. For Tx, only SOF timestamp is available. Since PTP/1588 specifies that SFD time stamps are to be used we store the RX SFD time stamp for all frames and add 640ns to the Tx SOF time stamp, the underlying assumption being that the preamble length is 8 bytes. This is correct because PRU-ICSS inserts it's own preamble and CRC instead of copying from the received frames. It is also important to compensate the PHY delay and other known delay parameters in timestamps for accurate timing. For this purpose, `txPhyLatency` and `rxPhyLatency` (which are a part of the configuration structure \ref TimeSync_Config_t) should be configured properly during initialization.

Following are details on timestamping availability for different PTP packets :
- Sync : Rx and Tx timestamps
- Follow Up : No timestamping
- Announce : No timestamping
- Delay Response : No timestamping
- Delay Request : Entry and exit timestamping is done in nanoseconds. Exit timestamp is used in delay calculations while entry timestamp is used for bridge delay calculation.
- Peer Delay Request : Rx and Tx timestamps
- Peer Delay Response : Rx and Tx timestamps
- Peer Delay Response Follow Up : No timestamp

## State Machine

The PTP state machine has the following states:

1. Enabled
2. BMCA has run and Time Source has been determined
3. First adjustment done
4. Line/Peer delay computed
5. Sync interval computed
6. Ready for adjustment. This happens once previous states have been triggered.
7. Synchronized. Average clock drift has gone below a specified threshold.
8. Error (Announce timeout, very large adjustment, sync interval too large, missed sync frame)

When the firmware receives PTP packets initially, it puts all Announce and Management frames in highest priority queue and no sync signal is generated (until sync Time Source is determined). RCF value used is the default 1.0. Once BMCA has determined the Time Source, it's MAC ID should be provided to firmware using \ref TimeSync_updateParentAddress. The firmware checks against this MAC ID to put Sync frames in Host queue, Announce and Management frames are always forwarded to Host. Clock synchronization starts and RCF is computed along with other values.

The `stateMachine` variable is part of TimeSync runtime variable \ref timeSync_RuntimeVar_t which is a part of \ref TimeSync_ParamsHandle. It has 5 states:

1. First adjustment done
2. Line/Peer delay computed
3. Sync interval computed
4. Ready for Sync
5. Synchronized

They correspond to states 3-7 of the states described earlier. States 1, 2 and 8 are implicit states.

## Synchronization

Synchronization is about ensuring that the local clock on both Time Source and receiver reflect the same value (after adjustment for path delay). This is done by
1. Copying the value of clock directly at first (also called the "Initial adjustment")
2. Calculating the drift every time a sync frame comes and then adjusting the local clock to take care of the drift.

Synchronization is done in the Sync frame context and adjustment must be complete by the time next Sync packet arrives. To calculate the drift the assumption is that seconds field is synchronized and hence only nanoseconds field is compared to find the drift irrespective of the sync interval. Exception is when there is a wraparound and Seconds field is not the same, this can happen when the receiver is so far from the Time Source that seconds field increments by the time Sync frame reaches it.
Actual synchronization is done by the function \ref TimeSync_synchronizeClock which computes the current offset from the Time Source and calls the function \ref TimeSync_adjTimeSlowComp to adjust the IEP counter using slow compensation mechanism.

### Determination of Sync Period

- Sync frame interval is determined in the driver from Sync frames. The driver waits for at least 3 sync timestamps to do this.
- Driver also checks if there is a large difference in the known Sync Period and the observed value, if it is determined that Sync interval has changed, then state machine is initialized and PTP receiver has to sync all over again.
- Driver monitors if the time base has changed significantly (by 1 s or more). This excludes special cases such as leap59 and leap60. Such a condition triggers state machine to be initialized again and PTP receiver has to sync all over again.
- The sync frame interval is determined in the same function which calculates RCF. \ref TimeSync_calcRcfAndSyncInterval does this calculation.

## Delay Calculation

### Line Delay Calculation for E2E mode {#ICSS_TIMESYNC_LINE_DELAY_CALCULATION}

Line delay refers to the delay between PTP Time Source and Receiver. The line delay is calculated in response to the Delay request frame which is sent every time a Sync frame is received. This behavior is not mandated by the standard but is chosen for simplicity.

The API used for this is \ref TimeSync_lineDelayCalc. It is called once the firmware receives a Delay Response packet meant for the receiver. This processing is done in the function \ref TimeSync_processDelayResFrame.

The calculation of line delay (time distance between two nodes) is required for finding out things like clock drift and frequency syntonization factor (RCF). The mechanism for Line Delay is roughly the same for both Delay Request and Peer Delay packets.

The basic concept to understand here is that we want to find out the time it takes for a packet to traverse between the nodes. This is done by sending a packet from one node to another (usually from Time Source to receiver), which is called a sync packet. The sync packet contains a timestamp that indicates when it left the Time Source. This is present either in the correction field or the timestamp. If the device is incapable of providing an accurate timestamp in the sync packet, a follow up packet is sent which contains this information.

Upon receipt of the follow up information the receiver sends a Delay Request packet which again is timestamped like the sync packet with the exit timestamp. When a delay request is received at the Time Source, a Delay Response is sent meant only for that receiver which issued the Delay Request (Delay Response is tagged with Receiver information). The Delay Response contains the time at which Delay Request was received at the Time Source.

The entire process is depicted in the diagram below.

\image html ICSS_TIMESYNC_lineDelayCalculation.png

The timestamps are respectively indicated by t1, t2, t3 and t4.

Line delay is calculated as
`Mean Path Delay = (Forward Delay + Reverse Delay) / 2 = ((t2 – t1) + (t4 – t3))/ 2`

Since on both receiver and Time Source the counters are free counters, it makes more sense to re-arrange the computation like this.
`Mean Path Delay = ((t4 - t1) – (t3 – t2))/2`

### Peer Delay Calculation for P2P mode

Peer delay refers to the time delay between adjacent PTP nodes in a P2P network. The API for this is \ref TimeSync_peerDelayCalc. It is called inside the function \ref TimeSync_processPdelayRespFrame which processes the peer delay response frame. As soon as a response is detected the peer delay calculation runs, it is run for both the ports.
For an adjacent node requesting a Pdelay Response the receiver gives a 2-step response. Upon receiving the Pdelay Request, it's Rx timestamp is stored and when Pdelay Response goes out, it's Tx timestamp is used to calculate the resident time. This resident time is reflected in the correction field of Pdelay Response.

The main differences compared to E2E mode is that Peer Delay Request/Response messages are used instead of Delay Request/Response messages and Sync packets are not required.

Since Peer to Peer Requests are meant for adjacent nodes, it is important that these packets are dropped and not forwarded.

## Syntonization

Syntonization is accounting for the frequency difference between Time Source and Receiver. This is done by

1. Keeping track of timestamps of two alternate Sync frames (not consecutive).
2. Taking the difference in arrival timestamp (as recorded on receiver), let's call this "receiver time".
3. Taking the difference in origin timestamp (as recorded on Time Source), let's call this "Time Source time".

The RCF or syntonization factor is computed as RCF = Time Source time/receiver time. Any delay computed on receiver is then multiplied by this factor.

For example if Time Source is running twice as fast as receiver then RCF will be 2 and any delay computed on receiver will get multiplied by this value to reflect time in terms of Time Source. In reality the RCF rarely goes out of the range 0.99-1.01 and any value outside this should be interpreted as an error.

Timestamps are recorded and Syntonization is done in a PTP task context and not inside Sync frame processing. The function which calculates RCF is \ref TimeSync_calcRcfAndSyncInterval.

If we consider the image shown in \ref ICSS_TIMESYNC_LINE_DELAY_CALCULATION, RCF is calculated as follows.

T3 is the sync transmit timestamp from Time Source and T4 is the sync receive timestamp on receiver. T3' is T3 measured again in future (same for T4).
`RCF = (T3' – T3) / (T4' – T4)`
`Mean Path Delay (compensated) = Mean Path Delay * rcf;`

## Reset

This section talks about the ICSS TimeSync driver reset. The various PTP variables and state machine inside the driver require a reset every time an exception occurs. \ref TimeSync_reset can be called for this reset.

The steps performed during this reset are

1. Call the sync loss callback function \ref TimeSync_SyncLossCallBack_t
2. Disable firmware by calling the function \ref TimeSync_drvDisable
3. Clear all internal structures used by ICSS TimeSync driver
4. Following memory locations in PRU-ICSS Shared RAM need to be reset to 0
    - SYNC_MASTER_MAC_OFFSET
    - TIMESYNC_TC_RCF_OFFSET is set to 1024
5. `stateMachine` set to 0
6. `rcf` set to 1.0
7. Neighbor rate ratio or nrr for both ports set to 1.0
8. Runtime variables `clockDrift` and `syncTimeoutInterval` set to 10000
9. Offset Algo variables `driftThreshold` set to init value \ref TIMESYNC_OFFSET_STABLE_ALGO_THRESHOLD
10. Call the PTP stack callback function \ref TimeSync_stackResetCallback_t. This callback can be registered by PTP stack to get a reset notification.
11. Enable firmware by calling \ref TimeSync_drvEnable


The reset is called under the followng conditions

1. Sync frame missed : When a sync frame is missed this is detected by the sync timeout mechanism. Three consecutive misses leads to a reset.
2. Current offset out of range : Whenever absolute offset exceeds a threshold, reset is triggered. This comparison is done in the function \ref TimeSync_synchronizeClock. The threshold of change is set by the macro \ref OFFSET_THRESHOLD_FOR_RESET, value is in nanoseconds.

## Forwarding Rules

PTP frames have a common format but the implementation varies widely depending on the Annex or Profile in use. ICSS TimeSync driver supports following profile:

1. PTP over UDP in E2E mode: PTP packets are encapsulated inside UDP and the delay measurement scheme is End to End. In this scheme all wire delays are measured relative to the Time Source. This support is bundled with EtherNet/IP firmware.
<!-- 2. PTP over 802.3 in P2P mode (HSR/PRP): PTP packets are not encapsulated and the delay measurement scheme is Peer to Peer. In this scheme all wire delays are with respect to adjacent node. Sync, Follow Up, Announce and Management messages have HSR headers or PRP trailers embedded. This support is bundled with HSR/PRP firmware. -->


The forwarding rules for different frame types are as follows. Here forward means to Host as well as to the other port if link is available unless mentioned specifically.

1. Announce : Forward without correction
2. Sync : Forward with correction
3. Follow Up : Forward without correction
4. Delay Request : Forward with correction and Tx only
5. Delay Response : Forward without correction if it's not a delay response corresponding to our delay request and don't send to Host. If the delay response matches our delay request then send to host and don't forward.
6. Management messages : Forward without correction

<!-- ## Dual Clock Functionality

Dual clock functionality refers to getting two sync frames from the same source in a redundant network topology. Such a case is applicable to HSR/PRP networks with a PTP Time Source. In such networks two identical sync frames with different characteristics reach a target's node at the same time, however only one sync frame must be used for correction to avoid jitter. This is done by locking one port for receiving sync frames in the firmware. This is done by matching the port number with the value written in shared memory at the location `MASTER_PORT_NUM_OFFSET`.

Under reset this location contains the value 0 and this implies that any port can receive sync frames. Upon receiving the first sync the corresponding PRU firmware writes it's own port number into the location. This way the port number which receives sync frames gets fixed. When a sync timeout is detected this value gets reset to 0 ensuring that we keep getting sync interrupts from other ports. This is how the dual clock functionality is implemented.
 -->

## OS Components

This section describes the OS entities used by ICSS TimeSync. All initialization is again done in the function `TimeSync_isrAndTaskInit`.

### Interrupts {#ICSS_TIMESYNC_DESIGN_INTERRUPTS}

- Tx Timestamp Interrupt for Host : This interrupt is registered based the value provided in `txIntNum` which is a part of configuration structure \ref TimeSync_Config_t. This interrupt is mapped to the ISR `TimeSync_txTSIsr`.

- Rx Interrupt : All incoming PTP frames are handled through the regular Rx interrupt registered by ICSS-EMAC (See \ref ICSS_EMAC_DESIGN_INTERRUPTS section in \ref ICSS_EMAC_DESIGN for more details). \ref TimeSync_processPTPFrame must be called for processing PTP frames. Firmware will use highest priority queue for PTP frames. As mentioned in \ref ICSS_EMAC_DESIGN_DATA_PATH_RX, `rxRTCallBack` needs to be registered to process these frames in queues. Therefore, either \ref TimeSync_processPTPFrame must be registered as `rxRTCallBack`, or  if different types of frames (other than PTP) are coming on the high priority queues, then application must differentiate PTP frames in `rxRTCallBack` and then call the \ref TimeSync_processPTPFrame API for PTP frames.

\note All PTP frames are sent by firmware on the highest priority queue to the host.

### Tasks

ICSS TimeSync driver uses a multitude of tasks to send/process frames, act as a watchdog and for book keeping.

<table>
<tr>
    <th> Task Function Name
    <th> Description
</tr>
<tr>
    <td> `TimeSync_PdelayReqSendTask`
    <td> Send Peer Delay Requests periodically on both ports. Delay is configurable. Only applicable for P2P configuration.
</tr>
<tr>
    <td> `TimeSync_delayReqSendTask`
    <td> Send Delay requests to PTP Time Source. This is currently done for every Sync frame. Pends indefinitely on `delayReqTxSemObject` semaphore. Only applicable for E2E configuration.
</tr>
<tr>
    <td> `TimeSync_TxTSTask_P1`
    <td> Process Tx timestamp for Port 1. Pends on an event posted by Tx ISR indefinitely.
</tr>
<tr>
    <td> `TimeSync_TxTSTask_P2`
    <td> Same as above but for Port 2.
</tr>
<tr>
    <td> `TimeSync_NRT_Task`
    <td> NRT stands for non real time. Processes Peer delay frames in the background and calculate peer delay.
</tr>
<tr>
    <td> `TimeSync_BackgroundTask`
    <td> Checks for Sync timeout and performs offset stabilization.
</tr>
</table>

### Semaphores

<table>
<tr>
    <th> Name
    <th> Description
</tr>
<tr>
    <td> `delayReqTxSemObject`
    <td> This is posted when a Sync frame is received. It enables a suspended task to send Delay Request frame to PTP Time Source. Only applicable for E2E configuration.
</tr>
</table>

### Events

ICSS TimeSync driver uses a lot of events to process PTP frames. All events are in pairs, one each for physical port.

<table>
<tr>
    <th> Name
    <th> Description
    <th> Corresponding Event IDs
</tr>
<tr>
    <td> `txTSAvailableEvtObject`
    <td> Posted by the Tx ISR to indicate that a timestamp is available and processing can be completed
    <td> `eventIdSync` and `eventIdPdelayResp`
</tr>
<tr>
    <td> `ptpPdelayResEvtObject`
    <td> Used to process Peer Delay Response and Peer Delay Response Follow Up messages
    <td> `eventIdPdelayResp` and `eventIdPdelayRespFlwUp`
</tr>
<tr>
    <td> `ptpPdelayReqEvtObject`
    <td> Used to process Pdelay request messages and send a response
    <td> `eventIdPdelayReq`
</tr>
<tr>
    <td> `ptpSendFollowUpEvtObject`
    <td> Used to generate Follow up frame in case of forced 2-step mode
    <td> `eventIdSync` and `eventIdFlwUpGenerated`
</tr>
</table>

\note All the interrupts, tasks and semaphores created in an application can be checked using the ROV. For more details, see \htmllink{@VAR_MCU_SDK_DOCS_PATH/ROV_INTRO_PAGE.html#PRUICSS_INTC, Using SDK with Real-time Object View (ROV)}.
