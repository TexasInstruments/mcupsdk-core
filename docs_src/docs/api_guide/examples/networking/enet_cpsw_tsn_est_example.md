
# Ethernet TSN EST Example {#EXAMPLES_ENET_CPSW_TSN_EST}

[TOC]

# Introduction

  This example application demonstrates how to configure the IEEE 802.1 Qbv (EST) through our TSN yang interface.

  The yang interface in the TSN is governed by a module called uniconf which runs as a daemon. Any application which interacts with the uniconf is called as a uniconf client. The uniconf client configures 802.1 Qbv by
  opening yang database (DB), write config yang parameters to DB and triggers the uniconf for reading parameters from DB and writing to HW. The uniconf reads or writes parameters from or to HW by calling Enet LLD driver.

  Please note that the file system support is not yet integrated to ethernet examples.

  In this example, we configure the talker DUT to send out traffic as per the EST schedule and the listner DUT can verify the time-slots of the received packets.

  \note Host based receive packet time-stamping is enabled to estimate the packet reception timing accuracy, on the listner side.However, note we have a HW errata i2401 regarding this feature and hence host based rx packet timestamping feature should be disabled in production code.

See also : \ref EXAMPLES_ENET_CPSW_EST, \ref ENET_CPSW_TSN_GPTP

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/est_cpsw_app

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/est_cpsw_app

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/est_cpsw_app

\endcond

# Constraints

- This application only enables one MAC port which has the interface name *tilld0*.
  This interface is mapped to Mac port 1 for all MCU devices.
  To change default Mac port for this example, please change the macro
  *DEFAULT_INTERFACE_INDEX* from *qosapp_misc.h* from 0 another value (1, 2, 3).

- The application can be only run as talker, listener or bridge mode depending on  
  input characters from UART terminal. (t: talker, l: listener; b: bridge mode)

- Num of streams on talker is 2 with traffic priorities 0 and 2 mapped to HW queue 0
  and 2 respectively. Num of streams and priority can be changed by modifying the
  *gEnetEstAppTestLists* from the est_init.c.

- Both talker and listener need to apply the same EST schedule with the same *baseTime*
  to have accurate test results. Thus, there is a *delayOffset* added to the *baseTime*
  of the EST's AdminList so that both schedules can be applied in the future
  and at the same time.

- The EST schedule should be applied and talker starts sending test traffic
  after PTP synchronization is in good condition to have accurate test results.

- The talker sends dummy data which is encapsulated in an AVTP packet. Packet
  sent with VLAN ID 110.


# Configuration Parameters

 - Default mac port can be changed by modifying the `DEFAULT_INTERFACE_INDEX`
   from the `qosapp_misc.h`.

 - EST Schedule (Input parameters)

   <table>
   <caption> Gate control list </caption>
   <tr><th> Gate Control<th> Time Interval
   <tr><td> `oCCCCCCo`  <td> 62 usecs
   <tr><td> `oCooCoCC`  <td> 62 usecs
   <tr><td> `oCCCCCCo`  <td> 62 usecs
   <tr><td> `oCCCCoCC`  <td> 62 usecs
   </table>
  
Each of the 8 gates (one per priority) can be in one of two states:
-  *Open*: Frames in the corresponding queue can be selected for transmission.
          This is represented by '`o`' in the *Gate Control* column of previous table.
-  *Closed*: Frames in the corresponding queue are not selected for transmission.
  This is represented by '`C`' in the *Gate Control* column.

  The *priority* being referred to in the EST schedule described above corresponds
  to the CPSW destination port hardware *switch priority*.  For VLAN tagged packets
  that ingress through CPSW host port, the hardware switch priority can be determined
  by either the CPPI channel number (`P0_RX_REMAP_VLAN = 0`) or the packet priority
  value from the VLAN tag (`P0_RX_REMAP_VLAN = 1`).

- The EST schedule is 248us long and it is composed of 4 intervals, each with a
  gate mask that enables transmission of 2 priorities. PTP traffic can be sent on any
  non-zero priority. Here we are using the priority 7 for PTP traffic.
  The gate of this priority is always opened to make sure there is no interruption
  of the PTP packet which keeps the PTP in good synchronization status.
  The remaining priorities are for test traffic (avtp packets).


<table>
<tr><th>Parameters<th>TimeValue
<tr><td> baseTime  <td> `((Current PTP Time + delayOffset)/delayOffset)*delayOffset`
<tr><td> cycleTime <td> 248us
</table>

   **Where**

   + `baseTime`: PTP time round-off to the `delayOffset` to have the same both
     `baseTime` on talker and listener to apply the EST schedule that the same
     time in the future.

   + `delayOffset`: added time to current PTP time for `baseTime` to apply the
     schedule in the future. It should be a multiple of the `cycleTime` and
     large enough so that user can have enough time to start talker and listener
     by entering a character to UART terminal.
     The current `delayOffset` is `100000*cycleTime` = 24800000us (24.8 seconds).
     The factor '100000' is chosen to have `delayOffset` above 20secs.

# Expected Behavior

-  With the schedule configured above, the expectation is that

   + all priority 0 packets arrive on listener inside the time windows
   <table>
   <tr><td> 0--62us <td> 124us--186us
   </table>

   + all priority 2 packets arrive on listener inside the time windows
   <table>
   <tr><td> 62us--124us <td> 186us--248us
   </table>

   + To check whether packets received inside a time window, the rx timestamp,
     called rxts (PTP time) of each received packet is captured by host port on
     the listener's side.
     Then the `timeSlot` of each packet is calculated using the following formula:

     \code
         timeSlot = (rxts - BaseTime)%CycleTime
     \endcode

     The *timeSlot* is compared with the time windows above for each packet
     to check whether the packet received inside or outside the expected time windows.
     The EST works when percentage of packets received outside of expected time
     windows less than or equal 1%.

# Application Integration Guide

The EST application needs to run with the gPTP and yang configuration daemon.
Therefore, the gPTP and uniconf must be run during initializing the application.
To register the EST application to the initializer of the TSN App for running
after the gPTP and uniconf, add the following line to the
*EnetApp_initTsnByCfg* of the tsninit.c

\code{.c}
#ifdef EST_APP_ENABLED
if (res == 0)
{
    res = EnetApp_addEstAppModCtx(gModCtxTable);
}
#endif //EST_APP_ENABLED
\endcode

Then the EST application will be started when the `EnetApp_startTsn` is called.

# Yang Configuration for 802.1Qbv

- Yang file for the 802.1Qbv is defined at
  `standard/ieee/draft/802.1/Qcw/ieee802-dot1q-sched-bridge.yang`
  and `standard/ieee/draft/802.1/Qcw/ieee802-dot1q-sched.yang`
  from the https://github.com/YangModels/yang.git

- Enet-lld supports to configure the `admin-control-list`,
  `baseTime` (`admin-base-time`) and cycleTime (`admin-cycle-time`),
  this section describes the parameters of the `admin-control-list`
  Here are parameters of the `admin-control-list` after converting
  parameters from yang to to xml


\code
      <?xml version='1.0' encoding='UTF-8'?>
      <data xmlns="urn:ietf:params:xml:ns:netconf:base:1.0">
        <interfaces xmlns="urn:ietf:params:xml:ns:yang:ietf-interfaces">
          <interface>
            <name/>
            <bridge-port xmlns="urn:ieee:std:802.1Q:yang:ieee802-dot1q-bridge">
              <traffic-class>
                <traffic-class-table>
                  <number-of-traffic-classes/>
                  <priority0/>
                  ...
                  <priority7/>
                </traffic-class-table>
                <tc-data>
                  <tc/>
                  <lqueue/>
                  ...
                </tc-data>
                <number-of-pqueues/>
                <pqueue-map>
                  <pqueue/>
                  <lqueue/>
                </pqueue-map>
              </traffic-class>
              <gate-parameter-table xmlns="urn:ieee:std:802.1Q:yang:ieee802-dot1q-sched-bridge">
                <gate-enabled></gate-enabled>
                <admin-control-list>
                  <gate-control-entry>
                    <index/>
                    <operation-name/>
                    <time-interval-value/>
                    <gate-states-value/>
                  </gate-control-entry>
                </admin-control-list>
                <admin-cycle-time>
                  <numerator/>
                  <denominator/>
                </admin-cycle-time>
                <admin-base-time>
                  <seconds/>
                  <nanoseconds/>
                </admin-base-time>
              </gate-parameter-table>
            </bridge-port>
          </interface>
        </interfaces>
      </data>

\endcode

 - All the supported parameters for 802.1Qbv are from yang standard except the mechanism for mapping
   traffic class and HW queue are augmented.
   To configure all parameters of 802.1Qbv above, application calls the
   `yang_db_runtime_put_oneline` to write data to DB.
   After all required parameters written to the DB, application trigger the uniconf by writing
   the `/ietf-interfaces/interfaces/interface|name:tilld0/bridge-port/gate-parameter-table/gate-enabled`
   so that the uniconf writes parameters to HW. See `EnetEstApp_setAdminControlList` of the `est_init.c`
   for yang configuration.

   Note
   The network interface `tilld0` is default network interface name, name of network interface
   can be changed by changing default Mac port configured for the example.

# Build Enet TSN EST Example

Refer \ref EXAMPLES_ENET_CPSW_TSN_GPTP to build the enet_cpsw_tsn_est_example

# HW Setup

Make sure you have setup the EVM with cable connections as shown in \ref EVM_SETUP_PAGE.
In addition, follow the steps in the next section.

\note Ethernet cable must be connected and link must be up in order for the example
      application to continue execution.

# Running Enet TSN EST example

- HW devices configuration
  Connect two MCUs directly through the MAC port 1.

\code
    [MCU#1 (talker)]<---------------->[MCU#2 (Listener)]
\endcode

- Launch a CCS debug session, load and run the example executable, see \ref CCS_LAUNCH_PAGE

- The talker or listener can be run on GM or Slave, whichever devices you choose.
  However, we recommend running the talker on the Slave device and listener on the GM.
  The reason for that is the slave device needs to be adjusted PTP time to sync
  with the GM.
  Hence, there is a higher possibility that EST use case will fail due to PTP
  synchronization issue than running the talker on the GM.
  After running both devices, the PTP slave device is the one which shows the
  following debug log on UART

\code

    IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to -14648ppb, GMdiff=-24nsec
    IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to -14666ppb, GMdiff=-30nsec
    IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to -14645ppb, GMdiff=-15nsec

\endcode

- When the abs(GMdiff) is below 1000ns, it is a good time to start the EST usecase.

- Start the listener.
  On the GM device, press `l` to start a listener. It will show log on UART terminal
  as follows

\code

    TimeSlots of PacketPriority: 0:   [0, 62000]ns,  [124000, 186000]ns,
    TimeSlots of PacketPriority: 2:   [62000, 124000]ns,  [186000, 248000]ns,
    The following AdminList param will be configured for EST:
    GateMask[7..0]=oCCCCCCo (0x81), start=0 ns, end=61999 ns, dur=62000 ns
    GateMask[7..0]=oCCCCoCC (0x84), start=62000 ns, end=123999 ns, dur=62000 ns
    GateMask[7..0]=oCCCCCCo (0x81), start=124000 ns, end=185999 ns, dur=62000 ns
    GateMask[7..0]=oCCCCoCC (0x84), start=186000 ns, end=247999 ns, dur=62000 ns
    Base time=74400000000ns,Cycle time=248000ns
    INF:cbase:Successfully configure TAS
    INF:cbase:LLDEnetFilter:destmac:91:E0:F0:00:FE:00, vlanId:110, ethType:0x22f0

\endcode

- Start the talker right after the listener.

  On the slave device, press `t` to start a talker, It will show log on UART
  terminal as follows

\code

    GateMask[7..0]=oCCCCCCo (0x81), start=0 ns, end=61999 ns, dur=62000 ns
    GateMask[7..0]=oCCCCoCC (0x84), start=62000 ns, end=123999 ns, dur=62000 ns
    GateMask[7..0]=oCCCCCCo (0x81), start=124000 ns, end=185999 ns, dur=62000 ns
    GateMask[7..0]=oCCCCoCC (0x84), start=186000 ns, end=247999 ns, dur=62000 ns
    Base time=74400000000ns,Cycle time=248000ns
    INF:cbase:Successfully configure TAS
\endcode

- The test result will have the best accuracy if the *Base time* is the same
  for both talker and listener. To achieve that, we need to start the talker (press *t*)
  right after pressing *l* on the listener.

- After starting the talker, it will start sending traffic after the EST schedule takes
  effect (around 20 seconds).

- Observe test result
  The number of packets received inside expected time windows are called good packets
  and outside the expected time windows are bad packets.
  Press *d* on the UART terminal of the listener to display number of good/bad packets
  and the percentage of the bad packets. The expected result is as follows

\code
    Presss d
    PacketPriority: 0, nGoodPackets: 10731108, nBadPackets: 88858, percentage of bad packets: 0%
    PacketPriority: 2, nGoodPackets: 15960287, nBadPackets: 131973, percentage of bad packets: 0%
    CPU Load: 39%
    Press d
    PacketPriority: 0, nGoodPackets: 10733815, nBadPackets: 88880, percentage of bad packets: 0%
    PacketPriority: 2, nGoodPackets: 15964301, nBadPackets: 132010, percentage of bad packets: 0%
    Press d
    PacketPriority: 0, nGoodPackets: 10736486, nBadPackets: 88901, percentage of bad packets: 0%
    PacketPriority: 2, nGoodPackets: 15968259, nBadPackets: 132056, percentage of bad packets: 0%
\endcode

The expected percentage of bad packets is 0-1%.

# Note

- The gPTP log will help to identify which HW device is a slave device.
  In case that log is not needed or noisy, please suppress them by changing the *gptp:66* to *gptp:55*
  in the enet_tsn/enet_tsn_est_example/makefile.
- If the *percentage of bad packets* is greater than 1%, please check the *GMdiff* from the
  gPTP log of the slave device mentioned above, the expected *GMdiff* less than 1000ns.
  The EST use case only works when PTP synchronization is in good condition.
