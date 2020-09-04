# ICSS-EMAC Debug Guide {#ICSS_EMAC_DEBUG_GUIDE}

[TOC]

It is highly recommended to go through \ref ICSS_EMAC_DESIGN page before proceeding with this page.

## Assumption

- A basic understanding of ICSS Architecture and SoC
- Some level of familiarity with Ethernet driver development
- In addition to this a basic familiarity with the following tools
    - Code Composer Studio
    - Any serial terminal (Putty/Tera Term/Terminal integrated with CCS)
    - Wireshark or any other packet sniffing tool
    - Any PC based tool capable of sending packets

## Scope

This page is meant to help the developer in
- Familiarizing with common debugging tools and techniques
- Identifying and resolving frequently faced issues

## Common Debugging Tasks

### Loading and running on CCS

See \ref CCS_LAUNCH_PAGE page for details on how to load and run binaries on EVM using CCS. Also, see \ref CCS_UART_TERMINAL section for information on how to open a UART terminal inside CCS.

### Checking Link Status {#ICSS_EMAC_DEBUG_GUIDE_CHECKING_LINK_STATUS}

Link Status is available as an array `linkStatus` in `ICSS_EMAC_Object`. The status is updated in the Link ISR function (`ICSS_EMAC_linkISR`). It can be read by putting `"((ICSS_EMAC_Object *)(icssEmacHandle->object))->linkStatus"` in the "Expressions" window of CCS (It can be opened in CCS from View->Expressions).

It is possible that the link interrupt is configured incorrectly or MDIO is not triggering the interrupts. In which case one needs to put a breakpoint in the ISR `ICSS_EMAC_linkISR` and disconnect or connect the cable on any one port. The breakpoint should get hit, if it does not then there is some issue with the interrupt configuration.

In case different PHYs (compared to those on EVM) are used that, the PHY status registers may be read incorrectly and that might lead to some issues. Please consult the porting guide and reference manual for the specific PHY to figure out where the issue could be.

### Checking if Receive is working

The receive path has been explained in \ref ICSS_EMAC_DESIGN_DATA_PATH_RX section. Rx issues can manifest themselves in several ways, the following table covers most of them (not exhaustively).

<table>
<tr>
    <th>Issue
    <th>Probable Cause
</tr>
<tr>
    <td> Host not receiving Multicast/Broadcast frames
    <td>
         - Rx interrupt not configured correctly
         - Firmware not receiving
         - Storm Prevention enabled
         - Rx is disabled
</tr>
<tr>
    <td> Unicast Packets (for Host) being dropped
    <td>
         - Rx interrupt not configured correctly
         - Firmware not receiving
         - Storm Prevention enabled
         - Rx is disabled
         - Interface MAC not configured correctly
</tr>
<tr>
    <td> Multicast/Broadcast frames not being forwarded
    <td>
         - Storm Prevention enabled
         - Firmware not receiving
         - Rx is disabled
</tr>
<tr>
    <td> Unicast frames (not for Host) not being forwarded
    <td>
         - Firmware not receiving
         - Rx is disabled
</tr>
<tr>
    <td> Packets are getting dropped
    <td>
         - Storm Prevention enabled
         - Data is coming in too fast. See \ref ICSS_EMAC_DESIGN_INTERRUPT_PACING section to understand this behavior
</tr>
</table>

The first step is to identify the exact problem. To do that please perform these steps in order.

1. Check if Rx is disabled : It is possible to disable Rx in firmware through IOCTL. This is controlled through a location in ICSS memory. Check the memory correspnding to `portControlAddr` configured in \ref ICSS_EMAC_FwStaticMmap. See \ref ICSS_EMAC_DEBUG_GUIDE_ACCESSING_MEMORY for details how to check this memory. `0x1` value for this byte means Rx is disabled , and `0x0` means Rx is enabled. Check this as a very first step. It is not a common error but it is possible that user is invoking the IOCTL for disabling by mistake.

2. Check if firmware is receiving packets : See \ref ICSS_EMAC_DEBUG_GUIDE_CHECKING_STATISTICS section to find out if PRU is receiving the frames. Failure to receive frames in firmware can indicate other issues like corrupted frames, link negotiation failure or PHY related issues.

3. Check if Rx interrupt is being asserted : This can be one of the reasons why Host would not receive packets. Put a break point in `ICSS_EMAC_rxInterruptHandler` and send a single packet using any PC based tool. The ISR should get hit. Please check if the interrupt numbers for being used for ARM are correct and as expected based on the firmware.

4. Check if Packets are being copied in driver : If the interrupt is asserted but packets do not reach the application, check Host statistics (as shown in \ref ICSS_EMAC_DEBUG_GUIDE_CHECKING_STATISTICS) to verify if packets are being received correctly in the driver. Refer to the \ref ICSS_EMAC_DESIGN_DATA_PATH_RX and put a breakpoint in `ICSS_EMAC_osRxTaskFnc` used for creating `RxTask` task. Step through to verify that the priority is set correctly and packets are being copied properly. If interrupt is being asserted correctly but packet length is zero it indicates some data corruption in the receive queues or firmware behaving incorrectly, this scenario should not occur.

5. Check if Storm prevention is enabled : This is one of the most common reasons why throughput may get lowered or if the threshold is set incorrectly packets may not reach Host at all. Check if storm prevention module is enabled by checking the `"((ICSS_EMAC_Object *)(icssEmacHandle->object))->stormPrev"` for both ports in the "Expressions" window of CCS. Additionally one should check the statistics to see if `stormPrevCounter`, `stormPrevCounterMC` or `stormPrevCounter` in \ref ICSS_EMAC_PruStatistics is getting incremented (See \ref ICSS_EMAC_DEBUG_GUIDE_CHECKING_STATISTICS).

6. Check Interface MAC : The firmware compares the interface MAC written to the PRU memory by the Host against the incoming packets destination MAC to verify if the packet must be forwarded to Host or cut-through. Please check the memory correspnding to `interfaceMacAddrOffset` configured in \ref ICSS_EMAC_FwStaticMmap and verify if the MAC value is what you are expecting it to be. See \ref ICSS_EMAC_DEBUG_GUIDE_ACCESSING_MEMORY for details how to check this memory.

7. Queue Overflow : If too many packets are received on a single queue without Host emptying them out then overflow may occur, packets will be lost in such a scenario. This is somewhat related to throughput issues but may occur independently as well.

8. Check throughput : This is applicable in case everything else appears to be correct but the number of packets reaching the Host is not 100% of the transmitted value. This can happen because of two reasons:
    1. Data rate is too fast and interrupt pacing is disabled.
    2. Processing on Host is too slow and driver cannot cope with the rate at which firmware is putting data in the queue. In such a scenario first turn on Interrupt pacing to find if it solves the issue (interrupt pacing has its own limitations, refer to \ref ICSS_EMAC_DESIGN_INTERRUPT_PACING to know more about it). If issue is still not resolved then try to find the throughput by comparing the number of packets received on the Host vs that in the firmware.

9. More on Throughput : If throughput is low then try to find out how the Rx processing on Host can be sped up or if any other high priority task is blocking the execution of `RxTask`.

### Checking if Transmit is working

Tx is much more reliable and there are far fewer issues related to it when compared to Rx. The transmit path has been explained in \ref ICSS_EMAC_DESIGN_DATA_PATH_TX section.

Transmit issues can be classified into two types:

1. Cut-through issues : Packets received on one port and meant to go out of the opposite port. (Not applicable to EMAC)
2. Transmit from Host : Packets sent out from the Host on any one port.

As a first step please check the statistics (as shown in \ref ICSS_EMAC_DEBUG_GUIDE_CHECKING_STATISTICS) on firmware as well as Host to see if any packets have been sent out. Try to trace where the issue is by comparing transmit statistics for Host and firmware.

The probable causes for transmit not working are listed below. This can also be used as a checklist for debugging.

1. Link down : The link event is mapped to an ISR `ICSS_EMAC_linkISR` which in turn triggers call to another API `ICSS_EMAC_updatePhyStatus` to update the link status in firmware. If this is not done correctly then it is possible that firmware will read the event as link down even though physically the link is up. Please check the link status to make sure that this is not the case. The Tx API checks for link and will not transmit if the link is down so this issue is more relevant to cut-through/store-forward. Please see \ref ICSS_EMAC_DEBUG_GUIDE_CHECKING_LINK_STATUS for details on how to check link status.

2. Incorrect speed : The link ISR also checks for speed and duplexity values. The values are written to in the `ICSS_EMAC_updatePhyStatus` function, if the speed is read incorrectly then it is possible that firmware will not send out packets or may send out garbage. In such a scenario firmware statistics will count the packets as successful transmit but Tx might not actually happen. The quickest way to debug this issue is to read the ICSS memory directly in CCS memory window. Interface speed is written directly in memory at the offset mentioned `phySpeedOffset` configured in \ref ICSS_EMAC_FwStaticMmap. Please check the value and compare with the actual interface speed. See \ref ICSS_EMAC_DEBUG_GUIDE_ACCESSING_MEMORY for details how to check this memory.

3. Incorrect pinmux for Collision/Carrier Sense : This is applicable in case of developers using their custom boards, it is important that the collision and carrier sense signals be wired correctly because the transmit code relies on these two signals to implement half duplex functionality and wrong values may result in transmit problems. If there is an issue with pinmuxing for these two pins, it is recommended that half duplex functionality be disabled. Half Duplex functionality can be controlled in the SysConfig.

4. Queue contention issues : Looking at the \ref ICSS_EMAC_DESIGN_QOS scheme, it is possible that there is a contention for the transmit queue when both Host and the opposite port are trying to transmit on the same port. In such cases if there are too many packets vying for the contention queue they will be dropped. Such conditions are rare.

5. Queue overflow : As the name suggests if too many packets are sent out on a single queue then overflow can happen and packets may get lost.

### Checking Statistics {#ICSS_EMAC_DEBUG_GUIDE_CHECKING_STATISTICS}

Statistics form the core of debugging so this section is very important. A brief introduction to statistics has been provided in \ref ICSS_EMAC_DESIGN_STATISTICS section. This part explains how to use it for the purpose of debugging.

As previously explained, statistics can be divided into two groups
- Statistics on PRU
- Statistics on Host

Host statistics are a subset of firmware based statistics except some specialized statistics like `rxUnknownProtocol` and `linkBreak`. This property can be used to find out how many packets are being received in the firmware and how many are reaching Host.

Following are two ways of reading these statistics:

- IOCTL : \ref ICSS_EMAC_ioctl API should be called with \ref ICSS_EMAC_IOCTL_STATS_CTRL as `ioctlCommand`, \ref ICSS_EMAC_IOCTL_STAT_CTRL_GET as `ioctlParams.command` and a pointer to \ref ICSS_EMAC_PruStatistics as `ioctlParams.ioctlVal` to access the PRU statistics.

- CCS : When using CCS the statistics can be read directly through the ICSS EMAC handle. Host statistics are available through `"((ICSS_EMAC_Object *)(icssEmacHandle->object))->hostStat"` , while PRU statistics are available through `"(ICSS_EMAC_PruStatistics *)((((*(((ICSS_EMAC_Object *)(icssEmacHandle->object)))).fwStaticMMap).statisticsOffset + (*((*((*(((ICSS_EMAC_Object *)(icssEmacHandle->object)))).pruicssHandle)).hwAttrs)).pru0DramBase))"` in the "Expressions" window of CCS.

## Accessing Memory {#ICSS_EMAC_DEBUG_GUIDE_ACCESSING_MEMORY}

This section explains how to access ICSS memory. Various firmware offsets are configured using \ref ICSS_EMAC_FwStaticMmap, \ref ICSS_EMAC_FwDynamicMmap, \ref ICSS_EMAC_FwVlanFilterParams and \ref ICSS_EMAC_FwMulticastFilterParams structures, which are a part of \ref ICSS_EMAC_Params needed for \ref ICSS_EMAC_open API call.

The Shared RAM address and DRAM base addresses for Port 1 and Port 2 can be obtained by adding `"(*((*((*(((ICSS_EMAC_Object *)(icssEmacHandle->object)))).pruicssHandle)).hwAttrs))"` in the "Expressions" window of CCS.

For example on AM64x, if `portMacAddr` location needs to be accessed for Port 0, then following steps give the value of the location using the "Memory Browser" of CCS.

<table style="border: 0 px; margin-left: auto; margin-right: auto">
    <tr>
        <td> \image html  ICSS_EMAC_Debug_Memory_Access_1.PNG
        <td> \image html  ICSS_EMAC_Debug_Memory_Access_2.PNG
        <td> \image html  ICSS_EMAC_Debug_Memory_Access_3.PNG
    </tr>
</table>

## Using ROV to Debug RTOS

For more details, see \ref ROV_INTRO_PAGE.