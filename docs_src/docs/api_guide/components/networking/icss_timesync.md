# ICSS TimeSync {#ICSS_TIMESYNC}

[TOC]

## Introduction

The ICSS TimeSync driver provide APIs for PTP/1588 v2 receiver implementation on PRU-ICSS.

Precision Time Protocol (PTP) comes in a variety of standards and flavors. Every industry has it's own set of sub rules and configurations which are defined by Annexes and Profiles. For example, the telecom industry uses the telecom profile, while power and substation automation requires the power profile. CIP sync requires the Drive profile.

<!-- HSR/PRP has it's own set of rules for interoperating with PTP/1588 and then there is the plain vanilla 1588 implementation for EMACs and switches. All of this however is based on the basic PTP-1588 standard published by IEEE so there are a lot of similarities to the implementation. -->

Keeping above points in mind, ICSS TimeSync driver provides support for following flavor :
1. End-to-End (E2E) version
    - Annex D based solution for EtherNet/IP
    - Delay Request/Response based Ordinary Clock and Transparent Clock
<!-- 2. Peer-to-Peer (P2P) version
    - Annex F based solution with adaptation for HSR/PRP on IEC 62439-3
    - Peer Delay Request/Response based Ordinary Clock and Transparent Clock -->

<!-- The usage of flavor is firmware specific. Both implementations will not be supported at the same time. For example, EtherNet/IP only has End-to-End (E2E) version of PTP, it will not support Peer-to-Peer (P2P). HSR/PRP implementation supports only Peer-to-Peer (P2P) and the implementation is unique to HSR/PRP. Standard Peer to Peer networks will not interoperate with this implementation. There are some additional differences imposed by the standards which are applicable. -->

Delay Request/Response based Ordinary Clock and Transparent Clock support is provided to support CIP Sync, which requires support for Annex D of PTP/1588 (PTP over UDP).
 <!-- while Peer Delay Request/Response based Ordinary Clock and Transparent Clock is coming from IEC 61588 and IEC 62439-3 (HSR/PRP) which requires support for Annex F of PTP/1588 (PTP over 802.3). So the support is provided as such i.e. E2E mode is implemented to be over UDP (Annex D) while P2P mode is over 802.3 (Annex F). Any profile which requires E2E in Annex F and P2P over Annex D is not supported at this point of time. -->

<!-- PTP for HSR/PRP requires that HSR header and PRP tag be added to Sync, Follow Up and Announce packets. Peer Delay Request and Response frames are considered link local and need not have the headers. In addition to this a PTP implementation need not add or honor redundancy tags in PRP so tags are only applicable with respect to HSR. The standard is still evolving on this one so the driver design is subject to change. This driver design follows Ed.2 of the IEC 62439-3 standard. -->

\note ICSS TimeSync will be supported for use in EtherNet/IP Adapter and PRP(Parallel Redundancy Protocol) examples only.

## Features Supported

- Annex D based PTP/1588 solution for EtherNet/IP
- Single/Dual Step : Capability to handle both single and two step PTP messages
- Single step Ordinary clock
- Syntonization factor calculation
- Simple design for time base (More details in \ref ICSS_TIMESYNC_DESIGN)
- 1PPS output for Sync, with configurable time period for sync generation.
- Statistics for debugging and monitoring

## Features not supported

- Annex F based PTP/1588 solution with adaptation for HSR/PRP on IEC 62439-3
- Single step Transparent clock
- Main clock
- BMCA (Best Time Source Clock Algorithm)
- Annex F based PTP/1588 solution in E2E mode
- Annex D based PTP/1588 solution in P2P mode
- Management Messages

## ICSS TimeSync Design

\subpage ICSS_TIMESYNC_DESIGN explains the driver design in detail.

## Usage


### Including the header file

Include the below file to access the APIs
\snippet Icss_timesync_sample.c icss_timesync_include

### Initializing the Handle

\ref TimeSync_drvInit should be called initialize the driver. It does everything and returns a value indicating whether initialization succeeded or failed. The driver call does not allocate the memory it needs. Therefore the allocation must be done by the application.

\snippet Icss_timesync_sample.c icss_timesync_open

The variables shown in the code above do not show the full list of configurable variables and structures for which memory should be allocated. Please check all members inside \ref TimeSync_ParamsHandle structure.

### Enabling/Disabling the Driver

A separate call is required to \ref TimeSync_drvEnable is needed for enabling PTP processing in firmware. The corresponding \ref TimeSync_drvDisable call disables the module in firmware.

### Updating IP Address

\ref TimeSync_addIP should be called to update IP and modify checksum in PTP packet buffers, whenever IP is assigned or changed.

## PTP Stack Porting Guide

Porting your own PTP/1588 stack is simple because most of the tasks, especially the time critical tasks are performed by the driver. The stack is only responsible for

- Running BMCA and finding out the PTP broadcaster
- Checking for Announce timeouts
- Handling management messages

Once \ref TimeSync_drvInit and \ref TimeSync_drvEnable APIs are called from the application, all PTP announce and management messages are sent on the highest priority queue to the host. All incoming PTP frames are handled through the regular Rx interrupt registered by ICSS-EMAC (See \ref ICSS_EMAC_DESIGN_INTERRUPTS section in \ref ICSS_EMAC_DESIGN for more details). \ref TimeSync_processPTPFrame must be called for processing PTP frames. As mentioned in \ref ICSS_EMAC_DESIGN_DATA_PATH_RX, `rxRTCallBack` needs to be registered to process these frames in queues. Therefore, either \ref TimeSync_processPTPFrame must be registered as `rxRTCallBack`, or if different types of frames (other than PTP) are coming on the high priority queues, then application must differentiate PTP frames in `rxRTCallBack` and then call the \ref TimeSync_processPTPFrame API for PTP frames.

In \ref TimeSync_processPTPFrame, the frame type is checked and if it is an Announce or Management message the data is copied to the buffer `ptpGeneralFrame`. Once the frame has been copied a flag `generalFrameFlag` is set. Another API called \ref TimeSync_getGeneralMessage checks the flag and copies the data from the buffer `ptpGeneralFrame` to another buffer buff which belongs to the PTP stack. The API \ref TimeSync_getGeneralMessage strips the header from the frame based on whether the mode is E2E or P2P and only copies the relevant data to the stack buffer. This way only the relevant data is sent to stack and stack need not worry whether the mode is E2E or P2P.

The stack can either modify \ref TimeSync_processPTPFrame to call the \ref TimeSync_getGeneralMessage API, or call \ref TimeSync_getGeneralMessage in a loop in a task.

### Running Best Main Clock Algorithm

PRU-ICSS firmware compares the Source MAC ID of Sync and Follow Up frames with the value stored at the offset `PTP_MASTER_SRC_MAC_ID` in PRU-ICSS Shared RAM, and if it does not match then the sync processing does not happen. For starting sync processing, BMCA has to complete and once the source MAC ID of a broadcaster is available, it should be shared with firmware by calling \ref TimeSync_updateParentAddress.

### Getting the current time

As explained in \ref ICSS_TIMESYNC_DESIGN_TIMESTAMPING section of \ref ICSS_TIMESYNC_DESIGN page, ICSS TimeSync has a very simple design where the IEP counter acts as the nanoseconds counter. \ref TimeSync_getCurrentTime returns the current time on device.

### Resetting the driver

In case of a timeout, link break, etc. the driver requires a reset. This is done using the \ref TimeSync_reset. This API also calls the \ref TimeSync_stackResetCallback_t which can be registered by stack.

### Getting other parameters

Most of the information related to PTP is encapsulated in Announce and Management messages. However certain information is part of the driver, and the stack may need access to these for its operation. Relevant information can be read from the runtime structure \ref timeSync_RuntimeVar_t based on the requirement.

## Debug Guide

Internal structures used by ICSS TimeSync Driver can be checked for debugging. Most relevant structures are \ref timeSync_RuntimeVar_t are \ref timeSync_SyntInfo_t.

## API

\ref NETWORKING_ICSS_TIMESYNC_MODULE

## See also
\ref ICSS_EMAC
