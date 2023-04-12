/**
 * @file  csl/src/ip/cpsw/V5/csl_cpsw.h
 *
 * @brief
 *  API Auxilary header file for Ethernet switch module CSL.
 *
 *  Contains the different control command and status query functions definations
 *
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef CSL_CPSW_V5_H_
#define CSL_CPSW_V5_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <drivers/hw_include/cslr_soc.h>

#include <cslr_ale.h>
#include <cslr_xge_cpsw.h>
#include <enet_cfg.h>

/** @addtogroup CSL_CPSWITCH_DATASTRUCT
 @{ */

/** @brief ALE control register configuration definitions */

/**  Enable Broadcast/Multicast rate limit */
#define    CSL_CPSW_ALECONTROL_RATELIMIT_EN      (1 << 0u)

/**  MAC auhorization mode enable */
#define    CSL_CPSW_ALECONTROL_AUTHMODE_EN       (1 << 1u)

/**  VLAN Aware Mode enable */
#define    CSL_CPSW_ALECONTROL_VLANAWARE_EN      (1 << 2u)

/**  Tx rate limit enable */
#define    CSL_CPSW_ALECONTROL_RATELIMIT_TX_EN   (1 << 3u)

/**  OUI deny enable */
#define    CSL_CPSW_ALECONTROL_OUIDENY_EN        (1 << 5u)

/**  VID0 mode enable */
#define    CSL_CPSW_ALECONTROL_VID0MODE_EN       (1 << 6u)

/**  Learn no VID enable */
#define    CSL_CPSW_ALECONTROL_LEARN_NO_VID_EN   (1 << 7u)

/**  Age out now enable */
#define    CSL_CPSW_ALECONTROL_AGEOUT_NOW_EN     (1 << 29u)

/**  Clear table enable */
#define    CSL_CPSW_ALECONTROL_CLRTABLE_EN       (1 << 30u)

/**  ALE enable */
#define    CSL_CPSW_ALECONTROL_ALE_EN            (1 << 31u)

/** @brief Port Mask definitions */

/**  Port 0 Enable */
#define    CSL_CPSW_PORTMASK_PORT0_EN            (1 << 0u)

/**  Port 1 Enable */
#define    CSL_CPSW_PORTMASK_PORT1_EN            (1 << 1u)

/**  Port 2 Enable */
#define    CSL_CPSW_PORTMASK_PORT2_EN            (1 << 2u)

/**  Port 3 Enable */
#define    CSL_CPSW_PORTMASK_PORT3_EN            (1 << 3u)

/**  Port 4 Enable */
#define    CSL_CPSW_PORTMASK_PORT4_EN            (1 << 4u)

/**  Port 5 Enable */
#define    CSL_CPSW_PORTMASK_PORT5_EN            (1 << 5u)

/**  Port 6 Enable */
#define    CSL_CPSW_PORTMASK_PORT6_EN            (1 << 6u)

/**  Port 7 Enable */
#define    CSL_CPSW_PORTMASK_PORT7_EN            (1 << 7u)

/**  Port 8 Enable */
#define    CSL_CPSW_PORTMASK_PORT8_EN            (1 << 8u)

/** @brief
 *
 *  Holds the Time sync submodule's version info.
 */
typedef struct {
    /**  Minor version value */
    Uint32      minorVer;

    /**  Major version value */
    Uint32      majorVer;

    /**  RTL version value */
    Uint32      rtlVer;

    /**  Identification value */
    Uint32      id;
} CSL_CPSW_VERSION;

/** @brief
 *
 *  Holds CPSW control register contents.
 */
typedef struct {
    /**  FIFO loopback mode */
    Uint32      fifoLb;

    /**  Vlan aware mode */
    Uint32      vlanAware;

    /** Port 0 Enable */
    Uint32      p0Enable;

    /**  Port 0 Pass Priority Tagged */
    Uint32      p0PassPriTag;

    /**  Port 1 Pass Priority Tagged */
    Uint32      p1PassPriTag;

    /**  Port 2 Pass Priority Tagged */
    Uint32      p2PassPriTag;

    /**  Port 3 Pass Priority Tagged */
    Uint32      p3PassPriTag;

    /**  Port 4 Pass Priority Tagged */
    Uint32      p4PassPriTag;

    /**  Port 5 Pass Priority Tagged */
    Uint32      p5PassPriTag;

    /**  Port 6 Pass Priority Tagged */
    Uint32      p6PassPriTag;

    /**  Port 7 Pass Priority Tagged */
    Uint32      p7PassPriTag;

    /**  Port 8 Pass Priority Tagged */
    Uint32      p8PassPriTag;

    /**  Port 0 Transmit CRC remove */
    Uint32      p0TxCrcRemove;

    /**  Port 0 Receive Short Packet Pad
         0 - short packets are dropped
         1 - short packets are padded to 64-bytes (with pad and added CRC)
             if the CRC is not passed in.  Short packets are dropped if the CRC is
             passed (in the Info0 word).
      */
    Uint32      p0RxPad;

    /**  Port 0 Pass Received CRC errors */
    Uint32      p0RxPassCrcErr;

    /**  Energy Efficient Ethernet enable */
    Uint32      eeeEnable;

    /**  Enhanced Scheduled Traffic enable (EST) */
    Uint32      estEnable;

    /**  Intersperced Express Traffic enable (IET) */
    Uint32      ietEnable;

    /** Cut Thru Switching Enable */
    Uint32      cutThruEnable;
} CSL_CPSW_CONTROL;

/** @brief  CPSW_THRU_RATE register
 *
 *  Holds CPSW_THRU_RATE register contents.
 */
typedef struct {
    /** Ethernet Port Switch FIFO receive through rate.
     *  This register value is the maximum throughput of the Ethernet ports
     *  to the crossbar SCR.  The default is one 8-byte word for every
     *  3 VBUSP_GCLK periods maximum.
     *  The minimum value is 2.
     *  This is not a field that is intended to be changed by a user
     */
    Uint32      enetRxThruRate;
    /** CPPI FIFO (port 0) receive through rate.
     * This register value is the maximum throughput of the CPPI FIFO (port 0)
     * into the CPSW_NU.
     * The minimum value is 1.
     * This field is not intended to be changed by the user
     */
    Uint32      cppiRxThruRate;
} CSL_CPSW_THRURATE;

/** @brief
 *
 *  Holds CPPI P0 Control register contents.
 */
typedef struct {
    /** Port 0 receive remap thread to DSCP IPV6 priority  */
    Uint32      p0RxRemapDscpIpv6;

    /** Port 0 receive remap thread to DSCP IPV6 priority.  */
    Uint32      p0RxRemapDscpIpv4;

    /** Port 0 receive remap thread to VLAN.  */
    Uint32      p0RxRemapVlan;

    /** Port 0 receive ECC Error Enable
      * This bit must be set to enable receive ECC error operations
      */
    Uint32      p0RxEccErrEn;

    /** Port 0 transmit ECC Error Enable
      * This bit must be set to enable transmit ECC error operations
      */
    Uint32     p0TxEccErrEn;

    /** Port 0 IPv6 DSCP enable
     *  0 - Ipv6 DSCP priority mapping is disabled
     *  1 - Ipv6 DSCP priority mapping is enabled
     */
    Uint32      p0DscpIpv6En;

    /** Port 0 IPv4 DSCP enable
      * 0 - Ipv4 DSCP priority mapping is disabled
      * 1 - Ipv4 DSCP priority mapping is enabled
      */
    Uint32      p0DscpIpv4En;

    /** Port 0 Receive (port 0 ingress) Checksum Enable
      *  0 - Port 0 receive checksum is disabled
      *  1 - Port 0 receive checksum is enabled
      */
    Uint32      p0RxChksumEn;

    /** Port 0 Transmit (port 0 egress) Checksum Enable
      *  0 - Port 0 receive checksum is disabled
      *  1 - Port 0 receive checksum is enabled
      */
    Uint32      p0TxChksumEn;

} CSL_CPSW_CPPI_P0_CONTROL;

/** @brief
 *
 *  Holds CPPI_P0_Rx_Gap register contents.
 *  This is applicable only for 2 port switch
 */
typedef struct
{
    /** Receive Gap Count - This is the number of clocks that will in the
     *  gap between received packet on port 0 when a priority has
     *  rxGapEnPri[7-0] set
     */
    Uint32 rxGapCnt;

    /** Receive Gap Enable for Priority 7 */
    Uint32 rxGapEnPri7;

    /** Receive Gap Enable for Priority 6 */
    Uint32 rxGapEnPri6;

    /** Receive Gap Enable for Priority 5 */
    Uint32 rxGapEnPri5;

    /** Receive Gap Enable for Priority 4 */
    Uint32 rxGapEnPri4;

    /** Receive Gap Enable for Priority 3 */
    Uint32 rxGapEnPri3;

    /** Receive Gap Enable for Priority 2 */
    Uint32 rxGapEnPri2;

    /** Receive Gap Enable for Priority 1 */
    Uint32 rxGapEnPri1;

    /** Receive Gap Enable for Priority 0 */
    Uint32 rxGapEnPri0;

} CSL_CPSW_CPPI_P0_RXGAP;

/** @brief
 *
 *  Holds CPPI_P0_FIFO_Status  register contents.
 *  This is not applicable for 2 port switch
 */
typedef struct
{
    /** Port 0 Transmit FIFO Priority Active for priority 0
      * Indicates whether the corresponding FIFO priority has one or more
      * queued packets on it or not.
      * note: for N=2 this field is always zero (there is no transmit FIFO).
      */
    Uint32 p0TxPriActivePri0;

    /** Port 0 Transmit FIFO Priority Active for priority 1
      * Indicates whether the corresponding FIFO priority has one or more
      * queued packets on it or not.
      * note: for N=2 this field is always zero (there is no transmit FIFO).
      */
    Uint32 p0TxPriActivePri1;

    /** Port 0 Transmit FIFO Priority Active for priority 2
      * Indicates whether the corresponding FIFO priority has one or more
      * queued packets on it or not.
      * note: for N=2 this field is always zero (there is no transmit FIFO).
      */
    Uint32 p0TxPriActivePri2;

    /** Port 0 Transmit FIFO Priority Active for priority 3
      * Indicates whether the corresponding FIFO priority has one or more
      * queued packets on it or not.
      * note: for N=2 this field is always zero (there is no transmit FIFO).
      */
    Uint32 p0TxPriActivePri3;

    /** Port 0 Transmit FIFO Priority Active for priority 4
      * Indicates whether the corresponding FIFO priority has one or more
      * queued packets on it or not.
      * note: for N=2 this field is always zero (there is no transmit FIFO).
      */
    Uint32 p0TxPriActivePri4;

    /** Port 0 Transmit FIFO Priority Active for priority 5
      * Indicates whether the corresponding FIFO priority has one or more
      * queued packets on it or not.
      * note: for N=2 this field is always zero (there is no transmit FIFO).
      */
    Uint32 p0TxPriActivePri5;

    /** Port 0 Transmit FIFO Priority Active for priority 6
      * Indicates whether the corresponding FIFO priority has one or more
      * queued packets on it or not.
      * note: for N=2 this field is always zero (there is no transmit FIFO).
      */
    Uint32 p0TxPriActivePri6;

    /** Port 0 Transmit FIFO Priority Active for priority 7
      * Indicates whether the corresponding FIFO priority has one or more
      * queued packets on it or not.
      * note: for N=2 this field is always zero (there is no transmit FIFO).
      */
    Uint32 p0TxPriActivePri7;

} CSL_CPSW_CPPI_P0_FIFOSTATUS;

/** @brief
 *
 *  Holds CSL_CPSW_CPPI_P0_HOSTBLKSPRI  register contents.
 *  This is not used for 2 port switch
 */
typedef struct
{
    /** Host Blocks Per Priority 7
      */
    Uint32 p0HostBlksPri7;

    /** Host Blocks Per Priority 6
      */
    Uint32 p0HostBlksPri6;

    /** Host Blocks Per Priority 5
      */
    Uint32 p0HostBlksPri5;

    /** Host Blocks Per Priority 4
      */
    Uint32 p0HostBlksPri4;

    /** Host Blocks Per Priority 3
      */
    Uint32 p0HostBlksPri3;

    /** Host Blocks Per Priority 2
      */
    Uint32 p0HostBlksPri2;

    /** Host Blocks Per Priority 1
      */
    Uint32 p0HostBlksPri1;

    /** Host Blocks Per Priority 0
      */
    Uint32 p0HostBlksPri0;

} CSL_CPSW_CPPI_P0_HOSTBLKSPRI;

/** @brief
 *
 *  Holds flow control register contents.
 */
typedef struct {
    /**  Port 0 flow control enable */
    Uint32      p0FlowEnable;

    /**  Port 1 flow control enable */
    Uint32      p1FlowEnable;

    /**  Port 2 flow control enable */
    Uint32      p2FlowEnable;

    /**  Port 3 flow control enable */
    Uint32      p3FlowEnable;

    /**  Port 4 flow control enable */
    Uint32      p4FlowEnable;

    /**  Port 5 flow control enable */
    Uint32      p5FlowEnable;

    /**  Port 6 flow control enable */
    Uint32      p6FlowEnable;

    /**  Port 7 flow control enable */
    Uint32      p7FlowEnable;

    /**  Port 8 flow control enable */
    Uint32      p8FlowEnable;

} CSL_CPSW_FLOWCNTL;

/** @brief
 *
 *  Holds the ALE submodule's version info.
 */
typedef struct {
    /**  Minor version value */
    Uint32      minorVer;

    /**  Major version value */
    Uint32      majorVer;

    /**  RTL version value */
    Uint32      rtlVer;

    /**  Identification value */
    Uint32      id;
} CSL_CPSW_ALE_VERSION;

/** @brief
 *
 *  Defines ALE table types support
 */
typedef enum {
    CSL_CPSW_ALETABLE_TYPE_4PORT,
    CSL_CPSW_ALETABLE_TYPE_9PORT,
} CSL_CPSW_ALETABLE_TYPE;

/** @brief
 *
 *  Defines ALE port states
 */
typedef enum {
    CSL_ALE_PORTSTATE_DISABLED = 0,
    CSL_ALE_PORTSTATE_BLOCKED,
    CSL_ALE_PORTSTATE_LEARN,
    CSL_ALE_PORTSTATE_FORWARD
} CSL_CPSW_ALE_PORTSTATE;

/** @brief
 *
 *  Holds the ALE Port control register info.
 */
typedef struct {
    /**  Port state */
    CSL_CPSW_ALE_PORTSTATE  portState;

    /**  Drop non-VLAN tagged ingress packets?  */
    Uint32                  dropUntaggedEnable;

    /**  VLAN ID Ingress check enable */
    Uint32                  vidIngressCheckEnable;

    /**  No learn mode enable */
    Uint32                  noLearnModeEnable;

    /**  No Source Address Update enable */
    Uint32                  noSaUpdateEnable;

    /** MAC only mode enable:
      * When set allows the port to be treated like
      * a Mac port for the host. All traffic received
      * is sent only to the host. The host must direct
      * traffic to this port as the lookup engine will
      * not send traffic to ports with macOnlyEnable
      * and noLearnModeEnable is set.
      * If macOnlyEnable is set and noLearnModeEnable
      * is not set, the host can send non-directed packets
      * to a lookup destination with macOnlyEnable set.
      * It is also possible that the host can broadcast
      * to all ports including Mac Only ports in this mode.
      */
    Uint32                  macOnlyEnable;

    /** Disable MAC Authorization Mode for this port
      * @note: This field is only valid when CPSW
      *        MAC authentication is enabled.
      */
    Uint32                  macAuthDisable;

    /** Mac Only Copy All Frames:
      * Set: A Mac Only port will transfer all received
      *       good frames to the host.
      * Clear: A Mac Only port will transfer packets to
      *        the host based on ALE destination address
      *        lookup operation.
      */
    Uint32                  macOnlyCafEnable;

    /**  Multicast packet rate limit */
    Uint32                  mcastLimit;

    /**  Broadcast packet rate limit */
    Uint32                  bcastLimit;
    /** Drop Dual VLAN - When set will cause any received
     *  packet with dual VLAN stag followed by ctag to be
     *  dropped     */
    Uint32                  dropDualVlan;
    /** Drop Double VLAN - When set cause any received
     *  packet with double VLANs to be dropped. That is if
     *  there are two ctag or two stag fields in the packet it will
     *  be dropped  */
    Uint32                  dropDoubleVlan;

} CSL_CPSW_ALE_PORTCONTROL;

/** @brief
 *
 *  Defines ALE Table Entry types
 */
typedef enum {
    CSL_ALE_ENTRYTYPE_FREE = 0,
    CSL_ALE_ENTRYTYPE_ADDRESS,
    CSL_ALE_ENTRYTYPE_VLAN,
    CSL_ALE_ENTRYTYPE_VLANADDRESS
} CSL_CPSW_ALE_ENTRYTYPE;

/** @brief
 *
 *  ALE Table entry type: MAC ADDRESS
 */
#define  CSL_ALE_ENTRYTYPE_MAC_ADDR     CSL_ALE_ENTRYTYPE_ADDRESS

/** @brief
 *
 *  ALE Table entry type: POLICER ENTRY
 */
#define  CSL_ALE_ENTRYTYPE_POLICER      CSL_ALE_ENTRYTYPE_VLAN

/** @brief
 *
 *  Defines ALE Unicast types
 */
typedef enum {
    CSL_ALE_UCASTTYPE_UCAST_NOAGE = 0,
    CSL_ALE_UCASTTYPE_UCAST_AGENOTOUCH,
    CSL_ALE_UCASTTYPE_UCAST_OUI,
    CSL_ALE_UCASTTYPE_UCAST_AGETOUCH
} CSL_CPSW_ALE_UCASTTYPE;

/** @brief
 *
 *  Defines ALE Address types
 */
typedef enum {
    CSL_ALE_ADDRTYPE_UCAST = 0,
    CSL_ALE_ADDRTYPE_MCAST,
    CSL_ALE_ADDRTYPE_OUI
} CSL_CPSW_ALE_ADDRTYPE;


/** @brief
 *
 *  Defines ALE Policer Entry  types
 */
typedef enum {
    CSL_ALE_POLICER_ENTRYTYPE_VLAN = 0,     /** VLAN or Inner VLAN */
    CSL_ALE_POLICER_ENTRYTYPE_OVLAN,        /** Outer VLAN */
    CSL_ALE_POLICER_ENTRYTYPE_ETHERTYPE,    /** Ethertype */
    CSL_ALE_POLICER_ENTRYTYPE_IPV4,         /** IPv4 address */
    CSL_ALE_POLICER_ENTRYTYPE_IPV6          /** IPv6 address */
} CSL_CPSW_ALE_POLICER_ENTRYTYPE;

/** @brief
 *
 *  Holds the ALE Multicast Address Table entry
 *  configuration.
 */
typedef struct {
    /**  Multicast address */
    Uint8                       macAddress [6];

    /**  Multicast forward state  */
    Uint32                      mcastFwdState;

    /**  Supervisory bit enable? */
    Uint32                      superEnable;

    /**  Port Mask. */
    Uint32                      portMask;

    /** Ignore bits in multicast address */
    Uint32                      ignMBits;
} CSL_CPSW_ALE_MCASTADDR_ENTRY;

/** @brief
 *
 *  Holds the ALE VLAN/Multicast Address Table entry
 *  configuration.
 */
typedef struct {
    /**  Multicast address */
    Uint8                       macAddress [6];

    /**  VLAN Id  */
    Uint32                      vlanId;

    /**  Multicast forward state  */
    Uint32                      mcastFwdState;

    /**  Supervisory bit enable? */
    Uint32                      superEnable;

    /**  Port Mask. */
    Uint32                      portMask;

    /**  Enable multicast address mask */
    Uint32                      ignMBits;
} CSL_CPSW_ALE_VLANMCASTADDR_ENTRY;

/** @brief
 *
 *  Holds the ALE Unicast Address Table entry
 *  configuration.
 */
typedef struct {
    /**  Unicast address */
    Uint8                       macAddress [6];

    /**  Entry ageable or not  */
    Uint32                      ageable;

    /**  Entry touched or not  */
    Uint32                      touched;


    /**  Secure bit enable?  */
    Uint32                      secureEnable;

    /**  Block bit enable? */
    Uint32                      blockEnable;

    /**  Port Number to forward matching packets to. */
    Uint32                      portNumber;

    /**  Trunk Indicator
      *  0: the port bits in the entry are the port number
      *  1: the port bits in the entry are the trunk  number*/
    Uint32                      trunkFlag;

} CSL_CPSW_ALE_UNICASTADDR_ENTRY;

/** @brief
 *
 *  Holds the ALE OUI Unicast Address Table entry
 *  configuration.
 */
typedef struct {
    /**  OUI Unicast address */
    Uint8                       ouiAddress [3];

    /**  Entry ageable or not  */
    Uint32                      ageable;

    /**  Entry touched or not  */
    Uint32                      touched;
} CSL_CPSW_ALE_OUIADDR_ENTRY;

/** @brief
 *
 *  Holds the ALE VLAN Unicast Address Table entry
 *  configuration.
 */
typedef struct {
    /**  Unicast address */
    Uint8                       macAddress [6];

    /**  VLAN Id  */
    Uint32                      vlanId;

    /**  Entry ageable or not  */
    Uint32                      ageable;

    /**  Entry touched or not  */
    Uint32                      touched;

    /**  Secure bit enable?  */
    Uint32                      secureEnable;

    /**  Block bit enable? */
    Uint32                      blockEnable;

    /**  Port Number to forward matching packets to. */
    Uint32                      portNumber;
    /** Flag indicating if port is part of trunk group */
    Uint32                      trunkFlag;
} CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY;

/** @brief
 *
 *  Holds the ALE (Inner) VLAN Table entry configuration.
 */
typedef struct {
    /**  VLAN Id  */
    Uint32                      vlanId;

    /**  VLAN member list */
    Uint32                      vlanMemList;

    /**  Unregistered Multicast Flood mask  */
    Uint32                      unRegMcastFloodIndex;

    /**  Registered Multicast Flood mask  */
    Uint32                      regMcastFloodIndex;
    /**  Unregistered Multicast Flood mask  */
    Uint32                      unRegMcastFloodMask;

    /**  Registered Multicast Flood mask  */
    Uint32                      regMcastFloodMask;
    /**  Force Untagged Packet Egress. */
    Uint32                      forceUntaggedEgress;

    /**  enable check for vlan member list at ingress. */
    Uint32                      ingressCheckFlag;

    /**  Port Mask to disable learning on specified port */
    Uint32                      noLearnMask;

    /**  enable limit IP nxt header */
    Uint32                      limitIPNxtHdr;

    /**  Drop fragmented IP packets */
    Uint32                      disallowIPFragmentation;

} CSL_CPSW_ALE_VLAN_ENTRY;

/** @brief
 *
 *  Holds the ALE Outer VLAN Table entry configuration.
 */
typedef CSL_CPSW_ALE_VLAN_ENTRY CSL_CPSW_ALE_OUTER_VLAN_ENTRY;

/** @brief
 *
 *  Holds the ALE Ethertype Table entry configuration.
 */
typedef struct {
    /**  Ethernet Type  */
    Uint32                      ethertype;

} CSL_CPSW_ALE_ETHERTYPE_ENTRY;


/** @brief
 *
 *  Holds the ALE IPv4 Address Table entry
 *  configuration.
 */
typedef struct {
    /**  IPv4 address */
    Uint8                       address [4];
    /**  CIDR Mask: Number of least significant bits to ignore in address match */
    UInt32                      numLSBIgnore;

} CSL_CPSW_ALE_IPv4_ENTRY;

/** @brief
 *
 *  Holds the ALE IPv6 Address Table entry
 *  configuration.
 */
typedef struct {
    /**  IPv6 address */
    Uint8                       address [16];
    /**  CIDR Mask: Number of least significant bits to ignore in address match */
    UInt32                      numLSBIgnore;

} CSL_CPSW_ALE_IPv6_ENTRY;


#define CSL_CPSW_NUMALE_ENTRIES_MIN (64)

#define CSL_CPSW_NUMSTATBLOCKS      (9)



/** @brief Number of statistic blocks.
 *
 *  EMAC has multiple statistics blocks.
 *
 *  STATS0 holds statistics for Host/CPU port (Switch port 0).
 *  STATSn holds statistics for MAC ports (Switch ports n).
 */


/** @brief
 *
 *  Defines ALE RAMDEPTH enums for storing IPv6 sliceIndex entry
 */
typedef enum {
    CSL_ALE_RAMDEPTH_32 = 0,  /** RAMDEPTH is 32  */
    CSL_ALE_RAMDEPTH_64 = 1,  /** RAMDEPTH is 32  */
    CSL_ALE_RAMDEPTH_128 = 2,  /** RAMDEPTH is 32  */
} CSL_CPSW_ALE_RAMDEPTH_E;

/** @brief
 *
 *  Defines ALE Aging Timer Prescale
 */
typedef enum {
    CSL_ALE_AGT_PRESACLE_1M = 0,  /** 1000,000 (default value) */
    CSL_ALE_AGT_PRESACLE_1000,    /** 1000 (test value) */
    CSL_ALE_AGT_PRESACLE_1        /** 1 (test value) */
} CSL_CPSW_ALE_AGT_PRESCALE_E;

/** @brief
 *
 *  Holds Port Statistics Enable register contents.
 */
typedef struct {
    /**  Port 0 Statistics Enable bit */
    Uint32      p0StatEnable;

    /**  Port 1 Statistics Enable bit */
    Uint32      p1StatEnable;

    /**  Port 2 Statistics Enable bit */
    Uint32      p2StatEnable;

    /**  Port 3 Statistics Enable bit */
    Uint32      p3StatEnable;

    /**  Port 4 Statistics Enable bit */
    Uint32      p4StatEnable;

    /**  Port 5 Statistics Enable bit */
    Uint32      p5StatEnable;

    /**  Port 6 Statistics Enable bit */
    Uint32      p6StatEnable;

    /**  Port 7 Statistics Enable bit */
    Uint32      p7StatEnable;

    /**  Port 8 Statistics Enable bit */
    Uint32      p8StatEnable;

} CSL_CPSW_PORTSTAT;

/** @brief
 *
 *  Holds Port Time Sync Control register contents.
 */
typedef struct {
    /**  Port Time sync receive Annex D enable bit */
    Uint32      tsRxAnnexDEnable;

    /**  Port Time sync receive Annex E enable bit */
    Uint32      tsRxAnnexEEnable;

    /**  Port Time sync receive Annex F enable bit */
    Uint32      tsRxAnnexFEnable;

    /**  Port Time sync receive VLAN LTYPE 1 enable bit */
    Uint32      tsRxVlanLType1Enable;

    /**  Port Time sync receive VLAN LTYPE 2 enable bit */
    Uint32      tsRxVlanLType2Enable;

    /**  Port Time sync transmit Annex D enable bit */
    Uint32      tsTxAnnexDEnable;

    /**  Port Time sync transmit Annex E enable bit */
    Uint32      tsTxAnnexEEnable;

    /**  Port Time sync transmit Annex F enable bit */
    Uint32      tsTxAnnexFEnable;

    /**  Port Time sync transmit VLAN LTYPE 1 enable bit */
    Uint32      tsTxVlanLType1Enable;

    /**  Port Time sync transmit VLAN LTYPE 2 enable bit */
    Uint32      tsTxVlanLType2Enable;

    /**  Port Time sync transmit host timestamp enable bit */
    Uint32      tsTxHostEnable;

    /**  Port Time sync transmit and receive LType2 enable bit */
    Uint32      tsLType2Enable;

    /**  Port Time sync message type enable bits */
    Uint32      tsMsgTypeEnable;

} CSL_CPSW_TSCNTL;


/** @brief
 *
 *  Holds Port Time Sync Configuration contents.
 */
typedef struct {
    /**  Port Time sync receive Annex D enable */
    Uint32      tsRxAnnexDEnable;

    /**  Port Time sync receive Annex E enable */
    Uint32      tsRxAnnexEEnable;

    /**  Port Time sync receive Annex F enable */
    Uint32      tsRxAnnexFEnable;

    /**  Port Time sync receive VLAN LTYPE 1 enable */
    Uint32      tsRxVlanLType1Enable;

    /**  Port Time sync receive VLAN LTYPE 2 enable */
    Uint32      tsRxVlanLType2Enable;

    /**  Port Time sync transmit Annex D enable */
    Uint32      tsTxAnnexDEnable;

    /**  Port Time sync transmit Annex E enable */
    Uint32      tsTxAnnexEEnable;

    /**  Port Time sync transmit Annex F enable */
    Uint32      tsTxAnnexFEnable;

    /**  Port Time sync transmit VLAN LTYPE 1 enable */
    Uint32      tsTxVlanLType1Enable;

    /**  Port Time sync transmit VLAN LTYPE 2 enable */
    Uint32      tsTxVlanLType2Enable;

    /**  Port Time sync transmit host timestamp enable */
    Uint32      tsTxHostEnable;

    /**  Port Time sync transmit and receive LType2 enable */
    Uint32      tsLType2Enable;

    /**  Port Time sync message type enable bitmap */
    Uint32      tsMsgTypeEnable;

    /**  Port Time Sync Unicast IP message enable
         1: all IP address valid
         0: perform multicast address check, only multicast addresses which are enabled are valid
      */
    Uint32      tsUniEnable;

    /**  Port Time Sync Destination IP Address 107 enable
         IPv4: 224.0.0.107
         IPv6: FF0M:0:0:0:0:0:0:006B
     */
    Uint32      ts107Enable;

    /**  Port Time Sync Destination IP Address 129 enable
         IPv4: 224.0.0.129
         IPv6: FF0M:0:0:0:0:0:0:0181
     */
    Uint32      ts129Enable;

    /**  Port Time Sync Destination IP Address 130 enable
         IPv4: 224.0.0.130
         IPv6: FF0M:0:0:0:0:0:0:0182
     */
    Uint32      ts130Enable;

    /**  Port Time Sync Destination IP Address 131 enable
         IPv4: 224.0.0.130
         IPv6: FF0M:0:0:0:0:0:0:0183
     */
    Uint32      ts131Enable;

    /**  Port Time Sync Destination IP Address 132 enable
         IPv4: 224.0.0.131
         IPv6: FF0M:0:0:0:0:0:0:0184
     */
    Uint32      ts132Enable;

    /**  Port Time Sync UDP Destination Port Number 319 enable */
    Uint32      ts319Enable;

    /**  Port Time Sync UDP Destination Port Number 320 enable */
    Uint32      ts320Enable;

    /**  Port Time sync message IPv6 Multicast Address FF0M enable */
    Uint32      tsMcastTypeEnable;

    /**  Port Time Sync Time to Live Non-zero enable */
    Uint32      tsTTLNonzeroEnable;

    /**  Port Time Sync LTYPE1 value for Annex F packets */
    Uint32      tsLType1;

    /**  Port Time Sync LTYPE2 value for Annex F packets */
    Uint32      tsLType2;

    /**  Port Time Sync VLAN LTYPE1 value for both Tx and Rx packets */
    Uint32      tsVlanLType1;

    /**  Port Time Sync VLAN LTYPE2 value for both Tx and Rx packets */
    Uint32      tsVlanLType2;

    /**  Port Time Sync Sequence ID field byte Offset in PTP message */
    Uint32      tsSeqIdOffset;

    /**  Port Time Sync Domain field byte Offset in PTP message */
    Uint32      tsDomainOffset;

} CSL_CPSW_TSCONFIG;


/** @brief
 *
 *  Holds CPSW Port Control contents.
 */
typedef struct {
    /** EST Port Enable */
    Uint32      estPortEnable;

    /** IET Port Enable */
    Uint32      ietPortEnable;

    /**  Eneregy Efficient Etherent (EEE) Transmit LPI clockstop enable
         for EMAC port only
         1: The GMII or RGMII transmit clock is stopped in the EEE
            LPI state.
         0: The GMII or RGMII transmit clock is not stopped in the
            EEE LPI state.
      */
    Uint32      txLpiClkstopEnable;

    /**  IPv6 DSCP to priority mapping enable */
    Uint32      dscpIpv6Enable;

    /**  IPv4 DSCP to priority mapping enable */
    Uint32      dscpIpv4Enable;

} CSL_CPSW_PORT_CONTROL;


/** @brief
 *
 *  Holds CPSW Port Rx Rate Limit Configuration for CPPI Port Ingress Rate Limitaion Operation.
 */
typedef struct {
    /**  Number of Rate Limitaion Channels. Rate limited channels must be the highest priority channels.
         For example, if two rate limited channels are required then they must be channel with priority
         7 and 6 respectively. The BW of rate limitation channel is calcualted as
         idleStep/(idleStep + sendStep)*Frequemcy*256 where frequency = the VBUSP_GCLK frequency (350 for 350Mhz)
      */
    Uint32      numRLimChans;

    /**  Rate Limitaion Idle Step Array */
    Uint32      idleStep[8];

    /**  Rate Limitaion Send Step Array  */
    Uint32      sendStep[8];

} CSL_CPSW_RX_RATE_LIMIT_CONFIG;

/** @brief
 *
 *  Holds CPSW EEE (Energy Efficient Ethernet)  Global Configuration.
 */
typedef struct {
    /**  Energy Efficient Ethernet enable */
    Uint32      enable;

    /**  Energy Efficient Ethernet Pre-scale count load value */
    Uint32      prescale;

} CSL_CPSW_EEE_GLOB_CONFIG;

/** @brief
 *
 *  Holds CPSW EEE (Energy Efficient Ethernet)  Per-Port Configuration.
 */
typedef struct {
    /**  EEE Transmit LPI clockstop enable
         for EMAC port only
         1: The GMII or RGMII transmit clock is stopped in the EEE
            LPI state.
         0: The GMII or RGMII transmit clock is not stopped in the
            EEE LPI state.
      */
    Uint32      txLpiClkstopEnable;

    /**  EEE Idle to LPI counter load value - After EEE_CLKSTOP_REQ is asserted,
         this value is loaded into the port idle to LPI counter on each clock
         that the port transmit or receive is not idle.  Port enters the LPI
         state when this count decrements to zero.
         This count value should be large relative to switch operations */
    Uint32      idle2lpi;

    /**  EEE LPI to wake counter load value - When the port is in the transmit
         LPI state and the EEE_CLKSTOP_REQ signal is deasserted, this value is
         loaded into the port 0 LPI to wake counter.
         Transmit and receive packet operations may begin (resume) when
         the LPI to wake count decrements to zero.  This is the time waited
         before CPPI packet operations begin (resume after wakeup).
         This count value should be large relative to switch operations. */
    Uint32      lpi2wake;

} CSL_CPSW_EEE_PORT_CONFIG;


/** @brief
 *
 *  Holds CPSW EEE (Energy Efficient Ethernet)  Per-Port STATUS.
 */
typedef struct {
    /**  CPSW port wait idle to LPI - asserted when port is counting the IDLE2LPI time */
    Uint32      wait_idle2lpi;

    /**  CPSW port receive LPI state - asserted when the port receive is in the LPI state */
    Uint32      rxLpi;

    /**  CPSW port transmit LPI state - asserted when the port transmit is in the LPI state */
    Uint32      txLpi;

    /**  CPSW port transmit wakeup - asserted when the port transmit is in the transmit
         LPI2WAKE count time. */
    Uint32      txWake;

    /**  CPSW port transmit FIFO hold - asserted in the LPI state and
         during the LPI2WAKE count time. */
    Uint32      txFifoHold;

    /** CPSW port receive FIFO (switch ingress) is empty - contains no packets */
    Uint32      rxFifoEmpty;

    /** CPSW port transmit FIFO (switch egress) is empty - contains no packets */
    Uint32      txFifoEmpty;

} CSL_CPSW_EEE_PORT_STATUS;

/** @brief
 *
 *  Defines ALE Update Bandwidth Control Value:
 *  The upd_bw_ctrl field within ALE control register specifies the rate in which adds,
 *  updates, touches, writes, and aging updates can occur. At frequencies of 350Mhz,
 *  the table update rate should be at its lowest or 5 Million updates per second.
 *  When operating the switch core at frequencies or above, the upd_bw_ctrl can be
 *  programmed more aggressive.
 *  If the upd_bw_ctrl is set but the frequency of the switch subsystem is below the
 *  associated value, ALE will drop packets due to insufficient time to complete
 *  lookup under high traffic loads.
 */
typedef enum {
    CSL_ALE_UPD_BW_350MHZ_5M = 0,           /** 0 - 350Mhz, 5M */
    CSL_ALE_UPD_BW_359MHZ_11M,              /** 1 - 359Mhz, 11M */
    CSL_ALE_UPD_BW_367MHZ_16M,              /** 2 - 367Mhz, 16M */
    CSL_ALE_UPD_BW_375MHZ_22M,              /** 3 - 375Mhz, 22M */
    CSL_ALE_UPD_BW_384MHZ_28M,              /** 4 - 384Mhz, 28M */
    CSL_ALE_UPD_BW_392MHZ_34M,              /** 5 - 392Mhz, 34M */
    CSL_ALE_UPD_BW_400MHZ_39M,              /** 6 - 400Mhz, 39M */
    CSL_ALE_UPD_BW_409MHZ_45M               /** 7 - 409Mhz, 45M */
} CSL_CPSW_ALE_UPD_BW;

/** @brief
 *
 *  Holds CPSW Policer  Global Configuration.
 */
typedef struct {
    /** ALE Policer Default Thread Enable:
        Set: the aleDeafultThread is used for the CPPI transmit thread
             if there are no matching classifiers.
        Clear, the switch default thread is used for the CPPI transmit thread i
        f there are no matching classifiers.
     */

    Uint32      defThreadEnable;

    /**  ALE Default Thread */
    Uint32      defThread;

} CSL_CPSW_ALE_POLICER_GLOB_CONFIG;

typedef struct {
    Uint32      trunkBase;
    bool     trunkEnableDestIP;
    bool     trunkEnableSrcIP;
    bool     trunkEnableInnerVLAN;
    bool     trunkEnablePri;
    bool     trunkEnableSrc;
    bool     trunkEnableDst;
} CSL_CPSW_ALE_CTRL2_TRUNK_CONFIG;


typedef struct {
    bool     ipPktFltEnableDefNxtHdrLimit;
    bool     ipPktFltEnableDefNoFrag;
} CSL_CPSW_ALE_CTRL2_IPPKTFLT_CONFIG;

typedef struct {
    bool     dropBadLen;
    bool     noDropSrcMcast;
} CSL_CPSW_ALE_CTRL2_MALFORMEDFRAME_CONFIG;

#define CSL_ALE_TABLE_POLICER_ENUM2REG(policerType) ((policerType) << 0x1)


/** @brief ALE Policer Entry configuration definitions */
/**  Input EMAC port is used for classification */
#define    CSL_CPSW_ALE_POLICER_PORT_VALID       (1 << 0u)

/**  VLAN Priority is used for classification */
#define    CSL_CPSW_ALE_POLICER_PRI_VALID        (1 << 1u)

/**  OUI is used for classification */
#define    CSL_CPSW_ALE_POLICER_OUI_VALID        (1 << 2u)

/**  Destination MAC is used for classification */
#define    CSL_CPSW_ALE_POLICER_DST_MAC_VALID    (1 << 3u)

/**  Source MAC is used for classification */
#define    CSL_CPSW_ALE_POLICER_SRC_MAC_VALID    (1 << 4u)

/**  Outer VLAN ID is used for classification */
#define    CSL_CPSW_ALE_POLICER_OVLAN_VALID      (1 << 5u)

/**  (Inner) VLAN ID is used for classification */
#define    CSL_CPSW_ALE_POLICER_VLAN_VALID       (1 << 6u)

/**  Ethertype is used for classification */
#define    CSL_CPSW_ALE_POLICER_ETHERTYPE_VALID  (1 << 7u)

/**  Source IP address is used for classification */
#define    CSL_CPSW_ALE_POLICER_SRC_IP_VALID    ( 1 << 8u)

/**  Destination IP address is used for classification */
#define    CSL_CPSW_ALE_POLICER_DST_IP_VALID     (1 << 9u)

/**  The specified thread value is used as the CPPI egress thread
     for any packet that matches the classifier  */
#define    CSL_CPSW_ALE_POLICER_THREAD_VALID    ( 1 << 10u)

/**  The specified pir_idl_inc value is used  */
#define    CSL_CPSW_ALE_POLICER_PIR_VALID       ( 1 << 11u)

/**  The specified cir_idl_inc value is used  */
#define    CSL_CPSW_ALE_POLICER_CIR_VALID       ( 1 << 12u)

/**  Input EMAC port is used for classification */
#define    CSL_CPSW_ALE_POLICER_PORT_TRUNK_VALID (1 << 13u)

/**  The specified egress op is used */
#define    CSL_CPSW_ALE_POLICER_EGRESSOP_VALID   (1 << 14u)

/** @brief
 *
 *  Holds the ALE Policer Table entry configuration.
 */
typedef struct {
    /**  Configuration control bitmap as defined above */
    Uint32                      validBitmap;

    /**  Ingress EMAC port */
    Uint32                      port;

    /**  VLAN priority */
    Uint32                      pri;

    /**  OUI entry index  */
    Uint32                      ouiIdx;

    /**  Destination MAC address entry index  */
    Uint32                      dstMacIdx;

    /**  Source MAC address entry index  */
    Uint32                      srcMacIdx;

    /**  Outer VLAN entry index  */
    Uint32                      ovlanIdx;

    /**  (Inner) VLAN entry index  */
    Uint32                      vlanIdx;

    /**  Ethertype entry index  */
    Uint32                      ethertypeIdx;

    /**  Source IP address entry index  */
    Uint32                      srcIpIdx;

    /**  Destination IP address entry index  */
    Uint32                      dstIpIdx;

    /**  CPPI Egress thread upon match  */
    Uint32                      thread;

    /** Peak Information Rate Idle Increment Value
      * The number added to the PIR counter every clock cycle. If zero the PIR
      * counter is disabled and packets will never be marked or
      * processed as RED  */
    Uint32                      pirIdleIncVal;

    /** Committed Information Rate Idle Increment Value
      * The number added to the CIR counter every clock cycle. If zero the CIR
      * counter is disabled and packets will never be marked or
      * processed as YELLOW  */
    Uint32                      cirIdleIncVal;
    /** Egress Op  Value
      * The Egress Operation value allows enabled classifiers with IPSA or IPDA
      * match to use the CPSW Egress Packet Operations Inter VLAN Routing
      * sub functions
      */
    Uint32                      egressOp;
    /** Egress trunk index
      * The Egress Trunk Index is the calculated trunk index from the SA,
      * DA or VLAN. If modified to that, InterVLAN routing will work on trunks
      * as well. The DA, SA and VLAN are ignored for trunk generation on
      * InterVLAN Routing so that this field is the index generated from the
      * Egress Op replacements XORed together into a three bit
      * index.
      */
    Uint32                      egressTrunkIndex;
    /** TTL check value
     * The TTL Check will cause any packet that fails TTL checks to not
     * be routed to the Inter VLAN Routing sub functions. The packet will
     * be routed to the host it was destined to.
     */
    Uint32                      enableTTLCheck;
    /** The Destination Ports is a list of the ports the classified packet will
      * be set to. If a destination is a Trunk, all the port bits for that trunck
      * must be set.
      */
    Uint32                      destPortMask;

} CSL_CPSW_ALE_POLICER_ENTRY;

union CSL_CPSW_STATS {
    CSL_Xge_cpswP0StatsRegs p0_stats;
    CSL_Xge_cpswPnStatsRegs pn_stats;
};

typedef enum {
    CSL_ALE_POLICER_CONTROL_POLICING_MATCH_MODE_NOMATCH_GREEN,
    CSL_ALE_POLICER_CONTROL_POLICING_MATCH_MODE_NOMATCH_YELLOW,
    CSL_ALE_POLICER_CONTROL_POLICING_MATCH_MODE_NOMATCH_RED,
    CSL_ALE_POLICER_CONTROL_POLICING_MATCH_MODE_NOMATCH_ENTRY0STATE,
} CSL_CPSW_ALE_POLICER_CONTROL_POLICING_MATCH_MODE;

typedef enum {
    CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_100,
    CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_50,
    CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_33,
    CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_25,
    CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_20,
    CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_17,
    CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_14,
    CSL_ALE_POLICER_CONTROL_YELLOWTHRESH_DROP_PERCENT_13,
} CSL_CPSW_ALE_POLICER_CONTROL_YELLOWTHRESH;

typedef struct
{
    CSL_CPSW_ALE_POLICER_CONTROL_POLICING_MATCH_MODE policeMatchMode;
    CSL_CPSW_ALE_POLICER_CONTROL_YELLOWTHRESH yellowDropThresh;
    Uint32 yellowDropEnable;
    Uint32 redDropEnable;
    Uint32 policingEnable;
    Uint32 enablePriorityOR;
    Uint32 disableMacPortDefaultThread;
} CSL_CPSW_ALE_POLICER_CONTROL;

typedef struct
{
    Uint32 polClrallHit;
    Uint32 polClrallRedhit;
    Uint32 polClrallYellowhit;
    Uint32 polClrselAll;
    Uint32 polTestIdx;
} CSL_CPSW_ALE_POLICER_TEST_CONTROL;

typedef struct
{
    Uint32  polHit        ;
    Uint32  polRedhit     ;
    Uint32  polYellowhit  ;
} CSL_CPSW_ALE_POLICER_HSTAT;

/** @brief
 *
 *  Holds CPSW priority type register contents.
 */
typedef struct {
    /**  Port 0 Priority Escalation Enable bit */
    Uint32      port0PriorityTypeEscalateEnable;

    /**  Port 1 Priority Escalation Enable bit */
    Uint32      port1PriorityTypeEscalateEnable;

    /**  Port 2 Priority Escalation Enable bit */
    Uint32      port2PriorityTypeEscalateEnable;

    /**  Port 3 Priority Escalation Enable bit */
    Uint32      port3PriorityTypeEscalateEnable;

    /**  Port 4 Priority Escalation Enable bit */
    Uint32      port4PriorityTypeEscalateEnable;

    /**  Port 5 Priority Escalation Enable bit */
    Uint32      port5PriorityTypeEscalateEnable;

    /**  Port 6 Priority Escalation Enable bit */
    Uint32      port6PriorityTypeEscalateEnable;

    /**  Port 7 Priority Escalation Enable bit */
    Uint32      port7PriorityTypeEscalateEnable;

    /**  Port 8 Priority Escalation Enable bit */
    Uint32      port8PriorityTypeEscalateEnable;

    /** Escalate priority load value
      * When a port is in escalate priority, this is the number of
      * higher priority packets sent before the next lower priority
      * is allowed to send a packet.
      * Escalate priority allows lower priority packets to be sent at a fixed rate
      * relative to the next higher priority.
      * The min value of esc_pri_ld_val = 2
      */
    Uint32      escPriLoadVal;
} CSL_CPSW_PTYPE;

/** @brief
 *
 *  Holds the Port intervlan configuration info.
 */
typedef struct {
    /**  Destination mac address to be replaced */
    Uint8                       dstMacAddress [6];
    /**  Source mac address to be replaced */
    Uint8                       srcMacAddress [6];
    /** Decrement Time To Live -
      * When set, the Time To Live (TTL) field in the header is decremented:
      *  IPV4 - Decrement the TTL byte and update the Header Checksum
      *  IPV6 - Decrement the Hop Limit.
      *  note: When this bit is set, the ALE should be configured to send any
      *  IPv4/6 packet with a zero or one TTL field to the host.
      *  When this bit is cleared the TTL/Hop Limit fields are not checked or
      *  modified.
      */
    Uint32                      decrementTtl;
    /** Destination VLAN Force Untagged Egress - When set, this bit indicates
      * that the VLAN should be removed on egress for the routed packet.
      */
    Uint32                      destForceUntaggedEgress;
    /** Replace Destination Address and Source Address - When set this bit
      * indicates that the routed packet destination address should be replaced
      * by da[47:0] and the source address should be replaced by sa[47:0]
      */
    Uint32                      replaceDaSa;
    /** Replace VLAN ID - When set this bit indicates that the VLAN ID
      * should be replaced for the routed packet
      */
    Uint32                      replaceVid;
    /** VLAN ID to be set on outgoing packets */
    Uint32                      vid;
} CSL_CPSW_INTERVLANCFG;

/** @brief
 *
 *  Holds the Enet_Pn_FIFO_Status register contents
 */
typedef struct {
    /** EST RAM active buffer  .
      * Indicates the active 64-word fetch buffer when pn_est_onebuf is
      * cleared to zero.
      * Indicates the fetch ram address MSB when pn_est_onebuf set to one.
      */
    Uint32 estBufAct;
    /** EST Address Error
      * Indicates that the fetch ram was read again after the previous maximum
      * buffer address read (the previous fetch from the maximum address is
      * reused).
      */
    Uint32 estAddErr;
    /** EST Fetch Count Error
      * Indicates that insufficient clocks were programmed into the fetch count
      * and that another fetch was commanded before the previous fetch
      * finished.
      */
    Uint32 estCntErr;
    /** EST transmit mac allow
      * Bus that indicates the actual priorities assigned to the express queue
      * (and inversely the priorities assigned to the prempt queue).
      * The pn_mac_prempt[7:0] field in the Enet_Pn_IET_Control register
      * indicates which priorities should be assigned to the express/prempt
      * queues.  The switch between queues happens only when the priority is
      * empty and the actual assignment is shown in this field.
      */
    Uint32 txExpressMacAllow;
    /** EST Transmit Priority Active
      * Bus that indicates which priorities have packets (non-empty) at the
      * time of the register read.
      */
    Uint32 txPriActive;
} CSL_CPGMAC_SL_FIFOSTATUS;


/** @brief
 *
 *  Holds the Enet_Pn_EST_Control register contents
 */
typedef struct {
    /** EST fill margin.
     *  Sets the fill margin (in bytes) required to ensure that the Ethernet
     *  wire is clear so that the timed EST express packet can egress at the
     *  correct required time.  Setting this value too high will put an
     *  unnecessary gap on the wire.  Setting this value too low will cause
     *  the timed express packet to egress at a time later than intended.
     */
    Uint32 estFillMargin;

    /** EST Prempt Comparison Value.
     *  When the count in a zero allow is less than or equal to this value in
     *  bytes (times 8), prempt packets are cleared from the wire. This is the
     *  prempt clear margin value.
     */
    Uint32 estPremptComp;

    /** EST Fill Enable.
     *  Enable EST fill mode.
     */
    Uint32 estFillEnable;

    /** EST Timestamp Express Priority.
     *  Selects the express priority that timestamp(s) will be generated on
     *  when PN_EST_TS_ONEPRI is set.
     */
    Uint32 estTsPri;

    /** EST Timestamp One Express Priority.
     *  When set, timestamp only enabled packets on the express priority
     *  selected by PN_EST_TS_PRI.  When cleared to zero, express packet
     *  selection for timestamp is independent of priority.
     */
    Uint32 estTsOnePri;

    /** EST Timestamp First Express Packet only.
     *  Generate a timestamp only on the first selected express packet in each
     *  EST time interval when express timestamps are enabled.
     *  (If PN_EST_TS_ONEPRI is also set then the timestamp is generated only
     *  on the first packet on PN_EST_TS_PRI).
     */
    Uint32 estTsFirst;

    /** EST Timestamp Enable.
     *  Enable express timestamps (when EST_EN and PN_EST_PORT_EN are set).
     */
    Uint32 estTsEnable;

    /** EST Buffer Select.
     *  If PN_EST_ONEBUF is cleared, this bit selects the upper (when set) or
     *  the lower (when cleared) 64-word fetch buffer.  The actual fetch buffer
     *  used changes only at the start of the EST time interval and can be read
     *  in the PN_FIFO_STATUS register PN_EST_BUFACT bit.
     */
    Uint32 estBufSel;

    /** EST One Fetch Buffer.
     *  When set indicates that all 128 fetch words are used in one buffer.
     *  When cleared, indicates that the 128 fetch words are split into two 64-word
     *  fetch buffers.  The pn_est_bufsel selects the buffer to be used when
     *  PN_EST_ONEBUF is cleared to zero.
     */
    Uint32 estOneBuf;
} CSL_CPSW_EST_CONFIG;

/** @brief
 *
 *  Holds the Enet_Pn_IET_Control register contents
 */
typedef struct {
    /** Mac Prempt Queue.
     *  Indicates which transmit FIFO queues are sent to the prempt MAC.
     *  Bit 0 indicates queue zero, bit 1 queue 1 and so on.  Packets will
     *  be sent to the prempt MAC only when pn_mac_penable is set, and when
     *  mac_verified (from Enet_Pn_IET_Status) or pn_mac_disableverify is set,
     *  and when pn_iet_port_en is set.
     */
    Uint32 macPremptQueue;

    /** Mac Fragment Size.
     *  An integer in the range 0:7 indicating, as a multiple of 64,
     *  the minimum additional length for nonfinal mPackets
     *  0 = 64
     *  1 = 128
     *  …
     *  7 = 512
     */
    Uint32 macAddFragSize;

    /** Mac Link Fail.
     *  Link Fail Indicatior to reset the verifly state machine.
     *  This bit is reset high.  Verify and response frames will
     *  be sent/allowed when this bit is cleared.
     */
    Uint32 macLinkFail;

    /** Mac Disable Verify.
     *  Disables verification on the port when set.  If this bit is set
     *  then packets will be sent to the prempt MAC when mac_penable is set
     *  (This is a forced mode with no IET verification).
     */
    Uint32 macDisableVerify;

    /** Mac Hold.
     *  Hold Premptable traffic on the port.
     */
    Uint32 macHold;

    /** Mac Premption Enable.
     *  Port Premption Enable. This takes effect only when pn_iet_port_en is set.
     */
    Uint32 macPremptEnable;
} CSL_CPSW_IET_CONFIG;

/** @brief
 *
 *  Holds the Enet_Pn_IET_Status register contents
 */
typedef struct {
    /** Mac Verified.
     *  Indication that verification was successful.
     */
    Uint8 macVerified;

    /** Mac Verification Failed.
     * Indication that verification was unsuccessful.
     */
    Uint8 macVerifyFail;

    /** Mac Received Respond Packet with Errors.
     *  Set when a respond packet with errors is received.
     *  Cleared when pn_mac_penable is cleared to zero.
     */
    Uint8 macRxRespondErr;

    /** Mac Received Verify Packet with Errors.
     *  Set when a verify packet with errors is received.
     *  Cleared when pn_mac_penable is cleared to zero.
     */
    Uint8 macRxVerifyErr;
} CSL_CPSW_IET_STATUS;

/**
@}
*/


/** @addtogroup CSL_CPSWITCH_FUNCTION
@{ */

/********************************************************************************
************************* Ethernet Switch (CPSW) Submodule **********************
********************************************************************************/


/** ============================================================================
 *   @n@b CSL_CPSW_nGF_getCpswVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the CPSW identification and version information.
 *
 *   @b Arguments
     @verbatim
        pVersionInfo        CSL_CPSW_VERSION structure that needs to be populated
                            with the version info read from the hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CPSW_ID_VER_REG_CPSW_5GF_MINOR_VER,
 *      XGE_CPSW_CPSW_ID_VER_REG_CPSW_5GF_MAJ_VER,
 *      XGE_CPSW_CPSW_ID_VER_REG_CPSW_5GF_RTL_VER,
 *      XGE_CPSW_CPSW_ID_VER_REG_CPSW_5GF_IDENT
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_VERSION    versionInfo;

        CSL_CPSW_getCpswVersionInfo (&versionInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getCpswVersionInfo (CSL_Xge_cpswRegs *hCpswRegs,
    CSL_CPSW_VERSION*       pVersionInfo
);


/** ============================================================================
 *   @n@b CSL_CPSW_isVlanAwareEnabled
 *
 *   @b Description
 *   @n This function indicates if VLAN aware mode is enabled in the CPSW
 *      control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   VLAN aware mode enabled.
 *   @n  FALSE                  VLAN aware mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isVlanAwareEnabled (portNum) == TRUE)
        {
            // VLAN aware mode enabled
        }
        else
        {
            // VLAN aware mode disabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isVlanAwareEnabled (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableVlanAware
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable VLAN aware
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableVlanAware ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableVlanAware (CSL_Xge_cpswRegs *hCpswRegs);


void CSL_CPSW_setVlanType (CSL_Xge_cpswRegs *hCpswRegs,Uint32 vlanType);


/** ============================================================================
 *   @n@b CSL_CPSW_disableVlanAware
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable VLAN aware
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableVlanAware ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableVlanAware (CSL_Xge_cpswRegs *hCpswRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_enableCutThru
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable cut-thru
 *      switching.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_CUT_THRU_ENABLE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableCutThru ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableCutThru (CSL_Xge_cpswRegs *hCpswRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_disableCutThru
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable cut-thru
 *      switching.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_CUT_THRU_ENABLE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableCutThru ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableCutThru (CSL_Xge_cpswRegs *hCpswRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_setCpswFrequency
 *
 *   @b Description
 *   @n This function configures the CPSW frequency used for enabling auto speed
 *      register for cut-thru switching.
 *
 *   @b Arguments
 *   @verbatim
        pCpswFrequency      CPSW_FREQUENCY that needs to be set.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_FREQUENCY_REG
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_setCpswFrequency ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setCpswFrequency (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      pCpswFrequency
);

/** ============================================================================
 *   @n@b CSL_CPSW_getCpswFrequency
 *
 *   @b Description
 *   @n This function gets the RX Cut thru priority
 *
 *   @b Arguments
     @verbatim
        pCpswFrequency   CPSW frequency
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_FREQUENCY_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32      pCpswFrequency;

        pCpswFrequency = CSL_CPSW_getCpswFrequency ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getCpswFrequency (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pCpswFrequency
);

/** ============================================================================
 *   @n@b CSL_CPSW_isPort0Enabled
 *
 *   @b Description
 *   @n This function indicates if CPPI Port (Port 0) is enabled in the CPSW
 *      control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   Port 0 enabled.
 *   @n  FALSE                  Port 0 disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_P0_ENABLE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isPort0Enabled (portNum) == TRUE)
        {
            // Port 0 enabled
        }
        else
        {
            // Port 0 disabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isPort0Enabled (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enablePort0
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable the Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P0_ENABLE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enablePort0 ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enablePort0 (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disablePort0
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable the Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P0_ENABLE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disablePort0 ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disablePort0 (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isPort0PassPriTagEnabled
 *
 *   @b Description
 *   @n This function indicates if priority tagging is enabled for Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   Port 0 ingress priority tagging enabled.
 *   @n  FALSE                  Port 0 ingress priority tagging disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isPort0PassPriTagEnabled () == TRUE)
        {
            // Port 0 pass priority tagging enabled
        }
        else
        {
            // Port 0 pass priority tagging disabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isPort0PassPriTagEnabled (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enablePort0PassPriTag
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable the Ingress
 *      priority tagging on Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enablePort0PassPriTag ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enablePort0PassPriTag (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disablePort0PassPriTag
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable the Ingress
 *      priority tagging on Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disablePort0PassPriTag ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disablePort0PassPriTag (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isPort1PassPriTagEnabled
 *
 *   @b Description
 *   @n This function indicates if priority tagging is enabled for Port 1.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   Port 1 ingress priority tagging enabled.
 *   @n  FALSE                  Port 1 ingress priority tagging disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isPort1PassPriTagEnabled () == TRUE)
        {
            // Port 1 pass priority tagging enabled
        }
        else
        {
            // Port 1 pass priority tagging disabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isPort1PassPriTagEnabled (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enablePort1PassPriTag
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable the Ingress
 *      priority tagging on Port 1.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enablePort1PassPriTag ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enablePortPassPriTag (CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNum);


/** ============================================================================
 *   @n@b CSL_CPSW_disablePort1PassPriTag
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable the Ingress
 *      priority tagging on Port 1.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disablePort1PassPriTag ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disablePortPassPriTag (CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNum);


/** ============================================================================
 *   @n@b CSL_CPSW_getCpswControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW Control register.
 *
 *   @b Arguments
     @verbatim
        pControlRegInfo     CSL_CPSW_CONTROL structure that needs to be populated
                            with the control register contents.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE,
 *      XGE_CPSW_CONTROL_REG_P0_ENABLE,
 *      XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P0_RX_PAD,
 *      XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR,
 *      XGE_CPSW_CONTROL_REG_EEE_ENABLE
 *
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_CONTROL    controlRegInfo;

        CSL_CPSW_getCpswControlReg (&controlRegInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getCpswControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    CSL_CPSW_CONTROL*   pControlRegInfo
);


/** ============================================================================
 *   @n@b CSL_CPSW_setCpswControlReg
 *
 *   @b Description
 *   @n This function populates the contents of the CPSW Control register.
 *
 *   @b Arguments
     @verbatim
        pControlRegInfo     CSL_CPSW_CONTROL structure that holds the values
                            that need to be configured to the CPSW control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSW control register modified with values provided.
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE,
 *      XGE_CPSW_CONTROL_REG_P0_ENABLE,
 *      XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P0_TX_CRC_REMOVE,
 *      XGE_CPSW_CONTROL_REG_P0_RX_PAD,
 *      XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR,
 *      XGE_CPSW_CONTROL_REG_EEE_ENABLE
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_CONTROL    controlRegInfo;

        controlRegInfo.vlanAware    =   0;
        ...

        CSL_CPSW_setCpswControlReg (&controlRegInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setCpswControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    CSL_CPSW_CONTROL*   pControlRegInfo
);


/** ============================================================================
 *   @n@b CSL_CPSW_getEmulationControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW Emulation Control register.
 *
 *   @b Arguments
     @verbatim
        pFree                   Emulation free bit read from the hardware.
        pSoft                   Emulation soft bit read from the hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_EM_CONTROL_REG_FREE,
 *      XGE_CPSW_EM_CONTROL_REG_SOFT
 *
 *   @b Example
 *   @verbatim
        Uint32  free, soft;

        CSL_CPSW_getEmulationControlReg (&free, &soft);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getEmulationControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pFree,
    Uint32*                     pSoft
);


/** ============================================================================
 *   @n@b CSL_CPSW_setEmulationControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW Emulation Control register.
 *
 *   @b Arguments
     @verbatim
        free                   Emulation free bit configuration
        soft                   Emulation soft bit configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_EM_CONTROL_REG_FREE,
 *      XGE_CPSW_EM_CONTROL_REG_SOFT
 *
 *   @b Example
 *   @verbatim
        Uint32 free, soft;

        free   =   0;
        soft   =   1;

        CSL_CPSW_setEmulationControlReg (free, soft);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setEmulationControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      free,
    Uint32                      soft
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortStatsEnableReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW Port Statistics
 *      Enable register.
 *
 *   @b Arguments
     @verbatim
        pPortStatsCfg       CSL_XGE_CPSW_PORTSTAT structure that needs to be populated
                            with the port statistics enable register contents.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN,
 *      XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN,
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_PORTSTAT       portStatsCfg;

        CSL_CPSW_getPortStatsEnableReg (&portStatsCfg);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortStatsEnableReg (CSL_Xge_cpswRegs *hCpswRegs,
    CSL_CPSW_PORTSTAT*  pPortStatsCfg
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortStatsEnableReg
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW Port Statistics
 *      Enable register.
 *
 *   @b Arguments
     @verbatim
        pPortStatsCfg       CSL_CPSW_PORTSTAT structure that contains the values
                            to be used to setup port statistics enable register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN,
 *      XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN,
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_PORTSTAT       portStatsCfg;

        portStatsCfg.p0StatEnable  =   1;
        portStatsCfg.p1StatEnable  =   1;
        ...

        CSL_CPSW_setPortStatsEnableReg (&portStatsCfg);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortStatsEnableReg (CSL_Xge_cpswRegs *hCpswRegs,
    CSL_CPSW_PORTSTAT*  pPortStatsCfg
);


/** ============================================================================
 *   @n@b CSL_CPSW_isSoftIdle
 *
 *   @b Description
 *   @n This function indicates if the CPSW is at Software Idle mode where
 *      no packets will be started to be unloaded from ports.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   Software Idle mode enabled.
 *   @n  FALSE                  Software Idle mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isSoftIdle () == TRUE)
        {
            // Software Idle mode enabled
        }
        else
        {
            // Software Idle mode disabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isSoftIdle (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableSoftIdle
 *
 *   @b Description
 *   @n This function configures the CPSW Soft Idle register to enable Software
 *      Idle mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableSoftIdle ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableSoftIdle (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableSoftIdle
 *
 *   @b Description
 *   @n This function configures the CPSW Soft Idle register to disable Software
 *      Idle mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableSoftIdle ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableSoftIdle (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Control Register
 *      corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register contents
                                must be read and returned.
        pControlInfo            CSL_CPSW_PORT_CONTROL structure that needs to be populated
                                with the control register contents.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  none
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN,
 *      XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN,
 *
 *      XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN,
 *      XGE_CPSW_PN_CONTROL_REG_IET_PORT_EN,
 *      XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN,
 *      XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN,
 *      XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN,
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32                      portNum;
        CSL_CPSW_PORT_CONTROL       controlInfo;

        portNum =   1;

        CSL_CPSW_getPortControlReg (portNum, &controlInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPSW_PORT_CONTROL*      pControlInfo
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port Control Register
 *      corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register contents
                                must be read and returned.
        pControlInfo            CSL_CPSW_PORT_CONTROL structure that holds the values
                                that need to be configured to the CPSW Port
                                control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  none
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN,
 *      XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN,
 *
 *      XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN,
 *      XGE_CPSW_PN_CONTROL_REG_IET_PORT_EN,
 *      XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN,
 *      XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN,
 *      XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN,
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32                      portNum;
        CSL_CPSW_PORT_CONTROL       controlInfo;

        portNum =   1;

        controlInfo.dscpIpv4Enable = 1;
        controlInfo.dscpIpv6Enable = 1;
        controlInfo.txLpiClkstopEnable = 0;


        CSL_CPSW_setPortControlReg (portNum, &controlInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPSW_PORT_CONTROL*      pControlInfo
);


/** ============================================================================
 *   @n@b CSL_CPSW_getCppiSourceIdReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPPI Source Identification
 *      register.
 *
 *   @b Arguments
     @verbatim
        pTxSrcId[8]         CPPI Info Word0 Source Id Value on Tx Ports respectively.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_SRC_ID_A_REG_PORT1
 *
 *   @b Example
 *   @verbatim
 *      Uint32      txSrcId[8];

        CSL_CPSW_getCppiSourceIdReg (txSrcId);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getCppiSourceIdReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                 pTxSrcId
);


/** ============================================================================
 *   @n@b CSL_CPSW_setCppiSourceIdReg
 *
 *   @b Description
 *   @n This function sets up the contents of the CPPI Source Identification
 *      register.
 *
 *   @b Arguments
     @verbatim
        pTxSrcId[8]         CPPI Info Word0 Source Id Value on Tx Ports respectively.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_SRC_ID_A_REG_PORT1
 *
 *   @b Example
 *   @verbatim
 *      Uint32      txSrcId[8];

        txSrcId[0]    =   1;
        ...

        CSL_CPSW_setCppiSourceIdReg (txSrcId);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setCppiSourceIdReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                 pTxSrcId
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPort0VlanReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port 0 VLAN Register.
 *
 *   @b Arguments
     @verbatim
        pPortVID                Port VLAN Id
        pPortCFI                Port CFI bit
        pPortPRI                Port VLAN priority (0-7, 7 is highest priority)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portVID, portCFI, portPRI;

        CSL_CPSW_getPort0VlanReg (&portVID, &portCFI, &portPRI);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPort0VlanReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pPortVID,
    Uint32*                     pPortCFI,
    Uint32*                     pPortPRI
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPort0VlanReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port 0 VLAN Register.
 *
 *   @b Arguments
     @verbatim
        portVID                 Port VLAN Id to be configured
        portCFI                 Port CFI bit to be configured
        portPRI                 Port VLAN priority to be configured
                                (0-7, 7 is highest priority)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portVID, portCFI, portPRI;

        portVID     =   1;
        portCFI     =   0;
        portPRI     =   7;

        CSL_CPSW_setPort0VlanReg (portVID, portCFI, portPRI);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPort0VlanReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portVID,
    Uint32                      portCFI,
    Uint32                      portPRI
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPort0RxPriMapReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port 0 Receive Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        pPortRxPriMap           Array of Port 0 Rx priority map priority values
                                read from the register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pPortRxPriMap' must be large enough to hold all
 *       the 8 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      port0RxPriMap [8];

        CSL_CPSW_getPort0RxPriMapReg (port0RxPriMap);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPort0RxPriMapReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pPortRxPriMap
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPort0RxPriMapReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port 0 Receive Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        pPortRxPriMap           Array of Port 0 Rx priority map priority values
                                that must be configured to the register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      i, port0RxPriMap [8];

        for (i = 0; i < 8; i ++)
            port0RxPriMap [i] = i;

        CSL_CPSW_setPort0RxPriMapReg (port0RxPriMap);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPort0RxPriMapReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pPortRxPriMap
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPort0FlowIdOffset
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port 0 Flow ID Offset
 *      Register, which is added to the thread/Flow_ID in CPPI transmit PSI
 *      Info Word 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE
 *
 *   @b Example
 *   @verbatim
 *      Uint32      flowIdOffset;

        flowIdOffset    =   CSL_CPSW_getPort0FlowIdOffset ();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getPort0FlowIdOffset (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_setPort0FlowIdOffset
 *
 *   @b Description
 *   @n This function sets up the Port0 Flow ID Offset register.
 *      which is added to the thread/Flow_ID in CPPI transmit PSI
 *      Info Word 0.
 *
 *   @b Arguments
     @verbatim
        flowIdOffset            CPPI Flow ID offset to configure.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE
 *
 *   @b Example
 *   @verbatim
 *      Uint32      flowIdOffset;

        flowIdOffset    =   0;

        CSL_CPSW_setPort0FlowIdOffset (flowIdOffset);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPort0FlowIdOffset (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  flowIdOffset
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPort0RxMaxLen
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port 0 Receive Maximum Length
 *      Register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxMaxLen;

        rxMaxLen    =   CSL_CPSW_getPort0RxMaxLen ();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getPort0RxMaxLen (CSL_Xge_cpswRegs *hCpswRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_setPort0RxMaxLen
 *
 *   @b Description
 *   @n This function sets up the Port0 Receive Maximum length register.
 *
 *   @b Arguments
     @verbatim
        rxMaxLen            Maximum receive frame length to configure.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxMaxLen;

        rxMaxLen    =   1518;

        CSL_CPSW_setPort0RxMaxLen (rxMaxLen);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPort0RxMaxLen (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  rxMaxLen
);



/** ============================================================================
 *   @n@b CSL_CPSW_getPortBlockCountReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Block Count register
 *      corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the block count
                                must be retrieved.
        pRxBlkCnt_e             Receive block count usage read for express MAC for this port.
        pRxBlkCnt_p             Receive block count usage read for preempt MAC for this port.
        pTxBlkCnt               Transmit block count usage read for this port.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_E,
 *      XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_P,
 *      XGE_CPSW_PN_BLK_CNT_REG_TX_BLK_CNT,
 *      XGE_CPSW_P0_BLK_CNT_REG_RX_BLK_CNT,
 *      XGE_CPSW_P0_BLK_CNT_REG_TX_BLK_CNT,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxBlkCnt_e, rxBlkCnt_p, txBlkCnt, portNum;

        portNum =   1;

        CSL_CPSW_getPortBlockCountReg (portNum, &rxBlkCnt, &rxBlkCnt_p,
                                       &txBlkCnt);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortBlockCountReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pRxBlkCnt_e,
    Uint32*                     pRxBlkCnt_p,
    Uint32*                     pTxBlkCnt);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortRxMaxLen
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW Port Receive Maximum Length
 *      Register.
 *
 *   @b Arguments
        portNum             CPSW port number for which the Receive Maximum Length
                            must be retrieved.
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN
 *      XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, rxMaxLen;

        portNum = 1;
        rxMaxLen    =   CSL_CPSW_getPortRxMaxLen (portNum);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getPortRxMaxLen (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortRxMaxLen
 *
 *   @b Description
 *   @n This function sets up the Port Receive Maximum length register.
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the Receive Maximum Length
                            must be retrieved.
        rxMaxLen            Maximum receive frame length to configure.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN
 *      XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, rxMaxLen;

        portNum     =   1;
        rxMaxLen    =   1518;

        CSL_CPSW_setPortRxMaxLen (portNum, rxMaxLen);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortRxMaxLen (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32                  rxMaxLen
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortTxPriMapReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Transmit Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the block count
                                must be retrieved.
        pPortRxPriMap           Array of Port Rx priority map priority values
                                read from the register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pPortRxPriMap' must be large enough to hold all
 *       the 8 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum = 1;
 *      Uint32      portRxPriMap [8];

        CSL_CPSW_getPortTxPriMapReg (portNum, portRxPriMap);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTxPriMapReg (
    CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortTxPriMap
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortTxPriMapReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port Transmit Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the priority mapping
                                registers must be configured.
        pPortRxPriMap           Array of Port Rx priority map priority values
                                that must be configured to the register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum = 1;
 *      Uint32      i, port0RxPriMap [8];

        for (i = 0; i < 8; i ++)
            port0TxPriMap [i] = i;

        CSL_CPSW_setPortTxPriMapReg (port0TxPriMap);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTxPriMapReg (
    CSL_Xge_cpswRegs           *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortTxPriMap
);



/** ============================================================================
 *   @n@b CSL_CPSW_getPortRxPriMapReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Receive Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the block count
                                must be retrieved.
        pPortRxPriMap           Array of Port Rx priority map priority values
                                read from the register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pPortRxPriMap' must be large enough to hold all
 *       the 8 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum = 1;
 *      Uint32      portRxPriMap [8];

        CSL_CPSW_getPortRxPriMapReg (portNum, portRxPriMap);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortRxPriMapReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortRxPriMap
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortRxPriMapReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port Receive Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the priority mapping
                                registers must be configured.
        pPortRxPriMap           Array of Port Rx priority map priority values
                                that must be configured to the register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum = 1;
 *      Uint32      i, port0RxPriMap [8];

        for (i = 0; i < 8; i ++)
            port0RxPriMap [i] = i;

        CSL_CPSW_setPortRxPriMapReg (port0RxPriMap);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortRxPriMapReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortRxPriMap
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortRxDscpMap
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port DSCP to Priority
 *      Mapping Registers corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the DSCP mapping
                                registers must be retrieved.
        pRxDscpPriMap           Array of Port Rx DSCP to priority mapping values
                                read from the registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pRxDscpPriMap' must be large enough to hold all
 *       the 64 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxDscpPriMap [64], portNum;

        portNum = 1;
        CSL_CPSW_getPortRxDscpMap (portNum, rxDscpPriMap);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortRxDscpMap (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pRxDscpPriMap
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortRxDscpMap
 *
 *   @b Description
 *   @n This function sets up the contents of the Port DSCP to Priority
 *      Mapping Registers corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the DSCP mapping
                                registers must be configured.
        pRxDscpPriMap           Array of Port Rx DSCP to priority mapping values
                                that must be configured to the registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pRxDscpPriMap' must be large enough to hold all
 *       the 64 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxDscpPriMap [64], portNum;

        portNum = 1;

        for (i = 0; i < 64; i ++)
            port0RxPriMap [i] = i/8;

        CSL_CPSW_setPortRxDscpMap (portNum, rxDscpPriMap);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortRxDscpMap (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pRxDscpPriMap
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortRxCutThruPri
 *
 *   @b Description
 *   @n This function sets up the RX Cut thru priority
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the RX Cut thru priority
                            must be set.
        pPortRxCutThruPri   RX Cut thru priority
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_CUT_THRU_REG_RX_PRI_CUT_THRU_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortRxCutThruPri;

        portNum           = 1;
        pPortRxCutThruPri = 1;

        CSL_CPSW_setPortRxCutThruPri (portNum, pPortRxCutThruPri);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortRxCutThruPri (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      pPortRxCutThruPri
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortRxCutThruPri
 *
 *   @b Description
 *   @n This function gets the RX Cut thru priority
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the RX Cut thru priority
                            must be retrieved.
        pPortRxCutThruPri   RX Cut thru priority
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_CUT_THRU_REG_RX_PRI_CUT_THRU_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortRxCutThruPri;

        portNum           = 1;

        pPortRxCutThruPri = CSL_CPSW_getPortRxCutThruPri (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortRxCutThruPri (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortRxCutThruPri
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortTxCutThruPri
 *
 *   @b Description
 *   @n This function sets up the TX Cut thru priority
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the TX Cut thru priority
                            must be set.
        pPortTxCutThruPri   TX Cut thru priority
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_CUT_THRU_REG_TX_PRI_CUT_THRU_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortRxCutThruPri;

        portNum           = 1;
        pPortTxCutThruPri = 1;

        CSL_CPSW_setPortTxCutThruPri (portNum, pPortTxCutThruPri);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTxCutThruPri (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      pPortTxCutThruPri
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortTxCutThruPri
 *
 *   @b Description
 *   @n This function gets the TX Cut thru priority
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the TX Cut thru priority
                            must be retrieved.
        pPortRxCutThruPri   TX Cut thru priority
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_CUT_THRU_REG_TX_PRI_CUT_THRU_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortTxCutThruPri;

        portNum           = 1;

        pPortTxCutThruPri = CSL_CPSW_getPortTxCutThruPri (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTxCutThruPri (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortTxCutThruPri
);

/** ============================================================================
 *   @n@b CSL_CPSW_setPortSpeedReg
 *
 *   @b Description
 *   @n This function sets up the port speed
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the speed
                            must be set.
        pPortSpeed          Port Speed
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_SPEED_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortSpeed;

        portNum           = 1;
        pPortSpeed        = 1;

        CSL_CPSW_setPortSpeedReg (portNum, pPortSpeed);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortSpeedReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      pPortSpeed
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortSpeedReg
 *
 *   @b Description
 *   @n This function gets the port speed
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the port speed
                            must be retrieved.
        pPortSpeed          Port Speed
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_SPEED_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortSpeed;

        portNum           = 1;

        pPortSpeed = CSL_CPSW_getPortSpeedReg (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortSpeedReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortSpeed
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortSpeedAutoEnable
 *
 *   @b Description
 *   @n This function sets up the port's speed for auto enable
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the speed
                            must be auto enabled.
        pPortSpeedAutoEn    Port Speed
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_SPEED_REG_AUTO_ENABLE
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortSpeedAutoEn;

        portNum           = 1;
        pPortSpeedAutoEn  = 1;

        CSL_CPSW_setPortSpeedAutoEnable (portNum, pPortSpeedAutoEn);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortSpeedAutoEnable (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    bool                        pPortSpeedAutoEn
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortSpeedAutoEnable
 *
 *   @b Description
 *   @n This function gets the auto enable speed status of the port
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the auto enable status
                            must be retrieved.
        pPortSpeed          Port Speed
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_SPEED_REG_AUTO_ENABLE
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortSpeedAutoEn;

        portNum           = 1;

        pPortSpeedAutoEn = CSL_CPSW_getPortSpeedAutoEnable (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortSpeedAutoEnable (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    bool*                       pPortSpeedAutoEn
);


/** ============================================================================
 *   @n@b CSL_CPSW_getEEEGlobConfig
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW EEE Global Configuration.
 *
 *   @b Arguments
     @verbatim
        pGlobConfig             CSL_CPSW_EEE_GLOB_CONFIG structure that needs to be populated
                                with the contents of the corresponging EEE global control
                                registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE
 *      XGE_CPSW_CONTROL_REG_EEE_ENABLE
 *
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_EEE_GLOB_CONFIG    globConfig;

        CSL_CPSW_getEEEGlobConfig (&globConfig);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getEEEGlobConfig (CSL_Xge_cpswRegs *hCpswRegs,
    CSL_CPSW_EEE_GLOB_CONFIG*   pGlobConfig
);


/** ============================================================================
 *   @n@b CSL_CPSW_setEEEGlobConfig
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW EEE related global registers
 *      per user-specified EEE Global Configuration.
 *
 *   @b Arguments
     @verbatim
        pGlobConfig         CSL_CPSW_EEE_GLOB_CONFIG structure that holds the values
                            that need to be configured to the EEE global control
                            registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSW EEE Global control register modified with values provided.
 *
 *   @b Writes
 *   @n XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE
 *      XGE_CPSW_CONTROL_REG_EEE_ENABLE
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_EEE_GLOB_CONFIG    globConfig;

        globConfig.enable       =   1;
        globalConfig.prescale   =   100;
        ...

        CSL_CPSW_setEEEGlobConfig (&globConfig);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setEEEGlobConfig (CSL_Xge_cpswRegs *hCpswRegs,
    CSL_CPSW_EEE_GLOB_CONFIG*   pGlobConfig
);


/** ============================================================================
 *   @n@b CSL_CPSW_getEEEPortConfig
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW EEE Port Configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the EEE Port Control
                                registers must be retrieved.
        pPortConfig             CSL_CPSW_EEE_PORT_CONFIG structure that needs to be populated
                                with the contents of the corresponging EEE port-specific control
                                registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_IDLE2LPI_REG_COUNT
 *      XGE_CPSW_P0_LPI2WAKE_REG_COUNT
 *
 *      XGE_CPSW_PN_IDLE2LPI_REG_COUNT
 *      XGE_CPSW_PN_LPI2WAKE_REG_COUNT
 *      XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN
 *
 *   @b Example
 *   @verbatim
        Uint32                      portNum;
        CSL_CPSW_EEE_PORT_CONFIG    portConfig;

        portNum = 1;
        CSL_CPSW_getEEEPortConfig (portNum, &portConfig);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getEEEPortConfig (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPSW_EEE_PORT_CONFIG*   pPortConfig
);


/** ============================================================================
 *   @n@b CSL_CPSW_setEEEPortConfig
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW EEE port-specific control
 *      registers per user-specified EEE Port Configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the EEE Port Control
                                registers must be configured.
        pPortConfig             CSL_CPSW_EEE_PORT_CONFIG structure holds the value
                                that needs to be configured to the EEE port-specific
                                control registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_IDLE2LPI_REG_COUNT
 *      XGE_CPSW_P0_LPI2WAKE_REG_COUNT
 *
 *      XGE_CPSW_PN_IDLE2LPI_REG_COUNT
 *      XGE_CPSW_PN_LPI2WAKE_REG_COUNT
 *      XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN
 *
 *   @b Example
 *   @verbatim
        Uint32                      portNum;
        CSL_CPSW_EEE_PORT_CONFIG    portConfig;

        portNum = 1;
        portConfig.idle2lpi = 10;
        portConfig.lpi2wake = 10;
        portConfig.txLpiClkstopEnable = 1;
        CSL_CPSW_setEEEPortConfig (portNum, &portConfig);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setEEEPortConfig (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPSW_EEE_PORT_CONFIG*   pPortConfig
);


/** ============================================================================
 *   @n@b CSL_CPSW_getEEEPortStatus
 *
 *   @b Description
 *   @n This function retrieves the contents of the EEE port-specific Status register
 *      corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the EEE status
                                must be retrieved.
        pPortStatus             CSL_CPSW_EEE_PORT_STATUS structure holds the EEE Port Status.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_EEE_STATUS_REG_WAIT_IDLE2LPI,
 *      XGE_CPSW_P0_EEE_STATUS_REG_RX_LPI,
 *      XGE_CPSW_P0_EEE_STATUS_REG_TX_LPI,
 *      XGE_CPSW_P0_EEE_STATUS_REG_TX_WAKE,
 *      XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_HOLD,
 *      XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_EMPTY,
 *      XGE_CPSW_P0_EEE_STATUS_REG_RX_FIFO_EMPTY,
 *
 *      XGE_CPSW_PN_EEE_STATUS_REG_WAIT_IDLE2LPI,
 *      XGE_CPSW_PN_EEE_STATUS_REG_RX_LPI,
 *      XGE_CPSW_PN_EEE_STATUS_REG_TX_LPI,
 *      XGE_CPSW_PN_EEE_STATUS_REG_TX_WAKE,
 *      XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_HOLD,
 *      XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_EMPTY,
 *      XGE_CPSW_PN_EEE_STATUS_REG_RX_FIFO_EMPTY
 *
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32                      portNum;
        CSL_CPSW_EEE_PORT_STATUS    portStatus;

        portNum =   1;

        CSL_CPSW_getEEEPortStatus (portNum, &portStatus);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_EEEPortStatus (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPSW_EEE_PORT_STATUS*   pPortStatus
);



/** ============================================================================
 *   @n@b CSL_CPSW_getPortVlanReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the VLAN Register corresponding
 *      to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the VLAN register
                                contents must be read
        pPortVID                Port VLAN Id
        pPortCFI                Port CFI bit
        pPortPRI                Port VLAN priority (0-7, 7 is highest priority)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI,
 *
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portVID, portCFI, portPRI, portNum;

        portNum =   1;

        CSL_CPSW_getPortVlanReg (portNum, &portVID, &portCFI, &portPRI);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortVlanReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortVID,
    Uint32*                     pPortCFI,
    Uint32*                     pPortPRI
);



/** ============================================================================
 *   @n@b CSL_CPSW_setPortVlanReg
 *
 *   @b Description
 *   @n This function sets up the contents of the VLAN Register corresponding to
 *      the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the VLAN register
                                must be configured.
        portVID                 Port VLAN Id to be configured
        portCFI                 Port CFI bit to be configured
        portPRI                 Port VLAN priority to be configured
                                (0-7, 7 is highest priority)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI,
 *
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portVID, portCFI, portPRI, portNum;

        portNum     =   1;
        portVID     =   1;
        portCFI     =   0;
        portPRI     =   7;

        CSL_CPSW_setPortVlanReg (portNum, portVID, portCFI, portPRI);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortVlanReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      portVID,
    Uint32                      portCFI,
    Uint32                      portPRI
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortMaxBlksReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Maxmium Block register
 *      corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the max block value
                                must be retrieved.
        pRxMaxBlks              Receive FIFO Maximum Blocks - This value is the
                                maximum number of 1k memory blocks that may be
                                allocated to the FIFO's logical receive queue.
                                This value must be greater than or equal to 0x3.
                                The recommended value of rx_max_blks is 0x9

        pTxMaxBlks              Transmit FIFO Maximum Blocks - This value is the
                                maximum number of 1k memory blocks that may be
                                allocated to the FIFO's logical transmit
                                priority queues.  The recommended value of
                                tx_max_blks is 0x3.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS,
 *      XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxMaxBlks, txMaxBlks, portNum;

        portNum =   1;

        CSL_CPSW_getPortMaxBlksReg (portNum, &rxMaxBlks, &txMaxBlks);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortMaxBlksReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pRxMaxBlks,
    Uint32*                     pTxMaxBlks
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortMaxBlksReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port Maxmium Block register
 *      corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the max block value
                                must be retrieved.
        rxMaxBlks               Receive FIFO Maximum Blocks - This value is the
                                maximum number of 1k memory blocks that may be
                                allocated to the FIFO's logical receive queue.
                                This value must be greater than or equal to 0x3.
                                The recommended value of rx_max_blks is 0x9

        txMaxBlks               Transmit FIFO Maximum Blocks - This value is the
                                maximum number of 1k memory blocks that may be
                                allocated to the FIFO's logical transmit
                                priority queues.  The recommended value of
                                tx_max_blks is 0x3.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS,
 *      XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxMaxBlks, txMaxBlks, portNum;

        portNum =   1;

        CSL_CPSW_setPortMaxBlksReg (portNum, rxMaxBlks, txMaxBlks);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortMaxBlksReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      rxMaxBlks,
    Uint32                      txMaxBlks
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortMACAddress
 *
 *   @b Description
 *   @n This function retreives the source MAC address of the Tx Pause Frame corresponding to the
 *      CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the source MAC address
                                must be read and returned. (1-8)
        pMacAddress             6 byte Source MAC address read.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pMacAddres' must be large enough the 6 byte
 *       MAC address returned by this API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0,
 *      XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40
 *
 *   @b Example
 *   @verbatim
 *      Uint8   macAddress [6], portNum;

        portNum =   1;

        CSL_CPSW_getPortMACAddress (portNum, macAddress);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortMACAddress (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint8*                      pMacAddress
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortMACAddress
 *
 *   @b Description
 *   @n This function sets up the source MAC address the Tx Pause Frame corresponding to the
 *      CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the source MAC address
                                must be setup. (1-8)
        pMacAddress             6 byte Source MAC address to configure.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pMacAddres' is expected to be 6 bytes long.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0,
 *      XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40
 *
 *   @b Example
 *   @verbatim
 *      Uint8   macAddress [6], portNum;

        portNum         =   1;
        macAddress [0]  =   0x01;
        macAddress [1]  =   0x02;
        macAddress [2]  =   0x03;
        macAddress [3]  =   0x04;
        macAddress [4]  =   0x05;
        macAddress [5]  =   0x06;

        CSL_CPSW_setPortMACAddress (portNum, macAddress);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortMACAddress (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint8*                      pMacAddress
);



/** ============================================================================
 *   @n@b CSL_CPSW_getPortTimeSyncCntlReg
 *
 *   @b Description
 *   @n This function retreives the contents of Time sync control register
 *      corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be read. (1-8)
        pTimeSyncCntlCfg        CSL_CPSW_TSCNTL that needs to be populated with
                                contents of time sync control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_TSCNTL     tsCtlCfg;

        portNum =   1;

        CSL_CPSW_getPortTimeSyncCntlReg (portNum, &tsCtlCfg);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTimeSyncCntlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    CSL_CPSW_TSCNTL*        pTimeSyncCntlCfg
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortTimeSyncCntlReg
 *
 *   @b Description
 *   @n This function sets up the contents of Time sync control register
 *      corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be
                                configured.
        pTimeSyncCntlCfg        CSL_CPSW_TSCNTL containing settings for time
                                sync control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_TSCNTL     tsCtlCfg;

        portNum =   1;

        tsCtlCfg.tsRxVlanLType1Enable   =   0;
        tsCtlCfg.tsRxVlanLType2Enable   =   0;
        ...

        CSL_CPSW_setPortTimeSyncCntlReg (portNum, &tsCtlCfg);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTimeSyncCntlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    CSL_CPSW_TSCNTL*        pTimeSyncCntlCfg
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortTimeSyncSeqIdReg
 *
 *   @b Description
 *   @n This function retreives the contents of Time Sync Sequence Id and
 *      LTYPE register corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be read. (1-8)
        pTsLtype                Time sync LTYPE read.
        pTsSeqIdOffset          Time sync sequence Id offset read.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1,
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype, tsSeqIdOffset;

        portNum =   1;

        CSL_CPSW_getPortTimeSyncSeqIdReg (portNum, &tsLtype, &tsSeqIdOffset);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTimeSyncSeqIdReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pTsLtype,
    Uint32*                     pTsSeqIdOffset
);


void CSL_CPSW_getVlanLTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pVlanLtypeInner,
    Uint32*                     pVlanLtypeOuter
);


/** ============================================================================
 *   @n@b CSL_CPSW_setVlanLTypeReg
 *
 *   @b Description
 *   @n This function retreives the contents of VLAN_LTYPE_REG
 *      register.
 *
 *   @b Arguments
     @verbatim
        pVlanLtype1           VLAN LTYPE1 value read.
        pVlanLtype2           VLAN LTYPE2 value read.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_INNER,
 *      XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype1, tsLtype2;

        portNum =   1;

        CSL_CPSW_setVlanLTypeReg (portNum, &tsLtype1, &tsLtype2);
     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setVlanLTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                     pVlanLtypeInner,
    Uint32                     pVlanLtypeOuter
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortTimeSyncSeqIdReg
 *
 *   @b Description
 *   @n This function sets up the contents of Time Sync Sequence Id and
 *      LTYPE register corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be
                                configured. (1-8)
        tsLtype                 Time sync LTYPE to be configured.
        tsSeqIdOffset           Time sync sequence Id offset to be configured.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1,
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype, tsSeqIdOffset;

        portNum         =   1;
        tsLtype         =   0;
        tsSeqIdOffset   =   30;

        CSL_CPSW_getPortTimeSyncSeqIdReg (portNum, tsLtype, tsSeqIdOffset);
     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTimeSyncSeqIdReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      tsLtype,
    Uint32                      tsSeqIdOffset
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortTimeSyncVlanLTypeReg
 *
 *   @b Description
 *   @n This function retreives the contents of Time Sync VLAN LTYPE
 *      register corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be read. (1-8)
        pTsVlanLtype1           Time sync VLAN LTYPE1 value read.
        pTsVlanLtype2           Time sync VLAN LTYPE2 value read.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1,
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype1, tsLtype2;

        portNum =   1;

        CSL_CPSW_getPortTimeSyncVlanLTypeReg (portNum, &tsLtype1, &tsLtype2);
     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTimeSyncVlanLTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pTsVlanLtype1,
    Uint32*                     pTsVlanLtype2
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortTimeSyncVlanLTypeReg
 *
 *   @b Description
 *   @n This function sets up the contents of Time Sync VLAN LTYPE
 *      register corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be read. (1-8)
        tsVlanLtype1            Time sync VLAN LTYPE1 value to be configured.
        tsVlanLtype2            Time sync VLAN LTYPE2 value to be configured.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1,
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype1, tsLtype2;

        portNum     =   1;
        tsLtype1    =   0x8100;
        tsLtype2    =   0x8100;

        CSL_CPSW_setPortTimeSyncVlanLTypeReg (portNum, &tsLtype1, &tsLtype2);
     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTimeSyncVlanLTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      tsVlanLtype1,
    Uint32                      tsVlanLtype2
);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortTimeSyncCntlReg
 *
 *   @b Description
 *   @n This function retreives the contents of Time sync configuration from
 *      time sync control registers corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be read. (1-8)
        pTimeSyncConfig         CSL_CPSW_TSCONFIG that needs to be populated with
                                contents of time sync control registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN,
 *
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1,
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET,
 *
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1,
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2,
 *
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_330,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN,
 *
 *      XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN,
 *      XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_TSCONFIG   tsConfig;

        portNum =   1;

        CSL_CPSW_getPortTimeSyncCntlReg (portNum, &tsConfig);

     @endverbatim
 * =============================================================================
 */

void CSL_CPSW_getPortTimeSyncConfig (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    CSL_CPSW_TSCONFIG*      pTimeSyncConfig
);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortTimeSyncConfig
 *
 *   @b Description
 *   @n This function sets up the contents of Time sync control registers
 *      corresponding to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        pTimeSyncConfig         CSL_CPSW_TSCONFIG containing settings for time
                                sync control registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN
 *
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1,
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET,
 *
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1,
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2,
 *
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_330,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN,
 *
 *      XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN,
 *      XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_TSCONFIG   tsConfig;

        portNum =   1;

        tsConfig.tsRxVlanLType1Enable   =   0;
        tsConfig.tsRxVlanLType2Enable   =   0;
        ...

        CSL_CPSW_setPortTimeSyncConfig (portNum, &tsConfig);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTimeSyncConfig (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    CSL_CPSW_TSCONFIG*        pTimeSyncConfig
);

/** ============================================================================
 *   @n@b CSL_CPSW_getEstTsDomain
 *
 *   @b Description
 *   @n This function retrieves the value used as the domain in the CPTS EST
 *      timestamp events.
 *
 *   @b Arguments
 *   @n  None
 *
 *   <b> Return Value </b>
 *   @n  Current EST timestmap domain.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_EST_TS_DOMAIN_REG_EST_TS_DOMAIN
 *
 *   @b Example
 *   @verbatim
 *      Uint8              domain;

        domain = CSL_CPSW_getEstTsDomain();

     @endverbatim
 */
Uint8 CSL_CPSW_getEstTsDomain(CSL_Xge_cpswRegs   *hCpswRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_setEstTsDomain
 *
 *   @b Description
 *   @n This function writes the value used as the domain in the CPTS EST
 *      timestamp events.
 *
 *   @b Arguments
     @verbatim
        domain              Domain to be used in CPTS EST timestamp events.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_EST_TS_DOMAIN_REG_EST_TS_DOMAIN
 *
 *   @b Example
 *   @verbatim
 *      Uint8              domain;

        domain = 0;
        CSL_CPSW_setEstTsDomain(domain);

     @endverbatim
 */
void CSL_CPSW_setEstTsDomain(CSL_Xge_cpswRegs    *hCpswRegs,
                             Uint8                domain);


/** ============================================================================
 *   @n@b CSL_CPSW_getPortEstConfig
 *
 *   @b Description
 *   @n This function retrieves the contents of EST control register
 *      corresponding to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                read.
        pEstConfig              CSL_CPSW_EST_CONFIG containing settings for
                                port's EST control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_ONEBUF
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_BUFSEL
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_EN
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_FIRST
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_ONEPRI
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_PRI
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_EN
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_MARGIN
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_EST_CONFIG estConfig;

        portNum = 1;

        CSL_CPSW_getEstConfig(portNum, &estConfig);

     @endverbatim
 */
void CSL_CPSW_getPortEstConfig(CSL_Xge_cpswRegs    *hCpswRegs,
                               Uint32              portNum,
                               CSL_CPSW_EST_CONFIG *pEstConfig);

/** ============================================================================
 *   @n@b CSL_CPSW_setPortEstConfig
 *
 *   @b Description
 *   @n This function sets up the contents of EST control register corresponding
 *      to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        pEstConfig              CSL_CPSW_EST_CONFIG containing settings for
                                port's EST control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_ONEBUF
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_BUFSEL
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_EN
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_FIRST
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_ONEPRI
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_PRI
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_EN
        CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_MARGIN
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_EST_CONFIG estConfig;

        portNum = 1;

        estConfig.estOneBuf = 1;
        estConfig.estBufSel = 1;
        ...

        CSL_CPSW_setEstConfig(portNum, &estConfig);

     @endverbatim
 */
void CSL_CPSW_setPortEstConfig(CSL_Xge_cpswRegs    *hCpswRegs,
                               Uint32              portNum,
                               CSL_CPSW_EST_CONFIG *pEstConfig);

/** ============================================================================
 *   @n@b CSL_CPSW_writeEstFetchCmd
 *
 *   @b Description
 *   @n This function writes an EST command comprised of fetch count (EST time
 *      interval) and fetch allow (gate mask) values to CPSW EST buffer.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        index                   EST buffer entry index (0 to 127).
        fetchCount              Fetch time in Ethernet wireside clocks.
        fetchAllow              8-bit gate mask.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CSL_XGE_CPSW_CPSW_NU_EST_FETCH_LOC
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        Uint32              fetchCount;
        Uint8               fetchAllow;

        portNum = 1;
        index   = 0;
        fetchCount = 64;
        fetchAllow = 0xC0;

        CSL_CPSW_writeEstFetchCmd(portNum, index, fetchCount, fetchAllow);

     @endverbatim
 */
void CSL_CPSW_writeEstFetchCmd(CSL_Xge_cpswRegs    *hCpswRegs,
                               Uint32              portNum,
                               Uint32              index,
                               Uint32              fetchCount,
                               Uint8               fetchAllow);

/** ============================================================================
 *   @n@b CSL_CPSW_readEstFetchCmd
 *
 *   @b Description
 *   @n This function read the EST command comprised of fetch count (EST time
 *      interval) and fetch allow (gate mask) values from CPSW EST buffer.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        index                   EST buffer entry index (0 to 127).
        fetchCount              Fetch time in wireside clocks to be populated
                                from EST buffer.
        fetchAllow              8-bit gate mask tobe populated from EST buffer
                                value.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CSL_XGE_CPSW_CPSW_NU_EST_FETCH_LOC
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        Uint32              index;
        Uint32              fetchCount;
        Uint8               fetchAllow;

        portNum = 1;
        index   = 0;

        CSL_CPSW_readEstFetchCmd(portNum, index, &fetchCount, &fetchAllow);

     @endverbatim
 */
void CSL_CPSW_readEstFetchCmd(CSL_Xge_cpswRegs    *hCpswRegs,
                              Uint32              portNum,
                              Uint32              index,
                              Uint32              *fetchCount,
                              Uint8               *fetchAllow);

/** ============================================================================
 *   @n@b CSL_CPSW_getPortIetControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of IET control register
 *      corresponding to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                read.
        pIetConfig              CSL_CPSW_IET_CONFIG containing settings for
                                port's IET control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_LINKFAIL
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_DISABLEVERIFY
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_IET_CONFIG ietConfig;

        portNum = 1;

        CSL_CPSW_getPortIetControlReg(portNum, &ietConfig);

     @endverbatim
 */
void CSL_CPSW_getPortIetControlReg(CSL_Xge_cpswRegs    *hCpswRegs,
                                   Uint32              portNum,
                                   CSL_CPSW_IET_CONFIG *pIetConfig);

/** ============================================================================
 *   @n@b CSL_CPSW_setPortIetControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of IET control register corresponding
 *      to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        pEstConfig              CSL_CPSW_IET_CONFIG containing settings for
                                port's IET control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_LINKFAIL
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_DISABLEVERIFY
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD
        CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_IET_CONFIG ietConfig;

        portNum = 1;

        ietConfig.macPremptQueue = 1;
        ietConfig.macPremptEnable = 1;
        ...

        CSL_CPSW_setPortIetControlReg(portNum, &ietConfig);

     @endverbatim
 */
void CSL_CPSW_setPortIetControlReg(CSL_Xge_cpswRegs    *hCpswRegs,
                                   Uint32              portNum,
                                   CSL_CPSW_IET_CONFIG *pIetConfig);

/** ============================================================================
 *   @n@b CSL_CPSW_getPortIetVerifyTimeout
 *
 *   @b Description
 *   @n This function reads the contents of IET Verify timeout register corresponding
 *      to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        pIetVerifyTimeout       Number of wireside clocks contained in the verify timeout
                                counter.

 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CSL_XGE_CPSW_PN_IET_VERIFY_REG_MAC_VERIFY_CNT
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        Uint32              ietVerifyTimeout;

        portNum = 1;

        CSL_CPSW_getPortIetVerifyTimeout(portNum, &ietVerifyTimeout);

     @endverbatim
 */
void CSL_CPSW_getPortIetVerifyTimeout(CSL_Xge_cpswRegs    *hCpswRegs,
                                      Uint32              portNum,
                                      Uint32              *pIetVerifyTimeout);


/** ============================================================================
 *   @n@b CSL_CPSW_setPortIetVerifyTimeout
 *
 *   @b Description
 *   @n This function sets the contents of IET Verify timeout register corresponding
 *      to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        pIetVerifyTimeout       Number of wireside clocks contained in the verify timeout
                                counter.

 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n CSL_XGE_CPSW_PN_IET_VERIFY_REG_MAC_VERIFY_CNT
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        Uint32              ietVerifyTimeout;

        portNum = 1;

        ietVerifyTimeout = 0x1312d0;

        CSL_CPSW_setPortIetVerifyTimeout(portNum, ietVerifyTimeout);

     @endverbatim
 */
void CSL_CPSW_setPortIetVerifyTimeout(CSL_Xge_cpswRegs    *hCpswRegs,
                                      Uint32              portNum,
                                      Uint32              ietVerifyTimeout);


/** ============================================================================
 *   @n@b CSL_CPSW_PortIetStatus
 *
 *   @b Description
 *   @n This function retrieves the contents of the IET status register corresponding
 *      to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        pIetStatus              CSL_CPSW_IET_STATUS containing settings for
                                port's IET status register.

 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFIED
 *      CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_FAIL
 *      CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_RESPOND_ERR
 *      CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_ERR
 *
 *   @b Example
 *   @verbatim
 *      Uint32               portNum;
        CSL_CPSW_IET_STATUS  ietStatus;

        portNum = 1;

        CSL_CPSW_PortIetStatus(portNum, &ietStatus);

     @endverbatim
 */
void CSL_CPSW_PortIetStatus(CSL_Xge_cpswRegs     *hCpswRegs,
                            Uint32               portNum,
                            CSL_CPSW_IET_STATUS  *pIetStatus);


/********************************************************************************
*************************  Statistics (STATS) Submodule *************************
********************************************************************************/

/** ============================================================================
 *   @n@b CSL_CPSW_getStats
 *
 *   @b Description
 *   @n The CPSW stats are divided into 9 blocks, i.e., Stats for Host port (switch Port 0)
 *      and Stats for CPSW ports (Port 1-8 ). This function
 *          - retreives hardware statistics for both the stat blocks.
 *          - Clear out the stats by the count being returned to application
 *          - Accumulates the stats count before returning to Application
 *      Function requires appplication to memset the stats (once before first
 *            use for accumulator, or once per use without accumulation)
 *      In the case of Linux ARM master use case all CPSW stats is recommended
 *      to be accessed from Linux.
 *      This function unconditionally clears the stats, so it requires the
 *      caller have exclusive ownership of the switch.  Otherwise, none of the
 *      callers (including Linux) will have complete accumulated stats.
 *      This function can be used to clear the stats by memesetting pCpswStats
 *      to 0 and discarding the returned stats.
 *
 *
 *   @b Arguments
     @verbatim
        pCpswStats              Union of CSL_CPSW_STATS structure that needs to be filled
                                with the stats read from the hardware. This function expects
                                that the array passed to it is big enough to hold the stats
                                for all stat blocks, i.e., size of array passed to this
                                function must be 5 or 9 for 5/9 port switch respectively.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSW_RXGOODFRAMES,
 *      CPSW_RXBROADCASTFRAMES,
 *      CPSW_RXMULTICASTFRAMES,
 *      CPSW_RXPAUSEFRAMES,
 *      CPSW_RXCRCERRORS,
 *      CPSW_RXALIGNCODEERRORS,
 *      CPSW_RXOVERSIZEDFRAMES,
 *      CPSW_RXJABBERFRAMES,
 *      CPSW_RXUNDERSIZEDFRAMES,
 *      CPSW_RXFRAGMENTS,
 *      CPSW_ALE_DROP,
 *      CPSW_ALE_OVERRUN_DROP,
 *      CPSW_RXOCTETS,
 *      CPSW_TXGOODFRAMES,
 *      CPSW_TXBROADCASTFRAMES,
 *      CPSW_TXMULTICASTFRAMES,
 *      CPSW_TXPAUSEFRAMES,
 *      CPSW_TXDEFERREDFRAMES,
 *      CPSW_TXCOLLISIONFRAMES,
 *      CPSW_TXSINGLECOLLFRAMES,
 *      CPSW_TXMULTCOLLFRAMES,
 *      CPSW_TXEXCESSIVECOLLISIONS,
 *      CPSW_TXLATECOLLISIONS,
 *      CPSW_TXUNDERRUN,
 *      CPSW_TXCARRIERSENSEERRORS,
 *      CPSW_TXOCTETS,
 *      CPSW_OCTETFRAMES64,
 *      CPSW_OCTETFRAMES65T127,
 *      CPSW_OCTETFRAMES128T255,
 *      CPSW_OCTETFRAMES256T511,
 *      CPSW_OCTETFRAMES512T1023,
 *      CPSW_OCTETFRAMES1024TUP,
 *      CPSW_NETOCTETS,
 *      CPSW_RX_BOTTOM_OF_FIFO_DROP,
 *      CPSW_PORTMASK_DROP,
 *      CPSW_RX_TOP_OF_FIFO_DROP,
 *      CPSW_ALE_ALE_RATE_LIMIT_DROP,
 *      CPSW_ALE_VID_INGRESS_DROP,
 *      CPSW_ALE_DA_EQ_SA_DROP,
 *      CPSW_ALE_UNKN_UNI,
 *      CPSW_ALE_UNKN_UNI_BCNT,
 *      CPSW_ALE_UNKN_MLT,
 *      CPSW_ALE_UNKN_MLT_BCNT,
 *      CPSW_ALE_UNKN_BRD,
 *      CPSW_ALE_UNKN_BRD_BCNT,
 *      CPSW_ALE_POLL_MATCH,
 *      CPSW_TX_MEMORY_PROTECT_ERROR
 *
 *   @b Affects
 *   @n CPSW_RXGOODFRAMES=0,
 *      CPSW_RXBROADCASTFRAMES=0,
 *      CPSW_RXMULTICASTFRAMES=0,
 *      CPSW_RXPAUSEFRAMES=0,
 *      CPSW_RXCRCERRORS=0,
 *      CPSW_RXALIGNCODEERRORS=0,
 *      CPSW_RXOVERSIZEDFRAMES=0,
 *      CPSW_RXJABBERFRAMES=0,
 *      CPSW_RXUNDERSIZEDFRAMES=0,
 *      CPSW_RXFRAGMENTS=0,
 *      CPSW_ALE_DROP=0,
 *      CPSW_ALE_OVERRUN_DROP=0,
 *      CPSW_RXOCTETS=0,
 *      CPSW_TXGOODFRAMES=0,
 *      CPSW_TXBROADCASTFRAMES=0,
 *      CPSW_TXMULTICASTFRAMES=0,
 *      CPSW_TXPAUSEFRAMES=0,
 *      CPSW_TXDEFERREDFRAMES=0,
 *      CPSW_TXCOLLISIONFRAMES=0,
 *      CPSW_TXSINGLECOLLFRAMES=0,
 *      CPSW_TXMULTCOLLFRAMES=0,
 *      CPSW_TXEXCESSIVECOLLISIONS=0,
 *      CPSW_TXLATECOLLISIONS=0,
 *      CPSW_TXUNDERRUN=0,
 *      CPSW_TXCARRIERSENSEERRORS=0,
 *      CPSW_TXOCTETS=0,
 *      CPSW_OCTETFRAMES64=0,
 *      CPSW_OCTETFRAMES65T127=0,
 *      CPSW_OCTETFRAMES128T255=0,
 *      CPSW_OCTETFRAMES256T511=0,
 *      CPSW_OCTETFRAMES512T1023=0,
 *      CPSW_OCTETFRAMES1024TUP=0,
 *      CPSW_NETOCTETS=0,
 *      CPSW_RX_BOTTOM_OF_FIFO_DROP=0,
 *      CPSW_PORTMASK_DROP=0,
 *      CPSW_RX_TOP_OF_FIFO_DROP=0,
 *      CPSW_ALE_ALE_RATE_LIMIT_DROP=0,
 *      CPSW_ALE_VID_INGRESS_DROP=0,
 *      CPSW_ALE_DA_EQ_SA_DROP=0,
 *      CPSW_ALE_UNKN_UNI=0,
 *      CPSW_ALE_UNKN_UNI_BCNT=0,
 *      CPSW_ALE_UNKN_MLT=0,
 *      CPSW_ALE_UNKN_MLT_BCNT=0,
 *      CPSW_ALE_UNKN_BRD=0,
 *      CPSW_ALE_UNKN_BRD_BCNT=0,
 *      CPSW_ALE_POLL_MATCH=0,
 *      CPSW_TX_MEMORY_PROTECT_ERROR=0
 *
 *   @b Example
 *   @verbatim
 *      CSL_CPSW_STATS     stats [9];

        CSL_CPSW_getStats (stats);
     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getStats (CSL_Xge_cpswRegs *hCpswRegs,
    union CSL_CPSW_STATS*         pCpswStats
);

void CSL_CPSW_getPortStats (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    union CSL_CPSW_STATS*         pCpswStats
);


/** ============================================================================
 *   @n@b CSL_CPSW_getRawStats
 *
 *   @b Description
 *   @n The CPSW stats are divided into 9 blocks, i.e., Stats for Host port (switch Port 0)
 *      and Stats for MAC ports (Port 1-8). This function retreives snapshot of
 *      hardware statistics for all the stat blocks. In the case of Linux ARM master
 *      use case all CPSW stats is recommended to be accessed from Linux.

 *      Since this function does not clear the stats, its possible to have
 *      non-exclusive ownership of the switch and use this function without
 *      corrupting other caller's view of the stats.
 *
 *      Additional Note: In order to avoid stats loss due to rollovers, application
 *      would need to poll the stats by determining the correct interval.
 *      The stat CPSW_NETOCTETS would be first one to roll over
 *      The software must poll and accumulate the stats faster than this rate.
 *      On a 1 gigabit network, it takes approximately
 *      (0x100000000/(1000000000/8)/2)=17 seconds to roll over (the /2 is because
 *      this stat contains both tx and rx, both of which run at gigabit).
 *      A good rule of thumb is to poll at twice this rate (8-9 seconds).
 *
 *      If it is really necessary for application to have multiple nonexclusive
 *      owners of the switch, it is possible for all callers to have a view of
 *      the accumulated statistics if they (including Linux) follows the
 *      differential accumulation of the stats defiened below:
 *      uint64_t accum_CPSW_NETOCTETS;
 *      uint32_t old_CPSW_NETOCTETS, new_CPSW_NETOCTETS, diff_CPSW_NETOCTETS;
 *
 *       diff_CPSW_NETOCTETS = new_CPSW_NETOCTETS - old_CPSW_NETOCTETS; // let rollover occur, no "if" required
 *       old_CPSW_NETOCTETS = new_CPSW_NETOCTETS;
 *       accum_CPSW_NETOCTETS += diff_CPSW_NETOCTETS
 *
 *   @b Arguments
     @verbatim
        pCpswStats              Union of CSL_CPSW_STATS structure that needs to be filled
                                with the stats read from the hardware. This function expects
                                that the array passed to it is big enough to hold the stats
                                for both stat blocks, i.e., size of array passed to this
                                function must be 5 or 9 for 5-port/9-port switch respectively.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSW_RXGOODFRAMES,
 *      CPSW_RXBROADCASTFRAMES,
 *      CPSW_RXMULTICASTFRAMES,
 *      CPSW_RXPAUSEFRAMES,
 *      CPSW_RXCRCERRORS,
 *      CPSW_RXALIGNCODEERRORS,
 *      CPSW_RXOVERSIZEDFRAMES,
 *      CPSW_RXJABBERFRAMES,
 *      CPSW_RXUNDERSIZEDFRAMES,
 *      CPSW_RXFRAGMENTS,
 *      CPSW_ALE_DROP,
 *      CPSW_ALE_OVERRUN_DROP,
 *      CPSW_RXOCTETS,
 *      CPSW_TXGOODFRAMES,
 *      CPSW_TXBROADCASTFRAMES,
 *      CPSW_TXMULTICASTFRAMES,
 *      CPSW_TXPAUSEFRAMES,
 *      CPSW_TXDEFERREDFRAMES,
 *      CPSW_TXCOLLISIONFRAMES,
 *      CPSW_TXSINGLECOLLFRAMES,
 *      CPSW_TXMULTCOLLFRAMES,
 *      CPSW_TXEXCESSIVECOLLISIONS,
 *      CPSW_TXLATECOLLISIONS,
 *      CPSW_TXUNDERRUN,
 *      CPSW_TXCARRIERSENSEERRORS,
 *      CPSW_TXOCTETS,
 *      CPSW_OCTETFRAMES64,
 *      CPSW_OCTETFRAMES65T127,
 *      CPSW_OCTETFRAMES128T255,
 *      CPSW_OCTETFRAMES256T511,
 *      CPSW_OCTETFRAMES512T1023,
 *      CPSW_OCTETFRAMES1024TUP,
 *      CPSW_NETOCTETS,
 *      CPSW_RX_BOTTOM_OF_FIFO_DROP,
 *      CPSW_PORTMASK_DROP,
 *      CPSW_RX_TOP_OF_FIFO_DROP,
 *      CPSW_ALE_ALE_RATE_LIMIT_DROP,
 *      CPSW_ALE_VID_INGRESS_DROP,
 *      CPSW_ALE_DA_EQ_SA_DROP,
 *      CPSW_ALE_UNKN_UNI,
 *      CPSW_ALE_UNKN_UNI_BCNT,
 *      CPSW_ALE_UNKN_MLT,
 *      CPSW_ALE_UNKN_MLT_BCNT,
 *      CPSW_ALE_UNKN_BRD,
 *      CPSW_ALE_UNKN_BRD_BCNT,
 *      CPSW_ALE_POLL_MATCH,
 *      CPSW_TX_MEMORY_PROTECT_ERROR
 *
 *   @b Example
 *   @verbatim
 *      CSL_CPSW_STATS     stats [9];

        CSL_CPSW_getRawStats (stats);
     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getRawStats (CSL_Xge_cpswRegs *hCpswRegs,
    union CSL_CPSW_STATS*         pCpswStats
);


void CSL_CPSW_getPortRawStats (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    union CSL_CPSW_STATS*         pCpswStats
);



/********************************************************************************
********************  Address Lookup Engine (ALE) Submodule *********************
********************************************************************************/

/** ============================================================================
 *   @n@b CSL_CPSW_getAleVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the ALE submodule identification and version
 *      information.
 *
 *   @b Arguments
     @verbatim
        pVersionInfo        CSL_CPSW_ALE_VERSION structure that needs to be populated
                            with the ALE version info read from the hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_MOD_VER_MINOR_REVISION,
 *      ALE_MOD_VER_MAJOR_REVISION,
 *      ALE_MOD_VER_RTL_VERSION,
 *      ALE_MOD_VER_MODULE_ID
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_ALE_VERSION    versionInfo;

        CSL_CPSW_getAleVersionInfo (&versionInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleVersionInfo (CSL_AleRegs *hCpswAleRegs,
    CSL_CPSW_ALE_VERSION*       pVersionInfo
);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleRateLimitEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE Broadcast and Multicast Rate Limit is
 *      enabled.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE Broadcast and multicast rate limit enabled.
 *                              Broadcast/multicast packet reception limited to
 *                              port control register rate limit fields.
 *   @n  FALSE                  ALE Broadcast and multicast rate limit disabled.
 *                              Broadcast/multicast rates not limited.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_RATE_LIMIT
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleRateLimitEnabled () == TRUE)
        {
            // ALE Broadcast/Multicast rate limit enabled
        }
        else
        {
            // ALE Broadcast/Multicast rate limit disabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleRateLimitEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleRateLimit
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable multicast,
 *      broadcast rate limiting.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_RATE_LIMIT=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleRateLimit ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleRateLimit (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleRateLimit
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable multicast,
 *      broadcast rate limiting.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_RATE_LIMIT=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleRateLimit ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleRateLimit (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleMacAuthModeEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE MAC Authorization mode is enabled.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE  is in MAC authorization mode.
 *   @n  FALSE                  ALE not in MAC authorization mode.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_AUTH_MODE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleMacAuthModeEnabled () == TRUE);
        {
            // ALE  is in MAC authorization mode
        }
        else
        {
            // ALE not in MAC authorization mode
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleMacAuthModeEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleMacAuthMode
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable MAC authorization
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_AUTH_MODE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleMacAuthMode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleMacAuthMode (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleMacAuthMode
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable MAC authorization
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_AUTH_MODE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleMacAuthMode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleMacAuthMode (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleVlanAwareEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be VLAN aware.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE VLAN aware. ALE drops packets if VLAN not found.
 *   @n  FALSE                  ALE not VLAN aware. Floods if VLAN not found.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ALE_VLAN_AWARE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleVlanAwareEnabled () == TRUE)
        {
            // ALE VLAN aware
        }
        else
        {
            // ALE not VLAN aware
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleVlanAwareEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleVlanAware
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable VLAN aware
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ALE_VLAN_AWARE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleVlanAware ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleVlanAware (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleVlanAware
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable VLAN aware
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ALE_VLAN_AWARE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleVlanAware ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleVlanAware (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleTxRateLimitEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be Tx-port based multicast,
 *      broadcast rate limited.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE Tx rate limit enabled. Broadcast, multicast
 *                              rate limit counters are transmit port based.
 *   @n  FALSE                  ALE Tx rate limit disabled. Broadcast, multicast
 *                              rate limit counters are receive port based.
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_RATE_LIMIT=1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_BCAST_MCAST_CTL
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleTxRateLimitEnabled () == TRUE)
        {
            // ALE Tx rate limit on
        }
        else
        {
            // ALE Tx rate limit off
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleTxRateLimitEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleTxRateLimit
 *
 *   @b Description
 *   @n This function configures the ALE control register to select Tx-port based
 *      multicast, broadcast rate limiting
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_RATE_LIMIT=1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_BCAST_MCAST_CTL=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleTxRateLimit ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleTxRateLimit (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleTxRateLimit
 *
 *   @b Description
 *   @n This function configures the ALE control register to select Rx-port based
 *      multicast, broadcast rate limiting
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_BCAST_MCAST_CTL=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleTxRateLimit ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleTxRateLimit (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleBypassEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be in Bypass mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE Bypass mode enabled.
 *   @n  FALSE                  ALE Bypass mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_BYPASS
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleBypassEnabled () == TRUE)
        {
            // ALE Bypass mode on
        }
        else
        {
            // ALE Bypass mode off
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleBypassEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleBypass
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable Bypass mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_BYPASS=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleBypass ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleBypass (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleBypass
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable Bypass mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_BYPASS=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleBypass ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleBypass (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleOUIDenyModeEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be in OUI deny mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE OUI deny mode enabled.
 *   @n  FALSE                  ALE OUI deny mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_OUI_DENY
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleOUIDenyModeEnabled () == TRUE)
        {
            // ALE OUI deny mode on
        }
        else
        {
            // ALE OUI deny mode off
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleOUIDenyModeEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleOUIDenyMode
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable OUI deny mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_OUI_DENY=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleOUIDenyMode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleOUIDenyMode (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleOUIDenyMode
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable OUI deny mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_OUI_DENY=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleOUIDenyMode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleOUIDenyMode (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleVID0ModeEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be in VID0 (VLAN ID=0) mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE VID0 mode enabled.
 *                              Process the packet with VLAN Id = 0
 *   @n  FALSE                  ALE VID0 mode disabled. Process the packet with
 *                              VLAN Id =PORT_VLAN[11-0]
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_VID0_MODE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleVID0ModeEnabled () == TRUE)
        {
            // ALE VID0 mode on
        }
        else
        {
            // ALE VID0 mode off
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleVID0ModeEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleVID0Mode
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable VID0 mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_VID0_MODE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleVID0Mode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleVID0Mode (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleVID0Mode
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable VID0 mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_VID0_MODE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleVID0Mode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleVID0Mode (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleLearnNoVIDEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to not learn VLAN Ids.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE Learn no VID enabled.
 *                              VLAN Id is not learned with source address (source
 *                              address is not tied to VID)
 *   @n  FALSE                  ALE VID learning mode enabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_LEARN_NO_VLANID
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleLearnNoVIDEnabled () == TRUE)
        {
            // ALE VID learning disabled
        }
        else
        {
            // ALE VID learning enabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleLearnNoVIDEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleLearnNoVID
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable VLAN Id No
 *      Learn, i.e., disable VLAN Id learning.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_LEARN_NO_VLANID=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleLearnNoVID ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleLearnNoVID (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleLearnNoVID
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable VLAN Id learning.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_LEARN_NO_VLANID=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleLearnNoVID ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleLearnNoVID (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleUUNIToHostEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to forward unkown unicast
 *      packets to host.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   Unknown unicast packets flood to host also.
 *   @n  FALSE                  Unknown unicast packets are dropped to the host.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleUUNIToHostEnabled () == TRUE)
        {
            // ALE Unknown UNI packets forwarded to host
        }
        else
        {
            // ALE Unknown UNI packets dropped to host
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleUUNIToHostEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleUUNIToHost
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable forwarding
 *      unkown unicast packets to host.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleUUNIToHost ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleUUNIToHost (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleUUNIToHost
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable forwarding
 *      unkown unicast packets to host.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleUUNIToHost ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleUUNIToHost (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleUVLANNoLearnEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to disable learning of the
 *      packets with unknown VLAN.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   Unknown VLAN No Learn enabled.
 *                              Source addresses of unknown VLANIDs are not added into
 *                              the look up table even if learning is enabled.
 *   @n  FALSE                  Unknown VLAN No Learn disabled.
 *                              Source addresses of unknown VLANIDs are added into
 *                              the look up table if learning is enabled.
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_ALE = 1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n     ALE_ALE_CONTROL_UVLAN_NO_LEARN
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleUVLANNoLearnEnabled () == TRUE)
        {
            // Unknown VLAN No Learn disabled
        }
        else
        {
            // Unknown VLAN No Learn enabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleUVLANNoLearnEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAleUVLANNoLearn
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable
 *      Unknown VLAN No Learn mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_ALE = 1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n     ALE_ALE_CONTROL_UVLAN_NO_LEARN=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleUVLANNoLearn ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleUVLANNoLearn (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAleUVLANNoLearn
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable
 *      unknown VLAN No Learn mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_ALE = 1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n     ALE_ALE_CONTROL_UVLAN_NO_LEARN=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleUVLANNoLearn ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleUVLANNoLearn (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_getAleUpdateBW
 *
 *   @b Description
 *   @n This function extracts the ALE Update Bandwidth of the ALE control register
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  aleUpdBW               ALE Update Bandwidth
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n     ALE_ALE_CONTROL_UPD_BW_CTRL
 *
 *   @b Example
 *   @verbatim

        Uint32  aleUpdBW;

        aleUpdBW = CSL_CPSW_getAleUpdateBW();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getAleUpdateBW (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleUpdateBW
 *
 *   @b Description
 *   @n This function configures the ALE Update Bandwidth of the ALE control
 *      register.
 *
 *   @b Arguments
 *   @n aleUpdBW               ALE Update Bandwidth
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n     ALE_ALE_CONTROL_UPD_BW_CTRL
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_setAleUpdateBW ((Uint32)ALE_UPD_BW_350MHZ_5M);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUpdateBW
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUpdBW
);



/** ============================================================================
 *   @n@b CSL_CPSW_startAleAgeOutNow
 *
 *   @b Description
 *   @n This function configures the ALE control register to initiate an ALE
 *      ageable entry cleanup. This enables the ALE hardware to remove any
 *      ageable table entry that does not have a set touch bit.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_AGE_OUT_NOW=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_startAleAgeOutNow ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_startAleAgeOutNow (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleAgeOutDone
 *
 *   @b Description
 *   @n This function reads the ALE control register's AGE_OUT_NOW bit to check
 *      if the ALE ageable entry cleanup process is done.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE age out process done.
 *   @n  FALSE                  ALE age out process not yet completed.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_AGE_OUT_NOW
 *
 *   @b Example
 *   @verbatim

        if (CSL_CPSW_isAleAgeOutDone ();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleAgeOutDone (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_clearAleTable
 *
 *   @b Description
 *   @n This function initiates a full ALE table cleanup. The ALE hardware
 *      clears all table entries.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_CLEAR_TABLE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_clearAleTable ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_clearAleTable (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_isAleEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE processing is enabled.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   ALE enabled. ALE packet processing will be done.
 *   @n  FALSE                  ALE disabled. All packets are dropped by ALE.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_ALE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleEnabled () == TRUE)
        {
            // ALE enabled
        }
        else
        {
            // ALE disabled
        }
     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleEnabled (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_enableAle
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable ALE processing.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_ALE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAle ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAle (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_disableAle
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable ALE processing.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_ALE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAle ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAle (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_getAleControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  >=0                ALE control register contents.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_CONTROL_REG
 *
 *   @b Example
 *   @verbatim
        Uint32      aleCtrlVal;

        aleCtrlVal  =   CSL_CPSW_getAleControlReg ();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getAleControlReg (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE control register.
 *
 *   @b Arguments
     @verbatim
        aleCtrlVal          Value to be configured to the ALE control register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_CONTROL_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32          aleCtrlVal = 0;

        aleCtrlVal      =   CSL_CPSW_getAleControlReg ();
        aleCtrlVal      |=  CSL_XGE_CPSW_ALECONTROL_CLRTABLE_EN;

        CSL_CPSW_setAleControlReg (&aleCtrlRegInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleControlReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleCtrlVal
);


/** ============================================================================
 *   @n@b CSL_CPSW_getAleStatusReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE Status register.
 *
 *   @b Arguments
     @verbatim
        pNumPolicers            Number of policers the ALE implements (multiple of 8)
        pNumEntries             Number of total table entries supported (multiple of 1024).
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_STATUS_KLUENTRIES
 *      ALE_ALE_STATUS_POLCNTDIV8
 *
 *   @b Example
 *   @verbatim
        Uint32      numPolicers, numEntries;

        CSL_CPSW_getAleStatusReg (&numPolicers,
                                  &numEntries);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleStatusReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        pNumPolicers,
    Uint32*                        pNumEntries
);


void CSL_CPSW_getAleStatusNumAleEntries(CSL_AleRegs *hCpswAleRegs,Uint32* pNumEntries);


CSL_CPSW_ALE_RAMDEPTH_E CSL_CPSW_getAleStatusRamDepth(CSL_AleRegs *hCpswAleRegs);



void CSL_CPSW_getAleStatusVlanMask(CSL_AleRegs *hCpswAleRegs,
                                                                    bool *vlanMsk08,
                                                                    bool *vlanMsk12);


/** ============================================================================
 *   @n@b CSL_CPSW_getAlePrescaleReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE Prescale register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *   @n  >=0                ALE prescale register contents.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_PRESCALE_ALE_PRESCALE
 *
 *   @b Example
 *   @verbatim
        Uint32      alePrescaleVal;

        alePrescaleVal  =   CSL_CPSW_getAlePrescaleReg ();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getAlePrescaleReg (CSL_AleRegs *hCpswAleRegs);


/** ============================================================================
 *   @n@b CSL_CPSW_setAlePrescaleReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE prescale register.
 *
 *   @b Arguments
     @verbatim
        alePrescaleVal      Value to be configured to the ALE Prescale register.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_PRESCALE_ALE_PRESCALE
 *
 *   @b Example
 *   @verbatim
 *      Uint32          alePrescaleVal = 0;

        alePrescaleVal  =   10;

        CSL_CPSW_setAlePrescaleReg (&aleCtrlRegInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAlePrescaleReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      alePrescaleVal
);


/** ============================================================================
 *   @n@b CSL_CPSW_getAleAgingTimerReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE Aging Timer register.
 *
 *   @b Arguments
     @verbatim
        pAgingPrescale          Aging Timer prescale (1, 1000, 1000000)
        pAgingPeriod            Aging period in units of prescale.
                                When non-zero, auto-aging is enabled.
                                This value (minus 1) times prescale is the number
                                of clock cycles after which auto-aging will automatically
                                be initiated.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_AGING_CTRL_ALE_AGING_TIMER
 *      ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE
 *      ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE
 *
 *   @b Example
 *   @verbatim
        Uint32      aleAgingPrescale;
        Uint32      aleAgingPeriod

        CSL_CPSW_getAleAgingTimerReg (&aleAgingPrescale,
                                      &aleAgingPeriod);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleAgingTimerReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        pAgingPrescale,
    Uint32*                        pAgingPeriod
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleAgingTimerReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE Aging Timer register.
 *
 *   @b Arguments
     @verbatim
        agingPrescale           Aging Timer prescale (1, 1000, 1000000)
        agingPeriod             Aging period in units of prescale.
                                When non-zero, auto-aging is enabled.
                                This value (minus 1) times prescale is the number
                                of clock cycles after which auto-aging will automatically
                                be initiated.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_AGING_CTRL_ALE_AGING_TIMER
 *      ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE
 *      ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE
 *
 *   @b Example
 *   @verbatim
        Uint32      aleAgingPrescale;
        Uint32      aleAgingPeriod;

        aleAgingPrescale = (Uint32)ALE_AGT_PRESACLE_1M;
        aleAgingPeriod = 1000;

        CSL_CPSW_setAleAgingTimerReg (aleAgingPrescale,
                                      aleAgingPeriod);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleAgingTimerReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                         agingPrescale,
    Uint32                         agingPeriod
);


/** ============================================================================
 *   @n@b CSL_CPSW_getAleUnkownVlanReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE Unknown VLAN and etc registers.
 *
 *   @b Arguments
     @verbatim
        pUnVlanMemList          Unknown VLAN member list.
        pUnMcastFloodMask       Unknown VLAN Multicast flood mask.
        pUnRegMcastFloodMask    Unknown VLAN Registered Multicast Flood mask.
        pUnForceUntagEgress     Unknown VLAN Force Untagged Egress.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_UNKNOWN_VLAN_REG_UNKNOWN_LIST,
 *      ALE_UNKNOWN_MCAST_FLOOD_REG_MASK,
 *      ALE_UNKNOWN_REG_MCAST_FLOOD_REG_MASK,
 *      ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS
 *
 *   @b Example
 *   @verbatim
        Uint32      unVlanMemList, unMcastFloodMask, unRegMcastFloodMask, unForceUntagEgress;

        CSL_CPSW_getAleUnkownVlanReg (&unVlanMemList,
                                      &unMcastFloodMask,
                                      &unRegMcastFloodMask,
                                      &unForceUntagEgress);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleUnkownVlanReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        pUnVlanMemList,
    Uint32*                        pUnMcastFloodMask,
    Uint32*                        pUnRegMcastFloodMask,
    Uint32*                        pUnForceUntagEgress
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleUnkownVlanReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE Unknown VLAN and etc. register.
 *
 *   @b Arguments
     @verbatim
        unVlanMemList           Unknown VLAN member list.
        unMcastFloodMask        Unknown VLAN Multicast flood mask.
        unRegMcastFloodMask     Unknown VLAN Registered Multicast Flood mask.
        unForceUntagEgress      Unknown VLAN Force Untagged Egress.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_UNKNOWN_VLAN_REG_UNKNOWN_LIST,
 *      ALE_UNKNOWN_MCAST_FLOOD_REG_MASK,
 *      ALE_UNKNOWN_REG_MCAST_FLOOD_REG_MASK,
 *      ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS
 *
 *   @b Example
 *   @verbatim
        Uint32      unVlanMemList, unMcastFloodMask, unRegMcastFloodMask, unForceUntagEgress;

        unVlanMemList           =   0;
        unMcastFloodMask        =   3;
        unRegMcastFloodMask     =   0;
        unForceUntagEgress      =   0;

        CSL_CPSW_setAleUnkownVlanReg (unVlanMemList,
                                      unMcastFloodMask,
                                      unRegMcastFloodMask,
                                      unForceUntagEgress);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUnkownVlanReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      unVlanMemList,
    Uint32                      unMcastFloodMask,
    Uint32                      unRegMcastFloodMask,
    Uint32                      unForceUntagEgress
);

#if ENET_CFG_IS_ON(ALE_VLAN_MASK_MUX)
/** ============================================================================
 *   @n@b CSL_CPSW_getAleVlanMaskMuxReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE VLAN Mask Mux registers.
 *
 *   @b Arguments
     @verbatim
        vlanMaskMux             Array of VLAN Mask Mux which is indexed by
                                the unreg_mcast_flood_index and reg_mcast_flood_
                                index values from the VLAN table entry to determine
                                the registered and unregistered multicast flood masks
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_VLAN_MASK_MUX_REG_MASK
 *
 *   @b Example
 *   @verbatim
        Uint32      vlanMaskMux[4];

        CSL_CPSW_getAleVlanMaskMuxReg (vlanMaskMux);

     @endverbatim
 *
 *   @b Note
 *   @n The value of VLAN_Mask_MUX_0 is read only and all ones (all ports are one).
 *
 * =============================================================================
 */
void CSL_CPSW_getAleVlanMaskMuxReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        vlanMaskMux
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanMaskMuxReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE VLAN Mask Mux registers.
 *
 *   @b Arguments
     @verbatim
        vlanMaskMux             Array of VLAN Mask Mux which is indexed by
                                the unreg_mcast_flood_index and reg_mcast_flood_
                                index values from the VLAN table entry to determine
                                the registered and unregistered multicast flood masks
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_VLAN_MASK_MUX_REG_MASK
 *
 *   @b Example
 *   @verbatim
        Uint32      vlanMaskMux[4];

        vlanMaskMux[0] = 0x3;
        vlanMaskMux[1] = 0;
        ...

        CSL_CPSW_setAleVlanMaskMuxReg (vlanMaskMux);

     @endverbatim
 *   @b Note
 *   @n The value of VLAN_Mask_MUX_0 is read only and all ones (all ports are one).
 *
 * =============================================================================
 */
void CSL_CPSW_setAleVlanMaskMuxReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        vlanMaskMux
);


/** ============================================================================
 *   @n@b CSL_CPSW_getAleVlanMaskMuxEntryReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE VLAN Mask Mux registers.
 *
 *   @b Arguments
     @verbatim
        vlanMaskMux             Array of VLAN Mask Mux which is indexed by
                                the unreg_mcast_flood_index and reg_mcast_flood_
                                index values from the VLAN table entry to determine
                                the registered and unregistered multicast flood masks
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  CSL_PASS               The function completed successfully
 *   @n  CSL_EOUT_OF_RANGE      The ifSelect argument is out-of-range
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_VLAN_MASK_MUX_REG_MASK
 *
 *   @b Example
 *   @verbatim
        Uint32      vlanMaskMux[4];

        CSL_CPSW_getAleVlanMaskMuxReg (vlanMaskMux);

     @endverbatim
 *   @b Note
 *   @n The value of VLAN_Mask_MUX_0 is read only and all ones (all ports are one).
 *
 * =============================================================================
 */
Int32 CSL_CPSW_getAleVlanMaskMuxEntryReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                         maskMuxIndex,
    Uint32*                        vlanMaskMuxPtr
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanMaskMuxEntryReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE VLAN Mask Mux registers.
 *
 *   @b Arguments
     @verbatim
        vlanMaskMux             Array of VLAN Mask Mux which is indexed by
                                the unreg_mcast_flood_index and reg_mcast_flood_
                                index values from the VLAN table entry to determine
                                the registered and unregistered multicast flood masks
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  CSL_PASS               The function completed successfully
 *   @n  CSL_EOUT_OF_RANGE      The ifSelect argument is out-of-range
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_VLAN_MASK_MUX_REG_MASK
 *
 *   @b Example
 *   @verbatim
        Uint32      vlanMaskMux[4];

        vlanMaskMux[0] = 0x3;
        vlanMaskMux[1] = 0;
        ...

        CSL_CPSW_setAleVlanMaskMuxReg (vlanMaskMux);

     @endverbatim
 *   @b Note
 *   @n The value of VLAN_Mask_MUX_0 is read only and all ones (all ports are one).
 *
 * =============================================================================
 */
Int32 CSL_CPSW_setAleVlanMaskMuxEntryReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                        maskMuxIndex,
    Uint32                        vlanMaskMuxVal
);
#endif

/** ============================================================================
 *   @n@b CSL_CPSW_getAleTableEntry
 *
 *   @b Description
 *   @n This function retrieves an ALE table entry corresponding to the
 *      ALE entry index specified in 'index' input parameter. The ALE
 *      entry values corresponding to the ALE_TBLW0, ALE_TBLW1 and
 *      ALE_TBLW2 registers are returned in 'pAleInfoWd0', 'pAleInfoWd1', 'pAleInfoWd2'
 *      output parameters.
 *
 *   @b Arguments
     @verbatim
        index                   ALE table index to be read.
        pAleInfoWd0             Contents of ALE Table Word 0 Register (ALE_TBLW0).
        pAleInfoWd1             Contents of ALE Table Word 1 Register (ALE_TBLW1).
        pAleInfoWd2             Contents of ALE Table Word 2 Register (ALE_TBLW2).
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_ALE_TBLW0_TABLEWRD0,
 *      ALE_ALE_TBLW1_TABLEWRD1,
 *      ALE_ALE_TBLW2_TABLEWRD2
 *
 *   @b Example
 *   @verbatim
        Uint32      index, info0, info1, info2;

        index   =   0;

        CSL_CPSW_getAleTableEntry (index,
                                   &info0,
                                   &info1,
                                   &info2);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleTableEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      index,
    Uint32*                     pAleInfoWd0,
    Uint32*                     pAleInfoWd1,
    Uint32*                     pAleInfoWd2
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleTableEntry
 *
 *   @b Description
 *   @n This function sets up an ALE table entry corresponding to the
 *      ALE entry index specified in 'index' input parameter. The ALE
 *      entry values corresponding to the ALE_TBLW0, ALE_TBLW1 and
 *      ALE_TBLW2 registers msut be specified in 'aleInfoWd0', 'aleInfoWd1', 'aleInfoWd2'
 *      input parameters.
 *
 *   @b Arguments
     @verbatim
        index                   ALE table index to be written.
        aleInfoWd0              Value to write to ALE Table Word 0 Register (ALE_TBLW0).
        aleInfoWd1              Value to write to ALE Table Word 1 Register (ALE_TBLW1).
        aleInfoWd2              Value to write to ALE Table Word 2 Register (ALE_TBLW2).
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLW0_TABLEWRD0,
 *      ALE_ALE_TBLW1_TABLEWRD1,
 *      ALE_ALE_TBLW2_TABLEWRD2,
 *      ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *
 *
 *   @b Example
 *   @verbatim
        Uint32      index, info0, info1, info2;

        index   =   0;
        info0   =   ...;
        info1   =   ...;
        info2   =   ...;

        CSL_CPSW_setAleTableEntry (index,
                                          info0,
                                          info1,
                                          info2);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleTableEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      index,
    Uint32                      aleInfoWd0,
    Uint32                      aleInfoWd1,
    Uint32                      aleInfoWd2
);


/** ============================================================================
 *   @n@b CSL_CPSW_getALEEntryType
 *
 *   @b Description
 *   @n This function returns the ALE entry type for any given ALE table
 *      entry index.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>  CSL_CPSW_ALE_ENTRYTYPE
 *   @n  ALE_ENTRYTYPE_FREE             ALE entry is free.
 *   @n  ALE_ENTRYTYPE_ADDRESS          ALE entry contains a unicast/multicast address.
 *   @n  ALE_ENTRYTYPE_VLAN             ALE entry contains a VLAN.
 *   @n  ALE_ENTRYTYPE_VLANADDRESS      ALE entry contains a VLAN and a unicast/multicast address.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD1_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32      index = 0;
        if (CSL_CPSW_getALEEntryType () == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry free
        }
     @endverbatim
 * =============================================================================
 */
CSL_CPSW_ALE_ENTRYTYPE CSL_CPSW_getALEEntryType(CSL_AleRegs *hCpswAleRegs,
                                                Uint32      index,
                                                CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getALEAddressType
 *
 *   @b Description
 *   @n This function returns the address type of an ALE entry.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>  CSL_CPSW_ALE_ADDRTYPE
 *   @n  ALE_ADDRTYPE_UCAST         Address at this entry is unicast
 *   @n  ALE_ADDRTYPE_MCAST         Address at this entry is multicast
 *   @n  ALE_ADDRTYPE_OUI           Address at this entry is OUI address
 *
 *   <b> Pre Condition </b>
 *   @n  This function must be called only for an ALE address entry, i.e.,
 *       if @a CSL_XGE_CPSW_getALEEntryType () returns ALE_ENTRYTYPE_ADDRESS
 *       or ALE_ENTRYTYPE_VLANADDRESS only.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0_REG,
 *      ALE_TABLE_WORD1_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32      index = 0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_ADDRESS)
        {
            // ALE entry has an address

            if (CSL_CPSW_getALEAddressType (index) ==  ALE_ADDRTYPE_UCAST)
            {
                // Unicast address
            }
        }
        else
        {
            // Do nothing
        }

        ...
     @endverbatim
 * =============================================================================
 */
CSL_CPSW_ALE_ADDRTYPE CSL_CPSW_getALEAddressType(CSL_AleRegs *hCpswAleRegs,
                                                 Uint32       index,
                                                 CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getALEPolicerEntryType
 *
 *   @b Description
 *   @n This function returns the entry type of an ALE Policer entry.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        aleType                 ALE type(4-Port/9-Port)

 *   @endverbatim
 *
 *   <b> Return Value </b>  CSL_CPSW_ALE_POLICER_ENTRYTYPE
 *   @n  ALE_POLICER_ENTRYTYPE_VLAN         (Inner) VLAN Entry
 *   @n  ALE_POLICER_ENTRYTYPE_OVLAN        Outer VLAN entry
 *   @n  ALE_POLICER_ENTRYTYPE_ETHERTYPEI   Ether Type entry
 *   @n  ALE_POLICER_ENTRYTYPE_IPV4         IPv4 Address entry
 *   @n  ALE_POLICER_ENTRYTYPE_IPV6         IPv6 Address entry
 *
 *   <b> Pre Condition </b>
 *   @n  This function must be called only for an ALE address entry, i.e.,
 *       if @a CSL_XGE_CPSW_getALEEntryType () returns ALE_ENTRYTYPE_POLICER.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD1_REG,
 *      ALE_TABLE_WORD2_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32      index = 0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_POLICER)
        {
            // ALE entry has an address

            if (CSL_CPSW_getALEPolicerEntryType (index) ==  ALE_POLICER_ENTRYTYPE_IPV4)
            {
                // IPv4 address
            }

        }
        else
        {
            // Do nothing
        }

        ...
     @endverbatim
 * =============================================================================
 */
CSL_CPSW_ALE_POLICER_ENTRYTYPE CSL_CPSW_getALEPolicerEntryType(CSL_AleRegs *hCpswAleRegs,
                                                Uint32     index,
                                                CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleMcastAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Multicast address configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pMcastAddrCfg           ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                     index;
        CSL_ALE_MCASTADDR_ENTRY    mcastAddrCfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_ADDRESS)
        {
            // ALE entry has an address

            if (CSL_CPSW_getALEAddressType (index) ==  ALE_ADDRTYPE_MCAST)
            {
                // Read Multicast address config from hardware
                CSL_CPSW_getAleMcastAddrEntry (index, &mcastAddrCfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleMcastAddrEntry(CSL_AleRegs    *hCpswAleRegs,
                                   Uint32         index,
                                   CSL_CPSW_ALE_MCASTADDR_ENTRY* pMcastAddrCfg,
                                   CSL_CPSW_ALETABLE_TYPE aleType);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleMcastAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      Multicast address configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pMcastAddrCfg           ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                              index;
        CSL_CPSW_ALE_MCASTADDR_ENTRY    mcastAddrCfg;

        index   =   0;
        mcastAddrCfg.macAddress [0] = 0x00;
        mcastAddrCfg.macAddress [1] = 0x01;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add Multicast address entry
            CSL_CPSW_setAleMcastAddrEntry (index, &mcastAddrCfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleMcastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                   Uint32                       index,
                                   CSL_CPSW_ALE_MCASTADDR_ENTRY*  pMcastAddrCfg,
                                   CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleVlanMcastAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with VLAN Multicast address configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pVlanMcastAddrCfg       ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_VLANMCASTADDR_ENTRY    vlanMcastAddrCfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_VLANADDRESS)
        {
            // ALE entry has a VLAN address

            if (CSL_CPSW_getALEAddressType (index) ==  ALE_ADDRTYPE_MCAST)
            {
                // Read VLAN Multicast address config from hardware
                CSL_CPSW_getAleVlanMcastAddrEntry (index, &vlanMcastAddrCfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleVlanMcastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                       Uint32      index,
                                       CSL_CPSW_ALE_VLANMCASTADDR_ENTRY*   pVlanMcastAddrCfg,
                                       CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanMcastAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      VLAN Multicast address configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pVlanMcastAddrCfg       ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_VLANMCASTADDR_ENTRY    vlanMcastAddrCfg;

        index   =   0;
        vlanMcastAddrCfg.macAddress [0] = 0x00;
        vlanMcastAddrCfg.macAddress [1] = 0x01;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add VLAN Multicast address entry
            CSL_CPSW_setAleVlanMcastAddrEntry (index, &vlanMcastAddrCfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleVlanMcastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                           Uint32      index,
                           CSL_CPSW_ALE_VLANMCASTADDR_ENTRY* pVlanMcastAddrCfg,
                           CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleUnicastAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Unicast address configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pUcastAddrCfg           ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_UNICASTADDR_ENTRY      ucastAddrCfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_ADDRESS)
        {
            // ALE entry has an address

            if (CSL_CPSW_getALEAddressType (index) ==  ALE_ADDRTYPE_UCAST)
            {
                // Read Unicast address config from hardware
                CSL_CPSW_getAleUnicastAddrEntry (index, &ucastAddrCfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleUnicastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                     Uint32      index,
                                     CSL_CPSW_ALE_UNICASTADDR_ENTRY*  pUcastAddrCfg,
                                     CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_setAleUnicastAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      unicast address configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pUcastAddrCfg           ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_UNICASTADDR_ENTRY      ucastAddrCfg;

        index   =   0;
        ucastAddrCfg.macAddress [0] = 0x00;
        ucastAddrCfg.macAddress [1] = 0x01;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add Unicast address entry
            CSL_CPSW_setAleUnicastAddrEntry (index, &ucastAddrCfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUnicastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                     Uint32      index,
                                     CSL_CPSW_ALE_UNICASTADDR_ENTRY* pUcastAddrCfg,
                                     CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleOUIAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with OUI address configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pOUIAddrCfg             ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_OUIADDR_ENTRY          ouiAddrCfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_ADDRESS)
        {
            // ALE entry has an address

            if (CSL_CPSW_getALEAddressType (index) ==  ALE_ADDRTYPE_OUI)
            {
                // Read Unicast address config from hardware
                CSL_CPSW_getAleOUIAddrEntry (index, &ouiAddrCfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleOUIAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32      index,
                                 CSL_CPSW_ALE_OUIADDR_ENTRY* pOUIAddrCfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_setAleOUIAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      OUI address configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pOUIAddrCfg             ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_OUIADDR_ENTRY          ouiAddrCfg;

        index   =   0;
        ouiAddrCfg.ouiAddress [0] = 0x00;
        ouiAddrCfg.ouiAddress [1] = 0x01;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add OUI address entry
            CSL_CPSW_setAleOUIAddrEntry (index, &ouiAddrCfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleOUIAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32       index,
                                 CSL_CPSW_ALE_OUIADDR_ENTRY* pOUIAddrCfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleVlanUnicastAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with VLAN Unicast address configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pVlanUcastAddrCfg       ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY  vlanUcastAddrCfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_VLANADDRESS)
        {
            // ALE entry has a VLAN address

            if (CSL_CPSW_getALEAddressType (index) ==  ALE_ADDRTYPE_UCAST)
            {
                // Read VLAN Unicast address config from hardware
                CSL_CPSW_getAleVlanUnicastAddrEntry (index, &ucvlanUcastAddrCfgastAddrCfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleVlanUnicastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                      Uint32      index,
                      CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY* pVlanUcastAddrCfg,
                      CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanUnicastAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      VLAN unicast address configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pVlanUcastAddrCfg       ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY  vlanUcastAddrCfg;

        index   =   0;
        vlanUcastAddrCfg.macAddress [0] = 0x00;
        vlanUcastAddrCfg.macAddress [1] = 0x01;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add VLAN Unicast address entry
            CSL_CPSW_setAleVlanUnicastAddrEntry (index, &vlanUcastAddrCfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleVlanUnicastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                         Uint32      index,
                         CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY* pVlanUcastAddrCfg,
                         CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleVlanEntry
 *
 *   @b Description
 *   @n This function reads the ALE entry info for inner vlan entry
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pVlanCfg                ALE entry info for VLAN populated by this function
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 * =============================================================================
 */
void CSL_CPSW_getAleVlanEntry(CSL_AleRegs               *hCpswAleRegs,
                              Uint32                    index,
                              CSL_CPSW_ALE_VLAN_ENTRY*  pVlanCfg,
                              CSL_CPSW_ALETABLE_TYPE aleType);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      VLAN configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pVlanCfg                ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_VLAN_ENTRY             vlanCfg;

        index   =   0;
        vlanCfg.vlanId  = 0x10;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add VLAN entry
            CSL_CPSW_setAleVlanEntry (index, &vlanCfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleVlanEntry(CSL_AleRegs *hCpswAleRegs,
                              Uint32      index,
                              CSL_CPSW_ALE_VLAN_ENTRY*   pVlanCfg,
                              CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleOutVlanEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Outer VLAN configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pOutValnCfg             ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD1
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_OUTER_VLAN_ENTRY           outVlanCfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_POLICER)
        {
            // ALE entry has a plicer entry
            if (CSL_CPSW_getALEPolicerEntryType (index) ==  ALE_POLICER_ENTRYTYPE_OVLAN)
            {
                // Read outer VALN config from hardware
                CSL_CPSW_getAleOutVlanEntry (index, &outVlanCfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleOutVlanEntry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32      index,
                                 CSL_CPSW_ALE_OUTER_VLAN_ENTRY * pOutVlanCfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_setAleOutVlanEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      Outer VLAN configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pOutValnCfg             ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)

 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_OUTER_VLAN_ENTRY           outVlanCfg;

        index   =   0;
        outVlanCfg.vlanId = 0x0123;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add Outer VLAN entry
            CSL_CPSW_setAleOutValnEntry (index, &outVlanCfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleOutVlanEntry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32       index,
                                 CSL_CPSW_ALE_OUTER_VLAN_ENTRY* pOutVlanCfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleEthertypeEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Ethertype configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pEthertypeCfg           ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_ETHERTYPE_ENTRY            ethertypeCfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_POLICER)
        {
            // ALE entry has a plicer entry
            if (CSL_CPSW_getALEPolicerEntryType (index) ==  ALE_POLICER_ENTRYTYPE_ETHERTYPE)
            {
                // Read Ethertype config from hardware
                CSL_CPSW_getAleEthertypeEntry (index, &ethertypeCfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleEthertypeEntry(CSL_AleRegs *hCpswAleRegs,
                                   Uint32      index,
                                   CSL_CPSW_ALE_ETHERTYPE_ENTRY*  pEthertypeCfg,
                                   CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_setAleEthertypeEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      Ethertype configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pEthertypeCfg           ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_ETHERTYPE_ENTRY            ethertypeCfg;

        index   =   0;
        ethertypeCfg.ethertype = 0x0800;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add Ethertype entry
            CSL_CPSW_setAleEthertypeEntry (index, &ethertypeCfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleEthertypeEntry(CSL_AleRegs *hCpswAleRegs,
                                   Uint32      index,
                                   CSL_CPSW_ALE_ETHERTYPE_ENTRY* pEthertypeCfg,
                                   CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleIPv4Entry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with IPv4 configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pIPv4Cfg                ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_IPv4_ENTRY                 ipv4Cfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_POLICER)
        {
            // ALE entry has a plicer entry
            if (CSL_CPSW_getALEPolicerEntryType (index) ==  ALE_POLICER_ENTRYTYPE_IPV4)
            {
                // Read IPv4 config from hardware
                CSL_CPSW_getAleIPv4Entry (index, &ipv4Cfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleIPv4Entry(CSL_AleRegs *hCpswAleRegs,
                              Uint32      index,
                              CSL_CPSW_ALE_IPv4_ENTRY*   pIPv4Cfg,
                              CSL_CPSW_ALETABLE_TYPE aleType);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleIPv4Entry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      IPv4 configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pIPv4Cfg                ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_IPv4_ENTRY                 ipv4Cfg;

        index   =   0;
        ipv4Cfg.ethertype = 0x0800;
        ...

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE)
        {
            // ALE entry is free

            // Add IPv4 entry
            CSL_CPSW_setAleIPv4Entry (index, &ipv4Cfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleIPv4Entry(CSL_AleRegs *hCpswAleRegs,
                              Uint32      index,
                              CSL_CPSW_ALE_IPv4_ENTRY* pIPv4Cfg,
                              CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleIPv6HighEntryOffset
 *
 *   @b Description
 *   @n This function returns the offset of the higher 64bit offset of IPv6 entry
 *      for the specific ALE type
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getAleIPv6HighEntryOffset(CSL_AleRegs *hCpswAleRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleIPv6HighEntryIndex
 *
 *   @b Description
 *   @n This function returns the higher 64bit offset of IPv6 entry given the
 *      lower 64bit ALE table entry index
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getAleIPv6HighEntryIndex(CSL_AleRegs *hCpswAleRegs,Uint32 entryIndex);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleIPv6LowEntryIndex
 *
 *   @b Description
 *   @n This function returns the lower 64bit ALE entry index of IPv6 entry given the
 *      higher 64bit ALE table entry index
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getAleIPv6LowEntryIndex(CSL_AleRegs *hCpswAleRegs,Uint32 entryIndex);

/** ============================================================================
 *   @n@b CSL_CPSW_getAleIPv6Entry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Ipv6 configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index to be read.
        pIPv6Cfg                ALE entry contents read.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_TABLE_WORD0
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_IPv6_ENTRY                 ipv6Cfg;

        index   =   0;

        if (CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_POLICER)
        {
            // ALE entry has a plicer entry
            if (CSL_CPSW_getALEPolicerEntryType (index) ==  ALE_POLICER_ENTRYTYPE_IPV6)
            {
                // Read Ipv6 config from hardware
                CSL_CPSW_getAleIPv6Entry (index, &ipv6Cfg);
            }
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleIPv6Entry(CSL_AleRegs *hCpswAleRegs,
                              Uint32      index,
                              CSL_CPSW_ALE_IPv6_ENTRY* pIPv6Cfg,
                              CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_setAleIPv6Entry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      IPv6 configuration specified here.
 *
 *   @b Arguments
     @verbatim
        hCpswAleRegs            ALE register overlay
        index                   ALE table index.
        pIPv6Cfg                ALE entry contents to be configured.
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0,
 *      ALE_TABLE_WORD1,
 *      ALE_TABLE_WORD2
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_IPv6_ENTRY                 ipv4Cfg;

        index   =   0;
        ipv4Cfg.ethertype = 0x0800;
        ...

        if ((CSL_CPSW_getALEEntryType (index) == ALE_ENTRYTYPE_FREE &&
            (CSL_CPSW_getALEEntryType (index+1) == ALE_ENTRYTYPE_FREE))
        {
            // ALE entry is free

            // Add IPv6 entry
            CSL_CPSW_setAleIPv6Entry (index, &ipv4Cfg);
        }

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleIPv6Entry(CSL_AleRegs *hCpswAleRegs,
                              Uint32      index,
                              CSL_CPSW_ALE_IPv6_ENTRY*  pIPv6Cfg,
                              CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_mapTableWord2MacAddr
 *
 *   @b Description
 *   @n This function extracts the mac address from the ALE table word0 and
 *      word 1
 *   @b Arguments
     @verbatim
        word0                   ALE table word 0 value
        word1                   ALE table word 0 value
        macAddr                 mac address which will be populated
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 * =============================================================================
 */
void CSL_CPSW_mapTableWord2MacAddr(uint32_t word0,
                                   uint32_t word1,
                                   uint8_t * macAddr,
                                   CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_mapMacAddr2TableWord
 *
 *   @b Description
 *   @n This function populates the ALE table word 0 and word 1 with the
 *      mac address passed
 *   @b Arguments
     @verbatim
        word0                   Pointer to ALE table word 0 to be populated
        word1                   Pointer to ALE table word 1 to be populated
        macAddr                 input mac address
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 * =============================================================================
 */
void CSL_CPSW_mapMacAddr2TableWord(uint32_t             *word0,
                                   uint32_t             *word1,
                                   uint8_t *            macAddr,
                                   CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_extractVid
 *
 *   @b Description
 *   @n This function extracts the vlan id field from the ALE table word 1
 *   @b Arguments
     @verbatim
        word1                   ALE table word 1 value for a VLAN entry
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   @b Return Value
 *   @n vid                     Extracted vlan id
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_extractVid(Uint32             word1,
                           CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_getEthertypeMax
 *
 *   @b Description
 *   @n This function returns the max value of EtherType field in ALE table
 *   @b Arguments
     @verbatim
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   @b Return Value
 *   @n Maximum value of Ethertype field in ALE table
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getEthertypeMax(CSL_CPSW_ALETABLE_TYPE aleType);


/** ============================================================================
 *   @n@b CSL_CPSW_getIpv4IgnBitsMax
 *
 *   @b Description
 *   @n This function returns the max value of IPv4 ignore bits field in ALE table
 *   @b Arguments
     @verbatim
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   @b Return Value
 *   @n Maximum value of max value of IPv4 ignore bits field in ALE table
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getIpv4IgnBitsMax(CSL_CPSW_ALETABLE_TYPE aleType);


/** ============================================================================
 *   @n@b CSL_CPSW_getIpv6IgnBitsMax
 *
 *   @b Description
 *   @n This function returns the max value of IPv6 ignore bits field in ALE table
 *   @b Arguments
     @verbatim
        aleType                 ALE type(4-Port/9-Port)
 *   @endverbatim
 *
 *   @b Return Value
 *   @n Maximum value of max value of IPv4 ignore bits field in ALE table
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getIpv6IgnBitsMax(CSL_CPSW_ALETABLE_TYPE aleType);

/** ============================================================================
 *   @n@b CSL_CPSW_clearAleEntry
 *
 *   @b Description
 *   @n This function clears the ALE entry corresponding to the index
 *      specified
 *
 *   @b Arguments
     @verbatim
        index                   ALE table index.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0=0,
 *      ALE_TABLE_WORD1=0,
 *      ALE_TABLE_WORD2=0
 *
 *   @b Example
 *   @verbatim
        Uint32                              index;

        index   =   0;

        CSL_CPSW_clearAleEntry (index);
     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_clearAleEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                                  index
);


/** ============================================================================
 *   @n@b CSL_CPSW_getAlePortControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of ALE Port control register
 *      corresponding to the port number specified.
 *
 *   @b Arguments
     @verbatim
        portNo                  Port number for which the ALE port control register
                                must be read.
        pPortControlInfo        CSL_CPSW_ALE_PORTCONTROL structure that needs to be
                                filled with Port control register info read from
                                the hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT
 *
 *   @b Example
 *   @verbatim
        Uint32                          index;
        CSL_CPSW_ALE_PORTCONTROL    portControlInfo;

        index   =   0;

        CSL_CPSW_getAlePortControlReg (index, &portControlInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAlePortControlReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    CSL_CPSW_ALE_PORTCONTROL*   pPortControlInfo
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAlePortControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of ALE Port control register
 *      corresponding to the port number specified.
 *
 *   @b Arguments
     @verbatim
        portNo                  Port number for which the ALE port control register
                                must be configured.
        pPortControlInfo        CSL_CPSW_ALE_PORTCONTROL structure that contains
                                port control register settings to be written.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT
 *
 *   @b Example
 *   @verbatim
        Uint32                          index;
        CSL_CPSW_ALE_PORTCONTROL    portControlInfo;

        index   =   0;
        portControlInfo.portState   =   ALE_PORTSTATE_FORWARD |
                                        ALE_PORTSTATE_LEARN;

        CSL_CPSW_setAlePortControlReg (index, &portControlInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAlePortControlReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    CSL_CPSW_ALE_PORTCONTROL*   pPortControlInfo
);


void CSL_CPSW_setAlePortControlTrunk
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    bool                     trunkEnable,
    Uint32                      trunkNum
);

/** ============================================================================
 *   @n@b CSL_CPSW_getAlePortControlReg
 *
 * =============================================================================
 */
void CSL_CPSW_getAlePortState
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    CSL_CPSW_ALE_PORTSTATE      *pPortState
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAlePortControlReg
 *
 * =============================================================================
 */
void CSL_CPSW_setAlePortState
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    CSL_CPSW_ALE_PORTSTATE      portState
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAlePortControlReg
 *
 * =============================================================================
 */
void CSL_CPSW_setAlePortMirrorSouce(CSL_AleRegs *hCpswAleRegs,
                                                    Uint32  portNo,
                                                    bool enableMirror);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleCtrl2MirrorMatchIndex
 *
 * =============================================================================
 */
void CSL_CPSW_setAleCtrl2MirrorMatchIndex(CSL_AleRegs *hCpswAleRegs,
                                                          Uint32  mirrorMatchIndex);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleCtrl2MirrorMatchIndex
 *
 * =============================================================================
 */
void CSL_CPSW_setAleCtrl2TrunkParams(CSL_AleRegs *hCpswAleRegs,
                                                     CSL_CPSW_ALE_CTRL2_TRUNK_CONFIG *trunkCfg);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleCtrl2IPPktFilterConfig
 *
 * =============================================================================
 */
void CSL_CPSW_setAleCtrl2IPPktFilterConfig(CSL_AleRegs *hCpswAleRegs,
                                                           CSL_CPSW_ALE_CTRL2_IPPKTFLT_CONFIG *ipPktFltCfg);


/** ============================================================================
 *   @n@b CSL_CPSW_setAleCtrl2MalformedPktConfig
 *
 * =============================================================================
 */
void CSL_CPSW_setAleCtrl2MalformedFrameConfig(CSL_AleRegs *hCpswAleRegs,
                                                              CSL_CPSW_ALE_CTRL2_MALFORMEDFRAME_CONFIG *badFrmCfg);


/** ============================================================================
 *   @n@b CSL_CPSW_setIPNxtHdrList
 *
 * =============================================================================
 */
void CSL_CPSW_setAleIPNxtHdrWhitelist(CSL_AleRegs *hCpswAleRegs,
                                                   Uint8 ipNxtHdr0,
                                                   Uint8 ipNxtHdr1,
                                                   Uint8 ipNxtHdr2,
                                                   Uint8 ipNxtHdr3);


/** ============================================================================
 *   @n@b CSL_CPSW_getAlePolicerGlobConfig
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW ALE Policer/Classifier
 *        Global Configuration.
 *
 *   @b Arguments
     @verbatim
        pGlobConfig             CSL_CPSW_ALE_POLICER_GLOB_CONFIG structure that needs to
                                be populated with the contents of the corresponging ALE Policer
                                global control registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_THREADMAPDEF_DEFTHREAD_EN,
 *      ALE_THREADMAPDEF_DEFTHREADVAL
 *
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_ALE_POLICER_GLOB_CONFIG    globConfig;

        CSL_CPSW_getAlePolicerGlobConfig (&globConfig);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAlePolicerGlobConfig (CSL_AleRegs *hCpswAleRegs,
    CSL_CPSW_ALE_POLICER_GLOB_CONFIG*   pGlobConfig
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAlePolicerGlobConfig
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW ALE Policer/Classifier
 *      global registers
 *      per user-specified ALE Policer Global Configuration.
 *
 *   @b Arguments
     @verbatim
        pGlobConfig         CSL_CPSW_ALE_POLICER_GLOB_CONFIG structure that holds the values
                            that need to be configured to the ALE Policer global control
                            registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSW ALE Policer Global control register modified with values provided.
 *
 *   @b Writes
 *   @n ALE_THREADMAPDEF_DEFTHREAD_EN,
 *      ALE_THREADMAPDEF_DEFTHREADVAL
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_ALE_POLICER_GLOB_CONFIG    globConfig;

        globConfig.defThreadEnable  =   1;
        ...

        CSL_CPSW_setAlePolicerGlobConfig (&globConfig);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAlePolicerGlobConfig (CSL_AleRegs *hCpswAleRegs,
    CSL_CPSW_ALE_POLICER_GLOB_CONFIG*   pGlobConfig
);


/** ============================================================================
 *   @n@b CSL_CPSW_getAlePolicerEntry
 *
 *   @b Description
 *   @n This function reads the ALE Policer table entry for the index specified and
 *      fills the output parameter structure with Policer configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        index                   ALE Policer table index to be read.
        pPolCfg                 ALE Policer entry contents read.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_POLICETBLCTL_POL_TBL_IDX
 *      ALE_POLICETBLCTL_WRITE_ENABLE=0
 *      ALE_THREADMAPCTL_CLASSINDEX
 *
 *   @b Reads
 *   @n ALE_POLICECFG0_PORT_MEN,
 *      ALE_POLICECFG0_PORT_NUM,
 *      ALE_POLICECFG0_PRI_MEN,
 *      ALE_POLICECFG0_PRI_VAL,
 *      ALE_POLICECFG0_ONU_MEN,
 *      ALE_POLICECFG0_ONU_INDEX,
 *      ALE_POLICECFG1_DST_MEN,
 *      ALE_POLICECFG1_DST_INDEX,
 *      ALE_POLICECFG1_SRC_MEN,
 *      ALE_POLICECFG1_SRC_INDEX,
 *      ALE_POLICECFG2_OVLAN_MEN,
 *      ALE_POLICECFG2_OVLAN_INDEX,
 *      ALE_POLICECFG2_IVLAN_MEN,
 *      ALE_POLICECFG2_IVLAN_INDEX,
 *      ALE_POLICECFG3_ETHERTYPE_MEN,
 *      ALE_POLICECFG3_ETHERTYPE_INDEX,
 *      ALE_POLICECFG3_IPSRC_MEN,
 *      ALE_POLICECFG3_IPSRC_INDEX,
 *      ALE_POLICECFG4_IPDST_MEN,
 *      ALE_POLICECFG4_IPDST_INDEX,
 *      ALE_THREADMAPVAL_THREAD_EN,
 *      ALE_THREADMAPVAL_THREADVAL
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_POLICER_ENTRY              polCfg;

        index   =   0;

         // Read Policer Entry config from hardware
         CSL_CPSW_getAlePolicerEntry (index, &polCfg);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAlePolicerEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                              index,
    CSL_CPSW_ALE_POLICER_ENTRY*         pPolCfg
);


/** ============================================================================
 *   @n@b CSL_CPSW_setAlePolicerEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      VLAN configuration specified here.
 *
 *   @b Arguments
     @verbatim
        index                   ALE table index.
        pPolCfg                 ALE entry contents to be configured.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_POLICETBLCTL_POL_TBL_IDX
 *      ALE_POLICETBLCTL_WRITE_ENABLE=1
 *      ALE_THREADMAPCTL_CLASSINDEX
 *      ALE_POLICECFG0_PORT_MEN,
 *      ALE_POLICECFG0_PORT_NUM,
 *      ALE_POLICECFG0_PRI_MEN,
 *      ALE_POLICECFG0_PRI_VAL,
 *      ALE_POLICECFG0_ONU_MEN,
 *      ALE_POLICECFG0_ONU_INDEX,
 *      ALE_POLICECFG1_DST_MEN,
 *      ALE_POLICECFG1_DST_INDEX,
 *      ALE_POLICECFG1_SRC_MEN,
 *      ALE_POLICECFG1_SRC_INDEX,
 *      ALE_POLICECFG2_OVLAN_MEN,
 *      ALE_POLICECFG2_OVLAN_INDEX,
 *      ALE_POLICECFG2_IVLAN_MEN,
 *      ALE_POLICECFG2_IVLAN_INDEX,
 *      ALE_POLICECFG3_ETHERTYPE_MEN,
 *      ALE_POLICECFG3_ETHERTYPE_INDEX,
 *      ALE_POLICECFG3_IPSRC_MEN,
 *      ALE_POLICECFG3_IPSRC_INDEX,
 *      ALE_POLICECFG4_IPDST_MEN,
 *      ALE_POLICECFG4_IPDST_INDEX,
 *      ALE_THREADMAPVAL_THREAD_EN,
 *      ALE_THREADMAPVAL_THREADVAL
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_POLICER_ENTRY              polCfg;

        index   =   0;
        polCfg.vlanId  = 0x10;
        ...

        // Add ALE Policer entry
        CSL_CPSW_setAlePolicerEntry (index, &polCfg);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAlePolicerEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                              index,
    CSL_CPSW_ALE_POLICER_ENTRY*         pPolCfg
);


void CSL_CPSW_disableAlePolicerThread(CSL_AleRegs *hCpswAleRegs,
                                                    Uint32           index);




/** ============================================================================
 *   @n@b CSL_CPSW_setAleUnknwnVlanMemberReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE unknown vlan register.
 *
 *   @b Arguments
     @verbatim
        aleUnknwnVlanVal  Value to be configured to the ALE unknown vlan reg
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n UNKNOWN_VLAN_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32          aleUnknwnVlanVal = 0;

        CSL_CPSW_getAleUnknwnVlanReg (&aleUnknwnVlanVal);
        aleUnknwnVlanVal      |=  CSL_XGE_CPSW_ALECONTROL_CLRTABLE_EN;

        CSL_CPSW_setAleUnknwnVlanReg (aleUnknwnVlanVal);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUnknwnVlanMemberReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUnknwnVlanMemberVal
);



/** ============================================================================
 *   @n@b CSL_CPSW_setAleUnknwnVlanUntagReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE unknown vlan register.
 *
 *   @b Arguments
     @verbatim
        aleUnknwnVlanVal  Value to be configured to the ALE unknown vlan reg
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n UNKNOWN_VLAN_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32          aleUnknwnVlanVal = 0;

        CSL_CPSW_getAleUnknwnVlanReg (&aleUnknwnVlanVal);
        aleUnknwnVlanVal      |=  CSL_XGE_CPSW_ALECONTROL_CLRTABLE_EN;

        CSL_CPSW_setAleUnknwnVlanReg (aleUnknwnVlanVal);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUnknwnVlanUntagReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUnknwnVlanUntagVal
);

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiP0Control
 *
 *   @b Description
 *   @n This function sets the P0_CONTROL_REG register contents.
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiP0ControlCfg   P0_CONTROL_REG configuration structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void  CSL_CPSW_setCppiP0Control(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_CONTROL *pCppiP0ControlCfg);
void CSL_CPSW_setAleUnknwnVlanUnregMcastReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUnknwnVlanUnregMcastVal
);


void CSL_CPSW_setAleUnknwnVlanRegMcastReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUnknwnVlanRegMcastVal
);


void CSL_CPSW_setAlePolicerControlReg(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_CONTROL *policerCntrlCfg);


void CSL_CPSW_getAlePolicerControlReg(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_CONTROL *policerCntrlCfg);


void CSL_CPSW_setAlePolicerTestControlReg(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_TEST_CONTROL *policerTestCntrlCfg);


void CSL_CPSW_getAlePolicerHstatReg(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_HSTAT *policerHStatCfg);



/** ============================================================================
 *   @n@b CSL_CPSW_setAleOAMLpbkControl
 *
 * =============================================================================
 */
void CSL_CPSW_setAleOAMLpbkControl(CSL_AleRegs *hCpswAleRegs,
                                                   Uint32 lpbkEnablePortMask);


void CSL_CPSW_getAleStatusNumPolicers(CSL_AleRegs *hCpswAleRegs,Uint32* pNumPolicers);


void CSL_CPSW_setCppiPriCirEir(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 cir, Uint32 eir);


void CSL_CPSW_getCppiPriCirEir(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 *cir, Uint32 *eir);


void CSL_CPSW_setPriCirEir(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 cir, Uint32 eir);


void CSL_CPSW_getPriCirEir(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 *cir, Uint32 *eir);


void CSL_CPSW_setCppiRxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint8 pri);


Uint8 CSL_CPSW_getCppiRxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs);


void CSL_CPSW_setCppiTxDstThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getCppiTxDstThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri);


void CSL_CPSW_setCppiTxDstThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getCppiDstTxThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri);


void CSL_CPSW_setCppiTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getCppiTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri);


void CSL_CPSW_setCppiTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getCppiTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri);


void CSL_CPSW_setcppiTxBlksPri(CSL_Xge_cpswRegs *hCpswRegs,Uint8 pri, Uint32 blks);


Uint32 CSL_CPSW_getCppiTxBlksPri(CSL_Xge_cpswRegs *hCpswRegs,Uint8 pri);


void CSL_CPSW_setTxBlksPri(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint8 pri, Uint32 blks);


Uint32 CSL_CPSW_getTxBlksPri(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint8 pri);


void CSL_CPSW_setTxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint8 pri);


Uint8 CSL_CPSW_getTxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo);


void CSL_CPSW_setRxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint8 pri);


Uint8 CSL_CPSW_getRxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo);


Uint32 CSL_CPSW_getTxHostBlksRem(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo);


void CSL_CPSW_setTxHostBlksRem(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 txBlksRem);


void CSL_CPSW_setTxDstThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getTxDstThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri);


void CSL_CPSW_setTxDstThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getTxDstThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri);


void CSL_CPSW_setTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri);


void CSL_CPSW_setTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri);


void CSL_CPSW_setTxDstBasedOutFlowAddValX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 addVal);


Uint32 CSL_CPSW_getTxDstBasedOutFlowAddValX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri);


void CSL_CPSW_setTxGlobalOutFlowThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getTxGlobalOutFlowThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri);


void CSL_CPSW_setTxGlobalOutFlowThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getTxGlobalOutFlowThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri);



void CSL_CPSW_setCpswTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getCpswTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri);


void CSL_CPSW_setCpswTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal);


Uint32 CSL_CPSW_getCpswTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri);

#if ENET_CFG_IS_ON(CPSW_CPPI_CAST)
/** ============================================================================
 *   @n@b CSL_CPSW_isP0TxCastagnoliCRCEnabled
 *
 *   @b Description
 *   @n This function indicates if Castagnoli CRC is enabled in the CPSW
 *      control register for host port.
 *
 *
 *   <b> Return Value </b>
 *   @n  TRUE                   Castagnoli CRC enabled
 *   @n  FALSE                  Castagnoli CRC disabled.
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_isP0TxCastagnoliCRCEnabled (CSL_Xge_cpswRegs *hCpswRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_enableP0TxCastagnoliCRC
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable Castagnoli CRC for
 *      host port specified.
 *
 *   @b Return Value
 *   @n  None
 *
 * =============================================================================
 */
void CSL_CPSW_enableP0TxCastagnoliCRC(CSL_Xge_cpswRegs *hCpswRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_disableP0TxCastagnoliCRC
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable Castagnoli CRC for
 *      host port.
 *
 *   @b Return Value
 *   @n None
 *
 * =============================================================================
 */
void CSL_CPSW_disableP0TxCastagnoliCRC(CSL_Xge_cpswRegs *hCpswRegs);
#endif

/** ============================================================================
 *   @n@b CSL_CPSW_getPTypeReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW PTYPE register.
 *
 *   @b Arguments
     @verbatim
        pPtypeCfg       CSL_CPSW_PTYPE structure that needs to be populated
                        with the CPSW PTYPE register contents.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 * =============================================================================
 */
void CSL_CPSW_getPTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
                           CSL_CPSW_PTYPE*  pPtypeCfg);

/** ============================================================================
 *   @n@b CSL_CPSW_setPTypeReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW PTYPE register.
 *
 *   @b Arguments
     @verbatim
        pPtypeCfg       CSL_CPSW_PTYPE structure that has the
                        CPSW PTYPE register configuration.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 * =============================================================================
 */
void CSL_CPSW_setPTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
                           CSL_CPSW_PTYPE*  pPtypeCfg);

/** ============================================================================
 *   @n@b CSL_CPSW_getThruRateReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW_THRU_RATE register.
 *
 *   @b Arguments
     @verbatim
        pThruRateCfg    CSL_CPSW_THRURATE structure that needs to be populated
                        with the CPSW_THRU_RATE register contents.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 * =============================================================================
 */
void CSL_CPSW_getThruRateReg (CSL_Xge_cpswRegs *hCpswRegs,
                              CSL_CPSW_THRURATE *  pThruRateCfg);

/** ============================================================================
 *   @n@b CSL_CPSW_getGapThreshold
 *
 *   @b Description
 *   @n This function retrieves the contents of the GAP_THRESH_REG register.
 *
 *  The Ethernet port transmit inter-packet gap (IPG) may be shortened by eight
 *  bit times when short gap is enabled and triggered.
 *  Setting the pn_tx_short_gap_en bit each Enet_Pn_Mac_Control register
 *  enables the gap to be shortened when triggered.
 *  The condition is triggered when the ports associated transmit packet FIFO
 *  has a user defined number of FIFO blocks used.
 *  The associated transmit FIFO blocks used value determines
 *  if the gap is shortened, and so on.
 *  The Gap_Thresh register value determines the short gap threshold.
 *  If the FIFO blocks used is greater than or equal to the Gap_Thresh value
 *  then short gap is triggered
 *   @b Arguments
 *    @verbatim
 *       none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Gap threshold configured
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getGapThreshold(CSL_Xge_cpswRegs *hCpswRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_setGapThreshold
 *
 *   @b Description
 *   @n This function retrieves the contents of the GAP_THRESH_REG register.
 *
 *  The Ethernet port transmit inter-packet gap (IPG) may be shortened by eight
 *  bit times when short gap is enabled and triggered.
 *  Setting the pn_tx_short_gap_en bit each Enet_Pn_Mac_Control register
 *  enables the gap to be shortened when triggered.
 *  The condition is triggered when the ports associated transmit packet FIFO
 *  has a user defined number of FIFO blocks used.
 *  The associated transmit FIFO blocks used value determines
 *  if the gap is shortened, and so on.
 *  The Gap_Thresh register value determines the short gap threshold.
 *  If the FIFO blocks used is greater than or equal to the Gap_Thresh value
 *  then short gap is triggered
 *   @b Arguments
 *    @verbatim
 *       gapThreshold     Ethernet Port  Short Gap Threshold -
 *                        This is the Ethernet port associated FIFO transmit
 *                        block usage value for triggering transmit short gap
 *                        (when short gap is enabled)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Gap threshold configured
 *
 * =============================================================================
 */
void CSL_CPSW_setGapThreshold(CSL_Xge_cpswRegs *hCpswRegs,Uint32 gapThreshold);

/** ============================================================================
 *   @n@b CSL_CPSW_getTxStartWdsReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the TX_START_WDS_REG register.
 *
 *   FIFO Packet Transmit (egress) Start Words.
 *   This value is the number of required 32-byte packet words in an Ethernet
 *   transmit FIFO before the packet egress will begin.
 *   This value is non-zero to preclude Ethernet transmit underrun.
 *   Decimal 8 is the recommended value.
 *   It should not be increased unnecessarily to prevent adding to the
 *   switch latency
 *
 *   @b Arguments
 *    @verbatim
 *       none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Gap threshold configured
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getTxStartWords(CSL_Xge_cpswRegs *hCpswRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_getTxMaxLenPerPriority
 *
 *   @b Description
 *   @n This function retrieves the max tx packet length per switch priority.
 *
 *   @b Arguments
 *    @verbatim
 *       priority   Priority for which tx max packet length is queried (0 -7)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Max tx packet length for given priority
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getTxMaxLenPerPriority(CSL_Xge_cpswRegs *hCpswRegs,
                                       Uint32 priority);

/** ============================================================================
 *   @n@b CSL_CPSW_setTxMaxLenPerPriority
 *
 *   @b Description
 *   @n This function sets the max tx packet length per switch priority.
 *
 *   @b Arguments
 *    @verbatim
 *       priority   Priority for which tx max packet length is set (0 -7)
 *       maxLen     Tx max packet length to be set
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Max tx packet length for given priority
 *
 * =============================================================================
 */
void  CSL_CPSW_setTxMaxLenPerPriority(CSL_Xge_cpswRegs *hCpswRegs,
                                       Uint32 priority,
                                       Uint32 maxLen);

/** ============================================================================
 *   @n@b CSL_CPSW_getCppiP0Control
 *
 *   @b Description
 *   @n This function gets the P0_CONTROL_REG register contents.
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiP0ControlCfg   P0_CONTROL_REG configuration structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void  CSL_CPSW_getCppiP0Control(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_CONTROL *pCppiP0ControlCfg);
/** ============================================================================
 *   @n@b CSL_CPSW_setCppiP0Control
 *
 *   @b Description
 *   @n This function sets the P0_CONTROL_REG register contents.
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiP0ControlCfg   P0_CONTROL_REG configuration structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void  CSL_CPSW_setCppiP0Control(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_CONTROL *pCppiP0ControlCfg);

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiRxPType
 *
 *   @b Description
 *   @n This function sets the P0 Receive priority type
 *
 *   @b Arguments
 *    @verbatim
 *       p0RxPtype   Receive Priority Type
 *                   0 - Fixed priority
 *                   1 - Round Robin priority
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_setCppiRxPType(CSL_Xge_cpswRegs *hCpswRegs,Uint32 p0RxPtype);

/** ============================================================================
 *   @n@b CSL_CPSW_getCppiRxPType
 *
 *   @b Description
 *   @n This function gets the P0 Receive priority type
 *
 *   @b Arguments
 *    @verbatim
 *       none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  p0RxPtype   Receive Priority Type
 *                   0 - Fixed priority
 *                   1 - Round Robin priority
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getCppiRxPType(CSL_Xge_cpswRegs *hCpswRegs);

#if ENET_CFG_IS_ON(CPSW_2PORTSWITCH)
/** ============================================================================
 *   @n@b CSL_CPSW_getCppiRxPacketsPriority
 *
 *   @b Description
 *   @n This function gets Port 0 Receive (same as Port 1 Transmit) Packets Per
 *      Priority
 *      This function is applicable for 2 port switch
 *
 *   @b Arguments
 *    @verbatim
 *       priority  Priority for which number of rx packets is queried
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  rxPackets   Number of rx packets for queried priority
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getCppiRxPacketsPriority(CSL_Xge_cpswRegs *hCpswRegs,
                                         Uint32 priority);

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiRxPacketsPriority
 *
 *   @b Description
 *   @n This function gets Port 0 Receive (same as Port 1 Transmit) Packets Per
 *      Priority
 *      This function is applicable for 2 port switch
 *
 *   @b Arguments
 *    @verbatim
 *       priority  Priority for which number of rx packets is to be set
 *       rxPackets Number of rx packets to be set
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_setCppiRxPacketsPriority(CSL_Xge_cpswRegs *hCpswRegs,
                                       Uint32 priority,
                                       Uint32 rxPackets);

/** ============================================================================
 *   @n@b CSL_CPSW_getCppiRxGapReg
 *
 *   @b Description
 *   @n This function gets CPPI_P0_Rx_Gap
 *      This function is applicable for 2 port switch
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiRxGap  CPPI_P0_Rx_Gap register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_getCppiRxGapReg(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_RXGAP *pCppiRxGap);

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiRxGapReg
 *
 *   @b Description
 *   @n This function sets CPPI_P0_Rx_Gap
 *      This function is applicable for 2 port switch
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiRxGap  CPPI_P0_Rx_Gap register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_setCppiRxGapReg(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_RXGAP *pCppiRxGap);
#endif
/** ============================================================================
 *   @n@b CSL_CPSW_getP0FifoStatus
 *
 *   @b Description
 *   @n This function gets CPPI_P0_FIFO_Status register
 *      This function is will return 0 for  2 port switch as there is no Tx FIFO
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiFifoStats  CPPI_P0_FIFO_Status register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_getP0FifoStatus(CSL_Xge_cpswRegs *hCpswRegs,
                              CSL_CPSW_CPPI_P0_FIFOSTATUS *pCppiFifoStats);

/** ============================================================================
 *   @n@b CSL_CPSW_getP0HostBlksPri
 *
 *   @b Description
 *   @n This function gets CPPI_P0_Host_Blks_Pri  register
 *      This function is not applicable to two port switch
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiHostBlksPri  CPPI_P0_Host_Blks_Pri register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_getP0HostBlksPri(CSL_Xge_cpswRegs *hCpswRegs,
                               CSL_CPSW_CPPI_P0_HOSTBLKSPRI *pCppiHostBlksPri);

/** ============================================================================
 *   @n@b CSL_CPSW_setP0HostBlksPri
 *
 *   @b Description
 *   @n This function sets CPPI_P0_Host_Blks_Pri  register
 *      This function is not applicable to two port switch
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiHostBlksPri  CPPI_P0_Host_Blks_Pri register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_setP0HostBlksPri(CSL_Xge_cpswRegs *hCpswRegs,
                               CSL_CPSW_CPPI_P0_HOSTBLKSPRI *pCppiHostBlksPri);

#ifdef __cplusplus
}
#endif

#endif

/**
@}
*/
