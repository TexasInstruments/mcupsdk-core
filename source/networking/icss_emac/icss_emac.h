/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 */

#ifndef ICSS_EMAC_H_
#define ICSS_EMAC_H_

/**
 *  \defgroup NETWORKING_ICSS_EMAC_MODULE APIs for ICSS-EMAC
 *  \ingroup NETWORKING_MODULE
 *
 *  ICSS-EMAC (Industrial Communications Sub-system Ethernet Media Access Controller)
 *  APIs to transmit and receive packets with a firmware based Ethernet switch that
 *  has been implemented on PRUICSS.
 *
 *  @{
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/pruicss.h>
#include <board/ethphy.h>
#include <drivers/hw_include/csl_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Maximum number of Ports in a single ICSS  */
#define ICSS_EMAC_MAX_PORTS_PER_INSTANCE    (2)

/**
 *  \anchor ICSS_EMAC_Modes
 *  \name ICSS EMAC Modes
 *
 *  @{
 */
/** \brief Single EMAC Mode. Port 1 Enabled */
#define ICSS_EMAC_MODE_MAC1                 (1U)
/** \brief Single EMAC Mode. Port 2 Enabled */
#define ICSS_EMAC_MODE_MAC2                 (2U)
/** \brief Switch Mode */
#define ICSS_EMAC_MODE_SWITCH               (3U)
/** \brief Dual MAC Mode. Both Ports Enabled */
#define ICSS_EMAC_MODE_DUALMAC              (4U)
/** @} */

/**
 *  \anchor ICSS_EMAC_PhyToMacInterfaceModes
 *  \name ICSS EMAC PHY to MAC Interface Modes
 *
 *  @{
 */
/** \brief Media-independent Interface */
#define ICSS_EMAC_MII_MODE                  (0U)
/** \brief Reduced Gigabit Media-independent Interface  */
#define ICSS_EMAC_RGMII_MODE                (1U)
/** @} */

/**
 *  \anchor ICSS_EMAC_LearningModes
 *  \name ICSS EMAC Learning Modes
 *
 *  @{
 */
/** \brief Driver-based Learning Mode Disabled  */
#define ICSS_EMAC_LEARNING_DISABLE   (0U)
/** \brief Driver-based Learning Mode Enabled  */
#define ICSS_EMAC_LEARNING_ENABLE    (1U)
/** @} */

/**
 *  \anchor ICSS_EMAC_InterruptPacingConfig
 *  \name ICSS EMAC Interrupt Pacing Configuration
 *
 *  @{
 */
/** \brief Interrupt pacing enabled*/
#define ICSS_EMAC_ENABLE_PACING             (0)
/** \brief Interrupt pacing disabled*/
#define ICSS_EMAC_DISABLE_PACING            (1)
/** @} */

/**
 *  \anchor ICSS_EMAC_InterruptPacingModes
 *  \name ICSS EMAC Interrupt Pacing Modes
 *
 *  @{
 */
/** \brief Frame Count based Interrupt pacing */
#define ICSS_EMAC_INTR_PACING_MODE1         (0)
/** @} */

/**
 *  \anchor ICSS_EMAC_Queues
 *  \name ICSS EMAC Queues
 *
 *  @{
 */
/** \brief Priority Queue 1*/
#define ICSS_EMAC_QUEUE1 ((uint32_t)0U)
/** \brief Priority Queue 2*/
#define ICSS_EMAC_QUEUE2 ((uint32_t)1U)
/** \brief Priority Queue 3*/
#define ICSS_EMAC_QUEUE3 ((uint32_t)2U)
/** \brief Priority Queue 4*/
#define ICSS_EMAC_QUEUE4 ((uint32_t)3U)
/** \brief Priority Queue 5*/
#define ICSS_EMAC_QUEUE5 ((uint32_t)4U)
/** \brief Priority Queue 6*/
#define ICSS_EMAC_QUEUE6 ((uint32_t)5U)
/** \brief Priority Queue 7*/
#define ICSS_EMAC_QUEUE7 ((uint32_t)6U)
/** \brief Priority Queue 8*/
#define ICSS_EMAC_QUEUE8 ((uint32_t)7U)
/** \brief Priority Queue 9*/
#define ICSS_EMAC_QUEUE9 ((uint32_t)8U)
/** \brief Priority Queue 10*/
#define ICSS_EMAC_QUEUE10 ((uint32_t)9U)
/** \brief Priority Queue 11*/
#define ICSS_EMAC_QUEUE11 ((uint32_t)10U)
/** \brief Priority Queue 12*/
#define ICSS_EMAC_QUEUE12 ((uint32_t)11U)
/** \brief Priority Queue 13*/
#define ICSS_EMAC_QUEUE13 ((uint32_t)12U)
/** \brief Priority Queue 14*/
#define ICSS_EMAC_QUEUE14 ((uint32_t)13U)
/** \brief Priority Queue 15*/
#define ICSS_EMAC_QUEUE15 ((uint32_t)14U)
/** \brief Priority Queue 16*/
#define ICSS_EMAC_QUEUE16 ((uint32_t)15U)
/** \brief Collision Queue*/
#define ICSS_EMAC_COLQUEUE ((uint32_t)16U)
/** @} */

/** \brief Total Queues available */
#define ICSS_EMAC_NUMQUEUES ((uint32_t)17U)

/** \brief Maximum Valid size (incl header + VLAN TAG..., no CRC) */
#define ICSS_EMAC_MAXMTU  (1518U)
/** \brief Minimum Valid size ( DA + SA + Ethertype) */
#define ICSS_EMAC_MINMTU  (14U)

/**
 * \brief Used to specify host side port
 */
#define ICSS_EMAC_PORT_0 (0)

/**
 * \brief Used to specify physical port 1 MII 0 (tx)
 */
#define ICSS_EMAC_PORT_1 (1U)

/**
 * \brief Used to specify physical port 2 MII 1 (tx)
 */
#define ICSS_EMAC_PORT_2 (2U)

/** \brief ICSS_EMAC_IOCTL_PORT_CTRL options */

/** \brief Generic IOCTL Enable macro*/
#define ICSS_EMAC_IOCTL_PORT_CTRL_DISABLE                   (0u)

/** \brief Generic IOCTL Disable macro*/
#define ICSS_EMAC_IOCTL_PORT_CTRL_ENABLE                    (1u)

/**
 *  \anchor ICSS_EMAC_IOCTL_STORM_PREV_CTRL_COMMANDS
 *  \name ICSS-EMAC Storm Prevention IOCTL Command Options
 *
 *  @{
 */
/** \brief IOCTL Storm Control Enable command*/
#define ICSS_EMAC_STORM_PREV_CTRL_ENABLE                    (0u)
/** \brief IOCTL Storm Control Disable command*/
#define ICSS_EMAC_STORM_PREV_CTRL_DISABLE                   (1u)
/** \brief IOCTL Storm Control set credit command */
#define ICSS_EMAC_STORM_PREV_CTRL_SET_CREDITS               (2u)
/** \brief IOCTL Storm Control initialize command */
#define ICSS_EMAC_STORM_PREV_CTRL_INIT                      (3u)
/** \brief IOCTL Storm Control reset command */
#define ICSS_EMAC_STORM_PREV_CTRL_RESET                     (4u)
/** \brief IOCTL Storm Control Enable command for BC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_ENABLE_BC                 (5u)
/** \brief IOCTL Storm Control Disable command for BC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_DISABLE_BC                (6u)
/** \brief IOCTL Storm Control set credit command for BC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_SET_CREDITS_BC            (7u)
/** \brief IOCTL Storm Control initialize command for BC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_INIT_BC                   (8u)
/** \brief IOCTL Storm Control reset command for BC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_RESET_BC                  (9u)
/** \brief IOCTL Storm Control Enable command for MC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_ENABLE_MC                 (10u)
/** \brief IOCTL Storm Control Disable command for MC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_DISABLE_MC                (11u)
/** \brief IOCTL Storm Control set credit command for MC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_SET_CREDITS_MC            (12u)
/** \brief IOCTL Storm Control initialize command for MC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_INIT_MC                   (13u)
/** \brief IOCTL Storm Control reset command for MC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_RESET_MC                  (14u)
/** \brief IOCTL Storm Control Enable command for UC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_ENABLE_UC                 (15u)
/** \brief IOCTL Storm Control Disable command for UC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_DISABLE_UC                (16u)
/** \brief IOCTL Storm Control set credit command for UC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_SET_CREDITS_UC            (17u)
/** \brief IOCTL Storm Control initialize command for UC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_INIT_UC                   (18u)
/** \brief IOCTL Storm Control reset command for UC traffic*/
#define ICSS_EMAC_STORM_PREV_CTRL_RESET_UC                  (19u)
/** @} */

/**
 *  \anchor ICSS_EMAC_IOCTL_LEARNING_CTRL_COMMANDS
 *  \name ICSS-EMAC Learning Control IOCTL Command Options
 *
 *  @{
 */
/** \brief IOCTL Learning Table update command */
#define ICSS_EMAC_LEARN_CTRL_UPDATE_TABLE                   (0u)
/** \brief IOCTL Learning Table clear table command */
#define ICSS_EMAC_LEARN_CTRL_CLR_TABLE                      (1u)
/** \brief IOCTL Learning Table age out entry command */
#define ICSS_EMAC_LEARN_CTRL_AGEING                         (2u)
/** \brief IOCTL Learning Table find port from MAC ID command */
#define ICSS_EMAC_LEARN_CTRL_FIND_MAC                       (3u)
/** \brief IOCTL Learning Table remove a MAC ID from table command */
#define ICSS_EMAC_LEARN_CTRL_REMOVE_MAC                     (4u)
/** \brief IOCTL Learning Table increment counters (for ageing) command */
#define ICSS_EMAC_LEARN_CTRL_INC_COUNTER                    (5u)
/** \brief IOCTL Learning Table initialize command */
#define ICSS_EMAC_LEARN_CTRL_INIT_TABLE                     (6u)
/** \brief IOCTL Learning Table set port state command */
#define ICSS_EMAC_LEARN_CTRL_SET_PORTSTATE                  (7u)
/** @} */

/**
 *  \anchor ICSS_EMAC_IOCTL_STATISTICS_COMMANDS
 *  \name ICSS-EMAC Statistics IOCTL Command Options
 *
 *  @{
 */
/** \brief IOCTL Statistics get from PRU command */
#define ICSS_EMAC_IOCTL_STAT_CTRL_GET                   (0u)
/** \brief IOCTL Statistics clear all counters command */
#define ICSS_EMAC_IOCTL_STAT_CTRL_CLEAR                 (1u)
/** @} */

/**
 *  \anchor ICSS_EMAC_IOCTL_COMMANDS
 *  \name ICSS-EMAC IOCTL Command for selecting a module
 *
 *  @{
 */
/** \brief IOCTL select port control APIs command */
#define ICSS_EMAC_IOCTL_PORT_CTRL                           (0u)
/** \brief IOCTL select learning Table APIs command */
#define ICSS_EMAC_IOCTL_LEARNING_CTRL                       (1u)
/** \brief IOCTL select Storm Prevention APIs command */
#define ICSS_EMAC_IOCTL_STORM_PREV_CTRL                     (2u)
/** \brief IOCTL select Statistics APIs command */
#define ICSS_EMAC_IOCTL_STATS_CTRL                          (3u)
/** \brief IOCTL select Promiscuous Mode APIs command */
#define ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL                    (4u)
/** \brief IOCTL select Multicast filtering APIs command */
#define ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL               (5u)
/** \brief IOCTL select Vlan filtering APIs command */
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL                    (6u)
/** \brief IOCTL select Port flush APIs command */
#define ICSS_EMAC_IOCTL_PORT_FLUSH_CTRL                     (7u)
/** @} */

/**
 *  \anchor ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_COMMANDS
 *  \name ICSS-EMAC Multicast Filtering IOCTL Command Options
 *
 *  @{
 */
/** \brief IOCTL Multicast filter Control Enable command*/
#define ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ENABLE        (0u)
/** \brief IOCTL Multicast filter Control Disable command*/
#define ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_DISABLE       (1u)
/** \brief IOCTL Multicast filter Control override hashmask command */
#define ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_OVERRIDE_HASHMASK    (2u)
/** \brief IOCTL Multicast filter Control allow MC MAC ID command */
#define ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ADD_MACID   (3u)
/** \brief IOCTL Multicast filter Control do not allow MC MAC ID command */
#define ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_REMOVE_MACID   (4u)
/** \brief IOCTL Multicast filter Control do not allow MC MAC ID command */
#define ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_GET_DROPPED   (5u)
/** @} */

/**
 *  \anchor ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_COMMANDS
 *  \name ICSS-EMAC VLAN Filtering IOCTL Command Options
 *
 *  @{
 */
/** \brief IOCTL Vlan filter Control Enable command*/
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_ENABLE_CMD                (0u)
/** \brief IOCTL Vlan filter Control Disable command*/
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_DISABLE_CMD               (1u)
/** \brief IOCTL Vlan filter allow Untagged frames Host Receive command */
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_UNTAG_HOST_RCV_ALL_CMD    (2u)
/** \brief IOCTL Vlan filter do not allow Untagged frames Host Receive command */
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_UNTAG_HOST_RCV_NAL_CMD    (3u)
/** \brief IOCTL Vlan filter allow Priority Tag frames Host Receive command */
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_PRIOTAG_HOST_RCV_ALL_CMD    (4u)
/** \brief IOCTL Vlan filter do not allow Priority Tag frames Host Receive command */
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_PRIOTAG_HOST_RCV_NAL_CMD    (5u)
/** \brief IOCTL Vlan filter 4096 VIDs, 1 bit VIDs => 4096 bits = 512 bytes, ADD VLAN ID to allow packet to host  */
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_ADD_VID_CMD                 (6u)
/** \brief IOCTL Vlan filter 4096 VIDs, 1 bit VIDs => 4096 bits = 512 bytes, Remove VLAN ID to do not allow packet to host  */
#define ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_REMOVE_VID_CMD              (7u)
/** @} */

/**
* \brief Macros for different port states in learning module
*
*/
#define ICSS_EMAC_LEARNING_PORT_STATE_LEARNING          (0U)
#define ICSS_EMAC_LEARNING_PORT_STATE_NOT_LEARNING      (1U)
#define ICSS_EMAC_LEARNING_PORT_STATE_LOCKED            (2U)

#define ICSS_EMAC_OBJECT_SIZE_IN_BYTES                  (42000)

/**
 * \brief Alias for ICSS EMAC Handle containing base addresses and modules
 */
typedef struct ICSS_EMAC_Config_s *ICSS_EMAC_Handle;

/**
 * \brief Definition for a generic callback function used in ICSS-EMAC. \n
 *        While calling this, first argument is always of type \ref ICSS_EMAC_Handle,
 *        second argument is specific to the callback (see \ref ICSS_EMAC_CallBackObject), and
 *        third argument is userArg as specified in \ref ICSS_EMAC_CallBackConfig.
 */
typedef int32_t (*ICSS_EMAC_CallBack)(void *arg0, void *arg1, void *arg2);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief ICSS EMAC Static Firmware Memory Map offsets.
 *        These are offsets for PRU0 and PRU1 DRAM memory.
 */
typedef struct ICSS_EMAC_FwStaticMmap_s
{
    uint32_t versionOffset;             /**< Version offset for release 1 */
    uint32_t version2Offset;            /**< Version offset for release 2 */
    uint32_t featureOffset;             /**< Feature offset */
    uint32_t futureFeatureOffset;       /**< Offset reserved for enhance features, future use*/
    uint32_t statisticsOffset;          /**< Statistics offset */
    uint32_t statisticsSize;            /**< Statistics block size */
    uint32_t stormPreventionOffsetBC;   /**< Storm prevention offset */
    uint32_t phySpeedOffset;            /**< Phy Speed Offset */
    uint32_t portStatusOffset;          /**< Port Status Offset */
    uint32_t portControlAddr;           /**< Port Control Addr offset */
    uint32_t portMacAddr;               /**< Port Mac Addr offset*/
    uint32_t rxInterruptStatusOffset;   /**< RX Interrupt Status Offset */
    uint32_t stormPreventionOffsetMC;   /**< Storm prevention offset (multicast) */
    uint32_t stormPreventionOffsetUC;   /**< Storm prevention offset (unicast) */
    uint32_t p0QueueDescOffset;         /**< Port 0 QueueDescOffset */
    uint32_t p0ColQueueDescOffset;      /**< Port 0 Collision QueueDescOffset */
    uint32_t emacTtsConfigBaseOffset;   /**< TTS Config Base Offset */
    uint32_t interfaceMacAddrOffset;    /**< Interface Mac AddressrOffset */
    uint32_t colStatusAddr;             /**< Collision status address offset */
    uint32_t promiscuousModeOffset;     /**< promiscuous mode feature control offset */
} ICSS_EMAC_FwStaticMmap;

/**
 * \brief ICSS EMAC Dynamic Firmware Memory Map offsets
 */
typedef struct ICSS_EMAC_FwDynamicMmap_s
{
    uint32_t queueSizeOffset;                           /**< offset for queue size */
    uint32_t queueOffset;                               /**< offset for queue */
    uint32_t queueDescriptorOffset;                     /**< offset for queue descriptors */
    uint32_t txQueueSize[ICSS_EMAC_NUMQUEUES-1U];       /**< TX queue sizes */
    uint32_t rxHostQueueSize[ICSS_EMAC_NUMQUEUES - 1U]; /**< RX Host queue sizes */
    uint32_t collisionQueueSize;                        /**< Collision queue size */
    uint32_t p0Q1BufferDescOffset;                      /**< Port 0 Queue1 Buffer Descriptor offset */
    uint32_t p0ColBufferDescOffset;                     /**< Port 0 Collision Buffer Descriptor offset */
    uint32_t p0Q1BufferOffset;                          /**< Port 0 Queue1 Buffer offset */
    uint32_t transmitQueuesBufferOffset;                /**< Transmit Queue Buffer offset */
    uint32_t p0ColBufferOffset;                         /**< Port 0 Collision Buffer offset */
    uint32_t hostQ1RxContextOffset;                     /**< Receive Host Queue 1 Context offset */
    uint32_t p1Q1SwitchTxContextOffset;                 /**< Port 1 queue 1 Switch TX Context offset */
    uint32_t portQueueDescOffset;                       /**< Port queue descriptor offset */
    uint32_t q1EmacTxContextOffset;                     /**< Queueu1 Emac TX Context Offset */
    uint32_t numQueues;                                 /**< number of port queues */
} ICSS_EMAC_FwDynamicMmap;

/**
 * \brief ICSS EMAC VLAN Filtering Parameters
 */
typedef struct ICSS_EMAC_FwVlanFilterParams_s
{
    uint32_t ctrlBitmapOffset;
    uint32_t ctrlEnableBit;
    uint32_t ctrlUntagHostRcvAllowBit;
    uint32_t ctrlPriotagHostRcvAllowBit;
    uint32_t filterTableBaseAddress;
    uint32_t vidMaxValue;
} ICSS_EMAC_FwVlanFilterParams;

/**
 * \brief ICSS EMAC Multicast Filtering Parameters
 */
typedef struct ICSS_EMAC_FwMulticastFilterParams_s
{
    uint32_t ctrlOffset;
    uint32_t maskSizeBytes;
    uint32_t maskInitVal;
    uint32_t maskOffset;
    uint32_t overrideStatusOffset;
    uint32_t tableOffset;
    uint32_t ctrlEnabledValue;
    uint32_t ctrlDisabledValue;
    uint32_t maskOverrideSetValue;
    uint32_t maskOverrideNotSetValue;
    uint32_t hostRcvAllowedValue;
    uint32_t hostRcvNotAllowedValue;
} ICSS_EMAC_FwMulticastFilterParams;

/*
*  \brief     ICSS EMAC Init Configuration Structure
*/
typedef struct ICSS_EMAC_Attrs_s
{
    uint8_t                     emacMode;
    /**< Mode from \ref ICSS_EMAC_Modes */
    uint32_t                    phyAddr[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**< Phy address of the ports.
     *   For mac mode, phyAddr[0] is used.
     *   For switch mode,  phyAddr[0] and  phyAddr[1] are used.
     */
    uint8_t                     phyToMacInterfaceMode;
    /**< PHY to MAC Interface Mode
     *   Valid values : \ref ICSS_EMAC_PhyToMacInterfaceModes
     */
    uint8_t                     halfDuplexEnable;
    /**< Flag to enable Half duplex capability. Firmware support also is required
     *   to enable the functionality.
     *   Valid values : 0 to disable, 1 to enable.
     */
    uint8_t                     enableIntrPacing;
    /**< Flag to enable Interrupt pacing.
     * Valid values : \ref ICSS_EMAC_InterruptPacingConfig
     */
    uint8_t                     intrPacingMode;
    /**< Pacing mode to be used.
     * Valid values : \ref ICSS_EMAC_InterruptPacingModes
     */
    uint16_t                    pacingThreshold;
    /**< Number of packets threshold for Pacing Mode1 */
    uint8_t                     ethPrioQueue;
    /**< Queue Priority separation for RT and NRT packets. If packets are in
     *   Queue <= ethPrioQueue, they will be forwarded to RT callback and others
     *   to NRT callback
     */
    uint8_t                     learningEnable;
    /**< Flag to enable learning. Not applicable for Mac mode.
     *   Valid values : \ref ICSS_EMAC_LearningModes
     */
    uint8_t                     portMask;
    /**< Port Mask. Indication to LLD which ports to be used
     *   Valid values: ICSS_EMAC_MODE_SWITCH, ICSS_EMAC_MODE_MAC1,
     *                  ICSS_EMAC_MODE_MAC2
     */
    uint8_t                     txInterruptEnable;
    /**< Flag to enable TX Interrupt.
     *   Valid values : 0 to disable, 1 to enable.
     *   If this flag is enabled, ICSS_EMAC_txInterruptHandler() is registered
     *   as ISR for the txIntNum interrupt. This interrupt handler unblocks
     *   ICSS_EMAC_osTxTaskFnc() and calls the user-specified txCallback.
     */
    uint32_t                    linkIntNum;
    /* Link interrupt number on R5F*/
    uint32_t                    rxIntNum;
    /* Receive Packet interrupt number on R5F*/
    uint32_t                    txIntNum;
    /* Transmit completion interrupt number on R5F.
       Valid only if txInterruptEnable is set to 1 */
    uint32_t                    l3OcmcBaseAddr;
    /**< L3 OCMC Base Address */
    uint32_t                    l3OcmcSize;
    /**< L3 OCMC Size */
    uint32_t                    linkTaskPriority;
    /**< Link Task Priority */
    uint32_t                    rxTaskPriority;
    /**< RX Task Priority */
    uint32_t                    txTaskPriority;
    /**< TX Task Priority. Valid only if txInterruptEnable is set to 1 */
    uint32_t                    splitQueue;
    /**< Flag to be set for using split Queue i.e. seperate queue for each of ports*/
} ICSS_EMAC_Attrs;

/**
 *  \brief Opaque ICSS EMAC driver object.
 */
typedef struct ICSS_EMAC_InternalObject_t
{
    uint32_t reserved[ICSS_EMAC_OBJECT_SIZE_IN_BYTES/sizeof(uint32_t)];
    /**< reserved, should NOT be modified by end users */
} ICSS_EMAC_InternalObject;

/**
 * \brief   Base EMAC handle containing pointers to all modules required for driver
 *          to work
 */
typedef struct ICSS_EMAC_Config_s
{
    void                    *object;
    /**< Pointer to a driver specific data object */
    const ICSS_EMAC_Attrs   *attrs;
    /**< Pointer to a driver specific hardware attributes structure */
} ICSS_EMAC_Config;

/**
 * \brief Generic callback configuration for protocol specific callbacks. \n
 *        \ref ICSS_EMAC_CallBack is the function prototype used. While calling this,
 *        first argument is always of type \ref ICSS_EMAC_Handle, second argument
 *        is specific to the callback (see \ref ICSS_EMAC_CallBackObject), and
 *        third argument is userArg as specified in this structure.
 */
typedef struct ICSS_EMAC_CallBackConfig_s
{
    ICSS_EMAC_CallBack  callBack;
    /**< Function pointer for the callback function */
    void                *userArg;
    /**<  User argument for callback function */
} ICSS_EMAC_CallBackConfig;

/**
 * \brief Different callbacks which can be registered. \n
 *        While calling the function set in \ref ICSS_EMAC_CallBack,
 *        first argument is always of type \ref ICSS_EMAC_Handle, second argument
 *        is specific to the callback (as mentioned in the member descriptions), and
 *        third argument is userArg as specified in \ref ICSS_EMAC_CallBackConfig.
 */
typedef struct ICSS_EMAC_CallBackObject_s
{
    ICSS_EMAC_CallBackConfig port0LinkCallBack;
    /**<  Callback function for link change on Port 0. Second argument to callback is linkStatus */
    ICSS_EMAC_CallBackConfig port1LinkCallBack;
    /**<  Callback function for link change on Port 1. Second argument to callback is linkStatus */
    ICSS_EMAC_CallBackConfig rxNRTCallBack;
    /**<  Callback function for packets received with queueNumber >= ethPrioQueue (of \ref ICSS_EMAC_Attrs).
     *    Second argument to callback is queueNumber.
     */
    ICSS_EMAC_CallBackConfig rxRTCallBack;
    /**<  Callback function for packets received with queueNumber < ethPrioQueue (of \ref ICSS_EMAC_Attrs).
     *    Second argument to callback is queueNumber.
     */
    ICSS_EMAC_CallBackConfig txCallBack;
    /**<  Callback function for packet Tx completion. Second argument is NULL */
    ICSS_EMAC_CallBackConfig learningExCallBack;
    /**<  Callback function for protocol specific exception adaptation in learning. Second argument is macId */
    ICSS_EMAC_CallBackConfig customTxCallBack;
    /**<  Custom Callback function for Tx.
     *    Second argument to callback is txArg
     */
    ICSS_EMAC_CallBackConfig customRxCallBack;
    /**<  Custom Callback function for packets received .
     *    Second argument to callback is rxArg.
     */
} ICSS_EMAC_CallBackObject;

/**
 *  \brief ICSS_EMAC Parameters
 *
 *  ICSS_EMAC Parameters are used to with the #ICSS_EMAC_open() call. Default
 *  values for these parameters are set using #ICSS_EMAC_Params_init().
 *
 *  [MANDATORY] is present in the description for members which are mandatory.
 *
 *  \sa #ICSS_EMAC_Params_init()
 */
typedef struct ICSS_EMAC_Params_s
{
    PRUICSS_Handle                      pruicssHandle;
    /**< [MANDATORY] PRUICSS Handle for which the ICSS-EMAC driver will be based on */
    const PRUICSS_IntcInitData          *pruicssIntcInitData;
    /**< [MANDATORY] PRUICSS INTC mapping structure pointer needed for \ref PRUICSS_intcInit call */
    ICSS_EMAC_FwStaticMmap              *fwStaticMMap;
    /**< [MANDATORY] Static Firmware Memory Map offsets */
    ICSS_EMAC_FwDynamicMmap             *fwDynamicMMap;
    /**< [MANDATORY] Dynamic Firmware Memory Map offsets */
    ICSS_EMAC_FwVlanFilterParams        *fwVlanFilterParams;
    /**<  VLAN filtering related Firmware Memory Map offsets. Should be passed if IOCTL commands for VLAN filtering are going to be used */
    ICSS_EMAC_FwMulticastFilterParams   *fwMulticastFilterParams;
    /**<  Multicast filtering related Firmware Memory Map offsets. Should be passed if IOCTL commands for Multicast filtering are going to be used */
    ICSS_EMAC_CallBackObject            callBackObject;
    /**<  Callback functions for difference scenarios like link change, Rx for NRT/RT, Tx and Learning exception*/
    ETHPHY_Handle                       ethphyHandle[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /**< [MANDATORY] Ethernet PHY Handle.
     *   For mac mode, ethphyHandle[0] is used.
     *   For switch mode,  ethphyHandle[0] and  ethphyHandle[1] are used.
     */
    uint8_t                             macId[6];
    /**< [MANDATORY] MacId to be used for the interface*/
} ICSS_EMAC_Params;

/**
 * \brief   Rx packet processing information block that needs to passed into
 *          call to ICSS_EMAC_RxPktGet
 */
typedef struct ICSS_EMAC_RxArgument_s
{
    ICSS_EMAC_Handle    icssEmacHandle;
    /**< Handle to ICSS_EMAC instance */
    uint32_t            destAddress;
    /**< Base address of data buffer where received frame has to be stored */
    uint8_t             queueNumber;
    /**< Receive queue from which frame has to be copied */
    uint8_t             port;
    /**< Returns port number on which frame was received */
    uint32_t            more;
    /**< Returns more which is set to 1 if there are more frames in the queue */
} ICSS_EMAC_RxArgument;

/**
 * \brief   Tx packet processing information block that needs to passed into
 *          call to ICSS_EMAC_TxPacket
 */
typedef struct ICSS_EMAC_TxArgument_s
{
    ICSS_EMAC_Handle    icssEmacHandle;
    /**< Handle to ICSS_EMAC instance */
    const uint8_t       *srcAddress;
    /**< Base address of the buffer where the frame to be transmitted resides */
    uint8_t             portNumber;
    /**< Port on which frame has to be transmitted */
    uint8_t             queuePriority;
    /**< Queue number in which frame will be  queued for transmission */
    uint16_t            lengthOfPacket;
    /**< Length of the frame in bytes */
} ICSS_EMAC_TxArgument;

/**
 * \brief IOCTL command members for configuring switch/EMAC
 */
typedef struct ICSS_EMAC_IoctlCmd_s
{
    uint8_t command;
    void    *ioctlVal;
} ICSS_EMAC_IoctlCmd;

/**
* \brief Statistics structure for capturing statistics on PRU
*
*/
typedef struct ICSS_EMAC_PruStatistics_s
{
/* The fields here are aligned here so that it's consistent
  with the memory layout in PRU DRAM, this is to facilitate easy
  memcpy or DMA transfer. Don't change the order of fields without
  modifying the order of fields in PRU DRAM. For details refer to guide
*/
    volatile uint32_t txBcast;                  /**Number of broadcast packets sent*/
    volatile uint32_t txMcast;                  /**Number of multicast packets sent*/
    volatile uint32_t txUcast;                  /**Number of unicast packets sent*/
    volatile uint32_t txOctets;                 /**Number of Tx packets*/

    volatile uint32_t rxBcast;                  /**Number of broadcast packets rcvd*/
    volatile uint32_t rxMcast;                  /**Number of multicast packets rcvd*/
    volatile uint32_t rxUcast;                  /**Number of unicast packets rcvd*/
    volatile uint32_t rxOctets;                 /**Number of Rx packets*/

    volatile uint32_t tx64byte;                 /**Number of 64 byte packets sent*/
    volatile uint32_t tx65_127byte;             /**Number of 65-127 byte packets sent*/
    volatile uint32_t tx128_255byte;            /**Number of 128-255 byte packets sent*/
    volatile uint32_t tx256_511byte;            /**Number of 256-511 byte packets sent*/
    volatile uint32_t tx512_1023byte;           /**Number of 512-1023 byte packets sent*/
    volatile uint32_t tx1024byte;               /**Number of 1024 and larger size packets sent*/

    volatile uint32_t rx64byte;                 /**Number of 64 byte packets rcvd*/
    volatile uint32_t rx65_127byte;             /**Number of 65-127 byte packets rcvd*/
    volatile uint32_t rx128_255byte;            /**Number of 128-255 byte packets rcvd*/
    volatile uint32_t rx256_511byte;            /**Number of 256-511 byte packets rcvd*/
    volatile uint32_t rx512_1023byte;           /**Number of 512-1023 byte packets rcvd*/
    volatile uint32_t rx1024byte;               /**Number of 1024 and larger size packets rcvd*/

    volatile uint32_t lateColl;                 /**Number of late collisions(Half Duplex)*/
    volatile uint32_t singleColl;               /**Number of single collisions (Half Duplex)*/
    volatile uint32_t multiColl;                /**Number of multiple collisions (Half Duplex)*/
    volatile uint32_t excessColl;               /**Number of excess collisions(Half Duplex)*/

    volatile uint32_t rxMisAlignmentFrames;     /**Number of non multiple of 8 byte frames rcvd*/
    volatile uint32_t stormPrevCounter;         /**Number of packets dropped because of Storm Prevention (broadcast)*/
    volatile uint32_t stormPrevCounterMC;         /**Number of packets dropped because of Storm Prevention (multicast)*/
    volatile uint32_t stormPrevCounterUC;         /**Number of packets dropped because of Storm Prevention (unicast)*/
    volatile uint32_t macRxError;               /**Number of MAC receive errors*/

    volatile uint32_t SFDError;                  /**Number of invalid SFD*/
    volatile uint32_t defTx;                    /**Number of transmissions deferred*/
    volatile uint32_t macTxError;               /**Number of MAC transmit errors*/
    volatile uint32_t rxOverSizedFrames;        /**Number of oversized frames rcvd*/
    volatile uint32_t rxUnderSizedFrames;       /**Number of undersized frames rcvd*/
    volatile uint32_t rxCRCFrames;              /**Number of CRC error frames rcvd*/

    volatile uint32_t droppedPackets;           /**Number of packets dropped due to a link down on opposite port*/
/* Debug variables, these are not part of standard MIB. Useful for debugging */
/* Reserved for future Use */
    volatile uint32_t txOverFlow;               /**Tx FIFO overflow count*/
    volatile uint32_t txUnderFlow;              /**Tx FIFO underflow count*/
    volatile uint32_t sqeTestError;             /**Number of MAC receive errors*/
    volatile uint32_t TXqueueLevel;             /**Current Tx queue level*/
    volatile uint32_t CSError;                  /**Number of carrier sense errors*/
} ICSS_EMAC_PruStatistics;
/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  This function initializes the ICSS_EMAC module
 */
void ICSS_EMAC_init(void);

/**
 *  \brief  This function de-initializes the ICSS_EMAC module
 */
void ICSS_EMAC_deinit(void);

/**
 *  \brief  Initialize the parmeters data structure with defaults
 *
 *  \param  params [out] Initialized parameters
 */
void ICSS_EMAC_Params_init(ICSS_EMAC_Params *params);

/**
 *  \brief  API to initialize and configure ICSS in MAC/Switch Mode
 *
 *  \param[in]  idx  ICSS_EMAC instance number
 *  \param[in]  params  Structure of type \ref ICSS_EMAC_Params
 *
 *  \return     #ICSS_EMAC_Handle in case of success, NULL otherwise
 */
ICSS_EMAC_Handle ICSS_EMAC_open(uint32_t idx, const ICSS_EMAC_Params *params);

/**
 *  \brief  API to stop MAC/Switch Mode
 *
 *  \param[in]  icssEmacHandle      Handle to ICSS_EMAC instance
 *
 */
void ICSS_EMAC_close(ICSS_EMAC_Handle icssEmacHandle);

/**
 *  \brief  IOCTL Function for ICSS EMAC
 *
 *          This Function can be used to configure various ICSS EMAC Functionalities.
 *          The Supported features are:
 *          (1)Port Enable/Disable       (2)StormControl
 *          (3)Learning configuration    (4)Statistics module
 *          (5)Multicast filterring      (6)VLAN filtering
 *
 *
 *  \param[in]      icssEmacHandle  Handle to ICSS_EMAC instance
 *  \param[in]      ioctlCommand    Command from \ref ICSS_EMAC_IOCTL_COMMANDS
 *  \param[in]      portNo          Port number for IOCTL
 *  \param[inout]   ioctlParams     Pointer to structure of type \ref ICSS_EMAC_IoctlCmd
 *                                  Please see following for setting ioctlParams.command
 *                                  \ref ICSS_EMAC_IOCTL_STORM_PREV_CTRL_COMMANDS,
 *                                  \ref ICSS_EMAC_IOCTL_LEARNING_CTRL_COMMANDS,
 *                                  \ref ICSS_EMAC_IOCTL_STATISTICS_COMMANDS,
 *                                  \ref ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_COMMANDS,
 *                                  \ref ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_COMMANDS
 *
 *
 *  \return         #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t ICSS_EMAC_ioctl(ICSS_EMAC_Handle icssEmacHandle,
                        uint32_t         ioctlCommand,
                        uint8_t          portNo,
                        void             *ioctlParams);

/**
 *  \brief  Retrieves a frame from a host queue and copies it in the allocated
 *          stack buffer
 *
 *  \param[in]  rxArg   Pointer to \ref ICSS_EMAC_RxArgument structure
 *  \param[in]  userArg Custom Rx packet callback packet options only required
 *                      for custom RxPacket implementations, default to NULL
 *                      when calling ICSS_EMAC_RxPktGet which is default Rx
 *                      Packet API
 *
 *  \return     Length of the frame received in number of bytes or #SystemP_FAILURE
 *              on failure
 */
int32_t ICSS_EMAC_rxPktGet(ICSS_EMAC_RxArgument *rxArg, void *userArg);

/**
 *  \brief  API to retrieve the information about the received frame which is
 *          then used to dequeue the frame from the host queues
 *
 *  \param[in]  icssEmacHandle  Handle to ICSS_EMAC instance
 *  \param[out] portNumber      Return pointer of port number where frame was
 *                              received
 *  \param[out] queueNumber     Return pointer of host queue where the received
 *                              frame is queued
 *
 *  \return     Length of packet or #SystemP_FAILURE if no packet found
 */
int32_t ICSS_EMAC_rxPktInfo(ICSS_EMAC_Handle icssEmacHandle,
                            int32_t          *portNumber,
                            int32_t          *queueNumber);

/**
 *  \brief  API to queue a frame which has to be transmitted on the
 *          specified port queue
 *
 *  \param[in]  txArg   Pointer to \ref ICSS_EMAC_TxArgument
 *  \param[in]  userArg Custom Tx packet callback packet options only required
 *                      for custom TxPacket implementations, default to NULL
 *                      when calling ICSS_EMAC_TxPacket which is default Tx
 *                      Packet API
 *
 *  \return     #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t ICSS_EMAC_txPacket(const ICSS_EMAC_TxArgument *txArg, void *userArg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* #ifndef ICSS_EMAC_H_ */
