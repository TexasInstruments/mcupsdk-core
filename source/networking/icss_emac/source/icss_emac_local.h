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

#ifndef ICSS_EMAC_LOCAL_H_
#define ICSS_EMAC_LOCAL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdbool.h>
#include <networking/icss_emac/icss_emac.h>
#include <drivers/hw_include/hw_types.h>
#include "icss_emac_learning.h"
#include "icss_emac_statistics.h"
#include "icss_emac_stormControl.h"
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/mdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ICSS_EMAC_INTR_SRC_LINK     (0U)
#define ICSS_EMAC_INTR_SRC_RX       (1U)
#define ICSS_EMAC_INTR_SRC_TX       (2U)

#define RX_TASK_STACK_SIZE              (8192)
#define TX_TASK_STACK_SIZE              (8192)
#define LINK_TASK_STACK_SIZE            (8192)

/***************************************************************************
*                       LINK/DUPLEX MACROS & MASKS                         *
*  Bits for Port Status and Duplexity. This is set by ARM and read by FW   *
*          Possible to add more info as bits 2-7 are free                  *
****************************************************************************
*/
#define ICSS_EMAC_PORT_LINK_MASK            (0x1U)
#define ICSS_EMAC_PORT_IS_HD_MASK         (0x2U)

/* ICSS_EmacPollLink can be called from ISR or from actual poll, these defines depect those 2 cases, local to LLD*/
#define ICSS_EMAC_POLL_FROM_ISR                     0x1U
#define ICSS_EMAC_POLL_FROM_POLL_LINK               0x2U

#define    ICSS_EMAC_Q_MAX_FILL_LEVEL_OFFSET     (6U)
#define    ICSS_EMAC_Q_OVERFLOW_CNT_OFFSET       (7U)

#define ICSS_EMAC_DEFAULT_FW_BD_SIZE                         (4U)            /* one buffer descriptor is 4 bytes */
#define ICSS_EMAC_DEFAULT_FW_BLOCK_SIZE                      (32U)           /* bytes derived from ICSS architecture */

/* Firmware feature set defines*/
#define ICSS_EMAC_FW_TTS_FEATURE_SHIFT                  ((uint32_t)0U)
#define ICSS_EMAC_FW_TTS_FEATURE_CTRL                   (((uint32_t)1U) << ICSS_EMAC_FW_TTS_FEATURE_SHIFT)

/* MODE indicates firmware running configuration which can be either dual emac(0U) or switch (1U) MODE */
#define ICSS_EMAC_FW_MODE_FEATURE_SHIFT                 ((uint32_t)1U)
#define ICSS_EMAC_FW_MODE_FEATURE_CTRL                  (((uint32_t)1U) << ICSS_EMAC_FW_MODE_FEATURE_SHIFT)

#define ICSS_EMAC_FW_VLAN_FEATURE_SHIFT                 ((uint32_t)2U)
#define ICSS_EMAC_FW_VLAN_FEATURE_CTRL                  (((uint32_t)1U) << ICSS_EMAC_FW_VLAN_FEATURE_SHIFT)

#define ICSS_EMAC_FW_STORM_PREVENTIION_FEATURE_SHIFT    ((uint32_t)3U)
#define ICSS_EMAC_FW_STORM_PREVENTIION_FEATURE_CTRL     (((uint32_t)1U) << ICSS_EMAC_FW_STORM_PREVENTIION_FEATURE_SHIFT)

#define ICSS_EMAC_FW_PROMISCOUS_MODE_FEATURE_SHIFT    ((uint32_t)19U)
#define ICSS_EMAC_FW_PROMISCOUS_MODE_FEATURE_CTRL     (((uint32_t)1U) << ICSS_EMAC_FW_PROMISCOUS_MODE_FEATURE_SHIFT)

#define ICSS_EMAC_FW_PTP_FEATURE_SHIFT                ((uint32_t)8U)
#define ICSS_EMAC_FW_PTP_FEATURE_CTRL                 (((uint32_t)1U) << ICSS_EMAC_FW_PTP_FEATURE_SHIFT)

#define ICSS_EMAC_FW_MULTICAST_FILTER_FEATURE_SHIFT   ((uint32_t)26U)
#define ICSS_EMAC_FW_MULTICAST_FILTER_FEATURE_CTRL    (((uint32_t)1U) << ICSS_EMAC_FW_MULTICAST_FILTER_FEATURE_SHIFT)

#define ICSS_EMAC_FW_VLAN_FILTER_FEATURE_SHIFT        ((uint32_t)27U)
#define ICSS_EMAC_FW_VLAN_FILTER_FEATURE_CTRL         (((uint32_t)1U) << ICSS_EMAC_FW_VLAN_FILTER_FEATURE_SHIFT)

/**This value in the MDIO Reg means 10 mbps mode is enabled*/
#define Ten_Mbps  0xa
/**This value in the MDIO Reg means 100 mbps mode is enabled*/
#define Hundred_Mbps 0x64

#define LINK0_PRU_EVT_MASK              (0x200U)
#define LINK1_PRU_EVT_MASK              (0x200000U)
#define TX_COMPLETION0_PRU_EVT_MASK        (((uint32_t)1U) << 22)
#define TX_COMPLETION1_PRU_EVT_MASK        (((uint32_t)1U) << 23)
#define TTS_CYC0_PRU_EVT_MASK            (((uint32_t)1U) << 24)
#define TTS_CYC1_PRU_EVT_MASK            (((uint32_t)1U) << 25)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * @brief ICSS EMAC Rx Pkt Info  structure.
 */
typedef struct ICSS_EMAC_PktInfo_s
{
    /*!  port number where frame was received */
    uint32_t portNumber;
    /*! host queue where the received frame is queued*/
    uint32_t queueNumber;
    /*! address of read buffer in L3 address space*/
    uint32_t rdBufferL3Addr;
    /*! FDB lookup was succssful in firmware. FW Learning Only. */
    uint32_t fdbLookupSuccess;
    /*! Packet was flooded. FW Learning Only. */
    uint32_t flooded;
} ICSS_EMAC_PktInfo;

/**
 * @brief Queue Statistics
 */
typedef struct ICSS_EMAC_QueueStats_s
{
    /**queue raw count*/
    uint32_t rawCount;
    /**queue error count*/
    uint32_t errCount;
} ICSS_EMAC_QueueStats;

/**
 * @brief Queue Parameters
 */
typedef struct ICSS_EMAC_QueueParams_s
{
    /**Queue statistics*/
    ICSS_EMAC_QueueStats    qStat;
    /**buffer  offset*/
    uint16_t                buffer_offset;
    /**buffer descriptor offset*/
    uint16_t                buffer_desc_offset;
    /**queue descriptor offset*/
    uint16_t                queue_desc_offset;
    /**queue size*/
    uint16_t                queue_size;
} ICSS_EMAC_QueueParams;

/**
 * @brief Port parameters
 */
typedef struct ICSS_EMAC_PortParams_s
{
    /**pointer to PTCP packet mem*/
    uint8_t*                ptcpPktBuff;
    /**statistics   - raw*/
    uint32_t                rawCount;
    /**Error count*/
    uint32_t                errCount;
    /**Queues per port*/
    ICSS_EMAC_QueueParams   queue[ICSS_EMAC_NUMQUEUES];
} ICSS_EMAC_PortParams;

/*
*  @brief     ICSSEMAC_Object
*             Handle containing pointers to all modules as well as Initialization Config
*/
typedef struct ICSS_EMAC_Object_s
{
    /*! PRUICSS Handle details where the EMAC driver will be based on       */
    PRUICSS_Handle                      pruicssHandle;
    /*! Mac Table Pointer for Learning module. Not applicable for Emac mode */
    ICSS_EMAC_HashTable                 macTable[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /*! Pointer All Driver specific Callback  structure                     */
    ICSS_EMAC_CallBackObject            callBackObject;
    /*! Pointer to  Emac driver Firmware statistics structure               */
    ICSS_EMAC_HostStatistics            hostStat[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /*! Pointer to Storm Prevention structure */
    ICSS_EMAC_StormPrevention           stormPrev[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /*! Rx Semaphore Handle for the emac instance.Required for receiving packets */
    SemaphoreP_Object                   rxSemaphoreObject;
    /*! Tx Complete Semaphore Handle for the emac instance.Required for notification of Transmit Complete indication from  PRUICSS firmware */
    SemaphoreP_Object                   txSemaphoreObject;
    /*! Link Semaphore Handle for the emac instance. Required for updating Phy speed and duplex configuration  */
    SemaphoreP_Object                   linkSemaphoreObject;
    /*! Rx interrupt handler */
    HwiP_Object                         rxInterruptObject;
    /*! Link interrupt handler */
    HwiP_Object                         linkInterruptObject;
    /*! Tx Complete interrupt handler */
    HwiP_Object                         txInterruptObject;
    /*! Link status for the ports */
    uint8_t                             linkStatus[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /*! Previous Link status for the ports .Used to update port status*/
    uint8_t                             prevlinkStatus[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    /*!    ICSS Revision Information */
    uint32_t                            icssRevision;
    ICSS_EMAC_PortParams                switchPort[ICSS_EMAC_MAX_PORTS_PER_INSTANCE + 1];
    ICSS_EMAC_FwStaticMmap              fwStaticMMap;
    ICSS_EMAC_FwDynamicMmap             fwDynamicMMap;
    ICSS_EMAC_FwVlanFilterParams        fwVlanFilterParams;
    bool                                fwVlanFilterConfigProvided;
    ICSS_EMAC_FwMulticastFilterParams   fwMulticastFilterParams;
    bool                                fwMulticastFilterConfigProvided;
    ETHPHY_Handle                       ethphyHandle[ICSS_EMAC_MAX_PORTS_PER_INSTANCE];
    uint8_t                             macId[6];
    /* flag to indicate module is open */
    bool                                isOpen;
    TaskP_Object                        linkTaskObject;
    TaskP_Object                        rxTaskObject;
    TaskP_Object                        txTaskObject;
    uint32_t                            linkTaskStack[LINK_TASK_STACK_SIZE/sizeof(uint32_t)]    __attribute__((aligned(32)));
    uint32_t                            rxTaskStack[RX_TASK_STACK_SIZE/sizeof(uint32_t)]        __attribute__((aligned(32)));
    uint32_t                            txTaskStack[TX_TASK_STACK_SIZE/sizeof(uint32_t)]        __attribute__((aligned(32)));
} ICSS_EMAC_Object;

/* NRT Buffer descriptor definition
 Each buffer descriptor points to a max 32 byte block and has 32 bit in size to have atomic operation.
 PRU can address bytewise into memory.
 Definition of 32 bit desriptor is as follows

 Bits       Name            Meaning
 ==================================================================================================
 0..7   Index           points to index in buffer queue, max 256 x 32 byte blocks can be addressed
 8..12  Block_length    number of valid bytes in this specific block. Will be <=32 bytes on last block of packet
 13     More            "More" bit indicating that there are more blocks for the
 14         Shadow          inidcates that "index" is pointing into shadow buffer
 15     TimeStamp       indicates that this packet has time stamp in seperate buffer - only needed of PTCP runs on host
 16..17 Port            different meaning for ingress and egress, ingress Port=0 inidcates phy port 1 and Port = 1
                            inidcates phy port 2. Egress: 0 sends on phy port 1 and 1 sends on phy port 2. Port = 2 goes
                             over MAC table look-up
 18..28   Length            11 bit of total packet length which is put into first BD only so that host access only one BD
 29     VlanTag         indicates that packet has Length/Type field of 0x08100 with VLAN tag in following byte
 30     Broadcast       inidcates that packet goes out on both physical ports,  there will be two bd but only one buffer
 31     Error           indicates there was an error in the packet
  */

/* NRT Queue Defintion
 Each port has up to 4 queues with variable length. The queus is processed as ring buffer with read and write pointer.
 Both pointer are address pointers and increment by 4 for each buffer descriptor/position.
 Queue has a length defined in constants and a status. Status is defined as described below

 Bits       Name            Meaning
 =====================================================================================================
 0      Busy_M          This queue is busy by the master port which is the PRU receiving packets from the master
 1      Collision       Slave is/has written into shadow queue, both descriptors and data.
 2      Overflow        there was not enough space to write to queue and packet was discarded
 4..7       Reserved        reserved

 There is busy slave flag in different byte address to grant access to queue to master in case
 of simultaneous access. Host will alwasys be slave in this case. The PRU which is sending the
 packet on phy port will be the master. When both PRUs wants to write to host queues PRU0
 is master and PRU1 is slave.

 Bits       Name            Meaning
 =====================================================================================================
 0      Busy_S          This queue is busy by the master port which is the PRU receiving packets from the master
 1..7       Reserved

 Length is the number of 32 byte blocks per queue. max_fill_level tells the minimum distance between write and read pointer.
 over_flow_cnt tells how many times the write pointer runs into the read_pointer.
  */

typedef struct ICSS_EMAC_Queue_s
{
    uint16_t    rd_ptr;
    uint16_t    wr_ptr;
    uint8_t     busy_s;
    uint8_t     status;
    uint8_t     max_fill_level;
    uint8_t     overflow_cnt;
} ICSS_EMAC_Queue;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  Task to enable copying of Rx data from queues to DDR. This is a
 *          function that calls another function to empty the queues.
 *
 *  \param[in]  a0  Generic argument
 *  \param[in]  a1  Generic argument
 *
 */
void ICSS_EMAC_osRxTaskFnc(void *args);

void ICSS_EMAC_osLinkTaskFnc(void *args);

/**
 *  \brief  Task to enable receiving Indication of transmit packet complete by
 *          PRU-ICSS firmware
 *
 *  \param[in]  a0  Generic argument
 *  \param[in]  a1  Generic argument
 *
 */
void ICSS_EMAC_osTxTaskFnc(void *args);

/**
 * \brief   Receive frame interrupt service routine
 *
 *  \param[in]  args    parameter list to interrupt service routine
 *
 */
void ICSS_EMAC_rxInterruptHandler(void *args);

/**
 *  \brief  Main TX interrupt service routine
 *          Handles TX Completion and TTS Cyclic Packet
 *          Insertion Notification interrupts
 *
 *  \param[in]  args    arguments if any
 *
 */
void ICSS_EMAC_txInterruptHandler(void *args);

/**
* \brief    Link change status interrupt for Port 0 and Port 1 calls a user
*           callback if defined to provide link info to stack
*
*  \param[in]   args    arguments if any
*
*/
void ICSS_EMAC_linkISR(void* arg);

void ICSS_EMAC_memInitToZero(uint32_t *addr, uint32_t size);

/**
 *  \name ICSS_EMAC_memInit
 *  @brief Function to clear ICSS shared,pru memory and L3 ocmc ram
 *
 *  @param none
 *
 *  \return 0 on success
 *
 */
uint8_t ICSS_EMAC_memInit(ICSS_EMAC_Handle icssEmacHandle);

void ICSS_EMAC_calcPort0BufferOffset(ICSS_EMAC_Handle icssEmacHandle,
                                     uint32_t bufferOffsets[ICSS_EMAC_NUMQUEUES],
                                     uint32_t bdOffsets[ICSS_EMAC_NUMQUEUES]);

void ICSS_EMAC_calcPort1BufferOffset(ICSS_EMAC_Handle icssEmacHandle,
                                     uint32_t bufferOffsets[ICSS_EMAC_NUMQUEUES],
                                     uint32_t bdOffsets[ICSS_EMAC_NUMQUEUES]);

void ICSS_EMAC_calcPort2BufferOffset(ICSS_EMAC_Handle icssEmacHandle,
                                     uint32_t bufferOffsets[ICSS_EMAC_NUMQUEUES],
                                     uint32_t bdOffsets[ICSS_EMAC_NUMQUEUES]);

void ICSS_EMAC_clearStatistics(ICSS_EMAC_Handle icssEmacHandle);

int32_t ICSS_EMAC_portInit(ICSS_EMAC_Handle icssEmacHandle);

static void ICSS_EMAC_pruicssCfgInit(ICSS_EMAC_Handle icssEmacHandle);

static void ICSS_EMAC_pruicssMiiRtCfgInit(ICSS_EMAC_Handle icssEmacHandle);

uint8_t ICSS_EMAC_switchConfig(ICSS_EMAC_Handle icssEmacHandle);

int8_t ICSS_EMAC_switchInit(ICSS_EMAC_Handle            icssEmacHandle,
                            const PRUICSS_IntcInitData  *pruicssIntcInitData);

void ICSS_EMAC_mdioIntrEnableSwitch(uint8_t             portNum,
                                    ICSS_EMAC_Handle    icssEmacHandle);

uint32_t ICSS_EMAC_calcTotalBufferPoolSize(ICSS_EMAC_Handle icssEmacHandle);

uint8_t ICSS_EMAC_hostMemInit(ICSS_EMAC_Handle icssEmacHandle);

uint8_t ICSS_EMAC_hostConfig(ICSS_EMAC_Handle icssEmacHandle);

void ICSS_EMAC_hostInit(ICSS_EMAC_Handle icssEmacHandle,
                        const PRUICSS_IntcInitData *pruicssIntcInitData);

void ICSS_EMAC_portMemInit(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle);

uint8_t ICSS_EMAC_macConfig(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle);

int8_t ICSS_EMAC_macInit(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle);

void ICSS_EMAC_mdioIntrEnable(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle);

void ICSS_EMAC_initLinkState(ICSS_EMAC_Handle   icssEmacHandle,
                            uint8_t             interfaceId,
                            uint8_t             portNo);

/**
 *  @brief Function to initialize Rx semaphore and Task
 *  @internal
 *  @param icssEmacHandle pointer to ICSS EMAC handle
 *  @retval 0 on success
 */
int32_t ICSS_EMAC_osInit(ICSS_EMAC_Handle icssEmacHandle);

void ICSS_EMAC_mdioIntrDisableSwitch(uint8_t portNum,
                                     ICSS_EMAC_Handle icssEmacHandle);

void ICSS_EMAC_mdioIntrDisable(uint8_t portNum,
                               ICSS_EMAC_Handle icssEmacHandle);

int32_t ICSS_EMAC_osDeinit(ICSS_EMAC_Handle icssEmacHandle);

int32_t ICSS_EMAC_validateFeatureSet(ICSS_EMAC_Handle icssEmacHandle,
                                     uint8_t portNo,
                                     uint32_t featureCtrl);

int32_t ICSS_EMAC_promiscuousModeInit(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle);

int32_t ICSS_EMAC_promiscuousModeDeinit(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle);

static void ICSS_EMAC_multicastFilterFeatureCtrl(ICSS_EMAC_FwMulticastFilterParams    *pMulticastFilterParams,
                                                 uintptr_t                          dataRamAddr,
                                                 uint8_t                            value);

static void ICSS_EMAC_multicastFilterOverrideHashmask(ICSS_EMAC_FwMulticastFilterParams   *pMulticastFilterParams,
                                                      uintptr_t                         dataRamAddr,
                                                      uint8_t                           *mask);

/* multicastAddr (48 bit) & multicastFilterMask (48 bit) | XOR the result to obtain a hashVal
 *
 * Update the byte in multicast table as specified by command. Command can take 2 values:
 *          0 : allow packet to host | ADD_MULTICAST_MAC_ID
 *          1 : do not allow packet to host | REMOVE_MULTICAST_MAC_ID
 * */
static void ICSS_EMAC_multicastFilterUpdateMacId(ICSS_EMAC_FwMulticastFilterParams   *pMulticastFilterParams,
                                                 uintptr_t                          dataRamAddr,
                                                 uint8_t                            *multicastAddr,
                                                 uint8_t                            command);

int32_t ICSS_EMAC_multicastFilterConfig(ICSS_EMAC_FwMulticastFilterParams *pMulticastFilterParams,
                                        uintptr_t                       dataRamAddr,
                                        uint8_t                         ioctlCmd,
                                        void                            *ioctlVal);

/* Command can take the values as under ioctlCmd and update the control under
*  firmware for VLAN filtering.
*/
static void ICSS_EMAC_vlanFilterFeatureCtrl(ICSS_EMAC_FwVlanFilterParams  *pVlanFilterParams,
                                            uintptr_t                   dataRamAddr,
                                            uint8_t                     ioctlCmd);

/* This function implements the updating (adding/removing) the VID for vlan filtering in the VLAN
 * filter table
 */
static int32_t ICSS_EMAC_vlanFilterUpdateVID(ICSS_EMAC_FwVlanFilterParams     *pVlanFilterParams,
                                             uintptr_t                      dataRamAddr,
                                             uint8_t                        ioctlCmd,
                                             uint16_t                       *vlanId);

/* Implements the configuratins for VLAN filtering feature */
int32_t ICSS_EMAC_vlanFilterConfig(ICSS_EMAC_FwVlanFilterParams   *pVlanFilterParams,
                                   uintptr_t                    dataRamAddr,
                                   uint8_t                      ioctlCmd,
                                   void                         *ioctlVal);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ICSS_EMAC_LOCAL_H_ */
