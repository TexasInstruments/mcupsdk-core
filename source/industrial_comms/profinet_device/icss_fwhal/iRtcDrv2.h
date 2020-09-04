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

#ifndef IRTCDRV2_H_
#define IRTCDRV2_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                          Doxygen                                           */
/* ========================================================================== */

/**
 *  \defgroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE APIs for Profinet Device FWHAL
 *  \ingroup INDUSTRIAL_COMMS_MODULE
 *
 *  Profinet Device FWHAL(Firmware and Hardware Abstraction Layer) APIs implement the key
 *  interface between Profinet firmware and slave stack.
 *
 *  @{
 */

/**
 * \defgroup PN_CPM_PPM_MANAGEMENT CPM/PPM Management APIs
 * \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "firmware/icss_pn_rtcx.h"
#include "PN_Handle.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \internal
 * \brief Used to specify Shadow list when reading or writing Descriptors
 */
#define SHADOW_LIST 0
/**
 * \internal
 * \brief Used to specify Active list when reading or writing Descriptors
 */
#define ACTIVE_LIST 1
/* currently we support 8 PPM split into two blocks (NO_PPM/2)*/
#define HALFSIZE    4

/** \addtogroup PN_IRT_TRIPLE_BUFFER
 @{ */
/**
 *  \brief Enumerates the Buffer indexes of Triple buffers.
 *  Buffer lock needs to be aligned with firmware
 */
typedef enum
{
    BUFF1 = 0,  /**< Buffer Index 0 */
    BUFF2,      /**< Buffer Index 1 */
    BUFF3,      /**< Buffer Index 2 */
    NOBUFF      /**< No Buffer  */
} buffLocks;
/**
@}
*/

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint32_t pRtcConfig;        /**< Stores PRUSS DataRAM0 */
uint32_t ocmcBaseAddress;   /**< Stores the OCMC Baseaddress */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 */
/**
 * \internal
 * \brief Index definition for CPM/PPM descriptors \n
 *
 * \details
 *     ------------ \n
 *     |          |   red port 1    <- red start index = 0 \n
 *     |----------| \n
 *     |          |   red port 2    <- redP2 \n
 *     |----------| \n
 *     |          |   green port 1  <- green \n
 *     |----------| \n
 *     |          |   green port 2  <- greenP2 \n
 *     ------------ \n
 *                                  <- end   (always number of entries+1) \n
 *
 * \image html Profinet_Device_desc_phases.png
 */
typedef struct listIndex
{
    uint8_t redP2;      /**< Start of port 2 red packets */
    uint8_t green;      /**< Start of green packets */
    uint8_t greenP2;    /**< Start of green port 2 */
    uint8_t end;        /**< End of list */
} t_listIndex;


/**
 * \internal
 * \brief buffer struct for a single block in CPM or PPM buffer mem
 */
typedef struct rtcMemBuff
{
    uint8_t
    *addr[3];        /*/< triple buffer start addresses - fixed after init - ARM view*/
    uint16_t
    regbase;        /*/< 16 bit PRU base address of tripple buffer addresses - CPM only*/
    uint8_t     slotUsed;       /*/< true if this slot is allocated*/
} t_rtcMemBuff;

/** \addtogroup PN_CPM_PPM_MANAGEMENT
 @{ */

/**
 * \internal
 * \brief block of PPM buffer memory
 */
typedef struct ppmBlock
{
    uint8_t
    *blockStart;             /*/< block start address (physical mem)*/
    t_rtcMemBuff
    slots[HALFSIZE];        /*/< each buffer can have maximum of 4 slots*/
} t_ppmBlock;

/**
 * \internal
 * \brief block of CPM buffer memory
 */
typedef struct cpmBlock
{
    uint8_t
    *blockStart;             /*/< block start address (physical mem)*/
    t_rtcMemBuff    slots[NO_CPM];          /*/< single buffer takes all CPM*/
} t_cpmBlock;


/**
 * \brief real-time packet structure
 */
typedef struct rtcPacket
{
    uint32_t
    frameOffset;    /*/< frame send offset (0 indicates green PPM packet!)*/
    t_rtcMemBuff    *pBuffer;       /*/< keep full buffer mem object pointer*/
    uint16_t        frameId;        /*/< main identifier for all PPM/CPM*/
    uint16_t        length;         /*/< length of packet (without space for CRC)*/
    uint16_t        reduRatio;      /*/< reduction ratio (green or red)*/
    uint16_t        phase;          /*/< assigned phase*/
    uint8_t         type;           /*/< CPM or PPM packet*/
    uint8_t         port;           /*/< where we want to sent to (PORT1 or PORT2)*/
    uint8_t
    crGroup;        /*/< the group we belong too (1-8, 0 not defined)*/
    uint8_t
    lIndex;         /*/< current index in descriptor list (0xFF = invalid)*/
    uint8_t         isActive;       /*/< store activation state*/
    uint8_t         block;          /*/< PPM only: store the buffer block number*/
    uint8_t         next;           /*/< tripple buffer NEXT index*/
    uint8_t         last;           /*/< tripple buffer LAST index*/
    uint8_t         validLast;      /*/< new valid LAST buffer available (=1)*/
    uint8_t
    proc;           /*/< tripple buffer PROC index -previously "buffIndex"*/
    void           *userRef; /*/< Reference for Application data"*/
} t_rtcPacket;

/**
 * \internal
 * \brief Descriptor for CPM/PPM Lists
 */
typedef struct descList
{
    uint32_t *pDescs[2];            /**< Desc list 0/1 pointer. Pointers to type \ref t_ppmDesc or \ref t_cpmDesc. This is a chain of \ref NO_PM descriptors. Two pointers available for Active/Shadow list. In the current implementation Shadow list not applicatble to CPM */
    uint8_t  *rtc_index_ptr[2];     /**< Pointers to phase index lists 1 and 2. Type \ref t_listIndex */
    uint8_t
    *pActive;            /**< Pointer to active byte. Indicates if the list is active or not */
    uint8_t
    *pStatus;            /**< Pointer to status byte. Can have values \n \ref RTC_PPM_OK \n \ref RTC_PPM_ERROR \n \ref RTC_CPM_RUN \n \ref RTC_CPM_FAILURE */
    uint8_t    *pEvent;             /**< Pointer to events/IRQ info */
    uint16_t  redFID;           /**< Store FID of single red frame in list */
    uint8_t size;               /**< List(s) size in bytes */
    uint8_t   shadow;               /**< Value of current shadow index */
    t_rtcPacket
    *lInitiate;   /**< last initiator object pointer. Only one toggle is allowed at a time. If this have a valid pointer, then a toggle is in process(by PRU) and ARM cannot toggle at this time. This is set during \ref PN_togglePpmList and cleared in \ref RTC_NOTIFY_PPM_LIST_CHANGE in \ref PN_dhtIsrHandler */
    uint8_t mrpFlag;            /**< MRP mode flag */
    uint8_t listToggleReq;     /**< PPM List Toggle request initiated by the stack */
} t_descList;

/**
 * \internal
 * \brief Main driver configuration structure.
 * This structure provides data to update lists, descriptor updates etc.
 */
typedef struct cfgPN
{
    t_rtcPacket
    cpmPkts[NO_CPM];    /*/< All CPM pkt config object internal to driver*/
    t_rtcPacket
    ppmPkts[NO_PPM];    /*/< All PPM pkt config object internal to driver*/
    uint16_t
    maxCpmSize;         /*/< maximum allowed data size for each CPM - incl overhead*/
    uint16_t
    maxPpmSize;         /*/< maximum allowed data size for each PPM - incl overhead*/
    uint8_t         cfgAR;              /*/< number of configured ARs*/
} t_cfgPN;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 * \brief Configures the driver memory
 *
 * \param pnHandle Profinet Handle
 * \param
 *    ar    number of allowed number of ARs (symmetrical PPM/CPM)
 *          valid values only: 1, 2, 4, 8
 * \param   size    RTC payload size (see below)
 *
 * \retval negative value on error, zero on success
 *
 * configuration function for static PPM buffer allocation
 * currently only supports symmetric PPM config with same max size
 * e.g.
 *
 * \arg 2 * 1440 bytes payload
 * \arg 4 * 720 bytes
 * \arg 8 * 360 bytes (lower size is ok...)
 *
 * PPM buffers will be allocated in buffer blocks 0/1 evenly distributed
 * Start addresses are only valid for PRU1/Port 1 TX...
 * Port 2 PPM needs to be adapted with correct offset at creation of PPM
 * descriptor as the port is not known yet.
 */
int32_t PN_cfgRtcMem(PN_Handle pnHandle, uint8_t ar, uint16_t size);

/**
 * \brief De-configures the driver memory
 * \param[in] pnHandle PROFINET Handle
 * \retval negative value on error, zero on success
 */
int32_t PN_deCfgRtcMem(PN_Handle pnHandle);

/**
 * \brief Initialize packet according to type
 * Utility API for stack. Doesn't update anything in driver
 *
 * \param[out] pPkt pointer to packet object to be initialized
 * \param[in] type  CPM or PPM
 * \retval 0 on success
 * \retval <0 if failure
 */
int32_t PN_initPacket(t_rtcPacket *pPkt, uint8_t type);

/**
 * \brief Allocate a packet object from internal array
 * Initializes pPkt with \ref PN_initPacket
 * Find the next freeslot in \ref t_ppmBlock \ref t_cpmBlock, update the pBuffer address
 *
 * \param pnHandle Profinet Handle
 * \param[out] pPkt a packet object pointer
 * \param type  \ref CPM or \ref PPM
 * \retval index in the list on success
 * \retval <0 if failure
 */
int32_t PN_allocPkt(PN_Handle pnHandle, t_rtcPacket **pPkt, uint8_t type);

/**
 * \brief Free a packet object for re-use
 * Utility API for stack. Doesn't update anything in driver.
 *
 * \param[out] pPkt     Pointer to previously allocated \ref t_rtcPacket
 * \retval 0 on success
 * \retval <0 if failure
 */
int32_t PN_freePkt(t_rtcPacket *pPkt);

/**
 * \brief Inserts PPM descriptor data into the lists
 * \details This now manages single RED packet and index list processing in general \n
 * It will call \ref PN_togglePpmList() in case of a good setup
 *
 * \param[in] pnHandle Profinet Handle
 * \param ppmPkt pointer to PPM packet to be inserted in list
 * \param legMode TRUE(1): enables legacy mode support (same packet sent as RTC1 with
 *                RR=128 during startup)
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_insPpmList(PN_Handle pnHandle, t_rtcPacket *ppmPkt, uint8_t legMode);

/**
 * \brief Deletes a descriptor from PPM or CPM list
 * \details Stop sending the CPM/PPM packet (clears active bit) and clear buffer usage \n
 * Clears the ARgroup relation \n
 * Remove packet from shadow descriptor list and toggle lists \n
 * \param[in] pnHandle Profinet Handle
 *
 * \param pmPkt pointer to RTC packet to be deleted from list
 *
 * \retval 0 on success
 * \retval <0 if failure
 */
int32_t PN_delPmList(PN_Handle pnHandle, t_rtcPacket *pmPkt);

/**
 * \brief Insert a packet into descriptor list and lock this buffer, indicating that ARM is consuming this buffer \n
 * \details Currently we do not distinguish between green and red packets
 * also there are no checks on receive at correct time or phase
 * so that fields are currently not set!
 * \param[in] pnHandle Profinet Handle
 * \param cpmPkt pointer to CPM packet to be inserted in list
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_insCpmList(PN_Handle pnHandle, t_rtcPacket *cpmPkt);

/**
 * \internal
 * \brief Control CPM activation state
 * \details Single active byte required for CPM \n
 * Here we also manage the AR group entries for CPM \n
 * \param pPkt  pointer to related packet struct
 * \param[in] pnHandle Profinet Handle
 * \param val   1 - activate, 0 - deactivate
 *
 * \retval listindex on success
 * \retval <0 if failure
 */
int32_t PN_setCpmState(PN_Handle pnHandle, t_rtcPacket *pPkt, uint8_t val);

/**
 * \internal
 * \brief control PPM activation state
 * \details New scheme uses a SHADOW active byte \n
 * Here we also manage the AR group entries for PPM \n
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 * \param pPkt  pointer to related packet struct
 *
 * \retval listIndex on success
 * \retval <0 if failure
 */
int32_t PN_setPpmState(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       t_rtcPacket *pPkt);

/**
 * \brief Update PPM descriptor to new buffer address (based on ARM buffer address)
 * \details Updates the \ref t_ppmDesc FrameIndex and FrameReference pointers in the PPM \ref ACTIVE_LIST
 * \param pnHandle Profinet Handle
 * \param[in] ppmPkt current packet data object pointer
 *
 * \retval 0 on success
 * \retval <0 if failure
 */
int32_t PN_chgPpmBuffer(PN_Handle pnHandle, t_rtcPacket *ppmPkt);

/**
 * \internal
 * \brief Access to local PROFINET configuration data
 *
 * \retval pointer to configuration struct
 */
t_cfgPN *PN_getConfig(void);

/**
 *  \brief Callback function for processing Profinet IRQs
 *
 *  \param  arg
 *  \param  arg2
 *  \retval none
 */
typedef void (*pnCallBack_t)(void *arg, uint8_t arg2);

/**
 * \brief Initializes RTC driver
 * \details Initializes the lists, clears the data \n
 * Sets the base clock in IEP \n
 * Initializes the Interrupts \n
 *
 * \param[in] pnHandle Profinet Handle
 *
 * \retval 0 on Success
 * \retval <0 if Failure
 *
 */
int32_t PN_initRtcDrv(PN_Handle pnHandle);

/**
 * \internal
 * \brief Read back the descriptor list corresponding to the direction
 *
 *  \param pList        Returns the pointer to the requested descriptor list
 *  \param dir          Direction
 *                          Valid values for 'dir' - CPM or PPM
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_getList(t_descList **pList, uint8_t dir);

/**
 * \internal
 * \brief Clears the index of PPM or CPM list. Clears active bits and resets all data
 *              This does not clear buffer and desc memory
 *
 *  \param[in] pnHandle Profinet Handle
 *  \param list     Direction
 *                      Valid values for 'list' - CPM or PPM
 *
 * \retval 0 on Success
 * \retval <0 if Failure
 *
 */
int32_t PN_clearList(PN_Handle pnHandle, uint8_t list);

/**
 * \internal
 * \brief  Remove AR group links
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 *  \param ARgroup      AR group number
 *                          Valid Range 0-7
 *  \param pmNum        CPM/PPM number
 *                          Valid Range 0-7
 *  \param dir          Direction
 *                          Valid values for 'dir' - CPM or PPM
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_resetARlink(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       uint8_t ARgroup, uint8_t pmNum, uint8_t dir);

/**
@}
*/


/** \addtogroup PN_IRT_TRIPLE_BUFFER
 @{ */
/**
 * \brief  Locks CPM Buffer indicating that ARM is consuming the particular buffer. PRU cannot write to this buffer
 *  \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 *  \param[in] pos      CPM descriptor position
 *                      Valid Range 0-7
 *  \param[in] buff     Indicates the index of Buffer
 *                      Valid values for "buff" are
 *                          \ref BUFF1 - Indicating the first buffer
 *                          \ref BUFF2 - Indicating the second buffer
 *                          \ref BUFF3 - Indicating the third buffer
 *                          \ref NOBUFF - Indicates No buffer selected
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_cpmBuffLock(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       uint8_t pos, buffLocks buff);

/**
@}
*/

/**
 * \brief Registers callback function required for PPM processing
 *
 * \param pnHandle Profinet Handle
 *  \param[in] callBack Callback function pointer
 *
  */
void PN_registerPpmCall(PN_Handle pnHandle, pnCallBack_t callBack);

/**
 * \brief Registers callback function required for CPM processing
 * \param pnHandle Profinet Handle
 *  \param[in] callBack Callback function pointer
 *
 */
void PN_registerCpmCall(PN_Handle pnHandle, pnCallBack_t callBack);

/**
 * \brief Registers callback function required for Stat processing
 * \param pnHandle Profinet Handle
 * \param[in] callBack Callback function pointer
 *
 */
void PN_registerStatCall(PN_Handle pnHandle, pnCallBack_t callBack);

/** \addtogroup PN_IRT_LEGACY
 @{ */
/** \brief possible states for legacy suppport state machine
 *  \pre IRT_LEGACY_STARTUP_SUPPORT defined
 * */
typedef enum
{
    NOINIT,         /**< not initialized yet */
    READY,          /**< initialized */
    SENDPPM,        /**< sending RTC3 packet in green at RR=128 */
    ABORT           /**< error case */
} tLegStates;
/**
 *  \brief Callback function for processing Profinet IRQs
 *
 *  \param  arg
 *  \param  arg2
 *
 * \pre IRT_LEGACY_STARTUP_SUPPORT defined
 */
typedef void (*pnLegCallBack_t)(void *arg, void *arg2);

/**
 * \brief Registers callback function for legacy state
 * This callback is used to change the Legacy state machine
 *
 * \param pnHandle Profinet Handle
 * \param[in] callBack Callback function pointer
 *
 * \pre IRT_LEGACY_STARTUP_SUPPORT defined
 *
 */
void PN_registerSetState(PN_Handle pnHandle, pnLegCallBack_t callBack);

/**
 * \brief Registers callback function for packet pointer
 * This callback is used to configure the packet send by the IRT state machine
 * \param pnHandle Profinet Handle
 * \param[in] callBack Callback function pointer
 *
 * \pre IRT_LEGACY_STARTUP_SUPPORT defined
 *
 */
void PN_registerSetPkt(PN_Handle pnHandle, pnLegCallBack_t callBack);
/**
@}
*/


/**
 * \internal
 * \brief Clears the Interrupt Event in ICSS INTC SECR0
 * \param[in] pruicssHwAttrs PRUICSS HW Attributes for base addresses
 *  \param irq_num   Interrupt number
 */
void PN_clearPruIRQ(PRUICSS_HwAttrs const *pruicssHwAttrs,
                    uint8_t irq_num);      /* event clearing*/


/**
 * \brief Sets Profinet base clock in multiples of 31.25us
 *
 * \param[in] pnHandle  Profinet Handle
 * \param     factor    Clock send factor: min value 8, max value 128
 *
 * \retval Slot number (>=0)
 * \retval <0 if failure
 *
 */
int32_t PN_setBaseClock(PN_Handle pnHandle, uint16_t factor);

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* IRTCDRV2_H_ */
