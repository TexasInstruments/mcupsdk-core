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

#ifndef IRTCDRV_H_
#define IRTCDRV_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "iRtcDrv2.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifdef RTC_DEBUG
void newLogPkt(uint32_t msg);

typedef enum
{
    INSPPM,
    INSCPM,
    DELPM
} eDebugAPI;
#endif

/**
 * \internal
 * \def DESC_WORD_LENGTH
 *  length of descriptors in 32 bit words
 */
#define DESC_WORD_LENGTH 4

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \addtogroup PN_CPM_PPM_MANAGEMENT
 @{ */
/**
 * \internal
 * \brief Internal only descriptor defs - these need to match as closely as possible to PRU defs
 * \details We now have different descriptors for CPM and PPM \n
 * Bit exact so direct copy possible - exactly 16 bytes \n
 * \image html cpm_desc.png \n
 */
typedef struct
#ifndef DOXYGEN
__attribute__((packed))
#endif
cpmDesc         /* CPM*/
{
    uint16_t
    FrameReference;         /**< 16 bit offset into start address pointer for triple buffer CPM (fixed address). Points into cpm triple buffer start addresses */
    uint16_t
    FrameLength;            /**< 11 bits of frame length including VLAN and FCS, set by host only. PRU may verify incoming frame on length which needs to be flexible in terms of stripped VLAN tag. Bit12.15 need to be zero*/
    uint16_t
    FrameDataPointer;       /**< 16 bit absolute address pointing to Profinet data of current index buffer. This pointer masks VLAN tag offset. Points to first byte after FID. \n \image html Profinet_Device_cpm_desc_fdp.png */
    uint8_t FrameIndex;                 /**< Current index of data source which is PRU on CPM \n
    Two bits used for pointing to index 1,2,3. Bit 2-7 needs to be 0. \n 00 = buffer 0 \n
          01 = buffer 1 \n
          10 = buffer 2 */
    uint8_t   FrameFlags2;              /**< Bit 0: 1 = VLAN tag present \n
          PORT: Bit 1: 0 = Port 1, 1: Port 2 \n
          RCV_PERIOD: Bit 2: 0 = red, 1 = green (error detection on host. optional for AM335x) \n
          Bit 3..7: reserved  */
    uint16_t
    FrameId;                /**< FrameID as provided by engineering. Set by host */
    uint8_t   FrameFlags1;              /**< TODO */
    uint8_t Reserved;                   /**< TODO */
    uint16_t
    RR;                     /**< Reduction Ratio for red period is in (RR-1) where RR is 1,2,4,8,16. \n
    Reduction Ration for green period is (RR-1) where RR is 1,2 ,4 …512. */
    uint16_t
    Phase;                  /**< used for DHT update: Phase starts from 1 .. RR */
} t_cpmDesc;

/**
 * \internal
 * \brief Internal only descriptor defs - these need to match as closely as possible to PRU defs
 * \details We now have different descriptors for CPM and PPM \n
 * Bit exact so direct copy possible - exactly 16 bytes \n
 */
typedef struct  ppmDesc         /* PPM*/
{
    uint16_t
    FrameReference;         /**< Lower 16 bit of byte address for the PPM buffer. It includes offset for current triple buffer. */
    uint16_t
    FrameLength;            /**< 11 bits of frame length, set by host only.  Frame length has total number of bytes without 4 bytes of CRC which are generated by hardware. Bit 12 to 15 must be 0. Used by PRU to fill in cycle counter and status towards the end of frame. */
    uint32_t
    FrameSendOffset;        /**< Send in RED period: FSO as provided by engineering and limited to 22 bits, i.e. 4.1ms. Set by host only. Higher bits must be set to 0. FSO =0 indicates green phase packet.*/
    uint16_t
    FrameId;                /**< FrameID as provided by engineering. Not used for PPM. */
    uint8_t   FrameFlags1;              /**< TODO */
    uint8_t FrameIndex;                 /**< Index is already considered in FrameReference. Used by host which first reads BufferLock register to get the ppm# and then looks up descriptor to read FrameIndex currently used */
    uint16_t
    RR;                     /**< Reduction Ratio for red period is in (RR-1) where RR is 1,2,4,8,16.
Reduction Ration for green period is (RR-1) where RR is 1,2 ,4 …512.  */
    uint16_t
    Phase;                  /**< used for sendlist generation.  Phase starts from 1 .. RR
 */
} t_ppmDesc;

/**
@}
*/
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
#ifdef MRP_SUPPORT
/** \addtogroup PN_MRP
@{ */
/* @pre MRP_SUPPORT defined*/
/**
* \internal
* \brief iterates over all CPM packets and check if learned already
* The MRP state can be updated accordingly
* learning needs to be done in CPM buffer complete ISR
*
* \param pnHandle Profinet Handle
* \return 1 on all CPM learned
* \return 0 if anyone CPM missing still
*/
uint32_t PN_allCpmKnown(PN_Handle pnHandle);
/**
* \internal
* \brief reset all CPM learned port states
* called in case of FDB flush / MRP support
*
* \param pnHandle Profinet Handle
*/
void PN_resetCpmPorts(PN_Handle pnHandle);

/**
* \internal
* \brief used to update port number in packet struct once CPM received
*
* Updates CPM port number of new received CPM
* Updates PPM in same group with same port
* Updates PPM descriptor list and toggles...
*
* \param pID pointer to packet object
*/
void PN_setCpmPort(PN_Handle pnHandle, t_rtcPacket *pID);
/**
@}
*/
#endif /*MRP_SUPPORT*/

/** \addtogroup PN_CPM_PPM_MANAGEMENT
 @{ */
/**
 * \internal
 * \brief Request to switch active and shadow list. Requests a toggle of active and shadow lists,
 *     it should be answered by PRU via status IRQ
 * \param[in] pruicssHwAttrs PRUICSS HW attrs
 * \param[in] ppmMrpPortShift It indicates if ppm toggle list function is called when ppm descriptor is shifted from one port to another
 *                            Valid values are 0 and 1. 0 is used when this API is called when a PPM descriptor is inserted or deleted
 * \retval 0 on Success
 * \retval <0 if Failure
 *
 */
int32_t PN_togglePpmList(PN_Handle pnHandle, t_rtcPacket *pktID,
                         uint8_t ppmMrpPortShift);


/**
 * \internal
 * \brief build new PPM descriptor from packet object and write to descriptor list \ref t_descList ->  pDescs
 *  only writes to shadow list
 *
 * \param pnHandle Profinet Handle
 * \param[in] pPkt PPM data to be stored in the PPM List
 * \param[in] pos  Position to insert in the list
 * \retval 0 on Success
 * \retval <0 if Failure
 *
 */
int32_t PN_writePpmDesc(PN_Handle pnHandle, t_rtcPacket *pPkt, uint8_t pos);

/**
 * \internal
 * \brief build new CPM descriptor from packet object and write to descriptor list \ref t_descList -> pDescs
 *        only writes to shadow list
 * \param pnHandle Profinet Handle
 * \param[in] pPkt CPM data to be stored in the PPM List
 * \param[in] pos  Position to insert in the list
 * \retval 0 on Success
 * \retval <0 if Failure
 */
int32_t PN_writeCpmDesc(PN_Handle pnHandle, t_rtcPacket *pPkt, uint8_t pos);

/**
 * \internal
 * \brief Read a PPM descriptor from the descriptor list
 *
 * \param pnHandle Profinet Handle
 * \param[out] pDesc    Returns the descriptor at position pos
 * \param[in] pos   Position of the descriptor to be read
 * \param[in] act   Read from \ref ACTIVE_LIST or \ref SHADOW_LIST
 * \retval 0 on Success
 * \retval <0 if Failure
 */
int32_t PN_readPpmDesc(PN_Handle pnHandle, t_ppmDesc *pDesc, uint8_t pos,
                       uint8_t act);

/**
 * \internal
 * \brief Read a CPM descriptor from the descriptor list
 * \param[out] pDesc    Returns the descriptor at position pos
 * \param[in] pos   Position of the descriptor to be read
 * \retval 0 on Success
 * \retval <0 if Failure
 */
int32_t PN_readCpmDesc(PN_Handle pnHandle, t_cpmDesc *pDesc, uint8_t pos);

/**
 * \internal
 * \brief   Configure the index PPM list (shadow only!)
 * \param pnHandle Profinet handle
 * \param index Information of differnet RTC Indexes
 * \retval 0 on Success
 * \retval <0 if Failure
 */
int32_t PN_setIndexInt(PN_Handle pnHandle, t_listIndex *index);

/**
 * \internal
 * \brief Retrieve current CPM Frame index
 *
 * \param pnHandle Profinet Handle
 * \param pos Index in descriptor list
 * \retval Frame index on success
 * \retval <0 if Failure
 */
int8_t PN_getLastCpmBuffIndex(PN_Handle pnHandle, uint8_t pos);

/**
 * \internal
 * \brief Retrieve active PPM list index configuration
 *
 * \param[out] index Indexe data return pointer
 * \retval 0 on Success
 * \retval <0 if Failure
 */
int32_t PN_getIndexInt(t_listIndex *index);

/**
 * \internal
 * \brief Remove all entries from a shadow list
 *
 * \param[out] pList  List to be cleared
 * \retval 0 on Success
 * \retval <0 if Failure
 */
int32_t PN_emptyList(t_descList *pList);


/**
 * \internal
 *  \brief Sorts the list by phase and port
 * \details Go through packet array and generate new shadow PPM list from scratch which is sorted by phase and port.Also computes list index array\n
 * Result of this API - \n
 * \image html desc_phases.png
 * \image latex  desc_phases.png
 * \param[in] pnHandle Profinet Handle
 * \param pPkts PPM/CPM Packet list saved in the Profinet Configuration \ref t_cfgPN
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_writeSortedList(PN_Handle pnHandle, t_rtcPacket *pPkts);

/**
 * \internal
 * \brief  Checks if the DHT event coccured and if yes then retrieves the packet which initiated it
 * \details Helper function for DHT status event passed from PRU \n
 * Multiple DHT events may be activated any time
 *
 * \param[in] pnHandle Profinet Handle
 * \param[out] pktID pointer to t_rtcPacket object (event initiator)
 *  \ref RTC_NOTIFY_DHT_EXPIRE \n
 *
 * \retval 0 when the DHT event has not occured
 * \retval 1 when the DHT event has occured
 * \retval <0 if failure
 */
int8_t PN_getDhtStatusEvent(PN_Handle pnHandle, t_rtcPacket **pktID);


/**
 * \internal
 * \brief  Retrieve the packet which initiated an event and the event type
 * \details Helper function for DHT and other status events passed from PRU \n
 * Multiple events may be activated any time
 *
 * \param[in] pruicssHwAttrs PRUICSS HW attrs
 * \param[out] pktID pointer to t_rtcPacket object (event initiator)
 *  \ref RTC_NOTIFY_PPM_LIST_CHANGE \n
 *
 * \retval 0 on success
 * \retval <0 if failure
 */
int8_t PN_getListToggleStatusEvent(PN_Handle pnHandle, t_rtcPacket **pktID);


/**
 * \internal
 * \brief Gets the shadow index
 *
 * \param[in] pruicssHwAttrs PRUICSS HW attrs
 * \param lType Direction - valid values for 'lType' - PPM only!
 *
 * \retval Current shadow index according to PRU data. -1 on error
 *
 */
int8_t PN_getShadowIndex(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t lType);

/**
 * \internal
 * \brief  Return PM status information as set by PRU
 * \param[in] pruicssHwAttrs PRUICSS HW attrs
 * \param dir       Direction
 *                      Valid values for 'dir' - \ref CPM or \ref PPM
 * \param numPm number of PM to check
 *                      Valid range 0-7
 *
 * \retval PM status
 * \n \ref RTC_PPM_OK
 * \n \ref RTC_PPM_ERROR
 * \n \ref RTC_CPM_RUN
 * \n \ref RTC_CPM_FAILURE
 *
 */
int32_t PN_getPmStatus(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       uint8_t dir, uint8_t numPm);


/**
 * \internal
 * \brief  Defines AR Groups of PPM. extended with shadow registers. we only write to shadow
 * \param[in] pruicssHwAttrs PRUICSS HW attrs
 * \param ARgroup       AR group number
 *                          Valid Range 1-8
 * \param PpmNum        PPM number
 *                          Valid Range 0-7
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_setPPMARlink(PRUICSS_HwAttrs const *pruicssHwAttrs,
                        uint8_t ARgroup, uint8_t PpmNum);

/**
 * \internal
 * \brief  Defines AR Groups of CPM
 * \param[in] pruicssHwAttrs PRUICSS HW attrs
 * \param ARgroup       AR group number
 *                          Valid Range 1-8
 * \param CpmNum        CPM number
 *                          Valid Range 0-7
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_setCPMARlink(PRUICSS_HwAttrs const *pruicssHwAttrs,
                        uint8_t ARgroup, uint8_t CpmNum);

/**
 * \brief  Sets the Data hold timer timeout value for a given descriptor
 *
 * \param[in] pruicssHwAttrs PRUICSS HW attrs
 * \param dht           Data hold timeout value (lost count * reduction ration)*
 * \param pos           position of CPM descriptor
 *                          Valid Range 0-7
 *
 * \retval 0 on success
 * \retval <0 if failure
 *
 */
int32_t PN_setCpmDHT(PRUICSS_HwAttrs const *pruicssHwAttrs,
                     uint16_t dht, uint8_t pos);

/**
@}
*/
/**
 * \brief  Clears the mrpFlag in ppmList data structure
 *
 * \param[in] pnHandle Profinet Handle
 *
 */
void PN_clearMrpFlag(PN_Handle pnHandle);
/**
 * \brief  Sets or clears the listToggleReq flag in ppmList data structure
 *
 * \param[in] pnHandle Profinet Handle
 * \param[in] enable It clears or sets the listToggleReq flag
 *                   valid values are 0 and 1
 */
void PN_setListToggleReq(PN_Handle pnHandle, uint8_t enable);



#ifdef __cplusplus
}
#endif

#endif /* IRTCDRV_H_ */
