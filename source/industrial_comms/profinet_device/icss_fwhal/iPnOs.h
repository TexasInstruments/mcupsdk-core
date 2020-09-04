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

#ifndef IPNOS_H_
#define IPNOS_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                          Doxygen                                           */
/* ========================================================================== */

/**
 * \defgroup PN_IRT_TRIPLE_BUFFER Triple buffer management APIs
 * \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
 */

/**
* \defgroup PN_MRP MRP APIs
* \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
*/

/**
* \defgroup PN_IRT_LEGACY Legacy Startup APIs
* \ingroup INDUSTRIAL_COMMS_PROFINET_DEVICE_FWHAL_MODULE
*/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "PN_Handle.h"
#include "iRtcDrv2.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/** \addtogroup PN_IRT_TRIPLE_BUFFER
 @{ */

/**
 * \brief Return a pointer to buffer indicated by NEXT \n
 * This will be the next available free buffer(in the triple buffer scheme).
 * Stack can now fill this buffer and once done call \ref PN_relPpmBuff to
 * indicate the PRU on new data.
 *
 * \param pkt pointer to packet object
 * \return pointer to NEXT packet buffer address
 */
uint8_t *PN_getPpmBuff(t_rtcPacket *pkt);

/**
 * \brief Swap NEXT with LAST and return new NEXT address \n
 * \details This API is called when stack has completed updating data to the buffer
 * received from \ref PN_getPpmBuff. This indicates the PRU about data update and updates
 * the \ref ACTIVE_LIST directly. Stack can now start filling data in the NEXT buffer returned
 *
 * \param pnHandle Profinet Handle
 * \param pkt pointer to packet object
 * \return pointer to NEXT packet buffer address
 */
uint8_t *PN_relPpmBuff(PN_Handle pnHandle, t_rtcPacket *pkt);

/**
 * \brief Update proc pointer in paket object and udpate descriptor if there is a new last buffer
 * \details Called from the \ref PN_ppmIsrHandler. The PRU firmware checks if there is a new
 * buffer available to send by checking the validLast flag in \ref t_rtcPacket . If available
 * swaps the proc buffer(which is currently being send by the PRU), with the last buffer(the
 * buffer which holds the latest data produced by the stack). This ensures that the PRU sends
 * out latest available valid data at any point of time.
 *
 * \param pnHandle Profinet Handle
 * \param pkt pointer to packet object
 * \return 1: new buffer activated \n
 *         0: no buffer update
 *         <0: Not a PPM packet
 */
int32_t PN_checkLastPPM(PN_Handle pnHandle, t_rtcPacket *pkt);

/**
 * \brief Swap CPM NEXT with LAST and set validLast flag
 * \details Called from \ref PN_cpmIsrHandler , when a new CPM frame has arrived
 * and sets the validLast flag in \ref t_rtcPacket . When the stack is ready to
 * receive an updated CPM packet, it calls \ref PN_getLastCpm and this LAST packet
 * is consumed by the stack
 *
 * \param pnHandle Profinet Handle
 * \param pkt pointer to packet object
 * \retval 0 buffers swapped \n
 * \retval 1 buffers swapped and overrun detected\n
 * This indicates the validLast flag was already set, and the stack has missed to
 * consume a CPM packet
 * \retval -1 error - wrong packet type\n
 */
int32_t PN_nextCpmRdy(PN_Handle pnHandle,
                      t_rtcPacket *pkt);


/**
 * \brief Swap CPM LAST with PROC if validLast is true, clear validLast flag
 * \details When the stack is ready to consume a new CPM packet, this API is
 * called. This swaps the LAST buffer(which has the latest updated/consumed data
 * by the PRU firmware) and the PROC buffer(which is being currently consumed by the
 * stack). This also locks the new PROC buffer indicating that the Stack is consuming
 * this buffer(PRU won't write to this buffer)
 *
 * \param[in] pnHandle Profinet Handle
 * \param pkt pointer to packet object
 * \return buffer PROC buffer address
 */
uint8_t *PN_getLastCpm(PN_Handle pnHandle, t_rtcPacket *pkt);

/**
@}
*/
/** \addtogroup PN_MRP
 @{ */
/**
 * \brief Used to signal topology change to MRP state machine
 *
 * \param pnHandle Profinet Handle
 * \return 0 if we went to MRPENTER state
 * \return 1 if already in flush operation
 *
 * \pre MRP_SUPPORT defined
 */
uint32_t PN_enterFlushMode(PN_Handle pnHandle);
/**
 * \internal
 * \def state machine states for MRP task
 */
typedef enum
{
    MRPREADY,       /**< MRP module initiliazed. Idle state, do nothing */
    MRPENTER,       /**< MRP flush mode entered. Break in network detected, flush FDB */
    MRPWAIT,        /**< Waiting for CPM received. Sleep and goto \ref MRPCHECK */
    MRPCHECK,       /**< Check if all CPM received. Check by \ref PN_allCpmKnown*/
    MRPEXIT         /**< Exit MRP flush mode, back to normal op i.e., to \ref MRPREADY */
} tMrpStates;
/**
@}
*/

/** \addtogroup PN_IRT_LEGACY
 @{ */

/**
 * \brief Returns the IRT Legacy mode state
 *
 *
 * \return Current state of the Legacy mode state machine
 */
tLegStates PN_getLegState(void);

/**
 * \brief Sets the state of legacy state machine
 * This API is registered as a callback in \ref PN_registerSetState
 *
 * \param arg   new state \ref tLegStates
 * \param arg2  not used
 */
void PN_setLegState(void *arg, void *arg2);

/**
 * \brief Sets the Ethernet packet to output during legacy startup
 * Usually a copy of RTC3 PPM. This API is registered as a callback in \ref PN_registerSetPkt
 *
 * \param arg   Pointer to valid packet buffer, no checks. \ref t_rtcPacket
 * \param arg2  not used
 */
void PN_setLegPkt(void *arg, void *arg2);
/**
@}
*/

/**
 * \brief Initialization function for all PN driver tasks, mutexes
 *
 * \param pnHandle Profinet Handle
 * \return None
 */
int32_t PN_initOs(PN_Handle pnHandle);

/**
 * \brief Interrupt management function
 *        Initializes PPM/CPM/DHT interrupts. (OS dependent!)
 *
 * \retval 0 on success
 */
int32_t PN_RTC_setupIsr(PN_Handle pnHandle);

/**
 * \brief Enables the PN interrupts
 * \param[in] pnHandle Profinet HAndle
 * \retval 0 on Success
 *
 */
int32_t PN_RTC_enableISR(PN_Handle pnHandle);

/**
 * \brief Disables the PN interrupts
 * \param pnHandle Profinet Handle
 * \retval 0 on Success
 *
 */
int32_t PN_RTC_disableISR(PN_Handle pnHandle);

/**
 * \brief Switch driver extension of TxPacket
 *
 * \details This is using a critical section to protect re-entry of TX function.
 * The protection scheme is borrowed from NDK and we use their code too.
 * This requires to adhere to NDK priority scheme
 *
 * \param[in] icssEmacHandle ICSS Emac LLD handle
 * \param srcAddress        pointer to TX packet
 * \param portNumber        output port number
 * \param queuePriority     output queue priority
 * \param lengthOfPacket    TX packet length (without CRC)
 * \callgraph
 */
int32_t PN_OS_txPacket(ICSS_EMAC_Handle icssEmacHandle,
                       const uint8_t *srcAddress, int32_t portNumber, int32_t queuePriority,
                       int32_t lengthOfPacket);


#ifdef __cplusplus
}
#endif

#endif /* IPNOS_H_ */
