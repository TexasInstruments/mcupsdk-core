/**
 * \file hsrPrp_red_tx.h
 * \brief Include file for hsrPrp_red_tx.c
 *
 * \par
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
 * \par
 */

#ifndef RED_TX_H_
#define RED_TX_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "hsrPrp_red_common.h"
#include "hsrPrp_handle.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \internal
 *  \brief Enqueues redundancy frames to transmit in HSR Mode T
 *  \param icssEmacHandle Pointer to ICSS EMAC Handle, parent structure containing all switch information
 *  \param packetData memory to copy packet data from
 *  \param packetLength length of the packet
 *  \param queuePriority which queue to write data to
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS RedTxPacketHsrModeT(ICSS_EMAC_Handle icssEmacHandle,
                               const uint8_t *packetData, int32_t packetLength, int32_t queuePriority);
/**
 *  \internal
 *  \brief Enqueues redundancy frames to transmit in HSR Mode M
 *         when local criteria met
 *  \param icssEmacHandle Pointer to ICSS EMAC Handle, parent structure containing all switch information
 *  \param packetData memory to copy packet data from
 *  \param packetLength length of the packet
 *  \param queuePriority which queue to write data to
 *
 *  \return RED_OK on success, RED_ERR otherwise
 *
 */
RED_STATUS RedTxPacketHsrModeM(ICSS_EMAC_Handle icssEmacHandle,
                               const uint8_t *packetData, int32_t packetLength, int32_t queuePriority, int32_t port);
/**
 *  \brief Function to enqueue packet for transmission
 *  \param icssEmacHandle Pointer to ICSS EMAC Handle, parent structure containing all switch information
 *  \param pRedFrame pointer to a RED_FRAME
 *  \param portNumber port from which data will be transmitted
 *  \param queuePriority which queue to write data to
 *
 *  \return RED_OK if success, RED_ERR otherwise
 */
RED_STATUS RedTxPacketEnqueue(hsrPrpHandle *, ICSS_EMAC_Handle icssEmacHandle,
                              RED_FRAME *pRedFrame, int32_t portNumber, int32_t queuePriority);


#ifdef __cplusplus
}
#endif

#endif /* RED_TX_H_ */
