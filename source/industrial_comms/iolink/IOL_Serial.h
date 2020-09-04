/*!
 *  \example IOL_Serial.h
 *
 *  \brief
 *  Module for communication via a serial interface
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2020-07-15
 *
 *  \copyright
 *  Copyright (c) 2020, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __IOL_SERIAL_H__
#define __IOL_SERIAL_H__


#include <IOLM_SMI.h>

/** \brief Defines the receive default length */
#define IOL_SERIAL_RX_DEFAULT_LENGTH (2048+100)

/** \brief Defines the receive timeout in ms */
#define IOL_SERIAL_TIMEOUT_MS 200


/**
\brief This structure defines the handle for a serial connection
*/
typedef struct IOL_Serial_SHandle
{
    INT32U u32BufferLen;                /**< Length of single Tx buffer */
    INT32U u32BufferCount;              /**< \Number of Tx buffers */
    INT8U* pu8Data;                     /**< Array with u32BufferLen * u32BufferCount Bytes */
    INT32U* pu32Length;                 /**< Array of u32BufferCount with actual length */

    INT32U u32RxBufferLen;              /**< Length of Rx buffer */
    INT8U* pu8RxBuffer;                 /**< Pointer to Rx buffer */
    IOLM_SMI_CBGenericCnf cbRxCallback; /**< Pointer to receive callback function */
    INT8U u8ForceClientId;              /**< Actual ID or 0 if not used */

    volatile TBOOL boWritePending;      /**< Flag for write control (TRUE if write operation is pending) */
    INT32U u32ReadPos;                  /**< Current read position in Tx buffer */
    INT32U u32WritePos;                 /**< Current write position in Tx buffer */
    INT32U u32RxPos;                    /**< Current position in Rx buffer */
    INT32S s32LastRxTime;               /**< Timestamp of last received packet */

}IOL_Serial_SHandle;


INT8U* IOL_Serial_pu8GetTxBuffer(IOL_Serial_SHandle* psuHandle_p, INT32U* pu32Length_p);
void IOL_Serial_vReleaseTxBuffer(IOL_Serial_SHandle* psuHandle_p);
INT32U IOL_Serial_vGetLenRemain(IOL_Serial_SHandle* psuHandle_p);
void IOL_Serial_vSend(IOL_Serial_SHandle* psuHandle_p, INT8U* pu8Data_p, INT32U u32Length_p);
void IOL_Serial_vSendSmi(IOL_Serial_SHandle* psuHandle_p, IOLM_SMI_SHeader* psuHeader_p, INT8U* pu8ArgBlock_p);
void IOL_Serial_vInit(IOL_Serial_SHandle* psuHandle_p);

void IOL_Serial_vReceive(IOL_Serial_SHandle* psuHandle_p, INT8U* pu8Data_p, INT32U u32Length_p);
void IOL_Serial_vRxProcessing(IOL_Serial_SHandle* psuHandle_p);

#endif
