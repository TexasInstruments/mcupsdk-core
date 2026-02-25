/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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


/**
 * \defgroup DAP_LINK DAP Link Layer
 * \ingroup DAP_AM263X
 *
 * DAP Link layer abstracts UART communication using MCU+SDK UART driver.
 * Provides interrupt-based transmit and receive operations for DAP protocol.
 *
 * @{
 */

#ifndef DAP_LINK_H
#define DAP_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Dap_Types.h"

/* ========================================================================== */
/*                 Public Functions - Common Link APIs                        */
/* ========================================================================== */

/**
 * \brief Set default link initialization parameters
 *
 * \param[out] paramsPtr Pointer to initialization parameters structure
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_InitParamsSetDefault(Dap_Link_InitParamsType *paramsPtr);

/**
 * \brief Initialize link layer and UART hardware
 *
 * This function initializes the UART hardware internally, so the application
 * does not need to call UART_open() separately. The UART configuration
 * is provided via the UartConfig field in the initialization parameters.
 *
 * \param[out] instancePtr Pointer to link layer instance structure
 * \param[in]  paramsPtr   Pointer to initialization parameters
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_Init(Dap_Link_InstanceType *instancePtr,
                     const Dap_Link_InitParamsType *paramsPtr);

/**
 * \brief De-initialize link layer
 *
 * \param[in] instancePtr Pointer to link layer instance
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_DeInit(Dap_Link_InstanceType *instancePtr);

/* ========================================================================== */
/*                 Public Functions - Convenience Wrappers                    */
/* ========================================================================== */

/**
 * \brief Send data frame (convenience wrapper)
 *
 * Initiates transmission using the data already in the TxInstance Frame buffer.
 * The TxInstance.Frame.Buffer and TxInstance.Frame.Len must be populated before
 * calling this function.
 *
 * \param[in] instancePtr Pointer to link layer instance (TxInstance.Frame must be set)
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_Send(Dap_Link_InstanceType *instancePtr);

/**
 * \brief Send raw bytes from provided buffer
 *
 * Sends raw data directly via UART without using the TX frame buffer.
 * Used for streaming mode where data is sent without per-packet framing.
 * Waits for transmission to complete before returning.
 *
 * \param[in] instancePtr  Pointer to link layer instance
 * \param[in] dataPtr      Pointer to data to send
 * \param[in] sizeInBytes  Number of bytes to send
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_SendRaw(Dap_Link_InstanceType *instancePtr,
                        uint8 *dataPtr,
                        uint32 sizeInBytes);

/**
 * \brief Start receiving data frame (convenience wrapper)
 *
 * Initiates reception into the RxInstance Frame buffer.
 * The RxInstance.Frame.Buffer must be allocated before calling this function.
 *
 * \param[in] instancePtr Pointer to link layer instance
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_StartReceive(Dap_Link_InstanceType *instancePtr);


/**
 * \brief Check if transmit is complete (convenience wrapper)
 *
 * \param[in]  instancePtr   Pointer to link layer instance
 * \param[out] isCompletePtr Pointer to store completion status
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_IsTxComplete(Dap_Link_InstanceType *instancePtr,
                             boolean *isCompletePtr);

/**
 * \brief Check if receive is idle
 *
 * \param[in]  instancePtr Pointer to link layer instance
 * \param[out] isIdlePtr   Pointer to store idle status
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_IsRxIdle(Dap_Link_InstanceType *instancePtr,
                         boolean *isIdlePtr);

/**
 * \brief Check if a complete frame is ready for processing
 *
 * \param[in]  instancePtr     Pointer to link layer instance
 * \param[out] isFrameReadyPtr Pointer to store frame ready status
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_IsFrameReady(const Dap_Link_InstanceType *instancePtr,
                             boolean *isFrameReadyPtr);

/**
 * \brief Clear receive buffer (convenience wrapper)
 *
 * \param[in] instancePtr Pointer to link layer instance
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_ClearRxBuffer(Dap_Link_InstanceType *instancePtr);

/**
 * \brief Clear transmit buffer
 *
 * \param[in] instancePtr Pointer to link layer instance
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_ClearTxBuffer(Dap_Link_InstanceType *instancePtr);

/**
 * \brief Wait for transmit to be ready
 *
 * Waits for any pending transmission to complete before returning.
 * Uses configurable timeout via DAP_CFG_TX_READY_TIMEOUT.
 *
 * \param[in] instancePtr Pointer to link layer instance
 *
 * \return DAP_LINK_ERROR_NONE on success, DAP_ERROR_LINK_TIMEOUT on timeout,
 *         DAP_ERROR_HW_BUSY if TX busy and wait disabled
 */
sint32 Dap_Link_WaitForTxReady(Dap_Link_InstanceType *instancePtr);

/**
 * \brief Reset receive operation and clear RX state
 *
 * Aborts any pending receive operation and clears the RX buffer.
 * Resets the RX state machine to idle.
 *
 * \param[in] instancePtr Pointer to link layer instance
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_ResetRx(Dap_Link_InstanceType *instancePtr);

/**
 * \brief Reset transmit operation and clear TX state
 *
 * Aborts any pending transmit operation and clears the TX buffer.
 * Resets the TX state machine to idle.
 *
 * \param[in] instancePtr Pointer to link layer instance
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_ResetTx(Dap_Link_InstanceType *instancePtr);

/* ========================================================================== */
/*                 Public Functions - Frame Access APIs                       */
/* ========================================================================== */

/**
 * \brief Get RX frame data (read-only access)
 *
 * Provides const pointer to received frame data for Core layer to process.
 *
 * \param[in]  instancePtr   Pointer to link layer instance
 * \param[out] bufferPtrPtr  Receives const pointer to RX buffer
 * \param[out] lengthPtr     Receives frame length in bytes
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_GetRxFrameData(const Dap_Link_InstanceType *instancePtr,
                               const uint8 **bufferPtrPtr,
                               uint32 *lengthPtr);

/**
 * \brief Get TX frame buffer for writing
 *
 * Provides writable pointer to TX buffer for Core layer to build response.
 * After populating buffer, Core must call Dap_Link_SetTxFrameLength().
 *
 * \param[in]  instancePtr   Pointer to link layer instance
 * \param[out] bufferPtrPtr  Receives pointer to TX buffer (writable)
 * \param[out] maxLengthPtr  Receives maximum buffer size in bytes
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_GetTxFrameBuffer(Dap_Link_InstanceType *instancePtr,
                                 uint8 **bufferPtrPtr,
                                 uint32 *maxLengthPtr);

/**
 * \brief Set TX frame length after population
 *
 * Core layer calls this after writing data to TX buffer obtained from
 * Dap_Link_GetTxFrameBuffer() to set the actual frame length.
 *
 * \param[in] instancePtr Pointer to link layer instance
 * \param[in] length      Frame length in bytes
 *
 * \return DAP_LINK_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Link_SetTxFrameLength(Dap_Link_InstanceType *instancePtr,
                                 uint32 length);

/**
 * \brief Continue receiving a DAP frame (callback context)
 *
 * This function processes UART RX completion events and manages the
 * frame reception state machine. It is designed to be called from
 * the UART read callback.
 *
 * \param[in] instancePtr     Pointer to DAP link instance
 * \param[in] bytesReceived   Number of bytes received in this callback
 *
 * \note This function is called from interrupt context.
 *
 * \see Dap_Link_StartReceive
 */
void Dap_Link_ContinueReceive(Dap_Link_InstanceType *instancePtr,
                              uint32 bytesReceived);

/**
 * \brief Update transmit state after completion (callback context)
 *
 * This function updates the DAP link transmit state machine when a
 * UART transmission completes. It is designed to be called from
 * the UART write callback.
 *
 * \param[in] instancePtr     Pointer to DAP link instance
 *
 * \note This function is called from interrupt context.
 */
void Dap_Link_ContinueTransmit(Dap_Link_InstanceType *instancePtr);

#ifdef __cplusplus
}
#endif

#endif /* DAP_LINK_H */

/** @} */
