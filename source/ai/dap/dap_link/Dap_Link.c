/*
 *  Copyright (C) 2025-2026 Texas Instruments Incorporated
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

#include "Dap_Link.h"
#include "dap_core/Dap_Core.h"
#include <drivers/uart.h>
#include <drivers/uart/v0/lld/uart_lld.h>
#include <kernel/dpl/SystemP.h>

/* ========================================================================== */
/*                 Internal Definitions                                       */
/* ========================================================================== */

/** Default UART instance index */
#define DAP_LINK_DEFAULT_UART_INSTANCE      (0U)

/** Smallest possible frame length */
#define DAP_LINK_RX_MIN_FRAME_LENGTH        (4U)

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static int32_t Dap_Link_Uart_Write(Dap_Link_InstanceType *instancePtr,
                                   uint8 *bufferPtr,
                                   uint32 count);

static int32_t Dap_Link_Uart_Read(Dap_Link_InstanceType *instancePtr,
                                  uint8 *bufferPtr,
                                  uint32 count);

static void Dap_Link_ResetRxState(Dap_Link_InstanceType *instancePtr);

static void Dap_Link_ResetTxState(Dap_Link_InstanceType *instancePtr);

static sint32 Dap_Link_DecodePayloadLength(const uint8 *bufferPtr,
                                           uint32 bufferLen,
                                           uint32 *payloadLenPtr,
                                           uint32 *headerLenPtr);

/* ========================================================================== */
/*                 Public Functions - Common Link APIs                        */
/* ========================================================================== */

sint32 Dap_Link_InitParamsSetDefault(Dap_Link_InitParamsType *paramsPtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (paramsPtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        paramsPtr->UartInstanceIndex = DAP_LINK_DEFAULT_UART_INSTANCE;
        paramsPtr->LinkTxMode = DAP_LINK_MODE_INTERRUPT;
        paramsPtr->LinkRxMode = DAP_LINK_MODE_INTERRUPT;
    }

    return retVal;
}

sint32 Dap_Link_Init(Dap_Link_InstanceType *instancePtr,
                     const Dap_Link_InitParamsType *paramsPtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    /* Parameter validation */
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (paramsPtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (paramsPtr->LinkRxMode >= DAP_LINK_MODE_MAX ||
             paramsPtr->LinkTxMode >= DAP_LINK_MODE_MAX)
    {
        retVal = DAP_LINK_ERROR_UNSUPPORTED_MODE;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        instancePtr->IsInitialized = FALSE;

        /* Get UART handle from MCU+SDK driver (opened via SysConfig) */
        instancePtr->UartHandle = UART_getHandle(paramsPtr->UartInstanceIndex);

        if (instancePtr->UartHandle == NULL_PTR)
        {
            retVal = DAP_LINK_ERROR_HW_INIT;
        }
    }

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        /* Initialize Tx side */
        instancePtr->TxInstance.ParentInstancePtr = instancePtr;
        instancePtr->TxInstance.Mode = paramsPtr->LinkTxMode;
        instancePtr->TxInstance.Frame.Ptr = 0U;
        instancePtr->TxInstance.Frame.Len = 0U;
        instancePtr->TxInstance.State = DAP_LINK_TX_STATE_IDLE;

        /* Initialize Rx side */
        instancePtr->RxInstance.ParentInstancePtr = instancePtr;
        instancePtr->RxInstance.Mode = paramsPtr->LinkRxMode;
        instancePtr->RxInstance.Frame.Ptr = 0U;
        instancePtr->RxInstance.Frame.Len = 0U;
        instancePtr->RxInstance.FrameLength = 0U;
        instancePtr->RxInstance.State = DAP_LINK_RX_STATE_IDLE;
        instancePtr->RxInstance.ExpectedFrameLen = 0U;
        instancePtr->RxInstance.HeaderDecoded = FALSE;

        /* Initialize other fields */
        instancePtr->LastFrameError = DAP_LINK_ERROR_NONE;
        instancePtr->LastProtocolError = 0U;
        instancePtr->IsCrcEnabled = FALSE;

        instancePtr->IsInitialized = TRUE;
    }

    return retVal;
}

sint32 Dap_Link_DeInit(Dap_Link_InstanceType *instancePtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    /* Parameter validation */
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        /* UART close is handled by application via Drivers_close() */
        instancePtr->UartHandle = (UART_Handle)NULL_PTR;
        instancePtr->IsInitialized = FALSE;
    }

    return retVal;
}

/* ========================================================================== */
/*                 Public Functions - Tx & Rx Operations                      */
/* ========================================================================== */

sint32 Dap_Link_Send(Dap_Link_InstanceType *instancePtr)
{
    sint32                   retVal = DAP_LINK_ERROR_NONE;
    Dap_Link_TxInstanceType *txInstancePtr;

    txInstancePtr = &instancePtr->TxInstance;

    if (txInstancePtr->Frame.Len == 0U)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (txInstancePtr->Frame.Len > DAP_LINK_BUFFER_SIZE_BYTES)
    {
        retVal = DAP_LINK_ERROR_BUFFER_OVERFLOW;
    }
    else
    {
        /* Use SendRaw to transmit the frame buffer */
        retVal = Dap_Link_SendRaw(instancePtr,
                                  txInstancePtr->Frame.Buffer,
                                  txInstancePtr->Frame.Len);
    }

    return retVal;
}

sint32 Dap_Link_SendRaw(Dap_Link_InstanceType *instancePtr,
                        uint8 *dataPtr,
                        uint32 sizeInBytes)
{
    sint32  retVal = DAP_LINK_ERROR_NONE;
    int32_t uartStatus;
    boolean txComplete = FALSE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (dataPtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (sizeInBytes == 0U)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        /* Wait for any previous transmission to complete */
#if DAP_CFG_TX_WAIT_FOR_COMPLETE == STD_ON
        uint32 txTimeoutCounter = DAP_CFG_TX_READY_TIMEOUT;
        do
        {
            (void)Dap_Link_IsTxComplete(instancePtr, &txComplete);
            txTimeoutCounter--;
        } while ((txComplete == FALSE) && (txTimeoutCounter > 0U));

        if ((txComplete == FALSE) && (txTimeoutCounter == 0U))
        {
            retVal = DAP_LINK_ERROR_TIMEOUT;
        }
#else
        if (instancePtr->TxInstance.State == DAP_LINK_TX_STATE_TRANSMITTING)
        {
            retVal = DAP_LINK_ERROR_HW_BUSY;
        }
#endif /* DAP_CFG_TX_WAIT_FOR_COMPLETE == STD_ON */

        if (retVal == DAP_LINK_ERROR_NONE)
        {
            if (instancePtr->TxInstance.Mode == DAP_LINK_MODE_INTERRUPT)
            {
                /* Reset TX state before starting new transmission */
                Dap_Link_ResetTxState(instancePtr);

                instancePtr->TxInstance.State = DAP_LINK_TX_STATE_TRANSMITTING;

                uartStatus = Dap_Link_Uart_Write(instancePtr, dataPtr, sizeInBytes);

                if (uartStatus != SystemP_SUCCESS)
                {
                    instancePtr->TxInstance.State = DAP_LINK_TX_STATE_IDLE;
                    retVal = DAP_LINK_ERROR_HW_BUSY;
                }
            }
            else
            {
                retVal = DAP_LINK_ERROR_UNSUPPORTED_MODE;
            }
        }
    }

    return retVal;
}

sint32 Dap_Link_StartReceive(Dap_Link_InstanceType *instancePtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    int32_t uartStatus;
    Dap_Link_RxInstanceType *rxInstancePtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        /*
         * Reset driver state to ensure previous transaction is cleared.
         * This is needed because UART_lld_Transaction_deInit may not have
         * fully cleaned up the transaction state.
         */
        Dap_Link_ResetRxState(instancePtr);

        (void)Dap_Link_ClearRxBuffer(instancePtr);

        rxInstancePtr = &instancePtr->RxInstance;

        if (rxInstancePtr->Mode == DAP_LINK_MODE_INTERRUPT)
        {
            rxInstancePtr->FrameLength = 0U;
            rxInstancePtr->Frame.Len = 0U;
            rxInstancePtr->Frame.Ptr = 0U;
            rxInstancePtr->State = DAP_LINK_RX_STATE_BUFFERING;

            /*
             * Start receiving with full buffer size.
             * With UART_READ_RETURN_MODE_PARTIAL, the callback fires on ANY data.
             * The callback calls Dap_Link_ContinueReceive() to continue reading.
             */
            uartStatus = Dap_Link_Uart_Read(instancePtr,
                                            rxInstancePtr->Frame.Buffer,
                                            DAP_LINK_BUFFER_SIZE_BYTES);

            if (uartStatus != SystemP_SUCCESS)
            {
                rxInstancePtr->State = DAP_LINK_RX_STATE_IDLE;
                retVal = DAP_LINK_ERROR_HW_BUSY;
            }
        }
        else
        {
            retVal = DAP_LINK_ERROR_UNSUPPORTED_MODE;
        }
    }

    return retVal;
}

sint32 Dap_Link_IsTxComplete(Dap_Link_InstanceType *instancePtr,
                             boolean *isCompletePtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    Dap_Link_TxInstanceType *txInstancePtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (isCompletePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        txInstancePtr = &instancePtr->TxInstance;

        /* Check state */
        if ((txInstancePtr->State == DAP_LINK_TX_STATE_IDLE) ||
            (txInstancePtr->State == DAP_LINK_TX_STATE_COMPLETE))
        {
            *isCompletePtr = TRUE;
        }
        else
        {
            *isCompletePtr = FALSE;
        }
    }

    return retVal;
}

sint32 Dap_Link_IsRxIdle(Dap_Link_InstanceType *instancePtr,
                         boolean *isIdlePtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    Dap_Link_RxInstanceType *rxInstancePtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (isIdlePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        rxInstancePtr = &instancePtr->RxInstance;

        if (rxInstancePtr->State == DAP_LINK_RX_STATE_IDLE)
        {
            *isIdlePtr = TRUE;
        }
        else
        {
            *isIdlePtr = FALSE;
        }
    }

    return retVal;
}

sint32 Dap_Link_IsFrameReady(const Dap_Link_InstanceType *instancePtr,
                             boolean *isFrameReadyPtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    const Dap_Link_RxInstanceType *rxInstancePtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (isFrameReadyPtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        rxInstancePtr = &instancePtr->RxInstance;

        if (rxInstancePtr->State == DAP_LINK_RX_STATE_FRAME_READY)
        {
            *isFrameReadyPtr = TRUE;
        }
        else
        {
            *isFrameReadyPtr = FALSE;
        }
    }

    return retVal;
}

sint32 Dap_Link_ClearRxBuffer(Dap_Link_InstanceType *instancePtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    Dap_Link_RxInstanceType *rxInstancePtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        rxInstancePtr               = &instancePtr->RxInstance;

        rxInstancePtr->Frame.Ptr      = 0U;
        rxInstancePtr->Frame.Len      = 0U;
        rxInstancePtr->FrameLength    = 0U;
        rxInstancePtr->State          = DAP_LINK_RX_STATE_IDLE;
        rxInstancePtr->ExpectedFrameLen = 0U;
        rxInstancePtr->HeaderDecoded  = FALSE;
    }

    return retVal;
}

sint32 Dap_Link_ClearTxBuffer(Dap_Link_InstanceType *instancePtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    Dap_Link_TxInstanceType *txInstancePtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        txInstancePtr = &instancePtr->TxInstance;

        txInstancePtr->Frame.Ptr    = 0U;
        txInstancePtr->Frame.Len    = 0U;
        txInstancePtr->State        = DAP_LINK_TX_STATE_IDLE;
    }

    return retVal;
}

sint32 Dap_Link_WaitForTxReady(Dap_Link_InstanceType *instancePtr)
{
    sint32 retVal = DAP_ERROR_NONE;

#if DAP_CFG_TX_WAIT_FOR_COMPLETE == STD_ON
    boolean txComplete       = FALSE;
    uint32  txTimeoutCounter = DAP_CFG_TX_READY_TIMEOUT;

    do
    {
        (void)Dap_Link_IsTxComplete(instancePtr, &txComplete);
        txTimeoutCounter--;
    } while ((txComplete == FALSE) && (txTimeoutCounter > 0U));

    if (txComplete == FALSE)
    {
        retVal = DAP_ERROR_LINK_TIMEOUT;
    }
#else
    if (instancePtr->TxInstance.State == DAP_LINK_TX_STATE_TRANSMITTING)
    {
        retVal = DAP_ERROR_HW_BUSY;
    }
#endif /* DAP_CFG_TX_WAIT_FOR_COMPLETE == STD_ON */

    return retVal;
}

sint32 Dap_Link_ResetRx(Dap_Link_InstanceType *instancePtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    Dap_Link_RxInstanceType *rxInstancePtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        rxInstancePtr = &instancePtr->RxInstance;

        rxInstancePtr->State = DAP_LINK_RX_STATE_IDLE;

        /* Reset internal driver state */
        Dap_Link_ResetRxState(instancePtr);

        /*
         * Clear any stale bytes from the RX FIFO.
         * This is critical to prevent frame corruption from residual bytes
         * left over from previous transactions.
         *
         * The MCU+SDK UART driver doesn't have a dedicated API for this,
         * so we read and discard any available bytes directly.
         */
        {
            UART_Config    *uartConfig;
            UART_Object    *uartObj;
            UARTLLD_Handle  lldHandle;
            uint8_t         dummyByte;

            uartConfig = (UART_Config *)instancePtr->UartHandle;
            if ((uartConfig != NULL_PTR) && (uartConfig->object != NULL_PTR))
            {
                uartObj = uartConfig->object;
                lldHandle = uartObj->uartLld_handle;
                if (lldHandle != NULL_PTR)
                {
                    /* Read and discard all bytes in RX FIFO */
                    while (UART_getChar(lldHandle->baseAddr, &dummyByte) != 0U)
                    {
                        /* Discard the byte - just clearing the FIFO */
                    }
                }
            }
        }

        /* Clear frame buffer tracking */
        rxInstancePtr->Frame.Ptr        = 0U;
        rxInstancePtr->Frame.Len        = 0U;
        rxInstancePtr->ExpectedFrameLen = 0U;
        rxInstancePtr->HeaderDecoded    = FALSE;
    }

    return retVal;
}

sint32 Dap_Link_ResetTx(Dap_Link_InstanceType *instancePtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    Dap_Link_TxInstanceType *txInstancePtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        txInstancePtr = &instancePtr->TxInstance;

        /* Reset internal driver state */
        Dap_Link_ResetTxState(instancePtr);

        /* Clear frame buffer tracking */
        txInstancePtr->Frame.Ptr = 0U;
        txInstancePtr->Frame.Len = 0U;
        txInstancePtr->State     = DAP_LINK_TX_STATE_IDLE;
    }

    return retVal;
}

/* ========================================================================== */
/*                 Public Functions - Frame Access APIs                       */
/* ========================================================================== */

sint32 Dap_Link_GetRxFrameData(const Dap_Link_InstanceType *instancePtr,
                               const uint8 **bufferPtrPtr,
                               uint32 *lengthPtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (bufferPtrPtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (lengthPtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        *bufferPtrPtr = instancePtr->RxInstance.Frame.Buffer;
        *lengthPtr    = instancePtr->RxInstance.Frame.Len;
    }

    return retVal;
}

sint32 Dap_Link_GetTxFrameBuffer(Dap_Link_InstanceType *instancePtr,
                                 uint8 **bufferPtrPtr,
                                 uint32 *maxLengthPtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (bufferPtrPtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (maxLengthPtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        *bufferPtrPtr  = instancePtr->TxInstance.Frame.Buffer;
        *maxLengthPtr  = DAP_LINK_BUFFER_SIZE_BYTES;
    }

    return retVal;
}

sint32 Dap_Link_SetTxFrameLength(Dap_Link_InstanceType *instancePtr,
                                 uint32 length)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_LINK_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_LINK_ERROR_NOT_INITIALIZED;
    }
    else if (length > DAP_LINK_BUFFER_SIZE_BYTES)
    {
        retVal = DAP_LINK_ERROR_BUFFER_OVERFLOW;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        instancePtr->TxInstance.Frame.Len = length;
    }

    return retVal;
}

/* ========================================================================== */
/*                 Public Functions - Callback Handler                        */
/* ========================================================================== */

void Dap_Link_ContinueReceive(Dap_Link_InstanceType *instancePtr,
                              uint32 bytesReceived)
{
    Dap_Link_RxInstanceType *rxInstancePtr;
    uint32                  currentFrameLength;
    uint32                  prevPtr;
    uint32                  status;
    int32_t                 uartStatus;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        return;
    }

    if (instancePtr->IsInitialized == FALSE)
    {
        return;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    rxInstancePtr = &instancePtr->RxInstance;
    prevPtr = rxInstancePtr->Frame.Ptr;
    status = DAP_LINK_ERROR_NONE;

    /*
     * With UART_READ_RETURN_MODE_PARTIAL, bytesReceived is the count of bytes
     * received in this callback. The buffer is filled starting from where we
     * requested (Frame.Buffer + Frame.Ptr).
     */
    currentFrameLength = prevPtr + bytesReceived;

    if (rxInstancePtr->State != DAP_LINK_RX_STATE_BUFFERING)
    {
        return;
    }

    /* Update frame tracking */
    rxInstancePtr->Frame.Ptr = currentFrameLength;
    rxInstancePtr->Frame.Len = currentFrameLength;

    /* Check start byte first */
    if ((currentFrameLength >= 1U) &&
        (rxInstancePtr->Frame.Buffer[DAP_FRAME_OFFSET_START] != DAP_FRAME_START_BYTE))
    {
        /* Invalid start byte - clear buffer and restart reception */
        instancePtr->LastFrameError = DAP_CORE_ERROR_INVALID_START_BYTE;
        Dap_Link_ResetRxState(instancePtr);
        (void)Dap_Link_ClearRxBuffer(instancePtr);
        (void)Dap_Link_StartReceive(instancePtr);
        return;
    }

    /* Try to decode payload length if not yet decoded */
    if (rxInstancePtr->HeaderDecoded == FALSE)
    {
        if (currentFrameLength >= DAP_FRAME_MIN_HEADER_DECODE_BYTES)
        {
            uint32 payloadLen;
            uint32 headerLen;

            if (Dap_Link_DecodePayloadLength(
                    &rxInstancePtr->Frame.Buffer[DAP_FRAME_OFFSET_LENGTH],
                    currentFrameLength - DAP_FRAME_OFFSET_LENGTH,
                    &payloadLen,
                    &headerLen) == DAP_LINK_ERROR_NONE)
            {
                /* Successfully decoded - calculate expected total frame length */
                rxInstancePtr->ExpectedFrameLen =
                    DAP_FRAME_HEADER_SIZE_BYTES + headerLen + payloadLen + DAP_FRAME_TRAILER_SIZE_BYTES;

                if (rxInstancePtr->ExpectedFrameLen > DAP_LINK_BUFFER_SIZE_BYTES)
                {
                    status = DAP_LINK_ERROR_BUFFER_OVERFLOW;
                }
                else
                {
                    rxInstancePtr->HeaderDecoded = TRUE;
                }
            }
        }
    }

    /* Check if frame is complete based on expected length */
    if ((status == DAP_LINK_ERROR_NONE) &&
        (rxInstancePtr->HeaderDecoded == TRUE) &&
        (currentFrameLength >= rxInstancePtr->ExpectedFrameLen))
    {
        /* Frame complete - ready for processing */
        rxInstancePtr->State = DAP_LINK_RX_STATE_FRAME_READY;
        return;
    }

    /* Continue receiving if no error and frame not complete yet */
    if ((status == DAP_LINK_ERROR_NONE) &&
        (currentFrameLength < DAP_LINK_BUFFER_SIZE_BYTES))
    {
        /* Reset driver state and continue reading */
        Dap_Link_ResetRxState(instancePtr);

        /* Continue receiving remaining bytes */
        uartStatus = Dap_Link_Uart_Read(instancePtr,
                           &rxInstancePtr->Frame.Buffer[currentFrameLength],
                           DAP_LINK_BUFFER_SIZE_BYTES - currentFrameLength);
        (void)uartStatus;
        return;
    }

    /* Buffer overflow check */
    if (currentFrameLength >= DAP_LINK_BUFFER_SIZE_BYTES)
    {
        status = DAP_LINK_ERROR_BUFFER_OVERFLOW;
    }

    /* Handle errors by clearing and restarting */
    if (status != DAP_LINK_ERROR_NONE)
    {
        instancePtr->LastFrameError = DAP_CORE_ERROR_BUFFER_OVERFLOW;
        Dap_Link_ResetRxState(instancePtr);
        (void)Dap_Link_ClearRxBuffer(instancePtr);
        (void)Dap_Link_StartReceive(instancePtr);
    }
}

void Dap_Link_ContinueTransmit(Dap_Link_InstanceType *instancePtr)
{
#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        return;
    }

    if (instancePtr->IsInitialized == FALSE)
    {
        return;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    /* Reset TX state to idle after transmission completes */
    (void)Dap_Link_ResetTx(instancePtr);
}

/* ========================================================================== */
/*                 Internal Functions                                         */
/* ========================================================================== */

/**
 * \brief Reset UART driver RX state to allow new read
 *
 * This function clears the driver's internal RX transaction state to allow
 * a new UART_read() call. It uses the NoCB (no callback) variant to avoid
 * triggering re-entrant callbacks which would corrupt the frame state.
 *
 * \param[in] instancePtr  Pointer to link instance
 */
static void Dap_Link_ResetRxState(Dap_Link_InstanceType *instancePtr)
{
    UART_Config    *uartConfig;
    UART_Object    *uartObj;
    UARTLLD_Handle  lldHandle;

    uartConfig = (UART_Config *)instancePtr->UartHandle;
    if (uartConfig != NULL_PTR && uartConfig->object != NULL_PTR)
    {
        uartObj = uartConfig->object;
        lldHandle = uartObj->uartLld_handle;
        if (lldHandle != NULL_PTR)
        {
            /*
             * Use UART_readCancelNoCB to cancel without triggering callback.
             * UART_lld_readCancel triggers readCompleteCallbackFxn which causes
             * re-entrant callback issues and corrupts the frame state.
             */
            (void)UART_readCancelNoCB(lldHandle);

            /* Clear HLD object RX state */
            uartObj->readTrans = (UART_Transaction *)NULL_PTR;
            uartObj->readBuf = (uint8_t *)NULL_PTR;
            uartObj->readCount = 0U;
            uartObj->readSizeRemaining = 0U;

            /* Clear LLD RX transaction state using driver API */
            UART_lld_Transaction_deInit(&lldHandle->readTrans);
            lldHandle->readBuf = (uint8_t *)NULL_PTR;
            lldHandle->readSizeRemaining = 0U;
            lldHandle->readCount = 0U;

            /* Set state to READY */
            lldHandle->state = UART_STATE_READY;
        }
    }
}

/**
 * \brief Reset UART driver TX state
 *
 * This function clears the driver's internal TX transaction state to allow
 * a new UART_write() call. It resets all TX-related state in both HLD and
 * LLD layers.
 *
 * \param[in] instancePtr  Pointer to link instance
 */
static void Dap_Link_ResetTxState(Dap_Link_InstanceType *instancePtr)
{
    
    UART_Transaction cancelTrans;

    /* Use proper UART API to cancel pending TX */
    (void)UART_writeCancel(instancePtr->UartHandle, &cancelTrans);

    /* Reset link layer TX state */
    instancePtr->TxInstance.State = DAP_LINK_TX_STATE_IDLE;
}

/**
 * \brief Internal wrapper for MCU+SDK UART write operation
 *
 * \param[in] instancePtr  Pointer to link instance
 * \param[in] bufferPtr    Pointer to data buffer
 * \param[in] count        Number of bytes to transmit
 *
 * \return SystemP_SUCCESS on success, error code on failure
 */
static int32_t Dap_Link_Uart_Write(Dap_Link_InstanceType *instancePtr,
                                   uint8 *bufferPtr,
                                   uint32 count)
{
    int32_t uartStatus;

    /* Initialize UART transaction */
    UART_Transaction_init(&instancePtr->TxTransaction);
    instancePtr->TxTransaction.buf   = bufferPtr;
    instancePtr->TxTransaction.count = count;
    instancePtr->TxTransaction.args  = (uint32_t *)instancePtr;

    /* Start async TX using MCU+SDK UART */
    uartStatus = UART_write(instancePtr->UartHandle, &instancePtr->TxTransaction);

    return uartStatus;
}

/**
 * \brief Internal wrapper for MCU+SDK UART read operation
 *
 * \param[in] instancePtr  Pointer to link instance
 * \param[in] bufferPtr    Pointer to receive buffer
 * \param[in] count        Number of bytes to receive
 *
 * \return SystemP_SUCCESS on success, error code on failure
 */
static int32_t Dap_Link_Uart_Read(Dap_Link_InstanceType *instancePtr,
                                  uint8 *bufferPtr,
                                  uint32 count)
{
    int32_t uartStatus;

    /* Initialize UART transaction */
    UART_Transaction_init(&instancePtr->RxTransaction);
    instancePtr->RxTransaction.buf   = bufferPtr;
    instancePtr->RxTransaction.count = count;
    instancePtr->RxTransaction.args  = (uint32_t *)instancePtr;

    /* Start async RX using MCU+SDK UART */
    uartStatus = UART_read(instancePtr->UartHandle, &instancePtr->RxTransaction);

    return uartStatus;
}

/**
 * \brief Decode payload length from variable-length encoding
 *
 * Decodes the payload length field according to DAP protocol:
 * - 1 byte: 0-127 (0x00-0x7F)
 * - 2 bytes: 128-16383 (0x80xx-0xBFxx)
 * - 3 bytes: 16384-4194303 (0xC0xxxx-0xFFFFFF)
 *
 * \param[in]  bufferPtr     Pointer to length field bytes
 * \param[in]  bufferLen     Available bytes in buffer
 * \param[out] payloadLenPtr Decoded payload length
 * \param[out] headerLenPtr  Number of bytes consumed by length field
 *
 * \return DAP_LINK_ERROR_NONE on success, error code on failure
 */
static sint32 Dap_Link_DecodePayloadLength(const uint8 *bufferPtr,
                                           uint32 bufferLen,
                                           uint32 *payloadLenPtr,
                                           uint32 *headerLenPtr)
{
    sint32 retVal = DAP_LINK_ERROR_NONE;
    uint8  firstByte;

    if ((bufferPtr == NULL_PTR) || (payloadLenPtr == NULL_PTR) ||
        (headerLenPtr == NULL_PTR) || (bufferLen == 0U))
    {
        retVal = DAP_LINK_ERROR_INVALID_PARAMS;
    }

    if (retVal == DAP_LINK_ERROR_NONE)
    {
        firstByte = bufferPtr[0];

        /* Check encoding type based on first byte */
        if ((firstByte & 0x80U) == 0U)
        {
            /* 1-byte encoding: 0x00-0x7F -> 0-127 */
            *payloadLenPtr = (uint32)firstByte;
            *headerLenPtr  = 1U;
        }
        else if ((firstByte & 0xC0U) == 0x80U)
        {
            /* 2-byte encoding: 0x80xx-0xBFxx -> 128-16383 */
            if (bufferLen >= 2U)
            {
                *payloadLenPtr = (((uint32)(firstByte & 0x3FU)) << 8U) |
                                 ((uint32)bufferPtr[1]);
                *headerLenPtr  = 2U;
            }
            else
            {
                /* Not enough bytes yet */
                retVal = DAP_LINK_ERROR_INVALID_PARAMS;
            }
        }
        else
        {
            /* 3-byte encoding: 0xC0xxxx-0xFFFFFF -> 16384-4194303 */
            if (bufferLen >= 3U)
            {
                *payloadLenPtr = (((uint32)(firstByte & 0x3FU)) << 16U) |
                                 (((uint32)bufferPtr[1]) << 8U) |
                                 ((uint32)bufferPtr[2]);
                *headerLenPtr  = 3U;
            }
            else
            {
                /* Not enough bytes yet */
                retVal = DAP_LINK_ERROR_INVALID_PARAMS;
            }
        }
    }

    return retVal;
}
