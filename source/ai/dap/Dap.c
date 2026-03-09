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

#include "Dap.h"
#include "dap_link/Dap_Link.h"
#include "dap_core/Dap_Core.h"
#include "dap_interface/Dap_Interface_Cfg.h"
#include <string.h>

/* ========================================================================== */
/*                 Private Definitions and Macros                              */
/* ========================================================================== */

/* Maximum sequence number of sample, next sample rolls over to 0 */
#define DAP_SEQUENCE_NUM_LIMIT (0x1FFFU)

/* ========================================================================== */
/*                 Public Functions                                           */
/* ========================================================================== */

sint32 Dap_InitParamsSetDefault(Dap_InitParamsType *paramsPtr)
{
    sint32 retVal;

    retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (paramsPtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        paramsPtr->InterfaceConfigPtr = (const Dap_InterfaceConfigType *)NULL_PTR;
        retVal                        = Dap_Link_InitParamsSetDefault(&paramsPtr->LinkParams);
    }

    return retVal;
}

sint32 Dap_Init(Dap_InstanceType *instancePtr, const Dap_InitParamsType *paramsPtr)
{
    sint32 retVal;

    retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (paramsPtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if (paramsPtr->InterfaceConfigPtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        instancePtr->IsInitialized = FALSE;

        retVal = Dap_Link_Init(&instancePtr->LinkInstance, &paramsPtr->LinkParams);

        if (retVal == DAP_ERROR_NONE)
        {
            uint32 i;

            instancePtr->InterfaceConfig = *(paramsPtr->InterfaceConfigPtr);

            instancePtr->PipelineConfig.Mode        = DAP_PIPELINE_MODE_UNINITIALIZED;
            instancePtr->PipelineConfig.SensorCount = 0U;
            instancePtr->PipelineConfig.ModelIndex  = 0U;

            for (i = 0U; i < DAP_INTERFACE_MAX_SENSORS; i++)
            {
                instancePtr->PipelineConfig.SensorIndex[i] = 0U;
            }

            instancePtr->StreamingContext.IsActive = FALSE;

            for (i = 0U; i < DAP_INTERFACE_MAX_SENSORS; i++)
            {
                instancePtr->StreamingContext.SequenceNumber[i] = 0U;
            }
            instancePtr->StreamingContext.TotalSampleCount   = 0U;
            instancePtr->StreamingContext.CurrentSampleCount = 0U;
            instancePtr->StreamingContext.HeaderSent         = FALSE;
            instancePtr->StreamingContext.Channel            = DAP_DATA_CHANNEL_SENSOR_SIGNAL;
            instancePtr->IsInitialized = TRUE;
        }
    }

    return retVal;
}

sint32 Dap_Open(Dap_InstanceType *instancePtr)
{
    sint32 retVal;

    retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    /* Parameter validation */
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        retVal = Dap_Link_StartReceive(&instancePtr->LinkInstance);
    }

    return retVal;
}

sint32 Dap_Process(Dap_InstanceType *instancePtr)
{
    sint32  retVal;
    boolean frameReady = FALSE;
    boolean isRxIdle   = FALSE;

    retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    /* Parameter validation */
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        (void)Dap_IsFrameReady(instancePtr, &frameReady);

        if (frameReady == TRUE)
        {
            (void)Dap_Core_ProcessFrame(instancePtr);

            retVal = Dap_Link_StartReceive(&instancePtr->LinkInstance);
        }
        else
        {
            (void)Dap_Link_IsRxIdle(&instancePtr->LinkInstance, &isRxIdle);

            if (isRxIdle == TRUE)
            {
                retVal = Dap_Link_StartReceive(&instancePtr->LinkInstance);
            }
        }
    }

    return retVal;
}

sint32 Dap_IsStreaming(const Dap_InstanceType *instancePtr, boolean *isStreamingPtr)
{
    sint32 retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    /* Parameter validation */
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (isStreamingPtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        *isStreamingPtr = instancePtr->StreamingContext.IsActive;
    }

    return retVal;
}

sint32 Dap_IsFrameReady(const Dap_InstanceType *instancePtr, boolean *isFrameReadyPtr)
{
    sint32 retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    /* Parameter validation */
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (isFrameReadyPtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        (void)Dap_Link_IsFrameReady(&instancePtr->LinkInstance, isFrameReadyPtr);
    }

    return retVal;
}

sint32 Dap_GetPipelineConfig(const Dap_InstanceType *instancePtr, Dap_PipelineConfigType *configPtr)
{
    sint32 retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    /* Parameter validation */
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (configPtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        *configPtr = instancePtr->PipelineConfig;
    }

    return retVal;
}

sint32 Dap_StartSensorStream(Dap_InstanceType *instancePtr, const uint16 *sampleSizesBytesPtr, uint32 sampleSizesCount,
                             uint32 totalSamples,Dap_DataChannelType channel)
{
    sint32                    retVal = DAP_ERROR_NONE;
    uint32                    i;
    uint32                    totalPayloadLen;
    Dap_StreamingContextType *ctxPtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
    else if (instancePtr->StreamingContext.IsActive == FALSE)
    {
        retVal = DAP_ERROR_NOT_STREAMING;
    }
    else if (sampleSizesBytesPtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if (sampleSizesCount < instancePtr->PipelineConfig.SensorCount)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if ((channel < DAP_DATA_CHANNEL_SENSOR_SIGNAL) ||

             (channel > DAP_DATA_CHANNEL_INF_LOG))

    {

        retVal = DAP_ERROR_INVALID_PARAMS;

    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        ctxPtr = &instancePtr->StreamingContext;
        // uint8 streamChannel;

        for (i = 0U; i < instancePtr->PipelineConfig.SensorCount; i++)
        {
            ctxPtr->SequenceNumber[i] = 0U;
        }

        ctxPtr->TotalSampleCount   = totalSamples;
        ctxPtr->CurrentSampleCount = 0U;
        ctxPtr->Channel            = channel;
        /* Calculate total payload length for the header */
        totalPayloadLen = 0U;
        if (totalSamples > 0U)
        {
            for (i = 0U; i < instancePtr->PipelineConfig.SensorCount; i++)
            {
#if DAP_INTERFACE_USE_SEQUENCE_HEADERS == 1
                if ((channel == DAP_DATA_CHANNEL_SENSOR_SIGNAL) || (channel == DAP_DATA_CHANNEL_INF_SIGNAL))

                {
                    /* [2 SEQ bytes + SIZEOFDATA DATA bytes] per sample */
                    totalPayloadLen += ((2U + (uint32)sampleSizesBytesPtr[i]) * totalSamples);
                }

                else

                {

                    /* [SIZEOFDATA DATA bytes] per sample */

                    totalPayloadLen += ((uint32)sampleSizesBytesPtr[i] * totalSamples);

                }
#else
                /* [SIZEOFDATA DATA bytes] per sample */
                totalPayloadLen += ((uint32)sampleSizesBytesPtr[i] * totalSamples);
#endif /* DAP_INTERFACE_USE_SEQUENCE_HEADERS == 1 */
            }
        }

        retVal = Dap_Core_SendStreamHeader(&instancePtr->LinkInstance, (uint8)channel, totalPayloadLen);

        if (retVal == DAP_ERROR_NONE)
        {
            ctxPtr->HeaderSent = TRUE;
        }
    }

    return retVal;
}

sint32 Dap_StreamSensorSample(Dap_InstanceType *instancePtr, uint8 sensorIndex, const uint8 *dataPtr,
                              uint32 sizeInBytes)
{
    sint32                    retVal = DAP_ERROR_NONE;
    uint8                     samplePacket[DAP_LINK_BUFFER_SIZE_BYTES];
    uint32                    packetIdx;
    uint8                     sensorId;
    Dap_StreamingContextType *ctxPtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
    else if (instancePtr->StreamingContext.IsActive == FALSE)
    {
        retVal = DAP_ERROR_NOT_STREAMING;
    }
    else if ((dataPtr == NULL_PTR) || (sizeInBytes == 0U))
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if (sensorIndex >= instancePtr->PipelineConfig.SensorCount)
    {
        retVal = DAP_ERROR_INVALID_SENSOR_INDEX;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        ctxPtr = &instancePtr->StreamingContext;

        /* Check that stream header was sent */
        if (ctxPtr->HeaderSent == FALSE)
        {
            retVal = DAP_ERROR_STREAM_NOT_STARTED;
        }
    }

    if (retVal == DAP_ERROR_NONE)
    {
        /* Get sensor ID from pipeline configuration */
        sensorId = instancePtr->PipelineConfig.SensorIndex[sensorIndex];

        if (sensorId >= DAP_INTERFACE_MAX_SENSORS)
        {
            retVal = DAP_ERROR_INVALID_SENSOR_INDEX;
        }
    }

    if (retVal == DAP_ERROR_NONE)
    {
        packetIdx = 0U;

#if DAP_INTERFACE_USE_SEQUENCE_HEADERS == 1
        if ((ctxPtr->Channel == (uint8)DAP_DATA_CHANNEL_SENSOR_SIGNAL) || (ctxPtr->Channel == (uint8)DAP_DATA_CHANNEL_INF_SIGNAL))

        {
            samplePacket[packetIdx++] = (((sensorId + 1U) << 5U) | ((ctxPtr->SequenceNumber[sensorIndex] >> 8U) & 0x1FU));
            samplePacket[packetIdx++] = (ctxPtr->SequenceNumber[sensorIndex] & 0xFFU);
        }
#endif /* DAP_INTERFACE_USE_SEQUENCE_HEADERS == 1 */

        if ((packetIdx + sizeInBytes) < DAP_LINK_BUFFER_SIZE_BYTES)
        {
            (void)memcpy(&samplePacket[packetIdx], dataPtr, sizeInBytes);
            packetIdx += sizeInBytes;
        }
        else
        {
            retVal = DAP_ERROR_BUFFER_OVERFLOW;
        }
    }

    if (retVal == DAP_ERROR_NONE)
    {
        /* Send the sample packet */
        retVal = Dap_Core_SendStreamSample(&instancePtr->LinkInstance, samplePacket, packetIdx);
    }

    if (retVal == DAP_ERROR_NONE)
    {
#if DAP_INTERFACE_USE_SEQUENCE_HEADERS == 1

        if ((ctxPtr->Channel == (uint8)DAP_DATA_CHANNEL_SENSOR_SIGNAL) || (ctxPtr->Channel == (uint8)DAP_DATA_CHANNEL_INF_SIGNAL))
        {
            if (ctxPtr->SequenceNumber[sensorIndex] < DAP_SEQUENCE_NUM_LIMIT)
            {
                ctxPtr->SequenceNumber[sensorIndex]++;
            }
            else
            {
                ctxPtr->SequenceNumber[sensorIndex] = 0;
            }
        }

#endif /* DAP_INTERFACE_USE_SEQUENCE_HEADERS == 1 */

        ctxPtr->CurrentSampleCount++;

        if ((ctxPtr->TotalSampleCount > 0U) && (ctxPtr->CurrentSampleCount >= ctxPtr->TotalSampleCount))
        {
            (void)Dap_StopSensorStream(instancePtr);
        }
    }

    return retVal;
}

sint32 Dap_StopSensorStream(Dap_InstanceType *instancePtr)
{
    sint32                    retVal = DAP_ERROR_NONE;
    Dap_StreamingContextType *ctxPtr;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        ctxPtr = &instancePtr->StreamingContext;

        /* Only send end byte if header was sent */
        if (ctxPtr->HeaderSent == TRUE)
        {
            retVal = Dap_Core_SendStreamEnd(&instancePtr->LinkInstance);

            /* Reset streaming context */
            ctxPtr->HeaderSent         = FALSE;
            ctxPtr->CurrentSampleCount = 0U;
            ctxPtr->TotalSampleCount   = 0U;
        }
        if((instancePtr->StreamingContext.Channel == DAP_CHANNEL_SENSOR_SIGNAL) || (instancePtr->StreamingContext.Channel == DAP_CHANNEL_INF_SIGNAL) )
        {
            instancePtr->StreamingContext.IsActive = false;
        }

    }

    return retVal;
}

sint32 Dap_DeInit(Dap_InstanceType *instancePtr)
{
    sint32 retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    /* Parameter validation */
    if (instancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
    else if (instancePtr->IsInitialized == FALSE)
    {
        retVal = DAP_ERROR_NOT_INITIALIZED;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        instancePtr->StreamingContext.IsActive = FALSE;

        (void)Dap_Link_DeInit(&instancePtr->LinkInstance);

        instancePtr->IsInitialized = FALSE;
    }

    return retVal;
}

/* ========================================================================== */
/*                 UART Callback Functions                                    */
/* ========================================================================== */

void Dap_ReceiveCallback(Dap_InstanceType *instancePtr, uint32 bytesReceived)
{
    Dap_Link_ContinueReceive(&instancePtr->LinkInstance, bytesReceived);
}

void Dap_TransmitCallback(Dap_InstanceType *instancePtr)
{
    Dap_Link_ContinueTransmit(&instancePtr->LinkInstance);
}
