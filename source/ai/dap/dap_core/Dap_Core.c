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

#include "Dap_Core.h"
#include "dap_link/Dap_Link.h"
#include "dap_interface/Dap_Interface.h"
#include "Dap.h"
#include <string.h>

/* ========================================================================== */
/*                 Internal Definitions                                       */
/* ========================================================================== */

/** Maximum response payload size */
#define DAP_CORE_RESPONSE_PAYLOAD_SIZE_BYTES \
    (DAP_LINK_BUFFER_SIZE_BYTES - (DAP_FRAME_FIXED_SIZE_BYTES + DAP_FRAME_MAX_LENGTH_SIZE_BYTES))

/* ========================================================================== */
/*                 Internal Function Prototypes                               */
/* ========================================================================== */

/* Frame validation */
static sint32 Dap_Core_ValidateFrame(Dap_Link_InstanceType *linkInstancePtr);

/* Payload length encoding */
static sint32 Dap_Core_EncodePayloadLength(uint32 length,
                                           uint8 *bufferPtr,
                                           uint32 *encodedLenPtr);

/* Response building */
static sint32 Dap_Core_BuildResponse(Dap_Link_InstanceType *linkInstancePtr,
                                     uint8 responseCode,
                                     const uint8 *payloadPtr,
                                     uint32 payloadLen);

static sint32 Dap_Core_BuildErrorResponse(Dap_Link_InstanceType *linkInstancePtr,
                                          uint8 originalCmd,
                                          uint8 errorNum);

/* Command handlers */
static sint32 Dap_Core_HandleGetCapabilities(Dap_Link_InstanceType *linkInstancePtr);
static sint32 Dap_Core_HandleListSensors(Dap_Link_InstanceType *linkInstancePtr,
                                         const Dap_InterfaceConfigType *interfaceConfigPtr);
static sint32 Dap_Core_HandleConfigurePipeline(Dap_Link_InstanceType *linkInstancePtr,
                                               Dap_PipelineConfigType *pipelineConfigPtr,
                                               const uint8 *payloadPtr,
                                               uint32 payloadLen);
static sint32 Dap_Core_HandleListModels(Dap_Link_InstanceType *linkInstancePtr,
                                        const Dap_InterfaceConfigType *interfaceConfigPtr);
static sint32 Dap_Core_HandleStartStreaming(Dap_Link_InstanceType *linkInstancePtr,
                                            volatile boolean *isStreamingPtr);
static sint32 Dap_Core_HandleStopStreaming(Dap_InstanceType *instancePtr);
static sint32 Dap_Core_HandleListInfValues(Dap_Link_InstanceType *linkInstancePtr,
                                           const Dap_InterfaceConfigType *interfaceConfigPtr);
static sint32 Dap_Core_HandleReadProperty(Dap_Link_InstanceType *linkInstancePtr,
                                          const Dap_InterfaceConfigType *interfaceConfigPtr,
                                          const uint8 *payloadPtr,
                                          uint32 payloadLen);
static sint32 Dap_Core_HandleWriteProperty(Dap_Link_InstanceType *linkInstancePtr,
                                           Dap_InterfaceConfigType *interfaceConfigPtr,
                                           const uint8 *payloadPtr,
                                           uint32 payloadLen);
static sint32 Dap_Core_HandleListProperties(Dap_Link_InstanceType *linkInstancePtr,
                                            const Dap_InterfaceConfigType *interfaceConfigPtr);

/* ========================================================================== */
/*                 Public Functions                                           */
/* ========================================================================== */

sint32 Dap_Core_ProcessFrame(Dap_InstanceType *instancePtr)
{
    sint32                 retVal;
    const uint8           *bufferPtr;
    uint32                 bufferLen;
    uint8                  command;
    uint32                 payloadLen;
    uint32                 headerLen;
    Dap_Link_InstanceType *linkInstancePtr;

    retVal = DAP_ERROR_NONE;

    /* Validate frame */
    linkInstancePtr = &instancePtr->LinkInstance;
    retVal          = Dap_Link_GetRxFrameData(linkInstancePtr, &bufferPtr, &bufferLen);
    retVal          = Dap_Core_ValidateFrame(linkInstancePtr);

    if (retVal == DAP_ERROR_NONE)
    {
        /* Extract command and payloads */
        command = bufferPtr[DAP_FRAME_OFFSET_CMD];

        retVal = Dap_Core_DecodePayloadLength(&bufferPtr[DAP_FRAME_OFFSET_LENGTH],
                                              bufferLen - DAP_FRAME_OFFSET_LENGTH,
                                              &payloadLen,
                                              &headerLen);
    }

    if (retVal == DAP_ERROR_NONE)
    {
        /* Payload offset: header (start + cmd) + length field size */
        uint32 payloadOffset = DAP_FRAME_HEADER_SIZE_BYTES + headerLen;

        switch (command)
        {
            case DAP_CMD_GET_CAPABILITIES:
                retVal = Dap_Core_HandleGetCapabilities(linkInstancePtr);
                break;

            case DAP_CMD_LIST_SENSORS:
                retVal = Dap_Core_HandleListSensors(linkInstancePtr,
                                                    &instancePtr->InterfaceConfig);
                break;

            case DAP_CMD_CONFIGURE_PIPELINE:
                retVal = Dap_Core_HandleConfigurePipeline(linkInstancePtr,
                                                          &instancePtr->PipelineConfig,
                                                          &bufferPtr[payloadOffset],
                                                          payloadLen);
                break;

            case DAP_CMD_LIST_MODELS:
                retVal = Dap_Core_HandleListModels(linkInstancePtr,
                                                   &instancePtr->InterfaceConfig);
                break;

            case DAP_CMD_START_STREAMING:
                retVal = Dap_Core_HandleStartStreaming(linkInstancePtr,
                                                       &instancePtr->StreamingContext.IsActive);
                break;

            case DAP_CMD_STOP_STREAMING:
                retVal = Dap_Core_HandleStopStreaming(instancePtr);
                break;

            case DAP_CMD_LIST_INF_VALUES:
                retVal = Dap_Core_HandleListInfValues(linkInstancePtr,
                                                      &instancePtr->InterfaceConfig);
                break;

            case DAP_CMD_READ_PROPERTY:
                retVal = Dap_Core_HandleReadProperty(linkInstancePtr,
                                                     &instancePtr->InterfaceConfig,
                                                     &bufferPtr[payloadOffset],
                                                     payloadLen);
                break;

            case DAP_CMD_WRITE_PROPERTY:
                retVal = Dap_Core_HandleWriteProperty(linkInstancePtr,
                                                      &instancePtr->InterfaceConfig,
                                                      &bufferPtr[payloadOffset],
                                                      payloadLen);
                break;

            case DAP_CMD_LIST_PROPERTIES:
                retVal = Dap_Core_HandleListProperties(linkInstancePtr,
                                                       &instancePtr->InterfaceConfig);
                break;

            case DAP_CMD_REMOVE_MODEL:
            case DAP_CMD_START_MODEL_UPLOAD:
            case DAP_CMD_END_MODEL_UPLOAD:
                /* Unsupported commands */
                (void)Dap_Core_BuildErrorResponse(linkInstancePtr,
                                                  command,
                                                  DAP_PROTOCOL_ERROR_UNSUPPORTED_CMD);
                break;

            default:
                /* Unknown command */
                (void)Dap_Core_BuildErrorResponse(linkInstancePtr,
                                                  command,
                                                  DAP_PROTOCOL_ERROR_UNKNOWN_CMD);
                break;
        }

        if (retVal != DAP_ERROR_NONE)
        {
            (void)Dap_Core_BuildErrorResponse(linkInstancePtr, command,
                                              DAP_PROTOCOL_ERROR_INVALID_PAYLOAD);
        }
    }
    else
    {
        /* Frame validation failed */
        uint8 protocolError;

        if (bufferLen >= DAP_FRAME_HEADER_SIZE_BYTES)
        {
            command = bufferPtr[DAP_FRAME_OFFSET_CMD];
        }
        else
        {
            command = 0U;
        }

        switch (retVal)
        {
            case DAP_ERROR_INVALID_START_BYTE:
                protocolError = DAP_PROTOCOL_ERROR_INVALID_START_BYTE;
                break;
            case DAP_ERROR_INVALID_END_BYTE:
                protocolError = DAP_PROTOCOL_ERROR_INVALID_END_BYTE;
                break;
            case DAP_ERROR_INVALID_CRC:
                protocolError = DAP_PROTOCOL_ERROR_INVALID_CRC;
                break;
            default:
                protocolError = DAP_PROTOCOL_ERROR_INVALID_PAYLOAD;
                break;
        }

        (void)Dap_Core_BuildErrorResponse(linkInstancePtr, command, protocolError);
    }

    return retVal;
}


/* ========================================================================== */
/*                 Internal Functions                                         */
/* ========================================================================== */

/* Validate received frame */
static sint32 Dap_Core_ValidateFrame(Dap_Link_InstanceType *linkInstancePtr)
{
    sint32       retVal;
    uint8        endByte;
    const uint8 *bufferPtr;
    uint32       length;

    retVal = Dap_Link_GetRxFrameData(linkInstancePtr, &bufferPtr, &length);

    if (retVal != DAP_ERROR_NONE)
    {
        return retVal;
    }

    /* Reset protocol error */
    linkInstancePtr->LastProtocolError = 0U;
    retVal                             = DAP_ERROR_NONE;

    if (length < DAP_FRAME_MIN_SIZE_BYTES)
    {
        retVal = DAP_ERROR_INVALID_PAYLOAD;
    }
    else if (bufferPtr[DAP_FRAME_OFFSET_START] != DAP_FRAME_START_BYTE)
    {
        retVal = DAP_ERROR_INVALID_START_BYTE;
    }
    else
    {
        endByte = bufferPtr[length - DAP_FRAME_TRAILER_SIZE_BYTES];
        if (endByte != DAP_FRAME_END_BYTE)
        {
            retVal = DAP_ERROR_INVALID_END_BYTE;
        }
    }

    return retVal;
}

/* Encode payload length into buffer */
static sint32 Dap_Core_EncodePayloadLength(uint32 length,
                                           uint8 *bufferPtr,
                                           uint32 *encodedLenPtr)
{
    sint32 retVal = DAP_ERROR_NONE;

    if ((bufferPtr == NULL_PTR) || (encodedLenPtr == NULL_PTR))
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if (length <= DAP_PAYLOAD_1B_MAX)
    {
        /* 1-byte encoding (0-127) */
        bufferPtr[0]   = (uint8)length;
        *encodedLenPtr = 1U;
    }
    else if (length <= DAP_PAYLOAD_2B_MAX)
    {
        /* 2-byte encoding (128-16383) */
        bufferPtr[0]   = (uint8)(0x80U | ((length >> 8U) & 0x3FU));
        bufferPtr[1]   = (uint8)(length & 0xFFU);
        *encodedLenPtr = 2U;
    }
    else if (length <= DAP_PAYLOAD_3B_MAX)
    {
        /* 3-byte encoding (16384-4194303) */
        bufferPtr[0]   = (uint8)(0xC0U | ((length >> 16U) & 0x3FU));
        bufferPtr[1]   = (uint8)((length >> 8U) & 0xFFU);
        bufferPtr[2]   = (uint8)(length & 0xFFU);
        *encodedLenPtr = 3U;
    }
    else
    {
        retVal = DAP_ERROR_BUFFER_OVERFLOW;
    }

    return retVal;
}

/* Decode payload length from buffer */
sint32 Dap_Core_DecodePayloadLength(const uint8 *bufferPtr,
                                    uint32 bufferLen,
                                    uint32 *payloadLenPtr,
                                    uint32 *headerLenPtr)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  firstByte;

    if ((bufferPtr == NULL_PTR) || (payloadLenPtr == NULL_PTR) || (headerLenPtr == NULL_PTR))
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else if (bufferLen < 1U)
    {
        retVal = DAP_ERROR_INVALID_PARAMS;
    }
    else
    {
        firstByte = bufferPtr[0];

        if ((firstByte & 0x80U) == 0U)
        {
            /* 1-byte length (0-127) */
            *payloadLenPtr = (uint32)firstByte;
            *headerLenPtr  = 1U;
        }
        else if ((firstByte & 0xC0U) == 0x80U)
        {
            /* 2-byte length (128-16383) */
            if (bufferLen >= 2U)
            {
                *payloadLenPtr = (((uint32)(firstByte & 0x3FU)) << 8U) |
                                 ((uint32)bufferPtr[1]);
                *headerLenPtr  = 2U;
            }
            else
            {
                retVal = DAP_ERROR_INVALID_PARAMS;
            }
        }
        else if ((firstByte & 0xC0U) == 0xC0U)
        {
            /* 3-byte length (16384-4194303) */
            if (bufferLen >= 3U)
            {
                *payloadLenPtr = (((uint32)(firstByte & 0x3FU)) << 16U) |
                                 (((uint32)bufferPtr[1]) << 8U) |
                                 ((uint32)bufferPtr[2]);
                *headerLenPtr  = 3U;
            }
            else
            {
                retVal = DAP_ERROR_INVALID_PARAMS;
            }
        }
        else
        {
            retVal = DAP_ERROR_INVALID_PARAMS;
        }
    }

    return retVal;
}

/* Build response frame into Link Tx Instance Frame buffer */
static sint32 Dap_Core_BuildResponse(Dap_Link_InstanceType *linkInstancePtr,
                                     uint8 responseCode,
                                     const uint8 *payloadPtr,
                                     uint32 payloadLen)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint32 idx    = 0U;
    uint32 encodedLenSize;
    uint8  lengthBuffer[3];
    uint8 *responseBufferPtr;
    uint32 maxBufferLen;
    uint32 totalFrameLen;

    /* Wait for any previous transmission to complete before overwriting buffer */
    retVal = Dap_Link_WaitForTxReady(linkInstancePtr);

    if (retVal == DAP_ERROR_NONE)
    {
        (void)Dap_Link_GetTxFrameBuffer(linkInstancePtr, &responseBufferPtr, &maxBufferLen);

        /* Start byte */
        responseBufferPtr[idx++] = DAP_FRAME_START_BYTE;

        /* Response code */
        responseBufferPtr[idx++] = responseCode;

        /* Encode payload length */
        retVal = Dap_Core_EncodePayloadLength(payloadLen, lengthBuffer, &encodedLenSize);
    }

    if (retVal == DAP_ERROR_NONE)
    {
        totalFrameLen = DAP_FRAME_FIXED_SIZE_BYTES + encodedLenSize + payloadLen;

        if (totalFrameLen > maxBufferLen)
        {
            retVal = DAP_ERROR_BUFFER_OVERFLOW;
        }
    }

    if (retVal == DAP_ERROR_NONE)
    {
        /* Copy encoded length */
        (void)memcpy(&responseBufferPtr[idx], lengthBuffer, encodedLenSize);
        idx += encodedLenSize;

        /* Copy payload if present */
        if ((payloadPtr != NULL_PTR) && (payloadLen > 0U))
        {
            (void)memcpy(&responseBufferPtr[idx], payloadPtr, payloadLen);
            idx += payloadLen;
        }

        /* End Byte */
        responseBufferPtr[idx++] = DAP_FRAME_END_BYTE;

        (void)Dap_Link_SetTxFrameLength(linkInstancePtr, idx);
    }

    return retVal;
}

/* Build error response */
static sint32 Dap_Core_BuildErrorResponse(Dap_Link_InstanceType *linkInstancePtr,
                                          uint8 originalCmd,
                                          uint8 errorNum)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  payload[2];

    /* Store protocol error code */
    linkInstancePtr->LastProtocolError = errorNum;

    /* Build error payload: original command + error number */
    payload[0] = originalCmd;
    payload[1] = errorNum;

    retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_ERROR, payload, 2U);

    /* Send error response directly */
    if (retVal == DAP_ERROR_NONE)
    {
        retVal = Dap_Link_Send(linkInstancePtr);
    }

    return retVal;
}

/* Handle Get Capabilities command */
static sint32 Dap_Core_HandleGetCapabilities(Dap_Link_InstanceType *linkInstancePtr)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  payload[4];
    uint8  i = 0;
    uint8  capabilityByte;

    capabilityByte = ((DAP_INTERFACE_CRC_ENABLED << 4) |
                      (DAP_INTERFACE_BIDIRECTIONAL_ENABLED << 3) |
                      (DAP_INTERFACE_BIG_ENDIAN_ENABLED << 2) |
                      (DAP_INTERFACE_UPLOAD_ENABLED << 1) |
                      (DAP_INTERFACE_INFERENCE_ENABLED));

    payload[i++] = DAP_INTERFACE_API_VERSION;
    payload[i++] = capabilityByte;
    payload[i++] = DAP_INTERFACE_SDK_MAJOR_VERSION;
    payload[i++] = DAP_INTERFACE_SDK_MINOR_VERSION;

    retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_GET_CAPABILITIES, payload, 4U);

    if (retVal == DAP_ERROR_NONE)
    {
        retVal = Dap_Link_Send(linkInstancePtr);
    }

    return retVal;
}

/* Handle List Sensors command */
static sint32 Dap_Core_HandleListSensors(Dap_Link_InstanceType         *linkInstancePtr,
                                         const Dap_InterfaceConfigType *interfaceConfigPtr)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  payload[DAP_CORE_RESPONSE_PAYLOAD_SIZE_BYTES];
    uint32 payloadIdx;
    uint8  sensorCount;
    uint8  i;

    sensorCount = interfaceConfigPtr->SensorCount;

    /* Validate count does not exceed array bounds */
    if (sensorCount > DAP_INTERFACE_MAX_SENSORS)
    {
        sensorCount = DAP_INTERFACE_MAX_SENSORS;
    }

    /* Handle empty list */
    if (sensorCount == 0U)
    {
        payload[0] = 0U;
        retVal     = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_LIST_SENSORS, payload, 1U);
        if (retVal == DAP_ERROR_NONE)
        {
            retVal = Dap_Link_Send(linkInstancePtr);
        }
        return retVal;
    }

    /* Send one frame per sensor */
    for (i = 0U; (i < sensorCount) && (retVal == DAP_ERROR_NONE); i++)
    {
        const char *sensorJsonPtr;
        uint32      jsonLen;

        sensorJsonPtr = interfaceConfigPtr->SensorList[i];
        if (sensorJsonPtr == NULL_PTR)
        {
            continue;
        }

        jsonLen    = (uint32)strlen(sensorJsonPtr);
        payloadIdx = 0U;

        /* Build payload: count + index + JSON */
        payload[payloadIdx++] = sensorCount;
        payload[payloadIdx++] = i;

        if ((payloadIdx + jsonLen) > DAP_CORE_RESPONSE_PAYLOAD_SIZE_BYTES)
        {
            retVal = DAP_ERROR_BUFFER_OVERFLOW;
            break;
        }

        (void)memcpy(&payload[payloadIdx], sensorJsonPtr, jsonLen);
        payloadIdx += jsonLen;

        /* Build and send frame */
        retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_LIST_SENSORS, payload, payloadIdx);
        if (retVal == DAP_ERROR_NONE)
        {
            retVal = Dap_Link_Send(linkInstancePtr);
        }
    }

    return retVal;
}

/* Handle Configure Pipeline command */
static sint32 Dap_Core_HandleConfigurePipeline(Dap_Link_InstanceType  *linkInstancePtr,
                                               Dap_PipelineConfigType *pipelineConfigPtr,
                                               const uint8 *payloadPtr,
                                               uint32 payloadLen)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  mode;
    uint8  modelIndex;
    uint8  sensorCount;
    uint8  i;

    if ((payloadPtr == NULL_PTR) || (payloadLen < 2U))
    {
        retVal = DAP_ERROR_INVALID_PAYLOAD;
    }
    else
    {
        /* Extract fields from payload */
        mode       = payloadPtr[0];
        modelIndex = payloadPtr[1];

        /* Validate mode */
        if ((mode < DAP_PIPELINE_MODE_DATA_ACQUISITION) ||
            (mode > DAP_PIPELINE_MODE_LOOPBACK))
        {
            retVal = DAP_ERROR_INVALID_PARAMS;
        }
        else
        {
            /* Store pipeline configuration */
            pipelineConfigPtr->Mode       = (Dap_PipelineModeType)mode;
            pipelineConfigPtr->ModelIndex = modelIndex;

            /* Extract sensor indices (remaining bytes) */
            sensorCount = payloadLen - 2U;
            if (sensorCount > DAP_INTERFACE_MAX_SENSORS)
            {
                sensorCount = DAP_INTERFACE_MAX_SENSORS;
            }

            pipelineConfigPtr->SensorCount = sensorCount;
            for (i = 0U; i < sensorCount; i++)
            {
                uint8 sensorIdx = payloadPtr[2U + i];

                if (sensorIdx >= DAP_INTERFACE_MAX_SENSORS)
                {
                    retVal = DAP_ERROR_INVALID_PARAMS;
                    break;
                }

                pipelineConfigPtr->SensorIndex[i] = sensorIdx;
            }

            retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_CONFIGURE_PIPELINE, (const uint8 *)NULL_PTR, 0U);

            if (retVal == DAP_ERROR_NONE)
            {
                retVal = Dap_Link_Send(linkInstancePtr);
            }
        }
    }

    return retVal;
}

/* Handle List Models command */
static sint32 Dap_Core_HandleListModels(Dap_Link_InstanceType         *linkInstancePtr,
                                        const Dap_InterfaceConfigType *interfaceConfigPtr)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  payload[DAP_CORE_RESPONSE_PAYLOAD_SIZE_BYTES];
    uint32 payloadIdx;
    uint8  modelCount;
    uint8  i;

    modelCount = interfaceConfigPtr->ModelCount;

    if (modelCount > DAP_INTERFACE_MAX_MODELS)
    {
        modelCount = DAP_INTERFACE_MAX_MODELS;
    }

    /* Handle empty list */
    if (modelCount == 0U)
    {
        payload[0] = 0U;
        retVal     = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_LIST_MODELS, payload, 1U);
        if (retVal == DAP_ERROR_NONE)
        {
            retVal = Dap_Link_Send(linkInstancePtr);
        }
        return retVal;
    }

    /* Send one frame per model */
    for (i = 0U; (i < modelCount) && (retVal == DAP_ERROR_NONE); i++)
    {
        const char *modelJsonPtr;
        uint32      jsonLen;

        modelJsonPtr = interfaceConfigPtr->ModelList[i];
        if (modelJsonPtr == NULL_PTR)
        {
            continue;
        }

        jsonLen    = (uint32)strlen(modelJsonPtr);
        payloadIdx = 0U;

        /* Build payload: count + index + JSON */
        payload[payloadIdx++] = modelCount;
        payload[payloadIdx++] = i;
        (void)memcpy(&payload[payloadIdx], modelJsonPtr, jsonLen);
        payloadIdx += jsonLen;

        retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_LIST_MODELS, payload, payloadIdx);

        if (retVal == DAP_ERROR_NONE)
        {
            retVal = Dap_Link_Send(linkInstancePtr);
        }
    }

    return retVal;
}

/* Handle Start Streaming command */
static sint32 Dap_Core_HandleStartStreaming(Dap_Link_InstanceType *linkInstancePtr,
                                            volatile boolean *isStreamingPtr)
{
    sint32 retVal = DAP_ERROR_NONE;

    /* Set streaming flag */
    *isStreamingPtr = TRUE;

    /* Build empty ACK response */
    retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_START_STREAMING, (const uint8 *)NULL_PTR, 0U);

    if (retVal == DAP_ERROR_NONE)
    {
        retVal = Dap_Link_Send(linkInstancePtr);
    }

    return retVal;
}

/* Handle Stop Streaming command */
static sint32 Dap_Core_HandleStopStreaming(Dap_InstanceType *instancePtr)
{
    sint32                 retVal = DAP_ERROR_NONE;
    Dap_Link_InstanceType *linkInstancePtr;

    linkInstancePtr = &instancePtr->LinkInstance;

    /* If streaming with header sent, send end byte first */
    if (instancePtr->StreamingContext.HeaderSent == TRUE)
    {
        (void)Dap_Core_SendStreamEnd(linkInstancePtr);

        /* Reset streaming context */
        instancePtr->StreamingContext.HeaderSent         = FALSE;
        instancePtr->StreamingContext.CurrentSampleCount = 0U;
        instancePtr->StreamingContext.TotalSampleCount   = 0U;
    }

    /* Clear streaming flag */
    instancePtr->StreamingContext.IsActive = FALSE;

    retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_STOP_STREAMING, (const uint8 *)NULL_PTR, 0U);

    if (retVal == DAP_ERROR_NONE)
    {
        retVal = Dap_Link_Send(linkInstancePtr);
    }

    return retVal;
}

/* Handle List Inferencing Values command */
static sint32 Dap_Core_HandleListInfValues(Dap_Link_InstanceType         *linkInstancePtr,
                                           const Dap_InterfaceConfigType *interfaceConfigPtr)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  payload[DAP_CORE_RESPONSE_PAYLOAD_SIZE_BYTES];
    uint32 payloadIdx;
    uint8  infValueCount;
    uint8  i;

    infValueCount = interfaceConfigPtr->InfValueCount;

    if (infValueCount > DAP_INTERFACE_MAX_INF_VALUES)
    {
        infValueCount = DAP_INTERFACE_MAX_INF_VALUES;
    }

    /* Handle empty list */
    if (infValueCount == 0U)
    {
        payload[0] = 0U;
        retVal     = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_LIST_INF_VALUES, payload, 1U);
        if (retVal == DAP_ERROR_NONE)
        {
            retVal = Dap_Link_Send(linkInstancePtr);
        }
        return retVal;
    }

    /* Send one frame per inference value */
    for (i = 0U; (i < infValueCount) && (retVal == DAP_ERROR_NONE); i++)
    {
        const Dap_Interface_InfValueInfoType *infValuePtr;
        uint32                                nameLen;

        infValuePtr = interfaceConfigPtr->InfValueList[i];
        if (infValuePtr == NULL_PTR)
        {
            continue;
        }

        nameLen    = (uint32)strlen(infValuePtr->NamePtr);
        payloadIdx = 0U;

        /* Build payload: count + index + format + name */
        payload[payloadIdx++] = infValueCount;
        payload[payloadIdx++] = i;
        payload[payloadIdx++] = (uint8)infValuePtr->Format;
        (void)memcpy(&payload[payloadIdx], infValuePtr->NamePtr, nameLen);
        payloadIdx += nameLen;

        retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_LIST_INF_VALUES, payload, payloadIdx);

        if (retVal == DAP_ERROR_NONE)
        {
            retVal = Dap_Link_Send(linkInstancePtr);
        }
    }

    return retVal;
}

/* Handle Read Property command */
static sint32 Dap_Core_HandleReadProperty(Dap_Link_InstanceType         *linkInstancePtr,
                                          const Dap_InterfaceConfigType *interfaceConfigPtr,
                                          const uint8 *payloadPtr,
                                          uint32 payloadLen)
{
    sint32                          retVal = DAP_ERROR_NONE;
    uint8                           payload[10];
    uint32                          payloadIdx = 0U;
    uint8                           propertyId;
    Dap_Interface_PropertyInfoType *propertyPtr;

    if ((payloadPtr == NULL_PTR) || (payloadLen < 1U))
    {
        retVal = DAP_ERROR_INVALID_PAYLOAD;
    }
    else
    {
        /* Extract property ID */
        propertyId = payloadPtr[0];

        /* Validate property ID */
        if (propertyId >= interfaceConfigPtr->PropertyCount)
        {
            retVal = DAP_ERROR_INVALID_PARAMS;
        }
        else
        {
            propertyPtr = interfaceConfigPtr->PropertyList[propertyId];

            if (propertyPtr == NULL_PTR)
            {
                retVal = DAP_ERROR_INVALID_PARAMS;
            }
            else
            {
                /* Build payload: property ID + value */
                payload[payloadIdx++] = propertyId;

                /* Add value based on type */
                switch (propertyPtr->Type)
                {
                    case DAP_DATA_FORMAT_UINT8:
                    case DAP_DATA_FORMAT_INT8:
                        payload[payloadIdx++] = propertyPtr->Value.U8;
                        break;

                    case DAP_DATA_FORMAT_UINT16:
                    case DAP_DATA_FORMAT_INT16:
                        payload[payloadIdx++] = (uint8)(propertyPtr->Value.U16 >> 8U);
                        payload[payloadIdx++] = (uint8)(propertyPtr->Value.U16 & 0xFFU);
                        break;

                    case DAP_DATA_FORMAT_UINT32:
                    case DAP_DATA_FORMAT_INT32:
                        payload[payloadIdx++] = (uint8)(propertyPtr->Value.U32 >> 24U);
                        payload[payloadIdx++] = (uint8)((propertyPtr->Value.U32 >> 16U) & 0xFFU);
                        payload[payloadIdx++] = (uint8)((propertyPtr->Value.U32 >> 8U) & 0xFFU);
                        payload[payloadIdx++] = (uint8)(propertyPtr->Value.U32 & 0xFFU);
                        break;

                    default:
                        retVal = DAP_ERROR_INVALID_PARAMS;
                        break;
                }

                if (retVal == DAP_ERROR_NONE)
                {
                    retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_READ_PROPERTY, payload, payloadIdx);

                    if (retVal == DAP_ERROR_NONE)
                    {
                        retVal = Dap_Link_Send(linkInstancePtr);
                    }
                }
            }
        }
    }

    return retVal;
}

/* Handle Write Property command */
static sint32 Dap_Core_HandleWriteProperty(Dap_Link_InstanceType   *linkInstancePtr,
                                           Dap_InterfaceConfigType *interfaceConfigPtr,
                                           const uint8 *payloadPtr,
                                           uint32 payloadLen)
{
    sint32                          retVal = DAP_ERROR_NONE;
    uint8                           payload[1];
    uint8                           propertyId;
    Dap_Interface_PropertyInfoType *propertyPtr;

    if ((payloadPtr == NULL_PTR) || (payloadLen < 2U))
    {
        retVal = DAP_ERROR_INVALID_PAYLOAD;
    }
    else
    {
        /* Extract property ID */
        propertyId = payloadPtr[0];

        /* Validate property ID */
        if (propertyId >= interfaceConfigPtr->PropertyCount)
        {
            retVal = DAP_ERROR_INVALID_PARAMS;
        }
        else
        {
            propertyPtr = interfaceConfigPtr->PropertyList[propertyId];

            if (propertyPtr == NULL_PTR)
            {
                retVal = DAP_ERROR_INVALID_PARAMS;
            }
            else
            {
                /* Write value based on type */
                switch (propertyPtr->Type)
                {
                    case DAP_DATA_FORMAT_UINT8:
                    case DAP_DATA_FORMAT_INT8:
                        if (payloadLen >= 2U)
                        {
                            propertyPtr->Value.U8 = payloadPtr[1];
                        }
                        else
                        {
                            retVal = DAP_ERROR_INVALID_PAYLOAD;
                        }
                        break;

                    case DAP_DATA_FORMAT_UINT16:
                    case DAP_DATA_FORMAT_INT16:
                        if (payloadLen >= 3U)
                        {
                            propertyPtr->Value.U16 = (((uint16)payloadPtr[1]) << 8U) |
                                                     ((uint16)payloadPtr[2]);
                        }
                        else
                        {
                            retVal = DAP_ERROR_INVALID_PAYLOAD;
                        }
                        break;

                    case DAP_DATA_FORMAT_UINT32:
                    case DAP_DATA_FORMAT_INT32:
                        if (payloadLen >= 5U)
                        {
                            propertyPtr->Value.U32 = (((uint32)payloadPtr[1]) << 24U) |
                                                     (((uint32)payloadPtr[2]) << 16U) |
                                                     (((uint32)payloadPtr[3]) << 8U) |
                                                     ((uint32)payloadPtr[4]);
                        }
                        else
                        {
                            retVal = DAP_ERROR_INVALID_PAYLOAD;
                        }
                        break;

                    default:
                        retVal = DAP_ERROR_INVALID_PARAMS;
                        break;
                }

                if (retVal == DAP_ERROR_NONE)
                {
                    /* Build response with property ID */
                    payload[0] = propertyId;

                    retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_WRITE_PROPERTY, payload, 1U);

                    if (retVal == DAP_ERROR_NONE)
                    {
                        retVal = Dap_Link_Send(linkInstancePtr);
                    }
                }
            }
        }
    }

    return retVal;
}

/* Handle List Properties command */
static sint32 Dap_Core_HandleListProperties(Dap_Link_InstanceType         *linkInstancePtr,
                                            const Dap_InterfaceConfigType *interfaceConfigPtr)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  payload[DAP_CORE_RESPONSE_PAYLOAD_SIZE_BYTES];
    uint32 payloadIdx;
    uint8  propertyCount;
    uint8  i;

    propertyCount = interfaceConfigPtr->PropertyCount;

    /* Validate count does not exceed array bounds */
    if (propertyCount > DAP_INTERFACE_MAX_PROPERTIES)
    {
        propertyCount = DAP_INTERFACE_MAX_PROPERTIES;
    }

    /* Handle empty list */
    if (propertyCount == 0U)
    {
        payload[0] = 0U;
        retVal     = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_LIST_PROPERTIES, payload, 1U);
        if (retVal == DAP_ERROR_NONE)
        {
            retVal = Dap_Link_Send(linkInstancePtr);
        }
        return retVal;
    }

    /* Send one frame per property */
    for (i = 0U; (i < propertyCount) && (retVal == DAP_ERROR_NONE); i++)
    {
        Dap_Interface_PropertyInfoType *propertyPtr;
        uint32                          nameLen;

        propertyPtr = interfaceConfigPtr->PropertyList[i];
        if (propertyPtr == NULL_PTR)
        {
            continue;
        }

        nameLen    = (uint32)strlen(propertyPtr->NamePtr);
        payloadIdx = 0U;

        /* Build payload: count + index + format + name */
        payload[payloadIdx++] = propertyCount;
        payload[payloadIdx++] = i;
        payload[payloadIdx++] = (uint8)propertyPtr->Type;
        (void)memcpy(&payload[payloadIdx], propertyPtr->NamePtr, nameLen);
        payloadIdx += nameLen;

        retVal = Dap_Core_BuildResponse(linkInstancePtr, DAP_RESP_LIST_PROPERTIES, payload, payloadIdx);

        if (retVal == DAP_ERROR_NONE)
        {
            retVal = Dap_Link_Send(linkInstancePtr);
        }
    }

    return retVal;
}

/* ========================================================================== */
/*                 De-initialization Function                                 */
/* ========================================================================== */

sint32 Dap_Core_DeInit(Dap_Link_InstanceType *linkInstancePtr)
{
    sint32 retVal = DAP_ERROR_NONE;

#if DAP_CFG_ERROR_CHECK == STD_ON
    if (linkInstancePtr == NULL_PTR)
    {
        retVal = DAP_ERROR_INVALID_INSTANCE;
    }
#endif /* DAP_CFG_ERROR_CHECK == STD_ON */

    if (retVal == DAP_ERROR_NONE)
    {
        /* Reset protocol error state */
        linkInstancePtr->LastProtocolError = 0U;
        linkInstancePtr->LastFrameError = 0U;
    }

    return retVal;
}

/* ========================================================================== */
/*                 Streaming Functions                                        */
/* ========================================================================== */

sint32 Dap_Core_SendStreamHeader(Dap_Link_InstanceType *linkInstancePtr,
                                 uint8 channel,
                                 uint32 totalPayloadLen)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  headerBuffer[6];
    uint8  lengthBuffer[3];
    uint32 idx = 0U;
    uint32 encodedLenSize;
    uint8  responseCode;

    responseCode = DAP_RESP_RECEIVE_DATA | channel;

    headerBuffer[idx++] = DAP_FRAME_START_BYTE;
    headerBuffer[idx++] = responseCode;

    retVal = Dap_Core_EncodePayloadLength(totalPayloadLen, lengthBuffer, &encodedLenSize);

    if (retVal == DAP_ERROR_NONE)
    {
        (void)memcpy(&headerBuffer[idx], lengthBuffer, encodedLenSize);
        idx += encodedLenSize;

        retVal = Dap_Link_SendRaw(linkInstancePtr, headerBuffer, idx);
    }

    return retVal;
}

sint32 Dap_Core_SendStreamSample(Dap_Link_InstanceType *linkInstancePtr,
                                 uint8 *packetPtr,
                                 uint32 packetLen)
{
    sint32 retVal = DAP_ERROR_NONE;

    retVal = Dap_Link_SendRaw(linkInstancePtr, packetPtr, packetLen);

    return retVal;
}

sint32 Dap_Core_SendStreamEnd(Dap_Link_InstanceType *linkInstancePtr)
{
    sint32 retVal = DAP_ERROR_NONE;
    uint8  endByte;

    endByte = DAP_FRAME_END_BYTE;
    retVal  = Dap_Link_SendRaw(linkInstancePtr, &endByte, 1U);

    return retVal;
}