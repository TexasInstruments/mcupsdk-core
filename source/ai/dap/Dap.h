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
 * \defgroup DAP_AM263X DAP API
 * \ingroup COMPONENTS_AM263X
 *
 * Device Agent Protocol (DAP) for Edge AI Studio communication.
 *
 * The DAP component provides a communication interface between Edge AI Studio
 * (host) and TI devices (target) over UART for data acquisition and sensor
 * streaming.
 *
 * ## Usage
 * 1. Initialize DAP with \ref Dap_Init
 * 2. Call \ref Dap_Process in main loop to handle commands
 * 3. Use \ref Dap_SendSensorData to stream sensor data when streaming is active
 *
 * @{
 */

#ifndef DAP_H
#define DAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Dap_Types.h"
#include "dap_link/Dap_Link.h"
#include "dap_core/Dap_Core.h"
#include "dap_interface/Dap_Interface.h"

/* ========================================================================== */
/*                 Public Functions                                           */
/* ========================================================================== */

/**
 * \brief Set default initialization parameters
 *
 * Sets all fields in the initialization parameters structure to default values.
 * Application should call this before modifying specific fields.
 *
 * \param[out] paramsPtr Pointer to initialization parameters structure
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_InitParamsSetDefault(Dap_InitParamsType *paramsPtr);

/**
 * \brief Initialize DAP instance
 *
 * Initializes all DAP layers (link, core, interface) and prepares
 * for communication with the host. The communication hardware is
 * initialized internally by the link layer.
 *
 * \param[out] instancePtr Pointer to DAP instance structure
 * \param[in]  paramsPtr   Pointer to initialization parameters
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Init(Dap_InstanceType *instancePtr, const Dap_InitParamsType *paramsPtr);

/**
 * \brief Start DAP instance
 *
 * Start receiving incoming frames from host. Dap_Init must be called prior
 * to calling this function
 *
 * \param[out] instancePtr Pointer to DAP instance structure
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Open(Dap_InstanceType *instancePtr);

/**
 * \brief Process DAP communication
 *
 * Main processing function to be called in the application's main loop.
 * Checks for received commands, processes them, and sends responses.
 *
 * \param[in] instancePtr Pointer to DAP instance
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Process(Dap_InstanceType *instancePtr);

/**
 * \brief Send sensor data to host
 *
 * Sends sensor data frame when streaming is active. The data is formatted
 * according to the DAP protocol and transmitted to the host.
 *
 * \param[in] instancePtr Pointer to DAP instance
 * \param[in] dataPtr     Pointer to sensor data
 * \param[in] sizeInBytes Size of data in bytes
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_SendSensorData(Dap_InstanceType *instancePtr,
                          const uint8 *dataPtr,
                          uint32 sizeInBytes);

/**
 * \brief Start continuous sensor streaming
 *
 * Initializes the streaming context, resets sequence numbers, and sends the
 * stream header. After calling this function, use Dap_StreamSensorSample()
 * to send individual sensor samples without per-packet framing.
 *
 * \param[in] instancePtr         Pointer to DAP instance
 * \param[in] sampleSizesBytesPtr Array of sample sizes in bytes per sensor (indexed by sensor slot)
 * \param[in] sampleSizesCount    Number of elements in sampleSizesBytesPtr array
 * \param[in] totalSamples        Total number of samples to stream
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_StartSensorStream(Dap_InstanceType *instancePtr, const uint16 *sampleSizesBytesPtr, uint32 sampleSizesCount,
                             uint32 totalSamples);

/**
 * \brief Stream a single sensor sample
 *
 * Sends a sensor sample in continuous streaming mode. The sample is sent without per-packet framing.
 * Sequence header is sent per-sample if DAP_INTERFACE_USE_SEQUENCE_HEADERS is enabled.
 * Must call Dap_StartSensorStream() first.
 *
 * \param[in] instancePtr  Pointer to DAP instance
 * \param[in] sensorIndex  Index into pipeline sensor configuration (0-based)
 * \param[in] dataPtr      Pointer to sensor sample data
 * \param[in] sizeInBytes  Size of sample data in bytes
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_StreamSensorSample(Dap_InstanceType *instancePtr, uint8 sensorIndex, const uint8 *dataPtr,
                              uint32 sizeInBytes);

/**
 * \brief Stop continuous sensor streaming
 *
 * Sends the stream end byte and resets the streaming context.
 *
 * \param[in] instancePtr Pointer to DAP instance
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_StopSensorStream(Dap_InstanceType *instancePtr);

/**
 * \brief Check if streaming is active
 *
 * \param[in]  instancePtr    Pointer to DAP instance
 * \param[out] isStreamingPtr Pointer to store streaming status
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_IsStreaming(const Dap_InstanceType *instancePtr, boolean *isStreamingPtr);

/**
 * \brief Check if command frame is ready to process
 *
 * \param[in]  instancePtr    Pointer to DAP instance
 * \param[out] isFrameReadyPtr Pointer to store frame ready status
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_IsFrameReady(const Dap_InstanceType *instancePtr, boolean *isFrameReadyPtr);

/**
 * \brief Get current pipeline configuration
 *
 * \param[in]  instancePtr Pointer to DAP instance
 * \param[out] configPtr   Pointer to store pipeline configuration
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_GetPipelineConfig(const Dap_InstanceType *instancePtr, Dap_PipelineConfigType *configPtr);

/**
 * \brief De-initialize DAP instance
 *
 * Stops any active streaming, de-initializes all layers, and releases
 * resources.
 *
 * \param[in] instancePtr Pointer to DAP instance
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_DeInit(Dap_InstanceType *instancePtr);

/* ========================================================================== */
/*                 UART Callback Functions                                    */
/* ========================================================================== */

/**
 * \brief UART receive completion callback
 *
 * This function must be invoked in UART HAL RX complete callback.
 * It continues the DAP frame reception state machine by calling Dap_Link_ContinueReceive.
 *
 * \param[in] instancePtr     Pointer to DAP instance
 * \param[in] bytesReceived   Number of bytes received
 *
 * \note This function is called from interrupt context.
 */
void Dap_ReceiveCallback(Dap_InstanceType *instancePtr, uint32 bytesReceived);

/**
 * \brief UART transmit completion callback
 *
 * This function must be invoked in UART HAL TX complete callback.
 * It updates the DAP link layer transmit state by calling Dap_Link_ContinueTransmit.
 *
 * \param[in] instancePtr Pointer to DAP instance
 *
 * \note This function is called from interrupt context.
 */
void Dap_TransmitCallback(Dap_InstanceType *instancePtr);

#ifdef __cplusplus
}
#endif

#endif /* DAP_H */

/** @} */