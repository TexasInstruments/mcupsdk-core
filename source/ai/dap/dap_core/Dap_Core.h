

/**
 * \defgroup DAP_CORE DAP Core Layer
 * \ingroup DAP_AM13E2X
 *
 * DAP Core layer implements the DAP protocol logic including command parsing,
 * response building, and state machine management.
 *
 * @{
 */

#ifndef DAP_CORE_H
#define DAP_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Dap_Types.h"

/* ========================================================================== */
/*                 Public Functions                                           */
/* ========================================================================== */

/**
 * \brief Process received frame
 *
 * Validates the received frame, dispatches to the appropriate command
 * handler, and sends the response.
 *
 * \param[in] instancePtr Pointer to DAP instance
 *
 * \return DAP_CORE_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Core_ProcessFrame(Dap_InstanceType *instancePtr);

/**
 * \brief Get frame length from received data
 *
 * Parses the payload length field from the received data to determine
 * the total frame length.
 *
 * \param[in]  linkInstancePtr Pointer to DAP link instance
 * \param[in]  bufferPtr       Pointer to received data buffer
 * \param[in]  bufferLen       Length of data in buffer
 * \param[out] lengthPtr       Pointer to store frame length
 *
 * \return DAP_CORE_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Core_GetFrameLength(Dap_Link_InstanceType *linkInstancePtr,
                               const uint8 *bufferPtr,
                               uint32 bufferLen,
                               uint32 *lengthPtr);

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
 * \return DAP_ERROR_NONE on success, error code on failure
 */
sint32 Dap_Core_DecodePayloadLength(const uint8 *bufferPtr,
                                    uint32 bufferLen,
                                    uint32 *payloadLenPtr,
                                    uint32 *headerLenPtr);

/**
 * \brief Send streaming frame header
 *
 * Sends the start byte, response code, and encoded payload length for
 * a streaming data frame. Used when streaming sensor data.
 *
 * \param[in] linkInstancePtr   Pointer to DAP link instance
 * \param[in] channel           Data channel identifier (0-15)
 * \param[in] totalPayloadLen   Total payload length for this stream
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Core_SendStreamHeader(Dap_Link_InstanceType *linkInstancePtr,
                                 uint8 channel,
                                 uint32 totalPayloadLen);

/**
 * \brief Send streaming sample data
 *
 * Sends raw sample data as part of a streaming frame. Must be called
 * after Dap_Core_SendStreamHeader and before Dap_Core_SendStreamEnd.
 *
 * \param[in] linkInstancePtr Pointer to DAP link instance
 * \param[in] packetPtr       Pointer to sample data
 * \param[in] packetLen       Length of sample data in bytes
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Core_SendStreamSample(Dap_Link_InstanceType *linkInstancePtr,
                                 uint8 *packetPtr,
                                 uint32 packetLen);

/**
 * \brief Send streaming frame end byte
 *
 * Sends the end byte to complete a streaming data frame. Must be called
 * after all samples have been sent via Dap_Core_SendStreamSample.
 *
 * \param[in] linkInstancePtr Pointer to DAP link instance
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Core_SendStreamEnd(Dap_Link_InstanceType *linkInstancePtr);

/**
 * \brief De-initialize core layer
 *
 * \param[in] linkInstancePtr Pointer to DAP link instance
 *
 * \return DAP_ERROR_NONE on success, positive error code on failure
 */
sint32 Dap_Core_DeInit(Dap_Link_InstanceType *linkInstancePtr);

#ifdef __cplusplus
}
#endif

#endif /* DAP_CORE_H */

/** @} */