/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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

#ifndef IPC_RPMSG_H_
#define IPC_RPMSG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/ipc_rpmsg/include/ipc_rpmsg_linux_resource_table.h>

/**
 * \defgroup DRV_IPC_RPMESSAGE_MODULE  APIs for IPC RPMessage
 * \ingroup DRV_MODULE
 *
 * See \ref DRIVERS_IPC_RPMESSAGE_PAGE for more details.
 *
 * @{
 */

/**
 * \brief Users can create \ref RPMessage_Object with local end value in the range of 0 .. \ref RPMESSAGE_MAX_LOCAL_ENDPT - 1
 *
 * This limit is put to allow fast indexing of end point object to
 * recevied messages and the value is kept small to save memory space
 */
#define RPMESSAGE_MAX_LOCAL_ENDPT       (64U)

/**
 * \brief Value to indicate if vring ID is valid or NOT
 *
 * When a core does not participate in IPC RPMessage set the vring ID in
 * \ref RPMessage_Params to this value
 */
#define RPMESSAGE_VRING_ADDR_INVALID      (0xFFFFFFFFU)

/**
 * \brief Returns the size needed for each VRING
 *
 * This is approximate size that is >= to actual required size.
 * Approximate size is returned to allow compile time macro for size calculation.
 * Actual required size is calculated in the internal run-time function `RPMessage_vringGetSize`. However this can
 * be ignored by end users. During initialization, a assert is used to check if user provided size is >=
 * actual required size.
 *
 * \param numBuf [in] Number of buffer in the VRING
 * \param bufSize [in] Size of each buffer, in units of bytes
 *
 * \return VRING size in bytes
 */
#define RPMESSAGE_VRING_SIZE(numBuf, bufSize)       (((numBuf)*(uint16_t)((bufSize)+32U))+32U)

/**
 * \brief Size of \ref RPMessage_Object
 */
#define RPMESSAGE_OBJECT_SIZE_MAX    (192U)

/**
 * \brief CRC size in bytes for IPC RPMsg.
 */
#define RPMESSAGE_CRC_SIZE           (2U)

/**
 * \brief Opaque RPMessage object used with the RPMessage APIs
 */
typedef struct RPMessage_Object_s {
    uintptr_t rsv[RPMESSAGE_OBJECT_SIZE_MAX/sizeof(uint32_t)]; /**< reserved, should NOT be modified by end users */
} RPMessage_Object;

/**
 * \brief Callback that is invoked when a message is received from any CPU at the specified local end point
 *
 * The callback can be optionally registered during \ref RPMessage_construct
 *
 * \note All message contents MUST be consumed in the callback.
 *       When callback returns the message buffer is released back to the sender.
 *       If the message contents are needed for deferred processing then take a copy of the message contents
 *
 * \param obj   [in] RPMessage end point object created with \ref RPMessage_construct
 * \param arg  [in] Arguments specified by user during \ref RPMessage_construct
 * \param data  [in] Pointer to message
 * \param dataLen [in] Length of message
 * \param crcStatus     [in] CRC Check status. SystemP_SUCCESS on success, else SystemP_FAILURE.
 * \param remoteCoreId [in] Core ID of sender
 * \param remoteEndPt [in] End point of sender
 */
typedef void (*RPMessage_RecvCallback)(RPMessage_Object *obj, void *arg,
    void *data, uint16_t dataLen, int32_t crcStatus,
    uint16_t remoteCoreId, uint16_t remoteEndPt);

/**
 * \brief Callback that is invoked when a message is received from any CPU at the specified local end point
 *
 * The callback can be optionally registered during \ref RPMessage_construct
 *
 * \note Unlike \ref RPMessage_RecvCallback, this callback only notifies that there is one or more messages to be read, but the
 *       message itself is not read by the driver unless \ref RPMessage_recv is called in this callback or later within a task.
 *
 * \note If \ref RPMessage_RecvCallback is set, then \ref RPMessage_RecvNotifyCallback callback is not used.
 *
 * \param obj   [in] RPMessage end point object created with \ref RPMessage_construct
 * \param arg  [in] Arguments specified by user during \ref RPMessage_construct
 */
typedef void (*RPMessage_RecvNotifyCallback)(RPMessage_Object *obj, void *arg);


/**
 * \brief Callback that is invoked when a annoucement message is received on the control end point
 *
 * The callback can be optionally registered during \ref RPMessage_init
 *
 * \note All message contents MUST be consumed in the callback.
 *       When callback returns the message buffer is released back to the sender.
 *       If the message contents are needed for deferred processing then take a copy of the message contents
 *
 * \param arg  [in] Arguments specified by user during \ref RPMessage_init
 * \param remoteCoreId [in] Core ID of sender
 * \param remoteEndPt [in] End point of sender that has annoucned the service over the control end point
 * \param remoteServiceName [in] Name of the remote service that is annoucned
 */
typedef void (*RPMessage_ControlEndPtCallback)(void *arg,
    uint16_t remoteCoreId, uint16_t remoteEndPt, const char *remoteServiceName);

/**
 * \brief This is the CRC Hook function to be defined in application for CRC Calculation.
 *
 * \note IPC Notify uses 16 Bit CRC and driver invokes this API with crcSize = 2 and passes a 16 bit variable as crc pointer.
 * \param data  [in] Pointer to data.
 * \param dataLen [in] Length of message in bytes.
 * \param crcSize [in] Size of CRC to be calculated in bytes.
 * \param crc [out] Pointer to the calculated CRC value.
 */
typedef int32_t (*RPMessage_CrcHookFxn)(uint8_t *data, uint16_t dataLen, uint8_t crcSize, void *crc);

/**
 * \brief Parameters passed to \ref RPMessage_construct
 *
 * It is recommended to set defaults using \ref RPMessage_CreateParams_init
 * and then override with user specified parameters.
 *
 * \note When `recvCallback` is enabled, \ref RPMessage_recv API should not be used and
 *       will always return failure. In this case, the received message is read and made available in the callback itself
 *       and the message needs to be consumed in the callback itself.
 *
 * \note When `recvNotifyCallback` is enabled, \ref RPMessage_recv API still needs to used to read the message.
 *       The callback just informs there are one or more messages are pending.
 */
typedef struct
{
    uint16_t localEndPt;    /**< local end point at which to listen for received mesages */
    RPMessage_RecvCallback recvCallback;    /**< Optional callback to invoke when a message is received at this end point */
    void *recvCallbackArgs; /**< Arguments to pass to the `recvCallback` callback */
    RPMessage_RecvNotifyCallback recvNotifyCallback;    /**< Optional callback to notify user when a message is received at this end point. If `recvCallback` is set, then this callback is not used */
    void *recvNotifyCallbackArgs; /**< Arguments to pass to the `recvNotifyCallback` callback */
} RPMessage_CreateParams;

/**
 * \brief Parameters passed to \ref RPMessage_init, these are generated via SysCfg
 *
 * \note Set vringTxBaseAddr[], vringTxBaseAddr[] to RPMESSAGE_VRING_ADDR_INVALID for cores that
 *       are not needed for IPC RPMessage. You can use \ref RPMessage_Params_init to set valid defaults.
 * \note All cores MUST set the same value for `vringSize`, `vringNumBuf`, `vringMsgSize`.
 * \note Set `vringTxBaseAddr`, `vringRxBaseAddr`, based on source and destination cores.
 * \note VRING memory is shared across all cores AND this memory MUST be marked
 *       as non-cached at all the cores.
 */
typedef struct
{
    uintptr_t vringTxBaseAddr[CSL_CORE_ID_MAX];  /**< VRING address of transmit rings to each core */
    uintptr_t vringRxBaseAddr[CSL_CORE_ID_MAX];  /**< VRING address of receive rings to each core */
    uint32_t vringSize;                   /**< Size of memory assigned to one VRING, use \ref RPMESSAGE_VRING_SIZE to find the size needed */
    uint16_t vringNumBuf;                 /**< Max number of buffers in one VRING */
    uint16_t vringMsgSize;                /**< Size of each message in one VRING */
    const RPMessage_ResourceTable *linuxResourceTable;  /**< Linux resoruce table for self core,
                                                   * when non-NULL Cortex A* is assumed to run Linux.
                                                   * And VRING info  for message exchange with LInux
                                                   * is specified in the resource table
                                                   */
    uint16_t linuxCoreId; /** ID of linux core */
    uint8_t  isCrcEnabled; /* CRC Enable/Disable flag */
    RPMessage_CrcHookFxn crcHookFxn; /* Hook Function to be provided by application for CRC calculation */
} RPMessage_Params;

/**
 * \brief Set default values to \ref RPMessage_Params
 *
 * \param params [out] default intialized structure
 */
void     RPMessage_Params_init(RPMessage_Params *params);

/**
 * \brief Set default values to \ref RPMessage_CreateParams
 *
 * \param params [out] default intialized structure
 */
void     RPMessage_CreateParams_init(RPMessage_CreateParams *params);

/**
 * \brief Initialize RPMessage module
 *
 * \param params [in] Initialization parameters
 */
int32_t  RPMessage_init(const RPMessage_Params *params);

/**
 * \brief De-Initialize RPMessage module
 */
void     RPMessage_deInit(void);

/**
 * \brief Wait for linux side RPMessage to be ready
 *
 * Messages should not be sent to Linux, if enabled, until this function
 * return success.
 *
 * \note When using RPMessage between RTOS/no-RTOS cores then this API
 *       needed not be called.
 *
 * \param timeout [in] Timeout in units of system ticks
 *
 * \return SystemP_SUCCESS, linux has initialized its side of RPMessage
 */
int32_t  RPMessage_waitForLinuxReady(uint32_t timeout);

/**
 * \brief Callback to call when a control message is received on a control end point
 *
 * \param controlEndPtCallback [in] User callback to invoke
 * \param controlEndPtCallbackArgs [in] Arguments pass to the user control end point callback
 */
void RPMessage_controlEndPtCallback(RPMessage_ControlEndPtCallback controlEndPtCallback,
    void  *controlEndPtCallbackArgs);

/**
 * \brief Create a RPMessage object to receive messages at a specified end-point
 *
 * \note Each new object that is created MUST have a unique local end point.
 * \note Local end point MUST be < RPMESSAGE_MAX_LOCAL_ENDPT
 * \note User MUST choose a value and `ANY` is not supported
 * \note When callback is registered in \ref RPMessage_CreateParams, \ref RPMessage_recv MUST not be used.
 *
 * \param obj [out] Created object
 * \param createParams [in] parameters
 *
 * \return SystemP_SUCCESS on success, else failure.
 */
int32_t  RPMessage_construct(RPMessage_Object *obj, const RPMessage_CreateParams *createParams);

/**
 * \brief Delete a previously created RPMessage object
 *
 * \param obj [in] object
 */
void     RPMessage_destruct(RPMessage_Object *obj);

/**
 * \brief Unblocks \ref RPMessage_recv, for the input object,
 *        if it is blocked waiting on messages and users want to exit that task
 *
 * \param obj [in] object
 */
void     RPMessage_unblock(RPMessage_Object *obj);

/**
 * \brief Return local end point of a \ref RPMessage_Object
 *
 * The value will be same as that was used to create the object earlier.
 *
 * \param obj [in] object
 *
 * \return local end point of input object
 */
uint16_t RPMessage_getLocalEndPt(const RPMessage_Object *obj);

/**
 * \brief Announce a local end point at which a `service` is created to a remote core
 *
 * \note Announcing end points is optional and is not used internally by IPC RPmessage in
 *       any way.
 * \note User MUST announce one by one to all remote core's of interest.
 *       There is no announce to `ALL` option
 * \note To handle announcement messages, make sure user handler is registered during
 *       \ref RPMessage_init via \ref RPMessage_Params
 * \note It is upto the end user to use the callback to signal or wait until a remote service is
 *       announced.
 *
 * \param remoteProcId [in] The remote core to annouce to.
 * \param localEndPt [in] Local end point of the service that is being announced
 * \param name  [in] Name of the service that is being announced
 *
 * \return SystemP_SUCCESS, when the annouce message was sent, else failure
 */
int32_t  RPMessage_announce(uint16_t remoteProcId, uint16_t localEndPt, const char* name);

/**
 * \brief Send a message to a remote core at a specified remote end point
 *
 * \note `dataLen` MUST be <= \ref RPMessage_Params.vringMsgSize - 16 bytes for internal header
 * \note In order for a remote core to receive the message,
 *       a end point should be created on the remote core at the same value as `remoteEndPt`
 * \note localEndPt, is strictly not needed, however this is available to the user on the remote core
 *       and can be used as a reply end point. Use \ref RPMessage_getLocalEndPt
 *       to set to the RPMessage object at which to listen for replies
 * \note When timeout is 0, then if a free buffer is not available to transmit, it will
 *       return will immediately SystemP_TIMEOUT. Else it will wait for specified `timeout`
 *       ticks for a free buffer to be available.
 *
 * \param data [in] Pointer to message data to send
 * \param dataLen [in] size of message data to send
 * \param remoteCoreId  [in] Remote core ID to whom the message is being sent
 * \param remoteEndPt  [in] Remote core end point ID to which the message is being sent
 * \param localEndPt [in] Local end point that is sending the message
 * \param timeout   [in] Amount of time to wait, in units of system ticks
 *
 * \return SystemP_SUCCESS, when the send message was successful
 * \return SystemP_TIMEOUT, message not sent since free transmit buffer not available and timeout happened.
 */
int32_t  RPMessage_send( void   *data,
                        uint16_t dataLen,
                        uint16_t remoteCoreId,
                        uint16_t remoteEndPt,
                        uint16_t localEndPt,
                        uint32_t timeout
                      );

/**
 * \brief Blocking API to wait till a message is received from any CPU at the specified local end point
 *
 * \note Local end point is specified during \ref RPMessage_construct
 * \note When callback is registered this API should not be used.
 * \note `dataLen` when passed by user contains the user message buffer size, i.e size of buffer pointer to by 'data`.
 *       If received message size exceeds *dataLen then it is truncated.
 *       If received message size is <= *dataLen then all received bytes are copied to `data` and
 *       *dataLen indicates the size of valid bytes in `data`
 *
 * \param obj   [in] RPMessage end point object created with \ref RPMessage_construct
 * \param data  [in] Pointer to received message contents
 * \param dataLen [in] Length of user message buffer, in bytes \n [out] Size of received message, in bytes
 * \param remoteCoreId [out] Core ID of sender
 * \param remoteEndPt [out] End point of sender
 * \param timeout   [in] Time in system ticks to block for a message receive
 *
 * \return SystemP_SUCCESS, new message received, all output parameters are valid
 * \return SystemP_TIMEOUT, API unblocked due to timeout and output parameters should not be used.
 */
int32_t  RPMessage_recv(RPMessage_Object *obj, void* data, uint16_t *dataLen,
                      uint16_t *remoteCoreId, uint16_t *remoteEndPt, uint32_t timeout);


/** @} */

#ifdef __cplusplus
}
#endif

#endif /* IPC_RPMSG_H_ */

