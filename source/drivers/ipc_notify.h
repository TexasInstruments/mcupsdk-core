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

#ifndef IPC_NOTIFY_H_
#define IPC_NOTIFY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr_soc.h>

/**
 * \defgroup DRV_IPC_NOTIFY_MODULE  APIs for IPC Notify
 * \ingroup DRV_MODULE
 *
 * See \ref DRIVERS_IPC_NOTIFY_PAGE for more details.
 *
 * @{
 */

/**
 * \brief Maximum number of clients possible for receiving messages.
 */
#define IPC_NOTIFY_CLIENT_ID_MAX    (16U)

/**
 * \brief Client ID used by rpmessage, this client ID should not be used by other users
 */
#define IPC_NOTIFY_CLIENT_ID_RPMSG  (0U)

/**
 * \brief Client ID used for sync messages, this client ID should not be used by other users
 */
#define IPC_NOTIFY_CLIENT_ID_SYNC  (1U)

/**
 * \brief Default Interrupt priority for IPC Notify.
 */
#define IPC_NOTIFY_DEFAULT_INTR_PRIORITY  (4U)

/**
 * \brief Maximum value of message that can be sent and received.
 */
#define IPC_NOTIFY_MSG_VALUE_MAX    (0x10000000U)

/**
 * \brief Maximum value of message that can be sent and received when CRC is enabled.
 */
#define IPC_NOTIFY_CRC_MSG_VALUE_MAX    (0x100000U)

/**
 * \brief Maximum value of CRC that can be sent and received.
 */
#define IPC_NOTIFY_CRC_MAX    (0x100U)

/**
 * \brief CRC size in bytes for IPC Notify.
 */
#define IPC_NOTIFY_CRC_SIZE    (1U)

/**
 * \brief Data size in bytes for IPC Notify.
 */
#define IPC_NOTIFY_CRC_DATASIZE    (4U)

/**
 * \brief Interrupt router enable for IPC.
 */
#define IPC_INTR_ROUTER_ENABLE    (0x01U)

/**
 * \brief Interrupt router disable for IPC.
 */
#define IPC_INTR_ROUTER_DISABLE    (0x00U)

/**
 * \brief User callback that is invoked when a message is received from a reote core for a given client ID
 *
 * Before invoking the API, the IPC module would have 'popped` the message from the HW or SW FIFO already.
 * This callback is called frm ISR context, so all constraints of ISR should be applied by the callback,
 * e.g. no blocking wait, quickly handle message and exit ISR.
 *
 * For most applications, it is recommended to put the message value into a local SW queue and defer the
 * message handling itself to a application task.
 *
 * \param remoteCoreId  [in] Remote core that has sent the message
 * \param localClientId [in] Local client ID to which the message is sent
 * \param msgValue      [in] Message value that is sent
 * \param crcStatus     [in] CRC Check status. SystemP_SUCCESS on success, else SystemP_FAILURE.
 * \param args          [in] Argument pointer passed by user when \ref IpcNotify_registerClient is called
 */
typedef void (*IpcNotify_FxnCallback)(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args);

/**
 * \brief This is a driver library internal API and is used in certain SOCs by the separate mailbox driver
 *
 * \attention This API should not be used by end users.
 *
 * \param remoteCoreId [in] remote core ID that generated the interrupt
 */
typedef void (*IpcNotify_NonNotifyCallback)(uint32_t remoteCoreId);

/**
 * \brief This is the API defined in application for CRC Calculation.
 *
 * \note IPC Notify uses 8 Bit CRC and driver invokes this API with crcSize = 1 and passes an 8 bit variable as crc pointer.
 * 
 * \param data  [in] Pointer to data.
 * \param dataLen [in] Length of message in bytes.
 * \param crcSize [in] Size of CRC to be calculated in bytes.
 * \param crc [out] Pointer to the calculated CRC value.
 */
typedef int32_t (*IpcNotify_CrcHookFxn)(uint8_t *data, uint16_t dataLen, uint8_t crcSize, void *crc);

/**
 * \brief Parameters used by \ref IpcNotify_init
 *
 * Recommend to call \ref IpcNotify_Params_init before setting values to this structure
 *
 * \note This structure and call to \ref IpcNotify_init would be generated by SysConfig.
 */
typedef struct IpcNotify_Params_ {

    uint32_t numCores;  /**< Number of remote cores participating in IPC, excluding the core on
                         *  which this API is called.
                         */
    uint32_t coreIdList[CSL_CORE_ID_MAX]; /**< List of Core ID's participating in IPC, excluding the core on
                                           *   which this API is called.
                                           *
                                           *   See \ref CSL_CoreID for valid values for this field.
                                           */
    uint32_t selfCoreId; /**< Core ID of the core calling this API
                          *
                          * See \ref CSL_CoreID for valid values for this field.
                          */
    uint32_t linuxCoreId; /**< When linux IPC is enabled, this is the core ID of linux */
    uint8_t  intrPriority; /**< Interrupt priority */
    uint8_t  isCrcEnabled; /* CRC Enable/Disable flag */
    IpcNotify_CrcHookFxn crcHookFxn; /* Hook Function to be provided by application for CRC calculation. */
    uint8_t  isMailboxIpcEnabled; /**< This is used to check if Mailbox IPC is enabled*/
    uint8_t  isIPCIntrRouter; /** < This is used to check if IPC Interrupt router is enabled */
} IpcNotify_Params;

/**
 *  \brief Set default value to \ref IpcNotify_Params
 *
 *  \param  params  [out] Default initialized structure
 */
void IpcNotify_Params_init(IpcNotify_Params *params);

/**
 * \brief Initialize IPC Notify module
 *
 * This API will initialize the HW used for IPC including registering
 * interrupts for receiving messages.
 *
 * \param params [in] Initializaion parameters
 */
int32_t IpcNotify_init(const IpcNotify_Params *params);

/**
 * \brief De-initialize IPC Notify module
 *
 * This API will de-initialize the HW used for IPC including un-registering
 * interrupts for receiving messages.
 */
void    IpcNotify_deInit(void);

/**
 * \brief Send message to a specific remote core and specific client ID on that remote core
 *
 * \note To reduce latency, error checks are avoided in this API.
 * Users need to make sure the client ID value is < \ref IPC_NOTIFY_CLIENT_ID_MAX
 * and message value is < \ref IPC_NOTIFY_MSG_VALUE_MAX
 *
 * \note This API can be called from within ISRs and is also thread-safe.
 * Internally this API disables interrupts for a short while to make the API ISR and thread safe.
 *
 * \note One cannot send messages to self,
 *       i.e remoteCoreId, cannot be same as core ID of the CPU that called this API.
 *
 * \param remoteCoreId   [in] Remote core to sent message to, see \ref CSL_CoreID for valid values.
 * \param remoteClientId [in] Remote core client ID to send message to, MUST be < \ref IPC_NOTIFY_CLIENT_ID_MAX
 * \param msgValue       [in] Message value to send, MUST be < IPC_NOTIFY_MSG_VALUE_MAX
 * \param waitForFifoNotFull [in] 1: wait for message to be inserted into HW or SW FIFO,
 *                                0: if FIFO is full, dont send message and return with error.
 *
 * \return SystemP_SUCCESS, message sent successfully
 * \return SystemP_FAILURE, message could not be sent since HW or SW FIFO for holding the message is full.
 */
int32_t IpcNotify_sendMsg(uint32_t remoteCoreId, uint16_t remoteClientId, uint32_t msgValue, uint32_t waitForFifoNotFull);

/**
 * \brief Register a callback to handle messages received from a specific remote core and for a specific local client ID
 *
 * \param localClientId [in] Client ID to which the message has been sent
 * \param msgCallback [in] Callback to invoke, if callback is already registered, error will be returned.
 * \param args [in] User arguments, that are passed back to user when the callback is invoked
 *
 * \return SystemP_SUCCESS, callback registered sucessfully
 * \return SystemP_FAILURE, callback registration failed, either remoteCoreId or localClientId is invalid or callback already registered.
 */
int32_t IpcNotify_registerClient(uint16_t localClientId, IpcNotify_FxnCallback msgCallback, void *args);


/**
 * \brief Un-register a previously registered callback
 *
 * \param localClientId [in] Client ID to which the message has been sent
 *
 * \return SystemP_SUCCESS, callback un-registered sucessfully
 * \return SystemP_FAILURE, callback un-registration failed, either remoteCoreId or localClientId is invalid
 */
int32_t IpcNotify_unregisterClient(uint16_t localClientId);

/**
 * \brief Return current core ID
 *
 * \return Core ID, see \ref CSL_CoreID for valid values.
 */
uint32_t IpcNotify_getSelfCoreId(void);

/**
 * \brief Check if a core is enabled for IPC
 *
 * \param coreId [in] Core ID, see \ref CSL_CoreID for valid values.
 *
 * \return 1: core is enabled for IPC, 0: core is not enabled for IPC
 */
uint32_t IpcNotify_isCoreEnabled(uint32_t coreId);

/**
 * \brief Send a sync message to specific core
 *
 * This API can be used to send sync message's to very specific core's
 * For most users recommend to use the more simpler IpcNotify_syncAll() API.
 *
 * \param remoteCoreId [in] Core ID, see \ref CSL_CoreID for valid values.
 *
 * \return SystemP_SUCCESS, sync message was sent successfully, else failure
 */
int32_t IpcNotify_sendSync(uint32_t remoteCoreId);

/**
 * \brief Wait for a sync message to be received from the specified core
 *
 * This API can be used to recevice sync message from very specific core's
 * For most users recommend to use the more simpler IpcNotify_syncAll() API.
 *
 * \param remoteCoreId [in] Core ID, see \ref CSL_CoreID for valid values.
 * \param timeout [in] Amount of time in units of ticks to wait
 *
 * \return SystemP_SUCCESS, sync message was recevied successfully
 * \return SystemP_TIMEOUT, sync message was NOT recevied after `timeout` ticks
 * \return SystemP_FAILURE, invalid arguments
 */
int32_t IpcNotify_waitSync(uint32_t remoteCoreId, uint32_t timeout);

/**
 * \brief Send a message to all enabled cores and wait for sync message from all enabled cores
 *
 * This API when called on all CPUs, make sure all CPUs execute upto a certain and then proceed
 * only when all other CPUs have also executed to the same point.
 *
 * This is useful esp during system init to make sure message exchange can be started only
 * after all CPUs have finished their system initialization.
 *
 * \param timeout [in] Amount of time in units of ticks to wait for the sync
 *
 * \return SystemP_SUCCESS, all sync messages recevied successfully
 * \return SystemP_TIMEOUT, some sync messages was NOT recevied after `timeout` ticks
 * \return SystemP_FAILURE, invalid arguments
 */
int32_t IpcNotify_syncAll(uint32_t timeout);


/**
 * \brief This is a driver library internal API and is used in certain SOCs by the separate mailbox driver
 *
 * \attention This API should not be used by end users.
 *
 * \param callback [in] Callback to call when a interrupt is received from a CPU which is not part of IPC Notify core list
 *
 */
void IpcNotify_registerNonNotifyCallback(IpcNotify_NonNotifyCallback callback);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* IPC_NOTIFY_H_ */

