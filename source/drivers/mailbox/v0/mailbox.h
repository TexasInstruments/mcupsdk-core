/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
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
 *
 */
#ifndef MAILBOX_V0_H_
#define MAILBOX_V0_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr_soc.h>

/**
 * \defgroup DRV_MAILBOX_MODULE  APIs for MAILBOX
 * \ingroup DRV_MODULE
 *
 * This module needs \ref DRIVERS_IPC_NOTIFY_PAGE to be enabled and initialized before this
 * module can be used.
 *
 * See \ref DRIVERS_MAILBOX_PAGE for more details.
 *
 * @{
 */

/**
 * \brief User callback that is called when a read interrupt is received
 *
 * \param remoteCoreId [in] Remote core from where the interrupt was recevied
 * \param args [in] Arguments passed via \ref Mailbox_setReadCallback
 */
typedef void (*Mailbox_ReadCallback)(uint32_t remoteCoreId, void *args);

/**
 * \brief Inialization parameters passed to Mailbox_init()
 */
typedef struct Mailbox_Params_ {

    uint32_t rsv; /**< reserved for future use */

} Mailbox_Params;

/**
 * \brief Set default values for Mailbox_Params
 *
 * \param params [out] Initialization parameters
 */
void Mailbox_Params_init(Mailbox_Params *params);

/**
 * \brief Initialize mailbox based IPC
 *
 * - This API MUST be called after \ref IpcNotify_init is called.
 * - Recommend to set defaults using \ref Mailbox_Params_init before overriding with user required parameters.
 *
 * \param params [in] Initialization parameters
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Mailbox_init(Mailbox_Params *params);

/**
 * \brief Set callback to invoke when a read message interrupt is received
 *
 * \param readCallback [in] User callback to call when a new read interrupt is received
 * \param readCallbackArgs [in] Arguments to pass to the user callback
 */
void Mailbox_setReadCallback(Mailbox_ReadCallback readCallback, void *readCallbackArgs);

/**
 * \brief Callback which gets invoked when a read message interrupt is received
 *
 * \param remoteCoreId  [in] One of CSL_CoreID
 */
void Mailbox_readCallback(uint32_t remoteCoreId);

/**
 * \brief Checks is remote core ID supports mailbox based communication.
 *
 * \param remoteCoreId  [in] One of CSL_CoreID
 *
 * \return 1 = mailbox communication is enabled
 * \return 0 = mailbox communication is not enabled
 */
uint32_t Mailbox_isCoreEnabled(uint32_t remoteCoreId);

/**
 * \brief Write data to send to remote core in shared memory and trigger interrupt to remote core
 *
 *    When timeToWaitInTicks is > 0, the function waits until a ACK is received, else it just returns
 *    with success
 *
 * \param remoteCoreId [in] One of CSL_CoreID, remote core MUST be enabled during Mailbox_init()
 * \param buffer [in] Data to send
 * \param size [in] Size of data in bytes
 * \param timeToWaitInTicks [in] Time to wait for ACK in units of ticks
 *
 * \return SystemP_SUCCESS on success
 * \return SystemP_TIMEOUT if ACK is not received in specified timeout
 * \return SystemP_FAILURE on any other failure
 */
int32_t Mailbox_write(uint32_t remoteCoreId, uint8_t *buffer, uint32_t size, uint32_t timeToWaitInTicks);

/**
 * \brief Read data to sent from remote core in shared memory
 *
 *    The function waits until a read interrupt is received.
 *    If no read interrupt is recevied for 'timeToWaitInTicks' then \ref SystemP_TIMEOUT is returned
 *
 *    This function when called repeatedly will read from the same buffer in a incrementing manner.
 *    When all the buffer is read, call Mailbox_readDone() to indicate that read buffer is
 *    consumed and new data can be sent by other side.
 *
 * \param remoteCoreId [in] One of CSL_CoreID, remote core MUST be enabled during Mailbox_init()
 * \param buffer [in] Buffer to read into
 * \param size [in] Size of data to read, in bytes
 * \param timeToWaitInTicks [in] Time to wait for recevied interrupts.
 *
 * \return SystemP_SUCCESS on success
 * \return SystemP_TIMEOUT if read interrupt is not received in specified timeout
 * \return SystemP_FAILURE on any other failure
 */
int32_t Mailbox_read(uint32_t remoteCoreId, uint8_t *buffer, uint32_t size, uint32_t timeToWaitInTicks);

/**
 * \brief Tell remote core that all data that was sent is read and new data can be sent if needed
 *
 * \param remoteCoreId [in] One of CSL_CoreID, remote core MUST be enabled during Mailbox_init()
 *
 * \return SystemP_SUCCESS on success
 * \return SystemP_FAILURE on any other failure
 */
int32_t Mailbox_readDone(uint32_t remoteCoreId);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
