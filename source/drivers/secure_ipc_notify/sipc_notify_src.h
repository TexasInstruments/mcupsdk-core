/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#ifndef SIPC_NOTIFY__H_
#define SIPC_NOTIFY__H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/sipc_notify.h>
#include <drivers/secure_ipc_notify/soc/sipc_notify_soc.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CpuIdP.h>
#include <drivers/secure_ipc_notify/sipc_notify_mailbox.h>

/**
 * @brief This structure describes the information related to one interrupt that is
 *        setup for receiving mailbox messages
 * One interrupt can be used to handle messages from multiple cores.
 */
typedef struct SIPC_InterruptConfig_s
{
    uint32_t intNum;    /**< interrupt number */
    uint32_t eventId;   /**< interrupt event ID, not used for ARM cores */
    HwiP_Object hwiObj; /**< HW interrupt object handle */
    uint8_t numCores;  /**< Number of remote cores attached to this interrupt. */

    uint8_t coreIdList[MAX_SEC_CORES_WITH_HSM]; /**< List of secure cores attached to this interrupt
                                           *
                                           *   See @ref SIPC_SecCoreId for valid values of this field.
                                           */
    uint32_t clearIntOnInit;            /* 0: do not clear pending interrupts during SIPC_init, 1: clear pending interrupts */
} SIPC_InterruptConfig;

/**
 * @brief This structure describes the mailbox information to send a message from core A to core B
 */
typedef struct SIPC_MailboxConfig_s
{
    uint32_t writeDoneMailboxBaseAddr;   /**< Mailbox register address at which core will post interrupt */
    uint32_t readReqMailboxBaseAddr;   /**< Mailbox register address at which core will receive interrupt */
    uint8_t wrIntrBitPos;            /**< Bit pos in the mailbox register which should be set to post the interrupt to other core */
    uint8_t rdIntrBitPos;            /**< Bit pos in the mailbox register which should cleared clear an interrupt by the other core */
    SIPC_SwQueue *swQ;      /**< Infomration about the SW queue associated with this HW mailbox */

} SIPC_MailboxConfig;

/**
 * @brief Global structure that is pre-defined for this SOC to configure any R5 CPU to HSM mailbox communication
 *
 * This is a pre-defined global since this config typically does not need to change based
 * on end user use-cases for this SOC.
 */
extern SIPC_MailboxConfig gSIPC_R5MboxConfig[CORE_ID_MAX - 1] ;
/**
 * @brief Global structure that is pre-defined for this SOC to configure any R5 CPU to HSM mailbox communication
 *
 * This is a pre-defined global since this config typically does not need to change based
 * on end user use-cases for this SOC.
 */
extern SIPC_MailboxConfig gSIPC_HsmMboxConfig[CORE_ID_MAX - 1] ;

/**
 * @brief Global structure holding R5 to HSM queues addresses indexed by sec core id
 *
 * This is a pre-defined global since this config typically does not need to change based
 * on end user use-cases for this SOC.
 */
extern SIPC_SwQueue* gSIPC_QueR5ToHsm [MAX_SEC_CORES_WITH_HSM - 1];
/**
 * @brief Global structure holding HSM -> R5 queues addresses indexed by sec core id
 *
 * This is a pre-defined global since this config typically does not need to change based
 * on end user use-cases for this SOC.
 */
extern SIPC_SwQueue* gSIPC_QueHsmToR5[MAX_SEC_CORES_WITH_HSM - 1];

#ifdef __cplusplus
}
#endif

#endif /* IPC_NOTIFY_V0_H_ */

