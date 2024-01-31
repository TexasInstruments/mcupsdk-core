/*
 *  Copyright (C) 2022-24 Texas Instruments Incorporated
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

#ifndef HSM_CLIENT_MSG_H_
#define HSM_CLIENT_MSG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdlib.h>

/**
 * \defgroup DRV_HSMCLIENT_MODULE APIs for HSMCLIENT
 * \ingroup DRV_MODULE
 *
 * See \ref DRIVERS_HSMCLIENT_PAGE for more details.
 *
 * @{
 */

/** @brief GetVersion service type ID */
#define HSM_MSG_GET_VERSION                      (0x0002)
/** @brief Boot Notify service type ID */
#define HSM_MSG_BOOT_NOTIFY                      (0x000A)
/** @brief Get UID service type ID */
#define HSM_MSG_GET_UID                          (0x9021)
/** @brief Open Debug Firewalls service type ID */
#define HSM_MSG_OPEN_DBG_FIREWALLS               (0x900C)
/** @brief Read Extended otp row type ID */
#define HSM_MSG_READ_OTP_ROW                     (0x9022)
/** @brief Write Extended otp row type ID */
#define HSM_MSG_WRITE_OTP_ROW                    (0x9023)
/** @brief Protect Extended otp row type ID */
#define HSM_MSG_PROT_OTP_ROW                     (0x9024)
/** @brief Get Extended otp row protection type ID */
#define HSM_MSG_GET_OTP_ROW_PROT                 (0x9026)
/** @brief Get Extended otp row count service type ID */
#define HSM_MSG_GET_OTP_ROW_COUNT                (0x9027)
/** @brief Secure Boot service type ID*/
#define HSM_MSG_PROC_AUTH_BOOT					 (0xC120)
/** @brief Set Firewall service type ID*/
#define HSM_MSG_SET_FIREWALL                     (0x9000U)
/** @brief Set Firewall Interrupt service type ID*/
#define HSM_MSG_SET_FIREWALL_INTR                (0x9002)
/** @brief send KeyWriter customer key certificate */
#define HSM_KEYWRITER_SEND_CUST_KEY_CERT         (0x9028)
/** @brief Read Software Revision service type ID */
#define HSM_MSG_READ_SWREV						 (0x9033)
/** @brief Write Software Revision service type ID */
#define HSM_MSG_WRITE_SWREV						 (0x9032)
/** @brief Get DKEK service type ID*/
#define HSM_MSG_GET_DKEK                         (0x9029)
/** @brief Get RNG service type ID*/
#define HSM_MSG_GET_RAND                         (0x9001)
/** @brief Get KEYRING import service type ID*/
#define HSM_MSG_KEYRING_IMPORT                   (0x9039)
/* message flags */
/**
 * @brief
 *  HSM FLAG used by HSM client to indicate that it expects
 *  ACK messasge from HSM and will wait for a response message.
 */
#define HSM_FLAG_AOP                             (0x11)


/**
 * @brief
 *  HSM FLAG used by HSM client to indicate that it
 *  does not expects an ACK messasge from HSM and will
 *  not wait for a response message.
 */
#define HSM_FLAG_NAOP                            (0x22)

/**
 * @brief
 * HSM FLAG used by HSM server to indicate that the request has
 * been processed.
 *
 */
#define HSM_FLAG_ACK                             (0xAA)
/**
 * @brief
 * HSM FLAG used by HSM server to indicate that the request has not been processed.
 *
 */
#define HSM_FLAG_NACK                            (0x55)

/** @brief HSM server SIPC client Id */
#define HSM_CLIENT_ID                            (0x01)

/**
 * @brief
 * UID or Unique ID is a device specific ID of 64 bytes
 *
 */
#define HSM_UID_SIZE                              (64U)

/**
 * @brief
 * Maximum Certificate Size allowed for Debug Open
 *
 */
#define HSM_DBG_CERT_SIZE                         (4096U)

/**
 * @brief
 * Maximum Certificate Size allowed for Keyring Import
 *
 */
#define HSM_KEYRING_CERT_SIZE                      (10280U)

/**
 * @brief
 * HSM client / server message format struct
 *
 */
typedef struct HsmMsg_t_
{
    uint8_t     destClientId;  /*< destination client Id */
    uint8_t     srcClientId;   /*< current task client Id */
    uint8_t     flags;         /*< flags */
    uint16_t    serType ;      /*< service type */
    void*       args ;         /*< pointer to args */
    uint16_t    crcArgs;       /*< args integirty check */
    uint16_t    crcMsg ;       /*< message integrity check */
}  __attribute__((packed)) HsmMsg_t;

typedef enum HSM_ClientIds_
{
    HSM_BOOT_NOTIFY_CLIENT_ID = 0,
    HSM_CLIENT_ID_1,
    HSM_CLIENT_ID_2,
    HSM_CLIENT_ID_3,
    HSM_CLIENT_ID_4,
} HSM_ClientIds_t ;

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* HSM_CLIENT_MSG_H_ */
