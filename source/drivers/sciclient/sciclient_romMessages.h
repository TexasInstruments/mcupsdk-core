/*
 *  Copyright (C) 2017-2018 Texas Instruments Incorporated
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
/**
 * \ingroup SCICLIENT_HAL
 * \defgroup SCICLIENT_ROM_HAL Sciclient Interface to DMSC ROM
 *
 * Sciclient needs to interact with ROM to be able to load the
 * central processing firmware during boot.
 * This is called only once via the SBL executing on MCU R5 in the
 * AM6x device family.
 *
 * DMSC ROM will set up 2 outbound/inbound threads(secure proxy)
 * for Normal and High priority messages and pairs this threads
 * via Ring Accelerator in MCU NAVSS for both DMSC and R5 ROM to
 * exchange messages.
 *
 * The thread ID's for each message type is \ref Sciclient_RomThreadIds
 *
 * |        Message Type            | Thread Id
 * |:------------------------------ |:--------
 * | DMSC Out Bound Normal Prioirty | Thread 0
 * | DMSC In Bound Normal Priority  | Thread 1
 * | DMSC Out Bound High Priority   | Thread 2
 * | DMSC In Bound High Priority    | Thread 3
 * | R5 Out Bound Normal Priority   | Thread 4
 * | R5 In Bound Normal Priority    | Thread 5
 * | R5 Out Bound High Priority     | Thread 6
 * | R5 In Bound High Priority      | Thread 7
 *
 * The DMSC ROM messages handled by the #Sciclient_loadFirmware
 * API is \ref Sciclient_RomMessageType.
 *
 * @{
 */
/**
 *  \file   sciclient_romMessages.h
 *
 *  \brief  This file contains the declaration of the messaging format to talk
 *          to ROM.
 */

#ifndef SCICLIENT_ROMMESSAGES_H_
#define SCICLIENT_ROMMESSAGES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
 *  \anchor Sciclient_RomMessageType
 *  Requirement: DOX_REQ_TAG(PDK-2143)
 *  \name Sciclient load firmware ROM Message Types.
 *  @{
 */
/** Firmware Load Command ID from R5 to M3 */
#define SCICLIENT_ROM_MSG_R5_TO_M3_M3FW                             (0x8105U)
/** Firmware Load Command ID from M3 to R5 */
#define SCICLIENT_ROM_MSG_M3_TO_R5_M3FW_RESULT                      (0x8805U)
/** Firmware Load Command response Authorization Passed */
#define SCICLIENT_ROM_MSG_CERT_AUTH_PASS                            (0x555555U)
/** Firmware Load Command response Authorization Failed */
#define SCICLIENT_ROM_MSG_CERT_AUTH_FAIL                            (0xffffffU)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 *  \brief ROM messaging: Header for loading firmware. This is the same as the
 *         firmware header.
 */
typedef struct tisci_header Sciclient_RomFirmwareLoadHdr_t;

/**
 *  \brief ROM messaging: Payload for loading firmware .
 */
typedef struct
{
    uint32_t bufferAddress;
    /**< Address at which the firmware is located */
    uint32_t bufferSizeBytes;
    /**< Size of firmware.
     *    Note : This field is not used read by the ROM. Its value is taken from
     *           x509 certificate .
     */
} Sciclient_RomFirmwareLoadPayload_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_ROMMESSAGES_H_ */

/** @} */
