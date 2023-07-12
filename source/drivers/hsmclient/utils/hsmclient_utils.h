/*
 *  Copyright (C) 2022-2023 Texas Instruments Incorporated
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
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON AN2
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HSM_CLIENT_UTILS_H_
#define HSM_CLIENT_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup DRV_HSMCLIENT_UTILS_MODULE APIs for HSMCLIENT_UTILS
 * \ingroup DRV_MODULE
 *
 * See \ref DRIVERS_HSMCLIENT_PAGE for more details.
 *
 * @{
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdlib.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** @brief SOC TYPE FLAG for AM263X */
#define SOC_TYPE_AM263X		0x1
/** @brief SOC TYPE FLAG for AM263p */
#define SOC_TYPE_AM263P		0x2
/** @brief SOC TYPE FLAG for AM273x */
#define SOC_TYPE_AM273X		0x3
/** @brief SOC TYPE FLAG for AWR294x */
#define SOC_TYPE_AWR294X	0x4
/** @brief Count of all supported SOC's */
#define NUM_SOC_TYPE		0x4

/** @brief Device type flag for HS-FS */
#define DEVICE_TYPE_HS_FS   0x00
/** @brief Device type flag for HS-SE */
#define DEVICE_TYPE_HS_SE   0xFF

/** @brief HSM Version flag */
#define HSM_V1				0x1

/** @brief Binary type flag for STANDARD */
#define BIN_TYPE_STANDARD	0x55
/** @brief Binary type flag for CUSTOM */
#define BIN_TYPE_CUSTOM		0xAA
/** @brief Binary type flag for OTPKW */
#define BIN_TYPE_OTPKW      0x33

typedef union HsmVer_t_ HsmVer_t;

/* ========================================================================== */
/*                             Function Declaration                           */
/* ========================================================================== */

/**
 * \brief Parses Version string.
 *
 * \param tifsMcuVer Pointer to HsmVer_t version union
 * \param parsedVer Pointer to parsed string
 *
 * \returns status returns System_SUCCESS on successful parsing,
 *			else System_FAILURE.
 */
int32_t HsmClient_parseVersion(HsmVer_t *tifsMcuVer, char* parsedVer);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* HSM_CLIENT_UTILS_H_ */
