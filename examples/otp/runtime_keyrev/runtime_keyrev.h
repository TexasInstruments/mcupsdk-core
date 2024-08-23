/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

/** \file runtime_keyrev.h
 *
 *   \brief This file contains runtime software revision update APIs.
 */

#ifndef RUNTIME_KEYREV_H_
#define RUNTIME_KEYREV_H_

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

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Function to read OTP key revision and key count from the efuses
 *
 *  \param keyCountVal    [OUT] Key count value read back
 *  \param keyRevVal      [OUT] KEYREV value read back
 *
 *  \return status [out] SystemP_SUCCESS on success
 */
int32_t runtime_keyrev_readKeyrevKeycnt(uint32_t *keyCountVal, uint32_t *keyRevVal);

/**
 *  \brief Function to write OTP key revision to the efuses
 *
 *  \return status [out] SystemP_SUCCESS on success
 */
int32_t runtime_keyrev_writeKeyrev();

/**
 *  \brief Function to set VPP, needed to write to eFUSEs
 * 
 *  \param ioIndex [IN] Index to the TCA6424 IO which needs to be set 
 *  \param LED_ioIndex [IN] Index to the TCA6416 IO LED which needs to be set
 * 
 */
void runtime_keyrev_setVpp(uint32_t ioIndex, uint32_t LED_ioIndex);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef RUNTIME_KEYREV_H */
