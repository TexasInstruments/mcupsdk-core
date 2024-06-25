/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *   \file  sdl/sdl_pbist.h
 *
 *   \brief This file contains the SDL PBIST API's.
 *
 *   \defgroup SDL_PBIST_MODULE APIs for PBIST (Memory Built-In Self Test)
 *
 *   \ingroup SDL_MODULE
 *
 *   Provides the APIs for PBIST.
 *
 *   @{
 */

#ifndef SDL_PBIST_H_
#define SDL_PBIST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <sdl/include/soc_config.h>


#include <sdl/pbist/v0/soc/sdl_soc_pbist.h>


/**
 *  \brief PBIST self test type
 *
 *  This enum defines the test types supported by the SDL_PBIST_selfTest API.
 */
typedef enum {
    /*!
     * PBIST Test
     * Performs the PBIST test for a the instance specified.
     * When a valid pConfig is passed, those values will be used to execute the
     * test. If pConfig is NULL (recommended), then the default "golden vectors"
     * will be used to execute the test.
     */
    SDL_PBIST_TEST,
    /*!
     * PBIST Negative Test
     * Performs the PBIST negative test to verify the PBIST logic for the
     * instance specified
     */
    SDL_PBIST_NEG_TEST
} SDL_PBIST_testType;

/**
 *  \brief PBIST self test
 *
 *  This function executes a PBIST self test of the specified type for a
 *  specific PBIST instance. The test types supported are defined in
 *  SDL_PBIST_testType.
 *  PBIST is a feature that is used for self test of the memory regions
 *  in the SoC. Multiple instances may be supported.
 *  Note: The PBIST test is destructive to the cores/IP which are being
 *  tested. Therefore it should be executed from a different core than
 *  the one under test.
 *  It is important to note that execution of PBIST self-tests require
 *  preparation of the IPS under test by bringing them to a certain
 *  power and reset state before executing the test. The application
 *  must take care of this sequence before executing the self-test.
 *  The SDL examples give a reference for the necessary sequences.
 *  The PBIST instances supported are specific to the SoC, and are
 *  defined in the soc-specific header file.
 *
 *  \param instance         [IN]  PBIST instance
 *  \param testType         [IN]  PBIST test type to run.
 *  \param timeout          [IN]  Max number of times to check for PBIST done
 *  \param pResult          [OUT] Pointer to variable to indicate result
 *
 *  \return The SDL error code for the API.
 *                                 If pResult is NULL: SDL_EBADARGS
 *                                 If testType is invalid: SDL_EBADARGS
 *                                 If other error happened: SDL_EFAIL
 *                                 Success: SDL_PASS
 */
int32_t SDL_PBIST_selfTest(SDL_PBIST_inst instance, SDL_PBIST_testType testType,
                           uint32_t timeout, bool *pResult);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SDL_PBIST_H_ */
