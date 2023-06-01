/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *   @file  sdl_soc_pbist.h
 *
 *   @brief This file contains the SoC-specific SDL PBIST API's and
 *          definitions.
 *
 *   @defgroup SDL_PBIST_MODULE APIs for PBIST (Memory Built-In Self Test)
 *
 *   Provides the APIs for PBIST.
 *
 *   @{
 */

#ifndef SDL_PBIST_SOC_H_
#define SDL_PBIST_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/soc_config.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
#if defined (SUBSYS_R5SS0)
#define SDL_INTR_PBIST_DONE SDL_R5FSS0_CORE0_INTR_PBIST_DONE
#elif defined (SUBSYS_R5SS1)
#define SDL_INTR_PBIST_DONE SDL_R5FSS1_CORE0_INTR_PBIST_DONE
#endif
/**
@defgroup SDL_PBIST_FUNCTION  PBIST Functions
@ingroup SDL_PBIST_API
*/

/**
@defgroup SDL_PBIST_ENUM PBIST Enumerated Data Types
@ingroup SDL_PBIST_API
*/

/**
@defgroup SDL_PBIST_DATASTRUCT  PBIST Data Structures
@ingroup SDL_PBIST_API
*/

/**
 *  @addtogroup SDL_PBIST_ENUM
    @{
 *
 */

/**
 *  \brief PBIST instance
 *
 *  This enum defines the PBIST instances supported by the SDL_PBIST_selfTest API.
 */
typedef enum {
    /*!
     * TOP PBIST Instance
     */
    SDL_PBIST_INST_TOP,
} SDL_PBIST_inst;

typedef enum {
    /*!
     * MCU Instance
     */
    SDL_PBIST_HWPOST_INST_MCU,
} SDL_PBIST_hwpostInst;

typedef enum {
    /*!
     * The HW POST PBIST completed and the test passed
     */
    SDL_PBIST_POST_COMPLETED_SUCCESS,
    /*!
     * The HW POST PBIST completed and the test failed
     */
    SDL_PBIST_POST_COMPLETED_FAILURE,
    /*!
     * The HW POST PBIST was run but timed out
     */
    SDL_PBIST_POST_TIMEOUT,
    /*!
     * The HW POST PBIST was not performed on this device
     */
    SDL_PBIST_POST_NOT_RUN
} SDL_PBIST_postStatus;

/** @} */

#define SDL_PBIST_NUM_INSTANCES 1U

#define SDL_PBIST_HWPOST_NUM_INSTANCES 1

/**
 *  @addtogroup SDL_PBIST_DATASTRUCT
    @{
 *
 */

typedef struct {
    /*!
     * HW POST PBIST status for MCU
     */
    SDL_PBIST_postStatus mcuPostStatus;
} SDL_PBIST_postResult;

/** @} */

/* Macro to define the PBIST execution on R5F*/
#define SDL_SOC_MCU_R5F      1U

#ifdef __cplusplus
}
#endif

#endif
 /** @} */
