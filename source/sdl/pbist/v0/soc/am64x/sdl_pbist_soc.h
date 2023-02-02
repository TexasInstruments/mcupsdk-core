/*
 * Copyright (C) 2023 Texas Instruments Incorporated.
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

#include <sdl/include/am64x_am243x/sdlr_soc_pbist.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief PBIST instance
 *
 *  This enum defines the PBIST instances supported by the SDL_PBIST_selfTest API.
 */
typedef enum {
    /*!
     * MCU Instance (NCGO)
     */
    SDL_PBIST_INST_MCU,
    /*!
     * Pulsar Instance 0
     */
    SDL_PBIST_INST_R5F0,
    /*!
     * Pulsar Instance 1
     */
    SDL_PBIST_INST_R5F1,
    /*!
     * A53 Instance
     */
    SDL_PBIST_INST_MPU,
    /*!
     * Infrastructure Instance
     */
    SDL_PBIST_INST_INFRA,
} SDL_PBIST_inst;

#define SDL_PBIST_NUM_INSTANCES 5U

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* SDL_PBIST_SOC_H_ */
