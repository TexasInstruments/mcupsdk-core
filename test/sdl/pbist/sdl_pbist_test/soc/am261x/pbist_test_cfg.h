/*
 *   Copyright (c) Texas Instruments Incorporated 2020
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
 *  \file     pbist_test_cfg.h
 *
 *  \brief    This file contains PBIST test configuration
 *
 *  \details  PBIST Test configuration
 **/
#ifndef PBIST_TEST_CFG_H
#define PBIST_TEST_CFG_H

#ifdef __cplusplus
extern "C"
{
#endif
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <sdl/include/soc_config.h>
#include <pbist_test_func.h>

/* #define DEBUG */

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
extern PBIST_TestHandle_t PBIST_TestHandleArray[SDL_PBIST_NUM_INSTANCES];

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the types of possible PBIST memory groups
 *
 *
 */
typedef uint32_t SDL_TOPPBISTMemGroup;

#define SDL_PBIST_MEMGRP_MEM_MSS_R5_STC                     ((uint32_t) 1U)
#define SDL_PBIST_MEMGRP_MEM_TOP_PBISTROM                  ((uint32_t) 2U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CPSW                   ((uint32_t) 3U)
#define SDL_PBIST_MEMGRP_MEM_MSS_ICSSM             ((uint32_t) 4U)
#define SDL_PBIST_MEMGRP_MEM_MSS_MBOX                   ((uint32_t) 5U)
#define SDL_PBIST_MEMGRP_MEM_MSS_MCAN                ((uint32_t) 6U)
#define SDL_PBIST_MEMGRP_MEM_MSS_TPCC                   ((uint32_t) 7U)
#define SDL_PBIST_MEMGRP_MEM_MSS_L2_0                ((uint32_t) 8U)
#define SDL_PBIST_MEMGRP_MEM_MSS_L2_1                   ((uint32_t) 9U)
#define SDL_PBIST_MEMGRP_MEM_MSS_L2_2                   ((uint32_t) 10U)
#define SDL_PBIST_MEMGRP_MEM_MSS_L2_3                  ((uint32_t) 11U)
#define SDL_PBIST_MEMGRP_MEM_MSS_R5SS0_VIM0                ((uint32_t) 12U)
#define SDL_PBIST_MEMGRP_MEM_MSS_R5SS0_VIM1             ((uint32_t) 13U)
#define SDL_PBIST_MEMGRP_MEM_MSS_R5SS1_VIM0              ((uint32_t) 14U)
#define SDL_PBIST_MEMGRP_MEM_MSS_R5SS1_VIM1             ((uint32_t) 15U)
#define SDL_PBIST_MEMGRP_MEM_MSS_TRACE                  ((uint32_t) 16U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CR5A_ATCM0                ((uint32_t) 17U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CR5A_ATCM1               ((uint32_t) 18U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CR5A_BTCM0               ((uint32_t) 19U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CR5A_BTCM1               ((uint32_t) 20U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CR5B_ATCM0               ((uint32_t) 21U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CR5B_ATCM1               ((uint32_t) 22U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CR5B_BTCM0               ((uint32_t) 23U)
#define SDL_PBIST_MEMGRP_MEM_MSS_CR5B_BTCM1               ((uint32_t) 24U)
#define SDL_PBIST_MEMGRP_MEM_R5SS0                        ((uint32_t) 25U)
#define SDL_PBIST_MEMGRP_MEM_R5SS1                        ((uint32_t) 26U)
#define SDL_PBIST_MEMGRP_MEM_MMCH0                        ((uint32_t) 27U)

#define APP_PBIST_TEST_NEG_INST          (0x5U)
/**
 * @brief
 *  PBIST Configuration Parameters
 *
 * @details
 *  This describes the parameters which need to be passed to the PBIST diagnostic.
 */
typedef struct SDL_TOPPBIST_Cfg_s
{
    /**
     * @brief   This describes the memory group used for the PBIST self test.
     */
    SDL_TOPPBISTMemGroup      memoryGroup;

    /**
     * @brief   Inject fault flag: This flag is used to indicate if a fault needs
     * to be injected. \n
     * 0 - Run the PBIST self test for the memory group selected above. \n
     * 1 - Inject a fault so that the diagnostic can detect the generated single/multi
     * bit errors during memory self test. \n
     * Note: The error is generated by running a March13N algorithm on PBIST ROM which
     * is an invalid memory group, algorithm combination.
     * The memoryGroup configuration mentioned above is not used.
     */
    uint32_t                   injectFault;
} SDL_TOPPBIST_Cfg;


int32_t SDL_TOPPBIST_execute(const SDL_TOPPBIST_Cfg* ptrCfg, uint64_t *timeTaken);

#ifdef __cplusplus
}
#endif

#endif /* PBIST_TEST_CFG_H */
/* Nothing past this point */
