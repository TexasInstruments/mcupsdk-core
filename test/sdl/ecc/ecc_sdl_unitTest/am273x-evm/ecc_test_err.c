/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
 *  \file     ecc_test_err.c
 *
 *  \brief    This file contains ECC Error module test code for R5 core.
 *
 *  \details  ECC Error module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include <sdl/include/am273x/sdlr_soc_baseaddress.h>
#include <sdl/include/am273x/sdlr_soc_ecc_aggr.h>
#include "ecc_test_main.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t ECC_errNegativeTest(void)
{
    int32_t result;
    int32_t retVal = 0;
    DebugP_log("\n Local Variables Init: Done");
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    DebugP_log("\n Local injectErrorConfig Variable Init: Done");
    SDL_ECC_staticRegs staticRegs;

    DebugP_log("\n ECC negative tests: starting");

    /* Negative tests with invalid mem type 133U */
    result = SDL_ECC_initMemory(133U,
                                0U);
    if (result == SDL_PASS) {
        DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
        retVal = -1;
    }

    if (retVal == 0) {
        /* Negative tests with invalid mem type 133U */
        result = SDL_ECC_getStaticRegisters(133U, &staticRegs);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Negative tests with invalid pointer */
        result = SDL_ECC_getStaticRegisters(SDL_MSS_ECC_AGG_MSS, NULL);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        result = SDL_ECC_getStaticRegisters (1000U, &staticRegs);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        result = SDL_ECC_getStaticRegisters (10U, NULL);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Negative tests with invalid pointer */
        result = SDL_ECC_getErrorInfo(5U, 1U, NULL);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Negative tests with invalid pointer */
        SDL_ECC_ErrorInfo_t eccErrorInfo;
        result = SDL_ECC_getErrorInfo(5U, 0U, &eccErrorInfo);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Negative tests with invalid pointer */
        SDL_ECC_ErrorInfo_t eccErrorInfo;
        result = SDL_ECC_getErrorInfo(121U, 1U, &eccErrorInfo);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Negative tests with invalid pointer */
        result = SDL_ECC_init(0U, NULL);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_initMemory(SDL_R5FSS0_CORE0_ECC_AGGR,
                                    50U);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Not actual negative tests with valid mem subtype 0U */
        result = SDL_ECC_initMemory(SDL_R5FSS0_CORE0_ECC_AGGR, 0U);
        if (result != SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_selfTest(SDL_R5FSS0_CORE0_ECC_AGGR,
                                  SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                  SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                  NULL,
                                  1000);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                  SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                  NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                  SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                  &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Pass invalid error configuration. No bits set */
        injectErrorConfig.flipBitMask = 0x00;
        /* Set Error address */
        injectErrorConfig.pErrMem = (uint32_t *)(0x100);
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_selfTest(SDL_R5FSS0_CORE0_ECC_AGGR,
                                  50U,
                                  SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                  &injectErrorConfig,
                                  1000);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        /* Negative tests with invalid pointer */
        result = SDL_ECC_getStaticRegisters(SDL_R5FSS0_CORE0_ECC_AGGR, NULL);
        if (result == SDL_PASS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDIRTY_RAM_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM0_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM1_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM2_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM3_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM4_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM5_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM6_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }

    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x101;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                     SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DDATA_RAM7_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                    SDL_MSS_ECC_AGG_MSS_MSS_MBOX_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                    SDL_MSS_ECC_AGG_MSS_MSS_GPADC_DATA_RAM_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                    SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A0_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                    SDL_MSS_ECC_AGG_MSS_MSS_TPTC_A1_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                     SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_MSS_ECC_AGG_MSS,
                                    SDL_MSS_ECC_AGG_MSS_MSS_TPTC_B0_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
	if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
	if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
	if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
	if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
	if (retVal == 0) {
        /* Negative tests with invalid mem subtype 50U */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     NULL);
        if (result != SDL_EBADARGS) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
        SDL_ECC_InjectErrorConfig_t injectErrorConfig;
        injectErrorConfig.flipBitMask = 0x0;
        /* Negative tests without mask bit */
        result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                     SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID,
                                     SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                     &injectErrorConfig);
        if (result != SDL_EFAIL) {
            DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
            retVal = -1;
        }
    }
    if (retVal == 0) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       injectErrorConfig.flipBitMask = 0x101;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                    SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID,
                                    SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\n  Negative test failed on line no: %d \n", __LINE__);
           retVal = -1;
       }
    }
   	if (retVal == 0U) {
       SDL_ECC_InjectErrorConfig_t injectErrorConfig;
       /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 2 bit error */
       /* Note the address is relative to start of ram */
       injectErrorConfig.pErrMem = (uint32_t *)(0x00000510u);
       injectErrorConfig.flipBitMask = 0x0u;
       /* Negative tests without mask bit */
       result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                    SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                    15U,
                                    &injectErrorConfig);
       if (result != SDL_EFAIL) {
           DebugP_log("\r\n  Negative test failed on line no: %d \r\n", __LINE__);
           retVal = -1;
       }
    }
    if (retVal == 0U)
    {
        SDL_ECC_MemType *eccMemType = (SDL_ECC_MemType* )10U;
        SDL_Ecc_AggrIntrSrc *intrSrcType = (SDL_ECC_MemType* )12U;
        /* Negative tests  */
        result = SDL_ECC_getESMErrorInfo(2U, 12U, eccMemType, intrSrcType);
        if (result == SDL_PASS) {
            DebugP_log("\r\n  Negative test failed on line no: %d \r\n", __LINE__);
            retVal = -1;
        }
    }
	if (retVal == 0U)
	{
        /* Negative tests  */
        result = SDL_ECC_getESMErrorInfo(2U, 12U, NULL, 0U);
        if (result == SDL_PASS) {
            DebugP_log("\r\n  Negative test failed on line no: %d \r\n", __LINE__);
            retVal = -1;
        }
	}
	if (retVal == 0U)
	{
        /* Negative tests  */
        result = SDL_ECC_getESMErrorInfo(5U, 12U, NULL, 0U);
        if (result == SDL_PASS) {
            DebugP_log("\r\n  Negative test failed on line no: %d \r\n", __LINE__);
            retVal = -1;
        }
	}
	if (retVal == 0U)
	{
        /* Negative tests  */
        SDL_ECC_initEsm(4U);
	}
	if ( retVal == 0) {
        DebugP_log("\n ECC negative tests: success");
    } else {
        DebugP_log("\n ECC negative tests: failed");
    }

    return retVal;
}

/* ECC Error module test */
int32_t ECC_errTest(void)
{
    int32_t testResult;
    DebugP_log("\n ECC negative tests started:");
    testResult = ECC_errNegativeTest();

    return (testResult);
}

/* Nothing past this point */