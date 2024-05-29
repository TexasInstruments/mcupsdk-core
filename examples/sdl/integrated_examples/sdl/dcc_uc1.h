/*
 *  Copyright (c) 2021-2024 Texas Instruments Incorporated
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
 *  \file     main.h
 *
 *  \brief    This file contains DCC Example test code declarations
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
/*===========================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/sdl_dcc.h>
#include <sdl/esm/sdlr_esm.h>
#include <sdl/include/sdl_types.h>
#include <sdl/esm/v0/sdl_esm.h>

#if !defined(SDL_DCC_TEST_H)
#define SDL_DCC_TEST_H

/* ========================================================================== */
/*                     Dependant macros in sdl_dcc_funcTest.c                  */
/* ========================================================================== */

#define APP_DCC_MODULE_INST             (SDL_DCC_INST_MSS_DCCA)
/**< Instance of DCC. While changing the instance, ensure update clock sources*/
#define APP_DCC_MODE                    (SDL_DCC_MODE_CONTINUOUS)
/**< One Shot mode, Stop counting when Counter 1, reaches 0. */
#define APP_DCC_SRC0_MAX_VAL            (0xFFFFFU)
/**< Maximum value that can be held in the COUNT0 register (ref clock) */
#define APP_DCC_SRC0_VALID_MAX_VAL      (0x0FFFFU)
/**< Maximum value that can be held in the VALID0 register (ref clock) */
#define APP_DCC_SRC1_MAX_VAL            (0xFFFFFU)
/**< Maximum value that can be held in the COUNT1 register (test clock) */

/* Defines that control the clock inputs to DCC and allowed variance */
#define APP_DCC_REF_CLOCK_SRC_0         (SDL_DCC_CLK0_SRC_CLOCK0_0)
#define APP_DCC_TEST_CLOCK_SRC_1        (SDL_DCC_CLK1_SRC_CLOCKSRC5)

#define APP_DCC_TEST_CLOCK_SRC_1_DRIFT  (5U)
/**< Allowed drift in percentage (+/-) */
#define APP_DCC_TEST_CLOCK_SRC_1_HIGHER (SDL_DCC_CLK1_SRC_CLOCKSRC0) /* SYSCLK(200MHZ) */
/**< Clock source for Counter 1, expected to be higher than
        APP_DCC_TEST_CLOCK_SRC_1, in this example to simulate an error*/



/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct sdlDccTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} sdlDccTest_t;

typedef struct {
    char srcStr[32];
    char testStr[32];
    SDL_DCC_Inst dccInst;
    SDL_DCC_clkSrc0 clk0;
    uint32_t clk0Freq;
    SDL_DCC_clkSrc1 clk1;
    uint32_t clk1Freq;
    SDL_DCC_mode mode;
    uint32_t intNum; /* Interrupt Num used in case of single-shot mode */
    SDL_DCC_clkSrc1 src1Higher;
    uint32_t errorTest; /* indicates if an error should be expected */
} DCC_TEST_UseCase;


/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

#define SDL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED         (-(int32_t) (1))
#define SDL_APP_TEST_PASS           ( (int32_t) (0))

#define DCC_NO_INTERRUPT            (0u)
#define DCC_INTERRUPT               (1u)

/*===========================================================================*/
/*                      ESM ERROR Macros                                     */
/*===========================================================================*/
#if defined (SOC_AM273X)

#define ESM_ERROR_GROUP_1    1U
#define ESM_ERROR_GROUP_2    2U
#define ESM_ERROR_GROUP_3    3U

#define  DCCA_MSS_ESM_ERROR  SDL_ESMG1_DCCA_ERR
#define  DCCB_MSS_ESM_ERROR  SDL_ESMG1_DCCB_ERR
#define  DCCC_MSS_ESM_ERROR  SDL_ESMG1_DCCC_ERR
#define  DCCD_MSS_ESM_ERROR  SDL_ESMG1_DCCD_ERR

#define  DCCA_DSS_ESM_ERROR  SDL_DSS_ESMG2_DSS_DCCA_ERR
#define  DCCB_DSS_ESM_ERROR  SDL_DSS_ESMG2_DSS_DCCB_ERR

#elif defined (SOC_AWR294X)

#define ESM_ERROR_GROUP_1    1U
#define ESM_ERROR_GROUP_2    2U
#define ESM_ERROR_GROUP_3    3U

#define  DCCA_MSS_ESM_ERROR  SDL_ESMG1_DCCA_ERR
#define  DCCB_MSS_ESM_ERROR  SDL_ESMG1_DCCB_ERR
#define  DCCC_MSS_ESM_ERROR  SDL_ESMG1_DCCC_ERR
#define  DCCD_MSS_ESM_ERROR  SDL_ESMG1_DCCD_ERR

#define  DCCA_DSS_ESM_ERROR  SDL_DSS_ESMG2_DSS_DCCA_ERR
#define  DCCB_DSS_ESM_ERROR  SDL_DSS_ESMG2_DSS_DCCB_ERR

#endif
/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/

/*===========================================================================*/
/*                         Local Function definitions                        */
/*===========================================================================*/

#endif /* SDL_DCC_TEST_H */
/* Nothing past this point */
