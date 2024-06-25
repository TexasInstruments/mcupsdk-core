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
 *  \brief    This file contains DCC Function test code declarations
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
/*===========================================================================*/
#include <sdl/sdl_dcc.h>
#include <sdl/esm/sdlr_esm.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <dpl_interface.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_dpl_config.h"


#if defined (SOC_AM263X)
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/include/am263x/sdlr_soc_baseaddress.h>
#include <sdl/include/am263x/sdlr_intr_r5fss0_core0.h>
#endif

#if defined (SOC_AM263PX) || defined (SOC_AM261X)
#include <sdl/esm/v2/sdl_esm.h>
#include <sdl/include/am263px/sdlr_soc_baseaddress.h>
#include <sdl/include/am263px/sdlr_intr_r5fss0_core0.h>
#endif

#if defined (SOC_AWR294X)
#include <sdl/esm/v1/sdl_esm.h>
#include <sdl/include/awr294x/sdlr_soc_baseaddress.h>
#include <sdl/include/awr294x/sdlr_intr_mss.h>
#include <sdl/include/awr294x/sdlr_intr_dss.h>
#include <sdl/include/awr294x/sdlr_dss_rcm.h>
#include <sdl/include/awr294x/sdlr_mss_rcm.h>
#include <sdl/include/awr294x/sdlr_intr_esm_dss.h>
#include <sdl/include/awr294x/sdlr_intr_esm_mss.h>
#endif

/* Interrupt Registrations */
#ifdef UNITY_INCLUDE_CONFIG_H
#include <unity/unity.h>
#include <unity/unity_config.h>
#endif


#if !defined(SDL_DCC_TEST_H)
#define SDL_DCC_TEST_H

/* ========================================================================== */
/*                     Dependant macros in sdl_dcc_funcTest.c                  */
/* ========================================================================== */

#define APP_DCC_STR                     "SDL DCC FUNCTION TEST"
/**< Example Common display string */
#if defined (SOC_AWR294X)
#if defined (R5F_INPUTS)
#define APP_DCC_MODULE_INST             (SDL_MSS_DCCA_U_BASE)
#endif
#if defined (C66_INPUTS)
#define APP_DCC_MODULE_INST             (SDL_DSS_DCCA_U_BASE)
#endif
#endif
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define APP_DCC_MODULE_INST             (SDL_DCC0_U_BASE)
#endif
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
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define APP_DCC_REF_CLOCK_SRC_0         (SDL_DCC_CLK0_SRC_CLOCK0_2) /*RCCLK10M (10MHZ) */
#define APP_DCC_TEST_CLOCK_SRC_1        (SDL_DCC_CLK1_SRC_CLOCKSRC3) /* XTALCLK(25MHZ)*/
#endif

#define APP_DCC_TEST_CLOCK_SRC_1_DRIFT  (2U)
/**< Allowed drift in percentage (+/-) */


#if defined (SOC_AWR294X)
#if defined (R5F_INPUTS)
#define APP_DCC_REF_CLOCK_SRC_0         (SDL_DCC_CLK0_SRC_CLOCK0_0) /*XTALCLK (40MHZ)*/
#define APP_DCC_TEST_CLOCK_SRC_1        (SDL_DCC_CLK1_SRC_CLOCKSRC2) /* SYSCLK(100MHZ) */
#endif
#if defined (C66_INPUTS)
#define APP_DCC_REF_CLOCK_SRC_0         (SDL_DCC_CLK0_SRC_CLOCK0_0) /*XTALCLK (40MHZ)*/
#define APP_DCC_TEST_CLOCK_SRC_1        (SDL_DCC_CLK1_SRC_CLOCKSRC3) /* DSS_RTIA(150MHZ)*/
#endif
#endif


#if defined (SOC_AWR294X)
#if defined (R5F_INPUTS)/*FOR R5F CORE */
#define APP_DCC_REF_CLOCK_FREQ_IN_KHZ   (40000U)
/**< Clock source for Counter 0, 40 MHz for AWR294X */
#define APP_DCC_TEST_CLOCK_FREQ_IN_KHZ  (30000U)
/**< Expected test clock frequency in KHz */
/**< Clock source for Counter 1, 30 MHz for AWR294X */
#define APP_DCC_TEST_CLOCK_SRC_1_HIGHER (DCC_DCCCLKSRC1_CLKSRC_2)
/**< Clock source for Counter 1, expected to be higher than
        APP_DCC_TEST_CLOCK_SRC_1, in this Test to simulate an error*/
#endif
#if defined (C66_INPUTS)/*FOR DSP CORE */
#define APP_DCC_REF_CLOCK_FREQ_IN_KHZ   (40000U)
/**< Clock source for Counter 0, 40 MHz for AWR294X */
#define APP_DCC_TEST_CLOCK_FREQ_IN_KHZ  (100000U)
/**< Expected test clock frequency in KHz */
/**< Clock source for Counter 1, 100 MHz for AWR294X */
#define APP_DCC_TEST_CLOCK_SRC_1_HIGHER (DCC_DCCCLKSRC1_CLKSRC_0)
/**< Clock source for Counter 1, expected to be higher than
        APP_DCC_TEST_CLOCK_SRC_1, in this Test to simulate an error*/
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define APP_DCC_REF_CLOCK_FREQ_IN_KHZ   (25000U)
/**< Clock source for Counter 0, 25 MHz for AM263X or AM263PX*/
#define APP_DCC_TEST_CLOCK_FREQ_IN_KHZ  (32U)
/**< Expected test clock frequency in KHz */
/**< Clock source for Counter 1, 32 KHz for AM263X or AM263PX*/
#define APP_DCC_TEST_CLOCK_SRC_1_HIGHER (DCC_DCCCLKSRC1_CLKSRC_0)
/**< Clock source for Counter 1, expected to be higher than
        APP_DCC_TEST_CLOCK_SRC_1, in this Test to simulate an error*/

#endif


/* DCC INPUT MACROS*/
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define ESM_INST_BASE      (SDL_TOP_ESM_U_BASE)
#define DCC_INST_BASE      (SDL_DCC0_U_BASE)
#define DCC_INST_NUM       (SDL_DCC_INST_MSS_DCCA)
#define DCC_DONE_INTR_NUM  (SDL_R5FSS0_CORE0_INTR_DCC0_DONE)
#endif

#if defined (SOC_AWR294X)
#if defined (R5F_INPUTS)
#define ESM_INST_BASE      (SDL_TOP_ESM_U_BASE)
#define DCC_INST_BASE      (SDL_MSS_DCCA_U_BASE)
#define DCC_INST_NUM       (SDL_DCC_INST_MSS_DCCA)
#define DCC_DONE_INTR_NUM  (SDL_MSS_INTR_MSS_DCCA_INT)
#endif
#if defined (C66_INPUTS)
#define ESM_INST_BASE      (SDL_TOP_ESM_U_BASE)
#define DCC_INST_BASE      (SDL_DSS_DCCA_U_BASE)
#define DCC_INST_NUM       (SDL_DCC_INST_DSS_DCCA)
#define DCC_DONE_INTR_NUM  (SDL_DSS_INTR_DSS_DCCA_INT)
#endif
#endif

/* ESM ERROR CONFIGURATION MACROS */

#if defined (SOC_AWR294X)

#define ESM_ERROR_GROUP_1    1U
#define ESM_ERROR_GROUP_2    2U
#define ESM_ERROR_GROUP_3    3U

#define  DCCA_MSS_ESM_ERROR  ESMG1_DCCA_ERR
#define  DCCB_MSS_ESM_ERROR  ESMG1_DCCB_ERR
#define  DCCC_MSS_ESM_ERROR  ESMG1_DCCC_ERR
#define  DCCD_MSS_ESM_ERROR  ESMG1_DCCD_ERR

#define  DCCA_DSS_ESM_ERROR  SDL_DSS_ESMG2_DSS_DCCA_ERR
#define  DCCB_DSS_ESM_ERROR  SDL_DSS_ESMG2_DSS_DCCA_ERR

#endif
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


/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

#define SDL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED         (-(int32_t) (1))
#define SDL_APP_TEST_PASS           ( (int32_t) (0))

#define DCC_NO_INTERRUPT    		(0u)
#define DCC_INTERRUPT				(1u)

/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern int32_t SDL_DCC_funcTest(void);

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg);



extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg);
#elif (SOC_AWR294X)

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            int grpChannel,
                                            int intSrc,
                                            void *arg);

extern int32_t SDL_DCC_example(void);
extern void SDL_DCCA_clockInit(void);

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            int grpChannel,
                                            int intSrc,
                                            void *arg);
SDL_Result SDL_ESM_init (const SDL_ESM_Inst esmInstType,
                         SDL_ESM_NotifyParams* params,
						 SDL_ESM_OpenParams *esmOpenParams,
                         void *arg);
#endif


/*===========================================================================*/
/*                         Local Function definitions                        */
/*===========================================================================*/

#endif /* SDL_DCC_TEST_H */
/* Nothing past this point */
