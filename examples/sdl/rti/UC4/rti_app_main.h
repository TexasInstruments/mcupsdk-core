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
 *  \file     rti_app_main.h
 *
 *  \brief    This file contains RTI Example test code declarations
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
/*===========================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <drivers/soc.h>
#include <sdl/sdl_rti.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/dpl/sdl_dpl.h>
#if defined (SOC_AM64X) || (SOC_AM243X)
#include <sdl/sdl_esm.h>
#endif
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#include <drivers/sciclient.h>
#include <sdl/sdl_esm.h>
#endif
#if defined (SOC_AM263X)
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/esm/v0/v0_0/sdl_ip_esm.h>
#elif defined (SOC_AM263PX) || defined (SOC_AM261X)
#include <sdl/esm/v2/sdl_esm.h>
#include <sdl/esm/v2/v2_0/sdl_ip_esm.h>
#elif defined (SOC_AM273X) || defined (SOC_AWR294X)
#include <sdl/esm/v1/sdl_esm.h>
#endif

#if !defined(SDL_RTI_TEST_H)
#define SDL_RTI_TEST_H

#if !defined (SOC_AWR294X) && !defined (SOC_AM273X)
#define RTI_WDT_TIMEOUT     (10000U)
#else
#define RTI_WDT_TIMEOUT     (100U)
#endif /* TPR12 , AWR294X */
#define SOC_MODULES_END     (0xFFFFFFFFu)
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define SDL_INSTANCE_ESM0 SDL_ESM_INST_MAIN_ESM0
#elif defined (R5F_INPUTS)
#define SDL_INSTANCE_ESM0 SDL_ESM_INST_MSS_ESM
#elif defined (C66_INPUTS)
#define SDL_INSTANCE_ESM0 SDL_ESM_INST_DSS_ESM
#endif

/* ========================================================================== */
/*                     Dependant macros in SDL_RTI_exampleTest.c                  */
/* ========================================================================== */

#define RTI_CLOCK_SOURCE_32KHZ_FREQ_KHZ        (32U)
#define RTI_CLOCK_SOURCE_200MHZ_FREQ_KHZ       (200000U)
/**< RTI Clock Source Selection */
#define RTI_NO_INTERRUPT                       (0U)
#define RTI_CPU_INTERRUPT                      (1U)
#define RTI_ESM_INTRPT                         (2U)


#if defined (SOC_AWR294X) || defined (SOC_AM273X)
#define MCU_ESM_RTI0_INTR                   ESMG2_WDT_NMI_REQ
#else
#define MCU_ESM_RTI0_INTR                     (104U)
#define MCU_ESM_RTI1_INTR                     (105U)
#endif

#define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_MASK                  (0x00000007U)

#if !defined (SOC_AM64X) && !defined (SOC_AM243X)
#define SDL_MCU_CTRL_MMR0_CFG0_BASE                                         (0x40f00000UL)
#endif
#define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL                               (0x00008180U)
#define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL_MASK                  (0x00000007U)
#define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)

#define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL                               (0x00008184U)
#define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
#define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_MAX                   (0x00000007U)

#define SDLR_MCU_R5FSS0_CORE0_INTR_MCU_RTI0_INTR_WWD_0                      (42U)
#define SDLR_MCU_R5FSS0_CORE0_INTR_MCU_ESM0_ESM_INT_HI_LVL_0                (49U)
#define SDLR_MCU_R5FSS0_CORE0_INTR_MCU_RTI1_INTR_WWD_0                      (43U)

#define RTI_DWD_MIN_PRELOAD_VAL             (0x1FFFU)

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct sdlRtiTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} sdlRtiTest_t;

typedef enum rtiIntrType
{
    RTI_CORE_INTERRUPT = 0U,
    /**< route RTI interrupt to core */
    RTI_ESM_INTERRUPT  = 1U,
    /**< route RTI interrupt to ESM */
}rtiIntrType_t;

typedef enum rtiClockSource
{
    RTI_CLOCK_SOURCE_HFOSC0_CLKOUT = 0U,
    /**< to select clock frequency of hfosc0 */
    RTI_CLOCK_SOURCE_LFOSC_CLKOUT = 1U,
    /**< to select clock frequency of lfosc */
    RTI_CLOCK_SOURCE_12MHZ = 2U,
    /**< to select clock frequency of 12 MHz */
    RTI_CLOCK_SOURCE_32KHZ = 3U,
    /**< to select clock frequency of 32KHz */
    RTI_CLOCK_SOURCE_200MHZ = 4U,
    /**< to select clock frequency of 200 MHz */
}rtiClockSource_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

#define SDL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED         (-(int32_t) (1))
#define SDL_APP_TEST_PASS           ( (int32_t) (0))
#define RTI_NO_INTERRUPT                        (0U)
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
#if defined (SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg);
#endif

/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/

extern int32_t SDL_RTI_exampleTest(void);
#if defined (SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg);
#elif defined (SOC_AM273X) || defined (SOC_AWR294X)
extern int32_t SDL_ESM_applicationCallbackFunction (SDL_ESM_Inst instance, int32_t grpChannel, int32_t vecNum, void *arg);
#endif

#endif /* SDL_RTI_TEST_H */
/* Nothing past this point */
