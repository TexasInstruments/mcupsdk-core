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
 *  \file     ecc_trigger.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Error Correcting Code (ECC) Module application.
 *
 *  \details  ECC Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include <drivers/edma/v0/edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#include <sdl/sdl_ecc.h>
#include "ecc_test_main.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* This macro shows how many ESM events are configured*/
#define SDL_INTR_GROUP_NUM_1                        (1U)
#define SDL_INTR_GROUP_NUM_2                        (2U)
#define SDL_INTR_GROUP_NUM_3                        (3U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

/* Defines */
#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (7u)
#define SDL_R5FSS0_CORE1_MAX_MEM_SECTIONS           (6u)
#define SDL_R5FSS0_CORE0_CACHE_MAX_MEM_SECTIONS     (21U)
#define SDL_MCAN0_MAX_MEM_SECTIONS                  (1U)
#define SDL_MCAN1_MAX_MEM_SECTIONS                  (1U)
#define SDL_MCAN2_MAX_MEM_SECTIONS                  (1U)
#define SDL_MCAN3_MAX_MEM_SECTIONS                  (1U)
#define SDL_MSS_MAX_MEM_SECTIONS					(7U)
#define SDL_ICSSM_MAX_MEM_SECTIONS           		(5u)

#define SDL_MSS_L2_MEM_INIT_ADDR                    (0x50D00240u)
#define SDL_MSS_L2_MEM_INIT_DONE_ADDR               (0x50D00244u)
#define SDL_ECC_AGGR_ERROR_STATUS1_ADDR             (0x53000020u)
#define SDL_ECC_MSS_L2_BANK_MEM_INIT                (0xcu) /*Bank 3*/

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* This is the list of exception handle and the parameters */
const SDL_R5ExptnHandlers ECC_Test_R5ExptnHandlers =
{
    .udefExptnHandler = &SDL_EXCEPTION_undefInstructionExptnHandler,
    .swiExptnHandler = &SDL_EXCEPTION_swIntrExptnHandler,
    .pabtExptnHandler = &SDL_EXCEPTION_prefetchAbortExptnHandler,
    .dabtExptnHandler = &SDL_EXCEPTION_dataAbortExptnHandler,
    .irqExptnHandler = &SDL_EXCEPTION_irqExptnHandler,
    .fiqExptnHandler = &SDL_EXCEPTION_fiqExptnHandler,
    .udefExptnHandlerArgs = ((void *)0u),
    .swiExptnHandlerArgs = ((void *)0u),
    .pabtExptnHandlerArgs = ((void *)0u),
    .dabtExptnHandlerArgs = ((void *)0u),
    .irqExptnHandlerArgs = ((void *)0u),
};

static SDL_ECC_MemSubType ECC_Test_R5FSS0_CORE0_CACHE_subMemTypeList[SDL_R5FSS0_CORE0_CACHE_MAX_MEM_SECTIONS] =
{
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK2_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_IDATA_BANK3_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID,
    SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE0_CACHE_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE0_CACHE_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE0_CACHE_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_R5FSS0_CORE0_subMemTypeList[SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS] =
{
     SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
     SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID,
     SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID,
     SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID,
     SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID,
     SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID,
     SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID,

};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE0_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE0_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_R5FSS0_CORE1_subMemTypeList[SDL_R5FSS0_CORE1_MAX_MEM_SECTIONS] =
{
     SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_ID,
     SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_RAM_ID,
     SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_ID,
     SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_ID,
     SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_ID,
     SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_ID,

};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE1_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE1_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE1_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MCAN0_subMemTypeList[SDL_MCAN0_MAX_MEM_SECTIONS] =
{
     SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCAN0_ECCInitConfig =
{
    .numRams = SDL_MCAN0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCAN0_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MCAN1_subMemTypeList[SDL_MCAN1_MAX_MEM_SECTIONS] =
{
     SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCAN1_ECCInitConfig =
{
    .numRams = SDL_MCAN1_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCAN1_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MCAN2_subMemTypeList[SDL_MCAN2_MAX_MEM_SECTIONS] =
{
     SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCAN2_ECCInitConfig =
{
    .numRams = SDL_MCAN2_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCAN2_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MCAN3_subMemTypeList[SDL_MCAN3_MAX_MEM_SECTIONS] =
{
     SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCAN3_ECCInitConfig =
{
    .numRams = SDL_MCAN3_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCAN3_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MSSsubMemTypeList[SDL_MSS_MAX_MEM_SECTIONS] =
{
    SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID,
    SDL_SOC_ECC_AGGR_TPTC_A1_ECC_RAM_ID
};

static SDL_ECC_InitConfig_t ECC_Test_MSSECCInitConfig =
{
    .numRams = SDL_MSS_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MSSsubMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_ICSSMsubMemTypeList[SDL_ICSSM_MAX_MEM_SECTIONS] =
{
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_ID,
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_RAM_ID,
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_RAM_ID,
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_RAM_ID,
    SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_ICSSMECCInitConfig =
{
    .numRams = SDL_ICSSM_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_ICSSMsubMemTypeList[0]),
    /**< Sub type list  */
};

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);

/* Event BitMap for ECC ESM callback for MAIN */
static uint32_t arg;

SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
     .esmErrorConfig = {1u, 8u}, /* Self test error config */
     .enableBitmap = {0x001803fcu, 0x007f8000u, 0x00006000u, 0x00000000u,
                      0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
      /**< All events enable: except clkstop events for unused clocks
       *   and PCIE events */
       /* CCM_1_SELFTEST_ERR and _R5FSS0COMPARE_ERR_PULSE_0 */
     .priorityBitmap = {0x001002a8u, 0x001d0000u, 0x00002000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
     /**< All events high priority: except clkstop events for unused clocks
      *   and PCIE events */
     .errorpinBitmap = {0x001803fcu, 0x007f8000u, 0x00006000u, 0x00000000u,
                        0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
     /**< All events high priority: except clkstop for unused clocks
      *   and PCIE events */
};

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ECC_Example_init function */
int32_t ECC_Test_MCAN_init (void);
int32_t ECC_Test_R5F_CACHE_init(void);
int32_t ECC_Test_R5F_init (void);
int32_t ECC_Test_MSS_init (void);
int32_t ECC_Test_ICSSM_init (void);
//static int32_t edma_interrupt_transfer(uint32_t edmaConfigNum, uint32_t injectType, uint32_t queueType, uint32_t channelEvent);
static int32_t ECC_sdlFuncTest(void);

//static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);

void ECC_Test_undefInstructionExptnCallback(void)
{
    printf("\r\nUndefined Instruction exception\r\n");
}

void ECC_Test_swIntrExptnCallback(void)
{
    printf("\r\nSoftware interrupt exception\r\n");
}

void ECC_Test_prefetchAbortExptnCallback(void)
{
    printf("\r\nPrefetch Abort exception\r\n");
}
void ECC_Test_dataAbortExptnCallback(void)
{
    printf("\r\nData Abort exception\r\n");
}
void ECC_Test_irqExptnCallback(void)
{
    printf("\r\nIrq exception\r\n");
}

void ECC_Test_fiqExptnCallback(void)
{
    printf("\r\nFiq exception\r\n");
}

void ECC_Test_exceptionInit(void)
{

    SDL_EXCEPTION_CallbackFunctions_t exceptionCallbackFunctions =
            {
             .udefExptnCallback = ECC_Test_undefInstructionExptnCallback,
             .swiExptnCallback = ECC_Test_swIntrExptnCallback,
             .pabtExptnCallback = ECC_Test_prefetchAbortExptnCallback,
             .dabtExptnCallback = ECC_Test_dataAbortExptnCallback,
             .irqExptnCallback = ECC_Test_irqExptnCallback,
             .fiqExptnCallback = ECC_Test_fiqExptnCallback,
            };

    /* Initialize SDL exception handler */
    SDL_EXCEPTION_init(&exceptionCallbackFunctions);
    /* Register SDL exception handler */
    Intc_RegisterExptnHandlers(&ECC_Test_R5ExptnHandlers);

    return;
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
* @fn      ECC_Test_MCAN_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t ECC_Test_MCAN_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;

    if (retValue == 0) {
         /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR, SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("\r\nECC_Test_init: Error initializing Memory of MCAN0 ECC: result = %d\r\n", result);

             retValue = -1;
         } else {
             DebugP_log("\r\nECC_Test_init: Initialize of MCAN0 ECC Memory is complete \r\n");
         }
    }
	
	if (retValue == 0) {
         /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR, SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("\r\nECC_Test_init: Error initializing Memory of MCAN1 ECC: result = %d\r\n", result);

             retValue = -1;
         } else {
             DebugP_log("\r\nECC_Test_init: Initialize of MCAN1 ECC Memory is complete \r\n");
         }
    }

    if (retValue == 0) {
         /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR, SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("\r\nECC_Test_init: Error initializing Memory of MCAN2 ECC: result = %d\r\n", result);

             retValue = -1;
         } else {
             DebugP_log("\r\nECC_Test_init: Initialize of MCAN2 ECC Memory is complete \r\n");
         }
    }

    if (retValue == 0) {
         /* Initialize ECC Memory */
         result = SDL_ECC_initMemory(SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR, SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID);
         if (result != SDL_PASS) {
             /* print error and quit */
             DebugP_log("\r\nECC_Test_init: Error initializing Memory of MCAN3 ECC: result = %d\r\n", result);

             retValue = -1;
         } else {
             DebugP_log("\r\nECC_Test_init: Initialize of MCAN3 ECC Memory is complete \r\n");
         }
    }


    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR, &ECC_Test_MCAN0_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MCAN0 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MCAN0 ECC initialization is completed \r\n");
        }
    }
	
	if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR, &ECC_Test_MCAN1_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MCAN1 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MCAN1 ECC initialization is completed \r\n");
        }
    }

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR, &ECC_Test_MCAN2_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MCAN2 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MCAN2 ECC initialization is completed \r\n");
        }
    }

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR, &ECC_Test_MCAN3_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MCAN3 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MCAN3 ECC initialization is completed \r\n");
        }
    }
    return retValue;
}/* End of ECC_Test_MCAN_init() */

/*********************************************************************
* @fn      ECC_Test_R5F_CACHE_init
*
* @brief   Initializes Software Diagnostics Test Framework
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*********************************************************************/
int32_t ECC_Test_R5F_CACHE_init (void)
{
    int32_t retValue=0U;
    SDL_ErrType_t result;

    /* Initialise exception handler */
    ECC_Test_exceptionInit();

    DebugP_log("\r\nECC_Test_init: Exception init complete \r\n");

	/*Enabling the Cache Event bus*/
	SDL_UTILS_enable_cache_event_bus();

    if (retValue == 0U) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_R5FSS0_CORE0_ECC_AGGR, &ECC_Test_R5FSS0_CORE0_CACHE_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Test_init: Error initializing R5FSS0 CORE0 CACHE ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: R5FSS0 CORE0 CACHE ECC initialization is completed \r\n");
        }
    }
	
    return retValue;
}/* End of ECC_Test_R5F_CACHE_init() */

/*********************************************************************
* @fn      ECC_Test_R5F_init
*
* @brief   Initializes Software Diagnostics Test Framework
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*********************************************************************/
int32_t ECC_Test_R5F_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;
	SDL_ECC_staticRegs staticRegs;

    /*Enabling the ATCM0 ECC module*/
    SDL_ECC_UTILS_enableECCATCM();
	
	/*Enabling the B0TCM ECC module*/
	SDL_ECC_UTILS_enableECCB0TCM();
	
    /*Enabling the B1TCM ECC module*/
    SDL_ECC_UTILS_enableECCB1TCM();

    /*Enabling the Event bus*/
    SDL_UTILS_enable_event_bus();

    /* Initialise exception handler */
    ECC_Test_exceptionInit();

    DebugP_log("\r\nECC_Test_init: Exception init complete \r\n");

    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_R5FSS0_CORE0_ECC_AGGR, &ECC_Test_R5FSS0_CORE0_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing R5FSS0 CORE0 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: R5FSS0 CORE0 ECC initialization is completed \r\n");
        }
    }
	if (retValue == 0U) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_R5FSS0_CORE1_ECC_AGGR, &ECC_Test_R5FSS0_CORE1_ECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Test_init: Error initializing R5FSS0 CORE1 ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: R5FSS0 CORE1 ECC initialization is completed \r\n");
        }
    }

    if (retValue == 0) {
        /* Read back the static registers */
        result = SDL_ECC_getStaticRegisters(SDL_R5FSS0_CORE0_ECC_AGGR, &staticRegs);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error reading the R5FSS0 CORE0 static registers: result = %d\r\n");

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: R5FSS0 CORE0 Memtype Register Readback successful \r\n");
        }
    }

	if (retValue == 0U) {
        /* Read back the static registers */
        result = SDL_ECC_getStaticRegisters(SDL_R5FSS0_CORE1_ECC_AGGR, &staticRegs);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error reading the R5FSS0 CORE1 static registers: result = %d\r\n");

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: R5FSS0 CORE1 Memtype Register Readback successful \r\n");
        }
    }
	
    return retValue;
}/* End of ECC_Test_R5F_init() */

/*********************************************************************
* @fn      ECC_Test_MSS_init
*
* @brief   Initializes Software Diagnostics MSS Test Framework
*
* @param   None
*
* @return    0 : Success; < 0 for failures
*********************************************************************/
int32_t ECC_Test_MSS_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;

    /* Clear Done memory*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, 0xfu);

    /* Initialization of MSS L2 memory*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);

    while(SDL_REG32_RD(SDL_MSS_L2_MEM_INIT_DONE_ADDR)!=SDL_ECC_MSS_L2_BANK_MEM_INIT);

    /* Clear Done memory after MEM init*/
    SDL_REG32_WR(SDL_MSS_L2_MEM_INIT_DONE_ADDR, SDL_ECC_MSS_L2_BANK_MEM_INIT);

    /*Clearing any old interrupt presented*/
    SDL_REG32_WR(SDL_ECC_AGGR_ERROR_STATUS1_ADDR, 0xF0Fu);
	
    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_SOC_ECC_AGGR, &ECC_Test_MSSECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing MSS ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: MSS ECC Init complete \r\n");
        }
    }
    return retValue;
}/* End of ECC_Test_MSS_init() */

/*********************************************************************
* @fn      ECC_Test_ICSSM_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t ECC_Test_ICSSM_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;
    void *ptr = (void *)&arg;
	uint8_t u8Counter = 0;

	if (retValue == 0U) {
	     for(u8Counter = 0; u8Counter < SDL_ICSSM_MAX_MEM_SECTIONS; u8Counter++)
	     {
             /* Initialize ECC Memory */
             result = SDL_ECC_initMemory(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR, ECC_Test_ICSSMsubMemTypeList[u8Counter]);
             if (result != SDL_PASS) {
                  /* print error and quit */
                  DebugP_log("\r\nECC_Test_init: Error initializing Memory of ICSSM ECC: result = %d\r\n", result);

                  retValue = -1;
             }
	     }
         if (result == SDL_PASS) {
             DebugP_log("\r\nECC_Test_init: Initialize of ICSSM ECC Memory is complete \r\n");
         }
    }
    if (retValue == 0) {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing ICSSM ESM: result = %d\r\n", result);


            retValue = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init ICSSM ESM complete \r\n");
        }

    }
    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR, &ECC_Test_ICSSMECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nECC_Test_init: Error initializing ICSSM ECC: result = %d\r\n", result);

            retValue = -1;
        } else {
            DebugP_log("\r\nECC_Test_init: ICSSM ECC initialization is completed \r\n");
        }
    }
    return retValue;
}
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_1BitInjectTest
 *
 * @brief   Execute ECC MCAN0  1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

	DebugP_log("\r\nMCAN0 Single bit error self test: starting \r\n");
			   
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0  1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_selfTest(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                             &injectErrorConfig,
                             10000);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {

        DebugP_log("\r\nMCAN0 Single bit error self test inject at pErrMem = 0x%p:test complete \r\n",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN1_1BitInjectTest
 *
 * @brief   Execute ECC MCAN1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nMCAN1 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52610000u);

    /* Run one shot test for MCAN3 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                          SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                          SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                          &injectErrorConfig);

    if (result != SDL_PASS ) {
     retVal = -1;
    } else {
     /* Access the memory where injection is expected */
     testLocationValue = injectErrorConfig.pErrMem[0];
     DebugP_log("\r\nMCAN1 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                    injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN2_1BitInjectTest
 *
 * @brief   Execute ECC MCAN2  1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN2_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nMCAN2 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52620000u);

    /* Run one shot test for MCAN3 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                          SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                          SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                          &injectErrorConfig);

    if (result != SDL_PASS ) {
     retVal = -1;
    } else {
     /* Access the memory where injection is expected */
     testLocationValue = injectErrorConfig.pErrMem[0];
     DebugP_log("\r\nMCAN2 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                    injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN2_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN3_1BitInjectTest
 *
 * @brief   Execute ECC MCAN3 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN3_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN3 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52630000u);

    /* Run one shot test for MCAN3 1 bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMCAN3 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                       injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN3_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_1Bit_N_ROWInjectTest
 *
 * @brief   Execute ECC MCAN0 1 N ROW bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_1Bit_N_ROWInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Single bit N ROW error inject: starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0  1 N Row bit error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Single bit N ROW error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_1Bit_N_ROWInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_1Bit_Repeat_InjectTest
 *
 * @brief   Execute ECC MCAN0  1 Repeat bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_1Bit_Repeat_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Single bit repeat error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0  1 bit repeat error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Single bit repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_1Bit_Repeat_InjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest
 *
 * @brief   Execute ECC MCAN0  1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Single bit N ROW Repeat error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0  1 bit N Row Repeat error */
    injectErrorConfig.flipBitMask = 0x002;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Single bit N ROW Repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_2BitInjectTest
 *
 * @brief   Execute ECC MCAN0  2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

	DebugP_log("\r\nMCAN0 Double bit error self test: starting \r\n");
		
    /* Run one shot test for MCAN0  2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_selfTest(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                                 SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig,
								 10000);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nMCAN0 Double bit error self test: pErrMem fixed location = 0x%p once test complete \r\n",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN1_2BitInjectTest
 *
 * @brief   Execute ECC MCAN1 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nMCAN1 Double bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52610000u);

    /* Run one shot test for MCAN1 2 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                          SDL_MCAN1_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                          SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                          &injectErrorConfig);

    if (result != SDL_PASS ) {
     retVal = -1;
    } else {
     /* Access the memory where injection is expected */
     testLocationValue = injectErrorConfig.pErrMem[0];
     DebugP_log("\r\nMCAN1 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                    injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN2_2BitInjectTest
 *
 * @brief   Execute ECC MCAN2 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN2_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nMCAN2 Double bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52620000u);

    /* Run one shot test for MCAN2 2 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                          SDL_MCAN2_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                          SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                          &injectErrorConfig);

    if (result != SDL_PASS ) {
     retVal = -1;
    } else {
     /* Access the memory where injection is expected */
     testLocationValue = injectErrorConfig.pErrMem[0];
     DebugP_log("\r\nMCAN2 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                    injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN2_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN3_2BitInjectTest
 *
 * @brief   Execute ECC MCAN3 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN3_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nMCAN3 Double bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52630000u);

    /* Run one shot test for MCAN3 2 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN3_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMCAN3 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                       injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN3_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_2Bit_N_ROWInjectTest
 *
 * @brief   Execute ECC MCAN0 2 N ROW bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_2Bit_N_ROWInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Double bit N ROW error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0 2 N Row bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
									SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
									SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE,
									&injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Double bit N ROW error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_2Bit_N_ROWInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_2Bit_Repeat_InjectTest
 *
 * @brief   Execute ECC MCAN0 2 Repeat bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_2Bit_Repeat_InjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Double bit repeat error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0 2 bit repeat error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Double bit repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_2Bit_Repeat_InjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest
 *
 * @brief   Execute ECC MCAN0 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nMCAN0 Double bit N ROW Repeat error inject: starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x52600000u);

    /* Run one shot test for MCAN0 2 bit N Row Repeat error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR,
                             SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID,
                             SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                             &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];

        DebugP_log("\r\nMCAN0 Double bit N ROW Repeat error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ITAG RAM0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM0 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE0 ITAG RAM0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM0 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ITAG RAM0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM0 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 ITAG RAM0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ITAG RAM1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM1 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000010u);

    /* Run one shot test for R5FSS0 CORE0 ITAG RAM1 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM1 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ITAG RAM1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM1 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 ITAG RAM1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000010u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM1 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ITAG RAM2 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM2 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000020u);

    /* Run one shot test for R5FSS0 CORE0 ITAG RAM2 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM2 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ITAG RAM2 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM2 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 ITAG RAM2 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000020u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM2 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ITAG RAM3 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM3 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000030u);

    /* Run one shot test for R5FSS0 CORE0 ITAG RAM3 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM3 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ITAG RAM3 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM3 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 ITAG RAM3 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000030u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ITAG RAM3 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 DTAG RAM0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM0 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE0 DTAG RAM0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM0 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 DTAG RAM0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM0 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 DTAG RAM0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 DTAG RAM1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM1 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE0 DTAG RAM0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM1 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 DTAG RAM1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM1 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 DTAG RAM1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM1 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 DTAG RAM2 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM2 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE0 DTAG RAM2 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM2 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 DTAG RAM2 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM2 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 DTAG RAM2 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM2 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 DTAG RAM3 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM3 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE0 DTAG RAM3 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM3 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 DTAG RAM3 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM3 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 DTAG RAM3 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG RAM3 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Single bit error self test: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000510u);

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_selfTest(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig,
                                 10000);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {

        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Single bit error self test at pErrMem = 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000510u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK1 Single bit error self test: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000514u);

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK1 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_selfTest(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig,
                                 10000);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {

        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK1 Single bit error self test at pErrMem = 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK1 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000514u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK1_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK1 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B0TCM0 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

	DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK0 single bit error self test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00080000u);

    /* Run one shot test for R5FSS0 CORE0 B0TCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_selfTest(SDL_R5FSS0_CORE0_ECC_AGGR,
                              SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig,
					    	  10000);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK0 single bit error self test inject: pErrMem fixed location = 0x%p once test complete \r\n",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B0TCM0 BANK0 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK0 double bit error inject : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00080000u);

    /* Run one shot test for R5FSS0 CORE0 B0TCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                              SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK0_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                              &injectErrorConfig);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK0 double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B0TCM0 BANK1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK1 single bit error inject : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00080004u);

    /* Run one shot test for R5FSS0 CORE0 B0TCM0 BANK1 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                              SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK1 single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B0TCM0 BANK1 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK1 double bit error inject : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00080004u);

    /* Run one shot test for R5FSS0 CORE0 B0TCM0 BANK1 1 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                              SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B0TCM0_BANK1_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                              &injectErrorConfig);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 B0TCM0 BANK1 double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B1TCM0 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 B1TCM0 BANK0 single bit error inject : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00080008u);

    /* Run one shot test for R5FSS0 CORE0 B1TCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 B1TCM0 BANK0 single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B1TCM0 BANK0 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 B1TCM0 BANK0 double bit error inject : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00080008u);

    /* Run one shot test for R5FSS0 CORE0 B1TCM0 BANK0 2 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 B1TCM0 BANK0 double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B1TCM0 BANK1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 B1TCM0 BANK1 single bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0008000cu);

    /* Run one shot test for R5FSS0 CORE0 B1TCM0 BANK1 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 B1TCM0 BANK1 single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 B1TCM0 BANK1 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE0 B1TCM0 BANK1 double bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0008000cu);

    /* Run one shot test for R5FSS0 CORE0 B1TCM0 BANK1 2 bit error */
    injectErrorConfig.flipBitMask = 0x30002;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE0 B1TCM0 BANK1 double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_VIM_1BitInjectTest
 *
 * @brief   Execute ECC VIM 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_VIM_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nVIM Single bit error inject: starting \r\n");

    /* Run one shot test for VIM 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x50F02000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nVIM Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_VIM_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_VIM_2BitInjectTest
 *
 * @brief   Execute ECC VIM 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_VIM_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nVIM Double bit error inject: starting \r\n");

    /* Run one shot test for VIM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x50F02000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_KS_VIM_RAMECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nVIM Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_VIM_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ITAG_RAM0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ITAG RAM0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ITAG_RAM0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM0 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE1 ITAG RAM0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM0 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ITAG_RAM0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ITAG_RAM0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ITAG RAM0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ITAG_RAM0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM0 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE1 ITAG RAM0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                 SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ITAG_RAM0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ITAG_RAM1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ITAG RAM1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ITAG_RAM1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM1 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE1 ITAG RAM1 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM1 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ITAG_RAM1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ITAG_RAM1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ITAG RAM1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ITAG_RAM1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM1 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE1 ITAG RAM1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                 SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM1_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM1 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ITAG_RAM1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ITAG_RAM2_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ITAG RAM2 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ITAG_RAM2_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM2 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE1 ITAG RAM2 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM2 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ITAG_RAM2_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ITAG_RAM2_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ITAG RAM2 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ITAG_RAM2_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM2 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE1 ITAG RAM2 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                 SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM2_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM2 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ITAG_RAM2_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ITAG_RAM3_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ITAG RAM3 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ITAG_RAM3_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM3 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE1 ITAG RAM3 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM3 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ITAG_RAM3_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ITAG_RAM3_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ITAG RAM3 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ITAG_RAM3_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM3 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE1 ITAG RAM3 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                 SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_ITAG_RAM3_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE1 ITAG RAM3 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ITAG_RAM3_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_DTAG_RAM0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 DTAG RAM0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_DTAG_RAM0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM0 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    /* Run one shot test for R5FSS0 CORE1 DTAG RAM0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM0 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_DTAG_RAM0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_DTAG_RAM0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 DTAG RAM0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_DTAG_RAM0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM0 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE1 DTAG RAM0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                 SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_DTAG_RAM0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_DTAG_RAM1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 DTAG RAM1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_DTAG_RAM1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM1 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000004u);

    /* Run one shot test for R5FSS0 CORE1 DTAG RAM0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM1 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_DTAG_RAM1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_DTAG_RAM1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 DTAG RAM1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_DTAG_RAM1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM1 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE1 DTAG RAM1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000004u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                 SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM1_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM1 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_DTAG_RAM1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_DTAG_RAM2_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 DTAG RAM2 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_DTAG_RAM2_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM2 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000008u);

    /* Run one shot test for R5FSS0 CORE1 DTAG RAM2 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM2 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_DTAG_RAM2_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_DTAG_RAM2_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 DTAG RAM2 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_DTAG_RAM2_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM2 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE1 DTAG RAM2 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000008u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                 SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM2_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM2 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_DTAG_RAM2_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_DTAG_RAM3_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 DTAG RAM3 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_DTAG_RAM3_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM3 Single bit error inject test: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x7000000cu);

    /* Run one shot test for R5FSS0 CORE1 DTAG RAM3 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM3 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_DTAG_RAM3_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_DTAG_RAM3_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 DTAG RAM3 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_DTAG_RAM3_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM3 Double bit error inject: starting \r\n");

    /* Run one shot test for R5FSS0 CORE1 DTAG RAM3 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x7000000cu);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                 SDL_R5FSS0_CORE1_ECC_AGGR_CPU1_DTAG_RAM3_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    /* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE1 DTAG RAM3 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_DTAG_RAM3_2BitInjectTest() */


/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ATCM1 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0U;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 ATCM1 BANK0 single bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00008510u);

    /* Run one shot test for R5FSS0 CORE1 ATCM1 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 ATCM1 BANK0 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ATCM1 BANK0 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0U;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 ATCM1 BANK0 double bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00008510u);

    /* Run one shot test for R5FSS0 CORE1 ATCM1 BANK0 2 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 ATCM1 BANK0 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ATCM1 BANK1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0U;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 ATCM1 BANK1 single bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00008514u);

    /* Run one shot test for R5FSS0 CORE1 ATCM1 BANK1 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK1_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 ATCM1 BANK1 Single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 ATCM1 BANK1 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0U;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 ATCM1 BANK1 double bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00008514u);

    /* Run one shot test for R5FSS0 CORE1 ATCM1 BANK1 2 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_ATCM1_BANK0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 ATCM1 BANK1 Double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 B0TCM1 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 B0TCM1 BANK0 single bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00088000u);

    /* Run one shot test for R5FSS0 CORE1 B0TCM1 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                              SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 B0TCM1 BANK0 single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 B0TCM1 BANK0 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 B0TCM1 BANK0 double bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00088000u);

    /* Run one shot test for R5FSS0 CORE1 B0TCM1 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                              SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK0_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                              &injectErrorConfig);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 B0TCM1 BANK0 double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 B0TCM1 BANK1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 B0TCM1 BANK1 single bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00088004u);

    /* Run one shot test for R5FSS0 CORE1 B0TCM1 BANK1 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                              SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                              &injectErrorConfig);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 B0TCM1 BANK1 single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 B0TCM1 BANK1 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 B0TCM1 BANK1 double bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00088004u);

    /* Run one shot test for R5FSS0 CORE1 B0TCM1 BANK1 1 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                              SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B0TCM1_BANK1_RAM_ID,
                              SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                              &injectErrorConfig);
		
    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 B0TCM1 BANK1 double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 B1TCM1 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 B1TCM1 BANK0 single bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00088008u);

    /* Run one shot test for R5FSS0 CORE1 B1TCM1 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 B1TCM1 BANK0 single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 B1TCM1 BANK0 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 B1TCM1 BANK0 double bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00088008u);

    /* Run one shot test for R5FSS0 CORE1 B1TCM1 BANK0 2 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK0_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 B1TCM1 BANK0 double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 B1TCM1 BANK1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 B1TCM1 BANK1 single bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0008800cu);

    /* Run one shot test for R5FSS0 CORE1 B1TCM1 BANK1 1 bit error */
    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 B1TCM1 BANK1 single bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE1 B1TCM1 BANK1 2 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

	DebugP_log("\r\nR5FSS0 CORE1 B1TCM1 BANK1 double bit error inject test : starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0008800cu);

    /* Run one shot test for R5FSS0 CORE1 B1TCM1 BANK1 2 bit error */
    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE1_ECC_AGGR,
                                SDL_R5FSS0_CORE1_ECC_AGGR_PULSAR_SL_B1TCM1_BANK1_RAM_ID,
                                SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                &injectErrorConfig);

    if (result != SDL_PASS ) {
        retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nR5FSS0 CORE1 B1TCM1 BANK1 double bit error inject: pErrMem fixed location = 0x%p once test complete: the value of pErrMem is 0x%p \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2_SLV0_1BitInjectTest
 *
 * @brief   Execute ECC MSS L2 SLV0 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2_SLV0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS L2 SLV0 Single bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 SLV0 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS L2 SLV0 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2_SLV0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2_SLV0_2BitInjectTest
 *
 * @brief   Execute ECC MSS L2 SLV0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2_SLV0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS L2 SLV0 Double bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 SLV0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS L2 SLV0 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2_SLV0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2_SLV1_1BitInjectTest
 *
 * @brief   Execute ECC MSS L2 SLV1 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2_SLV1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS L2 SLV1 Single bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 SLV1 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70080000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS L2 SLV1 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2_SLV1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2_SLV1_2BitInjectTest
 *
 * @brief   Execute ECC MSS L2 SLV1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2_SLV1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS L2 SLV1 Double bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 SLV1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70080000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS L2 SLV1 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2_SLV1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2_SLV2_1BitInjectTest
 *
 * @brief   Execute ECC MSS L2 SLV2 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2_SLV2_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS L2 SLV2 Single bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 SLV2 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70100000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS L2 SLV2 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2_SLV2_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2_SLV2_2BitInjectTest
 *
 * @brief   Execute ECC MSS L2 SLV2 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2_SLV2_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS L2 SLV2 Double bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 SLV2 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70100000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS L2 SLV2 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2_SLV2_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2_SLV3_1BitInjectTest
 *
 * @brief   Execute ECC MSS L2 SLV3 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2_SLV3_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS L2 SLV3 Single bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 SLV3 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70180000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS L2 SLV3 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2_SLV3_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_L2_SLV3_2BitInjectTest
 *
 * @brief   Execute ECC MSS L2 SLV3 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_L2_SLV3_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS L2 SLV3 Double bit error inject: starting \r\n");

    /* Run one shot test for MSS L2 SLV3 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x70180000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS L2 SLV3 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_L2_SLV3_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_MAILBOX_1BitInjectTest
 *
 * @brief   Execute ECC MSS MAILBOX 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_MAILBOX_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS MAILBOX Single bit error inject: starting \r\n");

    /* Run one shot test for MSS MAILBOX 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x72000000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS MAILBOX Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_MAILBOX_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_MAILBOX_2BitInjectTest
 *
 * @brief   Execute ECC MSS MAILBOX 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_MAILBOX_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS MAILBOX Double bit error inject: starting \r\n");

    /* Run one shot test for MSS MAILBOX 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x72000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS MAILBOX Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_MAILBOX_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_TPTC_A0_1BitInjectTest
 *
 * @brief   Execute ECC MSS TPTC A0 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_TPTC_A0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS TPTC A0 Single bit error inject: starting \r\n");

    /* Run one shot test for MSS TPTC A0 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS TPTC A0 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_TPTC_A0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_TPTC_A0_2BitInjectTest
 *
 * @brief   Execute ECC MSS TPTC A0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_TPTC_A0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS TPTC A0 Double bit error inject: starting \r\n");

    /* Run one shot test for MSS TPTC A0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS TPTC A0 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_TPTC_A0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_TPTC_A1_1BitInjectTest
 *
 * @brief   Execute ECC MSS TPTC A1 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_TPTC_A1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS TPTC A1 Single bit error inject: starting \r\n");

    /* Run one shot test for MSS TPTC A1 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_TPTC_A1_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS TPTC A1 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_TPTC_A1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_MSS_TPTC_A1_2BitInjectTest
 *
 * @brief   Execute ECC MSS TPTC A1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_MSS_TPTC_A1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	DebugP_log("\r\nMSS TPTC A1 Double bit error inject: starting \r\n");

    /* Run one shot test for MSS TPTC A1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x0u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_SOC_ECC_AGGR,
                                 SDL_SOC_ECC_AGGR_TPTC_A1_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
		/* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nMSS TPTC A1 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_MSS_TPTC_A1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_DRAM0_1BitInjectTest
 *
 * @brief   Execute ECC ICSSM DRAM0 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_DRAM0_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM DRAM0 Single bit error inject: starting \r\n");

    /* Run one shot test for ICSSM DRAM0 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48000000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM DRAM0 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_DRAM0_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_DRAM0_2BitInjectTest
 *
 * @brief   Execute ECC ICSSM DRAM0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_DRAM0_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM DRAM0 Double bit error inject: starting \r\n");

    /* Run one shot test for ICSSM DRAM0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48000000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM DRAM0 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_DRAM0_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_DRAM1_1BitInjectTest
 *
 * @brief   Execute ECC ICSSM DRAM1 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_DRAM1_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM DRAM1 Single bit error inject: starting \r\n");

    /* Run one shot test for ICSSM DRAM1 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48002000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM DRAM1 Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_DRAM1_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_DRAM1_2BitInjectTest
 *
 * @brief   Execute ECC ICSSM DRAM1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_DRAM1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM DRAM1 Double bit error inject: starting \r\n");

    /* Run one shot test for ICSSM DRAM1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48002000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM1_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM DRAM1 Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_DRAM1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_PR1_PDSP0_IRAM_1BitInjectTest
 *
 * @brief   Execute ECC ICSSM PR1 PDSP0 IRAM 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_PR1_PDSP0_IRAM_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM PR1 PDSP0 IRAM Single bit error inject: starting \r\n");

    /* Run one shot test for ICSSM PR1 PDSP0 IRAM 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48034000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM PR1 PDSP0 IRAM Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_PR1_PDSP0_IRAM_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_PR1_PDSP0_IRAM_2BitInjectTest
 *
 * @brief   Execute ECC ICSSM PR1 PDSP0 IRAM 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_PR1_PDSP0_IRAM_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM PR1 PDSP0 IRAM Double bit error inject: starting \r\n");

    /* Run one shot test for ICSSM PR1 PDSP0 IRAM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48034000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP0_IRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM PR1 PDSP0 IRAM Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_PR1_PDSP0_IRAM_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_PR1_PDSP1_IRAM_1BitInjectTest
 *
 * @brief   Execute ECC ICSSM PR1 PDSP1 IRAM 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_PR1_PDSP1_IRAM_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM PR1_PDSP1_IRAM Single bit error inject: starting \r\n");

    /* Run one shot test for ICSSM PR1 PDSP1 IRAM 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48038000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM PR1 PDSP1 IRAM Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_PR1_PDSP1_IRAM_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_PR1_PDSP1_IRAM_2BitInjectTest
 *
 * @brief   Execute ECC ICSSM PR1 PDSP1 IRAM 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_PR1_PDSP1_IRAM_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM PR1 PDSP1 IRAM Double bit error inject: starting \r\n");

    /* Run one shot test for ICSSM PR1 PDSP1 IRAM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48038000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_PR1_PDSP1_IRAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM PR1 PDSP1 IRAM Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_PR1_PDSP1_IRAM_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_RAM_1BitInjectTest
 *
 * @brief   Execute ECC ICSSM RAM 1 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_RAM_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM RAM Single bit error inject: starting \r\n");

    /* Run one shot test for ICSSM RAM 1 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48010000u);

    injectErrorConfig.flipBitMask = 0x02;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM RAM Single bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_RAM_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_ICSSM_RAM_2BitInjectTest
 *
 * @brief   Execute ECC ICSSM RAM 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_ICSSM_RAM_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;

    DebugP_log("\r\nICSSM RAM Double bit error inject: starting \r\n");

    /* Run one shot test for ICSSM RAM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48010000u);

    injectErrorConfig.flipBitMask = 0x03;
    result = SDL_ECC_injectError(SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR,
                                 SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_RAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result != SDL_PASS ) {
       retVal = -1;
    } else {
        /* Access the memory where injection is expected */
        testLocationValue = injectErrorConfig.pErrMem[0];
        DebugP_log("\r\nICSSM RAM Double bit error inject at pErrMem = 0x%p and the value of pErrMem is 0x%p :test complete \r\n",
                   injectErrorConfig.pErrMem, testLocationValue);
    }

    return retVal;
}/* End of ECC_Test_run_ICSSM_RAM_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_1BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_1BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0U;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
	
	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Negative Single bit error inject: starting \r\n");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000510u);

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 1 bit error */
    injectErrorConfig.flipBitMask = 0x0u;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    if (result == SDL_PASS ) {
        retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Negative Single bit error inject at pErrMem = 0x%p:test complete",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_1BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case1_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 Negative case 1 single bit 
 *			inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case1_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0U;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;

	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Negative case 1 Single bit error inject: starting \r\n");
		
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000510u);

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 Negative case 1 single bit error */
    injectErrorConfig.flipBitMask = 0x0u;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result == SDL_PASS ) {
        retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Negative case 1 Single bit error inject at pErrMem = 0x%p:test complete",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case1_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case2_2BitInjectTest
 *
 * @brief   Execute ECC R5FSS0 CORE0 ATCM0 BANK0 Negative 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case2_2BitInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0U;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
	
	DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Negative case 2 double bit error inject: starting \r\n");
	
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x00000510u);

    /* Run one shot test for R5FSS0 CORE0 ATCM0 BANK0 Negative1 bit error */
    injectErrorConfig.flipBitMask = 0x80000000u;
    result = SDL_ECC_injectError(SDL_R5FSS0_CORE0_ECC_AGGR,
                                 SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    if (result == SDL_PASS ) {
        retVal = -1;
    } else {
        DebugP_log("\r\nR5FSS0 CORE0 ATCM0 BANK0 Negative case 2 double bit error inject at pErrMem = 0x%p:test complete",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case2_2BitInjectTest() */

/*********************************************************************
 * @fn      ECC_sdlFuncTest
 *
 * @brief   Execute ECC sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t ECC_sdlFuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;
    void *ptr = (void *)&arg;

    if (retVal == 0) {
        /* Initialize ESM module */

        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);


            retVal = -1;
        } else {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }

    if (retVal == 0U) {
        /*Init of R5F*/
        retVal = ECC_Test_R5F_CACHE_init();

        if (retVal != 0U)
        {
            DebugP_log("\r\nECC SDL ECC_Test_R5F_CACHE_init: unsuccessful \r\n");
            return SDL_EFAIL;
        }
    }

    if (retVal == 0) {
           DebugP_log("\r\nR5FSS0 CORE0 ITAG CACHE Test\r\n");
           /* Initialize ECC Memory */
           SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
           result = ECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_1BitInjectTest();
           if (result != SDL_PASS) {
               retVal = -1;
               DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_1BitInjectTest has failed...\r\n");
           }
       }
       if (retVal == 0) {
           DebugP_log("\r\nR5FSS0 CORE0 ITAG CACHE Test\r\n");
           /* Initialize ECC Memory */
           SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
           result = ECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_2BitInjectTest();
           if (result != SDL_PASS) {
               retVal = -1;
               DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ITAG_RAM0_2BitInjectTest has failed...\r\n");
           }
       }
       if (retVal == 0) {
           DebugP_log("\r\nR5FSS0 CORE0 ITAG CACHE Test\r\n");
           /* Initialize ECC Memory */
           SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
           result = ECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_1BitInjectTest();
           if (result != SDL_PASS) {
               retVal = -1;
               DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_1BitInjectTest has failed...\r\n");
           }
       }
       if (retVal == 0) {
           DebugP_log("\r\nR5FSS0 CORE0 ITAG CACHE Test\r\n");
           /* Initialize ECC Memory */
           SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
           result = ECC_Test_run_R5FSS0_CORE0_ITAG_RAM1_2BitInjectTest();
           if (result != SDL_PASS) {
               retVal = -1;
               DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_1BitInjectTest has failed...\r\n");
           }
       }
       if (retVal == 0) {
           DebugP_log("\r\nR5FSS0 CORE0 ITAG CACHE Test\r\n");
           /* Initialize ECC Memory */
           SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
           result = ECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_1BitInjectTest();
           if (result != SDL_PASS) {
               retVal = -1;
               DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_1BitInjectTest has failed...\r\n");
           }
       }
       if (retVal == 0) {
           DebugP_log("\r\nR5FSS0 CORE0 ITAG CACHE Test\r\n");
           /* Initialize ECC Memory */
           SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
           result = ECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_2BitInjectTest();
           if (result != SDL_PASS) {
               retVal = -1;
               DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ITAG_RAM2_2BitInjectTest has failed...\r\n");
           }
       }
       if (retVal == 0) {
           DebugP_log("\r\nR5FSS0 CORE0 ITAG CACHE Test\r\n");
           /* Initialize ECC Memory */
           SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
           result = ECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_1BitInjectTest();
           if (result != SDL_PASS) {
               retVal = -1;
               DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_1BitInjectTest has failed...\r\n");
           }
       }
       if (retVal == 0) {
           DebugP_log("\r\nR5FSS0 CORE0 ITAG CACHE Test\r\n");
           /* Initialize ECC Memory */
           SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
           result = ECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_2BitInjectTest();
           if (result != SDL_PASS) {
               retVal = -1;
               DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ITAG_RAM3_2BitInjectTest has failed...\r\n");
           }
       }

    if (retVal == 0) {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG CACHE Test\r\n");
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
        result = ECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_1BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_1BitInjectTest has failed...\r\n");
        }
    }

    if (retVal == 0) {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG CACHE Test\r\n");
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
        result = ECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_1BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_1BitInjectTest has failed...\r\n");
        }
    }
    if (retVal == 0) {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG CACHE Test\r\n");
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
        result = ECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_1BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_1BitInjectTest has failed...\r\n");
        }
    }
    if (retVal == 0) {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG CACHE Test\r\n");
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
        result = ECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_1BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_1BitInjectTest has failed...\r\n");
        }
    }

    if (retVal == 0) {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG CACHE Test\r\n");
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
        result = ECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_2BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_DTAG_RAM0_2BitInjectTest has failed...\r\n");
        }
    }
    if (retVal == 0) {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG CACHE Test\r\n");
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
        result = ECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_2BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_DTAG_RAM1_2BitInjectTest has failed...\r\n");
        }
    }
    if (retVal == 0) {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG CACHE Test\r\n");
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
        result = ECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_2BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_DTAG_RAM2_2BitInjectTest has failed...\r\n");
        }
    }
    if (retVal == 0) {
        DebugP_log("\r\nR5FSS0 CORE0 DTAG CACHE Test\r\n");
        /* Initialize ECC Memory */
        SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
        result = ECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_2BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_DTAG_RAM3_2BitInjectTest has failed...\r\n");
        }
    }

    if (retVal == 0) {
        /*Init of R5F*/
        retVal = ECC_Test_R5F_init();

        if (retVal != 0)
        {
            DebugP_log("\r\nECC SDL ECC_Test_R5F_init: unsuccessful \r\n");
            return SDL_EFAIL;
        }
    }

	if (retVal == 0) {

		result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {

		result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK1_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {

		result = ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK0_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_2BitInjectTest has failed... \r\n");
		}
   }

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B0TCM0_BANK1_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {

		result = ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK0_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_B1TCM0_BANK1_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {

		result = ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
				retVal = -1;
           DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_ATCM1_BANK0_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {

		result = ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_ATCM1_BANK1_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {

		result = ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK0_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_B0TCM1_BANK1_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {

		result = ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK0_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_2BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE1_B1TCM1_BANK1_1BitInjectTest has failed... \r\n");
		}
	}

	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_1BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_1BitInjectTest has failed... \r\n");
		}
	}
	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case1_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case1_2BitInjectTest has failed... \r\n");
		}
	}
	if (retVal == 0) {
		result = ECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case2_2BitInjectTest();
		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_R5FSS0_CORE0_ATCM0_BANK0_Neg_case2_2BitInjectTest has failed... \r\n");
		}
	}

    if (retVal == 0) {
        result = ECC_Test_run_VIM_1BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\ECC_Test_run_VIM_1BitInjectTest has failed... \r\n");
        }
    }

    if (retVal == 0) {
        result = ECC_Test_run_VIM_2BitInjectTest();
        /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
        gMsmcMemParityInterrupt = false;
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\ECC_Test_run_VIM_2BitInjectTest has failed... \r\n");
        }
    }

	if (retVal == 0) {
		/*Init of MCAN*/
		retVal = ECC_Test_MCAN_init();

		if (retVal != 0)
		{
			DebugP_log("\r\nECC SDL ECC_Test_MCAN1_init: unsuccessful \r\n");
			return SDL_EFAIL;
		}
	}
	if (retVal == 0)
	{
		result = ECC_Test_run_MCAN0_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_1BitInjectTest has failed... \r\n");
			/*Low priority MCAN0 interrupt */
		}

		result = ECC_Test_run_MCAN0_1Bit_N_ROWInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_1Bit_N_ROWInjectTest has failed... \r\n");
			/*Low priority MCAN0 interrupt */
		}

		result = ECC_Test_run_MCAN0_1Bit_Repeat_InjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_1Bit_Repeat_InjectTes thas failed... \r\n");
			/*Low priority MCAN0 interrupt */
		}

		result = ECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_1Bit_N_ROW_RepeatInjectTest has failed... \r\n");
			/* Low priority MCAN0 interrupt */
		}
	}
	if (retVal == 0) {
		result = ECC_Test_run_MCAN0_2BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_2BitInjectTest has failed.... \r\n");
			/*High priority MCAN0 interrupt */
		}

		result = ECC_Test_run_MCAN0_2Bit_N_ROWInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_2Bit_N_ROWInjectTest has failed... \r\n");
			/*High priority MCAN0 interrupt */
		}

		result = ECC_Test_run_MCAN0_2Bit_Repeat_InjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_2Bit_Repeat_InjectTest has failed... \r\n");
			/*High priority MCAN0 interrupt */
		}

		result = ECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN0_2Bit_N_ROW_RepeatInjectTest has failed... \r\n");
			/* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		result = ECC_Test_run_MCAN1_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN1_1BitInjectTest has failed... \r\n");
			/*Low priority MCAN1 interrupt */
		}

		result = ECC_Test_run_MCAN1_2BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN1_2BitInjectTest has failed... \r\n");
			/*Low priority MCAN1 interrupt */
		}

		result = ECC_Test_run_MCAN2_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN2_1BitInjectTest has failed... \r\n");
			/*Low priority MCAN2 interrupt */
		}
		
		result = ECC_Test_run_MCAN2_2BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN2_2BitInjectTest has failed... \r\n");
			/*Low priority MCAN2 interrupt */
		}
		
		result = ECC_Test_run_MCAN3_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN3_1BitInjectTest has failed... \r\n");
			/*Low priority MCAN3 interrupt */
		}
		
		result = ECC_Test_run_MCAN3_2BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MCAN3_2BitInjectTest has failed... \r\n");
			/*Low priority MCAN3 interrupt */
		}

	}
	if (retVal == 0) {
		/*Init of MSS*/
		retVal = ECC_Test_ICSSM_init();
		if (retVal != 0)
		{
			DebugP_log("\r\nECC SDL ECC_Test_ICSSM_init: unsuccessful \r\n");
			return SDL_EFAIL;
		}
	}

	if (retVal == 0) {
		
		result = ECC_Test_run_ICSSM_DRAM0_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_ICSSM_DRAM0_1BitInjectTest has failed... \r\n");
			/*Low priority MCAN1 interrupt */
		}
	}
	
    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_DRAM0_2BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_DRAM0_2BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_DRAM1_1BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_DRAM1_1BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_DRAM1_2BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_DRAM1_2BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_PR1_PDSP0_IRAM_1BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_PR1_PDSP0_IRAM_1BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_PR1_PDSP0_IRAM_2BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_PR1_PDSP0_IRAM_2BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_PR1_PDSP1_IRAM_1BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_PR1_PDSP1_IRAM_1BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_PR1_PDSP1_IRAM_2BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_PR1_PDSP1_IRAM_2BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_RAM_1BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_RAM_1BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

    if (retVal == 0) {

        result = ECC_Test_run_ICSSM_RAM_2BitInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nECC_Test_run_ICSSM_RAM_2BitInjectTest has failed... \r\n");
            /*Low priority MCAN1 interrupt */
        }
    }

	if (retVal == 0) {
		/*Init of MSS*/
		retVal = ECC_Test_MSS_init();
		if (retVal != 0)
		{
			DebugP_log("\r\nECC SDL ECC_Test_MSS_init: unsuccessful \r\n");
			return SDL_EFAIL;
		}
	}
	if (retVal == 0)
	{
       result = ECC_Test_run_MSS_L2_SLV0_1BitInjectTest();
       if (result != SDL_PASS) {
           retVal = -1;
           DebugP_log("\r\nECC_Test_run_MSS_L2_SLV0_1BitInjectTest has failed... \r\n");
           /* High priority MCAN0 interrupt */
       }
	}

	if (retVal == 0)
   {
	    /* Initialize ECC Memory */
	    SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV0_ECC_RAM_ID);
	    /*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
	    gMsmcMemParityInterrupt = false;

		result = ECC_Test_run_MSS_L2_SLV0_2BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_L2_SLV0_2BitInjectTest has failed... \r\n");
			/* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		result = ECC_Test_run_MSS_L2_SLV1_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_L2_SLV1_1BitInjectTest has failed... \r\n");
			/* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		/* Initialize ECC Memory */
		SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV1_ECC_RAM_ID);

		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;

		result = ECC_Test_run_MSS_L2_SLV1_2BitInjectTest();
		if (result != SDL_PASS) 
		{
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_L2_SLV1_2BitInjectTest has failed... \r\n");
           /* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		result = ECC_Test_run_MSS_L2_SLV2_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_L2_SLV2_1BitInjectTest has failed... \r\n");
			/* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		/* Initialize ECC Memory */
		SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID);

		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;

		result = ECC_Test_run_MSS_L2_SLV2_2BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_L2_SLV2_2BitInjectTest has failed... \r\n");
			/* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		result = ECC_Test_run_MSS_L2_SLV3_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_L2_SLV3_1BitInjectTest has failed... \r\n");
			/* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		/* Initialize ECC Memory */
		SDL_ECC_initMemory(SDL_SOC_ECC_AGGR, SDL_SOC_ECC_AGGR_MSS_L2_SLV3_ECC_RAM_ID);

		/*Clear the global variable before ECC error injecting , in case ESM callback occurred due to any other operation*/
		gMsmcMemParityInterrupt = false;

		result = ECC_Test_run_MSS_L2_SLV3_2BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_L2_SLV3_2BitInjectTest has failed... \r\n");
			/* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		result = ECC_Test_run_MSS_MAILBOX_1BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_MAILBOX_1BitInjectTest has failed... \r\n");
           /* High priority MCAN0 interrupt */
		}
	}

	if (retVal == 0)
	{
		result = ECC_Test_run_MSS_MAILBOX_2BitInjectTest();
		if (result != SDL_PASS) {
			retVal = -1;
			DebugP_log("\r\nECC_Test_run_MSS_MAILBOX_2BitInjectTest has failed... \r\n");
			/* High priority MCAN0 interrupt */
		}
	}
	
    if ( retVal == 0) {
        DebugP_log("\r\nECC SDL API tests: success\r\n");
    } else {
        DebugP_log("\r\nECC SDL API tests: failed\r\n");
    }

    return retVal;
}

/* ECC Function module test */
int32_t ECC_sdl_funcTest(void)
{
    int32_t testResult = 0;
	
	/* Initialize ECC callbacks within the Main ESM */
	testResult = SDL_ECC_initEsm(SDL_ESM_INST_MAIN_ESM0);
	if (testResult != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing ECC callback for MSS ESM: result = %d\r\n", testResult);

		testResult = -1;
	} else {
		DebugP_log("\r\nECC_Test_init: ECC Callback Init complete for MSS ESM \r\n");
	}
	
    /*Execute ECC sdl function test*/
    testResult = ECC_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
