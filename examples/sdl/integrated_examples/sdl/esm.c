/*
 *   Copyright (c) 2022-2024 Texas Instruments Incorporated
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
 *  \file     esm.c
 *
 *  \brief    Application interface for the ESM.
 *
 *  \details
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include "sdlexample.h"
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* This macro shows how many ESM events are configured*/
#define SDL_MCANA_MAX_MEM_SECTIONS                  (1u)
#define SDL_ICSSM_MAX_MEM_SECTIONS                  (1u)
#define SDL_MSS_MAX_MEM_SECTIONS                    (2u)
#define SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS           (2u)
#define SDL_EXAMPLE_ECC_RAM_ADDR    (0x52600000u) /* MCAN0 address */
#define SDL_EXAMPLE_ECC_AGGR                              SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR
#define SDL_EXAMPLE_ECC_RAM_ID                            SDL_MCAN0_MCANSS_MSGMEM_WRAP_ECC_AGGR_MCANSS_MSGMEM_WRAP_MSGMEM_ECC_RAM_ID
#define SDL_EXAMPLE_ECC_ICSSM_AGGR                        SDL_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR
#define SDL_EXAMPLE_ECC_ICSSM_RAM_ID                      SDL_PRU_ICSSM_ICSS_G_CORE_BORG_ECC_AGGR_ICSS_G_CORE_DRAM0_ECC_RAM_ID
#define SDL_EXAMPLE_ECC_MSS_AGGR                          SDL_SOC_ECC_AGGR
#define SDL_EXAMPLE_ECC_MSSL2_RAM_ID                      SDL_SOC_ECC_AGGR_MSS_L2_SLV2_ECC_RAM_ID
#define SDL_EXAMPLE_ECC_ATCM_BTCM_AGGR                    SDL_R5FSS0_CORE0_ECC_AGGR
#define SDL_EXAMPLE_ECC_ATCM_RAM_ID                       SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_ATCM0_BANK0_RAM_ID
#define SDL_EXAMPLE_ECC_BTCM_RAM_ID                       SDL_R5FSS0_CORE0_ECC_AGGR_PULSAR_SL_B1TCM0_BANK1_RAM_ID
#define SDL_EXAMPLE_ECC_TPTC_RAM_ID                       SDL_SOC_ECC_AGGR_TPTC_A0_ECC_RAM_ID
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
/*                 Internal Function Declarations                             */
/* ========================================================================== */
static SDL_ECC_MemSubType ECC_Test_MCANA_subMemTypeList[SDL_MCANA_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MCANA_ECCInitConfig =
{
    .numRams = SDL_MCANA_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MCANA_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_ICSSM_subMemTypeList[SDL_ICSSM_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_ICSSM_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_ICSSM_ECCInitConfig =
{
    .numRams = SDL_ICSSM_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_ICSSM_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_MSS_L2_subMemTypeList[SDL_MSS_MAX_MEM_SECTIONS] =
{
     SDL_EXAMPLE_ECC_MSSL2_RAM_ID,
     SDL_EXAMPLE_ECC_TPTC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_MSS_ECCInitConfig =
{
    .numRams = SDL_MSS_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_MSS_L2_subMemTypeList[0]),
    /**< Sub type list  */
};

static SDL_ECC_MemSubType ECC_Test_R5FSS0_CORE0_subMemTypeList[SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS] =
{
    SDL_EXAMPLE_ECC_BTCM_RAM_ID,
    SDL_EXAMPLE_ECC_ATCM_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_R5FSS0_CORE0_ECCInitConfig =
{
    .numRams = SDL_R5FSS0_CORE0_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_R5FSS0_CORE0_subMemTypeList[0]),
    /**< Sub type list  */
};

/*
** application argument passed back to the callback.
** Not used by this example.
*/
volatile static uint32_t esmcbarg = 0;

/* defines for setting the bitmasks for the ESM strcuure we pass to SDL */
#define ESM_ENABLE_BITMAP_RTI_G2      0x00000001u     /* RTI */
#define ESM_PRIORITY_RTI_G0           0x00000001u
#define ESM_ERRORP_RTI_G2             0x00000001u

#define ESM_ENABLE_BITMAP_MCAN0ECC_G0 0x0000000cu     /* MCAN0 correctable or not ECC  */
#define ESM_PRIORITY_MCAN0ECC_G0      0x00000008u
#define ESM_ERRORP_MCAN0ECC_G0        0x0000000cu

#define ESM_ENABLE_BITMAP_ICSSMECC_G2 0x00006000u     /* ICSSM correctable or not ECC  */
#define ESM_PRIORITY_ICSSMECC_G2      0x00004000u
#define ESM_ERRORP_ICSSMECC_G2        0x00006000u

#define ESM_ENABLE_BITMAP_MSSL2ECC_G0 0x00180000u     /* MSSL2 correctable or not ECC  */
#define ESM_PRIORITY_MSSL2ECC_G0      0x00180000u
#define ESM_ERRORP_MSSL2ECC_G0        0x00180000u

#define ESM_ENABLE_BITMAP_R5F0ECC_G1  0x00018000u     /* R5F0 correctable or not ECC  */
#define ESM_PRIORITY_R5F0ECC_G1       0x00010000u
#define ESM_ERRORP_R5F0ECC_G1         0x00018000u

#define ESM_ENABLE_BITMAP_DCC_G0      0x01E00000u     /* DCC  */
#define ESM_PRIORITY_DCC_G0           0x01E00000u
#define ESM_ERRORP_DCC_G0             0x01E00000u

#define ESM_ENABLE_BITMAP_PARITYTCM_G0 0x0003C000u     /* Parity - TCM */
#define ESM_ENABLE_BITMAP_PARITYTCM_G2 0x00000010u
#define ESM_PRIORITY_PARITYTCM_G0      0x0000C000u
#define ESM_PRIORITY_PARITYTCM_G2      0x00000010u
#define ESM_ERRORP_PARITYTCM_G2        0x00000010u
#define ESM_ERRORP_PARITYTCM_G0        0x0003C000u

#define ESM_ENABLE_BITMAP_PARITYDMA_G1 0x80000000u     /* Parity - DMA */
#define ESM_ENABLE_BITMAP_PARITYDMA_G2 0x00000010u
#define ESM_PRIORITY_PARITYDMA_G1      0x80000000u
#define ESM_PRIORITY_PARITYDMA_G2      0x00000010u
#define ESM_ERRORP_PARITYDMA_G2        0x00000010u
#define ESM_ERRORP_PARITYDMA_G1        0x80000000u

#define ESM_ENABLE_BITMAP_ECCBUSC_G1   0x00000002u     /* ECC BUS */
#define ESM_ENABLE_BITMAP_BUSSAFETY_G0 0x80000000u
#define ESM_ERRORP_ECCBUS_G1           0x00000002u
#define ESM_ERRORP_ECCBUS_G0           0x80000000u

#define ESM_ENABLE_BITMAP_CCM_G2       0x00780000u     /* CCM */
#define ESM_PRIORITY_CCM_G2            0x00780000u
#define ESM_ERRORP_CCM_G2              0x00780000u

#define ESM_ENABLE_BITMAP_TPTC_G0      0x00180000u     /* ECC MSS TPTC */
#define ESM_PRIORITY_TPTC_G0           0x00180000u
#define ESM_ERRORP_TPTC_G2             0x00180000u

#define ESM_ENABLE_BITMAP_VTM_G1       0x00001000u     /* VTM */
#define ESM_PRIORITY_VTM_G1            0x00001000u
#define ESM_ERRORP_VTM_G1              0x00001000u

#define ESM_ENABLE_BITMAP_PARITYTMU_G2 0x01000000u     /* TMU PARITY */
#define ESM_PRIORITY_PARITYTMU_G2      0x01000000u
#define ESM_ERRORP_PARITYTMU_G2        0x01000000u

/* we will configure this  in the initlaization function for the ESM */
#define ESM_NOT_CONFIGURED            0x00000000u    /* used when not configuring any values for the ESM */

static SDL_ESM_config Example_esmInitConfig =
{
    /* Self test error config - not used by this example */
     .esmErrorConfig = {1u, 8u},
     /*  All events enable: except clkstop events for unused clocks and PCIE events */
     .enableBitmap    = {ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED , ESM_NOT_CONFIGURED,
                         ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED , ESM_NOT_CONFIGURED},
     /** All events high priority: except clkstop events for unused clocks and PCIE events */
     .priorityBitmap  = {ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED,
                         ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED},
    /* All events high priority: except clkstop for unused clocks  and PCIE events */
     .errorpinBitmap  = {ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED,
                         ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED, ESM_NOT_CONFIGURED},
};


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*********************************************************************
*  SDL_ESM_applicationCallbackFunction
*
*  return  0 : Success; < 0 for failures
**********************************************************************/
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void     *arg)
{
    int32_t             retVal;

    /* save off who called us */
    sdlstats.esmcb            = ESMCB_UNKNOWN; /* catch all -will  change below */
    sdlstats.esm.esmInst      = esmInst;
    sdlstats.esm.esmIntrType  = esmIntrType;
    sdlstats.esm.grpChannel   = grpChannel;
    sdlstats.esm.index        = index;
    sdlstats.esm.intSrc       = intSrc;

    retVal = SDL_PASS;

    if ( (intSrc == ESM_INT_MCAN0_ECC_CORRECTABLE) || (intSrc == ESM_INT_MCAN0_ECC_NOT_CORRECTABLE) )
    {
        /* MCAN ECC */
        sdlstats.esmcb = ESMCB_ECC;
        ecc_clearESM();
    }
    if ( (intSrc == ESM_INT_ICSSM_ECC_CORRECTABLE) || (intSrc ==  ESM_INT_ICSSM_ECC_UNCORRECTABLE) )
    {
        /* MCAN ECC */
        sdlstats.esmcb = ESMCB_ECC;
        ecc_icssm_clearESM();
    }
    if ( (intSrc == ESM_INT_MSSL2_ECC_CORRECTABLE) || (intSrc ==  ESM_INT_MSSL2_ECC_UNCORRECTABLE) )
    {
        /* MCAN ECC */
        sdlstats.esmcb = ESMCB_ECC;
        ecc_mssl2_clearESM();

        sdlstats.esmcb = ESMCB_TPTC_ECC;
        ecc_tptc_clearESM();
    }
    if ( (intSrc == ESM_INT_R5F0_ECC_CORRECTABLE) || (intSrc ==  ESM_INT_R5F0_ECC_UNCORRECTABLE) )
    {
        /* ATCM ECC */
        sdlstats.esmcb = ESMCB_ATCM_ECC;
        ecc_atcm_clearESM();
    
        /* BTCM ECC */
        sdlstats.esmcb = ESMCB_BTCM_ECC;
        ecc_btcm_clearESM();
    }
    else if ( (intSrc >= ESM_INT_DCC0) && (intSrc <= ESM_INT_DCC3) )
    {
        /* DCC */
        sdlstats.esmcb = ESMCB_DCC;
        dcc_clearESM();
    }
    else if ( (intSrc >= ESM_INT_TCMADDR_R500) && (intSrc <= ESM_INT_TCMADDR_R511) )
    {
        /* TCM Parity */
        sdlstats.esmcb = ESMCB_PARITYTCM ;
         ParityTCM_clear();
    }
    else if ((intSrc == ESM_INT_BUSSAFETY1) || (intSrc == ESM_INT_BUSSAFETY2))
    {
        /* ECC Bus Safety */
        sdlstats.esmcb = ESMCB_ECCBUS;
        EccBusSafety_clearESM();
    }
    else if (intSrc == ESM_INT_EDMA0_TPCC_ERRINTAGG)
    {
        /* DMA Parity */
        sdlstats.esmcb = ESMCB_PARITYDMA ;
        ParityDMA_clear();
    }
    else if (intSrc == ESM_INT_RTI)
    {
        /* RTI */
        sdlstats.esmcb = ESMCB_RTI;
        rti_clearESM(intSrc);
    }
    else if ((intSrc == ESM_INT_CCM0_LOCKSTEP) || (intSrc == ESM_INT_CCM0_SELFTEST) || (intSrc == ESM_INT_CCM1_SELFTEST) || (intSrc == ESM_INT_CCM1_LOCKSTEP))
    {
        sdlstats.esmcb = ESMCB_CCM;
        ccm_clearESM(intSrc);
    }
    #if defined (SOC_AM263PX)
    else if (intSrc == ESM_INT_VTM)
    {
        sdlstats.esmcb = ESMCB_VTM;
        VTM_clear();

    }
    else if (intSrc == ESM_INT_TMU_PARITY)
    {
        sdlstats.esmcb = ESMCB_TMUPARITY;
        tmu_parity_clearESM();

    }
    #endif
    /* if we are running diagnostics clear this */
    if (runningDiags())
    {
      sdlstats.esmcb = ESMCB_NONE;
    }

    return retVal;
}


/*********************************************************************
*   ESM_init
*
* return  0 : Success; < 0 for failures
**********************************************************************/
int32_t ESM_init (void)
{
    void          *ptr = (void *)&esmcbarg;
    SDL_ErrType_t result;


    /*
    ** We need to configure the ESM for all of the events we wish to see.
    */


    Example_esmInitConfig.enableBitmap[0]   = (ESM_ENABLE_BITMAP_BUSSAFETY_G0 | ESM_ENABLE_BITMAP_MCAN0ECC_G0 | ESM_ENABLE_BITMAP_DCC_G0 | ESM_ENABLE_BITMAP_PARITYTCM_G0 | ESM_ENABLE_BITMAP_MSSL2ECC_G0 | ESM_ENABLE_BITMAP_TPTC_G0);
    Example_esmInitConfig.enableBitmap[1]   = (ESM_ENABLE_BITMAP_ECCBUSC_G1 | ESM_ENABLE_BITMAP_PARITYDMA_G1 | ESM_ENABLE_BITMAP_R5F0ECC_G1 | ESM_ENABLE_BITMAP_VTM_G1);
    Example_esmInitConfig.enableBitmap[2]   = (ESM_ENABLE_BITMAP_RTI_G2 | ESM_ENABLE_BITMAP_PARITYTCM_G2 | ESM_ENABLE_BITMAP_CCM_G2 | ESM_ENABLE_BITMAP_PARITYDMA_G2 | ESM_ENABLE_BITMAP_ICSSMECC_G2 | ESM_ENABLE_BITMAP_PARITYTMU_G2);

    Example_esmInitConfig.priorityBitmap[0] = (ESM_PRIORITY_RTI_G0 | ESM_PRIORITY_MCAN0ECC_G0  | ESM_PRIORITY_DCC_G0 | ESM_PRIORITY_PARITYTCM_G0 |  ESM_ERRORP_ECCBUS_G0 | ESM_PRIORITY_MSSL2ECC_G0 | ESM_PRIORITY_TPTC_G0);
    Example_esmInitConfig.priorityBitmap[1] = (ESM_ERRORP_ECCBUS_G1 | ESM_PRIORITY_PARITYDMA_G1 | ESM_PRIORITY_R5F0ECC_G1 | ESM_PRIORITY_VTM_G1);
    Example_esmInitConfig.priorityBitmap[2] = (ESM_PRIORITY_PARITYTCM_G2 | ESM_PRIORITY_CCM_G2 | ESM_PRIORITY_PARITYDMA_G2 | ESM_PRIORITY_ICSSMECC_G2 | ESM_PRIORITY_PARITYTMU_G2);
    #if defined(SOC_AM263PX)
    Example_esmInitConfig.enableBitmap[1]   = (ESM_ENABLE_BITMAP_VTM_G1);
    Example_esmInitConfig.enableBitmap[2]   = (ESM_ENABLE_BITMAP_PARITYTMU_G2);
    Example_esmInitConfig.priorityBitmap[1] = (ESM_PRIORITY_VTM_G1);
    Example_esmInitConfig.priorityBitmap[2] = (ESM_PRIORITY_PARITYTMU_G2);
    #endif
/***
    We will leave the Safety LED Off on the EVM (LD016).  If we detect an ESM event that is not ecpected we will
    light it up. The SDL diagnostics tend to trigger ESM events and we dont want the LED to light up unless there
    is a failure.
    Example_esmInitConfig.errorpinBitmap[0] = (ESM_ERRORP_MCAN0ECC_G0  | ESM_PRIORITY_DCC_G0);
***/

    /* Initialize ECC Memory */
    result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_AGGR, SDL_EXAMPLE_ECC_RAM_ID);
    if (result != SDL_PASS)
    {
      return result;
    }
    /* Initialize ICSSM ECC Memory */
    result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_ICSSM_AGGR, SDL_EXAMPLE_ECC_ICSSM_RAM_ID);
    if (result != SDL_PASS)
    {
      return result;
    }
    /* Initialize MSSL2 ECC Memory */
    result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_MSS_AGGR, SDL_EXAMPLE_ECC_MSSL2_RAM_ID);
    if (result != SDL_PASS)
    {
      return result;
    }

    result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_MSS_AGGR, SDL_EXAMPLE_ECC_TPTC_RAM_ID);
    if (result != SDL_PASS)
    {
      return result;
    }

    /* Initialise exception handler */
    ECC_Test_exceptionInit();
    
    /* Initialize R5F0 ECC Memory ATCM */
    result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_ATCM_BTCM_AGGR, SDL_EXAMPLE_ECC_ATCM_RAM_ID);
    if (result != SDL_PASS)
    {
      return result;
    }
    /* Initialize R5F0 ECC Memory BTCM */
    result = SDL_ECC_initMemory(SDL_EXAMPLE_ECC_ATCM_BTCM_AGGR, SDL_EXAMPLE_ECC_BTCM_RAM_ID);
    if (result != SDL_PASS)
    {
      return result;
    }


    /* Initialize ESM module */
    result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &Example_esmInitConfig, SDL_ESM_applicationCallbackFunction, ptr);
    if (result != SDL_PASS)
    {
      return result;
    }

    /* Initialize ECC */
    result = SDL_ECC_init(SDL_EXAMPLE_ECC_AGGR, &ECC_Test_MCANA_ECCInitConfig);
    /* Initialize ICSSM ECC */
    result = SDL_ECC_init(SDL_EXAMPLE_ECC_ICSSM_AGGR, &ECC_Test_ICSSM_ECCInitConfig);
    /* Initialize MSSL2 ECC */
    result = SDL_ECC_init(SDL_EXAMPLE_ECC_MSS_AGGR, &ECC_Test_MSS_ECCInitConfig);
    /* Initialize R5F0 ECC */
    result = SDL_ECC_init(SDL_EXAMPLE_ECC_ATCM_BTCM_AGGR, &ECC_Test_R5FSS0_CORE0_ECCInitConfig);
    return result;
}

/* Nothing past this point */
