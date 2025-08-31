/* Copyright (c) 2022-2025 Texas Instruments Incorporated
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
 *  \file     main.c
 *
 *  \brief    This file contains PBIST example code.
 *
 *  \details  PBIST app
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <dpl_interface.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <sdl/sdl_pbist.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the application interface */


/**
 *  \brief PBIST configuration parameter structure.
 */


/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define PBIST_INSTANCE_NAME_MAX_LENGTH    20
#define APP_PBIST_TIMEOUT   (100000000U)
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#if defined (R5F0_INPUTS)
#define SDL_INTR_NUM SDL_R5FSS0_CORE0_INTR_PBIST_DONE
#elif defined (R5F1_INPUTS)
#define SDL_INTR_NUM SDL_R5FSS1_CORE0_INTR_PBIST_DONE
#endif
#endif
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/


/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#endif
typedef struct PBIST_TestHandle_s
{
    char testName[PBIST_INSTANCE_NAME_MAX_LENGTH];
    SDL_PBIST_inst pbistInst;
    SDL_pbistRegs *pPBISTRegs;
    uint32_t numPBISTRuns;
    SDL_PBIST_configNeg PBISTNegConfigRun;
    uint32_t interruptNumber;
    volatile bool doneFlag;
} PBIST_TestHandle_t;

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (R5F0_INPUTS)
PBIST_TestHandle_t PBIST_TestHandleArray[2] =
{
    {
        .testName               = "TOP PBIST",
        .pbistInst              = SDL_PBIST_INST_TOP,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_MSS_INTR_TOP_PBIST_DONE_INT,
        .doneFlag               = false,                /* Initialize done flag  */
    },
    {
        .testName               = "DSP PBIST",
        .pbistInst              = SDL_PBIST_INST_DSS,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_DSS_DSP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_MSS_INTR_DSS_DSP_PBIST_CTRL_DONE,
        .doneFlag               = false,                /* Initialize done flag  */
    },

};
#elif defined (R5F1_INPUTS)
  PBIST_TestHandle_t PBIST_TestHandleArray[1] =
  {
    {
        .testName               = "TOP PBIST",
        .pbistInst              = SDL_PBIST_INST_TOP,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_MSS_INTR_TOP_PBIST_DONE_INT,
        .doneFlag               = false,                /* Initialize done flag  */
    },
};
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#if defined R5F0_INPUTS
PBIST_TestHandle_t PBIST_TestHandleArray[1] =
{
    {
        .testName               = "TOP PBIST",
        .pbistInst              = SDL_PBIST_INST_TOP,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_R5FSS0_CORE0_INTR_PBIST_DONE,
        .doneFlag               = false,                /* Initialize done flag  */
    },
};
#elif defined R5F1_INPUTS
PBIST_TestHandle_t PBIST_TestHandleArray[1] =
{
    {
        .testName               = "TOP PBIST",
        .pbistInst              = SDL_PBIST_INST_TOP,
        .pPBISTRegs             = (SDL_pbistRegs *)SDL_TOP_PBIST_U_BASE,
        .numPBISTRuns           = 1u,
        .interruptNumber        = SDL_R5FSS1_CORE0_INTR_PBIST_DONE,
        .doneFlag               = false,                /* Initialize done flag  */
    },
};
#endif
#endif

int32_t PBIST_runTest(uint32_t instanceId, bool runNegTest)
{
    int32_t testResult = 0;
    SDL_ErrType_t status;
    bool PBISTResult;
    SDL_PBIST_testType testType;

    uint64_t testStartTime , testEndTime;
    uint64_t diffTime;

#if defined (SOC_AM273X) || (SOC_AWR294X)
    if (instanceId == SDL_PBIST_INST_TOP)
    {
        if (runNegTest == true)
        {
            {
                DebugP_log("\n Starting PBIST failure insertion test on TOP PBIST\r\n",
                            PBIST_TestHandleArray[instanceId].testName,
                            instanceId);
            }
            testType = SDL_PBIST_NEG_TEST;
        }
        else
        {
            {
                DebugP_log("\n Starting PBIST test on TOP PBIST\r\n",
                            PBIST_TestHandleArray[instanceId].testName,
                            instanceId);
            }
            testType = SDL_PBIST_TEST;
        }
    }
    if (instanceId == SDL_PBIST_INST_DSS)
    {
        if (runNegTest == true)
        {
            {
                DebugP_log("\n Starting PBIST failure insertion test on DSP PBIST\r\n",
                            PBIST_TestHandleArray[instanceId].testName,
                            instanceId);
            }
            testType = SDL_PBIST_NEG_TEST;
        }
        else
        {
            {
                DebugP_log("\n Starting PBIST test on DSP PBIST\r\n",
                            PBIST_TestHandleArray[instanceId].testName,
                            instanceId);
            }
            testType = SDL_PBIST_TEST;
        }
    }
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    if (runNegTest == true)
    {
        {
            DebugP_log("\n Starting PBIST failure insertion test on TOP PBIST\r\n",
                        PBIST_TestHandleArray[instanceId].testName,
                        instanceId);
        }
        testType = SDL_PBIST_NEG_TEST;
    }
    else
    {
        {
            DebugP_log("\n Starting PBIST test on TOP PBIST\n",
                        PBIST_TestHandleArray[instanceId].testName,
                        instanceId);
        }
        testType = SDL_PBIST_TEST;
    }

#endif



    /* Get start time of test */
    testStartTime = ClockP_getTimeUsec();

    status = SDL_PBIST_selfTest((SDL_PBIST_inst)PBIST_TestHandleArray[instanceId].pbistInst, testType, APP_PBIST_TIMEOUT, &PBISTResult);
    if ((status != SDL_PASS) || (PBISTResult == false))
    {
        testResult = -1;
    }

    /* Record test end time */
    testEndTime = ClockP_getTimeUsec();

    diffTime = testEndTime-testStartTime;
#if defined (SOC_AWR294X)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        #if defined (R5F0_INPUTS)
        {
            if (instanceId == SDL_PBIST_INST_TOP)
            {

                DebugP_log(" PBIST complete for ADCBUF\r\n");
                DebugP_log(" PBIST complete for TPCC\r\n");
                DebugP_log(" PBIST complete for MAILBOX\r\n");
                DebugP_log(" PBIST complete for COREB VIM\r\n");
                DebugP_log(" PBIST complete for MCAN\r\n");
                DebugP_log(" PBIST complete for SPIA\r\n");
                DebugP_log(" PBIST complete for SPIB\r\n");
                DebugP_log(" PBIST complete for CORE B R5FSS RAM\r\n");
                DebugP_log(" PBIST complete for MSS_L2_1\r\n");
                DebugP_log(" PBIST complete for CPSW\r\n");
                DebugP_log(" PBIST complete for GPADC\r\n");
                DebugP_log(" PBIST complete for RETRAM\r\n");
                DebugP_log(" PBIST complete for STCROM\r\n");
                DebugP_log(" PBIST complete for CORE B ATCM\r\n");
                DebugP_log(" PBIST complete for CORE B BTCM\r\n");

            }
            else if (instanceId == SDL_PBIST_INST_DSS)
            {
                DebugP_log(" PBIST complete for DSS C66 STCROM\r\n");
                DebugP_log(" PBIST complete for HWA STCROM\r\n");
                DebugP_log(" PBIST complete for DSS PBISTROM\r\n");
                DebugP_log(" PBIST complete for C66 L1D\r\n");
                DebugP_log(" PBIST complete for C66 L1P\r\n");
                DebugP_log(" PBIST complete for PBIST C66 L2 TAG\r\n");
                DebugP_log(" PBIST complete for DSS HWA\r\n");
                DebugP_log(" PBIST complete for DSS HWA MBOX\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKA SUB0\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB0\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKC SUB0\r\n");
                DebugP_log(" PBIST complete for DSS MBOX RAM\r\n");
                DebugP_log(" PBIST complete for DSS TPCC RAM\r\n");
                DebugP_log(" PBIST complete for DSS L2 BANK0\r\n");
                DebugP_log(" PBIST complete for DSS L2 BANK1\r\n");
                DebugP_log(" PBIST complete for DSS L2 PARITY\r\n");
                DebugP_log(" PBIST complete for HWA RAM\r\n");
                DebugP_log(" PBIST complete for DSS CBUF\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKA SUB1\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB1\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB2\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKC SUB1\r\n");

            }
        }
        #elif defined (R5F1_INPUTS)
        {
            DebugP_log(" PBIST complete for MSS_TCMAROM_0\r\n");
            DebugP_log(" PBIST complete for MSS_TCMAROM_1\r\n");
            DebugP_log(" PBIST complete for PBISTROM\r\n");
            DebugP_log(" PBIST complete for CORE A VIM\r\n");
            DebugP_log(" PBIST complete for MSS_L2_0\r\n");
            DebugP_log(" PBIST complete for CORE A ATCM\r\n");
            DebugP_log(" PBIST complete for CORE A BTCM\r\n");
            DebugP_log(" PBIST complete for CORE A R5SS RAM\r\n");
            DebugP_log(" PBIST complete for MEM_TOP_AURORA\r\n");
            DebugP_log(" PBIST complete for MEM_TOP_MDO\r\n");
            DebugP_log(" PBIST complete for DBGSS_TRACE\r\n");
        }
        #endif
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\nAll tests have passed. \r\n");
        }
        else
        {
            DebugP_log("\r\nSome tests have failed. \r\n");
        }
    }
#endif
#if defined (SOC_AM273X)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        #if defined (R5F0_INPUTS)
        {
            if (instanceId == SDL_PBIST_INST_TOP)
            {
                DebugP_log(" PBIST complete for TPCC\r\n");
                DebugP_log(" PBIST complete for MAILBOX\r\n");
                DebugP_log(" PBIST complete for COREB VIM\r\n");
                DebugP_log(" PBIST complete for MCAN\r\n");
                DebugP_log(" PBIST complete for SPIA\r\n");
                DebugP_log(" PBIST complete for SPIB\r\n");
                DebugP_log(" PBIST complete for CORE B R5FSS RAM\r\n");
                DebugP_log(" PBIST complete for MSS_L2_1\r\n");
                DebugP_log(" PBIST complete for CPSW\r\n");
                DebugP_log(" PBIST complete for GPADC\r\n");
                DebugP_log(" PBIST complete for RETRAM\r\n");
                DebugP_log(" PBIST complete for STCROM\r\n");
                DebugP_log(" PBIST complete for CORE B ATCM\r\n");
                DebugP_log(" PBIST complete for CORE B BTCM\r\n");

            }
            else if (instanceId == SDL_PBIST_INST_DSS)
            {
                DebugP_log(" PBIST complete for DSS C66 STCROM\r\n");
                DebugP_log(" PBIST complete for HWA STCROM\r\n");
                DebugP_log(" PBIST complete for DSS PBISTROM\r\n");
                DebugP_log(" PBIST complete for C66 L1D\r\n");
                DebugP_log(" PBIST complete for C66 L1P\r\n");
                DebugP_log(" PBIST complete for PBIST C66 L2 TAG\r\n");
                DebugP_log(" PBIST complete for DSS HWA\r\n");
                DebugP_log(" PBIST complete for DSS HWA MBOX\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKA SUB0\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB0\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKC SUB0\r\n");
                DebugP_log(" PBIST complete for DSS MBOX RAM\r\n");
                DebugP_log(" PBIST complete for DSS TPCC RAM\r\n");
                DebugP_log(" PBIST complete for DSS L2 BANK0\r\n");
                DebugP_log(" PBIST complete for DSS L2 BANK1\r\n");
                DebugP_log(" PBIST complete for DSS L2 PARITY\r\n");
                DebugP_log(" PBIST complete for HWA RAM\r\n");
                DebugP_log(" PBIST complete for DSS CBUF\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKA SUB1\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB1\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB2\r\n");
                DebugP_log(" PBIST complete for PBIST DSS L3 BANKC SUB1\r\n");

            }
        }
        #elif defined (R5F1_INPUTS)
        {
            DebugP_log(" PBIST complete for MSS_TCMAROM_0\r\n");
            DebugP_log(" PBIST complete for MSS_TCMAROM_1\r\n");
            DebugP_log(" PBIST complete for PBISTROM\r\n");
            DebugP_log(" PBIST complete for CORE A VIM\r\n");
            DebugP_log(" PBIST complete for MSS_L2_0\r\n");
            DebugP_log(" PBIST complete for CORE A ATCM\r\n");
            DebugP_log(" PBIST complete for CORE A BTCM\r\n");
            DebugP_log(" PBIST complete for CORE A R5SS RAM\r\n");
            DebugP_log(" PBIST complete for MEM_TOP_AURORA\r\n");
            DebugP_log(" PBIST complete for MEM_TOP_MDO\r\n");
            DebugP_log(" PBIST complete for DBGSS_TRACE\r\n");
        }
        #endif
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\nAll tests have passed. \r\n");
        }
        else
        {
            DebugP_log("\r\nSome tests have failed. \r\n");
        }
    }
#endif
#if defined (SOC_AM263X)
#if defined (R5F0_INPUTS)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        {
            DebugP_log(" PBIST complete for R5 STC\r\n");
            DebugP_log(" PBIST complete for R51 STC\r\n");
            DebugP_log(" PBIST complete for PBISTROM\r\n");
            DebugP_log(" PBIST complete for CPSW\r\n");
            DebugP_log(" PBIST complete for ICSSM\r\n");
            DebugP_log(" PBIST complete for MBOX\r\n");
            DebugP_log(" PBIST complete for MCAN\r\n");
            DebugP_log(" PBIST complete for TPCC\r\n");
            DebugP_log(" PBIST complete for MSS_L2_1\r\n");
            DebugP_log(" PBIST complete for MSS_L2_2\r\n");
            DebugP_log(" PBIST complete for MSS_L2_3\r\n");
            DebugP_log(" PBIST complete for VIM0 R5SS0\r\n");
            DebugP_log(" PBIST complete for VIM1 R5SS0\r\n");
            DebugP_log(" PBIST complete for VIM0 R5SS1\r\n");
            DebugP_log(" PBIST complete for VIM1 R5SS1\r\n");
            DebugP_log(" PBIST complete for R5SS1 RAM\r\n");
            DebugP_log(" PBIST complete for MSS CR5B ATCM0\r\n");
            DebugP_log(" PBIST complete for MSS CR5B ATCM1\r\n");
            DebugP_log(" PBIST complete for MSS CR5B BTCM0\r\n");
            DebugP_log(" PBIST complete for MSS CR5B BTCM1\r\n");
        }
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\nAll tests have passed. \r\n");
        }
        else
        {
            DebugP_log("\r\nSome tests have failed. \r\n");
        }
    }
#elif defined (R5F1_INPUTS)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        {
            DebugP_log(" PBIST complete for  VIM0 R5SS0\r\n");
            DebugP_log(" PBIST complete for  VIM1 R5SS0\r\n");
            DebugP_log(" PBIST complete for  VIM0 R5SS1\r\n");
            DebugP_log(" PBIST complete for  VIM1 R5SS1\r\n");
            DebugP_log(" PBIST complete for MSS_L2_0\r\n");
            DebugP_log(" PBIST complete for R5SS0 RAM\r\n");
            DebugP_log(" PBIST complete for CR5A ROM0\r\n");
            DebugP_log(" PBIST complete for TRACE\r\n");
            DebugP_log(" PBIST complete for MMCH0\r\n");
            DebugP_log(" PBIST complete for MSS CR5A ATCM0\r\n");
            DebugP_log(" PBIST complete for MSS CR5A ATCM1\r\n");
            DebugP_log(" PBIST complete for MSS CR5A BTCM0\r\n");
            DebugP_log(" PBIST complete for MSS CR5A BTCM1\r\n");

        }
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\nAll tests have passed. \r\n");
        }
        else
        {
            DebugP_log("\r\nSome tests have failed. \r\n");
        }
    }
#endif
#endif

#if defined (SOC_AM263PX)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        {
            DebugP_log(" PBIST complete for R5 STC\r\n");
            DebugP_log(" PBIST complete for R51 STC\r\n");
            DebugP_log(" PBIST complete for R50 TMU1\r\n");
            DebugP_log(" PBIST complete for R50 TMU2\r\n");
            DebugP_log(" PBIST complete for R50 TMU3\r\n");
            DebugP_log(" PBIST complete for R50 TMU4\r\n");
            DebugP_log(" PBIST complete for R50 TMU5\r\n");
            DebugP_log(" PBIST complete for R50 TMU6\r\n");
            DebugP_log(" PBIST complete for R51 TMU1\r\n");
            DebugP_log(" PBIST complete for R51 TMU2\r\n");
            DebugP_log(" PBIST complete for R51 TMU3\r\n");
            DebugP_log(" PBIST complete for R51 TMU4\r\n");
            DebugP_log(" PBIST complete for R51 TMU5\r\n");
            DebugP_log(" PBIST complete for R51 TMU6\r\n");
            DebugP_log(" PBIST complete for PBISTROM\r\n");
            DebugP_log(" PBIST complete for ROM0\r\n");
            DebugP_log(" PBIST complete for ROM1\r\n");
            DebugP_log(" PBIST complete for CPSW\r\n");
            DebugP_log(" PBIST complete for ECU_PERIPH\r\n");
            DebugP_log(" PBIST complete for FOTA\r\n");
            DebugP_log(" PBIST complete for ICSSM\r\n");
            DebugP_log(" PBIST complete for MBOX\r\n");
            DebugP_log(" PBIST complete for MSS_L2_1\r\n");
            DebugP_log(" PBIST complete for MSS_L2_2\r\n");
            DebugP_log(" PBIST complete for MSS_L2_3\r\n");
            DebugP_log(" PBIST complete for MSS_L2_4\r\n");
            DebugP_log(" PBIST complete for MSS_L2_5\r\n");
            DebugP_log(" PBIST complete for TPCC\r\n");
            DebugP_log(" PBIST complete for OSPI\r\n");
            DebugP_log(" PBIST complete for R5SS0 CPU0 RL2 \r\n");
            DebugP_log(" PBIST complete for R5SS0 CPU1 RL2 \r\n");
            DebugP_log(" PBIST complete for R5SS1 CPU0 RL2 \r\n");
            DebugP_log(" PBIST complete for R5SS1 CPU1 RL2 \r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 C0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 C1\r\n");
            DebugP_log(" PBIST complete for MSS R5SS1 C0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS1 C1\r\n");
            DebugP_log(" PBIST complete for MSS MMCH0\r\n");
            DebugP_log(" PBIST complete for MSS MMCH1\r\n");
            DebugP_log(" PBIST complete for MSS CR5B R5FSS0 ATCM0\r\n");
            DebugP_log(" PBIST complete for MSS CR5B R5FSS1 ATCM0\r\n");
            DebugP_log(" PBIST complete for MSS CR5B R5FSS0 BTCM0\r\n");
            DebugP_log(" PBIST complete for MSS CR5B R5FSS1 BTCM0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 VIM0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 VIM1\r\n");
            DebugP_log(" PBIST complete for MSS R5SS1 VIM0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS1 VIM1\r\n");
        }
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\nAll tests have passed. \r\n");
        }
        else
        {
            DebugP_log("\r\nSome tests have failed. \r\n");
        }
    }
#endif

#if defined (SOC_AM261X)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        {
            DebugP_log(" PBIST complete for R5 STC\r\n");
            DebugP_log(" PBIST complete for R50 TMU1\r\n");
            DebugP_log(" PBIST complete for R50 TMU2\r\n");
            DebugP_log(" PBIST complete for R50 TMU3\r\n");
            DebugP_log(" PBIST complete for R50 TMU4\r\n");
            DebugP_log(" PBIST complete for R50 TMU5\r\n");
            DebugP_log(" PBIST complete for R50 TMU6\r\n");
            DebugP_log(" PBIST complete for PBISTROM\r\n");
            DebugP_log(" PBIST complete for ROM0\r\n");
            DebugP_log(" PBIST complete for ROM1\r\n");
            DebugP_log(" PBIST complete for ICSSM0\r\n");
            DebugP_log(" PBIST complete for ICSSM1\r\n");
            DebugP_log(" PBIST complete for CPSW\r\n");
            DebugP_log(" PBIST complete for ECU_PERIPH\r\n");
            DebugP_log(" PBIST complete for FOTA\r\n");
            DebugP_log(" PBIST complete for MBOX\r\n");
            DebugP_log(" PBIST complete for TPCC\r\n");
            DebugP_log(" PBIST complete for OSPI\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 RL2 CPU0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 RL2 CPU1\r\n");
            DebugP_log(" PBIST complete for MSS TRACE\r\n");
            DebugP_log(" PBIST complete for MSS USB\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 C0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 C1\r\n");
            DebugP_log(" PBIST complete for MSS_L2_1\r\n");
            DebugP_log(" PBIST complete for MSS_L2_2\r\n");
            DebugP_log(" PBIST complete for MSS MMCH0\r\n");
            DebugP_log(" PBIST complete for MSS MMCH1\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 VIM0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 VIM1\r\n");
        }
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\nAll tests have passed. \r\n");
        }
        else
        {
            DebugP_log("\r\nSome tests have failed. \r\n");
        }
    }
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    if((status == SDL_PASS) && (testType == SDL_PBIST_NEG_TEST))
    {
        if (instanceId == SDL_PBIST_INST_TOP)
        {
            DebugP_log("PBIST failure Insertion test complete for TOP BIST\r\n");
            DebugP_log("PBIST Failure Insertion Test completed in %d micro secs \r\n", (uint32_t)diffTime );
        }
    }
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        if (instanceId == SDL_PBIST_INST_TOP)
        {
            DebugP_log("PBIST test complete for TOP BIST\r\n");
            DebugP_log("PBIST test completed in %d micro secs \r\n", (uint32_t)diffTime );
        }
    }
#endif
#if defined (SOC_AM273X) || (defined SOC_AWR294X)
    if((status == SDL_PASS) && (testType == SDL_PBIST_NEG_TEST))
    {
        if (instanceId == SDL_PBIST_INST_TOP)
        {
            DebugP_log(" PBIST failure Insertion test complete for TOP BIST\r\n");
            DebugP_log("PBIST Failure Insertion Test completed in %d micro secs \r\n", (uint32_t)diffTime );
        }
        else if (instanceId == SDL_PBIST_INST_DSS)
        {
            DebugP_log(" PBIST failure Insertion test complete for DSP BIST\r\n");
            DebugP_log("PBIST Failure Insertion Test completed in %d micro secs \r\n", (uint32_t)diffTime );
        }
    }
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        if (instanceId == SDL_PBIST_INST_TOP)
        {
            DebugP_log("PBIST test complete for TOP BIST\r\n");
            DebugP_log("PBIST test completed in %d micro secs \r\n", (uint32_t)diffTime );
        }
        else if (instanceId == SDL_PBIST_INST_DSS)
        {
            DebugP_log("PBIST test complete for DSP BIST\r\n");
            DebugP_log("PBIST test completed in %d micro secs \r\n", (uint32_t)diffTime );
        }
    }
#endif

    return (testResult);
}


/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void pbist_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    /* Declarations of variables */
    int32_t    result = SDL_PASS;
    int32_t    testResult = 0;

    /* Init Dpl */
    result = SDL_TEST_dplInit();
    if (result != SDL_PASS)
    {
       DebugP_log("Error: DPL Init Failed\r\n");
    }

    DebugP_log("\n PBIST Application\r\n");

    /* Run the test for diagnostics first */
    /* Run test on selected instance */
    testResult = PBIST_runTest(SDL_PBIST_INST_TOP, true);

    if (testResult == 0)
    {
        CacheP_disable(CacheP_TYPE_L1P);
        CacheP_disable(CacheP_TYPE_L1D);
        /* Run test on selected instance */
        testResult = PBIST_runTest(SDL_PBIST_INST_TOP, false);
    }
    #if defined (SOC_AM273X) || (SOC_AWR294X)
    #if defined R5F0_INPUTS
    /* Run the test for diagnostics first */
    /* Run test on selected instance */
        testResult = PBIST_runTest(SDL_PBIST_INST_DSS, true);

        if (testResult == 0)
        {
          /* Run test on selected instance */
          testResult = PBIST_runTest(SDL_PBIST_INST_DSS, false);
        }
      #endif
      #endif

      Drivers_close();

}

/* Nothing past this point */
