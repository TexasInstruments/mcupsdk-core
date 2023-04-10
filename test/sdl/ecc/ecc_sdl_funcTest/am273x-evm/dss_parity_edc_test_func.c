/*
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 *  \file     dss_parity_edc_test_func.c
 *
 *  \brief    This file contains DSS Parity and EDC SDL Function test
 *            code for c66 core.
 *  \details  ECC SDL API module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/sdl_ecc.h>
#include "ecc_test_main.h"

#pragma CODE_SECTION(EDC_dummyFunction, ".func");
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SDL_ESM_MAX_DSS_AGGR				(6u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_DSS_ECC_NC_ITERRUPT_ID					(111)
#define SDL_DSS_ECC_C_ITERRUPT_ID					(112)

#define SDL_ENABLE_L1D_DATA_MASK_FLG                (0x01u)
#define SDL_ENABLE_L1D_TAG_MASK_FLG                 (0x02u)
#define SDL_ENABLE_L2_TAG_MASK_FLG                  (0x04u)
#define SDL_ENABLE_L2_SNOP_MASK_FLG                 (0x08u)
#define SDL_ENABLE_L2_MPPA_MASK_FLG                 (0x10u)
#define SDL_ENABLE_L2_LRU_MASK_FLG                  (0x20u)
#define SDL_ENABLE_L1P_TAG_MASK_FLG                 (0x40u)

#define SDL_EDC_SEC                       			(0x01u)
#define SDL_EDC_DED                       			(0x03u)

#define SDL_DSS_L2_ORIGIN                           (0x00800000u)
#define SDL_DSS_L1P_ORIGIN                          (0x00E00000U)
#define SDL_DSS_INTH_INT_ID_IDMAINT1                (14)

#define SDL_DSS_ICFGF_L1PCFG                        (0x01840020U)
#define SDL_DSS_ICFGF_L1DCFG                        (0x01840040U)

/*Error Detect and Correct Interrupt Mask Register*/
#define SDL_DSP_ICFG_EDCINTMASK     				(0x01831100u)
/*Error Detect and Correct Interrupt Flag Register*/
#define SDL_DSP_ICFG_EDCINTFLG      				(0x01831104u)

#define SDL_INJECT_PARITY                           (0x01u)

#define SDL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1     (0x06020074u)
#define SDL_INITIAL_VALUE                           (0x11u)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern volatile bool gMsmcMemParityInterrupt;
extern volatile bool idmaTransferComplete;

extern int32_t SDL_ESM_DSP_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
													int32_t grpChannel,
													int32_t intSrc,
													void *arg);

extern int32_t SDL_ECC_DED_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                    int32_t grpChannel,
                                                    int32_t intSrc,
                                                    void *arg);

extern int32_t SDL_ECC_SEC_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                    int32_t grpChannel,
                                                    int32_t intSrc,
                                                    void *arg);

extern int32_t SDL_IDMA1_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg);
/* Event BitMap for EDC ESM callback for DSS */
SDL_ESM_NotifyParams EDC_TestparamsDSS[SDL_ESM_MAX_DSS_AGGR] =
{
    {
       /* Event BitMap for EDC ESM callback for DSS Single bit*/
       .groupNumber = SDL_INTR_GROUP_NUM,
       .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_SEC_ERR,
       .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
       .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
       .callBackFunction = &SDL_ESM_DSP_applicationCallbackFunction,
    },
    {
       /* Event BitMap for EDC ESM callback for DSS Double bit*/
       .groupNumber = SDL_INTR_GROUP_NUM,
       .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_DED_ERR,
       .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
       .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
       .callBackFunction = &SDL_ESM_DSP_applicationCallbackFunction,
    },
    {
	   /* Event BitMap for EDC ESM callback for DSS Single bit*/
	   .groupNumber = SDL_INTR_GROUP_NUM,
	   .errorNumber = SDL_DSS_ESMG1_DSS_DSP_EDC_SEC_ERR,
	   .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
	   .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
	   .callBackFunction = &SDL_ESM_DSP_applicationCallbackFunction,
	},
    {
	   /* Event BitMap for EDC ESM callback for DSS Double bit*/
	   .groupNumber = SDL_INTR_GROUP_NUM,
	   .errorNumber = SDL_DSS_ESMG1_DSS_DSP_EDC_DED_ERR,
	   .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
	   .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
	   .callBackFunction = &SDL_ESM_DSP_applicationCallbackFunction,
    },

    {
       /* Event BitMap for Parity ESM callback for DSS Single bit*/
       .groupNumber = SDL_INTR_GROUP_NUM,
       .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L1P_PARITY,
       .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
       .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
       .callBackFunction = &SDL_ESM_DSP_applicationCallbackFunction,
    },
    {
       /* Event BitMap for Parity ESM callback for DSS Single bit*/
       .groupNumber = SDL_INTR_GROUP_NUM,
       .errorNumber = SDL_DSS_ESMG1_DSS_DSP_L2_PARITY_ERR_VB0_EVEN,
       .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
       .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
       .callBackFunction = &SDL_ESM_DSP_applicationCallbackFunction,
    },
};
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t DSS_Test_init (void);
int32_t EDC_dummyFunction(void);
static int32_t DSS_FuncTest(void);
static int32_t EDC_Test_run_DSS_L2_EDC_Test(uint32_t injectType);
static int32_t EDC_Test_run_DSS_DSP_EDC_Errors_Test(uint32_t memory);
static int32_t EDC_Test_run_DSS_L1_Parity_sdlFuncTest(void);
static int32_t EDC_Test_run_DSS_L2_Parity_sdlFuncTest(void);


/*********************************************************************
* @fn      EDC_dummyFunction
*
* @param   None
*
* @return  dummy operation value
**********************************************************************/
int32_t EDC_dummyFunction(void)
{
    int32_t a = 0;
    int32_t b = 4;
    int32_t i;
    for (i = 0; i <= 3; i++)
    {
        b = b + 7;
        a = a + b;
    }
    return a;
}/* End of EDC_dummyFunction() */

/*********************************************************************
* @fn      DSS_Test_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t DSS_Test_init (void)
{
	int32_t retValue=0, counter = 0;
    SDL_ErrType_t result;
    if (retValue == 0) {
		for(counter = 0; counter < SDL_ESM_MAX_DSS_AGGR; counter++)
		{
			result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &EDC_TestparamsDSS[counter],NULL,NULL);
			if (result != SDL_PASS) {
				retValue = -1;
			   /* print error and quit */
				DebugP_log("\r\nESM_Test_init: Error initializing DSS ESM: result = %d\r\n", result);
			}
		}
        if(retValue == 0)
        {
            DebugP_log("\r\nESM_Test_init: Init DSS ESM complete\r\n");
        }
    }
    return retValue;
}/* End of DSS_ECC_Test_init() */

/*********************************************************************
 * @fn      EDC_Test_run_DSS_L2_EDC_Test
 *
 * @brief   Execute DSS L2 EDC single and double bit inject test
 *
 * @param   injectType : Number of bits injected
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
static int32_t EDC_Test_run_DSS_L2_EDC_Test(uint32_t injectType)
{
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000u;
    uint32_t timeOutCnt = 0u;
    SDL_DPL_HwipParams intrParams;
    pSDL_DPL_HwipHandle SDL_EDC_CfgHwiPHandle = NULL;
    volatile uint32_t rd_data;

    DebugP_log("\r\nDSS L2 EDC tests: starting\r\n");

    if (retVal == 0)
    {
        /* Configuring the ESM */
        intrParams.callbackArg = (uintptr_t)SDL_ESM_INST_DSS_ESM;

        /* Configuring the IDMA1 DED interrupt */
        intrParams.intNum = SDL_DSS_INTH_INT_ID_IDMAINT1;
        intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_IDMA1_applicationCallbackFunction;

        /* Register call back function for vector Config Interrupt */
        SDL_DPL_registerInterrupt(&intrParams, &SDL_EDC_CfgHwiPHandle);

        /*Enable the IDMA 1 interrupt*/
        SDL_DPL_enableInterrupt(SDL_DSS_INTH_INT_ID_IDMAINT1);

        DebugP_log("\r\nInitiliazed and Enabled IDMA1 Interrupt\r\n");

        DebugP_log("\r\nEnable the Error Detect logic...\r\n");
        retVal = SDL_ECC_dss_l2_edc_CMD_EN();
        if (retVal == SDL_PASS)
        {
            SDL_ECC_IDMA1_transfer(SDL_DSS_L2_ORIGIN, SDL_DSS_L2_ORIGIN);
            DebugP_log("\r\nWaiting for IDMA1 transfer Interrupt\r\n");
            do
            {
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    retVal = SDL_EFAIL;
                    DebugP_log("\r\nIDMA1 interrupt is not occurred.... Test is failed!!\r\n");
                    break;
                }
            } while (idmaTransferComplete == false);

            DebugP_log("\r\nIDMA1 transfer is done and got interrupt !!\r\n");

            EDC_dummyFunction();

            DebugP_log("\r\nSuspend the Error Detect logic...\r\n");
            retVal = SDL_ECC_dss_l2_CMD_SUSP();
            if (retVal == SDL_PASS)
            {
                if(injectType == SDL_EDC_SEC)
                {
                    DebugP_log("\r\nToggle a single bit in the Dummy function\r\n");
                    rd_data = SDL_REG32_RD(SDL_DSS_L2_ORIGIN);
                    rd_data = rd_data ^ SDL_EDC_SEC;
                    SDL_REG32_WR(SDL_DSS_L2_ORIGIN, rd_data);
                }
                else if(injectType == SDL_EDC_DED)
                {
                    DebugP_log("\r\nToggle double bit in the Dummy function\r\n");
                    rd_data = SDL_REG32_RD(SDL_DSS_L2_ORIGIN);
                    rd_data = rd_data ^ SDL_EDC_DED;
                    SDL_REG32_WR(SDL_DSS_L2_ORIGIN, rd_data);
                }
                else{
                    /* No EDC inject */
                }

                DebugP_log("\r\nEnable the Error Detect logic...\r\n");
                retVal = SDL_ECC_dss_l2_edc_CMD_EN();

                if (retVal == SDL_PASS)
                {
                    DebugP_log("\r\nCall dummy function\r\n");
                    EDC_dummyFunction();

                    DebugP_log("\r\nWaiting for ESM Interrupt\r\n");
                    timeOutCnt = 0;
                    do
                    {
                        timeOutCnt += 10;
                        if (timeOutCnt > maxTimeOutMilliSeconds)
                        {
                            retVal = SDL_EFAIL;
                            DebugP_log("\r\nESM interrupt is not occurred.... Test is failed!!\r\n");
                            break;
                        }
                    } while (gMsmcMemParityInterrupt == false);
                    if(gMsmcMemParityInterrupt == true)
                    {
                        DebugP_log("\r\nESM Interrupt has occurred!!\r\n");
                        if(injectType == SDL_EDC_SEC)
                        {
                            DebugP_log("\r\nSEC test has passed!!\r\n");
                        }
                        else
                        {
                            DebugP_log("\r\nDED test has passed!!\r\n");
                        }
                        DebugP_log("\r\n***************************************************************************\r\n");
                        gMsmcMemParityInterrupt = false;
                        retVal = SDL_PASS;
                    }
                }
                else
                {
                    DebugP_log("\r\nEnable Error Detect logic is failed after IDMA transfer and Error injection...\r\n");
                    retVal = SDL_EFAIL;
                }
            }
            else
            {
                DebugP_log("\r\nSuspend Error Detect logic is failed...\r\n");
                retVal = SDL_EFAIL;
            }
        }
        else
        {
            DebugP_log("\r\nEnable Error Detect logic is failed...\r\n");
            retVal = SDL_EFAIL;
        }
    }
    return(retVal);
}/* End of EDC_Test_run_DSS_DSP_EDC_Errors_Test() */
/*********************************************************************
 * @fn      EDC_Test_run_DSS_DSP_EDC_Errors_Test
 *
 * @brief   Execute DSS DSP EDC errors exception tests
 *
 * @param   memory : 
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
static int32_t EDC_Test_run_DSS_DSP_EDC_Errors_Test(uint32_t memory)
{
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;
    uint32_t readValue = 0;
    SDL_DPL_HwipParams intrParams;
    pSDL_DPL_HwipHandle SDL_EDC_CfgHwiPHandle = NULL;

    DebugP_log("\r\nDSS DSP EDC Errors tests: starting\r\n");

    if (retVal == 0)
    {
        /* Configuring the ESM */
		intrParams.callbackArg = (uintptr_t)SDL_ESM_INST_DSS_ESM;
		
		/* Configuring the ECC DED interrupt */
		intrParams.intNum = SDL_DSS_ECC_NC_ITERRUPT_ID;
		intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_ECC_DED_applicationCallbackFunction;

        /* Register call back function for vector Config Interrupt */
        SDL_DPL_registerInterrupt(&intrParams, &SDL_EDC_CfgHwiPHandle);

        /*Enable the DED interrupt*/
        SDL_DPL_enableInterrupt(SDL_DSS_ECC_NC_ITERRUPT_ID);

        DebugP_log("\r\nInitiliazed and Enabled ECC DED Interrupt\r\n");
		
        /* Configuring the ECC SEC interrupt */
        intrParams.intNum = SDL_DSS_ECC_C_ITERRUPT_ID;
        intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_ECC_SEC_applicationCallbackFunction;

        /* Register call back function for vector Config Interrupt */
        SDL_DPL_registerInterrupt(&intrParams, &SDL_EDC_CfgHwiPHandle);

        /*Enable the SEC interrupt*/
        SDL_DPL_enableInterrupt(SDL_DSS_ECC_C_ITERRUPT_ID);

        DebugP_log("\r\nInitiliazed and Enabled ECC SEC Interrupt\r\n");

        /*Write to DSP_ICFG__EDCINTMASK and DSP_ICFG__EDCINTFLG registers
         * to enable and propagate an DSS DSP Memories
         */
        DebugP_log("\r\nEnabling and Propagating the memory\r\n");
        SDL_ECC_DSP_Aggregated_EDC_Errors(memory);


        DebugP_log("\r\nWaiting for DSS Interrupt\r\n");
        do
        {
            timeOutCnt += 10;
            if (timeOutCnt > maxTimeOutMilliSeconds)
            {
                retVal = SDL_EFAIL;
                DebugP_log("\r\nDSS interrupt is not occurred.... Test is failed!!\r\n");
                break;
            }
        } while (gMsmcMemParityInterrupt == false);

        readValue = SDL_REG32_RD(SDL_DSP_ICFG_EDCINTMASK);
        if(readValue == 0){
            readValue = SDL_REG32_RD(SDL_DSP_ICFG_EDCINTFLG);
            if(readValue == 0)
            {
				DebugP_log("\r\nDisabled the MASK and FLG registers\r\n");
				if(memory == SDL_ENABLE_L1D_DATA_MASK_FLG)
				{
				    DebugP_log("\r\nDSP L1D DATA EDC Error tested and got all Interrupts \r\n");
				}
				else if(memory == SDL_ENABLE_L1D_TAG_MASK_FLG)
                {
                    DebugP_log("\r\nDSP L1D TAG EDC Error tested and got all Interrupts \r\n");
                }
                else if(memory == SDL_ENABLE_L2_TAG_MASK_FLG)
                {
                    DebugP_log("\r\nDSP L2 TAG EDC Error tested and got all Interrupts \r\n");
                }
                else if(memory == SDL_ENABLE_L2_SNOP_MASK_FLG)
                {
                    DebugP_log("\r\nDSP L2 SNOP EDC Error tested and got all Interrupts \r\n");
                }
                else if(memory == SDL_ENABLE_L2_MPPA_MASK_FLG)
                {
                    DebugP_log("\r\nDSP L2 MPPA EDC Error tested and got all Interrupts \r\n");
                }
                else if(memory == SDL_ENABLE_L2_LRU_MASK_FLG)
                {
                    DebugP_log("\r\nDSP L2 LRU EDC Error tested and got all Interrupts \r\n");
                }
                else if(memory == SDL_ENABLE_L1P_TAG_MASK_FLG)
                {
                    DebugP_log("\r\nDSP L1P TAG EDC Error tested and got all Interrupts \r\n");
                }
                else
                {
                    /*Nothing*/
                }
				DebugP_log("\r\n***************************************************************************\r\n");
				gMsmcMemParityInterrupt = false;
                retVal = SDL_PASS;
            }
            else
            {
				DebugP_log("\r\nEDC FLG register is not equal to 0...... Test is failed!!\r\n");
                retVal = SDL_EFAIL;
            }
        }
        else
        {
			DebugP_log("\r\nEDC MASK register is not equal to 0...... Test is failed!!\r\n");
            retVal = SDL_EFAIL;
        }
    }

    return retVal;
}/* End of EDC_Test_run_DSS_DSP_EDC_Errors_Test() */

/*********************************************************************
 * @fn      EDC_Test_run_DSS_L1_Parity_sdlFuncTest
 *
 * @brief   Execute DSS L1 Parity sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t EDC_Test_run_DSS_L1_Parity_sdlFuncTest(void)
{
    int32_t retVal = SDL_PASS;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0;
    SDL_DPL_HwipParams intrParams;
    pSDL_DPL_HwipHandle SDL_Parity_CfgHwiPHandle = NULL;
    volatile uint32_t rd_data;

    DebugP_log("\r\nL1 Parity tests: starting\r\n");

    if (retVal == SDL_PASS)
    {
        /* Configuring the ESM */
        intrParams.callbackArg = (uintptr_t)SDL_ESM_INST_DSS_ESM;

        /* Configuring the IDMA1 DED interrupt */
        intrParams.intNum = SDL_DSS_INTH_INT_ID_IDMAINT1;
        intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_IDMA1_applicationCallbackFunction;

        /* Register call back function for vector Config Interrupt */
        SDL_DPL_registerInterrupt(&intrParams, &SDL_Parity_CfgHwiPHandle);

        /*Enable the IDMA 1 interrupt*/
        SDL_DPL_enableInterrupt(SDL_DSS_INTH_INT_ID_IDMAINT1);

        DebugP_log("\r\nInitiliazed and Enabled IDMA1 Interrupt\r\n");

        DebugP_log("\r\nEnable the Error Detect logic...\r\n");
        retVal = SDL_ECC_dss_l1p_edc_CMD_EN();
        if (retVal == SDL_PASS)
        {
            SDL_ECC_IDMA1_transfer(SDL_DSS_L2_ORIGIN, SDL_DSS_L1P_ORIGIN);
            DebugP_log("\r\nWaiting for IDMA1 transfer Interrupt\r\n");
            do
            {
                timeOutCnt += 10;
                if (timeOutCnt > maxTimeOutMilliSeconds)
                {
                    retVal = SDL_EFAIL;
                    DebugP_log("\r\nIDMA1 interrupt is not occurred.... Test is failed!!\r\n");
                    break;
                }
            } while (idmaTransferComplete == false);
            idmaTransferComplete = false;

            DebugP_log("\r\nIDMA1 transfer is done and got interrupt !!\r\n");

            DebugP_log("\r\nSuspend the Error Detect logic...\r\n");
            retVal = SDL_ECC_dss_l1p_CMD_SUSP();
            if (retVal == SDL_PASS)
            {
                DebugP_log("\r\nToggle a single bit in the Dummy function\r\n");
                rd_data = SDL_REG32_RD(SDL_DSS_L2_ORIGIN);
                rd_data = rd_data ^ SDL_EDC_SEC;
                SDL_REG32_WR(SDL_DSS_L2_ORIGIN, rd_data);

                SDL_ECC_IDMA1_transfer(SDL_DSS_L2_ORIGIN, SDL_DSS_L1P_ORIGIN);
                DebugP_log("\r\nWaiting for IDMA1 transfer Interrupt\r\n");
                do
                {
                    timeOutCnt += 10;
                    if (timeOutCnt > maxTimeOutMilliSeconds)
                    {
                        retVal = SDL_EFAIL;
                        DebugP_log("\r\nIDMA1 interrupt is not occurred.... Test is failed!!\r\n");
                        break;
                    }
                } while (idmaTransferComplete == false);
                idmaTransferComplete = false;
                DebugP_log("\r\nIDMA1 transfer is done and got interrupt !!\r\n");
                DebugP_log("\r\nEnable the Error Detect logic...\r\n");
                retVal = SDL_ECC_dss_l1p_edc_CMD_EN();
                SDL_ECC_IDMA1_transfer(SDL_DSS_L1P_ORIGIN, SDL_DSS_L2_ORIGIN);
                DebugP_log("\r\nWaiting for IDMA1 transfer Interrupt\r\n");
                do
                {
                    timeOutCnt += 10;
                    if (timeOutCnt > maxTimeOutMilliSeconds)
                    {
                        retVal = SDL_EFAIL;
                        DebugP_log("\r\nIDMA1 interrupt is not occurred.... Test is failed!!\r\n");
                        break;
                    }
                } while (idmaTransferComplete == false);
                DebugP_log("\r\nIDMA1 transfer is done and got interrupt !!\r\n");
                if (retVal == SDL_PASS)
                {
                    DebugP_log("\r\nWaiting for ESM Interrupt\r\n");
                    do
                    {
                        timeOutCnt += 10;
                        if (timeOutCnt > maxTimeOutMilliSeconds)
                        {
                            retVal = SDL_EFAIL;
                            DebugP_log("\r\nESM interrupt is not occurred.... Test is failed!!\r\n");
                            break;
                        }
                    } while (gMsmcMemParityInterrupt == false);
                    if(retVal == SDL_PASS)
                    {
                        DebugP_log("\r\nL1 Parity tests have passed!!\r\n");
                        DebugP_log("\r\n***************************************************************************\r\n");
                        gMsmcMemParityInterrupt = false;
                    }
                }
                else
                {
                    DebugP_log("\r\nEnable Error Detect logic is failed after IDMA transfer and Error injection...\r\n");
                    retVal = SDL_EFAIL;
                }
            }
            else
            {
                DebugP_log("\r\nSuspend Error Detect logic is failed...\r\n");
                retVal = SDL_EFAIL;
            }
        }
        else
        {
            DebugP_log("\r\nEnable Error Detect logic is failed...\r\n");
            retVal = SDL_EFAIL;
        }
    }
    return(retVal);
}/* End of EDC_Test_run_DSS_L1_Parity_sdlFuncTest() */

/*********************************************************************
 * @fn      EDC_Test_run_DSS_L2_Parity_sdlFuncTest
 *
 * @brief   Execute DSS L2 Parity sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t EDC_Test_run_DSS_L2_Parity_sdlFuncTest(void)
{
    int32_t result = 0;
    int32_t retVal = 0;
    uint32_t maxTimeOutMilliSeconds = 1000000000;
    uint32_t timeOutCnt = 0u;
    uint32_t injectErrAdd = 0u;

    DebugP_log("\r\nDSS L2 parity tests: starting\r\n");

    if (retVal == 0)
    {
        /*
         *Disable the parity by clearing DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE
         *Disable register field
         */
        SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_DSS_L2RAM_PARITY_ERROR_CLEAR);
        /*
         * Enable the parity
         */
        SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_DSS_L2RAM_PARITY_ENABLE);
        DebugP_log("\r\nDSS L2 parity is enabled\r\n");

        /*
         *Disable the parity by clearing DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE
         *Disable register field
         */
        SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_DSS_L2RAM_PARITY_ERROR_CLEAR);
        /*dss l2 parity init*/
        SDL_ECC_dss_l2_parity_init();
        DebugP_log("\r\nDSS L2 parity initialization is completed\r\n");

        /* On a parity error from a particular bank, reading the register
         * DSS_CTRL.DSS_DSP_L2RAM_PARITY_ERR_STATUS_VBx gives the address
         * location
         */
        injectErrAdd = SDL_REG32_RD(SDL_DSS_DSP_L2RAM_PARITY_ERR_STATUS_VB1);

        gMsmcMemParityInterrupt = false;

        /*DSS L2 parity error inject*/
        SDL_ECC_dss_l2_parity_errorInject(SDL_INJECT_PARITY, injectErrAdd, SDL_INITIAL_VALUE);

        DebugP_log("\r\nDSS L2 parity error injected\r\n");

        DebugP_log("\r\nWaiting for ESM Interrupt \r\n");
        do
        {
            timeOutCnt += 1;
            if (timeOutCnt > maxTimeOutMilliSeconds)
            {
                result = SDL_EFAIL;
                break;
            }
        } while (gMsmcMemParityInterrupt == false);

        if(result == SDL_PASS){
            DebugP_log("\r\ncleared DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE\r\n");
            DebugP_log("\r\nParity error is injected 1-bit error and got ESM Interrupt \r\n");
            DebugP_log("\r\n***************************************************************************\r\n");
            gMsmcMemParityInterrupt = false;
        }

        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_L2_Parity_sdlFuncTest: has failed...\r\n");
            /* Low priority interrupt */
        }
    }

    return retVal;
}/* End of EDC_Test_run_DSS_L2_Parity_sdlFuncTest() */

/*********************************************************************
 * @fn      DSS_FuncTest
 *
 * @brief   Execute DSS DSP Parity and EDC sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t DSS_FuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;

    DebugP_log("\r\nEDC tests: starting\r\n");

    if (retVal == 0) {
        result = EDC_Test_run_DSS_L2_Parity_sdlFuncTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_L2_Parity_sdlFuncTest test has failed...\r\n");
        }
    }
    if (retVal == 0) {
        /*L1PCFG is set to 0*/
        SDL_REG32_WR(SDL_DSS_ICFGF_L1PCFG, 0x00);
        /*L1DCFG is set to 0*/
        SDL_REG32_WR(SDL_DSS_ICFGF_L1DCFG, 0x00);
        result = EDC_Test_run_DSS_L2_EDC_Test(SDL_EDC_SEC);
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_L2_EDC_Test Single test has failed...\r\n");
        }
    }
//
//    if (retVal == 0) {
//        /*L1PCFG is set to 0*/
//        SDL_REG32_WR(SDL_DSS_ICFGF_L1PCFG, 0x00);
//        /*L1DCFG is set to 0*/
//        SDL_REG32_WR(SDL_DSS_ICFGF_L1DCFG, 0x00);
//        result = EDC_Test_run_DSS_L2_EDC_Test(SDL_EDC_DED);
//        if (result != SDL_PASS) {
//            retVal = -1;
//            DebugP_log("\r\nEDC_Test_run_DSS_L2_EDC_Test Double test has failed...\r\n");
//        }
//    }

    if (retVal == 0) {
         result = EDC_Test_run_DSS_L1_Parity_sdlFuncTest();
         if (result != SDL_PASS) {
             retVal = -1;
             DebugP_log("\r\nEDC_Test_run_DSS_L1_Parity_sdlFuncTest test has failed...\r\n");
         }
    }

	if (retVal == 0) {

        result = EDC_Test_run_DSS_DSP_EDC_Errors_Test(SDL_ENABLE_L1D_DATA_MASK_FLG);
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_DSP_EDC_Errors_Test L1D DATA memory test has failed...\r\n");
        }
    }
    if (retVal == 0) {
        result = EDC_Test_run_DSS_DSP_EDC_Errors_Test(SDL_ENABLE_L1D_TAG_MASK_FLG);
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_DSP_EDC_Errors_Test L1D TAG memory test has failed...\r\n");
        }
    }
	if (retVal == 0) {
        result = EDC_Test_run_DSS_DSP_EDC_Errors_Test(SDL_ENABLE_L2_TAG_MASK_FLG);
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_DSP_EDC_Errors_Test L2 TAG memory test has failed...\r\n");
        }
    }
	if (retVal == 0) {
        result = EDC_Test_run_DSS_DSP_EDC_Errors_Test(SDL_ENABLE_L2_SNOP_MASK_FLG);
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_DSP_EDC_Errors_Test L2 SNOP memory test has failed...\r\n");
        }
    }
	if (retVal == 0) {
        result = EDC_Test_run_DSS_DSP_EDC_Errors_Test(SDL_ENABLE_L2_MPPA_MASK_FLG);
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_DSP_EDC_Errors_Test L2 MPPA memory test has failed...\r\n");
        }
    }
	if (retVal == 0) {
        result = EDC_Test_run_DSS_DSP_EDC_Errors_Test(SDL_ENABLE_L2_LRU_MASK_FLG);
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_DSP_EDC_Errors_Test L2 LRU memory test has failed...\r\n");
        }
    }
	if (retVal == 0) {
        result = EDC_Test_run_DSS_DSP_EDC_Errors_Test(SDL_ENABLE_L1P_TAG_MASK_FLG);
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\r\nEDC_Test_run_DSS_DSP_EDC_Errors_Test L1P TAG memory test has failed...\r\n");
        }
    }

	return retVal;
}/* End of DSS_FuncTest() */
/*********************************************************************
 * @fn      DSS_sdl_funcTest
 *
 * @brief   DSS DSP Parity and EDC Function modules test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t DSS_sdl_funcTest(void)
{
    int32_t testResult;

	testResult = DSS_Test_init();
	if (testResult != 0)
    {
        DebugP_log("\nDSS SDL API Init: unsuccessful");
        return SDL_EFAIL;
    }
    testResult = DSS_FuncTest();
    DebugP_log("\r\n***************************************************************************\r\n");
	if (testResult != 0)
    {
        DebugP_log("\n DSS SDL API tests: unsuccessful");
        return SDL_EFAIL;
    }
    return (testResult);
}/* End of DSS_sdl_funcTest() */

/* Nothing past this point */
