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
 *  \file     ecc_test_func.c
 *
 *  \brief    This file contains ECC SDL Function test code for c66 core. *
 *  \details  ECC SDL API module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/hw_types.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/am273x/sdlr_soc_ecc_aggr.h>
#include <sdl/include/am273x/sdlr_dss_ecc_agg.h>
#include <sdl/sdl_ecc.h>
#include "ecc_test_main.h"
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* Defines */
#define DELAY 1

#define SDL_ESM_MAX_DSS_EXAMPLE_AGGR				(2u)

#define SDL_INTR_GROUP_NUM                          (1U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)

#define SDL_DSS_MAX_MEM_SECTIONS                    (17u)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern uint32_t testCounter;
volatile bool esmErrorFlag = false;
static SDL_ECC_MemSubType ECC_Test_DSSsubMemTypeList[SDL_DSS_MAX_MEM_SECTIONS] =
{     
	 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
	 SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID,
	 SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID,
	 SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID,
	 SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
	 SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID,	
	 SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID,
	 SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_ID,
     SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_ID,
     SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_ID,
     SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_ID,
     SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_ID,
     SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_ID,
     SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_ID,
     SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_ID,
     SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_ID,
	 SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID,
};

static SDL_ECC_InitConfig_t ECC_Test_DSSECCInitConfig =
{
    .numRams = SDL_DSS_MAX_MEM_SECTIONS,
    /**< Number of Rams ECC is enabled  */
    .pMemSubTypeList = &(ECC_Test_DSSsubMemTypeList[0]),
    /**< Sub type list  */
};

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
													int32_t grpChannel,
													int32_t intSrc,
													void *arg);

/* Event BitMap for ECC ESM callback for DSS */
SDL_ESM_NotifyParams ECC_TestparamsDSS[SDL_ESM_MAX_DSS_EXAMPLE_AGGR] =
{
    {
	   /* Event BitMap for ECC ESM callback for DSS Single bit*/
	   .groupNumber = SDL_INTR_GROUP_NUM,
	   .errorNumber = SDL_DSS_ESMG1_DSS_ECC_AGG_SERR,
	   .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
	   .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
	   .callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
    {
	   /* Event BitMap for ECC ESM callback for DSS Double bit*/
	   .groupNumber = SDL_INTR_GROUP_NUM,
	   .errorNumber = SDL_DSS_ESMG1_DSS_ECC_AGG_UERR,
	   .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
	   .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
	   .callBackFunction = &SDL_ESM_applicationCallbackFunction,
    },
	
};
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/*********************************************************************
* @fn      DSS_L3RAMA_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t DSS_ECC_Test_init (void)
{
	int32_t retValue=0;
    SDL_ErrType_t result;
    
    if (retValue == 0) {
        /* Initialize ECC */
        result = SDL_ECC_init(SDL_DSS_ECC_AGG, &ECC_Test_DSSECCInitConfig);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ECC_Test_init: Error initializing DSS ECC AGGR: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nECC_Test_init: DSS ECC AGGR initialization is completed \n");
        }
    }
	if (retValue == 0) {
        /* Initialize ECC callbacks within the DSS ESM */
        result = SDL_ECC_initEsm(SDL_ESM_INST_DSS_ESM);
        if (result != SDL_PASS) {
            /* print error and quit */
             DebugP_log("ECC_Test_init: Error initializing ECC callback for Main ESM: result = %d\n", result);

            retValue = -1;
        } else {
            DebugP_log("\nECC_Test_init: ECC Callback Init complete for Main ESM \n");
        }
    }
    return retValue;
}
/*********************************************************************
 * @fn      ECC_Test_run_DSS_HWA_PARAM_1Bit_N_RowInjectTest
 *
 * @brief   Execute ECC DSS_HWA_PARAM 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_HWA_PARAM_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS_HWA_PARAM Single bit error inject: test starting");
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection", ecc_ctrl, ecc_sts);
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06060000u);

    /* Run one shot test for DSS_HWA_PARAM 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
	
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC single bit error injection",
                   ecc_ctrl, ecc_sts);
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS_HWA_PARAM Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        DebugP_log("\n DSS_HWA_PARAM Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_HWA_PARAM_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_HWA_PARAM_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_HWA_PARAM 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_HWA_PARAM_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS_HWA_PARAM Double bit error inject: starting");

    /* Run one shot test for DSS_HWA_PARAM 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06060000u);	
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
	
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS_HWA_PARAM Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {
		DebugP_log("\n DSS_HWA_PARAM Double bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_HWA_PARAM_2Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMA_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMA_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("DSS L3RAMA Single bit error inject: test starting\n");

	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection\n", ecc_ctrl, ecc_sts);
				   
	/* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
    /* Run one shot test for DSS L3RAMA 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC single bit error injection\n",
                       ecc_ctrl, ecc_sts);
					   
	if (result != SDL_PASS ) {
        DebugP_log("DSS L3RAMA Single bit error inject at pErrMem 0x%p test failed\n",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("DSS L3RAMA Single bit error inject at pErrMem 0x%p\n",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMA_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMA_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMA_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("DSS L3RAMA Double bit error inject: starting");

    /* Run one shot test for DSS L3RAMA 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\nDSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection\n",ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection\n",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("DSS L3RAMA Double bit error inject: at pErrMem 0x%p: fixed location once test failed\n",injectErrorConfig.pErrMem);
       retVal = -1;
    } else {
        DebugP_log("DSS L3RAMA Double bit error inject at pErrMem 0x%p\n",injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMA_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMA_1Bit_OnceInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMA_1Bit_OnceInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection\n", ecc_ctrl, ecc_sts);
				   
	/* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
    /* Run one shot test for DSS L3RAMA 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC single bit error injection\n",
                       ecc_ctrl, ecc_sts);
					   
	if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMA Single bit error inject at pErrMem 0x%p test failed\n",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\n DSS L3RAMA Single bit error inject at pErrMem 0x%p\n",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMA_1Bit_OnceInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMA_1Bit_N_RowOnceInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMA_1Bit_N_RowOnceInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection\n", ecc_ctrl, ecc_sts);
				   
	/* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
    /* Run one shot test for DSS L3RAMA 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_ONCE,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC single bit error injection\n",
                       ecc_ctrl, ecc_sts);
					   
	if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMA Single bit error inject at pErrMem 0x%p test failed\n",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\n DSS L3RAMA Single bit error inject at pErrMem 0x%p\n",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMA_1Bit_N_RowOnceInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMA_1Bit_N_RepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMA_1Bit_N_RepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection\n", ecc_ctrl, ecc_sts);
				   
	/* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
    /* Run one shot test for DSS L3RAMA 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC single bit error injection\n",
                       ecc_ctrl, ecc_sts);
					   
	if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMA Single bit error inject at pErrMem 0x%p test failed\n",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\n DSS L3RAMA Single bit error inject at pErrMem 0x%p\n",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMA_1Bit_N_RepeatInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMA_2Bit_OnceInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMA_2Bit_OnceInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    /* Run one shot test for DSS L3RAMA 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection\n",ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection\n",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMA Double bit error inject: at pErrMem 0x%p: fixed location once test failed\n",injectErrorConfig.pErrMem);
       retVal = -1;
    } else {
        DebugP_log("\n DSS L3RAMA Double bit error inject at pErrMem 0x%p\n",injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMA_2Bit_OnceInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMA_2Bit_N_RowOnceInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMA_2Bit_N_RowOnceInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    /* Run one shot test for DSS L3RAMA 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection\n",ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_ONCE,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection\n",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMA Double bit error inject: at pErrMem 0x%p: fixed location once test failed\n",injectErrorConfig.pErrMem);
       retVal = -1;
    } else {
        DebugP_log("\n DSS L3RAMA Double bit error inject at pErrMem 0x%p\n",injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMA_2Bit_N_RowOnceInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMA_2Bit_N_RepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMA 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMA_2Bit_N_RepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    /* Run one shot test for DSS L3RAMA 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88000000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection\n",ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection\n",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMA Double bit error inject: at pErrMem 0x%p: fixed location once test failed\n",injectErrorConfig.pErrMem);
       retVal = -1;
    } else {
        DebugP_log("\n DSS L3RAMA Double bit error inject at pErrMem 0x%p\n",injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMA_2Bit_N_RepeatInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMB_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMB 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMB_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS L3RAMB Single bit error inject: test starting");

	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection", ecc_ctrl, ecc_sts);
				   
	/* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88100000u);
	
    /* Run one shot test for DSS L3RAMA 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",ecc_ctrl, ecc_sts);
					   
	if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMB Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {

        DebugP_log("\n DSS L3RAMB Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem);
    }


    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMB_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMB_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMB 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMB_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS L3RAMB Double bit error inject: starting");

    /* Run one shot test for DSS L3RAMA 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88100000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection", ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);

    ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMB Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {
        DebugP_log("\n DSS L3RAMB Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMB_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMC_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMC 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMC_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS DSS_L3RAMC Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88200000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS DSS_L3RAMC 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS DSS_L3RAMC Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS DSS_L3RAMC Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMC_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMC_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMC 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMC_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS L3RAMC Double bit error inject: starting");

    /* Run one shot test for DSS L3RAMC 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88200000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMC Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS L3RAMC Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMC_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMD_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMD 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMD_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS DSS_L3RAMD Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88280000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS DSS_L3RAMD 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS DSS_L3RAMD Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS DSS_L3RAMD Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMC_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      SDL_Test_run_DSS_L3RAMD_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_L3RAMD 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t SDL_Test_run_DSS_L3RAMD_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS L3RAMD Double bit error inject: starting");

    /* Run one shot test for DSS L3RAMD 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x88280000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS L3RAMD Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS L3RAMD Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of SDL_Test_run_DSS_L3RAMD_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_MAILBOX_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_MAILBOX 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_MAILBOX_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS MAILBOX Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x83100000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x83100000u);

    /* Run one shot test for DSS MAILBOX 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS MAILBOX Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS MAILBOX Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_MAILBOX_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_MAILBOX_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_MAILBOX 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_MAILBOX_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS MAILBOX Double bit error inject: starting");

    /* Run one shot test for DSS MAILBOX 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x83100000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS MAILBOX Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS MAILBOX Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_MAILBOX_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_CM4_MAILBOX_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_CM4_MAILBOX 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_CM4_MAILBOX_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS CM4_MAILBOX Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x83100000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48000000u);

    /* Run one shot test for DSS CM4_RAM_B0 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS CM4_MAILBOX Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS CM4_MAILBOX Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_CM4_MAILBOX_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_CM4_MAILBOX_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_CM4_MAILBOX 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_CM4_MAILBOX_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS CM4_RAM_B0 Double bit error inject: starting");

    /* Run one shot test for DSS CM4_MAILBOX 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x48000000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_ONCE,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS CM4_MAILBOX Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS CM4_MAILBOX Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_CM4_MAILBOX_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_A0X_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_A0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_A0_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_A0 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x83100000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06160000u);

    /* Run one shot test for DSS TPTC_A0 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_A0 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_A0 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_A0_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_A0_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_A0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_A0_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_A0 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_A0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06160000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_A0_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_A0 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_A0 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_A0_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_A1_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_A1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_A1_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_A1 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06180000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_A1 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_A1 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_A1 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_A0_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_A1_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_A1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_A1_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_A1 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_A1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06180000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_A1_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_A1 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_A1 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_A1_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_B0_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_B0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_B0_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B0 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x061a0000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_B0 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_B0 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_B0 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_B0_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_B0_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_B0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_B0_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B0 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_B0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x061a0000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_B0_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_B0 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_B0 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_B0_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_B1_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_B1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_B1_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B1 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x061c0000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_B1 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_B1 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_B1 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_B1_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_B1_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_B1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_B1_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B1 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_B1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x061c0000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_B1_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_B1 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_B1 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_B1_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C0_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C0 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C0_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_C0 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x061e0000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_C0 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C0 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_C0 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C0_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C0_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C0 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C0_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B1 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_C0 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x061e0000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C0_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C0 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_C0 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C0_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C1_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C1 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C1_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_C1 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06200000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_C1 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C1 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_C1 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C1_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C1_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C1 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C1_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B1 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_C1 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06200000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C1_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C1 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_C1 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C1_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C2_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C2 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C2_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_C2 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06220000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_C2 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C2 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_C2 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C2_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C2_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C2 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C2_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B1 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_C2 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06220000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C2_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C2 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_C2 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C2_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C3_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C3 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C3_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_C3 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06240000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_C3 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C3 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_C3 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C3_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C3_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C3 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C3_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B1 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_C3 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06240000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C3_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C3 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_C3 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C3_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C4_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C4 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C4_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_C4 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06260000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_C4 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C4 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_C4 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C4_1Bit_N_RowRepeatInjectTest() */

/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C4_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C4 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C4_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B1 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_C4 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06260000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C4_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C4 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_C4 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C4_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C5_1Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C5 1 bit inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C5_1Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0 ;
		
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_C5 Single bit error inject: test starting");

    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06280000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0040);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                   ecc_ctrl, ecc_sts);

    /* Run one shot test for DSS TPTC_C5 1 bit error */
    injectErrorConfig.flipBitMask = 0x10;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_1BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0040);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC single bit error injection",
                       ecc_ctrl, ecc_sts);
					   
    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C5 Single bit error inject at pErrMem 0x%p test failed",
                    injectErrorConfig.pErrMem);
        retVal = -1;
    } else {
        
        DebugP_log("\n DSS TPTC_C5 Single bit error inject at pErrMem 0x%p",
                   injectErrorConfig.pErrMem, testLocationValue);
    }


    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C5_1Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_Test_run_DSS_TPTC_C5_2Bit_N_RowRepeatInjectTest
 *
 * @brief   Execute ECC DSS_TPTC_C5 2 bit Inject test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t ECC_Test_run_DSS_TPTC_C5_2Bit_N_RowRepeatInjectTest(void)
{
    SDL_ErrType_t result;
    int32_t retVal=0;
    uint32_t ecc_ctrl = 0, ecc_sts = 0;

    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    volatile uint32_t testLocationValue;
	
	SDL_dss_ecc_aggRegs *pEccAggrRegs = ((SDL_dss_ecc_aggRegs *)((uintptr_t)0x060A0000u));

    DebugP_log("\n DSS TPTC_B1 Double bit error inject: starting");

    /* Run one shot test for DSS TPTC_C5 2 bit error */
    /* Note the address is relative to start of ram */
    injectErrorConfig.pErrMem = (uint32_t *)(0x06280000u);
	
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	ecc_sts = SDL_REG32_RD(0x060A0140);
	DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values before ECC double bit error injection",
                   ecc_ctrl, ecc_sts);

    injectErrorConfig.flipBitMask = 0x101;
    result = SDL_ECC_injectError(SDL_DSS_ECC_AGG,
                                 SDL_DSS_ECC_AGG_DSS_TPTC_C5_FIFO_ECC_RAM_ID,
                                 SDL_INJECT_ECC_ERROR_FORCING_2BIT_N_ROW_REPEAT,
                                 &injectErrorConfig);
								 
	ecc_ctrl = SDL_REG32_RD(&pEccAggrRegs->CONTROL);
	/* Access the memory where injection is expected */
    testLocationValue = injectErrorConfig.pErrMem[0];
    ecc_sts = SDL_REG32_RD(0x060A0140);
            
    DebugP_log("\n DSS ECC control Register = 0x%p and Status Register = 0x%p values after ECC double bit error injection",
                       ecc_ctrl, ecc_sts);

    if (result != SDL_PASS ) {
        DebugP_log("\n DSS TPTC_C5 Double bit error inject: at pErrMem 0x%p: fixed location once test failed",
                    injectErrorConfig.pErrMem);
       retVal = -1;
    } else {

        DebugP_log("\n DSS TPTC_C5 Double bit error inject at pErrMem 0x%p ",
                   injectErrorConfig.pErrMem);
    }

    return retVal;
}/* End of ECC_Test_run_DSS_TPTC_C5_2Bit_N_RowRepeatInjectTest() */
/*********************************************************************
 * @fn      ECC_sdlFuncTest
 *
 * @brief   Execute ECC sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t DSS_ECC_FuncTest(void)
{
    int32_t result;
    int32_t retVal = 0;

    DebugP_log("\n\n ECC Safety Example tests: starting");
	
	/* Initialize ECC L3RAMA Memory */
	result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID);
	if (result != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

		retVal = -1;
	} else {
		DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
	}
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMA_1Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMA_1Bit_N_RowRepeatInjectTest has failed...");
        }
    }
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMA_1Bit_OnceInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMA_1Bit_OnceInjectTest has failed...");
        }
    }
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMA_1Bit_N_RowOnceInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMA_1Bit_RowOnceInjectTest has failed...");
        }
    }
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMA_1Bit_N_RepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMA_1Bit_N_RepeatInjectTest has failed...");
        }
    }
    /* Initialize ECC L3RAMA Memory */
	result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_L3RAMA_ECC_RAM_ID);
	if (result != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

		retVal = -1;
	} else {
		DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
	}
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMA_2Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMA_2Bit_N_RowRepeatInjectTest has failed...");
        }
    }
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMA_2Bit_OnceInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMA_2Bit_OnceInjectTest has failed...");
        }
    }
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMA_2Bit_N_RowOnceInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMA_2Bit_N_RowOnceInjectTest has failed...");
        }
    }
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMA_2Bit_N_RepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMA_2Bit_N_RepeatInjectTest has failed...");
        }
    }
	/* Initialize ECC L3RAMB Memory */
	result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_L3RAMB_ECC_RAM_ID);
	if (result != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

		retVal = -1;
	} else {
		DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
	}
    if (retVal == 0)
    {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMB_1Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMB_1Bit_N_RowRepeatInjectTest has failed...");
        }
    }
    if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMB_2Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_TPTC_A0_2Bit_N_RowRepeatInjectTest has failed...");
        }
    }
	/* Initialize ECC L3RAMC Memory */
	result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_L3RAMC_ECC_RAM_ID);
	if (result != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

		retVal = -1;
	} else {
		DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
	}
    if (retVal == 0)
    {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMC_1Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMC_1Bit_N_RowRepeatInjectTest has failed...");
        }
    }
    if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMC_2Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMC_2Bit_N_RowRepeatInjectTest has failed...");
        }
    }
	/* Initialize ECC L3RAMD Memory */
	result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_L3RAMD_ECC_RAM_ID);
	if (result != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

		retVal = -1;
	} else {
		DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
	}
    if (retVal == 0)
    {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMD_2Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMD_2Bit_N_RowRepeatInjectTest has failed...");
        }
    }
    if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = SDL_Test_run_DSS_L3RAMD_1Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_L3RAMD_1Bit_N_RowRepeatInjectTest has failed...");
        }
    }
    /* Initialize ECC MAILBOX Memory */
	result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_MAILBOX_ECC_RAM_ID);
	if (result != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

		retVal = -1;
	} else {
		DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
	}
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = ECC_Test_run_DSS_MAILBOX_1Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_MAILBOX_1Bit_N_RowRepeatInjectTest has failed...");
        }
    }
	if (retVal == 0) {
        SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = ECC_Test_run_DSS_MAILBOX_2Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_MAILBOX_2Bit_N_RowRepeatInjectTest has failed...");
        }
    }
    /* Initialize ECC CM4 MAILBOX Memory */
	result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_CM4_MAILBOX_ECC_RAM_ID);
	if (result != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

		retVal = -1;
	} else {
		DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
	}
	if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = ECC_Test_run_DSS_CM4_MAILBOX_1Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_CM4_MAILBOX_1Bit_N_RowRepeatInjectTest has failed...");
        }
    }
	if (retVal == 0) {
        SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = ECC_Test_run_DSS_CM4_MAILBOX_2Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_CM4_MAILBOX_2Bit_N_RowRepeatInjectTest has failed...");
        }
    }
    /* Initialize ECC HWA AGGR Memory */
	result = SDL_ECC_initMemory(SDL_DSS_ECC_AGG, SDL_DSS_ECC_AGG_DSS_HWA_PARAM_RAM_ECC_RAM_ID);
	if (result != SDL_PASS) {
		/* print error and quit */
		DebugP_log("ECC_Test_init: Error initializing Memory of DSS ECC AGGR: result = %d\n", result);

		retVal = -1;
	} else {
		DebugP_log("\nECC_Test_init: Initialize of DSS ECC AGGR Memory is complete \n");
	}
    if (retVal == 0)
    {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[0],NULL,NULL);
        result = ECC_Test_run_DSS_HWA_PARAM_1Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_HWA_PARAM_2Bit_N_RowRepeatInjectTest has failed...");
        }
    }
    if (retVal == 0) {
		SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_TestparamsDSS[1],NULL,NULL);
        result = ECC_Test_run_DSS_HWA_PARAM_2Bit_N_RowRepeatInjectTest();
        if (result != SDL_PASS) {
            retVal = -1;
            DebugP_log("\n ECC_Test_run_DSS_HWA_PARAM_1Bit_N_RowRepeatInjectTest has failed...");
        }
    }

	return retVal;
}
/*********************************************************************
 * @fn      ECC_sdl_funcTest
 *
 * @brief   ECC Function module test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 ********************************************************************/
int32_t DSS_ECC_sdl_funcTest(void)
{
    int32_t testResult;

	testResult = DSS_ECC_Test_init();
	if (testResult != 0)
    {
        DebugP_log("\nDSS ECC SDL API Init: unsuccessful");
        return SDL_EFAIL;
    }
    testResult = DSS_ECC_FuncTest();
	if (testResult != 0)
    {
        DebugP_log("\n ECC SDL API tests: unsuccessful");
        return SDL_EFAIL;
    }
    return (testResult);
}/* End of ECC_sdl_funcTest() */

/* Nothing past this point */
