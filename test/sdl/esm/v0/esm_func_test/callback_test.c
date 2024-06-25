/* Copyright (c) 2021 Texas Instruments Incorporated
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
 *  \file     callback_test.c
 *
 *  \brief    This file contains ESM test code.
 *
 *  \details  ESM tests
 **/


#include <kernel/dpl/DebugP.h>
#include "esm_test_main.h"
#define SDTF_NUM_RUNALL_TEST_COMMANDS    10

volatile uint8_t cfg_triggered = 0x0u;

#if defined (SOC_AM64X)
SDL_ESM_config SDTF_esmInitConfig_MCU_appcallback =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0x00000007u, 0xff0fffffu, 0x7fffffffu, 0x00000007u,
                },
     /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well*/
    .priorityBitmap = {0x0000003u, 0xff0ffffeu, 0x7fffffffu, 0x00000007u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0x00000000u, 0xff0fffffu, 0x7fffffffu, 0x00000007u,
                      },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
};

SDL_ESM_config SDTF_esmInitConfig_MAIN_appcallback =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0xfffffffbu, 0x7fffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0xffffffffu,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0xfffffffbu, 0x7fffffffu, 0x00000001u,
                         0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                         0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                         0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0xffffffffu,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0xfffffffbu, 0x7fffffffu, 0xffffffffu,
                       0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                       0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                       0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0xffffffffu,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};
#elif defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
SDL_ESM_config SDTF_esmInitConfig_MAIN_appcallback =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0xffffffffu, 0xffffffffu, 0x1ffbff, 0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
	  /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0xffffffffu, 0xffffffffu, 0x1ffbff, 0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0xffffffffu, 0xffffffffu, 0x1ffbff, 0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};
#endif

extern int32_t SDR_ESM_errorInsert (const SDL_ESM_Inst esmInstType,
                                const SDL_ESM_ErrorConfig_t *esmErrorConfig);

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                         SDL_ESM_IntType esmIntType,
                                         uint32_t grpChannel,
                                         uint32_t index,
                                         uint32_t intSrc,
                                         void *arg);

void  esm_init_appcb(SDL_ESM_Inst esmType)
{
    SDL_ErrType_t result = SDL_EBADARGS;
    /* Initialize MAIN ESM module */
    if(esmType == SDL_ESM_INST_MAIN_ESM0)
    {
        result = SDL_ESM_init(esmType, &SDTF_esmInitConfig_MAIN_appcallback,SDL_ESM_applicationCallbackFunction, NULL);
    }
#if defined (SOC_AM64X)
    else
    {
        result = SDL_ESM_init(esmType, &SDTF_esmInitConfig_MCU_appcallback,SDL_ESM_applicationCallbackFunction, NULL);
    }
#endif

    if (result != SDL_PASS) {
        /* print error and quit */
        if(esmType == SDL_ESM_INST_MAIN_ESM0)
        {
            DebugP_log("ESM_ECC_Example_init: Error initializing MAIN ESM: result = %d\r\n", result);
		}
#if defined (SOC_AM64X)
        else {
            DebugP_log("ESM_ECC_Example_init: Error initializing MCU ESM: result = %d\r\n", result);
        }
#endif
    }
	else {
        if(esmType == SDL_ESM_INST_MAIN_ESM0)
        {
            DebugP_log("\r\nESM_ECC_Example_init: Init MAIN ESM complete \r\n");

        }
#if defined (SOC_AM64X)
		else {
            DebugP_log("\r\nESM_ECC_Example_init: Init MCU ESM complete \r\n");
        }
#endif
    }
}

/*********************************************************************
 * @fn      SDTF_runESMInject
 *
 * @brief   Execute ESM Inject
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
static int32_t SDTF_runESMInjectInstance(SDL_ESM_Inst esmType,
                                         uint32_t groupNumber,
                                         uint32_t bitNumber)
{
    SDL_ErrType_t result;
    int32_t retVal=0;

    SDL_ESM_ErrorConfig_t esmErrorConfig;

    esmErrorConfig.groupNumber = groupNumber;
    esmErrorConfig.bitNumber = bitNumber;

    DebugP_log("\r\n ESM inject: test starting for Esm instance %d \r\n", esmType);

    /* Run esm test 2*/
    result = SDR_ESM_errorInsert(esmType, &esmErrorConfig);

    if (result != SDL_PASS ) {
        DebugP_log("\r\n ESM inject test for Esm instance %d failed \r\n", esmType);
        retVal = -1;
    } else {
        DebugP_log("\r\n ESM inject test for Esm instance %d Done \r\n", esmType);

    }
    SDL_ESM_clrNError(esmType);
    return retVal;
}

static int32_t SDTF_runESMInjectCfgInstance(SDL_ESM_Inst esmType,
                                            uint32_t groupNumber)
{
    uint32_t esmInstBaseAddr;
    int32_t retVal;

    SDL_ESM_getBaseAddr(esmType, &esmInstBaseAddr);
    retVal = SDL_ESM_setCfgIntrStatusRAW (esmInstBaseAddr, groupNumber);
    return retVal;
}

#if defined (SOC_AM64X)
int32_t SDTF_runESMInjectCfg_MCU(void)
{
    int32_t retVal = 0x0;

    cfg_triggered = 0x0;

    esm_init_appcb(SDL_ESM_INST_MCU_ESM0);
    SDTF_runESMInjectCfgInstance(SDL_ESM_INST_MCU_ESM0, 0x1);

    while(cfg_triggered != 0x1);

    cfg_triggered = 0x0;
    return retVal;
}
#endif
int32_t SDTF_runESMInjectCfg_MAIN(void)
{
    int32_t retVal = 0x0;

    cfg_triggered = 0x0;

    esm_init_appcb(SDL_ESM_INST_MAIN_ESM0);
    SDTF_runESMInjectCfgInstance(SDL_ESM_INST_MAIN_ESM0, 0x1);

    while(cfg_triggered != 0x1);

    cfg_triggered = 0x0;
    return retVal;
}

#if defined (SOC_AM64X)
/*********************************************************************
 * @fn      SDTF_runESMInjectHigh_MCU
 *
 * @brief   Execute ESM Inject for MCU
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t SDTF_runESMInjectHigh_MCU(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MCU_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MCU_ESM0, 1, 4);

    return retVal;
}

int32_t SDTF_runESMInjectLow1_MCU(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MCU_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MCU_ESM0, 1, 0);

    return retVal;
}

int32_t SDTF_runESMInjectLow2_MCU(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MCU_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MCU_ESM0, 3, 0);

    return retVal;
}
#endif

/*********************************************************************
 * @fn      SDTF_runESMInjectHigh_MAIN
 *
 * @brief   Execute ESM Inject for MAIN
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */

int32_t SDTF_runESMInjectHigh_MAIN(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MAIN_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MAIN_ESM0, 3, 0);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDTF_runESMInjectLow_MAIN(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MAIN_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MAIN_ESM0, 3, 1);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDTF_runESMInjectSelfTest_MAIN(void)
{
    int32_t retVal=0;
    esm_init_appcb(SDL_ESM_INST_MAIN_ESM0);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MAIN_ESM0, 1, 8);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t ESMIntNumberTest(void)
{
    int32_t retVal=0;
    uint32_t intNum;
    intNum = SDL_ESM_getIntNumber(SDL_ESM_INST_MAIN_ESM0, SDL_ESM_INT_TYPE_MAX);
#if defined(SOC_AM64X)
    if(intNum == 0xffffffffu)
    {
        intNum = SDL_ESM_getIntNumber(SDL_ESM_INST_MCU_ESM0, SDL_ESM_INT_TYPE_MAX);
    }
    else
    {
        retVal = -1;
    }
#endif
    if(intNum == 0xffffffffu)
    {
        intNum = SDL_ESM_getIntNumber(SDL_ESM_INSTANCE_MAX, SDL_ESM_INT_TYPE_MAX);
    }
    else
    {
        retVal = -1;
    }
    return retVal;
}

int32_t Negative_test_priv_file(void)
{
    int32_t retVal=0;
    uint32_t esmInstBaseAddr;
    SDL_ErrType_t result;
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
    result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &SDTF_esmInitConfig_MAIN_appcallback,SDL_ESM_applicationCallbackFunction, NULL);
#elif defined(SOC_AM64X)
    result = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &SDTF_esmInitConfig_MCU_appcallback,SDL_ESM_applicationCallbackFunction, NULL);
#endif
    if(result != SDL_PASS)
    {
        DebugP_log("SDL_ESM_init: failure \r\n");
    }
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
    SDL_ESM_getBaseAddr(SDL_ESM_INST_MAIN_ESM0, &esmInstBaseAddr);
#elif defined(SOC_AM64X)
    SDL_ESM_getBaseAddr(SDL_ESM_INST_MCU_ESM0, &esmInstBaseAddr);
#endif
    if (SDL_ESM_enableIntr(esmInstBaseAddr, 40U) != SDL_PASS)
    {
        DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
    }
    if (SDL_ESM_setIntrPriorityLvl(esmInstBaseAddr, 40U, ESM_INTR_PRIORITY_LEVEL_LOW) != SDL_PASS)
    {
        DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
    }
    if (SDL_ESM_setIntrStatusRAW(esmInstBaseAddr, 40U) != SDL_PASS)
    {
        DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
    }
    if (SDL_ESM_enableGlobalIntr(esmInstBaseAddr) != SDL_PASS)
    {
        DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
    }
    if (SDL_ESM_setInfluenceOnErrPin(esmInstBaseAddr, 40U, true) != SDL_PASS)
    {
        DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
    }
    if (SDL_ESM_setMode(esmInstBaseAddr, ESM_OPERATION_MODE_NORMAL) != SDL_PASS)
    {
        DebugP_log("sdlEsm_apiTest: failure on line no. %d \r\n", __LINE__);
    }
#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
    retVal =SDTF_runESMInjectInstance(SDL_ESM_INST_MAIN_ESM0, 1, 8);
#elif defined(SOC_AM64X)
    retVal =SDTF_runESMInjectInstance(SDL_ESM_INST_MCU_ESM0, 1, 8);
#endif
    return retVal;
}

/* Other commands not covered by run_all */
#define SDTF_NUM_OTHER_TEST_COMMANDS (0u)
#define SDTF_MAX_COMMANDS (SDTF_NUM_RUNALL_TEST_COMMANDS+SDTF_NUM_OTHER_TEST_COMMANDS)

#define SDTF_MAX_COMMAND_LEN (64u)
typedef int32_t (* SDTF_testFunctionPtr_t) (void);

typedef struct SDTF_commandList_s
{
    char commandString[SDTF_MAX_COMMAND_LEN+1];
    SDTF_testFunctionPtr_t commandFunction;
}SDTF_commandList_t;
/* Full list of commands */
SDTF_commandList_t SDTF_commandList[SDTF_MAX_COMMANDS] =
{
#if defined (SOC_AM64X)
    { "esm_injectHigh_MCU",             SDTF_runESMInjectHigh_MCU },
    { "esm_injectLow1_MCU",              SDTF_runESMInjectLow1_MCU },
    { "esm_injectLow2_MCU",              SDTF_runESMInjectLow2_MCU },
	{ "esm_injectCfg_MCU",              SDTF_runESMInjectCfg_MCU },
#endif
    { "esm_injectHigh_MAIN",            SDTF_runESMInjectHigh_MAIN },
    { "esm_injectLow_MAIN",             SDTF_runESMInjectLow_MAIN },
    { "esm_injectSelfTest_MAIN",        SDTF_runESMInjectSelfTest_MAIN },
    { "esm_injectCfg_MAIN",             SDTF_runESMInjectCfg_MAIN },
    { "Negative_test_priv_file",        Negative_test_priv_file },
    { "ESMIntNumberTest",               ESMIntNumberTest },
};

int32_t test_sdr_test(void)
{
    /* Declarations of variables */
    int32_t retVal = 0;
    int32_t i;

    DebugP_log("\r\n Running all sdr test commands supported");
    for(i = 0u; i< SDTF_NUM_RUNALL_TEST_COMMANDS; i++) {
        if (SDTF_commandList[i].commandFunction!= ((void *)(0u))) {
            retVal = (*SDTF_commandList[i].commandFunction)();
            if ( retVal != 0) {
                break;
            }
        }
    }

    if (retVal == 0)
    {
        DebugP_log("\r\n All tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\n Few/all tests Failed \r\n");
    }

return retVal;
}
