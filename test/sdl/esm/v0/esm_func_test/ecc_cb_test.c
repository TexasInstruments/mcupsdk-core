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
 *  \file     ecc_cb_test.c
 *
 *  \brief    This file contains ESM test code.
 *
 *  \details  ESM tests
 **/
#include <stdint.h>
#include "esm_test_main.h"
#include <kernel/dpl/DebugP.h>

#define SDTF_NUM_RUNALL_TEST_COMMANDS 3

#if defined (SOC_AM64X)
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0xfffffffbu, 0x7fffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                 0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0xffffffffu,0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0xffedfffbu, 0x7fffffffu, 0xffffffffu,
                         0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                         0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                         0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0xffffffffu,0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0xffedfffbu, 0x7fffffffu, 0xffffffffu,
                       0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                       0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
                       0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0xffffffffu,0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};

/* Event BitMap for ECC ESM callback for MAIN */
uint32_t eventBitMapMAIN[SDL_ESM_MAX_EVENT_MAP_NUM_WORDS] =
{
    0x00000000u, 0xfffffffbu, 0x7fffffffu, 0xffffffffu,
    0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
    0xffffffffu, 0xffffffffu, 0xffffffffu, 0xffffffffu,
    0xffffffffu, 0xffffffffu, 0xffffffffu, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0xffffffffu,
};

#elif defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
SDL_ESM_config ECC_Test_esmInitConfig_MAIN =
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

/* Event BitMap for ECC ESM callback for MAIN */
uint32_t eventBitMapMAIN[SDL_ESM_MAX_EVENT_MAP_NUM_WORDS] =
{
    0xffffffffu, 0xffffffffu, 0x1ffbff, 0x00000000u,
};
#endif

extern int32_t SDR_ESM_errorInsert (const SDL_ESM_Inst esmInstType,
                                const SDL_ESM_ErrorConfig_t *esmErrorConfig);

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

extern int32_t SDL_ESM_ECCapplicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                         SDL_ESM_IntType esmIntType,
                                         uint32_t grpChannel,
                                         uint32_t index,
                                         uint32_t intSrc,
                                         void *arg);

extern int32_t SDL_ESM_CCMapplicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                         SDL_ESM_IntType esmIntType,
                                         uint32_t grpChannel,
                                         uint32_t index,
                                         uint32_t intSrc,
                                         void *arg);


/*********************************************************************
 * @fn      SDTF_runESMInject_WKUP
 *
 * @brief   Execute ESM Inject for WKUP
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */


int32_t ecc_cb_test(void)
{
    int32_t retVal=0;
    DebugP_log("\r\n inside ecc testcase \r\n");
    SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &ECC_Test_esmInitConfig_MAIN, SDL_ESM_ECCapplicationCallbackFunction, (void*)0x1);
    SDL_ESM_registerECCCallback(SDL_ESM_INST_MAIN_ESM0,eventBitMapMAIN,
                                    SDL_ESM_ECCapplicationCallbackFunction,
                                    (void*)0x2);
    retVal = SDTF_runESMInjectInstance(SDL_ESM_INST_MAIN_ESM0, 3, 1);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
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
SDTF_commandList_t SDTF_commandList_ecc_cb[SDTF_MAX_COMMANDS] =
{
    { "ecc_cb_test",                ecc_cb_test },
};

int32_t sdl_ecc_cb_test(void)
{
    /* Declarations of variables */
    int32_t retVal = 0;
    int32_t i;

    DebugP_log("\r\n Running all sdr test commands supported");
    for(i = 0u; i< SDTF_NUM_RUNALL_TEST_COMMANDS; i++) {
        if (SDTF_commandList_ecc_cb[i].commandFunction!= ((void *)(0u))) {
            retVal = (*SDTF_commandList_ecc_cb[i].commandFunction)();
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
