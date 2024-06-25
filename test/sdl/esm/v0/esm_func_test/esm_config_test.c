#include <stdint.h>
#include "esm_test_main.h"
#include <kernel/dpl/DebugP.h>

#define SDTF_NUM_RUNALL_TEST_COMMANDS 4U
#define SDL_ESM_PWM_TEST_MAX_NUM     10U

#if defined (SOC_AM64X)

SDL_ESM_config SDTF_esmInitConfig_MCU_config =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0xff0fffffu, 0x7fffffffu, 0x00000007u,
                },
     /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well*/
    .priorityBitmap = {0x00000000u, 0xff0fffffu, 0x7fffffffu, 0x00000007u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0x00000000u, 0xff0fffffu, 0x7fffffffu, 0x00000007u,
                      },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
};

SDL_ESM_config SDTF_esmInitConfig_MAIN =
{
    .esmErrorConfig = {0u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};

#elif defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
SDL_ESM_config SDTF_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                },
     /**< Only DCC events enable:**/
	  /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                        },
    /**< DCC events high priority:**/
    .errorpinBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                      },
    /**< All events high priority:**/
};
#endif

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                         SDL_ESM_IntType esmIntType,
                                         uint32_t grpChannel,
                                         uint32_t index,
                                         uint32_t intSrc,
                                         void *arg);

static uint32_t arg;
void  esm_init(SDL_ESM_Inst esmType)
{
    void *ptr = (void *)&arg;
    SDL_ErrType_t result = SDL_EBADARGS;
    /* Initialize MAIN ESM module */
    if(esmType == SDL_ESM_INST_MAIN_ESM0)
    {
        result = SDL_ESM_init(esmType, &SDTF_esmInitConfig_MAIN,SDL_ESM_applicationCallbackFunction,ptr);
    }
#if defined (SOC_AM64X)
    else
    {
        result = SDL_ESM_init(esmType, &SDTF_esmInitConfig_MCU_config,SDL_ESM_applicationCallbackFunction,ptr);
    }
#endif
    if (result != SDL_PASS) {
        /* print error and quit */
        if(esmType == SDL_ESM_INST_MAIN_ESM0){
			DebugP_log("ESM_ECC_Example_init: Error initializing MAIN ESM: result = %d\r\n", result);
        }
#if defined (SOC_AM64X)
		else{
            DebugP_log("ESM_ECC_Example_init: Error initializing MCU ESM: result = %d\r\n", result);
        }
#endif
    } else {
        if(esmType == SDL_ESM_INST_MAIN_ESM0){
			DebugP_log("\r\nESM_ECC_Example_init: Init MAIN ESM complete \r\n");
        }
#if defined (SOC_AM64X)
		else{
			DebugP_log("\r\nESM_ECC_Example_init: Init MCU ESM complete \r\n");

        }
#endif
	}
}

uint8_t config_param = 1;
int32_t retVal=0;

#if defined (SOC_AM64X)
/*********************************************************************
 * @fn      SDTF_runESMInject_MCU
 *
 * @brief   Execute ESM Inject for MCU
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t sdl_config_MCU(void)
{

    esm_init(SDL_ESM_INST_MCU_ESM0);
    SDL_ESM_setNError(SDL_ESM_INST_MCU_ESM0);
    return retVal;
}

int32_t sdl_configGrp_MCU(void)
{

    esm_init(SDL_ESM_INST_MCU_ESM0);
    SDL_ESM_setNError(SDL_ESM_INST_MCU_ESM0);
    return retVal;
}
#endif

/*********************************************************************
 * @fn      SDTF_runESMInject_MAIN
 *
 * @brief   Execute ESM Inject for MAIN
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */

int32_t sdl_configGrp_MAIN(void)
{

    esm_init(SDL_ESM_INST_MAIN_ESM0);
    SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/*********************************************************************
 * @fn      SDTF_runESMInject_MAIN
 *
 * @brief   Execute ESM Inject for MAIN
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 */
int32_t sdl_config_MAIN(void)
{

    esm_init(SDL_ESM_INST_MAIN_ESM0);
    SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);
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
SDTF_commandList_t SDTF_commandList_config[SDTF_MAX_COMMANDS] =
{
#if defined (SOC_AM64X)
    { "esm_config_MCU",              sdl_config_MCU },
    { "esm_configGrp_MCU",           sdl_configGrp_MCU },
#endif
    { "esm_configGrp_MAIN",          sdl_configGrp_MAIN },
    { "esm_config_MAIN",             sdl_config_MAIN },
};

int32_t sdl_config_test(void)
{
    /* Declarations of variables */
    int32_t retVal = 0;
    int32_t i;

    DebugP_log("\r\n Running all sdr test commands supported\r\n");
    for(i = 0u; i< SDTF_NUM_RUNALL_TEST_COMMANDS; i++) {
        if (SDTF_commandList_config[i].commandFunction!= ((void *)(0u))) {
            retVal = (*SDTF_commandList_config[i].commandFunction)();
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

int32_t sdl_config_pwm_test(void)
{
    /* Declarations of variables */
    int32_t retVal = 0;
    uint32_t errorStatus;
    uint32_t baseAddr;
    esmErrOutMode_t  mode;
    bool prevStatusValue, currStatusValue;
    uint32_t i = 0;

    DebugP_log("\n sdl_config_pwm_test is started. \n");
    SDL_ESM_getBaseAddr(SDL_ESM_INST_MAIN_ESM0, &baseAddr);
    esm_init(SDL_ESM_INST_MAIN_ESM0);

    retVal = SDL_ESM_setPinOutMode(SDL_ESM_INST_MAIN_ESM0, SDL_ESM_PWM_PINOUT);
    if (retVal == SDL_PASS)
    {
        retVal = SDL_ESM_getErrorOutMode(baseAddr, &mode);

        if (retVal == SDL_PASS)
        {
            (mode == SDL_ESM_PWM_PINOUT) ? printf("\n Error out mode is in PWM mode. \n")
                                         : printf("\n Error out mode is in LVL mode. \n");
        }
    }

    do
    {
        retVal = SDL_ESM_getErrPinStatus(baseAddr, &errorStatus);
        currStatusValue = (bool) errorStatus;
        if(i == 0)
        {
            prevStatusValue = !(currStatusValue);
        }

        if (prevStatusValue != currStatusValue)
        {
            DebugP_log("\n Error status Value in PWM mode is = %d. \n", currStatusValue);
            i++;
        }

        prevStatusValue = currStatusValue;
        ClockP_usleep (10);

    } while(i <= SDL_ESM_PWM_TEST_MAX_NUM);

    if (retVal == 0)
    {
        DebugP_log("\n All tests have passed. \n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed \n");
    }

return retVal;
}

