#include <sdl/sdl_esm.h>
#include <stdint.h>
#include "esm_test_main.h"
#include <kernel/dpl/DebugP.h>

#define SDTF_NUM_RUNALL_TEST_COMMANDS 4
#if defined (SOC_AM64X) || defined (SOC_AM243X)

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
#endif
#if defined (SOC_AM64X)
#if defined (M4F_CORE)
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
#endif
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (R5F_CORE)
SDL_ESM_config SDTF_esmInitConfig_MAIN =
{
    .esmErrorConfig = {0u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0xffffffffu, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u,
                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                         0x00000000u, 0x00000000u,0xffffffffu, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                       0x00000000u, 0x00000000u, 0xffffffffu, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                 0x00000000u,
                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */
};
#endif
#endif

#if defined (SOC_AM263X)
SDL_ESM_config SDTF_esmInitConfig_MAIN =
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
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    else
    {
        result = SDL_ESM_init(esmType, &SDTF_esmInitConfig_MCU_config,SDL_ESM_applicationCallbackFunction,ptr);
    }
#endif	
    if (result != SDL_PASS) {
        /* print error and quit */
        if(esmType == SDL_ESM_INST_MAIN_ESM0){
			DebugP_log("ESM_ECC_Example_init: Error initializing MAIN ESM: result = %d\n", result);
        }
#if defined (SOC_AM64X)||defined (SOC_AM243X)
		else{
            DebugP_log("ESM_ECC_Example_init: Error initializing MCU ESM: result = %d\n", result);
        }
#endif
    } else {
        if(esmType == SDL_ESM_INST_MAIN_ESM0){
			DebugP_log("\nESM_ECC_Example_init: Init MAIN ESM complete \n");
        }
#if defined (SOC_AM64X)|| defined (SOC_AM243X)
		else{
			DebugP_log("\nESM_ECC_Example_init: Init MCU ESM complete \n");
            
        }
#endif
	}
}

uint8_t config_param = 1;
int32_t retVal=0;

#if defined (SOC_AM64X) || defined (SOC_AM243X)
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
#if defined (SOC_AM64X)|| defined (SOC_AM243X)
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

    DebugP_log("\n Running all sdr test commands supported");
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
        DebugP_log("\n All tests have passed. \n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed \n");
    }

return retVal;
}