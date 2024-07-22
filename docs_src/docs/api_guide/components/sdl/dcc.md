# DCC {#SDL_DCC_PAGE}

[TOC]

The Dual Clock Comparator (DCC) is used to determine the accuracy of a clock signal during the time executaion of an application. The DCC measure the frequency of a selectable clock sources using another input clock as a reference. The clock sources as well as the accuracy are programmed by the application.

The DCC can also be configured to operate in single-shot or continuous mode. In single-shot mode, DCC performs a one-time countdown where DCC stops operation when the counters reach 0. A completion interrupt is raised and the status can be checked. In continuous mode, DCC reloads both counts with the seed value upon completion without error. In case of an error, an ESM error event is raised.

## Features Supported

The DCC module provides the following functionality:

* Ability to configure a DCC instance
* Ability to verify the configuration of the DCC instance
* Ability to enable a DCC instance
* Ability to disable a DCC instance
* Ability to query the status of a DCC instance including current configuration, error information, interrupt status, and counter values
* Ability to clear the pending interrupt(s)
* Ability to readback the DCC static registers

Errors detected by the DCC module during operation are reported via ESM error (application will receive via ESM application callback).

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- The Input Source clock mapping for each DCC instance can be found in the DCC section of the TRM

## Example Usage

The following shows an example of SDL DCC API usage by the application.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_dcc.h>
\endcode

Initialize the ESM to report DCC errors

\cond SOC_AM64X || SOC_AM243X
\code{.c}
SDL_ESM_config DCC_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config - not used in this test*/
    .enableBitmap = {0x00000007u, 0x00000020u, 0x00000000u},
    /*                        ^           ^                            */
    /*                        Main ESM    MCU DCC0 event enabled       */
    /**< Enabling Main domain ESM output and MCU Domain DCC events     */

    .priorityBitmap = {0x0000003u, 0x00000020u, 0x00000000u},
    /**< All events high priority: except low-priority Main ESM output */

    .errorpinBitmap = {0x00000003u, 0x00000020u, 0x00000000u},
    /**< All high priority events to error pin */
};

SDL_ECC_init(SDL_ESM_INST_MCU_ESM0, &DCC_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, NULL);
\endcode
\endcond
\cond SOC_AM243X
\code{.c}
SDL_ESM_config DCC_Test_esmInitConfig_MCU =
{

     /**< All high priority events to error pin */
        .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */


};

SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &DCC_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, NULL);
\endcode
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_ESM_config DCC_Test_esmInitConfig_MAIN =
{
      .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                },
     /**< Only DCC events enable:**/
	  /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                        },
   /**< All events high priority: except low-priority Main ESM output */
    .errorpinBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                      },
    /**< All high priority events to error pin */
};

 SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &DCC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, NULL);
\endcode
\endcond
\cond  SOC_AWR294X
\code{.c}
SDL_ESM_NotifyParams gESM_Params=

{
    ESM_ERROR_GROUP_1,
     /* ESM group number */
    DCCA_MSS_ESM_ERROR,
     /* DCC error pin number connected to ESM */
    0U,
    /* Set the interrupt priority level to high or low. Applicable to Group 1 errors only. */
    TRUE,
    /* Enable failure influence on ERROR pin. Applicable to Group 1 errors only. */
    NULL,
    /* Argument passed back when the Notify function is invoked. */
    &SDL_ESM_applicationCallbackFunction
    /* Notify function called by the ESM driver. */

};

SDL_ESM_OpenParams esmOpenParams=
{
    TRUE
    /* boolean value to indicate if old ESM pending errors should be cleared or not
          This field will be set by SysCfg.
};

 SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &gESM_Params, &esmOpenParams, NULL);
\endcode
\endcond

\cond SOC_AM64X || SOC_AM243X
Configure MCU DCC Instance 0 seed values and clocks

\code{.c}
SDL_DCC_Config configParams;

/* Select Input0 to be CLOCK0[2] */
configParams.clk0Src = SDL_DCC_CLK0_SRC_CLOCK0_2;
/* Select Input1 to be CLKSRC2 */
configParams.clk1Src = SDL_DCC_CLK1_SRC_CLOCKSRC2;

/* Not shown:
 * Determine the clock frequencies for the sources
 * Figure out the seed values for successful completion
 * Check DCC example code for reference
 */

/* Assign the seed values for the clock selections */
configParams->clk0Seed = clk0_seed_value;
configParams->clk1Seed = clk1_seed_value;
configParams->clk0ValidSeed = clk0_valid_seed_value;

retVal = SDL_DCC_configure(SDL_DCC_INST_MCU_DCC0, &configParams);

Configure MCU DCC Instance 0 for continuous mode

\code{.c}
/* Select continuous mode */
configParams.mode    = SDL_DCC_MODE_CONTINUOUS;
\endcode

Or, configure the MCU DCC Instance 0 for single-shot mode.

\code{.c}
/* Select continuous mode */
configParams.mode    = SDL_DCC_MODE_SINGLE_SHOT_2;

retVal = SDL_DCC_configure(SDL_DCC_INST_MCU_DCC0, &configParams);
\endcode

If configuring single-shot mode, also register and enablei the done interrupt. In single-shot mode, the completion (Done) interrupt will happen at the end of the test if there are no errors. If error happens, then ESM interrupt will occur as well as the DCC error interrupt.

\code{.c}
int32_t                 status = SystemP_SUCCESS;
HwiP_Params             hwiPrms;

/* Register interrupt */
HwiP_Params_init(&hwiPrms);
hwiPrms.intNum      = APP_DCC_INTR_NUM;
hwiPrms.callback    = &SDL_DCCAppDoneIntrISR;
status              = HwiP_construct(&gDCCHwiObject, &hwiPrms);

/* Enable the Done interrupt for DCC instance */
SDL_DCC_enableIntr(SDL_DCC_INST_MCU_DCC0, SDL_DCC_INTERRUPT_DONE);

\endcode

The done interrupt handler should clear the interrupt

\code{.c}
static void SDL_DCCAppDoneIntrISR(void *arg)
{
    SDL_DCC_clearIntr(SDL_DCC_INST_MCU_DCC0, SDL_DCC_INTERRUPT_DONE);
}
\endcode

Enable DCC Instance

\code{.c}
SDL_DCC_enable(SDL_DCC_INST_MCU_DCC0);
\endcode
\endcond


\cond SOC_AM243X
Configure MCU DCC Instance 0 seed values and clocks

\code{.c}
SDL_DCC_Config configParams;

/* Select Input0 to be CLOCK0[2] */
configParams.clk0Src = SDL_DCC_CLK0_SRC_CLOCK0_2;
/* Select Input1 to be CLKSRC2 */
configParams.clk1Src = SDL_DCC_CLK1_SRC_CLOCKSRC2;

/* Not shown:
 * Determine the clock frequencies for the sources
 * Figure out the seed values for successful completion
 * Check DCC example code for reference
 */

/* Assign the seed values for the clock selections */
configParams->clk0Seed = clk0_seed_value;
configParams->clk1Seed = clk1_seed_value;
configParams->clk0ValidSeed = clk0_valid_seed_value;

retVal = SDL_DCC_configure(SDL_DCC_INST_MCU_DCC0, &configParams);

Configure MCU DCC Instance 0 for continuous mode

\code{.c}
/* Select continuous mode */
configParams.mode    = SDL_DCC_MODE_CONTINUOUS;
\endcode

Or, configure the MCU DCC Instance 0 for single-shot mode.

\code{.c}
/* Select continuous mode */
configParams.mode    = SDL_DCC_MODE_SINGLE_SHOT_2;

retVal = SDL_DCC_configure(SDL_DCC_INST_MCU_DCC0, &configParams);
\endcode

If configuring single-shot mode, also register and enablei the done interrupt. In single-shot mode, the completion (Done) interrupt will happen at the end of the test if there are no errors. If error happens, then ESM interrupt will occur as well as the DCC error interrupt.

\code{.c}
int32_t                 status = SystemP_SUCCESS;
HwiP_Params             hwiPrms;

/* Register interrupt */
HwiP_Params_init(&hwiPrms);
hwiPrms.intNum      = APP_DCC_INTR_NUM;
hwiPrms.callback    = &SDL_DCCAppDoneIntrISR;
status              = HwiP_construct(&gDCCHwiObject, &hwiPrms);

/* Enable the Done interrupt for DCC instance */
SDL_DCC_enableIntr(SDL_DCC_INST_MCU_DCC0, SDL_DCC_INTERRUPT_DONE);

\endcode

The done interrupt handler should clear the interrupt

\code{.c}
static void SDL_DCCAppDoneIntrISR(void *arg)
{
    SDL_DCC_clearIntr(SDL_DCC_INST_MCU_DCC0, SDL_DCC_INTERRUPT_DONE);
}
\endcode

Enable DCC Instance

\code{.c}
SDL_DCC_enable(SDL_DCC_INST_MCU_DCC0);
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AWR294X || SOC_AM261X
Configure MAIN DCC Instance 0 seed values and clocks

\code{.c}
SDL_DCC_Config configParams;

/* Select Input0 to be CLOCK0[2] */
configParams.clk0Src = SDL_DCC_CLK0_SRC_CLOCK0_2;
/* Select Input1 to be CLKSRC2 */
configParams.clk1Src = SDL_DCC_CLK1_SRC_CLOCKSRC2;

/* Not shown:
 * Determine the clock frequencies for the sources
 * Figure out the seed values for successful completion
 * Check DCC example code for reference
 */

/* Assign the seed values for the clock selections */
configParams->clk0Seed = clk0_seed_value;
configParams->clk1Seed = clk1_seed_value;
configParams->clk0ValidSeed = clk0_valid_seed_value;

retVal = SDL_DCC_configure(SDL_DCC_INST_MSS_DCCA, &configParams);

Configure MAIN DCC Instance 0 for continuous mode

\code{.c}
/* Select continuous mode */
configParams.mode    = SDL_DCC_MODE_CONTINUOUS;
\endcode

Or, configure the MCU DCC Instance 0 for single-shot mode.

\code{.c}
/* Select continuous mode */
configParams.mode    = SDL_DCC_MODE_SINGLE_SHOT;

retVal = SDL_DCC_configure(SDL_DCC_INST_MSS_DCCA, &configParams);
\endcode

If configuring single-shot mode, also register and enablei the done interrupt. In single-shot mode, the completion (Done) interrupt will happen at the end of the test if there are no errors. If error happens, then ESM interrupt will occur as well as the DCC error interrupt.

\code{.c}
int32_t                 status = SystemP_SUCCESS;
HwiP_Params             hwiPrms;

/* Register interrupt */
HwiP_Params_init(&hwiPrms);
hwiPrms.intNum      = APP_DCC_INTR_NUM;
hwiPrms.callback    = &SDL_DCCAppDoneIntrISR;
status              = HwiP_construct(&gDCCHwiObject, &hwiPrms);

/* Enable the Done interrupt for DCC instance */
SDL_DCC_enableIntr(SDL_DCC_INST_MSS_DCCA, SDL_DCC_INTERRUPT_DONE);

\endcode

The done interrupt handler should clear the interrupt

\code{.c}
static void SDL_DCCAppDoneIntrISR(void *arg)
{
    SDL_DCC_clearIntr(SDL_DCC_INST_MSS_DCCA, SDL_DCC_INTERRUPT_DONE);
}
\endcode

Enable DCC Instance

\code{.c}
SDL_DCC_enable(SDL_DCC_INST_MSS_DCCA);
\endcode
\endcond
## API

\ref SDL_DCC_MODULE
