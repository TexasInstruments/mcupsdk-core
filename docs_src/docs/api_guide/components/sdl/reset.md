# RESET {#SDL_RESET_PAGE}

[TOC]

The Reset is used to assert Warm reset by software and detect cause of warm reset. Since SoC DSP Core have Dynamic Power domain, it can be reset individually.



## Features Supported

The RESET module provides the following functionality:

* Ability to assert Warm reset for SoC.
* Ability to assert Local reset for individually DSP Core.
* Ability to detect latest reset cause for SoC.
* Ability to detect Warm reset cause for R5F Core.

For R5F Core here, Warm reset cause can be detected only that its happening or not.
## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

During the use of asserting Warm reset API, program need to be save in NVM. Because after Warm reset, volatile memory content will be gone.
## Example Usage

The following shows an example of SDL RESET API usage by the application.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_reset.h>
\endcode


\code{.c}
For getting the latest cause of reset of SoC, the below API will be used.

int32_t  test_Result;

/* get warm reset cause*/
test_Result=  SDL_getWarmResetCause();

According to return result of this API, Return value type is ENUM as below
typedef enum SDL_SOC_WarmResetCause_e
{
    /**
     * \brief Value specifying Power ON Reset
     */
    SDL_WarmResetCause_POWER_ON_RESET = 0x09U,
    /**
     * \brief Value specifying MSS WDT
     */
    SDL_WarmResetCause_MSS_WDT = 0x0AU,
    /**
     * \brief Value specifying Software Warm Reset
     */
    SDL_WarmResetCause_TOP_RCM_WARM_RESET_CONFIG = 0x0CU,
    /**
     * \brief Value specifying External Pad Reset
     */
    SDL_WarmResetCause_EXT_PAD_RESET = 0x08U,
    /**
     * \brief Value specifying HSM WDT
     */
    SDL_WarmResetCause_HSM_WDT = 0x18U,

}SDL_SOC_WarmResetCause;

if return value is (SDL_WarmResetCause_TOP_RCM_WARM_RESET_CONFIG ), It means SoC reset is happened by Software warm reset.
After checking the reset cause, it will clear the reset cause.

\endcode


\code
This API helps to check that R5F Core reset is happened by Warm reset or not.

int32_t  r5f_Reset_Cause;

/* get R5F core reset cause*/
r5f_Reset_Cause = SDL_r5fGetResetCause();

if(r5f_Reset_Cause == 1U)
{
  DebugP_log("Foe R5F core, Reset cause is Warm Reset.\r\n");
}
else
{
    DebugP_log("Foe R5F core, Reset cause is not warm reset.\r\n");
}

\endcode

\code
This API used to assert Warm reset for SoC.

  /* WARM Reset assert by SW, */
    SDL_generateSwWarmReset();

After running this API, SoC will be reset.

\endcode

\code
This API used to assert Local reset for DSP Core.

   /* Assert Local Reset for DSP Core */
    SDL_rcmDspLocalReset();

After running this API, Only DSP Core will be reset.

\endcode


## API

\ref SDL_RESET_API
