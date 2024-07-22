# SDL DPL {#SDL_DPL_PAGE}

[TOC]

The SDL requires the application to provide an DPL implementation for various functionalities in order to keep SDL as OS-agnostic.
Primarily, the SDL requires DPL implementations for interrupt registration and enable/disable, and also for a delay mechanism needed for POK programming.

## Features Supported

The SDL DPL layer provides support for the application to define the following functionalities to be used by SDL:

* Interrupt Enable/Disable
* Interrupt Register/De-register
* Address Translation
* Delay

These features are provided by the application by calling SDL_DPL_init() with the appropriate function pointers.

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- SDL_DPL_init() must be called before using any SDL module that utilizes the SDL DPL APIs.

## Example Usage

The SDL provides a sample implementation of these APIs as part of the SDL examples. The sample implementation re-uses PDK DPL APIs from the SDK in order to provide the services.
This is verified with the following Operating Systems:

* Non-OS (Baremetal)

The existing baremetal sample DPL interface can be used, or the application may implement it's own. To use the existing sample interface:

1. Add the source file dpl_interface.c (located in example/dpl/src/) to the application's build
2. Include the header dpl_interface.h (located in example/dpl/) to the application's source file
3. Call the example's DPL initializtion function SDL_TEST_dplInit() to initialize the DPL. Do this before calling any SDL APIs.

Alternatively, the application may implement it's own DPL. An example of how to do this is shown below

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_dpl.h>
\endcode

Define the DPL APIs

HwiP_Object gHwiObject;
\cond SOC_AM64X
\code{.c}
        pSDL_DPL_HwipHandle SDL_TEST_registerInterrupt(SDL_DPL_HwipParams *pParams)
        {
            HwiP_Params hwipParams;
            HwiP_Params_init(&hwipParams);

            hwipParams.args = (void *)pParams->callbackArg;
            /*
             * For M4F, external interrupt #10 at NVIC is
             * 16 internal interrupts + external interrupt number at NVIC
             */
            hwipParams.intNum = pParams->intNum + 16;
            hwipParams.callback = pParams->callback;

            HwiP_construct(&gHwiObject, &hwipParams);

            return &gHwiObject;
        }

		int32_t SDL_TEST_enableInterrupt(uint32_t intNum)
        {
            HwiP_enableInt(intNum+16);
            return SDL_PASS;
        }

        int32_t SDL_TEST_disableInterrupt(uint32_t intNum)
        {
            HwiP_disableInt(intNum+16);
            return SDL_PASS;
        }
\endcode
\endcond
\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X || SOC_AM261X
\code{.c}
        pSDL_DPL_HwipHandle SDL_TEST_registerInterrupt(SDL_DPL_HwipParams *pParams)
        {
            HwiP_Params hwipParams;
            HwiP_Params_init(&hwipParams);

            hwipParams.args = (void *)pParams->callbackArg;
            hwipParams.intNum = pParams->intNum;
            hwipParams.callback = pParams->callback;

            HwiP_construct(&gHwiObject, &hwipParams);

            return &gHwiObject;
        }

        int32_t SDL_TEST_enableInterrupt(uint32_t intNum)
        {
            HwiP_enableInt(intNum);
            return SDL_PASS;
        }

        int32_t SDL_TEST_disableInterrupt(uint32_t intNum)
        {
            HwiP_disableInt(intNum);
            return SDL_PASS;
        }
\endcode
\endcond
\code{.c}
        void* SDL_TEST_addrTranslate(uint64_t addr, uint32_t size)
        {
            uint32_t transAddr = (uint32_t)(-1);

            transAddr = (uint32_t)AddrTranslateP_getLocalAddr(addr);

            return (void *)transAddr;
        }

        int32_t SDL_TEST_deregisterInterrupt(pSDL_DPL_HwipHandle handle)
        {
            HwiP_destruct(handle);
            return SDL_PASS;
        }
\endcode
Initalize the DPL Interface

\code{.c}
        SDL_DPL_Interface dpl_interface =
        {
            .enableInterrupt = (pSDL_DPL_InterruptFunction) SDL_TEST_enableInterrupt,
            .disableInterrupt = (pSDL_DPL_InterruptFunction) SDL_TEST_disableInterrupt,
            .registerInterrupt = (pSDL_DPL_RegisterFunction) SDL_TEST_registerInterrupt,
            .deregisterInterrupt = (pSDL_DPL_DeregisterFunction) SDL_TEST_deregisterInterrupt,
            .delay = (pSDL_DPL_DelayFunction) ClockP_sleep,
            .addrTranslate = (pSDL_DPL_AddrTranslateFunction) SDL_TEST_addrTranslate
        };

        int32_t main(void)
        {
            SDL_ErrType_t ret = SDL_PASS;

            ret = SDL_DPL_init(&dpl_interface);

            return ret;
        }
\endcode

## API

\ref SDL_DPL_MODULE
