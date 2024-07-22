# ESM {#SDL_ESM_PAGE}

[TOC]

The Error Signaling Module (ESM) aggregates safety-related events and/or errors resulting from diagnostics from throughout the device into one location that can be monitored internally or externally. It allows designation of events as high priority or low priority interrupts, and also directly manipulate an I/O error pin to signal an external hardware (e.g. external monitor) that an error has occurred. Using the Error Signaling Module allows a single place in the SoC to manage these events, take corrective action, and clear error pin signaling external hardware if appropriate corrective action is taken. An external controller can monitor the error pin and is able to reset the device or keep the system in a safe, known state.

There can be multiple ESM instances on a device, and each one can support multiple error events that come from the various IP blocks in the device. Detailed information regarding the error events supported can be found in the device-specific TRM.

SDL provides the application with an interface to the ESM. Using SDL ESM, the application can initialize the ESM and listen for the desired error events from the device as well as configure the priority and error pin signaling of the events. As part of initialization of the ESM by the application, the application registers a callback for being notified of the error events. This is called by the SDL's ESM Handler. SDL provides APIs for initializing and configuring the ESM for detecting the HW errors. The application can use the APIs to configure the ESM interrupts and priorities. Also, configuration of the error pin reporting is supported. SDL also provides the ability for the application to manipulate the error pin.

## Features Supported

The SDL provides support for the ESM through:

* ESM Configuration API
* ESM Handler for error handling and notification
* ESM nError Pin manipulation API
* ESM static register/written config readback APIs

The following can be configured in ESM per instance for each supported event:

* Enable of interrupt for the event
* Interrupt priority for the event
* Enable nErrorPin assertion for the event

The ESM Handler is responsible for handling the errors at runtime and notifying the application via registered callback. The registered callback will run in interrupt context.

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL ESM API usage by the application to set up the ESM to monitor for specific safety events.

In this example, the application configures the ESM for a particular ESM Instance. If there are multiple ESM instances on a device, then all instances for which the application wishes to listen for error events need to be configured. In this example we will configure only one of the ESM Instances

First, we need to decide which event(s) we want to enable, at what priority, and what error pin behavior is desired. To do this, we can reference the |__PART_SOC_TRM__| to find the ESM error events for each ESM instance. In this example, let's say that we have reviewed the TRM, and we have decided that we want to

* monitor ESM events 10, 11, and 16 for a particular instance
* configure event 10 and 11 to be of High priority and to signal on the error pin
* configure event 16 to be Low priority and not to signal on the error pin

Include the below file to access the APIs

\code{.c}
#include <sdl/esm/v0/sdl_esm.h>
\endcode

We will set up our ESM config like this:

\code{.c}

        SDL_ESM_config esmInitConfig =
        {
            .esmErrorConfig = {0u, 8u}, /* Self test error config - error event to use for self test */
            .enableBitmap = {0x00010c00u, 0x00000000u, 0x00000000u},
            /**< Enable events 10, 11 and 16. All other events disabled. */
            .priorityBitmap = {0x00000c00u, 0x00000000u, 0x00000000u},
            /**< Events 10 and 11 high priority, event 16 low priority */
            .errorpinBitmap = {0x00000c00u, 0x00000000u, 0x00000000u},
            /**< Events 10 and 11 signal the error pin, event 16 does not */
        };
\endcode

Additionally, application should define the callback function through which it will receive ESM notifications, for example:

\code{.c}
        int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                                    SDL_ESM_IntType esmIntrType,
                                                    uint32_t grpChannel,
                                                    uint32_t index,
                                                    uint32_t intSrc,
                                                    void *arg)
        {
            int32_t retVal = SDL_PASS;
            DebugP_log("\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                        "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                        esmInst, esmIntrType, grpChannel, index, intSrc);
            DebugP_log("  Take action \n");
            /* Any additional application specific actions can be added here */

            return retVal;
        }
\endcode

Now we can call the SDL_ESM_init API:
\cond SOC_AM64X || SOC_AM243X
\code{.c}

        ret = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &esmInitConfig, SDL_ESM_applicationCallbackFunction,ptr);
        if (ret != SDL_PASS)
        {
            // handle error
        }
\endcode

When error events occur, the Application's registered callback will be invoked and the application can take action as needed.


If an error pin is asserted due to an error event, the application can additionally decide to clear the error pin:

\code{.c}

        SDL_ESM_clrNError(SDL_ESM_INST_MCU_ESM0);
\endcode

Note that the pin will remain active for the minimum time interval even after being cleared. Once the minimum time interval expires, the clear will take affect.

The application can also force the error pin active:

\code{.c}

        SDL_ESM_setNError(SDL_ESM_INST_MCU_ESM0);
\endcode

The SDL ESM also provides APIs to assist in implementing diagnostics related to static config register readback and readback of written config.
In order to verify the written config, after calling SDL_ESM_init, SDL_ESM_verifyConfig may be called:

\code{.c}

        ret = SDL_ESM_verifyConfig(SDL_ESM_INST_MCU_ESM0, esmInitConfig);
        if (ret != SDL_PASS)
        {
            // verification failed
        }
\endcode

In order to readback the static config:

\code{.c}

        ret = SDL_ESM_getStaticRegisters(SDL_ESM_INST_MCU_ESM0, &staticRegs);
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}

        ret = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &esmInitConfig, SDL_ESM_applicationCallbackFunction,ptr);
        if (ret != SDL_PASS)
        {
            // handle error
        }
\endcode

When error events occur, the Application's registered callback will be invoked and the application can take action as needed.


If an error pin is asserted due to an error event, the application can additionally decide to clear the error pin:

\code{.c}

        SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
\endcode

Note that the pin will remain active for the minimum time interval even after being cleared. Once the minimum time interval expires, the clear will take affect.

The application can also force the error pin active:

\code{.c}

        SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);
\endcode

The SDL ESM also provides APIs to assist in implementing diagnostics related to static config register readback and readback of written config.
In order to verify the written config, after calling SDL_ESM_init, SDL_ESM_verifyConfig may be called:

\code{.c}

        ret = SDL_ESM_verifyConfig(SDL_ESM_INST_MAIN_ESM0, esmInitConfig);
        if (ret != SDL_PASS)
        {
            // verification failed
        }
\endcode

In order to readback the static config:

\code{.c}

        ret = SDL_ESM_getStaticRegisters(SDL_ESM_INST_MAIN_ESM0, &staticRegs);
\endcode
\endcond
## API

\ref SDL_ESM_MODULE
