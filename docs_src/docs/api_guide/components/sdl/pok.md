# POK {#SDL_POK_PAGE}
POK: Power OK


Introduction
------------

This device has several internal voltage monitors called Power OK (POK). These monitors can be configured to detect undervoltage (UV) or overvoltage (OV) conditions, and to report errors via ESM.
SDL provides the support for programming of the POK modules, including the internal POKs for the POR module. The threshold limits can be configured, and the POK monitoring enabled. Programming is generally done at init time. All POK modules are in the mcu domain.

POK modules are responsible for monitoring and accurately detecting voltage levels. POK modules are capable of monitoring a range of supplies and indicating a failure within the programmable upper and lower threshold limits for the supply being monitored. POKs are used to monitor voltage supply levels with programmable threshold levels. As an example, the POK programmable threshold resolution is 12.5mV for the threshold setting range from 0.475V to 1.35V for under voltage protection.

This module provides the following functionality:

* Ability to initialize a POK
* Ability to verify written POK configuration
* Ability to readback of written configuration

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

Example Usage
-------------

The following shows an example of SDL POK API usage by the application to set up the POK and usage of the POK APIs. Events can be monitored by enabling the events in the associated ESM instance.

Initialize a POK Instance

    .. code:: bash

        int32_t sdlRet;
        SDL_POK_config pokCfg;
        SDL_POK_Inst instance;

        instance = SDL_POR_POKHV_OV_ID;
        pPokCfg.voltDetMode = SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE;
        pPokCfg.trim = 0; // Configure the OV (Over voltage) POK to a small value to force an error
                          // Alternatively, can provide a good value

        sdlRet = SDL_POK_init(instance, &pokCfg);
        if (sdlRet != SDL_PASS)
        {
            UART_printf("SDL_POK_init failed! \n");
        }

        /* If ESM event comes, then Over voltage condition is reached */

Verify the Written Config (can be called after SDL_POK_init)

    .. code:: bash

        int32_t ret;
        
        /* Pass the expected config */
        ret = SDL_POK_verifyConfig(instance, &pokCfg);
        if (ret != SDL_Pass)
        {
            // verification failed
        }

Read back the Static Registers

    .. code:: bash

        int32_t ret;
        SDL_POK_staticRegs staticRegs;

        ret = SDL_POK_getStaticRegisters(instance, &staticRegs);
        if (ret != SDL_PASS)
        {
            // failed to get static regs
        }

## API

\ref SDL_POK_API