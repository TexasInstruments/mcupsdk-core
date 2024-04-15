# VTM {#SDL_VTM_PAGE}

[TOC]

The Voltage and Thermal Management (VTM) controls the temperature monitors in the die. The VTM provides interrupt and event generation for overtemperature through the Error Signaling Module (ESM). It supports 3 different alerts:

* an early alert so that firmware/software can start doing high-temperature thermal management,
* a follow-up interrupt for reporting to firmware/software that the temperature has dropped to a safe level,
* and a late interrupt whose high level can be used to trigger a hardware-voltage domain reset


SDL supports configuration of the VTM voltage domain and temperature warning. The 3 different alerts are also supported via the SDL ESM Handler, like all other ESM events. The SDL API can also be used to configure the VTM Maximum temperature threshold.

The VTM module provides the following functionality  for the application

* Ability to initialize VTM
* Ability to check the status of the temperature sensor based on alert temperature
* Ability to clear and contorl the VTM event inerrupts

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL VTM API usage by the application to set up the VTM for monitoring for events. Events can be monitored by enabling the events in the associated ESM instance.

Configure the VTM Thresholds and enable events
\code{.c}
        int32_t retValue;
        uint32_t temp0, temp1, temp2, temp3;
        int32_t alert_th_hot, alert_th_cold;
        SDL_VTM_Stat_val SDL_VTM_Stat_value;
        void (*pTSenseHInterruptHandler)(void *);
        void  (*pTSenseLInterruptHandler)(void *);
        SDL_DPL_HwipParams intrParams;
        pSDL_DPL_HwipHandle SDL_TS_HiHwiPHandle, SDL_TS_LoHwiPHandle;

        /* Register Interrupt Handlers */
        pTSenseHInterruptHandler = &SDL_TS_hiInterruptHandler;
        pTSenseLInterruptHandler = &SDL_TS_loInterruptHandler;

        intrParams.intNum      = SDL_R5FSS0_CORE0_INTR_TSENSE_H;
        intrParams.callback    = (*pTSenseHInterruptHandler);
        SDL_DPL_registerInterrupt(&intrParams, &SDL_TS_HiHwiPHandle);

        intrParams.intNum      = SDL_R5FSS0_CORE0_INTR_TSENSE_L;
        intrParams.callback    = (*pTSenseLInterruptHandler);
        SDL_DPL_registerInterrupt(&intrParams, &SDL_TS_LoHwiPHandle);

        /* Enable Interrupts */
        SDL_DPL_enableInterrupt(SDL_R5FSS0_CORE0_INTR_TSENSE_H);
        SDL_DPL_enableInterrupt(SDL_R5FSS0_CORE0_INTR_TSENSE_L);

        /* UC1 - Receive Hot and Cold Interrupt. */
        DebugP_log("\r\n UC1 : ");
        SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, 0, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
        alert_th_hot = 58000; // temperatube in mc
        alert_th_cold = 72000; // temperatube in mc
        /* Device temperature is 0x40 => 70000mc */
        SDL_VTM_setAlertTemp(SDL_VTM_INSTANCE_TS_0, alert_th_hot, alert_th_cold);
        retValue = SDL_VTM_initTs(&SDL_VTM_configTempSense);
        SDL_VTM_enableTs(SDL_VTM_SENSOR_SEL0, 0);
        SDL_VTM_enableTc();
        SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);
        SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_1, &temp1);
        SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_2, &temp2);
        SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_3, &temp3);

        SDL_VTM_getSensorStatus(&SDL_VTM_Stat_value);
\endcode

Convert from Temperature to ADC Code
\code{.c}
        int32_t temp_val = 64000; // temperature in milli degrees Celsius
        SDL_VTM_adc_code adc_code;

        SDL_VTM_tsConvTempToAdc(temp_val, &adc_code);
\endcode

Convert from ADC Code to Temperature
\code{.c}
        SDL_VTM_adc_code adc_code = 50;
        int32_t temp_val; // temperature in milli degrees Celsius

        SDL_VTM_tsConvAdcToTemp(adc_code, &temp_val);
\endcode

Get Current Temperature Value
\code{.c}
        uint32_t temp0;
        /* Get current temperature value */
        SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);
\endcode

Acknowledge and Disable an interrupt
\code{.c}
        SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, SDL_VTM_MASK_HOT, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
\endcode

## API

\ref SDL_VTM_API
