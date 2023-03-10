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
        SDL_VTM_configTs cfgTs;
        SDL_VTM_intrCtrl ctrl;
        SDL_VTM_configVd cfgVd;
        
        /* Initialize the config structure */
        memset(&cfgTs, 0x0, sizeof(SDL_VTM_configTs));
        
        /* Set the control bit for the Temperature sensor config to "SET_THR", indicating we want to set the thresholds */
        cfgTs.configTsCtrl = SDL_VTM_VD_CONFIG_CTRL_SET_THR;

        /* Set the Temperature sensor config */
        cfgTs.thr_val.thrValidMap = SDL_VTM_GT_TH1_VALID | \
                                    SDL_VTM_GT_TH2_VALID | \
                                    SDL_VTM_LT_TH0_VALID;
        cfgTs.thr_val.gtTh2En     = TRUE;
        cfgTs.thr_val.gtTh2       = gt_thr2_adc_code;
        cfgTs.thr_val.gtTh1En     = TRUE;
        cfgTs.thr_val.gtTh1       = gt_thr1_adc_code;
        cfgTs.thr_val.ltTh0En     = TRUE;
        cfgTs.thr_val.ltTh0       = lt_thr0_adc_code;

        SDL_VTM_initTs(SDL_VTM_INSTANCE_TS_0 , &cfgTs);

        /* Select all 3 interrupts to enable using "INTR_EN_SET" control bits */
        ctrl = SDL_VTM_VD_GT_THR2_INTR_EN_SET | \
               SDL_VTM_VD_GT_THR1_INTR_EN_SET | \
               SDL_VTM_VD_LT_THR0_INTR_EN_CLR;
        
        /* enable the threshold interrupts */
        SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_1, ctrl);

        /* Initialize the config structure */
        memset(&cfgVd, 0x0, sizeof(SDL_VTM_configVd));
        cfgVd.configVdCtrl = SDL_VTM_VD_CONFIG_CTRL_EVT_SEL;
        cfgVd.vd_temp_evts = SDL_VTM_VD_EVT_SELECT_TEMP_SENSOR_0;
        
        /* enable the tracking of temperature events on this VD */
        SDL_VTM_initVd(SDL_VTM_INSTANCE_VD_DOMAIN_1, &cfgVd);
\endcode

Convert from Temperature to ADC Code
\code{.c}
        int32_t temp_val = 64000; // temperature in milli degrees Celsius
        SDL_VTM_adc_code adc_code;

        SDL_VTM_tsConvTempToAdc(temp_val, SDL_VTM_INSTANCE_TS_0,  &adc_code);
\endcode

Convert from ADC Code to Temperature
\code{.c}
        SDL_VTM_adc_code adc_code = 404;
        int32_t temp_val; // temperature in milli degrees Celsius

        SDL_VTM_tsConvAdcToTemp(adc_code, SDL_VTM_INSTANCE_TS_0, &temp_val);
\endcode

Get Current Temperature Value
\code{.c}    
        SDL_VTM_iStat_read_ctrl readCtrl;
        SDL_VTM_adc_code adc_code;
         
        /* Get current temperature value */
        readCtrl = SDL_VTM_TS_READ_DATA_OUT_VAL;
        SDL_VTM_getSensorStatus(iSDL_VTM_INSTANCE_TS_0, &readCtrl, &statusVal);
        adc_code = statusVal.data_out; // data_out has the current temperature
\endcode

Acknowledge and Disable an interrupt
\code{.c}          
        SDL_VTM_intrCtrl ctrl;
        
        /* Ack and disable the interrupt, by clearing the pending and enable bits */
        ctrl = (SDL_VTM_VD_GT_THR2_INTR_EN_CLR | \
                SDL_VTM_VD_GT_THR2_INTR_RAW_CLR);
        SDL_VTM_intrCntrl(SDL_VTM_INSTANCE_VD_DOMAIN_1, ctrl);
\endcode

## API

\ref SDL_VTM_API
