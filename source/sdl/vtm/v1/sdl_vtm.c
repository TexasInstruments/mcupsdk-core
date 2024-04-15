/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdint.h>
#include <sdl/sdl_vtm.h>
#include <sdl/include/sdl_types.h>

/*=============================================================================
 *  internal macros
 *===========================================================================*/
#define SDL_VTM_VALUES_ARE_UNINITIALIZED    (-1)
/* Delay for Reg Reads */
#define SDL_VTM_DOUT_REG_READ_DELAY         (100)

/* Below macros are used to enable VTM warm reset. */
#define SDL_MSS_TOPRCM_WARM_RST_CFG                            (0x53200044U)
#define SDL_TOP_RCM_LOCK0_KICK0                                (0x00001008U)
#define SDL_TOP_RCM_LOCK0_KICK1                                (0x0000100CU)
#define KICK_LOCK_VAL                                          (0x00000000U)
#define KICK0_UNLOCK_VAL                                       (0x01234567U)
#define KICK1_UNLOCK_VAL                                       (0x0FEDCBA8U)

#define SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_MASK      (0x00000700U)
#define SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_SHIFT     (0x00000008U)
#define SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_RESETVAL  (0x00000000U)
#define SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_MAX       (0x00000007U)

#define SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_MASK      (0x00007000U)
#define SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_SHIFT     (0x0000000CU)
#define SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_RESETVAL  (0x00000000U)
#define SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_MAX       (0x00000007U)

/*=============================================================================
 *  global variables
 *===========================================================================*/

/*=============================================================================
 *  Internal functions
 *===========================================================================*/

 /**
 * Design: PROC_SDL-1165,PROC_SDL-1302,PROC_SDL-1300
 */
int32_t SDL_VTM_initTs(const SDL_VTM_configTs *pConfig)
{
    int32_t             sdlResult = SDL_PASS;
    SDL_VTM_adc_code    ts0TsHotTempAdcCode;
    SDL_VTM_adc_code    ts0TsColdTempAdcCode;
    SDL_VTM_adc_code    ts0AlertHotTempAdcCode;
    SDL_VTM_adc_code    ts0AlertColdTempAdcCode;
    SDL_VTM_adc_code    ts1TsHotTempAdcCode;
    SDL_VTM_adc_code    ts1TsColdTempAdcCode;
    SDL_VTM_adc_code    ts1AlertHotTempAdcCode;
    SDL_VTM_adc_code    ts1AlertColdTempAdcCode;


    if(pConfig   == NULL_PTR)
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        /* Enable BGR bit */
        SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_CFG),\
                        TOP_CTRL_TSENSE_CFG_TSENSE_CFG_BGROFF, \
                         SDL_VTM_BGR_ON);
        /* Enable AIP bit */
        SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_CFG),\
                        TOP_CTRL_TSENSE_CFG_TSENSE_CFG_AIPOFF, \
                        SDL_VTM_AIP_ON);

        /* Enable TMPSOFF bit */
        SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_CFG),\
                        TOP_CTRL_TSENSE_CFG_TSENSE_CFG_TMPSOFF, \
                        SDL_VTM_TMPSOFF_ON);

       if(pConfig->cfgTs0Tshut == 1U)
        {
            /* Enable over ride so that configured values are used for
               thermal shutdown. */
            SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_TSHUT), \
                     TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE, \
                     SDL_VTM_OVERRIDE_PATTERN);

            /* Convert the given temperature from milli centigrade to adc
               code.*/
            if(SDL_PASS == SDL_VTM_tsConvTempToAdc(
                             pConfig->ts0_ts_hot_temp_in_milli_degree_celsius,
                             &ts0TsHotTempAdcCode))
            {
                /* Write the converted ADC code to register*/
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_TSHUT), \
                        TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT, \
                        ts0TsHotTempAdcCode);
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }

            /* Convert the given temperature from milli centigrade to adc
               code.*/
            if(SDL_PASS == SDL_VTM_tsConvTempToAdc(
                              pConfig->ts0_ts_cold_temp_in_milli_degree_celsius,
                              &ts0TsColdTempAdcCode))
            {
                /* Write the converted ADC code to register*/
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_TSHUT), \
                        TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD, \
                        ts0TsColdTempAdcCode);
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }
        }

        if(pConfig->cfgTs0Alert == 1U)
        {
            /* Convert the given temperature from milli centigrade to adc code.*/
            if(SDL_PASS == SDL_VTM_tsConvTempToAdc(
                    pConfig->ts0_alert_hot_temp_in_milli_degree_celsius,
                    &ts0AlertHotTempAdcCode))
            {
                /* Write the converted ADC code to register*/
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_ALERT),\
                            TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT,\
                            ts0AlertHotTempAdcCode);
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }

            /* Convert the given temperature from milli centigrade to adc code.*/
            if(SDL_PASS == SDL_VTM_tsConvTempToAdc(
                 pConfig->ts0_alert_cold_temp_in_milli_degree_celsius,
                 &ts0AlertColdTempAdcCode))
             {
                /* Write the converted ADC code to register*/
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_ALERT),
                                TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD,\
                                ts0AlertColdTempAdcCode);
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }

            /* Enable Cold comparator output */
            SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_CNTL),
                            TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD,
                            SDL_VTM_MASK_COLD);
            /* Enable hot comparator output */
            SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_CNTL),
                            TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT,
                            SDL_VTM_MASK_HOT);
            /* Enable low threhold comparator output */
            SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_CNTL),
                            TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD,
                            SDL_VTM_MASK_LOW_TH);
        }

       if(pConfig->cfgTs1Tshut == 1U)
        {
            /* Enable over ride so that configured values are used for thermal shutdown. */
            SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_TSHUT),
                            TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE,
                            SDL_VTM_OVERRIDE_PATTERN);

            /* Convert the given temperature from milli centigrade to adc code.*/
            if(SDL_PASS == SDL_VTM_tsConvTempToAdc(
                      pConfig->ts1_ts_hot_temp_in_milli_degree_celsius,
                      &ts1TsHotTempAdcCode))
            {
                /* Write the converted ADC code to register*/
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_TSHUT),
                                TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT,
                                ts1TsHotTempAdcCode);
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }

            /* Convert the given temperature from milli centigrade to adc code.*/
            if(SDL_PASS == SDL_VTM_tsConvTempToAdc(
                      pConfig->ts1_ts_cold_temp_in_milli_degree_celsius,
                      &ts1TsColdTempAdcCode))
            {
                /* Write the converted ADC code to register*/
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_TSHUT),
                                TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD,
                                ts1TsColdTempAdcCode);
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }
        }

        if(pConfig->cfgTs1Alert == 1U)
        {
            /* Convert the given temperature from milli centigrade to adc code.*/
            if(SDL_PASS == SDL_VTM_tsConvTempToAdc(
                      pConfig->ts1_alert_hot_temp_in_milli_degree_celsius,
                      &ts1AlertHotTempAdcCode))
            {
                /* Write the converted ADC code to register*/
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_ALERT),
                                TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_HOT,
                                ts1AlertHotTempAdcCode);
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }

            /* Convert the given temperature from milli centigrade to adc code.*/
           if(SDL_PASS ==  SDL_VTM_tsConvTempToAdc(
                      pConfig->ts1_alert_cold_temp_in_milli_degree_celsius,
                    &ts1AlertColdTempAdcCode))
            {
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_ALERT),
                                TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD,
                                ts1AlertColdTempAdcCode);
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }

            /* Enable Cold comparator output */
            SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_CNTL),
                            TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD,
                            SDL_VTM_MASK_COLD);

             /* Enable hot comparator output */
            SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_CNTL),
                            TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT,
                            SDL_VTM_MASK_HOT);

             /* Enable low threhold comparator output */
            SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_CNTL),
                            TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD,
                            SDL_VTM_MASK_LOW_TH);
        }
    }
    return (sdlResult);
}

/**
 * Design: PROC_SDL-7525
 */
void SDL_VTM_enableTs(uint32_t sensorSelect, uint8_t delay)
{
    /* Enable all the required sensors */
    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_CFG),
                    TOP_CTRL_TSENSE_CFG_TSENSE_CFG_SENSOR_SEL,
                    sensorSelect);

    /* Configure the delay. 0 is not a valid delay. */
    if((delay > 0U) && (delay < SDL_VTM_MAXDELAY))
    {
        SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_CFG),
                        TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY, delay);
    }
    else
    {
        /* In case of 0 configured delay, configure max delay. */
        SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_CFG),
                    TOP_CTRL_TSENSE_CFG_TSENSE_CFG_DELAY, SDL_VTM_MAXDELAY);
    }
}

/**
 * Design: PROC_SDL-7526
 */
void SDL_VTM_enableTc(void)
{
    /* Enable the Temperature Controller. */
    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_CFG),
                    TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE, SDL_VTM_TSENSE_ON);
}

 /**
 * Design: PROC_SDL-7528
 */
void SDL_VTM_disableTc(void)
{
    /* Dnable the Temperature Controller */
    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_CFG),
                    TOP_CTRL_TSENSE_CFG_TSENSE_CFG_ENABLE, SDL_VTM_TSENSE_OFF);
}

 /**
 * Design: PROC_SDL-1165,PROC_SDL-1303,PROC_SDL-1304
 */
int32_t SDL_VTM_getTemp(SDL_VTM_InstTs instance, uint32_t *pTempVal)
{
    SDL_VTM_adc_code           	adcCode=0xFF;
    int32_t                     pMilliDegreeTempVal;
    int32_t sdlResult     =     SDL_PASS;

    switch(instance)
    {
        case SDL_VTM_INSTANCE_TS_0:
            adcCode = (SDL_VTM_adc_code)SDL_REG32_FEXT(      \
                 (SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_RESULT), \
                TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP);
        break;

        case SDL_VTM_INSTANCE_TS_1:
            adcCode = (SDL_VTM_adc_code)SDL_REG32_FEXT((SDL_TOP_CTRL_U_BASE+ \
                                                     SDL_VTM_TSENSE1_RESULT), \
                                 TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP);
        break;

        case SDL_VTM_INSTANCE_TS_2:
            adcCode = (SDL_VTM_adc_code)SDL_REG32_FEXT((SDL_TOP_CTRL_U_BASE+ \
                                                    SDL_VTM_TSENSE2_RESULT),  \
                                 TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP);
        break;

        case SDL_VTM_INSTANCE_TS_3:
            adcCode = (SDL_VTM_adc_code)SDL_REG32_FEXT((SDL_TOP_CTRL_U_BASE+ \
                                                     SDL_VTM_TSENSE3_RESULT), \
                                 TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP);
        break;

        default:
            adcCode = 0xFF;
        break;
    }

    /* Convert the ADC code to milli degrees */
    sdlResult = SDL_VTM_tsConvADCToTemp (adcCode, &pMilliDegreeTempVal);
    if(sdlResult == SDL_PASS)
    {
        *pTempVal = (uint32_t)pMilliDegreeTempVal;
    }

    return (sdlResult);
}

/**
 * Design: PROC_SDL-7529, PROC_SDL-7530
 */
int32_t SDL_VTM_setAlertTemp(SDL_VTM_InstTs instance, int32_t alertThHot,
                             int32_t alertThCold)
{
    SDL_VTM_adc_code  	tsAlertHotadcCode=0xFF;
    SDL_VTM_adc_code  	tsAlertColdadcCode=0xFF;
    int32_t sdlResult = SDL_PASS;

    if(instance > SDL_VTM_INSTANCE_TS_1)
    {
         sdlResult = SDL_EBADARGS;
    }
    else
    {
        sdlResult = SDL_VTM_tsConvTempToAdc(alertThHot,
                                            &tsAlertHotadcCode);
        if(sdlResult == SDL_PASS)
        {
            sdlResult = SDL_VTM_tsConvTempToAdc(alertThCold,
                                                &tsAlertColdadcCode);
        }
        if(sdlResult == SDL_PASS)
        {
            switch(instance)
            {
                case SDL_VTM_INSTANCE_TS_0:
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_ALERT),\
                       TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT,  \
                                                         tsAlertHotadcCode);

                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_ALERT),\
                      TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_COLD, \
                                                        tsAlertColdadcCode);
                break;

                case SDL_VTM_INSTANCE_TS_1:
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_ALERT),\
                        TOP_CTRL_TSENSE0_ALERT_TSENSE0_ALERT_ALERT_THRHLD_HOT, \
                                                         tsAlertHotadcCode);
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_ALERT),\
                        TOP_CTRL_TSENSE1_ALERT_TSENSE1_ALERT_ALERT_THRHLD_COLD,\
                                                        tsAlertColdadcCode);
                break;

                default:
                break;
            }
        }
    }
    return (sdlResult);
}

/**
 * Design: PROC_SDL-7531, PROC_SDL-7532
 */
int32_t SDL_VTM_setTShutTemp(SDL_VTM_InstTs instance,
                             int32_t tshutThHot, int32_t tshutThCold)
{
    SDL_VTM_adc_code  	tsTshutHotadcCode=0xFF;
    SDL_VTM_adc_code  	tsTshutColdadcCode=0xFF;
    int32_t sdlResult = SDL_PASS;

    if(instance > SDL_VTM_INSTANCE_TS_1)
    {
         sdlResult = SDL_EBADARGS;
    }
    else
    {
        sdlResult = SDL_VTM_tsConvTempToAdc(tshutThHot,
                                            &tsTshutHotadcCode);
        if(sdlResult == SDL_PASS)
        {
            sdlResult = SDL_VTM_tsConvTempToAdc(tshutThCold,
                                                &tsTshutColdadcCode);
        }
        if(sdlResult == SDL_PASS)
        {
            switch(instance)
            {
                case SDL_VTM_INSTANCE_TS_0:
                    /* Enable over ride so that configured values are used for thermal shutdown. */
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_TSHUT),
                            TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_EFUSE_OVERRIDE,
                            SDL_VTM_OVERRIDE_PATTERN);
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_TSHUT),\
                        TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRHLD_HOT, \
                                                        tsTshutHotadcCode);
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_TSHUT),\
                        TOP_CTRL_TSENSE0_TSHUT_TSENSE0_TSHUT_TSHUT_THRSHLD_COLD, \
                                                        tsTshutColdadcCode);
                break;

                case SDL_VTM_INSTANCE_TS_1:
                    /* Enable over ride so that configured values are used for thermal shutdown. */
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_TSHUT),
                            TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_EFUSE_OVERRIDE,
                            SDL_VTM_OVERRIDE_PATTERN);
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_TSHUT),\
                        TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRHLD_HOT, \
                                                          tsTshutHotadcCode);
                    SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_TSHUT),\
                        TOP_CTRL_TSENSE1_TSHUT_TSENSE1_TSHUT_TSHUT_THRSHLD_COLD, \
                                                        tsTshutColdadcCode);
                break;
                default:
                break;
            }
        }
    }
    return (sdlResult);
}

/**
 * Design: PROC_SDL-7533
 */
int32_t SDL_VTM_setClearInterrupts(SDL_VTM_InstTs instance,
                                   uint8_t  hotIntr,
                                   uint8_t  coldIntr,
                                   uint8_t  lowThresholdIntr)
{
    int32_t sdlResult     =     SDL_PASS;

    if(instance > SDL_VTM_INSTANCE_TS_1)
    {
         sdlResult = SDL_EBADARGS;
    }
    else
    {
        switch(instance)
        {
            case SDL_VTM_INSTANCE_TS_0:
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_CNTL),\
                        TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_HOT, \
                            hotIntr);
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_CNTL),\
                        TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_COLD, \
                            coldIntr);
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_CNTL),\
                        TOP_CTRL_TSENSE0_CNTL_TSENSE0_CNTL_MASK_LOW_THRHLD, \
                            lowThresholdIntr);
            break;

            case SDL_VTM_INSTANCE_TS_1:
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_CNTL),\
                        TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_HOT, \
                            hotIntr);
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_CNTL),\
                        TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_COLD, \
                            coldIntr);
                SDL_REG32_FINS((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_CNTL),\
                        TOP_CTRL_TSENSE1_CNTL_TSENSE1_CNTL_MASK_LOW_THRHLD, \
                            lowThresholdIntr);
            break;
            default:
            break;
        }
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1164,PROC_SDL-1305,PROC_SDL-1306
 */
int32_t SDL_VTM_getSensorStatus(SDL_VTM_Stat_val *pStat_val)
{
    int32_t  sdlResult = SDL_EFAIL;

    /* argument checks */
    if(pStat_val == NULL_PTR)
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        pStat_val->s0HotEvent = (uint8_t) SDL_REG32_FEXT(( \
                                                SDL_TOP_CTRL_U_BASE+   \
                                                SDL_VTM_TSENSE_STATUS),\
                            TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_HOT);
        pStat_val->s0ColdEvent = (uint8_t) SDL_REG32_FEXT(( \
                            SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_STATUS), \
                            TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_COLD);
        pStat_val->s0LowThresholdEvent = (uint8_t) SDL_REG32_FEXT(( \
                            SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_STATUS), \
                    TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S0_LOW_THRHLD);

        pStat_val->s1HotEvent = (uint8_t) SDL_REG32_FEXT(( \
                            SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_STATUS), \
                            TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_HOT);
        pStat_val->s1ColdEvent = (uint8_t) SDL_REG32_FEXT(( \
                            SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_STATUS), \
                            TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_COLD);
        pStat_val->s1LowThresholdEvent = (uint8_t) SDL_REG32_FEXT(( \
                            SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE_STATUS), \
                    TOP_CTRL_TSENSE_STATUS_TSENSE_STATUS_S1_LOW_THRHLD);

        sdlResult = SDL_PASS;
    }
    return (sdlResult);
}

 /**
 * Design: PROC_SDL-1175,PROC_SDL-1309
 */
int32_t SDL_VTM_getStaticRegistersTs(SDL_VTM_staticRegsTs *pStaticRegs)
{
    const SDL_VTM_cfg1Regs  *p_cfg1;
	uint32_t baseAddr;
    int32_t sdlResult = SDL_EBADARGS;

	(void)SDL_VTM_getBaseAddr(&baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;

    /* arg checked */
    if (pStaticRegs  != NULL_PTR)
    {
        sdlResult = SDL_PASS;
    }

    /* if good args are passed */
    if(sdlResult == SDL_PASS)
    {
        /* Read all elements */
        pStaticRegs->vtm_ts_cfg    = p_cfg1->TSENSE_CFG;
        pStaticRegs->vtm_ts0_tshut = p_cfg1->TSENSE0_TSHUT;
        pStaticRegs->vtm_ts0_alert = p_cfg1->TSENSE0_ALERT;
        pStaticRegs->vtm_ts0_ctrl  = p_cfg1->TSENSE0_CNTL;
        pStaticRegs->vtm_ts1_tshut = p_cfg1->TSENSE1_TSHUT;
        pStaticRegs->vtm_ts1_alert = p_cfg1->TSENSE1_ALERT;
        pStaticRegs->vtm_ts1_ctrl  = p_cfg1->TSENSE1_CNTL;
    }

    return (sdlResult);
}
/**
 * Design: PROC_SDL-7534
 */
int32_t SDL_VTM_enableESMWarmReset(SDL_VTM_InstTs instance)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;
    int32_t sdlResult = SDL_PASS;

    if(instance > SDL_VTM_INSTANCE_TS_1)
    {
         sdlResult = SDL_EBADARGS;
    }
    else
    {

        /* Unlock CONTROLSS_CTRL registers */
        /* Unlock TOP_RCM */
        baseAddr = (uint32_t) SDL_TOP_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);      /* KICK 1 */

        switch (instance)
        {
            case SDL_VTM_INSTANCE_TS_0:
                /* Enable the warm reset */
                SDL_REG32_FINS(SDL_MSS_TOPRCM_WARM_RST_CFG, \
                            TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN, \
                            SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_MAX);
            break;

            case SDL_VTM_INSTANCE_TS_1:
                /* Enable the warm reset */
                SDL_REG32_FINS(SDL_MSS_TOPRCM_WARM_RST_CFG, \
                            TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN, \
                            SDL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_MAX);
            break;
            default:
            break;
        }

        /* Lock CONTROLSS_CTRL registers */
        /* Lock TOP_RCM */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK0);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK1);
        SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }
    return sdlResult;
}

/* Nothing past this point */
