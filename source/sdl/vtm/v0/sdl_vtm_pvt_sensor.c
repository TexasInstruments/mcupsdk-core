/**
 * @file  sdl_vtm_pvt_sensor.c
 *
 * @brief
 *  C implementation of the workaround computed temperature array.
 *
 *  Contains the look up table and control command and status query
 *  function definitions
 *
 * @details
 * The VTM Temperature Monitors (TEMPSENSORs) are trimmed during production
 * with resulting values stored in software readable registers.
 *
 * Software should use these register values when translating the
 * Temperature Monitor output codes to temperature values.
 *
 * A bug in the VTM-IP has zero-out the trim bits of the
 * Temperature Monitor IP (PVT). The end result is an increased error
 * in the temperature reading, which is estimated up to +/-20c.
 * The temperature monitoring feature is not usable with such
 * large error.
 *
 * This HW issues is workaround by soft trim by passing the desired trim
 * via the GP eFUSEs.
 *
 * - Use as golden reference the PVT J721e-PVT-Code values, produced by the
 *   J721e-PVTPolynomial for -40c, 30c and 125c and with it, de-compress the
 *   e-fuse AMTV values for a given sensor for the same 3 temperature points.
 * - Now using the AMTV 10-bit values vs the golden reference J7es-PVT-Code
 *   values
 *   create 2 error lines using linear interpolation of the errors.
 * - First using AMTV(-40c) and AMTV(30c) create the error line for that
 *   segment vs J721e-PVT-Code. We call that error line L_err_a1.
 * - Now to find out all the interpolation points In the look-up table for this
 *   segment we simply add L_err_a1 function values to
 *   J721e-PVT-Code values in this segment.
 * - Followed by creating the error line for the segment between AMTV(30c) and
 *   AMTV(125c) . We call that error line L_err_a2.
 * - Now to find out all the interpolation points In the look-up table for this segment we
 *   simply add L_err_a2 function values to J721e-PVT-Code values in this segment.
 * - Finally we extrapolate for the segment between 125c and 150c using the same
 *   method. Simply add L_err_a2 function values to J721e-PVT-Code values in this
 *   segment.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2023, Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS int32_tERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include <string.h>
#include <stdbool.h>
#include <sdl/include/soc_config.h>
#include "sdl_vtm_pvt_sensor.h"
#include "sdl_pvt_sensor_lut.h"
#include <sdl/include/sdl_types.h>
#include <stdint.h>
#include <sdl/vtm/v0/sdl_ip_vtm.h>

extern int32_t gNumTempSensors;
extern int32_t gNumCoreVoltageDomains;

/* Global variables */
int32_t              gSDL_pvt_poly_work_around[SDL_VTM_NUM_OF_SENSOR_WA_COMP][SDL_VTM_NUM_OF_ADC_CODES];
int32_t              gSDL_vtm_pvt_error[SDL_VTM_NUM_EFUSE_REGS];


/* lut_computation done */
bool gSDL_vtm_lut_done[SDL_VTM_NUM_OF_SENSOR_WA_COMP];
int32_t gSDL_lut_computation_init_done = SDL_VTM_VALUES_ARE_UNINITIALIZED;

/* Internal functions */
static void SDL_vtmPrepLookupTable(void);

static void SDL_vtmPrepLookupTable(void)
{
    uint32_t sens_id = 0u;
    int32_t  i;
    int32_t *derived_array = &gSDL_pvt_poly_work_around[0][0];

    if (gSDL_lut_computation_init_done == SDL_VTM_VALUES_ARE_UNINITIALIZED)
    {
        gSDL_vtm_lut_done[sens_id] = FALSE;
    }

    if (gSDL_vtm_lut_done[sens_id] == FALSE)
    {
        for ( i = 0; i < SDL_VTM_NUM_OF_ADC_CODES; i++)
        {
            derived_array[i] = gSDL_pvt_poly[i];
        }
    }

    gSDL_vtm_lut_done[sens_id] = TRUE;
    return;
}

 /**
 * Design: PROC_SDL-1322,PROC_SDL-1323
 */
int32_t SDL_VTM_tsConvADCToTemp (SDL_VTM_adc_code       adc_code,
                                 SDL_VTM_InstTs		instance,
                                 int32_t                *p_milli_degree_temp_val)
{
    int32_t retVal = SDL_PASS;
    const SDL_VTM_cfg1Regs               *p_cfg1;
	uint32_t baseAddr;

	SDL_VTM_getBaseAddr(SDL_VTM_CONFIG_REG_1, &baseAddr);
    p_cfg1 = (SDL_VTM_cfg1Regs *) baseAddr;

    /* Argument check for temperature sensor */
    if (gNumTempSensors == SDL_VTM_VALUES_ARE_UNINITIALIZED)
    {
        SDL_VTM_getSensorVDCount(p_cfg1);
    }

    if ((int32_t)instance <= gNumTempSensors)
    {
        SDL_vtmPrepLookupTable();
    }

    if ((adc_code < (SDL_VTM_adc_code)0) ||
        (adc_code > (SDL_VTM_adc_code)(SDL_VTM_NUM_OF_ADC_CODES-1)))
    {
        retVal = SDL_EBADARGS;
    }

    if ((p_milli_degree_temp_val != NULL_PTR) &&	\
        (retVal                  == SDL_PASS))
    {
        /* for all temp sensors, use the sensor 0 table */
        *p_milli_degree_temp_val = gSDL_pvt_poly_work_around[0][adc_code];
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);
}

 /**
 * Design: PROC_SDL-1320,PROC_SDL-1321
 */
int32_t SDL_VTM_tsConvTempToAdc (int32_t             milli_degree_temp_val,
								SDL_VTM_InstTs 		instance,
                                SDL_VTM_adc_code    *p_adc_code)

{
    int32_t             retVal;
    SDL_VTM_adc_code    low  = (SDL_VTM_adc_code)(0);
    SDL_VTM_adc_code    high = (SDL_VTM_adc_code)(SDL_VTM_NUM_OF_ADC_CODES-1);
    SDL_VTM_adc_code    mid;
	SDL_VTM_InstTs		ts_id;

    /* since pvt sensor 0 is used for all sensors, the input arg is not used */
    ts_id = SDL_VTM_INSTANCE_TS_0;

    if ((milli_degree_temp_val > SDL_VTM_TEMPERATURE_MILLI_DEGREE_C_MAX) ||
        (milli_degree_temp_val < SDL_VTM_TEMPERATURE_MILLI_DEGREE_C_MIN))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    /* Check the temperature sensor ID out of range values */
    if ((int32_t)instance > gNumTempSensors)
    {
        retVal = SDL_EBADARGS;
    }

    if ( (p_adc_code     != NULL_PTR) &&	\
         (retVal         == SDL_PASS))
    {

        SDL_vtmPrepLookupTable();

        /* Binary search to find the adc code */
        while (low < (high - (SDL_VTM_adc_code)1)) {
            mid = (low + high) / 2;
            if (milli_degree_temp_val <= gSDL_pvt_poly_work_around[ts_id][mid])
            {
                high = mid;
                if (milli_degree_temp_val == gSDL_pvt_poly_work_around[ts_id][mid])
                {
                    break;
                }
            }
            else
            {
                low = mid;
            }
        }

        *p_adc_code =  mid;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);
}

 /**
 * Design: PROC_SDL-1338,PROC_SDL-1339
 */
int32_t SDL_VTM_tsSetMaxTOutRgAlertThr( const SDL_VTM_cfg2Regs  *p_cfg2,
										SDL_VTM_InstTs 				instance,
                                       int32_t                 high_temp_in_milli_degree_celsius,
                                       int32_t                 low_temp_in_milli_degree_celsius)
{
    int32_t                 retVal = SDL_EBADARGS;
    volatile                int32_t i;
    SDL_VTM_adc_code        adc_code_h, adc_code_l;
    uint32_t                value;
    SDL_VTM_Ctrlcfg     ts_ctrl_cfg;

    if ((p_cfg2 != NULL_PTR))
    {
        retVal = SDL_VTM_tsConvTempToAdc(high_temp_in_milli_degree_celsius, SDL_VTM_INSTANCE_TS_0, &adc_code_h);
    }

    if (retVal == SDL_PASS)
    {
        retVal = SDL_VTM_tsConvTempToAdc(low_temp_in_milli_degree_celsius, SDL_VTM_INSTANCE_TS_0, &adc_code_l);
    }

    if (retVal == SDL_PASS)
    {
        /*
         * Program maximum temperature out of range thresholds
         * Step 1: set the thresholds to ~123C (sample value for high) and
         * 105C (sample value for low) WKUP_VTM_MISC_CTRL2
         * Step 2: WKUP_VTM_TMPSENS_CTRL_j set the MAXT_OUTRG_EN  bit This is already taken care as per of init
         * Step 3: WKUP_VTM_MISC_CTRL set the ANYMAXT_OUTRG_ALERT_EN  bit
         */

        /* Step 1 */
         ts_ctrl_cfg.valid_map = SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID | \
                                 SDL_VTM_TS_CTRL_RESET_CTRL_VALID      | \
                                 SDL_VTM_TS_CTRL_SOC_VALID             | \
                                 SDL_VTM_TS_CTRL_MODE_VALID;
         value =  0u;
         SDL_REG32_FINS(&value, VTM_CFG2_MISC_CTRL2_MAXT_OUTRG_ALERT_THR0, adc_code_l);
         SDL_REG32_FINS(&value, VTM_CFG2_MISC_CTRL2_MAXT_OUTRG_ALERT_THR, adc_code_h);
         SDL_REG32_WR(&p_cfg2->MISC_CTRL2,value);

         /* Step 2 */
         ts_ctrl_cfg.valid_map = SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID;
         ts_ctrl_cfg.maxt_outrg_alert_en = SDL_VTM_TS_CTRL_MAXT_OUTRG_GEN_ALERT;

         retVal = SDL_VTM_tsSetCtrl (p_cfg2,
                         instance,
                         &ts_ctrl_cfg);

         if (retVal == SDL_PASS)
         {
            /* have some delay before write */
            for (i = 0; i < SDL_VTM_REG_READ_DELAY;)
            {
                i = i + 1;
            }

            /* Step 3 */
            SDL_REG32_FINS(&p_cfg2->MISC_CTRL, \
                           VTM_CFG2_MISC_CTRL_ANY_MAXT_OUTRG_ALERT_EN, \
                           SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_ENABLE);
         }
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);

}

/* Nothing past this point */

