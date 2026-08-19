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
#include <kernel/dpl/ClockP.h>
#include "sdl_ip_vtm_priv.h"

/* Delay (in us) after Reg Writes */
#define SDL_VTM_REG_WRITE_DELAY             (uint64_t)(10)

/* Global variables */
int32_t              gSDL_pvt_poly_work_around[SDL_VTM_NUM_OF_SENSOR_WA_COMP][SDL_VTM_NUM_OF_ADC_CODES];
int32_t              gSDL_vtm_pvt_error[SDL_VTM_NUM_EFUSE_REGS];


/* Internal functions */
static void SDL_vtmPrepLookupTable(void);

static void SDL_vtmPrepLookupTable(void)
{
    /* lut_computation done */
    static bool gSDL_vtm_lut_done[SDL_VTM_NUM_OF_SENSOR_WA_COMP] = {(bool)false};

    /* Polynomial when no work around is needed */
    static const int32_t gSDL_pvt_poly_golden[] = {
        -49002, -48677, -48352, -48028, -47704, -47381, -47057, -46734, -46412, -46090,
        -45768, -45446, -45125, -44804, -44484, -44163, -43844, -43524, -43205,
        -42886, -42567, -42249, -41931, -41614, -41297, -40980, -40663, -40347,
        -40031, -39716, -39400, -39085, -38771, -38457, -38143, -37829, -37516,
        -37203, -36890, -36578, -36266, -35955, -35643, -35332, -35022, -34712,
        -34402, -34092, -33783, -33474, -33165, -32857, -32549, -32241, -31934,
        -31626, -31320, -31013, -30707, -30401, -30096, -29791, -29486, -29182,
        -28877, -28573, -28270, -27967, -27664, -27361, -27059, -26757, -26455,
        -26154, -25853, -25552, -25252, -24952, -24652, -24353, -24054, -23755,
        -23456, -23158, -22860, -22563, -22265, -21968, -21672, -21376, -21079,
        -20784, -20488, -20193, -19899, -19604, -19310, -19016, -18723, -18429,
        -18136, -17844, -17551, -17259, -16968, -16676, -16385, -16094, -15804,
        -15514, -15224, -14934, -14645, -14356, -14067, -13779, -13491, -13203,
        -12916, -12628, -12342, -12055, -11769, -11483, -11197, -10912, -10627,
        -10342, -10057, -9773, -9489, -9206, -8922, -8639, -8357, -8074, -7792,
        -7510, -7229, -6947, -6666, -6386, -6105, -5825, -5546, -5266, -4987, -4708,
        -4429, -4151, -3873, -3595, -3318, -3041, -2764, -2487, -2211, -1935, -1659,
        -1384, -1108, -834, -559, -285, -11, 263, 537, 810, 1083, 1355, 1628, 1900,
        2171, 2443, 2714, 2985, 3255, 3526, 3796, 4066, 4335, 4604, 4873, 5142,
        5410, 5679, 5946, 6214, 6481, 6748, 7015, 7281, 7548, 7813, 8079, 8344,
        8609, 8874, 9139, 9403, 9667, 9931, 10194, 10457, 10720, 10983, 11245,
        11507, 11769, 12030, 12292, 12553, 12813, 13074, 13334, 13594, 13854, 14113,
        14372, 14631, 14889, 15148, 15406, 15664, 15921, 16178, 16435, 16692, 16948,
        17205, 17461, 17716, 17972, 18227, 18482, 18736, 18991, 19245, 19498, 19752,
        20005, 20258, 20511, 20764, 21016, 21268, 21520, 21771, 22022, 22273, 22524,
        22774, 23025, 23274, 23524, 23774, 24023, 24272, 24520, 24769, 25017, 25265,
        25512, 25760, 26007, 26254, 26500, 26747, 26993, 27239, 27484, 27730, 27975,
        28220, 28464, 28708, 28953, 29196, 29440, 29683, 29927, 30169, 30412, 30654,
        30896, 31138, 31380, 31621, 31862, 32103, 32344, 32584, 32824, 33064, 33304,
        33543, 33783, 34022, 34260, 34499, 34737, 34975, 35213, 35450, 35687, 35924,
        36161, 36397, 36634, 36870, 37105, 37341, 37576, 37811, 38046, 38281, 38515,
        38749, 38983, 39217, 39450, 39683, 39916, 40149, 40381, 40614, 40846, 41077,
        41309, 41540, 41771, 42002, 42233, 42463, 42693, 42923, 43153, 43382, 43611,
        43840, 44069, 44297, 44526, 44754, 44981, 45209, 45436, 45663, 45890, 46117,
        46343, 46570, 46796, 47021, 47247, 47472, 47697, 47922, 48147, 48371, 48595,
        48819, 49043, 49266, 49490, 49713, 49935, 50158, 50380, 50603, 50824, 51046,
        51268, 51489, 51710, 51931, 52151, 52372, 52592, 52812, 53031, 53251, 53470,
        53689, 53908, 54127, 54345, 54563, 54781, 54999, 55217, 55434, 55651, 55868,
        56085, 56301, 56517, 56733, 56949, 57165, 57380, 57595, 57810, 58025, 58239,
        58454, 58668, 58882, 59095, 59309, 59522, 59735, 59948, 60160, 60373, 60585,
        60797, 61009, 61220, 61432, 61643, 61854, 62064, 62275, 62485, 62695, 62905,
        63115, 63324, 63534, 63743, 63952, 64160, 64369, 64577, 64785, 64993, 65200,
        65408, 65615, 65822, 66029, 66236, 66442, 66648, 66854, 67060, 67266, 67471,
        67676, 67881, 68086, 68291, 68495, 68699, 68903, 69107, 69311, 69514, 69717,
        69920, 70123, 70326, 70528, 70730, 70932, 71134, 71336, 71537, 71738, 71939,
        72140, 72341, 72541, 72742, 72942, 73142, 73341, 73541, 73740, 73939, 74138,
        74337, 74535, 74733, 74932, 75130, 75327, 75525, 75722, 75919, 76116, 76313,
        76510, 76706, 76902, 77098, 77294, 77490, 77685, 77881, 78076, 78271, 78465,
        78660, 78854, 79048, 79242, 79436, 79630, 79823, 80016, 80210, 80402, 80595,
        80788, 80980, 81172, 81364, 81556, 81747, 81939, 82130, 82321, 82512, 82702,
        82893, 83083, 83273, 83463, 83653, 83843, 84032, 84221, 84410, 84599, 84788,
        84976, 85165, 85353, 85541, 85728, 85916, 86103, 86291, 86478, 86665, 86851,
        87038, 87224, 87411, 87597, 87782, 87968, 88154, 88339, 88524, 88709, 88894,
        89078, 89263, 89447, 89631, 89815, 89999, 90182, 90366, 90549, 90732, 90915,
        91098, 91280, 91463, 91645, 91827, 92009, 92191, 92372, 92554, 92735, 92916,
        93097, 93277, 93458, 93638, 93818, 93998, 94178, 94358, 94537, 94717, 94896,
        95075, 95254, 95433, 95611, 95790, 95968, 96146, 96324, 96501, 96679, 96856,
        97033, 97211, 97387, 97564, 97741, 97917, 98093, 98269, 98445, 98621, 98797,
        98972, 99147, 99322, 99497, 99672, 99847, 100021, 100196, 100370, 100544,
        100718, 100891, 101065, 101238, 101411, 101584, 101757, 101930, 102103,
        102275, 102447, 102619, 102791, 102963, 103135, 103306, 103478, 103649,
        103820, 103991, 104161, 104332, 104502, 104673, 104843, 105013, 105182,
        105352, 105522, 105691, 105860, 106029, 106198, 106367, 106535, 106704,
        106872, 107040, 107208, 107376, 107544, 107711, 107878, 108046, 108213,
        108380, 108546, 108713, 108880, 109046, 109212, 109378, 109544, 109710,
        109875, 110041, 110206, 110371, 110536, 110701, 110866, 111030, 111195,
        111359, 111523, 111687, 111851, 112015, 112178, 112342, 112505, 112668,
        112831, 112994, 113157, 113319, 113482, 113644, 113806, 113968, 114130,
        114292, 114453, 114615, 114776, 114937, 115098, 115259, 115420, 115580,
        115741, 115901, 116061, 116221, 116381, 116541, 116701, 116860, 117019,
        117179, 117338, 117497, 117655, 117814, 117973, 118131, 118289, 118447,
        118605, 118763, 118921, 119078, 119236, 119393, 119550, 119707, 119864,
        120021, 120178, 120334, 120490, 120647, 120803, 120959, 121115, 121270,
        121426, 121581, 121737, 121892, 122047, 122202, 122356, 122511, 122666,
        122820, 122974, 123128, 123282, 123436, 123590, 123744, 123897, 124050,
        124204, 124357, 124510, 124662, 124815, 124968, 125120, 125273, 125425,
        125577, 125729, 125881, 126032, 126184, 126335, 126487, 126638, 126789,
        126940, 127091, 127241, 127392, 127542, 127693, 127843, 127993, 128143,
        128293, 128442, 128592, 128741, 128891, 129040, 129189, 129338, 129487,
        129635, 129784, 129932, 130081, 130229, 130377, 130525, 130673, 130821,
        130968, 131116, 131263, 131410, 131558, 131705, 131851, 131998, 132145,
        132291, 132438, 132584, 132730, 132876, 133022, 133168, 133314, 133460,
        133605, 133750, 133896, 134041, 134186, 134331, 134476, 134620, 134765,
        134909, 135054, 135198, 135342, 135486, 135630, 135773, 135917, 136061,
        136204, 136347, 136491, 136634, 136777, 136919, 137062, 137205, 137347,
        137490, 137632, 137774, 137916, 138058, 138200, 138342, 138483, 138625,
        138766, 138907, 139049, 139190, 139331, 139471, 139612, 139753, 139893,
        140034, 140174, 140314, 140454, 140594, 140734, 140874, 141013, 141153,
        141292, 141432, 141571, 141710, 141849, 141988, 142127, 142265, 142404,
        142542, 142681, 142819, 142957, 143095, 143233, 143371, 143509, 143646,
        143784, 143921, 144058, 144196, 144333, 144470, 144607, 144743, 144880,
        145017, 145153, 145289, 145426, 145562, 145698, 145834, 145970, 146105,
        146241, 146377, 146512, 146647, 146783, 146918, 147053, 147188, 147323,
        147457, 147592, 147727, 147861, 147995, 148130, 148264, 148398, 148532,
        148666, 148799, 148933, 149067, 149200, 149333, 149467, 149600, 149733,
        149866, 149999, 150131, 150264, 150397, 150529, 150661, 150794, 150926,
        151058, 151190, 151322, 151454, 151585, 151717, 151848, 151980, 152111,
        152242, 152373, 152505, 152635, 152766, 152897, 153028, 153158, 153289,
        153419, 153549, 153680, 153810, 153940, 154070, 154199, 154329, 154459,
        154588, 154718, 154847, 154976, 155105, 155235, 155363, 155492, 155621,
        155750, 155879, 156007, 156135, 156264, 156392, 156520, 156648, 156776,
        156904, 157032, 157160, 157287, 157415, 157542, 157670, 157797, 157924,
        158051, 158178, 158305, 158432, 158559, 158685, 158812, 158938, 159065,
        159191, 159317, 159443, 159569,
    };

    uint32_t sens_id = 0u;
    int32_t  i;
    int32_t *derived_array = &gSDL_pvt_poly_work_around[0][0];

    if (gSDL_vtm_lut_done[sens_id] == (bool)false)
    {
        for ( i = 0; i < SDL_VTM_NUM_OF_ADC_CODES; i++)
        {
            derived_array[i] = gSDL_pvt_poly_golden[i];
        }
        gSDL_vtm_lut_done[sens_id] = (bool)true;
    }

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
    SDL_VTM_adc_code        adc_code_h, adc_code_l;
    uint32_t                value;
    SDL_VTM_Ctrlcfg         ts_ctrl_cfg;

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
        /* Have some delay after Register write */
        ClockP_usleep(SDL_VTM_REG_WRITE_DELAY);

         /* Step 2 */
         ts_ctrl_cfg.valid_map = SDL_VTM_TS_CTRL_MAXT_OUTG_ALERT_VALID;
         ts_ctrl_cfg.maxt_outrg_alert_en = SDL_VTM_TS_CTRL_MAXT_OUTRG_GEN_ALERT;

         retVal = SDL_VTM_tsSetCtrl (p_cfg2,
                         instance,
                         &ts_ctrl_cfg);

         if (retVal == SDL_PASS)
         {

            /* Step 3 */
            SDL_REG32_FINS(&p_cfg2->MISC_CTRL, \
                           VTM_CFG2_MISC_CTRL_ANY_MAXT_OUTRG_ALERT_EN, \
                           SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_ENABLE);
            ClockP_usleep(SDL_VTM_REG_WRITE_DELAY);
         }
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);

}

int32_t SDL_VTM_tsSetMaxTOutRgAlertThrDisable(const SDL_VTM_cfg2Regs *p_cfg2, SDL_VTM_InstTs instance)
{
    int32_t retVal = SDL_EBADARGS;
    SDL_VTM_Ctrlcfg ts_ctrl_cfg;

    if (p_cfg2 != NULL_PTR)
    {
        /*
         * Disable maximum temperature out of range alert and thresholds
         * Step 1: WKUP_VTM_TMPSENS_CTRL_j unset the MAXT_OUTRG_EN  bit
         * Step 2: WKUP_VTM_MISC_CTRL unset the ANYMAXT_OUTRG_ALERT_EN  bit
         */

        /* Step 1 */
        ts_ctrl_cfg.valid_map = 0U;
        ts_ctrl_cfg.maxt_outrg_alert_en = SDL_VTM_TS_CTRL_MAXT_OUTRG_NO_ALERT;
        retVal = SDL_VTM_tsSetCtrl (p_cfg2,
                                    instance,
                                    &ts_ctrl_cfg);

        if (retVal == SDL_PASS)
        {
            /* Step 2 */
            SDL_REG32_FINS(&p_cfg2->MISC_CTRL,                          \
                           VTM_CFG2_MISC_CTRL_ANY_MAXT_OUTRG_ALERT_EN,  \
                           SDL_VTM_TSGLOBAL_ANY_MAXT_OUTRG_ALERT_DISABLE);
            /* Have some delay after Register write */
            ClockP_usleep(SDL_VTM_REG_WRITE_DELAY);
        }
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);

}

/* Nothing past this point */

