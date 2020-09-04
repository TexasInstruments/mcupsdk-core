/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <unity.h>
#include <drivers/eqep.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void EQEP_getSetPositionApiCheck(void *args);

static void EQEP_quadrature_pattern_detection(void *args);
static void EQEP_pulse_width_direction_bit_pattern_detection(void *args);
static void EQEP_qma_mode_1(void *args);
static void EQEP_qma_mode_2(void *args);
static void EQEP_eqep_cmpss_interaction(void *args);
static void EQEP_eqep_adc_interaction(void *args);
static void EQEP_eqep_pwm_xbar(void *args);
static void EQEP_invalid_pattern_detection(void *args);
static void EQEP_qep_watchdog(void *args);
static void EQEP_qep_position_counter_overflow_underflow(void *args);
    
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    RUN_TEST(EQEP_getSetPositionApiCheck, 1, NULL);

    RUN_TEST(EQEP_quadrature_pattern_detection , 3200, NULL);
    RUN_TEST(EQEP_pulse_width_direction_bit_pattern_detection , 3201, NULL);
    RUN_TEST(EQEP_qma_mode_1 , 3202, NULL);
    RUN_TEST(EQEP_qma_mode_2 , 3203, NULL);
    RUN_TEST(EQEP_eqep_cmpss_interaction , 3204, NULL);
    RUN_TEST(EQEP_eqep_adc_interaction , 3205, NULL);
    RUN_TEST(EQEP_eqep_pwm_xbar , 3206, NULL);
    RUN_TEST(EQEP_invalid_pattern_detection , 3207, NULL);
    RUN_TEST(EQEP_qep_watchdog , 3208, NULL);
    RUN_TEST(EQEP_qep_position_counter_overflow_underflow , 3209, NULL);

    
    UNITY_END();

    /* Close drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

/* Testcase 1 - Check the EQEP_setPosition and EQEP_getPosition API */
static void EQEP_getSetPositionApiCheck(void *args)
{
    uint32_t val = 0xFF;

    /* Call the EQEP_setPosition API */
    EQEP_setPosition(CSL_CONTROLSS_EQEP0_U_BASE, val);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32(EQEP_getPosition(CSL_CONTROLSS_EQEP0_U_BASE), val);
}

static void EQEP_quadrature_pattern_detection(void *args)
{

}
static void EQEP_pulse_width_direction_bit_pattern_detection(void *args)
{

}
static void EQEP_qma_mode_1(void *args)
{

}
static void EQEP_qma_mode_2(void *args)
{

}
static void EQEP_eqep_cmpss_interaction(void *args)
{

}
static void EQEP_eqep_adc_interaction(void *args)
{

}
static void EQEP_eqep_pwm_xbar(void *args)
{

}
static void EQEP_invalid_pattern_detection(void *args)
{

}
static void EQEP_qep_watchdog(void *args)
{

}
static void EQEP_qep_position_counter_overflow_underflow(void *args)
{

}
