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
#include <drivers/ecap.h>
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
static void ECAP_setEventPrescalerApiCheck(void *args);


static void ECAP_selection_of_sync_in_source(void *args);
static void ECAP_high_resolution_functions_for_ecap(void *args);
static void ECAP_capture_event_polarity(void *args);
static void ECAP_setting_the_mode_of_capture(void *args);
static void ECAP_support_absolute_and_difference_mode_timestamp_capture(void *args);
static void ECAP_add_additional_input_trigger_sources(void *args);
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args);
static void ECAP_selection_of_soc_trigger_source(void *args);

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

    RUN_TEST(ECAP_setEventPrescalerApiCheck, 3330, NULL);


    RUN_TEST(ECAP_selection_of_sync_in_source , 3401, NULL);
    RUN_TEST(ECAP_high_resolution_functions_for_ecap , 3402, NULL);
    RUN_TEST(ECAP_capture_event_polarity , 3403, NULL);
    RUN_TEST(ECAP_setting_the_mode_of_capture , 3404, NULL);
    RUN_TEST(ECAP_support_absolute_and_difference_mode_timestamp_capture , 3405, NULL);
    RUN_TEST(ECAP_add_additional_input_trigger_sources , 3406, NULL);
    RUN_TEST(ECAP_support_interrupt_generation_on_either_of_4_events , 3407, NULL);
    RUN_TEST(ECAP_selection_of_soc_trigger_source, 7906, NULL);

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

/* Testcase 1 - Check the ECAP_setEventPrescaler API */
static void ECAP_setEventPrescalerApiCheck(void *args)
{
    uint16_t val = 31;

    /* Call the ECAP_setEventPrescaler API */
    ECAP_setEventPrescaler(CSL_CONTROLSS_ECAP0_U_BASE, val);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32((HW_RD_REG16(CSL_CONTROLSS_ECAP0_U_BASE + CSL_ECAP_ECCTL1) &
        CSL_ECAP_ECCTL1_PRESCALE_MASK) >> CSL_ECAP_ECCTL1_PRESCALE_SHIFT, val);
}


static void ECAP_selection_of_sync_in_source(void *args)
{

}
static void ECAP_high_resolution_functions_for_ecap(void *args)
{

}
static void ECAP_capture_event_polarity(void *args)
{

}
static void ECAP_setting_the_mode_of_capture(void *args)
{

}
static void ECAP_support_absolute_and_difference_mode_timestamp_capture(void *args)
{

}
static void ECAP_add_additional_input_trigger_sources(void *args)
{

}
static void ECAP_support_interrupt_generation_on_either_of_4_events(void *args)
{

}
static void ECAP_selection_of_soc_trigger_source(void *args)
{
    uint16_t i, j , actual_value;
    for(i= 0; i<10; i++)
    {
        uint32_t ecap_base = CSL_CONTROLSS_ECAP0_U_BASE + i * 0x1000;

        for(j=ECAP_CAP_MODE_SOC_TRIGGER_SRC_CEVT1; j<=ECAP_APWM_MODE_SOC_TRIGGER_SRC_DISABLED; j++)
        {
            if(j>=ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD)
                actual_value = j - (uint16_t)ECAP_APWM_MODE_SOC_TRIGGER_SRC_PRD;
            else
                actual_value = j;
            ECAP_selectSocTriggerSource(ecap_base, j);
            TEST_ASSERT_EQUAL_INT32((HW_RD_REG32(ecap_base + CSL_ECAP_ECCTL0)
            & CSL_ECAP_ECCTL0_SOCEVTSEL_MASK) >> CSL_ECAP_ECCTL0_SOCEVTSEL_SHIFT, actual_value);
        }
    }
}