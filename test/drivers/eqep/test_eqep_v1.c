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

#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/cmpss.h>
#include <drivers/epwm.h>
#include <drivers/ecap.h>
#include <drivers/eqep.h>
#include <drivers/soc.h>
#include <drivers/adc.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/ClockP.h>

#include "menu.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */



/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Testcases */

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
/* Non-JIRA Test case*/
static void EQEP_apiCheck(void *args);

/*Utility/helper functions*/
int32_t test_eqep_cases(uint8_t in);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Tester dependent */
uint8_t gCmdTxBuffer[CMD_TX_BUFSIZE];
uint8_t gCmdRxBuffer[CMD_RX_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;
int32_t          transferOK;
UART_Transaction trans;

bool enableLog;
bool manual_mode;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


#define TOTAL_TEST_CASES (11)

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    tester_init();
    UNITY_BEGIN();
    char test_title[] = "----------------------EQEP-TEST-CASES----------------------";
    


    menu_input test_list[TOTAL_TEST_CASES] =
    {
        {0, 3200,  EQEP_quadrature_pattern_detection,                       "EQEP_quadrature_pattern_detection"},
        {0, 3201,  EQEP_pulse_width_direction_bit_pattern_detection,        "EQEP_pulse_width_direction_bit_pattern_detection"},
        {0, 3202,  EQEP_qma_mode_1,                                         "EQEP_qma_mode_1"},
        {0, 3203,  EQEP_qma_mode_2,                                         "EQEP_qma_mode_2"},
        {0, 3204,  EQEP_eqep_cmpss_interaction,                             "EQEP_eqep_cmpss_interaction"},
        {0, 3205,  EQEP_eqep_adc_interaction,                               "EQEP_eqep_adc_interaction"},
        {0, 3206,  EQEP_eqep_pwm_xbar,                                      "EQEP_eqep_pwm_xbar"},
        {0, 3207,  EQEP_invalid_pattern_detection,                          "EQEP_invalid_pattern_detection"},
        {0, 3208,  EQEP_qep_watchdog,                                       "EQEP_qep_watchdog"},
        {0, 3209,  EQEP_qep_position_counter_overflow_underflow,            "EQEP_qep_position_counter_overflow_underflow"},
        {1, 1,     EQEP_apiCheck,                                           "EQEP_apiCheck"},
    };

    menu(TOTAL_TEST_CASES, test_list, test_title);

    if(enableLog)
    {
        DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
        DebugP_logZoneEnable(DebugP_LOG_ZONE_INFO);
    }
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

static void EQEP_quadrature_pattern_detection(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(1), 0);
}
static void EQEP_pulse_width_direction_bit_pattern_detection(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(2), 0);
}
static void EQEP_qma_mode_1(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(3), 0);
}
static void EQEP_qma_mode_2(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(4), 0);
}
static void EQEP_eqep_cmpss_interaction(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(5), 0);
}
static void EQEP_eqep_adc_interaction(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(6), 0);
}
static void EQEP_eqep_pwm_xbar(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(7), 0);
}
static void EQEP_invalid_pattern_detection(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(8), 0);
}
static void EQEP_qep_watchdog(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(9), 0);
}
static void EQEP_qep_position_counter_overflow_underflow(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(10), 0);
}
static void EQEP_apiCheck(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_eqep_cases(11), 0);
}

uint32_t util_EQEP_getInstanceFromBase(uint32_t base)
{
    switch(base)
    {
    case CSL_CONTROLSS_EQEP0_U_BASE:
            return 0;
    case CSL_CONTROLSS_EQEP1_U_BASE:
            return 1;
    case CSL_CONTROLSS_EQEP2_U_BASE:
            return 2;
    default:
            return 0xFFFFFFFF;
    }

}

int32_t eqep_inst_tc_EQEP_quadrature_pattern_detection(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_pulse_width_direction_bit_pattern_detection(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_qma_mode_1(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_qma_mode_2(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_eqep_cmpss_interaction(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_eqep_adc_interaction(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_eqep_pwm_xbar(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_invalid_pattern_detection(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_qep_watchdog(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_qep_position_counter_overflow_underflow(base)
{
    int32_t error=0;

    return error;
}

int32_t eqep_inst_tc_EQEP_apiCheck(uint32_t base)
{
    int32_t error=0;

    uint32_t val = 0xFF;

    /* Call the EQEP_setPosition API */
    EQEP_setPosition(CSL_CONTROLSS_EQEP0_U_BASE, val);

    /* Check if the value was written correctly */
    if(EQEP_getPosition(CSL_CONTROLSS_EQEP0_U_BASE) != val)
    {
        error++;
    }

    return error;
}

uint8_t isTestSupported[3][11] =
{
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  //Supported tests for EQEP0
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  //Supported tests for EQEP1
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}   //Supported tests for EQEP2

};

int32_t test_eqep_cases(uint8_t in)
{
    int32_t failcount=0;

    uint32_t eqep_offset=CSL_CONTROLSS_EQEP1_U_BASE - CSL_CONTROLSS_EQEP0_U_BASE;
    uint32_t base;
    for(base = CSL_CONTROLSS_EQEP0_U_BASE; base<=CSL_CONTROLSS_EQEP2_U_BASE; base=base+eqep_offset)
    {

        if(enableLog)
        {
            DebugP_log("\r\n\nbase=0x%08x, instance=%d", base, util_EQEP_getInstanceFromBase(base));
        }        

        if(isTestSupported[util_EQEP_getInstanceFromBase(base)][in-1])
        {
            switch(in)
            {
                case 1:
                    failcount += eqep_inst_tc_EQEP_quadrature_pattern_detection(base);
                    break;
                case 2:
                    failcount += eqep_inst_tc_EQEP_pulse_width_direction_bit_pattern_detection(base);
                    break;
                case 3:
                    failcount += eqep_inst_tc_EQEP_qma_mode_1(base);
                    break;
                case 4:
                    failcount += eqep_inst_tc_EQEP_qma_mode_2(base);
                    break;
                case 5:
                    failcount += eqep_inst_tc_EQEP_eqep_cmpss_interaction(base);
                    break;
                case 6:
                    failcount += eqep_inst_tc_EQEP_eqep_adc_interaction(base);
                    break;
                case 7:
                    failcount += eqep_inst_tc_EQEP_eqep_pwm_xbar(base);
                    break;
                case 8:
                    failcount += eqep_inst_tc_EQEP_invalid_pattern_detection(base);
                    break;
                case 9:
                    failcount += eqep_inst_tc_EQEP_qep_watchdog(base);
                    break;
                case 10:
                    failcount += eqep_inst_tc_EQEP_qep_position_counter_overflow_underflow(base);
                    break;
                case 11:
                    failcount += eqep_inst_tc_EQEP_apiCheck(base);
                    break;
            }
            if(enableLog)
            {
                DebugP_log("\r\nfailcount=%d", failcount);
            }

        }
        else
        {
            if(enableLog)
            {
                DebugP_log("\r\EQEP %u, Test %u not supported", util_EQEP_getInstanceFromBase(base), in-1);
            }

        }

    }


    if(failcount!=0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nFAIL %d", failcount);
        }

        return 1;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }

}
