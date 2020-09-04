/*!
* \file IOLM_Port_LEDTask.c
*
* \brief
* Interface for LED Handling on IOLink Board
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-19
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <osal.h>
#include <IOLM_Types.h>
#include "IOLM_Port_LEDTask.h"
#include "IOLM_Port_spi.h"
#define IOLM_LED_CHANNEL    IOLM_SPI_LED_CHANNEL

/* ========================================================================== */
/*                                Defines                                     */
/* ========================================================================== */

#define IOLM_LEDSPERPORT                (2U)
#define IOLM_LED_TIMERCOUNT             (3U)
#define IOLM_TIMER_COUNTER_SLOW         (0U)
#define IOLM_TIMER_COUNTER_FAST         (1U)
#define IOLM_TIMER_COUNTER_DATA         (2)
#define IOLM_LED_RED_BOOT_STATE         IOLM_eLEDState_off
#define IOLM_LED_GREEN_BOOT_STATE       IOLM_eLEDState_off
#define IOLM_LED_BASEBOARD_HEARTBEAT    (0U)

/* times in ms (MUST be multiples of IOL_LED_TASK_TICKRATE) */
#define IOLM_LED_TASK_TICKRATE          (100U)
#define IOLM_LED_SLOWBLINKTIME          (500U)
#define IOLM_LED_FASTBLINKTIME          (200U)
#define IOLM_LED_DATA_EXCHANGE_ON       (1000U)
#define IOLM_LED_DATA_EXCHANGE_OFF      (100U)

/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */

static IOLM_LED_eState_t    IOLM_LED_aStates_s[IOLM_PORT_COUNT][IOLM_LEDSPERPORT];
static uint32_t             IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_LED_TIMERCOUNT];

static const IOLM_LED_sLedMapping_t IOLM_LED_aPortMapping_s[2 * IOLM_PORT_COUNT] =
{
    {7, IOLM_eLEDColor_red},      /* 1st LED on driver IC -> port 8, red LED */
    {7, IOLM_eLEDColor_green},    /* 2nd LED on driver IC -> port 8, green LED */
    {6, IOLM_eLEDColor_red},
    {6, IOLM_eLEDColor_green},
    {5, IOLM_eLEDColor_red},
    {5, IOLM_eLEDColor_green},
    {4, IOLM_eLEDColor_red},
    {4, IOLM_eLEDColor_green},
    {3, IOLM_eLEDColor_red},
    {3, IOLM_eLEDColor_green},
    {2, IOLM_eLEDColor_red},
    {2, IOLM_eLEDColor_green},
    {1, IOLM_eLEDColor_red},
    {1, IOLM_eLEDColor_green},
    {0, IOLM_eLEDColor_red},
    {0, IOLM_eLEDColor_green}
};

/* define which functions have to be used to control the LEDs */
static IOLM_LED_sLedCfg_t IOLM_LED_sLed_s =
{
    IOLM_SPI_init,
    IOLM_SPI_close,
    IOLM_SPI_setIolLeds,
    IOLM_SPI_baseBoardLED
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void IOLM_LED_setLedColorState(uint8_t SMIPort_p, IOLM_LED_eColor_t color_p, IOLM_LED_eState_t state_p)
{
    uint8_t portInternal = SMIPort_p - 1;

    if ((portInternal < IOLM_PORT_COUNT) && (color_p < IOLM_LEDSPERPORT))
    {
        IOLM_LED_aStates_s[portInternal][color_p] = state_p;
    }
}

void OSAL_FUNC_NORETURN IOLM_LED_switchingTask()
{
    uint8_t i;
    uint32_t led_state = 0;

    IOLM_LED_sLed_s.cbInit();

    IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_SLOW] = 0;
    IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_FAST] = 0;
    IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_DATA] = 0;

    for(i = 1; i <= IOLM_PORT_COUNT; i++)
    {
        IOLM_LED_setLedColorState(i, IOLM_eLEDColor_green, IOLM_LED_GREEN_BOOT_STATE);
        IOLM_LED_setLedColorState(i, IOLM_eLEDColor_red,   IOLM_LED_RED_BOOT_STATE);
    }

    while(1)
    {
        IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_SLOW]++;
        IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_FAST]++;
        IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_DATA]++;

        for(i=0; i<sizeof(IOLM_LED_aPortMapping_s)/sizeof(IOLM_LED_aPortMapping_s[0]); i++)
        {
            switch(IOLM_LED_aStates_s[IOLM_LED_aPortMapping_s[i].port][IOLM_LED_aPortMapping_s[i].eLedColor])
            {
            case IOLM_eLEDState_on:
                led_state |= (1<<i);
                break;
            case IOLM_eLEDState_off:
                led_state &= ~(1<<i);
                break;
            case IOLM_eLEDState_slow:
                if(IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_SLOW] <= IOLM_LED_SLOWBLINKTIME/IOLM_LED_TASK_TICKRATE)
                {                   
                    led_state |= (1<<i);
                }
                else
                {
                    led_state &= ~(1<<i);
                }
                break;
            case IOLM_eLEDState_fast:
                if(IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_FAST] <= IOLM_LED_FASTBLINKTIME/IOLM_LED_TASK_TICKRATE)
                {
                    led_state |= (1<<i);
                }
                else
                {
                    led_state &= ~(1<<i);
                }
                break;
            case IOLM_eLEDState_data:
                if(IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_DATA] <= IOLM_LED_DATA_EXCHANGE_ON/IOLM_LED_TASK_TICKRATE)
                {
                    led_state |= (1<<i);
                }
                else
                {
                    led_state &= ~(1<<i);
                }
                break;
            }
        }

        if (IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_SLOW] <= IOLM_LED_SLOWBLINKTIME / IOLM_LED_TASK_TICKRATE)
        {
            IOLM_LED_sLed_s.cbBaseBoardLED(IOLM_LED_BASEBOARD_HEARTBEAT, IOLM_eLEDState_on);
        }
        else
        {
            IOLM_LED_sLed_s.cbBaseBoardLED(IOLM_LED_BASEBOARD_HEARTBEAT, IOLM_eLEDState_off);
        }

        IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_SLOW] %=
            2 * (IOLM_LED_SLOWBLINKTIME/IOLM_LED_TASK_TICKRATE);
        IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_FAST] %=
            2 * (IOLM_LED_FASTBLINKTIME/IOLM_LED_TASK_TICKRATE);
        IOLM_LED_aLedBlinkSyncTimerCount_s[IOLM_TIMER_COUNTER_DATA] %=
            (IOLM_LED_DATA_EXCHANGE_ON/IOLM_LED_TASK_TICKRATE) +
            (IOLM_LED_DATA_EXCHANGE_OFF/IOLM_LED_TASK_TICKRATE);

        IOLM_LED_sLed_s.cbSetLEDs(led_state);

        OSAL_SCHED_sleep(IOLM_LED_TASK_TICKRATE);
    }
}
