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

/*
 *  This file contains eQEP waveform generation.
 *  This generated the eQEP signal by toggeling the gpio signals.
 */

#include <drivers/gpio.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "ti_drivers_config.h"
#include "eqep_pattern_gen.h"

/* ========================================================================== */
/*                          Macros And Typedefs                               */
/* ========================================================================== */
#define EQEP_A_GPIO_PIN             (CONFIG_GPIO0_PIN)
#define EQEP_B_GPIO_PIN             (CONFIG_GPIO1_PIN)
#define EQEP_S_GPIO_PIN             (CONFIG_GPIO2_PIN)
#define EQEP_I_GPIO_PIN             (CONFIG_GPIO3_PIN)

void App_eqepGeneratePattern(EqepAppPatternParams *patParam)
{
    uint32_t clockDelayCnt;
    uint32_t i, j;
    uint32_t gGpioBaseAddr;

    gGpioBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_GPIO0_BASE_ADDR);
    /* Configure all GPIO pins as output for generating EQEP Signals. */
    GPIO_setDirMode(gGpioBaseAddr, EQEP_A_GPIO_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_setDirMode(gGpioBaseAddr, EQEP_B_GPIO_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_setDirMode(gGpioBaseAddr, EQEP_S_GPIO_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_setDirMode(gGpioBaseAddr, EQEP_I_GPIO_PIN, GPIO_DIRECTION_OUTPUT);

    /* delay required for each quadrature of the EQEP signal in usec. */
    clockDelayCnt = (1000000 / patParam->eqepClockFreq) / 4;

    for (i = 0; i< patParam->idxEvtCnt; i++)
    {
        for (j =0; j< patParam->loopCnt; j++)
        {
            if (patParam->direction == EQEP_DIR_CLOCKWISE)
            {
                /* Configure all EQEP A and B lines to generate clockwise
                 * rotation. */
                GPIO_pinWriteLow(gGpioBaseAddr, EQEP_A_GPIO_PIN);
                GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_B_GPIO_PIN);
                ClockP_usleep(clockDelayCnt);

                GPIO_pinWriteLow(gGpioBaseAddr, EQEP_A_GPIO_PIN);
                GPIO_pinWriteLow(gGpioBaseAddr, EQEP_B_GPIO_PIN);
                ClockP_usleep(clockDelayCnt);

                GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_A_GPIO_PIN);
                GPIO_pinWriteLow(gGpioBaseAddr, EQEP_B_GPIO_PIN);
                ClockP_usleep(clockDelayCnt);

                GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_A_GPIO_PIN);
                GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_B_GPIO_PIN);
                ClockP_usleep(clockDelayCnt);
            }
            else
            {
                /* Configure all EQEP A and B lines to generate anti clockwise
                 * rotation. */
                GPIO_pinWriteLow(gGpioBaseAddr, EQEP_A_GPIO_PIN);
                GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_B_GPIO_PIN);
                ClockP_usleep(clockDelayCnt);

                GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_A_GPIO_PIN);
                GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_B_GPIO_PIN);
                ClockP_usleep(clockDelayCnt);

                GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_A_GPIO_PIN);
                GPIO_pinWriteLow(gGpioBaseAddr, EQEP_B_GPIO_PIN);
                ClockP_usleep(clockDelayCnt);

                GPIO_pinWriteLow(gGpioBaseAddr, EQEP_A_GPIO_PIN);
                GPIO_pinWriteLow(gGpioBaseAddr, EQEP_B_GPIO_PIN);
                ClockP_usleep(clockDelayCnt);
            }
        }
        if (patParam->generateIdxPulse == TRUE)
        {
            /* Generate the Index pulse by toggeling EQEP I signal. */
            GPIO_pinWriteHigh(gGpioBaseAddr, EQEP_I_GPIO_PIN);
            ClockP_usleep(clockDelayCnt);
            GPIO_pinWriteLow(gGpioBaseAddr, EQEP_I_GPIO_PIN);
        }
    }
}
