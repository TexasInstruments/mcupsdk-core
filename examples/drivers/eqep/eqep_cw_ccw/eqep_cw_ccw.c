/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/gpio.h>
#include <drivers/eqep.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 *  CW/CCW Input demonstration to eQEP
 * 
 *  This example emulates CW and CCW pulses using 2 GPIO's and Timer based interrupts, before feeding them to the eQEP module.
 *  QMA MODE 2 (Active Pulse High is Enabled)
 *  We calculate and watch position, direction, frequency and speed post feeding these
 *  signals to eQEP module.
 *  
 * 
 *  The configuration for this example is as follows
 *  - 50 CW pulses followed by 50 CCW pulses
 *  - Encoder resolution is configured to 1000 counts/revolution
 *  - GPIO's are routed through PWMXBAR to eQEP module
 *  - GPIO's (simulating CW/CCW encoder signals) are configured for a 2.5kHz frequency
 *    or 150 rpm (= 2500 cnts/sec * 60 sec/min) / 1000 cnts/rev)
 * 
 * 
 *  External Connections (Only needed if you want to view the input CW/CCW Signals)\n
 * 
 *  AM263x-CC, AM263Px-CC
 *  - Scope GPIO43/Hsec 49 (simulates eQEP Phase A signal)
 *  - Scope GPIO44/Hsec 51 (simulates eQEP Phase B signal)
 *  - Scope GPIO48/Hsec 52 (simulates eQEP Index Signal)
 * 
 *  AM263x-LP, AM263Px-LP
 *  - Scope GPIO43/J2.11 (simulates eQEP Phase A signal)
 *  - Scope GPIO44/J6.59 (simulates eQEP Phase B signal)
 *  - Scope GPIO48/J4.40 (simulates eQEP Index Signal)
 *
 *  AM261x-SOM
 *  - Scope GPIO49 (simulates eQEP Phase A signal)
 *  - Scope GPIO50 (simulates eQEP Phase B signal)
 *  - Scope GPIO48 (simulates eQEP Index Signal)
 * 
 *  AM261x-LP
 *  - Scope GPIO49/J4.38 (simulates eQEP Phase A signal)
 *  - Scope GPIO50/J4.37 (simulates eQEP Phase B signal)
 *  - Scope GPIO48/J4.36 (simulates eQEP Index Signal)
 * 
 */ 

#define APP_LOOP_CNT    1000U

/* Number of encoder slots per revolution */
#define ENCODER_SLOTS   1000U

/* Unit timeout period in microseconds (10ms) */
#define UNIT_PERIOD     10000U

/* Previous position count at last unit timeout */
uint32_t oldCount = 0;

/* Latest latched position count at unit timeout */
uint32_t newCount = 0;

/* Current absolute encoder position (increments with CW, decrements with CCW) */
uint32_t currentEncoderPos = 0;

/* Pulse frequency in Hz (pulses per second) */
int32_t freq = 0;

/* Motor speed in RPM */
float speed = 0.0F;

/* Current direction: 1 = CW, -1 = CCW */
int32_t direction = 0;

/* EQEP peripheral base address */
uint32_t gEqepBaseAddr;

/* Interrupt counter */
uint16_t gInterruptCount = 0;


static inline void GPIO_pinToggle(uint32_t baseAddr, uint32_t pinNum)
{
    uint32_t                regIndex, regVal;
    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) baseAddr);

    regIndex = GPIO_GET_REG_INDEX(pinNum);
    regVal = GPIO_GET_BIT_MASK(pinNum);
    CSL_REG32_WR(&hGpio->BANK_REGISTERS[regIndex].OUT_DATA, 
                 CSL_REG32_RD(&hGpio->BANK_REGISTERS[regIndex].OUT_DATA) ^ regVal);

    return;
}

void eqep_cw_ccw_main(void *args)
{
    uint32_t    i;
    int32_t     status;

    Drivers_open();
    Board_driversOpen();

    gEqepBaseAddr = CONFIG_EQEP0_BASE_ADDR;
	
    DebugP_log("EQEP Position Speed Test Started ...\r\n");
	
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

	for(i = 0; i < APP_LOOP_CNT; ++i)
    {
        currentEncoderPos = EQEP_getPosition(gEqepBaseAddr);
        direction         = EQEP_getDirection(gEqepBaseAddr);
        
        status = EQEP_getInterruptStatus(gEqepBaseAddr);

        if(status & EQEP_INT_QMA_ERROR) {
            EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_QMA_ERROR);
            DebugP_log("QMA Error detected!\r\n");
        }

        if((status & EQEP_INT_UNIT_TIME_OUT) != 0)
        {
            newCount          = EQEP_getPositionLatch(gEqepBaseAddr);
            if (direction > 0 )
            {
                if (newCount >= oldCount)
                    newCount = newCount - oldCount;
                else
                    newCount = (ENCODER_SLOTS - oldCount) + newCount;
            }
            else 
            {
                if (newCount <= oldCount)
                    newCount = oldCount - newCount;
                else
                    newCount = (ENCODER_SLOTS - newCount) + oldCount;
            }

            oldCount = currentEncoderPos;

            freq = (newCount * (uint32_t)1000000U) / ((uint32_t)UNIT_PERIOD);
            speed = (freq * 60) / ((float)(ENCODER_SLOTS));
            
            EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_UNIT_TIME_OUT);
        }

    }

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void timerISR(void) {
    static uint32_t toggleCount = 0;
    static uint32_t indexPulseCount = 0;

    toggleCount++;

    if(toggleCount <= 100) {  // 50 pulses
        GPIO_pinToggle(GPIO_CW_PIN_BASE_ADDR, GPIO_CW_PIN_PIN);
    }
    else if(toggleCount >= 101 && toggleCount <= 200) {  //50 pulses
        GPIO_pinToggle(GPIO_CCW_PIN_BASE_ADDR, GPIO_CCW_PIN_PIN);
    }
    else if(toggleCount >= 201) { 
        toggleCount = 0;
    }

    if(toggleCount % 1000 == 0) {
        GPIO_pinWriteHigh(GPIO_INDEX_PIN_BASE_ADDR, GPIO_INDEX_PIN_PIN);
        indexPulseCount = 1;
    }
    else if(indexPulseCount > 0) {
        indexPulseCount++;
        if(indexPulseCount >= 4)
        {
            GPIO_pinWriteLow(GPIO_INDEX_PIN_BASE_ADDR, GPIO_INDEX_PIN_PIN);
            indexPulseCount = 0;
        }
    }
}
