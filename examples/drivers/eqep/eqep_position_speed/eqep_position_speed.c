/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/eqep.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 *  Position and Speed Measurement Using eQEP
 * 
 *  This example provides position and speed measurement using the
 *  capture unit and speed measurement using unit time out of the eQEP module.
 *  ePWM0 and a GPIO are configured to generate simulated eQEP signals. The
 *  ePWM module will interrupt once every period and call the position/speed
 *  calculation function. This example uses the IQMath library to simplify
 *  high-precision calculations.
 * 
 *  The configuration for this example is as follows
 *  - Maximum speed is configured to 6000rpm (baseRPM)
 *  - Minimum speed is assumed at 10rpm for capture pre-scalar selection
 *  - Pole pair is configured to 2 (polePairs)
 *  - Encoder resolution is configured to 4000 counts/revolution (mechScaler)
 *  - Which means: 4000 / 4 = 1000 line/revolution quadrature encoder
 *    (simulated by ePWM0)
 *  - ePWM0 (simulating QEP encoder signals) is configured for a 5kHz frequency
 *    or 300 rpm (= 4 * 5000 cnts/sec * 60 sec/min) / 4000 cnts/rev)
 * 
 *  SPEEDRPM_FR: High Speed Measurement is obtained by counting the QEP
 *  input pulses for 10ms (unit timer set to 100Hz).
 * 
 *  SPEEDRPM_FR = (Position Delta / 10ms) * 60 rpm
 * 
 *  SPEEDRPM_PR: Low Speed Measurement is obtained by measuring time period
 *  of QEP edges. Time measurement is averaged over 64 edges for better results
 *  and the capture unit performs the time measurement using pre-scaled SYSCLK.
 * 
 *  Note that the pre-scaler for capture unit clock is selected such that the
 *  capture timer does not overflow at the required minimum frequency. 
 * 
 *  This example wait for 10 iterations of unit time out event and verifies the 
 *  measured speed:  295 < posSpeed.speedRPMFR < 305
 * 
 *  - posSpeed.speedRPMFR - Speed meas. in rpm using QEP position counter
 *  - posSpeed.speedRPMPR - Speed meas. in rpm using capture unit
 *  - posSpeed.thetaMech  - Motor mechanical angle (Q15)
 *  - posSpeed.thetaElec  - Motor electrical angle (Q15)
 * 
 *  External Connections \n
 *  - Connect eQEP0A to ePWM0A (simulates eQEP Phase A signal)
 *  - Connect eQEP0B to ePWM0B (simulates eQEP Phase B signal)
 *  - Connect eQEP0I to GPIO48 (simulates eQEP Index Signal)
 * 
 */ 

// Imported from iq library
// Assumption - GLOBAL_Q = 24
#define  GLOBAL_Q  (24)
#define  _IQ15toIQ(A)  ((long) (A) << (GLOBAL_Q - 15))
#define  _IQ24(A)  (long) ((A) * 16777216.0L)
#define  _IQ(A)  _IQ24(A)
#define  _IQmpy(A,B)  ((A) * (B))
#define  _IQdiv(A,B)  ((float)(A) / (float)(B))
typedef   uint64_t    _iq;

/* Defines */
/* Sysclk frequency */
#define DEVICE_SYSCLK_FREQ  (200000000U)
/* Macro for interrupt pulse */
#define APP_INT_IS_PULSE  (1U)
// .9999 / 4000 converted to IQ26 fixed point format
#define MECH_SCALER  (16776)
// 2 pole pairs in this example
#define POLE_PAIRS  (2)
// Angular offset between encoder and Phase A
#define CAL_ANGLE  (0)
// See Equation 5 in eqep_ex2_calculation.c
#define SPEED_SCALER  ((((uint64_t)32 * DEVICE_SYSCLK_FREQ / 64) * 60) / (24000000))
// Base/max rpm is 6000rpm
#define BASE_RPM  (6000)


/* Typedefs */
typedef struct
{
    /* Output: Motor electrical angle (Q15) */
    int16_t thetaElec;
    /* Output: Motor mechanical angle (Q15) */
    int16_t thetaMech;
    /* Output: Motor rotation direction (Q0) */
    int16_t directionQEP;
    /* Variable: Raw angle from timer 2 (Q0) */
    int16_t thetaRaw;
    /* Parameter: 0.9999 / total count, total count = 4000 (Q26) */
    int16_t mechScaler;
    /* Parameter: Number of pole pairs (Q0) */
    int16_t polePairs;
    /* Parameter: Raw angular offset between encoder and Phase A (Q0) */
    int16_t calAngle;
    /* Parameter: Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0)
    - independently with global Q */
    uint32_t speedScaler;
    /* Output: Speed in per-unit */
    _iq speedPR;
    /* Parameter: Scaler converting GLOBAL_Q speed to rpm (Q0) speed
    - independently with global Q */
    uint32_t baseRPM;
    /* Output: Speed in rpm (Q0) - independently with global Q */
    int16_t speedRPMPR;
    /* Output: Speed in per-unit */
    _iq oldPos;
    _iq speedFR;
    /* Output: Speed in rpm (Q0) - independently with global Q */
    int16_t speedRPMFR;
} PosSpeed_Object;

typedef PosSpeed_Object *PosSpeed_Handle;

/* Global variables and objects */
uint32_t gEqepBaseAddr;
uint32_t gEpwmBaseAddr;
static HwiP_Object gEpwmHwiObject;
PosSpeed_Object posSpeed =
{
    0, 0, 0, 0,     /* Initialize outputs to zero */
    MECH_SCALER,    /* mechScaler */
    POLE_PAIRS,     /* polePairs */
    CAL_ANGLE,      /* calAngle */
    SPEED_SCALER,   /* speedScaler */
    0,              /* Initialize output to zero */
    BASE_RPM,       /* baseRPM */
    0, 0, 0, 0      /* Initialize outputs to zero */
};
uint16_t gInterruptCount = 0;
/* counter to check measurement gets saturated */
uint32_t gCount = 0;
/* Pass or fail indicator */
uint32_t gPass = 0, gFail = 0;

/* Function Prototypes */
static void PosSpeed_calculate(PosSpeed_Handle, uint32_t*);
static void App_epwmIntrISR(void *handle);

void eqep_position_speed_main(void *args)
{
    int32_t status;
    HwiP_Params hwiPrms;

    /* Open drivers
		- Open the UART driver for console
		- Open EPWM and EQEP driver for position and speed measurement
		- Open XBAR driver for EPWM interrupt routing to R5F
	*/
    Drivers_open();
    Board_driversOpen();

    GPIO_setDirMode(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN, CONFIG_GPIO0_DIR);
	
    DebugP_log("EQEP Position Speed Test Started ...\r\n");
	DebugP_log("Please ensure EPWM to EQEP loopback is connected...\r\n");
	DebugP_log("Please wait few seconds ...\r\n");
	
    gEqepBaseAddr = CONFIG_EQEP0_BASE_ADDR;
    gEpwmBaseAddr = CONFIG_EPWM0_BASE_ADDR;

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_epwmIntrISR;
    /* Integrate with Syscfg */
    hwiPrms.isPulse     = APP_INT_IS_PULSE;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Clear ePWM interrupt */
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);

    while( *(volatile uint32_t *) ((uintptr_t) &gCount) < 10)
    {
        //Wait for posSpeed results
    }
	
	EPWM_setTimeBaseCounterMode(gEpwmBaseAddr, EPWM_COUNTER_MODE_STOP_FREEZE);
	
    if((gPass == 1) && (gFail == 0))
    {
		/* Expected 300 RPM based on programmed EPWM frequency*/
		DebugP_log("Expected speed = %d RPM, Measured speed = %d RPM \r\n", 300, posSpeed.speedRPMFR);	
		
		DebugP_log("Electrical angle (Q15) = %d \r\n", posSpeed.thetaElec);
		DebugP_log("Mechanical angle (Q15) = %d \r\n", posSpeed.thetaMech);
		
		DebugP_log("Rotation direction = ");
		if(posSpeed.directionQEP==1)
		{
			DebugP_log("CW, forward \r\n");
		}
		if(posSpeed.directionQEP==-1)
		{
			DebugP_log("CCW, reverse \r\n");
		}
	
        DebugP_log("EQEP Position Speed Test Passed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Fail\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

/* ePWM0 ISR--interrupts once every 4 QCLK counts (one period) */
static void App_epwmIntrISR(void *handle)
{
    uint16_t i;

    /* Position speed and measurement */
    PosSpeed_calculate(&posSpeed, &gCount);

    /* Comparing the eQEP measured frequency with the ePWM frequency
    / After count becomes 3 , eQEP measurement gets saturated and if
    / measure speed is 5 more or less than input speed then pass = 1 */
    if (gCount >= 2)
    {
        if (((posSpeed.speedRPMFR - 300) < 5) && ((posSpeed.speedRPMFR - 300) > -5))
        {
            gPass = 1; gFail = 0;
        }
        else
        {
            gFail = 1; gPass = 0;
        }
    }

    /* Control loop for position control and speed control */
    gInterruptCount++;
    if(gInterruptCount == 1000)
    {
        /* Pulse index signal (1 pulse/rev) */
        GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
        for(i = 0; i < 700; i++)
        {
            ;
        }
        GPIO_pinWriteLow(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
    }

    /* Clear interrupt flag */
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);
}

/* Function to calculate the frequency of the input signal using both the unit
   timer and the quadrature capture units. 

 * The position and speed calculation steps performed
 * by PosSpeed_calculate() are described below:
 *
 * 1. This program calculates: **thetaMech**
 *
 *      thetaMech = QPOSCNT / mechScaler
 *                = QPOSCNT / 4000, where 4000 is the number of counts in 1 rev
 *                                (4000 / 4 = 1000 line/rev quadrature encoder)
 *
 * Note this example uses mechScaler = 16776 which is _IQ26(1 / 4000)
 *
 * 2. This program calculates: **thetaElec**
 *
 *      thetaElec = (pole pairs) * thetaMech = 2 * QPOSCNT / 4000 in this case
 *
 * 3. This program calculates: **speedRPMFR**
 *
 *      speedRPMFR = [(x2 - x1) / 4000] / T                         -Equation 1
 *
 * Note (x2 - x1) = difference in number of QPOSCNT counts. Dividing (x2 - x1)
 * by 4000 gives position relative to Index in one revolution.
 *
 * If base RPM = 6000 rpm:
 *      6000 rpm = [(x2 - x1) / 4000] / 10ms                        -Equation 2
 *               = [(x2 - x1) / 4000] / (.01s * 1 min / 60 sec)
 *               = [(x2 - x1) / 4000] / (1 / 6000) min
 *
 *      max (x2 - x1) = 4000 counts or 1 revolution in 10 ms
 *
 * If both sides of Equation 2 are divided by 6000 rpm, then:
 *      1 = [(x2 - x1) / 4000] rev / [(1 / 6000) min * 6000rpm]
 *
 * Because (x2 - x1) must be < 4000 (max) for QPOSCNT increment,
 * (x2 - x1) / 4000 < 1 for CW rotation, and because (x2 - x1) must be >- 4000
 * for QPOSCNT decrement, (x2 - x1) / 4000 > -1  for CCW rotation
 *
 *      speedFR = [(x2 - x1) / 4000] / [(1 / 6000) min * 6000rpm]
 *              = (x2 - x1) / 4000                                  -Equation 3
 *
 * To convert speedFR to RPM, multiply Equation 3 by 6000 rpm:
 *      speedRPMFR = 6000rpm * (x2 - x1) / 4000                 -Final Equation
 *
 *
 * 2. **min rpm ** = selected at 10 rpm based on CCPS prescaler options
 * available (128 is greatest)
 *
 * 3. **speedRPMPR**
 *      speedRPMPR = X / (t2 - t1)                                  -Equation 4
 *
 * where X = QCAPCTL [UPPS] / 4000 rev (position relative to Index in 1 rev)
 *
 * If max / base speed = 6000 rpm:
 *      6000 = (32 / 4000) / [(t2 - t1) / (SYSCLKFREQ / 64)]
 *
 * where 32 = QCAPCTL[UPPS] (Unit timeout once every 32 edges)
 *
 *      32 / 4000 = position in 1 rev (position as a fraction of 1 revolution)
 *
 *      t2 - t1 / (SYSCLKFREQ / 64), t2 - t1 = # of QCAPCLK cycles
 *
 *      QCAPCLK cycle = 1 / (SYSCLKFREQ / 64)
 *                    = QCPRDLAT
 *
 * So:
 *      6000 rpm = [32(SYSCLKFREQ / 64) * 60s/min] / [4000(t2 - t1)]
 *
 *      t2 - t1 = [32(SYSCLKFREQ / 64) * 60s/min]/(4000 * 6000rpm)  -Equation 5
 *              = 250 CAPCLK cycles
 *              = maximum (t2 - t1) = speedScaler
 *
 * Divide both sides by (t2 - t1) and:
 *      1 = 32 / (t2 - t1)
 *        = [32(SYSCLKFREQ / 64) * 60 s/min] / (4000 * 6000rpm) / (t2 - t1)
 *
 * Because (t2 - t1) must be < 250 for QPOSCNT, increment 250 / (t2 - t1) < 1
 * for CW rotation, and because (t2 - t1) must be > -250 for QPOSCNT decrement
 * 250 / (t2 - t1) > -1 for CCW rotation
 *
 *    speedPR = 250 / (t2 - t1)
 *            = [32(SYSCLKFREQ / 64) * 60 s/min] / (4000 * 6000rpm) / (t2 - t1)
 *                                                                  -Equation 6
 *
 * To convert speedPR to RPM, multiply Equation 6 by 6000rpm:
 *      speedRPMFR = [32(SYSCLKFREQ / 64) * 60s/min] / [4000 * (t2 - t1)]
 *                 = [(32 / 4000) rev * 60s/min] / [(t2 - t1)(QCPRDLAT)]
 *                                                              -Final Equation
 *
 */
void PosSpeed_calculate(PosSpeed_Object *p, uint32_t *c)
{
    int32_t temp;
    uint16_t pos16bVal, temp1;
    long temp2, newPosCnt, oldPosCnt;

    /* Position calculation - mechanical and electrical motor angle
       Get the motor direction: -1 = CCW/reverse, 1 = CW/forward */
    p->directionQEP = EQEP_getDirection(gEqepBaseAddr);

    /* Capture position once per QA/QB period */
    pos16bVal = (uint16_t)EQEP_getPosition(gEqepBaseAddr);

    /* Raw theta = current pos. + ang. offset from QA */
    p->thetaRaw = pos16bVal + p->calAngle;

    /* p->thetaMech ~= QPOSCNT / mechScaler [current cnt/(total cnt in 1 rev)]
       where mechScaler = 4000 cnts/revolution */
    temp = (int32_t)p->thetaRaw * (int32_t)p->mechScaler;  // Q0 * Q26 = Q26
    temp &= 0x03FFF000;

    p->thetaMech = (int16_t)(temp >> 11);                  // Q26 -> Q15
    p->thetaMech &= 0x7FFF;

    /* The following lines calculate p->elec_mech */
    p->thetaElec = p->polePairs * p->thetaMech;            // Q0 * Q15 = Q15
    p->thetaElec &= 0x7FFF;

    /* Check for an index occurrence */
    if((EQEP_getInterruptStatus(gEqepBaseAddr) & EQEP_INT_INDEX_EVNT_LATCH) != 0U)
    {
        EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_INDEX_EVNT_LATCH);
    }

    /* High Speed Calculation using QEP Position counter
       Check for unit position event */
    if((EQEP_getInterruptStatus(gEqepBaseAddr) & EQEP_INT_UNIT_TIME_OUT) != 0)
    {
        /* The following lines calculate position:
           (x2 - x1) / 4000 (position in 1 revolution) */

        (*c)++; /* Incrementing the count value */
        pos16bVal = (uint16_t)EQEP_getPositionLatch(gEqepBaseAddr);
        temp = (int32_t)pos16bVal * (int32_t)p->mechScaler; // Q0 * Q26 = Q26
        temp &= 0x03FFF000;

        temp = (int16_t)(temp >> 11);                       // Q26 -> Q15
        temp &= 0x7FFF;

        newPosCnt = _IQ15toIQ(temp);
        oldPosCnt = p->oldPos;

        /* POSCNT is counting down */
        if(p->directionQEP == -1)
        {
            if(newPosCnt > oldPosCnt)
            {
                /* x2 - x1 should be negative */
                temp2 = -(_IQ(1) - newPosCnt + oldPosCnt);
            }
            else
            {
                temp2 = newPosCnt -oldPosCnt;
            }
        }
        /* POSCNT is counting up */
        else
        {
            if(newPosCnt < oldPosCnt)
            {
                temp2 = _IQ(1) + newPosCnt - oldPosCnt;
            }
            else
            {
                /* x2 - x1 should be positive */
                temp2 = newPosCnt - oldPosCnt;
            }
        }

        if(temp2 > _IQ(1))
        {
            p->speedFR = _IQ(1);
        }
        else if(temp2 < _IQ(-1))
        {
            p->speedFR = _IQ(-1);
        }
        else
        {
            p->speedFR = temp2;
        }

        /* Update the electrical angle */
        p->oldPos = newPosCnt;

        /* Change motor speed from pu value to rpm value (Q15 -> Q0)
           Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q */
        //p->speedRPMFR = _IQmpy(p->baseRPM, p->speedFR);

        _iq temp3 = _IQmpy(p->baseRPM, p->speedFR);
        p->speedRPMFR = (int16_t)(temp3 >> GLOBAL_Q);

        /* Clear unit time out flag */
        EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_UNIT_TIME_OUT);
    }

    /* Low-speed computation using QEP capture counter
       Check for unit position event */
    if((EQEP_getStatus(gEqepBaseAddr) & EQEP_STS_UNIT_POS_EVNT) != 0)
    {
        /* No Capture overflow */
        if((EQEP_getStatus(gEqepBaseAddr) & EQEP_STS_CAP_OVRFLW_ERROR) == 0)
        {
            temp1 = (uint32_t)EQEP_getCapturePeriodLatch(gEqepBaseAddr);
        }
        else
        {
            /* Capture overflow, saturate the result */
            temp1 = 0xFFFF;
        }

        /* p->speedPR = p->speedScaler / temp1 */
        p->speedPR = _IQdiv(p->speedScaler, temp1);
        temp2 = p->speedPR;

        if(temp2 > _IQ(1))
        {
           p->speedPR = _IQ(1);
        }
        else
        {
           p->speedPR = temp2;
        }

        /* Convert p->speedPR to RPM
           Reverse direction = negative */
        if(p->directionQEP == -1)
        {
            /* Q0 = Q0 * GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q */
            p->speedRPMPR = -_IQmpy(p->baseRPM, p->speedPR);
        }
        /* Forward direction = positive */
        else
        {
            /* Q0 = Q0 * GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q */
            p->speedRPMPR = _IQmpy(p->baseRPM, p->speedPR);
        }

        /* Clear unit position event flag and overflow error flag */
        EQEP_clearStatus(gEqepBaseAddr, (EQEP_STS_UNIT_POS_EVNT |
                                      EQEP_STS_CAP_OVRFLW_ERROR));
    }

}
