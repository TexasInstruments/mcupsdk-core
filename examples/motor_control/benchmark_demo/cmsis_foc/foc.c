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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#define USE_TI_MATHLIB

#include "foc.h"
#include "foc_test_data.h"
#include "foc_profile_data.h"

/* C2000 Control Suite */
#include "dmctype.h"
#include "IQmathLib.h"
#include "rmp_cntl.h"
#include "rampgen.h"
#include "speed_fr.h"
#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "pi.h"
#include "svgen_dq.h"

#if defined(USE_TI_MATHLIB)
#include "ti_r5fmath_trig.h"
#endif


#define NUM_POLES       ( 8 )                   /* number of poles */
#define BASE_FREQ       ( 266.66667 )           /* Base electrical frequency (Hz) */
#define PWM_PERIOD      ( 100000 )              /* 100 usec for 10 kHz */
#define ISR_FREQUENCY   ( 1000000.0 / PWM_PERIOD )

/*
 * FOC variable and instance definitions
*/
#define PI_SPD_KP       ( 50.0 )
#define PI_SPD_KI       ( T * gSpeedLoopPrescaler / 0.2 )
#define PI_SPD_UMIN     ( -1.0 )
#define PI_SPD_UMAX     ( 1.0 )

#define PI_ID_KP        ( 0.3 )
#define PI_ID_KI        ( 0.003 )
#define PI_ID_UMIN      ( -0.95 )
#define PI_ID_UMAX      ( 0.95 )

#define PI_IQ_KP        ( 0.3 )
#define PI_IQ_KI        ( 0.003 )
#define PI_IQ_UMIN      ( -0.95 )
#define PI_IQ_UMAX      ( 0.95 )

float T = 0.001 / ISR_FREQUENCY;    /* sampling period in sec for 10 kz T=0.0001. */
uint16_t gSpeedLoopPrescaler = 10;  /* speed loop pre-scaler */

/* Input test data */
_iq gIdRef = _IQ(0.0);              /* Id reference (per-unit) */
_iq  gSpeedRef = _IQ(0.15);         /* for Open Loop tests */
_iq gInData[4];             /* 0- Input Current Phase A; 1- Input Current Phase B; 2- Input Current Phase C; 3- Voltage of one of the Phases */
_iq gElecTheta = _IQ(0.0);  /* single_turn Theta, Electrical: converted from mechanical */

/* Calculated data */
_iq gSinElecTheta; /* sin of electrical angle */
_iq gCosElecTheta; /* cos of electrical angle */

RMPCNTL gRmpCntl = RMPCNTL_DEFAULTS;    /* Ramp controller to smoothly ramp the speed */
RAMPGEN gRampGen = RAMPGEN_DEFAULTS;    /* Ramp generator to simulate an electrical angle */
SPEED_MEAS_QEP gSpeedMeasQep = SPEED_MEAS_QEP_DEFAULTS;
CLARKE gClarke = CLARKE_DEFAULTS;
PARK gPark = PARK_DEFAULTS;
IPARK gIPark = IPARK_DEFAULTS;
PI_CONTROLLER gPiSpd = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER gPiId = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER gPiIq = PI_CONTROLLER_DEFAULTS;
SVGENDQ gSvgenDq = SVGENDQ_DEFAULTS;

static uint32_t gAdcSampCnt;        /* current ADC sample count */
static void readAdcSamps(_iq inData[4]);
static uint32_t gSvGenOutCnt;       /* current SVGen output count */
static void writeSvGenOut(SVGENDQ svGenDq);
static uint32_t gInvClarkeOutCnt;   /* current Inverse Clarke output count */
static void writeInvClarkeOut(float32_t Ia, float32_t Ib);

/* CMSIS Clarke transform output */
float32_t gCmsisClarkeAlphaOut;
float32_t gCmsisClarkeBetaOut;

/* CMSIS sin/cos */
float32_t gCmsisElecTheta;          /* electrical theta expressed degrees for CMSIS */
float32_t gCmsisSinElecThetaOut;    /* CMSIS sin output */
float32_t gCmsisCosElecThetaOut;    /* CMSIS cos output */
#if defined(USE_TI_MATHLIB)
float32_t gTiSinCosElecThetaOut[2];    /* TI Mathlib sin cos output */
#endif

/* CMSIS Park transform output */
float32_t gCmsisParkDsOut; 
float32_t gCmsisParkQsOut;

/* CMSIS PID (PI) instances */
arm_pid_instance_f32 gCmsisPiSpd;   /* Speed */
arm_pid_instance_f32 gCmsisPiId;    /* ID */
arm_pid_instance_f32 gCmsisPiIq;    /* IQ */
float32_t gCmsisPiSpdOut;           /* Speed output */
float32_t gCmsisPiIdOut ;            /* ID output */
float32_t gCmsisPiIqOut ;            /* IQ output */

/* CMSIS Inverse Park transform output */
float32_t gCmsisParkAlphaOut;
float32_t gCmsisParkBetaOut;

/* CMSIS Inverse Clarke transform output */
float32_t gCmsisInvClarkeIaOut;
float32_t gCmsisInvClarkeIbOut;
 
/* initialize FOC loop */
void focLoopInit(void)
{
    gAdcSampCnt = 0;      /* initialize ADC sample count */
    gSvGenOutCnt = 0;     /* initialize  SVGen output count */ 
    gInvClarkeOutCnt = 0; /* initialize Inverse Clarke count */

    gRampGen.StepAngleMax = _IQ(BASE_FREQ * T);

    /* Initialize speed estimator */
    gSpeedMeasQep.K1 = _IQ21(1 / (BASE_FREQ * T));
    gSpeedMeasQep.K2 = _IQ(1 / (1 + T * 2 * PI * 5));  /* low-pass filter cutoff frequency */
    gSpeedMeasQep.K3 = _IQ(1) - gSpeedMeasQep.K2;
    gSpeedMeasQep.BaseRpm = BASE_FREQ * 60 * (2 / NUM_POLES);

    /* Initialize PI Speed */
    gPiSpd.Kp = _IQ(PI_SPD_KP);
    gPiSpd.Ki = _IQ(PI_SPD_KI);
    gPiSpd.Umin = _IQ(PI_SPD_UMIN);
    gPiSpd.Umax = _IQ(PI_SPD_UMAX);

    /* Initialize PI Id */
    gPiId.Kp = _IQ(PI_ID_KP);
    gPiId.Ki = _IQ(PI_ID_KI);
    gPiId.Umin = _IQ(PI_ID_UMIN);
    gPiId.Umax = _IQ(PI_ID_UMAX);

    /* Initialize PI Iq */
    gPiIq.Kp = _IQ(PI_IQ_KP);
    gPiIq.Ki = _IQ(PI_IQ_KI);
    gPiIq.Umin = _IQ(PI_IQ_UMIN);
    gPiIq.Umax = _IQ(PI_IQ_UMAX);

    /* Initialize CMSIS PI Speed */
    gCmsisPiSpd.Kp = PI_SPD_KP;
    gCmsisPiSpd.Ki = PI_SPD_KI;
    gCmsisPiSpd.Kd = 0;
    arm_pid_init_f32(&gCmsisPiSpd, 1);

    /* Initialize CMSIS PI Id */
    gCmsisPiId.Kp = PI_ID_KP;
    gCmsisPiId.Ki = PI_ID_KI;
    gCmsisPiId.Kd = 0;
    arm_pid_init_f32(&gCmsisPiId, 1);

    /* Initialize CMSIS PI Iq */
    gCmsisPiIq.Kp = PI_IQ_KP;
    gCmsisPiIq.Ki = PI_IQ_KI;
    gCmsisPiIq.Kd = 0;
    arm_pid_init_f32(&gCmsisPiIq, 1);
}

void focLoop(uint16_t loopCnt)
{
    uint32_t start, end;

    /* Initialize FOC loop */
    focLoopInit();

    /* Get ADC samples */
    readAdcSamps(gInData);

    CycleCounterP_reset();
    start = CycleCounterP_getCount32();

    gRmpCntl.TargetValue = gSpeedRef;
    RC_MACRO(gRmpCntl);

    /* Calculate electrical angle based on EnDat position feedback */
    /* calcElecTheta(&gElecTheta); */
    /* Ramp generator simulates electrical angle output from encoder */
    gRampGen.Freq = gRmpCntl.SetpointValue;
    RG_MACRO(gRampGen);
    gElecTheta = gRampGen.Out;

    /* Connect inputs to speed calculation macro, 
       call speed calculation macro */
    gSpeedMeasQep.ElecTheta = gElecTheta;
    SPEED_FR_MACRO(gSpeedMeasQep);

    /* Connect inputs to Clarke transform macro, call macro */
    gClarke.As = gInData[0];
    gClarke.Bs = gInData[1];    
    /* CMSIS library call clarke */
    arm_clarke_f32((float32_t)gInData[0], (float32_t)gInData[1], &gCmsisClarkeAlphaOut, &gCmsisClarkeBetaOut);

    /* Compute sin/cos of electrical angle using CMSIS-DSPLIB */
    gCmsisElecTheta = gElecTheta*360;
    if (gCmsisElecTheta > 180.0)
    {
        gCmsisElecTheta = gCmsisElecTheta - 360.0;
    }

#if defined(USE_TI_MATHLIB)
    /* TI_R5FMATHLIB library call sin_cos */
    ti_r5fmath_sincos(gCmsisElecTheta, ti_r5fmath_PIconst, ti_r5fmath_sincosCoef, gTiSinCosElecThetaOut);
#else
    /* CMSIS library call sin_cos */
    arm_sin_cos_f32(gCmsisElecTheta, &gCmsisSinElecThetaOut, &gCmsisCosElecThetaOut);
#endif

    /* Connect inputs to Park transform macro, call macro */
    gPark.Alpha = gClarke.Alpha;
    gPark.Beta = gClarke.Beta;
    gPark.Angle = gElecTheta; /* unused */
    gPark.Sine = gSinElecTheta;
    gPark.Cosine = gCosElecTheta;

    /* CMSIS library call park */
    arm_park_f32(gCmsisClarkeAlphaOut, gCmsisClarkeBetaOut, &gCmsisParkDsOut, &gCmsisParkQsOut, gCmsisSinElecThetaOut, gCmsisCosElecThetaOut);

    /* CMSIS library call pid */
    gCmsisPiSpdOut = arm_pid_f32(&gCmsisPiSpd, gSpeedRef-gSpeedMeasQep.Speed);

    /* CMSIS library call pid */
    gCmsisPiIqOut = arm_pid_f32(&gCmsisPiIq, gCmsisPiSpdOut-gCmsisParkQsOut);

    /* CMSIS library call pid */
    gCmsisPiIdOut = arm_pid_f32(&gCmsisPiId, gIdRef-gCmsisParkDsOut);

    /* CMSIS library call inv-park */
    arm_inv_park_f32(gCmsisPiIdOut, gCmsisPiIqOut, &gCmsisParkAlphaOut, &gCmsisParkBetaOut, gCmsisSinElecThetaOut, gCmsisCosElecThetaOut);

    /* CMSIS library call inv-clarke */
    /* Connect inputs to Inverse Clark transform */
    /* Note: only purpose is benchmarking inverse Clarke transform in CMSIS-DSPLIB.
       This is because no SVGen macro exists in CMSIS-DSPLIB */
    arm_inv_clarke_f32(gCmsisParkAlphaOut, gCmsisParkBetaOut, &gCmsisInvClarkeIaOut, &gCmsisInvClarkeIbOut);

    writeSvGenOut(gSvgenDq);
    writeInvClarkeOut(gCmsisInvClarkeIaOut, gCmsisInvClarkeIbOut);

    end = CycleCounterP_getCount32();

    App_statsUpdate(start, end);
}

/* Read ADC samples */
static void readAdcSamps(_iq inData[4])
{
    uint32_t sampU;
    int32_t sampI;
    float sampF;

    /* Input Phase Current A */
    sampU = (uint32_t)gTestInAdcDataA[gAdcSampCnt];
    sampI = sampU - 32768;              /* unsigned to signed */
    sampI = sampI - ADC_SAMP_OFFSET;    /* remove DC offset */
    sampF = (float)sampI/32768.0 * ADC_SAMP_SCALEF;
    inData[0] = _IQ(sampF);

    /* Input Phase Current B */
    sampU = (uint32_t)gTestInAdcDataB[gAdcSampCnt];
    sampI = sampU - 32768;              /* unsigned to signed */
    sampI = sampI - ADC_SAMP_OFFSET;    /* remove DC offset */
    sampF = (float)sampI/32768.0 * ADC_SAMP_SCALEF;
    inData[1] = _IQ(sampF);

    gAdcSampCnt++;
}

/* Write Space Vector Generation outputs */
static void writeSvGenOut(SVGENDQ svGenDq)
{
    gTestOutSvGenTa[gSvGenOutCnt] = svGenDq.Ta;
    gTestOutSvGenTb[gSvGenOutCnt] = svGenDq.Tb;
    gTestOutSvGenTc[gSvGenOutCnt] = svGenDq.Tc;

    gSvGenOutCnt++;
}

/* Write Inverse Clarke outputs */
static void writeInvClarkeOut(float32_t Ia, float32_t Ib)
{
    gTestOutInvClarkeIa[gInvClarkeOutCnt] = Ia;
    gTestOutInvClarkeIb[gInvClarkeOutCnt] = Ib;

    gInvClarkeOutCnt++;
}
