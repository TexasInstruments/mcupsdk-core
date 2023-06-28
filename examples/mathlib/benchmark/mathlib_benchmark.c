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

#include <math.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <mathlib/trig/ti_arm_trig.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define SIN                 0
#define COS                 1
#define SINCOS              2
#define ASIN                3
#define ACOS                4
#define ATAN                5
#define ATAN2               6
#define TESTSIZE            500
#define MAX_TRIG_ERR        0.00001

#define TEST_DATA_SECTION   __attribute__((aligned(8), section(".testData")))

#ifndef PI
#define PI                  3.14159265358979323846f
#endif

TEST_DATA_SECTION float xvals[TESTSIZE];
TEST_DATA_SECTION float yvals[TESTSIZE];
TEST_DATA_SECTION float trigApprox[TESTSIZE];
TEST_DATA_SECTION float trigApprox2[TESTSIZE];
TEST_DATA_SECTION uint32_t fxnTimes[TESTSIZE];
TEST_DATA_SECTION uint32_t cycleOverhead[TESTSIZE];
TEST_DATA_SECTION uint32_t fxnTimesMathlib[TESTSIZE];
TEST_DATA_SECTION uint32_t cycleOverheadMathlib[TESTSIZE];

TRIG_DATA_SECTION float gSinCosRetVals[2];
float trigErr[TESTSIZE];
double gTrigRef[TESTSIZE];

void createInputs(uint32_t n);
void runTrigBench(uint32_t n);
int32_t printOutput(uint32_t n);

/*
 * Trig Bench Example
 * ----------------------------
 *   Benchmarks the speed and accuracy of the trigonometric functions in the TI Arm Trig Math Lib.
 *   Selects values along the unit circle to be used as input.
 *   Uses the double-precision trigonometric functions from Math.h to check against.
 */

void mathlib_benchmark_main(void)
{
    uint32_t i; /* Loop counter */
    int32_t status = SystemP_SUCCESS;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("\n\nTrig Benchmark Test \r\n\n");

    /* Print Benchmark conditions */
    DebugP_log("BENCHMARK START - MATHLIB - MATHLIB BENCHMARK \r\n");
    DebugP_log("- Calculated for the 500 samples taken between 0 and 2 * Pi \r\n");
    DebugP_log("- Trignometric function timings compared between the optimized Mathlib mcusdk implementation and "
                "the compiler mathlib version\r\n");
    DebugP_log("- The max error for each operation between the optimized Mathlib mcusdk functions and"
                " the compiler mathlib version is printed \r\n");

    DebugP_log("\n\nFunction\t| Err\t\t| Max Cycles Mathlib (mcusdk) \t| avg cycles Mathlib (mcusdk) \t| max cycles mathlib (clang) \t| avg cycles mathlib (clang) \t|\r\n");
        DebugP_log("----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|\r\n");
    for (i = 0; i < 7; i++)
    {
        /* Reset and enable the core cycle counter */
        CycleCounterP_reset();

        /* Create test inputs */
        createInputs(i);

        runTrigBench(i);

        status = printOutput(i);
    }

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("BENCHMARK END \r\n");
        DebugP_log("Trig Benchmark Test Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

void createInputs(uint32_t n)
{

    uint32_t i;
    uint32_t t1, t2, t3;
    volatile float trigVal;
    float testStep;

    testStep = 2 * PI / (float) TESTSIZE;

    switch (n)
    {

    case SIN:
        DebugP_log("sin \t\t|");
        for (i = 0; i < TESTSIZE; i++)
        {
            xvals[i] = (float) i * testStep;
            t1 = CycleCounterP_getCount32();
            trigVal = sin(xvals[i]);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            gTrigRef[i] = trigVal;
            fxnTimesMathlib[i] = t2 - t1;
            cycleOverheadMathlib[i] = t3 - t2;
        }
        break;

    case COS:
        DebugP_log("cos  \t\t|");
        for (i = 0; i < TESTSIZE; i++)
        {
            xvals[i] = (float) i * testStep;
            t1 = CycleCounterP_getCount32();
            trigVal = cos(xvals[i]);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            gTrigRef[i] = trigVal;
            fxnTimesMathlib[i] = t2 - t1;
            cycleOverheadMathlib[i] = t3 - t2;
        }
        break;

    case SINCOS:
        DebugP_log("sincos sin  \t|");
        for (i = 0; i < TESTSIZE; i++)
        {
            xvals[i] = (float) i * testStep;
            t1 = CycleCounterP_getCount32();
            trigVal = sin(xvals[i]);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            gTrigRef[i] = trigVal;
            fxnTimesMathlib[i] = t2 - t1;
            cycleOverheadMathlib[i] = t3 - t2;
        }
        break;

    case ASIN:
        DebugP_log("asin \t\t|");
        testStep = 2 / ((float) TESTSIZE);

        for (i = 0; i <= (float) TESTSIZE / 2; i++)
        {
            xvals[i] = testStep * i;
        }
        for (i = TESTSIZE / 2; i < (float) (TESTSIZE); i++)
        {
            xvals[i] = xvals[i - TESTSIZE / 2] * (-1);
        }
        for (i = 0; i < TESTSIZE; i++)
        {
            t1 = CycleCounterP_getCount32();
            trigVal = asin(xvals[i]);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            gTrigRef[i] = trigVal;
            fxnTimesMathlib[i] = t2 - t1;
            cycleOverheadMathlib[i] = t3 - t2;
        }
        break;

    case ACOS:
        DebugP_log("acos \t\t|");
        testStep = 2 / ((float) TESTSIZE);

        for (i = 0; i <= (float) TESTSIZE / 2; i++)
        {
            xvals[i] = testStep * i;
        }
        for (i = TESTSIZE / 2; i < (float) (TESTSIZE); i++)
        {
            xvals[i] = xvals[i - TESTSIZE / 2] * (-1);
        }
        for (i = 0; i < TESTSIZE; i++)
        {
            t1 = CycleCounterP_getCount32();
            trigVal = acos(xvals[i]);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            gTrigRef[i] = trigVal;
            fxnTimesMathlib[i] = t2 - t1;
            cycleOverheadMathlib[i] = t3 - t2;
        }
        break;

    case ATAN:
        DebugP_log("atan \t\t|");
        for (i = 0; i < (float) TESTSIZE; i++)
        {
            xvals[i] = cos(testStep * i + 2 * PI);
            yvals[i] = (float) sin(testStep * i + 2 * PI);

            if (ti_arm_abs(xvals[i]) > 0)
            {
                t1 = CycleCounterP_getCount32();
                trigVal = atan(yvals[i] / xvals[i]);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                gTrigRef[i] = trigVal;
                fxnTimesMathlib[i] = t2 - t1;
                cycleOverheadMathlib[i] = t3 - t2;
            }
            else
            {
                t1 = CycleCounterP_getCount32();
                trigVal = (float) PI / 2.0f;
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                gTrigRef[i] = trigVal;
                fxnTimesMathlib[i] = t2 - t1;
                cycleOverheadMathlib[i] = t3 - t2;
            }
        }
        break;

    case ATAN2:
        DebugP_log("atan2 \t\t|");
        for (i = 0; i < (float) TESTSIZE; i++)
        {
            xvals[i] = (float) cos(testStep * i + 2 * PI);
            yvals[i] = (float) sin(testStep * i + 2 * PI);
            if (yvals[i] != 0)
            {
                t1 = CycleCounterP_getCount32();
                trigVal = atan2(yvals[i], xvals[i]);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                gTrigRef[i] = trigVal;
                fxnTimesMathlib[i] = t2 - t1;
                cycleOverheadMathlib[i] = t3 - t2;
            }
            else
            {
                t1 = CycleCounterP_getCount32();
                trigVal = (float) PI / 2.0f;
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                gTrigRef[i] = trigVal;
                fxnTimesMathlib[i] = t2 - t1;
                cycleOverheadMathlib[i] = t3 - t2;
            }
        }
        break;
    default:
        DebugP_log("Error: invalid fxn num\r\n");
        while (1)
            ;
    }
}

void runTrigBench(uint32_t n)
{

    uint32_t i;
    uint32_t t1, t2, t3;
    volatile float trigVal;

    float x, y;

    switch (n)
    {

    case SIN:
        for (i = 0; i < TESTSIZE; i++)
        {
            t1 = CycleCounterP_getCount32();
            trigVal = ti_arm_sin(xvals[i]); //TI Arm Trig Lib
//            trigVal = sinf(xvals[i]); //C STD Lib (Math.h)
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            trigApprox[i] = trigVal;
            fxnTimes[i] = t2 - t1;
            cycleOverhead[i] = t3 - t2;
        }
        break;

    case COS:
        for (i = 0; i < TESTSIZE; i++)
        {
            t1 = CycleCounterP_getCount32();
            trigVal = ti_arm_cos(xvals[i]); //TI Arm Trig Lib
//            trigVal = cosf(xvals[i]); //C STD Lib (Math.h)
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            trigApprox[i] = trigVal;
            fxnTimes[i] = t2 - t1;
            cycleOverhead[i] = t3 - t2;
        }
        break;

    case SINCOS:
        for (i = 0; i < TESTSIZE; i++)
        {
            t1 = CycleCounterP_getCount32();
            ti_arm_sincos(xvals[i], gSinCosRetVals); //TI Arm Trig Lib
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            trigApprox[i] = gSinCosRetVals[0];
            trigApprox2[i] = gSinCosRetVals[1];
            fxnTimes[i] = t2 - t1;
            cycleOverhead[i] = t3 - t2;
        }
        break;

    case ASIN:
        for (i = 0; i < TESTSIZE; i++)
        {
            t1 = CycleCounterP_getCount32();
            trigVal = ti_arm_asin(xvals[i]); //TI Arm Trig Lib
//            trigVal = asinf(xvals[i]); //C STD Lib (Math.h)
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            trigApprox[i] = trigVal;
            fxnTimes[i] = t2 - t1;
            cycleOverhead[i] = t3 - t2;
        }
        break;

    case ACOS:
        for (i = 0; i < TESTSIZE; i++)
        {
            t1 = CycleCounterP_getCount32();
            trigVal = ti_arm_acos(xvals[i]); //TI Arm Trig Lib
//            trigVal = acosf(xvals[i]); C STD Lib (Math.h)
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            trigApprox[i] = trigVal;
            fxnTimes[i] = t2 - t1;
            cycleOverhead[i] = t3 - t2;
        }
        break;

    case ATAN:
        for (i = 0; i < TESTSIZE; i++)
        {
            volatile float x1 = yvals[i] / (float) xvals[i];
            t1 = CycleCounterP_getCount32();
            trigVal = ti_arm_atan(x1); //TI Arm Trig Lib
//            trigVal = atanf(x1); //C STD Lib (Math.h)
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            trigApprox[i] = trigVal;
            fxnTimes[i] = t2 - t1;
            cycleOverhead[i] = t3 - t2;
        }

        break;

    case ATAN2:
        for (i = 0; i < TESTSIZE; i++)
        {
            x = xvals[i];
            y = yvals[i];
            if (fabs(y) > 0)
            {
                t1 = CycleCounterP_getCount32();
                trigVal = ti_arm_atan2(y, x); //TI Arm Trig Lib
//                trigVal = atan2f(y, x);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                trigApprox[i] = trigVal;
            }
            else
            {
                trigApprox[i] = PI / 2;
            }
            fxnTimes[i] = t2 - t1;
            cycleOverhead[i] = t3 - t2;
        }
        break;
    default:
        DebugP_log("Invalid fxn\r\n");
    }
}

int32_t printOutput(uint32_t n)
{

    uint32_t i;
    uint32_t totalCycles, totalOverhead, maxCycles, maxCyclesMathlib;
    float avgCycles, avgOverhead;
    float avgCyclesMathlib, avgOverheadMathlib;
    double maxErr, maxErr2;
    totalOverhead = 0;
    totalCycles = 0;
    maxCycles = 0;
    avgCycles = 0;
    avgOverhead = 0;
    maxCyclesMathlib = 0;
    avgCyclesMathlib = 0;
    avgOverheadMathlib = 0;
    maxErr = 0;
    maxErr2 = 0;
    int32_t status = SystemP_SUCCESS;

    for (i = 0; i < TESTSIZE; i++)
    {
        totalCycles += fxnTimes[i];
        totalOverhead += cycleOverhead[i];
        trigErr[i] = (float) fabs(gTrigRef[i] - trigApprox[i]);
        if (trigErr[i] > maxErr)
            maxErr = trigErr[i];
        if (i > 10)
            if ((fxnTimes[i] - cycleOverhead[i]) > maxCycles)
                maxCycles = fxnTimes[i] - cycleOverhead[i];
    }
    avgCycles = (float) (totalCycles / (float) TESTSIZE);
    avgOverhead = (float) (totalOverhead / (float) TESTSIZE);

    totalOverhead = 0;
    totalCycles = 0;

    for (i = 0; i < TESTSIZE; i++)
    {
        totalCycles += fxnTimesMathlib[i];
        totalOverhead += cycleOverheadMathlib[i];
        if (i > 10)
            if ((fxnTimesMathlib[i] - cycleOverheadMathlib[i]) > maxCyclesMathlib)
                maxCyclesMathlib = fxnTimesMathlib[i] - cycleOverheadMathlib[i];
    }
    avgCyclesMathlib = (float) (totalCycles / (float) TESTSIZE);
    avgOverheadMathlib = (float) (totalOverhead / (float) TESTSIZE);


    DebugP_log("%.10f\t| %d\t\t\t| %f \t\t| %d\t\t\t| %f\t\t|\r\n",
           maxErr, maxCycles, (avgCycles - avgOverhead), maxCyclesMathlib, (avgCyclesMathlib - avgOverheadMathlib));

    if (n == SINCOS)
    {
        for (i = 0; i < TESTSIZE; i++)
        {
            trigErr[i] = (float) fabs(cos(xvals[i]) - trigApprox2[i]);
            if (trigErr[i] > maxErr2)
                maxErr2 = trigErr[i];
        }

        DebugP_log("sincos cos\t|%.10f\t|\t\t\t|\t\t\t|\t\t\t|\t\t\t|\r\n", maxErr2);
    }
    if ((maxErr > MAX_TRIG_ERR) || (maxErr2 > MAX_TRIG_ERR))
    {
        status = SystemP_FAILURE;
    }
    return status;
}