
/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
#include <mathlib/trig/ti_tmu_trig.h>


#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define SIN                 0
#define COS                 1
#define ATAN                2
#define EXP                 3
#define LOG                 4
#define ATAN2               5
#define SINCOS              6



#define TESTSIZE            500
#define TESTSIZE1           490
#define MAX_TRIG_ERR        0.00001

#define TEST_DATA_SECTION   __attribute__((aligned(8), section(".testData")))

#ifndef PI
#define PI                  3.14159265358979323846f
#endif

#ifndef PIby4
#define PIby4                  0.7853981634f
#endif

#ifndef ReciprocalOf2PI
#define ReciprocalOf2PI                0.159154943091895335768f
#endif


TEST_DATA_SECTION float xvals[TESTSIZE];
TEST_DATA_SECTION float yvals[TESTSIZE];
TEST_DATA_SECTION float xvals_arm[TESTSIZE];
TEST_DATA_SECTION float yvals_arm[TESTSIZE];
TEST_DATA_SECTION float xvals_tmu[TESTSIZE];
TEST_DATA_SECTION float yvals_tmu[TESTSIZE];
TEST_DATA_SECTION float trigApprox[TESTSIZE];
TEST_DATA_SECTION float trigApprox2[TESTSIZE];
TEST_DATA_SECTION float tmuApprox[TESTSIZE];
TEST_DATA_SECTION float tmuApprox2[TESTSIZE];
TEST_DATA_SECTION uint32_t fxnTimes[TESTSIZE];
TEST_DATA_SECTION uint32_t cycleOverhead[TESTSIZE];
TEST_DATA_SECTION uint32_t fxnTimesMathlib[TESTSIZE];
TEST_DATA_SECTION uint32_t cycleOverheadMathlib[TESTSIZE];
TEST_DATA_SECTION uint32_t fxnTimesTMU[TESTSIZE];
TEST_DATA_SECTION uint32_t cycleOverheadTMU[TESTSIZE];

TRIG_DATA_SECTION float gSinCosRetVals[2];
float trigErr[TESTSIZE];
float tmuErr[TESTSIZE];
float gTrigRef[TESTSIZE];
float gTrigRef2[TESTSIZE];
uint32_t POS_INFINITY;
uint32_t NEG_INFINITY;
uint32_t POS_ZERO;
uint32_t POS_NAN;
uint32_t NEG_NAN;
uint32_t POS_NORM;


void createInputs(uint32_t n);
void runTrigBench(uint32_t n);
void runTMUBench(uint32_t n);
int32_t printOutput(uint32_t n);



// float ti_arm_atan2(float e);


/*
 * Trig Bench Example
 * ----------------------------
 *   Benchmarks the speed and accuracy of the trigonometric functions in the Trigonometric Math Unit and TI Arm Trig Math Lib.
 *   Selects values along the unit circle to be used as input.
 *   Uses the double-precision trigonometric functions from Math.h to check against.
 */


void tmu_operations_main(void *args)
{
    uint32_t i; /* Loop counter */
    int32_t status = SystemP_SUCCESS;


    Drivers_open();
    Board_driversOpen();

    DebugP_log("TMU operations example Started ...\r\n");

    for (i = 0; i < 7; i++)
    {
        /* Reset and enable the core cycle counter */
        CycleCounterP_reset();

        /* Create test inputs and running math library functions*/
        createInputs(i);
        /* Running TI ARM SDK Trig library */
        runTrigBench(i);
        /* Running the TMU initrinsic functions */
        runTMUBench(i);

        status = printOutput(i);
    }

    if(status == SystemP_SUCCESS)
    {
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
    float test_tan;
    volatile float trigVal, trigVal2;
    float testStep, testStep_tan, testStep_quad, input, input2, input_tan1, input_tan2, tan_input;
    float x_quad, y_quad;
    // Positive infinity
    POS_INFINITY = 0x7f800000;
    // Negative infinity
    NEG_INFINITY = 0xff800000;
    // Positive NAN
    POS_NAN = 0x7fffffff;
    // Negative NAN
    NEG_NAN = 0xffffffff;
    // Zero
    POS_ZERO = 0x00000000;
    // Positive normal
    POS_NORM = 0x7f7fffff;



    uint32_t cnt=0;

    switch (n)
    {

    case SIN:
        cnt = 0;
        DebugP_log("\nSIN FUNCTION\n");
        DebugP_log("Clang mathlib sin \n");
        for (i = 0; i < 5; i++)
        {
            switch (i)
            {
                case 0: //Calculating sin values for 500 test samples
                    testStep = 2 * PI / (float) TESTSIZE;
                    for(cnt = 0; cnt < TESTSIZE; cnt++)
                    {
                        xvals[cnt] = (float) cnt * testStep;
                        xvals_arm[cnt] = xvals[cnt];
                        xvals_tmu[cnt] = xvals[cnt];
                        t1 = CycleCounterP_getCount32();
                        trigVal = sinf(xvals[cnt]);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        gTrigRef[cnt] = trigVal;
                        fxnTimesMathlib[cnt] = t2 - t1;
                        cycleOverheadMathlib[cnt] = t3 - t2;
                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = sinf(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function mathlib input (POS_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = sinf(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function mathlib input (NEG_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = sinf(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function mathlib input (+NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = sinf(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function mathlib input (-NaN) %f : %f \r\n", input , trigVal);
                    break;

                default:
                     DebugP_log("Invalid fxn\r\n");

            }
        }
        break;

    case COS:
        cnt = 0;
        DebugP_log("\nCOS FUNCTION \n");
        DebugP_log("Clang mathlib cos \n");
        for (i = 0; i < 5; i++)
        {
            switch (i)
            {
                case 0: //Calculating cos values for 500 test samples
                    testStep = 2 * PI / (float) TESTSIZE;
                    for(cnt = 0; cnt < TESTSIZE; cnt++)
                    {
                        xvals[cnt] = (float) cnt * testStep;
                        xvals_arm[cnt] = xvals[cnt];
                        xvals_tmu[cnt] = xvals[cnt];
                        t1 = CycleCounterP_getCount32();
                        trigVal = cosf(xvals[cnt] );
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        gTrigRef[cnt] = trigVal;
                        fxnTimesMathlib[cnt] = t2 - t1;
                        cycleOverheadMathlib[cnt] = t3 - t2;

                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = cosf(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function mathlib input (POS_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = cosf(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function mathlib input (NEG_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = cosf(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function mathlib input (POS_NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = cosf(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function mathlib input (NEG_NaN) %f : %f \r\n", input , trigVal);
                    break;

                default:
                     DebugP_log("Invalid fxn\r\n");

            }
        }
        break;

    case ATAN:
        DebugP_log("\nATAN FUNCTION \n");
        DebugP_log("Clang mathlib atan \n");
        cnt = 0;
        testStep_tan = 2 * PI / (float) TESTSIZE;
        for(i = 0; i < 8; i++)
        {
            switch(i)
            {
                case 0: //Calculating atan values for 500 test samples
                    for (cnt = 0; cnt < (float) TESTSIZE; cnt++)
                    {
                        test_tan = fmod((float)(testStep_tan * cnt + 2 * PI), PIby4);
                        xvals[cnt] = cosf((float)(test_tan));
                        yvals[cnt] =  sinf((float)(test_tan));
                        xvals_arm[cnt] = xvals[cnt];
                        yvals_arm[cnt] = yvals[cnt];
                        xvals_tmu[cnt] = xvals[cnt];
                        yvals_tmu[cnt] = yvals[cnt];

                    if (ti_arm_abs(xvals[cnt]) > 0)
                    {
                        tan_input = yvals[cnt] / (float)xvals[cnt];
                        t1 = CycleCounterP_getCount32();
                        trigVal = atanf(tan_input);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        gTrigRef[cnt] = trigVal;
                        fxnTimesMathlib[cnt] = t2 - t1;
                        cycleOverheadMathlib[cnt] = t3 - t2;
                    }
                    else
                    {
                        t1 = CycleCounterP_getCount32();
                        trigVal = ((float) PI / 2.0f);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        gTrigRef[cnt] = trigVal;
                        fxnTimesMathlib[cnt] = t2 - t1;
                        cycleOverheadMathlib[cnt] = t3 - t2;
                    }
                    }
                    break;

             case 1: // input: POS_ZERO/POS_ZERO
                input_tan1 = *((float *)(&POS_ZERO));
                input_tan2 = *((float *)(&POS_ZERO));
                tan_input = input_tan2 / (float)input_tan1;
                t1 = CycleCounterP_getCount32();
                trigVal = atanf(tan_input);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                DebugP_log("atan function mathlib input (ZERO/ZERO) %f %f : %f \r\n", input_tan1 , input_tan2, trigVal);
                break;

            case 2: // input: POS_ZERO/POS_INFINITY
                input_tan1 = *((float *)(&POS_INFINITY));
                input_tan2 = *((float *)(&POS_ZERO));
                tan_input = input_tan2 / (float)input_tan1;
                t1 = CycleCounterP_getCount32();
                trigVal = atanf(tan_input);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                DebugP_log("atan function mathlib input (ZERO/INFINITY) %f %f : %f \r\n", input_tan1 , input_tan2, trigVal);
                break;

            case 3: // input: POS_INFINITY/POS_ZERO
                input_tan1 = *((float *)(&POS_ZERO));
                input_tan2 = *((float *)(&POS_INFINITY));
                tan_input = input_tan2 / (float)input_tan1;
                t1 = CycleCounterP_getCount32();
                trigVal = atanf(tan_input);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                DebugP_log("atan function mathlib input (INFINITY/ZERO) %f %f : %f \r\n", input_tan1 , input_tan2, trigVal);
                break;

            case 4: // input : POS_INFINITY/POS_INFINITY
                input_tan1 = *((float *)(&POS_INFINITY));
                input_tan2 = *((float *)(&POS_INFINITY));
                tan_input = input_tan2 / (float)input_tan1;
                t1 = CycleCounterP_getCount32();
                trigVal = atanf(tan_input);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                DebugP_log("atan function mathlib input (INFINITY/INFINTY) %f %f : %f \r\n", input_tan1 , input_tan2, trigVal);
                break;

            case 5: // input : POS_INFINITY/POS_NORM
                input_tan1 = *((float *)(&POS_NORM));
                input_tan2 = *((float *)(&POS_INFINITY));
                tan_input = input_tan2 / (float)input_tan1;
                t1 = CycleCounterP_getCount32();
                trigVal = atanf(tan_input);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                DebugP_log("atan function mathlib input (INFINITY/NORMAL) %f %f : %f \r\n", input_tan1 , input_tan2, trigVal);
                break;

            case 6: // input : POS_NORMAL/POS_ZERO
                input_tan1 = *((float *)(&POS_ZERO));
                input_tan2 = *((float *)(&POS_NORM));
                tan_input = input_tan2 / (float)input_tan1;
                t1 = CycleCounterP_getCount32();
                trigVal = atanf(tan_input);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                DebugP_log("atan function mathlib input (NORMAL/ZERO) %f %f : %f \r\n", input_tan1 , input_tan2, trigVal);
                break;

            case 7: // input : POS_NORMAL/POS_INFINITY
                input_tan1 = *((float *)(&POS_INFINITY));
                input_tan2 = *((float *)(&POS_NORM));
                tan_input = input_tan2 / (float)input_tan1;
                t1 = CycleCounterP_getCount32();
                trigVal = atanf(tan_input);
                t2 = CycleCounterP_getCount32();
                t3 = CycleCounterP_getCount32();
                DebugP_log("atan function mathlib input (NORMAL/INFINITY) %f %f : %f \r\n", input_tan1 , input_tan2, trigVal);
                break;

            default:
                DebugP_log("Invalid fxn\r\n");
        }
    }
    break;

    case EXP:
        DebugP_log("\nEXPONENTIAL FUNCTION \n");
        DebugP_log("Clang mathlib exp \n");
        cnt = 0;

       for (i = 0; i < 7; i++)
        {
            switch (i)
            {
                case 0: // Calculating exponent for 500 test cases
                    for(cnt = 0; cnt < TESTSIZE; cnt++)
                    {
                        xvals[cnt] = (float)(cnt+1)/TESTSIZE;
                        xvals_arm[cnt] = xvals[cnt];
                        xvals_tmu[cnt] = xvals[cnt];
                        t1 = CycleCounterP_getCount32();
                        trigVal =  powf(2, (fabsf(xvals[cnt]) * -1));
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        gTrigRef[cnt] = trigVal;
                        fxnTimesMathlib[cnt] = t2 - t1;
                        cycleOverheadMathlib[cnt] = t3 - t2;
                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = powf(2, (fabsf(input) * -1));
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function mathlib input (POS_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = powf(2, (fabsf(input) * -1));
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function mathlib input (NEG_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = powf(2, (fabsf(input) * -1));
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function mathlib input (POS_NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = powf(2, (fabsf(input) * -1));
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function mathlib input (NEG_NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 5: //input: inf/nan
                    input = *((float *)(&POS_INFINITY));
                    input2 = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = powf(2, (fabsf(input) * -1));
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function mathlib input (INF/POS_NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 6: //input: -inf/nan
                    input = *((float *)(&NEG_INFINITY));
                    input2 = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = powf(2, (fabsf(input) * -1));
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function mathlib input (-INF/POS_NaN) %f : %f \r\n", input , trigVal);
                    break;

                default:
                    DebugP_log("Invalid fxn\r\n");

            }

        }
        break;


    case LOG:
        DebugP_log("\nLOG FUNCTION \n");
        DebugP_log("Clang mathlib log \n");
        cnt = 0;
        for (i = 0; i < 7; i++)
        {
            switch (i)
            {
                case 0: //Calculating log value of 500 test cases
                    for(cnt = 0; cnt < TESTSIZE; cnt++)
                    {
                        xvals[cnt] = (float) cnt;
                        xvals_arm[cnt] = xvals[cnt];
                        xvals_tmu[cnt] = xvals[cnt];
                        t1 = CycleCounterP_getCount32();
                        trigVal = log2f(xvals[cnt]);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        gTrigRef[cnt] = trigVal;
                        fxnTimesMathlib[cnt] = t2 - t1;
                        cycleOverheadMathlib[cnt] = t3 - t2;
                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = log2f(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function mathlib input (POS_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = log2f(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function mathlib input (NEG_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = log2f(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function mathlib input (POS_NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = log2f(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function mathlib input (NEG_NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 5: //input: inf/nan
                    input = *((float *)(&POS_INFINITY));
                    input2 = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = log2f((float)input/input2);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function mathlib input (POS_INF/POS_NAN) %f %f : %f \r\n", input , input2 , trigVal);
                    break;

                case 6: //input: -inf/nan
                    input = *((float *)(&NEG_INFINITY));
                    input2 = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = log2f((float)input/input2);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function mathlib input (NEG_INF/POS_NAN) %f %f : %f \r\n", input , input2 , trigVal);
                    break;


                default:
                     DebugP_log("Invalid fxn\r\n");

            }
        }
        break;

    case ATAN2:
        DebugP_log("\nATAN2 FUNCTION \n");
        testStep = 2 * PI / (float) TESTSIZE;
        for(i = 0; i < TESTSIZE; i++)
        {
            testStep_quad = (float)(i * testStep + 2* PI);
            x_quad = cos( (float)(testStep_quad));
            y_quad = sin((float)(testStep_quad));

            xvals[i] = x_quad;
            yvals[i] = y_quad;
            xvals_arm[i] = xvals[i];
            yvals_arm[i] = yvals[i];
            xvals_tmu[i] = xvals[i];
            yvals_tmu[i] = yvals[i];

            t1 = CycleCounterP_getCount32();
            trigVal =atan2f(yvals[i], xvals[i]);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            gTrigRef[i] = trigVal;

            fxnTimesMathlib[i] = t2 - t1;
            cycleOverheadMathlib[i] = t3 - t2;

        }
        break;

        case SINCOS:
        cnt = 0;
        DebugP_log("\nSINCOS FUNCTION \n");
        testStep = 2 * PI / (float) TESTSIZE;
        for(cnt = 0; cnt < TESTSIZE; cnt++)
        {
            xvals[cnt] = (float) cnt * testStep;
            xvals_arm[cnt] = xvals[cnt];
            xvals_tmu[cnt] = xvals[cnt];
            t1 = CycleCounterP_getCount32();
            trigVal = sinf(xvals[cnt]);
            trigVal2 = cosf(xvals[cnt]);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            gTrigRef[cnt] = trigVal;
            gTrigRef2[cnt] = trigVal2;
            fxnTimesMathlib[cnt] = t2 - t1;
            cycleOverheadMathlib[cnt] = t3 - t2;
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

    uint32_t i, j;
    uint32_t t1, t2, t3;
    volatile float trigVal, trigVal2;
    float  input, input_tan1, input_tan2, tan_input;


    switch (n)
    {

    case SIN:
        DebugP_log("MCU SDK SIN \n");
        for(i = 0; i < 5; i++)
        {
            switch(i)
            {
                case 0: //Calculating the sine values using TI ARM SDK Trig library
                    for (j = 0; j < TESTSIZE; j++)
                    {
                        t1 = CycleCounterP_getCount32();
                        trigVal = ti_arm_sin(xvals_arm[j]);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        trigApprox[j] = trigVal;
                        fxnTimes[j] = t2 - t1;
                        cycleOverhead[j] = t3 - t2;

                    }
                    break;

                case 1:  //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_sin(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function sdk input (POS_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_sin(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function sdk input (NEG_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_sin(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function sdk input (+NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_sin(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function sdk input (-NaN) %f : %f \r\n", input , trigVal);
                    break;

                default:
                     DebugP_log("Invalid fxn\r\n");
            }

        }
        break;

    case COS:
        DebugP_log("MCU SDK COS  \n");
        for(i = 0; i < 5; i++)
        {
            switch(i)
            {
                case 0: //Calculating cos values using TI ARM SDK Trig library
                    for (j = 0; j < TESTSIZE; j++)
                    {
                        t1 = CycleCounterP_getCount32();
                        trigVal = ti_arm_cos(xvals_arm[j]);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        trigApprox[j] = trigVal;
                        fxnTimes[j] = t2 - t1;
                        cycleOverhead[j] = t3 - t2;
                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal =  ti_arm_cos(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function sdk input (POS_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_cos(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function sdk input (NEG_INFINITY) %f : %f \r\n", input , trigVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_cos(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function sdk input (POS_NaN) %f : %f \r\n", input , trigVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_cos(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function sdk input (NEG_NaN) %f : %f \r\n", input , trigVal);
                    break;

                 default:
                     DebugP_log("Invalid fxn\r\n");
            }
        }
        break;

    case ATAN:
        DebugP_log("MCU SDK ATAN \t\t \n");
        for ( i = 0; i < 8; i++)
        {
            switch(i)
            {
                case 0: //Calculating atan values using TI ARM SDK Trig library
                     for (j = 0; j < TESTSIZE; j++)
                    {
                        tan_input = yvals_arm[j] / (float) xvals_arm[j];
                        t1 = CycleCounterP_getCount32();
                        trigVal = ti_arm_atan(tan_input); //TI Arm Trig Lib
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();

                        trigApprox[j] = trigVal;
                        fxnTimes[j] = t2 - t1;
                        cycleOverhead[j] = t3 - t2;
                    }
                    break;

                case 1: //input: (POS_ZERO/POS_ZERO)
                    input_tan1 = *((float *)(&POS_ZERO));
                    input_tan2 = *((float *)(&POS_ZERO));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function sdk input (ZERO/ZERO) %f %f : %f \r\n", input_tan2 , input_tan1, trigVal);
                    break;

                case 2: //input: (POS_ZERO/POS_INFINITY)
                    input_tan1 = *((float *)(&POS_INFINITY));
                    input_tan2 = *((float *)(&POS_ZERO));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function sdk input (ZERO/INFINITY) %f %f : %f \r\n", input_tan2 , input_tan1, trigVal);
                    break;

                case 3: //input: (POS_INFINITY/POS_ZERO)
                    input_tan1 = *((float *)(&POS_ZERO));
                    input_tan2 = *((float *)(&POS_INFINITY));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function sdk input (INFINITY/ZERO) %f %f : %f \r\n", input_tan2 , input_tan1, trigVal);
                    break;

                case 4: //input: (POS_INFINITY/POS_INFINITY)
                    input_tan1 = *((float *)(&POS_INFINITY));
                    input_tan2 = *((float *)(&POS_INFINITY));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function sdk input (INFINITY/INFINTY) %f %f : %f \r\n", input_tan2 , input_tan1, trigVal);
                    break;

                case 5: //input: (POS_INFINITY/POS_NORMAL)
                    input_tan1 = *((float *)(&POS_NORM));
                    input_tan2 = *((float *)(&POS_INFINITY));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function sdk input (INFINITY/NORMAL) %f %f : %f \r\n", input_tan2 , input_tan1, trigVal);
                    break;

                case 6: //input: (POS_NORMAL/POS_ZERO)
                    input_tan1 = *((float *)(&POS_ZERO));
                    input_tan2 = *((float *)(&POS_NORM));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function sdk input (NORMAL/ZERO) %f %f : %f \r\n", input_tan2 , input_tan1, trigVal);
                    break;

                case 7: //input: (POS_NORMAL/POS_INFINITY)
                    input_tan1 = *((float *)(&POS_INFINITY));
                    input_tan2 = *((float *)(&POS_NORM));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    trigVal = ti_arm_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function sdk input (NORMAL/INFINITY) %f %f : %f \r\n", input_tan2 , input_tan1, trigVal);
                    break;

                default:
                    DebugP_log("Invalid fxn\r\n");
            }
        }
        break;


    case EXP:
        // DebugP_log("MCU SDK LOG \r\n");
        break;

    case LOG: // Calculating log values using TI ARM Trig function
        break;

    case ATAN2: //Caluclating ATAN2 values using TI ARM Trig function

        for (j = 0; j < TESTSIZE; j++)
        {
            t1 = CycleCounterP_getCount32();
            trigVal = ti_arm_atan2(yvals_arm[j], xvals_arm[j]); //TI Arm Trig Lib
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            trigApprox[j] = trigVal;
            fxnTimes[j] = t2 - t1;
            cycleOverhead[j] = t3 - t2;
        }
        break;

    case SINCOS:
        for(j = 0; j < TESTSIZE; j++)
        {
            t1 = CycleCounterP_getCount32();
            trigVal = ti_arm_sin(xvals_arm[j]);
            trigVal2 = ti_arm_cos(xvals_arm[j]);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            trigApprox[j] = trigVal;
            trigApprox2[j] = trigVal2;
            fxnTimesMathlib[j] = t2 - t1;
            cycleOverheadMathlib[j] = t3 - t2;
        }
        break;

    default:
        DebugP_log("Invalid fxn\r\n");
    }
}

void runTMUBench(uint32_t n)
{
    uint32_t i, j;
    uint32_t t1, t2, t3;
    volatile float tmuVal;
    float  input, input2, tan_input, input_tan1, input_tan2;
    float sin_val, cos_val;



    switch (n)
    {

    case SIN:
        DebugP_log("TMU SIN \n");
        for(i = 0; i < 5; i++)
        {
            switch(i)
            {
                case 0: //Calculating sin values using TMU instrinsic functions
                    for (j = 0; j < TESTSIZE; j++)
                    {
                        t1 = CycleCounterP_getCount32();
                        tmuVal =  ti_tmu_sin(xvals_tmu[j]);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        tmuApprox[j] = tmuVal;
                        fxnTimesTMU[j] = t2 - t1;
                        cycleOverheadTMU[j] = t3 - t2;
                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_sin(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function tmu input (POS_INFINITY) %f : %f \r\n", input , tmuVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_sin(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function tmu input (NEG_INFINITY) %f : %f \r\n", input , tmuVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_sin(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function tmu input (+NaN) %f : %f \r\n", input , tmuVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_sin(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("sin function tmu input (-NaN) %f : %f \r\n", input , tmuVal);
                    break;

                default:
                     DebugP_log("Invalid fxn\r\n");
            }
        }
        break;

    case COS:
        DebugP_log("TMU COS \n");
        for(i = 0; i < 5; i++)
        {
            switch(i)
            {
                case 0: //Calculating cos values using TMU intrinsic function
                    for (j = 0; j < TESTSIZE; j++)
                    {
                        t1 = CycleCounterP_getCount32();
                        tmuVal =  ti_tmu_cos(xvals_tmu[j]);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        tmuApprox[j] = tmuVal;
                        fxnTimesTMU[j] = t2 - t1;
                        cycleOverheadTMU[j] = t3 - t2;
                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal =  ti_tmu_cos(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function tmu input (POS_INFINITY) %f : %f \r\n", input , tmuVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_cos(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function tmu input (NEG_INFINITY) %f : %f \r\n", input , tmuVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal =  ti_tmu_cos(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function tmu input (POS_NaN) %f : %f \r\n", input , tmuVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal =  ti_tmu_cos(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("cos function mathlib input (NEG_NaN) %f : %f \r\n", input , tmuVal);
                    break;

                default:
                     DebugP_log("Invalid fxn\r\n");

            }

        }
        break;

    case ATAN:
        DebugP_log("TMU ATAN \n");
        for(i = 0; i < 8; i++)
        {
            switch(i)
            {
                case 0: //Calculating ATAN values using TMU intrinsic function
                    for (j = 0; j < TESTSIZE; j++)
                    {
                        tan_input = yvals_tmu[j] / (float)xvals_tmu[j];
                        t1 = CycleCounterP_getCount32();
                        tmuVal =  ti_tmu_atan(tan_input);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        tmuApprox[j] = tmuVal;
                        fxnTimesTMU[j] = t2 - t1;
                        cycleOverheadTMU[j] = t3 - t2;
                    }
                    break;

                case 1: //input : (ZERO/ZERO)
                    input_tan1 = *((float *)(&POS_ZERO));
                    input_tan2 = *((float *)(&POS_ZERO));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function tmu input (ZERO/ZERO) %f %f : %f \r\n", input_tan2 , input_tan1, tmuVal);
                    break;

                case 2: //input : (ZERO/INFINITY)
                    input_tan1 = *((float *)(&POS_INFINITY));
                    input_tan2 = *((float *)(&POS_ZERO));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function tmu input (ZERO/INFINITY) %f %f : %f \r\n", input_tan2 , input_tan1, tmuVal);
                    break;

                case 3: //input : (INFINITY/ZERO)
                    input_tan1 = *((float *)(&POS_ZERO));
                    input_tan2 = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function tmu input (INFINITY/ZERO) %f %f : %f \r\n", input_tan2 , input_tan1, tmuVal);
                    break;

                case 4: //input : (INFINITY/INFINITY)
                    input_tan1 = *((float *)(&POS_INFINITY));
                    input_tan2 = *((float *)(&POS_INFINITY));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function tmu input (INFINITY/INFINTY) %f %f : %f \r\n", input_tan2 , input_tan1, tmuVal);
                    break;

                case 5: // input : (INFINITY/NORMAL)
                    input_tan1 = *((float *)(&POS_NORM));
                    input_tan2 = *((float *)(&POS_INFINITY));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function tmu input (INFINITY/NORMAL) %f %f : %f \r\n", input_tan2 , input_tan1, tmuVal);
                    break;

                case 6: // input : (NORMAL/ZERO)
                    input_tan1 = *((float *)(&POS_ZERO));
                    input_tan2 = *((float *)(&POS_NORM));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function tmu input (NORMAL/ZERO) %f %f : %f \r\n", input_tan2 , input_tan1, tmuVal);
                    break;

                case 7: // input : (NORMAL/INFINITY)
                    input_tan1 = *((float *)(&POS_INFINITY));
                    input_tan2 = *((float *)(&POS_NORM));
                    tan_input = input_tan2 / (float)input_tan1;
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_atan(tan_input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("atan function tmu input (NORMAL/INFINITY) %f %f : %f \r\n", input_tan2 , input_tan1, tmuVal);
                    break;

                default:
                    DebugP_log("Invalid fxn\r\n");

            }
        }
        break;


    case EXP:
        DebugP_log("TMU EXP \n");
        for(i = 0; i < 7; i++)
        {
            switch(i)
            {
                case 0: //Calculating exponent values using TMU intrinsic function
                    for(j = 0; j < (float) TESTSIZE; j++)
                    {
                        t1 = CycleCounterP_getCount32();
                        tmuVal = ti_tmu_iexp_pu(xvals_tmu[j]);
                        t2 = CycleCounterP_getCount32();
                        t3 = CycleCounterP_getCount32();
                        tmuApprox[j] = tmuVal;
                        fxnTimesTMU[j] = t2 - t1;
                        cycleOverheadTMU[j] = t3 - t2;

                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_iexp_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function tmu input (POS_INFINITY) %f : %f \r\n", input , tmuVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_iexp_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function tmu input (NEG_INFINITY) %f : %f \r\n", input , tmuVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_iexp_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function tmu input (POS_NaN) %f : %f \r\n", input , tmuVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_iexp_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function tmu input (NEG_NaN) %f : %f \r\n", input , tmuVal);
                    break;

                case 5: //input: inf/nan
                    input = *((float *)(&POS_INFINITY));
                    input2 = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_iexp_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function tmu input (INF/POS_NaN) %f : %f \r\n", input , tmuVal);
                    break;

                case 6: //input: -inf/nan
                    input = *((float *)(&NEG_INFINITY));
                    input2 = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_iexp_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("exp function mathlib input (-INF/POS_NaN) %f : %f \r\n", input , tmuVal);
                    break;

                default:
                    DebugP_log("Invalid fxn\r\n");

              }
        }
        break;

    case LOG:
        DebugP_log("TMU LOG \n");
        for(i = 0; i < 7; i++)
        {
            switch(i)
            {
                case 0: //Calculating log values using TMU intrinsic function
                    for(j = 0; j < (float) TESTSIZE; j++)
                    {
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_log_pu(xvals_tmu[j]);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    tmuApprox[j] = tmuVal;
                    fxnTimesTMU[j] = t2 - t1;
                    cycleOverheadTMU[j] = t3 - t2;

                    }
                    break;

                case 1: //input: positive infinity
                    input = *((float *)(&POS_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_log_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function tmu input (POS_INFINITY) %f : %f \r\n", input , tmuVal);
                    break;

                case 2: //input: negative infinity
                    input = *((float *)(&NEG_INFINITY));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_log_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function tmu input (NEG_INFINITY) %f : %f \r\n", input , tmuVal);
                    break;

                case 3: //input: +NaN
                    input = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_log_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function tmu input (POS_NaN) %f : %f \r\n", input , tmuVal);
                    break;

                case 4: //input: -NaN
                    input = *((float *)(&NEG_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_log_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function tmu input (NEG_NaN) %f : %f \r\n", input , tmuVal);
                    break;

                case 5: //input: inf/nan
                    input = *((float *)(&POS_INFINITY));
                    input2 = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_log_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function tmu input (inf/nan) %f %f : %f \r\n", input , input2 , tmuVal);
                    break;

                case 6: //input: -inf/nan
                    input = *((float *)(&NEG_INFINITY));
                    input2 = *((float *)(&POS_NAN));
                    t1 = CycleCounterP_getCount32();
                    tmuVal = ti_tmu_log_pu(input);
                    t2 = CycleCounterP_getCount32();
                    t3 = CycleCounterP_getCount32();
                    DebugP_log("log function tmu input (-inf/nan) %f %f : %f \r\n", input , input2 , tmuVal);
                    break;

                default:
                     DebugP_log("Invalid fxn\r\n");

            }
        }
        break;

    case ATAN2:
        // Calculating ATAN2 value using the results obltained from quadf32 TMU intrinsic function(ratio and quadrant)

        for(i = 0; i < TESTSIZE; i++)
        {
            t1 = CycleCounterP_getCount32();
            tmuVal = ti_tmu_atan2(xvals_tmu[i], yvals_tmu[i]); //TI Arm Trig Lib
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            fxnTimesTMU[i] = t2 - t1;
            cycleOverheadTMU[i] = t3 - t2;
            tmuApprox[i] = tmuVal;
        }
        break;

    case SINCOS:
        for (j = 0; j < TESTSIZE; j++)
        {
            t1 = CycleCounterP_getCount32();
            ti_tmu_sincos(xvals_tmu[j], &sin_val, &cos_val);
            t2 = CycleCounterP_getCount32();
            t3 = CycleCounterP_getCount32();
            tmuApprox[j] = sin_val;
            tmuApprox2[j] = cos_val;
            fxnTimesTMU[j] = t2 - t1;
            cycleOverheadTMU[j] = t3 - t2;
        }
        break;


    default:
        DebugP_log("Invalid fxn\r\n");
    }
}


int32_t printOutput(uint32_t n)
{
    // Calculating the average number of cycles, average number of overheads
    // Accuracy check : This is the max error value for each iteration
    // Calculating the max cycle
    uint32_t i;
    float totalCycles, totalOverhead, maxCycles, maxCyclesMathlib, maxCyclesTMU;
    float avgCycles, avgOverhead;
    float avgCyclesMathlib, avgOverheadMathlib;
    float avgCyclesTMU, avgOverheadTMU;
    float maxErr, maxErr2;
    totalOverhead = 0;
    totalCycles = 0;
    maxCycles = 0.0;
    avgCycles = 0;
    avgOverhead = 0;
    maxCyclesMathlib = 0.0;
    avgCyclesMathlib = 0;
    avgOverheadMathlib = 0;
    maxCyclesTMU = 0.0;
    avgCyclesTMU = 0;
    avgOverheadTMU = 0;
    maxErr = 0;
    maxErr2 = 0;
    int32_t status = SystemP_SUCCESS;

    // CLANG TMU function
    for (i = 0; i < TESTSIZE; i++)
    {
        totalCycles += fxnTimesTMU[i];
        totalOverhead += cycleOverheadTMU[i];

        if(n == 6)
            tmuErr[i] = (float)((float)fabs(gTrigRef[i] - tmuApprox[i]) + (float)fabs(gTrigRef2[i] - tmuApprox2[i]))/2;
        else
            tmuErr[i] = (float)fabs(gTrigRef[i] - tmuApprox[i]);

        if (tmuErr[i] > maxErr2)
            maxErr2 = tmuErr[i];


        if (((float)fxnTimesTMU[i] - (float)cycleOverheadTMU[i]) > maxCyclesTMU)
            maxCyclesTMU = (float)fxnTimesTMU[i] - (float)cycleOverheadTMU[i];

    }

    avgCyclesTMU = (float) (totalCycles)/ (float) TESTSIZE;
    avgOverheadTMU = (float) (totalOverhead / (float) TESTSIZE);

    DebugP_log("\nTMU LIBRARY \r\n");
    DebugP_log("Error: %.10f \r\n", maxErr2);
    DebugP_log("Max Cycles: %f \r\n", maxCyclesTMU);
    DebugP_log("Avg cycles: %f \r\n", avgCyclesTMU - avgOverheadTMU);

    totalOverhead = 0;
    totalCycles = 0;

    // TI ARM SDK Trig function
    if(n!=3 && n!=4)
    {
        for (i = 0; i < TESTSIZE; i++)
        {
            totalCycles += fxnTimes[i];
            totalOverhead += cycleOverhead[i];

            trigErr[i] = (float) fabs(gTrigRef[i] - trigApprox[i]);

            if (trigErr[i] > maxErr)
                maxErr = trigErr[i];

            if (((float)fxnTimes[i] - (float)cycleOverhead[i]) > maxCycles)
                maxCycles = (float)fxnTimes[i] - (float)cycleOverhead[i];

        }
        avgCycles = (float) (totalCycles / (float) TESTSIZE);
        avgOverhead = (float) (totalOverhead / (float) TESTSIZE);

        DebugP_log("\nMCU SDK Trig Math Library \r\n");
        DebugP_log("Error: %.10f \r\n", maxErr);
        DebugP_log("Max Cycles: %f \r\n", maxCycles);
        DebugP_log("Avg cycles: %f \r\n", (avgCycles - avgOverhead));
    }

    totalOverhead = 0;
    totalCycles = 0;

    // CLANG Mathlib function
    for (i = 0; i < TESTSIZE; i++)
    {

        totalCycles += fxnTimesMathlib[i];
        totalOverhead += cycleOverheadMathlib[i];

        if (((float)fxnTimesMathlib[i] - (float)cycleOverheadMathlib[i]) > maxCyclesMathlib)
            maxCyclesMathlib = (float)fxnTimesMathlib[i] - (float)cycleOverheadMathlib[i];

    }

    avgCyclesMathlib = (float) (totalCycles / (float) TESTSIZE);
    avgOverheadMathlib = (float) (totalOverhead / (float) TESTSIZE);

    DebugP_log("\nCLANG MATHLIB \r\n");
    DebugP_log("Max Cycles: %f \r\n", maxCyclesMathlib);
    DebugP_log("Avg cycles: %f \r\n", avgCyclesMathlib - avgOverheadMathlib);


    if ((maxErr > MAX_TRIG_ERR) || (maxErr2 > MAX_TRIG_ERR))
    {
        status = SystemP_FAILURE;
    }
    return status;
}


