/* ======================================================================= */
/* DSP_add32_d.c -- add32 driver code implementation                       */
/*                                                                         */
/* Rev 0.0.1                                                               */
/*                                                                         */
/* Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/  */
/*                                                                         */
/*                                                                         */
/*  Redistribution and use in source and binary forms, with or without     */
/*  modification, are permitted provided that the following conditions     */
/*  are met:                                                               */
/*                                                                         */
/*    Redistributions of source code must retain the above copyright       */
/*    notice, this list of conditions and the following disclaimer.        */
/*                                                                         */
/*    Redistributions in binary form must reproduce the above copyright    */
/*    notice, this list of conditions and the following disclaimer in the  */
/*    documentation and/or other materials provided with the               */
/*    distribution.                                                        */
/*                                                                         */
/*    Neither the name of Texas Instruments Incorporated nor the names of  */
/*    its contributors may be used to endorse or promote products derived  */
/*    from this software without specific prior written permission.        */
/*                                                                         */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
/*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   */
/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
/*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
/*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   */
/*                                                                         */
/* ======================================================================= */

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <ti/dsplib/dsplib.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Interface header files for the natural C and optimized C code */
#include "DSP_add32_cn.h"

#define MAXARRAYSIZE 256

#pragma DATA_ALIGN(x, 8);
#pragma DATA_ALIGN(y, 8);
#pragma DATA_ALIGN(r1, 8);
#pragma DATA_ALIGN(r2, 8);

/* Defines */
#if defined(__TI_EABI__)
#define kernel_size _kernel_size
#endif

extern char kernel_size;

#define FORMULA_SIZE          2
#define FORMULA_DEVIDE        4
#define CYCLE_FORMULA_NX_PT1  64
#define CYCLE_FORMULA_NX_PT2  128
/* inverse of [ 64 1] */
/*            [128 1] */
float form_inv[FORMULA_SIZE][FORMULA_SIZE] =
{{-0.0156,  0.0156},
 { 2.0000, -1.0000}
};
float form_temp  [FORMULA_SIZE];
int   form_cycle [FORMULA_SIZE];
int   form_result[FORMULA_SIZE];

int x[MAXARRAYSIZE];
int y[MAXARRAYSIZE];
int r1[MAXARRAYSIZE];
int r2[MAXARRAYSIZE];

void add32_example_main(void *args)
{
    clock_t t_overhead, time, time_n;
    int i, j, arraySize;
    int fail = 0;
    int form_error = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[DSPLIB] ADD32 Sample Application Started...\r\n");

    /* Initialize timer for clock */
    TSCL= 0,TSCH=0;
    /*  Compute the overhead of calling _itoll(TSCH, TSCL) twice to get timing info. */
    t_overhead = _itoll(TSCH, TSCL);
    t_overhead = _itoll(TSCH, TSCL) - t_overhead;

    /* Test arrays with size from 8 to 2048 elements. */
    /* Fill each array with random data and do add32. */
    for(i = 0; i < MAXARRAYSIZE; i++) {
        x[i]= rand()/2;
        y[i]= rand()/2;
    }

    for(i = 1, arraySize = 8; arraySize <= MAXARRAYSIZE; arraySize *= 2, i++) {

        /* And now measure the performance for an array size of 2048 elements. */
        time_n = _itoll(TSCH, TSCL);
        DSP_add32_cn(x, y, r2, arraySize);
        time_n = _itoll(TSCH, TSCL) - time_n - t_overhead;

        time = _itoll(TSCH, TSCL);
        DSP_add32(x, y, r1, arraySize);
        time = _itoll(TSCH, TSCL) - time - t_overhead;

        /* =================================================================== */
        /* Print timing results                                                */
        /* =================================================================== */
        DebugP_log("DSP_add32\tIter#: %d\t", i);

        /* =================================================================== */
        /* Check the results arrays, and report any failures                   */
        /* =================================================================== */
        if (memcmp(r1, r2, arraySize)) {
            fail++;
            DebugP_log("Result Failure (r_i)");
        }
        else
            DebugP_log("Result Successful (r_i)");

        DebugP_log("\tNX = %d\tnatC: %d\toptC: %d\r\n", arraySize, time_n, time);
        if (arraySize == CYCLE_FORMULA_NX_PT1)
          form_cycle[0] = time;
        if (arraySize == CYCLE_FORMULA_NX_PT2)
          form_cycle[1] = time;
    }

    /* Provide memory information */
#ifdef __TI_COMPILER_VERSION__            // for TI compiler only
    DebugP_log("Memory:  %d bytes\r\n", &kernel_size);
#endif

    /* Provide profiling information */
    for (i = 0; i < FORMULA_SIZE; i++) {
        form_temp[i] = 0;
        for (j = 0; j < FORMULA_SIZE; j++) {
            form_temp[i] += form_inv[i][j] * form_cycle[j];
        }
        if (i != (FORMULA_SIZE-1)) {
            form_result[i] = (int) (form_temp[i] * FORMULA_DEVIDE + 0.5);
            if ((form_result[i] - form_temp[i] * FORMULA_DEVIDE) >  0.1 ||
                (form_result[i] - form_temp[i] * FORMULA_DEVIDE) < -0.1) {
                form_error = 1;
            }
        }
        else {
            form_result[i] = (int) (form_temp[i] + 0.5);
        }
    }

    if (!form_error)
      DebugP_log("Cycles:  %d/%d*Nx + %d \r\n", form_result[0], FORMULA_DEVIDE, form_result[1]);
    else
      DebugP_log("Cycles Formula Not Available\r\n");

    if(fail == 0)
    {
        DebugP_log("[DSPLIB] ADD32 Sample Application Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
}

/* ======================================================================== */
/*  End of file:  DSP_add32_d.c                                             */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2011 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */

