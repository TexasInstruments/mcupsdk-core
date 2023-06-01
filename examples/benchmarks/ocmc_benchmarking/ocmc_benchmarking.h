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


#ifndef __OCMC_BENCHMARKING_H__
#define __OCMC_BENCHMARKING_H__


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/QueueP.h>
#include <kernel/dpl/SemaphoreP.h>

#define NUM_TASK 16
#define NUM_TEST 10
#define BUF_SIZE 4096
//#define MEM_CPY_OPER 2400000
#define MEM_CPY_OPER 500 // min > 10 msec
#define BUFFER_IN_USE 2
#define TASK_STACK_SIZE 0x1000



typedef struct
{
    QueueP_Elem elem;
    uint32_t task_call_number;
}msg;


#define CMPLX_FNCT_1 \
                denominator = 2 / buf_ocmc[33] + x1; \
                sinx = sinx * 1.23454 - buf_ocmc[34]; \
                buf_ocmc[55] = buf_ocmc[55] / buf_ocmc[43]; \
                buf_ocmc[33] = buf_ocmc[38] / buf_ocmc[23]; \
                buf_ocmc[44] = buf_ocmc[26] / buf_ocmc[28] / buf_ocmc[56] * 1.235; \
                denominator = 2 * m * (2 * m + 1);  \
                x1 = -x1 * n * n / denominator;  \
                sinx = sinx + x1;  \
                m =  + 1;  \
                denominator = 2 / buf_ocmc[33] + x1; \
                sinx = sinx * 1.23454 - buf_ocmc[34]; \
                buf_ocmc[55] = buf_ocmc[55] / buf_ocmc[43]; \
                buf_ocmc[33] = buf_ocmc[38] / buf_ocmc[23]; \
                buf_ocmc[44] = buf_ocmc[26] / buf_ocmc[28] / buf_ocmc[56] * 1.235; \
                denominator = 2 * m * (2 * m + 1);  \
                x1 = -x1 * n * n / denominator;  \
                sinx = sinx + x1;  \
                m =  + 1;  \
                denominator = 2 / buf_ocmc[33] + x1; \
                sinx = sinx * 1.23454 - buf_ocmc[34]; \
                buf_ocmc[55] = buf_ocmc[55] / buf_ocmc[43]; \
                buf_ocmc[33] = buf_ocmc[38] / buf_ocmc[23]; \
                buf_ocmc[44] = buf_ocmc[26] / buf_ocmc[28] / buf_ocmc[56] * 1.235; \
                denominator = 2 * m * (2 * m + 1);  \
                x1 = -x1 * n * n / denominator;  \
                sinx = sinx + x1;  \
                m =  + 1;  \
                denominator = 2 / buf_ocmc[33] + x1; \
                sinx = sinx * 1.23454 - buf_ocmc[34]; \

#define CMPLX_FNCT_2 \
                buf_ocmc[53] = buf_ocmc[53] + 1; \
                buf_ocmc[54] = buf_ocmc[54] + 1; \
                buf_ocmc[55] = buf_ocmc[55] + 1; \
                buf_ocmc[56] = buf_ocmc[56] + 1; \
                buf_ocmc[57] = buf_ocmc[57] + 1; \
                buf_ocmc[58] = buf_ocmc[58] + 1; \
                x1 = -x1 * n * n / denominator;  \
                sinx = sinx + x1;  \
                m =  + 1;  \

#define CMPLX_FNCT_3 \
                buf_ocmc[53] = buf_ocmc[53] + 1; \
                buf_ocmc[54] = buf_ocmc[54] + 1; \
                buf_ocmc[55] = buf_ocmc[55] + 1; \
                buf_ocmc[56] = buf_ocmc[56] + 1; \
                buf_ocmc[57] = buf_ocmc[57] + 1; \
                buf_ocmc[58] = buf_ocmc[58] + 1; \
                denominator = 2 * m * (2 * m + 1);  \
                x1 = -x1 * n * n / denominator;  \
                sinx = sinx + x1;  \
                m =  + 1;  \

#define CMPLX_FNCT_4 \
                buf_ocmc[43] = buf_ocmc[53] + 1; \
                buf_ocmc[44] = buf_ocmc[54] + 1; \
                buf_ocmc[45] = buf_ocmc[55] + 1; \
                buf_ocmc[46] = buf_ocmc[56] + 1; \
                buf_ocmc[47] = buf_ocmc[57] + 1; \
                buf_ocmc[48] = buf_ocmc[58] + 1; \
                denominator = 2 * m * (2 * m + 1);  \
                x1 = -x1 * n * n / denominator;  \
                sinx = sinx + x1;  \
                m =  + 1;  \


#define ADD_FN_1() do { asm(" NOP "); } while(0)
#define SUB_FN_1() do { asm(" NOP "); } while(0)

#define DUMB_FN_1()  do {\
    ADD_FN_1(); \
    SUB_FN_1(); \
} while (0)

#define DUMB_FN_2() do {\
    DUMB_FN_1(); \
    DUMB_FN_1(); \
} while (0)

#define DUMB_FN_4() do {\
    DUMB_FN_2(); \
    DUMB_FN_2(); \
} while (0)

#define DUMB_FN_8() do {\
    DUMB_FN_4(); \
    DUMB_FN_4(); \
} while (0)

#define DUMB_FN_128() do {\
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
    DUMB_FN_8(); \
} while (0)

#define DUMB_FN_1024() do {\
    DUMB_FN_128(); \
    DUMB_FN_128(); \
    DUMB_FN_128(); \
    DUMB_FN_128(); \
    DUMB_FN_128(); \
    DUMB_FN_128(); \
    DUMB_FN_128(); \
    DUMB_FN_128(); \
} while (0)

#define DUMB_FN_4096() do{\
    DUMB_FN_1024(); \
    DUMB_FN_1024(); \
    DUMB_FN_1024(); \
    DUMB_FN_1024(); \
} while (0)

/* The fucntion definition for all the tasks.  */
#define TSKFN \
    msg* rp; \
    int i,j, arr_number,m; \
    while(1) \
    { \
        SemaphoreP_pend(&gSemaphorePHandle[(uint32_t)a], SystemP_WAIT_FOREVER); \
        while (QueueP_EMPTY != QueueP_isEmpty(myQ[(uint32_t)a])) \
        { \
            rp = QueueP_get(myQ[(uint32_t)a]); \
            for (i = 0; i < iter; ++i) \
            { \
                arr_number = get_rand() % BUFFER_IN_USE; \
                for (j = 0; j < memcopy_size; ++j) \
                { \
                    buf_ocmc[j] = buf[arr_number][j]; \
                } \
                /*sin calculation*/ \
                float denominator, sinx;  \
                float n = 30;        \
                n = n * (3.142 / 180.0);   \
                float x1 = n;  \
                sinx = n;           \
                m = 1;  \
                denominator = 2 * m * (2 * m + 1);  \
                x1 = -x1 * n * n / denominator;  \
                sinx = sinx + x1;  \
                m =  + 1;  \
                buf_ocmc[23] = buf_ocmc[23] + 1; \
                buf_ocmc[24] = buf_ocmc[24] + 1; \
                buf_ocmc[25] = buf_ocmc[25] + 1; \
                buf_ocmc[26] = buf_ocmc[26] + 1; \
                buf_ocmc[27] = buf_ocmc[27] + 1; \
                buf_ocmc[28] = buf_ocmc[28] + 1; \
                buf_ocmc[29] = buf_ocmc[29] + 1; \
                buf_ocmc[30] = buf_ocmc[30] + 1; \
                buf_ocmc[31] = buf_ocmc[31] + 1; \
                buf_ocmc[32] = buf_ocmc[32] + 1; \
                buf_ocmc[33] = buf_ocmc[33] + 1; \
                buf_ocmc[34] = buf_ocmc[34] + 1; \
                buf_ocmc[35] = buf_ocmc[35] + 1; \
                buf_ocmc[36] = buf_ocmc[36] + 1; \
                buf_ocmc[37] = buf_ocmc[37] + 1; \
                buf_ocmc[38] = buf_ocmc[38] + 1; \
                buf_ocmc[39] = buf_ocmc[39] + 1; \
                buf_ocmc[40] = buf_ocmc[40] + 1; \
                buf_ocmc[41] = buf_ocmc[41] + 1; \
                buf_ocmc[42] = buf_ocmc[42] + 1; \
                buf_ocmc[43] = buf_ocmc[43] + 1; \
                buf_ocmc[44] = buf_ocmc[44] + 1; \
                buf_ocmc[45] = buf_ocmc[45] + 1; \
                buf_ocmc[46] = buf_ocmc[46] + 1; \
                buf_ocmc[47] = buf_ocmc[47] + 1; \
                buf_ocmc[48] = buf_ocmc[48] + 1; \
                buf_ocmc[49] = buf_ocmc[49] + 1; \
                buf_ocmc[50] = buf_ocmc[50] + 1; \
                buf_ocmc[51] = buf_ocmc[51] + 1; \
                buf_ocmc[52] = buf_ocmc[52] + 1; \
                CMPLX_FNCT_3 \
                CMPLX_FNCT_3 \
                CMPLX_FNCT_3 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_3 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_4 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_3 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_3 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_3 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_4 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_4 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_2 \
                CMPLX_FNCT_1 \
                CMPLX_FNCT_3 \
                CMPLX_FNCT_3 \
                DUMB_FN_1024(); \
            } \
            rp->task_call_number = rp->task_call_number + 1; \
        } \
        SemaphoreP_post(&gSemaphorePHandle[(uint32_t)a]); \
        TaskP_yield(); \
    }



#endif
