/**
 * benchmark_stat.h
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/* =================================================================================
File name:       benchmark_stat.h
===================================================================================*/


#ifndef _BENCHMARK_STAT_H_
#define _BENCHMARK_STAT_H_

#include <stdint.h>
#include <stdbool.h>
#include <kernel/dpl/CycleCounterP.h>

/* define ENABLE_IPC_RPMSG_CHAR to enable the IPC RPMSG_char between A53 and R5
 * In the case of R5 only test, keep this undefined
 */
#define ENABLE_IPC_RPMSG_CHAR

/*
 * Application ID
 */
typedef enum {
    APP_ID_CFFT            = 1,
    APP_ID_FIR             = 2,
    APP_ID_FOC             = 3,
    APP_ID_PID             = 4,
    APP_ID_ADC             = 5,
    APP_ID_STREAM          = 5
} App_Id;

typedef struct
{
    volatile uint64_t isrCount;     /* timer interrupt counter */
    uint64_t isrCountOld;           /* previous timer interrupt counter */
    uint32_t max;                   /* timer interrupt latency max */
    uint32_t avg;                   /* timer interrupt latency average */
    uint64_t total;                 /* timer interrupt latency total */
    uint32_t initSkipCount;         /* number of initial iterations to skip the benchmark */
} App_TimerStats;



void App_ipcInit();
void App_ipcRecv(void *buf, uint16_t *bufSize, uint16_t maxSize);
void App_ipcSend(void *buf, uint16_t bufSize);

void App_timerResetStats();
void App_timerSetHz(uint32_t freq);
uint32_t App_timerHasExpired();

void App_statsInit(uint32_t appId);
void App_statsUpdate(uint32_t start, uint32_t end);
void App_statsUpdateUI();
uint32_t App_statsGetOptionValue();

#endif /* _BENCHMARK_STAT_H_ */
