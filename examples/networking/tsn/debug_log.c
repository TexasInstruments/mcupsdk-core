/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include "debug_log.h"

Logger_onConsoleOut sDrvConsoleOut;
/* Reason: Printing to the console directly will create a lot of timing issue in gptp.
 * Solution: Writing to a buffer, a log task will print it out a bit later. */
#ifdef TSN_USE_LOG_BUFFER

#if defined(SAFERTOS)
#define LOGGER_STACK_SIZE                         (16U * 1024U)
#define LOGGER_STACK_ALIGN                        LOGGER_STACK_SIZE
#else
#define LOGGER_STACK_SIZE                         (10U * 1024U)
#define LOGGER_STACK_ALIGN                        (32U)
#endif

static CB_THREAD_MUTEX_T gLogMutex;
static CB_THREAD_T hLogTask;

#define INIT_LOG_TASK(priority) \
    if(Logger_startTask(priority) < 0){return -1;}

static uint8_t gLogStackBuf[LOGGER_STACK_SIZE] __attribute__ ((aligned(LOGGER_STACK_ALIGN)));
static uint8_t gLogBuf[4096];
static uint8_t gPrintBuf[4096];

static void *Logger_task(void *arg)
{
    int len;
    DPRINT("%s: started", __func__);

    while(1)
    {
        CB_THREAD_MUTEX_LOCK(&gLogMutex);
        len = strlen((const char*)gLogBuf);
        if (len > 0)
        {
            memcpy(gPrintBuf, gLogBuf, len);
            gLogBuf[0] = 0;
            gPrintBuf[len] = 0;
        }
        CB_THREAD_MUTEX_UNLOCK(&gLogMutex);

        if (len > 0)
        {
            /* The print function will take a long time, we should not
             * call it inside the mutex lock. */
            DPRINT("%s", gPrintBuf);
        }

        CB_USLEEP(10000);
    }
    return NULL;
}

int Logger_logToBuffer(bool flush, const char *str)
{
    int used_len;
    int remain_bufsize;
    int loglen = strlen(str);

    CB_THREAD_MUTEX_LOCK(&gLogMutex);
    used_len = strlen((const char *)gLogBuf);
    remain_bufsize = sizeof(gLogBuf)-used_len;
    if (remain_bufsize > loglen)
    {
        snprintf((char *)&gLogBuf[used_len], remain_bufsize, "%s", str);
    }
    else
    {
        snprintf((char *)&gLogBuf[0], sizeof(gLogBuf), "log ovflow!\n");
    }
    CB_THREAD_MUTEX_UNLOCK(&gLogMutex);

    return 0;
}

static int Logger_startTask(int log_pri)
{
    cb_tsn_thread_attr_t attr;
    int err = 0;

    cb_tsn_thread_attr_init(&attr, log_pri,
                            sizeof(gLogStackBuf), "log_task");
    cb_tsn_thread_attr_set_stackaddr(&attr, &gLogStackBuf[0]);
    if (CB_THREAD_CREATE(&hLogTask, &attr, Logger_task, NULL) < 0)
    {
        DPRINT("Failed to create log task!");
        err = -1;
    }

    return err;
}

#else //!TSN_USE_LOG_BUFFER

#define INIT_LOG_TASK(priority)

int Logger_directLog(bool flush, const char *str)
{
    if (str[0] == 0)
    {
        return 0;
    }
    char *lf = strrchr(str, '\n');
    if (lf)
    {
        *lf = 0;
    }
    sDrvConsoleOut("%s", (char*)str);
    if (flush || (lf && *lf == 0))
    {
        sDrvConsoleOut("\r\n");
    }
    return 0;
}

#endif //!TSN_USE_LOG_BUFFER

int Logger_init(Logger_onConsoleOut consoleOutCb)
{
    if (consoleOutCb)
    {
        sDrvConsoleOut = consoleOutCb;
    }
#ifdef TSN_USE_LOG_BUFFER
    if (CB_THREAD_MUTEX_INIT(&gLogMutex, NULL) < 0)
    {
        DPRINT("Failed to int mutex!");
        return -1;
    }
#endif

    INIT_LOG_TASK(1);
    return 0;
}

void Logger_deInit(void)
{
    #ifdef TSN_USE_LOG_BUFFER
    if (hLogTask != NULL)
    {
        CB_THREAD_JOIN(hLogTask, NULL);
        hLogTask = NULL;
    }
    CB_THREAD_MUTEX_DESTROY(&gLogMutex);
    #endif
}
