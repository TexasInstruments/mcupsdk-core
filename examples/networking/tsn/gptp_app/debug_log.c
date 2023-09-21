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

on_console_out s_drv_console_out;
static CB_THREAD_MUTEX_T g_log_mutex;
static CB_THREAD_T g_logtask_handle;
/* Reason: Printing to the console directly will create a lot of timing issue in gptp.
 * Solution: Writing to a buffer, a log task will print it out a bit later. */
#ifdef TSN_USE_LOG_BUFFER
#define INIT_LOG_TASK(priority) \
    if(start_logtask(priority) < 0){return -1;}

static uint8_t s_log_stack_buf[1024] __attribute__ ((aligned(32)));
static uint8_t s_log_buf[4096];
static uint8_t s_print_buf[4096];

static void *log_task(void *arg)
{
    int len;
    DPRINT("%s: started", __func__);

    while(1)
    {
        CB_THREAD_MUTEX_LOCK(&g_log_mutex);
        len = strlen((const char*)s_log_buf);
        if (len > 0)
        {
            memcpy(s_print_buf, s_log_buf, len);
            s_log_buf[0] = 0;
            s_print_buf[len] = 0;
        }
        CB_THREAD_MUTEX_UNLOCK(&g_log_mutex);

        if (len > 0)
        {
            /* The print function will take a long time, we should not
             * call it inside the mutex lock. */
            DPRINT("%s", s_print_buf);
        }

        CB_USLEEP(10000);
    }
    return NULL;
}

int log_to_buffer(bool flush, const char *str)
{
    int used_len;
    int remain_bufsize;
    int loglen = strlen(str);

    CB_THREAD_MUTEX_LOCK(&g_log_mutex);
    used_len = strlen((const char *)s_log_buf);
    remain_bufsize = sizeof(s_log_buf)-used_len;
    if (remain_bufsize > loglen)
    {
        snprintf((char *)&s_log_buf[used_len], remain_bufsize, "%s", str);
    }
    else
    {
        snprintf((char *)&s_log_buf[0], sizeof(s_log_buf), "log ovflow!\n");
    }
    CB_THREAD_MUTEX_UNLOCK(&g_log_mutex);

    return 0;
}

static int start_logtask(int log_pri)
{
    cb_tsn_thread_attr_t attr;
    int err = 0;

    if (CB_THREAD_MUTEX_INIT(&g_log_mutex, NULL) < 0)
    {
        DPRINT("Failed to int mutex!");
        err = -1;
    }
    else
    {
        cb_tsn_thread_attr_init(&attr, log_pri,
                                sizeof(s_log_stack_buf), "log_task");
        cb_tsn_thread_attr_set_stackaddr(&attr, &s_log_stack_buf[0]);
        if (CB_THREAD_CREATE(&g_logtask_handle, &attr, log_task, NULL) < 0)
        {
            DPRINT("Failed to create log task!");
            if (g_log_mutex != NULL)
            {
                CB_THREAD_MUTEX_DESTROY(&g_log_mutex);
                g_log_mutex = NULL;
            }
            if (g_logtask_handle != NULL)
            {
                CB_THREAD_JOIN(g_logtask_handle, NULL);
                g_logtask_handle = NULL;
            }
            err = -1;
        }
        else
        {
            err = 0;
        }
    }
    return err;
}

#else //!TSN_USE_LOG_BUFFER

#define INIT_LOG_TASK(priority)

int direct_log(bool flush, const char *str)
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
    s_drv_console_out("%s", (char*)str);
    if (flush || (lf && *lf == 0))
    {
        s_drv_console_out("\r\n");
    }
    return 0;
}

#endif //!TSN_USE_LOG_BUFFER

int debug_log_init(on_console_out console_out)
{
    s_drv_console_out = console_out;
    INIT_LOG_TASK(1);
    return 0;
}

void debug_log_deinit(void)
{
    if (g_logtask_handle != NULL)
    {
        CB_THREAD_JOIN(g_logtask_handle, NULL);
        g_logtask_handle = NULL;
    }
    CB_THREAD_MUTEX_DESTROY(&g_log_mutex);
}
