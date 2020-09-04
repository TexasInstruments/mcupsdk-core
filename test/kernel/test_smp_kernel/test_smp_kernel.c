/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

/* FreeRTOS includes. */
#include "FreeRTOS_POSIX.h"

/* System headers. */
#include <stdbool.h>
#include <string.h>

/* FreeRTOS+POSIX. */
#include "FreeRTOS_POSIX/pthread.h"
#include "FreeRTOS_POSIX/mqueue.h"
#include "FreeRTOS_POSIX/time.h"
#include "FreeRTOS_POSIX/fcntl.h"
#include "FreeRTOS_POSIX/errno.h"
#include "FreeRTOS_POSIX/semaphore.h"
#include "FreeRTOS_POSIX/unistd.h"

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <unity.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Thread stack size, allocated from FreeRTOS heap */
#define DEFAULT_THREAD_STACK_SIZE   (4*1024U)

#define DEFAULT_THREAD_PRI          (3u)
#define DELAY_US                    50000

#define TIMEOUT                     1000*1000

#define EQUAL_PRIORITY              1
#define THREAD_DELAY                1

#define THREAD_NUM                  configNUM_CORES

#define LOOP_COUNT                  2000

struct thread_info {
    pthread_t tid;
    pthread_attr_t thread_attr;
    int executed;
    int priority;
};

static struct thread_info tinfo[THREAD_NUM];

volatile int t2_count;
volatile int coop_testComplete = -1;

sem_t g_semaphore;
pthread_mutex_t g_mutex;

static int global_cnt;

static void cleanup_resources(void)
{
	for (int i = 0; i < THREAD_NUM; i++)
    {
		tinfo[i].tid = 0;
        tinfo[i].thread_attr = (pthread_attr_t){0};
		tinfo[i].executed = 0;
		tinfo[i].priority = 0;
	}
}

static void * thread_smpCoopThreads(void *args)
{
    t2_count = 0;

    /* This thread simply increments a counter while spinning on
     * the CPU.  The idea is that it will always be iterating
     * faster than the other thread so long as it is fairly
     * scheduled (and it's designed to NOT be fairly schedulable
     * without a separate CPU!), so the main thread can always
     * check its progress.
     */
    while (coop_testComplete != 1)
    {
        ClockP_usleep(DELAY_US);
        t2_count++;
    }

    return NULL;
}

static void spawn_threads(int prio, int thread_num, int equal_prio,
            void *( *startroutine )( void * ))
{
    int i, status = SystemP_SUCCESS;
    struct sched_param threadSchedule;

    /* Spawn threads of priority higher than
     * the previously created thread
     */
    for (i = 0; i < thread_num; i++)
    {
        pthread_attr_init(&tinfo[i].thread_attr);
        pthread_attr_setstacksize(&tinfo[i].thread_attr, DEFAULT_THREAD_STACK_SIZE);

        if (equal_prio)
        {
            tinfo[i].priority = prio;
        }
        else
        {
            /* Increase priority for each thread */
            tinfo[i].priority = prio + 1;
            prio = tinfo[i].priority;
        }

        threadSchedule.sched_priority = prio;
        pthread_attr_setschedparam(&tinfo[i].thread_attr, &threadSchedule);

        status = pthread_create (&tinfo[i].tid, &tinfo[i].thread_attr,
                           startroutine, (void *)i);
        DebugP_assert(status==SystemP_SUCCESS);
    }
}

static void *thread_entry(void *arg)
{
    int thread_num = (int)(arg);
    int count = 0;

    tinfo[thread_num].executed  = 1;

    while (count++ < 5)
    {
        ClockP_usleep(DELAY_US);
    }

    return NULL;
}

static void *inc_global_cnt(void *args)
{
    for (int i = 0; i < LOOP_COUNT; i++)
    {
        sem_wait(&g_semaphore);

		global_cnt++;
		global_cnt--;
		global_cnt++;

        sem_post(&g_semaphore);
	}

    return NULL;
}

/* Multiprocessing is verified by checking if two threads run simultaneously on different cores */
void test_smp_coop_threads(void *args)
{
    int i = 0;
    int32_t status = SystemP_SUCCESS;
    int test_status = 1;

    pthread_t threadID;
    pthread_attr_t threadAttr;
    struct sched_param threadSchedule;

    pthread_attr_init(&threadAttr);
    pthread_attr_setstacksize(&threadAttr, DEFAULT_THREAD_STACK_SIZE);
    threadSchedule.sched_priority = DEFAULT_THREAD_PRI;
    pthread_attr_setschedparam(&threadAttr, &threadSchedule);

    status = pthread_create (&threadID, &threadAttr, thread_smpCoopThreads, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    t2_count = -1;
    while (t2_count == -1)
    {}

    for (i = 0; i < 10; i++)
    {
        /* Wait slightly longer than the other thread so our
         * count will always be lower
         */
        ClockP_usleep(DELAY_US + (DELAY_US / 8));

        if (t2_count <= i)
        {
            test_status = 0;
            break;
        }
    }

    coop_testComplete = 1;

    pthread_join(threadID, NULL);
    TEST_ASSERT_EQUAL_UINT32(1, test_status);

    return;
}

/* Spawn thread equal to the number of cores, so last thread will be pending
 * call takYIELD from main thread. Now all threads must be executed */
void test_yield_threads(void *args)
{
    uint32_t test_status = 0;
	/* Spawn threads equal to the number
	 * of cores, so the last thread would be
	 * pending.
	 */
	spawn_threads(DEFAULT_THREAD_PRI, THREAD_NUM, !EQUAL_PRIORITY,
		      &thread_entry);

	taskYIELD();
	ClockP_usleep(DELAY_US);

	for (int i = 0; i < THREAD_NUM; i++)
    {
		if(tinfo[i].executed != 1)
        {
            test_status++;
        }

	}

	for (int i = 0; i < THREAD_NUM; i++)
    {
		pthread_join(tinfo[i].tid, NULL);
	}

    TEST_ASSERT_EQUAL_INT32(0, test_status);

    cleanup_resources();
}

/* Spawn cooperative thread and call sleep() from main thread. After timeout, all
 * threads has to be scheduled. */
void test_sleep_threads(void *args)
{
    uint32_t test_status = 0;

	spawn_threads(DEFAULT_THREAD_PRI, THREAD_NUM, !EQUAL_PRIORITY,
		      &thread_entry);

	usleep(TIMEOUT);

	for (int i = 0; i < THREAD_NUM; i++)
    {
		if(tinfo[i].executed != 1)
        {
            test_status++;
        }
	}

    for (int i = 0; i < THREAD_NUM; i++)
    {
		pthread_join(tinfo[i].tid, NULL);
	}

    TEST_ASSERT_EQUAL_UINT32(0, test_status);

    cleanup_resources();
}

/* Check if lock unlock APIs are SMP thread safe
 * Create 3 threads to increace global count in different CPUs
 * All threads perform lock and unlock for LOOP_COUNT times
 * No deadlock should occur and total global count shall be 3 * LOOP_COUNT */
void test_inc_concurrency(void *args)
{
    int status = SystemP_SUCCESS;
    struct sched_param threadSchedule;

    pthread_t tid;
    pthread_attr_t thread_attr;

    /* Test synchronisation using semaphores */
    {
        sem_init(&g_semaphore, 0, 1);
        global_cnt = 0;

        /* Initialize first thread */
        pthread_attr_init(&tinfo[0].thread_attr);
        pthread_attr_setstacksize(&tinfo[0].thread_attr, DEFAULT_THREAD_STACK_SIZE);

        threadSchedule.sched_priority = DEFAULT_THREAD_PRI;
        pthread_attr_setschedparam(&tinfo[0].thread_attr, &threadSchedule);

        status = pthread_create (&tinfo[0].tid, &tinfo[0].thread_attr,
                           inc_global_cnt, NULL);
        DebugP_assert(status==SystemP_SUCCESS);

        /* Initialize second thread */
        pthread_attr_init(&tinfo[1].thread_attr);
        pthread_attr_setstacksize(&tinfo[1].thread_attr, DEFAULT_THREAD_STACK_SIZE);

        threadSchedule.sched_priority = DEFAULT_THREAD_PRI;
        pthread_attr_setschedparam(&tinfo[1].thread_attr, &threadSchedule);

        status = pthread_create (&tinfo[1].tid, &tinfo[1].thread_attr,
                           inc_global_cnt, NULL);
        DebugP_assert(status==SystemP_SUCCESS);


        /* Initialize third thread */
        pthread_attr_init(&thread_attr);
        pthread_attr_setstacksize(&thread_attr, DEFAULT_THREAD_STACK_SIZE);

        threadSchedule.sched_priority = DEFAULT_THREAD_PRI;
        pthread_attr_setschedparam(&thread_attr, &threadSchedule);

        status = pthread_create (&tid, &thread_attr,
                           inc_global_cnt, NULL);
        DebugP_assert(status==SystemP_SUCCESS);


        pthread_join(tinfo[0].tid, NULL);
        pthread_join(tinfo[1].tid, NULL);
        pthread_join(tid, NULL);

        TEST_ASSERT_EQUAL_INT32(LOOP_COUNT * 3, global_cnt);

        cleanup_resources();

        sem_destroy(&g_semaphore);
    }
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_smp_kernel_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();

    /* Sleep a bit to guarantee both cores enter an idle task
     * from which the remaining tests can be run correctly */
    ClockP_sleep(1U);

    UNITY_BEGIN();

    RUN_TEST(test_smp_coop_threads, 2449, NULL);
    RUN_TEST(test_yield_threads, 2450, NULL);
    RUN_TEST(test_sleep_threads, 2451, NULL);
    RUN_TEST(test_inc_concurrency, 2452, NULL);

    UNITY_END();
}