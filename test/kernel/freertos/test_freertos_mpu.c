/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/TaskP.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <unity.h>

/*
* IMPORTANT NOTES:
*
* Alignment of stack to stack size is required with MPU port to add stack as an MPU region.
*
* Task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest
* For this example any valid task priority can be set.
*
* See FreeRTOSConfig.h for configMAX_PRIORITIES and StackType_t type.
* FreeRTOSConfig.h can be found under kernel/freertos/config/${device}/${cpu}/
*
* In this example,
* We create task's, semaphore's, ISR's and stack for the tasks using static allocation.
* We dont need to delete these semaphore's since static allocation is used.
*
* One MUST not return out of a FreeRTOS task instead one MUST call vTaskDelete instead.
*/

/**
 * Helper macros for various attributes 
 */
#define MPU_UT_DATA         __attribute__((section(".data.freertos_mpu_ut")))
#define MPU_UT_TEXT_USER1   __attribute__((section(".text.freertos_mpu_ut.user1")))
#define MPU_UT_TEXT_USER2   __attribute__((section(".text.freertos_mpu_ut.user2")))
#define ALIGN(size)         __attribute__((aligned(((size)))))

/** 
 * Defines for tasks and interrupt related params 
 */
#define NUM_USER_TASKS      (2U)

#define USER1_TASK_PRI      (2U)
#define USER2_TASK_PRI      (3U)
#define SYS_TASK_PRI        (4U)

#define USER1_TASK_SIZE     (1024U*4U)
#define USER2_TASK_SIZE     (1024U*4U)
#define SYS_TASK_SIZE       (1024U*4U)

#define INT1_NUM    (16U)
#define INT2_NUM    (17U)

/**
 * Set Interrupt priorities based on FreeRTOS configuration.
 *
 * ISR safe FreeRTOS API functions must only be called from interrupts that have been assigned
 * a priority at or below configMAX_SYSCALL_INTERRUPT_PRIORITY.
 *
 * Numerically low interrupt priority numbers represent logically high interrupt priorities, therefore
 * the priority of the interrupt must be set to a value equal to or numerically higher than
 * configMAX_SYSCALL_INTERRUPT_PRIORITY. If configMAX_SYSCALL_INTERRUPT_PRIORITY is not defined,
 * default priorities are assigned.
 *
 * Else there will be assertion failures that occur when an ISR with a priority above
 * configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API function.
 */
#ifdef configMAX_SYSCALL_INTERRUPT_PRIORITY
#define INT1_PRIORITY   (configMAX_SYSCALL_INTERRUPT_PRIORITY)
#define INT2_PRIORITY   (configMAX_SYSCALL_INTERRUPT_PRIORITY + 1u)
#else
#define INT1_PRIORITY   (3u)
#define INT2_PRIORITY   (4u)
#endif

/** 
 * Defines for shared memory 
 */
#define SHARED_MEM_REGION_BYTES     (32U)

#if(((SHARED_MEM_REGION_BYTES % 2U) != 0U) || (SHARED_MEM_REGION_BYTES < 32U))
#error SHARED_MEM_REGION_BYTES must be a power of 2 that is larger than 32
#endif

#define SHARED_MEM_REGION_NUM           (8U)
#define SHARED_MEM_REGION_NUM_USER1     (4U) /* User 1 is given read access only for first 4 regions */
#define SHARED_MEM_REGION_NUM_USER2     (5U) /* User 2 is given read-write access only for first 6 regions */

#define SHARED_MEM_REGION_USER1_INVALID_START   (SHARED_MEM_REGION_NUM_USER1)
#define SHARED_MEM_REGION_USER2_INVALID_START   (SHARED_MEM_REGION_NUM_USER2)

/** 
 * Defines for invalid function access 
 */
#define INVALID_FXN_ACCESS_NUM          (2U) /* 1x privileged function + 1x other user's function */

/** 
 * Defines for fault tracker 
 */
#define FAULT_TRACKER_SHARED_MEM_USER1_START    (0U)
#define FAULT_TRACKER_SHARED_MEM_USER2_START    (FAULT_TRACKER_SHARED_MEM_USER1_START + SHARED_MEM_REGION_NUM)
#define FAULT_TRACKER_SHARED_MEM_MAX            (FAULT_TRACKER_SHARED_MEM_USER2_START + SHARED_MEM_REGION_NUM)

#define FAULT_TRACKER_FXN_USER1_START           (FAULT_TRACKER_SHARED_MEM_MAX)
#define FAULT_TRACKER_FXN_USER2_START           (FAULT_TRACKER_FXN_USER1_START + INVALID_FXN_ACCESS_NUM)
#define FAULT_TRACKER_FXN_MAX                   (FAULT_TRACKER_FXN_USER2_START + INVALID_FXN_ACCESS_NUM)

#define FAULT_TRACKER_DATA_ABORT_START          (FAULT_TRACKER_SHARED_MEM_USER1_START)
#define FAULT_TRACKER_DATA_ABORT_MAX            (FAULT_TRACKER_SHARED_MEM_MAX)

#define FAULT_TRACKER_PREFETCH_ABORT_START      (FAULT_TRACKER_FXN_USER1_START)
#define FAULT_TRACKER_PREFETCH_ABORT_MAX        (FAULT_TRACKER_FXN_MAX)          

#define FAULT_TRACKER_MAX                       (FAULT_TRACKER_PREFETCH_ABORT_MAX)                        

/** User tasks cannot use unity TEST_ASSERT since unity RUN_TEST is performed by sys task.
 * Hence add following util macros to handle test failure asserts from user tasks */
#define SET_FAULT_TRACKER(idx)          gTestUserObj.faultTracker[(idx)] = 1U;
#define SET_FAULT_TRACKER_DOUBLE(idx)   gTestUserObj.faultTracker[(idx)] = 2U;

#define TEST_ASSERT_FROM_USER(expected, actual, userIdx) \
    if ((expected) != (actual)) { \
        gTestUserObj.userErrorCnt[(userIdx)]++; \
    }
#define TEST_ASSERT_FROM_USER1(expected, actual)    TEST_ASSERT_FROM_USER(expected, actual, 0U);
#define TEST_ASSERT_FROM_USER2(expected, actual)    TEST_ASSERT_FROM_USER(expected, actual, 1U);

#define TEST_ASSERT_FAULT_TRACKER_FROM_USER1(idx)   TEST_ASSERT_FROM_USER1(0U, gTestUserObj.faultTracker[(idx)]);
#define TEST_ASSERT_FAULT_TRACKER_FROM_USER2(idx)   TEST_ASSERT_FROM_USER2(0U, gTestUserObj.faultTracker[(idx)]);

#define TEST_ASSERT_USER1_ERROR_CHECK()             TEST_ASSERT_EQUAL_UINT32(0U, gTestUserObj.userErrorCnt[0U]);
#define TEST_ASSERT_USER2_ERROR_CHECK()             TEST_ASSERT_EQUAL_UINT32(0U, gTestUserObj.userErrorCnt[1U]);

#define RESET_USER1_ERROR_CNT()     gTestUserObj.userErrorCnt[0U] = 0U;
#define RESET_USER2_ERROR_CNT()     gTestUserObj.userErrorCnt[1U] = 0U;

/** 
 * Other defines and macros 
 */
#define NUM_LOOP_PER_BYTE       (4U)
#define TOTAL_NUM_LOOP          (NUM_LOOP_PER_BYTE * SHARED_MEM_REGION_BYTES)

#define GET_BYTE_IDX(loopCnt)   ((loopCnt) % SHARED_MEM_REGION_BYTES)     


/** 
 * Structure definitions 
 */
typedef struct test_freertos_mpu_obj_
{
    HwiP_Object hwi1Obj;
    HwiP_Object hwi2Obj;
    
    TaskP_Object user1TaskObj;
    TaskP_Object user2TaskObj;
    TaskP_Object sysTaskObj;
    
    StaticSemaphore_t user1SemObj;
    StaticSemaphore_t user2SemObj;
    StaticSemaphore_t sysSemObj;
    StaticSemaphore_t mpuSyncSemObj;

    SemaphoreHandle_t mpuSyncSem;

    volatile uint32_t sharedMemIsrCnt;
} test_freertos_mpu_obj;

typedef struct test_freertos_mpu_user_obj_
{
    SemaphoreHandle_t user1Sem;
    SemaphoreHandle_t user2Sem;
    SemaphoreHandle_t sysSem;

    volatile uint8_t  faultTracker[FAULT_TRACKER_MAX];
    volatile uint32_t userErrorCnt[NUM_USER_TASKS];
    
} test_freertos_mpu_user_obj;

/** 
 * Task Stacks
 */
MPU_UT_DATA ALIGN(USER1_TASK_SIZE) uint8_t gUser1TaskStack[USER1_TASK_SIZE];
MPU_UT_DATA ALIGN(USER2_TASK_SIZE) uint8_t gUser2TaskStack[USER2_TASK_SIZE];

PRIVILEGED_DATA ALIGN(USER2_TASK_SIZE) uint8_t gSysTaskStack[USER2_TASK_SIZE];

/** 
 * Global Variables
 */
PRIVILEGED_DATA test_freertos_mpu_obj gTestObj;

MPU_UT_DATA ALIGN(128U) test_freertos_mpu_user_obj gTestUserObj;

MPU_UT_DATA ALIGN(SHARED_MEM_REGION_BYTES)  \
volatile uint8_t gSharedMem[SHARED_MEM_REGION_NUM][SHARED_MEM_REGION_BYTES];

/* Forward declaration for user1_main to perform invalid function call */
MPU_UT_TEXT_USER2 void user2_main(void *args);

/** 
 * Shared memory related privileged util functions
 */
PRIVILEGED_FUNCTION static void shared_mem_reset()
{
    memset((void*)gSharedMem, 0, sizeof(gSharedMem));
    gTestObj.sharedMemIsrCnt = 0U;
}

PRIVILEGED_FUNCTION static void shared_mem_read_write(uint8_t loopCnt)
{
    for (uint8_t regionIdx = 0U; regionIdx < SHARED_MEM_REGION_NUM; regionIdx++)
    {
        gSharedMem[regionIdx][GET_BYTE_IDX(loopCnt)]++;
    }
}

PRIVILEGED_FUNCTION static void shared_mem_check_full(uint8_t expectedVal)
{
    TEST_ASSERT_EACH_EQUAL_UINT8(expectedVal, gSharedMem, SHARED_MEM_REGION_NUM * SHARED_MEM_REGION_BYTES);
}

PRIVILEGED_FUNCTION static void shared_mem_check_user2_region(uint8_t expectedVal)
{
    TEST_ASSERT_EACH_EQUAL_UINT8(expectedVal, gSharedMem, 
                                    SHARED_MEM_REGION_NUM_USER2 * SHARED_MEM_REGION_BYTES);
}

PRIVILEGED_FUNCTION static void shared_mem_check_sys_region(uint8_t expectedVal)
{
    volatile uint8_t *startAddrPtr = &gSharedMem[SHARED_MEM_REGION_NUM_USER2][0U];
    uint32_t numElements = (SHARED_MEM_REGION_NUM - SHARED_MEM_REGION_NUM_USER2) * SHARED_MEM_REGION_BYTES;
    
    TEST_ASSERT_EACH_EQUAL_UINT8(expectedVal, startAddrPtr, numElements);
}

PRIVILEGED_FUNCTION static void shared_mem_isr(void *arg)
{
    BaseType_t        doTaskSwitch = 0;
    SemaphoreHandle_t semHndl      = (SemaphoreHandle_t)arg;

    shared_mem_read_write(gTestObj.sharedMemIsrCnt++);
    xSemaphoreGiveFromISR(semHndl, &doTaskSwitch);
    portYIELD_FROM_ISR(doTaskSwitch);
}

PRIVILEGED_FUNCTION void register_shared_mem_isr(void *isr1Arg, void *isr2Arg)
{
    HwiP_Params hwiParams;

    HwiP_Params_init(&hwiParams);
    hwiParams.callback = shared_mem_isr;

    hwiParams.args     = isr1Arg;             
    hwiParams.intNum   = INT1_NUM;
    hwiParams.priority = INT1_PRIORITY;
    HwiP_construct(&gTestObj.hwi1Obj, &hwiParams);

    if(isr2Arg != NULL) {
        hwiParams.args     = isr2Arg;             
        hwiParams.intNum   = INT2_NUM;
        hwiParams.priority = INT2_PRIORITY;
        HwiP_construct(&gTestObj.hwi2Obj, &hwiParams);
    }
}

/** 
 * Shared memory related user util functions
 */
MPU_UT_TEXT_USER1 static void user1_valid_shared_mem_read(uint8_t loopCnt)
{
    for (uint8_t regionIdx = 0U; regionIdx < SHARED_MEM_REGION_NUM_USER1; regionIdx++)
    {
        volatile uint8_t val = gSharedMem[regionIdx][GET_BYTE_IDX(loopCnt)];
        (void) val;
    }
}

MPU_UT_TEXT_USER2 static void user2_valid_shared_mem_read_write(uint8_t loopCnt)
{
    for (uint8_t regionIdx = 0U; regionIdx < SHARED_MEM_REGION_NUM_USER2; regionIdx++)
    {
        gSharedMem[regionIdx][GET_BYTE_IDX(loopCnt)]++;
    }
}

MPU_UT_TEXT_USER1 static void user1_invalid_shared_mem_write(uint8_t loopCnt)
{
    uint8_t byteIdx = GET_BYTE_IDX(loopCnt);
    /** Use volatile with loop counter,
     * since otherwise with release mode optimization compiler would unwrap loops 
     * and this may result in multiple data aborts */
    volatile uint8_t regionIdx;

    /* Perform write to memory regions with read-only access */
    for (regionIdx = 0U; regionIdx < SHARED_MEM_REGION_NUM_USER1; regionIdx++)
    {
        SET_FAULT_TRACKER(FAULT_TRACKER_SHARED_MEM_USER1_START + regionIdx);
        gSharedMem[regionIdx][byteIdx] = 0xFF; /* This should cause data abort */
        TEST_ASSERT_FAULT_TRACKER_FROM_USER1(FAULT_TRACKER_SHARED_MEM_USER1_START + regionIdx);
    }
}

MPU_UT_TEXT_USER1 static void user1_invalid_shared_mem_read(uint8_t loopCnt)
{
    /** Use volatile with loop counter,
     * since otherwise with release mode optimization compiler would unwrap loops 
     * and this may result in multiple data aborts */
    volatile uint8_t regionIdx, val;

    /* Perform read to unmapped memory regions */
    for (regionIdx = SHARED_MEM_REGION_USER1_INVALID_START; regionIdx < SHARED_MEM_REGION_NUM; regionIdx++)  
    {
        SET_FAULT_TRACKER(FAULT_TRACKER_SHARED_MEM_USER1_START + regionIdx);
        val = gSharedMem[regionIdx][GET_BYTE_IDX(loopCnt)]; /* This should cause data abort */
        TEST_ASSERT_FAULT_TRACKER_FROM_USER1(FAULT_TRACKER_SHARED_MEM_USER1_START + regionIdx);
        
        (void) val;
    }
}

MPU_UT_TEXT_USER2 static void user2_invalid_shared_mem_read_write(uint8_t loopCnt)
{
    /** Use volatile with loop counter,
     * since otherwise with release mode optimization compiler would unwrap loops 
     * and this may result in multiple data aborts */
    volatile uint8_t regionIdx;

    /* Perform read-modify-write to unmapped memory regions */
    for (regionIdx = SHARED_MEM_REGION_USER2_INVALID_START; regionIdx < SHARED_MEM_REGION_NUM; regionIdx++)
    {
        /* This should cause data abort for read as well as for write (2x)  */
        SET_FAULT_TRACKER_DOUBLE(FAULT_TRACKER_SHARED_MEM_USER2_START + regionIdx);
        gSharedMem[regionIdx][GET_BYTE_IDX(loopCnt)]++;
        TEST_ASSERT_FAULT_TRACKER_FROM_USER2(FAULT_TRACKER_SHARED_MEM_USER2_START + regionIdx);
    }
}

/** 
 * Unity test functions
 * 
 * - These will be run by the sys task
 */
PRIVILEGED_FUNCTION void test_validMemAccessWithTaskSysSwitch(void *args)
{
    shared_mem_reset();
    for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
    {
        xSemaphoreGive(gTestUserObj.user1Sem);              /* Wakeup user 1 */
        /* User 1 performs valid read operations and signal user 2 */
        /* User 2 performs valid read-modify-write operation (increment) and signal sys (current task) */
        xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY); /* Wait for user 2 to signal */
        shared_mem_read_write(loopCnt);                     /* This increments the value in shared memory */
    }
    shared_mem_check_user2_region(2 * NUM_LOOP_PER_BYTE);   /* Shared memory value updated by user 2 + sys */
    shared_mem_check_sys_region(NUM_LOOP_PER_BYTE);         /* Shared memory value updated by sys only */
}

PRIVILEGED_FUNCTION void test_invalidMemAccessWithTaskSysSwitch(void *args)
{
    shared_mem_reset();
    RESET_USER1_ERROR_CNT();
    RESET_USER2_ERROR_CNT();
    for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
    {
        xSemaphoreGive(gTestUserObj.user1Sem);              /* Wakeup user 1 */
        /* User 1 performs invalid write & invalid read operations and signal user 2 */
        /* User 2 performs invalid read-modify-write operation and signal sys (current task) */
        xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY); /* Wait for user 2 to signal */
        shared_mem_read_write(loopCnt);                     /* This increments the value in shared memory */
    }
    TEST_ASSERT_USER1_ERROR_CHECK();                        /* Check for expected data aborts in user 1 */
    TEST_ASSERT_USER2_ERROR_CHECK();                        /* Check for expected data aborts in user 2 */
    shared_mem_check_full(NUM_LOOP_PER_BYTE);               /* Shared memory value updated by sys only */
}

PRIVILEGED_FUNCTION void test_validMemAccessWithTaskIsrSwitch(void *args)
{
    shared_mem_reset();
    xSemaphoreGive(gTestUserObj.user1Sem);                /* Wakeup user 1 */
    /* User 1 performs valid read operations and trigger ISR 1 */
    /* ISR 1 performs valid read-modify-write operations and signal user 1 */
    /* `User 1 -> ISR 1` routine repeats for TOTAL_NUM_LOOP times */
    xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY);   /* Wait for user 1 to signal */
    shared_mem_check_full(NUM_LOOP_PER_BYTE);             /* Shared memory value updated by ISR 1 only */
}

PRIVILEGED_FUNCTION void test_invalidMemAccessWithTaskIsrSwitch(void *args)
{
    shared_mem_reset();
    RESET_USER1_ERROR_CNT();
    xSemaphoreGive(gTestUserObj.user1Sem);                /* Wakeup user 1 */
    /* User 1 performs invalid write & invalid read operations and trigger ISR 1 */
    /* ISR 1 performs valid read-modify-write operations (increment) and signal user 1 */
    /* `User 1 -> ISR 1` routine repeats for TOTAL_NUM_LOOP times */
    xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY);   /* Wait for user 1 to signal */
    TEST_ASSERT_USER1_ERROR_CHECK();                      /* Check for expected data aborts in user 1 */
    shared_mem_check_full(NUM_LOOP_PER_BYTE);             /* Shared memory value updated by ISR 1 only */
}

PRIVILEGED_FUNCTION void test_validMemAccessWithTaskIsrTaskSwitch(void *args)
{
    shared_mem_reset();
    xSemaphoreGive(gTestUserObj.user1Sem);                /* Wakeup user 1 */
    /* User 1 performs valid read operations and trigger ISR 1 */
    /* ISR 1 performs valid read-modify-write operations (increment) and signal user 2 */
    /* User 2 performs valid read-modify-write operations (increment) and trigger ISR 2 */
    /* ISR 2 performs valid read-modify-write operations (increment) and signal user 1 */
    /* `User 1 -> ISR 1 -> User 2 -> ISR 2` routine repeats for TOTAL_NUM_LOOP times */
    xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY);   /* Wait for user 1 to signal */
    shared_mem_check_user2_region(3 * NUM_LOOP_PER_BYTE); /* Shared memory value updated by 2x ISR + user 2 */
    shared_mem_check_sys_region(2 * NUM_LOOP_PER_BYTE);   /* Shared memory value updated by 2x ISR */
}

PRIVILEGED_FUNCTION void test_invalidMemAccessWithTaskIsrTaskSwitch(void *args)
{
    shared_mem_reset();
    RESET_USER1_ERROR_CNT();
    RESET_USER2_ERROR_CNT();
    xSemaphoreGive(gTestUserObj.user1Sem);                /* Wakeup user 1 */
    /* User 1 performs invalid write & invalid read operations and trigger ISR 1 */
    /* ISR 1 performs valid read-modify-write operations (increment) and signal user 2 */
    /* User 2 performs invalid read-modify-write operations and trigger ISR 2 */
    /* ISR 2 performs valid read-modify-write operations (increment) and signal user 1 */
    /* `User 1 -> ISR 1 -> User 2 -> ISR 2` routine repeats for TOTAL_NUM_LOOP times */
    xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY);   /* Wait for user 1 to signal */
    TEST_ASSERT_USER1_ERROR_CHECK();                      /* Check for expected data aborts in user 1 */
    TEST_ASSERT_USER2_ERROR_CHECK();                      /* Check for expected data aborts in user 2 */
    shared_mem_check_full(2 * NUM_LOOP_PER_BYTE);         /* Shared memory value updated by 2x ISR */
}

PRIVILEGED_FUNCTION void test_validMemAccessWithTaskIsrSysSwitch(void *args)
{
    shared_mem_reset();
    for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
    {
        HwiP_post(INT1_NUM);                                /* Trigger interrupt */
        /* ISR 1 performs valid read-modify-write operations (increment) and signal user 1 */
        /* User 1 performs valid read operations and trigger ISR 2 */
        /* ISR 2 performs valid read-modify-write operation (increment) and signal sys (current task) */
        xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY); /* Wait for ISR 2 to signal */
        shared_mem_read_write(loopCnt);                     /* This increments the value in shared memory */
    }
    shared_mem_check_full(3 * NUM_LOOP_PER_BYTE);           /* Shared memory updated by 2x ISR + sys */
}

PRIVILEGED_FUNCTION void test_invalidMemAccessWithTaskIsrSysSwitch(void *args)
{
    shared_mem_reset();
    RESET_USER1_ERROR_CNT();
    for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
    {
        HwiP_post(INT1_NUM);                                /* Trigger interrupt */
        /* ISR 1 performs valid read-modify-write operations (increment) and signal user 1 */
        /* User 1 performs invalid read operations and trigger ISR 2 */
        /* ISR 2 performs valid read-modify-write operation (increment) and signal sys (current task) */
        xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY); /* Wait for ISR 2 to signal */
        shared_mem_read_write(loopCnt);                     /* This increments the value in shared memory */
    }
    TEST_ASSERT_USER1_ERROR_CHECK();                        /* Check for expected data aborts in user 1 */
    shared_mem_check_full(3 * NUM_LOOP_PER_BYTE);           /* Shared memory updated by 2x ISR + sys */
}

PRIVILEGED_FUNCTION void test_invalidFxnAccess(void *args)
{
    RESET_USER1_ERROR_CNT();
    RESET_USER2_ERROR_CNT();
    xSemaphoreGive(gTestUserObj.user1Sem);                /* Wakeup user 1 */ 
    /* User 1 will perform invalid function access (privileged function & user 2 function) */
    xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY);   /* Wait for user 1 to signal */
    TEST_ASSERT_USER1_ERROR_CHECK();                      /* Check for expected prefetch aborts in user 1 */
    xSemaphoreGive(gTestUserObj.user2Sem);                /* Wakeup user 2 */
    /* User 2 will perform invalid function access (privileged function & user 1 function) */
    xSemaphoreTake(gTestUserObj.sysSem, portMAX_DELAY);   /* Wait for user 2 to signal */
    TEST_ASSERT_USER2_ERROR_CHECK();                      /* Check for expected prefetch aborts in user 2 */
}

MPU_UT_TEXT_USER1 void user1_main(void *args)
{
    SemaphoreHandle_t user1Sem = gTestUserObj.user1Sem;
    SemaphoreHandle_t user2Sem = gTestUserObj.user2Sem;
    SemaphoreHandle_t sysSem   = gTestUserObj.sysSem;

    /* test_validMemAccessWithTaskSysSwitch */
    {
        for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            xSemaphoreTake(user1Sem, portMAX_DELAY);      /* Wait for sys to signal */
            user1_valid_shared_mem_read(loopCnt);         /* Perform valid read of shared memory */
            xSemaphoreGive(user2Sem);                     /* Wakeup user 2 */
        }
    }
    /* test_invalidMemAccessWithTaskSysSwitch */
    {
        for (volatile uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            xSemaphoreTake(user1Sem, portMAX_DELAY);      /* Wait for sys to signal */
            user1_invalid_shared_mem_write(loopCnt);      /* Perform write to regions mapped read-only */
            user1_invalid_shared_mem_read(loopCnt);       /* Perform read to un-mapped regions */
            xSemaphoreGive(user2Sem);                     /* Wakeup user 2 */
        }
    }
    /* test_validMemAccessWithTaskIsrSwitch */
    {
        xSemaphoreTake(user1Sem, portMAX_DELAY);          /* Wait for sys to signal */
        for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            user1_valid_shared_mem_read(loopCnt);         /* Perform valid read of shared memory */
            HwiP_post(INT1_NUM);                          /* Trigger interrupt */
            xSemaphoreTake(user1Sem, portMAX_DELAY);      /* Wait for ISR 1 to signal */
        }
        xSemaphoreGive(sysSem);                           /* Wakeup sys */
    }
    /* test_invalidMemAccessWithTaskIsrSwitch */
    {
        xSemaphoreTake(user1Sem, portMAX_DELAY);          /* Wait for sys to signal */
        for (volatile uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            user1_invalid_shared_mem_write(loopCnt);      /* Perform write to regions mapped read-only */
            user1_invalid_shared_mem_read(loopCnt);       /* Perform read to un-mapped regions */
            HwiP_post(INT1_NUM);                          /* Trigger interrupt */
            xSemaphoreTake(user1Sem, portMAX_DELAY);      /* Wait for ISR 1 to signal */
        }
        xSemaphoreGive(sysSem);                           /* Wakeup sys */
    }
    /* test_validMemAccessWithTaskIsrTaskSwitch */
    {
        xSemaphoreTake(user1Sem, portMAX_DELAY);          /* Wait for sys to signal */
        for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            user1_valid_shared_mem_read(loopCnt);         /* Perform valid read of shared memory */
            HwiP_post(INT1_NUM);                          /* Trigger interrupt */
            xSemaphoreTake(user1Sem, portMAX_DELAY);      /* Wait for ISR 2 to signal */
        }
        xSemaphoreGive(sysSem);                           /* Wakeup sys */
    }
    /* test_invalidMemAccessWithTaskIsrTaskSwitch */
    {
        xSemaphoreTake(user1Sem, portMAX_DELAY);          /* Wait for sys to signal */
        for (volatile uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            user1_invalid_shared_mem_write(loopCnt);      /* Perform write to regions mapped read-only */
            user1_invalid_shared_mem_read(loopCnt);       /* Perform read to un-mapped regions */
            HwiP_post(INT1_NUM);                          /* Trigger interrupt */
            xSemaphoreTake(user1Sem, portMAX_DELAY);      /* Wait for ISR 2 to signal */
        }
        xSemaphoreGive(sysSem);                           /* Wakeup sys */
    }
    /* test_validMemAccessWithTaskIsrSysSwitch */
    {
        for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            xSemaphoreTake(user1Sem, portMAX_DELAY);      /* Wait for ISR 1 to signal */
            user1_valid_shared_mem_read(loopCnt);         /* Perform valid read of shared memory */
            HwiP_post(INT2_NUM);                          /* Trigger interrupt */
        }
    }
    /* test_invalidMemAccessWithTaskIsrSysSwitch */
    {
        for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            xSemaphoreTake(user1Sem, portMAX_DELAY);      /* Wait for ISR 1 to signal */
            user1_invalid_shared_mem_write(loopCnt);      /* Perform write to regions mapped read-only */
            user1_invalid_shared_mem_read(loopCnt);       /* Perform read to un-mapped regions */
            HwiP_post(INT2_NUM);                          /* Trigger interrupt */
        }
    }
    /* test_invalidFxnAccess */
    {
        xSemaphoreTake(user1Sem, portMAX_DELAY);          /* Wait for sys to signal */

        SET_FAULT_TRACKER(FAULT_TRACKER_FXN_USER1_START + 0U);
        xSemaphoreCreateBinaryStatic(NULL);               /* Privileged fxn - should cause prefetch abort */
        TEST_ASSERT_FAULT_TRACKER_FROM_USER1(FAULT_TRACKER_FXN_USER1_START + 0U);

        SET_FAULT_TRACKER(FAULT_TRACKER_FXN_USER1_START + 1U);
        user2_main(NULL);                                 /* User 2 fxn - should cause prefetch abort */
        TEST_ASSERT_FAULT_TRACKER_FROM_USER1(FAULT_TRACKER_FXN_USER1_START + 1U);

        xSemaphoreGive(sysSem);                           /* Wakeup sys */
    }

    vTaskSuspend(NULL);
}

MPU_UT_TEXT_USER2 void user2_main(void *args)
{
    SemaphoreHandle_t user2Sem = gTestUserObj.user2Sem;
    SemaphoreHandle_t sysSem   = gTestUserObj.sysSem;
    
    /* test_validMemAccessWithTaskSysSwitch */
    {
        for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            xSemaphoreTake(user2Sem, portMAX_DELAY);      /* Wait for user 1 to signal */
            user2_valid_shared_mem_read_write(loopCnt);   /* This will increment the value in shared memory */
            xSemaphoreGive(sysSem);                       /* Wakeup sys */
        }
    }
    /* test_invalidMemAccessWithTaskSysSwitch */
    {
        for (volatile uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            xSemaphoreTake(user2Sem, portMAX_DELAY);      /* Wait for user 1 to signal */
            user2_invalid_shared_mem_read_write(loopCnt); /* Perform read-modify-write to un-mapped regions */
            xSemaphoreGive(sysSem);                       /* Wakeup sys */
        }
    }
    /* test_validMemAccessWithTaskIsrSwitch   - NA for user 2 */
    /* test_invalidMemAccessWithTaskIsrSwitch - NA for user 2 */
    /* test_validMemAccessWithTaskIsrTaskSwitch */
    {
        for (uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            xSemaphoreTake(user2Sem, portMAX_DELAY);      /* Wait for ISR 1 to signal */
            user2_valid_shared_mem_read_write(loopCnt);   /* This will increment the value in shared memory */
            HwiP_post(INT2_NUM);                          /* Trigger interrupt */
        }
    }
    /* test_invalidMemAccessWithTaskIsrTaskSwitch */
    {
        for (volatile uint8_t loopCnt = 0U; loopCnt < TOTAL_NUM_LOOP; loopCnt++)
        {
            xSemaphoreTake(user2Sem, portMAX_DELAY);      /* Wait for ISR 1 to signal */
            user2_invalid_shared_mem_read_write(loopCnt); /* Perform read-modify-write to un-mapped regions */
            HwiP_post(INT2_NUM);                          /* Trigger interrupt */
        }
    }
    /* test_validMemAccessWithTaskIsrSysSwitch   - NA for user 2 */
    /* test_invalidMemAccessWithTaskIsrSysSwitch - NA for user 2 */
    /* test_invalidFxnAccess */
    {
        xSemaphoreTake(user2Sem, portMAX_DELAY);          /* Wait for sys to signal */

        SET_FAULT_TRACKER(FAULT_TRACKER_FXN_USER2_START + 0U);
        xSemaphoreCreateBinaryStatic(NULL);               /* Privileged fxn - should cause prefetch abort */
        TEST_ASSERT_FAULT_TRACKER_FROM_USER2(FAULT_TRACKER_FXN_USER2_START + 0U);

        SET_FAULT_TRACKER(FAULT_TRACKER_FXN_USER2_START + 1U);
        user1_main(NULL);                                 /* User 1 fxn - should cause prefetch abort */
        TEST_ASSERT_FAULT_TRACKER_FROM_USER2(FAULT_TRACKER_FXN_USER2_START + 1U);

        xSemaphoreGive(sysSem);                           /* Wakeup sys */
    }

    vTaskSuspend(NULL);
}

PRIVILEGED_FUNCTION void sys_main(void *args)
{
    /* loop(SYS -> USER1<R> -> USER2<RW>) */
    RUN_TEST(test_validMemAccessWithTaskSysSwitch, 14445, NULL);
    RUN_TEST(test_invalidMemAccessWithTaskSysSwitch, 14446, NULL);

    /* SYS -> loop(USER1<R> -> ISR1) */
    register_shared_mem_isr(gTestUserObj.user1Sem, NULL);                   /* ISR 1 signal user 1 */
    RUN_TEST(test_validMemAccessWithTaskIsrSwitch, 14447, NULL);    
    RUN_TEST(test_invalidMemAccessWithTaskIsrSwitch, 14448, NULL);

    /* SYS -> loop(USER1<R> -> ISR1 -> USER2<RW> -> ISR2) */
    register_shared_mem_isr(gTestUserObj.user2Sem, gTestUserObj.user1Sem);  /* ISR1->user2 & ISR2->user1 */
    RUN_TEST(test_validMemAccessWithTaskIsrTaskSwitch, 14449, NULL);
    RUN_TEST(test_invalidMemAccessWithTaskIsrTaskSwitch, 14450, NULL);

    /* loop(SYS -> ISR1 -> USER1<R> -> ISR2) */
    register_shared_mem_isr(gTestUserObj.user1Sem, gTestUserObj.sysSem);    /* ISR1->user1 & ISR2->sys */
    RUN_TEST(test_validMemAccessWithTaskIsrSysSwitch, 14451, NULL);
    RUN_TEST(test_invalidMemAccessWithTaskIsrSysSwitch, 14452, NULL);

    RUN_TEST(test_invalidFxnAccess, 14453, NULL);

    /* Signal test completion to `test_freertos_mpu_run` main task */
    xSemaphoreGive(gTestObj.mpuSyncSem); 

    vTaskDelete(NULL);
}

/** 
 * Test setup related privileged functions
 */
/* Return 0 if start address aligned to size(power of 2), else return required align size */
PRIVILEGED_FUNCTION static uint32_t test_freertos_mpu_check_align(uint32_t start, uint32_t end)
{
    uint32_t size      = end - start;
    uint32_t alignSize = 1U << (32U - __builtin_clz(size - 1U));

    alignSize = ((start % alignSize) == 0U) ? 0U : alignSize;

    if(alignSize > 0U)
    {
        DebugP_log("0x%x is not aligned to 0x%x\r\n", start, alignSize);
    }

    return alignSize;
}

PRIVILEGED_FUNCTION static void test_freertos_mpu_create_user1_task(void)
{
    extern uint32_t __TEXT_FREERTOS_MPU_UT_USER1_START[];
    extern uint32_t __TEXT_FREERTOS_MPU_UT_USER1_END[];

    int32_t  status;
    uint32_t alignSize =  test_freertos_mpu_check_align((uint32_t)__TEXT_FREERTOS_MPU_UT_USER1_START, 
                                                        (uint32_t)__TEXT_FREERTOS_MPU_UT_USER1_END);

    if(alignSize != 0U)
    {
        DebugP_log("Update `.text.freertos_mpu_ut.user1` alignment to 0x%x bytes\r\n", alignSize);
        DebugP_assert(0);
    }

    TaskP_ParamsRestricted taskParams = {
        .params = {
            .name       = "user1",
            .stackSize  = USER1_TASK_SIZE,
            .stack      = gUser1TaskStack,
            .priority   = USER1_TASK_PRI,
            .args       = NULL,
            .taskMain   = user1_main,
        },
        /** Task stack will be added as an MPU entry @ `portSTACK_REGION` by the task create API on its own.
         * Hence, no need to explicitly add the same as part of task specific MPU regions */
        .regionConfig = {
            /* Read-only permission for Region 1 - 4 */
            [0U] = {
                /* Shared region 1 */
                .baseAddr   = (uint32_t) &gSharedMem[0U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_S_RW_U_R,
                },
            },
            [1U] = {
                /* Shared region 2 */
                .baseAddr   = (uint32_t) &gSharedMem[1U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_S_RW_U_R,
                },
            },
            [2U] = {
                /* Shared region 3 */
                .baseAddr   = (uint32_t) &gSharedMem[2U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_S_RW_U_R,
                },
            },
            [3U] = {
                /* Shared region 4 */
                .baseAddr   = (uint32_t) &gSharedMem[3U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_S_RW_U_R,
                },
            },
            /* Shared memory region 5 - 8 not given access to user 1 */
            [4U] = {
                /* User test object */
                .baseAddr   = (uint32_t) &gTestUserObj,
                .sizeBytes  = sizeof(test_freertos_mpu_user_obj),
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
            [5U] = {
                /* User 1 text */
                .baseAddr   = (uint32_t) __TEXT_FREERTOS_MPU_UT_USER1_START,
                .sizeBytes  = ((uint32_t) __TEXT_FREERTOS_MPU_UT_USER1_END - 
                               (uint32_t) __TEXT_FREERTOS_MPU_UT_USER1_START),
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 0U, /* Execute permission for text region*/
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
            /* Last region is not used with user 1 task */
        },
    };

    status = TaskP_constructRestricted(&gTestObj.user1TaskObj, &taskParams);
    DebugP_assert(status==SystemP_SUCCESS);
}

void test_freertos_mpu_create_user2_task(void)
{
    extern uint32_t __TEXT_FREERTOS_MPU_UT_USER2_START[];
    extern uint32_t __TEXT_FREERTOS_MPU_UT_USER2_END[];

    int32_t  status;
    uint32_t alignSize =  test_freertos_mpu_check_align((uint32_t)__TEXT_FREERTOS_MPU_UT_USER2_START, 
                                                        (uint32_t)__TEXT_FREERTOS_MPU_UT_USER2_END);

    if(alignSize != 0U)
    {
        DebugP_log("Update `.text.freertos_mpu_ut.user2` alignment to 0x%x bytes\r\n", alignSize);
        DebugP_assert(0);
    }

    TaskP_ParamsRestricted taskParams = {
        .params = {
            .name       = "user2",
            .stackSize  = USER2_TASK_SIZE,
            .stack      = gUser2TaskStack,
            .priority   = USER2_TASK_PRI,
            .args       = NULL,
            .taskMain   = user2_main,
        },
        /** Task stack will be added as an MPU entry @ `portSTACK_REGION` by the task create API on its own.
         * Hence, no need to explicitly add the same as part of task specific MPU regions */
        .regionConfig = {
            /* Read-write permission for Region 1 - 5 */
            [0U] = {
                /* Shared Region 1 */
                .baseAddr   = (uint32_t) &gSharedMem[0U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
            [1U] = {
                /* Shared region 2 */
                .baseAddr   = (uint32_t) &gSharedMem[1U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
            [2U] = {
                /* Shared region 3 */
                .baseAddr   = (uint32_t) &gSharedMem[2U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
            [3U] = {
                /* Shared region 4 */
                .baseAddr   = (uint32_t) &gSharedMem[3U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
            [4U] = {
                /* Shared region 5 */
                .baseAddr   = (uint32_t) &gSharedMem[4U][0U],
                .sizeBytes  = SHARED_MEM_REGION_BYTES,
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
            /* Shared memory region 6 - 8 is not given access to user 2 */
            [5U] = {
                /* User test object */
                .baseAddr   = (uint32_t) &gTestUserObj,
                .sizeBytes  = sizeof(test_freertos_mpu_user_obj),
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 1U,
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
            [6U] = {
                /* User 2 text */
                .baseAddr   = (uint32_t) __TEXT_FREERTOS_MPU_UT_USER2_START,
                .sizeBytes  = ((uint32_t) __TEXT_FREERTOS_MPU_UT_USER2_END - 
                               (uint32_t) __TEXT_FREERTOS_MPU_UT_USER2_START),
                .attrs      = {
                    .isCacheable    = 1U,
                    .isBufferable   = 1U,
                    .isSharable     = 0U,
                    .isExecuteNever = 0U, /* Execute permission for text region*/
                    .tex            = 1U,
                    .accessPerm     = MpuP_AP_ALL_RW,
                },
            },
        },
    };

    status = TaskP_constructRestricted(&gTestObj.user2TaskObj, &taskParams);
    DebugP_assert(status==SystemP_SUCCESS);
}

void test_freertos_mpu_create_sys_task(void)
{
    int32_t      status;
    TaskP_Params taskParams;

    TaskP_Params_init(&taskParams);
    taskParams.name      = "system";
    taskParams.stackSize = SYS_TASK_SIZE;
    taskParams.stack     = gSysTaskStack;
    taskParams.priority  = SYS_TASK_PRI;
    taskParams.args      = NULL;
    taskParams.taskMain  = sys_main;

    status = TaskP_construct(&gTestObj.sysTaskObj, &taskParams);
    DebugP_assert(status==SystemP_SUCCESS);
}

void test_freertos_mpu_run()
{
    DebugP_log("\r\nRunning FreeRTOS MPU unit tests...\r\n");

    gTestObj.mpuSyncSem = xSemaphoreCreateBinaryStatic(&gTestObj.mpuSyncSemObj);
    configASSERT(gTestObj.mpuSyncSem != NULL);

    gTestUserObj.user1Sem = xSemaphoreCreateBinaryStatic(&gTestObj.user1SemObj);
    configASSERT(gTestUserObj.user1Sem != NULL);

    gTestUserObj.user2Sem = xSemaphoreCreateBinaryStatic(&gTestObj.user2SemObj);
    configASSERT(gTestUserObj.user2Sem != NULL);

    gTestUserObj.sysSem = xSemaphoreCreateBinaryStatic(&gTestObj.sysSemObj);
    configASSERT(gTestUserObj.sysSem != NULL);

    /* create the tasks, order of task creation does not matter for this test */
    test_freertos_mpu_create_user1_task();
    test_freertos_mpu_create_user2_task();
    test_freertos_mpu_create_sys_task();

    /* Wait for tasks to complete tests */
    xSemaphoreTake(gTestObj.mpuSyncSem, portMAX_DELAY);
    
    TaskP_destruct(&gTestObj.user1TaskObj);
    TaskP_destruct(&gTestObj.user2TaskObj);
    TaskP_destruct(&gTestObj.sysTaskObj);

    DebugP_log("Completed FreeRTOS MPU tests!!!\r\n");
}

/** 
 * Various abort handlers 
 * 
 * This is for the test to proceed even after an expected fault.
 */
__attribute__((section(".text.hwi")))
void HwiP_user_data_abort_handler_c(DFSR dfsr, ADFSR adfsr, volatile uint32_t DFAR, volatile uint32_t LR,
                                    volatile uint32_t SPSR)
{
    uint32_t faultIdx;

    for (faultIdx = FAULT_TRACKER_DATA_ABORT_START; faultIdx < FAULT_TRACKER_DATA_ABORT_MAX; faultIdx++)
    {
        if (gTestUserObj.faultTracker[faultIdx] > 0U)
        {
            /** This is only for test purpose. 
             * Returning from a data abort exception may result in unexpected behaviour. */
            gTestUserObj.faultTracker[faultIdx]--;
            break;
        }
    }
    if (faultIdx == FAULT_TRACKER_DATA_ABORT_MAX)
    {
        /* Unexpected fault - loop forever */
        volatile uint32_t loop = 1;
        while(loop != 0U){ ; }
    }
}

__attribute__((section(".text.hwi")))
void  HwiP_user_prefetch_abort_handler_c(IFSR ifsr, AIFSR aifsr, volatile uint32_t IFAR, 
                                         volatile uint32_t LR,volatile uint32_t SPSR)
{
    uint32_t faultIdx;

    for (faultIdx = FAULT_TRACKER_PREFETCH_ABORT_START; faultIdx < FAULT_TRACKER_PREFETCH_ABORT_MAX; faultIdx++)
    {
        if (gTestUserObj.faultTracker[faultIdx] > 0U)
        {
            /** This is only for test purpose. 
             * Returning from a prefetch abort exception may result in unexpected behaviour. */
            gTestUserObj.faultTracker[faultIdx]--;
            break;
        }
    }
    if (faultIdx == FAULT_TRACKER_PREFETCH_ABORT_MAX)
    {
        /* Unexpected fault - loop forever */
        volatile uint32_t loop = 1;
        while(loop != 0U){ ; }
    }
}