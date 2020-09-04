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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "ecat_def.h"

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>

#include <tiescutils.h>
#include <tiesceoefoe.h>
#include <tiescsoc.h>

#include <applInterface.h>
#include <ecatslv.h>

#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>

//#define PROFILE_ECAT_STACK
#ifdef PROFILE_ECAT_STACK
uint32_t mainloop_delta, mainloop_max, pdi_delta, pdi_max, sync_delta, sync_max;
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TASK1_PRIORITY          (4)
#define PDI_TASK_PRIORITY       (6)
#define LED_TASK_PRIORITY       (4)
#define SYNC0_TASK_PRIORITY     (8)
#define SYNC1_TASK_PRIORITY     (8)

#define TASK1_STACK_SIZE        (2048*2)
#define PDI_TASK_STACK_SIZE     (1152*2)
#define LED_TASK_STACK_SIZE     (1512*2)
#define SYNC0_TASK_STACK_SIZE   (1152*2)
#define SYNC1_TASK_STACK_SIZE   (1152*2)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint32_t gTask1Stack[TASK1_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
uint32_t gPdiTaskStack[PDI_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
uint32_t gLedTaskStack[LED_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
uint32_t gSync0TaskStack[SYNC0_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
uint32_t gSync1TaskStack[SYNC1_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

/** \brief Global Structure pointer holding PRUSS0 memory Map. */
PRUICSS_Handle pruIcss1Handle;
/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle pruIcss0Handle;

TaskP_Object task1Object;               // ECAT mainloop
#ifdef ENABLE_PDI_TASK
TaskP_Object pdiTaskObject;             // ECAT sw ISR
#endif
TaskP_Object ledTaskObject;             // LED Control Task
#ifdef ENABLE_SYNC_TASK
TaskP_Object sync0TaskObject;           // ECAT SYNC0 ISR
TaskP_Object sync1TaskObject;           // ECAT SYNC1 ISR
#endif

uint32_t appState = 0;

/* ========================================================================== */
/*                       Function Definitions                                 */
/* ========================================================================== */

uint8_t task1_init()
{
    uint8_t u8Err = 0;
    uint32_t status = SystemP_FAILURE;
    TaskP_Params taskParams;

    if(tiesc_isEthercatDevice())
    {
        DebugP_log("EtherCAT Device\n\r");
    }

    else
    {
        DebugP_log("Non-EtherCAT Device\n\r");
    }

    DebugP_log("EtherCAT Sample application\n\r");

    /* initialize the Hardware and the EtherCAT Slave Controller */
    HW_Init();

    /* Disable IEEE Power Down mode so that PHY can establish link */
    tiesc_ethphyDisablePowerDown();

    u8Err = MainInit(); // EtherCAT stack init

#if EOE_SUPPORTED || FOE_SUPPORTED
    tiesc_foe_eoe_init();
#endif

    /* Create tasks */
    /* Create tree tasks that share a resource*/
#ifdef ENABLE_PDI_TASK
    TaskP_Params_init(&taskParams);
    taskParams.name = "PDItask";
    taskParams.stackSize = PDI_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gPdiTaskStack;
    taskParams.priority = PDI_TASK_PRIORITY;
    taskParams.args = (void *)pruIcss1Handle;
    taskParams.taskMain = (TaskP_FxnMain)PDItask;
    status = TaskP_construct(&pdiTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("PDI Task creation failed\r\n");
    }
#endif
#ifdef ENABLE_SYNC_TASK
    TaskP_Params_init(&taskParams);
    taskParams.name = "Sync0task";
    taskParams.stackSize = SYNC0_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gSync0TaskStack;
    taskParams.priority = SYNC0_TASK_PRIORITY;
    taskParams.args = (void *)pruIcss1Handle;
    taskParams.taskMain = (TaskP_FxnMain)Sync0task;
    status = TaskP_construct(&sync0TaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Sync0 Task creation failed\r\n");
    }

    TaskP_Params_init(&taskParams);
    taskParams.name = "Sync1task";
    taskParams.stackSize = SYNC1_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gSync1TaskStack;
    taskParams.priority = SYNC1_TASK_PRIORITY;
    taskParams.args = (void *)pruIcss1Handle;
    taskParams.taskMain = (TaskP_FxnMain)Sync1task;
    status = TaskP_construct(&sync1TaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Sync1 Task creation failed\r\n");
    }
#endif
    TaskP_Params_init(&taskParams);
    taskParams.name = "LEDtask";
    taskParams.stackSize = LED_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gLedTaskStack;
    taskParams.priority = LED_TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)LEDtask;
    status = TaskP_construct(&ledTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("LED Task creation failed\r\n");
    }

    return u8Err;
}

void task1(void *arg)
{
    static int task1init = FALSE;
    if(!task1init)
    {
        task1_init();
        task1init = TRUE;
    }
    bRunApplication = TRUE;
    do
    {
#ifdef PROFILE_ECAT_STACK
        {
            uint32_t mainloop_start, mainloop_stop;
            bsp_get_local_sys_time(&mainloop_start, 0);
#endif

            MainLoop();
#ifdef PROFILE_ECAT_STACK
            bsp_get_local_sys_time(&mainloop_stop,  0);

            if(mainloop_stop >= mainloop_start)
            {
                mainloop_delta = mainloop_stop - mainloop_start;
            }

            else
            {
                mainloop_delta = 0xFFFFFFFFF - mainloop_start + mainloop_stop;
            }

            if(mainloop_delta > mainloop_max)
            {
                mainloop_max = mainloop_delta;
            }
        }
#endif
        TaskP_yield();
    }
    while(bRunApplication == TRUE);

    HW_Release();

    /* TODO: Add appropriate replacement*/
//    OSAL_OS_exit(0);
}



#ifdef ENABLE_PDI_TASK
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief Interrupt service routine for the interrupts from the EtherCAT Slave Controller
*////////////////////////////////////////////////////////////////////////////////////////
#if AL_EVENT_ENABLED
void HW_EcatIsr(void)
{
#ifdef PROFILE_ECAT_STACK
    uintptr_t key1;
    uint32_t pdi_start, pdi_stop;
    /* get the AL event register */

    key1 = HwiP_disable();
    bsp_get_local_sys_time(&pdi_start,  0);
#endif //PROFILE_ECAT_STACK
    PDI_Isr();
#ifdef PROFILE_ECAT_STACK
    bsp_get_local_sys_time(&pdi_stop,   0);
    HwiP_restore(key1);

    if(pdi_stop >= pdi_start)
    {
        pdi_delta = pdi_stop - pdi_start;
    }
    else
    {
        pdi_delta = 0xFFFFFFFFF - pdi_start + pdi_stop;
    }

    if(pdi_delta > pdi_max)
    {
        pdi_max = pdi_delta;
    }

#endif //PROFILE_ECAT_STACK
}
#endif //AL_EVENT_ENABLED

void PDItask(void *arg)
{
//    TaskP_sleep(10 * OS_TICKS_IN_MILLI_SEC);
    /*TODO: Check if following is correct*/
    ClockP_usleep(10 * 1000);
#if AL_EVENT_ENABLED
    uint32_t evtOutNum = HOST_AL_EVENT - 20;
    while(1)
    {
        PRUICSS_waitEvent((PRUICSS_Handle)arg, evtOutNum);
        /* ISR processing */
        HW_EcatIsr();
    }
#endif //AL_EVENT_ENABLED
}
#endif //ENABLE_PDI_TASK

void LEDtask(void *args)
{
    VARVOLATILE uint8_t state = STATE_INIT;
#ifdef ENABLE_STARTUP_DIGOUT_ANIMATION
        VARVOLATILE uint8_t state = STATE_INIT;
        static uint16_t digout_led_mask = 1;

        while(STATE_INIT == state)
        {
            I2CSetLed(&gI2CObj, digout_led_mask);
            digout_led_mask <<= 1;

            if(digout_led_mask >= 256)
            {
                digout_led_mask = 1;
            }

            HW_EscReadByte(state, ESC_ADDR_ALSTATUS);
            TaskP_sleep(200 * OS_TICKS_IN_MILLI_SEC);
        }

#else
        ClockP_usleep(200 * 1000);
        tiesc_setOutputLed(0x6a);
#endif
    while(1)
    {
        ClockP_usleep(50 * 1000);

        uint32_t reset_reg_val;

#if ESC_EEPROM_EMULATION
        HW_EscReadByte(state, ESC_ADDR_ALSTATUS);
        state &= 0xF;
        {
            if(bsp_get_eeprom_update_status())
            {
                uint32_t t_cur_time;
#ifdef USE_ECAT_TIMER
                bsp_get_local_sys_time(&t_cur_time, NULL);
#else
                t_cur_time = Timestamp_get32();
#endif
                uint32_t t_last_time = bsp_get_eeprom_updated_time();
                t_cur_time = ((t_cur_time >= t_last_time) ? (t_cur_time - t_last_time) :
                              t_cur_time + (0xFFFFFFFF - t_last_time));

                if(t_cur_time >= (ECAT_TIMER_INC_P_MS * 100))
                {
                    bsp_set_eeprom_update_status(0);
                    bsp_eeprom_emulation_flush();
                }
            }
        }

#endif
        reset_reg_val = bsp_read_dword(pruIcss1Handle, ESC_ADDR_TI_ESC_RESET);

        if((reset_reg_val == TI_ESC_RST_CMD_U) ||
                (reset_reg_val == TI_ESC_RST_CMD_L))
        {
            /*Write 0x0 to 0xE14, else application will request reset continuously.*/
            bsp_write_dword(pruIcss1Handle, (uint32_t)0x0, ESC_ADDR_TI_ESC_RESET);
            ASSERT_DMB();
            ASSERT_DSB();
            //EtherCAT master has requested S/W RESET
            HW_RestartTarget();
        }
    }
}

#ifdef ENABLE_SYNC_TASK
void Sync0task(void *arg)
{
#ifdef PROFILE_ECAT_STACK
    uint32_t sync_start, sync_stop;
#endif
#ifndef EXEC_FROM_IRAM
    DebugP_log("SYNC0 task started\n\r");
#endif
    uint32_t evtOutNum = HOST_SYNC0_EVENT - 20;
    while(1)
    {
        PRUICSS_waitEvent((PRUICSS_Handle)arg, evtOutNum);

        //Do sync0 event handling
        DISABLE_ESC_INT();
#ifdef PROFILE_ECAT_STACK
        uintptr_t key1;
        key1 = HwiP_disable();
        bsp_get_local_sys_time(&sync_start, 0);
#endif
        Sync0_Isr();

        if(!bsp_read_word((PRUICSS_Handle)arg, ESC_ADDR_SYNC_PULSE_LENGTH))
        {
            bsp_read_byte((PRUICSS_Handle)arg, ESC_ADDR_SYNC_STATUS);
        }

#ifdef PROFILE_ECAT_STACK
        bsp_get_local_sys_time(&sync_stop,  0);
        HwiP_restore(key1);

        if(sync_stop >= sync_start)
        {
            sync_delta = sync_stop - sync_start;
        }
        else
        {
            sync_delta = 0xFFFFFFFFF - sync_start + sync_stop;
        }

        if(sync_delta > sync_max)
        {
            sync_max = sync_delta;
        }

#endif
        ENABLE_ESC_INT();
    }
}

void Sync1task(void *arg)
{

#ifndef EXEC_FROM_IRAM
    DebugP_log("SYNC1 task started\n\r");
#endif
    uint32_t evtOutNum = HOST_SYNC1_EVENT - 20;
    while(1)
    {
        PRUICSS_waitEvent((PRUICSS_Handle)arg, evtOutNum);

        //Do sync1 event handling
        DISABLE_ESC_INT();
        Sync1_Isr();

        if(!bsp_read_word((PRUICSS_Handle)arg, ESC_ADDR_SYNC_PULSE_LENGTH))
        {
            bsp_read_byte((PRUICSS_Handle)arg, ESC_ADDR_SYNC_STATUS + 1);
        }

        ENABLE_ESC_INT();
    }
}
#endif

void common_main()
{
    uint32_t status;
    TaskP_Params taskParams;

    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    tiesc_socEvmInit();
    #ifdef MDIO_MANUAL_MODE_ENABLED
    tiesc_mdioManualModeSetup();
    #endif

    /* Enable IEEE Power Down mode so that PHY does not establish any link */
    tiesc_ethphyEnablePowerDown();

    TaskP_Params_init(&taskParams);
    taskParams.name = "task1";
    taskParams.stackSize = TASK1_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTask1Stack;
    taskParams.priority = TASK1_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)task1;
    status = TaskP_construct(&task1Object, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Task1 creation failed\r\n");
    }

}
