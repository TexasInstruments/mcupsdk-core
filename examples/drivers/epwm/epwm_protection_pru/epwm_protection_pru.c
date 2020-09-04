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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include <drivers/pruicss.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/* Frequency of PWM output signal in Hz - 1 KHz is selected */
#define APP_EPWM_OUTPUT_FREQ    (1U * 1000U)
/* APP run time in seconds */
#define APP_EPWM_RUN_TIME    (60U)
/* FIXME : To be removed after syscfg integration */
#define APP_INT_IS_PULSE    (1U)

/* Global variables and objects */
static HwiP_Object  gEpwmHwiObject;
static SemaphoreP_Object  gEpwmSyncSemObject;

/* Function Prototypes */
static void App_epwmIntrISR(void *handle);

/* variable to hold base address of EPWM that is used */
uint32_t gEpwmBaseAddr;

const unsigned int PRUFirmware[]= {
0x2400001e,
0x2400001f,
0x24000018,
0x24001801,
0x24000294,
0x240000d4,
0x80013894,
0x24001c01,
0x90013899,
0x101f1f15,
0x570015ff,
0x90151816,
0x570016fd,
0x5100f904,
0x24000a1e,
0x0501f9f9,
0x6f00f9ff,
0x1016161e,
0x81101816,
0x91141817,
0x570017ff,
0x2400001e,
0x81101818,
0x81141818,
0x24001c01,
0x90013899,
0x7f0000ef };


void writePru0Firmware(uint32_t *ptr, uint32_t size)
{
    uint32_t i;
    
    for(i=0;i<(size/4);i++)
        
    HW_WR_REG32(CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_PDSP0_IRAM_RAM_REGS_BASE + i*4, ptr[i]);
}

uint8_t PRULut[]= {
0x00,
0x00,
0x0a,
0x0a,
0x0a,
0x0a,
0x0a,
0x0a,
0x00,
0x00,
0x0a,
0x0a,
0x06,
0x09,
0x06,
0x09};

void writePru0ProtectionLut(uint8_t *ptr, uint32_t size)
{
    uint32_t i;
    
    for(i=0;i<size;i++)
        
    HW_WR_REG8(CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_DRAM0_SLV_RAM_REGS_BASE + i, ptr[i]);
}

void gen_speed_level(void)
{
    GPIO_pinWriteHigh(GPIO_SPEED_LEVEL_OUT_BASE_ADDR, GPIO_SPEED_LEVEL_OUT_PIN);
    DebugP_log("Speed level set. ");
}
void gen_igbt_fault(void)
{
    GPIO_pinWriteHigh(GPIO_IGBT_FAULT_OUT_BASE_ADDR,  GPIO_IGBT_FAULT_OUT_PIN);
    DebugP_log("IGBT fault set. ");
}
void gen_ovp(void)
{
    GPIO_pinWriteHigh(GPIO_OVP_OUT_BASE_ADDR,         GPIO_OVP_OUT_PIN);
    DebugP_log("OVP set. ");
}
void gen_ocp(void)
{
    GPIO_pinWriteHigh(GPIO_OCP_OUT_BASE_ADDR,         GPIO_OCP_OUT_PIN);
    DebugP_log("OCP set. ");
}
void generateTestSignals(uint8_t testvec)
{
    switch(testvec&0x0F)
    {
        case 0:
            break;
        case 1:
            gen_igbt_fault();
            break;
        case 2:
            gen_ocp();
            break;
        case 3:
            gen_ocp();
            gen_igbt_fault();
            break;
        case 4:
            gen_ovp();
            break;
        case 5:
            gen_ovp();
            gen_igbt_fault();
            break;
        case 6:
            gen_ocp();
            gen_ovp();
            break;
        case 7:
            gen_ocp();
            gen_ovp();
            gen_igbt_fault();
            break;
        case 8:
            gen_speed_level();
            break;
        case 9:
            gen_speed_level();
            gen_igbt_fault();
            break;
        case 10:
            gen_speed_level();
            gen_ocp();
            break;
        case 11:
            gen_speed_level();
            gen_ocp();
            gen_igbt_fault();
            break;
        case 12:
            gen_speed_level();
            //gen_igbt_fault();
            gen_ovp();
            break;
        case 13:
            gen_speed_level();
            gen_igbt_fault();
            gen_ovp();
            break;
        case 14:
            gen_speed_level();
            //gen_igbt_fault();
            gen_ovp();
            gen_ocp();
            break;
        case 15:
            gen_speed_level();
            gen_igbt_fault();
            gen_ovp();
            gen_ocp();
            break;
        default:
            break;
    }
}

void epwm_protection_pru_main(void *args)
{
    int32_t  status;
    uint32_t  numIsrCnt = (APP_EPWM_RUN_TIME * APP_EPWM_OUTPUT_FREQ);
    HwiP_Params  hwiPrms;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EPWM Protection PRU Test Started ...\r\n");

    /* Get Address of ePWM */
    gEpwmBaseAddr = CONFIG_PWM0_BASE_ADDR;

    status = SemaphoreP_constructCounting(&gEpwmSyncSemObject, 0, numIsrCnt);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    /* Integrate with Syscfg */
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &App_epwmIntrISR;
    /* Integrate with Syscfg */
    hwiPrms.isPulse     = APP_INT_IS_PULSE;
    status              = HwiP_construct(&gEpwmHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);

    //HW_WR_REG32(0x50D0081C, 0x3FFFFFFF);
    SOC_selectIcssGpiMux(0, 0x3FFFFFFF);

    HW_WR_REG32(CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_PDSP0_IRAM_REGS_BASE +0, 0x0);
    DebugP_log("PRU0 Processor disabled ...\r\n");
    
    writePru0Firmware((uint32_t *) PRUFirmware, sizeof(PRUFirmware));
    DebugP_log("PRU0 IRAM written ...\r\n");

    writePru0ProtectionLut((uint8_t *) PRULut, sizeof(PRULut));
    DebugP_log("PRU LUT written ...\r\n");
    
    HW_WR_REG32(CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_DRAM0_SLV_RAM_REGS_BASE + 28, 100000);  /* Freewheeling time: 100000 * 10ns */
    DebugP_log("Freewheeling time written ...\r\n");
    
    HW_WR_REG32(CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_PDSP0_IRAM_REGS_BASE +0, 0x2);
    DebugP_log("PRU0 Processor enabled ...\r\n");

    GPIO_setDirMode(GPIO_IGBT_FAULT_IN_BASE_ADDR,   GPIO_IGBT_FAULT_IN_PIN,   GPIO_IGBT_FAULT_IN_DIR);
    GPIO_setDirMode(GPIO_OCP_IN_BASE_ADDR,          GPIO_OCP_IN_PIN,          GPIO_OCP_IN_DIR);
    GPIO_setDirMode(GPIO_OVP_IN_BASE_ADDR,          GPIO_OVP_IN_PIN,          GPIO_OVP_IN_DIR);
    GPIO_setDirMode(GPIO_SPEED_LEVEL_IN_BASE_ADDR,  GPIO_SPEED_LEVEL_IN_PIN,  GPIO_SPEED_LEVEL_IN_DIR);
    
    //Signals generated for testing
    GPIO_setDirMode(GPIO_IGBT_FAULT_OUT_BASE_ADDR,  GPIO_IGBT_FAULT_OUT_PIN,  GPIO_IGBT_FAULT_OUT_DIR);
    GPIO_setDirMode(GPIO_OCP_OUT_BASE_ADDR,         GPIO_OCP_OUT_PIN,         GPIO_OCP_OUT_DIR);
    GPIO_setDirMode(GPIO_OVP_OUT_BASE_ADDR,         GPIO_OVP_OUT_PIN,         GPIO_OVP_OUT_DIR);
    GPIO_setDirMode(GPIO_SPEED_LEVEL_OUT_BASE_ADDR, GPIO_SPEED_LEVEL_OUT_PIN, GPIO_SPEED_LEVEL_OUT_DIR);

    uint8_t testvec=0;
    
    while(true)
    {
        DebugP_log("Normal PWM output (check PWM0_A, PWM0_B, PWM1_A, PWM1_B, PWM2_A, PWM2_B)...\r\n");
        ClockP_sleep(1);
        
        DebugP_log("Generating test signals (IGBT fault, OCP, OVP, Speed level signals)...\r\n");
        
        while( ((testvec&0x0F)==0) ||((testvec&0x0F)==1) ||((testvec&0x0F)==8) ||((testvec&0x0F)==9) )
        {
            testvec++;
        }
        generateTestSignals(testvec);
        testvec++;
        DebugP_log("\r\n");
        
        ClockP_sleep(1);
        GPIO_pinWriteLow(GPIO_IGBT_FAULT_OUT_BASE_ADDR,  GPIO_IGBT_FAULT_OUT_PIN);
        GPIO_pinWriteLow(GPIO_OCP_OUT_BASE_ADDR,         GPIO_OCP_OUT_PIN);
        GPIO_pinWriteLow(GPIO_OVP_OUT_BASE_ADDR,         GPIO_OVP_OUT_PIN);
        GPIO_pinWriteLow(GPIO_SPEED_LEVEL_OUT_BASE_ADDR, GPIO_SPEED_LEVEL_OUT_PIN);
        ClockP_sleep(1);
        DebugP_log("Done...\r\n");
        
        DebugP_log("Waiting to enter protection status...\r\n");
        uint8_t protection_status=0;
        while(protection_status == 0)
        {
            ClockP_sleep(1);
            DebugP_log("Waiting to enter protection status...\r\n");
            protection_status = HW_RD_REG8(CSL_ICSSM0_INTERNAL_U_BASE + 16);
        }
        DebugP_log("Entered Protection status 0x%02x...\r\n", protection_status);
        
        
        HW_WR_REG8(CSL_ICSSM0_INTERNAL_U_BASE + 20, 0x01);
        DebugP_log("Clearing protection status...\r\n");
        uint8_t clr_status=0x01;
        while(clr_status == 0x01)
        {
            ClockP_sleep(1);
            DebugP_log("Waiting to clear protection status...\r\n");
            clr_status = HW_RD_REG8(CSL_ICSSM0_INTERNAL_U_BASE + 20);
        }
        DebugP_log("Cleared protection status...\r\n \r\n");
        
    }


    while(numIsrCnt > 0)
    {
        SemaphoreP_pend(&gEpwmSyncSemObject, SystemP_WAIT_FOREVER);
        numIsrCnt--;
    }

    EPWM_disableInterrupt(gEpwmBaseAddr);
    EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);     /* Clear any pending interrupts if any */
    HwiP_destruct(&gEpwmHwiObject);
    SemaphoreP_destruct(&gEpwmSyncSemObject);

    DebugP_log("EPWM Duty Cycle Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void App_epwmIntrISR(void *handle)
{
    volatile bool status;

    status = EPWM_getEventTriggerInterruptStatus(gEpwmBaseAddr);
    if(status == true)
    {
        SemaphoreP_post(&gEpwmSyncSemObject);
        EPWM_clearEventTriggerInterruptFlag(gEpwmBaseAddr);
    }

    return;
}


