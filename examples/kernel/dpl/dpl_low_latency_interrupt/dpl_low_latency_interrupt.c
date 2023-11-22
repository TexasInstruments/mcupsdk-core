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


#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <kernel/dpl/CycleCounterP.h>
#include <drivers/gpio.h>
#include <drivers/epwm.h>
#include <drivers/dac.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

uint32_t gVimBaseAddr = CSL_VIM_U_BASE;

void util_epwm_periodic_interrupt_config(uint32_t base, uint16_t period)
{
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounterMode( base, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBasePeriod(base, period);
    EPWM_enableInterrupt(base);
	EPWM_setInterruptSource(base, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(base, 1);
}

void util_epwm_en_epwm_clocks(uint32_t mask)
{
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);
    //SOC_setEpwmTbClk(10, TRUE);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC, mask);
}

void util_epwm_all_deInit()
{
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK0 ,0x01234567);
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_LOCK0_KICK1 ,0xFEDCBA8);

    uint8_t i;
    for(i=0;i<32;i++)
    {
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + (i*4),0x07);
        HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + (i*4),0x00);
    }
}

void util_intxbar_g0_config(uint32_t out, uint32_t g0mask)
{
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, out, g0mask, 0, 0, 0, 0, 0, 0);
}

void util_vim_register_irq(uint8_t intNum, uint8_t priority, uintptr_t fn)
{
    HwiP_setVecAddr(intNum, fn);
    HwiP_setPri(intNum, priority);
    HwiP_enableInt(intNum);
}

void util_vim_deregister_irq(uint8_t intNum)
{
    HwiP_setVecAddr(intNum, 0);
    HwiP_setPri(intNum, 15);
    HwiP_disableInt(intNum);
}

void util_iomux_config_for_gpio(uint32_t gpioNum)
{
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0,0x83E70B13);
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1,0x95A4F1E0);

    HW_WR_REG32(CSL_IOMUX_U_BASE + gpioNum*4, 0x07) ;
}


/*************** test_case_1 ***************/

uint32_t gpioNum_tc1 = 43;

void user_isr1_tc1()
{
    HW_WR_REG16(CSL_CONTROLSS_G0_EPWM10_U_BASE + CSL_EPWM_ETCLR, CSL_EPWM_ETCLR_INT_MASK);
}

uint32_t gEpwmIntrNum_tc1;
uint32_t gEpwm_vim_sts_addr_tc1;
uint32_t gEpwm_vim_sts_clr_mask_tc1;
void *epwmArgs_tc1;
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void irq_handler1_tc1(void)
{
    ISR_CALL_PULSE_NONFLOAT_NONREENTRANT(user_isr1_tc1, epwmArgs_tc1, gEpwmIntrNum_tc1, gEpwm_vim_sts_addr_tc1, gEpwm_vim_sts_clr_mask_tc1, gVimBaseAddr);
}

int32_t test_case_1()
{
    CycleCounterP_reset(); /* enable and reset CPU cycle counter */

    DebugP_log("\r\nConfiguring source of interrupts... ");
    uint32_t intNum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    util_epwm_periodic_interrupt_config(CSL_CONTROLSS_G0_EPWM10_U_BASE, 2000);
    util_epwm_en_epwm_clocks(0xFFFFFFFF);
    util_intxbar_g0_config(0, INT_XBAR_EPWM10_INT);
    DebugP_log("Done.");

    DebugP_log("\r\nRegistering interrupt handler in R5F VIM... ");
    gEpwmIntrNum_tc1 = intNum;
    gEpwm_vim_sts_addr_tc1 = gVimBaseAddr +  (0x404u + (((intNum)>> 5) & 0xFu) * 0x20u);
    gEpwm_vim_sts_clr_mask_tc1 = 0x1u << ((intNum) & 0x1Fu);
    epwmArgs_tc1 = NULL;
    util_vim_register_irq(intNum, 4, (uintptr_t)&irq_handler1_tc1);
    DebugP_log("Done.");

    DebugP_log("\r\nConfiguring pin to toggle in background loop... ");
    util_iomux_config_for_gpio(gpioNum_tc1);
    GPIO_setDirMode(CSL_GPIO0_U_BASE, gpioNum_tc1, 0);
    DebugP_log("Done.");

    DebugP_log("\r\nPlease observe pin (GPIO %d) ", gpioNum_tc1);
    DebugP_log("\r\n   If toggling         -> Background loop running");
    DebugP_log("\r\n   If gap/no toggling  -> Background loop interrupted. Interrupt routine running");
    DebugP_log("\r\nWaiting for few seconds... ");
    uint32_t i;
    for(i=0;i<200000000;i++)
    {
        GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, gpioNum_tc1);
        GPIO_pinWriteLow(CSL_GPIO0_U_BASE, gpioNum_tc1);
    }
    DebugP_log("Done.");

    GPIO_setDirMode(CSL_GPIO0_U_BASE, gpioNum_tc1, 1);

    DebugP_log("\r\nDe-initializations... ");
    util_vim_deregister_irq(intNum);
    util_intxbar_g0_config(0, 0);
    util_epwm_en_epwm_clocks(0x0);
    util_epwm_all_deInit();
    DebugP_log("Done.");

    return 0;
}


/*************** test_case_2 ***************/

static float gF1, gF2, gF3;

void float_arith_bg(void)
{
    gF1 = 3.3;

    if( (gF2<0.1) || (gF2>100.0) )
    {
        gF2 = 0.1;
    }
    else
    {
        gF2 = gF2 + 0.01;
    }

    gF3 = gF1*gF2;
}


void float_arith_isr()
{
    uint16_t dac_code = (uint16_t) ((gF3/3.3f)*4096);
    HW_WR_REG16(CSL_CONTROLSS_DAC0_U_BASE + 0x06, dac_code);
}

uint32_t count_isr1_tc2;
uint32_t count_isr2_tc2;

uint32_t gpioNum1_tc2 = 44;
uint32_t gpioNum2_tc2 = 45;

void user_isr1_tc2()
{
    /* Set pin */
    GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, gpioNum1_tc2);
    count_isr1_tc2++;

    /* Some floating point arithmetic*/
    float_arith_isr();

    /* Clear interrupt source */
    HW_WR_REG16(CSL_CONTROLSS_G0_EPWM10_U_BASE + CSL_EPWM_ETCLR, CSL_EPWM_ETCLR_INT_MASK);

    /* Clear pin */
    GPIO_pinWriteLow(CSL_GPIO0_U_BASE, gpioNum1_tc2);
}

uint32_t gEpwmIntrNum_1_tc2;
uint32_t gEpwm_vim_sts_addr_1_tc2;
uint32_t gEpwm_vim_sts_clr_mask_1_tc2;
void *epwmArgs_1_tc2;

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void irq_handler1_tc2(void)
{
    ISR_CALL_PULSE_FLOAT_REENTRANT(user_isr1_tc2, epwmArgs_1_tc2, gEpwmIntrNum_1_tc2, gEpwm_vim_sts_addr_1_tc2, gEpwm_vim_sts_clr_mask_1_tc2, gVimBaseAddr);
}

void user_isr2_tc2()
{
    /* Set pin */
    GPIO_pinWriteHigh(CSL_GPIO0_U_BASE, gpioNum2_tc2);
    count_isr2_tc2++;

    /* Some floating point arithmetic*/
    float_arith_isr();

    /* Clear interrupt source */
    HW_WR_REG16(CSL_CONTROLSS_G0_EPWM11_U_BASE + CSL_EPWM_ETCLR, CSL_EPWM_ETCLR_INT_MASK);

    /* Clear pin */
    GPIO_pinWriteLow(CSL_GPIO0_U_BASE, gpioNum2_tc2);
}

uint32_t gEpwmIntrNum_2_tc2;
uint32_t gEpwm_vim_sts_addr_2_tc2;
uint32_t gEpwm_vim_sts_clr_mask_2_tc2;
void *epwmArgs_2_tc2;

static __attribute__((__section__(".controlfnc"), noinline, naked, target("arm"), aligned(4))) void irq_handler2_tc2(void)
{
    ISR_CALL_PULSE_FLOAT_REENTRANT(user_isr2_tc2, epwmArgs_2_tc2, gEpwmIntrNum_2_tc2, gEpwm_vim_sts_addr_2_tc2, gEpwm_vim_sts_clr_mask_2_tc2, gVimBaseAddr);
}

int32_t test_case_2()
{
    DebugP_log("\r\nConfiguring source of interrupts... ");
    uint32_t intNum1 = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    uint32_t intNum2 = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_1;
    util_epwm_periodic_interrupt_config(CSL_CONTROLSS_G0_EPWM10_U_BASE, 1000);
    util_epwm_periodic_interrupt_config(CSL_CONTROLSS_G0_EPWM11_U_BASE, 999);
    util_epwm_en_epwm_clocks(0xFFFFFFFF);
    util_intxbar_g0_config(0, INT_XBAR_EPWM10_INT);
    util_intxbar_g0_config(1, INT_XBAR_EPWM11_INT);
    DebugP_log("Done.");

    DebugP_log("\r\nRegistering interrupt handlers in R5F VIM... ");
    count_isr1_tc2 = 0;
    count_isr2_tc2 = 0;
    gEpwmIntrNum_1_tc2 = intNum1;
    gEpwm_vim_sts_addr_1_tc2 = gVimBaseAddr +  (0x404u + (((intNum1)>> 5) & 0xFu) * 0x20u);
    gEpwm_vim_sts_clr_mask_1_tc2 = 0x1u << ((intNum1) & 0x1Fu);
    epwmArgs_1_tc2 = NULL;
    util_vim_register_irq(intNum1, 4, (uintptr_t)&irq_handler1_tc2);
    gEpwmIntrNum_2_tc2 = intNum2;
    gEpwm_vim_sts_addr_2_tc2 = gVimBaseAddr +  (0x404u + (((intNum2)>> 5) & 0xFu) * 0x20u);
    gEpwm_vim_sts_clr_mask_2_tc2 = 0x1u << ((intNum2) & 0x1Fu);
    epwmArgs_2_tc2 = NULL;
    util_vim_register_irq(intNum2, 3, (uintptr_t)&irq_handler2_tc2);
    DebugP_log("Done.");

    DebugP_log("\r\nConfiguring pins to use in interrupt routine... ");
    util_iomux_config_for_gpio(gpioNum1_tc2);
    GPIO_setDirMode(CSL_GPIO0_U_BASE, gpioNum1_tc2, 0);
    util_iomux_config_for_gpio(gpioNum2_tc2);
    GPIO_setDirMode(CSL_GPIO0_U_BASE, gpioNum2_tc2, 0);
    DebugP_log("Done.");

    DebugP_log("\r\nPlease observe pins (GPIO %d and %d) ", gpioNum1_tc2, gpioNum2_tc2);
    DebugP_log("\r\n   If pin %d high -> ISR 1 running", gpioNum1_tc2);
    DebugP_log("\r\n   If pin %d high -> ISR 2 running", gpioNum2_tc2);
    DebugP_log("\r\nWaiting for few seconds... ");
    uint32_t i;
    for(i=0;i<200000000;i++)
    {
        float_arith_bg();
    }

    DebugP_log("\r\nDe-initializations... ");
    GPIO_setDirMode(CSL_GPIO0_U_BASE, gpioNum1_tc2, 1);
    GPIO_setDirMode(CSL_GPIO0_U_BASE, gpioNum2_tc2, 1);
    util_vim_deregister_irq(intNum1);
    util_vim_deregister_irq(intNum2);
    util_intxbar_g0_config(0, 0);
    util_intxbar_g0_config(1, 0);
    util_epwm_en_epwm_clocks(0x0);
    util_epwm_all_deInit();
    DebugP_log("Done.");

    return 0;
}


void dpl_low_latency_interrupt_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("DPL Low latency interrupt example started...\r\n");

    DebugP_log("\r\nInterrupt Latency test...\r\n");
    test_case_1();

    DebugP_log("\r\n\nNesting and float point context save test...\r\n");
    test_case_2();

    DebugP_log("\r\n\nAll tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

