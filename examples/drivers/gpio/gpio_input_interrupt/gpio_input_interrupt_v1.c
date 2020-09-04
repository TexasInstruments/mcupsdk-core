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
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This example configures a GPIO pin in input mode
 * and configures it to generate interrupt on  edge.
 * The application waits for 5 key presses, prints the
 * number of times the keys are pressed and exits.
 */

uint32_t            gGpioBaseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;
HwiP_Object         gGpioHwiObject;
volatile uint32_t   gGpioIntrDone = 0;

static void GPIO_portIsrFxn(void *args);
extern void Board_gpioInit(void);
extern void Board_gpioDeinit(void);

void gpio_input_interrupt_main(void *args)
{
    int32_t         retVal;
    uint32_t        pinNum, intrNum;
    uint32_t        waitCount = 5;
    int32_t         status = SystemP_SUCCESS;
    HwiP_Params     hwiPrms;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("GPIO Input Interrupt Test Started ...\r\n");
    pinNum          = GPIO_PUSH_BUTTON_PIN;

    if(GPIO_PUSH_BUTTON_INTR_LEVEL == GPIO_INTR_LEVEL_HIGH)
    {
        /* High-level interrupt number */
        intrNum         = GPIO_PUSH_BUTTON_INTR_HIGH;
    }
    else
    {
        /* Low-level interrupt number */
        intrNum         = GPIO_PUSH_BUTTON_INTR_LOW;
    }

    /* Address translate */
    gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gGpioBaseAddr);
 
    /* Setup GPIO for interrupt generation */
    GPIO_setDirMode(gGpioBaseAddr, pinNum, GPIO_PUSH_BUTTON_DIR);

    status = GPIO_ignoreOrHonorPolarity(gGpioBaseAddr, pinNum, GPIO_PUSH_BUTTON_TRIG_TYPE);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    if((GPIO_PUSH_BUTTON_TRIG_TYPE == GPIO_TRIG_TYPE_FALL_EDGE) || (GPIO_PUSH_BUTTON_TRIG_TYPE == GPIO_TRIG_TYPE_RISE_EDGE))
    {
        status = GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_PUSH_BUTTON_TRIG_TYPE);
        if(status != SystemP_SUCCESS)
        {
             DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",pinNum);
             DebugP_assert(status == SystemP_SUCCESS);
        }
    }

    status = GPIO_markHighLowLevelInterrupt(gGpioBaseAddr, pinNum, GPIO_PUSH_BUTTON_INTR_LEVEL);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }
 
    status = GPIO_clearInterrupt(gGpioBaseAddr, pinNum);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    status = GPIO_enableInterrupt(gGpioBaseAddr, pinNum);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    /* Register pin interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = &GPIO_portIsrFxn;
    hwiPrms.args     = (void *) pinNum;
    retVal = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    DebugP_assert(retVal == SystemP_SUCCESS );

    DebugP_log("Press and release SW%d button on EVM to trigger GPIO interrupt ...\r\n", pinNum);
    while(gGpioIntrDone <= waitCount)
    {
        /* Keep printing the current GPIO value */
        DebugP_log("Key is pressed %d times\r\n", gGpioIntrDone);
        ClockP_sleep(1);
    }

    /* Unregister interrupt */
    status = GPIO_disableInterrupt(gGpioBaseAddr, pinNum);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    status = GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_TRIG_TYPE_NONE);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    status = GPIO_clearInterrupt(gGpioBaseAddr, pinNum);
    if(status != SystemP_SUCCESS)
    {
         DebugP_log("Interrupt not supported!!! for GPIO pin:%d \r\n",pinNum);
         DebugP_assert(status == SystemP_SUCCESS);
    }

    HwiP_destruct(&gGpioHwiObject);

    DebugP_log("GPIO Input Interrupt Test Passed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

static void GPIO_portIsrFxn(void *args)
{
    uint32_t    pendingInterrupt;
    uint32_t    pinNum = (uint32_t) args;

    /* Get the pending GPIO interrupt: */
    pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(gGpioBaseAddr,GPIO_PUSH_BUTTON_INTR_LEVEL);
    GPIO_clearInterrupt(gGpioBaseAddr, pinNum);

    /* Interrupt occured */
    if(pendingInterrupt )
    {
        gGpioIntrDone++;
    }
}
