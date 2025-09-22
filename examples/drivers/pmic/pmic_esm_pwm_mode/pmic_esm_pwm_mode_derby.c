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

/**
 *  \file pmic_esm.c
 *
 *  \brief This is a PMIC ESM example in pwm mode where, in the 
 *         first instance, ESM errors are configured to generate an 
 *         interrupt, and in the second instance, the MCU is reset 
 *         when error is detected in PWM signal on pmic esm_in pin.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>
#include <sdl/esm/v2/sdl_esm.h>
#include <sdl/dpl/sdl_dpl.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PMIC_REG_STATE_LOCK         (1U)
#define PMIC_NO_OF_ESM_ERRORS       (2U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                        External Function Declarations                      */
/* ========================================================================== */

extern void Board_gpioInit(void);
extern void Board_gpioDeinit(void);
extern uint32_t Board_getGpioIntrNum(void);

/* ========================================================================== */
/*                         Internal Function Declarations                     */
/* ========================================================================== */

static int32_t PMICApp_gpioIntConfigure();
static void PMICApp_esmPwmMode(Pmic_CoreHandle_t *coreHandle);
static int32_t PMICApp_setEsmIn(Pmic_CoreHandle_t *handle);
static int32_t PMICApp_setGpoNint(Pmic_CoreHandle_t *handle);
static void PMICApp_wait_ms(uint16_t milliseconds);
static void PMICApp_gpioISR(void *args);
void* Sdl_addrTranslate(uint64_t addr, uint32_t size);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* The address of GPIO instance used for receiving the interrupt from PMIC */
uint32_t            gGpioBaseAddr = GPIO_PMIC_INT_BASE_ADDR;

/* HwiP object for GPIO interrupt */
HwiP_Object         gGpioHwiObject;

/* Semaphore object to notify PMIC GPIO interrupt has occurred */
static SemaphoreP_Object gGpioISRDoneSem;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


void pmic_esm_pwm_mode_main(void *args)
{
    Pmic_CoreHandle_t* handle;
    int32_t status = PMIC_ST_SUCCESS;

    SDL_DPL_Interface dpl_interface =
    {
        .addrTranslate = (pSDL_DPL_AddrTranslateFunction) Sdl_addrTranslate
    };

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* PMIC interface handle initialized by PMIC_open */
    handle = PMIC_getCoreHandle(CONFIG_PMIC0);
    DebugP_assert(NULL != handle);

    DebugP_log("Starting ESM pwm mode example !!\r\n");

    // Unlock PMIC registers
    status = Pmic_unlockRegs(handle);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    /* Clear all errors statuses*/
    if(PMIC_ST_SUCCESS == status)
    {
        status = Pmic_irqClrAllFlags(handle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Configure the PMIC ESM_IN pin */
        status = PMICApp_setEsmIn(handle);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        /* Configure the PMIC NINT pin to genrate interrupt */
        status = PMICApp_setGpoNint(handle);
    }
       
    if(PMIC_ST_SUCCESS == status)
    {
        /* Configure the GPIO pin to receive interrupt from PMIC */
        status = PMICApp_gpioIntConfigure();
    }

    if(PMIC_ST_SUCCESS == status)
    {
        status = SDL_DPL_init(&dpl_interface);
        /*Set MCU ESM Pin in PWM mode*/
        status += SDL_ESM_setPinOutMode(SDL_ESM_INST_MAIN_ESM0, SDL_ESM_PWM_PINOUT);
    }

    if(PMIC_ST_SUCCESS == status)
    {
        PMICApp_esmPwmMode(handle);
    }

    Board_driversClose();
    Drivers_close();
    return;
}

void* Sdl_addrTranslate(uint64_t addr, uint32_t size)
{
    uint32_t transAddr = (uint32_t)(-1);

    transAddr = (uint32_t)AddrTranslateP_getLocalAddr(addr);

    return (void *)transAddr;
}

static int32_t PMICApp_setEsmIn(Pmic_CoreHandle_t *pmicHandle)
{
    uint32_t status = PMIC_ST_SUCCESS;

    Pmic_GpioCfg_t gpiocfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID,
        .functionality = PMIC_GPIO_ESM_INPUT,
    };
    status = Pmic_gpioSetCfg(pmicHandle, PMIC_GPIO, &gpiocfg);
    return status;
}

static int32_t PMICApp_setGpoNint(Pmic_CoreHandle_t *pmicHandle)
{
    uint32_t status = PMIC_ST_SUCCESS;

    Pmic_GpioCfg_t gpiocfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID,
        .functionality = PMIC_NINT_GPI_NINT,
    };
    status = Pmic_gpioSetCfg(pmicHandle, PMIC_NINT_GPI, &gpiocfg);
    return status;
}

static int32_t PMICApp_gpioIntConfigure()
{
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t pinNum, intrNum;
    HwiP_Params hwiPrms;

    Board_gpioInit();

    pinNum          = GPIO_PMIC_INT_PIN;
    intrNum         = Board_getGpioIntrNum();

    /* Address translate */
    gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gGpioBaseAddr);

    /* Register pin interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = &PMICApp_gpioISR;
    hwiPrms.args     = (void *) pinNum;
    /* GPIO interrupt is a pulse type interrupt */
    hwiPrms.isPulse  = TRUE;
    status = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    DebugP_assert(status == PMIC_ST_SUCCESS );
    return status;
}

static void PMICApp_esmPwmMode(Pmic_CoreHandle_t* pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool isEsmInt = FALSE;

    Pmic_EsmCfg_t esmCfg = {
        .validParams =
            (PMIC_ESM_ENABLE_VALID | PMIC_ESM_MODE_VALID |
                PMIC_ESM_ERR_CNT_THR_VALID | PMIC_ESM_DELAY1_VALID |
                PMIC_ESM_DELAY2_VALID),
        .enable = (bool)TRUE,
        .mode = ESM_PWM_MODE,
        .errCntThr = ESM_ERR_CNT_THR_MAX,
        .delay1 = 0xFFU,
        .delay2 = 0xFFU,
    };
    
    Pmic_EsmCfg_t esmCFG_verify = {
        .validParams =
        (PMIC_ESM_ENABLE_VALID | PMIC_ESM_MODE_VALID |
            PMIC_ESM_ERR_CNT_THR_VALID | PMIC_ESM_DELAY1_VALID |
            PMIC_ESM_DELAY2_VALID),
    };

    Pmic_EsmStat_t esmErr = { 
        .validParams = 
        (PMIC_ESM_STATUS_ALL_VALID )
    };

    DebugP_log("\r\n");
    DebugP_log("Interrupt: Setting ESM Delay1 and Delay2 error config to generate Interrupt\r\n");

    /* Configure ESM in pwm mode*/
    status = Pmic_esmSetCfg(pmicHandle, &esmCfg);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    /* Verify ESM Configurations are set correctly*/
    status = Pmic_esmGetCfg(pmicHandle, &esmCFG_verify);
    DebugP_assert(status == PMIC_ST_SUCCESS);
    DebugP_assert(esmCfg.enable == esmCFG_verify.enable);
    DebugP_assert(esmCfg.mode == esmCFG_verify.mode);
    DebugP_assert(esmCfg.errCntThr == esmCFG_verify.errCntThr);
    DebugP_assert(esmCfg.delay1 == esmCFG_verify.delay1);
    DebugP_assert(esmCfg.delay2 == esmCFG_verify.delay2);

    /*Clear all esm error status*/
    status = Pmic_esmClrStat(pmicHandle, &esmErr);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    /*Start the PMIC ESM*/
    status = Pmic_esmStart(pmicHandle);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    status = SemaphoreP_constructBinary(&gGpioISRDoneSem, 0);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    DebugP_log("Generating error from MCU to PMIC\r\n");
     
    /* Set MCU ESM Pin low*/
    status = SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);

    if(status == PMIC_ST_SUCCESS)
    {
        DebugP_log("Error generated from MCU.. Waiting for the PMIC interrupt... \r\n");
        while(!isEsmInt)
        {
            status = SemaphoreP_pend(&gGpioISRDoneSem, SystemP_WAIT_FOREVER);
            if(status == PMIC_ST_SUCCESS)
            {
                /*Interrupt occured clear the interrupt from MCU*/
                DebugP_log("Received interrupt for PMIC ESM error !! \r\n");
                /* Set MCU ESM Pin High*/
                status = SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
                /*Add delay to set error pin to normal mode*/
                ClockP_usleep(200000);
                /*Clear all esm error status*/
                status = Pmic_esmClrStat(pmicHandle, &esmErr);
                DebugP_assert(status == PMIC_ST_SUCCESS);
                DebugP_log("Cleared PMIC ESM error states and ESM error from MCU\r\n");
                isEsmInt = TRUE;
            }
        }
    }

    if(status == PMIC_ST_SUCCESS)
    {
        DebugP_log("\r\n");
        DebugP_log("Reset: Setting ESM Delay2 error config to device transitions to RESET-MCU state\r\n");
        
        DebugP_assert(status == PMIC_ST_SUCCESS);
        DebugP_log("Generating error from MCU to PMIC\r\n");
        status = SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);
        if(status == PMIC_ST_SUCCESS)
        {
            DebugP_log("Error generated from MCU..\r\n");
            DebugP_log("All tests have Passed!! MCU will reset once reset ocuurs from PMIC\r\n");
        }
        while (TRUE)
        {
            ;
        }
    }
}

static void PMICApp_gpioISR(void *args)
{
    uint32_t pinNum = (uint32_t) args;
    uint32_t bankNum =  GPIO_GET_BANK_INDEX(pinNum);
    uint32_t intrStatus, pinMask = GPIO_GET_BANK_BIT_MASK(pinNum);

    /* Get and clear bank interrupt status */
    intrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, bankNum);
    GPIO_clearBankIntrStatus(gGpioBaseAddr, bankNum, intrStatus);
    /* Per pin interrupt handling */
    if(intrStatus & pinMask)
    {
        SemaphoreP_post(&gGpioISRDoneSem);
    }
    return;
}