/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <drivers/pruicss.h>
#include <pru_load_bin.h> // > PRUFirmware array
#include <pru_ipc.h>
#include <board/ioexp/ioexp_tca6424.h>

/**
 *  @brief PRU Core
 *  Wil come from sysconfig
 */
#define PRUICSS_PRUx                PRUICSS_PRU0

#define PRUICSS_PRGM_FLOW_CNTRL_OFFSET  0xFC

/**
 * @brief PRU Firmware Code Sections
 */
#define IDLE_SECTION             0
#define IEP_SECTION              1
#define IPC_SECTION              2
#define ADC_CONFIG_SECTION       3
#define READ_SAMPLES_SECTION     4
#define RESET_SECTION            5
#define CONFIG_ADC0_SMEM_OFFSET  0 // should be received from sysconfig

/** \brief Global PRUICSS Handle */
extern PRUICSS_Handle gPruIcss0Handle;

extern PRU_IPC_Handle gPruIpc0Handle;

static TCA6424_Config  gTCA6424_Config;

extern void PRU_IPC_Isr(void *args);

/**
 * \brief   This function writes the specified core's DRAM memory with sectionID
 *           and creates an interrupt event to PRU
 *
 * \param   handle
 * \param   pruCore
 * \param   sectionId
 * \param   eventNum
 *
 * \return  #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t PRUICSS_goToSection(PRUICSS_Handle handle, uint32_t pruCore, uint32_t sectionId, uint32_t eventNum)
{
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_SUCCESS;

    if (handle != NULL)
    {
        hwAttrs  = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        if (pruCore == PRUICSS_PRU0)
            baseaddr = hwAttrs->pru0DramBase;
        else if (pruCore == PRUICSS_PRU1)
            baseaddr = hwAttrs->pru1DramBase;
        /* RTU_PRU, TX_PRU not yet supported */
        else
            return SystemP_FAILURE;

        /* TODO: decide the final memory and offset */
        CSL_REG32_WR(baseaddr + PRUICSS_PRGM_FLOW_CNTRL_OFFSET, sectionId);
        retVal = PRUICSS_sendEvent(handle, eventNum);
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

static void i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;
    TCA6424_Params_init(&tca6424Params);
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
    uint32_t            ioIndex;

    if(status == SystemP_SUCCESS)
    {
        /* set P12 high which controls CPSW_FET_SEL -> enable PRU1 and PRU0 GPIOs */
        ioIndex = 0x0a;
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);
    }
    TCA6424_close(&gTCA6424_Config);
}

void ADC_init()
{
    int status;
    /* Configure the IO Expander to connect the PRU IOs to HSE */
    i2c_io_expander(NULL);
    /* ----------------------------------------------------------------- */
    /* Program ADC code on PRU Core/s;                                   */
    /* depends on usecase - might have to program multiple cores         */
    /* ----------------------------------------------------------------- */
    /* clear ICSS PRUx data RAM */
    status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRUx));
    DebugP_assert(status != 0);
    status = PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware. Set buffer = write to Pru memory */
    status = PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx), 0,
                       (uint32_t *) PRUFirmware, sizeof(PRUFirmware));
    DebugP_assert(status != 0);

    status = PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Run firmware */
    status = PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);

    /* ----------------------------------------------------------------- */
    /* Initialize IPC between r5f and PRU cores                          */
    /* ----------------------------------------------------------------- */
    PRU_IPC_Params pruIpcparams = {
                             .pruicssHandle = gPruIcss0Handle,
                             .transferCallbackFxn = &PRU_IPC_Isr,
    };
    gPruIpc0Handle = PRU_IPC_open(CONFIG_PRU_IPC0, &pruIpcparams);
    DebugP_assert(gPruIpc0Handle != NULL);
}

void ADC_powerUp()
{
    /* Power up the adaptor board which powers ADC */
    GPIO_setDirMode(ADC_POWER_ENABLE_PIN_1_BASE_ADDR, ADC_POWER_ENABLE_PIN_1_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_pinWriteHigh(ADC_POWER_ENABLE_PIN_1_BASE_ADDR, ADC_POWER_ENABLE_PIN_1_PIN);
    ClockP_sleep(1);
}

void ADC_reset()
{
    /* Set the reset pin LOW (Active low) */
    GPIO_setDirMode(ADC_RESET_PIN_BASE_ADDR, ADC_RESET_PIN_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_pinWriteLow(ADC_RESET_PIN_BASE_ADDR, ADC_RESET_PIN_PIN);
    /* Wait for 1 second */
    ClockP_sleep(1);
    /* Set the reset pin HIGH */
    GPIO_setDirMode(ADC_RESET_PIN_BASE_ADDR, ADC_RESET_PIN_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_pinWriteHigh(ADC_RESET_PIN_BASE_ADDR, ADC_RESET_PIN_PIN);
}

void ADC_configure(uint16_t noOfCommands, uint16_t adcConfigCommands[])
{
    PRUICSS_Handle handle = gPruIcss0Handle;
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;

    DebugP_assert(handle != NULL);
    hwAttrs  = (PRUICSS_HwAttrs const *)handle->hwAttrs;
    baseaddr = hwAttrs->sharedDramBase + CONFIG_ADC0_SMEM_OFFSET;
    CSL_REG16_WR(baseaddr, noOfCommands);
    baseaddr += 2;

    /* Send commands to the SMEM */
    for (uint16_t i = 0; i < noOfCommands; i++)
    {
        CSL_REG16_WR(baseaddr + (i<<1), adcConfigCommands[i]);
    }

    /* Control the PRU to execute particular code sections */
    PRUICSS_goToSection(gPruIcss0Handle, PRUICSS_PRUx, IEP_SECTION, 22);
    ClockP_usleep(1);
    PRUICSS_goToSection(gPruIcss0Handle, PRUICSS_PRUx, IPC_SECTION, 22);
    ClockP_usleep(1);
    PRUICSS_goToSection(gPruIcss0Handle, PRUICSS_PRUx, ADC_CONFIG_SECTION, 22);
    ClockP_usleep(1);
}

void ADC_startConversion()
{
    /* Control the PRU to execute particular code sections */
    PRUICSS_goToSection(gPruIcss0Handle, PRUICSS_PRUx, READ_SAMPLES_SECTION, 22);
    ClockP_usleep(1);
}

void ADC_stopConversion()
{
    PRUICSS_goToSection(gPruIcss0Handle, PRUICSS_PRUx, RESET_SECTION, 22);
    ClockP_usleep(1);
    /* Can disable IEP of PRU from here only */
    PRUICSS_goToSection(gPruIcss0Handle, PRUICSS_PRUx, IDLE_SECTION, 22);
}
