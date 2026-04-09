/*
 *  Copyright (C) 2018-2025 Texas Instruments Incorporated
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
 *  \file  i2c_dma_driver.c
 *  \brief I2C DMA transfer orchestration for the i2c_led_blink_dma example.
 *
 *  This file handles the I2C hardware sequencing for DMA transfers:
 *  - Claiming the I2C bus (I2C_lld_setBusy)
 *  - Enabling only error/stop interrupts (NOT TX/RX-ready — DMA handles data)
 *  - Configuring EDMA via the shared I2C_DmaChConfig utility
 *  - Asserting the I2C START condition
 *  - Waiting for EDMA completion semaphore
 *
 *  EDMA channel allocation and ISR callbacks live in i2c_dma.c (shared utility).
 */

#include <stdint.h>
#include <drivers/hw_include/soc_config.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/i2c/v1/lld/i2c_lld.h>
#include <drivers/i2c/v1/i2c_dma.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */

/* I2C1 base address — must match the I2C instance configured in syscfg */
#define I2C_LED_BLINK_BASE_ADDR         (CSL_I2C1_U_BASE)

/* EDMA channel numbers — must match the XBAR instances in example.syscfg */
#define I2C_DMA_TX_CHANNEL              (DMA_TRIG_XBAR_EDMA_MODULE_0)
#define I2C_DMA_RX_CHANNEL              (DMA_TRIG_XBAR_EDMA_MODULE_1)

/* ========================================================================== */
/*                         Module-level DMA config object                     */
/* ========================================================================== */

/* Single config object covering both TX and RX directions */
static I2C_DmaChConfig gI2cDma;

/* ========================================================================== */
/*                         Public API                                         */
/* ========================================================================== */

uint8_t Board_getSocLedDeviceAddr(void)
{
    return 0x61U;
}

void I2C_dmaSetup(void)
{
    gI2cDma.i2cTxRegAddr = (uint32_t)(I2C_LED_BLINK_BASE_ADDR + CSL_I2C_ICDXR);
    gI2cDma.i2cRxRegAddr = (uint32_t)(I2C_LED_BLINK_BASE_ADDR + CSL_I2C_ICDRR);
    gI2cDma.edmaTxChNum  = I2C_DMA_TX_CHANNEL;
    gI2cDma.edmaRxChNum  = I2C_DMA_RX_CHANNEL;
    gI2cDma.edmaHandle   = gEdmaHandle[0];
    I2C_dmaOpen(&gI2cDma);
}

void I2C_dmaCleanup(void)
{
    I2C_dmaClose(&gI2cDma);
}

int32_t I2C_lld_writeDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams)
{
    int32_t        status = I2C_STS_SUCCESS;
    I2CLLD_Object *object = NULL;
    uint32_t       xsa;

    if ((handle == NULL) || (extendedParams == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    object = (I2CLLD_Object *)handle;

    /* Determine address width */
    xsa = (extendedParams->expandSA == true) ?
          I2C_CFG_CMD_10BIT_ADDRESS : I2C_CFG_CMD_7BIT_ADDRESS;

    /* Zero ISR byte counters — DMA owns data movement, not the ISR */
    object->writeBufIdx   = NULL;
    object->writeCountIdx = 0U;
    object->readBufIdx    = NULL;
    object->readCountIdx  = 0U;

    /* Configure EDMA TX channel for this transfer */
    I2C_dmaConfigTx(&gI2cDma, extendedParams->buffer, extendedParams->size);

    /* Clear any pending interrupts */
    I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

    /* Set target device address */
    I2CControllerTargetAddrSet(object->baseAddr, extendedParams->deviceAddress);

    /* Enable only error and stop interrupts — DMA handles data movement */
    I2CControllerIntEnableEx(object->baseAddr,
                             I2C_INT_MASK_ARBITRATION_LOST |
                             I2C_INT_MASK_NO_ACK           |
                             I2C_INT_MASK_STOP_CONDITION);

    /* Set byte count and configure controller for TX mode */
    I2CSetDataCount(object->baseAddr, extendedParams->size);
    I2CControllerControl(object->baseAddr,
                         I2C_CFG_MASK_TX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                         I2C_CFG_CMD_TX  | I2C_CFG_CMD_REPEAT_MODE_OFF | xsa);

    /* Assert STOP after write (no following read in this call) */
    I2CControllerControl(object->baseAddr,
                         I2C_CFG_MASK_STOP | I2C_CFG_MASK_XA,
                         I2C_CFG_CMD_STOP  | xsa);

    /* Assert START — EDMA fires on the first TX-ready event */
    I2CControllerStart(object->baseAddr);

    /* Wait for EDMA TX completion */
    SemaphoreP_pend(&gI2cDma.txDoneSem, SystemP_WAIT_FOREVER);

    return status;
}

int32_t I2C_lld_readDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams)
{
    int32_t        status = I2C_STS_SUCCESS;
    I2CLLD_Object *object = NULL;
    uint32_t       xsa;

    if ((handle == NULL) || (extendedParams == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    object = (I2CLLD_Object *)handle;

    /* Determine address width */
    xsa = (extendedParams->expandSA == true) ?
          I2C_CFG_CMD_10BIT_ADDRESS : I2C_CFG_CMD_7BIT_ADDRESS;

    /* Zero ISR byte counters — DMA owns data movement, not the ISR */
    object->writeBufIdx   = NULL;
    object->writeCountIdx = 0U;
    object->readBufIdx    = NULL;
    object->readCountIdx  = 0U;

    /* Configure EDMA RX channel for this transfer */
    I2C_dmaConfigRx(&gI2cDma, extendedParams->buffer, extendedParams->size);

    /* Clear any pending interrupts */
    I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

    /* Set target device address */
    I2CControllerTargetAddrSet(object->baseAddr, extendedParams->deviceAddress);

    /* Enable only error and stop interrupts — DMA handles data movement */
    I2CControllerIntEnableEx(object->baseAddr,
                             I2C_INT_MASK_ARBITRATION_LOST |
                             I2C_INT_MASK_NO_ACK           |
                             I2C_INT_MASK_STOP_CONDITION);

    /* Set byte count and configure controller for RX mode with stop */
    I2CSetDataCount(object->baseAddr, extendedParams->size);
    I2CControllerControl(object->baseAddr,
                         I2C_CFG_MASK_RX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                         I2C_CFG_CMD_RX  | I2C_CFG_CMD_REPEAT_MODE_OFF | xsa);

    /* Assert START — EDMA fires on the first RX-ready event */
    I2CControllerStart(object->baseAddr);

    /* Wait for EDMA RX completion */
    SemaphoreP_pend(&gI2cDma.rxDoneSem, SystemP_WAIT_FOREVER);

    return status;
}
