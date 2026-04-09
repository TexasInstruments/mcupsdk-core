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
 *  \brief I2C DMA transfer orchestration for the i2c_dma_read_write example.
 *
 *  This file handles the I2C hardware sequencing for DMA transfers on both
 *  the controller core (r5fss0-0, I2C1) and the target core (r5fss0-1, I2C3):
 *
 *  Controller (I2C1):
 *  - Allocates EDMA TX channel (MODULE_0) and RX channel (MODULE_1)
 *  - I2C_lld_writeDMA: claim bus, configure EDMA TX, start I2C, pend semaphore
 *  - I2C_lld_readDMA:  claim bus, configure EDMA RX, start I2C, pend semaphore
 *
 *  Target (I2C3):
 *  - Allocates EDMA RX channel (MODULE_40) and TX channel (MODULE_41)
 *  - I2C_lld_targetWriteDMA: claim bus, configure EDMA TX, enable target, pend semaphore
 *  - I2C_lld_targetReadDMA:  claim bus, configure EDMA RX, enable target, pend semaphore
 *
 *  EDMA channel allocation and ISR callbacks live in i2c_dma.c (shared utility).
 *
 *  XBAR and DMA configuration references i2c_led_blink_dma/example.syscfg.
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

/* I2C base addresses — must match the I2C instances in syscfg */
#define I2C_CTRL_BASE_ADDR          (CSL_I2C1_U_BASE)   /* r5fss0-0, controller */
#define I2C_TGT_BASE_ADDR           (CSL_I2C3_U_BASE)   /* r5fss0-1, target     */

/*
 * EDMA channel numbers — must match the XBAR instances configured in syscfg:
 *   r5fss0-0: DMA_TRIG_XBAR_I2C1_TX → MODULE_0,  DMA_TRIG_XBAR_I2C1_RX → MODULE_1
 *   r5fss0-1: DMA_TRIG_XBAR_I2C3_RX → MODULE_40, DMA_TRIG_XBAR_I2C3_TX → MODULE_41
 */
#define I2C_CTRL_DMA_TX_CHANNEL     (DMA_TRIG_XBAR_EDMA_MODULE_0)
#define I2C_CTRL_DMA_RX_CHANNEL     (DMA_TRIG_XBAR_EDMA_MODULE_1)
#define I2C_TGT_DMA_RX_CHANNEL      (DMA_TRIG_XBAR_EDMA_MODULE_40)
#define I2C_TGT_DMA_TX_CHANNEL      (DMA_TRIG_XBAR_EDMA_MODULE_41)

/* ========================================================================== */
/*                         External Declarations                              */
/* ========================================================================== */

extern uint8_t txBuf[];
extern uint8_t rxBuf[];

/* ========================================================================== */
/*                         Module-level DMA config objects                    */
/* ========================================================================== */

/*
 * One config object per core — each covers both TX and RX directions.
 * Only the object matching the compiled core is populated at runtime.
 */
static I2C_DmaChConfig gI2cCtrlDma;
static I2C_DmaChConfig gI2cTgtDma;

/* ========================================================================== */
/*                         Public API — DMA resource management               */
/* ========================================================================== */

/**
 * \brief  Allocate EDMA channels for the controller core (I2C1, r5fss0-0).
 *
 *  TX channel (MODULE_0): source = txBuf, destination = I2C1 TX register
 *  RX channel (MODULE_1): source = I2C1 RX register, destination = rxBuf
 */
void I2C_ctrlDmaSetup(void)
{
    gI2cCtrlDma.i2cTxRegAddr = (uint32_t)(I2C_CTRL_BASE_ADDR + CSL_I2C_ICDXR);
    gI2cCtrlDma.i2cRxRegAddr = (uint32_t)(I2C_CTRL_BASE_ADDR + CSL_I2C_ICDRR);
    gI2cCtrlDma.edmaTxChNum  = I2C_CTRL_DMA_TX_CHANNEL;
    gI2cCtrlDma.edmaRxChNum  = I2C_CTRL_DMA_RX_CHANNEL;
    gI2cCtrlDma.edmaHandle   = gEdmaHandle[0];
    I2C_dmaOpen(&gI2cCtrlDma);
}

/**
 * \brief  Free EDMA channels for the controller core.
 */
void I2C_ctrlDmaCleanup(void)
{
    I2C_dmaClose(&gI2cCtrlDma);
}

/**
 * \brief  Allocate EDMA channels for the target core (I2C3, r5fss0-1).
 *
 *  RX channel (MODULE_40): source = I2C3 RX register, destination = rxBuf
 *  TX channel (MODULE_41): source = txBuf, destination = I2C3 TX register
 */
void I2C_tgtDmaSetup(void)
{
    gI2cTgtDma.i2cTxRegAddr = (uint32_t)(I2C_TGT_BASE_ADDR + CSL_I2C_ICDXR);
    gI2cTgtDma.i2cRxRegAddr = (uint32_t)(I2C_TGT_BASE_ADDR + CSL_I2C_ICDRR);
    gI2cTgtDma.edmaTxChNum  = I2C_TGT_DMA_TX_CHANNEL;
    gI2cTgtDma.edmaRxChNum  = I2C_TGT_DMA_RX_CHANNEL;
    gI2cTgtDma.edmaHandle   = gEdmaHandle[0];
    I2C_dmaOpen(&gI2cTgtDma);
}

/**
 * \brief  Free EDMA channels for the target core.
 */
void I2C_tgtDmaCleanup(void)
{
    I2C_dmaClose(&gI2cTgtDma);
}

/* ========================================================================== */
/*                         Controller DMA transfers (I2C1)                    */
/* ========================================================================== */

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

    xsa = (extendedParams->expandSA == true) ?
          I2C_CFG_CMD_10BIT_ADDRESS : I2C_CFG_CMD_7BIT_ADDRESS;

    /* Zero ISR byte counters — DMA owns data movement, not the ISR */
    object->writeBufIdx   = NULL;
    object->writeCountIdx = 0U;
    object->readBufIdx    = NULL;
    object->readCountIdx  = 0U;

    /* Configure EDMA TX channel for this transfer */
    I2C_dmaConfigTx(&gI2cCtrlDma, extendedParams->buffer, extendedParams->size);

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

    /* Assert STOP after write */
    I2CControllerControl(object->baseAddr,
                         I2C_CFG_MASK_STOP | I2C_CFG_MASK_XA,
                         I2C_CFG_CMD_STOP  | xsa);

    /* Assert START — EDMA fires on the first TX-ready event */

    I2CControllerStart(object->baseAddr);

    /* Wait for EDMA TX completion */
    SemaphoreP_pend(&gI2cCtrlDma.txDoneSem, SystemP_WAIT_FOREVER);

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

    xsa = (extendedParams->expandSA == true) ?
          I2C_CFG_CMD_10BIT_ADDRESS : I2C_CFG_CMD_7BIT_ADDRESS;

    /* Zero ISR byte counters — DMA owns data movement, not the ISR */
    object->writeBufIdx   = NULL;
    object->writeCountIdx = 0U;
    object->readBufIdx    = NULL;
    object->readCountIdx  = 0U;

    /* Configure EDMA RX channel for this transfer */
    I2C_dmaConfigRx(&gI2cCtrlDma, extendedParams->buffer, extendedParams->size);

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
    SemaphoreP_pend(&gI2cCtrlDma.rxDoneSem, SystemP_WAIT_FOREVER);

    return status;
}

/* ========================================================================== */
/*                         Target DMA transfers (I2C3)                        */
/* ========================================================================== */

int32_t I2C_lld_targetReadDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams)
{
    int32_t        status = I2C_STS_SUCCESS;
    I2CLLD_Object *object = NULL;
    uint32_t       xsa;

    if ((handle == NULL) || (extendedParams == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    object = (I2CLLD_Object *)handle;

    xsa = (extendedParams->expandSA == true) ?
          I2C_CFG_CMD_10BIT_ADDRESS : I2C_CFG_CMD_7BIT_ADDRESS;

    /* Zero ISR byte counters — DMA owns data movement, not the ISR */
    object->writeBufIdx   = NULL;
    object->writeCountIdx = 0U;
    object->readBufIdx    = NULL;
    object->readCountIdx  = 0U;

    /* Configure EDMA RX channel for this transfer */
    

    /* Clear any pending interrupts */
    I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_ADRR_TARGET);
    /* Set data count */
    I2CSetDataCount(object->baseAddr, 0);
    /* Enable target mode — DMA will fire on first RX-ready event from controller */
    I2C_dmaConfigRx(&gI2cTgtDma, extendedParams->buffer, extendedParams->size);

    I2CTargetEnable(object->baseAddr);

    I2CModeControl(object->baseAddr, I2C_CFG_MASK_XA, xsa);

    
    /* Wait for EDMA RX completion */
    SemaphoreP_pend(&gI2cTgtDma.rxDoneSem, SystemP_WAIT_FOREVER);

    return status;
}

int32_t I2C_lld_targetWriteDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams)
{
    int32_t        status = I2C_STS_SUCCESS;
    I2CLLD_Object *object = NULL;
    uint32_t       xsa;

    if ((handle == NULL) || (extendedParams == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    object = (I2CLLD_Object *)handle;

    xsa = (extendedParams->expandSA == true) ?
          I2C_CFG_CMD_10BIT_ADDRESS : I2C_CFG_CMD_7BIT_ADDRESS;

    /* Zero ISR byte counters — DMA owns data movement, not the ISR */
    object->writeBufIdx   = NULL;
    object->writeCountIdx = 0U;
    object->readBufIdx    = NULL;
    object->readCountIdx  = 0U;

    /* Configure EDMA TX channel for this transfer */
    I2C_dmaConfigTx(&gI2cTgtDma, extendedParams->buffer, extendedParams->size);

    /* Clear any pending interrupts */
    I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);
    I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_ADRR_TARGET);

    /* Set data count */
    I2CSetDataCount(object->baseAddr, extendedParams->size);

    /* Enable target mode — DMA will fire on first TX-ready event from controller */
    I2CTargetEnable(object->baseAddr);
    I2CModeControl(object->baseAddr, I2C_CFG_MASK_XA, xsa);

    /* Wait for EDMA TX completion */
    SemaphoreP_pend(&gI2cTgtDma.txDoneSem, SystemP_WAIT_FOREVER);

    return status;
}
