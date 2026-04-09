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
 *  \brief I2C repeated-start DMA transfer orchestration.
 *
 *  Controller core (r5fss0-0, I2C1):
 *    I2C_ctrlDmaSetup()         – allocates TX (MODULE_0) and RX (MODULE_1) channels
 *    I2C_ctrlDmaCleanup()       – frees them
 *    I2C_lld_repeatedStartDMA() – full controller write-then-read sequence:
 *      1. Configure EDMA TX, assert START with REPEAT_MODE_ON
 *      2. Pend TX semaphore
 *      3. Configure EDMA RX, assert repeated START
 *      4. Pend RX semaphore
 *      5. Assert STOP
 *
 *  Target core (r5fss0-1, I2C3):
 *    I2C_tgtDmaSetup()          – allocates TX (MODULE_40) and RX (MODULE_41) channels
 *    I2C_tgtDmaCleanup()        – frees them
 *    I2C_tgtDmaArm()            – arms the target hardware and returns immediately:
 *      1. Configure EDMA RX
 *      2. I2CTargetEnable()
 *      3. I2CModeControl(REPEAT_MODE_ON)
 *      Caller signals the IPC barrier so the controller knows the target is ready.
 *    I2C_lld_tgtWaitTransfer()  – waits for both DMA phases to complete:
 *      1. Pend RX semaphore (controller write phase)
 *      2. Build TX response based on received register address
 *      3. Configure EDMA TX, pend TX semaphore (controller read phase)
 *      4. Set handle idle
 *
 *  Splitting arm and wait allows the application layer to insert an IPC barrier
 *  between I2C_tgtDmaArm() and I2C_lld_tgtWaitTransfer(), guaranteeing the
 *  target is fully armed before the controller issues I2CControllerStart().
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

#define I2C_CTRL_BASE_ADDR          (CSL_I2C1_U_BASE)
#define I2C_TGT_BASE_ADDR           (CSL_I2C3_U_BASE)

/*
 * Controller (r5fss0-0):
 *   DMA_TRIG_XBAR_I2C1_TX → MODULE_0  (CH0 in syscfg)
 *   DMA_TRIG_XBAR_I2C1_RX → MODULE_1  (CH1 in syscfg)
 *
 * Target (r5fss0-1):
 *   DMA_TRIG_XBAR_I2C3_TX → MODULE_40 (CH0 in syscfg)
 *   DMA_TRIG_XBAR_I2C3_RX → MODULE_41 (CH1 in syscfg)
 */
#define I2C_CTRL_DMA_TX_CHANNEL     (DMA_TRIG_XBAR_EDMA_MODULE_0)
#define I2C_CTRL_DMA_RX_CHANNEL     (DMA_TRIG_XBAR_EDMA_MODULE_1)
#define I2C_TGT_DMA_TX_CHANNEL      (DMA_TRIG_XBAR_EDMA_MODULE_40)
#define I2C_TGT_DMA_RX_CHANNEL      (DMA_TRIG_XBAR_EDMA_MODULE_41)

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

void I2C_ctrlDmaSetup(void)
{
    gI2cCtrlDma.i2cTxRegAddr = (uint32_t)(I2C_CTRL_BASE_ADDR + CSL_I2C_ICDXR);
    gI2cCtrlDma.i2cRxRegAddr = (uint32_t)(I2C_CTRL_BASE_ADDR + CSL_I2C_ICDRR);
    gI2cCtrlDma.edmaTxChNum  = I2C_CTRL_DMA_TX_CHANNEL;
    gI2cCtrlDma.edmaRxChNum  = I2C_CTRL_DMA_RX_CHANNEL;
    gI2cCtrlDma.edmaHandle   = gEdmaHandle[0];
    I2C_dmaOpen(&gI2cCtrlDma);
}

void I2C_ctrlDmaCleanup(void)
{
    I2C_dmaClose(&gI2cCtrlDma);
}

void I2C_tgtDmaSetup(void)
{
    gI2cTgtDma.i2cTxRegAddr = (uint32_t)(I2C_TGT_BASE_ADDR + CSL_I2C_ICDXR);
    gI2cTgtDma.i2cRxRegAddr = (uint32_t)(I2C_TGT_BASE_ADDR + CSL_I2C_ICDRR);
    gI2cTgtDma.edmaTxChNum  = I2C_TGT_DMA_TX_CHANNEL;
    gI2cTgtDma.edmaRxChNum  = I2C_TGT_DMA_RX_CHANNEL;
    gI2cTgtDma.edmaHandle   = gEdmaHandle[0];
    I2C_dmaOpen(&gI2cTgtDma);
}

void I2C_tgtDmaCleanup(void)
{
    I2C_dmaClose(&gI2cTgtDma);
}

/* ========================================================================== */
/*                         Controller: repeated-start DMA transfer            */
/* ========================================================================== */

/**
 * \brief  Perform a register-read with repeated start using DMA (controller).
 *
 *  Sends writeCount bytes (the register address) with REPEAT_MODE_ON so no
 *  STOP is generated after the write.  Then issues a repeated START and reads
 *  readCount bytes.  A STOP is generated after the last read byte.
 *
 *  Both phases use EDMA.  TX and RX semaphores are pended sequentially.
 *  The caller must have already called I2C_ctrlDmaSetup().
 *
 *  \param handle  I2C LLD handle for the controller (I2C1)
 *  \param msg     Message descriptor.  msg->txn[0] must have:
 *                   writeBuf / writeCount — register address byte(s)
 *                   readBuf  / readCount  — buffer for register data
 *
 *  \return I2C_STS_SUCCESS, I2C_STS_ERR_BUS_BUSY, or I2C_STS_ERR_INVALID_PARAM
 */
int32_t I2C_lld_repeatedStartDMA(I2CLLD_Handle handle, I2CLLD_Message *msg)
{
    int32_t         status = I2C_STS_SUCCESS;
    I2CLLD_Object  *object = NULL;
    I2C_DmaChConfig *dma   = &gI2cCtrlDma;
    uint32_t        xsa;
    uint32_t        writeCount, readCount;
    uint8_t        *writeBuf, *readBuf;

    if ((handle == NULL) || (msg == NULL) || (msg->txn == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    writeCount = (uint32_t)msg->txn[0].writeCount;
    readCount  = (uint32_t)msg->txn[0].readCount;
    writeBuf   = (uint8_t *)msg->txn[0].writeBuf;
    readBuf    = (uint8_t *)msg->txn[0].readBuf;

    if ((writeCount == 0U) || (readCount == 0U) ||
        (writeBuf == NULL) || (readBuf == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    object = (I2CLLD_Object *)handle;

    xsa = (msg->expandSA == true) ?
          I2C_CFG_CMD_10BIT_ADDRESS : I2C_CFG_CMD_7BIT_ADDRESS;

    /* Zero ISR byte counters — DMA owns data movement, not the ISR */
    object->writeBufIdx   = NULL;
    object->writeCountIdx = 0U;
    object->readBufIdx    = NULL;
    object->readCountIdx  = 0U;
    object->intStatusErr  = 0U;

    /* Clear all pending interrupts */
    I2CControllerIntClearEx(object->baseAddr, I2C_ALL_INTS);

    /* ------------------------------------------------------------------ */
    /* WRITE phase — send register address, hold bus (no STOP)            */
    /* ------------------------------------------------------------------ */

    I2CControllerTargetAddrSet(object->baseAddr, msg->targetAddress);

    /* Enable error interrupts only.  Do NOT enable STOP during the write
     * phase — if STOP is enabled, the ISR fires after EDMA drains the last
     * TX byte (SCD asserts when the data count reaches zero even with
     * REPEAT_MODE_ON), which calls I2C_lld_completeCurrTransfer and sets
     * the handle idle before the repeated-start read phase can run. */
    I2CControllerIntEnableEx(object->baseAddr,
                             I2C_INT_MASK_ARBITRATION_LOST |
                             I2C_INT_MASK_NO_ACK);

    /* Configure EDMA TX — fires on I2C1 TX-ready events */
    I2C_dmaConfigTx(dma, writeBuf, writeCount);

    I2CSetDataCount(object->baseAddr, writeCount);

    /* TX + REPEAT_MODE_ON — suppresses STOP so the bus stays active */
    I2CControllerControl(object->baseAddr,
                         I2C_CFG_MASK_TX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                         I2C_CFG_CMD_TX  | I2C_CFG_CMD_REPEAT_MODE_ON | xsa);

    /* START — EDMA fires on the first TX-ready event */
    I2CControllerStart(object->baseAddr);

    /* Wait for all write bytes to be consumed by EDMA */
    SemaphoreP_pend(&dma->txDoneSem, SystemP_WAIT_FOREVER);

    /* Poll ARDY — wait for the last TX byte to be fully shifted out on the
     * wire.  txDoneSem posts when EDMA writes the last byte to ICDXR, but
     * the I2C hardware may not have shifted it yet.  ARDY (ICSTR[2]) asserts
     * once the shift register is empty and the hardware is ready to accept a
     * new START condition.  Without this poll the repeated START fires too
     * early and the target never sees a valid address phase.
     *
     * I2CControllerIntStatusEx() is static in i2c_v1_lld.c so direct ICSTR
     * access is used for the poll.  I2CControllerIntClearEx() is public and
     * writes to ICSTR (W1C), so it is used for the clear. */
    {
        CSL_I2cRegsOvly i2cRegs = (CSL_I2cRegsOvly)object->baseAddr;
        while ((i2cRegs->ICSTR & CSL_I2C_ICSTR_ARDY_MASK) == 0U) { }
    }
    I2CControllerIntClearEx(object->baseAddr, CSL_I2C_ICSTR_ARDY_MASK);
    /* ------------------------------------------------------------------ */
    /* READ phase — repeated START, read register data                    */
    /* ------------------------------------------------------------------ */

    /* Configure EDMA RX — fires on I2C1 RX-ready events */
    I2C_dmaConfigRx(dma, readBuf, readCount);

    I2CSetDataCount(object->baseAddr, readCount);

    /* Switch to RX mode.  Keep REPEAT_MODE_ON so STP is not asserted here —
     * STOP is set after the last byte is clocked in (after rxDoneSem). */
    I2CControllerControl(object->baseAddr,
                         I2C_CFG_MASK_RX | I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                         I2C_CFG_CMD_RX  | I2C_CFG_CMD_REPEAT_MODE_ON | xsa);

    /* Repeated START — re-addresses the target in read direction */
    I2CControllerStart(object->baseAddr);

    /* Wait for all read bytes */
    SemaphoreP_pend(&dma->rxDoneSem, SystemP_WAIT_FOREVER);

    /* Generate STOP after all RX bytes have been clocked in */
    I2CControllerControl(object->baseAddr,
                         I2C_CFG_MASK_STOP,
                         I2C_CFG_CMD_STOP);

    return status;
}

/* ========================================================================== */
/*                         Target: arm + wait (split for IPC barrier)         */
/* ========================================================================== */

/**
 * \brief  Arm the target hardware for a repeated-start transfer and return.
 *
 *  Configures EDMA RX, enables the target, and sets REPEAT_MODE so the target
 *  stays addressed after the controller's write phase.  Returns immediately
 *  without pending any semaphore.
 *
 *  The caller must call IpcNotify_syncAll() after this function returns to
 *  signal the controller that the target is fully armed, then call
 *  I2C_lld_tgtWaitTransfer() to wait for both DMA phases to complete.
 *
 *  \param handle  I2C LLD handle for the target (I2C3)
 *  \param msg     Message descriptor.  msg->txn[0] must have:
 *                   readBuf  / readCount  — buffer to receive register address
 *                   writeBuf / writeCount — buffer for register data response
 *
 *  \return I2C_STS_SUCCESS, I2C_STS_ERR_BUS_BUSY, or I2C_STS_ERR_INVALID_PARAM
 */
int32_t I2C_tgtDmaArm(I2CLLD_Handle handle, I2CLLD_Message *msg)
{
    int32_t         status = I2C_STS_SUCCESS;
    I2CLLD_Object  *object = NULL;
    I2C_DmaChConfig *dma   = &gI2cTgtDma;
    uint32_t        xsa;
    uint32_t        readCount;
    uint8_t        *readBuf;

    if ((handle == NULL) || (msg == NULL) || (msg->txn == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    readCount = (uint32_t)msg->txn[0].readCount;
    readBuf   = (uint8_t *)msg->txn[0].readBuf;

    if ((readCount == 0U) || (readBuf == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    object = (I2CLLD_Object *)handle;

    xsa = (msg->expandSA == true) ?
          I2C_CFG_CMD_10BIT_ADDRESS : I2C_CFG_CMD_7BIT_ADDRESS;

    /* Zero ISR byte counters — DMA owns data movement, not the ISR */
    object->writeBufIdx   = NULL;
    object->writeCountIdx = 0U;
    object->readBufIdx    = NULL;
    object->readCountIdx  = 0U;
    object->intStatusErr  = 0U;

    /* Enable AAS interrupt — required for the target hardware to recognise
     * the controller's address phase and start gating RX-ready events.
     * Matches the sequence in the working I2C_lld_targetReadDMA(). */
    I2CControllerIntEnableEx(object->baseAddr, I2C_INT_MASK_ADRR_TARGET | I2C_INT_MASK_ADRR_READY_ACESS);

    /* Data count must be 0 in target mode — the actual byte count is
     * determined by how many events EDMA services, not by this register. */
    I2CSetDataCount(object->baseAddr, 0U);

    /* Configure EDMA RX — fires on I2C3 RX-ready events */
    I2C_dmaConfigRx(dma, readBuf, readCount);

    /* Enable target mode.
     * I2CTargetEnable() writes ICMDR = FREE_MASK (full register overwrite),
     * so REPEAT_MODE must be set AFTER this call, not before. */
    I2CTargetEnable(object->baseAddr);

    /* Set REPEAT_MODE so the target stays addressed after the write phase
     * and remains active when the controller issues the repeated START.
     * Also set XA to match the controller's address-mode configuration. */
    I2CModeControl(object->baseAddr,
                   I2C_CFG_MASK_REPEAT_MODE | I2C_CFG_MASK_XA,
                   I2C_CFG_CMD_REPEAT_MODE_ON | xsa);

    /* Return to caller — do NOT pend semaphore here.
     * The caller signals the IPC barrier so the controller knows the target
     * is armed, then calls I2C_lld_tgtWaitTransfer(). */
    return status;
}

/**
 * \brief  Wait for both DMA phases of a repeated-start transfer (target).
 *
 *  Must be called after I2C_tgtDmaArm() and after the IPC barrier that
 *  synchronises with the controller.
 *
 *  Phase 1 (controller write → target receive):
 *    Pends rxDoneSem.  On completion rxBuf[0] holds the register address
 *    sent by the controller.
 *
 *  Phase 2 (target transmit → controller read):
 *    Builds the response: txBuf[i] = reg_addr + i.
 *    Configures EDMA TX and pends txDoneSem.
 *
 *  \param handle  I2C LLD handle for the target (I2C3)
 *  \param msg     Same descriptor passed to I2C_tgtDmaArm()
 *
 *  \return I2C_STS_SUCCESS or I2C_STS_ERR_INVALID_PARAM
 */
int32_t I2C_lld_tgtWaitTransfer(I2CLLD_Handle handle, I2CLLD_Message *msg)
{
    I2C_DmaChConfig *dma = &gI2cTgtDma;
    uint32_t         writeCount, i;
    uint8_t         *writeBuf, *readBuf;
    uint8_t          regAddr;

    if ((handle == NULL) || (msg == NULL) || (msg->txn == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    writeCount = (uint32_t)msg->txn[0].writeCount;
    writeBuf   = (uint8_t *)msg->txn[0].writeBuf;
    readBuf    = (uint8_t *)msg->txn[0].readBuf;

    if ((writeCount == 0U) || (writeBuf == NULL) || (readBuf == NULL))
    {
        return I2C_STS_ERR_INVALID_PARAM;
    }

    /* ------------------------------------------------------------------ */
    /* Phase 1: receive register address from controller                  */
    /* ------------------------------------------------------------------ */

    SemaphoreP_pend(&dma->rxDoneSem, SystemP_WAIT_FOREVER);

    /* rxBuf[0] now holds the register address the controller requested */
    regAddr = readBuf[0];

    /* ------------------------------------------------------------------ */
    /* Phase 2: transmit register data back to controller                 */
    /* ------------------------------------------------------------------ */

    /* Build response: each byte = reg_addr + byte_index */
    for (i = 0U; i < writeCount; i++)
    {
        writeBuf[i] = regAddr + (uint8_t)i;
    }

    I2CSetDataCount(((I2CLLD_Object *)handle)->baseAddr, writeCount);

    /* Configure EDMA TX — fires on I2C3 TX-ready events triggered by the
     * controller's repeated START */
    I2C_dmaConfigTx(dma, writeBuf, writeCount);

    SemaphoreP_pend(&dma->txDoneSem, SystemP_WAIT_FOREVER);

    return I2C_STS_SUCCESS;
}
