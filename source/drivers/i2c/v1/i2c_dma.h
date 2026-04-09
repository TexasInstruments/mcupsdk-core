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
 *  \file  v1/i2c_dma.h
 *  \brief Application-level EDMA utility for I2C DMA transfers.
 *
 *  This module provides a thin utility layer on top of the EDMA driver to
 *  simplify setting up I2C TX and RX DMA transfers. It is intended to be used
 *  by application or example code — it is NOT part of the I2C LLD driver.
 *
 *  A single I2C_DmaChConfig object holds both TX and RX channel state.
 *
 *  Usage:
 *  1. Declare and zero-initialize an I2C_DmaChConfig.
 *  2. Fill in i2cTxRegAddr, i2cRxRegAddr, edmaTxChNum, edmaRxChNum,
 *     and edmaHandle before calling I2C_dmaOpen().
 *  3. Call I2C_dmaOpen() once — it allocates EDMA resources and registers
 *     interrupts for both TX and RX channels.
 *  4. Before each transfer, call I2C_dmaConfigTx() / I2C_dmaConfigRx() to
 *     program the PaRAM and arm the channel.
 *  5. Pend on txDoneSem / rxDoneSem for transfer completion.
 *  6. Call I2C_dmaClose() to free all resources when done.
 */

#ifndef I2C_DMA_H_
#define I2C_DMA_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <drivers/edma.h>
#include <kernel/dpl/SemaphoreP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief EDMA event queue to use for I2C DMA transfers */
#define I2C_EDMA_EVT_QUEUE_NO       (0U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief  Configuration and runtime state for I2C EDMA TX and RX channels.
 *
 *  A single instance covers both directions. The caller fills in
 *  i2cTxRegAddr, i2cRxRegAddr, edmaTxChNum, edmaRxChNum and edmaHandle
 *  before calling I2C_dmaOpen(). All other fields are managed internally.
 */
typedef struct
{
    /* --- Caller-filled before I2C_dmaOpen() --- */

    uint32_t        i2cTxRegAddr;
    /**< Physical address of the I2C TX data register:
     *   CSL_I2Cx_U_BASE + CSL_I2C_ICDXR */

    uint32_t        i2cRxRegAddr;
    /**< Physical address of the I2C RX data register:
     *   CSL_I2Cx_U_BASE + CSL_I2C_ICDRR */

    uint32_t        edmaTxChNum;
    /**< EDMA TX channel / XBAR module number.
     *   Must match the XBAR instance configured in the syscfg file, e.g.
     *   DMA_TRIG_XBAR_EDMA_MODULE_0. */

    uint32_t        edmaRxChNum;
    /**< EDMA RX channel / XBAR module number.
     *   Must match the XBAR instance configured in the syscfg file, e.g.
     *   DMA_TRIG_XBAR_EDMA_MODULE_1. */

    EDMA_Handle     edmaHandle;
    /**< EDMA handle, typically gEdmaHandle[0] from syscfg-generated code */

    /* --- Managed internally by I2C_dmaOpen / I2C_dmaClose --- */

    uint32_t        baseAddr;
    /**< EDMA controller base address, derived from edmaHandle in I2C_dmaOpen() */

    uint32_t        regionId;
    /**< EDMA region ID, derived from edmaHandle in I2C_dmaOpen() */

    /* TX channel resources */
    uint32_t        txCh;
    uint32_t        txTcc;
    uint32_t        txParam;

    /* RX channel resources */
    uint32_t        rxCh;
    uint32_t        rxTcc;
    uint32_t        rxParam;

    Edma_IntrObject txIntrObj;
    /**< EDMA interrupt object for TX completion; registered in I2C_dmaOpen() */

    Edma_IntrObject rxIntrObj;
    /**< EDMA interrupt object for RX completion; registered in I2C_dmaOpen() */

    SemaphoreP_Object txDoneSem;
    /**< Binary semaphore posted by the TX EDMA ISR on transfer completion */

    SemaphoreP_Object rxDoneSem;
    /**< Binary semaphore posted by the RX EDMA ISR on transfer completion */

    /* --- Set internally by I2C_dmaConfigRx() for use in the RX ISR --- */
    uint8_t            *rxBuf;
    /**< Pointer to the receive buffer; used by the RX ISR to invalidate cache */

    uint32_t            rxBufSize;
    /**< Size of the receive buffer in bytes */

} I2C_DmaChConfig;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  Allocate EDMA resources for I2C DMA TX and RX channels.
 *
 *  Allocates DMA channels, TCCs and param sets for both TX and RX directions.
 *  Configures the EDMA channel-to-region mapping and registers completion
 *  interrupts for each direction. Must be called once before any transfers.
 *
 *  \param cfg  Pointer to I2C_DmaChConfig with i2cTxRegAddr, i2cRxRegAddr,
 *              edmaTxChNum, edmaRxChNum and edmaHandle pre-filled by the caller.
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE otherwise.
 */
int32_t I2C_dmaOpen(I2C_DmaChConfig *cfg);

/**
 *  \brief  Configure and arm EDMA for an I2C TX (write) transfer.
 *
 *  Performs a cache write-back on the source buffer, programs the EDMA
 *  PaRAM entry (A-sync, 1 byte per trigger, source increments, destination
 *  fixed at I2C TX register) and arms the channel for event-triggered transfer.
 *
 *  Must be called after I2C_dmaOpen() and before asserting the I2C START.
 *
 *  \param cfg    Pointer to an opened I2C_DmaChConfig.
 *  \param buf    Pointer to the transmit data buffer.
 *  \param nBytes Number of bytes to transmit.
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE otherwise.
 */
int32_t I2C_dmaConfigTx(I2C_DmaChConfig *cfg, uint8_t *buf, uint32_t nBytes);

/**
 *  \brief  Configure and arm EDMA for an I2C RX (read) transfer.
 *
 *  Programs the EDMA PaRAM entry (A-sync, 1 byte per trigger, source fixed
 *  at I2C RX register, destination increments) and arms the channel for
 *  event-triggered transfer. Cache invalidation is performed in the RX ISR
 *  after data is received.
 *
 *  Must be called after I2C_dmaOpen() and before asserting the I2C START.
 *
 *  \param cfg    Pointer to an opened I2C_DmaChConfig.
 *  \param buf    Pointer to the receive data buffer.
 *  \param nBytes Number of bytes to receive.
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE otherwise.
 */
int32_t I2C_dmaConfigRx(I2C_DmaChConfig *cfg, uint8_t *buf, uint32_t nBytes);

/**
 *  \brief  Free all EDMA resources allocated by I2C_dmaOpen().
 *
 *  Unregisters TX and RX interrupts, destructs both semaphores, and frees
 *  all DMA channels, TCCs and param sets. Must be called when done with
 *  DMA transfers.
 *
 *  \param cfg  Pointer to an opened I2C_DmaChConfig.
 */
void I2C_dmaClose(I2C_DmaChConfig *cfg);

#ifdef __cplusplus
}
#endif

#endif /* I2C_DMA_H_ */
