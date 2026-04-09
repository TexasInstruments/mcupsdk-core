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
 *
 * ============================================================================
 *
 * This is a multicore example demonstrating basic I2C read and write operations
 * using DMA for data transfer. Two I2C instances are used:
 *   - r5fss0-0 (I2C1): Controller — transmits data to the target
 *   - r5fss0-1 (I2C3): Target    — receives data from the controller
 *
 * Both cores use EDMA for data movement. The I2C ISR handles only error and
 * stop-condition interrupts; data transfers are driven by EDMA events.
 *
 * ============================================================================
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/soc_config.h>
#include <drivers/ipc_notify.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>

/* Transfer size */
#define I2C_TRANSFER_SIZE                   (8U)

/* Target device address (must match i2c1.ownTargetAddr in r5fss0-1 syscfg) */
#define I2C_TARGET_DEVICE_ADDR              (0x2CU)

/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */

I2CLLD_Handle gI2cLldHandle0;   /* Controller handle (r5fss0-0) */
I2CLLD_Handle gI2cLldHandle1;   /* Target handle    (r5fss0-1) */

I2C_ExtendedParams extendedParamsTx, extendedParamsRx;

uint8_t txBuf[I2C_TRANSFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint8_t rxBuf[I2C_TRANSFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* ========================================================================== */
/*                         External Declarations                              */
/* ========================================================================== */

/* Controller core (r5fss0-0) */
extern void    I2C_ctrlDmaSetup(void);
extern void    I2C_ctrlDmaCleanup(void);
extern int32_t I2C_lld_writeDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams);
extern int32_t I2C_lld_readDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams);

/* Target core (r5fss0-1) */
extern void    I2C_tgtDmaSetup(void);
extern void    I2C_tgtDmaCleanup(void);
extern int32_t I2C_lld_targetReadDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams);
extern int32_t I2C_lld_targetWriteDMA(I2CLLD_Handle handle, I2C_ExtendedParams *extendedParams);

/* ========================================================================== */
/*                         Controller task (r5fss0-0)                        */
/* ========================================================================== */

/**
 * \brief  I2C DMA write — run on r5fss0-0 (controller, I2C1).
 *
 *  Initialises driver, fills txBuf, performs a DMA write of I2C_TRANSFER_SIZE
 *  bytes to the target, verifies the status, and cleans up.
 */
void i2c_dma_write(void *arg)
{
    int32_t status = I2C_STS_SUCCESS;
    uint8_t i;

    Drivers_open();
    Board_driversOpen();

    /* Fill transmit buffer with sequential values */
    for (i = 0U; i < I2C_TRANSFER_SIZE; i++)
    {
        txBuf[i] = i+1;
    }

    gI2cLldHandle0 = (I2CLLD_Handle)(gI2cLldHandle[0]);

    /* Allocate EDMA channels for controller */
    I2C_ctrlDmaSetup();

    /* Sync with target: both cores have finished DMA setup and are ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Drain any shared-memory log entries core 1 wrote before the sync */
    DebugP_shmLogRead();

    ClockP_sleep(1);

    extendedParamsTx.deviceAddress = I2C_TARGET_DEVICE_ADDR;
    extendedParamsTx.buffer        = txBuf;
    extendedParamsTx.size          = I2C_TRANSFER_SIZE;
    extendedParamsTx.expandSA      = false;

    DebugP_log("[I2C] Controller: DMA write starting ...\r\n");

    status = I2C_lld_writeDMA(gI2cLldHandle0, &extendedParamsTx);

    if (status == I2C_STS_SUCCESS)
    {
        DebugP_log("[I2C] Controller: DMA write complete\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("[I2C] Controller: DMA write failed (status=%d)\r\n", status);
        DebugP_log("Some tests have failed!!\r\n");
    }

    /* Wait for target to finish its validation and logging before we exit */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Flush all remaining shared-memory log entries written by core 1 */
    DebugP_shmLogRead();

    /* Free EDMA channels */
    I2C_ctrlDmaCleanup();

    Board_driversClose();
    Drivers_close();
}

/* ========================================================================== */
/*                         Target task (r5fss0-1)                            */
/* ========================================================================== */

/**
 * \brief  I2C DMA read — run on r5fss0-1 (target, I2C3).
 *
 *  Initialises driver, clears rxBuf, performs a DMA read of I2C_TRANSFER_SIZE
 *  bytes from the controller, validates the received data, and cleans up.
 */
void i2c_dma_read(void *arg)
{
    int32_t status = I2C_STS_SUCCESS;
    uint8_t i;

    Drivers_open();
    Board_driversOpen();

    /* Zero the receive buffer. Writeback-invalidate flushes the zeros to
     * physical memory so EDMA writes land there, not over stale cache lines. */
    for (i = 0U; i < I2C_TRANSFER_SIZE; i++)
    {
        rxBuf[i] = 0U;
    }
    CacheP_wbInv((void *)rxBuf, I2C_TRANSFER_SIZE, CacheP_TYPE_ALL);

    gI2cLldHandle1 = (I2CLLD_Handle)(gI2cLldHandle[0]);

    /* Allocate EDMA channels for target */
    I2C_tgtDmaSetup();

    IpcNotify_syncAll(SystemP_WAIT_FOREVER);


    extendedParamsRx.buffer   = rxBuf;
    extendedParamsRx.size     = I2C_TRANSFER_SIZE;
    extendedParamsRx.expandSA = false;

    DebugP_log("[I2C] Target: DMA read starting ...\r\n");

    status = I2C_lld_targetReadDMA(gI2cLldHandle1, &extendedParamsRx);

    ClockP_sleep(1);

    if (status == I2C_STS_SUCCESS)
    {
        /* Validate received data */
        for (i = 0U; i < I2C_TRANSFER_SIZE; i++)
        {
            if (rxBuf[i] != i+1)
            {
                DebugP_log("[I2C] Target: data mismatch at [%u]: expected=0x%02X got=0x%02X\r\n",
                           i, i, rxBuf[i]);
                status = I2C_STS_ERR;
                break;
            }
        }
    }

    if (status == I2C_STS_SUCCESS)
    {
        DebugP_log("[I2C] Target: DMA read complete — data verified\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("[I2C] Target: DMA read failed (status=%d)\r\n", status);
        DebugP_log("Some tests have failed!!\r\n");
    }

    /* Signal core 0 that all logging is done so it can flush and exit */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Free EDMA channels */
    I2C_tgtDmaCleanup();

    Board_driversClose();
    Drivers_close();
}
