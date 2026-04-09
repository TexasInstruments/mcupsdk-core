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
 * This multicore example demonstrates an I2C register-read using a repeated
 * start condition and EDMA for data transfer.  Two I2C instances are used:
 *
 *   r5fss0-0 (I2C1) — Controller:
 *     Writes a 1-byte register address to the target.  Without releasing the
 *     bus (no STOP), issues a repeated START and reads I2C_DATA_SIZE bytes of
 *     register data back.
 *
 *   r5fss0-1 (I2C3) — Target:
 *     Receives the 1-byte register address via EDMA.  Builds a response where
 *     each byte equals reg_addr + byte_index.  Transmits the response via EDMA
 *     after the repeated START.
 *
 * Synchronisation:
 *   Barrier 1 — both cores have completed driver/DMA setup.
 *   Barrier 2 — target hardware is fully armed (I2CTargetEnable +
 *               REPEAT_MODE set); controller is clear to assert START.
 *   Barrier 3 — transfer complete on both sides; controller flushes target
 *               shared-memory logs before cleanup.
 *
 * ============================================================================
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/soc_config.h>
#include <drivers/ipc_notify.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>

/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */

/* Number of data bytes returned by the target per register read */
#define I2C_DATA_SIZE                   (8U)

/* Register address the controller will request */
#define I2C_REG_ADDR                    (0x10U)

/* Target device address (must match i2c1.ownTargetAddr in r5fss0-1 syscfg) */
#define I2C_TARGET_DEVICE_ADDR          (0x2CU)

/* ========================================================================== */
/*                         Global Variables                                   */
/* ========================================================================== */

I2CLLD_Handle gI2cLldHandle0;   /* Controller handle (r5fss0-0, I2C1) */
I2CLLD_Handle gI2cLldHandle1;   /* Target handle    (r5fss0-1, I2C3) */

/*
 * Controller:  txBuf holds the 1-byte register address to write.
 *              rxBuf holds the I2C_DATA_SIZE bytes read back.
 * Target:      rxBuf receives the 1-byte register address.
 *              txBuf is filled with the response after rxBuf[0] is known.
 *
 * Both buffers are cache-line aligned so CacheP_wbInv covers them exactly.
 */
uint8_t txBuf[I2C_DATA_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint8_t rxBuf[I2C_DATA_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* ========================================================================== */
/*                         External Declarations                              */
/* ========================================================================== */

extern void    I2C_ctrlDmaSetup(void);
extern void    I2C_ctrlDmaCleanup(void);
extern void    I2C_tgtDmaSetup(void);
extern void    I2C_tgtDmaCleanup(void);
extern int32_t I2C_lld_repeatedStartDMA(I2CLLD_Handle handle, I2CLLD_Message *msg);
extern int32_t I2C_tgtDmaArm(I2CLLD_Handle handle, I2CLLD_Message *msg);
extern int32_t I2C_lld_tgtWaitTransfer(I2CLLD_Handle handle, I2CLLD_Message *msg);

/* ========================================================================== */
/*                         Controller task (r5fss0-0)                        */
/* ========================================================================== */

/**
 * \brief  I2C register-read with repeated start — controller side.
 *
 *  Writes a 1-byte register address (I2C_REG_ADDR) to the target.  Without
 *  releasing the bus, issues a repeated START and reads I2C_DATA_SIZE bytes
 *  back.  Validates that rxBuf[i] == I2C_REG_ADDR + i.
 */
void i2c_dma_write_read_repeated_start(void *arg)
{
    int32_t            status = I2C_STS_SUCCESS;
    uint8_t            i;
    I2CLLD_Transaction txn;
    I2CLLD_Message     msg;

    Drivers_open();
    Board_driversOpen();

    /* Prepare TX: single register address byte */
    txBuf[0] = I2C_REG_ADDR;

    /* Clear RX buffer and writeback-invalidate so EDMA writes land in
     * physical memory and are not overwritten by a stale cache flush */
    for (i = 0U; i < I2C_DATA_SIZE; i++)
    {
        rxBuf[i] = 0U;
    }
    CacheP_wbInv((void *)rxBuf, I2C_DATA_SIZE, CacheP_TYPE_ALL);

    gI2cLldHandle0 = (I2CLLD_Handle)(gI2cLldHandle[0]);

    I2C_ctrlDmaSetup();

    DebugP_shmLogReaderSetChunkSize(DebugP_SHM_LOG_READER_CHUNK_SIZE_INF);

    /* Barrier 1: both cores have finished setup */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);
    DebugP_shmLogRead();

    /* Barrier 2: wait for target to arm I2CTargetEnable + REPEAT_MODE.
     * No sleep needed — the barrier is the guarantee. */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Build message: write 1-byte register address, read I2C_DATA_SIZE bytes */
    (void)I2C_lld_Transaction_init(&txn);
    txn.writeBuf   = txBuf;
    txn.writeCount = 1U;
    txn.readBuf    = rxBuf;
    txn.readCount  = I2C_DATA_SIZE;

    (void)I2C_lld_Message_init(&msg);
    msg.txn            = &txn;
    msg.txnCount       = 1U;
    msg.targetAddress  = I2C_TARGET_DEVICE_ADDR;
    msg.expandSA       = false;
    msg.controllerMode = true;

    DebugP_log("[I2C] Controller: requesting register 0x%02X (%d bytes) ...\r\n",
               I2C_REG_ADDR, I2C_DATA_SIZE);

    status = I2C_lld_repeatedStartDMA(gI2cLldHandle0, &msg);

    /* Invalidate rxBuf cache lines so the CPU reads EDMA-written data from
     * physical memory, not stale cache.  Must be done before any CPU access
     * to rxBuf after the DMA transfer completes. */
    CacheP_inv((void *)rxBuf, I2C_DATA_SIZE, CacheP_TYPE_ALL);

    if (status == I2C_STS_SUCCESS)
    {
        /* Validate: each byte must equal I2C_REG_ADDR + index */
        for (i = 0U; i < I2C_DATA_SIZE; i++)
        {
            if (rxBuf[i] != (uint8_t)(I2C_REG_ADDR + i))
            {
                DebugP_log("[I2C] Controller: mismatch at [%u]: "
                           "expected=0x%02X got=0x%02X\r\n",
                           i, (uint8_t)(I2C_REG_ADDR + i), rxBuf[i]);
                status = I2C_STS_ERR;
                break;
            }
        }
    }

    if (status == I2C_STS_SUCCESS)
    {
        DebugP_log("[I2C] Controller: register read verified\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("[I2C] Controller: transfer failed (status=%d)\r\n", status);
        DebugP_log("Some tests have failed!!\r\n");
    }

    /* Barrier 3: wait for target to finish logging, then flush its logs */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);
    DebugP_shmLogRead();

    I2C_ctrlDmaCleanup();

    Board_driversClose();
    Drivers_close();
}

/* ========================================================================== */
/*                         Target task (r5fss0-1)                            */
/* ========================================================================== */

/**
 * \brief  I2C register-read with repeated start — target side.
 *
 *  Arms the I2C hardware via I2C_tgtDmaArm(), signals Barrier 2 so the
 *  controller knows it is safe to send START, then calls
 *  I2C_lld_tgtWaitTransfer() which:
 *    - receives the 1-byte register address
 *    - builds and transmits the response (reg_addr + i for each byte)
 */
void i2c_dma_target_repeated_start(void *arg)
{
    int32_t            status = I2C_STS_SUCCESS;
    I2CLLD_Transaction txn;
    I2CLLD_Message     msg;

    Drivers_open();
    Board_driversOpen();

    /* Clear both buffers; txBuf will be filled by I2C_lld_tgtWaitTransfer
     * once the register address is known */
    for (uint8_t i = 0U; i < I2C_DATA_SIZE; i++)
    {
        txBuf[i] = 0U;
        rxBuf[i] = 0U;
    }
    /* Writeback-invalidate rxBuf so EDMA writes are not lost to cache */
    CacheP_wbInv((void *)rxBuf, I2C_DATA_SIZE, CacheP_TYPE_ALL);

    gI2cLldHandle1 = (I2CLLD_Handle)(gI2cLldHandle[0]);

    I2C_tgtDmaSetup();

    /* Build message:
     *   readBuf/readCount  — receive the 1-byte register address
     *   writeBuf/writeCount — buffer for the response (filled after RX) */
    (void)I2C_lld_Transaction_init(&txn);
    txn.readBuf    = rxBuf;
    txn.readCount  = 1U;
    txn.writeBuf   = txBuf;
    txn.writeCount = I2C_DATA_SIZE;

    (void)I2C_lld_Message_init(&msg);
    msg.txn            = &txn;
    msg.txnCount       = 1U;
    msg.expandSA       = false;
    msg.controllerMode = false;

    /* Barrier 1: both cores have finished setup */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    DebugP_log("[I2C] Target: arming for repeated-start transfer ...\r\n");

    /* Arm the target hardware: SetDataCount, dmaConfigRx,
     * I2CTargetEnable, I2CModeControl(REPEAT_MODE_ON) */
    status = I2C_tgtDmaArm(gI2cLldHandle1, &msg);

    /* Barrier 2: signal controller that the target is fully armed.
     * Controller will call I2CControllerStart() only after this point. */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    if (status == I2C_STS_SUCCESS)
    {
        /* Wait for RX phase, build and send TX response */
        status = I2C_lld_tgtWaitTransfer(gI2cLldHandle1, &msg);
    }

    if (status == I2C_STS_SUCCESS)
    {
        DebugP_log("[I2C] Target: transfer complete — responded to register 0x%02X\r\n",
                   rxBuf[0]);
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("[I2C] Target: transfer failed (status=%d)\r\n", status);
        DebugP_log("Some tests have failed!!\r\n");
    }

    /* Barrier 3: signal controller that logging is done */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    I2C_tgtDmaCleanup();

    Board_driversClose();
    Drivers_close();
}
