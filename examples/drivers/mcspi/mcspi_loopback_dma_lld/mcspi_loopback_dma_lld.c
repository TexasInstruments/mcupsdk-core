/*
 *  Copyright (C) 2023-24 Texas Instruments Incorporated
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

/* This example demonstrates the McSPI RX and TX operation configured
 * in blocking, interrupt mode of operation.
 *
 * This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
 * and then receives the same in RX mode. Internal pad level loopback mode
 * is enabled to receive data.
 * To enable internal pad level loopback mode, D0 pin is configured to both
 * TX Enable as well as RX input pin in the SYSCFG.
 *
 * When transfer is completed, TX and RX buffer data are compared.
 * If data is matched, test result is passed otherwise failed.
 */

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

/* MCSPI Channel Macro */
#define APP_MCSPI_MSGSIZE               (100U)
#define APP_MCSPI_TRANSFER_LOOPCOUNT    (10U)

uint32_t gMutexLockUnlock = MUTEX_ARM_LOCKED;
uint8_t gMcspiTxBuffer[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint8_t gMcspiRxBuffer[APP_MCSPI_MSGSIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void *mcspi_loopback_dma_lld_main(void *args)
{
    int32_t               status = MCSPI_STATUS_SUCCESS;
    uint32_t              bufIndex;
    uint32_t              count;
    uint32_t              timeout = MCSPI_WAIT_FOREVER;
    MCSPI_ExtendedParams  extendedParams;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[MCSPI] Loopback example started ...\r\n");

    /* Memfill buffers */
    for(bufIndex = 0U; bufIndex < APP_MCSPI_MSGSIZE; bufIndex++)
    {
        gMcspiTxBuffer[bufIndex] = bufIndex;
        gMcspiRxBuffer[bufIndex] = 0U;
    }

    /* Writeback buffer */
    CacheP_wb(&gMcspiTxBuffer[0U], sizeof(gMcspiTxBuffer), CacheP_TYPE_ALLD);
    CacheP_wb(&gMcspiRxBuffer[0U], sizeof(gMcspiRxBuffer), CacheP_TYPE_ALLD);

    gMcspiObject[CONFIG_MCSPI0].transferMutex = &gMutexLockUnlock;

    /* populate extended parameters */
    extendedParams.channel    = gConfigMcspi0ChCfg[0].chNum;
    extendedParams.csDisable  = TRUE;
    extendedParams.dataSize   = 32;
    count = APP_MCSPI_MSGSIZE / (extendedParams.dataSize/8);

    for(uint32_t loopCount = 0; loopCount < APP_MCSPI_TRANSFER_LOOPCOUNT; loopCount++)
    {
        status += MCSPI_lld_readWriteDma(gMcspiHandle[CONFIG_MCSPI0], \
                                        gMcspiTxBuffer, \
                                        &gMcspiRxBuffer, \
                                        count, \
                                        timeout, \
                                        &extendedParams);
        while(try_lock_mutex(gMcspiObject[CONFIG_MCSPI0].transferMutex) == (uint32_t)MUTEX_ARM_LOCKED);
    }

    if(MCSPI_TRANSFER_COMPLETED != status)
    {
        /* MCSPI transfer failed!! */
        DebugP_assert(FALSE);
    }
    else
    {
        /* Invalidate cache */
        CacheP_inv(&gMcspiRxBuffer[0U], sizeof(gMcspiRxBuffer), CacheP_TYPE_ALLD);

        /* Compare data */
        for(bufIndex = 0U; bufIndex < APP_MCSPI_MSGSIZE; bufIndex++)
        {
            if(gMcspiTxBuffer[bufIndex] != gMcspiRxBuffer[bufIndex])
            {
                /* Data mismatch */
                status = MCSPI_STATUS_FAILURE;
                DebugP_log("Data Mismatch at offset %d\r\n", bufIndex);
                break;
            }
        }
    }

    if(MCSPI_STATUS_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return NULL;
}

void MCSPI_lld_transferCallback (void *args, uint32_t transferStatus)
{
    unlock_mutex(gMcspiObject[CONFIG_MCSPI0].transferMutex);
}

/* Not in Use */
void MCSPI_lld_errorCallback (void *args)
{
    return;
}