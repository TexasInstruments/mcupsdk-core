/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#define APP_MCSPI_MSGSIZE               (100U)

uint8_t  gMcspiTxBuffer[APP_MCSPI_MSGSIZE];
uint8_t  gMcspiRxBuffer[APP_MCSPI_MSGSIZE];
uint32_t gMutexLockUnlock = MUTEX_ARM_LOCKED;
uint32_t gMutexLockUnlockIntr = MUTEX_ARM_LOCKED;
uint32_t gMcspiVimStsAddr, intrNum;
uint32_t gMcspiVimStsClrMask;
uint32_t intcBaseAddr;
MCSPILLD_Handle gMcspiHandle2;

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_MCSPI_ISR(void);

void *mcspi_loopback_multi_instances_multi_channels_lld_main(void *args)
{
    int32_t               status = MCSPI_STATUS_SUCCESS;
    uint32_t              bufIndex;
    uint32_t              count;
    uint32_t              timeout = MCSPI_WAIT_FOREVER;
    MCSPI_ExtendedParams  extendedParams;

    gMcspiHandle2 = &gMcspiObject[CONFIG_MCSPI2];

    for(int8_t instanceIndex = 0; instanceIndex < CONFIG_MCSPI_NUM_INSTANCES;
         instanceIndex++)
    {
        for(uint8_t chCnt = 0U; chCnt < gMcspiNumCh[instanceIndex]; chCnt++)
        {
            if ((chCnt % 2) == 0U)
            {
                gConfigMcspiChCfg[instanceIndex][chCnt].inputSelect = MCSPI_IS_D1;
                gConfigMcspiChCfg[instanceIndex][chCnt].dpe0        = MCSPI_DPE_DISABLE;
                gConfigMcspiChCfg[instanceIndex][chCnt].dpe1        = MCSPI_DPE_ENABLE;
                gConfigMcspiChCfg[instanceIndex][chCnt].csPolarity  = MCSPI_CS_POL_HIGH;
            }
            else
            {
                gConfigMcspiChCfg[instanceIndex][chCnt].inputSelect = MCSPI_IS_D0;
                gConfigMcspiChCfg[instanceIndex][chCnt].dpe0        = MCSPI_DPE_ENABLE;
                gConfigMcspiChCfg[instanceIndex][chCnt].dpe1        = MCSPI_DPE_DISABLE;
                gConfigMcspiChCfg[instanceIndex][chCnt].csPolarity  = MCSPI_CS_POL_LOW;
            }
        }
    }

    Drivers_open();
    Board_driversOpen();

    gMcspiObject[CONFIG_MCSPI1].transferMutex = &gMutexLockUnlock;
    gMcspiObject[CONFIG_MCSPI2].transferMutex = &gMutexLockUnlockIntr;

    /*  Interrupt configuration and registration   */
    intrNum = gMcspiInitObject[CONFIG_MCSPI2].intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gMcspiVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gMcspiVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    HwiP_setVecAddr(intrNum, (uintptr_t)&App_MCSPI_ISR);
    HwiP_setPri(intrNum, gMcspiInitObject[CONFIG_MCSPI2].intrPriority);
    HwiP_enableInt(intrNum);

    DebugP_log("\n[MCSPI] Loopback example started ...\r\n");

    for(int8_t instanceIndex = 0; instanceIndex < CONFIG_MCSPI_NUM_INSTANCES;
         instanceIndex++)
    {
        for(uint32_t chCnt = 0U; chCnt < gMcspiNumCh[instanceIndex]; chCnt++)
        {
            /* Memfill buffers */
            for(bufIndex = 0U; bufIndex < APP_MCSPI_MSGSIZE; bufIndex++)
            {
                gMcspiTxBuffer[bufIndex] = bufIndex;
                gMcspiRxBuffer[bufIndex] = 0U;
            }

            /* populate extended parameters */
            extendedParams.channel    = gConfigMcspiChCfg[instanceIndex][chCnt].chNum;
            extendedParams.csDisable  = TRUE;
            extendedParams.dataSize   = 8;
            count = APP_MCSPI_MSGSIZE / (extendedParams.dataSize/8);

            if(gMcspiInitObject[instanceIndex].operMode == MCSPI_OPER_MODE_POLLED)
            {
                status += MCSPI_lld_readWrite(gMcspiHandle[instanceIndex], \
                                                gMcspiTxBuffer, \
                                                &gMcspiRxBuffer, \
                                                count, \
                                                timeout, \
                                                &extendedParams);
            }

            else if (gMcspiInitObject[instanceIndex].operMode == MCSPI_OPER_MODE_INTERRUPT)
            {
                 status += MCSPI_lld_readWriteIntr(gMcspiHandle2, \
                                                    gMcspiTxBuffer, \
                                                    &gMcspiRxBuffer, \
                                                    count, \
                                                    timeout, \
                                                    &extendedParams);
                while(try_lock_mutex(gMcspiObject[CONFIG_MCSPI2].transferMutex) == MUTEX_ARM_LOCKED);
            }

            else
            {
                /* Writeback buffer */
                CacheP_wb(&gMcspiTxBuffer[0U], sizeof(gMcspiTxBuffer), CacheP_TYPE_ALLD);
                CacheP_wb(&gMcspiRxBuffer[0U], sizeof(gMcspiRxBuffer), CacheP_TYPE_ALLD);
                status += MCSPI_lld_readWriteDma(gMcspiHandle[instanceIndex], \
                                                    gMcspiTxBuffer, \
                                                    &gMcspiRxBuffer, \
                                                    count, \
                                                    timeout, \
                                                    &extendedParams);
                while(try_lock_mutex(gMcspiObject[CONFIG_MCSPI1].transferMutex) == MUTEX_ARM_LOCKED);

                /* Invalidate cache only in DMA mode */
                CacheP_inv(&gMcspiRxBuffer[0U], sizeof(gMcspiRxBuffer), CacheP_TYPE_ALLD);
            }

            if(MCSPI_TRANSFER_COMPLETED != status)
            {
                /* MCSPI transfer failed!! */
                DebugP_assert(FALSE);
            }
            else
            {
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

            /* clear buffers */
            for(bufIndex = 0U; bufIndex < APP_MCSPI_MSGSIZE; bufIndex++)
            {
                gMcspiTxBuffer[bufIndex] = 0U;
                gMcspiRxBuffer[bufIndex] = 0U;
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
    unlock_mutex(gMcspiObject[CONFIG_MCSPI1].transferMutex);
}

void MCSPI_lld_errorCallback (void *args)
{
    return;
}

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_MCSPI_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(MCSPI_lld_controllerIsr, \
                                      gMcspiHandle2, \
                                      intrNum, \
                                      gMcspiVimStsAddr, \
                                      gMcspiVimStsClrMask,
                                      intcBaseAddr);
}

void MCSPI_lld_intrTransferCallback(void *args, uint32_t transferStatus)
{
    unlock_mutex(gMcspiObject[CONFIG_MCSPI2].transferMutex);
}

void MCSPI_lld_intrErrorCallback(void *args)
{
    return;
}