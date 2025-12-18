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
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <drivers/hw_include/cslr_soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* This example shows Controller Peripheral Communication between Two I2C instances
 * in a single core application using the I2C LLD (Low-Level Driver) API.
 *
 * One I2C instance is configured as a Controller and the other as a Peripheral.
 *
 * Three transactions are carried out in this example:
 *
 * 1. Controller writes 4 bytes to Peripheral
 * 2. Controller writes 2 bytes to Peripheral
 * 3. Controller reads 2 bytes from Peripheral
 *
 * This example uses dual semaphores (one for controller, one for peripheral)
 * with DSB memory barriers to ensure cache coherency and proper ISR completion.
 */

/* I2C Interrupt Priority */
#define I2C_INTERRUPT_PRIORITY              (4U)

/* I2C Target Address */
#define I2C_TARGET_ADDR                     (0x2CU)

/* I2C LLD Handles */
volatile I2CLLD_Handle gI2cControllerLldHandle;
volatile I2CLLD_Handle gI2cPeripheralLldHandle;

/* I2C LLD Transactions */
I2CLLD_Transaction gI2cControllerTransaction;

/* I2C LLD Message */
I2CLLD_Message gI2cControllerMessage;

/* I2C Data Buffers */
uint8_t gRxControllerBuffer[4];
uint8_t gTxControllerBuffer[4];
uint8_t gTxPeripheralBuffer[4];
uint8_t gRxPeripheralBuffer[4];

/* Semaphores for synchronization */
static SemaphoreP_Object gControllerTransferDoneSemaphore;
static SemaphoreP_Object gPeripheralTransferDoneSemaphore;

/* Define SemaphoreP_WAIT_FOREVER if not defined */
#ifndef SemaphoreP_WAIT_FOREVER
#define SemaphoreP_WAIT_FOREVER              (uint32_t)SystemP_WAIT_FOREVER
#endif

/* Interrupt vectors */
volatile uint32_t gI2cControllerIntrNum, gI2cPeripheralIntrNum;
volatile uint32_t gI2cControllerVimStsAddr, gI2cPeripheralVimStsAddr;
volatile uint32_t gI2cControllerVimStsClrMask, gI2cPeripheralVimStsClrMask;
volatile uint32_t gIntcBaseAddr;

extern HwiP_Config gHwiConfig;

/* Function prototypes */
void I2C_lld_controllerTransferCompleteCallback(void *args, const I2CLLD_Message *msg, int32_t transferStatus);
void I2C_lld_peripheralTransferCompleteCallback(void *args, const I2CLLD_targetTransaction *targetTxn, int32_t transferStatus);
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_Controller_ISR(void);
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_Peripheral_ISR(void);
void i2c_peripheral_transfer_lld_init(void);
void i2c_peripheral_transfer_lld_main(void *args);

void I2C_lld_controllerTransferCompleteCallback(void *args, const I2CLLD_Message *msg, int32_t transferStatus)
{
    if(transferStatus == I2C_STS_SUCCESS)
    {
        SemaphoreP_post(&gControllerTransferDoneSemaphore);
        /* Memory barrier to ensure semaphore update is visible to main thread */
        __asm__ volatile("dsb" ::: "memory");
    }
    else
    {
        DebugP_log("[I2C Controller] Transfer failed with status %d\r\n", transferStatus);

        SemaphoreP_post(&gControllerTransferDoneSemaphore);
        __asm__ volatile("dsb" ::: "memory");
    }
}

/* Peripheral transfer complete callback */
void I2C_lld_peripheralTransferCompleteCallback(void *args, const I2CLLD_targetTransaction *targetTxn, int32_t transferStatus)
{
    if(transferStatus == I2C_STS_SUCCESS)
    {
        SemaphoreP_post(&gPeripheralTransferDoneSemaphore);
        /* Memory barrier to ensure semaphore update is visible to main thread */
        __asm__ volatile("dsb" ::: "memory");
    }
    else if(transferStatus == I2C_STS_RESTART)
    {
        /* RESTART condition - don't post semaphore, transaction continues */
    }
    else
    {
        DebugP_log("[I2C Peripheral] Transfer failed with status %d\r\n", transferStatus);
        SemaphoreP_post(&gPeripheralTransferDoneSemaphore);
        __asm__ volatile("dsb" ::: "memory");
    }
}

/* Controller ISR */
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_Controller_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(I2C_lld_controllerIsr,
                                      gI2cControllerLldHandle,
                                      gI2cControllerIntrNum,
                                      gI2cControllerVimStsAddr,
                                      gI2cControllerVimStsClrMask,
                                      gIntcBaseAddr);
}

/* Peripheral ISR */
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_Peripheral_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(I2C_lld_targetIsr,
                                      gI2cPeripheralLldHandle,
                                      gI2cPeripheralIntrNum,
                                      gI2cPeripheralVimStsAddr,
                                      gI2cPeripheralVimStsClrMask,
                                      gIntcBaseAddr);
}

/* Initialize I2C LLD instances */
void i2c_peripheral_transfer_lld_init(void)
{
    /* Create semaphores for synchronization */
    SemaphoreP_constructBinary(&gControllerTransferDoneSemaphore, 0);
    SemaphoreP_constructBinary(&gPeripheralTransferDoneSemaphore, 0);

    /* Get I2C handles from pre-configured handles */
    gI2cControllerLldHandle = (I2CLLD_Handle)(gI2cLldHandle[CONFIG_I2C0]);
    gI2cPeripheralLldHandle = (I2CLLD_Handle)(gI2cLldHandle[CONFIG_I2C1]);

    /* Set callback functions */
    gI2cControllerLldHandle->transferCompleteCallback = I2C_lld_controllerTransferCompleteCallback;
    gI2cPeripheralLldHandle->targetTransferCompleteCallback = I2C_lld_peripheralTransferCompleteCallback;

    /* Set target address for peripheral mode */
    gI2cPeripheralLldHandle->ownTargetAddr = I2C_TARGET_ADDR;
    
    /* Get interrupt numbers and base address */
    gI2cControllerIntrNum = gI2cControllerLldHandle->intrNum;
    gI2cPeripheralIntrNum = gI2cPeripheralLldHandle->intrNum;
    gIntcBaseAddr = gHwiConfig.intcBaseAddr;

    /* Calculate VIM status addresses and clear masks for Controller */
    gI2cControllerVimStsAddr = gIntcBaseAddr + (0x404u + (((gI2cControllerIntrNum) >> 5) & 0xFu) * 0x20u);
    gI2cControllerVimStsClrMask = 0x1u << ((gI2cControllerIntrNum) & 0x1Fu);

    /* Calculate VIM status addresses and clear masks for Peripheral */
    gI2cPeripheralVimStsAddr = gIntcBaseAddr + (0x404u + (((gI2cPeripheralIntrNum) >> 5) & 0xFu) * 0x20u);
    gI2cPeripheralVimStsClrMask = 0x1u << ((gI2cPeripheralIntrNum) & 0x1Fu);

    /* Register Controller interrupt */
    HwiP_setVecAddr(gI2cControllerIntrNum, (uintptr_t)&App_I2C_Controller_ISR);
    HwiP_setPri(gI2cControllerIntrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_enableInt(gI2cControllerIntrNum);

    /* Register Peripheral interrupt */
    HwiP_setVecAddr(gI2cPeripheralIntrNum, (uintptr_t)&App_I2C_Peripheral_ISR);
    HwiP_setPri(gI2cPeripheralIntrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_enableInt(gI2cPeripheralIntrNum);
}

/* Main function */
void i2c_peripheral_transfer_lld_main(void *args)
{
    int32_t status;
    I2CLLD_targetTransaction targetTxn;

    /* Initialize drivers */
    Drivers_open();
    Board_driversOpen();

    /* Initialize I2C LLD instances */
    i2c_peripheral_transfer_lld_init();

    DebugP_log("[I2C] I2C Controller Peripheral Transaction Started ... !!!\r\n");

    /* Initialize all buffers */
    memset(gRxControllerBuffer, 0, sizeof(gRxControllerBuffer));
    memset(gRxPeripheralBuffer, 0, sizeof(gRxPeripheralBuffer));

    /* Fill TX Buffers */
    gTxControllerBuffer[0] = 0x01U;
    gTxControllerBuffer[1] = 0x02U;
    gTxControllerBuffer[2] = 0x03U;
    gTxControllerBuffer[3] = 0x04U;

    gTxPeripheralBuffer[0] = 0x05U;
    gTxPeripheralBuffer[1] = 0x06U;
    gTxPeripheralBuffer[2] = 0x07U;
    gTxPeripheralBuffer[3] = 0x08U;

    /* ==================================================== */
    /* === TRANSACTION 1: Controller writes 4 bytes === */
    /* ==================================================== */

    /* Prepare peripheral: receive 4 bytes */
    targetTxn.readBuf = gRxPeripheralBuffer;
    targetTxn.readCount = 4U;
    targetTxn.writeBuf = gTxPeripheralBuffer;
    targetTxn.writeCount = 0U;
    targetTxn.timeout = I2C_WAIT_FOREVER;
    targetTxn.expandSA = false;

    /* Start peripheral transfer */
    status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
    if(status == I2C_STS_SUCCESS)
    {
        DebugP_log("[I2C Peripheral] Transaction 1: Ready (receive 4)...\r\n");
        ClockP_usleep(1000);

        /* Initialize transaction structures */
        I2C_lld_Transaction_init(&gI2cControllerTransaction);
        I2C_lld_Message_init(&gI2cControllerMessage);

        /* Controller: write 4 bytes */
        gI2cControllerTransaction.writeBuf = gTxControllerBuffer;
        gI2cControllerTransaction.writeCount = 4U;
        gI2cControllerTransaction.readBuf = NULL;
        gI2cControllerTransaction.readCount = 0U;

        gI2cControllerMessage.txn = &gI2cControllerTransaction;
        gI2cControllerMessage.txnCount = 1U;
        gI2cControllerMessage.targetAddress = I2C_TARGET_ADDR;
        gI2cControllerMessage.timeout = I2C_WAIT_FOREVER;
        gI2cControllerMessage.controllerMode = true;
        gI2cControllerMessage.expandSA = false;

        DebugP_log("[I2C Controller] Transaction 1: Start (write 4)...\r\n");
        status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
    }

    if(status == I2C_STS_SUCCESS)
    {
        /* Wait for transaction 1 - wait for BOTH peripheral and controller */
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("[I2C Controller] Transmitted Data %u %u %u %u !!!\r\n",
                   gTxControllerBuffer[0], gTxControllerBuffer[1],
                   gTxControllerBuffer[2], gTxControllerBuffer[3]);
        DebugP_log("[I2C Peripheral] Received Data %u %u %u %u !!!\r\n",
                   gRxPeripheralBuffer[0], gRxPeripheralBuffer[1],
                   gRxPeripheralBuffer[2], gRxPeripheralBuffer[3]);

        /* Clear buffers */
        memset(gRxPeripheralBuffer, 0, sizeof(gRxPeripheralBuffer));
    }

    /* ==================================================== */
    /* === TRANSACTION 2: Controller writes 2 bytes === */
    /* ==================================================== */
    if(status == I2C_STS_SUCCESS)
    {
        /* Prepare peripheral: receive 2 bytes */
        targetTxn.readBuf = gRxPeripheralBuffer;
        targetTxn.readCount = 2U;
        targetTxn.writeBuf = gTxPeripheralBuffer;
        targetTxn.writeCount = 0U;
        targetTxn.timeout = I2C_WAIT_FOREVER;
        targetTxn.expandSA = false;

        /* Start peripheral transfer */
        status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
        if(status == I2C_STS_SUCCESS)
        {
            DebugP_log("[I2C Peripheral] Transaction 2: Ready (receive 2)...\r\n");
            ClockP_usleep(1000);

            /* Controller: write 2 bytes */
            gI2cControllerTransaction.writeBuf = gTxControllerBuffer;
            gI2cControllerTransaction.writeCount = 2U;
            gI2cControllerTransaction.readBuf = NULL;
            gI2cControllerTransaction.readCount = 0U;

            gI2cControllerMessage.txn = &gI2cControllerTransaction;
            gI2cControllerMessage.txnCount = 1U;
            gI2cControllerMessage.targetAddress = I2C_TARGET_ADDR;
            gI2cControllerMessage.timeout = I2C_WAIT_FOREVER;
            gI2cControllerMessage.controllerMode = true;
            gI2cControllerMessage.expandSA = false;

            DebugP_log("[I2C Controller] Transaction 2: Start (write 2)...\r\n");
            status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
        }
    }

    if(status == I2C_STS_SUCCESS)
    {
        /* Wait for transaction 2 - wait for BOTH peripheral and controller */
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("[I2C Controller] Transmitted Data %u %u !!!\r\n",
                   gTxControllerBuffer[0], gTxControllerBuffer[1]);
        DebugP_log("[I2C Peripheral] Received Data %u %u !!!\r\n",
                   gRxPeripheralBuffer[0], gRxPeripheralBuffer[1]);

        /* Clear buffers */
        memset(gRxPeripheralBuffer, 0, sizeof(gRxPeripheralBuffer));
    }

    /* ==================================================== */
    /* === TRANSACTION 3: Controller reads 2 bytes === */
    /* ==================================================== */
    if(status == I2C_STS_SUCCESS)
    {
        /* Prepare peripheral: send 2 bytes */
        targetTxn.readBuf = gRxPeripheralBuffer;
        targetTxn.readCount = 0U;
        targetTxn.writeBuf = gTxPeripheralBuffer;
        targetTxn.writeCount = 2U;
        targetTxn.timeout = I2C_WAIT_FOREVER;
        targetTxn.expandSA = false;

        /* Start peripheral transfer */
        status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
        if(status == I2C_STS_SUCCESS)
        {
            DebugP_log("[I2C Peripheral] Transaction 3: Ready (send 2)...\r\n");
            ClockP_usleep(1000);

            /* Controller: read 2 bytes */
            gI2cControllerTransaction.writeBuf = NULL;
            gI2cControllerTransaction.writeCount = 0U;
            gI2cControllerTransaction.readBuf = gRxControllerBuffer;
            gI2cControllerTransaction.readCount = 2U;

            gI2cControllerMessage.txn = &gI2cControllerTransaction;
            gI2cControllerMessage.txnCount = 1U;
            gI2cControllerMessage.targetAddress = I2C_TARGET_ADDR;
            gI2cControllerMessage.timeout = I2C_WAIT_FOREVER;
            gI2cControllerMessage.controllerMode = true;
            gI2cControllerMessage.expandSA = false;

            DebugP_log("[I2C Controller] Transaction 3: Start (read 2)...\r\n");
            status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
        }
    }

    if(status == I2C_STS_SUCCESS)
    {
        /* Wait for transaction 3 - wait for BOTH peripheral and controller */
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("[I2C Controller] Received Data %u %u !!!\r\n",
                   gRxControllerBuffer[0], gRxControllerBuffer[1]);
        DebugP_log("[I2C Peripheral] Transmitted Data %u %u !!!\r\n",
                   gTxPeripheralBuffer[0], gTxPeripheralBuffer[1]);

        DebugP_log("\r\n[I2C] All 3 transactions completed successfully!!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    /* Disable and deregister interrupts */
    HwiP_disableInt(gI2cControllerIntrNum);
    HwiP_disableInt(gI2cPeripheralIntrNum);
    HwiP_setVecAddr(gI2cControllerIntrNum, 0);
    HwiP_setVecAddr(gI2cPeripheralIntrNum, 0);

    /* Close drivers */
    Board_driversClose();
    Drivers_close();
}
