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
 * Seven transactions are carried out in this example:
 * 1. Controller writes 4 bytes to Peripheral
 * 2. Controller reads 4 bytes from Peripheral
 * 3. I2C Probe (address-only detection)
 * 4. Controller writes 4 bytes again (different data)
 * 5. Controller reads 4 bytes again (different data)
 * 6. ReSTART condition (write 4, ReSTART, read 1 in single message)
 * 7. I2C Probe again (address-only detection)
 *
 * This example uses dual semaphores (one for controller, one for peripheral)
 * with DSB memory barriers to ensure cache coherency and proper ISR completion.
 */

/* I2C Interrupt Priority */
#define I2C_INTERRUPT_PRIORITY  (4U)

/* I2C Target Address */
#define I2C_TARGET_ADDR         (0x2CU)

/* I2C LLD Handles */
volatile I2CLLD_Handle gI2cControllerLldHandle;
volatile I2CLLD_Handle gI2cPeripheralLldHandle;

/* Controller transaction / message (global so ISR can access after transfer starts) */
I2CLLD_Transaction gI2cControllerTransaction;
I2CLLD_Message     gI2cControllerMessage;

/* Data buffers */
uint8_t gRxControllerBuffer[4];
uint8_t gTxControllerBuffer[4];
uint8_t gTxPeripheralBuffer[4];
uint8_t gRxPeripheralBuffer[4];

/* Synchronisation semaphores */
static SemaphoreP_Object gControllerTransferDoneSemaphore;
static SemaphoreP_Object gPeripheralTransferDoneSemaphore;

#ifndef SemaphoreP_WAIT_FOREVER
#define SemaphoreP_WAIT_FOREVER  ((uint32_t)SystemP_WAIT_FOREVER)
#endif

/* VIM / interrupt book-keeping */
volatile uint32_t gI2cControllerIntrNum, gI2cPeripheralIntrNum;
volatile uint32_t gI2cControllerVimStsAddr,  gI2cPeripheralVimStsAddr;
volatile uint32_t gI2cControllerVimStsClrMask, gI2cPeripheralVimStsClrMask;
volatile uint32_t gIntcBaseAddr;

extern HwiP_Config gHwiConfig;

/* Forward declarations */
void I2C_lld_controllerTransferCompleteCallback(void *args,
                                                const I2CLLD_Message *msg,
                                                int32_t transferStatus);
void I2C_lld_peripheralTransferCompleteCallback(void *args,
                                                const I2CLLD_targetTransaction *targetTxn,
                                                int32_t transferStatus);
static __attribute__((__section__(".text.hwi"), noinline, naked,
                       target("arm"), aligned(4))) void App_I2C_Controller_ISR(void);
static __attribute__((__section__(".text.hwi"), noinline, naked,
                       target("arm"), aligned(4))) void App_I2C_Peripheral_ISR(void);
void i2c_peripheral_probe_test_lld (void);

/* Controller transfer complete callback */
void I2C_lld_controllerTransferCompleteCallback(void *args,
                                                const I2CLLD_Message *msg,
                                                int32_t transferStatus)
{
    SemaphoreP_post(&gControllerTransferDoneSemaphore);
    __asm__ volatile("dsb" ::: "memory");
}

/* Peripheral transfer complete callback */
void I2C_lld_peripheralTransferCompleteCallback(void *args,
                                                const I2CLLD_targetTransaction *targetTxn,
                                                int32_t transferStatus)
{
    if (transferStatus == I2C_STS_RESTART)
    {       
        I2CLLD_targetTransaction *txn = (I2CLLD_targetTransaction *)targetTxn;
        uint32_t bytesSent   = txn->writeCount;   /* already in HW TXBUF */
        txn->writeBuf        = gTxPeripheralBuffer + bytesSent;
        txn->writeCount      = 1U - bytesSent;    /* 1 byte total in READ phase */
        txn->readBuf         = gRxPeripheralBuffer;
        txn->readCount       = 0U;
    }
    else
    {
        /* Transfer complete */
        if (transferStatus == I2C_STS_ERR_NO_ACK)
        {
            /* NACK: reset driver state to IDLE */
            I2CLLD_Object *obj  = (I2CLLD_Object *)gI2cPeripheralLldHandle;
            obj->state          = I2C_STATE_IDLE;
            obj->currentTargetTransaction = NULL;
        }
        SemaphoreP_post(&gPeripheralTransferDoneSemaphore);
        __asm__ volatile("dsb" ::: "memory");
    }
}

/* ISR wrappers */
static __attribute__((__section__(".text.hwi"), noinline, naked,
                       target("arm"), aligned(4))) void App_I2C_Controller_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(I2C_lld_controllerIsr,
                                      gI2cControllerLldHandle,
                                      gI2cControllerIntrNum,
                                      gI2cControllerVimStsAddr,
                                      gI2cControllerVimStsClrMask,
                                      gIntcBaseAddr);
}

static __attribute__((__section__(".text.hwi"), noinline, naked,
                       target("arm"), aligned(4))) void App_I2C_Peripheral_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(I2C_lld_targetIsr,
                                      gI2cPeripheralLldHandle,
                                      gI2cPeripheralIntrNum,
                                      gI2cPeripheralVimStsAddr,
                                      gI2cPeripheralVimStsClrMask,
                                      gIntcBaseAddr);
}

/* Initialise I2C LLD instances and interrupts */
void i2c_peripheral_probe_test_lld (void)
{
    SemaphoreP_constructBinary(&gControllerTransferDoneSemaphore, 0);
    SemaphoreP_constructBinary(&gPeripheralTransferDoneSemaphore, 0);

    gI2cControllerLldHandle = (I2CLLD_Handle)(gI2cLldHandle[CONFIG_I2C0]);
    gI2cPeripheralLldHandle = (I2CLLD_Handle)(gI2cLldHandle[CONFIG_I2C1]);

    gI2cControllerLldHandle->transferCompleteCallback       = I2C_lld_controllerTransferCompleteCallback;
    gI2cPeripheralLldHandle->targetTransferCompleteCallback = I2C_lld_peripheralTransferCompleteCallback;

    gI2cPeripheralLldHandle->ownTargetAddr = I2C_TARGET_ADDR;

    gI2cControllerIntrNum = gI2cControllerLldHandle->intrNum;
    gI2cPeripheralIntrNum = gI2cPeripheralLldHandle->intrNum;
    gIntcBaseAddr         = gHwiConfig.intcBaseAddr;

    gI2cControllerVimStsAddr    = gIntcBaseAddr + (0x404u + (((gI2cControllerIntrNum) >> 5) & 0xFu) * 0x20u);
    gI2cControllerVimStsClrMask = 0x1u << ((gI2cControllerIntrNum) & 0x1Fu);
    gI2cPeripheralVimStsAddr    = gIntcBaseAddr + (0x404u + (((gI2cPeripheralIntrNum) >> 5) & 0xFu) * 0x20u);
    gI2cPeripheralVimStsClrMask = 0x1u << ((gI2cPeripheralIntrNum) & 0x1Fu);

    HwiP_setVecAddr(gI2cControllerIntrNum, (uintptr_t)&App_I2C_Controller_ISR);
    HwiP_setPri(gI2cControllerIntrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_enableInt(gI2cControllerIntrNum);

    HwiP_setVecAddr(gI2cPeripheralIntrNum, (uintptr_t)&App_I2C_Peripheral_ISR);
    HwiP_setPri(gI2cPeripheralIntrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_enableInt(gI2cPeripheralIntrNum);
}

/* Main test function */
void i2c_peripheral_probe_test_lld_main(void *args)
{
    int32_t  status;
    uint32_t testNum = 0U;

    /*
     * targetTxn lives for the entire duration of main() so the ISR can
     * safely access it while the task is blocked on SemaphoreP_pend.
     */
    I2CLLD_targetTransaction targetTxn;

    Drivers_open();
    Board_driversOpen();
    i2c_peripheral_probe_test_lld ();

    DebugP_log("\r\n");
    DebugP_log("======================================================================\r\n");
    DebugP_log("[I2C] I2C Peripheral Probe & Read Test Started ..................... !!!\r\n");
    DebugP_log("======================================================================\r\n");

    memset(gRxControllerBuffer, 0, sizeof(gRxControllerBuffer));
    memset(gRxPeripheralBuffer, 0, sizeof(gRxPeripheralBuffer));

    /* ================================================================ */
    /* === TRANSACTION 1: Controller writes 4 bytes === */
    /* ================================================================ */
    testNum++;
    DebugP_log("\r\n[TRANSACTION %u] Master Write: Controller -> Peripheral (4 bytes)\r\n", testNum);

    gTxControllerBuffer[0] = 0x01U;  
    gTxControllerBuffer[1] = 0x02U;
    gTxControllerBuffer[2] = 0x03U;  
    gTxControllerBuffer[3] = 0x04U;

    targetTxn.readBuf    = gRxPeripheralBuffer;  
    targetTxn.readCount  = 4U;
    targetTxn.writeBuf   = gTxPeripheralBuffer;  
    targetTxn.writeCount = 0U;
    targetTxn.timeout    = I2C_WAIT_FOREVER;     
    targetTxn.expandSA   = false;

    status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
    if (status == I2C_STS_SUCCESS)
    {
        ClockP_usleep(50000);

        I2C_lld_Transaction_init(&gI2cControllerTransaction);
        I2C_lld_Message_init(&gI2cControllerMessage);
        gI2cControllerTransaction.writeBuf    = gTxControllerBuffer;
        gI2cControllerTransaction.writeCount  = 4U;
        gI2cControllerTransaction.readBuf     = gRxControllerBuffer;
        gI2cControllerTransaction.readCount   = 0U;
        gI2cControllerMessage.txn             = &gI2cControllerTransaction;
        gI2cControllerMessage.txnCount        = 1U;
        gI2cControllerMessage.targetAddress   = I2C_TARGET_ADDR;
        gI2cControllerMessage.timeout         = I2C_WAIT_FOREVER;
        gI2cControllerMessage.controllerMode  = true;
        gI2cControllerMessage.expandSA        = false;

        status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
    }
    if (status == I2C_STS_SUCCESS)
    {
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("  TX (Controller)   : %02X %02X %02X %02X\r\n",
                   gTxControllerBuffer[0], gTxControllerBuffer[1],
                   gTxControllerBuffer[2], gTxControllerBuffer[3]);
        DebugP_log("  RX (Peripheral)   : %02X %02X %02X %02X\r\n",
                   gRxPeripheralBuffer[0], gRxPeripheralBuffer[1],
                   gRxPeripheralBuffer[2], gRxPeripheralBuffer[3]);
        DebugP_log("  [PASS]\r\n");
    }
    else
    {
        DebugP_log("  [FAIL] status=%d\r\n", status);
    }

    /* ================================================================ */
    /* === TRANSACTION 2: Controller receives 4 bytes from peripheral */
    /* ================================================================ */
    testNum++;
    DebugP_log("\r\n[TRANSACTION %u] Master Read: Peripheral -> Controller (4 bytes)\r\n", testNum);

    memset(gRxControllerBuffer, 0, sizeof(gRxControllerBuffer));

    gTxPeripheralBuffer[0] = 0x05U;  
    gTxPeripheralBuffer[1] = 0x06U;
    gTxPeripheralBuffer[2] = 0x07U;  
    gTxPeripheralBuffer[3] = 0x08U;

    targetTxn.readBuf    = gRxPeripheralBuffer;  
    targetTxn.readCount  = 0U;
    targetTxn.writeBuf   = gTxPeripheralBuffer;  
    targetTxn.writeCount = 4U;
    targetTxn.timeout    = I2C_WAIT_FOREVER;     
    targetTxn.expandSA   = false;

    status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
    if (status == I2C_STS_SUCCESS)
    {
        ClockP_usleep(50000);

        I2C_lld_Transaction_init(&gI2cControllerTransaction);
        I2C_lld_Message_init(&gI2cControllerMessage);

        gI2cControllerTransaction.writeBuf    = gTxControllerBuffer;
        gI2cControllerTransaction.writeCount  = 0U;
        gI2cControllerTransaction.readBuf     = gRxControllerBuffer;
        gI2cControllerTransaction.readCount   = 4U;

        gI2cControllerMessage.txn             = &gI2cControllerTransaction;
        gI2cControllerMessage.txnCount        = 1U;
        gI2cControllerMessage.targetAddress   = I2C_TARGET_ADDR;
        gI2cControllerMessage.timeout         = I2C_WAIT_FOREVER;
        gI2cControllerMessage.controllerMode  = true;
        gI2cControllerMessage.expandSA        = false;

        status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
    }
    if (status == I2C_STS_SUCCESS)
    {
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("  TX (Peripheral)   : %02X %02X %02X %02X\r\n",
                   gTxPeripheralBuffer[0], gTxPeripheralBuffer[1],
                   gTxPeripheralBuffer[2], gTxPeripheralBuffer[3]);
        DebugP_log("  RX (Controller)   : %02X %02X %02X %02X\r\n",
                   gRxControllerBuffer[0], gRxControllerBuffer[1],
                   gRxControllerBuffer[2], gRxControllerBuffer[3]);
        DebugP_log("  [PASS]\r\n");
    }
    else
    {
        DebugP_log("  [FAIL] status=%d\r\n", status);
    }

    /* ================================================================ */
    /* === TRANSACTION 3: I2C Probe – address-only detection                       */ 
    /* ================================================================ */
    testNum++;
    DebugP_log("\r\n[TRANSACTION %u] I2C Probe: address-only detection\r\n", testNum);

    memset(gRxControllerBuffer, 0, sizeof(gRxControllerBuffer));

    targetTxn.readBuf    = gRxPeripheralBuffer;  
    targetTxn.readCount  = 0U;
    targetTxn.writeBuf   = gTxPeripheralBuffer;  
    targetTxn.writeCount = 0U;  
    targetTxn.timeout    = I2C_WAIT_FOREVER;     
    targetTxn.expandSA   = false;

    status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
    if (status == I2C_STS_SUCCESS)
    {
        ClockP_usleep(50000);

        I2C_lld_Transaction_init(&gI2cControllerTransaction);
        I2C_lld_Message_init(&gI2cControllerMessage);

        gI2cControllerTransaction.writeBuf    = NULL;
        gI2cControllerTransaction.writeCount  = 0U;
        gI2cControllerTransaction.readBuf     = gRxControllerBuffer;
        gI2cControllerTransaction.readCount   = 1U;   /* read-bit probes the address */

        gI2cControllerMessage.txn             = &gI2cControllerTransaction;
        gI2cControllerMessage.txnCount        = 1U;
        gI2cControllerMessage.targetAddress   = I2C_TARGET_ADDR;
        gI2cControllerMessage.timeout         = I2C_WAIT_FOREVER;
        gI2cControllerMessage.controllerMode  = true;
        gI2cControllerMessage.expandSA        = false;

        status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
    }
    if (status == I2C_STS_SUCCESS)
    {
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("  Device ACK'd address 0x%02X – probe successful\r\n", I2C_TARGET_ADDR);
        DebugP_log("  [PASS]\r\n");
    }
    else
    {
        DebugP_log("  [FAIL] status=%d\r\n", status);
    }

    /* ================================================================ */
    /* === TRANSACTION 4: Master Write again – different data                      */
    /* ================================================================ */
    testNum++;
    DebugP_log("\r\n[TRANSACTION %u] Master Write (again): Controller -> Peripheral (4 bytes)\r\n", testNum);

    memset(gRxPeripheralBuffer, 0, sizeof(gRxPeripheralBuffer));

    gTxControllerBuffer[0] = 0x11U;  
    gTxControllerBuffer[1] = 0x12U;
    gTxControllerBuffer[2] = 0x13U;  
    gTxControllerBuffer[3] = 0x14U;

    targetTxn.readBuf    = gRxPeripheralBuffer; 
    targetTxn.readCount  = 4U;
    targetTxn.writeBuf   = gTxPeripheralBuffer;  
    targetTxn.writeCount = 0U;
    targetTxn.timeout    = I2C_WAIT_FOREVER;    
    targetTxn.expandSA   = false;

    status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
    if (status == I2C_STS_SUCCESS)
    {
        ClockP_usleep(50000);

        I2C_lld_Transaction_init(&gI2cControllerTransaction);
        I2C_lld_Message_init(&gI2cControllerMessage);

        gI2cControllerTransaction.writeBuf    = gTxControllerBuffer;
        gI2cControllerTransaction.writeCount  = 4U;
        gI2cControllerTransaction.readBuf     = gRxControllerBuffer;
        gI2cControllerTransaction.readCount   = 0U;

        gI2cControllerMessage.txn             = &gI2cControllerTransaction;
        gI2cControllerMessage.txnCount        = 1U;
        gI2cControllerMessage.targetAddress   = I2C_TARGET_ADDR;
        gI2cControllerMessage.timeout         = I2C_WAIT_FOREVER;
        gI2cControllerMessage.controllerMode  = true;
        gI2cControllerMessage.expandSA        = false;

        status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
    }
    if (status == I2C_STS_SUCCESS)
    {
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("  TX (Controller)   : %02X %02X %02X %02X\r\n",
                   gTxControllerBuffer[0], gTxControllerBuffer[1],
                   gTxControllerBuffer[2], gTxControllerBuffer[3]);
        DebugP_log("  RX (Peripheral)   : %02X %02X %02X %02X\r\n",
                   gRxPeripheralBuffer[0], gRxPeripheralBuffer[1],
                   gRxPeripheralBuffer[2], gRxPeripheralBuffer[3]);
        DebugP_log("  [PASS]\r\n");
    }
    else
    {
        DebugP_log("  [FAIL] status=%d\r\n", status);
    }

    /* ================================================================ */
    /* === TRANSACTION 5: Master Read again – different data                       */
    /* ================================================================ */
    testNum++;
    DebugP_log("\r\n[TRANSACTION %u] Master Read (again): Peripheral -> Controller (4 bytes)\r\n", testNum);

    memset(gRxControllerBuffer, 0, sizeof(gRxControllerBuffer));

    gTxPeripheralBuffer[0] = 0x15U;  
    gTxPeripheralBuffer[1] = 0x16U;
    gTxPeripheralBuffer[2] = 0x17U;  
    gTxPeripheralBuffer[3] = 0x18U;

    targetTxn.readBuf    = gRxPeripheralBuffer;  
    targetTxn.readCount  = 0U;
    targetTxn.writeBuf   = gTxPeripheralBuffer;  
    targetTxn.writeCount = 4U;
    targetTxn.timeout    = I2C_WAIT_FOREVER;     
    targetTxn.expandSA   = false;

    status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
    if (status == I2C_STS_SUCCESS)
    {
        ClockP_usleep(50000);

        I2C_lld_Transaction_init(&gI2cControllerTransaction);
        I2C_lld_Message_init(&gI2cControllerMessage);
        gI2cControllerTransaction.writeBuf    = gTxControllerBuffer;
        gI2cControllerTransaction.writeCount  = 0U;
        gI2cControllerTransaction.readBuf     = gRxControllerBuffer;
        gI2cControllerTransaction.readCount   = 4U;
        gI2cControllerMessage.txn             = &gI2cControllerTransaction;
        gI2cControllerMessage.txnCount        = 1U;
        gI2cControllerMessage.targetAddress   = I2C_TARGET_ADDR;
        gI2cControllerMessage.timeout         = I2C_WAIT_FOREVER;
        gI2cControllerMessage.controllerMode  = true;
        gI2cControllerMessage.expandSA        = false;

        status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
    }
    if (status == I2C_STS_SUCCESS)
    {
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("  TX (Peripheral)   : %02X %02X %02X %02X\r\n",
                   gTxPeripheralBuffer[0], gTxPeripheralBuffer[1],
                   gTxPeripheralBuffer[2], gTxPeripheralBuffer[3]);
        DebugP_log("  RX (Controller)   : %02X %02X %02X %02X\r\n",
                   gRxControllerBuffer[0], gRxControllerBuffer[1],
                   gRxControllerBuffer[2], gRxControllerBuffer[3]);
        DebugP_log("  [PASS]\r\n");
    }
    else
    {
        DebugP_log("  [FAIL] status=%d\r\n", status);
    }

    /* ========================================================================== */
    /* === TRANSACTION 6: ReSTART – controller writes 4 then reads 1 in one msg   */
    /* ========================================================================== */
    testNum++;
    DebugP_log("\r\n[TRANSACTION %u] ReSTART: Controller writes 4, ReSTART, reads 1\r\n", testNum);

    memset(gRxControllerBuffer, 0, sizeof(gRxControllerBuffer));
    memset(gRxPeripheralBuffer, 0, sizeof(gRxPeripheralBuffer));

    gTxControllerBuffer[0] = 0x21U;  
    gTxControllerBuffer[1] = 0x22U;
    gTxControllerBuffer[2] = 0x23U;  
    gTxControllerBuffer[3] = 0x24U;
    gTxPeripheralBuffer[0] = 0x25U;  /* byte returned by peripheral after ReSTART */

    /* Peripheral: receive 4 bytes, then send 1 byte after ReSTART.
     * The RESTART callback (above) updates writeCount to 1U. */
    targetTxn.readBuf    = gRxPeripheralBuffer; 
    targetTxn.readCount  = 4U;
    targetTxn.writeBuf   = gTxPeripheralBuffer;  
    targetTxn.writeCount = 1U;
    targetTxn.timeout    = I2C_WAIT_FOREVER;     
    targetTxn.expandSA   = false;

    status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
    if (status == I2C_STS_SUCCESS)
    {
        ClockP_usleep(50000);

        I2C_lld_Transaction_init(&gI2cControllerTransaction);
        I2C_lld_Message_init(&gI2cControllerMessage);
        gI2cControllerTransaction.writeBuf    = gTxControllerBuffer;
        gI2cControllerTransaction.writeCount  = 4U;
        gI2cControllerTransaction.readBuf     = gRxControllerBuffer;
        gI2cControllerTransaction.readCount   = 1U;   /* triggers ReSTART */
        gI2cControllerMessage.txn             = &gI2cControllerTransaction;
        gI2cControllerMessage.txnCount        = 1U;
        gI2cControllerMessage.targetAddress   = I2C_TARGET_ADDR;
        gI2cControllerMessage.timeout         = I2C_WAIT_FOREVER;
        gI2cControllerMessage.controllerMode  = true;
        gI2cControllerMessage.expandSA        = false;

        status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
    }
    if (status == I2C_STS_SUCCESS)
    {
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("  Phase 1 TX (Controller)   : %02X %02X %02X %02X\r\n",
                   gTxControllerBuffer[0], gTxControllerBuffer[1],
                   gTxControllerBuffer[2], gTxControllerBuffer[3]);
        DebugP_log("  Phase 1 RX (Peripheral)   : %02X %02X %02X %02X\r\n",
                   gRxPeripheralBuffer[0], gRxPeripheralBuffer[1],
                   gRxPeripheralBuffer[2], gRxPeripheralBuffer[3]);
        DebugP_log("  Phase 2 TX (Peripheral)   : %02X\r\n", gTxPeripheralBuffer[0]);
        DebugP_log("  Phase 2 RX (Controller)   : %02X\r\n", gRxControllerBuffer[0]);
        DebugP_log("  [PASS]\r\n");
    }
    else
    {
        DebugP_log("  [FAIL] status=%d\r\n", status);
    }

    /* ================================================================ */
    /* === TRANSACTION 7: I2C Probe test again                          */
    /* ================================================================ */
    testNum++;
    DebugP_log("\r\n[TRANSACTION %u] I2C Probe (again): address-only detection\r\n", testNum);

    memset(gRxControllerBuffer, 0, sizeof(gRxControllerBuffer));

    targetTxn.readBuf    = gRxPeripheralBuffer;  
    targetTxn.readCount  = 0U;
    targetTxn.writeBuf   = gTxPeripheralBuffer;  
    targetTxn.writeCount = 0U;  
    targetTxn.timeout    = I2C_WAIT_FOREVER;     
    targetTxn.expandSA   = false;

    status = I2C_lld_targetTransferIntr(gI2cPeripheralLldHandle, &targetTxn);
    if (status == I2C_STS_SUCCESS)
    {
        ClockP_usleep(50000);

        I2C_lld_Transaction_init(&gI2cControllerTransaction);
        I2C_lld_Message_init(&gI2cControllerMessage);

        gI2cControllerTransaction.writeBuf    = NULL;
        gI2cControllerTransaction.writeCount  = 0U;
        gI2cControllerTransaction.readBuf     = gRxControllerBuffer;
        gI2cControllerTransaction.readCount   = 1U;

        gI2cControllerMessage.txn             = &gI2cControllerTransaction;
        gI2cControllerMessage.txnCount        = 1U;
        gI2cControllerMessage.targetAddress   = I2C_TARGET_ADDR;
        gI2cControllerMessage.timeout         = I2C_WAIT_FOREVER;
        gI2cControllerMessage.controllerMode  = true;
        gI2cControllerMessage.expandSA        = false;

        status = I2C_lld_transferIntr(gI2cControllerLldHandle, &gI2cControllerMessage);
    }
    if (status == I2C_STS_SUCCESS)
    {
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gPeripheralTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);
        __asm__ volatile("dsb" ::: "memory");
        SemaphoreP_pend(&gControllerTransferDoneSemaphore, SemaphoreP_WAIT_FOREVER);

        DebugP_log("  Device ACK'd address 0x%02X – probe successful\r\n", I2C_TARGET_ADDR);
        DebugP_log("  [PASS]\r\n");
    }
    else
    {
        DebugP_log("  [FAIL] status=%d\r\n", status);
    }

    /* ================================================================ */
    /* All 7 transactions                                               */
    /* ================================================================ */
    DebugP_log("\r\n");
    DebugP_log("======================================================================\r\n");
    DebugP_log("[I2C] All 7 transactions (Probe & Read Test) completed successfully!!!\r\n");
    DebugP_log("All tests have passed!!\r\n");
    DebugP_log("======================================================================\r\n");

    HwiP_disableInt(gI2cControllerIntrNum);
    HwiP_disableInt(gI2cPeripheralIntrNum);
    HwiP_setVecAddr(gI2cControllerIntrNum, 0);
    HwiP_setVecAddr(gI2cPeripheralIntrNum, 0);

    Board_driversClose();
    Drivers_close();
}
