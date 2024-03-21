/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/soc_config.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

/* I2C Interrupt Priority */
#define I2C_INTERRUPT_PRIORITY          (4U)

/* Number of bytes to write to and read from EEPROM */
#define BYTE_COUNT                      (10U)

I2CLLD_Handle gI2cLldHandle0;
I2C_Memory_ExtendedParams mem_extendedParams_read;
I2C_Memory_ExtendedParams mem_extendedParams_write;

uint8_t txBuf[BYTE_COUNT];
uint8_t rxBuf[BYTE_COUNT];

uint32_t gI2cVimStsAddr, intrNum, gI2cVimStsClrMask, intcBaseAddr;
uint32_t gI2CTransferMutex = MUTEX_ARM_UNLOCKED;

extern HwiP_Config gHwiConfig;
extern uint32_t Board_i2cGetEepromDeviceAddr(void);
extern uint16_t Board_i2cGetEepromMemAddr(void);
extern uint8_t Board_i2cGetEepromAddrSize(void);

/* Transfer Complete Callback Function Declaration */
void I2C_lld_transferCompleteCallback_implementation(void * args,
                                            const I2CLLD_Message * msg,
                                            int32_t transferStatus);

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_ISR(void);

void i2c_memory_read_interrupt_main(void *arg)
{
    int32_t status = I2C_STS_SUCCESS;
    uint32_t count = 0U;
    uint16_t eepromMemAddr = Board_i2cGetEepromMemAddr();
    uint8_t eepromAddrSize = Board_i2cGetEepromAddrSize();

    Drivers_open();
    Board_driversOpen();

    DebugP_log("[I2C] LLD Memory Read test Started ... !!!\r\n");

    gI2cLldHandle0 = (I2CLLD_Handle)(gI2cLldHandle[0]);

    intrNum = gI2cLldHandle0->intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gI2cVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gI2cVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    /* Assign Transfer Complete Callback Function */
    gI2cLldHandle0->transferCompleteCallback = I2C_lld_transferCompleteCallback_implementation;

    /* Register Interrupt */
    HwiP_setPri(intrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_setAsPulse(intrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_setVecAddr(intrNum, (uintptr_t)&App_I2C_ISR);
    HwiP_enableInt(intrNum);

    /* Initialize I2C_Memory_ExtendedParams write object */
    mem_extendedParams_write.memAddr = eepromMemAddr;
    mem_extendedParams_write.memAddrSize = eepromAddrSize;
    mem_extendedParams_write.extendedParams.deviceAddress =
                                Board_i2cGetEepromDeviceAddr();
    mem_extendedParams_write.extendedParams.buffer = txBuf;
    mem_extendedParams_write.extendedParams.size = BYTE_COUNT;
    mem_extendedParams_write.extendedParams.expandSA = false;

    /* Fill write Buffer */
    for(count = 0; count < BYTE_COUNT; count++)
    {
        txBuf[count] = (uint8_t)count;
    }

    /* Initialize I2C_Memory_ExtendedParams read object */
    mem_extendedParams_read.memAddr = eepromMemAddr;
    mem_extendedParams_read.memAddrSize = eepromAddrSize;
    mem_extendedParams_read.extendedParams.deviceAddress =
                                Board_i2cGetEepromDeviceAddr();
    mem_extendedParams_read.extendedParams.buffer = rxBuf;
    mem_extendedParams_read.extendedParams.size = BYTE_COUNT;
    mem_extendedParams_read.extendedParams.expandSA = false;

    DebugP_log("[I2C] Application will write %d consecutive bytes in interrupt Mode to EEPROM memory address 0x%x ...\r\n", \
                mem_extendedParams_write.extendedParams.size, \
                mem_extendedParams_write.memAddr);

    /* Lock Mutex */
    gI2CTransferMutex = MUTEX_ARM_LOCKED;
    /* Initiate Memory Read operation */
    status = I2C_lld_mem_writeIntr(gI2cLldHandle0, &mem_extendedParams_write);
    /* Wait for the Mutex to Unlock */
    while(try_lock_mutex(&gI2CTransferMutex) == MUTEX_ARM_LOCKED);

    /* To ensure that eeprom is ready, added delay of 4ms */
    ClockP_usleep(4000);

    if(status == I2C_STS_SUCCESS)
    {
        DebugP_log("[I2C] Application will read %d consecutive bytes in interrupt Mode from EEPROM memory address 0x%x ...\r\n", \
                    mem_extendedParams_read.extendedParams.size, \
                    mem_extendedParams_read.memAddr);

        /* Lock Mutex */
        gI2CTransferMutex = MUTEX_ARM_LOCKED;
        /* Initiate Memory Read operation */
        status = I2C_lld_mem_readIntr(gI2cLldHandle0, &mem_extendedParams_read);
        /* Wait for the Mutex to Unlock */
        while(try_lock_mutex(&gI2CTransferMutex) == MUTEX_ARM_LOCKED);
    }

    /* De-Register Interrupt */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_disableInt(intrNum);

    if(status == I2C_STS_SUCCESS)
    {
        for(count = 0; count < 10; count++){
            DebugP_log("[I2C] Data at address 0x%x : 0x%x \r\n", (eepromMemAddr + count), rxBuf[count]);
        }
    }
    else
    {
        DebugP_log("[I2C] Memory read in interrupt mode Failed ... !!! \r\n");
    }

    if(status == I2C_STS_SUCCESS)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(I2C_lld_controllerIsr, \
                                      gI2cLldHandle0, \
                                      intrNum, \
                                      gI2cVimStsAddr, \
                                      gI2cVimStsClrMask,
                                      intcBaseAddr);
}

void I2C_lld_transferCompleteCallback_implementation (void * args, const I2CLLD_Message * msg, int32_t transferStatus)
{
    /* Unlock Mutex */
    unlock_mutex(&gI2CTransferMutex);
}
