/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/i2c.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/ipc_notify.h>

/* Number of INA devices installed */
#define NUM_OF_INA_DEVICES                (0x01U)
#define EXPECTED_R5_POWER_DROP             (70U)

/*Voltage rail to be monitored*/
#define RAIL_1V2                   (0x01) //1 - 1.2V rail to be monitored
#define MAX_EXP_CURRENT            (float)(2.5)

#if INA226
/* INA Register addresses*/
#define INA226_REG_CONFIG           (0x00)
#define INA226_REG_SHUNTVOLTAGE     (0x01)
#define INA226_REG_BUSVOLTAGE       (0x02)
#define INA226_REG_POWER            (0x03)
#define INA226_REG_CURRENT          (0x04)
#define INA226_REG_CALIBRATION      (0x05)
#define INA226_REG_MASKENABLE       (0x06)
#define INA226_REG_ALERTLIMIT       (0x07)


/* Values of the registers to be programmed */

/* Default value of Configuration register configures: continuous shunt and
 * bus measurement mode, Shunt voltage conversion time and Bus voltage conversion
 * time to 1.1ms each and Number of Averages to 1
 */
#define INA226_REG_CONFIG_MSB_VALUE       (0x41) /* Default value */
#define INA226_REG_CONFIG_LSB_VALUE       (0x27) /* Default value */

#define INA226_REG_CALIBRATION_MSB_VALUE  (0x64) /* Input from the user */
#define INA226_REG_CALIBRATION_LSB_VALUE  (0x00) /* Input from the user */

#define INA226_SHUNT_VOLTAGE_LSB_VAL      (float)(2.50)    /* constant 2.5  uV/bit */
#define INA226_CURRENT_LSB_VAL            (float)(100.00)  /* constant uA/bit */
#define INA226_BUS_VOLTAGE_LSB_VAL        (float)(1.25)    /* constant 1.25  mV/bit */
#define INA226_POWER_LSB_VAL              (float)(5000.00) /* constant 50*CURRENT_LSB */

#endif

#if INA228
/* INA Register addresses*/
#define INA228_REG_CONFIG           (0x00)
#define INA228_REG_ADC_CONFIG       (0x01)
#define INA228_REG_SHUNT_CAL        (0x02)
#define INA228_REG_SHUNT_TEMPCO     (0x03)
#define INA228_REG_VSHUNT           (0x04)
#define INA228_REG_VBUS             (0x05)
#define INA228_REG_DIETEMP          (0x06)
#define INA228_REG_CURRENT          (0x07)
#define INA228_REG_POWER            (0x08)
#define INA228_REG_ENERGY           (0x09)
#define INA228_REG_CHARGE           (0x0A)


/* Values of the registers to be programmed */

/* Default value of Configuration register configures: continuous shunt and
 * bus measurement mode, Shunt voltage conversion time and Bus voltage conversion
 * time to 1.1ms each and Number of Averages to 1
 */
#define INA228_REG_CONFIG_MSB_VALUE       (0x00) /* Input from the user */
#define INA228_REG_CONFIG_LSB_VALUE       (0x00) /* Input from the user */

#define INA228_REG_ADC_CONFIG_MSB_VALUE   (0xFB) /* Default value */
#define INA228_REG_ADC_CONFIG_LSB_VALUE   (0x68) /* Default value */

#define INA228_REG_SHUNT_CAL_MSB_VALUE    (0x33) /* Input from the user */
#define INA228_REG_SHUNT_CAL_LSB_VALUE    (0x33) /* Input from the user */

#define INA228_SHUNT_VOLTAGE_LSB_VAL      (float)(312.5)                      /* constant 312.5 nV/bit */
#define INA228_CURRENT_LSB_VAL            (float)(100)                        /* constant  uA/bit */
#define INA228_BUS_VOLTAGE_LSB_VAL        (float)(195.3125)                   /* constant 195.3125 uV/bit */
#define INA228_POWER_LSB_VAL              (float)(3.2*INA228_CURRENT_LSB_VAL/1000) /* constant 3.2*CURRENT_LSB */

#endif

extern uint8_t Board_get_inaAddr(void);

uint32_t gAppPowerStats[NUM_OF_INA_DEVICES];
uint32_t gAppCurrentStats[NUM_OF_INA_DEVICES];

/* client ID that is used to send and receive messages */
uint32_t gClientId = 4u;

/* semaphore's used to indicate a main core */
SemaphoreP_Object gMainDoneSem;

/* main core that starts the message exchange */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;
/* remote cores that echo messages from main core */
uint32_t gRemoteCoreId = CSL_CORE_ID_R5FSS1_0;

int32_t I2C_configCurrentMonitor(I2C_Handle i2cHandle, uint8_t  deviceAddress)
{
    uint8_t         txBuffer[3];
    int32_t         status;
    I2C_Transaction i2cTransaction;

    #if INA226
    /* found INA */
    /* Configure Configuration register */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 3;
    i2cTransaction.targetAddress = deviceAddress;
    txBuffer[0] = INA226_REG_CONFIG;
    txBuffer[1] = INA226_REG_CONFIG_MSB_VALUE;
    txBuffer[2] = INA226_REG_CONFIG_LSB_VALUE;

    status = I2C_transfer(i2cHandle, &i2cTransaction);
    if(status == SystemP_SUCCESS)
    {
        /* Configure Calibration register */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 3;
        i2cTransaction.targetAddress = deviceAddress;
        txBuffer[0] = INA226_REG_CALIBRATION;
        txBuffer[1] = INA226_REG_CALIBRATION_MSB_VALUE;
        txBuffer[2] = INA226_REG_CALIBRATION_LSB_VALUE;

        status = I2C_transfer(i2cHandle, &i2cTransaction);
    }
    #endif
    #if INA228
    /* found INA */
    /* Configure ADC Configuration register */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 3;
    i2cTransaction.targetAddress = deviceAddress;
    txBuffer[0] = INA228_REG_ADC_CONFIG;
    txBuffer[1] = INA228_REG_ADC_CONFIG_MSB_VALUE;
    txBuffer[2] = INA228_REG_ADC_CONFIG_LSB_VALUE;

    status = I2C_transfer(i2cHandle, &i2cTransaction);
    if(status == SystemP_SUCCESS)
    {
        /* Configure Shunt Calibration register */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = txBuffer;
        i2cTransaction.writeCount = 3;
        i2cTransaction.targetAddress = deviceAddress;
        txBuffer[0] = INA228_REG_SHUNT_CAL;
        txBuffer[1] = INA228_REG_SHUNT_CAL_MSB_VALUE;
        txBuffer[2] = INA228_REG_SHUNT_CAL_LSB_VALUE;

        status = I2C_transfer(i2cHandle, &i2cTransaction);
    }
    #endif

    return status;
}

int32_t I2C_measureCurrent(I2C_Handle i2cHandle, uint8_t  deviceAddress)
{
    uint8_t         txBuffer[3];
    uint8_t         rxBuffer[5];
    int32_t         status;
    I2C_Transaction i2cTransaction;

    #if INA226
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.targetAddress = deviceAddress;
    txBuffer[0] = INA226_REG_CURRENT;

    status = I2C_transfer(i2cHandle, &i2cTransaction);
    if(status == SystemP_SUCCESS)
    {
        /* Read current register*/
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 2;
        i2cTransaction.targetAddress = deviceAddress;
        status = I2C_transfer(i2cHandle, &i2cTransaction);
        if(status == SystemP_SUCCESS)
        {
            uint16_t regData = ((((uint16_t) rxBuffer[0]) << 8) | ((uint16_t) rxBuffer[1]));
            gAppCurrentStats[0] = (uint32_t )(regData * INA226_CURRENT_LSB_VAL);
        }
    }
    #endif

    #if INA228
    /* Select Current register */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.targetAddress = deviceAddress;
    txBuffer[0] = INA228_REG_CURRENT;

    status = I2C_transfer(i2cHandle, &i2cTransaction);
    if(status == SystemP_SUCCESS)
    {
       /* Read current register*/
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 3;
        i2cTransaction.targetAddress = deviceAddress;
        status = I2C_transfer(i2cHandle, &i2cTransaction);
        if(status == SystemP_SUCCESS)
        {
            uint32_t regData = ((((uint32_t) rxBuffer[0]) << 16)
                    | (((uint32_t) rxBuffer[1]) << 8)
                    | ((uint32_t) rxBuffer[2]));
            gAppCurrentStats[0] = (uint32_t )(INA228_CURRENT_LSB_VAL * (regData >> 4));
        }
    }
    #endif

    return status;
}

int32_t I2C_measurePower(I2C_Handle i2cHandle, uint8_t  deviceAddress)
{
    uint8_t         txBuffer[3];
    uint8_t         rxBuffer[5];
    int32_t         status;
    I2C_Transaction i2cTransaction;

    #if INA226
    /* Select Power register */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.targetAddress = deviceAddress;
    txBuffer[0] = INA226_REG_POWER;

    status = I2C_transfer(i2cHandle, &i2cTransaction);
    if(status == SystemP_SUCCESS)
    {
        /* Read Power register*/
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 2;
        i2cTransaction.targetAddress = deviceAddress;
        status = I2C_transfer(i2cHandle, &i2cTransaction);
        if(status == SystemP_SUCCESS)
        {
            uint32_t regData = ((((uint32_t) rxBuffer[0]) << 8) | ((uint32_t) rxBuffer[1]));
            gAppPowerStats[0] = (uint32_t )((INA226_POWER_LSB_VAL * regData)/1000);
        }
    }
    #endif
    #if INA228
       /* Select Power register */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.targetAddress = deviceAddress;
    txBuffer[0] = INA228_REG_POWER;

    status = I2C_transfer(i2cHandle, &i2cTransaction);
    if(status == SystemP_SUCCESS)
    {
        /* Read Power register*/
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 3;
        i2cTransaction.targetAddress = deviceAddress;
        status = I2C_transfer(i2cHandle, &i2cTransaction);
        if(status == SystemP_SUCCESS)
        {
            uint32_t regData = ((((uint32_t) rxBuffer[0]) << 16)
                    | (((uint32_t) rxBuffer[1]) << 8)
                    | ((uint32_t) rxBuffer[2]));
            gAppPowerStats[0] = (uint32_t )(INA228_POWER_LSB_VAL * regData);
        }
    }
    #endif

    return status;
}

void wfe_demo_handler_main_core(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    /* there is one semaphore for each core ID, so post the semaphore for the remote core that
     * has finished all message exchange iterations
     */
    SemaphoreP_post(&gMainDoneSem);
}


void wfi_standby_demo_main()
{
    uint8_t         deviceAddress;
    I2C_Handle      i2cHandle;
    uint32_t msgValue = 0;
    uint32_t status = SystemP_SUCCESS;
    uint32_t currentVal1, currentVal2 = 0;
    uint32_t powerVal1, powerVal2 = 0;

    Drivers_open();
    Board_driversOpen();

    i2cHandle = gI2cHandle[CONFIG_I2C0];

    /* create completion semaphores for all cores */
    SemaphoreP_constructBinary(&gMainDoneSem, 0);

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(gClientId, wfe_demo_handler_main_core, NULL);
    DebugP_assert(status==SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* Determine if INA sensor is present at 1.2V rail */
    deviceAddress = Board_get_inaAddr();
    status = I2C_probe(i2cHandle, deviceAddress);
    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[I2C] INA found at device address 0x%02x \r\n", deviceAddress);
    }
    else
    {
        DebugP_logError("[I2C] INA not found at device address 0x%02x \r\n", deviceAddress);
    }

    I2C_configCurrentMonitor(i2cHandle, deviceAddress);

    DebugP_log("[WFI] Values before wfi !!!\r\n");
    I2C_measureCurrent(i2cHandle, deviceAddress);
    I2C_measurePower(i2cHandle, deviceAddress);
    currentVal1 = gAppCurrentStats[0];
    powerVal1 = gAppPowerStats[0];
    DebugP_log("[WFI] power= %u mW , current=%u uA\r\n", powerVal1, currentVal1);
    status = IpcNotify_sendMsg(gRemoteCoreId, gClientId, msgValue, 1);
    SemaphoreP_pend(&gMainDoneSem, SystemP_WAIT_FOREVER);

    DebugP_log("[WFI] Values after wfi !!!\r\n");
    I2C_measureCurrent(i2cHandle, deviceAddress);
    I2C_measurePower(i2cHandle, deviceAddress);
    currentVal2 = gAppCurrentStats[0];
    powerVal2 = gAppPowerStats[0];
    DebugP_log("[WFI] power= %u mW , current=%u uA\r\n", powerVal2, currentVal2);
    status = IpcNotify_sendMsg(gRemoteCoreId, gClientId, msgValue, 1);
    DebugP_assert(status==SystemP_SUCCESS);

    if((powerVal2 <= powerVal1) && (currentVal2 <= currentVal1) && ((powerVal1 - powerVal2) >= EXPECTED_R5_POWER_DROP))
    {
        DebugP_log("[WFI] Actual Power Drop = %u mW\r\n", (powerVal1 - powerVal2));
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
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
