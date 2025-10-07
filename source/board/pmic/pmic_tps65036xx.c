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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <board/pmic.h>
#include <board/pmic/pmic_tps65036xx.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/i2c.h>
#include <pmic_core.h>
#include <pmic_power.h>
#include <pmic_irq.h>
#include <string.h>
#include <stdlib.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

SemaphoreP_Object gPmicCoreObj;


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t PMIC_regRead(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *rxbuf);
int32_t PMIC_regWrite(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, const uint8_t *txBuf);
 
void PMIC_criticalSectionStartFn(void);
void PMIC_criticalSectionStopFn(void);
PMIC_Handle PMIC_getHandle(uint32_t index);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

PMIC_Fxns gPmicFxns_TPS65036xx =
{
    .openFxn        = PMIC_tps65036xxOpen,
    .configureFxn   = PMIC_tps65036xxConfigure,
    .closeFxn       = PMIC_tps65036xxClose,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */



/**
 * @brief Initialize PMIC communication interface and resources.
 * This function initializes the PMIC communication interface and resources
 * based on the provided configuration and parameters.
 *
 * @param config Pointer to the PMIC configuration structure.
 * @param params Pointer to the PMIC parameters structure.
 * @return status Status of the operation (SystemP_SUCCESS if successful,
 * SystemP_FAILURE otherwise).
 */
int32_t PMIC_tps65036xxOpen(PMIC_Config *config, const PMIC_Params *params)
{
    int32_t         status = SystemP_SUCCESS;
    PMIC_Object     *object;
    Pmic_CoreCfg_t pmicConfigData = {0U};
    Pmic_CoreHandle_t *coreHandle = NULL;

    if((NULL == config) || (NULL == params))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        object = (PMIC_Object *) config->object;
        status += SemaphoreP_constructMutex(&gPmicCoreObj);

        /* Allocate memory for PMIC core Handle */
        coreHandle = (Pmic_CoreHandle_t *)malloc(sizeof(Pmic_CoreHandle_t));
        if (NULL != coreHandle)
        {
            memset(coreHandle, 0, sizeof(Pmic_CoreHandle_t));
            object->pmicCoreHandle = coreHandle;

            /* Fill parameters to pmicConfigData */
            pmicConfigData.i2cAddr = params->i2cAddr;
            pmicConfigData.commHandle = I2C_getHandle(params->instance);
            pmicConfigData.ioRead = PMIC_regRead;
            pmicConfigData.ioWrite = PMIC_regWrite;
            pmicConfigData.critSecStart = PMIC_criticalSectionStartFn;
            pmicConfigData.critSecStop = PMIC_criticalSectionStopFn;

            status += Pmic_init(&pmicConfigData, coreHandle);
        }
        else 
        {
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        object->isOpen = 1;
        object->handle = (PMIC_Handle)config;
    }
    else
    {
        PMIC_tps65036xxClose(config);
    }

   return status;
}


/**
 * @brief  Configure the PMIC instance
 * This function performs a LDO configurations and set the LDO's to respective voltage
 *
 * @param PMIC_Config  Pointer to the PMIC config.
 * @return  Status of the register read operation.
 *          - #SystemP_SUCCESS if the read operation is successful.
 *          - #SystemP_FAILURE if the read operation fails.
 */
int32_t PMIC_tps65036xxConfigure(PMIC_Config *config)
{

    int32_t status = SystemP_SUCCESS;
    PMIC_Object *object;
    bool wdgEnabled = (bool)false;
    bool crcIntMask = (bool)TRUE;

    if ((NULL == config))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        object = (PMIC_Object *)config->object;
        Pmic_CoreHandle_t *pmicCoreHandle = object->pmicCoreHandle;

        // Unlock PMIC registers
        status = Pmic_unlockRegs(pmicCoreHandle);

        /* Any time a setting is changed so that it is different from what is stored
         * in the PMIC’s OTP memory, the device will have a reset.
         * The workaround for this is to mask the CONFIG_CRC interrupt.
         */
        status = Pmic_irqSetMask(pmicCoreHandle, PMIC_CONFIG_CRC_INT, crcIntMask);

        // Disable WD if its enabled
        status = Pmic_wdgGetEnable(pmicCoreHandle, &wdgEnabled);
        if (wdgEnabled == true)
        {
            // Disable WDG
            status = Pmic_wdgDisable(pmicCoreHandle);
        }
    }

    return status;
    
}

/**
 * @brief Deinitialize PMIC communication interface and resources.
 * This function deinitializes the PMIC communication interface and resources.
 *
 * @param config Pointer to the PMIC configuration structure.
 * @return void No return value.
 */
void PMIC_tps65036xxClose(PMIC_Config *config)
{
    int32_t         status = SystemP_SUCCESS;
    PMIC_Object     *object;

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        object = (PMIC_Object *) config->object;
        object-> isOpen = 0;
        status = Pmic_deinit(object->pmicCoreHandle);
        free(object->pmicCoreHandle);
        SemaphoreP_destruct(&gPmicCoreObj);
    }

    return;
}

/**
 * @brief  Get the Pmic_CoreHandle_t driver handle
 * This function gets the PMIC driver handle for the specified index
 *
 * @param index  [in] Index within `PMIC_Config gPmicConfig[]`
 * @return  Handle to pmic driver
 */
Pmic_CoreHandle_t* PMIC_getCoreHandle(uint32_t index)
{
    Pmic_CoreHandle_t* handle = NULL;
    /* Check index */
    if(index < gPmicConfigNum)
    {
        PMIC_Object *obj;
        obj = (PMIC_Object *)gPmicConfig[index].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->pmicCoreHandle;
        }
    }
    return handle;
}


/**
 * @brief  Reads a register from the PMIC device.
 * This function performs a read operation on a specified register of
 * the PMIC device using the configured SPI communication. It utilizes
 * the I2C driver to execute the read transaction and updates the
 * received data in the provided buffer.
 *
 * @param pmicCorehandle  Pointer to the PMIC core handle.
 * @param regAddr         Register address to read.
 * @param pBuf            Pointer to the buffer to store the read data.
 * @param bufLen          Length of the buffer.
 * @return  Status of the register read operation.
 *          - #SystemP_SUCCESS if the read operation is successful.
 *          - #SystemP_FAILURE if the read operation fails.
 */
int32_t PMIC_regRead(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *rxBuf)
{
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    if ((pmicHandle == NULL) || (rxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (bufLen == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Initialize I2C handle and I2C transaction struct
        i2cHandle = (I2C_Handle)pmicHandle->commHandle;
        I2C_Transaction_init(&i2cTransaction);

        /*** Configure I2C transaction for a read ***/
        i2cTransaction.targetAddress = pmicHandle->i2cAddr;
        i2cTransaction.writeBuf = &regAddr;
        i2cTransaction.writeCount = 1U;
        i2cTransaction.readBuf = rxBuf;
        i2cTransaction.readCount = bufLen;

        // Initiate read
        status = I2C_transfer(i2cHandle, &i2cTransaction);

        // Convert platform-specific success/error code to driver success/error code
        if (status != I2C_STS_SUCCESS)
        {
            status = PMIC_ST_ERR_I2C_COMM_FAIL;
        }
        else
        {
            status = PMIC_ST_SUCCESS;
        }
    }

    return status;
}

/**
 * @brief  Writes data to a register of the PMIC device.
 * This function performs a write operation to a specified register of the
 * PMIC device using the configured SPI communication. It utilizes the
 * I2C driver to execute the write transaction with the provided data.
 *
 * @param pmicCorehandle Pointer to the PMIC core handle.
 * @param regAddr        Register address to write.
 * @param pBuf           Pointer to the buffer containing the data to write.
 * @param bufLen         Length of the buffer.
 * @return  Status of the register write operation.
 *          - #SystemP_SUCCESS if the write operation is successful.
 *          - #SystemP_FAILURE if the write operation fails.
 */
int32_t PMIC_regWrite(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, const uint8_t *txBuf)
{
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t writeBuf[3U] = {0U};

    // Parameter check
    if ((pmicHandle == NULL) || (txBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && ((bufLen == 0U) || (bufLen > 2U)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // writeBuf[0U]: Target device internal register address
        // writeBuf[1U] and onwards: txBuf
        writeBuf[0U] = regAddr;
        memcpy(&(writeBuf[1U]), txBuf, bufLen);

        // Initialize I2C handle and I2C transaction struct
        i2cHandle = (I2C_Handle)pmicHandle->commHandle;
        I2C_Transaction_init(&i2cTransaction);

        /*** Configure I2C transaction for a write ***/
        i2cTransaction.targetAddress = pmicHandle->i2cAddr;
        i2cTransaction.writeBuf = writeBuf;
        i2cTransaction.writeCount = bufLen + 1U;
        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0U;

        // Initiate write
        status = I2C_transfer(i2cHandle, &i2cTransaction);

        // Convert platform-specific success/error code to driver success/error code
        if (status != I2C_STS_SUCCESS)
        {
            status = PMIC_ST_ERR_I2C_COMM_FAIL;
        }
        else
        {
            status = PMIC_ST_SUCCESS;
        }
    }

    return status;
}

/**
 * @brief  Initiates the start of a critical section for PMIC operations.
 * This function attempts to acquire a semaphore, which is typically
 * used to ensure exclusive access to resources during PMIC operations.
 * If the semaphore acquisition fails, an error message is logged.
 *
 * @param   void
 * @return  NULL
 */
void PMIC_criticalSectionStartFn(void)
{
    SemaphoreP_pend(&gPmicCoreObj, SystemP_WAIT_FOREVER);
}

/**
 * @brief  Concludes a critical section for PMIC operations.
 * This function releases the semaphore, signifying
 * the end of a critical section initiated by a
 * corresponding "start" function.
 *
 * @param   void
 * @return  NULL
 */
void PMIC_criticalSectionStopFn(void)
{
    SemaphoreP_post(&gPmicCoreObj);
}
