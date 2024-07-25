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
#include <board/pmic/pmic_tps653860xx.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/mcspi.h>
#include <pmic_core.h>
#include <pmic_power.h>
#include <string.h>
#include <stdlib.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PMIC_MCSPI_MSGSIZE 1U

/* Sub address command as per tps653860xx datasheet */
#define CRC_POLY			(0x107U)
#define CRC_LEN 			(9U)
#define CMD_SHIFT 			(24U)
#define RW_SHIFT 			(16U)
#define DAT_SHIFT 			(8U)
#define CRC_SHIFT 			(0U)
#define CMD_RD_EN 			(0x10U)
#define CMD_WR_EN 			(0x00U)

SemaphoreP_Object gPmicCoreObj;

uint32_t gPmicMcspiTxBuffer[PMIC_MCSPI_MSGSIZE];
uint32_t gPmicMcspiRxBuffer[PMIC_MCSPI_MSGSIZE];


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t PMIC_regRead(Pmic_CoreHandle_t  *pmicCorehandle,
                          uint8_t             instType,
                          uint16_t            regAddr,
                          uint8_t            *pBuf,
                          uint8_t             bufLen);
int32_t PMIC_regWrite(Pmic_CoreHandle_t  *pmicCorehandle,
                           uint8_t             instType,
                           uint16_t            regAddr,
                           uint8_t            *pBuf,
                           uint8_t             bufLen);

void PMIC_criticalSectionStartFn(void);
void PMIC_criticalSectionStopFn(void);
PMIC_Handle PMIC_getHandle(uint32_t index);

/* Private Functions */
uint8_t PMIC_calcCRC8(uint8_t cmd, uint8_t rdwr, uint8_t dat);
int32_t PMIC_mcspiReadRegister(MCSPI_Handle handle, MCSPI_Transaction *spiTransaction, uint8_t cmd, uint8_t* data);
int32_t PMIC_mcspiWriteRegister(MCSPI_Handle handle, MCSPI_Transaction *spiTransaction, uint8_t cmd, uint8_t data);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

PMIC_Fxns gPmicFxns_TPS653860xx =
{
    .openFxn        = PMIC_tps653860xxOpen,
    .configureFxn   = PMIC_tps653860xxConfigure,
    .closeFxn       = PMIC_tps653860xxClose,
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
int32_t PMIC_tps653860xxOpen(PMIC_Config *config, const PMIC_Params *params)
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
        }

        /* Fill parameters to pmicConfigData */
        pmicConfigData.pmicDeviceType = params->deviceType;
        pmicConfigData.commMode = params->commMode;
        pmicConfigData.instType = params->instType;
        pmicConfigData.pCommHandle = MCSPI_getHandle(params->instance);
        pmicConfigData.pFnPmicCommIoRead = PMIC_regRead;
        pmicConfigData.pFnPmicCommIoWrite = PMIC_regWrite;
        pmicConfigData.pFnPmicCritSecStart = PMIC_criticalSectionStartFn;
        pmicConfigData.pFnPmicCritSecStop = PMIC_criticalSectionStopFn;

        pmicConfigData.validParams |= (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT | PMIC_CFG_COMM_MODE_VALID_SHIFT |
                                        PMIC_CFG_COMM_IO_RD_VALID_SHIFT | PMIC_CFG_COMM_IO_WR_VALID_SHIFT |
                                        PMIC_CFG_CRITSEC_START_VALID_SHIFT | PMIC_CFG_CRITSEC_STOP_VALID_SHIFT |
                                        PMIC_CFG_COMM_HANDLE_VALID_SHIFT);

        status += Pmic_init(&pmicConfigData, coreHandle);

    }

    if(SystemP_SUCCESS == status)
    {
        object->isOpen = 1;
        object->handle = (PMIC_Handle)config;
    }
    else
    {
        PMIC_tps653860xxClose(config);
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
int32_t PMIC_tps653860xxConfigure(PMIC_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    PMIC_Object *object;
    Pmic_CoreHandle_t *pmicCoreHandle = NULL;
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if ((NULL == config))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        object = (PMIC_Object *)config->object;
        pmicCoreHandle = object->pmicCoreHandle;

        Pmic_setRegLockState(pmicCoreHandle, PMIC_LOCK_DISABLE);

        uint8_t ldoNumber = 0U;
        Pmic_ldoCfgReg_t ldoCfg_exp;

        Pmic_ldoCtrlReg_t ldoCtrlConfig_exp =
        {
            PMIC_LDO_ENABLED_LDO_MODE, PMIC_LDO_ENABLED_LDO_MODE,
            PMIC_LDO_ENABLED_LDO_MODE, PMIC_LDO_ENABLED_LDO_MODE
        };

        ldoNumber = PMIC_LDO2;
        ldoCfg_exp.ldoRtCfg = PMIC_LDO_PLDO_LONG_RAMP_TIME;
        ldoCfg_exp.ldoIlimLvlCfg = PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT0;
        ldoCfg_exp.ldoLvlCfg = PMIC_LDO_PLDO_LVL_CFG_VOLT_3_3V;
        pmicStatus += Pmic_powerSetLdoConfigRegister(pmicCoreHandle, ldoNumber, &ldoCfg_exp);

        ldoNumber = PMIC_LDO3;
        ldoCfg_exp.ldoRtCfg = PMIC_LDO_PLDO_LONG_RAMP_TIME;
        ldoCfg_exp.ldoIlimLvlCfg = PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT0;
        ldoCfg_exp.ldoLvlCfg = PMIC_LDO_PLDO_LVL_CFG_VOLT_5V;
        pmicStatus += Pmic_powerSetLdoConfigRegister(pmicCoreHandle, ldoNumber, &ldoCfg_exp);

        ldoNumber = PMIC_LDO4;
        ldoCfg_exp.ldoRtCfg = PMIC_LDO_PLDO_LONG_RAMP_TIME;
        ldoCfg_exp.ldoIlimLvlCfg = PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT0;
        ldoCfg_exp.ldoLvlCfg = PMIC_LDO_PLDO_LVL_CFG_VOLT_1_8V;
        pmicStatus += Pmic_powerSetLdoConfigRegister(pmicCoreHandle, ldoNumber, &ldoCfg_exp);

        pmicStatus += Pmic_setLdoCtrl(pmicCoreHandle, &ldoCtrlConfig_exp);

        if (PMIC_ST_SUCCESS != pmicStatus)
        {
            status += SystemP_FAILURE;
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
void PMIC_tps653860xxClose(PMIC_Config *config)
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
 * @brief  Get the PMIC driver handle
 * This function gets the PMIC driver handle for the specified index
 *
 * @param index  [in] Index within `PMIC_Config gPmicConfig[]`
 * @return  Handle to pmic driver
 */
PMIC_Handle PMIC_getHandle(uint32_t index)
{
    PMIC_Handle         handle = NULL;
    /* Check index */
    if(index < gPmicConfigNum)
    {
        PMIC_Object *obj;
        obj = (PMIC_Object *)gPmicConfig[index].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}


/**
 * @brief  Reads a register from the PMIC device.
 * This function performs a read operation on a specified register of
 * the PMIC device using the configured SPI communication. It utilizes
 * the MCSPI driver to execute the read transaction and updates the
 * received data in the provided buffer.
 *
 * @param pmicCorehandle  Pointer to the PMIC core handle.
 * @param instType        PMIC instance type.
 * @param regAddr         Register address to read.
 * @param pBuf            Pointer to the buffer to store the read data.
 * @param bufLen          Length of the buffer.
 * @return  Status of the register read operation.
 *          - #SystemP_SUCCESS if the read operation is successful.
 *          - #SystemP_FAILURE if the read operation fails.
 */
int32_t PMIC_regRead(Pmic_CoreHandle_t  *pmicCorehandle,
                          uint8_t             instType,
                          uint16_t            regAddr,
                          uint8_t            *pBuf,
                          uint8_t             bufLen)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t pBuf_cmd      = *(pBuf);
    uint8_t pBuf_data     = 0;

    if (NULL == pmicCorehandle)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        /* Setup SPI transfers*/
        MCSPI_Transaction   spiTransaction;
        MCSPI_Transaction_init(&spiTransaction);
        spiTransaction.channel  = MCSPI_CHANNEL_0;
        /* number of bits/frame - this defines the buffer data type used */
        spiTransaction.dataSize = (uint32_t)32;
        /* CS will de-assert automatically after the frame completes */
        spiTransaction.csDisable = TRUE;
        /* number of frames in the transfer */
        spiTransaction.count    = (uint32_t)1;
        spiTransaction.txBuf    = (void *)gPmicMcspiTxBuffer;
        spiTransaction.rxBuf    = (void *)gPmicMcspiRxBuffer;
        spiTransaction.args     = NULL;

        status = PMIC_mcspiReadRegister((MCSPI_Handle)pmicCorehandle->pCommHandle, &spiTransaction,
                                        pBuf_cmd, &pBuf_data);
    }

    if(status != SystemP_SUCCESS)
    {
        status = SystemP_FAILURE;
        return status;
    }
    *(pBuf + 2) = pBuf_data;
    return status;
}

/**
 * @brief  Writes data to a register of the PMIC device.
 * This function performs a write operation to a specified register of the
 * PMIC device using the configured SPI communication. It utilizes the
 * MCSPI driver to execute the write transaction with the provided data.
 *
 * @param pmicCorehandle Pointer to the PMIC core handle.
 * @param instType       PMIC instance type.
 * @param regAddr        Register address to write.
 * @param pBuf           Pointer to the buffer containing the data to write.
 * @param bufLen         Length of the buffer.
 * @return  Status of the register write operation.
 *          - #SystemP_SUCCESS if the write operation is successful.
 *          - #SystemP_FAILURE if the write operation fails.
 */
int32_t PMIC_regWrite(Pmic_CoreHandle_t  *pmicCorehandle,
                           uint8_t             instType,
                           uint16_t            regAddr,
                           uint8_t            *pBuf,
                           uint8_t             bufLen)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t pBuf_cmd     = *(pBuf);
    uint8_t pBuf_data     = *(pBuf + 2);

    if (NULL == pmicCorehandle)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        MCSPI_Transaction   spiTransaction;
        MCSPI_Transaction_init(&spiTransaction);
        spiTransaction.channel  = MCSPI_CHANNEL_0;
        spiTransaction.dataSize = (uint32_t)32;
        spiTransaction.csDisable = TRUE;
        spiTransaction.count    = (uint32_t)1;
        spiTransaction.txBuf    = (void *)gPmicMcspiTxBuffer;
        spiTransaction.rxBuf    = (void *)gPmicMcspiRxBuffer;
        spiTransaction.args     = NULL;

        status = PMIC_mcspiWriteRegister((MCSPI_Handle)pmicCorehandle->pCommHandle, &spiTransaction,
                                         pBuf_cmd, pBuf_data);
    }

    if(status != SystemP_SUCCESS)
    {
        status = SystemP_FAILURE;
        return status;
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

/**
 * @brief Calculate CRC-8 checksum for PMIC SPI communication.
 * This function calculates the CRC-8 checksum based on the command, read/write
 * flag, and data provided for communication between the MCU and the
 * TPS653850A-Q1 device.
 *
 * @param cmd Command byte for communication.
 * @param rdwr Read/Write flag for communication (CMD_RD_EN or CMD_WR_EN).
 * @param dat Data byte for communication.
 * @return crc Calculated CRC-8 checksum value.
 */
uint8_t PMIC_calcCRC8(uint8_t cmd, uint8_t rdwr, uint8_t dat) {
    int8_t i = 0;
    uint32_t crc;
    uint32_t tmp;

    tmp = ((uint32_t) cmd << 16) | ((uint32_t) rdwr << 8) | (uint32_t) dat;
    crc = (uint32_t) 0xFF;

    /* Standard CRC-8 polynomial ,X8 + X2 + X1 + 1.,is used to calculate the
     * checksum value based on the command and data which the MCU transmits
     * to the TPS653850A-Q1 device.
     */

    for (i = 0; i < 24; i++) {
        uint64_t D;
        D = (uint64_t)((uint64_t) tmp & (uint64_t) 0x800000) / (uint64_t) 8388608;
        tmp = (tmp & (uint32_t) 0x7FFFFF) * (uint32_t) 2;
        D = D ^ ((uint64_t)((uint64_t) crc & (uint64_t) 0x80) / (uint64_t) 128);
        crc = (crc & 0x7FU) * 2U;
        D = D * (uint64_t) 7;
        crc = crc ^ (uint32_t) D;
    }

    /* Return the PMIC SPI MCRC value */
    return (uint8_t) crc;
}

/**
 * @brief Write data to a register via the MCSPI interface.
 * This function writes data to a register via the MCSPI interface, including
 * the calculated CRC-8 checksum.
 *
 * @param handle MCSPI handle for communication.
 * @param spiTransaction Pointer to the MCSPI transaction structure.
 * @param cmd Command byte for communication.
 * @param data Data byte to be written.
 * @return status Status of the operation (SystemP_SUCCESS if successful,
 * SystemP_FAILURE otherwise).
 */
int32_t PMIC_mcspiWriteRegister(MCSPI_Handle handle, MCSPI_Transaction *spiTransaction, uint8_t cmd, uint8_t data)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t crc = 0;

    crc = PMIC_calcCRC8(cmd, CMD_WR_EN, (char)data);
    gPmicMcspiTxBuffer[0] = (cmd << CMD_SHIFT) | (CMD_WR_EN << RW_SHIFT) | (data << DAT_SHIFT) | crc;
    gPmicMcspiRxBuffer[0] = 0;

    status = MCSPI_transfer(handle, spiTransaction);

    if((status != SystemP_SUCCESS) || (MCSPI_TRANSFER_COMPLETED != spiTransaction->status))
    {
        /* MCSPI transfer failed!! */
        DebugP_assert(FALSE);
        status = SystemP_FAILURE;
    }

    return status;
}

/**
 * @brief Read data from a register via the MCSPI interface.
 * This function reads data from a register via the MCSPI interface, including
 * the calculated CRC-8 checksum.
 *
 * @param handle MCSPI handle for communication.
 * @param spiTransaction Pointer to the MCSPI transaction structure.
 * @param cmd Command byte for communication.
 * @param data Pointer to store the read data.
 * @return status Status of the operation (SystemP_SUCCESS if successful,
 * SystemP_FAILURE otherwise).
 */
int32_t PMIC_mcspiReadRegister(MCSPI_Handle handle, MCSPI_Transaction *spiTransaction, uint8_t cmd, uint8_t* data)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t crc = 0;

    crc = PMIC_calcCRC8(cmd, CMD_RD_EN, 0);
    gPmicMcspiTxBuffer[0] = (cmd << CMD_SHIFT) | (CMD_RD_EN << RW_SHIFT) | (0 << DAT_SHIFT) | crc;
    gPmicMcspiRxBuffer[0] = 0;

    status = MCSPI_transfer(handle, spiTransaction);

    if((status != SystemP_SUCCESS) || (MCSPI_TRANSFER_COMPLETED != spiTransaction->status))
    {
        /* MCSPI transfer failed!! */
        DebugP_assert(FALSE);
        status = SystemP_FAILURE;
    }
    else
    {
        *data = (gPmicMcspiRxBuffer[0] >> 8) & 0xFF;
    }

    return status;
}
