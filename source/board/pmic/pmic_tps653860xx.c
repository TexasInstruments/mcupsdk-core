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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <board/pmic.h>
#include <board/pmic/pmic_tps653860xx.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/mcspi.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Sub address command as per tps653860xx datasheet */
#define CRC_POLY            (0x107U)
#define CRC_LEN             (9U)
#define CMD_SHIFT           (24U)
#define RW_SHIFT            (16U)
#define DAT_SHIFT           (8U)
#define CRC_SHIFT           (0U)

#define CMD_DEVICE_ID       (0x00)
#define CMD_REV_ID          (0x01)
#define CMD_RD_EN           (0x10)
#define CMD_WR_EN           (0x00)

#define CMD_REG_UNLOCK      (0x03)
#define CMD_REG_STATUS      (0x09)

#define CMD_LDO2_CFG        (0x1F)
#define CMD_LDO3_CFG        (0x20)
#define CMD_LDO4_CFG        (0x21)
#define CMD_LDO_CTRL        (0x26)

#define DAT_REG_UNLOCK_1    (0x98)
#define DAT_REG_UNLOCK_2    (0xB8)

#define DAT_LDO2_CFG        (0x93)
#define DAT_LDO3_CFG        (0x94)
#define DAT_LDO4_CFG        (0x90)
#define DAT_LDO_CTRL        (0x55)

#define PMIC_MCSPI_MSGSIZE              (1U)

uint32_t gPmicMcspiTxBuffer[PMIC_MCSPI_MSGSIZE];
uint32_t gPmicMcspiRxBuffer[PMIC_MCSPI_MSGSIZE];

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Private Functions */
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

uint8_t PMIC_calcCRC8(char cmd, char rdwr, char dat)
{
    int i = 0;
    uint8_t crc;
    uint32_t tmp;

    tmp = ((cmd << 16) | (rdwr << 8) | dat);
    crc = 0xFF;

    /* Standard CRC-8 polynomial ,X8 + X2 + X1 + 1.,is used to calculate the
     * checksum value based on the command and data which the MCU transmits
     * to the TPS653850A-Q1 device.
     */

    for (i = 0; i < 24; i++)
    {
        long D;
        D = (tmp & 0x800000) / 8388608;
        tmp = (tmp & 0x7FFFFF) * 2;
        D = (D ^ (((long)crc & 0x80) / 128));
        crc = (crc & 0x7Fu) * 2u;
        D = D * 7;
        crc = (crc ^ (unsigned char)D);
    }

    /* Return the PMIC SPI MCRC value */
    return crc;

}

int32_t PMIC_tps653860xxOpen(PMIC_Config *config, const PMIC_Params *params)
{
    int32_t         status = SystemP_SUCCESS;
    PMIC_Object     *object;

    if((NULL == config) || (NULL == params))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        object = (PMIC_Object *) config->object;
        object->mcspiInstance  = params->mcspiInstance;
        object->mcspiHandle      = MCSPI_getHandle(object->mcspiInstance);
        if(NULL == object->mcspiHandle)
        {
            status = SystemP_FAILURE;
        }
    }

   return status;
}

int32_t PMIC_tps653860xxConfigure(PMIC_Config *config)
{
    int32_t         status = SystemP_SUCCESS;
    PMIC_Object     *object;
    MCSPI_Transaction   spiTransaction;
    uint8_t data = 0;

    if((NULL == config))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        object = (PMIC_Object *) config->object;

        /* Setup SPI transfers*/
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

        status = PMIC_mcspiReadRegister((MCSPI_Handle)object->mcspiHandle, &spiTransaction, CMD_REV_ID, &data);

        if(status == SystemP_SUCCESS)
        {
            status += PMIC_mcspiWriteRegister((MCSPI_Handle)object->mcspiHandle, &spiTransaction, CMD_REG_UNLOCK, DAT_REG_UNLOCK_1);
            status += PMIC_mcspiWriteRegister((MCSPI_Handle)object->mcspiHandle, &spiTransaction, CMD_REG_UNLOCK, DAT_REG_UNLOCK_2);
            status += PMIC_mcspiWriteRegister((MCSPI_Handle)object->mcspiHandle, &spiTransaction, CMD_LDO2_CFG, DAT_LDO2_CFG);
            status += PMIC_mcspiWriteRegister((MCSPI_Handle)object->mcspiHandle, &spiTransaction, CMD_LDO3_CFG, DAT_LDO3_CFG);
            status += PMIC_mcspiWriteRegister((MCSPI_Handle)object->mcspiHandle, &spiTransaction, CMD_LDO4_CFG, DAT_LDO4_CFG);
            status += PMIC_mcspiWriteRegister((MCSPI_Handle)object->mcspiHandle, &spiTransaction, CMD_LDO_CTRL, DAT_LDO_CTRL);
        }
    }

    return status;
}

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

        object->mcspiHandle = NULL;
    }

    return;
}


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
