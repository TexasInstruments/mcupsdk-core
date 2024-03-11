/******************************************************************************
 *Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/**
 * @file  pmic_io.c
 *
 * @brief  This file contains LLD-Communication wrappers with CRC8 support for
 *         I2C/SPI
 */

/*========================================================================== */
/*                            Include Files                                  */
/*========================================================================== */

#include "pmic_core_priv.h"
#include "pmic_io_priv.h"

/*========================================================================== */
/*                          Macros & Typedefs                                */
/*========================================================================== */
#define CMD_RD_EN       (0x10)
#define CMD_WR_EN       (0x00)
#define CMD_SHIFT       (24U)
#define RW_SHIFT        (16U)
#define DAT_SHIFT       (8U)
#define CMD_DEVICE_ID   (0x00)

/*========================================================================== */
/*                         Function Declarations                             */
/*========================================================================== */
uint8_t PMIC_calcCRC8(uint8_t cmd, uint8_t rdwr, uint8_t dat);

/*========================================================================== */
/*                         Function Prototypes                               */
/*========================================================================== */

static int32_t
Pmic_validateCorehandle(const Pmic_CoreHandle_t *pPmicCoreHandle);

static int32_t Pmic_commIoReadData(Pmic_CoreHandle_t *pPmicCoreHandle,
    uint16_t *pRegAddr, uint8_t *pBuffLength,
    uint8_t *pRxBuf, uint8_t *pinstanceType);
/*========================================================================== */
/*                         Function Definitions                              */
/*========================================================================== */

int32_t Pmic_commIntf_sendByte(Pmic_CoreHandle_t *pPmicCoreHandle,
    uint16_t regAddr, uint8_t txData) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t buffLength = 1U;
    uint8_t instanceType = (uint8_t)PMIC_MAIN_INST;
    uint8_t txBuf[PMIC_IO_BUF_SIZE] = {0U};
    uint16_t pmicRegAddr = regAddr;
    uint8_t data = txData;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /*
         *Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
         *explained in PMIC TRM
         */

        buffLength = 0;
        /*Set ADDR to txbuf[0] with ADDR[7:0] */
        txBuf[buffLength] = (uint8_t)(pmicRegAddr & 0xFFU);
        buffLength++;

        /*Set PAGE to txBuf[1] 7:5 bits with PAGE[2:0] */
        txBuf[buffLength] = (uint8_t)((uint8_t)((uint8_t)(pmicRegAddr >> 8U) & 0x7U) << 5U);

        /*Set R/W in txBuf[1] as bit-4, for Write Request */
        txBuf[buffLength] &= (uint8_t) ~PMIC_IO_REQ_RW;
        buffLength++;

        /*Set write data to txBuf[2], with WDATA[7:0] */
        txBuf[buffLength] = data;
        buffLength++;

        /*Set CRC data to txBuf[3], Bits 25-32 CRC */
        txBuf[buffLength] =
            PMIC_calcCRC8((uint8_t) txBuf[0], (uint8_t) txBuf[1], (uint8_t) txBuf[2]);
        /*Increment 1 more byte to store CRC8 */
        buffLength++;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = pPmicCoreHandle -> pFnPmicCommIoWrite(
            pPmicCoreHandle, instanceType, pmicRegAddr, txBuf, buffLength);
    }
    return pmicStatus;
}

static int32_t
Pmic_validateCorehandle(const Pmic_CoreHandle_t *pPmicCoreHandle) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle -> pCommHandle) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (NULL == pPmicCoreHandle -> pFnPmicCommIoRead)) {
        pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    }

    return pmicStatus;
}

static int32_t Pmic_commIoReadData(Pmic_CoreHandle_t *pPmicCoreHandle,
    uint16_t *pRegAddr, uint8_t *pBuffLength,
    uint8_t pRxBuf[], uint8_t *pinstanceType) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t buffLength = *pBuffLength;
    uint16_t pmicRegAddr = *pRegAddr;
    uint8_t *tempCfg = pinstanceType;
    uint8_t instanceType = *tempCfg;

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /*
         *Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
         *explained in PMIC TRM
         */
        buffLength = 0U;
        /*Set ADDR to pRxBuf[0], with ADDR[7:0] */
        pRxBuf[buffLength] = (uint8_t)(pmicRegAddr & 0xFFU);

        buffLength++;
        /*Set PAGE to pRxBuf[1] 7:5 bits, with PAGE[2:0] */
        pRxBuf[buffLength] = (uint8_t)((uint8_t)((uint8_t)(pmicRegAddr >> 8U) & 0x7U) << 5U);

        /*Set R/W in pRxBuf[1] as bit-4, for read Request */
        pRxBuf[buffLength] |= (uint8_t)PMIC_IO_REQ_RW;
        buffLength++;
        /*Increment 1 more byte for 8-bit data read from PMIC register */
        buffLength++;

        /*Increment 1 more byte to read CRC8 */
        buffLength++;

        *pBuffLength = buffLength;
        *pRegAddr = pmicRegAddr;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = pPmicCoreHandle -> pFnPmicCommIoRead(
            pPmicCoreHandle, instanceType, pmicRegAddr, pRxBuf, buffLength);
    }

    return pmicStatus;
}

int32_t Pmic_commIntf_recvByte(Pmic_CoreHandle_t *pPmicCoreHandle,
    uint16_t regAddr, uint8_t *pRxBuffer) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t buffLength = 1U;
    uint8_t rxBuf[PMIC_IO_BUF_SIZE] = {
        0
    };
    uint8_t crcData[PMIC_IO_BUF_SIZE] = {
        0
    };
    uint8_t instanceType = (uint8_t)PMIC_MAIN_INST;
    uint8_t crcDataLen = 0U;
    uint16_t pmicRegAddr = regAddr;

    pmicStatus = Pmic_commIoReadData(pPmicCoreHandle, & pmicRegAddr, & buffLength,
        rxBuf, & instanceType);
    if (PMIC_ST_SUCCESS != pmicStatus) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if (PMIC_INTF_SPI == pPmicCoreHandle -> commMode) {
            /*Copy SPI frame data to crcData */
            for (crcDataLen = 0U; crcDataLen < (PMIC_IO_BUF_SIZE - 1U); crcDataLen++) {
                crcData[crcDataLen] = rxBuf[crcDataLen];
            }
        } else {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }
        buffLength--;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /*Copy data which shall be in rxBuf[2]/rxBuf[0] to pRxBuffer */
        *pRxBuffer = rxBuf[2];
    }

    return pmicStatus;
}
