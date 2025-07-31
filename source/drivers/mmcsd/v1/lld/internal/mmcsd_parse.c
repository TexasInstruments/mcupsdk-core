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

/**
 *  \file mmcsd_parse.c
 *
 *  \brief MMCSD purse source file.
 *
 */
#include <stdio.h> /* For snprintf */
#include <string.h> /* For memcpy */
#include <drivers/mmcsd/v1/lld/internal/mmcsd_parse.h>

#define YEAR_1997       (1997U)
#define YEAR_2013       (2013U)

static const char* gMonths[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

uint32_t MMCSD_GET_BITFIELD(uint32_t val, uint32_t start, uint32_t end)
{
    return (((val) >> (start)) & ((1U << ((end)-(start) + 1U)) - 1U));
}

uint32_t MMCSD_CMD(uint32_t x)
{
    return x;
}

int32_t MMCSD_parseCIDEmmc(MMCSD_EmmcDeviceData *data, uint32_t resp[4])
{
    int32_t status = MMCSD_STS_SUCCESS;

    if((data != NULL) && (resp != NULL))
    {
        /* Shift the responses left by 1 byte */
        uint32_t tempResp[4];
        tempResp[3] = (resp[3] << 8) | (resp[2] >> 24);
        tempResp[2] = (resp[2] << 8) | (resp[1] >> 24);
        tempResp[1] = (resp[1] << 8) | (resp[0] >> 24);
        tempResp[0] = (resp[0] << 8);

        /* Manufacturer ID - 8 bits 31:24 of last DWORD*/
        data->manuID = (uint8_t)MMCSD_GET_BITFIELD(tempResp[3], 24U, 31U);

        /* Product name */
        data->productName[6] = '\0';

        /* Product name is bits 56:103 (48 bits, 6 bytes), to extract we can use a byte pointer and do memcpy */
        uint8_t *pTempResp = (uint8_t *)&tempResp[0];

        (void)memcpy(data->productName, pTempResp + 7U, 6U);

        /* Manufacturing date */
        uint16_t monthCode = (uint16_t)MMCSD_GET_BITFIELD(tempResp[0], 12U, 15U);
        uint16_t yearCode  = (uint16_t)MMCSD_GET_BITFIELD(tempResp[0], 8U, 11U);

        if((monthCode < 1U) || (monthCode > 12U))
        {
            monthCode = 1U;
        }
        else
        {
            /* monthCode in limits, [1, 12] */
        }

        if(yearCode <= 12U)
        {
            yearCode += YEAR_2013;
        }
        else
        {
            yearCode += YEAR_1997;
        }

        (void)snprintf(data->manuDate, 8, "%s%04u", gMonths[monthCode-(uint16_t)1U], yearCode);
        data->manuDate[8] = '\0';
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

int32_t MMCSD_parseCSDEmmc(MMCSD_EmmcDeviceData *data, uint32_t resp[4])
{
    int32_t status = MMCSD_STS_SUCCESS;

    if((data != NULL) && (resp != NULL))
    {

        data->maxWriteBlockLen = (((uint32_t)2U)<<(((resp[0] & 0x03C00000U) >> 22)-1U));
        data->transferSpeed = (resp[3] & 0x000000FFU);

        if(((resp[3] & 0x3C000000U) >> 26) != 0x04U)
        {
            status = MMCSD_STS_ERR;
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

int32_t MMCSD_parseECSDEmmc(MMCSD_EmmcDeviceData *data, uint8_t ecsdData[512])
{
    int32_t status = MMCSD_STS_SUCCESS;

    if((data != NULL) && (ecsdData != NULL))
    {
        data->driveStrength = (uint8_t)(ecsdData[185] >> 4);
        data->blockCount = (((uint32_t)(ecsdData[215])) << 24) +
                           (((uint32_t)(ecsdData[214])) << 16) +
                           (((uint32_t)(ecsdData[213])) << 8) +
                           (((uint32_t)(ecsdData[212])));
        data->eStrobeSupport = ecsdData[184];
        /*  7.4.59 DEVICE_TYPE [196] (JESD84-B51)
            This field defines the type of the Device. */
        data->supportedModes = ecsdData[196];

        /* Manufacturing year corrections */
        uint32_t sdRev = (uint32_t)ecsdData[192];
        if(sdRev <= 4U)
        {
            int16_t year = (((int16_t)(data->manuDate[3]-'0') * (int16_t)1000) +
                            ((int16_t)(data->manuDate[4]-'0') * (int16_t)100) +
                            ((int16_t)(data->manuDate[5]-'0') * (int16_t)10) +
                            (int16_t)(data->manuDate[6]-'0'));
            year += 16;
            uint32_t i = 4U;
            while(i-- != 0U)
            {
                data->manuDate[3U + i] = (year % 10) + '0';
                year /= 10;
            }
        }
        else
        {
            /* do nothing */
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

int32_t MMCSD_parseCIDSd(MMCSD_SdDeviceData *data, uint32_t resp[4])
{
    int32_t status = MMCSD_STS_SUCCESS;

    if((data != NULL) && (resp != NULL))
    {
        /* Shift the responses left by 1 byte */
        uint32_t tempResp[4];
        tempResp[3] = (resp[3] << 8) | (resp[2] >> 24);
        tempResp[2] = (resp[2] << 8) | (resp[1] >> 24);
        tempResp[1] = (resp[1] << 8) | (resp[0] >> 24);
        tempResp[0] = (resp[0] << 8);

        /* Manufacturer ID - 8 bits 31:24 */
        data->manuID = (uint8_t)MMCSD_GET_BITFIELD(tempResp[3], 24U, 31U);

        /* Product name */
        data->productName[5] = '\0';
        data->productName[4] = MMCSD_GET_BITFIELD(tempResp[3], 0U, 7U);
        (void)memcpy(data->productName, &tempResp[2], 4U);

        /* Manufacturing date */
        uint32_t monthCode = MMCSD_GET_BITFIELD(tempResp[0], 8U, 11U);
        uint16_t yearCode  = (uint16_t)MMCSD_GET_BITFIELD(tempResp[0], 12U, 19U);

        monthCode = ((monthCode > 11U) || (monthCode < 1U)) ? 1U : monthCode;

        (void)snprintf(data->manuDate, 8, "%s%04u", gMonths[monthCode - 1U], yearCode+2000U);
        data->manuDate[8] = '\0';
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

int32_t MMCSD_parseCSDSd(MMCSD_SdDeviceData *data, uint32_t resp[4])
{
    int32_t status = MMCSD_STS_SUCCESS;

    if((data != NULL) && (resp != NULL))
    {
        /* Shift the responses left by 1 byte */
        uint32_t tempResp[4];
        tempResp[3] = (resp[3] << 8) | (resp[2] >> 24);
        tempResp[2] = (resp[2] << 8) | (resp[1] >> 24);
        tempResp[1] = (resp[1] << 8) | (resp[0] >> 24);
        tempResp[0] = (resp[0] << 8);

        data->maxWriteBlockLen = ((uint16_t)1U << (uint16_t)(MMCSD_GET_BITFIELD(tempResp[0], 22U, 25U))) << 8U;
        data->maxReadBlockLen  = ((uint16_t)1U << (uint16_t)(MMCSD_GET_BITFIELD(tempResp[2], 16U, 19U)));
        data->transferSpeed = (uint8_t)(MMCSD_GET_BITFIELD(tempResp[3], 0U, 7U));

        uint32_t csdVersion = MMCSD_GET_BITFIELD(tempResp[3], 30U, 31U);

        if(csdVersion == 0U)
        {
            /* CSD Ver 1.0 Standard Capacity */
            /* Calculate block count of user data area */
            uint32_t cSize = ((uint32_t)(MMCSD_GET_BITFIELD(tempResp[2], 0U, 9U)) << 2U) | (uint32_t)(MMCSD_GET_BITFIELD(tempResp[1], 30U, 31U));
            uint32_t mult = ((uint32_t)1U << ((uint32_t)2U + (uint32_t)(MMCSD_GET_BITFIELD(tempResp[1], 15U, 17U))));
            data->blockCount = (cSize + 1U) * mult;
        }
        else if(csdVersion == 1U)
        {
            /* CSD Ver 2.0 High Capacity Extended Capacity */
            /* Calculate block count of user data area */
            data->blockCount = (uint32_t)1024U * ((uint32_t)1U + (uint32_t)((MMCSD_GET_BITFIELD(tempResp[2], 0U, 5U) << 16U) | MMCSD_GET_BITFIELD(tempResp[1], 16U, 31U)));
        }
        else if(csdVersion == 2U)
        {
            /* CSD Ver 3.0 Ultra High Capacity */
            /* Calculate block count of user data area */
            data->blockCount = (uint32_t)1024U * ((uint32_t)1U + (uint32_t)((MMCSD_GET_BITFIELD(tempResp[2], 0U, 11U) << 16U) | MMCSD_GET_BITFIELD(tempResp[1], 16U, 31U)));
        }
        else
        {
            data->blockCount = 0U;
            status = MMCSD_STS_ERR;
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

int32_t MMCSD_parseSCRSd(MMCSD_SdDeviceData *data, uint8_t *scr)
{
    int32_t status = MMCSD_STS_SUCCESS;

    if((data != NULL) && (scr != NULL))
    {
        data->specVersion = (uint8_t)MMCSD_GET_BITFIELD(scr[0], 0U, 3U);
        data->supportedDataWidths = ((uint32_t)0xFFFFFFFFU & (MMCSD_GET_BITFIELD((uint32_t)scr[1], 0U, 3U)));
        data->isCmd23 = (uint32_t)(((MMCSD_GET_BITFIELD(scr[3], 0U, 3U) & (0x2U)) != 0U) ? 1U : 0U);
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}