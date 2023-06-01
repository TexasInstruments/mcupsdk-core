/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *  \file mmcsd_priv.c
 *
 *  \brief MMCSD private source file.
 *
 */
#include <stdio.h> /* For snprintf */
#include <kernel/dpl/SystemP.h>
#include "mmcsd_priv.h"
#include <string.h> /* For memcpy */

static char* gMonths[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

int32_t MMCSD_parseCIDEmmc(MMCSD_EmmcDeviceData *data, uint32_t resp[4])
{
    int32_t status = SystemP_SUCCESS;

    if((data != NULL) && (resp != NULL))
    {
        /* Shift the responses left by 1 byte */
        uint32_t tempResp[4];
        tempResp[3] = (resp[3] << 8) | (resp[2] >> 24);
        tempResp[2] = (resp[2] << 8) | (resp[1] >> 24);
        tempResp[1] = (resp[1] << 8) | (resp[0] >> 24);
        tempResp[0] = (resp[0] << 8);

        /* Manufacturer ID - 8 bits 31:24 of last DWORD*/
        data->manuID = MMCSD_GET_BITFIELD(tempResp[3], 24, 31);

        /* Product name */
        data->productName[6] = '\0';

        /* Product name is bits 56:103 (48 bits, 6 bytes), to extract we can use a byte pointer and do memcpy */
        uint8_t *pTempResp = (uint8_t *)&tempResp[0];
        
        memcpy(data->productName, pTempResp + 7, 6U);

        /* Manufacturing date */
        uint16_t monthCode = MMCSD_GET_BITFIELD(tempResp[0], 12, 15);
        uint16_t yearCode  = MMCSD_GET_BITFIELD(tempResp[0], 8, 11);

        if((monthCode < 1U) || (monthCode > 12U))
        {
            monthCode = 1U;
        }
        else
        {
            /* monthCode in limits, [1, 12] */
        }

        if(yearCode <= 12)
        {
            yearCode += 2013;
        }
        else
        {
            yearCode += 1997;
        }

        snprintf(data->manuDate, 8, "%s%04u", gMonths[monthCode-1], yearCode);
        data->manuDate[8] = '\0';
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t MMCSD_parseCSDEmmc(MMCSD_EmmcDeviceData *data, uint32_t resp[4])
{
    int32_t status = SystemP_SUCCESS;

    if((data != NULL) && (resp != NULL))
    {
        /* Shift the responses left by 1 byte */
        uint32_t tempResp[4];
        tempResp[3] = (resp[3] << 8) | (resp[2] >> 24);
        tempResp[2] = (resp[2] << 8) | (resp[1] >> 24);
        tempResp[1] = (resp[1] << 8) | (resp[0] >> 24);
        tempResp[0] = (resp[0] << 8);

        data->maxWriteBlockLen = (1 << (uint32_t)(MMCSD_GET_BITFIELD(tempResp[0], 22, 25)));
        data->maxReadBlockLen  = (1 << (uint32_t)(MMCSD_GET_BITFIELD(tempResp[2], 16, 19)));

        data->transferSpeed = MMCSD_GET_BITFIELD(tempResp[3], 0, 7);
        data->specVersion = MMCSD_GET_BITFIELD(tempResp[3], 26, 29);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t MMCSD_parseECSDEmmc(MMCSD_EmmcDeviceData *data, uint8_t ecsdData[512])
{
    int32_t status = SystemP_SUCCESS;

    if((data != NULL) && (ecsdData != NULL))
    {
        data->driveStrength = (uint8_t)(ecsdData[185] >> 4);
        data->blockCount = (((uint32_t)(ecsdData[215])) << 24) +
                           (((uint32_t)(ecsdData[214])) << 16) +
                           (((uint32_t)(ecsdData[213])) << 8) +
                           (((uint32_t)(ecsdData[212])));
        data->eStrobeSupport = ecsdData[184];
        data->supportedModes = ecsdData[196];

        /* Manufacturing year corrections */
        uint32_t sdRev = (uint32_t)ecsdData[192];
        if(sdRev <= 4)
        {
            uint16_t year = (data->manuDate[3]-'0')*1000 +
                            (data->manuDate[4]-'0')*100 +
                            (data->manuDate[5]-'0')*10 +
                            (data->manuDate[6]-'0');
            year += 16;
            uint32_t i = 4;
            while(i--)
            {
                data->manuDate[3+i] = (year % 10)+'0';
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
        status = SystemP_FAILURE;
    }

    return status;
}


int32_t MMCSD_parseCIDSd(MMCSD_SdDeviceData *data, uint32_t resp[4])
{
    int32_t status = SystemP_SUCCESS;

    if((data != NULL) && (resp != NULL))
    {
        /* Shift the responses left by 1 byte */
        uint32_t tempResp[4];
        tempResp[3] = (resp[3] << 8) | (resp[2] >> 24);
        tempResp[2] = (resp[2] << 8) | (resp[1] >> 24);
        tempResp[1] = (resp[1] << 8) | (resp[0] >> 24);
        tempResp[0] = (resp[0] << 8);

        /* Manufacturer ID - 8 bits 31:24 */
        data->manuID = MMCSD_GET_BITFIELD(tempResp[3], 24, 31);

        /* Product name */
        data->productName[5] = '\0';
        data->productName[4] = MMCSD_GET_BITFIELD(tempResp[3], 0, 7);
        memcpy(data->productName, &tempResp[2], 4U);

        /* Manufacturing date */
        uint32_t monthCode = MMCSD_GET_BITFIELD(tempResp[0], 8, 11);
        uint16_t yearCode  = MMCSD_GET_BITFIELD(tempResp[0], 12, 19);

        monthCode = ((monthCode > 11U) || (monthCode < 1U)) ? 1U : monthCode;

        snprintf(data->manuDate, 8, "%s%04u", gMonths[monthCode-1], yearCode+2000U);
        data->manuDate[8] = '\0';
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t MMCSD_parseCSDSd(MMCSD_SdDeviceData *data, uint32_t resp[4])
{
    int32_t status = SystemP_SUCCESS;

    if((data != NULL) && (resp != NULL))
    {
        /* Shift the responses left by 1 byte */
        uint32_t tempResp[4];
        tempResp[3] = (resp[3] << 8) | (resp[2] >> 24);
        tempResp[2] = (resp[2] << 8) | (resp[1] >> 24);
        tempResp[1] = (resp[1] << 8) | (resp[0] >> 24);
        tempResp[0] = (resp[0] << 8);

        data->maxWriteBlockLen = (1 << (uint32_t)(MMCSD_GET_BITFIELD(tempResp[0], 22, 25)));
        data->maxReadBlockLen  = (1 << (uint32_t)(MMCSD_GET_BITFIELD(tempResp[2], 16, 19)));
        data->transferSpeed = MMCSD_GET_BITFIELD(tempResp[3], 0, 7);

        uint32_t csdVersion = MMCSD_GET_BITFIELD(tempResp[3], 30, 31);

        if(csdVersion == 0)
        {
            /* CSD Ver 1.0 Standard Capacity */
            /* Calculate block count of user data area */
            uint32_t cSize = ((uint32_t)(MMCSD_GET_BITFIELD(tempResp[2], 0, 9)) << 2U) | (uint32_t)(MMCSD_GET_BITFIELD(tempResp[1], 30, 31));
            uint32_t mult = (1U << (2U+(uint32_t)(MMCSD_GET_BITFIELD(tempResp[1], 15, 17))));
            data->blockCount = (cSize+1)*mult;
        }
        else if(csdVersion == 1)
        {
            /* CSD Ver 2.0 High Capacity Extended Capacity */
            /* Calculate block count of user data area */
            data->blockCount = 1024*(1U + ((MMCSD_GET_BITFIELD(tempResp[2], 0, 5) << 16U) | MMCSD_GET_BITFIELD(tempResp[1], 16, 31)));
        }
        else if(csdVersion == 2)
        {
            /* CSD Ver 3.0 Ultra High Capacity */
            /* Calculate block count of user data area */
            data->blockCount = 1024*(1U + ((MMCSD_GET_BITFIELD(tempResp[2], 0, 11) << 16U) | MMCSD_GET_BITFIELD(tempResp[1], 16, 31)));
        }
        else
        {
            data->blockCount = 0U;
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t MMCSD_parseSCRSd(MMCSD_SdDeviceData *data, uint8_t *scr)
{
    int32_t status = SystemP_SUCCESS;

    if((data != NULL) && (scr != NULL))
    {
        data->specVersion = MMCSD_GET_BITFIELD(scr[0], 0, 3);
        data->supportedDataWidths = MMCSD_GET_BITFIELD(scr[1], 0, 3);
        data->isCmd23 = (MMCSD_GET_BITFIELD(scr[3], 0, 3) & (0x2)) ? 1U : 0U;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

