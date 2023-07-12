/*
 *  Copyright (C) 2022-2023 Texas Instruments Incorporated
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
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON AN2
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hsmclient.h>
#include "hsmclient_utils.h"

/* ========================================================================== */
/*                             Constant Declaration                           */
/* ========================================================================== */

static const char soc_type_arr[NUM_SOC_TYPE][8] = {
	"AM263x",
    "AM263p",
    "AM273x",
    "AWR294x"
};

/* ========================================================================== */
/*                             Static Function Declaration                    */
/* ========================================================================== */

/**
 * \brief Parse SOC Type
 *
 * \param soc_type SOC Type from version struct.
 * \param parsedVer Pointer to parsed string.
 *
 */
static void HsmClient_getSocType(uint8_t soc_type, char* parsedVer);

/**
 * \brief Parse Device Type
 *
 * \param device_type Device Type from version struct.
 * \param parsedVer Pointer to parsed string.
 *
 */
static void HsmClient_getDeviceType(uint8_t device_type, char* parsedVer);

/**
 * \brief Parse Binary Type
 *
 * \param hsm_type Binary Type from version struct.
 * \param parsedVer Pointer to parsed string.
 *
 */
static void HsmClient_getHSMType(uint8_t hsm_type, char* parsedVer);

/**
 * \brief Parse Binary Type
 *
 * \param bin_type Binary Type from version struct.
 * \param parsedVer Pointer to parsed string.
 *
 */
static void HsmClient_getBinType(uint8_t bin_type, char* parsedVer);

/**
 * \brief Integer to alphanumberic conversion.
 *
 * \param value Number to convert to string.
 * \param str Pointer to buffer large enough to hold number.
 * \param base Base to convert number to.
 *
 * \returns Pointer to str.
 */
static uint8_t *lib_itoa(uint32_t value, uint8_t *str, uint32_t base);

/* ========================================================================== */
/*                             Static Function Definition                     */
/* ========================================================================== */

static uint8_t *lib_itoa(uint32_t value, uint8_t *str, uint32_t base)
{
        uint32_t idx = 0;
        uint32_t val;
        uint32_t i;

        if (value == 0U) {
                str[0] = (uint8_t) '0';
                idx++;
        }

        while (value > 0U) {
                val = value % base;
                if (val < 10U) {
                        str[idx] = (uint8_t) (val + '0');
                } else {
                        str[idx] = (uint8_t) ((val - 10U) + 'A');
                }

                idx++;
                value /= base;
        }

        str[idx] = (uint8_t) '\0';

        if (idx > 1U) {
                /* Get length of string - NULL terminator*/
                idx--;

                /* Reverse the string as we converted from low digit to high */
                for (i = 0U; i <= idx / 2U; i++) {
                        val = str[idx - i];
                        str[idx - i] = str[i];
                        str[i] = (uint8_t) val;
                }
        }

        return str;

}

static void HsmClient_getSocType(uint8_t soc_type, char* parsedVer)
{
    strcpy(parsedVer, "\r\n[Soc Type]          = ");
    strcat(parsedVer, soc_type_arr[soc_type-1]);
}

static void HsmClient_getDeviceType(uint8_t device_type, char* parsedVer)
{
    strcat(parsedVer, "\r\n[Device Type]       = ");
    switch (device_type)
    {
        case DEVICE_TYPE_HS_FS:
            strcat(parsedVer, "HS-FS");
            break;
        case DEVICE_TYPE_HS_SE:
            strcat(parsedVer, "HS-SE");
            break;
        default:
            break;
    }
}

static void HsmClient_getHSMType(uint8_t hsm_type, char* parsedVer)
{
    strcat(parsedVer, "\r\n[HSM Type]          = ");
    switch (hsm_type)
    {
        case HSM_V1:
            strcat(parsedVer, "HSM_V1");
            break;
        default:
            break;
    }
}

static void HsmClient_getBinType(uint8_t bin_type, char* parsedVer)
{
    strcat(parsedVer, "\r\n[Bin Type]          = ");
    switch (bin_type)
    {
        case BIN_TYPE_STANDARD:
            strcat(parsedVer, "STANDARD");
            break;
        case BIN_TYPE_CUSTOM:
            strcat(parsedVer, "CUSTOM");
            break;
        case BIN_TYPE_OTPKW:
            strcat(parsedVer, "OTPKW");
            break;
        default:
            break;
    }
}

/* ========================================================================== */
/*                            Function Definition                             */
/* ========================================================================== */

int32_t HsmClient_parseVersion(HsmVer_t *tifsMcuVer, char* parsedVer)
{
    int32_t status = SystemP_FAILURE;

	HsmClient_getSocType(tifsMcuVer->VerStruct.SocType, parsedVer);
	HsmClient_getDeviceType(tifsMcuVer->VerStruct.DevType, parsedVer);
	HsmClient_getHSMType(tifsMcuVer->VerStruct.HsmType, parsedVer);
	HsmClient_getBinType(tifsMcuVer->VerStruct.BinType, parsedVer);
    if(tifsMcuVer->VerStruct.BinType == BIN_TYPE_OTPKW)
    {
        strcat(parsedVer, "\r\n[OTP-KW Version]    = ");
    }
    else
    {
        strcat(parsedVer, "\r\n[TIFS-MCU Version]  = ");
    }
	lib_itoa(tifsMcuVer->VerStruct.MajorVer,
	        (uint8_t *)&parsedVer[strlen(parsedVer)], 10);
	parsedVer[strlen(parsedVer)] = '.';
	lib_itoa(tifsMcuVer->VerStruct.MinorVer,
	        (uint8_t *)&parsedVer[strlen(parsedVer)], 10);
	parsedVer[strlen(parsedVer)] = '.';
	lib_itoa(tifsMcuVer->VerStruct.PatchVer,
	        (uint8_t *)&parsedVer[strlen(parsedVer)], 10);
	parsedVer[strlen(parsedVer)] = '\0';
	status = SystemP_SUCCESS;

    return status;
}
