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
 *  \file v0/lld/internal/mmcsd_parse.h
 *
 *  \brief MMCSD Driver private header file.
 */

#ifndef MMCSD_PARSE_H
#define MMCSD_PARSE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <drivers/mmcsd/v0/lld/mmcsd_lld.h>
#include <stdint.h>

/* Command types */
#define MMCSD_CMD_TYPE_NORMAL      (0U)
#define MMCSD_CMD_TYPE_SUSPEND     (1U)
#define MMCSD_CMD_TYPE_RESUME      (2U)
#define MMCSD_CMD_TYPE_ABORT       (3U)

/* Response types */
#define MMCSD_CMD_RESP_TYPE_NONE   (0U)
#define MMCSD_CMD_RESP_TYPE_L136   (1U)
#define MMCSD_CMD_RESP_TYPE_L48    (2U)
#define MMCSD_CMD_RESP_TYPE_L48_B  (3U)

/* Device data parsers */
int32_t MMCSD_parseCIDEmmc(MMCSD_EmmcDeviceData *data, uint32_t resp[4]);
int32_t MMCSD_parseCSDEmmc(MMCSD_EmmcDeviceData *data, uint32_t resp[4]);
int32_t MMCSD_parseECSDEmmc(MMCSD_EmmcDeviceData *data, uint8_t ecsdData[512]);

int32_t MMCSD_parseCIDSd(MMCSD_SdDeviceData *data, uint32_t resp[4]);
int32_t MMCSD_parseCSDSd(MMCSD_SdDeviceData *data, uint32_t resp[4]);
int32_t MMCSD_parseSCRSd(MMCSD_SdDeviceData *data, uint8_t *scr);

uint32_t MMCSD_GET_BITFIELD(uint32_t val, uint32_t start, uint32_t end);
uint32_t MMCSD_make_CMD(uint32_t idx, uint32_t  type,
                        uint32_t  dp, uint32_t  rlen);
/* Get MMC protocol Commands */
uint32_t MMCSD_MMC_CMD(uint32_t x);
/* Get SD protocol Commands */
uint32_t MMCSD_SD_CMD(uint32_t x);
/* Get SD ACMD Commands */
uint32_t MMCSD_SD_ACMD(uint32_t x);
/* Get SDIO Commands */
uint32_t MMCSD_SDIO_CMD(uint32_t x);

#ifdef __cplusplus
}
#endif

#endif /* MMCSD_PARSE_H */