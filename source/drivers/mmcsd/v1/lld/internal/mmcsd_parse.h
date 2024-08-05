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
 *  \file v1/lld/internal/mmcsd_parse.h
 *
 *  \brief MMCSD Driver private header file.
 */

#ifndef MMCSD_PARSE_H
#define MMCSD_PARSE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <drivers/mmcsd/v1/lld/mmcsd_lld.h>
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

/** \brief Bit mask. */
#define BIT(x) (((uint32_t)1U) << (x))

/**
 * Command/Response flags for notifying some information to controller.
 */

/** \brief To indicate no response. */
#define MMCSD_CMDRSP_NONE           (BIT(0U))
/** \brief Response to indicate stop condition. */
#define MMCSD_CMDRSP_STOP           (BIT(1U))
/** \brief Response to indicate stop condition. */
#define MMCSD_CMDRSP_FS             (BIT(2U))
/** \brief Response to indicate abort condition. */
#define MMCSD_CMDRSP_ABORT          (BIT(3U))
/** \brief Response to indicate busy state. */
#define MMCSD_CMDRSP_BUSY           (BIT(4U))
/** \brief Command to configure for 48bit R1 response */
#define MMCSD_CMDRSP_48BITS         (BIT(9U))
/** \brief Command to configure for 136 bits data width. */
#define MMCSD_CMDRSP_136BITS        (BIT(5U))
/** \brief Command to configure for data or response. */
#define MMCSD_CMDRSP_DATA           (BIT(6U))
/** \brief Command to configure for data read. */
#define MMCSD_CMDRSP_READ           (BIT(7U))
/** \brief Command to configure for data write. */
#define MMCSD_CMDRSP_WRITE          (BIT(8U))
/** \brief Command to configure for Read or Write interrupt */
#define MMCSD_CMDREQ_WR_RD          (BIT(31U))

/**
 * SD OCR register definitions.
 */

/** \brief High capacity card type. */
#define MMCSD_OCR_HIGH_CAPACITY     (BIT(30U))
#define MMCSD_OCR_S18R              (BIT(24U))

/**
 * Voltage configurations.
 */

/** \brief Configure for 2.7V to 2.8V VDD level. */
#define MMCSD_OCR_VDD_2P7_2P8       (BIT(15U))
/** \brief Configure for 2.8V to 2.9V VDD level. */
#define MMCSD_OCR_VDD_2P8_2P9       (BIT(16U))
/** \brief Configure for 2.9V to 3.0V VDD level. */
#define MMCSD_OCR_VDD_2P9_3P0       (BIT(17U))
/** \brief Configure for 3.0V to 3.1V VDD level. */
#define MMCSD_OCR_VDD_3P0_3P1       (BIT(18U))
/** \brief Configure for 3.1V to 3.2V VDD level. */
#define MMCSD_OCR_VDD_3P1_3P2       (BIT(19U))
/** \brief Configure for 3.2V to 3.3V VDD level. */
#define MMCSD_OCR_VDD_3P2_3P3       (BIT(20U))
/** \brief Configure for 3.3V to 3.4V VDD level. */
#define MMCSD_OCR_VDD_3P3_3P4       (BIT(21U))
/** \brief Configure for 3.4V to 3.5V VDD level. */
#define MMCSD_OCR_VDD_3P4_3P5       (BIT(22U))
/** \brief Configure for 3.5V to 3.6V VDD level. */
#define MMCSD_OCR_VDD_3P5_3P6       (BIT(23U))
/** \brief Wild card to configure for VDD level. */
#define MMCSD_OCR_VDD_WILDCARD      (((uint32_t)0x1FFU) << 15U)


/* Device data parsers */
int32_t MMCSD_parseCIDEmmc(MMCSD_EmmcDeviceData *data, uint32_t resp[4]);
int32_t MMCSD_parseCSDEmmc(MMCSD_EmmcDeviceData *data, uint32_t resp[4]);
int32_t MMCSD_parseECSDEmmc(MMCSD_EmmcDeviceData *data, uint8_t ecsdData[512]);

int32_t MMCSD_parseCIDSd(MMCSD_SdDeviceData *data, uint32_t resp[4]);
int32_t MMCSD_parseCSDSd(MMCSD_SdDeviceData *data, uint32_t resp[4]);
int32_t MMCSD_parseSCRSd(MMCSD_SdDeviceData *data, uint8_t *scr);

uint32_t MMCSD_GET_BITFIELD(uint32_t val, uint32_t start, uint32_t end);

/* Get MMC protocol Commands */
uint32_t MMCSD_CMD(uint32_t x);

#ifdef __cplusplus
}
#endif

#endif /* MMCSD_PARSE_H */