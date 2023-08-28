/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
 *  \ingroup BOARD_MODULE
 *  \defgroup FLASH_CONFIG_MODULE Flash configuration structures
 *
 *   The config structures defined here are used by SFDP layer, flash layer etc to configure
 *   dynamically.
 *
 *  @{
 */

/**
 *  \file flash_config.h
 *
 *  \brief Flash configuration structures file.
 */

#include <stdint.h>

#ifndef FLASH_CONFIG_H_
#define FLASH_CONFIG_H_

/**
 * \brief Flash protocols supported
 */

#define FLASH_CFG_MAX_PROTO (10U)

#define FLASH_CFG_PROTO_1S_1S_1S (0x0001)
#define FLASH_CFG_PROTO_1S_1S_2S (0x0002)
#define FLASH_CFG_PROTO_1S_1S_4S (0x0003)
#define FLASH_CFG_PROTO_1S_1S_8S (0x0004)
#define FLASH_CFG_PROTO_4S_4S_4S (0x0005)
#define FLASH_CFG_PROTO_4S_4D_4D (0x0006)
#define FLASH_CFG_PROTO_8S_8S_8S (0x0007)
#define FLASH_CFG_PROTO_8D_8D_8D (0x0008)
#define FLASH_CFG_PROTO_CUSTOM   (0x0009)

/**
 * \brief Configuration structure for flash erase
 */

typedef struct
{
    uint32_t blockSize;
    uint32_t sectorSize;
    uint8_t  cmdBlockErase3B;
    uint8_t  cmdBlockErase4B;
    uint8_t  cmdSectorErase3B;
    uint8_t  cmdSectorErase4B;
    uint8_t  cmdChipErase;
    uint8_t  cmdBlockErase;
    uint8_t  cmdBlockEraseCyc2;
    uint8_t rsvd;

} FlashCfg_EraseConfig;

/**
 * \brief Config structure for reading JEDEC ID
 */
typedef struct
{
    uint8_t cmd;
    uint8_t numBytes;
    uint8_t dummy4;
    uint8_t dummy8;
    uint8_t addrSize;
    uint8_t rsvd[3];
} FlashCfg_ReadIDConfig;

typedef struct
{
    uint8_t  isAddrReg;
    uint8_t  cmdRegRd;
    uint8_t  cmdRegWr;
    uint32_t cfgReg;
    uint16_t shift;
    uint16_t mask;
    uint8_t  cfgRegBitP;

} FlashCfg_RegConfig;

/**
 * \brief Configuration structure for enabling a particular flash data
 *        protocol
 */
typedef struct
{
    uint32_t protocol;
    uint16_t isDtr;
    uint8_t  cmdRd;
    uint8_t  cmdWr;
    uint8_t  modeClksCmd;
    uint8_t  modeClksRd;
    uint16_t dummyClksCmd;
    uint16_t dummyClksRd;
    uint8_t  enableType;
    uint8_t  enableSeq;

    FlashCfg_RegConfig protoCfg;
    FlashCfg_RegConfig dummyCfg;
    FlashCfg_RegConfig strDtrCfg;

} FlashCfg_ProtoEnConfig;

#endif /* FLASH_CONFIG_H_ */