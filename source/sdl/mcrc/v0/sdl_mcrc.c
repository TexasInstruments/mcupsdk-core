/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2024
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
 *
 */

/**
 *  \file     sdl_mcrc.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of MCRC.
 *            This also contains some related macros.
 */

 /**
 * @brief MCRC-64 polynomial: x^64 + x^4 + x^3 + x + 1
 */


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include "sdl_ip_mcrc.h"
#include "sdl_mcrc.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* channel specific register group offset */
static inline uint32_t SDL_MCRC_getRegsOffset(uint32_t channel);
static inline uint32_t SDL_MCRC_getRegsOffset(uint32_t channel)
{
    return (channel * (uint32_t)0x40U);
}

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  Design: PROC_SDL-2079
 */
int32_t SDL_MCRC_init(SDL_MCRC_InstType instance,
                      SDL_MCRC_Channel_t channel,
                      uint32_t watchdogPreload,
                      uint32_t blockPreload)
{
    int32_t status = SDL_PASS;
    uint32_t baseAddr;

    if ((watchdogPreload > SDL_MCRC_WDTOPLD_MAX) ||
        (blockPreload > SDL_MCRC_BCTOPLD_MAX) ||
        (SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                /* Configure watchdog pre-load value */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_WDTOPLD1,
                              SDL_MCRC_WDTOPLD1,
                              watchdogPreload);
                /* Configure clock pre-load value */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_BCTOPLD1,
                              SDL_MCRC_BCTOPLD1,
                              blockPreload);
                break;
            case SDL_MCRC_CHANNEL_2:
                /* Configure watchdog pre-load value */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_WDTOPLD2,
                              SDL_MCRC_WDTOPLD2,
                              watchdogPreload);
                /* Configure clock pre-load value */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_BCTOPLD2,
                              SDL_MCRC_BCTOPLD2,
                              blockPreload);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                /* Configure watchdog pre-load value */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_WDTOPLD3,
                              SDL_MCRC_WDTOPLD3,
                              watchdogPreload);
                /* Configure clock pre-load value */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_BCTOPLD3,
                              SDL_MCRC_BCTOPLD3,
                              blockPreload);
                break;
            case SDL_MCRC_CHANNEL_4:
                /* Configure watchdog pre-load value */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_WDTOPLD4,
                              SDL_MCRC_WDTOPLD4,
                              watchdogPreload);
                /* Configure clock pre-load value */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_BCTOPLD4,
                              SDL_MCRC_BCTOPLD4,
                              blockPreload);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2085
 */
int32_t SDL_MCRC_verifyInit(SDL_MCRC_InstType     instance,
                            SDL_MCRC_Channel_t channel,
                            uint32_t     watchdogPreload,
                            uint32_t     blockPreload)
{
    int32_t  status = SDL_PASS;
    uint32_t readwatchdogPreload;
    uint32_t readblockPreload;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS) ||
        (watchdogPreload > SDL_MCRC_WDTOPLD_MAX)  ||
        (blockPreload > SDL_MCRC_BCTOPLD_MAX))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        /* Read WDT preload value and block complete preload value */
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                readwatchdogPreload = HW_RD_FIELD32(baseAddr + SDL_MCRC_WDTOPLD1,
                                                    SDL_MCRC_WDTOPLD1);
                readblockPreload = HW_RD_FIELD32(baseAddr + SDL_MCRC_BCTOPLD1,
                                                 SDL_MCRC_BCTOPLD1);
                break;
            case SDL_MCRC_CHANNEL_2:
                readwatchdogPreload = HW_RD_FIELD32(baseAddr + SDL_MCRC_WDTOPLD2,
                                                    SDL_MCRC_WDTOPLD2);
                readblockPreload = HW_RD_FIELD32(baseAddr + SDL_MCRC_BCTOPLD2,
                                                 SDL_MCRC_BCTOPLD2);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                readwatchdogPreload = HW_RD_FIELD32(baseAddr + SDL_MCRC_WDTOPLD3,
                                                    SDL_MCRC_WDTOPLD3);
                readblockPreload = HW_RD_FIELD32(baseAddr + SDL_MCRC_BCTOPLD3,
                                                 SDL_MCRC_BCTOPLD3);
                break;
            case SDL_MCRC_CHANNEL_4:
                readwatchdogPreload = HW_RD_FIELD32(baseAddr + SDL_MCRC_WDTOPLD4,
                                                    SDL_MCRC_WDTOPLD4);
                readblockPreload = HW_RD_FIELD32(baseAddr + SDL_MCRC_BCTOPLD4,
                                                 SDL_MCRC_BCTOPLD4);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    if (status == SDL_PASS)
    {
        if ((readwatchdogPreload != watchdogPreload) ||
            (readblockPreload != blockPreload))
        {
            status = SDL_EFAIL;
        }
    }

    return (status);
}
/**
 *  Design: PROC_SDL-2080,PROC_SDL-2081
 */
int32_t SDL_MCRC_config(SDL_MCRC_InstType instance,
                     SDL_MCRC_Channel_t channel,
                     uint32_t patternCount,
                     uint32_t sectorCount,
                     SDL_MCRC_ModeType mode)
{
    int32_t status = SDL_PASS;
    uint32_t baseAddr;

    if (((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS))   ||
        (patternCount > SDL_MCRC_PATTERN_COUNT_MAX)  ||
        (sectorCount > SDL_MCRC_SECTOR_COUNT_MAX)    ||
        (mode > SDL_MCRC_CTRL2_CH1_MODE_FULLCPU))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                /* Configure MCRC pattern count */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_PCOUNT_REG1,
                              SDL_MCRC_PCOUNT_REG1_PAT_COUNT1,
                              patternCount);
                /* Configure MCRC sector count */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_SCOUNT_REG1,
                              SDL_MCRC_SCOUNT_REG1_SEC_COUNT1,
                              sectorCount);
                /* Configure MCRC operation mode */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                              SDL_MCRC_CTRL2_CH1_MODE,
                              mode);
                break;
            case SDL_MCRC_CHANNEL_2:
                /* Configure MCRC pattern count */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_PCOUNT_REG2,
                              SDL_MCRC_PCOUNT_REG2_PAT_COUNT2,
                              patternCount);
                /* Configure MCRC sector count */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_SCOUNT_REG2,
                              SDL_MCRC_SCOUNT_REG2_SEC_COUNT2,
                              sectorCount);
                /* Configure MSDL_MCRC operation mode */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                              SDL_MCRC_CTRL2_CH2_MODE,
                              mode);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                /* Configure MSDL_MCRC pattern count */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_PCOUNT_REG3,
                              SDL_MCRC_PCOUNT_REG3_PAT_COUNT3,
                              patternCount);
                /* Configure MSDL_MCRC sector count */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_SCOUNT_REG3,
                              SDL_MCRC_SCOUNT_REG3_SEC_COUNT3,
                              sectorCount);
                /* Configure MSDL_MCRC operation mode */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                              SDL_MCRC_CTRL2_CH3_MODE,
                              mode);
                break;
            case SDL_MCRC_CHANNEL_4:
                /* Configure MCRC pattern count */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_PCOUNT_REG4,
                              SDL_MCRC_PCOUNT_REG4_PAT_COUNT4,
                              patternCount);
                /* Configure MCRC sector count */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_SCOUNT_REG4,
                              SDL_MCRC_SCOUNT_REG4_SEC_COUNT4,
                              sectorCount);
                /* Configure MCRC operation mode */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                              SDL_MCRC_CTRL2_CH4_MODE,
                              mode);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2082
 */
int32_t SDL_MCRC_verifyConfig(SDL_MCRC_InstType  instance,
                              SDL_MCRC_Channel_t channel,
                              uint32_t           patternCount,
                              uint32_t           sectorCount,
                              SDL_MCRC_ModeType  mode)
{
    int32_t            status = SDL_PASS;
    uint32_t           pCount;
    uint32_t           sCount;
    uint32_t           baseAddr;
    SDL_MCRC_ModeType mcrcMode;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)    ||
        (patternCount > SDL_MCRC_PATTERN_COUNT_MAX)  ||
        (sectorCount > SDL_MCRC_SECTOR_COUNT_MAX)    ||
        (mode > SDL_MCRC_CTRL2_CH1_MODE_FULLCPU))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        /* Read MCRC pattern count, sector count and operation mode */
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                pCount = HW_RD_FIELD32(baseAddr + SDL_MCRC_PCOUNT_REG1,
                                       SDL_MCRC_PCOUNT_REG1_PAT_COUNT1);
                sCount = HW_RD_FIELD32(baseAddr + SDL_MCRC_SCOUNT_REG1,
                                       SDL_MCRC_SCOUNT_REG1_SEC_COUNT1);
                mcrcMode = HW_RD_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                                         SDL_MCRC_CTRL2_CH1_MODE);
                break;
            case SDL_MCRC_CHANNEL_2:
                pCount = HW_RD_FIELD32(baseAddr + SDL_MCRC_PCOUNT_REG2,
                                       SDL_MCRC_PCOUNT_REG2_PAT_COUNT2);
                sCount = HW_RD_FIELD32(baseAddr + SDL_MCRC_SCOUNT_REG2,
                                       SDL_MCRC_SCOUNT_REG2_SEC_COUNT2);
                mcrcMode = HW_RD_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                                         SDL_MCRC_CTRL2_CH2_MODE);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                pCount = HW_RD_FIELD32(baseAddr + SDL_MCRC_PCOUNT_REG3,
                                       SDL_MCRC_PCOUNT_REG3_PAT_COUNT3);
                sCount = HW_RD_FIELD32(baseAddr + SDL_MCRC_SCOUNT_REG3,
                                       SDL_MCRC_SCOUNT_REG3_SEC_COUNT3);
                mcrcMode = HW_RD_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                                         SDL_MCRC_CTRL2_CH3_MODE);
                break;
            case SDL_MCRC_CHANNEL_4:
                pCount = HW_RD_FIELD32(baseAddr + SDL_MCRC_PCOUNT_REG4,
                                       SDL_MCRC_PCOUNT_REG4_PAT_COUNT4);
                sCount = HW_RD_FIELD32(baseAddr + SDL_MCRC_SCOUNT_REG4,
                                       SDL_MCRC_SCOUNT_REG4_SEC_COUNT4);
                mcrcMode = HW_RD_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                                         SDL_MCRC_CTRL2_CH4_MODE);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    if (status == SDL_PASS)
    {
        if ((pCount != patternCount) ||
            (sCount != sectorCount)  ||
            (mcrcMode != mode))
        {
            status = SDL_EFAIL;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2086
 */
int32_t SDL_MCRC_channelReset(SDL_MCRC_InstType instance,
                              SDL_MCRC_Channel_t channel)
{
    int32_t status = SDL_PASS;
    uint32_t baseAddr;

    if (SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL0,
                              SDL_MCRC_CTRL0_CH1_PSA_SWRE,
                              SDL_MCRC_CTRL0_CH1_PSA_SWRE_ON);
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL0,
                              SDL_MCRC_CTRL0_CH1_PSA_SWRE,
                              SDL_MCRC_CTRL0_CH1_PSA_SWRE_OFF);
                break;
            case SDL_MCRC_CHANNEL_2:
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL0,
                              SDL_MCRC_CTRL0_CH2_PSA_SWRE,
                              SDL_MCRC_CTRL0_CH2_PSA_SWRE_ON);
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL0,
                              SDL_MCRC_CTRL0_CH2_PSA_SWRE,
                              SDL_MCRC_CTRL0_CH2_PSA_SWRE_OFF);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL0,
                              SDL_MCRC_CTRL0_CH3_PSA_SWRE,
                              SDL_MCRC_CTRL0_CH3_PSA_SWRE_ON);
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL0,
                              SDL_MCRC_CTRL0_CH3_PSA_SWRE,
                              SDL_MCRC_CTRL0_CH3_PSA_SWRE_OFF);
                break;
            case SDL_MCRC_CHANNEL_4:
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL0,
                              SDL_MCRC_CTRL0_CH4_PSA_SWRE,
                              SDL_MCRC_CTRL0_CH4_PSA_SWRE_ON);
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL0,
                              SDL_MCRC_CTRL0_CH4_PSA_SWRE,
                              SDL_MCRC_CTRL0_CH4_PSA_SWRE_OFF);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2090
 */
int32_t SDL_MCRC_getPSASig(SDL_MCRC_InstType     instance,
                           SDL_MCRC_Channel_t    channel,
                           SDL_MCRC_Signature_t *pPSAsign)
{
    int32_t status = SDL_PASS;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)    ||
        (pPSAsign == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                pPSAsign->regH = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SIGREGH1);
                pPSAsign->regL = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SIGREGL1);
                break;
            case SDL_MCRC_CHANNEL_2:
                pPSAsign->regH = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SIGREGH2);
                pPSAsign->regL = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SIGREGL2);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                pPSAsign->regH = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SIGREGH3);
                pPSAsign->regL = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SIGREGL3);
                break;
            case SDL_MCRC_CHANNEL_4:
                pPSAsign->regH = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SIGREGH4);
                pPSAsign->regL = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SIGREGL4);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2093
 */
int32_t SDL_MCRC_setPSASeedSig(SDL_MCRC_InstType           instance,
                               SDL_MCRC_Channel_t          channel,
                               const SDL_MCRC_Signature_t *pSeedSign)
{
    int32_t status = SDL_PASS;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)    ||
        (pSeedSign == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                /* Configure MCRC operation mode */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                              SDL_MCRC_CTRL2_CH1_MODE,
                              SDL_MCRC_CTRL2_CH1_MODE_DATA);
                HW_WR_REG32(baseAddr + SDL_MCRC_PSA_SIGREGH1, pSeedSign->regH);
                HW_WR_REG32(baseAddr + SDL_MCRC_PSA_SIGREGL1, pSeedSign->regL);
                break;
            case SDL_MCRC_CHANNEL_2:
                /* Configure MCRC operation mode */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                              SDL_MCRC_CTRL2_CH2_MODE,
                              SDL_MCRC_CTRL2_CH2_MODE_DATA);
                HW_WR_REG32(baseAddr + SDL_MCRC_PSA_SIGREGH2, pSeedSign->regH);
                HW_WR_REG32(baseAddr + SDL_MCRC_PSA_SIGREGL2, pSeedSign->regL);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                /* Configure MCRC operation mode */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                              SDL_MCRC_CTRL2_CH3_MODE,
                              SDL_MCRC_CTRL2_CH3_MODE_DATA);
                HW_WR_REG32(baseAddr + SDL_MCRC_PSA_SIGREGH3, pSeedSign->regH);
                HW_WR_REG32(baseAddr + SDL_MCRC_PSA_SIGREGL3, pSeedSign->regL);
                break;
            case SDL_MCRC_CHANNEL_4:
                /* Configure MCRC operation mode */
                HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                              SDL_MCRC_CTRL2_CH4_MODE,
                              SDL_MCRC_CTRL2_CH4_MODE_DATA);
                HW_WR_REG32(baseAddr + SDL_MCRC_PSA_SIGREGH4, pSeedSign->regH);
                HW_WR_REG32(baseAddr + SDL_MCRC_PSA_SIGREGL4, pSeedSign->regL);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2094
 */
int32_t SDL_MCRC_getPSASectorSig(SDL_MCRC_InstType     instance,
                                 SDL_MCRC_Channel_t    channel,
                                 SDL_MCRC_Signature_t *pSecSign)
{
    int32_t status = SDL_PASS;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)    ||
        (pSecSign == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                pSecSign->regH = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SECSIGREGH1);
                pSecSign->regL = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SECSIGREGL1);
                break;
            case SDL_MCRC_CHANNEL_2:
                pSecSign->regH = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SECSIGREGH2);
                pSecSign->regL = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SECSIGREGL2);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                pSecSign->regH = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SECSIGREGH3);
                pSecSign->regL = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SECSIGREGL3);
                break;
            case SDL_MCRC_CHANNEL_4:
                pSecSign->regH = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SECSIGREGH4);
                pSecSign->regL = HW_RD_REG32(baseAddr + SDL_MCRC_PSA_SECSIGREGL4);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2084
 */
int32_t SDL_MCRC_getIntrStatus(SDL_MCRC_InstType     instance,
                               SDL_MCRC_Channel_t    channel,
                               uint32_t             *pIntrStatus)
{
    int32_t  status = SDL_PASS;
    uint32_t intVal = 0U;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)    ||
        (pIntrStatus == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        intVal = HW_RD_REG32(baseAddr + SDL_MCRC_STATUS);
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                intVal = intVal & (SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL <<
                                   SDL_MCRC_STATUS_CH1_CCIT_SHIFT);
                break;
            case SDL_MCRC_CHANNEL_2:
                intVal = intVal & (SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL <<
                                   SDL_MCRC_STATUS_CH2_CCIT_SHIFT);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                intVal = intVal & (SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL <<
                                   SDL_MCRC_STATUS_CH3_CCIT_SHIFT);
                break;
            case SDL_MCRC_CHANNEL_4:
                intVal = intVal & (SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL <<
                                   SDL_MCRC_STATUS_CH4_CCIT_SHIFT);
                break;
#endif
            default:
                /* Invalid input */
                status = SDL_EBADARGS;
                break;
        }
    }

    if (status == SDL_PASS)
    {
        *pIntrStatus = intVal;
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2088
 */
int32_t SDL_MCRC_enableIntr(SDL_MCRC_InstType  instance,
                            SDL_MCRC_Channel_t channel,
                            uint32_t           intrMask)
{
    int32_t  status = SDL_PASS;
    uint32_t intVal = 0U;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)     ||
        ((intrMask & (~SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL)) != 0U))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                intVal = (intrMask << SDL_MCRC_INTS_CH1_CCITENS_SHIFT);
                break;
            case SDL_MCRC_CHANNEL_2:
                intVal = (intrMask << SDL_MCRC_INTS_CH2_CCITENS_SHIFT);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                intVal = (intrMask << SDL_MCRC_INTS_CH3_CCITENS_SHIFT);
                break;
            case SDL_MCRC_CHANNEL_4:
                intVal = (intrMask << SDL_MCRC_INTS_CH4_CCITENS_SHIFT);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    if (status == SDL_PASS)
    {
        HW_WR_REG32(baseAddr + SDL_MCRC_INTS, intVal);
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2089
 */
int32_t SDL_MCRC_disableIntr(SDL_MCRC_InstType  instance,
                             SDL_MCRC_Channel_t channel,
                             uint32_t           intrMask)
{
    int32_t  status = SDL_PASS;
    uint32_t intVal = 0U;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)     ||
        ((intrMask & (~SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL)) != 0U))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                intVal = (intrMask << SDL_MCRC_INTR_CH1_CCITENR_SHIFT);
                break;
            case SDL_MCRC_CHANNEL_2:
                intVal = (intrMask << SDL_MCRC_INTR_CH2_CCITENR_SHIFT);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                intVal = (intrMask << SDL_MCRC_INTR_CH3_CCITENR_SHIFT);
                break;
            case SDL_MCRC_CHANNEL_4:
                intVal = (intrMask << SDL_MCRC_INTR_CH4_CCITENR_SHIFT);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    if (status == SDL_PASS)
    {
        HW_WR_REG32(baseAddr + SDL_MCRC_INTR, intVal);
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2087
 */
int32_t SDL_MCRC_clearIntr(SDL_MCRC_InstType  instance,
                           SDL_MCRC_Channel_t channel,
                           uint32_t           intrMask)
{
    int32_t status = SDL_PASS;
    uint32_t intVal = 0U;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)     ||
        ((intrMask & (~SDL_MCRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL)) != 0U))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                intVal = (intrMask << SDL_MCRC_STATUS_CH1_CCIT_SHIFT);
                break;
            case SDL_MCRC_CHANNEL_2:
                intVal = (intrMask << SDL_MCRC_STATUS_CH2_CCIT_SHIFT);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                intVal = (intrMask << SDL_MCRC_STATUS_CH3_CCIT_SHIFT);
                break;
            case SDL_MCRC_CHANNEL_4:
                intVal = (intrMask << SDL_MCRC_STATUS_CH4_CCIT_SHIFT);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    if (status == SDL_PASS)
    {
        HW_WR_REG32(baseAddr + SDL_MCRC_STATUS, intVal);
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2092
 */
int32_t SDL_MCRC_isBusy(SDL_MCRC_InstType   instance,
                        SDL_MCRC_Channel_t  channel,
                        uint32_t           *pBusyFlag)
{
    int32_t  status = SDL_PASS;
    uint32_t busyVal;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)  ||
        (pBusyFlag == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        /* read busy status */
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                busyVal = HW_RD_FIELD32(baseAddr + SDL_MCRC_BUSY,
                                        SDL_MCRC_BUSY_CH1);

                break;
            case SDL_MCRC_CHANNEL_2:
                busyVal = HW_RD_FIELD32(baseAddr + SDL_MCRC_BUSY,
                                        SDL_MCRC_BUSY_CH2);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                busyVal = HW_RD_FIELD32(baseAddr + SDL_MCRC_BUSY,
                                        SDL_MCRC_BUSY_CH3);
                break;
            case SDL_MCRC_CHANNEL_4:
                busyVal = HW_RD_FIELD32(baseAddr + SDL_MCRC_BUSY,
                                        SDL_MCRC_BUSY_CH4);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    if (status == SDL_PASS)
    {
        *pBusyFlag = busyVal;
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2095
 */
int32_t SDL_MCRC_getCurSecNum(SDL_MCRC_InstType  instance,
                             SDL_MCRC_Channel_t  channel,
                             uint32_t           *pCurSecNum)
{
    int32_t  status = SDL_PASS;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)  ||
        (pCurSecNum == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        /* read current sector number */
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                *pCurSecNum = HW_RD_FIELD32(baseAddr + SDL_MCRC_CURSEC_REG1,
                                            SDL_MCRC_CURSEC_REG1_CURSEC1);
                break;
            case SDL_MCRC_CHANNEL_2:
                *pCurSecNum = HW_RD_FIELD32(baseAddr + SDL_MCRC_CURSEC_REG2,
                                            SDL_MCRC_CURSEC_REG2_CURSEC2);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                *pCurSecNum = HW_RD_FIELD32(baseAddr + SDL_MCRC_CURSEC_REG3,
                                            SDL_MCRC_CURSEC_REG3_CURSEC3);
                break;
            case SDL_MCRC_CHANNEL_4:
                *pCurSecNum = HW_RD_FIELD32(baseAddr + SDL_MCRC_CURSEC_REG4,
                                            SDL_MCRC_CURSEC_REG4_CURSEC4);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2096
 */
int32_t SDL_MCRC_getCurPSASig(SDL_MCRC_InstType     instance,
                              SDL_MCRC_Channel_t    channel,
                              SDL_MCRC_Signature_t *pCurPSASig)
{
    int32_t  status = SDL_PASS;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)  ||
        (pCurPSASig == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        /* read current PSA signature value */
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                pCurPSASig->regL = HW_RD_FIELD32(baseAddr + SDL_MCRC_REGL1,
                                                 SDL_MCRC_REGL1_CRC1);
                pCurPSASig->regH = HW_RD_FIELD32(baseAddr + SDL_MCRC_REGH1,
                                                 SDL_MCRC_REGH1_CRC1_47_32);
                break;
            case SDL_MCRC_CHANNEL_2:
                pCurPSASig->regL = HW_RD_FIELD32(baseAddr + SDL_MCRC_REGL2,
                                                 SDL_MCRC_REGL2_CRC2);
                pCurPSASig->regH = HW_RD_FIELD32(baseAddr + SDL_MCRC_REGH2,
                                                 SDL_MCRC_REGH2_CRC2_63_32);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                pCurPSASig->regL = HW_RD_FIELD32(baseAddr + SDL_MCRC_REGL3,
                                                 SDL_MCRC_REGL3_CRC3);
                pCurPSASig->regH = HW_RD_FIELD32(baseAddr + SDL_MCRC_REGH3,
                                                 SDL_MCRC_REGH3_CRC3_63_32);
                break;
            case SDL_MCRC_CHANNEL_4:
                pCurPSASig->regL = HW_RD_FIELD32(baseAddr + SDL_MCRC_REGL4,
                                                 SDL_MCRC_REGL4_CRC4);
                pCurPSASig->regH = HW_RD_FIELD32(baseAddr + SDL_MCRC_REGH4,
                                                 SDL_MCRC_REGH4_CRC4_63_32);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2083
 */
int32_t SDL_MCRC_readStaticReg(SDL_MCRC_InstType instance,
                               SDL_MCRC_StaticRegs_t *pStaticRegs)
{
    int32_t  status = SDL_PASS;
    uint32_t i;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)  ||
        (pStaticRegs == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        pStaticRegs->CTRL0   = HW_RD_REG32(baseAddr + SDL_MCRC_CTRL0);
        pStaticRegs->CTRL1   = HW_RD_REG32(baseAddr + SDL_MCRC_CTRL1);
        pStaticRegs->BUS_SEL = HW_RD_REG32(baseAddr + SDL_MCRC_MCRC_BUS_SEL);

        for (i = ((uint32_t) (0U)); i < SDL_MCRC_MAX_NUM_OF_CHANNELS; i++)
        {
            pStaticRegs->channelRegs[i].PCOUNT   = HW_RD_REG32(baseAddr + SDL_MCRC_getRegsOffset(i) + SDL_MCRC_PCOUNT_REG1);
            pStaticRegs->channelRegs[i].SCOUNT   = HW_RD_REG32(baseAddr + SDL_MCRC_getRegsOffset(i) + SDL_MCRC_SCOUNT_REG1);
            pStaticRegs->channelRegs[i].WDTOPLD  = HW_RD_REG32(baseAddr + SDL_MCRC_getRegsOffset(i) + SDL_MCRC_WDTOPLD1);
            pStaticRegs->channelRegs[i].BCTOPLD  = HW_RD_REG32(baseAddr + SDL_MCRC_getRegsOffset(i) + SDL_MCRC_BCTOPLD1);
        }
        status = SDL_PASS;
    }
    return status;
}

/**
 *  Design: PROC_SDL-2091
 */
int32_t SDL_MCRC_getPSASigRegAddr(SDL_MCRC_InstType instance, SDL_MCRC_Channel_t channel,
                                  SDL_MCRC_SignatureRegAddr_t *pMCRCregAddr)
{
    int32_t status = SDL_PASS;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)   ||
        (pMCRCregAddr == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                pMCRCregAddr->regH = (baseAddr + SDL_MCRC_PSA_SIGREGH1);
                pMCRCregAddr->regL = (baseAddr + SDL_MCRC_PSA_SIGREGL1);
                break;
            case SDL_MCRC_CHANNEL_2:
                pMCRCregAddr->regH = (baseAddr + SDL_MCRC_PSA_SIGREGH2);
                pMCRCregAddr->regL = (baseAddr + SDL_MCRC_PSA_SIGREGL2);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            case SDL_MCRC_CHANNEL_3:
                pMCRCregAddr->regH = (baseAddr + SDL_MCRC_PSA_SIGREGH3);
                pMCRCregAddr->regL = (baseAddr + SDL_MCRC_PSA_SIGREGL3);
                break;
            case SDL_MCRC_CHANNEL_4:
                pMCRCregAddr->regH = (baseAddr + SDL_MCRC_PSA_SIGREGH4);
                pMCRCregAddr->regL = (baseAddr + SDL_MCRC_PSA_SIGREGL4);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

/* Helper function to write the data */
static int32_t SDL_MCRC_dataWrite(const SDL_MCRC_DataConfig_t *pDataConfig,
                                  const SDL_MCRC_SignatureRegAddr_t *sigRegAddr)
{
    int32_t result = SDL_PASS;
    uint32_t i;

    switch (pDataConfig->dataBitSize)
    {
        case SDL_MCRC_DATA_8_BIT:
        {
            uint8_t *pData = (uint8_t *)(pDataConfig->pMCRCData);
            for (i = (uint32_t)(0U); i < pDataConfig->size; i++)
            {
                HW_WR_REG8(sigRegAddr->regL, pData[i]);
            }
	}
        break;
        case SDL_MCRC_DATA_16_BIT:
        {
            uint16_t *pData = (uint16_t *)(pDataConfig->pMCRCData);
            for (i = (uint32_t)(0U); i < (pDataConfig->size / (uint32_t)2U); i++)
            {
                HW_WR_REG16(sigRegAddr->regL, pData[i]);
            }
        }
	break;
	case SDL_MCRC_DATA_32_BIT:
	{
            uint32_t *pData = (uint32_t *)(pDataConfig->pMCRCData);
            for (i = (uint32_t)(0U); i < (pDataConfig->size / (uint32_t)4U); i++)
            {
                HW_WR_REG32(sigRegAddr->regL, pData[i]);
            }
	}
	break;
        default:
            result = SDL_EBADARGS;
        break;
    }

    return result;
}

/**
 *  Design: PROC_SDL-2097
 */
int32_t SDL_MCRC_computeSignCPUmode (SDL_MCRC_InstType instance,
                                     SDL_MCRC_Channel_t channel,
                                     const SDL_MCRC_DataConfig_t *pDataConfig,
                                     SDL_MCRC_Signature_t *sectSignVal)
{
    int32_t                      result = SDL_EFAIL;
    SDL_MCRC_SignatureRegAddr_t  sigRegAddr;

    if ((pDataConfig == (NULL_PTR)) || (sectSignVal == (NULL_PTR)) ||
        (pDataConfig->pMCRCData == (NULL_PTR)))
    {
        result = SDL_EBADARGS;
    }
    else
    {
        /* Get the MCRC signature register address for the MCRC channel */
        result = SDL_MCRC_getPSASigRegAddr(instance,
                                           channel,
                                           &sigRegAddr);

        /* Write MCRC data */
        if (result == SDL_PASS)
	{
            result = SDL_MCRC_dataWrite(pDataConfig, &sigRegAddr);
	}

        if (result == SDL_PASS)
        {
            /* Get the MCRC signature value */
            result = SDL_MCRC_getPSASectorSig(instance,
                                              channel,
                                              sectSignVal);
        }
    }
    return (result);
}

int32_t SDL_MCRC_getCRCRegAddr(SDL_MCRC_InstType instance,
                          SDL_MCRC_Channel_t           channel,
                          SDL_MCRC_SignatureRegAddr_t *pCRCRegAddr)
{
    int32_t status = SDL_PASS;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)   ||
            (pCRCRegAddr == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
                pCRCRegAddr->regH = (baseAddr + SDL_MCRC_REGH1);
                pCRCRegAddr->regL = (baseAddr + SDL_MCRC_REGL1);
                break;
            case SDL_MCRC_CHANNEL_2:
                pCRCRegAddr->regH = (baseAddr + SDL_MCRC_REGH2);
                pCRCRegAddr->regL = (baseAddr + SDL_MCRC_REGL2);
                break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
			case SDL_MCRC_CHANNEL_3:
			    pCRCRegAddr->regH = (baseAddr + SDL_MCRC_REGH3);
			    pCRCRegAddr->regL = (baseAddr + SDL_MCRC_REGL3);
                break;
            case SDL_MCRC_CHANNEL_4:
                pCRCRegAddr->regH = (baseAddr + SDL_MCRC_REGH4);
                pCRCRegAddr->regL = (baseAddr + SDL_MCRC_REGL4);
                break;
#endif
            default:
                status = SDL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t SDL_MCRC_configCRCType(SDL_MCRC_InstType instance,
					 SDL_MCRC_Channel_t       channel)
{
	int32_t status = SDL_PASS;
	uint32_t baseAddr;

	if (((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)))
	{
		status = SDL_EBADARGS;
	}
	else
	{
		switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
				SDL_REG32_WR(baseAddr + SDL_MCRC_CTRL0, SDL_MCRC_TYPE_64BIT);
				break;
			case SDL_MCRC_CHANNEL_2:
				SDL_REG32_WR(baseAddr + SDL_MCRC_CTRL0, (SDL_MCRC_TYPE_64BIT << 8U));
				break;
#if defined(SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
			case SDL_MCRC_CHANNEL_3:
				SDL_REG32_WR(baseAddr + SDL_MCRC_CTRL0, (SDL_MCRC_TYPE_64BIT << 16U));
				break;
			case SDL_MCRC_CHANNEL_4:
				SDL_REG32_WR(baseAddr + SDL_MCRC_CTRL0, (SDL_MCRC_TYPE_64BIT << 24U));
				break;
#endif
			default:
                status = SDL_EBADARGS;
                break;
		}
	}
    return (status);
}

#if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
int32_t SDL_MCRC_configDataWidth(SDL_MCRC_InstType instance,
					SDL_MCRC_Channel_t channel, uint32_t datawidth)
{
	int32_t status = SDL_PASS;
	uint32_t baseAddr;

	if (((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)))
	{
		status = SDL_EBADARGS;
	}
	else
	{
		switch (channel)
        {
            case SDL_MCRC_CHANNEL_1:
				SDL_REG32_WR(baseAddr + SDL_MCRC_CTRL0,
                            (datawidth << SDL_MCRC_CTRL0_CH1_DW_SEL_SHIFT));
				break;
			case SDL_MCRC_CHANNEL_2:
				SDL_REG32_WR(baseAddr + SDL_MCRC_CTRL0,
                            (datawidth << SDL_MCRC_CTRL0_CH2_DW_SEL_SHIFT));
				break;
			case SDL_MCRC_CHANNEL_3:
				SDL_REG32_WR(baseAddr + SDL_MCRC_CTRL0,
                            (datawidth << SDL_MCRC_CTRL0_CH3_DW_SEL_SHIFT));
				break;
			case SDL_MCRC_CHANNEL_4:
				SDL_REG32_WR(baseAddr + SDL_MCRC_CTRL0,
                            (datawidth << SDL_MCRC_CTRL0_CH4_DW_SEL_SHIFT));
				break;
			default:
                status = SDL_EBADARGS;
                break;
		}
	}
    return (status);
}
#endif

