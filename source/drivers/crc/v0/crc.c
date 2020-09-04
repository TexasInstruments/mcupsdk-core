/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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
 */

/**
 *  \file     crc.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of CRC.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/crc.h>
#include <drivers/hw_include/csl_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* channel specific register group offset */
static inline uint32_t CRCGetRegsOffset(uint32_t channel);
static inline uint32_t CRCGetRegsOffset(uint32_t channel)
{
    return (channel * 0x40U);
}

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t CRC_initialize(uint32_t     baseAddr,
                      CRC_Channel_t channel,
                      uint32_t     crcWatchdogPreload,
                      uint32_t     crcBlockPreload)
{
    int32_t status = CSL_PASS;

    if ((baseAddr == ((uint32_t)NULL))          ||
        (crcWatchdogPreload > CRC_WDTOPLD_MAX)  ||
        (crcBlockPreload > CRC_BCTOPLD_MAX))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                /* Configure watchdog pre-load value */
                HW_WR_FIELD32(baseAddr + CRC_WDTOPLD1,
                              CRC_WDTOPLD1,
                              crcWatchdogPreload);
                /* Configure clock pre-load value */
                HW_WR_FIELD32(baseAddr + CRC_BCTOPLD1,
                              CRC_BCTOPLD1,
                              crcBlockPreload);
                break;
            case CRC_CHANNEL_2:
                /* Configure watchdog pre-load value */
                HW_WR_FIELD32(baseAddr + CRC_WDTOPLD2,
                              CRC_WDTOPLD2,
                              crcWatchdogPreload);
                /* Configure clock pre-load value */
                HW_WR_FIELD32(baseAddr + CRC_BCTOPLD2,
                              CRC_BCTOPLD2,
                              crcBlockPreload);
                break;
            case CRC_CHANNEL_3:
                /* Configure watchdog pre-load value */
                HW_WR_FIELD32(baseAddr + CRC_WDTOPLD3,
                              CRC_WDTOPLD3,
                              crcWatchdogPreload);
                /* Configure clock pre-load value */
                HW_WR_FIELD32(baseAddr + CRC_BCTOPLD3,
                              CRC_BCTOPLD3,
                              crcBlockPreload);
                break;
            case CRC_CHANNEL_4:
                /* Configure watchdog pre-load value */
                HW_WR_FIELD32(baseAddr + CRC_WDTOPLD4,
                              CRC_WDTOPLD4,
                              crcWatchdogPreload);
                /* Configure clock pre-load value */
                HW_WR_FIELD32(baseAddr + CRC_BCTOPLD4,
                              CRC_BCTOPLD4,
                              crcBlockPreload);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_verifyInitialize(uint32_t     baseAddr,
                            CRC_Channel_t channel,
                            uint32_t     crcWatchdogPreload,
                            uint32_t     crcBlockPreload)
{
    int32_t  status = CSL_PASS;
    uint32_t watchdogPreload;
    uint32_t blockPreload;

    if ((baseAddr == ((uint32_t)NULL))          ||
        (crcWatchdogPreload > CRC_WDTOPLD_MAX)  ||
        (crcBlockPreload > CRC_BCTOPLD_MAX))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        /* Read WDT preload value and block complete preload value */
        switch (channel)
        {
            case CRC_CHANNEL_1:
                watchdogPreload = HW_RD_FIELD32(baseAddr + CRC_WDTOPLD1,
                                                CRC_WDTOPLD1);
                blockPreload = HW_RD_FIELD32(baseAddr + CRC_BCTOPLD1,
                                             CRC_BCTOPLD1);
                break;
            case CRC_CHANNEL_2:
                watchdogPreload = HW_RD_FIELD32(baseAddr + CRC_WDTOPLD2,
                                                CRC_WDTOPLD2);
                blockPreload = HW_RD_FIELD32(baseAddr + CRC_BCTOPLD2,
                                             CRC_BCTOPLD2);
                break;
            case CRC_CHANNEL_3:
                watchdogPreload = HW_RD_FIELD32(baseAddr + CRC_WDTOPLD3,
                                                CRC_WDTOPLD3);
                blockPreload = HW_RD_FIELD32(baseAddr + CRC_BCTOPLD3,
                                             CRC_BCTOPLD3);
                break;
            case CRC_CHANNEL_4:
                watchdogPreload = HW_RD_FIELD32(baseAddr + CRC_WDTOPLD4,
                                                CRC_WDTOPLD4);
                blockPreload = HW_RD_FIELD32(baseAddr + CRC_BCTOPLD4,
                                             CRC_BCTOPLD4);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    if (status == CSL_PASS)
    {
        if ((watchdogPreload != crcWatchdogPreload) ||
            (blockPreload != crcBlockPreload))
        {
            status = CSL_EFAIL;
        }
    }

    return (status);
}

int32_t CRC_configure(uint32_t           baseAddr,
                     CRC_Channel_t       channel,
                     uint32_t           crcPatternCount,
                     uint32_t           crcSectorCount,
                     CRC_OperationMode_t crcMode)
{
    int32_t status = CSL_PASS;

    if ((baseAddr == ((uint32_t)NULL))             ||
        (crcPatternCount > CRC_PATTERN_COUNT_MAX)  ||
        (crcSectorCount > CRC_SECTOR_COUNT_MAX)    ||
        (crcMode > CRC_CTRL2_CH1_MODE_FULLCPU))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                /* Configure CRC pattern count */
                HW_WR_FIELD32(baseAddr + CRC_PCOUNT_REG1,
                              CRC_PCOUNT_REG1_PAT_COUNT1,
                              crcPatternCount);
                /* Configure CRC sector count */
                HW_WR_FIELD32(baseAddr + CRC_SCOUNT_REG1,
                              CRC_SCOUNT_REG1_SEC_COUNT1,
                              crcSectorCount);
                /* Configure CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH1_MODE,
                              crcMode);
                break;
            case CRC_CHANNEL_2:
                /* Configure CRC pattern count */
                HW_WR_FIELD32(baseAddr + CRC_PCOUNT_REG2,
                              CRC_PCOUNT_REG2_PAT_COUNT2,
                              crcPatternCount);
                /* Configure CRC sector count */
                HW_WR_FIELD32(baseAddr + CRC_SCOUNT_REG2,
                              CRC_SCOUNT_REG2_SEC_COUNT2,
                              crcSectorCount);
                /* Configure CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH2_MODE,
                              crcMode);
                break;
            case CRC_CHANNEL_3:
                /* Configure CRC pattern count */
                HW_WR_FIELD32(baseAddr + CRC_PCOUNT_REG3,
                              CRC_PCOUNT_REG3_PAT_COUNT3,
                              crcPatternCount);
                /* Configure CRC sector count */
                HW_WR_FIELD32(baseAddr + CRC_SCOUNT_REG3,
                              CRC_SCOUNT_REG3_SEC_COUNT3,
                              crcSectorCount);
                /* Configure CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH3_MODE,
                              crcMode);
                break;
            case CRC_CHANNEL_4:
                /* Configure CRC pattern count */
                HW_WR_FIELD32(baseAddr + CRC_PCOUNT_REG4,
                              CRC_PCOUNT_REG4_PAT_COUNT4,
                              crcPatternCount);
                /* Configure CRC sector count */
                HW_WR_FIELD32(baseAddr + CRC_SCOUNT_REG4,
                              CRC_SCOUNT_REG4_SEC_COUNT4,
                              crcSectorCount);
                /* Configure CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH4_MODE,
                              crcMode);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_verifyConfigure(uint32_t           baseAddr,
                           CRC_Channel_t       channel,
                           uint32_t           crcPatternCount,
                           uint32_t           crcSectorCount,
                           CRC_OperationMode_t crcMode)
{
    int32_t            status = CSL_PASS;
    uint32_t           pCount;
    uint32_t           sCount;
    CRC_OperationMode_t mode;

    if ((baseAddr == ((uint32_t)NULL))             ||
        (crcPatternCount > CRC_PATTERN_COUNT_MAX)  ||
        (crcSectorCount > CRC_SECTOR_COUNT_MAX)    ||
        (crcMode > CRC_CTRL2_CH1_MODE_FULLCPU))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        /* Read CRC pattern count, sector count and operation mode */
        switch (channel)
        {
            case CRC_CHANNEL_1:
                pCount = HW_RD_FIELD32(baseAddr + CRC_PCOUNT_REG1,
                                       CRC_PCOUNT_REG1_PAT_COUNT1);
                sCount = HW_RD_FIELD32(baseAddr + CRC_SCOUNT_REG1,
                                       CRC_SCOUNT_REG1_SEC_COUNT1);
                mode = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                                     CRC_CTRL2_CH1_MODE);
                break;
            case CRC_CHANNEL_2:
                pCount = HW_RD_FIELD32(baseAddr + CRC_PCOUNT_REG2,
                                       CRC_PCOUNT_REG2_PAT_COUNT2);
                sCount = HW_RD_FIELD32(baseAddr + CRC_SCOUNT_REG2,
                                       CRC_SCOUNT_REG2_SEC_COUNT2);
                mode = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                                     CRC_CTRL2_CH2_MODE);
                break;
            case CRC_CHANNEL_3:
                pCount = HW_RD_FIELD32(baseAddr + CRC_PCOUNT_REG3,
                                       CRC_PCOUNT_REG3_PAT_COUNT3);
                sCount = HW_RD_FIELD32(baseAddr + CRC_SCOUNT_REG3,
                                       CRC_SCOUNT_REG3_SEC_COUNT3);
                mode = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                                     CRC_CTRL2_CH3_MODE);
                break;
            case CRC_CHANNEL_4:
                pCount = HW_RD_FIELD32(baseAddr + CRC_PCOUNT_REG4,
                                       CRC_PCOUNT_REG4_PAT_COUNT4);
                sCount = HW_RD_FIELD32(baseAddr + CRC_SCOUNT_REG4,
                                       CRC_SCOUNT_REG4_SEC_COUNT4);
                mode = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                                     CRC_CTRL2_CH4_MODE);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    if (status == CSL_PASS)
    {
        if ((pCount != crcPatternCount) ||
            (sCount != crcSectorCount)  ||
            (mode != crcMode))
        {
            status = CSL_EFAIL;
        }
    }

    return (status);
}

int32_t CRC_channelReset(uint32_t     baseAddr,
                        CRC_Channel_t channel)
{
    int32_t status = CSL_PASS;

    if (baseAddr == (uint32_t)NULL)
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                HW_WR_FIELD32(baseAddr + CRC_CTRL0,
                              CRC_CTRL0_CH1_PSA_SWRE,
                              CRC_CTRL0_CH1_PSA_SWRE_ON);
                HW_WR_FIELD32(baseAddr + CRC_CTRL0,
                              CRC_CTRL0_CH1_PSA_SWRE,
                              CRC_CTRL0_CH1_PSA_SWRE_OFF);
                break;
            case CRC_CHANNEL_2:
                HW_WR_FIELD32(baseAddr + CRC_CTRL0,
                              CRC_CTRL0_CH2_PSA_SWRE,
                              CRC_CTRL0_CH2_PSA_SWRE_ON);
                HW_WR_FIELD32(baseAddr + CRC_CTRL0,
                              CRC_CTRL0_CH2_PSA_SWRE,
                              CRC_CTRL0_CH2_PSA_SWRE_OFF);
                break;
            case CRC_CHANNEL_3:
                HW_WR_FIELD32(baseAddr + CRC_CTRL0,
                              CRC_CTRL0_CH3_PSA_SWRE,
                              CRC_CTRL0_CH3_PSA_SWRE_ON);
                HW_WR_FIELD32(baseAddr + CRC_CTRL0,
                              CRC_CTRL0_CH3_PSA_SWRE,
                              CRC_CTRL0_CH3_PSA_SWRE_OFF);
                break;
            case CRC_CHANNEL_4:
                HW_WR_FIELD32(baseAddr + CRC_CTRL0,
                              CRC_CTRL0_CH4_PSA_SWRE,
                              CRC_CTRL0_CH4_PSA_SWRE_ON);
                HW_WR_FIELD32(baseAddr + CRC_CTRL0,
                              CRC_CTRL0_CH4_PSA_SWRE,
                              CRC_CTRL0_CH4_PSA_SWRE_OFF);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_getPSASigRegAddr(uint32_t               baseAddr,
                            CRC_Channel_t           channel,
                            CRC_SignatureRegAddr *pCRCRegAddr)
{
    int32_t status = CSL_PASS;

    if ((baseAddr == (uint32_t)NULL)   ||
        (pCRCRegAddr == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                pCRCRegAddr->regH = (baseAddr + CRC_PSA_SIGREGH1);
                pCRCRegAddr->regL = (baseAddr + CRC_PSA_SIGREGL1);
                break;
            case CRC_CHANNEL_2:
                pCRCRegAddr->regH = (baseAddr + CRC_PSA_SIGREGH2);
                pCRCRegAddr->regL = (baseAddr + CRC_PSA_SIGREGL2);
                break;
            case CRC_CHANNEL_3:
                pCRCRegAddr->regH = (baseAddr + CRC_PSA_SIGREGH3);
                pCRCRegAddr->regL = (baseAddr + CRC_PSA_SIGREGL3);
                break;
            case CRC_CHANNEL_4:
                pCRCRegAddr->regH = (baseAddr + CRC_PSA_SIGREGH4);
                pCRCRegAddr->regL = (baseAddr + CRC_PSA_SIGREGL4);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_getPSASig(uint32_t        baseAddr,
                     CRC_Channel_t    channel,
                     CRC_Signature *pCRCPSASign)
{
    int32_t status = CSL_PASS;

    if ((baseAddr == ((uint32_t)NULL))   ||
        (pCRCPSASign == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                pCRCPSASign->regH = HW_RD_REG32(baseAddr + CRC_PSA_SIGREGH1);
                pCRCPSASign->regL = HW_RD_REG32(baseAddr + CRC_PSA_SIGREGL1);
                break;
            case CRC_CHANNEL_2:
                pCRCPSASign->regH = HW_RD_REG32(baseAddr + CRC_PSA_SIGREGH2);
                pCRCPSASign->regL = HW_RD_REG32(baseAddr + CRC_PSA_SIGREGL2);
                break;
            case CRC_CHANNEL_3:
                pCRCPSASign->regH = HW_RD_REG32(baseAddr + CRC_PSA_SIGREGH3);
                pCRCPSASign->regL = HW_RD_REG32(baseAddr + CRC_PSA_SIGREGL3);
                break;
            case CRC_CHANNEL_4:
                pCRCPSASign->regH = HW_RD_REG32(baseAddr + CRC_PSA_SIGREGH4);
                pCRCPSASign->regL = HW_RD_REG32(baseAddr + CRC_PSA_SIGREGL4);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_setPSASeedSig(uint32_t              baseAddr,
                         CRC_Channel_t          channel,
                         const CRC_Signature *pCRCPSASeedSign)
{
    int32_t status = CSL_PASS;
    uint32_t currentMode;

    if ((baseAddr == ((uint32_t)NULL))   ||
        (pCRCPSASeedSign == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                /* Get Current operation mode */
                currentMode = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH1_MODE);

                /* Configure CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH1_MODE,
                              CRC_CTRL2_CH1_MODE_DATA);
                HW_WR_REG32(baseAddr + CRC_PSA_SIGREGH1, pCRCPSASeedSign->regH);
                HW_WR_REG32(baseAddr + CRC_PSA_SIGREGL1, pCRCPSASeedSign->regL);

                /* reset the CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH1_MODE,
                              currentMode);
                break;
            case CRC_CHANNEL_2:
                /* Get Current operation mode */
                currentMode = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH2_MODE);

                /* Configure CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH2_MODE,
                              CRC_CTRL2_CH2_MODE_DATA);
                HW_WR_REG32(baseAddr + CRC_PSA_SIGREGH2, pCRCPSASeedSign->regH);
                HW_WR_REG32(baseAddr + CRC_PSA_SIGREGL2, pCRCPSASeedSign->regL);

                /* reset the CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH2_MODE,
                              currentMode);
                break;
            case CRC_CHANNEL_3:
                /* Get Current operation mode */
                currentMode = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH3_MODE);

                /* Configure CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH3_MODE,
                              CRC_CTRL2_CH3_MODE_DATA);
                HW_WR_REG32(baseAddr + CRC_PSA_SIGREGH3, pCRCPSASeedSign->regH);
                HW_WR_REG32(baseAddr + CRC_PSA_SIGREGL3, pCRCPSASeedSign->regL);

                /* reset the CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH3_MODE,
                              currentMode);
                break;
            case CRC_CHANNEL_4:
                /* Get Current operation mode */
                currentMode = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH4_MODE);

                /* Configure CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH4_MODE,
                              CRC_CTRL2_CH4_MODE_DATA);
                HW_WR_REG32(baseAddr + CRC_PSA_SIGREGH4, pCRCPSASeedSign->regH);
                HW_WR_REG32(baseAddr + CRC_PSA_SIGREGL4, pCRCPSASeedSign->regL);

                /* reset the CRC operation mode */
                HW_WR_FIELD32(baseAddr + CRC_CTRL2,
                              CRC_CTRL2_CH4_MODE,
                              currentMode);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_getPSASectorSig(uint32_t        baseAddr,
                           CRC_Channel_t    channel,
                           CRC_Signature *pCRCSectorSign)
{
    int32_t status = CSL_PASS;

    if ((baseAddr == ((uint32_t)NULL))   ||
        (pCRCSectorSign == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                pCRCSectorSign->regH = HW_RD_REG32(baseAddr + CRC_PSA_SECSIGREGH1);
                pCRCSectorSign->regL = HW_RD_REG32(baseAddr + CRC_PSA_SECSIGREGL1);
                break;
            case CRC_CHANNEL_2:
                pCRCSectorSign->regH = HW_RD_REG32(baseAddr + CRC_PSA_SECSIGREGH2);
                pCRCSectorSign->regL = HW_RD_REG32(baseAddr + CRC_PSA_SECSIGREGL2);
                break;
            case CRC_CHANNEL_3:
                pCRCSectorSign->regH = HW_RD_REG32(baseAddr + CRC_PSA_SECSIGREGH3);
                pCRCSectorSign->regL = HW_RD_REG32(baseAddr + CRC_PSA_SECSIGREGL3);
                break;
            case CRC_CHANNEL_4:
                pCRCSectorSign->regH = HW_RD_REG32(baseAddr + CRC_PSA_SECSIGREGH4);
                pCRCSectorSign->regL = HW_RD_REG32(baseAddr + CRC_PSA_SECSIGREGL4);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_getHighestPriorityIntrStatus(uint32_t baseAddr, uint32_t *pIntVecAddr)
{
    int32_t status;

    if ((baseAddr == ((uint32_t)NULL))   ||
        (pIntVecAddr == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        *pIntVecAddr = HW_RD_FIELD32(baseAddr + CRC_INT_OFFSET_REG, CRC_INT_OFFSET_REG);
        status = CSL_PASS;
    }

    return (status);
}

int32_t CRC_getIntrStatus(uint32_t      baseAddr,
                         CRC_Channel_t  channel,
                         uint32_t     *pIntrStatus)
{
    int32_t  status = CSL_PASS;
    uint32_t intVal = 0U;

    if ((baseAddr == ((uint32_t)NULL))   ||
        (pIntrStatus == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        intVal = HW_RD_REG32(baseAddr + CRC_STATUS);
        switch (channel)
        {
            case CRC_CHANNEL_1:
                intVal = intVal & (CRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL <<
                                   CRC_STATUS_CH1_CCIT_SHIFT);
                break;
            case CRC_CHANNEL_2:
                intVal = intVal & (CRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL <<
                                   CRC_STATUS_CH2_CCIT_SHIFT);
                break;
            case CRC_CHANNEL_3:
                intVal = intVal & (CRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL <<
                                   CRC_STATUS_CH3_CCIT_SHIFT);
                break;
            case CRC_CHANNEL_4:
                intVal = intVal & (CRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL <<
                                   CRC_STATUS_CH4_CCIT_SHIFT);
                break;
            default:
                /* Invalid input */
                status = CSL_EBADARGS;
                break;
        }
    }

    if (status == CSL_PASS)
    {
        *pIntrStatus = intVal;
    }

    return (status);
}

int32_t CRC_enableIntr(uint32_t     baseAddr,
                      CRC_Channel_t channel,
                      uint32_t     intrMask)
{
    int32_t  status = CSL_PASS;
    uint32_t intVal = 0U;

    if ((baseAddr == ((uint32_t)NULL))    ||
        ((intrMask & (~CRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL)) != ((uint32_t)NULL)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                intVal = (intrMask << CRC_INTS_CH1_CCITENS_SHIFT);
                break;
            case CRC_CHANNEL_2:
                intVal = (intrMask << CRC_INTS_CH2_CCITENS_SHIFT);
                break;
            case CRC_CHANNEL_3:
                intVal = (intrMask << CRC_INTS_CH3_CCITENS_SHIFT);
                break;
            case CRC_CHANNEL_4:
                intVal = (intrMask << CRC_INTS_CH4_CCITENS_SHIFT);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    if (status == CSL_PASS)
    {
        HW_WR_REG32(baseAddr + CRC_INTS, intVal);
    }

    return (status);
}

int32_t CRC_disableIntr(uint32_t     baseAddr,
                       CRC_Channel_t channel,
                       uint32_t     intrMask)
{
    int32_t  status = CSL_PASS;
    uint32_t intVal = 0U;

    if ((baseAddr == ((uint32_t)NULL))    ||
        ((intrMask & (~CRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL)) != ((uint32_t)NULL)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                intVal = (intrMask << CRC_INTR_CH1_CCITENR_SHIFT);
                break;
            case CRC_CHANNEL_2:
                intVal = (intrMask << CRC_INTR_CH2_CCITENR_SHIFT);
                break;
            case CRC_CHANNEL_3:
                intVal = (intrMask << CRC_INTR_CH3_CCITENR_SHIFT);
                break;
            case CRC_CHANNEL_4:
                intVal = (intrMask << CRC_INTR_CH4_CCITENR_SHIFT);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    if (status == CSL_PASS)
    {
        HW_WR_REG32(baseAddr + CRC_INTR, intVal);
    }

    return (status);
}

int32_t CRCClearIntr(uint32_t     baseAddr,
                     CRC_Channel_t channel,
                     uint32_t     intrMask)
{
    int32_t status = CSL_PASS;
    uint32_t intVal = 0U;

    if ((baseAddr == ((uint32_t)NULL))    ||
        ((intrMask & (~CRC_CHANNEL_IRQSTATUS_RAW_MAIN_ALL)) != ((uint32_t)NULL)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        switch (channel)
        {
            case CRC_CHANNEL_1:
                intVal = (intrMask << CRC_STATUS_CH1_CCIT_SHIFT);
                break;
            case CRC_CHANNEL_2:
                intVal = (intrMask << CRC_STATUS_CH2_CCIT_SHIFT);
                break;
            case CRC_CHANNEL_3:
                intVal = (intrMask << CRC_STATUS_CH3_CCIT_SHIFT);
                break;
            case CRC_CHANNEL_4:
                intVal = (intrMask << CRC_STATUS_CH4_CCIT_SHIFT);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    if (status == CSL_PASS)
    {
        HW_WR_REG32(baseAddr + CRC_STATUS, intVal);
    }

    return (status);
}

int32_t CRC_powerDownCtrl(uint32_t baseAddr,
                         uint32_t ctrlFlag)
{
    int32_t status;

    if ((baseAddr == ((uint32_t)NULL)) ||
        (ctrlFlag > CSL_TRUE))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        HW_WR_FIELD32(baseAddr + CRC_CTRL1,
                      CRC_CTRL1_PWDN,
                      ctrlFlag);
        status = CSL_PASS;
    }

    return (status);
}

int32_t CRC_isBusy(uint32_t      baseAddr,
                  CRC_Channel_t  channel,
                  uint32_t     *pBusyFlag)
{
    int32_t  status = CSL_PASS;
    uint32_t busyVal;

    if ((baseAddr == ((uint32_t)NULL)) ||
        (pBusyFlag == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        /* read busy status */
        switch (channel)
        {
            case CRC_CHANNEL_1:
                busyVal = HW_RD_FIELD32(baseAddr + CRC_BUSY,
                                        CRC_BUSY_CH1);

                break;
            case CRC_CHANNEL_2:
                busyVal = HW_RD_FIELD32(baseAddr + CRC_BUSY,
                                        CRC_BUSY_CH2);
                break;
            case CRC_CHANNEL_3:
                busyVal = HW_RD_FIELD32(baseAddr + CRC_BUSY,
                                        CRC_BUSY_CH3);
                break;
            case CRC_CHANNEL_4:
                busyVal = HW_RD_FIELD32(baseAddr + CRC_BUSY,
                                        CRC_BUSY_CH4);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    if (status == CSL_PASS)
    {
        *pBusyFlag = busyVal;
    }

    return (status);
}

int32_t CRCGetCurSecNum(uint32_t        baseAddr,
                        CRC_Channel_t    channel,
                        uint32_t       *pCurSecNum)
{
    int32_t  status = CSL_PASS;

    if ((baseAddr == ((uint32_t)NULL)) ||
        (pCurSecNum == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        /* read current sector number */
        switch (channel)
        {
            case CRC_CHANNEL_1:
                *pCurSecNum = HW_RD_FIELD32(baseAddr + CRC_CURSEC_REG1,
                                            CRC_CURSEC_REG1_CURSEC1);
                break;
            case CRC_CHANNEL_2:
                *pCurSecNum = HW_RD_FIELD32(baseAddr + CRC_CURSEC_REG2,
                                            CRC_CURSEC_REG2_CURSEC2);
                break;
            case CRC_CHANNEL_3:
                *pCurSecNum = HW_RD_FIELD32(baseAddr + CRC_CURSEC_REG3,
                                            CRC_CURSEC_REG3_CURSEC3);
                break;
            case CRC_CHANNEL_4:
                *pCurSecNum = HW_RD_FIELD32(baseAddr + CRC_CURSEC_REG4,
                                            CRC_CURSEC_REG4_CURSEC4);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_getCurPSASig(uint32_t        baseAddr,
                        CRC_Channel_t    channel,
                        CRC_Signature *pCurPSASig)
{
    int32_t  status = CSL_PASS;

    if ((baseAddr == ((uint32_t)NULL)) ||
        (pCurPSASig == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        /* read current PSA signature value */
        switch (channel)
        {
            case CRC_CHANNEL_1:
                pCurPSASig->regL = HW_RD_FIELD32(baseAddr + CRC_REGL1,
                                                 CRC_REGL1_CRC1);
                pCurPSASig->regH = HW_RD_FIELD32(baseAddr + CRC_REGH1,
                                                 CRC_REGH1_CRC1_47_32);
                break;
            case CRC_CHANNEL_2:
                pCurPSASig->regL = HW_RD_FIELD32(baseAddr + CRC_REGL2,
                                                 CRC_REGL2_CRC2);
                pCurPSASig->regH = HW_RD_FIELD32(baseAddr + CRC_REGH2,
                                                 CRC_REGH2_CRC2_63_32);
                break;
            case CRC_CHANNEL_3:
                pCurPSASig->regL = HW_RD_FIELD32(baseAddr + CRC_REGL3,
                                                 CRC_REGL3_CRC3);
                pCurPSASig->regH = HW_RD_FIELD32(baseAddr + CRC_REGH3,
                                                 CRC_REGH3_CRC3_63_32);
                break;
            case CRC_CHANNEL_4:
                pCurPSASig->regL = HW_RD_FIELD32(baseAddr + CRC_REGL4,
                                                 CRC_REGL4_CRC4);
                pCurPSASig->regH = HW_RD_FIELD32(baseAddr + CRC_REGH4,
                                                 CRC_REGH4_CRC4_63_32);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_getRawData(uint32_t        baseAddr,
                      CRC_Channel_t    channel,
                      CRC_Signature *pRawData)
{
    int32_t  status = CSL_PASS;

    if ((baseAddr == ((uint32_t)NULL)) ||
        (pRawData == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        /* read raw data value */
        switch (channel)
        {
            case CRC_CHANNEL_1:
                pRawData->regL = HW_RD_FIELD32(baseAddr + CRC_RAW_DATAREGL1,
                                               CRC_RAW_DATAREGL1_DATA1);
                pRawData->regH = HW_RD_FIELD32(baseAddr + CRC_RAW_DATAREGH1,
                                               CRC_RAW_DATAREGH1_DATA1_47_32);
                break;
            case CRC_CHANNEL_2:
                pRawData->regL = HW_RD_FIELD32(baseAddr + CRC_RAW_DATAREGL2,
                                               CRC_RAW_DATAREGL2_DATA2);
                pRawData->regH = HW_RD_FIELD32(baseAddr + CRC_RAW_DATAREGH2,
                                               CRC_RAW_DATAREGH2_DATA2_63_32);
                break;
            case CRC_CHANNEL_3:
                pRawData->regL = HW_RD_FIELD32(baseAddr + CRC_RAW_DATAREGL3,
                                               CRC_RAW_DATAREGL3_DATA3);
                pRawData->regH = HW_RD_FIELD32(baseAddr + CRC_RAW_DATAREGH3,
                                               CRC_RAW_DATAREGH3_DATA3_63_32);
                break;
            case CRC_CHANNEL_4:
                pRawData->regL = HW_RD_FIELD32(baseAddr + CRC_RAW_DATAREGL4,
                                               CRC_RAW_DATAREGL4_DATA4);
                pRawData->regH = HW_RD_FIELD32(baseAddr + CRC_RAW_DATAREGH4,
                                               CRC_RAW_DATAREGH4_DATA4_63_32);
                break;
            default:
                status = CSL_EBADARGS;
                break;
        }
    }

    return (status);
}

int32_t CRC_dataBusTracingCtrl(uint32_t         baseAddr,
                              uint32_t         ctrlFlag,
                              CRC_DataBusMask_t dataBusMask,
                              CRC_DataBusMask_t busEnableMask)
{
    int32_t  status = CSL_PASS;
    uint32_t regVal;

    if ((baseAddr == ((uint32_t)NULL))                                   ||
        (ctrlFlag > CSL_TRUE)                                            ||
        ((dataBusMask & (~CRC_DATA_BUS_MASK_ALL)) != ((uint32_t)NULL))   ||
        ((busEnableMask & (~CRC_DATA_BUS_MASK_ALL)) != ((uint32_t)NULL)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        /* Configure channel 1 data tracing */
        HW_WR_FIELD32(baseAddr + CRC_CTRL2, CRC_CTRL2_CH1_TRACEEN, ctrlFlag);

        /* Configure data bus tracing control */
        regVal = HW_RD_REG32(baseAddr + CRC_MCRC_BUS_SEL);
        regVal &= ~dataBusMask;
        regVal |= busEnableMask;
        HW_WR_REG32(baseAddr + CRC_MCRC_BUS_SEL, regVal);
    }

    return (status);
}

int32_t CRC_verifyBusTracingCtrl(uint32_t         baseAddr,
                                uint32_t         ctrlFlag,
                                CRC_DataBusMask_t dataBusMask,
                                CRC_DataBusMask_t busEnableMask)
{
    int32_t  status = CSL_PASS;
    uint32_t traceEnable;
    uint32_t regVal;

    if ((baseAddr == ((uint32_t)NULL))                                   ||
        (ctrlFlag > CSL_TRUE)                                            ||
        ((dataBusMask & (~CRC_DATA_BUS_MASK_ALL)) != ((uint32_t)NULL))   ||
        ((busEnableMask & (~CRC_DATA_BUS_MASK_ALL)) != ((uint32_t)NULL)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        /* Read channel 1 data tracing enable bit */
        traceEnable = HW_RD_FIELD32(baseAddr + CRC_CTRL2,
                                    CRC_CTRL2_CH1_TRACEEN);
        if (traceEnable != ctrlFlag)
        {
            status = CSL_EFAIL;
        }
    }

    if (status == CSL_PASS)
    {
        /* Read data bus tracing control enable bits */
        regVal = HW_RD_REG32(baseAddr + CRC_MCRC_BUS_SEL);

        if ((regVal & dataBusMask) !=  busEnableMask)
        {
            status = CSL_EFAIL;
        }
    }

    return (status);

}

int32_t CRCReadStaticRegs(uint32_t baseAddr, CRC_StaticRegs *pStaticRegs)
{
    int32_t  status = CSL_PASS;
    uint32_t i;

    if ((baseAddr == ((uint32_t)NULL)) ||
        (pStaticRegs == (NULL_PTR)))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        pStaticRegs->CTRL0   = HW_RD_REG32(baseAddr + CRC_CTRL0);
        pStaticRegs->CTRL1   = HW_RD_REG32(baseAddr + CRC_CTRL1);
        pStaticRegs->BUS_SEL = HW_RD_REG32(baseAddr + CRC_MCRC_BUS_SEL);

        for (i = ((uint32_t) (0U)); i < CRC_MAX_NUM_OF_CHANNELS; i++)
        {
            pStaticRegs->channelRegs[i].PCOUNT   = HW_RD_REG32(baseAddr + CRCGetRegsOffset(i) + CRC_PCOUNT_REG1);
            pStaticRegs->channelRegs[i].SCOUNT   = HW_RD_REG32(baseAddr + CRCGetRegsOffset(i) + CRC_SCOUNT_REG1);
            pStaticRegs->channelRegs[i].WDTOPLD  = HW_RD_REG32(baseAddr + CRCGetRegsOffset(i) + CRC_WDTOPLD1);
            pStaticRegs->channelRegs[i].BCTOPLD  = HW_RD_REG32(baseAddr + CRCGetRegsOffset(i) + CRC_BCTOPLD1);
        }
        status = CSL_PASS;
    }
    return status;
}
