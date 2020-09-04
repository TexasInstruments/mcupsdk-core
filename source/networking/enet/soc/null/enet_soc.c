/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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

/*!
 * \file  soc_null.c
 *
 * \brief This file contains the implementation of "null" SoC.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>
#include <include/core/enet_base.h>
#include <include/core/enet_soc.h>
#include <include/per/per_null.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*! \brief Number of Ethernet Peripherals in null SoC. */
#define ENET_NULL_PER_NUM                    (1U)

/*! \brief Frequency of dummy peripheral's MCLK. */
#define ENET_NULL_PER_MCLK_FREQ              (100000000U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

NullPer_Obj gEnetSoc_NullPerObj =
{
    .enetPer =
    {
        .name         = "NullPer",
        .features     = (ENET_NULLPER_FEAT1 | ENET_NULLPER_FEAT2),
        .errata       = (ENET_NULLPER_ERRATA1),
        .initCfg      = &NullPer_initCfg,
        .open         = &NullPer_open,
        .rejoin       = &NullPer_rejoin,
        .ioctl        = &NullPer_ioctl,
        .poll         = &NullPer_poll,
        .periodicTick = &NullPer_periodicTick,
        .registerEventCb = NULL,
        .unregisterEventCb = NULL,
        .close        = &NullPer_close,
    },
    .mod1 =
    {
        .enetMod =
        {
            .name       = "NullPer.mod1",
            .features   = (ENET_NULLMOD_FEAT1),
            .errata     = (ENET_NULLMOD_ERRATA1),
            .open       = &NullMod_open,
            .rejoin     = &NullMod_rejoin,
            .ioctl      = &NullMod_ioctl,
            .close      = &NullMod_close,
        },
    },
    .mod2 =
    {
        .enetMod =
        {
            .name       = "NullPer.mod2",
            .features   = (ENET_FEAT_BASE),
            .errata     = (ENET_NULLMOD_ERRATA1 | ENET_NULLMOD_ERRATA2),
            .open       = &NullMod_open,
            .rejoin     = &NullMod_rejoin,
            .ioctl      = &NullMod_ioctl,
            .close      = &NullMod_close,
        },
    },
};

Enet_Obj gEnetSoc_NullEnetObj[ENET_NULL_PER_NUM] =
{
    {
        .enetPer = &gEnetSoc_NullPerObj.enetPer,
    },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetSoc_init(void)
{
    /* Do any dynamic initialization */

    return ENET_SOK;
}

void EnetSoc_deinit(void)
{
}

Enet_Handle EnetSoc_getEnetHandleByIdx(uint32_t idx)
{
    Enet_Handle hEnet = NULL;

    if (idx < ENET_ARRAYSIZE(gEnetSoc_NullEnetObj))
    {
        hEnet = &gEnetSoc_NullEnetObj[0];
    }

    return hEnet;
}

Enet_Handle EnetSoc_getEnetHandle(Enet_Type enetType,
                                  uint32_t instId)
{
    Enet_Handle hEnet = NULL;

    switch (enetType)
    {
        case ENET_NULL:
            if (instId == 0U)
            {
                hEnet = &gEnetSoc_NullEnetObj[0];
            }
            break;

        default:
            break;
    }

    return hEnet;
}

uint32_t EnetSoc_getCoreId(void)
{
    uint32_t coreId;

#if defined(BUILD_MCU1_0)
    coreId = IPC_MCU1_0;
#else
#error "Unsupported core id"
#endif

    return coreId;
}

uint32_t EnetSoc_getCoreKey(uint32_t coreId)
{
    return coreId;
}

bool EnetSoc_isCoreAllowed(Enet_Type enetType,
                           uint32_t instId,
                           uint32_t coreId)
{
    /* All cores are allowed to use the dummy peripherals */
    return true;
}

uint32_t EnetSoc_getEnetNum(void)
{
    return ENET_ARRAYSIZE(gEnetSoc_NullEnetObj);
}

uint32_t EnetSoc_getMacPortMax(Enet_Type enetType,
                               uint32_t instId)
{
    /* Single MAC port */
    return 1U;
}

uint32_t EnetSoc_getClkFreq(Enet_Type enetType,
                            uint32_t instId,
                            uint32_t clkId)
{
    uint32_t freq = 0U;

    if (clkId == ENET_NULLPER_CLK_MCLK)
    {
        freq = ENET_NULL_PER_MCLK_FREQ;
    }

    return freq;
}

int32_t EnetSoc_getEFusedMacAddrs(uint8_t macAddr[][ENET_MAC_ADDR_LEN],
                                  uint32_t *num)
{
    /* No EFused MAC addresses */
    *num = 0U;

    return ENET_SOK;
}
