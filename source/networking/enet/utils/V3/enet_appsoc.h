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
 * \file     enet_appsoc.h
 *
 * \brief    This file contains the CPSW low-level driver SOC specific
 *           function implementations for AM65xx devices.
 */

#ifndef ENETAPP_SOC_H_
#define ENETAPP_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <enet.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENET_APP_SOC_TSYNC_ROUTER_NUM_INPUT                     (48U)
#define ENET_APP_SOC_TSYNC_ROUTER_NUM_OUTPUT                    (40U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief  CPTS Clock Select Mux enum
 */
typedef enum
{
    ENETAPPUTILS_CPTS_CLKSEL_CPSWHSDIV_CLKOUT2  = 0x0U,
    /**< CPSW HSDIV Clockout2 */
    ENETAPPUTILS_CPTS_CLKSEL_MAINHSDIV_CLKOUT3  = 0x1U,
    /**< Main HSDIV Clockout3 */
    ENETAPPUTILS_CPTS_CLKSEL_MCU_CPTS0_RFT_CLK  = 0x2U,
    /**< MCU CPTS RFT Clock */
    ENETAPPUTILS_CPTS_CLKSEL_CPTS0_RFT_CLK      = 0x3U,
    /**< CPTS RFT Clock */
    ENETAPPUTILS_CPTS_CLKSEL_MCU_EXT_REFCLK0    = 0x4U,
    /**< MCU External Reference Clock */
    ENETAPPUTILS_CPTS_CLKSEL_EXT_REFCLK1        = 0x5U,
    /**< External Reference Clock */
    ENETAPPUTILS_CPTS_CLKSEL_PCIE0_TXI0_CLK     = 0x6U,
    /**< PCIE0 TX IO Clock */
    ENETAPPUTILS_CPTS_CLKSEL_PCIE1_TXI0_CLK     = 0x7U,
    /**< PCIE1 TX IO Clock */
} EnetAppUtils_CptsClkSelMux;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENETAPP_SOC_H_ */
