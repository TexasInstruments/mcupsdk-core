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
 * \file  cpsw_soc.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        CPSW peripheral SoC interface in Jacinto 7 devices.
 */

/*!
 * \ingroup  ENET_MOD_SOC
 * \defgroup ENET_CPSW_SOC  Enet CPSW SOC APIs and data structures
 *
 * @{
 */

#ifndef CPSW_SOC_H_
#define CPSW_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "k3_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Number of CPSW interrupts: event (CPTS), MDIO and stats. */
#define CPSW_INTR_NUM                                (3U)

/*! \brief SYSFW SRC index values for CPSW event (CPTS) interrupt. */
#define CPSW_INTR_EVENT_PEND_SRCIDX                  (4U)

/*! \brief SYSFW SRC index values for CPSW MDIO interrupt. */
#define CPSW_INTR_MDIO_INTR_SRCIDX                   (5U)

/*! \brief SYSFW SRC index values for CPSW statistics interrupt. */
#define CPSW_INTR_STAT_PEND0_SRCIDX                  (6U)

/*! \brief RMII interface type (mode selection in ENET_CTRL register). */
#define CPSW_ENET_CTRL_MODE_RMII                     (1U)

/*! \brief RGMII interface type (mode selection in ENET_CTRL register). */
#define CPSW_ENET_CTRL_MODE_RGMII                    (2U)

/*! \brief SGMII interface type (mode selection in ENET_CTRL register). */
#define CPSW_ENET_CTRL_MODE_SGMII                    (3U)

/*! \brief QSGMII interface type (mode selection in ENET_CTRL register). */
#define CPSW_ENET_CTRL_MODE_QSGMII                   (4U)

/*! \brief QSGMII_SUB interface type (mode selection in ENET_CTRL register). */
#define CPSW_ENET_CTRL_MODE_QSGMII_SUB               (6U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief CPSW clocks config structure
 *
 * TI sci-client Clock ID information for CPSW clocks. This would be used to
 * configure interrupts using System firmware via sci-client.
 */
typedef struct CpswSoc_Clock_s
{
    /*! CPSW CPPI clock id */
    uint32_t cppiClkId;

    /*! CPSW RGMII 250MHz clock id */
    uint32_t rgmii250MHzClkId;

    /*! CPSW RGMII 50MHz clock id */
    uint32_t rgmii50MHzClkId;

    /*! CPSW RGMII 5MHz clock id */
    uint32_t rgmii5MHzClkId;
} CpswSoc_Clock;

/*!
 * \brief CPSW SoC configuration.
 *
 * SoC-level configuration information for the CPSW driver.
 */
typedef struct CpswSoc_Cfg_s
{
    /*! CPSW main clock (CPPI_ICLK) frequency in Hz. CPPI packet streaming
     * interface clock
     * Note: Clock frequency variable needs to be uint64_t as PMLIB API takes
     * clkRate input as uint64_t */
    uint64_t cppiClkFreqHz;

    /*! SCI-Client module ID for CPSW */
    uint16_t dmscDevId;

    /*! CPSW interrupts */
    EnetSoc_IntrConnCfg intrs[CPSW_INTR_NUM];

    /*! CPSW clocks */
    CpswSoc_Clock clocks;

    /*! Tx Ch Peer Thread Id */
    uint32_t txChPeerThreadId;

    /*! Rx Ch Peer Thread Id */
    uint32_t rxChPeerThreadId;

    /*! Tx Ch Count*/
    uint32_t txChCount;

    /*! Rx Flow Count */
    uint32_t rxFlowCount;
} CpswSoc_Cfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* CPSW_SOC_H_ */

/*! @} */
