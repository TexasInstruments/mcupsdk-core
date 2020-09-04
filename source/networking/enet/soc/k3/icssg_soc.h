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
 * \file  icssg_soc.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        ICSSG peripheral SoC interface in Jacinto 7 devices.
 */
/*!
 * \ingroup  ENET_MOD_SOC
 * \defgroup ENET_ICSSG_SOC Enet ICSSG SOC APIs and data structures
 *
 * @{
 */
#ifndef ICSSG_SOC_H_
#define ICSSG_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* Number of ICSSG interrupts: MDIO */
#define ICSSG_INTR_NUM                               (1U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief ICSSG SoC configuration.
 *
 * SoC-level configuration information for the ICSSG driver.
 */
typedef struct IcssgSoc_Cfg_s
{
    /*! SCI-client module ID for ICSSG */
    uint16_t dmscDevId;

    /*! ICSSG interrupts */
    EnetSoc_IntrConnCfg intrs[ICSSG_INTR_NUM];

    /*! Tx Ch Peer Thread Id */
    uint32_t txChPeerThreadId;

    /*! Rx Ch Peer Thread Id */
    uint32_t rxChPeerThreadId;

    /*! Tx Ch Count*/
    uint32_t txChCount;

    /*! Rx Flow Count */
    uint32_t rxFlowCount;

    /*! CPTS Hardware Push Event Count */
    uint32_t cptsHwPushCount;
} IcssgSoc_Cfg;

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

#endif /* ICSSG_SOC_H_ */

/*! @} */
