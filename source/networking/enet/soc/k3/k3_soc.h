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
 * \file  k3_soc.h
 *
 * \brief This file contains the type definitions and helper macros that are
 *        common in Keystone 3 devices.
 */
/*!
 * \ingroup  DRV_ENET_MODULE
 * \defgroup ENET_MOD_SOC Enet SOC APIs and data structures
 *
 * @{
 */

#ifndef K3_SOC_H_
#define K3_SOC_H_

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

/*!
 * \brief Source index invalid macro, used for direct interrupts.
 *
 * This source index invalid macro is used to indicate in #EnetSoc_IntrConnCfg
 * that an interrupt is not coming through interrupt router.  When it does
 * come via interrupt router, the #EnetSoc_IntrConnCfg.srcIdx will be set to
 * a valid index.
 *
 * Enet peripheral drivers will use this macro to check whether additional
 * configuration is required for the given interrupt (i.e. via SYSFW in
 * Jacinto 7 devices).
 */
#define ENET_SOC_DIRECT_INTR_SRCIDX_INVALID          (0xABCDU)

/*!
 * \brief Host port DMA type for K3 family of devices.
 */
#define ENET_SOC_HOSTPORT_DMA_TYPE_UDMA

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief Interrupt "SoC connection" configuration.
 *
 * In SoC, for any given core two types of interrupts exists, direct interrupts
 * and interrupts routed through the IR (interrupt router).
 *
 * For direct interrupts as name suggests, there is direct connection between
 * peripheral (i.e. CPSW) and the core.  As connection is direct, no additional
 * configuration is required (i.e. via SYSFW in Jacinto 7 devices).

 * The indirect interrupts are coming from an interrupt router (IR). So, the
 * peripheral's interrupt output connects to IR on one end and the core interrupt
 * input is connected to other end. Here the "any" of core interrupt connected
 * to IR can be selected to get peripheral interrupt. So, IR output number needs
 * to be "resource managed" by the application to avoid conflicts. Indirect
 * interrupts required additional configuration (i.e. via SYSFW to make the
 * connection as IR is shared resource at SoC level).
 */
typedef struct EnetSoc_IntrConnCfg_s
{
    /*! Id of the interrupt as defined in the Enet Per driver */
    uint32_t intrId;

    /*! Core Interrupt number in case of direct connection or interrupt router
     *  index for indirect interrupt */
    uint16_t coreIntNum;

    /*! Source index for the peripheral interrupt. Only used if interrupt is
     *  indirect via router. If interrupt is direct set this field should be
     *  set to #ENET_SOC_DIRECT_INTR_SRCIDX_INVALID. */
    uint16_t srcIdx;
} EnetSoc_IntrConnCfg;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief SoC-level setup for an interrupt.
 *
 * Perform any required SoC setup in order to route an interrupt from a
 * peripheral to a processing core within the SoC.
 *
 * \param intrId       Peripheral interrupt id (refer to #EnetSoc_IntrConnCfg.intrId)
 * \param coreDevId    SCI client device id of the processing core
 * \param perDevId     SCI client device id of the Ethernet peripheral
 * \param intrs     Array with SoC interrupt information (refer to #EnetSoc_IntrConnCfg)
 * \param numSocIntrs  Size of #intrs array
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetSocJ7x_setupIntrCfg(uint32_t intrId,
                                uint16_t coreDevId,
                                uint16_t perDevId,
                                const EnetSoc_IntrConnCfg *intrs,
                                uint32_t numSocIntrs);

/*!
 * \brief SoC-level release for an interrupt.
 *
 * Releases any required SoC setup previously done to route an interrupt.
 *
 * \param intrId       Peripheral interrupt id (refer to #EnetSoc_IntrConnCfg.intrId)
 * \param coreDevId    SCI client device id of the processing core
 * \param perDevId     SCI client device id of the Ethernet peripheral
 * \param intrs     Array with SoC interrupt information (refer to #EnetSoc_IntrConnCfg)
 * \param numSocIntrs  Size of #intrs array
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetSocJ7x_releaseIntrCfg(uint32_t intrId,
                                  uint16_t coreDevId,
                                  uint16_t perDevId,
                                  const EnetSoc_IntrConnCfg *intrs,
                                  uint32_t numSocIntrs);

/*!
 * \brief Get interrupt number for a given peripheral event.
 *
 * Gets the interrupt number that can be used in the processing core to register
 * an ISR for.
 *
 * \param intrId       Peripheral interrupt id (refer to #EnetSoc_IntrConnCfg.intrId)
 * \param intrs     Array with SoC interrupt information (refer to #EnetSoc_IntrConnCfg)
 * \param numSocIntrs  Size of #intrs array
 *
 * \return Interrupt number.
 */
uint32_t EnetSocJ7x_getIntrNum(uint32_t intrId,
                               const EnetSoc_IntrConnCfg *intrs,
                               uint32_t numSocIntrs);


/*!
 * \brief Validate QSGMII config
 *
 * Validate control module port mode config for QSGMII main and sub ports.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetSoc_validateQsgmiiCfg(Enet_Type enetType,
                                  uint32_t instId);

/*!
 * \brief Map #Enet_MacPort to QSGMII Id
 *
 * Maps Enet mac port to QSGMII id needed for accesing QSGMII registers in CPSW SS.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param macPort   MAC port number
 * \param qsgmiiId  Pointer to variable for getting QSGMII id pointer
 *
 * \return \ref Enet_ErrorCodes
 */
int32_t EnetSoc_mapPort2QsgmiiId(Enet_Type enetType,
                                 uint32_t instId,
                                 Enet_MacPort macPort,
                                 uint32_t *qsgmiiId);
/*!
 * \brief Get number of RX flows.
 *
 * Gets the total number of RX flow count supported by IP.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return Number of RX flows.
 */
uint32_t EnetSoc_getRxFlowCount(Enet_Type enetType,
                               uint32_t instId);

/*!
 * \brief Get CPSW TX channel peer ID.
 *
 * Gets a peer ID (PSIL thread ID for UDMA) of given CPSW TX channel
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param chNum     CPSW TX channel number for which peer ID is returned
 *
 * \return Peer ID for given TX channel or 0 if not found.
 */
uint32_t EnetSoc_getTxChPeerId(Enet_Type enetType,
                               uint32_t instId,
                               uint32_t chNum);

/*!
 * \brief Get CPSW RX channel peer ID.
 *
 * Gets a peer ID (PSIL thread ID for UDMA) of given CPSW RX channel
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param chIdx     0-relative channel index. Use 0 if single channel.
 *
 * \return Peer ID for given RX channel or 0 if not found.
 */
uint32_t EnetSoc_getRxChPeerId(Enet_Type enetType,
                               uint32_t instId,
                               uint32_t chIdx);

/*!
 * \brief Get number of TX channels.
 *
 * Gets the total number of TX channels count supported by IP.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return Number of TX channels
 */
uint32_t EnetSoc_getTxChCount(Enet_Type enetType,
                               uint32_t instId);

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

#endif /* K3_SOC_H_ */

/*! @} */
