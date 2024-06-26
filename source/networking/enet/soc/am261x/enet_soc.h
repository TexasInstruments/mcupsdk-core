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
 * \file  enet_soc.h
 *
 * \brief This file contains AM261X SoC specific definition.
 */

#ifndef ENET_SOC_AM261X_H_
#define ENET_SOC_AM261X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/dma/cpdma/enet_cpdma_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*!
 * \brief Host port DMA type for TPR family of devices.
 */
#define ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 *  \anchor CPSW_SocCfg
 *  \name CPSW SOC Configuration
 *
 *  CPSW Soc Cfg - Flags to indicate the availability of various SOC specific modules.
 *
 *  @note: Some of the flags are defined to remove features only to maintain backward
 *         compatibility
 *
 *  @{
 */
/** \brief Indicate the SOC does not support MAC serial sublayer */
#define CPSW_SOC_CFG_MAC_SUBLAYER_SERIAL_NOT_PRESENT

/** \brief Indicate the Interrupt wrapper is present */
#define CPSW_SOC_CFG_INT_WR_PRESENT

/*! @} */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/*!
 * \brief Get handle of the DMA driver for a given Ethernet peripheral.
 *
 * Gets the handle to the DMA driver corresponding to the Ethernet peripheral
 * identified by its type and instance id.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Instance Id
 *
 * \return DMA driver handle. NULL if no driver was found.
 */
EnetDma_Handle EnetSoc_getDmaHandle(Enet_Type enetType,
                                    uint32_t instId);

/*!
 * \brief Get number of RX channels.
 *
 * Gets the total number of RX flow count supported by IP.  For AM261X devices,
 * this actually refers to the number of RX channels.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 *
 * \return Number of RX channels.
 */
uint32_t EnetSoc_getRxFlowCount(Enet_Type enetType,
                               uint32_t instId);

/*!
 * \brief Get CPSW TX channel peer ID.
 *
 * This SoC operation is not supported by AM261X devices.
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param chNum     CPSW TX channel number for which peer ID is returned
 *
 * \return 0 which indicates that this operation is not supported.
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
 * \return 0 which indicates that this operation is not supported.
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


/*!
 * \brief Get cppi desc info.
 *
 * Gets baseaddress and size of CPPI descriptor
 *
 * \param enetType  Enet Peripheral type
 * \param instId    Enet Peripheral instance id
 * \param descStartAddr Will be populated with descriptor start address
 * \param size Will be populated with total descriptor memory size
 *
 * \return Number of TX channels
 */
void EnetSoc_getCppiDescInfo(Enet_Type enetType,
                             uint32_t instId,
                             uintptr_t *descStartAddr,
                             uint32_t *size);

/*!
 *  \brief EnetSoc_mapApp2CpdmaCoreId
 *
 *  Returns the coreId expected by CPDMA module.
 *
 *  \param appCoreId    Application passed coreId
 *
 *  return coreId
 */
uint32_t EnetSoC_mapApp2CpdmaCoreId(uint32_t appCoreId);

/*!
 *  \brief EnetSoC_toggleCPSWResetBit
 *
 *  Hard reset CPSW peripheral.
 */
void EnetSoC_toggleCPSWResetBit(void *pArg);

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

#endif /* ENET_SOC_AM261X_H_ */
