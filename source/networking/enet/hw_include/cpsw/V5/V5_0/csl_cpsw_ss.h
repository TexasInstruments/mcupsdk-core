/**
 * @file  csl_cpsw_ss.h
 *
 * @brief
 *  API Auxilary header file for Ethernet switch subsystem CSL.
 *
 *  Contains the different control command and status query functions definations
 *
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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

#ifndef CSL_CPSW_SS_V5_0_H_
#define CSL_CPSW_SS_V5_0_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <drivers/hw_include/cslr_soc.h>

#include <cslr_xge_cpsw_ss_s.h>

/** @addtogroup CSL_CPSW_SS_S_DATASTRUCT
 @{ */

/** @brief
 *
 *  SGMII Mode enumerators.
 */
typedef enum
{
    /** Fiber mode */
   CSL_SGMII_MODE_FIBER    =   0,

    /** SGMII mode */
   CSL_SGMII_MODE_SGMII    =   1
} CSL_SGMII_MODE;

/** @brief
 *
 *  Holds the Ethernet switch subsystem's version info.
 */
typedef struct {
    /**  Minor version value */
    Uint32      minorVer;

    /**  Major version value */
    Uint32      majorVer;

    /**  RTL version value */
    Uint32      rtlVer;

    /**  Identification value */
    Uint32      id;
} CSL_CPSW_SS_VERSION;

/** @brief
 *
 *  Holds the CPSW_SS INTD version info.
 */
typedef struct {
    /** Scheme value */
    Uint32      scheme;
    /** BU value. Read as zero. Write have no effect */
    Uint32      bu;
    /** Function value */
    Uint32      function;
    /** RTL version for instantiated IP */
    Uint32      rtlVer;
    /** Major revision */
    Uint32      majorRevision;
    /** Custom value */
    Uint32      custom;
    /** Minor revision */
    Uint32      minorRevision;
} CSL_CPSW_SS_INTD_VERSION;

/** @brief
 *
 *  Holds the CPSW_SS INTD interrupt status info
 */
typedef struct {
    /** ECC event active */
    Uint32      cpswEventActive;
    /** mdio interrupt active */
    Uint32      mdioInterruptActive;
    /** stat interrupt active */
    Uint32      statPendInterruptActive;
} CSL_CPSW_SS_INTD_INTRSTATUS;

/** @brief
 *
 *  Holds the CPSW_SS RGMII STATUS
 */
typedef struct {
    /** RGMII link status */
    Uint32      link;
    /** RGMII speed */
    Uint32      speed;
    /** RGMII full duplex */
    Uint32      fullDuplex;
} CSL_CPSW_SS_RGMIISTATUS;
/**
@}
*/

/** @addtogroup CSL_CPSW_SS_S_FUNCTION
@{ */

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the ethernet switch subsystem version
 *      information.
 *
 *   @b Arguments
     @verbatim
        versionInfo         CSL_CPSW_SS_VERSION structure that needs to be populated
                            with the version info read from the hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_SS_S_IDVER_REG_MINOR_VER,
 *      XGE_CPSW_SS_S_IDVER_REG_MAJOR_VER,
 *      XGE_CPSW_SS_S_IDVER_REG_RTL_VER,
 *      XGE_CPSW_SS_S_IDVER_REG_IDENT
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_SS_VERSION    versionInfo;

        CSL_CPSW_SS_getVersionInfo (&versionInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_SS_getVersionInfo (CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
    CSL_CPSW_SS_VERSION* versionInfo
);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getIntdVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the CPSW_SS INTD
 *      information.
 *
 *   @b Arguments
     @verbatim
        versionInfo         CSL_CPSW_SS_INTD_VERSION structure that needs to be
                            populated with the version info read from the hardware.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 * =============================================================================
 */
void  CSL_CPSW_SS_INTD_getVersionInfo(CSL_cpswRegs_INTD * hCpswIntDRegs,
                                      CSL_CPSW_SS_INTD_VERSION* versionInfo);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_getEOIVector
 *
 *   @b Description
 *   @n This function retrieves the CPSW_SS INTD EOI_VECTOR from EOI_REG
 *   The EOI Register allows the software to perform an end of interrupt
 *   handshake to interrupts with an IP that does not have an EOI register.
 *   The EOI handshake allows system interrupts to be re-evaluated for the
 *   associate logic with the EOI vector value (to allow multiple logic paths
 *   to re-evaluate individually without separate EOI registers). The EOI
 *   occurs the cycle after the register is written
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  eoiVector
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_INTD_getEOIVector(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_setEOIVector
 *
 *   @b Description
 *   @n This function set the CPSW_SS INTD EOI_VECTOR in EOI_REG
 *   The EOI Register allows the software to perform an end of interrupt
 *   handshake to interrupts with an IP that does not have an EOI register.
 *   The EOI handshake allows system interrupts to be re-evaluated for the
 *   associate logic with the EOI vector value (to allow multiple logic paths
 *   to re-evaluate individually without separate EOI registers). The EOI
 *   occurs the cycle after the register is written
 *
 *   @b Arguments
     @verbatim
        eoiVector EOI vector to be set
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_INTD_setEOIVector(CSL_cpswRegs_INTD * hCpswIntDRegs,
                                   Uint32 eoiVector);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_getEOIIntrVector
 *
 *   @b Description
 *   @n This function retrieves the CPSW_SS INTD INTR_VECTOR from INTR_VECTOR_REG
 *
 *   The EOI Interrupt Vector Register captures the active and prioritized interrupts
 *   so that software may quickly read the values rather than check every bit in the IP
 *   Interrupt Source Register. The exact definition of this register is
 *   configuration dependent
 @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  intrVector
 *      EOI Interrupt Vector value. This has the latest prioritized interrupt values.
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_INTD_getEOIIntrVector(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_getIntrVectorOutPulse
 *
 *   @b Description
 *   @n This function get the CPSW_SS INTD INTR_VECTOR_REG_OUT_PULSE
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n
        intrVectorOutPulse field from INTR_VECTOR_REG_OUT_PULSE
        The Interrupt Vector Registers captures the interrupt vector of active
        and prioritized interrupts for each host based in that hosts' enabled
        interrupts
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_INTD_getIntrVectorOutPulse(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_setEnableIntrCPSWEventInterrupt
 *
 *   @b Description
 *   @n This function enables/disables the CPSW event interrupt
 *
 *   @b Arguments
     @verbatim
        enable Flag to enable/disable
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_INTD_setEnableIntrCPSWEventInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs,
                                                     Uint32              enable);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_getEnableIntrCPSWEventInterrupt
 *
 *   @b Description
 *   @n This function returns flag indicating if the CPSW event interrupt is enabled
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Flag indicating if the interrupt is enabled or not
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_INTD_getEnableIntrCPSWEventInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_setClearIntrCPSWEventInterrupt
 *
 *   @b Description
 *   @n This function clears the CPSW EVENT Interrupt status by writing to the
 *      ENABLE_CLR_REG_OUT_PULSE_0 register
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_INTD_setClearIntrCPSWEventInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_getStatusIntr
 *
 *   @b Description
 *   @n This function returns CSL_CPSW_SS_INTD_INTRSTATUS populated with
 *      status of various CPSW interrupts
 *
 *   @b Arguments
     @verbatim
        intrStatus Interrupt status to be populated
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_INTD_getStatusIntr(CSL_cpswRegs_INTD * hCpswIntDRegs,
                                    CSL_CPSW_SS_INTD_INTRSTATUS *intrStatus);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_setEnableIntrMdioInterrupt
 *
 *   @b Description
 *   @n This function enables/disables the MDIO interrupt
 *
 *   @b Arguments
     @verbatim
        enable Flag to enable/disable
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_INTD_setEnableIntrMdioInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs,
                                                 Uint32              enable);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_getEnableIntrMdioInterrupt
 *
 *   @b Description
 *   @n This function returns flag indicating if the MDIO interrupt is enabled
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Flag indicating if the interrupt is enabled or not
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_INTD_getEnableIntrMdioInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_setClearIntrMdioInterrupt
 *
 *   @b Description
 *   @n This function clears the CPSW MDIO Interrupt status by writing to the
 *      ENABLE_CLR_REG_OUT_PULSE_0 register
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_INTD_setClearIntrMdioInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_setEnableIntrStatInterrupt
 *
 *   @b Description
 *   @n This function enables/disables the STAT interrupt
 *
 *   @b Arguments
     @verbatim
        enable Flag to enable/disable
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_INTD_setEnableIntrStatInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs,
                                                 Uint32              enable);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_getEnableIntrStatInterrupt
 *
 *   @b Description
 *   @n This function returns flag indicating if the STAT interrupt is enabled
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Flag indicating if the interrupt is enabled or not
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_INTD_getEnableIntrStatInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_INTD_setClearIntrStatInterrupt
 *
 *   @b Description
 *   @n This function clears the CPSW STAT Interrupt status by writing to the
 *      ENABLE_CLR_REG_OUT_PULSE_0 register
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_INTD_setClearIntrStatInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_setSGMIIMode
 *
 *   @b Description
 *   @n This function sets the SGMII mode (fiber or copper)  for the given macPort
 *
 *   @b Arguments
     @verbatim
        macPortNum Mac Port for which SGMII mode is set
        sgmiiModeId CSL_SGMII_MODE value
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_setSGMIIMode(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                              Uint32 macPortNum,
                              CSL_SGMII_MODE sgmiiModeId);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getRGMIIStatus
 *
 *   @b Description
 *   @n This function returns the RGMII status for the given macPort
 *
 *   @b Arguments
     @verbatim
        macPortNum Mac Port for which RGMII status is queried
        pRgmiiStatus RGMII status to be populated
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_getRGMIIStatus(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                Uint32 macPortNum,
                                CSL_CPSW_SS_RGMIISTATUS *pRgmiiStatus);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getSerdesResetIsolation
 *
 *   @b Description
 *   @n This function returns the Serdes reset isolation configuration
 *
 *   @b Arguments
     @verbatim
        none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Serdes Reset isolation value
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_getSerdesResetIsolation(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_setSerdesResetIsolation
 *
 *   @b Description
 *   @n This function sets the serdes reset isolation configuration
 *
 *   @b Arguments
     @verbatim
        resetIso Reset isolation configuration to be set
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_SS_setSerdesResetIsolation(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                         Uint32 resetIso);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getQSGMIIControlRdCd
 *
 *   @b Description
 *   @n This function returns the QSGMII control register for the given qsgmii
 *
 *   @b Arguments
     @verbatim
        qsgmiiId  QSGMII (0/1) for which the QSGMII control register is queried
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  QSGMII control configuration for the given QSGMII id
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_getQSGMIIControlRdCd(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                        Uint32 qsgmiiId);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_setQSGMIIControlRdCd
 *
 *   @b Description
 *   @n This function sets the QSGMII control register for the given qsgmii
 *
 *   @b Arguments
     @verbatim
        qsgmiiId  QSGMII (0/1) for which the QSGMII control register is set
        qsgmiiControlRdCd QSMII control configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  QSGMII control configuration for the given QSGMII id
 *
 * =============================================================================
 */
void CSL_CPSW_SS_setQSGMIIControlRdCd(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                      Uint32 qsgmiiId,
                                      Uint32 qsgmiiControlRdCd);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getQSGMIIStatusRxSync
 *
 *   @b Description
 *   @n This function returns the QSGMII status register for the given qsgmii
 *
 *   @b Arguments
     @verbatim
        qsgmiiId  QSGMII (0/1) for which the QSGMII status register is queried
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  QSGMII status for the given QSGMII id
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_getQSGMIIStatusRxSync(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                       Uint32 qsgmiiId);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getXGMIILinkStatus
 *
 *   @b Description
 *   @n This function returns the XGMII link status register for the given xgmii
 *
 *   @b Arguments
     @verbatim
        xgmiiid  XGMII (0/1) for which the XGMII status register is queried
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n XGMII status for the given XGMII id
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_getXGMIILinkStatus(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                      Uint32 xgmiiId);

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getSGMIILinkStatus
 *
 *   @b Description
 *   @n This function returns the SGMII link status register for the given sgmii
 *
 *   @b Arguments
     @verbatim
        qsgmiiId  QSGMII (0/1) for which the QSGMII status register is queried
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  SGMII link status for the given SGMII id
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_getSGMIILinkStatus(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                      Uint32 sgmiiId);


















#ifdef __cplusplus
}
#endif

#endif

/**
@}
*/
