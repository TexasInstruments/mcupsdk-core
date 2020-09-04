/**
 * @file  csl_cpsw_ss.c
 *
 * @brief
 *  API Functional layer file for Ethernet switch subsystem CSL.
 *
 *  Contains the different control command and status query functions definations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2019, Texas Instruments, Inc.
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
 * \brief This is to disable HW_SYNC_BARRIER for J7 due to performance
 *        requirement
 */
#if defined (SOC_J721E) || defined (SOC_J7200)
#define MEM_BARRIER_DISABLE
#endif

#include <stdbool.h>
#include <csl_cpswitch.h>


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
)
{
    versionInfo->minorVer   =   CSL_FEXT (hCpswSsRegs->IDVER_REG, XGE_CPSW_SS_S_IDVER_REG_MINOR_VER);
    versionInfo->majorVer   =   CSL_FEXT (hCpswSsRegs->IDVER_REG, XGE_CPSW_SS_S_IDVER_REG_MAJOR_VER);
    versionInfo->rtlVer     =   CSL_FEXT (hCpswSsRegs->IDVER_REG, XGE_CPSW_SS_S_IDVER_REG_RTL_VER);
    versionInfo->id         =   CSL_FEXT (hCpswSsRegs->IDVER_REG, XGE_CPSW_SS_S_IDVER_REG_IDENT);

    return;
}

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
                                      CSL_CPSW_SS_INTD_VERSION* versionInfo)
{
    versionInfo->scheme = CSL_FEXT(hCpswIntDRegs->REVISION,
                                   XGE_CPSW_SS_S_INTD_REVISION_SCHEME);
    versionInfo->bu = CSL_FEXT(hCpswIntDRegs->REVISION,
                               XGE_CPSW_SS_S_INTD_REVISION_BU);
    versionInfo->function = CSL_FEXT(hCpswIntDRegs->REVISION,
                                     XGE_CPSW_SS_S_INTD_REVISION_FUNCTION);
    versionInfo->rtlVer = CSL_FEXT(hCpswIntDRegs->REVISION,
                                   XGE_CPSW_SS_S_INTD_REVISION_RTLVER);
    versionInfo->majorRevision = CSL_FEXT(hCpswIntDRegs->REVISION,
                                          XGE_CPSW_SS_S_INTD_REVISION_MAJREV);
    versionInfo->custom = CSL_FEXT(hCpswIntDRegs->REVISION,
                                   XGE_CPSW_SS_S_INTD_REVISION_CUSTOM);
    versionInfo->minorRevision = CSL_FEXT(hCpswIntDRegs->REVISION,
                                          XGE_CPSW_SS_S_INTD_REVISION_MINREV);
}

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
Uint32 CSL_CPSW_SS_INTD_getEOIVector(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    return (CSL_FEXT(hCpswIntDRegs->EOI_REG,
                     XGE_CPSW_SS_S_INTD_EOI_REG_EOI_VECTOR));
}

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
                                   Uint32 eoiVector)
{
    CSL_FINS(hCpswIntDRegs->EOI_REG,
             XGE_CPSW_SS_S_INTD_EOI_REG_EOI_VECTOR,
             eoiVector);
}

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
Uint32 CSL_CPSW_SS_INTD_getEOIIntrVector(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    return (CSL_FEXT(hCpswIntDRegs->INTR_VECTOR_REG,
                     XGE_CPSW_SS_S_INTD_INTR_VECTOR_REG_INTR_VECTOR));
}



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
Uint32 CSL_CPSW_SS_INTD_getIntrVectorOutPulse(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    return (CSL_FEXT(hCpswIntDRegs->INTR_VECTOR_REG_OUT_PULSE,
           XGE_CPSW_SS_S_INTD_INTR_VECTOR_REG_OUT_PULSE_INTR_VECTOR_OUT_PULSE));
}

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
                                                     Uint32              enable)
{
    CSL_FINS(hCpswIntDRegs->ENABLE_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_EVNT_PENDA,
             enable);
}

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
Uint32 CSL_CPSW_SS_INTD_getEnableIntrCPSWEventInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    return (CSL_FEXT(hCpswIntDRegs->ENABLE_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_EVNT_PENDA));
}

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
void CSL_CPSW_SS_INTD_setClearIntrCPSWEventInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    CSL_FINS(hCpswIntDRegs->ENABLE_CLR_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_CLR_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_EVNT_PENDA_CLR,
             1);
}

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
                                    CSL_CPSW_SS_INTD_INTRSTATUS *intrStatus)
{
    intrStatus->cpswEventActive =
    CSL_FEXT(hCpswIntDRegs->STATUS_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_STATUS_REG_OUT_PULSE_0_STATUS_OUT_PULSE_EVNT_PENDA);
    intrStatus->mdioInterruptActive =
    CSL_FEXT(hCpswIntDRegs->STATUS_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_STATUS_REG_OUT_PULSE_0_STATUS_OUT_PULSE_MDIO_PENDA);
    intrStatus->statPendInterruptActive =
    CSL_FEXT(hCpswIntDRegs->STATUS_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_STATUS_REG_OUT_PULSE_0_STATUS_OUT_PULSE_STAT_PENDA);
}

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
                                                 Uint32              enable)
{
    CSL_FINS(hCpswIntDRegs->ENABLE_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_MDIO_PENDA,
             enable);
}

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
Uint32 CSL_CPSW_SS_INTD_getEnableIntrMdioInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    return (CSL_FEXT(hCpswIntDRegs->ENABLE_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_MDIO_PENDA));
}

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
void CSL_CPSW_SS_INTD_setClearIntrMdioInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    CSL_FINS(hCpswIntDRegs->ENABLE_CLR_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_CLR_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_MDIO_PENDA_CLR,
             1);
}

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
                                                 Uint32              enable)
{
    CSL_FINS(hCpswIntDRegs->ENABLE_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_STAT_PENDA,
             enable);
}

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
Uint32 CSL_CPSW_SS_INTD_getEnableIntrStatInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    return(CSL_FEXT(hCpswIntDRegs->ENABLE_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_STAT_PENDA));

}

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
void CSL_CPSW_SS_INTD_setClearIntrStatInterrupt(CSL_cpswRegs_INTD * hCpswIntDRegs)
{
    CSL_FINS(hCpswIntDRegs->ENABLE_CLR_REG_OUT_PULSE_0,
             XGE_CPSW_SS_S_INTD_ENABLE_CLR_REG_OUT_PULSE_0_ENABLE_OUT_PULSE_EN_STAT_PENDA_CLR,
             1);
}

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
                              CSL_SGMII_MODE sgmiiModeId)
{
    CSL_FINSR (hCpswSsRegs->SGMII_NON_FIBER_MODE_REG, macPortNum, macPortNum, sgmiiModeId);
}

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
                                CSL_CPSW_SS_RGMIISTATUS *pRgmiiStatus)
{
    switch(macPortNum)
    {
        case 0:
          pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII1_STATUS_REG, XGE_CPSW_SS_S_RGMII1_STATUS_REG_LINK);
          pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII1_STATUS_REG,XGE_CPSW_SS_S_RGMII1_STATUS_REG_SPEED);
          pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII1_STATUS_REG,XGE_CPSW_SS_S_RGMII1_STATUS_REG_FULLDUPLEX);
          break;
        case 1:
          pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII2_STATUS_REG, XGE_CPSW_SS_S_RGMII2_STATUS_REG_LINK);
          pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII2_STATUS_REG,XGE_CPSW_SS_S_RGMII2_STATUS_REG_SPEED);
          pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII2_STATUS_REG,XGE_CPSW_SS_S_RGMII2_STATUS_REG_FULLDUPLEX);
          break;
        case 2:
          pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII3_STATUS_REG, XGE_CPSW_SS_S_RGMII3_STATUS_REG_LINK);
          pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII3_STATUS_REG,XGE_CPSW_SS_S_RGMII3_STATUS_REG_SPEED);
          pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII3_STATUS_REG,XGE_CPSW_SS_S_RGMII3_STATUS_REG_FULLDUPLEX);
          break;
        case 3:
          pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII4_STATUS_REG, XGE_CPSW_SS_S_RGMII4_STATUS_REG_LINK);
          pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII4_STATUS_REG,XGE_CPSW_SS_S_RGMII4_STATUS_REG_SPEED);
          pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII4_STATUS_REG,XGE_CPSW_SS_S_RGMII4_STATUS_REG_FULLDUPLEX);
          break;
        case 4:
          pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII5_STATUS_REG, XGE_CPSW_SS_S_RGMII5_STATUS_REG_LINK);
          pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII5_STATUS_REG,XGE_CPSW_SS_S_RGMII5_STATUS_REG_SPEED);
          pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII5_STATUS_REG,XGE_CPSW_SS_S_RGMII5_STATUS_REG_FULLDUPLEX);
          break;
        case 5:
          pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII6_STATUS_REG, XGE_CPSW_SS_S_RGMII6_STATUS_REG_LINK);
          pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII6_STATUS_REG,XGE_CPSW_SS_S_RGMII6_STATUS_REG_SPEED);
          pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII6_STATUS_REG,XGE_CPSW_SS_S_RGMII6_STATUS_REG_FULLDUPLEX);
          break;
        case 6:
          pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII7_STATUS_REG, XGE_CPSW_SS_S_RGMII7_STATUS_REG_LINK);
          pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII7_STATUS_REG,XGE_CPSW_SS_S_RGMII7_STATUS_REG_SPEED);
          pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII7_STATUS_REG,XGE_CPSW_SS_S_RGMII7_STATUS_REG_FULLDUPLEX);
          break;
        case 7:
          pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII8_STATUS_REG, XGE_CPSW_SS_S_RGMII8_STATUS_REG_LINK);
          pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII8_STATUS_REG,XGE_CPSW_SS_S_RGMII8_STATUS_REG_SPEED);
          pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII8_STATUS_REG,XGE_CPSW_SS_S_RGMII8_STATUS_REG_FULLDUPLEX);
          break;
    }
}

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
Uint32 CSL_CPSW_SS_getSerdesResetIsolation(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs)
{
    return(CSL_FEXT(hCpswSsRegs->SERDES_RESET_ISO_REG,
                    XGE_CPSW_SS_S_SERDES_RESET_ISO_REG_SERDES_RESET_ISO));
}

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
                                         Uint32 resetIso)
{
    CSL_FINS(hCpswSsRegs->SERDES_RESET_ISO_REG,
             XGE_CPSW_SS_S_SERDES_RESET_ISO_REG_SERDES_RESET_ISO,
             resetIso);
}

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
                                        Uint32 qsgmiiId)
{
    Uint32 qsgmiiControlRdCd  = 0;

    switch (qsgmiiId)
    {
        case 0:
            qsgmiiControlRdCd = CSL_FEXT(hCpswSsRegs->QSGMII_CONTROL_REG,
                                     XGE_CPSW_SS_S_QSGMII_CONTROL_REG_Q0_RDCD);
            break;
        case 1:
            qsgmiiControlRdCd = CSL_FEXT(hCpswSsRegs->QSGMII_CONTROL_REG,
                                     XGE_CPSW_SS_S_QSGMII_CONTROL_REG_Q1_RDCD);
            break;
    }
    return qsgmiiControlRdCd;
}

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
                                      Uint32 qsgmiiControlRdCd)
{
    switch (qsgmiiId)
    {
        case 0:
            CSL_FINS(hCpswSsRegs->QSGMII_CONTROL_REG,
                     XGE_CPSW_SS_S_QSGMII_CONTROL_REG_Q0_RDCD,
                     qsgmiiControlRdCd);
            break;
        case 1:
            CSL_FINS(hCpswSsRegs->QSGMII_CONTROL_REG,
                     XGE_CPSW_SS_S_QSGMII_CONTROL_REG_Q1_RDCD,
                     qsgmiiControlRdCd);
            break;
    }
}

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
                                         Uint32 qsgmiiId)
{
    Uint32 qsgmiiStatusRxSync  = 0;

    switch (qsgmiiId)
    {
        case 0:
            qsgmiiStatusRxSync = CSL_FEXT(hCpswSsRegs->QSGMII_STATUS_REG,
                                        XGE_CPSW_SS_S_QSGMII_STATUS_REG_Q0_RX_SYNC);
            break;
        case 1:
            qsgmiiStatusRxSync = CSL_FEXT(hCpswSsRegs->QSGMII_STATUS_REG,
                                        XGE_CPSW_SS_S_QSGMII_STATUS_REG_Q1_RX_SYNC );
            break;
    }
    return qsgmiiStatusRxSync;
}

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
                                      Uint32 xgmiiId)
{
    Uint32 xgmiiStatus  = 0;

    switch (xgmiiId)
    {
        case 0:
            xgmiiStatus = CSL_FEXT(hCpswSsRegs->STATUS_XGMII_LINK_REG,
                                     XGE_CPSW_SS_S_STATUS_XGMII_LINK_REG_XGMII1_LINK);
            break;
        case 1:
            xgmiiStatus = CSL_FEXT(hCpswSsRegs->STATUS_XGMII_LINK_REG,
                                     XGE_CPSW_SS_S_STATUS_XGMII_LINK_REG_XGMII2_LINK );
            break;
    }
    return xgmiiStatus;
}

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getSGMIILinkStatus
 *
 *   @b Description
 *   @n This function returns the QSGMII status register for the given qsgmii
 *
 *   @b Arguments
     @verbatim
        macPortNum Mac Port for which SGMII link is checked
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  SGMII link status for the given SGMII MAC port
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_SS_getSGMIILinkStatus(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                      Uint32 macPortNum)
{
    Uint32 sgmiiLinkStatus  = 0;

    sgmiiLinkStatus = CSL_FEXTR(hCpswSsRegs->STATUS_SGMII_LINK_REG, macPortNum, macPortNum);

    return sgmiiLinkStatus;
}

/**
@}
*/
