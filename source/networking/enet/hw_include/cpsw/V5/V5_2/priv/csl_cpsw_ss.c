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
 *  @n   (C) Copyright 2020, Texas Instruments, Inc.
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
        hCpswSsRegs     Pointer to CSL_Xge_cpsw_ss_sRegs structure
        versionInfo     CSL_CPSW_SS_VERSION structure that needs to be populated
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
 *   @n@b CSL_CPSW_SS_getRGMIIStatus
 *
 *   @b Description
 *   @n This function returns the RGMII status for the given macPort
 *
 *   @b Arguments
     @verbatim
        hCpswSsRegs     Pointer to CSL_Xge_cpsw_ss_sRegs structure
        macPortNum      Mac Port for which RGMII status is queried
        pRgmiiStatus    RGMII status to be populated
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_SS_S_RGMII1_STATUS_REG_LINK
 *      XGE_CPSW_SS_S_RGMII1_STATUS_REG_SPEED
 *      XGE_CPSW_SS_S_RGMII1_STATUS_REG_FULLDUPLEX
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_SS_RGMIISTATUS    rgmiiStatus;

        CSL_CPSW_SS_getRGMIIStatus (0, &rgmiiStatus);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_SS_getRGMIIStatus(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                                Uint32 macPortNum,
                                CSL_CPSW_SS_RGMIISTATUS *pRgmiiStatus)
{
    if (macPortNum == 0U)
    {
        pRgmiiStatus->link = CSL_FEXT(hCpswSsRegs->RGMII1_STATUS_REG, XGE_CPSW_SS_S_RGMII1_STATUS_REG_LINK);
        pRgmiiStatus->speed = CSL_FEXT(hCpswSsRegs->RGMII1_STATUS_REG,XGE_CPSW_SS_S_RGMII1_STATUS_REG_SPEED);
    	pRgmiiStatus->fullDuplex = CSL_FEXT(hCpswSsRegs->RGMII1_STATUS_REG,XGE_CPSW_SS_S_RGMII1_STATUS_REG_FULLDUPLEX);
    }
    
    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_SS_getConfigInfo
 *
 *   @b Description
 *   @n This function returns the CPSW Sub-system Configuration information
 *
 *   @b Arguments
     @verbatim
        hCpswSsRegs     Pointer to CSL_Xge_cpsw_ss_sRegs structure
        pConfigInfo     CPSW Sub-system configuration information to be populated
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_PORTS
 *      XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_GENF
 *      XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RMII
 *      XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RGMI
 *      XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_SGMII
 *      XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_QSGMII
 *      XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_XGMI
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_SS_CONFIG_INFO    configInfo;

        CSL_CPSW_SS_getConfigInfo (&configInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_SS_getConfigInfo(CSL_Xge_cpsw_ss_sRegs * hCpswSsRegs,
                               CSL_CPSW_SS_CONFIG_INFO *pConfigInfo)
{
    pConfigInfo->numPorts = CSL_FEXT(hCpswSsRegs->SUBSYSTEM_CONFIG_REG, XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_PORTS);
    pConfigInfo->numGenfs = CSL_FEXT(hCpswSsRegs->SUBSYSTEM_CONFIG_REG, XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_NUM_GENF);
    pConfigInfo->rmii = CSL_FEXT(hCpswSsRegs->SUBSYSTEM_CONFIG_REG, XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RMII);
    pConfigInfo->rgmii = CSL_FEXT(hCpswSsRegs->SUBSYSTEM_CONFIG_REG, XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_RGMII);
    pConfigInfo->sgmii = CSL_FEXT(hCpswSsRegs->SUBSYSTEM_CONFIG_REG, XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_SGMII);
    pConfigInfo->qsgmii = CSL_FEXT(hCpswSsRegs->SUBSYSTEM_CONFIG_REG, XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_QSGMII);
    pConfigInfo->xgmii = CSL_FEXT(hCpswSsRegs->SUBSYSTEM_CONFIG_REG, XGE_CPSW_SS_S_SUBSYSTEM_CONFIG_REG_XGMII);
    
    return;
}                               

/**
@}
*/
