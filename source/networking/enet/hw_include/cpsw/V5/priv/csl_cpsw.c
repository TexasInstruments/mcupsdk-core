/**
 * @file  csl_cpsw.c
 *
 * @brief
 *  API Function layer file for Ethernet switch module CSL.
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
#include <stdint.h>
#include <drivers/hw_include/csl_types.h>
#include <cslr_ale.h>
#include <cslr_xge_cpsw.h>
#include <csl_cpswitch.h>

#if defined (SOC_AM273X) || defined (SOC_AWR294X) || defined(SOC_AM263X) || defined (SOC_AM263PX) || defined(SOC_AM261X)
#define CSL_NUM_ALE_VLAN_MASK_MUX1_ENTRIES		(3U)
#else
#define CSL_NUM_ALE_VLAN_MASK_MUX1_ENTRIES		(7U)
#endif

#define CPSW_EST_FETCH_COUNT_SHIFT                      (8U)
#define CPSW_EST_FETCH_COUNT_MASK                       (0x003FFF00U)
#define CPSW_EST_FETCH_ALLOW_SHIFT                      (0U)
#define CPSW_EST_FETCH_ALLOW_MASK                       (0x000000FFU)

#define CSL_CPSW_GET_HOSTPORT_STAT_START_ADDRESS(x)     (&(x->P0_STATS.RXGOODFRAMES))
#define CSL_CPSW_GET_MACPORT_STAT_START_ADDRESS(x, y)   (&(x->PN_STATS[y].RXGOODFRAMES))

/********************************************************************************
************************* Ethernet Switch (CPSW) Submodule **********************
********************************************************************************/


/** ============================================================================
 *   @n@b CSL_CPSW_nGF_getCpswVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the CPSW identification and version information.
 *
 *   @b Arguments
     @verbatim
        pVersionInfo        CSL_CPSW_VERSION structure that needs to be populated
                            with the version info read from the hardware.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CPSW_ID_VER_REG_CPSW_5GF_MINOR_VER,
 *      XGE_CPSW_CPSW_ID_VER_REG_CPSW_5GF_MAJ_VER,
 *      XGE_CPSW_CPSW_ID_VER_REG_CPSW_5GF_RTL_VER,
 *      XGE_CPSW_CPSW_ID_VER_REG_CPSW_5GF_IDENT
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_VERSION    versionInfo;

        CSL_CPSW_getCpswVersionInfo (&versionInfo);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getCpswVersionInfo (CSL_Xge_cpswRegs *hCpswRegs,
	CSL_CPSW_VERSION*       pVersionInfo
)
{

    pVersionInfo->minorVer  =   CSL_FEXT (hCpswRegs->ID_VER_REG, XGE_CPSW_ID_VER_REG_MINOR_VER);
    pVersionInfo->majorVer  =   CSL_FEXT (hCpswRegs->ID_VER_REG, XGE_CPSW_ID_VER_REG_MAJOR_VER);
    pVersionInfo->rtlVer    =   CSL_FEXT (hCpswRegs->ID_VER_REG, XGE_CPSW_ID_VER_REG_RTL_VER);
    pVersionInfo->id        =   CSL_FEXT (hCpswRegs->ID_VER_REG, XGE_CPSW_ID_VER_REG_IDENT);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isVlanAwareEnabled
 *
 *   @b Description
 *   @n This function indicates if VLAN aware mode is enabled in the CPSW
 *      control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   VLAN aware mode enabled.
 *	 @n  FALSE                  VLAN aware mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isVlanAwareEnabled (portNum) == TRUE)
        {
            // VLAN aware mode enabled
        }
        else
        {
            // VLAN aware mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isVlanAwareEnabled (CSL_Xge_cpswRegs *hCpswRegs)
{
    return CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_VLAN_AWARE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableVlanAware
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable VLAN aware
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableVlanAware ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableVlanAware (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_VLAN_AWARE, 1);

    return;
}

void CSL_CPSW_setVlanType (CSL_Xge_cpswRegs *hCpswRegs,Uint32 vlanType)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_S_CN_SWITCH, vlanType);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableVlanAware
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable VLAN aware
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableVlanAware ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableVlanAware (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_VLAN_AWARE, 0);

    return;
}

#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
/** ============================================================================
 *   @n@b CSL_CPSW_enableCutThru
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable cut-thru
 *      switching.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_CUT_THRU_ENABLE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableCutThru ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableCutThru (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_CPSW_CONTROL controlReg;

    CSL_CPSW_getCpswControlReg(hCpswRegs, &controlReg);
    controlReg.cutThruEnable = TRUE;
    CSL_CPSW_setCpswControlReg(hCpswRegs, &controlReg);
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableCutThru
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable cut-thru
 *      switching.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_CUT_THRU_ENABLE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableCutThru ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableCutThru (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_CPSW_CONTROL controlReg;

    CSL_CPSW_getCpswControlReg(hCpswRegs, &controlReg);
    controlReg.cutThruEnable = FALSE;
    CSL_CPSW_setCpswControlReg(hCpswRegs, &controlReg);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setCpswFrequency
 *
 *   @b Description
 *   @n This function configures the CPSW frequency used for enabling auto speed
 *      register for cut-thru switching.
 *
 *   @b Arguments
 *   @verbatim
        pCpswFrequency      CPSW_FREQUENCY that needs to be set.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_FREQUENCY_REG
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_setCpswFrequency ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setCpswFrequency (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      pCpswFrequency
)
{
    CSL_FINS(hCpswRegs->FREQUENCY, XGE_CPSW_FREQUENCY_REG, pCpswFrequency);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getCpswFrequency
 *
 *   @b Description
 *   @n This function gets the RX Cut thru priority
 *
 *   @b Arguments
     @verbatim
        pCpswFrequency   CPSW frequency
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
 *   @n XGE_CPSW_FREQUENCY_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32      pCpswFrequency;

        pCpswFrequency = CSL_CPSW_getCpswFrequency ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getCpswFrequency (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pCpswFrequency
)
{
    *pCpswFrequency = CSL_FEXT(hCpswRegs->FREQUENCY, XGE_CPSW_FREQUENCY_REG);
}
#endif

/** ============================================================================
 *   @n@b CSL_CPSW_isPort0Enabled
 *
 *   @b Description
 *   @n This function indicates if CPPI Port (Port 0) is enabled in the CPSW
 *      control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Port 0 enabled.
 *	 @n  FALSE                  Port 0 disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_P0_ENABLE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isPort0Enabled (portNum) == TRUE)
        {
            // Port 0 enabled
        }
        else
        {
            // Port 0 disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isPort0Enabled (CSL_Xge_cpswRegs *hCpswRegs)
{
    return CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_ENABLE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enablePort0
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable the Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P0_ENABLE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enablePort0 ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enablePort0 (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_ENABLE, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disablePort0
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable the Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P0_ENABLE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disablePort0 ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disablePort0 (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_ENABLE, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isPort0PassPriTagEnabled
 *
 *   @b Description
 *   @n This function indicates if priority tagging is enabled for Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Port 0 ingress priority tagging enabled.
 *	 @n  FALSE                  Port 0 ingress priority tagging disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isPort0PassPriTagEnabled () == TRUE)
        {
            // Port 0 pass priority tagging enabled
        }
        else
        {
            // Port 0 pass priority tagging disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isPort0PassPriTagEnabled (CSL_Xge_cpswRegs *hCpswRegs)
{
    return CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enablePort0PassPriTag
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable the Ingress
 *      priority tagging on Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enablePort0PassPriTag ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enablePort0PassPriTag (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disablePort0PassPriTag
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable the Ingress
 *      priority tagging on Port 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disablePort0PassPriTag ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disablePort0PassPriTag (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isPort1PassPriTagEnabled
 *
 *   @b Description
 *   @n This function indicates if priority tagging is enabled for Port 1.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Port 1 ingress priority tagging enabled.
 *	 @n  FALSE                  Port 1 ingress priority tagging disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isPort1PassPriTagEnabled () == TRUE)
        {
            // Port 1 pass priority tagging enabled
        }
        else
        {
            // Port 1 pass priority tagging disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isPort1PassPriTagEnabled (CSL_Xge_cpswRegs *hCpswRegs)
{
    return CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enablePort1PassPriTag
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable the Ingress
 *      priority tagging on Port 1.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enablePort1PassPriTag ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enablePortPassPriTag (CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNum)
{
    switch(portNum)
    {
        case 0:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED, 1);
            break;
        case 1:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED, 1);
            break;
        case 2:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P2_PASS_PRI_TAGGED, 1);
            break;
        case 3:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P3_PASS_PRI_TAGGED, 1);
            break;
        case 4:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P4_PASS_PRI_TAGGED, 1);
            break;
        case 5:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P5_PASS_PRI_TAGGED, 1);
            break;
        case 6:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P6_PASS_PRI_TAGGED, 1);
            break;
        case 7:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P7_PASS_PRI_TAGGED, 1);
            break;
        default:
            break;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disablePort1PassPriTag
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable the Ingress
 *      priority tagging on Port 1.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disablePort1PassPriTag ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disablePortPassPriTag (CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNum)
{
    switch(portNum)
    {
        case 0:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED, 0);
            break;
        case 1:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED, 0);
            break;
        case 2:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P2_PASS_PRI_TAGGED, 0);
            break;
        case 3:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P3_PASS_PRI_TAGGED, 0);
            break;
        case 4:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P4_PASS_PRI_TAGGED, 0);
            break;
        case 5:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P5_PASS_PRI_TAGGED, 0);
            break;
        case 6:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P6_PASS_PRI_TAGGED, 0);
            break;
        case 7:
            CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P7_PASS_PRI_TAGGED, 0);
            break;
        default:
            break;
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getCpswControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW Control register.
 *
 *   @b Arguments
     @verbatim
        pControlRegInfo     CSL_CPSW_CONTROL structure that needs to be populated
                            with the control register contents.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE,
 *      XGE_CPSW_CONTROL_REG_P0_ENABLE,
 *      XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P0_RX_PAD,
 *      XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR,
 *      XGE_CPSW_CONTROL_REG_EEE_ENABLE
 *      XGE_CPSW_CONTROL_REG_EST_ENABLE
 *
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_CONTROL    controlRegInfo;

        CSL_CPSW_getCpswControlReg (&controlRegInfo);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getCpswControlReg (CSL_Xge_cpswRegs *hCpswRegs,
	CSL_CPSW_CONTROL*   pControlRegInfo
)
{
    pControlRegInfo->fifoLb         =   0;
    pControlRegInfo->vlanAware      =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_VLAN_AWARE);
    pControlRegInfo->p0Enable       =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_ENABLE);
    pControlRegInfo->p0PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED);
    pControlRegInfo->p1PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED);
    pControlRegInfo->p2PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P2_PASS_PRI_TAGGED);
    pControlRegInfo->p3PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P3_PASS_PRI_TAGGED);
    pControlRegInfo->p4PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P4_PASS_PRI_TAGGED);
    pControlRegInfo->p5PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P5_PASS_PRI_TAGGED);
    pControlRegInfo->p6PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P6_PASS_PRI_TAGGED);
    pControlRegInfo->p7PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P7_PASS_PRI_TAGGED);
    pControlRegInfo->p8PassPriTag   =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P8_PASS_PRI_TAGGED);
    pControlRegInfo->p0TxCrcRemove  =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_TX_CRC_REMOVE);
    pControlRegInfo->p0RxPad        =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_RX_PAD);
    pControlRegInfo->p0RxPassCrcErr =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR);
    pControlRegInfo->eeeEnable      =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_EEE_ENABLE);
    pControlRegInfo->estEnable      =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_EST_ENABLE);
#if ENET_CFG_IS_ON(CPSW_IET_INCL)
    pControlRegInfo->ietEnable      =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_IET_ENABLE);
#endif
#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
    pControlRegInfo->cutThruEnable  =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_CUT_THRU_ENABLE);
#endif

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setCpswControlReg
 *
 *   @b Description
 *   @n This function populates the contents of the CPSW Control register.
 *
 *   @b Arguments
     @verbatim
        pControlRegInfo     CSL_CPSW_CONTROL structure that holds the values
                            that need to be configured to the CPSW control register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSW control register modified with values provided.
 *
 *   @b Writes
 *   @n XGE_CPSW_CONTROL_REG_VLAN_AWARE,
 *      XGE_CPSW_CONTROL_REG_P0_ENABLE,
 *      XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED,
 *      XGE_CPSW_CONTROL_REG_P0_TX_CRC_REMOVE,
 *      XGE_CPSW_CONTROL_REG_P0_RX_PAD,
 *      XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR,
 *      XGE_CPSW_CONTROL_REG_EEE_ENABLE
 *      XGE_CPSW_CONTROL_REG_EST_ENABLE
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_CONTROL    controlRegInfo;

        controlRegInfo.vlanAware    =   0;
        ...

        CSL_CPSW_setCpswControlReg (&controlRegInfo);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setCpswControlReg (CSL_Xge_cpswRegs *hCpswRegs,
	CSL_CPSW_CONTROL*   pControlRegInfo
)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_VLAN_AWARE, pControlRegInfo->vlanAware);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_ENABLE, pControlRegInfo->p0Enable);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED, pControlRegInfo->p0PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED, pControlRegInfo->p1PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P2_PASS_PRI_TAGGED, pControlRegInfo->p2PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P3_PASS_PRI_TAGGED, pControlRegInfo->p3PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P4_PASS_PRI_TAGGED, pControlRegInfo->p4PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P5_PASS_PRI_TAGGED, pControlRegInfo->p5PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P6_PASS_PRI_TAGGED, pControlRegInfo->p6PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P7_PASS_PRI_TAGGED, pControlRegInfo->p7PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P8_PASS_PRI_TAGGED, pControlRegInfo->p8PassPriTag);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_TX_CRC_REMOVE, pControlRegInfo->p0TxCrcRemove);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_RX_PAD, pControlRegInfo->p0RxPad);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR, pControlRegInfo->p0RxPassCrcErr);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_EEE_ENABLE, pControlRegInfo->eeeEnable);
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_EST_ENABLE, pControlRegInfo->estEnable);
#if ENET_CFG_IS_ON(CPSW_IET_INCL)
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_IET_ENABLE, pControlRegInfo->ietEnable);
#endif
#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_CUT_THRU_ENABLE, pControlRegInfo->cutThruEnable);
#endif

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getEmulationControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW Emulation Control register.
 *
 *   @b Arguments
     @verbatim
        pFree                   Emulation free bit read from the hardware.
        pSoft                   Emulation soft bit read from the hardware.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_EM_CONTROL_REG_FREE,
 *      XGE_CPSW_EM_CONTROL_REG_SOFT
 *
 *   @b Example
 *   @verbatim
        Uint32  free, soft;

        CSL_CPSW_getEmulationControlReg (&free, &soft);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getEmulationControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pFree,
    Uint32*                     pSoft
)
{

    *pFree      =   CSL_FEXT (hCpswRegs->EM_CONTROL_REG, XGE_CPSW_EM_CONTROL_REG_FREE);
    *pSoft      =   CSL_FEXT (hCpswRegs->EM_CONTROL_REG, XGE_CPSW_EM_CONTROL_REG_SOFT);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setEmulationControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW Emulation Control register.
 *
 *   @b Arguments
     @verbatim
        free                   Emulation free bit configuration
        soft                   Emulation soft bit configuration
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_EM_CONTROL_REG_FREE,
 *      XGE_CPSW_EM_CONTROL_REG_SOFT
 *
 *   @b Example
 *   @verbatim
        Uint32 free, soft;

        free   =   0;
        soft   =   1;

        CSL_CPSW_setEmulationControlReg (free, soft);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setEmulationControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      free,
    Uint32                      soft
)
{

    CSL_FINS (hCpswRegs->EM_CONTROL_REG, XGE_CPSW_EM_CONTROL_REG_FREE, free);
    CSL_FINS (hCpswRegs->EM_CONTROL_REG, XGE_CPSW_EM_CONTROL_REG_SOFT, soft);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortStatsEnableReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW Port Statistics
 *      Enable register.
 *
 *   @b Arguments
     @verbatim
        pPortStatsCfg       CSL_XGE_CPSW_PORTSTAT structure that needs to be populated
                            with the port statistics enable register contents.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN,
 *      XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN,
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_PORTSTAT       portStatsCfg;

        CSL_CPSW_getPortStatsEnableReg (&portStatsCfg);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortStatsEnableReg (CSL_Xge_cpswRegs *hCpswRegs,
	CSL_CPSW_PORTSTAT*  pPortStatsCfg
)
{
    pPortStatsCfg->p0StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN);
    pPortStatsCfg->p1StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN);
    pPortStatsCfg->p2StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P2_STAT_EN);
    pPortStatsCfg->p3StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P3_STAT_EN);
    pPortStatsCfg->p4StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P4_STAT_EN);
    pPortStatsCfg->p5StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P5_STAT_EN);
    pPortStatsCfg->p6StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P6_STAT_EN);
    pPortStatsCfg->p7StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P7_STAT_EN);
    pPortStatsCfg->p8StatEnable    =   CSL_FEXT (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P8_STAT_EN);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortStatsEnableReg
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW Port Statistics
 *      Enable register.
 *
 *   @b Arguments
     @verbatim
        pPortStatsCfg       CSL_CPSW_PORTSTAT structure that contains the values
                            to be used to setup port statistics enable register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN,
 *      XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN,
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_PORTSTAT       portStatsCfg;

        portStatsCfg.p0StatEnable  =   1;
        portStatsCfg.p1StatEnable  =   1;
        ...

        CSL_CPSW_setPortStatsEnableReg (&portStatsCfg);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortStatsEnableReg (CSL_Xge_cpswRegs *hCpswRegs,
	CSL_CPSW_PORTSTAT*  pPortStatsCfg
)
{
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN, pPortStatsCfg->p0StatEnable);
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN, pPortStatsCfg->p1StatEnable);
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P2_STAT_EN, pPortStatsCfg->p2StatEnable);
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P3_STAT_EN, pPortStatsCfg->p3StatEnable);
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P4_STAT_EN, pPortStatsCfg->p4StatEnable);
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P5_STAT_EN, pPortStatsCfg->p5StatEnable);
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P6_STAT_EN, pPortStatsCfg->p6StatEnable);
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P7_STAT_EN, pPortStatsCfg->p7StatEnable);
    CSL_FINS (hCpswRegs->STAT_PORT_EN_REG, XGE_CPSW_STAT_PORT_EN_REG_P8_STAT_EN, pPortStatsCfg->p8StatEnable);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isSoftIdle
 *
 *   @b Description
 *   @n This function indicates if the CPSW is at Software Idle mode where
 *      no packets will be started to be unloaded from ports.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Software Idle mode enabled.
 *	 @n  FALSE                  Software Idle mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isSoftIdle () == TRUE)
        {
            // Software Idle mode enabled
        }
        else
        {
            // Software Idle mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isSoftIdle (CSL_Xge_cpswRegs *hCpswRegs)
{
    return CSL_FEXT (hCpswRegs->SOFT_IDLE_REG, XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableSoftIdle
 *
 *   @b Description
 *   @n This function configures the CPSW Soft Idle register to enable Software
 *      Idle mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableSoftIdle ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableSoftIdle (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->SOFT_IDLE_REG, XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableSoftIdle
 *
 *   @b Description
 *   @n This function configures the CPSW Soft Idle register to disable Software
 *      Idle mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableSoftIdle ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableSoftIdle (CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->SOFT_IDLE_REG, XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Control Register
 *      corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register contents
                                must be read and returned.
        pControlInfo            CSL_CPSW_PORT_CONTROL structure that needs to be populated
                                with the control register contents.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  none
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN,
 *      XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN,
 *
 *      XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN,
 *      XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN,
 *      XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN,
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32                      portNum;
        CSL_CPSW_PORT_CONTROL       controlInfo;

        portNum =   1;

        CSL_CPSW_getPortControlReg (portNum, &controlInfo);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPSW_PORT_CONTROL*      pControlInfo
)
{
    if (portNum == 0)
    {
        pControlInfo->dscpIpv4Enable      =   CSL_FEXT (hCpswRegs->P0_CONTROL_REG, XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN);
        pControlInfo->dscpIpv6Enable      =   CSL_FEXT (hCpswRegs->P0_CONTROL_REG, XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN);

    } else
    {
        pControlInfo->estPortEnable      =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN);
#if ENET_CFG_IS_ON(CPSW_IET_INCL)
        pControlInfo->ietPortEnable      =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_IET_PORT_EN);
#endif
        pControlInfo->dscpIpv4Enable     =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN);
        pControlInfo->dscpIpv6Enable     =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN);
        pControlInfo->txLpiClkstopEnable =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN);
        pControlInfo->estPortEnable       =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN);
        pControlInfo->dscpIpv4Enable      =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN);
        pControlInfo->dscpIpv6Enable      =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN);
        pControlInfo->txLpiClkstopEnable  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port Control Register
 *      corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register contents
                                must be read and returned.
        pControlInfo            CSL_CPSW_PORT_CONTROL structure that holds the values
                                that need to be configured to the CPSW Port
                                control register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  none
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN,
 *      XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN,
 *
 *      XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN,
 *      XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN,
 *      XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN,
 *      XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN,
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32                      portNum;
        CSL_CPSW_PORT_CONTROL       controlInfo;

        portNum =   1;

        controlInfo.dscpIpv4Enable = 1;
        controlInfo.dscpIpv6Enable = 1;
        controlInfo.txLpiClkstopEnable = 0;


        CSL_CPSW_setPortControlReg (portNum, &controlInfo);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortControlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPSW_PORT_CONTROL*      pControlInfo
)
{
    if (portNum == 0)
    {
        CSL_FINS (hCpswRegs->P0_CONTROL_REG, XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN, pControlInfo->dscpIpv4Enable);
        CSL_FINS (hCpswRegs->P0_CONTROL_REG, XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN, pControlInfo->dscpIpv6Enable);

    } else
    {

        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN, pControlInfo->estPortEnable);
#if ENET_CFG_IS_ON(CPSW_IET_INCL)
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_IET_PORT_EN, pControlInfo->ietPortEnable);
#endif
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN, pControlInfo->dscpIpv4Enable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN, pControlInfo->dscpIpv6Enable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN, pControlInfo->txLpiClkstopEnable);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getCppiSourceIdReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPPI Source Identification
 *      register.
 *
 *   @b Arguments
     @verbatim
        pTxSrcId[8]         CPPI Info Word0 Source Id Value on Tx Ports respectively.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_SRC_ID_A_REG_PORT1
 *
 *   @b Example
 *   @verbatim
 *      Uint32      txSrcId[8];

        CSL_CPSW_getCppiSourceIdReg (txSrcId);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getCppiSourceIdReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                 pTxSrcId
)
{
    pTxSrcId[0]    =   CSL_FEXT (hCpswRegs->P0_SRC_ID_A_REG, XGE_CPSW_P0_SRC_ID_A_REG_PORT1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiSourceIdReg
 *
 *   @b Description
 *   @n This function sets up the contents of the CPPI Source Identification
 *      register.
 *
 *   @b Arguments
     @verbatim
        pTxSrcId[8]         CPPI Info Word0 Source Id Value on Tx Ports respectively.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_SRC_ID_A_REG_PORT1
 *
 *   @b Example
 *   @verbatim
 *      Uint32      txSrcId[8];

        txSrcId[0]    =   1;
        ...

        CSL_CPSW_setCppiSourceIdReg (txSrcId);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setCppiSourceIdReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                 pTxSrcId
)
{
    CSL_FINS (hCpswRegs->P0_SRC_ID_A_REG, XGE_CPSW_P0_SRC_ID_A_REG_PORT1, pTxSrcId[0]);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPort0VlanReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port 0 VLAN Register.
 *
 *   @b Arguments
     @verbatim
        pPortVID                Port VLAN Id
        pPortCFI                Port CFI bit
        pPortPRI                Port VLAN priority (0-7, 7 is highest priority)
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portVID, portCFI, portPRI;

        CSL_CPSW_getPort0VlanReg (&portVID, &portCFI, &portPRI);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPort0VlanReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pPortVID,
    Uint32*                     pPortCFI,
    Uint32*                     pPortPRI
)
{
    *pPortVID   =   CSL_FEXT (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID);
    *pPortCFI   =   CSL_FEXT (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI);
    *pPortPRI   =   CSL_FEXT (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPort0VlanReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port 0 VLAN Register.
 *
 *   @b Arguments
     @verbatim
        portVID                 Port VLAN Id to be configured
        portCFI                 Port CFI bit to be configured
        portPRI                 Port VLAN priority to be configured
                                (0-7, 7 is highest priority)
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portVID, portCFI, portPRI;

        portVID     =   1;
        portCFI     =   0;
        portPRI     =   7;

        CSL_CPSW_setPort0VlanReg (portVID, portCFI, portPRI);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPort0VlanReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portVID,
    Uint32                      portCFI,
    Uint32                      portPRI
)
{
    CSL_FINS (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID, portVID);
    CSL_FINS (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI, portCFI);
    CSL_FINS (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI, portPRI);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPort0RxPriMapReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port 0 Receive Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        pPortRxPriMap           Array of Port 0 Rx priority map priority values
                                read from the register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pPortRxPriMap' must be large enough to hold all
 *       the 8 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      port0RxPriMap [8];

        CSL_CPSW_getPort0RxPriMapReg (port0RxPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPort0RxPriMapReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pPortRxPriMap
)
{
    pPortRxPriMap [0]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0);
    pPortRxPriMap [1]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1);
    pPortRxPriMap [2]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2);
    pPortRxPriMap [3]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3);
    pPortRxPriMap [4]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4);
    pPortRxPriMap [5]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5);
    pPortRxPriMap [6]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6);
    pPortRxPriMap [7]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPort0RxPriMapReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port 0 Receive Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        pPortRxPriMap           Array of Port 0 Rx priority map priority values
                                that must be configured to the register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      i, port0RxPriMap [8];

        for (i = 0; i < 8; i ++)
            port0RxPriMap [i] = i;

        CSL_CPSW_setPort0RxPriMapReg (port0RxPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPort0RxPriMapReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pPortRxPriMap
)
{
    CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0, pPortRxPriMap [0]);
    CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1, pPortRxPriMap [1]);
    CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2, pPortRxPriMap [2]);
    CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3, pPortRxPriMap [3]);
    CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4, pPortRxPriMap [4]);
    CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5, pPortRxPriMap [5]);
    CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6, pPortRxPriMap [6]);
    CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7, pPortRxPriMap [7]);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPort0FlowIdOffset
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port 0 Flow ID Offset
 *      Register, which is added to the thread/Flow_ID in CPPI transmit PSI
 *      Info Word 0.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE
 *
 *   @b Example
 *   @verbatim
 *      Uint32      flowIdOffset;

        flowIdOffset    =   CSL_CPSW_getPort0FlowIdOffset ();

	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getPort0FlowIdOffset (CSL_Xge_cpswRegs *hCpswRegs)
{
    return CSL_FEXT (hCpswRegs->P0_FLOW_ID_OFFSET_REG, XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPort0FlowIdOffset
 *
 *   @b Description
 *   @n This function sets up the Port0 Flow ID Offset register.
 *      which is added to the thread/Flow_ID in CPPI transmit PSI
 *      Info Word 0.
 *
 *   @b Arguments
     @verbatim
        flowIdOffset            CPPI Flow ID offset to configure.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE
 *
 *   @b Example
 *   @verbatim
 *      Uint32      flowIdOffset;

        flowIdOffset    =   0;

        CSL_CPSW_setPort0FlowIdOffset (flowIdOffset);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPort0FlowIdOffset (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  flowIdOffset
)
{
    CSL_FINS (hCpswRegs->P0_FLOW_ID_OFFSET_REG, XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE, flowIdOffset);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPort0RxMaxLen
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port 0 Receive Maximum Length
 *      Register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxMaxLen;

        rxMaxLen    =   CSL_CPSW_getPort0RxMaxLen ();

	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getPort0RxMaxLen (CSL_Xge_cpswRegs *hCpswRegs)
{
    return CSL_FEXT (hCpswRegs->P0_RX_MAXLEN_REG, XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPort0RxMaxLen
 *
 *   @b Description
 *   @n This function sets up the Port0 Receive Maximum length register.
 *
 *   @b Arguments
     @verbatim
        rxMaxLen            Maximum receive frame length to configure.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxMaxLen;

        rxMaxLen    =   1518;

        CSL_CPSW_setPort0RxMaxLen (rxMaxLen);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPort0RxMaxLen (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  rxMaxLen
)
{
    CSL_FINS (hCpswRegs->P0_RX_MAXLEN_REG, XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN, rxMaxLen);

    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_getPortBlockCountReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Block Count register
 *      corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the block count
                                must be retrieved.
        pRxBlkCnt_e             Receive block count usage read for express MAC for this port.
        pRxBlkCnt_p             Receive block count usage read for preempt MAC for this port.
        pTxBlkCnt               Transmit block count usage read for this port.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_E,
 *      XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_P,
 *      XGE_CPSW_PN_BLK_CNT_REG_TX_BLK_CNT,
 *      XGE_CPSW_P0_BLK_CNT_REG_RX_BLK_CNT,
 *      XGE_CPSW_P0_BLK_CNT_REG_TX_BLK_CNT,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxBlkCnt_e, rxBlkCnt_p, txBlkCnt, portNum;

        portNum =   1;

        CSL_CPSW_getPortBlockCountReg (portNum, &rxBlkCnt, &rxBlkCnt_p,
                                       &txBlkCnt);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortBlockCountReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pRxBlkCnt_e,
    Uint32*                     pRxBlkCnt_p,
    Uint32*                     pTxBlkCnt
)
{
    if (portNum == 0)
    {
        *pRxBlkCnt_e  =   CSL_FEXT (hCpswRegs->P0_BLK_CNT_REG, XGE_CPSW_P0_BLK_CNT_REG_RX_BLK_CNT);
        /* No preempt MAC for host port so set block count to 0 */
        *pRxBlkCnt_p  =   0;
        *pTxBlkCnt  =   CSL_FEXT (hCpswRegs->P0_BLK_CNT_REG, XGE_CPSW_P0_BLK_CNT_REG_TX_BLK_CNT);
    } else
    {
        *pRxBlkCnt_e  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_BLK_CNT_REG, XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_E);
        *pRxBlkCnt_p  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_BLK_CNT_REG, XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_P);
        *pTxBlkCnt  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_BLK_CNT_REG, XGE_CPSW_PN_BLK_CNT_REG_TX_BLK_CNT);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortRxMaxLen
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW Port Receive Maximum Length
 *      Register.
 *
 *   @b Arguments
        portNum             CPSW port number for which the Receive Maximum Length
                            must be retrieved.
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN
 *      XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, rxMaxLen;

        portNum = 1;
        rxMaxLen    =   CSL_CPSW_getPortRxMaxLen (portNum);

	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getPortRxMaxLen (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum
)
{
    if(portNum == 0)
        return CSL_FEXT (hCpswRegs->P0_RX_MAXLEN_REG, XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN);
    else
        return CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_MAXLEN_REG, XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortRxMaxLen
 *
 *   @b Description
 *   @n This function sets up the Port Receive Maximum length register.
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the Receive Maximum Length
                            must be retrieved.
        rxMaxLen            Maximum receive frame length to configure.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN
 *      XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, rxMaxLen;

        portNum     =   1;
        rxMaxLen    =   1518;

        CSL_CPSW_setPortRxMaxLen (portNum, rxMaxLen);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortRxMaxLen (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32                  rxMaxLen
)
{
    if(portNum == 0)
    {
        CSL_FINS (hCpswRegs->P0_RX_MAXLEN_REG, XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN, rxMaxLen);
    }
    else if (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT))
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_MAXLEN_REG, XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN, rxMaxLen);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortTxPriMapReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Transmit Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the block count
                                must be retrieved.
        pPortRxPriMap           Array of Port Rx priority map priority values
                                read from the register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pPortRxPriMap' must be large enough to hold all
 *       the 8 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum = 1;
 *      Uint32      portRxPriMap [8];

        CSL_CPSW_getPortTxPriMapReg (portNum, portRxPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTxPriMapReg (
    CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortTxPriMap
)
{
    if (portNum == 0)
    {
        pPortTxPriMap [0]   =   CSL_FEXT (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0);
        pPortTxPriMap [1]   =   CSL_FEXT (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1);
        pPortTxPriMap [2]   =   CSL_FEXT (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2);
        pPortTxPriMap [3]   =   CSL_FEXT (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3);
        pPortTxPriMap [4]   =   CSL_FEXT (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4);
        pPortTxPriMap [5]   =   CSL_FEXT (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5);
        pPortTxPriMap [6]   =   CSL_FEXT (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6);
        pPortTxPriMap [7]   =   CSL_FEXT (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7);
    }
    else if (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT))
    {
        pPortTxPriMap [0]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0);
        pPortTxPriMap [1]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1);
        pPortTxPriMap [2]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2);
        pPortTxPriMap [3]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3);
        pPortTxPriMap [4]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4);
        pPortTxPriMap [5]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5);
        pPortTxPriMap [6]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6);
        pPortTxPriMap [7]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortTxPriMapReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port Transmit Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the priority mapping
                                registers must be configured.
        pPortRxPriMap           Array of Port Rx priority map priority values
                                that must be configured to the register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum = 1;
 *      Uint32      i, port0RxPriMap [8];

        for (i = 0; i < 8; i ++)
            port0TxPriMap [i] = i;

        CSL_CPSW_setPortTxPriMapReg (port0TxPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTxPriMapReg (
    CSL_Xge_cpswRegs           *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortTxPriMap
)
{
    if(portNum == 0)
    {
        CSL_FINS (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0, pPortTxPriMap [0]);
        CSL_FINS (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1, pPortTxPriMap [1]);
        CSL_FINS (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2, pPortTxPriMap [2]);
        CSL_FINS (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3, pPortTxPriMap [3]);
        CSL_FINS (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4, pPortTxPriMap [4]);
        CSL_FINS (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5, pPortTxPriMap [5]);
        CSL_FINS (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6, pPortTxPriMap [6]);
        CSL_FINS (hCpswRegs->P0_TX_PRI_MAP_REG, XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7, pPortTxPriMap [7]);
    }
    else if (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT))
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0, pPortTxPriMap [0]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1, pPortTxPriMap [1]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2, pPortTxPriMap [2]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3, pPortTxPriMap [3]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4, pPortTxPriMap [4]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5, pPortTxPriMap [5]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6, pPortTxPriMap [6]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TX_PRI_MAP_REG, XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7, pPortTxPriMap [7]);
    }
    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_getPortRxPriMapReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Receive Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the block count
                                must be retrieved.
        pPortRxPriMap           Array of Port Rx priority map priority values
                                read from the register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pPortRxPriMap' must be large enough to hold all
 *       the 8 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum = 1;
 *      Uint32      portRxPriMap [8];

        CSL_CPSW_getPortRxPriMapReg (portNum, portRxPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortRxPriMapReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortRxPriMap
)
{
    if (portNum == 0)
    {
        pPortRxPriMap [0]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0);
        pPortRxPriMap [1]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1);
        pPortRxPriMap [2]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2);
        pPortRxPriMap [3]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3);
        pPortRxPriMap [4]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4);
        pPortRxPriMap [5]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5);
        pPortRxPriMap [6]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6);
        pPortRxPriMap [7]   =   CSL_FEXT (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7);
    }
    else if (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT))
    {
        pPortRxPriMap [0]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0);
        pPortRxPriMap [1]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1);
        pPortRxPriMap [2]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2);
        pPortRxPriMap [3]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3);
        pPortRxPriMap [4]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4);
        pPortRxPriMap [5]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5);
        pPortRxPriMap [6]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6);
        pPortRxPriMap [7]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortRxPriMapReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port Receive Packet
 *      Priority to Header Priority Mapping Register.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the priority mapping
                                registers must be configured.
        pPortRxPriMap           Array of Port Rx priority map priority values
                                that must be configured to the register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6,
 *      XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum = 1;
 *      Uint32      i, port0RxPriMap [8];

        for (i = 0; i < 8; i ++)
            port0RxPriMap [i] = i;

        CSL_CPSW_setPortRxPriMapReg (port0RxPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortRxPriMapReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortRxPriMap
)
{
    if(portNum == 0)
    {
        CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0, pPortRxPriMap [0]);
        CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1, pPortRxPriMap [1]);
        CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2, pPortRxPriMap [2]);
        CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3, pPortRxPriMap [3]);
        CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4, pPortRxPriMap [4]);
        CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5, pPortRxPriMap [5]);
        CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6, pPortRxPriMap [6]);
        CSL_FINS (hCpswRegs->P0_RX_PRI_MAP_REG, XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7, pPortRxPriMap [7]);
    }
    else if (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT))
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0, pPortRxPriMap [0]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1, pPortRxPriMap [1]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2, pPortRxPriMap [2]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3, pPortRxPriMap [3]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4, pPortRxPriMap [4]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5, pPortRxPriMap [5]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6, pPortRxPriMap [6]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_PRI_MAP_REG, XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7, pPortRxPriMap [7]);
    }
    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortRxDscpMap
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port DSCP to Priority
 *      Mapping Registers corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the DSCP mapping
                                registers must be retrieved.
        pRxDscpPriMap           Array of Port Rx DSCP to priority mapping values
                                read from the registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pRxDscpPriMap' must be large enough to hold all
 *       the 64 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxDscpPriMap [64], portNum;

        portNum = 1;
        CSL_CPSW_getPortRxDscpMap (portNum, rxDscpPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortRxDscpMap (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pRxDscpPriMap
)
{
    int i, j;

    if(portNum == 0)
    {
        for( i = 0, j = 0; i < 8; j+=8, i++)
        {
            pRxDscpPriMap [j+0]   =   CSL_FEXT (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0);
            pRxDscpPriMap [j+1]   =   CSL_FEXT (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1);
            pRxDscpPriMap [j+2]   =   CSL_FEXT (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2);
            pRxDscpPriMap [j+3]   =   CSL_FEXT (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3);
            pRxDscpPriMap [j+4]   =   CSL_FEXT (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4);
            pRxDscpPriMap [j+5]   =   CSL_FEXT (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5);
            pRxDscpPriMap [j+6]   =   CSL_FEXT (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6);
            pRxDscpPriMap [j+7]   =   CSL_FEXT (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7);
        }
    }
    else
    {

        for( i = 0, j = 0; i < 8; j+=8, i++)
        {
            pRxDscpPriMap [j+0]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0);
            pRxDscpPriMap [j+1]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1);
            pRxDscpPriMap [j+2]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2);
            pRxDscpPriMap [j+3]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3);
            pRxDscpPriMap [j+4]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4);
            pRxDscpPriMap [j+5]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5);
            pRxDscpPriMap [j+6]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6);
            pRxDscpPriMap [j+7]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7);
        }
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortRxDscpMap
 *
 *   @b Description
 *   @n This function sets up the contents of the Port DSCP to Priority
 *      Mapping Registers corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the DSCP mapping
                                registers must be configured.
        pRxDscpPriMap           Array of Port Rx DSCP to priority mapping values
                                that must be configured to the registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pRxDscpPriMap' must be large enough to hold all
 *       the 64 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6,
 *      XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7,
 *
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6,
 *      XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxDscpPriMap [64], portNum;

        portNum = 1;

        for (i = 0; i < 64; i ++)
            port0RxPriMap [i] = i/8;

        CSL_CPSW_setPortRxDscpMap (portNum, rxDscpPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortRxDscpMap (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pRxDscpPriMap
)
{
    int i, j;

    if(portNum == 0)
    {
        for( i = 0, j = 0; i < 8; j+=8, i++)
        {
            CSL_FINS (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0, pRxDscpPriMap [j+0]);
            CSL_FINS (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1, pRxDscpPriMap [j+1]);
            CSL_FINS (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2, pRxDscpPriMap [j+2]);
            CSL_FINS (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3, pRxDscpPriMap [j+3]);
            CSL_FINS (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4, pRxDscpPriMap [j+4]);
            CSL_FINS (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5, pRxDscpPriMap [j+5]);
            CSL_FINS (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6, pRxDscpPriMap [j+6]);
            CSL_FINS (hCpswRegs->P0_RX_DSCP_MAP_REG[i], XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7, pRxDscpPriMap [j+7]);
        }
    }
    else
    {

        for( i = 0, j = 0; i < 8; j+=8, i++)
        {
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0, pRxDscpPriMap [j+0]);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1, pRxDscpPriMap [j+1]);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2, pRxDscpPriMap [j+2]);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3, pRxDscpPriMap [j+3]);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4, pRxDscpPriMap [j+4]);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5, pRxDscpPriMap [j+5]);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6, pRxDscpPriMap [j+6]);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_RX_DSCP_MAP_REG[i], XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7, pRxDscpPriMap [j+7]);
        }
    }

    return;
}

#if ENET_CFG_IS_ON(CPSW_CUTTHRU)
/** ============================================================================
 *   @n@b CSL_CPSW_setPortRxCutThruPri
 *
 *   @b Description
 *   @n This function sets up the RX Cut thru priority. This register is used to
 *      set the remapped recieve packet priority from 0..7
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the RX Cut thru priority
                            must be set.
        pPortRxCutThruPri   RX Cut thru priority
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
 *   @b Writes
 *   @n XGE_CPSW_PN_CUT_THRU_REG_RX_PRI_CUT_THRU_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortRxCutThruPri;

        portNum           = 1;
        pPortRxCutThruPri = 1;

        CSL_CPSW_setPortRxCutThruPri (portNum, pPortRxCutThruPri);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortRxCutThruPri (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      pPortRxCutThruPri
)
{
    CSL_FINS(hCpswRegs->ENETPORT[portNum].PN_CUT_THRU_REG, XGE_CPSW_PN_CUT_THRU_REG_RX_PRI_CUT_THRU_EN, pPortRxCutThruPri);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortRxCutThruPri
 *
 *   @b Description
 *   @n This function gets the RX Cut thru priority.This register is used to
 *      get the remapped recieve packet priority i.e., 0..7
 *      0x01 = Priority 0
 *      0x02 = Priority 1
 *      0x04 = Priority 2 an so on
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the RX Cut thru priority
                            must be retrieved.
        pPortRxCutThruPri   RX Cut thru priority
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
 *   @n XGE_CPSW_PN_CUT_THRU_REG_RX_PRI_CUT_THRU_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortRxCutThruPri;

        portNum           = 1;

        pPortRxCutThruPri = CSL_CPSW_getPortRxCutThruPri (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortRxCutThruPri (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortRxCutThruPri
)
{
    *pPortRxCutThruPri = CSL_FEXT(hCpswRegs->ENETPORT[portNum].PN_CUT_THRU_REG, XGE_CPSW_PN_CUT_THRU_REG_RX_PRI_CUT_THRU_EN);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortTxCutThruPri
 *
 *   @b Description
 *   @n This function sets up the TX Cut thru priority. This register is used to
 *      set the remapped transmit packet priority i.e., 0..7
 *      0x01 = Priority 0
 *      0x02 = Priority 1
 *      0x04 = Priority 2 an so on
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the TX Cut thru priority
                            must be set.
        pPortTxCutThruPri   TX Cut thru priority
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
 *   @b Writes
 *   @n XGE_CPSW_PN_CUT_THRU_REG_TX_PRI_CUT_THRU_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortRxCutThruPri;

        portNum           = 1;
        pPortTxCutThruPri = 1;

        CSL_CPSW_setPortTxCutThruPri (portNum, pPortTxCutThruPri);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTxCutThruPri (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      pPortTxCutThruPri
)
{
    CSL_FINS(hCpswRegs->ENETPORT[portNum].PN_CUT_THRU_REG, XGE_CPSW_PN_CUT_THRU_REG_TX_PRI_CUT_THRU_EN, pPortTxCutThruPri);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortTxCutThruPri
 *
 *   @b Description
 *   @n This function gets the TX Cut thru priority. This register is used to
 *      get the remapped transmit packet priority i.e., 0..7
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the TX Cut thru priority
                            must be retrieved.
        pPortRxCutThruPri   TX Cut thru priority
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
 *   @n XGE_CPSW_PN_CUT_THRU_REG_TX_PRI_CUT_THRU_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortTxCutThruPri;

        portNum           = 1;

        pPortTxCutThruPri = CSL_CPSW_getPortTxCutThruPri (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTxCutThruPri (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortTxCutThruPri
)
{
    if ((portNum >= 0) && (portNum < CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        *pPortTxCutThruPri = CSL_FEXT(hCpswRegs->ENETPORT[portNum].PN_CUT_THRU_REG, XGE_CPSW_PN_CUT_THRU_REG_TX_PRI_CUT_THRU_EN);
    }
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortSpeedAutoEnable
 *
 *   @b Description
 *   @n This function sets up the port's speed for auto enablement. Setting this value
 *      will ensure that port speed is automatically detected by the hardware. Cut-thru
 *      operations doesn't work if the port speed is diabled.
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the speed
                            must be auto enabled.
        pPortSpeedAutoEn    Port Speed
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
 *   @b Writes
 *   @n XGE_CPSW_PN_SPEED_REG_AUTO_ENABLE
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortSpeedAutoEn;

        portNum           = 1;
        pPortSpeedAutoEn  = 1;

        CSL_CPSW_setPortSpeedAutoEnable (portNum, pPortSpeedAutoEn);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortSpeedAutoEnable (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    bool                        pPortSpeedAutoEn
)
{
    if ((portNum >= 0) && (portNum < CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS(hCpswRegs->ENETPORT[portNum].PN_SPEED, XGE_CPSW_PN_SPEED_REG_AUTO_ENABLE, pPortSpeedAutoEn);
    }
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortSpeedAutoEnable
 *
 *   @b Description
 *   @n This function gets the auto enable speed status of the port
 *
 *   @b Arguments
     @verbatim
        portNum             CPSW port number for which the auto enable status
                            must be retrieved.
        pPortSpeed          Port Speed
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
 *   @n XGE_CPSW_PN_SPEED_REG_AUTO_ENABLE
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum, pPortSpeedAutoEn;

        portNum           = 1;

        pPortSpeedAutoEn = CSL_CPSW_getPortSpeedAutoEnable (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortSpeedAutoEnable (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    bool*                       pPortSpeedAutoEn
)
{
    if ((portNum >= 0) && (portNum < CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        *pPortSpeedAutoEn = CSL_FEXT(hCpswRegs->ENETPORT[portNum].PN_SPEED, XGE_CPSW_PN_SPEED_REG_AUTO_ENABLE);
    }
}
#endif

/** ============================================================================
 *   @n@b CSL_CPSW_getEEEGlobConfig
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW EEE Global Configuration.
 *
 *   @b Arguments
     @verbatim
        pGlobConfig             CSL_CPSW_EEE_GLOB_CONFIG structure that needs to be populated
                                with the contents of the corresponging EEE global control
                                registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE
 *      XGE_CPSW_CONTROL_REG_EEE_ENABLE
 *
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_EEE_GLOB_CONFIG    globConfig;

        CSL_CPSW_getEEEGlobConfig (&globConfig);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getEEEGlobConfig (CSL_Xge_cpswRegs *hCpswRegs,
	CSL_CPSW_EEE_GLOB_CONFIG*   pGlobConfig
)
{
    pGlobConfig->enable      =   CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_EEE_ENABLE);
    pGlobConfig->prescale    =   CSL_FEXT (hCpswRegs->EEE_PRESCALE_REG, XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setEEEGlobConfig
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW EEE related global registers
 *      per user-specified EEE Global Configuration.
 *
 *   @b Arguments
     @verbatim
        pGlobConfig         CSL_CPSW_EEE_GLOB_CONFIG structure that holds the values
                            that need to be configured to the EEE global control
                            registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSW EEE Global control register modified with values provided.
 *
 *   @b Writes
 *   @n XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE
 *      XGE_CPSW_CONTROL_REG_EEE_ENABLE
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_EEE_GLOB_CONFIG    globConfig;

        globConfig.enable       =   1;
        globalConfig.prescale   =   100;
        ...

        CSL_CPSW_setEEEGlobConfig (&globConfig);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setEEEGlobConfig (CSL_Xge_cpswRegs *hCpswRegs,
	CSL_CPSW_EEE_GLOB_CONFIG*   pGlobConfig
)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_EEE_ENABLE, pGlobConfig->enable);
    CSL_FINS (hCpswRegs->EEE_PRESCALE_REG, XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE, pGlobConfig->prescale);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getEEEPortConfig
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW EEE Port Configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the EEE Port Control
                                registers must be retrieved.
        pPortConfig             CSL_CPSW_EEE_PORT_CONFIG structure that needs to be populated
                                with the contents of the corresponging EEE port-specific control
                                registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_IDLE2LPI_REG_COUNT
 *      XGE_CPSW_P0_LPI2WAKE_REG_COUNT
 *
 *      XGE_CPSW_PN_IDLE2LPI_REG_COUNT
 *      XGE_CPSW_PN_LPI2WAKE_REG_COUNT
 *      XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN
 *
 *   @b Example
 *   @verbatim
        Uint32                      portNum;
        CSL_CPSW_EEE_PORT_CONFIG    portConfig;

        portNum = 1;
        CSL_CPSW_getEEEPortConfig (portNum, &portConfig);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getEEEPortConfig (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
	CSL_CPSW_EEE_PORT_CONFIG*   pPortConfig
)
{
    if(portNum == 0)
    {
        pPortConfig->idle2lpi    =   CSL_FEXT (hCpswRegs->P0_IDLE2LPI_REG, XGE_CPSW_P0_IDLE2LPI_REG_COUNT);
        pPortConfig->lpi2wake    =   CSL_FEXT (hCpswRegs->P0_LPI2WAKE_REG, XGE_CPSW_P0_LPI2WAKE_REG_COUNT);
    }
    else
    {
        pPortConfig->idle2lpi    =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_IDLE2LPI_REG, XGE_CPSW_PN_IDLE2LPI_REG_COUNT);
        pPortConfig->lpi2wake    =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_LPI2WAKE_REG, XGE_CPSW_PN_LPI2WAKE_REG_COUNT);
        pPortConfig->txLpiClkstopEnable =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setEEEPortConfig
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW EEE port-specific control
 *      registers per user-specified EEE Port Configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the EEE Port Control
                                registers must be configured.
        pPortConfig             CSL_CPSW_EEE_PORT_CONFIG structure holds the value
                                that needs to be configured to the EEE port-specific
                                control registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_IDLE2LPI_REG_COUNT
 *      XGE_CPSW_P0_LPI2WAKE_REG_COUNT
 *
 *      XGE_CPSW_PN_IDLE2LPI_REG_COUNT
 *      XGE_CPSW_PN_LPI2WAKE_REG_COUNT
 *      XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN
 *
 *   @b Example
 *   @verbatim
        Uint32                      portNum;
        CSL_CPSW_EEE_PORT_CONFIG    portConfig;

        portNum = 1;
        portConfig.idle2lpi = 10;
        portConfig.lpi2wake = 10;
        portConfig.txLpiClkstopEnable = 1;
        CSL_CPSW_setEEEPortConfig (portNum, &portConfig);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setEEEPortConfig (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
	CSL_CPSW_EEE_PORT_CONFIG*   pPortConfig
)
{
    if(portNum == 0)
    {
        CSL_FINS (hCpswRegs->P0_IDLE2LPI_REG, XGE_CPSW_P0_IDLE2LPI_REG_COUNT, pPortConfig->idle2lpi);
        CSL_FINS (hCpswRegs->P0_LPI2WAKE_REG, XGE_CPSW_P0_LPI2WAKE_REG_COUNT, pPortConfig->lpi2wake);
    }
    else
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_IDLE2LPI_REG, XGE_CPSW_PN_IDLE2LPI_REG_COUNT, pPortConfig->idle2lpi);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_LPI2WAKE_REG, XGE_CPSW_PN_LPI2WAKE_REG_COUNT, pPortConfig->lpi2wake);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_CONTROL_REG, XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN, pPortConfig->txLpiClkstopEnable);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getEEEPortStatus
 *
 *   @b Description
 *   @n This function retrieves the contents of the EEE port-specific Status register
 *      corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the EEE status
                                must be retrieved.
        pPortStatus             CSL_CPSW_EEE_PORT_STATUS structure holds the EEE Port Status.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_EEE_STATUS_REG_WAIT_IDLE2LPI,
 *      XGE_CPSW_P0_EEE_STATUS_REG_RX_LPI,
 *      XGE_CPSW_P0_EEE_STATUS_REG_TX_LPI,
 *      XGE_CPSW_P0_EEE_STATUS_REG_TX_WAKE,
 *      XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_HOLD,
 *      XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_EMPTY,
 *      XGE_CPSW_P0_EEE_STATUS_REG_RX_FIFO_EMPTY,
 *
 *      XGE_CPSW_PN_EEE_STATUS_REG_WAIT_IDLE2LPI,
 *      XGE_CPSW_PN_EEE_STATUS_REG_RX_LPI,
 *      XGE_CPSW_PN_EEE_STATUS_REG_TX_LPI,
 *      XGE_CPSW_PN_EEE_STATUS_REG_TX_WAKE,
 *      XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_HOLD,
 *      XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_EMPTY,
 *      XGE_CPSW_PN_EEE_STATUS_REG_RX_FIFO_EMPTY
 *
 *
 *
 *   @b Example
 *   @verbatim
 *      Uint32                      portNum;
        CSL_CPSW_EEE_PORT_STATUS    portStatus;

        portNum =   1;

        CSL_CPSW_getEEEPortStatus (portNum, &portStatus);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_EEEPortStatus (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPSW_EEE_PORT_STATUS*   pPortStatus
)
{
    if (portNum == 0)
    {
        pPortStatus->wait_idle2lpi  =   CSL_FEXT (hCpswRegs->P0_EEE_STATUS_REG, XGE_CPSW_P0_EEE_STATUS_REG_WAIT_IDLE2LPI);
        pPortStatus->rxLpi          =   CSL_FEXT (hCpswRegs->P0_EEE_STATUS_REG, XGE_CPSW_P0_EEE_STATUS_REG_RX_LPI);
        pPortStatus->txLpi          =   CSL_FEXT (hCpswRegs->P0_EEE_STATUS_REG, XGE_CPSW_P0_EEE_STATUS_REG_TX_LPI);
        pPortStatus->txWake         =   CSL_FEXT (hCpswRegs->P0_EEE_STATUS_REG, XGE_CPSW_P0_EEE_STATUS_REG_TX_WAKE);
        pPortStatus->txFifoHold     =   CSL_FEXT (hCpswRegs->P0_EEE_STATUS_REG, XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_HOLD);
        pPortStatus->txFifoEmpty    =   CSL_FEXT (hCpswRegs->P0_EEE_STATUS_REG, XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_EMPTY);
        pPortStatus->rxFifoEmpty    =   CSL_FEXT (hCpswRegs->P0_EEE_STATUS_REG, XGE_CPSW_P0_EEE_STATUS_REG_RX_FIFO_EMPTY);

    } else
    {
        pPortStatus->wait_idle2lpi  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_EEE_STATUS_REG, XGE_CPSW_PN_EEE_STATUS_REG_WAIT_IDLE2LPI);
        pPortStatus->rxLpi          =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_EEE_STATUS_REG, XGE_CPSW_PN_EEE_STATUS_REG_RX_LPI);
        pPortStatus->txLpi          =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_EEE_STATUS_REG, XGE_CPSW_PN_EEE_STATUS_REG_TX_LPI);
        pPortStatus->txWake         =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_EEE_STATUS_REG, XGE_CPSW_PN_EEE_STATUS_REG_TX_WAKE);
        pPortStatus->txFifoHold     =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_EEE_STATUS_REG, XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_HOLD);
        pPortStatus->txFifoEmpty    =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_EEE_STATUS_REG, XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_EMPTY);
        pPortStatus->rxFifoEmpty    =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_EEE_STATUS_REG, XGE_CPSW_PN_EEE_STATUS_REG_RX_FIFO_EMPTY);
    }

    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_getPortVlanReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the VLAN Register corresponding
 *      to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the VLAN register
                                contents must be read
        pPortVID                Port VLAN Id
        pPortCFI                Port CFI bit
        pPortPRI                Port VLAN priority (0-7, 7 is highest priority)
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI,
 *
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portVID, portCFI, portPRI, portNum;

        portNum =   1;

        CSL_CPSW_getPortVlanReg (portNum, &portVID, &portCFI, &portPRI);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortVlanReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pPortVID,
    Uint32*                     pPortCFI,
    Uint32*                     pPortPRI
)
{
    if(portNum == 0)
    {
        *pPortVID   =   CSL_FEXT (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID);
        *pPortCFI   =   CSL_FEXT (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI);
        *pPortPRI   =   CSL_FEXT (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI);
    }
    else
    {
        *pPortVID   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_PORT_VLAN_REG, XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID);
        *pPortCFI   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_PORT_VLAN_REG, XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI);
        *pPortPRI   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_PORT_VLAN_REG, XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI);
    }

    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_setPortVlanReg
 *
 *   @b Description
 *   @n This function sets up the contents of the VLAN Register corresponding to
 *      the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the VLAN register
                                must be configured.
        portVID                 Port VLAN Id to be configured
        portCFI                 Port CFI bit to be configured
        portPRI                 Port VLAN priority to be configured
                                (0-7, 7 is highest priority)
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI,
 *
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID,
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI,
 *      XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portVID, portCFI, portPRI, portNum;

        portNum     =   1;
        portVID     =   1;
        portCFI     =   0;
        portPRI     =   7;

        CSL_CPSW_setPortVlanReg (portNum, portVID, portCFI, portPRI);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortVlanReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      portVID,
    Uint32                      portCFI,
    Uint32                      portPRI
)
{
    if (portNum == 0)
    {
        CSL_FINS (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID, portVID);
        CSL_FINS (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI, portCFI);
        CSL_FINS (hCpswRegs->P0_PORT_VLAN_REG, XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI, portPRI);
    }
    else
    {
        if (portNum < CSL_ARRAYSIZE(hCpswRegs->ENETPORT))
        {
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_PORT_VLAN_REG, XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID, portVID);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_PORT_VLAN_REG, XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI, portCFI);
            CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_PORT_VLAN_REG, XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI, portPRI);
        }
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortMaxBlksReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the Port Maxmium Block register
 *      corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the max block value
                                must be retrieved.
        pRxMaxBlks              Receive FIFO Maximum Blocks - This value is the
                                maximum number of 1k memory blocks that may be
                                allocated to the FIFO's logical receive queue.
                                This value must be greater than or equal to 0x3.
                                The recommended value of rx_max_blks is 0x9

        pTxMaxBlks              Transmit FIFO Maximum Blocks - This value is the
                                maximum number of 1k memory blocks that may be
                                allocated to the FIFO's logical transmit
                                priority queues.  The recommended value of
                                tx_max_blks is 0x3.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS,
 *      XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxMaxBlks, txMaxBlks, portNum;

        portNum =   1;

        CSL_CPSW_getPortMaxBlksReg (portNum, &rxMaxBlks, &txMaxBlks);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortMaxBlksReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pRxMaxBlks,
    Uint32*                     pTxMaxBlks
)
{
    if ((portNum >= 1) && (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        *pRxMaxBlks  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_MAX_BLKS_REG, XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS);
        *pTxMaxBlks  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_MAX_BLKS_REG, XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortMaxBlksReg
 *
 *   @b Description
 *   @n This function sets up the contents of the Port Maxmium Block register
 *      corresponding to the CPSW port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the max block value
                                must be retrieved.
        rxMaxBlks               Receive FIFO Maximum Blocks - This value is the
                                maximum number of 1k memory blocks that may be
                                allocated to the FIFO's logical receive queue.
                                This value must be greater than or equal to 0x3.
                                The recommended value of rx_max_blks is 0x9

        txMaxBlks               Transmit FIFO Maximum Blocks - This value is the
                                maximum number of 1k memory blocks that may be
                                allocated to the FIFO's logical transmit
                                priority queues.  The recommended value of
                                tx_max_blks is 0x3.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS,
 *      XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS,
 *
 *   @b Example
 *   @verbatim
 *      Uint32      rxMaxBlks, txMaxBlks, portNum;

        portNum =   1;

        CSL_CPSW_setPortMaxBlksReg (portNum, rxMaxBlks, txMaxBlks);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortMaxBlksReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      rxMaxBlks,
    Uint32                      txMaxBlks
)
{
    if ((portNum >= 1) && (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_MAX_BLKS_REG, XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS, rxMaxBlks);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_MAX_BLKS_REG, XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS, txMaxBlks);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortMACAddress
 *
 *   @b Description
 *   @n This function retreives the source MAC address of the Tx Pause Frame corresponding to the
 *      CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the source MAC address
                                must be read and returned. (1-8)
        pMacAddress             6 byte Source MAC address read.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pMacAddres' must be large enough the 6 byte
 *       MAC address returned by this API.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0,
 *      XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40
 *
 *   @b Example
 *   @verbatim
 *      Uint8   macAddress [6], portNum;

        portNum =   1;

        CSL_CPSW_getPortMACAddress (portNum, macAddress);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortMACAddress (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint8*                      pMacAddress
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        pMacAddress [0]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_SA_L_REG, XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0);
        pMacAddress [1]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_SA_L_REG, XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8);
        pMacAddress [2]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_SA_H_REG, XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16);
        pMacAddress [3]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_SA_H_REG, XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24);
        pMacAddress [4]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_SA_H_REG, XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32);
        pMacAddress [5]   =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_SA_H_REG, XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortMACAddress
 *
 *   @b Description
 *   @n This function sets up the source MAC address the Tx Pause Frame corresponding to the
 *      CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the source MAC address
                                must be setup. (1-8)
        pMacAddress             6 byte Source MAC address to configure.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pMacAddres' is expected to be 6 bytes long.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0,
 *      XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32,
 *      XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40
 *
 *   @b Example
 *   @verbatim
 *      Uint8   macAddress [6], portNum;

        portNum         =   1;
        macAddress [0]  =   0x01;
        macAddress [1]  =   0x02;
        macAddress [2]  =   0x03;
        macAddress [3]  =   0x04;
        macAddress [4]  =   0x05;
        macAddress [5]  =   0x06;

        CSL_CPSW_setPortMACAddress (portNum, macAddress);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortMACAddress (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint8*                      pMacAddress
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_SA_L_REG, XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0,   pMacAddress [0]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_SA_L_REG, XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8,  pMacAddress [1]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_SA_H_REG, XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16, pMacAddress [2]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_SA_H_REG, XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24, pMacAddress [3]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_SA_H_REG, XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32, pMacAddress [4]);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_SA_H_REG, XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40, pMacAddress [5]);
    }

    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_getPortTimeSyncCntlReg
 *
 *   @b Description
 *   @n This function retreives the contents of Time sync control register
 *      corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be read. (1-8)
        pTimeSyncCntlCfg        CSL_CPSW_TSCNTL that needs to be populated with
                                contents of time sync control register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_TSCNTL     tsCtlCfg;

        portNum =   1;

        CSL_CPSW_getPortTimeSyncCntlReg (portNum, &tsCtlCfg);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTimeSyncCntlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    CSL_CPSW_TSCNTL*        pTimeSyncCntlCfg
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        pTimeSyncCntlCfg->tsRxAnnexDEnable          =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                  XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN);
        pTimeSyncCntlCfg->tsRxAnnexEEnable          =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                  XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN);
        pTimeSyncCntlCfg->tsRxAnnexFEnable          =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                  XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN);
        pTimeSyncCntlCfg->tsRxVlanLType1Enable      =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                  XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN);
        pTimeSyncCntlCfg->tsRxVlanLType2Enable      =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN);
        pTimeSyncCntlCfg->tsTxAnnexDEnable          =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN);
        pTimeSyncCntlCfg->tsTxAnnexEEnable          =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN);
        pTimeSyncCntlCfg->tsTxAnnexFEnable          =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN);
        pTimeSyncCntlCfg->tsTxVlanLType1Enable      =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN);
        pTimeSyncCntlCfg->tsTxVlanLType2Enable      =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN);
        pTimeSyncCntlCfg->tsMsgTypeEnable           =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN);
        pTimeSyncCntlCfg->tsTxHostEnable            =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN);
        pTimeSyncCntlCfg->tsLType2Enable            =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortTimeSyncCntlReg
 *
 *   @b Description
 *   @n This function sets up the contents of Time sync control register
 *      corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be
                                configured.
        pTimeSyncCntlCfg        CSL_CPSW_TSCNTL containing settings for time
                                sync control register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_TSCNTL     tsCtlCfg;

        portNum =   1;

        tsCtlCfg.tsRxVlanLType1Enable   =   0;
        tsCtlCfg.tsRxVlanLType2Enable   =   0;
        ...

        CSL_CPSW_setPortTimeSyncCntlReg (portNum, &tsCtlCfg);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTimeSyncCntlReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    CSL_CPSW_TSCNTL*        pTimeSyncCntlCfg
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
                  pTimeSyncCntlCfg->tsRxAnnexDEnable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
                  pTimeSyncCntlCfg->tsRxAnnexEEnable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
                  pTimeSyncCntlCfg->tsRxAnnexFEnable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
                  pTimeSyncCntlCfg->tsRxVlanLType1Enable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
                  pTimeSyncCntlCfg->tsRxVlanLType2Enable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
                  pTimeSyncCntlCfg->tsTxAnnexDEnable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
                  pTimeSyncCntlCfg->tsTxAnnexEEnable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
                  pTimeSyncCntlCfg->tsTxAnnexFEnable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
                  pTimeSyncCntlCfg->tsTxVlanLType1Enable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
                  pTimeSyncCntlCfg->tsTxVlanLType2Enable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
                  pTimeSyncCntlCfg->tsTxHostEnable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
                  pTimeSyncCntlCfg->tsLType2Enable);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG, XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN,
                  pTimeSyncCntlCfg->tsMsgTypeEnable);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortTimeSyncSeqIdReg
 *
 *   @b Description
 *   @n This function retreives the contents of Time Sync Sequence Id and
 *      LTYPE register corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be read. (1-8)
        pTsLtype                Time sync LTYPE read.
        pTsSeqIdOffset          Time sync sequence Id offset read.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1,
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype, tsSeqIdOffset;

        portNum =   1;

        CSL_CPSW_getPortTimeSyncSeqIdReg (portNum, &tsLtype, &tsSeqIdOffset);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTimeSyncSeqIdReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pTsLtype,
    Uint32*                     pTsSeqIdOffset
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        *pTsLtype           =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_SEQ_LTYPE_REG,
                                          XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1);
        *pTsSeqIdOffset     =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_SEQ_LTYPE_REG,
                                          XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET);
    }

    return;
}

void CSL_CPSW_getVlanLTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32*                     pVlanLtypeInner,
    Uint32*                     pVlanLtypeOuter
)
{
	*pVlanLtypeInner  =   CSL_FEXT (hCpswRegs->VLAN_LTYPE_REG,
			                        XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_INNER);
	*pVlanLtypeOuter  =   CSL_FEXT (hCpswRegs->VLAN_LTYPE_REG,
			                        XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER);
    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setVlanLTypeReg
 *
 *   @b Description
 *   @n This function retreives the contents of VLAN_LTYPE_REG
 *      register.
 *
 *   @b Arguments
     @verbatim
        pVlanLtype1           VLAN LTYPE1 value read.
        pVlanLtype2           VLAN LTYPE2 value read.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_INNER,
 *      XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype1, tsLtype2;

        portNum =   1;

        CSL_CPSW_setVlanLTypeReg (portNum, &tsLtype1, &tsLtype2);
	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setVlanLTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                     pVlanLtypeInner,
    Uint32                     pVlanLtypeOuter
)
{
	CSL_FINS (hCpswRegs->VLAN_LTYPE_REG,
			  XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_INNER, pVlanLtypeInner);
	CSL_FINS (hCpswRegs->VLAN_LTYPE_REG,
			  XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER, pVlanLtypeOuter);
    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortTimeSyncSeqIdReg
 *
 *   @b Description
 *   @n This function sets up the contents of Time Sync Sequence Id and
 *      LTYPE register corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be
                                configured. (1-8)
        tsLtype                 Time sync LTYPE to be configured.
        tsSeqIdOffset           Time sync sequence Id offset to be configured.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1,
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype, tsSeqIdOffset;

        portNum         =   1;
        tsLtype         =   0;
        tsSeqIdOffset   =   30;

        CSL_CPSW_getPortTimeSyncSeqIdReg (portNum, tsLtype, tsSeqIdOffset);
	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTimeSyncSeqIdReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      tsLtype,
    Uint32                      tsSeqIdOffset
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_SEQ_LTYPE_REG,
                  XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1, tsLtype);

        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_SEQ_LTYPE_REG,
                  XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET, tsSeqIdOffset);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortTimeSyncVlanLTypeReg
 *
 *   @b Description
 *   @n This function retreives the contents of Time Sync VLAN LTYPE
 *      register corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be read. (1-8)
        pTsVlanLtype1           Time sync VLAN LTYPE1 value read.
        pTsVlanLtype2           Time sync VLAN LTYPE2 value read.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1,
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype1, tsLtype2;

        portNum =   1;

        CSL_CPSW_getPortTimeSyncVlanLTypeReg (portNum, &tsLtype1, &tsLtype2);
	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getPortTimeSyncVlanLTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pTsVlanLtype1,
    Uint32*                     pTsVlanLtype2
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        *pTsVlanLtype1  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_VLAN_LTYPE_REG,
                                      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1);
        *pTsVlanLtype2  =   CSL_FEXT (hCpswRegs->ENETPORT[portNum-1].PN_TS_VLAN_LTYPE_REG,
                                      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortTimeSyncVlanLTypeReg
 *
 *   @b Description
 *   @n This function sets up the contents of Time Sync VLAN LTYPE
 *      register corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the register must be read. (1-8)
        tsVlanLtype1            Time sync VLAN LTYPE1 value to be configured.
        tsVlanLtype2            Time sync VLAN LTYPE2 value to be configured.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1,
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum, tsLtype1, tsLtype2;

        portNum     =   1;
        tsLtype1    =   0x8100;
        tsLtype2    =   0x8100;

        CSL_CPSW_setPortTimeSyncVlanLTypeReg (portNum, &tsLtype1, &tsLtype2);
	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTimeSyncVlanLTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32                      tsVlanLtype1,
    Uint32                      tsVlanLtype2
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_VLAN_LTYPE_REG,
                  XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1, tsVlanLtype1);
        CSL_FINS (hCpswRegs->ENETPORT[portNum-1].PN_TS_VLAN_LTYPE_REG,
                  XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2, tsVlanLtype2);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getPortTimeSyncCntlReg
 *
 *   @b Description
 *   @n This function retreives the contents of Time sync configuration from
 *      time sync control registers corresponding to the CPSW port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be read. (1-8)
        pTimeSyncConfig         CSL_CPSW_TSCONFIG that needs to be populated with
                                contents of time sync control registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN,
 *
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1,
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET,
 *
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1,
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2,
 *
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_330,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN,
 *
 *      XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN,
 *      XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_TSCONFIG   tsConfig;

        portNum =   1;

        CSL_CPSW_getPortTimeSyncCntlReg (portNum, &tsConfig);

	 @endverbatim
 * =============================================================================
 */

void CSL_CPSW_getPortTimeSyncConfig (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    CSL_CPSW_TSCONFIG*      pTimeSyncConfig
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        Uint32 value;

        /* Time Sync Control Register */
        value = hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG;

        pTimeSyncConfig->tsRxAnnexDEnable          =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN);
        pTimeSyncConfig->tsRxAnnexEEnable          =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN);
        pTimeSyncConfig->tsRxAnnexFEnable          =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN);
        pTimeSyncConfig->tsRxVlanLType1Enable      =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN);
        pTimeSyncConfig->tsRxVlanLType2Enable      =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN);
        pTimeSyncConfig->tsTxAnnexDEnable          =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN);
        pTimeSyncConfig->tsTxAnnexEEnable          =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN);
        pTimeSyncConfig->tsTxAnnexFEnable          =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN);
        pTimeSyncConfig->tsTxVlanLType1Enable      =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN);
        pTimeSyncConfig->tsTxVlanLType2Enable      =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN);
        pTimeSyncConfig->tsMsgTypeEnable           =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN);
        pTimeSyncConfig->tsTxHostEnable            =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN);
        pTimeSyncConfig->tsLType2Enable            =   CSL_FEXT (value,
                                                                   XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN);

        /* Time Sync Sequence and LTYPE Register */
        value = hCpswRegs->ENETPORT[portNum-1].PN_TS_SEQ_LTYPE_REG;

        pTimeSyncConfig->tsLType1                  =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1);
        pTimeSyncConfig->tsSeqIdOffset             =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET);

        /* Time Sync VLAN Register */
        value = hCpswRegs->ENETPORT[portNum-1].PN_TS_VLAN_LTYPE_REG;

        pTimeSyncConfig->tsVlanLType1              =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1);
        pTimeSyncConfig->tsVlanLType2              =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2);

        /* Time Sync Control and LTYPE 2 Register */
        value = hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_LTYPE2_REG;

        pTimeSyncConfig->tsLType2                  =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2);
        pTimeSyncConfig->ts107Enable               =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107);

        pTimeSyncConfig->ts129Enable               =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129);

        pTimeSyncConfig->ts130Enable               =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130);

        pTimeSyncConfig->ts131Enable               =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131);

        pTimeSyncConfig->ts132Enable               =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132);

        pTimeSyncConfig->ts319Enable               =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319);

        pTimeSyncConfig->ts320Enable               =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_320);

        pTimeSyncConfig->tsTTLNonzeroEnable        =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO);

        pTimeSyncConfig->tsUniEnable               =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN);

        /* Time Sync Control 2 Register */
        value = hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL2_REG;

        pTimeSyncConfig->tsMcastTypeEnable         =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN);
        pTimeSyncConfig->tsDomainOffset            =   CSL_FEXT (value,
                                                                  XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET);


    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPortTimeSyncConfig
 *
 *   @b Description
 *   @n This function sets up the contents of Time sync control registers
 *      corresponding to the CPSW port number specified per user configuration.
 *
 *   @b Arguments
     @verbatim
        portNum                 CPSW port number for which the registers must be
                                configured.
        pTimeSyncConfig         CSL_CPSW_TSCONFIG containing settings for time
                                sync control registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN,
 *      XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN
 *
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1,
 *      XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET,
 *
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1,
 *      XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2,
 *
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_330,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO,
 *      XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN,
 *
 *      XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN,
 *      XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET
 *
 *   @b Example
 *   @verbatim
 *      Uint32              portNum;
        CSL_CPSW_TSCONFIG   tsConfig;

        portNum =   1;

        tsConfig.tsRxVlanLType1Enable   =   0;
        tsConfig.tsRxVlanLType2Enable   =   0;
        ...

        CSL_CPSW_setPortTimeSyncConfig (portNum, &tsConfig);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setPortTimeSyncConfig (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    CSL_CPSW_TSCONFIG*        pTimeSyncConfig
)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        Uint32 value = 0;

        /* Time Sync Control Register */
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN, pTimeSyncConfig->tsRxAnnexDEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN, pTimeSyncConfig->tsRxAnnexEEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN, pTimeSyncConfig->tsRxAnnexFEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN, pTimeSyncConfig->tsRxVlanLType1Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN, pTimeSyncConfig->tsRxVlanLType2Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN, pTimeSyncConfig->tsTxAnnexDEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN, pTimeSyncConfig->tsTxAnnexEEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN, pTimeSyncConfig->tsTxAnnexFEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN, pTimeSyncConfig->tsTxVlanLType1Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN, pTimeSyncConfig->tsTxVlanLType2Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN, pTimeSyncConfig->tsTxHostEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN, pTimeSyncConfig->tsLType2Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN, pTimeSyncConfig->tsMsgTypeEnable);
        hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_REG = value;


        /* Time Sync Sequence and LTYPE Register */
        value = 0;

        CSL_FINS (value, XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1, pTimeSyncConfig->tsLType1);
        CSL_FINS (value, XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET, pTimeSyncConfig->tsSeqIdOffset);
        hCpswRegs->ENETPORT[portNum-1].PN_TS_SEQ_LTYPE_REG = value;

        /* Time Sync VLAN Register */
        value = 0;

        CSL_FINS (value, XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1, pTimeSyncConfig->tsVlanLType1);
        CSL_FINS (value, XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2, pTimeSyncConfig->tsVlanLType2);
        hCpswRegs->ENETPORT[portNum-1].PN_TS_VLAN_LTYPE_REG = value;

        /* Time Sync Control and LTYPE 2 Register */
        value = 0;

        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2, pTimeSyncConfig->tsLType2);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107, pTimeSyncConfig->ts107Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129, pTimeSyncConfig->ts129Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130, pTimeSyncConfig->ts130Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131, pTimeSyncConfig->ts131Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132, pTimeSyncConfig->ts132Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319, pTimeSyncConfig->ts319Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_320, pTimeSyncConfig->ts320Enable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO, pTimeSyncConfig->tsTTLNonzeroEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN, pTimeSyncConfig->tsUniEnable);
        hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL_LTYPE2_REG = value;

        /* Time Sync Control 2 Register */
        value = 0;

        CSL_FINS (value, XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN, pTimeSyncConfig->tsMcastTypeEnable);
        CSL_FINS (value, XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET, pTimeSyncConfig->tsDomainOffset);
        hCpswRegs->ENETPORT[portNum-1].PN_TS_CTL2_REG = value;
    }

    return;
}

Uint8 CSL_CPSW_getEstTsDomain(CSL_Xge_cpswRegs   *hCpswRegs)
{
    return CSL_FEXT(hCpswRegs->EST_TS_DOMAIN_REG, XGE_CPSW_EST_TS_DOMAIN_REG_EST_TS_DOMAIN);
}

void CSL_CPSW_setEstTsDomain(CSL_Xge_cpswRegs    *hCpswRegs,
                             Uint8                domain)
{
    CSL_FINS(hCpswRegs->EST_TS_DOMAIN_REG, XGE_CPSW_EST_TS_DOMAIN_REG_EST_TS_DOMAIN, domain);
}

void CSL_CPSW_getPortEstConfig(CSL_Xge_cpswRegs    *hCpswRegs,
                               Uint32              portNum,
                               CSL_CPSW_EST_CONFIG *pEstConfig)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        pEstConfig->estOneBuf = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                                         XGE_CPSW_PN_EST_CONTROL_REG_EST_ONEBUF);

        pEstConfig->estBufSel = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                                         XGE_CPSW_PN_EST_CONTROL_REG_EST_BUFSEL);

        pEstConfig->estTsEnable = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                                           XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_EN);

        pEstConfig->estTsFirst = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                                          XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_FIRST);

        pEstConfig->estTsOnePri = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                                           XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_ONEPRI);

        pEstConfig->estTsPri = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                                        XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_PRI);

        pEstConfig->estFillEnable = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                                             XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_EN);

        /* FIXME: EstPremptComp field is not part of CSLR v4 */
        //pEstConfig->estPremptComp = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
        //                                     XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL);

        pEstConfig->estFillMargin = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                                             XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_MARGIN);
    }
}

void CSL_CPSW_setPortEstConfig(CSL_Xge_cpswRegs    *hCpswRegs,
                               Uint32              portNum,
                               CSL_CPSW_EST_CONFIG *pEstConfig)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                 XGE_CPSW_PN_EST_CONTROL_REG_EST_ONEBUF, pEstConfig->estOneBuf);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                 XGE_CPSW_PN_EST_CONTROL_REG_EST_BUFSEL, pEstConfig->estBufSel);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                 XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_EN, pEstConfig->estTsEnable);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                 XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_FIRST, pEstConfig->estTsFirst);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                 XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_ONEPRI, pEstConfig->estTsOnePri);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                 XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_PRI, pEstConfig->estTsPri);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                 XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_EN, pEstConfig->estFillEnable);

        // FIXME: EstPremptComp field is not part of CSLR v4
        //CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
        //         XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL, pEstConfig->estPremptComp);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_EST_CONTROL_REG,
                 XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_MARGIN, pEstConfig->estFillMargin);
    }
}

void CSL_CPSW_writeEstFetchCmd(CSL_Xge_cpswRegs    *hCpswRegs,
                               Uint32              portNum,
                               Uint32              index,
                               Uint32              fetchCount,
                               Uint8               fetchAllow)
{
    Uint32 fetchVal;

    if ((portNum >= 1) && (portNum <= CSL_ARRAYSIZE(hCpswRegs->CPSW_NU_EST)))
    {
        fetchVal = (((fetchCount << CPSW_EST_FETCH_COUNT_SHIFT) & CPSW_EST_FETCH_COUNT_MASK) |
                    ((fetchAllow << CPSW_EST_FETCH_ALLOW_SHIFT) & CPSW_EST_FETCH_ALLOW_MASK));

        CSL_REG_WR(&hCpswRegs->CPSW_NU_EST[portNum-1].FETCH_LOC[index], fetchVal);
    }
}

void CSL_CPSW_readEstFetchCmd(CSL_Xge_cpswRegs    *hCpswRegs,
                              Uint32              portNum,
                              Uint32              index,
                              Uint32              *fetchCount,
                              Uint8               *fetchAllow)
{
    Uint32 fetchVal;

    if ((portNum >= 1) && (portNum <= CSL_ARRAYSIZE(hCpswRegs->CPSW_NU_EST)))
    {
        fetchVal = CSL_REG_RD(&hCpswRegs->CPSW_NU_EST[portNum-1].FETCH_LOC[index]);

        *fetchCount = (fetchVal & CPSW_EST_FETCH_COUNT_MASK) >> CPSW_EST_FETCH_COUNT_SHIFT;
        *fetchAllow = (fetchVal & CPSW_EST_FETCH_ALLOW_MASK) >> CPSW_EST_FETCH_ALLOW_SHIFT;
    }
}

#if ENET_CFG_IS_ON(CPSW_IET_INCL)
void CSL_CPSW_getPortIetControlReg(CSL_Xge_cpswRegs    *hCpswRegs,
                                   Uint32              portNum,
                                   CSL_CPSW_IET_CONFIG *pIetConfig)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        pIetConfig->macPremptQueue = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                                              XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT);

        pIetConfig->macAddFragSize = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                                              XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE);

        pIetConfig->macLinkFail = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                                           XGE_CPSW_PN_IET_CONTROL_REG_MAC_LINKFAIL);

        pIetConfig->macDisableVerify = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                                                XGE_CPSW_PN_IET_CONTROL_REG_MAC_DISABLEVERIFY);

        pIetConfig->macHold = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                                       XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD);

        pIetConfig->macPremptEnable = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                                               XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE);
    }
}

void CSL_CPSW_setPortIetControlReg(CSL_Xge_cpswRegs    *hCpswRegs,
                                   Uint32              portNum,
                                   CSL_CPSW_IET_CONFIG *pIetConfig)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT, pIetConfig->macPremptQueue);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE, pIetConfig->macAddFragSize);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_LINKFAIL, pIetConfig->macLinkFail);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_DISABLEVERIFY, pIetConfig->macDisableVerify);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD, pIetConfig->macHold);

        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_IET_CONTROL_REG,
                 XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE, pIetConfig->macPremptEnable);
    }
}

void CSL_CPSW_getPortIetVerifyTimeout(CSL_Xge_cpswRegs    *hCpswRegs,
                                      Uint32              portNum,
                                      Uint32              *pIetVerifyTimeout)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        *pIetVerifyTimeout = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_VERIFY_REG,
                                      XGE_CPSW_PN_IET_VERIFY_REG_MAC_VERIFY_CNT);
    }

}

void CSL_CPSW_setPortIetVerifyTimeout(CSL_Xge_cpswRegs    *hCpswRegs,
                                      Uint32              portNum,
                                      Uint32              ietVerifyTimeout)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        CSL_FINS(hCpswRegs->ENETPORT[portNum-1].PN_IET_VERIFY_REG,
                 XGE_CPSW_PN_IET_VERIFY_REG_MAC_VERIFY_CNT, ietVerifyTimeout);
    }
}

void CSL_CPSW_PortIetStatus(CSL_Xge_cpswRegs     *hCpswRegs,
                            Uint32               portNum,
                            CSL_CPSW_IET_STATUS  *pIetStatus)
{
    if ((portNum >= 1) &&  (portNum <= CSL_ARRAYSIZE(hCpswRegs->ENETPORT)))
    {
        pIetStatus->macVerified = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_STATUS_REG,
                                           XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFIED);
        pIetStatus->macVerifyFail = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_STATUS_REG,
                                             XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_FAIL);
        pIetStatus->macRxRespondErr = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_STATUS_REG,
                                               XGE_CPSW_PN_IET_STATUS_REG_MAC_RESPOND_ERR);
        pIetStatus->macRxVerifyErr = CSL_FEXT(hCpswRegs->ENETPORT[portNum-1].PN_IET_STATUS_REG,
                                              XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_ERR);
    }
}
#endif

/********************************************************************************
*************************  Statistics (STATS) Submodule *************************
********************************************************************************/

/** ============================================================================
 *   @n@b CSL_CPSW_getStats
 *
 *   @b Description
 *   @n The CPSW stats are divided into 9 blocks, i.e., Stats for Host port (switch Port 0)
 *      and Stats for CPSW ports (Port 1-8 ). This function
 *          - retreives hardware statistics for both the stat blocks.
 *          - Clear out the stats by the count being returned to application
 *          - Accumulates the stats count before returning to Application
 *      Function requires appplication to memset the stats (once before first
 *            use for accumulator, or once per use without accumulation)
 *      In the case of Linux ARM master use case all CPSW stats is recommended
 *      to be accessed from Linux.
 *      This function unconditionally clears the stats, so it requires the
 *      caller have exclusive ownership of the switch.  Otherwise, none of the
 *      callers (including Linux) will have complete accumulated stats.
 *      This function can be used to clear the stats by memesetting pCpswStats
 *      to 0 and discarding the returned stats.
 *
 *
 *   @b Arguments
     @verbatim
        pCpswStats              Union of CSL_CPSW_STATS structure that needs to be filled
                                with the stats read from the hardware. This function expects
                                that the array passed to it is big enough to hold the stats
                                for all stat blocks, i.e., size of array passed to this
                                function must be 5 or 9 for 5/9 port switch respectively.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSW_RXGOODFRAMES,
 *      CPSW_RXBROADCASTFRAMES,
 *      CPSW_RXMULTICASTFRAMES,
 *      CPSW_RXPAUSEFRAMES,
 *      CPSW_RXCRCERRORS,
 *      CPSW_RXALIGNCODEERRORS,
 *      CPSW_RXOVERSIZEDFRAMES,
 *      CPSW_RXJABBERFRAMES,
 *      CPSW_RXUNDERSIZEDFRAMES,
 *      CPSW_RXFRAGMENTS,
 *      CPSW_ALE_DROP,
 *      CPSW_ALE_OVERRUN_DROP,
 *      CPSW_RXOCTETS,
 *      CPSW_TXGOODFRAMES,
 *      CPSW_TXBROADCASTFRAMES,
 *      CPSW_TXMULTICASTFRAMES,
 *      CPSW_TXPAUSEFRAMES,
 *      CPSW_TXDEFERREDFRAMES,
 *      CPSW_TXCOLLISIONFRAMES,
 *      CPSW_TXSINGLECOLLFRAMES,
 *      CPSW_TXMULTCOLLFRAMES,
 *      CPSW_TXEXCESSIVECOLLISIONS,
 *      CPSW_TXLATECOLLISIONS,
 *      CPSW_TXUNDERRUN,
 *      CPSW_TXCARRIERSENSEERRORS,
 *      CPSW_TXOCTETS,
 *      CPSW_OCTETFRAMES64,
 *      CPSW_OCTETFRAMES65T127,
 *      CPSW_OCTETFRAMES128T255,
 *      CPSW_OCTETFRAMES256T511,
 *      CPSW_OCTETFRAMES512T1023,
 *      CPSW_OCTETFRAMES1024TUP,
 *      CPSW_NETOCTETS,
 *      CPSW_RX_BOTTOM_OF_FIFO_DROP,
 *      CPSW_PORTMASK_DROP,
 *      CPSW_RX_TOP_OF_FIFO_DROP,
 *      CPSW_ALE_ALE_RATE_LIMIT_DROP,
 *      CPSW_ALE_VID_INGRESS_DROP,
 *      CPSW_ALE_DA_EQ_SA_DROP,
 *      CPSW_ALE_UNKN_UNI,
 *      CPSW_ALE_UNKN_UNI_BCNT,
 *      CPSW_ALE_UNKN_MLT,
 *      CPSW_ALE_UNKN_MLT_BCNT,
 *      CPSW_ALE_UNKN_BRD,
 *      CPSW_ALE_UNKN_BRD_BCNT,
 *      CPSW_ALE_POLL_MATCH,
 *      CPSW_TX_MEMORY_PROTECT_ERROR
 *
 *   @b Affects
 *   @n CPSW_RXGOODFRAMES=0,
 *      CPSW_RXBROADCASTFRAMES=0,
 *      CPSW_RXMULTICASTFRAMES=0,
 *      CPSW_RXPAUSEFRAMES=0,
 *      CPSW_RXCRCERRORS=0,
 *      CPSW_RXALIGNCODEERRORS=0,
 *      CPSW_RXOVERSIZEDFRAMES=0,
 *      CPSW_RXJABBERFRAMES=0,
 *      CPSW_RXUNDERSIZEDFRAMES=0,
 *      CPSW_RXFRAGMENTS=0,
 *      CPSW_ALE_DROP=0,
 *      CPSW_ALE_OVERRUN_DROP=0,
 *      CPSW_RXOCTETS=0,
 *      CPSW_TXGOODFRAMES=0,
 *      CPSW_TXBROADCASTFRAMES=0,
 *      CPSW_TXMULTICASTFRAMES=0,
 *      CPSW_TXPAUSEFRAMES=0,
 *      CPSW_TXDEFERREDFRAMES=0,
 *      CPSW_TXCOLLISIONFRAMES=0,
 *      CPSW_TXSINGLECOLLFRAMES=0,
 *      CPSW_TXMULTCOLLFRAMES=0,
 *      CPSW_TXEXCESSIVECOLLISIONS=0,
 *      CPSW_TXLATECOLLISIONS=0,
 *      CPSW_TXUNDERRUN=0,
 *      CPSW_TXCARRIERSENSEERRORS=0,
 *      CPSW_TXOCTETS=0,
 *      CPSW_OCTETFRAMES64=0,
 *      CPSW_OCTETFRAMES65T127=0,
 *      CPSW_OCTETFRAMES128T255=0,
 *      CPSW_OCTETFRAMES256T511=0,
 *      CPSW_OCTETFRAMES512T1023=0,
 *      CPSW_OCTETFRAMES1024TUP=0,
 *      CPSW_NETOCTETS=0,
 *      CPSW_RX_BOTTOM_OF_FIFO_DROP=0,
 *      CPSW_PORTMASK_DROP=0,
 *      CPSW_RX_TOP_OF_FIFO_DROP=0,
 *      CPSW_ALE_ALE_RATE_LIMIT_DROP=0,
 *      CPSW_ALE_VID_INGRESS_DROP=0,
 *      CPSW_ALE_DA_EQ_SA_DROP=0,
 *      CPSW_ALE_UNKN_UNI=0,
 *      CPSW_ALE_UNKN_UNI_BCNT=0,
 *      CPSW_ALE_UNKN_MLT=0,
 *      CPSW_ALE_UNKN_MLT_BCNT=0,
 *      CPSW_ALE_UNKN_BRD=0,
 *      CPSW_ALE_UNKN_BRD_BCNT=0,
 *      CPSW_ALE_POLL_MATCH=0,
 *      CPSW_TX_MEMORY_PROTECT_ERROR=0
 *
 *   @b Example
 *   @verbatim
 *      union CSL_CPSW_STATS {
            CSL_CPSW_P0_STATS p0_stats;
            CSL_CPSW_PN_STATS pn_stats;
        };

        CSL_CPSW_getStats (stats);
	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getStats (CSL_Xge_cpswRegs *hCpswRegs,
                        union CSL_CPSW_STATS* pCpswStats
)
{
    Uint32                      numStats, numBlocks;
    volatile Uint32             *pRegAddr;
    Uint32                      *pStatAddr;
    Uint32                      statval;

    pStatAddr   =   (Uint32 *)(pCpswStats);

    /* Read the entire CPSW HW stats block for the Host port, save it in pCpswStats reg and
     * reset the stats block when done.
     */
    pRegAddr    =	CSL_CPSW_GET_HOSTPORT_STAT_START_ADDRESS(hCpswRegs);
    for (numStats = 0; numStats < sizeof(pCpswStats->p0_stats)/sizeof(Uint32); numStats++)
    {
        statval         =   *pRegAddr;
        *pRegAddr++     =   statval;
        statval         +=  *pStatAddr;
        *pStatAddr++    =   statval;
    }

    /* Read the entire CPSW HW stats block for the MAC ports,  save it in pCpswStats reg and
     * reset the stats block when done.
     */
    for (numBlocks = 0; numBlocks < sizeof(pCpswStats->pn_stats)/sizeof(hCpswRegs->PN_STATS[0]);
         numBlocks++)
    {
	    pRegAddr    =	CSL_CPSW_GET_MACPORT_STAT_START_ADDRESS(hCpswRegs, numBlocks);
        for (numStats = 0; numStats < sizeof(pCpswStats->pn_stats)/sizeof(Uint32); numStats++)
        {
            statval         =   *pRegAddr;
            *pRegAddr++     =   statval;
            statval         +=  *pStatAddr;
            *pStatAddr++    =   statval;
        }
    }

    return;
}

void CSL_CPSW_getPortStats (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    union CSL_CPSW_STATS*   pCpswStats
)
{
    Uint32                      numStats;
    volatile Uint32             *pRegAddr;
    Uint32                      *pStatAddr;
    Uint32                      statval;

    pStatAddr   =   (Uint32 *)(pCpswStats);

    if(portNum == 0)
    {
        pRegAddr    =	CSL_CPSW_GET_HOSTPORT_STAT_START_ADDRESS(hCpswRegs);
    }
    else
    {
        pRegAddr    =	CSL_CPSW_GET_MACPORT_STAT_START_ADDRESS(hCpswRegs, portNum - 1);
    }

    /* Read the entire CPSW HW stats block for the Host/Mac port, save it in pCpswStats reg and
     * reset the stats block when done.
     */
    for (numStats = 0; numStats < sizeof(union CSL_CPSW_STATS)/sizeof(Uint32); numStats++)
    {
    	statval         =   *pRegAddr;
    	*pRegAddr++     =   statval;
    	*pStatAddr++    =   statval;
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getRawStats
 *
 *   @b Description
 *   @n The CPSW stats are divided into 9 blocks, i.e., Stats for Host port (switch Port 0)
 *      and Stats for MAC ports (Port 1-8). This function retreives snapshot of
 *      hardware statistics for all the stat blocks. In the case of Linux ARM master
 *      use case all CPSW stats is recommended to be accessed from Linux.

 *      Since this function does not clear the stats, its possible to have
 *      non-exclusive ownership of the switch and use this function without
 *      corrupting other caller's view of the stats.
 *
 *      Additional Note: In order to avoid stats loss due to rollovers, application
 *      would need to poll the stats by determining the correct interval.
 *      The stat CPSW_NETOCTETS would be first one to roll over
 *      The software must poll and accumulate the stats faster than this rate.
 *      On a 1 gigabit network, it takes approximately
 *      (0x100000000/(1000000000/8)/2)=17 seconds to roll over (the /2 is because
 *      this stat contains both tx and rx, both of which run at gigabit).
 *      A good rule of thumb is to poll at twice this rate (8-9 seconds).
 *
 *      If it is really necessary for application to have multiple nonexclusive
 *      owners of the switch, it is possible for all callers to have a view of
 *      the accumulated statistics if they (including Linux) follows the
 *      differential accumulation of the stats defiened below:
 *      uint64_t accum_CPSW_NETOCTETS;
 *      uint32_t old_CPSW_NETOCTETS, new_CPSW_NETOCTETS, diff_CPSW_NETOCTETS;
 *
 *       diff_CPSW_NETOCTETS = new_CPSW_NETOCTETS - old_CPSW_NETOCTETS; // let rollover occur, no "if" required
 *       old_CPSW_NETOCTETS = new_CPSW_NETOCTETS;
 *       accum_CPSW_NETOCTETS += diff_CPSW_NETOCTETS
 *
 *   @b Arguments
     @verbatim
        pCpswStats              Union of CSL_CPSW_STATS structure that needs to be filled
                                with the stats read from the hardware. This function expects
                                that the array passed to it is big enough to hold the stats
                                for both stat blocks, i.e., size of array passed to this
                                function must be 5 or 9 for 5-port/9-port switch respectively.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSW_RXGOODFRAMES,
 *      CPSW_RXBROADCASTFRAMES,
 *      CPSW_RXMULTICASTFRAMES,
 *      CPSW_RXPAUSEFRAMES,
 *      CPSW_RXCRCERRORS,
 *      CPSW_RXALIGNCODEERRORS,
 *      CPSW_RXOVERSIZEDFRAMES,
 *      CPSW_RXJABBERFRAMES,
 *      CPSW_RXUNDERSIZEDFRAMES,
 *      CPSW_RXFRAGMENTS,
 *      CPSW_ALE_DROP,
 *      CPSW_ALE_OVERRUN_DROP,
 *      CPSW_RXOCTETS,
 *      CPSW_TXGOODFRAMES,
 *      CPSW_TXBROADCASTFRAMES,
 *      CPSW_TXMULTICASTFRAMES,
 *      CPSW_TXPAUSEFRAMES,
 *      CPSW_TXDEFERREDFRAMES,
 *      CPSW_TXCOLLISIONFRAMES,
 *      CPSW_TXSINGLECOLLFRAMES,
 *      CPSW_TXMULTCOLLFRAMES,
 *      CPSW_TXEXCESSIVECOLLISIONS,
 *      CPSW_TXLATECOLLISIONS,
 *      CPSW_TXUNDERRUN,
 *      CPSW_TXCARRIERSENSEERRORS,
 *      CPSW_TXOCTETS,
 *      CPSW_OCTETFRAMES64,
 *      CPSW_OCTETFRAMES65T127,
 *      CPSW_OCTETFRAMES128T255,
 *      CPSW_OCTETFRAMES256T511,
 *      CPSW_OCTETFRAMES512T1023,
 *      CPSW_OCTETFRAMES1024TUP,
 *      CPSW_NETOCTETS,
 *      CPSW_RX_BOTTOM_OF_FIFO_DROP,
 *      CPSW_PORTMASK_DROP,
 *      CPSW_RX_TOP_OF_FIFO_DROP,
 *      CPSW_ALE_ALE_RATE_LIMIT_DROP,
 *      CPSW_ALE_VID_INGRESS_DROP,
 *      CPSW_ALE_DA_EQ_SA_DROP,
 *      CPSW_ALE_UNKN_UNI,
 *      CPSW_ALE_UNKN_UNI_BCNT,
 *      CPSW_ALE_UNKN_MLT,
 *      CPSW_ALE_UNKN_MLT_BCNT,
 *      CPSW_ALE_UNKN_BRD,
 *      CPSW_ALE_UNKN_BRD_BCNT,
 *      CPSW_ALE_POLL_MATCH,
 *      CPSW_TX_MEMORY_PROTECT_ERROR
 *
 *   @b Example
 *   @verbatim
 *      CSL_CPSW_STATS     stats [9];

        CSL_CPSW_getRawStats (stats);
	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getRawStats (CSL_Xge_cpswRegs *hCpswRegs,
                           union CSL_CPSW_STATS* pCpswStats
)
{
    Uint32                      numStats, numBlocks;
    volatile Uint32             *pRegAddr;
    Uint32                      *pStatAddr;

    pStatAddr   =   (Uint32 *)(pCpswStats);

    /* Read the entire CPSW HW stats block for the Hostport, save it in pCpswStats reg and
     * reset the stats block when done.
     */
    pRegAddr    =	CSL_CPSW_GET_HOSTPORT_STAT_START_ADDRESS(hCpswRegs);
    for (numStats = 0; numStats < sizeof(union CSL_CPSW_STATS)/sizeof(Uint32); numStats++)
    {
        *pStatAddr++    =   *pRegAddr++;
    }

    /* Read the entire CPSW HW stats block for the Macport, save it in pCpswStats reg and
     * reset the stats block when done.
     */
    for (numBlocks = 0; numBlocks < sizeof(hCpswRegs->PN_STATS)/sizeof(hCpswRegs->PN_STATS[0]); numBlocks++)
    {
        pRegAddr    =	CSL_CPSW_GET_MACPORT_STAT_START_ADDRESS(hCpswRegs, numBlocks);
        for (numStats = 0; numStats < sizeof(union CSL_CPSW_STATS)/sizeof(Uint32); numStats++)
        {
            *pStatAddr++    =   *pRegAddr++;
        }
    }

    return;
}

void CSL_CPSW_getPortRawStats (CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    union CSL_CPSW_STATS*         pCpswStats
)
{
    Uint32                      numStats;
    volatile Uint32             *pRegAddr;
    Uint32                      *pStatAddr;

    pStatAddr   =   (Uint32 *)(pCpswStats);

    if(portNum == 0)
    {
        pRegAddr    =	CSL_CPSW_GET_HOSTPORT_STAT_START_ADDRESS(hCpswRegs);
    }
    else
    {
        pRegAddr    =	CSL_CPSW_GET_MACPORT_STAT_START_ADDRESS(hCpswRegs, portNum - 1);
    }

    /* Read the entire CPSW HW stats block for the Host/Mac port, save it in pCpswStats reg and
     * reset the stats block when done.
     */
    for (numStats = 0; numStats < sizeof(union CSL_CPSW_STATS)/sizeof(Uint32); numStats++)
	{
		*pStatAddr++    =   *pRegAddr++;
	}

    return;
}


/********************************************************************************
********************  Address Lookup Engine (ALE) Submodule *********************
********************************************************************************/

/** ============================================================================
 *   @n@b CSL_CPSW_getAleVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the ALE submodule identification and version
 *      information.
 *
 *   @b Arguments
     @verbatim
        pVersionInfo        CSL_CPSW_ALE_VERSION structure that needs to be populated
                            with the ALE version info read from the hardware.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_MOD_VER_MINOR_REVISION,
 *      ALE_MOD_VER_MAJOR_REVISION,
 *      ALE_MOD_VER_RTL_VERSION,
 *      ALE_MOD_VER_MODULE_ID
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_ALE_VERSION    versionInfo;

        CSL_CPSW_getAleVersionInfo (&versionInfo);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleVersionInfo (CSL_AleRegs *hCpswAleRegs,
	CSL_CPSW_ALE_VERSION*       pVersionInfo
)
{

    pVersionInfo->minorVer  =   CSL_FEXT (hCpswAleRegs->MOD_VER, ALE_MOD_VER_MINOR_REVISION);
    pVersionInfo->majorVer  =   CSL_FEXT (hCpswAleRegs->MOD_VER, ALE_MOD_VER_MAJOR_REVISION);
    pVersionInfo->rtlVer    =   CSL_FEXT (hCpswAleRegs->MOD_VER, ALE_MOD_VER_RTL_VERSION);
    pVersionInfo->id        =   CSL_FEXT (hCpswAleRegs->MOD_VER, ALE_MOD_VER_MODULE_ID);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleRateLimitEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE Broadcast and Multicast Rate Limit is
 *      enabled.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE Broadcast and multicast rate limit enabled.
 *	                            Broadcast/multicast packet reception limited to
 *	                            port control register rate limit fields.
 *	 @n  FALSE                  ALE Broadcast and multicast rate limit disabled.
 *	                            Broadcast/multicast rates not limited.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_RATE_LIMIT
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleRateLimitEnabled () == TRUE)
        {
            // ALE Broadcast/Multicast rate limit enabled
        }
        else
        {
            // ALE Broadcast/Multicast rate limit disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleRateLimitEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_RATE_LIMIT);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleRateLimit
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable multicast,
 *      broadcast rate limiting.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_RATE_LIMIT=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleRateLimit ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleRateLimit (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_RATE_LIMIT, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleRateLimit
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable multicast,
 *      broadcast rate limiting.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_RATE_LIMIT=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleRateLimit ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleRateLimit (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_RATE_LIMIT, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleMacAuthModeEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE MAC Authorization mode is enabled.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE  is in MAC authorization mode.
 *	 @n  FALSE                  ALE not in MAC authorization mode.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_AUTH_MODE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleMacAuthModeEnabled () == TRUE)
        {
            // ALE  is in MAC authorization mode
        }
        else
        {
            // ALE not in MAC authorization mode
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleMacAuthModeEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_AUTH_MODE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleMacAuthMode
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable MAC authorization
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_AUTH_MODE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleMacAuthMode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleMacAuthMode (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_AUTH_MODE, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleMacAuthMode
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable MAC authorization
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_AUTH_MODE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleMacAuthMode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleMacAuthMode (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_AUTH_MODE, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleVlanAwareEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be VLAN aware.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE VLAN aware. ALE drops packets if VLAN not found.
 *	 @n  FALSE                  ALE not VLAN aware. Floods if VLAN not found.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ALE_VLAN_AWARE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleVlanAwareEnabled () == TRUE)
        {
            // ALE VLAN aware
        }
        else
        {
            // ALE not VLAN aware
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleVlanAwareEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ALE_VLAN_AWARE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleVlanAware
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable VLAN aware
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ALE_VLAN_AWARE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleVlanAware ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleVlanAware (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ALE_VLAN_AWARE, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleVlanAware
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable VLAN aware
 *      mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ALE_VLAN_AWARE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleVlanAware ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleVlanAware (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ALE_VLAN_AWARE, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleTxRateLimitEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be Tx-port based multicast,
 *      broadcast rate limited.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE Tx rate limit enabled. Broadcast, multicast
 *	                            rate limit counters are transmit port based.
 *	 @n  FALSE                  ALE Tx rate limit disabled. Broadcast, multicast
 *	                            rate limit counters are receive port based.
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_RATE_LIMIT=1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_BCAST_MCAST_CTL
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleTxRateLimitEnabled () == TRUE)
        {
            // ALE Tx rate limit on
        }
        else
        {
            // ALE Tx rate limit off
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleTxRateLimitEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_BCAST_MCAST_CTL);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleTxRateLimit
 *
 *   @b Description
 *   @n This function configures the ALE control register to select Tx-port based
 *      multicast, broadcast rate limiting
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_RATE_LIMIT=1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_BCAST_MCAST_CTL=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleTxRateLimit ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleTxRateLimit (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_BCAST_MCAST_CTL, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleTxRateLimit
 *
 *   @b Description
 *   @n This function configures the ALE control register to select Rx-port based
 *      multicast, broadcast rate limiting
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_BCAST_MCAST_CTL=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleTxRateLimit ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleTxRateLimit (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_BCAST_MCAST_CTL, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleBypassEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be in Bypass mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE Bypass mode enabled.
 *	 @n  FALSE                  ALE Bypass mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_BYPASS
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleBypassEnabled () == TRUE)
        {
            // ALE Bypass mode on
        }
        else
        {
            // ALE Bypass mode off
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleBypassEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_BYPASS);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleBypass
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable Bypass mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_BYPASS=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleBypass ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleBypass (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_BYPASS, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleBypass
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable Bypass mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_BYPASS=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleBypass ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleBypass (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_BYPASS, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleOUIDenyModeEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be in OUI deny mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE OUI deny mode enabled.
 *	 @n  FALSE                  ALE OUI deny mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_OUI_DENY
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleOUIDenyModeEnabled () == TRUE)
        {
            // ALE OUI deny mode on
        }
        else
        {
            // ALE OUI deny mode off
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleOUIDenyModeEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_OUI_DENY);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleOUIDenyMode
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable OUI deny mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_OUI_DENY=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleOUIDenyMode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleOUIDenyMode (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_OUI_DENY, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleOUIDenyMode
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable OUI deny mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_OUI_DENY=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleOUIDenyMode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleOUIDenyMode (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_OUI_DENY, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleVID0ModeEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to be in VID0 (VLAN ID=0) mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE VID0 mode enabled.
 *	                            Process the packet with VLAN Id = 0
 *	 @n  FALSE                  ALE VID0 mode disabled. Process the packet with
 *	                            VLAN Id =PORT_VLAN[11-0]
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_VID0_MODE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleVID0ModeEnabled () == TRUE)
        {
            // ALE VID0 mode on
        }
        else
        {
            // ALE VID0 mode off
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleVID0ModeEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_VID0_MODE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleVID0Mode
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable VID0 mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_VID0_MODE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleVID0Mode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleVID0Mode (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_VID0_MODE, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleVID0Mode
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable VID0 mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_VID0_MODE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleVID0Mode ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleVID0Mode (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_VID0_MODE, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleLearnNoVIDEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to not learn VLAN Ids.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE Learn no VID enabled.
 *	                            VLAN Id is not learned with source address (source
 *	                            address is not tied to VID)
 *	 @n  FALSE                  ALE VID learning mode enabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_LEARN_NO_VLANID
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleLearnNoVIDEnabled () == TRUE)
        {
            // ALE VID learning disabled
        }
        else
        {
            // ALE VID learning enabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleLearnNoVIDEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_LEARN_NO_VLANID);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleLearnNoVID
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable VLAN Id No
 *      Learn, i.e., disable VLAN Id learning.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_LEARN_NO_VLANID=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleLearnNoVID ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleLearnNoVID (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_LEARN_NO_VLANID, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleLearnNoVID
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable VLAN Id learning.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_LEARN_NO_VLANID=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleLearnNoVID ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleLearnNoVID (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_LEARN_NO_VLANID, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleUUNIToHostEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to forward unkown unicast
 *      packets to host.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Unknown unicast packets flood to host also.
 *	 @n  FALSE                  Unknown unicast packets are dropped to the host.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleUUNIToHostEnabled () == TRUE)
        {
            // ALE Unknown UNI packets forwarded to host
        }
        else
        {
            // ALE Unknown UNI packets dropped to host
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleUUNIToHostEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleUUNIToHost
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable forwarding
 *      unkown unicast packets to host.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleUUNIToHost ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleUUNIToHost (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleUUNIToHost
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable forwarding
 *      unkown unicast packets to host.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleUUNIToHost ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleUUNIToHost (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_EN_HOST_UNI_FLOOD, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleUVLANNoLearnEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE is programmed to disable learning of the
 *      packets with unknown VLAN.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Unknown VLAN No Learn enabled.
 *                              Source addresses of unknown VLANIDs are not added into
 *                              the look up table even if learning is enabled.
 *	 @n  FALSE                  Unknown VLAN No Learn disabled.
 *                              Source addresses of unknown VLANIDs are added into
 *                              the look up table if learning is enabled.
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_ALE = 1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n 	ALE_ALE_CONTROL_UVLAN_NO_LEARN
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleUVLANNoLearnEnabled () == TRUE)
        {
            // Unknown VLAN No Learn disabled
        }
        else
        {
            // Unknown VLAN No Learn enabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleUVLANNoLearnEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, 	ALE_ALE_CONTROL_UVLAN_NO_LEARN);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAleUVLANNoLearn
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable
 *      Unknown VLAN No Learn mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_ALE = 1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n 	ALE_ALE_CONTROL_UVLAN_NO_LEARN=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAleUVLANNoLearn ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAleUVLANNoLearn (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, 	ALE_ALE_CONTROL_UVLAN_NO_LEARN, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAleUVLANNoLearn
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable
 *      unknown VLAN No Learn mode.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  ALE_ALE_CONTROL_ENABLE_ALE = 1
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n 	ALE_ALE_CONTROL_UVLAN_NO_LEARN=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAleUVLANNoLearn ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAleUVLANNoLearn (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, 	ALE_ALE_CONTROL_UVLAN_NO_LEARN, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAleUpdateBW
 *
 *   @b Description
 *   @n This function extracts the ALE Update Bandwidth of the ALE control register
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 aleUpdBW               ALE Update Bandwidth
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n 	ALE_ALE_CONTROL_UPD_BW_CTRL
 *
 *   @b Example
 *   @verbatim

        Uint32  aleUpdBW;

        aleUpdBW = CSL_CPSW_getAleUpdateBW();

	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getAleUpdateBW (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_UPD_BW_CTRL);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleUpdateBW
 *
 *   @b Description
 *   @n This function configures the ALE Update Bandwidth of the ALE control
 *      register.
 *
 *   @b Arguments
 *   @n aleUpdBW               ALE Update Bandwidth
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n 	ALE_ALE_CONTROL_UPD_BW_CTRL
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_setAleUpdateBW ((Uint32)ALE_UPD_BW_350MHZ_5M);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUpdateBW
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUpdBW
)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, 	ALE_ALE_CONTROL_UPD_BW_CTRL, aleUpdBW);

    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_startAleAgeOutNow
 *
 *   @b Description
 *   @n This function configures the ALE control register to initiate an ALE
 *      ageable entry cleanup. This enables the ALE hardware to remove any
 *      ageable table entry that does not have a set touch bit.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_AGE_OUT_NOW=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_startAleAgeOutNow ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_startAleAgeOutNow (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_AGE_OUT_NOW, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleAgeOutDone
 *
 *   @b Description
 *   @n This function reads the ALE control register's AGE_OUT_NOW bit to check
 *      if the ALE ageable entry cleanup process is done.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE age out process done.
 *	 @n  FALSE                  ALE age out process not yet completed.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_AGE_OUT_NOW
 *
 *   @b Example
 *   @verbatim

        if (CSL_CPSW_isAleAgeOutDone ();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleAgeOutDone (CSL_AleRegs *hCpswAleRegs)
{
    if ((CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_AGE_OUT_NOW) == 0))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/** ============================================================================
 *   @n@b CSL_CPSW_clearAleTable
 *
 *   @b Description
 *   @n This function initiates a full ALE table cleanup. The ALE hardware
 *      clears all table entries.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_CLEAR_TABLE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_clearAleTable ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_clearAleTable (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_CLEAR_TABLE, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_isAleEnabled
 *
 *   @b Description
 *   @n This function indicates if ALE processing is enabled.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   ALE enabled. ALE packet processing will be done.
 *	 @n  FALSE                  ALE disabled. All packets are dropped by ALE.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_CONTROL_ENABLE_ALE
 *
 *   @b Example
 *   @verbatim
        if (CSL_CPSW_isAleEnabled () == TRUE)
        {
            // ALE enabled
        }
        else
        {
            // ALE disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_isAleEnabled (CSL_AleRegs *hCpswAleRegs)
{

    return CSL_FEXT (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_ALE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableAle
 *
 *   @b Description
 *   @n This function configures the ALE control register to enable ALE processing.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_ALE=1
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_enableAle ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_enableAle (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_ALE, (Uint32) 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableAle
 *
 *   @b Description
 *   @n This function configures the ALE control register to disable ALE processing.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_CONTROL_ENABLE_ALE=0
 *
 *   @b Example
 *   @verbatim

        CSL_CPSW_disableAle ();

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_disableAle (CSL_AleRegs *hCpswAleRegs)
{
    CSL_FINS (hCpswAleRegs->ALE_CONTROL, ALE_ALE_CONTROL_ENABLE_ALE, 0);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAleControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE control register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 >=0                ALE control register contents.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_CONTROL_REG
 *
 *   @b Example
 *   @verbatim
        Uint32      aleCtrlVal;

        aleCtrlVal  =   CSL_CPSW_getAleControlReg ();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getAleControlReg (CSL_AleRegs *hCpswAleRegs)
{
    return hCpswAleRegs->ALE_CONTROL;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE control register.
 *
 *   @b Arguments
     @verbatim
        aleCtrlVal          Value to be configured to the ALE control register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_CONTROL_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32          aleCtrlVal = 0;

        aleCtrlVal      =   CSL_CPSW_getAleControlReg ();
        aleCtrlVal      |=  CSL_XGE_CPSW_ALECONTROL_CLRTABLE_EN;

        CSL_CPSW_setAleControlReg (&aleCtrlRegInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleControlReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleCtrlVal
)
{
    hCpswAleRegs->ALE_CONTROL  =   aleCtrlVal;

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAleStatusReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE Status register.
 *
 *   @b Arguments
     @verbatim
        pNumPolicers            Number of policers the ALE implements (multiple of 8)
        pNumEntries             Number of total table entries supported (multiple of 1024).
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_STATUS_KLUENTRIES
 *      ALE_ALE_STATUS_POLCNTDIV8
 *
 *   @b Example
 *   @verbatim
        Uint32      numPolicers, numEntries;

        CSL_CPSW_getAleStatusReg (&numPolicers,
                                  &numEntries);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleStatusReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        pNumPolicers,
    Uint32*                        pNumEntries
)
{
    *pNumPolicers     =   CSL_FEXT(hCpswAleRegs->ALE_STATUS, ALE_ALE_STATUS_POLCNTDIV8) << 3;
    *pNumEntries      =   CSL_FEXT(hCpswAleRegs->ALE_STATUS, ALE_ALE_STATUS_KLUENTRIES) << 10;
    if (*pNumEntries == 0)
    {
        *pNumEntries = CSL_CPSW_NUMALE_ENTRIES_MIN;
    }
}

void CSL_CPSW_getAleStatusNumAleEntries(CSL_AleRegs *hCpswAleRegs,Uint32* pNumEntries)
{
    Uint32 aleTblCtrlVal = 0;

    /* ALE_STATUS[4:0] KLUENTRIES has a granularity of 1024 entries, hence it's not
     * accurate enough for devices such as 2G and 5G that have 64 and 512 entries.
     * Writing max value for TABLEIDX field and then reading it back and adding '1'
     * will provide the number of entries. */
    aleTblCtrlVal =  CSL_FMK(ALE_ALE_TBLCTL_TABLEIDX, CSL_ALE_ALE_TBLCTL_TABLEIDX_MAX) |
                     CSL_FMK(ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL = aleTblCtrlVal;

    *pNumEntries = CSL_FEXT(hCpswAleRegs->ALE_TBLCTL, ALE_ALE_TBLCTL_TABLEIDX) + 1;
}

CSL_CPSW_ALE_RAMDEPTH_E CSL_CPSW_getAleStatusRamDepth(CSL_AleRegs *hCpswAleRegs)
{
    Uint32 ramDepth32;
    Uint32 ramDepth128;
    CSL_CPSW_ALE_RAMDEPTH_E retVal = CSL_ALE_RAMDEPTH_64;

    ramDepth32     =   CSL_FEXT(hCpswAleRegs->ALE_STATUS, ALE_ALE_STATUS_RAMDEPTH32);
    ramDepth128    =   CSL_FEXT(hCpswAleRegs->ALE_STATUS, ALE_ALE_STATUS_RAMDEPTH128);
    if (ramDepth128 != 0)
    {
        retVal = CSL_ALE_RAMDEPTH_128;
    }
    else if (ramDepth32 != 0)
    {
        retVal = CSL_ALE_RAMDEPTH_32;
    }
    return retVal;
}


void CSL_CPSW_getAleStatusVlanMask(CSL_AleRegs *hCpswAleRegs,
                                                                    bool *vlanMsk08,
                                                                    bool *vlanMsk12)
{

    *vlanMsk08     =   CSL_FEXT(hCpswAleRegs->ALE_STATUS, ALE_ALE_STATUS_UREGANDREGMSK08);
    *vlanMsk12    =   CSL_FEXT(hCpswAleRegs->ALE_STATUS, ALE_ALE_STATUS_UREGANDREGMSK12);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAlePrescaleReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE Prescale register.
 *
 *   @b Arguments
 *   @n None
 *
 *   <b> Return Value </b>
 *	 @n	 >=0                ALE prescale register contents.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_PRESCALE_ALE_PRESCALE
 *
 *   @b Example
 *   @verbatim
        Uint32      alePrescaleVal;

        alePrescaleVal  =   CSL_CPSW_getAlePrescaleReg ();

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPSW_getAlePrescaleReg (CSL_AleRegs *hCpswAleRegs)
{
    return CSL_FEXT(hCpswAleRegs->ALE_PRESCALE, ALE_ALE_PRESCALE_ALE_PRESCALE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAlePrescaleReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE prescale register.
 *
 *   @b Arguments
     @verbatim
        alePrescaleVal      Value to be configured to the ALE Prescale register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_PRESCALE_ALE_PRESCALE
 *
 *   @b Example
 *   @verbatim
 *      Uint32          alePrescaleVal = 0;

        alePrescaleVal  =   10;

        CSL_CPSW_setAlePrescaleReg (&aleCtrlRegInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAlePrescaleReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      alePrescaleVal
)
{
    CSL_FINS (hCpswAleRegs->ALE_PRESCALE, ALE_ALE_PRESCALE_ALE_PRESCALE, alePrescaleVal);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAleAgingTimerReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE Aging Timer register.
 *
 *   @b Arguments
     @verbatim
        pAgingPrescale          Aging Timer prescale (1, 1000, 1000000)
        pAgingPeriod            Aging period in units of prescale.
                                When non-zero, auto-aging is enabled.
                                This value (minus 1) times prescale is the number
                                of clock cycles after which auto-aging will automatically
                                be initiated.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_ALE_AGING_CTRL_ALE_AGING_TIMER
 *      ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE
 *      ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE
 *
 *   @b Example
 *   @verbatim
        Uint32      aleAgingPrescale;
        Uint32      aleAgingPeriod

        CSL_CPSW_getAleAgingTimerReg (&aleAgingPrescale,
                                      &aleAgingPeriod);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleAgingTimerReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        pAgingPrescale,
    Uint32*                        pAgingPeriod
)
{
    Uint32 prescale1Dis, prescale2Dis;

    prescale1Dis = CSL_FEXT(hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE);
    prescale2Dis = CSL_FEXT(hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE);

    if(prescale1Dis && prescale2Dis)
    {
        *pAgingPrescale = (Uint32)CSL_ALE_AGT_PRESACLE_1;
    }
    else if (prescale1Dis || prescale2Dis)
    {
        *pAgingPrescale = (Uint32)CSL_ALE_AGT_PRESACLE_1000;
    }else
    {
        *pAgingPrescale = (Uint32)CSL_ALE_AGT_PRESACLE_1M;
    }

    *pAgingPeriod = CSL_FEXT(hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_ALE_AGING_TIMER);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleAgingTimerReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE Aging Timer register.
 *
 *   @b Arguments
     @verbatim
        agingPrescale           Aging Timer prescale (1, 1000, 1000000)
        agingPeriod             Aging period in units of prescale.
                                When non-zero, auto-aging is enabled.
                                This value (minus 1) times prescale is the number
                                of clock cycles after which auto-aging will automatically
                                be initiated.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_AGING_CTRL_ALE_AGING_TIMER
 *      ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE
 *      ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE
 *
 *   @b Example
 *   @verbatim
        Uint32      aleAgingPrescale;
        Uint32      aleAgingPeriod;

        aleAgingPrescale = (Uint32)ALE_AGT_PRESACLE_1M;
        aleAgingPeriod = 1000;

        CSL_CPSW_setAleAgingTimerReg (aleAgingPrescale,
                                      aleAgingPeriod);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleAgingTimerReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                         agingPrescale,
    Uint32                         agingPeriod
)
{
    switch(agingPrescale)
    {
        case CSL_ALE_AGT_PRESACLE_1M:
            CSL_FINS (hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE, (Uint32) 0);
            CSL_FINS (hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE, (Uint32) 0);
            break;
        case CSL_ALE_AGT_PRESACLE_1000:
            CSL_FINS (hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE, (Uint32) 1);
            CSL_FINS (hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE, (Uint32) 0);
            break;
        case CSL_ALE_AGT_PRESACLE_1:
            CSL_FINS (hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_PRESCALE_1_DISABLE, (Uint32) 1);
            CSL_FINS (hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_PRESCALE_2_DISABLE, (Uint32) 1);
            break;
        default: break;
    }
    CSL_FINS (hCpswAleRegs->ALE_AGING_CTRL, ALE_ALE_AGING_CTRL_ALE_AGING_TIMER, agingPeriod);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAleUnkownVlanReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE Unknown VLAN and etc registers.
 *
 *   @b Arguments
     @verbatim
        pUnVlanMemList          Unknown VLAN member list.
        pUnMcastFloodMask       Unknown VLAN Multicast flood mask.
        pUnRegMcastFloodMask    Unknown VLAN Registered Multicast Flood mask.
        pUnForceUntagEgress     Unknown VLAN Force Untagged Egress.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_UNKNOWN_VLAN_REG_UNKNOWN_LIST,
 *      ALE_UNKNOWN_MCAST_FLOOD_REG_MASK,
 *      ALE_UNKNOWN_REG_MCAST_FLOOD_REG_MASK,
 *      ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS
 *
 *   @b Example
 *   @verbatim
        Uint32      unVlanMemList, unMcastFloodMask, unRegMcastFloodMask, unForceUntagEgress;

        CSL_CPSW_getAleUnkownVlanReg (&unVlanMemList,
                                      &unMcastFloodMask,
                                      &unRegMcastFloodMask,
                                      &unForceUntagEgress);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleUnkownVlanReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        pUnVlanMemList,
    Uint32*                        pUnMcastFloodMask,
    Uint32*                        pUnRegMcastFloodMask,
    Uint32*                        pUnForceUntagEgress
)
{
    *pUnVlanMemList         =   CSL_FEXT(hCpswAleRegs->ALE_UVLAN_MEMBER, ALE_ALE_UVLAN_MEMBER_UVLAN_MEMBER_LIST);
    *pUnMcastFloodMask      =   CSL_FEXT(hCpswAleRegs->ALE_UVLAN_URCAST, ALE_ALE_UVLAN_RMCAST_UVLAN_REG_MCAST_FLOOD_MASK);
    *pUnRegMcastFloodMask   =   CSL_FEXT(hCpswAleRegs->ALE_UVLAN_RMCAST, ALE_ALE_UVLAN_URCAST_UVLAN_UNREG_MCAST_FLOOD_MASK);
    *pUnForceUntagEgress    =   CSL_FEXT(hCpswAleRegs->ALE_UVLAN_UNTAG, ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleUnkownVlanReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE Unknown VLAN and etc. register.
 *
 *   @b Arguments
     @verbatim
        unVlanMemList           Unknown VLAN member list.
        unMcastFloodMask        Unknown VLAN Multicast flood mask.
        unRegMcastFloodMask     Unknown VLAN Registered Multicast Flood mask.
        unForceUntagEgress      Unknown VLAN Force Untagged Egress.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_UNKNOWN_VLAN_REG_UNKNOWN_LIST,
 *      ALE_UNKNOWN_MCAST_FLOOD_REG_MASK,
 *      ALE_UNKNOWN_REG_MCAST_FLOOD_REG_MASK,
 *      ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS
 *
 *   @b Example
 *   @verbatim
        Uint32      unVlanMemList, unMcastFloodMask, unRegMcastFloodMask, unForceUntagEgress;

        unVlanMemList           =   0;
        unMcastFloodMask        =   3;
        unRegMcastFloodMask     =   0;
        unForceUntagEgress      =   0;

        CSL_CPSW_setAleUnkownVlanReg (unVlanMemList,
                                      unMcastFloodMask,
                                      unRegMcastFloodMask,
                                      unForceUntagEgress);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUnkownVlanReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      unVlanMemList,
    Uint32                      unMcastFloodMask,
    Uint32                      unRegMcastFloodMask,
    Uint32                      unForceUntagEgress
)
{
    CSL_FINS (hCpswAleRegs->ALE_UVLAN_MEMBER, ALE_ALE_UVLAN_MEMBER_UVLAN_MEMBER_LIST, unVlanMemList);
    CSL_FINS (hCpswAleRegs->ALE_UVLAN_URCAST, ALE_ALE_UVLAN_RMCAST_UVLAN_REG_MCAST_FLOOD_MASK, unMcastFloodMask);
    CSL_FINS (hCpswAleRegs->ALE_UVLAN_RMCAST, ALE_ALE_UVLAN_URCAST_UVLAN_UNREG_MCAST_FLOOD_MASK, unRegMcastFloodMask);
    CSL_FINS (hCpswAleRegs->ALE_UVLAN_UNTAG, ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS, unForceUntagEgress);
}

#if ENET_CFG_IS_ON(ALE_VLAN_MASK_MUX)
/** ============================================================================
 *   @n@b CSL_CPSW_getAleVlanMaskMuxReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE VLAN Mask Mux registers.
 *
 *   @b Arguments
     @verbatim
        vlanMaskMux             Array of VLAN Mask Mux which is indexed by
                                the unreg_mcast_flood_index and reg_mcast_flood_
                                index values from the VLAN table entry to determine
                                the registered and unregistered multicast flood masks
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_VLAN_MASK_MUX_REG_MASK
 *
 *   @b Example
 *   @verbatim
        Uint32      vlanMaskMux[4];

        CSL_CPSW_getAleVlanMaskMuxReg (vlanMaskMux);

     @endverbatim
 *
 * @note: The value of VLAN_Mask_MUX_0 is read only and all ones (all ports are one).
 * =============================================================================
 */
void CSL_CPSW_getAleVlanMaskMuxReg
(   CSL_AleRegs *hCpswAleRegs,
    Uint32*      vlanMaskMux
)
{
	Uint32 index;

	for (index = 0; index < CSL_NUM_ALE_VLAN_MASK_MUX1_ENTRIES;index++)
	{
        vlanMaskMux[index] =  CSL_FEXT(hCpswAleRegs->I1_ALE_MSK_MUX1[index], ALE_I1_ALE_MSK_MUX1_I1_REG_VLAN_MASK_MUX_1);
	}
}

Uint32 CSL_CPSW_getAleVlanMaskMux0Reg (CSL_AleRegs *hCpswAleRegs)
{
	return(CSL_FEXT(hCpswAleRegs->ALE_MSK_MUX0, ALE_ALE_MSK_MUX0_VLAN_MASK_MUX_0));
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanMaskMuxReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE VLAN Mask Mux registers.
 *
 *   @b Arguments
     @verbatim
        vlanMaskMux             Array of VLAN Mask Mux which is indexed by
                                the unreg_mcast_flood_index and reg_mcast_flood_
                                index values from the VLAN table entry to determine
                                the registered and unregistered multicast flood masks
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_VLAN_MASK_MUX_REG_MASK
 *
 *   @b Example
 *   @verbatim
        Uint32      vlanMaskMux[4];

        vlanMaskMux[0] = 0x3;
        vlanMaskMux[1] = 0;
        ...

        CSL_CPSW_setAleVlanMaskMuxReg (vlanMaskMux);

     @endverbatim
 * @note: The value of VLAN_Mask_MUX_0 is read only and all ones (all ports are one).
 * =============================================================================
 */
void CSL_CPSW_setAleVlanMaskMuxReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32*                        vlanMaskMux
)
{
    Uint32  index;

	for (index = 0; index < CSL_NUM_ALE_VLAN_MASK_MUX1_ENTRIES;index++)
    {
        CSL_FINS (hCpswAleRegs->I1_ALE_MSK_MUX1[index], ALE_I1_ALE_MSK_MUX1_I1_REG_VLAN_MASK_MUX_1, vlanMaskMux[index]);
    }
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAleVlanMaskMuxEntryReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the ALE VLAN Mask Mux registers.
 *
 *   @b Arguments
     @verbatim
        vlanMaskMux             Array of VLAN Mask Mux which is indexed by
                                the unreg_mcast_flood_index and reg_mcast_flood_
                                index values from the VLAN table entry to determine
                                the registered and unregistered multicast flood masks
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  CSL_PASS               The function completed successfully
 *   @n  CSL_EOUT_OF_RANGE      The ifSelect argument is out-of-range
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_VLAN_MASK_MUX_REG_MASK
 *
 *   @b Example
 *   @verbatim
        Uint32      vlanMaskMux[4];

        CSL_CPSW_getAleVlanMaskMuxReg (vlanMaskMux);

     @endverbatim
 *
 * @note: The value of VLAN_Mask_MUX_0 is read only and all ones (all ports are one).
 * =============================================================================
 */

Int32 CSL_CPSW_getAleVlanMaskMuxEntryReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                         maskMuxIndex,
    Uint32*                        vlanMaskMuxPtr
)
{
    Int32 retVal = CSL_PASS;
    if (maskMuxIndex == 0U)
    {
        *vlanMaskMuxPtr =  CSL_FEXT(hCpswAleRegs->ALE_MSK_MUX0, ALE_ALE_MSK_MUX0_VLAN_MASK_MUX_0);
    }
    else if (maskMuxIndex <= CSL_NUM_ALE_VLAN_MASK_MUX1_ENTRIES)
    {
        *vlanMaskMuxPtr =  CSL_FEXT(hCpswAleRegs->I1_ALE_MSK_MUX1[maskMuxIndex - 1U], ALE_I1_ALE_MSK_MUX1_I1_REG_VLAN_MASK_MUX_1);
    }
    else
    {
        *vlanMaskMuxPtr = 0U;
        retVal = CSL_EOUT_OF_RANGE;
    }
    return (retVal);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanMaskMuxEntryReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE VLAN Mask Mux registers.
 *
 *   @b Arguments
     @verbatim
        vlanMaskMux             Array of VLAN Mask Mux which is indexed by
                                the unreg_mcast_flood_index and reg_mcast_flood_
                                index values from the VLAN table entry to determine
                                the registered and unregistered multicast flood masks
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  CSL_PASS               The function completed successfully
 *   @n  CSL_EOUT_OF_RANGE      The ifSelect argument is out-of-range
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_VLAN_MASK_MUX_REG_MASK
 *
 *   @b Example
 *   @verbatim
        Uint32      vlanMaskMux[4];

        vlanMaskMux[0] = 0x3;
        vlanMaskMux[1] = 0;
        ...

        CSL_CPSW_setAleVlanMaskMuxReg (vlanMaskMux);

     @endverbatim
 * @note: The value of VLAN_Mask_MUX_0 is read only and all ones (all ports are one).
 * =============================================================================
 */

Int32 CSL_CPSW_setAleVlanMaskMuxEntryReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                        maskMuxIndex,
    Uint32                        vlanMaskMuxVal
)
{
    Int32 retVal = CSL_PASS;
    if (maskMuxIndex < CSL_NUM_ALE_VLAN_MASK_MUX1_ENTRIES)
    {
        CSL_FINS (hCpswAleRegs->I1_ALE_MSK_MUX1[maskMuxIndex],
                  ALE_I1_ALE_MSK_MUX1_I1_REG_VLAN_MASK_MUX_1, vlanMaskMuxVal);
    }
    else
    {
        retVal = CSL_EOUT_OF_RANGE;
    }
    return (retVal);
}
#endif

/** ============================================================================
 *   @n@b CSL_CPSW_getAleTableEntry
 *
 *   @b Description
 *   @n This function retrieves an ALE table entry corresponding to the
 *      ALE entry index specified in 'index' input parameter. The ALE
 *      entry values corresponding to the ALE_TBLW0, ALE_TBLW1 and
 *      ALE_TBLW2 registers are returned in 'pAleInfoWd0', 'pAleInfoWd1', 'pAleInfoWd2'
 *      output parameters.
 *
 *   @b Arguments
     @verbatim
        index                   ALE table index to be read.
        pAleInfoWd0             Contents of ALE Table Word 0 Register (ALE_TBLW0).
        pAleInfoWd1             Contents of ALE Table Word 1 Register (ALE_TBLW1).
        pAleInfoWd2             Contents of ALE Table Word 2 Register (ALE_TBLW2).
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=0
 *
 *   @b Reads
 *   @n ALE_ALE_TBLW0_TABLEWRD0,
 *      ALE_ALE_TBLW1_TABLEWRD1,
 *      ALE_ALE_TBLW2_TABLEWRD2
 *
 *   @b Example
 *   @verbatim
        Uint32      index, info0, info1, info2;

        index   =   0;

        CSL_CPSW_getAleTableEntry (index,
                                   &info0,
                                   &info1,
                                   &info2);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAleTableEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      index,
    Uint32*                     pAleInfoWd0,
    Uint32*                     pAleInfoWd1,
    Uint32*                     pAleInfoWd2
)
{
    Uint32                      aleTblCtrlVal   =   0;

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;

    /* Read the ALE table word registers */
    *pAleInfoWd0            =   CSL_FEXT(hCpswAleRegs->ALE_TBLW0, ALE_ALE_TBLW0_TABLEWRD0);
    *pAleInfoWd1            =   CSL_FEXT(hCpswAleRegs->ALE_TBLW1, ALE_ALE_TBLW1_TABLEWRD1);
    *pAleInfoWd2            =   CSL_FEXT(hCpswAleRegs->ALE_TBLW2, ALE_ALE_TBLW2_TABLEWRD2);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleTableEntry
 *
 *   @b Description
 *   @n This function sets up an ALE table entry corresponding to the
 *      ALE entry index specified in 'index' input parameter. The ALE
 *      entry values corresponding to the ALE_TBLW0, ALE_TBLW1 and
 *      ALE_TBLW2 registers msut be specified in 'aleInfoWd0', 'aleInfoWd1', 'aleInfoWd2'
 *      input parameters.
 *
 *   @b Arguments
     @verbatim
        index                   ALE table index to be written.
        aleInfoWd0              Value to write to ALE Table Word 0 Register (ALE_TBLW0).
        aleInfoWd1              Value to write to ALE Table Word 1 Register (ALE_TBLW1).
        aleInfoWd2              Value to write to ALE Table Word 2 Register (ALE_TBLW2).
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLW0_TABLEWRD0,
 *      ALE_ALE_TBLW1_TABLEWRD1,
 *      ALE_ALE_TBLW2_TABLEWRD2,
 *      ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *
 *
 *   @b Example
 *   @verbatim
        Uint32      index, info0, info1, info2;

        index   =   0;
        info0   =   ...;
        info1   =   ...;
        info2   =   ...;

        CSL_CPSW_setAleTableEntry (index,
                                          info0,
                                          info1,
                                          info2);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleTableEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      index,
    Uint32                      aleInfoWd0,
    Uint32                      aleInfoWd1,
    Uint32                      aleInfoWd2
)
{
    Uint32                      aleTblCtrlVal   =   0;

    /* Set the ALE table word registers */
    CSL_FINS (hCpswAleRegs->ALE_TBLW0, ALE_ALE_TBLW0_TABLEWRD0, aleInfoWd0);
    CSL_FINS (hCpswAleRegs->ALE_TBLW1, ALE_ALE_TBLW1_TABLEWRD1, aleInfoWd1);
    CSL_FINS (hCpswAleRegs->ALE_TBLW2, ALE_ALE_TBLW2_TABLEWRD2, aleInfoWd2);

    /* Set the index in the ALE table to "write" operation and let
     * the hardware set up the ALE table entry corresponding to index
     * specified.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, (Uint32) 1);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;

    return;
}



Uint32 CSL_CPSW_getAleIPv6HighEntryOffset(CSL_AleRegs *hCpswAleRegs)
{
    CSL_CPSW_ALE_RAMDEPTH_E ramDepth;
    UInt32 offset = 0;

    ramDepth = CSL_CPSW_getAleStatusRamDepth(hCpswAleRegs);

    switch (ramDepth)
    {
        case CSL_ALE_RAMDEPTH_32:
            offset = 32;
            break;
        case CSL_ALE_RAMDEPTH_64:
            offset = 64;
            break;

        case CSL_ALE_RAMDEPTH_128:
            offset = 128;
            break;
    }
    return offset;
}


Uint32 CSL_CPSW_getAleIPv6HighEntryIndex(CSL_AleRegs *hCpswAleRegs,Uint32 entryIndex)
{
    return (entryIndex + CSL_CPSW_getAleIPv6HighEntryOffset(hCpswAleRegs));
}

Uint32 CSL_CPSW_getAleIPv6LowEntryIndex(CSL_AleRegs *hCpswAleRegs,Uint32 entryIndex)
{
    return (entryIndex);
}


/** ============================================================================
 *   @n@b CSL_CPSW_clearAleEntry
 *
 *   @b Description
 *   @n This function clears the ALE entry corresponding to the index
 *      specified
 *
 *   @b Arguments
     @verbatim
        index                   ALE table index.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_ALE_TBLCTL_TABLEIDX,
 *      ALE_ALE_TBLCTL_TABLEWR=1
 *      ALE_TABLE_WORD0=0,
 *      ALE_TABLE_WORD1=0,
 *      ALE_TABLE_WORD2=0
 *
 *   @b Example
 *   @verbatim
        Uint32                              index;

        index   =   0;

        CSL_CPSW_clearAleEntry (index);
     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_clearAleEntry(CSL_AleRegs *hCpswAleRegs,
    Uint32                                  index)
{
    Uint32                                  aleTblCtrlVal   =   0;

    /* Clear all the ALE words */
    hCpswAleRegs->ALE_TBLW0   =   0;
    hCpswAleRegs->ALE_TBLW1   =   0;
    hCpswAleRegs->ALE_TBLW2   =   0;

    /* Set the index in the ALE table to "write" operation and let
     * the hardware set up the ALE table entry corresponding to index
     * specified.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, (Uint32) 1);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAlePortControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of ALE Port control register
 *      corresponding to the port number specified.
 *
 *   @b Arguments
     @verbatim
        portNo                  Port number for which the ALE port control register
                                must be read.
        pPortControlInfo        CSL_CPSW_ALE_PORTCONTROL structure that needs to be
                                filled with Port control register info read from
                                the hardware.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT
 *
 *   @b Example
 *   @verbatim
        Uint32                          index;
        CSL_CPSW_ALE_PORTCONTROL    portControlInfo;

        index   =   0;

        CSL_CPSW_getAlePortControlReg (index, &portControlInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAlePortControlReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    CSL_CPSW_ALE_PORTCONTROL*   pPortControlInfo
)
{
    pPortControlInfo->portState             =   (CSL_CPSW_ALE_PORTSTATE) CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE);
    pPortControlInfo->dropUntaggedEnable    =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED);
    pPortControlInfo->vidIngressCheckEnable =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK);
    pPortControlInfo->noLearnModeEnable     =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN);
    pPortControlInfo->noSaUpdateEnable      =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE);
    pPortControlInfo->macOnlyEnable         =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY);
    pPortControlInfo->macAuthDisable        =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD);
    pPortControlInfo->macOnlyCafEnable      =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF);
    pPortControlInfo->mcastLimit            =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT);
    pPortControlInfo->bcastLimit            =   CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                                          ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT);
    pPortControlInfo->dropDualVlan = CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                               ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DUAL_VLAN);
    pPortControlInfo->dropDoubleVlan = CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],
                                               ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DOUBLE_VLAN);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAlePortControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of ALE Port control register
 *      corresponding to the port number specified.
 *
 *   @b Arguments
     @verbatim
        portNo                  Port number for which the ALE port control register
                                must be configured.
        pPortControlInfo        CSL_CPSW_ALE_PORTCONTROL structure that contains
                                port control register settings to be written.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT,
 *      ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT
 *
 *   @b Example
 *   @verbatim
        Uint32                          index;
        CSL_CPSW_ALE_PORTCONTROL    portControlInfo;

        index   =   0;
        portControlInfo.portState   =   ALE_PORTSTATE_FORWARD |
                                        ALE_PORTSTATE_LEARN;

        CSL_CPSW_setAlePortControlReg (index, &portControlInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAlePortControlReg
(CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    CSL_CPSW_ALE_PORTCONTROL*   pPortControlInfo
)
{
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE,
              pPortControlInfo->portState);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_UN_TAGGED,
              pPortControlInfo->dropUntaggedEnable);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_VID_INGRESS_CHECK,
              pPortControlInfo->vidIngressCheckEnable);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_LEARN,
              pPortControlInfo->noLearnModeEnable);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_NO_SA_UPDATE,
              pPortControlInfo->noSaUpdateEnable);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY,
              pPortControlInfo->macOnlyEnable);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_DIS_PAUTHMOD,
              pPortControlInfo->macAuthDisable);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_MACONLY_CAF,
              pPortControlInfo->macOnlyCafEnable);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_MCAST_LIMIT,
              pPortControlInfo->mcastLimit);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_BCAST_LIMIT,
              pPortControlInfo->bcastLimit);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DUAL_VLAN,
              pPortControlInfo->dropDualVlan);
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_DROP_DOUBLE_VLAN,
              pPortControlInfo->dropDoubleVlan);

    return;
}

void CSL_CPSW_setAlePortControlTrunk
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    bool                     trunkEnable,
    Uint32                      trunkNum
)
{
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0[portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_TRUNKNUM,
              trunkNum);

    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_TRUNKEN,
              trunkEnable);

}
/** ============================================================================
 *   @n@b CSL_CPSW_getAlePortControlReg
 *
 * =============================================================================
 */
void CSL_CPSW_getAlePortState
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    CSL_CPSW_ALE_PORTSTATE      *pPortState
)
{
    *pPortState =
    (CSL_CPSW_ALE_PORTSTATE)CSL_FEXT (hCpswAleRegs->I0_ALE_PORTCTL0[portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAlePortControlReg
 *
 * =============================================================================
 */
void CSL_CPSW_setAlePortState
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      portNo,
    CSL_CPSW_ALE_PORTSTATE      portState
)
{
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo], ALE_I0_ALE_PORTCTL0_I0_REG_P0_PORTSTATE,
              portState);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAlePortControlReg
 *
 * =============================================================================
 */
void CSL_CPSW_setAlePortMirrorSouce(CSL_AleRegs *hCpswAleRegs,
                                                    Uint32  portNo,
                                                    bool enableMirror)
{
    CSL_FINS (hCpswAleRegs->I0_ALE_PORTCTL0 [portNo],ALE_I0_ALE_PORTCTL0_I0_REG_P0_MIRROR_SP ,
              enableMirror);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleCtrl2MirrorMatchIndex
 *
 * =============================================================================
 */
void CSL_CPSW_setAleCtrl2MirrorMatchIndex(CSL_AleRegs *hCpswAleRegs,
                                                          Uint32  mirrorMatchIndex)
{
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_MIRROR_MIDX ,
              mirrorMatchIndex);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleCtrl2MirrorMatchIndex
 *
 * =============================================================================
 */
void CSL_CPSW_setAleCtrl2TrunkParams(CSL_AleRegs *hCpswAleRegs,
                                                     CSL_CPSW_ALE_CTRL2_TRUNK_CONFIG *trunkCfg)
{
    CSL_FINS (hCpswAleRegs->ALE_CTRL2, ALE_ALE_CTRL2_TRK_BASE ,
              trunkCfg->trunkBase);
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_TRK_EN_DIP ,
              trunkCfg->trunkEnableDestIP);
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_TRK_EN_SIP ,
              trunkCfg->trunkEnableSrcIP);
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_TRK_EN_IVLAN ,
              trunkCfg->trunkEnableInnerVLAN);
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_TRK_EN_PRI ,
              trunkCfg->trunkEnablePri);
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_TRK_EN_SRC ,
              trunkCfg->trunkEnableSrc);
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_TRK_EN_DST ,
              trunkCfg->trunkEnableDst);

}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleCtrl2IPPktFilterConfig
 *
 * =============================================================================
 */
void CSL_CPSW_setAleCtrl2IPPktFilterConfig(CSL_AleRegs *hCpswAleRegs,
                                                           CSL_CPSW_ALE_CTRL2_IPPKTFLT_CONFIG *ipPktFltCfg)
{
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_DEFNOFRAG ,
              ipPktFltCfg->ipPktFltEnableDefNoFrag);
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_DEFLMTNXTHDR ,
              ipPktFltCfg->ipPktFltEnableDefNxtHdrLimit);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleCtrl2MalformedPktConfig
 *
 * =============================================================================
 */
void CSL_CPSW_setAleCtrl2MalformedFrameConfig(CSL_AleRegs *hCpswAleRegs,
                                                              CSL_CPSW_ALE_CTRL2_MALFORMEDFRAME_CONFIG *badFrmCfg)
{
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_DROP_BADLEN ,
              badFrmCfg->dropBadLen);
    CSL_FINS (hCpswAleRegs->ALE_CTRL2,ALE_ALE_CTRL2_NODROP_SRCMCST,
              badFrmCfg->noDropSrcMcast);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setIPNxtHdrList
 *
 * =============================================================================
 */
void CSL_CPSW_setAleIPNxtHdrWhitelist(CSL_AleRegs *hCpswAleRegs,
                                                   Uint8 ipNxtHdr0,
                                                   Uint8 ipNxtHdr1,
                                                   Uint8 ipNxtHdr2,
                                                   Uint8 ipNxtHdr3)
{
    CSL_FINS (hCpswAleRegs->ALE_NXT_HDR ,ALE_ALE_NXT_HDR_IP_NXT_HDR0,
              ipNxtHdr0);
    CSL_FINS (hCpswAleRegs->ALE_NXT_HDR,ALE_ALE_NXT_HDR_IP_NXT_HDR1,
              ipNxtHdr1);
    CSL_FINS (hCpswAleRegs->ALE_NXT_HDR,ALE_ALE_NXT_HDR_IP_NXT_HDR2,
              ipNxtHdr2);
    CSL_FINS (hCpswAleRegs->ALE_NXT_HDR,ALE_ALE_NXT_HDR_IP_NXT_HDR3,
              ipNxtHdr3);

}

/** ============================================================================
 *   @n@b CSL_CPSW_getAlePolicerGlobConfig
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW ALE Policer/Classifier
 *        Global Configuration.
 *
 *   @b Arguments
     @verbatim
        pGlobConfig             CSL_CPSW_ALE_POLICER_GLOB_CONFIG structure that needs to
                                be populated with the contents of the corresponging ALE Policer
                                global control registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n ALE_THREADMAPDEF_DEFTHREAD_EN,
 *      ALE_THREADMAPDEF_DEFTHREADVAL
 *
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_ALE_POLICER_GLOB_CONFIG    globConfig;

        CSL_CPSW_getAlePolicerGlobConfig (&globConfig);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAlePolicerGlobConfig (CSL_AleRegs *hCpswAleRegs,
	CSL_CPSW_ALE_POLICER_GLOB_CONFIG*   pGlobConfig
)
{
    pGlobConfig->defThreadEnable    =   CSL_FEXT (hCpswAleRegs->THREADMAPDEF, ALE_THREADMAPDEF_DEFTHREAD_EN);
    pGlobConfig->defThread          =   CSL_FEXT (hCpswAleRegs->THREADMAPDEF, ALE_THREADMAPDEF_DEFTHREADVAL);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAlePolicerGlobConfig
 *
 *   @b Description
 *   @n This function sets up the contents of the CPSW ALE Policer/Classifier
 *      global registers
 *      per user-specified ALE Policer Global Configuration.
 *
 *   @b Arguments
     @verbatim
        pGlobConfig         CSL_CPSW_ALE_POLICER_GLOB_CONFIG structure that holds the values
                            that need to be configured to the ALE Policer global control
                            registers.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSW ALE Policer Global control register modified with values provided.
 *
 *   @b Writes
 *   @n ALE_THREADMAPDEF_DEFTHREAD_EN,
 *      ALE_THREADMAPDEF_DEFTHREADVAL
 *
 *   @b Example
 *   @verbatim
        CSL_CPSW_ALE_POLICER_GLOB_CONFIG    globConfig;

        globConfig.defThreadEnable  =   1;
        ...

        CSL_CPSW_setAlePolicerGlobConfig (&globConfig);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAlePolicerGlobConfig (CSL_AleRegs *hCpswAleRegs,
	CSL_CPSW_ALE_POLICER_GLOB_CONFIG*   pGlobConfig
)
{
    CSL_FINS (hCpswAleRegs->THREADMAPDEF, ALE_THREADMAPDEF_DEFTHREAD_EN, pGlobConfig->defThreadEnable);
    CSL_FINS (hCpswAleRegs->THREADMAPDEF, ALE_THREADMAPDEF_DEFTHREADVAL, pGlobConfig->defThread);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAlePolicerEntry
 *
 *   @b Description
 *   @n This function reads the ALE Policer table entry for the index specified and
 *      fills the output parameter structure with Policer configuration
 *      read from the hardware.
 *
 *   @b Arguments
     @verbatim
        index                   ALE Policer table index to be read.
        pPolCfg                 ALE Policer entry contents read.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_POLICETBLCTL_POL_TBL_IDX
 *      ALE_POLICETBLCTL_WRITE_ENABLE=0
 *      ALE_THREADMAPCTL_CLASSINDEX
 *
 *   @b Reads
 *   @n ALE_POLICECFG0_PORT_MEN,
 *      ALE_POLICECFG0_PORT_NUM,
 *      ALE_POLICECFG0_PRI_MEN,
 *      ALE_POLICECFG0_PRI_VAL,
 *      ALE_POLICECFG0_ONU_MEN,
 *      ALE_POLICECFG0_ONU_INDEX,
 *      ALE_POLICECFG1_DST_MEN,
 *      ALE_POLICECFG1_DST_INDEX,
 *      ALE_POLICECFG1_SRC_MEN,
 *      ALE_POLICECFG1_SRC_INDEX,
 *      ALE_POLICECFG2_OVLAN_MEN,
 *      ALE_POLICECFG2_OVLAN_INDEX,
 *      ALE_POLICECFG2_IVLAN_MEN,
 *      ALE_POLICECFG2_IVLAN_INDEX,
 *      ALE_POLICECFG3_ETHERTYPE_MEN,
 *      ALE_POLICECFG3_ETHERTYPE_INDEX,
 *      ALE_POLICECFG3_IPSRC_MEN,
 *      ALE_POLICECFG3_IPSRC_INDEX,
 *      ALE_POLICECFG4_IPDST_MEN,
 *      ALE_POLICECFG4_IPDST_INDEX,
 *      ALE_THREADMAPVAL_THREAD_EN,
 *      ALE_THREADMAPVAL_THREADVAL
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_POLICER_ENTRY              polCfg;

        index   =   0;

         // Read Policer Entry config from hardware
         CSL_CPSW_getAlePolicerEntry (index, &polCfg);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_getAlePolicerEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                              index,
    CSL_CPSW_ALE_POLICER_ENTRY*         pPolCfg
)
{
    Uint32                                  aleTblCtrlVal, value;

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_POLICETBLCTL_POL_TBL_IDX, index) |
                                CSL_FMK (ALE_POLICETBLCTL_WRITE_ENABLE, (Uint32) 0);

    hCpswAleRegs->POLICETBLCTL    =   aleTblCtrlVal;

    memset(pPolCfg, 0, sizeof(*pPolCfg));

    /* Read the Policer Entry configuration */
    value  =  hCpswAleRegs->POLICECFG0;

    if(CSL_FEXT(value, ALE_POLICECFG0_PORT_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_PORT_VALID;
        pPolCfg->port = CSL_FEXT(value, ALE_POLICECFG0_PORT_NUM);
    }
    if(CSL_FEXT(value, ALE_POLICECFG0_TRUNKID))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_PORT_TRUNK_VALID;
    }

    if(CSL_FEXT(value, ALE_POLICECFG0_PRI_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_PRI_VALID;
        pPolCfg->pri = CSL_FEXT(value, ALE_POLICECFG0_PRI_VAL);
    }

    if(CSL_FEXT(value, ALE_POLICECFG0_ONU_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_OUI_VALID;
        pPolCfg->ouiIdx = CSL_FEXT(value, ALE_POLICECFG0_ONU_INDEX);
    }

    value  =  hCpswAleRegs->POLICECFG1;

    if(CSL_FEXT(value, ALE_POLICECFG1_DST_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_DST_MAC_VALID;
        pPolCfg->dstMacIdx = CSL_FEXT(value, ALE_POLICECFG1_DST_INDEX);
    }

    if(CSL_FEXT(value, ALE_POLICECFG1_SRC_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_SRC_MAC_VALID;
        pPolCfg->srcMacIdx = CSL_FEXT(value, ALE_POLICECFG1_SRC_INDEX);
    }

    value  =  hCpswAleRegs->POLICECFG2;

    if(CSL_FEXT(value, ALE_POLICECFG2_OVLAN_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_OVLAN_VALID;
        pPolCfg->ovlanIdx = CSL_FEXT(value, ALE_POLICECFG2_OVLAN_INDEX);
    }

    if(CSL_FEXT(value, ALE_POLICECFG2_IVLAN_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_VLAN_VALID;
        pPolCfg->vlanIdx = CSL_FEXT(value, ALE_POLICECFG2_IVLAN_INDEX);
    }

    value  =  hCpswAleRegs->POLICECFG3;

    if(CSL_FEXT(value, ALE_POLICECFG3_ETHERTYPE_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_ETHERTYPE_VALID;
        pPolCfg->ethertypeIdx = CSL_FEXT(value, ALE_POLICECFG3_ETHERTYPE_INDEX);
    }

    if(CSL_FEXT(value, ALE_POLICECFG3_IPSRC_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_SRC_IP_VALID;
        pPolCfg->srcIpIdx = CSL_FEXT(value, ALE_POLICECFG3_IPSRC_INDEX);
    }

    value  =  hCpswAleRegs->POLICECFG4;

    if(CSL_FEXT(value, ALE_POLICECFG4_IPDST_MEN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_DST_IP_VALID;
        pPolCfg->dstIpIdx = CSL_FEXT(value, ALE_POLICECFG4_IPDST_INDEX);
    }

    value  =  hCpswAleRegs->POLICECFG6;
    pPolCfg->pirIdleIncVal = CSL_FEXT(value,ALE_POLICECFG6_PIR_IDLE_INC_VAL);
    if (value)
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_PIR_VALID;
    }

    value  =  hCpswAleRegs->POLICECFG7;
    pPolCfg->cirIdleIncVal = CSL_FEXT(value,ALE_POLICECFG7_CIR_IDLE_INC_VAL);
    if (value)
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_CIR_VALID;
    }

    value  =  hCpswAleRegs->EGRESSOP;
    if (value != 0)
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_EGRESSOP_VALID;

        pPolCfg->egressOp = CSL_FEXT(value,ALE_EGRESSOP_EGRESS_OP);
        pPolCfg->egressTrunkIndex= CSL_FEXT(value,ALE_EGRESSOP_EGRESS_TRK);
        pPolCfg->enableTTLCheck = CSL_FEXT(value,ALE_EGRESSOP_TTL_CHECK);
        pPolCfg->destPortMask = CSL_FEXT(value,ALE_EGRESSOP_DEST_PORTS);

    }

    /* Read Thread value */
    aleTblCtrlVal           =   CSL_FMK (ALE_THREADMAPCTL_CLASSINDEX, index);

    hCpswAleRegs->THREADMAPCTL    =   aleTblCtrlVal;

    value  =  hCpswAleRegs->THREADMAPVAL;

    if(CSL_FEXT(value, ALE_THREADMAPVAL_THREAD_EN))
    {
        pPolCfg->validBitmap |= CSL_CPSW_ALE_POLICER_THREAD_VALID;
        pPolCfg->thread = CSL_FEXT(value, ALE_THREADMAPVAL_THREADVAL);
    }

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAlePolicerEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      VLAN configuration specified here.
 *
 *   @b Arguments
     @verbatim
        index                   ALE table index.
        pPolCfg                 ALE entry contents to be configured.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n ALE_POLICETBLCTL_POL_TBL_IDX
 *      ALE_POLICETBLCTL_WRITE_ENABLE=1
 *      ALE_THREADMAPCTL_CLASSINDEX
 *      ALE_POLICECFG0_PORT_MEN,
 *      ALE_POLICECFG0_PORT_NUM,
 *      ALE_POLICECFG0_PRI_MEN,
 *      ALE_POLICECFG0_PRI_VAL,
 *      ALE_POLICECFG0_ONU_MEN,
 *      ALE_POLICECFG0_ONU_INDEX,
 *      ALE_POLICECFG1_DST_MEN,
 *      ALE_POLICECFG1_DST_INDEX,
 *      ALE_POLICECFG1_SRC_MEN,
 *      ALE_POLICECFG1_SRC_INDEX,
 *      ALE_POLICECFG2_OVLAN_MEN,
 *      ALE_POLICECFG2_OVLAN_INDEX,
 *      ALE_POLICECFG2_IVLAN_MEN,
 *      ALE_POLICECFG2_IVLAN_INDEX,
 *      ALE_POLICECFG3_ETHERTYPE_MEN,
 *      ALE_POLICECFG3_ETHERTYPE_INDEX,
 *      ALE_POLICECFG3_IPSRC_MEN,
 *      ALE_POLICECFG3_IPSRC_INDEX,
 *      ALE_POLICECFG4_IPDST_MEN,
 *      ALE_POLICECFG4_IPDST_INDEX,
 *      ALE_THREADMAPVAL_THREAD_EN,
 *      ALE_THREADMAPVAL_THREADVAL
 *
 *   @b Example
 *   @verbatim
        Uint32                                  index;
        CSL_CPSW_ALE_POLICER_ENTRY              polCfg;

        index   =   0;
        polCfg.vlanId  = 0x10;
        ...

        // Add ALE Policer entry
        CSL_CPSW_setAlePolicerEntry (index, &polCfg);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAlePolicerEntry
(CSL_AleRegs *hCpswAleRegs,
    Uint32                              index,
    CSL_CPSW_ALE_POLICER_ENTRY*         pPolCfg
)
{
    Uint32                                  aleTblCtrlVal, value;

    /* Read the Policer Entry configuration */
    value = 0;
    if(pPolCfg->validBitmap & CSL_CPSW_ALE_POLICER_PORT_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG0_PORT_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG0_PORT_NUM, pPolCfg->port);
        if (pPolCfg->validBitmap & CSL_CPSW_ALE_POLICER_PORT_TRUNK_VALID)
        {
            value |= CSL_FMK (ALE_POLICECFG0_TRUNKID,(Uint32) 1);
        }
    }

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_PRI_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG0_PRI_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG0_PRI_VAL, pPolCfg->pri);
    }

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_OUI_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG0_ONU_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG0_ONU_INDEX, pPolCfg->ouiIdx);
    }

    hCpswAleRegs->POLICECFG0 = value;

    value = 0;

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_DST_MAC_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG1_DST_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG1_DST_INDEX, pPolCfg->dstMacIdx);
    }

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_SRC_MAC_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG1_SRC_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG1_SRC_INDEX, pPolCfg->srcMacIdx);
    }

    hCpswAleRegs->POLICECFG1 = value;

    value = 0;

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_OVLAN_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG2_OVLAN_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG2_OVLAN_INDEX, pPolCfg->ovlanIdx);
    }

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_VLAN_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG2_IVLAN_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG2_IVLAN_INDEX, pPolCfg->vlanIdx);
    }

    hCpswAleRegs->POLICECFG2 = value;

    value = 0;

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_ETHERTYPE_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG3_ETHERTYPE_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG3_ETHERTYPE_INDEX, pPolCfg->ethertypeIdx);
    }

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_SRC_IP_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG3_IPSRC_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG3_IPSRC_INDEX, pPolCfg->srcIpIdx);
    }

    hCpswAleRegs->POLICECFG3 = value;

    value = 0;

    if(pPolCfg->validBitmap &  CSL_CPSW_ALE_POLICER_DST_IP_VALID)
    {
        value |= CSL_FMK (ALE_POLICECFG4_IPDST_MEN, (Uint32) 1) |
                 CSL_FMK (ALE_POLICECFG4_IPDST_INDEX, pPolCfg->dstIpIdx);
    }

    hCpswAleRegs->POLICECFG4 = value;

    value = 0;

    if(pPolCfg->validBitmap & CSL_CPSW_ALE_POLICER_PIR_VALID)
    {
        value       =   CSL_FMK (ALE_POLICECFG6_PIR_IDLE_INC_VAL, pPolCfg->pirIdleIncVal);
    }
    hCpswAleRegs->POLICECFG6 = value;

    value = 0;

    if(pPolCfg->validBitmap & CSL_CPSW_ALE_POLICER_CIR_VALID)
    {
        value       =   CSL_FMK (ALE_POLICECFG7_CIR_IDLE_INC_VAL, pPolCfg->cirIdleIncVal);
    }
    hCpswAleRegs->POLICECFG7 = value;

    value = 0;
    if(pPolCfg->validBitmap & CSL_CPSW_ALE_POLICER_EGRESSOP_VALID)
    {

        value  =   CSL_FMK (ALE_EGRESSOP_EGRESS_OP, pPolCfg->egressOp);
        value |=  CSL_FMK (ALE_EGRESSOP_EGRESS_TRK, pPolCfg->egressTrunkIndex);
        value |=  CSL_FMK (ALE_EGRESSOP_TTL_CHECK, pPolCfg->enableTTLCheck);
        value |=  CSL_FMK (ALE_EGRESSOP_DEST_PORTS, pPolCfg->destPortMask);
    }
    hCpswAleRegs->EGRESSOP = value;

    /* Set the index in the ALE Policertable to "write" operation and let
     * the hardware load the ALE Policer entry corresponding to index
     * specified into the ALE Policer registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_POLICETBLCTL_POL_TBL_IDX, index) |
                                CSL_FMK (ALE_POLICETBLCTL_WRITE_ENABLE, (Uint32) 1);

    hCpswAleRegs->POLICETBLCTL    =   aleTblCtrlVal;
    /* Write Thread value */
    if(pPolCfg->validBitmap & CSL_CPSW_ALE_POLICER_THREAD_VALID)
    {

        value =  CSL_FMK (ALE_THREADMAPVAL_THREAD_EN, (Uint32) 1) |
                 CSL_FMK (ALE_THREADMAPVAL_THREADVAL, pPolCfg->thread);

        hCpswAleRegs->THREADMAPVAL = value;

        aleTblCtrlVal       =   CSL_FMK (ALE_THREADMAPCTL_CLASSINDEX, index);

        hCpswAleRegs->THREADMAPCTL    =   aleTblCtrlVal;

    }

    return;
}

void CSL_CPSW_disableAlePolicerThread(CSL_AleRegs *hCpswAleRegs,
                                                    Uint32           index)
{
	Uint32 aleTblCtrlVal;


    aleTblCtrlVal =  CSL_FMK (ALE_THREADMAPVAL_THREAD_EN, (Uint32) 0) |
             CSL_FMK (ALE_THREADMAPVAL_THREADVAL, 0);

    hCpswAleRegs->THREADMAPVAL = aleTblCtrlVal;

    aleTblCtrlVal       =   CSL_FMK (ALE_THREADMAPCTL_CLASSINDEX, index);

    hCpswAleRegs->THREADMAPCTL    =   aleTblCtrlVal;
    return;
}



/** ============================================================================
 *   @n@b CSL_CPSW_setAleUnknwnVlanMemberReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE unknown vlan register.
 *
 *   @b Arguments
     @verbatim
        aleUnknwnVlanVal  Value to be configured to the ALE unknown vlan reg
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n UNKNOWN_VLAN_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32          aleUnknwnVlanVal = 0;

        CSL_CPSW_getAleUnknwnVlanReg (&aleUnknwnVlanVal);
        aleUnknwnVlanVal      |=  CSL_XGE_CPSW_ALECONTROL_CLRTABLE_EN;

        CSL_CPSW_setAleUnknwnVlanReg (aleUnknwnVlanVal);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUnknwnVlanMemberReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUnknwnVlanMemberVal
)
{
    CSL_FINS (hCpswAleRegs->ALE_UVLAN_MEMBER, ALE_ALE_UVLAN_MEMBER_UVLAN_MEMBER_LIST, aleUnknwnVlanMemberVal);
    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_setAleUnknwnVlanUntagReg
 *
 *   @b Description
 *   @n This function sets up the contents of the ALE unknown vlan register.
 *
 *   @b Arguments
     @verbatim
        aleUnknwnVlanVal  Value to be configured to the ALE unknown vlan reg
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n UNKNOWN_VLAN_REG
 *
 *   @b Example
 *   @verbatim
 *      Uint32          aleUnknwnVlanVal = 0;

        CSL_CPSW_getAleUnknwnVlanReg (&aleUnknwnVlanVal);
        aleUnknwnVlanVal      |=  CSL_XGE_CPSW_ALECONTROL_CLRTABLE_EN;

        CSL_CPSW_setAleUnknwnVlanReg (aleUnknwnVlanVal);

     @endverbatim
 * =============================================================================
 */
void CSL_CPSW_setAleUnknwnVlanUntagReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUnknwnVlanUntagVal
)
{
    CSL_FINS (hCpswAleRegs->ALE_UVLAN_UNTAG, ALE_ALE_UVLAN_UNTAG_UVLAN_FORCE_UNTAGGED_EGRESS, aleUnknwnVlanUntagVal);

    return;
}

void CSL_CPSW_setAleUnknwnVlanUnregMcastReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUnknwnVlanUnregMcastVal
)
{
    CSL_FINS (hCpswAleRegs->ALE_UVLAN_URCAST, ALE_ALE_UVLAN_RMCAST_UVLAN_REG_MCAST_FLOOD_MASK, aleUnknwnVlanUnregMcastVal);

    return;
}

void CSL_CPSW_setAleUnknwnVlanRegMcastReg
(
    CSL_AleRegs *hCpswAleRegs,
    Uint32                      aleUnknwnVlanRegMcastVal
)
{
    CSL_FINS (hCpswAleRegs->ALE_UVLAN_RMCAST, ALE_ALE_UVLAN_URCAST_UVLAN_UNREG_MCAST_FLOOD_MASK, aleUnknwnVlanRegMcastVal);

    return;
}

void CSL_CPSW_setAlePolicerControlReg(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_CONTROL *policerCntrlCfg)
{
    Uint32 alePolicerCtrlVal = 0;


    alePolicerCtrlVal       |=
        CSL_FMK (ALE_POLICECONTROL_POLMCHMODE, policerCntrlCfg->policeMatchMode);

    alePolicerCtrlVal       |=
        CSL_FMK (ALE_POLICECONTROL_POLICING_EN, policerCntrlCfg->policingEnable);

    alePolicerCtrlVal       |=
        CSL_FMK (ALE_POLICECONTROL_RED_DROP_EN, policerCntrlCfg->redDropEnable);

    alePolicerCtrlVal       |=
        CSL_FMK (ALE_POLICECONTROL_YELLOW_DROP_EN, policerCntrlCfg->yellowDropEnable);

    alePolicerCtrlVal       |=
        CSL_FMK (ALE_POLICECONTROL_YELLOWTHRESH, policerCntrlCfg->yellowDropThresh);

    alePolicerCtrlVal       |=
        CSL_FMK (ALE_POLICECONTROL_PRIORITY_THREAD_EN, policerCntrlCfg->enablePriorityOR);

    alePolicerCtrlVal       |=
        CSL_FMK (ALE_POLICECONTROL_MAC_ONLY_DEF_DIS, policerCntrlCfg->disableMacPortDefaultThread);

    hCpswAleRegs->POLICECONTROL = alePolicerCtrlVal;
}

void CSL_CPSW_getAlePolicerControlReg(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_CONTROL *policerCntrlCfg)
{
    Uint32 alePolicerCtrlVal = hCpswAleRegs->POLICECONTROL;


    policerCntrlCfg->policeMatchMode       =
        (CSL_CPSW_ALE_POLICER_CONTROL_POLICING_MATCH_MODE)CSL_FEXT (alePolicerCtrlVal,ALE_POLICECONTROL_POLMCHMODE);

    policerCntrlCfg->policingEnable       =
        CSL_FEXT (alePolicerCtrlVal,ALE_POLICECONTROL_POLICING_EN);

    policerCntrlCfg->redDropEnable       =
        CSL_FEXT (alePolicerCtrlVal,ALE_POLICECONTROL_RED_DROP_EN);

    policerCntrlCfg->yellowDropEnable       =
        CSL_FEXT (alePolicerCtrlVal,ALE_POLICECONTROL_YELLOW_DROP_EN);

    policerCntrlCfg->yellowDropThresh       =
        (CSL_CPSW_ALE_POLICER_CONTROL_YELLOWTHRESH)CSL_FEXT (alePolicerCtrlVal,ALE_POLICECONTROL_YELLOWTHRESH);

    policerCntrlCfg->enablePriorityOR       =
        CSL_FEXT (alePolicerCtrlVal,ALE_POLICECONTROL_PRIORITY_THREAD_EN);

    policerCntrlCfg->disableMacPortDefaultThread       =
        CSL_FEXT (alePolicerCtrlVal,ALE_POLICECONTROL_MAC_ONLY_DEF_DIS);

}

void CSL_CPSW_setAlePolicerTestControlReg(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_TEST_CONTROL *policerTestCntrlCfg)
{
    Uint32 alePolicerTestCntrlVal = 0;


    alePolicerTestCntrlVal |= CSL_FMK(ALE_POLICETESTCTL_POL_TEST_IDX        ,policerTestCntrlCfg->polTestIdx        );
    alePolicerTestCntrlVal |= CSL_FMK(ALE_POLICETESTCTL_POL_CLRSEL_ALL      ,policerTestCntrlCfg->polClrselAll      );
    alePolicerTestCntrlVal |= CSL_FMK(ALE_POLICETESTCTL_POL_CLRALL_YELLOWHIT,policerTestCntrlCfg->polClrallYellowhit);
    alePolicerTestCntrlVal |= CSL_FMK(ALE_POLICETESTCTL_POL_CLRALL_REDHIT   ,policerTestCntrlCfg->polClrallRedhit   );
    alePolicerTestCntrlVal |= CSL_FMK(ALE_POLICETESTCTL_POL_CLRALL_HIT      ,policerTestCntrlCfg->polClrallHit      );
    hCpswAleRegs->POLICETESTCTL = alePolicerTestCntrlVal;
}

void CSL_CPSW_getAlePolicerHstatReg(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_HSTAT *policerHStatCfg)
{
    Uint32 alePolicerHstatVal = hCpswAleRegs->POLICEHSTAT;

    policerHStatCfg->polHit        = CSL_FEXT(alePolicerHstatVal, ALE_POLICEHSTAT_POL_HIT       );
    policerHStatCfg->polRedhit     = CSL_FEXT(alePolicerHstatVal, ALE_POLICEHSTAT_POL_REDHIT    );
    policerHStatCfg->polYellowhit  = CSL_FEXT(alePolicerHstatVal, ALE_POLICEHSTAT_POL_YELLOWHIT );
}


/** ============================================================================
 *   @n@b CSL_CPSW_setAleOAMLpbkControl
 *
 * =============================================================================
 */
void CSL_CPSW_setAleOAMLpbkControl(CSL_AleRegs *hCpswAleRegs,
                                                   Uint32 lpbkEnablePortMask)
{
    CSL_FINS (hCpswAleRegs->ALE_OAM_LB_CTRL ,ALE_ALE_OAM_LB_CTRL_OAM_LB_CTRL,
              lpbkEnablePortMask);
}

void CSL_CPSW_getAleStatusNumPolicers(CSL_AleRegs *hCpswAleRegs,Uint32* pNumPolicers)
{
    *pNumPolicers     =   CSL_FEXT(hCpswAleRegs->ALE_STATUS, ALE_ALE_STATUS_POLCNTDIV8) << 3;

    if (*pNumPolicers == 0U) {
        /* On some SOCs like AWR294x/TPR12 the number of policers is less than 8 so ALE_ALE_STATUS_POLCNTDIV8 returns 0.
	    TO determine correct number of policers, set the POLICTBLCTL POL TBL IDX to 0x3 and read back. It will return number of actual policers are bits more than supported policer indexes will be tied to 0.We read the default value of POLICTBLCTL POL TBL IDX, set it to 0x3 then update the number of policers and set back the default value of POLICTBLCTL POL TBL IDX. */
	    Uint32 value = 0U;
	    value = CSL_FEXT (hCpswAleRegs->POLICETBLCTL, ALE_POLICETBLCTL_POL_TBL_IDX);
	    CSL_FINS (hCpswAleRegs->POLICETBLCTL ,ALE_POLICETBLCTL_POL_TBL_IDX, 0x3U);
        *pNumPolicers = CSL_FEXT(hCpswAleRegs->POLICETBLCTL, ALE_POLICETBLCTL_POL_TBL_IDX);
	    CSL_FINS (hCpswAleRegs->POLICETBLCTL ,ALE_POLICETBLCTL_POL_TBL_IDX, value);
    }
}

void CSL_CPSW_setCppiPriCirEir(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 cir, Uint32 eir)
{
    CSL_FINS (hCpswRegs->P0_PRI_CIR_REG[pri], XGE_CPSW_P0_PRI_CIR_REG_PRI_CIR, cir);
    CSL_FINS (hCpswRegs->P0_PRI_EIR_REG[pri], XGE_CPSW_P0_PRI_EIR_REG_PRI_EIR, eir);
}

void CSL_CPSW_getCppiPriCirEir(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 *cir, Uint32 *eir)
{
    *cir = CSL_FEXT (hCpswRegs->P0_PRI_CIR_REG[pri], XGE_CPSW_P0_PRI_CIR_REG_PRI_CIR);
    *eir = CSL_FEXT (hCpswRegs->P0_PRI_EIR_REG[pri], XGE_CPSW_P0_PRI_EIR_REG_PRI_EIR);
}

void CSL_CPSW_setPriCirEir(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 cir, Uint32 eir)
{
    CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_PRI_CIR_REG[pri], XGE_CPSW_PN_PRI_CIR_REG_PRI_CIR, cir);
    CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_PRI_EIR_REG[pri], XGE_CPSW_PN_PRI_EIR_REG_PRI_EIR, eir);
}

void CSL_CPSW_getPriCirEir(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 *cir, Uint32 *eir)
{
    *cir = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_PRI_CIR_REG[pri], XGE_CPSW_PN_PRI_CIR_REG_PRI_CIR);
    *eir = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_PRI_EIR_REG[pri], XGE_CPSW_PN_PRI_EIR_REG_PRI_EIR);
}

void CSL_CPSW_setCppiRxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint8 pri)
{
    CSL_FINS (hCpswRegs->P0_PRI_CTL_REG, XGE_CPSW_P0_PRI_CTL_REG_RX_FLOW_PRI, pri);
}

Uint8 CSL_CPSW_getCppiRxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs)
{
    return CSL_FEXT (hCpswRegs->P0_PRI_CTL_REG, XGE_CPSW_P0_PRI_CTL_REG_RX_FLOW_PRI);
}

void CSL_CPSW_setCppiTxDstThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_SET_L_REG, XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_SET_L_REG, XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_SET_L_REG, XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_SET_L_REG, XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_SET_H_REG, XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_SET_H_REG, XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_SET_H_REG, XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_SET_H_REG, XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getCppiTxDstThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_SET_L_REG, XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_SET_L_REG, XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_SET_L_REG, XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_SET_H_REG, XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_SET_H_REG, XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_SET_H_REG, XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_SET_H_REG, XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setCppiTxDstThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->P0_TX_D_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getCppiDstTxThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->P0_TX_D_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setCppiTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getCppiTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setCppiTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getCppiTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->P0_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setcppiTxBlksPri(CSL_Xge_cpswRegs *hCpswRegs,Uint8 pri, Uint32 blks)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI0, blks);
		break;
		case 1:
			CSL_FINS (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI1, blks);
		break;
		case 2:
			CSL_FINS (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI2, blks);
		break;
		case 3:
			CSL_FINS (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI3, blks);
		break;
		case 4:
			CSL_FINS (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI4, blks);
		break;
		case 5:
			CSL_FINS (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI5, blks);
		break;
		case 6:
			CSL_FINS (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI6, blks);
		break;
		case 7:
			CSL_FINS (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI7, blks);
		break;
		default: break;
	}
}

Uint32 CSL_CPSW_getCppiTxBlksPri(CSL_Xge_cpswRegs *hCpswRegs,Uint8 pri)
{
	uint32_t value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->P0_TX_BLKS_PRI_REG, XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI7);
		break;
		default: break;
	}

	return value;
}

void CSL_CPSW_setTxBlksPri(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint8 pri, Uint32 blks)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI0, blks);
		break;
		case 1:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI1, blks);
		break;
		case 2:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI2, blks);
		break;
		case 3:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI3, blks);
		break;
		case 4:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI4, blks);
		break;
		case 5:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI5, blks);
		break;
		case 6:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI6, blks);
		break;
		case 7:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI7, blks);
		break;
		default: break;
	}
}

Uint32 CSL_CPSW_getTxBlksPri(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint8 pri)
{
	uint32_t value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_BLKS_PRI_REG, XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI7);
		break;
		default: break;
	}
	return value;
}

void CSL_CPSW_setTxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint8 pri)
{
    CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_PRI_CTL_REG, XGE_CPSW_PN_PRI_CTL_REG_TX_FLOW_PRI, pri);
}

Uint8 CSL_CPSW_getTxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo)
{
	return CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_PRI_CTL_REG, XGE_CPSW_PN_PRI_CTL_REG_TX_FLOW_PRI);
}

void CSL_CPSW_setRxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint8 pri)
{
	CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_PRI_CTL_REG, XGE_CPSW_PN_PRI_CTL_REG_RX_FLOW_PRI, pri);
}

Uint8 CSL_CPSW_getRxPriFlowControl(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo)
{
	return CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_PRI_CTL_REG, XGE_CPSW_PN_PRI_CTL_REG_RX_FLOW_PRI);
}

Uint32 CSL_CPSW_getTxHostBlksRem(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo)
{
	return CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_PRI_CTL_REG, XGE_CPSW_PN_PRI_CTL_REG_TX_HOST_BLKS_REM);
}

void CSL_CPSW_setTxHostBlksRem(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 txBlksRem)
{
	CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_PRI_CTL_REG, XGE_CPSW_PN_PRI_CTL_REG_TX_HOST_BLKS_REM, txBlksRem);
}

void CSL_CPSW_setTxDstThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_H_REG, XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_H_REG, XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_H_REG, XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_H_REG, XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getTxDstThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_L_REG, XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_H_REG, XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_H_REG, XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_H_REG, XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_SET_H_REG, XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setTxDstThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getTxDstThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setTxDstBasedOutFlowAddValX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri, Uint32 addVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_L_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI0, addVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_L_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI1, addVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_L_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI2, addVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_L_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI3, addVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_H_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI4, addVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_H_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI5, addVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_H_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI6, addVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_H_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI7, addVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getTxDstBasedOutFlowAddValX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 portNo, Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_L_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_L_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_L_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_L_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_H_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_H_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_H_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->ENETPORT[portNo].PN_TX_D_OFLOW_ADDVAL_H_REG, XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setTxGlobalOutFlowThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getTxGlobalOutFlowThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_SET_REG, XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}

void CSL_CPSW_setTxGlobalOutFlowThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getTxGlobalOutFlowThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->TX_G_OFLOW_THRESH_CLR_REG, XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI7);
		break;
		default:
		    break;
	}

	return value;
}


void CSL_CPSW_setCpswTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getCpswTxGlobalBufThresholdSetX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri)
{
	Uint32 value = 0;

	switch(pri)
	{
		case 0:
			value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI0);
		break;
		case 1:
			value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI1);
		break;
		case 2:
			value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI2);
		break;
		case 3:
			value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI3);
		break;
		case 4:
			value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI4);
		break;
		case 5:
			value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI5);
		break;
		case 6:
			value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI6);
		break;
		case 7:
			value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_SET_H_REG, XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI7);
		break;
		default:
		    break;
	}
	return value;
}

void CSL_CPSW_setCpswTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri, Uint32 setVal)
{
	switch(pri)
	{
		case 0:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI0, setVal);
		break;
		case 1:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI1, setVal);
		break;
		case 2:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI2, setVal);
		break;
		case 3:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_SET_L_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI3, setVal);
		break;
		case 4:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI4, setVal);
		break;
		case 5:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI5, setVal);
		break;
		case 6:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI6, setVal);
		break;
		case 7:
			CSL_FINS (hCpswRegs->TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI7, setVal);
		break;
		default:
		    break;
	}
}

Uint32 CSL_CPSW_getCpswTxGlobalBufThresholdClrX(CSL_Xge_cpswRegs *hCpswRegs,Uint32 pri)
{
    Uint32 value = 0;

    switch(pri)
    {
        case 0:
            value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI0);
            break;
        case 1:
            value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI1);
            break;
        case 2:
            value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI2);
            break;
        case 3:
            value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_CLR_L_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI3);
            break;
        case 4:
            value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI4);
            break;
        case 5:
            value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI5);
            break;
        case 6:
            value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI6);
            break;
        case 7:
            value = CSL_FEXT (hCpswRegs->TX_G_BUF_THRESH_CLR_H_REG, XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI7);
            break;
        default:
            break;
    }
    return value;
}

#if ENET_CFG_IS_ON(CPSW_CPPI_CAST)
/** ============================================================================
 *   @n@b CSL_CPSW_isP0TxCastagnoliCRCEnabled
 *
 *   @b Description
 *   @n This function indicates if Castagnoli CRC is enabled in the CPSW
 *      control register for host port.
 *
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Castagnoli CRC enabled
 *	 @n  FALSE                  Castagnoli CRC disabled.
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_isP0TxCastagnoliCRCEnabled (CSL_Xge_cpswRegs *hCpswRegs)
{

    return CSL_FEXT (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_TX_CRC_TYPE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_enableP0TxCastagnoliCRC
 *
 *   @b Description
 *   @n This function configures the CPSW control register to enable Castagnoli CRC for
 *      host port specified.
 *   <b> Return Value </b>
 *	 @n	 None
 * =============================================================================
 */
void CSL_CPSW_enableP0TxCastagnoliCRC(CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_TX_CRC_TYPE, 1);

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_disableP0TxCastagnoliCRC
 *
 *   @b Description
 *   @n This function configures the CPSW control register to disable Castagnoli CRC for
 *      host port.
 *
 *   <b> Return Value </b>
 *   @n None
 * =============================================================================
 */
void CSL_CPSW_disableP0TxCastagnoliCRC(CSL_Xge_cpswRegs *hCpswRegs)
{
    CSL_FINS (hCpswRegs->CONTROL_REG, XGE_CPSW_CONTROL_REG_P0_TX_CRC_TYPE, 0);

    return;
}
#endif

/** ============================================================================
 *   @n@b CSL_CPSW_getPTypeReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW PTYPE register.
 *
 *   @b Arguments
     @verbatim
        pPtypeCfg       CSL_CPSW_PTYPE structure that needs to be populated
                        with the CPSW PTYPE register contents.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 * =============================================================================
 */
void CSL_CPSW_getPTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
                           CSL_CPSW_PTYPE*  pPtypeCfg)
{
    pPtypeCfg->port0PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P0_PTYPE_ESC);
    pPtypeCfg->port1PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P1_PTYPE_ESC);
    pPtypeCfg->port2PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P2_PTYPE_ESC);
    pPtypeCfg->port3PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P3_PTYPE_ESC);
    pPtypeCfg->port4PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P4_PTYPE_ESC);
    pPtypeCfg->port5PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P5_PTYPE_ESC);
    pPtypeCfg->port6PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P6_PTYPE_ESC);
    pPtypeCfg->port7PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P7_PTYPE_ESC);
    pPtypeCfg->port8PriorityTypeEscalateEnable    =   CSL_FEXT (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P8_PTYPE_ESC);
    pPtypeCfg->escPriLoadVal   =   CSL_FEXT (hCpswRegs->PTYPE_REG,XGE_CPSW_PTYPE_REG_ESC_PRI_LD_VAL);
    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setPTypeReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW PTYPE register.
 *
 *   @b Arguments
     @verbatim
        pPtypeCfg       CSL_CPSW_PTYPE structure that has the
                        CPSW PTYPE register configuration.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 * =============================================================================
 */
void CSL_CPSW_setPTypeReg (CSL_Xge_cpswRegs *hCpswRegs,
                           CSL_CPSW_PTYPE*  pPtypeCfg)
{
    CSL_FINS (hCpswRegs->PTYPE_REG,XGE_CPSW_PTYPE_REG_ESC_PRI_LD_VAL,pPtypeCfg->escPriLoadVal);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P0_PTYPE_ESC, pPtypeCfg->port0PriorityTypeEscalateEnable);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P1_PTYPE_ESC, pPtypeCfg->port1PriorityTypeEscalateEnable);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P2_PTYPE_ESC, pPtypeCfg->port2PriorityTypeEscalateEnable);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P3_PTYPE_ESC, pPtypeCfg->port3PriorityTypeEscalateEnable);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P4_PTYPE_ESC, pPtypeCfg->port4PriorityTypeEscalateEnable);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P5_PTYPE_ESC, pPtypeCfg->port5PriorityTypeEscalateEnable);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P6_PTYPE_ESC, pPtypeCfg->port6PriorityTypeEscalateEnable);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P7_PTYPE_ESC, pPtypeCfg->port7PriorityTypeEscalateEnable);
    CSL_FINS (hCpswRegs->PTYPE_REG, XGE_CPSW_PTYPE_REG_P8_PTYPE_ESC, pPtypeCfg->port8PriorityTypeEscalateEnable);
    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getThruRateReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the CPSW_THRU_RATE register.
 *
 *   @b Arguments
     @verbatim
        pThruRateCfg    CSL_CPSW_THRURATE structure that needs to be populated
                        with the CPSW_THRU_RATE register contents.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 * =============================================================================
 */
void CSL_CPSW_getThruRateReg (CSL_Xge_cpswRegs *hCpswRegs,
                              CSL_CPSW_THRURATE *  pThruRateCfg)
{
    pThruRateCfg->cppiRxThruRate =
    CSL_FEXT (hCpswRegs->THRU_RATE_REG,XGE_CPSW_THRU_RATE_REG_P0_RX_THRU_RATE);
    pThruRateCfg->enetRxThruRate =
    CSL_FEXT (hCpswRegs->THRU_RATE_REG,XGE_CPSW_THRU_RATE_REG_SL_RX_THRU_RATE);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getGapThreshold
 *
 *   @b Description
 *   @n This function retrieves the contents of the GAP_THRESH_REG register.
 *
 *  The Ethernet port transmit inter-packet gap (IPG) may be shortened by eight
 *  bit times when short gap is enabled and triggered.
 *  Setting the pn_tx_short_gap_en bit each Enet_Pn_Mac_Control register
 *  enables the gap to be shortened when triggered.
 *  The condition is triggered when the ports associated transmit packet FIFO
 *  has a user defined number of FIFO blocks used.
 *  The associated transmit FIFO blocks used value determines
 *  if the gap is shortened, and so on.
 *  The Gap_Thresh register value determines the short gap threshold.
 *  If the FIFO blocks used is greater than or equal to the Gap_Thresh value
 *  then short gap is triggered
 *   @b Arguments
 *    @verbatim
 *       none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Gap threshold configured
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getGapThreshold(CSL_Xge_cpswRegs *hCpswRegs)
{
    return (CSL_FEXT (hCpswRegs->GAP_THRESH_REG,
                      XGE_CPSW_GAP_THRESH_REG_GAP_THRESH));
}

/** ============================================================================
 *   @n@b CSL_CPSW_setGapThreshold
 *
 *   @b Description
 *   @n This function retrieves the contents of the GAP_THRESH_REG register.
 *
 *  The Ethernet port transmit inter-packet gap (IPG) may be shortened by eight
 *  bit times when short gap is enabled and triggered.
 *  Setting the pn_tx_short_gap_en bit each Enet_Pn_Mac_Control register
 *  enables the gap to be shortened when triggered.
 *  The condition is triggered when the ports associated transmit packet FIFO
 *  has a user defined number of FIFO blocks used.
 *  The associated transmit FIFO blocks used value determines
 *  if the gap is shortened, and so on.
 *  The Gap_Thresh register value determines the short gap threshold.
 *  If the FIFO blocks used is greater than or equal to the Gap_Thresh value
 *  then short gap is triggered
 *   @b Arguments
 *    @verbatim
 *       gapThreshold     Ethernet Port  Short Gap Threshold -
 *                        This is the Ethernet port associated FIFO transmit
 *                        block usage value for triggering transmit short gap
 *                        (when short gap is enabled)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Gap threshold configured
 *
 * =============================================================================
 */
void CSL_CPSW_setGapThreshold(CSL_Xge_cpswRegs *hCpswRegs,Uint32 gapThreshold)
{
    CSL_FINS(hCpswRegs->GAP_THRESH_REG, XGE_CPSW_GAP_THRESH_REG_GAP_THRESH,
             gapThreshold);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getTxStartWdsReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the TX_START_WDS_REG register.
 *
 *   FIFO Packet Transmit (egress) Start Words.
 *   This value is the number of required 32-byte packet words in an Ethernet
 *   transmit FIFO before the packet egress will begin.
 *   This value is non-zero to preclude Ethernet transmit underrun.
 *   Decimal 8 is the recommended value.
 *   It should not be increased unnecessarily to prevent adding to the
 *   switch latency
 *
 *   @b Arguments
 *    @verbatim
 *       none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Gap threshold configured
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getTxStartWords(CSL_Xge_cpswRegs *hCpswRegs)
{

    return (CSL_FEXT(hCpswRegs->TX_START_WDS_REG,XGE_CPSW_TX_START_WDS_REG_TX_START_WDS));
}

/** ============================================================================
 *   @n@b CSL_CPSW_getTxMaxLenPerPriority
 *
 *   @b Description
 *   @n This function retrieves the max tx packet length per switch priority.
 *
 *   @b Arguments
 *    @verbatim
 *       priority   Priority for which tx max packet length is queried (0 -7)
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Max tx packet length for given priority
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getTxMaxLenPerPriority(CSL_Xge_cpswRegs *hCpswRegs,
                                       Uint32 priority)
{
    Uint32 maxLen = 0;

    switch (priority)
    {
        case 0:
            maxLen = CSL_FEXT(hCpswRegs->TX_PRI0_MAXLEN_REG,
                     XGE_CPSW_TX_PRI0_MAXLEN_REG_TX_PRI0_MAXLEN);
            break;
        case 1:
            maxLen = CSL_FEXT(hCpswRegs->TX_PRI1_MAXLEN_REG,
                     XGE_CPSW_TX_PRI1_MAXLEN_REG_TX_PRI1_MAXLEN);
            break;
        case 2:
            maxLen = CSL_FEXT(hCpswRegs->TX_PRI2_MAXLEN_REG,
                     XGE_CPSW_TX_PRI2_MAXLEN_REG_TX_PRI2_MAXLEN);
            break;
        case 3:
            maxLen = CSL_FEXT(hCpswRegs->TX_PRI3_MAXLEN_REG,
                     XGE_CPSW_TX_PRI3_MAXLEN_REG_TX_PRI3_MAXLEN);
            break;
        case 4:
            maxLen = CSL_FEXT(hCpswRegs->TX_PRI4_MAXLEN_REG,
                     XGE_CPSW_TX_PRI4_MAXLEN_REG_TX_PRI4_MAXLEN);
            break;
        case 5:
            maxLen = CSL_FEXT(hCpswRegs->TX_PRI5_MAXLEN_REG,
                     XGE_CPSW_TX_PRI5_MAXLEN_REG_TX_PRI5_MAXLEN);
            break;
        case 6:
            maxLen = CSL_FEXT(hCpswRegs->TX_PRI6_MAXLEN_REG,
                     XGE_CPSW_TX_PRI6_MAXLEN_REG_TX_PRI6_MAXLEN);
            break;
        case 7:
            maxLen = CSL_FEXT(hCpswRegs->TX_PRI7_MAXLEN_REG,
                     XGE_CPSW_TX_PRI7_MAXLEN_REG_TX_PRI7_MAXLEN);
            break;
    }
    return maxLen;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setTxMaxLenPerPriority
 *
 *   @b Description
 *   @n This function sets the max tx packet length per switch priority.
 *
 *   @b Arguments
 *    @verbatim
 *       priority   Priority for which tx max packet length is set (0 -7)
 *       maxLen     Tx max packet length to be set
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  Max tx packet length for given priority
 *
 * =============================================================================
 */
void  CSL_CPSW_setTxMaxLenPerPriority(CSL_Xge_cpswRegs *hCpswRegs,
                                       Uint32 priority,
                                       Uint32 maxLen)
{
    switch (priority)
    {
        case 0:
            CSL_FINS(hCpswRegs->TX_PRI0_MAXLEN_REG,
                     XGE_CPSW_TX_PRI0_MAXLEN_REG_TX_PRI0_MAXLEN,
                     maxLen);
            break;
        case 1:
            CSL_FINS(hCpswRegs->TX_PRI1_MAXLEN_REG,
                     XGE_CPSW_TX_PRI1_MAXLEN_REG_TX_PRI1_MAXLEN,
                     maxLen);
            break;
        case 2:
            CSL_FINS(hCpswRegs->TX_PRI2_MAXLEN_REG,
                     XGE_CPSW_TX_PRI2_MAXLEN_REG_TX_PRI2_MAXLEN,
                     maxLen);
            break;
        case 3:
            CSL_FINS(hCpswRegs->TX_PRI3_MAXLEN_REG,
                     XGE_CPSW_TX_PRI3_MAXLEN_REG_TX_PRI3_MAXLEN,
                     maxLen);
            break;
        case 4:
            CSL_FINS(hCpswRegs->TX_PRI4_MAXLEN_REG,
                     XGE_CPSW_TX_PRI4_MAXLEN_REG_TX_PRI4_MAXLEN,
                     maxLen);
            break;
        case 5:
            CSL_FINS(hCpswRegs->TX_PRI5_MAXLEN_REG,
                     XGE_CPSW_TX_PRI5_MAXLEN_REG_TX_PRI5_MAXLEN,
                     maxLen);
            break;
        case 6:
            CSL_FINS(hCpswRegs->TX_PRI6_MAXLEN_REG,
                     XGE_CPSW_TX_PRI6_MAXLEN_REG_TX_PRI6_MAXLEN,
                     maxLen);
            break;
        case 7:
            CSL_FINS(hCpswRegs->TX_PRI7_MAXLEN_REG,
                     XGE_CPSW_TX_PRI7_MAXLEN_REG_TX_PRI7_MAXLEN,
                     maxLen);
            break;
    }
}

/** ============================================================================
 *   @n@b CSL_CPSW_getCppiP0Control
 *
 *   @b Description
 *   @n This function gets the P0_CONTROL_REG register contents.
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiP0ControlCfg   P0_CONTROL_REG configuration structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void  CSL_CPSW_getCppiP0Control(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_CONTROL *pCppiP0ControlCfg)
{
#if ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT)
    pCppiP0ControlCfg->p0RxChksumEn =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_CHECKSUM_EN);
#if ((ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_MAJOR_VER >= 1) && \
     (ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_MINOR_VER >= 3) && \
     (ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_RTL_VER >= 2))
    pCppiP0ControlCfg->p0TxChksumEn =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_TX_CHECKSUM_EN);
#endif
#endif
    pCppiP0ControlCfg->p0DscpIpv4En =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN);

    pCppiP0ControlCfg->p0DscpIpv6En =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN);

    pCppiP0ControlCfg->p0TxEccErrEn =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_TX_ECC_ERR_EN);

    pCppiP0ControlCfg->p0RxEccErrEn =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_ECC_ERR_EN);

    pCppiP0ControlCfg->p0RxRemapVlan =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_REMAP_VLAN);

    pCppiP0ControlCfg->p0RxRemapDscpIpv4 =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V4);

    pCppiP0ControlCfg->p0RxRemapDscpIpv6 =
    CSL_FEXT(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V6);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiP0Control
 *
 *   @b Description
 *   @n This function sets the P0_CONTROL_REG register contents.
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiP0ControlCfg   P0_CONTROL_REG configuration structure
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void  CSL_CPSW_setCppiP0Control(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_CONTROL *pCppiP0ControlCfg)
{
#if ENET_CFG_IS_ON(CPSW_CSUM_OFFLOAD_SUPPORT)
    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_CHECKSUM_EN,
             pCppiP0ControlCfg->p0RxChksumEn);
#if ((ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_MAJOR_VER >= 1) && \
     (ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_MINOR_VER >= 3) && \
     (ENET_SOC_CFG_XGE_CPSW_ID_VER_REG_RTL_VER >= 2))
    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_TX_CHECKSUM_EN,
             pCppiP0ControlCfg->p0TxChksumEn);
#endif
#endif
    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN,
             pCppiP0ControlCfg->p0DscpIpv4En);

    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN,
             pCppiP0ControlCfg->p0DscpIpv6En);

    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_TX_ECC_ERR_EN,
             pCppiP0ControlCfg->p0TxEccErrEn);

    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_ECC_ERR_EN,
             pCppiP0ControlCfg->p0RxEccErrEn);

    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_REMAP_VLAN,
             pCppiP0ControlCfg->p0RxRemapVlan);

    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V4,
             pCppiP0ControlCfg->p0RxRemapDscpIpv4);

    CSL_FINS(hCpswRegs->P0_CONTROL_REG,XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V6,
             pCppiP0ControlCfg->p0RxRemapDscpIpv6);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiRxPType
 *
 *   @b Description
 *   @n This function sets the P0 Receive priority type
 *
 *   @b Arguments
 *    @verbatim
 *       p0RxPtype   Receive Priority Type
 *                   0 - Fixed priority
 *                   1 - Round Robin priority
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_setCppiRxPType(CSL_Xge_cpswRegs *hCpswRegs,Uint32 p0RxPtype)
{
    CSL_FINS (hCpswRegs->P0_PRI_CTL_REG, XGE_CPSW_P0_PRI_CTL_REG_RX_PTYPE, p0RxPtype);
}

/** ============================================================================
 *   @n@b CSL_CPSW_getCppiRxPType
 *
 *   @b Description
 *   @n This function gets the P0 Receive priority type
 *
 *   @b Arguments
 *    @verbatim
 *       none
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  p0RxPtype   Receive Priority Type
 *                   0 - Fixed priority
 *                   1 - Round Robin priority
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getCppiRxPType(CSL_Xge_cpswRegs *hCpswRegs)
{
    return (CSL_FEXT (hCpswRegs->P0_PRI_CTL_REG, XGE_CPSW_P0_PRI_CTL_REG_RX_PTYPE));
}

#if ENET_CFG_IS_ON(CPSW_2PORTSWITCH)
/** ============================================================================
 *   @n@b CSL_CPSW_getCppiRxPacketsPriority
 *
 *   @b Description
 *   @n This function gets Port 0 Receive (same as Port 1 Transmit) Packets Per
 *      Priority
 *      This function is applicable for 2 port switch
 *
 *   @b Arguments
 *    @verbatim
 *       priority  Priority for which number of rx packets is queried
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  rxPackets   Number of rx packets for queried priority
 *
 * =============================================================================
 */
Uint32 CSL_CPSW_getCppiRxPacketsPriority(CSL_Xge_cpswRegs *hCpswRegs,
                                         Uint32 priority)
{
    Uint32 rxPackets = 0;

    switch(priority)
    {
        case 0:
            rxPackets =
            CSL_FEXT (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI0);
            break;
        case 1:
            rxPackets =
            CSL_FEXT (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI1);
            break;
        case 2:
            rxPackets =
            CSL_FEXT (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI2);
            break;
        case 3:
            rxPackets =
            CSL_FEXT (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI3);
            break;
        case 4:
            rxPackets =
            CSL_FEXT (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI4);
            break;
        case 5:
            rxPackets =
            CSL_FEXT (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI5);
            break;
        case 6:
            rxPackets =
            CSL_FEXT (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI6);
            break;
        case 7:
            rxPackets =
            CSL_FEXT (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI7);
            break;
    }
    return rxPackets;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiRxPacketsPriority
 *
 *   @b Description
 *   @n This function gets Port 0 Receive (same as Port 1 Transmit) Packets Per
 *      Priority
 *      This function is applicable for 2 port switch
 *
 *   @b Arguments
 *    @verbatim
 *       priority  Priority for which number of rx packets is to be set
 *       rxPackets Number of rx packets to be set
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_setCppiRxPacketsPriority(CSL_Xge_cpswRegs *hCpswRegs,
                                       Uint32 priority,
                                       Uint32 rxPackets)
{
    switch(priority)
    {
        case 0:
            CSL_FINS (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI0,rxPackets);
            break;
        case 1:
            CSL_FINS (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI1,rxPackets);
            break;
        case 2:
            CSL_FINS (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI2,rxPackets);
            break;
        case 3:
            CSL_FINS (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI3,rxPackets);
            break;
        case 4:
            CSL_FINS (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI4,rxPackets);
            break;
        case 5:
            CSL_FINS (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI5,rxPackets);
            break;
        case 6:
            CSL_FINS (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI6,rxPackets);
            break;
        case 7:
            CSL_FINS (hCpswRegs->P0_RX_PKTS_PRI_REG,XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI7,rxPackets);
            break;
    }
}

/** ============================================================================
 *   @n@b CSL_CPSW_getCppiRxGapReg
 *
 *   @b Description
 *   @n This function gets CPPI_P0_Rx_Gap
 *      This function is applicable for 2 port switch
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiRxGap  CPPI_P0_Rx_Gap register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_getCppiRxGapReg(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_RXGAP *pCppiRxGap)
{
    Uint32 rxGapEnPriMask;

    pCppiRxGap->rxGapCnt =
         CSL_FEXT (hCpswRegs->P0_RX_GAP_REG,XGE_CPSW_P0_RX_GAP_REG_RX_GAP_CNT);
    rxGapEnPriMask =
         CSL_FEXT (hCpswRegs->P0_RX_GAP_REG,XGE_CPSW_P0_RX_GAP_REG_RX_GAP_EN);
    pCppiRxGap->rxGapEnPri0 = (rxGapEnPriMask >> 0) & 0x1;
    pCppiRxGap->rxGapEnPri1 = (rxGapEnPriMask >> 1) & 0x1;
    pCppiRxGap->rxGapEnPri2 = (rxGapEnPriMask >> 2) & 0x1;
    pCppiRxGap->rxGapEnPri3 = (rxGapEnPriMask >> 3) & 0x1;
    pCppiRxGap->rxGapEnPri4 = (rxGapEnPriMask >> 4) & 0x1;
    pCppiRxGap->rxGapEnPri5 = (rxGapEnPriMask >> 5) & 0x1;
    pCppiRxGap->rxGapEnPri6 = (rxGapEnPriMask >> 6) & 0x1;
    pCppiRxGap->rxGapEnPri7 = (rxGapEnPriMask >> 7) & 0x1;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setCppiRxGapReg
 *
 *   @b Description
 *   @n This function sets CPPI_P0_Rx_Gap
 *      This function is applicable for 2 port switch
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiRxGap  CPPI_P0_Rx_Gap register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_setCppiRxGapReg(CSL_Xge_cpswRegs *hCpswRegs,
                                CSL_CPSW_CPPI_P0_RXGAP *pCppiRxGap)
{
    Uint32 rxGapEnPriMask;

    rxGapEnPriMask = ((pCppiRxGap->rxGapEnPri0 & 0x1) << 0) |
                     ((pCppiRxGap->rxGapEnPri1 & 0x1) << 1) |
                     ((pCppiRxGap->rxGapEnPri2 & 0x1) << 2) |
                     ((pCppiRxGap->rxGapEnPri3 & 0x1) << 3) |
                     ((pCppiRxGap->rxGapEnPri4 & 0x1) << 4) |
                     ((pCppiRxGap->rxGapEnPri5 & 0x1) << 5) |
                     ((pCppiRxGap->rxGapEnPri6 & 0x1) << 6) |
                     ((pCppiRxGap->rxGapEnPri7 & 0x1) << 7);
    CSL_FINS (hCpswRegs->P0_RX_GAP_REG,XGE_CPSW_P0_RX_GAP_REG_RX_GAP_CNT,pCppiRxGap->rxGapCnt);
    CSL_FINS (hCpswRegs->P0_RX_GAP_REG,XGE_CPSW_P0_RX_GAP_REG_RX_GAP_EN,rxGapEnPriMask);
}
#endif

/** ============================================================================
 *   @n@b CSL_CPSW_getP0FifoStatus
 *
 *   @b Description
 *   @n This function gets CPPI_P0_FIFO_Status register
 *      This function is will return 0 for  2 port switch as there is no Tx FIFO
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiFifoStats  CPPI_P0_FIFO_Status register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_getP0FifoStatus(CSL_Xge_cpswRegs *hCpswRegs,
                              CSL_CPSW_CPPI_P0_FIFOSTATUS *pCppiFifoStats)
{
    Uint32 fifoStatusPriActiveMask =
    CSL_FEXT(hCpswRegs->P0_FIFO_STATUS_REG,
             XGE_CPSW_P0_FIFO_STATUS_REG_TX_PRI_ACTIVE);

    pCppiFifoStats->p0TxPriActivePri0 = (fifoStatusPriActiveMask >> 0) & 0x1;
    pCppiFifoStats->p0TxPriActivePri1 = (fifoStatusPriActiveMask >> 1) & 0x1;
    pCppiFifoStats->p0TxPriActivePri2 = (fifoStatusPriActiveMask >> 2) & 0x1;
    pCppiFifoStats->p0TxPriActivePri3 = (fifoStatusPriActiveMask >> 3) & 0x1;
    pCppiFifoStats->p0TxPriActivePri4 = (fifoStatusPriActiveMask >> 4) & 0x1;
    pCppiFifoStats->p0TxPriActivePri5 = (fifoStatusPriActiveMask >> 5) & 0x1;
    pCppiFifoStats->p0TxPriActivePri6 = (fifoStatusPriActiveMask >> 6) & 0x1;
    pCppiFifoStats->p0TxPriActivePri7 = (fifoStatusPriActiveMask >> 7) & 0x1;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getP0HostBlksPri
 *
 *   @b Description
 *   @n This function gets CPPI_P0_Host_Blks_Pri  register
 *      This function is not applicable to two port switch
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiHostBlksPri  CPPI_P0_Host_Blks_Pri register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_getP0HostBlksPri(CSL_Xge_cpswRegs *hCpswRegs,
                               CSL_CPSW_CPPI_P0_HOSTBLKSPRI *pCppiHostBlksPri)
{
    pCppiHostBlksPri->p0HostBlksPri0 =
    CSL_FEXT(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI0);
    pCppiHostBlksPri->p0HostBlksPri1 =
    CSL_FEXT(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI1);
    pCppiHostBlksPri->p0HostBlksPri2 =
    CSL_FEXT(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI2);
    pCppiHostBlksPri->p0HostBlksPri3 =
    CSL_FEXT(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI3);
    pCppiHostBlksPri->p0HostBlksPri4 =
    CSL_FEXT(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI4);
    pCppiHostBlksPri->p0HostBlksPri5 =
    CSL_FEXT(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI5);
    pCppiHostBlksPri->p0HostBlksPri6 =
    CSL_FEXT(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI6);
    pCppiHostBlksPri->p0HostBlksPri7 =
    CSL_FEXT(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI7);
}

/** ============================================================================
 *   @n@b CSL_CPSW_setP0HostBlksPri
 *
 *   @b Description
 *   @n This function sets CPPI_P0_Host_Blks_Pri  register
 *      This function is not applicable to two port switch
 *
 *   @b Arguments
 *    @verbatim
 *       pCppiHostBlksPri  CPPI_P0_Host_Blks_Pri register configuration
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPSW_setP0HostBlksPri(CSL_Xge_cpswRegs *hCpswRegs,
                               CSL_CPSW_CPPI_P0_HOSTBLKSPRI *pCppiHostBlksPri)
{
    CSL_FINS(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI0,
             pCppiHostBlksPri->p0HostBlksPri0);
    CSL_FINS(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI1,
             pCppiHostBlksPri->p0HostBlksPri1);
    CSL_FINS(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI2,
             pCppiHostBlksPri->p0HostBlksPri2);
    CSL_FINS(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI3,
             pCppiHostBlksPri->p0HostBlksPri3);
    CSL_FINS(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI4,
             pCppiHostBlksPri->p0HostBlksPri4);
    CSL_FINS(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI5,
             pCppiHostBlksPri->p0HostBlksPri5);
    CSL_FINS(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI6,
             pCppiHostBlksPri->p0HostBlksPri6);
    CSL_FINS(hCpswRegs->P0_HOST_BLKS_PRI_REG,XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI7,
             pCppiHostBlksPri->p0HostBlksPri7);
}


/**
@}
*/
