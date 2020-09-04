/**
 * @file  csl_cpsw_ale_4g_tblcfg.c
 *
 * @brief
 *  API Function layer file for Ethernet switch module ALE table configuration CSL .
 *
 *  Contains the different functions definitions for configuring ALE
 *  table for CPSW 4 port ALE
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
#include <kernel/dpl/DebugP.h>
#include <cslr_ale.h>
#include <cslr_xge_cpsw.h>
#include <cpsw/V5/cslr_ale_tblwd_cpsw9g.h>
#include <csl_cpswitch.h>

#define CSL_CPSW_ALE_MACADDR_GET0(word1)       (CSL_FEXT((word1), \
                                                     ALE_TABLE_WORD1_REG_ENTRY_TYPE_ALLADDR_MACADDR_0))
#define CSL_CPSW_ALE_MACADDR_GET1(word1)       (CSL_FEXT((word1), \
                                                     ALE_TABLE_WORD1_REG_ENTRY_TYPE_ALLADDR_MACADDR_1))
#define CSL_CPSW_ALE_MACADDR_GET2(word0)       (CSL_FEXT((word0), \
                                                     ALE_TABLE_WORD0_REG_ENTRY_TYPE_ALLADDR_MACADDR_2))
#define CSL_CPSW_ALE_MACADDR_GET3(word0)       (CSL_FEXT((word0), \
                                                     ALE_TABLE_WORD0_REG_ENTRY_TYPE_ALLADDR_MACADDR_3))
#define CSL_CPSW_ALE_MACADDR_GET4(word0)       (CSL_FEXT((word0), \
                                                     ALE_TABLE_WORD0_REG_ENTRY_TYPE_ALLADDR_MACADDR_4))
#define CSL_CPSW_ALE_MACADDR_GET5(word0)       (CSL_FEXT((word0), \
                                                     ALE_TABLE_WORD0_REG_ENTRY_TYPE_ALLADDR_MACADDR_5))


/********************************************************************************
************************* Ethernet Switch (CPSW) Submodule **********************
********************************************************************************/

static void CSL_CPSW_initAleTblWds(CSL_AleRegs *hCpswAleRegs)
{
    hCpswAleRegs->ALE_TBLW0 = 0;
    hCpswAleRegs->ALE_TBLW1 = 0;
    hCpswAleRegs->ALE_TBLW2 = 0;
}

static void CSL_CPSW_setAlePolicerType(CSL_AleRegs *hCpswAleRegs,CSL_CPSW_ALE_POLICER_ENTRYTYPE policerType)
{
    Uint32 policerRegVal = CSL_ALE_TABLE_POLICER_ENUM2REG(policerType);

    CSL_ALE_TABLE_SET_POLICER_POLICERTYPE_BIT2((policerRegVal >> 2) & 0x1);
    CSL_ALE_TABLE_SET_POLICER_POLICERTYPE_BIT0TO1(policerRegVal & 0x3);
}


/** ============================================================================
 *   @n@b CSL_CPSW_getALEEntryType
 *
 *   @b Description
 *   @n This function returns the ALE entry type for any given ALE table
 *      entry index.
 *
 * =============================================================================
 */
CSL_CPSW_ALE_ENTRYTYPE CSL_CPSW_getALEEntryType(CSL_AleRegs *hCpswAleRegs,
                                                   Uint32      index,
                                      CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                      aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;

    return (CSL_CPSW_ALE_ENTRYTYPE) CSL_ALE_TABLE_GET_ALL_ENTRYTYPE();
}

/** ============================================================================
 *   @n@b CSL_CPSW_getALEAddressType
 *
 *   @b Description
 *   @n This function returns the address type of an ALE entry.
 *
 * =============================================================================
 */
CSL_CPSW_ALE_ADDRTYPE CSL_CPSW_getALEAddressType(CSL_AleRegs *hCpswAleRegs,
                                                    Uint32      index,
                                      CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                      aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;

    if (CSL_ALE_TABLE_GET_OUI_ADDR_ZEROS() == 0)
    {
        /* Lower 24 bits of Word 0 are all zeros for an OUI address */
        return  CSL_ALE_ADDRTYPE_OUI;
    }

    if (CSL_ALE_TABLE_GET_MCASTADDR_MACADDR_0() & 0x1)
    {
        /* 40th bit of MAC address is 1 for a multicast address. */
        return  CSL_ALE_ADDRTYPE_MCAST;
    }
    else
    {
        return  CSL_ALE_ADDRTYPE_UCAST;
    }
}

/** ============================================================================
 *   @n@b CSL_CPSW_getALEPolicerEntryType
 *
 *   @b Description
 *   @n This function returns the entry type of an ALE Policer entry.
 *
 * =============================================================================
 */
CSL_CPSW_ALE_POLICER_ENTRYTYPE CSL_CPSW_getALEPolicerEntryType(CSL_AleRegs *hCpswAleRegs,
                                                                  Uint32      index,
                                      CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                      aleTblCtrlVal   =   0;
    Uint32                      entryType;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;

    entryType = CSL_ALE_TABLE_GET_POLICER_POLICERTYPE();

    if(entryType & 1)
    {
        return(CSL_ALE_POLICER_ENTRYTYPE_IPV6);
    }
    else
    {
        return((CSL_CPSW_ALE_POLICER_ENTRYTYPE)(entryType >> 1));
    }
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAleMcastAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Multicast address configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleMcastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                      Uint32      index,
                                      CSL_CPSW_ALE_MCASTADDR_ENTRY* pMcastAddrCfg,
                                      CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the multicast address configuration */
    pMcastAddrCfg->macAddress[0]        =    CSL_ALE_TABLE_GET_MCASTADDR_MACADDR_0();
    pMcastAddrCfg->macAddress[1]        =    CSL_ALE_TABLE_GET_MCASTADDR_MACADDR_1();
    pMcastAddrCfg->macAddress[2]        =    CSL_ALE_TABLE_GET_MCASTADDR_MACADDR_2();
    pMcastAddrCfg->macAddress[3]        =    CSL_ALE_TABLE_GET_MCASTADDR_MACADDR_3();
    pMcastAddrCfg->macAddress[4]        =    CSL_ALE_TABLE_GET_MCASTADDR_MACADDR_4();
    pMcastAddrCfg->macAddress[5]        =    CSL_ALE_TABLE_GET_MCASTADDR_MACADDR_5();
    pMcastAddrCfg->mcastFwdState        =    CSL_ALE_TABLE_GET_MCASTADDR_FWDSTLVL();
    pMcastAddrCfg->superEnable          =    CSL_ALE_TABLE_GET_MCASTADDR_SUPER();
    pMcastAddrCfg->portMask             =    CSL_ALE_TABLE_GET_MCASTADDR_PORTMASK();
    pMcastAddrCfg->ignMBits             =    CSL_ALE_TABLE_GET_MCASTADDR_IGNMBITS();

    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_setAleMcastAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      Multicast address configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleMcastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                      Uint32      index,
                                      CSL_CPSW_ALE_MCASTADDR_ENTRY* pMcastAddrCfg,
                                      CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    CSL_CPSW_initAleTblWds(hCpswAleRegs);
    /* Setup the multicast address configuration */
    CSL_ALE_TABLE_SET_MCASTADDR_MACADDR_5(pMcastAddrCfg->macAddress[5]);
    CSL_ALE_TABLE_SET_MCASTADDR_MACADDR_4(pMcastAddrCfg->macAddress[4]);
    CSL_ALE_TABLE_SET_MCASTADDR_MACADDR_3(pMcastAddrCfg->macAddress[3]);
    CSL_ALE_TABLE_SET_MCASTADDR_MACADDR_2(pMcastAddrCfg->macAddress[2]);
    CSL_ALE_TABLE_SET_MCASTADDR_MACADDR_1(pMcastAddrCfg->macAddress[1]);
    CSL_ALE_TABLE_SET_MCASTADDR_MACADDR_0(pMcastAddrCfg->macAddress[0]);
    CSL_ALE_TABLE_SET_MCASTADDR_FWDSTLVL(pMcastAddrCfg->mcastFwdState);
    CSL_ALE_TABLE_SET_MCASTADDR_IGNMBITS(pMcastAddrCfg->ignMBits);
    CSL_ALE_TABLE_SET_MCASTADDR_SUPER(pMcastAddrCfg->superEnable);
    CSL_ALE_TABLE_SET_MCASTADDR_PORTMASK(pMcastAddrCfg->portMask);
    /* set entry type to address entry */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_ADDRESS);

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
 *   @n@b CSL_CPSW_getAleVlanMcastAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with VLAN Multicast address configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleVlanMcastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                          Uint32      index,
                                          CSL_CPSW_ALE_VLANMCASTADDR_ENTRY*   pVlanMcastAddrCfg,
                                          CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the multicast address configuration */
    pVlanMcastAddrCfg->macAddress[0]        =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_MACADDR_0();
    pVlanMcastAddrCfg->macAddress[1]        =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_MACADDR_1();
    pVlanMcastAddrCfg->macAddress[2]        =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_MACADDR_2();
    pVlanMcastAddrCfg->macAddress[3]        =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_MACADDR_3();
    pVlanMcastAddrCfg->macAddress[4]        =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_MACADDR_4();
    pVlanMcastAddrCfg->macAddress[5]        =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_MACADDR_5();
    pVlanMcastAddrCfg->vlanId               =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_VLANID();
    pVlanMcastAddrCfg->mcastFwdState        =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_FWDSTLVL();
    pVlanMcastAddrCfg->superEnable          =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_SUPER();
    pVlanMcastAddrCfg->portMask             =   CSL_ALE_TABLE_GET_MCASTADDRVLAN_PORTMASK();
    pVlanMcastAddrCfg->ignMBits             =    CSL_ALE_TABLE_GET_MCASTADDRVLAN_IGNMBITS();


    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanMcastAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      VLAN Multicast address configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleVlanMcastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                         Uint32                              index,
                         CSL_CPSW_ALE_VLANMCASTADDR_ENTRY*   pVlanMcastAddrCfg,
                         CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    CSL_CPSW_initAleTblWds(hCpswAleRegs);
    /* Setup the VLAN multicast address configuration */
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_MACADDR_5(pVlanMcastAddrCfg->macAddress[5]);
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_MACADDR_4(pVlanMcastAddrCfg->macAddress[4]);
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_MACADDR_3(pVlanMcastAddrCfg->macAddress[3]);
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_MACADDR_2(pVlanMcastAddrCfg->macAddress[2]);
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_MACADDR_1(pVlanMcastAddrCfg->macAddress[1]);
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_MACADDR_0(pVlanMcastAddrCfg->macAddress[0]);
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_VLANID(pVlanMcastAddrCfg->vlanId)          ;
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_FWDSTLVL(pVlanMcastAddrCfg->mcastFwdState) ;
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_IGNMBITS(pVlanMcastAddrCfg->ignMBits)      ;
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_SUPER(pVlanMcastAddrCfg->superEnable)      ;
    CSL_ALE_TABLE_SET_MCASTADDRVLAN_PORTMASK(pVlanMcastAddrCfg->portMask)      ;

    /* set entry type to VLAN address entry */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_VLANADDRESS);

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
 *   @n@b CSL_CPSW_getAleUnicastAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Unicast address configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleUnicastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                     Uint32                              index,
                                     CSL_CPSW_ALE_UNICASTADDR_ENTRY*     pUcastAddrCfg,
                                     CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the multicast address configuration */
    pUcastAddrCfg->macAddress[0]        =   CSL_ALE_TABLE_GET_UNIADDR_MACADDR_0();
    pUcastAddrCfg->macAddress[1]        =   CSL_ALE_TABLE_GET_UNIADDR_MACADDR_1();
    pUcastAddrCfg->macAddress[2]        =   CSL_ALE_TABLE_GET_UNIADDR_MACADDR_2();
    pUcastAddrCfg->macAddress[3]        =   CSL_ALE_TABLE_GET_UNIADDR_MACADDR_3();
    pUcastAddrCfg->macAddress[4]        =   CSL_ALE_TABLE_GET_UNIADDR_MACADDR_4();
    pUcastAddrCfg->macAddress[5]        =   CSL_ALE_TABLE_GET_UNIADDR_MACADDR_5();
    pUcastAddrCfg->ageable              =   CSL_ALE_TABLE_GET_UNIADDR_AGABLE();
    pUcastAddrCfg->touched              =   CSL_ALE_TABLE_GET_UNIADDR_TOUCH();
    pUcastAddrCfg->secureEnable         =   CSL_ALE_TABLE_GET_UNIADDR_SECURE();
    pUcastAddrCfg->blockEnable          =   CSL_ALE_TABLE_GET_UNIADDR_BLOCK();
    pUcastAddrCfg->portNumber           =   CSL_ALE_TABLE_GET_UNIADDR_PORTNUM();
    pUcastAddrCfg->trunkFlag            =   CSL_ALE_TABLE_GET_UNIADDR_TRUNK();

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleUnicastAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      unicast address configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleUnicastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                     Uint32      index,
                                     CSL_CPSW_ALE_UNICASTADDR_ENTRY*  pUcastAddrCfg,
                                     CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    CSL_CPSW_initAleTblWds(hCpswAleRegs);
    /* Setup the Unicast address configuration */
    CSL_ALE_TABLE_SET_UNIADDR_MACADDR_5(pUcastAddrCfg->macAddress[5]);
    CSL_ALE_TABLE_SET_UNIADDR_MACADDR_4(pUcastAddrCfg->macAddress[4]);
    CSL_ALE_TABLE_SET_UNIADDR_MACADDR_3(pUcastAddrCfg->macAddress[3]);
    CSL_ALE_TABLE_SET_UNIADDR_MACADDR_2(pUcastAddrCfg->macAddress[2]);
    CSL_ALE_TABLE_SET_UNIADDR_MACADDR_1(pUcastAddrCfg->macAddress[1]);
    CSL_ALE_TABLE_SET_UNIADDR_MACADDR_0(pUcastAddrCfg->macAddress[0]);
    CSL_ALE_TABLE_SET_UNIADDR_AGABLE(pUcastAddrCfg->ageable);
    CSL_ALE_TABLE_SET_UNIADDR_SECURE(pUcastAddrCfg->secureEnable);
    CSL_ALE_TABLE_SET_UNIADDR_BLOCK(pUcastAddrCfg->blockEnable);
    CSL_ALE_TABLE_SET_UNIADDR_PORTNUM(pUcastAddrCfg->portNumber);
    CSL_ALE_TABLE_SET_UNIADDR_TRUNK(pUcastAddrCfg->trunkFlag);
    /* set entry type to address entry */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_ADDRESS);

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
 *   @n@b CSL_CPSW_getAleOUIAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with OUI address configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleOUIAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                    Uint32      index,
                                    CSL_CPSW_ALE_OUIADDR_ENTRY* pOUIAddrCfg,
                                    CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the multicast address configuration */
    pOUIAddrCfg->ouiAddress[0]        =   CSL_ALE_TABLE_GET_OUI_OUIADDR_0();
    pOUIAddrCfg->ouiAddress[1]        =   CSL_ALE_TABLE_GET_OUI_OUIADDR_1();
    pOUIAddrCfg->ouiAddress[2]        =   CSL_ALE_TABLE_GET_OUI_OUIADDR_2();

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleOUIAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      OUI address configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleOUIAddrEntry(CSL_AleRegs *hCpswAleRegs,
                                    Uint32      index,
                                    CSL_CPSW_ALE_OUIADDR_ENTRY*   pOUIAddrCfg,
                                    CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    CSL_CPSW_initAleTblWds(hCpswAleRegs);

    /* Setup the Unicast address configuration */
    CSL_ALE_TABLE_SET_OUI_OUIADDR_2(pOUIAddrCfg->ouiAddress[2]);
    CSL_ALE_TABLE_SET_OUI_OUIADDR_1(pOUIAddrCfg->ouiAddress[1]);
    CSL_ALE_TABLE_SET_OUI_OUIADDR_0(pOUIAddrCfg->ouiAddress[0]);
    CSL_ALE_TABLE_SET_OUI_OUI_ENTRY_BIT_62_63();

    /* set entry type to address entry */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_ADDRESS);

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
 *   @n@b CSL_CPSW_getAleVlanUnicastAddrEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with VLAN Unicast address configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleVlanUnicastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                         Uint32      index,
                         CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY* pVlanUcastAddrCfg,
                         CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the multicast address configuration */
    pVlanUcastAddrCfg->macAddress[0]        =   CSL_ALE_TABLE_GET_UNIADDRVLAN_MACADDR_0();
    pVlanUcastAddrCfg->macAddress[1]        =   CSL_ALE_TABLE_GET_UNIADDRVLAN_MACADDR_1();
    pVlanUcastAddrCfg->macAddress[2]        =   CSL_ALE_TABLE_GET_UNIADDRVLAN_MACADDR_2();
    pVlanUcastAddrCfg->macAddress[3]        =   CSL_ALE_TABLE_GET_UNIADDRVLAN_MACADDR_3();
    pVlanUcastAddrCfg->macAddress[4]        =   CSL_ALE_TABLE_GET_UNIADDRVLAN_MACADDR_4();
    pVlanUcastAddrCfg->macAddress[5]        =   CSL_ALE_TABLE_GET_UNIADDRVLAN_MACADDR_5();
    pVlanUcastAddrCfg->vlanId               =   CSL_ALE_TABLE_GET_UNIADDRVLAN_VLANID()   ;
    pVlanUcastAddrCfg->ageable              =   CSL_ALE_TABLE_GET_UNIADDRVLAN_AGABLE()   ;
    pVlanUcastAddrCfg->touched              =   CSL_ALE_TABLE_GET_UNIADDRVLAN_TOUCH()    ;
    pVlanUcastAddrCfg->secureEnable         =   CSL_ALE_TABLE_GET_UNIADDRVLAN_SECURE()   ;
    pVlanUcastAddrCfg->blockEnable          =   CSL_ALE_TABLE_GET_UNIADDRVLAN_BLOCK()    ;
    pVlanUcastAddrCfg->portNumber           =   CSL_ALE_TABLE_GET_UNIADDRVLAN_PORTNUM()  ;
    pVlanUcastAddrCfg->trunkFlag            =   CSL_ALE_TABLE_GET_UNIADDRVLAN_TRUNK();

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanUnicastAddrEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      VLAN unicast address configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleVlanUnicastAddrEntry(CSL_AleRegs *hCpswAleRegs,
                         Uint32                         index,
                         CSL_CPSW_ALE_VLANUNICASTADDR_ENTRY* pVlanUcastAddrCfg,
                         CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    CSL_CPSW_initAleTblWds(hCpswAleRegs);

    /* Setup the VLAN Unicast address configuration */
    CSL_ALE_TABLE_SET_UNIADDRVLAN_MACADDR_5(pVlanUcastAddrCfg->macAddress[5]);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_MACADDR_4(pVlanUcastAddrCfg->macAddress[4]);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_MACADDR_3(pVlanUcastAddrCfg->macAddress[3]);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_MACADDR_2(pVlanUcastAddrCfg->macAddress[2]);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_MACADDR_1(pVlanUcastAddrCfg->macAddress[1]);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_MACADDR_0(pVlanUcastAddrCfg->macAddress[0]);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_VLANID(pVlanUcastAddrCfg->vlanId);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_AGABLE(pVlanUcastAddrCfg->ageable);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_SECURE(pVlanUcastAddrCfg->secureEnable);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_BLOCK(pVlanUcastAddrCfg->blockEnable);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_PORTNUM(pVlanUcastAddrCfg->portNumber);
    CSL_ALE_TABLE_SET_UNIADDRVLAN_TRUNK(pVlanUcastAddrCfg->trunkFlag);
    /* set entry type to address entry */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_VLANADDRESS);

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
 *   @n@b CSL_CPSW_getAleVlanEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with VLAN configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleVlanEntry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32                              index,
                                 CSL_CPSW_ALE_VLAN_ENTRY*            pVlanCfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the multicast address configuration */
    pVlanCfg->vlanMemList                   =   CSL_ALE_TABLE_GET_INNERVLAN_MEMBER();
    pVlanCfg->unRegMcastFloodMask          =   CSL_ALE_TABLE_GET_INNERVLAN_UNREGMSK();
    pVlanCfg->regMcastFloodMask            =   CSL_ALE_TABLE_GET_INNERVLAN_REGMSK();

    pVlanCfg->forceUntaggedEgress           =   CSL_ALE_TABLE_GET_INNERVLAN_FWDUTAG_PORT_0_7();
    pVlanCfg->forceUntaggedEgress           |= (CSL_ALE_TABLE_GET_INNERVLAN_FWDUTAG_PORT_8()  << 8);
    pVlanCfg->limitIPNxtHdr                 = CSL_ALE_TABLE_GET_INNERVLAN_LMTNXTHDR();
    pVlanCfg->disallowIPFragmentation       = CSL_ALE_TABLE_GET_INNERVLAN_NOFRAG();

    pVlanCfg->vlanId                        =   CSL_ALE_TABLE_GET_INNERVLAN_IVLANID();
    pVlanCfg->ingressCheckFlag              =   CSL_ALE_TABLE_GET_INNERVLAN_INGRESSCHK();
    pVlanCfg->noLearnMask                   =   CSL_ALE_TABLE_GET_INNERVLAN_NOLRNMASK();

    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_setAleVlanEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      VLAN configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleVlanEntry(CSL_AleRegs                       *hCpswAleRegs,
                                 Uint32                            index,
                                 CSL_CPSW_ALE_VLAN_ENTRY*          pVlanCfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    CSL_CPSW_initAleTblWds(hCpswAleRegs);

    /* Setup the (Inner) VLAN configuration */
    CSL_ALE_TABLE_SET_INNERVLAN_MEMBER(pVlanCfg->vlanMemList);
    CSL_ALE_TABLE_SET_INNERVLAN_UNREGMSK(pVlanCfg->unRegMcastFloodMask);
    CSL_ALE_TABLE_SET_INNERVLAN_REGMSK(pVlanCfg->regMcastFloodMask);

    CSL_ALE_TABLE_SET_INNERVLAN_FWDUTAG_PORT_0_7(pVlanCfg->forceUntaggedEgress & 0xFF);
    CSL_ALE_TABLE_SET_INNERVLAN_FWDUTAG_PORT_8((pVlanCfg->forceUntaggedEgress >> 8)  & 0x1);
    CSL_ALE_TABLE_SET_INNERVLAN_LMTNXTHDR(pVlanCfg->limitIPNxtHdr);
    CSL_ALE_TABLE_SET_INNERVLAN_NOFRAG(pVlanCfg->disallowIPFragmentation);

    CSL_ALE_TABLE_SET_INNERVLAN_IVLANID(pVlanCfg->vlanId);
    CSL_ALE_TABLE_SET_INNERVLAN_INGRESSCHK(pVlanCfg->ingressCheckFlag);
    CSL_ALE_TABLE_SET_INNERVLAN_NOLRNMASK(pVlanCfg->noLearnMask);
    /* Set the index in the ALE table to "write" operation and let
     * the hardware set up the ALE table entry corresponding to index
     * specified.
     */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_VLAN);
    CSL_CPSW_setAlePolicerType(hCpswAleRegs,CSL_ALE_POLICER_ENTRYTYPE_VLAN);
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, (Uint32) 1);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getAleOutVlanEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Outer VLAN configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleOutVlanEntry(CSL_AleRegs *hCpswAleRegs,
                                    Uint32      index,
                                    CSL_CPSW_ALE_OUTER_VLAN_ENTRY *     pOutVlanCfg,
                                    CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the outer valn configuration */
    pOutVlanCfg->vlanId                =   CSL_ALE_TABLE_GET_OUTERVLAN_OVLANID();
    pOutVlanCfg->vlanMemList           =   CSL_ALE_TABLE_GET_OUTERVLAN_MEMBER();
    pOutVlanCfg->unRegMcastFloodMask  =   CSL_ALE_TABLE_GET_OUTERVLAN_UNREGMSK();
    pOutVlanCfg->regMcastFloodMask    =   CSL_ALE_TABLE_GET_OUTERVLAN_REGMSK();
    pOutVlanCfg->forceUntaggedEgress   =   (CSL_ALE_TABLE_GET_OUTERVLAN_FWDUTAG_PORT_8() << 8) | CSL_ALE_TABLE_GET_OUTERVLAN_FWDUTAG_PORT_0_7();
    pOutVlanCfg->limitIPNxtHdr         = CSL_ALE_TABLE_GET_OUTERVLAN_LMTNXTHDR();
    pOutVlanCfg->disallowIPFragmentation = CSL_ALE_TABLE_GET_OUTERVLAN_NOFRAG();
    pOutVlanCfg->ingressCheckFlag      =   CSL_ALE_TABLE_GET_OUTERVLAN_INGRESSCHK();
    pOutVlanCfg->noLearnMask           =   CSL_ALE_TABLE_GET_OUTERVLAN_NOLRNMASK();

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleOutVlanEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      Outer VLAN configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleOutVlanEntry(CSL_AleRegs *hCpswAleRegs,
                                    Uint32       index,
                                    CSL_CPSW_ALE_OUTER_VLAN_ENTRY*  pOutVlanCfg,
                                    CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Setup the Outer VLAN configuration */
    CSL_CPSW_initAleTblWds(hCpswAleRegs);

    CSL_ALE_TABLE_SET_OUTERVLAN_MEMBER(pOutVlanCfg->vlanMemList);
    CSL_ALE_TABLE_SET_OUTERVLAN_UNREGMSK(pOutVlanCfg->unRegMcastFloodMask);
    CSL_ALE_TABLE_SET_OUTERVLAN_REGMSK(pOutVlanCfg->regMcastFloodMask);
    CSL_ALE_TABLE_SET_OUTERVLAN_FWDUTAG_PORT_0_7(pOutVlanCfg->forceUntaggedEgress & 0xFF);
    CSL_ALE_TABLE_SET_OUTERVLAN_FWDUTAG_PORT_8((pOutVlanCfg->forceUntaggedEgress >> 8) & 0x1);
    CSL_ALE_TABLE_SET_OUTERVLAN_LMTNXTHDR(pOutVlanCfg->limitIPNxtHdr);
    CSL_ALE_TABLE_SET_OUTERVLAN_NOFRAG(pOutVlanCfg->disallowIPFragmentation);
    CSL_ALE_TABLE_SET_OUTERVLAN_OVLANID(pOutVlanCfg->vlanId);
    CSL_ALE_TABLE_SET_OUTERVLAN_INGRESSCHK(pOutVlanCfg->ingressCheckFlag);
    CSL_ALE_TABLE_SET_OUTERVLAN_NOLRNMASK(pOutVlanCfg->noLearnMask);

    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_VLAN);

    /* set entry type (10) to address entry */
    /* set policer type (010) */
    CSL_CPSW_setAlePolicerType(hCpswAleRegs,CSL_ALE_POLICER_ENTRYTYPE_OVLAN);

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
 *   @n@b CSL_CPSW_getAleEthertypeEntry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Ethertype configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleEthertypeEntry(CSL_AleRegs *hCpswAleRegs,
                                  Uint32      index,
                                  CSL_CPSW_ALE_ETHERTYPE_ENTRY*  pEthertypeCfg,
                                  CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the Ethertype configuration */
    pEthertypeCfg->ethertype        =   CSL_ALE_TABLE_GET_ETHERTYPE_ETHERTYPEVAL();

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleEthertypeEntry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      Ethertype configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleEthertypeEntry(CSL_AleRegs *hCpswAleRegs,
                                  Uint32      index,
                                  CSL_CPSW_ALE_ETHERTYPE_ENTRY*  pEthertypeCfg,
                                  CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Setup the Ethertype configuration */
    CSL_CPSW_initAleTblWds(hCpswAleRegs);

    CSL_ALE_TABLE_SET_ETHERTYPE_ETHERTYPEVAL(pEthertypeCfg->ethertype);

    /* set entry type (10) to address entry */
    /* set policer type (100) */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_POLICER);
    CSL_CPSW_setAlePolicerType(hCpswAleRegs,CSL_ALE_POLICER_ENTRYTYPE_ETHERTYPE);

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
 *   @n@b CSL_CPSW_getAleIPv4Entry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with IPv4 configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleIPv4Entry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32                              index,
                                 CSL_CPSW_ALE_IPv4_ENTRY*            pIPv4Cfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, index) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    /* Read the IPv4 configuration */
    pIPv4Cfg->address[0]          =   CSL_ALE_TABLE_GET_IPV4_IPV4ADDR_0();
    pIPv4Cfg->address[1]          =   CSL_ALE_TABLE_GET_IPV4_IPV4ADDR_1();
    pIPv4Cfg->address[2]          =   CSL_ALE_TABLE_GET_IPV4_IPV4ADDR_2();
    pIPv4Cfg->address[3]          =   CSL_ALE_TABLE_GET_IPV4_IPV4ADDR_3();
    pIPv4Cfg->numLSBIgnore        = CSL_ALE_TABLE_GET_IPV4_IGNBITS();

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleIPv4Entry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      IPv4 configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleIPv4Entry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32                              index,
                                 CSL_CPSW_ALE_IPv4_ENTRY*            pIPv4Cfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32                                  aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    Int32                                   remainIgnBits = pIPv4Cfg->numLSBIgnore;
    Int32                                  i;

    CSL_CPSW_initAleTblWds(hCpswAleRegs);

    for (i = 3; (i >= 0) && (remainIgnBits > 0) ; i-- )
    {
        if (remainIgnBits >= 8)
        {
            pIPv4Cfg->address[i] = 0;
        }
        else
        {
            pIPv4Cfg->address[i] &= ~((1 << remainIgnBits) - 1);
        }
        remainIgnBits -= 8;
    }
    /* Setup the IPv4 configuration */
    CSL_ALE_TABLE_SET_IPV4_IPV4ADDR_3(pIPv4Cfg->address[3]);
    CSL_ALE_TABLE_SET_IPV4_IPV4ADDR_2(pIPv4Cfg->address[2]);
    CSL_ALE_TABLE_SET_IPV4_IPV4ADDR_1(pIPv4Cfg->address[1]);
    CSL_ALE_TABLE_SET_IPV4_IPV4ADDR_0(pIPv4Cfg->address[0]);
    CSL_ALE_TABLE_SET_IPV4_IGNBITS(pIPv4Cfg->numLSBIgnore);

    /* set entry type (10) to address entry */
    /* set policer type (110) */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_POLICER);
    CSL_CPSW_setAlePolicerType(hCpswAleRegs,CSL_ALE_POLICER_ENTRYTYPE_IPV4);

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
 *   @n@b CSL_CPSW_getAleIPv6Entry
 *
 *   @b Description
 *   @n This function reads the ALE table entry for the index specified and
 *      fills the output parameter structure with Ipv6 configuration
 *      read from the hardware.
 *
 * =============================================================================
 */
void CSL_CPSW_getAleIPv6Entry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32                              index,
                                 CSL_CPSW_ALE_IPv6_ENTRY*            pIPv6Cfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32      aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX,
                                         (CSL_CPSW_getAleIPv6HighEntryIndex(hCpswAleRegs,index))) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    pIPv6Cfg->address[0] = CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_0();
    pIPv6Cfg->address[1] = CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_1();
    pIPv6Cfg->address[2] = CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_2();
    pIPv6Cfg->address[3] = (CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_3_HIGH_NIBBLE() << 0x4) | (CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_3_LOW_NIBBLE());
    pIPv6Cfg->address[4] = CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_4();
    pIPv6Cfg->address[5] = CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_5();
    pIPv6Cfg->address[6] = CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_6();
    pIPv6Cfg->address[7] = CSL_ALE_TABLE_GET_IPV6_HIGH_IPV6ADDR_7_HIGH_NIBBLE() << 0x4;

    /* Read the IPv6 configuration */

    pIPv6Cfg->numLSBIgnore = CSL_ALE_TABLE_GET_IPV6_HIGH_IGNBITS();

    /* Set the index in the ALE table to "read" operation and let
     * the hardware load the ALE table entry corresponding to index
     * specified into the ALE table word registers.
     */
    aleTblCtrlVal  =  CSL_FMK (ALE_POLICETBLCTL_POL_TBL_IDX, CSL_CPSW_getAleIPv6LowEntryIndex(hCpswAleRegs,index)) |
                      CSL_FMK (ALE_POLICETBLCTL_WRITE_ENABLE, 0);

    hCpswAleRegs->ALE_TBLCTL =   aleTblCtrlVal;

    pIPv6Cfg->address[7] |= CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_7_LOW_NIBBLE();
    pIPv6Cfg->address[8] = ((CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_8_HIGH_NIBBLE_BIT0TO2()  | (CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_8_HIGH_NIBBLE_BIT3() << 3)) << 4)
                            |
                            (CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_8_LOW_NIBBLE());
    pIPv6Cfg->address[9] = CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_9();
    pIPv6Cfg->address[10] = CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_10();
    pIPv6Cfg->address[11] = CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_11();
    pIPv6Cfg->address[12] = CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_12();
    pIPv6Cfg->address[13] = CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_13();
    pIPv6Cfg->address[14] = CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_14();
    pIPv6Cfg->address[15] = CSL_ALE_TABLE_GET_IPV6_LOW_IPV6ADDR_15();

    return;
}

/** ============================================================================
 *   @n@b CSL_CPSW_setAleIPv6Entry
 *
 *   @b Description
 *   @n This function sets up the ALE table entry for the index specified with
 *      IPv6 configuration specified here.
 *
 * =============================================================================
 */
void CSL_CPSW_setAleIPv6Entry(CSL_AleRegs *hCpswAleRegs,
                                 Uint32       index,
                                 CSL_CPSW_ALE_IPv6_ENTRY*   pIPv6Cfg,
                                 CSL_CPSW_ALETABLE_TYPE aleType)
{
    Uint32          aleTblCtrlVal   =   0;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    Int32                                   remainIgnBits = pIPv6Cfg->numLSBIgnore;
    Int32                                  i;

    for (i = 15; (i >= 0) && (remainIgnBits > 0) ; i-- )
    {
        if (remainIgnBits >= 8)
        {
            pIPv6Cfg->address[i] = 0;
        }
        else
        {
            pIPv6Cfg->address[i] &= ~((1 << remainIgnBits) - 1);
        }
        remainIgnBits -= 8;
    }

    CSL_CPSW_initAleTblWds(hCpswAleRegs);

    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_7_HIGH_NIBBLE((pIPv6Cfg->address[7] >> 4) & 0xF);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_6(pIPv6Cfg->address[6]);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_5(pIPv6Cfg->address[5]);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_4(pIPv6Cfg->address[4]);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_3_LOW_NIBBLE(pIPv6Cfg->address[3] & 0xF);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_3_HIGH_NIBBLE((pIPv6Cfg->address[3] >> 4) & 0xF);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_2(pIPv6Cfg->address[2]);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_1(pIPv6Cfg->address[1]);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IPV6ADDR_0(pIPv6Cfg->address[0]);
    CSL_ALE_TABLE_SET_IPV6_HIGH_IGNBITS(pIPv6Cfg->numLSBIgnore);

    /* set entry type (10) to address entry */
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_POLICER);
    /* set policer type (xx1) */
    CSL_ALE_TABLE_SET_IPV6_HIGH_POLICERTYPE();

    /* Set the index in the ALE table to "write" operation and let
     * the hardware set up the ALE table entry corresponding to index
     * specified.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, (CSL_CPSW_getAleIPv6HighEntryIndex(hCpswAleRegs,index))) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, (Uint32) 1);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;

    CSL_CPSW_initAleTblWds(hCpswAleRegs);
    /* Update the second record */
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_15(pIPv6Cfg->address[15]);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_14(pIPv6Cfg->address[14]);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_13(pIPv6Cfg->address[13]);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_12(pIPv6Cfg->address[12]);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_11(pIPv6Cfg->address[11]);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_10(pIPv6Cfg->address[10]);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_9(pIPv6Cfg->address[9]);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_8_LOW_NIBBLE(pIPv6Cfg->address[8] & 0xF);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_8_HIGH_NIBBLE_BIT3(((pIPv6Cfg->address[8] >> 4) & 0x8) >> 3);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_8_HIGH_NIBBLE_BIT0TO2((pIPv6Cfg->address[8] >> 4) & 0x7);
    CSL_ALE_TABLE_SET_IPV6_LOW_IPV6ADDR_7_LOW_NIBBLE(pIPv6Cfg->address[7] & 0xF);
    CSL_ALE_TABLE_SET_ALL_ENTRYTYPE(CSL_ALE_ENTRYTYPE_POLICER);
    CSL_ALE_TABLE_SET_IPV6_LOW_POLICERTYPE();


    /* Set the index in the ALE table to "write" operation and let
     * the hardware set up the ALE table entry corresponding to index
     * specified.
     */
    aleTblCtrlVal           =   CSL_FMK (ALE_ALE_TBLCTL_TABLEIDX, CSL_CPSW_getAleIPv6LowEntryIndex(hCpswAleRegs,index)) |
                                CSL_FMK (ALE_ALE_TBLCTL_TABLEWR, (Uint32) 1);

    hCpswAleRegs->ALE_TBLCTL    =   aleTblCtrlVal;


    return;
}


/** ============================================================================
 *   @n@b CSL_CPSW_mapTableWord2MacAddr
 *
 *   @b Description
 *   @n This function extracts the mac address from the ALE table word0 and
 *      word 1
 *
 * =============================================================================
 */
void CSL_CPSW_mapTableWord2MacAddr(Uint32 word0,
                                      Uint32 word1,
                                      Uint8 * macAddr,
                                      CSL_CPSW_ALETABLE_TYPE aleType)
{
    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);
    macAddr[0] = CSL_CPSW_ALE_MACADDR_GET0(word1);
    macAddr[1] = CSL_CPSW_ALE_MACADDR_GET1(word1);
    macAddr[2] = CSL_CPSW_ALE_MACADDR_GET2(word0);
    macAddr[3] = CSL_CPSW_ALE_MACADDR_GET3(word0);
    macAddr[4] = CSL_CPSW_ALE_MACADDR_GET4(word0);
    macAddr[5] = CSL_CPSW_ALE_MACADDR_GET5(word0);
}


/** ============================================================================
 *   @n@b CSL_CPSW_mapMacAddr2TableWord_9G
 *
 *   @b Description
 *   @n This function populates the ALE table word 0 and word 1 with the
 *      mac address passed
 * =============================================================================
 */
void CSL_CPSW_mapMacAddr2TableWord(Uint32             *word0,
                                      Uint32             *word1,
                                      Uint8 *            macAddr,
                                      CSL_CPSW_ALETABLE_TYPE aleType)
{
    *word0 = 0U;
    *word1 = 0U;

    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);

    CSL_FINS(*word1, ALE_TABLE_WORD1_REG_ENTRY_TYPE_UNIADDR_MACADDR_0,
             macAddr[0]);
    CSL_FINS(*word1, ALE_TABLE_WORD1_REG_ENTRY_TYPE_UNIADDR_MACADDR_1,
             macAddr[1]);

    CSL_FINS(*word0, ALE_TABLE_WORD0_REG_ENTRY_TYPE_UNIADDR_MACADDR_2,
             macAddr[2]);
    CSL_FINS(*word0, ALE_TABLE_WORD0_REG_ENTRY_TYPE_UNIADDR_MACADDR_3,
             macAddr[3]);
    CSL_FINS(*word0, ALE_TABLE_WORD0_REG_ENTRY_TYPE_UNIADDR_MACADDR_4,
             macAddr[4]);
    CSL_FINS(*word0, ALE_TABLE_WORD0_REG_ENTRY_TYPE_UNIADDR_MACADDR_5,
             macAddr[5]);
}

/** ============================================================================
 *   @n@b CSL_CPSW_extractVid_9G
 *
 *   @b Description
 *   @n This function extracts the vlan id field from the ALE table word 1
 * =============================================================================
 */
Uint32 CSL_CPSW_extractVid(Uint32             word1,
                                      CSL_CPSW_ALETABLE_TYPE aleType)
{
    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);
    Uint32 vid = CSL_FEXT(word1,
                          ALE_TABLE_WORD1_REG_ENTRY_TYPE_ALLADDRVLAN_VLANID);

    return vid;
}


/** ============================================================================
 *   @n@b CSL_CPSW_getEthertypeMax_9G
 *
 *   @b Description
 *   @n This function returns the max value of EtherType field in ALE table
 * =============================================================================
 */
Uint32 CSL_CPSW_getEthertypeMax(CSL_CPSW_ALETABLE_TYPE aleType)
{
    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);
    return CSL_ALE_TABLE_WORD0_REG_ENTRY_TYPE_ETHERTYPE_ETHERTYPEVAL_MAX;
}


/** ============================================================================
 *   @n@b CSL_CPSW_getIpv4IgnBitsMax_9G
 *
 *   @b Description
 *   @n This function returns the max value of IPv4 ignore bits field in ALE table
 * =============================================================================
 */
Uint32 CSL_CPSW_getIpv4IgnBitsMax(CSL_CPSW_ALETABLE_TYPE aleType)
{
    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);
    return CSL_ALE_TABLE_WORD2_REG_ENTRY_TYPE_IPV4_IGNBITS_MAX;
}

/** ============================================================================
 *   @n@b CSL_CPSW_getIpv6IgnBitsMax_9G
 *
 *   @b Description
 *   @n This function returns the max value of IPv6 ignore bits field in ALE table
 * =============================================================================
 */
Uint32 CSL_CPSW_getIpv6IgnBitsMax(CSL_CPSW_ALETABLE_TYPE aleType)
{
    DebugP_assert(aleType == CSL_CPSW_ALETABLE_TYPE_9PORT);
    return CSL_ALE_TABLE_WORD2_REG_ENTRY_TYPE_IPV6_HIGH_IGNBITS_MAX;
}

/**
@}
*/
