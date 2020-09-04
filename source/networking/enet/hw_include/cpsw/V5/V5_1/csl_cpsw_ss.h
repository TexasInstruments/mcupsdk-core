/**
 * @file  csl_cpsw_ss.h
 *
 * @brief
 *  API Auxilary header file for Ethernet switch subsystem CSL.
 *
 *  Contains the different control command and status query functions definations
 *
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

#ifndef CSL_CPSW_SS_V5_1_H_
#define CSL_CPSW_SS_V5_1_H_

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

/** @brief
 *
 *  Holds the CPSW_SS Configuration Info
 */
typedef struct {
    /** Number of CPSW ports including the host port */
    Uint32      numPorts;
    /** Number of CPTS GENFs */
    Uint32      numGenfs;
    /** RMII interface: 0: disable; 1:enable*/
    Uint32      rmii;
    /** RGMII interface: 0: disable; 1:enable*/
    Uint32      rgmii;
    /** SGMII interface: 0: disable; 1:enable*/
    Uint32      sgmii;
    /** QSGMII interface: 0: disable; 1:enable*/
    Uint32      qsgmii;
    /** XRGMII interface: 0: disable; 1:enable*/
    Uint32      xgmii;
} CSL_CPSW_SS_CONFIG_INFO;

/** @brief
 *
 *  Defines CPSW Miscellaneous Intrrupt Masks
 */
#define CPSW_MISC_INT_MDIO_USERINT_MASK     CSL_WR_MISC_STAT_MDIO_USERINT_MASK
#define CPSW_MISC_INT_MDIO_LINKINT_MASK     CSL_WR_MISC_STAT_MDIO_LINKINT_MASK
#define CPSW_MISC_INT_HOSTERR_MASK          CSL_WR_MISC_STAT_HOST_PEND_MASK
#define CPSW_MISC_INT_STAT_OVERFLOW_MASK    CSL_WR_MISC_STAT_STAT_PEND_MASK
#define CPSW_MISC_INT_CPTS_EVENT_MASK       CSL_WR_MISC_STAT_EVNT_PEND_MASK
#define CPSW_MISC_INT_SEC_MEM_ERR_MASK      CSL_WR_MISC_STAT_SEC_PEND_MASK
#define CPSW_MISC_INT_DED_MEM_ERR_MASK      CSL_WR_MISC_STAT_DED_PEND_MASK


/** @brief
 *
 *  Defines CPSW Intrrupt PACING Bit definitions
 */
#define CPSW_INT_CONTROL_INT_PACE_EN_C0_RX 	(0x00000001U)
#define CPSW_INT_CONTROL_INT_PACE_EN_C0_TX 	(0x00000002U)
#define CPSW_INT_CONTROL_INT_PACE_EN_C1_RX 	(0x00000004U)
#define CPSW_INT_CONTROL_INT_PACE_EN_C1_TX 	(0x00000008U)
#define CPSW_INT_CONTROL_INT_PACE_EN_C2_RX 	(0x00000010U)
#define CPSW_INT_CONTROL_INT_PACE_EN_C2_TX 	(0x00000020U)

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
                                 CSL_CPSW_SS_VERSION* versionInfo);

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
                                CSL_CPSW_SS_RGMIISTATUS *pRgmiiStatus);

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
                               CSL_CPSW_SS_CONFIG_INFO *pConfigInfo);

/**
 * \brief   Enables Rx Thresh interrupt for the specified channel and core.
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 * \return  None
 */
static inline void CSL_CPSW_enableWrRxThresholdInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline void CSL_CPSW_enableWrRxThresholdInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    CSL_FINSR (hCpswSsRegs->CORE_INT_EN[core].RX_THRESH_EN, channel, channel, (uint32_t)1U);
}

/**
 * \brief   Enables Rx interrupt for the specified channel and core.
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 *
 * \return  None
 */
static inline void CSL_CPSW_enableWrRxInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline void CSL_CPSW_enableWrRxInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    CSL_FINSR (hCpswSsRegs->CORE_INT_EN[core].RX_EN, channel, channel, (uint32_t)1U);
}

/**
 * \brief   Enables Tx interrupt for the specified channel and core.
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 * \return  None
 */
static inline void CSL_CPSW_enableWrTxInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline void CSL_CPSW_enableWrTxInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    CSL_FINSR (hCpswSsRegs->CORE_INT_EN[core].TX_EN, channel, channel, (uint32_t)1U);
}

/**
 * \brief   Enables the specified Misc interrupts for the specified core.
 *
 * \param   hCpswSsRegs	 Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core         Core number
 * \param   miscIntrMask Interrupts to be enabled
 *    'intrMask' consists of the following Masks. \n
 *          CPSW_MISC_INT_MDIO_USERINT_MASK 
 *          CPSW_MISC_INT_MDIO_LINKINT_MASK 
 *          CPSW_MISC_INT_HOSTERR_MASK      
 *          CPSW_MISC_INT_STAT_OVERFLOW_MASK
 *          CPSW_MISC_INT_CPTS_EVENT_MASK   
 *          CPSW_MISC_INT_SEC_MEM_ERR_MASK  
 *          CPSW_MISC_INT_DED_MEM_ERR_MASK  
 *
 * \return  None
 */
static inline void CSL_CPSW_enableWrMiscInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    miscIntrMask
);/* for misra warnings*/
static inline void CSL_CPSW_enableWrMiscInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    miscIntrMask
)
{
    hCpswSsRegs->CORE_INT_EN[core].MISC_EN |= miscIntrMask;
}

/**
 * \brief   Disables Rx Thresh interrupt for the specified channel and core.
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 * \return  None
 */
static inline void CSL_CPSW_disableWrRxThresholdInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline void CSL_CPSW_disableWrRxThresholdInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    CSL_FINSR (hCpswSsRegs->CORE_INT_EN[core].RX_THRESH_EN, channel, channel, (uint32_t)0);
}

/**
 * \brief   Disables Rx interrupt for the specified channel and core.
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 * \return  None
 */
static inline void CSL_CPSW_disableWrRxInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline void CSL_CPSW_disableWrRxInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    CSL_FINSR (hCpswSsRegs->CORE_INT_EN[core].RX_EN, channel, channel, (uint32_t)0);
}

/**
 * \brief   Disables Tx interrupt for the specified channel and core.
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 * \return  None
 **/
static inline void CSL_CPSW_disableWrTxInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline void CSL_CPSW_disableWrTxInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    CSL_FINSR (hCpswSsRegs->CORE_INT_EN[core].TX_EN, channel, channel, (uint32_t)0);
}

/**
 * \brief   Disables the specified Misc interrupts for the specified core.
 *
 * \param   hCpswSsRegs  Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core         Core number
 * \param   miscIntrMask Interrupts to be disabled
 *    'intrMask' consists of the following Masks. \n
 *          CPSW_MISC_INT_MDIO_USERINT_MASK 
 *          CPSW_MISC_INT_MDIO_LINKINT_MASK 
 *          CPSW_MISC_INT_HOSTERR_MASK      
 *          CPSW_MISC_INT_STAT_OVERFLOW_MASK
 *          CPSW_MISC_INT_CPTS_EVENT_MASK   
 *          CPSW_MISC_INT_SEC_MEM_ERR_MASK  
 *          CPSW_MISC_INT_DED_MEM_ERR_MASK  
 *
 * \return  None
 **/
static inline void CSL_CPSW_disableWrMiscInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    miscIntrMask
);/* for misra warnings*/
static inline void CSL_CPSW_disableWrMiscInt
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    miscIntrMask
)
{
    hCpswSsRegs->CORE_INT_EN[core].MISC_EN &= ~miscIntrMask;
}

/**
 * \brief   Returns the Rx Thresh interrupt status of the  
 *          specified channel and core
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 *
 * \return  '1' if the status is set
 *          '0' if the status is cleared
 **/
static inline Uint32 CSL_CPSW_getWrRxThresholdIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline Uint32 CSL_CPSW_getWrRxThresholdIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    return (CSL_FEXTR (hCpswSsRegs->CORE_INT_STAT[core].RX_THRESH_STAT, channel, channel));
}

/**
 * \brief   Returns the Rx interrupt status of the specified
 *          channel and core
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 *
 * \return  '1' if the status is set
 *          '0' if the status is cleared
 **/
static inline Uint32 CSL_CPSW_getWrRxIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline Uint32 CSL_CPSW_getWrRxIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    return (CSL_FEXTR (hCpswSsRegs->CORE_INT_STAT[core].RX_STAT, channel, channel));
}

/**
 * \brief   Returns the Tx interrupt status of the specified
 *          channel and core
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   channel     Channel number
 *
 * \return  '1' if the status is set
 *          '0' if the status is cleared
 **/
static inline Uint32 CSL_CPSW_getWrTxIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
);/* for misra warnings*/
static inline Uint32 CSL_CPSW_getWrTxIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    channel
)
{
    return (CSL_FEXTR (hCpswSsRegs->CORE_INT_STAT[core].TX_STAT, channel, channel));
}

/**
 * \brief   Returns the Misc interrupt status of the specified
 *          core
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 *    The following masks could be set to indicate interrupts\n
 *          CPSW_MISC_INT_MDIO_USERINT_MASK 
 *          CPSW_MISC_INT_MDIO_LINKINT_MASK 
 *          CPSW_MISC_INT_HOSTERR_MASK      
 *          CPSW_MISC_INT_STAT_OVERFLOW_MASK
 *          CPSW_MISC_INT_CPTS_EVENT_MASK   
 *          CPSW_MISC_INT_SEC_MEM_ERR_MASK  
 *          CPSW_MISC_INT_DED_MEM_ERR_MASK  
 *
 * \return  intrrupt masks as indicated above
 *          '0' if the status is cleared
 **/
static inline Uint32 CSL_CPSW_getWrMiscIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core
);/* for misra warnings*/
static inline Uint32 CSL_CPSW_getWrMiscIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core
)
{
    return ((Uint32)hCpswSsRegs->CORE_INT_STAT[core].MISC_STAT);
}

/**
 * \brief   Sets the maximum number of Rx interrupts per millisecond
 *          (Interrupt Pacing) of the specified core
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   value       Number of interrupts per millisecond
 *
 * \return  None
 **/
static inline void CSL_CPSW_setWrRxIntPerMSec
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    value
);/* for misra warnings*/
static inline void CSL_CPSW_setWrRxIntPerMSec
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    value
)
{
    CSL_FINS (hCpswSsRegs->CORE_INTS_PER_MS[core].RX_IMAX, WR_RX_IMAX_RX_IMAX, value); 
}

/**
 * \brief   Sets the maximum number of Tx interrupts per millisecond
 *          (Interrupt Pacing) of the specified core
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core        Core number
 * \param   value       Number of interrupts per millisecond
 *
 * \return  None
 **/
static inline void CSL_CPSW_setWrTxIntPerMSec
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    value
);/* for misra warnings*/
static inline void CSL_CPSW_setWrTxIntPerMSec
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    core,
    Uint32                    value
)
{
    CSL_FINS (hCpswSsRegs->CORE_INTS_PER_MS[core].TX_IMAX, WR_TX_IMAX_TX_IMAX, value); 
}

/* TODO: find the interrupt pacing mask: Rx: bit 0, TX: bit 1?
 *       and the register name is strange 
*/

/**
 * \brief   Sets Interrupt pacing control
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   bitmask     The control bitmask to enable/disable interrupt pacing
 *
 * \return  None
 **/
static inline void CSL_CPSW_setWrIntPacingControl
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    bitmask
);/* for misra warnings*/
static inline void CSL_CPSW_setWrIntPacingControl
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    bitmask
)
{
    CSL_FINS (hCpswSsRegs->INT_CONTROL_REG,
              XGE_CPSW_SS_S_INT_CONTROL_REG_INT_BYPASS,
              bitmask); 
}

/**
 * \brief   Set Interrupt counter prescaler
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   value       prescaler
 *
 * \return  None
 **/
static inline void CSL_CPSW_setWrIntPrescaler
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    value
);/* for misra warnings*/
static inline void CSL_CPSW_setWrIntPrescaler
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
    Uint32                    value
)
{
    CSL_FINS (hCpswSsRegs->INT_CONTROL_REG,
              XGE_CPSW_SS_S_INT_CONTROL_REG_INT_PRESCALE,
              value); 
}

/**
 * \brief   Get TX interrupt status
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core       Core number
 *
 * \return  status for TX interrupt
 **/
static inline Uint32 CSL_CPSW_getTxIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
	Uint32                    core
);/* for misra warnings*/
static inline Uint32 CSL_CPSW_getTxIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
	Uint32                    core
)
{
	return ((Uint32)hCpswSsRegs->CORE_INT_STAT[core].TX_STAT);
}

/**
 * \brief   Get RX interrupt status
 *
 * \param   hCpswSsRegs Pointer to CSL_Xge_cpsw_ss_sRegs structure
 * \param   core       Core number
 *
 * \return  status for RX interrupt
 **/
static inline Uint32 CSL_CPSW_getRxIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
	Uint32                    core
);/* for misra warnings*/
static inline Uint32 CSL_CPSW_getRxIntStatus
(
    CSL_Xge_cpsw_ss_sRegs    *hCpswSsRegs,
	Uint32                    core
)
{
	return ((Uint32)hCpswSsRegs->CORE_INT_STAT[core].RX_STAT);
}

#ifdef __cplusplus
}
#endif

#endif

/**
@}
*/
