/**
 * @file  csl_cpgmac_sl.h
 *
 * @brief
 *  API Auxilary header file for Ethernet MAC submodule CSL.
 *
 *  Contains the different control command and status query functions definations
 *
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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

#ifndef CSL_CPGMAC_SL_V5_H_
#define CSL_CPGMAC_SL_V5_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <drivers/hw_include/cslr_soc.h>

#include <cslr_xge_cpsw.h>


/**
@defgroup CSL_CPGMAC_SL_SYMBOL  CPGMAC_SL Symbols Defined
@ingroup CSL_CPGMAC_SL_API
*/
/**
@defgroup CSL_CPGMAC_SL_DATASTRUCT  CPGMAC_SL Data Structures
@ingroup CSL_CPGMAC_SL_API
*/
/**
@defgroup CSL_CPGMAC_SL_FUNCTION  CPGMAC_SL Functions
@ingroup CSL_CPGMAC_SL_API
*/
/**
@defgroup CSL_CPGMAC_SL_ENUM CPGMAC_SL Enumerated Data Types
@ingroup CSL_CPGMAC_SL_API
*/

/**
@addtogroup CSL_CPGMAC_SL_SYMBOL
@{
*/

/**
@}
*/

/** @addtogroup CSL_CPGMAC_SL_DATASTRUCT
 @{ */

/** Constants for passing parameters to the functions.
 */

/** @brief MAC control register configuration definitions */

/** Enable full duplex mode */
#define    CSL_CPGMAC_SL_MACCONTROL_FULLDUPLEX_EN       (1 << 0u)

/** Enable loopback mode */
#define    CSL_CPGMAC_SL_MACCONTROL_LOOPBACK_EN         (1 << 1u)

/** Enable Rx flow control mode */
#define    CSL_CPGMAC_SL_MACCONTROL_RX_FLOW_EN          (1 << 3u)

/** Enable Tx flow control mode */
#define    CSL_CPGMAC_SL_MACCONTROL_TX_FLOW_EN          (1 << 4u)

/** Enable GMII */
#define    CSL_CPGMAC_SL_MACCONTROL_GMII_EN             (1 << 5u)

/** Enable Tx pacing */
#define    CSL_CPGMAC_SL_MACCONTROL_TX_PACE_EN          (1 << 6u)

/** Enable Gigabit mode */
#define    CSL_CPGMAC_SL_MACCONTROL_GIG_EN              (1 << 7u)

/** Enable Tx short gap */
#define    CSL_CPGMAC_SL_MACCONTROL_TX_SHORT_GAP_EN     (1 << 10u)

/** Enable idle mode */
#define    CSL_CPGMAC_SL_MACCONTROL_CMD_IDLE_EN         (1 << 11u)

/** Enable idle mode */
#define    CSL_CPGMAC_SL_MACCONTROL_CASTAGNOLI_CRC      (1 << 12u)

/** Set IFCTL_A bit to 1 */
#define    CSL_CPGMAC_SL_MACCONTROL_IFCTL_A_EN          (1 << 15u)

/** Set IFCTL_B bit to 1 */
#define    CSL_CPGMAC_SL_MACCONTROL_IFCTL_B_EN          (1 << 16u)

/** Enable forced Gigabit mode */
#define    CSL_CPGMAC_SL_MACCONTROL_GIG_FORCE_EN        (1 << 17u)

/** Enable external control mode */
#define    CSL_CPGMAC_SL_MACCONTROL_EXT_EN              (1 << 18u)

/** Enable external receive flow control mode */
#define    CSL_CPGMAC_SL_MACCONTROL_EXT_EN_RX_FLOW      (1 << 19u)

/** Enable external transmit flow control mode */
#define    CSL_CPGMAC_SL_MACCONTROL_EXT_EN_TX_FLOW      (1 << 20u)

/** Enable Rx copy error frames mode */
#define    CSL_CPGMAC_SL_MACCONTROL_RX_CEF_EN           (1 << 22u)

/** Enable Rx copy short frames mode */
#define    CSL_CPGMAC_SL_MACCONTROL_RX_CSF_EN           (1 << 23u)

/** Enable Rx copy MAC control frames mode */
#define    CSL_CPGMAC_SL_MACCONTROL_RX_CMF_EN           (1 << 24u)

/** @brief
 *
 *  Holds the Sliver submodule's version info.
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
} CSL_CPGMAC_SL_VERSION;

/** @brief
 *
 *  Holds MAC status register contents.
 */
typedef struct {
    /** Transmit Flow Control Active - When asserted, this bit indicates that
      * the pause time period is being observed for a received pause frame.
      * No new transmissions will begin while this bit is asserted except for the
      * transmission of pause frames.  Any transmission in progress when this bit
      * is asserted will complete.
      */
    Uint32      txFlowActive;
    /** Receive Flow Control Active - When asserted, indicates that receive flow
      * control is enabled and triggered
      */
    Uint32      rxFlowActive;
    /** External Fullduplex - Value of the EXT_FULLDUPLEX input bit. */
    Uint32      extFullDuplexEnabled;
    /** External GIG - This is the value of the EXT_GIG input bit. */
    Uint32      extGigabitEnabled;
    /** External Receive Flow Control Enable - This is the value of the
      * EXT_RX_FLOW_EN input bit.
      */
    Uint32      extRxFlowEnabled;
    /** External Transmit Flow Control Enable -
      * This is the value of the EXT_TX_FLOW_EN input bit
      */
    Uint32      extTxFlowEnabled;
    /** Receive Priority Based Flow Control Active (priority 7 down to 0) */
    Uint32      rxPfcFlowAct;
    /** Transmit Priority Based Flow Control Active (priority 7 down to 0) */
    Uint32      txPfcFlowAct;
    /** The lowest priority that caused top of receive FIFO flow control trigger
      * since the last write to clear.
      */
    Uint32      torfPri;
    /** Top of receive FIFO flow control trigger occurred.  */
    Uint32      torf;
    /** Mac Transmit Idle - Both Prempt and Express MAC Transmit in idle state. */
    Uint32      macTxIdle;
    /** Express MAC is idle .If IET is not enabled all traffic is express traffic */
    Uint32      expressMacIdle;
    /** Prempt MAC is idle */
    Uint32      preemptMacIdle;
    /** The Ethernet port (express and preempt (if present)) are in idle state */
    Uint32      idle;
} CSL_CPGMAC_SL_MACSTATUS;

/**
@}
*/

/** @addtogroup CSL_CPGMAC_SL_FUNCTION
@{ */


/********************************************************************************
*************** Ethernet Media Access Controller (EMAC) Submodule ***************
********************************************************************************/

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isFullDuplexEnabled
 *
 *   @b Description
 *   @n This function indicates if Full duplex mode is enabled in the
 *      MAC control register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must retrieve the
                                settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Full duplex mode enabled
 *	 @n  FALSE                  Half duplex mode enabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum;

        portNum =   1;
        if (CSL_CPGMAC_SL_isFullDuplexEnabled (portNum) == TRUE)
        {
            // full duplex mode enabled
        }
        else
        {
            // full duplex mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isFullDuplexEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableFullDuplex
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable full duplex
 *      mode for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX=1
 *
 *   @b Example
 *   @verbatim
 *      Uint32      portNum =   1;

        CSL_CPGMAC_SL_enableFullDuplex (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableFullDuplex
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableFullDuplex
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable full duplex
 *      mode for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX=0
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum = 1;

        CSL_CPGMAC_SL_disableFullDuplex (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableFullDuplex
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isLoopbackModeEnabled
 *
 *   @b Description
 *   @n This function indicates if loopback mode is enabled in the
 *      MAC control register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Internal full duplex lopoback mode enabled
 *	 @n  FALSE                  Loopback mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_LOOPBACK
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum = 1;

        if (CSL_CPGMAC_SL_isLoopbackModeEnabled (portNum) == TRUE)
        {
            // loopback mode enabled
        }
        else
        {
            // loopback mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isLoopbackModeEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableLoopback
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable lopback mode
 *      for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_LOOPBACK=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum = 1;

        CSL_CPGMAC_SL_enableLoopback (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableLoopback
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableLoopback
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable loopback
 *      mode for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_LOOPBACK=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_disableLoopback (portNum);
     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableLoopback
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isRxFlowControlEnabled
 *
 *   @b Description
 *   @n This function indicates if receive flow control is enabled in the
 *      MAC control register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Receive flow control enabled
 *	 @n  FALSE                  Receive flow control disabled
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_FLOW_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum = 1;

        if (CSL_CPGMAC_SL_isRxFlowControlEnabled (portNum) == TRUE)
        {
            // rx flow control enabled
        }
        else
        {
            // rx flow control disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isRxFlowControlEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableRxFlowControl
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable receive flow
 *      control for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_FLOW_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_enableRxFlowControl (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableRxFlowControl
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableRxFlowControl
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable Receive
 *      flow control mode for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_FLOW_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_disableRxFlowControl (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableRxFlowControl
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isTxFlowControlEnabled
 *
 *   @b Description
 *   @n This function indicates if transmit flow control is enabled in the
 *      MAC control register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Transmit flow control enabled
 *	 @n  FALSE                  Transmit flow control disabled
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_FLOW_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum =   1;

        if (CSL_CPGMAC_SL_isTxFlowControlEnabled (portNum) == TRUE)
        {
            // tx flow control enabled
        }
        else
        {
            // tx flow control disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isTxFlowControlEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableTxFlowControl
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable transmit
 *      flow control for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_FLOW_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_enableTxFlowControl (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableTxFlowControl
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableTxFlowControl
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable Transmit
 *      flow control for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_FLOW_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_disableTxFlowControl (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableTxFlowControl
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);


/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isGMIIEnabled
 *
 *   @b Description
 *   @n This function indicates if GMII is enabled in the MAC control register
 *      for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   GMII enabled
 *	 @n  FALSE                  GMII disabled
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum =   1;

        if (CSL_CPGMAC_SL_isGMIIEnabled (portNum) == TRUE)
        {
            // gmii enabled
        }
        else
        {
            // gmii disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isGMIIEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableGMII
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable GMII for the
 *      MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_enableGMII (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableGMII
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableGMII
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable GMII for the
 *      MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_disableGMII (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableGMII
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isTxPaceEnabled
 *
 *   @b Description
 *   @n This function indicates if transmit pacing is enabled in the MAC
 *      control register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Transmit pacing enabled
 *	 @n  FALSE                  Transmit pacing disabled
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_PACE
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum =   1;

        if (CSL_CPGMAC_SL_isTxPaceEnabled (portNum) == TRUE)
        {
            // tx pacing enabled
        }
        else
        {
            // tx pacing disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isTxPaceEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableTxPace
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable transmit
 *      pacing for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_PACE=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum = 1;

        CSL_CPGMAC_SL_enableTxPace (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableTxPace
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableTxPace
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable transmit
 *      pacing for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_PACE=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum = 1;

        CSL_CPGMAC_SL_disableTxPace (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableTxPace
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isGigabitEnabled
 *
 *   @b Description
 *   @n This function indicates if Gigabit mode is enabled in the MAC
 *      control register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Gigabit mode enabled
 *	 @n  FALSE                  Gigabit disabled. 10/100 mode enabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GIG
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum = 1;

        if (CSL_CPGMAC_SL_isGigabitEnabled (portNum) == TRUE)
        {
            // gig enabled
        }
        else
        {
            // gig disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isGigabitEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableGigabit
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable Gigabit
 *      mode (full duplex only) for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GIG=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_enableGigabit (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableGigabit
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableGigabit
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable Gigabit mode
 *      for MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GIG=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_disableGigabit (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableGigabit
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);



/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isTxShortGapEnabled
 *
 *   @b Description
 *   @n This function indicates if Transmit short gap is enabled in the MAC
 *      control register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Transmit short gap with a short IPG enabled
 *	 @n  FALSE                  Transmit short gap disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE
 *
 *   @b Example
 *   @verbatim
 *      Uint32 portNum  =   1;

        if (CSL_CPGMAC_SL_isTxShortGapEnabled (portNum) == TRUE)
        {
            // Tx short gap enabled
        }
        else
        {
            // Tx short gap disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isTxShortGapEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableTxShortGap
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable transmit short
 *      gap for MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_enableTxShortGap (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableTxShortGap
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableTxShortGap
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable transmit
 *      short gap for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_disableTxShortGap (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableTxShortGap
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);



/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isIdleModeEnabled
 *
 *   @b Description
 *   @n This function indicates if Idle mode is enabled in the MAC
 *      control register for MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Idle mode enabled
 *	 @n  FALSE                  Idle mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_CMD_IDLE
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum = 1;

        if (CSL_CPGMAC_SL_isTxShortGapEnabled (portNum) == TRUE)
        {
            // idle mode enabled
        }
        else
        {
            // idle mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isIdleModeEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableIdleMode
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable Idle mode for
 *      MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_CMD_IDLE=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_enableIdleMode (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableIdleMode
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableIdleMode
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable Idle mode for
 *      MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_CMD_IDLE=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_disableIdleMode (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableIdleMode
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);
/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isCastagnoliCRCEnabled
 *
 *   @b Description
 *   @n This function indicates if Castagnoli CRC is enabled in the MAC
 *      control register for MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Castagnoli CRC enabled
 *	 @n  FALSE                  Castagnoli CRC disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_CRC_TYPE
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum = 1;

        if (CSL_CPGMAC_SL_isTxShortGapEnabled (portNum) == TRUE)
        {
            // idle mode enabled
        }
        else
        {
            // idle mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isCastagnoliCRCEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableCastagnoliCRC
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable Castagnoli CRC for
 *      MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_CRC_TYPE=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_enableCastagnoliCRC (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableCastagnoliCRC
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableCastagnoliCRC
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable Castagnoli CRC for
 *      MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_CRC_TYPE=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_disableCastagnoliCRC (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableCastagnoliCRC
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);


/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isIFCTLAEnabled
 *
 *   @b Description
 *   @n This function indicates if IFCTL_A bit is set in the MAC
 *      control register for MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   IFCTL_A bit in MAC control register is set to 1.
 *	 @n  FALSE                  IFCTL_A bit in MAC control register reads 0.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A
 *
 *   @b Example
 *   @verbatim
 *      Uint32 portNum  =   1;

        if (CSL_CPGMAC_SL_isIFCTLAEnabled (portNum) == TRUE)
        {
            // IFCTL_A bit set
        }
        else
        {
            // IFCTL_A bit cleared
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isIFCTLAEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableIFCTLA
 *
 *   @b Description
 *   @n This function enables the IFCTL_A bit in the MAC control register for MAC
 *      port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_enableIFCTLA (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableIFCTLA
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableIFCTLA
 *
 *   @b Description
 *   @n This function configures the MAC control register to clear the IFCTL_A
 *      bit for MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_disableIFCTLA (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableIFCTLA
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isIFCTLBEnabled
 *
 *   @b Description
 *   @n This function indicates if IFCTL_B bit is set in the MAC
 *      control register for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   IFCTL_B bit in MAC control register is set to 1.
 *	 @n  FALSE                  IFCTL_B bit in MAC control register reads 0.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        if (CSL_CPGMAC_SL_isIFCTLBEnabled (portNum) == TRUE)
        {
            // IFCTL_B bit set
        }
        else
        {
            // IFCTL_B bit cleared
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isIFCTLBEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableIFCTLB
 *
 *   @b Description
 *   @n This function enables the IFCTL_B bit in the MAC control register for the
 *      MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_enableIFCTLB (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableIFCTLB
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableIFCTLB
 *
 *   @b Description
 *   @n This function configures the MAC control register to clear the IFCTL_B
 *      bit for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_disableIFCTLB (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableIFCTLB
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isGigForceModeEnabled
 *
 *   @b Description
 *   @n This function indicates if Gigabit mode force bit is enabled in the MAC
 *      control register for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Gigabit mode force enabled.
 *	 @n  FALSE                  Gigabit mode force disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GIG_FORCE
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        if (CSL_CPGMAC_SL_isGigForceModeEnabled (portNum) == TRUE)
        {
            // gig mode force enabled
        }
        else
        {
            // gig mode force disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isGigForceModeEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableGigForceMode
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable Gigabit
 *      force mode for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GIG_FORCE=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_enableGigForceMode (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableGigForceMode
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableGigForceMode
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable Gigabit
 *      force mode for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_GIG_FORCE=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_disableGigForceMode (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableGigForceMode
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isExtControlEnabled
 *
 *   @b Description
 *   @n This function indicates if EXT_EN bit is enabled in the MAC control
 *      register for the MAC port specified. When this bit is set to 1, the Gigabit
 *      and Full duplex mode input is taken from an external source and not from
 *      the settings configured in this register.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   External control mode enabled.
 *	 @n  FALSE                  External control mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum =   1;

        if (CSL_CPGMAC_SL_isExtControlEnabled (portNum) == TRUE)
        {
            // external control mode enabled
        }
        else
        {
            // external control mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isExtControlEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableExtControl
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable external
 *      control of the Gigabit and full duplex mode settings.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_enableExtControl (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableExtControl
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableExtControl
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable external
 *      control of the Gigabit and full duplex settings.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_disableExtControl (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableExtControl
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isExtRxFlowEnabled
 *
 *   @b Description
 *   @n This function indicates if EXT_RX_FLOW_EN bit is enabled in the MAC control
 *      register for the MAC port specified. When this bit is set to 1, the Rx Flow
 *      control enable input is taken from an external source and not from
 *      the rx_flow_en setting configured in this register.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   External Receive Flow control mode enabled.
 *	 @n  FALSE                  External Receive Flow control mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum =   0;

        if (CSL_CPGMAC_SL_isExtRxFlowEnabled (portNum) == TRUE)
        {
            // external control mode enabled
        }
        else
        {
            // external control mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isExtRxFlowEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableExtRxFlow
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable external
 *      control of the Receive Flow Control settings.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_enableExtRxFlow (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableExtRxFlow
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableExtRxFlow
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable external
 *      control of the Receive Flow Control settings.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   0;

        CSL_CPGMAC_SL_disableExtRxFlow (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableExtRxFlow
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isExtTxFlowEnabled
 *
 *   @b Description
 *   @n This function indicates if EXT_TX_FLOW_EN bit is enabled in the MAC control
 *      register for the MAC port specified. When this bit is set to 1, the Tx Flow
 *      control enable input is taken from an external source and not from
 *      the rx_flow_en setting configured in this register.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   External Transmit Flow control mode enabled.
 *	 @n  FALSE                  External Transmit Flow control mode disabled.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum =   0;

        if (CSL_CPGMAC_SL_isExtTxFlowEnabled (portNum) == TRUE)
        {
            // external control mode enabled
        }
        else
        {
            // external control mode disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isExtTxFlowEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableExtTxFlow
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable external
 *      control of the Transmit Flow Control settings.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_enableExtTxFlow (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableExtTxFlow
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableExtTxFlow
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable external
 *      control of the Transmit Flow Control settings.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   0;

        CSL_CPGMAC_SL_disableExtTxFlow (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableExtTxFlow
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);


/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isRxCEFEnabled
 *
 *   @b Description
 *   @n This function indicates if MAC control register is programmed to copy
 *      frames containing errors to its memory.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Copy Error frames mode enabled. Error frames
 *	                            will be transferred to memory.
 *	 @n  FALSE                  Copy Error frames mode disabled. Error frames
 *	                            will be filtered.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CEF_EN
 *
 *   @b Example
 *   @verbatim
 *      Uint32  portNum =   1;

        if (CSL_CPGMAC_SL_isRxCEFEnabled (portNum) == TRUE)
        {
            // Rx CEF enabled
        }
        else
        {
            // Rx CEF disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isRxCEFEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableRxCEF
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable transfer
 *      of frames containing errors.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CEF_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_enableRxCEF (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableRxCEF
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableRxCEF
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable any error
 *      frames being transferred to the memory.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CEF_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_disableRxCEF (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableRxCEF
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isRxCSFEnabled
 *
 *   @b Description
 *   @n This function indicates if MAC control register is programmed to copy
 *      short frames (frames shorter than 64 bytes) to its memory.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Copy Short frames mode enabled. Short frames
 *	                            will be transferred to memory.
 *	 @n  FALSE                  Copy Short frames mode disabled. Short frames
 *	                            will be filtered.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CSF_EN
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        if (CSL_CPGMAC_SL_isRxCSFEnabled (portNum) == TRUE)
        {
            // Rx CSF enabled
        }
        else
        {
            // Rx CSF disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isRxCSFEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableRxCSF
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable transfer
 *      of frames that are shorter than 64 bytes.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CSF_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_enableRxCSF (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableRxCSF
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableRxCSF
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable any short
 *      frames being transferred to the memory.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CSF_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_disableRxCSF (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableRxCSF
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isRxCMFEnabled
 *
 *   @b Description
 *   @n This function indicates if MAC control register is programmed to copy
 *      MAC Control Frames to its memory.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Copy MAC control frames mode enabled. MAC control
 *	                            frames will be transferred to memory.
 *	 @n  FALSE                  Copy MAC control frames mode disabled. MAC control
 *	                            frames will be filtered.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CMF_EN
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        if (CSL_CPGMAC_SL_isRxCMFEnabled (portNum) == TRUE)
        {
            // Rx CMF enabled
        }
        else
        {
            // Rx CMF disabled
        }
	 @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isRxCMFEnabled
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableRxCMF
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable transfer
 *      of MAC control frames.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CMF_EN=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum = 1;

        CSL_CPGMAC_SL_enableRxCMF (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableRxCMF
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableRxCMF
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable any MAC control
 *      frames being transferred to the memory.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_CONTROL_REG_RX_CMF_EN=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_CPGMAC_SL_disableRxCMF (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableRxCMF
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getMacControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of MAC control register in entirety.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 >=0                MAC control register contents.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPGMAC_SL_MACCONTROL_REG
 *
 *   @b Example
 *   @verbatim
        Uint32          macControlVal, portNum = 1;

        macControlVal   =   CSL_CPGMAC_SL_getMacControlReg (portNum);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_getMacControlReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_setMacControlReg
 *
 *   @b Description
 *   @n This function configures the contents of MAC control register in entirety.
 *
 *   @b Arguments
     @verbatim
        portNum             MAC port number for which the reset must be performed.
        macControlRegVal    value to be configured to the MAC control register.
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
 *   @n CPGMAC_SL_MACCONTROL_REG
 *
 *   @b Example
 *   @verbatim
        Uint32          macControlVal, portNum = 1;

        macControlVal   =   CSL_CPGMAC_SL_getMacControlReg ();

        // Enable full duplex, giagbit mode too
        macControlVal   |=  CSL_CPGMAC_SL_MACCONTROL_FULLDUPLEX_EN |
                            CSL_CPGMAC_SL_MACCONTROL_GMII_EN;

        CSL_CPGMAC_SL_setMacControlReg (macControlVal, portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_setMacControlReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32                  macControlRegVal
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getMacStatusReg
 *
 *   @b Description
 *   @n This function retrieves the contents of MAC status register.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
        pMacStatus              CSL_CPGMAC_SL_MACSTATUS structure to be populated with
                                the contents of MAC status register.
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
 *   @n XGE_CPSW_PN_MAC_STATUS_REG_TX_FLOW_ACT,
 *      XGE_CPSW_PN_MAC_STATUS_REG_RX_FLOW_ACT,
 *      XGE_CPSW_PN_MAC_STATUS_REG_EXT_FULLDUPLEX,
 *      XGE_CPSW_PN_MAC_STATUS_REG_EXT_GIG,
 *      XGE_CPSW_PN_MAC_STATUS_REG_EXT_TX_FLOW_EN,
 *      XGE_CPSW_PN_MAC_STATUS_REG_EXT_TX_FLOW_EN,
 *      XGE_CPSW_PN_MAC_STATUS_REG_IDLE
 *
 *   @b Example
 *   @verbatim
        CSL_CPGMAC_SL_MACSTATUS      macStatus;
        Uint32                       portNum = 1;

        CSL_CPGMAC_SL_getMacStatusReg (portNum, &macStatus);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_getMacStatusReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    CSL_CPGMAC_SL_MACSTATUS*    pMacStatus
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_resetMac
 *
 *   @b Description
 *   @n This function issues a software reset to the MAC for the MAC port number
 *      specified.
 *
 *   @b Arguments
     @verbatim
        portNum             MAC port number for which the reset must be performed.
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
 *   @n XGE_CPSW_PN_MAC_SOFT_RESET_REG_SOFT_RESET=1
 *
 *   @b Example
 *   @verbatim
        Uint32 portNum  =   1;

        CSL_CPGMAC_SL_resetMac (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_resetMac
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isMACResetDone
 *
 *   @b Description
 *   @n This function reads the MAC Soft Reset register to check if the software
 *      reset operation has completed.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE               MAC Soft reset done.
 *	 @n  FALSE              MAC Soft reset not yet completed.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_SOFT_RESET_REG_SOFT_RESET
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum = 1;

        CSL_CPGMAC_SL_resetMac (portNum);

        // Wait until MAC software reset completes
        while (!CSL_CPGMAC_SL_isMACResetDone (portNum));

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isMACResetDone
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getRxMaxLen
 *
 *   @b Description
 *   @n This function retrieves the Receive maximum frame length configured in
 *      Receive Maximum length register for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 >=0                Receive maximum frame length read.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
        Uint32          mtu, portNum = 1;

        mtu =   CSL_CPGMAC_SL_getRxMaxLen (portNum);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_getRxMaxLen
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_setRxMaxLen
 *
 *   @b Description
 *   @n This function sets up the Receive maximum frame length in Receive Maximum
 *      Length register for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum             MAC port number for which the settings must be configured.
        rxMaxLen            the Receive maximum frame length to be configered into
                            the Receive Maximum Length register
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
 *   @n XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN
 *
 *   @b Example
 *   @verbatim
        Uint32          mtu, portNum = 1;

        mtu =   1518;

        CSL_CPGMAC_SL_setRxMaxLen (portNum, mtu);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_setRxMaxLen
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32                  rxMaxLen
);


/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getTxGap
 *
 *   @b Description
 *   @n This function retrieves the Transmit inter-packet gap configured in
 *      Transmit Gap register for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 >=0                Transmit inter-packet gap read.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_TX_GAP_REG_TX_GAP
 *
 *   @b Example
 *   @verbatim
        Uint32          txGap, portNum = 0;

        txGap =   CSL_CPGMAC_SL_getTxGap (portNum);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_getTxGap
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_setTxGap
 *
 *   @b Description
 *   @n This function sets up the Transmit inter-packet gap in Transmit Gap
 *      register for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum             MAC port number for which the settings must be configured.
        txGap               Transmit inter-packet gap to be configured into the
                            Tx Gap register
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
 *   @n XGE_CPSW_PN_MAC_TX_GAP_REG_TX_GAP
 *
 *   @b Example
 *   @verbatim
        Uint32          txGap, portNum = 1;

        txGap =   100;

        CSL_CPGMAC_SL_setTxGap (portNum, txGap);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_setTxGap
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32                  txGap
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getRxPauseTimerReg
 *
 *   @b Description
 *   @n This function retrieves the Receive pause timer value configured in
 *      Receive pause timer register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 >=0                Receive pause timer value read.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_RX_PAUSETIMER_REG_RX_PAUSETIMER
 *
 *   @b Example
 *   @verbatim
        Uint32          rxPauseTimer, portNum = 1;

        rxPauseTimer =   CSL_CPGMAC_SL_getRxPauseTimerReg (portNum);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_getRxPauseTimerReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_setRxPauseTimerReg
 *
 *   @b Description
 *   @n This function sets up the Receive pause timer value in Receive pause
 *      timer register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum             MAC port number for which the settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_RX_PAUSETIMER_REG_RX_PAUSETIMER
 *
 *   @b Example
 *   @verbatim
        Uint32          rxPauseTimer = 2, portNum = 1;

        CSL_CPGMAC_SL_setRxPauseTimerReg (portNum, rxPauseTimer);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_setRxPauseTimerReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32                  rxPauseTimer
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getTxPauseTimerReg
 *
 *   @b Description
 *   @n This function retrieves the Transmit pause timer value configured in
 *      Transmit pause timer register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 >=0                Transmit pause timer value read.
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_TX_PAUSETIMER_REG_TX_PAUSETIMER
 *
 *   @b Example
 *   @verbatim
        Uint32          txPauseTimer, portNum = 1;

        txPauseTimer =   CSL_CPGMAC_SL_getTxPauseTimerReg (portNum);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_getTxPauseTimerReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_setTxPauseTimerReg
 *
 *   @b Description
 *   @n This function sets up the Transmit pause timer value in Transmit pause
 *      timer register for the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum             MAC port number for which the settings must be configured.
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
 *   @n XGE_CPSW_PN_MAC_TX_PAUSETIMER_REG_TX_PAUSETIMER
 *
 *   @b Example
 *   @verbatim
        Uint32          txPauseTimer = 2, portNum = 1;

        CSL_CPGMAC_SL_setTxPauseTimerReg (portNum, txPauseTimer);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_setTxPauseTimerReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32                  txPauseTimer
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getEmulControlReg
 *
 *   @b Description
 *   @n This function retrieves the contents of Emulation control register for
 *      the MAC port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum             MAC port number for which the API should retrieve
                            emulation settings for.
        pEmulFreeBit        Emulation free bit.
        pEmulSoftBit        Emulation soft bit.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_EMCONTROL_REG_FREE,
 *      XGE_CPSW_PN_MAC_EMCONTROL_REG_SOFT
 *
 *   @b Example
 *   @verbatim
        Uint32          emulSoftBit, emulFreeBit, portNum = 1;

        CSL_CPGMAC_SL_getEmulControlReg (portNum, &emulFreeBit, &emulSoftBit);
     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_getEmulControlReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32*                 pEmulFreeBit,
    Uint32*                 pEmulSoftBit
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_setEmulControlReg
 *
 *   @b Description
 *   @n This function sets up the contents of Emulation control register for the
 *      MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum             MAC port for which the settings must be configured.
        emulFreeBit         Emulation free bit.
        emulSoftBit         Emulation soft bit.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *   @n None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_MAC_EMCONTROL_REG_FREE,
 *      XGE_CPSW_PN_MAC_EMCONTROL_REG_SOFT
 *
 *   @b Example
 *   @verbatim
        Uint32          emulSoftBit, emulFreeBit, portNum = 1;

        emulFreeBit =   1;
        emulSoftBit =   0;

        CSL_CPGMAC_SL_setEmulControlReg (portNum, emulFreeBit, emulSoftBit);
     @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_setEmulControlReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                  portNum,
    Uint32                  emulFreeBit,
    Uint32                  emulSoftBit
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getMacRxPriMapReg
 *
 *   @b Description
 *   @n This function retrieves the contents of the MAC Receive Packet Priority to
 *      Header Priority Mapping Register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API should retrieve
                                settings for.
        pMacRxPriMap            Array of MAC Rx packet priority map priority values
                                read from the register.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 *   <b> Pre Condition </b>
 *   @n  The input parameter 'pMacRxPriMap' must be large enough to hold all
 *       the 8 priority values read from the register.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0,
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
 *      Uint32      pPortRxPriMap [8], portNum = 1;

        CSL_CPGMAC_SL_getMacRxPriMapReg (portNum, pPortRxPriMap);

	 @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_getMacRxPriMapReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pMacRxPriMap
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_setMacRxPriMapReg
 *
 *   @b Description
 *   @n This function sets up the contents of the MAC Receive Packet Priority
 *      to Header Priority Mapping Register for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port for which the settings must be configured.
        pMacRxPriMap            Array of MAC Rx priority map priority values
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
 *   @n XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0,
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
 *      Uint32      i, pMacRxPriMap [8], portNum = 1;

        for (i = 0; i < 8; i ++)
            pMacRxPriMap [i] = i;

        CSL_CPGMAC_SL_setMacRxPriMapReg (portNum, pMacRxPriMap);
	 @endverbatim
 * =============================================================================
 */
void CSL_CPGMAC_SL_setMacRxPriMapReg
(CSL_Xge_cpswRegs *hCpswRegs,
    Uint32                      portNum,
    Uint32*                     pMacRxPriMap
);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_clearMacStatusTorf
 *
 *   @b Description
 *   @n This function clears Top of receive FIFO flow control trigger bit in
 *      PN_MAC_STATUS register
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number
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
 * =============================================================================
 */
void CSL_CPGMAC_SL_clearMacStatusTorf (CSL_Xge_cpswRegs *hCpswRegs,
                                       Uint32           portNum);
/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_clearMacStatusTorf
 *
 *   @b Description
 *   @n This function clears Top of receive FIFO flow control Priority field in
 *      PN_MAC_STATUS register
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number
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
 * =============================================================================
 */
void CSL_CPGMAC_SL_clearMacStatusTorfPri (CSL_Xge_cpswRegs *hCpswRegs,
                                          Uint32           portNum);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getInterVLANCfg
 *
 *   @b Description
 *   @n This function gets the intervlan configuration for the given
 *      port and route index
 *
 *   @b Arguments
 *    @verbatim
 *       portNum   Port number for which the intervlan configuration is queried
 *       routeIndex Intervlan route index.Valid pointer locations are 1 to x
 *                  (where x is the number of locations - pointer location zero
 *                   is unused).
 *       pInterVLANCfg Intervlan configuration structure to be populated
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPGMAC_SL_getInterVLANCfg(CSL_Xge_cpswRegs *hCpswRegs,
                                   Uint32           portNum,
                                   Uint32           routeIndex,
                                   CSL_CPSW_INTERVLANCFG *pInterVLANCfg);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_setInterVLANCfg
 *
 *   @b Description
 *   @n This function sets the intervlan configuration for the given
 *      port and route index
 *
 *   @b Arguments
 *    @verbatim
 *       portNum   Port number for which the intervlan configuration is queried
 *       routeIndex Intervlan route index.Valid pointer locations are 1 to x
 *                  (where x is the number of locations - pointer location zero
 *                   is unused).
 *       pInterVLANCfg Intervlan configuration structure to be set
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPGMAC_SL_setInterVLANCfg(CSL_Xge_cpswRegs *hCpswRegs,
                                   Uint32           portNum,
                                   Uint32           routeIndex,
                                   CSL_CPSW_INTERVLANCFG *pInterVLANCfg);


/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_getFifoStatus
 *
 *   @b Description
 *   @n This function gets the Enet_Pn_FIFO_Status register contents
 *
 *   @b Arguments
 *    @verbatim
 *       portNum   Port number for which the intervlan configuration is queried
 *       pFifoStatus Fifo status structure to be populated
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  none
 *
 * =============================================================================
 */
void CSL_CPGMAC_SL_getFifoStatus(CSL_Xge_cpswRegs *hCpswRegs,
                                 Uint32           portNum,
                                 CSL_CPGMAC_SL_FIFOSTATUS *pFifoStatus);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_isTxShortGapLimitEnabled
 *
 *   @b Description
 *   @n This function indicates if Transmit short gap limit is enabled in the MAC
 *      control register for the MAC port specified.
 *      When set this bit limits the number of short gap packets transmitted to
 *      100ppm.  Each time a short gap packet is sent, a counter is loaded
 *      with 10,000 and decremented on each wireside clock.
 *      Another short gap packet will not be sent out until the counter
 *      decrements to zero.  This mode is included to preclude the host from
 *      filling up the FIFO and sending every packet out with short gap which
 *      would violate the maximum number of packets per second allowed.
 *      This bit is used only with GMII (not XGMII).
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which the API must
                                retrieve the settings.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 TRUE                   Transmit short gap limit with a short IPG enabled
 *	 @n  FALSE                  Transmit short gap limit disabled.
 *
 * =============================================================================
 */
Uint32 CSL_CPGMAC_SL_isTxShortGapLimitEnabled(CSL_Xge_cpswRegs *hCpswRegs,
                                              Uint32            portNum);

/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_enableTxShortGap
 *
 *   @b Description
 *   @n This function configures the MAC control register to enable transmit short
 *      gap limiting for MAC port specified.
 *      When set this bit limits the number of short gap packets transmitted to
 *      100ppm.  Each time a short gap packet is sent, a counter is loaded
 *      with 10,000 and decremented on each wireside clock.
 *      Another short gap packet will not be sent out until the counter
 *      decrements to zero.  This mode is included to preclude the host from
 *      filling up the FIFO and sending every packet out with short gap which
 *      would violate the maximum number of packets per second allowed.
 *      This bit is used only with GMII (not XGMII).
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
 *	 @endverbatim
 *
 *   @b Return Value
 *	 @n	 None
 *
 * =============================================================================
 */
void CSL_CPGMAC_SL_enableTxShortGapLimit(CSL_Xge_cpswRegs *hCpswRegs,
                                         Uint32            portNum);
/** ============================================================================
 *   @n@b CSL_CPGMAC_SL_disableTxShortGapLimit
 *
 *   @b Description
 *   @n This function configures the MAC control register to disable transmit
 *      short gap limit for the MAC port specified.
 *
 *   @b Arguments
     @verbatim
        portNum                 MAC port number for which settings must be configured.
 *	 @endverbatim
 *
 *   <b> Return Value </b>
 *	 @n	 None
 *
 * =============================================================================
 */
void CSL_CPGMAC_SL_disableTxShortGapLimit(CSL_Xge_cpswRegs *hCpswRegs,
                                          Uint32            portNum);

#ifdef __cplusplus
}
#endif

#endif

/**
@}
*/
