/**
 * @file  csl/src/ip/sgmii/V5/csl_cpsgmii.h
 *
 * @brief
 *  API Auxilary header file for SGMII CSL. Contains the different control
 *  command and status query functions definations
 *
 *  ============================================================================
 *  @n   (C) Copyright 2009-2012, Texas Instruments, Inc.
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

#ifndef CSL_CPSGMII_V4_H_
#define CSL_CPSGMII_V4_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <cslr_cpsgmii.h>
#include <csl_cpsgmii_def.h>

/** @addtogroup CSL_SGMII_FUNCTION
@{ */

/** ============================================================================
 *   @n@b CSL_SGMII_getVersionInfo
 *
 *   @b Description
 *   @n This function retrieves the SGMII version information for the SGMII port
 *      specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which settings must be retrieved.
        sgmiiVersionInfo    CSL_SGMII_VERSION structure that needs to be populated
                            with the version info read from the SGMII_IDVER register.
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
 *   @n CPSGMII_IDVER_REG_MINOR_VER,
 *      CPSGMII_IDVER_REG_MAJOR_VER,
 *      CPSGMII_IDVER_REG_RTL_VER,
 *      CPSGMII_IDVER_REG_TX_IDENT
 *
 *   @b Example
 *   @verbatim
        CSL_SGMII_VERSION   sgmiiVersionInfo;
        Uint32              portNum = 1;

        CSL_SGMII_getVersion (portNum, &sgmiiVersionInfo);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_getVersionInfo
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                  portNum,
    CSL_SGMII_VERSION*      sgmiiVersionInfo
);

/** ============================================================================
 *   @n@b CSL_SGMII_doSoftReset
 *
 *   @b Description
 *   @n This function initiates a CPSGMII logic software reset for the SGMII port
 *      specified by setting the 'SOFT_RESET' bit of the CPSGMII Software Reset
 *      register to 1.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which reset must be performed.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  Software reset on CPSGMII port is initiated. The Soft reset bit is set
 *       back to 0 once the reset operation is completed.
 *
 *   @b Writes
 *   @n CPSGMII_SOFT_RESET_REG_SOFT_RESET=1
 *
 *   @b Affects
 *   @n CPSGMII_SOFT_RESET_REG_SOFT_RESET=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum = 1;

        CSL_SGMII_doSoftReset (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_doSoftReset
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                  portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_getSoftResetStatus
 *
 *   @b Description
 *   @n This function reads the 'SOFT_RESET' bit of the CPSGMII Software Reset
 *      register and returns its contents for the port specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which reset status must be
                            retrieved.
 *   @endverbatim
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSGMII_SOFT_RESET_REG_SOFT_RESET
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum = 1;

        if (!CSL_SGMII_getSoftResetStatus (portNum))
        {
            // Software reset is completed.
        }

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_SGMII_getSoftResetStatus
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                  portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_startRxTxSoftReset
 *
 *   @b Description
 *   @n This function initiates a SGMII Receive and Transmit logic software reset
 *      by setting the 'RT_SOFT_RESET' bit of the CPSGMII Software Reset register
 *      to 1. This reset is to be started when switching between normal and loopback
 *      modes of operation.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which reset must started.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII Rx/Tx Software logic reset initiated.
 *
 *   @b Writes
 *   @n CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum = 1;

        CSL_SGMII_startRxTxSoftReset (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_startRxTxSoftReset
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_endRxTxSoftReset
 *
 *   @b Description
 *   @n This function terminates a SGMII Receive and Transmit logic software reset
 *      initiated earlier by calling @a CSL_SGMII_startRxTxSoftReset () API. This API
 *      sets the 'RT_SOFT_RESET' bit of the CPSGMII Software Reset register
 *      to 0 to do so. This API is to be called once the loopback/normal switch is done.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which reset must be ended.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  @a CSL_SGMII_startRxTxSoftReset () API must have been called earlier to
 *       initiate the Rx/Tx Soft reset before calling this API.
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII Rx/Tx Software logic reset cleared.
 *
 *   @b Writes
 *   @n CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_endRxTxSoftReset (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_endRxTxSoftReset
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_getRxTxSoftResetStatus
 *
 *   @b Description
 *   @n This function reads the 'RT_SOFT_RESET' bit of the CPSGMII Software Reset
 *      register and returns its contents.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which reset status must be
                            retrieved.
 *   @endverbatim
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        if (!CSL_SGMII_getRxTxSoftResetStatus (portNum))
        {
            // Software reset is completed.
        }

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_SGMII_getRxTxSoftResetStatus
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_getSerdesPLLLockStatus
 *
 *   @b Description
 *   @n This function reads the 'PLL_LOCK' bit of the CPSGMII status
 *      register and returns its contents.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which loopback must be enabled.
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
 *   @n CPSGMII_STATUS_REG_LOCK
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_getSerdesPLLLockStatus (portNum);

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_SGMII_getSerdesPLLLockStatus(CSL_CpsgmiiRegs *hCpSgmiiRegs,
                                        Uint32 portNum);

/** ============================================================================
 *   @n@b CSL_SGMII_enableTestPattern
 *
 *   @b Description
 *   @n This function enables test pattern (Force the output of K28.5 on TX_ENC for test purposes)
 *      for the port specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which the operation must be
                            performed.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII test-pattern enabled.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_TEST_PATTERN=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_enableTestPattern (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_enableTestPattern
( CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_DisableTestPattern
 *
 *   @b Description
 *   @n This function clears test pattern enable bit for the port specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which the operation must be
                            performed.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII test-pattern disabled.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_TEST_PATTERN=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_DisableTestPattern (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_DisableTestPattern
( CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_enableMasterMode
 *
 *   @b Description
 *   @n This function configures the CPSGMII in 'Master' mode. Master mode allows
 *      a CPSGMII direct connection with auto-negotiation  or with forced link.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which settings must be configured
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII Master mode enabled.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_MASTER=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_enableMasterMode (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_enableMasterMode
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_disableMasterMode
 *
 *   @b Description
 *   @n This function configures the CPSGMII in 'Slave' mode or in other words
 *      disables the master mode for the port specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which settings must be configured.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII Master mode disabled, slave mode enabled.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_MASTER=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_disableMasterMode (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_disableMasterMode
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_isMasterModeEnabled
 *
 *   @b Description
 *   @n This function reads the 'MASTER' bit of the CPSGMII Control register and
 *      returns 1 to indicate that the CPSGMII is configured in 'Master' mode
 *      and 0 otherwise to indicate 'Slave' mode.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which settings must be retrieved.
 *   @endverbatim
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSGMII_CONTROL_REG_MASTER
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        if (!CSL_SGMII_isMasterModeEnabled (portNum))
        {
            // Slave Mode
        }
        else
        {
            // Master mode
        }

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_SGMII_isMasterModeEnabled
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_enableLoopback
 *
 *   @b Description
 *   @n This function configures the CPSGMII in internal loopback mode for the
 *      port number specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which loopback must be enabled.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII internal loopback mode enabled.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_LOOPBACK=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_enableLoopback (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_enableLoopback
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_disableLoopback
 *
 *   @b Description
 *   @n This function disables CPSGMII internal loopback for the port specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which loopback must be disabled
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII loopback mode disabled.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_LOOPBACK=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_disableLoopback (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_disableLoopback
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_isLoopbackModeEnabled
 *
 *   @b Description
 *   @n This function reads the 'LOOPBACK' bit of the CPSGMII Control register and
 *      returns 1 to indicate that the CPSGMII is configured in 'Loopback' mode
 *      and 0 otherwise.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which loopback mode must be read.
 *   @endverbatim
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSGMII_CONTROL_REG_LOOPBACK
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        if (CSL_SGMII_isLoopbackModeEnabled (portNum))
        {
            // Loopback mode enabled.
        }

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_SGMII_isLoopbackModeEnabled
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_restartAutoNegotiation
 *
 *   @b Description
 *   @n This function restarts link auto-negotiation process for the port
 *      specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which the operation must be
                            performed
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII auto-negotiation restarted.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_MR_AN_RESTART=1,
 *      CPSGMII_CONTROL_REG_MR_AN_RESTART=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_restartAutoNegotiation (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_restartAutoNegotiation
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_enableAutoNegotiation
 *
 *   @b Description
 *   @n This function enables auto-negotiation process for the port specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which the operation must be
                            performed.
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII auto-negotiation enabled.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_MR_AN_ENABLE=1
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_enableAutoNegotiation (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_enableAutoNegotiation
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_disableAutoNegotiation
 *
 *   @b Description
 *   @n This function disables auto-negotiation process for the port number
 *      specified.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which the operation must be
                            performed
 *   @endverbatim
 *
 *   <b> Return Value </b>
 *   @n  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  CPSGMII auto-negotiation disabled.
 *
 *   @b Writes
 *   @n CPSGMII_CONTROL_REG_MR_AN_ENABLE=0
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        CSL_SGMII_disableAutoNegotiation (portNum);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_disableAutoNegotiation
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_isAutoNegotiationEnabled
 *
 *   @b Description
 *   @n This function reads the 'MR_AN_ENABLE' bit of the CPSGMII Control register and
 *      returns 1 to indicate that the CPSGMII is configured in 'auto-negotiate' mode
 *      and 0 otherwise.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which autonegotiation must be enabled.
 *   @endverbatim
 *
 *   <b> Return Value </b>  Uint32
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSGMII_CONTROL_REG_MR_AN_ENABLE
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;

        if (CSL_SGMII_isAutoNegotiationEnabled (portNum))
        {
            // auto-negotiation enabled.
        }

     @endverbatim
 * =============================================================================
 */
Uint32 CSL_SGMII_isAutoNegotiationEnabled
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum
);

/** ============================================================================
 *   @n@b CSL_SGMII_getStatus
 *
 *   @b Description
 *   @n This function retrieves the SGMII status information from the hardware.
 *      The status info returned by this function are valid only if
 *      the 'bIsLocked' bit of status info is set to 1 and must be disregarded
 *      otherwise.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which status must be retrieved.
 *      pSgmiiStatus        CSL_SGMII_STATUS structure pointer that needs to be
 *                          populated with link status information.
 *   @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSGMII_STATUS_REG_LINK,
 *      CPSGMII_STATUS_REG_AN_ERROR,
 *      CPSGMII_STATUS_REG_MR_AN_COMPLETE,
 *      CPSGMII_STATUS_REG_LOCK
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;
        CSL_SGMII_STATUS    sgmiiStatus;

        // Get link status info
        CSL_SGMII_getStatus (portNum, &sgmiiStatus);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_getStatus
(CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                  portNum,
    CSL_SGMII_STATUS*       pSgmiiStatus
);

/** ============================================================================
 *   @n@b CSL_SGMII_setAdvAbility
 *
 *   @b Description
 *   @n This function sets up the 'MR_ADV_ABILITY' register as per the input
 *      specified to this function in 'pSgmiiAdvAbility' param.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which configuration must be done.
 *      pSgmiiAdvAbility    CSL_SGMII_ADVABILITY input params structure that contains
 *                          the advertised ability configuration that needs to be
 *                          populated to the hardware registers.
 *   @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  MR_ADV_ABILITY register setup accordingly.
 *
 *   @b Writes
 *   @n CPSGMII_MR_ADV_ABILITY_REG
 *
 *   @b Example
 *   @verbatim
        Uint32  portNum =   1;
        CSL_SGMII_STATUS    sgmiiAdvAbility;

        sgmiiAdvAbility.bLinkUp     =   1;
        sgmiiAdvAbility.duplexMode  =   CSL_SGMII_FULL_DUPLEX;
        sgmiiAdvAbility.linkSpeed   =   CSL_SGMII_1000_MBPS;
        sgmiiAdvAbility.sgmiimode   =   CSL_SGMII_MODE_SGMII;

        // Configure SGMII Advertised ability info
        CSL_SGMII_setAdvAbility (portNum, &sgmiiAdvAbility);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_setAdvAbility (CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum,
    CSL_SGMII_ADVABILITY*       pSgmiiAdvAbility
);

/** ============================================================================
 *   @n@b CSL_SGMII_getAdvAbility
 *
 *   @b Description
 *   @n This function rerturns the contents of 'MR_ADV_ABILITY' register in the
 *      output parameter 'pSgmiiAdvAbility'.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which the read operation must
                            be done.
 *      pSgmiiAdvAbility    CSL_SGMII_ADVABILITY output param structure pointer
 *                          that needs to be populated with advertised ability
 *                          register contents.
 *   @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSGMII_MR_ADV_ABILITY_REG
 *
 *   @b Example
 *   @verbatim
 *      CSL_SGMII_STATUS    sgmiiAdvAbility;
        Uint32  portNum =   1;

        // Get SGMII Advertised ability info
        CSL_SGMII_getAdvAbility (portNum, &sgmiiAdvAbility);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_getAdvAbility (CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum,
    CSL_SGMII_ADVABILITY*       pSgmiiAdvAbility
);

/** ============================================================================
 *   @n@b CSL_SGMII_getLinkPartnerAdvAbility
 *
 *   @b Description
 *   @n This function rerturns the contents of 'MR_LP_ADV_ABILITY' register in the
 *      output parameter 'pSgmiiAdvAbility'.
 *
 *   @b Arguments
     @verbatim
        portNum             SGMII port number for which read operation must be done
 *      pSgmiiAdvAbility    CSL_SGMII_ADVABILITY output param structure pointer
 *                          that needs to be populated with link partnet advertised
 *                          ability register contents.
 *   @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n CPSGMII_MR_LP_ADV_ABILITY_REG
 *
 *   @b Example
 *   @verbatim
 *      CSL_SGMII_STATUS    sgmiiAdvAbility;
        Uint32  portNum =   1;

        // Get SGMII Advertised ability info
        CSL_SGMII_getLinkPartnerAdvAbility (portNum, &sgmiiAdvAbility);

     @endverbatim
 * =============================================================================
 */
void CSL_SGMII_getLinkPartnerAdvAbility (CSL_CpsgmiiRegs * hCpSgmiiRegs,
    Uint32                      portNum,
    CSL_SGMII_ADVABILITY*       pSgmiiAdvAbility
);


#ifdef __cplusplus
}
#endif

#endif

/**
@}
*/
