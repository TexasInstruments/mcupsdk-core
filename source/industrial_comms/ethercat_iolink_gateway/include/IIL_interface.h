/*!
 *  \file
 *
 *  \brief
 *  service functions to abstract IOLink Master communication
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-08-01
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if !(defined PROTECT_IILINTERFACE_H)
#define PROTECT_IILINTERFACE_H     1

#include <stdint.h>

#include <IOLM_SMI.h>

#include "gw_errorhandling.h"
#include "IIL_function.h"

#if (defined __cplusplus)
extern "C" {
#endif

// maximum port count (1..)
#define IIL_MAX_PORTNR          8U
#define IIL_MASTER_PORTNR       0U

// maximum Length of cyclic date per port
#define IIL_MAX_PD_LENGTH       32U

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of IIL_geMasterIdent() data
 *
 * */
typedef struct IIL_SMasterIdent
{
    IOLM_SMI_SMasterident masterIdent;    /**! \brief master identification ie vendorId, masterId ... */
    uint8_t  portTypes[IIL_MAX_PORTNR];   /**! \brief port type of all used ports */
} IIL_SMasterIdent_t;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of IIL_getStatus() data
 *
 * */
typedef struct IIL_SPortStatusList
{
    uint8_t  portStatusInfo;    /**! \brief mode of port, described in enum IOLM_SMI_EPortStatus */
    uint8_t  portQualityInfo;   /**! \brief quality of port */
    uint8_t  revisionID;        /**! \brief revisionId from connected device */
    uint8_t  connectionQuality; /**! \brief quality of connection */
    uint8_t  masterCycleTime;   /**! \brief master cycle time of port */
    uint8_t  inputDataLength;   /**! \brief length of PDin */
    uint8_t  outputDataLength;  /**! \brief length of PDout */
    uint16_t vendorID;          /**! \brief vendorId from connected device */
    uint32_t deviceID;          /**! \brief deviceId from connected device */
} IIL_SPortStatusList_t;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of IIL_setPortConfiguration(), IIL_getPortConfiguration() data
 *
 * */
typedef struct IIL_SPortConfigList
{
    uint8_t  portMode;          /**! \brief port mode, described in enum IOLM_SMI_EPortMode */
    uint8_t  validationBackup;  /**! \brief validation, described in enum IOLM_SMI_EValidationBackup */
    uint8_t  iQBehavior;        /**! \brief iQ, described in enum IOLM_SMI_EIQBehavior */
    uint8_t  portCycleTime;     /**! \brief cycleTime for connected device */
    uint16_t vendorID;          /**! \brief vendorId for connected device */
    uint32_t deviceID;          /**! \brief deviceId for connected device */
} IIL_SPortConfigList_t;

extern GW_ECIOL_ERRORCODE IIL_start(
    void);

extern GW_ECIOL_ERRORCODE IIL_stop(
    void);

extern GW_ECIOL_ERRORCODE IIL_getMasterIdent(
    IIL_SMasterIdent_t* const psMasterIdent_p,
    const IIL_CALLBACK_t      cbSMICallback_p);

extern GW_ECIOL_ERRORCODE IIL_getStatus(
    const uint8_t                portNr_p, 
    IIL_SPortStatusList_t* const psStatusList_p, 
    const IIL_CALLBACK_t         cbSMICallback_p);

extern GW_ECIOL_ERRORCODE IIL_readIODD(
    const uint8_t        portNr_p, 
    const uint16_t       index_p, 
    const uint8_t        subindex_p, 
    uint8_t*       const pResultBuffer_p,
    uint16_t*      const pLength_p, 
    const IIL_CALLBACK_t cbSMICallback_p);

extern GW_ECIOL_ERRORCODE IIL_writeIODD(
    const uint8_t        portNr_p, 
    const uint16_t       index_p,
    const uint8_t        subindex_p, 
    const uint8_t* const pResultBuffer_p,
    const uint8_t        length_p, 
    const IIL_CALLBACK_t cbSMICallback_p);

extern GW_ECIOL_ERRORCODE IIL_readCyclic(
    const uint8_t        portNr_p, 
    uint8_t*       const pResultBuffer_p, 
    uint16_t*      const pLength_p, 
    const IIL_CALLBACK_t cbSMICallback_p);

extern GW_ECIOL_ERRORCODE IIL_writeCyclic(
    const uint8_t        portNr_p, 
    const uint8_t        valid_p, 
    const uint8_t* const pResultBuffer_p, 
    const uint16_t       length_p, 
    const IIL_CALLBACK_t cbSMICallback_p);

extern GW_ECIOL_ERRORCODE IIL_readIQ(
    const uint8_t        portNr_p, 
    uint8_t*       const pResultBuffer_p, 
    uint16_t*      const pLength_p, 
    const IIL_CALLBACK_t cbSMICallback_p);

extern GW_ECIOL_ERRORCODE IIL_setPortConfiguration(
    const uint8_t                      portNr_p, 
    const IIL_SPortConfigList_t* const psConfigList_p,
    const IIL_CALLBACK_t               cbSMICallback_p);

extern GW_ECIOL_ERRORCODE IIL_getPortConfiguration(
    const uint8_t                portNr_p, 
    IIL_SPortConfigList_t* const psConfigList_p,
    const IIL_CALLBACK_t         cbSMICallback_p);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_IILINTERFACE_H */
