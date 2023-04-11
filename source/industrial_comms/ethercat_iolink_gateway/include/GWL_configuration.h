/*!
 *  \file
 *
 *  \brief
 *  Configuration handling
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-12-21
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

#if !(defined PROTECT_GWLCONFIGURATION_H)
#define PROTECT_GWLCONFIGURATION_H    1

#include <stdint.h>
#include <stdbool.h>

#include "gw_errorhandling.h"
#include "GWL_interface.h"        
#include "IIL_interface.h"        // used for IIL_SPortConfigList_t, IIL_SPortStatusList_t
#include "EIL_interface.h"        // used for EIL_EState_t

// IODD defines
#define GWL_CONFIG_IODD_MAXLEN           100U
#define GWL_CONFIG_IODD_MSEQCAP_IDX      0U
#define GWL_CONFIG_IODD_MSEQCAP_SUBIDX   4U
#define GWL_CONFIG_IODD_SERIALNR_IDX     21U
#define GWL_CONFIG_IODD_SERIALNR_SUBIDX  0U
#define GWL_CONFIG_IODD_SERIALNR_LENGTH  16U

// timeout defines
#define GWL_CONFIG_EVTWAITING_TIME_MS    500U

// identification defaults
#define DEFAULT_IOLM_VENDORID        1095U
#define DEFAULT_IOLM_MASTERID        5U
#define DEFAULT_IOLM_MASTERTYPE      2U
#define DEFAULT_IOLM_FEATURE1        0U
#define DEFAULT_IOLM_FEATURE2        0U
#define DEFAULT_IOLM_MAXPORTS        IIL_MAX_PORTNR
#define DEFAULT_IOLM_PORTTYPE        2U
#if ((defined TI_EC_VENDOR) && (TI_EC_VENDOR==1))
 #define DEFAULT_ECAT_VENDORID       0x0000059DU      // primary vendor id(TI), secondary vendor ID (1385 + 0xE0000000)
#else
 #define DEFAULT_ECAT_VENDORID       0x00000569U      // primary vendor id(KB) (0x0569 == 1385), secondary vendor ID (1385 + 0xE0000000)
#endif
#define DEFAULT_ECAT_PRODUCTCODE     0x000187FDU
#define DEFAULT_ECAT_REVISIONID      0x00010000U
#define DEFAULT_ECAT_SERIALNUMBER    0x00000000U
#define DEFAULT_ECAT_PRODUCTNAME     "KUNBUS EtherCAT-IOLink Gateway for AM243X.R5F"
#define DEFAULT_ECAT_HWVERSION       "----"
#define DEFAULT_ECAT_SWVERSION       "----"

#define DEFAULT_EXPECTED_MASTERCONTROL  0x23U

// workaround for Jira ECIOGW-509
#define GWL_WORKAROUND_PORT_CONFIGURATION     1U    // UseCase: reconfigure IOL port
#define GWL_WORKAROUND_PORT_RECONFIGURATION   100U  // UseCase: initial configure IOL port

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of complete port configuration
 *
 * */
typedef struct GWL_SConfIOLPort
{
    /**! \brief IOLink device configuration */
    GWL_sExpPortConfig_t expPortConfig;
    /**! \brief IOLink device status */
    GWL_sCurPortStatus_t curPortStatus;
    /**! \brief IOLink device pin2/pin4 access */
    GWL_sPinAccess_t portPinAccess;
} GWL_SConfIOLPort_t;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of identifications
 *
 * */
typedef struct GWL_SIdentification
{
    /**! \brief current vendorId/masterId/masterType of IOLink Master */
    GWL_SIolmIdent_t     curMasterIdentification;
    /**! \brief expected vendorId/masterId/masterType of IOLink Master */
    GWL_SIolmIdent_t     expMasterIdentification;
    /**! \brief vendorId/prodCOde/revisionIf of EtherCAT slave */
    GWL_SEcatSlaveIdent_t  ecatSlaveIdentification;
} GWL_SIdentification_t;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of configuration data
 *
 * */
typedef struct GWL_SConfiguration
{
    /**! \brief IOLink Master and EtherCAT slave identification */
    GWL_SIdentification_t identification;
    /**! \brief Gateway configuration */
    GWL_SControl_t config;
    /**! \brief IOLink stati */             
    GWL_SStatus_t status;
    /**! \brief Configuration for all IOLink Device ports */
    GWL_SConfIOLPort_t iolPort[IIL_MAX_PORTNR]; 
} GWL_SConfiguration_t;

extern GWL_SConfiguration_t GWL_sConfigData_g;

#define GWL_MIN_THREAD_PRIORITY     8U

#if (defined __cplusplus)
extern "C" {
#endif

extern GW_ECIOL_ERRORCODE GWL_setPDoutValidation(
    const bool validation);

extern bool GWL_getPDoutValidation(
    const uint8_t portNr_p);

extern uint8_t GWL_getIolmPortIndex(
    const uint8_t portNr_p);

extern GW_ECIOL_ERRORCODE GWL_setPortConfiguration(
    const uint8_t portNr_p);

extern GW_ECIOL_ERRORCODE GWL_evaluateIOLdeviceStatus(
    const uint8_t portNr_p);

#ifdef UNIT_TEST
extern void GW_API_dispConfig(void);
#endif

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_GWLCONFIGURATION_H)*/
