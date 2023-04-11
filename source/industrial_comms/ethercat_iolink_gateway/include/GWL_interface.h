/*!
 *  \file
 *
 *  \brief
 *  service functions of Gateway-Layer for EcSlaveIntegration-Layer and IOLMIntegration-Layer
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2023-01-09
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

#if !(defined PROTECT_GWLINTERFACE_H)
#define PROTECT_GWLINTERFACE_H        1

#include <stdint.h>
#include <stdbool.h>

#include <IOLM_SMI.h>

#include "gw_errorhandling.h"
#include "EIL_interface.h"

#if (defined __cplusplus)
extern "C" {
#endif

// event sources
#define GWL_EVENTSRC_INTERNAL        0x10U
#define GWL_EVENTSRC_CONFIGURATION   0x20U
#define GWL_EVENTSRC_IOLMASTER       0x30U    // + portNr
#define GWL_EVENTSRC_IOLDEVICE       0x40U    // + portNr
#define GWL_EVENTSRC_ETHERCAT        0x50U    // + portNr

// event codes for GWL_EVENTSRC_INTERNAL source
#define GW_INTERNALEVT_ECATSTARTED  0x100U
#define GW_INTERNALEVT_IOLMSTARTED  0x200U

// maximum simultaneous SMIdirect clients
#define GW_MAX_CLIENTID       10U
#define GW_BROADCAST_CLIENTID  0U
#define GW_ECAT_CLIENTID       1U

#define GW_MAX_IOLPORTS        8U
#define GW_MAX_IOLD_SERIALNR  64U

#define GW_ECAT_BASICSTATES 0x0FU

#define GWL_INVALID_PORTNR  0xFFU

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of open SMIdirect connections
 *
 * */
typedef struct GWL_sSMIdirectEntry
{
    bool                  busy;             /**! \brief FALSE==entry is free, TRUE==entry is in use */
    uint8_t               priority;         /**! \brief Gateway management, priority of connection  */
    uint8_t               accessRights;     /**! \brief Gateway management, access rights of connection */
    IOLM_SMI_CBGenericCnf cbClientCallBack; /**! \brief callback for SMI response */
} GWL_sSMIdirectEntry_t;

// connection list of ClientIds
extern GWL_sSMIdirectEntry_t GWL_aClientList_g[GW_MAX_CLIENTID];

extern GW_ECIOL_ERRORCODE GWL_init(
    void);

extern GW_ECIOL_ERRORCODE GWL_start(
    void);

extern GW_ECIOL_ERRORCODE GWL_stop(
    void);

extern GW_ECIOL_ERRORCODE GWL_checkIOLMasterIdentification(
    void);

extern void GWL_emitEvent(
    const uint8_t  evtSource_p,
    const uint8_t  evtQualifier_p,
    const uint16_t evtCode_p);

extern void GWL_emitCyclic(
    const uint8_t  evtSource_p);

extern GW_ECIOL_ERRORCODE GWL_readPortIODD(
    const uint8_t        portNr_p,
    const uint16_t       index_p,
    const uint8_t        subindex_p,
    uint8_t*       const pResultBuffer_p,
    uint8_t*       const pLength_p);

extern GW_ECIOL_ERRORCODE GWL_writePortIODD(
    const uint8_t        portNr_p,
    const uint16_t       index_p,
    const uint8_t        subindex_p,
    uint8_t*       const pResultBuffer_p,
    const uint8_t        length_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of EtherCAT slave identifications
 *
 * */
typedef struct GWL_SEcatSlaveIdent
{
    /**! \brief vendorId of EtherCAT slave */
    uint32_t vendorID;
    /**! \brief product code of EtherCAT slave */
    uint32_t productCode;
    /**! \brief revisionId of EtherCAT slave */
    uint32_t revisionID;
    /**! \brief serialNumber of EtherCAT slave */
    uint32_t serialNumber;
    /**! \brief product string of EtherCAT slave */
    char     nameStr[EIL_CONFIG_MAX_ECAT_PNAME];
    /**! \brief HW version string of EtherCAT slave */
    char     hwVersion[EIL_CONFIG_MAX_ECAT_VERSION];     
    /**! \brief SW version string of EtherCAT slave */
    char     swVersion[EIL_CONFIG_MAX_ECAT_VERSION];      
} GWL_SEcatSlaveIdent_t;

extern GW_ECIOL_ERRORCODE GWL_getEcatIdentification(GWL_SEcatSlaveIdent_t* const psEcatIdent_p);
extern GW_ECIOL_ERRORCODE GWL_setEcatIdentification(const GWL_SEcatSlaveIdent_t* const psExpectedData_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of IIL_geMasterIdent() data
 *
 * */
typedef struct GWL_SIolmIdent
{
    /**< \brief see IOL-Interface-Spec_10002_V113_Jun19.pdf E.2 MasterIdent */
    uint16_t vendorID;    /**< \brief Big endian. */
    uint32_t masterID;    /**< \brief Big endian. */
    uint8_t  masterType;                  
    uint8_t  features_1;                  
    uint8_t  features_2;                  
    uint8_t  maxNumberOfPorts;            
    uint8_t  portTypes[GW_MAX_IOLPORTS];  
} GWL_SIolmIdent_t;

extern GW_ECIOL_ERRORCODE GWL_getExpIolmIdentification( GWL_SIolmIdent_t* const psExpectedData_p);
extern GW_ECIOL_ERRORCODE GWL_setExpIolmIdentification(const GWL_SIolmIdent_t* const psExpectedData_p);
extern GW_ECIOL_ERRORCODE GWL_getCurIolmIdentification( GWL_SIolmIdent_t* const psExpectedData_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of current configuration data
 * 
 * */
typedef struct GWL_sCurPortConfig
{
    uint32_t deviceID;          /**! \brief 0x9nn0:04 32Bit.  CPC Data - SMI_PortConfiguration */
    uint32_t vendorID;          /**! \brief 0x9nn0:05 32Bit,  CPC Data - SMI_PortConfiguration */
    uint8_t  revisionID;        /**! \brief 0x9nn0:32 8Bit,   CPC Data - for check */
    uint8_t  mseqCapability;    /**! \brief 0x9nn0:33 8Bit,   IODD 0:4 read only!, perhaps used for older Revisions */
    uint8_t  masterCycleTime;   /**! \brief 0x9nn0:34 8Bit,   CPC Data - SMI_PortConfiguration */
    uint8_t  offsetTime;        /**! \brief 0x9nn0:35 8Bit,   0==not supported */
    uint8_t  inputDataLength;   /**! \brief 0x9nn0:36 8 Bit,  CPC Data - for check */
    uint8_t  outputDataLength;  /**! \brief 0x9nn0:37 8Bit,   CPC Data - for check */                                      
    bool     validPDout;        /**! \brief for EtherCAT Operational state true, otherwise false */
    uint8_t  lostFrames;        /**! \brief 0xAnn0:02 8Bit,   0==not supported */
    uint8_t  portStatusInfo;    /**! \brief mode of port, described in enum IOLM_SMI_EPortStatus */
    uint8_t  portQualityInfo;   /**! \brief quality of port */
    uint8_t  connectionQuality; /**! \brief quality of connection */
    uint8_t  portError;         /**! \brief 0xF100 4Bit */
    uint8_t  serialNumber[GW_MAX_IOLD_SERIALNR];  /**! \brief 0x9nn1:0  16Byte, CPC Data - for check */
} GWL_sCurPortStatus_t;

extern GW_ECIOL_ERRORCODE GWL_getCurPortStatus(const uint8_t portNr_p, GWL_sCurPortStatus_t * const psExpectedData_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of expected configuration data
 * 
 * */
typedef struct GWL_sExpPortConfig
{
    uint32_t deviceID;          /**! \brief 0x8nn0:04 32Bit.  CPC Data - SMI_PortConfiguration */
    uint32_t vendorID;          /**! \brief 0x8nn0:05 32Bit,  CPC Data - SMI_PortConfiguration */
    uint8_t  revisionID;        /**! \brief 0x8nn0:32 8Bit,   CPC Data - for check */
    uint8_t  mseqCapability;    /**! \brief 0x8nn0:33 8Bit,   IODD 0:4 read only!, perhaps used for older Revisions */
    uint8_t  portCycleTime;     /**! \brief 0x8nn0:34 8Bit,   CPC Data - SMI_PortConfiguration */
    uint8_t  offsetTime;        /**! \brief 0x8nn0:35 8Bit,   0==not supported */
    uint8_t  inputDataLength;   /**! \brief 0x8nn0:36 8 Bit,  CPC Data - for check */
    uint8_t  outputDataLength;  /**! \brief 0x8nn0:37 8Bit,   CPC Data - for check */
    uint8_t  validationType;    /**! \brief 0x20n2:2  8Bit,   CPC Data - SMI_PortConfiguration validationBackup */
    uint8_t  parameterServer;   /**! \brief 0x20n2:3  8Bit,   CPC Data - SMI_PortConfiguration validationBackup */
    uint8_t  iQBehavior;        /**! \brief 0x20n2:3  8Bit,   CPC Data - SMI_PortConfiguration iQBehavior? */
    uint16_t masterControl;     /**! \brief 0x8nn0:40 16Bit,  CPC Data - SMI_PortConfiguration portMode? */
    uint8_t  source;            /**! \brief GWL_EXPCONFIG_SRC_ECAT, GWL_EXPCONFIG_SRC_API, GWL_EXPCONFIG_SRC_SMI */
    bool     enablePullPlug;    /**! \brief used for hotplug, for future use */
    uint8_t  serialNumber[GW_MAX_IOLD_SERIALNR];  /**! \brief 0x8nn1:0  16Byte, CPC Data - for check */
} GWL_sExpPortConfig_t;

#define GWL_EXPCONFIG_SRC_ECAT           1U
#define GWL_EXPCONFIG_SRC_API            2U
#define GWL_EXPCONFIG_SRC_SMI            3U

#define GWL_EXPCONFIG_MC_MODEMASK        0x0FU
#define GWL_EXPCONFIG_MC_INACTIVE        0U
#define GWL_EXPCONFIG_MC_DI              1U
#define GWL_EXPCONFIG_MC_DO              2U
#define GWL_EXPCONFIG_MC_IOLINKPROT      3U
#define GWL_EXPCONFIG_MC_IOLCOMSTOP      4U
#define GWL_EXPCONFIG_MC_DS_ACTIVE       0x20U
#define GWL_EXPCONFIG_MC_DS_DISABLED     0x40U

extern GW_ECIOL_ERRORCODE GWL_setExpPortConfiguration(
    const uint8_t portNr_p,
    const uint8_t source_p,
    const GWL_sExpPortConfig_t * const pExpConf_p);

extern GW_ECIOL_ERRORCODE GWL_getExpPortConfiguration(
    const uint8_t portNr_p,
    GWL_sExpPortConfig_t * const pExpConf_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of access data Pin2 (IQ) and Pin4 (CQ)
 * 
 * */
typedef struct GWL_sPinAccess
{
    uint8_t  pin2SafeState;     /**! \brief 0x2nn3:01 8Bit,   safe state IQ low, high, last state */
    uint8_t  pin4SafeState;     /**! \brief 0x2nn2:01 8Bit,   safe state CQ low, high, last state */
    bool     pin2CurOut;        /**! \brief 0x3nn0:01 8Bit,   current output IQ (used for safe state) */
    bool     pin4CurOut;        /**! \brief cyclic data,      current output CQ (used for safe state) */
} GWL_sPinAccess_t;

#define GWL_PINACCESS_SAFESTATE_LOW   0x00U
#define GWL_PINACCESS_SAFESTATE_HIGH  0x01U
#define GWL_PINACCESS_SAFESTATE_LAST  0x02U

extern GW_ECIOL_ERRORCODE GWL_setSafeStateIQ(
    const uint8_t portNr_p,
    const uint8_t state_p);

extern GW_ECIOL_ERRORCODE GWL_setSafeStateCQ(
    const uint8_t portNr_p,
    const uint8_t state_p);

extern GW_ECIOL_ERRORCODE GWL_getSafeStateIQ(
    const uint8_t portNr_p,
    uint8_t* const pState_p);

extern GW_ECIOL_ERRORCODE GWL_getSafeStateCQ(
    const uint8_t portNr_p,
    uint8_t* const pState_p);

extern GW_ECIOL_ERRORCODE GWL_readPDInIQ(
    const uint8_t portNr_p,
    bool* const pValue_p);

extern GW_ECIOL_ERRORCODE GWL_readPDOutIQ(
    const uint8_t portNr_p,
    bool* const pValue_p);

extern GW_ECIOL_ERRORCODE GWL_writePDOutIQ(
    const uint8_t portNr_p,
    const bool value_p);

extern GW_ECIOL_ERRORCODE GWL_readPDOutCQ(
    const uint8_t portNr_p,
    bool* const pValue_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of gateway status
 *
 * */
typedef struct GWL_SStatus
{
    /**! \brief current EtherCAT state like PreOp, SafeOp, Op */
    EIL_EState_t ecatState;
    /**! \brief Bit0=IOLMstarted, Bit1=ECATstarted, Bit2=IOLconfigured */
    uint8_t      gwState;
    /**! \brief minimum highest gateway task priority */
    uint8_t      minHighestPriority;
} GWL_SStatus_t;

// layout of gwState
#define GWL_GWSTATE_IOLM_STARTED    0x01U
#define GWL_GWSTATE_ECAT_STARTED    0x02U
#define GWL_GWSTATE_CONFIGURED      0x04U
#define GWL_GWSTATE_POWER_ALLOWED   0x08U
#define GWL_GWSTATE_INIT_DEFAULTS   0x10U

extern GW_ECIOL_ERRORCODE GWL_getStatus( GWL_SStatus_t* const psStatusData_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of gateway configuration
 *
 * */
typedef struct GWL_SControl
{
    /**! \brief highest gateway task priority */
    uint8_t      highestPriority;
    /**! \brief EtherCAT PRU instance */
    uint8_t      ecatPRUInstance;                        
    /**! \brief IOLink Master PRU instance */
    uint8_t      iolmPRUInstance;
} GWL_SControl_t;

extern GW_ECIOL_ERRORCODE GWL_getControl( GWL_SControl_t* const psControlData_p);
extern GW_ECIOL_ERRORCODE GWL_setControl( const GWL_SControl_t* const psControlData_p);

extern GW_ECIOL_ERRORCODE GWL_storePortConfigurationToNVRAM(void);
extern GW_ECIOL_ERRORCODE GWL_restorePortConfigurationFromNVRAM(void);
extern GW_ECIOL_ERRORCODE GWL_storeIolmIdentificationToNVRAM(void);
extern GW_ECIOL_ERRORCODE GWL_restoreIolmIdentificationFromNVRAM(void);
extern GW_ECIOL_ERRORCODE GWL_storeEcatIdentificationToNVRAM(void);
extern GW_ECIOL_ERRORCODE GWL_restoreEcatIdentificationFromNVRAM(void);

extern uint8_t GWL_getMinThreadPriotity(void);
extern void GWL_setHighestPriority(const uint8_t priority_p);
extern uint8_t GWL_getHighestPriority(void);
extern void GWL_setEcatPruInstance(const uint8_t instance_p);
extern uint8_t GWL_getEcatPruInstance(void);
extern void GWL_setIolmPruInstance(const uint8_t instance_p);
extern uint8_t GWL_getIolmPruInstance(void);

extern void GWL_setEcatStarted(const bool isStarted_p);
extern bool GWL_isEcatStarted(void);
extern void GWL_setIolmStarted(const bool isStarted_p);
extern bool GWL_isIolmStarted(void);
extern bool GWL_isGwStarted(void);
extern void GWL_setGwDefaultInit(const bool isDefault_p);
extern bool GWL_isGwDefaultInit(void);
extern bool GWL_isPowerOnOffAllowed(void);
extern bool GWL_isConfigured(void);

extern GW_ECIOL_ERRORCODE GWL_prepareEcatStateChange(EIL_EState_t newState_p);
extern EIL_EState_t GWL_getCurEcatState(void);
extern void GWL_storeEcatState(EIL_EState_t curState);
extern bool GWL_isEcatPreOP(void);
extern bool GWL_isPortOperational(const uint8_t portNr_p);
extern bool GWL_isPortDigitalOut(const uint8_t portNr_p);

extern uint8_t GWL_getCurIolDeviceState(uint8_t portNr_p);
extern uint8_t GWL_getIolmPortCount(void);
extern bool GWL_isPortPDin(const uint8_t portNr_p);
extern bool GWL_isPortPDout(const uint8_t portNr_p);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_GWLINTERFACE_H */
