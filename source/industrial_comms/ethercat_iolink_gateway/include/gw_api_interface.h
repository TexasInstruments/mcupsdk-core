/*!
 *  \file
 *
 *  \brief
 *  Gateway API functions
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

#if !(defined PROTECT_GW_API_INTERFACE_H)
#define PROTECT_GW_API_INTERFACE_H      1

#include <stdint.h>
#include <IOLM_SMI.h>       // IO-Link master SMI API

#if (defined __cplusplus)
extern "C" {
#endif

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  enum of API errorcodes
 *
 * */
typedef enum GW_API_EErrorcode {
    GW_API_eSUCCESS = 0,      /** \brief Function successfully finished */
    GW_API_eFIELDBUS,         /** \brief fieldbus bus not supported */
    GW_API_eFIELDBUSSTART,    /** \brief fieldbus bus not started */
    GW_API_eFIELDBUSSTOP,     /** \brief fieldbus bus not stopped */
    GW_API_eFIELDBUSCOM,      /** \brief fieldbus bus communication (FBTL) */
    GW_API_eINVAL,            /** \brief Invalid parameter */
    GW_API_eRANGE,            /** \brief parameter out of range */
    GW_API_eMASTERIDENT,      /** \brief IOL MasterIdent not expected */
    GW_API_eCONF,             /** \brief configuration inconsistent */
    GW_API_eCONFACCESS,       /** \brief configuration not accessable */
    GW_API_ePERMISSION,       /** \brief no permission */
    GW_API_eSTARTED,          /** \brief Gateway is running */
    GW_API_eNOTSTARTED,       /** \brief Gateway is not running */
    GW_API_ePRUINCON,         /** \brief PRU instances inconsistent */  
    GW_API_eNOCLIENTID,       /** \brief no free client id for SMIdirect */  
    GW_API_eNVRAM,            /** \brief NVRAM store */  
    GW_API_eNOTSUPPORTED,     /** \brief function not supported yet */  
    GW_API_eSMIDIRECT_NOMBX,  /** \brief SMIdirect mailbox error */  
    GW_API_eSMIDIRECT_NOEVT,  /** \brief SMIdirect event error */  
    GW_API_eSMIDIRECT_NOUART, /** \brief SMIdirect UART handle error */  
    GW_API_eSMIDIRECT_TASK,   /** \brief SMIdirect task create error */  
} GW_API_EErrorcode_t;

#define GW_API_SERIALNR_LEN   16U
/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of GW_API_setExpPortConfiguration() data
 *
 * */
typedef struct GW_API_SPortExpConfig
{
    uint32_t vendorID;         /**! \brief vendorId for connected device - for check */
    uint32_t deviceID;         /**! \brief deviceId for connected device - for check */
    uint8_t  revisionID;       /**! \brief revisionId for connected device - for check */
    uint8_t  inputDataLength;  /**! \brief length of PDin - for check */
    uint8_t  outputDataLength; /**! \brief length of PDout - for check */
    uint8_t  serialNumber[GW_API_SERIALNR_LEN]; /**! \brief 0x8nn1:0  16Byte, CPC Data - for check */
    uint8_t  portCycleTime;    /**! \brief master cycle time of port */
    uint8_t  iQBehavior;       /**! \brief only GW_API_IQBEHAVIOR_DIGITALINPUT supported */
    uint8_t  validationType;   /**! \brief type 0=no check, 1=VID+DID, 2=VID+DID+SN */
    uint8_t  masterControl;    /**! \brief type 0=inactive, 1=DI, 2=DO, 3=protocol, 4=comstop */
} GW_API_SPortExpConfig_t;
#define GW_API_VALIDATION_TYPE_NONE        0U    
#define GW_API_VALIDATION_TYPE_ID          1U
#define GW_API_VALIDATION_TYPE_SERIAL      2U

#define GW_API_IQBEHAVIOR_NOTSUPPORTED     0U
#define GW_API_IQBEHAVIOR_DIGITALINPUT     1U
// not supported IQBehavior
#define GW_API_IQBEHAVIOR_DIGITALOUTPUT    2U
#define GW_API_IQBEHAVIOR_ANALOGINPUT      3U
#define GW_API_IQBEHAVIOR_ANALOGOUTPUT     4U
#define GW_API_IQBEHAVIOR_POWER2           5U

#define GW_API_MASTERCONTROL_DEACTIVATED   0U
#define GW_API_MASTERCONTROL_DI            1U
#define GW_API_MASTERCONTROL_DO            2U
#define GW_API_MASTERCONTROL_IOLINKPROT    3U
#define GW_API_MASTERCONTROL_IOLCOMSTOP    4U
#define GW_API_MASTERCONTROL_ACTIVE     0x20U
#define GW_API_MASTERCONTROL_DISABLED   0x40U

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of GW_API_getPortStatus() data
 *
 * */
typedef struct GW_API_SPortStatus
{
    uint16_t vendorID;          /**! \brief vendorId from connected device */
    uint32_t deviceID;          /**! \brief deviceId from connected device */
    uint8_t  revisionID;        /**! \brief revisionId from connected device */
    uint8_t  inputDataLength;   /**! \brief length of PDin */
    uint8_t  outputDataLength;  /**! \brief length of PDout */
    uint8_t  serialNumber[GW_API_SERIALNR_LEN];  /**! \brief serialnumber */
    uint8_t  portQualityInfo;   /**! \brief quality of port */
    uint8_t  connectionQuality; /**! \brief quality of connection */
    uint8_t  mseqCapability;    /**! \brief IODD 0x00:4 */                     
    uint8_t  portStatusInfo;    /**! \brief mode of port, described in enum IOLM_SMI_EPortStatus */
    uint8_t  masterCycleTime;   /**! \brief master cycle time of port */
} GW_API_SPortStatus_t;

// Bit defines of GW_API_setIOLFeatures1(), see IOL-Interface-Spec_10002_V113_Jun19 E2
/** DeviceParBatch (SMI_ParamWriteBatch) supported */
#define GW_API_CONF_FEATURE1_DPBATCH_W          0x01U
/** DeviceParBatch (SMI_ParamReadBatch) supported */
#define GW_API_CONF_FEATURE1_DPBATCH_R          0x02U
/** PortPowerOffOn (SMI_PortPowerOffOn) supported */
#define GW_API_CONF_FEATURE1_PORTPOW            0x04U

// Port type defines for GW_API_setIOLPortConfig()
#define GW_API_CONF_PORTTYPES_CLASS_A           0x00U
#define GW_API_CONF_PORTTYPES_CLASS_A_PWRCTR    0x01U
#define GW_API_CONF_PORTTYPES_CLASS_B           0x02U
#define GW_API_CONF_PORTTYPES_FS_PORT_A         0x03U
#define GW_API_CONF_PORTTYPES_FS_PORT_A_OSSDE   0x04U
#define GW_API_CONF_PORTTYPES_FS_PORT_B         0x05U
#define GW_API_CONF_PORTTYPES_W_MASTER          0x06U

extern GW_API_EErrorcode_t GW_API_storePortConfiguration(void);
extern GW_API_EErrorcode_t GW_API_restorePortConfiguration(void);
extern GW_API_EErrorcode_t GW_API_storeIolmIdent(void);
extern GW_API_EErrorcode_t GW_API_restoreIolmIdent(void);
extern GW_API_EErrorcode_t GW_API_storeEcatIdent(void);
extern GW_API_EErrorcode_t GW_API_restoreEcatIdent(void);
extern GW_API_EErrorcode_t GW_API_formatNVRAM(void);

extern uint32_t GW_API_getVersion(void);

extern GW_API_EErrorcode_t GW_API_start(void);
extern GW_API_EErrorcode_t GW_API_stop(void);

extern GW_API_EErrorcode_t GW_API_setIOLVendorId(const uint16_t vendorID_p);
extern GW_API_EErrorcode_t GW_API_setIOLMasterId(const uint32_t masterId_p);
extern GW_API_EErrorcode_t GW_API_setIOLFeatures1(const uint8_t features_p);
extern GW_API_EErrorcode_t GW_API_setIOLPortConfig(const uint8_t portcount_p, const uint8_t *pPortclass_p);

extern GW_API_EErrorcode_t GW_EC_API_setVendorId(const uint32_t vendorID_p);
extern GW_API_EErrorcode_t GW_EC_API_setProductCode(const uint32_t productCode_p);
extern GW_API_EErrorcode_t GW_EC_API_setRevision(const uint32_t revision_p);
extern GW_API_EErrorcode_t GW_EC_API_setProductName(const char* const pNameStr_p);
extern GW_API_EErrorcode_t GW_EC_API_setSerialNumber(const uint32_t serialNumber_p);
extern GW_API_EErrorcode_t GW_EC_API_setHWVersion(const char* const pHwVersion_p);
extern GW_API_EErrorcode_t GW_EC_API_setSWVersion(const char* const pSwVersion_p);

extern GW_API_EErrorcode_t GW_API_setGWHighestPriority(const uint8_t priority_p);
extern GW_API_EErrorcode_t GW_API_setIOLPRUInstance(const uint8_t pruInstance_p);
extern GW_API_EErrorcode_t GW_API_setEcatPRUInstance(const uint8_t pruInstance_p);

// Broadcast (ClientID == 0), EtherCAT (ClientID == 1), free for API (ClientID >= 2)
#define GW_API_FIRST_CLIENTID    2

extern GW_API_EErrorcode_t GW_API_registerSMIClient(
    const uint8_t priority_p, 
    const uint8_t accessRights_p, 
    uint8_t* const pClientId_p, 
    IOLM_SMI_CBGenericCnf cbClientCallBack_p);
extern GW_API_EErrorcode_t GW_API_deregisterSMIClient(const uint8_t clientId_p);
extern GW_API_EErrorcode_t GW_API_smiGenericCommand(
    const uint8_t clientId_p, 
    IOLM_SMI_SHeader* pHeader_p, 
    uint8_t* pArgBlock_p);

extern GW_API_EErrorcode_t GW_API_getExpPortConfiguration(
    const uint8_t portNr_p, 
    GW_API_SPortExpConfig_t* const  psExpPortConfig_p);
extern GW_API_EErrorcode_t GW_API_setExpPortConfiguration(
    const uint8_t portNr_p, 
    const GW_API_SPortExpConfig_t* const  psExpPortConfig_p);
extern GW_API_EErrorcode_t GW_API_getPortStatus(       
    const uint8_t portNr_p, 
    GW_API_SPortStatus_t* const  psCurPortStatus_p);

extern void GW_API_smiGenericCB(
    IOLM_SMI_SHeader* pHeader_p, 
    uint8_t* pArgBlock_p);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_GW_API_INTERFACE_H */
