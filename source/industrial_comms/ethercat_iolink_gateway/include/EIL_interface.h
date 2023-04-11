/*!
 *  \file
 *
 *  \brief
 *  EtherCAT integration layer (EIL) public interface.
 *
 *  \details
 *
 *  State may change several times per one cycle of upper layer. Hence the event
 *  concept is recommended to catch the important state changes.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-12-06
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

#if !(defined PROTECT_EILINTERFACE_H)
#define PROTECT_EILINTERFACE_H        1

#include <stdint.h>
#include <stdbool.h>

#if (defined __cplusplus)
extern "C" {
#endif

/*! Time to finish the initialization for other tasks (in miliseconds).*/
#define EIL_SLEEP_TIME_BEFORE_START                         50

/*! Number of modules(IO-Link ports) */
#define EIL_NUMBER_OF_MODULES                               8U

#define EIL_ID                                              0x50U

/*! Maximal internal buffer size for the SDO entry not aligned read/write */
#define EIL_SDO_ENTRY_ALIGNED_BUFFER_BYTE_SIZE              128

/* ------ Signals from the EIL ----------------------------------------------*/

/*! EtherCAT slave stack changed + EIL_EState_t */
#define EIL_SIGNAL_STATE_CHANGED                            0x01U
/*! The PDO mapping have been modified */
#define EIL_SIGNAL_CYCLIC_MAPPING_CHANGED                   0x02U
/*! Now prepare a new PDO data for the EtherCAT master */
#define EIL_SIGNAL_TIME_TO_UPDATE_CYCLIC_DATA               0x03U
/*! Received EtherCAT master PDO data */
#define EIL_SIGNAL_CYCLIC_DATA_CHANGED                      0x04U
/*! EtherCAT slave stack detected a communication error */
#define EIL_SIGNAL_COMMUNICATION_ERROR                      0x80U
/*! EtherCAT slave stack license have been expired */
#define EIL_SIGNAL_SLAVE_LICENSE_EXPIRED                    0x81U

/* ------ Signals to the EIL ------------------------------------------------*/

/*! TBD */

/*! EtherCAT integration layer states */
typedef enum EIL_EState {
    /*! EtherCAT intergration layer have been initialized */
    //EIL_eSTATE_INITIALIZED = 0x01,
    /*! EtherCAT slave stack starting to communicate*/
    //EIL_eSTATE_STARTING,
    /*! EtherCAT slave stack failed by initialization */
    //EIL_eSTATE_START_FAILED,
    /*! EtherCAT slave stack successfully started to communicate */
    //EIL_eSTATE_STARTED,
    /*! EtherCAT Slave stack changed state to INIT. */
    EIL_eSTATE_INIT = 1,
    /*! EtherCAT Slave stack changed state to PREOP. */
    EIL_eSTATE_PREOP,
    /*! EtherCAT Slave stack changed state to BOOTSTRAP. */
    EIL_eSTATE_BOOTSTRAP,
    /*! EtherCAT Slave stack changed state to SAFEOP. */
    EIL_eSTATE_SAFEOP,
    /*! EtherCAT Slave stack changed state to OP. */
    EIL_eSTATE_OP = 8,
    /*! EtherCAT slave stack stopping to communicate*/
    //EIL_eSTATE_STOPPING,
    /*! EtherCAT slave stack successfully stopped */
    //EIL_eSTATE_STOPPED
} EIL_EState_t;

/*! Error codes */
typedef enum EIL_EErrorcode {
    /* Function successfully finished */
    EIL_eSUCCESS = 0,
    /* Invalid parameter */
    EIL_eINVAL,
    /* Result too large */
    EIL_eRANGE,
    /* Unavailable try again */
    EIL_eAGAIN,
    /* Function not supported */
    EIL_eNOSYS,
    /* No permission */
    EIL_ePERMISSION,
    /* Operation not supported */
    EIL_eENOTSUP,
    /* Operation canceled */
    EIL_eECANCELED
} EIL_EErrorCode_t;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Notify callback for supported events
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]          eventSource_p       Event source
 *  \param[in]          eventQualifier_p    Event qualifier
 *  \param[in]          eventCode_p         Event cod
 *
 * */
typedef void(* EIL_CBNotify_t)(
    const uint8_t eventSource_p,    
    const uint8_t eventQualifier_p, 
    const uint16_t eventCode_p);   

// max length of EtherCAT product name
#define EIL_CONFIG_MAX_ECAT_PNAME    64U
// max length of EtherCAT version 
#define EIL_CONFIG_MAX_ECAT_VERSION  10U

/*! EtherCAT identification data */
typedef struct EIL_SEcatIdentification
{
    uint8_t   portCount;          /*! count of used IOLink ports */
    uint16_t  vendorId;           /*! used EtherCAT slave vendorId */
    uint32_t  productCode;        /*! used EtherCAT slave productCode */
    uint32_t  revisionId;         /*! used EtherCAT slave revisionId */
    uint32_t  serialNumber;       /*! used EtherCAT slave serial number */
    char      nameStr[EIL_CONFIG_MAX_ECAT_PNAME];        /*! used EtherCAT slave product name */
    char      hwVersion[EIL_CONFIG_MAX_ECAT_VERSION];    /*! used EtherCAT slave hardware version */
    char      swVersion[EIL_CONFIG_MAX_ECAT_VERSION];    /*! used EtherCAT slave software version */
} EIL_SEcatIdentification_t;

/*! Port status info data */
typedef struct EIL_SPortStatus
{
    uint8_t  ioLinkState;         /*! 0xAnn0:1  8Bit         SMI_PortStatus */
    uint8_t  lostFrames;          /*! 0xAnn0:2  8Bit         0, not supported */
    uint8_t  portQuality;         /*! 0xAnn0:3  8Bit         SMI_PortStatus */
    uint8_t  connectionQuality;   /*! 0xAnn0:4  8Bit         SMI_PortStatus */
    uint8_t  pdinPortQualifier;   /*! 0xFxxx:x  8Bit         SMI_PDin */
} EIL_SPortStatus_t;

/*! Expected IO-Link port configuration */
typedef struct EIL_SExpPortConfig
{
    uint32_t deviceID;            /*! 0x8nn0:04 32Bit */
    uint32_t vendorID;            /*! 0x8nn0:05 32Bit */
    uint8_t  revisionID;          /*! 0x8nn0:32 8Bit  */
    uint8_t  mseqCapability;      /*! 0x8nn0:33 8Bit  */
    uint8_t  masterCycleTime;     /*! 0x8nn0:34 8Bit  */
    uint8_t  offsetTime;          /*! 0x8nn0:35 8Bit  */
    uint8_t  inputDataLength;     /*! 0x8nn0:36 8Bit  */
    uint8_t  outputDataLength;    /*! 0x8nn0:37 8Bit  */
    uint8_t  validationType;      /*! 0x20n2:02 8Bit  */
    uint8_t  parameterServer;     /*! 0x20n2:03 8Bit  */
    uint8_t  iQBehavior;          /*! 0x20n3:02 8Bit  */
    uint16_t masterControl;       /*! 0x8nn0:40 8Bit  */
    uint8_t  serialNumber[64];    /*! 0x8nn1:0  64Byte ASCII IODD 21:0 */
} EIL_SExpPortConfig_t;

/*! Current IO-Link port configuration */
typedef struct EIL_SCurPortConfig
{
    uint32_t deviceID;            /*! 0x9nn0:04 32Bit        SMI_PortStatus */
    uint32_t vendorID;            /*! 0x9nn0:05 32Bit        SMI_PortStatus */
    uint8_t  revisionID;          /*! 0x9nn0:32 8Bit         SMI_PortStatus */
    uint8_t  mseqCapability;      /*! 0x9nn0:33 8Bit         IODD 0:4 */
    uint8_t  masterCycleTime;     /*! 0x9nn0:34 8Bit         SMI_PortStatus */
    uint8_t  offsetTime;          /*! 0x9nn0:35 8Bit         0==not supported */
    uint8_t  inputDataLength;     /*! 0x9nn0:36 8Bit         SMI_PortStatus */
    uint8_t  outputDataLength;    /*! 0x9nn0:37 8Bit         SMI_PortStatus */
    uint8_t  serialNumber[64];    /*! 0x9nn1:0  64Byte ASCII IODD 21:0 */
} EIL_SCurPortConfig_t;

extern EIL_SEcatIdentification_t ecatIdentification_g;

extern void EIL_init(void);

extern uint32_t EIL_configure(void);

extern bool EIL_isValidPortNumber(
    const uint8_t portNr_p);

extern uint32_t EIL_setState(
    const EIL_EState_t newState_p,
    const bool withErrorActive_p);

extern void EIL_report(
    const uint8_t eventSource_p,
    const uint8_t eventQualifier_p,
    const uint16_t eventCode_p);

extern uint32_t EIL_readPortPDOutCyclicData(
    const uint8_t portNr_p,
    uint8_t * const pBuffer_p,
    uint8_t * const pLength_p);

extern uint32_t EIL_writePortPDInCyclicData(
    const uint8_t portNr_p,
    const uint8_t * const pBuffer_p,
    const uint8_t length_p,
    const uint8_t qualifier_p);

extern uint32_t EIL_updatePortStatus(
    const uint8_t portNr_p,
    const EIL_SPortStatus_t * const pPortStatus_p,
    const uint8_t portError_p);

extern uint32_t EIL_storeCurPortConfiguration(
    const uint8_t portNr_p,
    const EIL_SCurPortConfig_t * const pCurConf_p);

extern uint32_t EIL_storeExpPortConfiguration(
    const uint8_t portNr_p,
    const EIL_SExpPortConfig_t * const pExpConf_p);

extern void EIL_informIOLMasterEvent(
    const uint8_t portNr_p,
    const uint16_t eventCode_p);

extern void EIL_informIOLDeviceEvent(
    const uint8_t portNr_p,
    const uint16_t eventCode_p);

extern uint32_t EIL_start(void);
extern uint32_t EIL_stop(void);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_EILINTERFACE_H */
