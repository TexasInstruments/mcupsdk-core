/*!
* \file EI_API_ADP_define.h
*
* \brief
* Common EtherNet/IP Adapter definitions .
*
* \author
* KUNBUS GmbH
*
* \date
* 2022-07-15
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef EI_API_ADP_DEFINE_H_INC
#define EI_API_ADP_DEFINE_H_INC

#include "EI_API_def.h" // include global definitions

#include <stdint.h>
#include <stdbool.h>

/*!
 *  \brief Default values for QoS object instance attributes.
 *
 */
#define EI_API_ADP_DEFAULT_8021Q 0
#define EI_API_ADP_DEFAULT_DSCP_PTP_EVENT 59
#define EI_API_ADP_DEFAULT_DSCP_PTP_GENERAL 47
#define EI_API_ADP_DEFAULT_DSCP_URGENT 55
#define EI_API_ADP_DEFAULT_DSCP_SCHEDULED 47
#define EI_API_ADP_DEFAULT_DSCP_HIGH 43
#define EI_API_ADP_DEFAULT_DSCP_LOW 31
#define EI_API_ADP_DEFAULT_DSCP_EXPLICIT 27

/*!
 *  \brief
 *  ADP error codes (base 0x38020Axx).
 *  \ingroup EI_API_ADP_ERROR_CODES
 */
typedef enum EI_API_ADP_EError
{
    // ADP general error codes (base 0x38020Axx)
    EI_API_ADP_eERR_OK                                  = 0x00000000,     /*!< No error, everything should be fine. */
    EI_API_ADP_eERR_GENERAL                             = 0x38020A01,     /*!< General ADP error. */
    EI_API_ADP_eERR_NOT_IMPLEMENTED                     = 0x38020A02,     /*!< Adapter function not implemented. */
    EI_API_ADP_eERR_LENGTH                              = 0x38020A03,     /*!< Length error. */
    EI_API_ADP_eERR_MEMALLOC                            = 0x38020A04,     /*!< Error during memory allocation. */
    EI_API_ADP_eERR_STUB_INVALIDPARAMETER               = 0x38020A05,     /*!< Invalid parameter populate interface. */
    EI_API_ADP_eERR_STUB_NOTINSTANTIATED                = 0x38020A06,     /*!< Interface function not registered. */
    EI_API_ADP_eERR_STUB_NOTIMPLEMENTED                 = 0x38020A07,     /*!< Interface function not implemented. */

    // ADP identity object error codes (base 0x38020Bxx)

    // ADP QoS object error codes (base 0x38020Cxx)
    EI_API_ADP_eERR_QOS_802_1Q_NOT_SUPPORTED            = 0x38020C01,     /*!< 802.1Q tagging not supported. */
    EI_API_ADP_eERR_QOS_PTP_EVENT_VALUE_OUT_OF_RANGE    = 0x38020C02,     /*!< DSCP value for PTP (IEEE 1588) Event messages out of range. */
    EI_API_ADP_eERR_QOS_PTP_GENERAL_VALUE_OUT_OF_RANGE  = 0x38020C03,     /*!< DSCP value for PTP (IEEE 1588) General messages out of range. */
    EI_API_ADP_eERR_QOS_URGENT_VALUE_OUT_OF_RANGE       = 0x38020C04,     /*!< DSCP value for CIP Urgent messages out of range. */
    EI_API_ADP_eERR_QOS_SCHEDULED_VALUE_OUT_OF_RANGE    = 0x38020C05,     /*!< DSCP value for CIP Scheduled priority messages out of range. */
    EI_API_ADP_eERR_QOS_HIGH_VALUE_OUT_OF_RANGE         = 0x38020C06,     /*!< DSCP value for CIP High priority messages out of range. */
    EI_API_ADP_eERR_QOS_LOW_VALUE_OUT_OF_RANGE          = 0x38020C07,     /*!< DSCP value for CIP Low priority messages out of range. */
    EI_API_ADP_eERR_QOS_EXPLICIT_VALUE_OUT_OF_RANGE     = 0x38020C08,     /*!< DSCP value for CIP UCMM, transport class 2/3 messages out of range. */

    /// ADP TCP/IP object error codes (base 0x38020Dxx)
    EI_API_ADP_eERR_TCPIP_IPADDR_VALUE_INVALID          = 0x38020D01,     /*!< IP address value is invalid or reserved. */
    EI_API_ADP_eERR_TCPIP_GATEWAY_VALUE_INVALID         = 0x38020D02,     /*!< Gateway address value is invalid or reserved. */
    EI_API_ADP_eERR_TCPIP_DOMAINNAME_NULL_POINTER       = 0x38020D03,     /*!< Domain name can't to be referenced with NULL pointer. */
    EI_API_ADP_eERR_TCPIP_DOMAINNAME_LENGTH             = 0x38020D04,     /*!< Domain name length is restricted to 48 bytes. */

    /// ADP Ethernet Link object error codes(base 0x38020Exx)

    /// ADP timeSync object error codes (base 0x38020Fxx)
    EI_API_ADP_eERR_TIMESYNC_WRONG_FORMAT               = 0x38020F01,     /*!< Format error. Check specification for this attribute.  */

} EI_API_ADP_EError_t;

/*!
 *  \details
 *  Structure to use as function parameter (see adapter setter and getter functions).
 */
typedef struct EI_API_ADP_SParam
{
    uint8_t  len;               /*!< Parameter length. */
    uint8_t* data;              /*!< Pointer to parameter data. */
} EI_API_ADP_SParam_t;


/*!
 *  \details
 *  The Revision attribute, which consists of Major and Minor Revisions,
 *  identifies the Revision of the item the Identity Object is representing.
 *  The value zero is not valid for either the Major or Minor Revision fields.
 *  If the Device Type of the instance is Embedded Component (0xC8),
 *  the Major Revision and/or Minor Revision may be zero.
 */
typedef struct EI_API_ADP_SRevision
{
    uint8_t major;              /*!< Major Revision number. Limited to values between 1 and 127;
                                     the eighth bit is reserved by CIP and shall have a value of zero. */
    uint8_t minor;              /*!< Used to identify changes in a product that do not affect user configuration choices.
                                     Changes in minor revision are not used by a configuration tool to match a device
                                     with an Electronic Data Sheet. */
} EI_API_ADP_SRevision_t;


///*!
// *  \details
// *  Structure with port specific settings.
// */
typedef struct EI_API_ADP_SPort
{
    // PHY
    void* mdioBaseAddr;         /*!< TODO */
    uint8_t type;               /*!< TODO */
    uint8_t address;            /*!< TODO */
    uint16_t resetTime;         /*!< Reset de-assertion time to be prepared for MDIO communication. */

    // Link
    bool fullDuplex;            /*!< TODO */
    bool linkUp;                /*!< TODO */
    uint8_t linkIntType;        /*!< TODO */
    uint8_t linkSysEvent;       /*!< TODO */
    uint8_t userPhySel;         /*!< TODO */
    uint32_t linkSpeed;         /*!< TODO */
} EI_API_ADP_SPort_t;

typedef struct EI_API_ADP_SInit
{
    OSAL_TASK_EPriority_t taskPrioCyclicIo;                   /* Cyclic IO task priority */
    OSAL_TASK_EPriority_t taskPrioPacket;                     /* Packet task priority */
    OSAL_TASK_EPriority_t taskPrioStatistic;                  /* Statistic task priority */

    struct                                                    /* Data-Link-Layer initialization parameters */
    {
        struct                                                /* Time SYNC initialization parameters */
        {
            OSAL_TASK_EPriority_t taskPrioTsDelayRqTx;        /* TimeSync task priority for TX Delay Request */
            OSAL_TASK_EPriority_t taskPrioTxTimeStamp;        /* TimeSync task priority for TX Time Stamp P1 and P2*/
            OSAL_TASK_EPriority_t taskPrioNRT;                /* TimeSync task priority for NRT */
            OSAL_TASK_EPriority_t taskPrioBackground;         /* TimeSync task priority for Background thread*/
        }ptp;
        struct                                                /* LLDP initialization parameters */
        {
            OSAL_TASK_EPriority_t taskPrioReceive;            /* LLDP receive task priority */
        }lldp;
    }dll;

}EI_API_ADP_SInit_t;

typedef void (*fnResetPhyT)(void* ctxt, uint8_t idx, bool bReset);

#define EIP_MAC_ADDR_LEN             6

typedef struct PRUICSS_Config* PRUICSS_ConfigPtr;
typedef struct ETHPHY_Config*  ETHPHY_ConfigPtr;
typedef void*  ETHPHY_Handle;

typedef struct EIP_SLoadParameter
{
    uint8_t                 ai8uMacAddr[EIP_MAC_ADDR_LEN];
    uint32_t                pruIcssCfgId;               /* PRU-ICSS block enumeration id listed in SysConfig */
    PRUICSS_ConfigPtr       pPruIcssCfg;                /* pointer to PRU-ICSS block configuration */
    ETHPHY_ConfigPtr        pEthPhyCfg[2];              /* array of pointers to ETHPHY configurations */
    ETHPHY_Handle           ethPhyHandle[2];            /* array of ETHPHY handlers */
    bool                    mdioManualMode;             /* MDIO manual mode control flag */
    uint32_t                mdioManualModeBaseAddress;  /* MDIO manual mode base address */
    OSAL_TASK_EPriority_t   taskPrioPhyMdixTask;        /* PHY MDIX task priority */
} EIP_SLoadParameter;

typedef enum
{
    EIP_enLST_INVALID,
    EIP_enLST_UP,
    EIP_enLST_DOWN,
    EIP_enLST_FORCE32BIT = 0xffffffff
} EIP_ELinkState;

typedef enum
{
    EIP_enPORT_INVALID,
    EIP_enPORT_DEFAULT,
    EIP_enPORT_1,
    EIP_enPORT_2,
    EIP_enPORT_ALL,
    EIP_enPORT_FORCE32BIT = 0xffffffff,
} EIP_EEthernetPort;

typedef enum
{
    EIP_enPHS_INVALID,
    EIP_enPHS_10MB,
    EIP_enPHS_100MB,
    EIP_enPHS_1GB,
    EIP_enPHS_FORCE32BIT = 0xffffffff
} EIP_EPhySpeed;

typedef enum
{
    EIP_enPHM_INVALID,
    EIP_enPHM_HALF,
    EIP_enPHM_FULL,
    EIP_enPHM_FORCE32BIT = 0xffffffff
} EIP_EPhyDuplexMode;

typedef enum
{
    EIP_eCFGMETHOD_STATIC = 0,
    EIP_eCFGMETHOD_BOOTP  = 1,
    EIP_eCFGMETHOD_DHCP   = 2,
} EIP_EConfigurationMethod_t;

typedef struct
{
    EIP_EConfigurationMethod_t configurationMethod  : 4;  //
    bool                       dnsEnable            : 1;  // DNS Enable, not supported now
    uint32_t                   reserved             : 27;
} EIP_SConfigurationControl_t;

typedef struct EIP_SPortState
{
    EIP_ELinkState      enLink;
    EIP_EPhySpeed       enSpeed;
    EIP_EPhyDuplexMode  enMode;
} EIP_TPortState;

#endif // EI_API_ADP_DEFINE_H_INC
