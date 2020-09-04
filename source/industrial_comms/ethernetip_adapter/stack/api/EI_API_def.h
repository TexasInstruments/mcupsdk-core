/*
 *  Copyright (c) 2021, Kunbus GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
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


#ifndef EI_API_DEF_H_INC
#define EI_API_DEF_H_INC

#include <stdint.h>
#include <stdarg.h>

#include <osal.h>
#if (defined ETHERNETIP_SO) && (ETHERNETIP_SO==1) // defined if EthernetIP is compiled as a DLL
#ifdef EIPAPI_EXPORTS // defined if we are building the ETHERNETIP DLL (instead of using it)
#define ETHIP_API OSAL_DLL_EXPORT
#else
#define ETHIP_API OSAL_DLL_IMPORT
#endif // EIPAPI_EXPORTS
#define ETHIP_LOC OSAL_DLL_LOCAL
#else // ECATSLAVE_SO is not defined: this means ECATSLAVE is a static lib.
#define ETHIP_API
#define ETHIP_LOC
#endif // ETHERNETIP_SO

#if defined(SOC_AM335x)
/*!
 *  @brief    PRUICSS Instance IDs
 */
typedef enum EI_API_ADP_PRUICSS_MaxInstances_s
{
    EI_API_ADP_PRUICCSS_INSTANCE_ONE=1,
    EI_API_ADP_PRUICCSS_INSTANCE_MAX=2
} EI_API_ADP_PRUICSS_MaxInstances;
#elif defined(SOC_AM65XX)
/*!
 *  @brief    PRUICSS Instance IDs
 */
typedef enum EI_API_ADP_PRUICSS_MaxInstances_s
{
    EI_API_ADP_PRUICCSS_INSTANCE_ONE=1,
    EI_API_ADP_PRUICCSS_INSTANCE_TWO=2,
    EI_API_ADP_PRUICCSS_INSTANCE_THREE=3,
    EI_API_ADP_PRUICCSS_INSTANCE_MAX=4
} EI_API_ADP_PRUICSS_MaxInstances;
#else
/*!
 *  @brief    PRUICSS Instance IDs
 */
typedef enum EI_API_ADP_PRUICSS_MaxInstances_s
{
    EI_API_ADP_PRUICCSS_INSTANCE_ONE=1,
    EI_API_ADP_PRUICCSS_INSTANCE_TWO=2,
    EI_API_ADP_PRUICCSS_INSTANCE_MAX=3
} EI_API_ADP_PRUICSS_MaxInstances;
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define EI_API_CIP_OBD_MAX 256 // set not lower as 20 because init holds around 17 common objects

/*!
 *  \brief Available module and network status LED states.
 */
typedef enum EI_API_ADP_EStatus
{
    EI_API_ADP_eSTATUS_LED_ALL_OFF,
    EI_API_ADP_eSTATUS_LED_ALL_ON,
    EI_API_ADP_eSTATUS_LED_ALL_BLINK,
    EI_API_ADP_eSTATUS_LED_GREEN_ON,
    EI_API_ADP_eSTATUS_LED_GREEN_BLINK,
    EI_API_ADP_eSTATUS_LED_RED_ON,
    EI_API_ADP_eSTATUS_LED_RED_BLINK,
/// @cond INTERNAL
    EI_API_ADP_STATUS_FORCE32BIT = 0xffffffff       //!< Force enum to 32 bit
/// @endcond
} EI_API_ADP_EStatus_t;

/*!
 *  \brief Available CIP service codes.
 *  \details Specifies which services are available on the CIP class or instance.
 *  \ingroup EI_API_CIP_ENUMERATIONS
 */
typedef enum EI_API_CIP_ESc
{
    EI_API_CIP_eSC_GETATTRIBUTESALL  = 0x01,         /*!< Returns the contents of the instance or class attributes defined in the object definition. */
    EI_API_CIP_eSC_RESET             = 0x05,         /*!< Perform a reset. */
    EI_API_CIP_eSC_GETATTRSINGLE     = 0x0E,         /*!< Get_Attribute_Single is used to return the content of the specified attribute. */
    EI_API_CIP_eSC_SETATTRSINGLE     = 0x10,         /*!< Set_Attribute_Single must be set if the attribute value should be modified. */
/// @cond INTERNAL
    EI_API_CIP_eSC_FORCE32BIT        = 0xffffffff    /*!< Force enum to 32 bit. */
/// @endcond
} EI_API_CIP_ESc_t;

/*!
 *  \brief These are the available attribute access rules.
 *  \ingroup EI_API_CIP_ENUMERATIONS
 */
typedef enum EI_API_CIP_EAr
{
    EI_API_CIP_eAR_SET           = 0x01,             //!< Attribute is settable
    EI_API_CIP_eAR_GET           = 0x02,             //!< Attribute is gettable
    EI_API_CIP_eAR_GET_AND_SET   = 0x03,             //!< Attribute is get- and settable
/// @cond INTERNAL
    EI_API_CIP_eAR_FORCE32BIT    = 0xffffffff        //!< Force enum to 32 bit
/// @endcond
} EI_API_CIP_EAr_t;

/*!
 *  \brief These are possible error code for callback functions.
 *  \ingroup EI_API_CIP_ENUMERATIONS
 */
typedef enum EI_API_CIP_CB_ERR_CODE
{
    EI_API_eERR_CB_NO_ERROR        = 0x00,
    EI_API_eERR_CB_VAL_TOO_HIGH    = 0x01,
    EI_API_eERR_CB_VAL_TOO_LOW     = 0x02,
    EI_API_eERR_CB_INVALID_VALUE   = 0x03,
    EI_API_eERR_CB_NOT_ENOUGH_DATA = 0x04,
    EI_API_eERR_CB_TOO_MUCH_DATA   = 0x05,
/// @cond INTERNAL
    EI_API_eERR_CB_FORCE32BIT      = 0xffffffff        //!< Force enum to 32 bit
/// @endcond
} EI_API_CIP_CB_ERR_CODE_t;

typedef uint8_t ei_api_cip_edt_bool;                //!< Elementary data type BOOL of CIP (Common Industrial Protocol)
typedef int8_t ei_api_cip_edt_sint;                 //!< Elementary data type SINT of CIP (Common Industrial Protocol)
typedef int16_t ei_api_cip_edt_int;                 //!< Elementary data type INT of CIP (Common Industrial Protocol)
typedef int32_t ei_api_cip_edt_dint;                //!< Elementary data type DINT of CIP (Common Industrial Protocol)
typedef int64_t ei_api_cip_edt_lint;                //!< Elementary data type LINT of CIP (Common Industrial Protocol)
typedef uint8_t ei_api_cip_edt_usint;               //!< Elementary data type USINT of CIP (Common Industrial Protocol)
typedef uint16_t ei_api_cip_edt_uint;               //!< Elementary data type UINT of CIP (Common Industrial Protocol)
typedef uint32_t ei_api_cip_edt_udint;              //!< Elementary data type UDINT of CIP (Common Industrial Protocol)
typedef uint64_t ei_api_cip_edt_ulint;              //!< Elementary data type ULINT of CIP (Common Industrial Protocol)
typedef float ei_api_cip_edt_real;                  //!< Elementary data type REAL of CIP (Common Industrial Protocol)
typedef double ei_api_cip_edt_lreal;                //!< Elementary data type LREAL of CIP (Common Industrial Protocol)
// other elementary data types just for completeness of the list here
//typedef x ei_api_cip_edt_stime;
//typedef x ei_api_cip_edt_itime;
//typedef x ei_api_cip_edt_time;
//typedef x ei_api_cip_edt_ftime;
//typedef x ei_api_cip_edt_ltime;
//typedef x ei_api_cip_edt_date;
//typedef x ei_api_cip_edt_timeofday;
//typedef x ei_api_cip_edt_dateandtime;             //!< Elementary data type for date and time in CIP (Common Industrial Protocol)
typedef uint8_t ei_api_cip_edt_string;              //!< Elementary data type STRING of CIP (Common Industrial Protocol)
typedef uint16_t ei_api_cip_edt_string2;            //!< Elementary data type STRING2 of CIP (Common Industrial Protocol)
//typedef x ei_api_cip_edt_stringn;
typedef uint8_t ei_api_cip_edt_shortstring;         //!< Elementary data type SHORT_STRING of CIP (Common Industrial Protocol)
//typedef x ei_api_cip_edt_stringi;
typedef uint8_t ei_api_cip_edt_byte;                //!< Elementary data type BYTE of CIP (Common Industrial Protocol)
typedef uint16_t ei_api_cip_edt_word;               //!< Elementary data type WORD of CIP (Common Industrial Protocol)
typedef uint32_t ei_api_cip_edt_dword;              //!< Elementary data type DWORD of CIP (Common Industrial Protocol)
typedef uint64_t ei_api_cip_edt_lword;              //!< Elementary data type LWORD of CIP (Common Industrial Protocol)
typedef uint8_t ei_api_cip_edt_epath;               //!< Elementary data type EPATH of CIP (Common Industrial Protocol)
//typedef uint64_t ei_api_cip_edt_engunit;


/*!
 *  \brief General adapter status collection.
 */
typedef struct EI_API_ADP_SModNetStatus
{
    EI_API_ADP_EStatus_t mod;                       /*!< Module status. */
    EI_API_ADP_EStatus_t net;                       /*!< Network status. */
} EI_API_ADP_SModNetStatus_t;


/*!
 *  \brief General Interface attribute parameter collection
 */
typedef struct EI_API_ADP_SIntfConfBits
{
    uint8_t ETHIntfActive  : 1;                     /*!< 1      Is the interface activated. */
    uint8_t ETHIntfAutoNeg : 1;                     /*!< 2      Is auto negotiation on. */
    uint8_t ETHIntfFDuplex : 1;                     /*!< 3      Is full duplex. */
    uint8_t ETHIntf100MB   : 1;                     /*!< 4      Is 100 Mbit/s. */
    uint8_t rsvd           : 4;                     /*!< 5:8    Reserved. */
} EI_API_ADP_SIntfConfBits_t;

/*!
 *  \brief Structure to use as function parameter (attribute parameters)
 */
typedef union EI_API_ADP_UIntfConf
{
    uint8_t                     all;                /*!< Access all bits. */
    EI_API_ADP_SIntfConfBits_t  bit;                /*!< Access bit directly. */
} EI_API_ADP_UIntfConf_t;

/*!
 *  \brief General QoS attribute parameter collection
 */
typedef struct EI_API_ADP_SQos
{
    uint8_t Q_Tag_Enable;                           /*!< Enables or disables sending 802.1Q frames on CIP and IEEE 1588 messages. */
    uint8_t DSCP_PTP_Event;                         /*!< DSCP value for PTP (IEEE 1588) event messages. */
    uint8_t DSCP_PTP_General;                       /*!< DSCP value for PTP (IEEE 1588) general messages. */
    uint8_t DSCP_Urgent;                            /*!< DSCP value for CIP transport class 0/1 Urgent priority messages. */
    uint8_t DSCP_Scheduled;                         /*!< DSCP value for CIP transport class 0/1 Scheduled priority messages. */
    uint8_t DSCP_High;                              /*!< DSCP value for CIP transport class 0/1 High priority messages. */
    uint8_t DSCP_Low;                               /*!< DSCP value for CIP transport class 0/1 low priority messages. */
    uint8_t DSCP_Explicit;                          /*!< DSCP value for CIP explicit messages (transport class 2/3 and UCMM)
                                                         and all other EtherNet/IP encapsulation messages. */
} EI_API_ADP_SQos_t;


/*!
 *  \brief General multi-cast attribute parameter collection
 *  \details IP multi-cast address configuration.
 */
typedef struct EI_API_ADP_SMcastConfig
{
    uint8_t  allocControl;                       /*!< Multi-cast address allocation control word. Determines how addresses are allocated. */
    uint8_t  reserved;                           /*!< Reserved. */
    uint16_t numMcast;                          /*!< Number of IP multi-cast addresses to allocate for EtherNet/IP. */
    uint32_t mcastStartAddr;                    /*!< Starting multi-cast address from which to begin allocation. */
} EI_API_ADP_SMcastConfig_t;


/*!
 *  \brief Definition for ConnectionManager callback functions
 *  \details Information from the ForwardOpen, LargeForwardOpen or ForwardClose message
 */
typedef struct EI_API_ADP_SCmgrForwardOpenInfo
{
    uint8_t prioTimeTick;   // Priority/Time_tick (high-/low-nibble)
    uint8_t timeOutTicks;   // Time-out ticks
    uint32_t o2tNwConId;    // O->T Network Connection ID
    uint32_t t2oNwConId;    // T->O Network Connection ID
    uint16_t conSerialNum;  // Connection Serial Number
    uint16_t orgVendorId;   // Originator Vendor ID
    uint32_t orgSerialNum;  // Originator Serial Number
    uint8_t timeOutMulti;   // Connection Timeout Multiplier
    uint32_t o2tRPI;        // O->T RPI
    uint32_t o2tConPara;    // O->T Network Connection Parameters
    uint32_t t2oRPI;        // T->O RPI
    uint16_t t2oConPara;    // T->O Network Connection Parameters
    uint8_t typeTrigger;    // Transport Type/Trigger
    uint8_t conPathSize;    // Connection Path Size
    uint8_t conPath[80];    // Connection Path
} EI_API_ADP_SCmgrForwardOpenInfo_t;

typedef struct EI_API_ADP_SCmgrForwardCloseInfo
{
    uint8_t prioTimeTick;   // Priority/Time_tick (high-/low-nibble)
    uint8_t timeOutTicks;   // Time-out ticks
    uint16_t conSerialNum;  // Connection Serial Number
    uint16_t orgVendorId;   // Originator Vendor ID
    uint32_t orgSerialNum;  // Originator Serial Number
    uint8_t conPathSize;    // Connection Path Size
    uint8_t conPath[80];    // Connection Path
} EI_API_ADP_SCmgrForwardCloseInfo_t;


typedef struct EI_API_ADP_SEipStatus{

    uint8_t   gen_status;          // Eip general status code
    uint8_t   extended_status_size;// Eip extended status array size
    uint16_t* extended_status_arr; // Eip extended status array

} EI_API_ADP_SEipStatus_t;


typedef union EI_API_ADP_UCmgrInfo
{
    EI_API_ADP_SCmgrForwardOpenInfo_t forwardOpenInfo;
    EI_API_ADP_SCmgrForwardCloseInfo_t forwardCloseInfo;
} EI_API_ADP_UCmgrInfo_u;

// callback function declaration/typedef

#define T EI_API_CIP_NODE_T
typedef struct T T;
/*!
 *  \brief Function prototype for CIP service callback functions.
 *  \ingroup EI_API_CIP_CALLBACK
 */
typedef void(*EI_API_CIP_CBService)(EI_API_CIP_NODE_T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, EI_API_CIP_ESc_t serviceCode_p, int16_t serviceFlag_p);
/*!
 *  \brief Function prototype for CIP get attribute callback function.
 *  \ingroup EI_API_CIP_CALLBACK
 */
typedef uint32_t(*EI_API_CIP_CBGetAttr)(EI_API_CIP_NODE_T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, uint16_t *len_p, void* pvValue_p);
/*!
 *  \brief Function prototype for CIP set attribute callback function.
 *  \ingroup EI_API_CIP_CALLBACK
 */
typedef uint32_t(*EI_API_CIP_CBSetAttr)(EI_API_CIP_NODE_T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, uint16_t len_p, void* pvValue_p);
#undef T

#define T EI_API_ADP_T
typedef struct T T;

/*!
 *  \brief Function prototype for adapter status (MS, NS) callback function.
 *  \ingroup EI_API_ADP_CALLBACK
 */
typedef void(*EI_API_ADP_CBStatus)(EI_API_ADP_T *pAdp_p, EI_API_ADP_SModNetStatus_t status_p);
typedef void(*EI_API_ADP_CBPhy)(EI_API_ADP_T *pAdp_p, uint8_t interfaceId_p, uint8_t portId_p);
/*!
 *  \brief Function prototype for general stack error callback.
 *  \ingroup EI_API_ADP_CALLBACK
 */
typedef void(*EI_API_ADP_CBStackError)(uint32_t errorCode_p, uint8_t fatal_p, uint8_t numOfParam_p, va_list arg_p);
/*!
 *  \brief Function prototype for CIP CMNGR callback functions.
 *  \ingroup EI_API_CIP_CALLBACK
 */
typedef EI_API_ADP_SEipStatus_t(*EI_API_ADP_CBCmgr)(uint32_t serviceCode_p, EI_API_ADP_UCmgrInfo_u CmgrInfo);
#undef T

#ifdef  __cplusplus 
}
#endif 

#endif // EI_API_DEF_H_INC
