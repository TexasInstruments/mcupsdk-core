/*
 *  Copyright (c) 2021, KUNBUS GmbH
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


#ifndef EI_API_CIP_H_INC
#define EI_API_CIP_H_INC

#include <api/EI_API_def.h>

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  \brief
 *  CIP error codes (base 0x38020xxx).
 *  \ingroup EI_API_CIP_ERROR_CODES
 */
    typedef enum EI_API_CIP_EError
    {
        EI_API_CIP_eERR_OK                          = 0x00000000,   /*!< No error, everything should be fine. */

        // CIP general error codes (base 0x380201xx)
        EI_API_CIP_eERR_GENERAL                     = 0x38020101,   /*!< General CIP error. */
        EI_API_CIP_eERR_MEMALLOC                    = 0x38020102,   /*!< Memory allocation error. */
        EI_API_CIP_eERR_NOT_IMPLEMENTED             = 0x38020103,   /*!< Not implemented function. */
        EI_API_CIP_eERR_DICTIONARY_FULL             = 0x38020104,   /*!< CIP dictionary is full, no more class or instance can be added. */
        EI_API_CIP_eERR_NODE_INVALID                = 0x38020105,   /*!< CIP node is invalid, possibly EI_API_CIP_NODE_new() was not called. */

        // CIP error codes for class (base 0x380202xx)
        EI_API_CIP_eERR_CLASS_ALREADY_EXIST         = 0x38020201,   /*!< Class already exists in CIP node dictionary. */
        EI_API_CIP_eERR_CLASS_DOES_NOT_EXIST        = 0x38020202,   /*!< Class does not exists in CIP node dictionary. */
        EI_API_CIP_eERR_CLASS_UNKNOWN_STATE         = 0x38020203,   /*!< Unknown state during class operation reached. */
        EI_API_CIP_eERR_CLASS_BLACKLIST             = 0x38020204,   /*!< Operation on the CIP object with specified identifier (class ID) are not allowed. */

        // CIP error codes for service (base 0x380203xx)
        EI_API_CIP_eERR_SERVICE_ALREADY_EXIST       = 0x38020301,   /*!< Service already exist. */
        EI_API_CIP_eERR_SERVICE_DOES_NOT_EXIST      = 0x38020302,   /*!< Service does not exist. */
        EI_API_CIP_eERR_SERVICE_PARAMETER           = 0x38020303,   /*!< Wrong or invalid service parameters. */
        EI_API_CIP_eERR_SERVICE_INVALID             = 0x38020304,   /*!< Service is invalid, possibly NULL pointer. */

        // CIP error codes for instance (base 0x380204xx)
        EI_API_CIP_eERR_INSTANCE_ALREADY_EXIST      = 0x38020401,   /*!< Instance already exists in CIP node dictionary. */
        EI_API_CIP_eERR_INSTANCE_DOES_NOT_EXIST     = 0x38020402,   /*!< Instance does not exist in CIP node dictionary. */
        EI_API_CIP_eERR_INSTANCE_UNKNOWN_STATE      = 0x38020403,   /*!< Unknown state during class operation reached. */

        // CIP error codes for attribute (base 0x380205xx)
        EI_API_CIP_eERR_ATTRIBUTE_ALREADY_EXIST     = 0x38020501,   /*!< Attribute already exists in class or instance. */
        EI_API_CIP_eERR_ATTRIBUTE_DOES_NOT_EXIST    = 0x38020502,   /*!< Attribute does not exist in class or instance. */
        EI_API_CIP_eERR_ATTRIBUTE_SET               = 0x38020503,   /*!< Attribute set error, this can be happen if wrong size was passed as parameter. */
        EI_API_CIP_eERR_ATTRIBUTE_NOT_IMPLEMENTED   = 0x38020504,   /*!< Attribute function not implemented. */
        EI_API_CIP_eERR_ATTRIBUTE_UNKNOWN_STATE     = 0x38020505,   /*!< Unknown state during attribute operation reached. */
        EI_API_CIP_eERR_ATTRIBUTE_EDT_SIZE          = 0x38020506,   /*!< Passed elementary data type size for e.g short string not in valid range. */
        EI_API_CIP_eERR_ATTRIBUTE_CDT_SIZE          = 0x38020507,   /*!< Passed constructed data type size for e.g farray not in valid range. */
        EI_API_CIP_eERR_ATTRIBUTE_NO_CALLBACK       = 0x38020508,   /*!< No user callback is registered for the attribute. */
        EI_API_CIP_eERR_ATTRIBUTE_INVALID_CALLBACK  = 0x38020509,   /*!< Callback can not be registered (invalid function pointer). */
        EI_API_CIP_eERR_ATTRIBUTE_INVALID_VALUE     = 0x3802050A,   /*!< Invalid data pointer. */
        EI_API_CIP_eERR_ATTRIBUTE_INVALID           = 0x3802050B,   /*!< Attribute is invalid, possibly NULL pointer. */
        EI_API_CIP_eERR_ATTRIBUTE_INVALID_TYPE      = 0x3802050C,   /*!< Attribute type is invalid. */

        // CIP error codes for assembly related operations (base 0x380206xx)
        EI_API_CIP_eERR_ASSEMBLY_ALREADY_EXISTS     = 0x38020601,   /*!< The assembly with the given ID already exists. */
        EI_API_CIP_eERR_ASSEMBLY_DOES_NOT_EXIST     = 0x38020602,   /*!< The assembly with the given ID could not be found. */
        EI_API_CIP_eERR_ASSEMBLY_BAD_ATTRIBUTE      = 0x38020603,   /*!< The attribute supplied is invalid. */
        EI_API_CIP_eERR_ASSEMBLY_MAXLENGTH_EXCEEDED = 0x38020604,   /*!< The total length of assemblies data exceeds the maximum connection size. */
        EI_API_CIP_eERR_ASSEMBLY_INVALID_ACCESSMODE = 0x38020605,   /*!< Attempted to add a read only attribute to a writable assembly. */
        EI_API_CIP_eERR_ASSEMBLY_BAD_BUFFERSIZE     = 0x38020606,   /*!< Buffer size does not match assembly members' size. */
        EI_API_CIP_eERR_ASSEMBLY_INVALID_SIZE       = 0x38020607,   /*!< Invalid size pointer. */
        EI_API_CIP_eERR_ASSEMBLY_INVALID_BUFFER     = 0x38020608,   /*!< Invalid buffer pointer. */

        // CIP error codes for I/O related operations (base 0x380207xx)
        EI_API_CIP_eERR_CYCLIC_BAD_ATTRIBUTE        = 0x38020701,   /*!< The attribute supplied is invalid. */
        EI_API_CIP_eERR_CYCLIC_ALREADY_CYCLIC       = 0x38020702,   /*!< The attribute supplied is already cyclic */
        EI_API_CIP_eERR_CYCLIC_NOT_ALLOCATED        = 0x38020703    /*!< The cyclic buffers are not allocated */
    } EI_API_CIP_EError_t;

/// @cond INTERNAL
#define T EI_API_CIP_NODE_T
/// @endcond
/// @cond INTERNAL
typedef struct T T;
/// @endcond

/*!
 *  \brief Elementary Data Type Reporting
 *  \ingroup EI_API_CIP_ENUMERATIONS
 */
typedef enum EI_API_CIP_EEdt
{
    EI_API_CIP_eEDT_NO =                                      0,             /*!< No elementary data type. */
    EI_API_CIP_eEDT_BOOL =                                    1,             /*!< Logical Boolean with values TRUE and FALSE. */
    EI_API_CIP_eEDT_SINT =                                    2,             /*!< Signed 8-bit integer value. */
    EI_API_CIP_eEDT_INT =                                     3,             /*!< Signed 16-bit integer value. */
    EI_API_CIP_eEDT_DINT =                                    4,             /*!< Signed 32-bit integer value. */
    EI_API_CIP_eEDT_LINT =                                    5,             /*!< Signed 64-bit integer value. */
    EI_API_CIP_eEDT_USINT =                                   6,             /*!< Unsigned 8-bit integer value. */
    EI_API_CIP_eEDT_UINT =                                    7,             /*!< Unsigned 16-bit integer value. */
    EI_API_CIP_eEDT_UDINT =                                   8,             /*!< Unsigned 32-bit integer value. */
    EI_API_CIP_eEDT_ULINT =                                   9,             /*!< Unsigned 64-bit integer value. */
    EI_API_CIP_eEDT_REAL =                                   10,             /*!< 32-bit floating point value. */
    EI_API_CIP_eEDT_LREAL =                                  11,             /*!< 64-bit floating point value. */
    EI_API_CIP_eEDT_STIME =                                  12,             /*!< System time information. */
    EI_API_CIP_eEDT_DATE =                                   13,             /*!< Date information. */
    EI_API_CIP_eEDT_TIMEOFDAY =                              14,             /*!< Time of day. */
    EI_API_CIP_eEDT_DATEANDTIME =                            15,             /*!< Date and time of day. */
    EI_API_CIP_eEDT_STRING =                                 16,             /*!< Character string (2 byte character count, 1 byte per character). */
    EI_API_CIP_eEDT_BYTE =                                   17,             /*!< bit string 8-bits. */
    EI_API_CIP_eEDT_WORD =                                   18,             /*!< bit string 16-bits. */
    EI_API_CIP_eEDT_DWORD =                                  19,             /*!< bit string 32-bits. */
    EI_API_CIP_eEDT_LWORD =                                  20,             /*!< bit string 64-bits. */
    EI_API_CIP_eEDT_STRING2 =                                21,             /*!< Character string (2 byte character count, 2 bytes per character). */
    EI_API_CIP_eEDT_FTIME =                                  22,             /*!< Duration (high resolution). */
    EI_API_CIP_eEDT_LTIME =                                  23,             /*!< Duration (long). */
    EI_API_CIP_eEDT_ITIME =                                  24,             /*!< Duration (short). */
    EI_API_CIP_eEDT_STRINGN =                                25,             /*!< Character string (2 byte character size (N), 2 byte character count, N bytes per character). */
    EI_API_CIP_eEDT_SHORTSTRING =                            26,             /*!< Character sting (1 byte character count, 1 byte per character). */
    EI_API_CIP_eEDT_TIME =                                   27,             /*!< Duration (milliseconds). */
    EI_API_CIP_eEDT_EPATH =                                  28,             /*!< CIP path segments. */
    EI_API_CIP_eEDT_ENGUNIT =                                29,             /*!< Engineering Units. */
    EI_API_CIP_eEDT_STRINGI =                                30,             /*!< International Character String. */
/// @cond INTERNAL
    EI_API_CIP_eEDT_FORCE32BIT =                             0xffffffff      /*!< Force enum to 32 bit. */
/// @endcond
} EI_API_CIP_EEdt_t;

/*!
 *  \brief Constructed Data Type Reporting
 *  \ingroup EI_API_CIP_ENUMERATIONS
 */
typedef enum EI_API_CIP_ECdt
{
    EI_API_CIP_eCDT_NO =                                     0x00,           /*!< No constructed data type. */
    EI_API_CIP_eCDT_ASTRUCTURE =                             0xA0,           /*!< Abbreviated Structure: Structure identified by 16-bit CRC value. */
    EI_API_CIP_eCDT_AARRAY =                                 0xA1,           /*!< Abbreviated Array: Array identified by 16-bit CRC value. */
    EI_API_CIP_eCDT_FSTRUCTURE =                             0xA2,           /*!< Formal Structure: Structure defined by member type list. */
    EI_API_CIP_eCDT_FARRAY =                                 0xA3,           /*!< Formal Array: Array defined by type and size. */
    EI_API_CIP_eCDT_FHSTRUCTURE =                            0xA8,           /*!< Formal Handle Structure: Structure defined by member type and member handle list. */
    EI_API_CIP_eCDT_DCDT =                                   0xB0,           /*!< Defined Constructed Data Types: Defined constructed data types. */
/// @cond INTERNAL
    EI_API_CIP_eCDT_FORCE32BIT =                             0xffffffff      /*!< Force enum to 32 bit. */
/// @endcond
} EI_API_CIP_ECdt_t;


/*!
 *  \brief General service parameter collection
 */
typedef struct EI_API_CIP_SService
{
    EI_API_CIP_ESc_t        code;                       /*!< The service code. */
    uint16_t                getAttrAllResponseCnt;      /*!< The count of the attribute IDs which are included on a Get_Attributes_All Response*/
    uint16_t*               getAttrAllResponse;         /*!< Pointer to an array of attributes IDs which are included on a Get_Attributes_All Response. */
    EI_API_CIP_CBService    callback;                   /*!< Pointer to the callback function. */
} EI_API_CIP_SService_t;


/*!
 *  \brief General attribute parameter collection
 */
typedef struct EI_API_CIP_SAttr
{
    uint16_t               id;                             /*!< The attribute identifier. */
    EI_API_CIP_EEdt_t      edt;                            /*!< Elementary data type. */
    EI_API_CIP_ECdt_t      cdt;                            /*!< Constructed data type. */
    EI_API_CIP_EAr_t       accessRule;                     /*!< Attribute access rules/flags. */
    uint16_t               edtSize;                        /*!< Elementary data type size for e.g. length for a string. */
    uint16_t               cdtSize;                        /*!< Constructed data type size for e.g. farray size, fstruct size. */
    uint16_t               index;                          /*!< The index of an array field. */
    void*                  pvValue;                        /*!< The data buffer. */
    EI_API_CIP_CBGetAttr   get_callback;                   /*!< Pointer to the get callback function. */
    EI_API_CIP_CBSetAttr   set_callback;                   /*!< Pointer to the set callback function. */
} EI_API_CIP_SAttr_t;


// CIP node functions
extern ETHIP_API T*       EI_API_CIP_NODE_new(void);
extern ETHIP_API uint32_t EI_API_CIP_NODE_delete(T* pCipNode_p);

// class functions
extern ETHIP_API uint32_t EI_API_CIP_createClass         (T *pCipNode_p, uint16_t classId_p);
extern ETHIP_API uint32_t EI_API_CIP_addClassService     (T *pCipNode_p, uint16_t classId_p, EI_API_CIP_SService_t *pService_p);
extern ETHIP_API uint32_t EI_API_CIP_setClassServiceFunc (T *pCipNode_p, uint16_t classId_p, EI_API_CIP_SService_t *pService_p);
extern ETHIP_API uint32_t EI_API_CIP_addClassAttr        (T *pCipNode_p, uint16_t classId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_setClassAttrFunc    (T *pCipNode_p, uint16_t classId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_getClassAttr        (T *pCipNode_p, uint16_t classId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_setClassAttr        (T *pCipNode_p, uint16_t classId_p, EI_API_CIP_SAttr_t    *pAttr_p);

// instance functions
extern ETHIP_API uint32_t EI_API_CIP_createInstance         (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p);
extern ETHIP_API uint32_t EI_API_CIP_addInstanceService     (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SService_t *pService_p);
extern ETHIP_API uint32_t EI_API_CIP_setInstanceServiceFunc (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SService_t *pService_p);
extern ETHIP_API uint32_t EI_API_CIP_addInstanceAttr        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_setInstanceAttrFunc    (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_cloneInstance          (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t               instanceIdSrc_p);
extern ETHIP_API uint32_t EI_API_CIP_getInstanceAttr        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_setInstanceAttr        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t    *pAttr_p);

// assembly functions
extern ETHIP_API uint32_t EI_API_CIP_createAssembly    (T *pCipNode_p, uint16_t assemblyInstanceId_p, EI_API_CIP_EAr_t accessRule_p);
extern ETHIP_API uint32_t EI_API_CIP_addAssemblyMember (T *pCipNode_p, uint16_t assemblyInstanceId_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attributeId_p);
extern ETHIP_API uint32_t EI_API_CIP_getAssemblySize   (T *pCipNode_p, uint16_t assemblyInstanceId_p, uint16_t* pSize_p);
extern ETHIP_API uint32_t EI_API_CIP_getAssemblyData   (T *pCipNode_p, uint16_t assemblyInstanceId_p, void *pDestinationBuffer_p, uint16_t destinationBufferLength_p);
extern ETHIP_API uint32_t EI_API_CIP_setAssemblyData   (T *pCipNode_p, uint16_t assemblyInstanceId_p, void *pSourceBuffer_p, uint16_t sourceBufferLength_p);
/*
// cyclic functions
uint32_t EI_API_CIP_makeAttributeCyclicInput(T *cipNode, uint16_t classId, uint16_t instanceId, uint16_t attributeId, uint16_t *pResOffset);
uint32_t EI_API_CIP_makeAttributeCyclicOutput(T *cipNode, uint16_t classId, uint16_t instanceId, uint16_t attributeId, uint16_t *pResOffset);
uint32_t EI_API_CIP_allocateCyclicBuffers(void);
uint8_t *EI_API_CIP_getCyclicInputProducerBuffer(void);
void EI_API_CIP_flipCyclicInputProducerBuffer(void);
uint8_t *EI_API_CIP_getCyclicOutputConsumerBuffer(void);
bool EI_API_CIP_flipOutputConsumerBuffer(uint8_t **ppBuffer);
uint16_t EI_API_CIP_getCyclicInputBufferSize(void);
uint16_t EI_API_CIP_getCyclicOutputBufferSize(void);
*/

// attribute functions (getter)
extern ETHIP_API uint32_t EI_API_CIP_getAttr_bool        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_bool         *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_sint        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_sint         *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_int         (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_int          *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_dint        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_dint         *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_lint        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_lint         *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_usint       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_usint        *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_uint        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_uint         *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_udint       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_udint        *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_ulint       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_ulint        *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_real        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_real         *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_lreal       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_lreal        *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_shortstring (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_shortstring **ppValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_byte        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_byte         *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_word        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_word         *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_dword       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_dword        *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_getAttr_lword       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_lword        *pValue_p);


// attribute functions (setters)
extern ETHIP_API uint32_t EI_API_CIP_setAttr_bool        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_bool          value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_sint        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_sint          value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_int         (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_int           value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_dint        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_dint          value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_lint        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_lint          value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_usint       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_usint         value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_uint        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_uint          value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_udint       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_udint         value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_ulint       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_ulint         value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_real        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_real          value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_lreal       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_lreal         value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_shortstring (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_shortstring  *pValue_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_byte        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_byte          value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_word        (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_word          value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_dword       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_dword         value_p);
extern ETHIP_API uint32_t EI_API_CIP_setAttr_lword       (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attrId_p, ei_api_cip_edt_lword         value_p);

#undef T

#ifdef  __cplusplus 
}
#endif 


#endif // EI_API_CIP_H_INC
