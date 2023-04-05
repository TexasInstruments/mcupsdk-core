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

#include <api/EI_API_CIP_define.h>

/// @cond INTERNAL
#define T EI_API_CIP_NODE_T
/// @endcond
/// @cond INTERNAL
typedef struct T T;
/// @endcond

#ifdef __cplusplus
extern "C" {
#endif

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
extern ETHIP_API uint32_t EI_API_CIP_createInstance              (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p);
extern ETHIP_API uint32_t EI_API_CIP_createRoutableInstance      (T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint8_t linkAddr_p);
extern ETHIP_API uint32_t EI_API_CIP_addInstanceService          (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SService_t *pService_p);
extern ETHIP_API uint32_t EI_API_CIP_addRoutableInstanceService  (T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SService_t* pService_p, uint8_t linkAddr_p);
extern ETHIP_API uint32_t EI_API_CIP_setInstanceServiceFunc      (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SService_t *pService_p);
extern ETHIP_API uint32_t EI_API_CIP_addInstanceAttr             (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_addRoutableInstanceAttr     (T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t* pAttr_p, uint8_t linkAddr_p);
extern ETHIP_API uint32_t EI_API_CIP_setInstanceAttrFunc         (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_setRoutableInstanceAttrFunc (T* pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t* pAttr_p, uint8_t linkAddr_p);
extern ETHIP_API uint32_t EI_API_CIP_cloneInstance               (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t               instanceIdSrc_p);
extern ETHIP_API uint32_t EI_API_CIP_getInstanceAttr             (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t    *pAttr_p);
extern ETHIP_API uint32_t EI_API_CIP_setInstanceAttr             (T *pCipNode_p, uint16_t classId_p, uint16_t instanceId_p, EI_API_CIP_SAttr_t    *pAttr_p);

//Connection Manager functions
extern ETHIP_API uint32_t EI_API_CIP_setRoutingMaxLinkAddr(T* pCipNode_p, uint8_t linkAddr_p);
extern ETHIP_API uint32_t EI_API_CIP_getRoutingMaxLinkAddr(T* pCipNode_p, uint8_t* plinkAddr_p);
extern ETHIP_API uint32_t EI_API_CIP_setRoutingRoutablePorts(T* pCipNode_p, const uint8_t*  pRoutablePortTable_p,uint8_t mapsize_p);


// assembly functions
extern ETHIP_API uint32_t EI_API_CIP_createAssembly    (T *pCipNode_p, uint16_t assemblyInstanceId_p, EI_API_CIP_EAr_t accessRule_p);
extern ETHIP_API uint32_t EI_API_CIP_addAssemblyMember (T *pCipNode_p, uint16_t assemblyInstanceId_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attributeId_p);
extern ETHIP_API uint32_t EI_API_CIP_getAssemblySize   (T *pCipNode_p, uint16_t assemblyInstanceId_p, uint16_t* pSize_p);
extern ETHIP_API uint32_t EI_API_CIP_getAssemblyData   (T *pCipNode_p, uint16_t assemblyInstanceId_p, void *pDestinationBuffer_p, uint16_t destinationBufferLength_p);
extern ETHIP_API uint32_t EI_API_CIP_setAssemblyData   (T *pCipNode_p, uint16_t assemblyInstanceId_p, void *pSourceBuffer_p, uint16_t sourceBufferLength_p);
extern ETHIP_API uint32_t EI_API_CIP_createCfgAssembly (T* pCipNode_p, uint16_t assemblyInstanceId_p, EI_API_CIP_CBCfgAssembly fuCfgAssemblyAccess_p);

extern ETHIP_API uint32_t EI_API_CIP_getRoutingAssemblySize   (T *pCipNode_p, uint16_t assemblyInstanceId_p, uint16_t* pSize_p, uint8_t linkAddr);
extern ETHIP_API uint32_t EI_API_CIP_getRoutingAssemblyData   (T *pCipNode_p, uint16_t assemblyInstanceId_p, void *pDestinationBuffer_p, uint16_t destinationBufferLength_p, uint8_t linkAddr);
extern ETHIP_API uint32_t EI_API_CIP_setRoutingAssemblyData   (T *pCipNode_p, uint16_t assemblyInstanceId_p, void *pSourceBuffer_p, uint16_t sourceBufferLength_p, uint8_t linkAddr);
extern ETHIP_API uint32_t EI_API_CIP_addRoutingAssemblyMember (T *pCipNode_p, uint16_t assemblyInstanceId_p, uint16_t classId_p, uint16_t instanceId_p, uint16_t attributeId_p, uint8_t linkAddr);
extern ETHIP_API uint32_t EI_API_CIP_createRoutingAssembly    (T *pCipNode_p, uint16_t assemblyInstanceId_p, EI_API_CIP_EAr_t accessRule_p, uint8_t linkAddr);
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

#ifdef  __cplusplus 
}
#endif 

#undef T

#endif // EI_API_CIP_H_INC
