/*!
 *  \example appClass71.c
 *
 *  \brief
 *  EtherNet/IP&trade; Adapter Example, create and handle user defined class 0x71.
 *  
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-06-09
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
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
 */


#include <stdio.h>
#include <string.h>
#include <api/EI_API.h>
#include <api/EI_API_def.h>

typedef struct
{
    uint16_t revision;
    uint16_t maxInstance;
    uint16_t noInstance;
} EI_APP_CLASS71_ClassData;

typedef struct
{
    bool run_state;
    uint32_t counter;
    uint16_t increment;
    char counter_text[32];
} EI_APP_CLASS71_InstanceData;

// Global variables and pointers used in this example.
static EI_API_CIP_NODE_T* pEI_API_CIP_NODE_g = NULL;
EI_APP_CLASS71_ClassData appClass71Data_g = {1, 1, 1};
EI_APP_CLASS71_InstanceData appClass71InstanceData_g = {true, 0, 1, "unitialized"};
const uint16_t classId_g = 0x71;
const uint16_t instanceId1_g = 1;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Add an attribute for the class instance
 *
 *  \details
 *  This function adds an attribute for the class instance
 */
static uint32_t EI_APP_CLASS71_addClassAttribute(uint16_t attrId, void* pvValue)
{
    uint32_t errcode;
    EI_API_CIP_SAttr_t attr;
    OSAL_MEMORY_memset(&attr, 0, sizeof(attr));
    attr.id = attrId;
    attr.edt = EI_API_CIP_eEDT_UINT;
    attr.accessRule = EI_API_CIP_eAR_GET;
    attr.pvValue = &pvValue;
    errcode = EI_API_CIP_addClassAttr(pEI_API_CIP_NODE_g, classId_g, &attr);
    if(errcode != EI_API_CIP_eERR_OK)
        return errcode;
    errcode = EI_API_CIP_setClassAttr(pEI_API_CIP_NODE_g, classId_g, &attr);
    if (errcode != EI_API_CIP_eERR_OK)
        return errcode;
    return EI_API_CIP_eERR_OK;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Add an attribute for the instance 1
 *
 *  \details
 *  This function adds an attribute with the data type, the access rule, callback
 *  functions for set and get attribute service and the initial value
 */
static uint32_t EI_APP_CLASS71_addInstance1Attribute(
                                    uint16_t attrId,
                                    EI_API_CIP_EEdt_t edt, EI_API_CIP_EAr_t accessRule,
                                    EI_API_CIP_CBGetAttr getCb, EI_API_CIP_CBSetAttr setCb,
                                    uint16_t len, void* pvValue)
{
    uint32_t errcode;

    EI_API_CIP_SAttr_t attr;
    OSAL_MEMORY_memset(&attr, 0, sizeof(attr));
    attr.id = attrId;
    attr.edt = edt;
    attr.edtSize = len;
    attr.accessRule = accessRule;
    attr.get_callback = getCb;
    attr.set_callback = setCb;
    attr.pvValue = pvValue;

    // add attribute
    errcode = EI_API_CIP_addInstanceAttr(pEI_API_CIP_NODE_g, classId_g, instanceId1_g, &attr);
    if (errcode != EI_API_CIP_eERR_OK)
    {
        return errcode;
    }

    // set initial value
    errcode = EI_API_CIP_setInstanceAttr(pEI_API_CIP_NODE_g, classId_g, instanceId1_g, &attr);
    if (errcode != EI_API_CIP_eERR_OK)
    {
        return errcode;
    }

    // set callback function
    if (attr.get_callback != NULL || attr.set_callback != NULL)
    {
        errcode = EI_API_CIP_setInstanceAttrFunc(pEI_API_CIP_NODE_g, classId_g, instanceId1_g, &attr);
        if (errcode != EI_API_CIP_eERR_OK)
        {
            return errcode;
        }
    }

    return EI_API_CIP_eERR_OK;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback function for the set service of the run state
 *
 *  \details
 *  This function will be called, when the set attribute single service is
 *  called for the run state
 */
uint32_t EI_APP_CLASS71_SetRunStateCb(EI_API_CIP_NODE_T* cipNode, uint16_t classId, uint16_t instanceId, uint16_t attrId, uint16_t len, void* pvValue)
{
    if (len == sizeof(bool))
    {
        if (*(uint8_t*)pvValue <= 1)
        {
            appClass71InstanceData_g.run_state = *(bool*)pvValue;
            return EI_API_eERR_CB_NO_ERROR;
        }
        else
        {
            return EI_API_eERR_CB_INVALID_VALUE;
        }
    }
    else
    {
        return len < sizeof(bool) ? EI_API_eERR_CB_NOT_ENOUGH_DATA : EI_API_eERR_CB_TOO_MUCH_DATA;
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback function for the get service of the run state
 *
 *  \details
 *  This function will be called, when the get attribute single service is
 *  called for the run state
 */
uint32_t EI_APP_CLASS71_GetRunStateCb(EI_API_CIP_NODE_T* cipNode, uint16_t classId, uint16_t instanceId, uint16_t attrId, uint16_t* len, void* pvValue)
{
    *len = sizeof(bool);
    *(bool*)pvValue = appClass71InstanceData_g.run_state;
    return EI_API_eERR_CB_NO_ERROR;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback function for the set service of the counter value
 *
 *  \details
 *  This function will be called, when the set attribute single service is
 *  called for the counter value. The only valid value is 0, which resets the
 *  counter value to zero.
 */
uint32_t EI_APP_CLASS71_SetCounterCb(EI_API_CIP_NODE_T* cipNode, uint16_t classId, uint16_t instanceId, uint16_t attrId, uint16_t len, void* pvValue)
{
    if (len == sizeof(uint32_t))
    {
        if (*(uint16_t*)pvValue == 0)
        {
            appClass71InstanceData_g.counter = 0;
            return EI_API_eERR_CB_NO_ERROR;
        }
        else
        {
            return EI_API_eERR_CB_VAL_TOO_HIGH;
        }
    }
    else
    {
        return len < sizeof(uint16_t) ? EI_API_eERR_CB_NOT_ENOUGH_DATA : EI_API_eERR_CB_TOO_MUCH_DATA;
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback function for the get service of the counter value
 *
 *  \details
 *  This function will be called, when the get attribute single service is
 *  called for the counter value
 */
uint32_t EI_APP_CLASS71_GetCounterCb(EI_API_CIP_NODE_T* cipNode, uint16_t classId, uint16_t instanceId, uint16_t attrId, uint16_t* len, void* pvValue)
{
    *len = sizeof(uint32_t);
    *(uint32_t*)pvValue = appClass71InstanceData_g.counter;
    return EI_API_eERR_CB_NO_ERROR;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback function for the set service of the increment value
 *
 *  \details
 *  This function will be called, when the set attribute single service is
 *  called for the increment value
 */
uint32_t EI_APP_CLASS71_SetIncrementCb(EI_API_CIP_NODE_T* cipNode, uint16_t classId, uint16_t instanceId, uint16_t attrId, uint16_t len, void* pvValue)
{
    if (len == sizeof(uint16_t))
    {
        if (*(uint16_t*)pvValue <= 1000)
        {
            appClass71InstanceData_g.increment = *(uint16_t*)pvValue;
            return EI_API_eERR_CB_NO_ERROR;
        }
        else
        {
            return EI_API_eERR_CB_VAL_TOO_HIGH;
        }
    }
    else
    {
        return len < sizeof(uint16_t) ? EI_API_eERR_CB_NOT_ENOUGH_DATA : EI_API_eERR_CB_TOO_MUCH_DATA;
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback function for the get service of the increment value
 *
 *  \details
 *  This function will be called, when the get attribute single service is
 *  called for the increment value
 */
uint32_t EI_APP_CLASS71_GetIncrementCb(EI_API_CIP_NODE_T* cipNode, uint16_t classId, uint16_t instanceId, uint16_t attrId, uint16_t* len, void* pvValue)
{
    *len = sizeof(uint16_t);
    *(uint16_t*)pvValue = appClass71InstanceData_g.increment;
    return EI_API_eERR_CB_NO_ERROR;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback function for the get service of the counter text
 *
 *  \details
 *  This function will be called, when the get attribute single service is
 *  called for the counter text
 */
uint32_t EI_APP_CLASS71_GetCounterTextCb(EI_API_CIP_NODE_T* cipNode, uint16_t classId, uint16_t instanceId, uint16_t attrId, uint16_t* len, void* pvValue)
{
    *len = strlen(appClass71InstanceData_g.counter_text);
    strcpy((char*)pvValue, appClass71InstanceData_g.counter_text);
    return EI_API_eERR_CB_NO_ERROR;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initialize class 71 data
 *
 *  \details
 *  This function creates the class 71 with class instance and instance 1.
 *  The class instance contains attributes for the revision, the maximum
 *..number of instances and the actual number of instances.
 *  The instance 1 contains attributes for the run state, the counter value,
 *  the increment value and the counter text representation.
 */
void EI_APP_CLASS71_init(EI_API_CIP_NODE_T* pEI_API_CIP_NODE_p)
{
    uint32_t errCode;
    EI_API_CIP_SService_t service;

    pEI_API_CIP_NODE_g = pEI_API_CIP_NODE_p;

    errCode = EI_API_CIP_createClass(pEI_API_CIP_NODE_g, classId_g);

    // set class instance
    OSAL_MEMORY_memset(&service, 0, sizeof(service));
    service.code = EI_API_CIP_eSC_GETATTRSINGLE;
    errCode = EI_API_CIP_addClassService(pEI_API_CIP_NODE_g, classId_g, &service);
    errCode = EI_APP_CLASS71_addClassAttribute(1, &appClass71Data_g.revision);
    errCode = EI_APP_CLASS71_addClassAttribute(2, &appClass71Data_g.maxInstance);
    errCode = EI_APP_CLASS71_addClassAttribute(3, &appClass71Data_g.noInstance);

    // Create instance 1
    errCode = EI_API_CIP_createInstance(pEI_API_CIP_NODE_g, classId_g, instanceId1_g);

    // Add get and set service for instance 1
    service.code = EI_API_CIP_eSC_GETATTRSINGLE;
    errCode = EI_API_CIP_addInstanceService(pEI_API_CIP_NODE_g, classId_g, instanceId1_g, &service);
    service.code = EI_API_CIP_eSC_SETATTRSINGLE;
    errCode = EI_API_CIP_addInstanceService(pEI_API_CIP_NODE_g, classId_g, instanceId1_g, &service);

    // Add attributes for instance 1
    errCode = EI_APP_CLASS71_addInstance1Attribute(
                1, EI_API_CIP_eEDT_BOOL, EI_API_CIP_eAR_GET_AND_SET,
                NULL, EI_APP_CLASS71_SetRunStateCb, 
                sizeof(appClass71InstanceData_g.run_state), &appClass71InstanceData_g.run_state);

    errCode = EI_APP_CLASS71_addInstance1Attribute(
                2, EI_API_CIP_eEDT_UDINT, EI_API_CIP_eAR_GET_AND_SET,
                EI_APP_CLASS71_GetCounterCb, EI_APP_CLASS71_SetCounterCb,
                sizeof(appClass71InstanceData_g.counter), &appClass71InstanceData_g.counter);

    errCode = EI_APP_CLASS71_addInstance1Attribute(
                3, EI_API_CIP_eEDT_UINT, EI_API_CIP_eAR_GET_AND_SET,
                EI_APP_CLASS71_GetIncrementCb, EI_APP_CLASS71_SetIncrementCb,
                sizeof(appClass71InstanceData_g.increment), &appClass71InstanceData_g.increment);

    errCode = EI_APP_CLASS71_addInstance1Attribute(
                4, EI_API_CIP_eEDT_SHORTSTRING, EI_API_CIP_eAR_GET,
                EI_APP_CLASS71_GetCounterTextCb, NULL,
                sizeof(appClass71InstanceData_g.counter_text), &appClass71InstanceData_g.counter_text);

    (void)errCode;
    return;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Run function for class 71
 *
 *  \details
 *  This function must be called cyclically. It updates the counter value
 *  and the counter text, when the run_state variable is true.
 */
void EI_APP_CLASS71_run(void)
{
    if (appClass71InstanceData_g.run_state)
    {
        appClass71InstanceData_g.counter += appClass71InstanceData_g.increment;
        //sprintf(appClass71InstanceData_g.counter_text, "counter is at %d (hex %08x)", appClass71InstanceData_g.counter, appClass71InstanceData_g.counter);
    }
}

