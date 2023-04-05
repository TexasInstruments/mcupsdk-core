/*!
 *  \file ecSlvSimple.c
 *
 *  \brief
 *  EtherCAT<sup>&reg;</sup> Slave Example Application
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-05-18
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
 *
 */

#define SHOW_LOOPCOUNT  0
#define SHOW_ESCSTATUS  0

#define ENABLE_I2CLEDS  0

#define BIT2BYTE(x)                     (((x)+7) >> 3)

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/

#include <ecSlvSimple.h>
#include <ecSlvApiDef.h>
#include "project.h"
#include "ecSlvSimple.h"

#include <ESL_os.h>
#include <ESL_BOARD_config.h>
#include <ESL_vendor.h>

#include <ESL_gpioHelper.h>
#include <ESL_foeDemo.h>
#include <ESL_soeDemo.h>
#include <ESL_eeprom.h>
#include <ESL_version.h>

#include <ecSlvApi.h>

#if !(defined MBXMEM)
#define MBXMEM
#endif

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/
#define I2C_IOEXP_ADDR 0x60                  // The I2C address for GPIO expander

static void EC_SLV_APP_applicationRun(void* pAppCtxt_p);

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This is a callback function which sends and EoE frame to the Master.

*////////////////////////////////////////////////////////////////////////////////////////

uint8_t EC_SLV_APP_sendEoEFrame(EC_SLV_APP_Sapplication_t* pApplicationInstance_p, uint16_t index_p
                               ,uint8_t subindex_p, uint32_t size_p
                               ,uint16_t MBXMEM * pData_p, uint8_t completeAccess_p)
{
    EC_API_EError_t     error   = EC_API_eERR_INVALID;

    if (!pApplicationInstance_p)
    {
        goto Exit;
    }

    OSALUNREF_PARM(index_p);
    OSALUNREF_PARM(subindex_p);
    OSALUNREF_PARM(completeAccess_p);

    error = (EC_API_EError_t)EC_API_SLV_EoE_sendFrame(pApplicationInstance_p->ptEcSlvApi
                                                     ,(uint16_t)size_p, (uint8_t *) pData_p);

Exit:
    return error;
}

uint8_t EC_SLV_APP_getValueFromMaster(void* pApplicationCtxt_p
                                       ,uint16_t index_p, uint8_t subindex_p, uint32_t size_p
                                       ,uint16_t MBXMEM * pData_p, uint8_t completeAccess_p)
{
    EC_API_EError_t     error   = (EC_API_EError_t)ABORT_NOERROR;

    OSAL_printf("%s ==> Idx: 0x%04x:%d | Size: %d | Value: %d\r\n", __func__
               ,index_p, subindex_p, size_p, pData_p[0]);

    if (pApplicationCtxt_p)
    {
        goto Exit;
    }

    OSALUNREF_PARM(pApplicationCtxt_p);
    OSALUNREF_PARM(completeAccess_p);

Exit:
    return (uint8_t)error;
}

uint8_t EC_SLV_APP_setValueToMaster(void* pApplicationCtxt_p, uint16_t index_p
                                     ,uint8_t subindex_p, uint32_t size_p
                                     ,uint16_t MBXMEM* pData_p, uint8_t completeAccess_p)
{
    EC_API_EError_t     error   = (EC_API_EError_t)ABORT_NOERROR;

    if (pApplicationCtxt_p)
    {
        goto Exit;
    }

    OSAL_printf("%s ==> Idx: 0x%04x\r\n", __func__, index_p);

    OSALUNREF_PARM(pApplicationCtxt_p);
    OSALUNREF_PARM(subindex_p);
    OSALUNREF_PARM(completeAccess_p);
    OSALUNREF_PARM(size_p);
    OSALUNREF_PARM(pData_p);

Exit:
    return (uint8_t)error;
}

uint8_t EC_SLV_APP_sendEmergencyMsg(EC_SLV_APP_Sapplication_t* pApplicationInstance_p, uint16_t index_p
                                   ,uint8_t subindex_p, uint32_t size_p, uint16_t MBXMEM* pData_p
                                   ,uint8_t completeAccess_p)
{
    EC_API_EError_t     error   = EC_API_eERR_INVALID;

    if (pApplicationInstance_p)
    {
        goto Exit;
    }

    OSALUNREF_PARM(pApplicationInstance_p);
    OSALUNREF_PARM(index_p);
    OSALUNREF_PARM(subindex_p);
    OSALUNREF_PARM(completeAccess_p);

    error = (EC_API_EError_t)EC_API_SLV_writeEmergency(0xAFFE, size_p, (uint8_t*) pData_p);

Exit:
    return error;
}

static EC_API_EError_t EC_SLV_APP_populateSlaveInfo(EC_SLV_APP_Sapplication_t* pApplicationInstance_p)
{
    EC_API_EError_t         error   = EC_API_eERR_INVALID;
    EC_API_SLV_SHandle_t*   ptSlave;

    if (!pApplicationInstance_p)
    {
        goto Exit;
    }
    ptSlave = pApplicationInstance_p->ptEcSlvApi;

    error = (EC_API_EError_t)EC_API_SLV_setVendorId      (ptSlave, ECAT_VENDORID);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setRevisionNumber(ptSlave, EC_REVISION);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setSerialNumber  (ptSlave, 0x00000000);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setProductCode   (ptSlave, ECAT_PRODUCTCODE_CTT);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setProductName   (ptSlave, ECAT_PRODUCTNAME_CTT);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setGroupType     (ptSlave, "EtherCAT Toolkit");
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setHwVersion     (ptSlave, "R01");
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = ESL_setSWVersion(ptSlave);
    if (error != EC_API_eERR_NONE)
    {
        goto Exit;
    }

    /* Former Project.h */
    error = (EC_API_EError_t)EC_API_SLV_setPDOSize(ptSlave, EC_MAX_PD_LEN, EC_MAX_PD_LEN);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setPDICfg(ptSlave, ESC_EE_PDI_CONTROL, ESC_EE_PDI_CONFIG);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setBootStrapMailbox(ptSlave,
                                                            EC_BOOTSTRAP_MBXOUT_START, EC_BOOTSTRAP_MBXOUT_DEF_LENGTH,
                                                            EC_BOOTSTRAP_MBXIN_START, EC_BOOTSTRAP_MBXIN_DEF_LENGTH);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setStandardMailbox(ptSlave,
                                                           EC_MBXOUT_START, EC_MBXOUT_DEF_LENGTH,
                                                           EC_MBXIN_START, EC_MBXIN_DEF_LENGTH);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setSyncManConfig(ptSlave,
                                                         0, EC_MBXOUT_START, EC_MBXOUT_DEF_LENGTH,
                                                         EC_MBXOUT_CONTROLREG, EC_MBXOUT_ENABLE);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setSyncManConfig(ptSlave,
                                                         1, EC_MBXIN_START, EC_MBXIN_DEF_LENGTH,
                                                         EC_MBXIN_CONTROLREG, EC_MBXIN_ENABLE);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setSyncManConfig(ptSlave,
                                                         2, EC_OUTPUT_START, EC_OUTPUT_DEF_LENGTH,
                                                         EC_OUTPUT_CONTROLREG, EC_OUTPUT_ENABLE);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setSyncManConfig(ptSlave,
                                                         3, EC_INPUT_START, EC_INPUT_DEF_LENGTH,
                                                         EC_INPUT_CONTROLREG, EC_INPUT_ENABLE);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* /Former Project.h */

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

static EC_API_EError_t EC_SLV_APP_populateOutObjects(EC_SLV_APP_Sapplication_t* pApplicationInstance)
{
    EC_API_EError_t         error   = EC_API_eERR_INVALID;
    EC_API_SLV_SHandle_t*   ptSlave;

    if (!pApplicationInstance)
    {
        goto Exit;
    }

    ptSlave = pApplicationInstance->ptEcSlvApi;

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddRecord(ptSlave, 0x2000, "Out Object Record"
                                                       ,NULL, NULL, NULL, NULL, &pApplicationInstance->ptRecObjOut);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2000 Record Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->ptRecObjOut
                                                                ,1, "SubIndex 1", DEFTYPE_UNSIGNED32, 32
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2000 SubIndex 1 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->ptRecObjOut
                                                                ,2, "i2c-leds", DEFTYPE_UNSIGNED8, 8
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2000 SubIndex 2 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->ptRecObjOut
                                                                ,3, "SubIndex 3", DEFTYPE_UNSIGNED8, 8
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2000 SubIndex 3 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->ptRecObjOut
                                                                ,4, "SubIndex 4", DEFTYPE_UNSIGNED16, 16
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2000 SubIndex 4 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddArray(ptSlave, 0x3000, "RxPDO Mapping", 32, DEFTYPE_UNSIGNED32, 32,
                                                       ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x3000 Record Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

static EC_API_EError_t EC_SLV_APP_populateInOutObjects(EC_SLV_APP_Sapplication_t* pApplicationInstance)
{
    EC_API_EError_t         error   = EC_API_eERR_INVALID;
    EC_API_SLV_SHandle_t*   ptSlave;

    if (!pApplicationInstance)
    {
        goto Exit;
    }

    ptSlave = pApplicationInstance->ptEcSlvApi;

    //////////////////////////////////////////
    // Create a Object Variable for test purposes
    //////////////////////////////////////////

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2001, "Test Variable"
                                                         ,DEFTYPE_UNSIGNED32, 32
                                                         ,ACCESS_READWRITE, NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2001 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    ////////////////////////////////////////
    // Create a Object Record for test purposes
    ////////////////////////////////////////

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddRecord(ptSlave, 0x2002, "Test Record"
                                                       ,NULL, NULL, NULL, NULL
                                                       ,&pApplicationInstance->pt2002RecObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object Record Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt2002RecObj
                                                                ,1, "SubIndex 1", DEFTYPE_UNSIGNED32, 32
                                                                ,ACCESS_READ | OBJACCESS_TXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2002 SubIndex 1 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt2002RecObj
                                                                ,2, "SubIndex 2", DEFTYPE_UNSIGNED8, 8
                                                                ,ACCESS_READ | OBJACCESS_TXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2002 SubIndex 2 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt2002RecObj
                                                                ,3, "SubIndex 3", DEFTYPE_UNSIGNED8, 8
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_TXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2002 SubIndex 3 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt2002RecObj
                                                                ,4, "SubIndex 4", DEFTYPE_UNSIGNED16, 16
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_TXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2002 SubIndex 4 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt2002RecObj
                                                                ,5, "SubIndex 5", DEFTYPE_UNSIGNED8, 8
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING | OBJACCESS_TXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2002 SubIndex 3 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    //Create an Object and provide a function to send an emergency message when the object is written
    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x200C, "Emergency message", DEFTYPE_UNSIGNED32, 32,
                                                          ACCESS_READWRITE, NULL, NULL, NULL, NULL); //&sendEmergencyMsg
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200C Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    //Send an EoE message when the Object is written
    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x200E, "EoE Send Frame", DEFTYPE_UNSIGNED32, 32,
                                                          ACCESS_READWRITE, NULL, NULL, NULL, NULL); //&sendEoEFrame
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200E Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    //Create a second record object with unaligned indexes
    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddRecord(ptSlave, 0x200F, "Test Record II"
                                                       ,NULL, NULL, NULL, NULL, &pApplicationInstance->pt200FRecObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt200FRecObj
                                                                ,1, "SubIndex 1", DEFTYPE_UNSIGNED16, 16
                                                                ,ACCESS_READ | OBJACCESS_TXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F SubIndex 1 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt200FRecObj
                                                                ,2, "SubIndex 2", DEFTYPE_UNSIGNED16, 16
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F SubIndex 2 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt200FRecObj
                                                                ,3, "SubIndex 3", DEFTYPE_UNSIGNED8, 8
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F SubIndex 3 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt200FRecObj
                                                                ,4, "SubIndex 4", DEFTYPE_UNSIGNED8, 8
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F SubIndex 4 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt200FRecObj
                                                                ,5, "SubIndex 5", DEFTYPE_UNSIGNED32, 32
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F SubIndex 5 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt200FRecObj
                                                                ,6, "SubIndex 6", DEFTYPE_UNSIGNED8, 8
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F SubIndex 6 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt200FRecObj
                                                                ,7, "SubIndex 7", DEFTYPE_UNSIGNED16, 16
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F SubIndex 7 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance->pt200FRecObj
                                                                ,8, "SubIndex 8", DEFTYPE_UNSIGNED16, 16
                                                                ,ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x200F SubIndex 8 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2015, "FoE File Downloaded"
                                                         ,DEFTYPE_BOOLEAN, 1, ACCESS_READ, NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2015 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2016, "Write Variable", DEFTYPE_UNSIGNED32, 32
                                                         ,ACCESS_READWRITE, NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2001 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2017, "Incr. by read", DEFTYPE_UNSIGNED32, 32
                                                         ,ACCESS_READ, NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2001 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddArray(ptSlave, 0x4000, "TxPDO Mapping", 32, DEFTYPE_UNSIGNED32, 32,
                                                       ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x4000 Record Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

static EC_API_EError_t EC_SLV_APP_populateDescriptionObjects(EC_SLV_APP_Sapplication_t* pApplicationInstance_p)
{
    EC_API_EError_t         error   = EC_API_eERR_INVALID;
    EC_API_SLV_SHandle_t*   ptSlave;

    if (!pApplicationInstance_p)
    {
        goto Exit;
    }

    ptSlave = pApplicationInstance_p->ptEcSlvApi;

    /* descriptions */
    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddArray(ptSlave, 0x2005, "Process Data Info"
                                                      ,2, DEFTYPE_UNSIGNED16, 16
                                                      ,ACCESS_READ, NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object Array Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddArray(ptSlave, 0x2006, "PDO Info", 8, DEFTYPE_UNSIGNED16, 16
                                                      ,ACCESS_READ, NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object Array Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    ////////////////////////////////////////
    // Create a Object Record for test purposes
    ////////////////////////////////////////

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddRecord(ptSlave, 0x2007, "Process Data Info Record"
                                                       ,NULL, NULL, NULL, NULL
                                                       ,&pApplicationInstance_p->pt2007RecObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,1, "Input process data length"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,2, "Output process data length"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,3, "TxPdo offset"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,4, "TxPdo bitsize"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,5, "TxPdo2 offset"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,6, "TxPdo2 bitsize"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,7, "RxPdo offset"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,8, "RxPdo bitsize"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,9, "RxPdo2 offset"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_configRecordSubIndex(ptSlave, pApplicationInstance_p->pt2007RecObj
                                                                ,10, "RxPdo2 bitsize"
                                                                ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }


    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2008, "Input Process Data Length"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2009, "Output Process Data Length"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }
    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x200A, "TxPdo Offset"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x200B, "TxPdo bit size"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x200D, "TxPdo2 Offset"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2010, "TxPdo2 bit size"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2011, "RxPdo Offset"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2012, "RxPdo bit size"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2013, "RxPdo2 Offset"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2014, "RxPdo2 bit size"
                                                         ,DEFTYPE_UNSIGNED16, 16, ACCESS_READ
                                                         ,NULL, NULL, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

static EC_API_EError_t EC_SLV_APP_populateDescriptionObjectValues(EC_SLV_APP_Sapplication_t* pApplicationInstance_p)
{
    EC_API_SLV_SHandle_t*           ptSlave;
    EC_API_EError_t                 error       = EC_API_eERR_INVALID;
    EC_API_SLV_SCoE_ObjEntry_t*     ptObjEntry;
    EC_API_SLV_SCoE_Object_t*       ptCoEObj;
    uint32_t                        value;

    if (!pApplicationInstance_p)
    {
        goto Exit;
    }

    ptSlave = pApplicationInstance_p->ptEcSlvApi;

    /* Slave Input Len */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2005, 1, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_getInputProcDataLength(ptSlave);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* Slave Output Len */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2005, 2, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_getOutputProcDataLength(ptSlave);
    OSAL_printf("%s:%d PDO Out Len: 0x%lx\r\n", __func__, __LINE__, value);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2006, 1, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptTxPdo1A00);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO Length */
    error =  (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2006, 2, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptTxPdo1A00);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO2 Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2006, 3, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptTxPdo1A01);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO2 Length */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2006, 4, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptTxPdo1A01);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2006, 5, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptRxPdo1600);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO Length */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2006, 6, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptRxPdo1600);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO2 Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2006, 7, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptRxPdo1601);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO2 Length */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2006, 8, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptRxPdo1601);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* Slave Input Len 2 */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 1, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_getInputProcDataLength(ptSlave);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* Slave Output Len 2 */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 2, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_getOutputProcDataLength(ptSlave);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 3, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptTxPdo1A00);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO Length */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 4, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptTxPdo1A00);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO2 Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 5, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptTxPdo1A01);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO2 Length */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 6, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptTxPdo1A01);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 7, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptRxPdo1601);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO Length */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 8, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptRxPdo1601);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO2 Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 9, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptRxPdo1601);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO2 Length */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2007, 10, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptRxPdo1601);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectEntryData(ptSlave, ptObjEntry, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* Slave Input Len 3 */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x2008, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_getInputProcDataLength(ptSlave);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* Slave Output Len 3 */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x2009, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_getOutputProcDataLength(ptSlave);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x200A, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptTxPdo1A00);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO Len */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x200B, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptTxPdo1A00);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO Offset 2*/
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x200D, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptTxPdo1A01);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* TxPDO Len 2 */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x2010, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptTxPdo1A01);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO Offset */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x2011, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptRxPdo1600);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO Len */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x2012, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptRxPdo1600);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO Offset 2*/
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x2013, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getOffset(pApplicationInstance_p->ptRxPdo1601);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    /* RxPDO Len 2 */
    error = (EC_API_EError_t)EC_API_SLV_CoE_getObject(ptSlave, 0x2014, &ptCoEObj);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    value = EC_API_SLV_PDO_getLength(pApplicationInstance_p->ptRxPdo1601);
    error = (EC_API_EError_t)EC_API_SLV_CoE_setObjectData(ptSlave, ptCoEObj, 2, (uint16_t*)&value);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

/* Rx from Slave View: ProcessData sent from master, received by slave */
static EC_API_EError_t EC_SLV_APP_populateRxPDO(EC_SLV_APP_Sapplication_t* pApplicationInstance_p)
{
    EC_API_SLV_SHandle_t*       ptSlave;
    EC_API_EError_t             error       = EC_API_eERR_INVALID;
    EC_API_SLV_SCoE_ObjEntry_t* ptObjEntry;
    uint8_t             	subIdx = 1;
    char                	scPdoEntryName[16];

    if (!pApplicationInstance_p)
    {
        goto Exit;
    }

    ptSlave = pApplicationInstance_p->ptEcSlvApi;

    error = (EC_API_EError_t)EC_API_SLV_PDO_create(ptSlave, "RxPDO", 0x1600, &pApplicationInstance_p->ptRxPdo1600);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Create PDO 0x1600 Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }
    OSAL_printf("RxPDO created 0x1600: 0x%lx\r\n", (uint32_t)pApplicationInstance_p->ptRxPdo1600);
    do
    {
        error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x3000, subIdx, &ptObjEntry);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        sprintf(scPdoEntryName, "SubIndex %d", subIdx);
        error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptRxPdo1600, scPdoEntryName, ptObjEntry);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        subIdx++;
    }  while (subIdx <= 32);

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

/* Tx from Slave View: ProcessData sent from slave, received by master */
static EC_API_EError_t EC_SLV_APP_populateTxPDO(EC_SLV_APP_Sapplication_t* pApplicationInstannce_p)
{
    EC_API_SLV_SHandle_t*           ptSlave;
    EC_API_EError_t                 error           = EC_API_eERR_INVALID;
    EC_API_SLV_SCoE_ObjEntry_t*     ptObjEntry;
    uint8_t                         subIdx          = 1;
    char                            aPdoEntryName[16];

    if (!pApplicationInstannce_p)
    {
        goto Exit;
    }

    ptSlave = pApplicationInstannce_p->ptEcSlvApi;

    error = (EC_API_EError_t)EC_API_SLV_PDO_create(ptSlave, "TxPDO", 0x1A00, &pApplicationInstannce_p->ptTxPdo1A00);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create PDO 0x1A00 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    OSAL_printf("TxPDO created 0x1A00: 0x%lx\r\n", (uint32_t)pApplicationInstannce_p->ptTxPdo1A00);

    do
    {
        error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x4000, subIdx, &ptObjEntry);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        sprintf(aPdoEntryName, "SubIndex %d", subIdx);
        error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstannce_p->ptTxPdo1A00, aPdoEntryName, ptObjEntry);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        subIdx++;
    }  while (subIdx <= 32);

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE)
static OSAL_FUNC_UNUSED void EC_SLV_APP_boardPhyReset(void* pCtxt_p, uint8_t phyIdx_p, bool reset_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstance    = (EC_SLV_APP_Sapplication_t*)pCtxt_p;

    ESL_BOARD_OS_phyReset(pApplicationInstance->gpioHandle, pApplicationInstance->selectedPruInstance, phyIdx_p, reset_p);
}
#endif

static void EC_SLV_APP_appBoardStatusLed(void* pCallContext_p, void* pLedContext_p, bool runLed_p, bool errLed_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstance    = (EC_SLV_APP_Sapplication_t*)pCallContext_p;

    OSALUNREF_PARM(pLedContext_p);

    if (NULL == pApplicationInstance)
    {
        goto Exit;
    }

    ESL_BOARD_OS_statusLED(pApplicationInstance->gpioHandle, pApplicationInstance->selectedPruInstance, runLed_p, errLed_p);
Exit:
    return;
}

static EC_API_EError_t EC_SLV_APP_populateBoardFunctions(EC_SLV_APP_Sapplication_t* pApplicationInstance)
{
    EC_API_EError_t         error   = EC_API_eERR_INVALID;

    if (!pApplicationInstance)
    {
        goto Exit;
    }

    EC_API_SLV_cbRegisterBoardStatusLed(pApplicationInstance->ptEcSlvApi, pApplicationInstance
                                       ,EC_SLV_APP_appBoardStatusLed
                                       ,&pApplicationInstance->selectedPruInstance);

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

void EC_SLV_APP_initBoardFunctions(EC_SLV_APP_Sapplication_t *pAppInstance_p)
{
    if (!pAppInstance_p)
    {
        goto Exit;
    }

    /* open gpio instance */
    pAppInstance_p->gpioHandle = ESL_GPIO_init();

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE)
    /* Configure manual MDIO mode workaround if configured in SysConfig or similar
     * This is to enable the TI workaround for errata i2329. The activation is
     * detected and provisioned in this call.
     */
    ESL_OS_manualMdioConfig(pAppInstance_p->ptEcSlvApi);

    /* configure Phy Reset Pin */
    ESL_BOARD_OS_configureResets(pAppInstance_p->gpioHandle, pAppInstance_p->selectedPruInstance);
#else
    OSALUNREF_PARM(pAppInstance_p);
#endif

    /* configure LED Pin */
    ESL_BOARD_OS_initStatusLED(pAppInstance_p->gpioHandle, pAppInstance_p->selectedPruInstance);

    ESL_GPIO_apply(pAppInstance_p->gpioHandle);

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE)
    /* set Phy Reset Pin */
    EC_SLV_APP_boardPhyReset(pAppInstance_p, EC_API_SLV_ePHY_IN, true);
    EC_SLV_APP_boardPhyReset(pAppInstance_p, EC_API_SLV_ePHY_OUT, true);
#endif
Exit:
    return;
}

void EC_SLV_APP_registerStacklessBoardFunctions(EC_SLV_APP_Sapplication_t *pAppInstance_p)
{
    if (!pAppInstance_p)
    {
        goto Exit;
    }

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE)
    ESL_BOARD_OS_registerPhys(pAppInstance_p->selectedPruInstance);

    EC_API_SLV_cbRegisterPhyReset(EC_SLV_APP_boardPhyReset, pAppInstance_p);
#endif

Exit:
    return;
}

void OSAL_FUNC_NORETURN EC_SLV_APP_usrAppRunWrapper(void* pArg_p)
{
    EC_SLV_APP_Sapplication_t* pAppInstance = (EC_SLV_APP_Sapplication_t*)pArg_p;

    for(;;)
    {
        //OSAL_SCHED_sleep(100); /* this has to be catched by an IRQ in Remote layer later on */
        EC_SLV_APP_applicationRun(pAppInstance);
        OSAL_SCHED_yield();
    }
}

bool EC_SLV_APP_EoE_settingIndHandler(void* pContext_p, uint16_t *pMac_p, uint16_t* pIp_p, uint16_t* pSubNet_p
                                     ,uint16_t* pDefaultGateway_p, uint16_t* pDnsIp_p )
{
    OSALUNREF_PARM(pContext_p);
    OSALUNREF_PARM(pMac_p);
    OSALUNREF_PARM(pIp_p);
    OSALUNREF_PARM(pSubNet_p);
    OSALUNREF_PARM(pDefaultGateway_p);
    OSALUNREF_PARM(pDnsIp_p);

    return true;
}

bool EC_SLV_APP_EoE_receiveHandler(void* pContext_p, uint16_t* pData_p, uint16_t length_p)
{
    OSALUNREF_PARM(pContext_p);
    OSALUNREF_PARM(pData_p);
    OSALUNREF_PARM(length_p);

    return true;
}
/////////////////////////////////////////////////////////////////////////////////////////
/**

\brief    This is the init function

*////////////////////////////////////////////////////////////////////////////////////////
void EC_SLV_APP_applicationInit(EC_SLV_APP_Sapplication_t* pAppInstance_p)
{
    EC_API_EError_t error;

    if (!pAppInstance_p)
    {
        goto Exit;
    }

    // Initialize SDK
    pAppInstance_p->ptEcSlvApi = EC_API_SLV_new();
    if (!pAppInstance_p->ptEcSlvApi)
    {
        OSAL_error(__func__, __LINE__, OSAL_CONTAINER_NOMEMORY, true, 0);
        goto Exit;
    }

    error = EC_SLV_APP_populateBoardFunctions(pAppInstance_p);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Populate board functions Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = EC_SLV_APP_populateSlaveInfo(pAppInstance_p);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create Slave Info Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    /////////////////////////////////////////////////////////
    //////////  Generate application OBD            /////////
    /////////////////////////////////////////////////////////

    /* Creation of Object Data */
    error = EC_SLV_APP_populateOutObjects(pAppInstance_p);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create Object Record 0x2000 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = EC_SLV_APP_populateInOutObjects(pAppInstance_p);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create Test Objects Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    /////////////////////////////////////////////////////////
    //////////        Define Application PDOs       /////////
    /////////////////////////////////////////////////////////

    /////////////////////////////////////
    // Output PDO (master to slave comm)
    /////////////////////////////////////

    error = EC_SLV_APP_populateRxPDO(pAppInstance_p);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create RX PDO Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    /////////////////////////////////////
    // Input PDO (slave to master comm)
    /////////////////////////////////////

    error = EC_SLV_APP_populateTxPDO(pAppInstance_p);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create TX PDO Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = EC_SLV_APP_populateDescriptionObjects(pAppInstance_p);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create Description Object Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    /*EoE*/
    EC_API_SLV_EoE_cbRegisterReceiveHandler     (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_EoE_receiveHandler);
    EC_API_SLV_EoE_cbRegisterSettingIndHandler  (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_EoE_settingIndHandler);

    /*FoE*/
    EC_API_SLV_FoE_cbRegisterOpenFileHandler    (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_FoE_fileOpen);
    EC_API_SLV_FoE_cbRegisterReadFileHandler    (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_FoE_fileRead);
    EC_API_SLV_FoE_cbRegisterWriteFileHandler   (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_FoE_fileWrite);
    EC_API_SLV_FoE_cbRegisterCloseFileHandler   (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_FoE_fileClose);

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE)
    EC_API_SLV_cbRegisterFlashInit              (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_EEP_initFlash);
    EC_API_SLV_EEPROM_cbRegisterWrite           (pAppInstance_p->ptEcSlvApi, OSPIFLASH_APP_STARTMAGIC,   EC_SLV_APP_EEP_writeEeprom);
    EC_API_SLV_EEPROM_cbRegisterLoad            (pAppInstance_p->ptEcSlvApi, OSPIFLASH_APP_STARTMAGIC,   EC_SLV_APP_EEP_loadEeprom);

    EC_API_SLV_cbRegisterUserApplicationRun     (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_applicationRun, pAppInstance_p);
#else
    OSAL_SCHED_startTask(EC_SLV_APP_usrAppRunWrapper, pAppInstance_p, OSAL_TASK_ePRIO_Idle
                        ,(uint8_t*)EC_SLV_APP_appRunWrapTaskStack_g, APPRWRAP_TASK_SIZE_BYTE, 0, "AppRun");
#endif

    error = (EC_API_EError_t)EC_API_SLV_init(pAppInstance_p->ptEcSlvApi);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Slave Init Error Code: 0x%08x\r\n", error);
        goto Exit;
    }

    /* has to be done after init, otherwise Objects do NOT really exist (nullptr) */
    error = EC_SLV_APP_populateDescriptionObjectValues(pAppInstance_p);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Fill Description Object Error code: 0x%08x\r\n", error);
        goto Exit;
    }

#if (defined ENABLE_I2CLEDS) && (ENABLE_I2CLEDS == 1)
    pAppInstance_p->ioexpLedHandle = ESL_OS_ioexp_leds_init() ;
#endif

    pAppInstance_p->state       = EC_API_SLV_eESM_init;
    pAppInstance_p->msec        = 0;
    pAppInstance_p->trigger     = 1000; /* 1000 ms */

    pAppInstance_p->prev        = ESL_OS_clockGet();

    EC_API_SLV_run(pAppInstance_p->ptEcSlvApi);

Exit:
    return;
}

void EC_SLV_APP_applicationDeInit(EC_SLV_APP_Sapplication_t* pAppInstance_p)
{
    if (pAppInstance_p)
    {
        EC_API_SLV_delete(pAppInstance_p->ptEcSlvApi);
        pAppInstance_p->ptEcSlvApi = NULL;
    }
}

#if (defined SHOW_ESCSTATUS) && (SHOW_ESCSTATUS==1)
static void EC_SLV_APP_escStatusAnalysis(EC_SLV_APP_Sapplication_t* pAppInstance_p)
{
    static
    uint16_t lastPortState      = 0;
    uint16_t portState;

    static
    uint16_t lastCounters[8]    = {0};
    uint16_t counters[8]        = {0};
    uint8_t  cntIdx             = 0;

    portState = EC_API_SLV_readWordEscRegister(pAppInstance_p->ptEcSlvApi, 0x110);
    if (lastPortState != portState)
    {
        OSAL_printf("PortState 0x%04x->0x%04x\r\n", lastPortState, portState);
    }
    lastPortState = portState;

    for (cntIdx = 0; cntIdx < 0x0e; cntIdx += 2)
    {
        counters[cntIdx>>1] = EC_API_SLV_readWordEscRegister(pAppInstance_p->ptEcSlvApi, (0x300|cntIdx));
    }

    if (0 != OSAL_MEMORY_memcmp(counters, lastCounters, sizeof(counters)))
    {
        for (cntIdx = 0; cntIdx < 8; ++cntIdx)
        {
            OSAL_printf("Counter 0x%04x: %04x\r\n", 0x300|(cntIdx*2), counters[cntIdx]);
        }
    }
    OSAL_MEMORY_memcpy(lastCounters, counters, sizeof(counters));
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This is the main function
*/
static void EC_SLV_APP_applicationRun(void* pAppCtxt_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstace = (EC_SLV_APP_Sapplication_t*)pAppCtxt_p;
    static
    bool                        gotPDInfo_s         = false;
    static
    uint32_t                    pdOutLen_s          = ~0;
    static
    uint32_t                    pdInLen_s           = ~0;
    uint8_t*                    pdRxBuffer          = NULL;
    uint8_t*                    pdTxBuffer          = NULL;

    if (EC_API_SLV_getState() != EC_API_SLV_eESM_op && gotPDInfo_s)
    {
        pdOutLen_s  = ~0;
        pdInLen_s   = ~0;
        gotPDInfo_s = false;
    }
    if (EC_API_SLV_getState() == EC_API_SLV_eESM_op)
    {
        if (!gotPDInfo_s)
        {
            pdOutLen_s  = BIT2BYTE(EC_API_SLV_getOutputProcDataLength(pApplicationInstace->ptEcSlvApi));
            pdInLen_s   = BIT2BYTE(EC_API_SLV_getInputProcDataLength(pApplicationInstace->ptEcSlvApi));
            gotPDInfo_s = true;
        }

        EC_API_SLV_preSeqOutputPDBuffer(pApplicationInstace->ptEcSlvApi, pdOutLen_s, (void**)&pdRxBuffer);
        EC_API_SLV_preSeqInputPDBuffer(pApplicationInstace->ptEcSlvApi, pdInLen_s, (void**)&pdTxBuffer);

        //Mirror output data into input data
        OSAL_MEMORY_memcpy(pdTxBuffer, pdRxBuffer, (pdOutLen_s>pdInLen_s)?pdInLen_s:pdOutLen_s);

        EC_API_SLV_postSeqOutputPDBuffer(pApplicationInstace->ptEcSlvApi, pdOutLen_s, pdRxBuffer);
        EC_API_SLV_postSeqInputPDBuffer(pApplicationInstace->ptEcSlvApi, pdInLen_s, pdTxBuffer);
    }
}
