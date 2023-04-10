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

#define ENABLE_I2CLEDS 1

#define BIT2BYTE(x)                     (((x)+7) >> 3)

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/

#include "ecSlvSimple.h"
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

#if !(defined FBTL_REMOTE) && !(defined DPRAM_REMOTE)
#include <CUST_PHY_base.h>
#endif

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

// object indices
#define SLS_OBJIDX_DIAGNOSIS 0x2018


static void EC_SLV_APP_applicationRun(void* pAppCtxt_p);

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Read Process data (CoE) callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationCtxt_p  application instance
 *  \param[in]  index_p             object index
 *  \param[in]  subindex_p          object subIndex
 *  \param[in]  size_p              size of data buffer
 *  \param[in]  pData_p             buffer to be read from
 *  \param[in]  completeAccess_p    using complete access
 *  \return     CoE ErrorCode
 *
 *  \remarks
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  uint8_t retVal = 0;
 *  uint8_t pData[0x100];
 *
 *  // the Call
 *  retVal = EC_SLV_APP_getValueFromMaster(pApplicationCtxt_p, 0x1600, 1, 0x100, (uint16_t*)pData, 0);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint8_t EC_SLV_APP_getValueFromMaster(void* pApplicationCtxt_p, uint16_t index_p, uint8_t subindex_p
                                     ,uint32_t size_p, uint16_t MBXMEM * pData_p, uint8_t completeAccess_p)
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback for diagnosis example. It Sends a new Diagnosis message with the received data.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationCtxt_p  application instance
 *  \param[in]  index_p             object index
 *  \param[in]  subindex_p          object subIndex
 *  \param[in]  size_p              size of data buffer
 *  \param[in]  pData_p             buffer to be read from
 *  \param[in]  completeAccess_p    using complete access
 *  \return     CoE ErrorCode
 *
 *  \remarks
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint8_t EC_SLV_APP_writeDiagnosis(void* pApplicationCtxt_p, uint16_t index_p, uint8_t subindex_p
        ,uint32_t size_p, uint16_t MBXMEM * pData_p, uint8_t completeAccess_p)
{
    EC_API_EError_t                 error           = (EC_API_EError_t)ABORT_NOERROR;
    EC_SLV_APP_Sapplication_t*      pAppInstance    = NULL;
    EC_API_SLV_SDIAG_parameter_t    param           = {DEFTYPE_UNSIGNED8,
                                                       DIAG_MSG_PARAM_TYPE_DATA,
                                                       sizeof(uint8_t),
                                                       (uint8_t*) pData_p};

    OSALUNREF_PARM(completeAccess_p);

    if((pApplicationCtxt_p != NULL) && (pData_p != NULL))
    {
        OSAL_printf("%s ==> Idx: 0x%04x:%d | Size: %d | Value: %d\r\n", __func__
                    ,index_p, subindex_p, size_p, pData_p[0]);

            pAppInstance = (EC_SLV_APP_Sapplication_t*) pApplicationCtxt_p;
            EC_API_SLV_DIAG_newMessage(pAppInstance->ptEcSlvApi,
                                       DIAG_CODE_EMCY(pData_p[0]),
                                       DIAG_MSG_TYPE_ERROR,
                                       pData_p[0],
                                       1,
                                       &param);

    }
    return (uint8_t)error;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Write Process data (CoE) callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationCtxt_p  application instance
 *  \param[in]  index_p             object index
 *  \param[in]  subindex_p          object subIndex
 *  \param[in]  size_p              size of data buffer
 *  \param[in]  pData_p             buffer to be read from
 *  \param[in]  completeAccess_p    using complete access
 *  \return     CoE ErrorCode
 *
 *  \remarks
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  uint8_t retVal = 0;
 *  uint8_t pData[0x100] = {0x00};
 *
 *  // the Call
 *  retVal = EC_SLV_APP_setValueToMaster(pApplicationCtxt_p, 0x1600, 1, 0x100, (uint16_t*)pData, 0);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint8_t EC_SLV_APP_setValueToMaster(void* pApplicationCtxt_p, uint16_t index_p, uint8_t subindex_p
                                   ,uint32_t size_p, uint16_t MBXMEM* pData_p, uint8_t completeAccess_p)
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Populate EC Slave meta information
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationInstance_p  Application instance
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_API_EError_t  retVal = 0;
 *  EC_SLV_APP_Sapplication_t* pApplicationInstance_p;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_populateSlaveInfo(pApplicationInstance_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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

    error = (EC_API_EError_t)EC_API_SLV_setProductCode   (ptSlave, ECAT_PRODUCTCODE_SIMPLE);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_setProductName   (ptSlave, ECAT_PRODUCTNAME_SIMPLE);
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Populate EC Slave output data objects
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationInstance_p  Application instance
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_API_EError_t  retVal = 0;
 *  EC_SLV_APP_Sapplication_t* pApplicationInstance_p;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_populateOutObjects(pApplicationInstance_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Populate EC Slave input data objects
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationInstance_p  Application instance
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_API_EError_t  retVal = 0;
 *  EC_SLV_APP_Sapplication_t* pApplicationInstance_p;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_populateInOutObjects(pApplicationInstance_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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
                                                         ,ACCESS_READWRITE, NULL, NULL
                                                         ,EC_SLV_APP_getValueFromMaster, pApplicationInstance);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2001 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, 0x2017, "Incr. by read", DEFTYPE_UNSIGNED32, 32
                                                         ,ACCESS_READ, EC_SLV_APP_setValueToMaster
                                                         ,pApplicationInstance, NULL, NULL);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x2001 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_odAddVariable(ptSlave, SLS_OBJIDX_DIAGNOSIS, "Test Diagnosis", DEFTYPE_UNSIGNED8, 8
                                                            ,ACCESS_READWRITE, NULL
                                                            ,NULL, EC_SLV_APP_writeDiagnosis, pApplicationInstance);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Object 0x%04X Error code: 0x%08x\r\n", SLS_OBJIDX_DIAGNOSIS, error);
        goto Exit;
    }

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Populate EC Slave CoE desciption objects
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationInstance_p  Application instance
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_API_EError_t  retVal = 0;
 *  EC_SLV_APP_Sapplication_t* pApplicationInstance_p;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_populateDescriptionObjects(pApplicationInstance_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Populate EC Slave CoE desciption objects data
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationInstance_p  Application instance
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_API_EError_t  retVal = 0;
 *  EC_SLV_APP_Sapplication_t* pApplicationInstance_p;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_populateDescriptionObjectValues(pApplicationInstance_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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
    OSAL_printf("%s:%d PDO Out Len: 0x%lx\r\r\n", __func__, __LINE__, value);
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Configure RX PDO entries
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationInstance_p  Application instance
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_API_EError_t  retVal = 0;
 *  EC_SLV_APP_Sapplication_t* pApplicationInstance_p;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_populateRxPDO(pApplicationInstance_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
EC_API_EError_t EC_SLV_APP_populateRxPDO(EC_SLV_APP_Sapplication_t* pApplicationInstance_p)
{
    EC_API_SLV_SHandle_t*       ptSlave;
    EC_API_EError_t             error       = EC_API_eERR_INVALID;
    EC_API_SLV_SCoE_ObjEntry_t* ptObjEntry;

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

    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2000, 2, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptRxPdo1600, "SubIndex 001", ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2000, 3, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptRxPdo1600, "SubIndex 002", ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2000, 4, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptRxPdo1600, "SubIndex 003", ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_create(ptSlave, "RxPDO2", 0x1601, &pApplicationInstance_p->ptRxPdo1601);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create PDO 0x1601 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    OSAL_printf("RxPDO created 0x1601: 0x%lx\r\n", (uint32_t)pApplicationInstance_p->ptRxPdo1601);

    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2000, 1, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptRxPdo1601, "SubIndex 001", ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Configure TX PDO entries
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationInstance_p  Application instance
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_API_EError_t  retVal = 0;
 *  EC_SLV_APP_Sapplication_t* pApplicationInstance_p;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_populateTxPDO(pApplicationInstance_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
EC_API_EError_t EC_SLV_APP_populateTxPDO(EC_SLV_APP_Sapplication_t* pApplicationInstance_p)
{
    EC_API_SLV_SHandle_t*           ptSlave;
    EC_API_EError_t                 error           = EC_API_eERR_INVALID;
    EC_API_SLV_SCoE_ObjEntry_t*     ptObjEntry;

    if (!pApplicationInstance_p)
    {
        goto Exit;
    }

    ptSlave = pApplicationInstance_p->ptEcSlvApi;

    error = (EC_API_EError_t)EC_API_SLV_PDO_create(ptSlave, "TxPDO", 0x1A00, &pApplicationInstance_p->ptTxPdo1A00);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create PDO 0x1A00 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    OSAL_printf("TxPDO created 0x1A00: 0x%lx\r\n", (uint32_t)pApplicationInstance_p->ptTxPdo1A00);

    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2002, 2, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptTxPdo1A00
                                                       ,"SubIndex 001", ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2002, 4, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptTxPdo1A00
                                                       ,"SubIndex 002", ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("PDO 0x1A00 Entry 2 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2002, 3, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptTxPdo1A00
                                                       ,"SubIndex 003", ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("PDO 0x1A00 Entry 3 Error code: 0x%08x\r\n", error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_create(ptSlave, "TxPDO2", 0x1A01, &pApplicationInstance_p->ptTxPdo1A01);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("Create PDO 0x1A01 Error code: 0x%08x\r\n", error);
        goto Exit;
    }
    OSAL_printf("TxPDO created 0x1A01: 0x%lx\r\n", (uint32_t)pApplicationInstance_p->ptTxPdo1A01);

    error = (EC_API_EError_t)EC_API_SLV_CoE_getObjectEntry(ptSlave, 0x2002, 1, &ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = (EC_API_EError_t)EC_API_SLV_PDO_createEntry(ptSlave, pApplicationInstance_p->ptTxPdo1A01
                                                       ,"SubIndex 001", ptObjEntry);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Variable Error code: 0x%08x\r\n", __func__, __LINE__, error);
        goto Exit;
    }

    error = EC_API_eERR_NONE;
Exit:
    return error;
}

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE)
/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Reset / UnReset Ethernet PHY
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pCtxt_p     application instance
 *  \param[in]  phyIdx_p    PHY index (0/1)
 *  \param[in]  reset_p     true: reset, false: run
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  void* pApp;
 *
 *  // the Call
 *  \endcode
 *  void EC_SLV_APP_boardPhyReset(pApp, 0, true);
 *
 *  <!-- References: -->
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
static OSAL_FUNC_UNUSED void EC_SLV_APP_boardPhyReset(void* pCtxt_p, uint8_t phyIdx_p, bool reset_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstance    = (EC_SLV_APP_Sapplication_t*)pCtxt_p;

    if (NULL == pApplicationInstance)
    {
        goto Exit;
    }

    ESL_BOARD_OS_phyReset(pApplicationInstance->gpioHandle, pApplicationInstance->selectedPruInstance, phyIdx_p, reset_p);

Exit:
    return;
}
#endif

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Stimulate board status LED callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pCallContext_p      application instance handle.
 *  \param[in]  pLedContext_p       LED specific context
 *  \param[in]  runLed_p            true: run LED on, false: off
 *  \param[in]  errLed_p            true: error LED on, false: off
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  void* pAppL;
 *
 *  // the Call
 *  EC_SLV_APP_appBoardStatusLed(pAppL, NULL, true, false);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Register board related functions
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppInstance_p      Application instance handle
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_API_EError_t retVal;
 *
 *  retVal = EC_SLV_APP_populateBoardFunctions *pAppInstance;
 *
 *  // the Call
 *  EC_SLV_APP_initBoardFunctions(pAppInstance);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initialize board related functions
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppInstance_p      Application instance handle
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_SLV_APP_Sapplication_t *pAppInstance;
 *
 *  // the Call
 *  EC_SLV_APP_initBoardFunctions(pAppInstance);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Register board related functions, which do not use stack handle
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppInstance_p      Application instance handle
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  EC_SLV_APP_Sapplication_t *pAppInstance;
 *
 *  // the Call
 *  EC_SLV_APP_registerStacklessBoardFunctions(pAppInstance);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
void EC_SLV_APP_registerStacklessBoardFunctions(EC_SLV_APP_Sapplication_t *pAppInstance_p)
{
    if (!pAppInstance_p)
    {
        goto Exit;
    }

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE)
    ESL_BOARD_OS_registerPhys(pAppInstance_p->selectedPruInstance);

    CUST_PHY_CBregisterLibDetect(CUST_PHY_detect, pAppInstance_p);
    CUST_PHY_CBregisterReset(EC_SLV_APP_boardPhyReset, pAppInstance_p);
#endif
Exit:
    return;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Change on PDO assignment configuration
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]      pContext_p          The pointer to the EtherCAT API instance.
 *  \param[in]      rx_p                Change on RX or TX PDOs.
 *  \param[in]      count_p             Number of PDOs assigned to SyncManager.
 *  \param[in]      pPdoIndexArray_p    Array of PDO indexes assigned to SyncManager.
  *  \return     DTK error code
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint32_t EC_SLAVE_APP_assignmentChangedHandler(void* pContext_p, bool rx_p, uint8_t count_p, uint16_t* pPdoIndexArray_p)
{
    uint32_t error  = EC_API_eERR_NONE;
    uint8_t i       = 0;
    OSALUNREF_PARM(pContext_p);

    if(pPdoIndexArray_p != NULL)
    {
        OSAL_printf("**************************************\r\n");
        if(rx_p == true)
        {
            OSAL_printf("New assignments for SyncManager 2:\r\n");
        }
        else
        {
            OSAL_printf("New assignments for SyncManager 3:\r\n");
        }
        for(i = 0; i < count_p; i++)
        {
            OSAL_printf("PDO: 0x%04x\r\n", pPdoIndexArray_p[i]);
        }
        OSAL_printf("**************************************\r\n");
    }
    if (count_p == 0)
    {
        error = EC_API_eERR_ABORT;
    }
    return error;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Change on PDO mapping configuration
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]      pContext_p          The pointer to the EtherCAT API instance.
 *  \param[in]      pdoIndex_p          PDO index number.
 *  \param[in]      count_p             Number of objects mapped as PDO.
 *  \param[in]      pPdoMap_p           PDO mapping entries.
  *  \return     DTK error code
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint32_t EC_SLAVE_APP_mappingChangedHandler(void* pContext_p, uint16_t pdoIndex_p, uint8_t count_p, EC_SLV_API_PDO_SEntryMap_t* pPdoMap_p)
{
    uint32_t error  = EC_API_eERR_NONE;
    uint8_t i       = 0;
    OSALUNREF_PARM(pContext_p);
    if(pPdoMap_p != NULL)
    {
        OSAL_printf("**************************************\r\n");
        OSAL_printf("New mapping for PDO 0x%04x: \r\n", pdoIndex_p);
        for(i = 0; i < count_p; i++)
        {
            OSAL_printf("0x%04x:%d\r\n", pPdoMap_p[i].index, pPdoMap_p[i].subIndex);
        }
        OSAL_printf("**************************************\r\n");
    }
    if (count_p == 0)
    {
        error = EC_API_eERR_ABORT;
    }
    return error;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  AoE read request handler
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]      pContext_p          The pointer to the EtherCAT API instance.
 *  \param[in]      port_p              AMS port.
 *  \param[in]      index_p             16 bit index value from IndexOffset.
 *  \param[in]      subIndex_p          8 bit subIndex value from IndexOffset.
 *  \param[in]      completeAccess_p    CoE Complete Access flag.
 *  \param[in,out]  pLength_p           Request data length.
 *  \param[in]      pData_p             Pointer to data.
 *  \return     ADS error code
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint16_t EC_SLV_APP_AoE_readRequest(void*       pContext_p,
                                    uint16_t    port_p,
                                    uint16_t    index_p,
                                    uint8_t     subIndex_p,
                                    bool        completeAccess_p,
                                    uint32_t*   pLength_p,
                                    uint16_t*   pData_p)
{
    uint32_t                    dtkError        = EC_API_eERR_NONE;
    uint16_t                    adsError        = ERR_NOERROR;

    EC_API_SLV_SHandle_t*       pEcSlvApi       = (EC_API_SLV_SHandle_t*) pContext_p;
    EC_API_SLV_SCoE_ObjEntry_t* pObjectEntry    = NULL;
    EC_API_SLV_SCoE_Object_t*   pObject         = NULL;
    uint32_t                    length          = 0;
    if(pLength_p != NULL && pData_p != NULL)
    {
        if(completeAccess_p == false)
        {
            dtkError = EC_API_SLV_CoE_getObjectEntry(pEcSlvApi, index_p, subIndex_p, &pObjectEntry);
            if(dtkError == EC_API_eERR_NONE)
            {
                EC_API_SLV_CoE_getObjectEntryLength(pEcSlvApi, pObjectEntry, &length);
                if(length <= *pLength_p)
                {
                    *pLength_p = length;
                    dtkError = EC_API_SLV_CoE_getObjectEntryData(pEcSlvApi, pObjectEntry,length, pData_p);
                    if(dtkError != EC_API_eERR_NONE)
                    {
                        adsError = ADSERR_DEVICE_ERROR;
                    }
                }
                else
                {
                    adsError = ADSERR_DEVICE_INVALIDSIZE;
                }
            }
            else
            {
                adsError = ADSERR_DEVICE_NOTFOUND;
            }
        }
        else
        {
            dtkError = EC_API_SLV_CoE_getObject(pEcSlvApi, index_p, &pObject);
            if(dtkError == EC_API_eERR_NONE)
            {
                if(subIndex_p == 0)
                {
                    EC_API_SLV_CoE_getObjectLength(pEcSlvApi, pObject, &length);
                    if(length <= *pLength_p)
                    {
                        *pLength_p = length;
                        EC_API_SLV_CoE_getObjectEntryCount(pEcSlvApi, pObject, (uint8_t*) &pData_p[0]);
                        dtkError = EC_API_SLV_CoE_getObjectData(pEcSlvApi, pObject, length, &pData_p[1]);
                        if(dtkError != EC_API_eERR_NONE)
                        {
                            adsError = ADSERR_DEVICE_ERROR;
                        }
                    }
                    else
                    {
                        adsError = ADSERR_DEVICE_INVALIDSIZE;
                    }
                }
                else if (subIndex_p == 1)
                {
                    EC_API_SLV_CoE_getObjectLength(pEcSlvApi, pObject, &length);
                    if(length <= *pLength_p)
                    {
                        *pLength_p = length;
                        dtkError = EC_API_SLV_CoE_getObjectData(pEcSlvApi, pObject,length, pData_p);
                        if(dtkError != EC_API_eERR_NONE)
                        {
                            adsError = ADSERR_DEVICE_ERROR;
                        }
                    }
                    else
                    {
                        adsError = ADSERR_DEVICE_INVALIDSIZE;
                    }
                }
                else
                {
                    adsError = ADSERR_DEVICE_INVALIDPARM;
                }
            }
            else
            {
                adsError = ADSERR_DEVICE_NOTFOUND;
            }
        }
    }
    else
    {
        adsError = ADSERR_DEVICE_INVALIDPARM;
    }
    return adsError;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  AoE write request handler
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]      pContext_p          The pointer to the EtherCAT API instance.
 *  \param[in]      port_p              AMS port.
 *  \param[in]      index_p             16 bit index value from IndexOffset.
 *  \param[in]      subIndex_p          8 bit subIndex value from IndexOffset.
 *  \param[in]      completeAccess_p    CoE Complete Access flag.
 *  \param[in,out]  pLength_p            Request data length.
 *  \param[in]      pData_p             Pointer to data.
 *  \return     ADS error code
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
uint16_t EC_SLV_APP_AoE_writeRequest(void*      pContext_p,
                                     uint16_t   port_p,
                                     uint16_t   index_p,
                                     uint8_t    subIndex_p,
                                     bool       completeAccess_p,
                                     uint32_t*  pLength_p,
                                     uint16_t*  pData_p)
{
    uint32_t dtkError = EC_API_eERR_NONE;
    uint16_t adsError = ERR_NOERROR;

    EC_API_SLV_SHandle_t*       pEcSlvApi       = (EC_API_SLV_SHandle_t*) pContext_p;
    EC_API_SLV_SCoE_ObjEntry_t* pObjectEntry    = NULL;
    EC_API_SLV_SCoE_Object_t*   pObject         = NULL;
    if(pData_p != NULL)
    {
        if(completeAccess_p == false)
        {
            dtkError = EC_API_SLV_CoE_getObjectEntry(pEcSlvApi, index_p, subIndex_p, &pObjectEntry);
            if(dtkError == EC_API_eERR_NONE)
            {
                dtkError = EC_API_SLV_CoE_setObjectEntryData(pEcSlvApi, pObjectEntry, *pLength_p, pData_p);
                if(dtkError != EC_API_eERR_NONE)
                {
                    adsError = ADSERR_DEVICE_ERROR;
                }
            }
            else
            {
                adsError = ADSERR_DEVICE_NOTFOUND;
            }
        }
        else
        {
            dtkError = EC_API_SLV_CoE_getObject(pEcSlvApi, index_p, &pObject);
            if(dtkError == EC_API_eERR_NONE)
            {
                if(subIndex_p == 0)
                {
                    dtkError = EC_API_SLV_CoE_setObjectData(pEcSlvApi, pObject, *pLength_p, &pData_p[1]);
                    if(dtkError != EC_API_eERR_NONE)
                    {
                        adsError = ADSERR_DEVICE_ERROR;
                    }
                }
                else if (subIndex_p == 1)
                {
                    dtkError = EC_API_SLV_CoE_setObjectData(pEcSlvApi, pObject, *pLength_p, pData_p);
                    if(dtkError != EC_API_eERR_NONE)
                    {
                        adsError = ADSERR_DEVICE_ERROR;
                    }
                }
                else
                {
                    adsError = ADSERR_DEVICE_INVALIDPARM;
                }
            }
            else
            {
                adsError = ADSERR_DEVICE_NOTFOUND;
            }
        }
    }
    else
    {
        adsError = ADSERR_DEVICE_NOMEMORY;
    }
    return adsError;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  EoE Settings Indication callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p          The pointer to the application instance.
 *  \param[in]  pMac_p              Virtual Net MAC address
 *  \param[in]  pIp_p               Virtual Net IP address
 *  \param[in]  pSubNet_p           Virtual Net Subnet
 *  \param[in]  pDefaultGateway_p   Virtual Net Default Gateway
 *  \param[in]  pDnsIp_p            Virtual Net DNS server address
 *  \return     true if settings are handled, false otherwise
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  User defined EoE receive function. Called when an EoE frame is received.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p          function context
 *  \param[in]  pData_p             EoE Frame Data
 *  \param[in]  length_p            EoE Frame Size
 *  \return true if frame is handle, false if it should be passed on.
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
bool EC_SLV_APP_EoE_receiveHandler(void* pContext_p, uint16_t* pData_p, uint16_t length_p)
{
    OSALUNREF_PARM(pContext_p);
    OSALUNREF_PARM(pData_p);
    OSALUNREF_PARM(length_p);

    return true;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initialize application
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppInstance_p  Application instance handle
 *  \return     ErrorCode p Closer description of ErrorCode, if required.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // required variables
 *  EC_SLV_APP_Sapplication_t* pAppInstance;
 *
 *  // the Call
 *  EC_SLV_APP_applicationInit(pAppInstance);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
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

    /*PDO Mapping Changes*/
    EC_API_SLV_PDO_registerAssignmentChanges(pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLAVE_APP_assignmentChangedHandler);
    EC_API_SLV_PDO_registerMappingChanges(pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLAVE_APP_mappingChangedHandler);

    /*AoE*/
    EC_API_SLV_AoE_cbRegisterReadRequestHandler (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_AoE_readRequest);
    EC_API_SLV_AoE_cbRegisterWriteRequestHandler(pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_AoE_writeRequest);

    /*EoE*/
    EC_API_SLV_EoE_cbRegisterReceiveHandler     (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_EoE_receiveHandler);
    EC_API_SLV_EoE_cbRegisterSettingIndHandler  (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_EoE_settingIndHandler);

    /*FoE*/
    EC_API_SLV_FoE_cbRegisterOpenFileHandler    (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_FoE_fileOpen);
    EC_API_SLV_FoE_cbRegisterReadFileHandler    (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_FoE_fileRead);
    EC_API_SLV_FoE_cbRegisterWriteFileHandler   (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_FoE_fileWrite);
    EC_API_SLV_FoE_cbRegisterCloseFileHandler   (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_FoE_fileClose);

    /*Diagnosis support */
    EC_API_SLV_DIAG_enable(pAppInstance_p->ptEcSlvApi);

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE) && !(defined OSAL_FREERTOS_JACINTO) /* first omit flash */
    EC_API_SLV_cbRegisterFlashInit              (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_EEP_initFlash);
    EC_API_SLV_EEPROM_cbRegisterWrite           (pAppInstance_p->ptEcSlvApi, OSPIFLASH_APP_STARTMAGIC,   EC_SLV_APP_EEP_writeEeprom);
    EC_API_SLV_EEPROM_cbRegisterLoad            (pAppInstance_p->ptEcSlvApi, OSPIFLASH_APP_STARTMAGIC,   EC_SLV_APP_EEP_loadEeprom);
/*#else
    OSAL_SCHED_startTask(EC_SLV_APP_usrAppRunWrapper, pAppInstance_p, OSAL_TASK_ePRIO_Idle
                        ,(uint8_t*)EC_SLV_APP_appRunWrapTaskStack_g, APPRWRAP_TASK_SIZE_BYTE, 0, "AppRun");*/
#endif
    EC_API_SLV_cbRegisterUserApplicationRun     (pAppInstance_p->ptEcSlvApi, pAppInstance_p->ptEcSlvApi, EC_SLV_APP_applicationRun, pAppInstance_p);

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

    pAppInstance_p->state       = EC_API_SLV_eESM_init;
    pAppInstance_p->msec        = 0;
    pAppInstance_p->trigger     = 1000; /* 1000 ms */

    pAppInstance_p->prev        = ESL_OS_clockGet();

    EC_API_SLV_run(pAppInstance_p->ptEcSlvApi);

Exit:
    return;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Deinitialize application
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppInstance_p  Application instance handle
 *  \return     ErrorCode p Closer description of ErrorCode, if required.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // required variables
 *  EC_SLV_APP_Sapplication_t* pAppInstance;
 *
 *  // the Call
 *  EC_SLV_APP_applicationDeInit(pAppInstance);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
void EC_SLV_APP_applicationDeInit(EC_SLV_APP_Sapplication_t* pAppInstance_p)
{
    if (pAppInstance_p)
    {
        EC_API_SLV_delete(pAppInstance_p->ptEcSlvApi);
        pAppInstance_p->ptEcSlvApi = NULL;
    }
}

/// \cond DO_NOT_DOCUMENT
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
/// \endcond

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Main application run
 *
 *  \details
 *
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p		application context
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // required variables
 *  void* pApp;
 *
 *  // the Call
 *  EC_SLV_APP_applicationRun(pApp);
 *
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
static void EC_SLV_APP_applicationRun(void* pAppCtxt_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstace = (EC_SLV_APP_Sapplication_t*)pAppCtxt_p;
    static uint8_t              lastLed             = 0;
    static
    EC_API_SLV_EEsmState_t      lastState           = EC_API_SLV_eESM_uninit;   /* last known stack state to notify changes */
    EC_API_SLV_EEsmState_t      curState            = EC_API_SLV_eESM_uninit;   /* current stack state */
    static
    bool                        gotLEDOffset        = false;                    /* process data offset of LED object acquired? */
    static
    uint32_t                    ledPDOffset         = 0;                        /* offset of PD LEDs in process data image */
    static
    uint32_t                    pdOutLen            = 0;                        /* length of out process data for buffer exchange */

#if (defined SHOW_LOOPCOUNT) && (SHOW_LOOPCOUNT==1)
    static uint32_t             loops               = 0;
#endif

    if (NULL == pApplicationInstace)
    {
        OSAL_error(__func__, __LINE__, OSAL_eERR_INVALIDSTATE, true, 1, "App Instance = NULL");
    }
#if (defined ENABLE_I2CLEDS) && (ENABLE_I2CLEDS == 1)
    /* be sure we own I2C in this thread */
    if (NULL == pApplicationInstace->ioexpLedHandle)
    {
        pApplicationInstace->ioexpLedHandle = ESL_OS_ioexp_leds_init() ;
    }
#endif

    curState = EC_API_SLV_getState();

    if (curState != lastState)
    {
        OSAL_printf("State change: 0x%x -> 0x%x\r\n", lastState, curState);
        pApplicationInstace->prev = ESL_OS_clockGet();
    }
    lastState = curState;

    if (EC_API_SLV_eESM_op != curState)
    {
        if (gotLEDOffset)
        {
            OSAL_printf("Discard LED Offset\r\n");
            ledPDOffset = 0;
            pdOutLen = 0;
            gotLEDOffset = false;
        }
    }

    if (EC_API_SLV_eESM_op == curState)
    {
        /*
         * uint32_t pdOutLen = EC_API_SLV_getOutputProcDataLength(ptEcSlvApi);
         * uint32_t pdInLen = EC_API_SLV_getInputProcDataLength(ptEcSlvApi);
         *
         * uint32_t offset = EC_API_SLV_PDO_getOffset(ptRxPdo1600);
         * uint32_t len = EC_API_SLV_PDO_getLength(ptRxPdo1600);
         * */

        EC_API_EError_t err;
        static volatile
        uint8_t         ledPDData;

        if (!gotLEDOffset)
        {
            pdOutLen = BIT2BYTE(EC_API_SLV_getOutputProcDataLength(pApplicationInstace->ptEcSlvApi));
            if (pdOutLen > sizeof(pApplicationInstace->pdBuffer))
            {
                pdOutLen = sizeof(pApplicationInstace->pdBuffer);
            }

            ledPDOffset = EC_API_SLV_PDO_getOffset(pApplicationInstace->ptRxPdo1600);
            gotLEDOffset = true;
            OSAL_printf("LED Offset = %d\r\n", ledPDOffset);
        }

        EC_API_SLV_getOutputData(pApplicationInstace->ptEcSlvApi, pdOutLen, pApplicationInstace->pdBuffer);

        /*Mirror output data into input data*/
        /*err = (EC_API_EError_t)EC_API_SLV_PDO_getEntryData(
                    pApplicationInstace->ptEcSlvApi,
                    pApplicationInstace->ptRxPdo1600,
                    1,
                    1,
                    (uint8_t*)(&ledPDData));
        if(err != EC_API_eERR_NONE)
        {
            OSAL_printf("Cannot get leddata 0x%08x\r\n", err);
        }*/

        ledPDData = pApplicationInstace->pdBuffer[ledPDOffset];

        if (pApplicationInstace->ioexpLedHandle && lastLed != ledPDData)
        {
            lastLed = ledPDData;
#if (defined ENABLE_I2CLEDS) && (ENABLE_I2CLEDS == 1)
            ESL_OS_ioexp_leds_write(pApplicationInstace->ioexpLedHandle, lastLed);
#else
            OSAL_printf("Update I2C to 0x%x\r\n", lastLed);
#endif
        }

#if !(defined FBTL_REMOTE) || (0 == FBTL_REMOTE)
        err = (EC_API_EError_t)EC_API_SLV_PDO_setEntryData(pApplicationInstace->ptEcSlvApi
                                                          ,pApplicationInstace->ptTxPdo1A00
                                                          ,1
                                                          ,sizeof(uint8_t)
                                                          ,(uint8_t*)&ledPDData);
        if (err != EC_API_eERR_NONE)
        {
            OSAL_printf("Fill Description Object Error code: 0x%08x\r\n", err);
        }
#endif
    }
    else
    {
        clock_t now = ESL_OS_clockGet();

        if (pApplicationInstace->prev > now)
        {
            pApplicationInstace->prev = now;
        }
        pApplicationInstace->diff = ESL_OS_clockDiff(pApplicationInstace->prev, &now);

        if (pApplicationInstace->ioexpLedHandle && pApplicationInstace->diff)
        {
#if (defined SHOW_LOOPCOUNT) && (SHOW_LOOPCOUNT==1)
            OSAL_printf("LoopCount 0x%08x\r\n", loops);
#endif

#if (defined ENABLE_I2CLEDS) && (ENABLE_I2CLEDS == 1)
            ESL_OS_ioexp_leds_write(pApplicationInstace->ioexpLedHandle, lastLed);
#else
            OSAL_printf("Update I2C to 0x%x\r\n", lastLed);
#endif

            pApplicationInstace->prev = now;

            lastLed = lastLed ? lastLed << 1 : 1;

            //Write Led data to the process data. It can be seen in OBD in Object 0x2002:2 as well.
            EC_API_SLV_PDO_setEntryData(pApplicationInstace->ptEcSlvApi, pApplicationInstace->ptTxPdo1A00, 1, sizeof(uint8_t), (uint8_t*)&lastLed);
        }
    }

#if (defined SHOW_ESCSTATUS) && (SHOW_ESCSTATUS==1)
    EC_SLV_APP_escStatusAnalysis(pApplicationInstace);
#endif
#if (defined SHOW_LOOPCOUNT) && (SHOW_LOOPCOUNT==1)
    loops++;
#endif

}
