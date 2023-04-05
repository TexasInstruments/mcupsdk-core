/*!
 *  \example ESL_cia402Demo.c
 *
 *  \brief
 *  CiA 402 Callbacks Example
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2020-06-19
 *
 *  \copyright
 *  Copyright (c) 2020, KUNBUS GmbH<br /><br />
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

#include <osal.h>

#include <ESL_os.h>
#include <ESL_BOARD_OS_config.h>

#include "ecSlvCiA402.h"

#include "ESL_cia402Demo.h"
#include "ESL_cia402Obd.h"
#include "ESL_gpioHelper.h"

#if !(defined BIT2BYTE)
#define BIT2BYTE(x)                     (((x)+7) >> 3)
#endif

/* if dynamic value change while OP is required, this costs 15usec per cycle ! */
#define     ENABLE_DYNAMIC_POSITION_LIMITS      0

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Read CiA402 Axis value
 *
 *  \details
 *  If objects are mapped and known (on SafeOP/OP) use direct access to linear process data memory
 *  when not mapped, use traditional PDO data API.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  type_p      Type of variable.
 *  \param[out] target_p    Target to read value to.
 *  \param[in]  axisDesc_p  Axis variable descriptor.
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
#define EC_SLV_APP_CIA_GETAXISVALUE(type_p, target_p, axisDesc_p) \
    if (gotOffsets && (NULL != (axisDesc_p).pdoObject)) { \
    (target_p) = ((type_p*)&(pApplication_p->pdRxBuffer[(axisDesc_p).pdoOffset]))[0]; } else { \
    EC_SLV_APP_getCiA402ObjectValue(pApplication_p, (axisDesc_p).pSdo, sizeof(type_p), (uint16_t*)&(target_p)); }

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Write CiA402 Axis value
 *
 *  \details
 *  If objects are mapped and known (on SafeOP/OP) use direct access to linear process data memory
 *  when not mapped, use traditional PDO data API.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  type_p      Type of variable.
 *  \param[in]  axisDesc_p  Axis variable descriptor.
 *  \param[in]  value_p     Variable to write value from.
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
#define EC_SLV_APP_CIA_SETAXISVALUE(type_p, axisDesc_p, value_p) \
    if (gotOffsets && (NULL != (axisDesc_p).pdoObject)) { \
    ((type_p*)&(pApplication_p->pdTxBuffer[(axisDesc_p).pdoOffset]))[0] = (value_p); } else { \
    EC_SLV_APP_setCiA402ObjectValue(pApplication_p, &(axisDesc_p), sizeof(type_p), (uint16_t*)&(value_p)); }

/** \brief Data structure to handle an CiA 402 axis */
typedef struct
{
    uint8_t     id; /**< \brief Axis Identification */
    bool        axisIsActive; /**< \brief Axis is active */
    bool        brakeApplied; /**< \brief Motor brake is applied */
    bool        lowLevelPowerApplied; /**< \brief Low-level power applied*/
    bool        highLevelPowerApplied; /**< \brief High-level power applied*/
    bool        axisFunctionEnabled; /**< \brief Axis functions enabled*/
    bool        configurationAllowed; /**< \brief Configuration allowed*/
    double      positionActualValue; /**< \brief Actual position within control loop*/
    uint32_t    cycleTime; /**< \brief Motion controller cycletime in us*/
}
EC_SLV_API_CiA402_SAxis_t;

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/
static EC_SLV_API_CiA402_SAxis_t       localAxes_s[AXES_NUMBER];

//Supported drive modes: ETG6010 6.8.1
#define SUPPORTED_DRIVE_MODE_CSP_BIT    7
#define SUPPORTED_DRIVE_MODE_CSV_BIT    8
#define SUPPORTED_DRIVE_MODE_CST_BIT    9
#define DRIVE_MODE_CSP                  (1 << SUPPORTED_DRIVE_MODE_CSP_BIT)
#define DRIVE_MODE_CSV                  (1 << SUPPORTED_DRIVE_MODE_CSV_BIT)
#define DRIVE_MODE_CST                  (1 << SUPPORTED_DRIVE_MODE_CST_BIT)

#define DRIVE_GEAR_RELATION             0.0010922
#define POSITION_MAX_LIMIT              0xFFFFFFFF

#define NON_DC_DEFAULT_CYCLE_TIME_USEC  4000
#define NSEC_TO_USEC                    1000
#define ESC_DC_SYNC0_CYCLETIME_REG      0x09A0

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Read CiA402 Objects
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplication_p  application Instance.
 *  \param[in]  pObject_p       CiA402 Object.
 *  \param[in]  length_p   		Object length.
 *  \param[out] pValue_p   		Object Value.
 *  \return     ErrorCode       SDK Error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  void* pAppInstance;
 *  uint16_t index;
 *  uint8_t axis;
 *  int16_t* value;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_getCiA402ObjectValue(pAppInstance, index, axis, sizeof(value), &value);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
uint32_t EC_SLV_APP_getCiA402ObjectValue(EC_SLV_APP_Sapplication_t* pApplication_p, EC_API_SLV_SCoE_Object_t* pObject_p, uint16_t length_p, uint16_t* pValue_p)
{
    EC_API_SLV_SHandle_t*   pEcApiSlv   = NULL;
    uint32_t                err         = EC_API_eERR_INVALID;

    if (NULL == pApplication_p || NULL == pObject_p)
    {
        goto Exit;
    }

    pEcApiSlv = pApplication_p->ptEcSlvApi;

    err = EC_API_SLV_CoE_getObjectData(pEcApiSlv, pObject_p, length_p, pValue_p);

Exit:
    return  err;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Write CiA402 Object value.
 *  \remark
 *  Use OBD indexes described in ETG6010 Chapter 16. axis_p parameter calculates the axis object index.
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplication_p	application Instance.
 *  \param[in]  pCiaObject_p    CiA402 application object descriptor.
 *  \param[in]  length_p   		Object length.
 *  \param[in]  pValue_p   		Object Value.
 *  \return     ErrorCode       SDK Error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_SLV_APP_Sapplication_t* S_appInstance;
 *  EC_SLV_APP_sCIA_object_t object;
 *  int16_t* value;
  *
 *  // the Call
 *  retVal = EC_SLV_APP_setCiA402ObjectValue(S_appInstance, &object, sizeof(value), &value);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
uint32_t EC_SLV_APP_setCiA402ObjectValue(EC_SLV_APP_Sapplication_t*  pApplication_p, EC_SLV_APP_sCIA_object_t* pCiaObject_p, uint16_t length_p, uint16_t* pValue_p)
{
    uint32_t err;
    EC_API_SLV_SCoE_Object_t* pObject = NULL;

    if (pCiaObject_p->pSdo)
    {
        pObject = pCiaObject_p->pSdo;
    }
    else
    {
        err = EC_API_SLV_CoE_getObject(pApplication_p->ptEcSlvApi, pCiaObject_p->objectIndex, &pObject);
        if (err)
        {
            goto Exit;
        }
    }

    err = EC_API_SLV_CoE_setObjectData(pApplication_p->ptEcSlvApi, pObject, length_p, pValue_p);

Exit:
    return  err;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Read CiA402 Object entry.
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p              Application Instance.
 *  \param[in]  pObjectEntry_p          CiA402 Object entry.
 *  \param[in]  length_p   		        Object entry length.
 *  \param[in]  pValue_p   		        Object entry value.
 *  \return     ErrorCode               SDK Error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SCoE_Object_t* pObject;
 *  EC_API_SLV_SCoE_ObjEntry_t* pObjectEntry;
 *  int16_t* value;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_getCiA402ObjectEntryValue(appInst, pObjectEnry, sizeof(value), &value);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
uint32_t EC_SLV_APP_getCiA402ObjectEntryValue(void* pAppCtxt_p, EC_API_SLV_SCoE_ObjEntry_t* pObjectEntry_p, uint16_t length_p, uint16_t* pValue_p)
{
    uint32_t                    error           = EC_API_eERR_INVALID;
    EC_API_SLV_SHandle_t*       pEcSlvApi       = NULL;
    EC_SLV_APP_Sapplication_t*  pApplication    = (EC_SLV_APP_Sapplication_t*)pAppCtxt_p;

    if (!pApplication || !pObjectEntry_p)
    {
        error = EC_API_eERR_NULLPTR;
        goto Exit;
    }

    pEcSlvApi = pApplication->ptEcSlvApi;

    error = EC_API_SLV_CoE_getObjectEntryData(pEcSlvApi, pObjectEntry_p, length_p, pValue_p);

Exit:
    return  error;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Write CiA402 Object entry.
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pEcApiSlv_p		SDK Instance.
 *  \param[in]  index_p   		CiA402 Object index.
 *  \param[in]  subIndex_p      CiA402 Object entry subIndex.
 *  \param[in]  length_p   		Object entry length.
 *  \param[in]  pValue_p   		Object entry value.
 *  \return     ErrorCode       SDK Error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *  uint16_t index;
 *  uint8_t subIndex;
 *  uint8_t axis;
 *  int16_t* value;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_setCiA402ObjectEntryValue(S_ecSlvApiHdl, index, subIndex, axis, sizeof(value), &value);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
uint32_t EC_SLV_APP_setCiA402ObjectEntryValue(EC_API_SLV_SHandle_t* pEcApiSlv_p, uint16_t index_p, uint8_t subIndex_p, uint16_t length_p, uint16_t* pValue_p)
{
    EC_API_SLV_SCoE_ObjEntry_t* pObjEntry;
    uint32_t err;
    err = EC_API_SLV_CoE_getObjectEntry(pEcApiSlv_p,index_p, subIndex_p, &pObjEntry);
    if (err == EC_API_eERR_NONE)
    {
        err = EC_API_SLV_CoE_setObjectEntryData(pEcApiSlv_p, pObjEntry, length_p, pValue_p);
    }
    return  err;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Set supported drive modes.
 *
 *  \details
 *  Set object 0x6502 (Supported drive modes). Activate support for CSP and CSV profiles.
 *  Check ETG6010 Object Definitions for more information.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pApplication_p		Application Instance.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *
 *  // the call
 *  EC_SLV_APP_setCiA402ObjectValue(S_ecSlvApiHdl);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 * */
uint32_t EC_SLV_APP_setSupportedDriveModes(EC_SLV_APP_Sapplication_t*  pApplication_p)
{
    uint32_t err;
    uint32_t driveMode = DRIVE_MODE_CSP | DRIVE_MODE_CSV | DRIVE_MODE_CST;
    uint8_t axisNo;

    for( axisNo = 0; axisNo < AXES_NUMBER; axisNo++)
    {
        err = EC_SLV_APP_setCiA402ObjectValue(pApplication_p,
                                              &pApplication_p->CiA402_axisData[axisNo].supportedDriveModesIndex,
                                              sizeof (driveMode), (uint16_t*) &driveMode);
    }
    return err;
}



/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Set default values for CiA 402 object dictionary
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pCtxt_p		function context.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *
 *  // the Call
 *  EC_SLV_APP_setObdValues(S_ecSlvApiHdl);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
void EC_SLV_APP_setObdValues(void* pCtxt_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstance    = (EC_SLV_APP_Sapplication_t*)pCtxt_p;
    int32_t                     posMaxLimit             = POSITION_MAX_LIMIT;
    uint8_t                     axisNo;

    OSAL_printf("+%s\r\n", __func__);

    EC_SLV_APP_setSupportedDriveModes(pApplicationInstance);

    for(axisNo = 0; axisNo < AXES_NUMBER; axisNo++)
    {
        EC_SLV_APP_setCiA402ObjectEntryValue(pApplicationInstance->ptEcSlvApi, OBD_SW_POSITION_LIMIT_INDEX(axisNo), 2, sizeof(posMaxLimit), (uint16_t*) &posMaxLimit);
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Get cycle time information
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pCtxt_p		Callback context.
 *  \param[in]  pIntMask_p	Register 0x204 value.
 *  \return     ErrorCode   Closer description of ErrorCode, if required.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *  uint16_t intMask;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_startInputHandler(S_ecSlvApiHdl, &intMask);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
EC_API_SLV_EUserRetCodes_t EC_SLV_APP_startInputHandler(void* pCtxt_p, uint16_t* pIntMask_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstance    = (EC_SLV_APP_Sapplication_t*)pCtxt_p;
    EC_API_SLV_SHandle_t*       pEcApiSlv               = NULL;
    uint32_t                    sync0CycleTime          = 0;
    EC_API_SLV_EUserRetCodes_t  retVal                  = EC_USR_eRET_ERROR;
    uint8_t                     axisNo;

    OSALUNREF_PARM(pIntMask_p);

    if (!pApplicationInstance)
    {
        goto Exit;
    }

    pEcApiSlv = pApplicationInstance->ptEcSlvApi;

    sync0CycleTime= EC_API_SLV_readDoubleWordEscRegister(pEcApiSlv, ESC_DC_SYNC0_CYCLETIME_REG);

    sync0CycleTime = sync0CycleTime / NSEC_TO_USEC; //get cycle time in us

    for(axisNo = 0; axisNo < AXES_NUMBER; axisNo++)
    {
        localAxes_s[axisNo].id = axisNo;

        if (localAxes_s[axisNo].axisIsActive)
        {
            localAxes_s[axisNo].cycleTime = sync0CycleTime;
        }
        if(!localAxes_s[axisNo].cycleTime)
        {
            localAxes_s[axisNo].cycleTime = NON_DC_DEFAULT_CYCLE_TIME_USEC;
        }
    }

    retVal = EC_USR_eRET_OK;
Exit:
    return retVal;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  This function shall calculate the desired Axis input values to move on a predefined ramp.
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  characteristic_p    Ramp description.
 *  \return     bool                True if transition finished.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // required variables
 *  int16_t characteristic
 *
 *  // the Call
 *  retVal = EC_SLV_APP_transitionAction(characteristic);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
bool EC_SLV_APP_transitionAction(int16_t characteristic_p)
{
   switch(characteristic_p)
   {
   case SLOW_DOWN_RAMP:
        //do stuff
        break;
   case QUICKSTOP_RAMP:
        //do stuff
        break;
   case STOP_ON_CURRENT_LIMIT:
        //do stuff
        break;
   case STOP_ON_VOLTAGE_LIMIT:
        //do stuff
        break;
   default:
      break;
    }

    return true;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Cyclic synchronous torque mode. ETG6010 6.4
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplication_p  Application instance.
 *  \param[in]  pCiA402Axis_p   Servo Axis description structure.
 *  \param[in]  gotOffsets      got offsets
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *  EC_SLV_API_CiA402_SAxis_t* pCiA402Axis;
 *
 *  // the Call
 *  EC_SLV_APP_CST(S_ecSlvApiHdl, pCiA402Axis);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 * */
void EC_SLV_APP_CST(EC_SLV_APP_Sapplication_t* pApplication_p, EC_SLV_API_CiA402_SAxis_t* pCiA402Axis_p, bool gotOffsets)
{
    int16_t targetTorque;

    //Read target torque value
    EC_SLV_APP_CIA_GETAXISVALUE(int16_t, targetTorque, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].targetTorqueIndex);

    //Update torque value
    EC_SLV_APP_CIA_SETAXISVALUE(int16_t, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].torqueActualValueIndex, targetTorque);
}

/*! <!-- Function: \fn void EC_SLV_APP_CSV(EC_API_SLV_SHandle_t* pEcApiSlv_p, EC_SLV_API_CiA402_SAxis_t* pCiA402Axis_p) -->
 *  <!-- Description: -->
 *
 *  \brief
 *  Cyclic synchronous velocity mode. ETG6010 6.3
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplication_p  Application instance.
 *  \param[in]  pCiA402Axis		Servo Axis description structure.
 *  \param[in]  gotOffsets      got PDO offsets
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *  EC_SLV_API_CiA402_SAxis_t* pCiA402Axis;
 *
 *  // the Call
 *  EC_SLV_APP_CSV(S_ecSlvApiHdl, pCiA402Axis);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 * */
void EC_SLV_APP_CSV(EC_SLV_APP_Sapplication_t* pApplication_p, EC_SLV_API_CiA402_SAxis_t* pCiA402Axis_p, bool gotOffsets)
{
    int32_t targetVelocity;

    //Read target velocity value
    EC_SLV_APP_CIA_GETAXISVALUE(int32_t, targetVelocity, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].targetVelocityIndex);

    //Update velocity value
    EC_SLV_APP_CIA_SETAXISVALUE(int32_t, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].velocityActualValueIndex, targetVelocity);
}

/*! <!-- Function: \fn void EC_SLV_APP_CSP(EC_API_SLV_SHandle_t* pEcApiSlv_p, EC_SLV_API_CiA402_SAxis_t* pCiA402Axis_p) -->
 *  <!-- Description: -->
 *
 *  \brief
 *  Cyclic synchronous position mode. ETG6010 6.2
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplication_p	application instance.
 *  \param[in]  pCiA402Axis		Servo Axis description structure.
 *  \param[in]  gotOffsets      got PDO offsets
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *  EC_SLV_API_CiA402_SAxis_t* pCiA402Axis;
 *
 *  // the Call
 *  EC_SLV_APP_CSP(S_ecSlvApiHdl, pCiA402Axis);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 * */
void EC_SLV_APP_CSP(EC_SLV_APP_Sapplication_t* pApplication_p, EC_SLV_API_CiA402_SAxis_t* pCiA402Axis_p, bool gotOffsets)
{
    uint32_t targetPosition, actualPosition;
    int32_t targetVelocity, actualVelocity;
    int16_t targetTorque, actualTorque;

    float incFactor = (float) (DRIVE_GEAR_RELATION * pCiA402Axis_p->cycleTime);

    //Read target position, velocity and torque values
    EC_SLV_APP_CIA_GETAXISVALUE(uint32_t, targetPosition, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].targetPositionIndex);
    EC_SLV_APP_CIA_GETAXISVALUE(int32_t,  targetVelocity, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].targetVelocityIndex);
    EC_SLV_APP_CIA_GETAXISVALUE(int16_t,  targetTorque,   pApplication_p->CiA402_axisData[pCiA402Axis_p->id].targetTorqueIndex);

    //Read actual position, velocity and torque values
    EC_SLV_APP_CIA_GETAXISVALUE(uint32_t, actualPosition, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].positionActualValueIndex);
    EC_SLV_APP_CIA_GETAXISVALUE(int32_t,  actualVelocity, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].velocityActualValueIndex);
    EC_SLV_APP_CIA_GETAXISVALUE(int16_t,  actualTorque,   pApplication_p->CiA402_axisData[pCiA402Axis_p->id].torqueActualValueIndex);

    if (incFactor != 0)
    {
        actualVelocity = (targetPosition - actualPosition) / incFactor;
        pCiA402Axis_p->positionActualValue = actualPosition + actualVelocity;
    }

    //Update position and velocity value
    EC_SLV_APP_CIA_SETAXISVALUE(int32_t,  pApplication_p->CiA402_axisData[pCiA402Axis_p->id].velocityActualValueIndex, actualVelocity);
    EC_SLV_APP_CIA_SETAXISVALUE(uint32_t, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].positionActualValueIndex, pCiA402Axis_p->positionActualValue);
}

/*! <!-- Function: \fn void CiA402_motionControl(EC_API_SLV_SHandle_t* ptSlvApi, EC_SLV_API_CiA402_SAxis_t* pCiA402Axis) -->
 *  <!-- Description: -->
 *
 *  \brief
 *  This functions provides a simple feedback functionality.
 *  \remarks
 *  Motion Controller shall only be triggered if application is trigger by DC Sync Signals
 *  and a valid mode of operation is set
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplication_p	application instance.
 *  \param[in]  pCiA402Axis		Servo Axis description structure.
 *  \param[in]  gotOffsets      got PDO offsets
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *  EC_SLV_API_CiA402_SAxis_t* pCiA402Axis;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_motionControl(S_ecSlvApiHdl, pCiA402Axis);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 * */
void EC_SLV_APP_motionControl(EC_SLV_APP_Sapplication_t*  pApplication_p, EC_SLV_API_CiA402_SAxis_t* pCiA402Axis_p, bool gotOffsets)
{
    uint16_t statusWord, controlWord;
    uint32_t targetPosition;
    uint32_t posMaxLimit, posMinLimit;
    uint8_t operationModeDisplay;

    if (!pApplication_p)
    {
        goto Exit;
    }

    //Read control, status and drive operation mode objects
    EC_SLV_APP_CIA_GETAXISVALUE(uint16_t, controlWord, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].controlWordIndex);
    EC_SLV_APP_CIA_GETAXISVALUE(uint16_t, statusWord, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].statusWordIndex);
    EC_SLV_APP_CIA_GETAXISVALUE(uint8_t, operationModeDisplay, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].modesOfOperationDisplayIndex);

    //Read target position
    EC_SLV_APP_CIA_GETAXISVALUE(uint32_t, targetPosition, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].targetPositionIndex);

    //Get software position limits
#if (defined ENABLE_DYNAMIC_POSITION_LIMITS) && (1==ENABLE_DYNAMIC_POSITION_LIMITS) /* if dynamic value change while OP is required, this costs 15usec per cycle ! */
    EC_SLV_APP_getCiA402ObjectEntryValue(pApplication_p,
                                         pApplication_p->CiA402_axisData[pCiA402Axis_p->id].positionLimitMin.pObjetEntry,
                                         sizeof(pApplication_p->CiA402_axisData[pCiA402Axis_p->id].posLimitMin),
                                         (uint16_t *) &pApplication_p->CiA402_axisData[pCiA402Axis_p->id].posLimitMin);
    EC_SLV_APP_getCiA402ObjectEntryValue(pApplication_p,
                                         pApplication_p->CiA402_axisData[pCiA402Axis_p->id].positionLimitMax.pObjetEntry,
                                         sizeof(pApplication_p->CiA402_axisData[pCiA402Axis_p->id].posLimitMax),
                                         (uint16_t *) &pApplication_p->CiA402_axisData[pCiA402Axis_p->id].posLimitMax);
#endif
    posMinLimit = pApplication_p->CiA402_axisData[pCiA402Axis_p->id].posLimitMin;
    posMaxLimit = pApplication_p->CiA402_axisData[pCiA402Axis_p->id].posLimitMax;

    //Calculate new targets for CSP, CSV or CST modes
    if(controlWord == CONTROLWORD_COMMAND_ENABLEOPERATION)
    {
        if((posMaxLimit> pCiA402Axis_p->positionActualValue  || pCiA402Axis_p->positionActualValue > targetPosition)
         &&(posMinLimit < pCiA402Axis_p->positionActualValue || pCiA402Axis_p->positionActualValue < targetPosition))
        {
            statusWord &= ~STATUSWORD_INTERNAL_LIMIT;

            switch(operationModeDisplay)
            {
            case CYCLIC_SYNC_POSITION_MODE:
                EC_SLV_APP_CSP(pApplication_p, pCiA402Axis_p, gotOffsets);
                break;
            case CYCLIC_SYNC_VELOCITY_MODE:
                //nothing
                EC_SLV_APP_CSV(pApplication_p, pCiA402Axis_p, gotOffsets);
                break;
            case CYCLIC_SYNC_TORQUE_MODE:
                //nothing
                EC_SLV_APP_CST(pApplication_p, pCiA402Axis_p, gotOffsets);
                break;
            default:
                break;
            }
        }
        else
        {
            statusWord |= STATUSWORD_INTERNAL_LIMIT;
        }
    }

    //Update drive status
    EC_SLV_APP_CIA_SETAXISVALUE(uint16_t,  pApplication_p->CiA402_axisData[pCiA402Axis_p->id].statusWordIndex, statusWord);

    //Accept new mode of operation
    EC_SLV_APP_CIA_GETAXISVALUE(uint8_t, operationModeDisplay, pApplication_p->CiA402_axisData[pCiA402Axis_p->id].modesOfOperationIndex);
    EC_SLV_APP_CIA_SETAXISVALUE(uint8_t,  pApplication_p->CiA402_axisData[pCiA402Axis_p->id].modesOfOperationDisplayIndex, operationModeDisplay);

Exit:
    return;
}

/*! <!-- Function: \fn void EC_SLV_APP_cia402Application(void* ctxt) -->
 *  <!-- Description: -->
 *
 *  \brief
 *  CiA402 Application function.
 *
 *  \details
 *  Write Control Word RxPDO to activate axis with value 7.
 *  Write to Operation Mode RxPDO the position, velocity or torque modes (8, 9, 10).
 *  Write Control Word RxPDO to enable axis operation with value 15.
 *  Write a target torque, velocity or position value to the RxPDO mapping.
 *  Read the TxPDO or OBD values of the actual position, velocity or torque.
 *  Read the TxPDO or OBD values of the status word and the operation mode display.
 *
 *  \remarks
 *  If device goes into OP state without activating the axis, status goes into quickstop mode (4096).
 *  Reset quickstop status by activating the axis in control word object.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pCtxt_p		EtherCAT slave handle.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *
 *  // the Call
 *  EC_SLV_APP_cia402Application(ctxt);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 * */
void EC_SLV_APP_cia402Application(void* pCtxt_p)
{
    EC_SLV_APP_Sapplication_t*  pApplication_p    = (EC_SLV_APP_Sapplication_t*)pCtxt_p;
    EC_API_SLV_SHandle_t*       pEcApiSlv               = NULL;
    static
    bool                        gotOffsets              = false;
    uint8_t                     axisNo;

    uint16_t controlWord, statusWord, errorCode;
    int16_t quickStopOptionCode, shutdownOptionCode, disableOperationCode, faultReactionCode;
    uint8_t operationDisplayCode;

    uint16_t driveRamp = DISABLE_DRIVE;

    EC_API_SLV_EEsmState_t  curState;

    if (!pApplication_p)
    {
        goto Exit;
    }

    pEcApiSlv = pApplication_p->ptEcSlvApi;

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#if (defined GPIO_TEST_PROFILE_SEL) && (GPIO_TEST_PROFILE_1 == GPIO_TEST_PROFILE_SEL)
        ESL_GPIO_testPins_set(ESL_TESTPIN_STATE_REG_BANK, ESL_TESTPIN_2_MASK);
#endif
#endif

    curState = EC_API_SLV_getState();

    curState &= ~EC_API_SLV_eESM_errState;

    if ((EC_API_SLV_eESM_op != curState) && (EC_API_SLV_eESM_safeop != curState) && gotOffsets)
    {
        EC_SLV_APP_CiA_dropPDOffsets(pApplication_p);
        pApplication_p->pdoOutLen = ~0;
        pApplication_p->pdoInLen  = ~0;
        gotOffsets = false;
    }

    if (((EC_API_SLV_eESM_op == curState) || (EC_API_SLV_eESM_safeop == curState)) && !gotOffsets)
    {
        EC_SLV_APP_CiA_fetchPDOffsets(pApplication_p);

        pApplication_p->pdoOutLen = BIT2BYTE(EC_API_SLV_getOutputProcDataLength(pApplication_p->ptEcSlvApi));
        pApplication_p->pdoInLen  = BIT2BYTE(EC_API_SLV_getInputProcDataLength(pApplication_p->ptEcSlvApi));

        OSAL_printf("PDO size Out:0x%x/0x%x, In:0x%x/0x%x\r\n", pApplication_p->pdoOutLen, pApplication_p->realPdoOutLen, pApplication_p->pdoInLen, pApplication_p->realPdoInLen);
        gotOffsets = true;
    }

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#if (defined GPIO_TEST_PROFILE_SEL) && (GPIO_TEST_PROFILE_1 == GPIO_TEST_PROFILE_SEL)
        ESL_GPIO_testPins_clear(ESL_TESTPIN_STATE_REG_BANK, ESL_TESTPIN_2_MASK);
#endif
#endif

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#if (defined GPIO_TEST_PROFILE_SEL) && (GPIO_TEST_PROFILE_1 == GPIO_TEST_PROFILE_SEL)
    ESL_GPIO_testPins_set(ESL_TESTPIN_STATE_REG_BANK, ESL_TESTPIN_2_MASK);
#endif
#endif

    if (((EC_API_SLV_eESM_op == curState) || (EC_API_SLV_eESM_safeop == curState)) && gotOffsets)
    {
        EC_API_SLV_preSeqInputPDBuffer(pApplication_p->ptEcSlvApi, pApplication_p->realPdoInLen, (void**)&(pApplication_p->pdTxBuffer));
    }
    if (EC_API_SLV_eESM_op == curState && gotOffsets)
    {
        EC_API_SLV_preSeqOutputPDBuffer(pApplication_p->ptEcSlvApi, pApplication_p->realPdoOutLen, (void**)&(pApplication_p->pdRxBuffer));
    }

    for(axisNo = 0; axisNo < AXES_NUMBER; axisNo++)
    {
        //Read drive control and status objects
        EC_SLV_APP_CIA_GETAXISVALUE(uint16_t, controlWord, pApplication_p->CiA402_axisData[axisNo].controlWordIndex);
        EC_SLV_APP_CIA_GETAXISVALUE(uint16_t, statusWord, pApplication_p->CiA402_axisData[axisNo].statusWordIndex);

        //Enable high level power, no torque on the motor.
        if(controlWord == CONTROLWORD_COMMAND_SWITCHON)
        {
            OSAL_printf("Axis %d Activated\n\r", axisNo);
            EC_API_SLV_CiA402_activateAxis(pEcApiSlv, axisNo, true);
        }

        //Read drive's operation mode
        EC_SLV_APP_CIA_GETAXISVALUE(uint8_t, operationDisplayCode, pApplication_p->CiA402_axisData[axisNo].modesOfOperationDisplayIndex);

        //Read supported error option codes (ETG6010 Chapter 4).
        errorCode = EC_API_SLV_CiA402_SM_getErrorCode(pEcApiSlv, axisNo);

        if(errorCode &&
                (operationDisplayCode == CYCLIC_SYNC_POSITION_MODE ||
                 operationDisplayCode == CYCLIC_SYNC_VELOCITY_MODE ||
                 operationDisplayCode == CYCLIC_SYNC_TORQUE_MODE))
        {
            statusWord &= ~ STATUSWORD_DRIVE_FOLLOWS_COMMAND;
        }
        else
        {
            statusWord |= STATUSWORD_DRIVE_FOLLOWS_COMMAND;
        }

        //Analyse error codes
        switch(errorCode)
        {
        case OBD_QUICKSTOP_INDEX(0):
            /*State transition 11 is pending analyse shutdown option code (0x605A)*/
            {
                EC_SLV_APP_CIA_GETAXISVALUE(uint16_t, quickStopOptionCode, pApplication_p->CiA402_axisData[axisNo].quickStopIndex);

                /*Masked and execute specified quick stop ramp characteristic */
                if(quickStopOptionCode >= SLOWDOWN_RAMP_NO_TRANSIT && quickStopOptionCode <= VOLTAGE_LIMIT_NO_TRANSIT)
                {
                    if (quickStopOptionCode == SLOWDOWN_RAMP_NO_TRANSIT)
                    {
                        driveRamp = SLOW_DOWN_RAMP;
                    }
                    if (quickStopOptionCode == QUICKSTOP_RAMP_NO_TRANSIT)
                    {
                        driveRamp = QUICKSTOP_RAMP;
                    }
                    if (quickStopOptionCode == CURRENT_LIMIT_NO_TRANSIT)
                    {
                        driveRamp = STOP_ON_CURRENT_LIMIT;
                    }
                    if (quickStopOptionCode == VOLTAGE_LIMIT_NO_TRANSIT)
                    {
                        driveRamp = STOP_ON_VOLTAGE_LIMIT;
                    }
                }

                if(EC_SLV_APP_transitionAction(driveRamp))
                {
                    /*Quick stop ramp is finished complete state transition*/
                    EC_API_SLV_CiA402_SM_clearErrorCode(pEcApiSlv, axisNo);
                    if (quickStopOptionCode >= SLOWDOWN_RAMP_NO_TRANSIT && quickStopOptionCode <= VOLTAGE_LIMIT_NO_TRANSIT)
                    {
                        statusWord |= STATUSWORD_TARGET_REACHED;
                    }
                }
            }
            break;
        case OBD_SHUTDOWN_INDEX(0):
            /*State transition 8 is pending analyse shutdown option code (0x605B)*/
            {
                EC_SLV_APP_CIA_GETAXISVALUE(uint16_t, shutdownOptionCode, pApplication_p->CiA402_axisData[axisNo].shutdownIndex);

                if(EC_SLV_APP_transitionAction(shutdownOptionCode))
                {
                    /*shutdown ramp is finished complete state transition*/
                    EC_API_SLV_CiA402_SM_clearErrorCode(pEcApiSlv, axisNo);
                }
            }
            break;
        case OBD_DISABLE_OPERATION_INDEX(0):
            /*State transition 5 is pending analyse Disable operation option code (0x605C)*/
            {
                EC_SLV_APP_CIA_GETAXISVALUE(uint16_t, disableOperationCode, pApplication_p->CiA402_axisData[axisNo].disableOperationIndex);

                if(EC_SLV_APP_transitionAction(disableOperationCode))
                {
                    /*disable operation ramp is finished complete state transition*/
                    EC_API_SLV_CiA402_SM_clearErrorCode(pEcApiSlv, axisNo);
                }
            }
            break;

        case OBD_FAULT_REACTION_INDEX(0):
            /*State transition 14 is pending analyse Fault reaction option code (0x605E)*/
            {
                EC_SLV_APP_CIA_GETAXISVALUE(uint16_t, faultReactionCode, pApplication_p->CiA402_axisData[axisNo].faultReactionIndex);

                if(EC_SLV_APP_transitionAction(faultReactionCode))
                {
                    /*fault reaction ramp is finished complete state transition*/
                    EC_API_SLV_CiA402_SM_clearErrorCode(pEcApiSlv, axisNo);
                }
            }
            break;
        default:
            //Pending transition code is invalid => values from the master are used
            statusWord |= STATUSWORD_DRIVE_FOLLOWS_COMMAND;
            break;
        }

        EC_SLV_APP_CIA_SETAXISVALUE(uint16_t, pApplication_p->CiA402_axisData[axisNo].statusWordIndex, statusWord);

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#if (defined GPIO_TEST_PROFILE_SEL) && (GPIO_TEST_PROFILE_1 == GPIO_TEST_PROFILE_SEL)
        ESL_GPIO_testPins_set(ESL_TESTPIN_STATE_REG_BANK, ESL_TESTPIN_1_MASK);
#endif
#endif

        EC_SLV_APP_motionControl(pApplication_p, &localAxes_s[axisNo], gotOffsets);

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#if (defined GPIO_TEST_PROFILE_SEL) && (GPIO_TEST_PROFILE_1 == GPIO_TEST_PROFILE_SEL)
        ESL_GPIO_testPins_clear(ESL_TESTPIN_STATE_REG_BANK, ESL_TESTPIN_1_MASK);
#endif
#endif
    }

    if (((EC_API_SLV_eESM_op == curState) || (EC_API_SLV_eESM_safeop == curState)) && gotOffsets)
    {
        EC_API_SLV_postSeqInputPDBuffer(pApplication_p->ptEcSlvApi, pApplication_p->realPdoInLen, pApplication_p->pdTxBuffer);
    }
    if (EC_API_SLV_eESM_op == curState && gotOffsets)
    {
        EC_API_SLV_postSeqOutputPDBuffer(pApplication_p->ptEcSlvApi, pApplication_p->realPdoOutLen, pApplication_p->pdRxBuffer);
    }

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#if (defined GPIO_TEST_PROFILE_SEL) && (GPIO_TEST_PROFILE_1 == GPIO_TEST_PROFILE_SEL)
        ESL_GPIO_testPins_clear(ESL_TESTPIN_STATE_REG_BANK, ESL_TESTPIN_2_MASK);
#endif
#endif
Exit:
    return;
}

/*! <!-- Function: \fn EC_SLV_APP_cia402LocalError(void* ctxt, uint16_t ErrorCode) -->
 *  <!-- Description: -->
 *
 *  \brief
 *  Local Error function handler.
 *  \details
 *  Called if CiA402 state machine changes to an error state.
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pCtxt_p     function context.
 *  \param[in]  errorCode_p error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  EC_API_SLV_SHandle_t* S_ecSlvApiHdl;
 *  uint16_t errorCode;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_cia402LocalError(S_ecSlvApiHdl, errorCode);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 * */
void EC_SLV_APP_cia402LocalError(void* pCtxt_p, uint16_t errorCode_p)
{
    OSALUNREF_PARM(pCtxt_p);
    OSAL_printf("Local error triggered: 0x%04x\r\n", errorCode_p);
}

//*************************************************************************************************
