/*!
 *  \example ESL_cia402Obd.c
 *
 *  \brief
 *  Interface for creating CiA 402 Object Dictionary - "AXES_NUMBER" axes.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-04-01
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

#include "ESL_cia402Obd.h"
#include "ESL_cia402Demo.h"
#include "ecSlvCiA402.h"

#include <ecSlvApi.h>
#include <ecSlvApi_Error.h>
#include <ecSlvApiDef.h>


/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Generate CiA402 Objects
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  ecSlaveApi		SDK Instance.
 *  \return     ErrorCode       Error code.
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
 *  retVal = EC_SLV_APP_usrApplCia402ObjectDictionary(S_ecSlvApiHdl);
 *  \endcode
 *
 *  <!-- References: -->
 *
 *  <!-- Group: -->
 *
 *  \ingroup CiA402
 *
 */
uint32_t EC_SLV_APP_cia402ObjectDictionary(void* pContext_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstance    = (EC_SLV_APP_Sapplication_t*)pContext_p;
    EC_API_SLV_SHandle_t*       pEcApiSlv               = NULL;
    uint32_t                    error                   = EC_API_eERR_INVALID;
    uint8_t                     axis;

    EC_API_SLV_SCoE_Object_t*   pObj607B                = NULL;
    EC_API_SLV_SCoE_Object_t*   pObj607D                = NULL;
    EC_API_SLV_SCoE_Object_t*   pObj608F                = NULL;
    EC_API_SLV_SCoE_Object_t*   pObj6090                = NULL;
    EC_API_SLV_SCoE_Object_t*   pObj6091                = NULL;
    EC_API_SLV_SCoE_Object_t*   pObj6092                = NULL;
    EC_API_SLV_SCoE_Object_t*   pObj6096                = NULL;
    EC_API_SLV_SCoE_Object_t*   pObj6099                = NULL;
    EC_API_SLV_SCoE_Object_t*   pObj60C2                = NULL;

    if (!pApplicationInstance)
    {
        goto Exit;
    }

    pEcApiSlv = pApplicationInstance->ptEcSlvApi;

    for(axis = 0; axis < AXES_NUMBER; ++axis)
    {
        //Object 0x6007: Abort connection
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_ABORT_CONNECTION_OPTION_CODE_INDEX(axis), "Abort connection option code", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6040: Controlword
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].controlWordIndex.objectIndex, "Controlword", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6041: Statusword
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].statusWordIndex.objectIndex, "Statusword", DEFTYPE_UNSIGNED16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x605A: Quickstop option code
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].quickStopIndex.objectIndex, "Quick stop option code", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x605B: Shutdown option code
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].shutdownIndex.objectIndex, "Shutdown option code", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x605C: Disable operation option code
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].disableOperationIndex.objectIndex, "Disable operation option code", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x605D: Halt option code
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_HALT_OPTION_CODE_INDEX(axis), "Halt option code", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x605E: Fault reaction option code
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].faultReactionIndex.objectIndex, "Fault reaction option code", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6060: Mode of operation
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].modesOfOperationIndex.objectIndex, "Modes of operation", DEFTYPE_INTEGER8, 8, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6061: Modes of operation display
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].modesOfOperationDisplayIndex.objectIndex, "Modes of operation display", DEFTYPE_INTEGER8, 8, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6062: Position demand value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_POSITION_DEMAND_VALUE_INDEX(axis), "Position demand value", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6063: Position actual internal value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_POSITION_ACTUAL_INTERNAL_VALUE_INDEX(axis), "Position actual internal value", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6064: Position actual value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].positionActualValueIndex.objectIndex, "Position actual value", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6065: Following error window
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_FOLLOWING_ERROR_WINDOW_INDEX(axis), "Following error window", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6066: Following error timeout
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_FOLLOWING_ERROR_TIMEOUT_INDEX(axis), "Following error time out", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6067: Position window
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_POSITION_WINDOW_INDEX(axis), "Position window", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6068: Position window time
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_POSITION_WINDOW_TIME_INDEX(axis), "Position window time", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6069: Velocity sensor actual value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_VELOCITY_SENSOR_ACTUAL_VALUE_INDEX(axis), "Velocity sensor actual value", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x606B: Velocity demand value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_VELOCITY_DEMAND_VALUE_INDEX(axis), "Velocity demand value", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x606C: Velocity actual value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].velocityActualValueIndex.objectIndex, "Velocity actual value", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x606D: Velocity window
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_VELOCITY_WINDOW_INDEX(axis), "Velocity window", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x606E: Velocity window time
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_VELOCITY_WINDOW_TIME_INDEX(axis), "Velocity window time", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x606F: Velocity threshold
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_VELOCITY_THRESHOLD_INDEX(axis), "Velocity threshold", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6070: Velocity threshold time
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_VELOCITY_THRESHOLD_TIME_INDEX(axis), "Velocity threshold time", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6071: Target torque
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].targetTorqueIndex.objectIndex, "Target torque", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6072: Max torque
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_MAX_TORQUE_INDEX(axis), "Max torque", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6074: Torque demand
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TORQUE_DEMAND_INDEX(axis), "Torque demand", DEFTYPE_INTEGER16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6077: Torque actual value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].torqueActualValueIndex.objectIndex, "Torque actual value", DEFTYPE_INTEGER16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6078: Current actual value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_CURRENT_ACTUAL_VALUE_INDEX(axis), "Current actual value", DEFTYPE_INTEGER16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x607A: Target position
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].targetPositionIndex.objectIndex, "Target position", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x607B: Position range limit
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, OBD_POSITION_RANGE_LIMIT_INDEX(axis), "Position range limit", NULL, NULL, NULL, NULL, &pObj607B);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj607B)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj607B, 1, "Min position range limit", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj607B, 2, "Max position range limit", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x607C: Home offset
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_HOME_OFFSET_INDEX(axis), "Home offset", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x607D: Software position limit
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].swPositionLimitIndex.objectIndex, "Software position limit", NULL, NULL, NULL, NULL, &pObj607D);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj607D)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj607D, 1, "Min position limit", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj607D, 2, "Max position limit", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x607E: Polarity
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_POLARITY_INDEX(axis), "Polarity", DEFTYPE_UNSIGNED8, 8, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6080: Max motor speed
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_MAX_MOTOR_SPEED_INDEX(axis), "Max motor speed", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6081: Profile velocity
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_PROFILE_VELOCITY_INDEX(axis), "Profile velocity", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6083: Profile acceleration
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_PROFILE_ACCELERATION_INDEX(axis), "Profile acceleration", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6084: Profile deceleration
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_PROFILE_DECELERATION_INDEX(axis), "Profile deceleration", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6085: Quickstop deceleration
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_QUISTOP_DECELERATION_INDEX(axis), "Quick stop deceleration", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6086: Motion profile type
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_MOTION_PROFILE_TYPE_INDEX(axis), "Motion profile type", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x608F: Position encoder resolution
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, OBD_POSITION_ENCODER_RESOLUTION_INDEX(axis), "Position encoder resolution", NULL, NULL, NULL, NULL, &pObj608F);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj608F)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj608F, 1, "Encoder increments", DEFTYPE_UNSIGNED32, 32, ACCESS_READ);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj608F, 2, "Motor revolutions", DEFTYPE_UNSIGNED32, 32, ACCESS_READ);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x6090: Velocity encoder resolution
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, OBD_VELOCITY_ENCONDER_RESOLUTION_INDEX(axis), "Velocity encoder resolution", NULL, NULL, NULL, NULL, &pObj6090);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj6090)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6090, 1, "Encoder increments", DEFTYPE_UNSIGNED32, 32, ACCESS_READ);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6090, 2, "Motor revolutions", DEFTYPE_UNSIGNED32, 32, ACCESS_READ);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x6091: Gear ratio
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, OBD_GEAR_RATIO_INDEX(axis), "Gear ratio", NULL, NULL, NULL, NULL, &pObj6091);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj6091)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6091, 1, "Motor shaft revolutions", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6091, 2, "Driving shaft revolutions", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x6092: Feed constant
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, OBD_FEED_CONSTANT_INDEX(axis), "Feed constant", NULL, NULL, NULL, NULL, &pObj6092);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj6092)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6092, 1, "Feed", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6092, 2, "Shaft revolutions", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x6096: Velocity factor
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, OBD_VELOCITY_FACTOR_INDEX(axis), "Velocity factor", NULL, NULL, NULL, NULL, &pObj6096);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj6096)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6096, 1, "Numerator", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6096, 2, "Divisor", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x6098: Homing method
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_HOMING_METHOD_INDEX(axis), "Homing method", DEFTYPE_INTEGER8, 8, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6099: Homing speeds
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, OBD_HOMING_SPEEDS_INDEX(axis), "Homing speeds", NULL, NULL, NULL, NULL, &pObj6099);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj6099)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6099, 1, "Switch seek velocity", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj6099, 2, "Homing speed", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x609A: Homing acceleration
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_HOMING_ACCELERATION_INDEX(axis), "Homing acceleration", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60B0: Position offset
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_POSITION_OFFSET_INDEX(axis), "Position offset", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60B1: Velocity offset
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_VELOCITY_OFFSET_INDEX(axis), "Velocity offset", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60B2: Torque offset
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TORQUE_OFFSET_INDEX(axis), "Torque offset", DEFTYPE_INTEGER16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60B8: Touch probe function
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_FUNCTION_INDEX(axis), "Touch probe function", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60B9: Touch probe status
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_STATUS_INDEX(axis), "Touch probe status", DEFTYPE_UNSIGNED16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60BA: Touch probe 1 positive edge
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_1_POS_EDGE_INDEX(axis), "Touch probe 1 positive edge", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60BB: Touch probe 1 negative edge
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_1_NEG_EDGE_INDEX(axis), "Touch probe 1 negative edge", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60BC: Touch probe 2 positive edge
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_2_POS_EDGE_INDEX(axis), "Touch probe 2 positive edge", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60BD: Touch probe 2 negative edge
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_2_NEG_EDGE_INDEX(axis), "Touch probe 2 negative edge", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60C2: Interpolation time period
        error = EC_API_SLV_CoE_odAddRecord(pEcApiSlv, OBD_INTERPOLATION_TIME_PERIOD_INDEX(axis), "Interpolation time period", NULL, NULL, NULL, NULL, &pObj60C2);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
        if(pObj60C2)
        {
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj60C2, 1, "Interpolation time period value", DEFTYPE_UNSIGNED8, 8, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
            error = EC_API_SLV_CoE_configRecordSubIndex(pEcApiSlv, pObj60C2, 2, "Interpolation time index", DEFTYPE_INTEGER8, 8, ACCESS_READ);
            if (error != EC_API_eERR_NONE)
            {
                OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
                goto Exit;
            }
        }

        //Object 0x60C5: Max acceleration
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_MAX_ACCELERATION_INDEX(axis), "Max acceleration", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60C6: Max deceleration
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_MAX_DECELERATION_INDEX(axis), "Max deceleration", DEFTYPE_UNSIGNED32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60D0: Touch probe source
        error = EC_API_SLV_CoE_odAddArray(pEcApiSlv, OBD_TOUCH_PROBE_SOURCE(axis), "Touch probe source", 2, DEFTYPE_INTEGER16, 16, ACCESS_READ, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60D5: Touch probe 1 positive edge counter
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_1_POS_EDGE_CNT_INDEX(axis), "Touch probe 1 positive edge counter", DEFTYPE_UNSIGNED16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60D6: Touch probe 1 negative edge counter
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_1_NEG_EDGE_CNT_INDEX(axis), "Touch probe 1 negative edge counter", DEFTYPE_UNSIGNED16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60D7: Touch probe 2 positive edge counter
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_2_POS_EDGE_CNT_INDEX(axis), "Touch probe 2 positive edge counter", DEFTYPE_UNSIGNED16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60D8: Touch probe 2 negative edge counter
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_TOUCH_PROBE_2_NEG_EDGE_CNT_INDEX(axis), "Touch probe 2 negative edge counter", DEFTYPE_UNSIGNED16, 16, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60E0: Positive torque limit value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_POSITIVE_TORQUE_LIMIT_VALUE_INDEX(axis), "Positive torque limit value", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60E1: Negative torque limit value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_NEGATIVE_TORQUE_LIMIT_VALUE_INDEX(axis), "Negative torque limit value", DEFTYPE_UNSIGNED16, 16, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60F4: Following error actual value
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, OBD_FOLLOWING_ERROR_ACTUAL_VALUE_INDEX(axis), "Following error actual value", DEFTYPE_INTEGER32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x60FF: Target velocity
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].targetVelocityIndex.objectIndex, "Target velocity", DEFTYPE_INTEGER32, 32, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }

        //Object 0x6502: Support drive modes
        error = EC_API_SLV_CoE_odAddVariable(pEcApiSlv, pApplicationInstance->CiA402_axisData[axis].supportedDriveModesIndex.objectIndex, "Supported drive modes", DEFTYPE_UNSIGNED32, 32, ACCESS_READ | OBJACCESS_TXPDOMAPPING, NULL, NULL, NULL, NULL);
        if (error != EC_API_eERR_NONE)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
            goto Exit;
        }
    }
    error = EC_API_eERR_NONE;
Exit:
    return error;
}

void EC_SLV_APP_cia402_initAxisObjects(void* pContext_p)
{
    EC_SLV_APP_Sapplication_t*  pApp_p      = (EC_SLV_APP_Sapplication_t*)pContext_p;
    uint8_t axisIter    = 0;

    if (!pApp_p)
    {
        goto Exit;
    }

    for (axisIter = 0; axisIter < AXES_NUMBER; ++axisIter)
    {
        pApp_p->CiA402_axisData[axisIter].controlWordIndex.objectIndex              = OBD_CONTROLWORD_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].statusWordIndex.objectIndex               = OBD_STATUSWORD_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].quickStopIndex.objectIndex                = OBD_QUICKSTOP_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].shutdownIndex.objectIndex                 = OBD_SHUTDOWN_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].disableOperationIndex.objectIndex         = OBD_DISABLE_OPERATION_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].faultReactionIndex.objectIndex            = OBD_FAULT_REACTION_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].modesOfOperationIndex.objectIndex         = OBD_MODES_OF_OPERATION_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].modesOfOperationDisplayIndex.objectIndex  = OBD_MODES_OF_OPERATION_DISPLAY_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].positionActualValueIndex.objectIndex      = OBD_POSITION_ACTUAL_VALUE_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].velocityActualValueIndex.objectIndex      = OBD_VELOCITY_ACTUAL_VALUE_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].targetTorqueIndex.objectIndex             = OBD_TARGET_TORQUE_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].torqueActualValueIndex.objectIndex        = OBD_TORQUE_ACTUAL_VALUE_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].targetPositionIndex.objectIndex           = OBD_TARGET_POSITION_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].swPositionLimitIndex.objectIndex          = OBD_SW_POSITION_LIMIT_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].targetVelocityIndex.objectIndex           = OBD_TARGET_VELOCITY_INDEX(axisIter);
        pApp_p->CiA402_axisData[axisIter].supportedDriveModesIndex.objectIndex      = OBD_SUPPORTED_DRIVE_MODES_INDEX(axisIter);
    }
Exit:
    return;
}

#define EC_SLV_APP_CoE_MAPSDO(error, axisObjectEntry) \
        (error) = EC_API_SLV_CoE_getObject(pApp_p->ptEcSlvApi, (axisObjectEntry).objectIndex, &((axisObjectEntry).pSdo)); \
        if (EC_API_eERR_NONE != (error)){ \
            OSAL_printf("Cannot get object at index <0x%x>\r\n", (axisObjectEntry).objectIndex); goto Exit; }

uint32_t EC_SLV_APP_CiA_fetchAxisObjects(void* pContext_p)
{
    EC_SLV_APP_Sapplication_t*  pApp_p      = (EC_SLV_APP_Sapplication_t*)pContext_p;
    uint8_t                     axisIter    = 0;
    uint32_t                    err         = EC_API_eERR_INVALID;

    if (!pApp_p)
    {
        goto Exit;
    }

    for ( axisIter = 0; axisIter < AXES_NUMBER; ++axisIter)
    {
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].controlWordIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].statusWordIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].quickStopIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].shutdownIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].disableOperationIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].faultReactionIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].modesOfOperationIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].modesOfOperationDisplayIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].positionActualValueIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].velocityActualValueIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].targetTorqueIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].torqueActualValueIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].targetPositionIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].swPositionLimitIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].targetVelocityIndex);
        EC_SLV_APP_CoE_MAPSDO(err, pApp_p->CiA402_axisData[axisIter].supportedDriveModesIndex);

        pApp_p->CiA402_axisData[axisIter].positionLimitMin.objectIndex      = pApp_p->CiA402_axisData[axisIter].swPositionLimitIndex.objectIndex;
        pApp_p->CiA402_axisData[axisIter].positionLimitMin.objectSubIndex   = 1;
        pApp_p->CiA402_axisData[axisIter].positionLimitMin.pSdo             = pApp_p->CiA402_axisData[axisIter].swPositionLimitIndex.pSdo;

        pApp_p->CiA402_axisData[axisIter].positionLimitMax.objectIndex      = pApp_p->CiA402_axisData[axisIter].swPositionLimitIndex.objectIndex;
        pApp_p->CiA402_axisData[axisIter].positionLimitMax.objectSubIndex   = 2;
        pApp_p->CiA402_axisData[axisIter].positionLimitMax.pSdo             = pApp_p->CiA402_axisData[axisIter].swPositionLimitIndex.pSdo;

        err = EC_API_SLV_CoE_getObjectEntryByObject(pApp_p->ptEcSlvApi,
                                                    pApp_p->CiA402_axisData[axisIter].positionLimitMin.pSdo,
                                                    pApp_p->CiA402_axisData[axisIter].positionLimitMin.objectSubIndex,
                                                    &pApp_p->CiA402_axisData[axisIter].positionLimitMin.pObjetEntry);
        if (EC_API_eERR_NONE != (err))
        {
            OSAL_printf("Cannot get objectEntry at index <0x%x> subindex <0x%x> \r\n",
                        pApp_p->CiA402_axisData[axisIter].positionLimitMin.objectIndex,
                        pApp_p->CiA402_axisData[axisIter].positionLimitMin.objectSubIndex );
            goto Exit;
        }

        err = EC_API_SLV_CoE_getObjectEntryByObject(pApp_p->ptEcSlvApi,
                                                    pApp_p->CiA402_axisData[axisIter].positionLimitMax.pSdo,
                                                    pApp_p->CiA402_axisData[axisIter].positionLimitMax.objectSubIndex,
                                                    &pApp_p->CiA402_axisData[axisIter].positionLimitMax.pObjetEntry);
        if (EC_API_eERR_NONE != (err))
        {
            OSAL_printf("Cannot get objectEntry at index <0x%x> subindex <0x%x> \r\n",
                        pApp_p->CiA402_axisData[axisIter].positionLimitMax.objectIndex,
                        pApp_p->CiA402_axisData[axisIter].positionLimitMax.objectSubIndex );
            goto Exit;
        }

        EC_SLV_APP_getCiA402ObjectEntryValue(pApp_p,
                                             pApp_p->CiA402_axisData[axisIter].positionLimitMin.pObjetEntry,
                                             sizeof(pApp_p->CiA402_axisData[axisIter].posLimitMin),
                                             (uint16_t *) &pApp_p->CiA402_axisData[axisIter].posLimitMin);
        EC_SLV_APP_getCiA402ObjectEntryValue(pApp_p,
                                             pApp_p->CiA402_axisData[axisIter].positionLimitMax.pObjetEntry,
                                             sizeof(pApp_p->CiA402_axisData[axisIter].posLimitMax),
                                             (uint16_t *) &pApp_p->CiA402_axisData[axisIter].posLimitMax);
    }

    err = EC_API_SLV_eERR_NO_ERROR;
Exit:
    return err;
}

#define EC_SLV_APP_CoE_GETPDOOFFSETS(axisObjectEntry) \
        if ((axisObjectEntry).pdoObject) { (axisObjectEntry).pdoOffset = EC_API_SLV_PDO_getOffset((axisObjectEntry).pdoObject) + (axisObjectEntry).pdoObjectOffset; }

uint32_t EC_SLV_APP_CiA_fetchPDOffsets(void* pContext_p)
{
    EC_SLV_APP_Sapplication_t*  pApp_p      = (EC_SLV_APP_Sapplication_t*)pContext_p;
    uint8_t                     axisIter    = 0;
    uint32_t                    err         = EC_API_eERR_INVALID;

    if (!pApp_p)
    {
        goto Exit;
    }

    for ( axisIter = 0; axisIter < AXES_NUMBER; ++axisIter)
    {
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].controlWordIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].statusWordIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].quickStopIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].shutdownIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].disableOperationIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].faultReactionIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].modesOfOperationIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].modesOfOperationDisplayIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].positionActualValueIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].velocityActualValueIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].targetTorqueIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].torqueActualValueIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].targetPositionIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].swPositionLimitIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].targetVelocityIndex);
        EC_SLV_APP_CoE_GETPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].supportedDriveModesIndex);
    }

    err = EC_API_SLV_eERR_NO_ERROR;
Exit:
    return err;
}

#define EC_SLV_APP_CoE_CLEARPDOOFFSETS(axisObjectEntry) \
        (axisObjectEntry).pdoObjectOffset = 0; (axisObjectEntry).pdoOffset = 0;

uint32_t EC_SLV_APP_CiA_dropPDOffsets(void* pContext_p)
{
    EC_SLV_APP_Sapplication_t*  pApp_p      = (EC_SLV_APP_Sapplication_t*)pContext_p;
    uint8_t                     axisIter    = 0;
    uint32_t                    err         = EC_API_eERR_INVALID;

    if (!pApp_p)
    {
        goto Exit;
    }

    for ( axisIter = 0; axisIter < AXES_NUMBER; ++axisIter)
    {
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].controlWordIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].statusWordIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].quickStopIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].shutdownIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].disableOperationIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].faultReactionIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].modesOfOperationIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].modesOfOperationDisplayIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].positionActualValueIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].velocityActualValueIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].targetTorqueIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].torqueActualValueIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].targetPositionIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].swPositionLimitIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].targetVelocityIndex);
        EC_SLV_APP_CoE_CLEARPDOOFFSETS(pApp_p->CiA402_axisData[axisIter].supportedDriveModesIndex);
    }

    err = EC_API_SLV_eERR_NO_ERROR;
Exit:
    return err;
}
