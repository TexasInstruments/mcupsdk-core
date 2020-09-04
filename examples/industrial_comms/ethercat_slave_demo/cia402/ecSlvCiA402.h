/*!
 *  \example ecSlvCiA402.h
 *
 *  \brief
 *  CiA402 Application interface.
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

#if !(defined __ECSLVCIA402_H__)
#define __ECSLVCIA402_H__		1

#include <osal.h>
#include <ecSlvApi.h>

#include <ESL_os.h>

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/

#define AXES_NUMBER     3

typedef struct EC_SLV_APP_sCIA_object
{
    uint16_t                        objectIndex;
    EC_API_SLV_SCoE_Object_t*       pSdo;

    EC_API_SLV_SPdo_t*              pdoObject;
    uint16_t                        pdoObjectOffset;
    uint16_t                        pdoOffset;
} EC_SLV_APP_sCIA_object_t;

typedef struct EC_SLV_APP_sCIA_objectEntry
{
    uint16_t                        objectIndex;
    uint8_t                         objectSubIndex;
    EC_API_SLV_SCoE_Object_t*       pSdo;

    EC_API_SLV_SCoE_ObjEntry_t*     pObjetEntry;
} EC_SLV_APP_sCIA_objectEntry_t;

typedef struct EC_SLV_APP_sCIA_axisData
{
    /*EC_SLV_APP_sCIA_object_t   abortConnectionOptionCodeIndex; not used */
    EC_SLV_APP_sCIA_object_t    controlWordIndex;
    EC_SLV_APP_sCIA_object_t    statusWordIndex;
    EC_SLV_APP_sCIA_object_t    quickStopIndex;
    EC_SLV_APP_sCIA_object_t    shutdownIndex;
    EC_SLV_APP_sCIA_object_t    disableOperationIndex;
    /* EC_SLV_APP_sCIA_object_t   haltOptionCodeIndex; not used */
    EC_SLV_APP_sCIA_object_t    faultReactionIndex;
    EC_SLV_APP_sCIA_object_t    modesOfOperationIndex;
    EC_SLV_APP_sCIA_object_t    modesOfOperationDisplayIndex;
    /* EC_SLV_APP_sCIA_object_t   positionDemandValueIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   positionActualInternalValueIndex; not used */
    EC_SLV_APP_sCIA_object_t    positionActualValueIndex;
    /* EC_SLV_APP_sCIA_object_t   followingErrorWindowIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   followingErrorTimeoutIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   positionWindowIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   positionWindowTimeIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   velocitySensorActualValueIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   velocityDemandValueIndex; not used */
    EC_SLV_APP_sCIA_object_t    velocityActualValueIndex;
    /* EC_SLV_APP_sCIA_object_t   velocityWindowIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   velocityWindowTimeIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   velocityThresholdIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   velocityThresholdTimeIndex; not used */
    EC_SLV_APP_sCIA_object_t    targetTorqueIndex;
    /* EC_SLV_APP_sCIA_object_t   maxTorqueIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   torqueDemandIndex; not used */
    EC_SLV_APP_sCIA_object_t    torqueActualValueIndex;
    /* EC_SLV_APP_sCIA_object_t   currentActualValueIndex; not used */
    EC_SLV_APP_sCIA_object_t    targetPositionIndex;
    /* EC_SLV_APP_sCIA_object_t   positionRangeLimitIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   homeOffsetIndex; not used */
    EC_SLV_APP_sCIA_object_t    swPositionLimitIndex;
    /* EC_SLV_APP_sCIA_object_t   polarityIndex; not used */
    EC_SLV_APP_sCIA_object_t    targetVelocityIndex;
    /* EC_SLV_APP_sCIA_object_t   maxMotorSpeedIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   profileVelocityIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   profileAccelerationIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   profileDecelerationIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   quickStopDecelerationIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   motionProfileTypeIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   positionEncoderResolutionIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   gearRatioIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   feedConstantIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   velocityFactorIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   homingMethodIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   homingSpeedsIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   homingAccelerationIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   positionOffsetIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   velocityOffsetIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   torqueOffsetIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbeFunctionIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbeStatusIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbe1PosEdgeIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbe1NegEdgeIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbe2PosEdgeIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbe2NegEdgeIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   interpolationTimePeriodIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   maxAccelerationIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   maxDecelerationIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbe1PosEdgeCntIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbe1NegEdgeCntIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbe2PosEdgeCntIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   touchProbe2NegEdgeCntIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   positiveTorqueLimitValueIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   negativeTorqueLimitValueIndex; not used */
    /* EC_SLV_APP_sCIA_object_t   followingErrorActualValueIndex; not used */
    EC_SLV_APP_sCIA_object_t    supportedDriveModesIndex;

    EC_SLV_APP_sCIA_objectEntry_t   positionLimitMax;
    EC_SLV_APP_sCIA_objectEntry_t   positionLimitMin;

    uint32_t                        posLimitMax;
    uint32_t                        posLimitMin;

} EC_SLV_APP_sCIA_axisData_t;

typedef struct EC_SLV_APP_Sapplication
{
    uint32_t                        selectedPruInstance;

    /* Threads */
    TaskP_Object                    mainThreadHandle;
    TaskP_Params                    mainThreadParam;
    void*                           loopThreadHandle;

    /* Resources */
    void*                           gpioHandle;
    void*                           remoteHandle;
    void*                           ioexpLedHandle;

    int32_t                         msec,
                                    trigger;

    uint8_t                         state;
    uint8_t                         rsvd[3]; /* better be uint32_t aligned */
    clock_t                         prev, diff;

    EC_API_SLV_SCoE_Object_t*       ptRecObjOut;
    EC_API_SLV_SCoE_Object_t*       pt2002RecObj;
    EC_API_SLV_SCoE_Object_t*       pt2007RecObj;
    EC_API_SLV_SCoE_Object_t*       pt200FRecObj;

    EC_API_SLV_SPdo_t*              ptRxPdo1600;
    EC_API_SLV_SPdo_t*              ptRxPdo1601;
    EC_API_SLV_SPdo_t*              ptRxPdo1602;
    EC_API_SLV_SPdo_t*              ptTxPdo1A00;
    EC_API_SLV_SPdo_t*              ptTxPdo1A01;
    EC_API_SLV_SPdo_t*              ptTxPdo1A02;

    uint16_t                        pdoOutLen;
    uint16_t                        pdoInLen;
    uint8_t*                        pdRxBuffer;
    uint8_t*                        pdTxBuffer;

    uint16_t                        realPdoOutLen;
    uint16_t                        realPdoInLen;
    EC_SLV_APP_sCIA_axisData_t      CiA402_axisData[AXES_NUMBER];

    EC_API_SLV_SHandle_t*           ptEcSlvApi;
} EC_SLV_APP_Sapplication_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern void EC_SLV_APP_initBoardFunctions             (EC_SLV_APP_Sapplication_t*     pAppInstance_p);
extern void EC_SLV_APP_registerStacklessBoardFunctions(EC_SLV_APP_Sapplication_t*     pAppInstance_p);
extern void EC_SLV_APP_applicationInit                (EC_SLV_APP_Sapplication_t*     pAppInstance_p);
extern void EC_SLV_APP_applicationDeInit              (EC_SLV_APP_Sapplication_t*     pAppInstance_p);

#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVSIMPLE_H__ */
