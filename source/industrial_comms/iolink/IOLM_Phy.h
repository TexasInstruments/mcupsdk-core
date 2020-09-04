/*!
 *  \example IOLM_Phy.h
 *
 *  \brief
 *  IO-Link generic Phy driver
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2020-07-15
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

#ifndef IOLM_PHY_H
#define IOLM_PHY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "IOLM_Types.h"

/* function typedefs for individual implementation */
typedef void                (*IOLM_PFU_PL_vInit)                    (INT8U u8Port_p);
typedef IOLM_EWakeupMode    (*IOLM_PFU_PL_eWakeUp)                  (INT8U u8Port_p);
typedef void                (*IOLM_PFU_PL_vSwitchPortPower)         (INT8U u8Port_p, TBOOL boOn_p);
typedef void                (*IOLM_PFU_PL_vSetMode)                 (INT8U u8Port_p, IOL_ETargetMode eTargetMode_p);
typedef void                (*IOLM_PFU_PL_vSetIQMode)               (INT8U u8Port_p, IOL_EIQMode eIQMode_p);
typedef void                (*IOLM_PFU_PL_vTransferPrepare)         (INT8U u8Port_p, const INT8U* pu8Data_p,
                                                                     INT8U u8DataLength_p, INT8U* pu8ResponseData_p,
                                                                     INT8U u8ResponseLength_p);
typedef void                (*IOLM_PFU_PL_vTransferTrig)            (INT8U u8Port_p);
typedef void                (*IOLM_PFU_PL_vSetDO)                   (INT8U u8Port_p, TBOOL boOutValue_p);
typedef TBOOL               (*IOLM_PFU_PL_boGetDI)                  (INT8U u8Port_p);
typedef void                (*IOLM_PFU_PL_vSetIQ)                   (INT8U u8Port_p, TBOOL boOutValue_p);
typedef TBOOL               (*IOLM_PFU_PL_boGetIQ)                  (INT8U u8Port_p);
typedef IOL_EBaudrate       (*IOLM_PFU_PL_eSetFHCfg)                (INT8U u8Port_p, INT8U u8PdOut_p, INT8U u8PdIn_p,
                                                                     INT8U u8ODLen_p, INT8U u8IOLCycle_p);
typedef void                (*IOLM_PFU_PL_vSetPhyCycleTimer)        (INT8U port_p, INT32U timeUs_p);

typedef enum
{
    IOLM_Phy_eType_Generic = 0,
    IOLM_Phy_eType_Atollic,
    IOLM_Phy_eType_Pru,
    IOLM_Phy_eType_MaximSpiOnly
}IOLM_Phy_EType;

/* generic phy structure */
typedef struct IOLM_SPhyGeneric
{
    IOLM_Phy_EType                  eType;
    INT8U                           u8Port;

    IOLM_PFU_PL_vInit               pfuInit;
    IOLM_PFU_PL_eWakeUp             pfuWakeup;
    IOLM_PFU_PL_vSwitchPortPower    pfuSwitchPortPower;
    IOLM_PFU_PL_vSetMode            pfuSetMode;
    IOLM_PFU_PL_vSetIQMode          pfuSetIQMode;
    IOLM_PFU_PL_vTransferPrepare    pfuTransferPrepare;
    IOLM_PFU_PL_vTransferTrig       pfuTransferTrig;
    IOLM_PFU_PL_vSetDO              pfuSetDO;
    IOLM_PFU_PL_boGetDI             pfuGetDI;
    IOLM_PFU_PL_vSetIQ              pfuSetIQ;
    IOLM_PFU_PL_boGetIQ             pfuGetIQ;
    IOLM_PFU_PL_eSetFHCfg           pfuSetFHCfg;
    IOLM_PFU_PL_vSetPhyCycleTimer   pfuSetPhyCycleTimer;
} IOLM_SPhyGeneric;

/* phy init functions */
void IOLM_Phy_Init(const IOLM_SPhyGeneric **apPhys_p);

extern const IOLM_SPhyGeneric   **ppPhys_g;

/* generic phy access functions */
extern void IOLM_Port_PL_vInit_Generic(INT8U u8Port_p);
extern void IOLM_Port_PL_vSetMode_Generic(INT8U u8Port_p, IOL_ETargetMode eTargetMode_p);
extern IOLM_EWakeupMode IOLM_Port_PL_eWakeUp_Generic(INT8U u8Port_p);
extern void IOLM_Port_PL_vSwitchPortPower_Generic(INT8U u8Port_p, TBOOL boOn_p);
extern void IOLM_Port_PL_vTransferPrepare_Generic(INT8U u8Port_p, const INT8U* pu8Data_p, INT8U u8DataLength_p,
                                                  INT8U* pu8ResponseData_p, INT8U u8ResponseLength_p);
extern void IOLM_Port_PL_vTransferTrig_Generic(INT8U u8Port_p);
extern void IOLM_Port_PL_vSetDO_Generic(INT8U u8Port_p, TBOOL boOutValue_p);
extern TBOOL IOLM_Port_PL_boGetDI_Generic(INT8U u8Port_p);
extern void IOLM_Port_PL_vSetIQMode_Generic(INT8U u8Port_p, IOL_EIQMode eIQMode_p);
extern void IOLM_Port_PL_vSetIQ_Generic(INT8U u8Port_p, TBOOL boOutValue_p);
extern TBOOL IOLM_Port_PL_boGetIQ_Generic(INT8U u8Port_p);
extern void IOLM_Port_PL_vSetPhyCycleTimer_Generic(INT8U port_p, INT32U timeUs_p);
extern IOL_EBaudrate IOLM_Port_PL_eSetFHCfg_Generic(INT8U u8Port_p, INT8U u8PdOut_p, INT8U u8PdIn_p,
                                                    INT8U u8ODLen_p, INT8U u8IOLCycle_p);

#ifdef __cplusplus
}
#endif

#endif
