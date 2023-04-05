/*!
 *  \file CUST_phy.h
 *
 *  \brief
 *  Custom PHY interface.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-03-16
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
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

#if !(defined _CUST_PHY_H_)
#define _CUST_PHY_H_     1

#include <osal.h>

#include <board/ethphy.h>

#include <drivers/pruicss/g_v0/pruicss.h>

typedef enum CUST_PHY_EError
{
    CUST_PHY_eERR_NOERROR           = 0,    /*!< No error, everything is fine. */
    CUST_PHY_eERR_GENERALERROR      = -1    /*!< General error */
} CUST_PHY_EError_t;

typedef struct CUST_PHY_SInit
{
    uint32_t                instance_0;               /* Instance of ETHPHY0 connected to PRU-ICSS block used by stack (as defined by SysConfig) */
    uint32_t                instance_1;               /* Instance of ETHPHY1 connected to PRU-ICSS block used by stack (as defined by SysConfig) */
    OSAL_TASK_EPriority_t   taskPrioPhyMdixTask;      /* Task priority for PHY MDIX task */
}CUST_PHY_SInit_t;

typedef struct CUST_PHY_SParams
{
    PRUICSS_Config* pPruIcssCfg;    /* pointer to PRU-ICSS block configuration */
    ETHPHY_Config*  pEthPhy0Cfg;    /* pointer to ETHPHY0 configuration */
    ETHPHY_Config*  pEthPhy1Cfg;    /* pointer to ETHPHY1 configuration */
}CUST_PHY_SParams_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern uint32_t CUST_PHY_init   (CUST_PHY_SParams_t* pParams_p);
extern uint32_t CUST_PHY_deInit (void);

#if (defined __cplusplus)
}
#endif

#endif /* _CUST_PHY_H_ */
