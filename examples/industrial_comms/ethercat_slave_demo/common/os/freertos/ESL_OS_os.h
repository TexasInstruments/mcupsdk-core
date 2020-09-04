/*!
 *  \file ESL_OS_os.h
 *
 *  \brief
 *  Application OS support FreeRTOS
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

#if !(defined __ESL_OS_OS_H__)
#define __ESL_OS_OS_H__		1

#include <drivers/soc.h>
#include <drivers/pinmux.h>
#include <drivers/pruicss.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>

#include <string.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"

#define FBTL_GENERAL_STACK_SIZE_BYTE    (0x1000U)

#define ETHERCAT_SOC_MODULES_END        (0xFFFFFFFFu)
#define BOARD_I2C_LED_INSTANCE          (1U)
#define BOARD_I2C_LED_ADDR              (0x60U)
#define BOARD_I2C_IOEXPANDER_INSTANCE   (1U)
#define BOARD_I2C_IOEXPANDER_ADDR       (0x22U)

#define MAIN_TASK_SIZE_BYTE             (0x2000U)
#define APPLLOOP_TASK_SIZE_BYTE         (0x800U)

#if ((defined FBTLPROVIDER) && (1==FBTLPROVIDER)) || ((defined FBTL_REMOTE) && (1==FBTL_REMOTE))
#define FBTLCYCLIC_TASK_SIZE_BYTE       FBTL_GENERAL_STACK_SIZE_BYTE
#define FBTLSENDACYC_TASK_SIZE_BYTE     FBTL_GENERAL_STACK_SIZE_BYTE
#define FBTLACYCIST_TASK_SIZE_BYTE      FBTL_GENERAL_STACK_SIZE_BYTE
#define FBTLSYNCEXEC_TASK_SIZE_BYTE     FBTL_GENERAL_STACK_SIZE_BYTE
#define FBTLRECEIVER_TASK_SIZE_BYTE     FBTL_GENERAL_STACK_SIZE_BYTE
#define FBTLSERVICE_TASK_SIZE_BYTE      FBTL_GENERAL_STACK_SIZE_BYTE
#define FBTLSLOWSERVICE_TASK_SIZE_BYTE  FBTL_GENERAL_STACK_SIZE_BYTE
#endif

#if (defined FBTL_REMOTE) && (1==FBTL_REMOTE)
#define FBTLSYNCIST_TASK_SIZE_BYTE      FBTL_GENERAL_STACK_SIZE_BYTE
#define FBTLLEDIST_TASK_SIZE_BYTE       FBTL_GENERAL_STACK_SIZE_BYTE
#endif

#if (defined DPRAM_REMOTE)
#define APPRWRAP_TASK_SIZE_BYTE         (0x800U)
#endif

#define MAIN_TASK_SIZE                  (MAIN_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#define APPLLOOP_TASK_SIZE              (APPLLOOP_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))

typedef struct ESL_GPIO_STestPinCfg
{
    uint32_t baseAddress;
    uint32_t pinNumber;
    uint32_t pinDirection;
} ESL_GPIO_STestPinCfg_t;

#if ((defined FBTLPROVIDER) && (1==FBTLPROVIDER)) || ((defined FBTL_REMOTE) && (1==FBTL_REMOTE))
#define FBTLCYCLIC_TASK_SIZE            (FBTLCYCLIC_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#define FBTLSENDACYC_TASK_SIZE          (FBTLSENDACYC_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#define FBTLACYCIST_TASK_SIZE           (FBTLACYCIST_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#define FBTLSYNCEXEC_TASK_SIZE          (FBTLSYNCEXEC_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#define FBTLRECEIVER_TASK_SIZE          (FBTLRECEIVER_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#define FBTLSERVICE_TASK_SIZE           (FBTLSERVICE_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#define FBTLSLOWSERVICE_TASK_SIZE       (FBTLSLOWSERVICE_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#endif

#if (defined FBTL_REMOTE) && (1==FBTL_REMOTE)
#define FBTLSYNCIST_TASK_SIZE           (FBTLSYNCIST_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#define FBTLLEDIST_TASK_SIZE            (FBTLLEDIST_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#endif

#if (defined DPRAM_REMOTE)
#define APPRWRAP_TASK_SIZE              (APPRWRAP_TASK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))
#endif

extern StackType_t EC_SLV_APP_mainTaskStack_g[];
extern StackType_t EC_SLV_APP_applLoopTaskStack_g[];

#if ((defined FBTLPROVIDER) && (1==FBTLPROVIDER)) || ((defined FBTL_REMOTE) && (1==FBTL_REMOTE))
extern StackType_t SYSLIB_fbtlCyclicTaskStack_g[];
extern StackType_t SYSLIB_fbtlSendAcycTaskStack_g[];
extern StackType_t SYSLIB_fbtlAcycISTTaskStack_g[];
extern StackType_t SYSLIB_fbtlSyncExecTaskStack_g[];
extern StackType_t SYSLIB_fbtlReceiverTaskStack_g[];
extern StackType_t SYSLIB_fbtlServiceTaskStack_g[];
extern StackType_t SYSLIB_fbtlSlowServiceTaskStack_g[];
#endif

#if (defined FBTL_REMOTE) && (1==FBTL_REMOTE)
extern StackType_t SYSLIB_fbtlSyncIstTaskStack_g[];
extern StackType_t SYSLIB_fbtlLedIstTaskStack_g[];
#endif

#if (defined DPRAM_REMOTE)
extern StackType_t EC_SLV_APP_appRunWrapTaskStack_g[];
#endif

#if (defined __cplusplus)
extern "C" {
#endif

extern void Board_initPruss(uint32_t pruSelect, int32_t* baseIrqOffset);

#if (defined __cplusplus)
}
#endif

#endif /* __ESL_OS_OS_H__ */
