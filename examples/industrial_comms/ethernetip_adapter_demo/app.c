/*!
 *  \example app.c
 *
 *  \brief
 *  EtherNet/IP&trade; Adapter Example Application.
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
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include <api/EI_API.h>
#include <api/EI_API_def.h>

#include <osal.h>
#include <osal_error.h>
#include <hwal.h>

#include <ti_drivers_config.h>
#include <ti_board_config.h>
#include <board.h>

#include <CMN_os.h>
#include <CMN_app.h>
#include <CMN_CPU_api.h>

#include <drivers/CUST_drivers.h>

#include <appWebServer.h>
#include <app.h>

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Main entry point.
 *
 *  \details
 *  Main entry point.<br />
 *  Initializes the general operating system abstraction layer,
 *  starts the EtherNet/IP&trade; main application task, and starts the
 *  operating system abstraction layer.
 *
 */
int main(
    int argc,
    char* argv[])
{
    uint32_t err;
    APP_SInstance_t appInstance = {0};
    APP_SParams_t*  pCfg        = &appInstance.config;

    /* Application configuration. */
    pCfg->application.taskPrio = OSAL_TASK_ePRIO_EIP_MAIN;

    /* HWAL configuration. */
    pCfg->hwal.taskPrioWatchDog = OSAL_TASK_ePRIO_HWAL_WATCHDOG;    // Value change will have no impact.
    pCfg->hwal.taskPrioLicense  = OSAL_TASK_ePRIO_HWAL_LICENSE;     // Value change will have no impact.

    /* LWIP configuration */
    pCfg->lwip.taskPrio = OSAL_TASK_ePRIO_EIP_LWIP_TCPIP;  // Value change will have no impact.

    /* EtherNet/IP Adapter configuration */
    pCfg->adapter.taskPrioCyclicIo  = OSAL_TASK_ePRIO_EIP_CYCLICIO;
    pCfg->adapter.taskPrioPacket    = OSAL_TASK_ePRIO_EIP_PACKET;
    pCfg->adapter.taskPrioStatistic = OSAL_TASK_ePRIO_EIP_STATISTIC;

    pCfg->adapter.taskPrioPtpDelayRqTx   = OSAL_TASK_ePRIO_EIP_TIMESYNC_DEL;
    pCfg->adapter.taskPrioPtpTxTimeStamp = OSAL_TASK_ePRIO_EIP_TIMESYNC_TS;
    pCfg->adapter.taskPrioPtpNRT         = OSAL_TASK_ePRIO_EIP_TIMESYNC_NRT;
    pCfg->adapter.taskPrioPtpBackground  = OSAL_TASK_ePRIO_EIP_TIMESYNC_BAC;

    pCfg->adapter.taskPrioLldpReceive = OSAL_TASK_ePRIO_Normal;

    /* Custom drivers configuration - PRU-ICSS. */
    pCfg->customDrivers.pruIcss.instance                       = PRU_ICSS_BLOCK_INSTANCE;
    pCfg->customDrivers.pruIcss.ethPhy.instance_0              = PRU_ICSS_ETHPHY_0_INSTANCE;
    pCfg->customDrivers.pruIcss.ethPhy.instance_1              = PRU_ICSS_ETHPHY_1_INSTANCE;
    pCfg->customDrivers.pruIcss.ethPhy.taskPrioPhyMdixTask     = OSAL_TASK_ePRIO_EIP_PHYMDIX;

    /* Custom drivers configuration - EEPROM. */
    pCfg->customDrivers.eeprom.taskPrio = OSAL_TASK_ePRIO_EIP_EEPROM,

    /* Custom drivers configuration - FLASH. */
    pCfg->customDrivers.flash.taskPrio  = OSAL_TASK_ePRIO_EIP_FLASH,

#if (defined CPU_LOAD_MONITOR) && (1==CPU_LOAD_MONITOR)
    /* Web Server configuration. */
    pCfg->webServer.taskPrio = OSAL_TASK_ePRIO_EIP_WEBSERVER;

    /* CPU load configuration. */
    pCfg->cpuLoad.taskPrio   = OSAL_TASK_ePRIO_EIP_CPULOAD;
    pCfg->cpuLoad.output     = CMN_CPU_API_eOUT_NONE;
#endif

    CMN_OS_init ();

    err = OSAL_init ();
    if (err != OSAL_NO_ERROR)
    {
        goto laError;
    }

    OSAL_registerErrorHandler (EI_APP_osErrorHandlerCb);

#if defined(QUICK_CONNECT)
    OSAL_printfSuppress(true);
#endif

    CMN_APP_mainCreate(EI_APP_mainTask, &appInstance, pCfg->application.taskPrio);

    OSAL_startOs ();

    return (0);

//-------------------------------------------------------------------------------------------------
laError:

    return (-1);
}
