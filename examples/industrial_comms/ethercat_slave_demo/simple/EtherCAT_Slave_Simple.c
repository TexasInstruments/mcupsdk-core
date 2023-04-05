/*!
 *  \example EtherCAT_Slave_Simple.c
 *
 *  \brief
 *  EtherCAT<sup>&reg;</sup> Slave Example Application.
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


/* this one shall the only one to be using it ! */
#include "project.h"
#include "ecSlvSimple.h"

#if (defined DPRAM_REMOTE) && (DPRAM_REMOTE==1)
 #include <DPR_Api.h>
 /* DPR implementation */
 #if (defined DPRIMPL_SHAREDRAM) && (DPRIMPL_SHAREDRAM==1)
  /* Shared RAM not TriPort */
  #include <dprLib_sharedMemory.h>
 #endif
#elif (defined FBTL_REMOTE) && (FBTL_REMOTE==1)
#if (defined FBTLIMPL_LINEUART) && (1==FBTLIMPL_LINEUART)
 #include "sysLib_lineUart.h"
#else
 #include "sysLib_sharedMemory.h"
#endif
 #include <FBTL_api.h>
#endif

/* OS */
#include <osal.h>

/* ESL */
#include <ESL_os.h>
#include <ESL_BOARD_config.h>
#include <ESL_version.h>

/* stack */
#include <ecSlvApi.h>

#define FrameProc   ECAT_FrameProcAPP
// FREERTOS contained FW #include <source/networking/ethercat_slave/icss_fwhal/ecat_frame_handler_bin.h>
#include <industrial_comms/ethercat_slave/icss_fwhal/firmware/g_v1.3/ecat_frame_handler_bin.h>
#undef FrameProc
#define HostProc   ECAT_HostProcAPP
// FREERTOS contained FW #include <source/networking/ethercat_slave/icss_fwhal/ecat_host_interface_bin.h>
#include <industrial_comms/ethercat_slave/icss_fwhal/firmware/g_v1.3/ecat_host_interface_bin.h>
#undef HostProc

#if (defined INCLUDE_MDIO_MANUAL_MODE_WORKAROUND) && (!defined SOC_AM263X)
#define PRUFirmware   PRUMDIOFirmwareECATAPP
#include <industrial_comms/ethercat_slave/icss_fwhal/firmware/g_v1.3/mdio_fw_bin.h>
#undef PRUFirmware
#endif

#define TIESC_HW	0
#define PRU_200MHZ  1

#define THREAD_IDLE_TIMEOUT   (100U)     /* 100msec idle timeout */

static OSAL_PJumpBuf_t  farJumpBuf_g;

static uint32_t EC_SLV_APP_remoteInit(EC_SLV_APP_Sapplication_t* pApplicationInstance_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Application Loop Task
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pArg_p      Application Parameter.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // required variables
 *  void* pvVariable = NULL;
 *
 *  // the Call
 *  EC_SLV_APP_loopTask(pvVariable);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
void EC_SLV_APP_loopTask(void* pArg_p)
{
    EC_SLV_APP_Sapplication_t*  pApplicationInstance    = (EC_SLV_APP_Sapplication_t*)pArg_p;
    uint32_t                    error                   = EC_API_eERR_NONE;

    if (!pApplicationInstance)
    {
        goto Exit;
    }

    EC_SLV_APP_initBoardFunctions(pApplicationInstance);
    EC_SLV_APP_registerStacklessBoardFunctions(pApplicationInstance);

#if !(defined FBTL_REMOTE) && !(defined DPRAM_REMOTE)
    /* inject firmware */
    error = EC_API_SLV_stackInsertPruFirmware((uint32_t*)ECAT_FrameProcAPP, sizeof(ECAT_FrameProcAPP),
                                              (uint32_t*)ECAT_HostProcAPP, sizeof(ECAT_HostProcAPP));
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
    }

#if (defined INCLUDE_MDIO_MANUAL_MODE_WORKAROUND) && (!defined SOC_AM263X)
    error = EC_API_SLV_stackInsertMdioManualFirmware((uint32_t*)PRUMDIOFirmwareECATAPP, sizeof(PRUMDIOFirmwareECATAPP));
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
    }
#endif
#endif

    error = EC_API_SLV_stackInit(); // EtherCAT stack init
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
    }

    EC_SLV_APP_applicationInit(pApplicationInstance);

    /* spit out versions */
    ESL_dumpVersions();

    //RAN 2021

    for(;;)
    {
        EC_API_SLV_mainLoopCyclic();

        if (EC_API_SLV_eESM_init < EC_API_SLV_getState())
        {
            OSAL_SCHED_yield();
        }
        else
        {
            /* for carve up give some air */
            OSAL_SCHED_sleep(10);
        }
    }
    // not reachable
    //EC_SLV_APP_applicationDeInit();
Exit:
    ESL_OS_taskLeave();

    return;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Main routine
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pArg_p		Application instance.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <theHeader.h>
 *
 *  // required variables
 *  uint32_t retVal = 0;
 *  void* pApp;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_mainTask(pApp);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
void EC_SLV_APP_mainTask(void* pArg_p)
{
    EC_SLV_APP_Sapplication_t*  applicationInstance = (EC_SLV_APP_Sapplication_t*)pArg_p;
    uint32_t                    retVal              = OSAL_eERR_NOERROR;

    if (!applicationInstance)
    {
        OSAL_error(__func__, __LINE__, OSAL_eERR_EINVAL, true, 1, "Application instance missing!!!\r\n");
    }

    retVal = ESL_OS_boardInit(applicationInstance->selectedPruInstance);
    if (OSAL_eERR_NOERROR != retVal)
    {
        OSAL_error(__func__, __LINE__, retVal, true, 1, "OS Board init error\r\n");
    }

    OSAL_registerPrintOut(NULL, ESL_OS_printf);

    retVal = EC_SLV_APP_remoteInit(applicationInstance);

    if (!(OSAL_CONTAINER_LOCALIMPLEMENTATION == retVal || OSAL_eERR_NOERROR == retVal))
    {
        OSAL_error(__func__, __LINE__, OSAL_eERR_EINVAL, true, 1, "Fatal error remote API init!!!\r\n");
        return;
    }

    retVal = EC_API_SLV_load(&farJumpBuf_g, NULL /* &applErrHandler*/, applicationInstance->selectedPruInstance);

    if (0 == retVal)
    {
        EC_API_SLV_prepareTasks(KBECSLV_PRIO_PDI, KBECSLV_PRIO_LED, KBECSLV_PRIO_SYNC0, KBECSLV_PRIO_SYNC1, KBECSLV_PRIO_EOE);

        applicationInstance->loopThreadHandle = OSAL_SCHED_startTask(EC_SLV_APP_loopTask
                                                                    ,applicationInstance
                                                                    ,OSAL_TASK_ePRIO_Normal
                                                                    ,(uint8_t*)EC_SLV_APP_applLoopTaskStack_g
                                                                    ,APPLLOOP_TASK_SIZE_BYTE
                                                                    ,0
                                                                    ,"Appl_LoopTask");
        if ( NULL == applicationInstance->loopThreadHandle)
        {
            OSAL_printf("Error return start Loop Task\r\n");
            OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
        }

        OSAL_SCHED_joinTask(applicationInstance->loopThreadHandle);

        /* No default tasks (run forever) here, so introduce a wait cycle */
        for (;;)
        {
            OSAL_SCHED_sleep(1000);
        }
    }
    else
    {
        for (;;)
        {

            for (;;)
            {
                OSAL_printf("long jmp restart\r\n");
                OSAL_SCHED_sleep(1000);
            }
        }
    }

    // not reachable

    //ESL_OS_boardDeinit();
    //EC_API_SLV_unLoad();

    //return 0;
}

/*!
 *  \brief
 *  Main entry point.
 *
 *  \details
 *  Simple EtherCAT Slave example demonstrating the configuration the hardware,
 *  creation of the base slave information as well as the Object Dictionary
 *  and the Process Data configuration. Furthermore, the FoE Protocol and the
 *  EEPROM handling are covered on this example.
 *
 *  \param[in]  argc                            Number of command line arguments.
 *  \param[in]  *argv                           Pointer to an array of command line arguments.
 *
 *  \return     uint32_t of type #NAMESPACE_EError_t.
 *  \retval     0                               Success.
 *  \retval     -1                              Board initialization failed.
 *
 */
int main(int argc, char *argv[])
{
    static
    uint32_t                    selectedPruInstance     = ~0;
    static
    EC_SLV_APP_Sapplication_t   applicationInstance     = {0};
    uint32_t                    error                   = OSAL_eERR_NOERROR;

    ESL_OS_init();

#if !(defined FBTL_REMOTE)
    selectedPruInstance = ESL_DEFAULT_PRUICSS;
#endif

    if (1 < argc)
    {
        char* inst = argv[1];
        selectedPruInstance = strtoul(inst, NULL, 0);
    }

    /* Init osal */
    OSAL_init();

    applicationInstance.selectedPruInstance = selectedPruInstance;

    TaskP_Params_init(&applicationInstance.mainThreadParam);

    applicationInstance.mainThreadParam.name        = "mainThread";
    applicationInstance.mainThreadParam.stackSize   = MAIN_TASK_SIZE_BYTE;
    applicationInstance.mainThreadParam.stack       = (uint8_t*)EC_SLV_APP_mainTaskStack_g;
    applicationInstance.mainThreadParam.priority    = (TaskP_PRIORITY_HIGHEST-1);
    applicationInstance.mainThreadParam.taskMain    = (TaskP_FxnMain)EC_SLV_APP_mainTask;
    applicationInstance.mainThreadParam.args        = (void*)&applicationInstance;

    error = TaskP_construct(&applicationInstance.mainThreadHandle
                           ,&applicationInstance.mainThreadParam);
    if (SystemP_SUCCESS != error)
    {
        OSAL_printf("Error setting create thread of %s (%ld)\r\n", applicationInstance.mainThreadParam.name, error);
        OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
    }

    OSAL_startOs();

    OSAL_error(__func__, __LINE__, OSAL_eERR_EINVAL, true, 1, "Not reachable by design!!!\r\n");

    // not reachable
    //EC_API_SLV_unLoad();

    return error;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initialize remote interface (FBTL)
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pApplicationInstance_p  Application instance handle
 *  \return     ErrorCode
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  // required variables
 *  uint32_t retVal = 0;
 *  EC_SLV_APP_Sapplication_t *pApplicationInstance_p;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_remoteInit(pApplicationInstance_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
static uint32_t EC_SLV_APP_remoteInit(EC_SLV_APP_Sapplication_t *pApplicationInstance_p)
{
    uint32_t retVal = OSAL_CONTAINER_LOCALIMPLEMENTATION;

    if (!pApplicationInstance_p)
    {
        goto Exit;
    }

#if (defined DPRAM_REMOTE) && (DPRAM_REMOTE==1)
    retVal = DPRLIB_init(pApplicationInstance_p->gpioHandle, false);
#elif (defined FBTL_REMOTE) && (FBTL_REMOTE==1)
#if (defined FBTLIMPL_LINEUART) && (FBTLIMPL_LINEUART==1)
    retVal = SYSLIB_createLibInstanceLine(  FBTL_LINE_UART_NAME,
                                            FBTL_MAX_ASYNC_LEN, FBTL_MAX_ASYNC_LEN,
                                            FBTL_MAX_PD_LEN, FBTL_MAX_PD_LEN
#if (defined FBTLTRACECALLS) && (FBTLTRACECALLS==1)
                                      ,true
#else
                                      ,false
#endif
                                      ,&(pApplicationInstance_p->remoteHandle)
    );
#else
    retVal = SYSLIB_createLibInstance(FBTLSHARED_MEM_NAME, FBTLSHARED_MEM_SIZE, false,
                                      FBTL_MAX_ASYNC_LEN, FBTL_MAX_ASYNC_LEN, FBTL_MAX_PD_LEN, FBTL_MAX_PD_LEN
#if (defined FBTLTRACECALLS) && (FBTLTRACECALLS==1)
                                      ,true
#else
                                      ,false
#endif
                                      ,&(pApplicationInstance_p->remoteHandle)
    );
#endif
#else
    OSALUNREF_PARM(pApplicationInstance_p);
    retVal = OSAL_CONTAINER_LOCALIMPLEMENTATION;
#endif

    if (OSAL_CONTAINER_LOCALIMPLEMENTATION == retVal)
    {
        OSAL_printf("\r\nLocal Implementation\r\n");
    }
#if (defined DPRAM_REMOTE) && (DPRAM_REMOTE==1)
    else if (0 == retVal)
    {
        OSAL_printf("\r\nDPRAM usage\r\n");

        retVal = EC_API_SLV_DPR_configuration(dprRamBase(), dprRamSize());
        if (0 != retVal)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, retVal);
            OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
            return retVal;
        }
    }
#elif (defined FBTL_REMOTE) && (FBTL_REMOTE==1)
    else if (0 == retVal)
    {
        retVal = EC_API_SLV_FBTL_configuration(pApplicationInstance_p->remoteHandle);
        if (0 != retVal)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, retVal);
            OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
            return retVal;
        }
    }
#endif

Exit:
    if (OSAL_eERR_NOERROR != retVal && OSAL_CONTAINER_LOCALIMPLEMENTATION != retVal)
    {
        OSAL_printf("%s:%d::> err ret = 0x%x\r\n", __func__, __LINE__, retVal);
    }
    return retVal;
}
