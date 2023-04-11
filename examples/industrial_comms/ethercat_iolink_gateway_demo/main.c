/*!
 *  \file main.c
 *
 *  \brief
 *  Main function of the Gateway Application.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-12-08
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

#include <osal.h>              /* Operating system layer */
#include <ESL_os.h>            /* Application OS support like TaskP_Params, TaskP_Object */
#include <hwal.h>              /* Hardware Adaption support */
#include <gw_api_interface.h>  /* interface to Gateway */
#include <GWL_configuration.h>  /* interface to Gateway */

#include <FreeRTOS.h>          /* FreeRTOS operating system */
#include <ti_dpl_config.h>     /* Hardware timers */
#include <task.h>              /* FreeRTOS task API */
#include "SMIdirect_UART.h"    /* SMIdirect over UART API */

#include "nvram_driver.h"      /* application specific NVRAM driver */
#include "nvram.h"             /* hardware-agnostic NVRAM module */

#include "version.h"           /* version file */
#include "config.h"            /* configuration for a given board*/

#define MAIN_TASK_SLEEPTIME_MS          10000U
#define MAIN_TASK_PRIO                  (TaskP_PRIORITY_HIGHEST-1)
#define MAIN_TASK_STACKSIZE             (0x2000U / sizeof(configSTACK_DEPTH_TYPE))
static StackType_t mainTaskStack_s[MAIN_TASK_STACKSIZE] __attribute__((aligned(32), section (".threadstack"))) = {0};

/* Use a proper memory offest in accordance with memory pages for a given flash memory 
 * Don't use the same offset and the memory area for the EtherCAT slave EEPROM.
 */
#define NVRAM_BASE_ADR 0x400000U

#define MAIN_WITH_CONFIG_EXAMPLE    0

// examples for IOLINK devices
#define IOL_VENDOR_UNDEF            0x00
#define IOL_VENDOR_BALLUFF          0x378
#define IOL_VENDOR_SICK             0x1a
#define IOL_VENDOR_IFM              0x136
#define IOL_VENDOR_AUTOSEN          0x345
#define IOL_DEVICE_UNDEF            0x00
#define IOL_DEVICE_IFMO8H273        0x45d
#define IOL_DEVICE_SICK1076674      0x106dc2
#define IOL_DEVICE_BALLUFFSMARTLED  0x50a05
#define IOL_DEVICE_BALLUFFBIS01E6   0x60231
#define IOL_DEVICE_AUTOSENAI402     0x61
#define IOL_DEVICE_AUTOSENA0002     0x34
#define IOL_REVISIONID_UNDEF        0x00
#define IOL_CYCLE_TIME_UNDEF        0x00

// vendorID, deviceID, revisionID, inputDataLength, outputDataLength, serialNumber, portCycleTime, 
// iqBehavior, validation, masterControl
const GW_API_SPortExpConfig_t IOL_sFixedConfiguration_g[IOLM_USEDPORTS] = {
// Port 1
[0] = { IOL_VENDOR_BALLUFF, IOL_DEVICE_BALLUFFSMARTLED, IOL_REVISIONID_UNDEF, 
    0, 1, "", IOL_CYCLE_TIME_UNDEF,
    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
    GW_API_MASTERCONTROL_IOLINKPROT | GW_API_MASTERCONTROL_ACTIVE},
// Port 2  
[1] = { IOL_VENDOR_AUTOSEN, IOL_DEVICE_AUTOSENAI402, IOL_REVISIONID_UNDEF, 
    2, 0, "000010593315", IOL_CYCLE_TIME_UNDEF, 
    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
    GW_API_MASTERCONTROL_IOLINKPROT | GW_API_MASTERCONTROL_ACTIVE},
// Port 3
[2] = { IOL_VENDOR_UNDEF, IOL_DEVICE_UNDEF, IOL_REVISIONID_UNDEF, 
    0, 0, "", IOL_CYCLE_TIME_UNDEF,
    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
    GW_API_MASTERCONTROL_IOLINKPROT | GW_API_MASTERCONTROL_ACTIVE},
// Port 4
[3] = { IOL_VENDOR_UNDEF, IOL_DEVICE_UNDEF, IOL_REVISIONID_UNDEF, 
    0, 0, "", IOL_CYCLE_TIME_UNDEF,
    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
    GW_API_MASTERCONTROL_IOLINKPROT | GW_API_MASTERCONTROL_ACTIVE},
// Port 5
[4] = { IOL_VENDOR_AUTOSEN, IOL_DEVICE_AUTOSENAI402, IOL_REVISIONID_UNDEF, 
    2, 0, "000010593533", IOL_CYCLE_TIME_UNDEF,
    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
    GW_API_MASTERCONTROL_IOLINKPROT | GW_API_MASTERCONTROL_ACTIVE},
// Port 6
[5] = { IOL_VENDOR_UNDEF, IOL_DEVICE_UNDEF, IOL_REVISIONID_UNDEF, 
    0, 0, "", IOL_CYCLE_TIME_UNDEF,
    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
    GW_API_MASTERCONTROL_IOLINKPROT | GW_API_MASTERCONTROL_ACTIVE},
// Port 7
[6] = { IOL_VENDOR_UNDEF, IOL_DEVICE_UNDEF, IOL_REVISIONID_UNDEF, 
    0, 0, "", IOL_CYCLE_TIME_UNDEF,
    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
    GW_API_MASTERCONTROL_IOLINKPROT | GW_API_MASTERCONTROL_ACTIVE},
// Port 8
[7] = { IOL_VENDOR_UNDEF, IOL_DEVICE_UNDEF, IOL_REVISIONID_UNDEF, 
    0, 0, "", IOL_CYCLE_TIME_UNDEF,
    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
    GW_API_MASTERCONTROL_IOLINKPROT | GW_API_MASTERCONTROL_ACTIVE},

// example for DI
//[x] = { IOL_VENDOR_UNDEF, IOL_DEVICE_UNDEF, IOL_REVISIONID_UNDEF, 
//    1, 0, "", IOL_CYCLE_TIME_UNDEF, 
//    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
//    GW_API_MASTERCONTROL_DI | GW_API_MASTERCONTROL_ACTIVE},
// example for DO
//[x] = { IOL_VENDOR_UNDEF, IOL_DEVICE_UNDEF, IOL_REVISIONID_UNDEF, 
//    0, 1, "", IOL_CYCLE_TIME_UNDEF,
//    GW_API_IQBEHAVIOR_NOTSUPPORTED, GW_API_VALIDATION_TYPE_NONE, 
//    GW_API_MASTERCONTROL_DO | GW_API_MASTERCONTROL_ACTIVE},

};

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  initialize driver and start gateway functionality
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]       pArg_p        not used
 *
 * */
static void OSAL_FUNC_NORETURN MainTask(void* pArg_p)
{
    uint32_t error;
    const uint8_t pPortclass[IOLM_USEDPORTS] = {
        GW_API_CONF_PORTTYPES_CLASS_B,
        GW_API_CONF_PORTTYPES_CLASS_B,
        GW_API_CONF_PORTTYPES_CLASS_B,
        GW_API_CONF_PORTTYPES_CLASS_B,
        GW_API_CONF_PORTTYPES_CLASS_B,
        GW_API_CONF_PORTTYPES_CLASS_B,
        GW_API_CONF_PORTTYPES_CLASS_B,
        GW_API_CONF_PORTTYPES_CLASS_B
    };

    OSALUNREF_PARM(pArg_p);

    OSAL_TIMER_set100usecTickSupport(CONFIG_TIMER0_USEC_PER_TICK == 100U);

    Board_init();
    Drivers_open();
    error = (uint32_t)Board_driversOpen();
    if(error != SystemP_SUCCESS)
    {
        OSAL_printf("Error Board_driversOpen:  0x%08x\r\n", error);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }

    OSAL_registerPrintOut(NULL, ESL_OS_printf);

    uint32_t version = GW_API_getVersion();
    OSAL_printf("Application %s version: %d.%d.%d.%d\r\n", "Gateway_EC_IOL API", 
                        (version >> 24U) & 0xFFU, (version >> 16U) & 0xFFU,
                        (version >> 8U) & 0xFFU, version & 0xFFU);

    error = HWAL_init();
    if (error != OSAL_eERR_NOERROR)
    {
        OSAL_printf("Error HWAL_init:  0x%08x\r\n", error);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }

    // init NVRAM
    // lower flash memory limit for the file system
    struct lfs_config* plfscfg = NVR_DRV_init(CONFIG_FLASH0, NVRAM_BASE_ADR);
    if(NVR_ERR_OK != NVR_init(plfscfg))
    {
        OSAL_printf("init NVRAM: FAIL\r\n");
    } else
    {
        OSAL_printf("init NVRAM: success\r\n");
    }
    unsigned int bootcount = 0;
    NVR_bootcount("bcount.bin", &bootcount);

    // examples for gateway configuration, not needed if defaults ok
    GW_API_EErrorcode_t apiError;
    apiError = GW_API_setIOLVendorId(IOLM_VENDORID);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_API_setIOLVendorId:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_API_setIOLMasterId(IOLM_MASTERID);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_API_setIOLMasterId:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_API_setIOLFeatures1(0);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_API_setIOLFeatures1:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }

    apiError = GW_API_setIOLPortConfig(IOLM_USEDPORTS, pPortclass);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_API_setIOLPortConfig:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_API_setGWHighestPriority(OSAL_TASK_ePRIO_16);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_API_setGWHighestPriority:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_API_setIOLPRUInstance(IOLM_PRUINSTANCE);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_API_setIOLPRUInstance:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_API_setEcatPRUInstance(ECAT_PRUINSTANCE);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_API_setEcatPRUInstance:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_EC_API_setVendorId(ECAT_VENDORID);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_EC_API_setVendorId:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_EC_API_setProductCode(ECAT_PCODE);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_EC_API_setProductCode:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_EC_API_setRevision(ECAT_REVISION);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_EC_API_setRevision:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_EC_API_setSerialNumber(ECAT_SERIALNR);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_EC_API_setSerialNumber:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_EC_API_setProductName(ECAT_PNAME);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_EC_API_setProductName:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_EC_API_setHWVersion(ECAT_HWVERSION);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_EC_API_setHWVersion:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }
    apiError = GW_EC_API_setSWVersion(ECAT_SWVERSION);
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_EC_API_setSWVersion:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }

    // example for configuration via API
    #if (defined MAIN_WITH_CONFIG_EXAMPLE) && (MAIN_WITH_CONFIG_EXAMPLE==1)
    for(uint8_t index=0; index<IOLM_USEDPORTS; index++)
    {
        apiError = GW_API_setExpPortConfiguration(index+1U, &IOL_sFixedConfiguration_g[index]);
        if(GW_API_eSUCCESS != apiError)
        {
            OSAL_printf("Error GW_API_setExpPortConfiguration:  %d (port %d)\r\n", apiError, index+1U);
            // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
            //cppcheck-suppress misra-c2012-15.1
            goto laExit;
        }
    }
    #endif

// end of examples for gateway configuration

    apiError = GW_API_start();
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error GW_API_start:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }

    // start SMIdirect via UART
    apiError = SMIdirect_UART_start();
    if(GW_API_eSUCCESS != apiError)
    {
        OSAL_printf("Error SMIdirect_UART_start:  %d\r\n", apiError);
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laExit;
    }

    for (;;)
    {
        OSAL_SCHED_sleep(MAIN_TASK_SLEEPTIME_MS);
        OSAL_printf("*** MainTask triggered ***\r\n");
    }

laExit:
    OSAL_printf("Initialisation error \r\n");
    for (;;)
    {
        // shall normaly not reached
        OSAL_SCHED_sleep(MAIN_TASK_SLEEPTIME_MS);
    }
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
 *  \param[in]  argc                            not used
 *  \param[in]  *argv                           not used
 *
 *  \return     uint32_t of type #NAMESPACE_EError_t.
 *  \retval     0                               Success.
 *  \retval     -1                              Board initialization failed.
 *
 */
int main(int argc, char *argv[])
{
    uint32_t error = OSAL_eERR_NOERROR;
    TaskP_Object mainThreadHandle;    /*<! main task's handler */
    TaskP_Params mainThreadParam;     /*<! main task's parameters */

    OSALUNREF_PARM(argc);
    OSALUNREF_PARM(argv);

    /* Initialize system */
    System_init();
    OSAL_init();

    /* Create main task to initialize all the others. It must be
     * with the highest priority task */
    TaskP_Params_init(&mainThreadParam);

    mainThreadParam.name        = "mainThread";
    mainThreadParam.stackSize   = sizeof(mainTaskStack_s);
    mainThreadParam.stack       = (uint8_t *)mainTaskStack_s;
    mainThreadParam.priority    = MAIN_TASK_PRIO;
    mainThreadParam.taskMain    = (TaskP_FxnMain)MainTask;
    mainThreadParam.args        = NULL;

    error = TaskP_construct(&mainThreadHandle, &mainThreadParam);
    if (SystemP_SUCCESS == error)
    {
        OSAL_startOs();
    }

    OSAL_printf("Error setting create thread of %s (%ld)\r\n", mainThreadParam.name, error);
    OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);

    return error;
}
