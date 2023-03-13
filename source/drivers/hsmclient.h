/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HSM_CLIENT_H_
#define HSM_CLIENT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Header file for HSM client driver */
#include <stdint.h>
#include <drivers/sipc_notify.h>
#include <drivers/hsmclient/hsmclient_msg.h>
#include <drivers/hsmclient/utils/hsmclient_utils.h>
#include <kernel/dpl/SemaphoreP.h>

/**
 * \defgroup DRV_HSMCLIENT_MODULE APIs for HSMCLIENT
 * \ingroup DRV_MODULE
 *
 * See \ref DRIVERS_HSMCLIENT_PAGE for more details.
 *
 * @{
 */


/**
 * @brief
 * type for reading HSMRt version.
 *
 */
typedef union HsmVer_t_
{
    uint64_t HsmrtVer /**< A 64 bit unique Version number */ ;
    struct
    {
        uint8_t  PatchVer ; /** patch Version number refer HSMRT_PATCH_VER in TIFS Docs  */
        uint8_t  MinorVer ; /** HSMRt minor version number refer HSMRT_MINOR_VER in TIFS Docs*/
        uint8_t  MajorVer;  /** HSMRt major version number refer HSMRT_MAJOR_VER in TIFS Docs*/
        uint8_t  ApiVer  ;  /** HSMRt API version number refer HSMRT_APIS_VER in TIFS Docs*/
        uint8_t  SocType ;  /** HSMRt Soc type ID refer HSMRT_SOC_TYPE in TIFS Docs*/
        uint8_t  BinType ;  /** HSMRt Binary type ID refer HSMRT_BIN_TYPE in TIFS Docs*/
        uint8_t  HsmType ;  /** HSM Architecture version */
        uint8_t  DevType ;  /** HSMRt Device type ID  refer HSMRT_DEVICE_TYPE in TIFS Docs*/

    } VerStruct;
#if defined(_TMS320C6X)
}__attribute__((packed)) HsmVer_t_;
#else
}__attribute__((packed)) HsmVer_t;
#endif


/**
 * @brief
 * This is a HSMClient type which holds the information
 * needed by hsm client to communicate with HSM .
 */
typedef struct HsmClient_t_
{
    SemaphoreP_Object Semaphore; /**  Registered client will pend of this semaphore
                                   till it receives a response from HSM server. */
    HsmMsg_t ReqMsg ;     /** message frame that is to be passed to HSM server.*/
    HsmMsg_t RespMsg ;   /** Stores a message frame received from HSM server.*/
    uint8_t RespFlag ;  /** Indicates weather a request has been Acked or Nacked by HSM server.*/
    uint8_t ClientId ;  /** object's ClientId.*/

} HsmClient_t ;

/**
 * @brief
 * This is a EfuseRead type which holds the information
 * regarding eFuse row and row data corresponding to it .
 */
typedef struct EfuseRead_t_
{
    uint32_t rowData ;     /** Points to data retrieved from gp otp registers.*/
    uint8_t  rowIdx ;      /** Points index of eFuse row to be read.*/
    uint8_t  rsvd[3];      /** Reserved **/
} EfuseRead_t;

/**
 * @brief
 * This is a EfuseRowCount type which holds the information
 * regarding eFuse row count and size of each row in bits.
 */
typedef struct EfuseRowCount_t_
{
    uint32_t rowCount ;     /** eFuse row count **/
    uint8_t  rowSize ;      /** Size of an eFuse row in bits. **/
    uint8_t  rsvd[3];       /** Reserved **/
} EfuseRowCount_t;

/**
 * @brief
 * Initialize the HSM client for current core.
 *
 * @param params [IN] SIPC_notify params.
 *
 * @return
 * 1. SystemP_SUCCESS if init sequence successful.
 * 2. SystemP_FAILURE if init sequence fails.
 */
int32_t HsmClient_init(SIPC_Params* params);

/**
 * @brief
 * De initialize the HSM client for current core.
 *
 */
void HsmClient_deInit(void);

/**
 * @brief
 *  populates the current HSMRT version Id
 *  by default the hsm flag is set to HSM_FLAG_AOP for this service
 *
 * @param timeToWaitInTick  [IN] amount of time to block waiting for
 * semaphore to be available, in units of system ticks (see KERNEL_DPL_CLOCK_PAGE)
 * @param HsmClient         [IN] Client object which is using this getversion API.
 * @param verId             [OUT] populates HsmVer_t struct which describes current version.
 *
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 * 3. SystemP_TIMEOUT if timeout exception occours.
 */
int32_t HsmClient_getVersion(HsmClient_t *HsmClient ,
                                        HsmVer_t* verId,uint32_t timeToWaitInTick);

/**
 * @brief
 *  The service issued to HSM Server populates the Device UID
 *  by default the hsm flag is set to HSM_FLAG_AOP for this service
 *
 * @param timeout           [IN] amount of time to block waiting for
 * semaphore to be available, in units of system ticks (see KERNEL_DPL_CLOCK_PAGE)
 * @param HsmClient         [IN] Client object which is using this getUID API.
 * @param uid               [OUT] populates UID value.
 *
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 * 3. SystemP_TIMEOUT if timeout exception occours.
 */
int32_t HsmClient_getUID(HsmClient_t* HsmClient,
                                        uint8_t* uid, uint32_t timeout);

/**
 * @brief
 *  The service issued to HSM Server verifies the certificate and
 *  by default the hsm flag is set to HSM_FLAG_AOP for this service
 *
 * @param timeout           [IN] amount of time to block waiting for
 * semaphore to be available, in units of system ticks (see KERNEL_DPL_CLOCK_PAGE)
 * @param HsmClient         [IN] Client object which is using this openDbgFirewalls API.
 * @param cert              [IN] point to the location of certificate in the device memory.
 * @param cert_size         [IN] size of certificate.
 *
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 * 3. SystemP_TIMEOUT if timeout exception occours.
 */
int32_t HsmClient_openDbgFirewall(HsmClient_t* HsmClient,
                                        uint8_t* cert,
                                        uint32_t cert_size,
                                        uint32_t timeout);

/**
 * @brief
 *  The service issued to HSM Server retrieves the data of GP OTP row
 *  based on row index provided as param.
 *
 * @param HsmClient [IN] HsmClient object.
 * @param readRow   [IN] populates EfuseRead_t struct with rowData
 *                       corresponding to rowIdx.
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_readOTPRow(HsmClient_t* HsmClient,
                                        EfuseRead_t* readRow);

/**
 * @brief
 *  The service issued to HSM Server retrieves the count of extended OTP
 *  rows.
 *
 * @param HsmClient [IN] HsmClient object.
 * @param rowCount  [IN] Pointer to EfuseRowCount_t struct which is
 *                       populated by HSM server with row count and row size
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_getOTPRowCount(HsmClient_t* HsmClient,
                                        EfuseRowCount_t* rowCount);

/**
 * @brief
 * register a client to a particular ClientId
 *
 * @param HsmClient [IN] HsmClient object.
 * @param clientId [IN] enable HSM <--> R5 communication for given clientID
 *
 * @return
 * 1. SystemP_SUCCESS if clientId is available
 * 2. SystemP_FAILURE if clientId is not available or already in use.
 */
int32_t HsmClient_register(HsmClient_t* HsmClient, uint8_t clientId);

/**
 * @brief
 * unregister a client to a particular ClientId
 *
 * @param HsmClient [IN] HsmClient object
 * @param clientId [IN] disable HSM <--> R5 communication for given clientId
 *
 */
void HsmClient_unregister(HsmClient_t* HsmClient,uint8_t clientId);
/**
 * @brief
 * Current core will wait for bootnotify message from HSM core.
 *
 * @param HsmClient [IN] HsmClient object
 *
 * @param timeToWaitInTicks  [IN] amount of time to block waiting for
 * semaphore to be available, in units of SystemP_timeout if timeout exception occours.system ticks (see KERNEL_DPL_CLOCK_PAGE)
 *
 * @return
 * 1. SystemP_SUCCESS -: when BootNotify received
 * 2. SystemP_FAILURE -: if faulty msg received
 */
int32_t HsmClient_waitForBootNotify(HsmClient_t* HsmClient,uint32_t timeToWaitInTicks);

/**
 *  @brief  Loads the HSMRt firmware. This is typically called by SBL.
 *
 *  @param pHSMRt_firmware     [IN]  Pointer to signed HSMRt binary
 *
 *  @return SystemP_SUCCESS on success, else SystemP_FAILURE
 *
 */
int32_t Hsmclient_loadHSMRtFirmware(const uint8_t *pHSMRt_firmware);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* HSM_CLIENT_H_ */
