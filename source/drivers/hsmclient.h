/*
 *  Copyright (C) 2022-24 Texas Instruments Incorporated
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
#define LABEL_AND_CONTEXT_LEN_MAX  48U

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
 * of eFuse row index and row data corresponding to it .
 */
typedef struct EfuseRead_t_
{
    uint32_t rowData ;     /** Points to data retrieved from gp otp registers.*/
    uint8_t  rowIdx ;      /** Points index of eFuse row to be read.*/
    uint8_t  rsvd[3];      /** Reserved **/
} EfuseRead_t;

/**
 * @brief
 * This is a EfuseRowWrite type which holds the information
 * regarding programming eFuse row.
 */
typedef struct EfuseRowWrite_t_
{
    uint32_t  rowData ;           /** Data to be written in eFuse row **/
    uint32_t  rowBitMask ;        /** Bit mask to apply to eFuse row data bits **/
    uint8_t   rowIdx ;            /** Index of eFuse row. **/
    uint8_t   rsvd[3];            /** Reserved **/
} EfuseRowWrite_t;

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
 * This is a EfuseRowProt type which holds the information
 * of eFuse row index and protection status corresponding
 * to the row index.
 */
typedef struct EfuseRowProt_t_
{
    uint8_t  rowidx ;         /** Index of eFuse row. **/
    uint8_t  readProt ;       /** Read row protection information used in getting or setting an eFuse row protection **/
    uint8_t  writeProt ;      /** Write row protection information used in getting or setting an eFuse row protection **/
    uint8_t  rsvd[1];         /** Reserved **/
} EfuseRowProt_t ;

/**
 * @brief
 * This is a keywriter_cert_header type which holds the information
 * of customer key certificate and debug responce.
 */
typedef struct keywriter_cert_header_t_
{
    uint8_t* certAddress;   /*For holding cerificate address*/
    uint32_t certSize;      /*Cerificate size*/
    uint32_t debugResponse; /*Debug response*/
    uint32_t reserved;      /*reserved for future use*/
} KeyWriterCertHeader_t;

/**
 * @brief
 * This is a FirewallRegionReq type which holds the information
 * of Firewall region configuration.
 */
typedef struct FirewallRegionReq_t_
{
    uint16_t  firewallId ;             /**< Index of the firewall. **/
    uint16_t  region ;                 /**< Region number **/
    uint32_t  permissionAttributes ;   /**< Memory Protection Permission Attributes corresponding to the firewall **/
    uint32_t  startAddress;            /**< Start Address of the region **/
    uint32_t  endAddress;              /**< End Address of the region **/
} FirewallRegionReq_t ;

/**
 * @brief
 * This is a FirewallReq_t type which holds the information
 * of Firewall configuration.
 */
typedef struct FirewallReq_t_
{
    uint16_t regionCount;                             /**< Region count **/
    uint16_t crcArr;                                  /**< crc of FirewallRegionArr **/
    FirewallRegionReq_t* FirewallRegionArr;           /**< Array containing set firewall region request **/
    uint16_t statusFirewallRegionArr;                 /**< Status of all region requests **/
}FirewallReq_t ;

/**
 * @brief
 * This is a FirewallIntrReq type which holds the information
 * of MPU Firewall request for interrupt enable, interrupt enable clear,
 * interrupt enable status clear and fault clear.
 */
typedef struct FirewallIntrReq_t_
{
    uint16_t  firewallId ;                  /**< Index of the firewall. **/
    uint8_t   interruptEnable;              /**< MPU Interrupt Enable **/
    uint8_t   interruptEnableClear;         /**< Clear MPU Interrupt **/
    uint8_t   interruptEnableStatusClear;   /**< Clear raw status **/
    uint8_t   faultClear;                   /**< Clear voilation status MMRs **/
} FirewallIntrReq_t ;

/**
 * @brief
 * This is SWRev type which holds the information
 * regarding Revision identifier and value corresponding to it .
 *
 * @param revValue  stores software revision value retreived from sec manager.
 * @param revId    revision identifier
 * @param rsvd     reserved for future use.
 */
typedef struct SWRev_t_
{
    uint32_t revValue; /**< Value for sw rev retreived from sec manager.*/
    uint8_t  revId;      /**< revision identifier*/
    uint8_t  rsvd[3];     /**< Reserved */
} SWRev_t;

/**
 * @brief
 * This is DKEK type which holds the label and context for derivation.
 * This also holds the 256 derived KEK value which is returned by TIFS.
 *
 * @param label_length		length of label.
 * @param context_length	length of context
 * @param label_and_context holds the label and context as an array
 * @param dkek				holds the derived key as returned by TIFS.
 */
typedef struct DKEK_t_
{
    uint8_t label_length; /**< label length.*/
    uint8_t context_length; /**< context length.*/
    uint8_t label_and_context[LABEL_AND_CONTEXT_LEN_MAX]; /**< label_and_context array */
    uint32_t dkek[8]; /**< derived KEK. */
} DKEK_t;

/**
 * @brief
 * This is RNG type which holds the resultPtr for derivation which is returned by TIFS.
 * This also holds the resultLengthPtr and DRBG Mode along with seedValue and seedSize.
 *
 * @param resultPtr		    Pointer to the random number generated
 * @param resultLengthPtr	Pointer to store the desired length in bytes
 * @param DRBGMode          Flag that determines whether DRBG mode is required or not
 * @param seedValue			Stores the seed values
 * @param seedSizeInDWords   Stores the seed size in double words
 */
typedef struct RNGReq_t_
{
    uint8_t* resultPtr; /**< Pointer to the random number.*/
    uint32_t* resultLengthPtr; /**< Pointer to determine result length.*/
    uint8_t DRBGMode; /**< Flag to enable DRBG Mode.*/
    uint32_t* seedValue; /**< Seed Value.*/
    uint8_t seedSizeInDWords; /**< Seed Size in double words.*/
    uint8_t reserved; /**< Reserved Variable.*/
} RNGReq_t;

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
 * @param verId             [OUT] populates HsmVer_t struct which describes current version. This object's memory address needs to be cache aligned.
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
 *  The service issued to HSM Server verifies the certificate and
 *  imports the keys from the certificate
 *
 * @param timeout           [IN] amount of time to block waiting for
 * semaphore to be available, in units of system ticks (see KERNEL_DPL_CLOCK_PAGE)
 * @param HsmClient         [IN] Client object which is using this importKeyring API.
 * @param cert              [IN] point to the location of certificate in the device memory.
 * @param cert_size         [IN] size of certificate.
 *
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 * 3. SystemP_TIMEOUT if timeout exception occours.
 */
int32_t HsmClient_importKeyring(HsmClient_t* HsmClient,
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
 *  The service issued to HSM Server writes the data to extended
 *  OTP efuse row based on row index provided as param.
 *
 * @param HsmClient  [IN] HsmClient object.
 * @param writeRow   [IN] populates EfuseRowWrite_t struct with rowData
 *                       corresponding to rowIdx.
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_writeOTPRow(HsmClient_t* HsmClient,
                                        EfuseRowWrite_t* writeRow);

/**
 * @brief
 *  The service issued to HSM Server sets the protection status bit of
 *  the specified row to 1.
 *
 * @param HsmClient [IN] HsmClient object.
 * @param rowProt   [IN] Pointer to EfuseRowProt_t struct which contains
 *                       the row index and row protection status
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_lockOTPRow(HsmClient_t* HsmClient,
                                        EfuseRowProt_t* rowProt);

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
 *  The service issued to HSM Server retrieves the extended otp efuse row
 *  protection status
 *
 * @param HsmClient [IN] HsmClient object.
 * @param rowProt  [IN]  Pointer to EfuseRowProt_t struct which is
 *                       populated by HSM server with row protection status
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_getOTPRowProtection(HsmClient_t* HsmClient,
                                        EfuseRowProt_t* rowProt);

/**
 * @brief
 *  The service issued to HSM Server helps with extended secure boot for
 *  applications.
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
int32_t HsmClient_procAuthBoot(HsmClient_t* HsmClient,
                                        uint8_t* cert,
                                        uint32_t cert_size,
                                        uint32_t timeout);

/**
 * @brief
 *  The service issued to HSM Server sets the firewall for the given firewall id and
 *  region.
 *
 * @param timeout           [IN] amount of time to block waiting for
 * semaphore to be available, in units of system ticks (see KERNEL_DPL_CLOCK_PAGE)
 * @param HsmClient              [IN] HsmClient object.
 * @param FirewallReqObj         [IN] Pointer to FirewallReq_t struct which contains
 *                                    information required for HSM to process set firewall request.
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 * 3. SystemP_TIMEOUT if timeout exception occours.
 */
int32_t HsmClient_setFirewall(HsmClient_t* HsmClient,
                                        FirewallReq_t* FirewallReqObj,
                                        uint32_t timeout);
/**
 * @brief
 *  The service issued to HSM Server sets the firewall interrupt request for the given firewall id.
 *
 * @param timeout           [IN] amount of time to block waiting for
 * semaphore to be available, in units of system ticks (see KERNEL_DPL_CLOCK_PAGE)
 * @param HsmClient                  [IN] HsmClient object.
 * @param FirewallIntrReqObj         [IN] Pointer to FirewallIntrReq_t struct which contains
 *                                    information required for HSM to process firewall interrupt
 *                                    request.
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 * 3. SystemP_TIMEOUT if timeout exception occours.
 */
int32_t HsmClient_FirewallIntr(HsmClient_t* HsmClient,
                                        FirewallIntrReq_t* FirewallIntrReqObj,
                                        uint32_t timeout);

/**
 * @brief
 * The service issued to HSM Server verifies the certificate and process the keywriter operations,
 * @param HsmClient         [IN] Client object which is using this API.
 * @param certHeader        [IN] point to the location of certificate in the device memory.  This object's memory address needs to be cache aligned.
 * @param timeout           [IN] amount of time to block waiting for
 * semaphore to be available, in units of system ticks (see KERNEL_DPL_CLOCK_PAGE)
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 * 3. SystemP_TIMEOUT if timeout exception occours.
 */
int32_t HsmClient_keyWriter(HsmClient_t* HsmClient,
                                        KeyWriterCertHeader_t* certHeader,
                                        uint32_t timeout);

/**
 * @brief
 *  The service issued to HSM Server retrieves the SWRevision value
 *  based on identifier as param.
 *
 * @param HsmClient [IN] HsmClient object.
 * @param readSWRev [IN] populates SWRev_t struct with SWRev value
 *                       corresponding to identifier.
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_readSWRev(HsmClient_t* HsmClient,
                                        SWRev_t* readSWRev);

/**
 * @brief
 *  The service issued to HSM Server writes the SWRevision value
 *  based on identifier as param.
 *
 * @param HsmClient [IN] HsmClient object.
 * @param writeSWRev [IN] updates the SWRev efuses with SWRev value
 *                       corresponding to identifier.
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_writeSWRev(HsmClient_t* HsmClient,
                                        SWRev_t* writeSWRev);

/**
 * @brief
 *  The service issued to HSM Server retrieves the derived KEK
 *  based on identifier as param.
 *
 *  The service issued to HSM Server retrieves the derived KEK.
 *
 * @param timeout           [IN] amount of time to block waiting for
 * semaphore to be available, in units of system ticks (see KERNEL_DPL_CLOCK_PAGE)
 * @param HsmClient         [IN] HsmClient object.
 * @param getDKEK           [IN] Pointer to DKEK_t which contains the request
 *								 structure for Derived KEK.
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_getDKEK(HsmClient_t* HsmClient,
                                        DKEK_t* getDKEK,
                                        uint32_t timeout);
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
 *  @param gHSMClient         [IN]  Pointer to registered HSM Client
 *
 * @param pHSMRt_firmware     [IN]  Pointer to signed HSMRt binary
 *
 *  @return SystemP_SUCCESS on success, else SystemP_FAILURE
 *
 */
int32_t Hsmclient_loadHSMRtFirmware(HsmClient_t *gHSMClient, const uint8_t *pHSMRt_firmware);

/**
 *  @brief  Returns the Random Number Generated.
 *
 *  @param  HsmClient            [IN] HsmClient object
 *  @param getRandomNum          [IN] Pointer to RNGReq_t which contains the request
 *								 structure for Random Number Generated.
 *
 * @return
 * 1. SystemP_SUCCESS if returns successfully
 * 2. SystemP_FAILURE if NACK message is received or client id not registered.
 */
int32_t HsmClient_getRandomNum(HsmClient_t* HsmClient,
                                        RNGReq_t* getRandomNum);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* HSM_CLIENT_H_ */
