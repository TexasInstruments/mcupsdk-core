/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

/**
 *  \file bootloader.h
 *
 *  \brief Bootloader Driver API/interface file.
 */

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/soc.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <drivers/bootloader/bootloader_profile.h>
#include <drivers/hw_include/soc_config.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup DRV_BOOTLOADER_MODULE APIs for BOOTLOADING CPUs
 *  \ingroup DRV_MODULE
 *
 *  See \ref DRIVERS_BOOTLOADER_PAGE for more details.
 *
 *  @{
 */

/**
 * \brief Invalid ID magic number to be used for initializations
 */
#define BOOTLOADER_INVALID_ID      (0xDEADBABE)

/**
 * \brief Operating mode type
 */
#define BOOTLOADER_OPMODE_LOCKSTEP   (0U)
#define BOOTLOADER_OPMODE_STANDALONE (1U)

/**
 * \brief Boot media IDs
 */
#define BOOTLOADER_MEDIA_MEM       (0xB0070001)
#define BOOTLOADER_MEDIA_FLASH     (0xB0070002)
#define BOOTLOADER_MEDIA_EMMC      (0xB0070003)
#define BOOTLOADER_MEDIA_SD        (0xB0070004)
#define BOOTLOADER_MEDIA_BUFIO     (0xB0070005)
#define BOOTLOADER_MEDIA_PCIE      (0xB0070006)
#define BOOTLOADER_MEDIA_USB       (0xB0070007)

/**
 * \brief Handle to the Bootloader driver returned by Bootloader_open()
 */
typedef void* Bootloader_Handle;

/**
 * \brief Parameters passed during Bootloader_open()
 */
typedef struct Bootloader_Params_s
{
    uint32_t memArgsAppImageBaseAddr;
    /* Base address of the appimage in case of a memory bootloader */

    uint8_t *bufIoTempBuf;
    /* Temporary buffer to be used if the boot media is a buffered IO */

    uint32_t bufIoTempBufSize;
    /* Size of the temporary buffer to be used if the boot media is a buffered IO */

    uint32_t bufIoDeviceIndex;
    /* Instance index of the IO device driver (UART) */

} Bootloader_Params;

/**
 * \name Bootloader driver implementation callbacks
 *
 * @{
 */

/**
 * \brief Driver implementation to open a specific bootloader driver - Memory, OSPI, UART, MMCSD etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new boot media needs to be supported.
 *
 * \param args   [in] Boot media specific arguments, obtained from the config
 * \param params [in] User controllable params
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Bootloader_imgOpenFxn)(void *args, Bootloader_Params *params);

/**
 * \brief Driver implementation to read from boot media using a specific bootloader driver - Memory, OSPI, UART, MMCSD etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new boot media needs to be supported.
 *
 * \param dstAddr [in] Destination address to which the data is to be read into
 * \param length  [in] Length in bytes of the data to be read
 * \param args    [in] Boot media specific arguments, obtained from the config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Bootloader_imgReadFxn)(void *dstAddr, uint32_t length, void *args);

/**
 * \brief Driver implementation to get the current offset in the boot media
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new boot media needs to be supported.
 *
 * \param args    [in] Boot media specific arguments, obtained from the config
 *
 * \return Current Offset
 */
typedef uint32_t (*Bootloader_imgOffsetFxn)(void *args);

/**
 * \brief Driver implementation to read from boot media using a specific bootloader driver - Memory, OSPI, UART, MMCSD etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new boot media needs to be supported.
 *
 * \param dstAddr [in] Destination address to which the data is to be read into
 * \param length  [in] Length in bytes of the data to be read
 * \param args    [in] Boot media specific arguments, obtained from the config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef void (*Bootloader_imgSeekFxn)(uint32_t location, void *args);

/**
 * \brief Driver implementation to close a specific bootloader driver - Memory, OSPI, UART, MMCSD etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new boot media needs to be supported.
 *
 * \param handle [in] Handle returned from Bootloader_imgOpen
 * \param args   [in] Boot media specific arguments, obtained from the config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef void (*Bootloader_imgCloseFxn)(void* handle, void *args);

/**
 * \brief Driver implementation to enable a custom function for a specific bootloader driver - Memory, OSPI, UART, MMCSD etc
 *
 * Typically this callback is hidden from the end application and is implemented
 * when a new boot media needs to be supported.
 *
 * \param args   [in] Boot media specific arguments, obtained from the config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*Bootloader_imgCustomFxn)(void *args);

/** @} */


/**
 * \brief Driver implementation callbacks
 */
typedef struct Bootloader_Fxns_s
{
    Bootloader_imgOpenFxn   imgOpenFxn;
    Bootloader_imgReadFxn   imgReadFxn;
    Bootloader_imgOffsetFxn imgOffsetFxn;
    Bootloader_imgSeekFxn   imgSeekFxn;
    Bootloader_imgCloseFxn  imgCloseFxn;
    Bootloader_imgCustomFxn imgCustomFxn;
    
} Bootloader_Fxns;

/**
 * \brief Bootloader driver configuration, these are filled by SysCfg based on the boot media selected.
 */
typedef struct Bootloader_Config_s
{
    Bootloader_Fxns *fxns;
    void *args;
    uint32_t bootMedia;
    uint32_t bootImageSize;
    uint32_t coresPresentMap;
    uint8_t *scratchMemPtr;
    void *socCoreOpMode;
    /* These options are for supporting HS-FS development, should be removed
       after the boot time degradation is fixed for HS devices. */
    uint32_t isAppimageSigned;
    uint32_t disableAppImageAuth;
    /* Whether to initialize ICSS cores or not */
    uint32_t initICSSCores;

} Bootloader_Config;

#include <drivers/bootloader/bootloader_flash.h>
#include <drivers/bootloader/bootloader_mem.h>
#include <drivers/bootloader/bootloader_buf_io.h>

/**
 * \brief Data structure containing information related to a particular CPU, required for RPRC loading
 */
typedef struct Bootloader_CpuInfo_s
{
    uint32_t  cpuId;
    uint32_t  clkHz;
    uint32_t  rprcOffset;
    uintptr_t entryPoint;

} Bootloader_CpuInfo;

/**
 * \brief Data structure containing information related all CPUs, this will be filled by Bootloader_parseMultiCoreAppImage
 */
typedef struct Bootloader_BootImageInfo_s
{
    Bootloader_CpuInfo cpuInfo[CSL_CORE_ID_MAX];

} Bootloader_BootImageInfo;

/**
 * \brief Initialize Bootloader params
 *
 * \param params [out] Pointer to a \ref Bootloader_Params structure
 */
void Bootloader_Params_init(Bootloader_Params *params);

/**
 * \brief Initialize BootImage info
 *
 * \param bootImageInfo [out] Pointer to a \ref Bootloader_BootImageInfo structure
 */
void Bootloader_BootImageInfo_init(Bootloader_BootImageInfo *bootImageInfo);

/**
 * \brief Initialize CPU info
 *
 * \param cpuInfo [out] Pointer to a \ref Bootloader_CpuInfo structure
 */
void Bootloader_CpuInfo_init(Bootloader_CpuInfo *cpuInfo);

/**
 * \brief Open bootloader driver
 *
 * Make sure the SOC periheral driver is open'ed before calling this API.
 * Drivers_open function generated by SysCfg opens the underlying SOC peripheral driver, e.g OSPI.
 *
 * Global variables `Bootloader_Config gBootloaderConfig[]` and `uint32_t gBootloaderConfigNum` is instantiated by SysCfg
 * to describe the boot media and other configuration based on user selection in SysCfg.
 *
 * \param instanceNum [in] Index within `Bootloader_Config gBootloaderConfig[]` denoting the bootloader driver to open
 * \param openParams  [in] Open parameters
 *
 * \return Handle to bootloader driver which should be used in subsequent API call
 * \return NULL in case of failure
 */
Bootloader_Handle Bootloader_open(uint32_t instanceNum, Bootloader_Params *openParams);

/**
 * \brief Close bootloader driver
 *
 * \param handle [in] Bootloader driver handle from \ref Bootloader_open
 */
void Bootloader_close(Bootloader_Handle handle);

/**
 * \brief API to initialize a non-self CPU
 *
 * This API will initialize a non-self CPU, i.e a CPU on which the bootloader application is not running. This API
 * is not applicable for cores from self cluster. They will be loaded by the \ref Bootloader_loadSelfCpu API.
 *
 * NOTE: No checks are done to confirm non-self CPU ID is passed, user need to make sure non-self CPU ID is passed, else
 *       the load could fail.
 *
 * \param handle  [in] Bootloader driver handle from \ref Bootloader_open
 * \param cpuInfo [in] Data structure containing information regarding the CPU. This should have been filled
 *                     by the \ref Bootloader_parseMultiCoreAppImage API
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_initCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo);

/**
 * \brief API to load a non-self CPU
 *
 * This API will load RPRC images on a non-self CPU, i.e a CPU on which the bootloader application is not running. This API
 * is not applicable for cores from self cluster. They will be loaded by the \ref Bootloader_loadSelfCpu API.
 *
 * NOTE: No checks are done to confirm non-self CPU ID is passed, user need to make sure non-self CPU ID is passed, else
 *       the load could fail.
 *
 * \param handle  [in] Bootloader driver handle from \ref Bootloader_open
 * \param cpuInfo [in] Data structure containing information regarding the CPU. This should have been filled
 *                     by the \ref Bootloader_parseMultiCoreAppImage API
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_loadCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo);

/**
 * \brief API to load self CPU
 *
 * This API will load RPRC images on self CPU, i.e a CPU on which the bootloader application is running
 *
 * NOTE: No checks are done to confirm self CPU ID is passed, user need to make sure self CPU ID is passed, else
 *       the load could fail.
 *
 * \param handle       [in] Bootloader driver handle from \ref Bootloader_open
 * \param cpuInfo      [in] Data structure containing information regarding the CPU. This should have been filled
 *                     by the \ref Bootloader_parseMultiCoreAppImage API
 * \param skipLoad     [in] Skip the image load, only do the initialization
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_loadSelfCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo, uint32_t skipLoad);

/**
 * \brief API to run a non-self CPU
 *
 * This API will run a non-self CPU, i.e a CPU on which the bootloader application is not running. This API
 * is not applicable for cores from self cluster. They will be run by the \ref Bootloader_runSelfCpu API. It is
 * expected that this API be called after \ref Bootloader_loadCpu API
 *
 * \param handle  [in] Bootloader driver handle from \ref Bootloader_open
 * \param cpuInfo [in] Data structure containing information regarding the CPU. This should have been filled
 *                     by the \ref Bootloader_parseMultiCoreAppImage API
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_runCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo);

/**
 * \brief API to boot self CPU
 *
 * This API will boot self CPU, i.e a CPU on which the bootloader application is running. It is
 * expected that this API be called after \ref Bootloader_loadSelfCpu API
 *
 * \param handle        [in] Bootloader driver handle from \ref Bootloader_open
 * \param bootImageInfo [in] Data structure of type Bootloader_BootImageInfo containing information regarding self CPU. This
 *                           should have been filled by the \ref Bootloader_parseMultiCoreAppImage API
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_runSelfCpu(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo);

/**
 * \brief API to boot a non-self CPU
 *
 * This API will boot a non-self CPU, i.e a CPU on which the bootloader application is not running. Now if the
 * self CPU is a dual core CPU, like a Cortex R5, this API is not applicable for the second core of the self CPU.
 * That will be booted by the \ref Bootloader_bootSelfCpu API.
 *
 * \param handle  [in] Bootloader driver handle from \ref Bootloader_open
 * \param cpuInfo [in] Data structure containing information regarding the CPU. This should have been filled
 *                     by the \ref Bootloader_parseMultiCoreAppImage API
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_bootCpu(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo);

/**
 * \brief API to boot self CPU
 *
 * This API will boot self CPU, i.e a CPU on which the bootloader application is running
 *
 * \param handle        [in] Bootloader driver handle from \ref Bootloader_open
 * \param bootImageInfo [in] Data structure of type Bootloader_BootImageInfo containing information regarding self CPU. This
 *                           should have been filled by the \ref Bootloader_parseMultiCoreAppImage API
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_bootSelfCpu(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo);

/**
 * \brief Parse Multicore Appimage
 *
 * When the booting is done through some boot media, unlike loading via CCS, the application binaries for each core
 * applicable are converted into a file format '.rprc' and combined together into a multicore appimage binary. The
 * bootloader needs to read this appimage, and load the binaries correctly into memories. This API helps in parsing
 * the multicore appimage and filling the metadata in the bootImageInfo structure passed.
 *
 * \param handle        [in] Bootloader driver handle from \ref Bootloader_open
 * \param bootImageInfo [in] Data structure of type Bootloader_BootImageInfo which will be filled
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_parseMultiCoreAppImage(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo);

/**
 * \brief Set Application entry point for self CPU in the AM65x SOC from reset
 *
 * \param bootImageInfo  [out] Pointer to a \ref Bootloader_BootImageInfo structure
 * \param bDualSelfR5F  TRUE (1U) if dual cores enabled, FALSE (0U) if not.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_cpuSetAppEntryPoint(Bootloader_BootImageInfo *bootImageInfo, uint32_t bDualSelfR5F);

/**
 * \brief Parse entrypoint from RPRC
 *
 * This API reads the RPRC image to parse the entry points of a particular CPU
 */
int32_t Bootloader_rprcImageParseEntryPoint(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo);

/**
 * \brief Load application binaries into SOC memory
 *
 * This API loads the application binaries for each core applicable is loaded from the boot media to the SOC memory
 */
int32_t Bootloader_rprcImageLoad(Bootloader_Handle handle, Bootloader_CpuInfo *cpuInfo);

/**
 * @brief Parse and load linux Appimage, containing linux binaries(ATF, OPTEE, SPL)
 *
 * @param handle        [in] Bootloader driver handle from \ref Bootloader_open
 * @param bootImageInfo [in] Data structure of type Bootloader_BootImageInfo which will be filled
 *
 * @return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_parseAndLoadLinuxAppImage(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo);

/**
 * \brief API to boot self CPU with Linux
 *
 * This API will boot self CPU, i.e a CPU on which the bootloader application is running. It is
 * expected that this API be called after \ref Bootloader_loadSelfCpu API. This API does not perform a security handoff
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_runSelfCpuWithLinux(void);

/**
 * \brief API to fetch the size of the multicore image
 *
 * This API simply returns the internal global variable which gets updated everytime a section is loaded.
 * So this API should only be called after all the rprc loading is complete.
 *
 * \param handle Bootloader driver handle from \ref Bootloader_open
 *
 * \return Size of the multicore image size
 */
uint32_t Bootloader_getMulticoreImageSize(Bootloader_Handle handle);

/**
 * \brief API to check if a particular core's RPRC image is present in the multicore image
 *
 * This API checks the csl core id against an internally maintained bitmap which will be populated
 * after parsing the image. So this API should be called only after the multicore image is parsed.
 *
 * \param handle Bootloader driver handle from \ref Bootloader_open
 *
 * \param cslCoreId CSL core ID of the interested core
 *
 * \return TRUE if the given core's rprc is present in the multicore image
 */
uint32_t Bootloader_isCorePresent(Bootloader_Handle handle, uint32_t cslCoreId);

/**
 * \brief API to get the selected boot media in the bootloader instance
 *
 * This API returns the selected boot media. This data will be filled by sysconfig
 *
 * \param handle [in] Bootloader driver handle from \ref Bootloader_open
 *
 * \return Boot media ID of the selected media
 */
uint32_t Bootloader_getBootMedia(Bootloader_Handle handle);

/**
 * \brief API to parse and load MCELF image
 *
 * This API parses the MCELF file and loads the loadable segments into xthe respective cores.
 *
 * \param handle Bootloader driver handle from \ref Bootloader_open
 * \param bootImageInfo [in] Data structure of type Bootloader_BootImageInfo which will be filled
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_parseAndLoadMultiCoreELF(Bootloader_Handle handle, Bootloader_BootImageInfo *bootImageInfo);

/**
 * \brief API to get the length of an x509 certificate
 *
 * This API calculates the length of an x509 certificate from the 4 byte x509 header present at the top.
 *
 * \param x509_cert_ptr Pointer to the 4 byte x509 header present at the start of the certificate.
 *
 * \return Length of the certificate
 */
uint32_t Bootloader_getX509CertLen(uint8_t *x509_cert_ptr);

/**
 * \brief API to get image length from a x509 certificate
 *
 * This API calculates the length of an image from the x509 certificate.
 *
 * \param x509_cert_ptr Pointer to the x509 certificate.
 * \param x509_cert_size Length of the x509 certificate.
 *
 * \return Length of the image 
 */
uint32_t Bootloader_getMsgLen(uint8_t *x509_cert_ptr, uint32_t x509_cert_size);
/** @} */

#ifdef __cplusplus
}
#endif

#endif
