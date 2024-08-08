
/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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

#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/bootloader.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <security/security_common/drivers/hsmclient/soc/am263px/hsmRtImg.h> /* hsmRt bin   header file */
#include <drivers/ospi.h>
#include <drivers/fss.h>



#define MAX_HSMRT_SIZE_IN_BYTES (248 * 1024U)

const uint8_t gHsmRtFw[MAX_HSMRT_SIZE_IN_BYTES] __attribute__((section(".rodata.hsmrt")));

extern HsmClient_t gHSMClient ;

/**
 * @brief Reset that flash to start from known default state.
 * 
 * @param oHandle OSPI handle
 */
void flashFixUpOspiBoot(OSPI_Handle oHandle);

/**
 * @brief Does the actual reset of the flash on board
 * 
 */
void board_flash_reset(void);

/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while (loop)
        ;
}

/*  this API is a weak function definition for keyring_init function
    which is defined in generated files if keyring module is enabled
    in syscfg
*/
__attribute__((weak)) int32_t Keyring_init(HsmClient_t *gHSMClient)
{
    return SystemP_SUCCESS;
}

uint32_t get_Hsmrt_size() 
{
    uint8_t x509Header[4U];
    uint8_t x509HsmrtCert[2000U];
    uint32_t hsmrt_Cert_Len = 0U;
    uint32_t hsmrt_image_Size = 0U;

    Flash_read(gFlashHandle[0U], HSMRT_FLASH_OFFSET, (uint8_t *) x509Header, 4);
    CacheP_wb((void *)x509Header, 4, CacheP_TYPE_ALL);

    hsmrt_Cert_Len = Bootloader_getX509CertLen(x509Header);

    Flash_read(gFlashHandle[0U], HSMRT_FLASH_OFFSET, (uint8_t *) x509HsmrtCert, hsmrt_Cert_Len);
    CacheP_wb((void *)x509Header, hsmrt_Cert_Len, CacheP_TYPE_ALL);

    hsmrt_image_Size = Bootloader_getMsgLen(x509HsmrtCert, hsmrt_Cert_Len);

    return (hsmrt_image_Size + hsmrt_Cert_Len);
}
int main(void)
{
    int32_t status;
    uint32_t hsmrt_size = 0U;
    Bootloader_socConfigurePll();
    Bootloader_socSetAutoClock();

    System_init();
    Drivers_open();
    /* ROM doesn't reset the OSPI flash. This can make the flash initialization
    troublesome because sequences are very different in Octal DDR mode. So for a
    moment switch OSPI controller to 8D mode and do a flash reset. */
    flashFixUpOspiBoot(gOspiHandle[CONFIG_OSPI0]);
    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    /* 
        Calculate the HSM Runtime image size from the flash offset specified. 
    */
    hsmrt_size = get_Hsmrt_size();

    /* 
        Read the HSM Runtime image from the flash offset specified. 
    */
    Flash_read(gFlashHandle[0U], HSMRT_FLASH_OFFSET, (uint8_t *) gHsmRtFw, hsmrt_size);
    CacheP_wb((void *)gHsmRtFw, hsmrt_size, CacheP_TYPE_ALL);

    /* 
        Request the HSM ROM to load the HSMRT image onto itself. 
    */
    Bootloader_socLoadHsmRtFw(&gHSMClient, gHsmRtFw, hsmrt_size);
    Bootloader_socInitL2MailBoxMemory();

    /* 
        Keyring init
    */
    status = Keyring_init(&gHSMClient);
    DebugP_assert(status == SystemP_SUCCESS);

    if (SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER0, &bootParams);

        if (bootHandle != NULL)
        {
            status = Bootloader_parseAndLoadMultiCoreELF(bootHandle, &bootImageInfo);

            if (status == SystemP_SUCCESS)
            {
                /* enable Phy and Phy pipeline for XIP execution */
                if (OSPI_isPhyEnable(gOspiHandle[CONFIG_OSPI0]))
                {
                    status = OSPI_enablePhy(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);

                    status = OSPI_enablePhyPipeline(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }

            /* Run CPUs */
            if (status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
            }
            if (status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
            }
            if (status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }
            if (status == SystemP_SUCCESS)
            {
                /* Load the RPRC image on self core now */
                status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
            }

            /* it should not return here, if it does, then there was some error */
            Bootloader_close(bootHandle);
        }
    }
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("SBL failed!!\r\n");
    }
    Drivers_close();
    System_deinit();

    return 0;
}

void flashFixUpOspiBoot(OSPI_Handle oHandle)
{
    board_flash_reset();
    OSPI_enableSDR(oHandle);
    OSPI_clearDualOpCodeMode(oHandle);
    OSPI_setProtocol(oHandle, OSPI_NOR_PROTOCOL(1,1,1,0));
}
