/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* This macro is used to specify if the target is booting in PCIe boot mode.
   If target is not booting from PCIe boot mode, host application need not
   send the SBL image for ROM to boot SBL */
/* If target board is not in PCIe boot mode, change the macro to 0 */
#define SBL_USE_PCIE_BOOT_MODE      (1U)

/* Base address to which SBL image can be sent to */
#define SBL_IMAGE_BASE              (0x70000000U)

/* Maximum size of SYSFW binary */
#define SBL_SYSFW_MAX_SIZE          (0x42000U)

/* "MAGIC WORD" to indicate image transfer is complete */
#define APPIMAGE_MAGIC_WORD         (0xC0DEFEED)

/* Hand shake "MAGIC WORD" to indicate SBL is ready to accept app images */
#define HANDSHAKE_MAGIC_WORD        (0xFEEDC0DE)

/* Hand shake "MAGIC WORD" to indicate SBL is ready to accept sysfw images */
#define SYSFW_MAGIC_WORD            (0xC0DEC0DE)

/* Offset to which Sysfw is sent to */
#define SYSFW_IMAGE_OFFSET          (0xE000U)

/* Offset to which Appimage is sent to */
#define APPIMAGE_OFFSET             (0x1000U)

/* Maximum size of Appimage supported */
#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE   (0x60000U)

#define BOOTIMAGE_FILENAME ("/sd0/sbl_pcie.release.tiimage")
#define SYSFW_FILENAME     ("/sd0/sysfw.bin")
#define APPIMAGE_FILENAME  ("/sd0/app")

/* Pointer to which the SBL image is send to is communicated via */
uint32_t *offsetPtr = (uint32_t *)(CONFIG_PCIE0_OB_REGION1_LOWER +0x7BFE0);

/* Variable to which the handshake from SBL is received, to start sending system firmware and appimage */
uint32_t gHandShake __attribute__ ((used, aligned(65536),section(".ghandshake")));

int main(void)
{
    int32_t status = SystemP_SUCCESS;

    System_init();
    Board_init();

    Drivers_open();
    Board_driversOpen();

    DebugP_log("Starting Host EVM application for PCIe Boot\r\n");

    /* Send SBL image to target EVM */
#if (1U ==  SBL_USE_PCIE_BOOT_MODE)
    {
        /* Open SBL file */
        FF_FILE *appFp = ff_fopen(BOOTIMAGE_FILENAME, "rb");

        if(appFp == NULL)
        {
            status =  SystemP_FAILURE;
        }
        else
        {
            uint32_t fileSize = ff_filelength(appFp);

            DebugP_log("Sending SBL image - \"%s\" of size %d\r\n", BOOTIMAGE_FILENAME, fileSize);

            /* Read and send SBL image */
            ff_fread((uint8_t *)(CONFIG_PCIE0_OB_REGION0_LOWER + CONFIG_PCIE0_OB_REGION0_UPPER), fileSize, 1, appFp);

            *offsetPtr = SBL_IMAGE_BASE ;
        }
    }

    DebugP_assert(status == SystemP_SUCCESS);
#endif

    /* Wait for handshake from SBL PCIe application to start sending SYSFW image */
    do {
        CacheP_inv(&gHandShake, 128, CacheP_TYPE_ALL);
    } while(gHandShake != SYSFW_MAGIC_WORD);

    /**************** Send SYSFW image to targert EVM *******************/
    {
        /* Open System Firmware file */
        FF_FILE *sysfwFp = ff_fopen(SYSFW_FILENAME, "rb");

        if(sysfwFp == NULL)
        {
            status =  SystemP_FAILURE;
        }
        else
        {
            uint32_t fileSize = ff_filelength(sysfwFp);

            /* Read and send Sysfw image */
            ff_fread((uint8_t *)((CONFIG_PCIE0_OB_REGION0_LOWER + CONFIG_PCIE0_OB_REGION0_UPPER)+SYSFW_IMAGE_OFFSET), fileSize, 1, sysfwFp);

            volatile uint32_t *magicWordPtr = (uint32_t *)((CONFIG_PCIE0_OB_REGION0_LOWER + SYSFW_IMAGE_OFFSET + SBL_SYSFW_MAX_SIZE - sizeof(uint32_t)));

            *magicWordPtr = SYSFW_MAGIC_WORD;
        }
    }

    DebugP_assert(status == SystemP_SUCCESS);

    /* Close the pcie driver  */
    Drivers_pcieClose();

    /* Re-establish the link between RC and EP , as board configuration changes the link state */
    Drivers_pcieOpen();

    do {
        CacheP_inv(&gHandShake, 128, CacheP_TYPE_ALL);
    } while(gHandShake != HANDSHAKE_MAGIC_WORD);

    /**************** Send application image to targert EVM *******************/
    {
        /* Open appimage file */
        FF_FILE *appFp = ff_fopen(APPIMAGE_FILENAME, "rb");

        if(appFp == NULL)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            uint32_t fileSize = ff_filelength(appFp);

            DebugP_log("Sending appimage - \"%s\" of size %d\r\n", APPIMAGE_FILENAME, fileSize);

            ff_fread((uint8_t *)(CONFIG_PCIE0_OB_REGION0_LOWER + CONFIG_PCIE0_OB_REGION0_UPPER), fileSize, 1, appFp);

            volatile uint32_t *magicWordPtr = (uint32_t *)((CONFIG_PCIE0_OB_REGION0_LOWER + BOOTLOADER_APPIMAGE_MAX_FILE_SIZE + sizeof(uint32_t)));

            *magicWordPtr = APPIMAGE_MAGIC_WORD;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("Images transferred successfully\r\n");
    }
    else
    {
        DebugP_logError("Image transfer failed\r\n");
    }

    Board_deinit();
    System_deinit();

    return 0;
}
