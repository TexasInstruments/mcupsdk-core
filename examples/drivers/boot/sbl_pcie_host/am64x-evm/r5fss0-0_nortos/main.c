/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
#define SBL_IMAGE_BASE              (CSL_MSRAM_256K0_RAM_BASE)

/* Offset in SRAM where SBL image is sent to. */
/* The SBL image should not overlap with the SBL load address */
#define SBL_IMAGE_OFFSET            (0x80000)

/* "MAGIC WORD" to indicate image transfer is complete */
#define APPIMAGE_MAGIC_WORD         (0xC0DEFEED)

/* Hand shake "MAGIC WORD" to indicate SBL is ready to accept images */
#define HANDSHAKE_MAGIC_WORD        (0xFEEDC0DE)

/* Offset to which Appimage is sent to */
#define APPIMAGE_OFFSET             (0x1000U)

/* Maximum size of Appimage supported */
#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE   (0x800000U - (2*sizeof(uint32_t)))

#define BOOTIMAGE_FILENAME ("/sd0/sbl_pcie.release.hs_fs.tiimage")
#define APPIMAGE_FILENAME ("/sd0/app")

/* Pointer to which the SBL image is send to is communicated via */
uint32_t *offsetPtr = (uint32_t *)(CONFIG_PCIE0_OB_REGION0_LOWER + 0x001BCFE0);

/* Variable to which the handshake from SBL is received, to start sending appimage */
uint32_t gHandShake __attribute__ ((used, aligned(4096)));

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
            ff_fread((uint8_t *)((CONFIG_PCIE0_OB_REGION0_LOWER + CONFIG_PCIE0_OB_REGION0_UPPER) + SBL_IMAGE_OFFSET), fileSize, 1, appFp);

            *offsetPtr = SBL_IMAGE_BASE + SBL_IMAGE_OFFSET;
        }
    }

    DebugP_assert(status == SystemP_SUCCESS);
#endif

    /* Wait for handshake from SBL PCIe application to start sending appimage */
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

            ff_fread((uint8_t *)(CONFIG_PCIE0_OB_REGION1_LOWER + APPIMAGE_OFFSET), fileSize, 1, appFp);

            volatile uint32_t *offsetPtr = (uint32_t *)((CONFIG_PCIE0_OB_REGION1_LOWER + BOOTLOADER_APPIMAGE_MAX_FILE_SIZE));

            *offsetPtr = APPIMAGE_OFFSET;

            volatile uint32_t *magicWordPtr = (uint32_t *)((CONFIG_PCIE0_OB_REGION1_LOWER + BOOTLOADER_APPIMAGE_MAX_FILE_SIZE + sizeof(uint32_t)));

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
