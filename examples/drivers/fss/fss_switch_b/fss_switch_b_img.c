/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/fss.h>
#include <drivers/ospi.h>
#include <drivers/flsopskd/v0/flsopskd.h>
#include <drivers/fota_agent/v0/fota_agent.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "board.h"
#include "new_application_images.h"

#define MAX_ELF_PROGRAM_HEADER (21U)
#define BOOINFO_ADDRESS (0x80000U)
#define BOOT_REGION_A (0U)
#define BOOT_REGION_B (1U)

typedef struct bootinfo_sector_s_t
{
    uint8_t phyTuningVector[OSPI_FLASH_ATTACK_VECTOR_SIZE];
    uint32_t bootRegion;
} bootinfo_sector_fields_t;

typedef union bootinfo_sector_u_t
{
    bootinfo_sector_fields_t fields;
    uint8_t bin[ERASE_SECTOR_SIZE];
} bootinfo_sector_t;

FOTAAgent_Handle gFotaAgentHandle;
FOTAAgent_Params gAgentParams;
uint8_t gFotaAgentProcessingBuffer[ERASE_SECTOR_SIZE];
ELFUP_ELFPH gProgramHeaderArray[MAX_ELF_PROGRAM_HEADER];

void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while (loop)
        ;
}

uint32_t FLSOPSKD_usrGetTicks()
{
    /* in syscfg, 1 tick is config as 1ms */
    return ClockP_getTicks();
}

void switch_b_img_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    bootinfo_sector_t bootinfo;

    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("Starting application\r\n");

    FOTAAgent_Params_init(&gAgentParams);
    gAgentParams.flsopsParams.eraseOpCode = ERASE_OP_CODE;
    gAgentParams.flsopsParams.eraseExOpCode = ERASE_EXOP_CODE;
    gAgentParams.flsopsParams.pageSizeInBytes = FLASH_PAGE_SIZE;
    gAgentParams.flsopsParams.eraseSizeInBytes = ERASE_SECTOR_SIZE;
    gAgentParams.pProcessingBuffer = gFotaAgentProcessingBuffer;
    gAgentParams.pProgramHeader = gProgramHeaderArray;
    gAgentParams.programHeaderCnt = MAX_ELF_PROGRAM_HEADER;
    FOTAAgent_init(&gFotaAgentHandle, &gAgentParams);

    DebugP_log("Receiving application... \r\n");

    /*
        new application can be received over any interface like Ethernet, CAN, etc.
        The entire application will never be received at once, and will be received in chunks.
        The following code emulates such a case.

        Usually, 2 different files are expected to be received on the interface, viz. mcelf and
        mcelf_xip, and each one must be handled separately.

    */

    /* Step 1: Handle mcelf file */
    DebugP_log("Got MCELF file\r\n");
    status = FOTAAgent_writeStart(&gFotaAgentHandle, FLASH_SIZE / 2 + 0x81000, FALSE);
    DebugP_assert(status == SystemP_SUCCESS);
    for (uint32_t cnt = 0; cnt < MCELF_FILE_LEN; cnt++)
    {
        /*
            as and when a byte is being received, it is being sent to agent, which will handle
            all the intricacies.
        */
        uint8_t byte = mcelf_file[cnt];
        status = FOTAAgent_writeUpdate(&gFotaAgentHandle, &byte, 1);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    status = FOTAAgent_writeEnd(&gFotaAgentHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("Got MCELF_XIP file\r\n");
    status = FOTAAgent_writeStart(&gFotaAgentHandle, FLASH_SIZE / 2, TRUE);
    DebugP_assert(status == SystemP_SUCCESS);
    for (uint32_t cnt = 0; cnt < MCELFXIP_FILE_LEN; cnt++)
    {
        /*
            as and when a byte is being received, it is being sent to agent, which will handle
            all the intricacies.
        */
        uint8_t byte = mcelfxip_file[cnt];
        status = FOTAAgent_writeUpdate(&gFotaAgentHandle, &byte, 1);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    status = FOTAAgent_writeEnd(&gFotaAgentHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("Done writing new application to flash...\r\nUpdating Flash's Boot Sector...\r\n");

    /* Update boot segment */
    offset = BOOINFO_ADDRESS;
    CacheP_inv((void *)(SOC_getFlashDataBaseAddr() + offset), ERASE_SECTOR_SIZE, CacheP_TYPE_ALL);
    memcpy((void *)&bootinfo.bin, (void *)(SOC_getFlashDataBaseAddr() + offset), ERASE_SECTOR_SIZE);
    bootinfo.fields.bootRegion = BOOT_REGION_B;
    status = FLSOPSKD_erase(&gFotaAgentHandle.flopsHandle, offset);
    DebugP_assert(status == SystemP_SUCCESS);
    status = FLSOPSKD_write(&gFotaAgentHandle.flopsHandle, offset, (uint8_t *)&bootinfo.bin, ERASE_SECTOR_SIZE);
    DebugP_assert(status == SystemP_SUCCESS);
    CacheP_inv((void *)(SOC_getFlashDataBaseAddr() + offset), ERASE_SECTOR_SIZE, CacheP_TYPE_ALL);

    DebugP_log("Finish Updating Flash's Boot Sector... Please reset board to start new application\r\n\r\n");

    Board_driversClose();
    Drivers_close();
}
