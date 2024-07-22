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

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/fss.h>
#include <drivers/ospi.h>

#define BUFFER_DATA_SIZE    (4096)
#define FLASH_SECTOR_SIZE   (4096U)
#define BOOINFO_ADDRESS     (0x80000U)
#define BOOT_REGION_A       (0U)
#define BOOT_REGION_B       (1U)

typedef struct bootinfo_sector_s_t
{
    uint8_t phyTuningVector[OSPI_FLASH_ATTACK_VECTOR_SIZE];
    uint32_t bootRegion;
}bootinfo_sector_fields_t;

typedef union bootinfo_sector_u_t
{
    bootinfo_sector_fields_t fields;
    uint8_t bin[FLASH_SECTOR_SIZE];
}bootinfo_sector_t;

/*
    This example: 
    1. writes data to flash at 18MB offset. 
    2. remaps address from 16MB and above to 0MB and above.
    3. reads back the data from 2MB offset.
    4. checks if data that is read back is correct or not.
*/
void switch_b_img_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t offset;
    bootinfo_sector_t bootinfo;
    uint32_t sector, page;

    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    offset = BOOINFO_ADDRESS;

    status = Flash_read(gFlashHandle[CONFIG_FLASH0], offset, (uint8_t*)&bootinfo, sizeof(bootinfo));

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Flash Read Failed at 0x%X offset !!!", offset);
    }
    else
    {
        bootinfo.fields.bootRegion = BOOT_REGION_B;
     
        Flash_offsetToSectorPage(gFlashHandle[CONFIG_FLASH0], offset, &sector, &page);
        status = Flash_eraseSector(gFlashHandle[CONFIG_FLASH0], sector);

        if(status != SystemP_SUCCESS)
        {   
            DebugP_log("Block Erase Failed at 0x%X offset !!!", offset);
        }
        else
        {
            status = Flash_write(gFlashHandle[CONFIG_FLASH0], offset, (uint8_t*)&bootinfo, sizeof(bootinfo));
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("Flash Write of %d bytes failed at 0x%X offset !!!", sizeof(bootinfo), offset);
            }
            else
            {
                
                DebugP_log("\r\nAll Test Passed\r\n");
            }
        }
    }

    Board_driversClose();
    Drivers_close();
}
