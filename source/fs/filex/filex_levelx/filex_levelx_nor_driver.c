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

#include "fx_api.h"
#include "lx_api.h"

#include <lx_nor_flash_driver.h>


VOID _fx_nor_driver(FX_MEDIA *media_ptr)
{
    LX_NOR_FLASH *p_nor_flash;
    UCHAR *source_buffer;
    UCHAR *destination_buffer;
    ULONG logical_sector;
    ULONG i;
    UINT status;

    /* Process the driver request specified in the media control block.  */
    switch(media_ptr -> fx_media_driver_request) {

        case FX_DRIVER_READ:

            p_nor_flash = (LX_NOR_FLASH *)media_ptr->fx_media_driver_info;

            /* Setup the destination buffer and logical sector.  */
            logical_sector =      media_ptr -> fx_media_driver_logical_sector;
            destination_buffer =  (UCHAR *) media_ptr -> fx_media_driver_buffer;

            /* Loop to read sectors from flash.  */
            for (i = 0; i < media_ptr -> fx_media_driver_sectors; i++)
            {

                /* Read a sector from NOR flash.  */
                status =  lx_nor_flash_sector_read(p_nor_flash, logical_sector, destination_buffer);

                /* Determine if the read was successful.  */
                if (status != LX_SUCCESS)
                {

                    /* Return an I/O error to FileX.  */
                    media_ptr -> fx_media_driver_status =  FX_IO_ERROR;

                    return;
                }

                /* Move to the next entries.  */
                logical_sector++;
                destination_buffer =  destination_buffer + media_ptr -> fx_media_bytes_per_sector;
            }

            /* Successful driver request.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;

        case FX_DRIVER_WRITE:

            p_nor_flash = (LX_NOR_FLASH *)media_ptr->fx_media_driver_info;

            /* Setup the source buffer and logical sector.  */
            logical_sector =      media_ptr -> fx_media_driver_logical_sector;
            source_buffer =       (UCHAR *) media_ptr -> fx_media_driver_buffer;

            /* Loop to write sectors to flash.  */
            for (i = 0; i < media_ptr -> fx_media_driver_sectors; i++)
            {

                /* Write a sector to NOR flash.  */
                status =  lx_nor_flash_sector_write(p_nor_flash, logical_sector, source_buffer);

                /* Determine if the write was successful.  */
                if (status != LX_SUCCESS)
                {

                    /* Return an I/O error to FileX.  */
                    media_ptr -> fx_media_driver_status =  FX_IO_ERROR;

                    return;
                }

                /* Move to the next entries.  */
                logical_sector++;
                source_buffer =  source_buffer + media_ptr -> fx_media_bytes_per_sector;
            }

            /* Successful driver request.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;

        case FX_DRIVER_RELEASE_SECTORS:

            p_nor_flash = (LX_NOR_FLASH *)media_ptr->fx_media_driver_info;

            /* Setup the logical sector.  */
            logical_sector =  media_ptr -> fx_media_driver_logical_sector;

            /* Release sectors.  */
            for (i = 0; i < media_ptr -> fx_media_driver_sectors; i++)
            {

                /* Release NOR flash sector.  */
                status =  lx_nor_flash_sector_release(p_nor_flash, logical_sector);

                /* Determine if the sector release was successful.  */
                if (status != LX_SUCCESS)
                {

                    /* Return an I/O error to FileX.  */
                    media_ptr -> fx_media_driver_status =  FX_IO_ERROR;

                    return;
                }

                /* Move to the next entries.  */
                logical_sector++;
            }

            /* Successful driver request.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;

        case FX_DRIVER_FLUSH:

            /* Return driver success.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;

        case FX_DRIVER_ABORT:

            /* Return driver success.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;

        case FX_DRIVER_INIT:

            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;

        case FX_DRIVER_UNINIT:
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;

        case FX_DRIVER_BOOT_READ:

            p_nor_flash = (LX_NOR_FLASH *)media_ptr->fx_media_driver_info;

            /* Read the boot record and return to the caller.  */

            /* Setup the destination buffer.  */
            destination_buffer =  (UCHAR *) media_ptr -> fx_media_driver_buffer;

            /* Read boot sector from NOR flash.  */
            status =  lx_nor_flash_sector_read(p_nor_flash, 0, destination_buffer);

            /* For NOR driver, determine if the boot record is valid.  */
            if ((destination_buffer[0] != (UCHAR) 0xEB) ||
                    (destination_buffer[1] != (UCHAR) 0x34) ||
                    (destination_buffer[2] != (UCHAR) 0x90))
            {

                /* Invalid boot record, return an error!  */
                media_ptr -> fx_media_driver_status =  FX_MEDIA_INVALID;
                return;
            }

            /* Determine if the boot read was successful.  */
            if (status != LX_SUCCESS)
            {

                /* Return an I/O error to FileX.  */
                media_ptr -> fx_media_driver_status =  FX_IO_ERROR;

                return;
            }

            /* Successful driver request.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;

        case FX_DRIVER_BOOT_WRITE:

            p_nor_flash = (LX_NOR_FLASH *)media_ptr->fx_media_driver_info;

            /* Make sure the media bytes per sector equals to the LevelX logical sector size.  */
            if (media_ptr -> fx_media_bytes_per_sector != (LX_NOR_SECTOR_SIZE) * sizeof(ULONG))
            {

                /* Sector size mismatch, return error.  */
                media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                break;
            }

            /* Write the boot record and return to the caller.  */

            /* Setup the source buffer.  */
            source_buffer =       (UCHAR *) media_ptr -> fx_media_driver_buffer;

            /* Write boot sector to NOR flash.  */
            status =  lx_nor_flash_sector_write(p_nor_flash, 0, source_buffer);

            /* Determine if the boot write was successful.  */
            if (status != LX_SUCCESS)
            {

                /* Return an I/O error to FileX.  */
                media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
                return;
            }

            /* Successful driver request.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break ;

        default:

            /* Invalid driver request.  */
            media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
            break;
    }
}

