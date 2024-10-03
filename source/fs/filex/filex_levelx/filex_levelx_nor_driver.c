/***************************************************************************
 * Copyright (c) 2024 Microsoft Corporation 
 * 
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 * 
 * SPDX-License-Identifier: MIT
 **************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */ 
/** FileX Component                                                       */ 
/**                                                                       */
/**   FileX NOR FLASH Simulator Driver                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#include "fx_api.h"
#include "lx_api.h"

#include <fs/filex/filex_levelx/lx_nor_flash_driver.h>


VOID _fx_nor_driver(FX_MEDIA *media_ptr)
{
    LX_NOR_FLASH *p_nor_flash;
    UCHAR *source_buffer;
    UCHAR *destination_buffer;
    ULONG logical_sector;
    ULONG i;
    UINT status;


    /* There are several useful/important pieces of information contained in the media
       structure, some of which are supplied by FileX and others are for the driver to
       setup. The following is a summary of the necessary FX_MEDIA structure members:

            FX_MEDIA Member                              Meaning

        fx_media_driver_request             FileX request type. Valid requests from FileX are 
                                            as follows:

                                                    FX_DRIVER_READ
                                                    FX_DRIVER_WRITE                 
                                                    FX_DRIVER_FLUSH
                                                    FX_DRIVER_ABORT
                                                    FX_DRIVER_INIT
                                                    FX_DRIVER_BOOT_READ
                                                    FX_DRIVER_RELEASE_SECTORS
                                                    FX_DRIVER_BOOT_WRITE
                                                    FX_DRIVER_UNINIT

        fx_media_driver_status              This value is RETURNED by the driver. If the 
                                            operation is successful, this field should be
                                            set to FX_SUCCESS for before returning. Otherwise,
                                            if an error occurred, this field should be set
                                            to FX_IO_ERROR. 

        fx_media_driver_buffer              Pointer to buffer to read or write sector data.
                                            This is supplied by FileX.

        fx_media_driver_logical_sector      Logical sector FileX is requesting.

        fx_media_driver_sectors             Number of sectors FileX is requesting.


       The following is a summary of the optional FX_MEDIA structure members:

            FX_MEDIA Member                              Meaning

        fx_media_driver_info                Pointer to any additional information or memory.
                                            This is optional for the driver use and is setup
                                            from the fx_media_open call. The RAM disk uses
                                            this pointer for the RAM disk memory itself.

        fx_media_driver_write_protect       The DRIVER sets this to FX_TRUE when media is write 
                                            protected. This is typically done in initialization, 
                                            but can be done anytime.

        fx_media_driver_free_sector_update  The DRIVER sets this to FX_TRUE when it needs to 
                                            know when clusters are released. This is important
                                            for FLASH wear-leveling drivers.

        fx_media_driver_system_write        FileX sets this flag to FX_TRUE if the sector being
                                            written is a system sector, e.g., a boot, FAT, or 
                                            directory sector. The driver may choose to use this
                                            to initiate error recovery logic for greater fault
                                            tolerance.

        fx_media_driver_data_sector_read    FileX sets this flag to FX_TRUE if the sector(s) being
                                            read are file data sectors, i.e., NOT system sectors.

        fx_media_driver_sector_type         FileX sets this variable to the specific type of 
                                            sector being read or written. The following sector
                                            types are identified:

                                                    FX_UNKNOWN_SECTOR 
                                                    FX_BOOT_SECTOR
                                                    FX_FAT_SECTOR
                                                    FX_DIRECTORY_SECTOR
                                                    FX_DATA_SECTOR  
     */


    /* Process the driver request specified in the media control block.  */
    switch(media_ptr -> fx_media_driver_request) {

        case FX_DRIVER_READ:

            p_nor_flash = lx_nor_driver_nor_flash_get((uint32_t)media_ptr->fx_media_driver_info);

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

            p_nor_flash = lx_nor_driver_nor_flash_get((uint32_t)media_ptr->fx_media_driver_info);

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

            p_nor_flash = lx_nor_driver_nor_flash_get((uint32_t)media_ptr->fx_media_driver_info);

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

            p_nor_flash = lx_nor_driver_nor_flash_get((uint32_t)media_ptr->fx_media_driver_info);

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

            p_nor_flash = lx_nor_driver_nor_flash_get((uint32_t)media_ptr->fx_media_driver_info);

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

