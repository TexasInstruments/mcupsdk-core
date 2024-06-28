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

#include <stdint.h>
#include <fx_api.h>
#include <fx_port.h>
#include <fx_media.h>
#include <drivers/mmcsd/v0/mmcsd.h>
#include "filex_mmcsd.h"



VOID _fx_mmcsd_driver(FX_MEDIA *media_ptr)
{
    MMCSD_Handle mmcsd_hndl;
    uint32_t block_size;
    int res;

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
    switch(media_ptr->fx_media_driver_request)
    {

        case FX_DRIVER_READ:
        {
            uint32_t start_blk;
            uint32_t blk_cnt;

            mmcsd_hndl = MMCSD_getHandle((uint32_t)media_ptr->fx_media_driver_info);
            if (mmcsd_hndl == NULL) {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }


            block_size = MMCSD_getBlockSize(mmcsd_hndl);
            start_blk = media_ptr->fx_media_driver_logical_sector * media_ptr->fx_media_bytes_per_sector / block_size;
            blk_cnt = media_ptr->fx_media_driver_sectors * media_ptr->fx_media_bytes_per_sector / block_size;

            res = MMCSD_read(mmcsd_hndl, media_ptr->fx_media_driver_buffer, start_blk, blk_cnt);
            if (res != SystemP_SUCCESS) {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }

            /* Successful driver request.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_WRITE:
        {
            uint32_t start_blk;
            uint32_t blk_cnt;

            mmcsd_hndl = MMCSD_getHandle((uint32_t)media_ptr->fx_media_driver_info);
            if (mmcsd_hndl == NULL) {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }

            block_size = MMCSD_getBlockSize(mmcsd_hndl);
            start_blk = media_ptr->fx_media_driver_logical_sector * media_ptr->fx_media_bytes_per_sector / block_size;
            blk_cnt = media_ptr->fx_media_driver_sectors * media_ptr->fx_media_bytes_per_sector / block_size;

            res = MMCSD_write(mmcsd_hndl, media_ptr->fx_media_driver_buffer, start_blk, blk_cnt);
            if (res != SystemP_SUCCESS) {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }

            /* Successful driver request.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_RELEASE_SECTORS:
        {
            /* Not supported. */
            /* Successful driver request.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_FLUSH:
        {
            /* Not supported. */
            /* Return driver success.  */
            media_ptr->fx_media_driver_status =  FX_SUCCESS;
            break;
        }

        case FX_DRIVER_ABORT:
        {
            /* Return driver success.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_INIT:
        {
            /* Return driver success.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_UNINIT:
        {
            /* Return driver success.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_BOOT_READ:
        {
            mmcsd_hndl = MMCSD_getHandle((uint32_t)media_ptr->fx_media_driver_info);
            if (mmcsd_hndl == NULL) {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }

            res = MMCSD_read(mmcsd_hndl, media_ptr->fx_media_driver_buffer, 0u, 1u);
            if (res != SystemP_SUCCESS) {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }


            /* Successful driver request.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        case FX_DRIVER_BOOT_WRITE:
        {
            mmcsd_hndl = MMCSD_getHandle((uint32_t)media_ptr->fx_media_driver_info);
            if (mmcsd_hndl == NULL) {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }

            res = MMCSD_write(mmcsd_hndl, media_ptr->fx_media_driver_buffer, 0u, 1u);
            if (res != SystemP_SUCCESS) {
                media_ptr->fx_media_driver_status = FX_IO_ERROR;
                break;
            }

            /* Successful driver request.  */
            media_ptr->fx_media_driver_status = FX_SUCCESS;
            break;
        }

        default:
        {
            /* Invalid driver request.  */
            media_ptr->fx_media_driver_status =  FX_INVALID_STATE;
            break;
        }
    }
}




