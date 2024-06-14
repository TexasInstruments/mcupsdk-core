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

 /**
 *  \file lfs_flash.h
 *
 *  \brief LittleFS flash Driver API/interface file.
 */

#ifndef lfs_flash_H

    #define lfs_flash_H

    #ifdef __cplusplus
        extern "C" {
    #endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

    #include <fs/littlefs/LittleFS/lfs.h>
    #include <fs/littlefs/LittleFS/lfs_util.h>
    #include <board/flash.h>

/* ========================================================================== */
/*                             Structure Definitions                          */
/* ========================================================================== */

/**
 *  \brief Structure to bind lfs_config structure with flash properties.
 */
    typedef struct LFS_FLASH_Config_t
    {
        /** Flash instance index */
        uint32_t lfsFlashIndex;
        /** Flash offset given for an LFS instance */
        uint32_t lfsFlashOffset;
        /** Total number of lfs instances*/
        uint32_t numLfsInstances;
        /** Pointer to lfs_config structure for given LFS instance*/
        struct lfs_config *lfsStructcfg;
    }LFS_FLASH_Config;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

    /**
     *  \brief Description
     *  \n
     *      This function wrapper api for block level read.
     *
     *  \param[in]  cfg   lfs_config object
     *
     *  \param[in]  block   block number given by lfs
     *
     *  \param[in]  offset  offset in the block given by lfs
     *
     *  \param[in]  size  size of read operation
     *
     *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
     */
    int LFS_blockRead( const struct lfs_config *cfg, lfs_block_t block,
                            lfs_off_t offset,  void *buffer, lfs_size_t size);

    /**
     *  \brief Description
     *  \n
     *      This is function wrapper api for block level write.
     *
     *  \param[in]  cfg   lfs_config object
     *
     *  \param[in]  block   block number given by lfs
     *
     *  \param[in]  offset  offset in the block given by lfs
     *
     *  \param[in]  buffer  write buffer
     *
     *  \param[in]  size  size of write operation
     *
     *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
     */
    int LFS_blockWrite( const struct lfs_config *cfg, lfs_block_t block,
                            lfs_off_t offset, const void *buffer, lfs_size_t size);

    /**
     *  \brief Description
     *  \n
     *      This is function wrapper api for block erase.
     *
     *  \param[in]  cfg   lfs_config object
     *
     *  \param[in]  block   block number given by lfs
     *
     *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
     */
    int LFS_blockErase( const struct lfs_config *cfg, lfs_block_t block);

    /**
     *  \brief Description
     *  \n
     *      This is function wrapper api for block sync.
     *
     *  \param[in]  cfg   lfs_config object
     *
     *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
     */
    int LFS_blockSync( const struct lfs_config *cfg);

    #ifdef __cplusplus
        } /* extern "C" */
    #endif

#endif