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

#ifndef LFS_OSPI_H

    #define LFS_OSPI_H

    #ifdef __cplusplus
        extern "C" {
    #endif

    /*Include files*/
    #include <fs/littlefs/lfs.h>
    #include <fs/littlefs/lfs_util.h>
    #include <board/flash.h>

    /**
     * \brief   Flash offset macro
     *
     * Used by block level wrapper apis to calculate the physical address of flash
     */
    #define LFS_OSPI_FLASH_OFFSET_BASE        (0x1600000U)


    /**
     *  \brief Description
     *  \n
     *      This function registers the block level wrappers.
     *      Must be called from the application.
     *
     *  \param[in]  cfg   lfs config object
     *
     *  \param[in]  flashHandleObj  Flash handle object
     */
    void lfsOspiInitOps(struct lfs_config *cfg, Flash_Handle flashHandleObj);


    #ifdef __cplusplus
        } /* extern "C" */
    #endif

#endif