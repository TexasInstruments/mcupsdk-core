/*!
 * \brief
 * Provide NVRAM hardware access functions to be called by littleFS
 *
 * \author
 * KUNBUS GmbH
 *
 * \date
 * 2022-06-02
 *
 * \copyright
 * Copyright (c) 2021, KUNBUS GmbH<br /><br />
 * All rights reserved.<br />
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:<br />
 * <ol>
 * <li>Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.</li>
 * <li>Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.</li>
 * <li>Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.</li>
 * </ol>
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "nvram_driver.h"
#include "nvram.h"
#include "osal.h"

/*!***********************************************************
 *
 * <!-- Description: -->
 *
 * \brief
 * Member variables of the driver instance
 *
 **************************************************************/
typedef struct nvram_driver {
    Flash_Handle flash; //!< handle to instance of the hardware driver
    uint32_t baseAdr; //!< minimum absolute flash address so that other things can be stored below
    uint32_t baseBlock; //!< absolute block at baseAdr
    uint32_t endAdr; //!< one after the last absolute flash address
    uint32_t endBlock; //!< one after the last absolute block
    struct lfs_config lfscfg; //!< backup of LitteFS config type
} nvram_driver_t;

// the one and only driver instance
static nvram_driver_t drv = {
        .flash = NULL,
        .baseAdr = 0,
        .baseBlock = 0,
        .endAdr = 0,
        .endBlock = 0,
        .lfscfg = { 0 }
};

/********** Private Function Declarations ********/

static int NVR_DRV_s_read(const struct lfs_config *pCfg, lfs_block_t block,
        lfs_off_t off, void *pBuffer, lfs_size_t size);
static int NVR_DRV_s_prog(const struct lfs_config *pCfg, lfs_block_t block,
        lfs_off_t off, const void *pBuffer, lfs_size_t size);
static int NVR_DRV_s_erase(const struct lfs_config *pCfg, lfs_block_t block);

/************ Private Function Definitions ******************/

/*!*************************************************************
 * <!-- Description: -->
 *
 * \brief
 * Read a region in a block.
 *
 * \param[in] pCfg Pointer to LittleFS configuration struct
 * \param[in] block Block number
 * \param[in] off Relative offset in block
 * \param[in] pBuffer Data buffer
 * \param[in] size Data size
 *
 * \return Negative littleFS error codes are propagated to the user.
 *
 **************************************************************/
static int NVR_DRV_s_read(const struct lfs_config *pCfg, lfs_block_t block,
        lfs_off_t off, void *pBuffer, lfs_size_t size)
{
    int ret = LFS_ERR_OK;
    // calculate absolute offset
    const uint32_t absOffs = drv.baseAdr + block * drv.lfscfg.block_size + off;
    NVR_LOG_DEBUG("block = %u, off = %u, size = %u, absOffs = 0x%X", block, off, size, absOffs);
    if (NULL == drv.flash)
    {
        NVR_LOG_ERROR("no flash instance");
        ret = LFS_ERR_INVAL;
    } else if((absOffs < drv.baseAdr) || (absOffs > drv.endAdr - size))
    {
        NVR_LOG_ERROR("address out of range [0x%08X, 0x%08X]", drv.baseAdr, drv.endAdr - size);
        ret = LFS_ERR_INVAL;
    } else if(SystemP_SUCCESS != Flash_read(drv.flash, absOffs, pBuffer, size))
    {
        NVR_LOG_ERROR("Flash_read FAILED");
        ret = LFS_ERR_IO;
    }
    return ret;
}
/*!*************************************************************
 * <!-- Description: -->
 *
 * \brief
 * Program a region in a block. The block must have previously
 * been erased.
 *
 * \param[in] pCfg Pointer to LittleFS configuration struct
 * \param[in] block Block number
 * \param[in] off Relative offset in block
 * \param[in] pBuffer Data buffer
 * \param[in] size Data size
 *
 * \return Negative littleFS error codes are propagated to the user.
 * May return LFS_ERR_CORRUPT if the block should be considered bad.
 *
 **************************************************************/
static int NVR_DRV_s_prog(const struct lfs_config *pCfg, lfs_block_t block,
        lfs_off_t off, const void *pBuffer, lfs_size_t size)
{
    int ret = LFS_ERR_OK;
    // calculate absolute offset
    const uint32_t absOffs = drv.baseAdr + block * drv.lfscfg.block_size + off;
    NVR_LOG_DEBUG("block = %u, off = %u, size = %u, absOffs = 0x%X", block, off, size, absOffs);
    if (NULL == drv.flash)
    {
        NVR_LOG_ERROR("no flash instance");
        ret = LFS_ERR_INVAL;
    } else if((absOffs < drv.baseAdr) || (absOffs > drv.endAdr - size))
    {
        NVR_LOG_ERROR("address out of range [0x%08X, 0x%08X]", drv.baseAdr, drv.endAdr - size);
        ret = LFS_ERR_INVAL;
    } else if(SystemP_SUCCESS != Flash_write(drv.flash, absOffs, (uint8_t*)pBuffer, size))
    {
        // absOffs must be aligned to page
        NVR_LOG_ERROR("Flash_write FAILED");
        ret = LFS_ERR_IO;
    }
    return ret;
}
/*!*************************************************************
 * <!-- Description: -->
 *
 * \brief
 * Erase a block. A block must be erased before being programmed.
 * The state of an erased block is undefined.
 *
 * \param[in] pCfg Pointer to LittleFS configuration struct
 * \param[in] block Block number
 *
 * \return Negative littleFS error codes are propagated to the user.
 * May return LFS_ERR_CORRUPT if the block should be considered bad.
 *
 **************************************************************/
static int NVR_DRV_s_erase(const struct lfs_config *pCfg, lfs_block_t block)
{
    // calculate absolute block
    const uint32_t absBlock = drv.baseBlock + block;
    int ret = LFS_ERR_OK;
    NVR_LOG_DEBUG("block = %u, absBlock = %u", block, absBlock);
    if (NULL == drv.flash)
    {
        NVR_LOG_ERROR("no flash instance");
        ret = LFS_ERR_INVAL;
    } else if((absBlock < drv.baseBlock) || (absBlock >= drv.endBlock))
    {
        NVR_LOG_ERROR("block out of range [%u, %u]", drv.baseBlock, drv.endBlock);
        ret = LFS_ERR_INVAL;
    } else if(SystemP_SUCCESS != Flash_eraseBlk(drv.flash, absBlock))
    {
        NVR_LOG_ERROR("Flash_eraseBlk FAILED");
        ret = LFS_ERR_IO;
    }
    return ret;
}

/*************** Public Function Definitions *************/

/*!*************************************************************
 * <!-- Description: -->
 *
 * \brief
 * Initialize the driver and return configuration struct for littleFS.
 * As of now there is only one static driver instance (one flash
 * interface), drv.
 *
 * \param[in] flashIdx Flash instance index, e.g. CONFIG_FLASH0
 * \param[in] startAdr Lower boundary in flash address space to
 *                     be used for the file system
 *
 * \return Pointer to littleFS configuration or NULL on error
 *
 **************************************************************/
extern struct lfs_config* NVR_DRV_init(const unsigned int instance, const unsigned int baseAdr)
{
    struct lfs_config* ret = NULL;
    NVR_LOG_DEBUG("");
#ifndef CONFIG_FLASH_NUM_INSTANCES
#error "flash driver not available, check sysconfig"
#else
    // NB: Flash_open() must have been called before via Board_driversOpen()
    if(CONFIG_FLASH_NUM_INSTANCES <= instance)
    {
        NVR_LOG_ERROR("flash driver instance %u out of range [0, %u]", instance, CONFIG_FLASH_NUM_INSTANCES);
    } else if(NULL == (drv.flash = gFlashHandle[instance]))
    {
        NVR_LOG_ERROR("flash driver not open, call Board_driversOpen() first");
    } else {
        NVR_LOG_INFO("base address for the NVRAM file system: 0x%04X", baseAdr);
        drv.baseAdr = baseAdr;
        drv.lfscfg.block_size = Flash_getAttrs(CONFIG_FLASH0)->blockSize;
        drv.lfscfg.read_size = Flash_getAttrs(CONFIG_FLASH0)->pageSize;
        if(drv.baseAdr % drv.lfscfg.block_size)
        {
            // start address must be block-aligned
            NVR_LOG_ERROR("base address %u must be block-aligned (%u)", drv.baseAdr,
                    drv.lfscfg.block_size);
        } else if(drv.lfscfg.block_size % drv.lfscfg.read_size)
        {
            NVR_LOG_ERROR("read size %u must be block-aligned (%u)", drv.lfscfg.read_size,
                    drv.lfscfg.block_size);
        } else {
            // littleFS configuration
            // block device operations
            drv.lfscfg.read  = NVR_DRV_s_read;
            drv.lfscfg.prog  = NVR_DRV_s_prog;
            drv.lfscfg.erase = NVR_DRV_s_erase;
            // other functions are defined and assigned in NVRAM module
            drv.lfscfg.sync = NULL;
            drv.lfscfg.lock = NULL;
            drv.lfscfg.unlock = NULL;
            // block device configuration
            drv.baseBlock = drv.baseAdr / drv.lfscfg.block_size;
            drv.lfscfg.block_count = Flash_getAttrs(CONFIG_FLASH0)->blockCount - baseAdr / drv.lfscfg.block_size;
            drv.endBlock = drv.baseBlock + drv.lfscfg.block_count;
            drv.endAdr = drv.baseAdr + drv.lfscfg.block_count * drv.lfscfg.block_size;
            drv.lfscfg.prog_size = drv.lfscfg.read_size;
            drv.lfscfg.cache_size = drv.lfscfg.read_size;
            drv.lfscfg.lookahead_size = (drv.lfscfg.read_size / 8u) * 8u;
            // buffers are allocated by LittleFS
            drv.lfscfg.read_buffer = NULL;
            drv.lfscfg.prog_buffer = NULL;
            drv.lfscfg.lookahead_buffer = NULL;
            // limit for wear leveling
            drv.lfscfg.block_cycles = 100;
            // a file cannot be greater than the entire memory space less the journal
            drv.lfscfg.file_max = (drv.lfscfg.block_count - 1u) * drv.lfscfg.block_size;
            ret = &drv.lfscfg;
        }
    }
#endif
    return ret;
}
/*!*************************************************************
 * <!-- Description: -->
 *
 * \brief
 * Finish the driver
 *
 * \return Error code
 *
 **************************************************************/
extern int NVR_DRV_fini(void)
{
    NVR_err_t ret = NVR_ERR_OK;
    NVR_LOG_DEBUG("");
    return ret;
    // NB: Flash_close() must afterwards be called via Board_driversClose()
}
