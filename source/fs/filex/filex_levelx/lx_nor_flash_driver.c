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


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "lx_api.h"
#include "lx_nor_flash_driver.h"
#include <tx_api.h>

#include <board/flash.h>
#include <kernel/dpl/DebugP.h>

#ifndef LX_NOR_ENABLE_CONTROL_BLOCK_FOR_DRIVER_INTERFACE
#error "Control block mode must be enabled."
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* Maximum number of this driver instances. */
#define LX_NOR_DRIVER_MAX_INSTANCE_COUNT         (4u)

#define LX_NOR_DRIVER_ERASED_VERIFY_BUF_SIZE   (256u)

#define FREE_BIT_MAP_WORDS                  ((PHYSICAL_SECTORS_PER_BLOCK-1)/32)+1
#define USABLE_SECTORS_PER_BLOCK            (PHYSICAL_SECTORS_PER_BLOCK-1)
#define UNUSED_METADATA_WORDS_PER_BLOCK     (WORDS_PER_PHYSICAL_SECTOR-(3+FREE_BIT_MAP_WORDS+USABLE_SECTORS_PER_BLOCK))


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* NOR driver instance. */
typedef struct lx_nor_driver {
    LX_NOR_FLASH *p_nor_flash;
    Flash_Handle flash_handle;
    Flash_Attrs *p_attrs;
    uint32_t offset;
    uint32_t size;
} lx_nor_driver_t;


/* Global NOR driver data. */
typedef struct lx_nor_driver_data {
    lx_nor_driver_t t_instances[LX_NOR_DRIVER_MAX_INSTANCE_COUNT];
    size_t instance_cnt;
    TX_MUTEX buf_mutex;
    bool init;
} lx_nor_driver_data_t;


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static lx_nor_driver_data_t g_nor_driver_data = {0};
static uint8_t gt_sec_bufs[LX_NOR_DRIVER_MAX_INSTANCE_COUNT][512] __attribute((aligned(128)));
static uint8_t gt_erased_verify_buf[LX_NOR_DRIVER_ERASED_VERIFY_BUF_SIZE] __attribute((aligned(128)));

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static UINT  lx_nor_driver_read(LX_NOR_FLASH *nor_flash, ULONG *flash_address, ULONG *destination, ULONG words);
static UINT  lx_nor_driver_write(LX_NOR_FLASH *nor_flash, ULONG *flash_address, ULONG *source, ULONG words);
static UINT  lx_nor_driver_block_erase(LX_NOR_FLASH *nor_flash, ULONG block, ULONG erase_count);
static UINT  lx_nor_driver_block_erased_verify(LX_NOR_FLASH *nor_flash, ULONG block);
static UINT  lx_nor_driver_system_error(LX_NOR_FLASH *nor_flash, UINT error_code, ULONG block, ULONG sector);

static lx_nor_driver_t *lx_nor_driver_get(LX_NOR_FLASH *p_nor_flash);


/* ========================================================================== */
/*                            Function Definitions                            */
/* ========================================================================== */

UINT  lx_nor_driver_create(LX_NOR_FLASH *p_nor_flash, uint32_t flash_instance_id, uint32_t offset, uint32_t size)
{
    lx_nor_driver_t *p_nor_driver;
    UINT res;

    p_nor_driver = &g_nor_driver_data.t_instances[g_nor_driver_data.instance_cnt];
    g_nor_driver_data.instance_cnt++;

    p_nor_driver->p_nor_flash = p_nor_flash;
    p_nor_driver->p_attrs = Flash_getAttrs(flash_instance_id);
    p_nor_driver->flash_handle = Flash_getHandle(flash_instance_id);

    DebugP_assert(offset % p_nor_driver->p_attrs->blockSize == 0);
    DebugP_assert((size == (uint32_t)-1) || (size % p_nor_driver->p_attrs->blockSize == 0));

    p_nor_driver->offset = offset;
    p_nor_driver->size = size == (uint32_t)-1 ? p_nor_driver->p_attrs->flashSize - offset : size;

    if (!g_nor_driver_data.init) {
        g_nor_driver_data.init = true;
        res = tx_mutex_create(&g_nor_driver_data.buf_mutex, "lx_nor_driver_buf_mutex", TX_INHERIT);
        if (res != SystemP_SUCCESS) {
            return (LX_ERROR);
        }
    }

    return (LX_SUCCESS);
}


UINT lx_nor_driver_init(LX_NOR_FLASH *p_nor_flash)
{
    lx_nor_driver_t *p_nor_driver;
    size_t driver_ix;

    p_nor_driver = lx_nor_driver_get(p_nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    /* Setup the base address of the flash memory.  */
    p_nor_flash->lx_nor_flash_base_address = (ULONG *)0;

    /* Setup geometry of the flash.  */
    p_nor_flash->lx_nor_flash_total_blocks = p_nor_driver->size / p_nor_driver->p_attrs->blockSize;
    p_nor_flash->lx_nor_flash_words_per_block = p_nor_driver->p_attrs->blockSize / 4u;

    /* Setup function pointers for the NOR flash services.  */
    p_nor_flash->lx_nor_flash_driver_read = lx_nor_driver_read;
    p_nor_flash->lx_nor_flash_driver_write = lx_nor_driver_write;
    p_nor_flash->lx_nor_flash_driver_block_erase = lx_nor_driver_block_erase;
    p_nor_flash->lx_nor_flash_driver_block_erased_verify = lx_nor_driver_block_erased_verify;

    /* Setup local buffer for NOR flash operation. This buffer must be the sector size of the NOR flash memory.  */
    driver_ix = p_nor_driver - &g_nor_driver_data.t_instances[0];
    p_nor_flash->lx_nor_flash_sector_buffer = (ULONG *)&gt_sec_bufs[driver_ix][0];

    return (LX_SUCCESS);
}


uint64_t lx_nor_driver_size_get(LX_NOR_FLASH *nor_flash)
{
    lx_nor_driver_t *p_nor_driver;

    p_nor_driver = lx_nor_driver_get(nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    return (p_nor_driver->size);
}

static UINT lx_nor_driver_read(LX_NOR_FLASH *nor_flash, ULONG *flash_address, ULONG *destination, ULONG words)
{
    lx_nor_driver_t *p_nor_driver;
    int32_t ret;

    p_nor_driver = lx_nor_driver_get(nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    ret = Flash_read(p_nor_driver->flash_handle, (uint32_t)flash_address + p_nor_driver->offset, (uint8_t *)destination, 4u * words);
    if (ret != SystemP_SUCCESS) return (LX_ERROR);

    return (LX_SUCCESS);
}


static UINT lx_nor_driver_write(LX_NOR_FLASH *nor_flash, ULONG *flash_address, ULONG *source, ULONG words)
{
    lx_nor_driver_t *p_nor_driver;
    int32_t ret;

    p_nor_driver = lx_nor_driver_get(nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    ret = Flash_write(p_nor_driver->flash_handle, (uint32_t)flash_address + p_nor_driver->offset, (uint8_t *)source, 4u * words);
    if (ret != SystemP_SUCCESS) return (LX_ERROR);

    return (LX_SUCCESS);
}


static UINT lx_nor_driver_block_erase(LX_NOR_FLASH *nor_flash, ULONG block, ULONG erase_count)
{
    lx_nor_driver_t *p_nor_driver;
    int32_t ret;

    (void)erase_count;

    p_nor_driver = lx_nor_driver_get(nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    ret = Flash_eraseBlk(p_nor_driver->flash_handle, block + p_nor_driver->offset / p_nor_driver->p_attrs->blockSize);
    if (ret != SystemP_SUCCESS) return (LX_ERROR);

    return (LX_SUCCESS);
}


static UINT lx_nor_driver_block_erased_verify(LX_NOR_FLASH *nor_flash, ULONG block)
{
    lx_nor_driver_t *p_nor_driver;
    uint32_t base_offset;
    uint32_t rel_offset;
    int32_t ret;
    UINT res;

    p_nor_driver = lx_nor_driver_get(nor_flash);

    res = tx_mutex_get(&g_nor_driver_data.buf_mutex, TX_WAIT_FOREVER);
    if (res != TX_SUCCESS) {
        return (LX_ERROR);
    }

    rel_offset = 0u;
    base_offset = p_nor_driver->offset + block * p_nor_driver->p_attrs->blockSize;
    while (rel_offset < p_nor_driver->p_attrs->blockSize) {

        ret = Flash_read(p_nor_driver->flash_handle, base_offset + rel_offset, &gt_erased_verify_buf[0], LX_NOR_DRIVER_ERASED_VERIFY_BUF_SIZE);
        if (ret != SystemP_SUCCESS) {
            (void)tx_mutex_put(&g_nor_driver_data.buf_mutex);
            return (LX_ERROR);
        }

        for (size_t k = 0u; k < LX_NOR_DRIVER_ERASED_VERIFY_BUF_SIZE / 4u; k++) {
            if (((uint32_t *)gt_erased_verify_buf)[k] != 0xFFFFFFFF) {
                (void)tx_mutex_put(&g_nor_driver_data.buf_mutex);
                return (LX_ERROR);
            }
        }

        rel_offset += LX_NOR_DRIVER_ERASED_VERIFY_BUF_SIZE;
    }

    res = tx_mutex_put(&g_nor_driver_data.buf_mutex);
    if (res != TX_SUCCESS) {
        return (LX_ERROR);
    }

    return (LX_SUCCESS);
}

static UINT lx_nor_driver_system_error(LX_NOR_FLASH *nor_flash, UINT error_code, ULONG block, ULONG sector)
{

#ifdef LX_NOR_ENABLE_CONTROL_BLOCK_FOR_DRIVER_INTERFACE
    LX_PARAMETER_NOT_USED(nor_flash);
#endif
    LX_PARAMETER_NOT_USED(error_code);
    LX_PARAMETER_NOT_USED(block);
    LX_PARAMETER_NOT_USED(sector);

    /* Custom processing goes here...  all errors are fatal.  */
    return(LX_ERROR);
}


static lx_nor_driver_t *lx_nor_driver_get(LX_NOR_FLASH *p_nor_flash)
{
    for (size_t k = 0u; k < LX_NOR_DRIVER_MAX_INSTANCE_COUNT; k++) {
        if (g_nor_driver_data.t_instances[k].p_nor_flash == p_nor_flash) {
            return (&g_nor_driver_data.t_instances[k]);
        }
    }
    return (NULL);
}

