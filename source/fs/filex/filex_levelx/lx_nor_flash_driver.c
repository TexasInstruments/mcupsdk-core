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
/** LevelX Component                                                      */ 
/**                                                                       */
/**   NOR Flash Simulator                                                 */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary files.  */

#include "lx_api.h"
#include "lx_nor_flash_driver.h"

#include <board/flash.h>
#include <kernel/dpl/DebugP.h>

#ifndef LX_NOR_ENABLE_CONTROL_BLOCK_FOR_DRIVER_INTERFACE
#error "Control block mode must be enabled."
#endif

/* Define constants for the NOR flash simulation. */

/* This configuration is for one physical sector of overhead.  */


#define LX_NOR_FLASH_DRIVER_GET(p_nor)      ((lx_nor_driver_t *)((uint8_t *)(p_nor) - (((uint8_t *)&((lx_nor_driver_t *)0)->nor_flash) - (uint8_t *)((lx_nor_driver_t *)0))))


#define LX_NOR_DRIVER_MAX_INSTANCE_COUNT      (4u)


#define FREE_BIT_MAP_WORDS                  ((PHYSICAL_SECTORS_PER_BLOCK-1)/32)+1
#define USABLE_SECTORS_PER_BLOCK            (PHYSICAL_SECTORS_PER_BLOCK-1)
#define UNUSED_METADATA_WORDS_PER_BLOCK     (WORDS_PER_PHYSICAL_SECTOR-(3+FREE_BIT_MAP_WORDS+USABLE_SECTORS_PER_BLOCK))

typedef struct lx_nor_driver {
    LX_NOR_FLASH nor_flash;
    Flash_Handle flash_handle;
    Flash_Attrs *p_attrs;
    uint8_t t_sec_buf[512u];
    uint32_t offset;
    uint32_t size;
} lx_nor_driver_t;


typedef struct lx_nor_driver_data {
    lx_nor_driver_t t_instances[LX_NOR_DRIVER_MAX_INSTANCE_COUNT];
} lx_nor_driver_data_t;


static lx_nor_driver_data_t g_nor_driver_data = {0};

UINT  lx_nor_driver_read(LX_NOR_FLASH *nor_flash, ULONG *flash_address, ULONG *destination, ULONG words);
UINT  lx_nor_driver_write(LX_NOR_FLASH *nor_flash, ULONG *flash_address, ULONG *source, ULONG words);
UINT  lx_nor_driver_block_erase(LX_NOR_FLASH *nor_flash, ULONG block, ULONG erase_count);
UINT  lx_nor_driver_block_erased_verify(LX_NOR_FLASH *nor_flash, ULONG block);
UINT  lx_nor_driver_system_error(LX_NOR_FLASH *nor_flash, UINT error_code, ULONG block, ULONG sector);


UINT  lx_nor_driver_create(uint32_t nor_driver_instance_id, uint32_t flash_instance_id, uint32_t offset, uint32_t size)
{
    lx_nor_driver_t *p_nor_driver;

    p_nor_driver = &g_nor_driver_data.t_instances[nor_driver_instance_id];

    p_nor_driver->p_attrs = Flash_getAttrs(flash_instance_id);
    p_nor_driver->flash_handle = Flash_getHandle(flash_instance_id);

    DebugP_assert(offset % p_nor_driver->p_attrs->blockSize == 0);
    DebugP_assert((size == (uint32_t)-1) || (size % p_nor_driver->p_attrs->blockSize == 0));

    p_nor_driver->offset = offset;
    p_nor_driver->size = size == (uint32_t)-1 ? p_nor_driver->p_attrs->flashSize : size;

    return(LX_SUCCESS);
}


UINT lx_nor_driver_init(LX_NOR_FLASH *p_nor_flash)
{
    lx_nor_driver_t *p_nor_driver;

    p_nor_driver = LX_NOR_FLASH_DRIVER_GET(p_nor_flash);

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
    p_nor_flash->lx_nor_flash_sector_buffer = (ULONG *)&p_nor_driver->t_sec_buf[0];

    return (LX_SUCCESS);
}


uint32_t lx_nor_driver_size_get(uint32_t nor_driver_instance_id)
{
    DebugP_assert(nor_driver_instance_id < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(g_nor_driver_data.t_instances[nor_driver_instance_id].p_attrs != NULL);
    return (g_nor_driver_data.t_instances[nor_driver_instance_id].size);
}


LX_NOR_FLASH *lx_nor_driver_nor_flash_get(uint32_t nor_driver_instance_id)
{
    DebugP_assert(nor_driver_instance_id < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(g_nor_driver_data.t_instances[nor_driver_instance_id].p_attrs != NULL);
    return (&g_nor_driver_data.t_instances[nor_driver_instance_id].nor_flash);
}


UINT lx_nor_driver_read(LX_NOR_FLASH *nor_flash, ULONG *flash_address, ULONG *destination, ULONG words)
{
    lx_nor_driver_t *p_nor_driver;
    int32_t ret;

    p_nor_driver = LX_NOR_FLASH_DRIVER_GET(nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    ret = Flash_read(p_nor_driver->flash_handle, (uint32_t)flash_address + p_nor_driver->offset, (uint8_t *)destination, 4u * words);
    if (ret != SystemP_SUCCESS) return (LX_ERROR);

    return (LX_SUCCESS);
}


UINT lx_nor_driver_write(LX_NOR_FLASH *nor_flash, ULONG *flash_address, ULONG *source, ULONG words)
{
    lx_nor_driver_t *p_nor_driver;
    int32_t ret;

    p_nor_driver = LX_NOR_FLASH_DRIVER_GET(nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    ret = Flash_write(p_nor_driver->flash_handle, (uint32_t)flash_address + p_nor_driver->offset, (uint8_t *)source, 4u * words);
    if (ret != SystemP_SUCCESS) return (LX_ERROR);

    return (LX_SUCCESS);
}


UINT lx_nor_driver_block_erase(LX_NOR_FLASH *nor_flash, ULONG block, ULONG erase_count)
{
    lx_nor_driver_t *p_nor_driver;
    int32_t ret;

    (void)erase_count;

    p_nor_driver = LX_NOR_FLASH_DRIVER_GET(nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    ret = Flash_eraseBlk(p_nor_driver->flash_handle, block + p_nor_driver->offset / p_nor_driver->p_attrs->blockSize);
    if (ret != SystemP_SUCCESS) return (LX_ERROR);

    return (LX_SUCCESS);
}


UINT lx_nor_driver_block_erased_verify(LX_NOR_FLASH *nor_flash, ULONG block)
{
    lx_nor_driver_t *p_nor_driver;
    int32_t ret;

    p_nor_driver = LX_NOR_FLASH_DRIVER_GET(nor_flash);

    DebugP_assert(p_nor_driver - &g_nor_driver_data.t_instances[0] < LX_NOR_DRIVER_MAX_INSTANCE_COUNT);
    DebugP_assert(p_nor_driver->p_attrs != NULL);

    ret = Flash_eraseBlk(p_nor_driver->flash_handle, block + p_nor_driver->offset / p_nor_driver->p_attrs->blockSize);
    if (ret != SystemP_SUCCESS) return (LX_ERROR);

    // TODO: Verify here?

    return (LX_SUCCESS);
}

UINT lx_nor_driver_system_error(LX_NOR_FLASH *nor_flash, UINT error_code, ULONG block, ULONG sector)
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

