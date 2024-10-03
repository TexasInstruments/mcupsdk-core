

#ifndef LX_NOR_FLASH_DRIVER_H
#define LX_NOR_FLASH_DRIVER_H

#include <stdint.h>
#include <lx_api.h>

UINT  lx_nor_driver_create(uint32_t nor_driver_instance_id, uint32_t flash_instance_id, uint32_t offset, uint32_t size);

UINT lx_nor_driver_init(LX_NOR_FLASH *p_nand_flash);

uint32_t lx_nor_driver_size_get(uint32_t nor_driver_instance_id);

LX_NOR_FLASH *lx_nor_driver_nor_flash_get(uint32_t nor_driver_instance_id);

#endif
