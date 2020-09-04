
#include <stdio.h>
//! [include]
#include <board/flash.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>

Flash_Handle handle;
uint8_t buffer[100];

void flash_read(void)
{
    int32_t     status;
//! [read]
    uint32_t offset;

    /* Set offset to read from */
    offset = 0;

    /* Do the read */
    status = Flash_read(handle, offset, buffer, 100);
    DebugP_assert(SystemP_SUCCESS == status);
//! [read]
}

void flash_erase(void)
{
	int32_t status;
//! [erase]
	uint32_t offset, blk, page;

	/* Set offset to erase */
	offset = 0;

	/* Find the block number corresponding to the offset */
	status = Flash_offsetToBlkPage(handle, offset, &blk, &page);
    DebugP_assert(SystemP_SUCCESS == status);

	/* Erase the block */
	status = Flash_eraseBlk(handle,blk);
    DebugP_assert(SystemP_SUCCESS == status);
//! [erase]
}

void flash_write(void)
{
    int32_t     status;
//! [write]
    uint32_t offset;

    /* Set offset to write to */
    offset = 0;

    /* Do the write */
    status = Flash_write(handle, offset, buffer, 100);
    DebugP_assert(SystemP_SUCCESS == status);
//! [write]
}
