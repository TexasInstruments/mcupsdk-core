
#include <stdio.h>
//! [include]
#include <board/ram.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>

Ram_Handle handle;
uint8_t buffer[100];

void ram_read(void)
{
    int32_t     status;
//! [read]
    uint32_t offset;

    /* Set offset to read from */
    offset = 0;

    /* Do the read */
    status = Ram_read(handle, offset, buffer, 100);
    DebugP_assert(SystemP_SUCCESS == status);
//! [read]
}

void ram_write(void)
{
    int32_t     status;
//! [write]
    uint32_t offset;

    /* Set offset to write to */
    offset = 0;

    /* Do the write */
    status = Ram_write(handle, offset, buffer, 100);
    DebugP_assert(SystemP_SUCCESS == status);
//! [write]
}
