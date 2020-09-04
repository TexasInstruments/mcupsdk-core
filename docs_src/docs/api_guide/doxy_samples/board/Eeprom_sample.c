
#include <stdio.h>
//! [include]
#include <board/eeprom.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>

EEPROM_Handle handle;
uint8_t buffer[100];

void eeprom_read(void)
{
    int32_t     status;
//! [read]
    status = EEPROM_read(handle, 0, buffer, 100);
    DebugP_assert(SystemP_SUCCESS == status);
//! [read]
}

void eeprom_write(void)
{
    int32_t     status;
//! [write]
    status = EEPROM_write(handle, 0, buffer, 100);
    DebugP_assert(SystemP_SUCCESS == status);
//! [write]
}
