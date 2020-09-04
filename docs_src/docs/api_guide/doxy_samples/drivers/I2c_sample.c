
#include <stdio.h>
//! [include]
#include <drivers/i2c.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>

#define CONFIG_I2C0            (0U)
I2C_Handle                     gI2cHandle;
uint8_t                        txBuffer[1];
uint8_t                        rxBuffer[2];

void open(void)
{
//! [open]
    I2C_Params      params;
  
    I2C_Params_init(&params);
    params.transferMode  = I2C_MODE_BLOCKING;
    I2C_init();
    gI2cHandle = I2C_open(CONFIG_I2C0, &params);
    if (!gI2cHandle) {
        DebugP_assert(FALSE);
    }
//! [open]
}

void close(void)
{
//! [close]
    I2C_close(gI2cHandle);
//! [close]
}

void i2c_transfer_blocking(void)
{
//! [i2c_transfer_blocking]
    int32_t status;

    I2C_Transaction i2cTransaction;
    I2C_Transaction_init(&i2cTransaction);
  
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1U;
  
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2U;
  
    status = I2C_transfer(gI2cHandle, &i2cTransaction);
    if (SystemP_SUCCESS != status) {
        DebugP_assert(FALSE);
    }
//! [i2c_transfer_blocking]
}
