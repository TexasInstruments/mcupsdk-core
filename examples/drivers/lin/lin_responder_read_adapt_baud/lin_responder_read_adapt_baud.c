/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

#include <drivers/lin.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define I2C_TARGET_ADDRESS      (0x20U)
#define I2C_POLARITY_INV        (0x05U)

#define LIN_FRAME_LENGTH        (0x08U)

/* ========================================================================== */
/*                          Example Description                               */
/* ==========================================================================  /
Description: Example demonstrates LIN Responder mode data read operation where
             the LIN Instance is set as a Responder. The transfer is initiated
             by PLIN-USB device connected to external PC.

The LIN instance waits for Reception of a Message from remote Commander.

Note: Example Can be run in Polling, Interrupt and DMA mode, The Operating mode
      is configurable in SYSCONFIG.
/ =========================================================================== */

void lin_responder_read_adapt_baud_main(void)
{
    int32_t                 status;
    I2C_Handle              i2cHandle;
    I2C_Transaction         i2cTransaction;
    LIN_Handle              handle;
    LIN_SCI_Frame           frame;
    uint8_t                 txBuffer[1];
    uint8_t                 rxData[8] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

    Drivers_open();
    Board_driversOpen();

    i2cHandle = gI2cHandle[CONFIG_I2C0];
    handle = gLinHandle[CONFIG_LIN0];

    DebugP_log("[LIN] LIN Responder Read Application Started ...\r\n");

    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1U;
    i2cTransaction.targetAddress = I2C_TARGET_ADDRESS;
    txBuffer[0] = I2C_POLARITY_INV;
    status = I2C_transfer(i2cHandle, &i2cTransaction);
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("[I2C] LIN Voltage Level Shifter started ...\r\n");

    DebugP_log("[LIN] Responder Read ... !!!\r\n");

    LIN_SCI_Frame_init(&frame);

    frame.dataBuf = rxData;
    frame.frameLen = LIN_FRAME_LENGTH;
    frame.txnType = LIN_HLD_TXN_TYPE_READ;

    status = LIN_SCI_transferFrame(handle, &frame);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("Received Message ID: 0x%x\r\n", frame.id);
        DebugP_log("Received Message Frame Length: %u\r\n", frame.frameLen);
        DebugP_log("Received Message Data:\r\n");

        for(uint8_t i = 0; i < frame.frameLen; i++)
        {
            DebugP_log("0x%x \r\n", rxData[i]);
        }

        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }


    Board_driversClose();
    Drivers_close();

}