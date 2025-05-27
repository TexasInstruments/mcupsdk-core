/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/uart.h>
#define XMODEM_STS_ACK  0x06

int xmodemReceive(unsigned char *dest, int destsz);
int xmodemTransmit(unsigned char *src, int srcsz);

static UART_Handle gUartHandle;

int32_t _inbyte(uint16_t timeout)
{
	uint8_t ch = 0xFF;
    int32_t status = SystemP_FAILURE;

	if(gUartHandle!=NULL)
    {
        UART_Transaction trans;
        UART_Transaction_init(&trans);
        trans.buf   = &ch;
        trans.count = 1;
        trans.timeout = (uint32_t)timeout;
        status = UART_read(gUartHandle, &trans);
        if(status == SystemP_SUCCESS)
            status = (int32_t)ch;
    }

    return (int32_t)status;
}

void _outbyte(int32_t ch)
{
	uint8_t buf = (uint8_t)ch;

	if(gUartHandle!=NULL)
    {
        UART_Transaction trans;
        UART_Transaction_init(&trans);
        trans.buf   = &buf;
        trans.count = 1;
        UART_write(gUartHandle, &trans);
    }
}

int32_t Bootloader_xmodemSendAck(uint32_t uartDrvIndex)
{
	gUartHandle = UART_getHandle(uartDrvIndex);

#if (defined (SOC_AM273X) || defined(SOC_AWR294X))
    /* Workaround for AM273X ROM not sending ACK for EOT which results in host
     * side terminal app still waiting after completing 100% transfer of
     * SBL image */
    _outbyte(XMODEM_STS_ACK);
#endif

    return SystemP_SUCCESS;
}

int32_t Bootloader_xmodemReceive(uint32_t uartDrvIndex, uint8_t *dstBuf, uint32_t maxBufSize, uint32_t *fileSize)
{
	int32_t status = SystemP_SUCCESS;
	int32_t rxBytes = 0;

	gUartHandle = UART_getHandle(uartDrvIndex);

	if(gUartHandle != NULL)
	{
		rxBytes = xmodemReceive(dstBuf, maxBufSize);

		if(rxBytes > 0)
		{
			*fileSize = rxBytes;
			status = SystemP_SUCCESS;
		}
		else
		{
			status = SystemP_FAILURE;
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

int32_t Bootloader_xmodemTransmit(uint32_t uartDrvIndex, uint8_t *src, uint32_t len)
{
	int32_t status = SystemP_SUCCESS;
	int32_t txBytes = 0;

	gUartHandle = UART_getHandle(uartDrvIndex);

	if(gUartHandle != NULL)
	{
		txBytes = xmodemTransmit(src, len);

		if(txBytes > 0)
		{
			status = SystemP_SUCCESS;
		}
		else
		{
			status = SystemP_FAILURE;
		}
	}
	else
	{
		status = SystemP_FAILURE;
	}

	return status;
}

