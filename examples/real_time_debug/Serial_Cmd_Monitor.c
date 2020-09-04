/*
 * Copyright (C) 2021-2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Serial_Cmd_HAL.h"
#include "Serial_Cmd_Monitor.h"

#define     TEMP_THRESHOLD      4

#define		RW_CMD 			0x80

#define		EXTENSION_BYTE		0x07

//RW CMD TYPE
#define READ 1
#define WRITE 0
//ENDIANNESS
#define BIG_ENDIAN 1
#define LITTLE_ENDIAN 0
//ADDRESSIBLE SIZE
#define ADDRESSIBLE_32_BIT 1
#define ADDRESSIBLE_16_BIT 0
// override these depends on target
#define CMD_BUFFER_SIZE 131 //1 + 4 + 63*2 = 131

unsigned char gInCmdBuffer[CMD_BUFFER_SIZE];
unsigned short gInCmdBufferIdx = 0;
volatile unsigned short gInCmdSkipCount;

void ClearBufferRelatedParam(void);

// override these depends on target
int GetTargetEndianness(void)
{
	return LITTLE_ENDIAN;
}
// override these depends on target
void Write8bitByteToCOM(unsigned char c)
{
    SerialCmd_send(c & 0xff);
}

int GetSizeOfMAUIn8bitByte(void)
{
	unsigned char maxMAUValue = (unsigned char)(-1);
	switch (maxMAUValue)
	{
	case 0xff:
		return 1;
	// case 0xffff:
	// 	return 2;
	default:
		return 0;
	}
}

int WriteToCmdBuffer(unsigned char* buf, unsigned short* bufIdx, unsigned char d)
{
	if ( (*bufIdx) < CMD_BUFFER_SIZE )
	{
		buf[*bufIdx] = d & 0xff;
		(*bufIdx)++;
		return 0;
	}

	return 1;
}

void ResetInCmdBuffer(void)
{
	gInCmdBufferIdx = 0;
}

int WriteByteToInCmdBuffer(unsigned char d)
{
	return WriteToCmdBuffer(gInCmdBuffer, &gInCmdBufferIdx, d);
}

int GetTransferSizeInMAU() //transfer size refer to the words to read/write of a given cmd, not the number of bytes for the whole cmd packet
{
	return (gInCmdBuffer[0] & 0x3f);
}

int VerifyInputCmdHeaders(void)
{
	return ((gInCmdBuffer[0] & 0x80) == 0x80) ? 0 : 1;
}

int GetInputCmdType(void)
{
	return (gInCmdBuffer[0] & 0x80);
}

int GetRWFlag()//equivalent to endianness on the MAU in transmission
{
	int ret = ((gInCmdBuffer[0] >> 6) & 0x1); //
	return ret;
}

unsigned char* GetInCmdAddress(void)
{
	unsigned char* addr = 0;
	unsigned long addr_value = 0;
	int i = 0;
	int addressSize = 4;
	for (; i < addressSize; i++)
	{
		addr_value |= (unsigned long)( gInCmdBuffer[1 + i] << 8 * (addressSize - 1 - i) ); //big endian
	}

	addr = (unsigned char*) addr_value;
	return addr;
}

void WriteMAUToCOM(unsigned char d)
{
	int MAUSize = GetSizeOfMAUIn8bitByte();

	switch (MAUSize)
	{
	case 1:
		Write8bitByteToCOM(d);
		break;
	case 2:
	{
		unsigned char MAU[2];
		MAU[0] = (unsigned char)(d & 0xff);
		MAU[1] = (unsigned char)(d >> 8);
		if (GetTargetEndianness() == LITTLE_ENDIAN)
		{
			Write8bitByteToCOM(MAU[0]);
			Write8bitByteToCOM(MAU[1]);
		} else {
			Write8bitByteToCOM(MAU[1]);
			Write8bitByteToCOM(MAU[0]);
		}
	}
	break;
	default://only handles 8bit, 16bit MAU
		break;
	}
}

unsigned char GetWriteCmdDataMAU(int idx)
{
	unsigned char startIdx = 1 + 4;

	unsigned char val = 0;
	int MAUSize = GetSizeOfMAUIn8bitByte();
	int byteOffset = idx*MAUSize;

	switch (MAUSize)
	{
	case 1:
		val = gInCmdBuffer[startIdx + byteOffset];
		break;
	case 2:
		if (GetTargetEndianness() == LITTLE_ENDIAN)
		{
			val = ( gInCmdBuffer[startIdx + byteOffset + 1] << 8 ) | gInCmdBuffer[startIdx + byteOffset];
		} else {
			val = ( gInCmdBuffer[startIdx + byteOffset] | gInCmdBuffer[startIdx + byteOffset + 1] << 8 );
		}
		break;
	default://only handles 8bit, 16bit MAU
		break;
	}

	return val;
}

void ClearBufferRelatedParam(void)
{
	gInCmdSkipCount = 0;
	gInCmdBufferIdx	= 0;
}

void MemAccessCmd(int RW)
{
	unsigned short MAUsToRead = 0;
	unsigned char dataChar = 0;
	unsigned char* addr = GetInCmdAddress();
	unsigned short i, j;

	for ( j = 0; j < 1; j++ )
	{
		Write8bitByteToCOM(gInCmdBuffer[j]);
	}

	MAUsToRead = GetTransferSizeInMAU();
	for ( i = 0; i < MAUsToRead; i++ )
	{
		if (RW == READ)
		{
			dataChar = *(addr + i);
			WriteMAUToCOM(dataChar);
		} else { //WRITE
			dataChar = GetWriteCmdDataMAU(i);
			*(addr + i) = dataChar;
		}
	}
}

int ProcessCommand(void)
{
	if ( VerifyInputCmdHeaders() )
	{
		return 1;
	}

	switch ( GetInputCmdType() )
	{
	case RW_CMD:
		MemAccessCmd(GetRWFlag());
		break;
	default:
		return 1;
	}

	return 0;
}

void receivedDataCommand(unsigned char d) // only lower byte will be used even if MAU is bigger than 1 byte
{
	WriteByteToInCmdBuffer(d);

	if (gInCmdSkipCount > 0)
	{
		gInCmdSkipCount--;
		return;
	}

	if (gInCmdBufferIdx > 0 && gInCmdSkipCount == 0)
	{ //wrong input header, clear cmd buffer
		if ( VerifyInputCmdHeaders() )
		{
			ClearBufferRelatedParam();
			return;
		}

		if (gInCmdBufferIdx == 1) {
			if (GetRWFlag() == WRITE)
			{
				gInCmdSkipCount = 4 - 1 + GetTransferSizeInMAU() * GetSizeOfMAUIn8bitByte();
			} else {
				gInCmdSkipCount = 4 - 1 ;
			}
		} else {
			ProcessCommand();
			ClearBufferRelatedParam();
		}
		return;
	}

}
// cmd structure end --------------------------------------------


//#########################################
