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

/**
 *  \file   pka_util.c
 *
 *  \brief  This file contains the utility functions of PKA driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <security/crypto/crypto_util.h>
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Crypto_Uint8ToUint32(const uint8_t *source, uint32_t sourceLengthInBytes, uint32_t *dest)
{
    uint32_t i, t = 0;

    for (i=0; i< sourceLengthInBytes; i++)
    {
        t = (t << 8) | source[i];
        if ((i & 3) == 3) {
            *dest++ = t;
            t = 0;
        }
    }
    if ((i & 3) != 0)
    {
        *dest = t << ((4-(i&3)) << 3);
    }
    return;
}

void Crypto_Uint32ToUint8(const uint32_t *src, uint32_t sourceLengthInBytes, uint8_t *dest)
{
    uint32_t i, t;

    for (i=0; i< sourceLengthInBytes; i+=4)
    {
        t = *src++;
        *dest++ = t >> 24;
        *dest++ = t >> 16;
        *dest++ = t >> 8;
        *dest++ = t;
    }
    return;
}

void Crypto_Uint32ToBigInt(uint32_t *source, uint32_t sourceLengthInWords, uint32_t *dest)
{
    uint32_t i, t = 0, t2 = 0;
    t2 = sourceLengthInWords / 2;

    for(i=0;i<t2;i++)
    {
        t = source[i];
        source[i] = source[sourceLengthInWords - 1 - i];
        source[sourceLengthInWords - 1 - i] = t;
    }
    dest[0] = sourceLengthInWords;
    for(i=0; i < sourceLengthInWords; i++)
    {
        dest[1 + i] = source[i];
    }

    return;
}

void Crypto_bigIntToUint32(uint32_t *source, uint32_t sourceLengthInWords, uint32_t *dest)
{
    uint32_t i, t = 0, t2 = 0;
    t2 = (sourceLengthInWords / 2)+1;

    for(i=1; i<t2; i++)
    {
        t = source[i];
        source[i] = source[sourceLengthInWords-(i-1)];
        source[sourceLengthInWords-(i-1)] = t;
    }
    for(i=0; i < sourceLengthInWords; i++)
    {
        dest[i] = source[i+1];
    }

    return;
}

void Crypto_PKCSPaddingForSign(const uint8_t *shaHash, uint32_t keyLengthInBytes, uint32_t typeOfAlgo, uint8_t *output)
{
    uint32_t  i, shaLen = 0, psLen = 0, offset = 0;

    switch(typeOfAlgo)
    {
        case 0:
            shaLen = 20, psLen = keyLengthInBytes - 3 - shaLen;
        break;
        case 1:
            shaLen = 32, psLen = keyLengthInBytes - 3  - shaLen;
        break;
        case 2:
            shaLen = 64, psLen = keyLengthInBytes - 3  - shaLen;
        break;
    }
    output[offset] = 0x00;
    offset++;
    output[offset] = 0x01;
    offset++;

    for(i = 0; i< psLen; i++)
    {
        output[offset+i] = 0xFF;
    }
    offset = offset + psLen;
    output[offset]= 0x00;
    offset++;

    switch(typeOfAlgo)
    {
        case 0:
            for(i = 0; i<shaLen; i++)
            {
                output[offset + i] = shaHash[i];
            }
            offset = offset + shaLen;
        break;
        case 1:
            for(i = 0; i<shaLen; i++)
            {
                output[offset + i] = shaHash[i];
            }
            offset = offset + shaLen;
        break;
        case 2:
            for(i = 0; i<shaLen; i++)
            {
                output[offset + i] = shaHash[i];
            }
            offset = offset + shaLen;
        break;
    }

    return;
}

void Crypto_PKCSPaddingForMessage(const uint8_t *message, uint32_t msgLengthInBytes, uint32_t keyLengthInBytes, uint8_t *output)
{
    uint32_t  i, psLen, offset = 0;
    uint32_t upper = 99, lower = 1;
    output[offset] = 0x00;
    offset++;
    output[offset] = 0x02;
    offset++;
    psLen = keyLengthInBytes - msgLengthInBytes - 3;

    /* Initializes random number generator */
       srand((unsigned) time(NULL));

       /* n random numbers from 0 to 99 */
       for( i = 0 ; i < psLen ; i++ )
       {
        output[offset + i] = (rand() %(upper-lower+1))+lower;
       }
       offset = offset + psLen;

    output[offset]= 0x00;
    offset++;

    for(i = 0; i<msgLengthInBytes; i++)
    {
        output[offset + i] = message[i];
    }

    return;
}
