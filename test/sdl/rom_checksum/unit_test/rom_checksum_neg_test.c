/*********************************************************************
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 *
 */

/**
 *  \file rom_checksum_neg_test.c
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "rom_checksum_test_main.h"


/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */
#define SDL_INVALID_CURLEN (130)
#define SDL_INVALID_GOLDEN_DATA_POINTER (uint64_t *)(0x004182FFCC)

int32_t ROM_Checksum_negTest()
{
    SDL_ROM_Checksum_obj md;
    md.length = 0;
    uint8_t *in = SDL_DATA_TO_BE_HASHED_POINTER;
    int32_t inlen = SDL_LENGTH_OF_DATA_TO_BE_HASHED;
    uint64_t *golden_vector_pointer = SDL_GOLDEN_DATA_POINTER;
    uint64_t  golden_vector_buflen = SDL_LENGTH_OF_GOLDEN_DATA;
    int32_t   testStatus = SDL_PASS;
    int32_t sdlRet;

/***************************************************************************************************
*     Call SDL API SDL_ROM_Checksum_process with Invalid Message Digest current length (md.curlen)
****************************************************************************************************/
    md.curlen = SDL_INVALID_CURLEN;
    sdlRet = SDL_ROM_Checksum_process (&md, (unsigned char *)in, (int32_t)inlen);
    if(sdlRet == SDL_PASS){
        testStatus = SDL_EFAIL;
        DebugP_log("SDL_ROM_Checksum_Process: failure on line no. %d \n", __LINE__);
    }

/*******************************************************************************************
*     Call SDL API SDL_ROM_Checksum_done with Invalid Message Digest current length
*******************************************************************************************/
    if(testStatus == SDL_PASS)
    {
        sdlRet = SDL_ROM_Checksum_done(&md);
        if(sdlRet == SDL_PASS){
            testStatus = SDL_EFAIL;
            DebugP_log("SD_ROM_Checksum_done : failure on line no. %d \n", __LINE__);
        }
    }

/*******************************************************************************************
*     Call SDL API SDL_ROM_Checksum_compareResult  with Invalid SDL_INVALID_GOLDEN_DATA_POINTER
*******************************************************************************************/
    if(testStatus == SDL_PASS)
    {
        golden_vector_pointer = SDL_INVALID_GOLDEN_DATA_POINTER;
        sdlRet = SDL_ROM_Checksum_compareResult (golden_vector_buflen, &md, golden_vector_pointer);
        if(sdlRet == SDL_PASS){
            testStatus = SDL_EBADARGS;
            DebugP_log("SD_ROM_Checksum_compareResult: failure on line no. %d \n", __LINE__);
        }
    }


    return testStatus;
}
