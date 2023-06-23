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
 * \file  sdl_rom_checksum.c
 *
 * \brief  SDL implementation file for the rom checksum module.
 */

#include <stdint.h>
#include <string.h>
#include <sdl/sdl_rom_checksum.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
/**
  Process full ROMChecksum for block of memory
  @param in     Pointer to be message being hashed
  @param inlen  The lenith of the data (octets)
  @return SDL_PASS if successful
 */

/**
 * Design: PROC_SDL-6002
 */
int32_t SDL_ROM_Checksum_compute (void)
{
 	uint8_t * in = SDL_DATA_TO_BE_HASHED_POINTER;
	uint32_t inlen = SDL_LENGTH_OF_DATA_TO_BE_HASHED;
 	uint64_t * golden_vector_pointer = (uint64_t *)SDL_GOLDEN_DATA_POINTER;
	uint64_t  golden_vector_buflen =  SDL_LENGTH_OF_GOLDEN_DATA;
	SDL_ROM_Checksum_obj md;
	int32_t sdl_result = SDL_PASS;
	SDL_ROM_Checksum_init(&md);
    sdl_result = SDL_ROM_Checksum_process(&md, in, (uint32_t)inlen);
	if(sdl_result == SDL_PASS)
	{
    	sdl_result = SDL_ROM_Checksum_done(&md);
	}
	if(sdl_result == SDL_PASS)
	{
		sdl_result = SDL_ROM_Checksum_compareResult (golden_vector_buflen, &md, golden_vector_pointer);
	}
	return sdl_result;
}


