/*********************************************************************
 *   Copyright (c) Texas Instruments Incorporated 2024
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
 * \file  sdl_tmu_rom_checksum.c
 *
 * \brief  SDL implementation file for the tmu rom checksum.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <sdl/tmu_rom_checksum/sdl_tmu_rom_checksum.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_mcrc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/**
 * Design: PROC_SDL-7377, PROC_SDL-7378
 */
int32_t SDL_TMU_ROM_Checksum_compute(SDL_MCRC_Channel_t mcrcChannelNumber,
                                     SDL_MCRC_Signature_t  *sectSignVal)
{
    int32_t result = SDL_EFAIL;
    SDL_MCRC_DataConfig_t mcrcData;

    mcrcData.pMCRCData       = (uint32_t *)(SDL_DEF_TMU_ROM_BA);
    mcrcData.size            = SDL_TMU_ROM_SIZE;
    mcrcData.dataBitSize     = SDL_MCRC_DATA_32_BIT;

    if ((mcrcChannelNumber > SDL_MCRC_CHANNEL_4) || (sectSignVal == (NULL_PTR)))
    {
        result = SDL_EBADARGS;
    }

    if (result != SDL_EBADARGS)
    {
        result = SDL_MCRC_init(MCRC0, mcrcChannelNumber, SDL_TMU_MCRC_WDPRELD,
                              SDL_TMU_MCRC_BLKPRELD);
    }
    if (result == SDL_PASS)
    {
        result = SDL_MCRC_channelReset(MCRC0, mcrcChannelNumber);
    }
    if (result == SDL_PASS)
    {
        result = SDL_MCRC_config(MCRC0, mcrcChannelNumber, SDL_TMU_MCRC_PCNT,
                                 SDL_TMU_MCRC_SCNT,
                                 SDL_MCRC_OPERATION_MODE_FULLCPU);
    }
    if (result == SDL_PASS)
    {
        result = SDL_MCRC_configDataWidth(MCRC0, mcrcChannelNumber,
                                          SDL_MCRC_DATAWIDTH_SEL_32BIT);
    }
    if (result == SDL_PASS)
    {
        result = SDL_MCRC_computeSignCPUmode(MCRC0,
                                    mcrcChannelNumber,
                                    &mcrcData, sectSignVal);
    }
    if (result == SDL_PASS)
    {
        /* Check if the calculated CRC matches with the golden reference CRC.*/
        if((sectSignVal->regH == SDL_TMU_CRC_GOLDEN_REFH) &&
           (sectSignVal->regL == SDL_TMU_CRC_GOLDEN_REFL))
        {
            result = SDL_PASS;
        }
        else
        {
            result = SDL_EFAIL;
        }
    }

	return result;
}