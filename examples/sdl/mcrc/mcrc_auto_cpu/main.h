/* Copyright (c) 2022 Texas Instruments Incorporated
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
 *  \file     main.h
 *
 *  \brief    This file contains MCRC example code defines.
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <sdl/mcrc/v0/sdl_mcrc.h>
#include <sdl/mcrc/v0/sdl_ip_mcrc.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>


#if !defined(MAIN_H)
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the application interface */
typedef struct sdlMCRCApp_s
{
    int32_t  (*application)(void);   /* The code that runs the application */
    char      *name;                  /* The application name */
    int32_t    status;            /* App Status */
} sdlMCRCApp_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define SDL_APP_NOT_RUN        			 (-(int32_t) (2))
#define SDL_APP_FAILED         			 (-(int32_t) (1))
#define SDL_APP_PASS           			 ( (int32_t) (0))

/*      Default values      */
#define MCRC_DEF_PATTERN_COUNT           ((uint32_t) 0U)
#define MCRC_DEF_SECTOR_COUNT            ((uint32_t) 1U)
#define MCRC_DEF_WATCHDOG_PRELOAD        ((uint32_t) 0U)
#define MCRC_DEF_BLOCK_PRELOAD           ((uint32_t) 0U)

#define MCRC_DDR_1080P_BUF_SIZE          (4147200U)
#define MCRC_DDR_720P_BUF_SIZE           (1843200U)
#define MCRC_DDR_SMALL_BUF_SIZE          (200000U)
#define MCRC_DDR_BUF_MAX_SIZE            (MCRC_DDR_1080P_BUF_SIZE)
#define MCRC_MSMC_BUF_MAX_SIZE           (1024U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 *  \brief MCRC configuration parameter structure.
 */
typedef struct
{
    SDL_MCRC_InstType    instance;
    /**< Base address of MCRC instance. */
    SDL_MCRC_Channel_t  mcrcChannelNumber;
    /**< MCRC channel to use. */
    SDL_MCRC_ModeType   mcrcMode;
    /**< MCRC operational mode. */
    uint32_t            mcrcPatternSize;
    /**< Pattern size. */
    uint32_t            mcrcPatternCount;
    /**< Pattern count. This is calculated in program.*/
    uint32_t            mcrcSectorCount;
    /**< sector count. */
    uint32_t            mcrcWatchdogPreload;
    /**< Watchdog preload value. */
    uint32_t            mcrcBlockPreload;
    /**< Block preload value. */
    uint32_t            mcrcSignHigh;
    /**< MSB 32 bits Pre-calculated Signature value . */
    uint32_t            mcrcSignLow;
    /**< LSB 32 bits Pre-calculated Signature value . */
    uint32_t            dataSize;
    /**< Specify size of data(in Bytes) on which MCRC is to be performed. */
    uint32_t            sourceMemory;
    /**< Specify memory address on which MCRC is to be performed. */
} SDL_MCRC_ConfigParams_t;

/* ========================================================================== */
/*                         Function Declarations                               */
/* ========================================================================== */

extern int32_t mcrcAutoCPU_main(void);

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif /* MAIN_H */
/* Nothing past this point */
