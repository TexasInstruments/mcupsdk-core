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
 *  \file     main.c
 *
 *  \brief    This file contains mcrc example code.
 *
 *  \details  MCRC app
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <dpl_interface.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_mcrc.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the application interface */
typedef struct sdlMCRCApp_s
{
    int32_t  (*application)(void);   /* The code that runs the application */
    char      *name;                  /* The application name */
    int32_t    status;            /* App Status */
} sdlMCRCApp_t;

/**
 *  \brief MCRC configuration parameter structure.
 */
typedef struct
{
    SDL_MCRC_InstType   instance;
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

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/*Macros for instances*/
#if defined (R5F_INPUTS)
#define MCRC_INSTANCE  MSS_MCRC
#endif
#if defined (C66_INPUTS)
#define MCRC_INSTANCE  DSS_MCRC
#endif
#define SDL_APP_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_FAILED         (-(int32_t) (1))
#define SDL_APP_PASS           ( (int32_t) (0))

/*      Default values      */
#define MCRC_DEF_PATTERN_COUNT           ((uint32_t) 0U)
#define MCRC_DEF_SECTOR_COUNT            ((uint32_t) 1U)
#define MCRC_DEF_WATCHDOG_PRELOAD        ((uint32_t) 0U)
#define MCRC_DEF_BLOCK_PRELOAD           ((uint32_t) 0U)

#define MCRC_BUF_MAX_SIZE            (40000U)

#define MCRC_NUM_USE_CASES           (2U)

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
static int32_t sdl_mcrc_full_cpu_test(void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */
static uint8_t gMCRCSrcBuffer[MCRC_BUF_MAX_SIZE] __attribute__((aligned(128), section(".bss:extMemCache:ramdisk"))) = {1U};

/** \brief Defines the various MCRC use cases. */
static SDL_MCRC_ConfigParams_t params[MCRC_NUM_USE_CASES] =
{
    {
#if defined(SOC_AM64X) || defined (SOC_AM243X)
        MCU_MCRC64_0,
#elif defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
	    MCRC0,
#elif defined(SOC_AWR294X) || defined(SOC_AM273X)
        MCRC_INSTANCE,
#endif

        (uint32_t) SDL_MCRC_CHANNEL_1,
        (uint32_t) SDL_MCRC_OPERATION_MODE_FULLCPU,
        4U,
        MCRC_DEF_PATTERN_COUNT,
        MCRC_DEF_SECTOR_COUNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x3668f7eaU,
        0xaef33083U,
        MCRC_BUF_MAX_SIZE,
        (uint32_t) &gMCRCSrcBuffer[0],
    },
    {
#if defined(SOC_AM64X) || defined (SOC_AM243X)
        MCU_MCRC64_0 ,
#elif defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
	    MCRC0,
#elif defined(SOC_AWR294X) || defined(SOC_AM273X)
        MCRC_INSTANCE,
#endif

        (uint32_t) SDL_MCRC_CHANNEL_2,
        (uint32_t) SDL_MCRC_OPERATION_MODE_FULLCPU,
        4U,
        MCRC_DEF_PATTERN_COUNT,
        MCRC_DEF_SECTOR_COUNT,
        MCRC_DEF_WATCHDOG_PRELOAD,
        MCRC_DEF_BLOCK_PRELOAD,
        0x3668f7eaU,
        0xaef33083U,
        MCRC_BUF_MAX_SIZE,
        (uint32_t) &gMCRCSrcBuffer[0],
    },
};

sdlMCRCApp_t  sdlmcrcAppList[] = {
    {sdl_mcrc_full_cpu_test,    "MCRC_fullCPU_mode",     SDL_APP_NOT_RUN },
    {NULL,                      "TERMINATING CONDITION", SDL_APP_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
static int32_t sdl_mcrc_full_cpu_test(void)
{
    int32_t result;
    int32_t retVal=0;
    SDL_MCRC_DataConfig_t mcrcData;
    uint32_t i;
    uint32_t *pMCRCData;
    uint32_t useCase;

    for (useCase=0; useCase < MCRC_NUM_USE_CASES; useCase++)
    {
        DebugP_log("\n MCRC FULL CPU mode : starting\r\n");

        mcrcData.pMCRCData       = (uint32_t *)params[useCase].sourceMemory;
        mcrcData.size            = params[useCase].dataSize;
        mcrcData.dataBitSize     = SDL_MCRC_DATA_32_BIT;
        SDL_MCRC_Signature_t  sectSignVal;

        result = SDL_MCRC_init(params[useCase].instance,params[useCase].mcrcChannelNumber,
                               params[useCase].mcrcWatchdogPreload,params[useCase].mcrcBlockPreload);
        if (result == SDL_PASS)
        {
            result = SDL_MCRC_channelReset(params[useCase].instance,params[useCase].mcrcChannelNumber);
        }
        if (result == SDL_PASS)
        {
            result = SDL_MCRC_config(params[useCase].instance,params[useCase].mcrcChannelNumber,params[useCase].mcrcPatternCount,
                        params[useCase].mcrcSectorCount, params[useCase].mcrcMode);
        }

        pMCRCData = (uint32_t *)mcrcData.pMCRCData;
        for (i = 0; i < (mcrcData.size / 4U); i++)
        {
            pMCRCData[i] = i;
        }

        result = SDL_MCRC_computeSignCPUmode(params[useCase].instance,
                                params[useCase].mcrcChannelNumber,
                                &mcrcData, &sectSignVal);
        if (result == SDL_PASS)
        {
            /*
             * Check if the generated MCRC signature value
             * matches with the reference signaure vaule
             */
             if(useCase == 0)
             {
                if ((sectSignVal.regH == params[0].mcrcSignHigh) &&
                    (sectSignVal.regL == params[0].mcrcSignLow))
                {
                    result = SDL_PASS;
                }
                else{
                    result = SDL_EFAIL;
                }
             }
        }

        if (result != SDL_PASS)
        {
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            if (params[useCase].instance == MCRC0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification failed for the instance MCRC0 \r\n");
            }
#elif defined (SOC_AM64X)
            if (params[useCase].instance == MCU_MCRC64_0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification failed for the instance MCU_MCRC64_0 \r\n");
            }
#elif defined(SOC_AWR294X) || defined(SOC_AM273X)
            if (params[useCase].instance == MCRC_INSTANCE )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification failed\r\n");
            }
#endif
#if defined (SOC_AM243X)
            if (params[useCase].instance == MCU_MCRC64_0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification failed for the instance MCU_MCRC64_0 \r\n");
            }
#endif
            retVal = SDL_EFAIL;
        }
        else
        {
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
            if (params[useCase].instance == MCRC0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification done successfully for the instance MCRC0 \r\n ");
            }
#elif defined (SOC_AM64X)
            if (params[useCase].instance == MCU_MCRC64_0  )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification done successfully for the instance MCU_MCRC64_0  \r\n ");
            }
#elif defined(SOC_AWR294X) || defined(SOC_AM273X)
            if (params[useCase].instance == MCRC_INSTANCE )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification done successfull\r\n");
            }
#endif
#if defined(SOC_AM243X)
            if (params[useCase].instance == MCU_MCRC64_0 )
            {
                DebugP_log("\n Full_CPU mode MCRC signature verification done successfully for the instance MCU_MCRC64_0 \r\n ");
            }
#endif
            retVal = SDL_PASS;
        }

        if (retVal == SDL_EFAIL)
        {
            break;
        }
    }

    return retVal;
}


/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void mcrc_full_cpu_main(void *args)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    int32_t    i;

    /* Init Dpl */
	Drivers_open();
	Board_driversOpen();
    result = SDL_TEST_dplInit();
    if (result != SDL_PASS)
    {
       DebugP_log("Error: DPL Init Failed\n");
    }

    DebugP_log("\n MCRC Application\r\n");

    for ( i = 0; sdlmcrcAppList[i].application != NULL; i++)
    {
        result = sdlmcrcAppList[i].application();
        sdlmcrcAppList[i].status = result;
    }

    result = SDL_APP_PASS;
    for ( i = 0; sdlmcrcAppList[i].application != NULL; i++)
    {
        if (sdlmcrcAppList[i].status != SDL_APP_PASS)
        {
            DebugP_log("Applications Name: %s  FAILED\r\n", sdlmcrcAppList[i].name);
            result = SDL_APP_FAILED;
            break;
        }
        else
        {
            DebugP_log("Applications Name: %s  PASSED\r\n", sdlmcrcAppList[i].name);
        }
    }

    if (result == SDL_APP_PASS)
    {
        DebugP_log("\n All tests have passed\r\n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed\r\n");
    }
	Board_driversClose();
	Drivers_close();
}

/* Nothing past this point */

