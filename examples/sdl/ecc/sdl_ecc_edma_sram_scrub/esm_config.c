/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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
 *  \file     esm_config.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the Error Signal Module (ESM) Module application.
 *
 *  \details  ESM Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "edma_rti_sram_scrub.h"
#include <drivers/soc.h>
#include <kernel/dpl/HwiP.h>
#include <sdl/esm/v0/v0_0/sdlr_esm.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define ECC_SEC_INT                     (19U)
#define ECC_DED_INT                     (20U)
#define ESM_GROUPS                      (4U)
#define CFG_ERR_INT                     (0U)
#define LOW_PRIO_INT                    (1U)
#define HIGH_PRIO_INT                   (2U)
#define GET_EVENT_GROUP(event)          ((event) / 32U)
#define GET_EVENT_BIT(event)            ((event) % 32U)
#define SDL_ESM_HI_PRI_RESETVAL         (0xFFFFFFFFU)
#define SDL_ESM_LOW_PRI_RESETVAL        (0xFFFFFFFFU)

/* ========================================================================== */
/*                  Function Declarations                                     */
/* ========================================================================== */

void ESM_configRegisterCorruptionISR(void* args);
void ESM_lowPriorityISR(void* args);
void ESM_highPriorityISR(void* args);

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

/**
 * @brief Disable the ESM module and clear all interrupts.
 */
static void disableESM()
{
    // Reset module
    ESM_REGISTERS->SFT_RST = SDL_ESM_SFT_RST_KEY_MASK;

    // Disable configuration, high, low and critical interrupts
    ESM_REGISTERS->EN = ESM_EN_DISABLE_VALUE;

    // Loop through ESM groups and clear all interrupt enables and error pin influence enables
    for (uint8_t i = 0U; i < ESM_GROUPS; i++)
    {
        ESM_REGISTERS->ERR_GRP[i].INTR_EN_CLR   = 0xFFEFFFFFU;   // Disable interrupt
        ESM_REGISTERS->ERR_GRP[i].PIN_EN_CLR    = 0xFFEFFFFFU;   // Disable output pin
        ESM_REGISTERS->ERR_GRP[i].STS           = 0xFFEFFFFFU;   // Signal status to clear pretriggered events
    }
}

/**
 * @brief Initialize ESMDriver module
 */
static void setupESM()
{
    // Always disable ESM before configuring registers
    disableESM();

    // Enable ESM interrupts
    ESM_REGISTERS->EN = ESM_EN_ENABLE_VALUE;

    // Enable interrupts SEC/DED, assign SEC to low priority ESM and DED to high priority ESM
    ESM_REGISTERS->ERR_GRP[GET_EVENT_GROUP((uint8_t)ECC_SEC_INT)].INTR_EN_SET |= (1UL << GET_EVENT_BIT((uint8_t)ECC_SEC_INT));
    ESM_REGISTERS->ERR_GRP[GET_EVENT_GROUP((uint8_t)ECC_SEC_INT)].INT_PRIO &= ~(1UL << GET_EVENT_BIT((uint8_t)ECC_SEC_INT));

    ESM_REGISTERS->ERR_GRP[GET_EVENT_GROUP((uint8_t)ECC_DED_INT)].INTR_EN_SET |= (1UL << GET_EVENT_BIT((uint8_t)ECC_DED_INT));
    ESM_REGISTERS->ERR_GRP[GET_EVENT_GROUP((uint8_t)ECC_DED_INT)].INT_PRIO |= (1UL << GET_EVENT_BIT((uint8_t)ECC_DED_INT));

    /* ESM 19 - SET(INT_SET) and CLR(INT_PRIO)
       ESM_20 - SET(INT_SET, INT_PRIO) */
    HW_WR_REG32(0x52D00408, 0x00180000);
    HW_WR_REG32(0x52D00410, 0x00100000);
}

/**
 * @brief Sets up ESM config/low/high priority ISRs
 * 
 */
static void setupESMInterrupts(void)
{
    HwiP_Params hwiParams;
    HwiP_Object hwiObject;

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum   = CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_LOW;
    hwiParams.callback = ESM_lowPriorityISR;
    hwiParams.isPulse  = 0;
    hwiParams.priority = 5;
    (void)HwiP_construct(&hwiObject, &hwiParams);

    HwiP_Params hwiParams1;
    HwiP_Object hwiObject1;

    HwiP_Params_init(&hwiParams1);
    hwiParams1.intNum   = CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_HI;
    hwiParams1.callback = ESM_highPriorityISR;
    hwiParams1.isPulse  = 0;
    hwiParams1.priority = 4;
    (void)HwiP_construct(&hwiObject1, &hwiParams1);

    HwiP_Params hwiParams2;
    HwiP_Object hwiObject2;

    HwiP_Params_init(&hwiParams2);
    hwiParams2.intNum   = CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_CFG;
    hwiParams2.callback = ESM_configRegisterCorruptionISR;
    hwiParams2.isPulse  = 0;
    hwiParams2.priority = 5;
    (void)HwiP_construct(&hwiObject2, &hwiParams2);
}

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * Initialize ESMDriver module
 */
void ESM_init(void)
{
     SDL_cleartcmStatusRegs(0x7);
    // Configure and enable ESM
    setupESM();
    setupESMInterrupts();
}

/* ========================================================================== */
/*                          Interrupt Handlers                                */
/* ========================================================================== */

void ESM_configRegisterCorruptionISR(void* args)
{
    (void)(args);

    ESM_REGISTERS->ERR_STS |= SDL_ESM_ERR_STS_MSK_MASK; // Clear interrupt for all event groups
    ESM_REGISTERS->EOI |= (CFG_ERR_INT & SDL_ESM_EOI_KEY_MASK);
}

void ESM_lowPriorityISR(void* args)
{
    (void)(args);
    const uint32_t interruptStatus = ESM_REGISTERS->LOW_PRI;

    // Interrupt is no longer asserted
    if(interruptStatus == SDL_ESM_LOW_PRI_RESETVAL)
    { 
        return; 
    }
    else 
    {
        uint32_t levelInterrupt = (interruptStatus & SDL_ESM_LOW_PRI_LVL_MASK);

        // service interrupt based on source
        switch(levelInterrupt)
        {
            case ECC_SEC_INT:
                App_serviceSecInterrupt();
                break;
            default:
                // shouldn't enter here
                break;
        }

        // clear interrupt bit
        ESM_REGISTERS->ERR_GRP[GET_EVENT_GROUP((uint8_t)levelInterrupt)].STS |= (1UL << GET_EVENT_BIT((uint8_t)levelInterrupt));
        
        ESM_REGISTERS->EOI |= (LOW_PRIO_INT & SDL_ESM_EOI_KEY_MASK);
    }
}

void ESM_highPriorityISR(void* args)
{
    (void)(args);
    const uint32_t interruptStatus = ESM_REGISTERS->HI_PRI;

    // Interrupt is no longer asserted
    if(interruptStatus == SDL_ESM_HI_PRI_RESETVAL)
    { 
        return; 
    }
    else 
    {
        uint32_t levelInterrupt = (interruptStatus & SDL_ESM_HI_PRI_LVL_MASK);

        // service interrupt based on source
        switch(levelInterrupt)
        {
            case ECC_DED_INT:
                break;
            default:
                // shouldn't enter here
                break;
        }

        // clear interrupt bit
        ESM_REGISTERS->ERR_GRP[GET_EVENT_GROUP((uint8_t)levelInterrupt)].STS |= (1UL << GET_EVENT_BIT((uint8_t)levelInterrupt));
        ESM_REGISTERS->EOI |= (HIGH_PRIO_INT & SDL_ESM_EOI_KEY_MASK);
    }
}

/* Nothing past this point */
