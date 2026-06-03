/*
 * AM263x Error Signaling Module Driver
 */

#include "edma_rti_sram_scrub.h"

// Platform drivers, vendor headers
#include <drivers/soc.h>
#include <kernel/dpl/HwiP.h>
#include "sdlr_esm.h"

/******************************************************************************
 *              P R I V A T E   D A T A   D E F I N I T I O N S               *
 ******************************************************************************/

void ESM_configRegisterCorruptionISR(void* args);
void ESM_lowPriorityISR(void* args);
void ESM_highPriorityISR(void* args);

/*****************************************************************************
*                          S T A T I C   D A T A                             *
******************************************************************************/

#define ECC_SEC_INT                     (19U)
#define ECC_DED_INT                     (20U)

#define ESM_GROUPS                      (4U)

// ESM EN disable interrupt value
#define ESM_EN_DISABLE_VALUE            (0U)

// ESM EN enable interrupt value
#define ESM_EN_ENABLE_VALUE             (0b1111)
#define CFG_ERR_INT                     (0U)
#define LOW_PRIO_INT                    (1U)
#define HIGH_PRIO_INT                   (2U)

#define ESM_REGISTERS                            ((volatile SDL_esmRegs *) CSL_TOP_ESM_U_BASE)
#define GET_EVENT_GROUP(event)                   ((event) / 32U)
#define GET_EVENT_BIT(event)                     ((event) % 32U)

#define SDL_ESM_HI_PRI_RESETVAL                                            (0xFFFFFFFFU)
#define SDL_ESM_LOW_PRI_RESETVAL                                           (0xFFFFFFFFU)

/*****************************************************************************
*                    P R I V A T E   F U N C T I O N S                       *
******************************************************************************/

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
        ESM_REGISTERS->ERR_GRP[i].INTR_EN_CLR   = 0xFFFFFFFFU;   // Disable interrupt
        ESM_REGISTERS->ERR_GRP[i].PIN_EN_CLR    = 0xFFFFFFFFU;   // Disable output pin
        ESM_REGISTERS->ERR_GRP[i].STS           = 0xFFFFFFFFU;   // Signal status to clear pretriggered events
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
    hwiParams.intNum   = CSLR_R5FSS0_CORE1_INTR_ESM0_ESM_INT_LOW;
    hwiParams.callback = ESM_lowPriorityISR;
    hwiParams.isPulse  = 0;
    hwiParams.priority = 5;
    (void)HwiP_construct(&hwiObject, &hwiParams);

    HwiP_Params hwiParams1;
    HwiP_Object hwiObject1;

    HwiP_Params_init(&hwiParams1);
    hwiParams1.intNum   = CSLR_R5FSS0_CORE1_INTR_ESM0_ESM_INT_HI;
    hwiParams1.callback = ESM_highPriorityISR;
    hwiParams1.isPulse  = 0;
    hwiParams1.priority = 4;
    (void)HwiP_construct(&hwiObject1, &hwiParams1);

    HwiP_Params hwiParams2;
    HwiP_Object hwiObject2;

    HwiP_Params_init(&hwiParams2);
    hwiParams2.intNum   = CSLR_R5FSS0_CORE1_INTR_ESM0_ESM_INT_CFG;
    hwiParams2.callback = ESM_configRegisterCorruptionISR;
    hwiParams2.isPulse  = 0;
    hwiParams2.priority = 5;
    (void)HwiP_construct(&hwiObject2, &hwiParams2);
}


/****************************************************************************
*                    M O D U L E   I N T E R F A C E                        *
*****************************************************************************/

/**
 * Initialize ESMDriver module
 */
void ESM_init(void)
{
    // Configure and enable ESM
    setupESM();
    setupESMInterrupts();
}

/******************************************************************************
 *                    I N T E R R U P T   H A N D L E R S                     *
 ******************************************************************************/

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
