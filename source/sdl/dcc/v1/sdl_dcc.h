/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *
 *  \defgroup SDL_DCC_API SDL Dual Clock Comparator(DCC)
 *  \ingroup SDL_DCC_MODULE
 *
 *   Provides the APIs for DCC.
 *  @{
 */

/**
 *  \file     sdl_dcc.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of DCC.
 *            This also contains some related macros.
 */

#ifndef SDL_DCC_H_
#define SDL_DCC_H_



/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */


#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/hw_types.h>
#include <sdl/include/sdl_types.h>

#include <sdl/dcc/v1/sdl_dcc.h>
#include <sdl/dcc/v1/soc/sdl_dcc_soc.h>




#ifdef __cplusplus
extern "C" {
#endif

/**
@defgroup SDL_DCC_MACROS DCC Macros
@ingroup SDL_DCC_API
*/

/**
@defgroup SDL_DCC_ENUM DCC Enumerated Data Types
@ingroup SDL_DCC_API
*/

/**
@defgroup SDL_DCC_FUNCTIONS DCC Functions
@ingroup SDL_DCC_API
*/

/**
@defgroup SDL_DCC_DATASTRUCT DCC Data Structures
@ingroup SDL_DCC_API
*/

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**

@addtogroup SDL_DCC_MACROS
@{
*/

/**
 * \brief  Macro defines maximum value of count for clock source 0.
 */
#define DCC_SRC0_COUNT_MAX      (0xFFFFFU)
/**
 * \brief  Macro defines maximum value of valid count for clock source 0.
 */
#define DCC_SRC0_VALID_MAX      (0x0FFFFU)
/**
 * \brief  Macro defines maximum value of count for clock source 1.
 */
#define DCC_SRC1_COUNT_MAX      (0xFFFFFU)

#define MIN_CLK0_VLD_SEED       (4u)

/** @} */



/**

@addtogroup SDL_DCC_ENUM
@{
*/

/**
 *  \anchor SDL_DCC_mode
 *  \name DCC Operation Mode
 *  @{
 */

/**
 * \brief  Enum to select the DCC Operation Mode.
 *
 *         DCC can either operate in single shot or continuous mode.
 */
typedef uint32_t SDL_DCC_mode;

#define SDL_DCC_MODE_SINGLE_SHOT     (DCC_DCCGCTRL_SINGLESHOT_MODE)
/**< Stop counting when counter0 and valid0 both reach zero */
//#define SDL_DCC_MODE_SINGLE_SHOT_2     (DCC_DCCGCTRL_SINGLESHOT_MODE2) //not applicable for am273x
/**< Stop counting when counter1 reaches zero */
#define SDL_DCC_MODE_CONTINUOUS        (DCC_DCCGCTRL_SINGLESHOT_DISABLE)
/**< Continuously repeat (until error) */
/** @} */

/**
 *  \anchor SDL_DCC_clkSrc0
 *  \name DCC Clock source of COUNT0
 *  @{
 */
/**
 * \brief  Enum to select the COUNT0 clock source.
 */
typedef uint32_t SDL_DCC_clkSrc0;

#define SDL_DCC_CLK0_SRC_CLOCK0_0               (DCC_DCCCLKSRC0_CLKSRC0_0)
/**< SYS_CLK1 is selected as source for COUNT0 */
#define SDL_DCC_CLK0_SRC_CLOCK0_1               (DCC_DCCCLKSRC0_CLKSRC0_1)
/**< SYS_CLK2 is selected as source for COUNT0 */
#define SDL_DCC_CLK0_SRC_CLOCK0_2               (DCC_DCCCLKSRC0_CLKSRC0_2)
/**< XREF_CLK is selected as source for COUNT0 */
/** @} */

/**
 *  \anchor SDL_DCC_clkSrc1
 *  \name DCC Clock source of COUNT1
 *  @{
 */
/**
 * \brief  Enum to select the COUNT1 clock source.
 */
typedef uint32_t SDL_DCC_clkSrc1;

#define SDL_DCC_CLK1_SRC_CLOCKSRC0               (DCC_DCCCLKSRC1_CLKSRC_0)
/**< TEST_CLK0 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC1               (DCC_DCCCLKSRC1_CLKSRC_1)
/**< TEST_CLK1 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC2               (DCC_DCCCLKSRC1_CLKSRC_2)
/**< TEST_CLK2 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC3               (DCC_DCCCLKSRC1_CLKSRC_3)
/**< TEST_CLK3 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC4               (DCC_DCCCLKSRC1_CLKSRC_4)
/**< TEST_CLK4 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC5               (DCC_DCCCLKSRC1_CLKSRC_5)
/**< TEST_CLK5 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC6               (DCC_DCCCLKSRC1_CLKSRC_6)
/**< TEST_CLK6 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC7               (DCC_DCCCLKSRC1_CLKSRC_7)
/**< TEST_CLK7 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_FICLK                   (SDL_DCC2_DCCCLKSRC1_CLKSRC_OTHER)
/**< OTHER_CLK is selected as source for COUNT1 */
/** @} */


/**
 *  \anchor SDL_DCC_intrType
 *  \name DCC Interrupt type
 *  @{
 */
/**
 * \brief  Enum for DCC interrupts.
 */
typedef uint32_t SDL_DCC_intrType;

#define SDL_DCC_INTERRUPT_ERR              (0x0U)
/**< The error signal */
#define SDL_DCC_INTERRUPT_DONE             (0x1U)
/**< Done interrupt signal */
/** @} */

/** @} */

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */


/**

@addtogroup SDL_DCC_DATASTRUCT
@{
*/
/**
 * \brief  Structure containing parameters for DCC module configuration.
 */
typedef struct SDL_DCC_config_st
{
    SDL_DCC_mode mode;
    /**< Mode of operation for DCC module.
     *  Refer enum SDL_DCC_mode
     */
    SDL_DCC_clkSrc0 clk0Src;
    /**< Clock source for COUNT0 i.e. reference clock.
     *  Refer enum SDL_DCC_clkSrc0
     */
    uint32_t clk1Src;
    /**< Clock source for COUNT1 i.e. clock signal to be tested.
     *  Refer enum #SDL_DCC_clkSrc1.
     */
    uint32_t clk0Seed;
    /**< Preload value/seed value for COUNT0 */
    uint32_t clk0ValidSeed;
    /**< Preload value/seed value for VALID0 */
    uint32_t clk1Seed;
    /**< Preload value/seed value for COUNT1 */
}SDL_DCC_config;


typedef struct
{
    bool doneIntr;
    /* Indicates if a done interrupt is currently pending */
    bool errIntr;
    /* Indicates if an error interrupt is currently pending */
    SDL_DCC_config config;
    /* Pointer to DCC config structure */
    uint32_t clk0Cnt;
    /* Current COUNT0 value of clock source 0 */
    uint32_t clk0Valid;
    /* Current VALID0 value */
    uint32_t clk1Cnt;
    /* Current COUNT1 value of clock source 1 */
}SDL_DCC_Status;


typedef struct SDL_DCC_staticRegs_st
{
    uint32_t  DCC_REV;
    /* DCC revision register */
    uint32_t DCC_CNTSEED0;
    /* COUNT0 SEED register */
    uint32_t DCC_VALIDSEED0;
    /* VALID0 SEED register */
    uint32_t DCC_CNTSEED1;
    /* COUNT1 SEED register */
    uint32_t DCC_CLKSRC1;
    /* CLOCK SOURCE1 register */
    uint32_t DCC_CLKSRC0;
    /* CLOCK SOURCE0 register */
}SDL_DCC_staticRegs;

/** @} */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**

@addtogroup SDL_DCC_FUNCTIONS
@{
*/

/**
 * \brief   This API is used to configure DCC module.
 *
 * \param   instance        Holds the instance for DCC module.
 *
 * \param   pConfig         pointer to the DCC config structure
 *
 * NOTE:    If the value of 'seedValid0' is less then 4 then it will programmed
 *          as 4. Since minimum programmable value of the 'seedValid0' is 4.
 */
int32_t SDL_DCC_configure(SDL_DCC_Inst instance, const SDL_DCC_config *pConfig);

/**
 * \brief   This API is used to verify the configuration for DCC module.
 *
 * \param   instance        Holds the instance for DCC module.
 *
 * \param   pConfig         pointer to the DCC config structure
 */
int32_t SDL_DCC_verifyConfig(SDL_DCC_Inst instance, const SDL_DCC_config *pConfig);
/**
 * \brief   This API is used to enable the DCC module.
 *
 * \param   instance        Holds the instance for DCC module.
 */
int32_t SDL_DCC_enable(SDL_DCC_Inst instance);
/**
 * \brief   This API is used to disable the DCC module.
 *
 * \param   instance        Holds the instance for DCC module.
 */
int32_t SDL_DCC_disable(SDL_DCC_Inst instance);
/**
 * \brief   This API is used to get the stauts of DCC module.
 *
 * \param   instance        Holds the instance for DCC module.
 *
 * \param   pStatus         pointer to the SDL_DCC_status structure
 */
int32_t SDL_DCC_getStatus(SDL_DCC_Inst instance, SDL_DCC_Status *pStatus);
/**
 * \brief   This API is used to Enable the interrupts.
 *
 * \param   instance        Holds the instance for DCC module.
 *
 * \param   intr            Interrupts to be enabled.
 */
int32_t SDL_DCC_enableIntr(SDL_DCC_Inst instance, SDL_DCC_intrType intr);
/**
 * \brief   This API is used to clear the interrupts.
 *
 * \param   instance         Holds the instance for DCC module.
 *
 * \param   intr             Interrupts to disable.
 */
int32_t SDL_DCC_clearIntr(SDL_DCC_Inst instance, SDL_DCC_intrType intr);
/**
 * \brief   This API is used to get the value of static registers for DCC module
 *
 * \param   instance         Holds the instance for DCC module.
 *
 * \param   pStaticRegs      Pointer to the SDL_DCC_staticRegs structure.
 */
int32_t SDL_DCC_getStaticRegs(SDL_DCC_Inst instance, SDL_DCC_staticRegs *pStaticRegs);

/** @} */

/** @} */

/****************************************************************************************************
* Register Definitions
****************************************************************************************************/

#define DCC_DCCGCTRL                                                (0x0U)
#define DCC_DCCREV                                                  (0x4U)
#define DCC_DCCCNTSEED0                                             (0x8U)
#define DCC_DCCVALIDSEED0                                           (0xcU)
#define DCC_DCCCNTSEED1                                             (0x10U)
#define DCC_DCCSTAT                                                 (0x14U)
#define DCC_DCCCNT0                                                 (0x18U)
#define DCC_DCCVALID0                                               (0x1cU)
#define DCC_DCCCNT1                                                 (0x20U)
#define DCC_DCCCLKSRC1                                              (0x24U)
#define DCC_DCCCLKSRC0                                              (0x28U)

/****************************************************************************************************
* Field Definition Macros
****************************************************************************************************/

#define DCC_DCCGCTRL_DCCENA_SHIFT                                                       (0U)
#define DCC_DCCGCTRL_DCCENA_MASK                                                        (0x0000000fU)
#define DCC_DCCGCTRL_DCCENA_ENABLE                                                      (0xAU)
#define DCC_DCCGCTRL_DCCENA_DISABLE                                                     (0x5U)

#define DCC_DCCGCTRL_ERRENA_SHIFT                                                       (4U)
#define DCC_DCCGCTRL_ERRENA_MASK                                                        (0x000000f0U)
#define DCC_DCCGCTRL_ERRENA_ENABLE                                                      (0xAU)
#define DCC_DCCGCTRL_ERRENA_DISABLE                                                     (0x5U)

#define DCC_DCCGCTRL_SINGLESHOT_SHIFT                                                   (8U)
#define DCC_DCCGCTRL_SINGLESHOT_MASK                                                    (0x00000f00U)
#define DCC_DCCGCTRL_SINGLESHOT_MODE                                                    (0xAU)

#define DCC_DCCGCTRL_SINGLESHOT_DISABLE                                                 (0x5U)

#define DCC_DCCGCTRL_DONEENA_SHIFT                                                      (12U)
#define DCC_DCCGCTRL_DONEENA_MASK                                                       (0x0000f000U)
#define DCC_DCCGCTRL_DONEENA_ENABLE                                                     (0xAU)
#define DCC_DCCGCTRL_DONEENA_DISABLE                                                    (0x5U)

#define DCC_DCCGCTRL_RES_SHIFT                                                          (16U)
#define DCC_DCCGCTRL_RES_MASK                                                           (0xffff0000U)

#define DCC_DCCREV_MINOR_SHIFT                                                          (0U)
#define DCC_DCCREV_MINOR_MASK                                                           (0x0000003fU)

#define DCC_DCCREV_CUSTOM_SHIFT                                                         (6U)
#define DCC_DCCREV_CUSTOM_MASK                                                          (0x000000c0U)

#define DCC_DCCREV_MAJOR_SHIFT                                                          (8U)
#define DCC_DCCREV_MAJOR_MASK                                                           (0x00000700U)

#define DCC_DCCREV_RTL_SHIFT                                                            (11U)
#define DCC_DCCREV_RTL_MASK                                                             (0x0000f800U)

#define DCC_DCCREV_FUNC_SHIFT                                                           (16U)
#define DCC_DCCREV_FUNC_MASK                                                            (0x0fff0000U)

#define DCC_DCCREV_RES_SHIFT                                                            (28U)
#define DCC_DCCREV_RES_MASK                                                             (0x30000000U)

#define DCC_DCCREV_SCHEME_SHIFT                                                         (30U)
#define DCC_DCCREV_SCHEME_MASK                                                          (0xc0000000U)

#define DCC_DCCCNTSEED0_COUNTSEED0_SHIFT                                                (0U)
#define DCC_DCCCNTSEED0_COUNTSEED0_MASK                                                 (0x000fffffU)

#define DCC_DCCCNTSEED0_RES_SHIFT                                                       (20U)
#define DCC_DCCCNTSEED0_RES_MASK                                                        (0xfff00000U)

#define DCC_DCCVALIDSEED0_VALIDSEED0_SHIFT                                              (0U)
#define DCC_DCCVALIDSEED0_VALIDSEED0_MASK                                               (0x0000ffffU)

#define DCC_DCCVALIDSEED0_RES_SHIFT                                                     (16U)
#define DCC_DCCVALIDSEED0_RES_MASK                                                      (0xffff0000U)

#define DCC_DCCCNTSEED1_COUNTSEED1_SHIFT                                                (0U)
#define DCC_DCCCNTSEED1_COUNTSEED1_MASK                                                 (0x000fffffU)

#define DCC_DCCCNTSEED1_RES_SHIFT                                                       (20U)
#define DCC_DCCCNTSEED1_RES_MASK                                                        (0xfff00000U)

#define DCC_DCCSTAT_ERRFLG_SHIFT                                                        (0U)
#define DCC_DCCSTAT_ERRFLG_MASK                                                         (0x00000001U)
#define DCC_DCCSTAT_ERRFLG_DISABLE                                                      (1U)

#define DCC_DCCSTAT_DONEFLG_SHIFT                                                       (1U)
#define DCC_DCCSTAT_DONEFLG_MASK                                                        (0x00000002U)
#define DCC_DCCSTAT_DONEFLG_DISABLE                                                     (1U)

#define DCC_DCCSTAT_RES_SHIFT                                                           (2U)
#define DCC_DCCSTAT_RES_MASK                                                            (0xfffffffcU)

#define DCC_DCCCNT0_COUNT0_SHIFT                                                        (0U)
#define DCC_DCCCNT0_COUNT0_MASK                                                         (0x000fffffU)

#define DCC_DCCCNT0_RES_SHIFT                                                           (20U)
#define DCC_DCCCNT0_RES_MASK                                                            (0xfff00000U)

#define DCC_DCCVALID0_VALID0_SHIFT                                                      (0U)
#define DCC_DCCVALID0_VALID0_MASK                                                       (0x0000ffffU)

#define DCC_DCCVALID0_RES_SHIFT                                                         (16U)
#define DCC_DCCVALID0_RES_MASK                                                          (0xffff0000U)

#define DCC_DCCCNT1_COUNT1_SHIFT                                                        (0U)
#define DCC_DCCCNT1_COUNT1_MASK                                                         (0x000fffffU)

#define DCC_DCCCNT1_RES_SHIFT                                                           (20U)
#define DCC_DCCCNT1_RES_MASK                                                            (0xfff00000U)

#define DCC_DCCCLKSRC1_CLKSRC_SHIFT                                                     (0U)
#define DCC_DCCCLKSRC1_CLKSRC_MASK                                                      (0x0000000fU)
#define DCC_DCCCLKSRC1_CLKSRC_0                                                         (0x0U)
#define DCC_DCCCLKSRC1_CLKSRC_1                                                         (0x1U)
#define DCC_DCCCLKSRC1_CLKSRC_2                                                         (0x2U)
#define DCC_DCCCLKSRC1_CLKSRC_3                                                         (0x3U)
#define DCC_DCCCLKSRC1_CLKSRC_4                                                         (0x4U)
#define DCC_DCCCLKSRC1_CLKSRC_5                                                         (0x5U)
#define DCC_DCCCLKSRC1_CLKSRC_6                                                         (0x6U)
#define DCC_DCCCLKSRC1_CLKSRC_7                                                         (0x7U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_OTHER                                                (0xFU)

#define DCC_DCCCLKSRC1_RES1_SHIFT                                                       (4U)
#define DCC_DCCCLKSRC1_RES1_MASK                                                        (0x00000ff0U)

#define DCC_DCCCLKSRC1_KEY_SHIFT                                                        (12U)
#define DCC_DCCCLKSRC1_KEY_MASK                                                         (0x0000f000U)
#define DCC_DCCCLKSRC1_KEY_ENABLE                                                       (0xAU)
#define DCC_DCCCLKSRC1_KEY_DISABLE                                                      (0x0U)

#define DCC_DCCCLKSRC1_RES0_SHIFT                                                       (16U)
#define DCC_DCCCLKSRC1_RES0_MASK                                                        (0xffff0000U)

#define DCC_DCCCLKSRC0_CLKSRC0_SHIFT                                                    (0U)
#define DCC_DCCCLKSRC0_CLKSRC0_MASK                                                     (0x0000000fU)
#define DCC_DCCCLKSRC0_CLKSRC0_0                                                        (0x0U)
#define DCC_DCCCLKSRC0_CLKSRC0_1                                                        (0x1U)
#define DCC_DCCCLKSRC0_CLKSRC0_2                                                        (0x2U)

#define DCC_DCCCLKSRC0_RES1_SHIFT                                                       (4U)
#define DCC_DCCCLKSRC0_RES1_MASK                                                        (0xfffffff0)

#define DCC_DCCCLKSRC0_KEY_SHIFT                                                        (12U)
#define DCC_DCCCLKSRC0_KEY_MASK                                                         (0x0000f000U)
#define DCC_DCCCLKSRC0_KEY_ENABLE                                                       (0xAU)
#define DCC_DCCCLKSRC0_KEY_DISABLE                                                      (0x0U)

#define DCC_DCCCLKSRC0_RES0_SHIFT                                                       (16U)
#define DCC_DCCCLKSRC0_RES0_MASK                                                        (0xffff0000U)

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif

