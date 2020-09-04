/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  @defgroup SDL_DCC_MODULE APIs for SDL Dual Clock Comparator(DCC) 
 *  @ingroup SDL_MODULE
 *  @{
 */

#ifndef SDL_DCC_H_
#define SDL_DCC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>

#include <sdl/dcc/v0/sdlr_dcc2.h>
#include <sdl/dcc/v0/soc/sdl_dcc_soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

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

#define DCC_MIN_CLK0_VLD_SEED       (4u)

/**
 * \brief  The number of valid Clk0 Src options
 */
#define SDL_DCC_CLK0_SRC_NUM        (3u)

/**
 * \brief  The number of valid clk1 Src options
 */
#define SDL_DCC_CLK1_SRC_NUM        (10u)

/**
 * \brief  The number of valid mode options
 */
#define SDL_DCC_MODES_NUM           (3u)

/**
 *  \anchor SDL_DCC_Mode
 *  \name DCC Operation Mode
 *  @{
 */

/**
 * \brief  Enum to select the DCC Operation Mode.
 *         DCC can either operate in single shot or continuous mode.
 */
typedef uint32_t SDL_DCC_Mode;

#define SDL_DCC_MODE_SINGLE_SHOT_1     (SDL_DCC2_DCCGCTRL_SINGLESHOT_MODE1)
/**< Stop counting when counter0 and valid0 both reach zero */
#define SDL_DCC_MODE_SINGLE_SHOT_2     (SDL_DCC2_DCCGCTRL_SINGLESHOT_MODE2)
/**< Stop counting when counter1 reaches zero */
#define SDL_DCC_MODE_CONTINUOUS        (SDL_DCC2_DCCGCTRL_SINGLESHOT_DISABLE)
/**< Continuously repeat (until error) */
/** @} */

/**
 *  \anchor SDL_DCC_ClkSrc0
 *  \name DCC Clock source of COUNT0
 *  @{
 */
/**
 * \brief  Enum to select the COUNT0 clock source.
 */
typedef uint32_t SDL_DCC_ClkSrc0;

#define SDL_DCC_CLK0_SRC_CLOCK0_0               (SDL_DCC2_DCCCLKSRC0_CLKSRC0_0)
/**< SYS_CLK1 is selected as source for COUNT0 */
#define SDL_DCC_CLK0_SRC_CLOCK0_1               (SDL_DCC2_DCCCLKSRC0_CLKSRC0_1)
/**< SYS_CLK2 is selected as source for COUNT0 */
#define SDL_DCC_CLK0_SRC_CLOCK0_2               (SDL_DCC2_DCCCLKSRC0_CLKSRC0_2)
/**< XREF_CLK is selected as source for COUNT0 */
/** @} */

/**
 *  \anchor SDL_DCC_ClkSrc1
 *  \name DCC Clock source of COUNT1
 *  @{
 */
/**
 * \brief  Enum to select the COUNT1 clock source.
 */
typedef uint32_t SDL_DCC_ClkSrc1;
#define SDL_DCC_CLK1_SRC_CLOCK1                  (SDL_DCC2_DCCCLKSRC1_CLKSRC_0)
/**< TEST_CLK1 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC0               (SDL_DCC2_DCCCLKSRC1_CLKSRC_1)
/**< TEST_CLK0 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC1               (SDL_DCC2_DCCCLKSRC1_CLKSRC_2)
/**< TEST_CLK1 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC2               (SDL_DCC2_DCCCLKSRC1_CLKSRC_3)
/**< TEST_CLK2 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC3               (SDL_DCC2_DCCCLKSRC1_CLKSRC_4)
/**< TEST_CLK3 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC4               (SDL_DCC2_DCCCLKSRC1_CLKSRC_5)
/**< TEST_CLK4 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC5               (SDL_DCC2_DCCCLKSRC1_CLKSRC_6)
/**< TEST_CLK5 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC6               (SDL_DCC2_DCCCLKSRC1_CLKSRC_7)
/**< TEST_CLK6 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_CLOCKSRC7               (SDL_DCC2_DCCCLKSRC1_CLKSRC_8)
/**< TEST_CLK7 is selected as source for COUNT1 */
#define SDL_DCC_CLK1_SRC_FICLK                   (SDL_DCC2_DCCCLKSRC1_CLKSRC_OTHER)
/**< OTHER_CLK is selected as source for COUNT1 */
/** @} */

/**
 *  \anchor SDL_DCC_IntrType
 *  \name DCC Interrupt type
 *  @{
 */
/**
 * \brief  Enum for DCC interrupts.
 */
typedef uint32_t SDL_DCC_IntrType;

#define SDL_DCC_INTERRUPT_ERR              (0x0U)
/**< The error signal */
#define SDL_DCC_INTERRUPT_DONE             (0x1U)
/**< Done interrupt signal */
/** @} */

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 * \brief  Structure containing parameters for DCC module configuration.
 */
typedef struct {
    SDL_DCC_Mode mode;
    /**< Mode of operation for DCC module.
     *  Refer enum SDL_DCC_mode
     */
    SDL_DCC_ClkSrc0 clk0Src;
    /**< Clock source for COUNT0 i.e. reference clock.
     *  Refer enum SDL_DCC_clkSrc0
     */
    SDL_DCC_ClkSrc1 clk1Src;
    /**< Clock source for COUNT1 i.e. clock signal to be tested.
     *  Refer enum #SDL_DCC_ClkSrc1.
     */
    uint32_t clk0Seed;
    /**< Preload value/seed value for COUNT0 */
    uint32_t clk0ValidSeed;
    /**< Preload value/seed value for VALID0 */
    uint32_t clk1Seed;
    /**< Preload value/seed value for COUNT1 */
}SDL_DCC_Config;


/**
 * \brief  Structure containing DCC status
 */
typedef struct {
    bool doneIntr;
    /* Indicates if a done interrupt is currently pending */
    bool errIntr;
    /* Indicates if an error interrupt is currently pending */
    SDL_DCC_Config config;
    /* DCC config structure */
    uint32_t clk0Cnt;
    /* Current COUNT0 value of clock source 0 */
    uint32_t clk0Valid;
    /* Current VALID0 value */
    uint32_t clk1Cnt;
    /* Current COUNT1 value of clock source 1 */
}SDL_DCC_Status;


/**
 * \brief Structure containing DCC Static Registers
 */
typedef struct {
    uint32_t DCC_REV;
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
}SDL_DCC_StaticRegs;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

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
int32_t SDL_DCC_configure(SDL_DCC_Inst instance, const SDL_DCC_Config *pConfig);

/**
 * \brief   This API is used to verify the configuration for DCC module.
 *
 * \param   instance        Holds the instance for DCC module.
 *
 * \param   pConfig         pointer to the DCC config structure
 */
int32_t SDL_DCC_verifyConfig(SDL_DCC_Inst instance, const SDL_DCC_Config *pConfig);

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
int32_t SDL_DCC_enableIntr(SDL_DCC_Inst instance, SDL_DCC_IntrType intr);

/**
 * \brief   This API is used to Disable the interrupts.
 *
 * \param   instance        Holds the instance for DCC module.
 *
 * \param   intr            Interrupts to be enabled.
 */
int32_t SDL_DCC_disableIntr(SDL_DCC_Inst instance, SDL_DCC_IntrType intr);

/**
 * \brief   This API is used to clear the interrupts.
 *
 * \param   instance         Holds the instance for DCC module.
 *
 * \param   intr             Interrupts to disable.
 */
int32_t SDL_DCC_clearIntr(SDL_DCC_Inst instance, SDL_DCC_IntrType intr);

/**
 * \brief   This API is used to get the value of static registers for DCC module
 *
 * \param   instance         Holds the instance for DCC module.
 *
 * \param   pStaticRegs      Pointer to the SDL_DCC_StaticRegs structure.
 */
int32_t SDL_DCC_getStaticRegs(SDL_DCC_Inst instance, SDL_DCC_StaticRegs *pStaticRegs);


/** @} */

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif
