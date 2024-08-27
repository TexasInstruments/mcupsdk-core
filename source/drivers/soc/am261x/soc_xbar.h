/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : soc_xbar.h
*/

#ifndef SOC_XBAR_AM261X_H_
#define SOC_XBAR_AM261X_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
/**
 *  \defgroup DRV_SOC_XBAR_MODULE APIs for SOC Xbars
 *  \ingroup DRV_SOC_MODULE
 *
 * For more details and example usage, see \ref DRIVERS_SOC_PAGE
 *
 *  @{
 */


#include <stdbool.h>
#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>

#define CSL_CONTROLSS_INPUTXBAR_STEP        (CSL_CONTROLSS_INPUTXBAR_INPUTXBAR1_GSEL - CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL)
#define CSL_CONTROLSS_PWMXBAR_STEP          (CSL_CONTROLSS_PWMXBAR_PWMXBAR1_G0 - CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G0)
#define CSL_CONTROLSS_MDLXBAR_STEP          (CSL_CONTROLSS_MDLXBAR_MDLXBAR1_G0 - CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0)
#define CSL_CONTROLSS_ICLXBAR_STEP          (CSL_CONTROLSS_ICLXBAR_ICLXBAR1_G0 - CSL_CONTROLSS_ICLXBAR_ICLXBAR0_G0)
#define CSL_CONTROLSS_INTXBAR_STEP          (CSL_CONTROLSS_INTXBAR_INTXBAR1_G0 - CSL_CONTROLSS_INTXBAR_INTXBAR0_G0)
#define CSL_CONTROLSS_DMAXBAR_STEP          (CSL_CONTROLSS_DMAXBAR_DMAXBAR1_GSEL - CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL)
#define CSL_CONTROLSS_OUTPUTXBAR_STEP       (CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G0 - CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_STEP   (CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G0 - CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0)

/**
 * \brief Trip & Sync xbar: API to select input source of Input XBar
 *
 * \param base [in] Input XBar base address
 * \param out [in] Instance of Input XBar
 * \param group0_muxctl [in] Mux control to select input from group 0 mux
 * \param group1_muxctl [in] Mux control to select input from group 1 mux
 * \param group_select [in] Mux control to select group 0 or 1
 *
 */
static inline void
SOC_xbarSelectInputXBarInputSource(uint32_t base, uint8_t out, uint8_t group_select, uint8_t group0_muxctl, uint8_t group1_muxctl)
{
    HW_WR_REG32(base + CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL + (out * CSL_CONTROLSS_INPUTXBAR_STEP), group_select & CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL_GSEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0   + (out * CSL_CONTROLSS_INPUTXBAR_STEP), group0_muxctl & CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1   + (out * CSL_CONTROLSS_INPUTXBAR_STEP), group1_muxctl & CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1_SEL_MASK);

}

/**
 * \brief Trip & Sync xbar: API to select input source of Input XBar
 *
 * \param base [in] Input XBar base address
 * \param out [in] Instance of Input XBar
 * \param group0_muxctl [in] Mux control to select input from group 0 mux
 * \param group1_muxctl [in] Mux control to select input from group 1 mux
 * \param group2_muxctl [in] Mux control to select input from group 2 mux
 * \param group_select [in] Mux control to select group 0,1 or 2
 *
 */
static inline void
SOC_xbarSelectInputXBarInputSource_ext(uint32_t base, uint8_t out, uint8_t group_select, uint8_t group0_muxctl, uint8_t group1_muxctl, uint8_t group2_muxctl)
{
    HW_WR_REG32(base + CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL + (out * CSL_CONTROLSS_INPUTXBAR_STEP), group_select & CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_GSEL_GSEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0   + (out * CSL_CONTROLSS_INPUTXBAR_STEP), group0_muxctl & CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1   + (out * CSL_CONTROLSS_INPUTXBAR_STEP), group1_muxctl & CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G2   + (out * CSL_CONTROLSS_INPUTXBAR_STEP), group1_muxctl & CSL_CONTROLSS_INPUTXBAR_INPUTXBAR0_G2_SEL_MASK);

}

/**
 * \brief Trip & Sync xbar: API to read raw output signal status of all PWM XBars
 *
 * \param base [in] PWM XBar base address
 *
 * \return uint32_t PWM XBar status
 */
static inline uint32_t
SOC_xbarGetPWMXBarOutputSignalStatus(uint32_t base)
{
    return(HW_RD_REG32(base + CSL_CONTROLSS_PWMXBAR_PWMXBAR_STATUS) & CSL_CONTROLSS_PWMXBAR_PWMXBAR_STATUS_STS_MASK);
}

/**
 * \brief Trip & Sync xbar: API to configure inversion of output signal status flag (latched) of PWM XBars
 *
 * \param base [in] PWM XBar base address
 * \param invert_mask [in] Mask defining the PWM XBar output signal flags (latched) to be inverted
 *
 */
static inline void
SOC_xbarInvertPWMXBarOutputSignalBeforeLatch(uint32_t base, uint32_t invert_mask)
{
    HW_WR_REG32(base + CSL_CONTROLSS_PWMXBAR_PWMXBAR_FLAGINVERT, invert_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR_FLAGINVERT_INVERT_MASK);
}

/**
 * \brief Trip & Sync xbar: API to read latched output signal status of all PWM XBars
 *
 * \param base [in] PWM XBar base address
 *
 * \return uint32_t PWM XBar latched status flags
 */
static inline uint32_t
SOC_xbarGetPWMXBarOutputSignalLatchedFlag(uint32_t base)
{
    return(HW_RD_REG32(base + CSL_CONTROLSS_PWMXBAR_PWMXBAR_FLAG));
}

/**
 * \brief Trip & Sync xbar: API to clear output signal status flag (latched) of PWM XBars
 *
 * \param base [in] PWM XBar base address
 * \param clr [in] Mask defining the PWM XBar output signal flags (latched) to be cleared
 *
 */
static inline void
SOC_xbarClearPWMXBarOutputSignalLatchedFlag(uint32_t base, uint32_t clr)
{
    HW_WR_REG32(base + CSL_CONTROLSS_PWMXBAR_PWMXBAR_FLAG_CLR, clr);
}

/**
 * \brief Trip & Sync xbar: API to select input sources of PWM XBar
 *
 * \param base [in] PWM XBar base address
 * \param out [in] Instance of PWM XBar
 * \param group0_mask [in] Mask to OR inputs from group 0
 * \param group1_mask [in] Mask to OR inputs from group 1
 * \param group2_mask [in] Mask to OR inputs from group 2
 * \param group3_mask [in] Mask to OR inputs from group 3
 * \param group4_mask [in] Mask to OR inputs from group 4
 * \param group5_mask [in] Mask to OR inputs from group 5
 * \param group6_mask [in] Mask to OR inputs from group 6
 * \param group7_mask [in] Mask to OR inputs from group 7
 * \param group8_mask [in] Mask to OR inputs from group 8
 *
 */
static inline void
SOC_xbarSelectPWMXBarInputSource(uint32_t base, uint8_t out, uint32_t group0_mask, uint32_t group1_mask, uint32_t group2_mask, uint32_t group3_mask, uint32_t group4_mask, uint32_t group5_mask, uint32_t group6_mask, uint32_t group7_mask, uint32_t group8_mask)
{
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G0, group0_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G1, group1_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G2, group2_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G2_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G3, group3_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G3_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G4, group4_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G4_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G5, group5_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G5_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G6, group6_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G6_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G7, group7_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G7_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G8, group8_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G8_SEL_MASK);
}


/**
 * \brief Trip & Sync xbar: API to select input sources of PWM XBar
 *
 * \param base [in] PWM XBar base address
 * \param out [in] Instance of PWM XBar
 * \param group0_mask [in] Mask to OR inputs from group 0
 * \param group1_mask [in] Mask to OR inputs from group 1
 * \param group2_mask [in] Mask to OR inputs from group 2
 * \param group3_mask [in] Mask to OR inputs from group 3
 * \param group4_mask [in] Mask to OR inputs from group 4
 * \param group5_mask [in] Mask to OR inputs from group 5
 * \param group6_mask [in] Mask to OR inputs from group 6
 * \param group7_mask [in] Mask to OR inputs from group 7
 * \param group8_mask [in] Mask to OR inputs from group 8
 * \param group9_mask [in] Mask to OR inputs from group 9
 *
 */
static inline void
SOC_xbarSelectPWMXBarInputSource_ext(uint32_t base, uint8_t out, uint32_t group0_mask, uint32_t group1_mask, uint32_t group2_mask, uint32_t group3_mask, uint32_t group4_mask, uint32_t group5_mask, uint32_t group6_mask, uint32_t group7_mask, uint32_t group8_mask, uint32_t group9_mask)
{
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G0, group0_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G1, group1_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G2, group2_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G2_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G3, group3_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G3_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G4, group4_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G4_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G5, group5_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G5_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G6, group6_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G6_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G7, group7_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G7_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_PWMXBAR_STEP + CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G8, group8_mask & CSL_CONTROLSS_PWMXBAR_PWMXBAR0_G8_SEL_MASK);
}

/**
 * \brief Trip & Sync xbar: API to select input sources of MDL XBar
 *
 * \param base [in] MDL XBar base address
 * \param out [in] Instance of MDL XBar
 * \param group0_mask [in] Mask to OR inputs from group 0
 * \param group1_mask [in] Mask to OR inputs from group 1
 * \param group2_mask [in] Mask to OR inputs from group 2
 *
 */
static inline void
SOC_xbarSelectMinimumDeadBandLogicXBarInputSource(uint32_t base, uint8_t out, uint32_t group0_mask, uint32_t group1_mask, uint32_t group2_mask)
{
    //TBD: 32 bit field required?
    HW_WR_REG32(base + out*CSL_CONTROLSS_MDLXBAR_STEP + CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0, group0_mask & CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_MDLXBAR_STEP + CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G1, group1_mask & CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_MDLXBAR_STEP + CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G2, group2_mask & CSL_CONTROLSS_MDLXBAR_MDLXBAR0_G2_SEL_MASK);
}

/**
 * \brief Trip & Sync xbar: API to select input sources of ICL XBar
 *
 * \param base [in] ICL XBar base address
 * \param out [in] Instance of ICL XBar
 * \param group0_mask [in] Mask to OR inputs from group 0
 * \param group1_mask [in] Mask to OR inputs from group 1
 * \param group2_mask [in] Mask to OR inputs from group 2
 *
 */
static inline void
SOC_xbarSelectIllegalComboLogicXBarInputSource(uint32_t base, uint8_t out, uint32_t group0_mask, uint32_t group1_mask, uint32_t group2_mask)
{
    //TBD: 32 bit field required?
    HW_WR_REG32(base + out*CSL_CONTROLSS_ICLXBAR_STEP + CSL_CONTROLSS_ICLXBAR_ICLXBAR0_G0, group0_mask & CSL_CONTROLSS_ICLXBAR_ICLXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_ICLXBAR_STEP + CSL_CONTROLSS_ICLXBAR_ICLXBAR0_G1, group1_mask & CSL_CONTROLSS_ICLXBAR_ICLXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_ICLXBAR_STEP + CSL_CONTROLSS_ICLXBAR_ICLXBAR0_G2, group2_mask & CSL_CONTROLSS_ICLXBAR_ICLXBAR0_G2_SEL_MASK);
}

/**
 * \brief Trip & Sync xbar: API to select input sources of Interrupt XBar
 *
 * \param base [in] Interrupt XBar base address
 * \param out [in] Instance of Interrupt XBar
 * \param group0_mask [in] Mask to OR inputs from group 0
 * \param group1_mask [in] Mask to OR inputs from group 1
 * \param group2_mask [in] Mask to OR inputs from group 2
 * \param group3_mask [in] Mask to OR inputs from group 3
 * \param group4_mask [in] Mask to OR inputs from group 4
 * \param group5_mask [in] Mask to OR inputs from group 5
 * \param group6_mask [in] Mask to OR inputs from group 6
 *
 */
static inline void
SOC_xbarSelectInterruptXBarInputSource(uint32_t base, uint8_t out, uint32_t group0_mask, uint32_t group1_mask, uint32_t group2_mask, uint32_t group3_mask, uint32_t group4_mask, uint32_t group5_mask, uint32_t group6_mask)
{
    //TBD: 32 bit field required?
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G0, group0_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G1, group1_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G2, group2_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G3, group3_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G4, group4_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G5, group5_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G6, group6_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_MASK);
}


/**
 * \brief Trip & Sync xbar: API to select input sources of Interrupt XBar
 *
 * \param base [in] Interrupt XBar base address
 * \param out [in] Instance of Interrupt XBar
 * \param group0_mask [in] Mask to OR inputs from group 0
 * \param group1_mask [in] Mask to OR inputs from group 1
 * \param group2_mask [in] Mask to OR inputs from group 2
 * \param group3_mask [in] Mask to OR inputs from group 3
 * \param group4_mask [in] Mask to OR inputs from group 4
 * \param group5_mask [in] Mask to OR inputs from group 5
 * \param group6_mask [in] Mask to OR inputs from group 6
 * \param group7_mask [in] Mask to OR inputs from group 7
 * \param group8_mask [in] Mask to OR inputs from group 8
 * \param group9_mask [in] Mask to OR inputs from group 9
 *
 */
static inline void
SOC_xbarSelectInterruptXBarInputSource_ext(uint32_t base, uint8_t out, uint32_t group0_mask, uint32_t group1_mask, uint32_t group2_mask, uint32_t group3_mask, uint32_t group4_mask, uint32_t group5_mask, uint32_t group6_mask, uint32_t group7_mask, uint32_t group8_mask, uint32_t group9_mask)
{
    //TBD: 32 bit field required?
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G0, group0_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G1, group1_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G2, group2_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G3, group3_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G4, group4_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G5, group5_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G6, group6_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_INTXBAR_STEP + CSL_CONTROLSS_INTXBAR_INTXBAR0_G7, group7_mask & CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_MASK);
}

/**
 * \brief Trip & Sync xbar: API to select input source of DMA XBar
 *
 * \param base [in] DMA XBar base address
 * \param out [in] Instance of DMA XBar
 * \param group0_muxctl [in] Mux control to select input from group 0 mux
 * \param group1_muxctl [in] Mux control to select input from group 1 mux
 * \param group2_muxctl [in] Mux control to select input from group 2 mux
 * \param group3_muxctl [in] Mux control to select input from group 3 mux
 * \param group4_muxctl [in] Mux control to select input from group 4 mux
 * \param group5_muxctl [in] Mux control to select input from group 5 mux
 * \param group_select [in] Mux control to select group 0/1/2/3/4/5
 *
 */
static inline void
SOC_xbarSelectDMAXBarInputSource(uint32_t base, uint8_t out, uint8_t group_select, uint8_t group0_muxctl, uint8_t group1_muxctl, uint8_t group2_muxctl, uint8_t group3_muxctl, uint8_t group4_muxctl, uint8_t group5_muxctl)
{
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL , group_select &  CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL_GSEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0   , group0_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1   , group1_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2   , group2_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3   , group3_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4   , group4_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5   , group5_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5_SEL_MASK);

}


/**
 * \brief Trip & Sync xbar: API to select input source of DMA XBar
 *
 * \param base [in] DMA XBar base address
 * \param out [in] Instance of DMA XBar
 * \param group0_muxctl [in] Mux control to select input from group 0 mux
 * \param group1_muxctl [in] Mux control to select input from group 1 mux
 * \param group2_muxctl [in] Mux control to select input from group 2 mux
 * \param group3_muxctl [in] Mux control to select input from group 3 mux
 * \param group4_muxctl [in] Mux control to select input from group 4 mux
 * \param group5_muxctl [in] Mux control to select input from group 5 mux
 * \param group6_muxctl [in] Mux control to select input from group 5 mux
 * \param group_select [in] Mux control to select group 0/1/2/3/4/5/6
 *
 */
static inline void
SOC_xbarSelectDMAXBarInputSource_ext(uint32_t base, uint8_t out, uint8_t group_select, uint8_t group0_muxctl, uint8_t group1_muxctl, uint8_t group2_muxctl, uint8_t group3_muxctl, uint8_t group4_muxctl, uint8_t group5_muxctl, uint8_t group6_muxctl)
{
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL , group_select &  CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL_GSEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0   , group0_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1   , group1_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2   , group2_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3   , group3_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4   , group4_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4_SEL_MASK);
    HW_WR_REG32(base + out*CSL_CONTROLSS_DMAXBAR_STEP + CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5   , group5_muxctl & CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5_SEL_MASK);

}

/**
 * \brief Trip & Sync xbar: API to read raw output signal status of all Output XBars
 *
 * \param base [in] Output XBar base address
 *
 * \return uint32_t Output XBar status
 */
static inline uint32_t
SOC_xbarGetOutputXBarOutputSignalStatus(uint32_t base)
{
    return(HW_RD_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_STATUS)& CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_STATUS_STS_MASK);
}

/**
 * \brief Trip & Sync xbar: API to configure inversion of output signal status flag (latched) of Output XBars
 *
 * \param base [in] Output XBar base address
 * \param invert [in] Mask defining the Output XBar output signal flags (latched) to be inverted
 *
 */
static inline void
SOC_xbarInvertOutputXBarOutputSignalBeforeLatch(uint32_t base, uint32_t invert)
{
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGINVERT, invert & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGINVERT_INVERT_MASK);
}

/**
 * \brief Trip & Sync xbar: API to read latched output signal status of all Output XBars
 *
 * \param base [in] Output XBar base address
 *
 * \return uint32_t Output XBar latched status flags
 */
static inline uint32_t
SOC_xbarGetOutputXBarOutputSignalLatchedFlag(uint32_t base)
{
    return(HW_RD_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG));
}

/**
 * \brief Trip & Sync xbar: API to clear output signal status flag (latched) of Output XBars
 *
 * \param base [in] Output XBar base address
 * \param clr [in] Mask defining the Output XBar output signal flags (latched) to be cleared
 *
 */
static inline void
SOC_xbarClearOutputXBarOutputSignalLatchedFlag(uint32_t base, uint32_t clr)
{
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR, clr);
}

/**
 * \brief Trip & Sync xbar: API to force output signal status flag (latched) of Output XBars
 *
 * \param base [in] Output XBar base address
 * \param force [in] Mask defining the Output XBar output signal flags (latched) to be forced to set
 *
 */
static inline void
SOC_xbarForceOutputXBarOutputSignalLatchedFlag(uint32_t base, uint32_t force)
{
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGFORCE, force & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGFORCE_FRC_MASK);
}

/**
 * \brief Trip & Sync xbar: API to select output of Output XBars
 *
 * \param base [in] Output XBar base address
 * \param latchselect [in] Select latched / non-latched output
 *
 */
static inline void
SOC_xbarSelectLatchOutputXBarOutputSignal(uint32_t base, uint32_t latchselect)
{
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLATCH, latchselect & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLATCH_LATCHSEL_MASK);
}

/**
 * \brief Trip & Sync xbar: API to enable pulse stretching of output of Output XBars
 *
 * \param base [in] Output XBar base address
 * \param stretchselect [in] Select stretched / non-stretched output
 *
 */
static inline void
SOC_xbarSelectStretchedPulseOutputXBarOutputSignal(uint32_t base, uint32_t stretchselect)
{
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTSTRETCH, stretchselect & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTSTRETCH_STRETCHSEL_MASK);
}

/**
 * \brief Trip & Sync xbar: API to configure pulse streching length of output of Output XBars
 *
 * \param base [in] Output XBar base address
 * \param lengthselect [in] Configure strech length
 *
 */
static inline void
SOC_xbarSelectStretchedPulseLengthOutputXBarOutputSignal(uint32_t base, uint32_t lengthselect)
{
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLENGTH, lengthselect & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLENGTH_LENGTHSEL_MASK);
}

/**
 * \brief Trip & Sync xbar: API to configure inversion of output signal of Output XBars
 *
 * \param base [in] Output XBar base address
 * \param invertout [in] Mask defining the Output XBar output signal to be inverted
 *
 */
static inline void
SOC_xbarInvertOutputXBarOutputSignal(uint32_t base, uint32_t invertout)
{
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTINVERT, invertout & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTINVERT_OUTINVERT_MASK);
}

/**
 * \brief Trip & Sync xbar: API to select input sources of Output XBar
 *
 * \param base [in] Output XBar base address
 * \param out [in] Instance of Output XBar
 * \param group0_mask [in] Mask to OR inputs from group 0
 * \param group1_mask [in] Mask to OR inputs from group 1
 * \param group2_mask [in] Mask to OR inputs from group 2
 * \param group3_mask [in] Mask to OR inputs from group 3
 * \param group4_mask [in] Mask to OR inputs from group 4
 * \param group5_mask [in] Mask to OR inputs from group 5
 * \param group6_mask [in] Mask to OR inputs from group 6
 * \param group7_mask [in] Mask to OR inputs from group 7
 * \param group8_mask [in] Mask to OR inputs from group 8
 * \param group9_mask [in] Mask to OR inputs from group 9
 * \param group10_mask [in] Left for API Backward Compatibility.
 *
 */
static inline void
SOC_xbarSelectOutputXBarInputSource(uint32_t base, uint8_t out, uint32_t group0_mask, uint32_t group1_mask, uint32_t group2_mask, uint32_t group3_mask, uint32_t group4_mask, uint32_t group5_mask, uint32_t group6_mask, uint32_t group7_mask, uint32_t group8_mask, uint32_t group9_mask, uint32_t group10_mask)
{
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group0_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G1 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group1_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G1_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G2 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group2_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G2_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G3 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group3_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G3_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G4 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group4_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G4_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G5 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group5_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G5_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G6 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group6_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G6_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G7 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group7_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G7_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G8 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group8_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G8_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G9 + out * CSL_CONTROLSS_OUTPUTXBAR_STEP, group9_mask & CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G9_SEL_MASK);
}


/**
 * \brief Trip & Sync xbar: API to select input sources of PWM Syncout XBar
 *
 * \param base [in] PWM Syncout XBar base address
 * \param out [in] Instance of PWM Syncout XBar
 * \param input [in] Mask to OR inputs
 *
 */
static inline void
SOC_xbarSelectPWMSyncOutXBarInput(uint32_t base, uint8_t out, uint32_t input)
{
    //TBD: 32 bit field for selecting 32 inputs
    HW_WR_REG32(base + CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0 + out * CSL_CONTROLSS_PWMSYNCOUTXBAR_STEP, input & CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0_SEL_MASK);
}


/**
 * \brief Trip & Sync xbar: API to select input sources of PWM Syncout XBar
 *
 * \param base [in] PWM Syncout XBar base address
 * \param out [in] Instance of PWM Syncout XBar
 * \param group0_mask [in] Mask to OR inputs from group 0
 * \param group1_mask [in] Mask to OR inputs from group 1
 *
 */
static inline void
SOC_xbarSelectPWMSyncOutXBarInput_ext(uint32_t base, uint8_t out, uint32_t group0_mask, uint32_t group1_mask)
{
    //TBD: 32 bit field for selecting 32 inputs
    HW_WR_REG32(base + CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0 + out * CSL_CONTROLSS_PWMSYNCOUTXBAR_STEP, group0_mask & CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0_SEL_MASK);
    HW_WR_REG32(base + CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G1 + out * CSL_CONTROLSS_PWMSYNCOUTXBAR_STEP, group1_mask & CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G1_SEL_MASK);
}


/**
 * \brief SoC level xbars: API to select input source of EDMA Trigger XBar
 *
 * \param base [in] EDMA Trigger XBar base address
 * \param out [in] Instance of EDMA Trigger XBar
 * \param mux_control [in] Mux control to select input source
 *
 */
static inline void
SOC_xbarSelectEdmaTrigXbarInputSource(uint32_t base, uint8_t out, uint8_t mux_control)
{
    HW_WR_REG32(base + CSL_EDMA_TRIG_XBAR_MUXCNTL(out), (CSL_EDMA_TRIG_XBAR_MUXCNTL_INT_ENABLE_MASK) | (mux_control & CSL_EDMA_TRIG_XBAR_MUXCNTL_ENABLE_MASK));
}

/**
 * \brief SoC level xbars: API to select input source of GPIO Interrupt XBar
 *
 * \param base [in] GPIO Interrupt XBar base address
 * \param out [in] Instance of GPIO Interrupt XBar
 * \param mux_control [in] Mux control to select input source
 *
 */
static inline void
SOC_xbarSelectGpioIntrXbarInputSource(uint32_t base, uint8_t out, uint8_t mux_control)
{
    HW_WR_REG32(base + CSL_GPIO_INTR_XBAR_MUXCNTL(out), (CSL_GPIO_INTR_XBAR_MUXCNTL_INT_ENABLE_MASK) | (mux_control & CSL_GPIO_INTR_XBAR_MUXCNTL_ENABLE_MASK));
}

/**
 * \brief SoC level xbars: API to select input source of ICSSM Interrupt XBar
 *
 * \param base [in] ICSSM Interrupt XBar base address
 * \param out [in] Instance of ICSSM Interrupt XBar
 * \param mux_control [in] Mux control to select input source
 *
 */
static inline void
SOC_xbarSelectIcssmIntrXbarInputSource(uint32_t base, uint8_t out, uint8_t mux_control)
{
    HW_WR_REG32(base + CSL_ICSSM_INTR_XBAR_MUXCNTL(out), (CSL_ICSSM_INTR_XBAR_MUXCNTL_INT_ENABLE_MASK) | (mux_control & CSL_ICSSM_INTR_XBAR_MUXCNTL_ENABLE_MASK));
}

/**
 * \brief SoC level xbars: API to select input source of TimeSync XBar0
 *
 * \param base [in] TimeSync XBar0 base address
 * \param out [in] Instance of TimeSync XBar0
 * \param mux_control [in] Mux control to select input source
 *
 */
static inline void
SOC_xbarSelectTimesyncXbar0InputSource(uint32_t base, uint8_t out, uint8_t mux_control)
{
    HW_WR_REG32(base + CSL_SOC_TIMESYNC_XBAR0_MUXCNTL(out), (CSL_SOC_TIMESYNC_XBAR0_MUXCNTL_INT_ENABLE_MASK) | (mux_control & CSL_SOC_TIMESYNC_XBAR0_MUXCNTL_ENABLE_MASK));
}

/**
 * \brief SoC level xbars: API to select input source of TimeSync XBar1
 *
 * \param base [in] TimeSync XBar1 base address
 * \param out [in] Instance of TimeSync XBar1
 * \param mux_control [in] Mux control to select input source
 *
 */
static inline void
SOC_xbarSelectTimesyncXbar1InputSource(uint32_t base, uint8_t out, uint8_t mux_control)
{
    HW_WR_REG32(base + CSL_SOC_TIMESYNC_XBAR1_MUXCNTL(out), (CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_INT_ENABLE_MASK) | (mux_control & CSL_SOC_TIMESYNC_XBAR1_MUXCNTL_ENABLE_MASK));
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif // SOC_XBAR_AM261X_H_
