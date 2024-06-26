/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/

/**
 *  \defgroup DRV_DDR_MODULE APIs for DDR
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the DDR module.
 *  See \ref DRIVERS_DDR_PAGE for more details.
 *
 *  @{
 */

/** \file v1/ddr.h
 *
 *   \brief This file contains DDR APIs.
 */

#ifndef BOARD_DDR_H_
#define BOARD_DDR_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* SDRAM Timing Register 17 */
#define CSL_EMIF_CTLCFG_DRAMTMG17 (0x00000144U)
#define BOARD_DDR_DELAY (1000)
#define BOARD_DDR_SSCFG_V2H_CTL_VAL    (0x000073FFU)

#define DDR3       (0)
#define DDR4       (1U)
#define LPDDR4     (2U)
#define NODDR      (3U)

#define HW_WR_DDRPHY_REG32(off,val)                       \
	do {                                              \
	HW_WR_REG32(CSL_DDRSS0_PHY_CFG_BASE + off,val);   \
	DDR_boardDelay();                                  \
	}                                                 \
	while (0)

#define HW_RD_DDRPHY_REG32(off)                                    \
	({                                                         \
	uint32_t val = HW_RD_REG32(CSL_DDRSS0_PHY_CFG_BASE + off);   \
	DDR_boardDelay();                                           \
	val;                                                       \
	})                                                         \

/* ========================================================================== */
/*                 Function Declarations                                      */
/* ========================================================================== */

/**
 * \brief DDR4 Initialization function
 *
 * 		   Initializes the DDR timing parameters. Sets the DDR timing parameters
 * 		   based in the DDR PLL controller configuration.
 * 		   Any changes to DDR PLL requires change to DDR timing.
 *
 * \param   prms     [in] Pointer to DDR configuration.
 *
 * \return  status 	SystemP_SUCCESS in case of success or appropriate error code
 *
 */

int32_t DDR_init();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BOARD_DDR_H_ */
