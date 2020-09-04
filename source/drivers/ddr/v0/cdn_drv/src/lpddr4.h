/******************************************************************************
 * Copyright (C) 2012-2021 Cadence Design Systems, Inc.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************************
 * Cadence Core Driver for LPDDR4.
 **********************************************************************/

#ifndef LPDDR4_H
#define LPDDR4_H

#include "../include/16bit/lpddr4_ctl_regs.h"
#include "lpddr4_sanity.h"
#ifdef DDR_16BIT
#include "lpddr4_16bit.h"
#include "lpddr4_16bit_sanity.h"
#else
#include "lpddr4_32bit.h"
#include "lpddr4_32bit_sanity.h"
#endif
#ifdef __cplusplus
extern "C" {
#endif

#define PRODUCT_ID (0x1046U)

#define BIT_MASK    (0x1U)
#define BYTE_MASK   (0xffU)
#define NIBBLE_MASK (0xfU)

#define WORD_SHIFT (32U)
#define WORD_MASK (0xffffffffU)
#define SLICE_WIDTH (0x100)

#define CTL_OFFSET 0
#define PI_OFFSET (((uint32_t)1) <<  11)
#define PHY_OFFSET (((uint32_t)1) << 12)

/* BIT[17] on INT_MASK_1 register. */
#define CTL_INT_MASK_ALL ((uint32_t)LPDDR4_LOR_BITS - WORD_SHIFT)

/* Init Error information bits */
#define PLL_READY (0x3U)
#define IO_CALIB_DONE ((uint32_t)0x1U << 23U)
#define IO_CALIB_FIELD ((uint32_t)NIBBLE_MASK << 28U)
#define IO_CALIB_STATE ((uint32_t)0xBU << 28U)
#define RX_CAL_DONE ((uint32_t)BIT_MASK << 4U)
#define CA_TRAIN_RL (((uint32_t)BIT_MASK << 5U) | ((uint32_t)BIT_MASK << 4U))
#define WR_LVL_STATE (((uint32_t)NIBBLE_MASK) << 13U)
#define GATE_LVL_ERROR_FIELDS (((uint32_t)BIT_MASK << 7U) | ((uint32_t)BIT_MASK << 6U))
#define READ_LVL_ERROR_FIELDS ((((uint32_t)NIBBLE_MASK) << 28U) | (((uint32_t)BYTE_MASK) << 16U))
#define DQ_LVL_STATUS (((uint32_t)BIT_MASK << 26U) | (((uint32_t)BYTE_MASK) << 18U))

/*User defined macros*/
#define CDN_TRUE  1U
#define CDN_FALSE 0U

#ifndef LPDDR4_CUSTOM_TIMEOUT_DELAY
#define LPDDR4_CUSTOM_TIMEOUT_DELAY 100000000U
#endif

#ifndef LPDDR4_CPS_NS_DELAY_TIME
#define LPDDR4_CPS_NS_DELAY_TIME 10000000U
#endif

void LPDDR4_SetSettings(LPDDR4_CtlRegs* ctlRegBase, const bool errorFound);
volatile uint32_t* LPDDR4_AddOffset(volatile uint32_t* addr, uint32_t regOffset);
uint32_t LPDDR4_PollCtlIrq(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt irqBit,  uint32_t delay);
bool LPDDR4_CheckLvlErrors(const LPDDR4_PrivateData* pD, LPDDR4_DebugInfo* debugInfo, bool errFound);
void LPDDR4_SetErrors(LPDDR4_CtlRegs* ctlRegBase,  LPDDR4_DebugInfo* debugInfo, uint8_t* errFoundPtr);

uint32_t LPDDR4_EnablePIInitiator(const LPDDR4_PrivateData* pD);
void LPDDR4_CheckWrLvlError(LPDDR4_CtlRegs* ctlRegBase, LPDDR4_DebugInfo* debugInfo, bool* errFoundPtr);
uint32_t LPDDR4_CheckMmrReadError(const LPDDR4_PrivateData* pD, uint64_t* mmrValue, uint8_t* mrrStatus);
uint32_t LPDDR4_getDSliceMask(uint32_t dSliceNum, uint32_t arrayOffset );
#ifdef __cplusplus
}
#endif

#endif  /* LPDDR4_H */
