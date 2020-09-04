/*
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>

#define KICK0_UNLOCK_VAL				(0x68EF3490U)
#define KICK1_UNLOCK_VAL				(0xD172BC5AU)

#define VTT_PAD_CFG_VAL                 (0x20007U)

#define GPIO_SET_DATA01                (0x18U)
#define GPIO_DIR01                     (0X10U)

void DDR_socEnableVttRegulator(void)
{
    uint32_t gpio_dirVal;

    /* Unlock PAD config MMR */
    HW_WR_REG32 (CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_LOCK1_KICK0, KICK0_UNLOCK_VAL);
    HW_WR_REG32 (CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_LOCK1_KICK1, KICK1_UNLOCK_VAL);

    /* Change to muxmode7, enable pullup, enable driver */
    HW_WR_REG32 (CSL_PADCFG_CTRL0_CFG0_BASE + CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG12, VTT_PAD_CFG_VAL);

    /* Pull VTT high */
    HW_WR_REG32 (CSL_GPIO0_BASE + GPIO_SET_DATA01, (1U << 12U));
    gpio_dirVal = HW_RD_REG32(CSL_GPIO0_BASE + GPIO_DIR01);
    HW_WR_REG32(CSL_GPIO0_BASE + GPIO_DIR01, gpio_dirVal | (1U << 12U));

    return;
}
