/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 */

#include <drivers/pinmux.h>

Pinmux_PerCfg_t gPruicssPinMuxCfg[] = {
            /* PRU-ICSS0-IEP pin config */
    /* PR0_IEP0_EDC_SYNC_OUT0 -> GPIO119 (C18) */
    {
        PIN_GPIO119,
        ( PIN_MODE(3) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0-IEP pin config */
    /* PR0_IEP0_EDC_SYNC_OUT1 -> GPIO124 (A19) */
    {
        PIN_GPIO124,
        ( PIN_MODE(3) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0-IEP pin config */
    /* PR0_IEP0_EDIO_DATA_IN_OUT30 -> GPIO105 (E19) */
    {
        PIN_GPIO105,
        ( PIN_MODE(3) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0-IEP pin config */
    /* PR0_IEP0_EDIO_DATA_IN_OUT31 -> GPIO104 (E20) */
    {
        PIN_GPIO104,
        ( PIN_MODE(3) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0-MDIO pin config */
    /* PR0_MDIO0_MDIO -> GPIO85 (R19) */
    {
        PIN_GPIO85,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0-MDIO pin config */
    /* PR0_MDIO0_MDC -> GPIO86 (R20) */
    {
        PIN_GPIO86,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO8 -> GPIO90 (J18) */
    {
        PIN_GPIO90,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO5 -> GPIO87 (K19) */
    {
        PIN_GPIO87,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO6 -> GPIO91 (N20) */
    {
        PIN_GPIO91,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO4 -> GPIO92 (L20) */
    {
        PIN_GPIO92,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO0 -> GPIO93 (N17) */
    {
        PIN_GPIO93,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO1 -> GPIO94 (N19) */
    {
        PIN_GPIO94,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO2 -> GPIO95 (M18) */
    {
        PIN_GPIO95,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO3 -> GPIO96 (M20) */
    {
        PIN_GPIO96,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO16 -> GPIO97 (M19) */
    {
        PIN_GPIO97,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO15 -> GPIO98 (P18) */
    {
        PIN_GPIO98,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO11 -> GPIO99 (P20) */
    {
        PIN_GPIO99,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO12 -> GPIO100 (P19) */
    {
        PIN_GPIO100,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO13 -> GPIO101 (K20) */
    {
        PIN_GPIO101,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU0_GPIO14 -> GPIO102 (L19) */
    {
        PIN_GPIO102,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO8 -> GPIO106 (G18) */
    {
        PIN_GPIO106,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO5 -> GPIO103 (F20) */
    {
        PIN_GPIO103,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO6 -> GPIO107 (F19) */
    {
        PIN_GPIO107,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO4 -> GPIO108 (G19) */
    {
        PIN_GPIO108,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO0 -> GPIO109 (H20) */
    {
        PIN_GPIO109,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO1 -> GPIO110 (H19) */
    {
        PIN_GPIO110,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO2 -> GPIO111 (H17) */
    {
        PIN_GPIO111,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO3 -> GPIO112 (G20) */
    {
        PIN_GPIO112,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO16 -> GPIO113 (B20) */
    {
        PIN_GPIO113,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO15 -> GPIO114 (E18) */
    {
        PIN_GPIO114,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO11 -> GPIO115 (F17) */
    {
        PIN_GPIO115,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO12 -> GPIO116 (D20) */
    {
        PIN_GPIO116,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO13 -> GPIO117 (C20) */
    {
        PIN_GPIO117,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PRU-ICSS0 pin config */
    /* PR0_PRU1_GPIO14 -> GPIO118 (D19) */
    {
        PIN_GPIO118,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

    {PINMUX_END, PINMUX_END}
};