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
            /* ICSSM_IEP pin config */
    /* PR0_IEP0_EDC_SYNC_OUT0 -> PR0_PRU1_GPIO19 (D15) */
    {
        PIN_PR0_PRU1_GPIO19,
        ( PIN_MODE(3) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_IEP0_EDC_SYNC_OUT1 -> SDFM0_CLK1 (A16) */
    {
        PIN_SDFM0_CLK1,
        ( PIN_MODE(3) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* ICSSM pin config */
    /* PR0_MDIO_MDIO -> PR0_MDIO_MDIO (L17) */
    {
        PIN_PR0_MDIO_MDIO,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_MDIO_MDC -> PR0_MDIO_MDC (L18) */
    {
        PIN_PR0_MDIO_MDC,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO0 -> PR0_PRU0_GPIO0 (K17) */
    {
        PIN_PR0_PRU0_GPIO0,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO1 -> PR0_PRU0_GPIO1 (K18) */
    {
        PIN_PR0_PRU0_GPIO1,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO2 -> PR0_PRU0_GPIO2 (J18) */
    {
        PIN_PR0_PRU0_GPIO2,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO3 -> PR0_PRU0_GPIO3 (J17) */
    {
        PIN_PR0_PRU0_GPIO3,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO4 -> PR0_PRU0_GPIO4 (K16) */
    {
        PIN_PR0_PRU0_GPIO4,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO5 -> PR0_PRU0_GPIO5 (G17) */
    {
        PIN_PR0_PRU0_GPIO5,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO6 -> PR0_PRU0_GPIO6 (K15) */
    {
        PIN_PR0_PRU0_GPIO6,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO8 -> PR0_PRU0_GPIO8 (G15) */
    {
        PIN_PR0_PRU0_GPIO8,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO9 -> PR0_PRU0_GPIO9 (F17) */
    {
        PIN_PR0_PRU0_GPIO9,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO10 -> PR0_PRU0_GPIO10 (G18) */
    {
        PIN_PR0_PRU0_GPIO10,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO11 -> PR0_PRU0_GPIO11 (M16) */
    {
        PIN_PR0_PRU0_GPIO11,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO12 -> PR0_PRU0_GPIO12 (M15) */
    {
        PIN_PR0_PRU0_GPIO12,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO13 -> PR0_PRU0_GPIO13 (H17) */
    {
        PIN_PR0_PRU0_GPIO13,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO14 -> PR0_PRU0_GPIO14 (H16) */
    {
        PIN_PR0_PRU0_GPIO14,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO15 -> PR0_PRU0_GPIO15 (L16) */
    {
        PIN_PR0_PRU0_GPIO15,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU0_GPIO16 -> PR0_PRU0_GPIO16 (H18) */
    {
        PIN_PR0_PRU0_GPIO16,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO0 -> PR0_PRU1_GPIO0 (F18) */
    {
        PIN_PR0_PRU1_GPIO0,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO1 -> PR0_PRU1_GPIO1 (G16) */
    {
        PIN_PR0_PRU1_GPIO1,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO2 -> PR0_PRU1_GPIO2 (E17) */
    {
        PIN_PR0_PRU1_GPIO2,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO3 -> PR0_PRU1_GPIO3 (E18) */
    {
        PIN_PR0_PRU1_GPIO3,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO4 -> PR0_PRU1_GPIO4 (F16) */
    {
        PIN_PR0_PRU1_GPIO4,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO5 -> PR0_PRU1_GPIO5 (F15) */
    {
        PIN_PR0_PRU1_GPIO5,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO6 -> PR0_PRU1_GPIO6 (E16) */
    {
        PIN_PR0_PRU1_GPIO6,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO8 -> PR0_PRU1_GPIO8 (D18) */
    {
        PIN_PR0_PRU1_GPIO8,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO9 -> PR0_PRU1_GPIO9 (C18) */
    {
        PIN_PR0_PRU1_GPIO9,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO11 -> PR0_PRU1_GPIO11 (B18) */
    {
        PIN_PR0_PRU1_GPIO11,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO12 -> PR0_PRU1_GPIO12 (B17) */
    {
        PIN_PR0_PRU1_GPIO12,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO13 -> PR0_PRU1_GPIO13 (D16) */
    {
        PIN_PR0_PRU1_GPIO13,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO14 -> PR0_PRU1_GPIO14 (C17) */
    {
        PIN_PR0_PRU1_GPIO14,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO15 -> PR0_PRU1_GPIO15 (A17) */
    {
        PIN_PR0_PRU1_GPIO15,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* PR0_PRU1_GPIO16 -> PR0_PRU1_GPIO16 (C16) */
    {
        PIN_PR0_PRU1_GPIO16,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

    {PINMUX_END, PINMUX_END}
};
