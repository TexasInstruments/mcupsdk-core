/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 *
 */

/**
 *  \defgroup DRV_PINMUX_MODULE APIs for PINMUX
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the pinmux module.
 *
 *  @{
 */

/**
 *  \file pinmux.h
 *
 *  \brief PINMUX Driver API/interface file.
 *
 * \brief  This file contains pad configure register offsets and bit-field
 *         value macros for different configurations,
 *
 *           BIT[21]        TXDISABLE       disable the pin's output driver
 *           BIT[18]        RXACTIVE        enable the pin's input buffer (typically kept enabled)
 *           BIT[17]        PULLTYPESEL     set the iternal resistor pull direction high or low (if enabled)
 *           BIT[16]        PULLUDEN        internal resistor disable (0 = enabled / 1 = disabled)
 *           BIT[3:0]       MUXMODE         select the desired function on the given pin
 */

#ifndef PINMUX_AM273X_H_
#define PINMUX_AM273X_H_

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

/**
 *  \anchor Pinmux_DomainId_t
 *  \name Pinmux Domain ID
 *  @{
 */
/** \brief NOT USED on this SOC */
#define PINMUX_DOMAIN_ID_MAIN           (0U)
/** \brief NOT USED on this SOC */
#define PINMUX_DOMAIN_ID_MCU            (1U)
/** @} */

/** \brief Macro to mark end of pinmux config array */
#define PINMUX_END                      (0xFFFFFFFFU)
/** \brief Pin mode */
#define PIN_MODE(mode)                  (mode)
/** \brief Resistor disable */
#define PIN_PULL_DISABLE                (0x1U << 8U)
/** \brief Pull direction, 0: pull-down, 1: pull-up */
#define PIN_PULL_DIRECTION              (0x1U << 9U)

/**
 *  \anchor Pinmux_Offsets
 *  \name Pad config register offset in control module
 *  @{
 */
#define PIN_PAD_AA          (0x00000000U)
#define PIN_PAD_AB          (0x00000004U)
#define PIN_PAD_AC          (0x00000008U)
#define PIN_PAD_AD          (0x0000000CU)
#define PIN_PAD_AE          (0x00000010U)
#define PIN_PAD_AF          (0x00000014U)
#define PIN_PAD_AG          (0x00000018U)
#define PIN_PAD_AH          (0x0000001CU)
#define PIN_PAD_AI          (0x00000020U)
#define PIN_PAD_AJ          (0x00000024U)
#define PIN_PAD_AK          (0x00000028U)
#define PIN_PAD_AL          (0x0000002CU)
#define PIN_PAD_AM          (0x00000030U)
#define PIN_PAD_AN          (0x00000034U)
#define PIN_PAD_AO          (0x00000038U)
#define PIN_PAD_AP          (0x0000003CU)
#define PIN_PAD_AQ          (0x00000040U)
#define PIN_PAD_AR          (0x00000044U)
#define PIN_PAD_AS          (0x00000048U)
#define PIN_PAD_AT          (0x0000004CU)
#define PIN_PAD_AU          (0x00000050U)
#define PIN_PAD_AV          (0x00000054U)
#define PIN_PAD_AW          (0x00000058U)
#define PIN_PAD_AX          (0x0000005CU)
#define PIN_PAD_AY          (0x00000060U)
#define PIN_PAD_AZ          (0x00000064U)
#define PIN_PAD_BA          (0x00000068U)
#define PIN_PAD_BB          (0x0000006CU)
#define PIN_PAD_BC          (0x00000070U)
#define PIN_PAD_BD          (0x00000074U)
#define PIN_PAD_BE          (0x00000078U)
#define PIN_PAD_BF          (0x0000007CU)
#define PIN_PAD_BG          (0x00000080U)
#define PIN_PAD_BH          (0x00000084U)
#define PIN_PAD_BI          (0x00000088U)
#define PIN_PAD_BJ          (0x0000008CU)
#define PIN_PAD_BK          (0x00000090U)
#define PIN_PAD_BL          (0x00000094U)
#define PIN_PAD_BM          (0x00000098U)
#define PIN_PAD_BN          (0x0000009CU)
#define PIN_PAD_BO          (0x000000A0U)
#define PIN_PAD_BP          (0x000000A4U)
#define PIN_PAD_BQ          (0x000000A8U)
#define PIN_PAD_BR          (0x000000ACU)
#define PIN_PAD_BS          (0x000000B0U)
#define PIN_PAD_BT          (0x000000B4U)
#define PIN_PAD_BU          (0x000000B8U)
#define PIN_PAD_BV          (0x000000BCU)
#define PIN_PAD_BW          (0x000000C0U)
#define PIN_PAD_BX          (0x000000C4U)
#define PIN_PAD_BY          (0x000000C8U)
#define PIN_PAD_BZ          (0x000000CCU)
#define PIN_PAD_CA          (0x000000D0U)
#define PIN_PAD_CB          (0x000000D4U)
#define PIN_PAD_CC          (0x000000D8U)
#define PIN_PAD_CD          (0x000000DCU)
#define PIN_PAD_CE          (0x000000E0U)
#define PIN_PAD_CF          (0x000000E4U)
#define PIN_PAD_CG          (0x000000E8U)
#define PIN_PAD_CH          (0x000000ECU)
#define PIN_PAD_CI          (0x000000F0U)
#define PIN_PAD_CJ          (0x000000F4U)
#define PIN_PAD_CK          (0x000000F8U)
#define PIN_PAD_CL          (0x000000FCU)
#define PIN_PAD_CM          (0x00000100U)
#define PIN_PAD_CN          (0x00000104U)
#define PIN_PAD_CO          (0x00000108U)
#define PIN_PAD_CP          (0x0000010CU)
#define PIN_PAD_CQ          (0x00000110U)
#define PIN_PAD_CR          (0x00000114U)
#define PIN_PAD_CS          (0x00000118U)
#define PIN_PAD_CT          (0x0000011CU)
#define PIN_PAD_CU          (0x00000120U)
#define PIN_PAD_CV          (0x00000124U)
#define PIN_PAD_CW          (0x00000128U)
#define PIN_PAD_CX          (0x0000012CU)
#define PIN_PAD_CY          (0x00000130U)
#define PIN_PAD_CZ          (0x00000134U)
#define PIN_PAD_DA          (0x00000138U)
#define PIN_PAD_DB          (0x0000013CU)
#define PIN_PAD_DC          (0x00000140U)
#define PIN_PAD_DD          (0x00000144U)
#define PIN_PAD_DE          (0x00000148U)
#define PIN_PAD_DF          (0x0000014CU)
#define PIN_PAD_DG          (0x00000150U)
#define PIN_PAD_DH          (0x00000154U)
#define PIN_PAD_DI          (0x00000158U)
#define PIN_PAD_DJ          (0x0000015CU)
#define PIN_PAD_DK          (0x00000160U)
#define PIN_PAD_DL          (0x00000164U)
#define PIN_PAD_DM          (0x00000168U)
#define PIN_PAD_DN          (0x0000016CU)
#define PIN_PAD_DO          (0x00000170U)
#define PIN_PAD_DP          (0x00000174U)
#define PIN_PAD_DQ          (0x00000178U)
#define PIN_PAD_DR          (0x0000017CU)
#define PIN_PAD_DS          (0x00000180U)
#define PIN_PAD_DT          (0x00000184U)
#define PIN_PAD_DU          (0x00000188U)
#define PIN_PAD_DV          (0x0000018CU)
#define PIN_PAD_DW          (0x00000190U)
#define PIN_PAD_DX          (0x00000194U)
#define PIN_PAD_DY          (0x00000198U)
#define PIN_PAD_DZ          (0x0000019CU)
#define PIN_PAD_EA          (0x000001A0U)
#define PIN_PAD_EB          (0x000001A4U)
#define PIN_PAD_EC          (0x000001A8U)
#define PIN_PAD_ED          (0x000001ACU)
#define PIN_PAD_EE          (0x000001B0U)
#define PIN_PAD_EF          (0x000001B4U)
#define PIN_PAD_EG          (0x000001B8U)
#define PIN_PAD_EH          (0x000001BCU)
#define PIN_PAD_EI          (0x000001C0U)
#define PIN_PAD_EJ          (0x000001C4U)
#define PIN_PAD_EK          (0x000001C8U)
#define PIN_PAD_EL          (0x000001CCU)
#define PIN_PAD_EM          (0x000001D0U)
#define PIN_PAD_EN          (0x000001D4U)
#define PIN_PAD_EO          (0x000001D8U)
#define PIN_PAD_EP          (0x000001DCU)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief Structure defining the pin configuration parameters */
typedef struct Pinmux_PerCfg
{
    uint32_t    offset;
    /**< Register offset for configuring the pin.
     *   Refer \ref Pinmux_Offsets.
     *   Set this to #PINMUX_END to demark the end of configuration array */
    uint32_t    settings;
    /**< Value to be configured.
     *   Active mode configurations like mux mode, pull resistor and
     *   buffer mode.
     *
     *   To set a value use like   : "| PIN_PULL_DISABLE"
     *   To reset a value use like : "& (~PIN_PULL_DIRECTION)"
     *
     *   For example,
     *   PIN_MODE(7) | ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION)
     */
} Pinmux_PerCfg_t;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This API configures the pinmux based on the domain
 *
 *  \param  pinmuxCfg   Pointer to list of pinmux configuration array.
 *                      This parameter cannot be NULL and the last entry should
 *                      be initialized with #PINMUX_END so that this function
 *                      knows the end of configuration.
 *  \param  domainId    NOT USED in this SOC
 */
void Pinmux_config(const Pinmux_PerCfg_t *pinmuxCfg, uint32_t domainId);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PINMUX_AM64X_H_ */

/** @} */
