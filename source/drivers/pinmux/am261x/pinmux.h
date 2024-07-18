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
 *           BIT[20]        INVERT          Invert input (0 = Non Inverted / 1 =  Inverted)
 *           BIT[19:18]     QUAL SEL        Select the input qualifier type
 *                                          (00 : SYNC, 01 : 3 SAMPLE, 10 : 6 SAMPLE, 11 : ASYNC)
 *           BIT[17:16]     GPIO SEL        Select the CPU ownership for GPIO pin
 *                                          (00 : GPIO0, 01 : GPIO1, 10 : GPIO2, 11 : GPIO3)
 *           BIT[10]        SLEWRATE        Set slew rate (0 = High / 1 =  Low)
 *           BIT[9]         PULLTYPESEL     Set the iternal resistor pull direction high or low (if enabled)
 *           BIT[8]         PULLUDEN        Internal resistor disable (0 = enabled / 1 = disabled)
 *           BIT[7:4]       OVERRIDE        Override the default input and output driver from IP
 *           BIT[3:0]       MUXMODE         select the desired function on the given pin
 */

#ifndef PINMUX_AM261X_H_
#define PINMUX_AM261X_H_

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
#define PINMUX_DOMAIN_ID_MAIN          (0U)
/** @} */

/** \brief Macro to mark end of pinmux config array */
#define PINMUX_END                      (-1)
/** \brief Pin mode - it is at 0th bit. No shift requried */
#define PIN_MODE(mode)                  ((uint32_t) mode)

/** \brief Override IP default to enable input */
#define PIN_FORCE_INPUT_ENABLE          (((uint32_t) 0x1U) << 4U)
/** \brief Override IP default to disable input */
#define PIN_FORCE_INPUT_DISABLE         (((uint32_t) 0x3U) << 4U)
/** \brief Override IP default to enable output */
#define PIN_FORCE_OUTPUT_ENABLE         (((uint32_t) 0x1U) << 6U)
/** \brief Override IP default to disable output */
#define PIN_FORCE_OUTPUT_DISABLE        (((uint32_t) 0x3U) << 6U)

/** \brief Resistor enable */
#define PIN_PULL_DISABLE                (((uint32_t) 0x1U) << 8U)
/** \brief Pull Up */
#define PIN_PULL_UP                     (((uint32_t) 0x1U) << 9U)
/** \brief Pull Down */
#define PIN_PULL_DOWN                   (((uint32_t) 0x0U) << 9U)

/** \brief Slew Rate High */
#define PIN_SLEW_RATE_HIGH              (((uint32_t) 0x0U) << 10U)
/** \brief Slew Rate Low */
#define PIN_SLEW_RATE_LOW               (((uint32_t) 0x1U) << 10U)

/** \brief GPIO Pin CPU ownership - R5SS0_0 */
#define PIN_GPIO_R5SS0_0                (((uint32_t) 0x0U) << 16U)
/** \brief GPIO Pin CPU ownership - R5SS0_1 */
#define PIN_GPIO_R5SS0_1                (((uint32_t) 0x1U) << 16U)
/** \brief GPIO Pin CPU ownership - R5SS1_0 */
#define PIN_GPIO_R5SS1_0                (((uint32_t) 0x2U) << 16U)
/** \brief GPIO Pin CPU ownership - R5SS1_1 */
#define PIN_GPIO_R5SS1_1                (((uint32_t) 0x3U) << 16U)

/** \brief Pin Qualifier - SYNC */
#define PIN_QUAL_SYNC                   (((uint32_t) 0x0U) << 18U)
/** \brief Pin Qualifier - 3 SAMPLE */
#define PIN_QUAL_3SAMPLE                (((uint32_t) 0x1U) << 18U)
/** \brief Pin Qualifier - 6 SAMPLE */
#define PIN_QUAL_6SAMPLE                (((uint32_t) 0x2U) << 18U)
/** \brief Pin Qualifier - ASYNC */
#define PIN_QUAL_ASYNC                  (((uint32_t) 0x3U) << 18U)

/** \brief Pin Invert */
#define PIN_INVERT                      (((uint32_t) 0x1U) << 20U)
/** \brief Pin Non Invert */
#define PIN_NON_INVERT                  (((uint32_t) 0x0U) << 20U)


/**
 *  \anchor Pinmux_Offsets
 *  \name Pad config register offset in control module
 *  @{
 */

#define PIN_GPIO0                                               (0x00000000U)
#define PIN_GPIO1                                               (0x00000004U)
#define PIN_GPIO2                                               (0x00000008U)
#define PIN_GPIO3                                               (0x0000000CU)
#define PIN_GPIO4                                               (0x00000010U)
#define PIN_GPIO5                                               (0x00000014U)
#define PIN_GPIO6                                               (0x00000018U)
#define PIN_GPIO7                                               (0x0000001CU)
#define PIN_GPIO8                                               (0x00000020U)
#define PIN_GPIO9                                               (0x00000024U)
#define PIN_GPIO10                                              (0x00000028U)
#define PIN_GPIO11                                              (0x0000002CU)
#define PIN_GPIO12                                              (0x00000030U)
#define PIN_GPIO13                                              (0x00000034U)
#define PIN_GPIO14                                              (0x00000038U)
#define PIN_GPIO15                                              (0x0000003CU)
#define PIN_GPIO16                                              (0x00000040U)
#define PIN_GPIO17                                              (0x00000044U)
#define PIN_GPIO18                                              (0x00000048U)
#define PIN_GPIO19                                              (0x0000004CU)
#define PIN_GPIO20                                              (0x00000050U)
#define PIN_GPIO21                                              (0x00000054U)
#define PIN_GPIO22                                              (0x00000058U)
#define PIN_GPIO23                                              (0x0000005CU)
#define PIN_GPIO24                                              (0x00000060U)
#define PIN_GPIO25                                              (0x00000064U)
#define PIN_GPIO26                                              (0x00000068U)
#define PIN_GPIO27                                              (0x0000006CU)
#define PIN_GPIO28                                              (0x00000070U)
#define PIN_GPIO29                                              (0x00000074U)
#define PIN_GPIO30                                              (0x00000078U)
#define PIN_GPIO31                                              (0x0000007CU)
#define PIN_GPIO32                                              (0x00000080U)
#define PIN_GPIO33                                              (0x00000084U)
#define PIN_GPIO34                                              (0x00000088U)
#define PIN_GPIO35                                              (0x0000008CU)
#define PIN_GPIO36                                              (0x00000090U)
#define PIN_GPIO37                                              (0x00000094U)
#define PIN_GPIO38                                              (0x00000098U)
#define PIN_GPIO39                                              (0x0000009CU)
#define PIN_GPIO40                                              (0x000000A0U)
#define PIN_GPIO41                                              (0x000000A4U)
#define PIN_GPIO42                                              (0x000000A8U)
#define PIN_GPIO43                                              (0x000000ACU)
#define PIN_GPIO44                                              (0x000000B0U)
#define PIN_GPIO45                                              (0x000000B4U)
#define PIN_GPIO46                                              (0x000000B8U)
#define PIN_GPIO47                                              (0x000000BCU)
#define PIN_GPIO48                                              (0x000000C0U)
#define PIN_GPIO49                                              (0x000000C4U)
#define PIN_GPIO50                                              (0x000000C8U)
#define PIN_GPIO51                                              (0x000000CCU)
#define PIN_GPIO52                                              (0x000000D0U)
#define PIN_GPIO53                                              (0x000000D4U)
#define PIN_GPIO54                                              (0x000000D8U)
#define PIN_GPIO55                                              (0x000000DCU)
#define PIN_GPIO56                                              (0x000000E0U)
#define PIN_GPIO57                                              (0x000000E4U)
#define PIN_GPIO58                                              (0x000000E8U)
#define PIN_GPIO59                                              (0x000000ECU)
#define PIN_GPIO60                                              (0x000000F0U)
#define PIN_GPIO61                                              (0x000000F4U)
#define PIN_GPIO62                                              (0x000000F8U)
#define PIN_GPIO63                                              (0x000000FCU)
#define PIN_GPIO64                                              (0x00000100U)
#define PIN_GPIO65                                              (0x00000104U)
#define PIN_GPIO66                                              (0x00000108U)
#define PIN_GPIO67                                              (0x0000010CU)
#define PIN_GPIO68                                              (0x00000110U)
#define PIN_GPIO69                                              (0x00000114U)
#define PIN_GPIO70                                              (0x00000118U)
#define PIN_GPIO71                                              (0x0000011CU)
#define PIN_GPIO72                                              (0x00000120U)
#define PIN_GPIO73                                              (0x00000124U)
#define PIN_GPIO74                                              (0x00000128U)
#define PIN_GPIO75                                              (0x0000012CU)
#define PIN_GPIO76                                              (0x00000130U)
#define PIN_GPIO77                                              (0x00000134U)
#define PIN_GPIO78                                              (0x00000138U)
#define PIN_GPIO79                                              (0x0000013CU)
#define PIN_GPIO80                                              (0x00000140U)
#define PIN_GPIO81                                              (0x00000144U)
#define PIN_GPIO82                                              (0x00000148U)
#define PIN_GPIO83                                              (0x0000014CU)
#define PIN_GPIO84                                              (0x00000150U)
#define PIN_GPIO85                                              (0x00000154U)
#define PIN_GPIO86                                              (0x00000158U)
#define PIN_GPIO87                                              (0x0000015CU)
#define PIN_GPIO88                                              (0x00000160U)
#define PIN_GPIO89                                              (0x00000164U)
#define PIN_GPIO90                                              (0x00000168U)
#define PIN_GPIO91                                              (0x0000016CU)
#define PIN_GPIO92                                              (0x00000170U)
#define PIN_GPIO93                                              (0x00000174U)
#define PIN_GPIO94                                              (0x00000178U)
#define PIN_GPIO95                                              (0x0000017CU)
#define PIN_GPIO96                                              (0x00000180U)
#define PIN_GPIO97                                              (0x00000184U)
#define PIN_GPIO98                                              (0x00000188U)
#define PIN_GPIO99                                              (0x0000018CU)
#define PIN_GPIO100                                             (0x00000190U)
#define PIN_GPIO101                                             (0x00000194U)
#define PIN_GPIO102                                             (0x00000198U)
#define PIN_GPIO103                                             (0x0000019CU)
#define PIN_GPIO104                                             (0x000001A0U)
#define PIN_GPIO105                                             (0x000001A4U)
#define PIN_GPIO106                                             (0x000001A8U)
#define PIN_GPIO107                                             (0x000001ACU)
#define PIN_GPIO108                                             (0x000001B0U)
#define PIN_GPIO109                                             (0x000001B4U)
#define PIN_GPIO110                                             (0x000001B8U)
#define PIN_GPIO111                                             (0x000001BCU)
#define PIN_GPIO112                                             (0x000001C0U)
#define PIN_GPIO113                                             (0x000001C4U)
#define PIN_GPIO114                                             (0x000001C8U)
#define PIN_GPIO115                                             (0x000001CCU)
#define PIN_GPIO116                                             (0x000001D0U)
#define PIN_GPIO117                                             (0x000001D4U)
#define PIN_GPIO118                                             (0x000001D8U)
#define PIN_GPIO119                                             (0x000001DCU)
#define PIN_GPIO120                                             (0x000001E0U)
#define PIN_GPIO121                                             (0x000001E4U)
#define PIN_GPIO122                                             (0x000001E8U)
#define PIN_GPIO123                                             (0x000001ECU)
#define PIN_GPIO124                                             (0x000001F0U)
#define PIN_GPIO125                                             (0x000001F4U)
#define PIN_GPIO126                                             (0x000001F8U)
#define PIN_GPIO127                                             (0x000001FCU)
#define PIN_GPIO128                                             (0x00000200U)
#define PIN_GPIO129                                             (0x00000204U)
#define PIN_GPIO130                                             (0x00000208U)
#define PIN_GPIO131                                             (0x0000020CU)
#define PIN_GPIO132                                             (0x00000210U)
#define PIN_GPIO133                                             (0x00000214U)
#define PIN_GPIO134                                             (0x00000218U)
#define PIN_GPIO135                                             (0x0000021CU)
#define PIN_GPIO136                                             (0x00000220U)
#define PIN_GPIO137                                             (0x00000224U)
#define PIN_GPIO138                                             (0x00000228U)
#define PIN_GPIO139                                             (0x0000022CU)
#define PIN_GPIO140                                             (0x00000230U)
#define PIN_WARMRSTn                                            (0x00000234U)
#define PIN_SAFETY_ERRORn                                       (0x00000238U)
#define PIN_TDI                                                 (0x0000023CU)
#define PIN_TDO                                                 (0x00000240U)
#define PIN_TMS                                                 (0x00000244U)
#define PIN_TCK                                                 (0x00000248U)
#define PIN_OSPI0_CLKLB                                         (0x0000024CU)
#define PIN_OSPI1_CLKLB                                         (0x00000250U)
#define PIN_PORz                                                (0x00000254U)
#define PIN_XTAL_XI                                             (0x00000258U)
#define PIN_XTAL_XO                                             (0x0000025CU)
#define PIN_ADC0_AIN0                                           (0x00000260U)
#define PIN_ADC0_AIN1                                           (0x00000264U)
#define PIN_ADC0_AIN2                                           (0x00000268U)
#define PIN_ADC0_AIN3                                           (0x0000026CU)
#define PIN_ADC0_AIN4                                           (0x00000270U)
#define PIN_ADC0_AIN5                                           (0x00000274U)
#define PIN_ADC0_AIN6                                           (0x00000278U)
#define PIN_ADC1_AIN0                                           (0x0000027CU)
#define PIN_ADC1_AIN1                                           (0x00000280U)
#define PIN_ADC1_AIN2                                           (0x00000284U)
#define PIN_ADC1_AIN3                                           (0x00000288U)
#define PIN_ADC1_AIN4                                           (0x0000028CU)
#define PIN_ADC1_AIN5                                           (0x00000290U)
#define PIN_ADC1_AIN6                                           (0x00000294U)
#define PIN_ADC2_AIN0                                           (0x00000298U)
#define PIN_ADC2_AIN1                                           (0x0000029CU)
#define PIN_ADC2_AIN2                                           (0x000002A0U)
#define PIN_ADC2_AIN3                                           (0x000002A4U)
#define PIN_ADC2_AIN4                                           (0x000002A8U)
#define PIN_ADC2_AIN5                                           (0x000002ACU)
#define PIN_ADC2_AIN6                                           (0x000002B0U)
#define PIN_ADC3_AIN0                                           (0x000002B4U)
#define PIN_ADC3_AIN1                                           (0x000002B8U)
#define PIN_ADC3_AIN2                                           (0x000002BCU)
#define PIN_ADC3_AIN3                                           (0x000002C0U)
#define PIN_ADC3_AIN4                                           (0x000002C4U)
#define PIN_ADC3_AIN5                                           (0x000002C8U)
#define PIN_ADC4_AIN0                                           (0x000002CCU)
#define PIN_ADC4_AIN1                                           (0x000002D0U)
#define PIN_ADC4_AIN2                                           (0x000002D4U)
#define PIN_ADC4_AIN3                                           (0x000002D8U)
#define PIN_ADC4_AIN4                                           (0x000002DCU)
#define PIN_ADC4_AIN5                                           (0x000002E0U)
#define PIN_ADC_VREFHISRC0                                      (0x000002E4U)
#define PIN_ADC_VREFLOSRC0                                      (0x000002E8U)
#define PIN_ADC_VREFHI0                                         (0x000002ECU)
#define PIN_ADC_VREFLO0                                         (0x000002F0U)
#define PIN_ADC_VREFHI1                                         (0x000002F4U)
#define PIN_ADC_VREFLO1                                         (0x000002F8U)
#define PIN_ADC_VREFHI2                                         (0x000002FCU)
#define PIN_ADC_VREFLO2                                         (0x00000300U)
#define PIN_ADC_VREFHISRC1                                      (0x00000304U)
#define PIN_ADC_VREFLOSRC1                                      (0x00000308U)
#define PIN_ADC_VREFHI3                                         (0x0000030CU)
#define PIN_ADC_VREFLO3                                         (0x00000310U)
#define PIN_ADC_VREFHI4                                         (0x00000314U)
#define PIN_ADC_VREFLO4                                         (0x00000318U)
#define PIN_ADC_CAL0                                            (0x0000031CU)
#define PIN_ADC_CAL1                                            (0x00000320U)
#define PIN_DAC_VREF0                                           (0x00000324U)
#define PIN_DAC_VREF1                                           (0x00000328U)
#define PIN_DAC_OUT                                             (0x0000032CU)
#define PIN_TEMPCAL                                             (0x00000330U)
#define PIN_VSYS_MON                                            (0x00000334U)
#define PIN_ATESTV0                                             (0x00000338U)
#define PIN_ATESTV1                                             (0x0000033CU)
#define PIN_PMUBYPASS                                           (0x00000340U)
#define PIN_USB0_CE                                             (0x00000344U)
#define PIN_SDADC0_DP                                           (0x00000348U)
#define PIN_SDADC0_DM                                           (0x0000034CU)
#define PIN_ADC_CAL2                                            (0x00000350U)
#define PIN_ADC_R0_AIN0                                         (0x00000354U)
#define PIN_ADC_R0_AIN1                                         (0x00000358U)
#define PIN_ADC_R0_AIN2                                         (0x0000035CU)
#define PIN_ADC_R0_AIN3                                         (0x00000360U)
#define PIN_ADC_VREFLO5                                         (0x00000364U)
#define PIN_ADC_VREFHI5                                         (0x00000368U)
#define PIN_ADC_VREFLO6                                         (0x0000036CU)
#define PIN_ADC_VREFHI6                                         (0x00000370U)
#define PIN_ADC_R1_AIN0                                         (0x00000374U)
#define PIN_ADC_R1_AIN1                                         (0x00000378U)
#define PIN_ADC_R1_AIN2                                         (0x0000037CU)
#define PIN_ADC_R1_AIN3                                         (0x00000380U)
#define PIN_ADC_CAL3                                            (0x00000384U)

/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief Structure defining the pin configuration parameters */
typedef struct Pinmux_PerCfg
{
    int16_t     offset;
    /**< Register offset for configuring the pin.
     *   Refer \ref Pinmux_Offsets.
     *   Set this to #PINMUX_END to demark the end of configuration array */
    uint32_t    settings;
    /**< Value to be configured.
     *   Active mode configurations like mux mode, pull resistor and Slew rate.
     *
     *   To set a value use like   : "| PIN_PULL_DISABLE"
     *   To reset a value use like : "& (~PIN_PULL_DISABLE)"
     *
     *   For example,
     *   PIN_MODE(7) | ((PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW))
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
 *  \param  domainId    Domain ID to set pinmux configuration.
 *                      Refer \ref Pinmux_DomainId_t
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