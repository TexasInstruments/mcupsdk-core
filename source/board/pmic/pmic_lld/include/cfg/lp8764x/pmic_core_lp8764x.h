/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file pmic_core_lp8764x.h
 *
 *  \brief  The macro definitions for LP8764x HERA PMIC driver specific
 *          PMIC common configuration
 */

#ifndef PMIC_CORE_LP8764X_H_
#define PMIC_CORE_LP8764X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* On J7200 1.0 EVM, PMIC_LP8764X_DEV_REV_ID is 0x03 */
#define PMIC_LP8764X_DEV_REV_ID_PG_1_0          (0x03U)

/* On J7200 2.0 EVM, PMIC_LP8764X_DEV_REV_ID is 0x43U */
#define PMIC_LP8764X_DEV_REV_ID_PG_2_0          (0x43U)

/**
 *  \anchor Pmic_Lp8764xHera_EepromDef_LdCfg
 *  \name   PMIC EEPROM Defaults Load to Conf register Configuration
 *
 *  @{
 */
 /** \brief EEPROM defaults are loaded to Conf registers */
#define PMIC_LP8764X_EEPROM_DEFAULTS_LOAD_TO_CONF_OTHER_REGS            (0U)
/** \brief EEPROM defaults are not loaded to Conf registers */
#define PMIC_LP8764X_EEPROM_DEFAULTS_NOT_LOADED_TO_CONF_OTHER_REGS      (1U)

/**
 *  \anchor Pmic_Lp8764xHera_Skip_EepromDef_LdCfg
 *  \name   PMIC Skip EEPROM Defaults Load to Conf register Configuration
 *
 *  @{
 */
 /** \brief Skip EEPROM defaults are loaded to Conf and Other registers is
   *        disabled */
#define PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED       0U
 /** \brief Skip EEPROM defaults are loaded to Conf and Other registers is
  *         enabled */
#define PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_ENABLED        1U

/**
 *  \anchor Pmic_Lp8764xHera_RefOutPinCtrl_Cfg
 *  \name   PMIC REF OUT Pin Control Configuration
 *
 *  @{
 */
 /** \brief Disable Bandgap voltage to REFOUT pin */
#define PMIC_LP8764X_REF_OUT_PIN_CFG_DISABLE      0U
/** \brief Enable Bandgap voltage to REFOUT pin  */
#define PMIC_LP8764X_REF_OUT_PIN_CFG_ENABLE       1U
/*  @} */

/**
 *  \anchor Pmic_Lp8764xHera_ExtClk_Freq_Sel
 *  \name   PMIC External Clock (SYNCCLKIN) Frequency selection
 *
 *  @{
 */
/** \brief  SYNCCLKIN Frequency as 1.1 MHz */
#define PMIC_LP8764X_SYNCCLKIN_1_1_MHZ      (0U)
/** \brief  SYNCCLKIN Frequency as 2.2 MHz */
#define PMIC_LP8764X_SYNCCLKIN_2_2_MHZ      (1U)
/** \brief  SYNCCLKIN Frequency as 4.4 MHz */
#define PMIC_LP8764X_SYNCCLKIN_4_4_MHZ      (2U)
/** \brief  SYNCCLKIN Frequency as 8.8 MHz */
#define PMIC_LP8764X_SYNCCLKIN_8_8_MHZ      (3U)
/*  @} */


/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_LP8764X_H_ */
