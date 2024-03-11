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
 * \file   pmic_rtc_priv.h
 *
 * \brief  The macro definitions, structures and function prototypes for
 *         configuring PMIC RTC
 */

#ifndef PMIC_RTC_PRIV_H_
#define PMIC_RTC_PRIV_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <pmic_rtc_tps6594x_priv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */
/*!
 * \brief   RTC max value for Minutes and Seconds
 */
#define PMIC_RTC_MINUTE_SEC_MAX                        (59U)

/*!
 * \brief   RTC 12 Hour Time values limit
 */
#define PMIC_RTC_12HFMT_HR_MIN                         (1U)
#define PMIC_RTC_12HFMT_HR_MAX                         (12U)

/*!
 * \brief   RTC 24 Hour Time values limit
 */
#define PMIC_RTC_24HFMT_HR_MAX                         (23U)

/*!
 * \brief  RTC month min values
 */
#define PMIC_RTC_DAY_MIN                               (1U)

/*!
 * \brief   RTC years values limit
 */
#define PMIC_RTC_YEAR_MIN                              (2000U)
#define PMIC_RTC_YEAR_MAX                              (2099U)

/*!
 * \brief  RTC month max value for
 *         February month in a Non-leap year.
 */
#define PMIC_RTC_NLPY_FEB_MNTH_DAY_MAX                 (28U)

/*!
 * \brief  RTC month max value for
 *         February month in a leap year.
 */
#define PMIC_RTC_LPY_FEB_MNTH_DAY_MAX                  (29U)

/*!
 * \brief  RTC month max value for
 *         general months.
 */
#define PMIC_RTC_MNTH_DAY_MAX_30                       (30U)
#define PMIC_RTC_MNTH_DAY_MAX_31                       (31U)

/*!
 * \brief   Used to Extract the First and Second
 *          digits of RTC Timer/Alarm decimal Values
 *
 */
#define PMIC_RTC_CONVERT_4BIT_MSB_TO_DEC               (10U)

/*!
 * \brief   Used to Extract last two decimal digits of given Year.
 */
#define PMIC_RTC_EXTRACT_YEAR_DECIMAL_0_99             (100U)

/*!
 * \brief   RTC Auto Compensation ON Value
 */
#define PMIC_RTC_AUTO_COMP_ON                          (0x1U)

/**
 *  \brief PMIC RTC Operations for RTC/ALARM
 */
#define PMIC_RTC_OPS_FOR_RTC                            ((bool)false)
#define PMIC_RTC_OPS_FOR_ALARM                          ((bool)true)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_RTC_PRIV_H_ */
