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
 * @file   pmic_power_tps65386x_priv.h
 *
 * @brief  PMIC TPS65386x BB PMIC Power Resources Driver API/interface file.
 *
 */

#ifndef PMIC_POWER_TPS65386x_PRIV_H_
#define PMIC_POWER_TPS65386x_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_core_priv.h"
#include "pmic_io_priv.h"
#include "pmic_power_priv.h"
#include "pmic_types.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

#define PMIC_BB_CFG_STATUS_NOT_INCLUDED 	(0x0U)
#define PMIC_BB_CFG_STATUS_INCLUDED 		(0x1U)

#define PMIC_BB_CFG_DRSS_DISABLED 			(0x0U)
#define PMIC_BB_CFG_DRSS_ENABLED 			(0x1U)

#define PMIC_BB_CFG_4V 						(0x0U)
#define PMIC_BB_CFG_SAME_BBCFG 				(0x1U)

#define PMIC_BB_CFG_BB_LVL_CFG_4_3V 		(0x0U)
#define PMIC_BB_CFG_BB_LVL_CFG_5V 			(0x1U)
#define PMIC_BB_CFG_BB_LVL_CFG_6V 			(0x2U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


#endif /* PMIC_POWER_TPS65386x_PRIV_H_ */
