/*
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 */

/**
 *  \file    sdl_soc_pok.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of POK.
 *            This also contains some related macros.
 */

#ifndef SDL_SOC_POK_H_
#define SDL_SOC_POK_H_

#include <sdl/esm/v0/v0_0/sdl_ip_esm.h>
#include <sdl/dpl/sdl_dpl.h>


#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include <sdl/include/am64x_am243x/sdlr_mcu_ctrl_mmr.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * @ingroup  SDL_MODULE
 * @defgroup SDL_POK_API Power OK(POK)
 *   Provides the APIs for POK.
 *
 */

/**
@defgroup SDL_POK_DATASTRUCT  POK Data Structures
@ingroup SDL_POK_API
*/
/**
@defgroup SDL_POK_FUNCTION  POK Functions
@ingroup SDL_POK_API
*/

/**************************************************************************
* Register Macros
**************************************************************************/

typedef int8_t  SDL_POK_Inst ;

/** Invalid POK/POR ID */
#define SDL_FIRST_POK_ID                                   (0)
/* PMIC POK ID */
#define SDL_POK_VDDA_PMIC_IN_ID                       (SDL_FIRST_POK_ID)
/* VDDS DDRIO Under Voltage/Over Voltage POK ID */
#define SDL_POK_VDDS_DDRIO_ID                               (1)
/* VDDR CORE Under Voltage/Over Voltage POK ID */
#define SDL_POK_VDDR_CORE_ID                                (2)
/* VDDSHV MCU 3P3 Under Voltage/Over Voltage POK ID */
#define SDL_POK_VDDSHV_MCU_3P3_ID                           (3)
/* VDDSHV MCU 1P8 Under Voltage/Over Voltage POK ID */
#define SDL_POK_VDDSHV_MCU_1P8_ID                           (4)
 /* VMON CAP MCU Under Voltage/Over Voltage POK ID */
#define SDL_POK_VMON_CAP_MCU_GENERAL_ID                     (5)
/* VDDSHV MAIN 1P8 Under Voltage/Over Voltage POK ID */
#define SDL_POK_VDDSHV_MAIN_1P8_ID                          (6)
/* VDDSHV MAIN 3P3 Under Voltage/Over Voltage POK ID */
#define SDL_POK_VDDSHV_MAIN_3P3_ID                          (7)
/* CORE Over Voltage POK ID */
#define SDL_POK_VDD_MCU_OV_ID                               (8)
/* VDDA MCU  Under Voltage POK ID */
#define SDL_POR_VDDA_MCU_UV_ID                              (9)
/* VDD Under Voltage POK ID */
#define SDL_POR_VDD_MCU_UV_ID                             (10)
/* VDDA MCU Over Voltage POK ID */
#define SDL_POR_VDDA_MCU_OV_ID                             (11)

/* LAST POK ID */
#define SDL_LAST_POK_ID                               (SDL_POR_VDDA_MCU_OV_ID)


typedef int8_t  SDL_PRG_Inst;

/** First PRG ID */
#define SDL_POK_PRG_FIRST_ID                          (0)
/** PRG MAIN */
#define SDL_POK_PRG_PP_0_ID                           (1)
/** PRG MCU */
#define SDL_POK_PRG_PP_1_ID                           (2)

/** Last POK ID */
#define SDL_POK_PRG_LAST_ID                           (SDL_POK_PRG_PP_1_ID)






#define SDL_POK_MMR_BASE                (SDL_MCU_CTRL_MMR0_CFG0_BASE)

#if defined (SOC_AM64X)
#if defined (M4F_CORE)
#define MCU_ESM_INSTANCE                    (SDL_MCU_ESM0_CFG_BASE)
#define MCU_ESM_INTID                       (SDLR_MCU_ESM0_ESM_LVL_EVENT_ESM0_ESM_INT_HI_LVL_0)
#endif

#if defined (R5F_CORE)
#define MCU_ESM_INSTANCE                    (SDL_ESM0_CFG_BASE)
#define MCU_ESM_INTID                       (SDLR_ESM0_ESM_LVL_EVENT_MCU_ESM0_ESM_INT_HI_LVL_0)
#endif
#endif


#define MCU_ESM_ERR_SIG_VDDA_PMIC_IN_UV     (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU0_POK_PGOOD_UV_OUT_N_TO_ESM_3)
#define MCU_ESM_ERR_SIG_VDD_MCU_UV         (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU0_POK_PGOOD_UV_OUT_N_TO_ESM_0)
#define MCU_ESM_ERR_SIG_VDD_MCU_OV         (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU0_POK_PGOOD_UV_OUT_N_TO_ESM_4)
#define MCU_ESM_ERR_SIG_VDDS_DDRIO_UV        (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_UV_OUT_N_TO_ESM_6)
#define MCU_ESM_ERR_SIG_VDDS_DDRIO_OV        (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_OV_OUT_N_TO_ESM_6)
#define MCU_ESM_ERR_SIG_VDDR_CORE_UV        (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_UV_OUT_N_TO_ESM_5)
#define MCU_ESM_ERR_SIG_VDDR_CORE_OV        (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_OV_OUT_N_TO_ESM_5)
#define MCU_ESM_ERR_SIG_VDDSHV_MCU_3P3_UV   (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_UV_OUT_N_TO_ESM_4) //- 75
#define MCU_ESM_ERR_SIG_VDDSHV_MCU_3P3_OV   (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_OV_OUT_N_TO_ESM_4)// 68
#define MCU_ESM_ERR_SIG_VDDSHV_MCU_1P8_UV   (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_UV_OUT_N_TO_ESM_3 ) //74
#define MCU_ESM_ERR_SIG_VDDSHV_MCU_1P8_OV       (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_OV_OUT_N_TO_ESM_3)//67
#define MCU_ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_UV    (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_UV_OUT_N_TO_ESM_2  )
#define MCU_ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_OV    (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_OV_OUT_N_TO_ESM_2)
#define MCU_ESM_ERR_SIG_VDDSHV_MAIN_1P8_UV    (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_UV_OUT_N_TO_ESM_1)
#define MCU_ESM_ERR_SIG_VDDSHV_MAIN_1P8_OV    (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_OV_OUT_N_TO_ESM_1)
#define MCU_ESM_ERR_SIG_VDDSHV_MAIN_3P3_UV    (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_UV_OUT_N_TO_ESM_0)
#define MCU_ESM_ERR_SIG_VDDSHV_MAIN_3P3_OV    (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU_7POKS0_POK_PGOOD_OV_OUT_N_TO_ESM_0)
#define MCU_ESM_ERR_SIG_VDDA_MCU_UV            (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU0_POK_PGOOD_UV_OUT_N_TO_ESM_2)
#define MCU_ESM_ERR_SIG_VDDA_MCU_OV            (SDLR_MCU_ESM0_ESM_PLS_EVENT0_MCU_PRG_MCU0_POK_PGOOD_UV_OUT_N_TO_ESM_1)



#define  ESM_ERR_SIG_VDDA_PMIC_IN_UV           MCU_ESM_ERR_SIG_VDDA_PMIC_IN_UV
#define  ESM_ERR_SIG_VDD_MCU_UV                MCU_ESM_ERR_SIG_VDD_MCU_UV
#define  ESM_ERR_SIG_VDD_MCU_OV                MCU_ESM_ERR_SIG_VDD_MCU_OV
#define  ESM_ERR_SIG_VDDS_DDRIO_UV              MCU_ESM_ERR_SIG_VDDS_DDRIO_UV
#define  ESM_ERR_SIG_VDDS_DDRIO_OV              MCU_ESM_ERR_SIG_VDDS_DDRIO_OV
#define  ESM_ERR_SIG_VDDR_CORE_UV               MCU_ESM_ERR_SIG_VDDR_CORE_UV
#define  ESM_ERR_SIG_VDDR_CORE_OV               MCU_ESM_ERR_SIG_VDDR_CORE_OV
#define  ESM_ERR_SIG_VDDSHV_MCU_3P3_UV          MCU_ESM_ERR_SIG_VDDSHV_MCU_3P3_UV
#define  ESM_ERR_SIG_VDDSHV_MCU_3P3_OV          MCU_ESM_ERR_SIG_VDDSHV_MCU_3P3_OV
#define  ESM_ERR_SIG_VDDSHV_MCU_1P8_UV          MCU_ESM_ERR_SIG_VDDSHV_MCU_1P8_UV
#define  ESM_ERR_SIG_VDDSHV_MCU_1P8_OV          MCU_ESM_ERR_SIG_VDDSHV_MCU_1P8_OV
#define  ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_UV    MCU_ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_UV
#define  ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_OV    MCU_ESM_ERR_SIG_VMON_CAP_MCU_GENERAL_OV
#define  ESM_ERR_SIG_VDDSHV_MAIN_1P8_UV         MCU_ESM_ERR_SIG_VDDSHV_MAIN_1P8_UV
#define  ESM_ERR_SIG_VDDSHV_MAIN_1P8_OV         MCU_ESM_ERR_SIG_VDDSHV_MAIN_1P8_OV
#define  ESM_ERR_SIG_VDDSHV_MAIN_3P3_UV         MCU_ESM_ERR_SIG_VDDSHV_MAIN_3P3_UV
#define  ESM_ERR_SIG_VDDSHV_MAIN_3P3_OV         MCU_ESM_ERR_SIG_VDDSHV_MAIN_3P3_OV
#define  ESM_ERR_SIG_VDDA_MCU_UV                MCU_ESM_ERR_SIG_VDDA_MCU_UV
#define  ESM_ERR_SIG_VDDA_MCU_OV                MCU_ESM_ERR_SIG_VDDA_MCU_OV





#define  ESM_INSTANCE                      MCU_ESM_INSTANCE
#define  ESM_INTID                         MCU_ESM_INTID






typedef enum{
   SDL_POK_MCU_CTRL_MMR0,

}SDL_POK_InstanceType;

bool SDL_POK_getBaseaddr(SDL_POK_InstanceType instance, uint32_t *pbaseAddress);




#ifdef __cplusplus
}
#endif
#endif /* SDL_SOC_POK_H_ */


