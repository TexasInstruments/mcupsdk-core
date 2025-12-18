
/* Copyright (c) 2022-25 Texas Instruments Incorporated
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
 *  \file     ecc_bus_safety_Interrupt.c
 *
 *  \brief    This file contains Ecc Bus Safety for all the nodes in MSS and DSS example code in Interrupt method.
 *
 *  \details  ECC_BUS_SAFETY APP
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "ecc_bus_safety.h"
#include <sdl/include/hw_types.h>
#include <kernel/dpl/AddrTranslateP.h>
/*===========================================================================*/
/*                         Macro                                             */
/*===========================================================================*/

#define SDL_ECC_BUS_SAFETY_MSS_ERR_STS_CLR      0x01U

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
/* generalized helper function for red test on mss */
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_test(uint32_t busSftyNode,\
    SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);
/* generalized helper function for ded test on mss */
static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_test(uint32_t busSftyNode ,uint32_t addr);
/* generalized helper function for sec test on mss */
static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_test(uint32_t busSftyNode ,uint32_t addr);
/* check L2OCRAM RAM bus node*/
static bool SDL_ECC_BUS_SAFETY_MSS_isL2OCRAMNode(uint32_t busSftyNode);
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
/* generalized helper function for red test  */
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
       uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);
/* generalized helper function for ded test  */
static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
            uint32_t busSftyNode, uint32_t addr);
/* generalized helper function for sec test */
static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
            uint32_t busSftyNode, uint32_t addr);

#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
/* Call back for MSS SEC for all the nodes */
int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,int32_t grpChannel,int32_t vecNum,void *arg);

/* Call back for MSS RED on  MSS_TPTC_A0_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for MSS RED on  MSS_TPTC_A1_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                   void *arg);
/* Call back for MSS RED on  MSS_TPTC_B0_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for MSS RED on  MSS_CR5A_AXI_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                            int32_t grpChannel,
                                                                            int32_t vecNum,
                                                                            void *arg);
/* Call back for MSS RED on  MSS_CR5B_AXI_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                        int32_t grpChannel,
                                                                        int32_t vecNum,
                                                                        void *arg);
/* Call back for MSS DED and RED on  MSS_TPTC_A0_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for MSS DED and RED on  MSS_TPTC_A1_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for MSS RED on  MSS_TPTC_B0 */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for MSS DED and RED on  MSS_CR5A_AHB */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                          int32_t grpChannel,
                                                                           int32_t vecNum,
                                                                          void *arg);
/* Call back for MSS DED and RED on MSS_CR5B_AHB */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                           int32_t grpChannel,
                                                                          int32_t vecNum,
                                                                        void *arg);
/* Call back for MSS DED and RED on  MSS_CR5B_AHB */
int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                            int32_t grpChannel,
                                                                            int32_t vecNum,
                                                                            void *arg);
/* Call back for MSS DED and RED on MSS_CR5A_AXI */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                          int32_t grpChannel,
                                                                          int32_t vecNum,
                                                                          void *arg);
/* Call back for MSS DED and RED on MSS_CR5B_AXI_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                  int32_t grpChannel,
                                                                  int32_t vecNum,
                                                                  void *arg);
/* Call back for MSS DED and RED on MSS_CR5A_AXI_S */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                              int32_t grpChannel,
                                                              int32_t vecNum,
                                                              void *arg);
/* Call back for MSS DED and RED on MSS_CR5B_AXI_S */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for MSS DED and RED on MSS_L2_A */
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for MSS DED and RED on MSS_L2_B */
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for MSS DED and RED on MSS_GPADC */
int32_t SDL_ECC_BUS_SAFETY_MSS_GPADC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for MSS DED and RED on MSS_DMM_SLV */
int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for MSS DED and RED on MSS_DMM */
int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for MSS DED and RED on MSS_PCR */
int32_t SDL_ECC_BUS_SAFETY_MSS_PCR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for MSS DED and RED on MSS_PCR2 */
int32_t SDL_ECC_BUS_SAFETY_MSS_PCR2_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for MSS DED and RED on MSS_CPSW */
int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                              int32_t grpChannel,
                                                              int32_t vecNum,
                                                              void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                              int32_t grpChannel,
                                                              int32_t vecNum,
                                                              void *arg);


int32_t SDL_ECC_BUS_SAFETY_MSS_SWBUF_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                            int32_t grpChannel,
                                                            int32_t vecNum,
                                                            void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                          int32_t grpChannel,
                                                          int32_t vecNum,
                                                          void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TO_MDO_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                          int32_t grpChannel,
                                                          int32_t vecNum,
                                                          void *arg);

int32_t SDL_ECC_BUS_SAFETY_DAP_R232_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                          int32_t grpChannel,
                                                          int32_t vecNum,
                                                          void *arg);

#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
/* Call back for DED and RED on MSS_CR5A_AHB */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5B_AHB */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/* Call back for DED and RED on MSS_CR5C_AHB */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5D_AHB */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif
/* Call back for SEC for all the nodes */
int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX__SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

/* Call back for DED on MSS_TPTC_A0_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED on MSS_TPTC_A1_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_TPTC_A0_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_TPTC_A1_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5A_AXI_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5B_AXI_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/* Call back for DED and RED on MSS_CR5C_AXI_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5D_AXI_WR */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif
/* Call back for DED and RED on MSS_CR5A_AXI_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5B_AXI_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/* Call back for DED and RED on MSS_CR5C_AXI_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                        uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5D_AXI_RD */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                        uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif
/* Call back for DED and RED on MSS_CR5A_AXI_S */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5B_AXI_S */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/* Call back for DED and RED on MSS_CR5C_AXI_S */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_CR5D_AXI_S */
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif
/* Call back for DED and RED on MSS_L2_A */
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_L2_B */
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for DED and RED on MSS_L2_C */
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/* Call back for DED and RED on MSS_L2_D */
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif
/* Call back for DED and RED on MSS_CPSW */
int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#if defined (SOC_AM261X)
/* Call back for DED and RED on MSS_GPMC */
int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif

/* Call back for DED and RED on MSS_MMC */
int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for RED on MSS_MAIN_VBUSP */
int32_t SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
/* Call back for RED on MSS_PERI_VBUSP */
int32_t SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
											uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

#if defined(SOC_AM263X)
int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#elif defined (SOC_AM261X)
int32_t SDL_ECC_BUS_SAFETY_MSS_OSPI_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif

int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

#if defined (SOC_AM261X)
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#else
int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif

int32_t SDL_ECC_BUS_SAFETY_DAP_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
#endif

#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
/* generalized helper function for red test on dss */
static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);
/* generalized helper function for ded test on dss */
static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_test(uint32_t busSftyNode ,uint32_t addr);
/* generalized helper function for sec test on dss */
static int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_test(uint32_t busSftyNode ,uint32_t addr);
/* Call back for DSS SEC for all the nodes */
int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
/* Call back for DSS DED and RED on DSS_MCRC */
int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                             int32_t grpChannel,
                                                                             int32_t vecNum,
                                                                             void *arg);
/* Call back for DSS DED and RED on DSS_L3_BANKA */
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_L3_BANKB */
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_L3_BANKC */
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_L3_BANKD */
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on HWA_DMA0 */
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on HWA_DMA1 */
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_MBOX */
int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_CBUFF_FIFO */
int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_A0_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_A1_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_B0_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_B1_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_C0_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_C1_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_C2_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_C3_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_C4_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED on DSS_TPTC_C5_WR */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_A1_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_A1 */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_B0_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_B1_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_C0_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_C1_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_C2_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_C3_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_C4_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_TPTC_C5_RD */
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
/* Call back for DSS DED and RED on DSS_PCR */
int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg);
/* Call back for DSS DED and RED on DSP_MDMA */
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg);
/* Call back for DSS DED and RED on DSP_SDMA */
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg);
#endif
#endif

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
volatile bool mssSecFlag = FALSE;
volatile bool SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_DAP_R232+1U];
#endif
#endif


#if  defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
volatile bool mssSecFlag = FALSE;
volatile bool SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_SEC_END_NODE+1U];
#endif


#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
/* Event BitMap for ECC ESM callback for MSS */
SDL_ESM_NotifyParams ECC_BUS_SAFETY_TestparamsMSS[30U] =
{
    {
        /* Event BitMap for ECC ESM callback for MSS TPTC A0 WR */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MSS_TPTCA0_WR ,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for MSS TPTC A1 WR */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MSS_TPTCA1_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for MSS TPTC B0 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MSS_TPTCB0_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for MSS TPTC A0 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MSS_TPTCA0_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for MSS TPTC A1 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MSS_TPTCA1_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for MSS TPTC B0 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MSS_TPTCB0_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for MSS SEC*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_MSS_AGG_SEC_BUS_SAFETY,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS CR5A AHB*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_CR5A_AHB,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS CR5B AHB*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_CR5B_AHB,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS CR5A AXI_S*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_CR5A_SLV,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS CR5B AXI_S*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_CR5B_SLV,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS MBOX*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MSS_MBOX,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for RED for MSS CR5A AXI_WR*/
        .groupNumber = SDL_INTR_GROUP2_NUM,
        .errorNumber = SDL_ESMG2_BUS_SAFETY_CR5A_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for RED for MSS CR5B AXI_WR*/
        .groupNumber = SDL_INTR_GROUP2_NUM,
        .errorNumber = SDL_ESMG2_BUS_SAFETY_CR5B_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS CR5A AXI_RD*/
        .groupNumber = SDL_INTR_GROUP2_NUM,
        .errorNumber = SDL_ESMG2_BUS_SAFETY_CR5A_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS CR5B AXI_RD*/
        .groupNumber = SDL_INTR_GROUP2_NUM,
        .errorNumber = SDL_ESMG2_BUS_SAFETY_CR5B_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for  MSS L2 A*/
        .groupNumber = SDL_INTR_GROUP2_NUM,
        .errorNumber = SDL_ESMG2_BUS_SAFETY_MSS_L2BANKA,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for  MSS L2 B*/
        .groupNumber = SDL_INTR_GROUP2_NUM,
        .errorNumber = SDL_ESMG2_BUS_SAFETY_MSS_L2BANKB,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for  MSS GPADC*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_GPADC,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_GPADC_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for  MSS DMM SLV*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_DMM_SLAVE,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for  MSS DMM */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_DMM_MST,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_DMM_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS PCR */
        .groupNumber = SDL_INTR_GROUP2_NUM,
        .errorNumber = SDL_ESMG2_BUS_SAFETY_PCRA,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_PCR_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS PCR */
        .groupNumber = SDL_INTR_GROUP2_NUM,
        .errorNumber = SDL_ESMG2_BUS_SAFETY_PCRB,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_PCR2_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS PCR */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_CCPSW,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS MCRC*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MCRC,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction,
     },
     {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS QSPI*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_QSPI,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_RED_ESM_ApplicationCallbackFunction,
     },
     {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS SWBUF*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MSS_RET_RAM,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_SWBUF_DED_RED_ESM_ApplicationCallbackFunction,
     },
     {
        /* Event BitMap for ECC ESM callback for DED and RED for MSS_TO_MDO*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_MDO,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_TO_MDO_DED_RED_ESM_ApplicationCallbackFunction,
     },
     {
        /* Event BitMap for ECC ESM callback for DED and RED for DAP_R232*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_DAP_RS232,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DAP_R232_DED_RED_ESM_ApplicationCallbackFunction,
     },
     {
        /* Event BitMap for ECC ESM callback for DED and RED for SCRP*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_ESMG1_BUS_SAFETY_SCRP,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_MSS_SCRP_DED_RED_ESM_ApplicationCallbackFunction,
     },

};
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX)

SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_RED =
{
   .esmErrorConfig = {1u, 8u}, /* Self test error config */
   .enableBitmap =   {0x80000000u, 0x00000002u, 0x00000000u, 0x00000000u,},
   .priorityBitmap = {0x80000000u, 0x00000002u, 0x00000000u, 0x00000000u,},
   .errorpinBitmap = {0x80000000u, 0x00000002u, 0x00000000u, 0x00000000u,},
};

SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_DED =
{
   .esmErrorConfig = {1u, 8u}, /* Self test error config */
   .enableBitmap =   {0x00000000u, 0x00000002u, 0x00000000u, 0x00000000u,},
   .priorityBitmap = {0x00000000u, 0x00000002u, 0x00000000u, 0x00000000u,},
   .errorpinBitmap = {0x00000000u, 0x00000002u, 0x00000000u, 0x00000000u,},
};

SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_SEC =
{
   .esmErrorConfig = {1u, 8u}, /* Self test error config */
   .enableBitmap =   {0x00000000u, 0x00000004u, 0x00000000u, 0x00000000u,},
   .priorityBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,},
   .errorpinBitmap = {0x00000000u, 0x00000004u, 0x00000000u, 0x00000000u,},
};

#endif

#if defined (SOC_AM261X)

SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_RED =
{
   .esmErrorConfig = {1u, 8u}, /* Self test error config */
   .enableBitmap =   {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,},
   .priorityBitmap = {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,},
   .errorpinBitmap = {0x00180000u, 0x00000000u, 0x00000000u, 0x00000000u,},
};

SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_DED =
{
   .esmErrorConfig = {1u, 8u}, /* Self test error config */
   .enableBitmap =   {0x00100000u, 0x00000000u, 0x00000000u, 0x00000000u,},
   .priorityBitmap = {0x00100000u, 0x00000000u, 0x00000000u, 0x00000000u,},
   .errorpinBitmap = {0x00100000u, 0x00000000u, 0x00000000u, 0x00000000u,},
};

SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_SEC =
{
   .esmErrorConfig = {1u, 8u}, /* Self test error config */
   .enableBitmap =   {0x00200000u, 0x00000000u, 0x00000000u, 0x00000000u,},
   .priorityBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,},
   .errorpinBitmap = {0x00200000u, 0x00000000u, 0x00000000u, 0x00000000u,},
};

#endif


#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)

volatile bool SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO+1];

#endif
#endif

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

volatile bool SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_MBOX+1];

#endif
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

volatile bool dssSecFlag = FALSE;

#endif
#endif

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

/* Event BitMap for ECC ESM callback for DSS */
SDL_ESM_NotifyParams ECC_BUS_SAFETY_TestparamsDSS[33U] =
{
    {
        /* Event BitMap for ECC ESM callback for DSS SEC */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_SEC_AGG ,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_SEC_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKA */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_L3RAM0,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKB*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_L3RAM1,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKC*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_L3RAM2,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_L3RAM3,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS HWA DMA 0*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_HWA_DMA0,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS HWA DMA 1*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_HWA_DMA1,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS MBOX*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_MAILBOX,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and DED for DSS MCRC*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_MCRC,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS CBUFF FIFO*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_CBUFF  ,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC A0 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCA0_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC A1 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCA1_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC B0 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCB0_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC B1 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCB1_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C0 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC0_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C1 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC1_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C2 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC2_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C3 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC3_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C4 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC4_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C5 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC5_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC A0 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCA0_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC A1 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCA1_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC B0 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCB0_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC B1 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCB1_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C0 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC0_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C1 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC1_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C2 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC2_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C3 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC3_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C4 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC4_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C5 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC5_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS PCR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_PCR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_PCR_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS MDMA*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_DSP_MDMA,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS SDMA*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_DSP_SDMA,
        .setIntrPriorityLvl = SDL_ENABLE_ERR_PIN,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_RED_ESM_ApplicationCallbackFunction,
    },
};

#endif
#endif

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
/* Event BitMap for ECC ESM callback for DSS */
SDL_ESM_NotifyParams ECC_BUS_SAFETY_TestparamsDSS[33U] =
{
    {
        /* Event BitMap for ECC ESM callback for DSS SEC */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_SEC_AGG ,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_SEC_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKA */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_L3RAM0,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKB*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_L3RAM1,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKC*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_L3RAM2,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_L3RAM3,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS HWA DMA 0*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_HWA_DMA0,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS HWA DMA 1*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_HWA_DMA1,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS MBOX*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_MAILBOX,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and DED for DSS MCRC*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_MCRC,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS CBUFF FIFO*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_CBUFF  ,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC A0 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCA0_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC A1 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCA1_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC B0 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCB0_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC B1 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCB1_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C0 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC0_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C1 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC1_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C2 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC2_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C3 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC3_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C4 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC4_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C5 WR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC5_WR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC A0 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCA0_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC A1 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCA1_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC B0 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCB0_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC B1 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCB1_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C0 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC0_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C1 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC1_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C2 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC2_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C3 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC3_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C4 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC4_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS TPTC C5 RD*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_TPTCC5_RD,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS PCR*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_PCR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_PCR_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS MDMA*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_DSP_MDMA,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_DED_RED_ESM_ApplicationCallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS SDMA*/
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG1_DSS_BUS_SAFETY_DSP_SDMA,
        .setIntrPriorityLvl = SDL_ENABLE_ERR_PIN,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_RED_ESM_ApplicationCallbackFunction,
    },
};
#endif
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
SDL_ESM_OpenParams gEsmOpenParams=
{
    TRUE
    /* boolean value to indicate if old ESM pending errors should be cleared or not
          This field will be set by SysCfg.*/
};
#endif

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
/********************************************************************************************************
*   ESM Callback Function for SEC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg)
{
    int32_t retVal = SDL_PASS;
    /*  MSS_TPTC_A0_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD );
    /*  MSS_TPTC_A1_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD );
    /*  MSS_TPTC_B0_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD );
    /* MSS_MBOX */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX );
    /*  MSS_MCRC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC);
    /*  MSS_QSPI */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_QSPI);
    /*  MSS_SWBUF */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_SWBUF);
    /*  MSS_TO_MDO */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TO_MDO);
    /*  DAP_R232 */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_DAP_R232);
    /*  MSS_AXI_CR5A_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD );
    /*  MSS_AXI_CR5B_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD );
    /*  MSS_AXI_CR5A_S */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S );
    /*  MSS_AXI_CR5B_S */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S );
    /*  MSS_PCR */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_PCR);
    /*  MSS_PCR2 */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_PCR2);
    /* MSS_L2_A */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A);
    /* MSS_L2_B */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B);
    /* MSS_GPADC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPADC);
    /* MSS_GPADC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPADC);
    /* MSS_DMM_SLV */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_DMM_SLV);
    /* MSS_DMM_SLV */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW);
    mssSecFlag = TRUE;
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_MBOX ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MBOX] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MBOX] = FALSE;
    }
    return retVal;
}


/********************************************************************************************************
*   For Node MSS_TPTC_A0_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_B0_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD] = FALSE;
    }
    return retVal;
}



/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_B0_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_CR5A_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR] = FALSE;
    }
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                                void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_CR5B_AHB ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD] = FALSE;
    }
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD] = FALSE;
    }
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S] = FALSE;
    }
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_MCRC ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MCRC] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MCRC] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_QSPI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_QSPI ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_QSPI )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_QSPI] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_QSPI] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_SWBUF ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SWBUF_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_SWBUF ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_SWBUF)))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_SWBUF] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_SWBUF] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_TO_MDO ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TO_MDO_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TO_MDO ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TO_MDO )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TO_MDO] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TO_MDO] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_SCRP ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_SCRP] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_SCRP] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node DAP_R232 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DAP_R232_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DAP_R232 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_DAP_R232 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_DAP_R232] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_DAP_R232] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_L2_A ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_A] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_A] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_L2_B ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B )))

    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_B] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_B] = FALSE;
    }
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_GPADC ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_GPADC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPADC ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPADC )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_GPADC] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_GPADC] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_DMM_SLV ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_DMM_SLV ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_DMM_SLV )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_DMM_SLV] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_DMM_SLV] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_DMM ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_DMM))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_DMM] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_DMM] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_PCR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_PCR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_PCR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_PCR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_PCR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_PCR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_PCR2 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_PCR2_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_PCR2 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_PCR2)))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_PCR2] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_PCR2] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CPSW ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW)))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CPSW] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CPSW] = FALSE;
    }
    return retVal;
}
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM261X)
/********************************************************************************************************
*   For Node MSS_GPMC ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPMC ))&&
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPMC )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_GPMC] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_GPMC] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)

/* To test L2OCRAM memory test, need to disable cache to  */
/* avoid code crash as test is running on OCRAM only      */
static bool SDL_ECC_BUS_SAFETY_MSS_isL2OCRAMNode(uint32_t busSftyNode)
{
    bool retVal = false;
    if((busSftyNode == SDL_ECC_BUS_SAFETY_MSS_L2_A) || (busSftyNode == SDL_ECC_BUS_SAFETY_MSS_L2_B) || 
       #if defined (SOC_AM263PX) 
       (busSftyNode == SDL_ECC_BUS_SAFETY_MSS_L2_E) || (busSftyNode == SDL_ECC_BUS_SAFETY_MSS_L2_F) ||
       #endif
       #if defined (SOC_AM263X) || defined (SOC_AM263PX) 
       (busSftyNode == SDL_ECC_BUS_SAFETY_MSS_L2_D) || 
       #endif
       (busSftyNode == SDL_ECC_BUS_SAFETY_MSS_L2_C))
    {
        retVal = true;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_L2_A ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A ))&&
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_A] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_A] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_L2_B ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B ))&&
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_B] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_B] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_L2_C ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_C ))&&
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_C )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_C] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_C] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_L2_D ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_D ))&&
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_D )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_D] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_D] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif
#if defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_L2_E ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_E_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_E ))&&
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_E )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_E] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_E] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_L2_F ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_F_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_F ))&&
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_F )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_F] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_L2_F] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif
/********************************************************************************************************
*   For Node MSS_MMC ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_MMC )) && \
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_MMC )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MMC] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MMC] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CPSW ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW )) && \
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CPSW] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CPSW] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_MAIN_VBUSP ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP ))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node SS_PERI_VBUSP ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP ))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_CR5A_AHB ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB ))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_CR5B_AHB ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB ))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_CR5C_AHB ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_CR5D_AHB ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

/********************************************************************************************************
*   ESM Callback Function for SEC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_STM_STIM);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MMC);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_S);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_C);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_D);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

#if defined (SOC_AM263PX)
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_E_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_E);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_F_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_F);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}                                        
int32_t SDL_ECC_BUS_SAFETY_DAP_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_DAP);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;

    /*  MSS_TPTC_A0_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD );
    /*  MSS_TPTC_A1_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD );
    /*  MSS_MBOX */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX);
    /*  MSS_MMC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MMC);
    #if defined(SOC_AM261X)
    /*  MSS_GPMC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPMC);
    #endif
    /* MSS_L2_A */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A);
    /* MSS_L2_B */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B);
    /* MSS_L2_C */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_C);
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
    /* MSS_L2_D */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_D);
#endif
    /* MSS_CPSW */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW);
    /* MSS_MCRC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC);
    mssSecFlag = TRUE;    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_A0_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_CR5C_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif
/********************************************************************************************************
*   For Node MSS_CR5A_AXI_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_CR5C_AXI_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S ))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S ))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S ))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S ))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

/********************************************************************************************************
*   For Node MSS_MCRC ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MCRC] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MCRC] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

#if defined(SOC_AM263X)
/********************************************************************************************************
*   For Node MSS_QSPI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_QSPI ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_QSPI )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_QSPI] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_QSPI] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#elif defined (SOC_AM263PX) || defined (SOC_AM261X)
/********************************************************************************************************
*   For Node MSS_OSPI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_OSPI_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_OSPI ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_OSPI )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_OSPI] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_OSPI] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif
#if defined (SOC_AM261X)
/********************************************************************************************************
*   For Node MSS_USBSS_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_USBSS_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_USBSS_RD )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_USBSS_RD] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_USBSS_RD] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_USBSS_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_USBSS_WR))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_USBSS_WR)))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_USBSS_WR] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_USBSS_WR] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

/********************************************************************************************************
*   For Node MSS_MBOX ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MBOX] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MBOX] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_STM_STIM ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_STM_STIM ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_STM_STIM )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_STM_STIM] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_STM_STIM] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_SCRP0 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP0 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP0 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_SCRP0] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_SCRP0] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_SCRP1 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP1 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_SCRP1 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_SCRP1] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_SCRP1] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

#if defined (SOC_AM261X)
/********************************************************************************************************
*   For Node ICSSM_PDSP0 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node ICSSM_PDSP1 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node ICSSM_SLAVE ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node ICSSM_PDSP0 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node ICSSM_PDSP1 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node ICSSM_SLAVE ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S] = FALSE;
    }    
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#else
/********************************************************************************************************
*   For Node ICSSM_PDSP0 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_PDSP0 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_PDSP0 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_ICSSM_PDSP0] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_ICSSM_PDSP0] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node ICSSM_PDSP1 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_PDSP1 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_PDSP1 )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_ICSSM_PDSP1] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_ICSSM_PDSP1] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node ICSSM_SLAVE ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_S ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_S )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_ICSSM_S] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_ICSSM_S] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif
/********************************************************************************************************
*   For Node DAP ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DAP_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DAP ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_DAP )))
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_DAP] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_DAP] = FALSE;
    }
    /* Clear ESM registers. */
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
#endif

#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
/********************************************************************************************************
*   For Node MSS_MBOX
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MSS_MBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MSS_MBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U));
}
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node MSS_TPTC_B0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD,0U));
}
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_B0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5B_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_MCRC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_MCRC,SDL_MSS_MCRC_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_MCRC,SDL_MSS_MCRC_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_MCRC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_QSPI
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_QSPI,SDL_MSS_QSPI_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_QSPI,SDL_MSS_QSPI_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_QSPI, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_SWBUF
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SWBUF_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_SWBUF,SDL_MSS_SWBUF_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SWBUF_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_SWBUF,SDL_MSS_SWBUF_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SWBUF_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_SWBUF, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TO_MDO
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TO_MDO_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_TO_MDO, 0xCA000000U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TO_MDO_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_TO_MDO, 0xCA000000U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TO_MDO_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_TO_MDO, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DAP_R232
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DAP_R232_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_DAP_R232, 0U));
}

int32_t SDL_ECC_BUS_SAFETY_DAP_R232_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_DAP_R232, 0U));
}

int32_t SDL_ECC_BUS_SAFETY_DAP_R232_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_DAP_R232, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_SCRP
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_SCRP, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_PCR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_PCR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_PCR,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_PCR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_PCR,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_PCR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_PCR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_PCR2
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_PCR2_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_PCR2,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_PCR2_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_PCR2,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_PCR2_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_PCR2, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CPSW
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_CPSW,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_CPSW,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_CPSW, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_L2_A
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_L2_A,SDL_MSS_L2_A_BASE_END));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_L2_A,SDL_MSS_L2_A_BASE_END));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_L2_A, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_L2_B
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_L2_B,SDL_MSS_L2_B_BASE_START));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_L2_B,SDL_MSS_L2_B_BASE_START));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_L2_B, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_GPADC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_GPADC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_GPADC,SDL_MSS_GPADC_DATA_RAM_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_GPADC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_GPADC,SDL_MSS_GPADC_DATA_RAM_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_GPADC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_GPADC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_DMM_SLV
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ECC_BUS_SAFETY_MSS_DMM_SLV,SDL_MSS_DMM_A_DATA_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ECC_BUS_SAFETY_MSS_DMM_SLV,SDL_MSS_DMM_A_DATA_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_DMM_SLV, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_DMM
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_DMM_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ECC_BUS_SAFETY_MSS_DMM, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
/********************************************************************************************************
*   For Node MSS_L2_A
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_A,SDL_L2OCRAM_BANK0));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_A,SDL_L2OCRAM_BANK0));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_L2_A, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_L2_B
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_B,SDL_L2OCRAM_BANK1));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_B,SDL_L2OCRAM_BANK1));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_L2_B, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_L2_C
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_C,SDL_L2OCRAM_BANK2));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_C,SDL_L2OCRAM_BANK2));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_L2_C, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_L2_D
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_D,SDL_L2OCRAM_BANK3));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_D,SDL_L2OCRAM_BANK3));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_L2_D, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif
#if defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_L2_E
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_E_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_E,SDL_L2OCRAM_BANK4));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_E_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_E,SDL_L2OCRAM_BANK4));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_E_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_L2_E, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_L2_F
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_F_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_F,SDL_L2OCRAM_BANK5));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_F_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_F,SDL_L2OCRAM_BANK5));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_F_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_L2_F, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif
/********************************************************************************************************
*   For Node MSS_MMC_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_MMC,SDL_MMC0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_MMC,SDL_MMC0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_MMC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CPSW
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CPSW, SDL_MSS_CPSW_BASE+8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CPSW,SDL_MSS_CPSW_BASE+8U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CPSW, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#if defined (SOC_AM263X) || defined(SOC_AM261X)
/********************************************************************************************************
*   For Node MSS_GPMC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_GPMC,SDL_GPMC0_CFG_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_GPMC,SDL_GPMC0_CFG_U_BASE+0x64));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_GPMC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif

/********************************************************************************************************
*   For Node MSS_MAIN_VBUSP
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_MAIN_VBUSP
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5A_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5B_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, &ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_CR5C_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5D_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, &ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif

/********************************************************************************************************
*   For Node MSS_TPTC_A0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,SDL_MSS_CTRL_TPCC_A0_RD_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,SDL_MSS_CTRL_TPCC_A0_RD_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,SDL_MSS_CTRL_TPCC_A1_RD_BASE));
}
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,SDL_MSS_CTRL_TPCC_A1_RD_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR,SDL_MSS_CTRL_TPCC_A0_WR_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR,SDL_MSS_CTRL_TPCC_A0_WR_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR,SDL_MSS_CTRL_TPCC_A1_WR_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR,SDL_MSS_CTRL_TPCC_A1_WR_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR,SDL_MSS_CR5A_AXI_WR_START + 0x8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR,SDL_MSS_CR5A_AXI_WR_START + 0x8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR,SDL_MSS_CR5B_AXI_WR_START + 0x8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR,SDL_MSS_CR5B_AXI_WR_START + 0x8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_CR5C_AXI_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR,SDL_MSS_CR5C_AXI_WR_START + 0x8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR,SDL_MSS_CR5C_AXI_WR_START + 0x8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR,SDL_MSS_CR5D_AXI_WR_START + 0x8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR,SDL_MSS_CR5D_AXI_WR_START + 0x8));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,SDL_MSS_CR5A_AXI_RD_START));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD, SDL_MSS_CR5A_AXI_RD_START+0x68U));
}
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD,SDL_MSS_CR5B_AXI_RD_START+68U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD,SDL_MSS_CR5B_AXI_RD_START+68U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_CR5C_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD,SDL_MSS_CR5C_AXI_RD_START+68U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD,SDL_MSS_CR5C_AXI_RD_START+68U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD,SDL_MSS_CR5D_AXI_RD_START+68U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD,SDL_MSS_CR5D_AXI_RD_START+68U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S,(SDL_R5SS0_CORE0_TCMB_U_BASE+0x64U)));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S,(SDL_R5SS0_CORE0_TCMB_U_BASE+0x64U)));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S,(SDL_R5SS0_CORE1_TCMB_U_BASE+0x500U)));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S,(SDL_R5SS0_CORE1_TCMB_U_BASE+0x500U)));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
/********************************************************************************************************
*   For Node MSS_CR5C_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S,(SDL_R5SS1_CORE0_TCMB_U_BASE+0x500U)));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S,(SDL_R5SS1_CORE0_TCMB_U_BASE+0x500U)));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S, (SDL_R5SS1_CORE1_TCMB_U_BASE+0x500U)));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S,(SDL_R5SS1_CORE1_TCMB_U_BASE+0x500U)));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif

/********************************************************************************************************
*   For Node MSS_MCRC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                        SDL_ECC_BUS_SAFETY_MSS_MCRC,SDL_MCRC_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                        SDL_ECC_BUS_SAFETY_MSS_MCRC,SDL_MCRC_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
            SDL_ECC_BUS_SAFETY_MSS_MCRC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#if defined (SOC_AM263X)
/********************************************************************************************************
*   For Node MSS_QSPI
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_QSPI,SDL_QSPI_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_QSPI,SDL_QSPI_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_QSPI, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#elif defined (SOC_AM263PX) || defined (SOC_AM261X)
/********************************************************************************************************
*   For Node MSS_OSPI
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_OSPI_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_OSPI,SDL_OSPI_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_OSPI_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_OSPI,SDL_OSPI_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_OSPI_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_OSPI, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif

#if defined (SOC_AM261X)
/********************************************************************************************************
*   For Node MSS_USBSS_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_USBSS_RD,SDL_USB_RD_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_USBSS_RD,SDL_USB_RD_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_USBSS_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_USBSS_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_USBSS_WR,SDL_USB_WR_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_USBSS_WR,SDL_USB_WR_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_USBSS_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif
/********************************************************************************************************
*   For Node MSS_MBOX
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_MBOX, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_STM_STIM
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_STM_STIM,SDL_MSS_STM_STIM_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_STM_STIM,SDL_MSS_STM_STIM_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_STM_STIM, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_SCRP0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_SCRP0,SDL_SCRP0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_SCRP0,SDL_SCRP0_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_SCRP0, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_SCRP1
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_SCRP1,SDL_SCRP1_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_SCRP1,SDL_SCRP1_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_SCRP1, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#if defined (SOC_AM261X)
/********************************************************************************************************
*   For Node ICSSM_PDSP0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0,SDL_ICSSM0_PDSP0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0,SDL_ICSSM0_PDSP0_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_PDSP1
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1,SDL_ICSSM0_PDSP1_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1,SDL_ICSSM0_PDSP1_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_SLAVE
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                    SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S,SDL_ICSSM0_S_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S,SDL_ICSSM0_S_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node ICSSM_PDSP0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0,SDL_ICSSM1_PDSP0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0,SDL_ICSSM1_PDSP0_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_PDSP1
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1,SDL_ICSSM1_PDSP1_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1,SDL_ICSSM1_PDSP1_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_SLAVE
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                    SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S,SDL_ICSSM1_S_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S,SDL_ICSSM1_S_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#else
/********************************************************************************************************
*   For Node ICSSM_PDSP0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_ICSSM_PDSP0,SDL_ICSSM_PDSP0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_ICSSM_PDSP0,SDL_ICSSM_PDSP0_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_ICSSM_PDSP0, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_PDSP1
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                      SDL_ECC_BUS_SAFETY_ICSSM_PDSP1,SDL_ICSSM_PDSP1_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_ICSSM_PDSP1,SDL_ICSSM_PDSP1_U_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_ICSSM_PDSP1, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_SLAVE
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                    SDL_ECC_BUS_SAFETY_ICSSM_S,SDL_ICSSM_S_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_ICSSM_S,SDL_ICSSM_S_BASE+64U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_ICSSM_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif

/********************************************************************************************************
*   For Node DAP
*********************************************************************************************************/
//#define CSL_TOP_RCM_HW_SPARE_RW0                                               (0x00000FD0U)
int32_t SDL_ECC_BUS_SAFETY_DAP_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_SEC,\
                                    SDL_ECC_BUS_SAFETY_DAP,SDL_TOP_RCM_U_BASE+CSL_TOP_RCM_HW_SPARE_RW0));
}

int32_t SDL_ECC_BUS_SAFETY_DAP_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_DED,\
                                      SDL_ECC_BUS_SAFETY_DAP,SDL_TOP_RCM_U_BASE+CSL_TOP_RCM_HW_SPARE_RW0));
}

int32_t SDL_ECC_BUS_SAFETY_DAP_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_RED, \
          SDL_ECC_BUS_SAFETY_DAP, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif

#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
/********************************************************************************************************
*   For SEC
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_test(uint32_t busSftyNode ,uint32_t addr)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;
    ret_val = SDL_ECC_BUS_SAFETY_MSS_secExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode)) ||
            (SDL_ECC_BUS_SAFETY_MSS_PCR == busSftyNode) || (SDL_ECC_BUS_SAFETY_MSS_PCR2 == busSftyNode ))
        {
            /* get EDMA parameter */
            SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            /* initiate the EDMA transfer */
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else if (SDL_ECC_BUS_SAFETY_MSS_CPSW ==busSftyNode  )
        {
            /* CPSW SetUP */
            setup_CPSW();
            /* CPSW Transfer */
            cpsw_transfer();
        }
        else
        {
            /* do nothing */
        }
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((mssSecFlag!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(mssSecFlag==TRUE)
        {
            mssSecFlag=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    return ret_val;
}

/********************************************************************************************************
*   For DED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_test(uint32_t busSftyNode ,uint32_t addr)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;
    ret_val = SDL_ECC_BUS_SAFETY_MSS_dedExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode)) ||
            (SDL_ECC_BUS_SAFETY_MSS_PCR == busSftyNode) || (SDL_ECC_BUS_SAFETY_MSS_PCR2 == busSftyNode ))
        {
        SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
        test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else if (SDL_ECC_BUS_SAFETY_MSS_CPSW ==busSftyNode  )
        {
            /* CPSW SetUP */
            setup_CPSW();
            /* CPSW Transfer */
            cpsw_transfer();
        }
        else
        {
            /* do nothing */
        }
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((SDL_MSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(SDL_MSS_intrFlg[busSftyNode]==TRUE)
        {
            SDL_MSS_intrFlg[busSftyNode]=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
             ret_val = SDL_EFAIL;
        }
    }
    return ret_val;
}
/********************************************************************************************************
*   For RED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_test(uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    ret_val= SDL_ECC_BUS_SAFETY_MSS_redExecute(busSftyNode, fiType, redType);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((SDL_MSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(SDL_MSS_intrFlg[busSftyNode]==TRUE)
        {
            SDL_MSS_intrFlg[busSftyNode]=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    return ret_val;
}

#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
/********************************************************************************************************
*   For SEC
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
                                                                      uint32_t busSftyNode, uint32_t addr)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;

    switch (busSftyNode)
    {
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_MBOX:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_MCRC:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_STM_STIM:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_STM_STIM_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_MMC:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_L2_C:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_L2_C_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
        case SDL_ECC_BUS_SAFETY_ICSSM_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_ICSSM_S_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_L2_D:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_L2_D_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
#if defined (SOC_AM263PX)
        case SDL_ECC_BUS_SAFETY_MSS_L2_E:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_L2_E_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_L2_F:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_L2_F_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        case SDL_ECC_BUS_SAFETY_MSS_L2_A:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_L2_A_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_L2_B:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_L2_B_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_CPSW:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_DAP:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_DAP_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        default:
        {
            ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
    }
    if (ret_val == SDL_PASS)
    {
        if(SDL_ECC_BUS_SAFETY_MSS_isL2OCRAMNode(busSftyNode))
        {
            CacheP_disable(CacheP_TYPE_L1D);
        }
        ret_val = SDL_ECC_BUS_SAFETY_MSS_secExecute(busSftyNode,addr,writeData);
        if(SDL_ECC_BUS_SAFETY_MSS_isL2OCRAMNode(busSftyNode))
        {
            CacheP_enable(CacheP_TYPE_L1D);
        }
        if(ret_val !=SDL_PASS )
        {
            ret_val = SDL_EFAIL;
            DebugP_log("\n ECC_BUS_SAFETY_MSS_secExecuted for node %d \r\n", busSftyNode);
        }
        else
        {
            
            #if defined (SOC_AM263X) || defined (SOC_AM263PX)
            if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR>=busSftyNode)) ||
            ((SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD>=busSftyNode)) || 
            (busSftyNode == SDL_ECC_BUS_SAFETY_DAP))
            #endif
            #if defined (SOC_AM261X) 
            if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD>=busSftyNode)) ||
            ((SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD>=busSftyNode)) || 
            (busSftyNode == SDL_ECC_BUS_SAFETY_DAP))
            #endif
            {
                SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
                test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
            }
            else if (SDL_ECC_BUS_SAFETY_MSS_CPSW == busSftyNode  )
            {
                /* CPSW SetUP */
                setup_CPSW();
                /* CPSW Transfer */
                cpsw_transfer();
            }
            else
            {
                /* do nothing */
            }
            /* wait for error notification from ESM, or for timeout */
            /* fault injection gets deasserted in ISR */
            while((mssSecFlag!=TRUE) && (timeout!=0U))
            {
                timeout--;
            }
            if(mssSecFlag==TRUE)
            {
                mssSecFlag=FALSE;
                ret_val = SDL_PASS;
            }
            else
            {
                ret_val = SDL_EFAIL;
            }
        }

    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

/********************************************************************************************************
*   For DED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
                                                                     uint32_t busSftyNode, uint32_t addr)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;
    switch(busSftyNode)
    {
        /* esm init for MSS_TPTC_A0_RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction , NULL);
            break;
        }
        /* esm init for MSS_TPTC_A1_RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for MSS_TPTC_A0_WR */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for MSS_TPTC_A1_WR */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for MSS_MCRC */
        case SDL_ECC_BUS_SAFETY_MSS_MCRC:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
#if defined(SOC_AM263X)
          /* esm init for MSS_QSPI */
        case SDL_ECC_BUS_SAFETY_MSS_QSPI:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
#elif defined(SOC_AM261X)
          /* esm init for MSS_OSPI */
        case SDL_ECC_BUS_SAFETY_MSS_OSPI:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_OSPI_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
          /* esm init for MSS_USBSS_RD */
        case SDL_ECC_BUS_SAFETY_MSS_USBSS_RD:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
          /* esm init for MSS_USBSS_WR */
        case SDL_ECC_BUS_SAFETY_MSS_USBSS_WR:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
#endif
          /* esm init for MSS_MBOX */
        case SDL_ECC_BUS_SAFETY_MSS_MBOX:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
          /* esm init for MSS_STM_STIM */
        case SDL_ECC_BUS_SAFETY_MSS_STM_STIM:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
          /* esm init for MSS_SCRP0 */
        case SDL_ECC_BUS_SAFETY_MSS_SCRP0:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for MSS_SCRP1 */
        case SDL_ECC_BUS_SAFETY_MSS_SCRP1:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
#if defined (SOC_AM261X)
        /* esm init for ICSSM0_PDSP0 */
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for ICSSM_PDSP1 */
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for ICSSM_S */
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for ICSSM1_PDSP0 */
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for ICSSM_PDSP1 */
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for ICSSM_S */
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
#else
        /* esm init for ICSSM_PDSP0 */
        case SDL_ECC_BUS_SAFETY_ICSSM_PDSP0:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for ICSSM_PDSP1 */
        case SDL_ECC_BUS_SAFETY_ICSSM_PDSP1:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for ICSSM_S */
        case SDL_ECC_BUS_SAFETY_ICSSM_S:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_ICSSM_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
#endif
        /* esm init for DAP */
        case SDL_ECC_BUS_SAFETY_DAP:
        {
           ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_DAP_DED_RED_ESM_ApplicationCallbackFunction, NULL);
           break;
        }
        /* esm init for MSS_MMC */
        case SDL_ECC_BUS_SAFETY_MSS_MMC:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_MMC_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        #if defined(SOC_AM261X)
        /* esm init for MSS_GPMC */
        case SDL_ECC_BUS_SAFETY_MSS_GPMC:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        #endif
        /* esm init for MSS_CPSW */
        case SDL_ECC_BUS_SAFETY_MSS_CPSW:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_L2_A */
        case SDL_ECC_BUS_SAFETY_MSS_L2_A:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_L2_B */
        case SDL_ECC_BUS_SAFETY_MSS_L2_B:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_L2_C */
        case SDL_ECC_BUS_SAFETY_MSS_L2_C:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_C_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5A_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5B_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5A_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5B_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5A_AXI_S */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5B_AXI_S */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }

#if defined (SOC_AM263X) || defined (SOC_AM263PX)
        /* esm init for MSS_L2_D */
        case SDL_ECC_BUS_SAFETY_MSS_L2_D:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_D_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5C_AXI_S */
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5D_AXI_S */
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5C_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5D_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5C_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for CR5D_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
#if defined (SOC_AM263PX)
        /* esm init for MSS_L2_E */
        case SDL_ECC_BUS_SAFETY_MSS_L2_E:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_E_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_L2_F */
        case SDL_ECC_BUS_SAFETY_MSS_L2_F:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_F_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        default :
        break;
    }
    if (ret_val == SDL_PASS)
    {
        if(SDL_ECC_BUS_SAFETY_MSS_isL2OCRAMNode(busSftyNode))
        {
            CacheP_disable(CacheP_TYPE_L1D);
        }
        ret_val = SDL_ECC_BUS_SAFETY_MSS_dedExecute(busSftyNode,addr,writeData);
        if(SDL_ECC_BUS_SAFETY_MSS_isL2OCRAMNode(busSftyNode))
        {
            CacheP_enable(CacheP_TYPE_L1D);
        }
        if(ret_val !=SDL_PASS )
        {
           ret_val = SDL_EFAIL;
           DebugP_log("\n ECC_BUS_SAFETY_MSS_dedExecuted Failed for node %d \r\n", busSftyNode);
        }
        else
        {
            #if defined (SOC_AM263X) || defined (SOC_AM263PX)
            if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR>=busSftyNode)) ||
            ((SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD>=busSftyNode)) || 
            (busSftyNode == SDL_ECC_BUS_SAFETY_DAP))
            #endif
            #if defined (SOC_AM261X) 
            if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD>=busSftyNode)) ||
            ((SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD>=busSftyNode)) || 
            (busSftyNode == SDL_ECC_BUS_SAFETY_DAP))
            #endif
            {
                SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
                test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
            }
            else if (SDL_ECC_BUS_SAFETY_MSS_CPSW ==busSftyNode  )
            {
                /* CPSW SetUP */
                setup_CPSW();
                /* CPSW Transfer */
                cpsw_transfer_ded();
            }
            else
            {
                /* do nothing */
            }
            /* wait for error notification from ESM, or for timeout */
            /* fault injection gets deasserted in ISR */
            while((SDL_MSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
            {
                timeout--;
            }
            if(SDL_MSS_intrFlg[busSftyNode]==TRUE)
            {
                SDL_MSS_intrFlg[busSftyNode]=FALSE;
                ret_val = SDL_PASS;
            }
            else
            {
                ret_val = SDL_EFAIL;
            }
        }
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}
/********************************************************************************************************
*   For RED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
            uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    switch(busSftyNode)
    {
        /* esm init for MSS_CR5A_AHB */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5B_AHB */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
        /* esm init for MSS_CR5C_AHB */
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5D_AHB */
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        /* esm init for MSS_TPTC_A0_RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_TPTC_A1_RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_TPTC_A0_WR */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_TPTC_A1_WR */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5A_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5B_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
        /* esm init for MSS_CR5C_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5D_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        /* esm init for MSS_CR5A_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5B_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
        /* esm init for MSS_CR5C_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5D_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        /* esm init for MSS_CR5A_AXI_S */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5B_AXI_S */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
        /* esm init for MSS_CR5C_AXI_S */
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CR5D_AXI_S */
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        case SDL_ECC_BUS_SAFETY_MSS_MCRC:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#if defined(SOC_AM263X)
        case SDL_ECC_BUS_SAFETY_MSS_QSPI:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#elif defined(SOC_AM263PX) || defined(SOC_AM261X)
        case SDL_ECC_BUS_SAFETY_MSS_OSPI:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_OSPI_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
#if defined(SOC_AM261X)
        case SDL_ECC_BUS_SAFETY_MSS_USBSS_RD:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_USBSS_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_USBSS_WR:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_USBSS_WR_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        case SDL_ECC_BUS_SAFETY_MSS_MBOX:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_STM_STIM:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_SCRP0:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_SCRP1:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#if defined (SOC_AM261X)
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP0_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM0_PDSP1_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM0_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP0_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM1_PDSP1_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_MSS_ICSSM1_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#else
        case SDL_ECC_BUS_SAFETY_ICSSM_PDSP0:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_ICSSM_PDSP1:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        case SDL_ECC_BUS_SAFETY_ICSSM_S:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_ICSSM_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        case SDL_ECC_BUS_SAFETY_DAP:
        {
            ret_val = SDL_ESM_init(esmInstType, params, SDL_ECC_BUS_SAFETY_DAP_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_MMC */
        case SDL_ECC_BUS_SAFETY_MSS_MMC:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_MMC_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        #if defined(SOC_AM261X)
        /* esm init for MSS_GPMC */
        case SDL_ECC_BUS_SAFETY_MSS_GPMC:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        #endif
        /* esm init for MSS_MAIN_VBUSP */
        case SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_PERI_VBUSP */
        case SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_CPSW */
        case SDL_ECC_BUS_SAFETY_MSS_CPSW:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_L2_A */
        case SDL_ECC_BUS_SAFETY_MSS_L2_A:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_L2_B */
        case SDL_ECC_BUS_SAFETY_MSS_L2_B:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_L2_C */
        case SDL_ECC_BUS_SAFETY_MSS_L2_C:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_C_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#if defined (SOC_AM263X) || defined (SOC_AM263PX)
        /* esm init for MSS_L2_D */
        case SDL_ECC_BUS_SAFETY_MSS_L2_D:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_D_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
#if defined (SOC_AM263PX)
        /* esm init for MSS_L2_E */
        case SDL_ECC_BUS_SAFETY_MSS_L2_E:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_E_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
        /* esm init for MSS_L2_F */
        case SDL_ECC_BUS_SAFETY_MSS_L2_F:
        {
            ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_L2_F_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
        }
#endif
        default :
        break;
    }
    if (ret_val == SDL_PASS)
    {
        ret_val= SDL_ECC_BUS_SAFETY_MSS_redExecute(busSftyNode, fiType, redType);
        if(ret_val !=SDL_PASS )
        {
            DebugP_log("\n ECC_BUS_SAFETY_MSS_redExecuted Failed for node %d \r\n", busSftyNode);
            ret_val = SDL_EFAIL;
        }
        else
        {
            /* wait for error notification from ESM, or for timeout */
            /* fault injection gets deasserted in ISR */
            while((SDL_MSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
            {
                timeout--;
            }
            if(SDL_MSS_intrFlg[busSftyNode]==TRUE)
            {
                SDL_MSS_intrFlg[busSftyNode]=FALSE;
                ret_val = SDL_PASS;
            }
            else
            {
                ret_val = SDL_EFAIL;
            }
        }
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}
#endif


#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   ESM Callback Function for SEC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg)
{
    int32_t retVal = SDL_PASS;
    /*  DSS_L3_BANKA */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA );
    /*  DSS_L3_BANKB */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKB );
    /*  DSS_L3_BANKC */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKC );
    /*  DSS_L3_BANKD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKD );
    /*  DSS_HWA_DMA0 */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0 );
    /*  DSS_HWA_DMA1 */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1 );
    /*  DSS_MBOX */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_MBOX );
    /*  DSS_MCRC */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_MCRC );
    /*  DSS_CBUFF_FIFO */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO );
    /*  DSS_TPTC_A0_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD );
    /*  DSS_TPTC_A1_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD );
    /*  DSS_TPTC_B0_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD );
    /*  DSS_TPTC_B1_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD );
    /*  DSS_TPTC_C0_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD );
    /*  DSS_TPTC_C1_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD );
    /*  DSS_TPTC_C2_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD );
    /*  DSS_TPTC_C3_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD );
    /*  DSS_TPTC_C4_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD );
    /*  DSS_TPTC_C5_RD */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD );
    /*  DSS_PCR */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_PCR );
    /*  _DSS_DSP_MDMA */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA );
    /*  _DSS_DSP_SDMA */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA );
#if defined (SOC_AM273X)
    /*  DSS_MDO_FIFO */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO );
#endif
    dssSecFlag = TRUE;
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_DSP_SDMA ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_DSP_MDMA ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_PCR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_PCR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_PCR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_PCR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_PCR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_L3_BANKA ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_L3_BANKA] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_L3_BANKA] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_L3_BANKB ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKB ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKB )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_L3_BANKB] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_L3_BANKB] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_L3_BANKC ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKC ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKC )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_L3_BANKC] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_L3_BANKC] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_L3_BANKD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_L3_BANKD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_L3_BANKD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_L3_BANKD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_HWA_DMA0 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0 )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_HWA_DMA1 ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1 ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1 )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_MBOX ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_MBOX ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_MBOX )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_MBOX] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_MBOX] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_MCRC ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_MCRC ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_MCRC )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_MCRC] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_MCRC] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_CBUFF_FIFO ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_A0_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_A1_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_B0_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_B1_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C0_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C1_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C2_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C3_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C4_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C5_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_A0_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_A1_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_B0_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_B1_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C0_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C1_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C2_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR] = FALSE;
    }
    return retVal;
}


/********************************************************************************************************
*   For Node DSS_TPTC_C3_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node DSS_TPTC_C4_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR] = FALSE;
    }
    return retVal;
}


/********************************************************************************************************
*   For Node DSS_TPTC_C5_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR] = FALSE;
    }
    return retVal;
}
#endif
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   For Node DSS_DSP_SDMA
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA,SDL_DSS_L2_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA,SDL_DSS_L2_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_DSP_MDMA
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA,SDL_DSS_L3_BANKA_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA,SDL_DSS_MAILBOX_U_BASE+4));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_PCR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_PCR,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_PCR,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_PCR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_L3_BANKA
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_DSS_L3_BANKA_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,SDL_DSS_L3_BANKA_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_L3_BANKA, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_L3_BANKB
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_L3_BANKB,SDL_DSS_L3_BANKB_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_L3_BANKB,SDL_DSS_L3_BANKB_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_L3_BANKB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_L3_BANKC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_L3_BANKC,SDL_DSS_L3_BANKC_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_L3_BANKC,SDL_DSS_L3_BANKC_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_L3_BANKC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_L3_BANKD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_L3_BANKD,SDL_DSS_L3_BANKD_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_L3_BANKD,SDL_DSS_L3_BANKD_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_L3_BANKD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_MBOX
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_MBOX,SDL_DSS_MAILBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_MBOX,SDL_DSS_MAILBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_MBOX, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_MCRC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_MCRC,SDL_DSS_MCRC_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_MCRC,SDL_DSS_MCRC_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_MCRC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_HWA_DMA0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0,SDL_DSS_HWA_DMA0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0,SDL_DSS_HWA_DMA0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_HWA_DMA1
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1,SDL_DSS_HWA_DMA1_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                           SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1,SDL_DSS_HWA_DMA1_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_CBUFF_FIFO
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO,SDL_DSS_CBUFF_FIFO_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO,SDL_DSS_CBUFF_FIFO_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_A0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_A1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD,0U));
}
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_TPTC_B0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD,0U));
}


int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_TPTC_B1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                         SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD ,SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_SEC_Test(void)
{
     return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                         SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_TPTC_C1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_SEC_Test(void)
{
     return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                         SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                         SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_TPTC_C2_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                         SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                                      SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_TPTC_C3_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                         SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                        SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C4_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                          SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                          SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_TPTC_C5_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(\
                                          SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(\
                                                      SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_A0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_A1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_B0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_B1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node DSS_TPTC_C0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node DSS_TPTC_C1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node DSS_TPTC_C2_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C3_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node DSS_TPTC_C4_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C5_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif
#endif

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   For Node DSS MDO FIF0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO,SDL_DSS_MDO_FIFO_U_BASE));
}
#endif
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   For SEC
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_test(uint32_t busSftyNode ,uint32_t addr)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;
    /* sec test */
    ret_val = SDL_ECC_BUS_SAFETY_DSS_secExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))
         ||(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode))

        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode, dmaCh,tcc,param,queNum,edmaNum );
        }
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((dssSecFlag!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(dssSecFlag==TRUE)
        {
            dssSecFlag=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    return ret_val;
}

/********************************************************************************************************
*   For DED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_test(uint32_t busSftyNode ,uint32_t addr)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;
    /* ded test */
    ret_val = SDL_ECC_BUS_SAFETY_DSS_dedExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))
         ||(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode))

        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((SDL_DSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(SDL_DSS_intrFlg[busSftyNode]==TRUE)
        {
            SDL_DSS_intrFlg[busSftyNode]=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    return ret_val;
}
/********************************************************************************************************
*   For RED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_test(uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    /* red test */
    ret_val= SDL_ECC_BUS_SAFETY_DSS_redExecute(busSftyNode, fiType, redType);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((SDL_DSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(SDL_DSS_intrFlg[busSftyNode]==TRUE)
        {
            SDL_DSS_intrFlg[busSftyNode]=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    return ret_val;
}

#endif
#endif

/* Nothing past this point */
