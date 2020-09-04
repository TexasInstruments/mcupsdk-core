/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file sdl_ecc_bus_safety.c
 *
 *  \brief Common across test-cases using ECC BUS
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "ecc_bus_safety_main.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)

volatile bool SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO+1];

#endif
#endif

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

volatile bool SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR+1];

#endif
#endif

#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)

volatile bool dssSecFlag = FALSE;

#endif
#endif

#if defined (SUBSYS_MSS)
volatile bool mssSecFlag = FALSE;
volatile bool SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_MBOX+1];
#endif
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(uint32_t busSftyNode ,uint32_t addr);
static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(uint32_t busSftyNode, \
                        SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);
#endif
#endif

#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)

static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);

static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_test(uint32_t busSftyNode ,uint32_t addr);

static int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_test(uint32_t busSftyNode ,uint32_t addr);

int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                             int32_t grpChannel,
                                                                             int32_t vecNum,
                                                                             void *arg);

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg);
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg);
#endif
#endif

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
int32_t SDL_ECC_BUS_SAFETY_RSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);
int32_t SDL_ECC_BUS_SAFETY_RSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

#endif
#endif

#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_test(uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);

static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_test(uint32_t busSftyNode ,uint32_t addr);

static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_test(uint32_t busSftyNode ,uint32_t addr);
#elif defined (SOC_AM263X)
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
            uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);

static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
            uint32_t busSftyNode, uint32_t addr);

static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_test(const SDL_ESM_Inst esmInstType,SDL_ESM_config* params,\
            uint32_t busSftyNode, uint32_t addr);

#endif
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,int32_t grpChannel,int32_t vecNum,void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                            int32_t grpChannel,
                                                                            int32_t vecNum,
                                                                            void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                        int32_t grpChannel,
                                                                        int32_t vecNum,
                                                                        void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                          int32_t grpChannel,
                                                                           int32_t vecNum,
                                                                          void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                           int32_t grpChannel,
                                                                          int32_t vecNum,
                                                                        void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                            int32_t grpChannel,
                                                                            int32_t vecNum,
                                                                            void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                          int32_t grpChannel,
                                                                          int32_t vecNum,
                                                                          void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                  int32_t grpChannel,
                                                                  int32_t vecNum,
                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                              int32_t grpChannel,
                                                              int32_t vecNum,
                                                              void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);

#endif

#if defined (SOC_AM263X)

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);


int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                        uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                        uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);


int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg);


#endif


#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
int32_t SDL_ECC_BUS_SAFETY_RSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                            int32_t grpChannel,
                                                                            int32_t vecNum,
                                                                            void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                        int32_t grpChannel,
                                                                        int32_t vecNum,
                                                                        void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                                  int32_t grpChannel,
                                                                                  int32_t vecNum,
                                                                                  void *arg);
#endif
#endif

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
/* Event BitMap for ECC ESM callback for DSS */
SDL_ESM_NotifyParams ECC_BUS_SAFETY_TestparamsDSS[37] =
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
      {
           /* Event BitMap for ECC ESM callback for DED and RED for RSS MBOX*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_RSS_BUS_SAFETY_MCASPC,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ECC_BUS_SAFETY_RSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction,
      },
      {
           /* Event BitMap for ECC ESM callback for RED for RSS ADCBUF RD*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_RSS_BUS_SAFETY_TPTCA1_RD,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD_RED_ESM_ApplicationCallbackFunction,
      },
      {
           /* Event BitMap for ECC ESM callback for DED and RED for RSS ADCBUF WR*/
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_RSS_BUS_SAFETY_TPTCA1_WR,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_DED_RED_ESM_ApplicationCallbackFunction,
      },
       {
           /* Event BitMap for ECC ESM callback for RSS SEC */
           .groupNumber = SDL_INTR_GROUP_NUM,
           .errorNumber = SDL_DSS_ESMG1_RSS_BUS_SAFETY_SEC_AGG,
           .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_LOW,
           .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
           .callBackFunction = &SDL_ECC_BUS_SAFETY_RSS_SEC_ESM_ApplicationCallbackFunction,
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
#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
/* Event BitMap for ECC ESM callback for MSS */
SDL_ESM_NotifyParams ECC_BUS_SAFETY_TestparamsMSS[16] =
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
};

#endif
#if defined (SOC_AM263X)
 SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_MAIN =
{
       .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x80000000u, 0x00000000u, 0x00000000, 0x00000000u,
                },
     /**< Only AHB events enable:**/
      /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0x80000000u, 0x00000000u, 0x00000000, 0x00000000u,
                        },
    /**< AHB events high priority:**/
    .errorpinBitmap = {0x80000000u, 0x00000000u, 0x00000000, 0x00000000u,
                      },
    /**< All events high priority:**/
};

SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN =
{
      .esmErrorConfig = {1u, 8u}, /* Self test error config */
   .enableBitmap = {0x00000000u, 0x00000002u, 0x00000000, 0x00000000u,
               },
    /**< Only TPTC events enable:**/
     /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
   .priorityBitmap = {0x00000000u, 0x00000000u, 0x00000000, 0x00000000u,
                       },
   /**< TPTC events high priority:**/
   .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00000000, 0x00000000u,
                     },
   /**< All events high priority:**/
};

SDL_ESM_config ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN =
{
      .esmErrorConfig = {1u, 8u}, /* Self test error config */
   .enableBitmap = {0x00000000u, 0x00000002u, 0x00000000, 0x00000000u,
               },
    /**< Only TPTC events enable:**/
     /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
   .priorityBitmap = {0x00000000u, 0x0000002u, 0x00000000, 0x00000000u,
                       },
   /**< TPTC events high priority:**/
   .errorpinBitmap = {0x00000000u, 0x00000002u, 0x00000000, 0x00000000u,
                     },
   /**< All events high priority:**/
};


#endif
#endif
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
SDL_ESM_OpenParams gEsmOpenParams=
{
    TRUE
    /* boolean value to indicate if old ESM pending errors should be cleared or not
          This field will be set by SysCfg.*/
};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/** \brief Defines the various ECC BUS SAFETY test cases. */

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
/********************************************************************************************************
*   For Node RSS_MBOX ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_MBOX_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_RSS_MBOX ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_RSS_MBOX )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_RSS_MBOX] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_RSS_MBOX] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node RSS_ADCBUF_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   For Node RSS_ADCBUF_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                               int32_t grpChannel,
                                                                               int32_t vecNum,
                                                                               void *arg)
{
    int32_t retVal = SDL_PASS;
    if ((SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD ))&&
    (SDL_PASS == SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD )))
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD] = TRUE;
    }
    else
    {
        SDL_DSS_intrFlg[SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD] = FALSE;
    }
    return retVal;
}

/********************************************************************************************************
*   ESM Callback Function for SEC in RSS
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst instance,
                                                                int32_t grpChannel,
                                                                int32_t vecNum,
                                                                void *arg)
{
    int32_t retVal = SDL_PASS;
    /*  DSS_RSS */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_RSS_MBOX );
    /*  RSS_ADCBUF_WR */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR );
    dssSecFlag = TRUE;
    return retVal;
}

#endif
#endif

#if defined (SOC_AM273X) ||  (SOC_AWR294X)
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
#if defined (SOC_AWR294X)
    /*  DSS_RSS */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_RSS_MBOX );
    /*  RSS_ADCBUF_WR */
    SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR );
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
#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
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
    /*  MSS_AXI_CR5A_RD */
   SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD );

   /*  MSS_AXI_CR5B_RD */
  SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD );

  /*  MSS_AXI_CR5A_S */
 SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S );

 /*  MSS_AXI_CR5B_S */
SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S );

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

#elif defined (SOC_AM263X)
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
    SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
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
    SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
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
    SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
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
    SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   ESM Callback Function for SEC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
     /*  MSS_TPTC_A0_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD );
     /*  MSS_TPTC_A1_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD );
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX);
    mssSecFlag = TRUE;
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
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
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
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
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
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
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
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
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
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
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR] = FALSE;
    }
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
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
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5C_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR] = FALSE;
    }
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR] = FALSE;
    }
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}
/********************************************************************************************************
*   For Node MSS_CR5A_AXI_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
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
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
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
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD] = FALSE;
    }
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

/********************************************************************************************************
*   For Node MSS_CR5C_AXI_RD ESM Callback Function
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD] = FALSE;
    }
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
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
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD ))
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD] = TRUE;
    }
    else
    {
    SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD] = FALSE;
    }
    /* Clear ESM registers. */
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

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
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
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
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

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
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
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
   SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
   SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
    return retVal;
}

#endif
#endif

#if defined (SOC_AM273X) ||  (SOC_AWR294X)
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
                                        SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA,SDL_DSS_MAILBOX_U_BASE+4));
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

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_1_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_L3_BANKA, SDL_ECC_BUS_SAFETY_FI_GLOBAL_MAIN, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_2_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(\
            SDL_ECC_BUS_SAFETY_DSS_L3_BANKA, SDL_ECC_BUS_SAFETY_FI_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_3_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test( \
            SDL_ECC_BUS_SAFETY_DSS_L3_BANKA, SDL_ECC_BUS_SAFETY_FI_MAIN, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
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

#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
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

#elif (SOC_AM263X)
/********************************************************************************************************
*   For Node MSS_CR5A_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5B_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, &ECC_Bus_Safety_Test_esmInitConfig_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5C_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5D_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, &ECC_Bus_Safety_Test_esmInitConfig_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U));
}
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5C_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5C_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5C_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,&ECC_Bus_Safety_Test_esmInitConfig_AXI_MAIN, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
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

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
/********************************************************************************************************
*   For Node RSS_MBOX
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_MBOX_SEC_Test(void)
{
     return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(SDL_ECC_BUS_SAFETY_RSS_MBOX,SDL_RSS_CR4_MBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_RSS_MBOX_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(SDL_ECC_BUS_SAFETY_RSS_MBOX,SDL_RSS_CR4_MBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_RSS_MBOX_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(SDL_ECC_BUS_SAFETY_RSS_MBOX, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node RSS_ADCBUF_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node RSS_ADCBUF_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_SEC_Test(void)
{
     return (SDL_ECC_BUS_SAFETY_DSS_SEC_test(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR,SDL_RSS_ADCBUF_WRITE_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_test(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR,SDL_RSS_ADCBUF_WRITE_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_RED_test(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#endif
#endif

#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
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
            if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode))
            {
            SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
            }
            else
            {
                 /* Do Nothing */
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
            if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode))
            {
            SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
            }
            else
            {
                 /* Do Nothing */
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
#if defined (SOC_AM263X)
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

    ret_val = SDL_ESM_init(esmInstType, params, &SDL_ECC_BUS_SAFETY_MSS_SEC_ESM_ApplicationCallbackFunction, NULL);
    if (ret_val == SDL_PASS)
    {
        ret_val = SDL_ECC_BUS_SAFETY_MSS_secExecute(busSftyNode,addr,writeData);
        if(ret_val !=SDL_PASS )
        {
            ret_val = SDL_EFAIL;
        }
        else
        {
            if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD>=busSftyNode))
            {
            SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
            }
            else
            {
                 /* Do Nothing */
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
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD:
            {  ret_val = SDL_ESM_init(esmInstType, &ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN, SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction , NULL);
            break;
            }
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD:
            { ret_val = SDL_ESM_init(esmInstType, &ECC_Bus_Safety_Test_esmInitConfig_TPTC_MAIN, SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
            }
        default :
           break;
    }
    if (ret_val == SDL_PASS)
    {
        ret_val = SDL_ECC_BUS_SAFETY_MSS_dedExecute(busSftyNode,addr,writeData);
        if(ret_val !=SDL_PASS )
        {
           ret_val = SDL_EFAIL;
        }
        else
        {
            if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD>=busSftyNode))
            {
            SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
            }
            else
            {
                 /* Do Nothing */
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
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB:
            { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
            }
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB:
            { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
            }
        case SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB:
            { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
            }
        case SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB:
            { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
            }
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD:
            { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
            }
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD:
            { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
            break;
            }
      case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
      case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S:
          { ret_val = SDL_ESM_init(esmInstType, params,SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_RED_ESM_ApplicationCallbackFunction, NULL);
          break;
          }
        default :
           break;
    }
    if (ret_val == SDL_PASS)
    {
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
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}
#endif
void SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(uint32_t busSftyNode , uint32_t *dmaCh, uint32_t *tcc,uint32_t *param, uint32_t *queNum, uint32_t *edmaNum )
{
    switch (busSftyNode)
    {
      #if defined (SOC_AM273X) || defined (SOC_AWR294X)
        /* MSS TPTC A0 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD :
        {
            *dmaCh = EDMA_MSS_TPCC_A_EVT_FREE_0;
            *tcc =   EDMA_MSS_TPCC_A_EVT_FREE_0;
            *param = EDMA_MSS_TPCC_A_EVT_FREE_0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* MSS TPTC A1 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD :
        {
            *dmaCh = EDMA_MSS_TPCC_A_EVT_FREE_0;
            *tcc =   EDMA_MSS_TPCC_A_EVT_FREE_0;
            *param = EDMA_MSS_TPCC_A_EVT_FREE_0;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* MSS TPTC B0 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD :
        {
            *dmaCh = EDMA_MSS_TPCC_B_EVT_FREE_0;
            *tcc =   EDMA_MSS_TPCC_B_EVT_FREE_0;
            *param = EDMA_MSS_TPCC_B_EVT_FREE_0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA1;
            break;
        }

        #endif
        #if defined (SOC_AM263X)
        /* MSS TPTC A0 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD :
        {
            *dmaCh = EDMA_RESOURCE_ALLOC_ANY;
            *tcc =   EDMA_RESOURCE_ALLOC_ANY;
            *param = EDMA_RESOURCE_ALLOC_ANY;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* MSS TPTC A1 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD :
        {
            *dmaCh = EDMA_RESOURCE_ALLOC_ANY;
            *tcc =   EDMA_RESOURCE_ALLOC_ANY;
            *param = EDMA_RESOURCE_ALLOC_ANY;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        #endif
        default :
        {
            *dmaCh = EDMA_RESOURCE_ALLOC_ANY;
            *tcc =   EDMA_RESOURCE_ALLOC_ANY;
            *param = EDMA_RESOURCE_ALLOC_ANY;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
    }
}

#endif

#if defined (SOC_AM273X) ||  (SOC_AWR294X)
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
    ret_val = SDL_ECC_BUS_SAFETY_DSS_secExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))||(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode, dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
            /* Do Nothing */
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
    ret_val = SDL_ECC_BUS_SAFETY_DSS_dedExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))||(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
            /* Do Nothing */
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

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   For Node DSS_MDO_FIFO
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO ,SDL_DSS_MDO_FIFO_U_BASE));
}

/********************************************************************************************************
*   For DSS DED
*********************************************************************************************************/

static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(uint32_t busSftyNode ,uint32_t addr)
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t status = 0U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;

    retval = SDL_ECC_BUS_SAFETY_DSS_dedExecute(busSftyNode,addr,writeData);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        if((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
            /* Do Nothing */
        }
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(busSftyNode);
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}
/********************************************************************************************************
*   For DSS RED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType)
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t status = 0U;
    retval= SDL_ECC_BUS_SAFETY_DSS_redExecute(busSftyNode, fiType, redType);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_DSS_redErrorClear(busSftyNode);
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}
#endif
#endif
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)
/********************************************************************************************************
*   Function to select the parameters for EDMA config according to Nodes
*********************************************************************************************************/

void SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(uint32_t busSftyNode , uint32_t *dmaCh, uint32_t *tcc,uint32_t *param, uint32_t *queNum, uint32_t *edmaNum )
{
    switch (busSftyNode)
    {
        /* DSS TPTC A0 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* DSS TPTC A1 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* DSS TPTC B0 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA1;
            break;
        }
        /* DSS TPTC B1 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA1;
            break;
        }
        /* DSS TPTC C0 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C1 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C2 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 2U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C3 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 3U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C4 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 4U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C5 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 5U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS PCR */
        case SDL_ECC_BUS_SAFETY_DSS_PCR :
        {
            *dmaCh = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        default :
        {
            *dmaCh = EDMA_RESOURCE_ALLOC_ANY;
            *tcc =   EDMA_RESOURCE_ALLOC_ANY;
            *param = EDMA_RESOURCE_ALLOC_ANY;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
    }
}

#endif
#endif
