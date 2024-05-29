/*
 *   Copyright (c) 2024 Texas Instruments Incorporated
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

#ifndef SDLEXAMPLE_H
#define SDLEXAMPLE_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <sdl/include/am263x/sdlr_intr_r5fss0_core0.h>
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#include <sdl/esm/sdlr_esm.h>
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/esm/v0/v0_0/sdl_ip_esm.h>



/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                 External Function Declarations                             */
/* ========================================================================== */

/***************************************************************************/
/*              General Defines                                            */
/***************************************************************************/
/* start up diagnostic error codes        */
#define STARTUPDIAGS_OK       0u
#define STARTUPDIAGS_CCMERR   1u
#define STARTUPDIAGS_PBISTERR 2u
#define STARTUPDIAGS_DPLERR   4u
#define STARTUPDIAGS_ESMERR   8u
#define STARTUPDIAGS_STCERR   16u

/* Used to denote what the ESM callback was for  */
#define ESMCB_NONE            0u
#define ESMCB_UNKNOWN         1u
#define ESMCB_CCM             2u
#define ESMCB_RTI             3u
#define ESMCB_ECC             4u
#define ESMCB_DCC             5u
#define ESMCB_PARITYTCM       6u
#define ESMCB_PARITYDMA       7u
#define ESMCB_ECCBUS          8u

/* used to denote Interrupt Source in the ESM */
#define ESM_INT_MCAN0_ECC_CORRECTABLE       2u
#define ESM_INT_MCAN0_ECC_NOT_CORRECTABLE   3u
#define ESM_INT_TCMADDR_R500                14u
#define ESM_INT_TCMADDR_R501                15u
#define ESM_INT_TCMADDR_R510                16u
#define ESM_INT_TCMADDR_R511                17u
#define ESM_INT_MSSL2_ECC_CORRECTABLE       19u
#define ESM_INT_MSSL2_ECC_UNCORRECTABLE     20u
#define ESM_INT_DCC0                        21u
#define ESM_INT_DCC1                        22u
#define ESM_INT_DCC2                        23u
#define ESM_INT_DCC3                        24u
#define ESM_INT_BUSSAFETY1                  31u
#define ESM_INT_BUSSAFETY2                  33u
#define ESM_INT_R5F0_ECC_CORRECTABLE        47u
#define ESM_INT_R5F0_ECC_UNCORRECTABLE      48u
#define ESM_INT_EDMA0_TPCC_ERRINTAGG        63u
#define ESM_INT_RTI                         64u   /* Pulse interrupt Int 0 in TRM */
#define ESM_INT_EDMA0_TPCC_ERINT            68u
#define ESM_INT_ICSSM_ECC_UNCORRECTABLE     77u
#define ESM_INT_ICSSM_ECC_CORRECTABLE       78u
#define ESM_INT_CCM0_SELFTEST               83u
#define ESM_INT_CCM0_LOCKSTEP               84u
#define ESM_INT_CCM1_SELFTEST               85u
#define ESM_INT_CCM1_LOCKSTEP               86u




/* denotes which diag is running */
#define DIAGRUNNING_NONE      0u
#define DIAGRUNNING_ECCMCAN0  1u
#define DIAGRUNNING_RTI       2u
#define DIAGRUNNING_DCC       3u
#define DIAGRUNNING_TCMPARITY 4u
#define DIAGRUNNING_MCRC      5u
#define DIAGRUNNING_ECCBUS    6u
#define DIAGRUNNING_CCM       7u
#define DIAGRUNNING_DMAPARITY 8u
#define DIAGRUNNING_ECCICSSM  9u
#define DIAGRUNNING_ECCMSSL2  10u
#define DIAGRUNNING_ECCATCM   11u

/* SDL related stats and counters we use  */
struct s_sdlstats
{
  uint32_t  runDiagnostics;     /* Set to true by timer if we need to run diagnostics   */
  uint32_t  diagRunning;        /* Used to distinguish which diag is running            */
  int32_t   stcResult[2];       /* Holds stc results for each R0 R5 core                */
  uint32_t  startupErrCode;     /* Bitmap flag holding startup errors or OK             */
  uint32_t  eccCorrectedCount;  /* Number of correctable ECC errors                     */
  uint32_t  esmcb;              /* What trigerred the esm callback                      */
  struct    /* esm error info */
  {
    SDL_ESM_Inst        esmInst;
    SDL_ESM_IntType     esmIntrType;
    uint32_t            grpChannel;
    uint32_t            index;
    uint32_t            intSrc;
  } esm;
  struct    /* RTI data */
  {
    uint32_t            isrFlag;      /* Flag used to indicate occurrence of the error interrupt */
  } rti;
  struct    /* DCC data */
  {
    uint32_t            isrFlag;      /* Flag used to indicate occurrence of the error interrupt */
    uint32_t            doneIsrFlag;  /* Flag used to indecate occurrence of the completion interrupt */
  } dcc;
  struct    /* holds ecc data if there was an error */
  {
    SDL_ECC_MemType     eccMemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
  } ecc;
  struct    /* TCM status register values */
  {
    uint32_t   statusRegValue0;  /* R5F0-0 */
    uint32_t   statusRegValue1;  /* R5F0-1 */
    uint32_t   statusRegValue2;  /* R5F1-0 */
    uint32_t   statusRegValue3;  /* R5F1-1 */
  } tcm;
};

extern volatile struct s_sdlstats  sdlstats;

/***************************************************************************/
/*              Diagnostic functions                                       */
/***************************************************************************/
/* Performs runtime diagnostis */
extern int32_t performDiags(void *args);
/* returns true if running diagnsotics or false if not */
extern bool runningDiags(void);

/***************************************************************************/
/*              STC Defines                                                */
/***************************************************************************/
extern void STC_run(void *args);

/***************************************************************************/
/*              PBIST Defines                                                */
/***************************************************************************/
extern int32_t pbist_run(void *args);

/***************************************************************************/
/*              DPL Defines                                                */
/***************************************************************************/
extern int32_t SDL_EXAMPLE_dplInit(void);

/***************************************************************************/
/*              CCM Defines                                                */
/***************************************************************************/
extern int32_t CCM_test(void *args);
extern void ccm_clearESM(uint32_t intSrc);
extern int32_t CCM_funcTest(void);

/***************************************************************************/
/*              ECC Defines                                                */
/***************************************************************************/

/* ECC MCAN MEMORY */
extern int32_t ECC_test(void);
extern void ecc_clearESM(void);
/* ECC ICSSM MEMORY */
extern int32_t ECC_ICSSM_test(void);
extern void ecc_icssm_clearESM(void);
/* ECC MSSL2 MEMORY */
extern int32_t ECC_MSSL2_test(void);
extern void ecc_mssl2_clearESM(void);
/* ECC R5F0 ATCM MEMORY */
extern int32_t ECC_R5F0_ATCM_test(void);
extern void ecc_r5f0_clearESM(SDL_ESM_IntType esmIntrType);

/***************************************************************************/
/*              DCC Defines                                                */
/***************************************************************************/
extern int32_t DCC_test (void);
extern void dcc_clearESM(void);

/***************************************************************************/
/*              MCRC Defines                                               */
/***************************************************************************/
extern int32_t MCRCAuto_test (void);
extern int32_t mcrcSemiCPU_test (void);
extern int32_t sdl_mcrc_full_cpu_test (void);

/***************************************************************************/
/*              ECC Bus Safety Defines                                     */
/***************************************************************************/
extern int32_t ecc_bus_safety_testExecute(void);
extern int32_t EccBusSafety_clearESM(void);

/***************************************************************************/
/*              Parity Defines                                             */
/***************************************************************************/
extern int32_t ParityTCM_test(void);
extern int32_t ParityTCM_clear(void);

extern int32_t ParityDMA_test(void);
extern int32_t ParityDMA_clear(void);

/***************************************************************************/
/*              RTI Defines                                             */
/***************************************************************************/
extern void rti_clearESM(uint32_t intSrc);

/***************************************************************************/
/*              ESM Defines                                                */
/***************************************************************************/

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                                   SDL_ESM_IntType esmIntrType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);

/* Sets up ESM Module */
int32_t ESM_init (void);

/***************************************************************************/
/*              RTI Defines                                                */
/***************************************************************************/
/* test for RTI on UC1 */
extern int32_t RTI_uc1(void);
/* test for RTI on UC2 */
extern int32_t RTI_uc2(void);

/***************************************************************************/
/*              BUS Defines                                                */
/***************************************************************************/
extern int32_t BUSsafety_run(void);

#ifdef __cplusplus
}
#endif

#endif /* SDLEXAMPLE_H */

/* Nothing past this point */
