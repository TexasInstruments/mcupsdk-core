/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *  Name        : sdl_stc.h
*/
/**
 *
 *  \defgroup SDL_STC_API SDL Self Test Controller(STC)
 *  \ingroup SDL_STC_MODULE
 *
 *   Provides the APIs for STC.
 *  @{
 */
/**
 *  \file     sdl_stc.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of STC.
 *            This also contains some related macros.
 */


#ifndef SDL_STC_H_
#define SDL_STC_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */


#include <stdbool.h>
#include <stdint.h>
#include <sdl/include/hw_types.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdlr.h>
#include <sdl/stc/v0/soc/sdl_soc_stc.h>
#include <sdl/soc.h>


/**
@defgroup SDL_STC_MACROS STC Macros
@ingroup SDL_STC_API
*/

/**
@defgroup SDL_STC_ENUM STC Enumerated Data Types
@ingroup SDL_STC_API
*/

/**
@defgroup SDL_STC_FUNCTIONS STC Functions
@ingroup SDL_STC_API
*/

/**
@defgroup SDL_STC_DATASTRUCT STC Data Structures
@ingroup SDL_STC_API
*/




/**************************************************************************
* STC Parameters:
**************************************************************************/
/**

@addtogroup SDL_STC_MACROS
@{
*/

/*
* STC Parameters R5F
*/

#define STC_MSS_INTERVAL_NUM                         (uint32_t)(1U)
#define STC_MSS_LP_SCAN_MODE                         (uint32_t)(0U)
#define STC_MSS_CODEC_SPREAD_MODE                    (uint32_t)(1U)
#define STC_MSS_CAP_IDLE_CYCLE                       (uint32_t)(3U)
#define STC_MSS_SCANEN_HIGH_CAP_IDLE_CYCLE           (uint32_t)(3U)
#define STC_MSS_MAX_RUN_TIME                         (uint32_t)(0xFFFFFFFFU)
#define STC_MSS_CLK_DIV                              (uint32_t)(1U)
#define STC_ROM_START_ADDRESS                        (uint32_t)(0U)
#define STC_pROM_START_ADDRESS                       (uint32_t)(1U)


/*
* STC Parameters DSP
*/

#define STC_DSS_INTERVAL_NUM                         (uint32_t)(1U)
#define STC_DSS_LP_SCAN_MODE                         (uint32_t)(0U)
#define STC_DSS_CODEC_SPREAD_MODE                    (uint32_t)(1U)
#define STC_DSS_CAP_IDLE_CYCLE                       (uint32_t)(3U)
#define STC_DSS_SCANEN_HIGH_CAP_IDLE_CYCLE           (uint32_t)(3U)
#define STC_DSS_MAX_RUN_TIME                         (uint32_t)(0xFFFFFFFFU)
#define STC_DSS_CLK_DIV                              (uint32_t)(1U)
#define STC_ROM_START_ADDRESS                        (uint32_t)(0U)
#define STC_pROM_START_ADDRESS                       (uint32_t)(1U)



/** @} */


/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */


/**

@addtogroup SDL_STC_DATASTRUCT
@{
*/
/**
 * \brief  Structure containing parameters for STC module configuration.
 */

typedef struct
{
    /** Scan mode configuration. Fixed Configuration – Only this configuration value is supported. */
    uint32_t lpScanMode;
    /** Scan mode configuration. Fixed Configuration – Only this configuration value is supported  */
    uint32_t codecSpreadMode;
    /** Scan mode configuration. Fixed Configuration – Only this configuration value is supported  */
    uint32_t capIdleCycle;
    /** Scan mode configuration. Fixed Configuration – Only this configuration value is supported  */
    uint32_t scanEnHighCap_idleCycle;

}SDL_STC_ScanModeconfig;



typedef struct
{
    /** Configure number of intervals */
    uint32_t intervalNum;
    /** Max run time for STC */
    uint32_t maxRunTime;
    /** Clock division value for STC clock */
    uint32_t clkDiv;
    /** Configure Rom start address */
    uint32_t romStartAddress;
    /** Pointer of Rom start address */
    uint32_t pRomStartAdd;
    /** Configure scan mode configuration */
    SDL_STC_ScanModeconfig modeConfig;

}SDL_STC_Config;


/** @} */

/**

@addtogroup SDL_STC_ENUM
@{
*/

typedef enum
{
    /** The STC completed and the test passed  */
    SDL_STC_COMPLETED_SUCCESS,
    /** The STC completed and the test failed  */
    SDL_STC_COMPLETED_FAILURE,
    /**The STC was active but could not be completed.  */
    SDL_STC_NOT_COMPLETED,
    /** The STC was not performed on this device  */
    SDL_STC_NOT_RUN,
    /** The STC was not performed on this device  */
    INVALID_RESULT

}SDL_STC_TestResult;

typedef enum
{
    /** The STC test should be  completed and the test passed for this testType  */
    SDL_STC_TEST,
    /** The STC test should be completed and the test failed for this testType */
    SDL_STC_NEG_TEST,
    /* Invalid test type */
    INVALID_TEST

}SDL_STC_TestType;

/** @} */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**

@addtogroup SDL_STC_FUNCTIONS
@{
*/



/**
 * \brief   This API is used to get status for STC result.
 *
 * \param   instance        Holds the instance for STC module.
 *
 * NOTE:    This API  informes only first that we should run STC test or not.if it will return
 *          that STC has been Done, then programe will be terminated simply from here.
 */
 int32_t SDL_STC_getStatus(SDL_STC_Inst instance);

/**
 * \brief   This API is used to run the STC module.
 *
 * \param   instance        Holds the instance for STC module.
 *
 * \param   testType        testType that in wwhich mode test need to be run.
 *
 * \param   pConfig         Configuration that need to be run.
 *
 *  NOTE:    This API  informes only first that we should run STC test or not.if it will return
 *           that STC has been Done, then programe will be terminated simply from here.
 *           There nothing will be returned as core will be reset after run this API.
*            testType that in wwhich mode test need to be run.
 */
int32_t   SDL_STC_selfTest(SDL_STC_Inst instance, SDL_STC_TestType testType,SDL_STC_Config *pConfig);
/**
 * \brief   This API is used to configure STC module.
 *
 * \param   instance        Holds the instance for STC module.
 *
 * \param   pConfig         pointer to the STC config structure
 *
 * \param   testType        Enum value for specifying testType
 *
 * NOTE:    There are two types of testType
 *          1. SDL_STC_TEST->     FOR THIS TEST STATUS SHOULD BE PASS.
 *          2. SDL_STC_NEG_TEST-> FOR THIS TEST STATUS SHOULD BE FAIL.
 */
static  int32_t SDL_STC_configure(SDL_STC_Inst instance, SDL_STC_Config *pConfig, SDL_STC_TestType testType);
/**
 *
 *
 * \brief   This API is used to enable the STC module.
 *
 * \param   instance        Holds the instance for STC module.
 */
static int32_t   SDL_STC_runTest(SDL_STC_Inst instance );
/**
 *
 *
 * \brief   This API is used to provide delay  for processor core.
 *
 *
 */
static void SDL_STC_delay(int32_t count);
/**
 *
 *
 * \brief   This API is used to execute asm nop operation.
 *
 *
 */
static void SDL_Delay(void);
/**
 *
 *
 * \brief   This API is used to initialize all the required configuration
 *          in RCM & CTRL Module for performing DSP STC.
 *
 */
void SDL_STC_dspInit(void);

/** @} */

/** @} */

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct
{
    /** STC Self test Global control register0 */
    volatile uint32_t STCGCR0;
    /** STC Self test Global control register1 */
    volatile uint32_t STCGCR1;
    /** STC Time out counter preload register */
    volatile uint32_t STCTPR;
    /** STC Current Address register for CORE1*/
    volatile  uint32_t STC_CADDR;
    /** STC Current Interval count register */
    volatile uint32_t STCCICR;
    /** STC Global Status Register */
    volatile  uint32_t STCGSTAT;
    /** STC Fail Status Register */
    volatile uint32_t STCFSTAT;
    /** STC Signature compare Self Check Register */
    volatile uint32_t STCSCSCR;
    /** STC Current Address register for CORE2 */
    volatile uint32_t STC_CADDR2;
    /** STC Clock Divider Register */
    volatile uint32_t STC_CLKDIV;
    /** STC Segment 1st interval Preload Register */
    volatile uint32_t STC_SEGPLR;
    /** STC ROM Start address for Segment0 */
    volatile uint32_t SEG0_START_ADDR;
    /** STC ROM Start address for Segment1 */
    volatile uint32_t SEG1_START_ADDR;
   /** STC 	ROM Start address for Segment2 */
    volatile uint32_t SEG2_START_ADDR;
   /** STC  ROM Start address for Segment3 */
    volatile uint32_t SEG3_START_ADDR;


   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_0;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_1;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_2;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_3;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_4;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_5;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_6;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_7;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_8;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_9;
    /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_10;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_11;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_12;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_13;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_14;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_15;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_16;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_17;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_18;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_19;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_20;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_21;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_22;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_23;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_24;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_25;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_26;
   /** STC Holds the MISR signature for CORE1 */
    volatile uint32_t CORE1_CURMISR_27;

} SDL_stcRegs;



/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_STC_STCGCR0                               (0x00000000U)
#define SDL_STC_STCGCR1                               (0x00000004U)
#define SDL_STC_STCTPR                                (0x00000008U)
#define SDL_STC_CADDR                                 (0x0000000CU)
#define SDL_STC_STCCICR                               (0x00000010U)
#define SDL_STC_STCGSTAT                              (0x00000014U)
#define SDL_STC_STCFSTAT                              (0x00000018U)
#define SDL_STC_STCSCSCR                              (0x0000001CU)
#define SDL_STC_CADDR2                                (0x00000020U)
#define SDL_STC_CLKDIV                                (0x00000024U)
#define SDL_STC_SEGPLR                                (0x00000028U)
#define SDL_STC_SEG0_START_ADDR                       (0x0000002CU)
#define SDL_STC_SEG1_START_ADDR                       (0x00000030U)
#define SDL_STC_SEG2_START_ADDR                       (0x00000034U)
#define SDL_STC_SEG3_START_ADDR                       (0x00000038U)


/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* STC_CTRL0 */

#define SDL_STC_STCGCR0_INTCOUNT_B16_MASK                            (0xFFFF0000U)
#define SDL_STC_STCGCR0_INTCOUNT_B16_SHIFT                           (16U)
#define SDL_STC_STCGCR0_INTCOUNT_B16_MAX                             (0xFFFF0000U)

#define SDL_STC_STCGCR0_CAP_IDLE_CYCLE_MASK                          (0x00000700U)
#define SDL_STC_STCGCR0_CAP_IDLE_CYCLE_SHIFT                         (8U)
#define SDL_STC_STCGCR0_CAP_IDLE_CYCLE_MAX                           (0x00000700U)

#define SDL_STC_STCGCR0_SCANEN_HIGH_CAP_IDLE_CYCLE_MASK              (0x000000E0U)
#define SDL_STC_STCGCR0_SCANEN_HIGH_CAP_IDLE_CYCLE_SHIFT             (5U)
#define SDL_STC_STCGCR0_SCANEN_HIGH_CAP_IDLE_CYCLE_MAX               (0x000000E0U)

#define SDL_STC_STCGCR0_RS_CNT_B1_MASK                               (0x00000003U)
#define SDL_STC_STCGCR0_RS_CNT_B1_SHIFT                              (0U)


/* STC_CTRL1 */

#define SDL_STC_SEG0_CORE_SEL_MASK                                   (0x00000F00U)
#define SDL_STC_SEG0_CORE_SEL_SHIFT                                  (8U)
#define SDL_STC_SEG0_CORE_SEL_ENABLE                                 (0x1U)


#define SDL_STC_CODEC_SPREAD_MODE_MASK                               (0x00000040U)
#define SDL_STC_CODEC_SPREAD_MODE_SHIFT                              (6U)
#define SDL_STC_CODEC_SPREAD_MODE_ENABLE                             (0x1U)
#define SDL_STC_CODEC_SPREAD_MODE_DISABLE                            (0x0U)

#define SDL_STC_LP_SCAN_MODE_MASK                                    (0x00000020U)
#define SDL_STC_LP_SCAN_MODE_SHIFT                                   (5U)
#define SDL_STC_LP_SCAN_MODE_ENABLE                                  (0x1U)
#define SDL_STC_LP_SCAN_MODE_DISABLE                                 (0x0U)


#define SDL_STC_ROM_ACCESS_INV_MASK                                  (0x00000010U)
#define SDL_STC_ROM_ACCESS_INV_SHIFT                                 (4U)
#define SDL_STC_ROM_ACCESS_INV_DISABLE                               (0x0U)

#define SDL_STC_ST_ENA_B4_MASK                                       (0x0000000FU)
#define SDL_STC_ST_ENA_B4_SHIFT                                      (0x00000000U)
#define SDL_STC_ST_ENA_B4_ENABLE                                     (0xAU)



/* STC_STCTPR */

#define SDL_STC_TO_PRELOAD_MASK                                      (0xFFFFFFFFU)
#define SDL_STC_TO_PRELOAD_SHIFT                                     (0x00000000U)
#define SDL_STC_TO_PRELOAD_MAX                                       (0xFFFFFFFFU)

/* STC_CADDR */

#define SDL_STC_ADDR1_MASK                                           (0xFFFFFFFFU)
#define SDL_STC_ADDR1_SHIFT                                          (0x00000000U)


/* STC_STCCICR */

#define SDL_STC_CORE2_ICOUNT_MASK                                    (0xFFFF0000U)
#define SDL_STC_CORE2_ICOUNT_SHIFT                                   (16U)

#define SDL_STC_CORE1_ICOUNT_MASK                                    (0x0000FFFFU)
#define SDL_STC_CORE1_ICOUNT_SHIFT                                   (0x00000000U)


/* STC_STCGSTAT */

#define SDL_STC_ST_ACTIVE_MASK                                        (0x00000F00U)
#define SDL_STC_ST_ACTIVE_SHIFT                                       (8U)
#define SDL_STC_ST_ACTIVE_ENABLE                                      (0xAU)


#define SDL_STC_TEST_FAIL_MASK                                        (0x00000002U)
#define SDL_STC_TEST_FAIL_SHIFT                                       (1U)
#define SDL_STC_TEST_FAIL_ENABLE                                      (0x1U)
#define SDL_STC_TEST_FAIL_DISABLE                                     (0x0U)

#define SDL_STC_TEST_DONE_MASK                                        (0x00000001U)
#define SDL_STC_TEST_DONE_SHIFT                                       (0U)
#define SDL_STC_TEST_DONE_ENABLE                                      (0x1U)
#define SDL_STC_TEST_DONE_DISABLE                                     (0x0U)

/* STC_STCFSTAT */

#define SDL_STC_FSEG_ID_MASK                                          (0x00000018U)
#define SDL_STC_FSEG_ID_SHIFT                                         (3U)


#define SDL_STC_TO_ER_B1_MASK                                         (0x00000004U)
#define SDL_STC_TO_ER_B1_SHIFT                                        (2U)
#define SDL_STC_TO_ER_B1_ENABLE                                       (0x1U)
#define SDL_STC_TO_ER_B1_DISABLE                                      (0x0U)

#define SDL_STC_CPU2_FAIL_B1_MASK                                     (0x00000002U)
#define SDL_STC_CPU2_FAIL_B1_SHIFT                                    (0x1U)
#define SDL_STC_CPU2_FAIL_B1_ENABLE                                   (0x1U)
#define SDL_STC_CPU2_FAIL_B1_DISABLE                                  (0x0U)

#define SDL_STC_CPU1_FAIL_B1_MASK                                     (0x00000001U)
#define SDL_STC_CPU1_FAIL_B1_SHIFT                                    (0U)
#define SDL_STC_CPU1_FAIL_B1_ENABLE                                   (0x1U)
#define SDL_STC_CPU1_FAIL_B1_DISABLE                                  (0x0U)

/* STCSCSCR */

#define SDL_STC_FAULT_INS_B1_MASK                                     (0x00000010U)
#define SDL_STC_FAULT_INS_B1_SHIFT                                    (4U)
#define SDL_STC_FAULT_INS_B1_ENABLE                                   (0x1U)
#define SDL_STC_FAULT_INS_B1_DISABLE                                  (0x0U)


#define SDL_STC_SELF_CHECK_KEY_B4_MASK                                (0x0000000FU)
#define SDL_STC_SELF_CHECK_KEY_B4_SHIFT                               (0U)
#define SDL_STC_SELF_CHECK_KEY_B4_ENABLE                              (0xAU)
#define SDL_STC_SELF_CHECK_KEY_B4_DISABLE                             (0U)



/* STC_CADDR2 */

#define SDL_STC_ADDR2_MASK                                            (0xFFFFFFFFU)
#define SDL_STC_ADDR2_SHIFT                                           (0x00000000U)

/* STC_CLKDIV */

#define SDL_STC_CLKDIV0_MASK                                           (0x07000000U)
#define SDL_STC_CLKDIV0_SHIFT                                          (24U)
#define SDL_STC_CLKDIV1_MASK                                           (0x00070000U)
#define SDL_STC_CLKDIV1_SHIFT                                          (16U)
#define SDL_STC_CLKDIV2_MASK                                           (0x00000700U)
#define SDL_STC_CLKDIV2_SHIFT                                          (8U)
#define SDL_STC_CLKDIV3_MASK                                           (0x00000007U)
#define SDL_STC_CLKDIV3_SHIFT                                          (0U)

/* STC_SEGPLR */

#define SDL_STC_SEGPLR_MASK                                            (0x00000003U)
#define SDL_STC_SEGPLR_SHIFT                                           (0U)

/* SEG0_START_ADDR */

#define SDL_STC_SEG0_START_ADDR_MASK                                   (0x000FFFFFU)
#define SDL_STC_SEG0_START_ADDR_SHIFT                                  (0U)

/* SEG1_START_ADDR */

#define SDL_STC_SEG1_START_ADDR0_MASK                                  (0x000FFFFFU)
#define SDL_STC_SEG1_START_ADDR0_SHIFT                                 (0U)

/* SEG2_START_ADDR */

#define SDL_STC_SEG2_START_ADDR0_MASK                                  (0x000FFFFFU)
#define SDL_STC_SEG2_START_ADDR0_SHIFT                                 (0U)

/* SEG3_START_ADDR */

#define SDL_STC_SEG3_START_ADDR0_MASK                                  (0x000FFFFFU)
#define SDL_STC_SEG3_START_ADDR0_SHIFT                                 (0U)

/* MSS_RCM */
#define SDL_MSS_STC_RESET_MASK                                         (0x00000004U)
#define SDL_MSS_STC_RESET_SHIFT                                        (2U)

#define SDL_MSS_STC_RESET_CLEAR_MASK                                   (0x00000007)
#define SDL_MSS_STC_RESET_CLEAR_SHIFT                                  (0U)
#define SDL_MSS_STC_RESET_CLEAR_ENABLE                                 (0x7U)

/* DSS_RCM */
#define SDL_DSS_STC_RESET_MASK                                         (0x00000020U)
#define SDL_DSS_STC_RESET_SHIFT                                        (5U)


/*DSS_ICFG*/
#define SDL_DSS_DSP_ICFG_PDCCMD_GEMPD_MASK                             (0x00010000U)
#define SDL_DSS_DSP_ICFG_PDCCMD_GEMPD_SHIFT                            (16U)

#ifdef __cplusplus
}
#endif
#endif /* SDLR_STC_H_ */
