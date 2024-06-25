/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
 *  \file     ecc_test_main.h
 *
 *  \brief    This file contains ECC main test defines.
 *
 *  \details  ECC unit tests
 **/
#ifndef ECC_TEST_MAIN_H
#define ECC_TEST_MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <sdl/include/sdl_types.h>



/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SDL_APP_TEST_NOT_RUN                        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED                         (-(int32_t) (1))
#define SDL_APP_TEST_PASS                           ( (int32_t) (0))

#if defined (SOC_AM64X)
#define  ECC_FUNC_TEST_ID         					(0U)
#define  ECC_ERROR_TEST_ID        					(1U)
#define  ECC_TOTAL_NUM_TESTS      					(2U)
#else
#define  ECC_ERROR_TEST_ID          				(0U)
#define  ECC_FUNC_TEST_ID           				(1U)
#define  ECC_TOTAL_NUM_TESTS        				(2U)
#endif

#define SDL_DSS_DSP_L2RAM_PARITY_CTRL               (0x0602006Cu)
#define SDL_DSS_L2RAM_PARITY_ENABLE                 (0xffu)
#define SDL_DSS_L2RAM_PARITY_ERROR_CLEAR            (0xff00u)

/*Error Detect and Correct Interrupt Mask Register*/
#define SDL_DSP_ICFG_EDCINTMASK                     (0x01831100u)
/*Error Detect and Correct Interrupt Flag Register*/
#define SDL_DSP_ICFG_EDCINTFLG                      (0x01831104u)

extern volatile bool gMsmcMemParityInterrupt;
extern volatile bool idmaTransferComplete;
/* ========================================================================== */
/*                 External Function Declarations                             */
/* ========================================================================== */

#if defined(SOC_AM263X)|| defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
#if defined(R5F_INPUTS)
extern int32_t ECC_ip_funcTest(void);
extern int32_t ECC_sdl_funcTest(void);
extern int32_t ECC_funcTest(void);
extern int32_t ECC_ip_errTest(void);
extern int32_t ECC_errTest(void);
extern int32_t ECC_r5_funcTest(void);
extern int32_t ECC_r5_errTest(void);
#elif defined(C66_INPUTS)
extern int32_t DSS_ECC_ip_funcTest(void);
extern int32_t DSS_ECC_sdl_funcTest(void);
extern int32_t DSS_ECC_funcTest(void);
extern int32_t DSS_ECC_ip_errTest(void);
extern int32_t DSS_ECC_errTest(void);
#if defined(SOC_AM273X)
extern int32_t DSS_sdl_funcTest(void);
#endif
#endif
#else
extern int32_t ECC_funcTest(void);
extern int32_t ECC_errTest(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ECC_TEST_MAIN_H */

/* Nothing past this point */
