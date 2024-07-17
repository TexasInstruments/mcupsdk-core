/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_mss_dcc.h
*/
#ifndef CSLR_MSS_DCC_H_
#define CSLR_MSS_DCC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t DCCGCTRL;
    volatile uint32_t DCCREV;
    volatile uint32_t DCCCNTSEED0;
    volatile uint32_t DCCVALIDSEED0;
    volatile uint32_t DCCCNTSEED1;
    volatile uint32_t DCCSTAT;
    volatile uint32_t DCCCNT0;
    volatile uint32_t DCCVALID0;
    volatile uint32_t DCCCNT1;
    volatile uint32_t DCCCLKSSRC1;
    volatile uint32_t DCCCLKSSRC0;
} CSL_mss_dccRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_DCC_DCCGCTRL                                                   (0x00000000U)
#define CSL_MSS_DCC_DCCREV                                                     (0x00000004U)
#define CSL_MSS_DCC_DCCCNTSEED0                                                (0x00000008U)
#define CSL_MSS_DCC_DCCVALIDSEED0                                              (0x0000000CU)
#define CSL_MSS_DCC_DCCCNTSEED1                                                (0x00000010U)
#define CSL_MSS_DCC_DCCSTAT                                                    (0x00000014U)
#define CSL_MSS_DCC_DCCCNT0                                                    (0x00000018U)
#define CSL_MSS_DCC_DCCVALID0                                                  (0x0000001CU)
#define CSL_MSS_DCC_DCCCNT1                                                    (0x00000020U)
#define CSL_MSS_DCC_DCCCLKSSRC1                                                (0x00000024U)
#define CSL_MSS_DCC_DCCCLKSSRC0                                                (0x00000028U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* DCCGCTRL */

#define CSL_MSS_DCC_DCCGCTRL_DCCENA_MASK                                       (0x0000000FU)
#define CSL_MSS_DCC_DCCGCTRL_DCCENA_SHIFT                                      (0x00000000U)
#define CSL_MSS_DCC_DCCGCTRL_DCCENA_RESETVAL                                   (0x00000005U)
#define CSL_MSS_DCC_DCCGCTRL_DCCENA_MAX                                        (0x0000000FU)

#define CSL_MSS_DCC_DCCGCTRL_ERRENA_MASK                                       (0x000000F0U)
#define CSL_MSS_DCC_DCCGCTRL_ERRENA_SHIFT                                      (0x00000004U)
#define CSL_MSS_DCC_DCCGCTRL_ERRENA_RESETVAL                                   (0x00000005U)
#define CSL_MSS_DCC_DCCGCTRL_ERRENA_MAX                                        (0x0000000FU)

#define CSL_MSS_DCC_DCCGCTRL_SINGLESHOT_MASK                                   (0x00000F00U)
#define CSL_MSS_DCC_DCCGCTRL_SINGLESHOT_SHIFT                                  (0x00000008U)
#define CSL_MSS_DCC_DCCGCTRL_SINGLESHOT_RESETVAL                               (0x00000005U)
#define CSL_MSS_DCC_DCCGCTRL_SINGLESHOT_MAX                                    (0x0000000FU)

#define CSL_MSS_DCC_DCCGCTRL_DONENA_MASK                                       (0x0000F000U)
#define CSL_MSS_DCC_DCCGCTRL_DONENA_SHIFT                                      (0x0000000CU)
#define CSL_MSS_DCC_DCCGCTRL_DONENA_RESETVAL                                   (0x00000005U)
#define CSL_MSS_DCC_DCCGCTRL_DONENA_MAX                                        (0x0000000FU)

#define CSL_MSS_DCC_DCCGCTRL_NU_MASK                                           (0xFFFF0000U)
#define CSL_MSS_DCC_DCCGCTRL_NU_SHIFT                                          (0x00000010U)
#define CSL_MSS_DCC_DCCGCTRL_NU_RESETVAL                                       (0x00000000U)
#define CSL_MSS_DCC_DCCGCTRL_NU_MAX                                            (0x0000FFFFU)

#define CSL_MSS_DCC_DCCGCTRL_RESETVAL                                          (0x00005555U)

/* DCCREV */

#define CSL_MSS_DCC_DCCREV_MINOR_MASK                                          (0x0000001FU)
#define CSL_MSS_DCC_DCCREV_MINOR_SHIFT                                         (0x00000000U)
#define CSL_MSS_DCC_DCCREV_MINOR_RESETVAL                                      (0x00000004U)
#define CSL_MSS_DCC_DCCREV_MINOR_MAX                                           (0x0000001FU)

#define CSL_MSS_DCC_DCCREV_CUSTOM_MASK                                         (0x00000020U)
#define CSL_MSS_DCC_DCCREV_CUSTOM_SHIFT                                        (0x00000005U)
#define CSL_MSS_DCC_DCCREV_CUSTOM_RESETVAL                                     (0x00000000U)
#define CSL_MSS_DCC_DCCREV_CUSTOM_MAX                                          (0x00000001U)

#define CSL_MSS_DCC_DCCREV_MAJOR_MASK                                          (0x000001C0U)
#define CSL_MSS_DCC_DCCREV_MAJOR_SHIFT                                         (0x00000006U)
#define CSL_MSS_DCC_DCCREV_MAJOR_RESETVAL                                      (0x00000000U)
#define CSL_MSS_DCC_DCCREV_MAJOR_MAX                                           (0x00000007U)

#define CSL_MSS_DCC_DCCREV_RTL_MASK                                            (0x00003E00U)
#define CSL_MSS_DCC_DCCREV_RTL_SHIFT                                           (0x00000009U)
#define CSL_MSS_DCC_DCCREV_RTL_RESETVAL                                        (0x00000001U)
#define CSL_MSS_DCC_DCCREV_RTL_MAX                                             (0x0000001FU)

#define CSL_MSS_DCC_DCCREV_FUNC_MASK                                           (0x03FFC000U)
#define CSL_MSS_DCC_DCCREV_FUNC_SHIFT                                          (0x0000000EU)
#define CSL_MSS_DCC_DCCREV_FUNC_RESETVAL                                       (0x00000000U)
#define CSL_MSS_DCC_DCCREV_FUNC_MAX                                            (0x00000FFFU)

#define CSL_MSS_DCC_DCCREV_NU1_MASK                                            (0x0C000000U)
#define CSL_MSS_DCC_DCCREV_NU1_SHIFT                                           (0x0000001AU)
#define CSL_MSS_DCC_DCCREV_NU1_RESETVAL                                        (0x00000000U)
#define CSL_MSS_DCC_DCCREV_NU1_MAX                                             (0x00000003U)

#define CSL_MSS_DCC_DCCREV_SCHEME_MASK                                         (0x70000000U)
#define CSL_MSS_DCC_DCCREV_SCHEME_SHIFT                                        (0x0000001CU)
#define CSL_MSS_DCC_DCCREV_SCHEME_RESETVAL                                     (0x00000004U)
#define CSL_MSS_DCC_DCCREV_SCHEME_MAX                                          (0x00000007U)

#define CSL_MSS_DCC_DCCREV_NU2_MASK                                            (0x80000000U)
#define CSL_MSS_DCC_DCCREV_NU2_SHIFT                                           (0x0000001FU)
#define CSL_MSS_DCC_DCCREV_NU2_RESETVAL                                        (0x00000000U)
#define CSL_MSS_DCC_DCCREV_NU2_MAX                                             (0x00000001U)

#define CSL_MSS_DCC_DCCREV_RESETVAL                                            (0x40000204U)

/* DCCCNTSEED0 */

#define CSL_MSS_DCC_DCCCNTSEED0_COUNTSEED0_MASK                                (0x000FFFFFU)
#define CSL_MSS_DCC_DCCCNTSEED0_COUNTSEED0_SHIFT                               (0x00000000U)
#define CSL_MSS_DCC_DCCCNTSEED0_COUNTSEED0_RESETVAL                            (0x00000000U)
#define CSL_MSS_DCC_DCCCNTSEED0_COUNTSEED0_MAX                                 (0x000FFFFFU)

#define CSL_MSS_DCC_DCCCNTSEED0_NU3_MASK                                       (0xFFF00000U)
#define CSL_MSS_DCC_DCCCNTSEED0_NU3_SHIFT                                      (0x00000014U)
#define CSL_MSS_DCC_DCCCNTSEED0_NU3_RESETVAL                                   (0x00000000U)
#define CSL_MSS_DCC_DCCCNTSEED0_NU3_MAX                                        (0x00000FFFU)

#define CSL_MSS_DCC_DCCCNTSEED0_RESETVAL                                       (0x00000000U)

/* DCCVALIDSEED0 */

#define CSL_MSS_DCC_DCCVALIDSEED0_VALIDSEED0_MASK                              (0x0000FFFFU)
#define CSL_MSS_DCC_DCCVALIDSEED0_VALIDSEED0_SHIFT                             (0x00000000U)
#define CSL_MSS_DCC_DCCVALIDSEED0_VALIDSEED0_RESETVAL                          (0x00000000U)
#define CSL_MSS_DCC_DCCVALIDSEED0_VALIDSEED0_MAX                               (0x0000FFFFU)

#define CSL_MSS_DCC_DCCVALIDSEED0_NU4_MASK                                     (0xFFFF0000U)
#define CSL_MSS_DCC_DCCVALIDSEED0_NU4_SHIFT                                    (0x00000010U)
#define CSL_MSS_DCC_DCCVALIDSEED0_NU4_RESETVAL                                 (0x00000000U)
#define CSL_MSS_DCC_DCCVALIDSEED0_NU4_MAX                                      (0x0000FFFFU)

#define CSL_MSS_DCC_DCCVALIDSEED0_RESETVAL                                     (0x00000000U)

/* DCCCNTSEED1 */

#define CSL_MSS_DCC_DCCCNTSEED1_COUNTSEED1_MASK                                (0x000FFFFFU)
#define CSL_MSS_DCC_DCCCNTSEED1_COUNTSEED1_SHIFT                               (0x00000000U)
#define CSL_MSS_DCC_DCCCNTSEED1_COUNTSEED1_RESETVAL                            (0x00000000U)
#define CSL_MSS_DCC_DCCCNTSEED1_COUNTSEED1_MAX                                 (0x000FFFFFU)

#define CSL_MSS_DCC_DCCCNTSEED1_NU5_MASK                                       (0xFFF00000U)
#define CSL_MSS_DCC_DCCCNTSEED1_NU5_SHIFT                                      (0x00000014U)
#define CSL_MSS_DCC_DCCCNTSEED1_NU5_RESETVAL                                   (0x00000000U)
#define CSL_MSS_DCC_DCCCNTSEED1_NU5_MAX                                        (0x00000FFFU)

#define CSL_MSS_DCC_DCCCNTSEED1_RESETVAL                                       (0x00000000U)

/* DCCSTAT */

#define CSL_MSS_DCC_DCCSTAT_ERR_MASK                                           (0x00000001U)
#define CSL_MSS_DCC_DCCSTAT_ERR_SHIFT                                          (0x00000000U)
#define CSL_MSS_DCC_DCCSTAT_ERR_RESETVAL                                       (0x00000000U)
#define CSL_MSS_DCC_DCCSTAT_ERR_MAX                                            (0x00000001U)

#define CSL_MSS_DCC_DCCSTAT_DONE_MASK                                          (0x00000002U)
#define CSL_MSS_DCC_DCCSTAT_DONE_SHIFT                                         (0x00000001U)
#define CSL_MSS_DCC_DCCSTAT_DONE_RESETVAL                                      (0x00000000U)
#define CSL_MSS_DCC_DCCSTAT_DONE_MAX                                           (0x00000001U)

#define CSL_MSS_DCC_DCCSTAT_NU6_MASK                                           (0xFFFFFFFCU)
#define CSL_MSS_DCC_DCCSTAT_NU6_SHIFT                                          (0x00000002U)
#define CSL_MSS_DCC_DCCSTAT_NU6_RESETVAL                                       (0x00000000U)
#define CSL_MSS_DCC_DCCSTAT_NU6_MAX                                            (0x3FFFFFFFU)

#define CSL_MSS_DCC_DCCSTAT_RESETVAL                                           (0x00000000U)

/* DCCCNT0 */

#define CSL_MSS_DCC_DCCCNT0_COUNT0_MASK                                        (0x000FFFFFU)
#define CSL_MSS_DCC_DCCCNT0_COUNT0_SHIFT                                       (0x00000000U)
#define CSL_MSS_DCC_DCCCNT0_COUNT0_RESETVAL                                    (0x00000000U)
#define CSL_MSS_DCC_DCCCNT0_COUNT0_MAX                                         (0x000FFFFFU)

#define CSL_MSS_DCC_DCCCNT0_NU7_MASK                                           (0xFFF00000U)
#define CSL_MSS_DCC_DCCCNT0_NU7_SHIFT                                          (0x00000014U)
#define CSL_MSS_DCC_DCCCNT0_NU7_RESETVAL                                       (0x00000000U)
#define CSL_MSS_DCC_DCCCNT0_NU7_MAX                                            (0x00000FFFU)

#define CSL_MSS_DCC_DCCCNT0_RESETVAL                                           (0x00000000U)

/* DCCVALID0 */

#define CSL_MSS_DCC_DCCVALID0_VALID0_MASK                                      (0x0000FFFFU)
#define CSL_MSS_DCC_DCCVALID0_VALID0_SHIFT                                     (0x00000000U)
#define CSL_MSS_DCC_DCCVALID0_VALID0_RESETVAL                                  (0x00000000U)
#define CSL_MSS_DCC_DCCVALID0_VALID0_MAX                                       (0x0000FFFFU)

#define CSL_MSS_DCC_DCCVALID0_NU8_MASK                                         (0xFFFF0000U)
#define CSL_MSS_DCC_DCCVALID0_NU8_SHIFT                                        (0x00000010U)
#define CSL_MSS_DCC_DCCVALID0_NU8_RESETVAL                                     (0x00000000U)
#define CSL_MSS_DCC_DCCVALID0_NU8_MAX                                          (0x0000FFFFU)

#define CSL_MSS_DCC_DCCVALID0_RESETVAL                                         (0x00000000U)

/* DCCCNT1 */

#define CSL_MSS_DCC_DCCCNT1_COUNT1_MASK                                        (0x000FFFFFU)
#define CSL_MSS_DCC_DCCCNT1_COUNT1_SHIFT                                       (0x00000000U)
#define CSL_MSS_DCC_DCCCNT1_COUNT1_RESETVAL                                    (0x00000000U)
#define CSL_MSS_DCC_DCCCNT1_COUNT1_MAX                                         (0x000FFFFFU)

#define CSL_MSS_DCC_DCCCNT1_NU9_MASK                                           (0xFFF00000U)
#define CSL_MSS_DCC_DCCCNT1_NU9_SHIFT                                          (0x00000014U)
#define CSL_MSS_DCC_DCCCNT1_NU9_RESETVAL                                       (0x00000000U)
#define CSL_MSS_DCC_DCCCNT1_NU9_MAX                                            (0x00000FFFU)

#define CSL_MSS_DCC_DCCCNT1_RESETVAL                                           (0x00000000U)

/* DCCCLKSSRC1 */

#define CSL_MSS_DCC_DCCCLKSSRC1_CLK_SRC1_MASK                                  (0x0000000FU)
#define CSL_MSS_DCC_DCCCLKSSRC1_CLK_SRC1_SHIFT                                 (0x00000000U)
#define CSL_MSS_DCC_DCCCLKSSRC1_CLK_SRC1_RESETVAL                              (0x00000000U)
#define CSL_MSS_DCC_DCCCLKSSRC1_CLK_SRC1_MAX                                   (0x0000000FU)

#define CSL_MSS_DCC_DCCCLKSSRC1_NU10_MASK                                      (0x00000FF0U)
#define CSL_MSS_DCC_DCCCLKSSRC1_NU10_SHIFT                                     (0x00000004U)
#define CSL_MSS_DCC_DCCCLKSSRC1_NU10_RESETVAL                                  (0x00000000U)
#define CSL_MSS_DCC_DCCCLKSSRC1_NU10_MAX                                       (0x000000FFU)

#define CSL_MSS_DCC_DCCCLKSSRC1_KEY_B4_MASK                                    (0x0000F000U)
#define CSL_MSS_DCC_DCCCLKSSRC1_KEY_B4_SHIFT                                   (0x0000000CU)
#define CSL_MSS_DCC_DCCCLKSSRC1_KEY_B4_RESETVAL                                (0x00000005U)
#define CSL_MSS_DCC_DCCCLKSSRC1_KEY_B4_MAX                                     (0x0000000FU)

#define CSL_MSS_DCC_DCCCLKSSRC1_NU11_MASK                                      (0xFFFF0000U)
#define CSL_MSS_DCC_DCCCLKSSRC1_NU11_SHIFT                                     (0x00000010U)
#define CSL_MSS_DCC_DCCCLKSSRC1_NU11_RESETVAL                                  (0x00000000U)
#define CSL_MSS_DCC_DCCCLKSSRC1_NU11_MAX                                       (0x0000FFFFU)

#define CSL_MSS_DCC_DCCCLKSSRC1_RESETVAL                                       (0x00005000U)

/* DCCCLKSSRC0 */

#define CSL_MSS_DCC_DCCCLKSSRC0_CLK_SRC0_MASK                                  (0x0000000FU)
#define CSL_MSS_DCC_DCCCLKSSRC0_CLK_SRC0_SHIFT                                 (0x00000000U)
#define CSL_MSS_DCC_DCCCLKSSRC0_CLK_SRC0_RESETVAL                              (0x00000005U)
#define CSL_MSS_DCC_DCCCLKSSRC0_CLK_SRC0_MAX                                   (0x0000000FU)

#define CSL_MSS_DCC_DCCCLKSSRC0_NU12_MASK                                      (0xFFFFFFF0U)
#define CSL_MSS_DCC_DCCCLKSSRC0_NU12_SHIFT                                     (0x00000004U)
#define CSL_MSS_DCC_DCCCLKSSRC0_NU12_RESETVAL                                  (0x00000000U)
#define CSL_MSS_DCC_DCCCLKSSRC0_NU12_MAX                                       (0x0FFFFFFFU)

#define CSL_MSS_DCC_DCCCLKSSRC0_RESETVAL                                       (0x00000005U)

#ifdef __cplusplus
}
#endif
#endif
