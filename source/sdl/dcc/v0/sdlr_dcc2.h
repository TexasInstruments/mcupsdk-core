/********************************************************************
 * Copyright (C) 2019 Texas Instruments Incorporated.
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
 *  Name        : sdlr_dcc2.h
*/
#ifndef SDLR_DCC2_H_
#define SDLR_DCC2_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t DCCGCTRL;                  /* DCC Global Control Register */
    volatile uint32_t DCCREV;                    /* DCC Revision ID */
    volatile uint32_t DCCCNTSEED0;               /* Count0 Seed Value Register */
    volatile uint32_t DCCVALIDSEED0;             /* Valid0 Seed Value Register */
    volatile uint32_t DCCCNTSEED1;               /* Count1 Seed Value Register */
    volatile uint32_t DCCSTATUS;                 /* DCC Status Register */
    volatile uint32_t DCCCNT0;                   /* Count0 Value Register */
    volatile uint32_t DCCVALID0;                 /* Valid0 Value Register */
    volatile uint32_t DCCCNT1;                   /* Count1 Value Register */
    volatile uint32_t DCCCLKSRC1;                /* Clock Source Selection Register 1 */
    volatile uint32_t DCCCLKSRC0;                /* Clock Source Selection Register 0 */
    volatile uint32_t DCCGCTRL2;                 /* DCC Global Control Register 2 */
    volatile uint32_t DCCSTATUS2;                /* DCC FIFO Status Register */
    volatile uint32_t DCCERRCNT;                 /* Error Count Register */
} SDL_dcc2Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_DCC2_DCCGCTRL                                                      (0x00000000U)
#define SDL_DCC2_DCCREV                                                        (0x00000004U)
#define SDL_DCC2_DCCCNTSEED0                                                   (0x00000008U)
#define SDL_DCC2_DCCVALIDSEED0                                                 (0x0000000CU)
#define SDL_DCC2_DCCCNTSEED1                                                   (0x00000010U)
#define SDL_DCC2_DCCSTATUS                                                     (0x00000014U)
#define SDL_DCC2_DCCCNT0                                                       (0x00000018U)
#define SDL_DCC2_DCCVALID0                                                     (0x0000001CU)
#define SDL_DCC2_DCCCNT1                                                       (0x00000020U)
#define SDL_DCC2_DCCCLKSRC1                                                    (0x00000024U)
#define SDL_DCC2_DCCCLKSRC0                                                    (0x00000028U)
#define SDL_DCC2_DCCGCTRL2                                                     (0x0000002CU)
#define SDL_DCC2_DCCSTATUS2                                                    (0x00000030U)
#define SDL_DCC2_DCCERRCNT                                                     (0x00000034U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* DCCGCTRL */

#define SDL_DCC2_DCCGCTRL_DCCENA_MASK                                          (0x0000000FU)
#define SDL_DCC2_DCCGCTRL_DCCENA_SHIFT                                         (0x00000000U)
#define SDL_DCC2_DCCGCTRL_DCCENA_RESETVAL                                      (0x00000005U)
#define SDL_DCC2_DCCGCTRL_DCCENA_ENABLE                                        (0x0000000FU)
#define SDL_DCC2_DCCGCTRL_DCCENA_DISABLE                                       (0x00000005U)
#define SDL_DCC2_DCCGCTRL_DCCENA_MAX                                           (0x0000000FU)

#define SDL_DCC2_DCCGCTRL_ERRENA_MASK                                          (0x000000F0U)
#define SDL_DCC2_DCCGCTRL_ERRENA_SHIFT                                         (0x00000004U)
#define SDL_DCC2_DCCGCTRL_ERRENA_RESETVAL                                      (0x00000005U)
#define SDL_DCC2_DCCGCTRL_ERRENA_ENABLE                                        (0x0000000FU)
#define SDL_DCC2_DCCGCTRL_ERRENA_DISABLE                                       (0x00000005U)
#define SDL_DCC2_DCCGCTRL_ERRENA_MAX                                           (0x0000000FU)

#define SDL_DCC2_DCCGCTRL_SINGLESHOT_MASK                                      (0x00000F00U)
#define SDL_DCC2_DCCGCTRL_SINGLESHOT_SHIFT                                     (0x00000008U)
#define SDL_DCC2_DCCGCTRL_SINGLESHOT_RESETVAL                                  (0x00000005U)
#define SDL_DCC2_DCCGCTRL_SINGLESHOT_MODE1                                     (0x0000000AU)
#define SDL_DCC2_DCCGCTRL_SINGLESHOT_MODE2                                     (0x0000000BU)
#define SDL_DCC2_DCCGCTRL_SINGLESHOT_DISABLE                                   (0x00000000U)
#define SDL_DCC2_DCCGCTRL_SINGLESHOT_MAX                                       (0x0000000FU)

#define SDL_DCC2_DCCGCTRL_DONEENA_MASK                                         (0x0000F000U)
#define SDL_DCC2_DCCGCTRL_DONEENA_SHIFT                                        (0x0000000CU)
#define SDL_DCC2_DCCGCTRL_DONEENA_RESETVAL                                     (0x00000005U)
#define SDL_DCC2_DCCGCTRL_DONEENA_ENABLE                                       (0x0000000FU)
#define SDL_DCC2_DCCGCTRL_DONEENA_DISABLE                                      (0x00000005U)
#define SDL_DCC2_DCCGCTRL_DONEENA_MAX                                          (0x0000000FU)

#define SDL_DCC2_DCCGCTRL_RESETVAL                                             (0x00005555U)

/* DCCREV */

#define SDL_DCC2_DCCREV_MINOR_MASK                                             (0x0000003FU)
#define SDL_DCC2_DCCREV_MINOR_SHIFT                                            (0x00000000U)
#define SDL_DCC2_DCCREV_MINOR_RESETVAL                                         (0x00000004U)
#define SDL_DCC2_DCCREV_MINOR_MAX                                              (0x0000003FU)

#define SDL_DCC2_DCCREV_CUSTOM_MASK                                            (0x000000C0U)
#define SDL_DCC2_DCCREV_CUSTOM_SHIFT                                           (0x00000006U)
#define SDL_DCC2_DCCREV_CUSTOM_RESETVAL                                        (0x00000000U)
#define SDL_DCC2_DCCREV_CUSTOM_MAX                                             (0x00000003U)

#define SDL_DCC2_DCCREV_MAJOR_MASK                                             (0x00000700U)
#define SDL_DCC2_DCCREV_MAJOR_SHIFT                                            (0x00000008U)
#define SDL_DCC2_DCCREV_MAJOR_RESETVAL                                         (0x00000002U)
#define SDL_DCC2_DCCREV_MAJOR_MAX                                              (0x00000007U)

#define SDL_DCC2_DCCREV_RTL_MASK                                               (0x0000F800U)
#define SDL_DCC2_DCCREV_RTL_SHIFT                                              (0x0000000BU)
#define SDL_DCC2_DCCREV_RTL_RESETVAL                                           (0x00000000U)
#define SDL_DCC2_DCCREV_RTL_MAX                                                (0x0000001FU)

#define SDL_DCC2_DCCREV_FUNC_MASK                                              (0x0FFF0000U)
#define SDL_DCC2_DCCREV_FUNC_SHIFT                                             (0x00000010U)
#define SDL_DCC2_DCCREV_FUNC_RESETVAL                                          (0x00000000U)
#define SDL_DCC2_DCCREV_FUNC_MAX                                               (0x00000FFFU)

#define SDL_DCC2_DCCREV_SCHEME_MASK                                            (0xC0000000U)
#define SDL_DCC2_DCCREV_SCHEME_SHIFT                                           (0x0000001EU)
#define SDL_DCC2_DCCREV_SCHEME_RESETVAL                                        (0x00000001U)
#define SDL_DCC2_DCCREV_SCHEME_MAX                                             (0x00000003U)

#define SDL_DCC2_DCCREV_RESETVAL                                               (0x40000204U)

/* DCCCNTSEED0 */

#define SDL_DCC2_DCCCNTSEED0_COUNTSEED0_MASK                                   (0x000FFFFFU)
#define SDL_DCC2_DCCCNTSEED0_COUNTSEED0_SHIFT                                  (0x00000000U)
#define SDL_DCC2_DCCCNTSEED0_COUNTSEED0_RESETVAL                               (0x00000000U)
#define SDL_DCC2_DCCCNTSEED0_COUNTSEED0_MAX                                    (0x000FFFFFU)

#define SDL_DCC2_DCCCNTSEED0_RESETVAL                                          (0x00000000U)

/* DCCVALIDSEED0 */

#define SDL_DCC2_DCCVALIDSEED0_VALIDSEED0_MASK                                 (0x0000FFFFU)
#define SDL_DCC2_DCCVALIDSEED0_VALIDSEED0_SHIFT                                (0x00000000U)
#define SDL_DCC2_DCCVALIDSEED0_VALIDSEED0_RESETVAL                             (0x00000000U)
#define SDL_DCC2_DCCVALIDSEED0_VALIDSEED0_MAX                                  (0x0000FFFFU)

#define SDL_DCC2_DCCVALIDSEED0_RESETVAL                                        (0x00000000U)

/* DCCCNTSEED1 */

#define SDL_DCC2_DCCCNTSEED1_COUNTSEED1_MASK                                   (0x000FFFFFU)
#define SDL_DCC2_DCCCNTSEED1_COUNTSEED1_SHIFT                                  (0x00000000U)
#define SDL_DCC2_DCCCNTSEED1_COUNTSEED1_RESETVAL                               (0x00000000U)
#define SDL_DCC2_DCCCNTSEED1_COUNTSEED1_MAX                                    (0x000FFFFFU)

#define SDL_DCC2_DCCCNTSEED1_RESETVAL                                          (0x00000000U)

/* DCCSTATUS */

#define SDL_DCC2_DCCSTATUS_ERR_MASK                                            (0x00000001U)
#define SDL_DCC2_DCCSTATUS_ERR_SHIFT                                           (0x00000000U)
#define SDL_DCC2_DCCSTATUS_ERR_RESETVAL                                        (0x00000000U)
#define SDL_DCC2_DCCSTATUS_ERR_MAX                                             (0x00000001U)

#define SDL_DCC2_DCCSTATUS_DONE_MASK                                           (0x00000002U)
#define SDL_DCC2_DCCSTATUS_DONE_SHIFT                                          (0x00000001U)
#define SDL_DCC2_DCCSTATUS_DONE_RESETVAL                                       (0x00000000U)
#define SDL_DCC2_DCCSTATUS_DONE_MAX                                            (0x00000001U)

#define SDL_DCC2_DCCSTATUS_RESETVAL                                            (0x00000000U)

/* DCCCNT0 */

#define SDL_DCC2_DCCCNT0_COUNT0_MASK                                           (0x000FFFFFU)
#define SDL_DCC2_DCCCNT0_COUNT0_SHIFT                                          (0x00000000U)
#define SDL_DCC2_DCCCNT0_COUNT0_RESETVAL                                       (0x00000000U)
#define SDL_DCC2_DCCCNT0_COUNT0_MAX                                            (0x000FFFFFU)

#define SDL_DCC2_DCCCNT0_RESETVAL                                              (0x00000000U)

/* DCCVALID0 */

#define SDL_DCC2_DCCVALID0_VALID0_MASK                                         (0x0000FFFFU)
#define SDL_DCC2_DCCVALID0_VALID0_SHIFT                                        (0x00000000U)
#define SDL_DCC2_DCCVALID0_VALID0_RESETVAL                                     (0x00000000U)
#define SDL_DCC2_DCCVALID0_VALID0_MAX                                          (0x0000FFFFU)

#define SDL_DCC2_DCCVALID0_RESETVAL                                            (0x00000000U)

/* DCCCNT1 */

#define SDL_DCC2_DCCCNT1_COUNT1_MASK                                           (0x000FFFFFU)
#define SDL_DCC2_DCCCNT1_COUNT1_SHIFT                                          (0x00000000U)
#define SDL_DCC2_DCCCNT1_COUNT1_RESETVAL                                       (0x00000000U)
#define SDL_DCC2_DCCCNT1_COUNT1_MAX                                            (0x000FFFFFU)

#define SDL_DCC2_DCCCNT1_RESETVAL                                              (0x00000000U)

/* DCCCLKSRC1 */

#define SDL_DCC2_DCCCLKSRC1_CLKSRC1_MASK                                       (0x0000001FU)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC1_SHIFT                                      (0x00000000U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC1_RESETVAL                                   (0x00000000U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_0                                           (0x00000000U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_1                                           (0x00000001U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_2                                           (0x00000002U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_3                                           (0x00000003U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_4                                           (0x00000004U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_5                                           (0x00000005U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_6                                           (0x00000006U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_7                                           (0x00000007U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_8                                           (0x00000008U)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC_OTHER                                       (0x0000000FU)
#define SDL_DCC2_DCCCLKSRC1_CLKSRC1_MAX                                        (0x0000001FU)

#define SDL_DCC2_DCCCLKSRC1_KEY_MASK                                           (0x0000F000U)
#define SDL_DCC2_DCCCLKSRC1_KEY_SHIFT                                          (0x0000000CU)
#define SDL_DCC2_DCCCLKSRC1_KEY_RESETVAL                                       (0x00000005U)
#define SDL_DCC2_DCCCLKSRC1_KEY_ENABLE                                         (0x0000000AU)
#define SDL_DCC2_DCCCLKSRC1_KEY_MAX                                            (0x0000000FU)

#define SDL_DCC2_DCCCLKSRC1_RESETVAL                                           (0x00005000U)

/* DCCCLKSRC0 */

#define SDL_DCC2_DCCCLKSRC0_CLKSRC0_MASK                                       (0x0000000FU)
#define SDL_DCC2_DCCCLKSRC0_CLKSRC0_SHIFT                                      (0x00000000U)
#define SDL_DCC2_DCCCLKSRC0_CLKSRC0_RESETVAL                                   (0x00000005U)
#define SDL_DCC2_DCCCLKSRC0_CLKSRC0_0                                          (0x00000000U)
#define SDL_DCC2_DCCCLKSRC0_CLKSRC0_1                                          (0x00000001U)
#define SDL_DCC2_DCCCLKSRC0_CLKSRC0_2                                          (0x00000002U)
#define SDL_DCC2_DCCCLKSRC0_CLKSRC0_MAX                                        (0x0000000FU)

#define SDL_DCC2_DCCCLKSRC0_KEY_MASK                                           (0x0000F000U)
#define SDL_DCC2_DCCCLKSRC0_KEY_SHIFT                                          (0x0000000CU)
#define SDL_DCC2_DCCCLKSRC0_KEY_RESETVAL                                       (0x00000000U)
#define SDL_DCC2_DCCCLKSRC0_KEY_ENABLE                                         (0x0000000AU)
#define SDL_DCC2_DCCCLKSRC0_KEY_MAX                                            (0x0000000FU)

#define SDL_DCC2_DCCCLKSRC0_RESETVAL                                           (0x00000005U)

/* DCCGCTRL2 */

#define SDL_DCC2_DCCGCTRL2_CONT_ON_ERR_MASK                                    (0x0000000FU)
#define SDL_DCC2_DCCGCTRL2_CONT_ON_ERR_SHIFT                                   (0x00000000U)
#define SDL_DCC2_DCCGCTRL2_CONT_ON_ERR_RESETVAL                                (0x00000005U)
#define SDL_DCC2_DCCGCTRL2_CONT_ON_ERR_MAX                                     (0x0000000FU)

#define SDL_DCC2_DCCGCTRL2_FIFO_READ_MASK                                      (0x000000F0U)
#define SDL_DCC2_DCCGCTRL2_FIFO_READ_SHIFT                                     (0x00000004U)
#define SDL_DCC2_DCCGCTRL2_FIFO_READ_RESETVAL                                  (0x00000005U)
#define SDL_DCC2_DCCGCTRL2_FIFO_READ_MAX                                       (0x0000000FU)

#define SDL_DCC2_DCCGCTRL2_FIFO_NONERR_MASK                                    (0x00000F00U)
#define SDL_DCC2_DCCGCTRL2_FIFO_NONERR_SHIFT                                   (0x00000008U)
#define SDL_DCC2_DCCGCTRL2_FIFO_NONERR_RESETVAL                                (0x00000005U)
#define SDL_DCC2_DCCGCTRL2_FIFO_NONERR_MAX                                     (0x0000000FU)

#define SDL_DCC2_DCCGCTRL2_RESETVAL                                            (0x00000555U)

/* DCCSTATUS2 */

#define SDL_DCC2_DCCSTATUS2_COUNT0_FIFO_EMPTY_MASK                             (0x00000001U)
#define SDL_DCC2_DCCSTATUS2_COUNT0_FIFO_EMPTY_SHIFT                            (0x00000000U)
#define SDL_DCC2_DCCSTATUS2_COUNT0_FIFO_EMPTY_RESETVAL                         (0x00000001U)
#define SDL_DCC2_DCCSTATUS2_COUNT0_FIFO_EMPTY_MAX                              (0x00000001U)

#define SDL_DCC2_DCCSTATUS2_VALID0_FIFO_EMPTY_MASK                             (0x00000002U)
#define SDL_DCC2_DCCSTATUS2_VALID0_FIFO_EMPTY_SHIFT                            (0x00000001U)
#define SDL_DCC2_DCCSTATUS2_VALID0_FIFO_EMPTY_RESETVAL                         (0x00000001U)
#define SDL_DCC2_DCCSTATUS2_VALID0_FIFO_EMPTY_MAX                              (0x00000001U)

#define SDL_DCC2_DCCSTATUS2_COUNT1_FIFO_EMPTY_MASK                             (0x00000004U)
#define SDL_DCC2_DCCSTATUS2_COUNT1_FIFO_EMPTY_SHIFT                            (0x00000002U)
#define SDL_DCC2_DCCSTATUS2_COUNT1_FIFO_EMPTY_RESETVAL                         (0x00000001U)
#define SDL_DCC2_DCCSTATUS2_COUNT1_FIFO_EMPTY_MAX                              (0x00000001U)

#define SDL_DCC2_DCCSTATUS2_COUNT0_FIFO_FULL_MASK                              (0x00000008U)
#define SDL_DCC2_DCCSTATUS2_COUNT0_FIFO_FULL_SHIFT                             (0x00000003U)
#define SDL_DCC2_DCCSTATUS2_COUNT0_FIFO_FULL_RESETVAL                          (0x00000000U)
#define SDL_DCC2_DCCSTATUS2_COUNT0_FIFO_FULL_MAX                               (0x00000001U)

#define SDL_DCC2_DCCSTATUS2_VALID0_FIFO_FULL_MASK                              (0x00000010U)
#define SDL_DCC2_DCCSTATUS2_VALID0_FIFO_FULL_SHIFT                             (0x00000004U)
#define SDL_DCC2_DCCSTATUS2_VALID0_FIFO_FULL_RESETVAL                          (0x00000000U)
#define SDL_DCC2_DCCSTATUS2_VALID0_FIFO_FULL_MAX                               (0x00000001U)

#define SDL_DCC2_DCCSTATUS2_COUNT1_FIFO_FULL_MASK                              (0x00000020U)
#define SDL_DCC2_DCCSTATUS2_COUNT1_FIFO_FULL_SHIFT                             (0x00000005U)
#define SDL_DCC2_DCCSTATUS2_COUNT1_FIFO_FULL_RESETVAL                          (0x00000000U)
#define SDL_DCC2_DCCSTATUS2_COUNT1_FIFO_FULL_MAX                               (0x00000001U)

#define SDL_DCC2_DCCSTATUS2_RESETVAL                                           (0x00000007U)

/* DCCERRCNT */

#define SDL_DCC2_DCCERRCNT_ERRCNT_MASK                                         (0x000003FFU)
#define SDL_DCC2_DCCERRCNT_ERRCNT_SHIFT                                        (0x00000000U)
#define SDL_DCC2_DCCERRCNT_ERRCNT_RESETVAL                                     (0x00000000U)
#define SDL_DCC2_DCCERRCNT_ERRCNT_MAX                                          (0x000003FFU)

#define SDL_DCC2_DCCERRCNT_RESETVAL                                            (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
