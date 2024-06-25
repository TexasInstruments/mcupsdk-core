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
 *  Name        : sdlr_ecc_ram.h
*/
#ifndef SDLR_ECC_RAM_H_
#define SDLR_ECC_RAM_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <sdl/sdlr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_16[16];
    volatile uint32_t WRAP_REV;                /* Revision register */
    volatile uint32_t CTRL;                    /* Control register */
    volatile uint32_t ERR_CTRL1;               /* Error inject 1 register */
    volatile uint32_t ERR_CTRL2;               /* Error inject 2 register */
    volatile uint32_t ERR_STAT1;               /* Error status 1 register */
    volatile uint32_t ERR_STAT2;               /* Error status 2 register */
    volatile uint32_t ERR_STAT3;               /* Error status 3 register */
} SDL_ecc_ramRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define SDL_ECC_RAM_WRAP_REV                                                  (0x00000010U)
#define SDL_ECC_RAM_CTRL                                                      (0x00000014U)
#define SDL_ECC_RAM_ERR_CTRL1                                                 (0x00000018U)
#define SDL_ECC_RAM_ERR_CTRL2                                                 (0x0000001CU)
#define SDL_ECC_RAM_ERR_STAT1                                                 (0x00000020U)
#define SDL_ECC_RAM_ERR_STAT2                                                 (0x00000024U)
#define SDL_ECC_RAM_ERR_STAT3                                                 (0x00000028U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* WRAP_REV */

#define SDL_ECC_RAM_WRAP_REV_SCHEME_MASK                                      (0xC0000000U)
#define SDL_ECC_RAM_WRAP_REV_SCHEME_SHIFT                                     (0x0000001EU)
#define SDL_ECC_RAM_WRAP_REV_SCHEME_MAX                                       (0x00000003U)

#define SDL_ECC_RAM_WRAP_REV_BU_MASK                                          (0x30000000U)
#define SDL_ECC_RAM_WRAP_REV_BU_SHIFT                                         (0x0000001CU)
#define SDL_ECC_RAM_WRAP_REV_BU_MAX                                           (0x00000003U)

#define SDL_ECC_RAM_WRAP_REV_MODULE_ID_MASK                                   (0x0FFF0000U)
#define SDL_ECC_RAM_WRAP_REV_MODULE_ID_SHIFT                                  (0x00000010U)
#define SDL_ECC_RAM_WRAP_REV_MODULE_ID_MAX                                    (0x00000FFFU)

#define SDL_ECC_RAM_WRAP_REV_REVRTL_MASK                                      (0x0000F800U)
#define SDL_ECC_RAM_WRAP_REV_REVRTL_SHIFT                                     (0x0000000BU)
#define SDL_ECC_RAM_WRAP_REV_REVRTL_MAX                                       (0x0000001FU)

#define SDL_ECC_RAM_WRAP_REV_REVMAJ_MASK                                      (0x00000700U)
#define SDL_ECC_RAM_WRAP_REV_REVMAJ_SHIFT                                     (0x00000008U)
#define SDL_ECC_RAM_WRAP_REV_REVMAJ_MAX                                       (0x00000007U)

#define SDL_ECC_RAM_WRAP_REV_CUSTOM_MASK                                      (0x000000C0U)
#define SDL_ECC_RAM_WRAP_REV_CUSTOM_SHIFT                                     (0x00000006U)
#define SDL_ECC_RAM_WRAP_REV_CUSTOM_MAX                                       (0x00000003U)

#define SDL_ECC_RAM_WRAP_REV_REVMIN_MASK                                      (0x0000003FU)
#define SDL_ECC_RAM_WRAP_REV_REVMIN_SHIFT                                     (0x00000000U)
#define SDL_ECC_RAM_WRAP_REV_REVMIN_MAX                                       (0x0000003FU)

/* CTRL */

#define SDL_ECC_RAM_CTRL_ECC_ENABLE_MASK                                      (0x00000001U)
#define SDL_ECC_RAM_CTRL_ECC_ENABLE_SHIFT                                     (0x00000000U)
#define SDL_ECC_RAM_CTRL_ECC_ENABLE_MAX                                       (0x00000001U)

#define SDL_ECC_RAM_CTRL_ECC_CHECK_MASK                                       (0x00000002U)
#define SDL_ECC_RAM_CTRL_ECC_CHECK_SHIFT                                      (0x00000001U)
#define SDL_ECC_RAM_CTRL_ECC_CHECK_MAX                                        (0x00000001U)

#define SDL_ECC_RAM_CTRL_ENABLE_RMW_MASK                                      (0x00000004U)
#define SDL_ECC_RAM_CTRL_ENABLE_RMW_SHIFT                                     (0x00000002U)
#define SDL_ECC_RAM_CTRL_ENABLE_RMW_MAX                                       (0x00000001U)

#define SDL_ECC_RAM_CTRL_FORCE_SEC_MASK                                       (0x00000008U)
#define SDL_ECC_RAM_CTRL_FORCE_SEC_SHIFT                                      (0x00000003U)
#define SDL_ECC_RAM_CTRL_FORCE_SEC_MAX                                        (0x00000001U)

#define SDL_ECC_RAM_CTRL_FORCE_DED_MASK                                       (0x00000010U)
#define SDL_ECC_RAM_CTRL_FORCE_DED_SHIFT                                      (0x00000004U)
#define SDL_ECC_RAM_CTRL_FORCE_DED_MAX                                        (0x00000001U)

#define SDL_ECC_RAM_CTRL_FORCE_N_ROW_MASK                                     (0x00000020U)
#define SDL_ECC_RAM_CTRL_FORCE_N_ROW_SHIFT                                    (0x00000005U)
#define SDL_ECC_RAM_CTRL_FORCE_N_ROW_MAX                                      (0x00000001U)

#define SDL_ECC_RAM_CTRL_ERROR_ONCE_MASK                                      (0x00000040U)
#define SDL_ECC_RAM_CTRL_ERROR_ONCE_SHIFT                                     (0x00000006U)
#define SDL_ECC_RAM_CTRL_ERROR_ONCE_MAX                                       (0x00000001U)

#define SDL_ECC_RAM_CTRL_CHECK_PARITY_MASK                                    (0x00000080U)
#define SDL_ECC_RAM_CTRL_CHECK_PARITY_SHIFT                                   (0x00000007U)
#define SDL_ECC_RAM_CTRL_CHECK_PARITY_MAX                                     (0x00000001U)

#define SDL_ECC_RAM_CTRL_CHECK_SVBUS_TIMEOUT_MASK                             (0x00000100U)
#define SDL_ECC_RAM_CTRL_CHECK_SVBUS_TIMEOUT_SHIFT                            (0x00000008U)
#define SDL_ECC_RAM_CTRL_CHECK_SVBUS_TIMEOUT_MAX                              (0x00000001U)

/* ERR_CTRL1 */

#define SDL_ECC_RAM_ERR_CTRL1_ECC_ROW_MASK                                    (0xFFFFFFFFU)
#define SDL_ECC_RAM_ERR_CTRL1_ECC_ROW_SHIFT                                   (0x00000000U)
#define SDL_ECC_RAM_ERR_CTRL1_ECC_ROW_MAX                                     (0xFFFFFFFFU)

/* ERR_CTRL2 */

#define SDL_ECC_RAM_ERR_CTRL2_ECC_BIT1_MASK                                   (0x0000FFFFU)
#define SDL_ECC_RAM_ERR_CTRL2_ECC_BIT1_SHIFT                                  (0x00000000U)
#define SDL_ECC_RAM_ERR_CTRL2_ECC_BIT1_MAX                                    (0x0000FFFFU)

#define SDL_ECC_RAM_ERR_CTRL2_ECC_BIT2_MASK                                   (0xFFFF0000U)
#define SDL_ECC_RAM_ERR_CTRL2_ECC_BIT2_SHIFT                                  (0x00000010U)
#define SDL_ECC_RAM_ERR_CTRL2_ECC_BIT2_MAX                                    (0x0000FFFFU)

/* ERR_STAT1 */

#define SDL_ECC_RAM_ERR_STAT1_ECC_SEC_MASK                                    (0x00000003U)
#define SDL_ECC_RAM_ERR_STAT1_ECC_SEC_SHIFT                                   (0x00000000U)
#define SDL_ECC_RAM_ERR_STAT1_ECC_SEC_MAX                                     (0x00000003U)

#define SDL_ECC_RAM_ERR_STAT1_ECC_DED_MASK                                    (0x0000000CU)
#define SDL_ECC_RAM_ERR_STAT1_ECC_DED_SHIFT                                   (0x00000002U)
#define SDL_ECC_RAM_ERR_STAT1_ECC_DED_MAX                                     (0x00000003U)

#define SDL_ECC_RAM_ERR_STAT1_ECC_OTHER_MASK                                  (0x00000010U)
#define SDL_ECC_RAM_ERR_STAT1_ECC_OTHER_SHIFT                                 (0x00000004U)
#define SDL_ECC_RAM_ERR_STAT1_ECC_OTHER_MAX                                   (0x00000001U)

#define SDL_ECC_RAM_ERR_STAT1_PARITY_ERR_MASK                                 (0x00000060U)
#define SDL_ECC_RAM_ERR_STAT1_PARITY_ERR_SHIFT                                (0x00000005U)
#define SDL_ECC_RAM_ERR_STAT1_PARITY_ERR_MAX                                  (0x00000003U)

#define SDL_ECC_RAM_ERR_STAT1_CTR_REG_ERR_MASK                                (0x00000080U)
#define SDL_ECC_RAM_ERR_STAT1_CTR_REG_ERR_SHIFT                               (0x00000007U)
#define SDL_ECC_RAM_ERR_STAT1_CTR_REG_ERR_MAX                                 (0x00000001U)

#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_SEC_MASK                                (0x00000300U)
#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_SEC_SHIFT                               (0x00000008U)
#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_SEC_MAX                                 (0x00000003U)

#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_DED_MASK                                (0x00000C00U)
#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_DED_SHIFT                               (0x0000000AU)
#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_DED_MAX                                 (0x00000003U)

#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_OTHER_MASK                              (0x00001000U)
#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_OTHER_SHIFT                             (0x0000000CU)
#define SDL_ECC_RAM_ERR_STAT1_CLR_ECC_OTHER_MAX                               (0x00000001U)

#define SDL_ECC_RAM_ERR_STAT1_CLR_PARITY_ERR_MASK                             (0x00006000U)
#define SDL_ECC_RAM_ERR_STAT1_CLR_PARITY_ERR_SHIFT                            (0x0000000DU)
#define SDL_ECC_RAM_ERR_STAT1_CLR_PARITY_ERR_MAX                              (0x00000003U)

#define SDL_ECC_RAM_ERR_STAT1_CLR_CTRL_REG_ERR_MASK                           (0x00008000U)
#define SDL_ECC_RAM_ERR_STAT1_CLR_CTRL_REG_ERR_SHIFT                          (0x0000000FU)
#define SDL_ECC_RAM_ERR_STAT1_CLR_CTRL_REG_ERR_MAX                            (0x00000001U)

#define SDL_ECC_RAM_ERR_STAT1_ECC_BIT1_MASK                                   (0xFFFF0000U)
#define SDL_ECC_RAM_ERR_STAT1_ECC_BIT1_SHIFT                                  (0x00000010U)
#define SDL_ECC_RAM_ERR_STAT1_ECC_BIT1_MAX                                    (0x0000FFFFU)

/* ERR_STAT2 */

#define SDL_ECC_RAM_ERR_STAT2_ECC_ROW_MASK                                    (0xFFFFFFFFU)
#define SDL_ECC_RAM_ERR_STAT2_ECC_ROW_SHIFT                                   (0x00000000U)
#define SDL_ECC_RAM_ERR_STAT2_ECC_ROW_MAX                                     (0xFFFFFFFFU)

/* ERR_STAT3 */

#define SDL_ECC_RAM_ERR_STAT3_WB_PEND_MASK                                    (0x00000001U)
#define SDL_ECC_RAM_ERR_STAT3_WB_PEND_SHIFT                                   (0x00000000U)
#define SDL_ECC_RAM_ERR_STAT3_WB_PEND_MAX                                     (0x00000001U)

#define SDL_ECC_RAM_ERR_STAT3_SVBUS_TIMEOUT_ERR_MASK                          (0x00000002U)
#define SDL_ECC_RAM_ERR_STAT3_SVBUS_TIMEOUT_ERR_SHIFT                         (0x00000001U)
#define SDL_ECC_RAM_ERR_STAT3_SVBUS_TIMEOUT_ERR_MAX                           (0x00000001U)

#define SDL_ECC_RAM_ERR_STAT3_CLR_SVBUS_TIMEOUT_ERR_MASK                      (0x00000200U)
#define SDL_ECC_RAM_ERR_STAT3_CLR_SVBUS_TIMEOUT_ERR_SHIFT                     (0x00000009U)
#define SDL_ECC_RAM_ERR_STAT3_CLR_SVBUS_TIMEOUT_ERR_MAX                       (0x00000001U)

#ifdef __cplusplus
}
#endif
#endif
