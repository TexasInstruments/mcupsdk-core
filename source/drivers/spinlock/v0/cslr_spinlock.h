/*
 *  Copyright (C) 2021 Texas Instruments Incorporated.
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

#ifndef CSLR_SPINLOCK_H_
#define CSLR_SPINLOCK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REVISION;                  /* Peripheral ID register */
    volatile uint8_t  Resv_16[12];
    volatile uint32_t SYSCONFIG;                 /* SpinLock top level configuration */
    volatile uint32_t SYSTATUS;                  /* SpinLock top level status */
    volatile uint8_t  Resv_2048[2024];
    volatile uint32_t LOCK_REG[256];             /* Lock[a] register */
} CSL_spinlockRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_SPINLOCK_REVISION                                                  (0x00000000U)
#define CSL_SPINLOCK_SYSCONFIG                                                 (0x00000010U)
#define CSL_SPINLOCK_SYSTATUS                                                  (0x00000014U)
#define CSL_SPINLOCK_LOCK_REG(LOCK_REG)                                        (0x00000800U+((LOCK_REG)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVISION */

#define CSL_SPINLOCK_REVISION_SCHEME_MASK                                      (0xC0000000U)
#define CSL_SPINLOCK_REVISION_SCHEME_SHIFT                                     (0x0000001EU)
#define CSL_SPINLOCK_REVISION_SCHEME_MAX                                       (0x00000003U)

#define CSL_SPINLOCK_REVISION_BU_MASK                                          (0x30000000U)
#define CSL_SPINLOCK_REVISION_BU_SHIFT                                         (0x0000001CU)
#define CSL_SPINLOCK_REVISION_BU_MAX                                           (0x00000003U)

#define CSL_SPINLOCK_REVISION_FUNC_MASK                                        (0x0FFF0000U)
#define CSL_SPINLOCK_REVISION_FUNC_SHIFT                                       (0x00000010U)
#define CSL_SPINLOCK_REVISION_FUNC_MAX                                         (0x00000FFFU)

#define CSL_SPINLOCK_REVISION_R_RTL_MASK                                       (0x0000F800U)
#define CSL_SPINLOCK_REVISION_R_RTL_SHIFT                                      (0x0000000BU)
#define CSL_SPINLOCK_REVISION_R_RTL_MAX                                        (0x0000001FU)

#define CSL_SPINLOCK_REVISION_X_MAJOR_MASK                                     (0x00000700U)
#define CSL_SPINLOCK_REVISION_X_MAJOR_SHIFT                                    (0x00000008U)
#define CSL_SPINLOCK_REVISION_X_MAJOR_MAX                                      (0x00000007U)

#define CSL_SPINLOCK_REVISION_CUSTOM_MASK                                      (0x000000C0U)
#define CSL_SPINLOCK_REVISION_CUSTOM_SHIFT                                     (0x00000006U)
#define CSL_SPINLOCK_REVISION_CUSTOM_MAX                                       (0x00000003U)

#define CSL_SPINLOCK_REVISION_Y_MINOR_MASK                                     (0x0000003FU)
#define CSL_SPINLOCK_REVISION_Y_MINOR_SHIFT                                    (0x00000000U)
#define CSL_SPINLOCK_REVISION_Y_MINOR_MAX                                      (0x0000003FU)

/* SYSCONFIG */

#define CSL_SPINLOCK_SYSCONFIG_SOFTRESET_MASK                                  (0x00000002U)
#define CSL_SPINLOCK_SYSCONFIG_SOFTRESET_SHIFT                                 (0x00000001U)
#define CSL_SPINLOCK_SYSCONFIG_SOFTRESET_MAX                                   (0x00000001U)

/* SYSTATUS */

#define CSL_SPINLOCK_SYSTATUS_IU0_MASK                                         (0x00000001U)
#define CSL_SPINLOCK_SYSTATUS_IU0_SHIFT                                        (0x00000000U)
#define CSL_SPINLOCK_SYSTATUS_IU0_MAX                                          (0x00000001U)

#define CSL_SPINLOCK_SYSTATUS_IU1_MASK                                         (0x00000002U)
#define CSL_SPINLOCK_SYSTATUS_IU1_SHIFT                                        (0x00000001U)
#define CSL_SPINLOCK_SYSTATUS_IU1_MAX                                          (0x00000001U)

#define CSL_SPINLOCK_SYSTATUS_IU2_MASK                                         (0x00000004U)
#define CSL_SPINLOCK_SYSTATUS_IU2_SHIFT                                        (0x00000002U)
#define CSL_SPINLOCK_SYSTATUS_IU2_MAX                                          (0x00000001U)

#define CSL_SPINLOCK_SYSTATUS_IU3_MASK                                         (0x00000008U)
#define CSL_SPINLOCK_SYSTATUS_IU3_SHIFT                                        (0x00000003U)
#define CSL_SPINLOCK_SYSTATUS_IU3_MAX                                          (0x00000001U)

#define CSL_SPINLOCK_SYSTATUS_IU4_MASK                                         (0x00000010U)
#define CSL_SPINLOCK_SYSTATUS_IU4_SHIFT                                        (0x00000004U)
#define CSL_SPINLOCK_SYSTATUS_IU4_MAX                                          (0x00000001U)

#define CSL_SPINLOCK_SYSTATUS_IU5_MASK                                         (0x00000020U)
#define CSL_SPINLOCK_SYSTATUS_IU5_SHIFT                                        (0x00000005U)
#define CSL_SPINLOCK_SYSTATUS_IU5_MAX                                          (0x00000001U)

#define CSL_SPINLOCK_SYSTATUS_IU6_MASK                                         (0x00000040U)
#define CSL_SPINLOCK_SYSTATUS_IU6_SHIFT                                        (0x00000006U)
#define CSL_SPINLOCK_SYSTATUS_IU6_MAX                                          (0x00000001U)

#define CSL_SPINLOCK_SYSTATUS_IU7_MASK                                         (0x00000080U)
#define CSL_SPINLOCK_SYSTATUS_IU7_SHIFT                                        (0x00000007U)
#define CSL_SPINLOCK_SYSTATUS_IU7_MAX                                          (0x00000001U)

#define CSL_SPINLOCK_SYSTATUS_NUMLOCKS_MASK                                    (0xFF000000U)
#define CSL_SPINLOCK_SYSTATUS_NUMLOCKS_SHIFT                                   (0x00000018U)
#define CSL_SPINLOCK_SYSTATUS_NUMLOCKS_MAX                                     (0x000000FFU)

/* LOCK_REG */

#define CSL_SPINLOCK_LOCK_REG_TAKEN_MASK                                       (0x00000001U)
#define CSL_SPINLOCK_LOCK_REG_TAKEN_SHIFT                                      (0x00000000U)
#define CSL_SPINLOCK_LOCK_REG_TAKEN_MAX                                        (0x00000001U)

#ifdef __cplusplus
}
#endif
#endif
