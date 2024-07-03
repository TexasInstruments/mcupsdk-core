/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *  Name        : cslr_controlss_pwmsyncoutxbar.h
*/
#ifndef CSLR_CONTROLSS_PWMSYNCOUTXBAR_H_
#define CSLR_CONTROLSS_PWMSYNCOUTXBAR_H_

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
    volatile uint32_t PID;
    volatile uint8_t  Resv_256[252];
    volatile uint32_t PWMSYNCOUTXBAR0_G0;
    volatile uint8_t  Resv_320[60];
    volatile uint32_t PWMSYNCOUTXBAR1_G0;
    volatile uint8_t  Resv_384[60];
    volatile uint32_t PWMSYNCOUTXBAR2_G0;
    volatile uint8_t  Resv_448[60];
    volatile uint32_t PWMSYNCOUTXBAR3_G0;
    volatile uint8_t  Resv_512[60];
    volatile uint32_t PWMSYNCOUTXBAR0_G1;
    volatile uint8_t  Resv_576[60];
    volatile uint32_t PWMSYNCOUTXBAR1_G1;
    volatile uint8_t  Resv_640[60];
    volatile uint32_t PWMSYNCOUTXBAR2_G1;
    volatile uint8_t  Resv_704[60];
    volatile uint32_t PWMSYNCOUTXBAR3_G1;
} CSL_controlss_pwmsyncoutxbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID                                       (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0                        (0x00000100U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G0                        (0x00000140U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G0                        (0x00000180U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G0                        (0x000001C0U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G1                        (0x00000200U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G1                        (0x00000240U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G1                        (0x00000280U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G1                        (0x000002C0U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MINOR_MASK                        (0x0000003FU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MINOR_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MINOR_RESETVAL                    (0x00000014U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MINOR_MAX                         (0x0000003FU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_CUSTOM_MASK                       (0x000000C0U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_CUSTOM_SHIFT                      (0x00000006U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_CUSTOM_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_CUSTOM_MAX                        (0x00000003U)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MAJOR_MASK                        (0x00000700U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MAJOR_SHIFT                       (0x00000008U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MAJOR_RESETVAL                    (0x00000002U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MAJOR_MAX                         (0x00000007U)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MISC_MASK                         (0x0000F800U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MISC_SHIFT                        (0x0000000BU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MISC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MISC_MAX                          (0x0000001FU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MSB16_MASK                        (0xFFFF0000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MSB16_SHIFT                       (0x00000010U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MSB16_RESETVAL                    (0x00006180U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_PID_MSB16_MAX                         (0x0000FFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PID_RESETVAL                              (0x61800214U)

/* PWMSYNCOUTXBAR0_G0 */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0_SEL_MASK               (0xFFFFFFFFU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0_SEL_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0_SEL_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0_SEL_MAX                (0xFFFFFFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G0_RESETVAL               (0x00000000U)

/* PWMSYNCOUTXBAR1_G0 */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G0_SEL_MASK               (0xFFFFFFFFU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G0_SEL_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G0_SEL_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G0_SEL_MAX                (0xFFFFFFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G0_RESETVAL               (0x00000000U)

/* PWMSYNCOUTXBAR2_G0 */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G0_SEL_MASK               (0xFFFFFFFFU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G0_SEL_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G0_SEL_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G0_SEL_MAX                (0xFFFFFFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G0_RESETVAL               (0x00000000U)

/* PWMSYNCOUTXBAR3_G0 */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G0_SEL_MASK               (0xFFFFFFFFU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G0_SEL_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G0_SEL_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G0_SEL_MAX                (0xFFFFFFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G0_RESETVAL               (0x00000000U)

/* PWMSYNCOUTXBAR0_G1 */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G1_SEL_MASK               (0x0000FFFFU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G1_SEL_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G1_SEL_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G1_SEL_MAX                (0x0000FFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR0_G1_RESETVAL               (0x00000000U)

/* PWMSYNCOUTXBAR1_G1 */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G1_SEL_MASK               (0x0000FFFFU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G1_SEL_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G1_SEL_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G1_SEL_MAX                (0x0000FFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR1_G1_RESETVAL               (0x00000000U)

/* PWMSYNCOUTXBAR2_G1 */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G1_SEL_MASK               (0x0000FFFFU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G1_SEL_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G1_SEL_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G1_SEL_MAX                (0x0000FFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR2_G1_RESETVAL               (0x00000000U)

/* PWMSYNCOUTXBAR3_G1 */

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G1_SEL_MASK               (0x0000FFFFU)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G1_SEL_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G1_SEL_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G1_SEL_MAX                (0x0000FFFFU)

#define CSL_CONTROLSS_PWMSYNCOUTXBAR_PWMSYNCOUTXBAR3_G1_RESETVAL               (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
