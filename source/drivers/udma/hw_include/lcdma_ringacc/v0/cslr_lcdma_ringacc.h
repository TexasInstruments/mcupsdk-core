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
 *  Name        : cslr_lcdma_ringacc.h
*/
#ifndef CSLR_LCDMA_RINGACC_H_
#define CSLR_LCDMA_RINGACC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : Ring Realtime Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_16[16];
    volatile uint32_t FDB;                       /* Realtime Ring N Forward Doorbell Register */
    volatile uint8_t  Resv_24[4];
    volatile uint32_t FOCC;                      /* Realtime Ring N Forward Occupancy Register */
    volatile uint8_t  Resv_4112[4084];
    volatile uint32_t RDB;                       /* Realtime Ring N Reverse Doorbell Register */
    volatile uint8_t  Resv_4120[4];
    volatile uint32_t ROCC;                      /* Realtime Ring N Reverse Occupancy Register */
    volatile uint8_t  Resv_8192[4068];
} CSL_lcdma_ringacc_ringrtRegs_ring;


typedef struct {
    CSL_lcdma_ringacc_ringrtRegs_ring RING[288];
} CSL_lcdma_ringacc_ringrtRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_LCDMA_RINGACC_RINGRT_RING_FDB(RING)                                (0x00000010U+((RING)*0x2000U))
#define CSL_LCDMA_RINGACC_RINGRT_RING_FOCC(RING)                               (0x00000018U+((RING)*0x2000U))
#define CSL_LCDMA_RINGACC_RINGRT_RING_RDB(RING)                                (0x00001010U+((RING)*0x2000U))
#define CSL_LCDMA_RINGACC_RINGRT_RING_ROCC(RING)                               (0x00001018U+((RING)*0x2000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_16[16];
    volatile uint32_t FDB;                       /* Realtime Ring N Forward Doorbell Register */
    volatile uint8_t  Resv_24[4];
    volatile uint32_t FOCC;                      /* Realtime Ring N Forward Occupancy Register */
    volatile uint8_t  Resv_4112[4084];
    volatile uint32_t RDB;                       /* Realtime Ring N Reverse Doorbell Register */
    volatile uint8_t  Resv_4120[4];
    volatile uint32_t ROCC;                      /* Realtime Ring N Reverse Occupancy Register */
    volatile uint8_t  Resv_8192[4068];
} CSL_lcdma_ringacc_ringrtRegs64_ring;


typedef struct {
    CSL_lcdma_ringacc_ringrtRegs64_ring RING[288];
} CSL_lcdma_ringacc_ringrtRegs64;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_LCDMA_RINGACC_RINGRT_64_RING_FDB(RING)                             (0x00000010U+((RING)*0x2000U))
#define CSL_LCDMA_RINGACC_RINGRT_64_RING_FOCC(RING)                            (0x00000018U+((RING)*0x2000U))
#define CSL_LCDMA_RINGACC_RINGRT_64_RING_RDB(RING)                             (0x00001010U+((RING)*0x2000U))
#define CSL_LCDMA_RINGACC_RINGRT_64_RING_ROCC(RING)                            (0x00001018U+((RING)*0x2000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* FDB */

#define CSL_LCDMA_RINGACC_RINGRT_RING_FDB_CNT_MASK                             (0x000000FFU)
#define CSL_LCDMA_RINGACC_RINGRT_RING_FDB_CNT_SHIFT                            (0x00000000U)
#define CSL_LCDMA_RINGACC_RINGRT_RING_FDB_CNT_MAX                              (0x000000FFU)


/* FOCC */

#define CSL_LCDMA_RINGACC_RINGRT_RING_FOCC_CNT_MASK                            (0x001FFFFFU)
#define CSL_LCDMA_RINGACC_RINGRT_RING_FOCC_CNT_SHIFT                           (0x00000000U)
#define CSL_LCDMA_RINGACC_RINGRT_RING_FOCC_CNT_MAX                             (0x001FFFFFU)


/* RDB */

#define CSL_LCDMA_RINGACC_RINGRT_RING_RDB_ACK_MASK                             (0x80000000U)
#define CSL_LCDMA_RINGACC_RINGRT_RING_RDB_ACK_SHIFT                            (0x0000001FU)
#define CSL_LCDMA_RINGACC_RINGRT_RING_RDB_ACK_MAX                              (0x00000001U)

#define CSL_LCDMA_RINGACC_RINGRT_RING_RDB_CNT_MASK                             (0x000000FFU)
#define CSL_LCDMA_RINGACC_RINGRT_RING_RDB_CNT_SHIFT                            (0x00000000U)
#define CSL_LCDMA_RINGACC_RINGRT_RING_RDB_CNT_MAX                              (0x000000FFU)


/* ROCC */

#define CSL_LCDMA_RINGACC_RINGRT_RING_ROCC_COMP_MASK                           (0x80000000U)
#define CSL_LCDMA_RINGACC_RINGRT_RING_ROCC_COMP_SHIFT                          (0x0000001FU)
#define CSL_LCDMA_RINGACC_RINGRT_RING_ROCC_COMP_MAX                            (0x00000001U)

#define CSL_LCDMA_RINGACC_RINGRT_RING_ROCC_CNT_MASK                            (0x001FFFFFU)
#define CSL_LCDMA_RINGACC_RINGRT_RING_ROCC_CNT_SHIFT                           (0x00000000U)
#define CSL_LCDMA_RINGACC_RINGRT_RING_ROCC_CNT_MAX                             (0x001FFFFFU)


/**************************************************************************
* Hardware Region  : PKTDMA Tx Ring Control / Status Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_64[64];
    volatile uint32_t BA_LO;                     /* Ring Base Address Lo Register */
    volatile uint32_t BA_HI;                     /* Ring Base Address Hi Register */
    volatile uint32_t SIZE;                      /* Ring Size Register */
    volatile uint8_t  Resv_256[180];
} CSL_lcdma_ringacc_ring_cfgRegs_RING;


typedef struct {
    CSL_lcdma_ringacc_ring_cfgRegs_RING RING[288];
} CSL_lcdma_ringacc_ring_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_LO(RING)                            (0x00000040U+((RING)*0x100U))
#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_HI(RING)                            (0x00000044U+((RING)*0x100U))
#define CSL_LCDMA_RINGACC_RING_CFG_RING_SIZE(RING)                             (0x00000048U+((RING)*0x100U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* BA_LO */

#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_LO_ADDR_LO_MASK                     (0xFFFFFFFFU)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_LO_ADDR_LO_SHIFT                    (0x00000000U)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_LO_ADDR_LO_MAX                      (0xFFFFFFFFU)


/* BA_HI */

#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_HI_ASEL_MASK                        (0x000F0000U)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_HI_ASEL_SHIFT                       (0x00000010U)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_HI_ASEL_MAX                         (0x0000000FU)

#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_HI_ADDR_HI_MASK                     (0x0000000FU)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_HI_ADDR_HI_SHIFT                    (0x00000000U)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_BA_HI_ADDR_HI_MAX                      (0x0000000FU)


/* SIZE */

#define CSL_LCDMA_RINGACC_RING_CFG_RING_SIZE_QMODE_MASK                        (0xE0000000U)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_SIZE_QMODE_SHIFT                       (0x0000001DU)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_SIZE_QMODE_MAX                         (0x00000007U)

#define CSL_LCDMA_RINGACC_RING_CFG_RING_SIZE_QMODE_VAL_DRING_MODE              (0x1U)

#define CSL_LCDMA_RINGACC_RING_CFG_RING_SIZE_ELCNT_MASK                        (0x0000FFFFU)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_SIZE_ELCNT_SHIFT                       (0x00000000U)
#define CSL_LCDMA_RINGACC_RING_CFG_RING_SIZE_ELCNT_MAX                         (0x0000FFFFU)


/**************************************************************************
* Hardware Region  : Ring Credentials Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CRED;                      /* Credentials Register */
    volatile uint8_t  Resv_16[12];
} CSL_lcdma_ringacc_credRegs_ring;


typedef struct {
    CSL_lcdma_ringacc_credRegs_ring RING[288];
} CSL_lcdma_ringacc_credRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_LCDMA_RINGACC_CRED_RING_CRED(RING)                                 (0x00000000U+((RING)*0x10U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CRED;                      /* Credentials Register */
    volatile uint8_t  Resv_16[12];
} CSL_lcdma_ringacc_credRegs64_ring;


typedef struct {
    CSL_lcdma_ringacc_credRegs64_ring RING[288];
} CSL_lcdma_ringacc_credRegs64;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_LCDMA_RINGACC_CRED_64_RING_CRED(RING)                              (0x00000000U+((RING)*0x10U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CRED */

#define CSL_LCDMA_RINGACC_CRED_RING_CRED_CHK_SECURE_MASK                       (0x80000000U)
#define CSL_LCDMA_RINGACC_CRED_RING_CRED_CHK_SECURE_SHIFT                      (0x0000001FU)
#define CSL_LCDMA_RINGACC_CRED_RING_CRED_CHK_SECURE_MAX                        (0x00000001U)

#define CSL_LCDMA_RINGACC_CRED_RING_CRED_SECURE_MASK                           (0x04000000U)
#define CSL_LCDMA_RINGACC_CRED_RING_CRED_SECURE_SHIFT                          (0x0000001AU)
#define CSL_LCDMA_RINGACC_CRED_RING_CRED_SECURE_MAX                            (0x00000001U)

#define CSL_LCDMA_RINGACC_CRED_RING_CRED_PRIV_MASK                             (0x03000000U)
#define CSL_LCDMA_RINGACC_CRED_RING_CRED_PRIV_SHIFT                            (0x00000018U)
#define CSL_LCDMA_RINGACC_CRED_RING_CRED_PRIV_MAX                              (0x00000003U)

#define CSL_LCDMA_RINGACC_CRED_RING_CRED_PRIVID_MASK                           (0x00FF0000U)
#define CSL_LCDMA_RINGACC_CRED_RING_CRED_PRIVID_SHIFT                          (0x00000010U)
#define CSL_LCDMA_RINGACC_CRED_RING_CRED_PRIVID_MAX                            (0x000000FFU)


#ifdef __cplusplus
}
#endif
#endif
