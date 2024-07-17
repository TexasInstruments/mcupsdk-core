/********************************************************************
 * Copyright (C) 2024 Texas Instruments Incorporated.
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
 *  Name        : cslr_fss.h
*/
#ifndef CSLR_FSS_H_
#define CSLR_FSS_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr_soc.h>
#include <stdint.h>

#define HWREG(x)                                                               \
        (*((volatile uint32_t *)(x)))

#define HWREGH(x)                                                               \
        (*((volatile uint16_t *)(x)))

#define HWREGB(x)                                                               \
        (*((volatile uint8_t *)(x)))

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_FSS_DAT_REG0_REGS_BASE                                             (0x60000000U)
#define CSL_FSS_DAT_REG1_REGS_BASE                                             (0x80000000U)
#define CSL_FSS_DAT_REG3_REGS_BASE                                             (0x88000000U)
#define CSL_FSS_ECC_AGGR_REGS_BASE                                             (0x5380F000U)
#define CSL_FSS_FOTA_GENREGS_REGS_BASE                                         (0x5380B000U)
#define CSL_FSS_FSAS_GENREGS_REGS_BASE                                         (0x53801000U)
#define CSL_FSS_FSAS_OTFA_REGS_REGS_BASE                                       (0x53802000U)
#define CSL_FSS_GENREGS_REGS_BASE                                              (0x53800000U)
#define CSL_FSS_IMEM_GENREGS_REGS_BASE                                         (0x5380D000U)
#define CSL_FSS_OSPI_CFG_REGS_BASE                                             (0x53806000U)
#define CSL_FSS_OSPI_FLASH_CFG_REGS_BASE                                       (0x53808000U)
#define CSL_FSS_PDMEM_GENREGS_REGS_BASE                                        (0x5380C000U)
#define CSL_FSS_WBUF_GENREGS_REGS_BASE                                         (0x5380E000U)


/**************************************************************************
* Hardware Region  : Global Control Registers, offset 0x00000000
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t EOI;                       /* IRQ_EOI */
    volatile uint32_t STATUS_RAW;                /* IRQ_STATUS_RAW */
    volatile uint32_t STATUS;                    /* IRQ_STATUS */
    volatile uint32_t ENABLE_SET;                /* IRQ_ENABLE_SET */
    volatile uint32_t ENABLE_CLR;                /* IRQ_ENABLE_CLR */
} CSL_fss_fsas_genregsRegs_IRQ;


typedef struct {
    volatile uint32_t ECC_RGSTRT;                /* RegionStart */
    volatile uint32_t ECC_RGSIZ;                 /* ECC RegionSize */
} CSL_fss_fsas_genregsRegs_ECC_REGCTRL;


typedef struct {
    volatile uint32_t ECC_BLOCK_ADR;             /* ERR_ECC_BLOCK_ADR */
    volatile uint32_t ECC_TYPE;                  /* ERR_ECC_TYPE */
    volatile uint32_t WRT_TYPE;                  /* ERR_WRT_TYPE */
    volatile uint8_t  Resv_16[4];
} CSL_fss_fsas_genregsRegs_ERR;


typedef struct {
    volatile uint32_t REVISION;                  /* REVISION */
    volatile uint32_t SYSCONFIG;                 /* SYSCONFIG */
    volatile uint8_t  Resv_16[8];
    CSL_fss_fsas_genregsRegs_IRQ IRQ;
    volatile uint8_t  Resv_48[12];
    CSL_fss_fsas_genregsRegs_ECC_REGCTRL ECC_REGCTRL[4];
    volatile uint8_t  Resv_112[32];
    CSL_fss_fsas_genregsRegs_ERR ERR;
    volatile uint32_t DP_ERR_INJ_CTRL;           /* Double pump error injection control */
} CSL_fss_fsas_genregsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSS_FSAS_GENREGS_REVISION                                          (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG                                         (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_EOI                                           (0x00000010U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW                                    (0x00000014U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS                                        (0x00000018U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET                                    (0x0000001CU)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR                                    (0x00000020U)
#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT(ECC_REGCTRL)               (0x00000030U+((ECC_REGCTRL)*0x8U))
#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ(ECC_REGCTRL)                (0x00000034U+((ECC_REGCTRL)*0x8U))
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_BLOCK_ADR                                 (0x00000070U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE                                      (0x00000074U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE                                      (0x00000078U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL                                   (0x00000080U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* EOI */

#define CSL_FSS_FSAS_GENREGS_IRQ_EOI_EOI_VECTOR_MASK                           (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_EOI_EOI_VECTOR_SHIFT                          (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_EOI_EOI_VECTOR_RESETVAL                       (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_EOI_EOI_VECTOR_MAX                            (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_EOI_RESETVAL                                  (0x00000000U)

/* STATUS_RAW */

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_DP_RET_ERROR_MASK                  (0x00000010U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_DP_RET_ERROR_SHIFT                 (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_DP_RET_ERROR_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_DP_RET_ERROR_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_DP_CMD_ERROR_MASK                  (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_DP_CMD_ERROR_SHIFT                 (0x00000003U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_DP_CMD_ERROR_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_DP_CMD_ERROR_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_WRITE_NONALIGN_MASK            (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_WRITE_NONALIGN_SHIFT           (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_WRITE_NONALIGN_RESETVAL        (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_WRITE_NONALIGN_MAX             (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_ERROR_2BIT_MASK                (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_ERROR_2BIT_SHIFT               (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_ERROR_2BIT_RESETVAL            (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_ERROR_2BIT_MAX                 (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_ERROR_1BIT_MASK                (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_ERROR_1BIT_SHIFT               (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_ERROR_1BIT_RESETVAL            (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_ECC_ERROR_1BIT_MAX                 (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RAW_RESETVAL                           (0x00000000U)

/* STATUS */

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_DP_RET_ERROR_MASK                      (0x00000010U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_DP_RET_ERROR_SHIFT                     (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_DP_RET_ERROR_RESETVAL                  (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_DP_RET_ERROR_MAX                       (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_DP_CMD_ERROR_MASK                      (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_DP_CMD_ERROR_SHIFT                     (0x00000003U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_DP_CMD_ERROR_RESETVAL                  (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_DP_CMD_ERROR_MAX                       (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_WRITE_NONALIGN_MASK                (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_WRITE_NONALIGN_SHIFT               (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_WRITE_NONALIGN_RESETVAL            (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_WRITE_NONALIGN_MAX                 (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_ERROR_2BIT_MASK                    (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_ERROR_2BIT_SHIFT                   (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_ERROR_2BIT_RESETVAL                (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_ERROR_2BIT_MAX                     (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_ERROR_1BIT_MASK                    (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_ERROR_1BIT_SHIFT                   (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_ERROR_1BIT_RESETVAL                (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_ECC_ERROR_1BIT_MAX                     (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_STATUS_RESETVAL                               (0x00000000U)

/* ENABLE_SET */

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_DP_RET_ERROR_MASK                  (0x00000010U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_DP_RET_ERROR_SHIFT                 (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_DP_RET_ERROR_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_DP_RET_ERROR_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_DP_CMD_ERROR_MASK                  (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_DP_CMD_ERROR_SHIFT                 (0x00000003U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_DP_CMD_ERROR_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_DP_CMD_ERROR_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_WRITE_NONALIGN_MASK            (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_WRITE_NONALIGN_SHIFT           (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_WRITE_NONALIGN_RESETVAL        (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_WRITE_NONALIGN_MAX             (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_ERROR_2BIT_MASK                (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_ERROR_2BIT_SHIFT               (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_ERROR_2BIT_RESETVAL            (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_ERROR_2BIT_MAX                 (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_ERROR_1BIT_MASK                (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_ERROR_1BIT_SHIFT               (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_ERROR_1BIT_RESETVAL            (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_ECC_ERROR_1BIT_MAX                 (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_SET_RESETVAL                           (0x00000000U)

/* ENABLE_CLR */

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_DP_RET_ERROR_MASK                  (0x00000010U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_DP_RET_ERROR_SHIFT                 (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_DP_RET_ERROR_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_DP_RET_ERROR_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_DP_CMD_ERROR_MASK                  (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_DP_CMD_ERROR_SHIFT                 (0x00000003U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_DP_CMD_ERROR_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_DP_CMD_ERROR_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_WRITE_NONALIGN_MASK            (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_WRITE_NONALIGN_SHIFT           (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_WRITE_NONALIGN_RESETVAL        (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_WRITE_NONALIGN_MAX             (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_ERROR_2BIT_MASK                (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_ERROR_2BIT_SHIFT               (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_ERROR_2BIT_RESETVAL            (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_ERROR_2BIT_MAX                 (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_ERROR_1BIT_MASK                (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_ERROR_1BIT_SHIFT               (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_ERROR_1BIT_RESETVAL            (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_ECC_ERROR_1BIT_MAX                 (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_IRQ_ENABLE_CLR_RESETVAL                           (0x00000000U)

/* ECC_RGSTRT */

#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT_R_START_MASK               (0x000FFFFFU)
#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT_R_START_SHIFT              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT_R_START_RESETVAL           (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT_R_START_MAX                (0x000FFFFFU)

#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSTRT_RESETVAL                   (0x00000000U)

/* ECC_RGSIZ */

#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ_R_SIZE_MASK                 (0x000FFFFFU)
#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ_R_SIZE_SHIFT                (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ_R_SIZE_RESETVAL             (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ_R_SIZE_MAX                  (0x000FFFFFU)

#define CSL_FSS_FSAS_GENREGS_ECC_REGCTRL_ECC_RGSIZ_RESETVAL                    (0x00000000U)

/* ECC_BLOCK_ADR */

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_BLOCK_ADR_ECC_ERROR_BLOCK_ADDR_MASK       (0xFFFFFFE0U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_BLOCK_ADR_ECC_ERROR_BLOCK_ADDR_SHIFT      (0x00000005U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_BLOCK_ADR_ECC_ERROR_BLOCK_ADDR_RESETVAL   (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_BLOCK_ADR_ECC_ERROR_BLOCK_ADDR_MAX        (0x07FFFFFFU)

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_BLOCK_ADR_RESETVAL                        (0x00000000U)

/* ECC_TYPE */

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_VALID_MASK                   (0x80000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_VALID_SHIFT                  (0x0000001FU)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_VALID_RESETVAL               (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_VALID_MAX                    (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_ADR_MASK                     (0x00000020U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_ADR_SHIFT                    (0x00000005U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_ADR_RESETVAL                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_ADR_MAX                      (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_MAC_MASK                     (0x00000010U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_MAC_SHIFT                    (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_MAC_RESETVAL                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_MAC_MAX                      (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DA1_MASK                     (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DA1_SHIFT                    (0x00000003U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DA1_RESETVAL                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DA1_MAX                      (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DA0_MASK                     (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DA0_SHIFT                    (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DA0_RESETVAL                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DA0_MAX                      (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DED_MASK                     (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DED_SHIFT                    (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DED_RESETVAL                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_DED_MAX                      (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_SEC_MASK                     (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_SEC_SHIFT                    (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_SEC_RESETVAL                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_ECC_ERR_SEC_MAX                      (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_ECC_TYPE_RESETVAL                             (0x00000000U)

/* WRT_TYPE */

#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_VALID_MASK                   (0x80000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_VALID_SHIFT                  (0x0000001FU)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_VALID_RESETVAL               (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_VALID_MAX                    (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_ADR_MASK                     (0x00001000U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_ADR_SHIFT                    (0x0000000CU)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_ADR_RESETVAL                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_ADR_MAX                      (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_BEN_MASK                     (0x00002000U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_BEN_SHIFT                    (0x0000000DU)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_BEN_RESETVAL                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_BEN_MAX                      (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_ROUTEID_MASK                 (0x00000FFFU)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_ROUTEID_SHIFT                (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_ROUTEID_RESETVAL             (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_WRT_ERR_ROUTEID_MAX                  (0x00000FFFU)

#define CSL_FSS_FSAS_GENREGS_ERR_WRT_TYPE_RESETVAL                             (0x00000000U)

/* REVISION */

#define CSL_FSS_FSAS_GENREGS_REVISION_MODID_MASK                               (0xFFFF0000U)
#define CSL_FSS_FSAS_GENREGS_REVISION_MODID_SHIFT                              (0x00000010U)
#define CSL_FSS_FSAS_GENREGS_REVISION_MODID_RESETVAL                           (0x00006850U)
#define CSL_FSS_FSAS_GENREGS_REVISION_MODID_MAX                                (0x0000FFFFU)

#define CSL_FSS_FSAS_GENREGS_REVISION_REVRTL_MASK                              (0x0000F800U)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVRTL_SHIFT                             (0x0000000BU)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVRTL_RESETVAL                          (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVRTL_MAX                               (0x0000001FU)

#define CSL_FSS_FSAS_GENREGS_REVISION_REVMAJ_MASK                              (0x00000700U)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVMAJ_SHIFT                             (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVMAJ_RESETVAL                          (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVMAJ_MAX                               (0x00000007U)

#define CSL_FSS_FSAS_GENREGS_REVISION_CUSTOM_MASK                              (0x000000C0U)
#define CSL_FSS_FSAS_GENREGS_REVISION_CUSTOM_SHIFT                             (0x00000006U)
#define CSL_FSS_FSAS_GENREGS_REVISION_CUSTOM_RESETVAL                          (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_REVISION_CUSTOM_MAX                               (0x00000003U)

#define CSL_FSS_FSAS_GENREGS_REVISION_REVMIN_MASK                              (0x0000003FU)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVMIN_SHIFT                             (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVMIN_RESETVAL                          (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_REVISION_REVMIN_MAX                               (0x0000003FU)

#define CSL_FSS_FSAS_GENREGS_REVISION_RESETVAL                                 (0x68501200U)

/* SYSCONFIG */

#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_DP_EN_MASK                              (0x00000200U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_DP_EN_SHIFT                             (0x00000009U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_DP_EN_RESETVAL                          (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_DP_EN_MAX                               (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_DISXIP_MASK                             (0x00000080U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_DISXIP_SHIFT                            (0x00000007U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_DISXIP_RESETVAL                         (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_DISXIP_MAX                              (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_OSPI_32B_DISABLE_MODE_MASK              (0x00000100U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_OSPI_32B_DISABLE_MODE_SHIFT             (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_OSPI_32B_DISABLE_MODE_RESETVAL          (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_OSPI_32B_DISABLE_MODE_MAX               (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_OSPI_DDR_DISABLE_MODE_MASK              (0x00000040U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_OSPI_DDR_DISABLE_MODE_SHIFT             (0x00000006U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_OSPI_DDR_DISABLE_MODE_RESETVAL          (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_OSPI_DDR_DISABLE_MODE_MAX               (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_DISABLE_ADR_MASK                    (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_DISABLE_ADR_SHIFT                   (0x00000003U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_DISABLE_ADR_RESETVAL                (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_DISABLE_ADR_MAX                     (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_FSS_AES_EN_IPCFG_MASK                   (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_FSS_AES_EN_IPCFG_SHIFT                  (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_FSS_AES_EN_IPCFG_RESETVAL               (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_FSS_AES_EN_IPCFG_MAX                    (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_EN_MASK                             (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_EN_SHIFT                            (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_EN_RESETVAL                         (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_ECC_EN_MAX                              (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_SYSCONFIG_RESETVAL                                (0x00000000U)

/* DP_ERR_INJ_CTRL */

#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_BIT_SEL_MASK                      (0x0000FF00U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_BIT_SEL_SHIFT                     (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_BIT_SEL_RESETVAL                  (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_BIT_SEL_MAX                       (0x000000FFU)

#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_RET_NFIRST_LAST_MASK              (0x00000010U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_RET_NFIRST_LAST_SHIFT             (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_RET_NFIRST_LAST_RESETVAL          (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_RET_NFIRST_LAST_MAX               (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_DUPL_RET_MASK                  (0x00000008U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_DUPL_RET_SHIFT                 (0x00000003U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_DUPL_RET_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_DUPL_RET_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_ORIG_RET_MASK                  (0x00000004U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_ORIG_RET_SHIFT                 (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_ORIG_RET_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_ORIG_RET_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_DUPL_CMD_MASK                  (0x00000002U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_DUPL_CMD_SHIFT                 (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_DUPL_CMD_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_DUPL_CMD_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_ORIG_CMD_MASK                  (0x00000001U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_ORIG_CMD_SHIFT                 (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_ORIG_CMD_RESETVAL              (0x00000000U)
#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_EN_ORIG_CMD_MAX                   (0x00000001U)

#define CSL_FSS_FSAS_GENREGS_DP_ERR_INJ_CTRL_RESETVAL                          (0x00000000U)

/**************************************************************************
* Hardware Region  : OTFA Control Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t RGCFG;
    volatile uint32_t RGMACST;
    volatile uint32_t RGST;
    volatile uint32_t RGSI;
    volatile uint32_t RKEYE[8];
    volatile uint32_t RKEYEP[8];
    volatile uint32_t RKEYA[4];
    volatile uint32_t RKEYAP[4];
    volatile uint32_t RIV[4];
} CSL_fsas_otfa_regsRegs_rg;


typedef struct {
    volatile uint32_t REVID;
    volatile uint32_t SCFG;
    volatile uint32_t ISR;
    volatile uint32_t IS;
    volatile uint32_t IES;
    volatile uint32_t IEC;
    volatile uint32_t CCFG;
    volatile uint32_t CSTATUS;
    CSL_fsas_otfa_regsRegs_rg RG[4];
    volatile uint32_t IRQADDINFO0;
    volatile uint32_t IRQADDINFO1;
    volatile uint32_t MACCACHEINFO;
    volatile uint32_t RMWRMCNT;
} CSL_fsas_otfa_regsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSAS_OTFA_REGS_REVID                                               (0x00000000U)
#define CSL_FSAS_OTFA_REGS_SCFG                                                (0x00000004U)
#define CSL_FSAS_OTFA_REGS_ISR                                                 (0x00000008U)
#define CSL_FSAS_OTFA_REGS_IS                                                  (0x0000000CU)
#define CSL_FSAS_OTFA_REGS_IES                                                 (0x00000010U)
#define CSL_FSAS_OTFA_REGS_IEC                                                 (0x00000014U)
#define CSL_FSAS_OTFA_REGS_CCFG                                                (0x00000018U)
#define CSL_FSAS_OTFA_REGS_CSTATUS                                             (0x0000001CU)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG(RG)                                        (0x00000020U+((RG)*0x80U))
#define CSL_FSAS_OTFA_REGS_RG_RGMACST(RG)                                      (0x00000024U+((RG)*0x80U))
#define CSL_FSAS_OTFA_REGS_RG_RGST(RG)                                         (0x00000028U+((RG)*0x80U))
#define CSL_FSAS_OTFA_REGS_RG_RGSI(RG)                                         (0x0000002CU+((RG)*0x80U))
#define CSL_FSAS_OTFA_REGS_RG_RKEYE(RG,RKEYE)                                  (0x00000030U+((RG)*0x80U)+((RKEYE)*0x4U))
#define CSL_FSAS_OTFA_REGS_RG_RKEYEP(RG,RKEYEP)                                (0x00000050U+((RG)*0x80U)+((RKEYEP)*0x4U))
#define CSL_FSAS_OTFA_REGS_RG_RKEYA(RG,RKEYA)                                  (0x00000070U+((RG)*0x80U)+((RKEYA)*0x4U))
#define CSL_FSAS_OTFA_REGS_RG_RKEYAP(RG,RKEYAP)                                (0x00000080U+((RG)*0x80U)+((RKEYAP)*0x4U))
#define CSL_FSAS_OTFA_REGS_RG_RIV(RG,RIV)                                      (0x00000090U+((RG)*0x80U)+((RIV)*0x4U))
#define CSL_FSAS_OTFA_REGS_IRQADDINFO0                                         (0x00000220U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1                                         (0x00000224U)
#define CSL_FSAS_OTFA_REGS_MACCACHEINFO                                        (0x00000228U)
#define CSL_FSAS_OTFA_REGS_RMWRMCNT                                            (0x0000022CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RGCFG */

#define CSL_FSAS_OTFA_REGS_RG_RGCFG_AES_MODE_MASK                              (0x00000003U)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_AES_MODE_SHIFT                             (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_AES_MODE_RESETVAL                          (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_AES_MODE_MAX                               (0x00000003U)

#define CSL_FSAS_OTFA_REGS_RG_RGCFG_MAC_MODE_MASK                              (0x0000000CU)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_MAC_MODE_SHIFT                             (0x00000002U)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_MAC_MODE_RESETVAL                          (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_MAC_MODE_MAX                               (0x00000003U)

#define CSL_FSAS_OTFA_REGS_RG_RGCFG_WRT_PROTECT_MASK                           (0x00000010U)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_WRT_PROTECT_SHIFT                          (0x00000004U)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_WRT_PROTECT_RESETVAL                       (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGCFG_WRT_PROTECT_MAX                            (0x00000001U)

#define CSL_FSAS_OTFA_REGS_RG_RGCFG_RESETVAL                                   (0x00000000U)

/* RGMACST */

#define CSL_FSAS_OTFA_REGS_RG_RGMACST_M_START_MASK                             (0x000FFFFFU)
#define CSL_FSAS_OTFA_REGS_RG_RGMACST_M_START_SHIFT                            (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGMACST_M_START_RESETVAL                         (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGMACST_M_START_MAX                              (0x000FFFFFU)

#define CSL_FSAS_OTFA_REGS_RG_RGMACST_RESETVAL                                 (0x00000000U)

/* RGST */

#define CSL_FSAS_OTFA_REGS_RG_RGST_R_START_MASK                                (0x000FFFFFU)
#define CSL_FSAS_OTFA_REGS_RG_RGST_R_START_SHIFT                               (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGST_R_START_RESETVAL                            (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGST_R_START_MAX                                 (0x000FFFFFU)

#define CSL_FSAS_OTFA_REGS_RG_RGST_RESETVAL                                    (0x00000000U)

/* RGSI */

#define CSL_FSAS_OTFA_REGS_RG_RGSI_R_SIZE_MASK                                 (0x000FFFFFU)
#define CSL_FSAS_OTFA_REGS_RG_RGSI_R_SIZE_SHIFT                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGSI_R_SIZE_RESETVAL                             (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RGSI_R_SIZE_MAX                                  (0x000FFFFFU)

#define CSL_FSAS_OTFA_REGS_RG_RGSI_RESETVAL                                    (0x00000000U)

/* RKEYE */

#define CSL_FSAS_OTFA_REGS_RG_RKEYE_R_KEY_E_MASK                               (0xFFFFFFFFU)
#define CSL_FSAS_OTFA_REGS_RG_RKEYE_R_KEY_E_SHIFT                              (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RKEYE_R_KEY_E_RESETVAL                           (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RKEYE_R_KEY_E_MAX                                (0xFFFFFFFFU)

#define CSL_FSAS_OTFA_REGS_RG_RKEYE_RESETVAL                                   (0x00000000U)

/* RKEYEP */

#define CSL_FSAS_OTFA_REGS_RG_RKEYEP_R_KEY_EP_MASK                             (0xFFFFFFFFU)
#define CSL_FSAS_OTFA_REGS_RG_RKEYEP_R_KEY_EP_SHIFT                            (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RKEYEP_R_KEY_EP_RESETVAL                         (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RKEYEP_R_KEY_EP_MAX                              (0xFFFFFFFFU)

#define CSL_FSAS_OTFA_REGS_RG_RKEYEP_RESETVAL                                  (0x00000000U)

/* RKEYA */

#define CSL_FSAS_OTFA_REGS_RG_RKEYA_R_KEY_A_MASK                               (0xFFFFFFFFU)
#define CSL_FSAS_OTFA_REGS_RG_RKEYA_R_KEY_A_SHIFT                              (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RKEYA_R_KEY_A_RESETVAL                           (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RKEYA_R_KEY_A_MAX                                (0xFFFFFFFFU)

#define CSL_FSAS_OTFA_REGS_RG_RKEYA_RESETVAL                                   (0x00000000U)

/* RKEYAP */

#define CSL_FSAS_OTFA_REGS_RG_RKEYAP_R_KEY_AP_MASK                             (0xFFFFFFFFU)
#define CSL_FSAS_OTFA_REGS_RG_RKEYAP_R_KEY_AP_SHIFT                            (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RKEYAP_R_KEY_AP_RESETVAL                         (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RKEYAP_R_KEY_AP_MAX                              (0xFFFFFFFFU)

#define CSL_FSAS_OTFA_REGS_RG_RKEYAP_RESETVAL                                  (0x00000000U)

/* RIV */

#define CSL_FSAS_OTFA_REGS_RG_RIV_R_IV_MASK                                    (0xFFFFFFFFU)
#define CSL_FSAS_OTFA_REGS_RG_RIV_R_IV_SHIFT                                   (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RIV_R_IV_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RG_RIV_R_IV_MAX                                     (0xFFFFFFFFU)

#define CSL_FSAS_OTFA_REGS_RG_RIV_RESETVAL                                     (0x00000000U)

/* REVID */

#define CSL_FSAS_OTFA_REGS_REVID_REVID_MASK                                    (0xFFFFFFFFU)
#define CSL_FSAS_OTFA_REGS_REVID_REVID_SHIFT                                   (0x00000000U)
#define CSL_FSAS_OTFA_REGS_REVID_REVID_RESETVAL                                (0x47500101U)
#define CSL_FSAS_OTFA_REGS_REVID_REVID_MAX                                     (0xFFFFFFFFU)

#define CSL_FSAS_OTFA_REGS_REVID_RESETVAL                                      (0x47500101U)

/* SCFG */

#define CSL_FSAS_OTFA_REGS_SCFG_IDLE_MODE_MASK                                 (0x00000003U)
#define CSL_FSAS_OTFA_REGS_SCFG_IDLE_MODE_SHIFT                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_SCFG_IDLE_MODE_RESETVAL                             (0x00000002U)
#define CSL_FSAS_OTFA_REGS_SCFG_IDLE_MODE_MAX                                  (0x00000003U)

#define CSL_FSAS_OTFA_REGS_SCFG_RESETVAL                                       (0x00000002U)

/* ISR */

#define CSL_FSAS_OTFA_REGS_ISR_CTR_WKV_MASK                                    (0x0000000FU)
#define CSL_FSAS_OTFA_REGS_ISR_CTR_WKV_SHIFT                                   (0x00000000U)
#define CSL_FSAS_OTFA_REGS_ISR_CTR_WKV_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_ISR_CTR_WKV_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_ISR_REGION_BV_MASK                                  (0x000000F0U)
#define CSL_FSAS_OTFA_REGS_ISR_REGION_BV_SHIFT                                 (0x00000004U)
#define CSL_FSAS_OTFA_REGS_ISR_REGION_BV_RESETVAL                              (0x00000000U)
#define CSL_FSAS_OTFA_REGS_ISR_REGION_BV_MAX                                   (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_ISR_WRT_ERR_MASK                                    (0x00000F00U)
#define CSL_FSAS_OTFA_REGS_ISR_WRT_ERR_SHIFT                                   (0x00000008U)
#define CSL_FSAS_OTFA_REGS_ISR_WRT_ERR_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_ISR_WRT_ERR_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_ISR_MAC_ERR_MASK                                    (0x0000F000U)
#define CSL_FSAS_OTFA_REGS_ISR_MAC_ERR_SHIFT                                   (0x0000000CU)
#define CSL_FSAS_OTFA_REGS_ISR_MAC_ERR_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_ISR_MAC_ERR_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_ISR_RESETVAL                                        (0x00000000U)

/* IS */

#define CSL_FSAS_OTFA_REGS_IS_CTR_WKV_MASK                                     (0x0000000FU)
#define CSL_FSAS_OTFA_REGS_IS_CTR_WKV_SHIFT                                    (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IS_CTR_WKV_RESETVAL                                 (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IS_CTR_WKV_MAX                                      (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IS_REGION_BV_MASK                                   (0x000000F0U)
#define CSL_FSAS_OTFA_REGS_IS_REGION_BV_SHIFT                                  (0x00000004U)
#define CSL_FSAS_OTFA_REGS_IS_REGION_BV_RESETVAL                               (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IS_REGION_BV_MAX                                    (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IS_WRT_ERR_MASK                                     (0x00000F00U)
#define CSL_FSAS_OTFA_REGS_IS_WRT_ERR_SHIFT                                    (0x00000008U)
#define CSL_FSAS_OTFA_REGS_IS_WRT_ERR_RESETVAL                                 (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IS_WRT_ERR_MAX                                      (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IS_MAC_ERR_MASK                                     (0x0000F000U)
#define CSL_FSAS_OTFA_REGS_IS_MAC_ERR_SHIFT                                    (0x0000000CU)
#define CSL_FSAS_OTFA_REGS_IS_MAC_ERR_RESETVAL                                 (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IS_MAC_ERR_MAX                                      (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IS_RESETVAL                                         (0x00000000U)

/* IES */

#define CSL_FSAS_OTFA_REGS_IES_CTR_WKV_MASK                                    (0x0000000FU)
#define CSL_FSAS_OTFA_REGS_IES_CTR_WKV_SHIFT                                   (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IES_CTR_WKV_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IES_CTR_WKV_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IES_REGION_BV_MASK                                  (0x000000F0U)
#define CSL_FSAS_OTFA_REGS_IES_REGION_BV_SHIFT                                 (0x00000004U)
#define CSL_FSAS_OTFA_REGS_IES_REGION_BV_RESETVAL                              (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IES_REGION_BV_MAX                                   (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IES_WRT_ERR_MASK                                    (0x00000F00U)
#define CSL_FSAS_OTFA_REGS_IES_WRT_ERR_SHIFT                                   (0x00000008U)
#define CSL_FSAS_OTFA_REGS_IES_WRT_ERR_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IES_WRT_ERR_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IES_MAC_ERR_MASK                                    (0x0000F000U)
#define CSL_FSAS_OTFA_REGS_IES_MAC_ERR_SHIFT                                   (0x0000000CU)
#define CSL_FSAS_OTFA_REGS_IES_MAC_ERR_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IES_MAC_ERR_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IES_RESETVAL                                        (0x00000000U)

/* IEC */

#define CSL_FSAS_OTFA_REGS_IEC_CTR_WKV_MASK                                    (0x0000000FU)
#define CSL_FSAS_OTFA_REGS_IEC_CTR_WKV_SHIFT                                   (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IEC_CTR_WKV_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IEC_CTR_WKV_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IEC_REGION_BV_MASK                                  (0x000000F0U)
#define CSL_FSAS_OTFA_REGS_IEC_REGION_BV_SHIFT                                 (0x00000004U)
#define CSL_FSAS_OTFA_REGS_IEC_REGION_BV_RESETVAL                              (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IEC_REGION_BV_MAX                                   (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IEC_WRT_ERR_MASK                                    (0x00000F00U)
#define CSL_FSAS_OTFA_REGS_IEC_WRT_ERR_SHIFT                                   (0x00000008U)
#define CSL_FSAS_OTFA_REGS_IEC_WRT_ERR_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IEC_WRT_ERR_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IEC_MAC_ERR_MASK                                    (0x0000F000U)
#define CSL_FSAS_OTFA_REGS_IEC_MAC_ERR_SHIFT                                   (0x0000000CU)
#define CSL_FSAS_OTFA_REGS_IEC_MAC_ERR_RESETVAL                                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IEC_MAC_ERR_MAX                                     (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IEC_RESETVAL                                        (0x00000000U)

/* CCFG */

#define CSL_FSAS_OTFA_REGS_CCFG_RD_WRT_OPT_MASK                                (0x0000000FU)
#define CSL_FSAS_OTFA_REGS_CCFG_RD_WRT_OPT_SHIFT                               (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_RD_WRT_OPT_RESETVAL                            (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_RD_WRT_OPT_MAX                                 (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_CCFG_KEY_SIZE_MASK                                  (0x00000010U)
#define CSL_FSAS_OTFA_REGS_CCFG_KEY_SIZE_SHIFT                                 (0x00000004U)
#define CSL_FSAS_OTFA_REGS_CCFG_KEY_SIZE_RESETVAL                              (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_KEY_SIZE_MAX                                   (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CCFG_CACHE_EVICT_MODE_MASK                          (0x00000020U)
#define CSL_FSAS_OTFA_REGS_CCFG_CACHE_EVICT_MODE_SHIFT                         (0x00000005U)
#define CSL_FSAS_OTFA_REGS_CCFG_CACHE_EVICT_MODE_RESETVAL                      (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_CACHE_EVICT_MODE_MAX                           (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CCFG_CACHE_ENABLE_MASK                              (0x00000040U)
#define CSL_FSAS_OTFA_REGS_CCFG_CACHE_ENABLE_SHIFT                             (0x00000006U)
#define CSL_FSAS_OTFA_REGS_CCFG_CACHE_ENABLE_RESETVAL                          (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_CACHE_ENABLE_MAX                               (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CCFG_OTFA_WAIT_MASK                                 (0x00000100U)
#define CSL_FSAS_OTFA_REGS_CCFG_OTFA_WAIT_SHIFT                                (0x00000008U)
#define CSL_FSAS_OTFA_REGS_CCFG_OTFA_WAIT_RESETVAL                             (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_OTFA_WAIT_MAX                                  (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CCFG_ERROR_RESP_EN_MASK                             (0x00000200U)
#define CSL_FSAS_OTFA_REGS_CCFG_ERROR_RESP_EN_SHIFT                            (0x00000009U)
#define CSL_FSAS_OTFA_REGS_CCFG_ERROR_RESP_EN_RESETVAL                         (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_ERROR_RESP_EN_MAX                              (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CCFG_MAC_SIZE_MASK                                  (0x00000C00U)
#define CSL_FSAS_OTFA_REGS_CCFG_MAC_SIZE_SHIFT                                 (0x0000000AU)
#define CSL_FSAS_OTFA_REGS_CCFG_MAC_SIZE_RESETVAL                              (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_MAC_SIZE_MAX                                   (0x00000003U)

#define CSL_FSAS_OTFA_REGS_CCFG_MAC_LSB_ALIGN_EN_MASK                          (0x00001000U)
#define CSL_FSAS_OTFA_REGS_CCFG_MAC_LSB_ALIGN_EN_SHIFT                         (0x0000000CU)
#define CSL_FSAS_OTFA_REGS_CCFG_MAC_LSB_ALIGN_EN_RESETVAL                      (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_MAC_LSB_ALIGN_EN_MAX                           (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CCFG_FE_PROC_EN_MASK                                (0x00008000U)
#define CSL_FSAS_OTFA_REGS_CCFG_FE_PROC_EN_SHIFT                               (0x0000000FU)
#define CSL_FSAS_OTFA_REGS_CCFG_FE_PROC_EN_RESETVAL                            (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_FE_PROC_EN_MAX                                 (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CCFG_MASTER_EN_RD_MASK                              (0x80000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_MASTER_EN_RD_SHIFT                             (0x0000001FU)
#define CSL_FSAS_OTFA_REGS_CCFG_MASTER_EN_RD_RESETVAL                          (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CCFG_MASTER_EN_RD_MAX                               (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CCFG_RESETVAL                                       (0x00000000U)

/* CSTATUS */

#define CSL_FSAS_OTFA_REGS_CSTATUS_WRT_STALL_EVENT_CNT_MASK                    (0x00003FFFU)
#define CSL_FSAS_OTFA_REGS_CSTATUS_WRT_STALL_EVENT_CNT_SHIFT                   (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_WRT_STALL_EVENT_CNT_RESETVAL                (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_WRT_STALL_EVENT_CNT_MAX                     (0x00003FFFU)

#define CSL_FSAS_OTFA_REGS_CSTATUS_RD_STALL_EVENT_CNT_MASK                     (0x3FFF0000U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_RD_STALL_EVENT_CNT_SHIFT                    (0x00000010U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_RD_STALL_EVENT_CNT_RESETVAL                 (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_RD_STALL_EVENT_CNT_MAX                      (0x00003FFFU)

#define CSL_FSAS_OTFA_REGS_CSTATUS_CRYPTO_BUSY_MASK                            (0x40000000U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_CRYPTO_BUSY_SHIFT                           (0x0000001EU)
#define CSL_FSAS_OTFA_REGS_CSTATUS_CRYPTO_BUSY_RESETVAL                        (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_CRYPTO_BUSY_MAX                             (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CSTATUS_BUSY_MASK                                   (0x80000000U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_BUSY_SHIFT                                  (0x0000001FU)
#define CSL_FSAS_OTFA_REGS_CSTATUS_BUSY_RESETVAL                               (0x00000000U)
#define CSL_FSAS_OTFA_REGS_CSTATUS_BUSY_MAX                                    (0x00000001U)

#define CSL_FSAS_OTFA_REGS_CSTATUS_RESETVAL                                    (0x00000000U)

/* IRQADDINFO0 */

#define CSL_FSAS_OTFA_REGS_IRQADDINFO0_IRQ_MADDR_MASK                          (0xFFFFFFFFU)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO0_IRQ_MADDR_SHIFT                         (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO0_IRQ_MADDR_RESETVAL                      (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO0_IRQ_MADDR_MAX                           (0xFFFFFFFFU)

#define CSL_FSAS_OTFA_REGS_IRQADDINFO0_RESETVAL                                (0x00000000U)

/* IRQADDINFO1 */

#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MID_MASK                            (0x000000FFU)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MID_SHIFT                           (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MID_RESETVAL                        (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MID_MAX                             (0x000000FFU)

#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MCMD_MASK                           (0x00000700U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MCMD_SHIFT                          (0x00000008U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MCMD_RESETVAL                       (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MCMD_MAX                            (0x00000007U)

#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MSEQ_MASK                           (0x00003800U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MSEQ_SHIFT                          (0x0000000BU)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MSEQ_RESETVAL                       (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MSEQ_MAX                            (0x00000007U)

#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MLEN_MASK                           (0x0003C000U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MLEN_SHIFT                          (0x0000000EU)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MLEN_RESETVAL                       (0x00000000U)
#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_IRQ_MLEN_MAX                            (0x0000000FU)

#define CSL_FSAS_OTFA_REGS_IRQADDINFO1_RESETVAL                                (0x00000000U)

/* MACCACHEINFO */

#define CSL_FSAS_OTFA_REGS_MACCACHEINFO_CACHE_MISS_EVENT_CNT_MASK              (0x0000FFFFU)
#define CSL_FSAS_OTFA_REGS_MACCACHEINFO_CACHE_MISS_EVENT_CNT_SHIFT             (0x00000000U)
#define CSL_FSAS_OTFA_REGS_MACCACHEINFO_CACHE_MISS_EVENT_CNT_RESETVAL          (0x00000000U)
#define CSL_FSAS_OTFA_REGS_MACCACHEINFO_CACHE_MISS_EVENT_CNT_MAX               (0x0000FFFFU)

#define CSL_FSAS_OTFA_REGS_MACCACHEINFO_RESETVAL                               (0x00000000U)

/* RMWRMCNT */

#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RMW_EVENT_CNT_MASK                         (0x0000FFFFU)
#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RMW_EVENT_CNT_SHIFT                        (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RMW_EVENT_CNT_RESETVAL                     (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RMW_EVENT_CNT_MAX                          (0x0000FFFFU)

#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RM_EVENT_CNT_MASK                          (0xFFFF0000U)
#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RM_EVENT_CNT_SHIFT                         (0x00000010U)
#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RM_EVENT_CNT_RESETVAL                      (0x00000000U)
#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RM_EVENT_CNT_MAX                           (0x0000FFFFU)

#define CSL_FSAS_OTFA_REGS_RMWRMCNT_RESETVAL                                   (0x00000000U)

/**************************************************************************
* Hardware Region  : Global Control Registers, offset 0x00000000
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REVISION;                  /* REVISION */
} CSL_fss_genregsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSS_GENREGS_REVISION                                               (0x00000000U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVISION */

#define CSL_FSS_GENREGS_REVISION_MODID_MASK                                    (0xFFFF0000U)
#define CSL_FSS_GENREGS_REVISION_MODID_SHIFT                                   (0x00000010U)
#define CSL_FSS_GENREGS_REVISION_MODID_RESETVAL                                (0x00006850U)
#define CSL_FSS_GENREGS_REVISION_MODID_MAX                                     (0x0000FFFFU)

#define CSL_FSS_GENREGS_REVISION_REVRTL_MASK                                   (0x0000F800U)
#define CSL_FSS_GENREGS_REVISION_REVRTL_SHIFT                                  (0x0000000BU)
#define CSL_FSS_GENREGS_REVISION_REVRTL_RESETVAL                               (0x00000002U)
#define CSL_FSS_GENREGS_REVISION_REVRTL_MAX                                    (0x0000001FU)

#define CSL_FSS_GENREGS_REVISION_REVMAJ_MASK                                   (0x00000700U)
#define CSL_FSS_GENREGS_REVISION_REVMAJ_SHIFT                                  (0x00000008U)
#define CSL_FSS_GENREGS_REVISION_REVMAJ_RESETVAL                               (0x00000002U)
#define CSL_FSS_GENREGS_REVISION_REVMAJ_MAX                                    (0x00000007U)

#define CSL_FSS_GENREGS_REVISION_CUSTOM_MASK                                   (0x000000C0U)
#define CSL_FSS_GENREGS_REVISION_CUSTOM_SHIFT                                  (0x00000006U)
#define CSL_FSS_GENREGS_REVISION_CUSTOM_RESETVAL                               (0x00000000U)
#define CSL_FSS_GENREGS_REVISION_CUSTOM_MAX                                    (0x00000003U)

#define CSL_FSS_GENREGS_REVISION_REVMIN_MASK                                   (0x0000003FU)
#define CSL_FSS_GENREGS_REVISION_REVMIN_SHIFT                                  (0x00000000U)
#define CSL_FSS_GENREGS_REVISION_REVMIN_RESETVAL                               (0x00000000U)
#define CSL_FSS_GENREGS_REVISION_REVMIN_MAX                                    (0x0000003FU)

#define CSL_FSS_GENREGS_REVISION_RESETVAL                                      (0x68501200U)

/**************************************************************************
* Hardware Region  : Firmware Over The Air (FOTA) Accelerator Registers, offset 0x00000000
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t EOI;                       /* IRQ_EOI */
    volatile uint32_t STATUS_RAW;                /* IRQ_STATUS_RAW */
    volatile uint32_t STATUS;                    /* IRQ_STATUS */
    volatile uint32_t ENABLE_SET;                /* IRQ_ENABLE_SET */
    volatile uint32_t ENABLE_CLR;                /* IRQ_ENABLE_CLR */
} CSL_fss_fota_genregsRegs_STS_IRQ;


typedef struct {
    volatile uint32_t EOI;                       /* IRQ_EOI */
    volatile uint32_t STATUS_RAW;                /* IRQ_STATUS_RAW */
    volatile uint32_t STATUS;                    /* IRQ_STATUS */
    volatile uint32_t ENABLE_SET;                /* IRQ_ENABLE_SET */
    volatile uint32_t ENABLE_CLR;                /* IRQ_ENABLE_CLR */
} CSL_fss_fota_genregsRegs_ERR_STS_IRQ;


typedef struct {
    volatile uint32_t FOTA_INIT;                 /* FOTA_INIT */
    volatile uint32_t FOTA_CTRL;                 /* FOTA_CTRL */
    volatile uint32_t FOTA_ERR_INFO;             /* FOTA_ERR_INFO */
    volatile uint8_t  Resv_16[4];
    volatile uint32_t FOTA_GP0;                  /* FOTA_GP0 */
    volatile uint32_t FOTA_GP1;                  /* FOTA_GP1 */
    volatile uint32_t FOTA_ADDR;                 /* FOTA_ADDR */
    volatile uint32_t FOTA_CNT;                  /* FOTA_CNT */
    CSL_fss_fota_genregsRegs_STS_IRQ STS_IRQ;
    volatile uint8_t  Resv_64[12];
    CSL_fss_fota_genregsRegs_ERR_STS_IRQ ERR_STS_IRQ;
} CSL_fss_fota_genregsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSS_FOTA_GENREGS_FOTA_INIT                                         (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL                                         (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO                                     (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0                                          (0x00000010U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1                                          (0x00000014U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR                                         (0x00000018U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CNT                                          (0x0000001CU)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_EOI                                       (0x00000020U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_RAW                                (0x00000024U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS                                    (0x00000028U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_SET                                (0x0000002CU)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_CLR                                (0x00000030U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_EOI                                   (0x00000040U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW                            (0x00000044U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS                                (0x00000048U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET                            (0x0000004CU)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR                            (0x00000050U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* EOI */

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_EOI_EOI_VECTOR_MASK                       (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_EOI_EOI_VECTOR_SHIFT                      (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_EOI_EOI_VECTOR_RESETVAL                   (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_EOI_EOI_VECTOR_MAX                        (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_EOI_RESETVAL                              (0x00000000U)

/* STATUS_RAW */

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_RAW_FOTA_DONE_MASK                 (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_RAW_FOTA_DONE_SHIFT                (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_RAW_FOTA_DONE_RESETVAL             (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_RAW_FOTA_DONE_MAX                  (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_RAW_RESETVAL                       (0x00000000U)

/* STATUS */

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_FOTA_DONE_MASK                     (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_FOTA_DONE_SHIFT                    (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_FOTA_DONE_RESETVAL                 (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_FOTA_DONE_MAX                      (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_STATUS_RESETVAL                           (0x00000000U)

/* ENABLE_SET */

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_SET_FOTA_DONE_MASK                 (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_SET_FOTA_DONE_SHIFT                (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_SET_FOTA_DONE_RESETVAL             (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_SET_FOTA_DONE_MAX                  (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_SET_RESETVAL                       (0x00000000U)

/* ENABLE_CLR */

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_CLR_FOTA_DONE_MASK                 (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_CLR_FOTA_DONE_SHIFT                (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_CLR_FOTA_DONE_RESETVAL             (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_CLR_FOTA_DONE_MAX                  (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_STS_IRQ_ENABLE_CLR_RESETVAL                       (0x00000000U)

/* EOI */

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_EOI_EOI_VECTOR_MASK                   (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_EOI_EOI_VECTOR_SHIFT                  (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_EOI_EOI_VECTOR_RESETVAL               (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_EOI_EOI_VECTOR_MAX                    (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_EOI_RESETVAL                          (0x00000000U)

/* STATUS_RAW */

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_CFG_READ_ERR_MASK          (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_CFG_READ_ERR_SHIFT         (0x00000002U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_CFG_READ_ERR_RESETVAL      (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_CFG_READ_ERR_MAX           (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_CFG_WRITE_ERR_MASK         (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_CFG_WRITE_ERR_SHIFT        (0x00000003U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_CFG_WRITE_ERR_RESETVAL     (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_CFG_WRITE_ERR_MAX          (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_DAT_READ_ERR_MASK          (0x00000010U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_DAT_READ_ERR_SHIFT         (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_DAT_READ_ERR_RESETVAL      (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_DAT_READ_ERR_MAX           (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_DAT_WRITE_ERR_MASK         (0x00000020U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_DAT_WRITE_ERR_SHIFT        (0x00000005U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_DAT_WRITE_ERR_RESETVAL     (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_DAT_WRITE_ERR_MAX          (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_MCU_ERR_MASK               (0x00000040U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_MCU_ERR_SHIFT              (0x00000006U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_MCU_ERR_RESETVAL           (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_MCU_ERR_MAX                (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RAW_RESETVAL                   (0x00000000U)

/* STATUS */

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_CFG_READ_ERR_MASK              (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_CFG_READ_ERR_SHIFT             (0x00000002U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_CFG_READ_ERR_RESETVAL          (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_CFG_READ_ERR_MAX               (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_CFG_WRITE_ERR_MASK             (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_CFG_WRITE_ERR_SHIFT            (0x00000003U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_CFG_WRITE_ERR_RESETVAL         (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_CFG_WRITE_ERR_MAX              (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_DAT_READ_ERR_MASK              (0x00000010U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_DAT_READ_ERR_SHIFT             (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_DAT_READ_ERR_RESETVAL          (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_DAT_READ_ERR_MAX               (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_DAT_WRITE_ERR_MASK             (0x00000020U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_DAT_WRITE_ERR_SHIFT            (0x00000005U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_DAT_WRITE_ERR_RESETVAL         (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_DAT_WRITE_ERR_MAX              (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_MCU_ERR_MASK                   (0x00000040U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_MCU_ERR_SHIFT                  (0x00000006U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_MCU_ERR_RESETVAL               (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_MCU_ERR_MAX                    (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_STATUS_RESETVAL                       (0x00000000U)

/* ENABLE_SET */

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_CFG_READ_ERR_MASK          (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_CFG_READ_ERR_SHIFT         (0x00000002U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_CFG_READ_ERR_RESETVAL      (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_CFG_READ_ERR_MAX           (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_CFG_WRITE_ERR_MASK         (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_CFG_WRITE_ERR_SHIFT        (0x00000003U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_CFG_WRITE_ERR_RESETVAL     (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_CFG_WRITE_ERR_MAX          (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_DAT_READ_ERR_MASK          (0x00000010U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_DAT_READ_ERR_SHIFT         (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_DAT_READ_ERR_RESETVAL      (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_DAT_READ_ERR_MAX           (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_DAT_WRITE_ERR_MASK         (0x00000020U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_DAT_WRITE_ERR_SHIFT        (0x00000005U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_DAT_WRITE_ERR_RESETVAL     (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_DAT_WRITE_ERR_MAX          (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_MCU_ERR_MASK               (0x00000040U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_MCU_ERR_SHIFT              (0x00000006U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_MCU_ERR_RESETVAL           (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_MCU_ERR_MAX                (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_SET_RESETVAL                   (0x00000000U)

/* ENABLE_CLR */

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_CFG_READ_ERR_MASK          (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_CFG_READ_ERR_SHIFT         (0x00000002U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_CFG_READ_ERR_RESETVAL      (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_CFG_READ_ERR_MAX           (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_CFG_WRITE_ERR_MASK         (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_CFG_WRITE_ERR_SHIFT        (0x00000003U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_CFG_WRITE_ERR_RESETVAL     (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_CFG_WRITE_ERR_MAX          (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_DAT_READ_ERR_MASK          (0x00000010U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_DAT_READ_ERR_SHIFT         (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_DAT_READ_ERR_RESETVAL      (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_DAT_READ_ERR_MAX           (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_DAT_WRITE_ERR_MASK         (0x00000020U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_DAT_WRITE_ERR_SHIFT        (0x00000005U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_DAT_WRITE_ERR_RESETVAL     (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_DAT_WRITE_ERR_MAX          (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_MCU_ERR_MASK               (0x00000040U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_MCU_ERR_SHIFT              (0x00000006U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_MCU_ERR_RESETVAL           (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_MCU_ERR_MAX                (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_ERR_STS_IRQ_ENABLE_CLR_RESETVAL                   (0x00000000U)

/* FOTA_INIT */

#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_RESET_MASK                              (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_RESET_SHIFT                             (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_RESET_RESETVAL                          (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_RESET_MAX                               (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_CLKDIS_MASK                             (0x00000002U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_CLKDIS_SHIFT                            (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_CLKDIS_RESETVAL                         (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_CLKDIS_MAX                              (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_MEMACCESS_MASK                          (0x00000004U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_MEMACCESS_SHIFT                         (0x00000002U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_MEMACCESS_RESETVAL                      (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_MEMACCESS_MAX                           (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_PDMEM_INIT_DONE_MASK                    (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_PDMEM_INIT_DONE_SHIFT                   (0x00000003U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_PDMEM_INIT_DONE_RESETVAL                (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_PDMEM_INIT_DONE_MAX                     (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_FOTA_INIT_RESETVAL                                (0x00000005U)

/* FOTA_CTRL */

#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_GO_MASK                                 (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_GO_SHIFT                                (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_GO_RESETVAL                             (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_GO_MAX                                  (0x00000001U)

#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_ACK_MASK                                 (0x00000002U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_ACK_SHIFT                                (0x00000001U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_ACK_RESETVAL                             (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_ACK_MAX                                  (0x00000002U)

#define CSL_FSS_FOTA_GENREGS_FOTA_CTRL_RESETVAL                                (0x00000000U)

/* FOTA_ERR_INFO */

#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_CFG_ERR_RSTATUS_MASK                (0x00000007U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_CFG_ERR_RSTATUS_SHIFT               (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_CFG_ERR_RSTATUS_RESETVAL            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_CFG_ERR_RSTATUS_MAX                 (0x00000007U)

#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_CFG_ERR_SSTATUS_MASK                (0x00000038U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_CFG_ERR_SSTATUS_SHIFT               (0x00000003U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_CFG_ERR_SSTATUS_RESETVAL            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_CFG_ERR_SSTATUS_MAX                 (0x00000007U)

#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_DAT_ERR_RSTATUS_MASK                (0x000001C0U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_DAT_ERR_RSTATUS_SHIFT               (0x00000006U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_DAT_ERR_RSTATUS_RESETVAL            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_DAT_ERR_RSTATUS_MAX                 (0x00000007U)

#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_DAT_ERR_SSTATUS_MASK                (0x00000E00U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_DAT_ERR_SSTATUS_SHIFT               (0x00000009U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_DAT_ERR_SSTATUS_RESETVAL            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_DAT_ERR_SSTATUS_MAX                 (0x00000007U)

#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_MCU_ERR_CODE_MASK                   (0x0001F000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_MCU_ERR_CODE_SHIFT                  (0x0000000CU)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_MCU_ERR_CODE_RESETVAL               (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_MCU_ERR_CODE_MAX                    (0x0000001FU)

#define CSL_FSS_FOTA_GENREGS_FOTA_ERR_INFO_RESETVAL                            (0x00000000U)

/* FOTA_GP0 */

#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL0_MASK                                (0x000000FFU)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL0_SHIFT                               (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL0_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL0_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL1_MASK                                (0x0000FF00U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL1_SHIFT                               (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL1_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL1_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL2_MASK                                (0x00FF0000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL2_SHIFT                               (0x00000010U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL2_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL2_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL3_MASK                                (0xFF000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL3_SHIFT                               (0x00000018U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL3_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_VAL3_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_GP0_RESETVAL                                 (0x00000000U)

/* FOTA_GP1 */

#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL0_MASK                                (0x000000FFU)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL0_SHIFT                               (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL0_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL0_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL1_MASK                                (0x0000FF00U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL1_SHIFT                               (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL1_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL1_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL2_MASK                                (0x00FF0000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL2_SHIFT                               (0x00000010U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL2_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL2_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL3_MASK                                (0xFF000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL3_SHIFT                               (0x00000018U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL3_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_VAL3_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_GP1_RESETVAL                                 (0x00000000U)

/* FOTA_ADDR */

#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL0_MASK                               (0x000000FFU)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL0_SHIFT                              (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL0_RESETVAL                           (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL0_MAX                                (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL1_MASK                               (0x0000FF00U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL1_SHIFT                              (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL1_RESETVAL                           (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL1_MAX                                (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL2_MASK                               (0x00FF0000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL2_SHIFT                              (0x00000010U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL2_RESETVAL                           (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL2_MAX                                (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL3_MASK                               (0xFF000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL3_SHIFT                              (0x00000018U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL3_RESETVAL                           (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_VAL3_MAX                                (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_ADDR_RESETVAL                                (0x00000000U)

/* FOTA_CNT */

#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_VAL0_MASK                                (0x000000FFU)
#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_VAL0_SHIFT                               (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_VAL0_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_VAL0_MAX                                 (0x000000FFU)

#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_VAL1_MASK                                (0x00000300U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_VAL1_SHIFT                               (0x00000008U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_VAL1_RESETVAL                            (0x00000000U)
#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_VAL1_MAX                                 (0x00000003U)

#define CSL_FSS_FOTA_GENREGS_FOTA_CNT_RESETVAL                                 (0x00000000U)

/**************************************************************************
* Hardware Region  : M8051EW program and external data memory
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t LOC[512];                  /* M8051EW program and external data memory array */
} CSL_fss_pdmem_genregsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSS_PDMEM_GENREGS_LOC(LOC)                                         (0x00000000U+((LOC)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* LOC */

#define CSL_FSS_PDMEM_GENREGS_LOC_VAL_MASK                                     (0xFFFFFFFFU)
#define CSL_FSS_PDMEM_GENREGS_LOC_VAL_SHIFT                                    (0x00000000U)
#define CSL_FSS_PDMEM_GENREGS_LOC_VAL_RESETVAL                                 (0x00000000U)
#define CSL_FSS_PDMEM_GENREGS_LOC_VAL_MAX                                      (0xFFFFFFFFU)

#define CSL_FSS_PDMEM_GENREGS_LOC_RESETVAL                                     (0x00000000U)

/**************************************************************************
* Hardware Region  : M8051ew internal memory
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t LOC[64];                   /* M8051EW Internal Data Memory array */
} CSL_fss_imem_genregsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSS_IMEM_GENREGS_LOC(LOC)                                          (0x00000000U+((LOC)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* LOC */

#define CSL_FSS_IMEM_GENREGS_LOC_VAL_MASK                                      (0xFFFFFFFFU)
#define CSL_FSS_IMEM_GENREGS_LOC_VAL_SHIFT                                     (0x00000000U)
#define CSL_FSS_IMEM_GENREGS_LOC_VAL_RESETVAL                                  (0x00000000U)
#define CSL_FSS_IMEM_GENREGS_LOC_VAL_MAX                                       (0xFFFFFFFFU)

#define CSL_FSS_IMEM_GENREGS_LOC_RESETVAL                                      (0x00000000U)

/**************************************************************************
* Hardware Region  : FOTA write buffer
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t LOC[128];                  /* FOTA write buffer array */
} CSL_fss_wbuf_genregsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSS_WBUF_GENREGS_LOC(LOC)                                          (0x00000000U+((LOC)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* LOC */

#define CSL_FSS_WBUF_GENREGS_LOC_VAL_MASK                                      (0xFFFFFFFFU)
#define CSL_FSS_WBUF_GENREGS_LOC_VAL_SHIFT                                     (0x00000000U)
#define CSL_FSS_WBUF_GENREGS_LOC_VAL_RESETVAL                                  (0x00000000U)
#define CSL_FSS_WBUF_GENREGS_LOC_VAL_MAX                                       (0xFFFFFFFFU)

#define CSL_FSS_WBUF_GENREGS_LOC_RESETVAL                                      (0x00000000U)

/**************************************************************************
* Hardware Region  : FSAS data region0
**************************************************************************/

/*

*************************************************************************
* Register Overlay Structure
*************************************************************************

typedef struct {
    volatile uint32_t HPB_DATA_MEM[4294967296];    FSAS data region0
} CSL_dat_reg0Regs;


*************************************************************************
* Register Macros
*************************************************************************

#define CSL_DAT_REG0_HPB_DATA_MEM(HPB_DATA_MEM)                                (0x00000000U+((HPB_DATA_MEM)*0x4U))

*************************************************************************
* Field Definition Macros
*************************************************************************


 HPB_DATA_MEM

#define CSL_DAT_REG0_HPB_DATA_MEM_HPB_DATA_MASK                                (0xFFFFFFFFU)
#define CSL_DAT_REG0_HPB_DATA_MEM_HPB_DATA_SHIFT                               (0x00000000U)
#define CSL_DAT_REG0_HPB_DATA_MEM_HPB_DATA_RESETVAL                            (0x00000000U)
#define CSL_DAT_REG0_HPB_DATA_MEM_HPB_DATA_MAX                                 (0xFFFFFFFFU)

#define CSL_DAT_REG0_HPB_DATA_MEM_RESETVAL                                     (0x00000000U)

*************************************************************************
* Hardware Region  : FSAS data region1
*************************************************************************


*************************************************************************
* Register Overlay Structure
*************************************************************************

typedef struct {
    volatile uint32_t HPB_DATA_MEM[134217728];    FSAS data region1
} CSL_dat_reg1Regs;


*************************************************************************
* Register Macros
*************************************************************************

#define CSL_DAT_REG1_HPB_DATA_MEM(HPB_DATA_MEM)                                (0x00000000U+((HPB_DATA_MEM)*0x4U))

*************************************************************************
* Field Definition Macros
*************************************************************************


 HPB_DATA_MEM

#define CSL_DAT_REG1_HPB_DATA_MEM_HPB_DATA_MASK                                (0xFFFFFFFFU)
#define CSL_DAT_REG1_HPB_DATA_MEM_HPB_DATA_SHIFT                               (0x00000000U)
#define CSL_DAT_REG1_HPB_DATA_MEM_HPB_DATA_RESETVAL                            (0x00000000U)
#define CSL_DAT_REG1_HPB_DATA_MEM_HPB_DATA_MAX                                 (0xFFFFFFFFU)

#define CSL_DAT_REG1_HPB_DATA_MEM_RESETVAL                                     (0x00000000U)

*************************************************************************
* Hardware Region  : FSAS data region3
*************************************************************************


*************************************************************************
* Register Overlay Structure
*************************************************************************

typedef struct {
    volatile uint32_t HPB_DATA_MEM[4294967296];    FSAS data region3
} CSL_dat_reg3Regs;


*************************************************************************
* Register Macros
*************************************************************************

#define CSL_DAT_REG3_HPB_DATA_MEM(HPB_DATA_MEM)                                (0x00000000U+((HPB_DATA_MEM)*0x4U))

*************************************************************************
* Field Definition Macros
*************************************************************************


 HPB_DATA_MEM

#define CSL_DAT_REG3_HPB_DATA_MEM_HPB_DATA_MASK                                (0xFFFFFFFFU)
#define CSL_DAT_REG3_HPB_DATA_MEM_HPB_DATA_SHIFT                               (0x00000000U)
#define CSL_DAT_REG3_HPB_DATA_MEM_HPB_DATA_RESETVAL                            (0x00000000U)
#define CSL_DAT_REG3_HPB_DATA_MEM_HPB_DATA_MAX                                 (0xFFFFFFFFU)

#define CSL_DAT_REG3_HPB_DATA_MEM_RESETVAL                                     (0x00000000U)
*/

#ifdef __cplusplus
}
#endif
#endif
