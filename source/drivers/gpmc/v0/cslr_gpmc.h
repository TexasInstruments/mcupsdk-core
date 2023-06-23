/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated
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
#ifndef CSLR_GPMC_H
#define CSLR_GPMC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>



/**************************************************************************
* Register Overlay Structure for BCH_RESULT
**************************************************************************/
typedef struct {
    volatile Uint32 BCH_RESULT_0;
    volatile Uint32 BCH_RESULT_1;
    volatile Uint32 BCH_RESULT_2;
    volatile Uint32 BCH_RESULT_3;
} CSL_GpmcBch_resultRegs;


/**************************************************************************
* Register Overlay Structure for Trailer
**************************************************************************/
typedef struct {
    volatile Uint32 BCH_SWDATA;
} CSL_GpmcTrailerRegs;


/**************************************************************************
* Register Overlay Structure for BCH_RESULT_EXTENSION
**************************************************************************/
typedef struct {
    volatile Uint32 BCH_RESULT_4;
    volatile Uint32 BCH_RESULT_5;
    volatile Uint32 BCH_RESULT_6;
    volatile Uint8  RSVD0[4];
} CSL_GpmcBch_result_extensionRegs;


/**************************************************************************
* Register Overlay Structure for CONFIG_BLOCK
**************************************************************************/
typedef struct {
    volatile Uint32 CONFIG1;
    volatile Uint32 CONFIG2;
    volatile Uint32 CONFIG3;
    volatile Uint32 CONFIG4;
    volatile Uint32 CONFIG5;
    volatile Uint32 CONFIG6;
    volatile Uint32 CONFIG7;
    volatile Uint32 NAND_COMMAND;
    volatile Uint32 NAND_ADDRESS;
    volatile Uint32 NAND_DATA;
    volatile Uint8  RSVD0[8];
} CSL_GpmcConfig_blockRegs;


/**************************************************************************
* Register Overlay Structure
**************************************************************************/
typedef struct {
    volatile Uint32 REVISION;
    volatile Uint8  RSVD1[12];
    volatile Uint32 SYSCONFIG;
    volatile Uint32 SYSSTATUS;
    volatile Uint32 IRQSTATUS;
    volatile Uint32 IRQENABLE;
    volatile Uint8  RSVD2[32];
    volatile Uint32 TIMEOUT_CONTROL;
    volatile Uint32 ERR_ADDRESS;
    volatile Uint32 ERR_TYPE;
    volatile Uint8  RSVD3[4];
    volatile Uint32 CONFIG;
    volatile Uint32 STATUS;
    volatile Uint8  RSVD4[8];
    CSL_GpmcConfig_blockRegs	CONFIG_BLOCK[8];
    volatile Uint32 PREFETCH_CONFIG1;
    volatile Uint32 PREFETCH_CONFIG2;
    volatile Uint8  RSVD5[4];
    volatile Uint32 PREFETCH_CONTROL;
    volatile Uint32 PREFETCH_STATUS;
    volatile Uint32 ECC_CONFIG;
    volatile Uint32 ECC_CONTROL;
    volatile Uint32 ECC_SIZE_CONFIG;
    volatile Uint32 ECC_RESULT[9];
    volatile Uint8  RSVD6[12];
    volatile Uint32 TESTMODE_CTRL;
    volatile Uint8  RSVD7[12];
    CSL_GpmcBch_resultRegs	BCH_RESULT[8];
    volatile Uint8  RSVD8[16];
    CSL_GpmcTrailerRegs	TRAILER;
    volatile Uint8  RSVD9[44];
    CSL_GpmcBch_result_extensionRegs	BCH_RESULT_EXTENSION[8];
} CSL_GpmcRegs;




/**************************************************************************
* Register Macros
**************************************************************************/

/* This register contains the IP revision code */
#define CSL_GPMC_REVISION                                       (0x0U)

/* This register controls the various parameters of the OCP interface */
#define CSL_GPMC_SYSCONFIG                                      (0x10U)

/* This register provides status information about the module, excluding the
 * interrupt status information */
#define CSL_GPMC_SYSSTATUS                                      (0x14U)

/* This interrupt status register regroups all the status of the module
 * internal events that can generate an interrupt. */
#define CSL_GPMC_IRQSTATUS                                      (0x18U)

/* The interrupt enable register allows to mask/unmask the module internal
 * sources of interrupt, on a event-by-event basis. */
#define CSL_GPMC_IRQENABLE                                      (0x1CU)

/* The GPMC_TIMEOUT_CONTROL register allows the user to set the start value of
 * the timeout counter */
#define CSL_GPMC_TIMEOUT_CONTROL                                (0x40U)

/* The GPMC_ERR_ADDRESS register stores the address of the illegal access when
 * an error occurs */
#define CSL_GPMC_ERR_ADDRESS                                    (0x44U)

/* The GPMC_ERR_TYPE register stores the type of error when an error occurs */
#define CSL_GPMC_ERR_TYPE                                       (0x48U)

/* The configuration register allows global configuration of the GPMC */
#define CSL_GPMC_CONFIG                                         (0x50U)

/* The status register provides global status bits of the GPMC */
#define CSL_GPMC_STATUS                                         (0x54U)

/* Prefetch engine configuration 1 */
#define CSL_GPMC_PREFETCH_CONFIG1                               (0x1E0U)

/* Prefetch engine configuration 2 */
#define CSL_GPMC_PREFETCH_CONFIG2                               (0x1E4U)

/* Prefetch engine control */
#define CSL_GPMC_PREFETCH_CONTROL                               (0x1ECU)

/* Prefetch engine status */
#define CSL_GPMC_PREFETCH_STATUS                                (0x1F0U)

/* ECC configuration */
#define CSL_GPMC_ECC_CONFIG                                     (0x1F4U)

/* ECC control */
#define CSL_GPMC_ECC_CONTROL                                    (0x1F8U)

/* ECC size */
#define CSL_GPMC_ECC_SIZE_CONFIG                                (0x1FCU)

/* ECC result register */
#define CSL_GPMC_ECC_RESULT(i)                                  (0x200U + ((i) * (0x4U)))

/* TestMode Control Register */
#define CSL_GPMC_TESTMODE_CTRL                                  (0x230U)

/* BCH ECC result, bits 0 to 31 */
#define CSL_GPMC_BCH_RESULT_0(n)                                (0x240U + ((n) * (0x10U)))

/* BCH ECC result, bits 32 to 63 */
#define CSL_GPMC_BCH_RESULT_1(n)                                (0x244U + ((n) * (0x10U)))

/* BCH ECC result, bits 64 to 95 */
#define CSL_GPMC_BCH_RESULT_2(n)                                (0x248U + ((n) * (0x10U)))

/* BCH ECC result, bits 96 to 127 */
#define CSL_GPMC_BCH_RESULT_3(n)                                (0x24CU + ((n) * (0x10U)))

/* This register is used to directly pass data to the BCH ECC calculator
 * without accessing the actual NAND flash interface. */
#define CSL_GPMC_BCH_SWDATA                                     (0x2D0U)

/* BCH ECC result, bits 160 to 191 */
#define CSL_GPMC_BCH_RESULT_5(n)                                (0x304U + ((n) * (0x10U)))

/* BCH ECC result, bits 192 to 207 */
#define CSL_GPMC_BCH_RESULT_6(n)                                (0x308U + ((n) * (0x10U)))

/* BCH ECC result, bits 128 to 159 */
#define CSL_GPMC_BCH_RESULT_4(n)                                (0x300U + ((n) * (0x10U)))

/* The configuration 1 register sets signal control parameters per chip select */
#define CSL_GPMC_CONFIG1(n)                                     (0x60U + ((n) * (0x30U)))

/* Chip-select signal timing parameter configuration */
#define CSL_GPMC_CONFIG2(n)                                     (0x64U + ((n) * (0x30U)))

/* ADV# signal timing parameter configuration */
#define CSL_GPMC_CONFIG3(n)                                     (0x68U + ((n) * (0x30U)))

/* WE# and OE# signals timing parameter configuration */
#define CSL_GPMC_CONFIG4(n)                                     (0x6CU + ((n) * (0x30U)))

/* RdAccessTime and CycleTime timing parameters configuration */
#define CSL_GPMC_CONFIG5(n)                                     (0x70U + ((n) * (0x30U)))

/* WrAccessTime, WrDataOnADmuxBus, Cycle2Cycle and BusTurnAround parameters
 * configuration */
#define CSL_GPMC_CONFIG6(n)                                     (0x74U + ((n) * (0x30U)))

/* Chip-select address mapping configuration Note: For CS0, the register reset
 * is 0xf40 while for all the other instances CS1-CS7, the reset is 0xf00. */
#define CSL_GPMC_CONFIG7(n)                                     (0x78U + ((n) * (0x30U)))

/* This Register is not a true register, just a address location. */
#define CSL_GPMC_NAND_COMMAND(n)                                (0x7CU + ((n) * (0x30U)))

/* This Register is not a true register, just a address location. */
#define CSL_GPMC_NAND_ADDRESS(n)                                (0x80U + ((n) * (0x30U)))

/* This Register is not a true register, just a address location. */
#define CSL_GPMC_NAND_DATA(n)                                   (0x84U + ((n) * (0x30U)))


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* REVISION */

#define CSL_GPMC_REVISION_REV_MASK                              (0x000000FFU)
#define CSL_GPMC_REVISION_REV_SHIFT                             (0U)
#define CSL_GPMC_REVISION_REV_RESETVAL                          (0x00000000U)
#define CSL_GPMC_REVISION_REV_MAX                               (0x000000ffU)

#define CSL_GPMC_REVISION_RESETVAL                              (0x00000000U)

/* SYSCONFIG */

#define CSL_GPMC_SYSCONFIG_SOFTRESET_MASK                       (0x00000002U)
#define CSL_GPMC_SYSCONFIG_SOFTRESET_SHIFT                      (1U)
#define CSL_GPMC_SYSCONFIG_SOFTRESET_RESETVAL                   (0x00000000U)
#define CSL_GPMC_SYSCONFIG_SOFTRESET_NORMAL                     (0x00000000U)
#define CSL_GPMC_SYSCONFIG_SOFTRESET_RESET                      (0x00000001U)

#define CSL_GPMC_SYSCONFIG_IDLEMODE_MASK                        (0x00000018U)
#define CSL_GPMC_SYSCONFIG_IDLEMODE_SHIFT                       (3U)
#define CSL_GPMC_SYSCONFIG_IDLEMODE_RESETVAL                    (0x00000000U)
#define CSL_GPMC_SYSCONFIG_IDLEMODE_FORCEIDLE                   (0x00000000U)
#define CSL_GPMC_SYSCONFIG_IDLEMODE_NOIDLE                      (0x00000001U)
#define CSL_GPMC_SYSCONFIG_IDLEMODE_SMARTIDLE                   (0x00000002U)
#define CSL_GPMC_SYSCONFIG_IDLEMODE_RESERVED                    (0x00000003U)

#define CSL_GPMC_SYSCONFIG_AUTOIDLE_MASK                        (0x00000001U)
#define CSL_GPMC_SYSCONFIG_AUTOIDLE_SHIFT                       (0U)
#define CSL_GPMC_SYSCONFIG_AUTOIDLE_RESETVAL                    (0x00000000U)
#define CSL_GPMC_SYSCONFIG_AUTOIDLE_FREERUN                     (0x00000000U)
#define CSL_GPMC_SYSCONFIG_AUTOIDLE_AUTORUN                     (0x00000001U)

#define CSL_GPMC_SYSCONFIG_RESETVAL                             (0x00000000U)

/* SYSSTATUS */

#define CSL_GPMC_SYSSTATUS_RESETDONE_MASK                       (0x00000001U)
#define CSL_GPMC_SYSSTATUS_RESETDONE_SHIFT                      (0U)
#define CSL_GPMC_SYSSTATUS_RESETDONE_RESETVAL                   (0x00000000U)
#define CSL_GPMC_SYSSTATUS_RESETDONE_RSTONGOING                 (0x00000000U)
#define CSL_GPMC_SYSSTATUS_RESETDONE_RSTDONE                    (0x00000001U)

#define CSL_GPMC_SYSSTATUS_RESETVAL                             (0x00000000U)

/* IRQSTATUS */

#define CSL_GPMC_IRQSTATUS_WAIT2EDGEDETECTIONSTATUS_MASK        (0x00000400U)
#define CSL_GPMC_IRQSTATUS_WAIT2EDGEDETECTIONSTATUS_SHIFT       (10U)
#define CSL_GPMC_IRQSTATUS_WAIT2EDGEDETECTIONSTATUS_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT2EDGEDETECTIONSTATUS_W2DET0_R    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT2EDGEDETECTIONSTATUS_W2DET0_W    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT2EDGEDETECTIONSTATUS_W2DET1_R    (0x00000001U)
#define CSL_GPMC_IRQSTATUS_WAIT2EDGEDETECTIONSTATUS_W2DET1_W    (0x00000001U)

#define CSL_GPMC_IRQSTATUS_WAIT3EDGEDETECTIONSTATUS_MASK        (0x00000800U)
#define CSL_GPMC_IRQSTATUS_WAIT3EDGEDETECTIONSTATUS_SHIFT       (11U)
#define CSL_GPMC_IRQSTATUS_WAIT3EDGEDETECTIONSTATUS_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT3EDGEDETECTIONSTATUS_W3DET0_R    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT3EDGEDETECTIONSTATUS_W3DET0_W    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT3EDGEDETECTIONSTATUS_W3DET1_R    (0x00000001U)
#define CSL_GPMC_IRQSTATUS_WAIT3EDGEDETECTIONSTATUS_W3DET1_W    (0x00000001U)

#define CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_MASK             (0x00000002U)
#define CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_SHIFT            (1U)
#define CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_RESETVAL         (0x00000000U)
#define CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_TCSTAT0_R        (0x00000000U)
#define CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_TCSTAT0_W        (0x00000000U)
#define CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_TCSTAT1_R        (0x00000001U)
#define CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_TCSTAT1_W        (0x00000001U)

#define CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_MASK        (0x00000100U)
#define CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_SHIFT       (8U)
#define CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_W0DET0_R    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_W0DET0_W    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_W0DET1_R    (0x00000001U)
#define CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_W0DET1_W    (0x00000001U)

#define CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_MASK        (0x00000200U)
#define CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_SHIFT       (9U)
#define CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_W1DET0_R    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_W1DET0_W    (0x00000000U)
#define CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_W1DET1_R    (0x00000001U)
#define CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_W1DET1_W    (0x00000001U)

#define CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_MASK                 (0x00000001U)
#define CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_SHIFT                (0U)
#define CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_RESETVAL             (0x00000000U)
#define CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_FIFOSTAT0_R          (0x00000000U)
#define CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_FIFOSTAT0_W          (0x00000000U)
#define CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_FIFOSTAT1_R          (0x00000001U)
#define CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_FIFOSTAT1_W          (0x00000001U)

#define CSL_GPMC_IRQSTATUS_RESETVAL                             (0x00000000U)

/* IRQENABLE */

#define CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_MASK        (0x00000002U)
#define CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_SHIFT       (1U)
#define CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_TCMASKED    (0x00000000U)
#define CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_TCENABLED   (0x00000001U)

#define CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_MASK        (0x00000100U)
#define CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_SHIFT       (8U)
#define CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_W0MASKED    (0x00000000U)
#define CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_W0ENABLED   (0x00000001U)

#define CSL_GPMC_IRQENABLE_WAIT2EDGEDETECTIONENABLE_MASK        (0x00000400U)
#define CSL_GPMC_IRQENABLE_WAIT2EDGEDETECTIONENABLE_SHIFT       (10U)
#define CSL_GPMC_IRQENABLE_WAIT2EDGEDETECTIONENABLE_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQENABLE_WAIT2EDGEDETECTIONENABLE_W2MASKED    (0x00000000U)
#define CSL_GPMC_IRQENABLE_WAIT2EDGEDETECTIONENABLE_W2ENABLED   (0x00000001U)

#define CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_MASK                 (0x00000001U)
#define CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_SHIFT                (0U)
#define CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_RESETVAL             (0x00000000U)
#define CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_FIFOMASKED           (0x00000000U)
#define CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_FIFOENABLED          (0x00000001U)

#define CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_MASK        (0x00000200U)
#define CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_SHIFT       (9U)
#define CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_W1MASKED    (0x00000000U)
#define CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_W1ENABLED   (0x00000001U)

#define CSL_GPMC_IRQENABLE_WAIT3EDGEDETECTIONENABLE_MASK        (0x00000800U)
#define CSL_GPMC_IRQENABLE_WAIT3EDGEDETECTIONENABLE_SHIFT       (11U)
#define CSL_GPMC_IRQENABLE_WAIT3EDGEDETECTIONENABLE_RESETVAL    (0x00000000U)
#define CSL_GPMC_IRQENABLE_WAIT3EDGEDETECTIONENABLE_W3MASKED    (0x00000000U)
#define CSL_GPMC_IRQENABLE_WAIT3EDGEDETECTIONENABLE_W3ENABLED   (0x00000001U)

#define CSL_GPMC_IRQENABLE_RESETVAL                             (0x00000000U)

/* TIMEOUT_CONTROL */

#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTSTARTVALUE_MASK         (0x00001FF0U)
#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTSTARTVALUE_SHIFT        (4U)
#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTSTARTVALUE_RESETVAL     (0x000001ffU)
#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTSTARTVALUE_MAX          (0x000001ffU)

#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE_MASK             (0x00000001U)
#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE_SHIFT            (0U)
#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE_RESETVAL         (0x00000000U)
#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE_TODISABLED       (0x00000000U)
#define CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE_TOENABLED        (0x00000001U)

#define CSL_GPMC_TIMEOUT_CONTROL_RESETVAL                       (0x00001ff0U)

/* ERR_ADDRESS */

#define CSL_GPMC_ERR_ADDRESS_ILLEGALADD_MASK                    (0x7FFFFFFFU)
#define CSL_GPMC_ERR_ADDRESS_ILLEGALADD_SHIFT                   (0U)
#define CSL_GPMC_ERR_ADDRESS_ILLEGALADD_RESETVAL                (0x00000000U)
#define CSL_GPMC_ERR_ADDRESS_ILLEGALADD_MAX                     (0x7fffffffU)

#define CSL_GPMC_ERR_ADDRESS_RESETVAL                           (0x00000000U)

/* ERR_TYPE */

#define CSL_GPMC_ERR_TYPE_ILLEGALMCMD_MASK                      (0x00000700U)
#define CSL_GPMC_ERR_TYPE_ILLEGALMCMD_SHIFT                     (8U)
#define CSL_GPMC_ERR_TYPE_ILLEGALMCMD_RESETVAL                  (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ILLEGALMCMD_MAX                       (0x00000007U)

#define CSL_GPMC_ERR_TYPE_ERRORTIMEOUT_MASK                     (0x00000004U)
#define CSL_GPMC_ERR_TYPE_ERRORTIMEOUT_SHIFT                    (2U)
#define CSL_GPMC_ERR_TYPE_ERRORTIMEOUT_RESETVAL                 (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ERRORTIMEOUT_NOERR                    (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ERRORTIMEOUT_ERR                      (0x00000001U)

#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPADD_MASK                  (0x00000010U)
#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPADD_SHIFT                 (4U)
#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPADD_RESETVAL              (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPADD_NOERR                 (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPADD_ERR                   (0x00000001U)

#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPMCMD_MASK                 (0x00000008U)
#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPMCMD_SHIFT                (3U)
#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPMCMD_RESETVAL             (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPMCMD_NOERR                (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ERRORNOTSUPPMCMD_ERR                  (0x00000001U)

#define CSL_GPMC_ERR_TYPE_ERRORVALID_MASK                       (0x00000001U)
#define CSL_GPMC_ERR_TYPE_ERRORVALID_SHIFT                      (0U)
#define CSL_GPMC_ERR_TYPE_ERRORVALID_RESETVAL                   (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ERRORVALID_NOTVALID                   (0x00000000U)
#define CSL_GPMC_ERR_TYPE_ERRORVALID_ERRDETECT                  (0x00000001U)

#define CSL_GPMC_ERR_TYPE_RESETVAL                              (0x00000000U)

/* CONFIG */

#define CSL_GPMC_CONFIG_NANDFORCEPOSTEDWRITE_MASK               (0x00000001U)
#define CSL_GPMC_CONFIG_NANDFORCEPOSTEDWRITE_SHIFT              (0U)
#define CSL_GPMC_CONFIG_NANDFORCEPOSTEDWRITE_RESETVAL           (0x00000000U)
#define CSL_GPMC_CONFIG_NANDFORCEPOSTEDWRITE_NOFORCEPWR         (0x00000000U)
#define CSL_GPMC_CONFIG_NANDFORCEPOSTEDWRITE_FORCEPWR           (0x00000001U)

#define CSL_GPMC_CONFIG_WAIT3PINPOLARITY_MASK                   (0x00000800U)
#define CSL_GPMC_CONFIG_WAIT3PINPOLARITY_SHIFT                  (11U)
#define CSL_GPMC_CONFIG_WAIT3PINPOLARITY_RESETVAL               (0x00000001U)
#define CSL_GPMC_CONFIG_WAIT3PINPOLARITY_W3ACTIVEL              (0x00000000U)
#define CSL_GPMC_CONFIG_WAIT3PINPOLARITY_W3ACTIVEH              (0x00000001U)

#define CSL_GPMC_CONFIG_WAIT1PINPOLARITY_MASK                   (0x00000200U)
#define CSL_GPMC_CONFIG_WAIT1PINPOLARITY_SHIFT                  (9U)
#define CSL_GPMC_CONFIG_WAIT1PINPOLARITY_RESETVAL               (0x00000001U)
#define CSL_GPMC_CONFIG_WAIT1PINPOLARITY_W1ACTIVEL              (0x00000000U)
#define CSL_GPMC_CONFIG_WAIT1PINPOLARITY_W1ACTIVEH              (0x00000001U)

#define CSL_GPMC_CONFIG_WRITEPROTECT_MASK                       (0x00000010U)
#define CSL_GPMC_CONFIG_WRITEPROTECT_SHIFT                      (4U)
#define CSL_GPMC_CONFIG_WRITEPROTECT_RESETVAL                   (0x00000000U)
#define CSL_GPMC_CONFIG_WRITEPROTECT_WPLOW                      (0x00000000U)
#define CSL_GPMC_CONFIG_WRITEPROTECT_WPHIGH                     (0x00000001U)

#define CSL_GPMC_CONFIG_WAIT2PINPOLARITY_MASK                   (0x00000400U)
#define CSL_GPMC_CONFIG_WAIT2PINPOLARITY_SHIFT                  (10U)
#define CSL_GPMC_CONFIG_WAIT2PINPOLARITY_RESETVAL               (0x00000000U)
#define CSL_GPMC_CONFIG_WAIT2PINPOLARITY_W2ACTIVEL              (0x00000000U)
#define CSL_GPMC_CONFIG_WAIT2PINPOLARITY_W2ACTIVEH              (0x00000001U)

#define CSL_GPMC_CONFIG_WAIT0PINPOLARITY_MASK                   (0x00000100U)
#define CSL_GPMC_CONFIG_WAIT0PINPOLARITY_SHIFT                  (8U)
#define CSL_GPMC_CONFIG_WAIT0PINPOLARITY_RESETVAL               (0x00000000U)
#define CSL_GPMC_CONFIG_WAIT0PINPOLARITY_W0ACTIVEL              (0x00000000U)
#define CSL_GPMC_CONFIG_WAIT0PINPOLARITY_W0ACTIVEH              (0x00000001U)

#define CSL_GPMC_CONFIG_LIMITEDADDRESS_MASK                     (0x00000002U)
#define CSL_GPMC_CONFIG_LIMITEDADDRESS_SHIFT                    (1U)
#define CSL_GPMC_CONFIG_LIMITEDADDRESS_RESETVAL                 (0x00000000U)
#define CSL_GPMC_CONFIG_LIMITEDADDRESS_NOLIMITED                (0x00000000U)
#define CSL_GPMC_CONFIG_LIMITEDADDRESS_LIMITED                  (0x00000001U)

#define CSL_GPMC_CONFIG_RESETVAL                                (0x00000a00U)

/* STATUS */

#define CSL_GPMC_STATUS_WAIT1STATUS_MASK                        (0x00000200U)
#define CSL_GPMC_STATUS_WAIT1STATUS_SHIFT                       (9U)
#define CSL_GPMC_STATUS_WAIT1STATUS_RESETVAL                    (0x00000000U)
#define CSL_GPMC_STATUS_WAIT1STATUS_W1ACTIVEL                   (0x00000000U)
#define CSL_GPMC_STATUS_WAIT1STATUS_W1ACTIVEH                   (0x00000001U)

#define CSL_GPMC_STATUS_WAIT3STATUS_MASK                        (0x00000800U)
#define CSL_GPMC_STATUS_WAIT3STATUS_SHIFT                       (11U)
#define CSL_GPMC_STATUS_WAIT3STATUS_RESETVAL                    (0x00000000U)
#define CSL_GPMC_STATUS_WAIT3STATUS_W3ACTIVEL                   (0x00000000U)
#define CSL_GPMC_STATUS_WAIT3STATUS_W3ACTIVEH                   (0x00000001U)

#define CSL_GPMC_STATUS_EMPTYWRITEBUFFERSTATUS_MASK             (0x00000001U)
#define CSL_GPMC_STATUS_EMPTYWRITEBUFFERSTATUS_SHIFT            (0U)
#define CSL_GPMC_STATUS_EMPTYWRITEBUFFERSTATUS_RESETVAL         (0x00000001U)
#define CSL_GPMC_STATUS_EMPTYWRITEBUFFERSTATUS_B0               (0x00000000U)
#define CSL_GPMC_STATUS_EMPTYWRITEBUFFERSTATUS_B1               (0x00000001U)

#define CSL_GPMC_STATUS_WAIT0STATUS_MASK                        (0x00000100U)
#define CSL_GPMC_STATUS_WAIT0STATUS_SHIFT                       (8U)
#define CSL_GPMC_STATUS_WAIT0STATUS_RESETVAL                    (0x00000000U)
#define CSL_GPMC_STATUS_WAIT0STATUS_W0ACTIVEL                   (0x00000000U)
#define CSL_GPMC_STATUS_WAIT0STATUS_W0ACTIVEH                   (0x00000001U)

#define CSL_GPMC_STATUS_WAIT2STATUS_MASK                        (0x00000400U)
#define CSL_GPMC_STATUS_WAIT2STATUS_SHIFT                       (10U)
#define CSL_GPMC_STATUS_WAIT2STATUS_RESETVAL                    (0x00000000U)
#define CSL_GPMC_STATUS_WAIT2STATUS_W2ACTIVEL                   (0x00000000U)
#define CSL_GPMC_STATUS_WAIT2STATUS_W2ACTIVEH                   (0x00000001U)

#define CSL_GPMC_STATUS_RESETVAL                                (0x00000001U)

/* PREFETCH_CONFIG1 */

#define CSL_GPMC_PREFETCH_CONFIG1_DMAMODE_MASK                  (0x00000004U)
#define CSL_GPMC_PREFETCH_CONFIG1_DMAMODE_SHIFT                 (2U)
#define CSL_GPMC_PREFETCH_CONFIG1_DMAMODE_RESETVAL              (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_DMAMODE_INTERRUPTSYNC         (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_DMAMODE_DMAREQSYNC            (0x00000001U)

#define CSL_GPMC_PREFETCH_CONFIG1_FIFOTHRESHOLD_MASK            (0x00007F00U)
#define CSL_GPMC_PREFETCH_CONFIG1_FIFOTHRESHOLD_SHIFT           (8U)
#define CSL_GPMC_PREFETCH_CONFIG1_FIFOTHRESHOLD_RESETVAL        (0x00000040U)
#define CSL_GPMC_PREFETCH_CONFIG1_FIFOTHRESHOLD_MAX             (0x0000007fU)

#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEENGINE_MASK             (0x00000080U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEENGINE_SHIFT            (7U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEENGINE_RESETVAL         (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEENGINE_PPDISABLED       (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEENGINE_PPENABLED        (0x00000001U)

#define CSL_GPMC_PREFETCH_CONFIG1_CYCLEOPTIMIZATION_MASK        (0x70000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_CYCLEOPTIMIZATION_SHIFT       (28U)
#define CSL_GPMC_PREFETCH_CONFIG1_CYCLEOPTIMIZATION_RESETVAL    (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_CYCLEOPTIMIZATION_MAX         (0x00000007U)

#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS_MASK    (0x08000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS_SHIFT   (27U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS_RESETVAL  (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS_OPTDISABLED  (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS_OPTENABLED  (0x00000001U)

#define CSL_GPMC_PREFETCH_CONFIG1_SYNCHROMODE_MASK              (0x00000008U)
#define CSL_GPMC_PREFETCH_CONFIG1_SYNCHROMODE_SHIFT             (3U)
#define CSL_GPMC_PREFETCH_CONFIG1_SYNCHROMODE_RESETVAL          (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_SYNCHROMODE_ATSTART           (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_SYNCHROMODE_ATSTARTANDWAIT    (0x00000001U)

#define CSL_GPMC_PREFETCH_CONFIG1_ACCESSMODE_MASK               (0x00000001U)
#define CSL_GPMC_PREFETCH_CONFIG1_ACCESSMODE_SHIFT              (0U)
#define CSL_GPMC_PREFETCH_CONFIG1_ACCESSMODE_RESETVAL           (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ACCESSMODE_PREFETCHREAD       (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ACCESSMODE_WRITEPOSTING       (0x00000001U)

#define CSL_GPMC_PREFETCH_CONFIG1_WAITPINSELECTOR_MASK          (0x00000030U)
#define CSL_GPMC_PREFETCH_CONFIG1_WAITPINSELECTOR_SHIFT         (4U)
#define CSL_GPMC_PREFETCH_CONFIG1_WAITPINSELECTOR_RESETVAL      (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_WAITPINSELECTOR_W3            (0x00000003U)
#define CSL_GPMC_PREFETCH_CONFIG1_WAITPINSELECTOR_W0            (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_WAITPINSELECTOR_W1            (0x00000001U)
#define CSL_GPMC_PREFETCH_CONFIG1_WAITPINSELECTOR_W2            (0x00000002U)

#define CSL_GPMC_PREFETCH_CONFIG1_ENGINECSSELECTOR_MASK         (0x07000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENGINECSSELECTOR_SHIFT        (24U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENGINECSSELECTOR_RESETVAL     (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_ENGINECSSELECTOR_MAX          (0x00000007U)

#define CSL_GPMC_PREFETCH_CONFIG1_PFPWWEIGHTEDPRIO_MASK         (0x000F0000U)
#define CSL_GPMC_PREFETCH_CONFIG1_PFPWWEIGHTEDPRIO_SHIFT        (16U)
#define CSL_GPMC_PREFETCH_CONFIG1_PFPWWEIGHTEDPRIO_RESETVAL     (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_PFPWWEIGHTEDPRIO_MAX          (0x0000000fU)

#define CSL_GPMC_PREFETCH_CONFIG1_PFPWENROUNDROBIN_MASK         (0x00800000U)
#define CSL_GPMC_PREFETCH_CONFIG1_PFPWENROUNDROBIN_SHIFT        (23U)
#define CSL_GPMC_PREFETCH_CONFIG1_PFPWENROUNDROBIN_RESETVAL     (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG1_PFPWENROUNDROBIN_RRENABLED    (0x00000001U)
#define CSL_GPMC_PREFETCH_CONFIG1_PFPWENROUNDROBIN_RRDISABLED   (0x00000000U)

#define CSL_GPMC_PREFETCH_CONFIG1_RESETVAL                      (0x00004000U)

/* PREFETCH_CONFIG2 */

#define CSL_GPMC_PREFETCH_CONFIG2_TRANSFERCOUNT_MASK            (0x00003FFFU)
#define CSL_GPMC_PREFETCH_CONFIG2_TRANSFERCOUNT_SHIFT           (0U)
#define CSL_GPMC_PREFETCH_CONFIG2_TRANSFERCOUNT_RESETVAL        (0x00000000U)
#define CSL_GPMC_PREFETCH_CONFIG2_TRANSFERCOUNT_MAX             (0x00003fffU)

#define CSL_GPMC_PREFETCH_CONFIG2_RESETVAL                      (0x00000000U)

/* PREFETCH_CONTROL */

#define CSL_GPMC_PREFETCH_CONTROL_STARTENGINE_MASK              (0x00000001U)
#define CSL_GPMC_PREFETCH_CONTROL_STARTENGINE_SHIFT             (0U)
#define CSL_GPMC_PREFETCH_CONTROL_STARTENGINE_RESETVAL          (0x00000000U)
#define CSL_GPMC_PREFETCH_CONTROL_STARTENGINE_STOP              (0x00000000U)
#define CSL_GPMC_PREFETCH_CONTROL_STARTENGINE_START             (0x00000001U)

#define CSL_GPMC_PREFETCH_CONTROL_RESETVAL                      (0x00000000U)

/* PREFETCH_STATUS */

#define CSL_GPMC_PREFETCH_STATUS_FIFOTHRESHOLDSTATUS_MASK       (0x00010000U)
#define CSL_GPMC_PREFETCH_STATUS_FIFOTHRESHOLDSTATUS_SHIFT      (16U)
#define CSL_GPMC_PREFETCH_STATUS_FIFOTHRESHOLDSTATUS_RESETVAL   (0x00000000U)
#define CSL_GPMC_PREFETCH_STATUS_FIFOTHRESHOLDSTATUS_SMALLERTHANTHRES  (0x00000000U)
#define CSL_GPMC_PREFETCH_STATUS_FIFOTHRESHOLDSTATUS_GREATERTHANTHRES  (0x00000001U)

#define CSL_GPMC_PREFETCH_STATUS_COUNTVALUE_MASK                (0x00003FFFU)
#define CSL_GPMC_PREFETCH_STATUS_COUNTVALUE_SHIFT               (0U)
#define CSL_GPMC_PREFETCH_STATUS_COUNTVALUE_RESETVAL            (0x00000000U)
#define CSL_GPMC_PREFETCH_STATUS_COUNTVALUE_MAX                 (0x00003fffU)

#define CSL_GPMC_PREFETCH_STATUS_FIFOPOINTER_MASK               (0x7F000000U)
#define CSL_GPMC_PREFETCH_STATUS_FIFOPOINTER_SHIFT              (24U)
#define CSL_GPMC_PREFETCH_STATUS_FIFOPOINTER_RESETVAL           (0x00000000U)
#define CSL_GPMC_PREFETCH_STATUS_FIFOPOINTER_MAX                (0x0000007fU)

#define CSL_GPMC_PREFETCH_STATUS_RESETVAL                       (0x00000000U)

/* ECC_CONFIG */

#define CSL_GPMC_ECC_CONFIG_ECC16B_MASK                         (0x00000080U)
#define CSL_GPMC_ECC_CONFIG_ECC16B_SHIFT                        (7U)
#define CSL_GPMC_ECC_CONFIG_ECC16B_RESETVAL                     (0x00000000U)
#define CSL_GPMC_ECC_CONFIG_ECC16B_EIGHTCOL                     (0x00000000U)
#define CSL_GPMC_ECC_CONFIG_ECC16B_SIXTEENCOL                   (0x00000001U)

#define CSL_GPMC_ECC_CONFIG_ECCWRAPMODE_MASK                    (0x00000F00U)
#define CSL_GPMC_ECC_CONFIG_ECCWRAPMODE_SHIFT                   (8U)
#define CSL_GPMC_ECC_CONFIG_ECCWRAPMODE_RESETVAL                (0x00000000U)
#define CSL_GPMC_ECC_CONFIG_ECCWRAPMODE_MAX                     (0x0000000fU)

#define CSL_GPMC_ECC_CONFIG_ECCCS_MASK                          (0x0000000EU)
#define CSL_GPMC_ECC_CONFIG_ECCCS_SHIFT                         (1U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_RESETVAL                      (0x00000000U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_CS0                           (0x00000000U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_CS1                           (0x00000001U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_CS2                           (0x00000002U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_CS3                           (0x00000003U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_CS4                           (0x00000004U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_CS5                           (0x00000005U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_CS6                           (0x00000006U)
#define CSL_GPMC_ECC_CONFIG_ECCCS_CS7                           (0x00000007U)

#define CSL_GPMC_ECC_CONFIG_ECCENABLE_MASK                      (0x00000001U)
#define CSL_GPMC_ECC_CONFIG_ECCENABLE_SHIFT                     (0U)
#define CSL_GPMC_ECC_CONFIG_ECCENABLE_RESETVAL                  (0x00000000U)
#define CSL_GPMC_ECC_CONFIG_ECCENABLE_ECCDISABLED               (0x00000000U)
#define CSL_GPMC_ECC_CONFIG_ECCENABLE_ECCENABLED                (0x00000001U)

#define CSL_GPMC_ECC_CONFIG_ECCBCHTSEL_MASK                     (0x00003000U)
#define CSL_GPMC_ECC_CONFIG_ECCBCHTSEL_SHIFT                    (12U)
#define CSL_GPMC_ECC_CONFIG_ECCBCHTSEL_RESETVAL                 (0x00000001U)
#define CSL_GPMC_ECC_CONFIG_ECCBCHTSEL_MAX                      (0x00000003U)

#define CSL_GPMC_ECC_CONFIG_ECCALGORITHM_MASK                   (0x00010000U)
#define CSL_GPMC_ECC_CONFIG_ECCALGORITHM_SHIFT                  (16U)
#define CSL_GPMC_ECC_CONFIG_ECCALGORITHM_RESETVAL               (0x00000000U)
#define CSL_GPMC_ECC_CONFIG_ECCALGORITHM_MAX                    (0x00000001U)

#define CSL_GPMC_ECC_CONFIG_ECCTOPSECTOR_MASK                   (0x00000070U)
#define CSL_GPMC_ECC_CONFIG_ECCTOPSECTOR_SHIFT                  (4U)
#define CSL_GPMC_ECC_CONFIG_ECCTOPSECTOR_RESETVAL               (0x00000003U)
#define CSL_GPMC_ECC_CONFIG_ECCTOPSECTOR_MAX                    (0x00000007U)

#define CSL_GPMC_ECC_CONFIG_RESETVAL                            (0x00001030U)

/* ECC_CONTROL */

#define CSL_GPMC_ECC_CONTROL_ECCCLEAR_MASK                      (0x00000100U)
#define CSL_GPMC_ECC_CONTROL_ECCCLEAR_SHIFT                     (8U)
#define CSL_GPMC_ECC_CONTROL_ECCCLEAR_RESETVAL                  (0x00000000U)
#define CSL_GPMC_ECC_CONTROL_ECCCLEAR_MAX                       (0x00000001U)

#define CSL_GPMC_ECC_CONTROL_ECCPOINTER_MASK                    (0x0000000FU)
#define CSL_GPMC_ECC_CONTROL_ECCPOINTER_SHIFT                   (0U)
#define CSL_GPMC_ECC_CONTROL_ECCPOINTER_RESETVAL                (0x00000000U)
#define CSL_GPMC_ECC_CONTROL_ECCPOINTER_MAX                     (0x0000000fU)

#define CSL_GPMC_ECC_CONTROL_RESETVAL                           (0x00000000U)

/* ECC_SIZE_CONFIG */

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC3RESULTSIZE_MASK            (0x00000004U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC3RESULTSIZE_SHIFT           (2U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC3RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC3RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC3RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC5RESULTSIZE_MASK            (0x00000010U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC5RESULTSIZE_SHIFT           (4U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC5RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC5RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC5RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC2RESULTSIZE_MASK            (0x00000002U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC2RESULTSIZE_SHIFT           (1U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC2RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC2RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC2RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC1RESULTSIZE_MASK            (0x00000001U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC1RESULTSIZE_SHIFT           (0U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC1RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC1RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC1RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECCSIZE0_MASK                  (0x003FF000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECCSIZE0_SHIFT                 (12U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECCSIZE0_RESETVAL              (0x000000ffU)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECCSIZE0_MAX                   (0x000000ffU)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC7RESULTSIZE_MASK            (0x00000040U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC7RESULTSIZE_SHIFT           (6U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC7RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC7RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC7RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC9RESULTSIZE_MASK            (0x00000100U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC9RESULTSIZE_SHIFT           (8U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC9RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC9RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC9RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC4RESULTSIZE_MASK            (0x00000008U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC4RESULTSIZE_SHIFT           (3U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC4RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC4RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC4RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC6RESULTSIZE_MASK            (0x00000020U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC6RESULTSIZE_SHIFT           (5U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC6RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC6RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC6RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECC8RESULTSIZE_MASK            (0x00000080U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC8RESULTSIZE_SHIFT           (7U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC8RESULTSIZE_RESETVAL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC8RESULTSIZE_SIZE0SEL        (0x00000000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECC8RESULTSIZE_SIZE1SEL        (0x00000001U)

#define CSL_GPMC_ECC_SIZE_CONFIG_ECCSIZE1_MASK                  (0xFFC00000U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECCSIZE1_SHIFT                 (22U)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECCSIZE1_RESETVAL              (0x000000ffU)
#define CSL_GPMC_ECC_SIZE_CONFIG_ECCSIZE1_MAX                   (0x000000ffU)

#define CSL_GPMC_ECC_SIZE_CONFIG_RESETVAL                       (0xfffff000U)

/* ECC_RESULT */

#define CSL_GPMC_ECC_RESULT_P512O_MASK                          (0x02000000U)
#define CSL_GPMC_ECC_RESULT_P512O_SHIFT                         (25U)
#define CSL_GPMC_ECC_RESULT_P512O_RESETVAL                      (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P512O_MAX                           (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P256O_MASK                          (0x01000000U)
#define CSL_GPMC_ECC_RESULT_P256O_SHIFT                         (24U)
#define CSL_GPMC_ECC_RESULT_P256O_RESETVAL                      (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P256O_MAX                           (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P128O_MASK                          (0x00800000U)
#define CSL_GPMC_ECC_RESULT_P128O_SHIFT                         (23U)
#define CSL_GPMC_ECC_RESULT_P128O_RESETVAL                      (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P128O_MAX                           (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P2048E_MASK                         (0x00000800U)
#define CSL_GPMC_ECC_RESULT_P2048E_SHIFT                        (11U)
#define CSL_GPMC_ECC_RESULT_P2048E_RESETVAL                     (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P2048E_MAX                          (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P64O_MASK                           (0x00400000U)
#define CSL_GPMC_ECC_RESULT_P64O_SHIFT                          (22U)
#define CSL_GPMC_ECC_RESULT_P64O_RESETVAL                       (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P64O_MAX                            (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P4E_MASK                            (0x00000004U)
#define CSL_GPMC_ECC_RESULT_P4E_SHIFT                           (2U)
#define CSL_GPMC_ECC_RESULT_P4E_RESETVAL                        (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P4E_MAX                             (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P1024E_MASK                         (0x00000400U)
#define CSL_GPMC_ECC_RESULT_P1024E_SHIFT                        (10U)
#define CSL_GPMC_ECC_RESULT_P1024E_RESETVAL                     (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P1024E_MAX                          (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P32O_MASK                           (0x00200000U)
#define CSL_GPMC_ECC_RESULT_P32O_SHIFT                          (21U)
#define CSL_GPMC_ECC_RESULT_P32O_RESETVAL                       (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P32O_MAX                            (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P2E_MASK                            (0x00000002U)
#define CSL_GPMC_ECC_RESULT_P2E_SHIFT                           (1U)
#define CSL_GPMC_ECC_RESULT_P2E_RESETVAL                        (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P2E_MAX                             (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P512E_MASK                          (0x00000200U)
#define CSL_GPMC_ECC_RESULT_P512E_SHIFT                         (9U)
#define CSL_GPMC_ECC_RESULT_P512E_RESETVAL                      (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P512E_MAX                           (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P16O_MASK                           (0x00100000U)
#define CSL_GPMC_ECC_RESULT_P16O_SHIFT                          (20U)
#define CSL_GPMC_ECC_RESULT_P16O_RESETVAL                       (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P16O_MAX                            (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P1E_MASK                            (0x00000001U)
#define CSL_GPMC_ECC_RESULT_P1E_SHIFT                           (0U)
#define CSL_GPMC_ECC_RESULT_P1E_RESETVAL                        (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P1E_MAX                             (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P256E_MASK                          (0x00000100U)
#define CSL_GPMC_ECC_RESULT_P256E_SHIFT                         (8U)
#define CSL_GPMC_ECC_RESULT_P256E_RESETVAL                      (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P256E_MAX                           (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P8O_MASK                            (0x00080000U)
#define CSL_GPMC_ECC_RESULT_P8O_SHIFT                           (19U)
#define CSL_GPMC_ECC_RESULT_P8O_RESETVAL                        (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P8O_MAX                             (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P2048O_MASK                         (0x08000000U)
#define CSL_GPMC_ECC_RESULT_P2048O_SHIFT                        (27U)
#define CSL_GPMC_ECC_RESULT_P2048O_RESETVAL                     (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P2048O_MAX                          (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P128E_MASK                          (0x00000080U)
#define CSL_GPMC_ECC_RESULT_P128E_SHIFT                         (7U)
#define CSL_GPMC_ECC_RESULT_P128E_RESETVAL                      (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P128E_MAX                           (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P4O_MASK                            (0x00040000U)
#define CSL_GPMC_ECC_RESULT_P4O_SHIFT                           (18U)
#define CSL_GPMC_ECC_RESULT_P4O_RESETVAL                        (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P4O_MAX                             (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P1024O_MASK                         (0x04000000U)
#define CSL_GPMC_ECC_RESULT_P1024O_SHIFT                        (26U)
#define CSL_GPMC_ECC_RESULT_P1024O_RESETVAL                     (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P1024O_MAX                          (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P64E_MASK                           (0x00000040U)
#define CSL_GPMC_ECC_RESULT_P64E_SHIFT                          (6U)
#define CSL_GPMC_ECC_RESULT_P64E_RESETVAL                       (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P64E_MAX                            (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P2O_MASK                            (0x00020000U)
#define CSL_GPMC_ECC_RESULT_P2O_SHIFT                           (17U)
#define CSL_GPMC_ECC_RESULT_P2O_RESETVAL                        (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P2O_MAX                             (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P32E_MASK                           (0x00000020U)
#define CSL_GPMC_ECC_RESULT_P32E_SHIFT                          (5U)
#define CSL_GPMC_ECC_RESULT_P32E_RESETVAL                       (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P32E_MAX                            (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P1O_MASK                            (0x00010000U)
#define CSL_GPMC_ECC_RESULT_P1O_SHIFT                           (16U)
#define CSL_GPMC_ECC_RESULT_P1O_RESETVAL                        (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P1O_MAX                             (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P16E_MASK                           (0x00000010U)
#define CSL_GPMC_ECC_RESULT_P16E_SHIFT                          (4U)
#define CSL_GPMC_ECC_RESULT_P16E_RESETVAL                       (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P16E_MAX                            (0x00000001U)

#define CSL_GPMC_ECC_RESULT_P8E_MASK                            (0x00000008U)
#define CSL_GPMC_ECC_RESULT_P8E_SHIFT                           (3U)
#define CSL_GPMC_ECC_RESULT_P8E_RESETVAL                        (0x00000000U)
#define CSL_GPMC_ECC_RESULT_P8E_MAX                             (0x00000001U)

#define CSL_GPMC_ECC_RESULT_RESETVAL                            (0x00000000U)

/* TESTMODE_CTRL */

#define CSL_GPMC_TESTMODE_CTRL_GPMCFDBKEN_MASK                  (0x00000001U)
#define CSL_GPMC_TESTMODE_CTRL_GPMCFDBKEN_SHIFT                 (0U)
#define CSL_GPMC_TESTMODE_CTRL_GPMCFDBKEN_RESETVAL              (0x00000000U)
#define CSL_GPMC_TESTMODE_CTRL_GPMCFDBKEN_NOEFFECT              (0x00000000U)
#define CSL_GPMC_TESTMODE_CTRL_GPMCFDBKEN_FBMODE                (0x00000001U)

#define CSL_GPMC_TESTMODE_CTRL_RESETVAL                         (0x00000000U)

/* BCH_RESULT_0 */

#define CSL_GPMC_BCH_RESULT_0_BCH_RESULT_0_MASK                 (0xFFFFFFFFU)
#define CSL_GPMC_BCH_RESULT_0_BCH_RESULT_0_SHIFT                (0U)
#define CSL_GPMC_BCH_RESULT_0_BCH_RESULT_0_RESETVAL             (0x00000000U)
#define CSL_GPMC_BCH_RESULT_0_BCH_RESULT_0_MAX                  (0xffffffffU)

#define CSL_GPMC_BCH_RESULT_0_RESETVAL                          (0x00000000U)

/* BCH_RESULT_1 */

#define CSL_GPMC_BCH_RESULT_1_BCH_RESULT_1_MASK                 (0xFFFFFFFFU)
#define CSL_GPMC_BCH_RESULT_1_BCH_RESULT_1_SHIFT                (0U)
#define CSL_GPMC_BCH_RESULT_1_BCH_RESULT_1_RESETVAL             (0x00000000U)
#define CSL_GPMC_BCH_RESULT_1_BCH_RESULT_1_MAX                  (0xffffffffU)

#define CSL_GPMC_BCH_RESULT_1_RESETVAL                          (0x00000000U)

/* BCH_RESULT_2 */

#define CSL_GPMC_BCH_RESULT_2_BCH_RESULT_2_MASK                 (0xFFFFFFFFU)
#define CSL_GPMC_BCH_RESULT_2_BCH_RESULT_2_SHIFT                (0U)
#define CSL_GPMC_BCH_RESULT_2_BCH_RESULT_2_RESETVAL             (0x00000000U)
#define CSL_GPMC_BCH_RESULT_2_BCH_RESULT_2_MAX                  (0xffffffffU)

#define CSL_GPMC_BCH_RESULT_2_RESETVAL                          (0x00000000U)

/* BCH_RESULT_3 */

#define CSL_GPMC_BCH_RESULT_3_BCH_RESULT_3_MASK                 (0xFFFFFFFFU)
#define CSL_GPMC_BCH_RESULT_3_BCH_RESULT_3_SHIFT                (0U)
#define CSL_GPMC_BCH_RESULT_3_BCH_RESULT_3_RESETVAL             (0x00000000U)
#define CSL_GPMC_BCH_RESULT_3_BCH_RESULT_3_MAX                  (0xffffffffU)

#define CSL_GPMC_BCH_RESULT_3_RESETVAL                          (0x00000000U)

/* BCH_SWDATA */

#define CSL_GPMC_BCH_SWDATA_BCH_DATA_MASK                       (0x0000FFFFU)
#define CSL_GPMC_BCH_SWDATA_BCH_DATA_SHIFT                      (0U)
#define CSL_GPMC_BCH_SWDATA_BCH_DATA_RESETVAL                   (0x00000000U)
#define CSL_GPMC_BCH_SWDATA_BCH_DATA_MAX                        (0x0000ffffU)

#define CSL_GPMC_BCH_SWDATA_RESETVAL                            (0x00000000U)

/* BCH_RESULT_5 */

#define CSL_GPMC_BCH_RESULT_5_BCH_RESULT_5_MASK                 (0xFFFFFFFFU)
#define CSL_GPMC_BCH_RESULT_5_BCH_RESULT_5_SHIFT                (0U)
#define CSL_GPMC_BCH_RESULT_5_BCH_RESULT_5_RESETVAL             (0x00000000U)
#define CSL_GPMC_BCH_RESULT_5_BCH_RESULT_5_MAX                  (0xffffffffU)

#define CSL_GPMC_BCH_RESULT_5_RESETVAL                          (0x00000000U)

/* BCH_RESULT_6 */

#define CSL_GPMC_BCH_RESULT_6_BCH_RESULT_6_MASK                 (0x0000FFFFU)
#define CSL_GPMC_BCH_RESULT_6_BCH_RESULT_6_SHIFT                (0U)
#define CSL_GPMC_BCH_RESULT_6_BCH_RESULT_6_RESETVAL             (0x00000000U)
#define CSL_GPMC_BCH_RESULT_6_BCH_RESULT_6_MAX                  (0x0000ffffU)

#define CSL_GPMC_BCH_RESULT_6_RESETVAL                          (0x00000000U)

/* BCH_RESULT_4 */

#define CSL_GPMC_BCH_RESULT_4_BCH_RESULT_4_MASK                 (0xFFFFFFFFU)
#define CSL_GPMC_BCH_RESULT_4_BCH_RESULT_4_SHIFT                (0U)
#define CSL_GPMC_BCH_RESULT_4_BCH_RESULT_4_RESETVAL             (0x00000000U)
#define CSL_GPMC_BCH_RESULT_4_BCH_RESULT_4_MAX                  (0xffffffffU)

#define CSL_GPMC_BCH_RESULT_4_RESETVAL                          (0x00000000U)

/* CONFIG1 */

#define CSL_GPMC_CONFIG1_DEVICESIZE_MASK                        (0x00003000U)
#define CSL_GPMC_CONFIG1_DEVICESIZE_SHIFT                       (12U)
#define CSL_GPMC_CONFIG1_DEVICESIZE_RESETVAL                    (0x00000000U)
#define CSL_GPMC_CONFIG1_DEVICESIZE_SIXTEENBITS                 (0x00000001U)
#define CSL_GPMC_CONFIG1_DEVICESIZE_EIGHTBITS                   (0x00000000U)
#define CSL_GPMC_CONFIG1_DEVICESIZE_THIRTYTWOBITS               (0x00000002U)
#define CSL_GPMC_CONFIG1_DEVICESIZE_RES                         (0x00000003U)

#define CSL_GPMC_CONFIG1_WRITEMULTIPLE_MASK                     (0x10000000U)
#define CSL_GPMC_CONFIG1_WRITEMULTIPLE_SHIFT                    (28U)
#define CSL_GPMC_CONFIG1_WRITEMULTIPLE_RESETVAL                 (0x00000000U)
#define CSL_GPMC_CONFIG1_WRITEMULTIPLE_WRMULTIPLE               (0x00000001U)
#define CSL_GPMC_CONFIG1_WRITEMULTIPLE_WRSINGLE                 (0x00000000U)

#define CSL_GPMC_CONFIG1_WRAPBURST_MASK                         (0x80000000U)
#define CSL_GPMC_CONFIG1_WRAPBURST_SHIFT                        (31U)
#define CSL_GPMC_CONFIG1_WRAPBURST_RESETVAL                     (0x00000000U)
#define CSL_GPMC_CONFIG1_WRAPBURST_WRAPNOTSUPP                  (0x00000000U)
#define CSL_GPMC_CONFIG1_WRAPBURST_WRAPSUPP                     (0x00000001U)

#define CSL_GPMC_CONFIG1_READTYPE_MASK                          (0x20000000U)
#define CSL_GPMC_CONFIG1_READTYPE_SHIFT                         (29U)
#define CSL_GPMC_CONFIG1_READTYPE_RESETVAL                      (0x00000000U)
#define CSL_GPMC_CONFIG1_READTYPE_RDASYNC                       (0x00000000U)
#define CSL_GPMC_CONFIG1_READTYPE_RDSYNC                        (0x00000001U)

#define CSL_GPMC_CONFIG1_READMULTIPLE_MASK                      (0x40000000U)
#define CSL_GPMC_CONFIG1_READMULTIPLE_SHIFT                     (30U)
#define CSL_GPMC_CONFIG1_READMULTIPLE_RESETVAL                  (0x00000000U)
#define CSL_GPMC_CONFIG1_READMULTIPLE_RDSINGLE                  (0x00000000U)
#define CSL_GPMC_CONFIG1_READMULTIPLE_RDMULTIPLE                (0x00000001U)

#define CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_MASK               (0x00000010U)
#define CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_SHIFT              (4U)
#define CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_RESETVAL           (0x00000000U)
#define CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_X1                 (0x00000000U)
#define CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_X2                 (0x00000001U)

#define CSL_GPMC_CONFIG1_DEVICETYPE_MASK                        (0x00000C00U)
#define CSL_GPMC_CONFIG1_DEVICETYPE_SHIFT                       (10U)
#define CSL_GPMC_CONFIG1_DEVICETYPE_RESETVAL                    (0x00000000U)
#define CSL_GPMC_CONFIG1_DEVICETYPE_RES2                        (0x00000003U)
#define CSL_GPMC_CONFIG1_DEVICETYPE_NORLIKE                     (0x00000000U)
#define CSL_GPMC_CONFIG1_DEVICETYPE_NANDLIKE                    (0x00000002U)
#define CSL_GPMC_CONFIG1_DEVICETYPE_RES1                        (0x00000001U)

#define CSL_GPMC_CONFIG1_GPMCFCLKDIVIDER_MASK                   (0x00000003U)
#define CSL_GPMC_CONFIG1_GPMCFCLKDIVIDER_SHIFT                  (0U)
#define CSL_GPMC_CONFIG1_GPMCFCLKDIVIDER_RESETVAL               (0x00000000U)
#define CSL_GPMC_CONFIG1_GPMCFCLKDIVIDER_DIVBY4                 (0x00000003U)
#define CSL_GPMC_CONFIG1_GPMCFCLKDIVIDER_DIVBY2                 (0x00000001U)
#define CSL_GPMC_CONFIG1_GPMCFCLKDIVIDER_DIVBY1                 (0x00000000U)
#define CSL_GPMC_CONFIG1_GPMCFCLKDIVIDER_DIVBY3                 (0x00000002U)

#define CSL_GPMC_CONFIG1_MUXADDDATA_MASK                        (0x00000300U)
#define CSL_GPMC_CONFIG1_MUXADDDATA_SHIFT                       (8U)
#define CSL_GPMC_CONFIG1_MUXADDDATA_RESETVAL                    (0x00000000U)
#define CSL_GPMC_CONFIG1_MUXADDDATA_MUX                         (0x00000002U)
#define CSL_GPMC_CONFIG1_MUXADDDATA_NONMUX                      (0x00000000U)
#define CSL_GPMC_CONFIG1_MUXADDDATA_AADMUX                      (0x00000001U)
#define CSL_GPMC_CONFIG1_MUXADDDATA_RESERVED                    (0x00000003U)

#define CSL_GPMC_CONFIG1_WAITMONITORINGTIME_MASK                (0x000C0000U)
#define CSL_GPMC_CONFIG1_WAITMONITORINGTIME_SHIFT               (18U)
#define CSL_GPMC_CONFIG1_WAITMONITORINGTIME_RESETVAL            (0x00000000U)
#define CSL_GPMC_CONFIG1_WAITMONITORINGTIME_NOTDEFINED          (0x00000003U)
#define CSL_GPMC_CONFIG1_WAITMONITORINGTIME_TWODEVICEB4         (0x00000002U)
#define CSL_GPMC_CONFIG1_WAITMONITORINGTIME_ONEDEVICEB4         (0x00000001U)
#define CSL_GPMC_CONFIG1_WAITMONITORINGTIME_ATVALID             (0x00000000U)

#define CSL_GPMC_CONFIG1_WAITREADMONITORING_MASK                (0x00400000U)
#define CSL_GPMC_CONFIG1_WAITREADMONITORING_SHIFT               (22U)
#define CSL_GPMC_CONFIG1_WAITREADMONITORING_RESETVAL            (0x00000000U)
#define CSL_GPMC_CONFIG1_WAITREADMONITORING_WMONIT              (0x00000001U)
#define CSL_GPMC_CONFIG1_WAITREADMONITORING_WNOTMONIT           (0x00000000U)

#define CSL_GPMC_CONFIG1_WAITPINSELECT_MASK                     (0x00030000U)
#define CSL_GPMC_CONFIG1_WAITPINSELECT_SHIFT                    (16U)
#define CSL_GPMC_CONFIG1_WAITPINSELECT_RESETVAL                 (0x00000000U)
#define CSL_GPMC_CONFIG1_WAITPINSELECT_W2                       (0x00000002U)
#define CSL_GPMC_CONFIG1_WAITPINSELECT_W1                       (0x00000001U)
#define CSL_GPMC_CONFIG1_WAITPINSELECT_W3                       (0x00000003U)
#define CSL_GPMC_CONFIG1_WAITPINSELECT_W0                       (0x00000000U)

#define CSL_GPMC_CONFIG1_CLKACTIVATIONTIME_MASK                 (0x06000000U)
#define CSL_GPMC_CONFIG1_CLKACTIVATIONTIME_SHIFT                (25U)
#define CSL_GPMC_CONFIG1_CLKACTIVATIONTIME_RESETVAL             (0x00000000U)
#define CSL_GPMC_CONFIG1_CLKACTIVATIONTIME_ATSTART              (0x00000000U)
#define CSL_GPMC_CONFIG1_CLKACTIVATIONTIME_TWOCLKB4             (0x00000002U)
#define CSL_GPMC_CONFIG1_CLKACTIVATIONTIME_ONECLKB4             (0x00000001U)
#define CSL_GPMC_CONFIG1_CLKACTIVATIONTIME_NOTDEFINED           (0x00000003U)

#define CSL_GPMC_CONFIG1_WAITWRITEMONITORING_MASK               (0x00200000U)
#define CSL_GPMC_CONFIG1_WAITWRITEMONITORING_SHIFT              (21U)
#define CSL_GPMC_CONFIG1_WAITWRITEMONITORING_RESETVAL           (0x00000000U)
#define CSL_GPMC_CONFIG1_WAITWRITEMONITORING_WNOTMONIT          (0x00000000U)
#define CSL_GPMC_CONFIG1_WAITWRITEMONITORING_WMONIT             (0x00000001U)

#define CSL_GPMC_CONFIG1_ATTACHEDDEVICEPAGELENGTH_MASK          (0x01800000U)
#define CSL_GPMC_CONFIG1_ATTACHEDDEVICEPAGELENGTH_SHIFT         (23U)
#define CSL_GPMC_CONFIG1_ATTACHEDDEVICEPAGELENGTH_RESETVAL      (0x00000000U)
#define CSL_GPMC_CONFIG1_ATTACHEDDEVICEPAGELENGTH_EIGHT         (0x00000001U)
#define CSL_GPMC_CONFIG1_ATTACHEDDEVICEPAGELENGTH_SIXTEEN       (0x00000002U)
#define CSL_GPMC_CONFIG1_ATTACHEDDEVICEPAGELENGTH_THIRTYTWO     (0x00000003U)
#define CSL_GPMC_CONFIG1_ATTACHEDDEVICEPAGELENGTH_FOUR          (0x00000000U)

#define CSL_GPMC_CONFIG1_WRITETYPE_MASK                         (0x08000000U)
#define CSL_GPMC_CONFIG1_WRITETYPE_SHIFT                        (27U)
#define CSL_GPMC_CONFIG1_WRITETYPE_RESETVAL                     (0x00000000U)
#define CSL_GPMC_CONFIG1_WRITETYPE_WRASYNC                      (0x00000000U)
#define CSL_GPMC_CONFIG1_WRITETYPE_WRSYNC                       (0x00000001U)

#define CSL_GPMC_CONFIG1_RESETVAL                               (0x00000000U)

/* CONFIG2 */

#define CSL_GPMC_CONFIG2_CSONTIME_MASK                          (0x0000000FU)
#define CSL_GPMC_CONFIG2_CSONTIME_SHIFT                         (0U)
#define CSL_GPMC_CONFIG2_CSONTIME_RESETVAL                      (0x00000001U)
#define CSL_GPMC_CONFIG2_CSONTIME_MAX                           (0x0000000fU)

#define CSL_GPMC_CONFIG2_CSWROFFTIME_MASK                       (0x001F0000U)
#define CSL_GPMC_CONFIG2_CSWROFFTIME_SHIFT                      (16U)
#define CSL_GPMC_CONFIG2_CSWROFFTIME_RESETVAL                   (0x00000010U)
#define CSL_GPMC_CONFIG2_CSWROFFTIME_MAX                        (0x0000001fU)

#define CSL_GPMC_CONFIG2_CSRDOFFTIME_MASK                       (0x00001F00U)
#define CSL_GPMC_CONFIG2_CSRDOFFTIME_SHIFT                      (8U)
#define CSL_GPMC_CONFIG2_CSRDOFFTIME_RESETVAL                   (0x00000010U)
#define CSL_GPMC_CONFIG2_CSRDOFFTIME_MAX                        (0x0000001fU)

#define CSL_GPMC_CONFIG2_CSEXTRADELAY_MASK                      (0x00000080U)
#define CSL_GPMC_CONFIG2_CSEXTRADELAY_SHIFT                     (7U)
#define CSL_GPMC_CONFIG2_CSEXTRADELAY_RESETVAL                  (0x00000000U)
#define CSL_GPMC_CONFIG2_CSEXTRADELAY_NOTDELAYED                (0x00000000U)
#define CSL_GPMC_CONFIG2_CSEXTRADELAY_DELAYED                   (0x00000001U)

#define CSL_GPMC_CONFIG2_RESETVAL                               (0x00101001U)

/* CONFIG3 */

#define CSL_GPMC_CONFIG3_ADVEXTRADELAY_MASK                     (0x00000080U)
#define CSL_GPMC_CONFIG3_ADVEXTRADELAY_SHIFT                    (7U)
#define CSL_GPMC_CONFIG3_ADVEXTRADELAY_RESETVAL                 (0x00000000U)
#define CSL_GPMC_CONFIG3_ADVEXTRADELAY_NOTDELAYED               (0x00000000U)
#define CSL_GPMC_CONFIG3_ADVEXTRADELAY_DELAYED                  (0x00000001U)

#define CSL_GPMC_CONFIG3_ADVAADMUXONTIME_MASK                   (0x00000070U)
#define CSL_GPMC_CONFIG3_ADVAADMUXONTIME_SHIFT                  (4U)
#define CSL_GPMC_CONFIG3_ADVAADMUXONTIME_RESETVAL               (0x00000001U)
#define CSL_GPMC_CONFIG3_ADVAADMUXONTIME_MAX                    (0x00000007U)

#define CSL_GPMC_CONFIG3_ADVONTIME_MASK                         (0x0000000FU)
#define CSL_GPMC_CONFIG3_ADVONTIME_SHIFT                        (0U)
#define CSL_GPMC_CONFIG3_ADVONTIME_RESETVAL                     (0x00000004U)
#define CSL_GPMC_CONFIG3_ADVONTIME_MAX                          (0x0000000fU)

#define CSL_GPMC_CONFIG3_ADVWROFFTIME_MASK                      (0x001F0000U)
#define CSL_GPMC_CONFIG3_ADVWROFFTIME_SHIFT                     (16U)
#define CSL_GPMC_CONFIG3_ADVWROFFTIME_RESETVAL                  (0x00000006U)
#define CSL_GPMC_CONFIG3_ADVWROFFTIME_MAX                       (0x0000001fU)

#define CSL_GPMC_CONFIG3_ADVRDOFFTIME_MASK                      (0x00001F00U)
#define CSL_GPMC_CONFIG3_ADVRDOFFTIME_SHIFT                     (8U)
#define CSL_GPMC_CONFIG3_ADVRDOFFTIME_RESETVAL                  (0x00000005U)
#define CSL_GPMC_CONFIG3_ADVRDOFFTIME_MAX                       (0x0000001fU)

#define CSL_GPMC_CONFIG3_ADVAADMUXRDOFFTIME_MASK                (0x07000000U)
#define CSL_GPMC_CONFIG3_ADVAADMUXRDOFFTIME_SHIFT               (24U)
#define CSL_GPMC_CONFIG3_ADVAADMUXRDOFFTIME_RESETVAL            (0x00000002U)
#define CSL_GPMC_CONFIG3_ADVAADMUXRDOFFTIME_MAX                 (0x00000007U)

#define CSL_GPMC_CONFIG3_ADVAADMUXWROFFTIME_MASK                (0x70000000U)
#define CSL_GPMC_CONFIG3_ADVAADMUXWROFFTIME_SHIFT               (28U)
#define CSL_GPMC_CONFIG3_ADVAADMUXWROFFTIME_RESETVAL            (0x00000002U)
#define CSL_GPMC_CONFIG3_ADVAADMUXWROFFTIME_MAX                 (0x00000007U)

#define CSL_GPMC_CONFIG3_RESETVAL                               (0x22060514U)

/* CONFIG4 */

#define CSL_GPMC_CONFIG4_OEONTIME_MASK                          (0x0000000FU)
#define CSL_GPMC_CONFIG4_OEONTIME_SHIFT                         (0U)
#define CSL_GPMC_CONFIG4_OEONTIME_RESETVAL                      (0x00000006U)
#define CSL_GPMC_CONFIG4_OEONTIME_MAX                           (0x0000000fU)

#define CSL_GPMC_CONFIG4_WEONTIME_MASK                          (0x000F0000U)
#define CSL_GPMC_CONFIG4_WEONTIME_SHIFT                         (16U)
#define CSL_GPMC_CONFIG4_WEONTIME_RESETVAL                      (0x00000005U)
#define CSL_GPMC_CONFIG4_WEONTIME_MAX                           (0x0000000fU)

#define CSL_GPMC_CONFIG4_OEOFFTIME_MASK                         (0x00001F00U)
#define CSL_GPMC_CONFIG4_OEOFFTIME_SHIFT                        (8U)
#define CSL_GPMC_CONFIG4_OEOFFTIME_RESETVAL                     (0x00000010U)
#define CSL_GPMC_CONFIG4_OEOFFTIME_MAX                          (0x0000001fU)

#define CSL_GPMC_CONFIG4_WEOFFTIME_MASK                         (0x1F000000U)
#define CSL_GPMC_CONFIG4_WEOFFTIME_SHIFT                        (24U)
#define CSL_GPMC_CONFIG4_WEOFFTIME_RESETVAL                     (0x00000010U)
#define CSL_GPMC_CONFIG4_WEOFFTIME_MAX                          (0x0000001fU)

#define CSL_GPMC_CONFIG4_OEEXTRADELAY_MASK                      (0x00000080U)
#define CSL_GPMC_CONFIG4_OEEXTRADELAY_SHIFT                     (7U)
#define CSL_GPMC_CONFIG4_OEEXTRADELAY_RESETVAL                  (0x00000000U)
#define CSL_GPMC_CONFIG4_OEEXTRADELAY_NOTDELAYED                (0x00000000U)
#define CSL_GPMC_CONFIG4_OEEXTRADELAY_DELAYED                   (0x00000001U)

#define CSL_GPMC_CONFIG4_WEEXTRADELAY_MASK                      (0x00800000U)
#define CSL_GPMC_CONFIG4_WEEXTRADELAY_SHIFT                     (23U)
#define CSL_GPMC_CONFIG4_WEEXTRADELAY_RESETVAL                  (0x00000000U)
#define CSL_GPMC_CONFIG4_WEEXTRADELAY_NOTDELAYED                (0x00000000U)
#define CSL_GPMC_CONFIG4_WEEXTRADELAY_DELAYED                   (0x00000001U)

#define CSL_GPMC_CONFIG4_OEAADMUXOFFTIME_MASK                   (0x0000E000U)
#define CSL_GPMC_CONFIG4_OEAADMUXOFFTIME_SHIFT                  (13U)
#define CSL_GPMC_CONFIG4_OEAADMUXOFFTIME_RESETVAL               (0x00000003U)
#define CSL_GPMC_CONFIG4_OEAADMUXOFFTIME_MAX                    (0x00000007U)

#define CSL_GPMC_CONFIG4_OEAADMUXONTIME_MASK                    (0x00000070U)
#define CSL_GPMC_CONFIG4_OEAADMUXONTIME_SHIFT                   (4U)
#define CSL_GPMC_CONFIG4_OEAADMUXONTIME_RESETVAL                (0x00000001U)
#define CSL_GPMC_CONFIG4_OEAADMUXONTIME_MAX                     (0x00000007U)

#define CSL_GPMC_CONFIG4_RESETVAL                               (0x10057016U)

/* CONFIG5 */

#define CSL_GPMC_CONFIG5_WRCYCLETIME_MASK                       (0x00001F00U)
#define CSL_GPMC_CONFIG5_WRCYCLETIME_SHIFT                      (8U)
#define CSL_GPMC_CONFIG5_WRCYCLETIME_RESETVAL                   (0x00000011U)
#define CSL_GPMC_CONFIG5_WRCYCLETIME_MAX                        (0x0000001fU)

#define CSL_GPMC_CONFIG5_RDCYCLETIME_MASK                       (0x0000001FU)
#define CSL_GPMC_CONFIG5_RDCYCLETIME_SHIFT                      (0U)
#define CSL_GPMC_CONFIG5_RDCYCLETIME_RESETVAL                   (0x00000011U)
#define CSL_GPMC_CONFIG5_RDCYCLETIME_MAX                        (0x0000001fU)

#define CSL_GPMC_CONFIG5_PAGEBURSTACCESSTIME_MASK               (0x0F000000U)
#define CSL_GPMC_CONFIG5_PAGEBURSTACCESSTIME_SHIFT              (24U)
#define CSL_GPMC_CONFIG5_PAGEBURSTACCESSTIME_RESETVAL           (0x00000001U)
#define CSL_GPMC_CONFIG5_PAGEBURSTACCESSTIME_MAX                (0x0000000fU)

#define CSL_GPMC_CONFIG5_RDACCESSTIME_MASK                      (0x001F0000U)
#define CSL_GPMC_CONFIG5_RDACCESSTIME_SHIFT                     (16U)
#define CSL_GPMC_CONFIG5_RDACCESSTIME_RESETVAL                  (0x0000000fU)
#define CSL_GPMC_CONFIG5_RDACCESSTIME_MAX                       (0x0000001fU)

#define CSL_GPMC_CONFIG5_RESETVAL                               (0x010f1111U)

/* CONFIG6 */

#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_MASK               (0x00000040U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_SHIFT              (6U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_RESETVAL           (0x00000000U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_NOC2CDELAY         (0x00000000U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_C2CDELAY           (0x00000001U)

#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDELAY_MASK                  (0x00000F00U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDELAY_SHIFT                 (8U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDELAY_RESETVAL              (0x00000000U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLEDELAY_MAX                   (0x0000000fU)

#define CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_MASK               (0x00000080U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_SHIFT              (7U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_RESETVAL           (0x00000000U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_NOC2CDELAY         (0x00000000U)
#define CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_C2CDELAY           (0x00000001U)

#define CSL_GPMC_CONFIG6_BUSTURNAROUND_MASK                     (0x0000000FU)
#define CSL_GPMC_CONFIG6_BUSTURNAROUND_SHIFT                    (0U)
#define CSL_GPMC_CONFIG6_BUSTURNAROUND_RESETVAL                 (0x00000000U)
#define CSL_GPMC_CONFIG6_BUSTURNAROUND_MAX                      (0x0000000fU)

#define CSL_GPMC_CONFIG6_WRDATAONADMUXBUS_MASK                  (0x000F0000U)
#define CSL_GPMC_CONFIG6_WRDATAONADMUXBUS_SHIFT                 (16U)
#define CSL_GPMC_CONFIG6_WRDATAONADMUXBUS_RESETVAL              (0x00000007U)
#define CSL_GPMC_CONFIG6_WRDATAONADMUXBUS_MAX                   (0x0000000fU)

#define CSL_GPMC_CONFIG6_WRACCESSTIME_MASK                      (0x1F000000U)
#define CSL_GPMC_CONFIG6_WRACCESSTIME_SHIFT                     (24U)
#define CSL_GPMC_CONFIG6_WRACCESSTIME_RESETVAL                  (0x0000000fU)
#define CSL_GPMC_CONFIG6_WRACCESSTIME_MAX                       (0x0000001fU)

#define CSL_GPMC_CONFIG6_RESETVAL                               (0x8f070000U)

/* CONFIG7 */

#define CSL_GPMC_CONFIG7_BASEADDRESS_MASK                       (0x0000003FU)
#define CSL_GPMC_CONFIG7_BASEADDRESS_SHIFT                      (0U)
#define CSL_GPMC_CONFIG7_BASEADDRESS_RESETVAL                   (0x00000000U)
#define CSL_GPMC_CONFIG7_BASEADDRESS_MAX                        (0x0000003fU)

#define CSL_GPMC_CONFIG7_MASKADDRESS_MASK                       (0x00000F00U)
#define CSL_GPMC_CONFIG7_MASKADDRESS_SHIFT                      (8U)
#define CSL_GPMC_CONFIG7_MASKADDRESS_RESETVAL                   (0x0000000fU)
#define CSL_GPMC_CONFIG7_MASKADDRESS_MAX                        (0x0000000fU)

#define CSL_GPMC_CONFIG7_CSVALID_MASK                           (0x00000040U)
#define CSL_GPMC_CONFIG7_CSVALID_SHIFT                          (6U)
#define CSL_GPMC_CONFIG7_CSVALID_RESETVAL                       (0x00000001U)
#define CSL_GPMC_CONFIG7_CSVALID_CSDISABLED                     (0x00000000U)
#define CSL_GPMC_CONFIG7_CSVALID_CSENABLED                      (0x00000001U)

#define CSL_GPMC_CONFIG7_RESETVAL                               (0x00000f40U)

/* NAND_COMMAND */

#define CSL_GPMC_NAND_COMMAND_GPMC_NAND_COMMAND_0_MASK          (0xFFFFFFFFU)
#define CSL_GPMC_NAND_COMMAND_GPMC_NAND_COMMAND_0_SHIFT         (0U)
#define CSL_GPMC_NAND_COMMAND_GPMC_NAND_COMMAND_0_RESETVAL      (0x00000000U)
#define CSL_GPMC_NAND_COMMAND_GPMC_NAND_COMMAND_0_MAX           (0xffffffffU)

#define CSL_GPMC_NAND_COMMAND_RESETVAL                          (0x00000000U)

/* NAND_ADDRESS */

#define CSL_GPMC_NAND_ADDRESS_GPMC_NAND_ADDRESS_0_MASK          (0xFFFFFFFFU)
#define CSL_GPMC_NAND_ADDRESS_GPMC_NAND_ADDRESS_0_SHIFT         (0U)
#define CSL_GPMC_NAND_ADDRESS_GPMC_NAND_ADDRESS_0_RESETVAL      (0x00000000U)
#define CSL_GPMC_NAND_ADDRESS_GPMC_NAND_ADDRESS_0_MAX           (0xffffffffU)

#define CSL_GPMC_NAND_ADDRESS_RESETVAL                          (0x00000000U)

/* NAND_DATA */

#define CSL_GPMC_NAND_DATA_GPMC_NAND_DATA_0_MASK                (0xFFFFFFFFU)
#define CSL_GPMC_NAND_DATA_GPMC_NAND_DATA_0_SHIFT               (0U)
#define CSL_GPMC_NAND_DATA_GPMC_NAND_DATA_0_RESETVAL            (0x00000000U)
#define CSL_GPMC_NAND_DATA_GPMC_NAND_DATA_0_MAX                 (0xffffffffU)

#define CSL_GPMC_NAND_DATA_RESETVAL                             (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif

