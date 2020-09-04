/*
 *  Copyright (C) 2014-2021 Texas Instruments Incorporated
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
*
*  \file   cslr_mibspi.h
*
*  \brief  register-level header file for MIBSPI
*
**/

#ifndef CSLR_MIBSPI_H_
#define CSLR_MIBSPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/
#define CSL_MIBSPIRAM_MAX_ELEMENTS                                   (128U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/** \brief This structure is used to access the MSS_MIBSPIA buffer memory. */
typedef volatile struct CSL_mibspiRam_t
{
    uint32_t tx[CSL_MIBSPIRAM_MAX_ELEMENTS];
    uint32_t rx[CSL_MIBSPIRAM_MAX_ELEMENTS];
} CSL_mibspiRam;

/** \brief Overlay Structure for __ALL__ */
typedef struct {
    volatile uint32_t SPIGCR0;
    volatile uint32_t SPIGCR1;
    volatile uint32_t SPIINT0;
    volatile uint32_t SPILVL;
    volatile uint32_t SPIFLG;
    volatile uint32_t SPIPC0;
    volatile uint32_t SPIPC1;
    volatile uint32_t SPIPC2;
    volatile uint32_t SPIPC3;
    volatile uint32_t SPIPC4;
    volatile uint32_t SPIPC5;
    volatile uint32_t SPIPC6;
    volatile uint32_t SPIPC7;
    volatile uint32_t SPIPC8;
    volatile uint32_t SPIDAT0;
    volatile uint32_t SPIDAT1;
    volatile uint32_t SPIBUF;
    volatile uint32_t SPIEMU;
    volatile uint32_t SPIDELAY;
    volatile uint32_t SPIDEF;
    volatile uint32_t SPIFMT[4];
    volatile uint32_t INTVEC[2];
    volatile uint32_t SPIPC9;
    volatile uint32_t SPIPMCTRL;
    volatile uint32_t MIBSPIE;
    volatile uint32_t TGITENST;
    volatile uint32_t TGITENCR;
    volatile uint32_t TGITLVST;
    volatile uint32_t TGITLVCR;
    volatile uint32_t TGINTFLAG;
    volatile uint8_t  Resv_144[8];
    volatile uint32_t TICKCNT;
    volatile uint32_t LTGPEND;
    volatile uint32_t TGCTRL[8];
    volatile uint8_t  Resv_216[32];
    volatile uint32_t DMACTRL[5];
    volatile uint8_t  Resv_248[12];
    volatile uint32_t DMACOUNT[5];
    volatile uint8_t  Resv_280[12];
    volatile uint32_t DMACNTLEN;
    volatile uint8_t  Resv_288[4];
    volatile uint32_t PAR_ECC_CTRL;
    volatile uint32_t PAR_ECC_STAT;
    volatile uint32_t UERRADDR1;
    volatile uint32_t UERRADDR0;
    volatile uint32_t RXOVRN_BUF_ADDR;
    volatile uint32_t IOLPBKTSTCR;
    volatile uint32_t EXTENDED_PRESCALE1;
    volatile uint32_t EXTENDED_PRESCALE2;
    volatile uint32_t ECCDIAG_CTRL;
    volatile uint32_t ECCDIAG_STAT;
    volatile uint32_t SBERRADDR1;
    volatile uint32_t SBERRADDR0;
    volatile uint8_t  Resv_508[172];
    volatile uint32_t SPIREV;
} CSL_mss_spiRegs;

/**************************************************************************
* Field Definition Macros
**************************************************************************/
#define CSL_MIBSPIRAM_TX_BUFMODE_SHIFT        (29U)
#define CSL_MIBSPIRAM_TX_BUFMODE_MASK        (0xE0000000U)

#define CSL_MIBSPIRAM_TX_CSHOLD_SHIFT        (28U)
#define CSL_MIBSPIRAM_TX_CSHOLD_MASK        (0x10000000U)

#define CSL_MIBSPIRAM_TX_LOCK_SHIFT        (27U)
#define CSL_MIBSPIRAM_TX_LOCK_MASK        (0x08000000U)

#define CSL_MIBSPIRAM_TX_WDEL_SHIFT        (26U)
#define CSL_MIBSPIRAM_TX_WDEL_MASK        (0x04000000U)

#define CSL_MIBSPIRAM_TX_DFSEL_SHIFT        (24U)
#define CSL_MIBSPIRAM_TX_DFSEL_MASK        (0x03000000U)

#define CSL_MIBSPIRAM_TX_CSNR_SHIFT        (16U)
#define CSL_MIBSPIRAM_TX_CSNR_MASK        (0x00FF0000U)

#define CSL_MIBSPIRAM_TX_TXDATA_SHIFT        (0U)
#define CSL_MIBSPIRAM_TX_TXDATA_MASK        (0x0000FFFFU)

#define CSL_MIBSPIRAM_RX_RXEMPTY_SHIFT        (31U)
#define CSL_MIBSPIRAM_RX_RXEMPTY_MASK        (0x80000000U)

#define CSL_MIBSPIRAM_RX_RXOVR_SHIFT        (30U)
#define CSL_MIBSPIRAM_RX_RXOVR_MASK        (0x40000000U)

#define CSL_MIBSPIRAM_RX_TXFULL_SHIFT        (29U)
#define CSL_MIBSPIRAM_RX_TXFULL_MASK        (0x20000000U)

#define CSL_MIBSPIRAM_RX_BITERR_SHIFT        (28U)
#define CSL_MIBSPIRAM_RX_BITERR_MASK        (0x10000000U)

#define CSL_MIBSPIRAM_RX_DESYNC_SHIFT        (27U)
#define CSL_MIBSPIRAM_RX_DESYNC_MASK        (0x08000000U)

#define CSL_MIBSPIRAM_RX_PARITYERR_SHIFT        (26U)
#define CSL_MIBSPIRAM_RX_PARITYERR_MASK        (0x04000000U)

#define CSL_MIBSPIRAM_RX_TIMEOUT_SHIFT        (25U)
#define CSL_MIBSPIRAM_RX_TIMEOUT_MASK        (0x02000000U)

#define CSL_MIBSPIRAM_RX_DLENERR_SHIFT        (24U)
#define CSL_MIBSPIRAM_RX_DLENERR_MASK        (0x01000000U)

#define CSL_MIBSPIRAM_RX_LCSNR_SHIFT        (16U)
#define CSL_MIBSPIRAM_RX_LCSNR_MASK        (0x00FF0000U)

#define CSL_MIBSPIRAM_RX_RXDATA_SHIFT        (0U)
#define CSL_MIBSPIRAM_RX_RXDATA_MASK        (0x0000FFFFU)

#define CSL_MIBSPIRAM_RX_RXFLAGS_SHIFT        (16U)
#define CSL_MIBSPIRAM_RX_RXFLAGS_MASK        (0xFFFF0000U)

#define CSL_MIBSPIRAM_GET_TX_BUFMODE(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_BUFMODE))
#define CSL_MIBSPIRAM_GET_TX_CSHOLD(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_CSHOLD))
#define CSL_MIBSPIRAM_GET_TX_LOCK(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_LOCK))
#define CSL_MIBSPIRAM_GET_TX_WDEL(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_WDEL))
#define CSL_MIBSPIRAM_GET_TX_DFSEL(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_DFSEL))
#define CSL_MIBSPIRAM_GET_TX_CSNR(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_CSNR))
#define CSL_MIBSPIRAM_GET_TX_TXDATA(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_TXDATA))

#define CSL_MIBSPIRAM_SET_TX_BUFMODE(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_BUFMODE,(val)))
#define CSL_MIBSPIRAM_SET_TX_CSHOLD(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_CSHOLD,(val)))
#define CSL_MIBSPIRAM_SET_TX_LOCK(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_LOCK,(val)))
#define CSL_MIBSPIRAM_SET_TX_WDEL(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_WDEL,(val)))
#define CSL_MIBSPIRAM_SET_TX_DFSEL(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_DFSEL,(val)))
#define CSL_MIBSPIRAM_SET_TX_CSNR(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_CSNR,(val)))
#define CSL_MIBSPIRAM_SET_TX_TXDATA(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->tx[idx]),MIBSPIRAM_TX_TXDATA,(val)))

#define CSL_MIBSPIRAM_GET_RX_RXEMPTY(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_RXEMPTY))
#define CSL_MIBSPIRAM_GET_RX_RXOVR(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_RXOVR))
#define CSL_MIBSPIRAM_GET_RX_TXFULL(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_TXFULL))
#define CSL_MIBSPIRAM_GET_RX_BITERR(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_BITERR))
#define CSL_MIBSPIRAM_GET_RX_DESYNC(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_DESYNC))
#define CSL_MIBSPIRAM_GET_RX_PARITYERR(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_PARITYERR))
#define CSL_MIBSPIRAM_GET_RX_TIMEOUT(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_TIMEOUT))
#define CSL_MIBSPIRAM_GET_RX_DLENERR(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_DLENERR))
#define CSL_MIBSPIRAM_GET_RX_LCSNR(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_LCSNR))
#define CSL_MIBSPIRAM_GET_RX_RXDATA(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_RXDATA))
#define CSL_MIBSPIRAM_GET_RX_RXFLAGS(ramBase,idx)    (CSL_FEXT((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_RXFLAGS))

#define CSL_MIBSPIRAM_SET_RX_RXEMPTY(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_RXEMPTY,(val)))
#define CSL_MIBSPIRAM_SET_RX_RXOVR(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_RXOVR,(val)))
#define CSL_MIBSPIRAM_SET_RX_TXFULL(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_TXFULL,(val)))
#define CSL_MIBSPIRAM_SET_RX_BITERR(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_BITERR,(val)))
#define CSL_MIBSPIRAM_SET_RX_DESYNC(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_DESYNC,(val)))
#define CSL_MIBSPIRAM_SET_RX_PARITYERR(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_PARITYERR,(val)))
#define CSL_MIBSPIRAM_SET_RX_TIMEOUT(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_TIMEOUT,(val)))
#define CSL_MIBSPIRAM_SET_RX_DLENERR(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_DLENERR,(val)))
#define CSL_MIBSPIRAM_SET_RX_LCSNR(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_LCSNR,(val)))
#define CSL_MIBSPIRAM_SET_RX_RXDATA(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_RXDATA,(val)))
#define CSL_MIBSPIRAM_SET_RX_RXFLAGS(ramBase,idx,val)    (CSL_FINS((((CSL_mibspiRam *)(ramBase))->rx[idx]),MIBSPIRAM_RX_RXFLAGS,(val)))


/**************************************************************************
* Register Macros
**************************************************************************/
#define CSL_SPI_SPIGCR0                                                    (0x00000000U)
#define CSL_SPI_SPIGCR1                                                    (0x00000004U)
#define CSL_SPI_SPIINT0                                                    (0x00000008U)
#define CSL_SPI_SPILVL                                                     (0x0000000CU)
#define CSL_SPI_SPIFLG                                                     (0x00000010U)
#define CSL_SPI_SPIPC0                                                     (0x00000014U)
#define CSL_SPI_SPIPC1                                                     (0x00000018U)
#define CSL_SPI_SPIPC2                                                     (0x0000001CU)
#define CSL_SPI_SPIPC3                                                     (0x00000020U)
#define CSL_SPI_SPIPC4                                                     (0x00000024U)
#define CSL_SPI_SPIPC5                                                     (0x00000028U)
#define CSL_SPI_SPIPC6                                                     (0x0000002CU)
#define CSL_SPI_SPIPC7                                                     (0x00000030U)
#define CSL_SPI_SPIPC8                                                     (0x00000034U)
#define CSL_SPI_SPIDAT0                                                    (0x00000038U)
#define CSL_SPI_SPIDAT1                                                    (0x0000003CU)
#define CSL_SPI_SPIBUF                                                     (0x00000040U)
#define CSL_SPI_SPIEMU                                                     (0x00000044U)
#define CSL_SPI_SPIDELAY                                                   (0x00000048U)
#define CSL_SPI_SPIDEF                                                     (0x0000004CU)
#define CSL_SPI_SPIFMT(i)                                                  ((uint32_t)0x50U + ((i) * 4U))
#define CSL_SPI_INTVEC(i)                                                  (0x60U + ((i) * 4U))
#define CSL_SPI_SPIPC9                                                     (0x00000068U)
#define CSL_SPI_SPIPMCTRL                                                  (0x0000006CU)
#define CSL_SPI_MIBSPIE                                                    (0x00000070U)
#define CSL_SPI_TGITENST                                                   (0x00000074U)
#define CSL_SPI_TGITENCR                                                   (0x00000078U)
#define CSL_SPI_TGITLVST                                                   (0x0000007CU)
#define CSL_SPI_TGITLVCR                                                   (0x00000080U)
#define CSL_SPI_TGINTFLAG                                                  (0x00000084U)
#define CSL_SPI_TICKCNT                                                    (0x00000090U)
#define CSL_SPI_LTGPEND                                                    (0x00000094U)
#define CSL_SPI_TGCTRL(i)                                                  (0x98U + ((i) * 4U))
#define CSL_SPI_DMACTRL(i)                                                 (0xD8U + ((i) * 4U))
#define CSL_SPI_DMACOUNT(i)                                                (0xF8U + ((i) * 4U))
#define CSL_SPI_DMACNTLEN                                                  (0x00000118U)
#define CSL_SPI_PAR_ECC_CTRL                                               (0x00000120U)
#define CSL_SPI_PAR_ECC_STAT                                               (0x00000124U)
#define CSL_SPI_UERRADDR1                                                  (0x00000128U)
#define CSL_SPI_UERRADDR0                                                  (0x0000012CU)
#define CSL_SPI_RXOVRN_BUF_ADDR                                            (0x00000130U)
#define CSL_SPI_IOLPBKTSTCR                                                (0x00000134U)
#define CSL_SPI_EXTENDED_PRESCALE1                                         (0x00000138U)
#define CSL_SPI_EXTENDED_PRESCALE2                                         (0x0000013CU)
#define CSL_SPI_ECCDIAG_CTRL                                               (0x00000140U)
#define CSL_SPI_ECCDIAG_STAT                                               (0x00000144U)
#define CSL_SPI_SBERRADDR1                                                 (0x00000148U)
#define CSL_SPI_SBERRADDR0                                                 (0x0000014CU)
#define CSL_SPI_SPIREV                                                     (0x000001FCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/
/* SPIGCR0 */
#define CSL_SPI_SPIGCR0_NRESET_MASK                                        (0x00000001U)
#define CSL_SPI_SPIGCR0_NRESET_SHIFT                                       (0x00000000U)
#define CSL_SPI_SPIGCR0_NRESET_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIGCR0_NRESET_MAX                                         (0x00000001U)

/* V1_0: equivalent */
#define CSL_SPI_SPIGCR0_RESET_MASK                                         (0x00000001U)
#define CSL_SPI_SPIGCR0_RESET_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIGCR0_RESET_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIGCR0_RESET_MAX                                          (0x00000001U)
/*----RESET Tokens----*/
#define CSL_SPI_SPIGCR0_RESET_IN_RESET               ((uint32_t)0x00000000u)
#define CSL_SPI_SPIGCR0_RESET_OUT_OF_RESET             ((uint32_t)0x00000001u)

#define CSL_SPI_SPIGCR0_NU_MASK                                            (0xFFFFFFFEU)
#define CSL_SPI_SPIGCR0_NU_SHIFT                                           (0x00000001U)
#define CSL_SPI_SPIGCR0_NU_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPIGCR0_NU_MAX                                             (0x7FFFFFFFU)
#define CSL_SPI_SPIGCR0_RESETVAL                                           (0x00000000U)

/* SPIGCR1 */
#define CSL_SPI_SPIGCR1_MASTER_MASK                                        (0x00000001U)
#define CSL_SPI_SPIGCR1_MASTER_SHIFT                                       (0x00000000U)
#define CSL_SPI_SPIGCR1_MASTER_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIGCR1_MASTER_MAX                                         (0x00000001U)
/*----MASTER Tokens----*/
#define CSL_SPI_SPIGCR1_MASTER_SLAVE                 ((uint32_t)0x00000000u)
#define CSL_SPI_SPIGCR1_MASTER_MASTER                ((uint32_t)0x00000001u)

#define CSL_SPI_SPIGCR1_CLKMOD_MASK                                        (0x00000002U)
#define CSL_SPI_SPIGCR1_CLKMOD_SHIFT                                       (0x00000001U)
#define CSL_SPI_SPIGCR1_CLKMOD_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIGCR1_CLKMOD_MAX                                         (0x00000001U)
/*----CLKMOD Tokens----*/
#define CSL_SPI_SPIGCR1_CLKMOD_EXTERNAL              ((uint32_t)0x00000000u)
#define CSL_SPI_SPIGCR1_CLKMOD_INTERNAL              ((uint32_t)0x00000001u)

#define CSL_SPI_SPIGCR1_NU1_MASK                                           (0x000000FCU)
#define CSL_SPI_SPIGCR1_NU1_SHIFT                                          (0x00000002U)
#define CSL_SPI_SPIGCR1_NU1_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIGCR1_NU1_MAX                                            (0x0000003FU)

#define CSL_SPI_SPIGCR1_POWERDOWN_MASK                                     (0x00000100U)
#define CSL_SPI_SPIGCR1_POWERDOWN_SHIFT                                    (0x00000008U)
#define CSL_SPI_SPIGCR1_POWERDOWN_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIGCR1_POWERDOWN_MAX                                      (0x00000001U)
/*----POWERDOWN Tokens----*/
#define CSL_SPI_SPIGCR1_POWERDOWN_DISABLE             ((uint32_t)0x00000000u)
#define CSL_SPI_SPIGCR1_POWERDOWN_ENABLE             ((uint32_t)0x00000001u)

#define CSL_SPI_SPIGCR1_NU2_MASK                                           (0x0000FE00U)
#define CSL_SPI_SPIGCR1_NU2_SHIFT                                          (0x00000009U)
#define CSL_SPI_SPIGCR1_NU2_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIGCR1_NU2_MAX                                            (0x0000007FU)

#define CSL_SPI_SPIGCR1_LOOPBACK_MASK                                      (0x00010000U)
#define CSL_SPI_SPIGCR1_LOOPBACK_SHIFT                                     (0x00000010U)
#define CSL_SPI_SPIGCR1_LOOPBACK_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIGCR1_LOOPBACK_MAX                                       (0x00000001U)
/*----LOOPBACK Tokens----*/
#define CSL_SPI_SPIGCR1_LOOPBACK_DISABLE             ((uint32_t)0x00000000u)
#define CSL_SPI_SPIGCR1_LOOPBACK_ENABLE              ((uint32_t)0x00000001u)

#define CSL_SPI_SPIGCR1_NU3_MASK                                           (0x00FE0000U)
#define CSL_SPI_SPIGCR1_NU3_SHIFT                                          (0x00000011U)
#define CSL_SPI_SPIGCR1_NU3_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIGCR1_NU3_MAX                                            (0x0000007FU)

#define CSL_SPI_SPIGCR1_SPIEN_MASK                                         (0x01000000U)
#define CSL_SPI_SPIGCR1_SPIEN_SHIFT                                        (0x00000018U)
#define CSL_SPI_SPIGCR1_SPIEN_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIGCR1_SPIEN_MAX                                          (0x00000001U)

/* V1_0: equivalent */
#define CSL_SPI_SPIGCR1_ENABLE_MASK                                        (0x01000000U)
#define CSL_SPI_SPIGCR1_ENABLE_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIGCR1_ENABLE_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIGCR1_ENABLE_MAX                                         (0x00000001U)

/*----ENABLE Tokens----*/
#define CSL_SPI_SPIGCR1_ENABLE_DISABLE               ((uint32_t)0x00000000u)
#define CSL_SPI_SPIGCR1_ENABLE_ENABLE                ((uint32_t)0x00000001u)

#define CSL_SPI_SPIGCR1_NU4_MASK                                           (0xFE000000U)
#define CSL_SPI_SPIGCR1_NU4_SHIFT                                          (0x00000019U)
#define CSL_SPI_SPIGCR1_NU4_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIGCR1_NU4_MAX                                            (0x0000007FU)
#define CSL_SPI_SPIGCR1_RESETVAL                                           (0x00000000U)

/* SPIINT0 */
#define CSL_SPI_SPIINT0_DLENERRENA_MASK                                    (0x00000001U)
#define CSL_SPI_SPIINT0_DLENERRENA_SHIFT                                   (0x00000000U)
#define CSL_SPI_SPIINT0_DLENERRENA_RESETVAL                                (0x00000000U)
#define CSL_SPI_SPIINT0_DLENERRENA_MAX                                     (0x00000001U)

#define CSL_SPI_SPIINT0_TIMEOUTENA_MASK                                    (0x00000002U)
#define CSL_SPI_SPIINT0_TIMEOUTENA_SHIFT                                   (0x00000001U)
#define CSL_SPI_SPIINT0_TIMEOUTENA_RESETVAL                                (0x00000000U)
#define CSL_SPI_SPIINT0_TIMEOUTENA_MAX                                     (0x00000001U)

#define CSL_SPI_SPIINT0_PARERRENA_MASK                                     (0x00000004U)
#define CSL_SPI_SPIINT0_PARERRENA_SHIFT                                    (0x00000002U)
#define CSL_SPI_SPIINT0_PARERRENA_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIINT0_PARERRENA_MAX                                      (0x00000001U)

#define CSL_SPI_SPIINT0_DESYNCENA_MASK                                     (0x00000008U)
#define CSL_SPI_SPIINT0_DESYNCENA_SHIFT                                    (0x00000003U)
#define CSL_SPI_SPIINT0_DESYNCENA_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIINT0_DESYNCENA_MAX                                      (0x00000001U)

#define CSL_SPI_SPIINT0_BITERRENA_MASK                                     (0x00000010U)
#define CSL_SPI_SPIINT0_BITERRENA_SHIFT                                    (0x00000004U)
#define CSL_SPI_SPIINT0_BITERRENA_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIINT0_BITERRENA_MAX                                      (0x00000001U)

#define CSL_SPI_SPIINT0_NU1_MASK                                           (0x00000020U)
#define CSL_SPI_SPIINT0_NU1_SHIFT                                          (0x00000005U)
#define CSL_SPI_SPIINT0_NU1_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIINT0_NU1_MAX                                            (0x00000001U)

#define CSL_SPI_SPIINT0_OVRNINTENA_MASK                                    (0x00000040U)
#define CSL_SPI_SPIINT0_OVRNINTENA_SHIFT                                   (0x00000006U)
#define CSL_SPI_SPIINT0_OVRNINTENA_RESETVAL                                (0x00000000U)
#define CSL_SPI_SPIINT0_OVRNINTENA_MAX                                     (0x00000001U)

#define CSL_SPI_SPIINT0_NU2_MASK                                           (0x00000080U)
#define CSL_SPI_SPIINT0_NU2_SHIFT                                          (0x00000007U)
#define CSL_SPI_SPIINT0_NU2_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIINT0_NU2_MAX                                            (0x00000001U)

#define CSL_SPI_SPIINT0_RXINTENA_MASK                                      (0x00000100U)
#define CSL_SPI_SPIINT0_RXINTENA_SHIFT                                     (0x00000008U)
#define CSL_SPI_SPIINT0_RXINTENA_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIINT0_RXINTENA_MAX                                       (0x00000001U)

#define CSL_SPI_SPIINT0_TXINTENA_MASK                                      (0x00000200U)
#define CSL_SPI_SPIINT0_TXINTENA_SHIFT                                     (0x00000009U)
#define CSL_SPI_SPIINT0_TXINTENA_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIINT0_TXINTENA_MAX                                       (0x00000001U)

#define CSL_SPI_SPIINT0_NU3_MASK                                           (0x0000FC00U)
#define CSL_SPI_SPIINT0_NU3_SHIFT                                          (0x0000000AU)
#define CSL_SPI_SPIINT0_NU3_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIINT0_NU3_MAX                                            (0x0000003FU)

#define CSL_SPI_SPIINT0_DMAREQEN_MASK                                      (0x00010000U)
#define CSL_SPI_SPIINT0_DMAREQEN_SHIFT                                     (0x00000010U)
#define CSL_SPI_SPIINT0_DMAREQEN_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIINT0_DMAREQEN_MAX                                       (0x00000001U)

#define CSL_SPI_SPIINT0_NU4_MASK                                           (0x00FE0000U)
#define CSL_SPI_SPIINT0_NU4_SHIFT                                          (0x00000011U)
#define CSL_SPI_SPIINT0_NU4_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIINT0_NU4_MAX                                            (0x0000007FU)

#define CSL_SPI_SPIINT0_ENABLEHIGHZ_MASK                                   (0x01000000U)
#define CSL_SPI_SPIINT0_ENABLEHIGHZ_SHIFT                                  (0x00000018U)
#define CSL_SPI_SPIINT0_ENABLEHIGHZ_RESETVAL                               (0x00000000U)
#define CSL_SPI_SPIINT0_ENABLEHIGHZ_MAX                                    (0x00000001U)

#define CSL_SPI_SPIINT0_NU5_MASK                                           (0xFE000000U)
#define CSL_SPI_SPIINT0_NU5_SHIFT                                          (0x00000019U)
#define CSL_SPI_SPIINT0_NU5_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIINT0_NU5_MAX                                            (0x0000007FU)

#define CSL_SPI_SPIINT0_RESETVAL                                           (0x00000000U)

/* SPILVL */
#define CSL_SPI_SPILVL_DLENERRLVL_MASK                                     (0x00000001U)
#define CSL_SPI_SPILVL_DLENERRLVL_SHIFT                                    (0x00000000U)
#define CSL_SPI_SPILVL_DLENERRLVL_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPILVL_DLENERRLVL_MAX                                      (0x00000001U)

#define CSL_SPI_SPILVL_TIMEOUTLVL_MASK                                     (0x00000002U)
#define CSL_SPI_SPILVL_TIMEOUTLVL_SHIFT                                    (0x00000001U)
#define CSL_SPI_SPILVL_TIMEOUTLVL_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPILVL_TIMEOUTLVL_MAX                                      (0x00000001U)

#define CSL_SPI_SPILVL_PARERRLVL_MASK                                      (0x00000004U)
#define CSL_SPI_SPILVL_PARERRLVL_SHIFT                                     (0x00000002U)
#define CSL_SPI_SPILVL_PARERRLVL_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPILVL_PARERRLVL_MAX                                       (0x00000001U)

#define CSL_SPI_SPILVL_DESYNCLVL_MASK                                      (0x00000008U)
#define CSL_SPI_SPILVL_DESYNCLVL_SHIFT                                     (0x00000003U)
#define CSL_SPI_SPILVL_DESYNCLVL_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPILVL_DESYNCLVL_MAX                                       (0x00000001U)

#define CSL_SPI_SPILVL_BITERRLVL_MASK                                      (0x00000010U)
#define CSL_SPI_SPILVL_BITERRLVL_SHIFT                                     (0x00000004U)
#define CSL_SPI_SPILVL_BITERRLVL_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPILVL_BITERRLVL_MAX                                       (0x00000001U)

#define CSL_SPI_SPILVL_NU1_MASK                                            (0x00000020U)
#define CSL_SPI_SPILVL_NU1_SHIFT                                           (0x00000005U)
#define CSL_SPI_SPILVL_NU1_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPILVL_NU1_MAX                                             (0x00000001U)

#define CSL_SPI_SPILVL_OVRNINTLVL_MASK                                     (0x00000040U)
#define CSL_SPI_SPILVL_OVRNINTLVL_SHIFT                                    (0x00000006U)
#define CSL_SPI_SPILVL_OVRNINTLVL_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPILVL_OVRNINTLVL_MAX                                      (0x00000001U)

#define CSL_SPI_SPILVL_NU2_MASK                                            (0x00000080U)
#define CSL_SPI_SPILVL_NU2_SHIFT                                           (0x00000007U)
#define CSL_SPI_SPILVL_NU2_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPILVL_NU2_MAX                                             (0x00000001U)

#define CSL_SPI_SPILVL_RXINTLVL_MASK                                       (0x00000100U)
#define CSL_SPI_SPILVL_RXINTLVL_SHIFT                                      (0x00000008U)
#define CSL_SPI_SPILVL_RXINTLVL_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPILVL_RXINTLVL_MAX                                        (0x00000001U)

#define CSL_SPI_SPILVL_TXINTLVL_MASK                                       (0x00000200U)
#define CSL_SPI_SPILVL_TXINTLVL_SHIFT                                      (0x00000009U)
#define CSL_SPI_SPILVL_TXINTLVL_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPILVL_TXINTLVL_MAX                                        (0x00000001U)

#define CSL_SPI_SPILVL_NU3_MASK                                            (0xFFFFFC00U)
#define CSL_SPI_SPILVL_NU3_SHIFT                                           (0x0000000AU)
#define CSL_SPI_SPILVL_NU3_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPILVL_NU3_MAX                                             (0x003FFFFFU)

#define CSL_SPI_SPILVL_RESETVAL                                            (0x00000000U)

/* SPIFLG */
#define CSL_SPI_SPIFLG_DLENERRFLG_MASK                                     (0x00000001U)
#define CSL_SPI_SPIFLG_DLENERRFLG_SHIFT                                    (0x00000000U)
#define CSL_SPI_SPIFLG_DLENERRFLG_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIFLG_DLENERRFLG_MAX                                      (0x00000001U)

#define CSL_SPI_SPIFLG_TIMEOUTFLG_MASK                                     (0x00000002U)
#define CSL_SPI_SPIFLG_TIMEOUTFLG_SHIFT                                    (0x00000001U)
#define CSL_SPI_SPIFLG_TIMEOUTFLG_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIFLG_TIMEOUTFLG_MAX                                      (0x00000001U)

#define CSL_SPI_SPIFLG_PARERRFLG_MASK                                      (0x00000004U)
#define CSL_SPI_SPIFLG_PARERRFLG_SHIFT                                     (0x00000002U)
#define CSL_SPI_SPIFLG_PARERRFLG_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIFLG_PARERRFLG_MAX                                       (0x00000001U)

#define CSL_SPI_SPIFLG_DESYNCFLG_MASK                                      (0x00000008U)
#define CSL_SPI_SPIFLG_DESYNCFLG_SHIFT                                     (0x00000003U)
#define CSL_SPI_SPIFLG_DESYNCFLG_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIFLG_DESYNCFLG_MAX                                       (0x00000001U)

#define CSL_SPI_SPIFLG_BITERRFLG_MASK                                      (0x00000010U)
#define CSL_SPI_SPIFLG_BITERRFLG_SHIFT                                     (0x00000004U)
#define CSL_SPI_SPIFLG_BITERRFLG_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIFLG_BITERRFLG_MAX                                       (0x00000001U)

#define CSL_SPI_SPIFLG_NU1_MASK                                            (0x00000020U)
#define CSL_SPI_SPIFLG_NU1_SHIFT                                           (0x00000005U)
#define CSL_SPI_SPIFLG_NU1_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPIFLG_NU1_MAX                                             (0x00000001U)

#define CSL_SPI_SPIFLG_OVRNINTFLG_MASK                                     (0x00000040U)
#define CSL_SPI_SPIFLG_OVRNINTFLG_SHIFT                                    (0x00000006U)
#define CSL_SPI_SPIFLG_OVRNINTFLG_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIFLG_OVRNINTFLG_MAX                                      (0x00000001U)

#define CSL_SPI_SPIFLG_NU2_MASK                                            (0x00000080U)
#define CSL_SPI_SPIFLG_NU2_SHIFT                                           (0x00000007U)
#define CSL_SPI_SPIFLG_NU2_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPIFLG_NU2_MAX                                             (0x00000001U)

#define CSL_SPI_SPIFLG_RXINTFLG_MASK                                       (0x00000100U)
#define CSL_SPI_SPIFLG_RXINTFLG_SHIFT                                      (0x00000008U)
#define CSL_SPI_SPIFLG_RXINTFLG_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIFLG_RXINTFLG_MAX                                        (0x00000001U)

#define CSL_SPI_SPIFLG_TXINTFLG_MASK                                       (0x00000200U)
#define CSL_SPI_SPIFLG_TXINTFLG_SHIFT                                      (0x00000009U)
#define CSL_SPI_SPIFLG_TXINTFLG_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIFLG_TXINTFLG_MAX                                        (0x00000001U)

#define CSL_SPI_SPIFLG_NU3_MASK                                            (0x00FFFC00U)
#define CSL_SPI_SPIFLG_NU3_SHIFT                                           (0x0000000AU)
#define CSL_SPI_SPIFLG_NU3_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPIFLG_NU3_MAX                                             (0x00003FFFU)

#define CSL_SPI_SPIFLG_BUFINITACTIVE_MASK                                  (0x01000000U)
#define CSL_SPI_SPIFLG_BUFINITACTIVE_SHIFT                                 (0x00000018U)
#define CSL_SPI_SPIFLG_BUFINITACTIVE_RESETVAL                              (0x00000000U)
#define CSL_SPI_SPIFLG_BUFINITACTIVE_MAX                                   (0x00000001U)

#define CSL_SPI_SPIFLG_NU4_MASK                                            (0xFE000000U)
#define CSL_SPI_SPIFLG_NU4_SHIFT                                           (0x00000019U)
#define CSL_SPI_SPIFLG_NU4_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPIFLG_NU4_MAX                                             (0x0000007FU)

#define CSL_SPI_SPIFLG_RESETVAL                                            (0x00000000U)

/* SPIPC0 */
#define CSL_SPI_SPIPC0_SCSFUN_MASK                                         (0x000000FFU)
#define CSL_SPI_SPIPC0_SCSFUN_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIPC0_SCSFUN_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC0_SCSFUN_MAX                                          (0x000000FFU)

#define CSL_SPI_SPIPC0_ENAFUN_MASK                                         (0x00000100U)
#define CSL_SPI_SPIPC0_ENAFUN_SHIFT                                        (0x00000008U)
#define CSL_SPI_SPIPC0_ENAFUN_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC0_ENAFUN_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC0_CLKFUN_MASK                                         (0x00000200U)
#define CSL_SPI_SPIPC0_CLKFUN_SHIFT                                        (0x00000009U)
#define CSL_SPI_SPIPC0_CLKFUN_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC0_CLKFUN_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC0_SIMOFUN0_MASK                                       (0x00000400U)
#define CSL_SPI_SPIPC0_SIMOFUN0_SHIFT                                      (0x0000000AU)
#define CSL_SPI_SPIPC0_SIMOFUN0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC0_SIMOFUN0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC0_SOMIFUN0_MASK                                       (0x00000800U)
#define CSL_SPI_SPIPC0_SOMIFUN0_SHIFT                                      (0x0000000BU)
#define CSL_SPI_SPIPC0_SOMIFUN0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC0_SOMIFUN0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC0_NU_MASK                                             (0x0000F000U)
#define CSL_SPI_SPIPC0_NU_SHIFT                                            (0x0000000CU)
#define CSL_SPI_SPIPC0_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIPC0_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_SPIPC0_SIMOFUN_MASK                                        (0x00FF0000U)
#define CSL_SPI_SPIPC0_SIMOFUN_SHIFT                                       (0x00000010U)
#define CSL_SPI_SPIPC0_SIMOFUN_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC0_SIMOFUN_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC0_SOMIFUN_MASK                                        (0xFF000000U)
#define CSL_SPI_SPIPC0_SOMIFUN_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIPC0_SOMIFUN_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC0_SOMIFUN_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC0_RESETVAL                                            (0x00000000U)

/* SPIPC1 */
#define CSL_SPI_SPIPC1_SCSDIR_MASK                                         (0x000000FFU)
#define CSL_SPI_SPIPC1_SCSDIR_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIPC1_SCSDIR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC1_SCSDIR_MAX                                          (0x000000FFU)

#define CSL_SPI_SPIPC1_ENADIR_MASK                                         (0x00000100U)
#define CSL_SPI_SPIPC1_ENADIR_SHIFT                                        (0x00000008U)
#define CSL_SPI_SPIPC1_ENADIR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC1_ENADIR_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC1_CLKDIR_MASK                                         (0x00000200U)
#define CSL_SPI_SPIPC1_CLKDIR_SHIFT                                        (0x00000009U)
#define CSL_SPI_SPIPC1_CLKDIR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC1_CLKDIR_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC1_SIMODIR0_MASK                                       (0x00000400U)
#define CSL_SPI_SPIPC1_SIMODIR0_SHIFT                                      (0x0000000AU)
#define CSL_SPI_SPIPC1_SIMODIR0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC1_SIMODIR0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC1_SOMIDIR0_MASK                                       (0x00000800U)
#define CSL_SPI_SPIPC1_SOMIDIR0_SHIFT                                      (0x0000000BU)
#define CSL_SPI_SPIPC1_SOMIDIR0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC1_SOMIDIR0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC1_NU_MASK                                             (0x0000F000U)
#define CSL_SPI_SPIPC1_NU_SHIFT                                            (0x0000000CU)
#define CSL_SPI_SPIPC1_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIPC1_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_SPIPC1_SIMODIR_MASK                                        (0x00FF0000U)
#define CSL_SPI_SPIPC1_SIMODIR_SHIFT                                       (0x00000010U)
#define CSL_SPI_SPIPC1_SIMODIR_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC1_SIMODIR_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC1_SOMIDIR_MASK                                        (0xFF000000U)
#define CSL_SPI_SPIPC1_SOMIDIR_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIPC1_SOMIDIR_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC1_SOMIDIR_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC1_RESETVAL                                            (0x00000000U)

/* SPIPC2 */
#define CSL_SPI_SPIPC2_SCSDIN_MASK                                         (0x000000FFU)
#define CSL_SPI_SPIPC2_SCSDIN_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIPC2_SCSDIN_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC2_SCSDIN_MAX                                          (0x000000FFU)

#define CSL_SPI_SPIPC2_ENADIN_MASK                                         (0x00000100U)
#define CSL_SPI_SPIPC2_ENADIN_SHIFT                                        (0x00000008U)
#define CSL_SPI_SPIPC2_ENADIN_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC2_ENADIN_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC2_CLKDIN_MASK                                         (0x00000200U)
#define CSL_SPI_SPIPC2_CLKDIN_SHIFT                                        (0x00000009U)
#define CSL_SPI_SPIPC2_CLKDIN_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC2_CLKDIN_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC2_SIMODIN0_MASK                                       (0x00000400U)
#define CSL_SPI_SPIPC2_SIMODIN0_SHIFT                                      (0x0000000AU)
#define CSL_SPI_SPIPC2_SIMODIN0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC2_SIMODIN0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC2_SOMIDIN0_MASK                                       (0x00000800U)
#define CSL_SPI_SPIPC2_SOMIDIN0_SHIFT                                      (0x0000000BU)
#define CSL_SPI_SPIPC2_SOMIDIN0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC2_SOMIDIN0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC2_NU_MASK                                             (0x0000F000U)
#define CSL_SPI_SPIPC2_NU_SHIFT                                            (0x0000000CU)
#define CSL_SPI_SPIPC2_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIPC2_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_SPIPC2_SIMODIN_MASK                                        (0x00FF0000U)
#define CSL_SPI_SPIPC2_SIMODIN_SHIFT                                       (0x00000010U)
#define CSL_SPI_SPIPC2_SIMODIN_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC2_SIMODIN_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC2_SOMIDIN_MASK                                        (0xFF000000U)
#define CSL_SPI_SPIPC2_SOMIDIN_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIPC2_SOMIDIN_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC2_SOMIDIN_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC2_RESETVAL                                            (0x00000000U)

/* SPIPC3 */
#define CSL_SPI_SPIPC3_SCSDOUT_MASK                                        (0x000000FFU)
#define CSL_SPI_SPIPC3_SCSDOUT_SHIFT                                       (0x00000000U)
#define CSL_SPI_SPIPC3_SCSDOUT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC3_SCSDOUT_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC3_ENADOUT_MASK                                        (0x00000100U)
#define CSL_SPI_SPIPC3_ENADOUT_SHIFT                                       (0x00000008U)
#define CSL_SPI_SPIPC3_ENADOUT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC3_ENADOUT_MAX                                         (0x00000001U)

#define CSL_SPI_SPIPC3_CLKDOUT_MASK                                        (0x00000200U)
#define CSL_SPI_SPIPC3_CLKDOUT_SHIFT                                       (0x00000009U)
#define CSL_SPI_SPIPC3_CLKDOUT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC3_CLKDOUT_MAX                                         (0x00000001U)

#define CSL_SPI_SPIPC3_SIMODOUT0_MASK                                      (0x00000400U)
#define CSL_SPI_SPIPC3_SIMODOUT0_SHIFT                                     (0x0000000AU)
#define CSL_SPI_SPIPC3_SIMODOUT0_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPC3_SIMODOUT0_MAX                                       (0x00000001U)

#define CSL_SPI_SPIPC3_SOMIDOUT0_MASK                                      (0x00000800U)
#define CSL_SPI_SPIPC3_SOMIDOUT0_SHIFT                                     (0x0000000BU)
#define CSL_SPI_SPIPC3_SOMIDOUT0_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPC3_SOMIDOUT0_MAX                                       (0x00000001U)

#define CSL_SPI_SPIPC3_NU_MASK                                             (0x0000F000U)
#define CSL_SPI_SPIPC3_NU_SHIFT                                            (0x0000000CU)
#define CSL_SPI_SPIPC3_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIPC3_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_SPIPC3_SIMODOUT_MASK                                       (0x00FF0000U)
#define CSL_SPI_SPIPC3_SIMODOUT_SHIFT                                      (0x00000010U)
#define CSL_SPI_SPIPC3_SIMODOUT_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC3_SIMODOUT_MAX                                        (0x000000FFU)

#define CSL_SPI_SPIPC3_SOMIDOUT_MASK                                       (0xFF000000U)
#define CSL_SPI_SPIPC3_SOMIDOUT_SHIFT                                      (0x00000018U)
#define CSL_SPI_SPIPC3_SOMIDOUT_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC3_SOMIDOUT_MAX                                        (0x000000FFU)

#define CSL_SPI_SPIPC3_RESETVAL                                            (0x00000000U)

/* SPIPC4 */
#define CSL_SPI_SPIPC4_SCSSET_MASK                                         (0x000000FFU)
#define CSL_SPI_SPIPC4_SCSSET_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIPC4_SCSSET_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC4_SCSSET_MAX                                          (0x000000FFU)

#define CSL_SPI_SPIPC4_ENASET_MASK                                         (0x00000100U)
#define CSL_SPI_SPIPC4_ENASET_SHIFT                                        (0x00000008U)
#define CSL_SPI_SPIPC4_ENASET_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC4_ENASET_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC4_CLKSET_MASK                                         (0x00000200U)
#define CSL_SPI_SPIPC4_CLKSET_SHIFT                                        (0x00000009U)
#define CSL_SPI_SPIPC4_CLKSET_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC4_CLKSET_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC4_SIMOSET0_MASK                                       (0x00000400U)
#define CSL_SPI_SPIPC4_SIMOSET0_SHIFT                                      (0x0000000AU)
#define CSL_SPI_SPIPC4_SIMOSET0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC4_SIMOSET0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC4_SOMISET0_MASK                                       (0x00000800U)
#define CSL_SPI_SPIPC4_SOMISET0_SHIFT                                      (0x0000000BU)
#define CSL_SPI_SPIPC4_SOMISET0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC4_SOMISET0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC4_NU_MASK                                             (0x0000F000U)
#define CSL_SPI_SPIPC4_NU_SHIFT                                            (0x0000000CU)
#define CSL_SPI_SPIPC4_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIPC4_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_SPIPC4_SIMOSET_MASK                                        (0x00FF0000U)
#define CSL_SPI_SPIPC4_SIMOSET_SHIFT                                       (0x00000010U)
#define CSL_SPI_SPIPC4_SIMOSET_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC4_SIMOSET_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC4_SOMISET_MASK                                        (0xFF000000U)
#define CSL_SPI_SPIPC4_SOMISET_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIPC4_SOMISET_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC4_SOMISET_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC4_RESETVAL                                            (0x00000000U)

/* SPIPC5 */
#define CSL_SPI_SPIPC5_SCSCLR_MASK                                         (0x000000FFU)
#define CSL_SPI_SPIPC5_SCSCLR_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIPC5_SCSCLR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC5_SCSCLR_MAX                                          (0x000000FFU)

#define CSL_SPI_SPIPC5_ENACLR_MASK                                         (0x00000100U)
#define CSL_SPI_SPIPC5_ENACLR_SHIFT                                        (0x00000008U)
#define CSL_SPI_SPIPC5_ENACLR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC5_ENACLR_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC5_CLKCLR_MASK                                         (0x00000200U)
#define CSL_SPI_SPIPC5_CLKCLR_SHIFT                                        (0x00000009U)
#define CSL_SPI_SPIPC5_CLKCLR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC5_CLKCLR_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC5_SIMOCLR0_MASK                                       (0x00000400U)
#define CSL_SPI_SPIPC5_SIMOCLR0_SHIFT                                      (0x0000000AU)
#define CSL_SPI_SPIPC5_SIMOCLR0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC5_SIMOCLR0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC5_SOMICLR0_MASK                                       (0x00000800U)
#define CSL_SPI_SPIPC5_SOMICLR0_SHIFT                                      (0x0000000BU)
#define CSL_SPI_SPIPC5_SOMICLR0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC5_SOMICLR0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC5_NU_MASK                                             (0x0000F000U)
#define CSL_SPI_SPIPC5_NU_SHIFT                                            (0x0000000CU)
#define CSL_SPI_SPIPC5_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIPC5_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_SPIPC5_SIMOCLR_MASK                                        (0x00FF0000U)
#define CSL_SPI_SPIPC5_SIMOCLR_SHIFT                                       (0x00000010U)
#define CSL_SPI_SPIPC5_SIMOCLR_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC5_SIMOCLR_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC5_SOMICLR_MASK                                        (0xFF000000U)
#define CSL_SPI_SPIPC5_SOMICLR_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIPC5_SOMICLR_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC5_SOMICLR_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC5_RESETVAL                                            (0x00000000U)

/* SPIPC6 */
#define CSL_SPI_SPIPC6_SCSPDR_MASK                                         (0x000000FFU)
#define CSL_SPI_SPIPC6_SCSPDR_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIPC6_SCSPDR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC6_SCSPDR_MAX                                          (0x000000FFU)

#define CSL_SPI_SPIPC6_ENAPDR_MASK                                         (0x00000100U)
#define CSL_SPI_SPIPC6_ENAPDR_SHIFT                                        (0x00000008U)
#define CSL_SPI_SPIPC6_ENAPDR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC6_ENAPDR_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC6_CLKPDR_MASK                                         (0x00000200U)
#define CSL_SPI_SPIPC6_CLKPDR_SHIFT                                        (0x00000009U)
#define CSL_SPI_SPIPC6_CLKPDR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC6_CLKPDR_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC6_SIMOPDR0_MASK                                       (0x00000400U)
#define CSL_SPI_SPIPC6_SIMOPDR0_SHIFT                                      (0x0000000AU)
#define CSL_SPI_SPIPC6_SIMOPDR0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC6_SIMOPDR0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC6_SOMIPDR0_MASK                                       (0x00000800U)
#define CSL_SPI_SPIPC6_SOMIPDR0_SHIFT                                      (0x0000000BU)
#define CSL_SPI_SPIPC6_SOMIPDR0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC6_SOMIPDR0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC6_NU_MASK                                             (0x0000F000U)
#define CSL_SPI_SPIPC6_NU_SHIFT                                            (0x0000000CU)
#define CSL_SPI_SPIPC6_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIPC6_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_SPIPC6_SIMOPDR_MASK                                        (0x00FF0000U)
#define CSL_SPI_SPIPC6_SIMOPDR_SHIFT                                       (0x00000010U)
#define CSL_SPI_SPIPC6_SIMOPDR_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC6_SIMOPDR_MAX                                         (0x000000FFU)

#define CSL_SPI_SPIPC6_SOMIPDR_MASK                                        (0xFF000000U)
#define CSL_SPI_SPIPC6_SOMIPDR_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIPC6_SOMIPDR_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIPC6_SOMIPDR_MAX                                         (0x000000FFU)
#define CSL_SPI_SPIPC6_RESETVAL                                            (0x00000000U)

/* SPIPC7 */
#define CSL_SPI_SPIPC7_SOMIDIS_MASK                                        (0xFF000000U)
#define CSL_SPI_SPIPC7_SOMIDIS_SHIFT                                       (24U)

#define CSL_SPI_SPIPC7_SIMODIS_MASK                                        (0x00FF0000U)
#define CSL_SPI_SPIPC7_SIMODIS_SHIFT                                       (16U)

#define CSL_SPI_SPIPC7_SOMIPDIS0_MASK                                      (0x00000800U)
#define CSL_SPI_SPIPC7_SOMIPDIS0_SHIFT                                     (11U)

#define CSL_SPI_SPIPC7_SIMOPDIS0_MASK                                      (0x00000400U)
#define CSL_SPI_SPIPC7_SIMOPDIS0_SHIFT                                     (10U)

#define CSL_SPI_SPIPC7_CLKPDIS_MASK                                        (0x00000200U)
#define CSL_SPI_SPIPC7_CLKPDIS_SHIFT                                       (9U)

#define CSL_SPI_SPIPC7_ENAPDIS_MASK                                        (0x00000100U)
#define CSL_SPI_SPIPC7_ENAPDIS_SHIFT                                       (8U)

#define CSL_SPI_SPIPC7_SCSPDIS_MASK                                        (0x000000FFU)
#define CSL_SPI_SPIPC7_SCSPDIS_SHIFT                                       (0U)

/* SPIPC8 */
#define CSL_SPI_SPIPC8_SOMIPSEL_MASK                                       (0xFF000000U)
#define CSL_SPI_SPIPC8_SOMIPSEL_SHIFT                                      (24U)

#define CSL_SPI_SPIPC8_SIMOPSEL_MASK                                       (0x00FF0000U)
#define CSL_SPI_SPIPC8_SIMOPSEL_SHIFT                                      (16U)

#define CSL_SPI_SPIPC8_SOMIPSEL0_MASK                                      (0x00000800U)
#define CSL_SPI_SPIPC8_SOMIPSEL0_SHIFT                                     (11U)

#define CSL_SPI_SPIPC8_SIMOPSEL0_MASK                                      (0x00000400U)
#define CSL_SPI_SPIPC8_SIMOPSEL0_SHIFT                                     (10U)

#define CSL_SPI_SPIPC8_CLKPSEL_MASK                                        (0x00000200U)
#define CSL_SPI_SPIPC8_CLKPSEL_SHIFT                                       (9U)

#define CSL_SPI_SPIPC8_ENAPSEL_MASK                                        (0x00000100U)
#define CSL_SPI_SPIPC8_ENAPSEL_SHIFT                                       (8U)

#define CSL_SPI_SPIPC8_SCSPSEL_MASK                                        (0x000000FFU)
#define CSL_SPI_SPIPC8_SCSPSEL_SHIFT                                       (0U)


/* SPIDAT0 */
#define CSL_SPI_SPIDAT0_TXDATA_MASK                                        (0x0000FFFFU)
#define CSL_SPI_SPIDAT0_TXDATA_SHIFT                                       (0x00000000U)
#define CSL_SPI_SPIDAT0_TXDATA_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIDAT0_TXDATA_MAX                                         (0x0000FFFFU)

#define CSL_SPI_SPIDAT0_NU_MASK                                            (0xFFFF0000U)
#define CSL_SPI_SPIDAT0_NU_SHIFT                                           (0x00000010U)
#define CSL_SPI_SPIDAT0_NU_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPIDAT0_NU_MAX                                             (0x0000FFFFU)

#define CSL_SPI_SPIDAT0_RESETVAL                                           (0x00000000U)

/* SPIDAT1 */
#define CSL_SPI_SPIDAT1_TXDATA_MASK                                        (0x0000FFFFU)
#define CSL_SPI_SPIDAT1_TXDATA_SHIFT                                       (0x00000000U)
#define CSL_SPI_SPIDAT1_TXDATA_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIDAT1_TXDATA_MAX                                         (0x0000FFFFU)

#define CSL_SPI_SPIDAT1_CSNR_MASK                                          (0x00FF0000U)
#define CSL_SPI_SPIDAT1_CSNR_SHIFT                                         (0x00000010U)
#define CSL_SPI_SPIDAT1_CSNR_RESETVAL                                      (0x00000000U)
#define CSL_SPI_SPIDAT1_CSNR_MAX                                           (0x000000FFU)

#define CSL_SPI_SPIDAT1_DFSEL_MASK                                         (0x03000000U)
#define CSL_SPI_SPIDAT1_DFSEL_SHIFT                                        (0x00000018U)
#define CSL_SPI_SPIDAT1_DFSEL_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIDAT1_DFSEL_MAX                                          (0x00000003U)

#define CSL_SPI_SPIDAT1_WDEL_MASK                                          (0x04000000U)
#define CSL_SPI_SPIDAT1_WDEL_SHIFT                                         (0x0000001AU)
#define CSL_SPI_SPIDAT1_WDEL_RESETVAL                                      (0x00000000U)
#define CSL_SPI_SPIDAT1_WDEL_MAX                                           (0x00000001U)

#define CSL_SPI_SPIDAT1_NU1_MASK                                           (0x08000000U)
#define CSL_SPI_SPIDAT1_NU1_SHIFT                                          (0x0000001BU)
#define CSL_SPI_SPIDAT1_NU1_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIDAT1_NU1_MAX                                            (0x00000001U)

#define CSL_SPI_SPIDAT1_CSHOLD_MASK                                        (0x10000000U)
#define CSL_SPI_SPIDAT1_CSHOLD_SHIFT                                       (0x0000001CU)
#define CSL_SPI_SPIDAT1_CSHOLD_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIDAT1_CSHOLD_MAX                                         (0x00000001U)

#define CSL_SPI_SPIDAT1_NU2_MASK                                           (0xE0000000U)
#define CSL_SPI_SPIDAT1_NU2_SHIFT                                          (0x0000001DU)
#define CSL_SPI_SPIDAT1_NU2_RESETVAL                                       (0x00000000U)
#define CSL_SPI_SPIDAT1_NU2_MAX                                            (0x00000007U)

#define CSL_SPI_SPIDAT1_RESETVAL                                           (0x00000000U)

/* SPIBUF */
#define CSL_SPI_SPIBUF_RXDATA_MASK                                         (0x0000FFFFU)
#define CSL_SPI_SPIBUF_RXDATA_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIBUF_RXDATA_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIBUF_RXDATA_MAX                                          (0x0000FFFFU)

#define CSL_SPI_SPIBUF_LCSNR_MASK                                          (0x00FF0000U)
#define CSL_SPI_SPIBUF_LCSNR_SHIFT                                         (0x00000010U)
#define CSL_SPI_SPIBUF_LCSNR_RESETVAL                                      (0x00000000U)
#define CSL_SPI_SPIBUF_LCSNR_MAX                                           (0x000000FFU)

#define CSL_SPI_SPIBUF_DLENERR_MASK                                        (0x01000000U)
#define CSL_SPI_SPIBUF_DLENERR_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIBUF_DLENERR_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIBUF_DLENERR_MAX                                         (0x00000001U)

#define CSL_SPI_SPIBUF_TIMEOUT_MASK                                        (0x02000000U)
#define CSL_SPI_SPIBUF_TIMEOUT_SHIFT                                       (0x00000019U)
#define CSL_SPI_SPIBUF_TIMEOUT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIBUF_TIMEOUT_MAX                                         (0x00000001U)

#define CSL_SPI_SPIBUF_PARITYERR_MASK                                      (0x04000000U)
#define CSL_SPI_SPIBUF_PARITYERR_SHIFT                                     (0x0000001AU)
#define CSL_SPI_SPIBUF_PARITYERR_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIBUF_PARITYERR_MAX                                       (0x00000001U)

#define CSL_SPI_SPIBUF_DESYNC_MASK                                         (0x08000000U)
#define CSL_SPI_SPIBUF_DESYNC_SHIFT                                        (0x0000001BU)
#define CSL_SPI_SPIBUF_DESYNC_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIBUF_DESYNC_MAX                                          (0x00000001U)

#define CSL_SPI_SPIBUF_BITERR_MASK                                         (0x10000000U)
#define CSL_SPI_SPIBUF_BITERR_SHIFT                                        (0x0000001CU)
#define CSL_SPI_SPIBUF_BITERR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIBUF_BITERR_MAX                                          (0x00000001U)

#define CSL_SPI_SPIBUF_TXFULL_MASK                                         (0x20000000U)
#define CSL_SPI_SPIBUF_TXFULL_SHIFT                                        (0x0000001DU)
#define CSL_SPI_SPIBUF_TXFULL_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIBUF_TXFULL_MAX                                          (0x00000001U)

#define CSL_SPI_SPIBUF_RXOVR_MASK                                          (0x40000000U)
#define CSL_SPI_SPIBUF_RXOVR_SHIFT                                         (0x0000001EU)
#define CSL_SPI_SPIBUF_RXOVR_RESETVAL                                      (0x00000000U)
#define CSL_SPI_SPIBUF_RXOVR_MAX                                           (0x00000001U)

#define CSL_SPI_SPIBUF_RXEMPTY_MASK                                        (0x80000000U)
#define CSL_SPI_SPIBUF_RXEMPTY_SHIFT                                       (0x0000001FU)
#define CSL_SPI_SPIBUF_RXEMPTY_RESETVAL                                    (0x00000001U)
#define CSL_SPI_SPIBUF_RXEMPTY_MAX                                         (0x00000001U)

#define CSL_SPI_SPIBUF_RESETVAL                                            (0x80000000U)

/* SPIEMU */
#define CSL_SPI_SPIEMU_RXDATA_MASK                                         (0x0000FFFFU)
#define CSL_SPI_SPIEMU_RXDATA_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIEMU_RXDATA_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIEMU_RXDATA_MAX                                          (0x0000FFFFU)

#define CSL_SPI_SPIEMU_LCSNR_MASK                                          (0x00FF0000U)
#define CSL_SPI_SPIEMU_LCSNR_SHIFT                                         (0x00000010U)
#define CSL_SPI_SPIEMU_LCSNR_RESETVAL                                      (0x00000000U)
#define CSL_SPI_SPIEMU_LCSNR_MAX                                           (0x000000FFU)

#define CSL_SPI_SPIEMU_DLENERR_MASK                                        (0x01000000U)
#define CSL_SPI_SPIEMU_DLENERR_SHIFT                                       (0x00000018U)
#define CSL_SPI_SPIEMU_DLENERR_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIEMU_DLENERR_MAX                                         (0x00000001U)

#define CSL_SPI_SPIEMU_TIMEOUT_MASK                                        (0x02000000U)
#define CSL_SPI_SPIEMU_TIMEOUT_SHIFT                                       (0x00000019U)
#define CSL_SPI_SPIEMU_TIMEOUT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIEMU_TIMEOUT_MAX                                         (0x00000001U)

#define CSL_SPI_SPIEMU_PARITYERR_MASK                                      (0x04000000U)
#define CSL_SPI_SPIEMU_PARITYERR_SHIFT                                     (0x0000001AU)
#define CSL_SPI_SPIEMU_PARITYERR_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIEMU_PARITYERR_MAX                                       (0x00000001U)

#define CSL_SPI_SPIEMU_DESYNC_MASK                                         (0x08000000U)
#define CSL_SPI_SPIEMU_DESYNC_SHIFT                                        (0x0000001BU)
#define CSL_SPI_SPIEMU_DESYNC_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIEMU_DESYNC_MAX                                          (0x00000001U)

#define CSL_SPI_SPIEMU_BITERR_MASK                                         (0x10000000U)
#define CSL_SPI_SPIEMU_BITERR_SHIFT                                        (0x0000001CU)
#define CSL_SPI_SPIEMU_BITERR_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIEMU_BITERR_MAX                                          (0x00000001U)

#define CSL_SPI_SPIEMU_TXFULL_MASK                                         (0x20000000U)
#define CSL_SPI_SPIEMU_TXFULL_SHIFT                                        (0x0000001DU)
#define CSL_SPI_SPIEMU_TXFULL_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIEMU_TXFULL_MAX                                          (0x00000001U)

#define CSL_SPI_SPIEMU_RXOVR_MASK                                          (0x40000000U)
#define CSL_SPI_SPIEMU_RXOVR_SHIFT                                         (0x0000001EU)
#define CSL_SPI_SPIEMU_RXOVR_RESETVAL                                      (0x00000000U)
#define CSL_SPI_SPIEMU_RXOVR_MAX                                           (0x00000001U)

#define CSL_SPI_SPIEMU_RXEMPTY_MASK                                        (0x80000000U)
#define CSL_SPI_SPIEMU_RXEMPTY_SHIFT                                       (0x0000001FU)
#define CSL_SPI_SPIEMU_RXEMPTY_RESETVAL                                    (0x00000001U)
#define CSL_SPI_SPIEMU_RXEMPTY_MAX                                         (0x00000001U)

#define CSL_SPI_SPIEMU_RESETVAL                                            (0x80000000U)

/* SPIDELAY */
#define CSL_SPI_SPIDELAY_C2EDELAY_MASK                                     (0x000000FFU)
#define CSL_SPI_SPIDELAY_C2EDELAY_SHIFT                                    (0x00000000U)
#define CSL_SPI_SPIDELAY_C2EDELAY_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIDELAY_C2EDELAY_MAX                                      (0x000000FFU)

#define CSL_SPI_SPIDELAY_T2EDELAY_MASK                                     (0x0000FF00U)
#define CSL_SPI_SPIDELAY_T2EDELAY_SHIFT                                    (0x00000008U)
#define CSL_SPI_SPIDELAY_T2EDELAY_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIDELAY_T2EDELAY_MAX                                      (0x000000FFU)

#define CSL_SPI_SPIDELAY_T2CDELAY_MASK                                     (0x00FF0000U)
#define CSL_SPI_SPIDELAY_T2CDELAY_SHIFT                                    (0x00000010U)
#define CSL_SPI_SPIDELAY_T2CDELAY_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIDELAY_T2CDELAY_MAX                                      (0x000000FFU)

#define CSL_SPI_SPIDELAY_C2TDELAY_MASK                                     (0xFF000000U)
#define CSL_SPI_SPIDELAY_C2TDELAY_SHIFT                                    (0x00000018U)
#define CSL_SPI_SPIDELAY_C2TDELAY_RESETVAL                                 (0x00000000U)
#define CSL_SPI_SPIDELAY_C2TDELAY_MAX                                      (0x000000FFU)

#define CSL_SPI_SPIDELAY_RESETVAL                                          (0x00000000U)

/* SPIDEF */
#define CSL_SPI_SPIDEF_CSDEF0_MASK                                         (0x000000FFU)
#define CSL_SPI_SPIDEF_CSDEF0_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIDEF_CSDEF0_RESETVAL                                     (0x00000001U)
#define CSL_SPI_SPIDEF_CSDEF0_MAX                                          (0x000000FFU)

/*----CSDEF0 Tokens----*/
#define CSL_SPI_SPIDEF_CSDEF0_LOW                               (0x00000000U)
#define CSL_SPI_SPIDEF_CSDEF0_HIGH                              (0x00000001U)

#define CSL_SPI_SPIDEF_NU_MASK                                             (0xFFFFFF00U)
#define CSL_SPI_SPIDEF_NU_SHIFT                                            (0x00000008U)
#define CSL_SPI_SPIDEF_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIDEF_NU_MAX                                              (0x00FFFFFFU)

#define CSL_SPI_SPIDEF_RESETVAL                                            (0x00000001U)

/* SPIFMT */
#define CSL_SPI_SPIFMT_CHARLEN_MASK                                        (0x0000001FU)
#define CSL_SPI_SPIFMT_CHARLEN_SHIFT                                       (0x00000000U)
#define CSL_SPI_SPIFMT_CHARLEN_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIFMT_CHARLEN_MAX                                         (0x0000001FU)

#define CSL_SPI_SPIFMT_NU_MASK                                             (0x000000E0U)
#define CSL_SPI_SPIFMT_NU_SHIFT                                            (0x00000005U)
#define CSL_SPI_SPIFMT_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIFMT_NU_MAX                                              (0x00000007U)

#define CSL_SPI_SPIFMT_PRESCALE_MASK                                       (0x0000FF00U)
#define CSL_SPI_SPIFMT_PRESCALE_SHIFT                                      (0x00000008U)
#define CSL_SPI_SPIFMT_PRESCALE_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIFMT_PRESCALE_MAX                                        (0x000000FFU)

#define CSL_SPI_SPIFMT_PHASE_MASK                                          (0x00010000U)
#define CSL_SPI_SPIFMT_PHASE_SHIFT                                         (0x00000010U)
#define CSL_SPI_SPIFMT_PHASE_RESETVAL                                      (0x00000000U)
#define CSL_SPI_SPIFMT_PHASE_MAX                                           (0x00000001U)

#define CSL_SPI_SPIFMT_POLARITY_MASK                                       (0x00020000U)
#define CSL_SPI_SPIFMT_POLARITY_SHIFT                                      (0x00000011U)
#define CSL_SPI_SPIFMT_POLARITY_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIFMT_POLARITY_MAX                                        (0x00000001U)

#define CSL_SPI_SPIFMT_DISCSTIMERS_MASK                                    (0x00040000U)
#define CSL_SPI_SPIFMT_DISCSTIMERS_SHIFT                                   (0x00000012U)
#define CSL_SPI_SPIFMT_DISCSTIMERS_RESETVAL                                (0x00000000U)
#define CSL_SPI_SPIFMT_DISCSTIMERS_MAX                                     (0x00000001U)

#define CSL_SPI_SPIFMT_HDUPLEX_ENA_MASK                                    (0x00080000U)
#define CSL_SPI_SPIFMT_HDUPLEX_ENA_SHIFT                                   (0x00000013U)
#define CSL_SPI_SPIFMT_HDUPLEX_ENA_RESETVAL                                (0x00000000U)
#define CSL_SPI_SPIFMT_HDUPLEX_ENA_MAX                                     (0x00000001U)

#define CSL_SPI_SPIFMT_SHIFTDIR_MASK                                       (0x00100000U)
#define CSL_SPI_SPIFMT_SHIFTDIR_SHIFT                                      (0x00000014U)
#define CSL_SPI_SPIFMT_SHIFTDIR_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIFMT_SHIFTDIR_MAX                                        (0x00000001U)

#define CSL_SPI_SPIFMT_WAITENA_MASK                                        (0x00200000U)
#define CSL_SPI_SPIFMT_WAITENA_SHIFT                                       (0x00000015U)
#define CSL_SPI_SPIFMT_WAITENA_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SPIFMT_WAITENA_MAX                                         (0x00000001U)

#define CSL_SPI_SPIFMT_PARITYENA_MASK                                      (0x00400000U)
#define CSL_SPI_SPIFMT_PARITYENA_SHIFT                                     (0x00000016U)
#define CSL_SPI_SPIFMT_PARITYENA_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIFMT_PARITYENA_MAX                                       (0x00000001U)

#define CSL_SPI_SPIFMT_PARPOL_MASK                                         (0x00800000U)
#define CSL_SPI_SPIFMT_PARPOL_SHIFT                                        (0x00000017U)
#define CSL_SPI_SPIFMT_PARPOL_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIFMT_PARPOL_MAX                                          (0x00000001U)

#define CSL_SPI_SPIFMT_WDELAY_MASK                                         (0xFF000000U)
#define CSL_SPI_SPIFMT_WDELAY_SHIFT                                        (0x00000018U)
#define CSL_SPI_SPIFMT_WDELAY_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIFMT_WDELAY_MAX                                          (0x000000FFU)

#define CSL_SPI_SPIFMT_RESETVAL                                            (0x00000000U)


/* INTVEC */
#define CSL_SPI_INTVEC_SUSPEND_MASK                                        (0x00000001U)
#define CSL_SPI_INTVEC_SUSPEND_SHIFT                                       (0x00000000U)
#define CSL_SPI_INTVEC_SUSPEND_RESETVAL                                    (0x00000000U)
#define CSL_SPI_INTVEC_SUSPEND_MAX                                         (0x00000001U)

#define CSL_SPI_INTVEC_INTVECT_MASK                                        (0x0000003EU)
#define CSL_SPI_INTVEC_INTVECT_SHIFT                                       (0x00000001U)
#define CSL_SPI_INTVEC_INTVECT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_INTVEC_INTVECT_MAX                                         (0x0000001FU)

#define CSL_SPI_INTVEC_NU_MASK                                             (0xFFFFFFC0U)
#define CSL_SPI_INTVEC_NU_SHIFT                                            (0x00000006U)
#define CSL_SPI_INTVEC_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_INTVEC_NU_MAX                                              (0x03FFFFFFU)

#define CSL_SPI_INTVEC_RESETVAL                                            (0x00000000U)

/* SPIPC9 */
#define CSL_SPI_SPIPC9_SCSSRS_MASK                                         (0x000000FFU)
#define CSL_SPI_SPIPC9_SCSSRS_SHIFT                                        (0x00000000U)
#define CSL_SPI_SPIPC9_SCSSRS_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC9_SCSSRS_MAX                                          (0x000000FFU)

#define CSL_SPI_SPIPC9_ENASRS_MASK                                         (0x00000100U)
#define CSL_SPI_SPIPC9_ENASRS_SHIFT                                        (0x00000008U)
#define CSL_SPI_SPIPC9_ENASRS_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC9_ENASRS_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC9_CLKSRS_MASK                                         (0x00000200U)
#define CSL_SPI_SPIPC9_CLKSRS_SHIFT                                        (0x00000009U)
#define CSL_SPI_SPIPC9_CLKSRS_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPC9_CLKSRS_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPC9_SIMOSRS0_MASK                                       (0x00000400U)
#define CSL_SPI_SPIPC9_SIMOSRS0_SHIFT                                      (0x0000000AU)
#define CSL_SPI_SPIPC9_SIMOSRS0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC9_SIMOSRS0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC9_SOMISRS0_MASK                                       (0x00000800U)
#define CSL_SPI_SPIPC9_SOMISRS0_SHIFT                                      (0x0000000BU)
#define CSL_SPI_SPIPC9_SOMISRS0_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC9_SOMISRS0_MAX                                        (0x00000001U)

#define CSL_SPI_SPIPC9_NU_MASK                                             (0x0000F000U)
#define CSL_SPI_SPIPC9_NU_SHIFT                                            (0x0000000CU)
#define CSL_SPI_SPIPC9_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIPC9_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_SPIPC9_SIMOSRS7_MASK                                       (0x00FF0000U)
#define CSL_SPI_SPIPC9_SIMOSRS7_SHIFT                                      (0x00000010U)
#define CSL_SPI_SPIPC9_SIMOSRS7_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC9_SIMOSRS7_MAX                                        (0x000000FFU)

#define CSL_SPI_SPIPC9_SOMISRS7_MASK                                       (0xFF000000U)
#define CSL_SPI_SPIPC9_SOMISRS7_SHIFT                                      (0x00000018U)
#define CSL_SPI_SPIPC9_SOMISRS7_RESETVAL                                   (0x00000000U)
#define CSL_SPI_SPIPC9_SOMISRS7_MAX                                        (0x000000FFU)

#define CSL_SPI_SPIPC9_RESETVAL                                            (0x00000000U)

/* SPIPMCTRL */
#define CSL_SPI_SPIPMCTRL_PMODE0_MASK                                      (0x00000003U)
#define CSL_SPI_SPIPMCTRL_PMODE0_SHIFT                                     (0x00000000U)
#define CSL_SPI_SPIPMCTRL_PMODE0_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPMCTRL_PMODE0_MAX                                       (0x00000003U)

#define CSL_SPI_SPIPMCTRL_MMODE0_MASK                                      (0x0000001CU)
#define CSL_SPI_SPIPMCTRL_MMODE0_SHIFT                                     (0x00000002U)
#define CSL_SPI_SPIPMCTRL_MMODE0_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPMCTRL_MMODE0_MAX                                       (0x00000007U)

#define CSL_SPI_SPIPMCTRL_MODCLKPOL0_MASK                                  (0x00000020U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL0_SHIFT                                 (0x00000005U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL0_RESETVAL                              (0x00000000U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL0_MAX                                   (0x00000001U)

#define CSL_SPI_SPIPMCTRL_HSM_MODE0_MASK                                   (0x00000040U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE0_SHIFT                                  (0x00000006U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE0_RESETVAL                               (0x00000000U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE0_MAX                                    (0x00000001U)

#define CSL_SPI_SPIPMCTRL_NU1_MASK                                         (0x00000080U)
#define CSL_SPI_SPIPMCTRL_NU1_SHIFT                                        (0x00000007U)
#define CSL_SPI_SPIPMCTRL_NU1_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPMCTRL_NU1_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPMCTRL_PMODE1_MASK                                      (0x00000300U)
#define CSL_SPI_SPIPMCTRL_PMODE1_SHIFT                                     (0x00000008U)
#define CSL_SPI_SPIPMCTRL_PMODE1_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPMCTRL_PMODE1_MAX                                       (0x00000003U)

#define CSL_SPI_SPIPMCTRL_MMODE1_MASK                                      (0x00001C00U)
#define CSL_SPI_SPIPMCTRL_MMODE1_SHIFT                                     (0x0000000AU)
#define CSL_SPI_SPIPMCTRL_MMODE1_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPMCTRL_MMODE1_MAX                                       (0x00000007U)

#define CSL_SPI_SPIPMCTRL_MODCLKPOL1_MASK                                  (0x00002000U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL1_SHIFT                                 (0x0000000DU)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL1_RESETVAL                              (0x00000000U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL1_MAX                                   (0x00000001U)

#define CSL_SPI_SPIPMCTRL_HSM_MODE1_MASK                                   (0x00004000U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE1_SHIFT                                  (0x0000000EU)
#define CSL_SPI_SPIPMCTRL_HSM_MODE1_RESETVAL                               (0x00000000U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE1_MAX                                    (0x00000001U)

#define CSL_SPI_SPIPMCTRL_NU2_MASK                                         (0x00008000U)
#define CSL_SPI_SPIPMCTRL_NU2_SHIFT                                        (0x0000000FU)
#define CSL_SPI_SPIPMCTRL_NU2_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPMCTRL_NU2_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPMCTRL_PMODE2_MASK                                      (0x00030000U)
#define CSL_SPI_SPIPMCTRL_PMODE2_SHIFT                                     (0x00000010U)
#define CSL_SPI_SPIPMCTRL_PMODE2_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPMCTRL_PMODE2_MAX                                       (0x00000003U)

#define CSL_SPI_SPIPMCTRL_MMODE2_MASK                                      (0x001C0000U)
#define CSL_SPI_SPIPMCTRL_MMODE2_SHIFT                                     (0x00000012U)
#define CSL_SPI_SPIPMCTRL_MMODE2_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPMCTRL_MMODE2_MAX                                       (0x00000007U)

#define CSL_SPI_SPIPMCTRL_MODCLKPOL2_MASK                                  (0x00200000U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL2_SHIFT                                 (0x00000015U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL2_RESETVAL                              (0x00000000U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL2_MAX                                   (0x00000001U)

#define CSL_SPI_SPIPMCTRL_HSM_MODE2_MASK                                   (0x00400000U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE2_SHIFT                                  (0x00000016U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE2_RESETVAL                               (0x00000000U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE2_MAX                                    (0x00000001U)

#define CSL_SPI_SPIPMCTRL_NU3_MASK                                         (0x00800000U)
#define CSL_SPI_SPIPMCTRL_NU3_SHIFT                                        (0x00000017U)
#define CSL_SPI_SPIPMCTRL_NU3_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPMCTRL_NU3_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPMCTRL_PMODE3_MASK                                      (0x03000000U)
#define CSL_SPI_SPIPMCTRL_PMODE3_SHIFT                                     (0x00000018U)
#define CSL_SPI_SPIPMCTRL_PMODE3_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPMCTRL_PMODE3_MAX                                       (0x00000003U)

#define CSL_SPI_SPIPMCTRL_MMODE3_MASK                                      (0x1C000000U)
#define CSL_SPI_SPIPMCTRL_MMODE3_SHIFT                                     (0x0000001AU)
#define CSL_SPI_SPIPMCTRL_MMODE3_RESETVAL                                  (0x00000000U)
#define CSL_SPI_SPIPMCTRL_MMODE3_MAX                                       (0x00000007U)

#define CSL_SPI_SPIPMCTRL_MODCLKPOL3_MASK                                  (0x20000000U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL3_SHIFT                                 (0x0000001DU)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL3_RESETVAL                              (0x00000000U)
#define CSL_SPI_SPIPMCTRL_MODCLKPOL3_MAX                                   (0x00000001U)

#define CSL_SPI_SPIPMCTRL_HSM_MODE3_MASK                                   (0x40000000U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE3_SHIFT                                  (0x0000001EU)
#define CSL_SPI_SPIPMCTRL_HSM_MODE3_RESETVAL                               (0x00000000U)
#define CSL_SPI_SPIPMCTRL_HSM_MODE3_MAX                                    (0x00000001U)

#define CSL_SPI_SPIPMCTRL_NU4_MASK                                         (0x80000000U)
#define CSL_SPI_SPIPMCTRL_NU4_SHIFT                                        (0x0000001FU)
#define CSL_SPI_SPIPMCTRL_NU4_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIPMCTRL_NU4_MAX                                          (0x00000001U)

#define CSL_SPI_SPIPMCTRL_RESETVAL                                         (0x00000000U)

/* MIBSPIE */
#define CSL_SPI_MIBSPIE_MSPIENA_MASK                                       (0x00000001U)
#define CSL_SPI_MIBSPIE_MSPIENA_SHIFT                                      (0x00000000U)
#define CSL_SPI_MIBSPIE_MSPIENA_RESETVAL                                   (0x00000000U)
#define CSL_SPI_MIBSPIE_MSPIENA_MAX                                        (0x00000001U)

#define CSL_SPI_MIBSPIE_NU1_MASK                                           (0x000000FEU)
#define CSL_SPI_MIBSPIE_NU1_SHIFT                                          (0x00000001U)
#define CSL_SPI_MIBSPIE_NU1_RESETVAL                                       (0x00000000U)
#define CSL_SPI_MIBSPIE_NU1_MAX                                            (0x0000007FU)

#define CSL_SPI_MIBSPIE_EXTENDED_BUF_ENA_MASK                              (0x00000F00U)
#define CSL_SPI_MIBSPIE_EXTENDED_BUF_ENA_SHIFT                             (0x00000008U)
#define CSL_SPI_MIBSPIE_EXTENDED_BUF_ENA_RESETVAL                          (0x00000005U)
#define CSL_SPI_MIBSPIE_EXTENDED_BUF_ENA_MAX                               (0x0000000FU)

#define CSL_SPI_MIBSPIE_NU2_MASK                                           (0x0000F000U)
#define CSL_SPI_MIBSPIE_NU2_SHIFT                                          (0x0000000CU)
#define CSL_SPI_MIBSPIE_NU2_RESETVAL                                       (0x00000000U)
#define CSL_SPI_MIBSPIE_NU2_MAX                                            (0x0000000FU)

#define CSL_SPI_MIBSPIE_RXRAMACCESS_MASK                                   (0x00010000U)
#define CSL_SPI_MIBSPIE_RXRAMACCESS_SHIFT                                  (0x00000010U)
#define CSL_SPI_MIBSPIE_RXRAMACCESS_RESETVAL                               (0x00000000U)
#define CSL_SPI_MIBSPIE_RXRAMACCESS_MAX                                    (0x00000001U)

#define CSL_SPI_MIBSPIE_NU3_MASK                                           (0xFFFE0000U)
#define CSL_SPI_MIBSPIE_NU3_SHIFT                                          (0x00000011U)
#define CSL_SPI_MIBSPIE_NU3_RESETVAL                                       (0x00000000U)
#define CSL_SPI_MIBSPIE_NU3_MAX                                            (0x00007FFFU)

#define CSL_SPI_MIBSPIE_RESETVAL                                           (0x00000500U)

/* TGITENST */
#define CSL_SPI_TGITENST_SETINTENSUS_MASK                                  (0x0000FFFFU)
#define CSL_SPI_TGITENST_SETINTENSUS_SHIFT                                 (0x00000000U)
#define CSL_SPI_TGITENST_SETINTENSUS_RESETVAL                              (0x00000000U)
#define CSL_SPI_TGITENST_SETINTENSUS_MAX                                   (0x0000FFFFU)

#define CSL_SPI_TGITENST_SETINTENRDY_MASK                                  (0xFFFF0000U)
#define CSL_SPI_TGITENST_SETINTENRDY_SHIFT                                 (0x00000010U)
#define CSL_SPI_TGITENST_SETINTENRDY_RESETVAL                              (0x00000000U)
#define CSL_SPI_TGITENST_SETINTENRDY_MAX                                   (0x0000FFFFU)

#define CSL_SPI_TGITENST_RESETVAL                                          (0x00000000U)

/* TGITENCR */
#define CSL_SPI_TGITENCR_CLRINTENSUS_MASK                                  (0x0000FFFFU)
#define CSL_SPI_TGITENCR_CLRINTENSUS_SHIFT                                 (0x00000000U)
#define CSL_SPI_TGITENCR_CLRINTENSUS_RESETVAL                              (0x00000000U)
#define CSL_SPI_TGITENCR_CLRINTENSUS_MAX                                   (0x0000FFFFU)

#define CSL_SPI_TGITENCR_CLRINTENRDY_MASK                                  (0xFFFF0000U)
#define CSL_SPI_TGITENCR_CLRINTENRDY_SHIFT                                 (0x00000010U)
#define CSL_SPI_TGITENCR_CLRINTENRDY_RESETVAL                              (0x00000000U)
#define CSL_SPI_TGITENCR_CLRINTENRDY_MAX                                   (0x0000FFFFU)

#define CSL_SPI_TGITENCR_RESETVAL                                          (0x00000000U)

/* TGITLVST */
#define CSL_SPI_TGITLVST_SETINTLVLSUS_MASK                                 (0x0000FFFFU)
#define CSL_SPI_TGITLVST_SETINTLVLSUS_SHIFT                                (0x00000000U)
#define CSL_SPI_TGITLVST_SETINTLVLSUS_RESETVAL                             (0x00000000U)
#define CSL_SPI_TGITLVST_SETINTLVLSUS_MAX                                  (0x0000FFFFU)

#define CSL_SPI_TGITLVST_SETINTLVLRDY_MASK                                 (0xFFFF0000U)
#define CSL_SPI_TGITLVST_SETINTLVLRDY_SHIFT                                (0x00000010U)
#define CSL_SPI_TGITLVST_SETINTLVLRDY_RESETVAL                             (0x00000000U)
#define CSL_SPI_TGITLVST_SETINTLVLRDY_MAX                                  (0x0000FFFFU)

#define CSL_SPI_TGITLVST_RESETVAL                                          (0x00000000U)

/* TGITLVCR */
#define CSL_SPI_TGITLVCR_CLRINTLVLSUS_MASK                                 (0x0000FFFFU)
#define CSL_SPI_TGITLVCR_CLRINTLVLSUS_SHIFT                                (0x00000000U)
#define CSL_SPI_TGITLVCR_CLRINTLVLSUS_RESETVAL                             (0x00000000U)
#define CSL_SPI_TGITLVCR_CLRINTLVLSUS_MAX                                  (0x0000FFFFU)

#define CSL_SPI_TGITLVCR_CLRINTLVLRDY_MASK                                 (0xFFFF0000U)
#define CSL_SPI_TGITLVCR_CLRINTLVLRDY_SHIFT                                (0x00000010U)
#define CSL_SPI_TGITLVCR_CLRINTLVLRDY_RESETVAL                             (0x00000000U)
#define CSL_SPI_TGITLVCR_CLRINTLVLRDY_MAX                                  (0x0000FFFFU)

#define CSL_SPI_TGITLVCR_RESETVAL                                          (0x00000000U)

/* TGINTFLAG */
#define CSL_SPI_TGINTFLAG_INTFLGSUS_MASK                                   (0x0000FFFFU)
#define CSL_SPI_TGINTFLAG_INTFLGSUS_SHIFT                                  (0x00000000U)
#define CSL_SPI_TGINTFLAG_INTFLGSUS_RESETVAL                               (0x00000000U)
#define CSL_SPI_TGINTFLAG_INTFLGSUS_MAX                                    (0x0000FFFFU)

#define CSL_SPI_TGINTFLAG_INTFLGRDY_MASK                                   (0xFFFF0000U)
#define CSL_SPI_TGINTFLAG_INTFLGRDY_SHIFT                                  (0x00000010U)
#define CSL_SPI_TGINTFLAG_INTFLGRDY_RESETVAL                               (0x00000000U)
#define CSL_SPI_TGINTFLAG_INTFLGRDY_MAX                                    (0x0000FFFFU)

#define CSL_SPI_TGINTFLAG_RESETVAL                                         (0x00000000U)

/* TICKCNT */
#define CSL_SPI_TICKCNT_TICKVALUE_MASK                                     (0x0000FFFFU)
#define CSL_SPI_TICKCNT_TICKVALUE_SHIFT                                    (0x00000000U)
#define CSL_SPI_TICKCNT_TICKVALUE_RESETVAL                                 (0x00000000U)
#define CSL_SPI_TICKCNT_TICKVALUE_MAX                                      (0x0000FFFFU)

#define CSL_SPI_TICKCNT_NU_MASK                                            (0x0FFF0000U)
#define CSL_SPI_TICKCNT_NU_SHIFT                                           (0x00000010U)
#define CSL_SPI_TICKCNT_NU_RESETVAL                                        (0x00000000U)
#define CSL_SPI_TICKCNT_NU_MAX                                             (0x00000FFFU)

#define CSL_SPI_TICKCNT_CLKCTRL_MASK                                       (0x30000000U)
#define CSL_SPI_TICKCNT_CLKCTRL_SHIFT                                      (0x0000001CU)
#define CSL_SPI_TICKCNT_CLKCTRL_RESETVAL                                   (0x00000000U)
#define CSL_SPI_TICKCNT_CLKCTRL_MAX                                        (0x00000003U)

#define CSL_SPI_TICKCNT_RELOAD_MASK                                        (0x40000000U)
#define CSL_SPI_TICKCNT_RELOAD_SHIFT                                       (0x0000001EU)
#define CSL_SPI_TICKCNT_RELOAD_RESETVAL                                    (0x00000000U)
#define CSL_SPI_TICKCNT_RELOAD_MAX                                         (0x00000001U)

#define CSL_SPI_TICKCNT_TICKENA_MASK                                       (0x80000000U)
#define CSL_SPI_TICKCNT_TICKENA_SHIFT                                      (0x0000001FU)
#define CSL_SPI_TICKCNT_TICKENA_RESETVAL                                   (0x00000000U)
#define CSL_SPI_TICKCNT_TICKENA_MAX                                        (0x00000001U)

#define CSL_SPI_TICKCNT_RESETVAL                                           (0x00000000U)

/* LTGPEND */
#define CSL_SPI_LTGPEND_NU1_MASK                                           (0x000000FFU)
#define CSL_SPI_LTGPEND_NU1_SHIFT                                          (0x00000000U)
#define CSL_SPI_LTGPEND_NU1_RESETVAL                                       (0x00000000U)
#define CSL_SPI_LTGPEND_NU1_MAX                                            (0x000000FFU)

#define CSL_SPI_LTGPEND_LPEND_MASK                                         (0x0000FF00U)
#define CSL_SPI_LTGPEND_LPEND_SHIFT                                        (0x00000008U)
#define CSL_SPI_LTGPEND_LPEND_RESETVAL                                     (0x00000000U)
#define CSL_SPI_LTGPEND_LPEND_MAX                                          (0x000000FFU)

#define CSL_SPI_LTGPEND_NU2_MASK                                           (0x00FF0000U)
#define CSL_SPI_LTGPEND_NU2_SHIFT                                          (0x00000010U)
#define CSL_SPI_LTGPEND_NU2_RESETVAL                                       (0x00000000U)
#define CSL_SPI_LTGPEND_NU2_MAX                                            (0x000000FFU)

#define CSL_SPI_LTGPEND_TGINSERVICE_MASK                                   (0x1F000000U)
#define CSL_SPI_LTGPEND_TGINSERVICE_SHIFT                                  (0x00000018U)
#define CSL_SPI_LTGPEND_TGINSERVICE_RESETVAL                               (0x00000000U)
#define CSL_SPI_LTGPEND_TGINSERVICE_MAX                                    (0x0000001FU)

#define CSL_SPI_LTGPEND_NU3_MASK                                           (0xE0000000U)
#define CSL_SPI_LTGPEND_NU3_SHIFT                                          (0x0000001DU)
#define CSL_SPI_LTGPEND_NU3_RESETVAL                                       (0x00000000U)
#define CSL_SPI_LTGPEND_NU3_MAX                                            (0x00000007U)

#define CSL_SPI_LTGPEND_RESETVAL                                           (0x00000000U)

/* TGCTRL */
#define CSL_SPI_TGCTRL_PCURRENT_MASK                                       (0x000000FFU)
#define CSL_SPI_TGCTRL_PCURRENT_SHIFT                                      (0x00000000U)
#define CSL_SPI_TGCTRL_PCURRENT_RESETVAL                                   (0x00000000U)
#define CSL_SPI_TGCTRL_PCURRENT_MAX                                        (0x000000FFU)

#define CSL_SPI_TGCTRL_PSTART_MASK                                         (0x0000FF00U)
#define CSL_SPI_TGCTRL_PSTART_SHIFT                                        (0x00000008U)
#define CSL_SPI_TGCTRL_PSTART_RESETVAL                                     (0x00000000U)
#define CSL_SPI_TGCTRL_PSTART_MAX                                          (0x000000FFU)

#define CSL_SPI_TGCTRL_TRIGSRC_MASK                                        (0x000F0000U)
#define CSL_SPI_TGCTRL_TRIGSRC_SHIFT                                       (0x00000010U)
#define CSL_SPI_TGCTRL_TRIGSRC_RESETVAL                                    (0x00000000U)
#define CSL_SPI_TGCTRL_TRIGSRC_MAX                                         (0x0000000FU)

#define CSL_SPI_TGCTRL_TRIGEVT_MASK                                        (0x00F00000U)
#define CSL_SPI_TGCTRL_TRIGEVT_SHIFT                                       (0x00000014U)
#define CSL_SPI_TGCTRL_TRIGEVT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_TGCTRL_TRIGEVT_MAX                                         (0x0000000FU)

#define CSL_SPI_TGCTRL_NU_MASK                                             (0x0F000000U)
#define CSL_SPI_TGCTRL_NU_SHIFT                                            (0x00000018U)
#define CSL_SPI_TGCTRL_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_TGCTRL_NU_MAX                                              (0x0000000FU)

#define CSL_SPI_TGCTRL_TGTD_MASK                                           (0x10000000U)
#define CSL_SPI_TGCTRL_TGTD_SHIFT                                          (0x0000001CU)
#define CSL_SPI_TGCTRL_TGTD_RESETVAL                                       (0x00000000U)
#define CSL_SPI_TGCTRL_TGTD_MAX                                            (0x00000001U)

#define CSL_SPI_TGCTRL_PRST_MASK                                           (0x20000000U)
#define CSL_SPI_TGCTRL_PRST_SHIFT                                          (0x0000001DU)
#define CSL_SPI_TGCTRL_PRST_RESETVAL                                       (0x00000000U)
#define CSL_SPI_TGCTRL_PRST_MAX                                            (0x00000001U)

#define CSL_SPI_TGCTRL_ONESHOT_MASK                                        (0x40000000U)
#define CSL_SPI_TGCTRL_ONESHOT_SHIFT                                       (0x0000001EU)
#define CSL_SPI_TGCTRL_ONESHOT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_TGCTRL_ONESHOT_MAX                                         (0x00000001U)

#define CSL_SPI_TGCTRL_TGENA_MASK                                          (0x80000000U)
#define CSL_SPI_TGCTRL_TGENA_SHIFT                                         (0x0000001FU)
#define CSL_SPI_TGCTRL_TGENA_RESETVAL                                      (0x00000000U)
#define CSL_SPI_TGCTRL_TGENA_MAX                                           (0x00000001U)

#define CSL_SPI_TGCTRL_RESETVAL                                            (0x00000000U)

/* DMACTRL */
#define CSL_SPI_DMA0CTRL_COUNT_MASK                                        (0x0000003FU)
#define CSL_SPI_DMA0CTRL_COUNT_SHIFT                                       (0x00000000U)
#define CSL_SPI_DMA0CTRL_COUNT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_DMA0CTRL_COUNT_MAX                                         (0x0000003FU)

#define CSL_SPI_DMA0CTRL_COUNTBIT17_MASK                                   (0x00000040U)
#define CSL_SPI_DMA0CTRL_COUNTBIT17_SHIFT                                  (0x00000006U)
#define CSL_SPI_DMA0CTRL_COUNTBIT17_RESETVAL                               (0x00000000U)
#define CSL_SPI_DMA0CTRL_COUNTBIT17_MAX                                    (0x00000001U)

#define CSL_SPI_DMACTRL_BUFID7_MASK                                        (0x00000080U)
#define CSL_SPI_DMACTRL_BUFID7_SHIFT                                       (0x00000007U)
#define CSL_SPI_DMACTRL_BUFID7_RESETVAL                                    (0x00000000U)
#define CSL_SPI_DMACTRL_BUFID7_MAX                                         (0x00000001U)

#define CSL_SPI_DMACTRL_ICOUNT_MASK                                        (0x00001F00U)
#define CSL_SPI_DMACTRL_ICOUNT_SHIFT                                       (0x00000008U)
#define CSL_SPI_DMACTRL_ICOUNT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_DMACTRL_ICOUNT_MAX                                         (0x0000001FU)

#define CSL_SPI_DMACTRL_NOBRK_MASK                                         (0x00002000U)
#define CSL_SPI_DMACTRL_NOBRK_SHIFT                                        (0x0000000DU)
#define CSL_SPI_DMACTRL_NOBRK_RESETVAL                                     (0x00000000U)
#define CSL_SPI_DMACTRL_NOBRK_MAX                                          (0x00000001U)

#define CSL_SPI_DMACTRL_TXDMAENA_MASK                                      (0x00004000U)
#define CSL_SPI_DMACTRL_TXDMAENA_SHIFT                                     (0x0000000EU)
#define CSL_SPI_DMACTRL_TXDMAENA_RESETVAL                                  (0x00000000U)
#define CSL_SPI_DMACTRL_TXDMAENA_MAX                                       (0x00000001U)

#define CSL_SPI_DMACTRL_RXDMAENA_MASK                                      (0x00008000U)
#define CSL_SPI_DMACTRL_RXDMAENA_SHIFT                                     (0x0000000FU)
#define CSL_SPI_DMACTRL_RXDMAENA_RESETVAL                                  (0x00000000U)
#define CSL_SPI_DMACTRL_RXDMAENA_MAX                                       (0x00000001U)

#define CSL_SPI_DMACTRL_TXDMA_MAP_MASK                                     (0x000F0000U)
#define CSL_SPI_DMACTRL_TXDMA_MAP_SHIFT                                    (0x00000010U)
#define CSL_SPI_DMACTRL_TXDMA_MAP_RESETVAL                                 (0x00000000U)
#define CSL_SPI_DMACTRL_TXDMA_MAP_MAX                                      (0x0000000FU)

#define CSL_SPI_DMACTRL_RXDMA_MAP_MASK                                     (0x00F00000U)
#define CSL_SPI_DMACTRL_RXDMA_MAP_SHIFT                                    (0x00000014U)
#define CSL_SPI_DMACTRL_RXDMA_MAP_RESETVAL                                 (0x00000000U)
#define CSL_SPI_DMACTRL_RXDMA_MAP_MAX                                      (0x0000000FU)

#define CSL_SPI_DMACTRL_BUFID_MASK                                         (0x7F000000U)
#define CSL_SPI_DMACTRL_BUFID_SHIFT                                        (0x00000018U)
#define CSL_SPI_DMACTRL_BUFID_RESETVAL                                     (0x00000000U)
#define CSL_SPI_DMACTRL_BUFID_MAX                                          (0x0000007FU)

#define CSL_SPI_DMACTRL_ONESHOT_MASK                                       (0x80000000U)
#define CSL_SPI_DMACTRL_ONESHOT_SHIFT                                      (0x0000001FU)
#define CSL_SPI_DMACTRL_ONESHOT_RESETVAL                                   (0x00000000U)
#define CSL_SPI_DMACTRL_ONESHOT_MAX                                        (0x00000001U)

#define CSL_SPI_DMACTRL_RESETVAL                                           (0x00000000U)

/* DMACOUNT */
#define CSL_SPI_DMACOUNT_COUNT_MASK                                        (0x0000FFFFU)
#define CSL_SPI_DMACOUNT_COUNT_SHIFT                                       (0x00000000U)
#define CSL_SPI_DMACOUNT_COUNT_RESETVAL                                    (0x00000000U)
#define CSL_SPI_DMACOUNT_COUNT_MAX                                         (0x0000FFFFU)

#define CSL_SPI_DMACOUNT_ICOUNT_MASK                                       (0xFFFF0000U)
#define CSL_SPI_DMACOUNT_ICOUNT_SHIFT                                      (0x00000010U)
#define CSL_SPI_DMACOUNT_ICOUNT_RESETVAL                                   (0x00000000U)
#define CSL_SPI_DMACOUNT_ICOUNT_MAX                                        (0x0000FFFFU)

#define CSL_SPI_DMACOUNT_RESETVAL                                          (0x00000000U)

/* DMACNTLEN */
#define CSL_SPI_DMACNTLEN_LARGE_COUNT_MASK                                 (0x00000001U)
#define CSL_SPI_DMACNTLEN_LARGE_COUNT_SHIFT                                (0x00000000U)
#define CSL_SPI_DMACNTLEN_LARGE_COUNT_RESETVAL                             (0x00000000U)
#define CSL_SPI_DMACNTLEN_LARGE_COUNT_MAX                                  (0x00000001U)

#define CSL_SPI_DMACNTLEN_NU_MASK                                          (0xFFFFFFFEU)
#define CSL_SPI_DMACNTLEN_NU_SHIFT                                         (0x00000001U)
#define CSL_SPI_DMACNTLEN_NU_RESETVAL                                      (0x00000000U)
#define CSL_SPI_DMACNTLEN_NU_MAX                                           (0x7FFFFFFFU)

#define CSL_SPI_DMACNTLEN_RESETVAL                                         (0x00000000U)

/* PAR_ECC_CTRL */
#define CSL_SPI_PAR_ECC_CTRL_EDEN_MASK                                     (0x0000000FU)
#define CSL_SPI_PAR_ECC_CTRL_EDEN_SHIFT                                    (0x00000000U)
#define CSL_SPI_PAR_ECC_CTRL_EDEN_RESETVAL                                 (0x00000005U)
#define CSL_SPI_PAR_ECC_CTRL_EDEN_MAX                                      (0x0000000FU)

#define CSL_SPI_PAR_ECC_CTRL_NU1_MASK                                      (0x000000F0U)
#define CSL_SPI_PAR_ECC_CTRL_NU1_SHIFT                                     (0x00000004U)
#define CSL_SPI_PAR_ECC_CTRL_NU1_RESETVAL                                  (0x00000000U)
#define CSL_SPI_PAR_ECC_CTRL_NU1_MAX                                       (0x0000000FU)

#define CSL_SPI_PAR_ECC_CTRL_PTESTEN_MASK                                  (0x00000100U)
#define CSL_SPI_PAR_ECC_CTRL_PTESTEN_SHIFT                                 (0x00000008U)
#define CSL_SPI_PAR_ECC_CTRL_PTESTEN_RESETVAL                              (0x00000000U)
#define CSL_SPI_PAR_ECC_CTRL_PTESTEN_MAX                                   (0x00000001U)

#define CSL_SPI_PAR_ECC_CTRL_NU2_MASK                                      (0x0000FE00U)
#define CSL_SPI_PAR_ECC_CTRL_NU2_SHIFT                                     (0x00000009U)
#define CSL_SPI_PAR_ECC_CTRL_NU2_RESETVAL                                  (0x00000000U)
#define CSL_SPI_PAR_ECC_CTRL_NU2_MAX                                       (0x0000007FU)

#define CSL_SPI_PAR_ECC_CTRL_EDAC_MODE_MASK                                (0x000F0000U)
#define CSL_SPI_PAR_ECC_CTRL_EDAC_MODE_SHIFT                               (0x00000010U)
#define CSL_SPI_PAR_ECC_CTRL_EDAC_MODE_RESETVAL                            (0x0000000AU)
#define CSL_SPI_PAR_ECC_CTRL_EDAC_MODE_MAX                                 (0x0000000FU)

#define CSL_SPI_PAR_ECC_CTRL_NU3_MASK                                      (0x00F00000U)
#define CSL_SPI_PAR_ECC_CTRL_NU3_SHIFT                                     (0x00000014U)
#define CSL_SPI_PAR_ECC_CTRL_NU3_RESETVAL                                  (0x00000000U)
#define CSL_SPI_PAR_ECC_CTRL_NU3_MAX                                       (0x0000000FU)

#define CSL_SPI_PAR_ECC_CTRL_SBE_EVT_EN_MASK                               (0x0F000000U)
#define CSL_SPI_PAR_ECC_CTRL_SBE_EVT_EN_SHIFT                              (0x00000018U)
#define CSL_SPI_PAR_ECC_CTRL_SBE_EVT_EN_RESETVAL                           (0x00000005U)
#define CSL_SPI_PAR_ECC_CTRL_SBE_EVT_EN_MAX                                (0x0000000FU)

#define CSL_SPI_PAR_ECC_CTRL_NU4_MASK                                      (0xF0000000U)
#define CSL_SPI_PAR_ECC_CTRL_NU4_SHIFT                                     (0x0000001CU)
#define CSL_SPI_PAR_ECC_CTRL_NU4_RESETVAL                                  (0x00000000U)
#define CSL_SPI_PAR_ECC_CTRL_NU4_MAX                                       (0x0000000FU)

#define CSL_SPI_PAR_ECC_CTRL_RESETVAL                                      (0x050A0005U)

/* PAR_ECC_STAT */
#define CSL_SPI_PAR_ECC_STAT_UERR_FLG0_MASK                                (0x00000001U)
#define CSL_SPI_PAR_ECC_STAT_UERR_FLG0_SHIFT                               (0x00000000U)
#define CSL_SPI_PAR_ECC_STAT_UERR_FLG0_RESETVAL                            (0x00000000U)
#define CSL_SPI_PAR_ECC_STAT_UERR_FLG0_MAX                                 (0x00000001U)

#define CSL_SPI_PAR_ECC_STAT_UERR_FLG1_MASK                                (0x00000002U)
#define CSL_SPI_PAR_ECC_STAT_UERR_FLG1_SHIFT                               (0x00000001U)
#define CSL_SPI_PAR_ECC_STAT_UERR_FLG1_RESETVAL                            (0x00000000U)
#define CSL_SPI_PAR_ECC_STAT_UERR_FLG1_MAX                                 (0x00000001U)

#define CSL_SPI_PAR_ECC_STAT_NU1_MASK                                      (0x000000FCU)
#define CSL_SPI_PAR_ECC_STAT_NU1_SHIFT                                     (0x00000002U)
#define CSL_SPI_PAR_ECC_STAT_NU1_RESETVAL                                  (0x00000000U)
#define CSL_SPI_PAR_ECC_STAT_NU1_MAX                                       (0x0000003FU)

#define CSL_SPI_PAR_ECC_STAT_SBE_FLG0_MASK                                 (0x00000100U)
#define CSL_SPI_PAR_ECC_STAT_SBE_FLG0_SHIFT                                (0x00000008U)
#define CSL_SPI_PAR_ECC_STAT_SBE_FLG0_RESETVAL                             (0x00000000U)
#define CSL_SPI_PAR_ECC_STAT_SBE_FLG0_MAX                                  (0x00000001U)

#define CSL_SPI_PAR_ECC_STAT_SBE_FLG1_MASK                                 (0x00000200U)
#define CSL_SPI_PAR_ECC_STAT_SBE_FLG1_SHIFT                                (0x00000009U)
#define CSL_SPI_PAR_ECC_STAT_SBE_FLG1_RESETVAL                             (0x00000000U)
#define CSL_SPI_PAR_ECC_STAT_SBE_FLG1_MAX                                  (0x00000001U)

#define CSL_SPI_PAR_ECC_STAT_NU2_MASK                                      (0xFFFFFC00U)
#define CSL_SPI_PAR_ECC_STAT_NU2_SHIFT                                     (0x0000000AU)
#define CSL_SPI_PAR_ECC_STAT_NU2_RESETVAL                                  (0x00000000U)
#define CSL_SPI_PAR_ECC_STAT_NU2_MAX                                       (0x003FFFFFU)

#define CSL_SPI_PAR_ECC_STAT_RESETVAL                                      (0x00000000U)

/* UERRADDR1 */
#define CSL_SPI_UERRADDR1_UERRADDR1_MASK                                   (0x000007FFU)
#define CSL_SPI_UERRADDR1_UERRADDR1_SHIFT                                  (0x00000000U)
#define CSL_SPI_UERRADDR1_UERRADDR1_RESETVAL                               (0x00000000U)
#define CSL_SPI_UERRADDR1_UERRADDR1_MAX                                    (0x000007FFU)

#define CSL_SPI_UERRADDR1_NU_MASK                                          (0xFFFFF800U)
#define CSL_SPI_UERRADDR1_NU_SHIFT                                         (0x0000000BU)
#define CSL_SPI_UERRADDR1_NU_RESETVAL                                      (0x00000000U)
#define CSL_SPI_UERRADDR1_NU_MAX                                           (0x001FFFFFU)

#define CSL_SPI_UERRADDR1_RESETVAL                                         (0x00000000U)

/* UERRADDR0 */
#define CSL_SPI_UERRADDR0_UERRADDR0_MASK                                   (0x000007FFU)
#define CSL_SPI_UERRADDR0_UERRADDR0_SHIFT                                  (0x00000000U)
#define CSL_SPI_UERRADDR0_UERRADDR0_RESETVAL                               (0x00000000U)
#define CSL_SPI_UERRADDR0_UERRADDR0_MAX                                    (0x000007FFU)

#define CSL_SPI_UERRADDR0_NU_MASK                                          (0xFFFFF800U)
#define CSL_SPI_UERRADDR0_NU_SHIFT                                         (0x0000000BU)
#define CSL_SPI_UERRADDR0_NU_RESETVAL                                      (0x00000000U)
#define CSL_SPI_UERRADDR0_NU_MAX                                           (0x001FFFFFU)

#define CSL_SPI_UERRADDR0_RESETVAL                                         (0x00000000U)

/* RXOVRN_BUF_ADDR */
#define CSL_SPI_RXOVRN_BUF_ADDR_RXOVRN_BUF_ADDR_MASK                       (0x000007FFU)
#define CSL_SPI_RXOVRN_BUF_ADDR_RXOVRN_BUF_ADDR_SHIFT                      (0x00000000U)
#define CSL_SPI_RXOVRN_BUF_ADDR_RXOVRN_BUF_ADDR_RESETVAL                   (0x00000200U)
#define CSL_SPI_RXOVRN_BUF_ADDR_RXOVRN_BUF_ADDR_MAX                        (0x000007FFU)

#define CSL_SPI_RXOVRN_BUF_ADDR_NU_MASK                                    (0xFFFFF800U)
#define CSL_SPI_RXOVRN_BUF_ADDR_NU_SHIFT                                   (0x0000000BU)
#define CSL_SPI_RXOVRN_BUF_ADDR_NU_RESETVAL                                (0x00000000U)
#define CSL_SPI_RXOVRN_BUF_ADDR_NU_MAX                                     (0x001FFFFFU)

#define CSL_SPI_RXOVRN_BUF_ADDR_RESETVAL                                   (0x00000200U)

/* IOLPBKTSTCR */
#define CSL_SPI_IOLPBKTSTCR_RXPENA_MASK                                    (0x00000001U)
#define CSL_SPI_IOLPBKTSTCR_RXPENA_SHIFT                                   (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_RXPENA_RESETVAL                                (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_RXPENA_MAX                                     (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_LPBKTYPE_MASK                                  (0x00000002U)
#define CSL_SPI_IOLPBKTSTCR_LPBKTYPE_SHIFT                                 (0x00000001U)
#define CSL_SPI_IOLPBKTSTCR_LPBKTYPE_RESETVAL                              (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_LPBKTYPE_MAX                                   (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_CTRLSCSPINERR_MASK                             (0x00000004U)
#define CSL_SPI_IOLPBKTSTCR_CTRLSCSPINERR_SHIFT                            (0x00000002U)
#define CSL_SPI_IOLPBKTSTCR_CTRLSCSPINERR_RESETVAL                         (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLSCSPINERR_MAX                              (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_ERRSCSPIN_MASK                                 (0x00000038U)
#define CSL_SPI_IOLPBKTSTCR_ERRSCSPIN_SHIFT                                (0x00000003U)
#define CSL_SPI_IOLPBKTSTCR_ERRSCSPIN_RESETVAL                             (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_ERRSCSPIN_MAX                                  (0x00000007U)

#define CSL_SPI_IOLPBKTSTCR_NU1_MASK                                       (0x000000C0U)
#define CSL_SPI_IOLPBKTSTCR_NU1_SHIFT                                      (0x00000006U)
#define CSL_SPI_IOLPBKTSTCR_NU1_RESETVAL                                   (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_NU1_MAX                                        (0x00000003U)

#define CSL_SPI_IOLPBKTSTCR_IOLPBKTSTENA_MASK                              (0x00000F00U)
#define CSL_SPI_IOLPBKTSTCR_IOLPBKTSTENA_SHIFT                             (0x00000008U)
#define CSL_SPI_IOLPBKTSTCR_IOLPBKTSTENA_RESETVAL                          (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_IOLPBKTSTENA_MAX                               (0x0000000FU)

#define CSL_SPI_IOLPBKTSTCR_NU2_MASK                                       (0x0000F000U)
#define CSL_SPI_IOLPBKTSTCR_NU2_SHIFT                                      (0x0000000CU)
#define CSL_SPI_IOLPBKTSTCR_NU2_RESETVAL                                   (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_NU2_MAX                                        (0x0000000FU)

#define CSL_SPI_IOLPBKTSTCR_CTRLDLENERR_MASK                               (0x00010000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLDLENERR_SHIFT                              (0x00000010U)
#define CSL_SPI_IOLPBKTSTCR_CTRLDLENERR_RESETVAL                           (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLDLENERR_MAX                                (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_CTRLTIMEOUT_MASK                               (0x00020000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLTIMEOUT_SHIFT                              (0x00000011U)
#define CSL_SPI_IOLPBKTSTCR_CTRLTIMEOUT_RESETVAL                           (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLTIMEOUT_MAX                                (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_CTRLPARERR_MASK                                (0x00040000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLPARERR_SHIFT                               (0x00000012U)
#define CSL_SPI_IOLPBKTSTCR_CTRLPARERR_RESETVAL                            (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLPARERR_MAX                                 (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_CTRLDESYNC_MASK                                (0x00080000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLDESYNC_SHIFT                               (0x00000013U)
#define CSL_SPI_IOLPBKTSTCR_CTRLDESYNC_RESETVAL                            (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLDESYNC_MAX                                 (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_CTRLBITERR_MASK                                (0x00100000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLBITERR_SHIFT                               (0x00000014U)
#define CSL_SPI_IOLPBKTSTCR_CTRLBITERR_RESETVAL                            (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_CTRLBITERR_MAX                                 (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_NU3_MASK                                       (0x00E00000U)
#define CSL_SPI_IOLPBKTSTCR_NU3_SHIFT                                      (0x00000015U)
#define CSL_SPI_IOLPBKTSTCR_NU3_RESETVAL                                   (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_NU3_MAX                                        (0x00000007U)

#define CSL_SPI_IOLPBKTSTCR_SCSFAILFLG_MASK                                (0x01000000U)
#define CSL_SPI_IOLPBKTSTCR_SCSFAILFLG_SHIFT                               (0x00000018U)
#define CSL_SPI_IOLPBKTSTCR_SCSFAILFLG_RESETVAL                            (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_SCSFAILFLG_MAX                                 (0x00000001U)

#define CSL_SPI_IOLPBKTSTCR_NU4_MASK                                       (0xFE000000U)
#define CSL_SPI_IOLPBKTSTCR_NU4_SHIFT                                      (0x00000019U)
#define CSL_SPI_IOLPBKTSTCR_NU4_RESETVAL                                   (0x00000000U)
#define CSL_SPI_IOLPBKTSTCR_NU4_MAX                                        (0x0000007FU)

#define CSL_SPI_IOLPBKTSTCR_RESETVAL                                       (0x00000000U)

/* EXTENDED_PRESCALE1 */
#define CSL_SPI_EXTENDED_PRESCALE1_EPRESCLAE_FMT0_MASK                     (0x000007FFU)
#define CSL_SPI_EXTENDED_PRESCALE1_EPRESCLAE_FMT0_SHIFT                    (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE1_EPRESCLAE_FMT0_RESETVAL                 (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE1_EPRESCLAE_FMT0_MAX                      (0x000007FFU)

#define CSL_SPI_EXTENDED_PRESCALE1_NU1_MASK                                (0x0000F800U)
#define CSL_SPI_EXTENDED_PRESCALE1_NU1_SHIFT                               (0x0000000BU)
#define CSL_SPI_EXTENDED_PRESCALE1_NU1_RESETVAL                            (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE1_NU1_MAX                                 (0x0000001FU)

#define CSL_SPI_EXTENDED_PRESCALE1_EPRESCLAE_FMT1_MASK                     (0x07FF0000U)
#define CSL_SPI_EXTENDED_PRESCALE1_EPRESCLAE_FMT1_SHIFT                    (0x00000010U)
#define CSL_SPI_EXTENDED_PRESCALE1_EPRESCLAE_FMT1_RESETVAL                 (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE1_EPRESCLAE_FMT1_MAX                      (0x000007FFU)

#define CSL_SPI_EXTENDED_PRESCALE1_NU2_MASK                                (0xF8000000U)
#define CSL_SPI_EXTENDED_PRESCALE1_NU2_SHIFT                               (0x0000001BU)
#define CSL_SPI_EXTENDED_PRESCALE1_NU2_RESETVAL                            (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE1_NU2_MAX                                 (0x0000001FU)

#define CSL_SPI_EXTENDED_PRESCALE1_RESETVAL                                (0x00000000U)

/* EXTENDED_PRESCALE2 */
#define CSL_SPI_EXTENDED_PRESCALE2_EPRESCLAE_FMT2_MASK                     (0x000007FFU)
#define CSL_SPI_EXTENDED_PRESCALE2_EPRESCLAE_FMT2_SHIFT                    (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE2_EPRESCLAE_FMT2_RESETVAL                 (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE2_EPRESCLAE_FMT2_MAX                      (0x000007FFU)

#define CSL_SPI_EXTENDED_PRESCALE2_NU3_MASK                                (0x0000F800U)
#define CSL_SPI_EXTENDED_PRESCALE2_NU3_SHIFT                               (0x0000000BU)
#define CSL_SPI_EXTENDED_PRESCALE2_NU3_RESETVAL                            (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE2_NU3_MAX                                 (0x0000001FU)

#define CSL_SPI_EXTENDED_PRESCALE2_EPRESCLAE_FMT3_MASK                     (0x07FF0000U)
#define CSL_SPI_EXTENDED_PRESCALE2_EPRESCLAE_FMT3_SHIFT                    (0x00000010U)
#define CSL_SPI_EXTENDED_PRESCALE2_EPRESCLAE_FMT3_RESETVAL                 (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE2_EPRESCLAE_FMT3_MAX                      (0x000007FFU)

#define CSL_SPI_EXTENDED_PRESCALE2_NU4_MASK                                (0xF8000000U)
#define CSL_SPI_EXTENDED_PRESCALE2_NU4_SHIFT                               (0x0000001BU)
#define CSL_SPI_EXTENDED_PRESCALE2_NU4_RESETVAL                            (0x00000000U)
#define CSL_SPI_EXTENDED_PRESCALE2_NU4_MAX                                 (0x0000001FU)

#define CSL_SPI_EXTENDED_PRESCALE2_RESETVAL                                (0x00000000U)

/* ECCDIAG_CTRL */
#define CSL_SPI_ECCDIAG_CTRL_ECCDIAG_EN_MASK                               (0x0000000FU)
#define CSL_SPI_ECCDIAG_CTRL_ECCDIAG_EN_SHIFT                              (0x00000000U)
#define CSL_SPI_ECCDIAG_CTRL_ECCDIAG_EN_RESETVAL                           (0x0000000AU)
#define CSL_SPI_ECCDIAG_CTRL_ECCDIAG_EN_MAX                                (0x0000000FU)

#define CSL_SPI_ECCDIAG_CTRL_NU_MASK                                       (0xFFFFFFF0U)
#define CSL_SPI_ECCDIAG_CTRL_NU_SHIFT                                      (0x00000004U)
#define CSL_SPI_ECCDIAG_CTRL_NU_RESETVAL                                   (0x00000000U)
#define CSL_SPI_ECCDIAG_CTRL_NU_MAX                                        (0x0FFFFFFFU)

#define CSL_SPI_ECCDIAG_CTRL_RESETVAL                                      (0x0000000AU)

/* ECCDIAG_STAT */
#define CSL_SPI_ECCDIAG_STAT_SEFLG0_MASK                                   (0x00000001U)
#define CSL_SPI_ECCDIAG_STAT_SEFLG0_SHIFT                                  (0x00000000U)
#define CSL_SPI_ECCDIAG_STAT_SEFLG0_RESETVAL                               (0x00000000U)
#define CSL_SPI_ECCDIAG_STAT_SEFLG0_MAX                                    (0x00000001U)

#define CSL_SPI_ECCDIAG_STAT_SEFLG1_MASK                                   (0x00000002U)
#define CSL_SPI_ECCDIAG_STAT_SEFLG1_SHIFT                                  (0x00000001U)
#define CSL_SPI_ECCDIAG_STAT_SEFLG1_RESETVAL                               (0x00000000U)
#define CSL_SPI_ECCDIAG_STAT_SEFLG1_MAX                                    (0x00000001U)

#define CSL_SPI_ECCDIAG_STAT_NU1_MASK                                      (0x0000FFFCU)
#define CSL_SPI_ECCDIAG_STAT_NU1_SHIFT                                     (0x00000002U)
#define CSL_SPI_ECCDIAG_STAT_NU1_RESETVAL                                  (0x00000000U)
#define CSL_SPI_ECCDIAG_STAT_NU1_MAX                                       (0x00003FFFU)

#define CSL_SPI_ECCDIAG_STAT_DEFLG0_MASK                                   (0x00010000U)
#define CSL_SPI_ECCDIAG_STAT_DEFLG0_SHIFT                                  (0x00000010U)
#define CSL_SPI_ECCDIAG_STAT_DEFLG0_RESETVAL                               (0x00000000U)
#define CSL_SPI_ECCDIAG_STAT_DEFLG0_MAX                                    (0x00000001U)

#define CSL_SPI_ECCDIAG_STAT_DEFLG1_MASK                                   (0x00020000U)
#define CSL_SPI_ECCDIAG_STAT_DEFLG1_SHIFT                                  (0x00000011U)
#define CSL_SPI_ECCDIAG_STAT_DEFLG1_RESETVAL                               (0x00000000U)
#define CSL_SPI_ECCDIAG_STAT_DEFLG1_MAX                                    (0x00000001U)

#define CSL_SPI_ECCDIAG_STAT_NU2_MASK                                      (0xFFFC0000U)
#define CSL_SPI_ECCDIAG_STAT_NU2_SHIFT                                     (0x00000012U)
#define CSL_SPI_ECCDIAG_STAT_NU2_RESETVAL                                  (0x00000000U)
#define CSL_SPI_ECCDIAG_STAT_NU2_MAX                                       (0x00003FFFU)

#define CSL_SPI_ECCDIAG_STAT_RESETVAL                                      (0x00000000U)

/* SBERRADDR1 */
#define CSL_SPI_SBERRADDR1_SBERRADDR1_MASK                                 (0x000007FFU)
#define CSL_SPI_SBERRADDR1_SBERRADDR1_SHIFT                                (0x00000000U)
#define CSL_SPI_SBERRADDR1_SBERRADDR1_RESETVAL                             (0x00000000U)
#define CSL_SPI_SBERRADDR1_SBERRADDR1_MAX                                  (0x000007FFU)

#define CSL_SPI_SBERRADDR1_NU1_MASK                                        (0xFFFFF800U)
#define CSL_SPI_SBERRADDR1_NU1_SHIFT                                       (0x0000000BU)
#define CSL_SPI_SBERRADDR1_NU1_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SBERRADDR1_NU1_MAX                                         (0x001FFFFFU)

#define CSL_SPI_SBERRADDR1_RESETVAL                                        (0x00000000U)

/* SBERRADDR0 */
#define CSL_SPI_SBERRADDR0_SBERRADDR0_MASK                                 (0x000007FFU)
#define CSL_SPI_SBERRADDR0_SBERRADDR0_SHIFT                                (0x00000000U)
#define CSL_SPI_SBERRADDR0_SBERRADDR0_RESETVAL                             (0x00000000U)
#define CSL_SPI_SBERRADDR0_SBERRADDR0_MAX                                  (0x000007FFU)

#define CSL_SPI_SBERRADDR0_NU2_MASK                                        (0xFFFFF800U)
#define CSL_SPI_SBERRADDR0_NU2_SHIFT                                       (0x0000000BU)
#define CSL_SPI_SBERRADDR0_NU2_RESETVAL                                    (0x00000000U)
#define CSL_SPI_SBERRADDR0_NU2_MAX                                         (0x001FFFFFU)

#define CSL_SPI_SBERRADDR0_RESETVAL                                        (0x00000000U)

/* SPIREV */
#define CSL_SPI_SPIREV_MINOR_MASK                                          (0x0000003FU)
#define CSL_SPI_SPIREV_MINOR_SHIFT                                         (0x00000000U)
#define CSL_SPI_SPIREV_MINOR_RESETVAL                                      (0x00000008U)
#define CSL_SPI_SPIREV_MINOR_MAX                                           (0x0000003FU)

#define CSL_SPI_SPIREV_CUSTOM_MASK                                         (0x000000C0U)
#define CSL_SPI_SPIREV_CUSTOM_SHIFT                                        (0x00000006U)
#define CSL_SPI_SPIREV_CUSTOM_RESETVAL                                     (0x00000000U)
#define CSL_SPI_SPIREV_CUSTOM_MAX                                          (0x00000003U)

#define CSL_SPI_SPIREV_MAJOR_MASK                                          (0x00000700U)
#define CSL_SPI_SPIREV_MAJOR_SHIFT                                         (0x00000008U)
#define CSL_SPI_SPIREV_MAJOR_RESETVAL                                      (0x00000003U)
#define CSL_SPI_SPIREV_MAJOR_MAX                                           (0x00000007U)

#define CSL_SPI_SPIREV_RTL_MASK                                            (0x0000F800U)
#define CSL_SPI_SPIREV_RTL_SHIFT                                           (0x0000000BU)
#define CSL_SPI_SPIREV_RTL_RESETVAL                                        (0x00000000U)
#define CSL_SPI_SPIREV_RTL_MAX                                             (0x0000001FU)

#define CSL_SPI_SPIREV_FUNC_MASK                                           (0x0FFF0000U)
#define CSL_SPI_SPIREV_FUNC_SHIFT                                          (0x00000010U)
#define CSL_SPI_SPIREV_FUNC_RESETVAL                                       (0x00000A05U)
#define CSL_SPI_SPIREV_FUNC_MAX                                            (0x00000FFFU)

#define CSL_SPI_SPIREV_NU_MASK                                             (0x30000000U)
#define CSL_SPI_SPIREV_NU_SHIFT                                            (0x0000001CU)
#define CSL_SPI_SPIREV_NU_RESETVAL                                         (0x00000000U)
#define CSL_SPI_SPIREV_NU_MAX                                              (0x00000003U)

#define CSL_SPI_SPIREV_SCHEME_MASK                                         (0xC0000000U)
#define CSL_SPI_SPIREV_SCHEME_SHIFT                                        (0x0000001EU)
#define CSL_SPI_SPIREV_SCHEME_RESETVAL                                     (0x00000001U)
#define CSL_SPI_SPIREV_SCHEME_MAX                                          (0x00000003U)

#define CSL_SPI_SPIREV_RESETVAL                                            (0x4A050308U)

#ifdef __cplusplus
}
#endif
#endif
