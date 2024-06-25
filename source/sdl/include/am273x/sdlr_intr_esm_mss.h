/********************************************************************
*
* MSS ESM INTR Map Header file
*
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
*/
#ifndef SDLR_INTR_ESM_MSS_H_
#define SDLR_INTR_ESM_MSS_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* List of intr sources for receiver: esm group 1 */
#define SDL_ESMG1_HSM_ESM_HI_INT                0
#define SDL_ESMG1_HSM_ESM_LO_INT                1

#define SDL_ESMG1_DSS_ESM_HI_INT                12
#define SDL_ESMG1_DSS_ESM_LO_INT                13
#define SDL_ESMG1_SWTRIG_IRQ0                   14
#define SDL_ESMG1_SWTRIG_IRQ1                   15
#define SDL_ESMG1_SWTRIG_IRQ2                   16
#define SDL_ESMG1_ECCAGGMSS_UERR                17
#define SDL_ESMG1_ECCAGGMSS_SERR                18
#define SDL_ESMG1_ECCAGGB_UERR                  19
#define SDL_ESMG1_ECCAGGB_SERR                  20
#define SDL_ESMG1_ECCAGGA_UERR                  21
#define SDL_ESMG1_ECCAGGA_SERR                  22
#define SDL_ESMG1_QSPI_WRERR                    23
#define SDL_ESMG1_CCMR5_ST_ERR                  25
#define SDL_ESMG1_STC_ERROR                     26
#define SDL_ESMG1_EFC_ERROR_SYNC                27
//#define SDL_ESMG1_TPTC_C_ERR                  28
#define SDL_ESMG1_MSS_TPCC_B_INTAGG_ERR         29
#define SDL_ESMG1_MSS_TPCC_A_INTAGG_ERR         30
#define SDL_ESMG1_DDATA1_SERR                   31
#define SDL_ESMG1_DTAG1_SERR                    32
#define SDL_ESMG1_IDATA1_SERR                   33
#define SDL_ESMG1_ITAG1_SERR                    34
#define SDL_ESMG1_DDATA0_SERR                   35
#define SDL_ESMG1_DTAG0_SERR                    36
#define SDL_ESMG1_IDATA0_SERR                   37
#define SDL_ESMG1_ITAG0_SERR                    38
#define SDL_ESMG1_B0TCM1_SERR                   39
#define SDL_ESMG1_B1TCM1_SERR                   40
#define SDL_ESMG1_ATCM1_SERR                    41
#define SDL_ESMG1_B0TCM0_SERR                   42
#define SDL_ESMG1_B1TCM0_SERR                   43
#define SDL_ESMG1_ATCM0_SERR                    44
#define SDL_ESMG1_NERROR_IN_SYNC                45
#define SDL_ESMG1_MCANB_TS_ERR                  46
#define SDL_ESMG1_MCANB_UERR                    47
#define SDL_ESMG1_MCANB_SERR                    48
#define SDL_ESMG1_MCANA_TS_ERR                  49
#define SDL_ESMG1_MCANA_UERR                    50
#define SDL_ESMG1_MCANA_SERR                    51
#define SDL_ESMG1_SPIB_UERR                     52
#define SDL_ESMG1_SPIA_UERR                     53
#define SDL_ESMG1_SPIB_SERR                     54
#define SDL_ESMG1_SPIA_SERR                     55
#define SDL_ESMG1_CCCB_ERR                      56
#define SDL_ESMG1_CCCA_ERR                      57
#define SDL_ESMG1_DCCD_ERR                      58
#define SDL_ESMG1_DCCC_ERR                      59
#define SDL_ESMG1_DCCB_ERR                      60
#define SDL_ESMG1_DCCA_ERR                      61
#define SDL_ESMG1_CRC_INT_REQ                   62
#define SDL_ESMG1_ANA_LIMP_MODE_SYNC            63
#define SDL_ESMG1_MSS_AGG_SEC_BUS_SAFETY        66
#define SDL_ESMG1_CPSW_UERR                     67
#define SDL_ESMG1_CPSW_SERR                     68
#define SDL_ESMG1_MPU_HSM_SLV_PROT_ERR          69
#define SDL_ESMG1_MPU_CR5B_SLV_PROT_ERR         70
#define SDL_ESMG1_MPU_CR5A_SLV_PROT_ERR         71
#define SDL_ESMG1_MPU_QSPI_PROT_ERR             72
#define SDL_ESMG1_MPU_PCRA_PROT_ERR             73
#define SDL_ESMG1_MPU_DTHE_PROT_ERR             74
#define SDL_ESMG1_MPU_HSM_SLV_ADDR_ERR          75
#define SDL_ESMG1_MPU_CR5B_SLV_ADDR_ERR         76
#define SDL_ESMG1_MPU_CR5A_SLV_ADDR_ERR         77
#define SDL_ESMG1_MPU_QSPI_ADDR_ERR             78
#define SDL_ESMG1_MPU_PCRA_ADDR_ERR             79
#define SDL_ESMG1_MPU_DTHE_ADDR_ERR             80
#define SDL_ESMG1_MPU_L2_BANKB_PROT_ERR         81
#define SDL_ESMG1_MPU_L2_BANKB_ADDR_ERR         82
#define SDL_ESMG1_MPU_L2_BANKA_PROT_ERR         83
#define SDL_ESMG1_MPU_L2_BANKA_ADDR_ERR         84
#define SDL_ESMG1_MPU_MBOX_PROT_ERR             85
#define SDL_ESMG1_MPU_MBOX_ADDR_ERR             86

#define SDL_ESMG1_BUS_SAFETY_CR5A_SLV           91
#define SDL_ESMG1_BUS_SAFETY_CR5B_SLV           92
#define SDL_ESMG1_BUS_SAFETY_DAP_RS232          93
#define SDL_ESMG1_BUS_SAFETY_HSM_MST            94
#define SDL_ESMG1_BUS_SAFETY_CCPSW              95
#define SDL_ESMG1_BUS_SAFETY_MSS_TPTCA0_RD      96
#define SDL_ESMG1_BUS_SAFETY_MSS_TPTCA1_RD      97
#define SDL_ESMG1_BUS_SAFETY_MSS_TPTCB0_RD      98
#define SDL_ESMG1_BUS_SAFETY_MSS_TPTCA0_WR      99
#define SDL_ESMG1_BUS_SAFETY_MSS_TPTCA1_WR      100
#define SDL_ESMG1_BUS_SAFETY_MSS_TPTCB0_WR      101
#define SDL_ESMG1_BUS_SAFETY_HSM_TPTCA0_RD      102
#define SDL_ESMG1_BUS_SAFETY_HSM_TPTCA0_WR      103
#define SDL_ESMG1_BUS_SAFETY_HSM_TPTCA1_RD      104
#define SDL_ESMG1_BUS_SAFETY_HSM_TPTCA1_WR      105
#define SDL_ESMG1_BUS_SAFETY_QSPI               106
#define SDL_ESMG1_BUS_SAFETY_MCRC               107
#define SDL_ESMG1_BUS_SAFETY_HSM_SLAVE          110
#define SDL_ESMG1_BUS_SAFETY_DTHE               111
#define SDL_ESMG1_BUS_SAFETY_MSS_MBOX           114
#define SDL_ESMG1_BUS_SAFETY_MSS_RET_RAM        115
#define SDL_ESMG1_BUS_SAFETY_GPADC              116
#define SDL_ESMG1_BUS_SAFETY_DMM_MST            117
#define SDL_ESMG1_BUS_SAFETY_DMM_SLAVE          118
#define SDL_ESMG1_BUS_SAFETY_MDO                119
#define SDL_ESMG1_BUS_SAFETY_SCRP               120
#define SDL_ESMG1_BUS_SAFETY_CR5A_AHB           121
#define SDL_ESMG1_BUS_SAFETY_CR5B_AHB           122
#define SDL_ESMG1_ANA_WU_AND_CLK_STATUS_ERR     123

/* List of intr sources for receiver: esm group 2 */
#define  SDL_ESMG2_HSM_ESM_HI_INT               0
#define  SDL_ESMG2_DSS_ESM_HI_INT               1
#define  SDL_ESMG2_CCMR5_COMPARE                2
#define  SDL_ESMG2_ATCM0_PARITY_ERR             3
#define  SDL_ESMG2_B0TCM0_PARITY_ERR            4
#define  SDL_ESMG2_B1TCM0_PARITY_ERR            5
#define  SDL_ESMG2_ATCM1_PARITY_ERR             6
#define  SDL_ESMG2_B0TCM1_PARITY_ERR            7
#define  SDL_ESMG2_B1TCM1_PARITY_ERR            8
#define  SDL_ESMG2_MSS_CR5B_LIVELOCK            9
#define  SDL_ESMG2_MSS_CR5A_LIVELOCK            10
#define  SDL_ESMG2_WDT_NMI_REQ                  11
#define  SDL_ESMG2_VIM_LOCK_ERR                 12
#define  SDL_ESMG2_MSS_L2BANKB_ERR              13
#define  SDL_ESMG2_MSS_L2BANKA_ERR              14
#define  SDL_ESMG2_ECCAGGB_UERR                 15
#define  SDL_ESMG2_ECCAGGA_UERR                 16
#define  SDL_ESMG2_BUS_SAFETY_PCRB              17
#define  SDL_ESMG2_BUS_SAFETY_PCRA              18
#define  SDL_ESMG2_BUS_SAFETY_MSS_L2BANKB       19
#define  SDL_ESMG2_BUS_SAFETY_MSS_L2BANKA       20
#define  SDL_ESMG2_BUS_SAFETY_CR5B_WR           21
#define  SDL_ESMG2_BUS_SAFETY_CR5A_WR           22
#define  SDL_ESMG2_BUS_SAFETY_CR5B_RD           23
#define  SDL_ESMG2_BUS_SAFETY_CR5A_RD           24
#define  SDL_ESMG2_ANA_WU_AND_CLK_STATUS_ERR    25

/* List of intr sources for receiver: esm group 3 */
#define  SDL_ESMG3_EFC_AUTO_ERR_SYNC            1
#define  SDL_ESMG3_B0TCM0_UERR                  3
#define  SDL_ESMG3_B1TCM0_UERR                  5
#define  SDL_ESMG3_ATCM0_UERR                   7
#define  SDL_ESMG3_B0TCM1_UERR                  9
#define  SDL_ESMG3_B1TCM1_UERR                  11
#define  SDL_ESMG3_ATCM1_UERR                   13
#define  SDL_ESMG3_ITAG0_UERR                   15
#define  SDL_ESMG3_IDATA0_UERR                  17
#define  SDL_ESMG3_DTAG0_UERR                   19
#define  SDL_ESMG3_DDATA0_UERR                  21
#define  SDL_ESMG3_ITAG1_UERR                   23
#define  SDL_ESMG3_IDATA1_UERR                  25
#define  SDL_ESMG3_DTAG1_UERR                   27
#define  SDL_ESMG3_DDATA1_UERR                  29


#ifdef __cplusplus
}
#endif
#endif /* SDLR_INTR_ESM_MSS_H_*/
