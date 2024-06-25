/********************************************************************
*
* MSS INTR Map Header file
*
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
*/
#ifndef SDLR_INTR_MSS_H_
#define SDLR_INTR_MSS_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* List of intr sources for receiver: MSS */
#define SDL_MSS_INTR_MSS_ESM_HI                       0           /* ESM High Priority Interrupt */
#define SDL_MSS_INTR_MSS_ESM_LO                       1           /* ESM Low Priority Interrupt */
#define SDL_MSS_INTR_MSS_R5SS_STC_DONE                2           /* R5SS STC Complete */
#define SDL_MSS_INTR_MSS_RTIA_INT0                    3           /* RTIA compare interrupt */
#define SDL_MSS_INTR_MSS_RTIA_INT1                    4           /* RTIA compare interrupt */
#define SDL_MSS_INTR_MSS_RTIA_INT2                    5           /* RTIA compare interrupt */
#define SDL_MSS_INTR_MSS_RTIA_INT3                    6           /* RTIA compare interrupt */
#define SDL_MSS_INTR_MSS_RTIA_OVERFLOW_INT0           7           /* RTIA overflow interrupt */
#define SDL_MSS_INTR_MSS_RTIA_OVERFLOW_INT1           8           /* RTIA overflow interrupt */
#define SDL_MSS_INTR_MSS_RTIB_INT0                    9           /* RTIB compare interrupt */
#define SDL_MSS_INTR_MSS_RTIB_INT1                    10          /* RTIB compare interrupt */
#define SDL_MSS_INTR_MSS_RTIB_OVERFLOW_INT0           11          /* RTIB overflow interrupt */
#define SDL_MSS_INTR_MSS_RTIB_OVERFLOW_INT1           12          /* RTIB overflow interrupt */
#define SDL_MSS_INTR_MSS_RTIC_INT0                    13          /* RTIC compare interrupt */
#define SDL_MSS_INTR_MSS_RTIC_INT1                    14          /* RTIC compare interrupt */
#define SDL_MSS_INTR_MSS_RTIC_OVERFLOW_INT0           15          /* RTIC overflow interrupt */
#define SDL_MSS_INTR_MSS_RTIC_OVERFLOW_INT1           16          /* RTIC overflow interrupt */
#define SDL_MSS_INTR_MSS_WDT_INT0                     17          /* WDT compare interrupt */
#define SDL_MSS_INTR_MSS_WDT_INT1                     18          /* WDT compare interrupt */
#define SDL_MSS_INTR_MSS_WDT_INT2                     19          /* WDT compare interrupt */
#define SDL_MSS_INTR_MSS_WDT_INT3                     20          /* WDT compare interrupt */
#define SDL_MSS_INTR_MSS_WDT_OVERFLOW_INT0            21          /* WDT over flow interrupt */
#define SDL_MSS_INTR_MSS_WDT_OVERFLOW_INT1            22          /* WDT over flow interrupt */
#define SDL_MSS_INTR_MSS_WDT_TB_INT                   23          /* WDT time base interrupt */
#define SDL_MSS_INTR_MSS_MCRC_INT                     24          /* Interrupt from MCRC */
#define SDL_MSS_INTR_MSS_DCCA_INT                     25          /* MSS_DCCA Clock compare done interrupt */
#define SDL_MSS_INTR_MSS_DCCB_INT                     26          /* MSS_DCCB  Clock compare done interrupt */
#define SDL_MSS_INTR_MSS_DCCC_INT                     27          /* MSS_DCCC  Clock compare done interrupt */
#define SDL_MSS_INTR_MSS_DCCD_INT                     28          /* MSS_DCCD  Clock compare done interrupt */
#define SDL_MSS_INTR_MSS_CCCA_INT                     29          /* MSS_CCCA dual clock compare done interrupt */
#define SDL_MSS_INTR_MSS_CCCB_INT                     30          /* MSS_CCCB dual clock compare done interrupt */
#define SDL_MSS_INTR_MSS_SPIA_INT0                    31          /* MSS_SPIA level 0 interrupt */
#define SDL_MSS_INTR_MSS_SPIA_INT1                    32          /* MSS_SPIA level 1 interrupt */
#define SDL_MSS_INTR_MSS_SPIB_INT0                    33          /* MSS_SPIB level 0 interrupt */
#define SDL_MSS_INTR_MSS_SPIB_INT1                    34          /* MSS_SPIB level 1 interrupt */
#define SDL_MSS_INTR_MSS_QSPI_INT                     35          /* MSS_QSPI interrupt */
#define SDL_MSS_INTR_MSS_GIO_INT0                     36          /* MSS_GIO high-level Interrupt  */
#define SDL_MSS_INTR_MSS_GIO_INT1                     37          /* MSS_GIO low-level interrupt  */
#define SDL_MSS_INTR_MSS_ETPWMA_INT0                  38          /* MSS_ETPWMA Interrupt  0 which is a output to pad  */
#define SDL_MSS_INTR_MSS_ETPWMA_INT1                  39          /* MSS_ETPWMA Interrupt  1 which is a output to pad  */
#define SDL_MSS_INTR_MSS_ETPWMB_INT0                  40          /* MSS_ETPWMB Interrupt  0 which is a output to pad  */
#define SDL_MSS_INTR_MSS_ETPWMB_INT1                  41          /* MSS_ETPWMB Interrupt  1 which is a output to pad  */
#define SDL_MSS_INTR_MSS_ETPWMC_INT0                  42          /* MSS_ETPWMC Interrupt  0 which is a output to pad  */
#define SDL_MSS_INTR_MSS_ETPWMC_INT1                  43          /* MSS_ETPWMC Interrupt  1 which is a output to pad  */
#define SDL_MSS_INTR_MSS_MCANA_INT0                   44          /* MSS_MCANA first interrupt */
#define SDL_MSS_INTR_MSS_MCANA_INT1                   45          /* MSS_MCANA second interrupt */
#define SDL_MSS_INTR_MSS_MCANA_FE_INT1                46          /* MSS_MCANA message filter interrupt1 */
#define SDL_MSS_INTR_MSS_MCANA_FE_INT2                47          /* MSS_MCANA message filter interrupt2 */
#define SDL_MSS_INTR_MSS_MCANB_INT0                   48          /* MSS_MCANB first interrupt */
#define SDL_MSS_INTR_MSS_MCANB_INT1                   49          /* MSS_MCANB second interrupt */
#define SDL_MSS_INTR_MSS_MCANB_FE_INT1                50          /* MSS_MCANB message filter interrupt1 */
#define SDL_MSS_INTR_MSS_MCANB_FE_INT2                51          /* MSS_MCANB message filter interrupt2 */
#define SDL_MSS_INTR_MSS_I2C_INT                      52          /* MSS_I2C interrupt */
#define SDL_MSS_INTR_MSS_SCIA_INT0                    53          /* MSS_SCIA level0 input */
#define SDL_MSS_INTR_MSS_SCIA_INT1                    54          /* MSS_SCIA level1 input */
#define SDL_MSS_INTR_MSS_SCIB_INT0                    55          /* MSS_SCIB level0 input */
#define SDL_MSS_INTR_MSS_SCIB_INT1                    56          /* MSS_SCIB level1 input */
#define SDL_MSS_INTR_TOP_PBIST_DONE_INT               57          /* TOP_PBIST done interrupt */
#define SDL_MSS_INTR_MSS_GIO_PAD_INT0                 58          /* Interrupt Triger from GIO[0][0] */
#define SDL_MSS_INTR_MSS_GIO_PAD_INT1                 59          /* Interrupt Triger from GIO[0][1] */
#define SDL_MSS_INTR_MSS_GIO_PAD_INT2                 60          /* Interrupt Triger from GIO[0][2] */
#define SDL_MSS_INTR_MSS_GIO_PAD_INT3                 61          /* Interrupt Triger from GIO[0][3] */
#define SDL_MSS_INTR_MSS_MCANA_FE_INT3                62          /* MSS_MCANA message filter interrupt3 */
#define SDL_MSS_INTR_MSS_MCANA_FE_INT4                63          /* MSS_MCANA message filter interrupt4 */
#define SDL_MSS_INTR_MSS_MCANA_FE_INT5                64          /* MSS_MCANA message filter interrupt5 */
#define SDL_MSS_INTR_MSS_MCANA_FE_INT6                65          /* MSS_MCANA message filter interrupt6 */
#define SDL_MSS_INTR_MSS_MCANA_FE_INT7                66          /* MSS_MCANA message filter interrupt7 */
#define SDL_MSS_INTR_MSS_TPCC_A_INTAGG                67          /* MSS_TPCC_A Aggregated Functional Interrupt */
#define SDL_MSS_INTR_MSS_TPCC_B_INTAGG                68          /* MSS_TPCC_B Aggregated Functional Interrupt */
#define SDL_MSS_INTR_MSS_TPCC_A_ERRAGG                69          /* MSS_TPCC_A Aggregated Error Interrupt */
#define SDL_MSS_INTR_MSS_TPCC_B_ERRAGG                70          /* MSS_TPCC_B Aggregated Error Interrupt */
#define SDL_MSS_INTR_MSS_GPADC_IFM_DONE               71          /* MSS_GPADC ifm done interrupt */
#define SDL_MSS_INTR_MSS_CPSW_TH_TRSH_INT             72          /* MSS CPSW T-host threshold interrupt */
#define SDL_MSS_INTR_MSS_CPSW_TH_INT                  73          /* MSS CPSW T-host interrupt */
#define SDL_MSS_INTR_MSS_CPSW_FH_INT                  74          /* MSS CPSW F-host interrupt */
#define SDL_MSS_INTR_MSS_CPSW_MISC_INT                75          /* MSS CPSW interrupt */
#define SDL_MSS_INTR_RESERVED76                       76          /* RESERVED */
#define SDL_MSS_INTR_MSS_CR5A_MBOX_READ_REQ           77          /* Aggregated interrupt to MSS CR5A from other processor asking it to read */
#define SDL_MSS_INTR_MSS_CR5A_MBOX_READ_ACK           78          /* Aggregated interrupt to MSS CR5A from other processor saying the reading from their mailbox is done. */
#define SDL_MSS_INTR_MSS_CR5B_MBOX_READ_REQ           79          /* Aggregated interrupt to MSS CR5B from other processor asking it to read */
#define SDL_MSS_INTR_MSS_CR5B_MBOX_READ_ACK           80          /* Aggregated interrupt to MSS CR5B from other processor saying the reading from their mailbox is done. */
#define SDL_MSS_INTR_RESERVED81                       81          /* RESERVED */
#define SDL_MSS_INTR_TOP_DEBUGSS_TXDATA_AVAIL         82          /* Interrupt from TOP_DEBUGSS */
#define SDL_MSS_INTR_MSS_CR5A_PMU_INT                 83          /* Pmu Interrupt from MSS_CR5A */
#define SDL_MSS_INTR_MSS_CR5B_PMU_INT                 84          /* Pmu Interrupt from MSS_CR5B */
#define SDL_MSS_INTR_MSS_CR5A_FPU_INT                 85          /* Floating point expection from MSS_CR5A */
#define SDL_MSS_INTR_MSS_CR5B_FPU_INT                 86          /* Floating point expection from MSS_CR5B */
#define SDL_MSS_INTR_RESERVED87                       87          /* RESERVED */
#define SDL_MSS_INTR_CR5A_CTI_IRQ                     88          /* IRQ request from CTI module from CR5A */
#define SDL_MSS_INTR_CR5B_CTI_IRQ                     89          /* IRQ request from CTI module from CR5B */
#define SDL_MSS_INTR_RESERVED90                       90          /* RESERVED */
#define SDL_MSS_INTR_MSS_SW_INT0                      91          /* Software Interrupt from MSS_CTRL */
#define SDL_MSS_INTR_MSS_SW_INT1                      92          /* Software Interrupt from MSS_CTRL */
#define SDL_MSS_INTR_MSS_SW_INT2                      93          /* Software Interrupt from MSS_CTRL */
#define SDL_MSS_INTR_MSS_SW_INT3                      94          /* Software Interrupt from MSS_CTRL */
#define SDL_MSS_INTR_MSS_SW_INT4                      95          /* Software Interrupt from MSS_CTRL */
#define SDL_MSS_INTR_RESERVED96                       96          /* RESERVED */
#define SDL_MSS_INTR_MSS_PERIPH_ACCESS_ERRARG         97          /* Aggregation of all access-errros from mpu and control spaces */
#define SDL_MSS_INTR_MSS_CR5A_AHB_WR_ERR              98          /* MSS_CR5A ahb brige getting write response as a error  */
#define SDL_MSS_INTR_MSS_CR5B_AHB_WR_ERR              99          /* MSS_CR5B ahb brige getting write response as a error  */
#define SDL_MSS_INTR_DTHE_SHA_S_INT                   100         /* Interrupt from HSM_SHA */
#define SDL_MSS_INTR_DTHE_SHA_P_INT                   101         /* Interrupt from HSM_SHA */
#define SDL_MSS_INTR_DTHE_TRNG_INT                    102         /* Interrupt from HSM_TRNG */
#define SDL_MSS_INTR_DTHE_PKAE_INT                    103         /* Interrupt from HSM_PKAE */
#define SDL_MSS_INTR_DTHE_AES_S_INT                   104         /* Interrupt from HSM_AES */
#define SDL_MSS_INTR_DTHE_AES_P_INT                   105         /* Interrupt from HSM_AES */
#define SDL_MSS_INTR_RESERVED106                      106         /* RESERVED */
#define SDL_MSS_INTR_MSS_RTIB_INT2                    107         /* RTIB compare interrupt */
#define SDL_MSS_INTR_MSS_RTIB_INT3                    108         /* RTIB compare interrupt */
#define SDL_MSS_INTR_MSS_RTIC_INT2                    109         /* RTIC compare interrupt */
#define SDL_MSS_INTR_MSS_RTIC_INT3                    110         /* RTIC compare interrupt */
#define SDL_MSS_INTR_RESERVED111                      111         /* RESERVED */
#define SDL_MSS_INTR_RESERVED112                      112         /* RESERVED */
#define SDL_MSS_INTR_MSS_MCANB_FE_INT3                113         /* MSS_MCANB message filter interrupt3 */
#define SDL_MSS_INTR_MSS_MCANB_FE_INT4                114         /* MSS_MCANB message filter interrupt4 */
#define SDL_MSS_INTR_MSS_MCANB_FE_INT5                115         /* MSS_MCANB message filter interrupt5 */
#define SDL_MSS_INTR_MSS_MCANB_FE_INT6                116         /* MSS_MCANB message filter interrupt6 */
#define SDL_MSS_INTR_MSS_MCANB_FE_INT7                117         /* MSS_MCANB message filter interrupt7 */
#define SDL_MSS_INTR_RESERVED118                      118         /* RESERVED */
#define SDL_MSS_INTR_RESERVED119                      119         /* RESERVED */
#define SDL_MSS_INTR_DSS_TPCC_A_INTAGG                120         /* DSS_TPCC_A Aggregated Functional Interrupt */
#define SDL_MSS_INTR_DSS_TPCC_A_ERRAGG                121         /* DSS_TPCC_A Agregated Error Interrupt */
#define SDL_MSS_INTR_DSS_TPCC_B_INTAGG                122         /* DSS_TPCC_B_Aggregated Functional Interrupt */
#define SDL_MSS_INTR_DSS_TPCC_B_ERRAGG                123         /* DSS_TPCC_B Agregated Error Interrupt */
#define SDL_MSS_INTR_DSS_TPCC_C_INTAGG                124         /* DSS_TPCC_C Aggregated Functional Interrupt */
#define SDL_MSS_INTR_DSS_TPCC_C_ERRAGG                125         /* DSS_TPCC_C Agregated Error Interrupt */
#define SDL_MSS_INTR_DSS_PERIPH_ACCESS_ERRAGG         126         /* Aggregation of access-errros from DSS peripherals. See Error access Response Section for more details */
#define SDL_MSS_INTR_TOP_MDO_INFRA_INT                127         /* TOP MDO INFRA Interrupt */
#define SDL_MSS_INTR_RESERVED128                      128         /* RESERVED */
#define SDL_MSS_INTR_DSS_DSP_PBIST_CTRL_DONE          129         /* DSS DSP PBIST Controller Done Interrupt */
#define SDL_MSS_INTR_DSS_SW_INT0                      130         /* SW interrupt generated by writing 0x1 to register  DSS_CTRL.DSS_SW_INT[0] */
#define SDL_MSS_INTR_DSS_SW_INT1                      131         /* SW interrupt generated by writing 0x1 to register  DSS_CTRL.DSS_SW_INT[1] */
#define SDL_MSS_INTR_DSS_SW_INT2                      132         /* SW interrupt generated by writing 0x1 to register  DSS_CTRL.DSS_SW_INT[2] */
#define SDL_MSS_INTR_DSS_SW_INT3                      133         /* SW interrupt generated by writing 0x1 to register  DSS_CTRL.DSS_SW_INT[3] */
#define SDL_MSS_INTR_RESERVED134                      134         /* RESERVED */
#define SDL_MSS_INTR_RESERVED135                      135         /* RESERVED */
#define SDL_MSS_INTR_DSS_MCRC_INT                     136         /* DSS MCRC Interrupt */
#define SDL_MSS_INTR_DSS_DSP_STC_DONE                 137         /* DSS DSP STC Done Interrupt */
#define SDL_MSS_INTR_DSS_DSP_PBIST_DONE               138         /* DSS DSP PBIST Done Interrupt */
#define SDL_MSS_INTR_DSS_SCIA_INT0                    139         /* DSS SCIA Interrupt 0 */
#define SDL_MSS_INTR_DSS_SCIA_INT1                    140         /* DSS SCIA Interrupt 1 */
#define SDL_MSS_INTR_RESERVED141                      141         /* RESERVED */
#define SDL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE         142         /* Raw ADC capture complete interrupt from DFE-DSP bridge */
#define SDL_MSS_INTR_DSS_CBUFF_INT                    143         /* DSS CBUFF Interrupt */
#define SDL_MSS_INTR_DSS_CBUFF_INT_ERR                144         /* DSSCBUFF Error Interrupt  */
#define SDL_MSS_INTR_DSS_HWA_LOOP_INTR1               145         /* DSS_HWA Loop complete interrupt 1*/
#define SDL_MSS_INTR_DSS_HWA_PARAM_DONE_INTR1         146         /* DSS_HWA Param done interrupt 1 */
#define SDL_MSS_INTR_RESERVED147                      147         /* RESERVED */
#define SDL_MSS_INTR_RESERVED148                      148         /* RESERVED */
#define SDL_MSS_INTR_RESERVED149                      149         /* RESERVED */
#define SDL_MSS_INTR_DFE_FRAME_START_TO_DSS           150         /* Frame start interrupt from BSS which is masked with “BSS_GPCFG:: DSS_FS_INTR_MASK” register */
#define SDL_MSS_INTR_RSS_TPCC_A_INTAGG                151         /* RSS_TPCC_A Aggregated Functional Interrupt */
#define SDL_MSS_INTR_RSS_TPCC_A_ERRAGG                152         /* RSS_TPCC_A Aggregated Error Interrupt */
#define SDL_MSS_INTR_ADC_VALID_FALL_EDGE              153         /* Interrupt is trigger during Falling edge of ADC valid  */
#define SDL_MSS_INTR_DFE_FRAME_START_TO_MSS           154         /* Frame start interrupt from BSS which is masked with “BSS_GPCFG:: MSS_FS_INTR_MASK” register */
#define SDL_MSS_INTR_DFE_CHIRP_CYCLE_START            155         /* Chirp cycle start interrupt from dfe */
#define SDL_MSS_INTR_DFE_CHIRP_CYCLE_END              156         /* Chirp cycle end interrupt from dfe */
#define SDL_MSS_INTR_DFE_END_OF_FRAME                 157         /* End of Frame interrupt from Dfe */
#define SDL_MSS_INTR_RSS_FRC_FRAME_START              158         /* Frame start interrupt from FRC  */
#define SDL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE_DITH    159         /* ADC capture complete interrupt from DFE-DSP bridge after dithering  */
#define SDL_MSS_INTR_DSS_HWA_LOOP_INTR2               160         /* DSS_HWA  Loop complete interrupt2 */
#define SDL_MSS_INTR_DSS_HWA_PARAM_DONE_INTR2         161         /* DSS_HWA Param done interrupt2 */
#define SDL_MSS_INTR_DSS_WDT_TB_INT                   162         /* DSS WDT Time Base Interrupt */
#define SDL_MSS_INTR_DSS_HWA_LOCAL_RAM_ERR            163         /* DSS_HWA Local RAM access error */
#define SDL_MSS_INTR_DSS DCCA_INT                     164         /* DSS DCCA Interrupt */
#define SDL_MSS_INTR_DSS_DCCB_INT                     165         /* DSS DCCB Interrupt */
#define SDL_MSS_INTR_DSS_RTIA_OVERFLOW_0              166         /* DSS_RTIA Overflow 0 */
#define SDL_MSS_INTR_DSS_RTIA_OVERFLOW_1              167         /* DSS_RTIA Overflow 1 */
#define SDL_MSS_INTR_DSS_RTIA_0                       168         /* DSS_RTIA Interrupt 0 */
#define SDL_MSS_INTR_DSS_RTIA_1                       169         /* DSS_RTIA Interrupt 1 */
#define SDL_MSS_INTR_DSS_RTIA_2                       170         /* DSS_RTIA Interrupt 2 */
#define SDL_MSS_INTR_DSS_RTIA_3                       171         /* DSS_RTIA Interrupt 3 */
#define SDL_MSS_INTR_DSS_RTIB_OVERFLOW_0              172         /* DSS_RTIB Overflow 0 */
#define SDL_MSS_INTR_DSS_RTIB_OVERFLOW_1              173         /* DSS_RTIB Overflow 1 */
#define SDL_MSS_INTR_DSS_RTIB_0                       174         /* DSS_RTIB Interrupt 0 */
#define SDL_MSS_INTR_DSS_RTIB_1                       175         /* DSS_RTIB Interrupt 1 */
#define SDL_MSS_INTR_DSS_RTIB_2                       176         /* DSS_RTIB Interrupt 2 */
#define SDL_MSS_INTR_DSS_RTIB_3                       177         /* DSS_RTIB Interrupt 3 */
#define SDL_MSS_INTR_DSS_WDT_OVERFLOW_0               178         /* DSS_WDT Overflow 0 */
#define SDL_MSS_INTR_DSS_WDT_OVERFLOW_1               179         /* DSS_WDT Overflow 1 */
#define SDL_MSS_INTR_DSS_WDT_0                        180         /* DSS_WDT Interrupt 0 */
#define SDL_MSS_INTR_DSS_WDT_1                        181         /* DSS_WDT Interrupt 1 */
#define SDL_MSS_INTR_DSS_WDT_2                        182         /* DSS_WDT Interrupt 2 */
#define SDL_MSS_INTR_DSS_WDT_3                        183         /* DSS_WDT Interrupt 3 */
#define SDL_MSS_INTR_RSS_CSI2A_INT                    184         /* RSS CSI2A Interrupt  */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_INT                185         /* RSS CSI2A End of Line Interrupt  */
#define SDL_MSS_INTR_RSS_CSI2A_SOF_INT0               186         /* RSS CSI2A Start of Frame Interrupt 0 */
#define SDL_MSS_INTR_RSS_CSI2A_SOF_INT1               187         /* RSS CSI2A Start of Frame Interrupt 1 */
#define SDL_MSS_INTR_DSS_SW_INT_RSS_PROC_CTRL0        188         /* Interrupt from RSS_PROC_CTRL */
#define SDL_MSS_INTR_DSS_SW_INT_RSS_PROC_CTRL1        189         /* Interrupt from RSS_PROC_CTRL */
#define SDL_MSS_INTR_DSS_SW_INT_RSS_PROC_CTRL2        190         /* Interrupt from RSS_PROC_CTRL */
#define SDL_MSS_INTR_DSS_SW_INT_RSS_PROC_CTRL3        191         /* Interrupt from RSS_PROC_CTRL */
#define SDL_MSS_INTR_RSS_DATA_CAPTURE_ENABLE_FALL     192         /* Interrupt is triggered data_capute enable fall from DFE-DSP bridge (toggles after completion of every chirp) */
#define SDL_MSS_INTR_RESERVED193                      193         /* RESERVED */
#define SDL_MSS_INTR_RESERVED194                      194         /* RESERVED */
#define SDL_MSS_INTR_RESERVED195                      195         /* RESERVED */
#define SDL_MSS_INTR_DSS_DSP_MBOX_READ_REQ            196         /* DSS DSP Mailbox Read Request Level */
#define SDL_MSS_INTR_DSS_DSP_MBOX_READ_ACK            197         /* DSS DSP Mailbox Read Acknowledge */
#define SDL_MSS_INTR_RESERVED198                      198         /* RESERVED */
#define SDL_MSS_INTR_RESERVED199                      199         /* RESERVED */
#define SDL_MSS_INTR_MSS_DMM_A_INT0                   200         /* Interrupt from MSS_DMM_A */
#define SDL_MSS_INTR_MSS_DMM_A_INT1                   201         /* Interrupt from MSS_DMM_A */
#define SDL_MSS_INTR_MSS_DMM_B_INT0                   202         /* Interrupt from MSS_DMM_B */
#define SDL_MSS_INTR_MSS_DMM_B_INT1                   203         /* Interrupt from MSS_DMM_B */
#define SDL_MSS_INTR_RESERVED204                      204         /* RESERVED */
#define SDL_MSS_INTR_RESERVED205                      205         /* RESERVED */
#define SDL_MSS_INTR_RESERVED206                      206         /* RESERVED */
#define SDL_MSS_INTR_RSS_EXT_SYNC_OUT                 207         /* rss_int_map[29]  */
#define SDL_MSS_INTR_RSS_EXT_SYNC_IN                  208         /* rss_int_map[30]  */
#define SDL_MSS_INTR_RESERVED209                      209         /* RESERVED */
#define SDL_MSS_INTR_RSS_CSI2A_EOF_INT                210         /* RSS CSI2A End of Frame Interrupt */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_CNTX0_INT          211         /* RSS_CSI2A End of Line Interrupt for Context 0 */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_CNTX1_INT          212         /* RSS_CSI2A End of Line Interrupt for Context 1 */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_CNTX2_INT          213         /* RSS_CSI2A End of Line Interrupt for Context 2 */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_CNTX3_INT          214         /* RSS_CSI2A End of Line Interrupt for Context 3 */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_CNTX4_INT          215         /* RSS_CSI2A End of Line Interrupt for Context 4 */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_CNTX5_INT          216         /* RSS_CSI2A End of Line Interrupt for Context 5 */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_CNTX6_INT          217         /* RSS_CSI2A End of Line Interrupt for Context 6 */
#define SDL_MSS_INTR_RSS_CSI2A_EOL_CNTX7_INT          218         /* RSS_CSI2A End of Line Interrupt for Context 7 */
#define SDL_MSS_INTR_RSS_BSS_CR4_STC_COMPLETE         219         /* rss_int_map[49] = bss_to_mss_irq[7] - cr4_stc_test_complete */
#define SDL_MSS_INTR_RSS_FRC_EVNT_GEN_1               220         /* rss_int_map[51]  */
#define SDL_MSS_INTR_RSS_FRC_EVNT_GEN_2               221         /* rss_int_map[52]  */
#define SDL_MSS_INTR_RSS_FRC_EVNT_GEN_3               222         /* rss_int_map[53]  */
#define SDL_MSS_INTR_RSS_FRC_EVNT_GEN_4               223         /* rss_int_map[54]  */
#define SDL_MSS_INTR_RSS_FRC_TS_1_TRIG                224         /* rss_int_map[55]  */
#define SDL_MSS_INTR_RSS_FRC_TS_2_TRIG                225         /* rss_int_map[56]  */
#define SDL_MSS_INTR_RSS_FRC_TS_3_TRIG                226         /* rss_int_map[57]  */
#define SDL_MSS_INTR_RSS_FRC_TS_4_TRIG                227         /* rss_int_map[58]  */
#define SDL_MSS_INTR_RESERVED228                      228         /* RESERVED */
#define SDL_MSS_INTR_MSS_GIO_PAD_INT4                 229         /* Interrupt Triger from MSS GIO[1][0] */
#define SDL_MSS_INTR_MSS_GIO_PAD_INT5                 230         /* Interrupt Triger from MSS GIO[1][1] */
#define SDL_MSS_INTR_MSS_GIO_PAD_INT6                 231         /* Interrupt Triger from MSS GIO[1][2] */
#define SDL_MSS_INTR_MSS_GIO_PAD_INT7                 232         /* Interrupt Triger from MSS GIO[1][3] */
#define SDL_MSS_INTR_TOP_AURORATX_INT                 233         /* TOP_AURORATX Interrupt */
#define SDL_MSS_INTR_TOP_AURORATX_ERR                 234         /* TOP_AURORATX Error Interrupt */
#define SDL_MSS_INTR_TOP_MDO_INFRA_INT_ALIAS          235         /* TOP_MDO_INFRA Interrupt */
//#define SDL_MSS_INTR_RESERVED                       236 to 255  /* RESERVED */

#ifdef __cplusplus
}
#endif
#endif /* SDLR_INTR_MSS_H_*/
