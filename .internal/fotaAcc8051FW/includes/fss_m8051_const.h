/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 */


/*===========================================================================
  ===========================================================================
  ==      File:    fss_m8051.h
  ==      Author:  Dave Brier
  ==
  ==      Purpose:  This file contains definitions for the various access
  ==                locations in the OptiFlash design that are accessed by
  ==                the M8051 processor. The attempt is to support two
  ==                compilers, the Keil compiler run in Windows and
  ==                the SDCC compiler run in Linux. There are different
  ==                syntax protocols between the two compilers and this
  ==                file will capture those.
  ==
  ==       Version:
  ==        0.1   03-16-22   dbrier
  ==
  ==========================================================================*/

#ifndef __FSS_M8051_CONST_H__
#define __FSS_M8051_CONST_H__

//FOTA EFSR Registers
__sfr __at (0x86)  ESFR_FOTA_CTRL    ;
__sfr __at (0x8E)  ESFR_FOTA_CMPL_STAT    ;
__sfr __at (0x8F)  ESFR_FOTA_ERR_STAT    ;
__sfr __at (0x9C)  ESFR_FOTA_GP0_VAL0    ;
__sfr __at (0x9D)  ESFR_FOTA_GP0_VAL1    ;
__sfr __at (0x9E)  ESFR_FOTA_GP0_VAL2    ;
__sfr __at (0x9F)  ESFR_FOTA_GP0_VAL3    ;
__sfr __at (0xA1)  ESFR_FOTA_GP1_VAL0    ;
__sfr __at (0xA2)  ESFR_FOTA_GP1_VAL1    ;
__sfr __at (0xA3)  ESFR_FOTA_GP1_VAL2    ;
__sfr __at (0xA4)  ESFR_FOTA_GP1_VAL3    ;
__sfr __at (0xA5)  ESFR_MCU_CFG_SEL    ;
__sfr __at (0xA6)  ESFR_MCU_CFG_CTRL    ;
__sfr __at (0xA7)  ESFR_MCU_CFG_ADDR0    ;
__sfr __at (0xAC)  ESFR_MCU_CFG_ADDR1    ;
__sfr __at (0xAD)  ESFR_MCU_CFG_WR_DAT0    ;
__sfr __at (0xAE)  ESFR_MCU_CFG_WR_DAT1    ;
__sfr __at (0xAF)  ESFR_MCU_CFG_WR_DAT2    ;
__sfr __at (0xB1)  ESFR_MCU_CFG_WR_DAT3    ;
__sfr __at (0xB2)  ESFR_MCU_CFG_STAT    ;
__sfr __at (0xB3)  ESFR_MCU_CFG_RD_DAT0    ;
__sfr __at (0xB4)  ESFR_MCU_CFG_RD_DAT1    ;
__sfr __at (0xB5)  ESFR_MCU_CFG_RD_DAT2    ;
__sfr __at (0xB6)  ESFR_MCU_CFG_RD_DAT3    ;
__sfr __at (0xBE)  ESFR_MCU_DAT_SEL    ;
__sfr __at (0xBF)  ESFR_MCU_DAT_CTRL    ;
__sfr __at (0xC0)  ESFR_MCU_DAT_ADDR0    ;
__sfr __at (0xC1)  ESFR_MCU_DAT_ADDR1    ;
__sfr __at (0xC2)  ESFR_MCU_DAT_ADDR2    ;
__sfr __at (0xC3)  ESFR_MCU_DAT_ADDR3    ;
__sfr __at (0xC4)  ESFR_MCU_DAT_CNT0    ;
__sfr __at (0xC5)  ESFR_MCU_DAT_CNT1    ;
__sfr __at (0xC6)  ESFR_MCU_DAT_WR_DAT0    ;
__sfr __at (0xC7)  ESFR_MCU_DAT_WR_DAT1    ;
__sfr __at (0xC9)  ESFR_MCU_DAT_WR_DAT2    ;
__sfr __at (0xCE)  ESFR_MCU_DAT_WR_DAT3    ;
__sfr __at (0xCF)  ESFR_MCU_DAT_WR_DAT4    ;
__sfr __at (0xD1)  ESFR_MCU_DAT_WR_DAT5    ;
__sfr __at (0xD2)  ESFR_MCU_DAT_WR_DAT6    ;
__sfr __at (0xD3)  ESFR_MCU_DAT_WR_DAT7    ;
__sfr __at (0xD4)  ESFR_MCU_DAT_WR_DAT8    ;
__sfr __at (0xD5)  ESFR_MCU_DAT_WR_DAT9    ;
__sfr __at (0xD6)  ESFR_MCU_DAT_WR_DAT10    ;
__sfr __at (0xD7)  ESFR_MCU_DAT_WR_DAT11    ;
__sfr __at (0xD8)  ESFR_MCU_DAT_WR_DAT12    ;
__sfr __at (0xD9)  ESFR_MCU_DAT_WR_DAT13    ;
__sfr __at (0xDA)  ESFR_MCU_DAT_WR_DAT14    ;
__sfr __at (0xDB)  ESFR_MCU_DAT_WR_DAT15    ;
__sfr __at (0xDC)  ESFR_MCU_DAT_STAT    ;
__sfr __at (0xDD)  ESFR_MCU_DAT_RD_DAT0    ;
__sfr __at (0xDE)  ESFR_MCU_DAT_RD_DAT1    ;
__sfr __at (0xDF)  ESFR_MCU_DAT_RD_DAT2    ;
__sfr __at (0xE1)  ESFR_MCU_DAT_RD_DAT3    ;
__sfr __at (0xE2)  ESFR_MCU_DAT_RD_DAT4    ;
__sfr __at (0xE3)  ESFR_MCU_DAT_RD_DAT5    ;
__sfr __at (0xE4)  ESFR_MCU_DAT_RD_DAT6    ;
__sfr __at (0xE5)  ESFR_MCU_DAT_RD_DAT7    ;
__sfr __at (0xE6)  ESFR_MCU_DAT_RD_DAT8    ;
__sfr __at (0xE7)  ESFR_MCU_DAT_RD_DAT9    ;
__sfr __at (0xE9)  ESFR_MCU_DAT_RD_DAT10    ;
__sfr __at (0xEA)  ESFR_MCU_DAT_RD_DAT11    ;
__sfr __at (0xEB)  ESFR_MCU_DAT_RD_DAT12    ;
__sfr __at (0xEC)  ESFR_MCU_DAT_RD_DAT13    ;
__sfr __at (0xED)  ESFR_MCU_DAT_RD_DAT14    ;
__sfr __at (0xEE)  ESFR_MCU_DAT_RD_DAT15    ;
__sfr __at (0xF5)  ESFR_CPU_DAT_CMD_INFO    ;
__sfr __at (0xFF)  ESFR_CPU_EO  ;

//Timer 2 Values
__sfr __at (0xC8)  T2CON;
__sfr __at (0xCA)  TL2;
__sfr __at (0xCB)  TH2;
__sfr __at (0xCC)  RCAP2L;
__sfr __at (0xCD)  RCAP2H;



//CFG BUS RSEL Values

#define ESFR_OSPI_REGS_RSEL               8

//FOTA EFSR Register Masks

//fota_ctrl ??? what about this reg ????

#define ESFR_FOTA_CTRL_WMASK               0x02
#define ESFR_FOTA_CTRL_RMASK               0x00  //should this be 0x2???
#define ESFR_FOTA_CTRL_GO                  0x01
#define ESFR_FOTA_CTRL_GOACK               0x02

#define ESFR_FOTA_GP0_VAL0_WMASK           0xFF
#define ESFR_FOTA_GP0_VAL0_RMASK           0x00

#define ESFR_FOTA_GP0_VAL1_WMASK           0xFF
#define ESFR_FOTA_GP0_VAL1_RMASK           0x00

#define ESFR_FOTA_GP0_VAL2_WMASK           0xFF
#define ESFR_FOTA_GP0_VAL2_RMASK           0x00

#define ESFR_FOTA_GP0_VAL3_WMASK           0xFF
#define ESFR_FOTA_GP0_VAL3_RMASK           0x00


#define ESFR_FOTA_GP1_VAL0_WMASK           0xFF
#define ESFR_FOTA_GP1_VAL0_RMASK           0xFF  //should be 0xFF

#define ESFR_FOTA_GP1_VAL1_WMASK           0xFF
#define ESFR_FOTA_GP1_VAL1_RMASK           0xFF

#define ESFR_FOTA_GP1_VAL2_WMASK           0xFF
#define ESFR_FOTA_GP1_VAL2_RMASK           0xFF

#define ESFR_FOTA_GP1_VAL3_WMASK           0xFF
#define ESFR_FOTA_GP1_VAL3_RMASK           0xFF

#define ESFR_FOTA_CMPL_STAT_WMASK          0x01
#define ESFR_FOTA_CMPL_STAT_RMASK          0x01

#define ESFR_FOTA_ERR_STAT_WMASK           0xFF
#define ESFR_FOTA_ERR_STAT_RMASK           0xFF
#define ESFR_FOTA_ERR_STAT_FLASH           0x01
#define ESFR_FOTA_ERR_STAT_CTRL            0x02
#define ESFR_FOTA_ERR_STAT_MCU             0x04

#define ESFR_MCU_CFG_SEL_WMASK             0x01
#define ESFR_MCU_CFG_SEL_RMASK             0x03
#define ESFR_MCU_CFG_SEL_BUSGNT            0x02

#define ESFR_MCU_CFG_CTRL_WMASK            0x3E
#define ESFR_MCU_CFG_CTRL_RMASK            0x3E
#define ESFR_MCU_CFG_CTRL_BUSGNT           0x02

#define ESFR_MCU_CFG_ADDR0_WMASK           0xFF
#define ESFR_MCU_CFG_ADDR0_RMASK           0xFF

#define ESFR_MCU_CFG_ADDR1_WMASK           0xFF
#define ESFR_MCU_CFG_ADDR1_RMASK           0xFF

#define ESFR_MCU_CFG_WR_DAT0_WMASK         0xFF
#define ESFR_MCU_CFG_WR_DAT0_RMASK         0xFF

#define ESFR_MCU_CFG_WR_DAT1_WMASK         0xFF
#define ESFR_MCU_CFG_WR_DAT1_RMASK         0xFF

#define ESFR_MCU_CFG_WR_DAT2_WMASK         0xFF
#define ESFR_MCU_CFG_WR_DAT2_RMASK         0xFF

#define ESFR_MCU_CFG_WR_DAT3_WMASK         0xFF
#define ESFR_MCU_CFG_WR_DAT3_RMASK         0xFF


#define ESFR_MCU_CFG_RD_DAT0_WMASK         0x00
#define ESFR_MCU_CFG_RD_DAT0_RMASK         0xFF

#define ESFR_MCU_CFG_RD_DAT1_WMASK         0x00
#define ESFR_MCU_CFG_RD_DAT1_RMASK         0xFF

#define ESFR_MCU_CFG_RD_DAT2_WMASK         0x00
#define ESFR_MCU_CFG_RD_DAT2_RMASK         0xFF

#define ESFR_MCU_CFG_RD_DAT3_WMASK         0x00
#define ESFR_MCU_CFG_RD_DAT3_RMASK         0xFF


#define ESFR_MCU_CFG_STAT_WMASK            0x00
#define ESFR_MCU_CFG_STAT_RMASK            0x00

#define ESFR_MCU_CFG_STAT_CFGDONE          0x01
#define ESFR_MCU_CFG_STAT_READ             0x07
#define ESFR_MCU_CFG_STAT_WRITE            0x0E

#define ESFR_MCU_CFG_RD_DAT0_WMASK         0x00
#define ESFR_MCU_CFG_RD_DAT0_RMASK         0xFF

#define ESFR_MCU_CFG_RD_DAT1_WMASK         0x00
#define ESFR_MCU_CFG_RD_DAT1_RMASK         0xFF

#define ESFR_MCU_CFG_RD_DAT2_WMASK         0x00
#define ESFR_MCU_CFG_RD_DAT2_RMASK         0xFF

#define ESFR_MCU_CFG_RD_DAT3_WWMASK        0x00
#define ESFR_MCU_CFG_RD_DAT3_RMASK         0xFF

#define ESFR_MCU_DAT_SEL_WMASK             0x05
#define ESFR_MCU_DAT_SEL_RMASK             0x07
#define ESFR_MCU_DAT_SEL_BUSGNT            0x02
#define ESFR_MCU_DAT_SEL_FOTAWR            0x04

#define ESFR_MCU_DAT_CTRL_WMASK            0x7E
#define ESFR_MCU_DAT_CTRL_RMASK            0x7E

#define ESFR_MCU_DAT_ADDR0_WMASK           0xFF
#define ESFR_MCU_DAT_ADDR0_RMASK           0xFF

#define ESFR_MCU_DAT_ADDR1_WMASK           0xFF
#define ESFR_MCU_DAT_ADDR1_RMASK           0xFF

#define ESFR_MCU_DAT_ADDR2_WMASK           0xFF
#define ESFR_MCU_DAT_ADDR2_RMASK           0xFF

#define ESFR_MCU_DAT_ADDR3_WMASK           0xFF
#define ESFR_MCU_DAT_ADDR3_RMASK           0xFF


#define ESFR_MCU_DAT_WR_DAT0_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT0_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT1_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT1_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT2_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT2_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT3_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT3_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT4_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT4_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT5_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT5_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT6_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT6_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT7_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT7_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT8_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT8_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT9_WMASK         0xFF
#define ESFR_MCU_DAT_WR_DAT9_RMASK         0xFF

#define ESFR_MCU_DAT_WR_DAT10_WMASK        0xFF
#define ESFR_MCU_DAT_WR_DAT10_RMASK        0xFF

#define ESFR_MCU_DAT_WR_DAT11_WMASK        0xFF
#define ESFR_MCU_DAT_WR_DAT11_RMASK        0xFF

#define ESFR_MCU_DAT_WR_DAT12_WMASK        0xFF
#define ESFR_MCU_DAT_WR_DAT12_RMASK        0xFF

#define ESFR_MCU_DAT_WR_DAT13_WMASK        0xFF
#define ESFR_MCU_DAT_WR_DAT13_RMASK        0xFF

#define ESFR_MCU_DAT_WR_DAT14_WMASK        0xFF
#define ESFR_MCU_DAT_WR_DAT14_RMASK        0xFF

#define ESFR_MCU_DAT_WR_DAT15_WMASK        0xFF
#define ESFR_MCU_DAT_WR_DAT15_RMASK        0xFF

#define ESFR_MCU_DAT_STAT_WMASK            0x00
#define ESFR_MCU_DAT_STAT_RMASK            0x7F

#define ESFR_MCU_DAT_RD_DAT0_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT0_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT1_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT1_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT2_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT2_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT3_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT3_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT4_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT4_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT5_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT5_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT6_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT6_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT7_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT7_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT8_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT8_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT9_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT9_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT10_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT10_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT11_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT11_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT12_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT12_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT13_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT13_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT14_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT14_RMASK         0xFF

#define ESFR_MCU_DAT_RD_DAT15_WMASK         0x00
#define ESFR_MCU_DAT_RD_DAT15_RMASK         0xFF

#define ESFR_CPU_DAT_CMD_INFO_WMASK         0x00
#define ESFR_CPU_DAT_CMD_INFO_RMASK         0x03




//OPTISPI Registers
#define OPTISPI_CFG_REG                     0x00
#define OPTISPI_DEV_INSTR_RD_CONFIG_REG     (0x04U)
#define OPTISPI_CFG_REG_RST_MASK3           0x80
#define OPTISPI_CFG_REG_RST_MASK2           0x78
#define OPTISPI_CFG_REG_RST_MASK1           0x00
#define OPTISPI_CFG_REG_RST_MASK0           0x00
#define OPTISPI_STIG_REG                    0x90
#define OPTISPI_CMD_ADDR_REG                0x94
#define OPTISPI_CMD_RDDATA_LOW_REG          0xA0
#define OPTISPI_CMD_RDDATA_HIGH_REG         0xA4
#define OPTISPI_CMD_WRDATA_LOW_REG          0xA8
#define OPTISPI_CMD_WRDATA_HIGH_REG         0xAC
#define OPTISPI_WRITE_STAT                  0x05920401
#define OPTISPI_FLHCMD_RD_REG_L             0xA0
#define OPTISPI_FLHCMD_RD_REG_U             0xA4
#define OPTISPI_WRACTV_MASK                 0x00000001
#define OPTISPI_INT_STAT_REG                0x40
#define OPTISPI_NON_IMPL1                   0x84
#define OPTISPI_SRAM_FILL_LVL               0x2C    //RO

#define OPTISPI_IND_RD_XFR_CONT_REG         0x60
#define OPTISPI_IND_RD_WMRK_REG             0x64
#define OPTISPI_IND_RD_XFR_STRT_REG         0x68
#define OPTISPI_IND_RD_XFR_NUM_BYTES_REG    0x6C
#define OPTISPI_IND_WR_XFR_CONT_REG         0x70
#define OPTISPI_IND_WR_WMRK_REG             0x74
#define OPTISPI_IND_WR_XFR_STRT_REG         0x78
#define OPTISPI_IND_WR_XFR_NUM_BYTES_REG    0x7C
#define OPTISPI_IND_TRIG_ADDR_RNG_REG       0x80
#define OPTISPI_IND_AHB_ADDR_TRIG_REG       0x1C
#define OPTISPI_SRM_PART_CNFG_REG           0x18
#define OPTISPI_CFG_OPCODE_EXT_LOWER_REG    0xE0


#define MEMORY_BYTE0                        0x400
#define MEMORY_BYTE1                        0x401
#define MEMORY_BYTE2                        0x402
#define MEMORY_BYTE3                        0x403

#define ISSI_ERASE0                         0x01
#define ISSI_ERASE1                         0x00
#define ISSI_ERASE2                         0x0B
#define ISSI_ERASE3                         0xDC

#define ISSI_WE0                            0x01
#define ISSI_WE1                            0x00
#define ISSI_WE2                            0x00
#define ISSI_WE3                            0x06


#endif    /*define __FSS_M8051_CONST_H__*/
