
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

#include <stdint.h>
#include <8051.h>
#include "boot.h"
#include "fss_m8051_const.h"
#include "fss_m8051_ospi.h"

#define FW_MAJOR_VER 1
#define FW_MINOR_VER 0
#define FW_PATCH_VER 0

/**
 * @brief 0xFD shuts down phy pipeline
 * 
 */
#define PIPELINE_SHUTDOWN_MASK (0xFDU)

uint8_t gStig_opcode = 0;
uint8_t gStig_exopcode = 0;
uint8_t gStig_readbytes = 0;
uint8_t gStig_writebytes = 10;
uint8_t gStig_addressBytes = 0;
uint8_t gStig_dummyCyc = 0;

uint8_t gStig_wrdata_0 = 0;
uint8_t gStig_wrdata_1 = 0;
uint8_t gStig_wrdata_2 = 0;
uint8_t gStig_wrdata_3 = 0;
uint8_t gStig_wrdata_4 = 0;
uint8_t gStig_wrdata_5 = 0;
uint8_t gStig_wrdata_6 = 0;
uint8_t gStig_wrdata_7 = 0;
uint8_t gStig_rdData_4 = 0;
uint8_t gStig_rdData_5 = 0;
uint8_t gStig_rdData_6 = 0;
uint8_t gStig_rdData_7 = 0;
uint8_t gStig_rdData_0 = 0;
uint8_t gStig_rdData_1 = 0;
uint8_t gStig_rdData_2 = 0;
uint8_t gStig_rdData_3 = 0;

void main()
{
    const uint8_t cfgmask = PIPELINE_SHUTDOWN_MASK; 

    uint8_t i = 0;
    uint8_t rdata3 = 0;
    uint8_t rdata0 = 0;

    while (1)
    {

        FOTA_WaitGo();
        ESFR_FOTA_CMPL_STAT = 0;

        switch(ESFR_FOTA_GP0_VAL0)
        {
        case 0 :
        {
            uint8_t initOspiCfg_0 = 0; // oritinal ospi cfg register values
            uint8_t initOspiCfg_1 = 0; // oritinal ospi cfg register values
            uint8_t initOspiCfg_2 = 0; // oritinal ospi cfg register values
            uint8_t initOspiCfg_3 = 0; // oritinal ospi cfg register values

            // 2.
            FOTA_GetCfgBusOwnership();
            // 3.
            // right now this is difficult to setup with variable
            // settings. the values of cfg reg need to remain fixed
            // then the loading of the DAT registers can take place
            // prior to the go signal.

            FOTA_ReadCfg(OPTISPI_CFG_REG, ESFR_OSPI_REGS_RSEL);
            initOspiCfg_3 = ESFR_MCU_CFG_RD_DAT3;
            initOspiCfg_2 = ESFR_MCU_CFG_RD_DAT2;
            initOspiCfg_1 = ESFR_MCU_CFG_RD_DAT1;
            initOspiCfg_0 = ESFR_MCU_CFG_RD_DAT0;
            ESFR_MCU_CFG_WR_DAT3 = initOspiCfg_3 & cfgmask;
            ESFR_MCU_CFG_WR_DAT2 = initOspiCfg_2;
            ESFR_MCU_CFG_WR_DAT1 = initOspiCfg_1;
            ESFR_MCU_CFG_WR_DAT0 = initOspiCfg_0;
            // before entering the FOTA code need to be certain that the
            // OSPI SPI pins are idle
            rdata3 = 0x00;
            while (rdata3 != 0x80)
            {
                P1 = 0x09;
                for (i = 0; i < 5; i++)
                {
                }

                FOTA_ReadCfg(OPTISPI_CFG_REG, ESFR_OSPI_REGS_RSEL);
                // FOTA_ReadCfg(OPTISPI_STIG_REG,ESFR_OSPI_REGS_RSEL);
                rdata3 = ESFR_MCU_CFG_RD_DAT3 & 0x80;
            }

            // Preload the ADDR0/1 registers with the target address for
            // cfg write to optimize the time of the timing sensitive
            // block.
            ESFR_MCU_CFG_ADDR0 = OPTISPI_CFG_REG & 0x00FF; // write the address out to esfr registers
            ESFR_MCU_CFG_ADDR1 = (OPTISPI_CFG_REG >> 8) & 0x00FF;

            //**********************************
            //**********************************
            //**********************************
            // BEGIN TIMING OPTMIZED CODE BLOCK
            FOTA_GetDatBusOwnershipWbuf();
            FOTA_WriteCfgPreload(ESFR_OSPI_REGS_RSEL);
            // 4.
            FOTA_TransferDataWrRsel3();
            // 5.
            // poll to see that the OSPI has started processing
            // the STIG command indicating that the interface
            // engine is free and the DAT can be dropped after
            // restoration of the control register (PHY pipeline on)
            rdata0 = 0x01;
            rdata3 = 0x00;

            // this cannot be loaded before entering the timing optimized block
            // becasue of the use of ADDR0/1 in the write to disable the PHY
            ESFR_MCU_CFG_ADDR0 = OPTISPI_CFG_REG & 0x00FF; // write the address out to esfr registers
            ESFR_MCU_CFG_ADDR1 = (OPTISPI_CFG_REG >> 8) & 0x00FF;

            while (rdata3 != 0x80)
            {
                for (i = 0; i < 5; i++)
                {
                }
                FOTA_ReadCfgPreload(ESFR_OSPI_REGS_RSEL);
                // FOTA_ReadCfg(OPTISPI_STIG_REG,ESFR_OSPI_REGS_RSEL);
                rdata3 = ESFR_MCU_CFG_RD_DAT3 & 0x80;
            }
            // release the dat bus
            //         FOTA_ReleaseDatBusOwnership();
            //**********************************
            //**********************************
            //**********************************
            // END TIMING OPTIMIZED CODE BLOCK
            // restore the cfg register, re-enable the phy pipeline
            ESFR_MCU_CFG_WR_DAT3 = initOspiCfg_3;
            ESFR_MCU_CFG_WR_DAT2 = initOspiCfg_2;
            ESFR_MCU_CFG_WR_DAT1 = initOspiCfg_1;
            ESFR_MCU_CFG_WR_DAT0 = initOspiCfg_0;
            FOTA_WriteCfg(OPTISPI_CFG_REG, ESFR_OSPI_REGS_RSEL);
            FOTA_ReleaseDatBusOwnership();
            FOTA_ReleaseCfgBusOwnership();
            // Poll the Flash wait until write has completed
            FOTA_WaitFlashBusy(0xFC00);
        }
        break;
        case 1:
        {
            FOTA_EraseSector();
        }
        break;
        case 2:
        {
            switch(ESFR_FOTA_GP0_VAL1)
            {
                case 0:
                    gStig_opcode = ESFR_FOTA_GP0_VAL2;
                    break;
                case 1:
                    gStig_exopcode = ESFR_FOTA_GP0_VAL2;
                    break;
                case 3:
                    gStig_readbytes = (ESFR_FOTA_GP0_VAL2 & 0xf) << 4;
                    break;
                case 4:
                    gStig_writebytes = (ESFR_FOTA_GP0_VAL2 & 0xf) << 4;
                    break;
                case 5:
                    gStig_addressBytes = ESFR_FOTA_GP0_VAL2 & 0xf;
                    break;
                case 6:
                    gStig_dummyCyc = ESFR_FOTA_GP0_VAL2;
                    break;
                case 7:
                    gStig_wrdata_0 = ESFR_MCU_DAT_ADDR0;
                    gStig_wrdata_1 = ESFR_MCU_DAT_ADDR1;
                    gStig_wrdata_2 = ESFR_MCU_DAT_ADDR2;
                    gStig_wrdata_3 = ESFR_MCU_DAT_ADDR3;
                    break;
                case 8:
                    gStig_wrdata_4 = ESFR_MCU_DAT_ADDR0;
                    gStig_wrdata_5 = ESFR_MCU_DAT_ADDR1;
                    gStig_wrdata_6 = ESFR_MCU_DAT_ADDR2;
                    gStig_wrdata_7 = ESFR_MCU_DAT_ADDR3;
                    break;
                case 9:
                    ESFR_FOTA_GP1_VAL0 = gStig_rdData_0;
                    ESFR_FOTA_GP1_VAL1 = gStig_rdData_1;
                    ESFR_FOTA_GP1_VAL2 = gStig_rdData_2;
                    ESFR_FOTA_GP1_VAL3 = gStig_rdData_3;
                    break;
                case 10:
                    ESFR_FOTA_GP1_VAL0 = gStig_rdData_4;
                    ESFR_FOTA_GP1_VAL1 = gStig_rdData_5;
                    ESFR_FOTA_GP1_VAL2 = gStig_rdData_6;
                    ESFR_FOTA_GP1_VAL3 = gStig_rdData_7;
                    break;

                default:
                    break;
            }
        }
        break;
        case 3:
        {
            uint8_t initOspiCfg_0 = 0; // oritinal ospi cfg register values
            uint8_t initOspiCfg_1 = 0; // oritinal ospi cfg register values
            uint8_t initOspiCfg_2 = 0; // oritinal ospi cfg register values
            uint8_t initOspiCfg_3 = 0; // oritinal ospi cfg register values

            uint8_t initOspiRdCfg_0 = 0; // oritinal ospi cfg register values
            uint8_t initOspiRdCfg_1 = 0; // oritinal ospi cfg register values
            uint8_t initOspiRdCfg_2 = 0; // oritinal ospi cfg register values
            uint8_t initOspiRdCfg_3 = 0; // oritinal ospi cfg register values

            FOTA_GetCfgBusOwnership();
            // before entering the FOTA code need to be certain that the
            // OSPI SPI pins are idle
            rdata3 = 0x00;
            while (rdata3 != 0x80)
            {
                for (i = 0; i < 5; i++)
                {
                }

                FOTA_ReadCfg(OPTISPI_CFG_REG, ESFR_OSPI_REGS_RSEL);
                // FOTA_ReadCfg(OPTISPI_STIG_REG,ESFR_OSPI_REGS_RSEL);
                rdata3 = ESFR_MCU_CFG_RD_DAT3 & 0x80;
            }

            // BEGIN TIMING OPTMIZED CODE BLOCK
            FOTA_GetDatBusOwnership();

            FOTA_ReadCfg(OPTISPI_DEV_INSTR_RD_CONFIG_REG, ESFR_OSPI_REGS_RSEL);
            initOspiRdCfg_3 = ESFR_MCU_CFG_RD_DAT3;
            initOspiRdCfg_2 = ESFR_MCU_CFG_RD_DAT2;
            initOspiRdCfg_1 = ESFR_MCU_CFG_RD_DAT1;
            initOspiRdCfg_0 = ESFR_MCU_CFG_RD_DAT0;

            // configure stig address
            ESFR_MCU_CFG_WR_DAT3 = ESFR_MCU_DAT_ADDR3;
            ESFR_MCU_CFG_WR_DAT2 = ESFR_MCU_DAT_ADDR2;
            ESFR_MCU_CFG_WR_DAT1 = ESFR_MCU_DAT_ADDR1;
            ESFR_MCU_CFG_WR_DAT0 = ESFR_MCU_DAT_ADDR0;
            FOTA_WriteCfg(OPTISPI_CMD_ADDR_REG, ESFR_OSPI_REGS_RSEL);

            ESFR_MCU_CFG_WR_DAT3 = gStig_wrdata_3;
            ESFR_MCU_CFG_WR_DAT2 = gStig_wrdata_2;
            ESFR_MCU_CFG_WR_DAT1 = gStig_wrdata_1;
            ESFR_MCU_CFG_WR_DAT0 = gStig_wrdata_0;
            FOTA_WriteCfg(OPTISPI_CMD_WRDATA_LOW_REG, ESFR_OSPI_REGS_RSEL);

            ESFR_MCU_CFG_WR_DAT3 = gStig_wrdata_7;
            ESFR_MCU_CFG_WR_DAT2 = gStig_wrdata_6;
            ESFR_MCU_CFG_WR_DAT1 = gStig_wrdata_5;
            ESFR_MCU_CFG_WR_DAT0 = gStig_wrdata_4;
            FOTA_WriteCfg(OPTISPI_CMD_WRDATA_HIGH_REG, ESFR_OSPI_REGS_RSEL);

            FOTA_ReadCfg(OPTISPI_CFG_OPCODE_EXT_LOWER_REG, ESFR_OSPI_REGS_RSEL);
            ESFR_MCU_CFG_WR_DAT3 = ESFR_MCU_CFG_RD_DAT3;
            ESFR_MCU_CFG_WR_DAT2 = ESFR_MCU_CFG_RD_DAT2;
            ESFR_MCU_CFG_WR_DAT1 = ESFR_MCU_CFG_RD_DAT1;
            ESFR_MCU_CFG_WR_DAT0 = gStig_exopcode;
            FOTA_WriteCfg(OPTISPI_CFG_OPCODE_EXT_LOWER_REG, ESFR_OSPI_REGS_RSEL);

            // configure STIG registers
            ESFR_MCU_CFG_WR_DAT3 = gStig_opcode;
            ESFR_MCU_CFG_WR_DAT2 = gStig_readbytes | gStig_addressBytes;
            ESFR_MCU_CFG_WR_DAT1 = gStig_writebytes |((gStig_dummyCyc >> 1) & 0xf);
            ESFR_MCU_CFG_WR_DAT0 = ((gStig_dummyCyc & 1) << 7);
            FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);

            // 2.
            FOTA_ReadCfg(OPTISPI_CFG_REG, ESFR_OSPI_REGS_RSEL);
            initOspiCfg_3 = ESFR_MCU_CFG_RD_DAT3;
            initOspiCfg_2 = ESFR_MCU_CFG_RD_DAT2;
            initOspiCfg_1 = ESFR_MCU_CFG_RD_DAT1;
            initOspiCfg_0 = ESFR_MCU_CFG_RD_DAT0;
            ESFR_MCU_CFG_WR_DAT3 = initOspiCfg_3 & cfgmask;
            ESFR_MCU_CFG_WR_DAT2 = initOspiCfg_2;
            ESFR_MCU_CFG_WR_DAT1 = initOspiCfg_1;
            ESFR_MCU_CFG_WR_DAT0 = initOspiCfg_0;

            // Preload the ADDR0/1 registers with the target address for
            // cfg write to optimize the time of the timing sensitive
            // block.
            ESFR_MCU_CFG_ADDR0 = OPTISPI_CFG_REG & 0x00FF; // write the address out to esfr registers
            ESFR_MCU_CFG_ADDR1 = (OPTISPI_CFG_REG >> 8) & 0x00FF;

            // disable pipeline
            FOTA_WriteCfgPreload(ESFR_OSPI_REGS_RSEL);

            //
            // need to disable read cfg. register
            // because of OSPI IP bug
            //
            ESFR_MCU_CFG_WR_DAT3 = initOspiRdCfg_3;
            ESFR_MCU_CFG_WR_DAT2 = initOspiRdCfg_2;
            ESFR_MCU_CFG_WR_DAT1 = initOspiRdCfg_1;
            ESFR_MCU_CFG_WR_DAT0 = 0;
            FOTA_WriteCfg(OPTISPI_DEV_INSTR_RD_CONFIG_REG, ESFR_OSPI_REGS_RSEL);

            // now send stig command
            FOTA_ReadCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
            ESFR_MCU_CFG_WR_DAT3 = ESFR_MCU_CFG_RD_DAT3;
            ESFR_MCU_CFG_WR_DAT2 = ESFR_MCU_CFG_RD_DAT2;
            ESFR_MCU_CFG_WR_DAT1 = ESFR_MCU_CFG_RD_DAT1;
            ESFR_MCU_CFG_WR_DAT0 = ESFR_MCU_CFG_RD_DAT0 | 1;
            FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);

            // wait for the stig command to complete
            uint8_t stigflag = 0x02; // set to stig active until reading act ual
            while (stigflag == 0x02)
            {
                for (i = 0; i < 5; i++)
                {
                }
                FOTA_ReadCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
                stigflag = ESFR_MCU_CFG_RD_DAT0 & 0x02;
            }


            // release the dat bus
            // END TIMING OPTIMIZED CODE BLOCK
            // restore the cfg register, re-enable the phy pipeline
            ESFR_MCU_CFG_WR_DAT3 = initOspiCfg_3;
            ESFR_MCU_CFG_WR_DAT2 = initOspiCfg_2;
            ESFR_MCU_CFG_WR_DAT1 = initOspiCfg_1;
            ESFR_MCU_CFG_WR_DAT0 = initOspiCfg_0;
            FOTA_WriteCfg(OPTISPI_CFG_REG, ESFR_OSPI_REGS_RSEL);

            ESFR_MCU_CFG_WR_DAT3 = 0;
            ESFR_MCU_CFG_WR_DAT2 = 0;
            ESFR_MCU_CFG_WR_DAT1 = 0;
            ESFR_MCU_CFG_WR_DAT0 = 0;
            FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);

            ESFR_MCU_CFG_WR_DAT3 = initOspiRdCfg_3;
            ESFR_MCU_CFG_WR_DAT2 = initOspiRdCfg_2;
            ESFR_MCU_CFG_WR_DAT1 = initOspiRdCfg_1;
            ESFR_MCU_CFG_WR_DAT0 = initOspiRdCfg_0;
            FOTA_WriteCfg(OPTISPI_DEV_INSTR_RD_CONFIG_REG, ESFR_OSPI_REGS_RSEL);
            FOTA_ReleaseDatBusOwnership();

            FOTA_ReadCfg(OPTISPI_FLHCMD_RD_REG_L, ESFR_OSPI_REGS_RSEL);
            gStig_rdData_0 = ESFR_MCU_CFG_RD_DAT0;
            gStig_rdData_1 = ESFR_MCU_CFG_RD_DAT1;
            gStig_rdData_2 = ESFR_MCU_CFG_RD_DAT2;
            gStig_rdData_3 = ESFR_MCU_CFG_RD_DAT3;

            FOTA_ReadCfg(OPTISPI_FLHCMD_RD_REG_U, ESFR_OSPI_REGS_RSEL);
            gStig_rdData_4 = ESFR_MCU_CFG_RD_DAT0;
            gStig_rdData_5 = ESFR_MCU_CFG_RD_DAT1;
            gStig_rdData_6 = ESFR_MCU_CFG_RD_DAT2;
            gStig_rdData_7 = ESFR_MCU_CFG_RD_DAT3;


            FOTA_ReleaseCfgBusOwnership();
        }
        break;
        case 4:
        {
            ESFR_FOTA_GP1_VAL0 = FW_PATCH_VER;
            ESFR_FOTA_GP1_VAL1 = FW_MINOR_VER;
            ESFR_FOTA_GP1_VAL2 = FW_MAJOR_VER;
        }
        default:
            break;
        }

        ESFR_FOTA_CMPL_STAT = 0x01;
    }
}

void FOTA_ReadCfg(uint16_t addr, uint8_t rsel)
{
    uint8_t done = 0;
    ESFR_MCU_CFG_ADDR0 = addr & 0x00FF; // write the address out to esfr registers
    ESFR_MCU_CFG_ADDR1 = (addr >> 8) & 0x00FF;
    ESFR_MCU_CFG_CTRL = ((rsel << 2) & 0xFC) | 0x2 | 0x1; // rsel to [5:2] [1] = read [0] = start
    // wait for read to complete
    while (!done)
    {
        done = ESFR_MCU_CFG_STAT & 0x1;
    }
}

void FOTA_ReadCfgPreload(uint8_t rsel)
{
    uint8_t done = 0;
    ESFR_MCU_CFG_CTRL = ((rsel << 2) & 0xFC) | 0x2 | 0x1; // rsel to [5:2] [1] = read [0] = start
    // wait for read to complete
    while (!done)
    {
        done = ESFR_MCU_CFG_STAT & 0x1;
    }
}

void FOTA_TransferDataWrRsel3()
{
    uint8_t busdone = 0;
    ESFR_MCU_DAT_CTRL = 0x0D;
    // wait for transfer to complete
    while (!busdone)
    {
        busdone = ESFR_MCU_DAT_STAT & 0x1;
    }
}

void FOTA_WaitGo()
{
    uint8_t gostat = 0x0;
    while (!gostat)
    {
        gostat = ESFR_FOTA_CTRL_GO & ESFR_FOTA_CTRL;
    }
    ESFR_FOTA_CTRL = ESFR_FOTA_CTRL_GOACK;
}

void FOTA_GetCfgBusOwnership()
{
    uint8_t busflag = 0;

    ESFR_MCU_CFG_SEL = 0x1; // request the config bus ownership
    while (!busflag)
    {
        busflag = ESFR_MCU_CFG_SEL & ESFR_MCU_CFG_SEL_BUSGNT; // read the ownership bit
    }
}

void FOTA_ReleaseCfgBusOwnership()
{
    uint8_t busflag = 0;

    ESFR_MCU_CFG_SEL = 0x0; // request the config bus ownership
    while (busflag != 0x0)
    {
        busflag = ESFR_MCU_CFG_SEL & ESFR_MCU_CFG_SEL_BUSGNT; // read the ownership bit
    }
}

void FOTA_GetDatBusOwnership()
{
    uint8_t busflag = 0;

    ESFR_MCU_DAT_SEL = 0x1; // request the data bus ownership
    while (!busflag)
    {
        busflag = ESFR_MCU_DAT_SEL & ESFR_MCU_DAT_SEL_BUSGNT; // read the ownership bit
    }
}

void FOTA_GetDatBusOwnershipWbuf()
{
    uint8_t busflag = 0;

    ESFR_MCU_DAT_SEL = 0x5; // request the data bus ownership
    while (!busflag)
    {
        busflag = ESFR_MCU_DAT_SEL & ESFR_MCU_DAT_SEL_BUSGNT; // read the ownership bit
    }
}

void FOTA_ReleaseDatBusOwnership()
{
    uint8_t busflag = 0;

    ESFR_MCU_DAT_SEL = 0x0; // request the data bus ownership
}

void FOTA_WaitTimer0(uint16_t time, uint8_t mode)
{
    (void)mode;
    uint8_t timeout = 0x00;
    TL0 = time & 0x00FF;
    TH0 = (time >> 8) & 0x00FF;
    TMOD = 0x1;
    TCON = 0x10;
    while (!timeout)
    {
        timeout = TCON & 0x20;
    }
    TCON = 0x00;
}

void FOTA_WriteCfg(uint16_t addr, uint8_t rsel)
{
    uint8_t busflag = 0;
    uint8_t busdone = 0;

    ESFR_MCU_CFG_ADDR0 = addr & 0x00FF; // write the address out to esfr registers
    ESFR_MCU_CFG_ADDR1 = (addr >> 8) & 0x00FF;

    // start the transaction
    //    P3 = rsel;
    ESFR_MCU_CFG_CTRL = (((rsel << 2) & 0xFC) | 0x1); // rsel to [5:2] [1] = write [0] = start
    while (!busflag)
    {
        busflag = ESFR_MCU_CFG_STAT;
        busdone = busflag & ESFR_MCU_CFG_STAT_CFGDONE;
    }
}

void FOTA_WriteCfgPreload(uint8_t rsel)
{
    uint8_t busflag = 0;
    uint8_t busdone = 0;

    // start the transaction
    ESFR_MCU_CFG_CTRL = (((rsel << 2) & 0xFC) | 0x1); // rsel to [5:2] [1] = write [0] = start
    while (!busflag)
    {
        busflag = ESFR_MCU_CFG_STAT;
        busdone = busflag & ESFR_MCU_CFG_STAT_CFGDONE;
    }
}
/*
void FOTA_GetFlashStatusSTIG()
{
    uint8_t stigflag = 0;
    uint8_t busflag = 0;
    uint8_t i = 0;

    uint8_t done = 0;
    // load the STIG command for status
    ESFR_MCU_CFG_WR_DAT3 = 0x05;
    ESFR_MCU_CFG_WR_DAT2 = 0xB0; // was B0
    // DLB    ESFR_MCU_CFG_WR_DAT2 = 0x00; //was B0
    ESFR_MCU_CFG_WR_DAT1 = 0x04; // DDR needs 8 dummy cycles
    ESFR_MCU_CFG_WR_DAT0 = 0x00;
    // STIG register address
    FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
    ESFR_MCU_CFG_WR_DAT0 = 0x01;
    FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
    //    ESFR_MCU_CFG_ADDR1 = 0x00;
    //    ESFR_MCU_CFG_ADDR0 = OPTISPI_STIG_REG;

    // Read the OPTISPI Read register
    while (done != 3)
    {
        // insert some delay to be able to see value change
        FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
        //        ESFR_MCU_CFG_ADDR1 = 0x00;
        //        ESFR_MCU_CFG_ADDR0 = OPTISPI_STIG_REG;

        //        ESFR_MCU_CFG_CTRL = 0x21;
        stigflag = 0x02; // set to stig active until reading act ual
        while (stigflag == 0x02)
        {
            for (i = 0; i < 5; i++)
            {
            }
            FOTA_ReadCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
            stigflag = ESFR_MCU_CFG_RD_DAT0 & 0x02;
        }

        FOTA_ReadCfg(OPTISPI_FLHCMD_RD_REG_L, ESFR_OSPI_REGS_RSEL);
        stigflag = ESFR_MCU_CFG_RD_DAT0;
        if (done == 0 && stigflag == 0x03)
        {
            done = 0x01;
        }
        else if (done == 0x01 && stigflag == 0x00)
        {
            done = 0x03;
        }
        FOTA_WaitTimer0(0xEFD8, 0x11);
        //        for(i = 0; i < 40; i++){}
    }
}
*/
void FOTA_WaitFlashBusy(uint16_t delay)
{

    uint8_t rdata0 = 0;
    uint8_t rdata1 = 1;
    uint8_t cfgrdata = 0;  // oritinal ospi cfg register values
    uint8_t cfgrdata1 = 0; // oritinal ospi cfg register values
    uint8_t cfgrdata2 = 0; // oritinal ospi cfg register values
    uint8_t cfgrdata3 = 0; // oritinal ospi cfg register values
    uint8_t opcode2 = 0;
    uint8_t i = 0;

    // to issue stig commands reading the flag_reg in
    // the flash and check results
    // poll this until the flag indicates that
    // the program operation is under way [7] == 1
    // Opcode extension lower reg
    FOTA_GetCfgBusOwnership();
    FOTA_ReadCfg(OPTISPI_CFG_OPCODE_EXT_LOWER_REG, ESFR_OSPI_REGS_RSEL);
    ESFR_MCU_CFG_WR_DAT3 = ESFR_MCU_CFG_RD_DAT3;
    ESFR_MCU_CFG_WR_DAT2 = ESFR_MCU_CFG_RD_DAT2;
    ESFR_MCU_CFG_WR_DAT1 = ESFR_MCU_CFG_RD_DAT1;
    ESFR_MCU_CFG_WR_DAT0 = 5;
    FOTA_WriteCfg(OPTISPI_CFG_OPCODE_EXT_LOWER_REG, ESFR_OSPI_REGS_RSEL);

    // Addr RDSR = 0x0
    ESFR_MCU_CFG_WR_DAT0 = 0x0;
    ESFR_MCU_CFG_WR_DAT1 = 0x0;
    ESFR_MCU_CFG_WR_DAT2 = 0x0;
    ESFR_MCU_CFG_WR_DAT3 = 0x0;
    FOTA_WriteCfg(OPTISPI_CMD_ADDR_REG, ESFR_OSPI_REGS_RSEL);
    FOTA_ReleaseCfgBusOwnership();

    ESFR_MCU_CFG_WR_DAT3 = 0x05;
    ESFR_MCU_CFG_WR_DAT2 = 0xB0; // read data bytes - 4
    ESFR_MCU_CFG_WR_DAT1 = 0x04; // 6 dummy cycles not working, try 8
    ESFR_MCU_CFG_WR_DAT0 = 0x01;
    while (rdata1 != 0x00)
    {
        if (delay > 0)
        {
            FOTA_WaitTimer0(delay, 0x01);
        }
        // issue stig command to the flash
        // P1 = 0xA1;
        FOTA_GetCfgBusOwnership();
        FOTA_GetDatBusOwnership();
        FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
        rdata0 = 0x01;
        while (rdata0 != 0x00)
        {
            // FOTA_ReadCfg(OPTISPI_CFG_REG,ESFR_OSPI_REGS_RSEL);
            // A delay needs to be inserted into this loop to
            // avoid race condition.
            for (i = 0; i < 5; i++)
            {
            }
            FOTA_ReadCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
            rdata0 = ESFR_MCU_CFG_RD_DAT0 & 0x02;
        }
        FOTA_ReleaseDatBusOwnership();
        FOTA_GetCfgBusOwnership();
        FOTA_ReadCfg(OPTISPI_FLHCMD_RD_REG_L, ESFR_OSPI_REGS_RSEL);
        FOTA_ReleaseCfgBusOwnership();
        rdata1 = ESFR_MCU_CFG_RD_DAT0 & 0x01;
    }
}


void FOTA_EraseSector()
{

    uint8_t stigflag = 0x02; // start out with busy value
    uint8_t i = 0x00;
    uint8_t cfgrdata0 = 0x00; // oritinal ospi cfg register values
    uint8_t cfgrdata1 = 0x00; // oritinal ospi cfg register values
    uint8_t cfgrdata2 = 0x00; // oritinal ospi cfg register values
    uint8_t cfgrdata3 = 0x00; // oritinal ospi cfg register values
    uint8_t opcode2 = 0x00;
    // load the address into the command address reg
    // the address must be loaded into FOTA_MCU_DATADDR by
    // SOC so that it can be used here.
    ESFR_MCU_CFG_WR_DAT0 = ESFR_MCU_DAT_ADDR0;
    ESFR_MCU_CFG_WR_DAT1 = ESFR_MCU_DAT_ADDR1;
    ESFR_MCU_CFG_WR_DAT2 = ESFR_MCU_DAT_ADDR2;
    ESFR_MCU_CFG_WR_DAT3 = ESFR_MCU_DAT_ADDR3;
    ESFR_FOTA_GP1_VAL0 = 1;
    FOTA_GetCfgBusOwnership();
    FOTA_WriteCfg(OPTISPI_CMD_ADDR_REG, ESFR_OSPI_REGS_RSEL);
    ESFR_FOTA_GP1_VAL0 = 2;

    // formulate the STIG command for WE
    // Opcode extension lower reg
    FOTA_ReadCfg(OPTISPI_CFG_OPCODE_EXT_LOWER_REG, ESFR_OSPI_REGS_RSEL);
    ESFR_MCU_CFG_WR_DAT3 = ESFR_MCU_CFG_RD_DAT3;
    ESFR_MCU_CFG_WR_DAT2 = ESFR_MCU_CFG_RD_DAT2;
    ESFR_MCU_CFG_WR_DAT1 = ESFR_MCU_CFG_RD_DAT1;
    ESFR_MCU_CFG_WR_DAT0 = 0x6;
    FOTA_WriteCfg(OPTISPI_CFG_OPCODE_EXT_LOWER_REG, ESFR_OSPI_REGS_RSEL);
    ESFR_MCU_CFG_WR_DAT0 = 0x01;
    ESFR_MCU_CFG_WR_DAT1 = 0x00;
    ESFR_MCU_CFG_WR_DAT2 = 0x00;
    ESFR_MCU_CFG_WR_DAT3 = 0x06;

    FOTA_GetDatBusOwnership();
    FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);

    // poll to determine when the STIG command is done
    while (stigflag == 0x02)
    {
        for (i = 0; i < 5; i++)
        {
        }
        FOTA_ReadCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
        stigflag = ESFR_MCU_CFG_RD_DAT0 & 0x02;
    }
    ESFR_FOTA_GP1_VAL0 = 3;
    // formulate the STIG for erase (128kB sector)
    FOTA_ReadCfg(OPTISPI_CFG_OPCODE_EXT_LOWER_REG, ESFR_OSPI_REGS_RSEL);
    ESFR_MCU_CFG_WR_DAT3 = ESFR_MCU_CFG_RD_DAT3;
    ESFR_MCU_CFG_WR_DAT2 = ESFR_MCU_CFG_RD_DAT2;
    ESFR_MCU_CFG_WR_DAT1 = ESFR_MCU_CFG_RD_DAT1;
    ESFR_MCU_CFG_WR_DAT0 = 0xdc;
    FOTA_WriteCfg(OPTISPI_CFG_OPCODE_EXT_LOWER_REG, ESFR_OSPI_REGS_RSEL);
    ESFR_MCU_CFG_WR_DAT0 = 0x01;
    ESFR_MCU_CFG_WR_DAT1 = 0x00;
    ESFR_MCU_CFG_WR_DAT2 = 0x0B;
    ESFR_MCU_CFG_WR_DAT3 = 0x21;
    FOTA_WriteCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);

    // poll to determine when the STIG command is done
    stigflag == 0x02;
    while (stigflag == 0x02)
    {
        for (i = 0; i < 5; i++)
        {
        }
        FOTA_ReadCfg(OPTISPI_STIG_REG, ESFR_OSPI_REGS_RSEL);
        stigflag = ESFR_MCU_CFG_RD_DAT0 & 0x02;
    }

    ESFR_FOTA_GP1_VAL0 = 4;
    // now setup the STIG command for the erase operation
    // ESFR_MCU_CFG_WR_DAT2 = 0x07;
    // FOTA_WriteCfg(OPTISPI_STIG_REG,ESFR_OSPI_REGS_RSEL);
    FOTA_ReleaseDatBusOwnership();
    FOTA_ReleaseCfgBusOwnership();
    P1 = 0xA5;
    // Poll the Flash wait until erase has complted
    FOTA_WaitFlashBusy(0xEC00);
}



