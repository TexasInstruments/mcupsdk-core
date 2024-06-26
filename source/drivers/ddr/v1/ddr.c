/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/

/** \file ddr.c
 *
 *  \brief This file used to configure the DDR Controller and PHY.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/soc_config.h>
#include <drivers/ddr.h>
#include <drivers/ddr/v1/soc/am65x/board_ddr_config.h>
#include <drivers/ddr/v1/cslr_emif.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* ========================================================================== */
/*                         Extern Function declerations                       */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void DDR_boardDelay(void)
{
    volatile uint32_t delay;

    for(delay = 0; delay < BOARD_DDR_DELAY; delay++);
}

/**
 * \brief    This function is used to initialize the DDR EMIF and PHY
 *
 */
static void DDR_controllerPHYConfig(void)
{
    volatile uint32_t i;

    /* Perform DDR subsystem configuration */
    /*  VBUSMC2HIF Control Register*/
    HW_WR_REG32((CSL_DDRSS0_SS_CFG_BASE + CSL_EMIF_SSCFG_V2H_CTL_REG),
                 BOARD_DDR_SSCFG_V2H_CTL_VAL);

    /* Perform controller configuration */

    /* Master Register0 */

    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_MSTR),
                 DDRCTL_MSTR);

    /* Refresh Control Register 0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_RFSHCTL0),
                DDRCTL_RFSHCTL0);

    /* ECC Configuration Register 0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ECCCFG0),
                 DDRCTL_ECCCFG0);

    /* Refresh Timing Register */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_RFSHTMG),
                 DDRCTL_RFSHTMG);

    /* CRC Parity Control Register0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_CRCPARCTL0),
                 DDRCTL_CRCPARCTL0);

    /* CRC Parity Control Register1 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_CRCPARCTL1),
                 DDRCTL_CRCPARCTL1);

    /* CRC Parity Control Register2 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_CRCPARCTL2),
                 DDRCTL_CRCPARCTL2);

    /* SDRAM Initialization Register 0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_INIT0),
                 DDRCTL_INIT0);

    /* SDRAM Initialization Register 1 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_INIT1),
                 DDRCTL_INIT1);

    /* SDRAM Initialization Register 3 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_INIT3),
                 DDRCTL_INIT3);

    /* SDRAM Initialization Register 4 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_INIT4),
                 DDRCTL_INIT4);

    /* SDRAM Initialization Register 5 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_INIT5),
                 DDRCTL_INIT5);

    /* SDRAM Initialization Register 6 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_INIT6),
                 DDRCTL_INIT6);

    /* SDRAM Initialization Register 7 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_INIT7),
                 DDRCTL_INIT7);

    /* SDRAM Timing Register 0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG0),
                 DDRCTL_DRAMTMG0);

    /* SDRAM Timing Register 1 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG1),
                 DDRCTL_DRAMTMG1);

    /* SDRAM Timing Register 2 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG2),
                 DDRCTL_DRAMTMG2);

    /* SDRAM Timing Register 3 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG3),
                 DDRCTL_DRAMTMG3);

    /* SDRAM Timing Register 4 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG4),
                 DDRCTL_DRAMTMG4);

    /* SDRAM Timing Register 5 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG5),
                 DDRCTL_DRAMTMG5);

    /* SDRAM Timing Register 6 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG6),
                 DDRCTL_DRAMTMG6);

    /* SDRAM Timing Register 7 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG7),
                 DDRCTL_DRAMTMG7);

    /* SDRAM Timing Register 8 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG8),
                 DDRCTL_DRAMTMG8);

    /* SDRAM Timing Register 9 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG9),
                 DDRCTL_DRAMTMG9);

    /* SDRAM Timing Register 10 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG10),
                 DDRCTL_DRAMTMG10);

    /* SDRAM Timing Register 11 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG11),
                 DDRCTL_DRAMTMG11);

    /* SDRAM Timing Register 12 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG12),
                 DDRCTL_DRAMTMG12);

    /* SDRAM Timing Register 13 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG13),
                 DDRCTL_DRAMTMG13);

    /* SDRAM Timing Register 14 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG14),
                 DDRCTL_DRAMTMG14);

    /* SDRAM Timing Register 15 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG15),
                 DDRCTL_DRAMTMG15);

    /* SDRAM Timing Register 17 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG17),
                 DDRCTL_DRAMTMG17);

    /* ZQ Control Register 0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ZQCTL0),
                 DDRCTL_ZQCTL0);

    /* ZQ Control Register 1 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ZQCTL1),
                 DDRCTL_ZQCTL1);

    /* DFI Timing Register 0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DFITMG0),
                 DDRCTL_DFITMG0);

    /* DFI Timing Register 1 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DFITMG1),
                 DDRCTL_DFITMG1);

    /* DFI Timing Register 2 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DFITMG2),
                 DDRCTL_DFITMG2);

    /* DFI Timing Misc Register */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DFIMISC),
                 DDRCTL_DFIMISC);

    /* Address Map Register 0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP0),
                 DDRCTL_ADDRMAP0);

    /* Address Map Register 1 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP1),
                 DDRCTL_ADDRMAP1);

    /* Address Map Register 2 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP2),
                 DDRCTL_ADDRMAP2);

    /* Address Map Register 3 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP3),
                 DDRCTL_ADDRMAP3);

    /* Address Map Register 4 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP4),
                 DDRCTL_ADDRMAP4);

    /* Address Map Register 5 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP5),
                 DDRCTL_ADDRMAP5);

    /* Address Map Register 6 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP6),
                 DDRCTL_ADDRMAP6);

    /* Address Map Register 7 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP7),
                 DDRCTL_ADDRMAP7);

    /* Address Map Register 8 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP8),
                 DDRCTL_ADDRMAP8);

    /* Address Map Register 9 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP9),
                 DDRCTL_ADDRMAP9);

    /* Address Map Register 10 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP10),
                 DDRCTL_ADDRMAP10);

    /* Address Map Register 11 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ADDRMAP11),
                 DDRCTL_ADDRMAP11);

    /* DQ Map Register 0 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DQMAP0),
                 DDRCTL_DQMAP0);

    /* DQ Map Register 1 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DQMAP1),
                 DDRCTL_DQMAP1);

    /* DQ Map Register 4 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DQMAP4),
                 DDRCTL_DQMAP4);

    /* DQ Map Register 5 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DQMAP5),
                 DDRCTL_DQMAP5);

    /* ODT Configuration Register */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ODTCFG),
                 DDRCTL_ODTCFG);

    /* ODT Rank Map */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ODTMAP),
                 DDRCTL_ODTMAP);


    /* disable refeshes to get ready for PHY initialization and training */
    i = HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_RFSHCTL3);
    /* Refresh Control Register 3 */
    HW_WR_REG32((CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_RFSHCTL3), (i | 0x1));


    /* De-assert core reset */

    /* Subsystem Control Register */
    if( ((HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_ECCCFG0) & 0x7) == 0x4) && ((HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_MSTR) & 0x20) == 0x20))
        HW_WR_REG32(CSL_DDRSS0_SS_CFG_BASE + CSL_EMIF_SSCFG_SS_CTL_REG,0x00010000); /* Subsystem Control Register */ 
    else
        HW_WR_REG32(CSL_DDRSS0_SS_CFG_BASE + CSL_EMIF_SSCFG_SS_CTL_REG,0x00000000); /* Subsystem Control Register */ 


    /* Configure PHY Initialization Registers */

    /* PHY General Configuration Register 0*/
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGCR0,
                 DDRPHY_PGCR0);

    /* PHY General Configuration Register 1*/
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGCR1,
                 DDRPHY_PGCR1);

    /* PHY General Configuration Register 2*/
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGCR2,
                 DDRPHY_PGCR2);

    /* PHY General Configuration Register 3*/
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGCR3,
                 DDRPHY_PGCR3);

    /* PHY General Configuration Register 6*/
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGCR6,
                 DDRPHY_PGCR6);

    /* PHY Timing Register 2 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PTR2,
                 DDRPHY_PTR2);

    /* PHY Timing Register 3 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PTR3,
                 DDRPHY_PTR3);

    /* PHY Timing Register 4 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PTR4,
                 DDRPHY_PTR4);

    /* PHY Timing Register 5 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PTR5,
                 DDRPHY_PTR5);

    /* PHY Timing Register 6 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PTR6,
                 DDRPHY_PTR6);

    /* PLL Control Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PLLCR0,
                 DDRPHY_PLLCR0);

    /* DATX8 Common Configuration Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DXCCR,
                 DDRPHY_DXCCR);

    /* DDR System General Configuration Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DSGCR,
                 DDRPHY_DSGCR);

    /* DRAM Configuration Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DCR,
                 DDRPHY_DCR);

    /* DRAM Timing Parameters Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTPR0,
                 DDRPHY_DTPR0);

    /* DRAM Timing Parameters Register 1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTPR1,
                 DDRPHY_DTPR1);

    /* DRAM Timing Parameters Register 2 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTPR2,
                 DDRPHY_DTPR2);

    /* DRAM Timing Parameters Register 3 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTPR3,
                 DDRPHY_DTPR3);

    /* DRAM Timing Parameters Register 4 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTPR4,
                 DDRPHY_DTPR4);

    /* DRAM Timing Parameters Register 5 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTPR5,
                 DDRPHY_DTPR5);

    /* DRAM Timing Parameters Register 6 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTPR6,
                 DDRPHY_DTPR6);

    /* Impedance Control Register   */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_ZQCR,
                 DDRPHY_ZQCR);

    /* Impedance Controller Program Register 0 for Calibration Segment 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_ZQ0PR0,
                 DDRPHY_ZQ0PR0);

    /* Impedance Controller Program Register 0 for Calibration Segment 1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_ZQ1PR0,
                 DDRPHY_ZQ1PR0);

    /* Configure DRAM Initialization registers */

    /* DDR Mode Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR0_DDR4,
                 DDRPHY_MR0);

    /* DDR Mode Register 1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR1_DDR4,
                 DDRPHY_MR1);

    /* DDR Mode Register 2 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR2_DDR4,
                 DDRPHY_MR2);

    /* DDR Mode Register 3 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR3_DDR4,
                 DDRPHY_MR3);

    /* DDR Mode Register 4 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR4_DDR4,
                 DDRPHY_MR4);

    /* DDR Mode Register 5 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR5_DDR4,
                 DDRPHY_MR5);

    /* DDR Mode Register 6 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR6_DDR4,
                 DDRPHY_MR6);

    /* DDR Mode Register 11 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR11_DDR4,
                 DDRPHY_MR11);

    /* DDR Mode Register 12 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR12_DDR4,
                 DDRPHY_MR12);

    /* DDR Mode Register 13 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR13_DDR4,
                 DDRPHY_MR13);

    /* DDR Mode Register 14 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR14_DDR4,
                 DDRPHY_MR14);

    /* DDR Mode Register 22 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_MR22_DDR4,
                 DDRPHY_MR22);

    /* Data Training Configuration Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_VTCR0,
                 DDRPHY_VTCR0);

    /* DAXT8 bytes 0-1 PLL Control Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL0PLLCR0,
                 DDRPHY_DX8SL0PLLCR0);

    /* DAXT8 bytes 2-3 PLL Control Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL1PLLCR0,
                 DDRPHY_DX8SL1PLLCR0);

    /* DAXT8 bytes 4 PLL Control Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL2PLLCR0,
                 DDRPHY_DX8SL2PLLCR0);

    /* Data Training Configuration Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTCR0,
                 DDRPHY_DTCR0);

    /* Data Training Configuration Register 1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DTCR1,
                 DDRPHY_DTCR1);

    /* AC I/O Configuration Register 3 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_ACIOCR3,
                 DDRPHY_ACIOCR3);

    /* AC I/O Configuration Register 5 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_ACIOCR5,
                 DDRPHY_ACIOCR5);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_IOVCR0,
                 DDRPHY_IOVCR0);

    /* DATX8 n General Configuration Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GCR0,
                 DDRPHY_DX2GCR0);
    /* DATX8 n General Configuration Register 1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GCR1,
                 DDRPHY_DX2GCR1);
    /* DATX8 n General Configuration Register 2 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GCR2,
                 DDRPHY_DX2GCR2);
    /* DATX8 n General Configuration Register 3 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GCR3,
                 DDRPHY_DX2GCR3);

    /* DATX8 n General Configuration Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GCR0,
                 DDRPHY_DX3GCR0);
    /* DATX8 n General Configuration Register 1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GCR1,
                 DDRPHY_DX3GCR1);
    /* DATX8 n General Configuration Register 2 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GCR2,
                 DDRPHY_DX3GCR2);
    /* DATX8 n General Configuration Register 3 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GCR3,
                 DDRPHY_DX3GCR3);

    /* DATX8 n General Configuration Register 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GCR0,
                 DDRPHY_DX4GCR0);
    /* DATX8 n General Configuration Register 1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GCR1,
                 DDRPHY_DX4GCR1);
    /* DATX8 n General Configuration Register 2 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GCR2,
                 DDRPHY_DX4GCR2);
    /* DATX8 n General Configuration Register 3 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GCR3,
                 DDRPHY_DX4GCR3);

    /* VREF for read DQS gate */

    /* DATX8 n General Configuration Register 4 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX0GCR4,
                 DDRPHY_DX0GCR4);

    /* DATX8 n General Configuration Register 4 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX1GCR4,
                 DDRPHY_DX1GCR4);

    /* DATX8 n General Configuration Register 4 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GCR4,
                 DDRPHY_DX2GCR4);

    /* DATX8 n General Configuration Register 4 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GCR4,
                 DDRPHY_DX3GCR4);

    /* DATX8 n General Configuration Register 4 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GCR4,
                 DDRPHY_DX4GCR4);

    /* VREF for DQS and DQ */
    /* PHY General Configuration Register 5 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGCR5,
                 DDRPHY_PGCR5);

    /* DATX8 n General Configuration Register 5 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX0GCR5,
                 DDRPHY_DX0GCR5);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX1GCR5,
                 DDRPHY_DX1GCR5);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GCR5,
                 DDRPHY_DX2GCR5);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GCR5,
                 DDRPHY_DX3GCR5);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GCR5,
                 DDRPHY_DX4GCR5);

    /* Rank ID Register rank 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_RANKIDR,
                 0x00000000);

    /* DATX8 n General Timing Registers for rank 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX0GTR0,
                 DDRPHY_DX0GTR0);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX1GTR0,
                 DDRPHY_DX1GTR0);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GTR0,
                 DDRPHY_DX2GTR0);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GTR0,
                 DDRPHY_DX3GTR0);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GTR0,
                 DDRPHY_DX4GTR0);

    /* ODT Configuration Register for rank 0 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_ODTCR,
                 DDRPHY_ODTCR);

    /* Rank ID Register rank 1*/
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_RANKIDR,
                 0x00010001);


    /* DATX8 n General Timing Registers for rank1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX0GTR0,
                 DDRPHY_DX0GTR0);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX1GTR0,
                 DDRPHY_DX1GTR0);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GTR0,
                 DDRPHY_DX2GTR0);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GTR0,
                 DDRPHY_DX3GTR0);

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GTR0,
                 DDRPHY_DX4GTR0);

    /* ODT Configuration Register for rank 1 */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_ODTCR,
                 DDRPHY_ODTCR);

    /* DATX8 0 I/O Configuration Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL0IOCR,
                 DDRPHY_DX8SL0IOCR);

    /* DATX8 1 I/O Configuration Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL1IOCR,
                 DDRPHY_DX8SL1IOCR);

    /* DATX8 2 I/O Configuration Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL2IOCR,
                 DDRPHY_DX8SL2IOCR);

    /* DATX8 0 Data Control Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL0DXCTL2,
                 DDRPHY_DX8SL0DXCTL2);

    /* DATX8 1 Data Control Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL1DXCTL2,
                 DDRPHY_DX8SL1DXCTL2);

    /* DATX8 2 Data Control Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL2DXCTL2,
                 DDRPHY_DX8SL2DXCTL2);
}

static void DDR_PHYInit()
{
    /* PHY Initialization Register - kicks off init */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR,0x73);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for PGSR0.IDONE */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x1U) != 0x1U);

    /* wait another 32 ctl_clk cycles before resuming further traffic */
    DDR_boardDelay();
}

static void DDR_SDRAMInit(unsigned int mask)
{
    /* PHY Initialization Register - kicks off init */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR,0x181);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for init done, PLL lock, DDL Calibration, Impedance Calibration,
       DRAM Init done)*/
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x8000001f) != 0x8000001fU);

    /* wait another 32 ctl_clk cycles before resuming further traffic */
    DDR_boardDelay();

    /* wait for operating_mode status */
    while ((HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_STAT)
            & mask) != 0x1U)
    {
    /* Busy Loop */
    }
}

static void DDR_enableDQSPD()
{
    // Enable DATX8 slice DQS pull-downs and DQSN pull-ups for DDR3 mode before Write Leveling and Gate training
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL0DQSCTL, 0x012640F7 );//355ohm DQSn PU/ DQS PD, Slew 0, DQSGX=0 ( Do not extend the gate)
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL1DQSCTL, 0x012640F7 );
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL2DQSCTL, 0x012640F7 );
    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();
}

static void DDR_disableDQSPD()
{
    // Disable DATX8 slice DQS pull-downs and DQSN pull-ups for DDR3 mode after Gate training is complete.
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL0DQSCTL, 0x01264000 );
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL1DQSCTL, 0x01264000 );
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL2DQSCTL, 0x01264000 );
    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();
}

static uint8_t DDR_writeLevelingTraining()
{
    uint32_t regVal;

    /* PHY Initialization Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x00000201);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x20U) != 0x20U);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x200000)
	return 0;
    else
	return 1;
}

static uint8_t DDR_DQSGateTraining()
{
    uint32_t regVal;

    /* PHY Initialization Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x0000401);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x40U) != 0x40U);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x400000)
	return 0;
    else
	return 1;
}

static uint8_t DDR_trainDQS2DQ()
{
    uint32_t regVal;

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x00100001U); /* PHY Initialization Register */

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while ((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x8000807fU) != 0x8000807fU);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x40000U)
        return 0;
    else
	return 1;
}

static uint8_t DDR_writeLevelAdjustment()
{
    uint32_t regVal;
    /* PHY Initialization Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x0000801);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x80U) != 0x80U);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x800000)
	return 0;
    else
	return 1;
}

static uint8_t DDR_training2()
{
    uint8_t status=1;
    uint32_t regVal;

    /* Read deskew */
    /* PHY Initialization Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x0001001);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x100U) != 0x100U);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x1000000)
	status = 0;  //fail

    /* Write deskew */

    /* PHY Initialization Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x0002001);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x200U) != 0x200U);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x2000000)
	status = 0;  //fail


    /* Read Eye training */

    /* PHY Initialization Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x0004001);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x400U) != 0x400U);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x4000000)
	status = 0;  //fail

    /* Write Eye training */


    /* PHY Initialization Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x0008001);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x800U) != 0x800U);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x8000000)
	status = 0;  //fail

    return status;
}

static uint8_t DDR_VREFTraining()
{
    uint32_t regVal;

    /* VREF training */
    /* PHY Initialization Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PIR, 0x0020001);

    /* must wait 10 clock cycles before polling for init done */
    DDR_boardDelay();

    /* wait for training to be done */
    while((HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0) & 0x4000U) != 0x4000U);

    /* wait another 32 ctl_clk cycles before resuming further traffic  */
    DDR_boardDelay();

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal & 0x80000)
	return 0;
    else
	return 1;
}

static uint8_t DDR_cleanupTraining()
{
    uint32_t i;
    uint32_t dgsl0;
    uint32_t dgsl1;
    uint32_t dgsl2;
    uint32_t dgsl3;
    uint32_t rddly;
    uint32_t rd2wr_wr2rd;
    uint32_t dgsl4;

    /* update RDDLY after training has determined DGSL */
    /* Rank ID Register */
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_RANKIDR,0x00000000);
    dgsl0 = (HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX0GTR0) & 0x1F) >>  2;
    dgsl1 = (HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX1GTR0) & 0x1F) >>  2;
    dgsl2 = (HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GTR0) & 0x1F) >>  2;
    dgsl3 = (HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GTR0) & 0x1F) >>  2;
    dgsl4 = (HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GTR0) & 0x1F) >>  2;

    rddly = dgsl0;

    if(dgsl1 < rddly )
        rddly = dgsl1;
    if(dgsl2 < rddly )
        rddly = dgsl2;
    if(dgsl3 < rddly )
        rddly = dgsl3;
    if(dgsl4 < rddly )
        rddly = dgsl4;

    rddly += 5;

    /* update RDDLY based on DGSL value*/
    i = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX0GCR0);
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX0GCR0,
                  (i & ~0xF00000) | (rddly << 20));

    i = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX1GCR0);
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX1GCR0,
                  (i & ~0xF00000) | (rddly << 20));

    i = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GCR0);
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GCR0,
                  (i & ~0xF00000) | (rddly << 20));

    i = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GCR0);
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GCR0,
                  (i & ~0xF00000) | (rddly << 20));

    i = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GCR0);
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GCR0,
              (i & ~0xF00000) | (rddly << 20));

    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_RANKIDR,0x00000000);
    /* set to static read mode after training, and set PREOEX=1.5 dram clock
       during DQS preamble */
    dgsl0 = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX0GTR0);
    dgsl0 &= 0x1F;

    dgsl1 = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX1GTR0);
    dgsl1 &= 0x1F;

    dgsl2 = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX2GTR0);
    dgsl2 &= 0x1F;

    dgsl3 = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX3GTR0);
    dgsl3 &= 0x1F;

    dgsl4 = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX4GTR0);
    dgsl4 &= 0x1F;


    /* find maximum value across all bytes */
    rd2wr_wr2rd = dgsl0;
    if(dgsl1 > rd2wr_wr2rd)
        rd2wr_wr2rd = dgsl1;
    if(dgsl2 > rd2wr_wr2rd)
        rd2wr_wr2rd = dgsl2;
    if(dgsl3 > rd2wr_wr2rd)
        rd2wr_wr2rd = dgsl3;
    if(dgsl4 > rd2wr_wr2rd)
        rd2wr_wr2rd = dgsl4;

    /* divide value by 2 */
    rd2wr_wr2rd = rd2wr_wr2rd >> 1;

    /* now add in adjustment to DRAMTMG2 bit fields for rd2wr and wr2rd */
    /* SWCTL.sw_done = 0 */
    i = HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_SWCTL)
        & (~0x1);
    HW_WR_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_SWCTL, i);

    /* adjust rd2wr */
    i = HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG2)
        + (rd2wr_wr2rd << 8);
    HW_WR_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG2, i);

    i = HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG2)
        + (rd2wr_wr2rd);
    HW_WR_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_DRAMTMG2, i);

        /* SWCTL.sw_done = 1 */
    i = HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_SWCTL)
         |(0x1);
    HW_WR_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_SWCTL, i);

    /* wait until SWSTAT.sw_done= 1 */  //PR: Check
    while((HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_SWSTAT)
            & 0x1) == 0x0);

    /* Refresh Control Register 3 */
    i = HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_RFSHCTL3)
        & (~0x1);
    HW_WR_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_RFSHCTL3, i);

    /* re-enable refresh and disable PUBMODE after training is complete */
    /* disable PUBMODE */
    i = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGCR1)
        & (~0x40);
    HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGCR1, i);

    return 1;
}

/**
 * \brief DDR4 VTT regulator enable function
 *
 * This function used to enable the VTT configurations.
 *
 */
static void DDR_boardEnableVTTRegulator(void)
{
    uint32_t i;

    HW_WR_REG32(0x4301D008, 0x68EF3491 );   //lock7 kick 0
    HW_WR_REG32(0x4301D00C, 0xD172BC5A );   //lock7 kick 1

    HW_WR_REG32(0x4301C040, 0x20007);

	/* Enable output for WKUP_GPIO0_28 */
    i =  (HW_RD_REG32(0x42110010)) & (~0x10000000);
    HW_WR_REG32(0x42110010, i);

	/* Set GPIO28 output high */
    i = (HW_RD_REG32(0x42110014)) | 0x10000000;
	HW_WR_REG32(0x42110014, i);

	DDR_boardDelay();
	DDR_boardDelay();
	DDR_boardDelay();
}

static void DDR_setFreq()
{
    uint32_t module = TISCI_DEV_DDRSS0;
    uint32_t clock = TISCI_DEV_DDRSS0_BUS_DDRSS_BYP_4X_CLK;
    uint64_t freq = DDR_PLL_FREQUENCY;
    int32_t status = SystemP_SUCCESS;

    status = SOC_moduleSetClockFrequency(module, clock, freq);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("SOC_moduleSetClockFrequency failed !!!\n");
    }
}

int32_t DDR_init()
{
    uint8_t DDRType;
    uint32_t regVal;
    int32_t status = SystemP_SUCCESS;

    DDR_setFreq();
    DDR_boardEnableVTTRegulator();
    DDR_controllerPHYConfig();

    //Determine which DDR is being used
    regVal = HW_RD_REG32(CSL_DDRSS0_CTL_CFG_BASE + CSL_EMIF_CTLCFG_MSTR);
    if( (regVal & 0x1) == 0x1)
	DDRType = DDR3;
    else if( (regVal & 0x10) == 0x10)
	DDRType = DDR4;
    else if( (regVal & 0x20) == 0x20)
	DDRType = LPDDR4;
    else
	DDRType = NODDR;

    if(DDRType == DDR4) {

    //if already initialized, don't re-initialize
    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
    if(regVal==0x80004FFF)
    {
        return status;
    }
    else {
        DDR_PHYInit();
        DDR_SDRAMInit(0x7);
        //Full_Training();
            if(DDR_writeLevelingTraining())
            {
                if(DDR_DQSGateTraining())
                    if(DDR_writeLevelAdjustment())
                        if(DDR_training2())
                            if(DDR_VREFTraining())
                                DDR_cleanupTraining();
            }
    }

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
	if(regVal==0x80004FFF)
	    return status;
	else
	    return SystemP_FAILURE;
    }
    if(DDRType == DDR3)
    {
	DDR_PHYInit();
	DDR_SDRAMInit(0x3);
	//Full_Training();
	if(DDR_writeLevelingTraining())
	{
	    DDR_enableDQSPD();
	    if(DDR_DQSGateTraining())
	    {
		DDR_disableDQSPD();
		if(DDR_writeLevelAdjustment())
		    if(DDR_training2())
			DDR_cleanupTraining();
	    }
	}

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
	if(regVal==0x80000FFF)
	    return status;
	else
	    return SystemP_FAILURE;
    }
    if(DDRType == LPDDR4)
    {
	DDR_PHYInit();
	DDR_SDRAMInit(0x7);
	DDR_SDRAMInit(0x7);
	//Full_Training();
	if(DDR_writeLevelingTraining())
	{
	    DDR_enableDQSPD();
	    if(DDR_DQSGateTraining())
	    {
		DDR_disableDQSPD();
            if(DDR_trainDQS2DQ())
		    if(DDR_writeLevelAdjustment())
			if(DDR_training2())
			    if(DDR_VREFTraining())
				DDR_cleanupTraining();
	    }
	}

	HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL0DXCTL2, 0x001C1830);
	HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL1DXCTL2, 0x001C1830);
	HW_WR_DDRPHY_REG32(CSL_EMIF_PHYCFG_DX8SL2DXCTL2, 0x001C1830);

    regVal = HW_RD_DDRPHY_REG32(CSL_EMIF_PHYCFG_PGSR0);
	if(regVal==0x8000CFFF)
	    return status;
	else
	    return SystemP_FAILURE;
    }

    //if NODDR
    return SystemP_FAILURE;

}

