/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include <drivers/mpu_firewall/v0/mpu_firewall.h>

/**
 *  \brief  MPU Firewall Config
 *  This config contains the parameters for all firewalls in the SOC
 *
 */
MPU_FIREWALL_Config gMpuFirewallConfig[CSL_FW_CNT] =
{
    {
        /********************************************************************** 
                             FW R5SS0_CORE0_AXIS_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_R5SS0_CORE0_AXIS_SLV_CFG_ADDR,
        .numRegions = CSL_FW_R5SS0_CORE0_AXIS_SLV_NUM_REGION,
        .targetCount = CSL_FW_R5SS0_CORE0_AXIS_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR0,
            .regionSize = CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR1,
            .regionSize = CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE1,
        },
        {
            .startAddr = CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR2,
            .regionSize = CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE2,
        },
        {
            .startAddr = CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR3,
            .regionSize = CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE3,
        },
        {
            .startAddr = CSL_FW_R5SS0_CORE0_AXIS_SLV_START_ADDR4,
            .regionSize = CSL_FW_R5SS0_CORE0_AXIS_SLV_REGION_SIZE4,
        }
        }

    },
    {
        /********************************************************************** 
                             FW R5SS0_CORE1_AXIS_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_R5SS0_CORE1_AXIS_SLV_CFG_ADDR,
        .numRegions = CSL_FW_R5SS0_CORE1_AXIS_SLV_NUM_REGION,
        .targetCount = CSL_FW_R5SS0_CORE1_AXIS_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR0,
            .regionSize = CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR1,
            .regionSize = CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE1,
        },
        {
            .startAddr = CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR2,
            .regionSize = CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE2,
        },
        {
            .startAddr = CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR3,
            .regionSize = CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE3,
        },
        {
            .startAddr = CSL_FW_R5SS0_CORE1_AXIS_SLV_START_ADDR4,
            .regionSize = CSL_FW_R5SS0_CORE1_AXIS_SLV_REGION_SIZE4,
        }
        }

    },
    {
        /***********************************************************************
        * FW R5SS1_CORE0_AXIS_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_R5SS1_CORE0_AXIS_SLV_CFG_ADDR,
        .numRegions = CSL_FW_R5SS1_CORE0_AXIS_SLV_NUM_REGION,
        .targetCount = CSL_FW_R5SS1_CORE0_AXIS_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS1_CORE0_AXIS_SLV_START_ADDR0,
            .regionSize = CSL_FW_R5SS1_CORE0_AXIS_SLV_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_R5SS1_CORE0_AXIS_SLV_START_ADDR1,
            .regionSize = CSL_FW_R5SS1_CORE0_AXIS_SLV_REGION_SIZE1,
        },
        {
            .startAddr = CSL_FW_R5SS1_CORE0_AXIS_SLV_START_ADDR2,
            .regionSize = CSL_FW_R5SS1_CORE0_AXIS_SLV_REGION_SIZE2,
        },
        {
            .startAddr = CSL_FW_R5SS1_CORE0_AXIS_SLV_START_ADDR3,
            .regionSize = CSL_FW_R5SS1_CORE0_AXIS_SLV_REGION_SIZE3,
        },
        {
            .startAddr = CSL_FW_R5SS1_CORE0_AXIS_SLV_START_ADDR4,
            .regionSize = CSL_FW_R5SS1_CORE0_AXIS_SLV_REGION_SIZE4,
        }
        }

    },
    {
        /***********************************************************************
        * FW R5SS1_CORE1_AXIS_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_R5SS1_CORE1_AXIS_SLV_CFG_ADDR,
        .numRegions = CSL_FW_R5SS1_CORE1_AXIS_SLV_NUM_REGION,
        .targetCount = CSL_FW_R5SS1_CORE1_AXIS_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS1_CORE1_AXIS_SLV_START_ADDR0,
            .regionSize = CSL_FW_R5SS1_CORE1_AXIS_SLV_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_R5SS1_CORE1_AXIS_SLV_START_ADDR1,
            .regionSize = CSL_FW_R5SS1_CORE1_AXIS_SLV_REGION_SIZE1,
        },
        {
            .startAddr = CSL_FW_R5SS1_CORE1_AXIS_SLV_START_ADDR2,
            .regionSize = CSL_FW_R5SS1_CORE1_AXIS_SLV_REGION_SIZE2,
        },
        {
            .startAddr = CSL_FW_R5SS1_CORE1_AXIS_SLV_START_ADDR3,
            .regionSize = CSL_FW_R5SS1_CORE1_AXIS_SLV_REGION_SIZE3,
        },
        {
            .startAddr = CSL_FW_R5SS1_CORE1_AXIS_SLV_START_ADDR4,
            .regionSize = CSL_FW_R5SS1_CORE1_AXIS_SLV_REGION_SIZE4,
        }
        }

    },
    {
        /***********************************************************************
        * FW L2OCRAM_BANK0_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_L2OCRAM_BANK0_SLV_CFG_ADDR,
        .numRegions = CSL_FW_L2OCRAM_BANK0_SLV_NUM_REGION,
        .targetCount = CSL_FW_L2OCRAM_BANK0_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L2OCRAM_BANK0_SLV_START_ADDR0,
            .regionSize = CSL_FW_L2OCRAM_BANK0_SLV_REGION_SIZE0,
        }
        }

    },
    {
        /***********************************************************************
        * FW L2OCRAM_BANK1_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_L2OCRAM_BANK1_SLV_CFG_ADDR,
        .numRegions = CSL_FW_L2OCRAM_BANK1_SLV_NUM_REGION,
        .targetCount = CSL_FW_L2OCRAM_BANK1_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L2OCRAM_BANK1_SLV_START_ADDR0,
            .regionSize = CSL_FW_L2OCRAM_BANK1_SLV_REGION_SIZE0,
        }
        }

    },
    {
        /***********************************************************************
        * FW L2OCRAM_BANK2_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_L2OCRAM_BANK2_SLV_CFG_ADDR,
        .numRegions = CSL_FW_L2OCRAM_BANK2_SLV_NUM_REGION,
        .targetCount = CSL_FW_L2OCRAM_BANK2_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L2OCRAM_BANK2_SLV_START_ADDR0,
            .regionSize = CSL_FW_L2OCRAM_BANK2_SLV_REGION_SIZE0,
        }
        }

    },
    {

        /***********************************************************************
        * FW L2OCRAM_BANK3_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_L2OCRAM_BANK3_SLV_CFG_ADDR,
        .numRegions = CSL_FW_L2OCRAM_BANK3_SLV_NUM_REGION,
        .targetCount = CSL_FW_L2OCRAM_BANK3_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L2OCRAM_BANK3_SLV_START_ADDR0,
            .regionSize = CSL_FW_L2OCRAM_BANK3_SLV_REGION_SIZE0,
        }
        }

    },
    {
        /***********************************************************************
        * FW MBOX_RAM_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_MBOX_RAM_SLV_CFG_ADDR,
        .numRegions = CSL_FW_MBOX_RAM_SLV_NUM_REGION,
        .targetCount = CSL_FW_MBOX_RAM_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_MBOX_RAM_SLV_START_ADDR0,
            .regionSize = CSL_FW_MBOX_RAM_SLV_REGION_SIZE0,
        }
        }

    },
    {
        /***********************************************************************
        * FW HSM_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_HSM_SLV_CFG_ADDR,
        .numRegions = CSL_FW_HSM_SLV_NUM_REGION,
        .targetCount = CSL_FW_HSM_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_HSM_SLV_START_ADDR0,
            .regionSize = CSL_FW_HSM_SLV_REGION_SIZE0,
        },
        {
            .startAddr = CSL_FW_HSM_SLV_START_ADDR1,
            .regionSize = CSL_FW_HSM_SLV_REGION_SIZE1,
        }
        }
    },
    {

        /***********************************************************************
        * FW DTHE_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_DTHE_SLV_CFG_ADDR,
        .numRegions = CSL_FW_DTHE_SLV_NUM_REGION,
        .targetCount = CSL_FW_DTHE_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_DTHE_SLV_START_ADDR0,
            .regionSize = CSL_FW_DTHE_SLV_REGION_SIZE0,
        }
        }


    },
    {
        /***********************************************************************
        * FW OSPI0_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_OSPI0_SLV_CFG_ADDR,
        .numRegions = CSL_FW_OSPI0_SLV_NUM_REGION,
        .targetCount = CSL_FW_OSPI0_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_OSPI0_SLV_START_ADDR0,
            .regionSize = CSL_FW_OSPI0_SLV_REGION_SIZE0,
        },
        {
            .startAddr = CSL_FW_OSPI0_SLV_START_ADDR1,
            .regionSize = CSL_FW_OSPI0_SLV_REGION_SIZE1,
        },
        {
            .startAddr = CSL_FW_OSPI0_SLV_START_ADDR2,
            .regionSize = CSL_FW_OSPI0_SLV_REGION_SIZE2,
        },
        }

    },

    {
        /***********************************************************************
        * FW SCRM2SCRP0
        ***********************************************************************/

        .baseAddr = CSL_FW_SCRM2SCRP0_SLV_CFG_ADDR,
        .numRegions = CSL_FW_SCRM2SCRP0_SLV_NUM_REGION,
        .targetCount = CSL_FW_SCRM2SCRP0_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_SCRM2SCRP0_SLV_START_ADDR0,
            .regionSize = CSL_FW_SCRM2SCRP0_SLV_REGION_SIZE0,
        }
        }


    },
    {


        /***********************************************************************
         * FW SCRM2SCRP1
         ***********************************************************************/

        .baseAddr = CSL_FW_SCRM2SCRP1_SLV_CFG_ADDR,
        .numRegions = CSL_FW_SCRM2SCRP1_SLV_NUM_REGION,
        .targetCount = CSL_FW_SCRM2SCRP1_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_SCRM2SCRP1_SLV_START_ADDR0,
            .regionSize = CSL_FW_SCRM2SCRP1_SLV_REGION_SIZE0,
        }
        }

    },
    {
        /***********************************************************************
        * FW  R5SS0_CORE0_AHB_MST
        ***********************************************************************/

        .baseAddr = CSL_FW_R5SS0_CORE0_AHB_MST_CFG_ADDR,
        .numRegions = CSL_FW_R5SS0_CORE0_AHB_MST_NUM_REGION,
        .targetCount = CSL_FW_R5SS0_CORE0_AHB_MST_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS0_CORE0_AHB_MST_START_ADDR0,
            .regionSize = CSL_FW_R5SS0_CORE0_AHB_MST_REGION_SIZE0,
        }
        }

    },
    {

        /***********************************************************************
        * FW  R5SS0_CORE1_MST
        ***********************************************************************/

        .baseAddr = CSL_FW_R5SS0_CORE1_AHB_MST_CFG_ADDR,
        .numRegions = CSL_FW_R5SS0_CORE1_AHB_MST_NUM_REGION,
        .targetCount = CSL_FW_R5SS0_CORE1_AHB_MST_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS0_CORE1_AHB_MST_START_ADDR0,
            .regionSize = CSL_FW_R5SS0_CORE1_AHB_MST_REGION_SIZE0,
        }
        }

    },
    {

        /***********************************************************************
        * FW  R5SS1_CORE0_MST
        ***********************************************************************/

        .baseAddr = CSL_FW_R5SS1_CORE0_AHB_MST_CFG_ADDR,
        .numRegions = CSL_FW_R5SS1_CORE0_AHB_MST_NUM_REGION,
        .targetCount = CSL_FW_R5SS1_CORE0_AHB_MST_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS1_CORE0_AHB_MST_START_ADDR0,
            .regionSize = CSL_FW_R5SS1_CORE0_AHB_MST_REGION_SIZE0,
        }
        }
    },
    {

        /***********************************************************************
        * FW  R5SS1_CORE1_MST
        ***********************************************************************/

        .baseAddr = CSL_FW_R5SS1_CORE1_AHB_MST_CFG_ADDR,
        .numRegions = CSL_FW_R5SS1_CORE1_AHB_MST_NUM_REGION,
        .targetCount = CSL_FW_R5SS1_CORE1_AHB_MST_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS1_CORE1_AHB_MST_START_ADDR0,
            .regionSize = CSL_FW_R5SS1_CORE1_AHB_MST_REGION_SIZE0,
        }
        }
    },
        {
        /***********************************************************************
        * FW L2OCRAM_BANK4_SLV
        ***********************************************************************/
       
        .baseAddr = CSL_FW_L2OCRAM_BANK4_SLV_CFG_ADDR,
        .numRegions = CSL_FW_L2OCRAM_BANK4_SLV_NUM_REGION,
        .targetCount = CSL_FW_L2OCRAM_BANK4_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L2OCRAM_BANK4_SLV_START_ADDR0,
            .regionSize = CSL_FW_L2OCRAM_BANK4_SLV_REGION_SIZE0,
        }
        }

    },
    {
        /***********************************************************************
        * FW L2OCRAM_BANK5_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_L2OCRAM_BANK5_SLV_CFG_ADDR,
        .numRegions = CSL_FW_L2OCRAM_BANK5_SLV_NUM_REGION,
        .targetCount = CSL_FW_L2OCRAM_BANK5_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L2OCRAM_BANK5_SLV_START_ADDR0,
            .regionSize = CSL_FW_L2OCRAM_BANK5_SLV_REGION_SIZE0,
        }
        }

    },
    {
        /***********************************************************************
        * FW OSPI0_CONFIG_SLV
        ***********************************************************************/

        .baseAddr = CSL_FW_OSPI0_CFG_SLV_CFG_ADDR,
        .numRegions = CSL_FW_OSPI0_CFG_SLV_NUM_REGION,
        .targetCount = CSL_FW_OSPI0_CFG_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR0,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE0,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR1,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE1,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR2,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE2,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR3,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE3,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR4,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE4,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR5,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE5,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR6,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE6,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR7,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE7,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR8,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE8,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR9,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE9,
        },
        {
            .startAddr = CSL_FW_OSPI0_CFG_SLV_START_ADDR10,
            .regionSize = CSL_FW_OSPI0_CFG_SLV_REGION_SIZE10,
        }
        }

    },
    {

        /***********************************************************************
         * FW R5SS0_CONFIG_SLV
         ***********************************************************************/

        .baseAddr = CSL_FW_R5SS0_SLV_CFG_ADDR,
        .numRegions = CSL_FW_R5SS0_SLV_NUM_REGION,
        .targetCount = CSL_FW_R5SS0_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS0_SLV_START_ADDR0,
            .regionSize = CSL_FW_R5SS0_SLV_REGION_SIZE0,
        },
        {
            .startAddr = CSL_FW_R5SS0_SLV_START_ADDR1,
            .regionSize = CSL_FW_R5SS0_SLV_REGION_SIZE1,
        },
        {
            .startAddr = CSL_FW_R5SS0_SLV_START_ADDR2,
            .regionSize = CSL_FW_R5SS0_SLV_REGION_SIZE2,
        },
        {
            .startAddr = CSL_FW_R5SS0_SLV_START_ADDR3,
            .regionSize = CSL_FW_R5SS0_SLV_REGION_SIZE3,
        },
        {
            .startAddr = CSL_FW_R5SS0_SLV_START_ADDR4,
            .regionSize = CSL_FW_R5SS0_SLV_REGION_SIZE4,
        },
        {
            .startAddr = CSL_FW_R5SS0_SLV_START_ADDR5,
            .regionSize = CSL_FW_R5SS0_SLV_REGION_SIZE5,
        },
        {
            .startAddr = CSL_FW_R5SS0_SLV_START_ADDR6,
            .regionSize = CSL_FW_R5SS0_SLV_REGION_SIZE6,
        },
        {
            .startAddr = CSL_FW_R5SS0_SLV_START_ADDR7,
            .regionSize = CSL_FW_R5SS0_SLV_REGION_SIZE7,
        },
        }
    },
    {
        /***********************************************************************
        * FW R5SS1_CONFIG_SLV
        ***********************************************************************/
       .baseAddr = CSL_FW_R5SS1_SLV_CFG_ADDR,
        .numRegions = CSL_FW_R5SS1_SLV_NUM_REGION,
        .targetCount = CSL_FW_R5SS1_SLV_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS1_SLV_START_ADDR0,
            .regionSize = CSL_FW_R5SS1_SLV_REGION_SIZE0,
        },
        {
            .startAddr = CSL_FW_R5SS1_SLV_START_ADDR1,
            .regionSize = CSL_FW_R5SS1_SLV_REGION_SIZE1,
        },
        {
            .startAddr = CSL_FW_R5SS1_SLV_START_ADDR2,
            .regionSize = CSL_FW_R5SS1_SLV_REGION_SIZE2,
        },
        {
            .startAddr = CSL_FW_R5SS1_SLV_START_ADDR3,
            .regionSize = CSL_FW_R5SS1_SLV_REGION_SIZE3,
        },
        {
            .startAddr = CSL_FW_R5SS1_SLV_START_ADDR4,
            .regionSize = CSL_FW_R5SS1_SLV_REGION_SIZE4,
        },
        {
            .startAddr = CSL_FW_R5SS1_SLV_START_ADDR5,
            .regionSize = CSL_FW_R5SS1_SLV_REGION_SIZE5,
        },
        {
            .startAddr = CSL_FW_R5SS1_SLV_START_ADDR6,
            .regionSize = CSL_FW_R5SS1_SLV_REGION_SIZE6,
        },
        {
            .startAddr = CSL_FW_R5SS1_SLV_START_ADDR7,
            .regionSize = CSL_FW_R5SS1_SLV_REGION_SIZE7,
        },
        }
    }
    
};
