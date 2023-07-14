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
        .baseAddr = CSL_FW_L2_BANKA_CFG_ADDR,
        .numRegions = CSL_FW_L2_BANKA_NUM_REGION,
        .targetCount = CSL_FW_L2_BANKA_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L2_BANKA_START_ADDR0,
            .regionSize = CSL_FW_L2_BANKA_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_L2_BANKB_CFG_ADDR,
        .numRegions = CSL_FW_L2_BANKB_NUM_REGION,
        .targetCount = CSL_FW_L2_BANKB_NUM_REGION,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L2_BANKB_START_ADDR0,
            .regionSize = CSL_FW_L2_BANKB_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_HSM_DTHE_CFG_ADDR,
        .numRegions = CSL_FW_HSM_DTHE_NUM_REGION,
        .targetCount = CSL_FW_HSM_DTHE_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_HSM_DTHE_START_ADDR0,
            .regionSize = CSL_FW_HSM_DTHE_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_MSS_MBOX_CFG_ADDR,
        .numRegions = CSL_FW_MSS_MBOX_NUM_REGION,
        .targetCount = CSL_FW_MSS_MBOX_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_MSS_MBOX_START_ADDR0,
            .regionSize = CSL_FW_MSS_MBOX_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_MSS_PCRA_CFG_ADDR,
        .numRegions = CSL_FW_MSS_PCRA_NUM_REGION,
        .targetCount = CSL_FW_MSS_PCRA_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_MSS_PCRA_START_ADDR0,
            .regionSize = CSL_FW_MSS_PCRA_REGION_SIZE0
        }
        }

    },
    {
        .baseAddr = CSL_FW_QSPI0_CFG_ADDR,
        .numRegions = CSL_FW_QSPI0_NUM_REGION,
        .targetCount = CSL_FW_QSPI0_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_QSPI0_START_ADDR0,
            .regionSize = CSL_FW_QSPI0_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_QSPI0_START_ADDR1,
            .regionSize = CSL_FW_QSPI0_REGION_SIZE1
        }
        }
    },
    {
        .baseAddr = CSL_FW_R5SS_COREA_AXIS_CFG_ADDR,
        .numRegions = CSL_FW_R5SS_COREA_AXIS_NUM_REGION,
        .targetCount = CSL_FW_R5SS_COREA_AXIS_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS_COREA_AXIS_START_ADDR0,
            .regionSize = CSL_FW_R5SS_COREA_AXIS_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_R5SS_COREA_AXIS_START_ADDR1,
            .regionSize = CSL_FW_R5SS_COREA_AXIS_REGION_SIZE1
        },
        {
            .startAddr = CSL_FW_R5SS_COREA_AXIS_START_ADDR2,
            .regionSize = CSL_FW_R5SS_COREA_AXIS_REGION_SIZE2
        },
        {
            .startAddr = CSL_FW_R5SS_COREA_AXIS_START_ADDR3,
            .regionSize = CSL_FW_R5SS_COREA_AXIS_REGION_SIZE3
        }
        }
    },
    {
        .baseAddr = CSL_FW_R5SS_COREB_AXIS_CFG_ADDR,
        .numRegions = CSL_FW_R5SS_COREB_AXIS_NUM_REGION,
        .targetCount = CSL_FW_R5SS_COREB_AXIS_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_R5SS_COREB_AXIS_START_ADDR0,
            .regionSize = CSL_FW_R5SS_COREB_AXIS_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_R5SS_COREB_AXIS_START_ADDR1,
            .regionSize = CSL_FW_R5SS_COREB_AXIS_REGION_SIZE1
        },
        {
            .startAddr = CSL_FW_R5SS_COREB_AXIS_START_ADDR2,
            .regionSize = CSL_FW_R5SS_COREB_AXIS_REGION_SIZE2
        },
        {
            .startAddr = CSL_FW_R5SS_COREB_AXIS_START_ADDR3,
            .regionSize = CSL_FW_R5SS_COREB_AXIS_REGION_SIZE3
        }
        }
    },
    {
        .baseAddr = CSL_FW_L3_BANKA_CFG_ADDR,
        .numRegions = CSL_FW_L3_BANKA_NUM_REGION,
        .targetCount = CSL_FW_L3_BANKA_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L3_BANKA_START_ADDR0,
            .regionSize = CSL_FW_L3_BANKA_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_L3_BANKB_CFG_ADDR,
        .numRegions = CSL_FW_L3_BANKB_NUM_REGION,
        .targetCount = CSL_FW_L3_BANKB_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L3_BANKB_START_ADDR0,
            .regionSize = CSL_FW_L3_BANKB_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_L3_BANKC_CFG_ADDR,
        .numRegions = CSL_FW_L3_BANKC_NUM_REGION,
        .targetCount = CSL_FW_L3_BANKC_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L3_BANKC_START_ADDR0,
            .regionSize = CSL_FW_L3_BANKC_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_L3_BANKD_CFG_ADDR,
        .numRegions = CSL_FW_L3_BANKD_NUM_REGION,
        .targetCount = CSL_FW_L3_BANKD_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_L3_BANKD_START_ADDR0,
            .regionSize = CSL_FW_L3_BANKD_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_HWA_DMA0_CFG_ADDR,
        .numRegions = CSL_FW_HWA_DMA0_NUM_REGION,
        .targetCount = CSL_FW_HWA_DMA0_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_HWA_DMA0_START_ADDR0,
            .regionSize = CSL_FW_HWA_DMA0_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_HWA_DMA1_CFG_ADDR,
        .numRegions = CSL_FW_HWA_DMA1_NUM_REGION,
        .targetCount = CSL_FW_HWA_DMA1_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_HWA_DMA1_START_ADDR0,
            .regionSize = CSL_FW_HWA_DMA1_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_DSS_HWA_PROC_CFG_ADDR,
        .numRegions = CSL_FW_DSS_HWA_PROC_NUM_REGION,
        .targetCount = CSL_FW_DSS_HWA_PROC_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_DSS_HWA_PROC_START_ADDR0,
            .regionSize = CSL_FW_DSS_HWA_PROC_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_DSS_HWA_PROC_START_ADDR1,
            .regionSize = CSL_FW_DSS_HWA_PROC_REGION_SIZE1
        }
        }
    },
    {
        .baseAddr = CSL_FW_DSS_MBOX_CFG_ADDR,
        .numRegions = CSL_FW_DSS_MBOX_NUM_REGION,
        .targetCount = CSL_FW_DSS_MBOX_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_DSS_MBOX_START_ADDR0,
            .regionSize = CSL_FW_DSS_MBOX_REGION_SIZE0
        }
        }
    },
    {
        .baseAddr = CSL_FW_HSM_CFG_ADDR,
        .numRegions = CSL_FW_HSM_NUM_REGION,
        .targetCount = CSL_FW_HSM_NUM_PROTECTED,
        .target = (Firewall_Target[])
        {
        {
            .startAddr = CSL_FW_HSM_START_ADDR0,
            .regionSize = CSL_FW_HSM_REGION_SIZE0
        },
        {
            .startAddr = CSL_FW_HSM_START_ADDR1,
            .regionSize = CSL_FW_HSM_REGION_SIZE1
        }
        }
    },
};
