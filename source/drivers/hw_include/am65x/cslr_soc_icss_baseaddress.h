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
*/
#ifndef CSLR_ICSS_BASEADDRESS_H_
#define CSLR_ICSS_BASEADDRESS_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#ifdef __cplusplus
extern "C"
{
#endif

#define CSL_ICSS_DRAM0_BASE                                                                            (0x0U)     // ICSS_dram0
#define CSL_ICSS_DRAM0_SIZE                                                                            (0x2000U)
#define CSL_ICSS_DRAM1_BASE                                                                            (0x2000U)  // ICSS_dram1
#define CSL_ICSS_DRAM1_SIZE                                                                            (0x2000U)
#define CSL_ICSS_RTU0_IRAM_BASE                                                                        (0x4000U)  // ICSS_rtu0_iram
#define CSL_ICSS_RTU0_IRAM_SIZE                                                                        (0x2000U)
#define CSL_ICSS_RTU1_IRAM_BASE                                                                        (0x6000U)  // ICSS_rtu1_iram
#define CSL_ICSS_RTU1_IRAM_SIZE                                                                        (0x2000U)
#define CSL_ICSS_RAT_SLICE0_CFG_BASE                                                                   (0x8000U)  // ICSS_rat_slice0
#define CSL_ICSS_RAT_SLICE0_CFG_SIZE                                                                   (0x1000U)
#define CSL_ICSS_RAT_SLICE1_CFG_BASE                                                                   (0x9000U)  // ICSS_rat_slice0
#define CSL_ICSS_RAT_SLICE1_CFG_SIZE                                                                   (0x1000U)
#define CSL_ICSS_TX0_IRAM_BASE                                                                         (0xA000U)  // ICSS_tx0_iram
#define CSL_ICSS_TX0_IRAM_SIZE                                                                         (0x1800U)
#define CSL_ICSS_TX1_IRAM_BASE                                                                         (0xC000U)  // ICSS_tx1_iram
#define CSL_ICSS_TX1_IRAM_SIZE                                                                         (0x1800U)
#define CSL_ICSS_RAM_BASE                                                                              (0x10000U)  // ICSS_ram
#define CSL_ICSS_RAM_SIZE                                                                              (0x10000U)
#define CSL_ICSS_INTC_CFG_BASE                                                                         (0x20000U)  // ICSS_intc_cfg
#define CSL_ICSS_INTC_CFG_SIZE                                                                         (0x2000U)
#define CSL_ICSS_PDSP0_IRAM_CFG_BASE                                                                   (0x22000U)  // ICSS_pdsp0_iram_cfg
#define CSL_ICSS_PDSP0_IRAM_CFG_SIZE                                                                   (0x100U)
#define CSL_ICSS_PDSP0_IRAM_DBG_BASE                                                                   (0x22400U)  // ICSS_pdsp0_iram_dbg
#define CSL_ICSS_PDSP0_IRAM_DBG_SIZE                                                                   (0x100U)
#define CSL_ICSS_RTU0_IRAM_CFG_BASE                                                                    (0x23000U)  // ICSS_rtu0_iram_cfg
#define CSL_ICSS_RTU0_IRAM_CFG_SIZE                                                                    (0x100U)
#define CSL_ICSS_RTU0_IRAM_DBG_BASE                                                                    (0x23400U)  // ICSS_rtu0_iram_dbg
#define CSL_ICSS_RTU0_IRAM_DBG_SIZE                                                                    (0x100U)
#define CSL_ICSS_RTU1_IRAM_CFG_BASE                                                                    (0x23800U)  // ICSS_rtu1_iram_cfg
#define CSL_ICSS_RTU1_IRAM_CFG_SIZE                                                                    (0x100U)
#define CSL_ICSS_RTU1_IRAM_DBG_BASE                                                                    (0x23c00U)  // ICSS_rtu1_iram_dbg
#define CSL_ICSS_RTU1_IRAM_DBG_SIZE                                                                    (0x100U)
#define CSL_ICSS_PDSP1_IRAM_CFG_BASE                                                                   (0x24000U)  // ICSS_pdsp1_iram_cfg
#define CSL_ICSS_PDSP1_IRAM_CFG_SIZE                                                                   (0x100U)
#define CSL_ICSS_PDSP1_IRAM_DBG_BASE                                                                   (0x24400U)  // ICSS_pdsp1_iram_dbg
#define CSL_ICSS_PDSP1_IRAM_DBG_SIZE                                                                   (0x100U)
#define CSL_ICSS_PROT_BASE                                                                             (0x24C00U)  // ICSS_prot
#define CSL_ICSS_PROT_SIZE                                                                             (0x100U)
#define CSL_ICSS_TX0_IRAM_CFG_BASE                                                                     (0x25000U)  // ICSS_tx0_iram_cfg
#define CSL_ICSS_TX0_IRAM_CFG_SIZE                                                                     (0x100U)
#define CSL_ICSS_TX0_IRAM_DBG_BASE                                                                     (0x25400U)  // ICSS_tx0_iram_dbg
#define CSL_ICSS_TX0_IRAM_DBG_SIZE                                                                     (0x100U)
#define CSL_ICSS_TX1_IRAM_CFG_BASE                                                                     (0x25800U)  // ICSS_tx0_iram_cfg
#define CSL_ICSS_TX1_IRAM_CFG_SIZE                                                                     (0x100U)
#define CSL_ICSS_TX1_IRAM_DBG_BASE                                                                     (0x25C00U)  // ICSS_tx0_iram_dbg
#define CSL_ICSS_TX1_IRAM_DBG_SIZE                                                                     (0x100U)
#define CSL_ICSS_CFG_BASE                                                                              (0x26000U)  // ICSS_cfg
#define CSL_ICSS_CFG_SIZE                                                                              (0x200U)
#define CSL_ICSS_PA_STATS_QRAM_BASE                                                                    (0x27000U)  // ICSS_pa_stats_qram
#define CSL_ICSS_PA_STATS_QRAM_SIZE                                                                    (0x1000U)
#define CSL_ICSS_UART_CFG_BASE                                                                         (0x28000U)  // ICSS_uart_cfg
#define CSL_ICSS_UART_CFG_SIZE                                                                         (0x40U)
#define CSL_ICSS_TASKS_MGR_PRU0_CFG_BASE                                                               (0x2a000U)  // ICSS_tasks_mgr_pru0_cfg
#define CSL_ICSS_TASKS_MGR_PRU0_CFG_SIZE                                                               (0x100U)
#define CSL_ICSS_TASKS_MGR_RTU0_CFG_BASE                                                               (0x2a100U)  // ICSS_tasks_mgr_rtu0_cfg
#define CSL_ICSS_TASKS_MGR_RTU0_CFG_SIZE                                                               (0x100U)
#define CSL_ICSS_TASKS_MGR_PRU1_CFG_BASE                                                               (0x2a200U)  // ICSS_tasks_mgr_pru1_cfg
#define CSL_ICSS_TASKS_MGR_PRU1_CFG_SIZE                                                               (0x100U)
#define CSL_ICSS_TASKS_MGR_RTU1_CFG_BASE                                                               (0x2a300U)  // ICSS_tasks_mgr_rtu1_cfg
#define CSL_ICSS_TASKS_MGR_RTU1_CFG_SIZE                                                               (0x100U)
#define CSL_ICSS_TASKS_MGR_TX0_CFG_BASE                                                                (0x2a400U)  // ICSS_tasks_mgr_tx0_cfg
#define CSL_ICSS_TASKS_MGR_TX0_CFG_SIZE                                                                (0x100U)
#define CSL_ICSS_TASKS_MGR_TX1_CFG_BASE                                                                (0x2a500U)  // ICSS_tasks_mgr_tx1_cfg
#define CSL_ICSS_TASKS_MGR_TX1_CFG_SIZE                                                                (0x100U)
#define CSL_ICSS_PA_STATS_CRAM_BASE                                                                    (0x2C000U)  // ICSS_pa_stats_cram
#define CSL_ICSS_PA_STATS_CRAM_SIZE                                                                    (0x1000U)
#define CSL_ICSS_IEP0_CFG_BASE                                                                         (0x2e000U)  // ICSS_iep0_cfg
#define CSL_ICSS_IEP0_CFG_SIZE                                                                         (0x1000U)
#define CSL_ICSS_IEP1_CFG_BASE                                                                         (0x2f000U)  // ICSS_iep1_cfg
#define CSL_ICSS_IEP1_CFG_SIZE                                                                         (0x1000U)
#define CSL_ICSS_ECAP_CFG_BASE                                                                         (0x30000U)  // ICSS_ecap_cfg
#define CSL_ICSS_ECAP_CFG_SIZE                                                                         (0x100U)
#define CSL_ICSS_MII_RT_CFG_BASE                                                                       (0x32000U)  // ICSS_mii_rt_cfg
#define CSL_ICSS_MII_RT_CFG_SIZE                                                                       (0x100U)
#define CSL_ICSS_SGMII0_CFG_BASE                                                                       (0x32100U)  // ICSS_sgmii0_cfg
#define CSL_ICSS_SGMII0_CFG_SIZE                                                                       (0x100U)
#define CSL_ICSS_SGMII1_CFG_BASE                                                                       (0x32200U)  // ICSS_sgmii1_cfg
#define CSL_ICSS_SGMII1_CFG_SIZE                                                                       (0x100U)
#define CSL_ICSS_MDIO_CFG_BASE                                                                         (0x32400U)  // ICSS_mdio_cfg
#define CSL_ICSS_MDIO_CFG_SIZE                                                                         (0x100U)
#define CSL_ICSS_MII_RT_G_CFG_BASE                                                                     (0x33000U)  // ICSS_mii_rt_g_cfg
#define CSL_ICSS_MII_RT_G_CFG_SIZE                                                                     (0x800U)
#define CSL_ICSS_PDSP0_IRAM_BASE                                                                       (0x34000U)  // ICSS_pdsp0_iram
#define CSL_ICSS_PDSP0_IRAM_SIZE                                                                       (0x4000U)
#define CSL_ICSS_PDSP1_IRAM_BASE                                                                       (0x38000U)  // ICSS_pdsp1_iram
#define CSL_ICSS_PDSP1_IRAM_SIZE                                                                       (0x4000U)
#define CSL_ICSS_PA_STATS_CFG_BASE                                                                     (0x3C000U)  // ICSS_pa_stats_cfg
#define CSL_ICSS_PA_STATS_CFG_SIZE                                                                     (0x1000U)
#define CSL_ICSS_RAT_REGION0_BASE                                                                      (0x40000U)  // ICSS_rat_region0
#define CSL_ICSS_RAT_REGION0_SIZE                                                                      (0xfffc0000U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_ICSS_BASEADDRESS_H_ */
