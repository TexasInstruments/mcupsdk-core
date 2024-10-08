/*
 *  Copyright (C) 2020-24 Texas Instruments Incorporated
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
#ifndef CSLR_SOC_IN_H_
#define CSLR_SOC_IN_H_


#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/am261x/cslr_controlss_ctrl.h>
#include <drivers/hw_include/am261x/cslr_controlss_dmaxbar.h>
#include <drivers/hw_include/am261x/cslr_controlss_iclxbar.h>
#include <drivers/hw_include/am261x/cslr_controlss_inputxbar.h>
#include <drivers/hw_include/am261x/cslr_controlss_intxbar.h>
#include <drivers/hw_include/am261x/cslr_controlss_mdlxbar.h>
#include <drivers/hw_include/am261x/cslr_controlss_outputxbar.h>
#include <drivers/hw_include/am261x/cslr_controlss_pwmsyncoutxbar.h>
#include <drivers/hw_include/am261x/cslr_controlss_pwmxbar.h>
#include <drivers/hw_include/am261x/cslr_ecc_aggr.h>
#include <drivers/hw_include/am261x/cslr_edma_trig_xbar.h>
#include <drivers/hw_include/am261x/cslr_efuse_farm.h>
#include <drivers/hw_include/am261x/cslr_ext_flash.h>
#include <drivers/hw_include/am261x/cslr_firewall_defines.h>
#include <drivers/hw_include/am261x/cslr_gpio_intr_xbar.h>
#include <drivers/hw_include/am261x/cslr_icssm_intr_xbar.h>
#include <drivers/hw_include/am261x/cslr_intr_r5fss0_core0.h>
#include <drivers/hw_include/am261x/cslr_intr_r5fss0_core1.h>
#include <drivers/hw_include/am261x/cslr_intr_tpcc0_dmatrigger_xbar.h>
#include <drivers/hw_include/am261x/cslr_iomux.h>
#include <drivers/hw_include/am261x/cslr_mss_ctrl.h>
#include <drivers/hw_include/am261x/cslr_mss_debugss.h>
#include <drivers/hw_include/am261x/cslr_mss_ecc_agga.h>
#include <drivers/hw_include/am261x/cslr_mss_ecc_aggb.h>
#include <drivers/hw_include/am261x/cslr_mss_ecc_agg_mss.h>
#include <drivers/hw_include/am261x/cslr_mss_esm.h>
#include <drivers/hw_include/am261x/cslr_mss_l2.h>
#include <drivers/hw_include/am261x/cslr_mss_mbox.h>
#include <drivers/hw_include/am261x/cslr_mss_mcan_ecc.h>
#include <drivers/hw_include/am261x/cslr_mss_mcan_msg_ram.h>
#include <drivers/hw_include/am261x/cslr_mss_mcrc.h>
#include <drivers/hw_include/am261x/cslr_mss_rcm.h>
#include <drivers/hw_include/am261x/cslr_mss_tcma_cr5a.h>
#include <drivers/hw_include/am261x/cslr_mss_tcma_rom.h>
#include <drivers/hw_include/am261x/cslr_mss_tcmb_cr5a.h>
#include <drivers/hw_include/am261x/cslr_mss_tcm.h>
#include <drivers/hw_include/am261x/cslr_mss_vim.h>
#include <drivers/hw_include/am261x/cslr_pbist.h>
#include <drivers/hw_include/am261x/cslr_rl2_of_r5fss0_core0.h>
#include <drivers/hw_include/am261x/cslr_rl2_of_r5fss0_core1.h>
#include <drivers/hw_include/am261x/cslr_soc_baseaddress.h>
#include <drivers/hw_include/am261x/cslr_soc_defines.h>
#include <drivers/hw_include/am261x/cslr_soc_hsm_baseaddress.h>
#include <drivers/hw_include/am261x/cslr_soc_r5_baseaddress.h>
#include <drivers/hw_include/am261x/cslr_soc_r5_lockstep_baseaddress.h>
#include <drivers/hw_include/am261x/cslr_soc_r5_rom_baseaddress.h>
#include <drivers/hw_include/am261x/cslr_soc_timesync_xbar0.h>
#include <drivers/hw_include/am261x/cslr_soc_timesync_xbar1.h>
#include <drivers/hw_include/am261x/cslr_stc.h>
#include <drivers/hw_include/am261x/cslr_tmu.h>
#include <drivers/hw_include/am261x/cslr_tmu_rom.h>
#include <drivers/hw_include/am261x/cslr_top_ctrl.h>
#include <drivers/hw_include/am261x/cslr_top_rcm.h>
#include <drivers/hw_include/am261x/soc_config.h>

#ifdef __cplusplus
}
#endif
#endif /* CSLR_SOC_IN_H_ */
