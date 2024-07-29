/*
 *  Copyright (C) 2019-2020 Texas Instruments Incorporated
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

#ifndef CSLR_SOC_IN_H
#define CSLR_SOC_IN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Memory map header files */
#include <drivers/hw_include/am273x/cslr_soc_defines.h>
#include <drivers/hw_include/am273x/cslr_soc_baseaddress.h>
#include <drivers/hw_include/am273x/cslr_soc_dsp_baseaddress.h>
#include <drivers/hw_include/am273x/cslr_soc_r5_baseaddress.h>

/* ECC aggr header files */
#include <drivers/hw_include/am273x/cslr_mss_ecc_agg_mss.h>
#include <drivers/hw_include/am273x/cslr_mss_ecc_agga.h>
#include <drivers/hw_include/am273x/cslr_mss_ecc_aggb.h>
#include <drivers/hw_include/am273x/cslr_dss_ecc_agg.h>

/* Interrupt map header files */
#include <drivers/hw_include/am273x/cslr_intr_mss.h>
#include <drivers/hw_include/am273x/cslr_intr_dss.h>
#include <drivers/hw_include/am273x/cslr_intr_esm_mss.h>
#include <drivers/hw_include/am273x/cslr_intr_esm_dss.h>

/* Interrupt control header files */
#include <drivers/hw_include/am273x/cslr_mss_vim.h>

/* Pinmux control header files */
#include <drivers/hw_include/am273x/cslr_mss_iomux.h>

/* PSC, LPSC indices header file */
#include <drivers/hw_include/am273x/cslr_mss_toprcm.h>
#include <drivers/hw_include/am273x/cslr_mss_rcm.h>
#include <drivers/hw_include/am273x/cslr_dss_rcm.h>
#include <drivers/hw_include/am273x/cslr_rcss_rcm.h>

/* PBIST header file */
#include <drivers/hw_include/am273x/cslr_pbist.h>

/* Firewall defines header file */
#include <drivers/hw_include/am273x/cslr_firewall_defines.h>

/* System control register header files */
#include <drivers/hw_include/am273x/cslr_top_ctrl.h>
#include <drivers/hw_include/am273x/cslr_mss_ctrl.h>
#include <drivers/hw_include/am273x/cslr_dss_ctrl.h>
#include <drivers/hw_include/am273x/cslr_rcss_ctrl.h>
#include <drivers/hw_include/am273x/cslr_hsm_ctrl.h>
#include <drivers/hw_include/am273x/cslr_hsm_soc_ctrl.h>
#include <drivers/hw_include/am273x/cslr_top_anareg.h>

/* Hsm base address header file */
#include <drivers/hw_include/am273x/cslr_soc_hsm_baseaddress.h>

/* EFuse control register header files */
#include <drivers/hw_include/am273x/cslr_efuse_farm.h>

#ifdef __cplusplus
}
#endif


#endif /* CSLR_SOC_IN_H */
