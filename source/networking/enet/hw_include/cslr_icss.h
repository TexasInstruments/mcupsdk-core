/********************************************************************
 * Copyright (C) 2013-2019 Texas Instruments Incorporated.
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
#ifndef CSLR_ICSS_H_
#define CSLR_ICSS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>
#if defined(SOC_AM574x) || defined(SOC_AM572x)
#include <icss/V1/cslr_icss_cfg.h>
#include <icss/V1/cslr_icss_ecap.h>
#include <icss/V1/cslr_icss_iep.h>
#include <icss/V1/cslr_icss_intc.h>
#include <icss/V1/cslr_icss_mii_mdio.h>
#include <icss/V1/cslr_icss_pru_ctrl.h>
#include <icss/V1/cslr_icss_pru_debug.h>
#include <icss/V1/cslr_icss_uart.h>
#include <icss/V0/cslr_icssm_cfg.h>
#include <icss/V0/cslr_icssm_ecap.h>
#include <icss/V0/cslr_icssm_iep.h>
#include <icss/V0/cslr_icssm_intc.h>
#include <icss/V0/cslr_icssm_mii_mdio.h>
#include <icss/V0/cslr_icssm_mii_rt_cfg.h>
#include <icss/V0/cslr_icssm_pru_ctrl.h>
#include <icss/V0/cslr_icssm_pru_debug.h>
#include <icss/V0/cslr_icssm_uart.h>
#include <icss/V0/cslr_icss_mii_rt.h>
#elif defined(SOC_AM571x) || defined(SOC_K2G)
#include <icss/V1/cslr_icss_cfg.h>
#include <icss/V1/cslr_icss_ecap.h>
#include <icss/V1/cslr_icss_iep.h>
#include <icss/V1/cslr_icss_intc.h>
#include <icss/V1/cslr_icss_mii_mdio.h>
#include <icss/V1/cslr_icss_pru_ctrl.h>
#include <icss/V1/cslr_icss_pru_debug.h>
#include <icss/V1/cslr_icss_uart.h>
#include <icss/V1/cslr_icss_mii_rt.h>
#elif defined(SOC_AM335x) || defined(SOC_AM437x)
#include <icss/V0/cslr_icssm_cfg.h>
#include <icss/V0/cslr_icssm_ecap.h>
#include <icss/V0/cslr_icssm_iep.h>
#include <icss/V0/cslr_icssm_intc.h>
#include <icss/V0/cslr_icssm_mii_mdio.h>
#include <icss/V0/cslr_icssm_mii_rt_cfg.h>
#include <icss/V0/cslr_icssm_pru_ctrl.h>
#include <icss/V0/cslr_icssm_pru_debug.h>
#include <icss/V0/cslr_icssm_uart.h>
#elif defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
/*  PRUSS CFG and INTC registers of OMAPL137 are same as that of K2G PRU-ICSS.
    Note that there is no ICSS core on OMAPL137 but low level register macros
    may have ICSS tag as we are re-using them from K2G platform */
#include <icss/V1/cslr_icss_intc.h>
#include <icss/V1/cslr_icss_pru_ctrl.h>
#elif defined(SOC_AM65XX) || defined (SOC_J721E)
#include <icss/V2/cslr_icss_g.h>
#include <icss/V1/cslr_icss_ecap.h>
#include <icss/V1/cslr_icss_iep.h>
#include <icss/V1/cslr_icss_intc.h>
#include <icss/V1/cslr_icss_mii_mdio.h>
#include <icss/V1/cslr_icss_mii_rt.h>
#include <icss/V1/cslr_icss_pru_ctrl.h>
#include <icss/V1/cslr_icss_pru_debug.h>
#include <icss/V1/cslr_icss_uart.h>
#elif defined(SOC_AM64X) || defined(SOC_AM243X)
#include <icss/V3/cslr_icss_g.h>
#include <icss/V1/cslr_icss_ecap.h>
#include <icss/V1/cslr_icss_iep.h>
#include <icss/V1/cslr_icss_intc.h>
#include <icss/V1/cslr_icss_mii_mdio.h>
#include <icss/V1/cslr_icss_mii_rt.h>
#include <icss/V1/cslr_icss_pru_ctrl.h>
#include <icss/V1/cslr_icss_pru_debug.h>
#include <icss/V1/cslr_icss_uart.h>
#endif /* SOC_XXXXX */

#ifdef __cplusplus
}
#endif

#endif
