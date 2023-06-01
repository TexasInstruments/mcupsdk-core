/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/pruicss.h>
#include <drivers/hw_include/cslr_soc.h>

PRUICSS_HwAttrs const gPruIcssHwAttrs_ICSSM0 =
{
    0,                                                                                      /* instance */
    CSL_ICSSM0_INTERNAL_U_BASE,                                                             /* baseAddr */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_PDSP0_IRAM_REGS_BASE,                       /* pru0CtrlRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_PDSP1_IRAM_REGS_BASE,                       /* pru1CtrlRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_ICSS_INTC_SLV_REGS_BASE,                    /* intcRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_CFG_SLV_REGS_BASE,                          /* cfgRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_ICSS_UART_SLV_REGS_BASE,                    /* uartRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_IEP0_SLV_REGS_BASE,                         /* iep0RegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_ICSS_ECAP0_ECAP_SLV_REGS_BASE,              /* ecapRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_MII_RT_PR1_MII_RT_CFG_REGS_BASE,            /* miiRtCfgRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_REGS_BASE,   /* miiGRtCfgRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_MDIO_V1P7_MDIO_REGS_BASE,                   /* miiMdioRegBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_DRAM0_SLV_RAM_REGS_BASE,                        /* pru0DramBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_DRAM1_SLV_RAM_REGS_BASE,                        /* pru1DramBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_PDSP0_IRAM_RAM_REGS_BASE,                   /* pru0IramBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_PDSP1_IRAM_RAM_REGS_BASE,                   /* pru1IramBase */
    CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_RAM_SLV_RAM_REGS_BASE,                          /* sharedDramBase */
    /*TODO: Replace these values with macros*/
    (0x2000UL),                                                                             /* pru0DramSize */
    (0x2000UL),                                                                             /* pru1DramSize */
    (0x3000UL),                                                                             /* pru0IramSize */
    (0x3000UL),                                                                             /* pru1IramSize */
    (0x8000UL),                                                                             /* sharedDramSize */
};
