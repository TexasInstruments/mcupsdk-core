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

PRUICSS_HwAttrs const gPruIcssHwAttrs_ICSSG0 =
{
    0,                                                      /* instance */
    CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE,                      /* baseAddr */
    CSL_PRU_ICSSG0_PR1_PDSP0_IRAM_BASE,                     /* pru0CtrlRegBase */
    CSL_PRU_ICSSG0_PR1_PDSP1_IRAM_BASE,                     /* pru1CtrlRegBase */
    CSL_PRU_ICSSG0_PR1_ICSS_INTC_INTC_SLV_BASE,             /* intcRegBase */
    CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE,                        /* cfgRegBase */
    CSL_PRU_ICSSG0_PR1_ICSS_UART_UART_SLV_BASE,             /* uartRegBase */
    CSL_PRU_ICSSG0_IEP0_BASE,                               /* iep0RegBase */
    CSL_PRU_ICSSG0_IEP1_BASE,                               /* iep1RegBase */
    CSL_PRU_ICSSG0_PR1_ICSS_ECAP0_ECAP_SLV_BASE,            /* ecapRegBase */
    CSL_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_CFG_BASE,          /* miiRtCfgRegBase */
    CSL_PRU_ICSSG0_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_BASE, /* miiGRtCfgRegBase */
    CSL_PRU_ICSSG0_PR1_MDIO_V1P7_MDIO_BASE,                 /* miiMdioRegBase */
    CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE,                      /* pru0DramBase */
    CSL_PRU_ICSSG0_DRAM1_SLV_RAM_BASE,                      /* pru1DramBase */
    CSL_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_BASE,                 /* pru0IramBase */
    CSL_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_BASE,                 /* pru1IramBase */
    CSL_PRU_ICSSG0_RAM_SLV_RAM_BASE,                        /* sharedDramBase */
    CSL_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_BASE,         /* rtu0IramBase */
    CSL_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_BASE,         /* rtu1IramBase */
    CSL_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_BASE,             /* rtu0CtrlRegBase */
    CSL_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_BASE,             /* rtu1CtrlRegBase */
    CSL_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_BASE,                  /* txPru0CtrlRegBase */
    CSL_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_BASE,                  /* txPru1CtrlRegBase */
    CSL_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_BASE,              /* txPru0IramBase */
    CSL_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_BASE,              /* txPru1IramBase */
    CSL_PRU_ICSSG0_DRAM0_SLV_RAM_SIZE,                      /* pru0DramSize */
    CSL_PRU_ICSSG0_DRAM1_SLV_RAM_SIZE,                      /* pru1DramSize */
    CSL_PRU_ICSSG0_PR1_PDSP0_IRAM_RAM_SIZE,                 /* pru0IramSize */
    CSL_PRU_ICSSG0_PR1_PDSP1_IRAM_RAM_SIZE,                 /* pru1IramSize */
    CSL_PRU_ICSSG0_RAM_SLV_RAM_SIZE,                        /* sharedDramSize */
    CSL_PRU_ICSSG0_PR1_RTU0_PR1_RTU0_IRAM_RAM_SIZE,         /* rtu0IramSize */
    CSL_PRU_ICSSG0_PR1_RTU1_PR1_RTU1_IRAM_RAM_SIZE,         /* rtu1IramSize */
    CSL_PRU_ICSSG0_PR1_PDSP_TX0_IRAM_RAM_SIZE,              /* txPru0IramSize */
    CSL_PRU_ICSSG0_PR1_PDSP_TX1_IRAM_RAM_SIZE,              /* txPru1IramSize */
};

PRUICSS_HwAttrs const gPruIcssHwAttrs_ICSSG1 =
{
    1,                                                      /* instance */
    CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE,                      /* baseAddr */
    CSL_PRU_ICSSG1_PR1_PDSP0_IRAM_BASE,                     /* pru0CtrlRegBase */
    CSL_PRU_ICSSG1_PR1_PDSP1_IRAM_BASE,                     /* pru1CtrlRegBase */
    CSL_PRU_ICSSG1_PR1_ICSS_INTC_INTC_SLV_BASE,             /* intcRegBase */
    CSL_PRU_ICSSG1_PR1_CFG_SLV_BASE,                        /* cfgRegBase */
    CSL_PRU_ICSSG1_PR1_ICSS_UART_UART_SLV_BASE,             /* uartRegBase */
    CSL_PRU_ICSSG1_IEP0_BASE,                               /* iep0RegBase */
    CSL_PRU_ICSSG1_IEP1_BASE,                               /* iep1RegBase */
    CSL_PRU_ICSSG1_PR1_ICSS_ECAP0_ECAP_SLV_BASE,            /* ecapRegBase */
    CSL_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_CFG_BASE,          /* miiRtCfgRegBase */
    CSL_PRU_ICSSG1_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_BASE, /* miiGRtCfgRegBase */
    CSL_PRU_ICSSG1_PR1_MDIO_V1P7_MDIO_BASE,                 /* miiMdioRegBase */
    CSL_PRU_ICSSG1_DRAM0_SLV_RAM_BASE,                      /* pru0DramBase */
    CSL_PRU_ICSSG1_DRAM1_SLV_RAM_BASE,                      /* pru1DramBase */
    CSL_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_BASE,                 /* pru0IramBase */
    CSL_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_BASE,                 /* pru1IramBase */
    CSL_PRU_ICSSG1_RAM_SLV_RAM_BASE,                        /* sharedDramBase */
    CSL_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_BASE,         /* rtu0IramBase */
    CSL_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_BASE,         /* rtu1IramBase */
    CSL_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_BASE,             /* rtu0CtrlRegBase */
    CSL_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_BASE,             /* rtu1CtrlRegBase */
    CSL_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_BASE,                  /* txPru0CtrlRegBase */
    CSL_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_BASE,                  /* txPru1CtrlRegBase */
    CSL_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_BASE,              /* txPru0IramBase */
    CSL_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_BASE,              /* txPru1IramBase */
    CSL_PRU_ICSSG1_DRAM0_SLV_RAM_SIZE,                      /* pru0DramSize */
    CSL_PRU_ICSSG1_DRAM1_SLV_RAM_SIZE,                      /* pru1DramSize */
    CSL_PRU_ICSSG1_PR1_PDSP0_IRAM_RAM_SIZE,                 /* pru0IramSize */
    CSL_PRU_ICSSG1_PR1_PDSP1_IRAM_RAM_SIZE,                 /* pru1IramSize */
    CSL_PRU_ICSSG1_RAM_SLV_RAM_SIZE,                        /* sharedDramSize */
    CSL_PRU_ICSSG1_PR1_RTU0_PR1_RTU0_IRAM_RAM_SIZE,         /* rtu0IramSize */
    CSL_PRU_ICSSG1_PR1_RTU1_PR1_RTU1_IRAM_RAM_SIZE,         /* rtu1IramSize */
    CSL_PRU_ICSSG1_PR1_PDSP_TX0_IRAM_RAM_SIZE,              /* txPru0IramSize */
    CSL_PRU_ICSSG1_PR1_PDSP_TX1_IRAM_RAM_SIZE,              /* txPru1IramSize */
};
