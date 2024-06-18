/*
 *  Copyright (C) 2020-22 Texas Instruments Incorporated
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
#ifndef SOC_CONFIG_IN_H_
#define SOC_CONFIG_IN_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* IP versions */
#define IP_VERSION_ADC_V0
#define IP_VERSION_BCDMA_V0
#define IP_VERSION_CRC_V0
#define IP_VERSION_ECAP_V0
#define IP_VERSION_ELM_V0
#define IP_VERSION_EPWM_V0
#define IP_VERSION_EQEP_V0
#define IP_VERSION_FIREWALL_V0
#define IP_VERSION_FSI_V0
#define IP_VERSION_GPIO_V0
#define IP_VERSION_GPMC_V0
#define IP_VERSION_GTC_V0
#define IP_VERSION_I2C_V0
#define IP_VERSION_INTAGGR_V0
#define IP_VERSION_INTR_ROUTER_V0
#define IP_VERSION_LCDMA_RINGACC_V0
#define IP_VERSION_DDR_V0
#define IP_VERSION_ESM_V1
#define IP_VERSION_MCAN_V0
#define IP_VERSION_MCSPI_V0
#define IP_VERSION_MDIO_V0
#define IP_VERSION_MMCSD_V0
#define IP_VERSION_OSPI_V0
#define IP_VERSION_PCIE_V0
#define IP_VERSION_PKTDMA_V0
#define IP_VERSION_PRUICSS_G_V0
#define IP_VERSION_PSILCFG_V0
#define IP_VERSION_RINGACC_V0
#define IP_VERSION_SERDES_V1
#define IP_VERSION_SPINLOCK_V0
#define IP_VERSION_TIMER_MGR_V0
#define IP_VERSION_UART_V0
#define DDR_16BIT
// #define INTR_PROF

/* Driver versions */
#define DRV_VERSION_ADC_V0
#define DRV_VERSION_CRC_V0
#define DRV_VERSION_ECAP_V0
#define DRV_VERSION_ELM_V0
#define DRV_VERSION_EPWM_V0
#define DRV_VERSION_EQEP_V0
#define DRV_VERSION_ESM_V1
#define DRV_VERSION_FIREWALL_V0
#define DRV_VERSION_FLASH_V0
#define DRV_VERSION_FSI_V0
#define DRV_VERSION_GPIO_V0
#define DRV_VERSION_GPMC_V0
#define DRV_VERSION_GTC_V0
#define DRV_VERSION_I2C_V0
#define DRV_VERSION_DDR_V0
#define DRV_VERSION_MCAN_V0
#define DRV_VERSION_MCSPI_V0
#define DRV_VERSION_MDIO_V0
#define DRV_VERSION_MMCSD_V0
#define DRV_VERSION_OSPI_V0
#define DRV_VERSION_PCIE_V0
#define DRV_VERSION_PRUICSS_G_V0
#define DRV_VERSION_PSRAM_V0
#define DRV_VERSION_SERDES_V1
#define DRV_VERSION_SPINLOCK_V0
#define DRV_VERSION_TIMER_MGR_V0
#define DRV_VERSION_UART_V0
#define DRV_VERSION_UDMA_V0
#define DRV_VERSION_WATCHDOG_V1

/* Driver DMA integration */
#define DMA_VERSION_MCSPI_UDMA
#define DMA_VERSION_UART_UDMA

#ifdef __cplusplus
}
#endif

#endif
