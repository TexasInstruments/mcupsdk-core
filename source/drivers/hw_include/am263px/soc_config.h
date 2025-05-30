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
#define IP_VERSION_ADC_V2
#define IP_VERSION_CMPSS_V0
#define IP_VERSION_DAC_V0
#define IP_VERSION_ECAP_V1
#define IP_VERSION_EDMA_V0
#define IP_VERSION_EPWM_V1
#define IP_VERSION_EQEP_V1
#define IP_VERSION_FSI_V1
#define IP_VERSION_FSS_V0
#define IP_VERSION_GPIO_V0
#define IP_VERSION_I2C_V1
#define IP_VERSION_MCAN_V0
#define IP_VERSION_MCSPI_V0
#define IP_VERSION_MDIO_V0
#define IP_VERSION_MMCSD_V1
#define IP_VERSION_PRUICSS_M_V0
#define IP_VERSION_OSPI_V0
#define IP_VERSION_RESOLVER_V0
#define IP_VERSION_RTI_V0
#define IP_VERSION_SDFM_V0
#define IP_VERSION_SPINLOCK_V0
#define IP_VERSION_UART_V0

/* Driver versions */
#define DRV_VERSION_ADC_V2
#define DRV_VERSION_CMPSS_V0
#define DRV_VERSION_DAC_V0
#define DRV_VERSION_ECAP_V1
#define DRV_VERSION_EDMA_V0
#define DRV_VERSION_EPWM_V1
#define DRV_VERSION_EQEP_V1
#define DRV_VERSION_FLASH_V2
#define DRV_VERSION_FSI_V1
#define DRV_VERSION_FSS_V0
#define DRV_VERSION_FLSOPSKD_V0
#define DRV_VERSION_FOTAAGENT_V0
#define DRV_VERSION_GPIO_V0
#define DRV_VERSION_I2C_V1
#define DRV_VERSION_LIN_V0
#define DRV_VERSION_MCAN_V0
#define DRV_VERSION_MCSPI_V0
#define DRV_VERSION_MDIO_V0
#define DRV_VERSION_MMCSD_V1
#define DRV_VERSION_PRUICSS_M_V0
#define DRV_VERSION_OSPI_V0
#define DRV_VERSION_RESOLVER_V0
#define DRV_VERSION_RTI_V0
#define DRV_VERSION_SDFM_V0
#define DRV_VERSION_SPINLOCK_V0
#define DRV_VERSION_UART_V0
#define DRV_VERSION_WATCHDOG_V0

/* Driver DMA integration */
#define DMA_VERSION_MCSPI_EDMA
#define DMA_VERSION_OSPI_EDMA
#define DMA_VERSION_UART_EDMA
#define DMA_VERSION_CANFD_EDMA

#ifdef __cplusplus
}
#endif

#endif
