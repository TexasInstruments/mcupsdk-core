/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

#include <drivers/soc.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/CpuIdP.h>
#include <drivers/hw_include/am263px/cslr_mss_ctrl.h>
#include <drivers/hw_include/am263px/cslr_soc_defines.h>
#include <drivers/hw_include/am263px/cslr_soc_baseaddress.h>

#define EPWM_HALTEN_STEP  (CSL_CONTROLSS_CTRL_EPWM1_HALTEN - CSL_CONTROLSS_CTRL_EPWM0_HALTEN)
#define ECAP_HALTEN_STEP  (CSL_CONTROLSS_CTRL_ECAP1_HALTEN - CSL_CONTROLSS_CTRL_ECAP0_HALTEN)

#define EDMA_M4F_VIRT_TO_PHY_OFFSET             (0x20020000U)

typedef struct
{
    uint32_t tcmaSize;
    uint32_t tcmbSize;

    CSL_ArmR5CPUInfo cpuInfo;

}SOC_VirtToPhyMap;

SOC_VirtToPhyMap virtToPhymap;
uint8_t isMapAvailable = 0u;

int32_t SOC_moduleClockEnable(uint32_t moduleId, uint32_t enable)
{
    int32_t status = SystemP_SUCCESS;

    status = SOC_rcmEnablePeripheralClock((SOC_RcmPeripheralId)moduleId, enable);

    return status;
}

int32_t SOC_moduleSetClockFrequency(uint32_t moduleId, uint32_t clkId, uint64_t clkRate)
{
    int32_t status = SystemP_SUCCESS;

    SOC_controlModuleUnlockMMR(0, MSS_RCM_PARTITION0);

    status = SOC_rcmSetPeripheralClock((SOC_RcmPeripheralId)moduleId, (SOC_RcmPeripheralClockSource)clkId, (uint32_t)clkRate);

	SOC_controlModuleLockMMR(0, MSS_RCM_PARTITION0);

    return status;
}

const char *SOC_getCoreName(uint16_t coreId)
{
    static char *coreIdNames[CSL_CORE_ID_MAX+2] = {
        "r5f0-0",
        "r5f0-1",
        "r5f1-0",
        "r5f1-1",
        "hsm0-0",
        "unknown"
    };
    const char *name;

    if(coreId < CSL_CORE_ID_MAX+1)
    {
        name = coreIdNames[coreId];
    }
    else
    {
        name = coreIdNames[CSL_CORE_ID_MAX+1];
    }
    return name;
}

uint64_t SOC_getSelfCpuClk(void)
{
    uint64_t cpuClockRate = 0U;

    cpuClockRate = SOC_rcmGetR5Clock(CSL_CORE_ID_R5FSS0_0);

    return cpuClockRate;
}

void SOC_controlModuleLockMMR(uint32_t domainId, uint32_t partition)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(partition==MSS_CTRL_PARTITION0)
    {
        /*Lock MSS_CTRL*/
        baseAddr = (uint32_t) CSL_MSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    if(partition==MSS_RCM_PARTITION0)
    {
        /*Lock MSS_RCM*/
        baseAddr = (uint32_t) CSL_MSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }
    if(partition==TOP_CTRL_PARTITION0)
    {
        /*Lock TOP_CTRL*/
        baseAddr = (uint32_t) CSL_TOP_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }
    if(partition==TOP_RCM_PARTITION0)
    {
        /*Lock TOP_RCM*/
        baseAddr = (uint32_t) CSL_TOP_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }
    if(partition==CONTROLSS_CTRL_PARTITION0)
    {
        /*Lock CONTROLSS_CTRL*/
        baseAddr = (uint32_t) CSL_CONTROLSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_CONTROLSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_CONTROLSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */
    }

    return;
}

void SOC_controlModuleUnlockMMR(uint32_t domainId, uint32_t partition)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(partition==MSS_CTRL_PARTITION0)
    {
        /*Unlock MSS_CTRL*/
        baseAddr = (uint32_t) CSL_MSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);      /* KICK 1 */
    }

    if(partition==MSS_RCM_PARTITION0)
    {
        /*Unlock MSS_RCM*/
        baseAddr = (uint32_t) CSL_MSS_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_MSS_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);      /* KICK 1 */
    }
    if(partition==TOP_CTRL_PARTITION0)
    {
        /*Unlock TOP_CTRL*/
        baseAddr = (uint32_t) CSL_TOP_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);      /* KICK 1 */
    }
    if(partition==TOP_RCM_PARTITION0)
    {
        /*Unlock TOP_RCM*/
        baseAddr = (uint32_t) CSL_TOP_RCM_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_RCM_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_RCM_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);      /* KICK 1 */
    }
    if(partition==CONTROLSS_CTRL_PARTITION0)
    {
        /*Unlock CONTROLSS_CTRL*/
        baseAddr = (uint32_t) CSL_CONTROLSS_CTRL_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_CONTROLSS_CTRL_LOCK0_KICK0);
        CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);      /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_CONTROLSS_CTRL_LOCK0_KICK1);
        CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);      /* KICK 1 */
    }

    return;
}

void SOC_setEpwmTbClk(uint32_t epwmInstance, uint32_t enable)
{
    if(epwmInstance < CSL_ETPWM_PER_CNT)
    {
        /* Time base clock enable register belongs to partition 1 of the CTRL MMR */

        /* Unlock CONTROLSS_CTRL registers */
        SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

        if(TRUE == enable)
        {
            /* Enable Time base clock in CTRL MMR */
            CSL_REG32_WR(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC,
                ((CSL_REG32_RD(CSL_CONTROLSS_CTRL_U_BASE +
                  CSL_CONTROLSS_CTRL_EPWM_CLKSYNC) & CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MASK) | (1 << epwmInstance)));
        }
        else
        {
            /* Disable Time base clock in CTRL MMR */
            CSL_REG32_WR(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC,
                ((CSL_REG32_RD(CSL_CONTROLSS_CTRL_U_BASE +
                  CSL_CONTROLSS_CTRL_EPWM_CLKSYNC) & CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MASK) & ~(1 << epwmInstance)));
        }

        /* Lock CONTROLSS_CTRL registers */
        SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
    }
}

void SOC_setMultipleEpwmTbClk(uint32_t epwmMask, uint32_t enable)
{
    if(epwmMask <= CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MAX)
    {
        /* Time base clock enable register belongs to partition 1 of the CTRL MMR */

        /* Unlock CONTROLSS_CTRL registers */
        SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

        if(TRUE == enable)
        {
            /* Enable Time base clock in CTRL MMR */
            CSL_REG32_WR(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC,
                ((CSL_REG32_RD(CSL_CONTROLSS_CTRL_U_BASE +
                  CSL_CONTROLSS_CTRL_EPWM_CLKSYNC) & CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MASK) | (epwmMask)));
        }
        else
        {
            /* Disable Time base clock in CTRL MMR */
            CSL_REG32_WR(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC,
                ((CSL_REG32_RD(CSL_CONTROLSS_CTRL_U_BASE +
                  CSL_CONTROLSS_CTRL_EPWM_CLKSYNC) & CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MASK) & ~(epwmMask)));
        }

        /* Lock CONTROLSS_CTRL registers */
        SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
    }
}

void SOC_enableAdcReference(uint32_t adcInstance)
{
    /* Determine the group number of the ADC and the mask to be written to compctl register */
    uint32_t compctlmask = 0x7;

    uint32_t refbufCtrl_regOffset = CSL_TOP_CTRL_ADC_REFBUF0_CTRL;

    if(adcInstance == 1 || adcInstance == 2)
    {
        compctlmask = (compctlmask << 4);
        refbufCtrl_regOffset = CSL_TOP_CTRL_ADC_REFBUF0_CTRL;
    }
    else if(adcInstance == 3 || adcInstance == 4)
    {
        compctlmask = (compctlmask << 8);
        refbufCtrl_regOffset = CSL_TOP_CTRL_ADC_REFBUF1_CTRL;
    }
    else if(adcInstance == 5 || adcInstance == 6)
    {
        compctlmask = (compctlmask << 12);
        refbufCtrl_regOffset = CSL_TOP_CTRL_ADC_REFBUF2_CTRL;
    }

    /* Unlock Top Control Space */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);

    /* Enable ADC references by writing to MMR */
    CSL_REG16_WR(CSL_TOP_CTRL_U_BASE + refbufCtrl_regOffset, 0x7);
    CSL_REG16_WR(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_ADC_REF_COMP_CTRL,
        CSL_REG16_RD(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_ADC_REF_COMP_CTRL) | compctlmask);

    /* Lock Top Control Space */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);
}


void SOC_enableAdcInternalReference(uint32_t adcInstance, uint32_t enable)
{
    /* Determine the group number of the ADC and the mask to be written to compctl register */
    uint32_t refbufCtrl_regOffset = CSL_TOP_CTRL_ADC_REFBUF0_CTRL;
    uint16_t compctlmask = 0x7;

    if(adcInstance == 1 || adcInstance == 2)
    {
        compctlmask = (compctlmask << 4);
        refbufCtrl_regOffset = CSL_TOP_CTRL_ADC_REFBUF0_CTRL;
    }
    else if(adcInstance == 3 || adcInstance == 4)
    {
        compctlmask = (compctlmask << 8);
        refbufCtrl_regOffset = CSL_TOP_CTRL_ADC_REFBUF1_CTRL;
    }
    else if(adcInstance == 5 || adcInstance == 6)
    {
        compctlmask = (compctlmask << 12);
        refbufCtrl_regOffset = CSL_TOP_CTRL_ADC_REFBUF2_CTRL;
    }

    uint16_t mask = (enable == TRUE) ? (uint16_t)CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_MASK : (~(uint16_t)CSL_TOP_CTRL_ADC_REFBUF0_CTRL_ENABLE_MASK);
    /* Unlock Top Control Space */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);

    /* Mask HHV before enabling reference buffer to ensure the ADC does not cause an MCU reset. */
    CSL_REG16_WR(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_MASK_ANA_ISO, (0x7 & CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_MASK));

    /* Enable/ Disable ADC references by writing to MMR */
    CSL_REG16_WR(CSL_TOP_CTRL_U_BASE + refbufCtrl_regOffset, mask);

    /* Unmask HHV */
    CSL_REG16_WR(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_MASK_ANA_ISO, ((~0x7) & CSL_TOP_CTRL_MASK_ANA_ISO_MASK_ANA_ISO_MASK_MASK));

    /* Lock Top Control Space */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);
}

void SOC_enableAdcReferenceMonitor(uint32_t adcInstance, uint32_t enable)
{
    /* Determine the group number of the ADC and the mask to be written to compctl register */
    uint16_t compctlmask = CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC0_REFOK_EN_MASK;

    if(adcInstance == 1 || adcInstance == 2)
    {
        compctlmask = CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC12_REFOK_EN_MASK;
    }
    else if(adcInstance == 3 || adcInstance == 4)
    {
        compctlmask = CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADC34_REFOK_EN_MASK;
    }
    else if(adcInstance == 5 || adcInstance == 6)
    {
        compctlmask = CSL_TOP_CTRL_ADC_REF_COMP_CTRL_ADCR01_REFOK_EN_MASK;
    }
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);
    if(enable == TRUE)
    {
        /* write to Monitor enable register */
        CSL_REG16_WR(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_ADC_REF_COMP_CTRL,
            CSL_REG16_RD(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_ADC_REF_COMP_CTRL) | compctlmask);
    }
    else
    {
        /* write to Monitor Disable register */
        CSL_REG16_WR(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_ADC_REF_COMP_CTRL,
            CSL_REG16_RD(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_ADC_REF_COMP_CTRL) & ~compctlmask);
    }
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);
}

uint32_t SOC_getAdcReferenceStatus(uint32_t adcInstance)
{
    uint16_t statusMask = CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_OV_GOOD_MASK | CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_UV_GOOD_MASK;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);
    uint16_t refStatus = CSL_REG16_RD(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_ADC_REF_GOOD_STATUS);
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);

    if(adcInstance == 0){
        statusMask = CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_OV_GOOD_MASK | CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC0_REF_UV_GOOD_MASK;
    }
    else if((adcInstance == 1) || (adcInstance == 2)){
        statusMask = CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_OV_GOOD_MASK | CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC12_REF_UV_GOOD_MASK;
    }
    else if((adcInstance == 3) || (adcInstance == 4)){
        statusMask = CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_OV_GOOD_MASK | CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADC34_REF_UV_GOOD_MASK;
    }
    else if((adcInstance == 5) || (adcInstance == 6)){
        statusMask = CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_OV_GOOD_MASK | CSL_TOP_CTRL_ADC_REF_GOOD_STATUS_ADCR01_REF_UV_GOOD_MASK;
    }

    if((refStatus & statusMask) == statusMask)
    {
        /* Monitor Status is good */
        return TRUE;
    }
    else
    {
        /* Monitor Status is Bad */
        return FALSE;
    }
}

void SOC_enableAdcOsdChannel(uint32_t adcInstance, uint32_t channel, uint32_t enable)
{
    uint32_t regOffset = CSL_TOP_CTRL_U_BASE;
    uint32_t mask = CSL_TOP_CTRL_ADC0_OSD_CHEN_ADC0_OSD_CHEN_CH_OSD_EN_MASK;
    uint32_t channel_shift = channel;

    if(adcInstance < 5)
    {
        /* for ADC 0 - 4 */
        regOffset += CSL_TOP_CTRL_ADC0_OSD_CHEN + adcInstance*(CSL_TOP_CTRL_ADC1_OSD_CHEN - CSL_TOP_CTRL_ADC0_OSD_CHEN);
    }
    else if(adcInstance <7)
    {
        /* for ADC_R0, ADC_R1 */
        regOffset += CSL_TOP_CTRL_ADCR01_OSD_CHEN;
        mask = CSL_TOP_CTRL_ADCR01_OSD_CHEN_ADCR01_OSD_CHEN_CH_OSD_EN_MASK;

        /* adc_r0 [passed as instance 5] needs no shift,
        whereas adc_r1 [passed as adcInstance 6] needs a shift of 4 bits */
        channel_shift += ((adcInstance - 5U) * 4U);
    }

    /* Unlock Top Control Space */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);

    if(TRUE == enable)
    {
        CSL_REG32_WR(regOffset,
                ((CSL_REG32_RD(regOffset) & mask) | (1U << channel_shift)));
    }
    else
    {
        CSL_REG32_WR(regOffset,
                ((CSL_REG32_RD(regOffset) & mask) & ~(1U << channel_shift)));
    }

    /* Lock Top Control Space */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);

}
void SOC_setAdcOsdConfig(uint32_t adcInstance, uint32_t config)
{
    uint32_t regOffset = CSL_TOP_CTRL_U_BASE;
    uint32_t mask = CSL_TOP_CTRL_ADC0_OSD_CTRL_ADC0_OSD_CTRL_FUNCTION_MASK;

    if(adcInstance < 5)
    {
        /* for ADC 0 - 4 */
        regOffset += CSL_TOP_CTRL_ADC0_OSD_CTRL + adcInstance*(CSL_TOP_CTRL_ADC1_OSD_CTRL - CSL_TOP_CTRL_ADC0_OSD_CTRL);
    }
    else if(adcInstance < 7)
    {
        /* for ADC_R0, ADC_R1 */
        regOffset += CSL_TOP_CTRL_ADCR01_OSD_CTRL;
    }
    /* Unlock Top Control Space */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);

    CSL_REG32_WR(regOffset,
            ((CSL_REG32_RD(regOffset) & ~mask) | config));

    /* Lock Top Control Space */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);
}

void SOC_enableAdcGlobalForce(uint32_t adcInstance, uint32_t enable)
{
    uint32_t regOffset = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL;

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
    if(TRUE == enable)
    {
        CSL_REG32_WR(regOffset,
            ((CSL_REG32_RD(regOffset) & CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_MASK)
            | (1U << adcInstance)));
    }
    else
    {
        CSL_REG32_WR(regOffset,
            ((CSL_REG32_RD(regOffset) & CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_MASK)
            & ~(1U << adcInstance)));
    }

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

}


void SOC_adcSocGlobalForce(uint32_t socNumber)
{
    uint32_t regOffset = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADCSOCFRCGB;

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Write to the SOC Number bit */
    /* Note: 0 to 1 transission triggers. and is a sticky bit */
    CSL_REG32_WR(regOffset, (~(1U << socNumber) & CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_MASK));
    CSL_REG32_WR(regOffset, ((1U << socNumber) & CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_MASK));
    CSL_REG32_WR(regOffset, (~(1U << socNumber) & CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_MASK));

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

}

void SOC_selectAdcExtChXbar(uint32_t extChXbarOut, uint32_t extChXbarIn)
{
    uint32_t regOffset = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL;
    regOffset += extChXbarOut * (CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL - CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Write the extChXbarIn Value */
    CSL_REG32_WR(regOffset,
            ((CSL_REG32_RD(regOffset) & ~CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_MASK)
            | extChXbarIn));

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

}

void SOC_selextAdcExtChDelay(uint32_t delay)
{
    uint32_t regOffset = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL;

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Write the extChXbarIn Value */
    CSL_REG32_WR(regOffset,
            ((CSL_REG32_RD(regOffset) & ~CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_MASK)
            | delay));

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_enableAdcDacLoopback(uint32_t enable)
{
    uint32_t regOffset = CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_ADC_LOOPBACK_CTRL;

   /* Unlock Top Control Space */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);

    if(TRUE == enable)
    {
        CSL_REG32_WR(regOffset,
            (CSL_REG32_RD(regOffset) | CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_MASK));
    }
    else
    {
        CSL_REG32_WR(regOffset,
            (CSL_REG32_RD(regOffset) & ~CSL_TOP_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_CTRL_ADC_LOOPBACK_EN_MASK));
    }
    /* Lock Top Control Space */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_CTRL_PARTITION0);
}

void SOC_enableCmpssaDacLoopBack(uint32_t cmpssaInstance, uint32_t dacType, uint32_t enable)
{
    uint32_t regOffset = CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL;
    uint32_t shift = CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_SHIFT;
    if(dacType == CMPSS_LOOP_BACK_INH)
    {
        shift = CSL_TOP_CTRL_CMPSSA_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_SHIFT;
    }
    shift += cmpssaInstance;

    if(enable == TRUE)
    {
        CSL_REG32_WR(regOffset,
            (CSL_REG32_RD(regOffset) | (1U << shift)));
    }
    else
    {
        CSL_REG32_WR(regOffset,
            (CSL_REG32_RD(regOffset) & ~(1U << shift)));
    }
}

void SOC_enableCmpssbDacLoopBack(uint32_t cmpssbInstance, uint32_t dacType, uint32_t enable)
{
    uint32_t regOffset = CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL;
    uint32_t shift = CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSL0_LOOPBACK_EN_SHIFT;
    if(dacType == CMPSS_LOOP_BACK_INH)
    {
        shift = CSL_TOP_CTRL_CMPSSB_LOOPBACK_CTRL_CMPSSH0_LOOPBACK_EN_SHIFT;
    }
    shift += cmpssbInstance;

    if(enable == TRUE)
    {
        CSL_REG32_WR(regOffset,
            (CSL_REG32_RD(regOffset) | (1U << shift)));
    }
    else
    {
        CSL_REG32_WR(regOffset,
            (CSL_REG32_RD(regOffset) & ~(1U << shift)));
    }
}

void SOC_setEpwmGroup(uint32_t epwmInstance, uint32_t group)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0;
    uint32_t mask, shift;

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Choose the correct base address depending on which ePWM instance is selected*/
    if(epwmInstance > 15)
    {
        baseAddr = baseAddr + 0x4;
        epwmInstance = epwmInstance - 16;
    }

    shift = (epwmInstance << 1);
    /* Create the mask to be written to register */
    mask = (0x3 << shift);

    /* Configure the group for the ePWM instance */
    CSL_REG32_WR(baseAddr, (( CSL_REG32_RD(baseAddr) & ~mask) | (group <<shift)));

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_selectSdfm1Clk0Source(uint8_t source)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL;

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Configure the group for the ePWM instance */
    CSL_REG32_WR(baseAddr, source & CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_sdfmClkLoopBackConfig(uint32_t sdfmInstance, uint32_t clkInstance, uint32_t defaultValue)
{
    uint32_t regOffset = CSL_CONTROLSS_CTRL_U_BASE;
    regOffset += (sdfmInstance == 0)?CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL:CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL;
    regOffset += (clkInstance)*4;
    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
    if(TRUE == defaultValue)
    {
        CSL_REG32_WR(regOffset,
            (CSL_REG32_RD(regOffset) & ~CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_MASK));
    }
    else
    {
        CSL_REG32_WR(regOffset,
            (CSL_REG32_RD(regOffset) | CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_MASK));

    }

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateEpwmClock(uint32_t epwmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE + (0x4*epwmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateEpwmClock(uint32_t epwmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE + (0x4*epwmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateFsitxClock(uint32_t fsitxInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE + (0x4*fsitxInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateFsirxClock(uint32_t fsirxInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE + (0x4*fsirxInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateCmpssaClock(uint32_t cmpssaInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE + (0x4*cmpssaInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateCmpssaClock(uint32_t cmpssaInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE + (0x4*cmpssaInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateCmpssbClock(uint32_t cmpssbInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE + (0x4*cmpssbInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateCmpssbClock(uint32_t cmpssbInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE + (0x4*cmpssbInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateEcapClock(uint32_t ecapInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE + (0x4*ecapInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateEcapClock(uint32_t ecapInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE + (0x4*ecapInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateEqepClock(uint32_t eqepInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE + (0x4*eqepInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateEqepClock(uint32_t eqepInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE + (0x4*eqepInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateSdfmClock(uint32_t sdfmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE + (0x4*sdfmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateSdfmClock(uint32_t sdfmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE + (0x4*sdfmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateDacClock()
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_CLK_GATE;

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateDacClock()
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_CLK_GATE;

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateAdcClock(uint32_t adcInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADC0_CLK_GATE + (0x4*adcInstance);

    /* for ADC_R0, ADC_R1 these config registers are placed differently*/
    if(adcInstance >= 4)
    {
        uint32_t adcR0Instance = 5;
        baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADCR0_CLK_GATE + (0x4* (adcInstance - adcR0Instance));
    }
    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Gate Mask is same for ADCx and ADC_Rx*/
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateAdcClock(uint32_t adcInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADC0_CLK_GATE + (0x4*adcInstance);

    /* for ADC_R0, ADC_R1 these config registers are placed differently*/
    if(adcInstance >= 4)
    {
        uint32_t adcR0Instance = 5;
        baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADCR0_CLK_GATE + (0x4* (adcInstance - adcR0Instance));
    }
    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Gate Mask is same for ADCx and ADC_Rx*/
    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateRdcClock(uint32_t rdcInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE + (0x4*rdcInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateRdcClock(uint32_t rdcInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE + (0x4*rdcInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateOttoClock(uint32_t ottoInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE + (0x4*ottoInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateOttoClock(uint32_t ottoInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE + (0x4*ottoInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateSdfmPllClock(uint32_t sdfmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE + (0x4*sdfmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_ungateSdfmPllClock(uint32_t sdfmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE + (0x4*sdfmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, 0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_gateFsiPllClock(uint32_t fsiInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE + (0x4*fsiInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_MASK);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateEpwmReset(uint32_t ePWMInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ETPWM0_RST + (0x4*ePWMInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateFsiTxReset(uint32_t fsitxInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_FSI_TX0_RST + (0x4*fsitxInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateFsiRxReset(uint32_t fsirxInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_FSI_RX0_RST + (0x4*fsirxInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateCmpssaReset(uint32_t cmpssaInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_CMPSSA0_RST + (0x4*cmpssaInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateCmpssbReset(uint32_t cmpssbInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_CMPSSB0_RST + (0x4*cmpssbInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_CMPSSB0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_CMPSSB0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateEcapReset(uint32_t ecapInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ECAP0_RST + (0x4*ecapInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ECAP0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ECAP0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateEqepReset(uint32_t eqepInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EQEP0_RST + (0x4*eqepInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_EQEP0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_EQEP0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateSdfmReset(uint32_t sdfmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_SDFM0_RST + (0x4*sdfmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_SDFM0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_SDFM0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateDacReset()
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_DAC_RST;

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_DAC_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_DAC_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateAdcReset(uint32_t adcInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADC0_RST + (0x4*adcInstance);
    if(adcInstance > 4)
    {
        uint32_t adcR0Instance = 5;
        baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ADCR0_RST + (0x4*(adcInstance - adcR0Instance));
    }
    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* masks and reset values are same for ADCx and ADC_Rx*/
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ADC0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ADC0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_generateRdcReset(uint32_t rdcInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_HW_RESOLVER_RST + (0x4*rdcInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* masks and reset values are same for ADCx and ADC_Rx*/
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_HW_RESOLVER_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_HW_RESOLVER_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void Soc_enableEPWMHalt (uint32_t epwmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM0_HALTEN + (EPWM_HALTEN_STEP*epwmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Get Core ID Info */
    CSL_armR5GetCpuID(&virtToPhymap.cpuInfo);

    uint32_t shift = 0;

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0) /* R5SS0-0 */
            shift = CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_SHIFT;

        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)   /* R5SS0-1 */
            shift = CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_SHIFT;
    }
    else if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1)
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0) /* R5SS1-0 */
            shift = CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A1_SHIFT;

        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)   /* R5SS1-1 */
            shift = CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B1_SHIFT;
    }

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_MASK << shift);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void Soc_disableEPWMHalt (uint32_t epwmInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM0_HALTEN + (EPWM_HALTEN_STEP*epwmInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Get Core ID Info */
    CSL_armR5GetCpuID(&virtToPhymap.cpuInfo);

    uint32_t shift = 0;

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0) /* R5SS0-0 */
            shift = CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_SHIFT;

        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)   /* R5SS0-1 */
            shift = CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_SHIFT;
    }
    else if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1)
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0) /* R5SS1-0 */
            shift = CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A1_SHIFT;

        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)   /* R5SS1-1 */
            shift = CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B1_SHIFT;
    }

    CSL_REG32_WR(baseAddr, CSL_REG32_RD(baseAddr) &  ~(CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_MASK << shift));

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

static inline void Soc_ECAPHalt(uint32_t ecapInstance, bool enable)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_ECAP0_HALTEN + (ECAP_HALTEN_STEP*ecapInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    /* Get Core ID Info */
    CSL_ArmR5CPUInfo cpuInfo;
    CSL_armR5GetCpuID(&cpuInfo);

    uint32_t shift = CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_SHIFT;   // init with R5FSS0-0

    if (cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0) /* R5SS0-0 */
        {    
            shift = CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_SHIFT;
        }else                                   /* R5SS0-1 */
        {
            /* cpuInfo.cpuID is CSL_ARM_R5_CPU_ID_1 */
            shift = CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_SHIFT;
        }

    }
    else /* cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1 */
    {
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0) /* R5SS1-0 */
        {
            shift = CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A1_SHIFT;
        }else                                    /* R5SS1-1 */
        {
            /* cpuInfo.cpuID = CSL_ARM_R5_CPU_ID_1 */
            shift = CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B1_SHIFT;
        }

    }

    if(true == enable)
    {
        CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_MASK << shift);
    }else 
    {
        /* enable = false */
        CSL_REG32_WR(baseAddr, CSL_REG32_RD(baseAddr) &  ~(CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_MASK << shift));
    }

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}
void Soc_enableECAPHalt (uint32_t ecapInstance)
{
    Soc_ECAPHalt(ecapInstance, true);
}

void Soc_disableECAPHalt (uint32_t ecapInstance)
{
    Soc_ECAPHalt(ecapInstance, false);
}

void SOC_generateOttoReset(uint32_t ottoInstance)
{
    uint32_t baseAddr = CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_OTTO0_RST + (0x4*ottoInstance);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_OTTO0_RST_RST_MASK);
    CSL_REG32_WR(baseAddr, CSL_CONTROLSS_CTRL_OTTO0_RST_RST_RESETVAL);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
}

void SOC_selectIcssGpiMux(uint8_t pru_instance, uint32_t mask)
{
    uint32_t baseAddr;

    /* Unlock MSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, MSS_CTRL_PARTITION0);

    if(pru_instance == 0)
    {
        baseAddr = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU0_GPI_SEL;
        CSL_REG32_WR(baseAddr, mask & CSL_MSS_CTRL_ICSSM_PRU0_GPI_SEL_SEL_MASK);
    }
    if(pru_instance == 1)
    {
        baseAddr = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU1_GPI_SEL;
        CSL_REG32_WR(baseAddr, mask & CSL_MSS_CTRL_ICSSM_PRU1_GPI_SEL_SEL_MASK);
    }

    /* Lock MSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, MSS_CTRL_PARTITION0);
}

uint64_t SOC_virtToPhy(void *virtAddr)
{
    uintptr_t   temp = (uintptr_t) virtAddr;
    uint64_t    phyAddr = (uint64_t) temp;                  /* Default case */

    /* R5F overrides */
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    uint32_t    retVal;

    if (0u == isMapAvailable)
    {
        /* Get Core ID Info */
        CSL_armR5GetCpuID(&virtToPhymap.cpuInfo);

        retVal = SOC_rcmIsR5FInLockStepMode(virtToPhymap.cpuInfo.grpId);
        /* LockStep Mode TCM Size is 64KB */

        if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
	    {
            if (retVal == TRUE)
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE * 2U);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE * 2U);
            }
            else
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE);
            }
        }
        else if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1) /* R5SS1-0 */
        {
            if (retVal == TRUE)
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE * 2U);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE * 2U);
            }
            else
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE);
            }
        }

        isMapAvailable = 1u;
    }

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* TCMA R5FSS0-0 */
            if((temp >= CSL_MSS_TCMA_RAM_BASE) &&
               (temp < (CSL_MSS_TCMA_RAM_BASE + virtToPhymap.tcmaSize)))
            {
                phyAddr -= CSL_MSS_TCMA_RAM_BASE;
                phyAddr += CSL_R5SS0_CORE0_TCMA_U_BASE;
            }

            /* TCMB R5FSS0-0 */
            else if((temp >= CSL_MSS_TCMB_RAM_BASE) &&
               (temp < (CSL_MSS_TCMB_RAM_BASE + virtToPhymap.tcmbSize)))
            {
                phyAddr -= CSL_MSS_TCMB_RAM_BASE;
                phyAddr += CSL_R5SS0_CORE0_TCMB_U_BASE;
            }
        }
        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            /* TCMA R5FSS0-1 */
            if((temp >= CSL_MSS_TCMA_RAM_BASE) &&
               (temp < (CSL_MSS_TCMA_RAM_BASE + CSL_MSS_TCMA_RAM_SIZE)))
            {
                phyAddr -= CSL_MSS_TCMA_RAM_BASE;
                phyAddr += CSL_R5SS0_CORE1_TCMA_U_BASE;
            }

            /* TCMB R5FSS0-1 */
            else if((temp >= CSL_MSS_TCMB_RAM_BASE) &&
               (temp < (CSL_MSS_TCMB_RAM_BASE + CSL_MSS_TCMB_RAM_SIZE)))
            {
                phyAddr -= CSL_MSS_TCMB_RAM_BASE;
                phyAddr += CSL_R5SS0_CORE1_TCMB_U_BASE;
            }
        }
    }

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1) /* R5SS1-0 */
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* TCMA R5FSS1-0 */
            if((temp >= CSL_MSS_TCMA_RAM_BASE) &&
               (temp < (CSL_MSS_TCMA_RAM_BASE + virtToPhymap.tcmaSize)))
            {
                phyAddr -= CSL_MSS_TCMA_RAM_BASE;
                phyAddr += CSL_R5SS1_CORE0_TCMA_U_BASE;
            }

            /* TCMB R5FSS1-0 */
            else if((temp >= CSL_MSS_TCMB_RAM_BASE) &&
               (temp < (CSL_MSS_TCMB_RAM_BASE + virtToPhymap.tcmbSize)))
            {
                phyAddr -= CSL_MSS_TCMB_RAM_BASE;
                phyAddr += CSL_R5SS1_CORE0_TCMB_U_BASE;
            }
        }
        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            /* TCMA R5FSS1-1 */
            if((temp >= CSL_MSS_TCMA_RAM_BASE) &&
               (temp < (CSL_MSS_TCMA_RAM_BASE + CSL_MSS_TCMA_RAM_SIZE)))
            {
                phyAddr -= CSL_MSS_TCMA_RAM_BASE;
                phyAddr += CSL_R5SS1_CORE1_TCMA_U_BASE;
            }

            /* TCMB R5FSS1-1 */
            else if((temp >= CSL_MSS_TCMB_RAM_BASE) &&
               (temp < (CSL_MSS_TCMB_RAM_BASE + CSL_MSS_TCMB_RAM_SIZE)))
            {
                phyAddr -= CSL_MSS_TCMB_RAM_BASE;
                phyAddr += CSL_R5SS1_CORE1_TCMB_U_BASE;
            }
        }
    }
#endif

#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'M')
    if ( temp >= (CSL_HSM_M4_RAM_BASE - CSL_HSM_M4_RAM_BASE) &&
        (temp < CSL_HSM_M4_RAM_SIZE))
    {
        phyAddr += EDMA_M4F_VIRT_TO_PHY_OFFSET;
    }
#endif

    return (phyAddr);
}

void *SOC_phyToVirt(uint64_t phyAddr)
{
    void       *virtAddr = (void *) ((uintptr_t) phyAddr);  /* Default case */

    /* R5F overrides */
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    uint32_t    retVal;

    if (0u == isMapAvailable)
    {
        /* Get Core ID Info */
        CSL_armR5GetCpuID(&virtToPhymap.cpuInfo);

        retVal = SOC_rcmIsR5FInLockStepMode(virtToPhymap.cpuInfo.grpId);
        /* LockStep Mode TCM Size is 64KB */

        if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
	    {
            if (retVal == TRUE)
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE * 2U);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE * 2U);
            }
            else
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE);
            }
        }
        else if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1) /* R5SS1-0 */
        {
            if (retVal == TRUE)
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE * 2U);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE * 2U);
            }
            else
            {
                virtToPhymap.tcmaSize = (CSL_MSS_TCMA_RAM_SIZE);
                virtToPhymap.tcmbSize = (CSL_MSS_TCMB_RAM_SIZE);
            }
        }

        isMapAvailable = 1u;
    }

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* TCMA - R5FSS0-0 */
            if((phyAddr >= CSL_R5SS0_CORE0_TCMA_U_BASE) &&
               (phyAddr < (CSL_R5SS0_CORE0_TCMA_U_BASE + virtToPhymap.tcmaSize)))
            {
                phyAddr -= CSL_R5SS0_CORE0_TCMA_U_BASE;
                phyAddr += CSL_MSS_TCMA_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
            /* TCMB - R5FSS0-0 */
            else if((phyAddr >= CSL_R5SS0_CORE0_TCMB_U_BASE) &&
               (phyAddr < (CSL_R5SS0_CORE0_TCMB_U_BASE + virtToPhymap.tcmbSize)))
            {
                phyAddr -= CSL_R5SS0_CORE0_TCMB_U_BASE;
                phyAddr += CSL_MSS_TCMB_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
        }
        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            /* TCMA - R5FSS0-1 */
            if((phyAddr >= CSL_R5SS0_CORE1_TCMA_U_BASE) &&
               (phyAddr < (CSL_R5SS0_CORE1_TCMA_U_BASE + CSL_MSS_TCMA_RAM_SIZE)))
            {
                phyAddr -= CSL_R5SS0_CORE1_TCMA_U_BASE;
                phyAddr += CSL_MSS_TCMA_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
            /* TCMB - R5FSS0-1 */
            else if((phyAddr >= CSL_R5SS0_CORE1_TCMB_U_BASE) &&
               (phyAddr < (CSL_R5SS0_CORE1_TCMB_U_BASE + CSL_MSS_TCMB_RAM_SIZE)))
            {
                phyAddr -= CSL_R5SS0_CORE1_TCMB_U_BASE;
                phyAddr += CSL_MSS_TCMB_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
        }
    }

    if (virtToPhymap.cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1) /* R5SS1-0 */
    {
        if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* TCMA - R5FSS1-0 */
            if((phyAddr >= CSL_R5SS1_CORE0_TCMA_U_BASE) &&
               (phyAddr < (CSL_R5SS1_CORE0_TCMA_U_BASE + virtToPhymap.tcmaSize)))
            {
                phyAddr -= CSL_R5SS1_CORE0_TCMA_U_BASE;
                phyAddr += CSL_MSS_TCMA_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
            /* TCMB - R5FSS1-0 */
            else if((phyAddr >= CSL_R5SS1_CORE0_TCMB_U_BASE) &&
               (phyAddr < (CSL_R5SS1_CORE0_TCMB_U_BASE + virtToPhymap.tcmbSize)))
            {
                phyAddr -= CSL_R5SS1_CORE0_TCMB_U_BASE;
                phyAddr += CSL_MSS_TCMB_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
        }
        else if(virtToPhymap.cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            /* TCMA - R5FSS1-1 */
            if((phyAddr >= CSL_R5SS1_CORE1_TCMA_U_BASE) &&
               (phyAddr < (CSL_R5SS1_CORE1_TCMA_U_BASE + CSL_MSS_TCMA_RAM_SIZE)))
            {
                phyAddr -= CSL_R5SS1_CORE1_TCMA_U_BASE;
                phyAddr += CSL_MSS_TCMA_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
            /* TCMB - R5FSS1-1 */
            else if((phyAddr >= CSL_R5SS1_CORE1_TCMB_U_BASE) &&
               (phyAddr < (CSL_R5SS1_CORE1_TCMB_U_BASE + CSL_MSS_TCMB_RAM_SIZE)))
            {
                phyAddr -= CSL_R5SS1_CORE1_TCMB_U_BASE;
                phyAddr += CSL_MSS_TCMB_RAM_BASE;
                virtAddr = (void *) ((uintptr_t) phyAddr);
            }
        }
    }
#endif

    return (virtAddr);
}

uint32_t SOC_getFlashDataBaseAddr(void)
{
    return CSL_EXT_FLASH0_U_BASE;
}

void SOC_sendSoftwareInterrupt(uint16_t coreId)
{
    volatile CSL_mss_ctrlRegs * regs = (volatile CSL_mss_ctrlRegs*)CSL_MSS_CTRL_U_BASE;
    switch(coreId)
    {
        case CSL_CORE_ID_R5FSS0_0:
            regs->R5SS0_CORE0_SW_INT = 1;
            break;

        case CSL_CORE_ID_R5FSS0_1:
            regs->R5SS0_CORE1_SW_INT = 1;
            break;

        case CSL_CORE_ID_R5FSS1_0:
            regs->R5SS1_CORE0_SW_INT = 1;
            break;

        case CSL_CORE_ID_R5FSS1_1:
            regs->R5SS1_CORE1_SW_INT = 1;
            break;
    };
}