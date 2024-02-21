/*
 *  Copyright (C) 2022-2024 Texas Instruments Incorporated
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

#include <string.h>
#include <drivers/pcie.h>
#include <drivers/pcie/pcie_priv.h>
#include <drivers/pcie/v0/pcie_v0_reg.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_pcie.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PCIE_REV0_CLASSCODE_MASK ( \
           CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_PIB_MASK | \
           CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_SCC_MASK | \
           CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_CC_MASK)
#define PCIE_REV0_CLASSCODE_SHIFT (CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_PIB_SHIFT)

/* BAR mask */
#define PCIE_BAR_MASK       0x0FFFFFFF

/* Maximum number of outbound regions */
#define OB_REGION_MAX       32

/* Maximum number of inbound regions for EP */
#define IB_REGION_MAX_EP    8

/* Maximum number of inbound regions for RC */
#define IB_REGION_MAX_RC    2

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    /**
     * @brief Used to simulate IATU window selection where only one IATU config
     * is visible at time.  Instead of adding 32 windows to API, keep using old
     * set index model, except remember the index and direction to be read or
     * modified here
     */
    Pcie_PlconfIatuIndexReg simIatuWindow[PCIE_MAX_PERIPHS];
} Pciev0_LocalObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Register read functions */
static int32_t Pcie_readPidReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PidReg *swReg);
static int32_t Pcie_readCmdStatusReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_CmdStatusReg *swReg);
static int32_t Pcie_readLinkStatusReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_TiConfDeviceCmdReg *swReg);
static int32_t Pcie_readVendorDevId(volatile const uint32_t *hwReg_VENDOR_ID_DEV_ID, Pcie_VndDevIdReg *swReg);
static int32_t Pcie_readSubIdReg(volatile const uint32_t *hwReg_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_ID, Pcie_SubIdReg *swReg);
static int32_t Pcie_readIntPinReg(volatile const uint32_t *hwReg_INTRPT_LINE_INTRPT_PIN, Pcie_IntPinReg *swReg);
static int32_t Pcie_readDevCapReg(volatile const uint32_t *hwReg_DEV_CAS, Pcie_DeviceCapReg *swReg);
static int32_t Pcie_readDevStatCtrlReg (volatile const uint32_t *hwReg_DEV_CAS, Pcie_DevStatCtrlReg *swReg);
static int32_t Pcie_readLinkCapReg (volatile const uint32_t *hwReg_LNK_CAP, Pcie_LinkCapReg *swReg);
static int32_t Pcie_readLinkStatCtrlReg (volatile const uint32_t *hwReg_LNK_CAS, Pcie_LinkStatCtrlReg *swReg);
static int32_t Pcie_readAccrReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_AccrReg *swReg);
static int32_t Pcie_readEpIrqSetReg(const CSL_user_cfgRegs *baseAddr, Pcie_EpIrqSetReg *swReg,
                            int_fast32_t swRegNum);
static int32_t Pcie_readEpIrqClrReg (const CSL_user_cfgRegs *baseAddr, Pcie_EpIrqClrReg *swReg,
                            int_fast32_t swRegNum);
static int32_t Pcie_readRcIrqEoi (CSL_user_cfgRegs *baseAddr, Pcie_IrqEOIReg *swReg);
static int32_t Pcie_readStatusCmdReg(volatile const uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                            Pcie_StatusCmdReg *swReg);
static int32_t Pcie_readBaseAddrReg(volatile const uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                            Pcie_BaseAddrReg *swReg);
static int32_t Pcie_readRevIdReg(volatile const uint32_t *hwReg_CLASSCODE_REVISIONID,
                            Pcie_RevIdReg *swReg);
static int32_t Pcie_readType0BarReg( const CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                            int32_t barNum);
static int32_t Pcie_readType1BarReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                            int32_t barNum);
static int32_t Pcie_readType0Bar32bitReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_Bar32bitReg *swReg,
                            int32_t barNum, uint8_t pfNum);
static int32_t Pcie_readPlconfIatuRegCtrl1Reg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                            Pcie_PlconfIatuRegCtrl1Reg *swReg);
static int32_t Pcie_readPlconfIatuRegLowerBaseReg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                            Pcie_PlconfIatuRegLowerBaseReg *swReg);
static int32_t Pcie_readPlconfIatuRegUpperBaseReg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                            Pcie_PlconfIatuRegUpperBaseReg *swReg);
static int32_t Pcie_readPlconfIatuRegLowerTargetReg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                            Pcie_PlconfIatuRegLowerTargetReg *swReg);
static int32_t Pcie_readPlconfIatuRegUpperTargetReg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                            Pcie_PlconfIatuRegUpperTargetReg *swReg);
static int32_t Pcie_readMsiCapReg (const CSL_pcie_ep_coreRegs *baseAddr,Pcie_MsiCapReg *swReg);
static int32_t Pcie_readMsiLo32Reg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiLo32Reg *swReg);
static int32_t Pcie_readMsiUp32Reg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiUp32Reg *swReg);
static int32_t Pcie_readMsiDataReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiDataReg *swReg);
static int32_t Pcie_readMsixCapReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsixCapReg *swReg);
static int32_t Pcie_readMsixTblOffset (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsixTblOffset *swReg);
static int32_t Pcie_readPmMgmtCtrlStatReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PMCapCtlStatReg *swReg);

static int32_t Pcie_readUsrInitCfgReg(const CSL_user_cfgRegs *baseAddr, Pcie_UserCfgInitCfgReg *swReg);
static int32_t Pcie_readUsrPmCmdReg(const CSL_user_cfgRegs *baseAddr, Pcie_UserCfgPmCmdReg *swReg);
static int32_t Pcie_readIntdCfgEnSys0Reg(const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys0Reg *swReg);
static int32_t Pcie_readIntdCfgEnSys1Reg(const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys1Reg *swReg);
static int32_t Pcie_readIntdCfgEnSys2Reg(const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys2Reg *swReg);
static int32_t Pcie_readIntdCfgStatusSys0Reg(const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys0Reg *swReg);
static int32_t Pcie_readIntdCfgStatusSys1Reg(const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys1Reg *swReg);
static int32_t Pcie_readIntdCfgStatusSys2Reg(const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys2Reg *swReg);
static int32_t Pcie_readIntdCfgStatusClrSys2Reg(const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusClrSys2Reg *swReg);
static int32_t Pcie_readVndrSpecCntrlReg(volatile const uint32_t *hwReg_VENDOR_SPECIFIC_CONTROL_REG, Pcie_VndrSpecCntrlReg *swReg);
static int32_t Pcie_readLmVendorIdReg(volatile const uint32_t *hwReg_I_VENDOR_ID_REG, Pcie_LmVendorIdReg *swReg);

/* Register write functions */
static int32_t Pcie_writeVendorDevId(volatile uint32_t *hwReg_VENDOR_ID_DEV_ID, Pcie_VndDevIdReg *swReg);
static int32_t Pcie_writeSubIdReg(volatile uint32_t *hwReg_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_ID, Pcie_SubIdReg *swReg);
static int32_t Pcie_writeIntPinReg(volatile uint32_t *hwReg_INTRPT_LINE_INTRPT_PIN, Pcie_IntPinReg *swReg);
static int32_t Pcie_writeCmdStatusReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_CmdStatusReg *swReg);
static int32_t Pcie_writeDevCapReg(volatile uint32_t *hwReg_DEV_CAS, Pcie_DeviceCapReg *swReg);
static int32_t Pcie_writeDevStatCtrlReg (volatile uint32_t *hwReg_DEV_CAS, Pcie_DevStatCtrlReg *swReg);
static int32_t Pcie_writeLinkCapReg (volatile uint32_t *hwReg_LNK_CAP, Pcie_LinkCapReg *swReg);
static int32_t Pcie_writeLinkStatCtrlReg(volatile uint32_t *hwReg_LNK_CAS, Pcie_LinkStatCtrlReg *swReg);
static int32_t Pcie_writeAccrReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_AccrReg *swReg);
static int32_t Pcie_writeLinkStatusReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_TiConfDeviceCmdReg *swReg);
static int32_t Pcie_writeEpIrqSetReg (CSL_user_cfgRegs *baseAddr, Pcie_EpIrqSetReg *swReg,
                            int_fast32_t swRegNum);
static int32_t Pcie_writeEpIrqClrReg (CSL_user_cfgRegs *baseAddr, Pcie_EpIrqClrReg *swReg,
                            int_fast32_t swRegNum);
static int32_t Pcie_writeRcIrqEoi (CSL_user_cfgRegs *baseAddr, Pcie_IrqEOIReg *swReg);
static int32_t Pcie_writeStatusCmdReg (volatile uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                            Pcie_StatusCmdReg *swReg);
static int32_t Pcie_writeRevIdReg (volatile uint32_t *hwReg_CLASSCODE_REVISIONID,
                            Pcie_RevIdReg *swReg);
static int32_t Pcie_writeType0BarReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                            int32_t barNum);
static int32_t Pcie_writeType0Bar32bitReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_Bar32bitReg *swReg,
                            int32_t barNum, uint8_t pfNum);
static int32_t Pcie_writePlconfIatuRegLowerBaseReg (CSL_pcie_ep_coreRegs *baseAddr,
                            const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegLowerBaseReg *swReg);
static int32_t Pcie_writePlconfIatuRegLowerBaseRcReg (CSL_pcie_ep_coreRegs *baseAddr,
                            const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegLowerBaseReg *swReg);
static int32_t Pcie_writePlconfIatuRegUpperBaseReg (CSL_pcie_ep_coreRegs *baseAddr,
                            const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegUpperBaseReg *swReg);
static int32_t Pcie_writePlconfIatuRegLowerTargetReg (CSL_pcie_ep_coreRegs *baseAddr,
                            const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegLowerTargetReg *swReg);
static int32_t Pcie_writePlconfIatuRegUpperTargetReg (CSL_pcie_ep_coreRegs *baseAddr,
                            const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegUpperTargetReg *swReg);
static int32_t Pcie_writePlconfIatuRegCtrl1Reg (CSL_pcie_ep_coreRegs *baseAddr,
                            const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegCtrl1Reg *swReg);
static int32_t Pcie_writeMsiCapReg (CSL_pcie_ep_coreRegs *baseAddr,Pcie_MsiCapReg *swReg);
static int32_t Pcie_writeMsiLo32Reg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiLo32Reg *swReg);
static int32_t Pcie_writeMsiUp32Reg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiUp32Reg *swReg);
static int32_t Pcie_writeMsiDataReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiDataReg *swReg);
static int32_t Pcie_writeMsixCapReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsixCapReg *swReg);
static int32_t Pcie_writeMsixTblOffset (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsixTblOffset *swReg);
static int32_t Pcie_writePmMgmtCtrlStatReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PMCapCtlStatReg *swReg);

static int32_t Pcie_writeUsrInitCfgReg(CSL_user_cfgRegs *baseAddr, Pcie_UserCfgInitCfgReg *swReg);
static int32_t Pcie_writeUsrPmCmdReg(CSL_user_cfgRegs *baseAddr, Pcie_UserCfgPmCmdReg *swReg);
static int32_t Pcie_writeIntdCfgEnSys0Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys0Reg *swReg);
static int32_t Pcie_writeIntdCfgEnSys1Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys1Reg *swReg);
static int32_t Pcie_writeIntdCfgEnSys2Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys2Reg *swReg);
static int32_t Pcie_writeIntdCfgStatusSys0Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys0Reg *swReg);
static int32_t Pcie_writeIntdCfgStatusSys1Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys1Reg *swReg);
static int32_t Pcie_writeIntdCfgStatusSys2Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys2Reg *swReg);
static int32_t Pcie_writeIntdCfgStatusClrSys2Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusClrSys2Reg *swReg);
static int32_t Pcie_writeVndrSpecCntrlReg(volatile uint32_t *hwReg_VENDOR_SPECIFIC_CONTROL_REG, Pcie_VndrSpecCntrlReg *swReg);
static int32_t Pcie_writeLmVendorIdReg(volatile uint32_t *hwReg_I_VENDOR_ID_REG, Pcie_LmVendorIdReg *swReg);

static int32_t Pcie_obTransCfg(Pcie_Handle handle);
static void Pcie_setMode (Pcie_DevParams *devParams, Pcie_Mode mode, Pcie_Gen gen);
static uint32_t Pcie_getNumPassBitFromWinSize(uint32_t winSize);
static int32_t Pcie_checkAtuAlign(uint64_t size, uint32_t lowerAddr, uint32_t upperAddr);
static uint64_t Pcie_aperture2size(uint32_t barAperture);

static int32_t Pcie_setVendorId(Pcie_Handle handle);
static int32_t Pcie_setDeviceId(Pcie_Handle handle);
static int32_t Pcie_setSubSysVendorId(Pcie_Handle handle);
static int32_t Pcie_setSubSystemId(Pcie_Handle handle);
static int32_t Pcie_setRevIdClassCode(Pcie_Handle handle);
static int32_t Pcie_setFlr(Pcie_Handle handle, int enable);
static int32_t Pcie_setIntPin(Pcie_Handle handle);

/* Read number of lanes configured */
extern int32_t Pcie_readLnkCtrlReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_LnkCtrlReg *swReg);
/* Set number of lanes */
extern int32_t Pcie_writeLnkCtrlReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_LnkCtrlReg *swReg);
/* Lock PCIe MMR configuration register */
extern void Pcie_lockMMRCtrlReg(void);
/* Unlock PCIe MMR configuration register */
extern void Pcie_unlockMMRCtrlReg(void);

/* Write Legacy interrupt enable set register */
extern int32_t Pcie_writeLegacyIrqEnableSetReg (CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqEnableSetReg *swReg,
                            int_fast32_t swRegNum);
/* Write Legacy interrupt enable clear register */
extern int32_t Pcie_writeLegacyIrqEnableClrReg (CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqEnableClrReg *swReg,
                            int_fast32_t swRegNum);
/* Write Legacy interrupt status register */
extern int32_t Pcie_writeLegacyIrqStatusReg (CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqStatusReg *swReg,
                            int_fast32_t swRegNum);

/* Read Legacy interrupt enable set register */
extern int32_t Pcie_readLegacyIrqEnableSetReg(const CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqEnableSetReg *swReg,
                            int_fast32_t swRegNum);
/* Read Legacy interrupt enable clear register */
extern int32_t Pcie_readLegacyIrqEnableClrReg (const CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqEnableClrReg *swReg,
                            int_fast32_t swRegNum);
/* Read Legacy interrupt status register */
extern int32_t Pcie_readLegacyIrqStatusReg(const CSL_intd_cfgRegs *baseAddr, Pcie_LegacyIrqStatusReg *swReg,
                            int_fast32_t swRegNum);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Local object simulating IATU window selection. */
Pciev0_LocalObj pciev0LocalObj =
{
    {
        { 0u, 0u },
        { 0u, 0u },
        { 0u, 0u },
        { 0u, 0u }
    }
};

/*****************************************************************************
 * Read and split up the Peripheral Version and ID swRegister
 ****************************************************************************/
static int32_t Pcie_readPidReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PidReg *swReg)
{
    int32_t status = SystemP_SUCCESS;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        CSL_user_cfgRegs *userCfg = (CSL_user_cfgRegs *)baseAddr;
        uint32_t val = swReg->raw = userCfg->REVID;

        PCIE_GETBITS(val, CSL_USER_CFG_REVID_MODID, swReg->modId);
        PCIE_GETBITS(val, CSL_USER_CFG_REVID_REVRTL, swReg->rtl);
        PCIE_GETBITS(val, CSL_USER_CFG_REVID_REVMAJ, swReg->revMaj);
        PCIE_GETBITS(val, CSL_USER_CFG_REVID_CUSTOM, swReg->cust);
        PCIE_GETBITS(val, CSL_USER_CFG_REVID_REVMIN, swReg->revMin);

        /* Set unused fields to 0 */
        swReg->scheme = 0u;
        swReg->func   = 0u;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readPidReg */

/*****************************************************************************
 * Read and split up the Command Status swRegister
 ****************************************************************************/
static int32_t Pcie_readCmdStatusReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_CmdStatusReg *swReg)
{
    int32_t status = SystemP_SUCCESS;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        CSL_user_cfgRegs *userCfg = (CSL_user_cfgRegs *)baseAddr;
        uint32_t val = swReg->raw = userCfg->CMD_STATUS;

        PCIE_GETBITS(val, CSL_USER_CFG_CMD_STATUS_LINK_TRAINING_ENABLE, swReg->ltssmEn);

        /* Set unused fields to 0 (only used by rev 0/1 hw) */
        swReg->postedWrEn = 0u;
        swReg->ibXltEn    = 0u;
        swReg->obXltEn    = 0u;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readCmdStatusReg */

/*****************************************************************************
 * Set mode of interface as RC or EP
 ****************************************************************************/
static void Pcie_setMode (Pcie_DevParams *devParams, Pcie_Mode mode, Pcie_Gen gen)
{
    uint32_t val;
    uint32_t curMode, linkSpd;

    switch (mode)
    {
        case PCIE_EP_MODE:
            curMode = 0U;
            break;
        case PCIE_RC_MODE:
        default:
            curMode = 1U;
            break;
    }

    switch (gen)
    {
        case PCIE_GEN1:
            linkSpd = 0U;
            break;
        case PCIE_GEN2:
            linkSpd = 1U;
            break;
        case PCIE_GEN3:
            linkSpd = 2U;
            break;
        default:
            linkSpd = 1U;
            break;
    }

    if (devParams != NULL)
    {
        if (devParams->pcieCtrlAddr)
        {
            Pcie_unlockMMRCtrlReg();

            val = *(devParams->pcieCtrlAddr);
            PCIE_SETBITS(val, CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_MODE_SEL, curMode);
            PCIE_SETBITS(val, CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_GENERATION_SEL, linkSpd);
            *(devParams->pcieCtrlAddr) = val;

            Pcie_lockMMRCtrlReg();
        }
    }
} /* Pcie_setMode */

int32_t Pcie_setInterfaceMode(Pcie_Handle handle, Pcie_Mode mode, Pcie_Gen gen)
{
    Pcie_DeviceCfgBaseAddr *baseAddr;
    Pcie_DevParams *devParams;
    int32_t status = SystemP_SUCCESS;

    if (handle != NULL)
    {
        baseAddr = Pcie_handleGetBases (handle);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        if (baseAddr != NULL)
        {
            devParams = (Pcie_DevParams*)baseAddr->devParams;

            Pcie_setMode(devParams, mode, gen);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

/*****************************************************************************
 * Determine if link is up by examining LTSSM state
 ****************************************************************************/
int32_t Pcie_isLinkUp(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers getRegs;

    memset (&getRegs, 0, sizeof(getRegs));

    Pcie_TiConfDeviceCmdReg   ltssmStateReg;
    getRegs.tiConfDeviceCmd = &ltssmStateReg;

    memset (&ltssmStateReg,  0, sizeof(ltssmStateReg));

    uint8_t ltssmState = 0xFFu;

    status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &getRegs);

    if (status != SystemP_SUCCESS)
        return status;

    ltssmState = ltssmStateReg.ltssmState;

    if (ltssmState == PCIE_LTSSM_L0-1)
        return SystemP_SUCCESS;
    else
        return SystemP_TIMEOUT;

    return status;
}

/*****************************************************************************
 * Wait for the PCIe link to be up
 ****************************************************************************/
int32_t Pcie_waitLinkUp(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    while (1)
    {
        status = Pcie_isLinkUp(handle);

        if (status == SystemP_SUCCESS)
                return SystemP_SUCCESS;

        else if (status == SystemP_TIMEOUT)
                ClockP_usleep(10);
    }

    return status;
}

int32_t Pcie_checkLinkParams(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_LinkStatCtrlReg linkStatCtrl;
    Pcie_Config *pcieCfg = (Pcie_Config *)handle;

    int32_t expLanes = 1, expSpeed = 1;

    expSpeed = pcieCfg->attrs->gen;
    expLanes = pcieCfg->attrs->numLanes;

    /* Get link status */
    memset (&regs, 0, sizeof(regs));
    regs.linkStatCtrl = &linkStatCtrl;

    do
    {
        status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &regs);
    }while (((expLanes != linkStatCtrl.negotiatedLinkWd)||(expSpeed != linkStatCtrl.linkSpeed)) &&
                    (status == SystemP_SUCCESS));

    return status;
}

/*****************************************************************************
 * Return current link parameter
 ****************************************************************************/
int32_t Pcie_getLinkParams(Pcie_Handle handle, Pcie_Gen *gen, uint32_t *numLanes)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_LinkStatCtrlReg linkStatCtrl;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        memset (&linkStatCtrl, 0, sizeof(linkStatCtrl));

        regs.linkStatCtrl = &linkStatCtrl;

        status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        switch (linkStatCtrl.linkSpeed)
        {
            case 1U:
                *gen = PCIE_GEN1;  /**< PCIe Gen1 for 2.5 GT/s speed */
                break;
            case 2U:
                *gen = PCIE_GEN2;  /**< PCIe Gen2 for 5.0 GT/s speed */
                break;
            case 3U:
                *gen = PCIE_GEN3;  /**< PCIe Gen3 for 8.0 GT/s speed */
                break;
        }

        *numLanes = (uint32_t) linkStatCtrl.negotiatedLinkWd;
    }

    return status;
}

/*****************************************************************************
 * Return current power state
 ****************************************************************************/
int32_t Pcie_getPwrState(Pcie_Handle handle, Pcie_PwrState *pwrState)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_PMCapCtlStatReg pmCapCtlStat;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    /*
     * Reading pmCapCtlStat Register only valid
     * if SERDES RefClk on resp. PCIe Link established
     */
    status = Pcie_isLinkUp(handle);

    if (status == SystemP_TIMEOUT)
    {
        return SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        memset (&pmCapCtlStat, 0, sizeof(pmCapCtlStat));

        regs.pmCapCtlStat = &pmCapCtlStat;

        status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        switch (pmCapCtlStat.pwrState)
        {
            case 0U:
                *pwrState = PCIE_PWR_STATE_D0;
                break;
            case 1U:
                *pwrState = PCIE_PWR_STATE_D1;
                break;
            case 3U:
                *pwrState = PCIE_PWR_STATE_D3hot;
                break;
        }
    }

    return status;
}

/*****************************************************************************
 * Enable/Disable LTSSM (Link Training)
 * This function demonstrates how one can write one binary to use either
 * rev of PCIE
 ****************************************************************************/
int32_t Pcie_LtssmCtrl (Pcie_Handle handle, uint8_t enable)
{
    Pcie_CmdStatusReg cmdStatus;
    Pcie_TiConfDeviceCmdReg deviceCmd;
    Pcie_Registers regs;
    int32_t status;

    memset (&cmdStatus, 0, sizeof(cmdStatus));
    memset (&deviceCmd, 0, sizeof(deviceCmd));
    memset (&regs, 0, sizeof(regs));

    regs.cmdStatus = &cmdStatus;

    status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &regs);

    DebugP_assert (status == SystemP_SUCCESS);

    if(enable)
    {
        deviceCmd.ltssmEn = cmdStatus.ltssmEn = 1;
    }
    else
    {
        deviceCmd.ltssmEn = cmdStatus.ltssmEn = 0;
    }

    status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &regs);

    DebugP_assert (status == SystemP_SUCCESS);

    return status;
}

/*****************************************************************************
 * Function: Configure PCIe in Root complex Mode
 ****************************************************************************/
int32_t Pcie_cfgRC (Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_Registers setRegs;
    Pcie_Registers getRegs;
    Pcie_StatusCmdReg statusCmd;
    Pcie_DevStatCtrlReg devStatCtrl;
    Pcie_AccrReg accr;
    Pcie_AtuRegionParams regionParams;
    Pcie_BarCfg barCfg;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (SystemP_SUCCESS == status)
    {
        status = Pcie_LtssmCtrl(handle, FALSE);
    }

    /* Enable memory access and control of the bus */
    memset (&setRegs, 0, sizeof(setRegs));
    memset (&getRegs, 0, sizeof(getRegs));

    getRegs.statusCmd = &statusCmd;

    status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &getRegs);

    if (SystemP_SUCCESS == status)
    {
        statusCmd.memSp  = 1;
        statusCmd.busMs  = 1;
        statusCmd.resp   = 1;
        statusCmd.serrEn = 1;
        setRegs.statusCmd = &statusCmd;

        status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &setRegs);
    }

    /* Enable Error Reporting */
    if (SystemP_SUCCESS == status)
    {
        memset (&setRegs, 0, sizeof(setRegs));
        memset (&getRegs, 0, sizeof(getRegs));

        getRegs.devStatCtrl = &devStatCtrl;

        status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &getRegs);

        if (SystemP_SUCCESS == status)
        {
            devStatCtrl.reqRp = 1;
            devStatCtrl.fatalErRp = 1;
            devStatCtrl.nFatalErRp = 1;
            devStatCtrl.corErRp = 1;
            setRegs.devStatCtrl = &devStatCtrl;

            status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &setRegs);
        }
    }

    /* Enable ECRC */
    if (SystemP_SUCCESS == status)
    {
        memset (&setRegs, 0, sizeof(setRegs));

        accr.chkEn=1;
        accr.chkCap=1;
        accr.genEn=1;
        accr.genCap=1;
        setRegs.accr = &accr;

        status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &setRegs);
    }

    /* configure BARs */
    for (int i = 0; i < pcieCfg->attrs->ibAtuNum; i++)
    {
        if (status == SystemP_SUCCESS)
        {
            memset (&barCfg, 0, sizeof(Pcie_BarCfg));

            barCfg.location = PCIE_LOCATION_LOCAL;
            barCfg.mode     = PCIE_RC_MODE;
            barCfg.barxc    = pcieCfg->attrs->ibAtu[i].barCfg;
            barCfg.barxa    = pcieCfg->attrs->ibAtu[i].barAperture;
            barCfg.idx      = pcieCfg->attrs->ibAtu[i].regionIndex;

            status = Pcie_cfgBar (handle, &barCfg);

            DebugP_assert(SystemP_SUCCESS == status);
        }
        else
        {
            break;
        }
    }

    /* configure inbound ATU */
    for (int i = 0; i < pcieCfg->attrs->ibAtuNum; i++)
    {
        if (status == SystemP_SUCCESS)
        {
            memset(&regionParams, 0, sizeof(regionParams));

            regionParams.enableRegion     = 1;
            regionParams.regionDir        = PCIE_ATU_REGION_DIR_INBOUND;
            regionParams.tlpType          = pcieCfg->attrs->ibAtu[i].tlpType;
            regionParams.lowerBaseAddr    = pcieCfg->attrs->ibAtu[i].lowerBaseAddr;
            regionParams.upperBaseAddr    = pcieCfg->attrs->ibAtu[i].upperBaseAddr;
            regionParams.regionWindowSize = pcieCfg->attrs->ibAtu[i].regionWindowSize;
            regionParams.lowerTargetAddr  = pcieCfg->attrs->ibAtu[i].lowerTargetAddr;
            regionParams.upperTargetAddr  = pcieCfg->attrs->ibAtu[i].upperTargetAddr;

            status = Pcie_atuRegionConfig (handle, PCIE_LOCATION_LOCAL, (uint32_t) pcieCfg->attrs->ibAtu[i].regionIndex, &regionParams);

            DebugP_assert(SystemP_SUCCESS == status);
        }
        else
        {
            break;
        }
    }

    /* configure outbound ATU */
    for (int i = 0; i < pcieCfg->attrs->obAtuNum; i++)
    {
        if (status == SystemP_SUCCESS)
        {
            memset(&regionParams, 0, sizeof(regionParams));

            regionParams.enableRegion     = 1;
            regionParams.regionDir        = PCIE_ATU_REGION_DIR_OUTBOUND;
            regionParams.tlpType          = pcieCfg->attrs->obAtu[i].tlpType;
            regionParams.lowerBaseAddr    = pcieCfg->attrs->obAtu[i].lowerBaseAddr;
            regionParams.upperBaseAddr    = pcieCfg->attrs->obAtu[i].upperBaseAddr;
            regionParams.regionWindowSize = pcieCfg->attrs->obAtu[i].regionWindowSize;
            regionParams.lowerTargetAddr  = pcieCfg->attrs->obAtu[i].lowerTargetAddr;
            regionParams.upperTargetAddr  = pcieCfg->attrs->obAtu[i].upperTargetAddr;

            status = Pcie_atuRegionConfig (handle, PCIE_LOCATION_LOCAL, (uint32_t) pcieCfg->attrs->obAtu[i].regionIndex, &regionParams);

            DebugP_assert(SystemP_SUCCESS == status);
        }
        else
        {
            break;
        }
    }

    return status;
}

/*****************************************************************************
 * Function: Configure one BAR in EP mode
 ****************************************************************************/
static int32_t Pcie_cfgEP1Bar(Pcie_Handle handle, Pcie_IbAtuCfg ibAtu)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_BarCfg barCfg;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&barCfg, 0, sizeof(barCfg));

        barCfg.mode     = PCIE_EP_MODE;
        barCfg.location = PCIE_LOCATION_LOCAL;
        barCfg.idx      = ibAtu.regionIndex;
        barCfg.barxc    = ibAtu.barCfg;
        barCfg.barxa    = ibAtu.barAperture;

        status = Pcie_cfgBar (handle, &barCfg);
    }

    return status;
}

/*****************************************************************************
 * Determine ATU region windows size from BAR aperture
 ****************************************************************************/
static uint64_t Pcie_aperture2size(uint32_t barAperture)
{
    uint64_t size;

    switch (barAperture)
    {
        case PCIE_EPBARA_128B:
                size = 0x80;
                break;
        case PCIE_EPBARA_256B:
                size = 0x100;
                break;
        case PCIE_EPBARA_512B:
                size = 0x200;
                break;
        case PCIE_EPBARA_1K:
                size = 0x400;
                break;
        case PCIE_EPBARA_2K:
                size = 0x800;
                break;
        case PCIE_EPBARA_4K:
                size = 0x1000;
                break;
        case PCIE_EPBARA_8K:
                size = 0x2000;
                break;
        case PCIE_EPBARA_16K:
                size = 0x4000;
                break;
        case PCIE_EPBARA_32K:
                size = 0x8000;
                break;
        case PCIE_EPBARA_64K:
                size = 0x10000;
                break;
        case PCIE_EPBARA_128K:
                size = 0x20000;
                break;
        case PCIE_EPBARA_256K:
                size = 0x40000;
                break;
        case PCIE_EPBARA_512K:
                size = 0x8000;
                break;
        case PCIE_EPBARA_1M:
                size = 0x100000;
                break;
        case PCIE_EPBARA_2M:
                size = 0x200000;
                break;
        case PCIE_EPBARA_4M:
                size = 0x400000;
                break;
        case PCIE_EPBARA_8M:
                size = 0x800000;
                break;
        case PCIE_EPBARA_16M:
                size = 0x1000000;
                break;
        case PCIE_EPBARA_32M:
                size = 0x2000000;
                break;
        case PCIE_EPBARA_64M:
                size = 0x4000000;
                break;
        case PCIE_EPBARA_128M:
                size = 0x8000000;
                break;
        case PCIE_EPBARA_256M:
                size = 0x10000000;
                break;
        case PCIE_EPBARA_512M:
                size = 0x20000000;
                break;
        case PCIE_EPBARA_1G:
                size = 0x40000000;
                break;
        case PCIE_EPBARA_2G:
                size = 0x80000000;
                break;
        case PCIE_EPBARA_4G:
                size = 0x100000000;
                break;
        case PCIE_EPBARA_8G:
                size = 0x200000000;
                break;
        case PCIE_EPBARA_16G:
                size = 0x400000000;
                break;
        case PCIE_EPBARA_32G:
                size = 0x800000000;
                break;
        case PCIE_EPBARA_64G:
                size = 0x1000000000;
                break;
        case PCIE_EPBARA_128G:
                size = 0x2000000000;
                break;
        case PCIE_EPBARA_256G:
                size = 0x4000000000;
                break;
    }

    return size;
}

/*****************************************************************************
 * Function: Configure BARs and inbound ATUs in End Point Mode
 ****************************************************************************/
static int32_t Pcie_cfgEPBars(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_AtuRegionParams regionParams;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        /* check if maximum number of configurable inbound ATUs not exceeded */
        if (pcieCfg->attrs->ibAtuNum > PCIE_MAX_NUM_IBATU)
        {
            status = SystemP_FAILURE;
        }
    }

    /* check if region index of a configurable inbound ATU not duplicated */
    for (int i = 0; i < pcieCfg->attrs->ibAtuNum; i++)
    {
        if (status == SystemP_SUCCESS)
        {
            for (int j = 0; j < pcieCfg->attrs->ibAtuNum; j++)
            {
                if ((j != i) &&
                    (pcieCfg->attrs->ibAtu[j].regionIndex == pcieCfg->attrs->ibAtu[i].regionIndex))
                {
                    status = SystemP_FAILURE;
                    break;
                }
            }
        }
        else
        {
            break;
        }
    }

    /* Configure BARs */
    for (int i = 0; i < pcieCfg->attrs->ibAtuNum; i++)
    {
        if (status == SystemP_SUCCESS)
        {
            /* Configuration of 32bit or Disabled BARs */
            if (pcieCfg->attrs->ibAtu[i].barCfg == PCIE_BARC_DISABLED ||
                pcieCfg->attrs->ibAtu[i].barCfg == PCIE_BARC_32B_IO_BAR ||
                pcieCfg->attrs->ibAtu[i].barCfg == PCIE_BARC_32B_MEM_BAR_NON_PREFETCH ||
                pcieCfg->attrs->ibAtu[i].barCfg == PCIE_BARC_32B_MEM_BAR_PREFETCH)
            {

                /* check aperture for 32bit BARs */
                if (pcieCfg->attrs->ibAtu[i].barCfg != PCIE_BARC_DISABLED &&
                    pcieCfg->attrs->ibAtu[i].barAperture > PCIE_EPBARA_2G)
                {
                    status = SystemP_FAILURE;
                    break;
                }

                status = Pcie_cfgEP1Bar(handle, pcieCfg->attrs->ibAtu[i]);
            }

            /*
             * Configuration of 64bit memory BARs
             * Only BARs on index 0,2,4 can be configured as 64bit memory BARs
             */
            else if ((pcieCfg->attrs->ibAtu[i].regionIndex % 2 == 0) &&
                     (pcieCfg->attrs->ibAtu[i].barCfg == PCIE_BARC_64B_MEM_BAR_NON_PREFETCH ||
                      pcieCfg->attrs->ibAtu[i].barCfg == PCIE_BARC_64B_MEM_BAR_PREFETCH))
            {
                /*
                 * If the BAR configuration on index 0,2,4 is defined as a 64bit
                 * memory BAR, then the BAR configuration on the next index 1,3,5
                 * must be unused or disabled, otherwise the complete BAR
                 * configuration is invalid
                 */
                for (int j=0; j < pcieCfg->attrs->ibAtuNum; j++)
                {
                    if ((pcieCfg->attrs->ibAtu[j].regionIndex == pcieCfg->attrs->ibAtu[i].regionIndex + 1) &&
                        (pcieCfg->attrs->ibAtu[j].barCfg != PCIE_BARC_DISABLED))
                    {
                        status = SystemP_FAILURE;
                        break;
                    }
                }

                if (status == SystemP_SUCCESS)
                {
                    /* check aperture for 64bit memory BARs */
                    if (pcieCfg->attrs->ibAtu[i].barAperture > PCIE_EPBARA_256G)
                    {
                        status = SystemP_FAILURE;
                        break;
                    }
                }

                if (status == SystemP_SUCCESS)
                {
                    status = Pcie_cfgEP1Bar(handle, pcieCfg->attrs->ibAtu[i]);
                }

                /*
                 * On valid 64bit BAR configuration the BARs on index 1,3,5
                 * needs to be disabled
                 */
                if (status == SystemP_SUCCESS)
                {
                    Pcie_IbAtuCfg ibAtuDisabled;
                    memset (&ibAtuDisabled, 0, sizeof(ibAtuDisabled));

                    ibAtuDisabled.regionIndex = pcieCfg->attrs->ibAtu[i].regionIndex + 1;
                    ibAtuDisabled.barCfg      = PCIE_BARC_DISABLED;
                    /* BAR aperture irrelevant since BAR disabled, set to default */
                    ibAtuDisabled.barAperture = PCIE_EPBARA_4K;

                    status = Pcie_cfgEP1Bar(handle, ibAtuDisabled);
                }
            }

            /* BARs on index 1,3,5 cannot be configured as 64bit memory BARs */
            else if ((pcieCfg->attrs->ibAtu[i].regionIndex % 2 == 1) &&
                     (pcieCfg->attrs->ibAtu[i].barCfg == PCIE_BARC_64B_MEM_BAR_NON_PREFETCH ||
                      pcieCfg->attrs->ibAtu[i].barCfg == PCIE_BARC_64B_MEM_BAR_PREFETCH))
            {
                status = SystemP_FAILURE;
                break;
            }
        }
        else
        {
            break;
        }
    }

    /* Configure Inbound ATU for each BAR */
    for (int i = 0; i < pcieCfg->attrs->ibAtuNum; i++)
    {
        if (status == SystemP_SUCCESS)
        {
            memset(&regionParams, 0, sizeof(regionParams));

            regionParams.enableRegion       = 1;
            regionParams.regionDir          = PCIE_ATU_REGION_DIR_INBOUND;
            regionParams.tlpType            = pcieCfg->attrs->ibAtu[i].tlpType;
            regionParams.lowerBaseAddr      = pcieCfg->attrs->ibAtu[i].lowerBaseAddr;
            regionParams.upperBaseAddr      = pcieCfg->attrs->ibAtu[i].upperBaseAddr;
            regionParams.regionWindowSize   = Pcie_aperture2size(pcieCfg->attrs->ibAtu[i].barAperture) - 1;
            regionParams.lowerTargetAddr    = pcieCfg->attrs->ibAtu[i].lowerTargetAddr;
            regionParams.upperTargetAddr    = pcieCfg->attrs->ibAtu[i].upperTargetAddr;

            status = Pcie_atuRegionConfig(handle, PCIE_LOCATION_LOCAL, (uint32_t) pcieCfg->attrs->ibAtu[i].regionIndex, &regionParams);

            DebugP_assert(SystemP_SUCCESS == status);
        }
        else
        {
            break;
        }
    }

    return status;
}

/*****************************************************************************
 * Function: Configure PCIe in End Point Mode
 ****************************************************************************/
int32_t Pcie_cfgEP (Pcie_Handle handle)
{
    int32_t status  = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    /* Check if PCIe device really in EP mode */
    if (pcieCfg->attrs->operationMode == PCIE_EP_MODE)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        status = Pcie_setVendorId(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        status = Pcie_setDeviceId(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        status = Pcie_setSubSysVendorId(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        status = Pcie_setSubSystemId(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        status = Pcie_setRevIdClassCode(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        /* configure EP inbound ATUs */
        status = Pcie_cfgEPBars(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        /* configure MSI multi message capability */
        status = Pcie_epSetMsiMultiMessCap(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        /* disable MSI Per-Vector Masking capability */
        status = Pcie_epDisableMsiPerVecMaskCap(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        /* set legacy interrupt pin */
        status = Pcie_setIntPin(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        /* disable FLR mechanism */
        status = Pcie_setFlr(handle, 0);
    }

    /* Configure Outbound ATU */
    for (int i = 0; i < pcieCfg->attrs->obAtuNum; i++)
    {
        if (status == SystemP_SUCCESS)
        {
            Pcie_AtuRegionParams regionParams;
            memset(&regionParams, 0, sizeof(regionParams));

            regionParams.enableRegion     = 1;
            regionParams.regionDir        = PCIE_ATU_REGION_DIR_OUTBOUND;
            regionParams.tlpType          = pcieCfg->attrs->obAtu[i].tlpType;
            regionParams.lowerBaseAddr    = pcieCfg->attrs->obAtu[i].lowerBaseAddr;
            regionParams.upperBaseAddr    = pcieCfg->attrs->obAtu[i].upperBaseAddr;
            regionParams.regionWindowSize = pcieCfg->attrs->obAtu[i].regionWindowSize;
            regionParams.lowerTargetAddr  = pcieCfg->attrs->obAtu[i].lowerTargetAddr;
            regionParams.upperTargetAddr  = pcieCfg->attrs->obAtu[i].upperTargetAddr;

            status = Pcie_atuRegionConfig (handle, PCIE_LOCATION_LOCAL, (uint32_t) pcieCfg->attrs->obAtu[i].regionIndex, &regionParams);

            DebugP_assert(SystemP_SUCCESS == status);
        }
        else
        {
            break;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* disable MSI Per-Vector Masking capability */
        status = Pcie_epDisableMsiPerVecMaskCap(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        /* disable MSIX capability */
        status = Pcie_epDisableMsixCap(handle);
    }

    if (status == SystemP_SUCCESS)
    {
        /*
         * Set CONFIG_ENABLE bit to '1', this will generate a Successful
         * Completion (SC) or Unsupported Request (UR) in response to
         * configuration request from RC.
         *
         * This effectively signals the RC that it can configure the EP now.
         *
         */
        status = Pcie_setCfgEn(handle, 1);
    }

    return status;
}

/*********************************************************************
 * FUNCTION PURPOSE: Configures a BAR Register (32bits)
 ********************************************************************/
int32_t Pcie_cfgBar(Pcie_Handle handle, const Pcie_BarCfg *barCfg)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Type0BarIdx  type0BarIdx;
    Pcie_Type1BarIdx  type1BarIdx;
    Pcie_Registers setRegs;
    Pcie_Registers getRegs;
    uint32_t barAddrField = 0;

    memset (&getRegs,     0, sizeof(getRegs));
    memset (&setRegs,     0, sizeof(setRegs));
    memset (&type0BarIdx, 0, sizeof(type0BarIdx));
    memset (&type1BarIdx, 0, sizeof(type1BarIdx));

    if(barCfg != NULL)
    {
        if(barCfg->mode == PCIE_RC_MODE)
        {
            getRegs.type1BarIdx = &type1BarIdx;
            type1BarIdx.idx = barCfg->idx;
            status = Pcie_readRegs(handle, barCfg->location, &getRegs);

            if(SystemP_SUCCESS == status)
            {
                type1BarIdx.reg.base = barAddrField;
                type1BarIdx.reg.barxc = barCfg->barxc;
                type1BarIdx.reg.barxa = barCfg->barxa;
                type1BarIdx.idx = barCfg->idx;

                setRegs.type1BarIdx = &type1BarIdx;
                status = Pcie_writeRegs(handle, barCfg->location, &setRegs);
            }

        }
        else
        {
            getRegs.type0BarIdx = &type0BarIdx;
            type0BarIdx.idx = barCfg->idx;
            status = Pcie_readRegs (handle, barCfg->location, &getRegs);

            if(SystemP_SUCCESS == status)
            {
                type0BarIdx.reg.base = barAddrField;
                type0BarIdx.reg.barxc = barCfg->barxc;
                type0BarIdx.reg.barxa = barCfg->barxa;
                type0BarIdx.idx = barCfg->idx;

                setRegs.type0BarIdx = &type0BarIdx;
                status = Pcie_writeRegs(handle, barCfg->location, &setRegs);
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static uint32_t Pcie_getNumPassBitFromWinSize(uint32_t winSize)
{
    uint32_t numPassBit = 0;
    uint32_t temp = winSize;

    while (temp!=0)
    {
        numPassBit++;
        temp >>= 1;
    }

    return (numPassBit-1);
}

/*********************************************************************
 * Returns amount of reserved space between beginning of hardware's
 * data area and the base returned by @ref Pcie_getMemSpaceRange.
 * This enables sw to position windows correctly
 *********************************************************************/
int32_t Pcie_getMemSpaceReserved (Pcie_Handle  handle, uint32_t *resSize)
{
    int32_t status = SystemP_SUCCESS;

    if (resSize != NULL)
    {
        Pcie_DeviceCfgBaseAddr *bases = Pcie_handleGetBases (handle);
        if (bases != NULL)
        {
            *resSize = bases->dataReserved;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_getMemSpaceReserved */

/*********************************************************************
 * Returns the PCIe Internal Address Range for the Memory Space.
 * This range is used for accessing memory.
 *********************************************************************/
int32_t Pcie_getMemSpaceRange (Pcie_Handle  handle, void **base, uint32_t *size)
{
    int32_t status = SystemP_SUCCESS;

    if (base != NULL)
    {
        Pcie_DeviceCfgBaseAddr *bases = Pcie_handleGetBases (handle);
        if (bases != NULL)
        {
            *base = bases->dataBase;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (size != NULL)
    {
        *size = (uint32_t)0x8000000; /* 128 MB */
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_getMemSpaceRange */

/*********************************************************************
 * Check if ATU address is aligned to region size
 ********************************************************************/
static int32_t Pcie_checkAtuAlign(uint64_t size, uint32_t lowerAddr, uint32_t upperAddr)
{
    int32_t status = SystemP_SUCCESS;

    uint64_t addr = ((uint64_t) upperAddr << 32) | (uint64_t) lowerAddr;

    if (addr & (size))
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/*********************************************************************
 * FUNCTION PURPOSE: Configures an ATU (address translation) region
 ********************************************************************/
int32_t Pcie_atuRegionConfig (Pcie_Handle handle, Pcie_Location location,
            uint32_t atuRegionIndex, const Pcie_AtuRegionParams *atuRegionParams)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_DeviceCfgBaseAddr *cfg = NULL;
    Pcie_DeviceCfgBaseAddrs *bases = NULL;
    Pcie_DevParams *params = NULL;
    CSL_pcie_rp_coreRegs *baseCfgRc = NULL;
    CSL_pcie_ep_coreRegs *baseCfgEp = NULL;

    Pcie_PlconfIatuIndexReg index;
    Pcie_PlconfIatuRegCtrl1Reg ctrl1;
    Pcie_PlconfIatuRegLowerBaseReg lowerBase;
    Pcie_PlconfIatuRegUpperBaseReg upperBase;
    Pcie_PlconfIatuRegLowerTargetReg lowerTarget;
    Pcie_PlconfIatuRegUpperTargetReg upperTarget;
    Pcie_Registers regs;

    if(handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
        cfg = Pcie_handleGetBases (handle);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (cfg != NULL)
    {
        bases = (Pcie_DeviceCfgBaseAddrs*)cfg->cfgBase;
        params = (Pcie_DevParams*)cfg->devParams;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (bases != NULL)
    {
        baseCfgRc = (CSL_pcie_rp_coreRegs *)bases->cfgBase;
        baseCfgEp = (CSL_pcie_ep_coreRegs *)bases->cfgBase;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if ((atuRegionParams != NULL) && (status == SystemP_SUCCESS))
    {
        /* Set up register pointer for interesting registers */
        memset (&regs, 0, sizeof(regs));
        regs.plconfIatuIndex = &index;

        /* Read current values for index */
        status = Pcie_readRegs (handle, location, &regs);

        /* Check ATU alignment */
        if (status == SystemP_SUCCESS)
        {
            /* target address is always used */
            status = Pcie_checkAtuAlign(atuRegionParams->regionWindowSize, atuRegionParams->lowerTargetAddr, atuRegionParams->upperTargetAddr);

            /* for EP mode, the base address is ignored for inbound mappings,
             * so don't need to check
             */
            if (status == SystemP_SUCCESS &&
                (pcieCfg->attrs->operationMode != PCIE_EP_MODE ||
                 atuRegionParams->regionDir != PCIE_ATU_REGION_DIR_INBOUND))
            {
                status = Pcie_checkAtuAlign(atuRegionParams->regionWindowSize, atuRegionParams->lowerBaseAddr, atuRegionParams->upperBaseAddr);
            }
        }

        if (status == SystemP_SUCCESS)
        {
            /* Update ATU index register with new region direction and region index.
            **/
            switch (atuRegionParams->regionDir)
            {
                case PCIE_ATU_REGION_DIR_OUTBOUND:
                {
                    index.regionDirection = 0U; /* Outbound - emulates v1 CSL */
                    break;
                }
                case PCIE_ATU_REGION_DIR_INBOUND:
                default:
                {
                    index.regionDirection = 1U; /* Inbound - emulates v1 CSL */
                    break;
                }
            }
            index.regionIndex = atuRegionIndex;

            /* Writeback the new values for index */
            status = Pcie_writeRegs (handle, location, &regs);
            if (status == SystemP_SUCCESS)
            {
                regs.plconfIatuIndex          = NULL;
                regs.plconfIatuRegCtrl1       = &ctrl1;
                regs.plconfIatuRegLowerBase   = &lowerBase;
                regs.plconfIatuRegUpperBase   = &upperBase;
                regs.plconfIatuRegLimit       = NULL; /* plconfIatuRegLimit is not supported on J7ES */
                regs.plconfIatuRegLowerTarget = &lowerTarget;
                regs.plconfIatuRegUpperTarget = &upperTarget;

                /* Read current values of rest of registers for this index */
                status = Pcie_readRegs (handle, location, &regs);
                if (status == SystemP_SUCCESS)
                {
                    if (atuRegionParams->enableRegion != 0)
                    {
                        /* Set TLP(Transaction Layer packet) type. */
                        switch (atuRegionParams->tlpType)
                        {
                            case PCIE_TLP_TYPE_MEM:
                                ctrl1.type = 2U;
                                break;
                            case PCIE_TLP_TYPE_CFG:
                                ctrl1.type = 10U;
                                break;
                            default:
                                ctrl1.type = 2U;
                                break;
                        }

                        /* Configure lower base. */
                        lowerBase.iatuRegLowerBase = atuRegionParams->lowerTargetAddr >> 8;
                        lowerBase.num_bits = Pcie_getNumPassBitFromWinSize(atuRegionParams->regionWindowSize);

                        /* Configure upper base. */
                        upperBase.iatuRegUpperBase = atuRegionParams->upperTargetAddr;

                        /* Configure window size. */
                        /*
                        limit.iatuRegLimit = (atuRegionParams->lowerBaseAddr +
                                atuRegionParams->regionWindowSize) >> 8;
                        */
                        /* Configure lower target. */
                        lowerTarget.iatuRegLowerTarget = atuRegionParams->lowerBaseAddr >> 8;
                        lowerTarget.num_bits = Pcie_getNumPassBitFromWinSize(atuRegionParams->regionWindowSize);

                        /* Configure Upper target. */
                        upperTarget.iatuRegUpperTarget = atuRegionParams->upperBaseAddr;
                    }
                    else
                    {
                        ctrl1.type                     = 2U;
                        lowerBase.iatuRegLowerBase     = 0;
                        lowerBase.num_bits             = 0;
                        upperBase.iatuRegUpperBase     = 0;
                        lowerTarget.iatuRegLowerTarget = 0;
                        lowerTarget.num_bits           = 0;
                        upperTarget.iatuRegUpperTarget = 0;
                    }

                    /* Writeback the new values */
                    status = Pcie_writeRegs (handle, location, &regs);

                    /* For INBOUND, set the PCIE_CORE_RP_I_RC_BAR_x properly */
                    if (atuRegionParams->regionDir == PCIE_ATU_REGION_DIR_INBOUND)
                    {
                        if (pcieCfg->attrs->operationMode != PCIE_EP_MODE)
                        {
                            if (atuRegionIndex == 0)
                            {
                                baseCfgRc->RC_I_RC_PCIE_BASE.I_RC_BAR_0 = atuRegionParams->lowerBaseAddr;
                            }
                            else
                            {
                                baseCfgRc->RC_I_RC_PCIE_BASE.I_RC_BAR_1 = atuRegionParams->lowerBaseAddr;
                            }
                        }
                        else
                        {
                            baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_BASE_ADDR[atuRegionIndex]
                                    = atuRegionParams->lowerBaseAddr;
                        }
                    }
                }
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_atuRegionConfig */

/*****************************************************************************
 * Configure outbound address translation for accessing remote configuration
 * space from RC
 ****************************************************************************/
static int32_t Pcie_obTransCfg(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_DeviceCfgBaseAddrs *cfgBase = NULL;
    Pcie_Config *pcieCfg = NULL;
    Pcie_DeviceCfgBaseAddr *cfg = NULL;
    Pcie_AtuRegionParams regionParams;

    uint32_t resSize;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
        cfg = Pcie_handleGetBases (handle);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if ((status == SystemP_SUCCESS) && (cfg != NULL))
    {
        cfgBase = (Pcie_DeviceCfgBaseAddrs *) cfg->cfgBase;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        memset (&regionParams, 0, sizeof(regionParams));

        status = Pcie_getMemSpaceReserved (handle, &resSize);
    }

    if(SystemP_SUCCESS == status)
    {
        if(pcieCfg->attrs->operationMode == PCIE_RC_MODE)
        {
            /*Configure OB region for remote configuration access space*/
            regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
            regionParams.tlpType      = PCIE_TLP_TYPE_CFG;
            regionParams.enableRegion = 1;

            regionParams.lowerBaseAddr = (uint32_t)cfg->dataBase + cfgBase->remoteOffset + resSize;
            regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
            regionParams.regionWindowSize = 0x0000FFFFU;

            regionParams.lowerTargetAddr = 0U;
            regionParams.upperTargetAddr = 0U;

            status = Pcie_atuRegionConfig (handle, PCIE_LOCATION_LOCAL, (uint32_t)0U, &regionParams);
        }
    }
    return status;
}

int32_t Pcie_setLanes (Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_LnkCtrlReg lnkCtrlReg;
    Pcie_Registers regs;
    uint8_t origLanes;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        regs.lnkCtrl = &lnkCtrlReg;

        status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if(SystemP_SUCCESS == status)
    {
        origLanes = lnkCtrlReg.lnkMode;
        lnkCtrlReg.lnkMode = pcieCfg->attrs->numLanes - 1;

        if (origLanes != lnkCtrlReg.lnkMode)
        {
            status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &regs);
        }
    }

    return status;
}

Pcie_Handle Pcie_open (uint32_t index)
{
    int32_t              status  = SystemP_SUCCESS;
    Pcie_Handle          handle  = NULL;
    Pcie_Config          *config = NULL;
    Pcie_Object          *object = NULL;

    if(index >= gPcieConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gPcieConfig[index];
    }

    if (SystemP_SUCCESS == status)
    {
        object = config->object;
        DebugP_assert(NULL != object);

        if (TRUE == object->cfgDone)
        {
            /* Handle is already opened */
            status = SystemP_FAILURE;
        }
    }

    if (SystemP_SUCCESS == status)
    {
        object->handle = (Pcie_Handle)config;
    }

    /*
     * Set CONFIG_ENABLE bit to '0', this will generate a Configuration Request
     * Retry Status (CRS) in response to configuration request from RC.
     *
     * This effectively signals the RC that it cannot currently configure the EP.
     *
     */
    if (status == SystemP_SUCCESS)
    {
        if (config->attrs->operationMode == PCIE_EP_MODE)
        {
            Pcie_setCfgEn(object->handle, 0);
        }
    }

    /* Initialize SERDES */
    if(status == SystemP_SUCCESS)
    {
        status = Pcie_serdesInit(object->handle, config->attrs->deviceNum);
        DebugP_assert(SystemP_SUCCESS == status);

        /* Wait for refclock to stabilize */
        ClockP_usleep(100000);
    }

    if (SystemP_SUCCESS == status)
    {
        /* Set interface mode RC or EP */
        status = Pcie_setInterfaceMode(object->handle, config->attrs->operationMode, config->attrs->gen);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    if (SystemP_SUCCESS == status)
    {
        if(config->attrs->operationMode == PCIE_RC_MODE)
        {
            /* Configure for RC mode of operation */
            status = Pcie_cfgRC(object->handle);

            DebugP_assert(SystemP_SUCCESS == status);

            status = Pcie_obTransCfg(object->handle);
        }
    }

    if (SystemP_SUCCESS == status)
    {
        /* Configure number of lanes */
        status = Pcie_setLanes(object->handle);
    }

    /* Set SRIS */
    if (status == SystemP_SUCCESS)
    {
        if (config->attrs->sris_mode == PCIE_REFCLK_SRIS_DISABLED)
        {
            Pcie_srisControl(object->handle, 0);
        }
        else if (config->attrs->sris_mode == PCIE_REFCLK_SRIS_ENABLED)
        {
            Pcie_srisControl(object->handle, 1);
        }
    }

    if (SystemP_SUCCESS == status)
    {
        /* Enable link training */
        status = Pcie_LtssmCtrl(object->handle, TRUE);

        if (SystemP_SUCCESS == status && config->attrs->operationMode == PCIE_RC_MODE)
        {
            /* Wait for link to be up */
            status = Pcie_waitLinkUp(object->handle);
        }

        if (SystemP_SUCCESS == status && config->attrs->operationMode == PCIE_RC_MODE)
        {
            /* Verify Link width & speed */
            status = Pcie_checkLinkParams(object->handle);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        object->cfgDone = TRUE;
        handle = (Pcie_Handle) config;
    }

    return handle;
}

void Pcie_close(Pcie_Handle handle)
{
    Pcie_Config *config = NULL;

    if (handle != NULL)
    {
        config = (Pcie_Config *)handle;

        config->object->cfgDone = FALSE;
    }

    return;
}

/*****************************************************************************
 * Enable or disable support for FLR in our config space
 ****************************************************************************/
static int32_t Pcie_setFlr(Pcie_Handle handle, int enable)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_DeviceCapReg deviceCap;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset(&regs, 0, sizeof(regs));
        regs.deviceCap = &deviceCap;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (enable)
            deviceCap.flrEn = 1;
        else
            deviceCap.flrEn = 0;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Enable or disable response to configuration requests (CRS)
 ****************************************************************************/
int32_t Pcie_setCfgEn(Pcie_Handle handle, int enable)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_UserCfgInitCfgReg usrCfgInitCfg;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        regs.usrCfgInitCfg = &usrCfgInitCfg;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (enable)
            usrCfgInitCfg.cfgEn = 1;
        else
            usrCfgInitCfg.cfgEn = 0;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Set or clear Slot Clock Configuration in PCIe Link Status Register
 * If set indicates the EP uses the reference clock provided on the connector,
 * else the EP uses a independent clock
 ****************************************************************************/
int32_t Pcie_setSlotClockCnfg(Pcie_Handle handle, int enable)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_LinkStatCtrlReg linkStatCtrl;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        memset (&linkStatCtrl, 0, sizeof(linkStatCtrl));

        regs.linkStatCtrl = &linkStatCtrl;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (enable)
            linkStatCtrl.slotClkCfg = 1;
        else
            linkStatCtrl.slotClkCfg = 0;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Enables downstream interrupt
 ****************************************************************************/
int32_t Pcie_setDwnStrIrq(Pcie_Handle handle, int enable)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_IntdCfgEnSys0Reg intCfgEnSys0Reg;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        regs.intCfgEnSys0Reg = &intCfgEnSys0Reg;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (enable)
            intCfgEnSys0Reg.dwnStrEn = 1;
        else
            intCfgEnSys0Reg.dwnStrEn = 0;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Enables link down status interrupt
 ****************************************************************************/
int32_t Pcie_setLnkDwnStateIrq(Pcie_Handle handle, int enable)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_IntdCfgEnSys2Reg intCfgEnSys2Reg;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        regs.intCfgEnSys2Reg = &intCfgEnSys2Reg;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (enable)
        {
            intCfgEnSys2Reg.lnkStateEn = 1;
        }
        else
        {
            intCfgEnSys2Reg.lnkStateEn = 0;
        }

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Enables power management state interrupts
 ****************************************************************************/
int32_t Pcie_setPwrStateIrq(Pcie_Handle handle, int enable)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_IntdCfgEnSys1Reg intCfgEnSys1Reg;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        regs.intCfgEnSys1Reg = &intCfgEnSys1Reg;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (enable)
        {
            intCfgEnSys1Reg.pwrStateEn = 1;
        }
        else
        {
            intCfgEnSys1Reg.pwrStateEn = 0;
        }

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Enables PCIe hot reset interrupts
 ****************************************************************************/
int32_t Pcie_setHotResetIrq (Pcie_Handle handle, int enable)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_IntdCfgEnSys2Reg intCfgEnSys2Reg;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        regs.intCfgEnSys2Reg = &intCfgEnSys2Reg;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (enable)
        {
            intCfgEnSys2Reg.hotRstEn = 1;
        }
        else
        {
            intCfgEnSys2Reg.hotRstEn = 0;
        }

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Sets legacy interrupt (INTx) according to configuration
 ****************************************************************************/
static int32_t Pcie_setIntPin(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_IntPinReg intPin;
    Pcie_Registers regs;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    memset (&regs, 0, sizeof(regs));
    regs.intPin = &intPin;

    status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);

    if (status == SystemP_SUCCESS)
    {
        switch (pcieCfg->attrs->intPin)
        {
            case PCIE_INT_PINNONE:
                    intPin.intPin = 0;
                    break;
            case PCIE_INT_PINA:
                    intPin.intPin = 1;
                    break;
            case PCIE_INT_PINB:
                    intPin.intPin = 2;
                    break;
            case PCIE_INT_PINC:
                    intPin.intPin = 3;
                    break;
            case PCIE_INT_PIND:
                    intPin.intPin = 4;
                    break;
        }

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Acknowledge downstream interrupt
 ****************************************************************************/
int32_t Pcie_ackDwnStrIrq(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_IntdCfgStatusSys0Reg intCfgStatusSys0Reg;
    Pcie_VndrSpecCntrlReg vndrSpecCntrl;
    Pcie_IrqEOIReg irqEOI;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.intCfgStatusSys0Reg = &intCfgStatusSys0Reg;
        regs.vndrSpecCntrl = &vndrSpecCntrl;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (intCfgStatusSys0Reg.statusDwnStr)
        {
            regs.intCfgStatusSys0Reg = NULL;

            /* clear HTI bit first so that downstream IRQ isn't signaled anymore */
            vndrSpecCntrl.hti = 0;

            status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);

            if (status == SystemP_SUCCESS)
            {
                regs.vndrSpecCntrl = NULL;

                /* after HTI bit is cleared signal EOI to PCIe INTD */
                irqEOI.EOI  = 0x00;
                regs.irqEOI = &irqEOI;

                status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
            }
        }
    }

    return status;
}

/*****************************************************************************
 * Acknowledge link down status interrupt
 ****************************************************************************/
int32_t Pcie_ackLnkDwnStateIrq(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_IntdCfgStatusSys2Reg intCfgStatusSys2Reg;
    Pcie_IntdCfgStatusClrSys2Reg intCfgStatusClrSys2Reg;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.intCfgStatusSys2Reg    = &intCfgStatusSys2Reg;
        regs.intCfgStatusClrSys2Reg = &intCfgStatusClrSys2Reg;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (intCfgStatusSys2Reg.statusLnkState)
        {
            regs.intCfgStatusSys2Reg = NULL;
            intCfgStatusClrSys2Reg.clrStatusLnkState = 1;

            status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
        }
    }

    return status;
}

/*****************************************************************************
 * Acknowledge power management state interrupt
 ****************************************************************************/
int32_t Pcie_ackPwrStateIrq(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_IntdCfgStatusSys1Reg intCfgStatusSys1Reg;
    Pcie_UserCfgPmCmdReg usrCfgPmCmd;
    Pcie_IrqEOIReg irqEOI;
    Pcie_Registers regs;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.intCfgStatusSys1Reg = &intCfgStatusSys1Reg;
        regs.usrCfgPmCmd = &usrCfgPmCmd;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        if (intCfgStatusSys1Reg.statusPwrState)
        {
            regs.intCfgStatusSys1Reg = NULL;

            usrCfgPmCmd.pwrStateChgAck = 0x01;
            status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &regs);

            usrCfgPmCmd.pwrStateChgAck = 0x00;
            status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &regs);

            regs.usrCfgPmCmd = NULL;

            irqEOI.EOI  = 0x03;
            regs.irqEOI = &irqEOI;
            status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
        }
    }

    return status;
}

/*****************************************************************************
 * Read and split up the Endpoint Interrupt Request Set swRegister
 ****************************************************************************/
static int32_t Pcie_readEpIrqSetReg(const CSL_user_cfgRegs *baseAddr, Pcie_EpIrqSetReg *swReg,
                                int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->LEGACY_INTR_SET;

        switch (swRegNum)
        {
            case 0:
                PCIE_GETBITS(val, CSL_USER_CFG_LEGACY_INTR_SET_INTA_IN, swReg->epIrqSet);
                break;
            case 1:
                PCIE_GETBITS(val, CSL_USER_CFG_LEGACY_INTR_SET_INTB_IN, swReg->epIrqSet);
                break;
            case 2:
                PCIE_GETBITS(val, CSL_USER_CFG_LEGACY_INTR_SET_INTC_IN, swReg->epIrqSet);
                break;
            case 3:
                PCIE_GETBITS(val, CSL_USER_CFG_LEGACY_INTR_SET_INTD_IN, swReg->epIrqSet);
                break;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readEpIrqSetReg */

/*****************************************************************************
 * Read and split up the Endpoint Interrupt Request Clear swRegister
 ****************************************************************************/
static int32_t Pcie_readEpIrqClrReg (const CSL_user_cfgRegs *baseAddr, Pcie_EpIrqClrReg *swReg,
                                    int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->LEGACY_INTR_SET;

        switch (swRegNum)
        {
            case 0:
                PCIE_GETBITS(val, CSL_USER_CFG_LEGACY_INTR_SET_INTA_IN, swReg->epIrqClr);
                break;
            case 1:
                PCIE_GETBITS(val, CSL_USER_CFG_LEGACY_INTR_SET_INTB_IN, swReg->epIrqClr);
                break;
            case 2:
                PCIE_GETBITS(val, CSL_USER_CFG_LEGACY_INTR_SET_INTC_IN, swReg->epIrqClr);
                break;
            case 3:
                PCIE_GETBITS(val, CSL_USER_CFG_LEGACY_INTR_SET_INTD_IN, swReg->epIrqClr);
                break;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readEpIrqClrReg */

/*****************************************************************************
 * Read EOI Vector swRegister
 ****************************************************************************/
static int32_t Pcie_readRcIrqEoi (CSL_user_cfgRegs *baseAddr, Pcie_IrqEOIReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->EOI_VECTOR;

        PCIE_GETBITS(val, CSL_USER_CFG_EOI_REG_EOI_VECTOR, swReg->EOI);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/*****************************************************************************
 * Read and split up the Status and Command register
 ****************************************************************************/
static int32_t Pcie_readStatusCmdReg(volatile const uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                                    Pcie_StatusCmdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_STATUS_COMMAND_REGISTER != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_STATUS_COMMAND_REGISTER;

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_ISE, swReg->ioSp);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MSE, swReg->memSp);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_BE, swReg->busMs);

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_PERE, swReg->resp);

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SE, swReg->serrEn);

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IMD, swReg->dis);

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IS, swReg->stat);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_CL, swReg->capList);

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MDPE, swReg->parError);

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_STA, swReg->sigTgtAbort);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RTA, swReg->tgtAbort);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RMA, swReg->mstAbort);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SSE, swReg->sysError);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_DPE, swReg->parity);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readStatusCmdReg */


/*****************************************************************************
 * Read and split up the Base Address register
 ****************************************************************************/
static int32_t Pcie_readBaseAddrReg(volatile const uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                                    Pcie_BaseAddrReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_STATUS_COMMAND_REGISTER != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_STATUS_COMMAND_REGISTER;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_MSI0, swReg->msio);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_R7, swReg->r7);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_S0, swReg->s0);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_P0, swReg->p0);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_R8, swReg->r8);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_BAMR0, swReg->bamr0);

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_BAMRW, swReg->bamrw);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readBaseAddrReg */


/*****************************************************************************
 * Read and split up the Class Code and Revision ID register
 ****************************************************************************/
static int32_t Pcie_readRevIdReg(volatile const uint32_t *hwReg_CLASSCODE_REVISIONID,
                                        Pcie_RevIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_CLASSCODE_REVISIONID != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_CLASSCODE_REVISIONID;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_CC, swReg->classCode);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_RID, swReg->revId);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_PIB, swReg->progIntrface);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_SCC, swReg->subClassCode);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readRevIdReg */


/*****************************************************************************
 * Read and split up the type 0 BAR register
 ****************************************************************************/
static int32_t Pcie_readType0BarReg( const CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                                    int32_t barNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->LM_I_REGF_LM_PCIE_BASE.I_PF_BAR_CONFIG_REG[0].I_PF_BAR_CONFIG_REG[barNum/4];

        /* Get the BARxA and BARxC accroding to the barNum */
        switch (barNum%4)
        {
            case 0:
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0C, swReg->barxc);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0A, swReg->barxa);
                break;
            case 1:
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1C, swReg->barxc);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1A, swReg->barxa);
                break;
            case 2:
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2C, swReg->barxc);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2A, swReg->barxa);
                break;
            case 3:
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3C, swReg->barxc);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3A, swReg->barxa);
                break;
            default:
                swReg->prefetch = 0;
                swReg->base = 0;
                break;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readType0BarReg */

/*****************************************************************************
 * Read and split up the type1 BAR register
 ****************************************************************************/
static int32_t Pcie_readType1BarReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                                    int32_t barNum)
{
    int32_t status = SystemP_SUCCESS;
    CSL_pcie_rp_coreRegs *baseAddrRc = NULL;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        baseAddrRc = (CSL_pcie_rp_coreRegs *)baseAddr;
        val = swReg->raw = baseAddrRc->LM_I_REGF_LM_PCIE_BASE.I_RC_BAR_CONFIG_REG;

        /* Get the BARxA and BARxC accroding to the barNum */
        switch (barNum)
        {
            case 0:
                PCIE_GETBITS(val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0C, swReg->barxc);
                PCIE_GETBITS(val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0A, swReg->barxa);
                break;
            case 1:
                PCIE_GETBITS(val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1C, swReg->barxc);
                PCIE_GETBITS(val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1A, swReg->barxa);
                break;
            default:
                swReg->prefetch = 0;
                swReg->base = 0;
                break;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readType1BarReg */

/*****************************************************************************
 * Read and split up the BAR 32bits register
 ****************************************************************************/
static int32_t Pcie_readType0Bar32bitReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_Bar32bitReg *swReg,
                                            int32_t barNum, uint8_t pfNum)
{
    int32_t status = SystemP_SUCCESS;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        swReg->reg32 = swReg->raw = baseAddr->EP_PF_I_PCIE[pfNum].EP_PF_I_PCIE_BASE.I_BASE_ADDR[barNum];
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readType0Bar32bitReg */

/*****************************************************************************
 * Read and split up the Device Capability register
 ****************************************************************************/
static int32_t Pcie_readDevCapReg (volatile const uint32_t *hwReg_DEV_CAS, Pcie_DeviceCapReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_DEV_CAS != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_DEV_CAS;

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_FLRC, swReg->flrEn);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CPLS, swReg->pwrLimitScale);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CSP,  swReg->pwrLimitValue);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_RER,  swReg->errRpt);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL1L, swReg->l1Latency);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL0L, swReg->l0Latency);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_ETFS, swReg->extTagFld);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_PFS,  swReg->phantomFld);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_MP,   swReg->maxPayldSz);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readDevCapReg */

/*****************************************************************************
 * Read and split up the Device Status and Control register
 ****************************************************************************/
static int32_t Pcie_readDevStatCtrlReg (volatile const uint32_t *hwReg_DEV_CAS, Pcie_DevStatCtrlReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_DEV_CAS != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_DEV_CAS;

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ECER, swReg->corErRp);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENFER, swReg->nFatalErRp);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EFER, swReg->fatalErRp);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EURR, swReg->reqRp);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ERO, swReg->relaxed);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MP, swReg->maxPayld);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE, swReg->xtagEn);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE, swReg->phantomEn);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE, swReg->auxPwrEn);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENS, swReg->noSnoop);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MRR, swReg->maxSz);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_CED, swReg->corrEr);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_NFED, swReg->nFatalEr);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_FED, swReg->fatalEr);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_URD, swReg->rqDet);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_APD, swReg->auxPwr);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_TP, swReg->tpend);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readDevStatCtrlReg */

/*****************************************************************************
 * Read and split up the Link Capabilities register
 ****************************************************************************/
static int32_t Pcie_readLinkCapReg (volatile const uint32_t *hwReg_LNK_CAP, Pcie_LinkCapReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_LNK_CAP != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_LNK_CAP;

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_MLS, swReg->maxLinkSpeed);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_MLW, swReg->maxLinkWidth);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPM, swReg->asLinkPm);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_L0EL, swReg->losExitLat);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_L1EL, swReg->l1ExitLat);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_CPM, swReg->clkPwrMgmt);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_SERC, swReg->downErrRepCap);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_DARC, swReg->dllRepCap);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_LBNC, swReg->bwNotifyCap);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPMOC, swReg->aspmOptComp);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPMOC, swReg->portNum);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readLinkCapReg */

/*****************************************************************************
 * Read and split up the Link Status and Control register
 ****************************************************************************/
static int32_t Pcie_readLinkStatCtrlReg (volatile const uint32_t *hwReg_LNK_CAS, Pcie_LinkStatCtrlReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_LNK_CAS != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_LNK_CAS;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_CAP_STRUCT_I_LINK_CTRL_STATUS_NLS, swReg->linkSpeed);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_CAP_STRUCT_I_LINK_CTRL_STATUS_NLW, swReg->negotiatedLinkWd);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_CAP_STRUCT_I_LINK_CTRL_STATUS_SCC, swReg->slotClkCfg);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readLinkStatCtrlReg */

/*****************************************************************************
 * Read and split up the Advanced Error Capabilities and Control register
 ****************************************************************************/
static int32_t Pcie_readAccrReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_AccrReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    CSL_pcie_rp_coreRegs *baseAddrRp;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        baseAddrRp = (CSL_pcie_rp_coreRegs *)baseAddr;
        val = swReg->raw = baseAddrRp->RC_I_RC_PCIE_BASE.I_ADV_ERR_CAP_CTL;

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRE, swReg->multHdrEn);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRC, swReg->multHdrCap);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEC, swReg->chkEn);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_ECC, swReg->chkCap);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEG, swReg->genEn);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EGC, swReg->genCap);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_FEP, swReg->erPtr);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readAccrReg */

/*****************************************************************************
 * Read and split up the PL CONF iATU Region Control 1 register
 ****************************************************************************/
static int32_t Pcie_readPlconfIatuRegCtrl1Reg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                                            Pcie_PlconfIatuRegCtrl1Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;
    uint8_t regionIndex = 0;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        regionIndex = simIatuWindow->regionIndex;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                val = swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].DESC0;

                /* TLP type bit[3:0] - 0xA - Config, 0x2 - MEmory WR/RD, 0xC - message, 0xD - Vendor message, 0x6 */
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_DESC0_TT, swReg->type);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_DESC0_PTC, swReg->tc);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_DESC0_ATTR, swReg->attr);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_DESC0_FNUM, swReg->functionNumber);
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }

        /* Set unused fields to 0 */
        swReg->at = 0u;
    }

    return status;
} /* Pcie_readPlconfIatuRegCtrl1Reg */

/*****************************************************************************
 * Read and split up the PL CONF iATU Region Lower Base Address register
 ****************************************************************************/
static int32_t Pcie_readPlconfIatuRegLowerBaseReg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                                                    Pcie_PlconfIatuRegLowerBaseReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;
    uint8_t regionIndex = 0;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        regionIndex = simIatuWindow->regionIndex;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        if (((simIatuWindow->regionDirection == 0) && (regionIndex < OB_REGION_MAX))
            || ((simIatuWindow->regionDirection == 1) && (regionIndex < IB_REGION_MAX_EP)))
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                val = swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].ADDR0;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR0_DATA, swReg->iatuRegLowerBase);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR0_NUM_BITS, swReg->num_bits);
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                val = swReg->raw = baseAddr->ATU_FUNC_WRAPPER_IB_EP[0][regionIndex].ADDR0;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR0_DATA, swReg->iatuRegLowerBase);
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
} /* Pcie_readPlconfIatuRegLowerBaseReg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Limit Address register
 ****************************************************************************/
static int32_t Pcie_readPlconfIatuRegUpperBaseReg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                                            Pcie_PlconfIatuRegUpperBaseReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;
    uint8_t regionIndex = 0;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        regionIndex = simIatuWindow->regionIndex;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        if (((simIatuWindow->regionDirection == 0) && (regionIndex < OB_REGION_MAX))
            || ((simIatuWindow->regionDirection == 1) && (regionIndex < IB_REGION_MAX_EP)))
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                val = swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].ADDR1;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR1_DATA, swReg->iatuRegUpperBase);
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                val = swReg->raw = baseAddr->ATU_FUNC_WRAPPER_IB_EP[0][regionIndex].ADDR1;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR1_DATA, swReg->iatuRegUpperBase);
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
} /* Pcie_readPlconfIatuRegUpperBaseReg */

/*****************************************************************************
 * Read and split up the PL CONF iATU Region Lower Target Address register
 ****************************************************************************/
static int32_t Pcie_readPlconfIatuRegLowerTargetReg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                                                    Pcie_PlconfIatuRegLowerTargetReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;
    uint8_t regionIndex = 0;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        regionIndex = simIatuWindow->regionIndex;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                val = swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].AXI_ADDR0;

                PCIE_GETBITS(val, CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_DATA, swReg->iatuRegLowerTarget);
                PCIE_GETBITS(val, CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_REGION_SIZE, swReg->num_bits);
            }
            else
            {
                /* INBOUND */
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
} /* Pcie_readPlconfIatuRegLowerTargetReg */

/*****************************************************************************
 * Read and split up the PL CONF iATU Region Upper Target Address register
 ****************************************************************************/
static int32_t Pcie_readPlconfIatuRegUpperTargetReg (const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                                    Pcie_PlconfIatuRegUpperTargetReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;
    uint8_t regionIndex = 0;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        regionIndex = simIatuWindow->regionIndex;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {

        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                val = swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].AXI_ADDR1;

                PCIE_GETBITS(val, CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR1_DATA, swReg->iatuRegUpperTarget);
            }
            else
            {
                /* INBOUND */
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
} /* Pcie_readPlconfIatuRegUpperTargetReg */

/*****************************************************************************
 * Read and split up the LINKSTATUS swRegister
 ****************************************************************************/
static int32_t Pcie_readLinkStatusReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_TiConfDeviceCmdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    CSL_user_cfgRegs *userCfg;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        userCfg = (CSL_user_cfgRegs *)baseAddr;
        val = swReg->raw = userCfg->LINKSTATUS;

        PCIE_GETBITS(val, CSL_USER_CFG_LINKSTATUS_LTSSM_STATE, swReg->ltssmState);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readLinkStatusReg */

/* Read MSI capabilities Register */
/*****************************************************************************
 * Read and split up the Message Signaled Interrupt Capability register
 ****************************************************************************/
static int32_t Pcie_readMsiCapReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiCapReg *swReg)
{
    uint32_t *regVal = (uint32_t *)baseAddr;
    uint32_t val = swReg->raw = *regVal;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_CID1, swReg->capId);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_CP1, swReg->nextCap);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_ME, swReg->msiEn);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_MMC, swReg->multMsgCap);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_MME, swReg->multMsgEn);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_BAC64, swReg->en64bit);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_MC, swReg->pvmEn);

    return SystemP_SUCCESS;
} /* Pcie_readMsiCapReg */

/*****************************************************************************
 * Read and split up the Address of MSI write TLP req lower 32 bits register
 ****************************************************************************/
static int32_t Pcie_readMsiLo32Reg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiLo32Reg *swReg)
{
    CSL_pcie_rp_coreRegs *baseAddrRc = (CSL_pcie_rp_coreRegs *)baseAddr;
    uint32_t val = swReg->raw = baseAddrRc->RC_I_RC_PCIE_BASE.I_MSI_MSG_LOW_ADDR;

    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR_MAL, swReg->addr);

    return SystemP_SUCCESS;
} /* Pcie_readMsiLo32Reg */

/*****************************************************************************
 * Read and split up the Address of MSI write TLP req upper 32 bits register
 ****************************************************************************/
static int32_t Pcie_readMsiUp32Reg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiUp32Reg *swReg)
{
    CSL_pcie_rp_coreRegs *baseAddrRc = (CSL_pcie_rp_coreRegs *)baseAddr;
    uint32_t val = swReg->raw = baseAddrRc->RC_I_RC_PCIE_BASE.I_MSI_MSG_HI_ADDR;

    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_HI_ADDR_MAH, swReg->addr);

    return SystemP_SUCCESS;
} /* Pcie_readMsiUp32Reg */

/*****************************************************************************
 * Read and split up the Data of MSI write TLP req register
 ****************************************************************************/
static int32_t Pcie_readMsiDataReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiDataReg *swReg)
{
    CSL_pcie_rp_coreRegs *baseAddrRc = (CSL_pcie_rp_coreRegs *)baseAddr;
    uint32_t val = swReg->raw = baseAddrRc->RC_I_RC_PCIE_BASE.I_MSI_MSG_DATA;

    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA_MD, swReg->data);

    return SystemP_SUCCESS;
} /* Pcie_readMsiDataReg */

/* Read MSIx capabilities Register */
/*****************************************************************************
 * Read and split up the MSIx Capability register
 ****************************************************************************/
static int32_t Pcie_readMsixCapReg (const CSL_pcie_ep_coreRegs *baseAddr,Pcie_MsixCapReg *swReg)
{
    uint32_t *regVal = (uint32_t *)baseAddr;
    uint32_t val = swReg->raw = *regVal;

    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXE, swReg->msixEn);
    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_FM, swReg->maskIrq);
    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXTS, swReg->msixTblSize);
    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CP, swReg->nextCap);
    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CID, swReg->capId);

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up the MSI table offset register
 ****************************************************************************/
static int32_t Pcie_readMsixTblOffset (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsixTblOffset *swReg)
{
    uint32_t *regVal = (uint32_t *)baseAddr;
    uint32_t val = swReg->raw = *regVal;

    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_TO, swReg->offset);
    PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_BARI, swReg->barIndex);

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up the power management control status register
 ****************************************************************************/
static int32_t Pcie_readPmMgmtCtrlStatReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PMCapCtlStatReg *swReg)
{
    uint32_t *regVal = (uint32_t *)baseAddr;
    uint32_t val = swReg->raw = *regVal;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_DR,   swReg->dataReg);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_PMES, swReg->pmeStatus);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_PE,   swReg->pmeEn);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_NSR,  swReg->noSoftRst);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_PS,   swReg->pwrState);

    return SystemP_SUCCESS;
}

int32_t Pcie_getVendorId(Pcie_Handle handle, Pcie_Location location,
                            uint32_t *vendorId, uint32_t *deviceId)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers getRegs;
    Pcie_VndDevIdReg vendorDevId;

    memset (&getRegs, 0, sizeof(getRegs));

    getRegs.vndDevId = &vendorDevId;

    status = Pcie_readRegs(handle, location, &getRegs);

    if (SystemP_SUCCESS == status)
    {
        if ((NULL != vendorId) && (NULL != deviceId))
        {
            *vendorId = vendorDevId.vndId;
            *deviceId = vendorDevId.devId;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

static int32_t Pcie_setVendorId(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_LmVendorIdReg lmVendorIdReg;
    Pcie_Registers regs;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        memset (&lmVendorIdReg, 0, sizeof(lmVendorIdReg));

        regs.lmVendorIdReg = &lmVendorIdReg;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        lmVendorIdReg.vid = pcieCfg->attrs->vendorId;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

static int32_t Pcie_setDeviceId(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_VndDevIdReg vndDevId;
    Pcie_Registers regs;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        memset (&vndDevId, 0, sizeof(vndDevId));

        regs.vndDevId = &vndDevId;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        vndDevId.devId = pcieCfg->attrs->deviceId;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

static int32_t Pcie_setSubSysVendorId(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_LmVendorIdReg lmVendorIdReg;
    Pcie_Registers regs;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        memset (&lmVendorIdReg, 0, sizeof(lmVendorIdReg));

        regs.lmVendorIdReg = &lmVendorIdReg;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        lmVendorIdReg.svid = pcieCfg->attrs->subSysVendorId;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

static int32_t Pcie_setSubSystemId(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_SubIdReg subId;
    Pcie_Registers regs;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        memset (&subId, 0, sizeof(subId));

        regs.subId = &subId;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        subId.subId = pcieCfg->attrs->subSystemId;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

static int32_t Pcie_setRevIdClassCode(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_RevIdReg revId;
    Pcie_Registers regs;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));
        memset (&revId, 0, sizeof(revId));

        regs.revId = &revId;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        revId.classCode    = pcieCfg->attrs->classCode;
        revId.subClassCode = pcieCfg->attrs->subClassCode;
        revId.progIntrface = pcieCfg->attrs->progIntrface;
        revId.revId        = pcieCfg->attrs->revId;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

static int32_t Pcie_readVendorDevIdReg(volatile const uint32_t *hwReg_VENDOR_ID_DEV_ID,
                                    Pcie_VndDevIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_VENDOR_ID_DEV_ID != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_VENDOR_ID_DEV_ID;

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_VID, swReg->vndId);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_DID, swReg->devId);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Pcie_readSubIdReg(volatile const uint32_t *hwReg_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_ID, Pcie_SubIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_ID != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_ID;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_I_SVID, swReg->subVndId);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_I_SID, swReg->subId);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Pcie_readIntPinReg(volatile const uint32_t *hwReg_INTRPT_LINE_INTRPT_PIN,
                                  Pcie_IntPinReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_INTRPT_LINE_INTRPT_PIN != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_INTRPT_LINE_INTRPT_PIN;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_ILR, swReg->intLine);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_IPR, swReg->intPin);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/*****************************************************************************
 * Read and split up the User Config Init Cfg register
 ****************************************************************************/
static int32_t Pcie_readUsrInitCfgReg (const CSL_user_cfgRegs *baseAddr, Pcie_UserCfgInitCfgReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->INITCFG;

        PCIE_GETBITS(val, CSL_USER_CFG_INITCFG_CONFIG_ENABLE, swReg->cfgEn);
        PCIE_GETBITS(val, CSL_USER_CFG_INITCFG_VC_COUNT, swReg->vcCnt);
        PCIE_GETBITS(val, CSL_USER_CFG_INITCFG_MAX_EVAL_ITERATION, swReg->maxEvalItr);
        PCIE_GETBITS(val, CSL_USER_CFG_INITCFG_BYPASS_PHASE23, swReg->bypassPh23);
        PCIE_GETBITS(val, CSL_USER_CFG_INITCFG_BYPASS_REMOTE_TX_EQUALIZATION, swReg->bypassRmTxEQ);
        PCIE_GETBITS(val, CSL_USER_CFG_INITCFG_SUPPORTED_PRESET, swReg->suppPrst);
        PCIE_GETBITS(val, CSL_USER_CFG_INITCFG_DISABLE_GEN3_DC_BALANCE, swReg->disableGen3DcBlnc);
        PCIE_GETBITS(val, CSL_USER_CFG_INITCFG_SRIS_ENABLE, swReg->srisEn);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readUsrInitCfgReg */

/*****************************************************************************
 * Read and split up the User Config Pm Cmd register
 ****************************************************************************/
static int32_t Pcie_readUsrPmCmdReg (const CSL_user_cfgRegs *baseAddr, Pcie_UserCfgPmCmdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->PMCMD;

        PCIE_GETBITS(val, CSL_USER_CFG_PMCMD_PWR_STATE_CHANGE_ACK, swReg->pwrStateChgAck);
        PCIE_GETBITS(val, CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1_SUBSTATE, swReg->clientReqExtL1Substate);
        PCIE_GETBITS(val, CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1, swReg->clientReqExtL1);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readUsrPmCmdReg */

/*****************************************************************************
 * Read and split up the Intd Config Enable Sys0 register
 ****************************************************************************/
static int32_t Pcie_readIntdCfgEnSys0Reg (const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys0Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->ENABLE_REG_SYS_0;

        PCIE_GETBITS(val, CSL_INTD_CFG_ENABLE_REG_SYS_0_ENABLE_SYS_EN_PCIE_DOWNSTREAM, swReg->dwnStrEn);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readIntdCfgEnSys0Reg */

/*****************************************************************************
 * Read and split up the Intd Config Enable Sys1 register
 ****************************************************************************/
static int32_t Pcie_readIntdCfgEnSys1Reg (const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys1Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->ENABLE_REG_SYS_1;

        PCIE_GETBITS(val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_PWR_STATE, swReg->pwrStateEn);
        PCIE_GETBITS(val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_LEGACY_3,  swReg->legacy3En);
        PCIE_GETBITS(val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_LEGACY_2,  swReg->legacy2En);
        PCIE_GETBITS(val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_LEGACY_1,  swReg->legacy1En);
        PCIE_GETBITS(val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_LEGACY_0,  swReg->legacy0En);
        PCIE_GETBITS(val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_FLR,       swReg->flrEn);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readIntdCfgEnSys1Reg */

/*****************************************************************************
 * Read and split up the Intd Config Enable Sys2 register
 ****************************************************************************/
static int32_t Pcie_readIntdCfgEnSys2Reg (const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys2Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->ENABLE_REG_SYS_2;

        PCIE_GETBITS(val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_DPA,        swReg->dpaEn);
        PCIE_GETBITS(val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_ERROR_0,    swReg->err0En);
        PCIE_GETBITS(val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_ERROR_1,    swReg->err1En);
        PCIE_GETBITS(val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_ERROR_2,    swReg->err2En);
        PCIE_GETBITS(val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_HOT_RESET,  swReg->hotRstEn);
        PCIE_GETBITS(val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_Link_STATE, swReg->lnkStateEn);
        PCIE_GETBITS(val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_PTM,        swReg->ptmEn);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readIntdCfgEnSys2Reg */

/*****************************************************************************
 * Read and split up the Intd Config Status Sys0 register
 ****************************************************************************/
static int32_t Pcie_readIntdCfgStatusSys0Reg (const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys0Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->STATUS_REG_SYS_0;

        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_0_STATUS_SYS_PCIE_DOWNSTREAM, swReg->statusDwnStr);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readIntdCfgStatusSys0Reg */

/*****************************************************************************
 * Read and split up the Intd Config Status Sys1 register
 ****************************************************************************/
static int32_t Pcie_readIntdCfgStatusSys1Reg (const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys1Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->STATUS_REG_SYS_1;

        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_PWR_STATE, swReg->statusPwrState);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_LEGACY_3,  swReg->statusLegacy3);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_LEGACY_2,  swReg->statusLegacy2);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_LEGACY_1,  swReg->statusLegacy1);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_LEGACY_0,  swReg->statusLegacy0);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_FLR,       swReg->statusFlr);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readIntdCfgStatusSys1Reg */

/*****************************************************************************
 * Read and split up the Intd Config Status Sys2 register
 ****************************************************************************/
static int32_t Pcie_readIntdCfgStatusSys2Reg (const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys2Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->STATUS_REG_SYS_2;

        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_PTM,        swReg->statusPtm);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_LINK_STATE, swReg->statusLnkState);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_HOT_RESET,  swReg->statusHotRst);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_ERROR_2,    swReg->statusErr2);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_ERROR_1,    swReg->statusErr1);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_ERROR_0,    swReg->statusErr0);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_DPA,        swReg->statusDpa);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readIntdCfgStatusSys2Reg */

/*****************************************************************************
 * Read and split up the Intd Config Status Clear Sys2 register
 ****************************************************************************/
static int32_t Pcie_readIntdCfgStatusClrSys2Reg (const CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusClrSys2Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->STATUS_CLR_REG_SYS_2;

        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_PTM,        swReg->clrStatusPtm);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_LINK_STATE, swReg->clrStatusLnkState);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_HOT_RESET,  swReg->clrStatusHotRst);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_ERROR_2,    swReg->clrStatusErr2);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_ERROR_1,    swReg->clrStatusErr1);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_ERROR_0,    swReg->clrStatusErr0);
        PCIE_GETBITS(val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_DPA,        swReg->clrStatusDpa);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readIntdCfgStatusClrSys2Reg */

/*****************************************************************************
 * Read and split up the Vendor Specific Control register
 ****************************************************************************/
static int32_t Pcie_readVndrSpecCntrlReg (volatile const uint32_t *hwReg_VENDOR_SPECIFIC_CONTROL_REG, Pcie_VndrSpecCntrlReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_VENDOR_SPECIFIC_CONTROL_REG != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_VENDOR_SPECIFIC_CONTROL_REG;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_REGF_VSEC_STRUCT_I_VENDOR_SPECIFIC_CONTROL_REG_VSEC_COUT, swReg->vsecCout);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_REGF_VSEC_STRUCT_I_VENDOR_SPECIFIC_CONTROL_REG_HTI,       swReg->hti);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EP_PF0_I_REGF_VSEC_STRUCT_I_VENDOR_SPECIFIC_CONTROL_REG_VSEC_CIN,  swReg->vsecCin);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readVndrSpecCntrlReg */

/*****************************************************************************
 * Read and split up the LM Vendor ID and Subsystem Vendor ID register
 ****************************************************************************/
static int32_t Pcie_readLmVendorIdReg (volatile const uint32_t *hwReg_I_VENDOR_ID_REG, Pcie_LmVendorIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_I_VENDOR_ID_REG != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_I_VENDOR_ID_REG;

        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_SVID, swReg->svid);
        PCIE_GETBITS(val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_VID,  swReg->vid);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readLmVendorIdReg */

/*********************************************************************
 * Reads any register
 ********************************************************************/
int32_t Pcie_readRegs (Pcie_Handle handle, Pcie_Location location, Pcie_Registers *readRegs)
{
    int32_t status = SystemP_SUCCESS;
    int32_t i;
    Pcie_Config *pcieCfg = NULL;
    Pcie_DeviceCfgBaseAddr *cfg = NULL;
    Pcie_DeviceCfgBaseAddrs *bases = NULL;
    Pcie_DevParams *params = NULL;

    Pcie_PlconfIatuIndexReg *simIatuWindow = NULL;

    /* Base Address for the Config Space
       These registers can be Local/Remote and Type0(EP)/Type1(RC) */
    CSL_pcie_ep_coreRegs *baseCfgEp = NULL;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
        cfg = Pcie_handleGetBases (handle);

        simIatuWindow = &pciev0LocalObj.simIatuWindow[pcieCfg->attrs->deviceNum];

        if(cfg != NULL)
        {
            bases = (Pcie_DeviceCfgBaseAddrs*)cfg->cfgBase;
            params = (Pcie_DevParams*)cfg->devParams;
        }
        else
        {
            status = SystemP_FAILURE;
        }

        if (status == SystemP_SUCCESS)
        {
            baseCfgEp = (CSL_pcie_ep_coreRegs *)bases->cfgBase;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        /* Get base address for Local or Remote config space */
        if (location != PCIE_LOCATION_LOCAL)
        {
            char *remoteBase  = (char *)cfg->dataBase + bases->remoteOffset;

            if (remoteBase != NULL)
            {
                baseCfgEp = (CSL_pcie_ep_coreRegs *)(remoteBase);
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }

    /*****************************************************************************************
    * Application Registers
    *****************************************************************************************/
    if ((status == SystemP_SUCCESS) && (readRegs->pid != NULL))
    {
        status = Pcie_readPidReg ((CSL_pcie_ep_coreRegs *)params->userCfgBase, readRegs->pid);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->cmdStatus != NULL))
    {
        status = Pcie_readCmdStatusReg ((CSL_pcie_ep_coreRegs *)params->userCfgBase, readRegs->cmdStatus);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->cfgTrans != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->ioBase != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tlpCfg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->rstCmd != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->ptmCfg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->pmCmd != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->pmCfg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->actStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->obSize != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->diagCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->endian != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->priority != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->irqEOI != NULL))
    {
        status = Pcie_readRcIrqEoi ((CSL_user_cfgRegs *)params->userCfgBase, readRegs->irqEOI);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->msiIrq != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->epIrqSet != NULL))
    {
        status = Pcie_readEpIrqSetReg ((CSL_user_cfgRegs *)params->userCfgBase, readRegs->epIrqSet, 0);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->epIrqClr != NULL))
    {
        status = Pcie_readEpIrqClrReg ((CSL_user_cfgRegs *)params->userCfgBase, readRegs->epIrqClr, 0);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->epIrqStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    for (i = 0; i < 4; i++)
    {
        if ((status == SystemP_SUCCESS) && (readRegs->genPurpose[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }
    for (i = 0; i < 8; i++)
    {
        if ((status == SystemP_SUCCESS) && (readRegs->msiIrqStatusRaw[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->msiIrqStatus[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->msiIrqEnableSet[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->msiIrqEnableClr[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }
    for (i = 0; i < 4; i++)
    {
        if ((status == SystemP_SUCCESS) && (readRegs->legacyIrqStatusRaw[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->legacyIrqStatus[i] != NULL))
        {
            status = Pcie_readLegacyIrqStatusReg ((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->legacyIrqStatus[i], i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->legacyIrqEnableSet[i] != NULL))
        {
          status = Pcie_readLegacyIrqEnableSetReg ((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->legacyIrqEnableSet[i], i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->legacyIrqEnableClr[i] != NULL))
        {
            status = Pcie_readLegacyIrqEnableClrReg ((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->legacyIrqEnableClr[i], i);
        }
    }
    if ((status == SystemP_SUCCESS) && (readRegs->errIrqStatusRaw != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->errIrqStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->errIrqEnableSet != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->errIrqEnableClr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    if ((status == SystemP_SUCCESS) && (readRegs->pmRstIrqStatusRaw != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->pmRstIrqStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->pmRstIrqEnableSet != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->pmRstIrqEnableClr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    if ((status == SystemP_SUCCESS) && (readRegs->ptmIrqStatusRaw != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->ptmIrqStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->ptmIrqEnableSet != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->ptmIrqEnableClr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    for (i = 0; i < 8; i ++)
    {
        if ((status == SystemP_SUCCESS) && (readRegs->obOffsetLo[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->obOffsetHi[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }

    for (i = 0; i < 4; i ++)
    {
        if ((status == SystemP_SUCCESS) && (readRegs->ibBar[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->ibStartLo[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->ibStartHi[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->ibOffset[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }

    if ((status == SystemP_SUCCESS) && (readRegs->pcsCfg0 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->pcsCfg1 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->pcsStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    if ((status == SystemP_SUCCESS) && (readRegs->serdesCfg0 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->serdesCfg1 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /*****************************************************************************************
    *Configuration Registers
    *****************************************************************************************/

    /*Type 0, Type1 Common Registers*/

    if ((status == SystemP_SUCCESS) && (readRegs->vndDevId != NULL))
    {
        status = Pcie_readVendorDevIdReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_VENDOR_ID_DEVICE_ID), readRegs->vndDevId);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->statusCmd != NULL))
    {
        status = Pcie_readStatusCmdReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_COMMAND_STATUS), readRegs->statusCmd);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->baseAddr != NULL))
    {
        status = Pcie_readBaseAddrReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_BASE_ADDR[0]), readRegs->baseAddr);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->revId != NULL))
    {
        status = Pcie_readRevIdReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_REVISION_ID_CLASS_CODE), readRegs->revId);
    }

    if ((status == SystemP_SUCCESS) && (readRegs->bist != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /*Type 0 Registers*/
    if ((status == SystemP_SUCCESS) && (readRegs->type0BarIdx != NULL))
    {
        status = Pcie_readType0BarReg (baseCfgEp, &(readRegs->type0BarIdx->reg),
                                                          readRegs->type0BarIdx->idx);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type0Bar32bitIdx != NULL))
    {
        status = Pcie_readType0Bar32bitReg (baseCfgEp, &(readRegs->type0Bar32bitIdx->reg),
                                                             readRegs->type0Bar32bitIdx->idx,
                                                             params->pfNum);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type0BarMask32bitIdx != NULL))
    {
        status = Pcie_readType0Bar32bitReg (baseCfgEp, &(readRegs->type0BarMask32bitIdx->reg),
                                                            readRegs->type0BarMask32bitIdx->idx,
                                                            params->pfNum);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->subId != NULL))
    {
        status = Pcie_readSubIdReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_I), readRegs->subId);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->cardbusCisPointer != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->expRom != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->capPtr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->intPin != NULL))
    {
        status = Pcie_readIntPinReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_INTRPT_LINE_INTRPT_PIN), readRegs->intPin);
    }

    /* Type 1 Registers*/
    if ((status == SystemP_SUCCESS) && (readRegs->type1BistHeader != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1BarIdx != NULL))
    {
        status = Pcie_readType1BarReg (baseCfgEp, &(readRegs->type1BarIdx->reg),
                                                          readRegs->type1BarIdx->idx);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1Bar32bitIdx != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1BarMask32bitIdx != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1BusNum != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1SecStat != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1Memspace != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->prefMem != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->prefBaseUpper != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->prefLimitUpper != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1IOSpace != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1CapPtr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1ExpnsnRom != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->type1BridgeInt != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* Power Management Capabilities Registers */
    if ((status == SystemP_SUCCESS) && (readRegs->pmCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->pmCapCtlStat != NULL))
    {
        status = Pcie_readPmMgmtCtrlStatReg((CSL_pcie_ep_coreRegs *)&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_POWER_MGMT_CAP_STRUCT.I_PWR_MGMT_CTRL_STAT_REP,
                                            readRegs->pmCapCtlStat);
    }

    /* MSI Registers */
    if ((status == SystemP_SUCCESS) && (readRegs->msiCap != NULL))
    {
        status = Pcie_readMsiCapReg((CSL_pcie_ep_coreRegs *)&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_MSI_CAP_STRUCT.I_MSI_CTRL_REG, readRegs->msiCap);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->msiLo32 != NULL))
    {
        status = Pcie_readMsiLo32Reg(baseCfgEp, readRegs->msiLo32);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->msiUp32 != NULL))
    {
        status = Pcie_readMsiUp32Reg(baseCfgEp, readRegs->msiUp32);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->msiData != NULL))
    {
        status = Pcie_readMsiDataReg (baseCfgEp, readRegs->msiData);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->msiCapOff10H != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->msiCapOff14H != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* MSIx Registers */
    if ((status == SystemP_SUCCESS) && (readRegs->msixCap != NULL))
    {
        status = Pcie_readMsixCapReg ((CSL_pcie_ep_coreRegs *)&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_MSIX_CAP_STRUCT.I_MSIX_CTRL,
                                        readRegs->msixCap);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->msixTblOffset != NULL))
    {

        status = Pcie_readMsixTblOffset ((CSL_pcie_ep_coreRegs *)&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_MSIX_CAP_STRUCT.I_MSIX_TBL_OFFSET,
                                         readRegs->msixTblOffset);
    }

    /*Capabilities Registers*/
    if ((status == SystemP_SUCCESS) && (readRegs->pciesCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->deviceCap != NULL))
    {
        status = Pcie_readDevCapReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_CAP_STRUCT.I_PCIE_DEV_CAP), readRegs->deviceCap);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->devStatCtrl != NULL))
    {
        status = Pcie_readDevStatCtrlReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_CAP_STRUCT.I_PCIE_DEV_CTRL_STATUS), readRegs->devStatCtrl);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->linkCap != NULL))
    {
        status = Pcie_readLinkCapReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_CAP_STRUCT.I_LINK_CAP), readRegs->linkCap);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->linkStatCtrl != NULL))
    {
        status = Pcie_readLinkStatCtrlReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_CAP_STRUCT.I_LINK_CTRL_STATUS), readRegs->linkStatCtrl);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->slotCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->slotStatCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->rootCtrlCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->rootStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->devCap2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->devStatCtrl2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->linkCap2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->linkCtrl2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /*Capabilities Extended Registers*/
    if ((status == SystemP_SUCCESS) && (readRegs->extCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->uncErr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->uncErrMask != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->uncErrSvrty != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->corErr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->corErrMask != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->accr != NULL))
    {
        status = Pcie_readAccrReg (baseCfgEp, readRegs->accr);
    }
    for (i = 0; i < 4; i ++)
    {
        if ((status == SystemP_SUCCESS) && (readRegs->hdrLog[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }
    if ((status == SystemP_SUCCESS) && (readRegs->rootErrCmd != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->rootErrSt != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->errSrcID != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* Port Logic Registers */
    if ((status == SystemP_SUCCESS) && (readRegs->plAckTimer != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plOMsg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plForceLink != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->ackFreq != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->lnkCtrl != NULL))
    {
        status = Pcie_readLnkCtrlReg ((CSL_pcie_ep_coreRegs *)params->pcieCtrlAddr, readRegs->lnkCtrl);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->laneSkew != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->symNum != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->symTimerFltMask != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->fltMask2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->debug0 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->debug1 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->gen2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    if ((status == SystemP_SUCCESS) && (readRegs->plconfObnpSubreqCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfTrPStsR != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfTrNpStsR != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfTrCStsR != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfQStsR != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfVcTrAR1 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfVcTrAR2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfVc0PrQC != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfVc0NprQC != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfVc0CrQC != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    for (i = 0; i < 3; i++)
    {
        if ((status == SystemP_SUCCESS) && (readRegs->plconfVcPrQC[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->plconfVcNprQC[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->plconfVcCrQC[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfPhyStsR != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfPhyCtrlR != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlAddress != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlUpperAddress != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    for (i = 0; i < 8; i++)
    {
        if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlIntEnable[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlIntMask[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlIntStatus[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlGpio != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfPipeLoopback != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfDbiRoWrEn != NULL)) {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfAxiSlvErrResp != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfAxiSlvTimeout != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuIndex != NULL))
    {
        /* Return the simulated window address */
        *readRegs->plconfIatuIndex = *simIatuWindow;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegCtrl1 != NULL))
    {
        status = Pcie_readPlconfIatuRegCtrl1Reg (baseCfgEp, simIatuWindow, readRegs->plconfIatuRegCtrl1);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegLowerBase != NULL))
    {
        status = Pcie_readPlconfIatuRegLowerBaseReg (baseCfgEp, simIatuWindow, readRegs->plconfIatuRegLowerBase);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegUpperBase != NULL))
    {
        status = Pcie_readPlconfIatuRegUpperBaseReg (baseCfgEp, simIatuWindow, readRegs->plconfIatuRegUpperBase);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegLimit != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegLowerTarget != NULL))
    {
        status = Pcie_readPlconfIatuRegLowerTargetReg (baseCfgEp, simIatuWindow, readRegs->plconfIatuRegLowerTarget);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegUpperTarget != NULL))
    {
        status = Pcie_readPlconfIatuRegUpperTargetReg (baseCfgEp, simIatuWindow, readRegs->plconfIatuRegUpperTarget);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegCtrl3 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* TI CONF registers */
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfRevision != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfSysConfig != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEoi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqStatusRawMain != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqStatusMain != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEnableSetMain != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEnableClrMain != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqStatusRawMsi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqStatusMsi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEnableSetMsi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEnableClrMsi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfDeviceType != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfDeviceCmd != NULL))
    {
        status = Pcie_readLinkStatusReg ((CSL_pcie_ep_coreRegs *)params->userCfgBase, readRegs->tiConfDeviceCmd);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfPmCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfPhyCs != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIntxAssert != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfIntxDeassert != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfMsiXmt != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfDebugCfg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfDebugData != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (readRegs->tiConfDiagCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* User Config Registers */
    if ((status == SystemP_SUCCESS) && (readRegs->usrCfgInitCfg != NULL))
    {
        status = Pcie_readUsrInitCfgReg((CSL_user_cfgRegs *)params->userCfgBase, readRegs->usrCfgInitCfg);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->usrCfgPmCmd != NULL))
    {
        status = Pcie_readUsrPmCmdReg((CSL_user_cfgRegs *)params->userCfgBase, readRegs->usrCfgPmCmd);
    }

    /* Intd Config Registers */
    if ((status == SystemP_SUCCESS) && (readRegs->intCfgEnSys0Reg != NULL))
    {
        status = Pcie_readIntdCfgEnSys0Reg((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->intCfgEnSys0Reg);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->intCfgEnSys1Reg != NULL))
    {
        status = Pcie_readIntdCfgEnSys1Reg((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->intCfgEnSys1Reg);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->intCfgEnSys2Reg != NULL))
    {
        status = Pcie_readIntdCfgEnSys2Reg((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->intCfgEnSys2Reg);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->intCfgStatusSys0Reg != NULL))
    {
        status = Pcie_readIntdCfgStatusSys0Reg((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->intCfgStatusSys0Reg);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->intCfgStatusSys1Reg != NULL))
    {
        status = Pcie_readIntdCfgStatusSys1Reg((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->intCfgStatusSys1Reg);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->intCfgStatusSys2Reg != NULL))
    {
        status = Pcie_readIntdCfgStatusSys2Reg((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->intCfgStatusSys2Reg);
    }
    if ((status == SystemP_SUCCESS) && (readRegs->intCfgStatusClrSys2Reg != NULL))
    {
        status = Pcie_readIntdCfgStatusClrSys2Reg((CSL_intd_cfgRegs *)params->intCfgBase, readRegs->intCfgStatusClrSys2Reg);
    }

    /* Read Vendor Specific Control register */
    if ((status == SystemP_SUCCESS) && (readRegs->vndrSpecCntrl != NULL))
    {
        status = Pcie_readVndrSpecCntrlReg(&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_REGF_VSEC_STRUCT.I_VENDOR_SPECIFIC_CONTROL_REG, readRegs->vndrSpecCntrl);
    }

    /* Configuration registers accessed via LM */
    if ((status == SystemP_SUCCESS) && (readRegs->lmVendorIdReg != NULL))
    {
        status =  Pcie_readLmVendorIdReg(&baseCfgEp->LM_I_REGF_LM_PCIE_BASE.I_VENDOR_ID_REG, readRegs->lmVendorIdReg);
    }

    return status;
}

/*****************************************************************************
 * Combine and write the Vendor ID Device ID register
 ****************************************************************************/
static int32_t Pcie_writeVendorDevIdReg(volatile uint32_t *hwReg_VENDOR_ID_DEV_ID, Pcie_VndDevIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_VENDOR_ID_DEV_ID != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_VID, swReg->vndId);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_DID, swReg->devId);

        *hwReg_VENDOR_ID_DEV_ID = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/*****************************************************************************
 * Combine and write the Subsystem Vendor ID register
 ****************************************************************************/
static int32_t Pcie_writeSubIdReg(volatile uint32_t *hwReg_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_ID, Pcie_SubIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_ID != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_I_SID, swReg->subId);

        *hwReg_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_ID = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/*****************************************************************************
 * Combine and write the Interrupt Line & Pin register
 ****************************************************************************/
static int32_t Pcie_writeIntPinReg(volatile uint32_t *hwReg_INTRPT_LINE_INTRPT_PIN, Pcie_IntPinReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_INTRPT_LINE_INTRPT_PIN != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_ILR, swReg->intLine);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_IPR, swReg->intPin);

        *hwReg_INTRPT_LINE_INTRPT_PIN = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/*****************************************************************************
 * Combine and write the Command Status swRegister
 ****************************************************************************/
static int32_t Pcie_writeCmdStatusReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_CmdStatusReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    CSL_user_cfgRegs *userCfg;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        userCfg = (CSL_user_cfgRegs *)baseAddr;
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_USER_CFG_CMD_STATUS_LINK_TRAINING_ENABLE, swReg->ltssmEn);

        userCfg->CMD_STATUS = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeCmdStatusReg */

/*****************************************************************************
 * Combine and write the Endpoint Interrupt Request Set swRegister
 ****************************************************************************/
static int32_t Pcie_writeEpIrqSetReg (CSL_user_cfgRegs *baseAddr, Pcie_EpIrqSetReg *swReg,
                    int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        switch (swRegNum)
        {
            case 0:
                PCIE_SETBITS(new_val, CSL_USER_CFG_LEGACY_INTR_SET_INTA_IN, swReg->epIrqSet);
                break;
            case 1:
                PCIE_SETBITS(new_val, CSL_USER_CFG_LEGACY_INTR_SET_INTB_IN, swReg->epIrqSet);
                break;
            case 2:
                PCIE_SETBITS(new_val, CSL_USER_CFG_LEGACY_INTR_SET_INTC_IN, swReg->epIrqSet);
                break;
            case 3:
                PCIE_SETBITS(new_val, CSL_USER_CFG_LEGACY_INTR_SET_INTD_IN, swReg->epIrqSet);
                break;
        }

        baseAddr->LEGACY_INTR_SET = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeEpIrqSetReg */

/*****************************************************************************
 * Combine and write the Endpoint Interrupt Request Clear swRegister
 ****************************************************************************/
static int32_t Pcie_writeEpIrqClrReg (CSL_user_cfgRegs *baseAddr, Pcie_EpIrqClrReg *swReg,
                                    int_fast32_t swRegNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;
        switch (swRegNum)
        {
            case 0:
            PCIE_SETBITS(new_val, CSL_USER_CFG_LEGACY_INTR_SET_INTA_IN, swReg->epIrqClr);
            break;
            case 1:
                PCIE_SETBITS(new_val, CSL_USER_CFG_LEGACY_INTR_SET_INTB_IN, swReg->epIrqClr);
                break;
            case 2:
                PCIE_SETBITS(new_val, CSL_USER_CFG_LEGACY_INTR_SET_INTC_IN, swReg->epIrqClr);
                break;
            case 3:
                PCIE_SETBITS(new_val, CSL_USER_CFG_LEGACY_INTR_SET_INTD_IN, swReg->epIrqClr);
                break;
        }

        baseAddr->LEGACY_INTR_SET = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeEpIrqClrReg */

/*****************************************************************************
 * Write Root Complex End of Interrupt Vector swRegister
 ****************************************************************************/
static int32_t Pcie_writeRcIrqEoi (CSL_user_cfgRegs *baseAddr, Pcie_IrqEOIReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_USER_CFG_EOI_REG_EOI_VECTOR, swReg->EOI);

        baseAddr->EOI_VECTOR = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/*****************************************************************************
 * Combine and write the Status and Command register
 ****************************************************************************/
static int32_t Pcie_writeStatusCmdReg (volatile uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                                Pcie_StatusCmdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_STATUS_COMMAND_REGISTER != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_ISE, swReg->ioSp);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MSE, swReg->memSp);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_BE, swReg->busMs);

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_PERE, swReg->resp);

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SE, swReg->serrEn);

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IMD, swReg->dis);

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IS, swReg->stat);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_CL, swReg->capList);

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MDPE, swReg->parError);

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_STA, swReg->sigTgtAbort);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RTA, swReg->tgtAbort);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RMA, swReg->mstAbort);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SSE, swReg->sysError);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_DPE, swReg->parity);

        *hwReg_STATUS_COMMAND_REGISTER = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeStatusCmdReg */

/*****************************************************************************
 * Combine and write the Status and Command register
 ****************************************************************************/
static int32_t Pcie_writeBaseAddrReg (volatile uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                                    Pcie_BaseAddrReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_STATUS_COMMAND_REGISTER != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_MSI0, swReg->msio);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_R7, swReg->r7);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_S0, swReg->s0);

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_P0, swReg->p0);

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_R8, swReg->r8);

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_BAMR0, swReg->bamr0);

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_BASE_ADDR_0_BAMRW, swReg->bamrw);

        *hwReg_STATUS_COMMAND_REGISTER = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeBaseAddrReg */

/*****************************************************************************
 * Combine and write the Class Code and Revision ID register
 ****************************************************************************/
static int32_t Pcie_writeRevIdReg (volatile uint32_t *hwReg_CLASSCODE_REVISIONID,
                                Pcie_RevIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_CLASSCODE_REVISIONID != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_CC,  swReg->classCode);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_RID, swReg->revId);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_PIB, swReg->progIntrface);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_BASE_I_REVISION_ID_CLASS_CODE_SCC, swReg->subClassCode);

        *hwReg_CLASSCODE_REVISIONID = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;

    return SystemP_SUCCESS;
} /* Pcie_writeRevIdReg */

/*****************************************************************************
 * Combine and write the type0 BAR register
 ****************************************************************************/
static int32_t Pcie_writeType0BarReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                                int32_t barNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        /* set the BARxA and BARxC accroding to the barNum */
        switch (barNum%4)
        {
            case 0:
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0C, swReg->barxc);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0A, swReg->barxa);
                break;
            case 1:
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1C, swReg->barxc);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1A, swReg->barxa);
                break;
            case 2:
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2C, swReg->barxc);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2A, swReg->barxa);
                break;
            case 3:
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3C, swReg->barxc);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3A, swReg->barxa);
                break;
            default:
                break;
        }

        baseAddr->LM_I_REGF_LM_PCIE_BASE.I_PF_BAR_CONFIG_REG[0].I_PF_BAR_CONFIG_REG[barNum/4] = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeType0BarReg */

/*****************************************************************************
 * Combine and write the type1 BAR register
 ****************************************************************************/
static int32_t Pcie_writeType1BarReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                                int32_t barNum)
{
    int32_t status = SystemP_SUCCESS;
    CSL_pcie_rp_coreRegs *baseAddrRc = NULL;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;
        baseAddrRc = (CSL_pcie_rp_coreRegs *)baseAddr;

        /* set the BARxA and BARxC accroding to the barNum */
        switch (barNum)
        {
            case 0:
                PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0C, swReg->barxc);
                PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0A, swReg->barxa);
                break;
            case 1:
                PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1C, swReg->barxc);
                PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1A, swReg->barxa);
                break;
            default:
                break;
        }

        if((swReg->barxc == PCIE_BARC_32B_MEM_BAR_PREFETCH)
                    || (swReg->barxc == PCIE_BARC_64B_MEM_BAR_PREFETCH))
        {
            PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPME, 0x1U);
        }

        if((swReg->barxc == PCIE_BARC_64B_MEM_BAR_NON_PREFETCH)
                    || (swReg->barxc == PCIE_BARC_64B_MEM_BAR_PREFETCH))
        {
            PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPMS, 0x1U);
        }

        baseAddrRc->LM_I_REGF_LM_PCIE_BASE.I_RC_BAR_CONFIG_REG = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeType1BarReg */

/*****************************************************************************
 * Combine and write the BAR 32bits register
 ****************************************************************************/
static int32_t Pcie_writeType0Bar32bitReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_Bar32bitReg *swReg,
                        int32_t barNum, uint8_t pfNum)
{
    int32_t status = SystemP_SUCCESS;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        baseAddr->EP_PF_I_PCIE[pfNum].EP_PF_I_PCIE_BASE.I_BASE_ADDR[barNum] = swReg->raw = swReg->reg32;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeType0Bar32bitReg */

/*****************************************************************************
 * Combine and write the Device Capability register
 ****************************************************************************/
static int32_t Pcie_writeDevCapReg (volatile uint32_t *hwReg_DEV_CAS, Pcie_DeviceCapReg *swReg)
{

    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_DEV_CAS != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_FLRC, swReg->flrEn);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CPLS, swReg->pwrLimitScale);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CSP,  swReg->pwrLimitValue);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_RER,  swReg->errRpt);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL1L, swReg->l1Latency);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL0L, swReg->l0Latency);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_ETFS, swReg->extTagFld);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_PFS,  swReg->phantomFld);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_MP,   swReg->maxPayldSz);

        *hwReg_DEV_CAS = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeDevCapReg */

/*****************************************************************************
 * Combine and write the Device Status and Control register
 ****************************************************************************/
static int32_t Pcie_writeDevStatCtrlReg (volatile uint32_t *hwReg_DEV_CAS, Pcie_DevStatCtrlReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_DEV_CAS != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ECER, swReg->corErRp);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENFER, swReg->nFatalErRp);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EFER, swReg->fatalErRp);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EURR, swReg->reqRp);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ERO, swReg->relaxed);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MP, swReg->maxPayld);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE, swReg->xtagEn);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE, swReg->phantomEn);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE, swReg->auxPwrEn);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENS, swReg->noSnoop);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MRR, swReg->maxSz);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_CED, swReg->corrEr);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_NFED, swReg->nFatalEr);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_FED, swReg->fatalEr);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_URD, swReg->rqDet);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_APD, swReg->auxPwr);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_TP, swReg->tpend);

        *hwReg_DEV_CAS = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeDevStatCtrlReg */

/*****************************************************************************
 * Combine and write the Link Capabilities register
 ****************************************************************************/
static int32_t Pcie_writeLinkCapReg(volatile uint32_t *hwReg_LNK_CAP, Pcie_LinkCapReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_LNK_CAP != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_MAIN_CTRL_MMR_CFG0_PCIE0_CTRL_GENERATION_SEL, swReg->maxLinkSpeed);

        *hwReg_LNK_CAP = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeLinkCapReg */

/*****************************************************************************
 * Combine and write the Link Status and Control register
 ****************************************************************************/
static int32_t Pcie_writeLinkStatCtrlReg(volatile uint32_t *hwReg_LNK_CAS, Pcie_LinkStatCtrlReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_LNK_CAS != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_PCIE_CAP_STRUCT_I_LINK_CTRL_STATUS_SCC, swReg->slotClkCfg);

        *hwReg_LNK_CAS = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeLinkStatCtrlReg */

/*****************************************************************************
 * Combine and write the Advanced Error Capabilities and Control register
 ****************************************************************************/
static int32_t Pcie_writeAccrReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_AccrReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    CSL_pcie_rp_coreRegs *baseAddrRp;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        baseAddrRp = (CSL_pcie_rp_coreRegs *)baseAddr;
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRE, swReg->multHdrEn);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRC, swReg->multHdrCap);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEC, swReg->chkEn);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_ECC, swReg->chkCap);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEG, swReg->genEn);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EGC, swReg->genCap);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_FEP, swReg->erPtr);

        baseAddrRp->RC_I_RC_PCIE_BASE.I_ADV_ERR_CAP_CTL = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeAccrReg */

/*****************************************************************************
 * Combine and write the PL CONF iATU Region Lower Base Address register
 ****************************************************************************/
static int32_t Pcie_writePlconfIatuRegLowerBaseReg (CSL_pcie_ep_coreRegs *baseAddr,
            const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegLowerBaseReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;
    uint8_t regionIndex;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        new_val = swReg->raw;
        regionIndex = simIatuWindow->regionIndex;

        if (((simIatuWindow->regionDirection == 0) && (regionIndex < OB_REGION_MAX))
            || ((simIatuWindow->regionDirection == 1) && (regionIndex < IB_REGION_MAX_EP)))
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR0_DATA, swReg->iatuRegLowerBase);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR0_NUM_BITS, swReg->num_bits);

                swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].ADDR0 = new_val;
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR0_DATA, swReg->iatuRegLowerBase);

                swReg->raw = baseAddr->ATU_FUNC_WRAPPER_IB_EP[0][regionIndex].ADDR0 = new_val << 8;

                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writePlconfIatuRegLowerBaseReg */

/*****************************************************************************
 * Combine and write the PL CONF iATU Region Lower Base Address register
 ****************************************************************************/
static int32_t Pcie_writePlconfIatuRegLowerBaseRcReg (CSL_pcie_ep_coreRegs *baseAddr,
                        const Pcie_PlconfIatuIndexReg *simIatuWindow,
                                Pcie_PlconfIatuRegLowerBaseReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;
    uint8_t regionIndex;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        new_val = swReg->raw;
        regionIndex = simIatuWindow->regionIndex;

        if (((simIatuWindow->regionDirection == 0) && (regionIndex < OB_REGION_MAX))
            || ((simIatuWindow->regionDirection == 1) && (regionIndex < IB_REGION_MAX_RC)))
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR0_DATA, swReg->iatuRegLowerBase);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR0_NUM_BITS, swReg->num_bits);

                swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].ADDR0 = new_val;
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR0_DATA, swReg->iatuRegLowerBase);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR0_NUM_BITS, swReg->num_bits);

                swReg->raw = baseAddr->ATU_WRAPPER_IB[regionIndex].ADDR0 = new_val;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writePlconfIatuRegLowerBaseRcReg */

/*****************************************************************************
 * Combine and write the PL CONF iATU Region Limit Address register
 ****************************************************************************/
static int32_t Pcie_writePlconfIatuRegUpperBaseReg (CSL_pcie_ep_coreRegs *baseAddr,
                const Pcie_PlconfIatuIndexReg *simIatuWindow,
                Pcie_PlconfIatuRegUpperBaseReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;
    uint8_t regionIndex;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        new_val = swReg->raw;
        regionIndex = simIatuWindow->regionIndex;

        if (((simIatuWindow->regionDirection == 0) && (regionIndex < OB_REGION_MAX))
            || ((simIatuWindow->regionDirection == 1) && (regionIndex < IB_REGION_MAX_EP)))
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_ADDR1_DATA, swReg->iatuRegUpperBase);

                swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].ADDR1 = new_val;
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR1_DATA, swReg->iatuRegUpperBase);

                swReg->raw = baseAddr->ATU_FUNC_WRAPPER_IB_EP[0][regionIndex].ADDR1 = new_val;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writePlconfIatuRegUpperBaseReg */

/*****************************************************************************
 * Combine and write the PL CONF iATU Region Lower Target Address register
 ****************************************************************************/
static int32_t Pcie_writePlconfIatuRegLowerTargetReg (CSL_pcie_ep_coreRegs *baseAddr,
            const Pcie_PlconfIatuIndexReg *simIatuWindow,
            Pcie_PlconfIatuRegLowerTargetReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;
    uint8_t regionIndex;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        new_val = swReg->raw;
        regionIndex = simIatuWindow->regionIndex;

        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_DATA, swReg->iatuRegLowerTarget);
                PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_REGION_SIZE, swReg->num_bits);
                swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].AXI_ADDR0 = new_val;
            }
            else
            {
                /* INBOUND */
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writePlconfIatuRegLowerTargetReg */

/*****************************************************************************
 * Combine and write the PL CONF iATU Region Upper Target Address register
 ****************************************************************************/
static int32_t Pcie_writePlconfIatuRegUpperTargetReg (CSL_pcie_ep_coreRegs *baseAddr,
                const Pcie_PlconfIatuIndexReg *simIatuWindow,
                Pcie_PlconfIatuRegUpperTargetReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;
    uint8_t regionIndex;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        new_val = swReg->raw;
        regionIndex = simIatuWindow->regionIndex;

        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR1_DATA, swReg->iatuRegUpperTarget);
                swReg->raw = baseAddr->ATU_WRAPPER_OB[regionIndex].AXI_ADDR1 = new_val;
            }
            else
            {
                /* INBOUND */
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writePlconfIatuRegUpperTargetReg */

/*****************************************************************************
 * Combine and write the PL CONF iATU Region Control 1 register
 ****************************************************************************/
static int32_t Pcie_writePlconfIatuRegCtrl1Reg (CSL_pcie_ep_coreRegs *baseAddr,
                        const Pcie_PlconfIatuIndexReg *simIatuWindow,
                        Pcie_PlconfIatuRegCtrl1Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;
    uint8_t regionIndex;

    if ((baseAddr != NULL) && (swReg != NULL) && (simIatuWindow != NULL))
    {
        new_val = swReg->raw;
        regionIndex = simIatuWindow->regionIndex;

        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                /* TLP type bit[3:0] - 0xA - Config, 0x2 - MEmory WR/RD, 0xC - message, 0xD - Vendor message, 0x6 */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_DESC0_TT, swReg->type);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_DESC0_PTC, swReg->tc);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_DESC0_PTC, swReg->attr);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ATU_WRAPPER_OB_0_DESC0_FNUM, swReg->functionNumber);
                baseAddr->ATU_WRAPPER_OB[regionIndex].DESC0 = new_val;
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writePlconfIatuRegCtrl1Reg */

/*****************************************************************************
 * Combine and write the LINKSTATUS swRegister
 ****************************************************************************/
static int32_t Pcie_writeLinkStatusReg (CSL_pcie_ep_coreRegs *baseAddr,
                        Pcie_TiConfDeviceCmdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    CSL_user_cfgRegs *userCfg;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        userCfg = (CSL_user_cfgRegs *)baseAddr;
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_USER_CFG_LINKSTATUS_LTSSM_STATE, swReg->ltssmState);

        userCfg->LINKSTATUS = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeLinkStatusReg */

/* Write MSI capabilities Register */
/*****************************************************************************
 * Combine and write the Message Signaled Interrupt Capability register
 ****************************************************************************/
static int32_t Pcie_writeMsiCapReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiCapReg *swReg)
{
    uint32_t new_val = swReg->raw;
    uint32_t *regVal = (uint32_t *)baseAddr;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_CID1, swReg->capId);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_CP1, swReg->nextCap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_ME, swReg->msiEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_MMC, swReg->multMsgCap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_MME, swReg->multMsgEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_BAC64, swReg->en64bit);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_MSI_CAP_STRUCT_I_MSI_CTRL_REG_MC, swReg->pvmEn);

    *regVal = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_writeMsiCapReg */

/*****************************************************************************
 * Combine and write the Address of MSI write TLP req lower 32 bits register
 ****************************************************************************/
static int32_t Pcie_writeMsiLo32Reg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiLo32Reg *swReg)
{
    CSL_pcie_rp_coreRegs *baseAddrRc = (CSL_pcie_rp_coreRegs *)baseAddr;
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR_MAL, swReg->addr);

    baseAddrRc->RC_I_RC_PCIE_BASE.I_MSI_MSG_LOW_ADDR = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_writeMsiLo32Reg */

/*****************************************************************************
 * Combine and write the Address of MSI write TLP req upper 32 bits register
 ****************************************************************************/
static int32_t Pcie_writeMsiUp32Reg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiUp32Reg *swReg)
{
    CSL_pcie_rp_coreRegs *baseAddrRc = (CSL_pcie_rp_coreRegs *)baseAddr;
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_HI_ADDR_MAH, swReg->addr);

    baseAddrRc->RC_I_RC_PCIE_BASE.I_MSI_MSG_HI_ADDR = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_writeMsiUp32Reg */

/*****************************************************************************
 * Combine and write the Data of MSI write TLP req register
 ****************************************************************************/
static int32_t Pcie_writeMsiDataReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiDataReg *swReg)
{
    CSL_pcie_rp_coreRegs *baseAddrRc = (CSL_pcie_rp_coreRegs *)baseAddr;
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA_MD, swReg->data);

    baseAddrRc->RC_I_RC_PCIE_BASE.I_MSI_MSG_DATA = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_writeMsiDataReg */

/* Write MSIx capabilities Register */
static int32_t Pcie_writeMsixCapReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsixCapReg *swReg)
{
    uint32_t new_val = swReg->raw;
    uint32_t *regVal = (uint32_t *)baseAddr;

    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXE, swReg->msixEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_FM, swReg->maskIrq);
    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXTS, swReg->msixTblSize);
    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CID, swReg->capId);

    *regVal = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write data of the MSI table offset register
 ****************************************************************************/
static int32_t Pcie_writeMsixTblOffset (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsixTblOffset *swReg)
{
    uint32_t new_val = swReg->raw;
    uint32_t *regVal = (uint32_t *)baseAddr;

    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_TO, swReg->offset);
    PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_BARI, swReg->barIndex);

    *regVal = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the data of the power management control status register
 ****************************************************************************/
static int32_t Pcie_writePmMgmtCtrlStatReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PMCapCtlStatReg *swReg)
{
    uint32_t new_val = swReg->raw;
    uint32_t *regVal = (uint32_t *)baseAddr;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_PMES, swReg->pmeStatus);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_PE,   swReg->pmeEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_NSR,  swReg->noSoftRst);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_POWER_MGMT_CAP_STRUCT_I_PWR_MGMT_CTRL_STAT_REP_PS,   swReg->pwrState);

    *regVal = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the data of the User Config Init Cfg register
 ****************************************************************************/
static int32_t Pcie_writeUsrInitCfgReg(CSL_user_cfgRegs *baseAddr, Pcie_UserCfgInitCfgReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_USER_CFG_INITCFG_CONFIG_ENABLE, swReg->cfgEn);
        PCIE_SETBITS(new_val, CSL_USER_CFG_INITCFG_VC_COUNT, swReg->vcCnt);
        PCIE_SETBITS(new_val, CSL_USER_CFG_INITCFG_MAX_EVAL_ITERATION, swReg->maxEvalItr);
        PCIE_SETBITS(new_val, CSL_USER_CFG_INITCFG_BYPASS_PHASE23, swReg->bypassPh23);
        PCIE_SETBITS(new_val, CSL_USER_CFG_INITCFG_BYPASS_REMOTE_TX_EQUALIZATION, swReg->bypassRmTxEQ);
        PCIE_SETBITS(new_val, CSL_USER_CFG_INITCFG_SUPPORTED_PRESET, swReg->suppPrst);
        PCIE_SETBITS(new_val, CSL_USER_CFG_INITCFG_DISABLE_GEN3_DC_BALANCE, swReg->disableGen3DcBlnc);
        PCIE_SETBITS(new_val, CSL_USER_CFG_INITCFG_SRIS_ENABLE, swReg->srisEn);

        baseAddr->INITCFG = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeUsrInitCfgReg */

/*****************************************************************************
 * Combine and write the Data of the User Config Pm Cmd register
 ****************************************************************************/
static int32_t Pcie_writeUsrPmCmdReg(CSL_user_cfgRegs *baseAddr, Pcie_UserCfgPmCmdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_USER_CFG_PMCMD_PWR_STATE_CHANGE_ACK, swReg->pwrStateChgAck);
        PCIE_SETBITS(new_val, CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1_SUBSTATE, swReg->clientReqExtL1Substate);
        PCIE_SETBITS(new_val, CSL_USER_CFG_PMCMD_CLIENT_REQ_EXIT_L1, swReg->clientReqExtL1);

        baseAddr->PMCMD = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeUsrPmCmdReg */

/*****************************************************************************
 * Combine and write the Data of the Intd Config Enable Sys0 register
 ****************************************************************************/
static int32_t Pcie_writeIntdCfgEnSys0Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys0Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_INTD_CFG_ENABLE_REG_SYS_0_ENABLE_SYS_EN_PCIE_DOWNSTREAM, swReg->dwnStrEn);

        baseAddr->ENABLE_REG_SYS_0 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeIntdCfgEnSys0Reg */

/*****************************************************************************
 * Combine and write the Data of the Intd Config Enable Sys1 register
 ****************************************************************************/
static int32_t Pcie_writeIntdCfgEnSys1Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys1Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_PWR_STATE, swReg->pwrStateEn);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_LEGACY_3,  swReg->legacy3En);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_LEGACY_2,  swReg->legacy2En);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_LEGACY_1,  swReg->legacy1En);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_LEGACY_0,  swReg->legacy0En);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_ENABLE_REG_SYS_1_ENABLE_SYS_EN_PCIE_FLR,       swReg->flrEn);

        baseAddr->ENABLE_REG_SYS_1 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeIntdCfgEnSys1Reg */

/*****************************************************************************
 * Combine and write the Data of the Intd Config Enable Sys2 register
 ****************************************************************************/
static int32_t Pcie_writeIntdCfgEnSys2Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgEnSys2Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_DPA,        swReg->dpaEn);
        PCIE_SETBITS(new_val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_ERROR_0,    swReg->err0En);
        PCIE_SETBITS(new_val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_ERROR_1,    swReg->err1En);
        PCIE_SETBITS(new_val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_ERROR_2,    swReg->err2En);
        PCIE_SETBITS(new_val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_HOT_RESET,  swReg->hotRstEn);
        PCIE_SETBITS(new_val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_Link_STATE, swReg->lnkStateEn);
        PCIE_SETBITS(new_val,CSL_INTD_CFG_ENABLE_REG_SYS_2_ENABLE_SYS_EN_PCIE_PTM,        swReg->ptmEn);

        baseAddr->ENABLE_REG_SYS_2 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeIntdCfgEnSys2Reg */

/*****************************************************************************
 * Combine and write the Data of the Intd Config Status Sys0 register
 ****************************************************************************/
static int32_t Pcie_writeIntdCfgStatusSys0Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys0Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_0_STATUS_SYS_PCIE_DOWNSTREAM, swReg->statusDwnStr);

        baseAddr->STATUS_REG_SYS_0 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeIntdCfgStatusSys0Reg */

/*****************************************************************************
 * Combine and write the Data of the Intd Config Status Sys1 register
 ****************************************************************************/
static int32_t Pcie_writeIntdCfgStatusSys1Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys1Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_PWR_STATE, swReg->statusPwrState);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_LEGACY_3,  swReg->statusLegacy3);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_LEGACY_2,  swReg->statusLegacy2);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_LEGACY_1,  swReg->statusLegacy1);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_LEGACY_0,  swReg->statusLegacy0);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_1_STATUS_SYS_PCIE_FLR,       swReg->statusFlr);

        baseAddr->STATUS_REG_SYS_1 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeIntdCfgStatusSys1Reg */

/*****************************************************************************
 * Combine and write the Data of the Intd Config Status Sys2 register
 ****************************************************************************/
static int32_t Pcie_writeIntdCfgStatusSys2Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusSys2Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_PTM,        swReg->statusPtm);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_LINK_STATE, swReg->statusLnkState);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_HOT_RESET,  swReg->statusHotRst);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_ERROR_2,    swReg->statusErr2);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_ERROR_1,    swReg->statusErr1);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_ERROR_0,    swReg->statusErr0);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_REG_SYS_2_STATUS_SYS_PCIE_DPA,        swReg->statusDpa);

        baseAddr->STATUS_REG_SYS_2 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeIntdCfgStatusSys2Reg */

/*****************************************************************************
 * Combine and write the Data of the Intd Config Status Clear Sys2 register
 ****************************************************************************/
static int32_t Pcie_writeIntdCfgStatusClrSys2Reg(CSL_intd_cfgRegs *baseAddr, Pcie_IntdCfgStatusClrSys2Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_PTM,        swReg->clrStatusPtm);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_LINK_STATE, swReg->clrStatusLnkState);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_HOT_RESET,  swReg->clrStatusHotRst);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_ERROR_2,    swReg->clrStatusErr2);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_ERROR_1,    swReg->clrStatusErr1);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_ERROR_0,    swReg->clrStatusErr0);
        PCIE_SETBITS(new_val, CSL_INTD_CFG_STATUS_CLR_REG_SYS_2_STATUS_SYS_PCIE_DPA,        swReg->clrStatusDpa);

        baseAddr->STATUS_CLR_REG_SYS_2 = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeIntdCfgStatusClrSys2Reg */

/*****************************************************************************
 * Combine and write the Data of the Vendor Specific Control register
 ****************************************************************************/
static int32_t Pcie_writeVndrSpecCntrlReg(volatile uint32_t *hwReg_VENDOR_SPECIFIC_CONTROL_REG, Pcie_VndrSpecCntrlReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_VENDOR_SPECIFIC_CONTROL_REG != NULL) && (swReg != NULL))
    {

        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_REGF_VSEC_STRUCT_I_VENDOR_SPECIFIC_CONTROL_REG_VSEC_COUT, swReg->vsecCout);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EP_PF0_I_REGF_VSEC_STRUCT_I_VENDOR_SPECIFIC_CONTROL_REG_HTI,       swReg->hti);

        *hwReg_VENDOR_SPECIFIC_CONTROL_REG = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeVndrSpecCntrlReg */

/*****************************************************************************
 * Combine and write the Data of the LM Vendor ID and Subsystem Vendor ID register
 ****************************************************************************/
static int32_t Pcie_writeLmVendorIdReg(volatile uint32_t *hwReg_I_VENDOR_ID_REG, Pcie_LmVendorIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_I_VENDOR_ID_REG != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_SVID, swReg->svid);
        PCIE_SETBITS(new_val, CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_VID,  swReg->vid);

        *hwReg_I_VENDOR_ID_REG = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeLmVendorIdReg */

/*********************************************************************
 * Writes any register
 ********************************************************************/
int32_t Pcie_writeRegs (Pcie_Handle handle, Pcie_Location location, Pcie_Registers *writeRegs)
{
    int32_t status = SystemP_SUCCESS;
    int32_t i;
    Pcie_Config *pcieCfg = NULL;
    Pcie_DeviceCfgBaseAddr *cfg = NULL;
    Pcie_DeviceCfgBaseAddrs *bases = NULL;
    Pcie_DevParams *params = NULL;

    Pcie_PlconfIatuIndexReg *simIatuWindow = NULL;

    /* Base Address for the Config Space
       These registers can be Local/Remote and Type0(EP)/Type1(RC) */
    CSL_pcie_ep_coreRegs *baseCfgEp = NULL;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
        cfg = Pcie_handleGetBases (handle);

        simIatuWindow = &pciev0LocalObj.simIatuWindow[pcieCfg->attrs->deviceNum];

        if(cfg != NULL)
        {
            bases = (Pcie_DeviceCfgBaseAddrs*)cfg->cfgBase;
            params = (Pcie_DevParams*)cfg->devParams;
        }
        else
        {
            status = SystemP_FAILURE;
        }

        if (status == SystemP_SUCCESS)
        {
            baseCfgEp = (CSL_pcie_ep_coreRegs *)bases->cfgBase;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        /* Get base address for Local or Remote config space */
        if (location != PCIE_LOCATION_LOCAL)
        {
            char *remoteBase  = (char *)cfg->dataBase + bases->remoteOffset;

            if (remoteBase != NULL)
            {
                baseCfgEp = (CSL_pcie_ep_coreRegs *)(remoteBase);
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }

    if ((status == SystemP_SUCCESS) && (writeRegs->cmdStatus != NULL))
    {
        status = Pcie_writeCmdStatusReg ((CSL_pcie_ep_coreRegs *)params->userCfgBase, writeRegs->cmdStatus);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->cfgTrans != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->ioBase != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tlpCfg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->rstCmd != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->ptmCfg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->pmCmd != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->pmCfg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->obSize != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->diagCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->endian != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->priority != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->irqEOI != NULL))
    {
        status = Pcie_writeRcIrqEoi ((CSL_user_cfgRegs *)params->userCfgBase, writeRegs->irqEOI);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->msiIrq != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->epIrqSet != NULL))
    {
        status = Pcie_writeEpIrqSetReg ((CSL_user_cfgRegs *)params->userCfgBase, writeRegs->epIrqSet, 0);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->epIrqClr != NULL))
    {
        status = Pcie_writeEpIrqClrReg ((CSL_user_cfgRegs *)params->userCfgBase, writeRegs->epIrqClr, 0);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->epIrqStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    for (i = 0; i < 4; i++)
    {
        if ((status == SystemP_SUCCESS) && (writeRegs->genPurpose[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }
    for (i = 0; i < 8; i++)
    {
        if ((status == SystemP_SUCCESS) && (writeRegs->msiIrqStatusRaw[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->msiIrqStatus[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->msiIrqEnableSet[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->msiIrqEnableClr[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }
    for (i = 0; i < 4; i++)
    {
        if ((status == SystemP_SUCCESS) && (writeRegs->legacyIrqStatusRaw[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->legacyIrqStatus[i] != NULL))
        {
            status = Pcie_writeLegacyIrqStatusReg ((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->legacyIrqStatus[i], i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->legacyIrqEnableSet[i] != NULL))
        {
            status = Pcie_writeLegacyIrqEnableSetReg ((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->legacyIrqEnableSet[i], i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->legacyIrqEnableClr[i] != NULL))
        {
            status = Pcie_writeLegacyIrqEnableClrReg ((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->legacyIrqEnableClr[i], i);
        }
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->errIrqStatusRaw != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->errIrqStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->errIrqEnableSet != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->errIrqEnableClr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    if ((status == SystemP_SUCCESS) && (writeRegs->pmRstIrqStatusRaw != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->pmRstIrqStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->pmRstIrqEnableSet != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->pmRstIrqEnableClr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    if ((status == SystemP_SUCCESS) && (writeRegs->ptmIrqStatusRaw != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->ptmIrqStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->ptmIrqEnableSet != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->ptmIrqEnableClr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    for (i = 0; i < 8; i ++)
    {
        if ((status == SystemP_SUCCESS) && (writeRegs->obOffsetLo[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->obOffsetHi[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }

    for (i = 0; i < 4; i ++)
    {
        if ((status == SystemP_SUCCESS) && (writeRegs->ibBar[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->ibStartLo[i] != NULL))
        {
          /* Not supported in this version */
          status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->ibStartHi[i] != NULL))
        {
          /* Not supported in this version */
          status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->ibOffset[i] != NULL))
        {
          /* Not supported in this version */
          status = SystemP_FAILURE;
        }
    }

    if ((status == SystemP_SUCCESS) && (writeRegs->pcsCfg0 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->pcsCfg1 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    if ((status == SystemP_SUCCESS) && (writeRegs->serdesCfg0 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->serdesCfg1 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /*****************************************************************************************
    * Configuration Registers
    *****************************************************************************************/

    /* Type 0, Type1 Common Registers*/

    if ((status == SystemP_SUCCESS) && (writeRegs->vndDevId != NULL))
    {
        status = Pcie_writeVendorDevIdReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_VENDOR_ID_DEVICE_ID), writeRegs->vndDevId);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->statusCmd != NULL))
    {
        status = Pcie_writeStatusCmdReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_COMMAND_STATUS), writeRegs->statusCmd);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->baseAddr != NULL))
    {
        status = Pcie_writeBaseAddrReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_BASE_ADDR[0]), writeRegs->baseAddr);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->revId != NULL))
    {
        status = Pcie_writeRevIdReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_REVISION_ID_CLASS_CODE), writeRegs->revId);
    }

    if ((status == SystemP_SUCCESS) && (writeRegs->bist != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* Type 0 Registers*/
    if ((status == SystemP_SUCCESS) && (writeRegs->type0BarIdx != NULL))
    {
        status = Pcie_writeType0BarReg (baseCfgEp, &(writeRegs->type0BarIdx->reg),
                                                         writeRegs->type0BarIdx->idx);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type0BarMask32bitIdx != NULL))
    {
        status = Pcie_writeType0Bar32bitReg (baseCfgEp, &(writeRegs->type0BarMask32bitIdx->reg),
                                                              writeRegs->type0BarMask32bitIdx->idx,
                                                              params->pfNum);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type0Bar32bitIdx != NULL))
    {
        status = Pcie_writeType0Bar32bitReg (baseCfgEp, &(writeRegs->type0Bar32bitIdx->reg),
                                                              writeRegs->type0Bar32bitIdx->idx,
                                                              params->pfNum);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->subId != NULL))
    {
        status = Pcie_writeSubIdReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_SUBSYSTEM_VENDOR_ID_SUBSYSTEM_I), writeRegs->subId);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->cardbusCisPointer != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->expRom != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->capPtr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->intPin != NULL))
    {
        status = Pcie_writeIntPinReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_BASE.I_INTRPT_LINE_INTRPT_PIN), writeRegs->intPin);
    }

    /*Type 1 Registers*/
    if ((status == SystemP_SUCCESS) && (writeRegs->type1BistHeader != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1BarIdx != NULL))
    {
        status = Pcie_writeType1BarReg (baseCfgEp, &(writeRegs->type1BarIdx->reg),
                                                         writeRegs->type1BarIdx->idx);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1BarMask32bitIdx != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1Bar32bitIdx != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1BusNum != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1SecStat != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1Memspace != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->prefMem != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->prefBaseUpper != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->prefLimitUpper != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1IOSpace != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1CapPtr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1ExpnsnRom != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->type1BridgeInt != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* Power Management Capabilities Registers */
    if ((status == SystemP_SUCCESS) && (writeRegs->pmCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->pmCapCtlStat != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* MSI Registers */
    if ((status == SystemP_SUCCESS) && (writeRegs->msiCap != NULL))
    {
        status = Pcie_writeMsiCapReg((CSL_pcie_ep_coreRegs *)&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_MSI_CAP_STRUCT.I_MSI_CTRL_REG, writeRegs->msiCap);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->msiLo32 != NULL))
    {
        status = Pcie_writeMsiLo32Reg(baseCfgEp, writeRegs->msiLo32);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->msiUp32 != NULL))
    {
        status = Pcie_writeMsiUp32Reg(baseCfgEp, writeRegs->msiUp32);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->msiData != NULL))
    {
        status = Pcie_writeMsiDataReg (baseCfgEp, writeRegs->msiData);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->msiCapOff10H != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->msiCapOff14H != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* MSIx Registers */
    if ((status == SystemP_SUCCESS) && (writeRegs->msixCap != NULL))
    {
        status = Pcie_writeMsixCapReg ((CSL_pcie_ep_coreRegs *)&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_MSIX_CAP_STRUCT.I_MSIX_CTRL,
                                        writeRegs->msixCap);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->msixTblOffset != NULL))
    {

        status = Pcie_writeMsixTblOffset ((CSL_pcie_ep_coreRegs *)&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_MSIX_CAP_STRUCT.I_MSIX_TBL_OFFSET,
                                         writeRegs->msixTblOffset);
    }

    /* Capabilities Registers */
    if ((status == SystemP_SUCCESS) && (writeRegs->pciesCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->deviceCap != NULL))
    {
        status = Pcie_writeDevCapReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_CAP_STRUCT.I_PCIE_DEV_CAP), writeRegs->deviceCap);
    }

    if ((status == SystemP_SUCCESS) && (writeRegs->devStatCtrl != NULL))
    {
        status = Pcie_writeDevStatCtrlReg (&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_CAP_STRUCT.I_PCIE_DEV_CTRL_STATUS),
                        writeRegs->devStatCtrl);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->linkCap != NULL))
    {
        status = Pcie_writeLinkCapReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_CAP_STRUCT.I_LINK_CAP), writeRegs->linkCap);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->linkStatCtrl != NULL))
    {
        status = Pcie_writeLinkStatCtrlReg(&(baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_PCIE_CAP_STRUCT.I_LINK_CTRL_STATUS), writeRegs->linkStatCtrl);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->slotCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->slotStatCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->rootCtrlCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->rootStatus != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->devCap2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->devStatCtrl2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->linkCap2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->linkCtrl2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* Capabilities Extended Registers*/
    if ((status == SystemP_SUCCESS) && (writeRegs->extCap != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->uncErr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->uncErrMask != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->uncErrSvrty != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->corErr != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->corErrMask != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->accr != NULL))
    {
        status = Pcie_writeAccrReg (baseCfgEp, writeRegs->accr);
    }
    for (i = 0; i < 4; i ++)
    {
        if ((status == SystemP_SUCCESS) && (writeRegs->hdrLog[i] != NULL))
        {
            /* Not supported in this version */
            status = SystemP_FAILURE;
        }
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->rootErrCmd != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->rootErrSt != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->errSrcID != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /*Port Logic Registers*/
    if ((status == SystemP_SUCCESS) && (writeRegs->plAckTimer != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plOMsg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plForceLink != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->ackFreq != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->lnkCtrl != NULL))
    {
        status = Pcie_writeLnkCtrlReg ((CSL_pcie_ep_coreRegs *)params->pcieCtrlAddr, writeRegs->lnkCtrl);

        if (status == SystemP_SUCCESS)
        {
            params->numLane = writeRegs->lnkCtrl->lnkMode+1;
        }
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->laneSkew != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->symNum != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->symTimerFltMask != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->fltMask2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->gen2 != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* Write Vendor Specific Control register */
    if ((status == SystemP_SUCCESS) && (writeRegs->vndrSpecCntrl != NULL))
    {
        status = Pcie_writeVndrSpecCntrlReg(&baseCfgEp->EP_PF_I_PCIE[params->pfNum].EP_PF_I_REGF_VSEC_STRUCT.I_VENDOR_SPECIFIC_CONTROL_REG, writeRegs->vndrSpecCntrl);
    }

    /* PLCONF registers */
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfObnpSubreqCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfTrPStsR != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfTrNpStsR != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfTrCStsR != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfQStsR != NULL))
    {
        /* Not supported on rev 3 */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfVcTrAR1 != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfVcTrAR2 != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfVc0PrQC != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfVc0NprQC != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfVc0CrQC != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    for (i = 0; i < 3; i++)
    {
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfVcPrQC[i] != NULL))
        {
            /* Pure RO register */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfVcNprQC[i] != NULL))
        {
            /* Pure RO register */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfVcCrQC[i] != NULL))
        {
            /* Pure RO register */
            status = SystemP_FAILURE;
        }
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfPhyStsR != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfPhyCtrlR != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlAddress != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlUpperAddress != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    for (i = 0; i < 8; i++)
    {
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlIntEnable[i] != NULL))
        {
            /* Pure RO register */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlIntMask[i] != NULL))
        {
            /* Pure RO register */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlIntStatus[i] != NULL))
        {
            /* Pure RO register */
            status = SystemP_FAILURE;
        }
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlGpio != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfPipeLoopback != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfDbiRoWrEn != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfAxiSlvErrResp != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfAxiSlvTimeout != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuIndex != NULL))
    {
        /* Set the simulated window address */
        *simIatuWindow = *writeRegs->plconfIatuIndex;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegLowerBase != NULL))
    {
        /* EP or RC */
        if (pcieCfg->attrs->operationMode == PCIE_EP_MODE)
        {
            status = Pcie_writePlconfIatuRegLowerBaseReg (baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegLowerBase);
        }
        else
        {
            status = Pcie_writePlconfIatuRegLowerBaseRcReg (baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegLowerBase);
        }
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegUpperBase != NULL))
    {
        status = Pcie_writePlconfIatuRegUpperBaseReg (baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegUpperBase);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegLimit != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegLowerTarget != NULL))
    {
        status = Pcie_writePlconfIatuRegLowerTargetReg (baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegLowerTarget);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegUpperTarget != NULL))
    {
        status = Pcie_writePlconfIatuRegUpperTargetReg (baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegUpperTarget);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegCtrl3 != NULL))
    {
        /* Pure RO register */
        status = SystemP_FAILURE;
    }
    /* Ctrl1 is done last since it has enable bit */
    if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegCtrl1 != NULL))
    {
        status = Pcie_writePlconfIatuRegCtrl1Reg (baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegCtrl1);
    }

    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfRevision != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfSysConfig != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEoi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqStatusRawMain != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqStatusMain != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEnableSetMain != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEnableClrMain != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqStatusRawMsi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqStatusMsi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEnableSetMsi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEnableClrMsi != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDeviceType != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDeviceCmd != NULL))
    {
        status = Pcie_writeLinkStatusReg ((CSL_pcie_ep_coreRegs *)params->userCfgBase, writeRegs->tiConfDeviceCmd);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfPmCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfPhyCs != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIntxAssert != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIntxDeassert != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfMsiXmt != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDebugCfg != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDebugData != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDiagCtrl != NULL))
    {
        /* Not supported in this version */
        status = SystemP_FAILURE;
    }

    /* User Config Registers */
    if ((status == SystemP_SUCCESS) && (writeRegs->usrCfgInitCfg != NULL))
    {
        status = Pcie_writeUsrInitCfgReg((CSL_user_cfgRegs *)params->userCfgBase, writeRegs->usrCfgInitCfg);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->usrCfgPmCmd != NULL))
    {
        status = Pcie_writeUsrPmCmdReg((CSL_user_cfgRegs *)params->userCfgBase, writeRegs->usrCfgPmCmd);
    }

    /* Intd Config Registers */
    if ((status == SystemP_SUCCESS) && (writeRegs->intCfgEnSys0Reg != NULL))
    {
        status = Pcie_writeIntdCfgEnSys0Reg((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->intCfgEnSys0Reg);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->intCfgEnSys1Reg != NULL))
    {
        status = Pcie_writeIntdCfgEnSys1Reg((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->intCfgEnSys1Reg);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->intCfgEnSys2Reg != NULL))
    {
        status = Pcie_writeIntdCfgEnSys2Reg((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->intCfgEnSys2Reg);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->intCfgStatusSys0Reg != NULL))
    {
        status = Pcie_writeIntdCfgStatusSys0Reg((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->intCfgStatusSys0Reg);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->intCfgStatusSys0Reg != NULL))
    {
        status = Pcie_writeIntdCfgStatusSys1Reg((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->intCfgStatusSys1Reg);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->intCfgStatusSys2Reg != NULL))
    {
        status = Pcie_writeIntdCfgStatusSys2Reg((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->intCfgStatusSys2Reg);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->intCfgStatusClrSys2Reg != NULL))
    {
        status = Pcie_writeIntdCfgStatusClrSys2Reg((CSL_intd_cfgRegs *)params->intCfgBase, writeRegs->intCfgStatusClrSys2Reg);
    }
    if ((status == SystemP_SUCCESS) && (writeRegs->lmVendorIdReg != NULL))
    {
        status =  Pcie_writeLmVendorIdReg(&(baseCfgEp->LM_I_REGF_LM_PCIE_BASE.I_VENDOR_ID_REG), writeRegs->lmVendorIdReg);
    }

    return status;
} /* Pcie_writeRegs */


int32_t Pcie_getMsiRegs (Pcie_Handle handle, Pcie_Location location, Pcie_MsiParams *params)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_Registers regs;
    Pcie_MsiCapReg msiCapReg;
    Pcie_MsiLo32Reg msiLo32;
    Pcie_MsiUp32Reg msiUp32;
    Pcie_MsiDataReg msiData;

    if ((handle != NULL) || (params == NULL))
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msiCap = &msiCapReg;
        regs.msiLo32 = &msiLo32;
        regs.msiUp32 = &msiUp32;
        regs.msiData = &msiData;

        status = Pcie_readRegs (handle, location, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        params->enable = msiCapReg.msiEn;
        params->loAddr = msiLo32.addr;
        params->upAddr = msiUp32.addr;
        params->data   = msiData.data;
        params->multMsgEn = msiCapReg.multMsgEn;
        pcieCfg->attrs->msiMme = 1 << msiCapReg.multMsgEn;
    }

    return status;
}

int32_t Pcie_setMsiRegs (Pcie_Handle handle, Pcie_Location location, Pcie_MsiParams params)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_MsiCapReg msiCapReg;
    Pcie_MsiLo32Reg msiLo32;
    Pcie_MsiUp32Reg msiUp32;
    Pcie_MsiDataReg msiData;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msiCap = &msiCapReg;
        regs.msiLo32 = &msiLo32;
        regs.msiUp32 = &msiUp32;
        regs.msiData = &msiData;

        status = Pcie_readRegs (handle, location, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        msiCapReg.msiEn = params.enable;
        msiLo32.addr = params.loAddr;
        msiUp32.addr = params.upAddr;
        msiData.data = params.data;

        status = Pcie_writeRegs (handle, location, &regs);
    }

    return status;
}

int32_t Pcie_setMsixTblOffset (Pcie_Handle handle, Pcie_Location location, Pcie_MsixTblParams params)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_MsixTblOffset msixTblOffset;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msixTblOffset = &msixTblOffset;

        status = Pcie_readRegs(handle, location, &regs);

        if(status == SystemP_SUCCESS)
        {
            msixTblOffset.barIndex = params.barIndex;
            msixTblOffset.offset = (params.tbl_offset & (~0x7u));

            status = Pcie_writeRegs (handle, location, &regs);
        }
    }

    return status;
}

int32_t Pcie_getMsixTblOffset (Pcie_Handle handle, Pcie_Location location, Pcie_MsixTblParams *params)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_MsixTblOffset msixTblOffset;

    if ((handle == NULL) || (params == NULL))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msixTblOffset = &msixTblOffset;

        status = Pcie_readRegs(handle, location, &regs);

        if(status == SystemP_SUCCESS)
        {
            params->tbl_offset = msixTblOffset.offset;
            params->barIndex = msixTblOffset.barIndex;
        }
    }

    return status;
}

int32_t Pcie_getMsixCtrlRegister (Pcie_Handle handle, Pcie_Location location,
                                         Pcie_MsixCtrlReg *params)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_MsixCapReg msixCap;

    if ((params == NULL) || (handle == NULL))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msixCap = &msixCap;

        status = Pcie_readRegs(handle, location, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        params->enable = msixCap.msixEn;
        params->tblSize = msixCap.msixTblSize;
    }

    return status;
}

int32_t Pcie_setMsixCtrlRegister (Pcie_Handle handle, Pcie_Location location,
                                         Pcie_MsixCtrlReg params)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_MsixCapReg msixCap;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msixCap = &msixCap;

        status = Pcie_readRegs(handle, location, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        msixCap.msixEn = params.enable;
        msixCap.msixTblSize = params.tblSize;

        status = Pcie_writeRegs(handle, location, &regs);
    }

    return status;
}

/*****************************************************************************
 * Disable support for MSI-X
 ****************************************************************************/
int32_t Pcie_epDisableMsixCap(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers  regs;
    Pcie_MsixCapReg msixCap;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msixCap = &msixCap;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        msixCap.capId = 0x00;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Set number of MSI messages we request (multiple message capable)
 * according to configuration
 ****************************************************************************/
int32_t Pcie_epSetMsiMultiMessCap(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Config *pcieCfg = NULL;
    Pcie_Registers  regs;
    Pcie_MsiCapReg msiCap;

    if (handle != NULL)
    {
        pcieCfg = (Pcie_Config *)handle;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msiCap = &msiCap;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {

        /*
         * determine the multMsgCap bit field by counting trailing zeros,
         * since the number of requested MSI vectors is encoded as power of two
         */
        msiCap.multMsgCap = __builtin_ctz(pcieCfg->attrs->msiMmc);

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

/*****************************************************************************
 * Disable support for MSI per vector masking
 ****************************************************************************/
int32_t Pcie_epDisableMsiPerVecMaskCap(Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers  regs;
    Pcie_MsiCapReg msiCap;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.msiCap = &msiCap;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        msiCap.pvmEn = 0;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;
}

int32_t Pcie_rcLegacyIrqEoi (Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_IrqEOIReg irqEoi;

    if (handle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        memset (&regs, 0, sizeof(regs));

        regs.irqEOI = &irqEoi;

        status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    if (status == SystemP_SUCCESS)
    {
        /* Signal EOI for legacy IRQ */
        irqEoi.EOI = 0x2U;

        status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs);
    }

    return status;

    return status;
}
