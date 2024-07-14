/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include <drivers/pcie/v1/pcie_v1_reg.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_pcie.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PCIE_REV2_CLASSCODE_MASK (                                   \
    CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_PROGRAM_INTERFACE_MASK | \
    CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_SUBCLASS_CODE_MASK |     \
    CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_BASE_CLASS_CODE_MASK)

#define PCIE_REV2_CLASSCODE_SHIFT (CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_PROGRAM_INTERFACE_SHIFT)

#define PCIE_REV2_TPH_CMPLT_SUPPORT_MASK (                                        \
    CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_0_MASK | \
    CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_1_MASK)
#define PCIE_REV2_TPH_CMPLT_SUPPORT_SHIFT (CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_0_SHIFT)

/* BAR mask */
#define PCIE_BAR_MASK       0x0FFFFFFF

/* Maximum number of outbound regions */
#define OB_REGION_MAX       16

/* Maximum number of inbound regions for EP */
#define IB_REGION_MAX_EP    8

/* Maximum number of inbound regions for RC */
#define IB_REGION_MAX_RC    2

#define KICK0 0x68EF3490ull

#define KICK1 0xD172BC5Aull
/*****************************************************************************
 **********  PCIe LOCAL/REMOTE PORT LOGIC REGISTERS **********************
 ****************************************************************************/

/* Define bitfield positions */
#define PCIE_FLTMASK1_CFG_DROP_MASK (0x00008000U)
#define PCIE_FLTMASK1_CFG_DROP_SHIFT (0x0000000FU)
#define PCIE_FLTMASK1_IO_DROP_MASK (0x00004000U)
#define PCIE_FLTMASK1_IO_DROP_SHIFT (0x0000000EU)
#define PCIE_FLTMASK1_MSG_DROP_MASK (0x00002000U)
#define PCIE_FLTMASK1_MSG_DROP_SHIFT (0x0000000DU)
#define PCIE_FLTMASK1_CPL_ECRC_DROP_MASK (0x00001000U)
#define PCIE_FLTMASK1_CPL_ECRC_DROP_SHIFT (0x0000000CU)

#define PCIE_FLTMASK1_ECRC_DROP_MASK (0x00000800U)
#define PCIE_FLTMASK1_ECRC_DROP_SHIFT (0x0000000BU)
#define PCIE_FLTMASK1_CPL_LEN_TEST_MASK (0x00000400U)
#define PCIE_FLTMASK1_CPL_LEN_TEST_SHIFT (0x0000000AU)
#define PCIE_FLTMASK1_CPL_ATTR_TEST_MASK (0x00000200U)
#define PCIE_FLTMASK1_CPL_ATTR_TEST_SHIFT (0x00000009U)
#define PCIE_FLTMASK1_CPL_TC_TEST_MASK (0x00000100U)
#define PCIE_FLTMASK1_CPL_TC_TEST_SHIFT (0x00000008U)

#define PCIE_FLTMASK1_CPL_FUNC_TEST_MASK (0x00000080U)
#define PCIE_FLTMASK1_CPL_FUNC_TEST_SHIFT (0x00000007U)
#define PCIE_FLTMASK1_CPL_REQID_TEST_MASK (0x00000040U)
#define PCIE_FLTMASK1_CPL_REQID_TEST_SHIFT (0x00000006U)
#define PCIE_FLTMASK1_CPL_TAGERR_TEST_MASK (0x00000020U)
#define PCIE_FLTMASK1_CPL_TAGERR_TEST_SHIFT (0x00000005U)
#define PCIE_FLTMASK1_LOCKED_RD_AS_UR_MASK (0x00000010U)
#define PCIE_FLTMASK1_LOCKED_RD_AS_UR_SHIFT (0x00000004U)

#define PCIE_FLTMASK1_CFG1_RE_AS_US_MASK (0x00000008U)
#define PCIE_FLTMASK1_CFG1_RE_AS_US_SHIFT (0x00000003U)
#define PCIE_FLTMASK1_UR_OUT_OF_BAR_MASK (0x00000004U)
#define PCIE_FLTMASK1_UR_OUT_OF_BAR_SHIFT (0x00000002U)
#define PCIE_FLTMASK1_UR_POISON_MASK (0x00000002U)
#define PCIE_FLTMASK1_UR_POISON_SHIFT (0x00000001U)
#define PCIE_FLTMASK1_UR_FUN_MISMATCH_MASK (0x00000001U)
#define PCIE_FLTMASK1_UR_FUN_MISMATCH_SHIFT (0x00000000U)

/* Define bitfield positions */
#define PCIE_FLTMASK2_DROP_PRS_MASK (0x00000080U)
#define PCIE_FLTMASK2_DROP_PRS_SHIFT (0x00000007U)
#define PCIE_FLTMASK2_UNMASK_TD_MASK (0x00000040U)
#define PCIE_FLTMASK2_UNMASK_TD_SHIFT (0x00000006U)
#define PCIE_FLTMASK2_UNMASK_UR_POIS_MASK (0x00000020U)
#define PCIE_FLTMASK2_UNMASK_UR_POIS_SHIFT (0x00000005U)
#define PCIE_FLTMASK2_DROP_LN_MASK (0x00000010U)
#define PCIE_FLTMASK2_DROP_LN_SHIFT (0x00000004U)
#define PCIE_FLTMASK2_FLUSH_REQ_MASK (0x00000008U)
#define PCIE_FLTMASK2_FLUSH_REQ_SHIFT (0x00000003U)
#define PCIE_FLTMASK2_DLLP_ABORT_MASK (0x00000004U)
#define PCIE_FLTMASK2_DLLP_ABORT_SHIFT (0x00000002U)
#define PCIE_FLTMASK2_VMSG1_DROP_MASK (0x00000002U)
#define PCIE_FLTMASK2_VMSG1_DROP_SHIFT (0x00000001U)
#define PCIE_FLTMASK2_VMSG0_DROP_MASK (0x00000001U)
#define PCIE_FLTMASK2_VMSG0_DROP_SHIFT (0x00000000U)

/* DEBUG0 */
#define PCIE_DEBUG0_TS_LINK_CTRL_MASK (0xF0000000u)
#define PCIE_DEBUG0_TS_LINK_CTRL_SHIFT (0x0000001Cu)

#define PCIE_DEBUG0_TS_LANE_K237_MASK (0x08000000u)
#define PCIE_DEBUG0_TS_LANE_K237_SHIFT (0x0000001Bu)

#define PCIE_DEBUG0_TS_LINK_K237_MASK (0x04000000u)
#define PCIE_DEBUG0_TS_LINK_K237_SHIFT (0x0000001Au)

#define PCIE_DEBUG0_RCVD_IDLE0_MASK (0x02000000u)
#define PCIE_DEBUG0_RCVD_IDLE0_SHIFT (0x00000019u)

#define PCIE_DEBUG0_RCVD_IDLE1_MASK (0x01000000u)
#define PCIE_DEBUG0_RCVD_IDLE1_SHIFT (0x00000018u)

#define PCIE_DEBUG0_PIPE_TXDATA_MASK (0x00FFFF00u)
#define PCIE_DEBUG0_PIPE_TXDATA_SHIFT (0x00000008u)

#define PCIE_DEBUG0_PIPE_TXDATAK_MASK (0x000000C0u)
#define PCIE_DEBUG0_PIPE_TXDATAK_SHIFT (0x00000006u)

#define PCIE_DEBUG0_TXB_SKIP_TX_MASK (0x00000020u)
#define PCIE_DEBUG0_TXB_SKIP_TX_SHIFT (0x00000005u)

#define PCIE_DEBUG0_LTSSM_STATE_MASK (0x0000001Fu)
#define PCIE_DEBUG0_LTSSM_STATE_SHIFT (0x00000000u)

/* DEBUG1 */

#define PCIE_DEBUG1_SCRAMBLER_DISABLE_MASK (0x80000000u)
#define PCIE_DEBUG1_SCRAMBLER_DISABLE_SHIFT (0x0000001Fu)

#define PCIE_DEBUG1_LINK_DISABLE_MASK (0x40000000u)
#define PCIE_DEBUG1_LINK_DISABLE_SHIFT (0x0000001Eu)

#define PCIE_DEBUG1_LINK_IN_TRAINING_MASK (0x20000000u)
#define PCIE_DEBUG1_LINK_IN_TRAINING_SHIFT (0x0000001Du)

#define PCIE_DEBUG1_RCVR_REVRS_POL_EN_MASK (0x10000000u)
#define PCIE_DEBUG1_RCVR_REVRS_POL_EN_SHIFT (0x0000001Cu)

#define PCIE_DEBUG1_TRAINING_RST_N_MASK (0x08000000u)
#define PCIE_DEBUG1_TRAINING_RST_N_SHIFT (0x0000001Bu)

#define PCIE_DEBUG1_PIPE_TXDETECTRX_LB_MASK (0x00400000u)
#define PCIE_DEBUG1_PIPE_TXDETECTRX_LB_SHIFT (0x00000016u)

#define PCIE_DEBUG1_PIPE_TXELECIDLE_MASK (0x00200000u)
#define PCIE_DEBUG1_PIPE_TXELECIDLE_SHIFT (0x00000015u)

#define PCIE_DEBUG1_PIPE_TXCOMPLIANCE_MASK (0x00100000u)
#define PCIE_DEBUG1_PIPE_TXCOMPLIANCE_SHIFT (0x00000014u)

#define PCIE_DEBUG1_APP_INIT_RST_MASK (0x00080000u)
#define PCIE_DEBUG1_APP_INIT_RST_SHIFT (0x00000013u)

#define PCIE_DEBUG1_RMLH_TS_LINK_NUM_MASK (0x0000FF00u)
#define PCIE_DEBUG1_RMLH_TS_LINK_NUM_SHIFT (0x00000008u)

#define PCIE_DEBUG1_XMLH_LINK_UP_MASK (0x00000010u)
#define PCIE_DEBUG1_XMLH_LINK_UP_SHIFT (0x00000004u)

#define PCIE_DEBUG1_RMLH_INSKIP_RCV_MASK (0x00000008u)
#define PCIE_DEBUG1_RMLH_INSKIP_RCV_SHIFT (0x00000003u)

#define PCIE_DEBUG1_RMLH_TS1_RCVD_MASK (0x00000004u)
#define PCIE_DEBUG1_RMLH_TS1_RCVD_SHIFT (0x00000002u)

#define PCIE_DEBUG1_RMLH_TS2_RCVD_MASK (0x00000002u)
#define PCIE_DEBUG1_RMLH_TS2_RCVD_SHIFT (0x00000001u)

#define PCIE_DEBUG1_RMLH_RCVD_LANE_REV_MASK (0x00000001u)
#define PCIE_DEBUG1_RMLH_RCVD_LANE_REV_SHIFT (0x00000000u)


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
extern uint32_t dst_buf[];
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
static int32_t Pcie_readVendorDevId(volatile const uint32_t *hwReg_VENDOR_ID_DEV_ID, Pcie_VndDevIdReg *swReg);
static int32_t Pcie_readDevStatCtrlReg (volatile const uint32_t *hwReg_DEV_CAS, Pcie_DevStatCtrlReg *swReg);
static int32_t Pcie_readLinkCapReg (volatile const uint32_t *hwReg_LNK_CAP, Pcie_LinkCapReg *swReg);
static int32_t Pcie_readLinkStatCtrlReg (volatile const uint32_t *hwReg_LNK_CAS, Pcie_LinkStatCtrlReg *swReg);
static int32_t Pcie_readAccrReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_AccrReg *swReg);
static int32_t Pcie_readEpIrqSetReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_EpIrqSetReg *swReg);
static int32_t Pcie_readEpIrqClrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_EpIrqClrReg *swReg);

static int32_t Pcie_readStatusCmdReg(volatile const uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                            Pcie_StatusCmdReg *swReg);

static int32_t Pcie_readRevIdReg(volatile const uint32_t *hwReg_CLASSCODE_REVISIONID,
                            Pcie_RevIdReg *swReg);
static int32_t Pcie_readType0BarReg( const CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                            int32_t barNum);
static int32_t Pcie_readType1BarReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                            int32_t barNum);
static int32_t Pcie_readType0Bar32bitReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_Bar32bitReg *swReg,
                            int32_t barNum);
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
static int32_t Pcie_readMsixTblOffset (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsixTblOffset *swReg);

/* Register write functions */
static int32_t Pcie_writeCmdStatusReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_CmdStatusReg *swReg);
static int32_t Pcie_writeDevStatCtrlReg (volatile uint32_t *hwReg_DEV_CAS, Pcie_DevStatCtrlReg *swReg);
static int32_t Pcie_writeLinkCapReg (volatile uint32_t *hwReg_LNK_CAP, Pcie_LinkCapReg *swReg);
static int32_t Pcie_writeAccrReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_AccrReg *swReg);
static int32_t Pcie_writeLinkStatusReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_TiConfDeviceCmdReg *swReg);
static int32_t Pcie_writeEpIrqSetReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_EpIrqSetReg *swReg);
static int32_t Pcie_writeEpIrqClrReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_EpIrqClrReg *swReg);
static int32_t Pcie_writeStatusCmdReg (volatile uint32_t *hwReg_STATUS_COMMAND_REGISTER,
                            Pcie_StatusCmdReg *swReg);
static int32_t Pcie_writeRevIdReg (volatile uint32_t *hwReg_CLASSCODE_REVISIONID,
                            Pcie_RevIdReg *swReg);
static int32_t Pcie_writeType0BarReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_BarReg *swReg,
                            int32_t barNum);
static int32_t Pcie_writeType0Bar32bitReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_Bar32bitReg *swReg,
                            int32_t barNum);
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

static int32_t Pcie_obTransCfg(Pcie_Handle handle);
static void Pcie_setMode(Pcie_DevParams *devParams, uint32_t index, Pcie_Mode mode);
static uint32_t Pcie_getNumPassBitFromWinSize(uint32_t winSize);

/* Read number of lanes configured */
extern int32_t Pcie_readLnkCtrlReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_LnkCtrlReg *swReg);
/* Set number of lanes */
extern int32_t Pcie_writeLnkCtrlReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_LnkCtrlReg *swReg);
/* Lock PCIe MMR configuration register */
extern void Pcie_lockMMRCtrlReg(void);
/* Unlock PCIe MMR configuration register */
extern void Pcie_unlockMMRCtrlReg(void);

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
        uint32_t val = swReg->raw = baseAddr->PID;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PID_MODID,   swReg->modId);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PID_RTL,     swReg->rtl);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PID_MAJOR,   swReg->revMaj);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PID_CUSTOM,  swReg->cust);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PID_MINOR,   swReg->revMin);

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
        uint32_t val = swReg->raw = baseAddr->CMD_STATUS;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CMD_STATUS_RX_LANE_FLIP_EN,  swReg->rxLaneFlipEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CMD_STATUS_TX_LANE_FLIP_EN,  swReg->txLaneFlipEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CMD_STATUS_DBI_CS2,          swReg->dbi);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CMD_STATUS_APP_RETRY_EN,     swReg->appRetryEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CMD_STATUS_LTSSM_EN,         swReg->ltssmEn);

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
static void Pcie_setMode(Pcie_DevParams *devParams, uint32_t index, Pcie_Mode mode)
{
    uint32_t regVal;

    switch (mode)
    {
        case PCIE_EP_MODE:
            regVal = 0U;
            break;
        case PCIE_LEGACY_EP_MODE:
            regVal = 1U;
        case PCIE_RC_MODE:
        default:
            regVal = 2U;
            break;
    }

    /* reset pcie -- moves to sciclient */
    *((volatile uint32_t *)(uintptr_t)(CSL_PSC0_BASE + 0x324)) = 0x00000001;
    *((volatile uint32_t *)(uintptr_t)(CSL_PSC0_BASE + 0xa4c + 0x4 * index)) = 0x00000101;
    *((volatile uint32_t *)(uintptr_t)(CSL_PSC0_BASE + 0x120)) = 0x00000200;

    while (HW_RD_REG32(CSL_PSC0_BASE + 0x128) != 0x0);
    *devParams->Pcie_SSModeAddr = regVal;

    /* un-reset pcie -- moves to sciclient */
    *((volatile uint32_t *)(uintptr_t)(CSL_PSC0_BASE + 0x324)) = 0x00000001;
    *((volatile uint32_t *)(uintptr_t)(CSL_PSC0_BASE + 0xa4c + 0x4 * index)) = 0x00000103;
    *((volatile uint32_t *)(uintptr_t)(CSL_PSC0_BASE + 0x120)) = 0x00000200;

    while (HW_RD_REG32(CSL_PSC0_BASE + 0x128) != 0x0);

} /* Pcie_setMode */

int32_t Pcie_setInterfaceMode(Pcie_Handle handle, Pcie_Mode mode, Pcie_Gen gen)
{
    Pcie_DeviceCfgBaseAddr *baseAddr;
    Pcie_DevParams *devParams;
    Pcie_Config *pcieCfg = NULL;

    pcieCfg = (Pcie_Config *)handle;
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
            devParams = (Pcie_DevParams *)baseAddr->devParams;
            Pcie_setMode(devParams, pcieCfg->attrs->deviceNum, mode);
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

/*****************************************************************************
 * Check LTSSM status and wait for the link to be up
 ****************************************************************************/
int32_t Pcie_waitLinkUp(Pcie_Handle handle)
{
    Pcie_Registers getRegs;
    int32_t status = SystemP_SUCCESS;

    memset (&getRegs, 0, sizeof(getRegs));

    Pcie_Debug0Reg             ltssmStateReg;
    getRegs.debug0 =           &ltssmStateReg;

    memset (&ltssmStateReg,  0, sizeof(ltssmStateReg));

    uint8_t ltssmState = 0;

    while((ltssmState != PCIE_LTSSM_L0) && (SystemP_SUCCESS == status))
    {
        ClockP_usleep(10);

        status = Pcie_readRegs (handle, PCIE_LOCATION_LOCAL, &getRegs);

        ltssmState = ltssmStateReg.ltssmState;
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

static int32_t Pcie_CfgDbiRWE(Pcie_Handle handle, uint8_t enable)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_PlconfDbiRoWrEnReg dbiRo;

    memset(&dbiRo, 0, sizeof(dbiRo));
    memset(&regs, 0, sizeof(regs));

    dbiRo.cxDbiRoWrEn = enable;
    regs.plconfDbiRoWrEn = &dbiRo;

    if ((status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs)) != SystemP_SUCCESS)
    {
        return status;
    }
    return status;
}

static int32_t Pcie_SetGen2(Pcie_Handle handle)
{
    int32_t status;

    Pcie_Registers regs;
    Pcie_LinkCapReg linkCap;
    Pcie_LinkCtrl2Reg linkCtrl2;
    Pcie_Gen2Reg gen2;

    uint8_t targetGen, dirSpd;

    targetGen = 2;
    dirSpd = 1;

    memset(&gen2, 0, sizeof(gen2));
    memset(&linkCap, 0, sizeof(linkCap));
    memset(&linkCtrl2, 0, sizeof(linkCtrl2));
    memset(&regs, 0, sizeof(regs));

    /* Set gen1/gen2 in link cap */
    regs.linkCap = &linkCap;
    if ((status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs)) != SystemP_SUCCESS)
    {
        return status;
    }

    if (linkCap.maxLinkSpeed != targetGen)
    {
        linkCap.maxLinkSpeed = targetGen;
    }
    else
    {
        regs.linkCap = NULL; /* Nothing to write back */
    }
    /* Set gen2/gen3 in link ctrl2 */
    regs.linkCtrl2 = &linkCtrl2;
    if ((status = Pcie_readRegs(handle, PCIE_LOCATION_LOCAL, &regs)) != SystemP_SUCCESS)
    {
        return status;
    }

    if (linkCtrl2.tgtSpeed != targetGen)
    {
        linkCtrl2.tgtSpeed = targetGen;
    }
    else
    {
        regs.linkCtrl2 = NULL; /* Nothing to write back */
    }

    /* Setting PL_GEN2 */
    gen2.numFts = 0xF;
    gen2.dirSpd = dirSpd;
    gen2.lnEn = 2;
    regs.gen2 = &gen2;

    if ((status = Pcie_writeRegs(handle, PCIE_LOCATION_LOCAL, &regs)) != SystemP_SUCCESS)
    {
        return status;
    }

    return status;
}

/*****************************************************************************
 * Function: Configure PCIe in Root complex Mode
 ****************************************************************************/
int32_t Pcie_cfgRC (Pcie_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    Pcie_Registers setRegs;
    Pcie_Registers getRegs;
    Pcie_StatusCmdReg statusCmd;
    Pcie_DevStatCtrlReg devStatCtrl;
    Pcie_AccrReg accr;

    status = Pcie_LtssmCtrl(handle, FALSE);

    if ((status = Pcie_CfgDbiRWE(handle, 1)) != SystemP_SUCCESS)
    {
        return status;
    }

    if ((status = Pcie_SetGen2(handle)) != SystemP_SUCCESS)
    {
        return status;
    }

    if ((status = Pcie_CfgDbiRWE(handle, 0)) != SystemP_SUCCESS)
    {
        return status;
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

    return status;
}

/*****************************************************************************
 * Function: Configure PCIe in End Point Mode
 ****************************************************************************/
int32_t Pcie_cfgEP (Pcie_Handle handle)
{
    int32_t status;

    Pcie_Type0Bar32bitIdx type0Bar32bitIdx;
    Pcie_StatusCmdReg     statusCmd;
    Pcie_DevStatCtrlReg   devStatCtrl;
    Pcie_AccrReg          accr;

    Pcie_Registers        setRegs;
    Pcie_Registers        getRegs;

    memset (&type0Bar32bitIdx, 0, sizeof(type0Bar32bitIdx));
    memset (&statusCmd,        0, sizeof(statusCmd));
    memset (&devStatCtrl,      0, sizeof(devStatCtrl));
    memset (&accr,             0, sizeof(accr));

    status = Pcie_LtssmCtrl(handle, FALSE);

    /* Configure the size of the translation regions */
    memset (&setRegs, 0, sizeof(setRegs));
    memset (&getRegs, 0, sizeof(getRegs));

    if (SystemP_SUCCESS == status)
    {
        /* Configure Masks*/
        memset (&setRegs, 0, sizeof(setRegs));
        memset (&getRegs, 0, sizeof(getRegs));
        type0Bar32bitIdx.reg.reg32 = PCIE_BAR_MASK;
        setRegs.type0BarMask32bitIdx = &type0Bar32bitIdx;

        status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &setRegs);
    }

    if (SystemP_SUCCESS == status)
    {
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

    }

    if (SystemP_SUCCESS == status)
    {
        /* Enable Error Reporting */
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

    if (SystemP_SUCCESS == status)
    {
        /* Enable ECRC */
        memset (&setRegs, 0, sizeof(setRegs));

        accr.chkEn=1;
        accr.chkCap=1;
        accr.genEn=1;
        accr.genCap=1;
        setRegs.accr = &accr;

        status = Pcie_writeRegs (handle, PCIE_LOCATION_LOCAL, &setRegs);
    }

    return status;
}

/*********************************************************************
 * FUNCTION PURPOSE: Configures a BAR Register (32bits)
 ********************************************************************/
int32_t Pcie_cfgBar (Pcie_Handle handle, const Pcie_BarCfg *barCfg)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Type0BarIdx  type0BarIdx;
    Pcie_Registers setRegs;
    Pcie_Registers getRegs;
    uint32_t barAddrField = 0;

    memset (&getRegs,     0, sizeof(getRegs));
    memset (&setRegs,     0, sizeof(setRegs));
    memset (&type0BarIdx, 0, sizeof(type0BarIdx));

    if (barCfg != NULL)
    {
        if(barCfg->mode == PCIE_RC_MODE)
        {
            status = SystemP_FAILURE;
        }
        else
        {

            PCIE_GETBITS(barCfg->base, CSL_PCIE_EP_CORE_BAR_REG_BAR_START, barAddrField);

            if(SystemP_SUCCESS == status)
            {
                type0BarIdx.reg.base        = barAddrField;
                type0BarIdx.reg.prefetch    = barCfg->prefetch;
                type0BarIdx.reg.type        = barCfg->type;
                type0BarIdx.reg.memSpace    = barCfg->memSpace;
                type0BarIdx.idx             = barCfg->idx;

                setRegs.type0BarIdx = &type0BarIdx;
                status = Pcie_writeRegs (handle, barCfg->location, &setRegs);
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
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
        *size = (uint32_t)0x10000000; /* 256 MB */
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_getMemSpaceRange */

/*********************************************************************
 * FUNCTION PURPOSE: Configures an ATU (address translation) region
 ********************************************************************/
int32_t Pcie_atuRegionConfig (Pcie_Handle handle, Pcie_Location location,
            uint32_t atuRegionIndex, const Pcie_AtuRegionParams *atuRegionParams)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_DeviceCfgBaseAddr *cfg = NULL;
    Pcie_DeviceCfgBaseAddrs *bases = NULL;

    Pcie_PlconfIatuIndexReg index;
    Pcie_PlconfIatuRegCtrl1Reg ctrl1;
    Pcie_PlconfIatuRegCtrl2Reg ctrl2;
    Pcie_PlconfIatuRegLowerBaseReg lowerBase;
    Pcie_PlconfIatuRegUpperBaseReg upperBase;
    Pcie_PlconfIatuRegLimitReg limit;
    Pcie_PlconfIatuRegLowerTargetReg lowerTarget;
    Pcie_PlconfIatuRegUpperTargetReg upperTarget;
    Pcie_Registers regs;

    if(handle != NULL)
    {
        cfg = Pcie_handleGetBases (handle);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (cfg != NULL)
    {
        bases = (Pcie_DeviceCfgBaseAddrs*)cfg->cfgBase;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (bases == NULL)
    {
        status = SystemP_FAILURE;
    }

  /* Set up register pointer for interesting registers */
  memset (&regs, 0, sizeof(regs));
  regs.plconfIatuIndex       = &index;

  /* Read current values for index */
  status = Pcie_readRegs (handle, location, &regs);
  if (status == SystemP_SUCCESS)
  {
    /* Update ATU index register with new region direction and region index.
    **/
    switch (atuRegionParams->regionDir)
    {
      case PCIE_ATU_REGION_DIR_OUTBOUND:
        index.regionDirection = 0U; /* Outbound - emulates v1 CSL */
        break;
      case PCIE_ATU_REGION_DIR_INBOUND:
      default:
        index.regionDirection = 1U; /* Inbound - emulates v1 CSL */
        break;
    }
    index.regionIndex = atuRegionIndex;

    /* Writeback the new values for index */
    status = Pcie_writeRegs (handle, location, &regs);
    if (status == SystemP_SUCCESS)
    {
      regs.plconfIatuIndex          = NULL;
      regs.plconfIatuRegCtrl1       = &ctrl1;
      regs.plconfIatuRegCtrl2       = &ctrl2;
      regs.plconfIatuRegLowerBase   = &lowerBase;
      regs.plconfIatuRegUpperBase   = &upperBase;
      regs.plconfIatuRegLimit       = &limit;
      regs.plconfIatuRegLowerTarget = &lowerTarget;
      regs.plconfIatuRegUpperTarget = &upperTarget;

      /* Read current values of rest of registers for this index */
      status = Pcie_readRegs (handle, location, &regs);
      if (status == SystemP_SUCCESS)
      {
        /* Set TLP(Transaction Layer packet) type. */
        switch (atuRegionParams->tlpType)
        {
          case PCIE_TLP_TYPE_IO:
            ctrl1.type = 2U;
            break;
          case PCIE_TLP_TYPE_CFG:
            ctrl1.type = 4U;
            break;
          case PCIE_TLP_TYPE_MEM:
          default:
            ctrl1.type = 0U;
            break;
        }

        /* Configure ATU control2 register. */
        /* Enable region. */
        ctrl2.regionEnable = atuRegionParams->enableRegion;
        if (PCIE_ATU_REGION_DIR_INBOUND == atuRegionParams->regionDir)
        {
          /* Set match mode. */
          switch (atuRegionParams->matchMode)
          {
            case PCIE_ATU_REGION_MATCH_MODE_ADDR:
             ctrl2.matchMode = 0u;
             break;
            case PCIE_ATU_REGION_MATCH_MODE_BAR:
            default:
             ctrl2.matchMode = 1u;
             break;
          }

          /* Set BAR number. */
          ctrl2.barNumber = atuRegionParams->barNumber;
        }

        /* Configure lower base. */
        lowerBase.iatuRegLowerBase = atuRegionParams->lowerBaseAddr >> 16;

        /* Configure upper base. */
        upperBase.iatuRegUpperBase = atuRegionParams->upperBaseAddr;

        /* Configure window size. */
        limit.iatuRegLimit = (atuRegionParams->lowerBaseAddr +
                  atuRegionParams->regionWindowSize) >> 16;

        /* Configure lower target. */
        lowerTarget.iatuRegLowerTarget = atuRegionParams->lowerTargetAddr >> 16;

        /* Configure Upper target. */
        upperTarget.iatuRegUpperTarget = atuRegionParams->upperTargetAddr;

        /* Writeback the new values */
        status = Pcie_writeRegs (handle, location, &regs);
      }
    }
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
        /* Do nothing */
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
            regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
            regionParams.tlpType      = PCIE_TLP_TYPE_CFG;
            regionParams.enableRegion = 1;

            regionParams.lowerBaseAddr    = PCIE_WINDOW_CFG_BASE + resSize;

            regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
            regionParams.regionWindowSize = PCIE_WINDOW_CFG_MASK;

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
    Pcie_AtuRegionParams regionParams;
    Pcie_BarCfg          barCfg;
    uint32_t             resSize;

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

    /* MMR unlock for PCIe register access */
    *(uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK0) = KICK0;
    *(uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK2_KICK1) = KICK1;

    *(uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK0) = KICK0;
    *(uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK6_KICK1) = KICK1;

        /* MMR unlock for PCIe register access */
    *(uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0) = KICK0;
    *(uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1) = KICK1;


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
        status = Pcie_setInterfaceMode(object->handle, config->attrs->operationMode, PCIE_GEN1);

        DebugP_assert(SystemP_SUCCESS == status);
    }

    if (SystemP_SUCCESS == status)
    {
        status = Pcie_getMemSpaceReserved(handle, &resSize);
        if(config->attrs->operationMode == PCIE_RC_MODE)
        {
            /* Configure for RC mode of operation */
            status = Pcie_cfgRC(object->handle);

            DebugP_assert(SystemP_SUCCESS == status);

            status = Pcie_obTransCfg(object->handle);
        }
        else
        {
            /* Configure for EP mode of operation */
            status = Pcie_cfgEP(object->handle);

            DebugP_assert(SystemP_SUCCESS == status);
        }

        if (SystemP_SUCCESS == status)
        {
            /* Configure Outbound ATU */
            for (int i = 0; i < config->attrs->obAtuNum; i++)
            {
                memset(&regionParams, 0, sizeof(regionParams));

                regionParams.regionDir = PCIE_ATU_REGION_DIR_OUTBOUND;
                regionParams.tlpType   = config->attrs->obAtu[i].tlpType;
                regionParams.enableRegion = 1;
                regionParams.lowerBaseAddr = config->attrs->obAtu[i].lowerBaseAddr;
                regionParams.upperBaseAddr    = config->attrs->obAtu[i].upperBaseAddr;

                regionParams.regionWindowSize = config->attrs->obAtu[i].regionWindowSize;

                regionParams.lowerTargetAddr    = config->attrs->obAtu[i].lowerTargetAddr;
                regionParams.upperTargetAddr    = config->attrs->obAtu[i].upperTargetAddr;

                status = Pcie_atuRegionConfig (object->handle, PCIE_LOCATION_LOCAL,
                            (uint32_t) config->attrs->obAtu[i].regionIndex, &regionParams);

                DebugP_assert(SystemP_SUCCESS == status);

            }

            /* Configure Inbound ATU */
            for (int i = 0; i < config->attrs->ibAtuNum; i++)
            {
                if (config->attrs->operationMode == PCIE_EP_MODE)
                {
                    memset (&barCfg, 0, sizeof(Pcie_BarCfg));

                    barCfg.location = PCIE_LOCATION_LOCAL;
                    barCfg.mode     = PCIE_EP_MODE;
                    barCfg.base     = 0x70000000;
                    barCfg.prefetch = 0;
                    barCfg.type     = 0;
                    barCfg.memSpace = 0;
                    barCfg.idx      = 1;

                    status = Pcie_cfgBar(object->handle, &barCfg);

                    DebugP_assert(SystemP_SUCCESS == status);
                }

                memset(&regionParams, 0, sizeof(regionParams));

                regionParams.regionDir = PCIE_ATU_REGION_DIR_INBOUND;
                regionParams.tlpType   = config->attrs->ibAtu[i].tlpType;
                 regionParams.enableRegion = 1;
                regionParams.lowerBaseAddr    = config->attrs->ibAtu[i].lowerBaseAddr;
                regionParams.upperBaseAddr    = config->attrs->ibAtu[i].upperBaseAddr;

                regionParams.regionWindowSize = config->attrs->ibAtu[i].regionWindowSize;

                regionParams.lowerTargetAddr    = config->attrs->ibAtu[i].lowerTargetAddr;
                regionParams.upperTargetAddr    = config->attrs->ibAtu[i].upperTargetAddr;

                status = Pcie_atuRegionConfig (object->handle, PCIE_LOCATION_LOCAL,
                            (uint32_t) config->attrs->ibAtu[i].regionIndex, &regionParams);

                DebugP_assert(SystemP_SUCCESS == status);
            }

        }

    }

    if (SystemP_SUCCESS == status)
    {
        /* Configure number of lanes */
        status = Pcie_setLanes(object->handle);
    }

    if (SystemP_SUCCESS == status)
    {
        /* Enable link training */
        status = Pcie_LtssmCtrl(object->handle, TRUE);

        if (SystemP_SUCCESS == status)
        {
            /* Wait for link to be up */
            status = Pcie_waitLinkUp(object->handle);
        }

        if (SystemP_SUCCESS == status)
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
 * Read and split up the Endpoint Interrupt Request Set swRegister
 ****************************************************************************/
static int32_t Pcie_readEpIrqSetReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_EpIrqSetReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->LEGACY_IRQ_SET;
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_SET_LEGACY_IRQ_SET_0, swReg->epIrqSet);
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
static int32_t Pcie_readEpIrqClrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_EpIrqClrReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->LEGACY_IRQ_CLR;
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_CLR_LEGACY_IRQ_CLR_0, swReg->epIrqClr);
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
static int32_t Pcie_readIrqEoi(CSL_pcie_ep_coreRegs *baseAddr, Pcie_IrqEOIReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->IRQ_EOI;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IRQ_EOI_EOI, swReg->EOI);
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

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_IO_EN,                      swReg->ioSp);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_MEM_SPACE_EN,               swReg->memSp);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_BUS_MASTER_EN,              swReg->busMs);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_SPECIAL_CYCLE_OPERATION,    swReg->specCycleEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_MWI_ENABLE,                  swReg->memWrInva);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_VGA_PALETTE_SNOOP,           swReg->vgaSnoop);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_PARITY_ERR_EN,              swReg->resp);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_IDSEL_STEPPING,              swReg->idselCtrl);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_SERREN,                     swReg->serrEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_INT_EN,                     swReg->dis);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_INT_STATUS,                           swReg->stat);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_CAP_LIST,                             swReg->capList);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_FAST_66MHZ_CAP,                       swReg->c66MhzCap);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_FAST_B2B_CAP,                         swReg->fastB2B);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_MASTER_DPE,                           swReg->parError);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_DEV_SEL_TIMING,                       swReg->devSelTime);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_SIGNALED_TARGET_ABORT,                swReg->sigTgtAbort);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_RCVD_TARGET_ABORT,                    swReg->tgtAbort);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_RCVD_MASTER_ABORT,                    swReg->mstAbort);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_SIGNALED_SYS_ERR,                     swReg->sysError);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_DETECTED_PARITY_ERR,                  swReg->parity);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readStatusCmdReg */

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

        PCIE_GETBITS(val, PCIE_REV2_CLASSCODE,                                  swReg->classCode);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_REVISION_ID,  swReg->revId);
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
        val = swReg->raw = baseAddr->BAR_REG[barNum];

        /* Get the BARxA and BARxC accroding to the barNum */
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BAR_REG_BAR_MEM_IO,      swReg->memSpace);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BAR_REG_BAR_TYPE,        swReg->type);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BAR_REG_BAR_PREFETCH,    swReg->prefetch);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BAR_REG_BAR_START,       swReg->base);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readType0BarReg */

/*****************************************************************************
 * Read and split up the BAR 32bits register
 ****************************************************************************/
static int32_t Pcie_readType0Bar32bitReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_Bar32bitReg *swReg,
                                            int32_t barNum)
{
    int32_t status = SystemP_SUCCESS;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        swReg->reg32 = swReg->raw = baseAddr->BAR_REG[barNum];
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readType0Bar32bitReg */

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

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_REPORT_EN, swReg->corErRp);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_REPORT_EN, swReg->nFatalErRp);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_REPORT_EN, swReg->fatalErRp);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORT_REQ_REP_EN, swReg->reqRp);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_REL_ORDER, swReg->relaxed);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_PAYLOAD_SIZE_CS, swReg->maxPayld);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EXT_TAG_EN, swReg->xtagEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_PHANTOM_FUNC_EN, swReg->phantomEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_PM_EN, swReg->auxPwrEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_NO_SNOOP, swReg->noSnoop);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_READ_REQ_SIZE, swReg->maxSz);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_INITIATE_FLR, swReg->initFLR);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_DETECTED, swReg->corrEr);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_DETECTED, swReg->nFatalEr);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_DETECTED, swReg->fatalEr);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORTED_REQ_DETECTED, swReg->rqDet);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_DETECTED, swReg->auxPwr);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_TRANS_PENDING, swReg->tpend);
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

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_SPEED, swReg->maxLinkSpeed);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_WIDTH, swReg->maxLinkWidth);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT, swReg->asLinkPm);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L0S_EXIT_LATENCY, swReg->losExitLat);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L1_EXIT_LATENCY, swReg->l1ExitLat);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_CLOCK_POWER_MAN, swReg->clkPwrMgmt);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_SURPRISE_DOWN_ERR_REP_CAP, swReg->downErrRepCap);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_DLL_ACTIVE_REP_CAP, swReg->dllRepCap);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_LINK_BW_NOT_CAP, swReg->bwNotifyCap);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ASPM_OPT_COMPLIANCE, swReg->aspmOptComp);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_PORT_NUM, swReg->portNum);
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

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL, swReg->activeLinkPm);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RCB, swReg->rcb);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_DISABLE, swReg->linkDisable);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RETRAIN_LINK, swReg->retrainLink);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_COMMON_CLK_CONFIG, swReg->commonClkCfg);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EXTENDED_SYNCH, swReg->extSync);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EN_CLK_POWER_MAN, swReg->clkPwrMgmtEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_HW_AUTO_WIDTH_DISABLE, swReg->hwAutoWidthDis);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_INT_EN, swReg->linkBwMgmtIntEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_INT_EN, swReg->linkBwIntEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DRS_SIGNALING_CONTROL, swReg->drsSigCtrl);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_SPEED, swReg->linkSpeed);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_NEGO_LINK_WIDTH, swReg->negotiatedLinkWd);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_TRAINING, swReg->linkTraining);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_SLOT_CLK_CONFIG, swReg->slotClkCfg);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DLL_ACTIVE, swReg->dllActive);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_STATUS, swReg->linkBwMgmtStatus);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_STATUS, swReg->linkBwStatus);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_readLinkStatCtrlReg */

/*****************************************************************************
 * Read and split up the Advanced Capabilities and Control register
 ****************************************************************************/
static int32_t Pcie_readAccrReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_AccrReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->ADV_ERR_CAP_CTRL_OFF;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_EN, swReg->multHdrEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_CAP, swReg->multHdrCap);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_EN, swReg->chkEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_CAP, swReg->chkCap);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_EN, swReg->genEn);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_CAP, swReg->genCap);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_FIRST_ERR_POINTER, swReg->erPtr);
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
                    val = swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_1_OFF_OUTBOUND;

                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TYPE,                    swReg->type);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TC,                      swReg->tc);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TD,                      swReg->td);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_ATTR,                    swReg->attr);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_INCREASE_REGION_SIZE,    swReg->increaseRegionSize);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_CTRL_1_FUNC_NUM,         swReg->functionNumber);
            }
            else
            {
                    val = swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_1_OFF_INBOUND;

                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TYPE,                     swReg->type);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TC,                       swReg->tc);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TD,                       swReg->td);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_ATTR,                     swReg->attr);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_INCREASE_REGION_SIZE,     swReg->increaseRegionSize);
                    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_CTRL_1_FUNC_NUM,          swReg->functionNumber);
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

static int32_t Pcie_readPlconfIatuRegCtrl2Reg(const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow,
                                              Pcie_PlconfIatuRegCtrl2Reg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;
    /* Don't need to check NULL this is internal API */
    uint8_t regionIndex = simIatuWindow->regionIndex;

    if (regionIndex < OB_REGION_MAX)
    {
        if (simIatuWindow->regionDirection == 0U)
        {
            /* 0U == OUTBOUND */
            val = swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_2_OFF_OUTBOUND;

            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_MSG_CODE, swReg->messagecode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG, swReg->tag);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_SUBSTITUTE_EN, swReg->tagSubstEn);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_FUNC_BYPASS, swReg->functionNumberMatchEnable);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_SNP, swReg->SNP);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INHIBIT_PAYLOAD, swReg->inhibitPayload);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_HEADER_SUBSTITUTE_EN, swReg->headerSubstEn);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_CFG_SHIFT_MODE, swReg->cfgShiftMode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INVERT_MODE, swReg->invertMode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_REGION_EN, swReg->regionEnable);

            /* Set unused fields to 0 (only used by rev 0/1 hw or inbound) */
            swReg->barNumber = 0u;
            swReg->tcMatchEnable = 0u;
            swReg->tdMatchEnable = 0u;
            swReg->attrMatchEnable = 0u;
            swReg->atMatchEnable = 0u;
            swReg->virtualFunctionNumberMatchEnable = 0u;
            swReg->messageCodeMatchEnable = 0u;
            swReg->responseCode = 0u;
            swReg->fuzzyTypeMatchMode = 0u;
            swReg->singleAddrLocTransEn = 0u;
            swReg->matchMode = 0u;
        }
        else
        {
            /* INBOUND */
            val = swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_2_OFF_INBOUND;

            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE, swReg->messagecode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_BAR_NUM, swReg->barNumber);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_TYPE_MATCH_MODE, swReg->msgTypeMatchMode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TC_MATCH_EN, swReg->tcMatchEnable);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TD_MATCH_EN, swReg->tdMatchEnable);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_ATTR_MATCH_EN, swReg->attrMatchEnable);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUNC_NUM_MATCH_EN, swReg->functionNumberMatchEnable);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MATCH_EN, swReg->messageCodeMatchEnable);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_SINGLE_ADDR_LOC_TRANS_EN, swReg->singleAddrLocTransEn);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_RESPONSE_CODE, swReg->responseCode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUZZY_TYPE_MATCH_CODE, swReg->fuzzyTypeMatchMode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_CFG_SHIFT_MODE, swReg->cfgShiftMode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_INVERT_MODE, swReg->invertMode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MATCH_MODE, swReg->matchMode);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_REGION_EN, swReg->regionEnable);

            /* Set unused fields to 0 (only used by rev 0/1 hw or outbound) */
            swReg->tag = 0u;
            swReg->tagSubstEn = 0u;
            swReg->atMatchEnable = 0u;
            swReg->virtualFunctionNumberMatchEnable = 0u;
            swReg->SNP = 0u;
            swReg->inhibitPayload = 0u;
            swReg->headerSubstEn = 0u;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* pciev2_read_plconfIatuRegCtrl2_reg */

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
        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_BASE_ADDR_OFF_OUTBOUND;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_RW, swReg->iatuRegLowerBase);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_HW, swReg->zero);
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_BASE_ADDR_OFF_INBOUND;
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_RW, swReg->iatuRegLowerBase);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_HW, swReg->zero);
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
        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                val = swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_BASE_ADDR_OFF_OUTBOUND;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_UPPER_BASE_RW, swReg->iatuRegUpperBase);
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                val = swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_BASE_ADDR_OFF_INBOUND;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_UPPER_BASE_RW, swReg->iatuRegUpperBase);
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
                val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_TARGET_ADDR_OFF_OUTBOUND;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_LWR_TARGET_RW_OUTBOUND, swReg->iatuRegLowerTarget);
                swReg->zero = swReg->iatuRegLowerTarget & 0xffffu;
                swReg->iatuRegLowerTarget >>= 16;
            }
            else
            {
                /* INBOUND */
                val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_TARGET_ADDR_OFF_INBOUND;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_RW, swReg->iatuRegLowerTarget);
                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_HW, swReg->zero);
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
                val = swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_UPPER_TARGET_RW, swReg->iatuRegUpperTarget);
            }
            else
            {
                /* INBOUND */
                val = swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_TARGET_ADDR_OFF_INBOUND;

                PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_UPPER_TARGET_RW, swReg->iatuRegUpperTarget);
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
} /* Pcie_readPlconfIatuRegUpperTargetReg */

/* Read MSI capabilities Register */
/*****************************************************************************
 * Read and split up the Message Signaled Interrupt Capability register
 ****************************************************************************/
static int32_t Pcie_readMsiCapReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiCapReg *swReg)
{
    uint32_t val = swReg->raw = baseAddr->PCI_MSI_CAP_ID_NEXT_CTRL_REG;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_ID,             swReg->capId);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_NEXT_OFFSET,    swReg->nextCap);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_ENABLE,             swReg->msiEn);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_CAP,   swReg->multMsgCap);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_EN,    swReg->multMsgEn);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_64_BIT_ADDR_CAP,    swReg->en64bit);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_PVM_SUPPORT,            swReg->pvmEn);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_CAP,       swReg->extDataCap);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_EN,        swReg->extDataEn);

    return SystemP_SUCCESS;
} /* Pcie_readMsiCapReg */

/*****************************************************************************
 * Read and split up the Address of MSI write TLP req lower 32 bits register
 ****************************************************************************/
static int32_t Pcie_readMsiLo32Reg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiLo32Reg *swReg)
{
    uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_04H_REG;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_04H_REG_PCI_MSI_CAP_OFF_04H, swReg->addr);

    return SystemP_SUCCESS;
} /* Pcie_readMsiLo32Reg */

/*****************************************************************************
 * Read and split up the Address of MSI write TLP req upper 32 bits register
 ****************************************************************************/
static int32_t Pcie_readMsiUp32Reg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiUp32Reg *swReg)
{
    uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_08H_REG;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_08H, swReg->addr);

    return SystemP_SUCCESS;
} /* Pcie_readMsiUp32Reg */

/*****************************************************************************
 * Read and split up the Data of MSI write TLP req register
 ****************************************************************************/
static int32_t Pcie_readMsiDataReg (const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiDataReg *swReg)
{
    uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_0CH_REG;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0CH, swReg->data);

    return SystemP_SUCCESS;
} /* Pcie_readMsiDataReg */

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

static int32_t Pcie_readVendorDevId(volatile const uint32_t *hwReg_VENDOR_ID_DEV_ID,
                                    Pcie_VndDevIdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((hwReg_VENDOR_ID_DEV_ID != NULL) && (swReg != NULL))
    {
        val = swReg->raw = *hwReg_VENDOR_ID_DEV_ID;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_ID_VENDOR_ID_REG_PCI_TYPE0_VENDOR_ID, swReg->vndId);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_ID_VENDOR_ID_REG_PCI_TYPE0_DEVICE_ID, swReg->devId);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Pcie_readRstCmdReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_RstCmdReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        val = swReg->raw = baseAddr->RSTCMD;

        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_RSTCMD_FLR_PF_ACTIVE, swReg->flrPfActive);
        PCIE_GETBITS(val, CSL_PCIE_EP_CORE_RSTCMD_INIT_RST, swReg->initRst);

        /* Set unused fields to 0 (only used by rev 0/1 hw) */
        swReg->flush = 0u;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

/*****************************************************************************
 * Read and split up the PTM Config Command swRegister
 ****************************************************************************/
static int32_t Pcie_readPtmCfgReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PtmCfgReg *swReg)
{
    uint32_t val = swReg->raw = baseAddr->PTMCFG;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PTMCFG_PTM_CLK_SEL, swReg->ptmClkSel);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PTMCFG_PTM_CONTEXT_VALID, swReg->ptmContextValid);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PTMCFG_PTM_MANUAL_UPDATE, swReg->ptmManualUpdate);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PTMCFG_PTM_AUTO_UPDATE, swReg->ptmAutoUpdate);

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up the Power Management Command swRegister
 ****************************************************************************/
static int32_t Pcie_readPmCmdReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PmCmdReg *swReg)
{
    uint32_t val = swReg->raw = baseAddr->PMCMD;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMCMD_PM_XMT_TURNOFF, swReg->turnOff);
    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMCMD_PM_XMT_PE, swReg->pme);

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up the End of Interrupt swRegister
 ****************************************************************************/
static int32_t Pcie_readIrqEOIReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_IrqEOIReg *swReg)
{
    uint32_t val = swReg->raw = baseAddr->IRQ_EOI;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IRQ_EOI_EOI, swReg->EOI);

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up the MSI Interrupt IRQ swRegister
 ****************************************************************************/
static int32_t Pcie_readMsiIrqReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiIrqReg *swReg)
{
    uint32_t val = swReg->raw = baseAddr->MMR_IRQ;

    PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MMR_IRQ_MMR_IRQ, swReg->msiIrq);

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up a General Purpose swRegister [0-3]
 ****************************************************************************/
static int32_t Pcie_ReadGenPurposeReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_GenPurposeReg *swReg, int_fast32_t swRegNum)
{
  /* swRegNum generated internally no need for bounds check */
  swReg->raw = swReg->genPurpose = baseAddr->GPR[swRegNum];

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up a MSI Raw Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_ReadMsiIrqStatusRaw(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiIrqStatusRawReg *swReg, int_fast32_t swRegNum)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_STATUS_RAW;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MMR_IRQ_STATUS_RAW_MMR_IRQ_STATUS_RAW, swReg->msiRawStatus);

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up a MSI Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_readMsiIrqStatusReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiIrqStatusReg *swReg, int_fast32_t swRegNum)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_STATUS;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MMR_IRQ_STATUS_MMR_IRQ_STATUS, swReg->msiIrqStatus);

  return SystemP_SUCCESS;
} /* pciev2_read_msiIrqStatus_reg */

/*****************************************************************************
 * Read and split up a MSI Interrupt Enable Set swRegister
 ****************************************************************************/
int32_t Pcie_readMsiIrqEnableSetReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiIrqEnableSetReg *swReg, int_fast32_t swRegNum)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_ENABLE_SET;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MMR_IRQ_ENABLE_SET_MMR_IRQ_EN_SET, swReg->msiIrqEnSet);

  return SystemP_SUCCESS;
} /* pciev2_read_msiIrqEnableSet_reg */

/*****************************************************************************
 * Read and split up the MSI Interrupt Enable Clear swRegister
 ****************************************************************************/
static int32_t Pcie_readMsiIrqEnableClrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiIrqEnableClrReg *swReg, int_fast32_t swRegNum)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_ENABLE_CLR;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MMR_IRQ_ENABLE_CLR_MMR_IRQ_EN_CLR, swReg->msiIrqEnClr);

  return SystemP_SUCCESS;
} /* pciev2_read_msiIrqEnableClr_reg */

/*****************************************************************************
 * Read and split up a Legacy Raw Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_readLegacyIrqStatusRawReg(const CSL_pcie_ep_coreRegs  *baseAddr, Pcie_LegacyIrqStatusRawReg *swReg, int_fast32_t swRegNum)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_STATUS_RAW;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_RAW_INT_RAW, swReg->legacyRawStatus);

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up a Raw ERR Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_readErrIrqStatusRawReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_ErrIrqStatusRawReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_STATUS_RAW;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_AER_RAW,        swReg->errAer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_CORR_RAW,       swReg->errCorr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_NONFATAL_RAW,   swReg->errNonFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_FATAL_RAW,      swReg->errFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_SYS_RAW,        swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return SystemP_SUCCESS;
} /* pciev2_read_errIrqStatusRaw_reg */

/*****************************************************************************
 * Read and split up a Err Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_readRrrIrqStatusReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_ErrIrqStatusReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_STATUS;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_AER, swReg->errAer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_CORR, swReg->errCorr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_NONFATAL, swReg->errNonFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_FATAL, swReg->errFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_SYS, swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up a Err Interrupt Enable Set swRegister
 ****************************************************************************/
static int32_t Pcie_readErrIrqEnableSetReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_ErrIrqEnableSetReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_ENABLE_SET;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_AER_EN_SET, swReg->errAer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_CORR_EN_SET, swReg->errCorr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_NONFATAL_EN_SET, swReg->errNonFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_FATAL_EN_SET, swReg->errFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_SYS_EN_SET, swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up the Err Interrupt Enable Clear swRegister
 ****************************************************************************/
static int32_t Pcie_readErrIrqEnableClrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_ErrIrqEnableClrReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_ENABLE_CLR;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_AER_EN_CLR, swReg->errAer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_CORR_EN_CLR, swReg->errCorr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL_EN_CLR, swReg->errNonFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_FATAL_EN_CLR, swReg->errFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_SYS_EN_CLR, swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up a Raw Power Management and Reset Status swRegister
 ****************************************************************************/
static int32_t Pcie_readPmRstIrqStatusRawReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PmRstIrqStatusRawReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PMRST_IRQ_STATUS_RAW;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_LNK_RST_REQ_RAW, swReg->linkRstReq);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_PME_RAW, swReg->pmPme);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_TO_ACK_RAW, swReg->pmToAck);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_TURNOFF_RAW, swReg->pmTurnoff);

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up a Power Management and Reset Status swRegister
 ****************************************************************************/
static int32_t Pcie_readPmRstIrqStatusReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PmRstIrqStatusReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PMRST_IRQ_STATUS;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_LNK_RST_REQ, swReg->linkRstReq);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_PME, swReg->pmPme);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_TO_ACK, swReg->pmToAck);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_TURNOFF, swReg->pmTurnoff);

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up a Power Management and Reset Enable Set swRegister
 ****************************************************************************/
static int32_t Pcie_readPmRstIrqEnableSetReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PmRstIrqEnableSetReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PMRST_IRQ_ENABLE_SET;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_LNK_RST_REQ_EN_SET, swReg->linkRstReq);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_PME_EN_SET, swReg->pmPme);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_TO_ACK_EN_SET, swReg->pmToAck);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_TURNOFF_EN_SET, swReg->pmTurnoff);

  return SystemP_SUCCESS;
}

static int32_t Pcie_readPmRstIrqEnableClrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PmRstIrqEnableClrReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PMRST_IRQ_ENABLE_CLR;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_LNK_RST_REQ_EN_CLR, swReg->linkRstReq);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_PME_EN_CLR, swReg->pmPme);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_TO_ACK_EN_CLR, swReg->pmToAck);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_TURNOFF_EN_CLR, swReg->pmTurnoff);

  return SystemP_SUCCESS;
}

static int32_t Pcie_readPtmIrqStatusRawReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PtmIrqStatusRawReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PTM_IRQ_STATUS_RAW;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PTM_IRQ_STATUS_RAW_PTM_CLK_UPDATED_RAW, swReg->ptmClkUpdated);

  return SystemP_SUCCESS;
} /* pciev2_read_ptmIrqStatusRaw_reg */

/*****************************************************************************
 * Read and split up a Precision Time Measurement Status swRegister
 ****************************************************************************/
static int32_t Pcie_readPtmIrqStatusReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PtmIrqStatusReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PTM_IRQ_STATUS;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PTM_IRQ_STATUS_PTM_CLK_UPDATED, swReg->ptmClkUpdated);

  return SystemP_SUCCESS;
} /* pciev2_read_ptmIrqStatus_reg */

/*****************************************************************************
 * Read and split up a Precision Time Measurement Enable Set swRegister
 ****************************************************************************/
static int32_t Pcie_readPtmIrqEnableSetReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PtmIrqEnableSetReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PTM_IRQ_ENABLE_SET;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PTM_IRQ_ENABLE_SET_PTM_CLK_UPDATED_EN_SET, swReg->ptmClkUpdated);

  return SystemP_SUCCESS;
} /* pciev2_read_ptmIrqEnableSet_reg */

/*****************************************************************************
 * Read and split up the Precision Time Measurement Enable Clear swRegister
 ****************************************************************************/
static int32_t Pcie_readPtmIrqEnableClrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PtmIrqEnableClrReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PTM_IRQ_ENABLE_CLR;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PTM_IRQ_ENABLE_CLR_PTM_CLK_UPDATED_EN_CLR, swReg->ptmClkUpdated);

  return SystemP_SUCCESS;
} /* pciev2_read_ptmIrqEnableClr_reg */

/*****************************************************************************
 * Read and split up the BIST and Header register
 ****************************************************************************/
static int32_t Pcie_readBistReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_BistReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE, swReg->cacheLnSize);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER, swReg->latTmr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_HEADER_TYPE, swReg->hdrType);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_MULTI_FUNC, swReg->mulfunDev);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_BIST, swReg->bist);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->bistCap = 0;
  swReg->startBist = 0;
  swReg->compCode = 0;

  return SystemP_SUCCESS;
} /* pciev2_read_bist_reg */

static int32_t Pcie_readSubIdReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_SubIdReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG_SUBSYS_VENDOR_ID, swReg->subVndId);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG_SUBSYS_DEV_ID, swReg->subId);

  return SystemP_SUCCESS;
} /* Pcie_read_subId_reg */

/*****************************************************************************
 * Read and split up the Cardbus CIS Pointer register
 ****************************************************************************/
static int32_t Pcie_readCardbusCisPointerReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_CardbusCisPointerReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->CARDBUS_CIS_PTR_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CARDBUS_CIS_PTR_REG_CARDBUS_CIS_POINTER, swReg->cisPointer);

  return SystemP_SUCCESS;
} /* Pcie_read_cardbusCisPointer_reg */

/*****************************************************************************
 * Read and split up the Expansion ROM Base Address register
 ****************************************************************************/
static int32_t Pcie_readExpRomReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_ExpRomReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->EXP_ROM_BASE_ADDR_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EXP_ROM_BASE_ADDR_REG_ROM_BAR_ENABLE, swReg->enable);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_EXP_ROM_BASE_ADDR_REG_EXP_ROM_BASE_ADDRESS, swReg->expRomAddr);

  return SystemP_SUCCESS;
} /* Pcie_read_expRom_reg */

/*****************************************************************************
 * Read and split up the Capabilities Pointer register
 ****************************************************************************/
static int32_t Pcie_readCapPtrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_CapPtrReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PCI_CAP_PTR_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCI_CAP_PTR_REG_CAP_POINTER, swReg->ptr);

  return SystemP_SUCCESS;
} /* Pcie_read_capPtr_reg */

/*****************************************************************************
 * Read and split up the Interrupt Pin register
 ****************************************************************************/
static int32_t Pcie_readIntPinReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_IntPinReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG_INT_LINE, swReg->intLine);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG_INT_PIN, swReg->intPin);

  return SystemP_SUCCESS;
} /* Pcie_read_intPin_reg */

/*****************************************************************************
 * Read and split up the BIST and Header register
 ****************************************************************************/
static int32_t Pcie_readType1BistHeaderReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1BistHeaderReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE, swReg->cacheLnSize);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER, swReg->latTmr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_HEADER_TYPE, swReg->hdrType);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_MULTI_FUNC, swReg->mulFunDev);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_BIST, swReg->bist);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->bistCap = 0u;
  swReg->startBist = 0u;
  swReg->compCode = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_bist_reg */

static int32_t Pcie_readType1BusNumReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1BusNumReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_PRIM_BUS, swReg->priBusNum);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_BUS, swReg->secBusNum);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SUB_BUS, swReg->subBusNum);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_LAT_TIMER, swReg->secLatTmr);

  return SystemP_SUCCESS;
} /* Pcie_read_type1BusNum_reg */

/*****************************************************************************
 * Read and split up the Secondary Status and IO Base/Limit Register
 ****************************************************************************/
static int32_t Pcie_readType1SecStatReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1SecStatReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->SEC_STAT_IO_LIMIT_IO_BASE_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE, swReg->IOBaseAddr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_BASE, swReg->IOBase);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_BIT8, swReg->IOLimitAddr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_LIMIT, swReg->IOLimit);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_MDPE, swReg->mstDPErr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_SIG_TRGT_ABRT, swReg->txTgtAbort);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_TRGT_ABRT, swReg->rxTgtAbort);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_MSTR_ABRT, swReg->rxMstAbort);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_SYS_ERR, swReg->rxSysError);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_DPE, swReg->dtctPError);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->devselTiming = 0u;
  swReg->c66mhzCapa = 0u;
  swReg->fastB2bCap = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_type1SecStat_reg */

/*****************************************************************************
 * Read and split up the Memory Limit and Base register
 ****************************************************************************/
static int32_t Pcie_readType1MemspaceReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1MemspaceReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->MEM_LIMIT_MEM_BASE_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE, swReg->base);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT, swReg->limit);

  return SystemP_SUCCESS;
} /* Pcie_read_type1Memspace_reg */

/*****************************************************************************
 * Read and split up the Prefetchable Memory Limit and Base register
 ****************************************************************************/
static int32_t Pcie_readPrefMemReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_PrefMemReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PREF_MEM_LIMIT_PREF_MEM_BASE_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_DECODE, swReg->baseAddr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_BASE, swReg->base);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_DECODE, swReg->limitAddr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT, swReg->limit);

  return SystemP_SUCCESS;
} /* Pcie_read_prefMem_reg */

/*****************************************************************************
 * Read and split up the Prefetchable Memory Base Upper register
 ****************************************************************************/
static int32_t Pcie_readPrefBaseUpperReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_PrefBaseUpperReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PREF_BASE_UPPER_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_PREF_BASE_UPPER_REG_PREF_MEM_BASE_UPPER, swReg->base);

  return SystemP_SUCCESS;
} /* Pcie_read_prefBaseUp_reg */

/*****************************************************************************
 * Read and split up the Prefetchable Memory Limit Upper register
 ****************************************************************************/
static int32_t Pcie_readPrefLimitUpperReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_PrefLimitUpperReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PREF_LIMIT_UPPER_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_PREF_LIMIT_UPPER_REG_PREF_MEM_LIMIT_UPPER, swReg->limit);

  return SystemP_SUCCESS;
} /* Pcie_read_prefLimitUp_reg */

/*****************************************************************************
 * Read and split up the IO Base and Limit Upper 16 bits register
 ****************************************************************************/
static int32_t Pcie_readType1IOSpaceReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1IOSpaceReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->IO_LIMIT_UPPER_IO_BASE_UPPER_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_BASE_UPPER, swReg->IOBase);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_LIMIT_UPPER, swReg->IOLimit);

  return SystemP_SUCCESS;
} /* Pcie_read_type1IOSpace_reg */

/*****************************************************************************
 * Read and split up the Capabilities Pointer register
 ****************************************************************************/
static int32_t Pcie_readType1CapPtrReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1CapPtrReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->TYPE1_CAP_PTR_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_CAP_POINTER, swReg->capPtr);

  return SystemP_SUCCESS;
} /* Pcie_read_type1CapPtr_reg */

/*****************************************************************************
 * Read and split up the Expansion ROM Base Address register
 ****************************************************************************/
static int32_t Pcie_readType1ExpnsnRomReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1ExpnsnRomReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->TYPE1_EXP_ROM_BASE_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_ROM_BAR_ENABLE, swReg->expRomEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_EXP_ROM_BASE_ADDRESS, swReg->expRomBaseAddr);

  return SystemP_SUCCESS;
} /* Pcie_read_type1ExpnsnRom_reg */

/* power management capabilities*/
/*****************************************************************************
 * Read and split up the Power Management Capability register
 ****************************************************************************/
static int32_t Pcie_readPmCapReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PMCapReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->CAP_ID_NXT_PTR_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_CAP_ID, swReg->pmCapID);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_NEXT_POINTER, swReg->pmNextPtr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_SPEC_VER, swReg->pmeSpecVer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PME_CLK, swReg->pmeClk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_DSI, swReg->dsiN);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_AUX_CURR, swReg->auxCurrN);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_D1_SUPPORT, swReg->d1SuppN);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_D2_SUPPORT, swReg->d2SuppN);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PME_SUPPORT, swReg->pmeSuppN);

  return SystemP_SUCCESS;
} /* Pcie_read_pmCap_reg */

/*****************************************************************************
 * Read and split up the Power Management Control and Status register
 ****************************************************************************/
static int32_t Pcie_readPmCapCtlStatReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PMCapCtlStatReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->CON_STATUS_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_POWER_STATE, swReg->pwrState);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_NO_SOFT_RST, swReg->noSoftRst);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_PME_ENABLE, swReg->pmeEn);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_SELECT, swReg->dataSelect);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_SCALE, swReg->dataScale);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_PME_STATUS, swReg->pmeStatus);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_B2_B3_SUPPORT, swReg->b2b3Support);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_BUS_PWR_CLK_CON_EN, swReg->clkCtrlEn);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_REG_ADD_INFO, swReg->dataReg);

  return SystemP_SUCCESS;
} /* Pcie_read_pmCapCtlStat_reg */

/*****************************************************************************
 * Read and split up the Bridge Control and Interrupt register
 ****************************************************************************/
static int32_t Pcie_readType1BridgeIntReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1BridgeIntReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->BRIDGE_CTRL_INT_PIN_INT_LINE_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_LINE, swReg->intLine);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_PIN, swReg->intPin);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_PERE, swReg->pErrRespEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SERR_EN, swReg->serrEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_ISA_EN, swReg->isaEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_EN, swReg->vgaEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_16B_DEC, swReg->vgaDecode);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_MSTR_ABORT_MODE, swReg->mstAbortMode);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SBR, swReg->secBusRst);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->b2bEn = 0u;
  swReg->priTimer = 0u;
  swReg->secTimer = 0u;
  swReg->timerStatus = 0u;
  swReg->serrEnStatus = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_type1BridgeInt_reg */

/*****************************************************************************
 * Read and split up the Data of MSI CAP OFF 10H Reg
 ****************************************************************************/
static int32_t Pcie_readMsiCapOff10HReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiCapOff10HReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_10H_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_10H_REG_PCI_MSI_CAP_OFF_10H, swReg->data);

  return SystemP_SUCCESS;
} /* Pcie_read_msiCapOff10H_reg */

/*****************************************************************************
 * Read and split up the Data of MSI CAP OFF 14H Reg
 ****************************************************************************/
static int32_t Pcie_readMsiCapOff14HReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiCapOff14HReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_14H_REG;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_14H_REG_PCI_MSI_CAP_OFF_14H, swReg->data);

  return SystemP_SUCCESS;
} /* Pcie_read_msiCapOff14H_reg */

/*****************************************************************************
 * Read and split up the PCIE Capabilities register
 ****************************************************************************/
static int32_t Pcie_readPciesCapReg(volatile const uint32_t *hwReg_PCIE_CAP, Pcie_PciesCapReg *swReg)
{
  uint32_t val = swReg->raw = *hwReg_PCIE_CAP;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_ID, swReg->capId);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_NEXT_PTR, swReg->nextCap);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_REG, swReg->pcieCap);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_DEV_PORT_TYPE, swReg->dportType);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_SLOT_IMP, swReg->sltImplN);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_INT_MSG_NUM, swReg->intMsg);

  return SystemP_SUCCESS;
} /* Pcie_read_pciesCap_reg */

/*****************************************************************************
 * Read and split up the Device Capabilities register
 ****************************************************************************/
static int32_t Pcie_readDeviceCapReg(volatile const uint32_t *hwReg_DEV_CAP, Pcie_DeviceCapReg *swReg)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAP;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_MAX_PAYLOAD_SIZE, swReg->maxPayldSz);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_PHANTOM_FUNC_SUPPORT, swReg->phantomFld);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EXT_TAG_SUPP, swReg->extTagFld);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EP_L0S_ACCPT_LATENCY, swReg->l0Latency);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EP_L1_ACCPT_LATENCY, swReg->l1Latency);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_ROLE_BASED_ERR_REPORT, swReg->errRpt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_CAP_SLOT_PWR_LMT_VALUE, swReg->pwrLimitValue);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_CAP_SLOT_PWR_LMT_SCALE, swReg->pwrLimitScale);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_FLR_CAP, swReg->flrEn);

  return SystemP_SUCCESS;
} /* Pcie_read_deviceCap_reg */

/*****************************************************************************
 * Read and split up the Slot Capabilities register
 ****************************************************************************/
static int32_t Pcie_readSlotCapReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_SlotCapReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->SLOT_CAPABILITIES_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_BUTTON, swReg->attnButton);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_CONTROLLER, swReg->pwrCtl);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_MRL_SENSOR, swReg->mrlSensor);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR, swReg->attnInd);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_INDICATOR, swReg->pwrInd);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_SURPRISE, swReg->hpSurprise);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_CAPABLE, swReg->hpCap);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_VALUE, swReg->pwrLmtValue);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_SCALE, swReg->pwrLmtScale);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ELECTROMECH_INTERLOCK, swReg->emlPresent);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_NO_CMD_CPL_SUPPORT, swReg->cmdCompSupp);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_PHY_SLOT_NUM, swReg->slotNum);

  return SystemP_SUCCESS;
} /* Pcie_read_slotCap_reg */

/*****************************************************************************
 * Read and split up the Slot Status and Control register
 ****************************************************************************/
static int32_t Pcie_readSlotStatCtrlReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_SlotStatCtrlReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->SLOT_CONTROL_SLOT_STATUS;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_EN, swReg->attnButtEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_EN, swReg->pwrFltDetEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_EN, swReg->mrlChgEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_CHANGE_EN, swReg->prsDetChgEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPL_INT_EN, swReg->cmdCmpIntEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_HOT_PLUG_INT_EN, swReg->hpIntEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_INDICATOR_CTRL, swReg->attnIndCtl);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_INDICATOR_CTRL, swReg->pmIndCtl);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_CONTROLLER_CTRL, swReg->pmCtl);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_CTRL, swReg->emLockCtl);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_EN, swReg->dllChgEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED, swReg->attnPressed);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED, swReg->pwrFault);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED, swReg->mrlChange);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECTED_CHANGED, swReg->presenceChg);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPLD, swReg->cmdComplete);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_STATE, swReg->mrlState);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_STATE, swReg->presenceDet);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_STATUS, swReg->emLock);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED, swReg->dllState);

  return SystemP_SUCCESS;
} /* Pcie_read_slotStatCtrl_reg */

/*****************************************************************************
 * Read and split up the Root Control and Capabilities register
 ****************************************************************************/
static int32_t Pcie_readRootCtrlCapReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_RootCtrlCapReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ROOT_CONTROL_ROOT_CAPABILITIES_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN, swReg->serrEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN, swReg->serrNFatalErr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN, swReg->serrFatalErr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_PME_INT_EN, swReg->pmeIntEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_EN, swReg->crsSwEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY, swReg->crsSw);

  return SystemP_SUCCESS;
} /* Pcie_read_rootCtrlCap_reg */

/*****************************************************************************
 * Read and split up the Root Status and Control register
 ****************************************************************************/
static int32_t Pcie_readRootStatusReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_RootStatusReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ROOT_STATUS_REG;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_REQ_ID, swReg->pmeReqID);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_STATUS, swReg->pmeStatus);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_PENDING, swReg->pmePend);

  return SystemP_SUCCESS;
} /* Pcie_read_rootStatus_reg */

/*****************************************************************************
 * Read and split up the Device Capabilities 2 register
 ****************************************************************************/
static int32_t Pcie_readDevCap2Reg(volatile const uint32_t *hwReg_DEV_CAP_2, Pcie_DevCap2Reg *swReg)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAP_2;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_RANGE, swReg->cmplToEn);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_SUPPORT, swReg->cmplToDisSupp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT, swReg->ariFwdSp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ATOMIC_ROUTING_SUPP, swReg->aorSp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_32_ATOMIC_CPL_SUPP, swReg->aoc32Sp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_64_ATOMIC_CPL_SUPP, swReg->aoc64Sp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_128_CAS_CPL_SUPP, swReg->casc128Sp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_NO_RO_EN_PR2PR_PAR, swReg->noRoPR);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_LTR_SUPP, swReg->ltrSupp);
  PCIE_GETBITS(val, PCIE_REV2_TPH_CMPLT_SUPPORT, swReg->tphcSp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_LN_SYS_CLS, swReg->lnSysCls);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_COMP_SUPPORT, swReg->tag10bitCompSupp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_REQ_SUPPORT, swReg->tag10bitReqSupp);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_OBFF_SUPPORT, swReg->obffSupp);

  return SystemP_SUCCESS;
} /* Pcie_read_devCap2_reg */

/*****************************************************************************
 * Read and split up the Device Status and Control Register 2 register
 ****************************************************************************/
static int32_t Pcie_readDevStatCtrl2Reg(volatile const uint32_t *hwReg_DEV_CAS_2, Pcie_DevStatCtrl2Reg *swReg)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAS_2;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_VALUE, swReg->cmplTo);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE, swReg->cmplToDis);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_CS, swReg->ariFwdSp);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->aopReqEn = 0u;
  swReg->aopEgBlk = 0u;
  swReg->idoReqEn = 0u;
  swReg->idoCplEn = 0u;
  swReg->ltrEn = 0u;
  swReg->obffEn = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_devStatCtrl2_reg */

/*****************************************************************************
 * Read and split up the Link Capabilities 2 register
 ****************************************************************************/
static int32_t Pcie_readLinkCap2Reg(volatile const uint32_t *hwReg_LNK_CAP_2, Pcie_LnkCap2Reg *swReg)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAP_2;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_SUPPORT_LINK_SPEED_VECTOR, swReg->spLsVec);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_CROSS_LINK_SUPPORT, swReg->crosslinkSp);

  return SystemP_SUCCESS;
} /* Pcie_read_linkCap2_reg */

/*****************************************************************************
 * Read and split up the Link Control 2 register
 ****************************************************************************/
static int32_t Pcie_readLinkCtrl2Reg(volatile const uint32_t *hwReg_LNK_CAS_2, Pcie_LinkCtrl2Reg *swReg)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAS_2;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TARGET_LINK_SPEED, swReg->tgtSpeed);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_COMPLIANCE, swReg->entrCompl);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_HW_AUTO_SPEED_DISABLE, swReg->hwAutoSpeedDis);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_SEL_DEEMPHASIS, swReg->selDeemph);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TX_MARGIN, swReg->txMargin);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_MODIFIED_COMPLIANCE, swReg->entrModCompl);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_SOS, swReg->cmplSos);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_PRESET, swReg->complPrstDeemph);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_CURR_DEEMPHASIS, swReg->deEmph);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL, swReg->eqComplete);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P1, swReg->eqPh1);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P2, swReg->eqPh2);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P3, swReg->eqPh3);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_LINK_EQ_REQ, swReg->linkEqReq);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DOWNSTREAM_COMPO_PRESENCE, swReg->downCompPres);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DRS_MESSAGE_RECEIVED, swReg->drsMsgRecv);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->pollDeemph = 0; /* Note: low bit of complPrstDeeph */

  return SystemP_SUCCESS;
} /* Pcie_read_linkCtrl2_reg */

/*****************************************************************************
 * Read and split up the PCIE Extended Capabilities Header register
 ****************************************************************************/
static int32_t Pcie_readExtCapReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_ExtCapReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->AER_EXT_CAP_HDR_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AER_EXT_CAP_HDR_OFF_NEXT_OFFSET, swReg->nextCap);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AER_EXT_CAP_HDR_OFF_CAP_VERSION, swReg->extCapVer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AER_EXT_CAP_HDR_OFF_CAP_ID, swReg->extCapID);

  return SystemP_SUCCESS;
} /* Pcie_read_extCap_reg */

/*****************************************************************************
 * Read and split up the Uncorrectable Error Status register
 ****************************************************************************/
static int32_t Pcie_readUncErrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_UncErrReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->UNCORR_ERR_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_TLP_PRFX_BLOCKED_ERR_STATUS, swReg->tlpPrfxBlockedErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_INTERNAL_ERR_STATUS, swReg->intErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_UNSUPPORTED_REQ_ERR_STATUS, swReg->urErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_ECRC_ERR_STATUS, swReg->ecrcErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_MALF_TLP_ERR_STATUS, swReg->mtlpErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_REC_OVERFLOW_ERR_STATUS, swReg->rcvrOfSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_UNEXP_CMPLT_ERR_STATUS, swReg->ucmpSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_ABORT_ERR_STATUS, swReg->cmplAbrtSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_TIMEOUT_ERR_STATUS, swReg->cmplTmotSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_FC_PROTOCOL_ERR_STATUS, swReg->fcpErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_POIS_TLP_ERR_STATUS, swReg->psndTlpSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_SURPRISE_DOWN_ERR_STATUS, swReg->srpsDnSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_DL_PROTOCOL_ERR_STATUS, swReg->dlpErrSt);

  return SystemP_SUCCESS;
} /* Pcie_read_uncErr_reg */

/*****************************************************************************
 * Read and split up the Uncorrectable Error Mask register
 ****************************************************************************/
static int32_t Pcie_readUncErrMaskReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_UncErrMaskReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->UNCORR_ERR_MASK_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_TLP_PRFX_BLOCKED_ERR_MASK, swReg->tlpPrfxBlockedErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_ATOMIC_EGRESS_BLOCKED_ERR_MASK, swReg->atomicEgressBlockedErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_INTERNAL_ERR_MASK, swReg->intErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_UNSUPPORTED_REQ_ERR_MASK, swReg->urErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_ECRC_ERR_MASK, swReg->ecrcErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_MALF_TLP_ERR_MASK, swReg->mtlpErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_REC_OVERFLOW_ERR_MASK, swReg->rcvrOfMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_UNEXP_CMPLT_ERR_MASK, swReg->ucmpMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_CMPLT_ABORT_ERR_MASK, swReg->cmplAbrtMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_CMPLT_TIMEOUT_ERR_MASK, swReg->cmplTmotMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_FC_PROTOCOL_ERR_MASK, swReg->fcpErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_POIS_TLP_ERR_MASK, swReg->psndTlpMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_SURPRISE_DOWN_ERR_MASK, swReg->srpsDnMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_DL_PROTOCOL_ERR_MASK, swReg->dlpErrMsk);

  return SystemP_SUCCESS;
} /* Pcie_read_uncErrMask_reg */

/*****************************************************************************
 * Read and split up the Uncorrectable Error Severity register
 ****************************************************************************/
static int32_t Pcie_readUncErrSvrtyReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_UncErrSvrtyReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->UNCORR_ERR_SEV_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_TLP_PRFX_BLOCKED_ERR_SEVERITY, swReg->tlpPrfxBlockedErrSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_ATOMIC_EGRESS_BLOCKED_ERR_SEVERITY, swReg->atomicEgressBlockedErrSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_INTERNAL_ERR_SEVERITY, swReg->intErrSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_UNSUPPORTED_REQ_ERR_SEVERITY, swReg->urErrSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_ECRC_ERR_SEVERITY, swReg->ecrcErrSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_MALF_TLP_ERR_SEVERITY, swReg->mtlpErrSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_REC_OVERFLOW_ERR_SEVERITY, swReg->rcvrOfSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_UNEXP_CMPLT_ERR_SEVERITY, swReg->ucmpSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_CMPLT_ABORT_ERR_SEVERITY, swReg->cmplAbrtSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_CMPLT_TIMEOUT_ERR_SEVERITY, swReg->cmplTmotSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_FC_PROTOCOL_ERR_SEVERITY, swReg->fcpErrSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_POIS_TLP_ERR_SEVERITY, swReg->psndTlpSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_SURPRISE_DOWN_ERR_SVRITY, swReg->srpsDnSvrty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_DL_PROTOCOL_ERR_SEVERITY, swReg->dlpErrSvrty);

  return SystemP_SUCCESS;
} /* Pcie_read_uncErrSvrty_reg */

/*****************************************************************************
 * Read and split up the Correctable Error Status register
 ****************************************************************************/
static int32_t Pcie_readCorErrReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_CorErrReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->CORR_ERR_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_HEADER_LOG_OVERFLOW_STATUS, swReg->hdrLogOverflowErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_CORRECTED_INT_ERR_STATUS, swReg->corrIntErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_ADVISORY_NON_FATAL_ERR_STATUS, swReg->advNFErrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_RPL_TIMER_TIMEOUT_STATUS, swReg->rplyTmrSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_REPLAY_NO_ROLEOVER_STATUS, swReg->rpltRoSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_BAD_DLLP_STATUS, swReg->badDllpSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_BAD_TLP_STATUS, swReg->badTlpSt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_RX_ERR_STATUS, swReg->rcvrErrSt);

  return SystemP_SUCCESS;
} /* Pcie_read_corErr_reg */

/*****************************************************************************
 * Read and split up the Correctable Error Mask register
 ****************************************************************************/
static int32_t Pcie_readCorErrMaskReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_CorErrMaskReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->CORR_ERR_MASK_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_HEADER_LOG_OVERFLOW_MASK, swReg->hdrLogOverflowErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_CORRECTED_INT_ERR_MASK, swReg->corrIntErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_ADVISORY_NON_FATAL_ERR_MASK, swReg->advNFErrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_RPL_TIMER_TIMEOUT_MASK, swReg->rplyTmrMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_REPLAY_NO_ROLEOVER_MASK, swReg->rpltRoMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_BAD_DLLP_MASK, swReg->badDllpMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_BAD_TLP_MASK, swReg->badTlpMsk);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_RX_ERR_MASK, swReg->rcvrErrMsk);

  return SystemP_SUCCESS;
} /* Pcie_read_corErrMask_reg */

/*****************************************************************************
 * Read and split up the Header Log register
 ****************************************************************************/
static int32_t Pcie_readHdrLogReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_HdrLogReg *reg, int32_t regNum)
{
  reg->raw = reg->hdrDW = baseAddr->HDR_LOG_OFF[regNum];

  return SystemP_SUCCESS;
} /* Pcie_read_hdrLog_reg */

/*****************************************************************************
 * Read and split up the Root Error Command register
 ****************************************************************************/
static int32_t Pcie_readRootErrCmdReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_RootErrCmdReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ROOT_ERR_CMD_OFF;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_FATAL_ERR_REPORTING_EN, swReg->ferrRptEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_NON_FATAL_ERR_REPORTING_EN, swReg->nferrRptEn);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_CORR_ERR_REPORTING_EN, swReg->cerrRptEn);

  return SystemP_SUCCESS;
} /* Pcie_read_rootErrCmd_reg */

/*****************************************************************************
 * Read and split up the Root Error Status register
 ****************************************************************************/
static int32_t Pcie_readRootErrStReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_RootErrStReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ROOT_ERR_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ADV_ERR_INT_MSG_NUM, swReg->aerIntMsg);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FATAL_ERR_MSG_RX, swReg->ferrRcv);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_NON_FATAL_ERR_MSG_RX, swReg->nfErr);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FIRST_UNCORR_FATAL, swReg->uncorFatal);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_FATAL_NON_FATAL_RX, swReg->multFnf);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_FATAL_NON_FATAL_RX, swReg->errFnf);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_COR_RX, swReg->multCor);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_COR_RX, swReg->corrErr);

  return SystemP_SUCCESS;
} /* Pcie_read_rootErrSt_reg */

/*****************************************************************************
 * Read and split up the Error Source Identification register
 ****************************************************************************/
static int32_t Pcie_readErrSrcIDReg(const CSL_pcie_rc_coreRegs *baseAddr, Pcie_ErrSrcIDReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ERR_SRC_ID_OFF;

  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_FATAL_NON_FATAL_SOURCE_ID, swReg->fnfSrcID);
  PCIE_GETBITS(val, CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_COR_SOURCE_ID, swReg->corrSrcID);

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up the PL CONF Ack Latency and Replay Timer register
 ****************************************************************************/
static int32_t Pcie_readPlAckTimerReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlAckTimerReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ACK_LATENCY_TIMER_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ACK_LATENCY_TIMER_OFF_ROUND_TRIP_LATENCY_TIME_LIMIT, swReg->rndTrpLmt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ACK_LATENCY_TIMER_OFF_REPLAY_TIME_LIMIT, swReg->rplyLmt);

  return SystemP_SUCCESS;
} /* Pcie_read_plAckTimer_reg */

/*****************************************************************************
 * Read and split up the PL CONF Vendor Specific DLLP register
 ****************************************************************************/
static int32_t Pcie_readPlOMsgReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlOMsgReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->VENDOR_SPEC_DLLP_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VENDOR_SPEC_DLLP_OFF_VENDOR_SPEC_DLLP, swReg->oMsg);

  return SystemP_SUCCESS;
} /* Pcie_read_plOMsg_reg */

/*****************************************************************************
 * Read and split up the PL CONF Port Force Link register
 ****************************************************************************/
static int32_t Pcie_readPlForceLinkReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlForceLinkReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PORT_FORCE_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_LINK_NUM, swReg->linkNum);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_FORCE_EN, swReg->forceLink);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_LINK_STATE, swReg->lnkState);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_FORCED_LTSSM, swReg->forcedLtssmState);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_DO_DESKEW_FOR_SRIS, swReg->doDeskewForSris);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->lpeCnt = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_plForceLink_reg */

/*****************************************************************************
 * Read and split up the PL CONF Ack Frequency and L0-L1 ASPM register
 ****************************************************************************/
static int32_t Pcie_readAckFreqReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_AckFreqReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ACK_F_ASPM_CTRL_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ACK_FREQ, swReg->ackFreq);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ACK_N_FTS, swReg->nFts);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_COMMON_CLK_N_FTS, swReg->commNFts);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_L0S_ENTRANCE_LATENCY, swReg->l0sEntryLatency);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_L1_ENTRANCE_LATENCY, swReg->l1EntryLatency);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ENTER_ASPM, swReg->aspmL1);

  return SystemP_SUCCESS;
} /* Pcie_read_ackFreq_reg */

/*****************************************************************************
 * Read and split up the PL CONF Lane Skew register
 ****************************************************************************/
static int32_t Pcie_readLaneSkewReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_LaneSkewReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->LANE_SKEW_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_INSERT_LANE_SKEW, swReg->laneSkew);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_FLOW_CTRL_DISABLE, swReg->fcDisable);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_ACK_NAK_DISABLE, swReg->ackDisable);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_IMPLEMENT_NUM_LANES, swReg->implementNumLanes);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_DISABLE_LANE_TO_LANE_DESKEW, swReg->l2Deskew);

  return SystemP_SUCCESS;
} /* Pcie_read_laneSkew_reg */

/*****************************************************************************
 * Read and split up the PL CONF Timer Control and Symbol Number (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_readSymNumReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_SymNumReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->TIMER_CTRL_MAX_FUNC_NUM_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_MAX_FUNC_NUM, swReg->maxFunc);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_REPLAY_TIMER, swReg->replayTimer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_ACK_NAK, swReg->ackLatencyTimer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_FAST_LINK_SCALING_FACTOR, swReg->fastLinkScalingFactor);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->fcWatchTimer = 0;
  swReg->skpCount = 0;
  swReg->numTs2Symbols = 0;
  swReg->tsCount = 0;

  return SystemP_SUCCESS;
} /* Pcie_read_symNum_reg */

/*****************************************************************************
 * Read and split up the PL CONF Symbol Timer and Filter Mask (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_readSymTimerFltMaskReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_SymTimerFltMaskReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->SYMBOL_TIMER_FILTER_1_OFF;
  uint32_t mask1;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_SKP_INT_VAL, swReg->skpValue);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_DISABLE_FC_WD_TIMER, swReg->fcWdogDisable);

  /* Extract MASK_RADM_1 */
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_MASK_RADM_1, mask1);

  /* Repack into named bits per rev 0, they actually match */
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CFG_DROP, swReg->f1CfgDrop);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_IO_DROP, swReg->f1IoDrop);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_MSG_DROP, swReg->f1MsgDrop);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CPL_ECRC_DROP, swReg->f1CplEcrcDrop);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_ECRC_DROP, swReg->f1EcrcDrop);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CPL_LEN_TEST, swReg->f1CplLenTest);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CPL_ATTR_TEST, swReg->f1CplAttrTest);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CPL_TC_TEST, swReg->f1CplTcTest);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CPL_FUNC_TEST, swReg->f1CplFuncTest);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CPL_REQID_TEST, swReg->f1CplReqIDTest);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CPL_TAGERR_TEST, swReg->f1CplTagErrTest);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_LOCKED_RD_AS_UR, swReg->f1LockedRdAsUr);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_CFG1_RE_AS_US, swReg->f1Cfg1ReAsUs);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_UR_OUT_OF_BAR, swReg->f1UrOutOfBar);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_UR_POISON, swReg->f1UrPoison);
  PCIE_GETBITS(mask1, PCIE_FLTMASK1_UR_FUN_MISMATCH, swReg->f1UrFunMismatch);

  return SystemP_SUCCESS;
} /* Pcie_read_symTimerFltMask_reg */

/*****************************************************************************
 * Read and split up the PL CONF Filter Mask 2 register
 ****************************************************************************/
static int32_t Pcie_readFltMask2Reg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_FltMask2Reg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->FILTER_MASK_2_OFF;

  /* hw rev 0 bits are still valid plus 4 new ones.  Other 24 bits are reserved */
  PCIE_GETBITS(val, PCIE_FLTMASK2_DROP_PRS, swReg->dropPRS);
  PCIE_GETBITS(val, PCIE_FLTMASK2_UNMASK_TD, swReg->unmaskTD);
  PCIE_GETBITS(val, PCIE_FLTMASK2_UNMASK_UR_POIS, swReg->unmaskUrPOIS);
  PCIE_GETBITS(val, PCIE_FLTMASK2_DROP_LN, swReg->dropLN);
  PCIE_GETBITS(val, PCIE_FLTMASK2_FLUSH_REQ, swReg->flushReq);
  PCIE_GETBITS(val, PCIE_FLTMASK2_DLLP_ABORT, swReg->dllpAbort);
  PCIE_GETBITS(val, PCIE_FLTMASK2_VMSG1_DROP, swReg->vmsg1Drop);
  PCIE_GETBITS(val, PCIE_FLTMASK2_VMSG0_DROP, swReg->vmsg0Drop);

  return SystemP_SUCCESS;
} /* Pcie_read_fltMask2_reg */

/*****************************************************************************
 * Read and split up the Debug 0 register
 ****************************************************************************/
static int32_t Pcie_readDebug0Reg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_Debug0Reg *reg)
{
  uint32_t val = reg->raw = baseAddr->PL_DEBUG0_OFF;

  PCIE_GETBITS(val, PCIE_DEBUG0_TS_LINK_CTRL, reg->tsLnkCtrl);
  PCIE_GETBITS(val, PCIE_DEBUG0_TS_LANE_K237, reg->tsLaneK237);
  PCIE_GETBITS(val, PCIE_DEBUG0_TS_LINK_K237, reg->tsLinkK237);
  PCIE_GETBITS(val, PCIE_DEBUG0_RCVD_IDLE0, reg->rcvdIdle0);
  PCIE_GETBITS(val, PCIE_DEBUG0_RCVD_IDLE1, reg->rcvdIdle1);
  PCIE_GETBITS(val, PCIE_DEBUG0_PIPE_TXDATA, reg->pipeTxData);
  PCIE_GETBITS(val, PCIE_DEBUG0_PIPE_TXDATAK, reg->pipeTxDataK);
  PCIE_GETBITS(val, PCIE_DEBUG0_TXB_SKIP_TX, reg->skipTx);
  PCIE_GETBITS(val, PCIE_DEBUG0_LTSSM_STATE, reg->ltssmState);

  return SystemP_SUCCESS;
} /* Pcie_read_debug0_reg */

/*****************************************************************************
 * Read and split up the Debug 1 register
 ****************************************************************************/
static int32_t Pcie_readDebug1Reg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_Debug1Reg *reg)
{
  uint32_t val = reg->raw = baseAddr->PL_DEBUG1_OFF;

  PCIE_GETBITS(val, PCIE_DEBUG1_SCRAMBLER_DISABLE, reg->scramblerDisable);
  PCIE_GETBITS(val, PCIE_DEBUG1_LINK_DISABLE, reg->linkDisable);
  PCIE_GETBITS(val, PCIE_DEBUG1_LINK_IN_TRAINING, reg->linkInTraining);
  PCIE_GETBITS(val, PCIE_DEBUG1_RCVR_REVRS_POL_EN, reg->rcvrRevrsPolEn);
  PCIE_GETBITS(val, PCIE_DEBUG1_TRAINING_RST_N, reg->trainingRstN);
  PCIE_GETBITS(val, PCIE_DEBUG1_PIPE_TXDETECTRX_LB, reg->pipeTxdetectrxLb);
  PCIE_GETBITS(val, PCIE_DEBUG1_PIPE_TXELECIDLE, reg->pipeTxelecidle);
  PCIE_GETBITS(val, PCIE_DEBUG1_PIPE_TXCOMPLIANCE, reg->pipeTxcompliance);
  PCIE_GETBITS(val, PCIE_DEBUG1_APP_INIT_RST, reg->appInitRst);
  PCIE_GETBITS(val, PCIE_DEBUG1_RMLH_TS_LINK_NUM, reg->rmlhTsLinkNum);
  PCIE_GETBITS(val, PCIE_DEBUG1_XMLH_LINK_UP, reg->xmlhLinkUp);
  PCIE_GETBITS(val, PCIE_DEBUG1_RMLH_INSKIP_RCV, reg->rmlhInskipRcv);
  PCIE_GETBITS(val, PCIE_DEBUG1_RMLH_TS1_RCVD, reg->rmlhTs1Rcvd);
  PCIE_GETBITS(val, PCIE_DEBUG1_RMLH_TS2_RCVD, reg->rmlhTs2Rcvd);
  PCIE_GETBITS(val, PCIE_DEBUG1_RMLH_RCVD_LANE_REV, reg->rmlhRcvdLaneRev);

  return SystemP_SUCCESS;
} /* Pcie_read_debug1_reg */

/*****************************************************************************
 * Read and split up the PL CONF Link Width and Speed Change Control (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_readGen2Reg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_Gen2Reg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->GEN2_CTRL_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_FAST_TRAINING_SEQ, swReg->numFts);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_NUM_OF_LANES, swReg->lnEn);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_PRE_DET_LANE, swReg->preDetLane);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_AUTO_LANE_FLIP_CTRL_EN, swReg->autoFlipEn);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE, swReg->dirSpd);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_CONFIG_PHY_TX_CHANGE, swReg->txSwing);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_CONFIG_TX_COMP_RX, swReg->txCmpl);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_SEL_DEEMPHASIS, swReg->deemph);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_GEN1_EI_INFERENCE, swReg->gen1EiInference);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfWidthSpeedCtl_reg */

/*****************************************************************************
 * Read and split up the PL CONF AXI Multiple Outbound Decomposed NP
 * SubRequests Control Register (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfObnpSubreqCtrlReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfObnpSubreqCtrlReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_OB_RD_SPLIT_BURST_EN, swReg->enObnpSubreq);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfObnpSubreqCtrl_reg */

static int32_t Pcie_readErrIrqStatusReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_ErrIrqStatusReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_STATUS;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_AER, swReg->errAer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_CORR, swReg->errCorr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_NONFATAL, swReg->errNonFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_FATAL, swReg->errFatal);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_SYS, swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Read and split up the PL CONF Transmit Posted FC Credit Status
 * (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfTrPStsRReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfTrPStsRReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->TX_P_FC_CREDIT_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_DATA_FC_CREDIT, swReg->pdCrdt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_HEADER_FC_CREDIT, swReg->phCrdt);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfTrPStsR_reg */

/*****************************************************************************
 * Read and split up the PL CONF Transmit Non-Posted FC Credit Status
 * (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfTrNpStsRReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfTrNpStsRReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->TX_NP_FC_CREDIT_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_DATA_FC_CREDIT, swReg->npdCrdt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_HEADER_FC_CREDIT, swReg->nphCrdt);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfTrNpStsR_reg */

/*****************************************************************************
 * Read and split up the PL CONF Transmit Completion FC Credit Status
 * (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfTrCStsRReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfTrCStsRReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->TX_CPL_FC_CREDIT_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_DATA_FC_CREDIT, swReg->cpldCrdt);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_HEADER_FC_CREDIT, swReg->cplhCrdt);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfTrCStsR_reg */

/*****************************************************************************
 * Read and split up the PL CONF Queue Status (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfQStsRReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfQStsRReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->QUEUE_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_TLP_FC_CREDIT_NON_RETURN, swReg->crdtNotRtrn);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TX_RETRY_BUFFER_NE, swReg->rtybNotEmpty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_QUEUE_NON_EMPTY, swReg->rcvqNotEmpty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_QUEUE_OVERFLOW, swReg->rxQueueOverflow);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_NON_EMPTY, swReg->rxSerQNEmpty);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_WRITE_ERR, swReg->rxSerQWErr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_READ_ERR, swReg->rxSerRErr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL, swReg->fcLatencyOvr);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_EN, swReg->fcLatencyOvrEn);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfQStsR_reg */

/*****************************************************************************
 * Read and split up the PL CONF VC Transmit Arbitration 1 (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfVcTrAR1Reg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfVcTrAR1Reg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->VC_TX_ARBI_1_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_0, swReg->wrrVc0);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_1, swReg->wrrVc1);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_2, swReg->wrrVc2);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_3, swReg->wrrVc3);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfVcTrAR1_reg */

/*****************************************************************************
 * Read and split up the PL CONF VC Transmit Arbitration 2 (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfVcTrAR2Reg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfVcTrAR2Reg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->VC_TX_ARBI_2_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_4, swReg->wrrVc4);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_5, swReg->wrrVc5);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_6, swReg->wrrVc6);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_7, swReg->wrrVc7);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfVcTrAR2_reg */

/*****************************************************************************
 * Read and split up the PL CONF VC# Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_readPlconfVcPrQCReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfVcPrQCReg *swReg, int32_t vcNum)
{
  /* Note: not checking vcNum as its internally generated */
  uint32_t val = swReg->raw = baseAddr->VC_RX_Q_CTRL[vcNum].VC_P_RX_Q_CTRL_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_CREDIT, swReg->pDcrd);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HEADER_CREDIT, swReg->pHcrd);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_SCALE, swReg->pDataScale);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HDR_SCALE, swReg->pHdrScale);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC, swReg->orderingRules);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q, swReg->strictVcPriority);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->pQmode = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_plconfVcPrQC_reg */

/*****************************************************************************
 * Read and split up the PL CONF VC0 Non-Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_readPlconfVcNprQCReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfVcNprQCReg *swReg, int32_t vcNum)
{
  /* Note: not checking vcNum as its internally generated */
  uint32_t val = swReg->raw = baseAddr->VC_RX_Q_CTRL[vcNum].VC_NP_RX_Q_CTRL_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_CREDIT, swReg->npDcrd);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HEADER_CREDIT, swReg->npHcrd);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_SCALE, swReg->npDataScale);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HDR_SCALE, swReg->npHdrScale);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->npQmode = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_plconfVcNprQC_reg */

/*****************************************************************************
 * Read and split up the PL CONF VC0 Completion Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_readPlconfVcCrQCReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfVcCrQCReg *swReg, int32_t vcNum)
{
  /* Note: not checking vcNum as its internally generated */
  uint32_t val = swReg->raw = baseAddr->VC_RX_Q_CTRL[vcNum].VC_CPL_RX_Q_CTRL_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_CREDIT, swReg->cplDcrd);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HEADER_CREDIT, swReg->cplHcrd);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_SCALE, swReg->cplDataScale);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HDR_SCALE, swReg->cplHdrScale);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->cplQmode = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_plconfVcCrQC_reg */

/*****************************************************************************
 * Read and split up the PL CONF PHY Status (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfPhyStsRReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfPhyStsRReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PHY_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PHY_STATUS_OFF_PHY_STATUS, swReg->phySts);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfPhyStsR_reg */

/*****************************************************************************
 * Read and split up the PL CONF PHY Control (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfPhyCtrlRReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfPhyCtrlRReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PHY_CONTROL_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PHY_CONTROL_OFF_PHY_CONTROL, swReg->phyCtrl);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfPhyCtrlR_reg */

/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Address (RC-mode MSI receiver)
 * register
 ****************************************************************************/
static int32_t Pcie_readPlconfMsiCtrlAddressReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlAddressReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL_ADDR_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CTRL_ADDR_OFF_MSI_CTRL_ADDR, swReg->msiCtrlAddress);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfMsiCtrlAddress_reg */

/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Upper Address
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_readPlconfMsiCtrlUpperAddressReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlUpperAddressReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL_UPPER_ADDR_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CTRL_UPPER_ADDR_OFF_MSI_CTRL_UPPER_ADDR, swReg->msiCtrlUpperAddress);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfMsiCtrlUpperAddress_reg */

/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Enable
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_readPlconfMsiCtrlIntEnableReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlIntEnableReg *swReg, int32_t n)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_EN_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_EN_OFF_MSI_CTRL_INT_EN, swReg->msiCtrlIntEnable);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfMsiCtrlIntEnable_reg */

/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Mask
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_readPlconfMsiCtrlIntMaskReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlIntMaskReg *swReg, int32_t n)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_MASK_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_MASK_OFF_MSI_CTRL_INT_MASK, swReg->msiCtrlIntMask);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfMsiCtrlIntMask_reg */

/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Status
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_readPlconfMsiCtrlIntStatusReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlIntStatusReg *swReg, int32_t n)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_STATUS_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_STATUS_OFF_MSI_CTRL_INT_STATUS, swReg->msiCtrlIntStatus);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfMsiCtrlIntStatus_reg */

/*****************************************************************************
 * Read and split up the PL CONF MSI Controller General Purpose IO
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_readPlconfMsiCtrlGpioReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlGpioReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->MSI_GPIO_IO_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MSI_GPIO_IO_OFF_MSI_GPIO_REG, swReg->msiCtrlGpio);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfMsiCtrlGpio_reg */

/*****************************************************************************
 * Read and split up the PL CONF PIPE loopback control (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfPipeLoopbackReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfPipeLoopbackReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->PIPE_LOOPBACK_CONTROL_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_PIPE_LOOPBACK_CONTROL_OFF_PIPE_LOOPBACK, swReg->loopbackEn);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfPipeLoopback_reg */

/*****************************************************************************
 * Read and split up the PL CONF DIF Read-Only register Write Enable (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_readPlconfDbiRoWrEnReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfDbiRoWrEnReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->MISC_CONTROL_1_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_DBI_RO_WR_EN, swReg->cxDbiRoWrEn);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_DEFAULT_TARGET, swReg->defaultTarget);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_UR_CA_MASK_4_TRGT1, swReg->urCaMask4Trgt1);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_SIMPLIFIED_REPLAY_TIMER, swReg->simpReplayTimer);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_RSVDP_4, swReg->ariDevNumber);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfDbiRoWrEn_reg */

/*****************************************************************************
 * Read and split up the PL CONF AXI Slave Error Response (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfAxiSlvErrRespReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfAxiSlvErrRespReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->AMBA_ERROR_RESPONSE_DEFAULT_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_GLOBAL, swReg->peripheralErrMap);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_VENDORID, swReg->noVidErrMap);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_CRS, swReg->errorResponseCrs);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_MAP, swReg->errorResponseMap);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->dbiErrMap = 0u;
  swReg->resetTimeoutErrMap = 0u;

  return SystemP_SUCCESS;
} /* Pcie_read_plconfAxiSlvErrResp_reg */

/*****************************************************************************
 * Read and split up the PL CONF Link Down AXI Slave Timeout (Sticky) register
 ****************************************************************************/
static int32_t Pcie_readPlconfAxiSlvTimeoutReg(const CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfAxiSlvTimeoutReg *swReg)
{
  uint32_t val = swReg->raw = baseAddr->AMBA_LINK_TIMEOUT_OFF;

  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_PERIOD_DEFAULT, swReg->timeoutValue);
  PCIE_GETBITS(val, CSL_PCIE_EP_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_ENABLE_DEFAULT, swReg->flushEn);

  return SystemP_SUCCESS;
} /* Pcie_read_plconfAxiSlvTimeout_reg */

/*****************************************************************************
 * Read and split up the PL CONF iATU Region Limit Address register
 ****************************************************************************/
static int32_t Pcie_readPlconfIatuRegLimitReg(const CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegLimitReg *swReg)
{
  static int32_t status = SystemP_SUCCESS;
  uint32_t val;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
        if (simIatuWindow->regionDirection == 0U)
        {
            /* 0U == OUTBOUND */
            val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LIMIT_ADDR_OFF_OUTBOUND;

            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_RW, swReg->iatuRegLimit);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_HW, swReg->ones);
        }
        else
        {
            /* INBOUND */
            val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LIMIT_ADDR_OFF_INBOUND;

            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_RW, swReg->iatuRegLimit);
            PCIE_GETBITS(val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_HW, swReg->ones);
        }
  }
  else
  {
        status = SystemP_FAILURE;
  }

  return status;
} /* Pcie_read_plconfIatuRegLimit_reg */

/*********************************************************************
 * Reads any register
 ********************************************************************/
int32_t Pcie_readRegs(Pcie_Handle handle, Pcie_Location location, Pcie_Registers * readRegs)
{
  int32_t status = SystemP_SUCCESS;
  int32_t i;
  Pcie_Config *pcieCfg = NULL;
  Pcie_DeviceCfgBaseAddr *cfg = NULL;
  Pcie_DeviceCfgBaseAddrs *bases = NULL;

  Pcie_PlconfIatuIndexReg *simIatuWindow = NULL;


  /* Base Address for the Config Space
  These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_pcie_ep_coreRegs *baseCfgEp = NULL;
  CSL_pcie_rc_coreRegs *baseCfgRc = NULL;

  if (handle != NULL)
  {
        pcieCfg = (Pcie_Config *)handle;
        cfg = Pcie_handleGetBases(handle);

        simIatuWindow = &pciev0LocalObj.simIatuWindow[pcieCfg->attrs->deviceNum];

        if (cfg != NULL)
        {
            bases = (Pcie_DeviceCfgBaseAddrs *)cfg->cfgBase;
        }
        else
        {
            status = SystemP_FAILURE;
        }

        if (status == SystemP_SUCCESS)
        {
            baseCfgEp = (CSL_pcie_ep_coreRegs *)bases->cfgBase;
            baseCfgRc = (CSL_pcie_rc_coreRegs *)bases->cfgBase;
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
            char *remoteBase = (char *)cfg->dataBase + bases->remoteOffset;

            if (remoteBase != NULL)
            {
                baseCfgEp = (CSL_pcie_ep_coreRegs *)(remoteBase);
                baseCfgRc = (CSL_pcie_rc_coreRegs *)(remoteBase);
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
        status = Pcie_readPidReg(baseCfgEp, readRegs->pid);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->cmdStatus != NULL))
  {
        status = Pcie_readCmdStatusReg(baseCfgEp, readRegs->cmdStatus);
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
        status = Pcie_readRstCmdReg(baseCfgEp, readRegs->rstCmd);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->ptmCfg != NULL))
  {
        status = Pcie_readPtmCfgReg(baseCfgEp, readRegs->ptmCfg);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->pmCmd != NULL))
  {
        /* Not supported in this version */
        status = Pcie_readPmCmdReg(baseCfgEp, readRegs->pmCmd);
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
        status = Pcie_readIrqEOIReg(baseCfgEp, readRegs->irqEOI);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->msiIrq != NULL))
  {
        status = Pcie_readMsiIrqReg(baseCfgEp, readRegs->msiIrq);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->epIrqSet != NULL))
  {
        status = Pcie_readEpIrqSetReg(baseCfgEp, readRegs->epIrqSet);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->epIrqClr != NULL))
  {
        status = Pcie_readEpIrqClrReg(baseCfgEp, readRegs->epIrqClr);
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
            status = Pcie_ReadGenPurposeReg(baseCfgEp, readRegs->genPurpose[i], i);
        }
  }
  for (i = 0; i < 8; i++)
  {
        if ((status == SystemP_SUCCESS) && (readRegs->msiIrqStatusRaw[i] != NULL))
        {
            status = Pcie_ReadMsiIrqStatusRaw(baseCfgEp, readRegs->msiIrqStatusRaw[i], i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->msiIrqStatus[i] != NULL))
        {
            status = Pcie_readMsiIrqStatusReg(baseCfgEp, readRegs->msiIrqStatus[i], i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->msiIrqEnableSet[i] != NULL))
        {
            status = Pcie_readMsiIrqEnableSetReg(baseCfgEp, readRegs->msiIrqEnableSet[i], i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->msiIrqEnableClr[i] != NULL))
        {
            status = Pcie_readMsiIrqEnableClrReg(baseCfgEp, readRegs->msiIrqEnableClr[i], i);
        }
  }
  for (i = 0; i < 4; i++)
  {
        if ((status == SystemP_SUCCESS) && (readRegs->legacyIrqStatusRaw[i] != NULL))
        {
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->legacyIrqStatus[i] != NULL))
        {
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->legacyIrqEnableSet[i] != NULL))
        {
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (readRegs->legacyIrqEnableClr[i] != NULL))
        {
            status = SystemP_FAILURE;
        }
  }
  if ((status == SystemP_SUCCESS) && (readRegs->errIrqStatusRaw != NULL))
  {
        status = Pcie_readErrIrqStatusRawReg(baseCfgEp, readRegs->errIrqStatusRaw);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->errIrqStatus != NULL))
  {
        status = Pcie_readErrIrqStatusReg(baseCfgEp, readRegs->errIrqStatus);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->errIrqEnableSet != NULL))
  {
        status = Pcie_readErrIrqEnableSetReg(baseCfgEp, readRegs->errIrqEnableSet);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->errIrqEnableClr != NULL))
  {
        status = Pcie_readErrIrqEnableClrReg(baseCfgEp, readRegs->errIrqEnableClr);
  }

  if ((status == SystemP_SUCCESS) && (readRegs->pmRstIrqStatusRaw != NULL))
  {
        status = Pcie_readPmRstIrqStatusRawReg(baseCfgEp, readRegs->pmRstIrqStatusRaw);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->pmRstIrqStatus != NULL))
  {
        status = Pcie_readPmRstIrqStatusReg(baseCfgEp, readRegs->pmRstIrqStatus);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->pmRstIrqEnableSet != NULL))
  {
        status = Pcie_readPmRstIrqEnableSetReg(baseCfgEp, readRegs->pmRstIrqEnableSet);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->pmRstIrqEnableClr != NULL))
  {
        status = Pcie_readPmRstIrqEnableClrReg(baseCfgEp, readRegs->pmRstIrqEnableClr);
  }

  if ((status == SystemP_SUCCESS) && (readRegs->ptmIrqStatusRaw != NULL))
  {
        status = Pcie_readPtmIrqStatusRawReg(baseCfgEp, readRegs->ptmIrqStatusRaw);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->ptmIrqStatus != NULL))
  {
        status = Pcie_readPtmIrqStatusReg(baseCfgEp, readRegs->ptmIrqStatus);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->ptmIrqEnableSet != NULL))
  {
        status = Pcie_readPtmIrqEnableSetReg(baseCfgEp, readRegs->ptmIrqEnableSet);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->ptmIrqEnableClr != NULL))
  {
        status = Pcie_readPtmIrqEnableClrReg(baseCfgEp, readRegs->ptmIrqEnableClr);
  }

  for (i = 0; i < 8; i++)
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

  for (i = 0; i < 4; i++)
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
        status = Pcie_readVendorDevId(&baseCfgEp->DEVICE_ID_VENDOR_ID_REG, readRegs->vndDevId);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->statusCmd != NULL))
  {
        status = Pcie_readStatusCmdReg(&baseCfgEp->STATUS_COMMAND_REG, readRegs->statusCmd);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->revId != NULL))
  {
        status = Pcie_readRevIdReg(&baseCfgEp->CLASS_CODE_REVISION_ID, readRegs->revId);
  }

  if ((status == SystemP_SUCCESS) && (readRegs->bist != NULL))
  {
        status = Pcie_readBistReg(baseCfgEp, readRegs->bist);
  }

  /*Type 0 Registers*/
  if ((status == SystemP_SUCCESS) && (readRegs->type0BarIdx != NULL))
  {
        status = Pcie_readType0BarReg(baseCfgEp, &(readRegs->type0BarIdx->reg),
                                      readRegs->type0BarIdx->idx);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type0Bar32bitIdx != NULL))
  {
        status = Pcie_readType0Bar32bitReg(baseCfgEp, &(readRegs->type0Bar32bitIdx->reg),
                                           readRegs->type0Bar32bitIdx->idx);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type0BarMask32bitIdx != NULL))
  {
        status = Pcie_readType0Bar32bitReg(baseCfgEp, &(readRegs->type0BarMask32bitIdx->reg),
                                           readRegs->type0BarMask32bitIdx->idx);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->subId != NULL))
  {
        status = Pcie_readSubIdReg(baseCfgEp, readRegs->subId);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->cardbusCisPointer != NULL))
  {
        status = Pcie_readCardbusCisPointerReg(baseCfgEp, readRegs->cardbusCisPointer);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->expRom != NULL))
  {
        status = Pcie_readExpRomReg(baseCfgEp, readRegs->expRom);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->capPtr != NULL))
  {
        status = Pcie_readCapPtrReg(baseCfgEp, readRegs->capPtr);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->intPin != NULL))
  {
        status = Pcie_readIntPinReg(baseCfgEp, readRegs->intPin);
  }

  /*Type 1 Registers*/
  if ((status == SystemP_SUCCESS) && (readRegs->type1BistHeader != NULL))
  {
        status = Pcie_readType1BistHeaderReg(baseCfgRc, readRegs->type1BistHeader);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1BarIdx != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1Bar32bitIdx != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1BarMask32bitIdx != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1BusNum != NULL))
  {
        status = Pcie_readType1BusNumReg(baseCfgRc, readRegs->type1BusNum);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1SecStat != NULL))
  {
        status = Pcie_readType1SecStatReg(baseCfgRc, readRegs->type1SecStat);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1Memspace != NULL))
  {
        status = Pcie_readType1MemspaceReg(baseCfgRc, readRegs->type1Memspace);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->prefMem != NULL))
  {
        status = Pcie_readPrefMemReg(baseCfgRc, readRegs->prefMem);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->prefBaseUpper != NULL))
  {
        status = Pcie_readPrefBaseUpperReg(baseCfgRc, readRegs->prefBaseUpper);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->prefLimitUpper != NULL))
  {
        status = Pcie_readPrefLimitUpperReg(baseCfgRc, readRegs->prefLimitUpper);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1IOSpace != NULL))
  {
        status = Pcie_readType1IOSpaceReg(baseCfgRc, readRegs->type1IOSpace);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1CapPtr != NULL))
  {
        status = Pcie_readType1CapPtrReg(baseCfgRc, readRegs->type1CapPtr);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1ExpnsnRom != NULL))
  {
        status = Pcie_readType1ExpnsnRomReg(baseCfgRc, readRegs->type1ExpnsnRom);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->type1BridgeInt != NULL))
  {
        status = Pcie_readType1BridgeIntReg(baseCfgRc, readRegs->type1BridgeInt);
  }

  /* Power Management Capabilities Registers */
  if ((status == SystemP_SUCCESS) && (readRegs->pmCap != NULL))
  {
        status = Pcie_readPmCapReg(baseCfgEp, readRegs->pmCap);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->pmCapCtlStat != NULL))
  {
        status = Pcie_readPmCapCtlStatReg(baseCfgEp, readRegs->pmCapCtlStat);
  }

  /*MSI Registers*/
  if ((status == SystemP_SUCCESS) && (readRegs->msiCap != NULL))
  {
        status = Pcie_readMsiCapReg(baseCfgEp, readRegs->msiCap);
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
        status = Pcie_readMsiDataReg(baseCfgEp, readRegs->msiData);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->msiCapOff10H != NULL))
  {
        status = Pcie_readMsiCapOff10HReg(baseCfgEp, readRegs->msiCapOff10H);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->msiCapOff14H != NULL))
  {
        status = Pcie_readMsiCapOff14HReg(baseCfgEp, readRegs->msiCapOff14H);
  }

  /*Capabilities Registers*/
  if ((status == SystemP_SUCCESS) && (readRegs->pciesCap != NULL))
  {
        status = Pcie_readPciesCapReg(&baseCfgEp->PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG, readRegs->pciesCap);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->deviceCap != NULL))
  {
        status = Pcie_readDeviceCapReg(&baseCfgEp->DEVICE_CAPABILITIES_REG, readRegs->deviceCap);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->devStatCtrl != NULL))
  {
        status = Pcie_readDevStatCtrlReg(&baseCfgEp->DEVICE_CONTROL_DEVICE_STATUS, readRegs->devStatCtrl);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->linkCap != NULL))
  {
        status = Pcie_readLinkCapReg(&baseCfgEp->LINK_CAPABILITIES_REG, readRegs->linkCap);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->linkStatCtrl != NULL))
  {
        status = Pcie_readLinkStatCtrlReg(&baseCfgEp->LINK_CONTROL_LINK_STATUS_REG, readRegs->linkStatCtrl);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->slotCap != NULL))
  {
        status = Pcie_readSlotCapReg(baseCfgRc, readRegs->slotCap);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->slotStatCtrl != NULL))
  {
        status = Pcie_readSlotStatCtrlReg(baseCfgRc, readRegs->slotStatCtrl);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->rootCtrlCap != NULL))
  {
        status = Pcie_readRootCtrlCapReg(baseCfgRc, readRegs->rootCtrlCap);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->rootStatus != NULL))
  {
        status = Pcie_readRootStatusReg(baseCfgRc, readRegs->rootStatus);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->devCap2 != NULL))
  {
        status = Pcie_readDevCap2Reg(&baseCfgEp->DEVICE_CAPABILITIES2_REG, readRegs->devCap2);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->devStatCtrl2 != NULL))
  {
        status = Pcie_readDevStatCtrl2Reg(&baseCfgEp->DEVICE_CONTROL2_DEVICE_STATUS2_REG, readRegs->devStatCtrl2);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->linkCap2 != NULL))
  {
        status = Pcie_readLinkCap2Reg(&baseCfgEp->LINK_CAPABILITIES2_REG, readRegs->linkCap2);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->linkCtrl2 != NULL))
  {
        status = Pcie_readLinkCtrl2Reg(&baseCfgEp->LINK_CONTROL2_LINK_STATUS2_REG, readRegs->linkCtrl2);
  }

  /*Capabilities Extended Registers*/
  if ((status == SystemP_SUCCESS) && (readRegs->extCap != NULL))
  {
        status = Pcie_readExtCapReg(baseCfgEp, readRegs->extCap);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->uncErr != NULL))
  {
        status = Pcie_readUncErrReg(baseCfgEp, readRegs->uncErr);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->uncErrMask != NULL))
  {
        status = Pcie_readUncErrMaskReg(baseCfgEp, readRegs->uncErrMask);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->uncErrSvrty != NULL))
  {
        status = Pcie_readUncErrSvrtyReg(baseCfgEp, readRegs->uncErrSvrty);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->corErr != NULL))
  {
        status = Pcie_readCorErrReg(baseCfgEp, readRegs->corErr);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->corErrMask != NULL))
  {
        status = Pcie_readCorErrMaskReg(baseCfgEp, readRegs->corErrMask);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->accr != NULL))
  {
        status = Pcie_readAccrReg(baseCfgEp, readRegs->accr);
  }
  for (i = 0; i < 4; i++)
  {
        if ((status == SystemP_SUCCESS) && (readRegs->hdrLog[i] != NULL))
        {
            status = Pcie_readHdrLogReg(baseCfgEp, readRegs->hdrLog[i], i);
        }
  }
  if ((status == SystemP_SUCCESS) && (readRegs->rootErrCmd != NULL))
  {
        status = Pcie_readRootErrCmdReg(baseCfgRc, readRegs->rootErrCmd);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->rootErrSt != NULL))
  {
        status = Pcie_readRootErrStReg(baseCfgRc, readRegs->rootErrSt);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->errSrcID != NULL))
  {
        status = Pcie_readErrSrcIDReg(baseCfgRc, readRegs->errSrcID);
  }

  /*Port Logic Registers*/
  if ((status == SystemP_SUCCESS) && (readRegs->plAckTimer != NULL))
  {
        status = Pcie_readPlAckTimerReg(baseCfgEp, readRegs->plAckTimer);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plOMsg != NULL))
  {
        status = Pcie_readPlOMsgReg(baseCfgEp, readRegs->plOMsg);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plForceLink != NULL))
  {
        status = Pcie_readPlForceLinkReg(baseCfgEp, readRegs->plForceLink);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->ackFreq != NULL))
  {
        status = Pcie_readAckFreqReg(baseCfgEp, readRegs->ackFreq);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->lnkCtrl != NULL))
  {
        status = Pcie_readLnkCtrlReg(baseCfgEp, readRegs->lnkCtrl);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->laneSkew != NULL))
  {
        status = Pcie_readLaneSkewReg(baseCfgEp, readRegs->laneSkew);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->symNum != NULL))
  {
        status = Pcie_readSymNumReg(baseCfgEp, readRegs->symNum);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->symTimerFltMask != NULL))
  {
        status = Pcie_readSymTimerFltMaskReg(baseCfgEp, readRegs->symTimerFltMask);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->fltMask2 != NULL))
  {
        status = Pcie_readFltMask2Reg(baseCfgEp, readRegs->fltMask2);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->debug0 != NULL))
  {
        status = Pcie_readDebug0Reg(baseCfgEp, readRegs->debug0);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->debug1 != NULL))
  {
        status = Pcie_readDebug1Reg(baseCfgEp, readRegs->debug1);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->gen2 != NULL))
  {
        status = Pcie_readGen2Reg(baseCfgEp, readRegs->gen2);
  }

  /* hw this revision PLCONF registers */
  if ((status == SystemP_SUCCESS) && (readRegs->plconfObnpSubreqCtrl != NULL))
  {
        status = Pcie_readPlconfObnpSubreqCtrlReg(baseCfgEp, readRegs->plconfObnpSubreqCtrl);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfTrPStsR != NULL))
  {
        status = Pcie_readPlconfTrPStsRReg(baseCfgEp, readRegs->plconfTrPStsR);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfTrNpStsR != NULL))
  {
        status = Pcie_readPlconfTrNpStsRReg(baseCfgEp, readRegs->plconfTrNpStsR);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfTrCStsR != NULL))
  {
        status = Pcie_readPlconfTrCStsRReg(baseCfgEp, readRegs->plconfTrCStsR);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfQStsR != NULL))
  {
        status = Pcie_readPlconfQStsRReg(baseCfgEp, readRegs->plconfQStsR);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfVcTrAR1 != NULL))
  {
        status = Pcie_readPlconfVcTrAR1Reg(baseCfgEp, readRegs->plconfVcTrAR1);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfVcTrAR2 != NULL))
  {
        status = Pcie_readPlconfVcTrAR2Reg(baseCfgEp, readRegs->plconfVcTrAR2);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfVc0PrQC != NULL))
  {
        status = Pcie_readPlconfVcPrQCReg(baseCfgEp, readRegs->plconfVc0PrQC, 0);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfVc0NprQC != NULL))
  {
        status = Pcie_readPlconfVcNprQCReg(baseCfgEp, readRegs->plconfVc0NprQC, 0);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfVc0CrQC != NULL))
  {
        status = Pcie_readPlconfVcCrQCReg(baseCfgEp, readRegs->plconfVc0CrQC, 0);
  }
  for (i = 0; i < 3; i++)
  {
        if ((status == SystemP_SUCCESS) && (readRegs->plconfVcPrQC[i] != NULL))
        {
            status = Pcie_readPlconfVcPrQCReg(baseCfgEp, readRegs->plconfVcPrQC[i], 1 + i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->plconfVcNprQC[i] != NULL))
        {
            status = Pcie_readPlconfVcNprQCReg(baseCfgEp, readRegs->plconfVcNprQC[i], 1 + i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->plconfVcCrQC[i] != NULL))
        {
            status = Pcie_readPlconfVcCrQCReg(baseCfgEp, readRegs->plconfVcCrQC[i], 1 + i);
        }
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfPhyStsR != NULL))
  {
        status = Pcie_readPlconfPhyStsRReg(baseCfgEp, readRegs->plconfPhyStsR);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfPhyCtrlR != NULL))
  {
        status = Pcie_readPlconfPhyCtrlRReg(baseCfgEp, readRegs->plconfPhyCtrlR);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlAddress != NULL))
  {
        status = Pcie_readPlconfMsiCtrlAddressReg(baseCfgEp, readRegs->plconfMsiCtrlAddress);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlUpperAddress != NULL))
  {
        status = Pcie_readPlconfMsiCtrlUpperAddressReg(baseCfgEp, readRegs->plconfMsiCtrlUpperAddress);
  }
  for (i = 0; i < 8; i++)
  {
        if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlIntEnable[i] != NULL))
        {
            status = Pcie_readPlconfMsiCtrlIntEnableReg(baseCfgEp, readRegs->plconfMsiCtrlIntEnable[i], i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlIntMask[i] != NULL))
        {
            status = Pcie_readPlconfMsiCtrlIntMaskReg(baseCfgEp, readRegs->plconfMsiCtrlIntMask[i], i);
        }
        if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlIntStatus[i] != NULL))
        {
            status = Pcie_readPlconfMsiCtrlIntStatusReg(baseCfgEp, readRegs->plconfMsiCtrlIntStatus[i], i);
        }
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfMsiCtrlGpio != NULL))
  {
        status = Pcie_readPlconfMsiCtrlGpioReg(baseCfgEp, readRegs->plconfMsiCtrlGpio);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfPipeLoopback != NULL))
  {
        status = Pcie_readPlconfPipeLoopbackReg(baseCfgEp, readRegs->plconfPipeLoopback);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfDbiRoWrEn != NULL))
  {
        status = Pcie_readPlconfDbiRoWrEnReg(baseCfgEp, readRegs->plconfDbiRoWrEn);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfAxiSlvErrResp != NULL))
  {
        status = Pcie_readPlconfAxiSlvErrRespReg(baseCfgEp, readRegs->plconfAxiSlvErrResp);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfAxiSlvTimeout != NULL))
  {
        status = Pcie_readPlconfAxiSlvTimeoutReg(baseCfgEp, readRegs->plconfAxiSlvTimeout);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuIndex != NULL))
  {
        /* Return the simulated window address */
        *readRegs->plconfIatuIndex = *simIatuWindow;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegCtrl1 != NULL))
  {
        status = Pcie_readPlconfIatuRegCtrl1Reg(baseCfgEp, simIatuWindow, readRegs->plconfIatuRegCtrl1);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegCtrl2 != NULL))
  {
        status = Pcie_readPlconfIatuRegCtrl2Reg(baseCfgEp, simIatuWindow, readRegs->plconfIatuRegCtrl2);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegLowerBase != NULL))
  {
        status = Pcie_readPlconfIatuRegLowerBaseReg(baseCfgEp, simIatuWindow, readRegs->plconfIatuRegLowerBase);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegUpperBase != NULL))
  {
        status = Pcie_readPlconfIatuRegUpperBaseReg(baseCfgEp, simIatuWindow, readRegs->plconfIatuRegUpperBase);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegLimit != NULL))
  {
        status = Pcie_readPlconfIatuRegLimitReg(baseCfgEp, simIatuWindow, readRegs->plconfIatuRegLimit);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegLowerTarget != NULL))
  {
        status = Pcie_readPlconfIatuRegLowerTargetReg(baseCfgEp, simIatuWindow, readRegs->plconfIatuRegLowerTarget);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegUpperTarget != NULL))
  {
        status = Pcie_readPlconfIatuRegUpperTargetReg(baseCfgEp, simIatuWindow, readRegs->plconfIatuRegUpperTarget);
  }
  if ((status == SystemP_SUCCESS) && (readRegs->plconfIatuRegCtrl3 != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }

  /* TI CONF registers */
  /* reject hw this revision TI CONF registers */
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfRevision != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfSysConfig != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEoi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqStatusRawMain != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqStatusMain != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEnableSetMain != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEnableClrMain != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqStatusRawMsi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqStatusMsi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEnableSetMsi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIrqEnableClrMsi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfDeviceType != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfDeviceCmd != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfPmCtrl != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfPhyCs != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIntxAssert != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfIntxDeassert != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfMsiXmt != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfDebugCfg != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfDebugData != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (readRegs->tiConfDiagCtrl != NULL))
  {
        /* Not supported on this revision */
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
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_RX_LANE_FLIP_EN, swReg->rxLaneFlipEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_TX_LANE_FLIP_EN, swReg->txLaneFlipEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_DBI_CS2, swReg->dbi);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_APP_RETRY_EN, swReg->appRetryEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_LTSSM_EN, swReg->ltssmEn);

        baseAddr->CMD_STATUS = swReg->raw = new_val;
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
static int32_t Pcie_writeEpIrqSetReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_EpIrqSetReg * swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_SET_LEGACY_IRQ_SET_0, swReg->epIrqSet);

        baseAddr->LEGACY_IRQ_SET = swReg->raw = new_val;
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
static int32_t Pcie_writeEpIrqClrReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_EpIrqClrReg * swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_CLR_LEGACY_IRQ_CLR_0, swReg->epIrqClr);

        baseAddr->LEGACY_IRQ_CLR = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeEpIrqClrReg */

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

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_IO_EN, swReg->ioSp);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_MEM_SPACE_EN, swReg->memSp);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_BUS_MASTER_EN, swReg->busMs);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_SPECIAL_CYCLE_OPERATION, swReg->specCycleEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_MWI_ENABLE, swReg->memWrInva);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_VGA_PALETTE_SNOOP, swReg->vgaSnoop);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_PARITY_ERR_EN, swReg->resp);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_IDSEL_STEPPING, swReg->idselCtrl);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_SERREN, swReg->serrEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_INT_EN, swReg->dis);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_INT_STATUS, swReg->stat);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_CAP_LIST, swReg->capList);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_FAST_66MHZ_CAP, swReg->c66MhzCap);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_FAST_B2B_CAP, swReg->fastB2B);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_MASTER_DPE, swReg->parError);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_DEV_SEL_TIMING, swReg->devSelTime);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_SIGNALED_TARGET_ABORT, swReg->sigTgtAbort);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_RCVD_TARGET_ABORT, swReg->tgtAbort);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_RCVD_MASTER_ABORT, swReg->mstAbort);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_SIGNALED_SYS_ERR, swReg->sysError);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_DETECTED_PARITY_ERR, swReg->parity);

        *hwReg_STATUS_COMMAND_REGISTER = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeStatusCmdReg */

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

        PCIE_SETBITS(new_val, PCIE_REV2_CLASSCODE, swReg->classCode);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_REVISION_ID, swReg->revId);

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

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BAR_REG_BAR_MEM_IO, swReg->memSpace);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BAR_REG_BAR_TYPE, swReg->type);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BAR_REG_BAR_PREFETCH, swReg->prefetch);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BAR_REG_BAR_START, swReg->base);

        baseAddr->BAR_REG[barNum] = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeType0BarReg */

/*****************************************************************************
 * Combine and write the BAR 32bits register
 ****************************************************************************/
static int32_t Pcie_writeType0Bar32bitReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_Bar32bitReg *swReg,
                        int32_t barNum)
{
    int32_t status = SystemP_SUCCESS;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        swReg->reg32 = swReg->raw = baseAddr->BAR_REG[barNum];
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeType0Bar32bitReg */

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

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_REPORT_EN, swReg->corErRp);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_REPORT_EN, swReg->nFatalErRp);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_REPORT_EN, swReg->fatalErRp);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORT_REQ_REP_EN, swReg->reqRp);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_REL_ORDER, swReg->relaxed);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_PAYLOAD_SIZE_CS, swReg->maxPayld);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EXT_TAG_EN, swReg->xtagEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_PHANTOM_FUNC_EN, swReg->phantomEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_PM_EN, swReg->auxPwrEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_NO_SNOOP, swReg->noSnoop);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_READ_REQ_SIZE, swReg->maxSz);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_INITIATE_FLR, swReg->initFLR);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_DETECTED, swReg->corrEr);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_DETECTED, swReg->nFatalEr);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_DETECTED, swReg->fatalEr);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORTED_REQ_DETECTED, swReg->rqDet);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_DETECTED, swReg->auxPwr);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_TRANS_PENDING, swReg->tpend);

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
static int32_t Pcie_writeLinkCapReg (volatile uint32_t *hwReg_LNK_CAP, Pcie_LinkCapReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((hwReg_LNK_CAP != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_SPEED, swReg->maxLinkSpeed);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_WIDTH, swReg->maxLinkWidth);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT, swReg->asLinkPm);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L0S_EXIT_LATENCY, swReg->losExitLat);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L1_EXIT_LATENCY, swReg->l1ExitLat);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_CLOCK_POWER_MAN, swReg->clkPwrMgmt);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_SURPRISE_DOWN_ERR_REP_CAP, swReg->downErrRepCap);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_DLL_ACTIVE_REP_CAP, swReg->dllRepCap);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_LINK_BW_NOT_CAP, swReg->bwNotifyCap);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ASPM_OPT_COMPLIANCE, swReg->aspmOptComp);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_PORT_NUM, swReg->portNum);

        *hwReg_LNK_CAP = swReg->raw = new_val;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
} /* Pcie_writeLinkCapReg */

/*****************************************************************************
 * Combine and write the Advanced Capabilities and Control register
 ****************************************************************************/
static int32_t Pcie_writeAccrReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_AccrReg *swReg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t new_val;

    if ((baseAddr != NULL) && (swReg != NULL))
    {
        new_val = swReg->raw;

        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_EN, swReg->multHdrEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_CAP, swReg->multHdrCap);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_EN, swReg->chkEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_CAP, swReg->chkCap);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_EN, swReg->genEn);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_CAP, swReg->genCap);
        PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_FIRST_ERR_POINTER, swReg->erPtr);

        baseAddr->ADV_ERR_CAP_CTRL_OFF = swReg->raw = new_val;
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

        if (regionIndex < 16)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_RW, swReg->iatuRegLowerBase);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_HW, swReg->zero);

                swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_BASE_ADDR_OFF_OUTBOUND = new_val;
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_RW, swReg->iatuRegLowerBase);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_HW, swReg->zero);

                swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_BASE_ADDR_OFF_INBOUND = new_val;

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

        if (regionIndex < OB_REGION_MAX)
        {
            if (simIatuWindow->regionDirection == 0U)
            {
                /* 0U == OUTBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_UPPER_BASE_RW, swReg->iatuRegUpperBase);
                swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_BASE_ADDR_OFF_OUTBOUND = new_val;
            }
            else if (simIatuWindow->regionDirection == 1U)
            {
                /* INBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_UPPER_BASE_RW, swReg->iatuRegUpperBase);
                swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_BASE_ADDR_OFF_INBOUND = new_val;
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
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_LWR_TARGET_RW_OUTBOUND,
                             (swReg->iatuRegLowerTarget << 16) | (swReg->zero & 0xffffu));
                swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_TARGET_ADDR_OFF_OUTBOUND = new_val;
            }
            else
            {
                /* INBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_RW, swReg->iatuRegLowerTarget);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_HW, swReg->zero);

                swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_TARGET_ADDR_OFF_INBOUND = new_val;
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
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_UPPER_TARGET_RW, swReg->iatuRegUpperTarget);
                swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND = new_val;
            }
            else
            {
                /* INBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_UPPER_TARGET_RW, swReg->iatuRegUpperTarget);

                swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_TARGET_ADDR_OFF_INBOUND = new_val;
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
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TYPE, swReg->type);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TC, swReg->tc);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TD, swReg->td);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_ATTR, swReg->attr);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_INCREASE_REGION_SIZE, swReg->increaseRegionSize);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_CTRL_1_FUNC_NUM, swReg->functionNumber);

                swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_1_OFF_OUTBOUND = new_val;
            }
            else
            {
                /* INBOUND */
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TYPE, swReg->type);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TC, swReg->tc);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TD, swReg->td);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_ATTR, swReg->attr);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_INCREASE_REGION_SIZE, swReg->increaseRegionSize);
                PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_CTRL_1_FUNC_NUM, swReg->functionNumber);

                swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_1_OFF_INBOUND = new_val;
            }

        }
         else
        {
            status = SystemP_FAILURE;
        }
    }
    return status;
} /* Pcie_writePlconfIatuRegCtrl1Reg */

/* Write MSI capabilities Register */
/*****************************************************************************
 * Combine and write the Message Signaled Interrupt Capability register
 ****************************************************************************/
static int32_t Pcie_writeMsiCapReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiCapReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_ID, swReg->capId);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_NEXT_OFFSET, swReg->nextCap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_ENABLE, swReg->msiEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_CAP, swReg->multMsgCap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_EN, swReg->multMsgEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_64_BIT_ADDR_CAP, swReg->en64bit);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_PVM_SUPPORT, swReg->pvmEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_CAP, swReg->extDataCap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_EN, swReg->extDataEn);

    baseAddr->PCI_MSI_CAP_ID_NEXT_CTRL_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_writeMsiCapReg */

/*****************************************************************************
 * Combine and write the Address of MSI write TLP req lower 32 bits register
 ****************************************************************************/
static int32_t Pcie_writeMsiLo32Reg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiLo32Reg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_04H_REG_PCI_MSI_CAP_OFF_04H, swReg->addr);

    baseAddr->MSI_CAP_OFF_04H_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_writeMsiLo32Reg */

/*****************************************************************************
 * Combine and write the Address of MSI write TLP req upper 32 bits register
 ****************************************************************************/
static int32_t Pcie_writeMsiUp32Reg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiUp32Reg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_08H, swReg->addr);

    baseAddr->MSI_CAP_OFF_08H_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_writeMsiUp32Reg */

/*****************************************************************************
 * Combine and write the Data of MSI write TLP req register
 ****************************************************************************/
static int32_t Pcie_writeMsiDataReg (CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiDataReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0CH, swReg->data);

    baseAddr->MSI_CAP_OFF_0CH_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_writeMsiDataReg */

/*****************************************************************************
 * Combine and write the Slot Capabilities register
 ****************************************************************************/
static int32_t Pcie_writeSlotCapReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_SlotCapReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_BUTTON, swReg->attnButton);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_CONTROLLER, swReg->pwrCtl);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_MRL_SENSOR, swReg->mrlSensor);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR, swReg->attnInd);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_INDICATOR, swReg->pwrInd);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_SURPRISE, swReg->hpSurprise);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_CAPABLE, swReg->hpCap);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_VALUE, swReg->pwrLmtValue);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_SCALE, swReg->pwrLmtScale);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ELECTROMECH_INTERLOCK, swReg->emlPresent);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_NO_CMD_CPL_SUPPORT, swReg->cmdCompSupp);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_PHY_SLOT_NUM, swReg->slotNum);

    baseAddr->SLOT_CAPABILITIES_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_slotCap_reg */

/*****************************************************************************
 * Combine and write the Slot Status and Control register
 ****************************************************************************/
static int32_t Pcie_writeSlotStatCtrlReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_SlotStatCtrlReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_EN, swReg->attnButtEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_EN, swReg->pwrFltDetEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_EN, swReg->mrlChgEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_CHANGE_EN, swReg->prsDetChgEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPL_INT_EN, swReg->cmdCmpIntEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_HOT_PLUG_INT_EN, swReg->hpIntEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_INDICATOR_CTRL, swReg->attnIndCtl);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_INDICATOR_CTRL, swReg->pmIndCtl);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_CONTROLLER_CTRL, swReg->pmCtl);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_CTRL, swReg->emLockCtl);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_EN, swReg->dllChgEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED, swReg->attnPressed);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED, swReg->pwrFault);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED, swReg->mrlChange);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECTED_CHANGED, swReg->presenceChg);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPLD, swReg->cmdComplete);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_STATE, swReg->mrlState);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_STATE, swReg->presenceDet);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_STATUS, swReg->emLock);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED, swReg->dllState);
    baseAddr->SLOT_CONTROL_SLOT_STATUS = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_slotStatCtrl_reg */

/*****************************************************************************
 * Combine and write the Root Control and Capabilities register
 ****************************************************************************/
static int32_t Pcie_writeRootCtrlCapReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_RootCtrlCapReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN, swReg->serrEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN, swReg->serrNFatalErr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN, swReg->serrFatalErr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_PME_INT_EN, swReg->pmeIntEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_EN, swReg->crsSwEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY, swReg->crsSw);

    baseAddr->ROOT_CONTROL_ROOT_CAPABILITIES_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_rootCtrlCap_reg */

/*****************************************************************************
 * Combine and write the Root Status and Control register
 ****************************************************************************/
static int32_t Pcie_writeRootStatusReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_RootStatusReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_REQ_ID, swReg->pmeReqID);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_STATUS, swReg->pmeStatus);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_PENDING, swReg->pmePend);

    baseAddr->ROOT_STATUS_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Device Capabilities 2 register
 ****************************************************************************/
static int32_t Pcie_writeDevCap2Reg(volatile uint32_t *hwReg_DEV_CAP_2, Pcie_DevCap2Reg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_RANGE, swReg->cmplToEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_SUPPORT, swReg->cmplToDisSupp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT, swReg->ariFwdSp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ATOMIC_ROUTING_SUPP, swReg->aorSp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_32_ATOMIC_CPL_SUPP, swReg->aoc32Sp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_64_ATOMIC_CPL_SUPP, swReg->aoc64Sp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_128_CAS_CPL_SUPP, swReg->casc128Sp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_NO_RO_EN_PR2PR_PAR, swReg->noRoPR);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_LTR_SUPP, swReg->ltrSupp);
    PCIE_SETBITS(new_val, PCIE_REV2_TPH_CMPLT_SUPPORT, swReg->tphcSp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_LN_SYS_CLS, swReg->lnSysCls);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_COMP_SUPPORT, swReg->tag10bitCompSupp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_REQ_SUPPORT, swReg->tag10bitReqSupp);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_OBFF_SUPPORT, swReg->obffSupp);

    *hwReg_DEV_CAP_2 = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_devCap2_reg */

/*****************************************************************************
 * Combine and write the Device Status and Control Register 2 register
 ****************************************************************************/
static int32_t Pcie_writeDevStatCtrl2Reg(volatile uint32_t *hwReg_DEV_CAS_2, Pcie_DevStatCtrl2Reg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_VALUE, swReg->cmplTo);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE, swReg->cmplToDis);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_CS, swReg->ariFwdSp);

    *hwReg_DEV_CAS_2 = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_devStatCtrl2_reg */

/*****************************************************************************
 * Combine and write the Link Capabilites 2 register
 ****************************************************************************/
static int32_t Pcie_writeLinkCap2Reg(volatile uint32_t *hwReg_LNK_CAP_2, Pcie_LnkCap2Reg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_SUPPORT_LINK_SPEED_VECTOR, swReg->spLsVec);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_CROSS_LINK_SUPPORT, swReg->crosslinkSp);

    *hwReg_LNK_CAP_2 = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_linkCap2_reg */

/*****************************************************************************
 * Combine and write the Link Control 2 register
 ****************************************************************************/
static int32_t Pcie_writeLinkCtrl2Reg(volatile uint32_t *hwReg_LNK_CAS_2, Pcie_LinkCtrl2Reg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TARGET_LINK_SPEED, swReg->tgtSpeed);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_COMPLIANCE, swReg->entrCompl);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_HW_AUTO_SPEED_DISABLE, swReg->hwAutoSpeedDis);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_SEL_DEEMPHASIS, swReg->selDeemph);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TX_MARGIN, swReg->txMargin);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_MODIFIED_COMPLIANCE, swReg->entrModCompl);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_SOS, swReg->cmplSos);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_PRESET, swReg->complPrstDeemph);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_CURR_DEEMPHASIS, swReg->deEmph);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL, swReg->eqComplete);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P1, swReg->eqPh1);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P2, swReg->eqPh2);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P3, swReg->eqPh3);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_LINK_EQ_REQ, swReg->linkEqReq);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DOWNSTREAM_COMPO_PRESENCE, swReg->downCompPres);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DRS_MESSAGE_RECEIVED, swReg->drsMsgRecv);

    *hwReg_LNK_CAS_2 = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Uncorrectable Error Status register
 ****************************************************************************/
static int32_t Pcie_writeUncErrReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_UncErrReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_TLP_PRFX_BLOCKED_ERR_STATUS, swReg->tlpPrfxBlockedErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_INTERNAL_ERR_STATUS, swReg->intErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_UNSUPPORTED_REQ_ERR_STATUS, swReg->urErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_ECRC_ERR_STATUS, swReg->ecrcErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_MALF_TLP_ERR_STATUS, swReg->mtlpErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_REC_OVERFLOW_ERR_STATUS, swReg->rcvrOfSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_UNEXP_CMPLT_ERR_STATUS, swReg->ucmpSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_ABORT_ERR_STATUS, swReg->cmplAbrtSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_TIMEOUT_ERR_STATUS, swReg->cmplTmotSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_FC_PROTOCOL_ERR_STATUS, swReg->fcpErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_POIS_TLP_ERR_STATUS, swReg->psndTlpSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_SURPRISE_DOWN_ERR_STATUS, swReg->srpsDnSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_DL_PROTOCOL_ERR_STATUS, swReg->dlpErrSt);

    baseAddr->UNCORR_ERR_STATUS_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_uncErr_reg */

/*****************************************************************************
 * Combine and write the Uncorrectable Error Mask register
 ****************************************************************************/
static int32_t Pcie_writeUncErrMaskReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_UncErrMaskReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_TLP_PRFX_BLOCKED_ERR_MASK, swReg->tlpPrfxBlockedErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_ATOMIC_EGRESS_BLOCKED_ERR_MASK, swReg->atomicEgressBlockedErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_INTERNAL_ERR_MASK, swReg->intErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_UNSUPPORTED_REQ_ERR_MASK, swReg->urErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_ECRC_ERR_MASK, swReg->ecrcErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_MALF_TLP_ERR_MASK, swReg->mtlpErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_REC_OVERFLOW_ERR_MASK, swReg->rcvrOfMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_UNEXP_CMPLT_ERR_MASK, swReg->ucmpMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_CMPLT_ABORT_ERR_MASK, swReg->cmplAbrtMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_CMPLT_TIMEOUT_ERR_MASK, swReg->cmplTmotMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_FC_PROTOCOL_ERR_MASK, swReg->fcpErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_POIS_TLP_ERR_MASK, swReg->psndTlpMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_SURPRISE_DOWN_ERR_MASK, swReg->srpsDnMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_DL_PROTOCOL_ERR_MASK, swReg->dlpErrMsk);

    baseAddr->UNCORR_ERR_MASK_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_uncErrMask_reg */

/*****************************************************************************
 * Combine and write the Uncorrectable Error Severity register
 ****************************************************************************/
static int32_t Pcie_writeUncErrSvrtyReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_UncErrSvrtyReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_TLP_PRFX_BLOCKED_ERR_SEVERITY, swReg->tlpPrfxBlockedErrSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_ATOMIC_EGRESS_BLOCKED_ERR_SEVERITY, swReg->atomicEgressBlockedErrSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_INTERNAL_ERR_SEVERITY, swReg->intErrSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_UNSUPPORTED_REQ_ERR_SEVERITY, swReg->urErrSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_ECRC_ERR_SEVERITY, swReg->ecrcErrSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_MALF_TLP_ERR_SEVERITY, swReg->mtlpErrSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_REC_OVERFLOW_ERR_SEVERITY, swReg->rcvrOfSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_UNEXP_CMPLT_ERR_SEVERITY, swReg->ucmpSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_CMPLT_ABORT_ERR_SEVERITY, swReg->cmplAbrtSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_CMPLT_TIMEOUT_ERR_SEVERITY, swReg->cmplTmotSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_FC_PROTOCOL_ERR_SEVERITY, swReg->fcpErrSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_POIS_TLP_ERR_SEVERITY, swReg->psndTlpSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_SURPRISE_DOWN_ERR_SVRITY, swReg->srpsDnSvrty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_DL_PROTOCOL_ERR_SEVERITY, swReg->dlpErrSvrty);

    baseAddr->UNCORR_ERR_SEV_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_uncErrSvrty_reg */

/*****************************************************************************
 * Combine and write the Correctable Error Status register
 ****************************************************************************/
static int32_t Pcie_writeCorErrReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_CorErrReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_HEADER_LOG_OVERFLOW_STATUS, swReg->hdrLogOverflowErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_CORRECTED_INT_ERR_STATUS, swReg->corrIntErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_ADVISORY_NON_FATAL_ERR_STATUS, swReg->advNFErrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_RPL_TIMER_TIMEOUT_STATUS, swReg->rplyTmrSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_REPLAY_NO_ROLEOVER_STATUS, swReg->rpltRoSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_BAD_DLLP_STATUS, swReg->badDllpSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_BAD_TLP_STATUS, swReg->badTlpSt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_RX_ERR_STATUS, swReg->rcvrErrSt);

    baseAddr->CORR_ERR_STATUS_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_corErr_reg */

/*****************************************************************************
 * Combine and write the Correctable Error Mask register
 ****************************************************************************/
static int32_t Pcie_writeCorErrMaskReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_CorErrMaskReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_HEADER_LOG_OVERFLOW_MASK, swReg->hdrLogOverflowErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_CORRECTED_INT_ERR_MASK, swReg->corrIntErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_ADVISORY_NON_FATAL_ERR_MASK, swReg->advNFErrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_RPL_TIMER_TIMEOUT_MASK, swReg->rplyTmrMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_REPLAY_NO_ROLEOVER_MASK, swReg->rpltRoMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_BAD_DLLP_MASK, swReg->badDllpMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_BAD_TLP_MASK, swReg->badTlpMsk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_RX_ERR_MASK, swReg->rcvrErrMsk);

    baseAddr->CORR_ERR_MASK_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Root Error Command register
 ****************************************************************************/
static int32_t Pcie_writeRootErrCmdReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_RootErrCmdReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_FATAL_ERR_REPORTING_EN, swReg->ferrRptEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_NON_FATAL_ERR_REPORTING_EN, swReg->nferrRptEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_CORR_ERR_REPORTING_EN, swReg->cerrRptEn);

    baseAddr->ROOT_ERR_CMD_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_rootErrCmd_reg */

/*****************************************************************************
 * Combine and write the Root Error Status register
 ****************************************************************************/
static int32_t Pcie_writeRootErrStReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_RootErrStReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ADV_ERR_INT_MSG_NUM, swReg->aerIntMsg);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FATAL_ERR_MSG_RX, swReg->ferrRcv);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_NON_FATAL_ERR_MSG_RX, swReg->nfErr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FIRST_UNCORR_FATAL, swReg->uncorFatal);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_FATAL_NON_FATAL_RX, swReg->multFnf);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_FATAL_NON_FATAL_RX, swReg->errFnf);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_COR_RX, swReg->multCor);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_COR_RX, swReg->corrErr);

    baseAddr->ROOT_ERR_STATUS_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_rootErrSt_reg */

/*****************************************************************************
 * Combine and write the PL CONF Ack Latency and Replay Timer register
 ****************************************************************************/
static int32_t Pcie_writePlAckTimerReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlAckTimerReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ACK_LATENCY_TIMER_OFF_ROUND_TRIP_LATENCY_TIME_LIMIT, swReg->rndTrpLmt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ACK_LATENCY_TIMER_OFF_REPLAY_TIME_LIMIT, swReg->rplyLmt);

    baseAddr->ACK_LATENCY_TIMER_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plAckTimer_reg */

/*****************************************************************************
 * Combine and write the PL CONF Vendor Specific DLL register
 ****************************************************************************/
static int32_t Pcie_writePlOMsgReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlOMsgReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VENDOR_SPEC_DLLP_OFF_VENDOR_SPEC_DLLP, swReg->oMsg);

    baseAddr->VENDOR_SPEC_DLLP_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plOMsg_reg */

/*****************************************************************************
 * Combine and write the PL CONF Port Force Link register
 ****************************************************************************/
static int32_t Pcie_writePlForceLinkReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlForceLinkReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_LINK_NUM, swReg->linkNum);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_FORCE_EN, swReg->forceLink);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_LINK_STATE, swReg->lnkState);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_FORCED_LTSSM, swReg->forcedLtssmState);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_DO_DESKEW_FOR_SRIS, swReg->doDeskewForSris);

    baseAddr->PORT_FORCE_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plForceLink_reg */

/*****************************************************************************
 * Combine and write the PL CONF Ack Frequency and L0-L1 ASPM register
 ****************************************************************************/
static int32_t Pcie_writeAckFreqReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_AckFreqReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ACK_FREQ, swReg->ackFreq);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ACK_N_FTS, swReg->nFts);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_COMMON_CLK_N_FTS, swReg->commNFts);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_L0S_ENTRANCE_LATENCY, swReg->l0sEntryLatency);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_L1_ENTRANCE_LATENCY, swReg->l1EntryLatency);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ENTER_ASPM, swReg->aspmL1);

    baseAddr->ACK_F_ASPM_CTRL_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_ackFreq_reg */

/*****************************************************************************
 * Combine and write the PL CONF Lane Skew register
 ****************************************************************************/
static int32_t Pcie_writeLaneSkewReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_LaneSkewReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_INSERT_LANE_SKEW, swReg->laneSkew);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_FLOW_CTRL_DISABLE, swReg->fcDisable);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_ACK_NAK_DISABLE, swReg->ackDisable);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_IMPLEMENT_NUM_LANES, swReg->implementNumLanes);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_DISABLE_LANE_TO_LANE_DESKEW, swReg->l2Deskew);

    baseAddr->LANE_SKEW_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_laneSkew_reg */

/*****************************************************************************
 * Combine and write the PL CONF Timer Control and Symbol Number (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_writeSymNumReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_SymNumReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_MAX_FUNC_NUM, swReg->maxFunc);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_REPLAY_TIMER, swReg->replayTimer);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_ACK_NAK, swReg->ackLatencyTimer);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_FAST_LINK_SCALING_FACTOR, swReg->fastLinkScalingFactor);

    baseAddr->TIMER_CTRL_MAX_FUNC_NUM_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_symNum_reg */

/*****************************************************************************
 * Combine and write the PL CONF Symbol Timer and Filter Mask (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_writeSymTimerFltMaskReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_SymTimerFltMaskReg *swReg)
{
    uint32_t new_val = swReg->raw;

    uint32_t mask1 = 0;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_SKP_INT_VAL, swReg->skpValue);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_DISABLE_FC_WD_TIMER, swReg->fcWdogDisable);

    /* Repack into named bits per rev 0, they actually match */
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CFG_DROP, swReg->f1CfgDrop);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_IO_DROP, swReg->f1IoDrop);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_MSG_DROP, swReg->f1MsgDrop);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CPL_ECRC_DROP, swReg->f1CplEcrcDrop);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_ECRC_DROP, swReg->f1EcrcDrop);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CPL_LEN_TEST, swReg->f1CplLenTest);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CPL_ATTR_TEST, swReg->f1CplAttrTest);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CPL_TC_TEST, swReg->f1CplTcTest);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CPL_FUNC_TEST, swReg->f1CplFuncTest);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CPL_REQID_TEST, swReg->f1CplReqIDTest);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CPL_TAGERR_TEST, swReg->f1CplTagErrTest);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_LOCKED_RD_AS_UR, swReg->f1LockedRdAsUr);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_CFG1_RE_AS_US, swReg->f1Cfg1ReAsUs);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_UR_OUT_OF_BAR, swReg->f1UrOutOfBar);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_UR_POISON, swReg->f1UrPoison);
    PCIE_SETBITS(mask1, PCIE_FLTMASK1_UR_FUN_MISMATCH, swReg->f1UrFunMismatch);

    /* Put mask into register image */
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_MASK_RADM_1, mask1);

    baseAddr->SYMBOL_TIMER_FILTER_1_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfSymbTR_reg */

/*****************************************************************************
 * Combine and write the PL CONF Filter Mask 2 register
 ****************************************************************************/
static int32_t Pcie_writeFltMask2Reg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_FltMask2Reg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, PCIE_FLTMASK2_DROP_PRS, swReg->dropPRS);
    PCIE_SETBITS(new_val, PCIE_FLTMASK2_UNMASK_TD, swReg->unmaskTD);
    PCIE_SETBITS(new_val, PCIE_FLTMASK2_UNMASK_UR_POIS, swReg->unmaskUrPOIS);
    PCIE_SETBITS(new_val, PCIE_FLTMASK2_DROP_LN, swReg->dropLN);
    PCIE_SETBITS(new_val, PCIE_FLTMASK2_FLUSH_REQ, swReg->flushReq);
    PCIE_SETBITS(new_val, PCIE_FLTMASK2_DLLP_ABORT, swReg->dllpAbort);
    PCIE_SETBITS(new_val, PCIE_FLTMASK2_VMSG1_DROP, swReg->vmsg1Drop);
    PCIE_SETBITS(new_val, PCIE_FLTMASK2_VMSG0_DROP, swReg->vmsg0Drop);

    baseAddr->FILTER_MASK_2_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the PL CONF Link Width and Speed Change Control (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_writeGen2Reg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_Gen2Reg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_FAST_TRAINING_SEQ, swReg->numFts);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_NUM_OF_LANES, swReg->lnEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_PRE_DET_LANE, swReg->preDetLane);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_AUTO_LANE_FLIP_CTRL_EN, swReg->autoFlipEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE, swReg->dirSpd);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_CONFIG_PHY_TX_CHANGE, swReg->txSwing);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_CONFIG_TX_COMP_RX, swReg->txCmpl);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_SEL_DEEMPHASIS, swReg->deemph);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_GEN1_EI_INFERENCE, swReg->gen1EiInference);

    baseAddr->GEN2_CTRL_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the PL CONF AXI Multiple Outbound Decomposed NP
 * SubRequests Control Register (Sticky) register
 ****************************************************************************/
static int32_t Pcie_writePlconfObnpSubreqCtrlReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfObnpSubreqCtrlReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_OB_RD_SPLIT_BURST_EN, swReg->enObnpSubreq);

    baseAddr->AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfObnpSubreqCtrl_reg */

/*****************************************************************************
 * Combine and write the PL CONF Queue Status (Sticky) register
 ****************************************************************************/
static int32_t Pcie_writePlconfQStsRReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfQStsRReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_TLP_FC_CREDIT_NON_RETURN, swReg->crdtNotRtrn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TX_RETRY_BUFFER_NE, swReg->rtybNotEmpty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_QUEUE_NON_EMPTY, swReg->rcvqNotEmpty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_QUEUE_OVERFLOW, swReg->rxQueueOverflow);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_NON_EMPTY, swReg->rxSerQNEmpty);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_WRITE_ERR, swReg->rxSerQWErr);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_READ_ERR, swReg->rxSerRErr);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL, swReg->fcLatencyOvr);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_EN, swReg->fcLatencyOvrEn);

    baseAddr->QUEUE_STATUS_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfQStsR_reg */

/*****************************************************************************
 * Combine and write the Link Status and Control register
 ****************************************************************************/
static int32_t Pcie_writeLinkStatCtrlReg(volatile uint32_t *hwReg_LNK_CAS, Pcie_LinkStatCtrlReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL, swReg->activeLinkPm);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RCB, swReg->rcb);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_DISABLE, swReg->linkDisable);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RETRAIN_LINK, swReg->retrainLink);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_COMMON_CLK_CONFIG, swReg->commonClkCfg);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EXTENDED_SYNCH, swReg->extSync);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EN_CLK_POWER_MAN, swReg->clkPwrMgmtEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_HW_AUTO_WIDTH_DISABLE, swReg->hwAutoWidthDis);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_INT_EN, swReg->linkBwMgmtIntEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_INT_EN, swReg->linkBwIntEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DRS_SIGNALING_CONTROL, swReg->drsSigCtrl);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_SPEED, swReg->linkSpeed);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_NEGO_LINK_WIDTH, swReg->negotiatedLinkWd);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_TRAINING, swReg->linkTraining);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_SLOT_CLK_CONFIG, swReg->slotClkCfg);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DLL_ACTIVE, swReg->dllActive);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_STATUS, swReg->linkBwMgmtStatus);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_STATUS, swReg->linkBwStatus);

    *hwReg_LNK_CAS = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* pciev2_write_linkStatCtrl_reg */

/*****************************************************************************
 * Combine and write the PL CONF VC0 Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_writePlconfVcPrQCReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfVcPrQCReg *swReg, int32_t vcNum)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_CREDIT, swReg->pDcrd);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HEADER_CREDIT, swReg->pHcrd);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_SCALE, swReg->pDataScale);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HDR_SCALE, swReg->pHdrScale);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC, swReg->orderingRules);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q, swReg->strictVcPriority);

    /* Note: not checking vcNum as its internally generated */
    baseAddr->VC_RX_Q_CTRL[vcNum].VC_P_RX_Q_CTRL_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfVcPrQC_reg */

/*****************************************************************************
 * Combine and write the PL CONF VC0 Non-Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_writePlconfVcNprQCReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfVcNprQCReg *swReg, int32_t vcNum)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_CREDIT, swReg->npDcrd);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HEADER_CREDIT, swReg->npHcrd);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_SCALE, swReg->npDataScale);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HDR_SCALE, swReg->npHdrScale);

    /* Note: not checking vcNum as its internally generated */
    baseAddr->VC_RX_Q_CTRL[vcNum].VC_NP_RX_Q_CTRL_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfVcNprQC_reg */

/*****************************************************************************
 * Combine and write the PL CONF VC0 Completion Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_writePlconfVcCrQCReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfVcCrQCReg *swReg, int32_t vcNum)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_CREDIT, swReg->cplDcrd);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HEADER_CREDIT, swReg->cplHcrd);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_SCALE, swReg->cplDataScale);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HDR_SCALE, swReg->cplHdrScale);

    /* Note: not checking vcNum as its internally generated */
    baseAddr->VC_RX_Q_CTRL[vcNum].VC_CPL_RX_Q_CTRL_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the PL CONF PHY Control (Sticky) register
 ****************************************************************************/
static int32_t Pcie_writePlconfPhyCtrlRReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfPhyCtrlRReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PHY_CONTROL_OFF_PHY_CONTROL, swReg->phyCtrl);

    baseAddr->PHY_CONTROL_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfPhyCtrlR_reg */

/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Address (RC-mode MSI receiver)
 * register
 ****************************************************************************/
static int32_t Pcie_writePlconfMsiCtrlAddressReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlAddressReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_ADDR_OFF_MSI_CTRL_ADDR, swReg->msiCtrlAddress);

    baseAddr->MSI_CTRL_ADDR_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfMsiCtrlAddress_reg */

/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Upper Address
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_writePlconfMsiCtrlUpperAddressReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlUpperAddressReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_UPPER_ADDR_OFF_MSI_CTRL_UPPER_ADDR, swReg->msiCtrlUpperAddress);

    baseAddr->MSI_CTRL_UPPER_ADDR_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfMsiCtrlUpperAddress_reg */

/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Enable
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_writePlconfMsiCtrlIntEnableReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlIntEnableReg *swReg, int32_t n)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_EN_OFF_MSI_CTRL_INT_EN, swReg->msiCtrlIntEnable);

    baseAddr->MSI_CTRL[n].MSI_CTRL_INT_EN_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfMsiCtrlIntEnable_reg */

/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Mask
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_writePlconfMsiCtrlIntMaskReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlIntMaskReg *swReg, int32_t n)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_MASK_OFF_MSI_CTRL_INT_MASK, swReg->msiCtrlIntMask);

    baseAddr->MSI_CTRL[n].MSI_CTRL_INT_MASK_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfMsiCtrlIntMask_reg */

/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Status
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_writePlconfMsiCtrlIntStatusReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlIntStatusReg *swReg, int32_t n)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_STATUS_OFF_MSI_CTRL_INT_STATUS, swReg->msiCtrlIntStatus);

    baseAddr->MSI_CTRL[n].MSI_CTRL_INT_STATUS_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfMsiCtrlIntStatus_reg */

/*****************************************************************************
 * Combine and write the PL CONF MSI Controller General Purpose IO
 * (RC-mode MSI receiver) register
 ****************************************************************************/
static int32_t Pcie_writePlconfMsiCtrlGpioReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfMsiCtrlGpioReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_GPIO_IO_OFF_MSI_GPIO_REG, swReg->msiCtrlGpio);

    baseAddr->MSI_GPIO_IO_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfMsiCtrlGpio_reg */

/*****************************************************************************
 * Combine and write the PL CONF PIPE loopback control (Sticky) register
 ****************************************************************************/
static int32_t Pcie_writePlconfPipeLoopbackReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfPipeLoopbackReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PIPE_LOOPBACK_CONTROL_OFF_PIPE_LOOPBACK, swReg->loopbackEn);

    baseAddr->PIPE_LOOPBACK_CONTROL_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfPipeLoopback_reg */

/*****************************************************************************
 * Combine and write the PL CONF DIF Read-Only register Write Enable (Sticky)
 * register
 ****************************************************************************/
static int32_t Pcie_writePlconfDbiRoWrEnReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfDbiRoWrEnReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_DBI_RO_WR_EN, swReg->cxDbiRoWrEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_DEFAULT_TARGET, swReg->defaultTarget);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_UR_CA_MASK_4_TRGT1, swReg->urCaMask4Trgt1);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_SIMPLIFIED_REPLAY_TIMER, swReg->simpReplayTimer);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_RSVDP_4, swReg->ariDevNumber);

    baseAddr->MISC_CONTROL_1_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfDbiRoWrEn_reg */

/*****************************************************************************
 * Combine and write the PL CONF AXI Slave Error Response (Sticky) register
 ****************************************************************************/
static int32_t Pcie_writePlconfAxiSlvErrRespReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfAxiSlvErrRespReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_GLOBAL, swReg->peripheralErrMap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_VENDORID, swReg->noVidErrMap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_CRS, swReg->errorResponseCrs);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_MAP, swReg->errorResponseMap);

    baseAddr->AMBA_ERROR_RESPONSE_DEFAULT_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_plconfAxiSlvErrResp_reg */

/*****************************************************************************
 * Combine and write the PL CONF Link Down AXI Slave Timeout (Sticky) register
 ****************************************************************************/
static int32_t Pcie_writePlconfAxiSlvTimeoutReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PlconfAxiSlvTimeoutReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_PERIOD_DEFAULT, swReg->timeoutValue);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_ENABLE_DEFAULT, swReg->flushEn);

    baseAddr->AMBA_LINK_TIMEOUT_OFF = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the PL CONF iATU Region Limit Address register
 ****************************************************************************/
static int32_t Pcie_writePlconfIatuRegLimitReg(CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegLimitReg *swReg)
{
    uint32_t new_val = swReg->raw;
    static int32_t status = SystemP_SUCCESS;
    /* Don't need to check NULL this is internal API */
    uint8_t regionIndex = simIatuWindow->regionIndex;



    if (regionIndex < 16)
    {
        if (simIatuWindow->regionDirection == 0U)
        {
            /* 0U == OUTBOUND */
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_RW, swReg->iatuRegLimit);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_HW, swReg->ones);

            swReg->raw = baseAddr->iatu[regionIndex].IATU_LIMIT_ADDR_OFF_OUTBOUND = new_val;
        }
        else
        {
            /* INBOUND */
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_RW, swReg->iatuRegLimit);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_HW, swReg->ones);

            swReg->raw = baseAddr->iatu[regionIndex].IATU_LIMIT_ADDR_OFF_INBOUND = new_val;

            status = SystemP_SUCCESS;
        }
    }
    else
    {
        status = SystemP_SUCCESS;
    }

    return status;
}

/*****************************************************************************
 * Combine and write the PCIE Capabilities register
 ****************************************************************************/
static int32_t Pcie_writePciesCapReg(volatile uint32_t *hwReg_PCIE_CAP, Pcie_PciesCapReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_ID, swReg->capId);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_NEXT_PTR, swReg->nextCap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_REG, swReg->pcieCap);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_DEV_PORT_TYPE, swReg->dportType);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_SLOT_IMP, swReg->sltImplN);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_INT_MSG_NUM, swReg->intMsg);

    *hwReg_PCIE_CAP = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_pciesCap_reg */

/*****************************************************************************
 * Combine and write the Device Capabilities register
 ****************************************************************************/
static int32_t Pcie_writeDeviceCapReg(volatile uint32_t *hwReg_DEV_CAP, Pcie_DeviceCapReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_MAX_PAYLOAD_SIZE, swReg->maxPayldSz);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_PHANTOM_FUNC_SUPPORT, swReg->phantomFld);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EXT_TAG_SUPP, swReg->extTagFld);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EP_L0S_ACCPT_LATENCY, swReg->l0Latency);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EP_L1_ACCPT_LATENCY, swReg->l1Latency);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_ROLE_BASED_ERR_REPORT, swReg->errRpt);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_CAP_SLOT_PWR_LMT_VALUE, swReg->pwrLimitValue);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_CAP_SLOT_PWR_LMT_SCALE, swReg->pwrLimitScale);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_FLR_CAP, swReg->flrEn);

    *hwReg_DEV_CAP = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Data of MSI CAP OFF 10H register
 ****************************************************************************/
static int32_t Pcie_writeMsiCapOff10HReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiCapOff10HReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_10H_REG_PCI_MSI_CAP_OFF_10H, swReg->data);

    baseAddr->MSI_CAP_OFF_10H_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_msiCapOff10H_reg */

/*****************************************************************************
 * Combine and write the Data of MSI CAP OFF 14H register
 ****************************************************************************/
static int32_t Pcie_writeMsiCapOff14HReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_MsiCapOff14HReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_14H_REG_PCI_MSI_CAP_OFF_14H, swReg->data);

    baseAddr->MSI_CAP_OFF_14H_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Power Management Capability register
 ****************************************************************************/
static int32_t Pcie_writePmCapReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PMCapReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_CAP_ID, swReg->pmCapID);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_NEXT_POINTER, swReg->pmNextPtr);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_SPEC_VER, swReg->pmeSpecVer);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PME_CLK, swReg->pmeClk);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_DSI, swReg->dsiN);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_AUX_CURR, swReg->auxCurrN);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_D1_SUPPORT, swReg->d1SuppN);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_D2_SUPPORT, swReg->d2SuppN);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PME_SUPPORT, swReg->pmeSuppN);

    baseAddr->CAP_ID_NXT_PTR_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_pmCap_reg */

/*****************************************************************************
 * Combine and write the Power Management Control and Status register
 ****************************************************************************/
static int32_t Pcie_writePmCapCtlStatReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_PMCapCtlStatReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_POWER_STATE, swReg->pwrState);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_NO_SOFT_RST, swReg->noSoftRst);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_PME_ENABLE, swReg->pmeEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_SELECT, swReg->dataSelect);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_SCALE, swReg->dataScale);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_PME_STATUS, swReg->pmeStatus);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_B2_B3_SUPPORT, swReg->b2b3Support);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_BUS_PWR_CLK_CON_EN, swReg->clkCtrlEn);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_REG_ADD_INFO, swReg->dataReg);

    baseAddr->CON_STATUS_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Latency Timer and Bus Number register
 ****************************************************************************/
static int32_t Pcie_writeType1BusNumReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1BusNumReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_PRIM_BUS, swReg->priBusNum);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_BUS, swReg->secBusNum);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SUB_BUS, swReg->subBusNum);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_LAT_TIMER, swReg->secLatTmr);

    baseAddr->SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_type1BusNum_reg */

/*****************************************************************************
 * Combine and write the Secondary Status and IO Base/Limit Register
 ****************************************************************************/
static int32_t Pcie_writeType1SecStatReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1SecStatReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE, swReg->IOBaseAddr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_BASE, swReg->IOBase);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_BIT8, swReg->IOLimitAddr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_LIMIT, swReg->IOLimit);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_MDPE, swReg->mstDPErr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_SIG_TRGT_ABRT, swReg->txTgtAbort);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_TRGT_ABRT, swReg->rxTgtAbort);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_MSTR_ABRT, swReg->rxMstAbort);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_SYS_ERR, swReg->rxSysError);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_DPE, swReg->dtctPError);

    baseAddr->SEC_STAT_IO_LIMIT_IO_BASE_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_type1SecStat_reg */

/*****************************************************************************
 * Combine and write the Memory Limit and Base register
 ****************************************************************************/
static int32_t Pcie_writeType1MemspaceReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1MemspaceReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE, swReg->base);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT, swReg->limit);

    baseAddr->MEM_LIMIT_MEM_BASE_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_type1Memspace_reg */

/*****************************************************************************
 * Combine and write the Prefetchable Memory Limit and Base register
 ****************************************************************************/
static int32_t Pcie_writePrefMemReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_PrefMemReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_DECODE, swReg->baseAddr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_BASE, swReg->base);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_DECODE, swReg->limitAddr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT, swReg->limit);

    baseAddr->PREF_MEM_LIMIT_PREF_MEM_BASE_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_prefMem_reg */

/*****************************************************************************
 * Combine and write the Prefetchable Memory Base Upper register
 ****************************************************************************/
static int32_t Pcie_writePrefBaseUpperReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_PrefBaseUpperReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_PREF_BASE_UPPER_REG_PREF_MEM_BASE_UPPER, swReg->base);

    baseAddr->PREF_BASE_UPPER_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_prefBaseUp_reg */

/*****************************************************************************
 * Combine and write the Prefetchable Memory Limit Upper  register
 ****************************************************************************/
static int32_t Pcie_writePrefLimitUpperReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_PrefLimitUpperReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_PREF_LIMIT_UPPER_REG_PREF_MEM_LIMIT_UPPER, swReg->limit);

    baseAddr->PREF_LIMIT_UPPER_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_prefLimitUp_reg */

/*****************************************************************************
 * Combine and write the IO Base and Limit Upper 16 bits register
 ****************************************************************************/
static int32_t Pcie_writeType1IOSpaceReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1IOSpaceReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_BASE_UPPER, swReg->IOBase);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_LIMIT_UPPER, swReg->IOLimit);

    baseAddr->IO_LIMIT_UPPER_IO_BASE_UPPER_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_type1IOSpace_reg */

/*****************************************************************************
 * Combine and write the Capabilities Pointer register
 ****************************************************************************/
static int32_t Pcie_writeType1CapPtrReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1CapPtrReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_CAP_POINTER, swReg->capPtr);

    baseAddr->TYPE1_CAP_PTR_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_type1CapPtr_reg */

/*****************************************************************************
 * Combine and write the Expansion ROM Base Address register
 ****************************************************************************/
static int32_t Pcie_writeType1ExpnsnRomReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1ExpnsnRomReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_ROM_BAR_ENABLE, swReg->expRomEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_EXP_ROM_BASE_ADDRESS, swReg->expRomBaseAddr);

    baseAddr->TYPE1_EXP_ROM_BASE_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_type1ExpnsnRom_reg */

/*****************************************************************************
 * Combine and write the Bridge Control and Interrupt register
 ****************************************************************************/
static int32_t Pcie_writeType1BridgeIntReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1BridgeIntReg *swReg)
{
    uint32_t new_val = swReg->raw;


    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_LINE, swReg->intLine);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_PIN, swReg->intPin);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_PERE, swReg->pErrRespEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SERR_EN, swReg->serrEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_ISA_EN, swReg->isaEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_EN, swReg->vgaEn);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_16B_DEC, swReg->vgaDecode);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_MSTR_ABORT_MODE, swReg->mstAbortMode);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SBR, swReg->secBusRst);

    baseAddr->BRIDGE_CTRL_INT_PIN_INT_LINE_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the BIST and Header register
 ****************************************************************************/
static int32_t Pcie_writeType1BistHeaderReg(CSL_pcie_rc_coreRegs *baseAddr, Pcie_Type1BistHeaderReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE, swReg->cacheLnSize);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER, swReg->latTmr);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_HEADER_TYPE, swReg->hdrType);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_MULTI_FUNC, swReg->mulFunDev);
    PCIE_SETBITS(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_BIST, swReg->bist);

    baseAddr->TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Interrupt Pin register
 ****************************************************************************/
static int32_t Pcie_writeIntPinReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_IntPinReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG_INT_LINE, swReg->intLine);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG_INT_PIN, swReg->intPin);

    baseAddr->MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Capabilities Pointer register
 ****************************************************************************/
static int32_t Pcie_writeCapPtrReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_CapPtrReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PCI_CAP_PTR_REG_CAP_POINTER, swReg->ptr);

    baseAddr->PCI_CAP_PTR_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Expansion ROM Base Address register
 ****************************************************************************/
static int32_t Pcie_writeExpRomReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_ExpRomReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EXP_ROM_BASE_ADDR_REG_ROM_BAR_ENABLE, swReg->enable);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_EXP_ROM_BASE_ADDR_REG_EXP_ROM_BASE_ADDRESS, swReg->expRomAddr);

    baseAddr->EXP_ROM_BASE_ADDR_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Cardbus CIS Pointer register
 ****************************************************************************/
static int32_t Pcie_writeCardbusCisPointerReg(CSL_pcie_ep_coreRegs *baseAddr, Pcie_CardbusCisPointerReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_CARDBUS_CIS_PTR_REG_CARDBUS_CIS_POINTER, swReg->cisPointer);

    baseAddr->CARDBUS_CIS_PTR_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the Subsystem and Subsystem Vendor ID register
 ****************************************************************************/
static int32_t Pcie_writeSubIdReg(CSL_pcie_ep_coreRegs *baseAddr,Pcie_SubIdReg *swReg)
{
    uint32_t new_val = swReg->raw;

    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG_SUBSYS_VENDOR_ID, swReg->subVndId);
    PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG_SUBSYS_DEV_ID, swReg->subId);

    baseAddr->SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG = swReg->raw = new_val;

    return SystemP_SUCCESS;
} /* Pcie_write_subId_reg */

/*****************************************************************************
 * Combine and write the BIST and Header register
 ****************************************************************************/
static int32_t Pcie_writeBistReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_BistReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE, swReg->cacheLnSize);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER, swReg->latTmr);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_HEADER_TYPE, swReg->hdrType);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_MULTI_FUNC, swReg->mulfunDev);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_BIST, swReg->bist);

  baseAddr->BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_bist_reg */

/*****************************************************************************
 * Combine and write the Reset Command swRegister
 ****************************************************************************/
static int32_t Pcie_writeRstCmdReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_RstCmdReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_RSTCMD_FLR_PF_ACTIVE, swReg->flrPfActive);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_RSTCMD_INIT_RST, swReg->initRst);

  baseAddr->RSTCMD = swReg->raw = new_val;

  return SystemP_SUCCESS;
}

/*****************************************************************************
 * Combine and write the PTM Config Command swRegister
 ****************************************************************************/
static int32_t Pcie_writePtmCfgReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PtmCfgReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PTMCFG_PTM_CLK_SEL, swReg->ptmClkSel);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PTMCFG_PTM_CONTEXT_VALID, swReg->ptmContextValid);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PTMCFG_PTM_MANUAL_UPDATE, swReg->ptmManualUpdate);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PTMCFG_PTM_AUTO_UPDATE, swReg->ptmAutoUpdate);

  baseAddr->PTMCFG = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_ptmCfgReg */

/*****************************************************************************
 * Combine and write the Power Management Command swRegister
 ****************************************************************************/
static int32_t Pcie_writePmCmdReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PmCmdReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMCMD_PM_XMT_TURNOFF, swReg->turnOff);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMCMD_PM_XMT_PE, swReg->pme);

  baseAddr->PMCMD = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_pmCmdReg */

static int32_t Pcie_writeIrqEOIReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_IrqEOIReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IRQ_EOI_EOI, swReg->EOI);

  baseAddr->IRQ_EOI = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_irqEOIReg */

/*****************************************************************************
 * Combine and write the MSI Interrupt IRQ swRegister
 ****************************************************************************/
static int32_t Pcie_writeMsiIrqReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_MsiIrqReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_MMR_IRQ, swReg->msiIrq);

  baseAddr->MMR_IRQ = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_msiIrqReg */

/*****************************************************************************
 * Combine and write the Endpoint Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_writeEpIrqStatusReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_EpIrqStatusReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_LEGACY_IRQ_STATUS_0, swReg->epIrqStatus);

  baseAddr->LEGACY_IRQ_STATUS = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_epIrqStatusReg */

/*****************************************************************************
 * Combine and write a General Purpose swRegister [0-3]
 ****************************************************************************/
static int32_t Pcie_writeGenPurposeReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_GenPurposeReg * swReg, int_fast32_t swRegNum)
{
  /* swRegNum generated internally no need for bounds check */
  baseAddr->GPR[swRegNum] = swReg->raw = swReg->genPurpose;

  return SystemP_SUCCESS;
} /* Pcie_write_epIrqStatusReg */

/*****************************************************************************
 * Combine and write a MSI Raw Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_writeMsiIrqStatusRawReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_MsiIrqStatusRawReg * swReg, int_fast32_t swRegNum)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_STATUS_RAW_MMR_IRQ_STATUS_RAW, swReg->msiRawStatus);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_STATUS_RAW = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_msiIrqStatusRawReg */

/*****************************************************************************
 * Combine and write a MSI Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_writeMsiIrqStatusReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_MsiIrqStatusReg * swReg, int_fast32_t swRegNum)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_STATUS_MMR_IRQ_STATUS, swReg->msiIrqStatus);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_STATUS = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_msiIrqStatusReg */

/*****************************************************************************
 * Combine and write a MSI Interrupt Enable Set swRegister
 ****************************************************************************/
static int32_t Pcie_writeMsiIrqEnableSetReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_MsiIrqEnableSetReg * swReg, int_fast32_t swRegNum)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_ENABLE_SET_MMR_IRQ_EN_SET, swReg->msiIrqEnSet);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_ENABLE_SET = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_msiIrqEnableSetReg */

/*****************************************************************************
 * Combine and write the MSI Interrupt Enable Clear swRegister
 ****************************************************************************/
static int32_t Pcie_writeMsiIrqEnableClrReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_MsiIrqEnableClrReg * swReg, int_fast32_t swRegNum)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_ENABLE_CLR_MMR_IRQ_EN_CLR, swReg->msiIrqEnClr);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_msiIrqEnableClrReg */

/*****************************************************************************
 * Combine and write a Legacy Raw Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_writeLegacyIrqStatusRawReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_LegacyIrqStatusRawReg * swReg, int_fast32_t swRegNum)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_RAW_INT_RAW, swReg->legacyRawStatus);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_STATUS_RAW = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_legacyIrqStatusRawReg */

/*****************************************************************************
 * Combine and write a Raw ERR Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_writeErrIrqStatusRawReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_ErrIrqStatusRawReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_AER_RAW, swReg->errAer);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_CORR_RAW, swReg->errCorr);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_NONFATAL_RAW, swReg->errNonFatal);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_FATAL_RAW, swReg->errFatal);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_SYS_RAW, swReg->errSys);

  baseAddr->ERR_IRQ_STATUS_RAW = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_errIrqStatusRawReg */

/*****************************************************************************
 * Combine and write a Err Interrupt Status swRegister
 ****************************************************************************/
static int32_t Pcie_writeErrIrqStatusReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_ErrIrqStatusReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_AER, swReg->errAer);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_CORR, swReg->errCorr);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_NONFATAL, swReg->errNonFatal);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_FATAL, swReg->errFatal);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_SYS, swReg->errSys);

  baseAddr->ERR_IRQ_STATUS = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_errIrqStatusReg */

/*****************************************************************************
 * Combine and write a Err Interrupt Enable Set swRegister
 ****************************************************************************/
static int32_t Pcie_writeErrIrqEnableSetReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_ErrIrqEnableSetReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_AER_EN_SET, swReg->errAer);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_CORR_EN_SET, swReg->errCorr);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_NONFATAL_EN_SET, swReg->errNonFatal);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_FATAL_EN_SET, swReg->errFatal);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_SYS_EN_SET, swReg->errSys);

  baseAddr->ERR_IRQ_ENABLE_SET = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_errIrqEnableSetReg */

/*****************************************************************************
 * Combine and write the Err Interrupt Enable Clear swRegister
 ****************************************************************************/
static int32_t Pcie_writeErrIrqEnableClrReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_ErrIrqEnableClrReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_AER_EN_CLR, swReg->errAer);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_CORR_EN_CLR, swReg->errCorr);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL_EN_CLR, swReg->errNonFatal);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_FATAL_EN_CLR, swReg->errFatal);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_SYS_EN_CLR, swReg->errSys);

  baseAddr->ERR_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_errIrqEnableClrReg */

/*****************************************************************************
 * Combine and write a Raw Power Management and Reset Status swRegister
 ****************************************************************************/
static int32_t Pcie_writePmRstIrqStatusRawReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PmRstIrqStatusRawReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_LNK_RST_REQ_RAW, swReg->linkRstReq);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_PME_RAW, swReg->pmPme);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_TO_ACK_RAW, swReg->pmToAck);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_TURNOFF_RAW, swReg->pmTurnoff);

  baseAddr->PMRST_IRQ_STATUS_RAW = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_pmRstIrqStatusRawReg */

/*****************************************************************************
 * Combine and write a Power Management and Reset Status swRegister
 ****************************************************************************/
static int32_t Pcie_writePmRstIrqStatusReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PmRstIrqStatusReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_LNK_RST_REQ, swReg->linkRstReq);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_PME, swReg->pmPme);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_TO_ACK, swReg->pmToAck);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_TURNOFF, swReg->pmTurnoff);

  baseAddr->PMRST_IRQ_STATUS = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_pmRstIrqStatusReg */

/*****************************************************************************
 * Combine and write a Power Management and Reset Enable Set swRegister
 ****************************************************************************/
static int32_t Pcie_writePmRstIrqEnableSetReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PmRstIrqEnableSetReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_LNK_RST_REQ_EN_SET, swReg->linkRstReq);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_PME_EN_SET, swReg->pmPme);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_TO_ACK_EN_SET, swReg->pmToAck);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_TURNOFF_EN_SET, swReg->pmTurnoff);

  baseAddr->PMRST_IRQ_ENABLE_SET = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_pmRstIrqEnableSetReg */

/*****************************************************************************
 * Combine and write the Power Management and Reset Enable Clear swRegister
 ****************************************************************************/
static int32_t Pcie_writePmRstIrqEnableClrReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PmRstIrqEnableClrReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_LNK_RST_REQ_EN_CLR, swReg->linkRstReq);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_PME_EN_CLR, swReg->pmPme);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_TO_ACK_EN_CLR, swReg->pmToAck);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_TURNOFF_EN_CLR, swReg->pmTurnoff);

  baseAddr->PMRST_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_pmRstIrqEnableClrReg */

/*****************************************************************************
 * Combine and write a Raw Precision Time Measurement Status swRegister
 ****************************************************************************/
static int32_t Pcie_writePtmIrqStatusRawReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PtmIrqStatusRawReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PTM_IRQ_STATUS_RAW_PTM_CLK_UPDATED_RAW, swReg->ptmClkUpdated);

  baseAddr->PTM_IRQ_STATUS_RAW = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_ptmIrqStatusRawReg */

/*****************************************************************************
 * Combine and write a Precision Time Measurement Status swRegister
 ****************************************************************************/
static int32_t Pcie_writePtmIrqStatusReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PtmIrqStatusReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PTM_IRQ_STATUS_PTM_CLK_UPDATED, swReg->ptmClkUpdated);

  baseAddr->PTM_IRQ_STATUS = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_ptmIrqStatusReg */

/*****************************************************************************
 * Combine and write a Precision Time Measurement Enable Set swRegister
 ****************************************************************************/
static int32_t Pcie_writePtmIrqEnableSetReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PtmIrqEnableSetReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PTM_IRQ_ENABLE_SET_PTM_CLK_UPDATED_EN_SET, swReg->ptmClkUpdated);

  baseAddr->PTM_IRQ_ENABLE_SET = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_ptmIrqEnableSetReg */

/*****************************************************************************
 * Combine and write the Precision Time Measurement Enable Clear swRegister
 ****************************************************************************/
static int32_t Pcie_writePtmIrqEnableClrReg(CSL_pcie_ep_coreRegs * baseAddr, Pcie_PtmIrqEnableClrReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_PTM_IRQ_ENABLE_CLR_PTM_CLK_UPDATED_EN_CLR, swReg->ptmClkUpdated);

  baseAddr->PTM_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_ptmIrqEnableClrReg */

/*****************************************************************************
 * Combine and write the Vendor and Device Identification register
 ****************************************************************************/
static int32_t Pcie_writeVndDevIdReg(volatile uint32_t * hwReg_DEVICE_VENDORID, Pcie_VndDevIdReg * swReg)
{
  uint32_t new_val = swReg->raw;

  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_ID_VENDOR_ID_REG_PCI_TYPE0_VENDOR_ID, swReg->vndId);
  PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_DEVICE_ID_VENDOR_ID_REG_PCI_TYPE0_DEVICE_ID, swReg->devId);

  *hwReg_DEVICE_VENDORID = swReg->raw = new_val;

  return SystemP_SUCCESS;
} /* Pcie_write_vndDevIdReg */

/*****************************************************************************
 * Combine and write the PL CONF iATU Region Control 2 register
 ****************************************************************************/
static int32_t Pcie_writePlconfIatuRegCtrl2Reg(CSL_pcie_ep_coreRegs *baseAddr, const Pcie_PlconfIatuIndexReg *simIatuWindow, Pcie_PlconfIatuRegCtrl2Reg *swReg)
{
  uint32_t new_val = swReg->raw;
  uint32_t status = SystemP_SUCCESS;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
        if (simIatuWindow->regionDirection == 0U)
        {
            /* 0U == OUTBOUND */
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_MSG_CODE, swReg->messagecode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG, swReg->tag);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_SUBSTITUTE_EN, swReg->tagSubstEn);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_FUNC_BYPASS, swReg->functionNumberMatchEnable);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_SNP, swReg->SNP);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INHIBIT_PAYLOAD, swReg->inhibitPayload);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_HEADER_SUBSTITUTE_EN, swReg->headerSubstEn);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_CFG_SHIFT_MODE, swReg->cfgShiftMode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INVERT_MODE, swReg->invertMode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_REGION_EN, swReg->regionEnable);

            swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_2_OFF_OUTBOUND = new_val;
        }
        else
        {
            /* INBOUND */
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE, swReg->messagecode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_BAR_NUM, swReg->barNumber);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_TYPE_MATCH_MODE, swReg->msgTypeMatchMode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TC_MATCH_EN, swReg->tcMatchEnable);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TD_MATCH_EN, swReg->tdMatchEnable);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_ATTR_MATCH_EN, swReg->attrMatchEnable);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUNC_NUM_MATCH_EN, swReg->functionNumberMatchEnable);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MATCH_EN, swReg->messageCodeMatchEnable);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_SINGLE_ADDR_LOC_TRANS_EN, swReg->singleAddrLocTransEn);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_RESPONSE_CODE, swReg->responseCode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUZZY_TYPE_MATCH_CODE, swReg->fuzzyTypeMatchMode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_CFG_SHIFT_MODE, swReg->cfgShiftMode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_INVERT_MODE, swReg->invertMode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MATCH_MODE, swReg->matchMode);
            PCIE_SETBITS(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_REGION_EN, swReg->regionEnable);

            swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_2_OFF_INBOUND = new_val;

            status = SystemP_SUCCESS;
        }
  }
  else
  {
        status = SystemP_SUCCESS;
  }

  return status;
}

/*********************************************************************
 * Writes any register
 ********************************************************************/
int32_t Pcie_writeRegs(Pcie_Handle handle, Pcie_Location location, Pcie_Registers * writeRegs)
{
  int32_t status = SystemP_SUCCESS;
  int32_t i;
  Pcie_Config *pcieCfg = NULL;
  Pcie_DeviceCfgBaseAddr *cfg = NULL;
  Pcie_DeviceCfgBaseAddrs *bases = NULL;

  Pcie_PlconfIatuIndexReg *simIatuWindow = NULL;

  /* Base Address for the Config Space
     These registers can be Local/Remote and Type0(EP)/Type1(RC) */
  CSL_pcie_ep_coreRegs *baseCfgEp = NULL;
  CSL_pcie_rc_coreRegs *baseCfgRc = NULL;

  if (handle != NULL)
  {
        pcieCfg = (Pcie_Config *)handle;
        cfg = Pcie_handleGetBases(handle);

        simIatuWindow = &pciev0LocalObj.simIatuWindow[pcieCfg->attrs->deviceNum];

        if (cfg != NULL)
        {
            bases = (Pcie_DeviceCfgBaseAddrs *)cfg->cfgBase;
        }
        else
        {
            status = SystemP_FAILURE;
        }

        if (status == SystemP_SUCCESS)
        {
            baseCfgEp = (CSL_pcie_ep_coreRegs *)bases->cfgBase;
            baseCfgRc = (CSL_pcie_rc_coreRegs *)bases->cfgBase;
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
            char *remoteBase = (char *)cfg->dataBase + bases->remoteOffset;

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
        status = Pcie_writeCmdStatusReg(baseCfgEp, writeRegs->cmdStatus);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->cfgTrans != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->ioBase != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tlpCfg != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->rstCmd != NULL))
  {
        status = Pcie_writeRstCmdReg(baseCfgEp, writeRegs->rstCmd);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->ptmCfg != NULL))
  {
        status = Pcie_writePtmCfgReg(baseCfgEp, writeRegs->ptmCfg);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->pmCmd != NULL))
  {
        status = Pcie_writePmCmdReg(baseCfgEp, writeRegs->pmCmd);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->pmCfg != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->obSize != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->diagCtrl != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->endian != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->priority != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->irqEOI != NULL))
  {
        status = Pcie_writeIrqEOIReg(baseCfgEp, writeRegs->irqEOI);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->msiIrq != NULL))
  {
        status = Pcie_writeMsiIrqReg(baseCfgEp, writeRegs->msiIrq);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->epIrqSet != NULL))
  {
        status = Pcie_writeEpIrqSetReg(baseCfgEp, writeRegs->epIrqSet);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->epIrqClr != NULL))
  {
        status = Pcie_writeEpIrqClrReg(baseCfgEp, writeRegs->epIrqClr);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->epIrqStatus != NULL))
  {
        status = Pcie_writeEpIrqStatusReg(baseCfgEp, writeRegs->epIrqStatus);
  }
  for (i = 0; i < 4; i++)
  {
        if ((status == SystemP_SUCCESS) && (writeRegs->genPurpose[i] != NULL))
        {
            status = Pcie_writeGenPurposeReg(baseCfgEp, writeRegs->genPurpose[i], i);
        }
  }
  for (i = 0; i < 8; i++)
  {
        if ((status == SystemP_SUCCESS) && (writeRegs->msiIrqStatusRaw[i] != NULL))
        {
            status = Pcie_writeMsiIrqStatusRawReg(baseCfgEp, writeRegs->msiIrqStatusRaw[i], i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->msiIrqStatus[i] != NULL))
        {
            status = Pcie_writeMsiIrqStatusReg(baseCfgEp, writeRegs->msiIrqStatus[i], i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->msiIrqEnableSet[i] != NULL))
        {
            status = Pcie_writeMsiIrqEnableSetReg(baseCfgEp, writeRegs->msiIrqEnableSet[i], i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->msiIrqEnableClr[i] != NULL))
        {
            status = Pcie_writeMsiIrqEnableClrReg(baseCfgEp, writeRegs->msiIrqEnableClr[i], i);
        }
  }
  for (i = 0; i < 4; i++)
  {
        if ((status == SystemP_SUCCESS) && (writeRegs->legacyIrqStatusRaw[i] != NULL))
        {
            status = Pcie_writeLegacyIrqStatusRawReg(baseCfgEp, writeRegs->legacyIrqStatusRaw[i], i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->legacyIrqStatus[i] != NULL))
        {
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->legacyIrqEnableSet[i] != NULL))
        {
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->legacyIrqEnableClr[i] != NULL))
        {
            status = SystemP_FAILURE;
        }
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->errIrqStatusRaw != NULL))
  {
        status = Pcie_writeErrIrqStatusRawReg(baseCfgEp, writeRegs->errIrqStatusRaw);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->errIrqStatus != NULL))
  {
        status = Pcie_writeErrIrqStatusReg(baseCfgEp, writeRegs->errIrqStatus);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->errIrqEnableSet != NULL))
  {
        status = Pcie_writeErrIrqEnableSetReg(baseCfgEp, writeRegs->errIrqEnableSet);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->errIrqEnableClr != NULL))
  {
        status = Pcie_writeErrIrqEnableClrReg(baseCfgEp, writeRegs->errIrqEnableClr);
  }

  if ((status == SystemP_SUCCESS) && (writeRegs->pmRstIrqStatusRaw != NULL))
  {
        status = Pcie_writePmRstIrqStatusRawReg(baseCfgEp, writeRegs->pmRstIrqStatusRaw);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->pmRstIrqStatus != NULL))
  {
        status = Pcie_writePmRstIrqStatusReg(baseCfgEp, writeRegs->pmRstIrqStatus);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->pmRstIrqEnableSet != NULL))
  {
        status = Pcie_writePmRstIrqEnableSetReg(baseCfgEp, writeRegs->pmRstIrqEnableSet);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->pmRstIrqEnableClr != NULL))
  {
        status = Pcie_writePmRstIrqEnableClrReg(baseCfgEp, writeRegs->pmRstIrqEnableClr);
  }

  if ((status == SystemP_SUCCESS) && (writeRegs->ptmIrqStatusRaw != NULL))
  {
        status = Pcie_writePtmIrqStatusRawReg(baseCfgEp, writeRegs->ptmIrqStatusRaw);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->ptmIrqStatus != NULL))
  {
        status = Pcie_writePtmIrqStatusReg(baseCfgEp, writeRegs->ptmIrqStatus);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->ptmIrqEnableSet != NULL))
  {
        status = Pcie_writePtmIrqEnableSetReg(baseCfgEp, writeRegs->ptmIrqEnableSet);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->ptmIrqEnableClr != NULL))
  {
        status = Pcie_writePtmIrqEnableClrReg(baseCfgEp, writeRegs->ptmIrqEnableClr);
  }

  for (i = 0; i < 8; i++)
  {
        if ((status == SystemP_SUCCESS) && (writeRegs->obOffsetLo[i] != NULL))
        {
            /* Not supported on this revision */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->obOffsetHi[i] != NULL))
        {
            /* Not supported on this revision */
            status = SystemP_FAILURE;
        }
  }

  for (i = 0; i < 4; i++)
  {
        if ((status == SystemP_SUCCESS) && (writeRegs->ibBar[i] != NULL))
        {
            /* Not supported on this revision */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->ibStartLo[i] != NULL))
        {
            /* Not supported on this revision */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->ibStartHi[i] != NULL))
        {
            /* Not supported on this revision */
            status = SystemP_FAILURE;
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->ibOffset[i] != NULL))
        {
            /* Not supported on this revision */
            status = SystemP_FAILURE;
        }
  }

  if ((status == SystemP_SUCCESS) && (writeRegs->pcsCfg0 != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->pcsCfg1 != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }

  if ((status == SystemP_SUCCESS) && (writeRegs->serdesCfg0 != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->serdesCfg1 != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }

  /*****************************************************************************************
   *Configuration Registers
   *****************************************************************************************/

  /*Type 0, Type1 Common Registers*/

  if ((status == SystemP_SUCCESS) && (writeRegs->vndDevId != NULL))
  {
        status = Pcie_writeVndDevIdReg(&baseCfgEp->DEVICE_ID_VENDOR_ID_REG, writeRegs->vndDevId);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->statusCmd != NULL))
  {
        status = Pcie_writeStatusCmdReg(&baseCfgEp->STATUS_COMMAND_REG, writeRegs->statusCmd);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->revId != NULL))
  {
        status = Pcie_writeRevIdReg(&baseCfgEp->CLASS_CODE_REVISION_ID, writeRegs->revId);
  }

  if ((status == SystemP_SUCCESS) && (writeRegs->bist != NULL))
  {
        status = Pcie_writeBistReg(baseCfgEp, writeRegs->bist);
  }

  /*Type 0 Registers*/
  if ((status == SystemP_SUCCESS) && (writeRegs->type0BarIdx != NULL))
  {
        status = Pcie_writeType0BarReg(baseCfgEp, &(writeRegs->type0BarIdx->reg),
                                        writeRegs->type0BarIdx->idx);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type0BarMask32bitIdx != NULL))
  {
        status = Pcie_writeType0Bar32bitReg(baseCfgEp, &(writeRegs->type0BarMask32bitIdx->reg),
                                             writeRegs->type0BarMask32bitIdx->idx);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type0Bar32bitIdx != NULL))
  {
        status = Pcie_writeType0Bar32bitReg(baseCfgEp, &(writeRegs->type0Bar32bitIdx->reg),
                                             writeRegs->type0Bar32bitIdx->idx);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->subId != NULL))
  {
        status = Pcie_writeSubIdReg(baseCfgEp, writeRegs->subId);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->cardbusCisPointer != NULL))
  {
        status = Pcie_writeCardbusCisPointerReg(baseCfgEp, writeRegs->cardbusCisPointer);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->expRom != NULL))
  {
        status = Pcie_writeExpRomReg(baseCfgEp, writeRegs->expRom);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->capPtr != NULL))
  {
        status = Pcie_writeCapPtrReg(baseCfgEp, writeRegs->capPtr);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->intPin != NULL))
  {
        status = Pcie_writeIntPinReg(baseCfgEp, writeRegs->intPin);
  }

  /*Type 1 Registers*/
  if ((status == SystemP_SUCCESS) && (writeRegs->type1BistHeader != NULL))
  {
        status = Pcie_writeType1BistHeaderReg(baseCfgRc, writeRegs->type1BistHeader);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1BarIdx != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1BarMask32bitIdx != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1Bar32bitIdx != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1BusNum != NULL))
  {
        status = Pcie_writeType1BusNumReg(baseCfgRc, writeRegs->type1BusNum);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1SecStat != NULL))
  {
        status = Pcie_writeType1SecStatReg(baseCfgRc, writeRegs->type1SecStat);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1Memspace != NULL))
  {
        status = Pcie_writeType1MemspaceReg(baseCfgRc, writeRegs->type1Memspace);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->prefMem != NULL))
  {
        status = Pcie_writePrefMemReg(baseCfgRc, writeRegs->prefMem);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->prefBaseUpper != NULL))
  {
        status = Pcie_writePrefBaseUpperReg(baseCfgRc, writeRegs->prefBaseUpper);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->prefLimitUpper != NULL))
  {
        status = Pcie_writePrefLimitUpperReg(baseCfgRc, writeRegs->prefLimitUpper);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1IOSpace != NULL))
  {
        status = Pcie_writeType1IOSpaceReg(baseCfgRc, writeRegs->type1IOSpace);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1CapPtr != NULL))
  {
        status = Pcie_writeType1CapPtrReg(baseCfgRc, writeRegs->type1CapPtr);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1ExpnsnRom != NULL))
  {
        status = Pcie_writeType1ExpnsnRomReg(baseCfgRc, writeRegs->type1ExpnsnRom);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->type1BridgeInt != NULL))
  {
        status = Pcie_writeType1BridgeIntReg(baseCfgRc, writeRegs->type1BridgeInt);
  }

  /* Power Management Capabilities Registers */
  if ((status == SystemP_SUCCESS) && (writeRegs->pmCap != NULL))
  {
        status = Pcie_writePmCapReg(baseCfgEp, writeRegs->pmCap);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->pmCapCtlStat != NULL))
  {
        status = Pcie_writePmCapCtlStatReg(baseCfgEp, writeRegs->pmCapCtlStat);
  }

  /*MSI Registers*/
  if ((status == SystemP_SUCCESS) && (writeRegs->msiCap != NULL))
  {
        status = Pcie_writeMsiCapReg(baseCfgEp, writeRegs->msiCap);
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
        status = Pcie_writeMsiDataReg(baseCfgEp, writeRegs->msiData);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->msiCapOff10H != NULL))
  {
        status = Pcie_writeMsiCapOff10HReg(baseCfgEp, writeRegs->msiCapOff10H);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->msiCapOff14H != NULL))
  {
        status = Pcie_writeMsiCapOff10HReg(baseCfgEp, writeRegs->msiCapOff10H);
  }

  /*Capabilities Registers*/
  if ((status == SystemP_SUCCESS) && (writeRegs->pciesCap != NULL))
  {
        status = Pcie_writePciesCapReg(&baseCfgEp->PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG, writeRegs->pciesCap);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->deviceCap != NULL))
  {
        status = Pcie_writeDeviceCapReg(&baseCfgEp->DEVICE_CAPABILITIES_REG, writeRegs->deviceCap);
  }

  if ((status == SystemP_SUCCESS) && (writeRegs->devStatCtrl != NULL))
  {
        status = Pcie_writeDevStatCtrlReg(&baseCfgEp->DEVICE_CONTROL_DEVICE_STATUS, writeRegs->devStatCtrl);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->linkCap != NULL))
  {
        status = Pcie_writeLinkCapReg(&baseCfgEp->LINK_CAPABILITIES_REG, writeRegs->linkCap);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->linkStatCtrl != NULL))
  {
        status = Pcie_writeLinkStatCtrlReg(&baseCfgEp->LINK_CONTROL_LINK_STATUS_REG, writeRegs->linkStatCtrl);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->slotCap != NULL))
  {
        status = Pcie_writeSlotCapReg(baseCfgRc, writeRegs->slotCap);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->slotStatCtrl != NULL))
  {
        status = Pcie_writeSlotStatCtrlReg(baseCfgRc, writeRegs->slotStatCtrl);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->rootCtrlCap != NULL))
  {
        status = Pcie_writeRootCtrlCapReg(baseCfgRc, writeRegs->rootCtrlCap);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->rootStatus != NULL))
  {
        status = Pcie_writeRootStatusReg(baseCfgRc, writeRegs->rootStatus);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->devCap2 != NULL))
  {
        status = Pcie_writeDevCap2Reg(&baseCfgEp->DEVICE_CAPABILITIES2_REG, writeRegs->devCap2);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->devStatCtrl2 != NULL))
  {
        status = Pcie_writeDevStatCtrl2Reg(&baseCfgEp->DEVICE_CONTROL2_DEVICE_STATUS2_REG, writeRegs->devStatCtrl2);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->linkCap2 != NULL))
  {
        status = Pcie_writeLinkCap2Reg(&baseCfgEp->LINK_CAPABILITIES2_REG, writeRegs->linkCap2);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->linkCtrl2 != NULL))
  {
        status = Pcie_writeLinkCtrl2Reg(&baseCfgEp->LINK_CONTROL2_LINK_STATUS2_REG, writeRegs->linkCtrl2);
  }

  /*Capabilities Extended Registers*/
  if ((status == SystemP_SUCCESS) && (writeRegs->extCap != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->uncErr != NULL))
  {
        status = Pcie_writeUncErrReg(baseCfgEp, writeRegs->uncErr);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->uncErrMask != NULL))
  {
        status = Pcie_writeUncErrMaskReg(baseCfgEp, writeRegs->uncErrMask);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->uncErrSvrty != NULL))
  {
        status = Pcie_writeUncErrSvrtyReg(baseCfgEp, writeRegs->uncErrSvrty);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->corErr != NULL))
  {
        status = Pcie_writeCorErrReg(baseCfgEp, writeRegs->corErr);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->corErrMask != NULL))
  {
        status = Pcie_writeCorErrMaskReg(baseCfgEp, writeRegs->corErrMask);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->accr != NULL))
  {
        status = Pcie_writeAccrReg(baseCfgEp, writeRegs->accr);
  }
  for (i = 0; i < 4; i++)
  {
        if ((status == SystemP_SUCCESS) && (writeRegs->hdrLog[i] != NULL))
        {
            /* Not supported on this revision */
            status = SystemP_FAILURE;
        }
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->rootErrCmd != NULL))
  {
        status = Pcie_writeRootErrCmdReg(baseCfgRc, writeRegs->rootErrCmd);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->rootErrSt != NULL))
  {
        status = Pcie_writeRootErrStReg(baseCfgRc, writeRegs->rootErrSt);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->errSrcID != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }

  /*Port Logic Registers*/
  if ((status == SystemP_SUCCESS) && (writeRegs->plAckTimer != NULL))
  {
        status = Pcie_writePlAckTimerReg(baseCfgEp, writeRegs->plAckTimer);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plOMsg != NULL))
  {
        status = Pcie_writePlOMsgReg(baseCfgEp, writeRegs->plOMsg);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plForceLink != NULL))
  {
        status = Pcie_writePlForceLinkReg(baseCfgEp, writeRegs->plForceLink);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->ackFreq != NULL))
  {
        status = Pcie_writeAckFreqReg(baseCfgEp, writeRegs->ackFreq);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->lnkCtrl != NULL))
  {
        status = Pcie_writeLnkCtrlReg(baseCfgEp, writeRegs->lnkCtrl);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->laneSkew != NULL))
  {
        status = Pcie_writeLaneSkewReg(baseCfgEp, writeRegs->laneSkew);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->symNum != NULL))
  {
        status = Pcie_writeSymNumReg(baseCfgEp, writeRegs->symNum);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->symTimerFltMask != NULL))
  {
        status = Pcie_writeSymTimerFltMaskReg(baseCfgEp, writeRegs->symTimerFltMask);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->fltMask2 != NULL))
  {
        status = Pcie_writeFltMask2Reg(baseCfgEp, writeRegs->fltMask2);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->gen2 != NULL))
  {
        status = Pcie_writeGen2Reg(baseCfgEp, writeRegs->gen2);
  }

  /* hw rev 2 PLCONF registers */
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfObnpSubreqCtrl != NULL))
  {
        status = Pcie_writePlconfObnpSubreqCtrlReg(baseCfgEp, writeRegs->plconfObnpSubreqCtrl);
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
        status = Pcie_writePlconfQStsRReg(baseCfgEp, writeRegs->plconfQStsR);
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
        status = Pcie_writePlconfVcPrQCReg(baseCfgEp, writeRegs->plconfVc0PrQC, 0);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfVc0NprQC != NULL))
  {
        status = Pcie_writePlconfVcNprQCReg(baseCfgEp, writeRegs->plconfVc0NprQC, 0);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfVc0CrQC != NULL))
  {
        status = Pcie_writePlconfVcCrQCReg(baseCfgEp, writeRegs->plconfVc0CrQC, 0);
  }
  for (i = 0; i < 3; i++)
  {
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfVcPrQC[i] != NULL))
        {
            status = Pcie_writePlconfVcPrQCReg(baseCfgEp, writeRegs->plconfVcPrQC[i], 1 + i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfVcNprQC[i] != NULL))
        {
            status = Pcie_writePlconfVcNprQCReg(baseCfgEp, writeRegs->plconfVcNprQC[i], 1 + i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfVcCrQC[i] != NULL))
        {
            status = Pcie_writePlconfVcCrQCReg(baseCfgEp, writeRegs->plconfVcCrQC[i], 1 + i);
        }
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfPhyStsR != NULL))
  {
        /* Pure RO register */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfPhyCtrlR != NULL))
  {
        status = Pcie_writePlconfPhyCtrlRReg(baseCfgEp, writeRegs->plconfPhyCtrlR);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlAddress != NULL))
  {
        status = Pcie_writePlconfMsiCtrlAddressReg(baseCfgEp, writeRegs->plconfMsiCtrlAddress);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlUpperAddress != NULL))
  {
        status = Pcie_writePlconfMsiCtrlUpperAddressReg(baseCfgEp, writeRegs->plconfMsiCtrlUpperAddress);
  }
  for (i = 0; i < 8; i++)
  {
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlIntEnable[i] != NULL))
        {
            status = Pcie_writePlconfMsiCtrlIntEnableReg(baseCfgEp, writeRegs->plconfMsiCtrlIntEnable[i], i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlIntMask[i] != NULL))
        {
            status = Pcie_writePlconfMsiCtrlIntMaskReg(baseCfgEp, writeRegs->plconfMsiCtrlIntMask[i], i);
        }
        if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlIntStatus[i] != NULL))
        {
            status = Pcie_writePlconfMsiCtrlIntStatusReg(baseCfgEp, writeRegs->plconfMsiCtrlIntStatus[i], i);
        }
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfMsiCtrlGpio != NULL))
  {
        status = Pcie_writePlconfMsiCtrlGpioReg(baseCfgEp, writeRegs->plconfMsiCtrlGpio);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfPipeLoopback != NULL))
  {
        status = Pcie_writePlconfPipeLoopbackReg(baseCfgEp, writeRegs->plconfPipeLoopback);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfDbiRoWrEn != NULL))
  {
        status = Pcie_writePlconfDbiRoWrEnReg(baseCfgEp, writeRegs->plconfDbiRoWrEn);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfAxiSlvErrResp != NULL))
  {
        status = Pcie_writePlconfAxiSlvErrRespReg(baseCfgEp, writeRegs->plconfAxiSlvErrResp);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfAxiSlvTimeout != NULL))
  {
        status = Pcie_writePlconfAxiSlvTimeoutReg(baseCfgEp, writeRegs->plconfAxiSlvTimeout);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuIndex != NULL))
  {
        /* Set the simulated window address */
        *simIatuWindow = *writeRegs->plconfIatuIndex;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegCtrl2 != NULL))
  {
        status = Pcie_writePlconfIatuRegCtrl2Reg(baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegCtrl2);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegLowerBase != NULL))
  {
        status = Pcie_writePlconfIatuRegLowerBaseReg(baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegLowerBase);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegUpperBase != NULL))
  {
        status = Pcie_writePlconfIatuRegUpperBaseReg(baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegUpperBase);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegLimit != NULL))
  {
        status = Pcie_writePlconfIatuRegLimitReg(baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegLimit);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegLowerTarget != NULL))
  {
        status = Pcie_writePlconfIatuRegLowerTargetReg(baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegLowerTarget);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegUpperTarget != NULL))
  {
        status = Pcie_writePlconfIatuRegUpperTargetReg(baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegUpperTarget);
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegCtrl3 != NULL))
  {
        /* Pure RO register */
        status = SystemP_FAILURE;
  }
  /* Ctrl1 is done last since it has enable bit */
  if ((status == SystemP_SUCCESS) && (writeRegs->plconfIatuRegCtrl1 != NULL))
  {
        status = Pcie_writePlconfIatuRegCtrl1Reg(baseCfgEp, simIatuWindow, writeRegs->plconfIatuRegCtrl1);
  }

  /* Reject hw rev 2 TI CONF registers */
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfRevision != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfSysConfig != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEoi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqStatusRawMain != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqStatusMain != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEnableSetMain != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEnableClrMain != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqStatusRawMsi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqStatusMsi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEnableSetMsi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIrqEnableClrMsi != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDeviceType != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDeviceCmd != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfPmCtrl != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfPhyCs != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIntxAssert != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfIntxDeassert != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfMsiXmt != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDebugCfg != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDebugData != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }
  if ((status == SystemP_SUCCESS) && (writeRegs->tiConfDiagCtrl != NULL))
  {
        /* Not supported on this revision */
        status = SystemP_FAILURE;
  }

  return status;
}

int32_t Pcie_getMsiRegs (Pcie_Handle handle, Pcie_Location location, Pcie_MsiParams *params)
{
    int32_t status = SystemP_SUCCESS;
    Pcie_Registers regs;
    Pcie_MsiCapReg msiCapReg;
    Pcie_MsiLo32Reg msiLo32;
    Pcie_MsiUp32Reg msiUp32;
    Pcie_MsiDataReg msiData;

    if ((handle == NULL) || (params == NULL))
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
        params->data = msiData.data;
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
