/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#ifndef PCIE_V0_REG_H_
#define PCIE_V0_REG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * \brief Specification of the PCIe Peripheral ID Register
 *
 * This Register contains the major and minor revisions
 * for the PCIe module.
 */
typedef struct pciePidReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Scheme
     * Field size: 2 bits
     */
    uint8_t scheme;
    /**
     * [ro] Function code
     *
     * 0xe30 is PCIe
     * Field size: 12 bits
     */
    uint16_t func;
    /**
     * [ro] Module ID of the Peripheral
     *
     * 0x6810 is PCIe
     * Field size: 16 bits
     */
    uint16_t modId;
    /**
     * [ro] RTL Version
     * Field size: 5 bits
     */
    uint8_t rtl;
    /**
     * [ro] Major revision
     * Field size: 3 bits
     */
    uint8_t revMaj;
    /**
     * [ro] Customer special version
     * Field size: 2 bits
     */
    uint8_t cust;
    /**
     * [ro] Minor revision
     * Field size: 6 bits
     */
    uint8_t revMin;
} Pcie_PidReg;


/**
 * \brief Specification of the Command Status Register
 *
 * This Register is used to enable address translation, link training
 * and writing to BAR mask registers.
 */
typedef struct pcieCmdStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Set to enable manual reversal for RX lanes.
     * Field size: 1 bit
     */
    uint8_t rxLaneFlipEn;
    /**
     * [rw] Set to enable manual reversal for TX lanes.
     * Field size: 1 bit
     */
    uint8_t txLaneFlipEn;
    /**
     * [rw] Set to enable writing to BAR mask registers that are overlaid on BAR registers.
     * Field size: 1 bit
     */
    uint8_t dbi;
    /**
     * [rw] Application retry Enable
     *
     * This feature can be used if initialization can take longer than PCIe
     * stipulated time frame.
     *
     * 1 = Enable all incoming PCIe transactions to be returned with a retry response.
     * Field size: 1 bit
     */
    uint8_t appRetryEn;
    /**
     * [rw] Posted Write Enable
     *
     * Default is 0 with all internal bus master writes defaulting to non-posted.
     *
     * 1 = Enable the internal bus master to use posted write commands.
     * Field size: 1 bit
     */
    uint8_t postedWrEn;
    /**
     * [rw] Inbound Translation Enable
     *
     * 1 = Enable translation of inbound memory/IO read/write requests
     * into memory read/write requests.
     * Field size: 1 bit
     */
    uint8_t ibXltEn;
    /**
     * [rw] Outbound Translation Enable
     *
     * 1 = Enable translation of outbound memory read/write requests into
     * memory/IO/configuration read/write requests.
     * Field size: 1 bit
     */
    uint8_t obXltEn;
    /**
     * [rw] Link Training Enable
     *
     * 1 = Enable LTSSM in PCI Express core and link negotiation with
     * link partner will begin.
     * Field size: 1 bit
     */
    uint8_t ltssmEn;
} Pcie_CmdStatusReg;

/**
 * \brief Specification of the Configuration Transaction Setup Register
 */
typedef struct pcieCfgTransReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Configuration type for outbound configuration accesses
     *
     * 0 = Type 0 access.
     * 1 = Type 1 access.
     * Field size: 1 bit
     */
    uint8_t type;
    /**
     * [rw] PCIe bus number for outbound configuration accesses
     * Field size: 8 bits
     */
    uint8_t bus;
    /**
     * [rw] PCIe device number for outbound configuration accesses
     * Field size: 5 bit
     */
    uint8_t device;
    /**
     * [rw] PCIe function number for outbound configuration accesses
     * Field size: 3 bits
     */
    uint8_t func;
} Pcie_CfgTransReg;

/**
 * \brief Specification of the IO TLP Base Register
 */
typedef struct pcieIoBaseReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] outgoing IO TLP. RC mode only
     * Field size: 20 bits
     *
     */
    uint32_t ioBase;
} Pcie_IoBaseReg;

/**
 * \brief Specification of the TLP configuration Register
 */
typedef struct pcieTlpCfgReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Enable relaxed ordering for all outgoing TLPs
     * Field size: 1 bit
     */
    uint8_t relaxed;
    /**
     * [rw] Enable No Snoop attribute on all outgoing TLPs
     * Field size: 1 bit
     */
    uint8_t noSnoop;
} Pcie_TlpCfgReg;

/**
 * \brief Specification of the Reset Command Register
 */
typedef struct pcieRstCmdReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Bridge flush status
     *
     * Used to ensure no pending transactions prior to issuing warm reset.
     * 0 = No transaction is pending.
     * 1 = There are transactions pending.
     * Field size: 1 bit
     */
    uint8_t flush;
    /**
     * [ro] Bridge flush status
     *
     * Used to ensure no pending transactions prior to issuing warm reset.
     * 0 = No transaction is pending.
     * 1 = There are transactions pending.
     * Field size: 1 bit
     */
    uint8_t flrPfActive;
    /**
     * [w1] Write 1 to initiate a downstream hot reset sequence on downstream.
     * Field size: 1 bit
     */
    uint8_t initRst;
} Pcie_RstCmdReg;

/**
 * \brief Specification of the PTM Config register
 */
typedef struct pciePtmCfgReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Select ptm_local_clk bit input to CPTS.
     *
     * 0 will select ptm_local_clk[0],
     * 1 will select ptm_local_clk[1] ...
     * 63 will select ptm_local_clk[63]
     * Field size: 6 bit
     */
    uint8_t ptmClkSel;
    /**
     * [ro] '1' indicates PTM context is valid-EP only
     * Field size: 1 bit
     */
    uint8_t ptmContextValid;
    /**
     * [w1] Write '1' to enable PTM transaction.EP only.
     *
     * EP will initiate one PTM transaction when this field is updated.
     * Always reads '0'
     * Field size: 1 bit
     */
    uint8_t ptmManualUpdate;
    /**
     * [rw] Write '1' to enable PTM auto-update-EP only
     *
     * EP will automatically initiate PTM transaction every 10ms
     * Field size: 1 bit
     */
    uint8_t ptmAutoUpdate;
} Pcie_PtmCfgReg;

/**
 * \brief Specification of the Power Management Command Register
 */
typedef struct pciePmCmdReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [w1] PM Turn off
     *
     * Write 1 to transmit a PM_TURNOFF message. Reads 0. Applicable in RC mode only.
     *
     * 0 = No effect\n
     * 1 = Transmit a PM_TURNOFF message
     * Field size: 1 bit
     */
    uint8_t turnOff;
    /**
     * [w1] Transmit PM PME message
     *
     * Write 1 to transmit a PM_PME message. Reads 0. Applicable to EP mode only.
     *
     * 0 = No effect\n
     * 1 = Transmit a PM_PME message
     * Field size: 1 bit
     */
    uint8_t pme;
} Pcie_PmCmdReg;

/**
 * \brief Specification of the Power Management Configuration Register
 */
typedef struct pciePmCfgReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] PM Turn off
     *
     * Write 1 to enable entry to L2/L3 ready state. Read to check L2/L3 entry readiness. Applicable to RC and EP.
     *
     * 0 = Disable entry to L2/L3 ready state.\n
     * 1 = Enable entry to L2/L3 ready state.
     * Field size: 1 bit
     */
    uint8_t entrL23;
} Pcie_PmCfgReg;


/**
 * \brief Specification of the Activity Status Register
 */
typedef struct pcieActStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Outbound Buffers Not Empty
     *
     * Field size: 1 bit
     */
    uint8_t obNotEmpty;
    /**
     * [ro] Inbound Buffers Not Empty
     *
     * Field size: 1 bit
     */
    uint8_t ibNotEmpty;
} Pcie_ActStatusReg;

/**
 * \brief Specification of the Outbound Size Register
 */
typedef struct pcieObSizeReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Set each outbound translation window size
     *
     * <TABLE>
     * <TR><TH>@ref size</TH><TH>Window Size</TH></TR>
     * <TR><TD>0</TD>        <TD>1 MB</TD></TR>
     * <TR><TD>1</TD>        <TD>2 MB</TD></TR>
     * <TR><TD>2</TD>        <TD>4 MB</TD></TR>
     * <TR><TD>3</TD>        <TD>8 MB</TD></TR>
     * <TR><TD>others</TD>   <TD>reserved</TD></TR>
     * </TABLE>
     *
     * Field size: 3 bits
     */
    uint8_t size;
} Pcie_ObSizeReg;

/**
 * \brief Specification of the Diagnostic Control register
 */
typedef struct pcieDiagCtrlReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Force ECRC error
     *
     * Write 1 to force inversion of LSB of ECRC for the next one packet.
     * It is self cleared when the ECRC error has been injected on one TLP.
     *
     * Field size: 1 bit
     */
    uint8_t invEcrc;
    /**
     * [rw] Force LCRC error
     *
     * Write 1 to force inversion of LSB of LCRC for the next one packet.
     * It is self cleared when the LCRC error has been injected on one TLP.
     *
     * Field size: 1 bit
     */
    uint8_t invLcrc;
} Pcie_DiagCtrlReg;

/**
 * \brief Specification of the Endian Register
 */
typedef struct pcieEndianReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     *  [rw] Endian mode.
     *
     * <TABLE>
     * <TR><TH>@ref mode</TH><TH>Endian Swap Mode</TH></TR>
     * <TR><TD>0</TD>        <TD>Swap on 1 byte</TD></TR>
     * <TR><TD>1</TD>        <TD>Swap on 2 bytes</TD></TR>
     * <TR><TD>2</TD>        <TD>Swap on 4 bytes</TD></TR>
     * <TR><TD>3</TD>        <TD>Swap on 8 bytes</TD></TR>
     * </TABLE>
     *
     * Field size: 2 bits
     */
    uint8_t mode;
} Pcie_EndianReg;

/**
 * \brief Specification of the Transaction Priority Register
 */
typedef struct pciePriorityReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Master PRIV value on master transactions
     *
     * Field size: 1 bit
     */
    uint8_t mstPriv;
    /**
     * [rw] Master PRIVID value on master transactions
     *
     * Field size: 4 bits
     */
    uint8_t mstPrivID;
    /**
     * [rw] Priority level for each inbound transaction on the
     * internal master port
     *
     * Field size: 3 bits
     */
    uint8_t mstPriority;
} Pcie_PriorityReg;

/**
 * \brief Specification of the End of Interrupt Register
 */
typedef struct pcieIrqEOIReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [wo] EOI for interrupts.
     *
     * Write to indicate end-of-interrupt for the interrupt events.
     *
     * Field size: 8 bits
     */
    uint8_t EOI;
} Pcie_IrqEOIReg;

/**
 * \brief Specification of the MSI Interrupt IRQ Register
 */
typedef struct pcieMsiIrqReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] To generate MSI Interrupt 0, the EP should write 0x0000_0000 to this register.
     *
     * Field size: 32 bits (rev 0) or 31 bits (rev 2)
     */
    uint32_t msiIrq;
} Pcie_MsiIrqReg;

/**
 * \brief Specification of the Endpoint Interrupt Request Set Register
 */
typedef struct pcieEpIrqSetReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Write 1 to generate assert interrupt message
     *
     * If MSI is disabled, legacy interrupt assert message will
     * be generated. On read, a 1 indicates currently asserted interrupt.
     *
     * Field size: 1 bit
     */
    uint8_t epIrqSet;
} Pcie_EpIrqSetReg;

/**
 * \brief Specification of the Endpoint Interrupt Request Clear Register
 */
typedef struct pcieEpIrqClrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Write 1 to generate deassert interrupt message.
     *
     * If MSI is disabled, legacy interrupt deassert message will be generated.
     * On read, a 1 indicates currently asserted interrupt.
     *
     * Field size: 1 bit
     */
    uint8_t epIrqClr;
} Pcie_EpIrqClrReg;

/**
 * \brief Specification of the Endpoint Interrupt status Register
 */
typedef struct pcieEpIrqStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Indicates whether interrupt for function 0 is asserted or not
     *
     * Field size: 1 bit
     */
    uint8_t epIrqStatus;
} Pcie_EpIrqStatusReg;

/**
 * \brief Specification of a General Purpose register
 */
typedef struct pcieGenPurposeReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Gen Purpose register value
     *
     * Field size: 32 bit
     */
    uint8_t genPurpose;
} Pcie_GenPurposeReg;

/**
 * \brief Specification of the MSI Raw Interrupt Status Register Register
 */
typedef struct pcieMsiIrqStatusRawReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Each bit indicates raw status of MSI vectors (24, 16, 8, 0) associated with the bit
     *
     * Typically, writes to this register are only done for debug purposes.
     *
     * Field size: 4 bits
     */
    uint8_t msiRawStatus;
} Pcie_MsiIrqStatusRawReg;

/**
 * \brief Specification of the MSI Interrupt Enabled Status Register Register
 */
typedef struct pcieMsiIrqStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Each bit indicates status of MSI vector (24, 16, 8, 0) associated with the bit
     *
     * Field size: 4 bits
     */
    uint8_t msiIrqStatus;
} Pcie_MsiIrqStatusReg;

/**
 * \brief Specification of the MSI Interrupt Enable Set Register
 */
typedef struct pcieMsiIrqEnableSetReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Each bit, when written to, enables the MSI interrupt (24, 16, 8, 0) associated with the bit
     *
     * Field size: 4 bits
     */
    uint8_t msiIrqEnSet;
} Pcie_MsiIrqEnableSetReg;

/**
 * \brief Specification of the MSI Interrupt Enable Clear Register
 */
typedef struct pcieMsiIrqEnableClrReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Each bit, when written to, disables the MSI interrupt (24, 16, 8, 0) associated with the bit
     *
     * Field size: 4 bits
     */
    uint8_t msiIrqEnClr;
} Pcie_MsiIrqEnableClrReg;

/**
 * \brief Specification of the Legacy Raw Interrupt Status Register
 */
typedef struct pcieLegacyIrqStatusRawReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Legacy Interrupt Raw Status, RC mode only
     *
     * Field size: 1 bit
     */
    uint8_t legacyRawStatus;
} Pcie_LegacyIrqStatusRawReg;

/**
 * \brief Specification of the Legacy Interrupt Enabled Status Register
 */
typedef struct pcieLegacyIrqStatusReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Legacy Interrupt status
     *
     * Set when interrupt is active. Write one to clear the interrupt event.
     * RC mode only.
     *
     * Field size: 1 bit
     */
    uint8_t legacyIrqStatus;
} Pcie_LegacyIrqStatusReg;

/**
 * \brief Specification of the Legacy Interrupt Enable Set Register
 */
typedef struct pcieLegacyIrqEnableSetReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] 0: has no effect; 1: enables the interrupt
     *
     * Field size: 1 bit
     */
    uint8_t legacyIrqEnSet;
} Pcie_LegacyIrqEnableSetReg;

/**
 * \brief Specification of the Legacy Interrupt Enable Clear Register
 */
typedef struct pcieLegacyIrqEnableClrReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] 0 has no effect; 1 disables the interrupt
     *
     * Field size: 1 bit
     */
    uint8_t legacyIrqEnClr;
} Pcie_LegacyIrqEnableClrReg;

/**
 * \brief Specification of the Raw ERR Interrupt Status Register
 */
typedef struct pcieErrIrqStatusRawReg_s {
    /**  [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] ECRC error raw status
     *
     * Field size: 1 bit
     */
    uint8_t errAer;
    /**
     * [rw] AXI tag lookup fatal error raw status
     *
     * Field size: 1 bit
     */
    uint8_t errAxi;
    /**
     * [rw] correctable error raw status
     *
     * Field size: 1 bit
     */
    uint8_t errCorr;
    /**
     * [rw] nonfatal error raw status
     *
     * Field size: 1 bit
     */
    uint8_t errNonFatal;
    /**
     * [rw] fatal error raw status
     *
     * Field size: 1 bit
     */
    uint8_t errFatal;
    /**
     * [rw] system error (fatal, nonfatal, correctable error) raw status
     *
     * Field size: 1 bit
     */
    uint8_t errSys;
} Pcie_ErrIrqStatusRawReg;

/**
 * \brief Specification of the ERR Interrupt Enabled Status Register
 */
typedef struct pcieErrIrqStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] ECRC error status
     *
     * Field size: 1 bit
     */
    uint8_t errAer;
    /**
     * [rw] AXI tag lookup fatal error status
     *
     * Field size: 1 bit
     */
    uint8_t errAxi;
    /**
     * [rw] correctable error status
     *
     * Field size: 1 bit
     */
    uint8_t errCorr;
    /**
     * [rw] nonfatal error status
     *
     * Field size: 1 bit
     */
    uint8_t errNonFatal;
    /**
     * [rw] fatal error status
     *
     * Field size: 1 bit
     */
    uint8_t errFatal;
    /**
     * [rw] system error (fatal, nonfatal, correctable error) status
     *
     * Field size: 1 bit
     */
    uint8_t errSys;
} Pcie_ErrIrqStatusReg;

/**
 * \brief Specification of the ERR Interrupt Enable Set Register
 */
typedef struct pcieErrIrqEnableSetReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] set to enable the ECRC error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errAer;
    /**
     * [rw] set to enable the AXI tag lookup fatal error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errAxi;
    /**
     * [rw] set to enable the correctable error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errCorr;
    /**
     * [rw] set to enable the nonfatal error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errNonFatal;
    /**
     * [rw] set to enable the fatal error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errFatal;
    /**
     * [rw] set to enable the system error (fatal, nonfatal, correctable error) interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errSys;
} Pcie_ErrIrqEnableSetReg;

/**
 * \brief Specification of the ERR Interrupt Enable Clear Register
 */
typedef struct pcieErrIrqEnableClrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] set to disable the ECRC error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errAer;
    /**
     * [rw] set to disable the AXI tag lookup fatal error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errAxi;
    /**
     * [rw] set to disable the correctable error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errCorr;
    /**
     * [rw] set to disable the nonfatal error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errNonFatal;
    /**
     * [rw] set to disable the fatal error interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errFatal;
    /**
     * [rw] set to disable the system error (fatal, nonfatal, correctable error) interrupt
     *
     * Field size: 1 bit
     */
    uint8_t errSys;
} Pcie_ErrIrqEnableClrReg;

/**
 * \brief Specification of the Raw Power Management and Reset Interrupt Status Register
 */
typedef struct pciePmRstIrqStatusRawReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Link Request Reset interrupt raw status
     *
     * Field size: 1 bit
     */
    uint8_t linkRstReq;
    /**
     * [rw] Power management PME message received interrupt raw status
     *
     * Field size: 1 bit
     */
    uint8_t pmPme;
    /**
     * [rw] Power mangement ACK received interrupt raw status
     *
     * Field size: 1 bit
     */
    uint8_t pmToAck;
    /**
     * [rw] Power management turnoff messages received raw status
     *
     * Field size: 1 bit
     */
    uint8_t pmTurnoff;
} Pcie_PmRstIrqStatusRawReg;

/**
 * \brief Specification of the Power Management and Reset Interrupt Enabled Status Register
 */
typedef struct pciePmRstIrqStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Link Request Reset interrupt status
     *
     * Field size: 1 bit
     */
    uint8_t linkRstReq;
    /**
     * [rw] Power management PME message received interrupt status
     *
     * Field size: 1 bit
     */
    uint8_t pmPme;
    /**
     * [rw] Power mangement ACK received interrupt status
     *
     * Field size: 1 bit
     */
    uint8_t pmToAck;
    /**
     * [rw] Power management turnoff messages received status
     *
     * Field size: 1 bit
     */
    uint8_t pmTurnoff;
} Pcie_PmRstIrqStatusReg;

/**
 * \brief Specification of the Power Management and Reset Interrupt Enable Set Register
 */
typedef struct pciePmRstIrqEnableSetReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] set to enable the Link Request Reset interrupt
     *
     * Field size: 1 bit
     */
    uint8_t linkRstReq;
    /**
     * [rw] set to enable the Power management PME message received interrupt
     *
     * Field size: 1 bit
     */
    uint8_t pmPme;
    /**
     * [rw] set to enable the Power mangement ACK received interrupt
     *
     * Field size: 1 bit
     */
    uint8_t pmToAck;
    /**
     * [rw] set to enable the Power management turnoff messages received interrupt
     *
     * Field size: 1 bit
     */
    uint8_t pmTurnoff;
} Pcie_PmRstIrqEnableSetReg;

/**
 * \brief Specification of the Power Management and Reset Interrupt Enable Clear Register
 */
typedef struct pciePmRstIrqEnableClrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] set to disable the Link Request Reset interrupt
     *
     * Field size: 1 bit
     */
    uint8_t linkRstReq;
    /**
     * [rw] set to disable the Power management PME message received interrupt
     *
     * Field size: 1 bit
     */
    uint8_t pmPme;
    /**
     * [rw] set to disable the Power mangement ACK received interrupt
     *
     * Field size: 1 bit
     */
    uint8_t pmToAck;
    /**
     * [rw] set to disable the Power management turnoff messages received interrupt
     *
     * Field size: 1 bit
     */
    uint8_t pmTurnoff;
} Pcie_PmRstIrqEnableClrReg;

/**
 * \brief Specification of the Precision Time Measurement Raw Status Register
 */
typedef struct pciePtmIrqStatusRawReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Link Request Reset interrupt raw status
     *
     * Field size: 1 bit
     */
    uint8_t ptmClkUpdated;
} Pcie_PtmIrqStatusRawReg;

/**
 * \brief Specification of the Precision Time Measurement Status Register
 */
typedef struct pciePtmIrqStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Link Request Reset interrupt raw status
     *
     * Field size: 1 bit
     */
    uint8_t ptmClkUpdated;
} Pcie_PtmIrqStatusReg;

/**
 * \brief Specification of the Precision Time Measurement Raw Status Register
 */
typedef struct pciePtmIrqEnableSetReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Link Request Reset interrupt raw status
     *
     * Field size: 1 bit
     */
    uint8_t ptmClkUpdated;
} Pcie_PtmIrqEnableSetReg;

/**
 * \brief Specification of the Precision Time Measurement Raw Status Register
 */
typedef struct pciePtmIrqEnableClrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Link Request Reset interrupt raw status
     *
     * Field size: 1 bit
     */
    uint8_t ptmClkUpdated;
} Pcie_PtmIrqEnableClrReg;

/**
 * \brief Specification of the Outbound Translation Region Offset Low and Index Register
 */
typedef struct pcieObOffsetLoReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Offset bits for the translation region
     *
     * Field size: 12 bits
     */
    uint16_t offsetLo;
    /**
     * [rw] Enable translation region
     *
     * Field size: 1 bit
     */
    uint8_t enable;
} Pcie_ObOffsetLoReg;

/**
 * \brief Specification of the Outbound Translation Region Offset High Register
 */
typedef struct pcieObOffsetHiReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Offset high bits [63:32] for translation region
     *
     * Field size: 32 bits
     */
    uint32_t offsetHi;
} Pcie_ObOffsetHiReg;

/**
 * \brief Specification of the Inbound Translation BAR Match Register
 */
typedef struct pcieIbBarReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] BAR number to match for inbound translation region
     *
     * Field size: 3 bits
     */
    uint8_t ibBar;
} Pcie_IbBarReg;

/**
 * \brief Specification of the Inbound Translation Start Address Low Register
 */
typedef struct pcieIbStartLoReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Start address bits [31:8] for inbound translation region
     *
     * Field size: 24 bits
     */
    uint32_t ibStartLo;
} Pcie_IbStartLoReg;

/**
 * \brief Specification of the Inbound Translation Start Address High Register
 */
typedef struct pcieIbStartHiReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Start address high bits [63:32] for inbound translation region
     *
     * Field size: 32 bits
     */
    uint32_t ibStartHi;
} Pcie_IbStartHiReg;

/**
 * \brief Specification of the Inbound Translation Address Offset Register
 */
typedef struct pcieIbOffsetReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Offset address bits [31:8] for inbound translation region
     *
     * Field size: 24 bits
     */
    uint32_t ibOffset;
} Pcie_IbOffsetReg;

/**
 * \brief Specification of the PCS Configuration 0 Register
 */
typedef struct pciePcsCfg0Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Receiver lock/sync control.
     *
     * Field size: 5 bits
     */
    uint8_t pcsSync;
    /**
     * [rw] Receiver initialization holdoff control.
     *
     * Field size: 8 bits
     */
    uint8_t pcsHoldOff;
    /**
     * [rw] Rate change delay.
     *
     * Field size: 2 bits
     */
    uint8_t pcsRCDelay;
    /**
     * [rw] Detection delay.
     *
     * Field size: 4 bits
     */
    uint8_t pcsDetDelay;
    /**
     * [rw] Enable short times for debug purposes.
     *
     * Field size: 1 bit
     */
    uint8_t pcsShrtTM;
    /**
     * [rw] Enable PIPE Spec 1.86 for phystatus behavior.
     *
     * Field size: 1 bit
     */
    uint8_t pcsStat186;
    /**
     * [rw] Fed term output to 3'b100 during reset.
     *
     * Field size: 1 bit
     */
    uint8_t pcsFixTerm;
    /**
     * [rw] Fix std output to 2'b10.
     *
     * Field size: 1 bit
     */
    uint8_t pcsFixStd;
    /**
     * [rw] Deassert enidl during L2 state.
     *
     * Field size: 1 bit
     */
    uint8_t pcsL2EnidlOff;
    /**
     * [rw] Deassert Rx enable in L0s state.
     *
     * Field size: 1 bit
     */
    uint8_t pcsL2L0SRxOff;
    /**
     * [rw] RX and TX on during reset and TX also on in P1 state.
     *
     * Field size: 1 bit
     */
    uint8_t pcsRxTxOn;
    /**
     * [rw] RX and TX on during reset.
     *
     * Field size: 1 bit
     */
    uint8_t pcsRxTxRst;
} Pcie_PcsCfg0Reg;

/**
 * \brief Specification of the PCS Configuration 1 Register
 */
typedef struct pciePcsCfg1Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Error bit enable.
     *
     * Field size: 10 bits
     */
    uint16_t pcsErrBit;
    /**
     * [rw] Error lane enable
     *
     * Field size: 2 bits
     */
    uint8_t pcsErrLn;
    /**
     * [rw] Error injection mode
     *
     * Field size: 2 bits
     */
    uint8_t pcsErrMode;
} Pcie_PcsCfg1Reg;

/**
 * \brief Specification of the PCS Status Register
 */
typedef struct pciePcsStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] PCS RTL Revision.
     *
     * Field size: 3 bits
     */
    uint8_t pcsRev;
    /**
     * [ro] PCS lanes enabled status.
     *
     * Field size: 2 bits
     */
    uint8_t pcsLnEn;
    /**
     * [ro] PCS transmitters enabled status.
     *
     * Field size: 2 bits
     */
    uint8_t pcsTxEn;
    /**
     * [ro] PCS receivers enabled status.
     *
     * Field size: 2 bits
     */
    uint8_t pcsRxEn;
} Pcie_PcsStatusReg;

/**
 * \brief Specification of the SERDES config 0 Register
 */
typedef struct pcieSerdesCfg0Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Enable Tx loopback. Set both bits high to enable.
     *
     * Field size: 2 bits
     */
    uint8_t txLoopback;
    /**
     * [rw] Master mode for synchronization.
     *
     * Field size: 1 bit
     */
    uint8_t txMsync;
    /**
     * [rw] Enable common mode adjustment.
     *
     * Field size: 1 bit
     */
    uint8_t txCm;
    /**
     * [rw] Invert Tx pair polarity.
     *
     * Field size: 1 bit
     */
    uint8_t txInvpair;
    /**
     * [rw] Enable Rx loopback. Set both bits to high to enable loopback.
     *
     * Field size: 2 bits
     */
    uint8_t rxLoopback;
    /**
     * [rw] Enable Rx offset compensation.
     *
     * Field size: 1 bit
     */
    uint8_t rxEnoc;
    /**
     * [rw] Enable Rx adaptive equalization.
     *
     * Field size: 4 bits
     */
    uint8_t rxEq;
    /**
     * [rw] Enable Rx clock data recovery.
     *
     * Field size: 3 bits
     */
    uint8_t rxCdr;
    /**
     * [rw] Enable Rx loss of signal detection.
     *
     * Field size: 3 bits
     */
    uint8_t rxLos;
    /**
     * [rw] Enable Rx symbol alignment.
     *
     * Field size: 2 bits
     */
    uint8_t rxAlign;
    /**
     * [rw] Invert Rx pair polarity.
     *
     * Field size: 1 bit
     */
    uint8_t rxInvpair;
} Pcie_SerdesCfg0Reg;

/**
 * \brief Specification of the SERDES config 1 Register
 */
typedef struct pcieSerdesCfg1Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw; /**  [ro] Raw image of register on read; actual value on write */
    /**
     * [rw] Enable Tx loopback. Set both bits high to enable.
     *
     * Field size: 2 bits
     */
    uint8_t txLoopback;
    /**
     * [rw] Master mode for synchronization.
     *
     * Field size: 1 bit
     */
    uint8_t txMsync;
    /**
     * [rw] Enable common mode adjustment.
     *
     * Field size: 1 bit
     */
    uint8_t txCm;
    /**
     * [rw] Invert Tx pair polarity.
     *
     * Field size: 1 bit
     */
    uint8_t txInvpair;
    /**
     * [rw] Enable Rx loopback. Set both bits to high to enable loopback.
     *
     * Field size: 2 bits
     */
    uint8_t rxLoopback;
    /**
     * [rw] Enable Rx offset compensation.
     *
     * Field size: 1 bit
     */
    uint8_t rxEnoc;
    /**
     * [rw] Enable Rx adaptive equalization.
     *
     * Field size: 4 bits
     */
    uint8_t rxEq;
    /**
     * [rw] Enable Rx clock data recovery.
     *
     * Field size: 3 bits
     */
    uint8_t rxCdr;
    /**
     * [rw] Enable Rx loss of signal detection.
     *
     * Field size: 3 bits
     */
    uint8_t rxLos;
    /**
     * [rw] Enable Rx symbol alignment.
     *
     * Field size: 2 bits
     */
    uint8_t rxAlign;
    /**
     * [rw] Invert Rx pair polarity.
     *
     * Field size: 1 bit
     */
    uint8_t rxInvpair;
} Pcie_SerdesCfg1Reg;


/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 0 and TYPE 1 REGISTERS ************
 **********        Registers that are common to both Types        ************
 ****************************************************************************/

/**
 * \brief Specification of the Vendor Device ID Register
 */
typedef struct pcieVndDevIdReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Device ID
     *
     * Field size: 16 bits
     */
    uint16_t devId;
    /**
     * [rw] Vendor ID
     *
     * Field size: 16 bits
     */
    uint16_t vndId;
} Pcie_VndDevIdReg;

/**
 * \brief Specification of the Status Command Register
 */
typedef struct pcieStatusCmdReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] parity
     *
     * Set if received a poisoned TLP
     *
     * Field size: 1 bit
     */
    uint8_t parity;
    /**
     * [rw] sys error
     *
     * Set if function sends an ERR_FATAL or ERR_NONFATAL message and
     * serrEn bit is set
     *
     * Field size: 1 bit
     */
    uint8_t sysError;
    /**
     * [rw] mst abort
     *
     * Set when a requester receives a completion with unsupported request
     * completion status
     *
     * Field size: 1 bit
     */
    uint8_t mstAbort;
    /**
     * [rw] tgt abort
     *
     * Set when a requester receives a completion with completer abort status.
     *
     * Field size: 1 bit
     */
    uint8_t tgtAbort;
    /**
     * [rw] sig tgt abort
     *
     * Set when a function acting as a completer terminates a request by issuing
     * completer abort completion status to the requester.
     *
     * Field size: 1 bit
     */
    uint8_t sigTgtAbort;
    /**
     * [ro] DevSel Timing
     *
     * Hardwired to 0 for PCIExpress.
     *
     * Field size: 2 bits
     */
    uint8_t devSelTime;
    /**
     * [rw] par error
     *
     * This bit is set by a requester if the parError bit is set
     * in its Command register and either the condition that the requester
     * receives a poisoned completion or the condition that the
     * requester poisons a write request is true.
     *
     * Field size: 1 bit
     */
    uint8_t parError;
    /**
     * [ro] Back to Back Capable
     *
     * Hardwired to 0 for PCIExpress.
     *
     * Field size: 1 bit
     */
    uint8_t fastB2B;
    /**
     * [ro] 66MHz Capable
     *
     * Hardwired to 0 for PCIExpress.
     *
     * Field size: 1 bit
     */
    uint8_t c66MhzCap;
    /**
     * [ro] cap list
     *
     * For PCIe, this field must be set to 1.
     *
     * Field size: 1 bit
     */
    uint8_t capList;
    /**
     * [rw] stat
     *
     * Indicates that the function has received an interrupt.
     *
     * Field size: 1 bit
     */
    uint8_t stat;
    /**
     * [ro] dis
     *
     * Setting this bit disables generation of INTx messages.
     *
     * Field size: 1 bit
     */
    uint8_t dis;
    /**
     * [rw] serr en
     *
     * When set, it enables generation of the appropriate PCI Express error
     * messages to the Root Complex.
     *
     * Field size: 1 bit
     */
    uint8_t serrEn;
    /**
     * [ro] Bit hardwired to 0 for PCIExpress
     *
     * Hardwired to 0 for PCIExpress.
     *
     * Field size: 1 bit
     */
    uint8_t idselCtrl;
    /**
     * [rw] resp
     *
     * This bit controls whether or not the device responds to detected
     * parity errors (poisoned TLP). This error is typically reported as an
     * unsupported request and may also result in a non-fatal error
     * message if @ref serrEn = 1. If this bit is set, the PCIESS will respond
     * normally to parity errors. If this bit is cleared, the PCIESS
     * will ignore detected parity errors.
     *
     * Field size: 1 bit
     */
    uint8_t resp;
    /**
     * [ro] Bit hardwired to 0 for PCIExpress
     *
     * Hardwired to 0 for PCIExpress.
     *
     * Field size: 1 bit
     */
    uint8_t vgaSnoop;
    /**
     * [ro] Bit hardwired to 0 for PCIExpress
     *
     * Hardwired to 0 for PCIExpress.
     *
     * Field size: 1 bit
     */
    uint8_t memWrInva;
    /**
     * [ro] Bit hardwired to 0 for PCIExpress
     *
     * Hardwired to 0 for PCIExpress.
     *
     * Field size: 1 bit
     */
    uint8_t specCycleEn;
    /**
     * [rw] enables mastership of the bus
     *
     * Field size: 1 bit
     */
    uint8_t busMs;
    /**
     * [rw] enables device to respond to memory access
     *
     * Field size: 1 bit
     */
    uint8_t memSp;
    /**
     * [rw] enables device to respond to IO access
     *
     * This functionality is not supported in PCIESS and therefore
     * this bit is set to 0.
     *
     * Field size: 1 bit
     */
    uint8_t ioSp;
} Pcie_StatusCmdReg;

/**
 * \brief Specification of the Base Address Register
 */
typedef struct pcieBaseAddrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] msio
     *
     * Set if it is a IO base address
     *
     * Field size: 1 bit
     */
    uint8_t msio;
    /**
     * [rw] r7
     *
     * Set if R7 needs to set
     *
     * Field size: 1 bit
     */
    uint8_t r7;
    /**
     * [rw] so
     *
     * Set if SO needs to set
     *
     * Field size: 1 bit
     */
    uint8_t s0;
    /**
     * [rw] po
     *
     * Set if PO needs to set
     *
     * Field size: 1 bit
     */
    uint8_t p0;
    /**
     * [rw] r8
     *
     * Set if R8 needs to set
     *
     * Field size: 1 bit
     */
    uint8_t r8;
    /**
     * [rw] bamr0
     *
     * Set if BAMR0 needs to set
     *
     * Field size: 4 bit
     */
    uint8_t bamr0;
    /**
     * [rw] bamrw
     *
     * Set if BAMRW needs to set
     *
     * Field size: 20 bit
     */
    uint32_t bamrw;
} Pcie_BaseAddrReg;

/**
 * \brief Specification of the Class code and revision ID Register
 */
typedef struct pcieRevIdReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Class Code
     *
     * Field size: 24 bits
     */
    uint32_t classCode;
    /**
     * [ro] Revision ID
     *
     * Field size: 8 bits
     */
    uint8_t revId;
} Pcie_RevIdReg;

/**
 * \brief Specification of the Base Address Register (BAR)
 *
 * This should be used to access a BAR register.
 *
 * There are two situations when this structure should be used:\n
 * 1. When setting up a 32 bit BAR\n
 * 2. When setting up the lower 32bits of a 64bits BAR
 *
 * Refer to @ref Pcie_Bar32bitReg for the other possible BAR configurations
 */
typedef struct pcieBarReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Base Address
     *
     * Field size: 28 bits
     */
    uint32_t base;
    /**
     * [rw] Prefetchable region?
     *
     * For memory BARs, it indicates whether the region is prefetchable.\n
     * 0 = Non-prefetchable.\n
     * 1 = Prefetchable.
     *
     * For I/O Bars, it is used as second least significant bit (LSB)
     * of the base address.
     *
     * Field size: 1 bit
     */
    uint8_t prefetch;
    /**
     * [rw] Bar Type
     *
     * For memory BARs, they determine the BAR type.\n
     * 0h = 32-bit BAR.\n
     * 2h = 64-bit BAR.\n
     * Others = Reserved.
     *
     * For I/O BARs, bit 2 is the least significant bit (LSB) of the
     * base address and bit 1 is 0.
     *
     * Field size: 2 bits
     */
    uint8_t type;
    /**
     * [rw] Memory or IO BAR
     *
     * 0 = Memory BAR.\n
     * 1 = I/O BAR.
     *
     * Field size: 1 bit
     */
    uint8_t memSpace;
    /**
     * BARxC
     */
    uint8_t barxc;
    /**
     * BARxA
     */
    uint8_t barxa;
} Pcie_BarReg;

/**
 * \brief Specification of the Base Address Register (BAR).
 *
 * This should be used to read/write a 32bit word to the BAR register.
 *
 * There are two situations when this structure should be used:\n
 * 1. When setting up BAR masks\n
 * 2. When setting up the upper 32bits of a 64bits BAR
 *
 * Refer to @ref Pcie_BarReg for the other possible BAR configurations
 */
typedef struct pcieBar32bitReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] 32bits word (BAR mask or BAR address)
     *
     * Field size: 32 bits
     */
    uint32_t reg32;
} Pcie_Bar32bitReg;


/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 0 REGISTERS  **********************
 ****************************************************************************/

/**
 * \brief Specification of the BIST Header Register
 */
typedef struct pcieBistReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Bist capability
     *
     * Returns a one for BIST capability and zero otherwise. Not supported by PCIESS.
     *
     * Field size: 1 bit
     */
    uint8_t bistCap;
    /**
     * [ro] Start Bist
     *
     * Write a one to start BIST. Not supported by PCIESS.
     *
     * Field size: 1 bit
     */
    uint8_t startBist;
    /**
     * [ro]  Completion code
     *
     * Not supported by PCIESS.
     *
     * Field size: 4 bits
     */
    uint8_t compCode;
    /**
     * [ro]  hw rev 1 bist field
     *
     * Field size: 8 bits
     */
    uint8_t bist;
    /**
     * [ro]  Multifunction device
     *
     * Field size:  1 bit
     */
    uint8_t mulfunDev;
    /**
     * [ro]  Header type
     *
     * Configuration header format.
     *
     * 0 = EP mode\n
     * 1 = RC mode
     *
     * Field size: 7 bits
     */
    uint8_t hdrType;
    /**
     * [ro] Not applicable in PCIe
     *
     * Field size:  8 bits
     */
    uint8_t latTmr;
    /**
     * [ro] Not applicable in PCIe
     *
     * Field size:  8 bits
     */
    uint8_t cacheLnSize;
} Pcie_BistReg;

/**
 * \brief @ref pcieBarReg_s register plus an index (End Point BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access an End Point BAR. For more details, please refer to @ref Pcie_BarReg
 */
typedef struct pcieType0BarIdx_s {
    Pcie_BarReg reg;  /**< Register Structure */
    uint8_t      idx;  /**< Index in the array of registers of this type */
} Pcie_Type0BarIdx;

/**
 * \brief @ref pcieBar32bitReg_s register plus an index (End Point BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access an End Point BAR. For more details, please refer to @ref Pcie_Bar32bitReg
 */
typedef struct pcieType0Bar32bitIdx_s {
    Pcie_Bar32bitReg reg;  /**< Register Structure */
    uint8_t           idx;  /**< Index in the array of registers of this type */
} Pcie_Type0Bar32bitIdx;

/**
 * \brief Specification of the Cardbus CIS pointer register
 */
typedef struct pcieCardbusCisPointerReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Cardbus CIS pointer (CS)
     *
     * Field size: 32 bits
     */
    uint32_t cisPointer;
} Pcie_CardbusCisPointerReg;

/**
 * \brief Specification of the Subsystem Vendor ID Register
 */
typedef struct pcieSubIdReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Subsystem ID
     *
     * Field size: 16 bits
     */
    uint16_t subId;
    /**
     * [ro] Subsystem Vendor ID
     *
     * Field size: 16 bits
     */
    uint16_t subVndId;
} Pcie_SubIdReg;

/**
 * \brief Specification of the Expansion ROM Register
 */
typedef struct pcieExpRomReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * Field size: 21 bits
     */
    uint32_t expRomAddr;
    /**
     * Field size: 1 bit
     */
    uint8_t enable;
} Pcie_ExpRomReg;

/**
 * \brief Specification of the Capability Pointer Register
 */
typedef struct pcieCapPtrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] First Capability Pointer
     *
     * By default, it points to Power Management Capability structure.
     *
     * Field size: 8 bits
     */
    uint8_t ptr;
} Pcie_CapPtrReg;

/**
 * \brief Specification of the Interrupt Pin Register
 */
typedef struct pcieIntPinReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] interrupt Pin
     *
     * It identifies the legacy interrupt message that the device uses.
     * For single function configuration, the core only uses INTA.
     *
     * <TABLE>
     * <TR><TH>@ref intPin</TH><TH>Legacy Interrupt</TH></TR>
     * <TR><TD>0</TD>          <TD>none</TD></TR>
     * <TR><TD>1</TD>          <TD>INTA</TD></TR>
     * <TR><TD>2</TD>          <TD>INTB</TD></TR>
     * <TR><TD>3</TD>          <TD>INTC</TD></TR>
     * <TR><TD>4</TD>          <TD>INTD</TD></TR>
     * <TR><TD>others</TD>     <TD>reserved</TD></TR>
     * </TABLE>
     *
     * Field size: 8 bits
     */
    uint8_t intPin;
    /**
     * [rw] interrupt line
     *
     * Field size: 8 bits
     */
    uint8_t intLine;
} Pcie_IntPinReg;


/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 1 REGISTERS  **********************
 ****************************************************************************/

/**
 * \brief @ref pcieBarReg_s register plus an index (Root Complex BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access a Root Complex BAR. For more details, please refer to @ref Pcie_BarReg.
 */
typedef struct pcieType1BarIdx_s {
    Pcie_BarReg reg;  /**< Register Structure */
    uint8_t      idx;  /**< Index in the array of registers of this type */
} Pcie_Type1BarIdx;

/**
 * \brief @ref pcieBar32bitReg_s register plus an index (Root Complex BAR)
 *
 * There are multiple instances of this register. The index is used to select which
 * instance of the register will be accessed.
 *
 * This structure is used to access a Root Complex BAR. For more details, please refer to @ref Pcie_Bar32bitReg.
 */
typedef struct pcieType1Bar32bitIdx_s {
    Pcie_Bar32bitReg reg;  /**< Register Structure */
    uint8_t           idx;  /**< Index in the array of registers of this type */
} Pcie_Type1Bar32bitIdx;

/**
 * \brief Specification of the BIST, Header Type, Latency Time and Cache Line Size Regiser
 */
typedef struct pcieType1BistHeaderReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Returns a 1 for BIST capability and 0 otherwise.
     *
     * Not supported by PCIESS.
     *
     * Field size: 1 bit
     */
    uint8_t bistCap;
    /**
     * [ro] Write a one to start BIST.
     *
     * Not supported by PCIESS.
     *
     * Field size: 1 bit
     */
    uint8_t startBist;
    /**
     * [rw] Completion Code.
     *
     * Not supported by PCIESS.
     *
     * Field size: 4 bits
     */
    uint8_t compCode;
    /**
     * [ro]  hw rev 1 bist field
     *
     * Field size: 8 bits
     */
    uint8_t bist;
    /**
     * [rw] Returns 1 if it is a multi-function device.
     *
     * Field size: 1 bit
     */
    uint8_t mulFunDev;
    /**
     * [rw] Configuration Header Format.
     *
     * 0 = EP mode\n
     * 1 = RC mode
     *
     * Field size: 7 bits
     */
    uint8_t hdrType;
    /**
     * [ro] Not applicable in PCIe
     *
     * Field size: 8 bits
     */
    uint8_t latTmr;
    /**
     * [ro] Not applicable in PCIe
     *
     * Field size: 8 bits
     */
    uint8_t cacheLnSize;
} Pcie_Type1BistHeaderReg;

/**
 * \brief Specification of the Latency Timer and Bus Number Register
 */
typedef struct pcieType1BusNumReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Secondary Latency Timer (N/A for PCIe)
     *
     * Field size: 8 bits
     */
    uint8_t secLatTmr;
    /**
     * [rw] Subordinate Bus Number. This is highest bus
     *             number on downstream interface.
     *
     * Field size: 8 bits
     */
    uint8_t subBusNum;
    /**
     * [rw] Secondary Bus Number. It is typically 1h for RC.
     *
     * Field size: 8 bits
     */
    uint8_t secBusNum;
    /**
     * [rw] Primary Bus Number. It is 0 for RC and nonzero for
     *             switch devices only.
     *
     * Field size: 8 bits
     */
    uint8_t priBusNum;
} Pcie_Type1BusNumReg;

/**
 * \brief Specification of the Secondary Status and IO Base/Limit Register
 */
typedef struct pcieType1SecStatReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Detected Parity Error.
     *
     * Read 1 if received a poisoned TLP.
     * Write 1 to clear; write 0 has no effect.
     *
     * Field size: 1 bit
     */
    uint8_t dtctPError;
    /**
     * [rw] Received System Error.
     *
     * Read 1 if received an ERR_FATAL or ERR_NONFATAL message.
     * Write 1 to clear; write 0 has no effect.
     *
     * Field size: 1 bit
     */
    uint8_t rxSysError;
    /**
     * [rw] Received Master Abort.
     *
     * Read 1 if received a completion with unsupported request completion status.
     * Write 1 to clear; write 0 has no effect.
     *
     * Field size: 1 bit
     */
    uint8_t rxMstAbort;
    /**
     * [rw] Received Target Abort.
     *
     * Read 1 if received a completion with completer abort completion status.
     * Write 1 to clear; write 0 has no effect.
     *
     * Field size: 1 bit
     */
    uint8_t rxTgtAbort;
    /**
     * [rw] Signaled Target Abort.
     *
     * Read 1 if sent a posted or non-posted request as a completer abort error.
     * Write 1 to clear; write 0 has no effect.
     *
     * Field size: 1 bit
     */
    uint8_t txTgtAbort;
    /**
     * [ro] DEVSEL Timing
     *
     * Field size: 2 bits
     */
    uint8_t devselTiming;
    /**
     * [rw] Master Data Parity Error.
     *
     * Read 1 if the parity error enable bit
     * @ref pcieType1BridgeIntReg_s::pErrRespEn is set and either the condition
     * that the requester receives a poisoned completion or the condition
     * that the requester poisons a write request is true.
     * Write 1 to clear; write 0 has no effect.
     *
     * Field size: 1 bit
     */
    uint8_t mstDPErr;
    /**
     * [ro] Fast Back to Back Capable.
     *
     * Field size: 1 bit
     */
    uint8_t fastB2bCap;
    /**
     * [ro] 66Mhz Capable.
     *
     * Field size: 1 bit
     */
    uint8_t c66mhzCapa;
    /**
     * [rw] Upper 4 bits of 16bit IO Space Limit Address.
     *
     * Field size: 4 bits
     */
    uint8_t IOLimit;
    /**
     * [rw] Indicates addressing for IO Limit Address.
     *
     * Writable from internal bus interface.
     * 0 = 16-bit IO addressing.
     * 1 = 32-bit IO addressing.
     *
     * Field size: 1 bit
     */
    uint8_t IOLimitAddr;
    /**
     * [rw] Upper 4 bits of 16bit IO Space Base Address.
     *
     * Field size: 4 bits
     */
    uint8_t IOBase;
    /**
     * [rw] Indicates addressing for the IO Base Address.
     *
     * Writable from internal bus interface.
     * 0 = 16-bit IO addressing.
     * 1 = 32-bit IO addressing.
     *
     * Field size: 1 bit
     */
    uint8_t IOBaseAddr;
} Pcie_Type1SecStatReg;

/**
 * \brief Specification of the Memory Limit and Base Register
 */
typedef struct pcieType1MemspaceReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Upper 12 bits of 32bit Memory Limit Address.
     *
     * Field size: 12 bits
     */
    uint16_t limit;
    /**
     * [rw] Upper 12 bits of 32bit Memory Base Address.
     *
     * Field size: 12 bit
     */
    uint16_t base;
} Pcie_Type1MemspaceReg;

/**
 * \brief Specification of the Prefetchable Memory Limit and Base Register
 */
typedef struct pciePrefMemReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Memory limit
     *
     * Upper 12 bits of 32bit prefetchable memory limit address (end address).
     *
     * Field size: 12 bits
     */
    uint16_t limit;
    /**
     * [rw] 32 or 64 bit addressing
     *
     * Indicates addressing for prefetchable memory limit address (end address).
     *
     * 0 = 32-bit memory addressing\n
     * 1 = 64-bit memory addressing
     *
     * Field size: 1 bit
     */
    uint8_t limitAddr;
    /**
     * [rw] Memory base
     *
     * Upper 12 bits of 32bit prefetchable memory base address (start address).
     *
     * Field size: 12 bits
     */
    uint16_t base;
    /**
     * [rw] 32 or 64 bit addressing
     *
     * Indicates addressing for the prefetchable memory base address (start address).
     *
     * 0 = 32-bit memory addressing\n
     * 1 = 64-bit memory addressing
     *
     * Field size: 1 bit
     */
    uint8_t baseAddr;
} Pcie_PrefMemReg;

/**
 * \brief Specification of the Prefetchable Memory Base Upper Register
 */
typedef struct pciePrefBaseUpperReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Base upper 32bits
     *
     * Upper 32 bits of Prefetchable Memory Base Address. Used with 64bit
     * prefetchable memory addressing only.
     *
     * Field size: 32 bits
     */
    uint32_t base;
} Pcie_PrefBaseUpperReg;

/**
 * \brief Specification of the Prefetchable Memory Limit Upper Register
 */
typedef struct pciePrefLimitUpperReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Base upper 32bits
     *
     * Upper 32 bits of Prefetchable Memory Limit Address. Used with 64 bit
     * prefetchable memory addressing only.
     *
     * Field size: 32 bits
     */
    uint32_t limit;
} Pcie_PrefLimitUpperReg;

/**
 * \brief Specification of the IO Base and Limit Upper 16 bits Register
 */
typedef struct pcieType1IOSpaceReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Upper 16 bits of IO Base Address.
     *
     * Used with 32 bit IO space addressing only.
     *
     * Field size: 16 bits
     */
    uint16_t IOBase;
    /**
     * [rw] Upper 16 bits of IO Limit Address.
     *
     * Used with 32 bit IO space addressing only.
     *
     * Field size: 16 bits
     */
    uint16_t IOLimit;
} Pcie_Type1IOSpaceReg;

/**
 * \brief Specification of the Capabilities Pointer Register
 */
typedef struct pcieType1CapPtrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] First Capability Pointer.
     *
     * By default, it points to Power Management Capability structure.
     * Writable from internal bus interface.
     *
     * Field size: 8 bits
     */
    uint8_t capPtr;
} Pcie_Type1CapPtrReg;

/**
 * \brief Specification of the Expansion ROM Base Address Register
 */
typedef struct pcieType1ExpnsnRomReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Address of Expansion ROM
     *
     * Field size: 21CAPPTR bits [0-0x1FFFFF]
     */
    uint32_t expRomBaseAddr;
    /**
     * [rw] Expansion ROM enable
     *
     * Field size: 1 bit
     */
    uint8_t expRomEn;
} Pcie_Type1ExpnsnRomReg;

/**
 * \brief Specification of the Bridge Control and Interrupt Register
 */
typedef struct pcieType1BridgeIntReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Discard Timer SERR Enable Status.
     *
     * Not Applicable to PCI Express. Hardwired to 0.
     *
     * Field size: 1 bit
     */
    uint8_t serrEnStatus;
    /**
     * [ro] Discard Timer Status.
     *
     * Not applicable to PCI Express. Hardwired to 0.
     *
     * Field size: 1 bit
     */
    uint8_t timerStatus;
    /**
     * [ro] Secondary Discard Timer.
     *
     * Not applicable to PCI Express. Hardwired to 0.
     *
     * Field size: 1 bit
     */
    uint8_t secTimer;
    /**
     * [ro] Primary Discard Timer.
     *
     * Not applicable to PCI Express. Hardwired to 0.
     *
     * Field size: 1 bit
     */
    uint8_t priTimer;
    /**
     * [ro] Fast Back to Back Transactions Enable.
     *
     * Not applicable to PCI Express. Hardwired to 0.
     *
     * Field size: 1 bit
     */
    uint8_t b2bEn;
    /**
     * [rw] Secondary Bus Reset.
     *
     * Field size: 1 bit
     */
    uint8_t secBusRst;
    /**
     * [ro] Master Abort Mode.
     *
     * Not applicable to PCI Express. Hardwired to 0.
     *
     * Field size: 1 bit
     */
    uint8_t mstAbortMode;
    /**
     * [rw] VGA 16 bit Decode
     *
     * Field size: 1 bit
     */
    uint8_t vgaDecode;
    /**
     * [rw] VGA Enable
     *
     * Field size: 1 bit
     */
    uint8_t vgaEn;
    /**
     * [rw] ISA Enable
     *
     * Field size: 1 bit
     */
    uint8_t isaEn;
    /**
     * [rw] SERR Enable.
     *
     * Set to enable forwarding of ERR_COR, ERR_NONFATAL and ERR_FATAL messages.
     *
     * Field size: 1 bit
     */
    uint8_t serrEn;
    /**
     * [rw] Parity Error Response Enable.
     *
     * This bit controls the logging of poisoned TLPs in
     * @ref pcieType1SecStatReg_s::mstDPErr
     *
     * Field size: 1 bit
     */
    uint8_t pErrRespEn;
    /**
     * [rw] Interrupt Pin.
     *
     * It identifies the legacy interrupt message that the device uses.
     * For single function configuration, the core only uses INTA. This register
     * is writable through internal bus interface.
     *
     *  0  = Legacy interrupt is not being used
     *  1h = INTA
     *  2h = INTB
     *  3h = INTC
     *  4h = INTD
     * Others = Reserved.
     *
     * Field size: 8 bits
     */
    uint8_t intPin;
    /**
     * [rw] Interrupt Line. Value is system software specified.
     *
     * Field size: 8 bits
     */
    uint8_t intLine;
} Pcie_Type1BridgeIntReg;


/*****************************************************************************
 **********  Power Management Capabilities REGISTERS  ************************
 ****************************************************************************/

/**
 * \brief Specification of the Power Management Capability Register
 */
typedef struct pciePMCapReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] PME Support.
     *
     * Identifies the power states from which generates PME Messages. A value of
     * 0 for any bit indicates that the device (or function) is not capable
     * of generating PME Messages while in that power state.
     *
     * bit 0x10: If set, PME Messages can be generated from D3cold.\n
     * bit 0x08: If set, PME Messages can be generated from D3hot.\n
     * bit 0x04: If set, PME Messages can be generated from D2.\n
     * bit 0x02: If set, PME Messages can be generated from D1.\n
     * bit 0x01: If set, PME Messages can be generated from D0.
     *
     * Field size: 5 bits
     *
     */
    uint8_t pmeSuppN;
    /**
     * [rw] D2 Support.
     *
     * Field size: 1 bit
     *
     */
    uint8_t d2SuppN;
    /**
     * [rw] D1 Support.
     *
     * Field size: 1 bit
     *
     */
    uint8_t d1SuppN;
    /**
     * [rw] Auxiliary Current
     *
     * Field size: 3 bits
     *
     */
    uint8_t auxCurrN;
    /**
     * [rw] Device Specific Initialization
     *
     * Field size: 1 bit
     *
     */
    uint8_t dsiN;
    /**
     * [ro] PME clock.  Hardwired to zero.
     *
     * Field size: 1 bit
     *
     */
    uint8_t pmeClk;
    /**
     * [rw] Power Management Specification Version
     *
     * Field size: 3 bits
     *
     */
    uint8_t pmeSpecVer;
    /**
     * [rw] Next capability pointer.
     *
     * By default, it points to Message Signaled Interrupt structure.
     *
     * Field size: 8 bits
     *
     */
    uint8_t pmNextPtr;
    /**
     * [ro] Power Management Capability ID.
     *
     * Field size: 8 bits
     *
     */
    uint8_t pmCapID;
} Pcie_PMCapReg;

/**
 * \brief Specification of the Power Management Capabilities Control and Status Register
 */
typedef struct pciePMCapCtlStatReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Data register for additional information. Not supported.
     *
     * Field size: 8 bits
     *
     */
    uint8_t dataReg;
    /**
     * [ro] Bus Power/Clock Control Enable. Hardwired to zero.
     *
     * Field size: 1 bit
     *
     */
    uint8_t clkCtrlEn;
    /**
     * [ro] B2 and B3 support. Hardwired to zero.
     *
     * Field size: 1 bit
     *
     */
    uint8_t b2b3Support;
    /**
     * [rw] PME Status. Indicates if a previously enabled PME event occurred or not.
     *
     * Write 1 to clear.
     *
     * Field size: 1 bit
     *
     */
    uint8_t pmeStatus;
    /**
     * [ro] Data Scale. Not supported.
     *
     * Field size: 2 bits
     *
     */
    uint8_t dataScale;
    /**
     * [ro] Data Select. Not supported.
     *
     * Field size: 4 bits
     *
     */
    uint8_t dataSelect;
    /**
     * [rw] PME Enable. Value of 1 indicates device is enabled to generate PME.
     *
     * Field size: 1 bit
     *
     */
    uint8_t pmeEn;
    /**
     * [rw] No Soft Reset.
     *
     * It is set to disable reset during a transition from D3 to D0.
     *
     * Field size: 1 bit
     *
     */
    uint8_t noSoftRst;
    /**
     * [rw] Power State.
     *
     * Controls the device power state. Writes are ignored if the state is not
     * supported.
     * 0 = D0 power state
     * 1h = D1 power state
     * 2h = D2 power state
     * 3h = D3 power states
     *
     * Field size: 2 bits
     *
     */
    uint8_t pwrState;
} Pcie_PMCapCtlStatReg;


/*****************************************************************************
 **********  Message Signaling Interrupt  REGISTERS  *************************
 ****************************************************************************/

/**
 * \brief Specification of the MSI capabilities Register
 */
typedef struct pcieMsiCapReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSI Per Vector Masking supported
     *
     * Field size: 1 bit
     *
     */
    uint8_t extDataEn;
    /**
     * [ro] Extended message data capable
     *
     * Field size: 1 bit
     *
     */
    uint8_t extDataCap;
    /**
     * [ro] MSI Per Vector Masking supported
     *
     * Field size: 1 bit
     *
     */
    uint8_t pvmEn;
    /**
     * [rw] 64bit addressing enabled
     *
     * Field size: 1 bit
     *
     */
    uint8_t en64bit;
    /**
     * [rw] Multiple Msg enabled
     *
     * Indicates that multiple message mode is enabled by software. Number
     * of messages enabled must not be greater than @ref multMsgCap
     *
     * <TABLE>
     * <TR><TH>@ref multMsgEn</TH><TH>Number of messages</TH></TR>
     * <TR><TD>0</TD>             <TD>1</TD></TR>
     * <TR><TD>1</TD>             <TD>2</TD></TR>
     * <TR><TD>2</TD>             <TD>4</TD></TR>
     * <TR><TD>3</TD>             <TD>8</TD></TR>
     * <TR><TD>4</TD>             <TD>16</TD></TR>
     * <TR><TD>5</TD>             <TD>32</TD></TR>
     * <TR><TD>others</TD>        <TD>reserved</TD></TR>
     * </TABLE>
     *
     * Field size: 3 bits
     */
    uint8_t multMsgEn;
    /**
     * [rw] Multipe Msg capable
     *
     * Multiple message capable.
     *
     * <TABLE>
     * <TR><TH>@ref multMsgCap</TH><TH>Number of messages</TH></TR>
     * <TR><TD>0</TD>              <TD>1</TD></TR>
     * <TR><TD>1</TD>              <TD>2</TD></TR>
     * <TR><TD>2</TD>              <TD>4</TD></TR>
     * <TR><TD>3</TD>              <TD>8</TD></TR>
     * <TR><TD>4</TD>              <TD>16</TD></TR>
     * <TR><TD>5</TD>              <TD>32</TD></TR>
     * <TR><TD>others</TD>         <TD>reserved</TD></TR>
     * </TABLE>
     *
     * Field size: 3 bits
     */
    uint8_t multMsgCap;
    /**
     * [rw] MSI enabled
     *
     * MSI Enabled. When set, INTx must be disabled.
     *
     * Field size: 1 bit
     */
    uint8_t msiEn;
    /**
     * [rw] Next capability pointer
     *
     * By default, it points to PCI Express Capabilities structure.
     *
     * Field size: 8 bits
     */
    uint8_t nextCap;
    /**
     * [ro] MSI capability ID
     *
     * Field size: 8 bits
     */
    uint8_t capId;
} Pcie_MsiCapReg;


/**
 * \brief Specification of the MSI lower 32 bits Register
 */
typedef struct pcieMsiLo32Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Lower 32bits address
     *
     * Field size: 30 bits
     *
     */
    uint32_t addr;
} Pcie_MsiLo32Reg;

/**
 * \brief Specification of the MSI upper 32 bits Register
 */
typedef struct pcieMsiUp32Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Upper 32bits address
     *
     * Field size: 32 bits
     *
     */
    uint32_t addr;
} Pcie_MsiUp32Reg;

/**
 * \brief Specification of the MSI Data Register
 */
typedef struct pcieMsiDataReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSI data
     *
     * Field size: 16 bits
     *
     */
    uint16_t data;
} Pcie_MsiDataReg;

/**
 * \brief Specification of the MSI_CAP_OFF_14H_REG Register
 */
typedef struct pcieMsiCapOff10H {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSI data
     *
     * Field size: 32 bits
     *
     */
    uint32_t data;
} Pcie_MsiCapOff10HReg;

/**
 * \brief Specification of the MSI Data Register
 */
typedef struct pcieMsiCapOff14H {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSI data
     *
     * Field size: 32 bits
     *
     */
    uint32_t data;
} Pcie_MsiCapOff14HReg;

/**
 * \brief Specification of the MSIx capabilities Register
 */
typedef struct pcieMsixCapReg_s {
    /** [rw] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSIx enabled
     *
     *
     * Field size: 1 bit
     */
    uint8_t msixEn;
    /**
     * [rw] Mask interrpt for the function
     *
     * MSIx IRQ mask
     *
     * Field size: 1 bit
     */
    uint8_t maskIrq;
    /**
     * [rw] MSIx table size
     *
     * Field size: 11 bits
     */
    uint16_t msixTblSize;
    /**
     * [ro] Next capability pointer
     *
     * By default, it points to PCI Express Capabilities structure.
     *
     * Field size: 8 bits
     */
    uint8_t nextCap;
    /**
     * [rw] MSIx capability ID
     *
     * Field size: 8 bits
     */
    uint8_t capId;
} Pcie_MsixCapReg;

/**
 * \brief Specification of PCI Express MSIx table offset register
 */
typedef struct pcieMsixTblOffset_s {
    /** [rw] Raw image of register on read; actual value on write */
    uint32_t raw;
    /** [rw] Offset of memory address where MSIx table is located relative
     * to selected BAR */
    uint32_t offset;
    /** [rw] Bar corresponding to memory address where MSIx table is located */
    uint8_t barIndex;
} Pcie_MsixTblOffset;



/*****************************************************************************
 **********  PCIe CAPABILITIES  REGISTERS ************************************
 ****************************************************************************/
/**
 * \brief Specification of the PCI Express Capabilities Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pciePciesCapReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Interrupt Message Number. Updated by hardware and writable through internal bus Interface.
     *
     * Field size: 5 bits
     */
    uint8_t intMsg;
    /**
     * [rw] Slot Implemented. Writable from internal bus interface.
     *
     * Field size: 1 bit
     */
    uint8_t sltImplN;
    /**
     * [rw] Device Port Type.
     *
     * 0 = EP type\n
     * 4h = RC type\n
     * Others = Reserved
     *
     * Field size: 4 bits
     */
    uint8_t dportType;
    /**
     * [rw] PCI Express Capability Version
     *
     * Field size: 4 bits
     */
    uint8_t pcieCap;
    /**
     * [rw] Next capability pointer. Writable from internal bus interface.
     *
     * Field size: 8 bits
     */
    uint8_t nextCap;
    /**
     * [rw] PCIe Capability ID.
     *
     * Field size: 8 bits
     */
    uint8_t capId;
} Pcie_PciesCapReg;

/**
 * \brief Specification of the Device Capabilities Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieDeviceCapReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Function Level Reset Capability
     *
     * used on EP only where it is rw.  On RC reserved and should be 0
     *
     * Field size: 1 bit
     */
    uint8_t flrEn;
    /**
     * [rw] Captured Slot Power Limit Scale. For upstream ports (EP ports) only.
     *
     * Field size: 2 bits
     */
    uint8_t pwrLimitScale;
    /**
     * [rw] Captured Slow Power Limit Value. For upstream ports (EP ports) only.
     *
     * Field size: 8 bits
     */
    uint8_t pwrLimitValue;
    /**
     * [rw] Role-based Error Reporting. Writable from internal bus interface.
     *
     * Field size: 1 bit
     */
    uint8_t errRpt;
    /**
     * [rw] Endpoint L1 Acceptable Latency. Must be 0 in RC mode. It is 3h for EP mode.
     *
     * Field size: 3 bits
     */
    uint8_t l1Latency;
    /**
     * [rw] Endpoint L0s Acceptable Latency. Must be 0 in RC mode. It is 4h for EP mode.
     *
     * Field size: 3 bits
     */
    uint8_t l0Latency;
    /**
     * [rw] Extended Tag Field Supported. Writable from internal interface
     *
     * Field size: 1 bit
     */
    uint8_t extTagFld;
    /**
     * [rw] Phantom Field Supported. Writable from internal bus interface.
     *
     * Field size: 2 bits
     */
    uint8_t phantomFld;
    /**
     * [rw] Maximum Payload size supported. Writable from internal bus interface.
     *
     * Field size: 3 bits
     */
    uint8_t maxPayldSz;
} Pcie_DeviceCapReg;

/**
 * \brief Specification of the Device Status and Control Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieDevStatCtrlReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Transaction Pending
     *
     * Field size: 1 bit
     */
    uint8_t tpend;
    /**
     * [ro] Auxiliary Power Detected
     *
     * Field size: 1 bit
     */
    uint8_t auxPwr;
    /**
     * [rw] Unsupported Request Detected
     *
     * Field size: 1 bit
     */
    uint8_t rqDet;
    /**
     * [rw] Fatal Error Detected
     *
     * Field size: 1 bit
     */
    uint8_t fatalEr;
    /**
     * [rw] Non-fatal Error Detected
     *
     * Field size: 1 bit
     */
    uint8_t nFatalEr;
    /**
     * [rw] Correctable Error Detected
     *
     * Field size: 1 bit
     */
    uint8_t corrEr;
    /**
     * [rw] Initiate Function Level Reset (for EP)
     *
     * Field size: 3 bits
     */
    uint8_t initFLR;
    /**
     * [rw] Maximum Read Request Size
     *
     * Field size: 3 bits
     */
    uint8_t maxSz;
    /**
     * [rw] Enable no snoop
     *
     * Field size: 1 bit
     */
    uint8_t noSnoop;
    /**
     * [rw] AUX Power PM Enable
     *
     * Field size: 1 bit
     */
    uint8_t auxPwrEn;
    /**
     * [rw] Phantom Function Enable
     *
     * Field size: 1 bit
     */
    uint8_t phantomEn;
    /**
     * [rw] Extended Tag Field Enable
     *
     * Field size: 1 bit
     */
    uint8_t xtagEn;
    /**
     * [rw] Maximum Payload Size
     *
     * Field size: 3 bits
     */
    uint8_t maxPayld;
    /**
     * [rw] Enable Relaxed Ordering
     *
     * Field size: 1 bit
     */
    uint8_t relaxed;
    /**
     * [rw] Enable Unsupported Request Reporting
     *
     * Field size: 1 bit
     */
    uint8_t reqRp;
    /**
     * [rw] Fatal Error Reporting Enable
     *
     * Field size: 1 bit
     */
    uint8_t fatalErRp;
    /**
     * [rw] Non-fatal Error Reporting Enable
     *
     * Field size: 1 bit
     */
    uint8_t nFatalErRp;
    /**
     * [rw] Correctable Error Reporting Enable
     *
     * Field size: 1 bit
     */
    uint8_t corErRp;
} Pcie_DevStatCtrlReg;

/**
 *  \brief Specification of the Link Capabilities Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieLinkCapReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Port Number. Writable from internal bus interface.
     *
     * Field size: 8 bits
     */
    uint8_t portNum;
    /**
     * [rw] ASPM Optionality Compliance
     *
     * Field size: 1 bit
     */
    uint8_t aspmOptComp;
    /**
     * [rw] Bandwidth Notification Capable.
     *
     * 0 = For upstream ports (EP ports)\n
     * 1 = For downstream ports (RC ports)
     *
     * Field size: 1 bit
     */
    uint8_t bwNotifyCap;
    /**
     * [rw] Link Layer Active Reporting Capable.
     *
     * 0 = For upstream ports (EP ports)\n
     * 1 = For downstream ports (RC ports)
     *
     * Field size: 1 bit
     */
    uint8_t dllRepCap;
    /**
     * [rw] Surprise Down Error Reporting Capable. Not supported. Always zero.
     *
     * Field size: 1 bit
     */
    uint8_t downErrRepCap;
    /**
     * [rw] Clock Power Management. Writable from internal bus interface.
     *
     * For upstream ports (EP Ports), a value of 1h in this bit indicates that
     * the component tolerates the removal of any reference clock(s) in the L1
     * and L2/L3 Ready Link states. A value of 0 indicates the reference
     * clock(s) must not be removed in these Link states.
     *
     * For downstream ports (RC Ports), this bit is always 0.
     *
     * Field size: 1 bit
     */
    uint8_t clkPwrMgmt;
    /**
     * [rw] L1 Exit Latency when common clock is used. Writable from internal bus interface.
     *
     * <TABLE>
     * <TR><TH>@ref l1ExitLat</TH><TH>low range</TH><TH>high range</TH></TR>
     * <TR><TD>0</TD>             <TD>0</TD>        <TD>64 ns</TD></TR>
     * <TR><TD>1</TD>             <TD>64ns</TD>     <TD>128ns</TD></TR>
     * <TR><TD>2</TD>             <TD>128ns</TD>    <TD>256ns</TD></TR>
     * <TR><TD>3</TD>             <TD>256ns</TD>    <TD>512ns</TD></TR>
     * <TR><TD>4</TD>             <TD>512ns</TD>    <TD>1�s</TD></TR>
     * <TR><TD>5</TD>             <TD>1�s</TD>      <TD>2�s</TD></TR>
     * <TR><TD>6</TD>             <TD>2�s</TD>      <TD>4�s</TD></TR>
     * <TR><TD>7</TD>             <TD>4�s</TD>      <TD>and up</TD></TR>
     * </TABLE>
     *
     * Field size: 3 bits
     */
    uint8_t l1ExitLat;
    /**
     * [rw] L0s Exit Latency. Writable from internal bus interface.
     *
     * <TABLE>
     * <TR><TH>@ref l1ExitLat</TH><TH>low range</TH><TH>high range</TH></TR>
     * <TR><TD>0</TD>             <TD>0</TD>        <TD>64 ns</TD></TR>
     * <TR><TD>1</TD>             <TD>64ns</TD>     <TD>128ns</TD></TR>
     * <TR><TD>2</TD>             <TD>128ns</TD>    <TD>256ns</TD></TR>
     * <TR><TD>3</TD>             <TD>256ns</TD>    <TD>512ns</TD></TR>
     * <TR><TD>4</TD>             <TD>512ns</TD>    <TD>1�s</TD></TR>
     * <TR><TD>5</TD>             <TD>1�s</TD>      <TD>2�s</TD></TR>
     * <TR><TD>6</TD>             <TD>2�s</TD>      <TD>4�s</TD></TR>
     * <TR><TD>7</TD>             <TD>4�s</TD>      <TD>and up</TD></TR>
     * </TABLE>
     *
     * Field size: 3 bits
     */
    uint8_t losExitLat;
    /**
     * [rw] Active State Link Power Management Support. Writable from internal bus interface.
     *
     * 1h = L0s entry supported.\n
     * 3h = L0s and L1 supported.\n
     * Others = Reserved.
     *
     * Field size: 2 bits
     */
    uint8_t asLinkPm;
    /**
     * [rw] Maximum Link Width. Writable from internal bus interface.
     *
     * 1h = �1\n
     * 2h = �2\n
     * Others = Reserved.
     *
     * Field size: 6 bits
     */
    uint8_t maxLinkWidth;
    /**
     * [rw] Maximum Link Speed. Writable from internal bus interface.
     *
     * 1h = 2.5GT/s Link speed supported.\n
     * 2h = 5.0 GT/s and 2.5 GT/s Link speeds supported.\n
     * Others = Reserved.
     *
     * Field size: 4 bits
     */
    uint8_t maxLinkSpeed;
} Pcie_LinkCapReg;

/**
 * \brief Specification of the Link Status and Control Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieLinkStatCtrlReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Link Autonomous Bandwidth Status.
     *
     * This bit is Set by hardware to indicate that hardware has autonomously
     * changed Link speed or width, without the Port transitioning through
     * DL_Down status, for reasons other than to attempt to correct unreliable
     * Link operation.  This bit must be set if the Physical Layer reports a
     * speed or width change was initiated by the downstream component that
     * was indicated as an autonomous change.
     *
     * Not applicable and reserved for EP.
     *
     * Field size: 1 bit
     */
    uint8_t linkBwStatus;
    /**
     * [rw] Link Bandwidth Management Status.
     *
     * This bit is Set by hardware to indicate that either of the following
     * has occurred without the Port transitioning through DL_Down status:
     *
     * - A Link retraining has completed following a write of 1b to the
     *   Retrain Link bit
     * - Hardware has changed Link speed or width to attempt to correct
     *   unreliable Link operation, either through an LTSSM timeout or
     *   a higher level process.
     *
     * This bit must be set if the Physical Layer reports a speed or width
     * change was initiated by the downstream component that was not
     * indicated as an autonomous change.
     *
     * Not applicable and reserved for EP.
     *
     * Field size: 1 bit
     */
    uint8_t linkBwMgmtStatus;
    /**
     * [rw] Data Link Layer Active
     *
     * This bit indicates the status of the Data Link Control and
     * Management State Machine. It returns a 1 to indicate the DL_Active state,
     * 0 otherwise.
     *
     * Field size: 1 bit
     */
    uint8_t dllActive;
    /**
     * [rw] Slot Clock Configuration. Writable from internal bus interface.
     *
     * This bit indicates that the component uses the same
     * physical reference clock that the platform provides on the connector.
     *
     * Field size: 1 bit
     */
    uint8_t slotClkCfg;
    /**
     * [rw] Link Training. Not applicable to EP.
     *
     * Field size: 1 bit
     */
    uint8_t linkTraining;
    /**
     * [rw] Undefined for PCI Express.
     *
     * Field size: 1 bit
     */
    uint8_t undef;
    /**
     * [rw] Negotiated Link Width. Set automatically by hardware after link initialization.
     *
     * Field size: 6 bits
     */
    uint8_t negotiatedLinkWd;
    /**
     * [rw] Link Speed. Set automatically by hardware after link initialization.
     *
     * Field size: 4 bits
     */
    uint8_t linkSpeed;
    /**
     * [rw]  DRS Signalling Control
     *
     * Field size: 1 bit
     */
    uint8_t drsSigCtrl;
    /**
     * [rw]  Link Autonomous Bandwidth Interrupt Enable. Not applicable and is reserved for EP
     *
     * Field size: 1 bit
     */
    uint8_t linkBwIntEn;
    /**
     * [rw] Link Bandwidth Management Interrupt Enable. Not applicable and is reserved for EP.
     *
     * Field size: 1 bit
     */
    uint8_t linkBwMgmtIntEn;
    /**
     * [rw] Hardware Autonomous Width Disable. Not supported and hardwired to zero.
     *
     * Field size: 1 bit
     */
    uint8_t hwAutoWidthDis;
    /**
     * [rw] Enable Clock Power Management.
     *
     * Field size: 1 bit
     */
    uint8_t clkPwrMgmtEn;
    /**
     * [rw] Extended Synchronization.
     *
     * Field size: 1 bit
     */
    uint8_t extSync;
    /**
     * [rw] Common Clock Configuration.
     *
     * 0 = Indicates that this device and the device at the opposite end of the
     * link are operating with separate reference clock sources.\n
     * 1 = Indicates that this device and the device at the opposite end of the
     * link are operating with a common clock source.
     *
     * Field size: 1 bit
     */
    uint8_t commonClkCfg;
    /**
     * [rw] Retrain Link. Not applicable and reserved for EP.
     *
     * Field size: 1 bit
     */
    uint8_t retrainLink;
    /**
     * [rw] Disables the link by directing the LTSSM to the Disabled state when set.
     *
     * Field size: 1 bit
     */
    uint8_t linkDisable;
    /**
     * [rw] Read Completion Boundary.
     *
     * 0 = 64 bytes\n
     * 1 = 128 bytes
     *
     * Field size: 1 bit
     */
    uint8_t rcb;
    /**
     * [rw] Active State Link Power Management Control
     *
     * 0 = Disabled.\n
     * 1h = L0s entry enabled.\n
     * 2h = L1 entry enabled.\n
     * 3h = L0s and L1 entry enabled.\n
     *
     * Field size: 2 bits
     */
    uint8_t activeLinkPm;
} Pcie_LinkStatCtrlReg;

/**
 * \brief Specification of the Slot Capabilities register
 *
 * This register may only be used for root complex mode.
 */
typedef struct pcieSlotCapReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Physical Slot Number.
     *
     * Field size: 13 bits [0-0x1FFF]
     */
    uint16_t slotNum;
    /**
     * [rw] No Command Complete Support
     *
     * When Set, this bit indicates that this slot does not generate software
     * notification when an issued command is completed by the Hot-Plug Controller
     *
     * Field size: 1 bit
     */
    uint8_t cmdCompSupp;
    /**
     * [rw] Electromechanical Interlock Present.
     *
     * When Set, this bit indicates that an Electromechanical Interlock
     * is implemented on the chassis for this slot.
     *
     * Field size: 1 bit
     */
    uint8_t emlPresent;
    /**
     * [rw] Slot Power Limit Scale.
     *
     * Field size: 2 bits
     */
    uint8_t pwrLmtScale;
    /**
     * [rw] Slot Power Limit Value.
     *
     * Field size: 8 bits
     */
    uint8_t pwrLmtValue;
    /**
     * [rw] Hot Plug Capable.
     *
     * Field size: 1 bit
     */
    uint8_t hpCap;
    /**
     * [rw] Hot Plug Surprise.
     *
     * Field size: 1 bit
     */
    uint8_t hpSurprise;
    /**
     * [rw] Power Indicator Present.
     *
     * Field size: 1 bit
     */
    uint8_t pwrInd;
    /**
     * [rw] Attention Indicator Present.
     *
     * Field size: 1 bit
     */
    uint8_t attnInd;
    /**
     * [rw] MRL Sensor Present.
     *
     * Field size: 1 bit
     */
    uint8_t mrlSensor;
    /**
     * [rw] Power Controller Present.
     *
     * If there is no power controller, software must ensure that system power
     * is up before reading Presence Detect state
     *
     * Field size: 1 bit
     */
    uint8_t pwrCtl;
    /**
     * [rw] Attention Indicator Present.
     *
     * Field size: 1 bit
     */
    uint8_t attnButton;
} Pcie_SlotCapReg;

/**
 * \brief Specification of the Slot Status and Control register
 *
 * This register may only be used for root complex mode.
 */
typedef struct pcieSlotStatCtrlReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Data Link Layer State Changed
     *
     * Write 1 to clear.
     *
     * Field size: 1 bit
     */
    uint8_t dllState;
    /**
     * [ro] Electromechanical Lock Status
     *
     * Field size: 1 bit
     */
    uint8_t emLock;
    /**
     * [ro] Presence Detect State
     *
     * Field size: 1 bit
     */
    uint8_t presenceDet;
    /**
     * [ro] MRL Sensor State
     *
     * Field size: 1 bit
     */
    uint8_t mrlState;
    /**
     * [rw] Command Completed
     *
     * Write 1 to clear.
     *
     * Field size: 1 bit
     */
    uint8_t cmdComplete;
    /**
     * [rw] Presence Detect Changed
     *
     * Write 1 to clear.
     *
     * Field size: 1 bit
     */
    uint8_t presenceChg;
    /**
     * [rw] MRL Sensor Changed
     *
     * Write 1 to clear.
     *
     * Field size: 1 bit
     */
    uint8_t mrlChange;
    /**
     * [rw] Power Fault Detected
     *
     * Write 1 to clear.
     *
     * Field size: 1 bit
     */
    uint8_t pwrFault;
    /**
     * [rw] Attention Button Pressed.
     *
     * Write 1 to clear.
     *
     * Field size: 1 bit
     */
    uint8_t attnPressed;
    /**
     * [rw] Data Link Layer State Changed Enable.
     *
     * Field size: 1 bit
     */
    uint8_t dllChgEn;
    /**
     * [rw] Electromechanical Interlock Control.
     *
     * Field size: 1 bit
     */
    uint8_t emLockCtl;
    /**
     * [rw] Power Controller Control
     *
     * Field size: 1 bit
     */
    uint8_t pmCtl;
    /**
     * [rw] Power Indicator Control
     *
     * Field size: 2 bits
     */
    uint8_t pmIndCtl;
    /**
     * [rw] Attention Indicator Control.
     *
     * Field size: 2 bits
     */
    uint8_t attnIndCtl;
    /**
     * [rw] Hot Plug Interrupt Enable.
     *
     * Field size: 1 bit
     */
    uint8_t hpIntEn;
    /**
     * [rw] Command Completed Interrupt Enable.
     *
     * Field size: 1 bit
     */
    uint8_t cmdCmpIntEn;
    /**
     * [rw] Presence Detect Changed Enable.
     *
     * Field size: 1 bit
     */
    uint8_t prsDetChgEn;
    /**
     * [rw] MRL Sensor Changed Enable.
     *
     * Field size: 1 bit
     */
    uint8_t mrlChgEn;
    /**
     * [rw] Power Fault Detected Enable.
     *
     * Field size: 1 bit
     */
    uint8_t pwrFltDetEn;
    /**
     * [rw] Attention Button Pressed Enable.
     *
     * Field size: 1 bit
     */
    uint8_t attnButtEn;
} Pcie_SlotStatCtrlReg;

/**
 * \brief Specification of the Root Control and Capabilities Register
 *
 * This register may only be used for root complex mode.
 */
typedef struct pcieRootCtrlCapReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] CRS Software Visibility. Not supported and set to 0.
     *
     * Field size: 1 bit
     */
    uint8_t crsSw;
    /**
     * [ro] CRS Software Visibility Enable. Not supported and set to 0x0.
     *
     * Field size: 1 bit
     */
    uint8_t crsSwEn;
    /**
     * [rw] PME Interrupt Enable
     *
     * Field size: 1 bit
     */
    uint8_t pmeIntEn;
    /**
     * [rw] System Error on Fatal Error Enable
     *
     * Field size: 1 bit
     */
    uint8_t serrFatalErr;
    /**
     * [rw] System Error on Non-fatal Error Enable
     *
     * Field size: 1 bit
     */
    uint8_t serrNFatalErr;
    /**
     * [rw] System Error on Correctable Error Enable
     *
     * Field size: 1 bit
     */
    uint8_t serrEn;
} Pcie_RootCtrlCapReg;

/**
 * \brief Specification of the Root Status and Control register
 *
 * This register may only be used for root complex mode.
 */
typedef struct pcieRootStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Indicates that another PME is pending when the PME Status bit is Set.
     *
     * Field size: 1 bit
     */
    uint8_t pmePend;
    /**
     * [rw] Indicates that PME was asserted by the PME Requester.
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t pmeStatus;
    /**
     * [ro] ID of the last PME Requester.
     *
     * This field is only valid when the PME Status bit is Set.
     *
     * Field size: 16 bits
     */
    uint16_t pmeReqID;
} Pcie_RootStatusReg;

/**
 * \brief Specification of the Device Capabilities 2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieDevCap2Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Completion timeout disable supported
     *
     * Field size: 1 bit
     */
    uint8_t cmplToDisSupp;
    /**
     * [rw] Completion timeout ranges supported. Applicable to RC/EP that issue requests on own behalf.
     *
     * Field size: 4 bits
     */
    uint8_t cmplToEn;
    /**
     * [ro] ARI Forwarding Supported
     *
     * Field size: 1 bit
     */
    uint8_t ariFwdSp;
    /**
     * [ro] AtomicOp Routing Supported
     *
     * Field size: 1 bit
     */
    uint8_t aorSp;
    /**
     * [ro] 32-bit AtomicOp Completer Supported
     *
     * Field size: 1 bit
     */
    uint8_t aoc32Sp;
    /**
     * [ro] 64-bit AtomicOp Completer Supported
     *
     * Field size: 1 bit
     */
    uint8_t aoc64Sp;
    /**
     * [ro] 128-bit CAS Completer Supported
     *
     * Field size: 1 bit
     */
    uint8_t casc128Sp;
    /**
     * [ro] No RO-enabled PR-PR Passing
     *
     * Field size: 1 bit
     */
    uint8_t noRoPR;
    /**
     * [ro] LTR Mechanism Supported
     *
     * Field size: 2 bit
     */
    uint8_t ltrSupp;
    /**
     * [ro] TPH Completer Supported
     *
     * Field size: 2 bit
     */
    uint8_t tphcSp;
    /**
     * [ro] LN System CLS
     *
     * Field size: 1 bit
     */
    uint8_t lnSysCls;
    /**
     * [ro] 10-Bit Tag Completer Supported (ep only)
     *
     * Field size: 1 bit
     */
    uint8_t tag10bitCompSupp;
    /**
     * [ro] 10-Bit Tag Requester Supported
     *
     * Field size: 1 bit
     */
    uint8_t tag10bitReqSupp;
    /**
     * [ro] [OBFF] Optimized Buffer Flush/fill Supported
     *
     * Field size: 1 bit
     */
    uint8_t obffSupp;
} Pcie_DevCap2Reg;

/**
 * \brief Specification of the Device Status and Control Register 2
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieDevStatCtrl2Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Completion timeout disable
     *
     * Field size: 1 bit
     */
    uint8_t cmplToDis;
    /**
     * [rw] Completion timeout value.
     *
     * It is strongly recommended that the Completion Timeout mechanism
     * not expire in less than 10 ms.
     *
     * <TABLE>
     * <TR><TH>@ref cmplTo</TH><TH>low range</TH><TH>high range</TH></TR>
     * <TR><TD>0x0</TD>        <TD>50ms</TD>     <TD>50s</TD></TR>
     * <TR><TD>0x1</TD>        <TD>50s</TD>      <TD>100s</TD></TR>
     * <TR><TD>0x2</TD>        <TD>1ms</TD>      <TD>10ms</TD></TR>
     * <TR><TD>0x5</TD>        <TD>16ms</TD>     <TD>55ms</TD></TR>
     * <TR><TD>0x6</TD>        <TD>65ms</TD>     <TD>210ms</TD></TR>
     * <TR><TD>0x9</TD>        <TD>260ms</TD>    <TD>900ms</TD></TR>
     * <TR><TD>0xA</TD>        <TD>1s</TD>       <TD>3.5s</TD></TR>
     * <TR><TD>0xD</TD>        <TD>4s</TD>       <TD>13s</TD></TR>
     * <TR><TD>0xE</TD>        <TD>17s</TD>      <TD>64s</TD></TR>
     * <TR><TD>others</TD>     <TD>reserved</TD> <TD>reserved</TD></TR>
     * </TABLE>
     *
     * Field size: 4 bits
     */
    uint8_t cmplTo;
    /**
     * [rw] ARI Forwarding Supported
     *
     * Field size: 1 bit
     */
    uint8_t ariFwdSp;
    /**
     * [rw] AtomicOp Requester Enabled
     *
     * Field size: 1 bit
     */
    uint8_t aopReqEn;
    /**
     * [rw] AtomicOp Egress Blocking
     *
     * Field size: 1 bit
     */
    uint8_t aopEgBlk;
    /**
     * [rw] IDO Request Enable
     *
     * Field size: 1 bit
     */
    uint8_t idoReqEn;
    /**
     * [rw] IDO Completion Enable
     *
     * Field size: 1 bit
     */
    uint8_t idoCplEn;
    /**
     * [rw] LTR Mechanism Enable
     *
     * Field size: 1 bit
     */
    uint8_t ltrEn;
    /**
     * [rw] OBFF Enable
     *
     * Field size: 1 bit
     */
    uint8_t obffEn;
} Pcie_DevStatCtrl2Reg;

/**
 * \brief Specification of the Link Capabilities 2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieLnkCap2Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Supported Link Speeds Vector
     *
     * Field size: 7 bits
     */
    uint8_t spLsVec;
    /**
     * [ro] Crosslink Supported
     *
     * Field size: 1 bit
     */
    uint8_t crosslinkSp;
} Pcie_LnkCap2Reg;

/**
 * \brief Specification of the Link Control 2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieLinkCtrl2Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Current De-emphasis level
     *
     * 0 = -6 dB\n
     * 1 = -3.5 dB
     *
     * Field size: 1 bit
     */
    uint8_t deEmph;
    /**
     * [rw] De-emphasis level in polling-compliance state
     *
     * This bit sets the de-emphasis level in Polling Compliance state if the
     * entry occurred due to the Enter Compliance bit being 1.
     *
     * 0 = -6 dB\n
     * 1 = -3.5 dB
     *
     * Field size: 1 bit
     */
    uint8_t pollDeemph;
    /**
     * [rw] Compliance SOS.
     *
     * When this bit is set to 1, the LTSSM is required to send SKP
     * Ordered Sets periodically in between the modified compliance patterns.
     *
     * Field size: 1 bit
     */
    uint8_t cmplSos;
    /**
     * [rw] Enter modified compliance.
     *
     * When this bit is set to 1, the device transmits Modified Compliance
     * Pattern if the LTSSM enters Polling Compliance substate.
     *
     * Field size: 1 bit
     */
    uint8_t entrModCompl;
    /**
     * [rw] Value of non-de-emphasized voltage level at transmitter pins.
     *
     * Field size: 3 bits
     */
    uint8_t txMargin;
    /**
     * [rw] Selectable De-emphasis.
     *
     * When the Link is operating at 5.0 GT/s speed, this bit selects the level
     * of de-emphasis for an upstream component.  When the Link is operating at
     * 2.5 GT/s speed, the setting of this bit has no effect.
     *
     * 0 = -6 dB\n
     * 1 = -3.5 dB
     *
     * Field size: 1 bit
     */
    uint8_t selDeemph;
    /**
     * [rw] Hardware Autonomous Speed Disable.
     *
     * 0 = Enable hardware to change the link speed.\n
     * 1 = Disables hardware from changing the Link speed for device specific
     * reasons other than attempting to correct unreliable Link operation by
     * reducing Link speed.
     *
     * Field size: 1 bit
     */
    uint8_t hwAutoSpeedDis;
    /**
     * [rw] Enter Compliance.
     *
     * Software is permitted to force a Link to enter Compliance mode at the
     * speed indicated in the Target Link Speed field by setting this bit to
     * 1 in both components on a Link and then initiating a hot reset on the Link.
     *
     * Field size: 1 bit
     */
    uint8_t entrCompl;
    /**
     * [rw] Target Link Speed.
     *
     * 1h = 2.5 GT/s Target Link Speed.\n
     * 2h = 5.0 GT/s Target Link Speed.\n
     * Others = Reserved.
     *
     * Field size: 4 bits
     */
    uint8_t tgtSpeed;
    /**
     * [rw] Compliance Pre-set/De-emphasis
     *
     * Field size: 4 bits
     */
    uint8_t complPrstDeemph;
    /**
     * [ro] Equalization Complete, Gen3 Only
     *
     * Field size: 1 bit
     */
    uint8_t eqComplete;
    /**
     * [ro] Equalization Ph1 Success, Gen3 Only
     *
     * Field size: 1 bit
     */
    uint8_t eqPh1;
    /**
     * [ro] Equalization Ph2 Success, Gen3 Only
     *
     * Field size: 1 bit
     */
    uint8_t eqPh2;
    /**
     * [ro] Equalization Ph3 Success, Gen3 Only
     *
     * Field size: 1 bit
     */
    uint8_t eqPh3;
    /**
     * [rw] Link Equilization Request
     *
     * Field size: 1 bit
     */
    uint8_t linkEqReq;
    /**
     * [rw] Downstream Component Presence
     *
     * Field size: 1 bit
     */
    uint8_t downCompPres;
    /**
     * [rw] DRS Message Received
     *
     * Field size: 1 bit
     */
    uint8_t drsMsgRecv;
} Pcie_LinkCtrl2Reg;


/*****************************************************************************
 **********  PCIe EXTENDED CAPABILITIES  REGISTERS ***************************
 ****************************************************************************/
/**
 * \brief Specification of the Extended Capabilities Header register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieExtCapReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Next Capability Pointer
     *
     * Field size: 12 bits
     */
    uint16_t nextCap;
    /**
     * [ro] Extended Capability Version
     *
     * Field size: 4 bits
     */
    uint8_t extCapVer;
    /**
     * [ro] PCIe Extended Capability ID
     *
     * Field size: 16 bits
     */
    uint16_t extCapID;
} Pcie_ExtCapReg;

/**
 * brief Specification of the Uncorrectable Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieUncErrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] TLP Prefix Blocked Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t tlpPrfxBlockedErrSt;
    /**
     * [rw] Uncorrectable Internal Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t intErrSt;
    /**
     * [rw] Unsupported Request Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t urErrSt;
    /**
     * [rw] ECRC Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t ecrcErrSt;
    /**
     * [rw] Malformed TLP Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t mtlpErrSt;
    /**
     * [rw] Receiver Overflow Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t rcvrOfSt;
    /**
     * [rw] Unexpected Completion Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t ucmpSt;
    /**
     * [rw] Completer Abort Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t cmplAbrtSt;
    /**
     * [rw] Completion Timeout Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t cmplTmotSt;
    /**
     * [rw] Flow Control Protocol Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t fcpErrSt;
    /**
     * [rw] Poisoned TLP Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t psndTlpSt;
    /**
     * [ro] Surprise Down Error Status. Not supported (always 0)
     *
     * Field size: 1 bit
     */
    uint8_t srpsDnSt;
    /**
     * [rw] Data Link Protocol Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t dlpErrSt;
} Pcie_UncErrReg;

/**
 * Specification of the Uncorrectable Error Mask register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieUncErrMaskReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] TLP Prefix Blocked Error Mask
     *
     * Field size: 1 bit
     */
    uint8_t tlpPrfxBlockedErrMsk;
    /**
     * [ro] AtomicOp Egress Block Mask
     *
     * Field size: 1 bit
     */
    uint8_t atomicEgressBlockedErrMsk;
    /**
     * [rw] Uncorrectable Internal Error Mask
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t intErrMsk;
    /**
     * [rw] Unsupported Request Error Mask
     *
     * Field size: 1 bit
     */
    uint8_t urErrMsk;
    /**
     * [rw] ECRC Error Mask
     *
     * Field size: 1 bit
     */
    uint8_t ecrcErrMsk;
    /**
     * [rw] Malformed TLP Mask
     *
     * Field size: 1 bit
     */
    uint8_t mtlpErrMsk;
    /**
     * [rw] Receiver Overflow Mask
     *
     * Field size: 1 bit
     */
    uint8_t rcvrOfMsk;
    /**
     * [rw] Unexpected Completion Mask
     *
     * Field size: 1 bit
     */
    uint8_t ucmpMsk;
    /**
     * [rw] Completer Abort Mask
     *
     * Field size: 1 bit
     */
    uint8_t cmplAbrtMsk;
    /**
     * [rw] Completion Timeout Mask
     *
     * Field size: 1 bit
     */
    uint8_t cmplTmotMsk;
    /**
     * [rw] Flow Control Protocol Error Mask
     *
     * Field size: 1 bit
     */
    uint8_t fcpErrMsk;
    /**
     * [rw] Poisoned TLP Mask
     *
     * Field size: 1 bit
     */
    uint8_t psndTlpMsk;
    /**
     * [ro] Surprise Down Error Mask. Not supported (always 0)
     *
     * Field size: 1 bit
     */
    uint8_t srpsDnMsk;
    /**
     * [rw] Data Link Protocol Error Mask
     *
     * Field size: 1 bit
     */
    uint8_t dlpErrMsk;
} Pcie_UncErrMaskReg;

/**
 * \brief Specification of the Uncorrectable Error Severity register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * Set each bit to 0 to indicate the error is non-fatal
 * Set each bit to 1 to indicate the error is fatal.
 */
typedef struct pcieUncErrSvrtyReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] TLP Prefix Blocked Error Mask
     *
     * Field size: 1 bit
     */
    uint8_t tlpPrfxBlockedErrSvrty;
    /**
     * [ro] AtomicOp Egress Block Mask
     *
     * Field size: 1 bit
     */
    uint8_t atomicEgressBlockedErrSvrty;
    /**
     * [rw] Uncorrectable Internal Error Mask
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t intErrSvrty;
    /**
     * [rw] Unsupported Request Error Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t urErrSvrty;
    /**
     * [rw] ECRC Error Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t ecrcErrSvrty;
    /**
     * [rw] Malformed TLP Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t mtlpErrSvrty;
    /**
     * [rw] Receiver Overflow Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t rcvrOfSvrty;
    /**
     * [rw] Unexpected Completion Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t ucmpSvrty;
    /**
     * [rw] Completer Abort Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t cmplAbrtSvrty;
    /**
     * [rw] Completion Timeout Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t cmplTmotSvrty;
    /**
     * [rw] Flow Control Protocol Error Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t fcpErrSvrty;
    /**
     * [rw] Poisoned TLP Severity
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t psndTlpSvrty;
    /**
     * [ro] Surprise Down Error Severity. Not supported (always 0)
     *
     * 0=Non-Fatal; 1=Fatal
     *
     * Field size: 1 bit
     */
    uint8_t srpsDnSvrty;
    /**
     * [rw] Data Link Protocol Error Severity
     *
     * 0 = Non-fatal; 1 = Fatal
     *
     * Field size: 1 bit
     */
    uint8_t dlpErrSvrty;
} Pcie_UncErrSvrtyReg;

/**
 * \brief Specification of the Correctable Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieCorErrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Header Log Overflow Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t hdrLogOverflowErrSt;
    /**
     * [rw] Corrected Internal Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t corrIntErrSt;
    /**
     * [rw] Advisory Non-Fatal Error Status
     *
     * This bit is Set by default to enable compatibility with software
     * that does not comprehend Role-Based Error Reporting.
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t advNFErrSt;
    /**
     * [rw] Replay Timer Timeout Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t rplyTmrSt;
    /**
     * [rw] REPLAY_NUM Rollover Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t rpltRoSt;
    /**
     * [rw] Bad DLLP Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t badDllpSt;
    /**
     * [rw] Bad TLP Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t badTlpSt;
    /**
     * [rw] Receiver Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t rcvrErrSt;
} Pcie_CorErrReg;

/**
 * \brief Specification of the Correctable Error Mask register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieCorErrMaskReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Header Log Overflow Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t hdrLogOverflowErrMsk;
    /**
     * [rw] Corrected Internal Error Status
     *
     * Write 1 to clear
     *
     * Field size: 1 bit
     */
    uint8_t corrIntErrMsk;
    /**
     * [rw] Advisory Non-Fatal Error Mask
     *
     * Field size: 1 bit
     */
    uint8_t advNFErrMsk;
    /**
     * [rw] Replay Timer Timeout Mask
     *
     * Field size: 1 bit
     */
    uint8_t rplyTmrMsk;
    /**
     * [rw] REPLAY_NUM Rollover Mask
     *
     * Field size: 1 bit
     */
    uint8_t rpltRoMsk;
    /**
     * [rw] Bad DLLP Mask
     *
     * Field size: 1 bit
     */
    uint8_t badDllpMsk;
    /**
     * [rw] Bad TLP Mask
     *
     * Field size: 1 bit
     */
    uint8_t badTlpMsk;
    /**
     * [rw] Receiver Error Mask
     *
     * Field size: 1 bit
     */
    uint8_t rcvrErrMsk;
} Pcie_CorErrMaskReg;

/**
 * \brief Specification of the Advanced capabilities and control Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieAccrReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Multiple Header Recording Enable
     *
     * Field size:  1bit
     */
    uint8_t multHdrEn;
    /**
     * [ro] Multiple Header Recording Capable
     *
     * Field size:  1bit
     */
    uint8_t multHdrCap;
    /**
     * [rw] ECRC Check Enable
     *
     * Field size:  1bit
     */
    uint8_t chkEn;
    /**
     * [rw] ECRC Check Capable
     *
     * Field size:  1 bit
     */
    uint8_t chkCap;
    /**
     * [rw] ECRC Generation Enable
     *
     * Field size:  1 bit
     */
    uint8_t genEn;
    /**
     * [rw] ECRC Generation Capability
     *
     * Field size:  1 bit
     */
    uint8_t genCap;
    /**
     * [rw] First Error Pointer
     *
     * The First Error Pointer is a field that identifies the bit position
     * of the first error reported in the @ref pcieUncErrReg_s
     *
     * Field size:  5 bits
     */
    uint8_t erPtr;
} Pcie_AccrReg;

/**
 * \brief Specification of the Header Log registers
 *
 * These registers may be used for both endpoint and root complex modes.
 *
 * There are 4 Header Log registers
 */
typedef struct pcieHdrLogReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] DWORD of header for a detected error
     *
     * Field size: 32 bits
     */
    uint32_t hdrDW;
} Pcie_HdrLogReg;

/**
 * \brief Specification of the Root Error Command register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieRootErrCmdReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Fatal Error Reporting Enable.
     *
     * Field size:  1 bit
     */
    uint8_t ferrRptEn;
    /**
     * [rw] Nonfatal Error Reporting Enable.
     *
     * Field size:  1 bit
     */
    uint8_t nferrRptEn;
    /**
     * [rw] Correctable Error Reporting Enable.
     *
     * Field size:  1 bit
     */
    uint8_t cerrRptEn;
} Pcie_RootErrCmdReg;

/**
 * \brief Specification of the Root Error Status register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieRootErrStReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] AER Interrupt Message Number.
     *
     * Field size:  5 bits
     */
    uint8_t aerIntMsg;
    /**
     * [rw] Fatal Error Messages Received.
     *
     * Write 1 to clear
     *
     * Field size:  1 bit
     */
    uint8_t ferrRcv;
    /**
     * [rw] Non-Fatal Error Messages Received.
     *
     * Write 1 to clear
     *
     * Field size:  1 bit
     */
    uint8_t nfErr;
    /**
     * [rw] First Uncorrectable Fatal Received.
     *
     * Write 1 to clear
     *
     * Field size:  1 bit
     */
    uint8_t uncorFatal;
    /**
     * [rw] Multiple Uncorrectable Error (ERR_FATAL/NONFATAL) Received.
     *
     * Write 1 to clear
     *
     * Field size:  1 bit
     */
    uint8_t multFnf;
    /**
     * [rw] Uncorrectable Error (ERR_FATAL/NONFATAL) Received.
     *
     * Write 1 to clear
     *
     * Field size:  1 bit
     */
    uint8_t errFnf;
    /**
     * [rw] Multiple Correctable Error (ERR_COR) Received.
     *
     * Write 1 to clear
     *
     * Field size:  1 bit
     */
    uint8_t multCor;
    /**
     * [rw] Correctable Error (ERR_COR) Received.
     *
     * Write 1 to clear
     *
     * Field size:  1 bit
     */
    uint8_t corrErr;
} Pcie_RootErrStReg;

/**
 * \brief Specification of the Error Source Identification register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieErrSrcIDReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Fatal or Non-Fatal error source identification
     *
     *
     * Field size:  16 bits
     */
    uint16_t fnfSrcID;
    /**
     * [ro] Correctable error source identification
     *
     * Field size:  16 bits
     */
    uint16_t corrSrcID;
} Pcie_ErrSrcIDReg;


/*****************************************************************************
 **********  PCIe LOCAL/REMOTE PORT LOGIC REGISTERS **************************
 ****************************************************************************/
/**
 * Specification of the Ack Latency Time and Replay Timer register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pciePlAckTimerReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Replay Time Limit.
     *
     * Field size:  16 bits
     */
    uint16_t rplyLmt;
    /**
     * [rw] Round Trip Latency Time Limit.
     *
     * Field size:  16 bits
     */
    uint16_t rndTrpLmt;
} Pcie_PlAckTimerReg;

/**
 * \brief Specification of the Other Message register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pciePlOMsgReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Other Message Register
     *
     * It can be used to send a specific PCI Express message in which
     * case this register is programmed with the payload and bit
     * @ref pcieLnkCtrlReg_s::msgReq set to transmit the message.
     *
     * Field size:  32 bits
     */
    uint32_t oMsg;
} Pcie_PlOMsgReg;

/**
 * Specification of the Port Force Link register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pciePlForceLinkReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Low Power Entrance Count
     *
     * Field size:  8 bits
     */
    uint8_t lpeCnt;
    /**
     * [rw] Do Deskew for SRIS
     *
     * Use the transitions from TS2 to Logical Idle Symbol, SKP OS to
     * Logical Idle Symbol, and FTS Sequence to SKP OS to do deskew
     * for SRIS instead of using received SKP OS if
     * doDeskewForSris is set to 1
     * Note: This register field is sticky
     *
     * Field size:  1 bits
     */
    uint8_t doDeskewForSris;
    /**
     * [rw] Link State.
     *
     * The link state that the PCIe will be forced to when
     * @ref forceLink field is set.  See @ref pcieLtssmState_e
     * for LTSSM states encoded values.
     *
     * Field size:  6 bits
     */
    uint8_t lnkState;
    /**
     * [rw] Force Link.
     *
     * Forces the link to the state specified by the @ref lnkState field.
     * The Force Link pulse will trigger link re-negotiation.  Self clears.
     *
     * Field size:  1 bit
     */
    uint8_t forceLink;
    /**
     * [rw] LTSSM state forced by setting @ref forceLink
     *
     * Field size:  4 bits
     */
    uint8_t forcedLtssmState;
    /**
     * [rw] Link Number. Not used for EP.
     *
     * Field size:  8 bits
     */
    uint8_t linkNum;
} Pcie_PlForceLinkReg;

/**
 * \brief Specification of the Ack Frequency register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieAckFreqReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Allow ASPM L1 without partner going to L0s.
     *
     * Set to allow entering ASPM L1 even when link partner did not
     * go to L0s. When cleared, the ASPM L1 state is entered only after idle
     * period during which both RX and TX are in L0s.
     *
     * Field size:  1 bit
     */
    uint8_t aspmL1;
    /**
     * [rw] L1 entrance latency.
     *
     * The latency is set to 2^@ref l1EntryLatency microseconds with
     * the max being 64 microseconds.
     * <TABLE>
     * <TR><TH>@ref l1EntryLatency</TH><TH>latency in �s</TH></TR>
     * <TR><TD>0</TD>                  <TD>1�s</TD></TR>
     * <TR><TD>1</TD>                  <TD>2�s</TD></TR>
     * <TR><TD>2</TD>                  <TD>4�s</TD></TR>
     * <TR><TD>3</TD>                  <TD>8�s</TD></TR>
     * <TR><TD>4</TD>                  <TD>16�s</TD></TR>
     * <TR><TD>5</TD>                  <TD>32�s</TD></TR>
     * <TR><TD>6</TD>                  <TD>64�s</TD></TR>
     * <TR><TD>7</TD>                  <TD>64�s</TD></TR>
     * </TABLE>
     *
     * Field size:  3 bits
     */
    uint8_t l1EntryLatency;
    /**
     * [rw] L0s entrance latency.
     *
     * The latency is set to @ref l0sEntryLatency + 1 microseconds.
     * Maximum is 7 microseconds.
     *
     * <TABLE>
     * <TR><TH>@ref l0sEntryLatency</TH><TH>latency in �s</TH></TR>
     * <TR><TD>0</TD>                   <TD>1�s</TD></TR>
     * <TR><TD>1</TD>                   <TD>2�s</TD></TR>
     * <TR><TD>2</TD>                   <TD>3�s</TD></TR>
     * <TR><TD>3</TD>                   <TD>4�s</TD></TR>
     * <TR><TD>4</TD>                   <TD>5�s</TD></TR>
     * <TR><TD>5</TD>                   <TD>6�s</TD></TR>
     * <TR><TD>6</TD>                   <TD>7�s</TD></TR>
     * <TR><TD>7</TD>                   <TD>7�s</TD></TR>
     * </TABLE>
     *
     * Field size:  3 bits
     */
    uint8_t l0sEntryLatency;
    /**
     * [rw] Number of fast training sequences for common clock
     *
     * Number of fast training sequences when common clock is used
     * and when transitioning from L0s to L0.
     *
     * Field size:  8 bits
     */
    uint8_t commNFts;
    /**
     * [rw] Number of fast training sequences to be transmitted
     *
     * Number of fast training sequences to be transmitted
     * when transitioning from L0s to L0. Value of 0 is not supported.
     *
     * Field size:  8 bits
     */
    uint8_t nFts;
    /**
     * [rw] Ack Frequency.
     *
     * Default is to wait until 255 Ack DLLPs are pending before it is sent.
     *
     * Field size:  8 bits
     */
    uint8_t ackFreq;
} Pcie_AckFreqReg;

/**
 * \brief Specification of the Port Link Control Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieLnkCtrlReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Crosslink Active
     *
     * Field size: 1 bit
     */
    uint8_t crosslinkAct;
    /**
     * [rw] Crosslink Enable
     *
     * Field size: 1 bit
     */
    uint8_t crosslinkEn;
    /**
     * [rw] Link Mode
     *
     * <TABLE>
     * <TR><TH>@ref lnkMode</TH><TH># of lanes</TH></TR>
     * <TR><TD>0x1</TD>         <TD>1</TD></TR>
     * <TR><TD>0x3</TD>         <TD>2</TD></TR>
     * <TR><TD>0x7</TD>         <TD>4</TD></TR>
     * <TR><TD>0xf</TD>         <TD>8</TD></TR>
     * <TR><TD>0x1f</TD>        <TD>16</TD></TR>
     * <TR><TD>0x3f</TD>        <TD>32</TD></TR>
     * <TR><TD>others</TD>      <TD>reserved</TD></TR>
     * </TABLE>
     *
     * Field size: 6 bits
     */
    uint8_t lnkMode;
    /**
     * [rw] Link Rate
     *
     * For 2.5 GT/s it is 0x1. This register does not affect any functionality.
     *
     * Field size: 4 bits
     */
    uint8_t lnkRate;
    /**
     * [rw] Fast link mode
     *
     * Set all internal timers to fast mode for simulation purposes.
     *
     * Field size: 1 bit
     */
    uint8_t fLnkMode;
    /**
     * [rw] DLL link enable
     *
     * DLL Link Enable. Enable link initialization.
     *
     * Field size: 1 bit
     */
    uint8_t dllEn;
    /**
     * [rw] Reset Assert
     *
     * Triggers a recovery and forces the LTSSM to the Hot Reset state.
     * Downstream ports (RC ports) only.
     *
     * Field size: 1 bit
     */
    uint8_t rstAsrt;
    /**
     * [rw] Loopback Enable
     *
     * Field size: 1 bit
     */
    uint8_t lpbkEn;
    /**
     * [rw] Scramble Disable
     *
     * Field size: 1 bit
     */
    uint8_t scrmDis;
    /**
     * [rw] Other Message Request
     *
     * Set to transmit the message contained in @ref pciePlOMsgReg_s
     *
     * Field size: 1 bit
     */
    uint8_t msgReq;
} Pcie_LnkCtrlReg;

/**
 * \brief Specification of the Lane Skew register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieLaneSkewReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Set to Disable Lane to Lane Deskew.
     *
     * Field size: 1 bit
     */
    uint8_t l2Deskew;
    /**
     * [rw] Set lanes allowed for loopback
     *
     * Implementation-specific Number of Lanes Set the implementation specific
     * number of lanes
     * Allowed values are:
     * 0000b: 1 lane
     * 0001b: 2 lanes
     * 0011b: 4 lanes
     * 0111b: 8 lanes
     * 1111b: 16 lanes
     * The number of lanes to be used when in Loopback
     * Master The number of lanes programmed must be equal to or less
     * than the valid number of lanes set in LINK_CAPABLE field You must
     * configure this field before initiating Loopback by writing in the
     * LOOPBACK_ENABLE field The controller will transition from
     * LoopbackEntry to LoopbackActive after receiving two consecutive
     * TS1 Ordered Sets with the Loopback bit asserted on the
     * implementation specific number of lanes configured in this field Note:
     * This register field is sticky
     *
     * Field size: 4 bits
     */
    uint8_t implementNumLanes;
    /**
     * [rw] Set to disable Ack and Nak DLLP transmission.
     *
     * Field size: 1 bit
     */
    uint8_t ackDisable;
    /**
     * [rw] Set to disable transmission of Flow Control DLLPs.
     *
     * Field size: 1 bit
     */
    uint8_t fcDisable;
    /**
     * [rw] Insert Lane Skew for Transmit.
     *
     * The value is in units of one symbol time. Thus a value 0x02 will
     * force a skew of two symbol times for that lane. Max allowed is
     * 5 symbol times. This 24 bit field is used for programming skew
     * for eight lanes with three bits per lane.
     *
     * Field size: 24 bits
     */
    uint32_t laneSkew;
} Pcie_LaneSkewReg;

/**
 * \brief Specification of the Symbol Number register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieSymNumReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Configuration requests targeted at function numbers above this
     * value will result in UR response.
     *
     * Field size: 8 bits
     */
    uint8_t maxFunc;
    /**
     * [rw] Timer Modifier for Flow Control Watchdog Timer.
     *
     * Increases the timer value for Flow Control watchdog timer in
     * increments of 16 clock cycles.
     *
     * Field size: 5 bits
     */
    uint8_t fcWatchTimer;
    /**
     * [rw] Timer Modifier for Ack/Nak Latency Timer.
     *
     * Increases the timer value for the Ack/Nak latency timer in
     * increments of 64 clock periods.
     *
     * Field size: 5 bits
     */
    uint8_t ackLatencyTimer;
    /**
     * [rw] Timer for replaying TLPs in increments of 64 clock cycles.
     *
     * Increases the timer value for Ack/Nak latency timer in increments of
     * 64 clock cycles.
     *
     * Field size: 5 bits
     */
    uint8_t replayTimer;
    /**
     * [rw] Number of SKP Symbols.
     *
     * Field size: 3 bits
     */
    uint8_t skpCount;
    /**
     * [rw] Number of TS2 Symbols.
     *
     * This field does not affect any functionality.
     *
     * Field size: 4 bits
     */
    uint8_t numTs2Symbols;
    /**
     * [rw] Number of TS Symbols.
     *
     * Set the number of TS identifier symbols that are sent in TS1 and TS2
     * ordered sets.
     *
     * Field size: 4 bits
     */
    uint8_t tsCount;
    /**
     * [rw] Number of TS Symbols.
     *
     * Set the number of TS identifier symbols that are sent in TS1 and TS2
     * ordered sets.
     *
     * Fast Link Timer Scaling Factor Sets the scaling factor of LTSSM
     * timer when FAST_LINK_MODE field in PCIE_EP_PORT_LINK_CTRL_OFF is set to '1'
     * 0: Scaling Factor is 1024 [1ms is 1us]
     * 1: Scaling Factor is 256 [1ms is 4us]
     * 2: Scaling Factor is 64 [1ms is 16us]
     * 3: Scaling Factor is 16 [1ms is 64us]
     * Not used for M-PCIe Note: This register field is sticky
     *
     * Field size: 3 bits
     */
    uint8_t fastLinkScalingFactor;
} Pcie_SymNumReg;

/**
 * \brief Specification of the Symbol Timer and Filter Mask register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieSymTimerFltMaskReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] 1 = Allow CFG transaction being received on RC.
     *
     * Field size: 1 bit
     */
    uint8_t f1CfgDrop;
    /**
     * [rw] 1 = Allow IO transaction being received on RC.
     *
     * Field size: 1 bit
     */
    uint8_t f1IoDrop;
    /**
     * [rw] 1 = Allow MSG transaction being received on RC.
     *
     * Field size: 1 bit
     */
    uint8_t f1MsgDrop;
    /**
     * [rw] 1 = Allow completion TLPs with ECRC errors to be passed up.
     *
     * Field size: 1 bit
     */
    uint8_t f1CplEcrcDrop;
    /**
     * [rw] 1 = Allow TLPs with ECRC errors to be passed up.
     *
     * Field size: 1 bit
     */
    uint8_t f1EcrcDrop;
    /**
     * [rw] 1 = Mask length match for received completion TLPs.
     *
     * Field size: 1 bit
     */
    uint8_t f1CplLenTest;
    /**
     * [rw] 1 = Mask attribute match on received completion TLPs.
     *
     * Field size: 1 bit
     */
    uint8_t f1CplAttrTest;
    /**
     * [rw] 1 = Mask traffic class match on received completion TLPs.
     *
     * Field size: 1 bit
     */
    uint8_t f1CplTcTest;
    /**
     * [rw] 1 = Mask function match for received completion TLPs.
     *
     * Field size: 1 bit
     */
    uint8_t f1CplFuncTest;
    /**
     * [rw] 1 = Mask request ID match for received completion TLPs.
     *
     * Field size: 1 bit
     */
    uint8_t f1CplReqIDTest;
    /**
     * [rw] 1 = Mask tag error rules for received completion TLPs.
     *
     * Field size: 1 bit
     */
    uint8_t f1CplTagErrTest;
    /**
     * [rw] 1 = Treat locked read TLPs as supported for EP, UR for RC.
     *
     * Field size: 1 bit
     */
    uint8_t f1LockedRdAsUr;
    /**
     * [rw] 1 = Treat type 1 CFG TLPs as supported for EP and UR for RC.
     *
     * Field size: 1 bit
     */
    uint8_t f1Cfg1ReAsUs;
    /**
     * [rw] 1 = Treat out-of-BAR TLPs as supported requests.
     *
     * Field size: 1 bit
     */
    uint8_t f1UrOutOfBar;
    /**
     * [rw] 1 = Treat poisoned TLPs as supported requests.
     *
     * Field size: 1 bit
     */
    uint8_t f1UrPoison;
    /**
     * [rw] 1 = Treat function mismatched TLPs as supported requests.
     *
     * Field size: 1 bit
     */
    uint8_t f1UrFunMismatch;
    /**
     * [rw] 1 = Disable Flow Control watchdog timer.
     *
     * Field size: 1 bit
     */
    uint8_t fcWdogDisable;
    /**
     * [rw] Wait time between SKP ordered sets
     *
     * Number of symbol times to wait between transmitting SKP
     * ordered sets. For example, for a setting of 1536 decimal,
     * the wait will be for 1537 symbol times.
     *
     * Field size: 11 bits
     */
    uint16_t skpValue;
} Pcie_SymTimerFltMaskReg;

/**
 * \brief Specification of the Filter Mask 2 register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieFltMask2Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] 1 = Drop PRS messages silently
     *
     * Field size: 1 bit
     */
    uint8_t dropPRS;
    /**
     * [rw] 1 = Enable unmask TD bit if CX_STRIP_ECRC_ENABLE
     *
     * Field size: 1 bit
     */
    uint8_t unmaskTD;
    /**
     * [rw] 1 = Enable unmask CX_FLT_MASK_UR_POIS with TRGT0 destination
     *
     * Field size: 1 bit
     */
    uint8_t unmaskUrPOIS;
    /**
     * [rw] 1 = Drop LN Messages silently
     *
     * Field size: 1 bit
     */
    uint8_t dropLN;
    /**
     * [rw] 1 = Enable the filter to handle flush request.
     *
     * Field size: 1 bit
     */
    uint8_t flushReq;
    /**
     * [rw] 1 = Disable DLLP abort for unexpected CPL.
     *
     * Field size: 1 bit
     */
    uint8_t dllpAbort;
    /**
     * [rw] 1 = Disable dropping of Vendor MSG Type 1.
     *
     * Field size: 1 bit
     */
    uint8_t vmsg1Drop;
    /**
     * [rw] 1 = Disable dropping of Vendor MSG Type 0 with UR error reporting.
     *
     * Field size: 1 bit
     */
    uint8_t vmsg0Drop;
} Pcie_FltMask2Reg;

/**
 * \brief Specification of the Debug0 Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieDebug0Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Link control bits advertised by link partner.
     *
     * Field size: 4 bits
     */
    uint8_t tsLnkCtrl;
    /**
     * [ro] Currently receiving k237 (PAD) in place of lane number.
     *
     * Field size: 1 bit
     */
    uint8_t tsLaneK237;
    /**
     * [ro] Currently receiving k237 (PAD) in place of link number.
     *
     * Field size: 1 bit
     */
    uint8_t tsLinkK237;
    /**
     * [ro] Receiver is receiving logical idle
     *
     * Field size: 1 bit
     */
    uint8_t rcvdIdle0;
    /**
     * [ro] 2nd symbol is also idle
     *
     * Field size: 1 bit
     */
    uint8_t rcvdIdle1;
    /**
     * [ro] Pipe TX data
     *
     * Field size: 16 bits
     */
    uint16_t pipeTxData;
    /**
     * [ro] Pipe transmit K indication
     *
     * Field size: 2 bits
     */
    uint8_t pipeTxDataK;
    /**
     * [ro] A skip ordered set has been transmitted.
     *
     * Field size: 1 bit
     */
    uint8_t skipTx;
    /**
     * [ro] LTSSM current state @ref pcieLtssmState_e
     *
     * Field size: 5 bits
     */
    uint8_t ltssmState;
} Pcie_Debug0Reg;

/**
 * \brief Specification of the Debug 1 Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieDebug1Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Scrambling disabled for the link.
     *
     * Field size: 1 bit
     */
    uint8_t scramblerDisable;
    /**
     * [rw] LTSSM in DISABLE state. Link inoperable.
     *
     * Field size: 1 bit
     */
    uint8_t linkDisable;
    /**
     * [rw] LTSSM performing link training.
     *
     * Field size: 1 bit
     */
    uint8_t linkInTraining;
    /**
     * [rw] LTSSM testing for polarity reversal.
     *
     * Field size: 1 bit
     */
    uint8_t rcvrRevrsPolEn;
    /**
     * [rw] LTSSM-negotiated link reset.
     *
     * Field size: 1 bit
     */
    uint8_t trainingRstN;
    /**
     * [rw] PIPE receiver detect/loopback request.
     *
     * Field size: 1 bit
     */
    uint8_t pipeTxdetectrxLb;
    /**
     * [rw] PIPE transmit electrical idle request.
     *
     * Field size: 1 bit
     */
    uint8_t pipeTxelecidle;
    /**
     * [rw] PIPE transmit compliance request.
     *
     * Field size: 1 bit
     */
    uint8_t pipeTxcompliance;
    /**
     * [rw] Application request to initiate training reset.
     *
     * Field size: 1 bit
     */
    uint8_t appInitRst;
    /**
     * [rw] Link number advertised/confirmed by link partner.
     *
     * Field size: 8 bits
     */
    uint8_t rmlhTsLinkNum;
    /**
     * [rw] LTSSM reports PHY link up.
     *
     * Field size: 1 bit
     */
    uint8_t xmlhLinkUp;
    /**
     * [rw] Receiver reports skip reception.
     *
     * Field size: 1 bit
     */
    uint8_t rmlhInskipRcv;
    /**
     * [rw] TS1 training sequence received (pulse).
     *
     * Field size: 1 bit
     */
    uint8_t rmlhTs1Rcvd;
    /**
     * [rw] TS2 training sequence received (pulse).
     *
     * Field size: 1 bit
     */
    uint8_t rmlhTs2Rcvd;
    /**
     * [rw] Receiver detected lane reversal.
     *
     * Field size: 1 bit
     */
    uint8_t rmlhRcvdLaneRev;
} Pcie_Debug1Reg;

/**
 * \brief Specification of the Gen2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieGen2Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     *  [rw] Electrical Idle Inference Mode at Gen1 Rate
     *
     *  0: Use RxElecIdle signal to infer Electrical Idle -
     *  1: Use RxValid signal to infer Electrical Idle
     *
     * Note: This register field is sticky
     *
     * Field size: 1 bit
     */
    uint8_t gen1EiInference;
    /**
     *  [rw] Set de-emphasis level for upstream (EP) ports
     *
     * Field size: 1 bit
     */
    uint8_t deemph;
    /**
     *  [rw] Configure TX compliance receive bit.
     *
     * Field size: 1 bit
     */
    uint8_t txCmpl;
    /**
     *  [rw] Configure PHY TX Swing
     *
     * 0 = Low Swing\n
     * 1 = Full Swing
     *
     * Field size: 1 bit
     */
    uint8_t txSwing;
    /**
     *  [rw] direct speed change
     *
     * 0 = Indicates to the LTSSM not to initiate a speed change to Gen2
     * after the link is initialized at Gen1 speed.\n
     * 1 = Indicates to the LTSSM to initiate a speed change to Gen2
     * after the link is initialized at Gen1 speed.
     *
     * Field size: 1 bit
     */
    uint8_t dirSpd;
    /**
     *  [rw] Enable Auto flipping of the lanes
     *
     * Field size: 1 bits
     */
    uint8_t autoFlipEn;
    /**
     *  [rw] Predetermined Lane for Auto Flip
     *
     * Flip This field defines which physical
     * lane is connected to logical Lane0 by the flip operation
     * performed in Detect
     * Allowed values are: - 3'b
     * 000: Connect logical Lane0 to physical lane 0 or CX_NL-1 or CX_NL/
     *      2-1 or CX_NL/
     *      4-1 or CX_NL/
     *      8-1, depending on which lane is detected
     * 001: Connect logical Lane0 to physical lane 1
     * 010: Connect logical Lane0 to physical lane 3
     * 011: Connect logical Lane0 to physical lane 7
     * 100: Connect logical Lane0 to physical lane 15
     *
     * Field size: 3 bits
     */
    uint8_t preDetLane;
    /**
     *  [rw]  Lane enable. 1h=x1, 2h=x2. Other values reserved.
     *
     * Field size: 9 bits (5 bits rev 2)
     */
    uint16_t lnEn;
    /**
     *  [rw] number of fast training sequences
     *
     * Field size: 8 bit
     */
    uint8_t numFts;
} Pcie_Gen2Reg;

/**
 * \brief Specification of the AXI Multiple Outbound Decomposed NP SubRequests
 * Control Register (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfObnpSubreqCtrlReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Enable AXI Multiple Outbound Decomposed NP Sub-Requests
     *
     * Field size: 1 bit
     */
    uint8_t enObnpSubreq;
} Pcie_PlconfObnpSubreqCtrlReg;

/**
 * \brief Specification of the Transmit Posted FC Credit Status Register (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 */
typedef struct  pciePlconfTrPStsRReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Transmit Posted Data FC Credits
     *
     * Field size: 12 bits
     */
    uint16_t pdCrdt;
    /**
     * [ro] Transmit Posted Header FC Credits
     *
     * Field size: 8 bits
     */
    uint8_t phCrdt;
} Pcie_PlconfTrPStsRReg;

/**
 * \brief Specification of the Transmit Non-Posted FC Credit Status Register
 * (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 */
typedef struct  pciePlconfTrNpStsRReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Transmit Non-Posted Data FC Credits
     *
     * Field size: 12 bits
     */
    uint16_t npdCrdt;
    /**
     * [ro] Transmit Non-Posted Header FC Credits
     *
     * Field size: 8 bits
     */
    uint8_t nphCrdt;
} Pcie_PlconfTrNpStsRReg;

/**
 * \brief Specification of the Transmit Completion FC Credit Status Register
 * (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 */
typedef struct  pciePlconfTrCStsRReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Transmit Completion Data FC Credits
     *
     * Field size: 12 bits
     */
    uint16_t cpldCrdt;
    /**
     * [ro] Transmit Completion Header FC Credits
     *
     * Field size: 8 bits
     */
    uint8_t cplhCrdt;
} Pcie_PlconfTrCStsRReg;

/**
 * \brief Specification of the Queue Status Register (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfQStsRReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Received TLP FC Credits Not Returned
     *
     * Field size: 1 bit
     */
    uint8_t crdtNotRtrn;
    /**
     *  [ro] Transmit Retry Buffer Not Empty
     *
     * Field size: 1 bit
     */
    uint8_t rtybNotEmpty;
    /**
     *  [ro] Received Queue Not Empty
     *
     * Field size: 1 bit
     */
    uint8_t rcvqNotEmpty;
    /**
     *  [r/w1c] Receive Credit Queue Overflow
     *
     * Field size: 1 bit
     */
    uint8_t rxQueueOverflow;
    /**
     *  [r/w1c] Receive Serialization Queue Not Empty
     *
     * Field size: 1 bit
     */
    uint8_t rxSerQNEmpty;
    /**
     *  [r/w1c] Receive Serialization Queue Write Error
     *
     * Field size: 1 bit
     */
    uint8_t rxSerQWErr;
    /**
     *  [r/w1c] Receive Serialization Read Error
     *
     * Field size: 1 bit
     */
    uint8_t rxSerRErr;
    /**
     *  [rw] FC Latency Timer Override Value
     *
     * Field size: 13 bits
     */
    uint16_t fcLatencyOvr;
    /**
     *  [rw] FC Latency Timer Override Enable
     *
     * Field size: 1 bit
     */
    uint8_t fcLatencyOvrEn;
} Pcie_PlconfQStsRReg;

/**
 * \brief Specification of the VC Transmit Arbitration Register 1 (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 */
typedef struct  pciePlconfVcTrAR1Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] WRR Weight for VC0
     *
     * Field size: 8 bits
     */
    uint8_t wrrVc0;
    /**
     * [ro] WRR Weight for VC1
     *
     * Field size: 8 bits
     */
    uint8_t wrrVc1;
    /**
     * [ro] WRR Weight for VC2
     *
     * Field size: 8 bits
     */
    uint8_t wrrVc2;
    /**
     * [ro] WRR Weight for VC3
     *
     * Field size: 8 bits
     */
    uint8_t wrrVc3;
} Pcie_PlconfVcTrAR1Reg;

/**
 * \brief Specification of the VC Transmit Arbitration Register 2 (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 */
typedef struct  pciePlconfVcTrAR2Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] WRR Weight for VC4
     *
     * Field size: 8 bits
     */
    uint8_t wrrVc4;
    /**
     * [ro] WRR Weight for VC5
     *
     * Field size: 8 bits
     */
    uint8_t wrrVc5;
    /**
     * [ro] WRR Weight for VC6
     *
     * Field size: 8 bits
     */
    uint8_t wrrVc6;
    /**
     * [ro] WRR Weight for VC7
     *
     * Field size: 8 bits
     */
    uint8_t wrrVc7;
} Pcie_PlconfVcTrAR2Reg;

/**
 * \brief Specification of the VC0 Posted Receive Queue Control (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfVcPrQCReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     *  [ro] VC0 Posted Data Credits
     *
     * Field size: 12 bits
     */
    uint16_t pDcrd;
    /**
     *  [ro] VC0 Posted Header Credits
     *
     * Field size: 8 bits
     */
    uint8_t pHcrd;
    /**
     *  [rw] VC0 Scale Posted Data Credits
     *
     * Note: This register field is sticky
     *
     * Field size: 2 bits
     */
    uint8_t pDataScale;
    /**
     *  [rw] VC0 Scale Posted Header Credits
     *
     * Note: This register field is sticky
     *
     * Field size: 2 bits
     */
    uint8_t pHdrScale;
    /**
     *  [rw] VC0 TLP Type Ordering Rules
     *
     * <table>
     * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
     * <tr><td>0x0</td><td>STRICT</td><td>Posted, then Completion, then
     *                                    Non-Posted</td></tr>
     * <tr><td>0x1</td><td>STANDARD</td><td>As per PCIe standard</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t orderingRules;
    /**
     *  [rw] VC0 TLP Type Ordering Rules
     *
     * <table>
     * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
     * <tr><td>0x0</td><td>ROUND_ROBIN</td></tr>
     * <tr><td>0x1</td><td>STRICT</td><td>Ordering by VC</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t strictVcPriority;
    /**
     *  [rw] VC0 Poster TLP Queue Mode
     *
     * <table>
     * <tr><th>Action/Value</th><th>Mode</th></tr>
     * <tr><td>Read 0x1</td><td>STORE_AND_FORWARD</td>/tr>
     * <tr><td>Read 0x2</td><td>CUT_THROUGH</td>/tr>
     * <tr><td>Read 0x4</td><td>BYPASS</td>/tr>
     * <tr><td>Others</td><td>Reserved</td>/tr>
     * </table>
     *
     * Field size: 3 bits
     */
    uint8_t pQmode;
} Pcie_PlconfVcPrQCReg;

/**
 * \brief backwards compatibility alias for Vc0 */
typedef Pcie_PlconfVcPrQCReg Pcie_PlconfVc0PrQCReg;

/**
 * \brief Specification of the VC0 Non-Posted Receive Queue Control (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfVcNprQCReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] VC0 Non-Posted Data Credits
     *
     * Field size: 12 bits
     */
    uint16_t npDcrd;
    /**
     * [ro] VC0 Non-Posted Header Credits
     *
     * Field size: 8 bits
     */
    uint8_t npHcrd;
    /**
     * [rw] VC0 Scale Non-Posted Data Credits
     *
     * Note: This register field is sticky
     *
     * Field size: 2 bits
     */
    uint8_t npDataScale;
    /**
     * [rw] VC0 Scale Non-Posted Header Credits
     *
     * Note: This register field is sticky
     *
     * Field size: 2 bits
     */
    uint8_t npHdrScale;
    /**
     * [rw] VC0 Non-Poster TLP Queue Mode
     *
     * <table>
     * <tr><th>Action/Value</th><th>Mode</th></tr>
     * <tr><td>Read 0x1</td><td>STORE_AND_FORWARD</td>/tr>
     * <tr><td>Read 0x2</td><td>CUT_THROUGH</td>/tr>
     * <tr><td>Read 0x4</td><td>BYPASS</td>/tr>
     * <tr><td>Others</td><td>Reserved</td>/tr>
     * </table>
     *
     * Field size: 3 bits
     */
    uint8_t npQmode;
} Pcie_PlconfVcNprQCReg;

/**
 * \brief backwards compatibility alias for Vc0 */
typedef Pcie_PlconfVcNprQCReg Pcie_PlconfVc0NprQCReg;

/**
 * \brief Specification of the VC0 Completion Receive Queue Control (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfVcCrQCReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] VC0 Completion Data Credits
     *
     * Field size: 12 bits
     */
    uint16_t cplDcrd;
    /**
     * [ro] VC0 Completion Header Credits
     *
     * Field size: 8 bits
     */
    uint8_t cplHcrd;
    /**
     * [rw] VC0 Scale CPL Data Credits
     *
     * Note: This register field is sticky
     *
     * Field size: 2 bits
     */
    uint8_t cplDataScale;
    /**
     * [rw] VC0 Scale CPL Header Credits
     *
     * Note: This register field is sticky
     *
     * Field size: 2 bits
     */
    uint8_t cplHdrScale;
    /**
     * [rw] VC0 Completion TLP Queue Mode
     *
     * <table>
     * <tr><th>Action/Value</th><th>Mode</th></tr>
     * <tr><td>Read 0x1</td><td>STORE_AND_FORWARD</td>/tr>
     * <tr><td>Read 0x2</td><td>CUT_THROUGH</td>/tr>
     * <tr><td>Read 0x4</td><td>BYPASS</td>/tr>
     * <tr><td>Others</td><td>Reserved</td>/tr>
     * </table>
     *
     * Field size: 3 bits
     */
    uint8_t cplQmode;
} Pcie_PlconfVcCrQCReg;

/**
 * \brief backwards compatibility alias for Vc0 */
typedef Pcie_PlconfVcCrQCReg Pcie_PlconfVc0CrQCReg;

/**
 * \brief Specification of the PHY Status Register (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 */
typedef struct  pciePlconfPhyStsRReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] PHY Status
     *
     * Field size: 32 bits
     */
    uint32_t phySts;
} Pcie_PlconfPhyStsRReg;

/**
 * \brief Specification of the PHY Control Register (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfPhyCtrlRReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] PHY Control
     *
     * Field size: 32 bits
     */
    uint32_t phyCtrl;
} Pcie_PlconfPhyCtrlRReg;

/**
 * \brief Specification of the MSI Controller Address Register
 * (RC-mode MSI receiver)
 *
 * This register may be used only for root complex modes.
 */
typedef struct  pciePlconfMsiCtrlAddressReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSI CTRL ADDRESS
     *
     * Field size: 32 bits
     */
    uint32_t msiCtrlAddress;
} Pcie_PlconfMsiCtrlAddressReg;

/**
 * \brief Specification of the MSI Controller Upper Address Register
 * (RC-mode MSI receiver)
 *
 * This register may be used only for root complex modes.
 */
typedef struct  pciePlconfMsiCtrlUpperAddressReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSI CTRL UPPER ADDRESS
     *
     * Field size: 32 bits
     */
    uint32_t msiCtrlUpperAddress;
} Pcie_PlconfMsiCtrlUpperAddressReg;

/**
 * \brief Specification of the MSI Controller Interrupt # N(1) Enable Register
 *
 * (RC-mode MSI receiver) with N = MSI data [7:5] and ENABLE[i] = enable MSI vector # i,
 * with i = MSI data [4:0]
 *
 * This register may be used only for root complex modes.
 */
typedef struct  pciePlconfMsiCtrlIntEnableReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Status of an enabled bit (vectors) is set upon incoming MSI
     *
     * Field size: 32 bits
     */
    uint32_t msiCtrlIntEnable;
} Pcie_PlconfMsiCtrlIntEnableReg;

/**
 * \brief Specification of the MSI Controller Interrupt # N(1) Mask Register
 *
 * (RC-mode MSI receiver) with N = MSI data [7:5] and MASK[i] = mask of MSI
 * vector # i, with i = MSI data [4:0]
 *
 * This register may be used only for root complex modes.
 */
typedef struct  pciePlconfMsiCtrlIntMaskReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Status of a masked bit (vector) triggers no IRQ to MPU when set
     *
     * Field size: 32 bits
     */
    uint32_t msiCtrlIntMask;
} Pcie_PlconfMsiCtrlIntMaskReg;

/**
 * \brief Specification of the MSI Controller Interrupt # N(1) Status Register
 *
 * (RC-mode MSI receiver) with N = MSI data [7:5] and STATUS[i] = status
 * of MSI vector # i, with i = MSI data [4:0]
 *
 * This register may be used only for root complex modes.
 */
typedef struct  pciePlconfMsiCtrlIntStatusReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Status of an enabled bit (vectors) is set upon incoming MSI
     *
     * Field size: 32 bits
     */
    uint32_t msiCtrlIntStatus;
} Pcie_PlconfMsiCtrlIntStatusReg;

/**
 * \brief Specification of the MSI Controller General Purpose IO Register
 * (RC-mode MSI receiver)
 *
 * This register may be used only for root complex modes.
 */
typedef struct  pciePlconfMsiCtrlGpioReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSI CTRL GPIO
     *
     * Field size: 32 bits
     */
    uint32_t msiCtrlGpio;
} Pcie_PlconfMsiCtrlGpioReg;

/**
 * \brief Specification of the PIPE loopback control register (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfPipeLoopbackReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] PIPE Loopback Enable
     *
     * Field size: 1 bit
     */
    uint8_t loopbackEn;
} Pcie_PlconfPipeLoopbackReg;

/**
 * \brief Specification of the DIF Read-Only register Write Enable (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfDbiRoWrEnReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Control the writability over DIF of certain configuration
     *
     * Control the writability over DIF of certain configuration fields that
     * are RO over the PCIe wire
     *
     * <table>
     * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
     * <tr><td>0x0</td><td>WRDIS</td><td>
     * RO fields are also RO over DIF; Use for RC mode (Type-1) config
     * to mimic PCIe wire access when using DIF</td></tr>
     * <tr><td>0x1</td><td>WREN</td><td>
     * Some RO fields are writable over DIF</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t cxDbiRoWrEn;
    /**
     * [rw] Default target a received IO or MEM request with UR/CA/CRS
     *
     * 0: The controller drops all incoming I/O or MEM requests [after
     *    corresponding error reporting] A completion with UR status will be
     *    generated for non-posted requests -
     * 1: The controller forwards all incoming I/O or MEM requests with
     *    UR/CA/CRS status to your application Default value is
     *    DEFAULT_TARGET configuration parameter Note: This register field
     *    is sticky
     *
     * Field size: 1 bit
     */
    uint8_t defaultTarget;
    /**
     * [rw] UR_CA_MASK_4_TRGT1
     *
     * This field only applies to request TLPs [with UR filtering status] that
     * you have chosen to forward to the application [when you set
     * DEFAULT_TARGET in this register] -
     *
     * '1' the core suppresses error logging, Error Message generation, and
     * CPL generation [for non-posted requests] - You should set this if you
     * have set the Default Target port logic register to '1' Default is
     * CX_MASK_UR_CA_4_TRGT1 configuration parameter
     *
     * Note: This register field is sticky
     *
     * Field size: 1 bit
     */
    uint8_t urCaMask4Trgt1;
    /**
     * [rw] Enables Simplified Replay Timer [Gen4]
     *
     * Note: This register field is sticky
     *
     * Field size: 1 bit
     */
    uint8_t simpReplayTimer;
    /**
     * [rw] When ARI is enabled, enables use of the device ID
     *
     * Note: This register field is sticky
     *
     * Field size: 1 bit
     */
    uint8_t ariDevNumber;
} Pcie_PlconfDbiRoWrEnReg;

/**
 * \brief Specification of the AXI Slave Error Response Register (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfAxiSlvErrRespReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Global Slave Error Response Mapping
     *
     * Field size: 1 bit
     */
    uint8_t slaveErrMap;
    /**
     * [rw] DIF Slave Error Response Mapping
     *
     * Field size: 1 bit
     */
    uint8_t dbiErrMap;
    /**
     * [rw] Vendor ID Non-existent Slave Error Response Mapping
     *
     * Field size: 1 bit
     */
    uint8_t noVidErrMap;
    /**
     * [rw] Graceful Reset and Link Timeout Slave Error Response Mapping
     *
     * Field size: 1 bit
     */
    uint8_t resetTimeoutErrMap;
    /**
     * [rw] CRS Slave Error Response Mapping
     *
     * CRS Slave Error Response Mapping Determines the AXI slave
     * response for CRS completions AHB: - always returns OKAY AXI: -
     * 00: OKAY
     * 01: OKAY with all FFFF_FFFF data for all CRS completions
     * 10: OKAY with FFFF_0001 data for CRS completions to vendor ID
     *     read requests, OKAY with FFFF_FFFF data for all other CRS
     *     completions
     * 11: SLVERR/DECERR [the AXI_ERROR_RESPONSE_MAP field
     *     determines the PCIe-to-AXI Slave error response mapping]
     *
     * This register field is sticky
     *
     * Field size: 2 bit
     */
    uint8_t errorResponseCrs;
    /**
     * [rw] AXI Slave Response Error Map
     *
     * AXI Slave Response Error Map Allows you to selectively map the
     * errors received from the PCIe completion [for non-posted requests]
     * to the AXI slave responses, slv_rresp or slv_bresp The
     * recommended setting is SLVERR CRS is always mapped to OKAY -
     * [bit 0] 0: UR [unsupported request] -> DECERR
     *         1: UR [unsupported request] -> SLVERR
     * [bit 1] 0: CRS [configuration retry status] -> DECERR
     *         1: CRS [configuration retry status] -> SLVERR
     * [bit 2] 0: CA [completer abort] -> DECERR
     *         1: CA [completer abort] -> SLVERR
     * [bit 3]: Reserved
     * [bit 4]: Reserved
     * [bit 5]: 0: Completion Timeout -> DECERR
     *          1: Completion Timeout -> SLVERR
     * The AXI bridge internally drops
     * [processes internally but not passed to your application] a
     * completion that has been marked by the Rx filter as UC or MLF, and
     * does not pass its status directly down to the slave interface It waits
     * for a timeout and then signals "Completion Timeout" to the slave
     * interface The controller sets the AXI slave read databus to 0xFFFF
     * for all error responses
     *
     * Note: This register field is sticky
     *
     * Field size: 6 bits
     */
    uint8_t errorResponseMap;
} Pcie_PlconfAxiSlvErrRespReg;

/**
 * \brief Specification of the Link Down AXI Slave Timeout Register (Sticky)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfAxiSlvTimeoutReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Timeout Value (ms)
     *
     * Field size: 8 bits
     */
    uint8_t timeoutValue;
    /**
     * [rw] Enable flush
     *
     * Field size: 1 bit
     */
    uint8_t flushEn;
} Pcie_PlconfAxiSlvTimeoutReg;

/**
 * \brief Specification of the iATU Viewport Register
 *
 * makes the registers of the corresponding iATU region accessible
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfIatuIndexReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Region Direction
     *
     * <table>
     * <tr><th>Value</th><th>Mode</th></tr>
     * <tr><td>0x0</td><td>OUTBOUND</td></tr>
     * <tr><td>0x1</td><td>INBOUND</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t regionDirection;
    /**
     * [rw] Region Index
     *
     * Outbound region, from 0 to 15.
     * Inbound region, from 0 to 3 or 0 to 15
     *
     * Field size: 4 bits
     */
    uint8_t regionIndex;
} Pcie_PlconfIatuIndexReg;

/**
 * \brief Specification of the iATU Region Control 1 Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfIatuRegCtrl1Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] type
     *
     * Outbound: TYPE applied to outgoing TLP with matching addess
     * Inbound: TYPE-match criteria
     *
     * Field size: 5 bits
     */
    uint8_t type;
    /**
     * [rw] TC
     *
     * Outbound: TC applied to outgoing TLP with matching addess
     * Inbound: TC-match criteria (if TC_match_enable=1)
     *
     * Field size: 3 bits
     */
    uint8_t tc;
    /**
     * [rw] TD
     *
     * Outbound: TD applied to outgoing TLP with matching addess
     * Inbound: TD-match criteria (if TD_match_enable=1)
     *
     * Field size: 1 bit
     */
    uint8_t td;
    /**
     * [rw] ATTR
     *
     * Outbound: ATTR applied to outgoing TLP with matching addess
     * Inbound: ATTR-match criteria (if ATTR_match_enable=1)
     *
     * Field size: 2 bits
     */
    uint8_t attr;
    /**
     * [rw] AT
     *
     * Outbound: AT applied to outgoing TLP with matching addess
     * Inbound: AT-match criteria for matching TLP (if AT_match_enable=1)
     *
     * Field size: 2 bits
     */
    uint8_t at;
    /**
     * [rw] Increase the maximum ATU Region size
     *
     * Outbound: F.N; applied to outgoing TLP (RID) with RW 0x0 matching addess
     * Inbound: F.N.-match criteria for incoming TLP
     *          (if Function_Number_match_enable=1)
     *
     * Field size: rev 1: 5 bits; rev 2: 3 bits
     */
    uint8_t increaseRegionSize;
    /**
     * [rw] function number
     *
     * Outbound: F.N; applied to outgoing TLP (RID) with RW 0x0 matching addess
     * Inbound: F.N.-match criteria for incoming TLP
     *          (if Function_Number_match_enable=1)
     *
     * Field size: rev 1: 5 bits; rev 2: 3 bits
     */
    uint8_t functionNumber;
} Pcie_PlconfIatuRegCtrl1Reg;

/**
 * \brief Specification of the iATU Region Control 2 Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfIatuRegCtrl2Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MESSAGECODE
     *
     * Outbound: MessageCode applied to outgoing message RW 0x0 TLP with
     *           matching addess
     * Inbound: MessageCode-match criteria for infoming message TLP
     *          (if Message_Code_match_enable=1)
     *
     * Field size: 8 bits
     */
    uint8_t messagecode;
    /**
     * [rw] BAR_NUMBER
     *
     * BAR number for mayching with incoming MEM, I/O TLP RW 0x0
     * (if Match_Mode = 1)
     *
     * <table>
     * <tr><th>Value</th><th>Mode</th></tr>
     * <tr><td>0x0</td><td>BAR0</td></tr>
     * <tr><td>0x1</td><td>BAR1</td></tr>
     * <tr><td>0x2</td><td>BAR2</td></tr>
     * <tr><td>0x3</td><td>BAR3</td></tr>
     * <tr><td>0x4</td><td>BAR4</td></tr>
     * <tr><td>0x5</td><td>BAR5</td></tr>
     * <tr><td>0x6</td><td>ROM</td></tr>
     * </table>
     *
     * Field size: 3 bits
     */
    uint8_t barNumber;
    /**
     * [rw] TAG
     *
     * The substituted TAG field [byte 6] in the outgoing TLP header
     * when TAG_SUBSTITUTE_EN is set
     *
     * Field size: 8 bits
     */
    uint8_t tag;
    /**
     * [rw]
     *
     * Field size: 1 bit
     */
    uint8_t msgTypeMatchMode;
    /**
     * [rw] Enable TC match criteria on inbound TLP
     *
     * Field size: 1 bit
     */
    uint8_t tcMatchEnable;
    /**
     * [rw] Enable TD match criteria on inbound TLP
     *
     * Field size: 1 bit
     */
    uint8_t tdMatchEnable;
    /**
     * [rw] Enable ATTR match criteria on inbound TLP
     *
     * Field size: 1 bit
     */
    uint8_t attrMatchEnable;
    /**
     * [rw] Enable AT match criteria on inbound TLP
     *
     * ATS NOT SUPPORTED: DO NOT USE
     *
     * Field size: 1 bit
     */
    uint8_t atMatchEnable;
    /**
     * [rw] TAG Substitute Enable
     *
     * Field size: 1 bit
     */
    uint8_t tagSubstEn;
    /**
     * [rw] function number match enable
     *
     * Outbound: Function Number Translation Bypass
     * Inbound: Enable Function Number match criteria
     *
     * Field size: 1 bit
     */
    uint8_t functionNumberMatchEnable;
    /**
     * [rw] VIRTUAL FUNCTIONS NOT IMPLEMENTED
     *
     * Field size: 1 bit
     */
    uint8_t virtualFunctionNumberMatchEnable;
    /**
     * [rw] Serialize Non-Posted Requests
     *
     * Field size: 1 bit
     */
    uint8_t SNP;
    /**
     * [rw] Enable MessageCode match criteria on inbound TLP
     *
     * Field size: 1 bit
     */
    uint8_t messageCodeMatchEnable;
    /**
     * [rw] Inhibit TLP Payload Data for TLP's in Matched Region
     *
     * Field size: 1 bit
     */
    uint8_t inhibitPayload;
    /**
     * [rw] Header Substitute Enable (outbound)
     *
     * Field size: 1 bit
     */
    uint8_t headerSubstEn;
    /**
     * [rw] Single Address Location Translate Enable (inbound)
     *
     * Field size: 1 bit
     */
    uint8_t singleAddrLocTransEn;
    /**
     * [rw] Override HW-generated completion status when responding inbound TLP
     *
     * <table>
     * <tr><th>Value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>No override, use HW-generated CS</td></tr>
     * <tr><td>0x1</td><td>Unsupported Request: CS= 3'b001</td></tr>
     * <tr><td>0x2</td><td>Completer Abort: CS= 3'b100</td></tr>
     * </table>
     *
     * Field size: 2 bits
     */
    uint8_t responseCode;
    /**
     * [rw] fuzzy type match mode
     *
     * Outbound: DMA Bypass Mode RW 0x0
     * Inbound: Relax matching on inbound TLP TYPE:
     * CfgRd0 == CfgRd1
     * CfgWr0 == CfgWr1
     * MRd == MRdLk
     * routing field of Msg/MsgD ignored
     *
     * Field size: 1 bit
     */
    uint8_t fuzzyTypeMatchMode;
    /**
     * [rw] Enable the shifting of CFG CID (BDF),
     *
     * Incoming and outgoing TLP;
     * CFG get mapped to a contiguous 2**28 = * 256 MByte address space
     * Untranslated CID = CFG_DW#3[31:16]
     * Shifted CID = CFG_DW#3[27:12]
     *
     * Field size: 1 bit
     */
    uint8_t cfgShiftMode;
    /**
     * [rw] Redefine match criteria as outside the defined range
     *
     * (instead of inside)
     *
     * Field size: 1 bit
     */
    uint8_t invertMode;
    /**
     * [rw] Sets inbound TLP match mode
     *
     * On rev 1 hw, this corresponds to MATCH_MODE
     * On rev 2 hw, this corresponds to MATCH_MODE (inbound)
     *
     * Depending on TYPE
     * 0x0: MEM,I/O: Address Match: as per region base & limit registers;
     *      CFG0: Routing ID Match: Completer ID (BDF) + reg
     *            address matches base & limit-defined region;
     *      MSG[D]: Address Match: as per region base & limit registers
     * 0x1: MEM,I/O: BAR match: as defined in BAR_number field;
     *      CFG0: Accept mode: Completer ID (BDF) is ignored;
     *      MSG[D]: VendorID match: VendorID = upper_base[15:0] +
     *              VendorDefined = lower_base/limit
     *
     * Field size: 1 bit
     */
    uint8_t matchMode;
    /**
     * [rw] Enable AT for this region
     *
     * Field size: 1 bit
     */
    uint8_t regionEnable;
} Pcie_PlconfIatuRegCtrl2Reg;

/**
 * \brief Specification of the iATU Region Lower Base Address Register
 * (2**12 = 4kbyte - aligned)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfIatuRegLowerBaseReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Lower Base Address (read-write part)
     *
     * Note: On rev 1 @ref iatuRegLowerBase is >> 12 version of address, with 0 LSBs cut off
     * Note: On rev 2 @ref iatuRegLowerBase is >> 16 version of address, with 0 LSBs cut off
     *
     * Field size: 20 bits (rev 1) or 16 bits (rev 2)
     */
    uint32_t iatuRegLowerBase;
    /**
     * [ro] Lower Base Address (read-only part)
     *
     * Field size: 12 bits (rev 1) or 16 bits (rev 2)
     */
    uint16_t zero;
} Pcie_PlconfIatuRegLowerBaseReg;

/**
 * \brief Specification of the iATU Region Upper Base Address Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfIatuRegUpperBaseReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Upper Base Address
     *
     * Field size: 32 bits
     */
    uint32_t iatuRegUpperBase;
} Pcie_PlconfIatuRegUpperBaseReg;

/**
 * \brief Specification of the iATU Region Limit Address Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfIatuRegLimitReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Region limit address
     *
     * Field size: 20 bits or 16 bits
     */
    uint32_t iatuRegLimit;
    /**
     * [ro] Region limit address (mask)
     *
     * On rev 1 hw, this corresponds to ONES
     * On rev 2 hw, this corresponds to LIMIT_ADDR_HW
     *
     * This portion is always all 1s to enforce 4K alignment or 64K
     *
     * Field size: 12 bits or 16 bits
     */
    uint16_t ones;
} Pcie_PlconfIatuRegLimitReg;

/**
 * \brief Specification of the iATU Region Lower Target Address Register
 * (2**12 = 4kbyte - aligned)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfIatuRegLowerTargetReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Lower Target Address (read-write part)
     *
     * Field size: 20 bits or 16 bits
     */
    uint32_t iatuRegLowerTarget;
    /**
     * [ro] Lower Target Address (read-only part)
     *
     * This portion is always 0 to enforce 4K alignment or 64K
     *
     * Field size: 12 bits or 16 bits
     */
    uint16_t zero;
} Pcie_PlconfIatuRegLowerTargetReg;

/**
 * \brief Specification of the iATU Region Upper Target Address Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct  pciePlconfIatuRegUpperTargetReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Upper Target Address
     *
     * Field size: 32 bits
     */
    uint8_t iatuRegUpperTarget;
} Pcie_PlconfIatuRegUpperTargetReg;

/**
 * \brief Specification of the iATU Region Control 3 Register
 *
 * VIRTUAL FUNCTIONS NOT IMPLEMENTED: NOT USED
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This is an entirely [ro] register
 */
typedef struct  pciePlconfIatuRegCtrl3Reg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] IATU CTRL 3
     *
     * This register is not used as virtual functions are not supported.  It is
     * always 0.
     *
     * Field size: 32 bits
     */
    uint32_t iatuRegCtrl3;
} Pcie_PlconfIatuRegCtrl3Reg;

/**
 * \brief Specification of the TI CONF Revision register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is completely RO.
 */
typedef struct pcieTiConfRevisionReg_s {
    uint32_t raw;   /** [ro] Raw image of register on read; actual value on write */
    uint8_t yMinor; /** [ro] minor rev - 6 bits*/
    uint8_t custom; /** [ro] customer special rev - 2 bits */
    uint8_t xMajor; /** [ro] major rev - 3 bits*/
    uint8_t rRtl;   /** [ro] RTL rev - 5 bits */
    uint8_t func;   /** [ro] function code - 12 bits */
    uint8_t scheme; /** [ro] scheme - 2 bits */
    uint8_t bu;     /** [ro] business unit - 2 bits */
} Pcie_TiConfRevisionReg;

/**
 * \brief Specification of the TI CONF Sys Config Register
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_SYSCONFIG)
 */
typedef struct pcieTiConfSysConfigReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] PM mode of local target (slave)
     *
     * PM mode of local target (slave); Target shall be capable
     * RW 0x2 of handling read/write transaction as long as it is
     * out of IDLE state.
     *
     * <table>
     * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
     * <tr><td>0x0</td><td>Force-idle</td><td>
     * local target's idle state follows (acknowledges) the system's idle
     * requests unconditionally, regardless of the IP module's internal
     * requirements.
     * </tr><tr><td>0x1</td><td>No-idle</td><td>
     * local target never enters idle state.
     * </tr><tr><td>0x2</td><td>Smart-idl</td><td>
     * local target's idle state eventually follows (acknowledges) the
     * system's idle requests, depending on the IP module's internal
     * requirements.  Module shall not generate (IRQ- or DMA-request-related)
     * wakeup events.
     * </tr><tr><td>0x3</td><td>Smart-idle wakeup-capable</td><td>
     * local target's idle state eventually follows (acknowledges) the
     * system's idle requests, depending on the IP module's internal
     * requirements. IP module may generate (IRQ- or DMArequest-
     * related) wakeup events when in idle state.
     * </td></tr></table>
     *
     * Field size: 2 bits
     */
    uint8_t idlemode;
    /**
     * [rw] number of fast training sequences
     *
     * PM mode of local initiator (master); Initiator may generate read/write
     * transaction as long as it is out of STANDBY state.
     *
     * <table>
     * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
     * <tr><td>0x0</td><td>Force-standby</td><td>
     * Initiator is unconditionally placed in standby state.
     * </tr><tr><td>0x1</td><td>No-standby</td><td>
     * initiator is unconditionally placed out of standby state.
     * </tr><tr><td>0x2</td><td>Smart-standby</td><td>
     * initiator's standby state depends on internal conditions, i.e. the
     * module's functional requirements. Asynchronous wakeup events
     * cannot be generated.
     * </tr><tr><td>0x3</td><td>Smart-standby, wakeup-capable</td><td>
     * initiator's standby state depends on internal conditions, ie the
     * module's functional requirements. Asynchronous wakeup events can
     * be generated.
     * </td></tr></table>
     *
     * Field size: 2 bits
     */
    uint8_t standbymode;
    /**
     * [rw] no-snoop to coherent mapping
     *
     * Allows the no-snoop (NS) attribute of inbound PCIe TLPs to be passed
     * to SoC system bus (AXI) master as a 'coherent' inband flag.
     *
     * <table>
     * <tr><th>Value</th><th>Mode</th><th>description</th></tr>
     * <tr><td>0x0</td><td>DIS</td><td>AXI not coherent</td></tr>
     * <tr><td>0x1</td><td>EN</td><td>
     * AXI coherent = not(PCIE "NS") i.e. cache-coherence is preserved
     * </td></tr></table>
     *
     * Field size: 1 bit
     */
    uint8_t mcoherentEn;
} Pcie_TiConfSysConfigReg;

/**
 * \brief Specification of the TI CONF IRQ EOI Register
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfIrqEoiReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] IRQ Line Number to EOI
     *
     * Write the IRQ line number to apply SW EOI to it.
     * Write 0x0: SW EOI on main interrupt line
     * Write 0x1: SW EOI on message-signalled (MSI) interrupt line
     *
     * Read always returns zeros
     *
     * Field size: 4 bits
     */
    uint8_t lineNumber;
} Pcie_TiConfIrqEoiReg;

/**
 * \brief Specification of the TI CONF IRQ Main common spec
 *
 * This structure is used for reading/writing 4 different registers:
 * Pcie_TiConfIrqStatusRawMainReg
 * Pcie_TiConfIrqStatusMainReg
 * Pcie_TiConfIrqEnableSetMainReg
 * Pcie_TiConfIrqEnableClrMainReg.
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfIrqMain_s {
    uint32_t raw;        /** [ro] Raw image of register on read; actual value on write */
    uint8_t errSys;      /** System Error IRQ */
    uint8_t errFatal;    /** Fatal Error message received IRQ */
    uint8_t errNonfatal; /** Non-Fatal Error message received IRQ */
    uint8_t errCor;      /** Correctable Error message received IRQ */
    uint8_t errAxi;      /** AXI tag lookup fatal Error IRQ */
    uint8_t errEcrc;     /** ECRC Error IRQ */
    uint8_t pmeTurnOff;  /** Power Management Event Turn-Off message received IRQ */
    uint8_t pmeToAck;    /** Power Management Event Turn-Off Ack message IRQ */
    uint8_t pmPme;       /** PM Power Management Event message received IRQ */
    uint8_t linkReqRst;  /** Link Request Reset IRQ */
    uint8_t linkUpEvt;   /** Link-up state change IRQ */
    uint8_t cfgBmeEvt;   /** CFG "Bus Master Enable" change IRQ */
    uint8_t cfgMseEvt;   /** CFG "Memory Space Enable" change IRQ */
} Pcie_TiConfIrqMain;

/**
 * \brief Specification of the TI CONF IRQ Status Raw main Register
 *
 * Raw status of 'main' interrupt requests; Set even if event is not enabled.
 * Write 1 to set the (raw) status, mostly for debug (regular status also gets set).
 *
 * For each IRQ in @ref Pcie_TiConfIrqMain
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Triggers IRQ Event by software</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event not pending</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is pending</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 */
typedef Pcie_TiConfIrqMain Pcie_TiConfIrqStatusRawMainReg;

/**
 * \brief Specification of the TI CONF IRQ Status Main register
 *
 * Regular status of 'main' interrupt requests; Set only when enabled.
 * Write 1 to clear after interrupt has been serviced
 * (raw status also gets cleared).
 *
 * For each IRQ in @ref Pcie_TiConfIrqMain
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Clear pending IRQ Event, if any</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event not pending</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is pending</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef Pcie_TiConfIrqMain Pcie_TiConfIrqStatusMainReg;

/**
 * \brief Specification of the TI CONF IRQ Enable Set register
 *
 * Enable of 'main' interrupt requests; Write 1 to set (ie to enable
 * interrupt). Readout is the same as corresponding _CLR register.
 *
 * For each IRQ in @ref Pcie_TiConfIrqMain
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Enable event</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event is disabled</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is enabled</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef Pcie_TiConfIrqMain Pcie_TiConfIrqEnableSetMainReg;

/**
 * \brief Specification of the TI CONF IRQ Enable Clear register
 *
 * Enable of 'main' interrupt requests; Write 1 to clear
 * (ie to disable interrupt). Readout is the same
 * as corresponding _SET register.
 *
 * For each IRQ in @ref Pcie_TiConfIrqMain
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Disable event</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event is disabled</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is enabled</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef Pcie_TiConfIrqMain Pcie_TiConfIrqEnableClrMainReg;

/**
 * \brief Specification of the TI CONF IRQ MSI common spec
 *
 * This structure is used for reading/writing 4 different registers:
 * Pcie_TiConfIrqStatusRawMsiReg
 * Pcie_TiConfIrqStatusMsiReg
 * Pcie_TiConfIrqEnableSetMsiReg
 * Pcie_TiConfIrqEnableClrMsiReg.
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfIrqMsi_s {
    uint32_t raw; /** [ro] Raw image of register on read; actual value on write */
    uint8_t inta; /** INTA IRQ */
    uint8_t intb; /** INTB IRQ */
    uint8_t intc; /** INTC IRQ */
    uint8_t intd; /** INTD IRQ */
    uint8_t msi;  /** Message Signaled Interrupt (MSI) IRQ */
} Pcie_TiConfIrqMsi;

/**
 * \brief Specification of the TI CONF IRQ Status Raw MSI Register
 *
 * Raw status of legacy and MSI interrupt requests; Set even if
 * event is not enabled. Write 1 to set the (raw) status, mostly
 * for debug (regular status also gets set).
 *
 * For each IRQ in @ref Pcie_TiConfIrqMsi
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Triggers IRQ Event by software</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event not pending</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is pending</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef Pcie_TiConfIrqMsi Pcie_TiConfIrqStatusRawMsiReg;

/**
 * Specification of the TI CONF IRQ Status MSI register
 *
 * Regular status of legacy and MSI interrupt requests; Set only
 * when enabled. Write 1 to clear after interrupt has been serviced
 * (raw status also gets cleared). HW-generated events are self-clearing.
 *
 * For each IRQ in @ref Pcie_TiConfIrqMsi
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Clear pending IRQ Event, if any</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event not pending</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is pending</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * This register is only available on rev 1 hw (TI_CONF_IRQSTATUS_MSI)
 */
typedef Pcie_TiConfIrqMsi Pcie_TiConfIrqStatusMsiReg;

/**
 * \brief Specification of the TI CONF IRQ MSI Enable Set register
 *
 * Enable of legacy and MSI interrupt requests; Write 1 to set (ie to
 * enable interrupt). Readout is the same as corresponding _CLR register.
 *
 * For each IRQ in @ref Pcie_TiConfIrqMsi
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Enable event</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event is disabled</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is enabled</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef Pcie_TiConfIrqMsi Pcie_TiConfIrqEnableSetMsiReg;

/**
 * \brief Specification of the TI CONF IRQ MSI Enable Clear register
 *
 * Enable of legacy and MSI interrupt requests; Write 1 to clear
 * (ie to disable interrupt).  Readout is the same as corresponding
 * _SET register.
 *
 * For each IRQ in @ref Pcie_TiConfIrqMsi
 * <table>
 * <tr><th>action</th><th>value</th><th>description</th></tr>
 * <tr><td>Write</td><td>0</td><td>No effect</td></tr>
 * <tr><td>Write</td><td>1</td><td>Disable event</td></tr>
 * <tr><td>Read</td><td>0</td><td>IRQ event is disabled</td></tr>
 * <tr><td>Read</td><td>1</td><td>IRQ event is enabled</td></tr>
 * </table>
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef Pcie_TiConfIrqMsi Pcie_TiConfIrqEnableClrMsiReg;

/**
 * \brief Specification of the TI CONF Device Type register
 *
 * Sets the Dual-Mode device's type
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfDeviceTypeReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Device Type
     *
     * PCIe device type including the contents of the PCI config
     * space (Type-0 for EP, Type-1 for RC); Apply fundamental
     * reset after change; Do not change during core operation.
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>PCIe endpoint (EP)</td></tr>
     * <tr><td>0x1</td><td>Legacy PCIe endpoint (LEG_EP)</td></tr>
     * <tr><td>0x4</td><td>Root Complex (RC)</td></tr>
     * <tr><td>Other values</td><td>Reserved</td></tr>
     * </table>
     *
     * Field size: 4 bits
     */
    uint8_t type;
} Pcie_TiConfDeviceTypeReg;

/**
 * \brief Specification of the TI CONF Device Command register
 *
 * Device command (startup control and status);
 * WARNING: cleared by all reset conditions, including fundamental reset
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfDeviceCmdReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] LTSSM state/substate implementation-specific, for
     * debug (@ref pcieLtssmState_e)
     *
     * <table>
     * <tr><th>read value</th><th>description</th></tr>
     * <tr><td>0x00</td><td>DETECT_QUIET</td></tr>
     * <tr><td>0x01</td><td>DETECT_ACT</td></tr>
     * <tr><td>0x02</td><td>POLL_ACTIVE</td></tr>
     * <tr><td>0x03</td><td>POLL_COMPLIANCE</td></tr>
     * <tr><td>0x04</td><td>POLL_CONFIG</td></tr>
     * <tr><td>0x05</td><td>PRE_DETECT_QUIET</td></tr>
     * <tr><td>0x06</td><td>DETECT_WAIT</td></tr>
     * <tr><td>0x07</td><td>CFG_LINKWD_START</td></tr>
     * <tr><td>0x08</td><td>CFG_LINKWD_ACEPT</td></tr>
     * <tr><td>0x09</td><td>CFG_LANENUM_WAIT</td></tr>
     * <tr><td>0x0A</td><td>CFG_LANENUM_ACEPT</td></tr>
     * <tr><td>0x0B</td><td>CFG_COMPLETE</td></tr>
     * <tr><td>0x0C</td><td>CFG_IDLE</td></tr>
     * <tr><td>0x0D</td><td>RCVRY_LOCK</td></tr>
     * <tr><td>0x0E</td><td>RCVRY_SPEED</td></tr>
     * <tr><td>0x0F</td><td>RCVRY_RCVRCFG</td></tr>
     * <tr><td>0x10</td><td>RCVRY_IDLE</td></tr>
     * <tr><td>0x11</td><td>L0</td></tr>
     * <tr><td>0x12</td><td>L0S</td></tr>
     * <tr><td>0x13</td><td>L123_SEND_EIDLE</td></tr>
     * <tr><td>0x14</td><td>L1_IDLE</td></tr>
     * <tr><td>0x15</td><td>L2_IDLE</td></tr>
     * <tr><td>0x16</td><td>L2_WAKE</td></tr>
     * <tr><td>0x17</td><td>DISABLED_ENTRY</td></tr>
     * <tr><td>0x18</td><td>DISABLED_IDLE</td></tr>
     * <tr><td>0x19</td><td>DISABLED</td></tr>
     * <tr><td>0x1A</td><td>LPBK_ENTRY</td></tr>
     * <tr><td>0x1B</td><td>LPBK_ACTIVE</td></tr>
     * <tr><td>0x1C</td><td>LPBK_EXIT</td></tr>
     * <tr><td>0x1D</td><td>LPBK_EXIT_TIMEOUT</td></tr>
     * <tr><td>0x1E</td><td>HOT_RESET_ENTRY</td></tr>
     * <tr><td>0x1F</td><td>HOT_RESET</td></tr>
     * <tr><td>0x20</td><td>RCVRY_EQ0</td></tr>
     * <tr><td>0x21</td><td>RCVRY_EQ1</td></tr>
     * <tr><td>0x22</td><td>RCVRY_EQ2</td></tr>
     * <tr><td>0x23</td><td>RCVRY_EQ3</td></tr>
     * </table>
     *
     * Field size: 6 bits
     */
    uint8_t ltssmState;
    /**
     * [rw] LTSSM enable: start the PCI link
     *
     * Set bit to start PCIE link training.  Note: this bit is
     * CLEARED BY FUNDAMENTAL RESET)
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>DISABLED</td></tr>
     * <tr><td>0x1</td><td>ENABLED</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t ltssmEn;
    /**
     * [rw] Application Request Retry Enable
     *
     * Application Request Retry Enable.  Note: this bit is CLEARED
     * BY FUNDAMENTAL RESET)
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>DISABLED:
     *   Incoming PCI transactions are processed normally</td></tr>
     * <tr><td>0x1</td><td>ENABLED:
     *   Incoming PCI transactions are responded with "retry"</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t appReqRetryEn;
    uint8_t devNum; /** [ro] PCIe device number (5 bits) */
    uint8_t busNum; /** [ro] PCIe bus number (8 bits) */
} Pcie_TiConfDeviceCmdReg;

/**
 * \brief Specification of the TI CONF PM Control Register
 *
 * Power Management Control
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfPmCtrlReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [wo] Transmits PME_Turn_Off message downstream (RC mode only)
     *
     * Eventually sends all links of hierarchy domain to L2L/3_ready
     *
     * <table>
     * <tr><th>write value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>NOACTION</td></tr>
     * <tr><td>0x1</td><td>TRANSMIT</td></tr>
     * </table>
     *
     * Reads always return 0.
     *
     * Field size: 1 bit
     */
    uint8_t pmeTurnOff;
    /**
     * [wo] Transmits PM_PME wakeup message (EP mode only)
     *
     * <table>
     * <tr><th>write value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>NOACTION</td></tr>
     * <tr><td>0x1</td><td>TRANSMIT</td></tr>
     * </table>
     *
     * Reads always return 0.
     *
     * Field size: 1 bit
     */
    uint8_t pmPme;
    /**
     * [rw] Indicates system readiness for the link to enter L2/L3
     *
     * Allows the transmission of PM_Enter_L23 following
     * PM_Turn_OFF / PME_TO_Ack handshake. Self-cleared upon transition to L2/L3.
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>NOT_READY</td></tr>
     * <tr><td>0x1</td><td>READY</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t l23Ready;
    /**
     * [rw] Request to transition to L1 state
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>INACTIVE (No request)</td></tr>
     * <tr><td>0x1</td><td>ACTIVE (L1 entry request)</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t reqEntrL1;
    /**
     * [rw] Request to exit L1 state (to L0)
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>INACTIVE (No request)</td></tr>
     * <tr><td>0x1</td><td>ACTIVE (L1 exit request)</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t reqExitL1;
    /**
     * [rw] Auxilliary Power Detection
     *
     * Status of Vaux detection for the PCIe controller.
     * Determines transition to L2 vs L3 upon Vmain turn-off.
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>UNPOWERED:
     *         Vaux not present: D3cold maps to L3 link state</td></tr>
     * <tr><td>0x1</td><td>POWERED:
     *         Vaux present: D3cold maps to L2 link state</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t auxPwrDet;
} Pcie_TiConfPmCtrlReg;

/**
 * \brief Specification of the TI CONF PHY CS
 *
 * Physical Layer Control and Status
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfPhyCsReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Link status, from LTSSM
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>DOWN</td></tr>
     * <tr><td>0x1</td><td>UP</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t linkUp;
    /**
     * [rw] Manual lane reversal control
     *
     * Manual lane reversal control, allowing lane 0 and lane 1
     * to be swapped by default; Both Tx and Rx are reversed;
     * Polarity of the individual lane is unchanged
     *
     * <table>
     * <tr><th>value</th><th>description</th></tr>
     * <tr><td>0x0</td><td>STRAIGHT</td></tr>
     * <tr><td>0x1</td><td>REVERSED</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t reverseLanes;
} Pcie_TiConfPhyCsReg;

/**
 * Specification of the TI CONF INTX ASSERT
 *
 * Legacy INTx ASSERT message control, with 'x' in (A,B,C,D) set by
 * the 'Interrupt Pin' field.
 *
 * Write 1 to send message, read to get the status; EP mode only
 *
 * This register may be used for endpoint mode only.
 */
typedef struct pcieTiConfIntxAssertReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] INTx ASSERT for function 0
     *
     * <table>
     * <tr><th>action/value</th><th>description</th></tr>
     * <tr><td>Write 0x0</td><td>No action</td></tr>
     * <tr><td>Write 0x1</td><td>Transmit INTx ASSERT to RC</td></tr>
     * <tr><td>Read 0x0</td><td>INTx is inactive (deasserted)</td></tr>
     * <tr><td>Read 0x1</td><td>INTx is active (asserted)</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t assertF0;
} Pcie_TiConfIntxAssertReg;

/**
 * Specification of the TI CONF INTX DEASSERT
 *
 * Legacy INTx DEASSERT message control, with 'x' in (A,B,C,D) set by
 * the 'Interrupt Pin' field.
 *
 * Write 1 to send message, read to get the status; EP mode only
 *
 * This register may be used for endpoint mode only.
 */
typedef struct pcieTiConfIntxDeassertReg_s {
    uint32_t raw; /** [ro] Raw image of register on read; actual value on write */
    /**
     * [rw] INTx ASSERT for function 0
     *
     * <table>
     * <tr><th>action/value</th><th>description</th></tr>
     * <tr><td>Write 0x0</td><td>No action</td></tr>
     * <tr><td>Write 0x1</td><td>Transmit INTx DEASSERT to RC</td></tr>
     * <tr><td>Read 0x0</td><td>INTx is inactive (deasserted)</td></tr>
     * <tr><td>Read 0x1</td><td>INTx is active (asserted)</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t deassertF0;
} Pcie_TiConfIntxDeassertReg;

/**
 * \brief Specification of the TI CONF MSI XMT Register
 *
 * MSI transmitter (EP mode); Specifies parameters of MSI, together
 * with MSI capability descriptor already configured by remote RC.
 *
 * This register may be used for endpoint mode only.
 */
typedef struct pcieTiConfMsiXmtReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] MSI transmit request (and grant status)
     *
     * <table>
     * <tr><th>action/value</th><th>description</th></tr>
     * <tr><td>Write 0x0</td><td>No action</td></tr>
     * <tr><td>Write 0x1</td><td>Request MSI transmission</td></tr>
     * <tr><td>Read 0x0</td><td>MSI transmission request pending</td></tr>
     * <tr><td>Read 0x1</td><td>No MSI request pending (last request granted)</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t msiReqGrant;
    /**
     * [rw] Function number for transmitted MSI
     *
     * Always 0 for single-function EP
     *
     * Field size: 3 bits
     */
    uint8_t msiFuncNum;
    /**
     * [rw] Vector number for transmitted MSI
     *
     * (as allowed by RC at RW 0x0 enumeration)
     *
     * Field size: 5 bits
     */
    uint8_t msiVector;
    /**
     * [rw] Traffic class (TC) for transmitted MSI
     *
     * Field size: 3 bits
     */
    uint8_t msiTc;
} Pcie_TiConfMsiXmtReg;

/**
 * \brief Specification of the TI CONF Debug Config
 *
 * Configuration of debug_data output and register (observability)
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfDebugCfgReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Debug_data mode
     *
     * Field size: 6 bits
     */
    uint8_t sel;
} Pcie_TiConfDebugCfgReg;

/**
 * \brief Specification of the TI CONF Debug Data
 *
 * Debug data vector, depending on DEBUG_CFG.sel value
 *
 * This register may be used for both endpoint and root complex modes.
 *
 * Entire register is [RO]
 */
typedef struct pcieTiConfDebugDataReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [ro] Debug
     *
     * Field size: 32 bits
     */
    uint32_t debug;
} Pcie_TiConfDebugDataReg;

/**
 * Specification of the TI CONF Diag Control
 *
 * Diagnostic control
 *
 * This register may be used for both endpoint and root complex modes.
 */
typedef struct pcieTiConfDiagCtrlReg_s {
    /** [ro] Raw image of register on read; actual value on write */
    uint32_t raw;
    /**
     * [rw] Corrupts LSB of LCRC in the next packet, then self-clears.
     *
     * <table>
     * <tr><th>action/value</th><th>description</th></tr>
     * <tr><td>Write 0x0</td><td>not defined</td></tr>
     * <tr><td>Write 0x1</td><td>Request CRC corruption</td></tr>
     * <tr><td>Read 0x0</td><td>No CRC corruption pending</td></tr>
     * <tr><td>Read 0x1</td><td>CRC corruption pending</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t invLcrc;
    /**
     * [rw] Corrupts LSB of ECRC in the next packet, then self-clears.
     *
     * <table>
     * <tr><th>action/value</th><th>description</th></tr>
     * <tr><td>Write 0x0</td><td>not defined</td></tr>
     * <tr><td>Write 0x1</td><td>Request CRC corruption</td></tr>
     * <tr><td>Read 0x0</td><td>No CRC corruption pending</td></tr>
     * <tr><td>Read 0x1</td><td>CRC corruption pending</td></tr>
     * </table>
     *
     * Field size: 1 bit
     */
    uint8_t invEcrc;
    /**
     * [rw] SW must write 0
     *
     * Field size: 1 bit
     */
    uint8_t fastLinkMode;
} Pcie_TiConfDiagCtrlReg;

/**
 * \brief Specification all registers
 *
 * This structure allows one or more registers to be read or written
 * through a single call.
 *
 * The user populates one or more pointers to structures.  All structures
 * that are non-NULL are read or written.
 *
 * Once the pointers are populated, use @ref Pcie_readRegs and/or
 * @ref Pcie_writeRegs to perform the actual register accesses
 */
typedef struct pcieRegisters_s {

    /*****************************************************************************************
    * Application Registers
    *****************************************************************************************/
    Pcie_PidReg                      *pid;                     /**< PID */
    Pcie_CmdStatusReg                *cmdStatus;               /**< Command Status*/
    Pcie_CfgTransReg                 *cfgTrans;                /**< Config Transaction*/
    Pcie_IoBaseReg                   *ioBase;                  /**< IO TLP base*/
    Pcie_TlpCfgReg                   *tlpCfg;                  /**< TLP Config*/
    Pcie_RstCmdReg                   *rstCmd;                  /**< Reset Command*/
    Pcie_PtmCfgReg                   *ptmCfg;                  /**< PTM Config Command*/
    Pcie_PmCmdReg                    *pmCmd;                   /**< Power Management Command*/
    Pcie_PmCfgReg                    *pmCfg;                   /**< Power Management Config*/
    Pcie_ActStatusReg                *actStatus;               /**< Activity Status */
    Pcie_ObSizeReg                   *obSize;                  /**< Outbound Translation region size*/
    Pcie_DiagCtrlReg                 *diagCtrl;                /**< Diagnostic Control */
    Pcie_EndianReg                   *endian;                  /**< Endian Register*/
    Pcie_PriorityReg                 *priority;                /**< Transaction Priority Register */
    Pcie_IrqEOIReg                   *irqEOI;                  /**< End of Interrupt Register */
    Pcie_MsiIrqReg                   *msiIrq;                  /**< MSI Interrupt IRQ Register*/
    Pcie_EpIrqSetReg                 *epIrqSet;                /**< Endpoint Interrupt Request Set Register*/
    Pcie_EpIrqClrReg                 *epIrqClr;                /**< Endpoint Interrupt Request clear Register*/
    Pcie_EpIrqStatusReg              *epIrqStatus;             /**< Endpoint Interrupt status Register*/
    Pcie_GenPurposeReg               *genPurpose[4];           /**< General Purpose Registers */
    Pcie_MsiIrqStatusRawReg          *msiIrqStatusRaw[8];      /**< MSI Raw Interrupt Status Register*/
    Pcie_MsiIrqStatusReg             *msiIrqStatus[8];         /**< MSI Interrupt Enabled Status Register*/
    Pcie_MsiIrqEnableSetReg          *msiIrqEnableSet[8];      /**< MSI Interrupt Enable Set Register*/
    Pcie_MsiIrqEnableClrReg          *msiIrqEnableClr[8];      /**< MSI Interrupt Enable Clear Register*/
    Pcie_LegacyIrqStatusRawReg       *legacyIrqStatusRaw[4];   /**< Raw Interrupt Status Register*/
    Pcie_LegacyIrqStatusReg          *legacyIrqStatus[4];      /**< Interrupt Enabled Status Register*/
    Pcie_LegacyIrqEnableSetReg       *legacyIrqEnableSet[4];   /**< Interrupt Enable Set Register*/
    Pcie_LegacyIrqEnableClrReg       *legacyIrqEnableClr[4];   /**< Interrupt Enable Clear Register*/
    Pcie_ErrIrqStatusRawReg          *errIrqStatusRaw;         /**< Raw Interrupt Status Register*/
    Pcie_ErrIrqStatusReg             *errIrqStatus;            /**< Interrupt Enabled Status Register*/
    Pcie_ErrIrqEnableSetReg          *errIrqEnableSet;         /**< Interrupt Enable Set Register*/
    Pcie_ErrIrqEnableClrReg          *errIrqEnableClr;         /**< Interrupt Enable Clear Register*/
    Pcie_PmRstIrqStatusRawReg        *pmRstIrqStatusRaw;       /**< Power Management and Reset Raw Interrupt Status Register*/
    Pcie_PmRstIrqStatusReg           *pmRstIrqStatus;          /**< Power Management and Reset Interrupt Enabled Status Register*/
    Pcie_PmRstIrqEnableSetReg        *pmRstIrqEnableSet;       /**< Power Management and Reset Interrupt Enable Set Register*/
    Pcie_PmRstIrqEnableClrReg        *pmRstIrqEnableClr;       /**< Power Management and Reset Interrupt Enable Clear Register*/
    Pcie_PtmIrqStatusRawReg          *ptmIrqStatusRaw;         /**< Precision Time Measurement Raw Interrupt Status Register*/
    Pcie_PtmIrqStatusReg             *ptmIrqStatus;            /**< Precision Time Measurement Interrupt Enabled Status Register*/
    Pcie_PtmIrqEnableSetReg          *ptmIrqEnableSet;         /**< Precision Time Measurement Interrupt Enable Set Register*/
    Pcie_PtmIrqEnableClrReg          *ptmIrqEnableClr;         /**< Precision Time Measurement Interrupt Enable Clear Register*/
    Pcie_ObOffsetLoReg               *obOffsetLo[8];           /**< Outbound Translation region offset Low*/
    Pcie_ObOffsetHiReg               *obOffsetHi[8];           /**< Outbound Translation region offset High*/
    Pcie_IbBarReg                    *ibBar[4];                /**< Inbound Translation BAR*/
    Pcie_IbStartLoReg                *ibStartLo[4];            /**< Inbound Translation start Low*/
    Pcie_IbStartHiReg                *ibStartHi[4];            /**< Inbound Translation start High*/
    Pcie_IbOffsetReg                 *ibOffset[4];             /**< Inbound Translation offset*/
    Pcie_PcsCfg0Reg                  *pcsCfg0;                 /**< PCS Configuration 0 Register */
    Pcie_PcsCfg1Reg                  *pcsCfg1;                 /**< PCS Configuration 1 Register */
    Pcie_PcsStatusReg                *pcsStatus;               /**< PCS Status Register */
    Pcie_SerdesCfg0Reg               *serdesCfg0;              /**< SERDES config 0 Register*/
    Pcie_SerdesCfg1Reg               *serdesCfg1;              /**< SERDES config 1 Register*/

    /*****************************************************************************************
    * Configuration Registers
    *****************************************************************************************/

    /* Type 0, Type1 Common Registers*/
    Pcie_VndDevIdReg                 *vndDevId;                /**< Vendor and device ID*/
    Pcie_StatusCmdReg                *statusCmd;               /**< Status Command*/
    Pcie_RevIdReg                    *revId;                   /**< Class code and Revision ID*/
    Pcie_BaseAddrReg                 *baseAddr;                /**< Base Address*/

    /* Type 0 Registers*/
    Pcie_BistReg                     *bist;                    /**< Bist Header*/
    Pcie_Type0BarIdx                 *type0BarIdx;             /**< Type 0 (EP) BAR register*/
    Pcie_Type0Bar32bitIdx            *type0Bar32bitIdx;        /**< Type 0 BAR 32bits register*/
    Pcie_Type0Bar32bitIdx            *type0BarMask32bitIdx;    /**< Type 0 BAR mask register*/
    Pcie_CardbusCisPointerReg        *cardbusCisPointer;       /**< cardbus CIS pointer register*/
    Pcie_SubIdReg                    *subId;                   /**< Subsystem ID*/
    Pcie_ExpRomReg                   *expRom;                  /**< Expansion ROM base addr*/
    Pcie_CapPtrReg                   *capPtr;                  /**< Capabilities Pointer*/
    Pcie_IntPinReg                   *intPin;                  /**< Interrupt Pin*/

    /* Type 1 Registers*/
    Pcie_Type1BistHeaderReg          *type1BistHeader;         /**< Bist Header, Latency Timer, Cache Line */
    Pcie_Type1BarIdx                 *type1BarIdx;             /**< Type 1 (RC) BAR register*/
    Pcie_Type1Bar32bitIdx            *type1Bar32bitIdx;        /**< Type 1 BAR 32bits register*/
    Pcie_Type1Bar32bitIdx            *type1BarMask32bitIdx;    /**< Type 1 bar mask register*/
    Pcie_Type1BusNumReg              *type1BusNum;             /**< Latency Timer and Bus Number */
    Pcie_Type1SecStatReg             *type1SecStat;            /**< Secondary Status and IO space */
    Pcie_Type1MemspaceReg            *type1Memspace;           /**< Memory Limit*/
    Pcie_PrefMemReg                  *prefMem;                 /**< Prefetch Memory Limit and Base*/
    Pcie_PrefBaseUpperReg            *prefBaseUpper;           /**< Prefetch Memory Base Upper*/
    Pcie_PrefLimitUpperReg           *prefLimitUpper;          /**< Prefetch Memory Limit Upper*/
    Pcie_Type1IOSpaceReg             *type1IOSpace;            /**< IO Base and Limit Upper 16 bits */
    Pcie_Type1CapPtrReg              *type1CapPtr;             /**< Capabilities pointer */
    Pcie_Type1ExpnsnRomReg           *type1ExpnsnRom;          /**< Expansion ROM base addr */
    Pcie_Type1BridgeIntReg           *type1BridgeInt;          /**< Bridge Control and Interrupt Pointer */

    /* Power Management Capabilities Registers */
    Pcie_PMCapReg                    *pmCap;                   /**< Power Management Capabilities */
    Pcie_PMCapCtlStatReg             *pmCapCtlStat;            /**< Power Management Control and Status */

    /* MSI Capabilities Registers */
    Pcie_MsiCapReg                   *msiCap;                  /**< MSI Capabilities */
    Pcie_MsiLo32Reg                  *msiLo32;                 /**< MSI Lower 32 bits */
    Pcie_MsiUp32Reg                  *msiUp32;                 /**< MSI Upper 32 bits */
    Pcie_MsiDataReg                  *msiData;                 /**< MSI Data */
    Pcie_MsiCapOff10HReg             *msiCapOff10H;            /**< MSI Cap Off 10H */
    Pcie_MsiCapOff14HReg             *msiCapOff14H;            /**< MSI Cap Off 14H */

    /* MSIx Capabilities Registers */
    Pcie_MsixCapReg                  *msixCap;                 /**< MSIx Capabilities */
    Pcie_MsixTblOffset               *msixTblOffset;            /**< MSIx Table offset */

    /* Capabilities Registers */
    Pcie_PciesCapReg                 *pciesCap;                /**< PCI Express Capabilities Register*/
    Pcie_DeviceCapReg                *deviceCap;               /**< Device Capabilities Register*/
    Pcie_DevStatCtrlReg              *devStatCtrl;             /**< Device Status and Control*/
    Pcie_LinkCapReg                  *linkCap;                 /**< Link Capabilities Register*/
    Pcie_LinkStatCtrlReg             *linkStatCtrl;            /**< Link Status and Control Register*/
    Pcie_SlotCapReg                  *slotCap;                 /**< Slot Capabilities Register */
    Pcie_SlotStatCtrlReg             *slotStatCtrl;            /**< Slot Status and Control Register */
    Pcie_RootCtrlCapReg              *rootCtrlCap;             /**< Root Control and Capabilities Register */
    Pcie_RootStatusReg               *rootStatus;              /**< Root Status and Control Register */
    Pcie_DevCap2Reg                  *devCap2;                 /**< Device Capabilities 2 Register*/
    Pcie_DevStatCtrl2Reg             *devStatCtrl2;            /**< Device Status and Control 2 Register*/
    Pcie_LnkCap2Reg                  *linkCap2;                /**< Link Capabilities 2 Register*/
    Pcie_LinkCtrl2Reg                *linkCtrl2;               /**< Link Control 2 Register*/

    /* Capabilities Extended Registers */
    Pcie_ExtCapReg                   *extCap;                  /**< Extended Capabilties Header */
    Pcie_UncErrReg                   *uncErr;                  /**< Uncorrectable Error Status */
    Pcie_UncErrMaskReg               *uncErrMask;              /**< Uncorrectable Error Mask */
    Pcie_UncErrSvrtyReg              *uncErrSvrty;             /**< Uncorrectable Error Severity */
    Pcie_CorErrReg                   *corErr;                  /**< Correctable Error Status */
    Pcie_CorErrMaskReg               *corErrMask;              /**< Correctable Error Mask */
    Pcie_AccrReg                     *accr;                    /**< Advanced Capabilities and Control*/
    Pcie_HdrLogReg                   *hdrLog[4];               /**< Header Log Registers */
    Pcie_RootErrCmdReg               *rootErrCmd;              /**< Root Error Command */
    Pcie_RootErrStReg                *rootErrSt;               /**< Root Error Status */
    Pcie_ErrSrcIDReg                 *errSrcID;                /**< Error Source Identification */

    /* Port Logic Registers */
    Pcie_PlAckTimerReg               *plAckTimer;              /**< Ack Latency Time and Replay Timer */
    Pcie_PlOMsgReg                   *plOMsg;                  /**< Other Message */
    Pcie_PlForceLinkReg              *plForceLink;             /**< Port Force Link */
    Pcie_AckFreqReg                  *ackFreq;                 /**< Ack Frequency */
    Pcie_LnkCtrlReg                  *lnkCtrl;                 /**< Port Link Control*/
    Pcie_LaneSkewReg                 *laneSkew;                /**< Lane Skew */
    Pcie_SymNumReg                   *symNum;                  /**< Symbol Number */
    Pcie_SymTimerFltMaskReg          *symTimerFltMask;         /**< Symbol Timer and Filter Mask */
    Pcie_FltMask2Reg                 *fltMask2;                /**< Filter Mask 2 */
    Pcie_Debug0Reg                   *debug0;                  /**< Debug 0*/
    Pcie_Debug1Reg                   *debug1;                  /**< Debug 1 Register*/
    Pcie_Gen2Reg                     *gen2;                    /**< Gen2 */

    /* Rev 1 PLCONF */
    Pcie_PlconfObnpSubreqCtrlReg      *plconfObnpSubreqCtrl;   /**< PCIECTRL_PL_OBNP_SUBREQ_CTRL*/
    Pcie_PlconfTrPStsRReg             *plconfTrPStsR;          /**< PCIECTRL_PL_TR_P_STS_R*/
    Pcie_PlconfTrNpStsRReg            *plconfTrNpStsR;         /**< PCIECTRL_PL_TR_NP_STS_R*/
    Pcie_PlconfTrCStsRReg             *plconfTrCStsR;          /**< PCIECTRL_PL_TR_C_STS_R*/
    Pcie_PlconfQStsRReg               *plconfQStsR;            /**< PCIECTRL_PL_Q_STS_R*/
    Pcie_PlconfVcTrAR1Reg             *plconfVcTrAR1;          /**< PCIECTRL_PL_VC_TR_A_R1*/
    Pcie_PlconfVcTrAR2Reg             *plconfVcTrAR2;          /**< PCIECTRL_PL_VC_TR_A_R2*/
    Pcie_PlconfVc0PrQCReg             *plconfVc0PrQC;          /**< PCIECTRL_PL_VC0_PR_Q_C*/
    Pcie_PlconfVc0NprQCReg            *plconfVc0NprQC;         /**< PCIECTRL_PL_VC0_NPR_Q_C*/
    Pcie_PlconfVc0CrQCReg             *plconfVc0CrQC;          /**< PCIECTRL_PL_VC0_CR_Q_C*/
    Pcie_PlconfVcPrQCReg              *plconfVcPrQC[3];        /**< for VC1..VC3 */
    Pcie_PlconfVcNprQCReg             *plconfVcNprQC[3];       /**< for VC1..VC3 */
    Pcie_PlconfVcCrQCReg              *plconfVcCrQC[3];        /**< for VC1..VC3 */

    Pcie_PlconfPhyStsRReg             *plconfPhyStsR;          /**< PCIECTRL_PL_PHY_STS_R*/
    Pcie_PlconfPhyCtrlRReg            *plconfPhyCtrlR;         /**< PCIECTRL_PL_PHY_CTRL_R*/
    Pcie_PlconfMsiCtrlAddressReg      *plconfMsiCtrlAddress;   /**< PCIECTRL_PL_MSI_CTRL_ADDRESS*/
    Pcie_PlconfMsiCtrlUpperAddressReg *plconfMsiCtrlUpperAddress; /**< PCIECTRL_PL_MSI_CTRL_UPPER_ADDRESS*/
    Pcie_PlconfMsiCtrlIntEnableReg    *plconfMsiCtrlIntEnable[8]; /**< PCIECTRL_PL_MSI_CTRL_INT_ENABLE_N*/
    Pcie_PlconfMsiCtrlIntMaskReg      *plconfMsiCtrlIntMask[8];   /**< PCIECTRL_PL_MSI_CTRL_INT_MASK_N*/
    Pcie_PlconfMsiCtrlIntStatusReg    *plconfMsiCtrlIntStatus[8]; /**< PCIECTRL_PL_MSI_CTRL_INT_STATUS_N*/
    Pcie_PlconfMsiCtrlGpioReg         *plconfMsiCtrlGpio;      /**< PCIECTRL_PL_MSI_CTRL_GPIO*/
    Pcie_PlconfPipeLoopbackReg        *plconfPipeLoopback;     /**< PCIECTRL_PL_PIPE_LOOPBACK*/
    Pcie_PlconfDbiRoWrEnReg           *plconfDbiRoWrEn;        /**< PCIECTRL_PL_DBI_RO_WR_EN*/
    Pcie_PlconfAxiSlvErrRespReg       *plconfAxiSlvErrResp;    /**< PCIECTRL_PL_AXI_SLV_ERR_RESP*/
    Pcie_PlconfAxiSlvTimeoutReg       *plconfAxiSlvTimeout;    /**< PCIECTRL_PL_AXI_SLV_TIMEOUT*/
    Pcie_PlconfIatuIndexReg           *plconfIatuIndex;        /**< PCIECTRL_PL_IATU_INDEX*/
    Pcie_PlconfIatuRegCtrl1Reg        *plconfIatuRegCtrl1;     /**< PCIECTRL_PL_IATU_REG_CTRL_1*/
    Pcie_PlconfIatuRegCtrl2Reg        *plconfIatuRegCtrl2;     /**< PCIECTRL_PL_IATU_REG_CTRL_2*/
    Pcie_PlconfIatuRegLowerBaseReg    *plconfIatuRegLowerBase; /**< PCIECTRL_PL_IATU_REG_LOWER_BASE*/
    Pcie_PlconfIatuRegUpperBaseReg    *plconfIatuRegUpperBase; /**< PCIECTRL_PL_IATU_REG_UPPER_BASE*/
    Pcie_PlconfIatuRegLimitReg        *plconfIatuRegLimit;     /**< PCIECTRL_PL_IATU_REG_LIMIT*/
    Pcie_PlconfIatuRegLowerTargetReg  *plconfIatuRegLowerTarget; /**< PCIECTRL_PL_IATU_REG_LOWER_TARGET*/
    Pcie_PlconfIatuRegUpperTargetReg  *plconfIatuRegUpperTarget; /**< PCIECTRL_PL_IATU_REG_UPPER_TARGET*/
    Pcie_PlconfIatuRegCtrl3Reg        *plconfIatuRegCtrl3;     /**< PCIECTRL_PL_IATU_REG_CTRL_3*/

    /*****************************************************************************************
    * HW Rev 1 configuration registers
    *****************************************************************************************/
    /* TI Configuration registers (PCIECTRL_TI_CONF* in TRM) */
    Pcie_TiConfRevisionReg           *tiConfRevision;          /**< PCIECTRL_TI_CONF_REVISION*/
    Pcie_TiConfSysConfigReg          *tiConfSysConfig;         /**< PCIECTRL_TI_CONF_SYSCONFIG*/
    Pcie_TiConfIrqEoiReg             *tiConfIrqEoi;            /**< PCIECTRL_TI_CONF_IRQ_EOI*/
    Pcie_TiConfIrqStatusRawMainReg   *tiConfIrqStatusRawMain;  /**< PCIECTRL_TI_CONF_IRQSTATUS_RAW_MAIN*/
    Pcie_TiConfIrqStatusMainReg      *tiConfIrqStatusMain;     /**< PCIECTRL_TI_CONF_IRQSTATUS_MAIN*/
    Pcie_TiConfIrqEnableSetMainReg   *tiConfIrqEnableSetMain;  /**< PCIECTRL_TI_CONF_IRQENABLE_SET_MAIN*/
    Pcie_TiConfIrqEnableClrMainReg   *tiConfIrqEnableClrMain;  /**< PCIECTRL_TI_CONF_IRQENABLE_CLR_MAIN*/
    Pcie_TiConfIrqStatusRawMsiReg    *tiConfIrqStatusRawMsi;   /**< PCIECTRL_TI_CONF_IRQSTATUS_RAW_MSI*/
    Pcie_TiConfIrqStatusMsiReg       *tiConfIrqStatusMsi;      /**< PCIECTRL_TI_CONF_IRQSTATUS_MSI*/
    Pcie_TiConfIrqEnableSetMsiReg    *tiConfIrqEnableSetMsi;   /**< PCIECTRL_TI_CONF_IRQENABLE_SET_MSI*/
    Pcie_TiConfIrqEnableClrMsiReg    *tiConfIrqEnableClrMsi;   /**< PCIECTRL_TI_CONF_IRQENABLE_CLR_MSI*/
    Pcie_TiConfDeviceTypeReg         *tiConfDeviceType;        /**< PCIECTRL_TI_CONF_DEVICE_TYPE*/
    Pcie_TiConfDeviceCmdReg          *tiConfDeviceCmd;         /**< PCIECTRL_TI_CONF_DEVICE_CMD*/
    Pcie_TiConfPmCtrlReg             *tiConfPmCtrl;            /**< PCIECTRL_TI_CONF_PM_CTRL*/
    Pcie_TiConfPhyCsReg              *tiConfPhyCs;             /**< PCIECTRL_TI_CONF_PHY_CS*/
    Pcie_TiConfIntxAssertReg         *tiConfIntxAssert;        /**< PCIECTRL_TI_CONF_INTX_ASSERT*/
    Pcie_TiConfIntxDeassertReg       *tiConfIntxDeassert;      /**< PCIECTRL_TI_CONF_INTX_DEASSERT*/
    Pcie_TiConfMsiXmtReg             *tiConfMsiXmt;            /**< PCIECTRL_TI_CONF_MSI_XMT*/
    Pcie_TiConfDebugCfgReg           *tiConfDebugCfg;          /**< PCIECTRL_TI_CONF_DEBUG_CFG*/
    Pcie_TiConfDebugDataReg          *tiConfDebugData;         /**< PCIECTRL_TI_CONF_DEBUG_DATA*/
    Pcie_TiConfDiagCtrlReg           *tiConfDiagCtrl;          /**< PCIECTRL_TI_CONF_DIAG_CTRL*/
} Pcie_Registers;


#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PCIE_V0_REG_H_ */
