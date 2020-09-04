/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_mss_debugss.h
*/
#ifndef CSLR_MSS_DEBUGSS_H_
#define CSLR_MSS_DEBUGSS_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t ONEMCU_APB_BASE;
    volatile uint8_t  Resv_4092[4088];
    volatile uint32_t ONEMCU_APB_BASE_END;
    volatile uint32_t ONEMCU_CTI_CONTROL;
    volatile uint8_t  Resv_4112[12];
    volatile uint32_t ONEMCU_CTI_INTACK;
    volatile uint32_t ONEMCU_CTI_APPSET;
    volatile uint32_t ONEMCU_CTI_APPCLEAR;
    volatile uint32_t ONEMCU_CTI_APPPULSE;
    volatile uint32_t ONEMCU_CTI_INEN0;
    volatile uint32_t ONEMCU_CTI_INEN1;
    volatile uint32_t ONEMCU_CTI_INEN2;
    volatile uint32_t ONEMCU_CTI_INEN3;
    volatile uint32_t ONEMCU_CTI_INEN4;
    volatile uint32_t ONEMCU_CTI_INEN5;
    volatile uint32_t ONEMCU_CTI_INEN6;
    volatile uint32_t ONEMCU_CTI_INEN7;
    volatile uint8_t  Resv_4256[96];
    volatile uint32_t ONEMCU_CTI_OUTEN0;
    volatile uint32_t ONEMCU_CTI_OUTEN1;
    volatile uint32_t ONEMCU_CTI_OUTEN2;
    volatile uint32_t ONEMCU_CTI_OUTEN3;
    volatile uint32_t ONEMCU_CTI_OUTEN4;
    volatile uint32_t ONEMCU_CTI_OUTEN5;
    volatile uint32_t ONEMCU_CTI_OUTEN6;
    volatile uint32_t ONEMCU_CTI_OUTEN7;
    volatile uint8_t  Resv_4400[112];
    volatile uint32_t ONEMCU_CTI_TRIGINSTATUS;
    volatile uint32_t ONEMCU_CTI_TRIGOUTSTATUS;
    volatile uint32_t ONEMCU_CTI_CHINSTATUS;
    volatile uint32_t ONEMCU_CTI_CHOUTSTATUS;
    volatile uint32_t ONEMCU_CTI_GATE;
    volatile uint32_t ONEMCU_CTI_ASICCTL;
    volatile uint8_t  Resv_7900[3476];
    volatile uint32_t ONEMCU_CTI_ITCHINACK;
    volatile uint32_t ONEMCU_CTI_ITTRIGINACK;
    volatile uint32_t ONEMCU_CTI_ITCHOUT;
    volatile uint32_t ONEMCU_CTI_ITTRIGOUT;
    volatile uint32_t ONEMCU_CTI_ITCHOUTACK;
    volatile uint32_t ONEMCU_CTI_ITTRIGOUTACK;
    volatile uint32_t ONEMCU_CTI_ITCHIN;
    volatile uint32_t ONEMCU_CTI_ITTRIGIN;
    volatile uint8_t  Resv_7936[4];
    volatile uint32_t ONEMCU_CTI_ITCTRL;
    volatile uint8_t  Resv_8096[156];
    volatile uint32_t ONEMCU_CTI_CLAIM_TAG_SET;
    volatile uint32_t ONEMCU_CTI_CLAIM_TAG_CLEAR;
    volatile uint8_t  Resv_8112[8];
    volatile uint32_t ONEMCU_CTI_LOCK_ACCESS_REGISTER;
    volatile uint32_t ONEMCU_CTI_LOCK_STATUS_REGISTER;
    volatile uint32_t ONEMCU_CTI_AUTHENTICATION_STATUS;
    volatile uint8_t  Resv_8136[12];
    volatile uint32_t ONEMCU_CTI_DEVICE_ID;
    volatile uint32_t ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER;
    volatile uint32_t ONEMCU_CTI_PERIPHERALID4;
    volatile uint32_t ONEMCU_CTI_PERIPHERALID5;
    volatile uint32_t ONEMCU_CTI_PERIPHERALID6;
    volatile uint32_t ONEMCU_CTI_PERIPHERALID7;
    volatile uint32_t ONEMCU_CTI_PERIPHERALID0;
    volatile uint32_t ONEMCU_CTI_PERIPHERALID1;
    volatile uint32_t ONEMCU_CTI_PERIPHERALID2;
    volatile uint32_t ONEMCU_CTI_PERIPHERALID3;
    volatile uint32_t ONEMCU_CTI_COMPONENT_ID0;
    volatile uint32_t ONEMCU_CTI_COMPONENT_ID1;
    volatile uint32_t ONEMCU_CTI_COMPONENT_ID2;
    volatile uint32_t ONEMCU_CTI_COMPONENT_ID3;
    volatile uint32_t ONEMCU_TPIU_SPORTSZ;
    volatile uint32_t ONEMCU_TPIU_CPORTSZ;
    volatile uint8_t  Resv_8448[248];
    volatile uint32_t ONEMCU_TPIU_STRIGM;
    volatile uint32_t ONEMCU_TPIU_TRIGCNT;
    volatile uint32_t ONEMCU_TPIU_TRIGMUL;
    volatile uint8_t  Resv_8704[244];
    volatile uint32_t ONEMCU_TPIU_STSTPTRN;
    volatile uint32_t ONEMCU_TPIU_CTSTPTRN;
    volatile uint32_t ONEMCU_TPIU_TPRCNTR;
    volatile uint8_t  Resv_8960[244];
    volatile uint32_t ONEMCU_TPIU_FFSTS;
    volatile uint32_t ONEMCU_TPIU_FFCTRL;
    volatile uint32_t ONEMCU_TPIU_FSCNTR;
    volatile uint8_t  Resv_9216[244];
    volatile uint32_t ONEMCU_TPIU_EXCTLIN;
    volatile uint32_t ONEMCU_TPIU_EXCTLOUT;
    volatile uint8_t  Resv_12004[2780];
    volatile uint32_t ONEMCU_TPIU_ITTRFLINACK;
    volatile uint32_t ONEMCU_TPIU_ITTRFLIN;
    volatile uint32_t ONEMCU_TPIU_ITATBDATA0;
    volatile uint32_t ONEMCU_TPIU_ITATBCTR2;
    volatile uint32_t ONEMCU_TPIU_ITATBCTR1;
    volatile uint32_t ONEMCU_TPIU_ITATBCTR0;
    volatile uint8_t  Resv_12032[4];
    volatile uint32_t ONEMCU_TPIU_ITCTRL;
    volatile uint8_t  Resv_12192[156];
    volatile uint32_t ONEMCU_TPIU_CLAIMSET;
    volatile uint32_t ONEMCU_TPIU_CLAIMCLR;
    volatile uint8_t  Resv_12208[8];
    volatile uint32_t ONEMCU_TPIU_LAR;
    volatile uint32_t ONEMCU_TPIU_LSR;
    volatile uint32_t ONEMCU_TPIU_AUTHSTATUS;
    volatile uint8_t  Resv_12232[12];
    volatile uint32_t ONEMCU_TPIU_DEVID;
    volatile uint32_t ONEMCU_TPIU_DEVTYPE;
    volatile uint32_t ONEMCU_TPIU_PIDR4;
    volatile uint32_t ONEMCU_TPIU_PIDR5;
    volatile uint32_t ONEMCU_TPIU_PIDR6;
    volatile uint32_t ONEMCU_TPIU_PIDR7;
    volatile uint32_t ONEMCU_TPIU_PIDR0;
    volatile uint32_t ONEMCU_TPIU_PIDR1;
    volatile uint32_t ONEMCU_TPIU_PIDR2;
    volatile uint32_t ONEMCU_TPIU_PIDR3;
    volatile uint32_t ONEMCU_TPIU_CIDR0;
    volatile uint32_t ONEMCU_TPIU_CIDR1;
    volatile uint32_t ONEMCU_TPIU_CIDR2;
    volatile uint32_t ONEMCU_TPIU_CIDR3;
    volatile uint8_t  Resv_65536[53248];
    volatile uint32_t APB_EXT_PORT0_ROM;
    volatile uint8_t  Resv_69632[4092];
    volatile uint32_t ATB_REPLICATOR_IDFILTER0;
    volatile uint32_t ATB_REPLICATOR_IDFILTER1;
    volatile uint8_t  Resv_73464[3824];
    volatile uint32_t ATB_REPLICATOR_ITATBCTR1;
    volatile uint32_t ATB_REPLICATOR_ITATBCTR0;
    volatile uint32_t ATB_REPLICATOR_ITCTRL;
    volatile uint8_t  Resv_73632[156];
    volatile uint32_t ATB_REPLICATOR_CLAIMSET;
    volatile uint32_t ATB_REPLICATOR_CLAIMCLR;
    volatile uint8_t  Resv_73648[8];
    volatile uint32_t ATB_REPLICATOR_LAR;
    volatile uint32_t ATB_REPLICATOR_LSR;
    volatile uint32_t ATB_REPLICATOR_AUTHSTATUS;
    volatile uint8_t  Resv_73672[12];
    volatile uint32_t ATB_REPLICATOR_DEVID;
    volatile uint32_t ATB_REPLICATOR_DEVTYPE;
    volatile uint32_t ATB_REPLICATOR_PIDR4;
    volatile uint32_t ATB_REPLICATOR_RESERVED1;
    volatile uint32_t ATB_REPLICATOR_RESERVED2;
    volatile uint32_t ATB_REPLICATOR_RESERVED3;
    volatile uint32_t ATB_REPLICATOR_PIDR0;
    volatile uint32_t ATB_REPLICATOR_PIDR1;
    volatile uint32_t ATB_REPLICATOR_PIDR2;
    volatile uint32_t ATB_REPLICATOR_PIDR3;
    volatile uint32_t ATB_REPLICATOR_CIDR0;
    volatile uint32_t ATB_REPLICATOR_CIDR1;
    volatile uint32_t ATB_REPLICATOR_CIDR2;
    volatile uint32_t ATB_REPLICATOR_CIDR3;
    volatile uint8_t  Resv_73732[4];
    volatile uint32_t ETB_RDP;
    volatile uint8_t  Resv_73740[4];
    volatile uint32_t ETB_STS;
    volatile uint32_t ETB_RRD;
    volatile uint32_t ETB_RRP;
    volatile uint32_t ETB_RWP;
    volatile uint32_t ETB_TRG;
    volatile uint32_t ETB_CTL;
    volatile uint32_t ETB_RWD;
    volatile uint8_t  Resv_74496[728];
    volatile uint32_t ETB_FFSR;
    volatile uint32_t ETB_FFCR;
    volatile uint8_t  Resv_77536[3032];
    volatile uint32_t ETB_ITMISCOP0;
    volatile uint32_t ETB_ITTRFLINACK;
    volatile uint32_t ETB_ITTRFLIN;
    volatile uint32_t ETB_ITATBDATA0;
    volatile uint32_t ETB_ITATBCTR2;
    volatile uint32_t ETB_ITATBCTR1;
    volatile uint32_t ETB_ITATBCTR0;
    volatile uint8_t  Resv_77568[4];
    volatile uint32_t ETB_ITCTRL;
    volatile uint8_t  Resv_77728[156];
    volatile uint32_t ETB_CLAIMSET;
    volatile uint32_t ETB_CLAIMCLR;
    volatile uint8_t  Resv_77744[8];
    volatile uint32_t ETB_LAR;
    volatile uint32_t ETB_LSR;
    volatile uint32_t ETB_AUTHSTATUS;
    volatile uint8_t  Resv_77768[12];
    volatile uint32_t ETB_DEVID;
    volatile uint32_t ETB_DEVTYPE;
    volatile uint32_t ETB_PIDR4;
    volatile uint32_t ETB_RESERVED4;
    volatile uint32_t ETB_RESERVED5;
    volatile uint32_t ETB_RESERVED6;
    volatile uint32_t ETB_PIDR0;
    volatile uint32_t ETB_PIDR1;
    volatile uint32_t ETB_PIDR2;
    volatile uint32_t ETB_PIDR3;
    volatile uint32_t ETB_CIDR0;
    volatile uint32_t ETB_CIDR1;
    volatile uint32_t ETB_CIDR2;
    volatile uint32_t ETB_CIDR3;
    volatile uint8_t  Resv_80900[3076];
    volatile uint32_t STMDMASTARTR;
    volatile uint32_t STMDMASTOPR;
    volatile uint32_t STMDMASTATR;
    volatile uint8_t STMDMACTLR;
    volatile uint8_t  Resv_81148[235];
    volatile uint32_t STMDMAIDR;
    volatile uint32_t STMHEER;
    volatile uint8_t  Resv_81184[28];
    volatile uint32_t STMHETER;
    volatile uint8_t  Resv_81248[60];
    volatile uint32_t STMHEBSR;
    volatile uint32_t STMHEMCR;
    volatile uint8_t STMHEEXTMUXR;
    volatile uint8_t  Resv_81396[139];
    volatile uint16_t STMHEMASTR;
    volatile uint8_t  Resv_81400[2];
    volatile uint32_t STMHEFEAT1R;
    volatile uint16_t STMHEIDR;
    volatile uint8_t  Resv_81408[2];
    volatile uint32_t STMSPER;
    volatile uint8_t  Resv_81440[28];
    volatile uint32_t STMSPTER;
    volatile uint8_t  Resv_81504[60];
    volatile uint32_t STMSPSCR;
    volatile uint32_t STMSPMSCR;
    volatile uint32_t STMSPOVERRIDER;
    volatile uint32_t STMSPMOVERRIDER;
    volatile uint32_t STMSPTRIGCSR;
    volatile uint8_t  Resv_81536[12];
    volatile uint32_t STMTCSR;
    volatile uint32_t STMTSSTIMR;
    volatile uint8_t  Resv_81548[4];
    volatile uint32_t STMTSFREQR;
    volatile uint32_t STMSYNCR;
    volatile uint8_t STMAUXCR;
    volatile uint8_t  Resv_81568[11];
    volatile uint32_t STMFEAT1R;
    volatile uint32_t STMFEAT2R;
    volatile uint8_t STMFEAT3R;
    volatile uint8_t  Resv_81640[63];
    volatile uint8_t STMITTRIGGER;
    volatile uint8_t  Resv_81644[3];
    volatile uint16_t STMITATBDATA0;
    volatile uint8_t  Resv_81648[2];
    volatile uint8_t STMITATBCTR2;
    volatile uint8_t  Resv_81652[3];
    volatile uint8_t STMITATBID;
    volatile uint8_t  Resv_81656[3];
    volatile uint16_t STMITATBCTR0;
    volatile uint8_t  Resv_81664[6];
    volatile uint8_t STMITCTRL;
    volatile uint8_t  Resv_81824[159];
    volatile uint32_t STMCLAIMSET;
    volatile uint32_t STMCLAIMCLR;
    volatile uint8_t  Resv_81840[8];
    volatile uint32_t STMLAR;
    volatile uint8_t STMLSR;
    volatile uint8_t  Resv_81848[3];
    volatile uint8_t STMAUTHSTATUS;
    volatile uint8_t  Resv_81852[3];
    volatile uint32_t STMDEVARCH;
    volatile uint8_t  Resv_81864[8];
    volatile uint32_t STMDEVID;
    volatile uint8_t STMDEVTYPE;
    volatile uint8_t  Resv_81872[3];
    volatile uint8_t STMPIDR4;
    volatile uint8_t  Resv_81888[15];
    volatile uint8_t STMPIDR0;
    volatile uint8_t  Resv_81892[3];
    volatile uint8_t STMPIDR1;
    volatile uint8_t  Resv_81896[3];
    volatile uint8_t STMPIDR2;
    volatile uint8_t  Resv_81900[3];
    volatile uint8_t STMPIDR3;
    volatile uint8_t  Resv_81904[3];
    volatile uint8_t STMCIDR0;
    volatile uint8_t  Resv_81908[3];
    volatile uint8_t STMCIDR1;
    volatile uint8_t  Resv_81912[3];
    volatile uint8_t STMCIDR2;
    volatile uint8_t  Resv_81916[3];
    volatile uint8_t STMCIDR3;
    volatile uint8_t  Resv_81920[3];
    volatile uint32_t STM_CTI_CONTROL;
    volatile uint8_t  Resv_81936[12];
    volatile uint32_t STM_CTI_INTACK;
    volatile uint32_t STM_CTI_APPSET;
    volatile uint32_t STM_CTI_APPCLEAR;
    volatile uint32_t STM_CTI_APPPULSE;
    volatile uint32_t STM_CTI_INEN0;
    volatile uint32_t STM_CTI_INEN1;
    volatile uint32_t STM_CTI_INEN2;
    volatile uint32_t STM_CTI_INEN3;
    volatile uint32_t STM_CTI_INEN4;
    volatile uint32_t STM_CTI_INEN5;
    volatile uint32_t STM_CTI_INEN6;
    volatile uint32_t STM_CTI_INEN7;
    volatile uint8_t  Resv_82080[96];
    volatile uint32_t STM_CTI_OUTEN0;
    volatile uint32_t STM_CTI_OUTEN1;
    volatile uint32_t STM_CTI_OUTEN2;
    volatile uint32_t STM_CTI_OUTEN3;
    volatile uint32_t STM_CTI_OUTEN4;
    volatile uint32_t STM_CTI_OUTEN5;
    volatile uint32_t STM_CTI_OUTEN6;
    volatile uint32_t STM_CTI_OUTEN7;
    volatile uint8_t  Resv_82224[112];
    volatile uint32_t STM_CTI_TRIGINSTATUS;
    volatile uint32_t STM_CTI_TRIGOUTSTATUS;
    volatile uint32_t STM_CTI_CHINSTATUS;
    volatile uint32_t STM_CTI_CHOUTSTATUS;
    volatile uint32_t STM_CTI_GATE;
    volatile uint32_t STM_CTI_ASICCTL;
    volatile uint8_t  Resv_85724[3476];
    volatile uint32_t STM_CTI_ITCHINACK;
    volatile uint32_t STM_CTI_ITTRIGINACK;
    volatile uint32_t STM_CTI_ITCHOUT;
    volatile uint32_t STM_CTI_ITTRIGOUT;
    volatile uint32_t STM_CTI_ITCHOUTACK;
    volatile uint32_t STM_CTI_ITTRIGOUTACK;
    volatile uint32_t STM_CTI_ITCHIN;
    volatile uint32_t STM_CTI_ITTRIGIN;
    volatile uint8_t  Resv_85760[4];
    volatile uint32_t STM_CTI_ITCTRL;
    volatile uint8_t  Resv_85920[156];
    volatile uint32_t STM_CTI_CLAIM_TAG_SET;
    volatile uint32_t STM_CTI_CLAIM_TAG_CLEAR;
    volatile uint8_t  Resv_85936[8];
    volatile uint32_t STM_CTI_LOCK_ACCESS_REGISTER;
    volatile uint32_t STM_CTI_LOCK_STATUS_REGISTER;
    volatile uint32_t STM_CTI_AUTHENTICATION_STATUS;
    volatile uint8_t  Resv_85960[12];
    volatile uint32_t STM_CTI_DEVICE_ID;
    volatile uint32_t STM_CTI_DEVICE_TYPE_IDENTIFIER;
    volatile uint32_t STM_CTI_PERIPHERALID4;
    volatile uint32_t STM_CTI_PERIPHERALID5;
    volatile uint32_t STM_CTI_PERIPHERALID6;
    volatile uint32_t STM_CTI_PERIPHERALID7;
    volatile uint32_t STM_CTI_PERIPHERALID0;
    volatile uint32_t STM_CTI_PERIPHERALID1;
    volatile uint32_t STM_CTI_PERIPHERALID2;
    volatile uint32_t STM_CTI_PERIPHERALID3;
    volatile uint32_t STM_CTI_COMPONENT_ID0;
    volatile uint32_t STM_CTI_COMPONENT_ID1;
    volatile uint32_t STM_CTI_COMPONENT_ID2;
    volatile uint32_t STM_CTI_COMPONENT_ID3;
    volatile uint32_t HSM_CM4_CTI_CONTROL;
    volatile uint8_t  Resv_86032[12];
    volatile uint32_t HSM_CM4_CTI_INTACK;
    volatile uint32_t HSM_CM4_CTI_APPSET;
    volatile uint32_t HSM_CM4_CTI_APPCLEAR;
    volatile uint32_t HSM_CM4_CTI_APPPULSE;
    volatile uint32_t HSM_CM4_CTI_INEN0;
    volatile uint32_t HSM_CM4_CTI_INEN1;
    volatile uint32_t HSM_CM4_CTI_INEN2;
    volatile uint32_t HSM_CM4_CTI_INEN3;
    volatile uint32_t HSM_CM4_CTI_INEN4;
    volatile uint32_t HSM_CM4_CTI_INEN5;
    volatile uint32_t HSM_CM4_CTI_INEN6;
    volatile uint32_t HSM_CM4_CTI_INEN7;
    volatile uint8_t  Resv_86176[96];
    volatile uint32_t HSM_CM4_CTI_OUTEN0;
    volatile uint32_t HSM_CM4_CTI_OUTEN1;
    volatile uint32_t HSM_CM4_CTI_OUTEN2;
    volatile uint32_t HSM_CM4_CTI_OUTEN3;
    volatile uint32_t HSM_CM4_CTI_OUTEN4;
    volatile uint32_t HSM_CM4_CTI_OUTEN5;
    volatile uint32_t HSM_CM4_CTI_OUTEN6;
    volatile uint32_t HSM_CM4_CTI_OUTEN7;
    volatile uint8_t  Resv_86320[112];
    volatile uint32_t HSM_CM4_CTI_TRIGINSTATUS;
    volatile uint32_t HSM_CM4_CTI_TRIGOUTSTATUS;
    volatile uint32_t HSM_CM4_CTI_CHINSTATUS;
    volatile uint32_t HSM_CM4_CTI_CHOUTSTATUS;
    volatile uint32_t HSM_CM4_CTI_GATE;
    volatile uint32_t HSM_CM4_CTI_ASICCTL;
    volatile uint8_t  Resv_89820[3476];
    volatile uint32_t HSM_CM4_CTI_ITCHINACK;
    volatile uint32_t HSM_CM4_CTI_ITTRIGINACK;
    volatile uint32_t HSM_CM4_CTI_ITCHOUT;
    volatile uint32_t HSM_CM4_CTI_ITTRIGOUT;
    volatile uint32_t HSM_CM4_CTI_ITCHOUTACK;
    volatile uint32_t HSM_CM4_CTI_ITTRIGOUTACK;
    volatile uint32_t HSM_CM4_CTI_ITCHIN;
    volatile uint32_t HSM_CM4_CTI_ITTRIGIN;
    volatile uint8_t  Resv_89856[4];
    volatile uint32_t HSM_CM4_CTI_ITCTRL;
    volatile uint8_t  Resv_90016[156];
    volatile uint32_t HSM_CM4_CTI_CLAIM_TAG_SET;
    volatile uint32_t HSM_CM4_CTI_CLAIM_TAG_CLEAR;
    volatile uint8_t  Resv_90032[8];
    volatile uint32_t HSM_CM4_CTI_LOCK_ACCESS_REGISTER;
    volatile uint32_t HSM_CM4_CTI_LOCK_STATUS_REGISTER;
    volatile uint32_t HSM_CM4_CTI_AUTHENTICATION_STATUS;
    volatile uint8_t  Resv_90056[12];
    volatile uint32_t HSM_CM4_CTI_DEVICE_ID;
    volatile uint32_t HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER;
    volatile uint32_t HSM_CM4_CTI_PERIPHERALID4;
    volatile uint32_t HSM_CM4_CTI_PERIPHERALID5;
    volatile uint32_t HSM_CM4_CTI_PERIPHERALID6;
    volatile uint32_t HSM_CM4_CTI_PERIPHERALID7;
    volatile uint32_t HSM_CM4_CTI_PERIPHERALID0;
    volatile uint32_t HSM_CM4_CTI_PERIPHERALID1;
    volatile uint32_t HSM_CM4_CTI_PERIPHERALID2;
    volatile uint32_t HSM_CM4_CTI_PERIPHERALID3;
    volatile uint32_t HSM_CM4_CTI_COMPONENT_ID0;
    volatile uint32_t HSM_CM4_CTI_COMPONENT_ID1;
    volatile uint32_t HSM_CM4_CTI_COMPONENT_ID2;
    volatile uint32_t HSM_CM4_CTI_COMPONENT_ID3;
    volatile uint8_t  Resv_131072[40960];
    volatile uint32_t MSS_R5SS0_ROM;
    volatile uint8_t  Resv_196608[65532];
    volatile uint32_t MSS_R5SS0_A_APB_DBGDIDR;
    volatile uint8_t  Resv_196632[20];
    volatile uint32_t MSS_R5SS0_A_APB_DBGWFAR;
    volatile uint32_t MSS_R5SS0_A_APB_DBGVCR;
    volatile uint8_t  Resv_196644[4];
    volatile uint32_t MSS_R5SS0_A_APB_DBGECR;
    volatile uint32_t MSS_R5SS0_A_APB_DBGDSCCR;
    volatile uint8_t  Resv_196736[84];
    volatile uint32_t MSS_R5SS0_A_APB_DBGDTRRX;
    volatile uint32_t MSS_R5SS0_A_APB_DBGITR;
    volatile uint32_t MSS_R5SS0_A_APB_DBGDSCR;
    volatile uint32_t MSS_R5SS0_A_APB_DBGDTRTX;
    volatile uint32_t MSS_R5SS0_A_APB_DBGDRCR;
    volatile uint8_t  Resv_196864[108];
    volatile uint32_t MSS_R5SS0_A_APB_DBGBVR0;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBVR1;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBVR2;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBVR3;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBVR4;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBVR5;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBVR6;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBVR7;
    volatile uint8_t  Resv_196928[32];
    volatile uint32_t MSS_R5SS0_A_APB_DBGBCR0;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBCR1;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBCR2;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBCR3;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBCR4;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBCR5;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBCR6;
    volatile uint32_t MSS_R5SS0_A_APB_DBGBCR7;
    volatile uint8_t  Resv_196992[32];
    volatile uint32_t MSS_R5SS0_A_APB_DBGWVR0;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWVR1;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWVR2;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWVR3;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWVR4;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWVR5;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWVR6;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWVR7;
    volatile uint8_t  Resv_197056[32];
    volatile uint32_t MSS_R5SS0_A_APB_DBGWCR0;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWCR1;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWCR2;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWCR3;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWCR4;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWCR5;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWCR6;
    volatile uint32_t MSS_R5SS0_A_APB_DBGWCR7;
    volatile uint8_t  Resv_197376[288];
    volatile uint32_t MSS_R5SS0_A_APB_DBGOSLAR;
    volatile uint32_t MSS_R5SS0_A_APB_DBGOSLSR;
    volatile uint32_t MSS_R5SS0_A_APB_DBGOSSRR;
    volatile uint8_t  Resv_197392[4];
    volatile uint32_t MSS_R5SS0_A_APB_DBGPRCR;
    volatile uint32_t MSS_R5SS0_A_APB_DBGPRSR;
    volatile uint8_t  Resv_199936[2536];
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_MIDR;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_CTR;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_TCMTR;
    volatile uint8_t  Resv_199952[4];
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_MPUIR;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_MPIDR;
    volatile uint8_t  Resv_199968[8];
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_PFR0;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_PFR1;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_DFR0;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_AFR0;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_MMFR0;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_MMFR1;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_MMFR2;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_MMFR3;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_ISAR0;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_ISAR1;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_ISAR2;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_ISAR3;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_ISAR4;
    volatile uint32_t MSS_R5SS0_A_APB_PROCID_ISAR5;
    volatile uint8_t  Resv_200448[424];
    volatile uint32_t MSS_R5SS0_A_APB_MR_ITCTRL;
    volatile uint8_t  Resv_200608[156];
    volatile uint32_t MSS_R5SS0_A_APB_MR_CLAIMSET;
    volatile uint32_t MSS_R5SS0_A_APB_MR_CLAIMCLR;
    volatile uint8_t  Resv_200624[8];
    volatile uint32_t MSS_R5SS0_A_APB_MR_LOCKACCESS;
    volatile uint32_t MSS_R5SS0_A_APB_MR_LOCKSTATUS;
    volatile uint32_t MSS_R5SS0_A_APB_MR_AUTHSTATUS;
    volatile uint8_t  Resv_200648[12];
    volatile uint32_t MSS_R5SS0_A_APB_MR_DEVID;
    volatile uint32_t MSS_R5SS0_A_APB_MR_DEVTYPE;
    volatile uint32_t MSS_R5SS0_A_APB_PERIP_ID4;
    volatile uint8_t  Resv_200672[12];
    volatile uint32_t MSS_R5SS0_A_APB_PERIP_ID0;
    volatile uint32_t MSS_R5SS0_A_APB_PERIP_ID1;
    volatile uint32_t MSS_R5SS0_A_APB_PERIP_ID2;
    volatile uint32_t MSS_R5SS0_A_APB_PERIP_ID3;
    volatile uint32_t MSS_R5SS0_A_APB_COMP_ID0;
    volatile uint32_t MSS_R5SS0_A_APB_COMP_ID1;
    volatile uint32_t MSS_R5SS0_A_APB_COMP_ID2;
    volatile uint32_t MSS_R5SS0_A_APB_COMP_ID3;
    volatile uint8_t  Resv_204800[4096];
    volatile uint32_t MSS_R5SS0_B_APB_DBGDIDR;
    volatile uint8_t  Resv_204824[20];
    volatile uint32_t MSS_R5SS0_B_APB_DBGWFAR;
    volatile uint32_t MSS_R5SS0_B_APB_DBGVCR;
    volatile uint8_t  Resv_204836[4];
    volatile uint32_t MSS_R5SS0_B_APB_DBGECR;
    volatile uint32_t MSS_R5SS0_B_APB_DBGDSCCR;
    volatile uint8_t  Resv_204928[84];
    volatile uint32_t MSS_R5SS0_B_APB_DBGDTRRX;
    volatile uint32_t MSS_R5SS0_B_APB_DBGITR;
    volatile uint32_t MSS_R5SS0_B_APB_DBGDSCR;
    volatile uint32_t MSS_R5SS0_B_APB_DBGDTRTX;
    volatile uint32_t MSS_R5SS0_B_APB_DBGDRCR;
    volatile uint8_t  Resv_205056[108];
    volatile uint32_t MSS_R5SS0_B_APB_DBGBVR0;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBVR1;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBVR2;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBVR3;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBVR4;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBVR5;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBVR6;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBVR7;
    volatile uint8_t  Resv_205120[32];
    volatile uint32_t MSS_R5SS0_B_APB_DBGBCR0;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBCR1;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBCR2;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBCR3;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBCR4;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBCR5;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBCR6;
    volatile uint32_t MSS_R5SS0_B_APB_DBGBCR7;
    volatile uint8_t  Resv_205184[32];
    volatile uint32_t MSS_R5SS0_B_APB_DBGWVR0;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWVR1;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWVR2;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWVR3;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWVR4;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWVR5;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWVR6;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWVR7;
    volatile uint8_t  Resv_205248[32];
    volatile uint32_t MSS_R5SS0_B_APB_DBGWCR0;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWCR1;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWCR2;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWCR3;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWCR4;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWCR5;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWCR6;
    volatile uint32_t MSS_R5SS0_B_APB_DBGWCR7;
    volatile uint8_t  Resv_205568[288];
    volatile uint32_t MSS_R5SS0_B_APB_DBGOSLAR;
    volatile uint32_t MSS_R5SS0_B_APB_DBGOSLSR;
    volatile uint32_t MSS_R5SS0_B_APB_DBGOSSRR;
    volatile uint8_t  Resv_205584[4];
    volatile uint32_t MSS_R5SS0_B_APB_DBGPRCR;
    volatile uint32_t MSS_R5SS0_B_APB_DBGPRSR;
    volatile uint8_t  Resv_208128[2536];
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_MIDR;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_CTR;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_TCMTR;
    volatile uint8_t  Resv_208144[4];
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_MPUIR;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_MPIDR;
    volatile uint8_t  Resv_208160[8];
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_PFR0;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_PFR1;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_DFR0;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_AFR0;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_MMFR0;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_MMFR1;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_MMFR2;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_MMFR3;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_ISAR0;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_ISAR1;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_ISAR2;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_ISAR3;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_ISAR4;
    volatile uint32_t MSS_R5SS0_B_APB_PROCID_ISAR5;
    volatile uint8_t  Resv_208640[424];
    volatile uint32_t MSS_R5SS0_B_APB_MR_ITCTRL;
    volatile uint8_t  Resv_208800[156];
    volatile uint32_t MSS_R5SS0_B_APB_MR_CLAIMSET;
    volatile uint32_t MSS_R5SS0_B_APB_MR_CLAIMCLR;
    volatile uint8_t  Resv_208816[8];
    volatile uint32_t MSS_R5SS0_B_APB_MR_LOCKACCESS;
    volatile uint32_t MSS_R5SS0_B_APB_MR_LOCKSTATUS;
    volatile uint32_t MSS_R5SS0_B_APB_MR_AUTHSTATUS;
    volatile uint8_t  Resv_208840[12];
    volatile uint32_t MSS_R5SS0_B_APB_MR_DEVID;
    volatile uint32_t MSS_R5SS0_B_APB_MR_DEVTYPE;
    volatile uint32_t MSS_R5SS0_B_APB_PERIP_ID4;
    volatile uint8_t  Resv_208864[12];
    volatile uint32_t MSS_R5SS0_B_APB_PERIP_ID0;
    volatile uint32_t MSS_R5SS0_B_APB_PERIP_ID1;
    volatile uint32_t MSS_R5SS0_B_APB_PERIP_ID2;
    volatile uint32_t MSS_R5SS0_B_APB_PERIP_ID3;
    volatile uint32_t MSS_R5SS0_B_APB_COMP_ID0;
    volatile uint32_t MSS_R5SS0_B_APB_COMP_ID1;
    volatile uint32_t MSS_R5SS0_B_APB_COMP_ID2;
    volatile uint32_t MSS_R5SS0_B_APB_COMP_ID3;
    volatile uint8_t  Resv_229376[20480];
    volatile uint32_t MSS_R5SS0_A_CTI_CONTROL;
    volatile uint8_t  Resv_229392[12];
    volatile uint32_t MSS_R5SS0_A_CTI_INTACK;
    volatile uint32_t MSS_R5SS0_A_CTI_APPSET;
    volatile uint32_t MSS_R5SS0_A_CTI_APPCLEAR;
    volatile uint32_t MSS_R5SS0_A_CTI_APPPULSE;
    volatile uint32_t MSS_R5SS0_A_CTI_INEN0;
    volatile uint32_t MSS_R5SS0_A_CTI_INEN1;
    volatile uint32_t MSS_R5SS0_A_CTI_INEN2;
    volatile uint32_t MSS_R5SS0_A_CTI_INEN3;
    volatile uint32_t MSS_R5SS0_A_CTI_INEN4;
    volatile uint32_t MSS_R5SS0_A_CTI_INEN5;
    volatile uint32_t MSS_R5SS0_A_CTI_INEN6;
    volatile uint32_t MSS_R5SS0_A_CTI_INEN7;
    volatile uint8_t  Resv_229536[96];
    volatile uint32_t MSS_R5SS0_A_CTI_OUTEN0;
    volatile uint32_t MSS_R5SS0_A_CTI_OUTEN1;
    volatile uint32_t MSS_R5SS0_A_CTI_OUTEN2;
    volatile uint32_t MSS_R5SS0_A_CTI_OUTEN3;
    volatile uint32_t MSS_R5SS0_A_CTI_OUTEN4;
    volatile uint32_t MSS_R5SS0_A_CTI_OUTEN5;
    volatile uint32_t MSS_R5SS0_A_CTI_OUTEN6;
    volatile uint32_t MSS_R5SS0_A_CTI_OUTEN7;
    volatile uint8_t  Resv_229680[112];
    volatile uint32_t MSS_R5SS0_A_CTI_TRIGINSTATUS;
    volatile uint32_t MSS_R5SS0_A_CTI_TRIGOUTSTATUS;
    volatile uint32_t MSS_R5SS0_A_CTI_CHINSTATUS;
    volatile uint32_t MSS_R5SS0_A_CTI_CHOUTSTATUS;
    volatile uint32_t MSS_R5SS0_A_CTI_GATE;
    volatile uint32_t MSS_R5SS0_A_CTI_ASICCTL;
    volatile uint8_t  Resv_233180[3476];
    volatile uint32_t MSS_R5SS0_A_CTI_ITCHINACK;
    volatile uint32_t MSS_R5SS0_A_CTI_ITTRIGINACK;
    volatile uint32_t MSS_R5SS0_A_CTI_ITCHOUT;
    volatile uint32_t MSS_R5SS0_A_CTI_ITTRIGOUT;
    volatile uint32_t MSS_R5SS0_A_CTI_ITCHOUTACK;
    volatile uint32_t MSS_R5SS0_A_CTI_ITTRIGOUTACK;
    volatile uint32_t MSS_R5SS0_A_CTI_ITCHIN;
    volatile uint32_t MSS_R5SS0_A_CTI_ITTRIGIN;
    volatile uint8_t  Resv_233216[4];
    volatile uint32_t MSS_R5SS0_A_CTI_ITCTRL;
    volatile uint8_t  Resv_233376[156];
    volatile uint32_t MSS_R5SS0_A_CTI_CLAIM_TAG_SET;
    volatile uint32_t MSS_R5SS0_A_CTI_CLAIM_TAG_CLEAR;
    volatile uint8_t  Resv_233392[8];
    volatile uint32_t MSS_R5SS0_A_CTI_LOCK_ACCESS_REGISTER;
    volatile uint32_t MSS_R5SS0_A_CTI_LOCK_STATUS_REGISTER;
    volatile uint32_t MSS_R5SS0_A_CTI_AUTHENTICATION_STATUS;
    volatile uint8_t  Resv_233416[12];
    volatile uint32_t MSS_R5SS0_A_CTI_DEVICE_ID;
    volatile uint32_t MSS_R5SS0_A_CTI_DEVICE_TYPE_IDENTIFIER;
    volatile uint32_t MSS_R5SS0_A_CTI_PERIPHERALID4;
    volatile uint32_t MSS_R5SS0_A_CTI_PERIPHERALID5;
    volatile uint32_t MSS_R5SS0_A_CTI_PERIPHERALID6;
    volatile uint32_t MSS_R5SS0_A_CTI_PERIPHERALID7;
    volatile uint32_t MSS_R5SS0_A_CTI_PERIPHERALID0;
    volatile uint32_t MSS_R5SS0_A_CTI_PERIPHERALID1;
    volatile uint32_t MSS_R5SS0_A_CTI_PERIPHERALID2;
    volatile uint32_t MSS_R5SS0_A_CTI_PERIPHERALID3;
    volatile uint32_t MSS_R5SS0_A_CTI_COMPONENT_ID0;
    volatile uint32_t MSS_R5SS0_A_CTI_COMPONENT_ID1;
    volatile uint32_t MSS_R5SS0_A_CTI_COMPONENT_ID2;
    volatile uint32_t MSS_R5SS0_A_CTI_COMPONENT_ID3;
    volatile uint32_t MSS_R5SS0_B_CTI_CONTROL;
    volatile uint8_t  Resv_233488[12];
    volatile uint32_t MSS_R5SS0_B_CTI_INTACK;
    volatile uint32_t MSS_R5SS0_B_CTI_APPSET;
    volatile uint32_t MSS_R5SS0_B_CTI_APPCLEAR;
    volatile uint32_t MSS_R5SS0_B_CTI_APPPULSE;
    volatile uint32_t MSS_R5SS0_B_CTI_INEN0;
    volatile uint32_t MSS_R5SS0_B_CTI_INEN1;
    volatile uint32_t MSS_R5SS0_B_CTI_INEN2;
    volatile uint32_t MSS_R5SS0_B_CTI_INEN3;
    volatile uint32_t MSS_R5SS0_B_CTI_INEN4;
    volatile uint32_t MSS_R5SS0_B_CTI_INEN5;
    volatile uint32_t MSS_R5SS0_B_CTI_INEN6;
    volatile uint32_t MSS_R5SS0_B_CTI_INEN7;
    volatile uint8_t  Resv_233632[96];
    volatile uint32_t MSS_R5SS0_B_CTI_OUTEN0;
    volatile uint32_t MSS_R5SS0_B_CTI_OUTEN1;
    volatile uint32_t MSS_R5SS0_B_CTI_OUTEN2;
    volatile uint32_t MSS_R5SS0_B_CTI_OUTEN3;
    volatile uint32_t MSS_R5SS0_B_CTI_OUTEN4;
    volatile uint32_t MSS_R5SS0_B_CTI_OUTEN5;
    volatile uint32_t MSS_R5SS0_B_CTI_OUTEN6;
    volatile uint32_t MSS_R5SS0_B_CTI_OUTEN7;
    volatile uint8_t  Resv_233776[112];
    volatile uint32_t MSS_R5SS0_B_CTI_TRIGINSTATUS;
    volatile uint32_t MSS_R5SS0_B_CTI_TRIGOUTSTATUS;
    volatile uint32_t MSS_R5SS0_B_CTI_CHINSTATUS;
    volatile uint32_t MSS_R5SS0_B_CTI_CHOUTSTATUS;
    volatile uint32_t MSS_R5SS0_B_CTI_GATE;
    volatile uint32_t MSS_R5SS0_B_CTI_ASICCTL;
    volatile uint8_t  Resv_237276[3476];
    volatile uint32_t MSS_R5SS0_B_CTI_ITCHINACK;
    volatile uint32_t MSS_R5SS0_B_CTI_ITTRIGINACK;
    volatile uint32_t MSS_R5SS0_B_CTI_ITCHOUT;
    volatile uint32_t MSS_R5SS0_B_CTI_ITTRIGOUT;
    volatile uint32_t MSS_R5SS0_B_CTI_ITCHOUTACK;
    volatile uint32_t MSS_R5SS0_B_CTI_ITTRIGOUTACK;
    volatile uint32_t MSS_R5SS0_B_CTI_ITCHIN;
    volatile uint32_t MSS_R5SS0_B_CTI_ITTRIGIN;
    volatile uint8_t  Resv_237312[4];
    volatile uint32_t MSS_R5SS0_B_CTI_ITCTRL;
    volatile uint8_t  Resv_237472[156];
    volatile uint32_t MSS_R5SS0_B_CTI_CLAIM_TAG_SET;
    volatile uint32_t MSS_R5SS0_B_CTI_CLAIM_TAG_CLEAR;
    volatile uint8_t  Resv_237488[8];
    volatile uint32_t MSS_R5SS0_B_CTI_LOCK_ACCESS_REGISTER;
    volatile uint32_t MSS_R5SS0_B_CTI_LOCK_STATUS_REGISTER;
    volatile uint32_t MSS_R5SS0_B_CTI_AUTHENTICATION_STATUS;
    volatile uint8_t  Resv_237512[12];
    volatile uint32_t MSS_R5SS0_B_CTI_DEVICE_ID;
    volatile uint32_t MSS_R5SS0_B_CTI_DEVICE_TYPE_IDENTIFIER;
    volatile uint32_t MSS_R5SS0_B_CTI_PERIPHERALID4;
    volatile uint32_t MSS_R5SS0_B_CTI_PERIPHERALID5;
    volatile uint32_t MSS_R5SS0_B_CTI_PERIPHERALID6;
    volatile uint32_t MSS_R5SS0_B_CTI_PERIPHERALID7;
    volatile uint32_t MSS_R5SS0_B_CTI_PERIPHERALID0;
    volatile uint32_t MSS_R5SS0_B_CTI_PERIPHERALID1;
    volatile uint32_t MSS_R5SS0_B_CTI_PERIPHERALID2;
    volatile uint32_t MSS_R5SS0_B_CTI_PERIPHERALID3;
    volatile uint32_t MSS_R5SS0_B_CTI_COMPONENT_ID0;
    volatile uint32_t MSS_R5SS0_B_CTI_COMPONENT_ID1;
    volatile uint32_t MSS_R5SS0_B_CTI_COMPONENT_ID2;
    volatile uint32_t MSS_R5SS0_B_CTI_COMPONENT_ID3;
    volatile uint8_t  Resv_245760[8192];
    volatile uint32_t MSS_R5SS0_A_ETM_CR;
    volatile uint32_t MSS_R5SS0_A_ETM_CCR;
    volatile uint32_t MSS_R5SS0_A_ETM_TRIGGER;
    volatile uint32_t MSS_R5SS0_A_ETM_ASICCTLR;
    volatile uint32_t MSS_R5SS0_A_ETM_SR;
    volatile uint32_t MSS_R5SS0_A_ETM_SCR;
    volatile uint32_t MSS_R5SS0_A_ETM_TSSCR;
    volatile uint32_t MSS_R5SS0_A_ETM_TECR2;
    volatile uint32_t MSS_R5SS0_A_ETM_TEEVR;
    volatile uint32_t MSS_R5SS0_A_ETM_TECR1;
    volatile uint32_t MSS_R5SS0_A_ETM_FFRR;
    volatile uint32_t MSS_R5SS0_A_ETM_FFLR;
    volatile uint32_t MSS_R5SS0_A_ETM_VDEVR;
    volatile uint32_t MSS_R5SS0_A_ETM_VDCR1;
    volatile uint32_t MSS_R5SS0_A_ETM_VDCR2;
    volatile uint32_t MSS_R5SS0_A_ETM_VDCR3;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR1;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR2;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR3;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR4;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR5;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR6;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR7;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR8;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR9;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR10;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR11;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR12;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR13;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR14;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR15;
    volatile uint32_t MSS_R5SS0_A_ETM_ACVR16;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR1;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR2;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR3;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR4;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR5;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR6;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR7;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR8;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR9;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR10;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR11;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR12;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR13;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR14;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR15;
    volatile uint32_t MSS_R5SS0_A_ETM_ACTR16;
    volatile uint32_t MSS_R5SS0_A_ETM_DCVR1;
    volatile uint8_t  Resv_245960[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCVR2;
    volatile uint8_t  Resv_245968[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCVR3;
    volatile uint8_t  Resv_245976[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCVR4;
    volatile uint8_t  Resv_245984[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCVR5;
    volatile uint8_t  Resv_245992[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCVR6;
    volatile uint8_t  Resv_246000[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCVR7;
    volatile uint8_t  Resv_246008[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCVR8;
    volatile uint8_t  Resv_246016[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCMR1;
    volatile uint8_t  Resv_246024[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCMR2;
    volatile uint8_t  Resv_246032[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCMR3;
    volatile uint8_t  Resv_246040[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCMR4;
    volatile uint8_t  Resv_246048[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCMR5;
    volatile uint8_t  Resv_246056[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCMR6;
    volatile uint8_t  Resv_246064[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCMR7;
    volatile uint8_t  Resv_246072[4];
    volatile uint32_t MSS_R5SS0_A_ETM_DCMR8;
    volatile uint8_t  Resv_246080[4];
    volatile uint32_t MSS_R5SS0_A_ETM_CNTRLDVR1;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTRLDVR2;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTRLDVR3;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTRLDVR4;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTENR1;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTENR2;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTENR3;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTENR4;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTRLDEVR1;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTRLDEVR2;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTRLDEVR3;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTRLDEVR4;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTVR1;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTVR2;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTVR3;
    volatile uint32_t MSS_R5SS0_A_ETM_CNTVR4;
    volatile uint32_t MSS_R5SS0_A_ETM_SQ12EVR;
    volatile uint32_t MSS_R5SS0_A_ETM_SQ21EVR;
    volatile uint32_t MSS_R5SS0_A_ETM_SQ23EVR;
    volatile uint32_t MSS_R5SS0_A_ETM_SQ31EVR;
    volatile uint32_t MSS_R5SS0_A_ETM_SQ32EVR;
    volatile uint32_t MSS_R5SS0_A_ETM_SQ13EVR;
    volatile uint8_t  Resv_246172[4];
    volatile uint32_t MSS_R5SS0_A_ETM_SQR;
    volatile uint32_t MSS_R5SS0_A_ETM_EXTOUTEVR1;
    volatile uint32_t MSS_R5SS0_A_ETM_EXTOUTEVR2;
    volatile uint32_t MSS_R5SS0_A_ETM_EXTOUTEVR3;
    volatile uint32_t MSS_R5SS0_A_ETM_EXTOUTEVR4;
    volatile uint32_t MSS_R5SS0_A_ETM_CIDCVR1;
    volatile uint32_t MSS_R5SS0_A_ETM_CIDCVR2;
    volatile uint32_t MSS_R5SS0_A_ETM_CIDCVR3;
    volatile uint32_t MSS_R5SS0_A_ETM_CIDCMR;
    volatile uint8_t  Resv_246240[32];
    volatile uint32_t MSS_R5SS0_A_ETM_SYNCFR;
    volatile uint32_t MSS_R5SS0_A_ETM_IDR;
    volatile uint32_t MSS_R5SS0_A_ETM_CCER;
    volatile uint32_t MSS_R5SS0_A_ETM_EXTINSELR;
    volatile uint8_t  Resv_246272[16];
    volatile uint32_t MSS_R5SS0_A_ETM_TRACEIDR;
    volatile uint8_t  Resv_246548[272];
    volatile uint32_t MSS_R5SS0_A_ETM_PDSR;
    volatile uint8_t  Resv_249560[3008];
    volatile uint32_t MSS_R5SS0_A_ETM_ITETMIF;
    volatile uint32_t MSS_R5SS0_A_ETM_ITMISCOUT;
    volatile uint32_t MSS_R5SS0_A_ETM_ITMISCIN;
    volatile uint32_t MSS_R5SS0_A_ETM_ITTRIGGERACK;
    volatile uint32_t MSS_R5SS0_A_ETM_ITTRIGGERREQ;
    volatile uint32_t MSS_R5SS0_A_ETM_ITATBDATA0;
    volatile uint32_t MSS_R5SS0_A_ETM_ITATBCTR2;
    volatile uint32_t MSS_R5SS0_A_ETM_ITATBCTR1;
    volatile uint32_t MSS_R5SS0_A_ETM_ITATBCTR0;
    volatile uint8_t  Resv_249600[4];
    volatile uint32_t MSS_R5SS0_A_ETM_ITCTRL;
    volatile uint8_t  Resv_249760[156];
    volatile uint32_t MSS_R5SS0_A_ETM_CLAIMSET;
    volatile uint32_t MSS_R5SS0_A_ETM_CLAIMCLR;
    volatile uint8_t  Resv_249776[8];
    volatile uint32_t MSS_R5SS0_A_ETM_LAR;
    volatile uint32_t MSS_R5SS0_A_ETM_LSR;
    volatile uint32_t MSS_R5SS0_A_ETM_AUTHSTATUS;
    volatile uint8_t  Resv_249800[12];
    volatile uint32_t MSS_R5SS0_A_ETM_DEVID;
    volatile uint32_t MSS_R5SS0_A_ETM_DEVTYPE;
    volatile uint32_t MSS_R5SS0_A_ETM_PIDR4;
    volatile uint32_t MSS_R5SS0_A_ETM_PIDR5;
    volatile uint32_t MSS_R5SS0_A_ETM_PIDR6;
    volatile uint32_t MSS_R5SS0_A_ETM_PIDR7;
    volatile uint32_t MSS_R5SS0_A_ETM_PIDR0;
    volatile uint32_t MSS_R5SS0_A_ETM_PIDR1;
    volatile uint32_t MSS_R5SS0_A_ETM_PIDR2;
    volatile uint32_t MSS_R5SS0_A_ETM_PIDR3;
    volatile uint32_t MSS_R5SS0_A_ETM_CIDR0;
    volatile uint32_t MSS_R5SS0_A_ETM_CIDR1;
    volatile uint32_t MSS_R5SS0_A_ETM_CIDR2;
    volatile uint32_t MSS_R5SS0_A_ETM_CIDR3;
    volatile uint32_t MSS_R5SS0_B_ETM_CR;
    volatile uint32_t MSS_R5SS0_B_ETM_CCR;
    volatile uint32_t MSS_R5SS0_B_ETM_TRIGGER;
    volatile uint32_t MSS_R5SS0_B_ETM_ASICCTLR;
    volatile uint32_t MSS_R5SS0_B_ETM_SR;
    volatile uint32_t MSS_R5SS0_B_ETM_SCR;
    volatile uint32_t MSS_R5SS0_B_ETM_TSSCR;
    volatile uint32_t MSS_R5SS0_B_ETM_TECR2;
    volatile uint32_t MSS_R5SS0_B_ETM_TEEVR;
    volatile uint32_t MSS_R5SS0_B_ETM_TECR1;
    volatile uint32_t MSS_R5SS0_B_ETM_FFRR;
    volatile uint32_t MSS_R5SS0_B_ETM_FFLR;
    volatile uint32_t MSS_R5SS0_B_ETM_VDEVR;
    volatile uint32_t MSS_R5SS0_B_ETM_VDCR1;
    volatile uint32_t MSS_R5SS0_B_ETM_VDCR2;
    volatile uint32_t MSS_R5SS0_B_ETM_VDCR3;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR1;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR2;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR3;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR4;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR5;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR6;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR7;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR8;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR9;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR10;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR11;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR12;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR13;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR14;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR15;
    volatile uint32_t MSS_R5SS0_B_ETM_ACVR16;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR1;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR2;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR3;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR4;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR5;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR6;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR7;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR8;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR9;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR10;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR11;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR12;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR13;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR14;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR15;
    volatile uint32_t MSS_R5SS0_B_ETM_ACTR16;
    volatile uint32_t MSS_R5SS0_B_ETM_DCVR1;
    volatile uint8_t  Resv_250056[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCVR2;
    volatile uint8_t  Resv_250064[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCVR3;
    volatile uint8_t  Resv_250072[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCVR4;
    volatile uint8_t  Resv_250080[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCVR5;
    volatile uint8_t  Resv_250088[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCVR6;
    volatile uint8_t  Resv_250096[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCVR7;
    volatile uint8_t  Resv_250104[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCVR8;
    volatile uint8_t  Resv_250112[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCMR1;
    volatile uint8_t  Resv_250120[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCMR2;
    volatile uint8_t  Resv_250128[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCMR3;
    volatile uint8_t  Resv_250136[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCMR4;
    volatile uint8_t  Resv_250144[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCMR5;
    volatile uint8_t  Resv_250152[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCMR6;
    volatile uint8_t  Resv_250160[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCMR7;
    volatile uint8_t  Resv_250168[4];
    volatile uint32_t MSS_R5SS0_B_ETM_DCMR8;
    volatile uint8_t  Resv_250176[4];
    volatile uint32_t MSS_R5SS0_B_ETM_CNTRLDVR1;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTRLDVR2;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTRLDVR3;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTRLDVR4;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTENR1;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTENR2;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTENR3;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTENR4;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTRLDEVR1;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTRLDEVR2;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTRLDEVR3;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTRLDEVR4;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTVR1;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTVR2;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTVR3;
    volatile uint32_t MSS_R5SS0_B_ETM_CNTVR4;
    volatile uint32_t MSS_R5SS0_B_ETM_SQ12EVR;
    volatile uint32_t MSS_R5SS0_B_ETM_SQ21EVR;
    volatile uint32_t MSS_R5SS0_B_ETM_SQ23EVR;
    volatile uint32_t MSS_R5SS0_B_ETM_SQ31EVR;
    volatile uint32_t MSS_R5SS0_B_ETM_SQ32EVR;
    volatile uint32_t MSS_R5SS0_B_ETM_SQ13EVR;
    volatile uint8_t  Resv_250268[4];
    volatile uint32_t MSS_R5SS0_B_ETM_SQR;
    volatile uint32_t MSS_R5SS0_B_ETM_EXTOUTEVR1;
    volatile uint32_t MSS_R5SS0_B_ETM_EXTOUTEVR2;
    volatile uint32_t MSS_R5SS0_B_ETM_EXTOUTEVR3;
    volatile uint32_t MSS_R5SS0_B_ETM_EXTOUTEVR4;
    volatile uint32_t MSS_R5SS0_B_ETM_CIDCVR1;
    volatile uint32_t MSS_R5SS0_B_ETM_CIDCVR2;
    volatile uint32_t MSS_R5SS0_B_ETM_CIDCVR3;
    volatile uint32_t MSS_R5SS0_B_ETM_CIDCMR;
    volatile uint8_t  Resv_250336[32];
    volatile uint32_t MSS_R5SS0_B_ETM_SYNCFR;
    volatile uint32_t MSS_R5SS0_B_ETM_IDR;
    volatile uint32_t MSS_R5SS0_B_ETM_CCER;
    volatile uint32_t MSS_R5SS0_B_ETM_EXTINSELR;
    volatile uint8_t  Resv_250368[16];
    volatile uint32_t MSS_R5SS0_B_ETM_TRACEIDR;
    volatile uint8_t  Resv_250644[272];
    volatile uint32_t MSS_R5SS0_B_ETM_PDSR;
    volatile uint8_t  Resv_253656[3008];
    volatile uint32_t MSS_R5SS0_B_ETM_ITETMIF;
    volatile uint32_t MSS_R5SS0_B_ETM_ITMISCOUT;
    volatile uint32_t MSS_R5SS0_B_ETM_ITMISCIN;
    volatile uint32_t MSS_R5SS0_B_ETM_ITTRIGGERACK;
    volatile uint32_t MSS_R5SS0_B_ETM_ITTRIGGERREQ;
    volatile uint32_t MSS_R5SS0_B_ETM_ITATBDATA0;
    volatile uint32_t MSS_R5SS0_B_ETM_ITATBCTR2;
    volatile uint32_t MSS_R5SS0_B_ETM_ITATBCTR1;
    volatile uint32_t MSS_R5SS0_B_ETM_ITATBCTR0;
    volatile uint8_t  Resv_253696[4];
    volatile uint32_t MSS_R5SS0_B_ETM_ITCTRL;
    volatile uint8_t  Resv_253856[156];
    volatile uint32_t MSS_R5SS0_B_ETM_CLAIMSET;
    volatile uint32_t MSS_R5SS0_B_ETM_CLAIMCLR;
    volatile uint8_t  Resv_253872[8];
    volatile uint32_t MSS_R5SS0_B_ETM_LAR;
    volatile uint32_t MSS_R5SS0_B_ETM_LSR;
    volatile uint32_t MSS_R5SS0_B_ETM_AUTHSTATUS;
    volatile uint8_t  Resv_253896[12];
    volatile uint32_t MSS_R5SS0_B_ETM_DEVID;
    volatile uint32_t MSS_R5SS0_B_ETM_DEVTYPE;
    volatile uint32_t MSS_R5SS0_B_ETM_PIDR4;
    volatile uint32_t MSS_R5SS0_B_ETM_PIDR5;
    volatile uint32_t MSS_R5SS0_B_ETM_PIDR6;
    volatile uint32_t MSS_R5SS0_B_ETM_PIDR7;
    volatile uint32_t MSS_R5SS0_B_ETM_PIDR0;
    volatile uint32_t MSS_R5SS0_B_ETM_PIDR1;
    volatile uint32_t MSS_R5SS0_B_ETM_PIDR2;
    volatile uint32_t MSS_R5SS0_B_ETM_PIDR3;
    volatile uint32_t MSS_R5SS0_B_ETM_CIDR0;
    volatile uint32_t MSS_R5SS0_B_ETM_CIDR1;
    volatile uint32_t MSS_R5SS0_B_ETM_CIDR2;
    volatile uint32_t MSS_R5SS0_B_ETM_CIDR3;
    volatile uint8_t  Resv_262144[8192];
    volatile uint32_t MSS_R5SS1_ROM;
    volatile uint8_t  Resv_327680[65532];
    volatile uint32_t MSS_R5SS1_A_APB_DBGDIDR;
    volatile uint8_t  Resv_327704[20];
    volatile uint32_t MSS_R5SS1_A_APB_DBGWFAR;
    volatile uint32_t MSS_R5SS1_A_APB_DBGVCR;
    volatile uint8_t  Resv_327716[4];
    volatile uint32_t MSS_R5SS1_A_APB_DBGECR;
    volatile uint32_t MSS_R5SS1_A_APB_DBGDSCCR;
    volatile uint8_t  Resv_327808[84];
    volatile uint32_t MSS_R5SS1_A_APB_DBGDTRRX;
    volatile uint32_t MSS_R5SS1_A_APB_DBGITR;
    volatile uint32_t MSS_R5SS1_A_APB_DBGDSCR;
    volatile uint32_t MSS_R5SS1_A_APB_DBGDTRTX;
    volatile uint32_t MSS_R5SS1_A_APB_DBGDRCR;
    volatile uint8_t  Resv_327936[108];
    volatile uint32_t MSS_R5SS1_A_APB_DBGBVR0;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBVR1;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBVR2;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBVR3;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBVR4;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBVR5;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBVR6;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBVR7;
    volatile uint8_t  Resv_328000[32];
    volatile uint32_t MSS_R5SS1_A_APB_DBGBCR0;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBCR1;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBCR2;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBCR3;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBCR4;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBCR5;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBCR6;
    volatile uint32_t MSS_R5SS1_A_APB_DBGBCR7;
    volatile uint8_t  Resv_328064[32];
    volatile uint32_t MSS_R5SS1_A_APB_DBGWVR0;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWVR1;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWVR2;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWVR3;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWVR4;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWVR5;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWVR6;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWVR7;
    volatile uint8_t  Resv_328128[32];
    volatile uint32_t MSS_R5SS1_A_APB_DBGWCR0;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWCR1;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWCR2;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWCR3;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWCR4;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWCR5;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWCR6;
    volatile uint32_t MSS_R5SS1_A_APB_DBGWCR7;
    volatile uint8_t  Resv_328448[288];
    volatile uint32_t MSS_R5SS1_A_APB_DBGOSLAR;
    volatile uint32_t MSS_R5SS1_A_APB_DBGOSLSR;
    volatile uint32_t MSS_R5SS1_A_APB_DBGOSSRR;
    volatile uint8_t  Resv_328464[4];
    volatile uint32_t MSS_R5SS1_A_APB_DBGPRCR;
    volatile uint32_t MSS_R5SS1_A_APB_DBGPRSR;
    volatile uint8_t  Resv_331008[2536];
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_MIDR;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_CTR;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_TCMTR;
    volatile uint8_t  Resv_331024[4];
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_MPUIR;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_MPIDR;
    volatile uint8_t  Resv_331040[8];
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_PFR0;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_PFR1;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_DFR0;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_AFR0;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_MMFR0;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_MMFR1;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_MMFR2;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_MMFR3;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_ISAR0;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_ISAR1;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_ISAR2;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_ISAR3;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_ISAR4;
    volatile uint32_t MSS_R5SS1_A_APB_PROCID_ISAR5;
    volatile uint8_t  Resv_331520[424];
    volatile uint32_t MSS_R5SS1_A_APB_MR_ITCTRL;
    volatile uint8_t  Resv_331680[156];
    volatile uint32_t MSS_R5SS1_A_APB_MR_CLAIMSET;
    volatile uint32_t MSS_R5SS1_A_APB_MR_CLAIMCLR;
    volatile uint8_t  Resv_331696[8];
    volatile uint32_t MSS_R5SS1_A_APB_MR_LOCKACCESS;
    volatile uint32_t MSS_R5SS1_A_APB_MR_LOCKSTATUS;
    volatile uint32_t MSS_R5SS1_A_APB_MR_AUTHSTATUS;
    volatile uint8_t  Resv_331720[12];
    volatile uint32_t MSS_R5SS1_A_APB_MR_DEVID;
    volatile uint32_t MSS_R5SS1_A_APB_MR_DEVTYPE;
    volatile uint32_t MSS_R5SS1_A_APB_PERIP_ID4;
    volatile uint8_t  Resv_331744[12];
    volatile uint32_t MSS_R5SS1_A_APB_PERIP_ID0;
    volatile uint32_t MSS_R5SS1_A_APB_PERIP_ID1;
    volatile uint32_t MSS_R5SS1_A_APB_PERIP_ID2;
    volatile uint32_t MSS_R5SS1_A_APB_PERIP_ID3;
    volatile uint32_t MSS_R5SS1_A_APB_COMP_ID0;
    volatile uint32_t MSS_R5SS1_A_APB_COMP_ID1;
    volatile uint32_t MSS_R5SS1_A_APB_COMP_ID2;
    volatile uint32_t MSS_R5SS1_A_APB_COMP_ID3;
    volatile uint8_t  Resv_335872[4096];
    volatile uint32_t MSS_R5SS1_B_APB_DBGDIDR;
    volatile uint8_t  Resv_335896[20];
    volatile uint32_t MSS_R5SS1_B_APB_DBGWFAR;
    volatile uint32_t MSS_R5SS1_B_APB_DBGVCR;
    volatile uint8_t  Resv_335908[4];
    volatile uint32_t MSS_R5SS1_B_APB_DBGECR;
    volatile uint32_t MSS_R5SS1_B_APB_DBGDSCCR;
    volatile uint8_t  Resv_336000[84];
    volatile uint32_t MSS_R5SS1_B_APB_DBGDTRRX;
    volatile uint32_t MSS_R5SS1_B_APB_DBGITR;
    volatile uint32_t MSS_R5SS1_B_APB_DBGDSCR;
    volatile uint32_t MSS_R5SS1_B_APB_DBGDTRTX;
    volatile uint32_t MSS_R5SS1_B_APB_DBGDRCR;
    volatile uint8_t  Resv_336128[108];
    volatile uint32_t MSS_R5SS1_B_APB_DBGBVR0;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBVR1;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBVR2;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBVR3;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBVR4;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBVR5;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBVR6;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBVR7;
    volatile uint8_t  Resv_336192[32];
    volatile uint32_t MSS_R5SS1_B_APB_DBGBCR0;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBCR1;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBCR2;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBCR3;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBCR4;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBCR5;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBCR6;
    volatile uint32_t MSS_R5SS1_B_APB_DBGBCR7;
    volatile uint8_t  Resv_336256[32];
    volatile uint32_t MSS_R5SS1_B_APB_DBGWVR0;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWVR1;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWVR2;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWVR3;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWVR4;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWVR5;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWVR6;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWVR7;
    volatile uint8_t  Resv_336320[32];
    volatile uint32_t MSS_R5SS1_B_APB_DBGWCR0;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWCR1;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWCR2;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWCR3;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWCR4;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWCR5;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWCR6;
    volatile uint32_t MSS_R5SS1_B_APB_DBGWCR7;
    volatile uint8_t  Resv_336640[288];
    volatile uint32_t MSS_R5SS1_B_APB_DBGOSLAR;
    volatile uint32_t MSS_R5SS1_B_APB_DBGOSLSR;
    volatile uint32_t MSS_R5SS1_B_APB_DBGOSSRR;
    volatile uint8_t  Resv_336656[4];
    volatile uint32_t MSS_R5SS1_B_APB_DBGPRCR;
    volatile uint32_t MSS_R5SS1_B_APB_DBGPRSR;
    volatile uint8_t  Resv_339200[2536];
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_MIDR;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_CTR;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_TCMTR;
    volatile uint8_t  Resv_339216[4];
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_MPUIR;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_MPIDR;
    volatile uint8_t  Resv_339232[8];
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_PFR0;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_PFR1;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_DFR0;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_AFR0;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_MMFR0;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_MMFR1;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_MMFR2;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_MMFR3;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_ISAR0;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_ISAR1;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_ISAR2;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_ISAR3;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_ISAR4;
    volatile uint32_t MSS_R5SS1_B_APB_PROCID_ISAR5;
    volatile uint8_t  Resv_339712[424];
    volatile uint32_t MSS_R5SS1_B_APB_MR_ITCTRL;
    volatile uint8_t  Resv_339872[156];
    volatile uint32_t MSS_R5SS1_B_APB_MR_CLAIMSET;
    volatile uint32_t MSS_R5SS1_B_APB_MR_CLAIMCLR;
    volatile uint8_t  Resv_339888[8];
    volatile uint32_t MSS_R5SS1_B_APB_MR_LOCKACCESS;
    volatile uint32_t MSS_R5SS1_B_APB_MR_LOCKSTATUS;
    volatile uint32_t MSS_R5SS1_B_APB_MR_AUTHSTATUS;
    volatile uint8_t  Resv_339912[12];
    volatile uint32_t MSS_R5SS1_B_APB_MR_DEVID;
    volatile uint32_t MSS_R5SS1_B_APB_MR_DEVTYPE;
    volatile uint32_t MSS_R5SS1_B_APB_PERIP_ID4;
    volatile uint8_t  Resv_339936[12];
    volatile uint32_t MSS_R5SS1_B_APB_PERIP_ID0;
    volatile uint32_t MSS_R5SS1_B_APB_PERIP_ID1;
    volatile uint32_t MSS_R5SS1_B_APB_PERIP_ID2;
    volatile uint32_t MSS_R5SS1_B_APB_PERIP_ID3;
    volatile uint32_t MSS_R5SS1_B_APB_COMP_ID0;
    volatile uint32_t MSS_R5SS1_B_APB_COMP_ID1;
    volatile uint32_t MSS_R5SS1_B_APB_COMP_ID2;
    volatile uint32_t MSS_R5SS1_B_APB_COMP_ID3;
    volatile uint8_t  Resv_360448[20480];
    volatile uint32_t MSS_R5SS1_A_CTI_CONTROL;
    volatile uint8_t  Resv_360464[12];
    volatile uint32_t MSS_R5SS1_A_CTI_INTACK;
    volatile uint32_t MSS_R5SS1_A_CTI_APPSET;
    volatile uint32_t MSS_R5SS1_A_CTI_APPCLEAR;
    volatile uint32_t MSS_R5SS1_A_CTI_APPPULSE;
    volatile uint32_t MSS_R5SS1_A_CTI_INEN0;
    volatile uint32_t MSS_R5SS1_A_CTI_INEN1;
    volatile uint32_t MSS_R5SS1_A_CTI_INEN2;
    volatile uint32_t MSS_R5SS1_A_CTI_INEN3;
    volatile uint32_t MSS_R5SS1_A_CTI_INEN4;
    volatile uint32_t MSS_R5SS1_A_CTI_INEN5;
    volatile uint32_t MSS_R5SS1_A_CTI_INEN6;
    volatile uint32_t MSS_R5SS1_A_CTI_INEN7;
    volatile uint8_t  Resv_360608[96];
    volatile uint32_t MSS_R5SS1_A_CTI_OUTEN0;
    volatile uint32_t MSS_R5SS1_A_CTI_OUTEN1;
    volatile uint32_t MSS_R5SS1_A_CTI_OUTEN2;
    volatile uint32_t MSS_R5SS1_A_CTI_OUTEN3;
    volatile uint32_t MSS_R5SS1_A_CTI_OUTEN4;
    volatile uint32_t MSS_R5SS1_A_CTI_OUTEN5;
    volatile uint32_t MSS_R5SS1_A_CTI_OUTEN6;
    volatile uint32_t MSS_R5SS1_A_CTI_OUTEN7;
    volatile uint8_t  Resv_360752[112];
    volatile uint32_t MSS_R5SS1_A_CTI_TRIGINSTATUS;
    volatile uint32_t MSS_R5SS1_A_CTI_TRIGOUTSTATUS;
    volatile uint32_t MSS_R5SS1_A_CTI_CHINSTATUS;
    volatile uint32_t MSS_R5SS1_A_CTI_CHOUTSTATUS;
    volatile uint32_t MSS_R5SS1_A_CTI_GATE;
    volatile uint32_t MSS_R5SS1_A_CTI_ASICCTL;
    volatile uint8_t  Resv_364252[3476];
    volatile uint32_t MSS_R5SS1_A_CTI_ITCHINACK;
    volatile uint32_t MSS_R5SS1_A_CTI_ITTRIGINACK;
    volatile uint32_t MSS_R5SS1_A_CTI_ITCHOUT;
    volatile uint32_t MSS_R5SS1_A_CTI_ITTRIGOUT;
    volatile uint32_t MSS_R5SS1_A_CTI_ITCHOUTACK;
    volatile uint32_t MSS_R5SS1_A_CTI_ITTRIGOUTACK;
    volatile uint32_t MSS_R5SS1_A_CTI_ITCHIN;
    volatile uint32_t MSS_R5SS1_A_CTI_ITTRIGIN;
    volatile uint8_t  Resv_364288[4];
    volatile uint32_t MSS_R5SS1_A_CTI_ITCTRL;
    volatile uint8_t  Resv_364448[156];
    volatile uint32_t MSS_R5SS1_A_CTI_CLAIM_TAG_SET;
    volatile uint32_t MSS_R5SS1_A_CTI_CLAIM_TAG_CLEAR;
    volatile uint8_t  Resv_364464[8];
    volatile uint32_t MSS_R5SS1_A_CTI_LOCK_ACCESS_REGISTER;
    volatile uint32_t MSS_R5SS1_A_CTI_LOCK_STATUS_REGISTER;
    volatile uint32_t MSS_R5SS1_A_CTI_AUTHENTICATION_STATUS;
    volatile uint8_t  Resv_364488[12];
    volatile uint32_t MSS_R5SS1_A_CTI_DEVICE_ID;
    volatile uint32_t MSS_R5SS1_A_CTI_DEVICE_TYPE_IDENTIFIER;
    volatile uint32_t MSS_R5SS1_A_CTI_PERIPHERALID4;
    volatile uint32_t MSS_R5SS1_A_CTI_PERIPHERALID5;
    volatile uint32_t MSS_R5SS1_A_CTI_PERIPHERALID6;
    volatile uint32_t MSS_R5SS1_A_CTI_PERIPHERALID7;
    volatile uint32_t MSS_R5SS1_A_CTI_PERIPHERALID0;
    volatile uint32_t MSS_R5SS1_A_CTI_PERIPHERALID1;
    volatile uint32_t MSS_R5SS1_A_CTI_PERIPHERALID2;
    volatile uint32_t MSS_R5SS1_A_CTI_PERIPHERALID3;
    volatile uint32_t MSS_R5SS1_A_CTI_COMPONENT_ID0;
    volatile uint32_t MSS_R5SS1_A_CTI_COMPONENT_ID1;
    volatile uint32_t MSS_R5SS1_A_CTI_COMPONENT_ID2;
    volatile uint32_t MSS_R5SS1_A_CTI_COMPONENT_ID3;
    volatile uint32_t MSS_R5SS1_B_CTI_CONTROL;
    volatile uint8_t  Resv_364560[12];
    volatile uint32_t MSS_R5SS1_B_CTI_INTACK;
    volatile uint32_t MSS_R5SS1_B_CTI_APPSET;
    volatile uint32_t MSS_R5SS1_B_CTI_APPCLEAR;
    volatile uint32_t MSS_R5SS1_B_CTI_APPPULSE;
    volatile uint32_t MSS_R5SS1_B_CTI_INEN0;
    volatile uint32_t MSS_R5SS1_B_CTI_INEN1;
    volatile uint32_t MSS_R5SS1_B_CTI_INEN2;
    volatile uint32_t MSS_R5SS1_B_CTI_INEN3;
    volatile uint32_t MSS_R5SS1_B_CTI_INEN4;
    volatile uint32_t MSS_R5SS1_B_CTI_INEN5;
    volatile uint32_t MSS_R5SS1_B_CTI_INEN6;
    volatile uint32_t MSS_R5SS1_B_CTI_INEN7;
    volatile uint8_t  Resv_364704[96];
    volatile uint32_t MSS_R5SS1_B_CTI_OUTEN0;
    volatile uint32_t MSS_R5SS1_B_CTI_OUTEN1;
    volatile uint32_t MSS_R5SS1_B_CTI_OUTEN2;
    volatile uint32_t MSS_R5SS1_B_CTI_OUTEN3;
    volatile uint32_t MSS_R5SS1_B_CTI_OUTEN4;
    volatile uint32_t MSS_R5SS1_B_CTI_OUTEN5;
    volatile uint32_t MSS_R5SS1_B_CTI_OUTEN6;
    volatile uint32_t MSS_R5SS1_B_CTI_OUTEN7;
    volatile uint8_t  Resv_364848[112];
    volatile uint32_t MSS_R5SS1_B_CTI_TRIGINSTATUS;
    volatile uint32_t MSS_R5SS1_B_CTI_TRIGOUTSTATUS;
    volatile uint32_t MSS_R5SS1_B_CTI_CHINSTATUS;
    volatile uint32_t MSS_R5SS1_B_CTI_CHOUTSTATUS;
    volatile uint32_t MSS_R5SS1_B_CTI_GATE;
    volatile uint32_t MSS_R5SS1_B_CTI_ASICCTL;
    volatile uint8_t  Resv_368348[3476];
    volatile uint32_t MSS_R5SS1_B_CTI_ITCHINACK;
    volatile uint32_t MSS_R5SS1_B_CTI_ITTRIGINACK;
    volatile uint32_t MSS_R5SS1_B_CTI_ITCHOUT;
    volatile uint32_t MSS_R5SS1_B_CTI_ITTRIGOUT;
    volatile uint32_t MSS_R5SS1_B_CTI_ITCHOUTACK;
    volatile uint32_t MSS_R5SS1_B_CTI_ITTRIGOUTACK;
    volatile uint32_t MSS_R5SS1_B_CTI_ITCHIN;
    volatile uint32_t MSS_R5SS1_B_CTI_ITTRIGIN;
    volatile uint8_t  Resv_368384[4];
    volatile uint32_t MSS_R5SS1_B_CTI_ITCTRL;
    volatile uint8_t  Resv_368544[156];
    volatile uint32_t MSS_R5SS1_B_CTI_CLAIM_TAG_SET;
    volatile uint32_t MSS_R5SS1_B_CTI_CLAIM_TAG_CLEAR;
    volatile uint8_t  Resv_368560[8];
    volatile uint32_t MSS_R5SS1_B_CTI_LOCK_ACCESS_REGISTER;
    volatile uint32_t MSS_R5SS1_B_CTI_LOCK_STATUS_REGISTER;
    volatile uint32_t MSS_R5SS1_B_CTI_AUTHENTICATION_STATUS;
    volatile uint8_t  Resv_368584[12];
    volatile uint32_t MSS_R5SS1_B_CTI_DEVICE_ID;
    volatile uint32_t MSS_R5SS1_B_CTI_DEVICE_TYPE_IDENTIFIER;
    volatile uint32_t MSS_R5SS1_B_CTI_PERIPHERALID4;
    volatile uint32_t MSS_R5SS1_B_CTI_PERIPHERALID5;
    volatile uint32_t MSS_R5SS1_B_CTI_PERIPHERALID6;
    volatile uint32_t MSS_R5SS1_B_CTI_PERIPHERALID7;
    volatile uint32_t MSS_R5SS1_B_CTI_PERIPHERALID0;
    volatile uint32_t MSS_R5SS1_B_CTI_PERIPHERALID1;
    volatile uint32_t MSS_R5SS1_B_CTI_PERIPHERALID2;
    volatile uint32_t MSS_R5SS1_B_CTI_PERIPHERALID3;
    volatile uint32_t MSS_R5SS1_B_CTI_COMPONENT_ID0;
    volatile uint32_t MSS_R5SS1_B_CTI_COMPONENT_ID1;
    volatile uint32_t MSS_R5SS1_B_CTI_COMPONENT_ID2;
    volatile uint32_t MSS_R5SS1_B_CTI_COMPONENT_ID3;
    volatile uint8_t  Resv_376832[8192];
    volatile uint32_t MSS_R5SS1_A_ETM_CR;
    volatile uint32_t MSS_R5SS1_A_ETM_CCR;
    volatile uint32_t MSS_R5SS1_A_ETM_TRIGGER;
    volatile uint32_t MSS_R5SS1_A_ETM_ASICCTLR;
    volatile uint32_t MSS_R5SS1_A_ETM_SR;
    volatile uint32_t MSS_R5SS1_A_ETM_SCR;
    volatile uint32_t MSS_R5SS1_A_ETM_TSSCR;
    volatile uint32_t MSS_R5SS1_A_ETM_TECR2;
    volatile uint32_t MSS_R5SS1_A_ETM_TEEVR;
    volatile uint32_t MSS_R5SS1_A_ETM_TECR1;
    volatile uint32_t MSS_R5SS1_A_ETM_FFRR;
    volatile uint32_t MSS_R5SS1_A_ETM_FFLR;
    volatile uint32_t MSS_R5SS1_A_ETM_VDEVR;
    volatile uint32_t MSS_R5SS1_A_ETM_VDCR1;
    volatile uint32_t MSS_R5SS1_A_ETM_VDCR2;
    volatile uint32_t MSS_R5SS1_A_ETM_VDCR3;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR1;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR2;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR3;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR4;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR5;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR6;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR7;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR8;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR9;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR10;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR11;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR12;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR13;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR14;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR15;
    volatile uint32_t MSS_R5SS1_A_ETM_ACVR16;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR1;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR2;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR3;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR4;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR5;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR6;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR7;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR8;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR9;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR10;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR11;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR12;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR13;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR14;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR15;
    volatile uint32_t MSS_R5SS1_A_ETM_ACTR16;
    volatile uint32_t MSS_R5SS1_A_ETM_DCVR1;
    volatile uint8_t  Resv_377032[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCVR2;
    volatile uint8_t  Resv_377040[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCVR3;
    volatile uint8_t  Resv_377048[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCVR4;
    volatile uint8_t  Resv_377056[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCVR5;
    volatile uint8_t  Resv_377064[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCVR6;
    volatile uint8_t  Resv_377072[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCVR7;
    volatile uint8_t  Resv_377080[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCVR8;
    volatile uint8_t  Resv_377088[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCMR1;
    volatile uint8_t  Resv_377096[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCMR2;
    volatile uint8_t  Resv_377104[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCMR3;
    volatile uint8_t  Resv_377112[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCMR4;
    volatile uint8_t  Resv_377120[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCMR5;
    volatile uint8_t  Resv_377128[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCMR6;
    volatile uint8_t  Resv_377136[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCMR7;
    volatile uint8_t  Resv_377144[4];
    volatile uint32_t MSS_R5SS1_A_ETM_DCMR8;
    volatile uint8_t  Resv_377152[4];
    volatile uint32_t MSS_R5SS1_A_ETM_CNTRLDVR1;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTRLDVR2;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTRLDVR3;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTRLDVR4;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTENR1;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTENR2;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTENR3;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTENR4;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTRLDEVR1;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTRLDEVR2;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTRLDEVR3;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTRLDEVR4;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTVR1;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTVR2;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTVR3;
    volatile uint32_t MSS_R5SS1_A_ETM_CNTVR4;
    volatile uint32_t MSS_R5SS1_A_ETM_SQ12EVR;
    volatile uint32_t MSS_R5SS1_A_ETM_SQ21EVR;
    volatile uint32_t MSS_R5SS1_A_ETM_SQ23EVR;
    volatile uint32_t MSS_R5SS1_A_ETM_SQ31EVR;
    volatile uint32_t MSS_R5SS1_A_ETM_SQ32EVR;
    volatile uint32_t MSS_R5SS1_A_ETM_SQ13EVR;
    volatile uint8_t  Resv_377244[4];
    volatile uint32_t MSS_R5SS1_A_ETM_SQR;
    volatile uint32_t MSS_R5SS1_A_ETM_EXTOUTEVR1;
    volatile uint32_t MSS_R5SS1_A_ETM_EXTOUTEVR2;
    volatile uint32_t MSS_R5SS1_A_ETM_EXTOUTEVR3;
    volatile uint32_t MSS_R5SS1_A_ETM_EXTOUTEVR4;
    volatile uint32_t MSS_R5SS1_A_ETM_CIDCVR1;
    volatile uint32_t MSS_R5SS1_A_ETM_CIDCVR2;
    volatile uint32_t MSS_R5SS1_A_ETM_CIDCVR3;
    volatile uint32_t MSS_R5SS1_A_ETM_CIDCMR;
    volatile uint8_t  Resv_377312[32];
    volatile uint32_t MSS_R5SS1_A_ETM_SYNCFR;
    volatile uint32_t MSS_R5SS1_A_ETM_IDR;
    volatile uint32_t MSS_R5SS1_A_ETM_CCER;
    volatile uint32_t MSS_R5SS1_A_ETM_EXTINSELR;
    volatile uint8_t  Resv_377344[16];
    volatile uint32_t MSS_R5SS1_A_ETM_TRACEIDR;
    volatile uint8_t  Resv_377620[272];
    volatile uint32_t MSS_R5SS1_A_ETM_PDSR;
    volatile uint8_t  Resv_380632[3008];
    volatile uint32_t MSS_R5SS1_A_ETM_ITETMIF;
    volatile uint32_t MSS_R5SS1_A_ETM_ITMISCOUT;
    volatile uint32_t MSS_R5SS1_A_ETM_ITMISCIN;
    volatile uint32_t MSS_R5SS1_A_ETM_ITTRIGGERACK;
    volatile uint32_t MSS_R5SS1_A_ETM_ITTRIGGERREQ;
    volatile uint32_t MSS_R5SS1_A_ETM_ITATBDATA0;
    volatile uint32_t MSS_R5SS1_A_ETM_ITATBCTR2;
    volatile uint32_t MSS_R5SS1_A_ETM_ITATBCTR1;
    volatile uint32_t MSS_R5SS1_A_ETM_ITATBCTR0;
    volatile uint8_t  Resv_380672[4];
    volatile uint32_t MSS_R5SS1_A_ETM_ITCTRL;
    volatile uint8_t  Resv_380832[156];
    volatile uint32_t MSS_R5SS1_A_ETM_CLAIMSET;
    volatile uint32_t MSS_R5SS1_A_ETM_CLAIMCLR;
    volatile uint8_t  Resv_380848[8];
    volatile uint32_t MSS_R5SS1_A_ETM_LAR;
    volatile uint32_t MSS_R5SS1_A_ETM_LSR;
    volatile uint32_t MSS_R5SS1_A_ETM_AUTHSTATUS;
    volatile uint8_t  Resv_380872[12];
    volatile uint32_t MSS_R5SS1_A_ETM_DEVID;
    volatile uint32_t MSS_R5SS1_A_ETM_DEVTYPE;
    volatile uint32_t MSS_R5SS1_A_ETM_PIDR4;
    volatile uint32_t MSS_R5SS1_A_ETM_PIDR5;
    volatile uint32_t MSS_R5SS1_A_ETM_PIDR6;
    volatile uint32_t MSS_R5SS1_A_ETM_PIDR7;
    volatile uint32_t MSS_R5SS1_A_ETM_PIDR0;
    volatile uint32_t MSS_R5SS1_A_ETM_PIDR1;
    volatile uint32_t MSS_R5SS1_A_ETM_PIDR2;
    volatile uint32_t MSS_R5SS1_A_ETM_PIDR3;
    volatile uint32_t MSS_R5SS1_A_ETM_CIDR0;
    volatile uint32_t MSS_R5SS1_A_ETM_CIDR1;
    volatile uint32_t MSS_R5SS1_A_ETM_CIDR2;
    volatile uint32_t MSS_R5SS1_A_ETM_CIDR3;
    volatile uint32_t MSS_R5SS1_B_ETM_CR;
    volatile uint32_t MSS_R5SS1_B_ETM_CCR;
    volatile uint32_t MSS_R5SS1_B_ETM_TRIGGER;
    volatile uint32_t MSS_R5SS1_B_ETM_ASICCTLR;
    volatile uint32_t MSS_R5SS1_B_ETM_SR;
    volatile uint32_t MSS_R5SS1_B_ETM_SCR;
    volatile uint32_t MSS_R5SS1_B_ETM_TSSCR;
    volatile uint32_t MSS_R5SS1_B_ETM_TECR2;
    volatile uint32_t MSS_R5SS1_B_ETM_TEEVR;
    volatile uint32_t MSS_R5SS1_B_ETM_TECR1;
    volatile uint32_t MSS_R5SS1_B_ETM_FFRR;
    volatile uint32_t MSS_R5SS1_B_ETM_FFLR;
    volatile uint32_t MSS_R5SS1_B_ETM_VDEVR;
    volatile uint32_t MSS_R5SS1_B_ETM_VDCR1;
    volatile uint32_t MSS_R5SS1_B_ETM_VDCR2;
    volatile uint32_t MSS_R5SS1_B_ETM_VDCR3;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR1;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR2;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR3;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR4;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR5;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR6;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR7;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR8;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR9;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR10;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR11;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR12;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR13;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR14;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR15;
    volatile uint32_t MSS_R5SS1_B_ETM_ACVR16;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR1;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR2;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR3;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR4;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR5;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR6;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR7;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR8;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR9;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR10;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR11;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR12;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR13;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR14;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR15;
    volatile uint32_t MSS_R5SS1_B_ETM_ACTR16;
    volatile uint32_t MSS_R5SS1_B_ETM_DCVR1;
    volatile uint8_t  Resv_381128[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCVR2;
    volatile uint8_t  Resv_381136[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCVR3;
    volatile uint8_t  Resv_381144[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCVR4;
    volatile uint8_t  Resv_381152[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCVR5;
    volatile uint8_t  Resv_381160[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCVR6;
    volatile uint8_t  Resv_381168[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCVR7;
    volatile uint8_t  Resv_381176[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCVR8;
    volatile uint8_t  Resv_381184[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCMR1;
    volatile uint8_t  Resv_381192[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCMR2;
    volatile uint8_t  Resv_381200[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCMR3;
    volatile uint8_t  Resv_381208[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCMR4;
    volatile uint8_t  Resv_381216[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCMR5;
    volatile uint8_t  Resv_381224[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCMR6;
    volatile uint8_t  Resv_381232[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCMR7;
    volatile uint8_t  Resv_381240[4];
    volatile uint32_t MSS_R5SS1_B_ETM_DCMR8;
    volatile uint8_t  Resv_381248[4];
    volatile uint32_t MSS_R5SS1_B_ETM_CNTRLDVR1;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTRLDVR2;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTRLDVR3;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTRLDVR4;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTENR1;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTENR2;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTENR3;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTENR4;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTRLDEVR1;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTRLDEVR2;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTRLDEVR3;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTRLDEVR4;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTVR1;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTVR2;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTVR3;
    volatile uint32_t MSS_R5SS1_B_ETM_CNTVR4;
    volatile uint32_t MSS_R5SS1_B_ETM_SQ12EVR;
    volatile uint32_t MSS_R5SS1_B_ETM_SQ21EVR;
    volatile uint32_t MSS_R5SS1_B_ETM_SQ23EVR;
    volatile uint32_t MSS_R5SS1_B_ETM_SQ31EVR;
    volatile uint32_t MSS_R5SS1_B_ETM_SQ32EVR;
    volatile uint32_t MSS_R5SS1_B_ETM_SQ13EVR;
    volatile uint8_t  Resv_381340[4];
    volatile uint32_t MSS_R5SS1_B_ETM_SQR;
    volatile uint32_t MSS_R5SS1_B_ETM_EXTOUTEVR1;
    volatile uint32_t MSS_R5SS1_B_ETM_EXTOUTEVR2;
    volatile uint32_t MSS_R5SS1_B_ETM_EXTOUTEVR3;
    volatile uint32_t MSS_R5SS1_B_ETM_EXTOUTEVR4;
    volatile uint32_t MSS_R5SS1_B_ETM_CIDCVR1;
    volatile uint32_t MSS_R5SS1_B_ETM_CIDCVR2;
    volatile uint32_t MSS_R5SS1_B_ETM_CIDCVR3;
    volatile uint32_t MSS_R5SS1_B_ETM_CIDCMR;
    volatile uint8_t  Resv_381408[32];
    volatile uint32_t MSS_R5SS1_B_ETM_SYNCFR;
    volatile uint32_t MSS_R5SS1_B_ETM_IDR;
    volatile uint32_t MSS_R5SS1_B_ETM_CCER;
    volatile uint32_t MSS_R5SS1_B_ETM_EXTINSELR;
    volatile uint8_t  Resv_381440[16];
    volatile uint32_t MSS_R5SS1_B_ETM_TRACEIDR;
    volatile uint8_t  Resv_381716[272];
    volatile uint32_t MSS_R5SS1_B_ETM_PDSR;
    volatile uint8_t  Resv_384728[3008];
    volatile uint32_t MSS_R5SS1_B_ETM_ITETMIF;
    volatile uint32_t MSS_R5SS1_B_ETM_ITMISCOUT;
    volatile uint32_t MSS_R5SS1_B_ETM_ITMISCIN;
    volatile uint32_t MSS_R5SS1_B_ETM_ITTRIGGERACK;
    volatile uint32_t MSS_R5SS1_B_ETM_ITTRIGGERREQ;
    volatile uint32_t MSS_R5SS1_B_ETM_ITATBDATA0;
    volatile uint32_t MSS_R5SS1_B_ETM_ITATBCTR2;
    volatile uint32_t MSS_R5SS1_B_ETM_ITATBCTR1;
    volatile uint32_t MSS_R5SS1_B_ETM_ITATBCTR0;
    volatile uint8_t  Resv_384768[4];
    volatile uint32_t MSS_R5SS1_B_ETM_ITCTRL;
    volatile uint8_t  Resv_384928[156];
    volatile uint32_t MSS_R5SS1_B_ETM_CLAIMSET;
    volatile uint32_t MSS_R5SS1_B_ETM_CLAIMCLR;
    volatile uint8_t  Resv_384944[8];
    volatile uint32_t MSS_R5SS1_B_ETM_LAR;
    volatile uint32_t MSS_R5SS1_B_ETM_LSR;
    volatile uint32_t MSS_R5SS1_B_ETM_AUTHSTATUS;
    volatile uint8_t  Resv_384968[12];
    volatile uint32_t MSS_R5SS1_B_ETM_DEVID;
    volatile uint32_t MSS_R5SS1_B_ETM_DEVTYPE;
    volatile uint32_t MSS_R5SS1_B_ETM_PIDR4;
    volatile uint32_t MSS_R5SS1_B_ETM_PIDR5;
    volatile uint32_t MSS_R5SS1_B_ETM_PIDR6;
    volatile uint32_t MSS_R5SS1_B_ETM_PIDR7;
    volatile uint32_t MSS_R5SS1_B_ETM_PIDR0;
    volatile uint32_t MSS_R5SS1_B_ETM_PIDR1;
    volatile uint32_t MSS_R5SS1_B_ETM_PIDR2;
    volatile uint32_t MSS_R5SS1_B_ETM_PIDR3;
    volatile uint32_t MSS_R5SS1_B_ETM_CIDR0;
    volatile uint32_t MSS_R5SS1_B_ETM_CIDR1;
    volatile uint32_t MSS_R5SS1_B_ETM_CIDR2;
    volatile uint32_t MSS_R5SS1_B_ETM_CIDR3;
} CSL_mss_debugssRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE                                        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_END                                    (0x00000FFCU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CONTROL                                     (0x00001000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INTACK                                      (0x00001010U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPSET                                      (0x00001014U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPCLEAR                                    (0x00001018U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPPULSE                                    (0x0000101CU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN0                                       (0x00001020U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN1                                       (0x00001024U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN2                                       (0x00001028U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN3                                       (0x0000102CU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN4                                       (0x00001030U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN5                                       (0x00001034U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN6                                       (0x00001038U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN7                                       (0x0000103CU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN0                                      (0x000010A0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN1                                      (0x000010A4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN2                                      (0x000010A8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN3                                      (0x000010ACU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN4                                      (0x000010B0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN5                                      (0x000010B4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN6                                      (0x000010B8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN7                                      (0x000010BCU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGINSTATUS                                (0x00001130U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGOUTSTATUS                               (0x00001134U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHINSTATUS                                  (0x00001138U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHOUTSTATUS                                 (0x0000113CU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_GATE                                        (0x00001140U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ASICCTL                                     (0x00001144U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHINACK                                   (0x00001EDCU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGINACK                                 (0x00001EE0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUT                                     (0x00001EE4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUT                                   (0x00001EE8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUTACK                                  (0x00001EECU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUTACK                                (0x00001EF0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHIN                                      (0x00001EF4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGIN                                    (0x00001EF8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCTRL                                      (0x00001F00U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_SET                               (0x00001FA0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_CLEAR                             (0x00001FA4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_ACCESS_REGISTER                        (0x00001FB0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_STATUS_REGISTER                        (0x00001FB4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_AUTHENTICATION_STATUS                       (0x00001FB8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_ID                                   (0x00001FC8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER                      (0x00001FCCU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID4                               (0x00001FD0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID5                               (0x00001FD4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID6                               (0x00001FD8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID7                               (0x00001FDCU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID0                               (0x00001FE0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID1                               (0x00001FE4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID2                               (0x00001FE8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID3                               (0x00001FECU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID0                               (0x00001FF0U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID1                               (0x00001FF4U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID2                               (0x00001FF8U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID3                               (0x00001FFCU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_SPORTSZ                                    (0x00002000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CPORTSZ                                    (0x00002004U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STRIGM                                     (0x00002100U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGCNT                                    (0x00002104U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGMUL                                    (0x00002108U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STSTPTRN                                   (0x00002200U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CTSTPTRN                                   (0x00002204U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TPRCNTR                                    (0x00002208U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFSTS                                      (0x00002300U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFCTRL                                     (0x00002304U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FSCNTR                                     (0x00002308U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLIN                                    (0x00002400U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLOUT                                   (0x00002404U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLINACK                                (0x00002EE4U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLIN                                   (0x00002EE8U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBDATA0                                 (0x00002EECU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR2                                  (0x00002EF0U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR1                                  (0x00002EF4U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR0                                  (0x00002EF8U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITCTRL                                     (0x00002F00U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMSET                                   (0x00002FA0U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMCLR                                   (0x00002FA4U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LAR                                        (0x00002FB0U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LSR                                        (0x00002FB4U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_AUTHSTATUS                                 (0x00002FB8U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVID                                      (0x00002FC8U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVTYPE                                    (0x00002FCCU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR4                                      (0x00002FD0U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR5                                      (0x00002FD4U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR6                                      (0x00002FD8U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR7                                      (0x00002FDCU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR0                                      (0x00002FE0U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR1                                      (0x00002FE4U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR2                                      (0x00002FE8U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR3                                      (0x00002FECU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR0                                      (0x00002FF0U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR1                                      (0x00002FF4U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR2                                      (0x00002FF8U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR3                                      (0x00002FFCU)
#define CSL_MSS_DEBUGSS_APB_EXT_PORT0_ROM                                      (0x00010000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER0                               (0x00011000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER1                               (0x00011004U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR1                               (0x00011EF8U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR0                               (0x00011EFCU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITCTRL                                  (0x00011F00U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMSET                                (0x00011FA0U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMCLR                                (0x00011FA4U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LAR                                     (0x00011FB0U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LSR                                     (0x00011FB4U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_AUTHSTATUS                              (0x00011FB8U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVID                                   (0x00011FC8U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVTYPE                                 (0x00011FCCU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR4                                   (0x00011FD0U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED1                               (0x00011FD4U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED2                               (0x00011FD8U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED3                               (0x00011FDCU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR0                                   (0x00011FE0U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR1                                   (0x00011FE4U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR2                                   (0x00011FE8U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR3                                   (0x00011FECU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR0                                   (0x00011FF0U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR1                                   (0x00011FF4U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR2                                   (0x00011FF8U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR3                                   (0x00011FFCU)
#define CSL_MSS_DEBUGSS_ETB_RDP                                                (0x00012004U)
#define CSL_MSS_DEBUGSS_ETB_STS                                                (0x0001200CU)
#define CSL_MSS_DEBUGSS_ETB_RRD                                                (0x00012010U)
#define CSL_MSS_DEBUGSS_ETB_RRP                                                (0x00012014U)
#define CSL_MSS_DEBUGSS_ETB_RWP                                                (0x00012018U)
#define CSL_MSS_DEBUGSS_ETB_TRG                                                (0x0001201CU)
#define CSL_MSS_DEBUGSS_ETB_CTL                                                (0x00012020U)
#define CSL_MSS_DEBUGSS_ETB_RWD                                                (0x00012024U)
#define CSL_MSS_DEBUGSS_ETB_FFSR                                               (0x00012300U)
#define CSL_MSS_DEBUGSS_ETB_FFCR                                               (0x00012304U)
#define CSL_MSS_DEBUGSS_ETB_ITMISCOP0                                          (0x00012EE0U)
#define CSL_MSS_DEBUGSS_ETB_ITTRFLINACK                                        (0x00012EE4U)
#define CSL_MSS_DEBUGSS_ETB_ITTRFLIN                                           (0x00012EE8U)
#define CSL_MSS_DEBUGSS_ETB_ITATBDATA0                                         (0x00012EECU)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR2                                          (0x00012EF0U)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR1                                          (0x00012EF4U)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR0                                          (0x00012EF8U)
#define CSL_MSS_DEBUGSS_ETB_ITCTRL                                             (0x00012F00U)
#define CSL_MSS_DEBUGSS_ETB_CLAIMSET                                           (0x00012FA0U)
#define CSL_MSS_DEBUGSS_ETB_CLAIMCLR                                           (0x00012FA4U)
#define CSL_MSS_DEBUGSS_ETB_LAR                                                (0x00012FB0U)
#define CSL_MSS_DEBUGSS_ETB_LSR                                                (0x00012FB4U)
#define CSL_MSS_DEBUGSS_ETB_AUTHSTATUS                                         (0x00012FB8U)
#define CSL_MSS_DEBUGSS_ETB_DEVID                                              (0x00012FC8U)
#define CSL_MSS_DEBUGSS_ETB_DEVTYPE                                            (0x00012FCCU)
#define CSL_MSS_DEBUGSS_ETB_PIDR4                                              (0x00012FD0U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED4                                          (0x00012FD4U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED5                                          (0x00012FD8U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED6                                          (0x00012FDCU)
#define CSL_MSS_DEBUGSS_ETB_PIDR0                                              (0x00012FE0U)
#define CSL_MSS_DEBUGSS_ETB_PIDR1                                              (0x00012FE4U)
#define CSL_MSS_DEBUGSS_ETB_PIDR2                                              (0x00012FE8U)
#define CSL_MSS_DEBUGSS_ETB_PIDR3                                              (0x00012FECU)
#define CSL_MSS_DEBUGSS_ETB_CIDR0                                              (0x00012FF0U)
#define CSL_MSS_DEBUGSS_ETB_CIDR1                                              (0x00012FF4U)
#define CSL_MSS_DEBUGSS_ETB_CIDR2                                              (0x00012FF8U)
#define CSL_MSS_DEBUGSS_ETB_CIDR3                                              (0x00012FFCU)
#define CSL_MSS_DEBUGSS_STMDMASTARTR                                           (0x00013C04U)
#define CSL_MSS_DEBUGSS_STMDMASTOPR                                            (0x00013C08U)
#define CSL_MSS_DEBUGSS_STMDMASTATR                                            (0x00013C0CU)
#define CSL_MSS_DEBUGSS_STMDMACTLR                                             (0x00013C10U)
#define CSL_MSS_DEBUGSS_STMDMAIDR                                              (0x00013CFCU)
#define CSL_MSS_DEBUGSS_STMHEER                                                (0x00013D00U)
#define CSL_MSS_DEBUGSS_STMHETER                                               (0x00013D20U)
#define CSL_MSS_DEBUGSS_STMHEBSR                                               (0x00013D60U)
#define CSL_MSS_DEBUGSS_STMHEMCR                                               (0x00013D64U)
#define CSL_MSS_DEBUGSS_STMHEEXTMUXR                                           (0x00013D68U)
#define CSL_MSS_DEBUGSS_STMHEMASTR                                             (0x00013DF4U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R                                            (0x00013DF8U)
#define CSL_MSS_DEBUGSS_STMHEIDR                                               (0x00013DFCU)
#define CSL_MSS_DEBUGSS_STMSPER                                                (0x00013E00U)
#define CSL_MSS_DEBUGSS_STMSPTER                                               (0x00013E20U)
#define CSL_MSS_DEBUGSS_STMSPSCR                                               (0x00013E60U)
#define CSL_MSS_DEBUGSS_STMSPMSCR                                              (0x00013E64U)
#define CSL_MSS_DEBUGSS_STMSPOVERRIDER                                         (0x00013E68U)
#define CSL_MSS_DEBUGSS_STMSPMOVERRIDER                                        (0x00013E6CU)
#define CSL_MSS_DEBUGSS_STMSPTRIGCSR                                           (0x00013E70U)
#define CSL_MSS_DEBUGSS_STMTCSR                                                (0x00013E80U)
#define CSL_MSS_DEBUGSS_STMTSSTIMR                                             (0x00013E84U)
#define CSL_MSS_DEBUGSS_STMTSFREQR                                             (0x00013E8CU)
#define CSL_MSS_DEBUGSS_STMSYNCR                                               (0x00013E90U)
#define CSL_MSS_DEBUGSS_STMAUXCR                                               (0x00013E94U)
#define CSL_MSS_DEBUGSS_STMFEAT1R                                              (0x00013EA0U)
#define CSL_MSS_DEBUGSS_STMFEAT2R                                              (0x00013EA4U)
#define CSL_MSS_DEBUGSS_STMFEAT3R                                              (0x00013EA8U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER                                           (0x00013EE8U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0                                          (0x00013EECU)
#define CSL_MSS_DEBUGSS_STMITATBCTR2                                           (0x00013EF0U)
#define CSL_MSS_DEBUGSS_STMITATBID                                             (0x00013EF4U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0                                           (0x00013EF8U)
#define CSL_MSS_DEBUGSS_STMITCTRL                                              (0x00013F00U)
#define CSL_MSS_DEBUGSS_STMCLAIMSET                                            (0x00013FA0U)
#define CSL_MSS_DEBUGSS_STMCLAIMCLR                                            (0x00013FA4U)
#define CSL_MSS_DEBUGSS_STMLAR                                                 (0x00013FB0U)
#define CSL_MSS_DEBUGSS_STMLSR                                                 (0x00013FB4U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS                                          (0x00013FB8U)
#define CSL_MSS_DEBUGSS_STMDEVARCH                                             (0x00013FBCU)
#define CSL_MSS_DEBUGSS_STMDEVID                                               (0x00013FC8U)
#define CSL_MSS_DEBUGSS_STMDEVTYPE                                             (0x00013FCCU)
#define CSL_MSS_DEBUGSS_STMPIDR4                                               (0x00013FD0U)
#define CSL_MSS_DEBUGSS_STMPIDR0                                               (0x00013FE0U)
#define CSL_MSS_DEBUGSS_STMPIDR1                                               (0x00013FE4U)
#define CSL_MSS_DEBUGSS_STMPIDR2                                               (0x00013FE8U)
#define CSL_MSS_DEBUGSS_STMPIDR3                                               (0x00013FECU)
#define CSL_MSS_DEBUGSS_STMCIDR0                                               (0x00013FF0U)
#define CSL_MSS_DEBUGSS_STMCIDR1                                               (0x00013FF4U)
#define CSL_MSS_DEBUGSS_STMCIDR2                                               (0x00013FF8U)
#define CSL_MSS_DEBUGSS_STMCIDR3                                               (0x00013FFCU)
#define CSL_MSS_DEBUGSS_STM_CTI_CONTROL                                        (0x00014000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INTACK                                         (0x00014010U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPSET                                         (0x00014014U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPCLEAR                                       (0x00014018U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPPULSE                                       (0x0001401CU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN0                                          (0x00014020U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN1                                          (0x00014024U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN2                                          (0x00014028U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN3                                          (0x0001402CU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN4                                          (0x00014030U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN5                                          (0x00014034U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN6                                          (0x00014038U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN7                                          (0x0001403CU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN0                                         (0x000140A0U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN1                                         (0x000140A4U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN2                                         (0x000140A8U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN3                                         (0x000140ACU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN4                                         (0x000140B0U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN5                                         (0x000140B4U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN6                                         (0x000140B8U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN7                                         (0x000140BCU)
#define CSL_MSS_DEBUGSS_STM_CTI_TRIGINSTATUS                                   (0x00014130U)
#define CSL_MSS_DEBUGSS_STM_CTI_TRIGOUTSTATUS                                  (0x00014134U)
#define CSL_MSS_DEBUGSS_STM_CTI_CHINSTATUS                                     (0x00014138U)
#define CSL_MSS_DEBUGSS_STM_CTI_CHOUTSTATUS                                    (0x0001413CU)
#define CSL_MSS_DEBUGSS_STM_CTI_GATE                                           (0x00014140U)
#define CSL_MSS_DEBUGSS_STM_CTI_ASICCTL                                        (0x00014144U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHINACK                                      (0x00014EDCU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGINACK                                    (0x00014EE0U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUT                                        (0x00014EE4U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUT                                      (0x00014EE8U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUTACK                                     (0x00014EECU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUTACK                                   (0x00014EF0U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHIN                                         (0x00014EF4U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGIN                                       (0x00014EF8U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCTRL                                         (0x00014F00U)
#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_SET                                  (0x00014FA0U)
#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_CLEAR                                (0x00014FA4U)
#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_ACCESS_REGISTER                           (0x00014FB0U)
#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_STATUS_REGISTER                           (0x00014FB4U)
#define CSL_MSS_DEBUGSS_STM_CTI_AUTHENTICATION_STATUS                          (0x00014FB8U)
#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_ID                                      (0x00014FC8U)
#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_TYPE_IDENTIFIER                         (0x00014FCCU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID4                                  (0x00014FD0U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID5                                  (0x00014FD4U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID6                                  (0x00014FD8U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID7                                  (0x00014FDCU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID0                                  (0x00014FE0U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID1                                  (0x00014FE4U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID2                                  (0x00014FE8U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID3                                  (0x00014FECU)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID0                                  (0x00014FF0U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID1                                  (0x00014FF4U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID2                                  (0x00014FF8U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID3                                  (0x00014FFCU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CONTROL                                    (0x00015000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INTACK                                     (0x00015010U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPSET                                     (0x00015014U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPCLEAR                                   (0x00015018U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPPULSE                                   (0x0001501CU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN0                                      (0x00015020U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN1                                      (0x00015024U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN2                                      (0x00015028U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN3                                      (0x0001502CU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN4                                      (0x00015030U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN5                                      (0x00015034U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN6                                      (0x00015038U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN7                                      (0x0001503CU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN0                                     (0x000150A0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN1                                     (0x000150A4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN2                                     (0x000150A8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN3                                     (0x000150ACU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN4                                     (0x000150B0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN5                                     (0x000150B4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN6                                     (0x000150B8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN7                                     (0x000150BCU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGINSTATUS                               (0x00015130U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGOUTSTATUS                              (0x00015134U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHINSTATUS                                 (0x00015138U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHOUTSTATUS                                (0x0001513CU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_GATE                                       (0x00015140U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ASICCTL                                    (0x00015144U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHINACK                                  (0x00015EDCU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGINACK                                (0x00015EE0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUT                                    (0x00015EE4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUT                                  (0x00015EE8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUTACK                                 (0x00015EECU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUTACK                               (0x00015EF0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHIN                                     (0x00015EF4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGIN                                   (0x00015EF8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCTRL                                     (0x00015F00U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_SET                              (0x00015FA0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_CLEAR                            (0x00015FA4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_ACCESS_REGISTER                       (0x00015FB0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_STATUS_REGISTER                       (0x00015FB4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_AUTHENTICATION_STATUS                      (0x00015FB8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_ID                                  (0x00015FC8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER                     (0x00015FCCU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID4                              (0x00015FD0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID5                              (0x00015FD4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID6                              (0x00015FD8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID7                              (0x00015FDCU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID0                              (0x00015FE0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID1                              (0x00015FE4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID2                              (0x00015FE8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID3                              (0x00015FECU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID0                              (0x00015FF0U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID1                              (0x00015FF4U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID2                              (0x00015FF8U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID3                              (0x00015FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_ROM                                          (0x00020000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDIDR                                (0x00030000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWFAR                                (0x00030018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGVCR                                 (0x0003001CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGECR                                 (0x00030024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCCR                               (0x00030028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRRX                               (0x00030080U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGITR                                 (0x00030084U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCR                                (0x00030088U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRTX                               (0x0003008CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDRCR                                (0x00030090U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR0                                (0x00030100U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR1                                (0x00030104U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR2                                (0x00030108U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR3                                (0x0003010CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR4                                (0x00030110U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR5                                (0x00030114U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR6                                (0x00030118U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR7                                (0x0003011CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR0                                (0x00030140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR1                                (0x00030144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR2                                (0x00030148U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR3                                (0x0003014CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR4                                (0x00030150U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR5                                (0x00030154U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR6                                (0x00030158U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR7                                (0x0003015CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR0                                (0x00030180U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR1                                (0x00030184U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR2                                (0x00030188U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR3                                (0x0003018CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR4                                (0x00030190U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR5                                (0x00030194U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR6                                (0x00030198U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR7                                (0x0003019CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR0                                (0x000301C0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR1                                (0x000301C4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR2                                (0x000301C8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR3                                (0x000301CCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR4                                (0x000301D0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR5                                (0x000301D4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR6                                (0x000301D8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR7                                (0x000301DCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLAR                               (0x00030300U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLSR                               (0x00030304U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSSRR                               (0x00030308U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRCR                                (0x00030310U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRSR                                (0x00030314U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MIDR                            (0x00030D00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_CTR                             (0x00030D04U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_TCMTR                           (0x00030D08U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPUIR                           (0x00030D10U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPIDR                           (0x00030D14U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR0                            (0x00030D20U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR1                            (0x00030D24U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_DFR0                            (0x00030D28U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_AFR0                            (0x00030D2CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR0                           (0x00030D30U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR1                           (0x00030D34U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR2                           (0x00030D38U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR3                           (0x00030D3CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR0                           (0x00030D40U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR1                           (0x00030D44U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR2                           (0x00030D48U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR3                           (0x00030D4CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR4                           (0x00030D50U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR5                           (0x00030D54U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_ITCTRL                              (0x00030F00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMSET                            (0x00030FA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMCLR                            (0x00030FA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKACCESS                          (0x00030FB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKSTATUS                          (0x00030FB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_AUTHSTATUS                          (0x00030FB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVID                               (0x00030FC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVTYPE                             (0x00030FCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID4                              (0x00030FD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID0                              (0x00030FE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID1                              (0x00030FE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID2                              (0x00030FE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID3                              (0x00030FECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID0                               (0x00030FF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID1                               (0x00030FF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID2                               (0x00030FF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID3                               (0x00030FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDIDR                                (0x00032000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWFAR                                (0x00032018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGVCR                                 (0x0003201CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGECR                                 (0x00032024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCCR                               (0x00032028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRRX                               (0x00032080U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGITR                                 (0x00032084U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCR                                (0x00032088U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRTX                               (0x0003208CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDRCR                                (0x00032090U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR0                                (0x00032100U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR1                                (0x00032104U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR2                                (0x00032108U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR3                                (0x0003210CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR4                                (0x00032110U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR5                                (0x00032114U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR6                                (0x00032118U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR7                                (0x0003211CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR0                                (0x00032140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR1                                (0x00032144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR2                                (0x00032148U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR3                                (0x0003214CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR4                                (0x00032150U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR5                                (0x00032154U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR6                                (0x00032158U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR7                                (0x0003215CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR0                                (0x00032180U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR1                                (0x00032184U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR2                                (0x00032188U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR3                                (0x0003218CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR4                                (0x00032190U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR5                                (0x00032194U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR6                                (0x00032198U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR7                                (0x0003219CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR0                                (0x000321C0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR1                                (0x000321C4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR2                                (0x000321C8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR3                                (0x000321CCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR4                                (0x000321D0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR5                                (0x000321D4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR6                                (0x000321D8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR7                                (0x000321DCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLAR                               (0x00032300U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLSR                               (0x00032304U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSSRR                               (0x00032308U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRCR                                (0x00032310U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRSR                                (0x00032314U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MIDR                            (0x00032D00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_CTR                             (0x00032D04U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_TCMTR                           (0x00032D08U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPUIR                           (0x00032D10U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPIDR                           (0x00032D14U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR0                            (0x00032D20U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR1                            (0x00032D24U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_DFR0                            (0x00032D28U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_AFR0                            (0x00032D2CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR0                           (0x00032D30U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR1                           (0x00032D34U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR2                           (0x00032D38U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR3                           (0x00032D3CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR0                           (0x00032D40U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR1                           (0x00032D44U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR2                           (0x00032D48U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR3                           (0x00032D4CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR4                           (0x00032D50U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR5                           (0x00032D54U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_ITCTRL                              (0x00032F00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMSET                            (0x00032FA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMCLR                            (0x00032FA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKACCESS                          (0x00032FB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKSTATUS                          (0x00032FB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_AUTHSTATUS                          (0x00032FB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVID                               (0x00032FC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVTYPE                             (0x00032FCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID4                              (0x00032FD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID0                              (0x00032FE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID1                              (0x00032FE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID2                              (0x00032FE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID3                              (0x00032FECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID0                               (0x00032FF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID1                               (0x00032FF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID2                               (0x00032FF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID3                               (0x00032FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CONTROL                                (0x00038000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INTACK                                 (0x00038010U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPSET                                 (0x00038014U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPCLEAR                               (0x00038018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPPULSE                               (0x0003801CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN0                                  (0x00038020U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN1                                  (0x00038024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN2                                  (0x00038028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN3                                  (0x0003802CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN4                                  (0x00038030U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN5                                  (0x00038034U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN6                                  (0x00038038U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN7                                  (0x0003803CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN0                                 (0x000380A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN1                                 (0x000380A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN2                                 (0x000380A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN3                                 (0x000380ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN4                                 (0x000380B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN5                                 (0x000380B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN6                                 (0x000380B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN7                                 (0x000380BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGINSTATUS                           (0x00038130U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGOUTSTATUS                          (0x00038134U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHINSTATUS                             (0x00038138U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHOUTSTATUS                            (0x0003813CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_GATE                                   (0x00038140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ASICCTL                                (0x00038144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHINACK                              (0x00038EDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGINACK                            (0x00038EE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUT                                (0x00038EE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUT                              (0x00038EE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUTACK                             (0x00038EECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUTACK                           (0x00038EF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHIN                                 (0x00038EF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGIN                               (0x00038EF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCTRL                                 (0x00038F00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_SET                          (0x00038FA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_CLEAR                        (0x00038FA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_ACCESS_REGISTER                   (0x00038FB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_STATUS_REGISTER                   (0x00038FB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_AUTHENTICATION_STATUS                  (0x00038FB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_ID                              (0x00038FC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_TYPE_IDENTIFIER                 (0x00038FCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID4                          (0x00038FD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID5                          (0x00038FD4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID6                          (0x00038FD8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID7                          (0x00038FDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID0                          (0x00038FE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID1                          (0x00038FE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID2                          (0x00038FE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID3                          (0x00038FECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID0                          (0x00038FF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID1                          (0x00038FF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID2                          (0x00038FF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID3                          (0x00038FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CONTROL                                (0x00039000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INTACK                                 (0x00039010U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPSET                                 (0x00039014U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPCLEAR                               (0x00039018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPPULSE                               (0x0003901CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN0                                  (0x00039020U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN1                                  (0x00039024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN2                                  (0x00039028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN3                                  (0x0003902CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN4                                  (0x00039030U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN5                                  (0x00039034U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN6                                  (0x00039038U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN7                                  (0x0003903CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN0                                 (0x000390A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN1                                 (0x000390A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN2                                 (0x000390A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN3                                 (0x000390ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN4                                 (0x000390B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN5                                 (0x000390B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN6                                 (0x000390B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN7                                 (0x000390BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGINSTATUS                           (0x00039130U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGOUTSTATUS                          (0x00039134U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHINSTATUS                             (0x00039138U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHOUTSTATUS                            (0x0003913CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_GATE                                   (0x00039140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ASICCTL                                (0x00039144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHINACK                              (0x00039EDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGINACK                            (0x00039EE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUT                                (0x00039EE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUT                              (0x00039EE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUTACK                             (0x00039EECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUTACK                           (0x00039EF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHIN                                 (0x00039EF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGIN                               (0x00039EF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCTRL                                 (0x00039F00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_SET                          (0x00039FA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_CLEAR                        (0x00039FA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_ACCESS_REGISTER                   (0x00039FB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_STATUS_REGISTER                   (0x00039FB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_AUTHENTICATION_STATUS                  (0x00039FB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_ID                              (0x00039FC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_TYPE_IDENTIFIER                 (0x00039FCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID4                          (0x00039FD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID5                          (0x00039FD4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID6                          (0x00039FD8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID7                          (0x00039FDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID0                          (0x00039FE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID1                          (0x00039FE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID2                          (0x00039FE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID3                          (0x00039FECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID0                          (0x00039FF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID1                          (0x00039FF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID2                          (0x00039FF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID3                          (0x00039FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CR                                     (0x0003C000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCR                                    (0x0003C004U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRIGGER                                (0x0003C008U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ASICCTLR                               (0x0003C00CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SR                                     (0x0003C010U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SCR                                    (0x0003C014U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TSSCR                                  (0x0003C018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR2                                  (0x0003C01CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TEEVR                                  (0x0003C020U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR1                                  (0x0003C024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFRR                                   (0x0003C028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFLR                                   (0x0003C02CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDEVR                                  (0x0003C030U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR1                                  (0x0003C034U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR2                                  (0x0003C038U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR3                                  (0x0003C03CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR1                                  (0x0003C040U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR2                                  (0x0003C044U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR3                                  (0x0003C048U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR4                                  (0x0003C04CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR5                                  (0x0003C050U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR6                                  (0x0003C054U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR7                                  (0x0003C058U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR8                                  (0x0003C05CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR9                                  (0x0003C060U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR10                                 (0x0003C064U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR11                                 (0x0003C068U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR12                                 (0x0003C06CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR13                                 (0x0003C070U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR14                                 (0x0003C074U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR15                                 (0x0003C078U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR16                                 (0x0003C07CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR1                                  (0x0003C080U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR2                                  (0x0003C084U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR3                                  (0x0003C088U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR4                                  (0x0003C08CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR5                                  (0x0003C090U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR6                                  (0x0003C094U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR7                                  (0x0003C098U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR8                                  (0x0003C09CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR9                                  (0x0003C0A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR10                                 (0x0003C0A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR11                                 (0x0003C0A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR12                                 (0x0003C0ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR13                                 (0x0003C0B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR14                                 (0x0003C0B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR15                                 (0x0003C0B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR16                                 (0x0003C0BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR1                                  (0x0003C0C0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR2                                  (0x0003C0C8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR3                                  (0x0003C0D0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR4                                  (0x0003C0D8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR5                                  (0x0003C0E0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR6                                  (0x0003C0E8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR7                                  (0x0003C0F0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR8                                  (0x0003C0F8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR1                                  (0x0003C100U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR2                                  (0x0003C108U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR3                                  (0x0003C110U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR4                                  (0x0003C118U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR5                                  (0x0003C120U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR6                                  (0x0003C128U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR7                                  (0x0003C130U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR8                                  (0x0003C138U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR1                              (0x0003C140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR2                              (0x0003C144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR3                              (0x0003C148U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR4                              (0x0003C14CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR1                                (0x0003C150U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR2                                (0x0003C154U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR3                                (0x0003C158U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR4                                (0x0003C15CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR1                             (0x0003C160U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR2                             (0x0003C164U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR3                             (0x0003C168U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR4                             (0x0003C16CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR1                                 (0x0003C170U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR2                                 (0x0003C174U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR3                                 (0x0003C178U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR4                                 (0x0003C17CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ12EVR                                (0x0003C180U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ21EVR                                (0x0003C184U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ23EVR                                (0x0003C188U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ31EVR                                (0x0003C18CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ32EVR                                (0x0003C190U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ13EVR                                (0x0003C194U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQR                                    (0x0003C19CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR1                             (0x0003C1A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR2                             (0x0003C1A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR3                             (0x0003C1A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR4                             (0x0003C1ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR1                                (0x0003C1B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR2                                (0x0003C1B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR3                                (0x0003C1B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCMR                                 (0x0003C1BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SYNCFR                                 (0x0003C1E0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_IDR                                    (0x0003C1E4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCER                                   (0x0003C1E8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTINSELR                              (0x0003C1ECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRACEIDR                               (0x0003C200U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PDSR                                   (0x0003C314U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITETMIF                                (0x0003CED8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCOUT                              (0x0003CEDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCIN                               (0x0003CEE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERACK                           (0x0003CEE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERREQ                           (0x0003CEE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBDATA0                             (0x0003CEECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR2                              (0x0003CEF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR1                              (0x0003CEF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR0                              (0x0003CEF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITCTRL                                 (0x0003CF00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMSET                               (0x0003CFA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMCLR                               (0x0003CFA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LAR                                    (0x0003CFB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LSR                                    (0x0003CFB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_AUTHSTATUS                             (0x0003CFB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVID                                  (0x0003CFC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVTYPE                                (0x0003CFCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR4                                  (0x0003CFD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR5                                  (0x0003CFD4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR6                                  (0x0003CFD8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR7                                  (0x0003CFDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR0                                  (0x0003CFE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR1                                  (0x0003CFE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR2                                  (0x0003CFE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR3                                  (0x0003CFECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR0                                  (0x0003CFF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR1                                  (0x0003CFF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR2                                  (0x0003CFF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR3                                  (0x0003CFFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CR                                     (0x0003D000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCR                                    (0x0003D004U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRIGGER                                (0x0003D008U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ASICCTLR                               (0x0003D00CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SR                                     (0x0003D010U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SCR                                    (0x0003D014U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TSSCR                                  (0x0003D018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR2                                  (0x0003D01CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TEEVR                                  (0x0003D020U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR1                                  (0x0003D024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFRR                                   (0x0003D028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFLR                                   (0x0003D02CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDEVR                                  (0x0003D030U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR1                                  (0x0003D034U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR2                                  (0x0003D038U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR3                                  (0x0003D03CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR1                                  (0x0003D040U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR2                                  (0x0003D044U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR3                                  (0x0003D048U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR4                                  (0x0003D04CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR5                                  (0x0003D050U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR6                                  (0x0003D054U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR7                                  (0x0003D058U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR8                                  (0x0003D05CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR9                                  (0x0003D060U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR10                                 (0x0003D064U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR11                                 (0x0003D068U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR12                                 (0x0003D06CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR13                                 (0x0003D070U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR14                                 (0x0003D074U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR15                                 (0x0003D078U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR16                                 (0x0003D07CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR1                                  (0x0003D080U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR2                                  (0x0003D084U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR3                                  (0x0003D088U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR4                                  (0x0003D08CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR5                                  (0x0003D090U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR6                                  (0x0003D094U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR7                                  (0x0003D098U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR8                                  (0x0003D09CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR9                                  (0x0003D0A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR10                                 (0x0003D0A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR11                                 (0x0003D0A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR12                                 (0x0003D0ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR13                                 (0x0003D0B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR14                                 (0x0003D0B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR15                                 (0x0003D0B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR16                                 (0x0003D0BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR1                                  (0x0003D0C0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR2                                  (0x0003D0C8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR3                                  (0x0003D0D0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR4                                  (0x0003D0D8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR5                                  (0x0003D0E0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR6                                  (0x0003D0E8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR7                                  (0x0003D0F0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR8                                  (0x0003D0F8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR1                                  (0x0003D100U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR2                                  (0x0003D108U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR3                                  (0x0003D110U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR4                                  (0x0003D118U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR5                                  (0x0003D120U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR6                                  (0x0003D128U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR7                                  (0x0003D130U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR8                                  (0x0003D138U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR1                              (0x0003D140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR2                              (0x0003D144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR3                              (0x0003D148U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR4                              (0x0003D14CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR1                                (0x0003D150U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR2                                (0x0003D154U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR3                                (0x0003D158U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR4                                (0x0003D15CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR1                             (0x0003D160U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR2                             (0x0003D164U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR3                             (0x0003D168U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR4                             (0x0003D16CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR1                                 (0x0003D170U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR2                                 (0x0003D174U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR3                                 (0x0003D178U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR4                                 (0x0003D17CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ12EVR                                (0x0003D180U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ21EVR                                (0x0003D184U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ23EVR                                (0x0003D188U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ31EVR                                (0x0003D18CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ32EVR                                (0x0003D190U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ13EVR                                (0x0003D194U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQR                                    (0x0003D19CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR1                             (0x0003D1A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR2                             (0x0003D1A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR3                             (0x0003D1A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR4                             (0x0003D1ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR1                                (0x0003D1B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR2                                (0x0003D1B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR3                                (0x0003D1B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCMR                                 (0x0003D1BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SYNCFR                                 (0x0003D1E0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_IDR                                    (0x0003D1E4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCER                                   (0x0003D1E8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTINSELR                              (0x0003D1ECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRACEIDR                               (0x0003D200U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PDSR                                   (0x0003D314U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITETMIF                                (0x0003DED8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCOUT                              (0x0003DEDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCIN                               (0x0003DEE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERACK                           (0x0003DEE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERREQ                           (0x0003DEE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBDATA0                             (0x0003DEECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR2                              (0x0003DEF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR1                              (0x0003DEF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR0                              (0x0003DEF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITCTRL                                 (0x0003DF00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMSET                               (0x0003DFA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMCLR                               (0x0003DFA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LAR                                    (0x0003DFB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LSR                                    (0x0003DFB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_AUTHSTATUS                             (0x0003DFB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVID                                  (0x0003DFC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVTYPE                                (0x0003DFCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR4                                  (0x0003DFD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR5                                  (0x0003DFD4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR6                                  (0x0003DFD8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR7                                  (0x0003DFDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR0                                  (0x0003DFE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR1                                  (0x0003DFE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR2                                  (0x0003DFE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR3                                  (0x0003DFECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR0                                  (0x0003DFF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR1                                  (0x0003DFF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR2                                  (0x0003DFF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR3                                  (0x0003DFFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_ROM                                          (0x00040000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDIDR                                (0x00050000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWFAR                                (0x00050018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGVCR                                 (0x0005001CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGECR                                 (0x00050024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCCR                               (0x00050028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRRX                               (0x00050080U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGITR                                 (0x00050084U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCR                                (0x00050088U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRTX                               (0x0005008CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDRCR                                (0x00050090U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR0                                (0x00050100U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR1                                (0x00050104U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR2                                (0x00050108U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR3                                (0x0005010CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR4                                (0x00050110U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR5                                (0x00050114U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR6                                (0x00050118U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR7                                (0x0005011CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR0                                (0x00050140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR1                                (0x00050144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR2                                (0x00050148U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR3                                (0x0005014CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR4                                (0x00050150U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR5                                (0x00050154U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR6                                (0x00050158U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR7                                (0x0005015CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR0                                (0x00050180U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR1                                (0x00050184U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR2                                (0x00050188U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR3                                (0x0005018CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR4                                (0x00050190U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR5                                (0x00050194U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR6                                (0x00050198U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR7                                (0x0005019CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR0                                (0x000501C0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR1                                (0x000501C4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR2                                (0x000501C8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR3                                (0x000501CCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR4                                (0x000501D0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR5                                (0x000501D4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR6                                (0x000501D8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR7                                (0x000501DCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLAR                               (0x00050300U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLSR                               (0x00050304U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSSRR                               (0x00050308U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRCR                                (0x00050310U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRSR                                (0x00050314U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MIDR                            (0x00050D00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_CTR                             (0x00050D04U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_TCMTR                           (0x00050D08U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPUIR                           (0x00050D10U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPIDR                           (0x00050D14U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR0                            (0x00050D20U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR1                            (0x00050D24U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_DFR0                            (0x00050D28U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_AFR0                            (0x00050D2CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR0                           (0x00050D30U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR1                           (0x00050D34U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR2                           (0x00050D38U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR3                           (0x00050D3CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR0                           (0x00050D40U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR1                           (0x00050D44U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR2                           (0x00050D48U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR3                           (0x00050D4CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR4                           (0x00050D50U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR5                           (0x00050D54U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_ITCTRL                              (0x00050F00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMSET                            (0x00050FA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMCLR                            (0x00050FA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKACCESS                          (0x00050FB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKSTATUS                          (0x00050FB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_AUTHSTATUS                          (0x00050FB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVID                               (0x00050FC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVTYPE                             (0x00050FCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID4                              (0x00050FD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID0                              (0x00050FE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID1                              (0x00050FE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID2                              (0x00050FE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID3                              (0x00050FECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID0                               (0x00050FF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID1                               (0x00050FF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID2                               (0x00050FF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID3                               (0x00050FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDIDR                                (0x00052000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWFAR                                (0x00052018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGVCR                                 (0x0005201CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGECR                                 (0x00052024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCCR                               (0x00052028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRRX                               (0x00052080U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGITR                                 (0x00052084U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCR                                (0x00052088U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRTX                               (0x0005208CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDRCR                                (0x00052090U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR0                                (0x00052100U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR1                                (0x00052104U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR2                                (0x00052108U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR3                                (0x0005210CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR4                                (0x00052110U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR5                                (0x00052114U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR6                                (0x00052118U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR7                                (0x0005211CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR0                                (0x00052140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR1                                (0x00052144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR2                                (0x00052148U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR3                                (0x0005214CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR4                                (0x00052150U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR5                                (0x00052154U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR6                                (0x00052158U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR7                                (0x0005215CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR0                                (0x00052180U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR1                                (0x00052184U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR2                                (0x00052188U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR3                                (0x0005218CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR4                                (0x00052190U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR5                                (0x00052194U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR6                                (0x00052198U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR7                                (0x0005219CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR0                                (0x000521C0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR1                                (0x000521C4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR2                                (0x000521C8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR3                                (0x000521CCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR4                                (0x000521D0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR5                                (0x000521D4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR6                                (0x000521D8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR7                                (0x000521DCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLAR                               (0x00052300U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLSR                               (0x00052304U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSSRR                               (0x00052308U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRCR                                (0x00052310U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRSR                                (0x00052314U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MIDR                            (0x00052D00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_CTR                             (0x00052D04U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_TCMTR                           (0x00052D08U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPUIR                           (0x00052D10U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPIDR                           (0x00052D14U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR0                            (0x00052D20U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR1                            (0x00052D24U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_DFR0                            (0x00052D28U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_AFR0                            (0x00052D2CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR0                           (0x00052D30U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR1                           (0x00052D34U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR2                           (0x00052D38U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR3                           (0x00052D3CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR0                           (0x00052D40U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR1                           (0x00052D44U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR2                           (0x00052D48U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR3                           (0x00052D4CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR4                           (0x00052D50U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR5                           (0x00052D54U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_ITCTRL                              (0x00052F00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMSET                            (0x00052FA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMCLR                            (0x00052FA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKACCESS                          (0x00052FB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKSTATUS                          (0x00052FB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_AUTHSTATUS                          (0x00052FB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVID                               (0x00052FC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVTYPE                             (0x00052FCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID4                              (0x00052FD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID0                              (0x00052FE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID1                              (0x00052FE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID2                              (0x00052FE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID3                              (0x00052FECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID0                               (0x00052FF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID1                               (0x00052FF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID2                               (0x00052FF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID3                               (0x00052FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CONTROL                                (0x00058000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INTACK                                 (0x00058010U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPSET                                 (0x00058014U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPCLEAR                               (0x00058018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPPULSE                               (0x0005801CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN0                                  (0x00058020U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN1                                  (0x00058024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN2                                  (0x00058028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN3                                  (0x0005802CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN4                                  (0x00058030U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN5                                  (0x00058034U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN6                                  (0x00058038U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN7                                  (0x0005803CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN0                                 (0x000580A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN1                                 (0x000580A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN2                                 (0x000580A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN3                                 (0x000580ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN4                                 (0x000580B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN5                                 (0x000580B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN6                                 (0x000580B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN7                                 (0x000580BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGINSTATUS                           (0x00058130U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGOUTSTATUS                          (0x00058134U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHINSTATUS                             (0x00058138U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHOUTSTATUS                            (0x0005813CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_GATE                                   (0x00058140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ASICCTL                                (0x00058144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHINACK                              (0x00058EDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGINACK                            (0x00058EE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUT                                (0x00058EE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUT                              (0x00058EE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUTACK                             (0x00058EECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUTACK                           (0x00058EF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHIN                                 (0x00058EF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGIN                               (0x00058EF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCTRL                                 (0x00058F00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_SET                          (0x00058FA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_CLEAR                        (0x00058FA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_ACCESS_REGISTER                   (0x00058FB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_STATUS_REGISTER                   (0x00058FB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_AUTHENTICATION_STATUS                  (0x00058FB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_ID                              (0x00058FC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_TYPE_IDENTIFIER                 (0x00058FCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID4                          (0x00058FD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID5                          (0x00058FD4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID6                          (0x00058FD8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID7                          (0x00058FDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID0                          (0x00058FE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID1                          (0x00058FE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID2                          (0x00058FE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID3                          (0x00058FECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID0                          (0x00058FF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID1                          (0x00058FF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID2                          (0x00058FF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID3                          (0x00058FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CONTROL                                (0x00059000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INTACK                                 (0x00059010U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPSET                                 (0x00059014U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPCLEAR                               (0x00059018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPPULSE                               (0x0005901CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN0                                  (0x00059020U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN1                                  (0x00059024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN2                                  (0x00059028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN3                                  (0x0005902CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN4                                  (0x00059030U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN5                                  (0x00059034U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN6                                  (0x00059038U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN7                                  (0x0005903CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN0                                 (0x000590A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN1                                 (0x000590A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN2                                 (0x000590A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN3                                 (0x000590ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN4                                 (0x000590B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN5                                 (0x000590B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN6                                 (0x000590B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN7                                 (0x000590BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGINSTATUS                           (0x00059130U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGOUTSTATUS                          (0x00059134U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHINSTATUS                             (0x00059138U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHOUTSTATUS                            (0x0005913CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_GATE                                   (0x00059140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ASICCTL                                (0x00059144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHINACK                              (0x00059EDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGINACK                            (0x00059EE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUT                                (0x00059EE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUT                              (0x00059EE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUTACK                             (0x00059EECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUTACK                           (0x00059EF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHIN                                 (0x00059EF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGIN                               (0x00059EF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCTRL                                 (0x00059F00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_SET                          (0x00059FA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_CLEAR                        (0x00059FA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_ACCESS_REGISTER                   (0x00059FB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_STATUS_REGISTER                   (0x00059FB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_AUTHENTICATION_STATUS                  (0x00059FB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_ID                              (0x00059FC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_TYPE_IDENTIFIER                 (0x00059FCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID4                          (0x00059FD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID5                          (0x00059FD4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID6                          (0x00059FD8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID7                          (0x00059FDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID0                          (0x00059FE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID1                          (0x00059FE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID2                          (0x00059FE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID3                          (0x00059FECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID0                          (0x00059FF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID1                          (0x00059FF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID2                          (0x00059FF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID3                          (0x00059FFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CR                                     (0x0005C000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCR                                    (0x0005C004U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRIGGER                                (0x0005C008U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ASICCTLR                               (0x0005C00CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SR                                     (0x0005C010U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SCR                                    (0x0005C014U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TSSCR                                  (0x0005C018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR2                                  (0x0005C01CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TEEVR                                  (0x0005C020U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR1                                  (0x0005C024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFRR                                   (0x0005C028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFLR                                   (0x0005C02CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDEVR                                  (0x0005C030U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR1                                  (0x0005C034U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR2                                  (0x0005C038U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR3                                  (0x0005C03CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR1                                  (0x0005C040U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR2                                  (0x0005C044U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR3                                  (0x0005C048U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR4                                  (0x0005C04CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR5                                  (0x0005C050U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR6                                  (0x0005C054U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR7                                  (0x0005C058U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR8                                  (0x0005C05CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR9                                  (0x0005C060U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR10                                 (0x0005C064U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR11                                 (0x0005C068U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR12                                 (0x0005C06CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR13                                 (0x0005C070U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR14                                 (0x0005C074U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR15                                 (0x0005C078U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR16                                 (0x0005C07CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR1                                  (0x0005C080U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR2                                  (0x0005C084U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR3                                  (0x0005C088U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR4                                  (0x0005C08CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR5                                  (0x0005C090U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR6                                  (0x0005C094U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR7                                  (0x0005C098U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR8                                  (0x0005C09CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR9                                  (0x0005C0A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR10                                 (0x0005C0A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR11                                 (0x0005C0A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR12                                 (0x0005C0ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR13                                 (0x0005C0B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR14                                 (0x0005C0B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR15                                 (0x0005C0B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR16                                 (0x0005C0BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR1                                  (0x0005C0C0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR2                                  (0x0005C0C8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR3                                  (0x0005C0D0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR4                                  (0x0005C0D8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR5                                  (0x0005C0E0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR6                                  (0x0005C0E8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR7                                  (0x0005C0F0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR8                                  (0x0005C0F8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR1                                  (0x0005C100U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR2                                  (0x0005C108U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR3                                  (0x0005C110U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR4                                  (0x0005C118U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR5                                  (0x0005C120U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR6                                  (0x0005C128U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR7                                  (0x0005C130U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR8                                  (0x0005C138U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR1                              (0x0005C140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR2                              (0x0005C144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR3                              (0x0005C148U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR4                              (0x0005C14CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR1                                (0x0005C150U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR2                                (0x0005C154U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR3                                (0x0005C158U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR4                                (0x0005C15CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR1                             (0x0005C160U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR2                             (0x0005C164U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR3                             (0x0005C168U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR4                             (0x0005C16CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR1                                 (0x0005C170U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR2                                 (0x0005C174U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR3                                 (0x0005C178U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR4                                 (0x0005C17CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ12EVR                                (0x0005C180U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ21EVR                                (0x0005C184U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ23EVR                                (0x0005C188U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ31EVR                                (0x0005C18CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ32EVR                                (0x0005C190U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ13EVR                                (0x0005C194U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQR                                    (0x0005C19CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR1                             (0x0005C1A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR2                             (0x0005C1A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR3                             (0x0005C1A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR4                             (0x0005C1ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR1                                (0x0005C1B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR2                                (0x0005C1B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR3                                (0x0005C1B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCMR                                 (0x0005C1BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SYNCFR                                 (0x0005C1E0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_IDR                                    (0x0005C1E4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCER                                   (0x0005C1E8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTINSELR                              (0x0005C1ECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRACEIDR                               (0x0005C200U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PDSR                                   (0x0005C314U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITETMIF                                (0x0005CED8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCOUT                              (0x0005CEDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCIN                               (0x0005CEE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERACK                           (0x0005CEE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERREQ                           (0x0005CEE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBDATA0                             (0x0005CEECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR2                              (0x0005CEF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR1                              (0x0005CEF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR0                              (0x0005CEF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITCTRL                                 (0x0005CF00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMSET                               (0x0005CFA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMCLR                               (0x0005CFA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LAR                                    (0x0005CFB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LSR                                    (0x0005CFB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_AUTHSTATUS                             (0x0005CFB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVID                                  (0x0005CFC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVTYPE                                (0x0005CFCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR4                                  (0x0005CFD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR5                                  (0x0005CFD4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR6                                  (0x0005CFD8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR7                                  (0x0005CFDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR0                                  (0x0005CFE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR1                                  (0x0005CFE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR2                                  (0x0005CFE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR3                                  (0x0005CFECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR0                                  (0x0005CFF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR1                                  (0x0005CFF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR2                                  (0x0005CFF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR3                                  (0x0005CFFCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CR                                     (0x0005D000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCR                                    (0x0005D004U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRIGGER                                (0x0005D008U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ASICCTLR                               (0x0005D00CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SR                                     (0x0005D010U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SCR                                    (0x0005D014U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TSSCR                                  (0x0005D018U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR2                                  (0x0005D01CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TEEVR                                  (0x0005D020U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR1                                  (0x0005D024U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFRR                                   (0x0005D028U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFLR                                   (0x0005D02CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDEVR                                  (0x0005D030U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR1                                  (0x0005D034U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR2                                  (0x0005D038U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR3                                  (0x0005D03CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR1                                  (0x0005D040U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR2                                  (0x0005D044U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR3                                  (0x0005D048U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR4                                  (0x0005D04CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR5                                  (0x0005D050U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR6                                  (0x0005D054U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR7                                  (0x0005D058U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR8                                  (0x0005D05CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR9                                  (0x0005D060U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR10                                 (0x0005D064U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR11                                 (0x0005D068U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR12                                 (0x0005D06CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR13                                 (0x0005D070U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR14                                 (0x0005D074U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR15                                 (0x0005D078U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR16                                 (0x0005D07CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR1                                  (0x0005D080U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR2                                  (0x0005D084U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR3                                  (0x0005D088U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR4                                  (0x0005D08CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR5                                  (0x0005D090U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR6                                  (0x0005D094U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR7                                  (0x0005D098U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR8                                  (0x0005D09CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR9                                  (0x0005D0A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR10                                 (0x0005D0A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR11                                 (0x0005D0A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR12                                 (0x0005D0ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR13                                 (0x0005D0B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR14                                 (0x0005D0B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR15                                 (0x0005D0B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR16                                 (0x0005D0BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR1                                  (0x0005D0C0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR2                                  (0x0005D0C8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR3                                  (0x0005D0D0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR4                                  (0x0005D0D8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR5                                  (0x0005D0E0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR6                                  (0x0005D0E8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR7                                  (0x0005D0F0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR8                                  (0x0005D0F8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR1                                  (0x0005D100U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR2                                  (0x0005D108U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR3                                  (0x0005D110U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR4                                  (0x0005D118U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR5                                  (0x0005D120U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR6                                  (0x0005D128U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR7                                  (0x0005D130U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR8                                  (0x0005D138U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR1                              (0x0005D140U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR2                              (0x0005D144U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR3                              (0x0005D148U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR4                              (0x0005D14CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR1                                (0x0005D150U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR2                                (0x0005D154U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR3                                (0x0005D158U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR4                                (0x0005D15CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR1                             (0x0005D160U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR2                             (0x0005D164U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR3                             (0x0005D168U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR4                             (0x0005D16CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR1                                 (0x0005D170U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR2                                 (0x0005D174U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR3                                 (0x0005D178U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR4                                 (0x0005D17CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ12EVR                                (0x0005D180U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ21EVR                                (0x0005D184U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ23EVR                                (0x0005D188U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ31EVR                                (0x0005D18CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ32EVR                                (0x0005D190U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ13EVR                                (0x0005D194U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQR                                    (0x0005D19CU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR1                             (0x0005D1A0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR2                             (0x0005D1A4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR3                             (0x0005D1A8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR4                             (0x0005D1ACU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR1                                (0x0005D1B0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR2                                (0x0005D1B4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR3                                (0x0005D1B8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCMR                                 (0x0005D1BCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SYNCFR                                 (0x0005D1E0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_IDR                                    (0x0005D1E4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCER                                   (0x0005D1E8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTINSELR                              (0x0005D1ECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRACEIDR                               (0x0005D200U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PDSR                                   (0x0005D314U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITETMIF                                (0x0005DED8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCOUT                              (0x0005DEDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCIN                               (0x0005DEE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERACK                           (0x0005DEE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERREQ                           (0x0005DEE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBDATA0                             (0x0005DEECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR2                              (0x0005DEF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR1                              (0x0005DEF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR0                              (0x0005DEF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITCTRL                                 (0x0005DF00U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMSET                               (0x0005DFA0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMCLR                               (0x0005DFA4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LAR                                    (0x0005DFB0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LSR                                    (0x0005DFB4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_AUTHSTATUS                             (0x0005DFB8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVID                                  (0x0005DFC8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVTYPE                                (0x0005DFCCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR4                                  (0x0005DFD0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR5                                  (0x0005DFD4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR6                                  (0x0005DFD8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR7                                  (0x0005DFDCU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR0                                  (0x0005DFE0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR1                                  (0x0005DFE4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR2                                  (0x0005DFE8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR3                                  (0x0005DFECU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR0                                  (0x0005DFF0U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR1                                  (0x0005DFF4U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR2                                  (0x0005DFF8U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR3                                  (0x0005DFFCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* ONEMCU_APB_BASE */

#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_ONEMCU_APB_BASE_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_ONEMCU_APB_BASE_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_ONEMCU_APB_BASE_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_ONEMCU_APB_BASE_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_RESETVAL                               (0x00000000U)

/* ONEMCU_APB_BASE_END */

#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_END_ONEMCU_APB_BASE_END_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_END_ONEMCU_APB_BASE_END_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_END_ONEMCU_APB_BASE_END_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_END_ONEMCU_APB_BASE_END_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_APB_BASE_END_RESETVAL                           (0x00000000U)

/* ONEMCU_CTI_CONTROL */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CONTROL_ONEMCU_CTI_CONTROL_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CONTROL_ONEMCU_CTI_CONTROL_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CONTROL_ONEMCU_CTI_CONTROL_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CONTROL_ONEMCU_CTI_CONTROL_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CONTROL_RESETVAL                            (0x00000000U)

/* ONEMCU_CTI_INTACK */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INTACK_ONEMCU_CTI_INTACK_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INTACK_ONEMCU_CTI_INTACK_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INTACK_ONEMCU_CTI_INTACK_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INTACK_ONEMCU_CTI_INTACK_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INTACK_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_APPSET */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPSET_ONEMCU_CTI_APPSET_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPSET_ONEMCU_CTI_APPSET_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPSET_ONEMCU_CTI_APPSET_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPSET_ONEMCU_CTI_APPSET_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPSET_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_APPCLEAR */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPCLEAR_ONEMCU_CTI_APPCLEAR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPCLEAR_ONEMCU_CTI_APPCLEAR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPCLEAR_ONEMCU_CTI_APPCLEAR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPCLEAR_ONEMCU_CTI_APPCLEAR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPCLEAR_RESETVAL                           (0x00000000U)

/* ONEMCU_CTI_APPPULSE */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPPULSE_ONEMCU_CTI_APPPULSE_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPPULSE_ONEMCU_CTI_APPPULSE_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPPULSE_ONEMCU_CTI_APPPULSE_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPPULSE_ONEMCU_CTI_APPPULSE_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_APPPULSE_RESETVAL                           (0x00000000U)

/* ONEMCU_CTI_INEN0 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN0_ONEMCU_CTI_INEN0_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN0_ONEMCU_CTI_INEN0_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN0_ONEMCU_CTI_INEN0_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN0_ONEMCU_CTI_INEN0_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN0_RESETVAL                              (0x00000000U)

/* ONEMCU_CTI_INEN1 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN1_ONEMCU_CTI_INEN1_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN1_ONEMCU_CTI_INEN1_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN1_ONEMCU_CTI_INEN1_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN1_ONEMCU_CTI_INEN1_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN1_RESETVAL                              (0x00000000U)

/* ONEMCU_CTI_INEN2 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN2_ONEMCU_CTI_INEN2_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN2_ONEMCU_CTI_INEN2_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN2_ONEMCU_CTI_INEN2_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN2_ONEMCU_CTI_INEN2_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN2_RESETVAL                              (0x00000000U)

/* ONEMCU_CTI_INEN3 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN3_ONEMCU_CTI_INEN3_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN3_ONEMCU_CTI_INEN3_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN3_ONEMCU_CTI_INEN3_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN3_ONEMCU_CTI_INEN3_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN3_RESETVAL                              (0x00000000U)

/* ONEMCU_CTI_INEN4 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN4_ONEMCU_CTI_INEN4_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN4_ONEMCU_CTI_INEN4_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN4_ONEMCU_CTI_INEN4_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN4_ONEMCU_CTI_INEN4_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN4_RESETVAL                              (0x00000000U)

/* ONEMCU_CTI_INEN5 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN5_ONEMCU_CTI_INEN5_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN5_ONEMCU_CTI_INEN5_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN5_ONEMCU_CTI_INEN5_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN5_ONEMCU_CTI_INEN5_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN5_RESETVAL                              (0x00000000U)

/* ONEMCU_CTI_INEN6 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN6_ONEMCU_CTI_INEN6_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN6_ONEMCU_CTI_INEN6_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN6_ONEMCU_CTI_INEN6_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN6_ONEMCU_CTI_INEN6_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN6_RESETVAL                              (0x00000000U)

/* ONEMCU_CTI_INEN7 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN7_ONEMCU_CTI_INEN7_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN7_ONEMCU_CTI_INEN7_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN7_ONEMCU_CTI_INEN7_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN7_ONEMCU_CTI_INEN7_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_INEN7_RESETVAL                              (0x00000000U)

/* ONEMCU_CTI_OUTEN0 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN0_ONEMCU_CTI_OUTEN0_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN0_ONEMCU_CTI_OUTEN0_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN0_ONEMCU_CTI_OUTEN0_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN0_ONEMCU_CTI_OUTEN0_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN0_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_OUTEN1 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN1_ONEMCU_CTI_OUTEN1_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN1_ONEMCU_CTI_OUTEN1_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN1_ONEMCU_CTI_OUTEN1_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN1_ONEMCU_CTI_OUTEN1_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN1_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_OUTEN2 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN2_ONEMCU_CTI_OUTEN2_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN2_ONEMCU_CTI_OUTEN2_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN2_ONEMCU_CTI_OUTEN2_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN2_ONEMCU_CTI_OUTEN2_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN2_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_OUTEN3 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN3_ONEMCU_CTI_OUTEN3_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN3_ONEMCU_CTI_OUTEN3_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN3_ONEMCU_CTI_OUTEN3_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN3_ONEMCU_CTI_OUTEN3_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN3_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_OUTEN4 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN4_ONEMCU_CTI_OUTEN4_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN4_ONEMCU_CTI_OUTEN4_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN4_ONEMCU_CTI_OUTEN4_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN4_ONEMCU_CTI_OUTEN4_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN4_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_OUTEN5 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN5_ONEMCU_CTI_OUTEN5_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN5_ONEMCU_CTI_OUTEN5_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN5_ONEMCU_CTI_OUTEN5_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN5_ONEMCU_CTI_OUTEN5_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN5_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_OUTEN6 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN6_ONEMCU_CTI_OUTEN6_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN6_ONEMCU_CTI_OUTEN6_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN6_ONEMCU_CTI_OUTEN6_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN6_ONEMCU_CTI_OUTEN6_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN6_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_OUTEN7 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN7_ONEMCU_CTI_OUTEN7_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN7_ONEMCU_CTI_OUTEN7_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN7_ONEMCU_CTI_OUTEN7_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN7_ONEMCU_CTI_OUTEN7_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_OUTEN7_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_TRIGINSTATUS */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGINSTATUS_ONEMCU_CTI_TRIGINSTATUS_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGINSTATUS_ONEMCU_CTI_TRIGINSTATUS_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGINSTATUS_ONEMCU_CTI_TRIGINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGINSTATUS_ONEMCU_CTI_TRIGINSTATUS_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGINSTATUS_RESETVAL                       (0x00000000U)

/* ONEMCU_CTI_TRIGOUTSTATUS */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGOUTSTATUS_ONEMCU_CTI_TRIGOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGOUTSTATUS_ONEMCU_CTI_TRIGOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGOUTSTATUS_ONEMCU_CTI_TRIGOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGOUTSTATUS_ONEMCU_CTI_TRIGOUTSTATUS_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_TRIGOUTSTATUS_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_CHINSTATUS */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHINSTATUS_ONEMCU_CTI_CHINSTATUS_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHINSTATUS_ONEMCU_CTI_CHINSTATUS_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHINSTATUS_ONEMCU_CTI_CHINSTATUS_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHINSTATUS_ONEMCU_CTI_CHINSTATUS_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHINSTATUS_RESETVAL                         (0x00000000U)

/* ONEMCU_CTI_CHOUTSTATUS */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHOUTSTATUS_ONEMCU_CTI_CHOUTSTATUS_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHOUTSTATUS_ONEMCU_CTI_CHOUTSTATUS_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHOUTSTATUS_ONEMCU_CTI_CHOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHOUTSTATUS_ONEMCU_CTI_CHOUTSTATUS_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CHOUTSTATUS_RESETVAL                        (0x00000000U)

/* ONEMCU_CTI_GATE */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_GATE_ONEMCU_CTI_GATE_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_GATE_ONEMCU_CTI_GATE_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_GATE_ONEMCU_CTI_GATE_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_GATE_ONEMCU_CTI_GATE_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_GATE_RESETVAL                               (0x00000000U)

/* ONEMCU_CTI_ASICCTL */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ASICCTL_ONEMCU_CTI_ASICCTL_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ASICCTL_ONEMCU_CTI_ASICCTL_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ASICCTL_ONEMCU_CTI_ASICCTL_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ASICCTL_ONEMCU_CTI_ASICCTL_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ASICCTL_RESETVAL                            (0x00000000U)

/* ONEMCU_CTI_ITCHINACK */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHINACK_ONEMCU_CTI_ITCHINACK_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHINACK_ONEMCU_CTI_ITCHINACK_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHINACK_ONEMCU_CTI_ITCHINACK_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHINACK_ONEMCU_CTI_ITCHINACK_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHINACK_RESETVAL                          (0x00000000U)

/* ONEMCU_CTI_ITTRIGINACK */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGINACK_ONEMCU_CTI_ITTRIGINACK_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGINACK_ONEMCU_CTI_ITTRIGINACK_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGINACK_ONEMCU_CTI_ITTRIGINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGINACK_ONEMCU_CTI_ITTRIGINACK_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGINACK_RESETVAL                        (0x00000000U)

/* ONEMCU_CTI_ITCHOUT */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUT_ONEMCU_CTI_ITCHOUT_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUT_ONEMCU_CTI_ITCHOUT_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUT_ONEMCU_CTI_ITCHOUT_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUT_ONEMCU_CTI_ITCHOUT_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUT_RESETVAL                            (0x00000000U)

/* ONEMCU_CTI_ITTRIGOUT */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUT_ONEMCU_CTI_ITTRIGOUT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUT_ONEMCU_CTI_ITTRIGOUT_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUT_ONEMCU_CTI_ITTRIGOUT_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUT_ONEMCU_CTI_ITTRIGOUT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUT_RESETVAL                          (0x00000000U)

/* ONEMCU_CTI_ITCHOUTACK */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUTACK_ONEMCU_CTI_ITCHOUTACK_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUTACK_ONEMCU_CTI_ITCHOUTACK_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUTACK_ONEMCU_CTI_ITCHOUTACK_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUTACK_ONEMCU_CTI_ITCHOUTACK_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHOUTACK_RESETVAL                         (0x00000000U)

/* ONEMCU_CTI_ITTRIGOUTACK */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUTACK_ONEMCU_CTI_ITTRIGOUTACK_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUTACK_ONEMCU_CTI_ITTRIGOUTACK_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUTACK_ONEMCU_CTI_ITTRIGOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUTACK_ONEMCU_CTI_ITTRIGOUTACK_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGOUTACK_RESETVAL                       (0x00000000U)

/* ONEMCU_CTI_ITCHIN */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHIN_ONEMCU_CTI_ITCHIN_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHIN_ONEMCU_CTI_ITCHIN_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHIN_ONEMCU_CTI_ITCHIN_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHIN_ONEMCU_CTI_ITCHIN_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCHIN_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_ITTRIGIN */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGIN_ONEMCU_CTI_ITTRIGIN_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGIN_ONEMCU_CTI_ITTRIGIN_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGIN_ONEMCU_CTI_ITTRIGIN_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGIN_ONEMCU_CTI_ITTRIGIN_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITTRIGIN_RESETVAL                           (0x00000000U)

/* ONEMCU_CTI_ITCTRL */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCTRL_ONEMCU_CTI_ITCTRL_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCTRL_ONEMCU_CTI_ITCTRL_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCTRL_ONEMCU_CTI_ITCTRL_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCTRL_ONEMCU_CTI_ITCTRL_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_ITCTRL_RESETVAL                             (0x00000000U)

/* ONEMCU_CTI_CLAIM_TAG_SET */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_SET_ONEMCU_CTI_CLAIM_TAG_SET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_SET_ONEMCU_CTI_CLAIM_TAG_SET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_SET_ONEMCU_CTI_CLAIM_TAG_SET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_SET_ONEMCU_CTI_CLAIM_TAG_SET_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_SET_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_CLAIM_TAG_CLEAR */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_CLEAR_ONEMCU_CTI_CLAIM_TAG_CLEAR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_CLEAR_ONEMCU_CTI_CLAIM_TAG_CLEAR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_CLEAR_ONEMCU_CTI_CLAIM_TAG_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_CLEAR_ONEMCU_CTI_CLAIM_TAG_CLEAR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_CLAIM_TAG_CLEAR_RESETVAL                    (0x00000000U)

/* ONEMCU_CTI_LOCK_ACCESS_REGISTER */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_ACCESS_REGISTER_ONEMCU_CTI_LOCK_ACCESS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_ACCESS_REGISTER_ONEMCU_CTI_LOCK_ACCESS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_ACCESS_REGISTER_ONEMCU_CTI_LOCK_ACCESS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_ACCESS_REGISTER_ONEMCU_CTI_LOCK_ACCESS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_ACCESS_REGISTER_RESETVAL               (0x00000000U)

/* ONEMCU_CTI_LOCK_STATUS_REGISTER */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_STATUS_REGISTER_ONEMCU_CTI_LOCK_STATUS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_STATUS_REGISTER_ONEMCU_CTI_LOCK_STATUS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_STATUS_REGISTER_ONEMCU_CTI_LOCK_STATUS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_STATUS_REGISTER_ONEMCU_CTI_LOCK_STATUS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_LOCK_STATUS_REGISTER_RESETVAL               (0x00000000U)

/* ONEMCU_CTI_AUTHENTICATION_STATUS */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_AUTHENTICATION_STATUS_ONEMCU_CTI_AUTHENTICATION_STATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_AUTHENTICATION_STATUS_ONEMCU_CTI_AUTHENTICATION_STATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_AUTHENTICATION_STATUS_ONEMCU_CTI_AUTHENTICATION_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_AUTHENTICATION_STATUS_ONEMCU_CTI_AUTHENTICATION_STATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_AUTHENTICATION_STATUS_RESETVAL              (0x00000000U)

/* ONEMCU_CTI_DEVICE_ID */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_ID_ONEMCU_CTI_DEVICE_ID_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_ID_ONEMCU_CTI_DEVICE_ID_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_ID_ONEMCU_CTI_DEVICE_ID_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_ID_ONEMCU_CTI_DEVICE_ID_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_ID_RESETVAL                          (0x00000000U)

/* ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL             (0x00000000U)

/* ONEMCU_CTI_PERIPHERALID4 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID4_ONEMCU_CTI_PERIPHERALID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID4_ONEMCU_CTI_PERIPHERALID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID4_ONEMCU_CTI_PERIPHERALID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID4_ONEMCU_CTI_PERIPHERALID4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID4_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_PERIPHERALID5 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID5_ONEMCU_CTI_PERIPHERALID5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID5_ONEMCU_CTI_PERIPHERALID5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID5_ONEMCU_CTI_PERIPHERALID5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID5_ONEMCU_CTI_PERIPHERALID5_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID5_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_PERIPHERALID6 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID6_ONEMCU_CTI_PERIPHERALID6_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID6_ONEMCU_CTI_PERIPHERALID6_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID6_ONEMCU_CTI_PERIPHERALID6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID6_ONEMCU_CTI_PERIPHERALID6_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID6_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_PERIPHERALID7 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID7_ONEMCU_CTI_PERIPHERALID7_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID7_ONEMCU_CTI_PERIPHERALID7_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID7_ONEMCU_CTI_PERIPHERALID7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID7_ONEMCU_CTI_PERIPHERALID7_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID7_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_PERIPHERALID0 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID0_ONEMCU_CTI_PERIPHERALID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID0_ONEMCU_CTI_PERIPHERALID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID0_ONEMCU_CTI_PERIPHERALID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID0_ONEMCU_CTI_PERIPHERALID0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID0_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_PERIPHERALID1 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID1_ONEMCU_CTI_PERIPHERALID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID1_ONEMCU_CTI_PERIPHERALID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID1_ONEMCU_CTI_PERIPHERALID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID1_ONEMCU_CTI_PERIPHERALID1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID1_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_PERIPHERALID2 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID2_ONEMCU_CTI_PERIPHERALID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID2_ONEMCU_CTI_PERIPHERALID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID2_ONEMCU_CTI_PERIPHERALID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID2_ONEMCU_CTI_PERIPHERALID2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID2_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_PERIPHERALID3 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID3_ONEMCU_CTI_PERIPHERALID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID3_ONEMCU_CTI_PERIPHERALID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID3_ONEMCU_CTI_PERIPHERALID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID3_ONEMCU_CTI_PERIPHERALID3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_PERIPHERALID3_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_COMPONENT_ID0 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID0_ONEMCU_CTI_COMPONENT_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID0_ONEMCU_CTI_COMPONENT_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID0_ONEMCU_CTI_COMPONENT_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID0_ONEMCU_CTI_COMPONENT_ID0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID0_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_COMPONENT_ID1 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID1_ONEMCU_CTI_COMPONENT_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID1_ONEMCU_CTI_COMPONENT_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID1_ONEMCU_CTI_COMPONENT_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID1_ONEMCU_CTI_COMPONENT_ID1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID1_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_COMPONENT_ID2 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID2_ONEMCU_CTI_COMPONENT_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID2_ONEMCU_CTI_COMPONENT_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID2_ONEMCU_CTI_COMPONENT_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID2_ONEMCU_CTI_COMPONENT_ID2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID2_RESETVAL                      (0x00000000U)

/* ONEMCU_CTI_COMPONENT_ID3 */

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID3_ONEMCU_CTI_COMPONENT_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID3_ONEMCU_CTI_COMPONENT_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID3_ONEMCU_CTI_COMPONENT_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID3_ONEMCU_CTI_COMPONENT_ID3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_CTI_COMPONENT_ID3_RESETVAL                      (0x00000000U)

/* ONEMCU_TPIU_SPORTSZ */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_SPORTSZ_ONEMCU_TPIU_SPORTSZ_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_SPORTSZ_ONEMCU_TPIU_SPORTSZ_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_SPORTSZ_ONEMCU_TPIU_SPORTSZ_RESETVAL       (0x0FFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_SPORTSZ_ONEMCU_TPIU_SPORTSZ_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_SPORTSZ_RESETVAL                           (0x0FFFFFFFU)

/* ONEMCU_TPIU_CPORTSZ */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CPORTSZ_ONEMCU_TPIU_CPORTSZ_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CPORTSZ_ONEMCU_TPIU_CPORTSZ_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CPORTSZ_ONEMCU_TPIU_CPORTSZ_RESETVAL       (0x00000001U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CPORTSZ_ONEMCU_TPIU_CPORTSZ_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CPORTSZ_RESETVAL                           (0x00000001U)

/* ONEMCU_TPIU_STRIGM */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STRIGM_ONEMCU_TPIU_STRIGM_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STRIGM_ONEMCU_TPIU_STRIGM_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STRIGM_ONEMCU_TPIU_STRIGM_RESETVAL         (0x0000011FU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STRIGM_ONEMCU_TPIU_STRIGM_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STRIGM_RESETVAL                            (0x0000011FU)

/* ONEMCU_TPIU_TRIGCNT */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGCNT_ONEMCU_TPIU_TRIGCNT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGCNT_ONEMCU_TPIU_TRIGCNT_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGCNT_ONEMCU_TPIU_TRIGCNT_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGCNT_ONEMCU_TPIU_TRIGCNT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGCNT_RESETVAL                           (0x00000000U)

/* ONEMCU_TPIU_TRIGMUL */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGMUL_ONEMCU_TPIU_TRIGMUL_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGMUL_ONEMCU_TPIU_TRIGMUL_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGMUL_ONEMCU_TPIU_TRIGMUL_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGMUL_ONEMCU_TPIU_TRIGMUL_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TRIGMUL_RESETVAL                           (0x00000000U)

/* ONEMCU_TPIU_STSTPTRN */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STSTPTRN_ONEMCU_TPIU_STSTPTRN_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STSTPTRN_ONEMCU_TPIU_STSTPTRN_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STSTPTRN_ONEMCU_TPIU_STSTPTRN_RESETVAL     (0x0003000FU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STSTPTRN_ONEMCU_TPIU_STSTPTRN_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_STSTPTRN_RESETVAL                          (0x0003000FU)

/* ONEMCU_TPIU_CTSTPTRN */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CTSTPTRN_ONEMCU_TPIU_CTSTPTRN_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CTSTPTRN_ONEMCU_TPIU_CTSTPTRN_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CTSTPTRN_ONEMCU_TPIU_CTSTPTRN_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CTSTPTRN_ONEMCU_TPIU_CTSTPTRN_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CTSTPTRN_RESETVAL                          (0x00000000U)

/* ONEMCU_TPIU_TPRCNTR */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TPRCNTR_ONEMCU_TPIU_TPRCNTR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TPRCNTR_ONEMCU_TPIU_TPRCNTR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TPRCNTR_ONEMCU_TPIU_TPRCNTR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TPRCNTR_ONEMCU_TPIU_TPRCNTR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_TPRCNTR_RESETVAL                           (0x00000000U)

/* ONEMCU_TPIU_FFSTS */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFSTS_ONEMCU_TPIU_FFSTS_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFSTS_ONEMCU_TPIU_FFSTS_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFSTS_ONEMCU_TPIU_FFSTS_RESETVAL           (0x00000002U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFSTS_ONEMCU_TPIU_FFSTS_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFSTS_RESETVAL                             (0x00000002U)

/* ONEMCU_TPIU_FFCTRL */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFCTRL_ONEMCU_TPIU_FFCTRL_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFCTRL_ONEMCU_TPIU_FFCTRL_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFCTRL_ONEMCU_TPIU_FFCTRL_RESETVAL         (0x00001000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFCTRL_ONEMCU_TPIU_FFCTRL_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FFCTRL_RESETVAL                            (0x00001000U)

/* ONEMCU_TPIU_FSCNTR */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FSCNTR_ONEMCU_TPIU_FSCNTR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FSCNTR_ONEMCU_TPIU_FSCNTR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FSCNTR_ONEMCU_TPIU_FSCNTR_RESETVAL         (0x00000040U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FSCNTR_ONEMCU_TPIU_FSCNTR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_FSCNTR_RESETVAL                            (0x00000040U)

/* ONEMCU_TPIU_EXCTLIN */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLIN_ONEMCU_TPIU_EXCTLIN_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLIN_ONEMCU_TPIU_EXCTLIN_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLIN_ONEMCU_TPIU_EXCTLIN_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLIN_ONEMCU_TPIU_EXCTLIN_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLIN_RESETVAL                           (0x00000000U)

/* ONEMCU_TPIU_EXCTLOUT */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLOUT_ONEMCU_TPIU_EXCTLOUT_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLOUT_ONEMCU_TPIU_EXCTLOUT_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLOUT_ONEMCU_TPIU_EXCTLOUT_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLOUT_ONEMCU_TPIU_EXCTLOUT_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_EXCTLOUT_RESETVAL                          (0x00000000U)

/* ONEMCU_TPIU_ITTRFLINACK */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLINACK_ONEMCU_TPIU_ITTRFLINACK_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLINACK_ONEMCU_TPIU_ITTRFLINACK_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLINACK_ONEMCU_TPIU_ITTRFLINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLINACK_ONEMCU_TPIU_ITTRFLINACK_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLINACK_RESETVAL                       (0x00000000U)

/* ONEMCU_TPIU_ITTRFLIN */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLIN_ONEMCU_TPIU_ITTRFLIN_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLIN_ONEMCU_TPIU_ITTRFLIN_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLIN_ONEMCU_TPIU_ITTRFLIN_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLIN_ONEMCU_TPIU_ITTRFLIN_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITTRFLIN_RESETVAL                          (0x00000000U)

/* ONEMCU_TPIU_ITATBDATA0 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBDATA0_ONEMCU_TPIU_ITATBDATA0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBDATA0_ONEMCU_TPIU_ITATBDATA0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBDATA0_ONEMCU_TPIU_ITATBDATA0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBDATA0_ONEMCU_TPIU_ITATBDATA0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBDATA0_RESETVAL                        (0x00000000U)

/* ONEMCU_TPIU_ITATBCTR2 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR2_ONEMCU_TPIU_ITATBCTR2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR2_ONEMCU_TPIU_ITATBCTR2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR2_ONEMCU_TPIU_ITATBCTR2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR2_ONEMCU_TPIU_ITATBCTR2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR2_RESETVAL                         (0x00000000U)

/* ONEMCU_TPIU_ITATBCTR1 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR1_ONEMCU_TPIU_ITATBCTR1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR1_ONEMCU_TPIU_ITATBCTR1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR1_ONEMCU_TPIU_ITATBCTR1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR1_ONEMCU_TPIU_ITATBCTR1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR1_RESETVAL                         (0x00000000U)

/* ONEMCU_TPIU_ITATBCTR0 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR0_ONEMCU_TPIU_ITATBCTR0_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR0_ONEMCU_TPIU_ITATBCTR0_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR0_ONEMCU_TPIU_ITATBCTR0_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR0_ONEMCU_TPIU_ITATBCTR0_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITATBCTR0_RESETVAL                         (0x00000000U)

/* ONEMCU_TPIU_ITCTRL */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITCTRL_ONEMCU_TPIU_ITCTRL_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITCTRL_ONEMCU_TPIU_ITCTRL_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITCTRL_ONEMCU_TPIU_ITCTRL_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITCTRL_ONEMCU_TPIU_ITCTRL_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_ITCTRL_RESETVAL                            (0x00000000U)

/* ONEMCU_TPIU_CLAIMSET */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMSET_ONEMCU_TPIU_CLAIMSET_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMSET_ONEMCU_TPIU_CLAIMSET_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMSET_ONEMCU_TPIU_CLAIMSET_RESETVAL     (0x0000000FU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMSET_ONEMCU_TPIU_CLAIMSET_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMSET_RESETVAL                          (0x0000000FU)

/* ONEMCU_TPIU_CLAIMCLR */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMCLR_ONEMCU_TPIU_CLAIMCLR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMCLR_ONEMCU_TPIU_CLAIMCLR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMCLR_ONEMCU_TPIU_CLAIMCLR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMCLR_ONEMCU_TPIU_CLAIMCLR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CLAIMCLR_RESETVAL                          (0x00000000U)

/* ONEMCU_TPIU_LAR */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LAR_ONEMCU_TPIU_LAR_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LAR_ONEMCU_TPIU_LAR_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LAR_ONEMCU_TPIU_LAR_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LAR_ONEMCU_TPIU_LAR_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LAR_RESETVAL                               (0x00000000U)

/* ONEMCU_TPIU_LSR */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LSR_ONEMCU_TPIU_LSR_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LSR_ONEMCU_TPIU_LSR_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LSR_ONEMCU_TPIU_LSR_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LSR_ONEMCU_TPIU_LSR_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_LSR_RESETVAL                               (0x00000000U)

/* ONEMCU_TPIU_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_AUTHSTATUS_ONEMCU_TPIU_AUTHSTATUS_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_AUTHSTATUS_ONEMCU_TPIU_AUTHSTATUS_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_AUTHSTATUS_ONEMCU_TPIU_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_AUTHSTATUS_ONEMCU_TPIU_AUTHSTATUS_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_AUTHSTATUS_RESETVAL                        (0x00000000U)

/* ONEMCU_TPIU_DEVID */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVID_ONEMCU_TPIU_DEVID_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVID_ONEMCU_TPIU_DEVID_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVID_ONEMCU_TPIU_DEVID_RESETVAL           (0x000000A0U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVID_ONEMCU_TPIU_DEVID_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVID_RESETVAL                             (0x000000A0U)

/* ONEMCU_TPIU_DEVTYPE */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVTYPE_ONEMCU_TPIU_DEVTYPE_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVTYPE_ONEMCU_TPIU_DEVTYPE_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVTYPE_ONEMCU_TPIU_DEVTYPE_RESETVAL       (0x00000011U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVTYPE_ONEMCU_TPIU_DEVTYPE_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_DEVTYPE_RESETVAL                           (0x00000011U)

/* ONEMCU_TPIU_PIDR4 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR4_ONEMCU_TPIU_PIDR4_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR4_ONEMCU_TPIU_PIDR4_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR4_ONEMCU_TPIU_PIDR4_RESETVAL           (0x00000004U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR4_ONEMCU_TPIU_PIDR4_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR4_RESETVAL                             (0x00000004U)

/* ONEMCU_TPIU_PIDR5 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR5_ONEMCU_TPIU_PIDR5_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR5_ONEMCU_TPIU_PIDR5_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR5_ONEMCU_TPIU_PIDR5_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR5_ONEMCU_TPIU_PIDR5_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR5_RESETVAL                             (0x00000000U)

/* ONEMCU_TPIU_PIDR6 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR6_ONEMCU_TPIU_PIDR6_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR6_ONEMCU_TPIU_PIDR6_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR6_ONEMCU_TPIU_PIDR6_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR6_ONEMCU_TPIU_PIDR6_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR6_RESETVAL                             (0x00000000U)

/* ONEMCU_TPIU_PIDR7 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR7_ONEMCU_TPIU_PIDR7_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR7_ONEMCU_TPIU_PIDR7_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR7_ONEMCU_TPIU_PIDR7_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR7_ONEMCU_TPIU_PIDR7_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR7_RESETVAL                             (0x00000000U)

/* ONEMCU_TPIU_PIDR0 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR0_ONEMCU_TPIU_PIDR0_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR0_ONEMCU_TPIU_PIDR0_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR0_ONEMCU_TPIU_PIDR0_RESETVAL           (0x00000012U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR0_ONEMCU_TPIU_PIDR0_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR0_RESETVAL                             (0x00000012U)

/* ONEMCU_TPIU_PIDR1 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR1_ONEMCU_TPIU_PIDR1_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR1_ONEMCU_TPIU_PIDR1_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR1_ONEMCU_TPIU_PIDR1_RESETVAL           (0x000000B9U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR1_ONEMCU_TPIU_PIDR1_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR1_RESETVAL                             (0x000000B9U)

/* ONEMCU_TPIU_PIDR2 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR2_ONEMCU_TPIU_PIDR2_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR2_ONEMCU_TPIU_PIDR2_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR2_ONEMCU_TPIU_PIDR2_RESETVAL           (0x0000003BU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR2_ONEMCU_TPIU_PIDR2_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR2_RESETVAL                             (0x0000003BU)

/* ONEMCU_TPIU_PIDR3 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR3_ONEMCU_TPIU_PIDR3_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR3_ONEMCU_TPIU_PIDR3_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR3_ONEMCU_TPIU_PIDR3_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR3_ONEMCU_TPIU_PIDR3_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_PIDR3_RESETVAL                             (0x00000000U)

/* ONEMCU_TPIU_CIDR0 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR0_ONEMCU_TPIU_CIDR0_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR0_ONEMCU_TPIU_CIDR0_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR0_ONEMCU_TPIU_CIDR0_RESETVAL           (0x0000000DU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR0_ONEMCU_TPIU_CIDR0_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR0_RESETVAL                             (0x0000000DU)

/* ONEMCU_TPIU_CIDR1 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR1_ONEMCU_TPIU_CIDR1_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR1_ONEMCU_TPIU_CIDR1_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR1_ONEMCU_TPIU_CIDR1_RESETVAL           (0x00000090U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR1_ONEMCU_TPIU_CIDR1_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR1_RESETVAL                             (0x00000090U)

/* ONEMCU_TPIU_CIDR2 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR2_ONEMCU_TPIU_CIDR2_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR2_ONEMCU_TPIU_CIDR2_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR2_ONEMCU_TPIU_CIDR2_RESETVAL           (0x00000005U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR2_ONEMCU_TPIU_CIDR2_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR2_RESETVAL                             (0x00000005U)

/* ONEMCU_TPIU_CIDR3 */

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR3_ONEMCU_TPIU_CIDR3_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR3_ONEMCU_TPIU_CIDR3_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR3_ONEMCU_TPIU_CIDR3_RESETVAL           (0x000000B1U)
#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR3_ONEMCU_TPIU_CIDR3_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ONEMCU_TPIU_CIDR3_RESETVAL                             (0x000000B1U)

/* APB_EXT_PORT0_ROM */


/* ATB_REPLICATOR_IDFILTER0 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER0_ATB_REPLICATOR_IDFILTER0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER0_ATB_REPLICATOR_IDFILTER0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER0_ATB_REPLICATOR_IDFILTER0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER0_ATB_REPLICATOR_IDFILTER0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER0_RESETVAL                      (0x00000000U)

/* ATB_REPLICATOR_IDFILTER1 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER1_ATB_REPLICATOR_IDFILTER1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER1_ATB_REPLICATOR_IDFILTER1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER1_ATB_REPLICATOR_IDFILTER1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER1_ATB_REPLICATOR_IDFILTER1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_IDFILTER1_RESETVAL                      (0x00000000U)

/* ATB_REPLICATOR_ITATBCTR1 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR1_ATB_REPLICATOR_ITATBCTR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR1_ATB_REPLICATOR_ITATBCTR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR1_ATB_REPLICATOR_ITATBCTR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR1_ATB_REPLICATOR_ITATBCTR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR1_RESETVAL                      (0x00000000U)

/* ATB_REPLICATOR_ITATBCTR0 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR0_ATB_REPLICATOR_ITATBCTR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR0_ATB_REPLICATOR_ITATBCTR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR0_ATB_REPLICATOR_ITATBCTR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR0_ATB_REPLICATOR_ITATBCTR0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITATBCTR0_RESETVAL                      (0x00000000U)

/* ATB_REPLICATOR_ITCTRL */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITCTRL_ATB_REPLICATOR_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITCTRL_ATB_REPLICATOR_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITCTRL_ATB_REPLICATOR_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITCTRL_ATB_REPLICATOR_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_ITCTRL_RESETVAL                         (0x00000000U)

/* ATB_REPLICATOR_CLAIMSET */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMSET_ATB_REPLICATOR_CLAIMSET_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMSET_ATB_REPLICATOR_CLAIMSET_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMSET_ATB_REPLICATOR_CLAIMSET_RESETVAL (0x0000000FU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMSET_ATB_REPLICATOR_CLAIMSET_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMSET_RESETVAL                       (0x0000000FU)

/* ATB_REPLICATOR_CLAIMCLR */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMCLR_ATB_REPLICATOR_CLAIMCLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMCLR_ATB_REPLICATOR_CLAIMCLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMCLR_ATB_REPLICATOR_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMCLR_ATB_REPLICATOR_CLAIMCLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CLAIMCLR_RESETVAL                       (0x00000000U)

/* ATB_REPLICATOR_LAR */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LAR_ATB_REPLICATOR_LAR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LAR_ATB_REPLICATOR_LAR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LAR_ATB_REPLICATOR_LAR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LAR_ATB_REPLICATOR_LAR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LAR_RESETVAL                            (0x00000000U)

/* ATB_REPLICATOR_LSR */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LSR_ATB_REPLICATOR_LSR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LSR_ATB_REPLICATOR_LSR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LSR_ATB_REPLICATOR_LSR_RESETVAL         (0x00000003U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LSR_ATB_REPLICATOR_LSR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_LSR_RESETVAL                            (0x00000003U)

/* ATB_REPLICATOR_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_AUTHSTATUS_ATB_REPLICATOR_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_AUTHSTATUS_ATB_REPLICATOR_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_AUTHSTATUS_ATB_REPLICATOR_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_AUTHSTATUS_ATB_REPLICATOR_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_AUTHSTATUS_RESETVAL                     (0x00000000U)

/* ATB_REPLICATOR_DEVID */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVID_ATB_REPLICATOR_DEVID_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVID_ATB_REPLICATOR_DEVID_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVID_ATB_REPLICATOR_DEVID_RESETVAL     (0x00000002U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVID_ATB_REPLICATOR_DEVID_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVID_RESETVAL                          (0x00000002U)

/* ATB_REPLICATOR_DEVTYPE */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVTYPE_ATB_REPLICATOR_DEVTYPE_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVTYPE_ATB_REPLICATOR_DEVTYPE_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVTYPE_ATB_REPLICATOR_DEVTYPE_RESETVAL (0x00000022U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVTYPE_ATB_REPLICATOR_DEVTYPE_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_DEVTYPE_RESETVAL                        (0x00000022U)

/* ATB_REPLICATOR_PIDR4 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR4_ATB_REPLICATOR_PIDR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR4_ATB_REPLICATOR_PIDR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR4_ATB_REPLICATOR_PIDR4_RESETVAL     (0x00000004U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR4_ATB_REPLICATOR_PIDR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR4_RESETVAL                          (0x00000004U)

/* ATB_REPLICATOR_RESERVED1 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED1_ATB_REPLICATOR_RESERVED1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED1_ATB_REPLICATOR_RESERVED1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED1_ATB_REPLICATOR_RESERVED1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED1_ATB_REPLICATOR_RESERVED1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED1_RESETVAL                      (0x00000000U)

/* ATB_REPLICATOR_RESERVED2 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED2_ATB_REPLICATOR_RESERVED2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED2_ATB_REPLICATOR_RESERVED2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED2_ATB_REPLICATOR_RESERVED2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED2_ATB_REPLICATOR_RESERVED2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED2_RESETVAL                      (0x00000000U)

/* ATB_REPLICATOR_RESERVED3 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED3_ATB_REPLICATOR_RESERVED3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED3_ATB_REPLICATOR_RESERVED3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED3_ATB_REPLICATOR_RESERVED3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED3_ATB_REPLICATOR_RESERVED3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_RESERVED3_RESETVAL                      (0x00000000U)

/* ATB_REPLICATOR_PIDR0 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR0_ATB_REPLICATOR_PIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR0_ATB_REPLICATOR_PIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR0_ATB_REPLICATOR_PIDR0_RESETVAL     (0x00000009U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR0_ATB_REPLICATOR_PIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR0_RESETVAL                          (0x00000009U)

/* ATB_REPLICATOR_PIDR1 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR1_ATB_REPLICATOR_PIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR1_ATB_REPLICATOR_PIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR1_ATB_REPLICATOR_PIDR1_RESETVAL     (0x000000B9U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR1_ATB_REPLICATOR_PIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR1_RESETVAL                          (0x000000B9U)

/* ATB_REPLICATOR_PIDR2 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR2_ATB_REPLICATOR_PIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR2_ATB_REPLICATOR_PIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR2_ATB_REPLICATOR_PIDR2_RESETVAL     (0x0000001BU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR2_ATB_REPLICATOR_PIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR2_RESETVAL                          (0x0000001BU)

/* ATB_REPLICATOR_PIDR3 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR3_ATB_REPLICATOR_PIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR3_ATB_REPLICATOR_PIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR3_ATB_REPLICATOR_PIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR3_ATB_REPLICATOR_PIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_PIDR3_RESETVAL                          (0x00000000U)

/* ATB_REPLICATOR_CIDR0 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR0_ATB_REPLICATOR_CIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR0_ATB_REPLICATOR_CIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR0_ATB_REPLICATOR_CIDR0_RESETVAL     (0x0000000DU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR0_ATB_REPLICATOR_CIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR0_RESETVAL                          (0x0000000DU)

/* ATB_REPLICATOR_CIDR1 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR1_ATB_REPLICATOR_CIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR1_ATB_REPLICATOR_CIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR1_ATB_REPLICATOR_CIDR1_RESETVAL     (0x00000090U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR1_ATB_REPLICATOR_CIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR1_RESETVAL                          (0x00000090U)

/* ATB_REPLICATOR_CIDR2 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR2_ATB_REPLICATOR_CIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR2_ATB_REPLICATOR_CIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR2_ATB_REPLICATOR_CIDR2_RESETVAL     (0x00000005U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR2_ATB_REPLICATOR_CIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR2_RESETVAL                          (0x00000005U)

/* ATB_REPLICATOR_CIDR3 */

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR3_ATB_REPLICATOR_CIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR3_ATB_REPLICATOR_CIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR3_ATB_REPLICATOR_CIDR3_RESETVAL     (0x000000B1U)
#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR3_ATB_REPLICATOR_CIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ATB_REPLICATOR_CIDR3_RESETVAL                          (0x000000B1U)

/* ETB_RDP */

#define CSL_MSS_DEBUGSS_ETB_RDP_ETB_RDP_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_RDP_ETB_RDP_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RDP_ETB_RDP_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RDP_ETB_RDP_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_RDP_RESETVAL                                       (0x00000000U)

/* ETB_STS */

#define CSL_MSS_DEBUGSS_ETB_STS_ETB_STS_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_STS_ETB_STS_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_STS_ETB_STS_RESETVAL                               (0x00000008U)
#define CSL_MSS_DEBUGSS_ETB_STS_ETB_STS_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_STS_RESETVAL                                       (0x00000008U)

/* ETB_RRD */

#define CSL_MSS_DEBUGSS_ETB_RRD_ETB_RRD_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_RRD_ETB_RRD_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RRD_ETB_RRD_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RRD_ETB_RRD_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_RRD_RESETVAL                                       (0x00000000U)

/* ETB_RRP */

#define CSL_MSS_DEBUGSS_ETB_RRP_ETB_RRP_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_RRP_ETB_RRP_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RRP_ETB_RRP_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RRP_ETB_RRP_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_RRP_RESETVAL                                       (0x00000000U)

/* ETB_RWP */

#define CSL_MSS_DEBUGSS_ETB_RWP_ETB_RWP_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_RWP_ETB_RWP_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RWP_ETB_RWP_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RWP_ETB_RWP_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_RWP_RESETVAL                                       (0x00000000U)

/* ETB_TRG */

#define CSL_MSS_DEBUGSS_ETB_TRG_ETB_TRG_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_TRG_ETB_TRG_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_TRG_ETB_TRG_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_TRG_ETB_TRG_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_TRG_RESETVAL                                       (0x00000000U)

/* ETB_CTL */

#define CSL_MSS_DEBUGSS_ETB_CTL_ETB_CTL_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_CTL_ETB_CTL_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CTL_ETB_CTL_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CTL_ETB_CTL_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_CTL_RESETVAL                                       (0x00000000U)

/* ETB_RWD */

#define CSL_MSS_DEBUGSS_ETB_RWD_ETB_RWD_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_RWD_ETB_RWD_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RWD_ETB_RWD_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RWD_ETB_RWD_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_RWD_RESETVAL                                       (0x00000000U)

/* ETB_FFSR */

#define CSL_MSS_DEBUGSS_ETB_FFSR_ETB_FFSR_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_FFSR_ETB_FFSR_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_FFSR_ETB_FFSR_RESETVAL                             (0x00000002U)
#define CSL_MSS_DEBUGSS_ETB_FFSR_ETB_FFSR_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_FFSR_RESETVAL                                      (0x00000002U)

/* ETB_FFCR */

#define CSL_MSS_DEBUGSS_ETB_FFCR_ETB_FFCR_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_FFCR_ETB_FFCR_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_FFCR_ETB_FFCR_RESETVAL                             (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_FFCR_ETB_FFCR_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_FFCR_RESETVAL                                      (0x00000000U)

/* ETB_ITMISCOP0 */

#define CSL_MSS_DEBUGSS_ETB_ITMISCOP0_ETB_ITMISCOP0_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_ITMISCOP0_ETB_ITMISCOP0_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITMISCOP0_ETB_ITMISCOP0_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITMISCOP0_ETB_ITMISCOP0_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_ITMISCOP0_RESETVAL                                 (0x00000000U)

/* ETB_ITTRFLINACK */

#define CSL_MSS_DEBUGSS_ETB_ITTRFLINACK_ETB_ITTRFLINACK_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_ITTRFLINACK_ETB_ITTRFLINACK_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITTRFLINACK_ETB_ITTRFLINACK_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITTRFLINACK_ETB_ITTRFLINACK_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_ITTRFLINACK_RESETVAL                               (0x00000000U)

/* ETB_ITTRFLIN */

#define CSL_MSS_DEBUGSS_ETB_ITTRFLIN_ETB_ITTRFLIN_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_ITTRFLIN_ETB_ITTRFLIN_SHIFT                        (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITTRFLIN_ETB_ITTRFLIN_RESETVAL                     (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITTRFLIN_ETB_ITTRFLIN_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_ITTRFLIN_RESETVAL                                  (0x00000000U)

/* ETB_ITATBDATA0 */

#define CSL_MSS_DEBUGSS_ETB_ITATBDATA0_ETB_ITATBDATA0_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_ITATBDATA0_ETB_ITATBDATA0_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITATBDATA0_ETB_ITATBDATA0_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITATBDATA0_ETB_ITATBDATA0_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_ITATBDATA0_RESETVAL                                (0x00000000U)

/* ETB_ITATBCTR2 */

#define CSL_MSS_DEBUGSS_ETB_ITATBCTR2_ETB_ITATBCTR2_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR2_ETB_ITATBCTR2_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR2_ETB_ITATBCTR2_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR2_ETB_ITATBCTR2_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_ITATBCTR2_RESETVAL                                 (0x00000000U)

/* ETB_ITATBCTR1 */

#define CSL_MSS_DEBUGSS_ETB_ITATBCTR1_ETB_ITATBCTR1_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR1_ETB_ITATBCTR1_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR1_ETB_ITATBCTR1_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR1_ETB_ITATBCTR1_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_ITATBCTR1_RESETVAL                                 (0x00000000U)

/* ETB_ITATBCTR0 */

#define CSL_MSS_DEBUGSS_ETB_ITATBCTR0_ETB_ITATBCTR0_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR0_ETB_ITATBCTR0_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR0_ETB_ITATBCTR0_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITATBCTR0_ETB_ITATBCTR0_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_ITATBCTR0_RESETVAL                                 (0x00000000U)

/* ETB_ITCTRL */

#define CSL_MSS_DEBUGSS_ETB_ITCTRL_ETB_ITCTRL_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_ITCTRL_ETB_ITCTRL_SHIFT                            (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITCTRL_ETB_ITCTRL_RESETVAL                         (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_ITCTRL_ETB_ITCTRL_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_ITCTRL_RESETVAL                                    (0x00000000U)

/* ETB_CLAIMSET */

#define CSL_MSS_DEBUGSS_ETB_CLAIMSET_ETB_CLAIMSET_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_CLAIMSET_ETB_CLAIMSET_SHIFT                        (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CLAIMSET_ETB_CLAIMSET_RESETVAL                     (0x0000000FU)
#define CSL_MSS_DEBUGSS_ETB_CLAIMSET_ETB_CLAIMSET_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_CLAIMSET_RESETVAL                                  (0x0000000FU)

/* ETB_CLAIMCLR */

#define CSL_MSS_DEBUGSS_ETB_CLAIMCLR_ETB_CLAIMCLR_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_CLAIMCLR_ETB_CLAIMCLR_SHIFT                        (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CLAIMCLR_ETB_CLAIMCLR_RESETVAL                     (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CLAIMCLR_ETB_CLAIMCLR_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_CLAIMCLR_RESETVAL                                  (0x00000000U)

/* ETB_LAR */

#define CSL_MSS_DEBUGSS_ETB_LAR_ETB_LAR_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_LAR_ETB_LAR_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_LAR_ETB_LAR_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_LAR_ETB_LAR_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_LAR_RESETVAL                                       (0x00000000U)

/* ETB_LSR */

#define CSL_MSS_DEBUGSS_ETB_LSR_ETB_LSR_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_LSR_ETB_LSR_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_LSR_ETB_LSR_RESETVAL                               (0x00000003U)
#define CSL_MSS_DEBUGSS_ETB_LSR_ETB_LSR_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_LSR_RESETVAL                                       (0x00000003U)

/* ETB_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_ETB_AUTHSTATUS_ETB_AUTHSTATUS_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_AUTHSTATUS_ETB_AUTHSTATUS_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_AUTHSTATUS_ETB_AUTHSTATUS_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_AUTHSTATUS_ETB_AUTHSTATUS_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_AUTHSTATUS_RESETVAL                                (0x00000000U)

/* ETB_DEVID */

#define CSL_MSS_DEBUGSS_ETB_DEVID_ETB_DEVID_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_DEVID_ETB_DEVID_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_DEVID_ETB_DEVID_RESETVAL                           (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_DEVID_ETB_DEVID_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_DEVID_RESETVAL                                     (0x00000000U)

/* ETB_DEVTYPE */

#define CSL_MSS_DEBUGSS_ETB_DEVTYPE_ETB_DEVTYPE_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_DEVTYPE_ETB_DEVTYPE_SHIFT                          (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_DEVTYPE_ETB_DEVTYPE_RESETVAL                       (0x00000021U)
#define CSL_MSS_DEBUGSS_ETB_DEVTYPE_ETB_DEVTYPE_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_DEVTYPE_RESETVAL                                   (0x00000021U)

/* ETB_PIDR4 */

#define CSL_MSS_DEBUGSS_ETB_PIDR4_ETB_PIDR4_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_PIDR4_ETB_PIDR4_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_PIDR4_ETB_PIDR4_RESETVAL                           (0x00000004U)
#define CSL_MSS_DEBUGSS_ETB_PIDR4_ETB_PIDR4_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_PIDR4_RESETVAL                                     (0x00000004U)

/* ETB_RESERVED4 */

#define CSL_MSS_DEBUGSS_ETB_RESERVED4_ETB_RESERVED4_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_RESERVED4_ETB_RESERVED4_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED4_ETB_RESERVED4_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED4_ETB_RESERVED4_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_RESERVED4_RESETVAL                                 (0x00000000U)

/* ETB_RESERVED5 */

#define CSL_MSS_DEBUGSS_ETB_RESERVED5_ETB_RESERVED5_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_RESERVED5_ETB_RESERVED5_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED5_ETB_RESERVED5_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED5_ETB_RESERVED5_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_RESERVED5_RESETVAL                                 (0x00000000U)

/* ETB_RESERVED6 */

#define CSL_MSS_DEBUGSS_ETB_RESERVED6_ETB_RESERVED6_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_RESERVED6_ETB_RESERVED6_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED6_ETB_RESERVED6_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_RESERVED6_ETB_RESERVED6_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_RESERVED6_RESETVAL                                 (0x00000000U)

/* ETB_PIDR0 */

#define CSL_MSS_DEBUGSS_ETB_PIDR0_ETB_PIDR0_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_PIDR0_ETB_PIDR0_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_PIDR0_ETB_PIDR0_RESETVAL                           (0x00000007U)
#define CSL_MSS_DEBUGSS_ETB_PIDR0_ETB_PIDR0_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_PIDR0_RESETVAL                                     (0x00000007U)

/* ETB_PIDR1 */

#define CSL_MSS_DEBUGSS_ETB_PIDR1_ETB_PIDR1_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_PIDR1_ETB_PIDR1_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_PIDR1_ETB_PIDR1_RESETVAL                           (0x000000B9U)
#define CSL_MSS_DEBUGSS_ETB_PIDR1_ETB_PIDR1_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_PIDR1_RESETVAL                                     (0x000000B9U)

/* ETB_PIDR2 */

#define CSL_MSS_DEBUGSS_ETB_PIDR2_ETB_PIDR2_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_PIDR2_ETB_PIDR2_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_PIDR2_ETB_PIDR2_RESETVAL                           (0x0000003BU)
#define CSL_MSS_DEBUGSS_ETB_PIDR2_ETB_PIDR2_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_PIDR2_RESETVAL                                     (0x0000003BU)

/* ETB_PIDR3 */

#define CSL_MSS_DEBUGSS_ETB_PIDR3_ETB_PIDR3_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_PIDR3_ETB_PIDR3_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_PIDR3_ETB_PIDR3_RESETVAL                           (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_PIDR3_ETB_PIDR3_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_PIDR3_RESETVAL                                     (0x00000000U)

/* ETB_CIDR0 */

#define CSL_MSS_DEBUGSS_ETB_CIDR0_ETB_CIDR0_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_CIDR0_ETB_CIDR0_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CIDR0_ETB_CIDR0_RESETVAL                           (0x0000000DU)
#define CSL_MSS_DEBUGSS_ETB_CIDR0_ETB_CIDR0_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_CIDR0_RESETVAL                                     (0x0000000DU)

/* ETB_CIDR1 */

#define CSL_MSS_DEBUGSS_ETB_CIDR1_ETB_CIDR1_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_CIDR1_ETB_CIDR1_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CIDR1_ETB_CIDR1_RESETVAL                           (0x00000090U)
#define CSL_MSS_DEBUGSS_ETB_CIDR1_ETB_CIDR1_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_CIDR1_RESETVAL                                     (0x00000090U)

/* ETB_CIDR2 */

#define CSL_MSS_DEBUGSS_ETB_CIDR2_ETB_CIDR2_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_CIDR2_ETB_CIDR2_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CIDR2_ETB_CIDR2_RESETVAL                           (0x00000005U)
#define CSL_MSS_DEBUGSS_ETB_CIDR2_ETB_CIDR2_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_CIDR2_RESETVAL                                     (0x00000005U)

/* ETB_CIDR3 */

#define CSL_MSS_DEBUGSS_ETB_CIDR3_ETB_CIDR3_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_ETB_CIDR3_ETB_CIDR3_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_ETB_CIDR3_ETB_CIDR3_RESETVAL                           (0x000000B1U)
#define CSL_MSS_DEBUGSS_ETB_CIDR3_ETB_CIDR3_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_ETB_CIDR3_RESETVAL                                     (0x000000B1U)

/* STMDMASTARTR */

#define CSL_MSS_DEBUGSS_STMDMASTARTR_STMDMASTARTR_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMDMASTARTR_STMDMASTARTR_SHIFT                        (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDMASTARTR_STMDMASTARTR_RESETVAL                     (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDMASTARTR_STMDMASTARTR_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMDMASTARTR_RESETVAL                                  (0x00000000U)

/* STMDMASTOPR */

#define CSL_MSS_DEBUGSS_STMDMASTOPR_STMDMASTOPR_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMDMASTOPR_STMDMASTOPR_SHIFT                          (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDMASTOPR_STMDMASTOPR_RESETVAL                       (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDMASTOPR_STMDMASTOPR_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMDMASTOPR_RESETVAL                                   (0x00000000U)

/* STMDMASTATR */

#define CSL_MSS_DEBUGSS_STMDMASTATR_STMDMASTATR_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMDMASTATR_STMDMASTATR_SHIFT                          (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDMASTATR_STMDMASTATR_RESETVAL                       (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDMASTATR_STMDMASTATR_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMDMASTATR_RESETVAL                                   (0x00000000U)

/* STMDMACTLR */

#define CSL_MSS_DEBUGSS_STMDMACTLR_SENS_MASK                                   (0x0CU)
#define CSL_MSS_DEBUGSS_STMDMACTLR_SENS_SHIFT                                  (0x02U)
#define CSL_MSS_DEBUGSS_STMDMACTLR_SENS_RESETVAL                               (0x00U)
#define CSL_MSS_DEBUGSS_STMDMACTLR_SENS_MAX                                    (0x03U)

#define CSL_MSS_DEBUGSS_STMDMACTLR_RESETVAL                                    (0x00U)

/* STMDMAIDR */

#define CSL_MSS_DEBUGSS_STMDMAIDR_STMDMAIDR_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMDMAIDR_STMDMAIDR_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDMAIDR_STMDMAIDR_RESETVAL                           (0x00000002U)
#define CSL_MSS_DEBUGSS_STMDMAIDR_STMDMAIDR_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMDMAIDR_RESETVAL                                     (0x00000002U)

/* STMHEER */

#define CSL_MSS_DEBUGSS_STMHEER_STMHEER_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMHEER_STMHEER_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHEER_STMHEER_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHEER_STMHEER_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMHEER_RESETVAL                                       (0x00000000U)

/* STMHETER */

#define CSL_MSS_DEBUGSS_STMHETER_STMHETER_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMHETER_STMHETER_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHETER_STMHETER_RESETVAL                             (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHETER_STMHETER_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMHETER_RESETVAL                                      (0x00000000U)

/* STMHEBSR */

#define CSL_MSS_DEBUGSS_STMHEBSR_STMHEBSR_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMHEBSR_STMHEBSR_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHEBSR_STMHEBSR_RESETVAL                             (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHEBSR_STMHEBSR_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMHEBSR_RESETVAL                                      (0x00000000U)

/* STMHEMCR */

#define CSL_MSS_DEBUGSS_STMHEMCR_STMHEMCR_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMHEMCR_STMHEMCR_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHEMCR_STMHEMCR_RESETVAL                             (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHEMCR_STMHEMCR_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMHEMCR_RESETVAL                                      (0x00000000U)

/* STMHEEXTMUXR */

#define CSL_MSS_DEBUGSS_STMHEEXTMUXR_EXTMUX_MASK                               (0xFFU)
#define CSL_MSS_DEBUGSS_STMHEEXTMUXR_EXTMUX_SHIFT                              (0x00U)
#define CSL_MSS_DEBUGSS_STMHEEXTMUXR_EXTMUX_RESETVAL                           (0x00U)
#define CSL_MSS_DEBUGSS_STMHEEXTMUXR_EXTMUX_MAX                                (0xFFU)

#define CSL_MSS_DEBUGSS_STMHEEXTMUXR_RESETVAL                                  (0x00U)

/* STMHEMASTR */

#define CSL_MSS_DEBUGSS_STMHEMASTR_MASTER_MASK                                 (0xFFFFU)
#define CSL_MSS_DEBUGSS_STMHEMASTR_MASTER_SHIFT                                (0x0000U)
#define CSL_MSS_DEBUGSS_STMHEMASTR_MASTER_RESETVAL                             (0x0080U)
#define CSL_MSS_DEBUGSS_STMHEMASTR_MASTER_MAX                                  (0xFFFFU)

#define CSL_MSS_DEBUGSS_STMHEMASTR_RESETVAL                                    (0x0080U)

/* STMHEFEAT1R */

#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEEXTMUXSIZE_MASK                          (0x70000000U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEEXTMUXSIZE_SHIFT                         (0x0000001CU)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEEXTMUXSIZE_RESETVAL                      (0x00000003U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEEXTMUXSIZE_MAX                           (0x00000007U)

#define CSL_MSS_DEBUGSS_STMHEFEAT1R_NUMHE_MASK                                 (0x00FF8000U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_NUMHE_SHIFT                                (0x0000000FU)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_NUMHE_RESETVAL                             (0x00000040U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_NUMHE_MAX                                  (0x000001FFU)

#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HECOMP_MASK                                (0x00000030U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HECOMP_SHIFT                               (0x00000004U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HECOMP_RESETVAL                            (0x00000003U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HECOMP_MAX                                 (0x00000003U)

#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEMASTR_MASK                               (0x00000008U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEMASTR_SHIFT                              (0x00000003U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEMASTR_RESETVAL                           (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEMASTR_MAX                                (0x00000001U)

#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEERR_MASK                                 (0x00000004U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEERR_SHIFT                                (0x00000002U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEERR_RESETVAL                             (0x00000001U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HEERR_MAX                                  (0x00000001U)

#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HETER_MASK                                 (0x00000001U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HETER_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HETER_RESETVAL                             (0x00000001U)
#define CSL_MSS_DEBUGSS_STMHEFEAT1R_HETER_MAX                                  (0x00000001U)

#define CSL_MSS_DEBUGSS_STMHEFEAT1R_RESETVAL                                   (0x30200035U)

/* STMHEIDR */

#define CSL_MSS_DEBUGSS_STMHEIDR_VENDSPEC_MASK                                 (0x0F00U)
#define CSL_MSS_DEBUGSS_STMHEIDR_VENDSPEC_SHIFT                                (0x0008U)
#define CSL_MSS_DEBUGSS_STMHEIDR_VENDSPEC_RESETVAL                             (0x0000U)
#define CSL_MSS_DEBUGSS_STMHEIDR_VENDSPEC_MAX                                  (0x000FU)

#define CSL_MSS_DEBUGSS_STMHEIDR_CLASSREV_MASK                                 (0x00F0U)
#define CSL_MSS_DEBUGSS_STMHEIDR_CLASSREV_SHIFT                                (0x0004U)
#define CSL_MSS_DEBUGSS_STMHEIDR_CLASSREV_RESETVAL                             (0x0001U)
#define CSL_MSS_DEBUGSS_STMHEIDR_CLASSREV_MAX                                  (0x000FU)

#define CSL_MSS_DEBUGSS_STMHEIDR_CLASS_MASK                                    (0x000FU)
#define CSL_MSS_DEBUGSS_STMHEIDR_CLASS_SHIFT                                   (0x0000U)
#define CSL_MSS_DEBUGSS_STMHEIDR_CLASS_RESETVAL                                (0x0001U)
#define CSL_MSS_DEBUGSS_STMHEIDR_CLASS_MAX                                     (0x000FU)

#define CSL_MSS_DEBUGSS_STMHEIDR_RESETVAL                                      (0x0011U)

/* STMSPER */

#define CSL_MSS_DEBUGSS_STMSPER_STMSPER_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMSPER_STMSPER_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPER_STMSPER_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPER_STMSPER_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMSPER_RESETVAL                                       (0x00000000U)

/* STMSPTER */

#define CSL_MSS_DEBUGSS_STMSPTER_STMSPTER_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMSPTER_STMSPTER_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPTER_STMSPTER_RESETVAL                             (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPTER_STMSPTER_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMSPTER_RESETVAL                                      (0x00000000U)

/* STMSPSCR */

#define CSL_MSS_DEBUGSS_STMSPSCR_STMSPSCR_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMSPSCR_STMSPSCR_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPSCR_STMSPSCR_RESETVAL                             (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPSCR_STMSPSCR_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMSPSCR_RESETVAL                                      (0x00000000U)

/* STMSPMSCR */

#define CSL_MSS_DEBUGSS_STMSPMSCR_STMSPMSCR_MASK                               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMSPMSCR_STMSPMSCR_SHIFT                              (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPMSCR_STMSPMSCR_RESETVAL                           (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPMSCR_STMSPMSCR_MAX                                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMSPMSCR_RESETVAL                                     (0x00000000U)

/* STMSPOVERRIDER */

#define CSL_MSS_DEBUGSS_STMSPOVERRIDER_STMSPOVERRIDER_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMSPOVERRIDER_STMSPOVERRIDER_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPOVERRIDER_STMSPOVERRIDER_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPOVERRIDER_STMSPOVERRIDER_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMSPOVERRIDER_RESETVAL                                (0x00000000U)

/* STMSPMOVERRIDER */

#define CSL_MSS_DEBUGSS_STMSPMOVERRIDER_STMSPMOVERRIDER_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMSPMOVERRIDER_STMSPMOVERRIDER_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPMOVERRIDER_STMSPMOVERRIDER_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPMOVERRIDER_STMSPMOVERRIDER_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMSPMOVERRIDER_RESETVAL                               (0x00000000U)

/* STMSPTRIGCSR */

#define CSL_MSS_DEBUGSS_STMSPTRIGCSR_STMSPTRIGCSR_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMSPTRIGCSR_STMSPTRIGCSR_SHIFT                        (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPTRIGCSR_STMSPTRIGCSR_RESETVAL                     (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSPTRIGCSR_STMSPTRIGCSR_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMSPTRIGCSR_RESETVAL                                  (0x00000000U)

/* STMTCSR */

#define CSL_MSS_DEBUGSS_STMTCSR_BUSY_MASK                                      (0x00800000U)
#define CSL_MSS_DEBUGSS_STMTCSR_BUSY_SHIFT                                     (0x00000017U)
#define CSL_MSS_DEBUGSS_STMTCSR_BUSY_RESETVAL                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTCSR_BUSY_MAX                                       (0x00000001U)

#define CSL_MSS_DEBUGSS_STMTCSR_TRACEID_MASK                                   (0x007F0000U)
#define CSL_MSS_DEBUGSS_STMTCSR_TRACEID_SHIFT                                  (0x00000010U)
#define CSL_MSS_DEBUGSS_STMTCSR_TRACEID_RESETVAL                               (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTCSR_TRACEID_MAX                                    (0x0000007FU)

#define CSL_MSS_DEBUGSS_STMTCSR_COMPEN_MASK                                    (0x00000020U)
#define CSL_MSS_DEBUGSS_STMTCSR_COMPEN_SHIFT                                   (0x00000005U)
#define CSL_MSS_DEBUGSS_STMTCSR_COMPEN_RESETVAL                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTCSR_COMPEN_MAX                                     (0x00000001U)

#define CSL_MSS_DEBUGSS_STMTCSR_SYNCEN_MASK                                    (0x00000004U)
#define CSL_MSS_DEBUGSS_STMTCSR_SYNCEN_SHIFT                                   (0x00000002U)
#define CSL_MSS_DEBUGSS_STMTCSR_SYNCEN_RESETVAL                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTCSR_SYNCEN_MAX                                     (0x00000001U)

#define CSL_MSS_DEBUGSS_STMTCSR_TSEN_MASK                                      (0x00000002U)
#define CSL_MSS_DEBUGSS_STMTCSR_TSEN_SHIFT                                     (0x00000001U)
#define CSL_MSS_DEBUGSS_STMTCSR_TSEN_RESETVAL                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTCSR_TSEN_MAX                                       (0x00000001U)

#define CSL_MSS_DEBUGSS_STMTCSR_EN_MASK                                        (0x00000001U)
#define CSL_MSS_DEBUGSS_STMTCSR_EN_SHIFT                                       (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTCSR_EN_RESETVAL                                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTCSR_EN_MAX                                         (0x00000001U)

#define CSL_MSS_DEBUGSS_STMTCSR_RESETVAL                                       (0x00000000U)

/* STMTSSTIMR */

#define CSL_MSS_DEBUGSS_STMTSSTIMR_STMTSSTIMR_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMTSSTIMR_STMTSSTIMR_SHIFT                            (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTSSTIMR_STMTSSTIMR_RESETVAL                         (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTSSTIMR_STMTSSTIMR_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMTSSTIMR_RESETVAL                                    (0x00000000U)

/* STMTSFREQR */

#define CSL_MSS_DEBUGSS_STMTSFREQR_STMTSFREQR_MASK                             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMTSFREQR_STMTSFREQR_SHIFT                            (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTSFREQR_STMTSFREQR_RESETVAL                         (0x00000000U)
#define CSL_MSS_DEBUGSS_STMTSFREQR_STMTSFREQR_MAX                              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMTSFREQR_RESETVAL                                    (0x00000000U)

/* STMSYNCR */

#define CSL_MSS_DEBUGSS_STMSYNCR_STMSYNCR_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMSYNCR_STMSYNCR_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSYNCR_STMSYNCR_RESETVAL                             (0x00000000U)
#define CSL_MSS_DEBUGSS_STMSYNCR_STMSYNCR_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMSYNCR_RESETVAL                                      (0x00000000U)

/* STMAUXCR */

#define CSL_MSS_DEBUGSS_STMAUXCR_QHWEVOVERRIDE_MASK                            (0x80U)
#define CSL_MSS_DEBUGSS_STMAUXCR_QHWEVOVERRIDE_SHIFT                           (0x07U)
#define CSL_MSS_DEBUGSS_STMAUXCR_QHWEVOVERRIDE_RESETVAL                        (0x00U)
#define CSL_MSS_DEBUGSS_STMAUXCR_QHWEVOVERRIDE_MAX                             (0x01U)

#define CSL_MSS_DEBUGSS_STMAUXCR_PRIORINVDIS_MASK                              (0x04U)
#define CSL_MSS_DEBUGSS_STMAUXCR_PRIORINVDIS_SHIFT                             (0x02U)
#define CSL_MSS_DEBUGSS_STMAUXCR_PRIORINVDIS_RESETVAL                          (0x00U)
#define CSL_MSS_DEBUGSS_STMAUXCR_PRIORINVDIS_MAX                               (0x01U)

#define CSL_MSS_DEBUGSS_STMAUXCR_ASYNCPE_MASK                                  (0x02U)
#define CSL_MSS_DEBUGSS_STMAUXCR_ASYNCPE_SHIFT                                 (0x01U)
#define CSL_MSS_DEBUGSS_STMAUXCR_ASYNCPE_RESETVAL                              (0x00U)
#define CSL_MSS_DEBUGSS_STMAUXCR_ASYNCPE_MAX                                   (0x01U)

#define CSL_MSS_DEBUGSS_STMAUXCR_FIFOAF_MASK                                   (0x01U)
#define CSL_MSS_DEBUGSS_STMAUXCR_FIFOAF_SHIFT                                  (0x00U)
#define CSL_MSS_DEBUGSS_STMAUXCR_FIFOAF_RESETVAL                               (0x00U)
#define CSL_MSS_DEBUGSS_STMAUXCR_FIFOAF_MAX                                    (0x01U)

#define CSL_MSS_DEBUGSS_STMAUXCR_RESETVAL                                      (0x00U)

/* STMFEAT1R */

#define CSL_MSS_DEBUGSS_STMFEAT1R_SWOEN_MASK                                   (0x00C00000U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SWOEN_SHIFT                                  (0x00000016U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SWOEN_RESETVAL                               (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SWOEN_MAX                                    (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_SYNCEN_MASK                                  (0x00300000U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SYNCEN_SHIFT                                 (0x00000014U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SYNCEN_RESETVAL                              (0x00000002U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SYNCEN_MAX                                   (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_HWTEN_MASK                                   (0x000C0000U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_HWTEN_SHIFT                                  (0x00000012U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_HWTEN_RESETVAL                               (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_HWTEN_MAX                                    (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_TSPRESCALE_MASK                              (0x00030000U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TSPRESCALE_SHIFT                             (0x00000010U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TSPRESCALE_RESETVAL                          (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TSPRESCALE_MAX                               (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_TRIGCTL_MASK                                 (0x0000C000U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TRIGCTL_SHIFT                                (0x0000000EU)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TRIGCTL_RESETVAL                             (0x00000002U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TRIGCTL_MAX                                  (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_TRACEBUS_MASK                                (0x00003C00U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TRACEBUS_SHIFT                               (0x0000000AU)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TRACEBUS_RESETVAL                            (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TRACEBUS_MAX                                 (0x0000000FU)

#define CSL_MSS_DEBUGSS_STMFEAT1R_SYNC_MASK                                    (0x00000300U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SYNC_SHIFT                                   (0x00000008U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SYNC_RESETVAL                                (0x00000003U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_SYNC_MAX                                     (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_FORCETS_MASK                                 (0x00000080U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_FORCETS_SHIFT                                (0x00000007U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_FORCETS_RESETVAL                             (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_FORCETS_MAX                                  (0x00000001U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_TSFREQ_MASK                                  (0x00000040U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TSFREQ_SHIFT                                 (0x00000006U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TSFREQ_RESETVAL                              (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TSFREQ_MAX                                   (0x00000001U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_TS_MASK                                      (0x00000030U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TS_SHIFT                                     (0x00000004U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TS_RESETVAL                                  (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_TS_MAX                                       (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT1R_PROT_MASK                                    (0x0000000FU)
#define CSL_MSS_DEBUGSS_STMFEAT1R_PROT_SHIFT                                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_PROT_RESETVAL                                (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT1R_PROT_MAX                                     (0x0000000FU)

#define CSL_MSS_DEBUGSS_STMFEAT1R_RESETVAL                                     (0x006587D1U)

/* STMFEAT2R */

#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTYPE_MASK                                  (0x00030000U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTYPE_SHIFT                                 (0x00000010U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTYPE_RESETVAL                              (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTYPE_MAX                                   (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT2R_DSIZE_MASK                                   (0x0000F000U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_DSIZE_SHIFT                                  (0x0000000CU)
#define CSL_MSS_DEBUGSS_STMFEAT2R_DSIZE_RESETVAL                               (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_DSIZE_MAX                                    (0x0000000FU)

#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTRTYPE_MASK                                (0x00000600U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTRTYPE_SHIFT                               (0x00000009U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTRTYPE_RESETVAL                            (0x00000002U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTRTYPE_MAX                                 (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT2R_PRIVMASK_MASK                                (0x00000180U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_PRIVMASK_SHIFT                               (0x00000007U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_PRIVMASK_RESETVAL                            (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_PRIVMASK_MAX                                 (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT2R_SPOVERRIDE_MASK                              (0x00000040U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPOVERRIDE_SHIFT                             (0x00000006U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPOVERRIDE_RESETVAL                          (0x00000001U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPOVERRIDE_MAX                               (0x00000001U)

#define CSL_MSS_DEBUGSS_STMFEAT2R_SPCOMP_MASK                                  (0x00000030U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPCOMP_SHIFT                                 (0x00000004U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPCOMP_RESETVAL                              (0x00000003U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPCOMP_MAX                                   (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT2R_SPER_MASK                                    (0x00000004U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPER_SHIFT                                   (0x00000002U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPER_RESETVAL                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPER_MAX                                     (0x00000001U)

#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTER_MASK                                   (0x00000003U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTER_SHIFT                                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTER_RESETVAL                               (0x00000002U)
#define CSL_MSS_DEBUGSS_STMFEAT2R_SPTER_MAX                                    (0x00000003U)

#define CSL_MSS_DEBUGSS_STMFEAT2R_RESETVAL                                     (0x000114F2U)

/* STMFEAT3R */

#define CSL_MSS_DEBUGSS_STMFEAT3R_NUMMAST_MASK                                 (0x7FU)
#define CSL_MSS_DEBUGSS_STMFEAT3R_NUMMAST_SHIFT                                (0x00U)
#define CSL_MSS_DEBUGSS_STMFEAT3R_NUMMAST_RESETVAL                             (0x7FU)
#define CSL_MSS_DEBUGSS_STMFEAT3R_NUMMAST_MAX                                  (0x7FU)

#define CSL_MSS_DEBUGSS_STMFEAT3R_RESETVAL                                     (0x7FU)

/* STMITTRIGGER */

#define CSL_MSS_DEBUGSS_STMITTRIGGER_ASYNCOUT_W_MASK                           (0x08U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_ASYNCOUT_W_SHIFT                          (0x03U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_ASYNCOUT_W_RESETVAL                       (0x00U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_ASYNCOUT_W_MAX                            (0x01U)

#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTHETE_W_MASK                        (0x04U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTHETE_W_SHIFT                       (0x02U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTHETE_W_RESETVAL                    (0x00U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTHETE_W_MAX                         (0x01U)

#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTSW_W_MASK                          (0x02U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTSW_W_SHIFT                         (0x01U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTSW_W_RESETVAL                      (0x00U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTSW_W_MAX                           (0x01U)

#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTSPTE_W_MASK                        (0x01U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTSPTE_W_SHIFT                       (0x00U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTSPTE_W_RESETVAL                    (0x00U)
#define CSL_MSS_DEBUGSS_STMITTRIGGER_TRIGOUTSPTE_W_MAX                         (0x01U)

#define CSL_MSS_DEBUGSS_STMITTRIGGER_RESETVAL                                  (0x00U)

/* STMITATBDATA0 */

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM63_W_MASK                         (0x0100U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM63_W_SHIFT                        (0x0008U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM63_W_RESETVAL                     (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM63_W_MAX                          (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM55_W_MASK                         (0x0080U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM55_W_SHIFT                        (0x0007U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM55_W_RESETVAL                     (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM55_W_MAX                          (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM47_W_MASK                         (0x0040U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM47_W_SHIFT                        (0x0006U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM47_W_RESETVAL                     (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM47_W_MAX                          (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM39_W_MASK                         (0x0020U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM39_W_SHIFT                        (0x0005U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM39_W_RESETVAL                     (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM39_W_MAX                          (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM31_W_MASK                         (0x0010U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM31_W_SHIFT                        (0x0004U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM31_W_RESETVAL                     (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM31_W_MAX                          (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM23_W_MASK                         (0x0008U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM23_W_SHIFT                        (0x0003U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM23_W_RESETVAL                     (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM23_W_MAX                          (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM15_W_MASK                         (0x0004U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM15_W_SHIFT                        (0x0002U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM15_W_RESETVAL                     (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM15_W_MAX                          (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM7_W_MASK                          (0x0002U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM7_W_SHIFT                         (0x0001U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM7_W_RESETVAL                      (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM7_W_MAX                           (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM0_W_MASK                          (0x0001U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM0_W_SHIFT                         (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM0_W_RESETVAL                      (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBDATA0_ATDATAM0_W_MAX                           (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBDATA0_RESETVAL                                 (0x0000U)

/* STMITATBCTR2 */

#define CSL_MSS_DEBUGSS_STMITATBCTR2_AFVALIDM_R_MASK                           (0x02U)
#define CSL_MSS_DEBUGSS_STMITATBCTR2_AFVALIDM_R_SHIFT                          (0x01U)
#define CSL_MSS_DEBUGSS_STMITATBCTR2_AFVALIDM_R_RESETVAL                       (0x00U)
#define CSL_MSS_DEBUGSS_STMITATBCTR2_AFVALIDM_R_MAX                            (0x01U)

#define CSL_MSS_DEBUGSS_STMITATBCTR2_ATREADYM_R_MASK                           (0x01U)
#define CSL_MSS_DEBUGSS_STMITATBCTR2_ATREADYM_R_SHIFT                          (0x00U)
#define CSL_MSS_DEBUGSS_STMITATBCTR2_ATREADYM_R_RESETVAL                       (0x00U)
#define CSL_MSS_DEBUGSS_STMITATBCTR2_ATREADYM_R_MAX                            (0x01U)

#define CSL_MSS_DEBUGSS_STMITATBCTR2_RESETVAL                                  (0x00U)

/* STMITATBID */

#define CSL_MSS_DEBUGSS_STMITATBID_ATIDM_W_MASK                                (0x7FU)
#define CSL_MSS_DEBUGSS_STMITATBID_ATIDM_W_SHIFT                               (0x00U)
#define CSL_MSS_DEBUGSS_STMITATBID_ATIDM_W_RESETVAL                            (0x00U)
#define CSL_MSS_DEBUGSS_STMITATBID_ATIDM_W_MAX                                 (0x7FU)

#define CSL_MSS_DEBUGSS_STMITATBID_RESETVAL                                    (0x00U)

/* STMITATBCTR0 */

#define CSL_MSS_DEBUGSS_STMITATBCTR0_ATBYTESM_W_MASK                           (0x0700U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_ATBYTESM_W_SHIFT                          (0x0008U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_ATBYTESM_W_RESETVAL                       (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_ATBYTESM_W_MAX                            (0x0007U)

#define CSL_MSS_DEBUGSS_STMITATBCTR0_AFREADYM_W_MASK                           (0x0002U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_AFREADYM_W_SHIFT                          (0x0001U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_AFREADYM_W_RESETVAL                       (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_AFREADYM_W_MAX                            (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBCTR0_ATVALIDM_W_MASK                           (0x0001U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_ATVALIDM_W_SHIFT                          (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_ATVALIDM_W_RESETVAL                       (0x0000U)
#define CSL_MSS_DEBUGSS_STMITATBCTR0_ATVALIDM_W_MAX                            (0x0001U)

#define CSL_MSS_DEBUGSS_STMITATBCTR0_RESETVAL                                  (0x0000U)

/* STMITCTRL */

#define CSL_MSS_DEBUGSS_STMITCTRL_IME_MASK                                     (0x01U)
#define CSL_MSS_DEBUGSS_STMITCTRL_IME_SHIFT                                    (0x00U)
#define CSL_MSS_DEBUGSS_STMITCTRL_IME_RESETVAL                                 (0x00U)
#define CSL_MSS_DEBUGSS_STMITCTRL_IME_MAX                                      (0x01U)

#define CSL_MSS_DEBUGSS_STMITCTRL_RESETVAL                                     (0x00U)

/* STMCLAIMSET */

#define CSL_MSS_DEBUGSS_STMCLAIMSET_STMCLAIMSET_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMCLAIMSET_STMCLAIMSET_SHIFT                          (0x00000000U)
#define CSL_MSS_DEBUGSS_STMCLAIMSET_STMCLAIMSET_RESETVAL                       (0x0000000FU)
#define CSL_MSS_DEBUGSS_STMCLAIMSET_STMCLAIMSET_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMCLAIMSET_RESETVAL                                   (0x0000000FU)

/* STMCLAIMCLR */

#define CSL_MSS_DEBUGSS_STMCLAIMCLR_STMCLAIMCLR_MASK                           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMCLAIMCLR_STMCLAIMCLR_SHIFT                          (0x00000000U)
#define CSL_MSS_DEBUGSS_STMCLAIMCLR_STMCLAIMCLR_RESETVAL                       (0x00000000U)
#define CSL_MSS_DEBUGSS_STMCLAIMCLR_STMCLAIMCLR_MAX                            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMCLAIMCLR_RESETVAL                                   (0x00000000U)

/* STMLAR */

#define CSL_MSS_DEBUGSS_STMLAR_KEY_MASK                                        (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STMLAR_KEY_SHIFT                                       (0x00000000U)
#define CSL_MSS_DEBUGSS_STMLAR_KEY_RESETVAL                                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STMLAR_KEY_MAX                                         (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STMLAR_RESETVAL                                        (0x00000000U)

/* STMLSR */

#define CSL_MSS_DEBUGSS_STMLSR_NTT_MASK                                        (0x04U)
#define CSL_MSS_DEBUGSS_STMLSR_NTT_SHIFT                                       (0x02U)
#define CSL_MSS_DEBUGSS_STMLSR_NTT_RESETVAL                                    (0x00U)
#define CSL_MSS_DEBUGSS_STMLSR_NTT_MAX                                         (0x01U)

#define CSL_MSS_DEBUGSS_STMLSR_SLK_MASK                                        (0x02U)
#define CSL_MSS_DEBUGSS_STMLSR_SLK_SHIFT                                       (0x01U)
#define CSL_MSS_DEBUGSS_STMLSR_SLK_RESETVAL                                    (0x00U)
#define CSL_MSS_DEBUGSS_STMLSR_SLK_MAX                                         (0x01U)

#define CSL_MSS_DEBUGSS_STMLSR_SLI_MASK                                        (0x01U)
#define CSL_MSS_DEBUGSS_STMLSR_SLI_SHIFT                                       (0x00U)
#define CSL_MSS_DEBUGSS_STMLSR_SLI_RESETVAL                                    (0x00U)
#define CSL_MSS_DEBUGSS_STMLSR_SLI_MAX                                         (0x01U)

#define CSL_MSS_DEBUGSS_STMLSR_RESETVAL                                        (0x00U)

/* STMAUTHSTATUS */

#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_SNID_MASK                                (0xC0U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_SNID_SHIFT                               (0x06U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_SNID_RESETVAL                            (0x02U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_SNID_MAX                                 (0x03U)

#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_SID_MASK                                 (0x30U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_SID_SHIFT                                (0x04U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_SID_RESETVAL                             (0x02U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_SID_MAX                                  (0x03U)

#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_NSNID_MASK                               (0x0CU)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_NSNID_SHIFT                              (0x02U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_NSNID_RESETVAL                           (0x02U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_NSNID_MAX                                (0x03U)

#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_NSID_MASK                                (0x03U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_NSID_SHIFT                               (0x00U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_NSID_RESETVAL                            (0x02U)
#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_NSID_MAX                                 (0x03U)

#define CSL_MSS_DEBUGSS_STMAUTHSTATUS_RESETVAL                                 (0xAAU)

/* STMDEVARCH */

#define CSL_MSS_DEBUGSS_STMDEVARCH_ARCHITECT_MASK                              (0xFFE00000U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_ARCHITECT_SHIFT                             (0x00000015U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_ARCHITECT_RESETVAL                          (0x0000023BU)
#define CSL_MSS_DEBUGSS_STMDEVARCH_ARCHITECT_MAX                               (0x000007FFU)

#define CSL_MSS_DEBUGSS_STMDEVARCH_PRESENT_MASK                                (0x00100000U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_PRESENT_SHIFT                               (0x00000014U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_PRESENT_RESETVAL                            (0x00000001U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_PRESENT_MAX                                 (0x00000001U)

#define CSL_MSS_DEBUGSS_STMDEVARCH_REVISION_MASK                               (0x000F0000U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_REVISION_SHIFT                              (0x00000010U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_REVISION_RESETVAL                           (0x00000001U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_REVISION_MAX                                (0x0000000FU)

#define CSL_MSS_DEBUGSS_STMDEVARCH_ARCHID_MASK                                 (0x0000FFFFU)
#define CSL_MSS_DEBUGSS_STMDEVARCH_ARCHID_SHIFT                                (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_ARCHID_RESETVAL                             (0x00000A63U)
#define CSL_MSS_DEBUGSS_STMDEVARCH_ARCHID_MAX                                  (0x0000FFFFU)

#define CSL_MSS_DEBUGSS_STMDEVARCH_RESETVAL                                    (0x47710A63U)

/* STMDEVID */

#define CSL_MSS_DEBUGSS_STMDEVID_NUMSP_MASK                                    (0x0001FFFFU)
#define CSL_MSS_DEBUGSS_STMDEVID_NUMSP_SHIFT                                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STMDEVID_NUMSP_RESETVAL                                (0x00010000U)
#define CSL_MSS_DEBUGSS_STMDEVID_NUMSP_MAX                                     (0x0001FFFFU)

#define CSL_MSS_DEBUGSS_STMDEVID_RESETVAL                                      (0x00010000U)

/* STMDEVTYPE */

#define CSL_MSS_DEBUGSS_STMDEVTYPE_SUB_MASK                                    (0xF0U)
#define CSL_MSS_DEBUGSS_STMDEVTYPE_SUB_SHIFT                                   (0x04U)
#define CSL_MSS_DEBUGSS_STMDEVTYPE_SUB_RESETVAL                                (0x06U)
#define CSL_MSS_DEBUGSS_STMDEVTYPE_SUB_MAX                                     (0x0FU)

#define CSL_MSS_DEBUGSS_STMDEVTYPE_MAJOR_MASK                                  (0x0FU)
#define CSL_MSS_DEBUGSS_STMDEVTYPE_MAJOR_SHIFT                                 (0x00U)
#define CSL_MSS_DEBUGSS_STMDEVTYPE_MAJOR_RESETVAL                              (0x03U)
#define CSL_MSS_DEBUGSS_STMDEVTYPE_MAJOR_MAX                                   (0x0FU)

#define CSL_MSS_DEBUGSS_STMDEVTYPE_RESETVAL                                    (0x63U)

/* STMPIDR4 */

#define CSL_MSS_DEBUGSS_STMPIDR4_SIZE_MASK                                     (0xF0U)
#define CSL_MSS_DEBUGSS_STMPIDR4_SIZE_SHIFT                                    (0x04U)
#define CSL_MSS_DEBUGSS_STMPIDR4_SIZE_RESETVAL                                 (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR4_SIZE_MAX                                      (0x0FU)

#define CSL_MSS_DEBUGSS_STMPIDR4_DES_2_MASK                                    (0x0FU)
#define CSL_MSS_DEBUGSS_STMPIDR4_DES_2_SHIFT                                   (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR4_DES_2_RESETVAL                                (0x04U)
#define CSL_MSS_DEBUGSS_STMPIDR4_DES_2_MAX                                     (0x0FU)

#define CSL_MSS_DEBUGSS_STMPIDR4_RESETVAL                                      (0x04U)

/* STMPIDR0 */

#define CSL_MSS_DEBUGSS_STMPIDR0_PART_0_MASK                                   (0xFFU)
#define CSL_MSS_DEBUGSS_STMPIDR0_PART_0_SHIFT                                  (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR0_PART_0_RESETVAL                               (0x63U)
#define CSL_MSS_DEBUGSS_STMPIDR0_PART_0_MAX                                    (0xFFU)

#define CSL_MSS_DEBUGSS_STMPIDR0_RESETVAL                                      (0x63U)

/* STMPIDR1 */

#define CSL_MSS_DEBUGSS_STMPIDR1_DES_0_MASK                                    (0xF0U)
#define CSL_MSS_DEBUGSS_STMPIDR1_DES_0_SHIFT                                   (0x04U)
#define CSL_MSS_DEBUGSS_STMPIDR1_DES_0_RESETVAL                                (0x0BU)
#define CSL_MSS_DEBUGSS_STMPIDR1_DES_0_MAX                                     (0x0FU)

#define CSL_MSS_DEBUGSS_STMPIDR1_PART_1_MASK                                   (0x0FU)
#define CSL_MSS_DEBUGSS_STMPIDR1_PART_1_SHIFT                                  (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR1_PART_1_RESETVAL                               (0x09U)
#define CSL_MSS_DEBUGSS_STMPIDR1_PART_1_MAX                                    (0x0FU)

#define CSL_MSS_DEBUGSS_STMPIDR1_RESETVAL                                      (0xB9U)

/* STMPIDR2 */

#define CSL_MSS_DEBUGSS_STMPIDR2_REVISION_MASK                                 (0xF0U)
#define CSL_MSS_DEBUGSS_STMPIDR2_REVISION_SHIFT                                (0x04U)
#define CSL_MSS_DEBUGSS_STMPIDR2_REVISION_RESETVAL                             (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR2_REVISION_MAX                                  (0x0FU)

#define CSL_MSS_DEBUGSS_STMPIDR2_JEDEC_MASK                                    (0x08U)
#define CSL_MSS_DEBUGSS_STMPIDR2_JEDEC_SHIFT                                   (0x03U)
#define CSL_MSS_DEBUGSS_STMPIDR2_JEDEC_RESETVAL                                (0x01U)
#define CSL_MSS_DEBUGSS_STMPIDR2_JEDEC_MAX                                     (0x01U)

#define CSL_MSS_DEBUGSS_STMPIDR2_DES_1_MASK                                    (0x07U)
#define CSL_MSS_DEBUGSS_STMPIDR2_DES_1_SHIFT                                   (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR2_DES_1_RESETVAL                                (0x03U)
#define CSL_MSS_DEBUGSS_STMPIDR2_DES_1_MAX                                     (0x07U)

#define CSL_MSS_DEBUGSS_STMPIDR2_RESETVAL                                      (0x0BU)

/* STMPIDR3 */

#define CSL_MSS_DEBUGSS_STMPIDR3_REVAND_MASK                                   (0xF0U)
#define CSL_MSS_DEBUGSS_STMPIDR3_REVAND_SHIFT                                  (0x04U)
#define CSL_MSS_DEBUGSS_STMPIDR3_REVAND_RESETVAL                               (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR3_REVAND_MAX                                    (0x0FU)

#define CSL_MSS_DEBUGSS_STMPIDR3_CMOD_MASK                                     (0x0FU)
#define CSL_MSS_DEBUGSS_STMPIDR3_CMOD_SHIFT                                    (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR3_CMOD_RESETVAL                                 (0x00U)
#define CSL_MSS_DEBUGSS_STMPIDR3_CMOD_MAX                                      (0x0FU)

#define CSL_MSS_DEBUGSS_STMPIDR3_RESETVAL                                      (0x00U)

/* STMCIDR0 */

#define CSL_MSS_DEBUGSS_STMCIDR0_PRMBL_0_MASK                                  (0xFFU)
#define CSL_MSS_DEBUGSS_STMCIDR0_PRMBL_0_SHIFT                                 (0x00U)
#define CSL_MSS_DEBUGSS_STMCIDR0_PRMBL_0_RESETVAL                              (0x0DU)
#define CSL_MSS_DEBUGSS_STMCIDR0_PRMBL_0_MAX                                   (0xFFU)

#define CSL_MSS_DEBUGSS_STMCIDR0_RESETVAL                                      (0x0DU)

/* STMCIDR1 */

#define CSL_MSS_DEBUGSS_STMCIDR1_CLASS_MASK                                    (0xF0U)
#define CSL_MSS_DEBUGSS_STMCIDR1_CLASS_SHIFT                                   (0x04U)
#define CSL_MSS_DEBUGSS_STMCIDR1_CLASS_RESETVAL                                (0x09U)
#define CSL_MSS_DEBUGSS_STMCIDR1_CLASS_MAX                                     (0x0FU)

#define CSL_MSS_DEBUGSS_STMCIDR1_PRMBL_1_MASK                                  (0x0FU)
#define CSL_MSS_DEBUGSS_STMCIDR1_PRMBL_1_SHIFT                                 (0x00U)
#define CSL_MSS_DEBUGSS_STMCIDR1_PRMBL_1_RESETVAL                              (0x00U)
#define CSL_MSS_DEBUGSS_STMCIDR1_PRMBL_1_MAX                                   (0x0FU)

#define CSL_MSS_DEBUGSS_STMCIDR1_RESETVAL                                      (0x90U)

/* STMCIDR2 */

#define CSL_MSS_DEBUGSS_STMCIDR2_PRMBL_2_MASK                                  (0xFFU)
#define CSL_MSS_DEBUGSS_STMCIDR2_PRMBL_2_SHIFT                                 (0x00U)
#define CSL_MSS_DEBUGSS_STMCIDR2_PRMBL_2_RESETVAL                              (0x05U)
#define CSL_MSS_DEBUGSS_STMCIDR2_PRMBL_2_MAX                                   (0xFFU)

#define CSL_MSS_DEBUGSS_STMCIDR2_RESETVAL                                      (0x05U)

/* STMCIDR3 */

#define CSL_MSS_DEBUGSS_STMCIDR3_PRMBL_3_MASK                                  (0xFFU)
#define CSL_MSS_DEBUGSS_STMCIDR3_PRMBL_3_SHIFT                                 (0x00U)
#define CSL_MSS_DEBUGSS_STMCIDR3_PRMBL_3_RESETVAL                              (0xB1U)
#define CSL_MSS_DEBUGSS_STMCIDR3_PRMBL_3_MAX                                   (0xFFU)

#define CSL_MSS_DEBUGSS_STMCIDR3_RESETVAL                                      (0xB1U)

/* STM_CTI_CONTROL */

#define CSL_MSS_DEBUGSS_STM_CTI_CONTROL_STM_CTI_CONTROL_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_CONTROL_STM_CTI_CONTROL_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CONTROL_STM_CTI_CONTROL_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CONTROL_STM_CTI_CONTROL_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_CONTROL_RESETVAL                               (0x00000000U)

/* STM_CTI_INTACK */

#define CSL_MSS_DEBUGSS_STM_CTI_INTACK_STM_CTI_INTACK_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INTACK_STM_CTI_INTACK_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INTACK_STM_CTI_INTACK_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INTACK_STM_CTI_INTACK_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INTACK_RESETVAL                                (0x00000000U)

/* STM_CTI_APPSET */

#define CSL_MSS_DEBUGSS_STM_CTI_APPSET_STM_CTI_APPSET_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_APPSET_STM_CTI_APPSET_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPSET_STM_CTI_APPSET_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPSET_STM_CTI_APPSET_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_APPSET_RESETVAL                                (0x00000000U)

/* STM_CTI_APPCLEAR */

#define CSL_MSS_DEBUGSS_STM_CTI_APPCLEAR_STM_CTI_APPCLEAR_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_APPCLEAR_STM_CTI_APPCLEAR_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPCLEAR_STM_CTI_APPCLEAR_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPCLEAR_STM_CTI_APPCLEAR_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_APPCLEAR_RESETVAL                              (0x00000000U)

/* STM_CTI_APPPULSE */

#define CSL_MSS_DEBUGSS_STM_CTI_APPPULSE_STM_CTI_APPPULSE_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_APPPULSE_STM_CTI_APPPULSE_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPPULSE_STM_CTI_APPPULSE_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_APPPULSE_STM_CTI_APPPULSE_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_APPPULSE_RESETVAL                              (0x00000000U)

/* STM_CTI_INEN0 */

#define CSL_MSS_DEBUGSS_STM_CTI_INEN0_STM_CTI_INEN0_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN0_STM_CTI_INEN0_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN0_STM_CTI_INEN0_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN0_STM_CTI_INEN0_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INEN0_RESETVAL                                 (0x00000000U)

/* STM_CTI_INEN1 */

#define CSL_MSS_DEBUGSS_STM_CTI_INEN1_STM_CTI_INEN1_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN1_STM_CTI_INEN1_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN1_STM_CTI_INEN1_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN1_STM_CTI_INEN1_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INEN1_RESETVAL                                 (0x00000000U)

/* STM_CTI_INEN2 */

#define CSL_MSS_DEBUGSS_STM_CTI_INEN2_STM_CTI_INEN2_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN2_STM_CTI_INEN2_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN2_STM_CTI_INEN2_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN2_STM_CTI_INEN2_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INEN2_RESETVAL                                 (0x00000000U)

/* STM_CTI_INEN3 */

#define CSL_MSS_DEBUGSS_STM_CTI_INEN3_STM_CTI_INEN3_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN3_STM_CTI_INEN3_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN3_STM_CTI_INEN3_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN3_STM_CTI_INEN3_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INEN3_RESETVAL                                 (0x00000000U)

/* STM_CTI_INEN4 */

#define CSL_MSS_DEBUGSS_STM_CTI_INEN4_STM_CTI_INEN4_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN4_STM_CTI_INEN4_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN4_STM_CTI_INEN4_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN4_STM_CTI_INEN4_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INEN4_RESETVAL                                 (0x00000000U)

/* STM_CTI_INEN5 */

#define CSL_MSS_DEBUGSS_STM_CTI_INEN5_STM_CTI_INEN5_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN5_STM_CTI_INEN5_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN5_STM_CTI_INEN5_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN5_STM_CTI_INEN5_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INEN5_RESETVAL                                 (0x00000000U)

/* STM_CTI_INEN6 */

#define CSL_MSS_DEBUGSS_STM_CTI_INEN6_STM_CTI_INEN6_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN6_STM_CTI_INEN6_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN6_STM_CTI_INEN6_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN6_STM_CTI_INEN6_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INEN6_RESETVAL                                 (0x00000000U)

/* STM_CTI_INEN7 */

#define CSL_MSS_DEBUGSS_STM_CTI_INEN7_STM_CTI_INEN7_MASK                       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN7_STM_CTI_INEN7_SHIFT                      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN7_STM_CTI_INEN7_RESETVAL                   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_INEN7_STM_CTI_INEN7_MAX                        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_INEN7_RESETVAL                                 (0x00000000U)

/* STM_CTI_OUTEN0 */

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN0_STM_CTI_OUTEN0_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN0_STM_CTI_OUTEN0_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN0_STM_CTI_OUTEN0_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN0_STM_CTI_OUTEN0_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN0_RESETVAL                                (0x00000000U)

/* STM_CTI_OUTEN1 */

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN1_STM_CTI_OUTEN1_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN1_STM_CTI_OUTEN1_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN1_STM_CTI_OUTEN1_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN1_STM_CTI_OUTEN1_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN1_RESETVAL                                (0x00000000U)

/* STM_CTI_OUTEN2 */

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN2_STM_CTI_OUTEN2_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN2_STM_CTI_OUTEN2_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN2_STM_CTI_OUTEN2_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN2_STM_CTI_OUTEN2_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN2_RESETVAL                                (0x00000000U)

/* STM_CTI_OUTEN3 */

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN3_STM_CTI_OUTEN3_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN3_STM_CTI_OUTEN3_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN3_STM_CTI_OUTEN3_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN3_STM_CTI_OUTEN3_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN3_RESETVAL                                (0x00000000U)

/* STM_CTI_OUTEN4 */

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN4_STM_CTI_OUTEN4_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN4_STM_CTI_OUTEN4_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN4_STM_CTI_OUTEN4_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN4_STM_CTI_OUTEN4_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN4_RESETVAL                                (0x00000000U)

/* STM_CTI_OUTEN5 */

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN5_STM_CTI_OUTEN5_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN5_STM_CTI_OUTEN5_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN5_STM_CTI_OUTEN5_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN5_STM_CTI_OUTEN5_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN5_RESETVAL                                (0x00000000U)

/* STM_CTI_OUTEN6 */

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN6_STM_CTI_OUTEN6_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN6_STM_CTI_OUTEN6_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN6_STM_CTI_OUTEN6_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN6_STM_CTI_OUTEN6_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN6_RESETVAL                                (0x00000000U)

/* STM_CTI_OUTEN7 */

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN7_STM_CTI_OUTEN7_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN7_STM_CTI_OUTEN7_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN7_STM_CTI_OUTEN7_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN7_STM_CTI_OUTEN7_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_OUTEN7_RESETVAL                                (0x00000000U)

/* STM_CTI_TRIGINSTATUS */

#define CSL_MSS_DEBUGSS_STM_CTI_TRIGINSTATUS_STM_CTI_TRIGINSTATUS_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_TRIGINSTATUS_STM_CTI_TRIGINSTATUS_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_TRIGINSTATUS_STM_CTI_TRIGINSTATUS_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_TRIGINSTATUS_STM_CTI_TRIGINSTATUS_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_TRIGINSTATUS_RESETVAL                          (0x00000000U)

/* STM_CTI_TRIGOUTSTATUS */

#define CSL_MSS_DEBUGSS_STM_CTI_TRIGOUTSTATUS_STM_CTI_TRIGOUTSTATUS_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_TRIGOUTSTATUS_STM_CTI_TRIGOUTSTATUS_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_TRIGOUTSTATUS_STM_CTI_TRIGOUTSTATUS_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_TRIGOUTSTATUS_STM_CTI_TRIGOUTSTATUS_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_TRIGOUTSTATUS_RESETVAL                         (0x00000000U)

/* STM_CTI_CHINSTATUS */

#define CSL_MSS_DEBUGSS_STM_CTI_CHINSTATUS_STM_CTI_CHINSTATUS_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_CHINSTATUS_STM_CTI_CHINSTATUS_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CHINSTATUS_STM_CTI_CHINSTATUS_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CHINSTATUS_STM_CTI_CHINSTATUS_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_CHINSTATUS_RESETVAL                            (0x00000000U)

/* STM_CTI_CHOUTSTATUS */

#define CSL_MSS_DEBUGSS_STM_CTI_CHOUTSTATUS_STM_CTI_CHOUTSTATUS_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_CHOUTSTATUS_STM_CTI_CHOUTSTATUS_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CHOUTSTATUS_STM_CTI_CHOUTSTATUS_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CHOUTSTATUS_STM_CTI_CHOUTSTATUS_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_CHOUTSTATUS_RESETVAL                           (0x00000000U)

/* STM_CTI_GATE */

#define CSL_MSS_DEBUGSS_STM_CTI_GATE_STM_CTI_GATE_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_GATE_STM_CTI_GATE_SHIFT                        (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_GATE_STM_CTI_GATE_RESETVAL                     (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_GATE_STM_CTI_GATE_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_GATE_RESETVAL                                  (0x00000000U)

/* STM_CTI_ASICCTL */

#define CSL_MSS_DEBUGSS_STM_CTI_ASICCTL_STM_CTI_ASICCTL_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ASICCTL_STM_CTI_ASICCTL_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ASICCTL_STM_CTI_ASICCTL_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ASICCTL_STM_CTI_ASICCTL_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ASICCTL_RESETVAL                               (0x00000000U)

/* STM_CTI_ITCHINACK */

#define CSL_MSS_DEBUGSS_STM_CTI_ITCHINACK_STM_CTI_ITCHINACK_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHINACK_STM_CTI_ITCHINACK_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHINACK_STM_CTI_ITCHINACK_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHINACK_STM_CTI_ITCHINACK_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITCHINACK_RESETVAL                             (0x00000000U)

/* STM_CTI_ITTRIGINACK */

#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGINACK_STM_CTI_ITTRIGINACK_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGINACK_STM_CTI_ITTRIGINACK_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGINACK_STM_CTI_ITTRIGINACK_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGINACK_STM_CTI_ITTRIGINACK_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGINACK_RESETVAL                           (0x00000000U)

/* STM_CTI_ITCHOUT */

#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUT_STM_CTI_ITCHOUT_MASK                   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUT_STM_CTI_ITCHOUT_SHIFT                  (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUT_STM_CTI_ITCHOUT_RESETVAL               (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUT_STM_CTI_ITCHOUT_MAX                    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUT_RESETVAL                               (0x00000000U)

/* STM_CTI_ITTRIGOUT */

#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUT_STM_CTI_ITTRIGOUT_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUT_STM_CTI_ITTRIGOUT_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUT_STM_CTI_ITTRIGOUT_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUT_STM_CTI_ITTRIGOUT_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUT_RESETVAL                             (0x00000000U)

/* STM_CTI_ITCHOUTACK */

#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUTACK_STM_CTI_ITCHOUTACK_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUTACK_STM_CTI_ITCHOUTACK_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUTACK_STM_CTI_ITCHOUTACK_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUTACK_STM_CTI_ITCHOUTACK_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITCHOUTACK_RESETVAL                            (0x00000000U)

/* STM_CTI_ITTRIGOUTACK */

#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUTACK_STM_CTI_ITTRIGOUTACK_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUTACK_STM_CTI_ITTRIGOUTACK_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUTACK_STM_CTI_ITTRIGOUTACK_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUTACK_STM_CTI_ITTRIGOUTACK_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGOUTACK_RESETVAL                          (0x00000000U)

/* STM_CTI_ITCHIN */

#define CSL_MSS_DEBUGSS_STM_CTI_ITCHIN_STM_CTI_ITCHIN_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHIN_STM_CTI_ITCHIN_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHIN_STM_CTI_ITCHIN_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCHIN_STM_CTI_ITCHIN_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITCHIN_RESETVAL                                (0x00000000U)

/* STM_CTI_ITTRIGIN */

#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGIN_STM_CTI_ITTRIGIN_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGIN_STM_CTI_ITTRIGIN_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGIN_STM_CTI_ITTRIGIN_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGIN_STM_CTI_ITTRIGIN_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITTRIGIN_RESETVAL                              (0x00000000U)

/* STM_CTI_ITCTRL */

#define CSL_MSS_DEBUGSS_STM_CTI_ITCTRL_STM_CTI_ITCTRL_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCTRL_STM_CTI_ITCTRL_SHIFT                    (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCTRL_STM_CTI_ITCTRL_RESETVAL                 (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_ITCTRL_STM_CTI_ITCTRL_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_ITCTRL_RESETVAL                                (0x00000000U)

/* STM_CTI_CLAIM_TAG_SET */

#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_SET_STM_CTI_CLAIM_TAG_SET_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_SET_STM_CTI_CLAIM_TAG_SET_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_SET_STM_CTI_CLAIM_TAG_SET_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_SET_STM_CTI_CLAIM_TAG_SET_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_SET_RESETVAL                         (0x00000000U)

/* STM_CTI_CLAIM_TAG_CLEAR */

#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_CLEAR_STM_CTI_CLAIM_TAG_CLEAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_CLEAR_STM_CTI_CLAIM_TAG_CLEAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_CLEAR_STM_CTI_CLAIM_TAG_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_CLEAR_STM_CTI_CLAIM_TAG_CLEAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_CLAIM_TAG_CLEAR_RESETVAL                       (0x00000000U)

/* STM_CTI_LOCK_ACCESS_REGISTER */

#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_ACCESS_REGISTER_STM_CTI_LOCK_ACCESS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_ACCESS_REGISTER_STM_CTI_LOCK_ACCESS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_ACCESS_REGISTER_STM_CTI_LOCK_ACCESS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_ACCESS_REGISTER_STM_CTI_LOCK_ACCESS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_ACCESS_REGISTER_RESETVAL                  (0x00000000U)

/* STM_CTI_LOCK_STATUS_REGISTER */

#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_STATUS_REGISTER_STM_CTI_LOCK_STATUS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_STATUS_REGISTER_STM_CTI_LOCK_STATUS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_STATUS_REGISTER_STM_CTI_LOCK_STATUS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_STATUS_REGISTER_STM_CTI_LOCK_STATUS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_LOCK_STATUS_REGISTER_RESETVAL                  (0x00000000U)

/* STM_CTI_AUTHENTICATION_STATUS */

#define CSL_MSS_DEBUGSS_STM_CTI_AUTHENTICATION_STATUS_STM_CTI_AUTHENTICATION_STATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_AUTHENTICATION_STATUS_STM_CTI_AUTHENTICATION_STATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_AUTHENTICATION_STATUS_STM_CTI_AUTHENTICATION_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_AUTHENTICATION_STATUS_STM_CTI_AUTHENTICATION_STATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_AUTHENTICATION_STATUS_RESETVAL                 (0x00000000U)

/* STM_CTI_DEVICE_ID */

#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_ID_STM_CTI_DEVICE_ID_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_ID_STM_CTI_DEVICE_ID_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_ID_STM_CTI_DEVICE_ID_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_ID_STM_CTI_DEVICE_ID_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_ID_RESETVAL                             (0x00000000U)

/* STM_CTI_DEVICE_TYPE_IDENTIFIER */

#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_TYPE_IDENTIFIER_STM_CTI_DEVICE_TYPE_IDENTIFIER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_TYPE_IDENTIFIER_STM_CTI_DEVICE_TYPE_IDENTIFIER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_TYPE_IDENTIFIER_STM_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_TYPE_IDENTIFIER_STM_CTI_DEVICE_TYPE_IDENTIFIER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL                (0x00000000U)

/* STM_CTI_PERIPHERALID4 */

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID4_STM_CTI_PERIPHERALID4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID4_STM_CTI_PERIPHERALID4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID4_STM_CTI_PERIPHERALID4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID4_STM_CTI_PERIPHERALID4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID4_RESETVAL                         (0x00000000U)

/* STM_CTI_PERIPHERALID5 */

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID5_STM_CTI_PERIPHERALID5_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID5_STM_CTI_PERIPHERALID5_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID5_STM_CTI_PERIPHERALID5_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID5_STM_CTI_PERIPHERALID5_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID5_RESETVAL                         (0x00000000U)

/* STM_CTI_PERIPHERALID6 */

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID6_STM_CTI_PERIPHERALID6_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID6_STM_CTI_PERIPHERALID6_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID6_STM_CTI_PERIPHERALID6_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID6_STM_CTI_PERIPHERALID6_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID6_RESETVAL                         (0x00000000U)

/* STM_CTI_PERIPHERALID7 */

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID7_STM_CTI_PERIPHERALID7_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID7_STM_CTI_PERIPHERALID7_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID7_STM_CTI_PERIPHERALID7_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID7_STM_CTI_PERIPHERALID7_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID7_RESETVAL                         (0x00000000U)

/* STM_CTI_PERIPHERALID0 */

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID0_STM_CTI_PERIPHERALID0_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID0_STM_CTI_PERIPHERALID0_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID0_STM_CTI_PERIPHERALID0_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID0_STM_CTI_PERIPHERALID0_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID0_RESETVAL                         (0x00000000U)

/* STM_CTI_PERIPHERALID1 */

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID1_STM_CTI_PERIPHERALID1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID1_STM_CTI_PERIPHERALID1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID1_STM_CTI_PERIPHERALID1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID1_STM_CTI_PERIPHERALID1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID1_RESETVAL                         (0x00000000U)

/* STM_CTI_PERIPHERALID2 */

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID2_STM_CTI_PERIPHERALID2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID2_STM_CTI_PERIPHERALID2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID2_STM_CTI_PERIPHERALID2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID2_STM_CTI_PERIPHERALID2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID2_RESETVAL                         (0x00000000U)

/* STM_CTI_PERIPHERALID3 */

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID3_STM_CTI_PERIPHERALID3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID3_STM_CTI_PERIPHERALID3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID3_STM_CTI_PERIPHERALID3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID3_STM_CTI_PERIPHERALID3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_PERIPHERALID3_RESETVAL                         (0x00000000U)

/* STM_CTI_COMPONENT_ID0 */

#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID0_STM_CTI_COMPONENT_ID0_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID0_STM_CTI_COMPONENT_ID0_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID0_STM_CTI_COMPONENT_ID0_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID0_STM_CTI_COMPONENT_ID0_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID0_RESETVAL                         (0x00000000U)

/* STM_CTI_COMPONENT_ID1 */

#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID1_STM_CTI_COMPONENT_ID1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID1_STM_CTI_COMPONENT_ID1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID1_STM_CTI_COMPONENT_ID1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID1_STM_CTI_COMPONENT_ID1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID1_RESETVAL                         (0x00000000U)

/* STM_CTI_COMPONENT_ID2 */

#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID2_STM_CTI_COMPONENT_ID2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID2_STM_CTI_COMPONENT_ID2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID2_STM_CTI_COMPONENT_ID2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID2_STM_CTI_COMPONENT_ID2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID2_RESETVAL                         (0x00000000U)

/* STM_CTI_COMPONENT_ID3 */

#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID3_STM_CTI_COMPONENT_ID3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID3_STM_CTI_COMPONENT_ID3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID3_STM_CTI_COMPONENT_ID3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID3_STM_CTI_COMPONENT_ID3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_STM_CTI_COMPONENT_ID3_RESETVAL                         (0x00000000U)

/* HSM_CM4_CTI_CONTROL */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CONTROL_HSM_CM4_CTI_CONTROL_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CONTROL_HSM_CM4_CTI_CONTROL_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CONTROL_HSM_CM4_CTI_CONTROL_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CONTROL_HSM_CM4_CTI_CONTROL_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CONTROL_RESETVAL                           (0x00000000U)

/* HSM_CM4_CTI_INTACK */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INTACK_HSM_CM4_CTI_INTACK_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INTACK_HSM_CM4_CTI_INTACK_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INTACK_HSM_CM4_CTI_INTACK_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INTACK_HSM_CM4_CTI_INTACK_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INTACK_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_APPSET */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPSET_HSM_CM4_CTI_APPSET_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPSET_HSM_CM4_CTI_APPSET_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPSET_HSM_CM4_CTI_APPSET_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPSET_HSM_CM4_CTI_APPSET_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPSET_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_APPCLEAR */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPCLEAR_HSM_CM4_CTI_APPCLEAR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPCLEAR_HSM_CM4_CTI_APPCLEAR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPCLEAR_HSM_CM4_CTI_APPCLEAR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPCLEAR_HSM_CM4_CTI_APPCLEAR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPCLEAR_RESETVAL                          (0x00000000U)

/* HSM_CM4_CTI_APPPULSE */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPPULSE_HSM_CM4_CTI_APPPULSE_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPPULSE_HSM_CM4_CTI_APPPULSE_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPPULSE_HSM_CM4_CTI_APPPULSE_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPPULSE_HSM_CM4_CTI_APPPULSE_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_APPPULSE_RESETVAL                          (0x00000000U)

/* HSM_CM4_CTI_INEN0 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN0_HSM_CM4_CTI_INEN0_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN0_HSM_CM4_CTI_INEN0_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN0_HSM_CM4_CTI_INEN0_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN0_HSM_CM4_CTI_INEN0_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN0_RESETVAL                             (0x00000000U)

/* HSM_CM4_CTI_INEN1 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN1_HSM_CM4_CTI_INEN1_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN1_HSM_CM4_CTI_INEN1_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN1_HSM_CM4_CTI_INEN1_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN1_HSM_CM4_CTI_INEN1_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN1_RESETVAL                             (0x00000000U)

/* HSM_CM4_CTI_INEN2 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN2_HSM_CM4_CTI_INEN2_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN2_HSM_CM4_CTI_INEN2_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN2_HSM_CM4_CTI_INEN2_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN2_HSM_CM4_CTI_INEN2_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN2_RESETVAL                             (0x00000000U)

/* HSM_CM4_CTI_INEN3 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN3_HSM_CM4_CTI_INEN3_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN3_HSM_CM4_CTI_INEN3_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN3_HSM_CM4_CTI_INEN3_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN3_HSM_CM4_CTI_INEN3_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN3_RESETVAL                             (0x00000000U)

/* HSM_CM4_CTI_INEN4 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN4_HSM_CM4_CTI_INEN4_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN4_HSM_CM4_CTI_INEN4_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN4_HSM_CM4_CTI_INEN4_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN4_HSM_CM4_CTI_INEN4_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN4_RESETVAL                             (0x00000000U)

/* HSM_CM4_CTI_INEN5 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN5_HSM_CM4_CTI_INEN5_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN5_HSM_CM4_CTI_INEN5_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN5_HSM_CM4_CTI_INEN5_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN5_HSM_CM4_CTI_INEN5_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN5_RESETVAL                             (0x00000000U)

/* HSM_CM4_CTI_INEN6 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN6_HSM_CM4_CTI_INEN6_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN6_HSM_CM4_CTI_INEN6_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN6_HSM_CM4_CTI_INEN6_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN6_HSM_CM4_CTI_INEN6_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN6_RESETVAL                             (0x00000000U)

/* HSM_CM4_CTI_INEN7 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN7_HSM_CM4_CTI_INEN7_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN7_HSM_CM4_CTI_INEN7_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN7_HSM_CM4_CTI_INEN7_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN7_HSM_CM4_CTI_INEN7_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_INEN7_RESETVAL                             (0x00000000U)

/* HSM_CM4_CTI_OUTEN0 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN0_HSM_CM4_CTI_OUTEN0_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN0_HSM_CM4_CTI_OUTEN0_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN0_HSM_CM4_CTI_OUTEN0_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN0_HSM_CM4_CTI_OUTEN0_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN0_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_OUTEN1 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN1_HSM_CM4_CTI_OUTEN1_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN1_HSM_CM4_CTI_OUTEN1_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN1_HSM_CM4_CTI_OUTEN1_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN1_HSM_CM4_CTI_OUTEN1_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN1_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_OUTEN2 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN2_HSM_CM4_CTI_OUTEN2_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN2_HSM_CM4_CTI_OUTEN2_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN2_HSM_CM4_CTI_OUTEN2_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN2_HSM_CM4_CTI_OUTEN2_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN2_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_OUTEN3 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN3_HSM_CM4_CTI_OUTEN3_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN3_HSM_CM4_CTI_OUTEN3_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN3_HSM_CM4_CTI_OUTEN3_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN3_HSM_CM4_CTI_OUTEN3_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN3_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_OUTEN4 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN4_HSM_CM4_CTI_OUTEN4_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN4_HSM_CM4_CTI_OUTEN4_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN4_HSM_CM4_CTI_OUTEN4_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN4_HSM_CM4_CTI_OUTEN4_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN4_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_OUTEN5 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN5_HSM_CM4_CTI_OUTEN5_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN5_HSM_CM4_CTI_OUTEN5_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN5_HSM_CM4_CTI_OUTEN5_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN5_HSM_CM4_CTI_OUTEN5_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN5_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_OUTEN6 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN6_HSM_CM4_CTI_OUTEN6_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN6_HSM_CM4_CTI_OUTEN6_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN6_HSM_CM4_CTI_OUTEN6_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN6_HSM_CM4_CTI_OUTEN6_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN6_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_OUTEN7 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN7_HSM_CM4_CTI_OUTEN7_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN7_HSM_CM4_CTI_OUTEN7_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN7_HSM_CM4_CTI_OUTEN7_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN7_HSM_CM4_CTI_OUTEN7_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_OUTEN7_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_TRIGINSTATUS */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGINSTATUS_HSM_CM4_CTI_TRIGINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGINSTATUS_HSM_CM4_CTI_TRIGINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGINSTATUS_HSM_CM4_CTI_TRIGINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGINSTATUS_HSM_CM4_CTI_TRIGINSTATUS_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGINSTATUS_RESETVAL                      (0x00000000U)

/* HSM_CM4_CTI_TRIGOUTSTATUS */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGOUTSTATUS_HSM_CM4_CTI_TRIGOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGOUTSTATUS_HSM_CM4_CTI_TRIGOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGOUTSTATUS_HSM_CM4_CTI_TRIGOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGOUTSTATUS_HSM_CM4_CTI_TRIGOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_TRIGOUTSTATUS_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_CHINSTATUS */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHINSTATUS_HSM_CM4_CTI_CHINSTATUS_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHINSTATUS_HSM_CM4_CTI_CHINSTATUS_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHINSTATUS_HSM_CM4_CTI_CHINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHINSTATUS_HSM_CM4_CTI_CHINSTATUS_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHINSTATUS_RESETVAL                        (0x00000000U)

/* HSM_CM4_CTI_CHOUTSTATUS */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHOUTSTATUS_HSM_CM4_CTI_CHOUTSTATUS_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHOUTSTATUS_HSM_CM4_CTI_CHOUTSTATUS_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHOUTSTATUS_HSM_CM4_CTI_CHOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHOUTSTATUS_HSM_CM4_CTI_CHOUTSTATUS_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CHOUTSTATUS_RESETVAL                       (0x00000000U)

/* HSM_CM4_CTI_GATE */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_GATE_HSM_CM4_CTI_GATE_MASK                 (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_GATE_HSM_CM4_CTI_GATE_SHIFT                (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_GATE_HSM_CM4_CTI_GATE_RESETVAL             (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_GATE_HSM_CM4_CTI_GATE_MAX                  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_GATE_RESETVAL                              (0x00000000U)

/* HSM_CM4_CTI_ASICCTL */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ASICCTL_HSM_CM4_CTI_ASICCTL_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ASICCTL_HSM_CM4_CTI_ASICCTL_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ASICCTL_HSM_CM4_CTI_ASICCTL_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ASICCTL_HSM_CM4_CTI_ASICCTL_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ASICCTL_RESETVAL                           (0x00000000U)

/* HSM_CM4_CTI_ITCHINACK */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHINACK_HSM_CM4_CTI_ITCHINACK_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHINACK_HSM_CM4_CTI_ITCHINACK_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHINACK_HSM_CM4_CTI_ITCHINACK_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHINACK_HSM_CM4_CTI_ITCHINACK_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHINACK_RESETVAL                         (0x00000000U)

/* HSM_CM4_CTI_ITTRIGINACK */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGINACK_HSM_CM4_CTI_ITTRIGINACK_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGINACK_HSM_CM4_CTI_ITTRIGINACK_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGINACK_HSM_CM4_CTI_ITTRIGINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGINACK_HSM_CM4_CTI_ITTRIGINACK_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGINACK_RESETVAL                       (0x00000000U)

/* HSM_CM4_CTI_ITCHOUT */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUT_HSM_CM4_CTI_ITCHOUT_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUT_HSM_CM4_CTI_ITCHOUT_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUT_HSM_CM4_CTI_ITCHOUT_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUT_HSM_CM4_CTI_ITCHOUT_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUT_RESETVAL                           (0x00000000U)

/* HSM_CM4_CTI_ITTRIGOUT */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUT_HSM_CM4_CTI_ITTRIGOUT_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUT_HSM_CM4_CTI_ITTRIGOUT_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUT_HSM_CM4_CTI_ITTRIGOUT_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUT_HSM_CM4_CTI_ITTRIGOUT_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUT_RESETVAL                         (0x00000000U)

/* HSM_CM4_CTI_ITCHOUTACK */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUTACK_HSM_CM4_CTI_ITCHOUTACK_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUTACK_HSM_CM4_CTI_ITCHOUTACK_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUTACK_HSM_CM4_CTI_ITCHOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUTACK_HSM_CM4_CTI_ITCHOUTACK_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHOUTACK_RESETVAL                        (0x00000000U)

/* HSM_CM4_CTI_ITTRIGOUTACK */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUTACK_HSM_CM4_CTI_ITTRIGOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUTACK_HSM_CM4_CTI_ITTRIGOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUTACK_HSM_CM4_CTI_ITTRIGOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUTACK_HSM_CM4_CTI_ITTRIGOUTACK_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGOUTACK_RESETVAL                      (0x00000000U)

/* HSM_CM4_CTI_ITCHIN */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHIN_HSM_CM4_CTI_ITCHIN_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHIN_HSM_CM4_CTI_ITCHIN_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHIN_HSM_CM4_CTI_ITCHIN_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHIN_HSM_CM4_CTI_ITCHIN_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCHIN_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_ITTRIGIN */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGIN_HSM_CM4_CTI_ITTRIGIN_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGIN_HSM_CM4_CTI_ITTRIGIN_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGIN_HSM_CM4_CTI_ITTRIGIN_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGIN_HSM_CM4_CTI_ITTRIGIN_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITTRIGIN_RESETVAL                          (0x00000000U)

/* HSM_CM4_CTI_ITCTRL */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCTRL_HSM_CM4_CTI_ITCTRL_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCTRL_HSM_CM4_CTI_ITCTRL_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCTRL_HSM_CM4_CTI_ITCTRL_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCTRL_HSM_CM4_CTI_ITCTRL_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_ITCTRL_RESETVAL                            (0x00000000U)

/* HSM_CM4_CTI_CLAIM_TAG_SET */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_SET_HSM_CM4_CTI_CLAIM_TAG_SET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_SET_HSM_CM4_CTI_CLAIM_TAG_SET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_SET_HSM_CM4_CTI_CLAIM_TAG_SET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_SET_HSM_CM4_CTI_CLAIM_TAG_SET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_SET_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_CLAIM_TAG_CLEAR */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_CLEAR_HSM_CM4_CTI_CLAIM_TAG_CLEAR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_CLEAR_HSM_CM4_CTI_CLAIM_TAG_CLEAR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_CLEAR_HSM_CM4_CTI_CLAIM_TAG_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_CLEAR_HSM_CM4_CTI_CLAIM_TAG_CLEAR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_CLAIM_TAG_CLEAR_RESETVAL                   (0x00000000U)

/* HSM_CM4_CTI_LOCK_ACCESS_REGISTER */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_ACCESS_REGISTER_RESETVAL              (0x00000000U)

/* HSM_CM4_CTI_LOCK_STATUS_REGISTER */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_STATUS_REGISTER_HSM_CM4_CTI_LOCK_STATUS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_STATUS_REGISTER_HSM_CM4_CTI_LOCK_STATUS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_STATUS_REGISTER_HSM_CM4_CTI_LOCK_STATUS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_STATUS_REGISTER_HSM_CM4_CTI_LOCK_STATUS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_LOCK_STATUS_REGISTER_RESETVAL              (0x00000000U)

/* HSM_CM4_CTI_AUTHENTICATION_STATUS */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_AUTHENTICATION_STATUS_HSM_CM4_CTI_AUTHENTICATION_STATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_AUTHENTICATION_STATUS_HSM_CM4_CTI_AUTHENTICATION_STATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_AUTHENTICATION_STATUS_HSM_CM4_CTI_AUTHENTICATION_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_AUTHENTICATION_STATUS_HSM_CM4_CTI_AUTHENTICATION_STATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_AUTHENTICATION_STATUS_RESETVAL             (0x00000000U)

/* HSM_CM4_CTI_DEVICE_ID */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_ID_HSM_CM4_CTI_DEVICE_ID_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_ID_HSM_CM4_CTI_DEVICE_ID_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_ID_HSM_CM4_CTI_DEVICE_ID_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_ID_HSM_CM4_CTI_DEVICE_ID_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_ID_RESETVAL                         (0x00000000U)

/* HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL            (0x00000000U)

/* HSM_CM4_CTI_PERIPHERALID4 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID4_HSM_CM4_CTI_PERIPHERALID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID4_HSM_CM4_CTI_PERIPHERALID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID4_HSM_CM4_CTI_PERIPHERALID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID4_HSM_CM4_CTI_PERIPHERALID4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID4_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_PERIPHERALID5 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID5_HSM_CM4_CTI_PERIPHERALID5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID5_HSM_CM4_CTI_PERIPHERALID5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID5_HSM_CM4_CTI_PERIPHERALID5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID5_HSM_CM4_CTI_PERIPHERALID5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID5_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_PERIPHERALID6 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID6_HSM_CM4_CTI_PERIPHERALID6_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID6_HSM_CM4_CTI_PERIPHERALID6_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID6_HSM_CM4_CTI_PERIPHERALID6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID6_HSM_CM4_CTI_PERIPHERALID6_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID6_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_PERIPHERALID7 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID7_HSM_CM4_CTI_PERIPHERALID7_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID7_HSM_CM4_CTI_PERIPHERALID7_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID7_HSM_CM4_CTI_PERIPHERALID7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID7_HSM_CM4_CTI_PERIPHERALID7_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID7_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_PERIPHERALID0 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID0_HSM_CM4_CTI_PERIPHERALID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID0_HSM_CM4_CTI_PERIPHERALID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID0_HSM_CM4_CTI_PERIPHERALID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID0_HSM_CM4_CTI_PERIPHERALID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID0_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_PERIPHERALID1 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID1_HSM_CM4_CTI_PERIPHERALID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID1_HSM_CM4_CTI_PERIPHERALID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID1_HSM_CM4_CTI_PERIPHERALID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID1_HSM_CM4_CTI_PERIPHERALID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID1_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_PERIPHERALID2 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID2_HSM_CM4_CTI_PERIPHERALID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID2_HSM_CM4_CTI_PERIPHERALID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID2_HSM_CM4_CTI_PERIPHERALID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID2_HSM_CM4_CTI_PERIPHERALID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID2_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_PERIPHERALID3 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID3_HSM_CM4_CTI_PERIPHERALID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID3_HSM_CM4_CTI_PERIPHERALID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID3_HSM_CM4_CTI_PERIPHERALID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID3_HSM_CM4_CTI_PERIPHERALID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_PERIPHERALID3_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_COMPONENT_ID0 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID0_HSM_CM4_CTI_COMPONENT_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID0_HSM_CM4_CTI_COMPONENT_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID0_HSM_CM4_CTI_COMPONENT_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID0_HSM_CM4_CTI_COMPONENT_ID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID0_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_COMPONENT_ID1 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID1_HSM_CM4_CTI_COMPONENT_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID1_HSM_CM4_CTI_COMPONENT_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID1_HSM_CM4_CTI_COMPONENT_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID1_HSM_CM4_CTI_COMPONENT_ID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID1_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_COMPONENT_ID2 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID2_HSM_CM4_CTI_COMPONENT_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID2_HSM_CM4_CTI_COMPONENT_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID2_HSM_CM4_CTI_COMPONENT_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID2_HSM_CM4_CTI_COMPONENT_ID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID2_RESETVAL                     (0x00000000U)

/* HSM_CM4_CTI_COMPONENT_ID3 */

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID3_HSM_CM4_CTI_COMPONENT_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID3_HSM_CM4_CTI_COMPONENT_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID3_HSM_CM4_CTI_COMPONENT_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID3_HSM_CM4_CTI_COMPONENT_ID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_HSM_CM4_CTI_COMPONENT_ID3_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_ROM */


/* MSS_R5SS0_A_APB_DBGDIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDIDR_MSS_R5SS0_APB_DBGDIDR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDIDR_MSS_R5SS0_APB_DBGDIDR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDIDR_MSS_R5SS0_APB_DBGDIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDIDR_MSS_R5SS0_APB_DBGDIDR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDIDR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWFAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWFAR_MSS_R5SS0_APB_DBGWFAR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWFAR_MSS_R5SS0_APB_DBGWFAR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWFAR_MSS_R5SS0_APB_DBGWFAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWFAR_MSS_R5SS0_APB_DBGWFAR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWFAR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGVCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGVCR_MSS_R5SS0_APB_DBGVCR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGVCR_MSS_R5SS0_APB_DBGVCR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGVCR_MSS_R5SS0_APB_DBGVCR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGVCR_MSS_R5SS0_APB_DBGVCR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGVCR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_APB_DBGECR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGECR_MSS_R5SS0_APB_DBGECR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGECR_MSS_R5SS0_APB_DBGECR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGECR_MSS_R5SS0_APB_DBGECR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGECR_MSS_R5SS0_APB_DBGECR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGECR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_APB_DBGDSCCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCCR_MSS_R5SS0_APB_DBGDSCCR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCCR_MSS_R5SS0_APB_DBGDSCCR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCCR_MSS_R5SS0_APB_DBGDSCCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCCR_MSS_R5SS0_APB_DBGDSCCR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCCR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_DBGDTRRX */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRRX_MSS_R5SS0_APB_DBGDTRRX_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRRX_MSS_R5SS0_APB_DBGDTRRX_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRRX_MSS_R5SS0_APB_DBGDTRRX_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRRX_MSS_R5SS0_APB_DBGDTRRX_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRRX_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_DBGITR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGITR_MSS_R5SS0_APB_DBGITR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGITR_MSS_R5SS0_APB_DBGITR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGITR_MSS_R5SS0_APB_DBGITR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGITR_MSS_R5SS0_APB_DBGITR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGITR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_APB_DBGDSCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCR_MSS_R5SS0_APB_DBGDSCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCR_MSS_R5SS0_APB_DBGDSCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCR_MSS_R5SS0_APB_DBGDSCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCR_MSS_R5SS0_APB_DBGDSCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDSCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGDTRTX */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRTX_MSS_R5SS0_APB_DBGDTRTX_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRTX_MSS_R5SS0_APB_DBGDTRTX_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRTX_MSS_R5SS0_APB_DBGDTRTX_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRTX_MSS_R5SS0_APB_DBGDTRTX_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDTRTX_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_DBGDRCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDRCR_MSS_R5SS0_APB_DBGDRCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDRCR_MSS_R5SS0_APB_DBGDRCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDRCR_MSS_R5SS0_APB_DBGDRCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDRCR_MSS_R5SS0_APB_DBGDRCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGDRCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBVR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR0_MSS_R5SS0_APB_DBGBVR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR0_MSS_R5SS0_APB_DBGBVR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR0_MSS_R5SS0_APB_DBGBVR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR0_MSS_R5SS0_APB_DBGBVR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR1_MSS_R5SS0_APB_DBGBVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR1_MSS_R5SS0_APB_DBGBVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR1_MSS_R5SS0_APB_DBGBVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR1_MSS_R5SS0_APB_DBGBVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR2_MSS_R5SS0_APB_DBGBVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR2_MSS_R5SS0_APB_DBGBVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR2_MSS_R5SS0_APB_DBGBVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR2_MSS_R5SS0_APB_DBGBVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR3_MSS_R5SS0_APB_DBGBVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR3_MSS_R5SS0_APB_DBGBVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR3_MSS_R5SS0_APB_DBGBVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR3_MSS_R5SS0_APB_DBGBVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR4_MSS_R5SS0_APB_DBGBVR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR4_MSS_R5SS0_APB_DBGBVR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR4_MSS_R5SS0_APB_DBGBVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR4_MSS_R5SS0_APB_DBGBVR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR5_MSS_R5SS0_APB_DBGBVR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR5_MSS_R5SS0_APB_DBGBVR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR5_MSS_R5SS0_APB_DBGBVR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR5_MSS_R5SS0_APB_DBGBVR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR6_MSS_R5SS0_APB_DBGBVR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR6_MSS_R5SS0_APB_DBGBVR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR6_MSS_R5SS0_APB_DBGBVR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR6_MSS_R5SS0_APB_DBGBVR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR7_MSS_R5SS0_APB_DBGBVR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR7_MSS_R5SS0_APB_DBGBVR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR7_MSS_R5SS0_APB_DBGBVR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR7_MSS_R5SS0_APB_DBGBVR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBVR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBCR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR0_MSS_R5SS0_APB_DBGBCR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR0_MSS_R5SS0_APB_DBGBCR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR0_MSS_R5SS0_APB_DBGBCR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR0_MSS_R5SS0_APB_DBGBCR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR1_MSS_R5SS0_APB_DBGBCR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR1_MSS_R5SS0_APB_DBGBCR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR1_MSS_R5SS0_APB_DBGBCR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR1_MSS_R5SS0_APB_DBGBCR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR2_MSS_R5SS0_APB_DBGBCR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR2_MSS_R5SS0_APB_DBGBCR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR2_MSS_R5SS0_APB_DBGBCR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR2_MSS_R5SS0_APB_DBGBCR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR3_MSS_R5SS0_APB_DBGBCR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR3_MSS_R5SS0_APB_DBGBCR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR3_MSS_R5SS0_APB_DBGBCR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR3_MSS_R5SS0_APB_DBGBCR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBCR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR4_MSS_R5SS0_APB_DBGBCR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR4_MSS_R5SS0_APB_DBGBCR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR4_MSS_R5SS0_APB_DBGBCR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR4_MSS_R5SS0_APB_DBGBCR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBCR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR5_MSS_R5SS0_APB_DBGBCR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR5_MSS_R5SS0_APB_DBGBCR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR5_MSS_R5SS0_APB_DBGBCR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR5_MSS_R5SS0_APB_DBGBCR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBCR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR6_MSS_R5SS0_APB_DBGBCR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR6_MSS_R5SS0_APB_DBGBCR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR6_MSS_R5SS0_APB_DBGBCR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR6_MSS_R5SS0_APB_DBGBCR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGBCR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR7_MSS_R5SS0_APB_DBGBCR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR7_MSS_R5SS0_APB_DBGBCR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR7_MSS_R5SS0_APB_DBGBCR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR7_MSS_R5SS0_APB_DBGBCR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGBCR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWVR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR0_MSS_R5SS0_APB_DBGWVR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR0_MSS_R5SS0_APB_DBGWVR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR0_MSS_R5SS0_APB_DBGWVR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR0_MSS_R5SS0_APB_DBGWVR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR1_MSS_R5SS0_APB_DBGWVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR1_MSS_R5SS0_APB_DBGWVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR1_MSS_R5SS0_APB_DBGWVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR1_MSS_R5SS0_APB_DBGWVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR2_MSS_R5SS0_APB_DBGWVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR2_MSS_R5SS0_APB_DBGWVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR2_MSS_R5SS0_APB_DBGWVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR2_MSS_R5SS0_APB_DBGWVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR3_MSS_R5SS0_APB_DBGWVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR3_MSS_R5SS0_APB_DBGWVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR3_MSS_R5SS0_APB_DBGWVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR3_MSS_R5SS0_APB_DBGWVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR4_MSS_R5SS0_APB_DBGWVR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR4_MSS_R5SS0_APB_DBGWVR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR4_MSS_R5SS0_APB_DBGWVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR4_MSS_R5SS0_APB_DBGWVR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR5_MSS_R5SS0_APB_DBGWVR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR5_MSS_R5SS0_APB_DBGWVR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR5_MSS_R5SS0_APB_DBGWVR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR5_MSS_R5SS0_APB_DBGWVR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR6_MSS_R5SS0_APB_DBGWVR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR6_MSS_R5SS0_APB_DBGWVR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR6_MSS_R5SS0_APB_DBGWVR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR6_MSS_R5SS0_APB_DBGWVR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR7_MSS_R5SS0_APB_DBGWVR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR7_MSS_R5SS0_APB_DBGWVR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR7_MSS_R5SS0_APB_DBGWVR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR7_MSS_R5SS0_APB_DBGWVR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWVR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWCR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR0_MSS_R5SS0_APB_DBGWCR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR0_MSS_R5SS0_APB_DBGWCR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR0_MSS_R5SS0_APB_DBGWCR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR0_MSS_R5SS0_APB_DBGWCR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR1_MSS_R5SS0_APB_DBGWCR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR1_MSS_R5SS0_APB_DBGWCR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR1_MSS_R5SS0_APB_DBGWCR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR1_MSS_R5SS0_APB_DBGWCR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR2_MSS_R5SS0_APB_DBGWCR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR2_MSS_R5SS0_APB_DBGWCR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR2_MSS_R5SS0_APB_DBGWCR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR2_MSS_R5SS0_APB_DBGWCR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR3_MSS_R5SS0_APB_DBGWCR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR3_MSS_R5SS0_APB_DBGWCR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR3_MSS_R5SS0_APB_DBGWCR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR3_MSS_R5SS0_APB_DBGWCR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWCR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR4_MSS_R5SS0_APB_DBGWCR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR4_MSS_R5SS0_APB_DBGWCR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR4_MSS_R5SS0_APB_DBGWCR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR4_MSS_R5SS0_APB_DBGWCR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWCR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR5_MSS_R5SS0_APB_DBGWCR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR5_MSS_R5SS0_APB_DBGWCR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR5_MSS_R5SS0_APB_DBGWCR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR5_MSS_R5SS0_APB_DBGWCR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWCR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR6_MSS_R5SS0_APB_DBGWCR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR6_MSS_R5SS0_APB_DBGWCR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR6_MSS_R5SS0_APB_DBGWCR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR6_MSS_R5SS0_APB_DBGWCR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGWCR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR7_MSS_R5SS0_APB_DBGWCR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR7_MSS_R5SS0_APB_DBGWCR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR7_MSS_R5SS0_APB_DBGWCR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR7_MSS_R5SS0_APB_DBGWCR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGWCR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGOSLAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLAR_MSS_R5SS0_APB_DBGOSLAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLAR_MSS_R5SS0_APB_DBGOSLAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLAR_MSS_R5SS0_APB_DBGOSLAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLAR_MSS_R5SS0_APB_DBGOSLAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLAR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_DBGOSLSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLSR_MSS_R5SS0_APB_DBGOSLSR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLSR_MSS_R5SS0_APB_DBGOSLSR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLSR_MSS_R5SS0_APB_DBGOSLSR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLSR_MSS_R5SS0_APB_DBGOSLSR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSLSR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_DBGOSSRR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSSRR_MSS_R5SS0_APB_DBGOSSRR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSSRR_MSS_R5SS0_APB_DBGOSSRR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSSRR_MSS_R5SS0_APB_DBGOSSRR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSSRR_MSS_R5SS0_APB_DBGOSSRR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGOSSRR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_DBGPRCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRCR_MSS_R5SS0_APB_DBGPRCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRCR_MSS_R5SS0_APB_DBGPRCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRCR_MSS_R5SS0_APB_DBGPRCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRCR_MSS_R5SS0_APB_DBGPRCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_DBGPRSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRSR_MSS_R5SS0_APB_DBGPRSR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRSR_MSS_R5SS0_APB_DBGPRSR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRSR_MSS_R5SS0_APB_DBGPRSR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRSR_MSS_R5SS0_APB_DBGPRSR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_DBGPRSR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_MIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MIDR_MSS_R5SS0_APB_PROCID_MIDR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MIDR_MSS_R5SS0_APB_PROCID_MIDR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MIDR_MSS_R5SS0_APB_PROCID_MIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MIDR_MSS_R5SS0_APB_PROCID_MIDR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MIDR_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_CTR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_CTR_MSS_R5SS0_APB_PROCID_CTR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_CTR_MSS_R5SS0_APB_PROCID_CTR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_CTR_MSS_R5SS0_APB_PROCID_CTR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_CTR_MSS_R5SS0_APB_PROCID_CTR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_CTR_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_TCMTR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_TCMTR_MSS_R5SS0_APB_PROCID_TCMTR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_TCMTR_MSS_R5SS0_APB_PROCID_TCMTR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_TCMTR_MSS_R5SS0_APB_PROCID_TCMTR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_TCMTR_MSS_R5SS0_APB_PROCID_TCMTR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_TCMTR_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_MPUIR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPUIR_MSS_R5SS0_APB_PROCID_MPUIR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPUIR_MSS_R5SS0_APB_PROCID_MPUIR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPUIR_MSS_R5SS0_APB_PROCID_MPUIR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPUIR_MSS_R5SS0_APB_PROCID_MPUIR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPUIR_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_MPIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPIDR_MSS_R5SS0_APB_PROCID_MPIDR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPIDR_MSS_R5SS0_APB_PROCID_MPIDR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPIDR_MSS_R5SS0_APB_PROCID_MPIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPIDR_MSS_R5SS0_APB_PROCID_MPIDR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MPIDR_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_PFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR0_MSS_R5SS0_APB_PROCID_PFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR0_MSS_R5SS0_APB_PROCID_PFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR0_MSS_R5SS0_APB_PROCID_PFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR0_MSS_R5SS0_APB_PROCID_PFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_PFR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR1_MSS_R5SS0_APB_PROCID_PFR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR1_MSS_R5SS0_APB_PROCID_PFR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR1_MSS_R5SS0_APB_PROCID_PFR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR1_MSS_R5SS0_APB_PROCID_PFR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_PFR1_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_DFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_DFR0_MSS_R5SS0_APB_PROCID_DFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_DFR0_MSS_R5SS0_APB_PROCID_DFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_DFR0_MSS_R5SS0_APB_PROCID_DFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_DFR0_MSS_R5SS0_APB_PROCID_DFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_DFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_AFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_AFR0_MSS_R5SS0_APB_PROCID_AFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_AFR0_MSS_R5SS0_APB_PROCID_AFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_AFR0_MSS_R5SS0_APB_PROCID_AFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_AFR0_MSS_R5SS0_APB_PROCID_AFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_AFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_MMFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR0_MSS_R5SS0_APB_PROCID_MMFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR0_MSS_R5SS0_APB_PROCID_MMFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR0_MSS_R5SS0_APB_PROCID_MMFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR0_MSS_R5SS0_APB_PROCID_MMFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR0_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_MMFR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR1_MSS_R5SS0_APB_PROCID_MMFR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR1_MSS_R5SS0_APB_PROCID_MMFR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR1_MSS_R5SS0_APB_PROCID_MMFR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR1_MSS_R5SS0_APB_PROCID_MMFR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR1_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_MMFR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR2_MSS_R5SS0_APB_PROCID_MMFR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR2_MSS_R5SS0_APB_PROCID_MMFR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR2_MSS_R5SS0_APB_PROCID_MMFR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR2_MSS_R5SS0_APB_PROCID_MMFR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR2_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_MMFR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR3_MSS_R5SS0_APB_PROCID_MMFR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR3_MSS_R5SS0_APB_PROCID_MMFR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR3_MSS_R5SS0_APB_PROCID_MMFR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR3_MSS_R5SS0_APB_PROCID_MMFR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_MMFR3_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_ISAR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR0_MSS_R5SS0_APB_PROCID_ISAR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR0_MSS_R5SS0_APB_PROCID_ISAR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR0_MSS_R5SS0_APB_PROCID_ISAR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR0_MSS_R5SS0_APB_PROCID_ISAR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR0_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_ISAR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR1_MSS_R5SS0_APB_PROCID_ISAR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR1_MSS_R5SS0_APB_PROCID_ISAR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR1_MSS_R5SS0_APB_PROCID_ISAR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR1_MSS_R5SS0_APB_PROCID_ISAR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR1_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_ISAR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR2_MSS_R5SS0_APB_PROCID_ISAR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR2_MSS_R5SS0_APB_PROCID_ISAR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR2_MSS_R5SS0_APB_PROCID_ISAR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR2_MSS_R5SS0_APB_PROCID_ISAR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR2_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_ISAR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR3_MSS_R5SS0_APB_PROCID_ISAR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR3_MSS_R5SS0_APB_PROCID_ISAR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR3_MSS_R5SS0_APB_PROCID_ISAR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR3_MSS_R5SS0_APB_PROCID_ISAR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR3_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_ISAR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR4_MSS_R5SS0_APB_PROCID_ISAR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR4_MSS_R5SS0_APB_PROCID_ISAR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR4_MSS_R5SS0_APB_PROCID_ISAR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR4_MSS_R5SS0_APB_PROCID_ISAR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR4_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_PROCID_ISAR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR5_MSS_R5SS0_APB_PROCID_ISAR5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR5_MSS_R5SS0_APB_PROCID_ISAR5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR5_MSS_R5SS0_APB_PROCID_ISAR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR5_MSS_R5SS0_APB_PROCID_ISAR5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PROCID_ISAR5_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_APB_MR_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_ITCTRL_MSS_R5SS0_APB_MR_ITCTRL_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_ITCTRL_MSS_R5SS0_APB_MR_ITCTRL_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_ITCTRL_MSS_R5SS0_APB_MR_ITCTRL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_ITCTRL_MSS_R5SS0_APB_MR_ITCTRL_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_ITCTRL_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_APB_MR_CLAIMSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMSET_MSS_R5SS0_APB_MR_CLAIMSET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMSET_MSS_R5SS0_APB_MR_CLAIMSET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMSET_MSS_R5SS0_APB_MR_CLAIMSET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMSET_MSS_R5SS0_APB_MR_CLAIMSET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMSET_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_APB_MR_CLAIMCLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMCLR_MSS_R5SS0_APB_MR_CLAIMCLR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMCLR_MSS_R5SS0_APB_MR_CLAIMCLR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMCLR_MSS_R5SS0_APB_MR_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMCLR_MSS_R5SS0_APB_MR_CLAIMCLR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_CLAIMCLR_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_APB_MR_LOCKACCESS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKACCESS_MSS_R5SS0_APB_MR_LOCKACCESS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKACCESS_MSS_R5SS0_APB_MR_LOCKACCESS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKACCESS_MSS_R5SS0_APB_MR_LOCKACCESS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKACCESS_MSS_R5SS0_APB_MR_LOCKACCESS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKACCESS_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_APB_MR_LOCKSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKSTATUS_MSS_R5SS0_APB_MR_LOCKSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKSTATUS_MSS_R5SS0_APB_MR_LOCKSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKSTATUS_MSS_R5SS0_APB_MR_LOCKSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKSTATUS_MSS_R5SS0_APB_MR_LOCKSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_LOCKSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_APB_MR_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_AUTHSTATUS_MSS_R5SS0_APB_MR_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_AUTHSTATUS_MSS_R5SS0_APB_MR_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_AUTHSTATUS_MSS_R5SS0_APB_MR_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_AUTHSTATUS_MSS_R5SS0_APB_MR_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_AUTHSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_APB_MR_DEVID */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVID_MSS_R5SS0_APB_MR_DEVID_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVID_MSS_R5SS0_APB_MR_DEVID_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVID_MSS_R5SS0_APB_MR_DEVID_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVID_MSS_R5SS0_APB_MR_DEVID_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVID_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_MR_DEVTYPE */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVTYPE_MSS_R5SS0_APB_MR_DEVTYPE_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVTYPE_MSS_R5SS0_APB_MR_DEVTYPE_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVTYPE_MSS_R5SS0_APB_MR_DEVTYPE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVTYPE_MSS_R5SS0_APB_MR_DEVTYPE_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_MR_DEVTYPE_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_APB_PERIP_ID4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID4_MSS_R5SS0_APB_PERIP_ID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID4_MSS_R5SS0_APB_PERIP_ID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID4_MSS_R5SS0_APB_PERIP_ID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID4_MSS_R5SS0_APB_PERIP_ID4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID4_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_APB_PERIP_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID0_MSS_R5SS0_APB_PERIP_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID0_MSS_R5SS0_APB_PERIP_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID0_MSS_R5SS0_APB_PERIP_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID0_MSS_R5SS0_APB_PERIP_ID0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID0_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_APB_PERIP_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID1_MSS_R5SS0_APB_PERIP_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID1_MSS_R5SS0_APB_PERIP_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID1_MSS_R5SS0_APB_PERIP_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID1_MSS_R5SS0_APB_PERIP_ID1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID1_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_APB_PERIP_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID2_MSS_R5SS0_APB_PERIP_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID2_MSS_R5SS0_APB_PERIP_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID2_MSS_R5SS0_APB_PERIP_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID2_MSS_R5SS0_APB_PERIP_ID2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID2_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_APB_PERIP_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID3_MSS_R5SS0_APB_PERIP_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID3_MSS_R5SS0_APB_PERIP_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID3_MSS_R5SS0_APB_PERIP_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID3_MSS_R5SS0_APB_PERIP_ID3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_PERIP_ID3_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_APB_COMP_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID0_MSS_R5SS0_APB_COMP_ID0_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID0_MSS_R5SS0_APB_COMP_ID0_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID0_MSS_R5SS0_APB_COMP_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID0_MSS_R5SS0_APB_COMP_ID0_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID0_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_COMP_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID1_MSS_R5SS0_APB_COMP_ID1_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID1_MSS_R5SS0_APB_COMP_ID1_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID1_MSS_R5SS0_APB_COMP_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID1_MSS_R5SS0_APB_COMP_ID1_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID1_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_COMP_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID2_MSS_R5SS0_APB_COMP_ID2_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID2_MSS_R5SS0_APB_COMP_ID2_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID2_MSS_R5SS0_APB_COMP_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID2_MSS_R5SS0_APB_COMP_ID2_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID2_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_APB_COMP_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID3_MSS_R5SS0_APB_COMP_ID3_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID3_MSS_R5SS0_APB_COMP_ID3_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID3_MSS_R5SS0_APB_COMP_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID3_MSS_R5SS0_APB_COMP_ID3_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_APB_COMP_ID3_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_DBGDIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDIDR_MSS_R5SS0_APB_DBGDIDR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDIDR_MSS_R5SS0_APB_DBGDIDR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDIDR_MSS_R5SS0_APB_DBGDIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDIDR_MSS_R5SS0_APB_DBGDIDR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDIDR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWFAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWFAR_MSS_R5SS0_APB_DBGWFAR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWFAR_MSS_R5SS0_APB_DBGWFAR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWFAR_MSS_R5SS0_APB_DBGWFAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWFAR_MSS_R5SS0_APB_DBGWFAR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWFAR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGVCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGVCR_MSS_R5SS0_APB_DBGVCR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGVCR_MSS_R5SS0_APB_DBGVCR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGVCR_MSS_R5SS0_APB_DBGVCR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGVCR_MSS_R5SS0_APB_DBGVCR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGVCR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_APB_DBGECR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGECR_MSS_R5SS0_APB_DBGECR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGECR_MSS_R5SS0_APB_DBGECR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGECR_MSS_R5SS0_APB_DBGECR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGECR_MSS_R5SS0_APB_DBGECR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGECR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_APB_DBGDSCCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCCR_MSS_R5SS0_APB_DBGDSCCR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCCR_MSS_R5SS0_APB_DBGDSCCR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCCR_MSS_R5SS0_APB_DBGDSCCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCCR_MSS_R5SS0_APB_DBGDSCCR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCCR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_DBGDTRRX */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRRX_MSS_R5SS0_APB_DBGDTRRX_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRRX_MSS_R5SS0_APB_DBGDTRRX_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRRX_MSS_R5SS0_APB_DBGDTRRX_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRRX_MSS_R5SS0_APB_DBGDTRRX_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRRX_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_DBGITR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGITR_MSS_R5SS0_APB_DBGITR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGITR_MSS_R5SS0_APB_DBGITR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGITR_MSS_R5SS0_APB_DBGITR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGITR_MSS_R5SS0_APB_DBGITR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGITR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_APB_DBGDSCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCR_MSS_R5SS0_APB_DBGDSCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCR_MSS_R5SS0_APB_DBGDSCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCR_MSS_R5SS0_APB_DBGDSCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCR_MSS_R5SS0_APB_DBGDSCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDSCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGDTRTX */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRTX_MSS_R5SS0_APB_DBGDTRTX_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRTX_MSS_R5SS0_APB_DBGDTRTX_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRTX_MSS_R5SS0_APB_DBGDTRTX_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRTX_MSS_R5SS0_APB_DBGDTRTX_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDTRTX_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_DBGDRCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDRCR_MSS_R5SS0_APB_DBGDRCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDRCR_MSS_R5SS0_APB_DBGDRCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDRCR_MSS_R5SS0_APB_DBGDRCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDRCR_MSS_R5SS0_APB_DBGDRCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGDRCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBVR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR0_MSS_R5SS0_APB_DBGBVR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR0_MSS_R5SS0_APB_DBGBVR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR0_MSS_R5SS0_APB_DBGBVR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR0_MSS_R5SS0_APB_DBGBVR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR1_MSS_R5SS0_APB_DBGBVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR1_MSS_R5SS0_APB_DBGBVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR1_MSS_R5SS0_APB_DBGBVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR1_MSS_R5SS0_APB_DBGBVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR2_MSS_R5SS0_APB_DBGBVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR2_MSS_R5SS0_APB_DBGBVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR2_MSS_R5SS0_APB_DBGBVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR2_MSS_R5SS0_APB_DBGBVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR3_MSS_R5SS0_APB_DBGBVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR3_MSS_R5SS0_APB_DBGBVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR3_MSS_R5SS0_APB_DBGBVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR3_MSS_R5SS0_APB_DBGBVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR4_MSS_R5SS0_APB_DBGBVR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR4_MSS_R5SS0_APB_DBGBVR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR4_MSS_R5SS0_APB_DBGBVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR4_MSS_R5SS0_APB_DBGBVR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR5_MSS_R5SS0_APB_DBGBVR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR5_MSS_R5SS0_APB_DBGBVR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR5_MSS_R5SS0_APB_DBGBVR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR5_MSS_R5SS0_APB_DBGBVR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR6_MSS_R5SS0_APB_DBGBVR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR6_MSS_R5SS0_APB_DBGBVR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR6_MSS_R5SS0_APB_DBGBVR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR6_MSS_R5SS0_APB_DBGBVR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR7_MSS_R5SS0_APB_DBGBVR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR7_MSS_R5SS0_APB_DBGBVR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR7_MSS_R5SS0_APB_DBGBVR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR7_MSS_R5SS0_APB_DBGBVR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBVR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBCR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR0_MSS_R5SS0_APB_DBGBCR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR0_MSS_R5SS0_APB_DBGBCR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR0_MSS_R5SS0_APB_DBGBCR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR0_MSS_R5SS0_APB_DBGBCR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR1_MSS_R5SS0_APB_DBGBCR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR1_MSS_R5SS0_APB_DBGBCR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR1_MSS_R5SS0_APB_DBGBCR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR1_MSS_R5SS0_APB_DBGBCR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR2_MSS_R5SS0_APB_DBGBCR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR2_MSS_R5SS0_APB_DBGBCR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR2_MSS_R5SS0_APB_DBGBCR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR2_MSS_R5SS0_APB_DBGBCR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR3_MSS_R5SS0_APB_DBGBCR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR3_MSS_R5SS0_APB_DBGBCR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR3_MSS_R5SS0_APB_DBGBCR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR3_MSS_R5SS0_APB_DBGBCR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBCR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR4_MSS_R5SS0_APB_DBGBCR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR4_MSS_R5SS0_APB_DBGBCR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR4_MSS_R5SS0_APB_DBGBCR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR4_MSS_R5SS0_APB_DBGBCR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBCR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR5_MSS_R5SS0_APB_DBGBCR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR5_MSS_R5SS0_APB_DBGBCR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR5_MSS_R5SS0_APB_DBGBCR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR5_MSS_R5SS0_APB_DBGBCR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBCR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR6_MSS_R5SS0_APB_DBGBCR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR6_MSS_R5SS0_APB_DBGBCR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR6_MSS_R5SS0_APB_DBGBCR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR6_MSS_R5SS0_APB_DBGBCR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGBCR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR7_MSS_R5SS0_APB_DBGBCR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR7_MSS_R5SS0_APB_DBGBCR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR7_MSS_R5SS0_APB_DBGBCR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR7_MSS_R5SS0_APB_DBGBCR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGBCR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWVR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR0_MSS_R5SS0_APB_DBGWVR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR0_MSS_R5SS0_APB_DBGWVR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR0_MSS_R5SS0_APB_DBGWVR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR0_MSS_R5SS0_APB_DBGWVR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR1_MSS_R5SS0_APB_DBGWVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR1_MSS_R5SS0_APB_DBGWVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR1_MSS_R5SS0_APB_DBGWVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR1_MSS_R5SS0_APB_DBGWVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR2_MSS_R5SS0_APB_DBGWVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR2_MSS_R5SS0_APB_DBGWVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR2_MSS_R5SS0_APB_DBGWVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR2_MSS_R5SS0_APB_DBGWVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR3_MSS_R5SS0_APB_DBGWVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR3_MSS_R5SS0_APB_DBGWVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR3_MSS_R5SS0_APB_DBGWVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR3_MSS_R5SS0_APB_DBGWVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR4_MSS_R5SS0_APB_DBGWVR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR4_MSS_R5SS0_APB_DBGWVR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR4_MSS_R5SS0_APB_DBGWVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR4_MSS_R5SS0_APB_DBGWVR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR5_MSS_R5SS0_APB_DBGWVR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR5_MSS_R5SS0_APB_DBGWVR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR5_MSS_R5SS0_APB_DBGWVR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR5_MSS_R5SS0_APB_DBGWVR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR6_MSS_R5SS0_APB_DBGWVR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR6_MSS_R5SS0_APB_DBGWVR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR6_MSS_R5SS0_APB_DBGWVR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR6_MSS_R5SS0_APB_DBGWVR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR7_MSS_R5SS0_APB_DBGWVR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR7_MSS_R5SS0_APB_DBGWVR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR7_MSS_R5SS0_APB_DBGWVR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR7_MSS_R5SS0_APB_DBGWVR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWVR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWCR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR0_MSS_R5SS0_APB_DBGWCR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR0_MSS_R5SS0_APB_DBGWCR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR0_MSS_R5SS0_APB_DBGWCR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR0_MSS_R5SS0_APB_DBGWCR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR1_MSS_R5SS0_APB_DBGWCR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR1_MSS_R5SS0_APB_DBGWCR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR1_MSS_R5SS0_APB_DBGWCR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR1_MSS_R5SS0_APB_DBGWCR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR2_MSS_R5SS0_APB_DBGWCR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR2_MSS_R5SS0_APB_DBGWCR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR2_MSS_R5SS0_APB_DBGWCR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR2_MSS_R5SS0_APB_DBGWCR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR3_MSS_R5SS0_APB_DBGWCR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR3_MSS_R5SS0_APB_DBGWCR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR3_MSS_R5SS0_APB_DBGWCR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR3_MSS_R5SS0_APB_DBGWCR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWCR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR4_MSS_R5SS0_APB_DBGWCR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR4_MSS_R5SS0_APB_DBGWCR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR4_MSS_R5SS0_APB_DBGWCR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR4_MSS_R5SS0_APB_DBGWCR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWCR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR5_MSS_R5SS0_APB_DBGWCR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR5_MSS_R5SS0_APB_DBGWCR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR5_MSS_R5SS0_APB_DBGWCR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR5_MSS_R5SS0_APB_DBGWCR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWCR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR6_MSS_R5SS0_APB_DBGWCR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR6_MSS_R5SS0_APB_DBGWCR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR6_MSS_R5SS0_APB_DBGWCR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR6_MSS_R5SS0_APB_DBGWCR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGWCR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR7_MSS_R5SS0_APB_DBGWCR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR7_MSS_R5SS0_APB_DBGWCR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR7_MSS_R5SS0_APB_DBGWCR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR7_MSS_R5SS0_APB_DBGWCR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGWCR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGOSLAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLAR_MSS_R5SS0_APB_DBGOSLAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLAR_MSS_R5SS0_APB_DBGOSLAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLAR_MSS_R5SS0_APB_DBGOSLAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLAR_MSS_R5SS0_APB_DBGOSLAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLAR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_DBGOSLSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLSR_MSS_R5SS0_APB_DBGOSLSR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLSR_MSS_R5SS0_APB_DBGOSLSR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLSR_MSS_R5SS0_APB_DBGOSLSR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLSR_MSS_R5SS0_APB_DBGOSLSR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSLSR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_DBGOSSRR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSSRR_MSS_R5SS0_APB_DBGOSSRR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSSRR_MSS_R5SS0_APB_DBGOSSRR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSSRR_MSS_R5SS0_APB_DBGOSSRR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSSRR_MSS_R5SS0_APB_DBGOSSRR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGOSSRR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_DBGPRCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRCR_MSS_R5SS0_APB_DBGPRCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRCR_MSS_R5SS0_APB_DBGPRCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRCR_MSS_R5SS0_APB_DBGPRCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRCR_MSS_R5SS0_APB_DBGPRCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_DBGPRSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRSR_MSS_R5SS0_APB_DBGPRSR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRSR_MSS_R5SS0_APB_DBGPRSR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRSR_MSS_R5SS0_APB_DBGPRSR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRSR_MSS_R5SS0_APB_DBGPRSR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_DBGPRSR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_MIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MIDR_MSS_R5SS0_APB_PROCID_MIDR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MIDR_MSS_R5SS0_APB_PROCID_MIDR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MIDR_MSS_R5SS0_APB_PROCID_MIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MIDR_MSS_R5SS0_APB_PROCID_MIDR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MIDR_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_CTR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_CTR_MSS_R5SS0_APB_PROCID_CTR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_CTR_MSS_R5SS0_APB_PROCID_CTR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_CTR_MSS_R5SS0_APB_PROCID_CTR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_CTR_MSS_R5SS0_APB_PROCID_CTR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_CTR_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_TCMTR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_TCMTR_MSS_R5SS0_APB_PROCID_TCMTR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_TCMTR_MSS_R5SS0_APB_PROCID_TCMTR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_TCMTR_MSS_R5SS0_APB_PROCID_TCMTR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_TCMTR_MSS_R5SS0_APB_PROCID_TCMTR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_TCMTR_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_MPUIR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPUIR_MSS_R5SS0_APB_PROCID_MPUIR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPUIR_MSS_R5SS0_APB_PROCID_MPUIR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPUIR_MSS_R5SS0_APB_PROCID_MPUIR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPUIR_MSS_R5SS0_APB_PROCID_MPUIR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPUIR_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_MPIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPIDR_MSS_R5SS0_APB_PROCID_MPIDR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPIDR_MSS_R5SS0_APB_PROCID_MPIDR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPIDR_MSS_R5SS0_APB_PROCID_MPIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPIDR_MSS_R5SS0_APB_PROCID_MPIDR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MPIDR_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_PFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR0_MSS_R5SS0_APB_PROCID_PFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR0_MSS_R5SS0_APB_PROCID_PFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR0_MSS_R5SS0_APB_PROCID_PFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR0_MSS_R5SS0_APB_PROCID_PFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_PFR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR1_MSS_R5SS0_APB_PROCID_PFR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR1_MSS_R5SS0_APB_PROCID_PFR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR1_MSS_R5SS0_APB_PROCID_PFR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR1_MSS_R5SS0_APB_PROCID_PFR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_PFR1_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_DFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_DFR0_MSS_R5SS0_APB_PROCID_DFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_DFR0_MSS_R5SS0_APB_PROCID_DFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_DFR0_MSS_R5SS0_APB_PROCID_DFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_DFR0_MSS_R5SS0_APB_PROCID_DFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_DFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_AFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_AFR0_MSS_R5SS0_APB_PROCID_AFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_AFR0_MSS_R5SS0_APB_PROCID_AFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_AFR0_MSS_R5SS0_APB_PROCID_AFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_AFR0_MSS_R5SS0_APB_PROCID_AFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_AFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_MMFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR0_MSS_R5SS0_APB_PROCID_MMFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR0_MSS_R5SS0_APB_PROCID_MMFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR0_MSS_R5SS0_APB_PROCID_MMFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR0_MSS_R5SS0_APB_PROCID_MMFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR0_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_MMFR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR1_MSS_R5SS0_APB_PROCID_MMFR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR1_MSS_R5SS0_APB_PROCID_MMFR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR1_MSS_R5SS0_APB_PROCID_MMFR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR1_MSS_R5SS0_APB_PROCID_MMFR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR1_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_MMFR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR2_MSS_R5SS0_APB_PROCID_MMFR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR2_MSS_R5SS0_APB_PROCID_MMFR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR2_MSS_R5SS0_APB_PROCID_MMFR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR2_MSS_R5SS0_APB_PROCID_MMFR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR2_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_MMFR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR3_MSS_R5SS0_APB_PROCID_MMFR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR3_MSS_R5SS0_APB_PROCID_MMFR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR3_MSS_R5SS0_APB_PROCID_MMFR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR3_MSS_R5SS0_APB_PROCID_MMFR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_MMFR3_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_ISAR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR0_MSS_R5SS0_APB_PROCID_ISAR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR0_MSS_R5SS0_APB_PROCID_ISAR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR0_MSS_R5SS0_APB_PROCID_ISAR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR0_MSS_R5SS0_APB_PROCID_ISAR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR0_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_ISAR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR1_MSS_R5SS0_APB_PROCID_ISAR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR1_MSS_R5SS0_APB_PROCID_ISAR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR1_MSS_R5SS0_APB_PROCID_ISAR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR1_MSS_R5SS0_APB_PROCID_ISAR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR1_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_ISAR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR2_MSS_R5SS0_APB_PROCID_ISAR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR2_MSS_R5SS0_APB_PROCID_ISAR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR2_MSS_R5SS0_APB_PROCID_ISAR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR2_MSS_R5SS0_APB_PROCID_ISAR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR2_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_ISAR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR3_MSS_R5SS0_APB_PROCID_ISAR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR3_MSS_R5SS0_APB_PROCID_ISAR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR3_MSS_R5SS0_APB_PROCID_ISAR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR3_MSS_R5SS0_APB_PROCID_ISAR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR3_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_ISAR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR4_MSS_R5SS0_APB_PROCID_ISAR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR4_MSS_R5SS0_APB_PROCID_ISAR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR4_MSS_R5SS0_APB_PROCID_ISAR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR4_MSS_R5SS0_APB_PROCID_ISAR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR4_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_PROCID_ISAR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR5_MSS_R5SS0_APB_PROCID_ISAR5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR5_MSS_R5SS0_APB_PROCID_ISAR5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR5_MSS_R5SS0_APB_PROCID_ISAR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR5_MSS_R5SS0_APB_PROCID_ISAR5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PROCID_ISAR5_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_APB_MR_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_ITCTRL_MSS_R5SS0_APB_MR_ITCTRL_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_ITCTRL_MSS_R5SS0_APB_MR_ITCTRL_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_ITCTRL_MSS_R5SS0_APB_MR_ITCTRL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_ITCTRL_MSS_R5SS0_APB_MR_ITCTRL_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_ITCTRL_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_APB_MR_CLAIMSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMSET_MSS_R5SS0_APB_MR_CLAIMSET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMSET_MSS_R5SS0_APB_MR_CLAIMSET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMSET_MSS_R5SS0_APB_MR_CLAIMSET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMSET_MSS_R5SS0_APB_MR_CLAIMSET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMSET_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_APB_MR_CLAIMCLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMCLR_MSS_R5SS0_APB_MR_CLAIMCLR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMCLR_MSS_R5SS0_APB_MR_CLAIMCLR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMCLR_MSS_R5SS0_APB_MR_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMCLR_MSS_R5SS0_APB_MR_CLAIMCLR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_CLAIMCLR_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_APB_MR_LOCKACCESS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKACCESS_MSS_R5SS0_APB_MR_LOCKACCESS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKACCESS_MSS_R5SS0_APB_MR_LOCKACCESS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKACCESS_MSS_R5SS0_APB_MR_LOCKACCESS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKACCESS_MSS_R5SS0_APB_MR_LOCKACCESS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKACCESS_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_APB_MR_LOCKSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKSTATUS_MSS_R5SS0_APB_MR_LOCKSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKSTATUS_MSS_R5SS0_APB_MR_LOCKSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKSTATUS_MSS_R5SS0_APB_MR_LOCKSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKSTATUS_MSS_R5SS0_APB_MR_LOCKSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_LOCKSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_APB_MR_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_AUTHSTATUS_MSS_R5SS0_APB_MR_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_AUTHSTATUS_MSS_R5SS0_APB_MR_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_AUTHSTATUS_MSS_R5SS0_APB_MR_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_AUTHSTATUS_MSS_R5SS0_APB_MR_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_AUTHSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_APB_MR_DEVID */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVID_MSS_R5SS0_APB_MR_DEVID_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVID_MSS_R5SS0_APB_MR_DEVID_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVID_MSS_R5SS0_APB_MR_DEVID_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVID_MSS_R5SS0_APB_MR_DEVID_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVID_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_MR_DEVTYPE */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVTYPE_MSS_R5SS0_APB_MR_DEVTYPE_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVTYPE_MSS_R5SS0_APB_MR_DEVTYPE_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVTYPE_MSS_R5SS0_APB_MR_DEVTYPE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVTYPE_MSS_R5SS0_APB_MR_DEVTYPE_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_MR_DEVTYPE_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_APB_PERIP_ID4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID4_MSS_R5SS0_APB_PERIP_ID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID4_MSS_R5SS0_APB_PERIP_ID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID4_MSS_R5SS0_APB_PERIP_ID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID4_MSS_R5SS0_APB_PERIP_ID4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID4_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_APB_PERIP_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID0_MSS_R5SS0_APB_PERIP_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID0_MSS_R5SS0_APB_PERIP_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID0_MSS_R5SS0_APB_PERIP_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID0_MSS_R5SS0_APB_PERIP_ID0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID0_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_APB_PERIP_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID1_MSS_R5SS0_APB_PERIP_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID1_MSS_R5SS0_APB_PERIP_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID1_MSS_R5SS0_APB_PERIP_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID1_MSS_R5SS0_APB_PERIP_ID1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID1_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_APB_PERIP_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID2_MSS_R5SS0_APB_PERIP_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID2_MSS_R5SS0_APB_PERIP_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID2_MSS_R5SS0_APB_PERIP_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID2_MSS_R5SS0_APB_PERIP_ID2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID2_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_APB_PERIP_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID3_MSS_R5SS0_APB_PERIP_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID3_MSS_R5SS0_APB_PERIP_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID3_MSS_R5SS0_APB_PERIP_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID3_MSS_R5SS0_APB_PERIP_ID3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_PERIP_ID3_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_APB_COMP_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID0_MSS_R5SS0_APB_COMP_ID0_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID0_MSS_R5SS0_APB_COMP_ID0_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID0_MSS_R5SS0_APB_COMP_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID0_MSS_R5SS0_APB_COMP_ID0_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID0_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_COMP_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID1_MSS_R5SS0_APB_COMP_ID1_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID1_MSS_R5SS0_APB_COMP_ID1_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID1_MSS_R5SS0_APB_COMP_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID1_MSS_R5SS0_APB_COMP_ID1_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID1_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_COMP_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID2_MSS_R5SS0_APB_COMP_ID2_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID2_MSS_R5SS0_APB_COMP_ID2_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID2_MSS_R5SS0_APB_COMP_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID2_MSS_R5SS0_APB_COMP_ID2_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID2_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_APB_COMP_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID3_MSS_R5SS0_APB_COMP_ID3_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID3_MSS_R5SS0_APB_COMP_ID3_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID3_MSS_R5SS0_APB_COMP_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID3_MSS_R5SS0_APB_COMP_ID3_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_APB_COMP_ID3_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_CTI_CONTROL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CONTROL_MSS_R5SS0_CTI_CONTROL_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CONTROL_MSS_R5SS0_CTI_CONTROL_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CONTROL_MSS_R5SS0_CTI_CONTROL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CONTROL_MSS_R5SS0_CTI_CONTROL_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CONTROL_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_CTI_INTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INTACK_MSS_R5SS0_CTI_INTACK_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INTACK_MSS_R5SS0_CTI_INTACK_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INTACK_MSS_R5SS0_CTI_INTACK_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INTACK_MSS_R5SS0_CTI_INTACK_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INTACK_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_APPSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPSET_MSS_R5SS0_CTI_APPSET_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPSET_MSS_R5SS0_CTI_APPSET_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPSET_MSS_R5SS0_CTI_APPSET_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPSET_MSS_R5SS0_CTI_APPSET_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPSET_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_APPCLEAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPCLEAR_MSS_R5SS0_CTI_APPCLEAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPCLEAR_MSS_R5SS0_CTI_APPCLEAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPCLEAR_MSS_R5SS0_CTI_APPCLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPCLEAR_MSS_R5SS0_CTI_APPCLEAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPCLEAR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_CTI_APPPULSE */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPPULSE_MSS_R5SS0_CTI_APPPULSE_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPPULSE_MSS_R5SS0_CTI_APPPULSE_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPPULSE_MSS_R5SS0_CTI_APPPULSE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPPULSE_MSS_R5SS0_CTI_APPPULSE_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_APPPULSE_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_CTI_INEN0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN0_MSS_R5SS0_CTI_INEN0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN0_MSS_R5SS0_CTI_INEN0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN0_MSS_R5SS0_CTI_INEN0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN0_MSS_R5SS0_CTI_INEN0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN0_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_CTI_INEN1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN1_MSS_R5SS0_CTI_INEN1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN1_MSS_R5SS0_CTI_INEN1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN1_MSS_R5SS0_CTI_INEN1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN1_MSS_R5SS0_CTI_INEN1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_CTI_INEN2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN2_MSS_R5SS0_CTI_INEN2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN2_MSS_R5SS0_CTI_INEN2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN2_MSS_R5SS0_CTI_INEN2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN2_MSS_R5SS0_CTI_INEN2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_CTI_INEN3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN3_MSS_R5SS0_CTI_INEN3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN3_MSS_R5SS0_CTI_INEN3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN3_MSS_R5SS0_CTI_INEN3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN3_MSS_R5SS0_CTI_INEN3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_CTI_INEN4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN4_MSS_R5SS0_CTI_INEN4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN4_MSS_R5SS0_CTI_INEN4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN4_MSS_R5SS0_CTI_INEN4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN4_MSS_R5SS0_CTI_INEN4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_CTI_INEN5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN5_MSS_R5SS0_CTI_INEN5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN5_MSS_R5SS0_CTI_INEN5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN5_MSS_R5SS0_CTI_INEN5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN5_MSS_R5SS0_CTI_INEN5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_CTI_INEN6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN6_MSS_R5SS0_CTI_INEN6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN6_MSS_R5SS0_CTI_INEN6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN6_MSS_R5SS0_CTI_INEN6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN6_MSS_R5SS0_CTI_INEN6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_CTI_INEN7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN7_MSS_R5SS0_CTI_INEN7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN7_MSS_R5SS0_CTI_INEN7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN7_MSS_R5SS0_CTI_INEN7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN7_MSS_R5SS0_CTI_INEN7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_INEN7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_CTI_OUTEN0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN0_MSS_R5SS0_CTI_OUTEN0_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN0_MSS_R5SS0_CTI_OUTEN0_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN0_MSS_R5SS0_CTI_OUTEN0_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN0_MSS_R5SS0_CTI_OUTEN0_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN0_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_OUTEN1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN1_MSS_R5SS0_CTI_OUTEN1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN1_MSS_R5SS0_CTI_OUTEN1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN1_MSS_R5SS0_CTI_OUTEN1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN1_MSS_R5SS0_CTI_OUTEN1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN1_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_OUTEN2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN2_MSS_R5SS0_CTI_OUTEN2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN2_MSS_R5SS0_CTI_OUTEN2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN2_MSS_R5SS0_CTI_OUTEN2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN2_MSS_R5SS0_CTI_OUTEN2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN2_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_OUTEN3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN3_MSS_R5SS0_CTI_OUTEN3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN3_MSS_R5SS0_CTI_OUTEN3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN3_MSS_R5SS0_CTI_OUTEN3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN3_MSS_R5SS0_CTI_OUTEN3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN3_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_OUTEN4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN4_MSS_R5SS0_CTI_OUTEN4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN4_MSS_R5SS0_CTI_OUTEN4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN4_MSS_R5SS0_CTI_OUTEN4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN4_MSS_R5SS0_CTI_OUTEN4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN4_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_OUTEN5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN5_MSS_R5SS0_CTI_OUTEN5_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN5_MSS_R5SS0_CTI_OUTEN5_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN5_MSS_R5SS0_CTI_OUTEN5_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN5_MSS_R5SS0_CTI_OUTEN5_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN5_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_OUTEN6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN6_MSS_R5SS0_CTI_OUTEN6_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN6_MSS_R5SS0_CTI_OUTEN6_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN6_MSS_R5SS0_CTI_OUTEN6_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN6_MSS_R5SS0_CTI_OUTEN6_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN6_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_OUTEN7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN7_MSS_R5SS0_CTI_OUTEN7_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN7_MSS_R5SS0_CTI_OUTEN7_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN7_MSS_R5SS0_CTI_OUTEN7_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN7_MSS_R5SS0_CTI_OUTEN7_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_OUTEN7_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_TRIGINSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGINSTATUS_MSS_R5SS0_CTI_TRIGINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGINSTATUS_MSS_R5SS0_CTI_TRIGINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGINSTATUS_MSS_R5SS0_CTI_TRIGINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGINSTATUS_MSS_R5SS0_CTI_TRIGINSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGINSTATUS_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_CTI_TRIGOUTSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGOUTSTATUS_MSS_R5SS0_CTI_TRIGOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGOUTSTATUS_MSS_R5SS0_CTI_TRIGOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGOUTSTATUS_MSS_R5SS0_CTI_TRIGOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGOUTSTATUS_MSS_R5SS0_CTI_TRIGOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_TRIGOUTSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_CHINSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHINSTATUS_MSS_R5SS0_CTI_CHINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHINSTATUS_MSS_R5SS0_CTI_CHINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHINSTATUS_MSS_R5SS0_CTI_CHINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHINSTATUS_MSS_R5SS0_CTI_CHINSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHINSTATUS_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_CTI_CHOUTSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHOUTSTATUS_MSS_R5SS0_CTI_CHOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHOUTSTATUS_MSS_R5SS0_CTI_CHOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHOUTSTATUS_MSS_R5SS0_CTI_CHOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHOUTSTATUS_MSS_R5SS0_CTI_CHOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CHOUTSTATUS_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_CTI_GATE */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_GATE_MSS_R5SS0_CTI_GATE_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_GATE_MSS_R5SS0_CTI_GATE_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_GATE_MSS_R5SS0_CTI_GATE_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_GATE_MSS_R5SS0_CTI_GATE_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_GATE_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_A_CTI_ASICCTL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ASICCTL_MSS_R5SS0_CTI_ASICCTL_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ASICCTL_MSS_R5SS0_CTI_ASICCTL_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ASICCTL_MSS_R5SS0_CTI_ASICCTL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ASICCTL_MSS_R5SS0_CTI_ASICCTL_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ASICCTL_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_CTI_ITCHINACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHINACK_MSS_R5SS0_CTI_ITCHINACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHINACK_MSS_R5SS0_CTI_ITCHINACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHINACK_MSS_R5SS0_CTI_ITCHINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHINACK_MSS_R5SS0_CTI_ITCHINACK_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHINACK_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_CTI_ITTRIGINACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGINACK_MSS_R5SS0_CTI_ITTRIGINACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGINACK_MSS_R5SS0_CTI_ITTRIGINACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGINACK_MSS_R5SS0_CTI_ITTRIGINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGINACK_MSS_R5SS0_CTI_ITTRIGINACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGINACK_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_A_CTI_ITCHOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUT_MSS_R5SS0_CTI_ITCHOUT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUT_MSS_R5SS0_CTI_ITCHOUT_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUT_MSS_R5SS0_CTI_ITCHOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUT_MSS_R5SS0_CTI_ITCHOUT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUT_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_CTI_ITTRIGOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUT_MSS_R5SS0_CTI_ITTRIGOUT_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUT_MSS_R5SS0_CTI_ITTRIGOUT_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUT_MSS_R5SS0_CTI_ITTRIGOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUT_MSS_R5SS0_CTI_ITTRIGOUT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUT_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_CTI_ITCHOUTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUTACK_MSS_R5SS0_CTI_ITCHOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUTACK_MSS_R5SS0_CTI_ITCHOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUTACK_MSS_R5SS0_CTI_ITCHOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUTACK_MSS_R5SS0_CTI_ITCHOUTACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHOUTACK_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_CTI_ITTRIGOUTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUTACK_MSS_R5SS0_CTI_ITTRIGOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUTACK_MSS_R5SS0_CTI_ITTRIGOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUTACK_MSS_R5SS0_CTI_ITTRIGOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUTACK_MSS_R5SS0_CTI_ITTRIGOUTACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGOUTACK_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_CTI_ITCHIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHIN_MSS_R5SS0_CTI_ITCHIN_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHIN_MSS_R5SS0_CTI_ITCHIN_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHIN_MSS_R5SS0_CTI_ITCHIN_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHIN_MSS_R5SS0_CTI_ITCHIN_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCHIN_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_ITTRIGIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGIN_MSS_R5SS0_CTI_ITTRIGIN_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGIN_MSS_R5SS0_CTI_ITTRIGIN_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGIN_MSS_R5SS0_CTI_ITTRIGIN_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGIN_MSS_R5SS0_CTI_ITTRIGIN_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITTRIGIN_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_CTI_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCTRL_MSS_R5SS0_CTI_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCTRL_MSS_R5SS0_CTI_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCTRL_MSS_R5SS0_CTI_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCTRL_MSS_R5SS0_CTI_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_ITCTRL_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_CTI_CLAIM_TAG_SET */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_SET_MSS_R5SS0_CTI_CLAIM_TAG_SET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_SET_MSS_R5SS0_CTI_CLAIM_TAG_SET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_SET_MSS_R5SS0_CTI_CLAIM_TAG_SET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_SET_MSS_R5SS0_CTI_CLAIM_TAG_SET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_SET_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_CLAIM_TAG_CLEAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_CLEAR_MSS_R5SS0_CTI_CLAIM_TAG_CLEAR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_CLEAR_MSS_R5SS0_CTI_CLAIM_TAG_CLEAR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_CLEAR_MSS_R5SS0_CTI_CLAIM_TAG_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_CLEAR_MSS_R5SS0_CTI_CLAIM_TAG_CLEAR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_CLAIM_TAG_CLEAR_RESETVAL               (0x00000000U)

/* MSS_R5SS0_A_CTI_LOCK_ACCESS_REGISTER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS0_CTI_LOCK_ACCESS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS0_CTI_LOCK_ACCESS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS0_CTI_LOCK_ACCESS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS0_CTI_LOCK_ACCESS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_ACCESS_REGISTER_RESETVAL          (0x00000000U)

/* MSS_R5SS0_A_CTI_LOCK_STATUS_REGISTER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_STATUS_REGISTER_MSS_R5SS0_CTI_LOCK_STATUS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_STATUS_REGISTER_MSS_R5SS0_CTI_LOCK_STATUS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_STATUS_REGISTER_MSS_R5SS0_CTI_LOCK_STATUS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_STATUS_REGISTER_MSS_R5SS0_CTI_LOCK_STATUS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_LOCK_STATUS_REGISTER_RESETVAL          (0x00000000U)

/* MSS_R5SS0_A_CTI_AUTHENTICATION_STATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_AUTHENTICATION_STATUS_MSS_R5SS0_CTI_AUTHENTICATION_STATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_AUTHENTICATION_STATUS_MSS_R5SS0_CTI_AUTHENTICATION_STATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_AUTHENTICATION_STATUS_MSS_R5SS0_CTI_AUTHENTICATION_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_AUTHENTICATION_STATUS_MSS_R5SS0_CTI_AUTHENTICATION_STATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_AUTHENTICATION_STATUS_RESETVAL         (0x00000000U)

/* MSS_R5SS0_A_CTI_DEVICE_ID */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_ID_MSS_R5SS0_CTI_DEVICE_ID_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_ID_MSS_R5SS0_CTI_DEVICE_ID_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_ID_MSS_R5SS0_CTI_DEVICE_ID_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_ID_MSS_R5SS0_CTI_DEVICE_ID_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_ID_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_CTI_DEVICE_TYPE_IDENTIFIER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS0_CTI_DEVICE_TYPE_IDENTIFIER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS0_CTI_DEVICE_TYPE_IDENTIFIER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS0_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS0_CTI_DEVICE_TYPE_IDENTIFIER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL        (0x00000000U)

/* MSS_R5SS0_A_CTI_PERIPHERALID4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID4_MSS_R5SS0_CTI_PERIPHERALID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID4_MSS_R5SS0_CTI_PERIPHERALID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID4_MSS_R5SS0_CTI_PERIPHERALID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID4_MSS_R5SS0_CTI_PERIPHERALID4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID4_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_PERIPHERALID5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID5_MSS_R5SS0_CTI_PERIPHERALID5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID5_MSS_R5SS0_CTI_PERIPHERALID5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID5_MSS_R5SS0_CTI_PERIPHERALID5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID5_MSS_R5SS0_CTI_PERIPHERALID5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID5_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_PERIPHERALID6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID6_MSS_R5SS0_CTI_PERIPHERALID6_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID6_MSS_R5SS0_CTI_PERIPHERALID6_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID6_MSS_R5SS0_CTI_PERIPHERALID6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID6_MSS_R5SS0_CTI_PERIPHERALID6_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID6_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_PERIPHERALID7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID7_MSS_R5SS0_CTI_PERIPHERALID7_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID7_MSS_R5SS0_CTI_PERIPHERALID7_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID7_MSS_R5SS0_CTI_PERIPHERALID7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID7_MSS_R5SS0_CTI_PERIPHERALID7_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID7_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_PERIPHERALID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID0_MSS_R5SS0_CTI_PERIPHERALID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID0_MSS_R5SS0_CTI_PERIPHERALID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID0_MSS_R5SS0_CTI_PERIPHERALID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID0_MSS_R5SS0_CTI_PERIPHERALID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID0_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_PERIPHERALID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID1_MSS_R5SS0_CTI_PERIPHERALID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID1_MSS_R5SS0_CTI_PERIPHERALID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID1_MSS_R5SS0_CTI_PERIPHERALID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID1_MSS_R5SS0_CTI_PERIPHERALID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID1_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_PERIPHERALID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID2_MSS_R5SS0_CTI_PERIPHERALID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID2_MSS_R5SS0_CTI_PERIPHERALID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID2_MSS_R5SS0_CTI_PERIPHERALID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID2_MSS_R5SS0_CTI_PERIPHERALID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID2_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_PERIPHERALID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID3_MSS_R5SS0_CTI_PERIPHERALID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID3_MSS_R5SS0_CTI_PERIPHERALID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID3_MSS_R5SS0_CTI_PERIPHERALID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID3_MSS_R5SS0_CTI_PERIPHERALID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_PERIPHERALID3_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_COMPONENT_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID0_MSS_R5SS0_CTI_COMPONENT_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID0_MSS_R5SS0_CTI_COMPONENT_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID0_MSS_R5SS0_CTI_COMPONENT_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID0_MSS_R5SS0_CTI_COMPONENT_ID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID0_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_COMPONENT_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID1_MSS_R5SS0_CTI_COMPONENT_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID1_MSS_R5SS0_CTI_COMPONENT_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID1_MSS_R5SS0_CTI_COMPONENT_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID1_MSS_R5SS0_CTI_COMPONENT_ID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID1_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_COMPONENT_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID2_MSS_R5SS0_CTI_COMPONENT_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID2_MSS_R5SS0_CTI_COMPONENT_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID2_MSS_R5SS0_CTI_COMPONENT_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID2_MSS_R5SS0_CTI_COMPONENT_ID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID2_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_CTI_COMPONENT_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID3_MSS_R5SS0_CTI_COMPONENT_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID3_MSS_R5SS0_CTI_COMPONENT_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID3_MSS_R5SS0_CTI_COMPONENT_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID3_MSS_R5SS0_CTI_COMPONENT_ID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_CTI_COMPONENT_ID3_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_CONTROL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CONTROL_MSS_R5SS0_CTI_CONTROL_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CONTROL_MSS_R5SS0_CTI_CONTROL_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CONTROL_MSS_R5SS0_CTI_CONTROL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CONTROL_MSS_R5SS0_CTI_CONTROL_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CONTROL_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_CTI_INTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INTACK_MSS_R5SS0_CTI_INTACK_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INTACK_MSS_R5SS0_CTI_INTACK_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INTACK_MSS_R5SS0_CTI_INTACK_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INTACK_MSS_R5SS0_CTI_INTACK_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INTACK_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_APPSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPSET_MSS_R5SS0_CTI_APPSET_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPSET_MSS_R5SS0_CTI_APPSET_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPSET_MSS_R5SS0_CTI_APPSET_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPSET_MSS_R5SS0_CTI_APPSET_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPSET_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_APPCLEAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPCLEAR_MSS_R5SS0_CTI_APPCLEAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPCLEAR_MSS_R5SS0_CTI_APPCLEAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPCLEAR_MSS_R5SS0_CTI_APPCLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPCLEAR_MSS_R5SS0_CTI_APPCLEAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPCLEAR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_CTI_APPPULSE */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPPULSE_MSS_R5SS0_CTI_APPPULSE_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPPULSE_MSS_R5SS0_CTI_APPPULSE_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPPULSE_MSS_R5SS0_CTI_APPPULSE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPPULSE_MSS_R5SS0_CTI_APPPULSE_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_APPPULSE_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_CTI_INEN0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN0_MSS_R5SS0_CTI_INEN0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN0_MSS_R5SS0_CTI_INEN0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN0_MSS_R5SS0_CTI_INEN0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN0_MSS_R5SS0_CTI_INEN0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN0_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_CTI_INEN1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN1_MSS_R5SS0_CTI_INEN1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN1_MSS_R5SS0_CTI_INEN1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN1_MSS_R5SS0_CTI_INEN1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN1_MSS_R5SS0_CTI_INEN1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_CTI_INEN2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN2_MSS_R5SS0_CTI_INEN2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN2_MSS_R5SS0_CTI_INEN2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN2_MSS_R5SS0_CTI_INEN2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN2_MSS_R5SS0_CTI_INEN2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_CTI_INEN3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN3_MSS_R5SS0_CTI_INEN3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN3_MSS_R5SS0_CTI_INEN3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN3_MSS_R5SS0_CTI_INEN3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN3_MSS_R5SS0_CTI_INEN3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_CTI_INEN4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN4_MSS_R5SS0_CTI_INEN4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN4_MSS_R5SS0_CTI_INEN4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN4_MSS_R5SS0_CTI_INEN4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN4_MSS_R5SS0_CTI_INEN4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_CTI_INEN5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN5_MSS_R5SS0_CTI_INEN5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN5_MSS_R5SS0_CTI_INEN5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN5_MSS_R5SS0_CTI_INEN5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN5_MSS_R5SS0_CTI_INEN5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_CTI_INEN6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN6_MSS_R5SS0_CTI_INEN6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN6_MSS_R5SS0_CTI_INEN6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN6_MSS_R5SS0_CTI_INEN6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN6_MSS_R5SS0_CTI_INEN6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_CTI_INEN7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN7_MSS_R5SS0_CTI_INEN7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN7_MSS_R5SS0_CTI_INEN7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN7_MSS_R5SS0_CTI_INEN7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN7_MSS_R5SS0_CTI_INEN7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_INEN7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_CTI_OUTEN0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN0_MSS_R5SS0_CTI_OUTEN0_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN0_MSS_R5SS0_CTI_OUTEN0_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN0_MSS_R5SS0_CTI_OUTEN0_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN0_MSS_R5SS0_CTI_OUTEN0_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN0_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_OUTEN1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN1_MSS_R5SS0_CTI_OUTEN1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN1_MSS_R5SS0_CTI_OUTEN1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN1_MSS_R5SS0_CTI_OUTEN1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN1_MSS_R5SS0_CTI_OUTEN1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN1_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_OUTEN2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN2_MSS_R5SS0_CTI_OUTEN2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN2_MSS_R5SS0_CTI_OUTEN2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN2_MSS_R5SS0_CTI_OUTEN2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN2_MSS_R5SS0_CTI_OUTEN2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN2_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_OUTEN3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN3_MSS_R5SS0_CTI_OUTEN3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN3_MSS_R5SS0_CTI_OUTEN3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN3_MSS_R5SS0_CTI_OUTEN3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN3_MSS_R5SS0_CTI_OUTEN3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN3_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_OUTEN4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN4_MSS_R5SS0_CTI_OUTEN4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN4_MSS_R5SS0_CTI_OUTEN4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN4_MSS_R5SS0_CTI_OUTEN4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN4_MSS_R5SS0_CTI_OUTEN4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN4_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_OUTEN5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN5_MSS_R5SS0_CTI_OUTEN5_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN5_MSS_R5SS0_CTI_OUTEN5_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN5_MSS_R5SS0_CTI_OUTEN5_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN5_MSS_R5SS0_CTI_OUTEN5_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN5_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_OUTEN6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN6_MSS_R5SS0_CTI_OUTEN6_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN6_MSS_R5SS0_CTI_OUTEN6_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN6_MSS_R5SS0_CTI_OUTEN6_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN6_MSS_R5SS0_CTI_OUTEN6_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN6_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_OUTEN7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN7_MSS_R5SS0_CTI_OUTEN7_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN7_MSS_R5SS0_CTI_OUTEN7_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN7_MSS_R5SS0_CTI_OUTEN7_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN7_MSS_R5SS0_CTI_OUTEN7_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_OUTEN7_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_TRIGINSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGINSTATUS_MSS_R5SS0_CTI_TRIGINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGINSTATUS_MSS_R5SS0_CTI_TRIGINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGINSTATUS_MSS_R5SS0_CTI_TRIGINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGINSTATUS_MSS_R5SS0_CTI_TRIGINSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGINSTATUS_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_CTI_TRIGOUTSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGOUTSTATUS_MSS_R5SS0_CTI_TRIGOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGOUTSTATUS_MSS_R5SS0_CTI_TRIGOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGOUTSTATUS_MSS_R5SS0_CTI_TRIGOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGOUTSTATUS_MSS_R5SS0_CTI_TRIGOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_TRIGOUTSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_CHINSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHINSTATUS_MSS_R5SS0_CTI_CHINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHINSTATUS_MSS_R5SS0_CTI_CHINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHINSTATUS_MSS_R5SS0_CTI_CHINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHINSTATUS_MSS_R5SS0_CTI_CHINSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHINSTATUS_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_CTI_CHOUTSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHOUTSTATUS_MSS_R5SS0_CTI_CHOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHOUTSTATUS_MSS_R5SS0_CTI_CHOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHOUTSTATUS_MSS_R5SS0_CTI_CHOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHOUTSTATUS_MSS_R5SS0_CTI_CHOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CHOUTSTATUS_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_CTI_GATE */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_GATE_MSS_R5SS0_CTI_GATE_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_GATE_MSS_R5SS0_CTI_GATE_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_GATE_MSS_R5SS0_CTI_GATE_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_GATE_MSS_R5SS0_CTI_GATE_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_GATE_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_B_CTI_ASICCTL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ASICCTL_MSS_R5SS0_CTI_ASICCTL_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ASICCTL_MSS_R5SS0_CTI_ASICCTL_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ASICCTL_MSS_R5SS0_CTI_ASICCTL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ASICCTL_MSS_R5SS0_CTI_ASICCTL_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ASICCTL_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_CTI_ITCHINACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHINACK_MSS_R5SS0_CTI_ITCHINACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHINACK_MSS_R5SS0_CTI_ITCHINACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHINACK_MSS_R5SS0_CTI_ITCHINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHINACK_MSS_R5SS0_CTI_ITCHINACK_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHINACK_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_CTI_ITTRIGINACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGINACK_MSS_R5SS0_CTI_ITTRIGINACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGINACK_MSS_R5SS0_CTI_ITTRIGINACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGINACK_MSS_R5SS0_CTI_ITTRIGINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGINACK_MSS_R5SS0_CTI_ITTRIGINACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGINACK_RESETVAL                   (0x00000000U)

/* MSS_R5SS0_B_CTI_ITCHOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUT_MSS_R5SS0_CTI_ITCHOUT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUT_MSS_R5SS0_CTI_ITCHOUT_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUT_MSS_R5SS0_CTI_ITCHOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUT_MSS_R5SS0_CTI_ITCHOUT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUT_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_CTI_ITTRIGOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUT_MSS_R5SS0_CTI_ITTRIGOUT_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUT_MSS_R5SS0_CTI_ITTRIGOUT_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUT_MSS_R5SS0_CTI_ITTRIGOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUT_MSS_R5SS0_CTI_ITTRIGOUT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUT_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_CTI_ITCHOUTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUTACK_MSS_R5SS0_CTI_ITCHOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUTACK_MSS_R5SS0_CTI_ITCHOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUTACK_MSS_R5SS0_CTI_ITCHOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUTACK_MSS_R5SS0_CTI_ITCHOUTACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHOUTACK_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_CTI_ITTRIGOUTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUTACK_MSS_R5SS0_CTI_ITTRIGOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUTACK_MSS_R5SS0_CTI_ITTRIGOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUTACK_MSS_R5SS0_CTI_ITTRIGOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUTACK_MSS_R5SS0_CTI_ITTRIGOUTACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGOUTACK_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_CTI_ITCHIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHIN_MSS_R5SS0_CTI_ITCHIN_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHIN_MSS_R5SS0_CTI_ITCHIN_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHIN_MSS_R5SS0_CTI_ITCHIN_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHIN_MSS_R5SS0_CTI_ITCHIN_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCHIN_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_ITTRIGIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGIN_MSS_R5SS0_CTI_ITTRIGIN_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGIN_MSS_R5SS0_CTI_ITTRIGIN_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGIN_MSS_R5SS0_CTI_ITTRIGIN_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGIN_MSS_R5SS0_CTI_ITTRIGIN_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITTRIGIN_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_CTI_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCTRL_MSS_R5SS0_CTI_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCTRL_MSS_R5SS0_CTI_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCTRL_MSS_R5SS0_CTI_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCTRL_MSS_R5SS0_CTI_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_ITCTRL_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_CTI_CLAIM_TAG_SET */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_SET_MSS_R5SS0_CTI_CLAIM_TAG_SET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_SET_MSS_R5SS0_CTI_CLAIM_TAG_SET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_SET_MSS_R5SS0_CTI_CLAIM_TAG_SET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_SET_MSS_R5SS0_CTI_CLAIM_TAG_SET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_SET_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_CLAIM_TAG_CLEAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_CLEAR_MSS_R5SS0_CTI_CLAIM_TAG_CLEAR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_CLEAR_MSS_R5SS0_CTI_CLAIM_TAG_CLEAR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_CLEAR_MSS_R5SS0_CTI_CLAIM_TAG_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_CLEAR_MSS_R5SS0_CTI_CLAIM_TAG_CLEAR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_CLAIM_TAG_CLEAR_RESETVAL               (0x00000000U)

/* MSS_R5SS0_B_CTI_LOCK_ACCESS_REGISTER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS0_CTI_LOCK_ACCESS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS0_CTI_LOCK_ACCESS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS0_CTI_LOCK_ACCESS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS0_CTI_LOCK_ACCESS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_ACCESS_REGISTER_RESETVAL          (0x00000000U)

/* MSS_R5SS0_B_CTI_LOCK_STATUS_REGISTER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_STATUS_REGISTER_MSS_R5SS0_CTI_LOCK_STATUS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_STATUS_REGISTER_MSS_R5SS0_CTI_LOCK_STATUS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_STATUS_REGISTER_MSS_R5SS0_CTI_LOCK_STATUS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_STATUS_REGISTER_MSS_R5SS0_CTI_LOCK_STATUS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_LOCK_STATUS_REGISTER_RESETVAL          (0x00000000U)

/* MSS_R5SS0_B_CTI_AUTHENTICATION_STATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_AUTHENTICATION_STATUS_MSS_R5SS0_CTI_AUTHENTICATION_STATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_AUTHENTICATION_STATUS_MSS_R5SS0_CTI_AUTHENTICATION_STATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_AUTHENTICATION_STATUS_MSS_R5SS0_CTI_AUTHENTICATION_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_AUTHENTICATION_STATUS_MSS_R5SS0_CTI_AUTHENTICATION_STATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_AUTHENTICATION_STATUS_RESETVAL         (0x00000000U)

/* MSS_R5SS0_B_CTI_DEVICE_ID */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_ID_MSS_R5SS0_CTI_DEVICE_ID_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_ID_MSS_R5SS0_CTI_DEVICE_ID_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_ID_MSS_R5SS0_CTI_DEVICE_ID_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_ID_MSS_R5SS0_CTI_DEVICE_ID_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_ID_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_CTI_DEVICE_TYPE_IDENTIFIER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS0_CTI_DEVICE_TYPE_IDENTIFIER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS0_CTI_DEVICE_TYPE_IDENTIFIER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS0_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS0_CTI_DEVICE_TYPE_IDENTIFIER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL        (0x00000000U)

/* MSS_R5SS0_B_CTI_PERIPHERALID4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID4_MSS_R5SS0_CTI_PERIPHERALID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID4_MSS_R5SS0_CTI_PERIPHERALID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID4_MSS_R5SS0_CTI_PERIPHERALID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID4_MSS_R5SS0_CTI_PERIPHERALID4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID4_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_PERIPHERALID5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID5_MSS_R5SS0_CTI_PERIPHERALID5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID5_MSS_R5SS0_CTI_PERIPHERALID5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID5_MSS_R5SS0_CTI_PERIPHERALID5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID5_MSS_R5SS0_CTI_PERIPHERALID5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID5_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_PERIPHERALID6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID6_MSS_R5SS0_CTI_PERIPHERALID6_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID6_MSS_R5SS0_CTI_PERIPHERALID6_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID6_MSS_R5SS0_CTI_PERIPHERALID6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID6_MSS_R5SS0_CTI_PERIPHERALID6_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID6_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_PERIPHERALID7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID7_MSS_R5SS0_CTI_PERIPHERALID7_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID7_MSS_R5SS0_CTI_PERIPHERALID7_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID7_MSS_R5SS0_CTI_PERIPHERALID7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID7_MSS_R5SS0_CTI_PERIPHERALID7_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID7_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_PERIPHERALID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID0_MSS_R5SS0_CTI_PERIPHERALID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID0_MSS_R5SS0_CTI_PERIPHERALID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID0_MSS_R5SS0_CTI_PERIPHERALID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID0_MSS_R5SS0_CTI_PERIPHERALID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID0_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_PERIPHERALID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID1_MSS_R5SS0_CTI_PERIPHERALID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID1_MSS_R5SS0_CTI_PERIPHERALID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID1_MSS_R5SS0_CTI_PERIPHERALID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID1_MSS_R5SS0_CTI_PERIPHERALID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID1_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_PERIPHERALID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID2_MSS_R5SS0_CTI_PERIPHERALID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID2_MSS_R5SS0_CTI_PERIPHERALID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID2_MSS_R5SS0_CTI_PERIPHERALID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID2_MSS_R5SS0_CTI_PERIPHERALID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID2_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_PERIPHERALID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID3_MSS_R5SS0_CTI_PERIPHERALID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID3_MSS_R5SS0_CTI_PERIPHERALID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID3_MSS_R5SS0_CTI_PERIPHERALID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID3_MSS_R5SS0_CTI_PERIPHERALID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_PERIPHERALID3_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_COMPONENT_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID0_MSS_R5SS0_CTI_COMPONENT_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID0_MSS_R5SS0_CTI_COMPONENT_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID0_MSS_R5SS0_CTI_COMPONENT_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID0_MSS_R5SS0_CTI_COMPONENT_ID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID0_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_COMPONENT_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID1_MSS_R5SS0_CTI_COMPONENT_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID1_MSS_R5SS0_CTI_COMPONENT_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID1_MSS_R5SS0_CTI_COMPONENT_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID1_MSS_R5SS0_CTI_COMPONENT_ID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID1_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_COMPONENT_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID2_MSS_R5SS0_CTI_COMPONENT_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID2_MSS_R5SS0_CTI_COMPONENT_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID2_MSS_R5SS0_CTI_COMPONENT_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID2_MSS_R5SS0_CTI_COMPONENT_ID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID2_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_B_CTI_COMPONENT_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID3_MSS_R5SS0_CTI_COMPONENT_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID3_MSS_R5SS0_CTI_COMPONENT_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID3_MSS_R5SS0_CTI_COMPONENT_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID3_MSS_R5SS0_CTI_COMPONENT_ID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_CTI_COMPONENT_ID3_RESETVAL                 (0x00000000U)

/* MSS_R5SS0_A_ETM_CR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CR_MSS_R5SS0_ETM_CR_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CR_MSS_R5SS0_ETM_CR_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CR_MSS_R5SS0_ETM_CR_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CR_MSS_R5SS0_ETM_CR_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CR_RESETVAL                            (0x00000000U)

/* MSS_R5SS0_A_ETM_CCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCR_MSS_R5SS0_ETM_CCR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCR_MSS_R5SS0_ETM_CCR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCR_MSS_R5SS0_ETM_CCR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCR_MSS_R5SS0_ETM_CCR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_A_ETM_TRIGGER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRIGGER_MSS_R5SS0_ETM_TRIGGER_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRIGGER_MSS_R5SS0_ETM_TRIGGER_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRIGGER_MSS_R5SS0_ETM_TRIGGER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRIGGER_MSS_R5SS0_ETM_TRIGGER_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRIGGER_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_ASICCTLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ASICCTLR_MSS_R5SS0_ETM_ASICCTLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ASICCTLR_MSS_R5SS0_ETM_ASICCTLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ASICCTLR_MSS_R5SS0_ETM_ASICCTLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ASICCTLR_MSS_R5SS0_ETM_ASICCTLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ASICCTLR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_ETM_SR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SR_MSS_R5SS0_ETM_SR_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SR_MSS_R5SS0_ETM_SR_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SR_MSS_R5SS0_ETM_SR_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SR_MSS_R5SS0_ETM_SR_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SR_RESETVAL                            (0x00000000U)

/* MSS_R5SS0_A_ETM_SCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SCR_MSS_R5SS0_ETM_SCR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SCR_MSS_R5SS0_ETM_SCR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SCR_MSS_R5SS0_ETM_SCR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SCR_MSS_R5SS0_ETM_SCR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SCR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_A_ETM_TSSCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TSSCR_MSS_R5SS0_ETM_TSSCR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TSSCR_MSS_R5SS0_ETM_TSSCR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TSSCR_MSS_R5SS0_ETM_TSSCR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TSSCR_MSS_R5SS0_ETM_TSSCR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TSSCR_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_TECR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR2_MSS_R5SS0_ETM_TECR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR2_MSS_R5SS0_ETM_TECR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR2_MSS_R5SS0_ETM_TECR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR2_MSS_R5SS0_ETM_TECR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_TEEVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TEEVR_MSS_R5SS0_ETM_TEEVR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TEEVR_MSS_R5SS0_ETM_TEEVR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TEEVR_MSS_R5SS0_ETM_TEEVR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TEEVR_MSS_R5SS0_ETM_TEEVR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TEEVR_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_TECR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR1_MSS_R5SS0_ETM_TECR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR1_MSS_R5SS0_ETM_TECR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR1_MSS_R5SS0_ETM_TECR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR1_MSS_R5SS0_ETM_TECR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TECR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_FFRR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFRR_MSS_R5SS0_ETM_FFRR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFRR_MSS_R5SS0_ETM_FFRR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFRR_MSS_R5SS0_ETM_FFRR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFRR_MSS_R5SS0_ETM_FFRR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFRR_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_A_ETM_FFLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFLR_MSS_R5SS0_ETM_FFLR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFLR_MSS_R5SS0_ETM_FFLR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFLR_MSS_R5SS0_ETM_FFLR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFLR_MSS_R5SS0_ETM_FFLR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_FFLR_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_A_ETM_VDEVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDEVR_MSS_R5SS0_ETM_VDEVR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDEVR_MSS_R5SS0_ETM_VDEVR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDEVR_MSS_R5SS0_ETM_VDEVR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDEVR_MSS_R5SS0_ETM_VDEVR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDEVR_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_VDCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR1_MSS_R5SS0_ETM_VDCR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR1_MSS_R5SS0_ETM_VDCR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR1_MSS_R5SS0_ETM_VDCR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR1_MSS_R5SS0_ETM_VDCR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_VDCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR2_MSS_R5SS0_ETM_VDCR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR2_MSS_R5SS0_ETM_VDCR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR2_MSS_R5SS0_ETM_VDCR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR2_MSS_R5SS0_ETM_VDCR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_VDCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR3_MSS_R5SS0_ETM_VDCR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR3_MSS_R5SS0_ETM_VDCR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR3_MSS_R5SS0_ETM_VDCR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR3_MSS_R5SS0_ETM_VDCR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_VDCR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR1_MSS_R5SS0_ETM_ACVR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR1_MSS_R5SS0_ETM_ACVR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR1_MSS_R5SS0_ETM_ACVR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR1_MSS_R5SS0_ETM_ACVR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR2_MSS_R5SS0_ETM_ACVR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR2_MSS_R5SS0_ETM_ACVR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR2_MSS_R5SS0_ETM_ACVR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR2_MSS_R5SS0_ETM_ACVR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR3_MSS_R5SS0_ETM_ACVR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR3_MSS_R5SS0_ETM_ACVR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR3_MSS_R5SS0_ETM_ACVR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR3_MSS_R5SS0_ETM_ACVR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR4_MSS_R5SS0_ETM_ACVR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR4_MSS_R5SS0_ETM_ACVR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR4_MSS_R5SS0_ETM_ACVR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR4_MSS_R5SS0_ETM_ACVR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR5_MSS_R5SS0_ETM_ACVR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR5_MSS_R5SS0_ETM_ACVR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR5_MSS_R5SS0_ETM_ACVR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR5_MSS_R5SS0_ETM_ACVR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR6_MSS_R5SS0_ETM_ACVR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR6_MSS_R5SS0_ETM_ACVR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR6_MSS_R5SS0_ETM_ACVR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR6_MSS_R5SS0_ETM_ACVR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR7_MSS_R5SS0_ETM_ACVR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR7_MSS_R5SS0_ETM_ACVR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR7_MSS_R5SS0_ETM_ACVR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR7_MSS_R5SS0_ETM_ACVR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR8_MSS_R5SS0_ETM_ACVR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR8_MSS_R5SS0_ETM_ACVR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR8_MSS_R5SS0_ETM_ACVR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR8_MSS_R5SS0_ETM_ACVR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR9 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR9_MSS_R5SS0_ETM_ACVR9_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR9_MSS_R5SS0_ETM_ACVR9_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR9_MSS_R5SS0_ETM_ACVR9_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR9_MSS_R5SS0_ETM_ACVR9_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR9_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR10 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR10_MSS_R5SS0_ETM_ACVR10_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR10_MSS_R5SS0_ETM_ACVR10_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR10_MSS_R5SS0_ETM_ACVR10_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR10_MSS_R5SS0_ETM_ACVR10_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR10_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR11 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR11_MSS_R5SS0_ETM_ACVR11_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR11_MSS_R5SS0_ETM_ACVR11_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR11_MSS_R5SS0_ETM_ACVR11_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR11_MSS_R5SS0_ETM_ACVR11_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR11_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR12 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR12_MSS_R5SS0_ETM_ACVR12_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR12_MSS_R5SS0_ETM_ACVR12_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR12_MSS_R5SS0_ETM_ACVR12_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR12_MSS_R5SS0_ETM_ACVR12_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR12_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR13 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR13_MSS_R5SS0_ETM_ACVR13_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR13_MSS_R5SS0_ETM_ACVR13_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR13_MSS_R5SS0_ETM_ACVR13_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR13_MSS_R5SS0_ETM_ACVR13_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR13_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR14 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR14_MSS_R5SS0_ETM_ACVR14_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR14_MSS_R5SS0_ETM_ACVR14_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR14_MSS_R5SS0_ETM_ACVR14_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR14_MSS_R5SS0_ETM_ACVR14_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR14_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR15 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR15_MSS_R5SS0_ETM_ACVR15_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR15_MSS_R5SS0_ETM_ACVR15_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR15_MSS_R5SS0_ETM_ACVR15_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR15_MSS_R5SS0_ETM_ACVR15_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR15_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACVR16 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR16_MSS_R5SS0_ETM_ACVR16_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR16_MSS_R5SS0_ETM_ACVR16_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR16_MSS_R5SS0_ETM_ACVR16_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR16_MSS_R5SS0_ETM_ACVR16_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACVR16_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR1_MSS_R5SS0_ETM_ACTR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR1_MSS_R5SS0_ETM_ACTR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR1_MSS_R5SS0_ETM_ACTR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR1_MSS_R5SS0_ETM_ACTR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR2_MSS_R5SS0_ETM_ACTR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR2_MSS_R5SS0_ETM_ACTR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR2_MSS_R5SS0_ETM_ACTR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR2_MSS_R5SS0_ETM_ACTR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR3_MSS_R5SS0_ETM_ACTR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR3_MSS_R5SS0_ETM_ACTR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR3_MSS_R5SS0_ETM_ACTR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR3_MSS_R5SS0_ETM_ACTR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR4_MSS_R5SS0_ETM_ACTR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR4_MSS_R5SS0_ETM_ACTR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR4_MSS_R5SS0_ETM_ACTR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR4_MSS_R5SS0_ETM_ACTR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR5_MSS_R5SS0_ETM_ACTR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR5_MSS_R5SS0_ETM_ACTR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR5_MSS_R5SS0_ETM_ACTR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR5_MSS_R5SS0_ETM_ACTR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR6_MSS_R5SS0_ETM_ACTR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR6_MSS_R5SS0_ETM_ACTR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR6_MSS_R5SS0_ETM_ACTR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR6_MSS_R5SS0_ETM_ACTR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR7_MSS_R5SS0_ETM_ACTR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR7_MSS_R5SS0_ETM_ACTR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR7_MSS_R5SS0_ETM_ACTR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR7_MSS_R5SS0_ETM_ACTR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR8_MSS_R5SS0_ETM_ACTR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR8_MSS_R5SS0_ETM_ACTR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR8_MSS_R5SS0_ETM_ACTR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR8_MSS_R5SS0_ETM_ACTR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR9 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR9_MSS_R5SS0_ETM_ACTR9_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR9_MSS_R5SS0_ETM_ACTR9_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR9_MSS_R5SS0_ETM_ACTR9_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR9_MSS_R5SS0_ETM_ACTR9_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR9_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR10 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR10_MSS_R5SS0_ETM_ACTR10_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR10_MSS_R5SS0_ETM_ACTR10_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR10_MSS_R5SS0_ETM_ACTR10_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR10_MSS_R5SS0_ETM_ACTR10_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR10_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR11 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR11_MSS_R5SS0_ETM_ACTR11_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR11_MSS_R5SS0_ETM_ACTR11_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR11_MSS_R5SS0_ETM_ACTR11_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR11_MSS_R5SS0_ETM_ACTR11_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR11_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR12 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR12_MSS_R5SS0_ETM_ACTR12_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR12_MSS_R5SS0_ETM_ACTR12_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR12_MSS_R5SS0_ETM_ACTR12_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR12_MSS_R5SS0_ETM_ACTR12_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR12_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR13 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR13_MSS_R5SS0_ETM_ACTR13_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR13_MSS_R5SS0_ETM_ACTR13_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR13_MSS_R5SS0_ETM_ACTR13_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR13_MSS_R5SS0_ETM_ACTR13_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR13_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR14 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR14_MSS_R5SS0_ETM_ACTR14_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR14_MSS_R5SS0_ETM_ACTR14_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR14_MSS_R5SS0_ETM_ACTR14_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR14_MSS_R5SS0_ETM_ACTR14_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR14_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR15 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR15_MSS_R5SS0_ETM_ACTR15_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR15_MSS_R5SS0_ETM_ACTR15_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR15_MSS_R5SS0_ETM_ACTR15_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR15_MSS_R5SS0_ETM_ACTR15_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR15_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_ACTR16 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR16_MSS_R5SS0_ETM_ACTR16_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR16_MSS_R5SS0_ETM_ACTR16_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR16_MSS_R5SS0_ETM_ACTR16_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR16_MSS_R5SS0_ETM_ACTR16_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ACTR16_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_DCVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR1_MSS_R5SS0_ETM_DCVR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR1_MSS_R5SS0_ETM_DCVR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR1_MSS_R5SS0_ETM_DCVR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR1_MSS_R5SS0_ETM_DCVR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR2_MSS_R5SS0_ETM_DCVR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR2_MSS_R5SS0_ETM_DCVR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR2_MSS_R5SS0_ETM_DCVR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR2_MSS_R5SS0_ETM_DCVR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR3_MSS_R5SS0_ETM_DCVR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR3_MSS_R5SS0_ETM_DCVR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR3_MSS_R5SS0_ETM_DCVR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR3_MSS_R5SS0_ETM_DCVR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR4_MSS_R5SS0_ETM_DCVR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR4_MSS_R5SS0_ETM_DCVR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR4_MSS_R5SS0_ETM_DCVR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR4_MSS_R5SS0_ETM_DCVR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR5_MSS_R5SS0_ETM_DCVR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR5_MSS_R5SS0_ETM_DCVR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR5_MSS_R5SS0_ETM_DCVR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR5_MSS_R5SS0_ETM_DCVR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR6_MSS_R5SS0_ETM_DCVR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR6_MSS_R5SS0_ETM_DCVR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR6_MSS_R5SS0_ETM_DCVR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR6_MSS_R5SS0_ETM_DCVR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR7_MSS_R5SS0_ETM_DCVR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR7_MSS_R5SS0_ETM_DCVR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR7_MSS_R5SS0_ETM_DCVR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR7_MSS_R5SS0_ETM_DCVR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCVR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR8_MSS_R5SS0_ETM_DCVR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR8_MSS_R5SS0_ETM_DCVR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR8_MSS_R5SS0_ETM_DCVR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR8_MSS_R5SS0_ETM_DCVR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCVR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCMR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR1_MSS_R5SS0_ETM_DCMR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR1_MSS_R5SS0_ETM_DCMR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR1_MSS_R5SS0_ETM_DCMR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR1_MSS_R5SS0_ETM_DCMR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCMR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR2_MSS_R5SS0_ETM_DCMR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR2_MSS_R5SS0_ETM_DCMR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR2_MSS_R5SS0_ETM_DCMR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR2_MSS_R5SS0_ETM_DCMR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCMR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR3_MSS_R5SS0_ETM_DCMR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR3_MSS_R5SS0_ETM_DCMR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR3_MSS_R5SS0_ETM_DCMR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR3_MSS_R5SS0_ETM_DCMR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCMR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR4_MSS_R5SS0_ETM_DCMR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR4_MSS_R5SS0_ETM_DCMR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR4_MSS_R5SS0_ETM_DCMR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR4_MSS_R5SS0_ETM_DCMR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCMR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR5_MSS_R5SS0_ETM_DCMR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR5_MSS_R5SS0_ETM_DCMR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR5_MSS_R5SS0_ETM_DCMR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR5_MSS_R5SS0_ETM_DCMR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCMR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR6_MSS_R5SS0_ETM_DCMR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR6_MSS_R5SS0_ETM_DCMR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR6_MSS_R5SS0_ETM_DCMR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR6_MSS_R5SS0_ETM_DCMR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCMR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR7_MSS_R5SS0_ETM_DCMR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR7_MSS_R5SS0_ETM_DCMR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR7_MSS_R5SS0_ETM_DCMR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR7_MSS_R5SS0_ETM_DCMR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DCMR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR8_MSS_R5SS0_ETM_DCMR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR8_MSS_R5SS0_ETM_DCMR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR8_MSS_R5SS0_ETM_DCMR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR8_MSS_R5SS0_ETM_DCMR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DCMR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTRLDVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR1_MSS_R5SS0_ETM_CNTRLDVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR1_MSS_R5SS0_ETM_CNTRLDVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR1_MSS_R5SS0_ETM_CNTRLDVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR1_MSS_R5SS0_ETM_CNTRLDVR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR1_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTRLDVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR2_MSS_R5SS0_ETM_CNTRLDVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR2_MSS_R5SS0_ETM_CNTRLDVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR2_MSS_R5SS0_ETM_CNTRLDVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR2_MSS_R5SS0_ETM_CNTRLDVR2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR2_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTRLDVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR3_MSS_R5SS0_ETM_CNTRLDVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR3_MSS_R5SS0_ETM_CNTRLDVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR3_MSS_R5SS0_ETM_CNTRLDVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR3_MSS_R5SS0_ETM_CNTRLDVR3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR3_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTRLDVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR4_MSS_R5SS0_ETM_CNTRLDVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR4_MSS_R5SS0_ETM_CNTRLDVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR4_MSS_R5SS0_ETM_CNTRLDVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR4_MSS_R5SS0_ETM_CNTRLDVR4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDVR4_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTENR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR1_MSS_R5SS0_ETM_CNTENR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR1_MSS_R5SS0_ETM_CNTENR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR1_MSS_R5SS0_ETM_CNTENR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR1_MSS_R5SS0_ETM_CNTENR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTENR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR2_MSS_R5SS0_ETM_CNTENR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR2_MSS_R5SS0_ETM_CNTENR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR2_MSS_R5SS0_ETM_CNTENR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR2_MSS_R5SS0_ETM_CNTENR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTENR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR3_MSS_R5SS0_ETM_CNTENR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR3_MSS_R5SS0_ETM_CNTENR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR3_MSS_R5SS0_ETM_CNTENR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR3_MSS_R5SS0_ETM_CNTENR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTENR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR4_MSS_R5SS0_ETM_CNTENR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR4_MSS_R5SS0_ETM_CNTENR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR4_MSS_R5SS0_ETM_CNTENR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR4_MSS_R5SS0_ETM_CNTENR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTENR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTRLDEVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR1_MSS_R5SS0_ETM_CNTRLDEVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR1_MSS_R5SS0_ETM_CNTRLDEVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR1_MSS_R5SS0_ETM_CNTRLDEVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR1_MSS_R5SS0_ETM_CNTRLDEVR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR1_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTRLDEVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR2_MSS_R5SS0_ETM_CNTRLDEVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR2_MSS_R5SS0_ETM_CNTRLDEVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR2_MSS_R5SS0_ETM_CNTRLDEVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR2_MSS_R5SS0_ETM_CNTRLDEVR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR2_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTRLDEVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR3_MSS_R5SS0_ETM_CNTRLDEVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR3_MSS_R5SS0_ETM_CNTRLDEVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR3_MSS_R5SS0_ETM_CNTRLDEVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR3_MSS_R5SS0_ETM_CNTRLDEVR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR3_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTRLDEVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR4_MSS_R5SS0_ETM_CNTRLDEVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR4_MSS_R5SS0_ETM_CNTRLDEVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR4_MSS_R5SS0_ETM_CNTRLDEVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR4_MSS_R5SS0_ETM_CNTRLDEVR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTRLDEVR4_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR1_MSS_R5SS0_ETM_CNTVR1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR1_MSS_R5SS0_ETM_CNTVR1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR1_MSS_R5SS0_ETM_CNTVR1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR1_MSS_R5SS0_ETM_CNTVR1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR1_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR2_MSS_R5SS0_ETM_CNTVR2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR2_MSS_R5SS0_ETM_CNTVR2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR2_MSS_R5SS0_ETM_CNTVR2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR2_MSS_R5SS0_ETM_CNTVR2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR2_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR3_MSS_R5SS0_ETM_CNTVR3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR3_MSS_R5SS0_ETM_CNTVR3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR3_MSS_R5SS0_ETM_CNTVR3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR3_MSS_R5SS0_ETM_CNTVR3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR3_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_CNTVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR4_MSS_R5SS0_ETM_CNTVR4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR4_MSS_R5SS0_ETM_CNTVR4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR4_MSS_R5SS0_ETM_CNTVR4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR4_MSS_R5SS0_ETM_CNTVR4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CNTVR4_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_SQ12EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ12EVR_MSS_R5SS0_ETM_SQ12EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ12EVR_MSS_R5SS0_ETM_SQ12EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ12EVR_MSS_R5SS0_ETM_SQ12EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ12EVR_MSS_R5SS0_ETM_SQ12EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ12EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_SQ21EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ21EVR_MSS_R5SS0_ETM_SQ21EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ21EVR_MSS_R5SS0_ETM_SQ21EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ21EVR_MSS_R5SS0_ETM_SQ21EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ21EVR_MSS_R5SS0_ETM_SQ21EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ21EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_SQ23EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ23EVR_MSS_R5SS0_ETM_SQ23EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ23EVR_MSS_R5SS0_ETM_SQ23EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ23EVR_MSS_R5SS0_ETM_SQ23EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ23EVR_MSS_R5SS0_ETM_SQ23EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ23EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_SQ31EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ31EVR_MSS_R5SS0_ETM_SQ31EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ31EVR_MSS_R5SS0_ETM_SQ31EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ31EVR_MSS_R5SS0_ETM_SQ31EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ31EVR_MSS_R5SS0_ETM_SQ31EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ31EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_SQ32EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ32EVR_MSS_R5SS0_ETM_SQ32EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ32EVR_MSS_R5SS0_ETM_SQ32EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ32EVR_MSS_R5SS0_ETM_SQ32EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ32EVR_MSS_R5SS0_ETM_SQ32EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ32EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_SQ13EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ13EVR_MSS_R5SS0_ETM_SQ13EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ13EVR_MSS_R5SS0_ETM_SQ13EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ13EVR_MSS_R5SS0_ETM_SQ13EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ13EVR_MSS_R5SS0_ETM_SQ13EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQ13EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_SQR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQR_MSS_R5SS0_ETM_SQR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQR_MSS_R5SS0_ETM_SQR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQR_MSS_R5SS0_ETM_SQR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQR_MSS_R5SS0_ETM_SQR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SQR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_A_ETM_EXTOUTEVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR1_MSS_R5SS0_ETM_EXTOUTEVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR1_MSS_R5SS0_ETM_EXTOUTEVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR1_MSS_R5SS0_ETM_EXTOUTEVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR1_MSS_R5SS0_ETM_EXTOUTEVR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR1_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_EXTOUTEVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR2_MSS_R5SS0_ETM_EXTOUTEVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR2_MSS_R5SS0_ETM_EXTOUTEVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR2_MSS_R5SS0_ETM_EXTOUTEVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR2_MSS_R5SS0_ETM_EXTOUTEVR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR2_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_EXTOUTEVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR3_MSS_R5SS0_ETM_EXTOUTEVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR3_MSS_R5SS0_ETM_EXTOUTEVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR3_MSS_R5SS0_ETM_EXTOUTEVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR3_MSS_R5SS0_ETM_EXTOUTEVR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR3_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_EXTOUTEVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR4_MSS_R5SS0_ETM_EXTOUTEVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR4_MSS_R5SS0_ETM_EXTOUTEVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR4_MSS_R5SS0_ETM_EXTOUTEVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR4_MSS_R5SS0_ETM_EXTOUTEVR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTOUTEVR4_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_CIDCVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR1_MSS_R5SS0_ETM_CIDCVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR1_MSS_R5SS0_ETM_CIDCVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR1_MSS_R5SS0_ETM_CIDCVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR1_MSS_R5SS0_ETM_CIDCVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_CIDCVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR2_MSS_R5SS0_ETM_CIDCVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR2_MSS_R5SS0_ETM_CIDCVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR2_MSS_R5SS0_ETM_CIDCVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR2_MSS_R5SS0_ETM_CIDCVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_CIDCVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR3_MSS_R5SS0_ETM_CIDCVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR3_MSS_R5SS0_ETM_CIDCVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR3_MSS_R5SS0_ETM_CIDCVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR3_MSS_R5SS0_ETM_CIDCVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_CIDCMR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCMR_MSS_R5SS0_ETM_CIDCMR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCMR_MSS_R5SS0_ETM_CIDCMR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCMR_MSS_R5SS0_ETM_CIDCMR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCMR_MSS_R5SS0_ETM_CIDCMR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDCMR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_SYNCFR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SYNCFR_MSS_R5SS0_ETM_SYNCFR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SYNCFR_MSS_R5SS0_ETM_SYNCFR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SYNCFR_MSS_R5SS0_ETM_SYNCFR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SYNCFR_MSS_R5SS0_ETM_SYNCFR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_SYNCFR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_IDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_IDR_MSS_R5SS0_ETM_IDR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_IDR_MSS_R5SS0_ETM_IDR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_IDR_MSS_R5SS0_ETM_IDR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_IDR_MSS_R5SS0_ETM_IDR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_IDR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_A_ETM_CCER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCER_MSS_R5SS0_ETM_CCER_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCER_MSS_R5SS0_ETM_CCER_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCER_MSS_R5SS0_ETM_CCER_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCER_MSS_R5SS0_ETM_CCER_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CCER_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_A_ETM_EXTINSELR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTINSELR_MSS_R5SS0_ETM_EXTINSELR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTINSELR_MSS_R5SS0_ETM_EXTINSELR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTINSELR_MSS_R5SS0_ETM_EXTINSELR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTINSELR_MSS_R5SS0_ETM_EXTINSELR_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_EXTINSELR_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_TRACEIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRACEIDR_MSS_R5SS0_ETM_TRACEIDR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRACEIDR_MSS_R5SS0_ETM_TRACEIDR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRACEIDR_MSS_R5SS0_ETM_TRACEIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRACEIDR_MSS_R5SS0_ETM_TRACEIDR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_TRACEIDR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_ETM_PDSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PDSR_MSS_R5SS0_ETM_PDSR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PDSR_MSS_R5SS0_ETM_PDSR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PDSR_MSS_R5SS0_ETM_PDSR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PDSR_MSS_R5SS0_ETM_PDSR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PDSR_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_A_ETM_ITETMIF */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITETMIF_MSS_R5SS0_ETM_ITETMIF_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITETMIF_MSS_R5SS0_ETM_ITETMIF_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITETMIF_MSS_R5SS0_ETM_ITETMIF_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITETMIF_MSS_R5SS0_ETM_ITETMIF_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITETMIF_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_ITMISCOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCOUT_MSS_R5SS0_ETM_ITMISCOUT_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCOUT_MSS_R5SS0_ETM_ITMISCOUT_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCOUT_MSS_R5SS0_ETM_ITMISCOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCOUT_MSS_R5SS0_ETM_ITMISCOUT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCOUT_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_ITMISCIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCIN_MSS_R5SS0_ETM_ITMISCIN_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCIN_MSS_R5SS0_ETM_ITMISCIN_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCIN_MSS_R5SS0_ETM_ITMISCIN_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCIN_MSS_R5SS0_ETM_ITMISCIN_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITMISCIN_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_ETM_ITTRIGGERACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERACK_MSS_R5SS0_ETM_ITTRIGGERACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERACK_MSS_R5SS0_ETM_ITTRIGGERACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERACK_MSS_R5SS0_ETM_ITTRIGGERACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERACK_MSS_R5SS0_ETM_ITTRIGGERACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERACK_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_ETM_ITTRIGGERREQ */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERREQ_MSS_R5SS0_ETM_ITTRIGGERREQ_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERREQ_MSS_R5SS0_ETM_ITTRIGGERREQ_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERREQ_MSS_R5SS0_ETM_ITTRIGGERREQ_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERREQ_MSS_R5SS0_ETM_ITTRIGGERREQ_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITTRIGGERREQ_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_A_ETM_ITATBDATA0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBDATA0_MSS_R5SS0_ETM_ITATBDATA0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBDATA0_MSS_R5SS0_ETM_ITATBDATA0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBDATA0_MSS_R5SS0_ETM_ITATBDATA0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBDATA0_MSS_R5SS0_ETM_ITATBDATA0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBDATA0_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_ITATBCTR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR2_MSS_R5SS0_ETM_ITATBCTR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR2_MSS_R5SS0_ETM_ITATBCTR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR2_MSS_R5SS0_ETM_ITATBCTR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR2_MSS_R5SS0_ETM_ITATBCTR2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR2_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_ITATBCTR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR1_MSS_R5SS0_ETM_ITATBCTR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR1_MSS_R5SS0_ETM_ITATBCTR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR1_MSS_R5SS0_ETM_ITATBCTR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR1_MSS_R5SS0_ETM_ITATBCTR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR1_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_ITATBCTR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR0_MSS_R5SS0_ETM_ITATBCTR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR0_MSS_R5SS0_ETM_ITATBCTR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR0_MSS_R5SS0_ETM_ITATBCTR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR0_MSS_R5SS0_ETM_ITATBCTR0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITATBCTR0_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_A_ETM_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITCTRL_MSS_R5SS0_ETM_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITCTRL_MSS_R5SS0_ETM_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITCTRL_MSS_R5SS0_ETM_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITCTRL_MSS_R5SS0_ETM_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_ITCTRL_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_A_ETM_CLAIMSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMSET_MSS_R5SS0_ETM_CLAIMSET_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMSET_MSS_R5SS0_ETM_CLAIMSET_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMSET_MSS_R5SS0_ETM_CLAIMSET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMSET_MSS_R5SS0_ETM_CLAIMSET_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMSET_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_ETM_CLAIMCLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMCLR_MSS_R5SS0_ETM_CLAIMCLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMCLR_MSS_R5SS0_ETM_CLAIMCLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMCLR_MSS_R5SS0_ETM_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMCLR_MSS_R5SS0_ETM_CLAIMCLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CLAIMCLR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_A_ETM_LAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LAR_MSS_R5SS0_ETM_LAR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LAR_MSS_R5SS0_ETM_LAR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LAR_MSS_R5SS0_ETM_LAR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LAR_MSS_R5SS0_ETM_LAR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LAR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_A_ETM_LSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LSR_MSS_R5SS0_ETM_LSR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LSR_MSS_R5SS0_ETM_LSR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LSR_MSS_R5SS0_ETM_LSR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LSR_MSS_R5SS0_ETM_LSR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_LSR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_A_ETM_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_AUTHSTATUS_MSS_R5SS0_ETM_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_AUTHSTATUS_MSS_R5SS0_ETM_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_AUTHSTATUS_MSS_R5SS0_ETM_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_AUTHSTATUS_MSS_R5SS0_ETM_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_AUTHSTATUS_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_A_ETM_DEVID */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVID_MSS_R5SS0_ETM_DEVID_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVID_MSS_R5SS0_ETM_DEVID_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVID_MSS_R5SS0_ETM_DEVID_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVID_MSS_R5SS0_ETM_DEVID_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVID_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_DEVTYPE */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVTYPE_MSS_R5SS0_ETM_DEVTYPE_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVTYPE_MSS_R5SS0_ETM_DEVTYPE_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVTYPE_MSS_R5SS0_ETM_DEVTYPE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVTYPE_MSS_R5SS0_ETM_DEVTYPE_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_DEVTYPE_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_A_ETM_PIDR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR4_MSS_R5SS0_ETM_PIDR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR4_MSS_R5SS0_ETM_PIDR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR4_MSS_R5SS0_ETM_PIDR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR4_MSS_R5SS0_ETM_PIDR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_PIDR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR5_MSS_R5SS0_ETM_PIDR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR5_MSS_R5SS0_ETM_PIDR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR5_MSS_R5SS0_ETM_PIDR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR5_MSS_R5SS0_ETM_PIDR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_PIDR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR6_MSS_R5SS0_ETM_PIDR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR6_MSS_R5SS0_ETM_PIDR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR6_MSS_R5SS0_ETM_PIDR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR6_MSS_R5SS0_ETM_PIDR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_PIDR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR7_MSS_R5SS0_ETM_PIDR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR7_MSS_R5SS0_ETM_PIDR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR7_MSS_R5SS0_ETM_PIDR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR7_MSS_R5SS0_ETM_PIDR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_PIDR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR0_MSS_R5SS0_ETM_PIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR0_MSS_R5SS0_ETM_PIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR0_MSS_R5SS0_ETM_PIDR0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR0_MSS_R5SS0_ETM_PIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR0_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_PIDR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR1_MSS_R5SS0_ETM_PIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR1_MSS_R5SS0_ETM_PIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR1_MSS_R5SS0_ETM_PIDR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR1_MSS_R5SS0_ETM_PIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_PIDR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR2_MSS_R5SS0_ETM_PIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR2_MSS_R5SS0_ETM_PIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR2_MSS_R5SS0_ETM_PIDR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR2_MSS_R5SS0_ETM_PIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_PIDR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR3_MSS_R5SS0_ETM_PIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR3_MSS_R5SS0_ETM_PIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR3_MSS_R5SS0_ETM_PIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR3_MSS_R5SS0_ETM_PIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_PIDR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_CIDR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR0_MSS_R5SS0_ETM_CIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR0_MSS_R5SS0_ETM_CIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR0_MSS_R5SS0_ETM_CIDR0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR0_MSS_R5SS0_ETM_CIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR0_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_CIDR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR1_MSS_R5SS0_ETM_CIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR1_MSS_R5SS0_ETM_CIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR1_MSS_R5SS0_ETM_CIDR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR1_MSS_R5SS0_ETM_CIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_CIDR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR2_MSS_R5SS0_ETM_CIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR2_MSS_R5SS0_ETM_CIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR2_MSS_R5SS0_ETM_CIDR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR2_MSS_R5SS0_ETM_CIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_A_ETM_CIDR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR3_MSS_R5SS0_ETM_CIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR3_MSS_R5SS0_ETM_CIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR3_MSS_R5SS0_ETM_CIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR3_MSS_R5SS0_ETM_CIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_A_ETM_CIDR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_CR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CR_MSS_R5SS0_ETM_CR_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CR_MSS_R5SS0_ETM_CR_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CR_MSS_R5SS0_ETM_CR_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CR_MSS_R5SS0_ETM_CR_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CR_RESETVAL                            (0x00000000U)

/* MSS_R5SS0_B_ETM_CCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCR_MSS_R5SS0_ETM_CCR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCR_MSS_R5SS0_ETM_CCR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCR_MSS_R5SS0_ETM_CCR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCR_MSS_R5SS0_ETM_CCR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_B_ETM_TRIGGER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRIGGER_MSS_R5SS0_ETM_TRIGGER_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRIGGER_MSS_R5SS0_ETM_TRIGGER_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRIGGER_MSS_R5SS0_ETM_TRIGGER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRIGGER_MSS_R5SS0_ETM_TRIGGER_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRIGGER_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_ASICCTLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ASICCTLR_MSS_R5SS0_ETM_ASICCTLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ASICCTLR_MSS_R5SS0_ETM_ASICCTLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ASICCTLR_MSS_R5SS0_ETM_ASICCTLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ASICCTLR_MSS_R5SS0_ETM_ASICCTLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ASICCTLR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_ETM_SR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SR_MSS_R5SS0_ETM_SR_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SR_MSS_R5SS0_ETM_SR_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SR_MSS_R5SS0_ETM_SR_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SR_MSS_R5SS0_ETM_SR_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SR_RESETVAL                            (0x00000000U)

/* MSS_R5SS0_B_ETM_SCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SCR_MSS_R5SS0_ETM_SCR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SCR_MSS_R5SS0_ETM_SCR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SCR_MSS_R5SS0_ETM_SCR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SCR_MSS_R5SS0_ETM_SCR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SCR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_B_ETM_TSSCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TSSCR_MSS_R5SS0_ETM_TSSCR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TSSCR_MSS_R5SS0_ETM_TSSCR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TSSCR_MSS_R5SS0_ETM_TSSCR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TSSCR_MSS_R5SS0_ETM_TSSCR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TSSCR_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_TECR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR2_MSS_R5SS0_ETM_TECR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR2_MSS_R5SS0_ETM_TECR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR2_MSS_R5SS0_ETM_TECR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR2_MSS_R5SS0_ETM_TECR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_TEEVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TEEVR_MSS_R5SS0_ETM_TEEVR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TEEVR_MSS_R5SS0_ETM_TEEVR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TEEVR_MSS_R5SS0_ETM_TEEVR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TEEVR_MSS_R5SS0_ETM_TEEVR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TEEVR_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_TECR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR1_MSS_R5SS0_ETM_TECR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR1_MSS_R5SS0_ETM_TECR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR1_MSS_R5SS0_ETM_TECR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR1_MSS_R5SS0_ETM_TECR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TECR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_FFRR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFRR_MSS_R5SS0_ETM_FFRR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFRR_MSS_R5SS0_ETM_FFRR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFRR_MSS_R5SS0_ETM_FFRR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFRR_MSS_R5SS0_ETM_FFRR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFRR_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_B_ETM_FFLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFLR_MSS_R5SS0_ETM_FFLR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFLR_MSS_R5SS0_ETM_FFLR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFLR_MSS_R5SS0_ETM_FFLR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFLR_MSS_R5SS0_ETM_FFLR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_FFLR_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_B_ETM_VDEVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDEVR_MSS_R5SS0_ETM_VDEVR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDEVR_MSS_R5SS0_ETM_VDEVR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDEVR_MSS_R5SS0_ETM_VDEVR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDEVR_MSS_R5SS0_ETM_VDEVR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDEVR_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_VDCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR1_MSS_R5SS0_ETM_VDCR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR1_MSS_R5SS0_ETM_VDCR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR1_MSS_R5SS0_ETM_VDCR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR1_MSS_R5SS0_ETM_VDCR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_VDCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR2_MSS_R5SS0_ETM_VDCR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR2_MSS_R5SS0_ETM_VDCR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR2_MSS_R5SS0_ETM_VDCR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR2_MSS_R5SS0_ETM_VDCR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_VDCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR3_MSS_R5SS0_ETM_VDCR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR3_MSS_R5SS0_ETM_VDCR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR3_MSS_R5SS0_ETM_VDCR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR3_MSS_R5SS0_ETM_VDCR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_VDCR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR1_MSS_R5SS0_ETM_ACVR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR1_MSS_R5SS0_ETM_ACVR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR1_MSS_R5SS0_ETM_ACVR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR1_MSS_R5SS0_ETM_ACVR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR2_MSS_R5SS0_ETM_ACVR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR2_MSS_R5SS0_ETM_ACVR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR2_MSS_R5SS0_ETM_ACVR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR2_MSS_R5SS0_ETM_ACVR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR3_MSS_R5SS0_ETM_ACVR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR3_MSS_R5SS0_ETM_ACVR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR3_MSS_R5SS0_ETM_ACVR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR3_MSS_R5SS0_ETM_ACVR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR4_MSS_R5SS0_ETM_ACVR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR4_MSS_R5SS0_ETM_ACVR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR4_MSS_R5SS0_ETM_ACVR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR4_MSS_R5SS0_ETM_ACVR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR5_MSS_R5SS0_ETM_ACVR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR5_MSS_R5SS0_ETM_ACVR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR5_MSS_R5SS0_ETM_ACVR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR5_MSS_R5SS0_ETM_ACVR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR6_MSS_R5SS0_ETM_ACVR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR6_MSS_R5SS0_ETM_ACVR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR6_MSS_R5SS0_ETM_ACVR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR6_MSS_R5SS0_ETM_ACVR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR7_MSS_R5SS0_ETM_ACVR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR7_MSS_R5SS0_ETM_ACVR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR7_MSS_R5SS0_ETM_ACVR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR7_MSS_R5SS0_ETM_ACVR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR8_MSS_R5SS0_ETM_ACVR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR8_MSS_R5SS0_ETM_ACVR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR8_MSS_R5SS0_ETM_ACVR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR8_MSS_R5SS0_ETM_ACVR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR9 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR9_MSS_R5SS0_ETM_ACVR9_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR9_MSS_R5SS0_ETM_ACVR9_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR9_MSS_R5SS0_ETM_ACVR9_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR9_MSS_R5SS0_ETM_ACVR9_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR9_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR10 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR10_MSS_R5SS0_ETM_ACVR10_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR10_MSS_R5SS0_ETM_ACVR10_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR10_MSS_R5SS0_ETM_ACVR10_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR10_MSS_R5SS0_ETM_ACVR10_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR10_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR11 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR11_MSS_R5SS0_ETM_ACVR11_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR11_MSS_R5SS0_ETM_ACVR11_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR11_MSS_R5SS0_ETM_ACVR11_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR11_MSS_R5SS0_ETM_ACVR11_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR11_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR12 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR12_MSS_R5SS0_ETM_ACVR12_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR12_MSS_R5SS0_ETM_ACVR12_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR12_MSS_R5SS0_ETM_ACVR12_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR12_MSS_R5SS0_ETM_ACVR12_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR12_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR13 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR13_MSS_R5SS0_ETM_ACVR13_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR13_MSS_R5SS0_ETM_ACVR13_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR13_MSS_R5SS0_ETM_ACVR13_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR13_MSS_R5SS0_ETM_ACVR13_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR13_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR14 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR14_MSS_R5SS0_ETM_ACVR14_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR14_MSS_R5SS0_ETM_ACVR14_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR14_MSS_R5SS0_ETM_ACVR14_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR14_MSS_R5SS0_ETM_ACVR14_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR14_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR15 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR15_MSS_R5SS0_ETM_ACVR15_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR15_MSS_R5SS0_ETM_ACVR15_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR15_MSS_R5SS0_ETM_ACVR15_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR15_MSS_R5SS0_ETM_ACVR15_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR15_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACVR16 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR16_MSS_R5SS0_ETM_ACVR16_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR16_MSS_R5SS0_ETM_ACVR16_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR16_MSS_R5SS0_ETM_ACVR16_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR16_MSS_R5SS0_ETM_ACVR16_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACVR16_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR1_MSS_R5SS0_ETM_ACTR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR1_MSS_R5SS0_ETM_ACTR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR1_MSS_R5SS0_ETM_ACTR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR1_MSS_R5SS0_ETM_ACTR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR2_MSS_R5SS0_ETM_ACTR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR2_MSS_R5SS0_ETM_ACTR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR2_MSS_R5SS0_ETM_ACTR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR2_MSS_R5SS0_ETM_ACTR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR3_MSS_R5SS0_ETM_ACTR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR3_MSS_R5SS0_ETM_ACTR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR3_MSS_R5SS0_ETM_ACTR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR3_MSS_R5SS0_ETM_ACTR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR4_MSS_R5SS0_ETM_ACTR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR4_MSS_R5SS0_ETM_ACTR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR4_MSS_R5SS0_ETM_ACTR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR4_MSS_R5SS0_ETM_ACTR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR5_MSS_R5SS0_ETM_ACTR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR5_MSS_R5SS0_ETM_ACTR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR5_MSS_R5SS0_ETM_ACTR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR5_MSS_R5SS0_ETM_ACTR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR6_MSS_R5SS0_ETM_ACTR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR6_MSS_R5SS0_ETM_ACTR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR6_MSS_R5SS0_ETM_ACTR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR6_MSS_R5SS0_ETM_ACTR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR7_MSS_R5SS0_ETM_ACTR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR7_MSS_R5SS0_ETM_ACTR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR7_MSS_R5SS0_ETM_ACTR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR7_MSS_R5SS0_ETM_ACTR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR8_MSS_R5SS0_ETM_ACTR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR8_MSS_R5SS0_ETM_ACTR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR8_MSS_R5SS0_ETM_ACTR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR8_MSS_R5SS0_ETM_ACTR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR9 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR9_MSS_R5SS0_ETM_ACTR9_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR9_MSS_R5SS0_ETM_ACTR9_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR9_MSS_R5SS0_ETM_ACTR9_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR9_MSS_R5SS0_ETM_ACTR9_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR9_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR10 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR10_MSS_R5SS0_ETM_ACTR10_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR10_MSS_R5SS0_ETM_ACTR10_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR10_MSS_R5SS0_ETM_ACTR10_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR10_MSS_R5SS0_ETM_ACTR10_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR10_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR11 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR11_MSS_R5SS0_ETM_ACTR11_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR11_MSS_R5SS0_ETM_ACTR11_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR11_MSS_R5SS0_ETM_ACTR11_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR11_MSS_R5SS0_ETM_ACTR11_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR11_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR12 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR12_MSS_R5SS0_ETM_ACTR12_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR12_MSS_R5SS0_ETM_ACTR12_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR12_MSS_R5SS0_ETM_ACTR12_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR12_MSS_R5SS0_ETM_ACTR12_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR12_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR13 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR13_MSS_R5SS0_ETM_ACTR13_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR13_MSS_R5SS0_ETM_ACTR13_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR13_MSS_R5SS0_ETM_ACTR13_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR13_MSS_R5SS0_ETM_ACTR13_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR13_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR14 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR14_MSS_R5SS0_ETM_ACTR14_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR14_MSS_R5SS0_ETM_ACTR14_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR14_MSS_R5SS0_ETM_ACTR14_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR14_MSS_R5SS0_ETM_ACTR14_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR14_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR15 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR15_MSS_R5SS0_ETM_ACTR15_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR15_MSS_R5SS0_ETM_ACTR15_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR15_MSS_R5SS0_ETM_ACTR15_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR15_MSS_R5SS0_ETM_ACTR15_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR15_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_ACTR16 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR16_MSS_R5SS0_ETM_ACTR16_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR16_MSS_R5SS0_ETM_ACTR16_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR16_MSS_R5SS0_ETM_ACTR16_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR16_MSS_R5SS0_ETM_ACTR16_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ACTR16_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_DCVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR1_MSS_R5SS0_ETM_DCVR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR1_MSS_R5SS0_ETM_DCVR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR1_MSS_R5SS0_ETM_DCVR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR1_MSS_R5SS0_ETM_DCVR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR2_MSS_R5SS0_ETM_DCVR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR2_MSS_R5SS0_ETM_DCVR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR2_MSS_R5SS0_ETM_DCVR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR2_MSS_R5SS0_ETM_DCVR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR3_MSS_R5SS0_ETM_DCVR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR3_MSS_R5SS0_ETM_DCVR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR3_MSS_R5SS0_ETM_DCVR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR3_MSS_R5SS0_ETM_DCVR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR4_MSS_R5SS0_ETM_DCVR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR4_MSS_R5SS0_ETM_DCVR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR4_MSS_R5SS0_ETM_DCVR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR4_MSS_R5SS0_ETM_DCVR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR5_MSS_R5SS0_ETM_DCVR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR5_MSS_R5SS0_ETM_DCVR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR5_MSS_R5SS0_ETM_DCVR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR5_MSS_R5SS0_ETM_DCVR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR6_MSS_R5SS0_ETM_DCVR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR6_MSS_R5SS0_ETM_DCVR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR6_MSS_R5SS0_ETM_DCVR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR6_MSS_R5SS0_ETM_DCVR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR7_MSS_R5SS0_ETM_DCVR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR7_MSS_R5SS0_ETM_DCVR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR7_MSS_R5SS0_ETM_DCVR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR7_MSS_R5SS0_ETM_DCVR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCVR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR8_MSS_R5SS0_ETM_DCVR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR8_MSS_R5SS0_ETM_DCVR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR8_MSS_R5SS0_ETM_DCVR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR8_MSS_R5SS0_ETM_DCVR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCVR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCMR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR1_MSS_R5SS0_ETM_DCMR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR1_MSS_R5SS0_ETM_DCMR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR1_MSS_R5SS0_ETM_DCMR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR1_MSS_R5SS0_ETM_DCMR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCMR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR2_MSS_R5SS0_ETM_DCMR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR2_MSS_R5SS0_ETM_DCMR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR2_MSS_R5SS0_ETM_DCMR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR2_MSS_R5SS0_ETM_DCMR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCMR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR3_MSS_R5SS0_ETM_DCMR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR3_MSS_R5SS0_ETM_DCMR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR3_MSS_R5SS0_ETM_DCMR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR3_MSS_R5SS0_ETM_DCMR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCMR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR4_MSS_R5SS0_ETM_DCMR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR4_MSS_R5SS0_ETM_DCMR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR4_MSS_R5SS0_ETM_DCMR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR4_MSS_R5SS0_ETM_DCMR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCMR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR5_MSS_R5SS0_ETM_DCMR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR5_MSS_R5SS0_ETM_DCMR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR5_MSS_R5SS0_ETM_DCMR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR5_MSS_R5SS0_ETM_DCMR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCMR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR6_MSS_R5SS0_ETM_DCMR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR6_MSS_R5SS0_ETM_DCMR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR6_MSS_R5SS0_ETM_DCMR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR6_MSS_R5SS0_ETM_DCMR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCMR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR7_MSS_R5SS0_ETM_DCMR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR7_MSS_R5SS0_ETM_DCMR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR7_MSS_R5SS0_ETM_DCMR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR7_MSS_R5SS0_ETM_DCMR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DCMR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR8_MSS_R5SS0_ETM_DCMR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR8_MSS_R5SS0_ETM_DCMR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR8_MSS_R5SS0_ETM_DCMR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR8_MSS_R5SS0_ETM_DCMR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DCMR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTRLDVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR1_MSS_R5SS0_ETM_CNTRLDVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR1_MSS_R5SS0_ETM_CNTRLDVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR1_MSS_R5SS0_ETM_CNTRLDVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR1_MSS_R5SS0_ETM_CNTRLDVR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR1_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTRLDVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR2_MSS_R5SS0_ETM_CNTRLDVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR2_MSS_R5SS0_ETM_CNTRLDVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR2_MSS_R5SS0_ETM_CNTRLDVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR2_MSS_R5SS0_ETM_CNTRLDVR2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR2_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTRLDVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR3_MSS_R5SS0_ETM_CNTRLDVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR3_MSS_R5SS0_ETM_CNTRLDVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR3_MSS_R5SS0_ETM_CNTRLDVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR3_MSS_R5SS0_ETM_CNTRLDVR3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR3_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTRLDVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR4_MSS_R5SS0_ETM_CNTRLDVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR4_MSS_R5SS0_ETM_CNTRLDVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR4_MSS_R5SS0_ETM_CNTRLDVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR4_MSS_R5SS0_ETM_CNTRLDVR4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDVR4_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTENR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR1_MSS_R5SS0_ETM_CNTENR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR1_MSS_R5SS0_ETM_CNTENR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR1_MSS_R5SS0_ETM_CNTENR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR1_MSS_R5SS0_ETM_CNTENR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTENR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR2_MSS_R5SS0_ETM_CNTENR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR2_MSS_R5SS0_ETM_CNTENR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR2_MSS_R5SS0_ETM_CNTENR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR2_MSS_R5SS0_ETM_CNTENR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTENR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR3_MSS_R5SS0_ETM_CNTENR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR3_MSS_R5SS0_ETM_CNTENR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR3_MSS_R5SS0_ETM_CNTENR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR3_MSS_R5SS0_ETM_CNTENR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTENR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR4_MSS_R5SS0_ETM_CNTENR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR4_MSS_R5SS0_ETM_CNTENR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR4_MSS_R5SS0_ETM_CNTENR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR4_MSS_R5SS0_ETM_CNTENR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTENR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTRLDEVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR1_MSS_R5SS0_ETM_CNTRLDEVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR1_MSS_R5SS0_ETM_CNTRLDEVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR1_MSS_R5SS0_ETM_CNTRLDEVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR1_MSS_R5SS0_ETM_CNTRLDEVR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR1_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTRLDEVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR2_MSS_R5SS0_ETM_CNTRLDEVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR2_MSS_R5SS0_ETM_CNTRLDEVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR2_MSS_R5SS0_ETM_CNTRLDEVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR2_MSS_R5SS0_ETM_CNTRLDEVR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR2_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTRLDEVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR3_MSS_R5SS0_ETM_CNTRLDEVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR3_MSS_R5SS0_ETM_CNTRLDEVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR3_MSS_R5SS0_ETM_CNTRLDEVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR3_MSS_R5SS0_ETM_CNTRLDEVR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR3_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTRLDEVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR4_MSS_R5SS0_ETM_CNTRLDEVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR4_MSS_R5SS0_ETM_CNTRLDEVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR4_MSS_R5SS0_ETM_CNTRLDEVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR4_MSS_R5SS0_ETM_CNTRLDEVR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTRLDEVR4_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR1_MSS_R5SS0_ETM_CNTVR1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR1_MSS_R5SS0_ETM_CNTVR1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR1_MSS_R5SS0_ETM_CNTVR1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR1_MSS_R5SS0_ETM_CNTVR1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR1_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR2_MSS_R5SS0_ETM_CNTVR2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR2_MSS_R5SS0_ETM_CNTVR2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR2_MSS_R5SS0_ETM_CNTVR2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR2_MSS_R5SS0_ETM_CNTVR2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR2_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR3_MSS_R5SS0_ETM_CNTVR3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR3_MSS_R5SS0_ETM_CNTVR3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR3_MSS_R5SS0_ETM_CNTVR3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR3_MSS_R5SS0_ETM_CNTVR3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR3_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_CNTVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR4_MSS_R5SS0_ETM_CNTVR4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR4_MSS_R5SS0_ETM_CNTVR4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR4_MSS_R5SS0_ETM_CNTVR4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR4_MSS_R5SS0_ETM_CNTVR4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CNTVR4_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_SQ12EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ12EVR_MSS_R5SS0_ETM_SQ12EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ12EVR_MSS_R5SS0_ETM_SQ12EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ12EVR_MSS_R5SS0_ETM_SQ12EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ12EVR_MSS_R5SS0_ETM_SQ12EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ12EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_SQ21EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ21EVR_MSS_R5SS0_ETM_SQ21EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ21EVR_MSS_R5SS0_ETM_SQ21EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ21EVR_MSS_R5SS0_ETM_SQ21EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ21EVR_MSS_R5SS0_ETM_SQ21EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ21EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_SQ23EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ23EVR_MSS_R5SS0_ETM_SQ23EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ23EVR_MSS_R5SS0_ETM_SQ23EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ23EVR_MSS_R5SS0_ETM_SQ23EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ23EVR_MSS_R5SS0_ETM_SQ23EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ23EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_SQ31EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ31EVR_MSS_R5SS0_ETM_SQ31EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ31EVR_MSS_R5SS0_ETM_SQ31EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ31EVR_MSS_R5SS0_ETM_SQ31EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ31EVR_MSS_R5SS0_ETM_SQ31EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ31EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_SQ32EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ32EVR_MSS_R5SS0_ETM_SQ32EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ32EVR_MSS_R5SS0_ETM_SQ32EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ32EVR_MSS_R5SS0_ETM_SQ32EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ32EVR_MSS_R5SS0_ETM_SQ32EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ32EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_SQ13EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ13EVR_MSS_R5SS0_ETM_SQ13EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ13EVR_MSS_R5SS0_ETM_SQ13EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ13EVR_MSS_R5SS0_ETM_SQ13EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ13EVR_MSS_R5SS0_ETM_SQ13EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQ13EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_SQR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQR_MSS_R5SS0_ETM_SQR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQR_MSS_R5SS0_ETM_SQR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQR_MSS_R5SS0_ETM_SQR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQR_MSS_R5SS0_ETM_SQR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SQR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_B_ETM_EXTOUTEVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR1_MSS_R5SS0_ETM_EXTOUTEVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR1_MSS_R5SS0_ETM_EXTOUTEVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR1_MSS_R5SS0_ETM_EXTOUTEVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR1_MSS_R5SS0_ETM_EXTOUTEVR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR1_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_EXTOUTEVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR2_MSS_R5SS0_ETM_EXTOUTEVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR2_MSS_R5SS0_ETM_EXTOUTEVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR2_MSS_R5SS0_ETM_EXTOUTEVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR2_MSS_R5SS0_ETM_EXTOUTEVR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR2_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_EXTOUTEVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR3_MSS_R5SS0_ETM_EXTOUTEVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR3_MSS_R5SS0_ETM_EXTOUTEVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR3_MSS_R5SS0_ETM_EXTOUTEVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR3_MSS_R5SS0_ETM_EXTOUTEVR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR3_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_EXTOUTEVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR4_MSS_R5SS0_ETM_EXTOUTEVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR4_MSS_R5SS0_ETM_EXTOUTEVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR4_MSS_R5SS0_ETM_EXTOUTEVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR4_MSS_R5SS0_ETM_EXTOUTEVR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTOUTEVR4_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_CIDCVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR1_MSS_R5SS0_ETM_CIDCVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR1_MSS_R5SS0_ETM_CIDCVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR1_MSS_R5SS0_ETM_CIDCVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR1_MSS_R5SS0_ETM_CIDCVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_CIDCVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR2_MSS_R5SS0_ETM_CIDCVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR2_MSS_R5SS0_ETM_CIDCVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR2_MSS_R5SS0_ETM_CIDCVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR2_MSS_R5SS0_ETM_CIDCVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_CIDCVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR3_MSS_R5SS0_ETM_CIDCVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR3_MSS_R5SS0_ETM_CIDCVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR3_MSS_R5SS0_ETM_CIDCVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR3_MSS_R5SS0_ETM_CIDCVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_CIDCMR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCMR_MSS_R5SS0_ETM_CIDCMR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCMR_MSS_R5SS0_ETM_CIDCMR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCMR_MSS_R5SS0_ETM_CIDCMR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCMR_MSS_R5SS0_ETM_CIDCMR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDCMR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_SYNCFR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SYNCFR_MSS_R5SS0_ETM_SYNCFR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SYNCFR_MSS_R5SS0_ETM_SYNCFR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SYNCFR_MSS_R5SS0_ETM_SYNCFR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SYNCFR_MSS_R5SS0_ETM_SYNCFR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_SYNCFR_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_IDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_IDR_MSS_R5SS0_ETM_IDR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_IDR_MSS_R5SS0_ETM_IDR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_IDR_MSS_R5SS0_ETM_IDR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_IDR_MSS_R5SS0_ETM_IDR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_IDR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_B_ETM_CCER */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCER_MSS_R5SS0_ETM_CCER_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCER_MSS_R5SS0_ETM_CCER_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCER_MSS_R5SS0_ETM_CCER_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCER_MSS_R5SS0_ETM_CCER_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CCER_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_B_ETM_EXTINSELR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTINSELR_MSS_R5SS0_ETM_EXTINSELR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTINSELR_MSS_R5SS0_ETM_EXTINSELR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTINSELR_MSS_R5SS0_ETM_EXTINSELR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTINSELR_MSS_R5SS0_ETM_EXTINSELR_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_EXTINSELR_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_TRACEIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRACEIDR_MSS_R5SS0_ETM_TRACEIDR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRACEIDR_MSS_R5SS0_ETM_TRACEIDR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRACEIDR_MSS_R5SS0_ETM_TRACEIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRACEIDR_MSS_R5SS0_ETM_TRACEIDR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_TRACEIDR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_ETM_PDSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PDSR_MSS_R5SS0_ETM_PDSR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PDSR_MSS_R5SS0_ETM_PDSR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PDSR_MSS_R5SS0_ETM_PDSR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PDSR_MSS_R5SS0_ETM_PDSR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PDSR_RESETVAL                          (0x00000000U)

/* MSS_R5SS0_B_ETM_ITETMIF */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITETMIF_MSS_R5SS0_ETM_ITETMIF_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITETMIF_MSS_R5SS0_ETM_ITETMIF_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITETMIF_MSS_R5SS0_ETM_ITETMIF_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITETMIF_MSS_R5SS0_ETM_ITETMIF_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITETMIF_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_ITMISCOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCOUT_MSS_R5SS0_ETM_ITMISCOUT_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCOUT_MSS_R5SS0_ETM_ITMISCOUT_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCOUT_MSS_R5SS0_ETM_ITMISCOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCOUT_MSS_R5SS0_ETM_ITMISCOUT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCOUT_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_ITMISCIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCIN_MSS_R5SS0_ETM_ITMISCIN_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCIN_MSS_R5SS0_ETM_ITMISCIN_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCIN_MSS_R5SS0_ETM_ITMISCIN_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCIN_MSS_R5SS0_ETM_ITMISCIN_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITMISCIN_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_ETM_ITTRIGGERACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERACK_MSS_R5SS0_ETM_ITTRIGGERACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERACK_MSS_R5SS0_ETM_ITTRIGGERACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERACK_MSS_R5SS0_ETM_ITTRIGGERACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERACK_MSS_R5SS0_ETM_ITTRIGGERACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERACK_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_ETM_ITTRIGGERREQ */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERREQ_MSS_R5SS0_ETM_ITTRIGGERREQ_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERREQ_MSS_R5SS0_ETM_ITTRIGGERREQ_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERREQ_MSS_R5SS0_ETM_ITTRIGGERREQ_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERREQ_MSS_R5SS0_ETM_ITTRIGGERREQ_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITTRIGGERREQ_RESETVAL                  (0x00000000U)

/* MSS_R5SS0_B_ETM_ITATBDATA0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBDATA0_MSS_R5SS0_ETM_ITATBDATA0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBDATA0_MSS_R5SS0_ETM_ITATBDATA0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBDATA0_MSS_R5SS0_ETM_ITATBDATA0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBDATA0_MSS_R5SS0_ETM_ITATBDATA0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBDATA0_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_ITATBCTR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR2_MSS_R5SS0_ETM_ITATBCTR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR2_MSS_R5SS0_ETM_ITATBCTR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR2_MSS_R5SS0_ETM_ITATBCTR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR2_MSS_R5SS0_ETM_ITATBCTR2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR2_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_ITATBCTR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR1_MSS_R5SS0_ETM_ITATBCTR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR1_MSS_R5SS0_ETM_ITATBCTR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR1_MSS_R5SS0_ETM_ITATBCTR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR1_MSS_R5SS0_ETM_ITATBCTR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR1_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_ITATBCTR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR0_MSS_R5SS0_ETM_ITATBCTR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR0_MSS_R5SS0_ETM_ITATBCTR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR0_MSS_R5SS0_ETM_ITATBCTR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR0_MSS_R5SS0_ETM_ITATBCTR0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITATBCTR0_RESETVAL                     (0x00000000U)

/* MSS_R5SS0_B_ETM_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITCTRL_MSS_R5SS0_ETM_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITCTRL_MSS_R5SS0_ETM_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITCTRL_MSS_R5SS0_ETM_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITCTRL_MSS_R5SS0_ETM_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_ITCTRL_RESETVAL                        (0x00000000U)

/* MSS_R5SS0_B_ETM_CLAIMSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMSET_MSS_R5SS0_ETM_CLAIMSET_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMSET_MSS_R5SS0_ETM_CLAIMSET_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMSET_MSS_R5SS0_ETM_CLAIMSET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMSET_MSS_R5SS0_ETM_CLAIMSET_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMSET_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_ETM_CLAIMCLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMCLR_MSS_R5SS0_ETM_CLAIMCLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMCLR_MSS_R5SS0_ETM_CLAIMCLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMCLR_MSS_R5SS0_ETM_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMCLR_MSS_R5SS0_ETM_CLAIMCLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CLAIMCLR_RESETVAL                      (0x00000000U)

/* MSS_R5SS0_B_ETM_LAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LAR_MSS_R5SS0_ETM_LAR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LAR_MSS_R5SS0_ETM_LAR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LAR_MSS_R5SS0_ETM_LAR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LAR_MSS_R5SS0_ETM_LAR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LAR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_B_ETM_LSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LSR_MSS_R5SS0_ETM_LSR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LSR_MSS_R5SS0_ETM_LSR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LSR_MSS_R5SS0_ETM_LSR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LSR_MSS_R5SS0_ETM_LSR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_LSR_RESETVAL                           (0x00000000U)

/* MSS_R5SS0_B_ETM_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_AUTHSTATUS_MSS_R5SS0_ETM_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_AUTHSTATUS_MSS_R5SS0_ETM_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_AUTHSTATUS_MSS_R5SS0_ETM_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_AUTHSTATUS_MSS_R5SS0_ETM_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_AUTHSTATUS_RESETVAL                    (0x00000000U)

/* MSS_R5SS0_B_ETM_DEVID */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVID_MSS_R5SS0_ETM_DEVID_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVID_MSS_R5SS0_ETM_DEVID_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVID_MSS_R5SS0_ETM_DEVID_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVID_MSS_R5SS0_ETM_DEVID_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVID_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_DEVTYPE */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVTYPE_MSS_R5SS0_ETM_DEVTYPE_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVTYPE_MSS_R5SS0_ETM_DEVTYPE_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVTYPE_MSS_R5SS0_ETM_DEVTYPE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVTYPE_MSS_R5SS0_ETM_DEVTYPE_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_DEVTYPE_RESETVAL                       (0x00000000U)

/* MSS_R5SS0_B_ETM_PIDR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR4_MSS_R5SS0_ETM_PIDR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR4_MSS_R5SS0_ETM_PIDR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR4_MSS_R5SS0_ETM_PIDR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR4_MSS_R5SS0_ETM_PIDR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_PIDR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR5_MSS_R5SS0_ETM_PIDR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR5_MSS_R5SS0_ETM_PIDR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR5_MSS_R5SS0_ETM_PIDR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR5_MSS_R5SS0_ETM_PIDR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_PIDR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR6_MSS_R5SS0_ETM_PIDR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR6_MSS_R5SS0_ETM_PIDR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR6_MSS_R5SS0_ETM_PIDR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR6_MSS_R5SS0_ETM_PIDR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_PIDR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR7_MSS_R5SS0_ETM_PIDR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR7_MSS_R5SS0_ETM_PIDR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR7_MSS_R5SS0_ETM_PIDR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR7_MSS_R5SS0_ETM_PIDR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_PIDR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR0_MSS_R5SS0_ETM_PIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR0_MSS_R5SS0_ETM_PIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR0_MSS_R5SS0_ETM_PIDR0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR0_MSS_R5SS0_ETM_PIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR0_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_PIDR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR1_MSS_R5SS0_ETM_PIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR1_MSS_R5SS0_ETM_PIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR1_MSS_R5SS0_ETM_PIDR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR1_MSS_R5SS0_ETM_PIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_PIDR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR2_MSS_R5SS0_ETM_PIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR2_MSS_R5SS0_ETM_PIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR2_MSS_R5SS0_ETM_PIDR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR2_MSS_R5SS0_ETM_PIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_PIDR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR3_MSS_R5SS0_ETM_PIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR3_MSS_R5SS0_ETM_PIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR3_MSS_R5SS0_ETM_PIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR3_MSS_R5SS0_ETM_PIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_PIDR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_CIDR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR0_MSS_R5SS0_ETM_CIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR0_MSS_R5SS0_ETM_CIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR0_MSS_R5SS0_ETM_CIDR0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR0_MSS_R5SS0_ETM_CIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR0_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_CIDR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR1_MSS_R5SS0_ETM_CIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR1_MSS_R5SS0_ETM_CIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR1_MSS_R5SS0_ETM_CIDR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR1_MSS_R5SS0_ETM_CIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_CIDR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR2_MSS_R5SS0_ETM_CIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR2_MSS_R5SS0_ETM_CIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR2_MSS_R5SS0_ETM_CIDR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR2_MSS_R5SS0_ETM_CIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS0_B_ETM_CIDR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR3_MSS_R5SS0_ETM_CIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR3_MSS_R5SS0_ETM_CIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR3_MSS_R5SS0_ETM_CIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR3_MSS_R5SS0_ETM_CIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS0_B_ETM_CIDR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_ROM */


/* MSS_R5SS1_A_APB_DBGDIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDIDR_MSS_R5SS1_APB_DBGDIDR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDIDR_MSS_R5SS1_APB_DBGDIDR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDIDR_MSS_R5SS1_APB_DBGDIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDIDR_MSS_R5SS1_APB_DBGDIDR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDIDR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWFAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWFAR_MSS_R5SS1_APB_DBGWFAR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWFAR_MSS_R5SS1_APB_DBGWFAR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWFAR_MSS_R5SS1_APB_DBGWFAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWFAR_MSS_R5SS1_APB_DBGWFAR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWFAR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGVCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGVCR_MSS_R5SS1_APB_DBGVCR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGVCR_MSS_R5SS1_APB_DBGVCR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGVCR_MSS_R5SS1_APB_DBGVCR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGVCR_MSS_R5SS1_APB_DBGVCR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGVCR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_APB_DBGECR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGECR_MSS_R5SS1_APB_DBGECR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGECR_MSS_R5SS1_APB_DBGECR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGECR_MSS_R5SS1_APB_DBGECR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGECR_MSS_R5SS1_APB_DBGECR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGECR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_APB_DBGDSCCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCCR_MSS_R5SS1_APB_DBGDSCCR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCCR_MSS_R5SS1_APB_DBGDSCCR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCCR_MSS_R5SS1_APB_DBGDSCCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCCR_MSS_R5SS1_APB_DBGDSCCR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCCR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_DBGDTRRX */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRRX_MSS_R5SS1_APB_DBGDTRRX_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRRX_MSS_R5SS1_APB_DBGDTRRX_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRRX_MSS_R5SS1_APB_DBGDTRRX_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRRX_MSS_R5SS1_APB_DBGDTRRX_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRRX_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_DBGITR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGITR_MSS_R5SS1_APB_DBGITR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGITR_MSS_R5SS1_APB_DBGITR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGITR_MSS_R5SS1_APB_DBGITR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGITR_MSS_R5SS1_APB_DBGITR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGITR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_APB_DBGDSCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCR_MSS_R5SS1_APB_DBGDSCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCR_MSS_R5SS1_APB_DBGDSCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCR_MSS_R5SS1_APB_DBGDSCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCR_MSS_R5SS1_APB_DBGDSCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDSCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGDTRTX */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRTX_MSS_R5SS1_APB_DBGDTRTX_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRTX_MSS_R5SS1_APB_DBGDTRTX_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRTX_MSS_R5SS1_APB_DBGDTRTX_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRTX_MSS_R5SS1_APB_DBGDTRTX_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDTRTX_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_DBGDRCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDRCR_MSS_R5SS1_APB_DBGDRCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDRCR_MSS_R5SS1_APB_DBGDRCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDRCR_MSS_R5SS1_APB_DBGDRCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDRCR_MSS_R5SS1_APB_DBGDRCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGDRCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBVR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR0_MSS_R5SS1_APB_DBGBVR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR0_MSS_R5SS1_APB_DBGBVR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR0_MSS_R5SS1_APB_DBGBVR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR0_MSS_R5SS1_APB_DBGBVR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR1_MSS_R5SS1_APB_DBGBVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR1_MSS_R5SS1_APB_DBGBVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR1_MSS_R5SS1_APB_DBGBVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR1_MSS_R5SS1_APB_DBGBVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR2_MSS_R5SS1_APB_DBGBVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR2_MSS_R5SS1_APB_DBGBVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR2_MSS_R5SS1_APB_DBGBVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR2_MSS_R5SS1_APB_DBGBVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR3_MSS_R5SS1_APB_DBGBVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR3_MSS_R5SS1_APB_DBGBVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR3_MSS_R5SS1_APB_DBGBVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR3_MSS_R5SS1_APB_DBGBVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR4_MSS_R5SS1_APB_DBGBVR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR4_MSS_R5SS1_APB_DBGBVR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR4_MSS_R5SS1_APB_DBGBVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR4_MSS_R5SS1_APB_DBGBVR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR5_MSS_R5SS1_APB_DBGBVR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR5_MSS_R5SS1_APB_DBGBVR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR5_MSS_R5SS1_APB_DBGBVR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR5_MSS_R5SS1_APB_DBGBVR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR6_MSS_R5SS1_APB_DBGBVR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR6_MSS_R5SS1_APB_DBGBVR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR6_MSS_R5SS1_APB_DBGBVR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR6_MSS_R5SS1_APB_DBGBVR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR7_MSS_R5SS1_APB_DBGBVR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR7_MSS_R5SS1_APB_DBGBVR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR7_MSS_R5SS1_APB_DBGBVR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR7_MSS_R5SS1_APB_DBGBVR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBVR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBCR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR0_MSS_R5SS1_APB_DBGBCR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR0_MSS_R5SS1_APB_DBGBCR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR0_MSS_R5SS1_APB_DBGBCR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR0_MSS_R5SS1_APB_DBGBCR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR1_MSS_R5SS1_APB_DBGBCR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR1_MSS_R5SS1_APB_DBGBCR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR1_MSS_R5SS1_APB_DBGBCR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR1_MSS_R5SS1_APB_DBGBCR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR2_MSS_R5SS1_APB_DBGBCR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR2_MSS_R5SS1_APB_DBGBCR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR2_MSS_R5SS1_APB_DBGBCR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR2_MSS_R5SS1_APB_DBGBCR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR3_MSS_R5SS1_APB_DBGBCR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR3_MSS_R5SS1_APB_DBGBCR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR3_MSS_R5SS1_APB_DBGBCR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR3_MSS_R5SS1_APB_DBGBCR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBCR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR4_MSS_R5SS1_APB_DBGBCR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR4_MSS_R5SS1_APB_DBGBCR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR4_MSS_R5SS1_APB_DBGBCR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR4_MSS_R5SS1_APB_DBGBCR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBCR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR5_MSS_R5SS1_APB_DBGBCR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR5_MSS_R5SS1_APB_DBGBCR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR5_MSS_R5SS1_APB_DBGBCR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR5_MSS_R5SS1_APB_DBGBCR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBCR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR6_MSS_R5SS1_APB_DBGBCR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR6_MSS_R5SS1_APB_DBGBCR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR6_MSS_R5SS1_APB_DBGBCR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR6_MSS_R5SS1_APB_DBGBCR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGBCR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR7_MSS_R5SS1_APB_DBGBCR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR7_MSS_R5SS1_APB_DBGBCR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR7_MSS_R5SS1_APB_DBGBCR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR7_MSS_R5SS1_APB_DBGBCR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGBCR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWVR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR0_MSS_R5SS1_APB_DBGWVR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR0_MSS_R5SS1_APB_DBGWVR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR0_MSS_R5SS1_APB_DBGWVR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR0_MSS_R5SS1_APB_DBGWVR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR1_MSS_R5SS1_APB_DBGWVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR1_MSS_R5SS1_APB_DBGWVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR1_MSS_R5SS1_APB_DBGWVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR1_MSS_R5SS1_APB_DBGWVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR2_MSS_R5SS1_APB_DBGWVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR2_MSS_R5SS1_APB_DBGWVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR2_MSS_R5SS1_APB_DBGWVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR2_MSS_R5SS1_APB_DBGWVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR3_MSS_R5SS1_APB_DBGWVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR3_MSS_R5SS1_APB_DBGWVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR3_MSS_R5SS1_APB_DBGWVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR3_MSS_R5SS1_APB_DBGWVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR4_MSS_R5SS1_APB_DBGWVR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR4_MSS_R5SS1_APB_DBGWVR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR4_MSS_R5SS1_APB_DBGWVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR4_MSS_R5SS1_APB_DBGWVR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR5_MSS_R5SS1_APB_DBGWVR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR5_MSS_R5SS1_APB_DBGWVR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR5_MSS_R5SS1_APB_DBGWVR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR5_MSS_R5SS1_APB_DBGWVR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR6_MSS_R5SS1_APB_DBGWVR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR6_MSS_R5SS1_APB_DBGWVR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR6_MSS_R5SS1_APB_DBGWVR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR6_MSS_R5SS1_APB_DBGWVR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR7_MSS_R5SS1_APB_DBGWVR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR7_MSS_R5SS1_APB_DBGWVR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR7_MSS_R5SS1_APB_DBGWVR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR7_MSS_R5SS1_APB_DBGWVR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWVR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWCR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR0_MSS_R5SS1_APB_DBGWCR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR0_MSS_R5SS1_APB_DBGWCR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR0_MSS_R5SS1_APB_DBGWCR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR0_MSS_R5SS1_APB_DBGWCR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR1_MSS_R5SS1_APB_DBGWCR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR1_MSS_R5SS1_APB_DBGWCR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR1_MSS_R5SS1_APB_DBGWCR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR1_MSS_R5SS1_APB_DBGWCR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR2_MSS_R5SS1_APB_DBGWCR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR2_MSS_R5SS1_APB_DBGWCR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR2_MSS_R5SS1_APB_DBGWCR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR2_MSS_R5SS1_APB_DBGWCR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR3_MSS_R5SS1_APB_DBGWCR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR3_MSS_R5SS1_APB_DBGWCR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR3_MSS_R5SS1_APB_DBGWCR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR3_MSS_R5SS1_APB_DBGWCR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWCR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR4_MSS_R5SS1_APB_DBGWCR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR4_MSS_R5SS1_APB_DBGWCR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR4_MSS_R5SS1_APB_DBGWCR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR4_MSS_R5SS1_APB_DBGWCR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWCR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR5_MSS_R5SS1_APB_DBGWCR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR5_MSS_R5SS1_APB_DBGWCR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR5_MSS_R5SS1_APB_DBGWCR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR5_MSS_R5SS1_APB_DBGWCR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWCR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR6_MSS_R5SS1_APB_DBGWCR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR6_MSS_R5SS1_APB_DBGWCR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR6_MSS_R5SS1_APB_DBGWCR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR6_MSS_R5SS1_APB_DBGWCR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGWCR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR7_MSS_R5SS1_APB_DBGWCR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR7_MSS_R5SS1_APB_DBGWCR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR7_MSS_R5SS1_APB_DBGWCR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR7_MSS_R5SS1_APB_DBGWCR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGWCR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGOSLAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLAR_MSS_R5SS1_APB_DBGOSLAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLAR_MSS_R5SS1_APB_DBGOSLAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLAR_MSS_R5SS1_APB_DBGOSLAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLAR_MSS_R5SS1_APB_DBGOSLAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLAR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_DBGOSLSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLSR_MSS_R5SS1_APB_DBGOSLSR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLSR_MSS_R5SS1_APB_DBGOSLSR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLSR_MSS_R5SS1_APB_DBGOSLSR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLSR_MSS_R5SS1_APB_DBGOSLSR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSLSR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_DBGOSSRR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSSRR_MSS_R5SS1_APB_DBGOSSRR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSSRR_MSS_R5SS1_APB_DBGOSSRR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSSRR_MSS_R5SS1_APB_DBGOSSRR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSSRR_MSS_R5SS1_APB_DBGOSSRR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGOSSRR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_DBGPRCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRCR_MSS_R5SS1_APB_DBGPRCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRCR_MSS_R5SS1_APB_DBGPRCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRCR_MSS_R5SS1_APB_DBGPRCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRCR_MSS_R5SS1_APB_DBGPRCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_DBGPRSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRSR_MSS_R5SS1_APB_DBGPRSR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRSR_MSS_R5SS1_APB_DBGPRSR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRSR_MSS_R5SS1_APB_DBGPRSR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRSR_MSS_R5SS1_APB_DBGPRSR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_DBGPRSR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_MIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MIDR_MSS_R5SS1_APB_PROCID_MIDR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MIDR_MSS_R5SS1_APB_PROCID_MIDR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MIDR_MSS_R5SS1_APB_PROCID_MIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MIDR_MSS_R5SS1_APB_PROCID_MIDR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MIDR_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_CTR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_CTR_MSS_R5SS1_APB_PROCID_CTR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_CTR_MSS_R5SS1_APB_PROCID_CTR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_CTR_MSS_R5SS1_APB_PROCID_CTR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_CTR_MSS_R5SS1_APB_PROCID_CTR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_CTR_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_TCMTR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_TCMTR_MSS_R5SS1_APB_PROCID_TCMTR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_TCMTR_MSS_R5SS1_APB_PROCID_TCMTR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_TCMTR_MSS_R5SS1_APB_PROCID_TCMTR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_TCMTR_MSS_R5SS1_APB_PROCID_TCMTR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_TCMTR_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_MPUIR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPUIR_MSS_R5SS1_APB_PROCID_MPUIR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPUIR_MSS_R5SS1_APB_PROCID_MPUIR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPUIR_MSS_R5SS1_APB_PROCID_MPUIR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPUIR_MSS_R5SS1_APB_PROCID_MPUIR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPUIR_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_MPIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPIDR_MSS_R5SS1_APB_PROCID_MPIDR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPIDR_MSS_R5SS1_APB_PROCID_MPIDR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPIDR_MSS_R5SS1_APB_PROCID_MPIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPIDR_MSS_R5SS1_APB_PROCID_MPIDR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MPIDR_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_PFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR0_MSS_R5SS1_APB_PROCID_PFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR0_MSS_R5SS1_APB_PROCID_PFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR0_MSS_R5SS1_APB_PROCID_PFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR0_MSS_R5SS1_APB_PROCID_PFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_PFR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR1_MSS_R5SS1_APB_PROCID_PFR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR1_MSS_R5SS1_APB_PROCID_PFR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR1_MSS_R5SS1_APB_PROCID_PFR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR1_MSS_R5SS1_APB_PROCID_PFR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_PFR1_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_DFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_DFR0_MSS_R5SS1_APB_PROCID_DFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_DFR0_MSS_R5SS1_APB_PROCID_DFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_DFR0_MSS_R5SS1_APB_PROCID_DFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_DFR0_MSS_R5SS1_APB_PROCID_DFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_DFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_AFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_AFR0_MSS_R5SS1_APB_PROCID_AFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_AFR0_MSS_R5SS1_APB_PROCID_AFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_AFR0_MSS_R5SS1_APB_PROCID_AFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_AFR0_MSS_R5SS1_APB_PROCID_AFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_AFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_MMFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR0_MSS_R5SS1_APB_PROCID_MMFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR0_MSS_R5SS1_APB_PROCID_MMFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR0_MSS_R5SS1_APB_PROCID_MMFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR0_MSS_R5SS1_APB_PROCID_MMFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR0_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_MMFR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR1_MSS_R5SS1_APB_PROCID_MMFR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR1_MSS_R5SS1_APB_PROCID_MMFR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR1_MSS_R5SS1_APB_PROCID_MMFR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR1_MSS_R5SS1_APB_PROCID_MMFR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR1_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_MMFR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR2_MSS_R5SS1_APB_PROCID_MMFR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR2_MSS_R5SS1_APB_PROCID_MMFR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR2_MSS_R5SS1_APB_PROCID_MMFR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR2_MSS_R5SS1_APB_PROCID_MMFR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR2_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_MMFR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR3_MSS_R5SS1_APB_PROCID_MMFR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR3_MSS_R5SS1_APB_PROCID_MMFR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR3_MSS_R5SS1_APB_PROCID_MMFR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR3_MSS_R5SS1_APB_PROCID_MMFR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_MMFR3_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_ISAR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR0_MSS_R5SS1_APB_PROCID_ISAR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR0_MSS_R5SS1_APB_PROCID_ISAR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR0_MSS_R5SS1_APB_PROCID_ISAR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR0_MSS_R5SS1_APB_PROCID_ISAR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR0_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_ISAR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR1_MSS_R5SS1_APB_PROCID_ISAR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR1_MSS_R5SS1_APB_PROCID_ISAR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR1_MSS_R5SS1_APB_PROCID_ISAR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR1_MSS_R5SS1_APB_PROCID_ISAR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR1_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_ISAR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR2_MSS_R5SS1_APB_PROCID_ISAR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR2_MSS_R5SS1_APB_PROCID_ISAR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR2_MSS_R5SS1_APB_PROCID_ISAR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR2_MSS_R5SS1_APB_PROCID_ISAR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR2_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_ISAR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR3_MSS_R5SS1_APB_PROCID_ISAR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR3_MSS_R5SS1_APB_PROCID_ISAR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR3_MSS_R5SS1_APB_PROCID_ISAR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR3_MSS_R5SS1_APB_PROCID_ISAR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR3_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_ISAR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR4_MSS_R5SS1_APB_PROCID_ISAR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR4_MSS_R5SS1_APB_PROCID_ISAR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR4_MSS_R5SS1_APB_PROCID_ISAR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR4_MSS_R5SS1_APB_PROCID_ISAR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR4_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_PROCID_ISAR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR5_MSS_R5SS1_APB_PROCID_ISAR5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR5_MSS_R5SS1_APB_PROCID_ISAR5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR5_MSS_R5SS1_APB_PROCID_ISAR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR5_MSS_R5SS1_APB_PROCID_ISAR5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PROCID_ISAR5_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_APB_MR_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_ITCTRL_MSS_R5SS1_APB_MR_ITCTRL_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_ITCTRL_MSS_R5SS1_APB_MR_ITCTRL_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_ITCTRL_MSS_R5SS1_APB_MR_ITCTRL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_ITCTRL_MSS_R5SS1_APB_MR_ITCTRL_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_ITCTRL_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_APB_MR_CLAIMSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMSET_MSS_R5SS1_APB_MR_CLAIMSET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMSET_MSS_R5SS1_APB_MR_CLAIMSET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMSET_MSS_R5SS1_APB_MR_CLAIMSET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMSET_MSS_R5SS1_APB_MR_CLAIMSET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMSET_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_APB_MR_CLAIMCLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMCLR_MSS_R5SS1_APB_MR_CLAIMCLR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMCLR_MSS_R5SS1_APB_MR_CLAIMCLR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMCLR_MSS_R5SS1_APB_MR_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMCLR_MSS_R5SS1_APB_MR_CLAIMCLR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_CLAIMCLR_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_APB_MR_LOCKACCESS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKACCESS_MSS_R5SS1_APB_MR_LOCKACCESS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKACCESS_MSS_R5SS1_APB_MR_LOCKACCESS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKACCESS_MSS_R5SS1_APB_MR_LOCKACCESS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKACCESS_MSS_R5SS1_APB_MR_LOCKACCESS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKACCESS_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_APB_MR_LOCKSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKSTATUS_MSS_R5SS1_APB_MR_LOCKSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKSTATUS_MSS_R5SS1_APB_MR_LOCKSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKSTATUS_MSS_R5SS1_APB_MR_LOCKSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKSTATUS_MSS_R5SS1_APB_MR_LOCKSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_LOCKSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_APB_MR_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_AUTHSTATUS_MSS_R5SS1_APB_MR_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_AUTHSTATUS_MSS_R5SS1_APB_MR_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_AUTHSTATUS_MSS_R5SS1_APB_MR_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_AUTHSTATUS_MSS_R5SS1_APB_MR_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_AUTHSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_APB_MR_DEVID */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVID_MSS_R5SS1_APB_MR_DEVID_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVID_MSS_R5SS1_APB_MR_DEVID_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVID_MSS_R5SS1_APB_MR_DEVID_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVID_MSS_R5SS1_APB_MR_DEVID_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVID_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_MR_DEVTYPE */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVTYPE_MSS_R5SS1_APB_MR_DEVTYPE_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVTYPE_MSS_R5SS1_APB_MR_DEVTYPE_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVTYPE_MSS_R5SS1_APB_MR_DEVTYPE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVTYPE_MSS_R5SS1_APB_MR_DEVTYPE_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_MR_DEVTYPE_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_APB_PERIP_ID4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID4_MSS_R5SS1_APB_PERIP_ID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID4_MSS_R5SS1_APB_PERIP_ID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID4_MSS_R5SS1_APB_PERIP_ID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID4_MSS_R5SS1_APB_PERIP_ID4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID4_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_APB_PERIP_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID0_MSS_R5SS1_APB_PERIP_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID0_MSS_R5SS1_APB_PERIP_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID0_MSS_R5SS1_APB_PERIP_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID0_MSS_R5SS1_APB_PERIP_ID0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID0_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_APB_PERIP_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID1_MSS_R5SS1_APB_PERIP_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID1_MSS_R5SS1_APB_PERIP_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID1_MSS_R5SS1_APB_PERIP_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID1_MSS_R5SS1_APB_PERIP_ID1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID1_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_APB_PERIP_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID2_MSS_R5SS1_APB_PERIP_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID2_MSS_R5SS1_APB_PERIP_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID2_MSS_R5SS1_APB_PERIP_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID2_MSS_R5SS1_APB_PERIP_ID2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID2_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_APB_PERIP_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID3_MSS_R5SS1_APB_PERIP_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID3_MSS_R5SS1_APB_PERIP_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID3_MSS_R5SS1_APB_PERIP_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID3_MSS_R5SS1_APB_PERIP_ID3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_PERIP_ID3_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_APB_COMP_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID0_MSS_R5SS1_APB_COMP_ID0_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID0_MSS_R5SS1_APB_COMP_ID0_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID0_MSS_R5SS1_APB_COMP_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID0_MSS_R5SS1_APB_COMP_ID0_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID0_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_COMP_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID1_MSS_R5SS1_APB_COMP_ID1_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID1_MSS_R5SS1_APB_COMP_ID1_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID1_MSS_R5SS1_APB_COMP_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID1_MSS_R5SS1_APB_COMP_ID1_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID1_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_COMP_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID2_MSS_R5SS1_APB_COMP_ID2_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID2_MSS_R5SS1_APB_COMP_ID2_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID2_MSS_R5SS1_APB_COMP_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID2_MSS_R5SS1_APB_COMP_ID2_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID2_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_APB_COMP_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID3_MSS_R5SS1_APB_COMP_ID3_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID3_MSS_R5SS1_APB_COMP_ID3_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID3_MSS_R5SS1_APB_COMP_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID3_MSS_R5SS1_APB_COMP_ID3_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_APB_COMP_ID3_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_DBGDIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDIDR_MSS_R5SS1_APB_DBGDIDR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDIDR_MSS_R5SS1_APB_DBGDIDR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDIDR_MSS_R5SS1_APB_DBGDIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDIDR_MSS_R5SS1_APB_DBGDIDR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDIDR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWFAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWFAR_MSS_R5SS1_APB_DBGWFAR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWFAR_MSS_R5SS1_APB_DBGWFAR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWFAR_MSS_R5SS1_APB_DBGWFAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWFAR_MSS_R5SS1_APB_DBGWFAR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWFAR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGVCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGVCR_MSS_R5SS1_APB_DBGVCR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGVCR_MSS_R5SS1_APB_DBGVCR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGVCR_MSS_R5SS1_APB_DBGVCR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGVCR_MSS_R5SS1_APB_DBGVCR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGVCR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_APB_DBGECR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGECR_MSS_R5SS1_APB_DBGECR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGECR_MSS_R5SS1_APB_DBGECR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGECR_MSS_R5SS1_APB_DBGECR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGECR_MSS_R5SS1_APB_DBGECR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGECR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_APB_DBGDSCCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCCR_MSS_R5SS1_APB_DBGDSCCR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCCR_MSS_R5SS1_APB_DBGDSCCR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCCR_MSS_R5SS1_APB_DBGDSCCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCCR_MSS_R5SS1_APB_DBGDSCCR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCCR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_DBGDTRRX */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRRX_MSS_R5SS1_APB_DBGDTRRX_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRRX_MSS_R5SS1_APB_DBGDTRRX_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRRX_MSS_R5SS1_APB_DBGDTRRX_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRRX_MSS_R5SS1_APB_DBGDTRRX_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRRX_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_DBGITR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGITR_MSS_R5SS1_APB_DBGITR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGITR_MSS_R5SS1_APB_DBGITR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGITR_MSS_R5SS1_APB_DBGITR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGITR_MSS_R5SS1_APB_DBGITR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGITR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_APB_DBGDSCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCR_MSS_R5SS1_APB_DBGDSCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCR_MSS_R5SS1_APB_DBGDSCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCR_MSS_R5SS1_APB_DBGDSCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCR_MSS_R5SS1_APB_DBGDSCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDSCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGDTRTX */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRTX_MSS_R5SS1_APB_DBGDTRTX_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRTX_MSS_R5SS1_APB_DBGDTRTX_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRTX_MSS_R5SS1_APB_DBGDTRTX_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRTX_MSS_R5SS1_APB_DBGDTRTX_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDTRTX_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_DBGDRCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDRCR_MSS_R5SS1_APB_DBGDRCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDRCR_MSS_R5SS1_APB_DBGDRCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDRCR_MSS_R5SS1_APB_DBGDRCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDRCR_MSS_R5SS1_APB_DBGDRCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGDRCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBVR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR0_MSS_R5SS1_APB_DBGBVR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR0_MSS_R5SS1_APB_DBGBVR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR0_MSS_R5SS1_APB_DBGBVR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR0_MSS_R5SS1_APB_DBGBVR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR1_MSS_R5SS1_APB_DBGBVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR1_MSS_R5SS1_APB_DBGBVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR1_MSS_R5SS1_APB_DBGBVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR1_MSS_R5SS1_APB_DBGBVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR2_MSS_R5SS1_APB_DBGBVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR2_MSS_R5SS1_APB_DBGBVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR2_MSS_R5SS1_APB_DBGBVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR2_MSS_R5SS1_APB_DBGBVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR3_MSS_R5SS1_APB_DBGBVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR3_MSS_R5SS1_APB_DBGBVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR3_MSS_R5SS1_APB_DBGBVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR3_MSS_R5SS1_APB_DBGBVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR4_MSS_R5SS1_APB_DBGBVR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR4_MSS_R5SS1_APB_DBGBVR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR4_MSS_R5SS1_APB_DBGBVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR4_MSS_R5SS1_APB_DBGBVR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR5_MSS_R5SS1_APB_DBGBVR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR5_MSS_R5SS1_APB_DBGBVR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR5_MSS_R5SS1_APB_DBGBVR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR5_MSS_R5SS1_APB_DBGBVR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR6_MSS_R5SS1_APB_DBGBVR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR6_MSS_R5SS1_APB_DBGBVR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR6_MSS_R5SS1_APB_DBGBVR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR6_MSS_R5SS1_APB_DBGBVR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR7_MSS_R5SS1_APB_DBGBVR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR7_MSS_R5SS1_APB_DBGBVR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR7_MSS_R5SS1_APB_DBGBVR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR7_MSS_R5SS1_APB_DBGBVR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBVR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBCR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR0_MSS_R5SS1_APB_DBGBCR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR0_MSS_R5SS1_APB_DBGBCR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR0_MSS_R5SS1_APB_DBGBCR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR0_MSS_R5SS1_APB_DBGBCR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR1_MSS_R5SS1_APB_DBGBCR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR1_MSS_R5SS1_APB_DBGBCR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR1_MSS_R5SS1_APB_DBGBCR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR1_MSS_R5SS1_APB_DBGBCR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR2_MSS_R5SS1_APB_DBGBCR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR2_MSS_R5SS1_APB_DBGBCR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR2_MSS_R5SS1_APB_DBGBCR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR2_MSS_R5SS1_APB_DBGBCR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR3_MSS_R5SS1_APB_DBGBCR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR3_MSS_R5SS1_APB_DBGBCR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR3_MSS_R5SS1_APB_DBGBCR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR3_MSS_R5SS1_APB_DBGBCR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBCR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR4_MSS_R5SS1_APB_DBGBCR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR4_MSS_R5SS1_APB_DBGBCR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR4_MSS_R5SS1_APB_DBGBCR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR4_MSS_R5SS1_APB_DBGBCR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBCR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR5_MSS_R5SS1_APB_DBGBCR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR5_MSS_R5SS1_APB_DBGBCR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR5_MSS_R5SS1_APB_DBGBCR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR5_MSS_R5SS1_APB_DBGBCR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBCR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR6_MSS_R5SS1_APB_DBGBCR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR6_MSS_R5SS1_APB_DBGBCR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR6_MSS_R5SS1_APB_DBGBCR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR6_MSS_R5SS1_APB_DBGBCR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGBCR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR7_MSS_R5SS1_APB_DBGBCR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR7_MSS_R5SS1_APB_DBGBCR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR7_MSS_R5SS1_APB_DBGBCR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR7_MSS_R5SS1_APB_DBGBCR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGBCR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWVR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR0_MSS_R5SS1_APB_DBGWVR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR0_MSS_R5SS1_APB_DBGWVR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR0_MSS_R5SS1_APB_DBGWVR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR0_MSS_R5SS1_APB_DBGWVR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR1_MSS_R5SS1_APB_DBGWVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR1_MSS_R5SS1_APB_DBGWVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR1_MSS_R5SS1_APB_DBGWVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR1_MSS_R5SS1_APB_DBGWVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR2_MSS_R5SS1_APB_DBGWVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR2_MSS_R5SS1_APB_DBGWVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR2_MSS_R5SS1_APB_DBGWVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR2_MSS_R5SS1_APB_DBGWVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR3_MSS_R5SS1_APB_DBGWVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR3_MSS_R5SS1_APB_DBGWVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR3_MSS_R5SS1_APB_DBGWVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR3_MSS_R5SS1_APB_DBGWVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR4_MSS_R5SS1_APB_DBGWVR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR4_MSS_R5SS1_APB_DBGWVR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR4_MSS_R5SS1_APB_DBGWVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR4_MSS_R5SS1_APB_DBGWVR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR5_MSS_R5SS1_APB_DBGWVR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR5_MSS_R5SS1_APB_DBGWVR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR5_MSS_R5SS1_APB_DBGWVR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR5_MSS_R5SS1_APB_DBGWVR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR6_MSS_R5SS1_APB_DBGWVR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR6_MSS_R5SS1_APB_DBGWVR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR6_MSS_R5SS1_APB_DBGWVR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR6_MSS_R5SS1_APB_DBGWVR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR7_MSS_R5SS1_APB_DBGWVR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR7_MSS_R5SS1_APB_DBGWVR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR7_MSS_R5SS1_APB_DBGWVR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR7_MSS_R5SS1_APB_DBGWVR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWVR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWCR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR0_MSS_R5SS1_APB_DBGWCR0_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR0_MSS_R5SS1_APB_DBGWCR0_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR0_MSS_R5SS1_APB_DBGWCR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR0_MSS_R5SS1_APB_DBGWCR0_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR0_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR1_MSS_R5SS1_APB_DBGWCR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR1_MSS_R5SS1_APB_DBGWCR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR1_MSS_R5SS1_APB_DBGWCR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR1_MSS_R5SS1_APB_DBGWCR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR2_MSS_R5SS1_APB_DBGWCR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR2_MSS_R5SS1_APB_DBGWCR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR2_MSS_R5SS1_APB_DBGWCR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR2_MSS_R5SS1_APB_DBGWCR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR3_MSS_R5SS1_APB_DBGWCR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR3_MSS_R5SS1_APB_DBGWCR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR3_MSS_R5SS1_APB_DBGWCR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR3_MSS_R5SS1_APB_DBGWCR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWCR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR4_MSS_R5SS1_APB_DBGWCR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR4_MSS_R5SS1_APB_DBGWCR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR4_MSS_R5SS1_APB_DBGWCR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR4_MSS_R5SS1_APB_DBGWCR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWCR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR5_MSS_R5SS1_APB_DBGWCR5_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR5_MSS_R5SS1_APB_DBGWCR5_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR5_MSS_R5SS1_APB_DBGWCR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR5_MSS_R5SS1_APB_DBGWCR5_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR5_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWCR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR6_MSS_R5SS1_APB_DBGWCR6_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR6_MSS_R5SS1_APB_DBGWCR6_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR6_MSS_R5SS1_APB_DBGWCR6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR6_MSS_R5SS1_APB_DBGWCR6_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR6_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGWCR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR7_MSS_R5SS1_APB_DBGWCR7_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR7_MSS_R5SS1_APB_DBGWCR7_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR7_MSS_R5SS1_APB_DBGWCR7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR7_MSS_R5SS1_APB_DBGWCR7_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGWCR7_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGOSLAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLAR_MSS_R5SS1_APB_DBGOSLAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLAR_MSS_R5SS1_APB_DBGOSLAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLAR_MSS_R5SS1_APB_DBGOSLAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLAR_MSS_R5SS1_APB_DBGOSLAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLAR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_DBGOSLSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLSR_MSS_R5SS1_APB_DBGOSLSR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLSR_MSS_R5SS1_APB_DBGOSLSR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLSR_MSS_R5SS1_APB_DBGOSLSR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLSR_MSS_R5SS1_APB_DBGOSLSR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSLSR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_DBGOSSRR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSSRR_MSS_R5SS1_APB_DBGOSSRR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSSRR_MSS_R5SS1_APB_DBGOSSRR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSSRR_MSS_R5SS1_APB_DBGOSSRR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSSRR_MSS_R5SS1_APB_DBGOSSRR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGOSSRR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_DBGPRCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRCR_MSS_R5SS1_APB_DBGPRCR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRCR_MSS_R5SS1_APB_DBGPRCR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRCR_MSS_R5SS1_APB_DBGPRCR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRCR_MSS_R5SS1_APB_DBGPRCR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRCR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_DBGPRSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRSR_MSS_R5SS1_APB_DBGPRSR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRSR_MSS_R5SS1_APB_DBGPRSR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRSR_MSS_R5SS1_APB_DBGPRSR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRSR_MSS_R5SS1_APB_DBGPRSR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_DBGPRSR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_MIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MIDR_MSS_R5SS1_APB_PROCID_MIDR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MIDR_MSS_R5SS1_APB_PROCID_MIDR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MIDR_MSS_R5SS1_APB_PROCID_MIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MIDR_MSS_R5SS1_APB_PROCID_MIDR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MIDR_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_CTR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_CTR_MSS_R5SS1_APB_PROCID_CTR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_CTR_MSS_R5SS1_APB_PROCID_CTR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_CTR_MSS_R5SS1_APB_PROCID_CTR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_CTR_MSS_R5SS1_APB_PROCID_CTR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_CTR_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_TCMTR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_TCMTR_MSS_R5SS1_APB_PROCID_TCMTR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_TCMTR_MSS_R5SS1_APB_PROCID_TCMTR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_TCMTR_MSS_R5SS1_APB_PROCID_TCMTR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_TCMTR_MSS_R5SS1_APB_PROCID_TCMTR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_TCMTR_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_MPUIR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPUIR_MSS_R5SS1_APB_PROCID_MPUIR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPUIR_MSS_R5SS1_APB_PROCID_MPUIR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPUIR_MSS_R5SS1_APB_PROCID_MPUIR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPUIR_MSS_R5SS1_APB_PROCID_MPUIR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPUIR_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_MPIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPIDR_MSS_R5SS1_APB_PROCID_MPIDR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPIDR_MSS_R5SS1_APB_PROCID_MPIDR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPIDR_MSS_R5SS1_APB_PROCID_MPIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPIDR_MSS_R5SS1_APB_PROCID_MPIDR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MPIDR_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_PFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR0_MSS_R5SS1_APB_PROCID_PFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR0_MSS_R5SS1_APB_PROCID_PFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR0_MSS_R5SS1_APB_PROCID_PFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR0_MSS_R5SS1_APB_PROCID_PFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_PFR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR1_MSS_R5SS1_APB_PROCID_PFR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR1_MSS_R5SS1_APB_PROCID_PFR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR1_MSS_R5SS1_APB_PROCID_PFR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR1_MSS_R5SS1_APB_PROCID_PFR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_PFR1_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_DFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_DFR0_MSS_R5SS1_APB_PROCID_DFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_DFR0_MSS_R5SS1_APB_PROCID_DFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_DFR0_MSS_R5SS1_APB_PROCID_DFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_DFR0_MSS_R5SS1_APB_PROCID_DFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_DFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_AFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_AFR0_MSS_R5SS1_APB_PROCID_AFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_AFR0_MSS_R5SS1_APB_PROCID_AFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_AFR0_MSS_R5SS1_APB_PROCID_AFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_AFR0_MSS_R5SS1_APB_PROCID_AFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_AFR0_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_MMFR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR0_MSS_R5SS1_APB_PROCID_MMFR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR0_MSS_R5SS1_APB_PROCID_MMFR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR0_MSS_R5SS1_APB_PROCID_MMFR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR0_MSS_R5SS1_APB_PROCID_MMFR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR0_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_MMFR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR1_MSS_R5SS1_APB_PROCID_MMFR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR1_MSS_R5SS1_APB_PROCID_MMFR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR1_MSS_R5SS1_APB_PROCID_MMFR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR1_MSS_R5SS1_APB_PROCID_MMFR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR1_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_MMFR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR2_MSS_R5SS1_APB_PROCID_MMFR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR2_MSS_R5SS1_APB_PROCID_MMFR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR2_MSS_R5SS1_APB_PROCID_MMFR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR2_MSS_R5SS1_APB_PROCID_MMFR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR2_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_MMFR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR3_MSS_R5SS1_APB_PROCID_MMFR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR3_MSS_R5SS1_APB_PROCID_MMFR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR3_MSS_R5SS1_APB_PROCID_MMFR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR3_MSS_R5SS1_APB_PROCID_MMFR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_MMFR3_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_ISAR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR0_MSS_R5SS1_APB_PROCID_ISAR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR0_MSS_R5SS1_APB_PROCID_ISAR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR0_MSS_R5SS1_APB_PROCID_ISAR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR0_MSS_R5SS1_APB_PROCID_ISAR0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR0_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_ISAR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR1_MSS_R5SS1_APB_PROCID_ISAR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR1_MSS_R5SS1_APB_PROCID_ISAR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR1_MSS_R5SS1_APB_PROCID_ISAR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR1_MSS_R5SS1_APB_PROCID_ISAR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR1_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_ISAR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR2_MSS_R5SS1_APB_PROCID_ISAR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR2_MSS_R5SS1_APB_PROCID_ISAR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR2_MSS_R5SS1_APB_PROCID_ISAR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR2_MSS_R5SS1_APB_PROCID_ISAR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR2_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_ISAR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR3_MSS_R5SS1_APB_PROCID_ISAR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR3_MSS_R5SS1_APB_PROCID_ISAR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR3_MSS_R5SS1_APB_PROCID_ISAR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR3_MSS_R5SS1_APB_PROCID_ISAR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR3_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_ISAR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR4_MSS_R5SS1_APB_PROCID_ISAR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR4_MSS_R5SS1_APB_PROCID_ISAR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR4_MSS_R5SS1_APB_PROCID_ISAR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR4_MSS_R5SS1_APB_PROCID_ISAR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR4_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_PROCID_ISAR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR5_MSS_R5SS1_APB_PROCID_ISAR5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR5_MSS_R5SS1_APB_PROCID_ISAR5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR5_MSS_R5SS1_APB_PROCID_ISAR5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR5_MSS_R5SS1_APB_PROCID_ISAR5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PROCID_ISAR5_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_APB_MR_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_ITCTRL_MSS_R5SS1_APB_MR_ITCTRL_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_ITCTRL_MSS_R5SS1_APB_MR_ITCTRL_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_ITCTRL_MSS_R5SS1_APB_MR_ITCTRL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_ITCTRL_MSS_R5SS1_APB_MR_ITCTRL_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_ITCTRL_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_APB_MR_CLAIMSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMSET_MSS_R5SS1_APB_MR_CLAIMSET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMSET_MSS_R5SS1_APB_MR_CLAIMSET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMSET_MSS_R5SS1_APB_MR_CLAIMSET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMSET_MSS_R5SS1_APB_MR_CLAIMSET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMSET_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_APB_MR_CLAIMCLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMCLR_MSS_R5SS1_APB_MR_CLAIMCLR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMCLR_MSS_R5SS1_APB_MR_CLAIMCLR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMCLR_MSS_R5SS1_APB_MR_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMCLR_MSS_R5SS1_APB_MR_CLAIMCLR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_CLAIMCLR_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_APB_MR_LOCKACCESS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKACCESS_MSS_R5SS1_APB_MR_LOCKACCESS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKACCESS_MSS_R5SS1_APB_MR_LOCKACCESS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKACCESS_MSS_R5SS1_APB_MR_LOCKACCESS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKACCESS_MSS_R5SS1_APB_MR_LOCKACCESS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKACCESS_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_APB_MR_LOCKSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKSTATUS_MSS_R5SS1_APB_MR_LOCKSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKSTATUS_MSS_R5SS1_APB_MR_LOCKSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKSTATUS_MSS_R5SS1_APB_MR_LOCKSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKSTATUS_MSS_R5SS1_APB_MR_LOCKSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_LOCKSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_APB_MR_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_AUTHSTATUS_MSS_R5SS1_APB_MR_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_AUTHSTATUS_MSS_R5SS1_APB_MR_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_AUTHSTATUS_MSS_R5SS1_APB_MR_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_AUTHSTATUS_MSS_R5SS1_APB_MR_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_AUTHSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_APB_MR_DEVID */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVID_MSS_R5SS1_APB_MR_DEVID_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVID_MSS_R5SS1_APB_MR_DEVID_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVID_MSS_R5SS1_APB_MR_DEVID_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVID_MSS_R5SS1_APB_MR_DEVID_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVID_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_MR_DEVTYPE */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVTYPE_MSS_R5SS1_APB_MR_DEVTYPE_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVTYPE_MSS_R5SS1_APB_MR_DEVTYPE_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVTYPE_MSS_R5SS1_APB_MR_DEVTYPE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVTYPE_MSS_R5SS1_APB_MR_DEVTYPE_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_MR_DEVTYPE_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_APB_PERIP_ID4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID4_MSS_R5SS1_APB_PERIP_ID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID4_MSS_R5SS1_APB_PERIP_ID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID4_MSS_R5SS1_APB_PERIP_ID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID4_MSS_R5SS1_APB_PERIP_ID4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID4_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_APB_PERIP_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID0_MSS_R5SS1_APB_PERIP_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID0_MSS_R5SS1_APB_PERIP_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID0_MSS_R5SS1_APB_PERIP_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID0_MSS_R5SS1_APB_PERIP_ID0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID0_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_APB_PERIP_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID1_MSS_R5SS1_APB_PERIP_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID1_MSS_R5SS1_APB_PERIP_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID1_MSS_R5SS1_APB_PERIP_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID1_MSS_R5SS1_APB_PERIP_ID1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID1_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_APB_PERIP_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID2_MSS_R5SS1_APB_PERIP_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID2_MSS_R5SS1_APB_PERIP_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID2_MSS_R5SS1_APB_PERIP_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID2_MSS_R5SS1_APB_PERIP_ID2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID2_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_APB_PERIP_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID3_MSS_R5SS1_APB_PERIP_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID3_MSS_R5SS1_APB_PERIP_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID3_MSS_R5SS1_APB_PERIP_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID3_MSS_R5SS1_APB_PERIP_ID3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_PERIP_ID3_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_APB_COMP_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID0_MSS_R5SS1_APB_COMP_ID0_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID0_MSS_R5SS1_APB_COMP_ID0_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID0_MSS_R5SS1_APB_COMP_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID0_MSS_R5SS1_APB_COMP_ID0_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID0_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_COMP_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID1_MSS_R5SS1_APB_COMP_ID1_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID1_MSS_R5SS1_APB_COMP_ID1_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID1_MSS_R5SS1_APB_COMP_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID1_MSS_R5SS1_APB_COMP_ID1_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID1_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_COMP_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID2_MSS_R5SS1_APB_COMP_ID2_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID2_MSS_R5SS1_APB_COMP_ID2_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID2_MSS_R5SS1_APB_COMP_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID2_MSS_R5SS1_APB_COMP_ID2_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID2_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_APB_COMP_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID3_MSS_R5SS1_APB_COMP_ID3_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID3_MSS_R5SS1_APB_COMP_ID3_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID3_MSS_R5SS1_APB_COMP_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID3_MSS_R5SS1_APB_COMP_ID3_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_APB_COMP_ID3_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_CTI_CONTROL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CONTROL_MSS_R5SS1_CTI_CONTROL_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CONTROL_MSS_R5SS1_CTI_CONTROL_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CONTROL_MSS_R5SS1_CTI_CONTROL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CONTROL_MSS_R5SS1_CTI_CONTROL_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CONTROL_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_CTI_INTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INTACK_MSS_R5SS1_CTI_INTACK_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INTACK_MSS_R5SS1_CTI_INTACK_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INTACK_MSS_R5SS1_CTI_INTACK_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INTACK_MSS_R5SS1_CTI_INTACK_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INTACK_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_APPSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPSET_MSS_R5SS1_CTI_APPSET_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPSET_MSS_R5SS1_CTI_APPSET_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPSET_MSS_R5SS1_CTI_APPSET_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPSET_MSS_R5SS1_CTI_APPSET_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPSET_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_APPCLEAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPCLEAR_MSS_R5SS1_CTI_APPCLEAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPCLEAR_MSS_R5SS1_CTI_APPCLEAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPCLEAR_MSS_R5SS1_CTI_APPCLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPCLEAR_MSS_R5SS1_CTI_APPCLEAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPCLEAR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_CTI_APPPULSE */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPPULSE_MSS_R5SS1_CTI_APPPULSE_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPPULSE_MSS_R5SS1_CTI_APPPULSE_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPPULSE_MSS_R5SS1_CTI_APPPULSE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPPULSE_MSS_R5SS1_CTI_APPPULSE_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_APPPULSE_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_CTI_INEN0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN0_MSS_R5SS1_CTI_INEN0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN0_MSS_R5SS1_CTI_INEN0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN0_MSS_R5SS1_CTI_INEN0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN0_MSS_R5SS1_CTI_INEN0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN0_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_CTI_INEN1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN1_MSS_R5SS1_CTI_INEN1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN1_MSS_R5SS1_CTI_INEN1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN1_MSS_R5SS1_CTI_INEN1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN1_MSS_R5SS1_CTI_INEN1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_CTI_INEN2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN2_MSS_R5SS1_CTI_INEN2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN2_MSS_R5SS1_CTI_INEN2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN2_MSS_R5SS1_CTI_INEN2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN2_MSS_R5SS1_CTI_INEN2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_CTI_INEN3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN3_MSS_R5SS1_CTI_INEN3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN3_MSS_R5SS1_CTI_INEN3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN3_MSS_R5SS1_CTI_INEN3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN3_MSS_R5SS1_CTI_INEN3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_CTI_INEN4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN4_MSS_R5SS1_CTI_INEN4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN4_MSS_R5SS1_CTI_INEN4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN4_MSS_R5SS1_CTI_INEN4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN4_MSS_R5SS1_CTI_INEN4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_CTI_INEN5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN5_MSS_R5SS1_CTI_INEN5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN5_MSS_R5SS1_CTI_INEN5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN5_MSS_R5SS1_CTI_INEN5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN5_MSS_R5SS1_CTI_INEN5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_CTI_INEN6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN6_MSS_R5SS1_CTI_INEN6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN6_MSS_R5SS1_CTI_INEN6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN6_MSS_R5SS1_CTI_INEN6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN6_MSS_R5SS1_CTI_INEN6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_CTI_INEN7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN7_MSS_R5SS1_CTI_INEN7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN7_MSS_R5SS1_CTI_INEN7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN7_MSS_R5SS1_CTI_INEN7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN7_MSS_R5SS1_CTI_INEN7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_INEN7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_CTI_OUTEN0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN0_MSS_R5SS1_CTI_OUTEN0_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN0_MSS_R5SS1_CTI_OUTEN0_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN0_MSS_R5SS1_CTI_OUTEN0_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN0_MSS_R5SS1_CTI_OUTEN0_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN0_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_OUTEN1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN1_MSS_R5SS1_CTI_OUTEN1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN1_MSS_R5SS1_CTI_OUTEN1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN1_MSS_R5SS1_CTI_OUTEN1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN1_MSS_R5SS1_CTI_OUTEN1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN1_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_OUTEN2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN2_MSS_R5SS1_CTI_OUTEN2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN2_MSS_R5SS1_CTI_OUTEN2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN2_MSS_R5SS1_CTI_OUTEN2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN2_MSS_R5SS1_CTI_OUTEN2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN2_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_OUTEN3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN3_MSS_R5SS1_CTI_OUTEN3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN3_MSS_R5SS1_CTI_OUTEN3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN3_MSS_R5SS1_CTI_OUTEN3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN3_MSS_R5SS1_CTI_OUTEN3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN3_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_OUTEN4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN4_MSS_R5SS1_CTI_OUTEN4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN4_MSS_R5SS1_CTI_OUTEN4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN4_MSS_R5SS1_CTI_OUTEN4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN4_MSS_R5SS1_CTI_OUTEN4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN4_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_OUTEN5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN5_MSS_R5SS1_CTI_OUTEN5_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN5_MSS_R5SS1_CTI_OUTEN5_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN5_MSS_R5SS1_CTI_OUTEN5_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN5_MSS_R5SS1_CTI_OUTEN5_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN5_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_OUTEN6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN6_MSS_R5SS1_CTI_OUTEN6_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN6_MSS_R5SS1_CTI_OUTEN6_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN6_MSS_R5SS1_CTI_OUTEN6_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN6_MSS_R5SS1_CTI_OUTEN6_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN6_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_OUTEN7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN7_MSS_R5SS1_CTI_OUTEN7_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN7_MSS_R5SS1_CTI_OUTEN7_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN7_MSS_R5SS1_CTI_OUTEN7_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN7_MSS_R5SS1_CTI_OUTEN7_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_OUTEN7_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_TRIGINSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGINSTATUS_MSS_R5SS1_CTI_TRIGINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGINSTATUS_MSS_R5SS1_CTI_TRIGINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGINSTATUS_MSS_R5SS1_CTI_TRIGINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGINSTATUS_MSS_R5SS1_CTI_TRIGINSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGINSTATUS_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_CTI_TRIGOUTSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGOUTSTATUS_MSS_R5SS1_CTI_TRIGOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGOUTSTATUS_MSS_R5SS1_CTI_TRIGOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGOUTSTATUS_MSS_R5SS1_CTI_TRIGOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGOUTSTATUS_MSS_R5SS1_CTI_TRIGOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_TRIGOUTSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_CHINSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHINSTATUS_MSS_R5SS1_CTI_CHINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHINSTATUS_MSS_R5SS1_CTI_CHINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHINSTATUS_MSS_R5SS1_CTI_CHINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHINSTATUS_MSS_R5SS1_CTI_CHINSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHINSTATUS_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_CTI_CHOUTSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHOUTSTATUS_MSS_R5SS1_CTI_CHOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHOUTSTATUS_MSS_R5SS1_CTI_CHOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHOUTSTATUS_MSS_R5SS1_CTI_CHOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHOUTSTATUS_MSS_R5SS1_CTI_CHOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CHOUTSTATUS_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_CTI_GATE */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_GATE_MSS_R5SS1_CTI_GATE_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_GATE_MSS_R5SS1_CTI_GATE_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_GATE_MSS_R5SS1_CTI_GATE_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_GATE_MSS_R5SS1_CTI_GATE_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_GATE_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_A_CTI_ASICCTL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ASICCTL_MSS_R5SS1_CTI_ASICCTL_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ASICCTL_MSS_R5SS1_CTI_ASICCTL_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ASICCTL_MSS_R5SS1_CTI_ASICCTL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ASICCTL_MSS_R5SS1_CTI_ASICCTL_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ASICCTL_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_CTI_ITCHINACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHINACK_MSS_R5SS1_CTI_ITCHINACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHINACK_MSS_R5SS1_CTI_ITCHINACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHINACK_MSS_R5SS1_CTI_ITCHINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHINACK_MSS_R5SS1_CTI_ITCHINACK_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHINACK_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_CTI_ITTRIGINACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGINACK_MSS_R5SS1_CTI_ITTRIGINACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGINACK_MSS_R5SS1_CTI_ITTRIGINACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGINACK_MSS_R5SS1_CTI_ITTRIGINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGINACK_MSS_R5SS1_CTI_ITTRIGINACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGINACK_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_A_CTI_ITCHOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUT_MSS_R5SS1_CTI_ITCHOUT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUT_MSS_R5SS1_CTI_ITCHOUT_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUT_MSS_R5SS1_CTI_ITCHOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUT_MSS_R5SS1_CTI_ITCHOUT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUT_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_CTI_ITTRIGOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUT_MSS_R5SS1_CTI_ITTRIGOUT_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUT_MSS_R5SS1_CTI_ITTRIGOUT_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUT_MSS_R5SS1_CTI_ITTRIGOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUT_MSS_R5SS1_CTI_ITTRIGOUT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUT_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_CTI_ITCHOUTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUTACK_MSS_R5SS1_CTI_ITCHOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUTACK_MSS_R5SS1_CTI_ITCHOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUTACK_MSS_R5SS1_CTI_ITCHOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUTACK_MSS_R5SS1_CTI_ITCHOUTACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHOUTACK_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_CTI_ITTRIGOUTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUTACK_MSS_R5SS1_CTI_ITTRIGOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUTACK_MSS_R5SS1_CTI_ITTRIGOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUTACK_MSS_R5SS1_CTI_ITTRIGOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUTACK_MSS_R5SS1_CTI_ITTRIGOUTACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGOUTACK_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_CTI_ITCHIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHIN_MSS_R5SS1_CTI_ITCHIN_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHIN_MSS_R5SS1_CTI_ITCHIN_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHIN_MSS_R5SS1_CTI_ITCHIN_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHIN_MSS_R5SS1_CTI_ITCHIN_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCHIN_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_ITTRIGIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGIN_MSS_R5SS1_CTI_ITTRIGIN_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGIN_MSS_R5SS1_CTI_ITTRIGIN_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGIN_MSS_R5SS1_CTI_ITTRIGIN_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGIN_MSS_R5SS1_CTI_ITTRIGIN_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITTRIGIN_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_CTI_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCTRL_MSS_R5SS1_CTI_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCTRL_MSS_R5SS1_CTI_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCTRL_MSS_R5SS1_CTI_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCTRL_MSS_R5SS1_CTI_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_ITCTRL_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_CTI_CLAIM_TAG_SET */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_SET_MSS_R5SS1_CTI_CLAIM_TAG_SET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_SET_MSS_R5SS1_CTI_CLAIM_TAG_SET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_SET_MSS_R5SS1_CTI_CLAIM_TAG_SET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_SET_MSS_R5SS1_CTI_CLAIM_TAG_SET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_SET_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_CLAIM_TAG_CLEAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_CLEAR_MSS_R5SS1_CTI_CLAIM_TAG_CLEAR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_CLEAR_MSS_R5SS1_CTI_CLAIM_TAG_CLEAR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_CLEAR_MSS_R5SS1_CTI_CLAIM_TAG_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_CLEAR_MSS_R5SS1_CTI_CLAIM_TAG_CLEAR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_CLAIM_TAG_CLEAR_RESETVAL               (0x00000000U)

/* MSS_R5SS1_A_CTI_LOCK_ACCESS_REGISTER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS1_CTI_LOCK_ACCESS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS1_CTI_LOCK_ACCESS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS1_CTI_LOCK_ACCESS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS1_CTI_LOCK_ACCESS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_ACCESS_REGISTER_RESETVAL          (0x00000000U)

/* MSS_R5SS1_A_CTI_LOCK_STATUS_REGISTER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_STATUS_REGISTER_MSS_R5SS1_CTI_LOCK_STATUS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_STATUS_REGISTER_MSS_R5SS1_CTI_LOCK_STATUS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_STATUS_REGISTER_MSS_R5SS1_CTI_LOCK_STATUS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_STATUS_REGISTER_MSS_R5SS1_CTI_LOCK_STATUS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_LOCK_STATUS_REGISTER_RESETVAL          (0x00000000U)

/* MSS_R5SS1_A_CTI_AUTHENTICATION_STATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_AUTHENTICATION_STATUS_MSS_R5SS1_CTI_AUTHENTICATION_STATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_AUTHENTICATION_STATUS_MSS_R5SS1_CTI_AUTHENTICATION_STATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_AUTHENTICATION_STATUS_MSS_R5SS1_CTI_AUTHENTICATION_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_AUTHENTICATION_STATUS_MSS_R5SS1_CTI_AUTHENTICATION_STATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_AUTHENTICATION_STATUS_RESETVAL         (0x00000000U)

/* MSS_R5SS1_A_CTI_DEVICE_ID */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_ID_MSS_R5SS1_CTI_DEVICE_ID_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_ID_MSS_R5SS1_CTI_DEVICE_ID_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_ID_MSS_R5SS1_CTI_DEVICE_ID_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_ID_MSS_R5SS1_CTI_DEVICE_ID_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_ID_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_CTI_DEVICE_TYPE_IDENTIFIER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS1_CTI_DEVICE_TYPE_IDENTIFIER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS1_CTI_DEVICE_TYPE_IDENTIFIER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS1_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS1_CTI_DEVICE_TYPE_IDENTIFIER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL        (0x00000000U)

/* MSS_R5SS1_A_CTI_PERIPHERALID4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID4_MSS_R5SS1_CTI_PERIPHERALID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID4_MSS_R5SS1_CTI_PERIPHERALID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID4_MSS_R5SS1_CTI_PERIPHERALID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID4_MSS_R5SS1_CTI_PERIPHERALID4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID4_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_PERIPHERALID5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID5_MSS_R5SS1_CTI_PERIPHERALID5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID5_MSS_R5SS1_CTI_PERIPHERALID5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID5_MSS_R5SS1_CTI_PERIPHERALID5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID5_MSS_R5SS1_CTI_PERIPHERALID5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID5_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_PERIPHERALID6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID6_MSS_R5SS1_CTI_PERIPHERALID6_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID6_MSS_R5SS1_CTI_PERIPHERALID6_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID6_MSS_R5SS1_CTI_PERIPHERALID6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID6_MSS_R5SS1_CTI_PERIPHERALID6_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID6_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_PERIPHERALID7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID7_MSS_R5SS1_CTI_PERIPHERALID7_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID7_MSS_R5SS1_CTI_PERIPHERALID7_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID7_MSS_R5SS1_CTI_PERIPHERALID7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID7_MSS_R5SS1_CTI_PERIPHERALID7_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID7_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_PERIPHERALID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID0_MSS_R5SS1_CTI_PERIPHERALID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID0_MSS_R5SS1_CTI_PERIPHERALID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID0_MSS_R5SS1_CTI_PERIPHERALID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID0_MSS_R5SS1_CTI_PERIPHERALID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID0_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_PERIPHERALID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID1_MSS_R5SS1_CTI_PERIPHERALID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID1_MSS_R5SS1_CTI_PERIPHERALID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID1_MSS_R5SS1_CTI_PERIPHERALID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID1_MSS_R5SS1_CTI_PERIPHERALID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID1_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_PERIPHERALID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID2_MSS_R5SS1_CTI_PERIPHERALID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID2_MSS_R5SS1_CTI_PERIPHERALID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID2_MSS_R5SS1_CTI_PERIPHERALID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID2_MSS_R5SS1_CTI_PERIPHERALID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID2_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_PERIPHERALID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID3_MSS_R5SS1_CTI_PERIPHERALID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID3_MSS_R5SS1_CTI_PERIPHERALID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID3_MSS_R5SS1_CTI_PERIPHERALID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID3_MSS_R5SS1_CTI_PERIPHERALID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_PERIPHERALID3_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_COMPONENT_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID0_MSS_R5SS1_CTI_COMPONENT_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID0_MSS_R5SS1_CTI_COMPONENT_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID0_MSS_R5SS1_CTI_COMPONENT_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID0_MSS_R5SS1_CTI_COMPONENT_ID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID0_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_COMPONENT_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID1_MSS_R5SS1_CTI_COMPONENT_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID1_MSS_R5SS1_CTI_COMPONENT_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID1_MSS_R5SS1_CTI_COMPONENT_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID1_MSS_R5SS1_CTI_COMPONENT_ID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID1_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_COMPONENT_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID2_MSS_R5SS1_CTI_COMPONENT_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID2_MSS_R5SS1_CTI_COMPONENT_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID2_MSS_R5SS1_CTI_COMPONENT_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID2_MSS_R5SS1_CTI_COMPONENT_ID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID2_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_CTI_COMPONENT_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID3_MSS_R5SS1_CTI_COMPONENT_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID3_MSS_R5SS1_CTI_COMPONENT_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID3_MSS_R5SS1_CTI_COMPONENT_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID3_MSS_R5SS1_CTI_COMPONENT_ID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_CTI_COMPONENT_ID3_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_CONTROL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CONTROL_MSS_R5SS1_CTI_CONTROL_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CONTROL_MSS_R5SS1_CTI_CONTROL_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CONTROL_MSS_R5SS1_CTI_CONTROL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CONTROL_MSS_R5SS1_CTI_CONTROL_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CONTROL_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_CTI_INTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INTACK_MSS_R5SS1_CTI_INTACK_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INTACK_MSS_R5SS1_CTI_INTACK_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INTACK_MSS_R5SS1_CTI_INTACK_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INTACK_MSS_R5SS1_CTI_INTACK_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INTACK_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_APPSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPSET_MSS_R5SS1_CTI_APPSET_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPSET_MSS_R5SS1_CTI_APPSET_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPSET_MSS_R5SS1_CTI_APPSET_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPSET_MSS_R5SS1_CTI_APPSET_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPSET_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_APPCLEAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPCLEAR_MSS_R5SS1_CTI_APPCLEAR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPCLEAR_MSS_R5SS1_CTI_APPCLEAR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPCLEAR_MSS_R5SS1_CTI_APPCLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPCLEAR_MSS_R5SS1_CTI_APPCLEAR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPCLEAR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_CTI_APPPULSE */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPPULSE_MSS_R5SS1_CTI_APPPULSE_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPPULSE_MSS_R5SS1_CTI_APPPULSE_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPPULSE_MSS_R5SS1_CTI_APPPULSE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPPULSE_MSS_R5SS1_CTI_APPPULSE_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_APPPULSE_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_CTI_INEN0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN0_MSS_R5SS1_CTI_INEN0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN0_MSS_R5SS1_CTI_INEN0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN0_MSS_R5SS1_CTI_INEN0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN0_MSS_R5SS1_CTI_INEN0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN0_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_CTI_INEN1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN1_MSS_R5SS1_CTI_INEN1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN1_MSS_R5SS1_CTI_INEN1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN1_MSS_R5SS1_CTI_INEN1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN1_MSS_R5SS1_CTI_INEN1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_CTI_INEN2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN2_MSS_R5SS1_CTI_INEN2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN2_MSS_R5SS1_CTI_INEN2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN2_MSS_R5SS1_CTI_INEN2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN2_MSS_R5SS1_CTI_INEN2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_CTI_INEN3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN3_MSS_R5SS1_CTI_INEN3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN3_MSS_R5SS1_CTI_INEN3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN3_MSS_R5SS1_CTI_INEN3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN3_MSS_R5SS1_CTI_INEN3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_CTI_INEN4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN4_MSS_R5SS1_CTI_INEN4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN4_MSS_R5SS1_CTI_INEN4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN4_MSS_R5SS1_CTI_INEN4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN4_MSS_R5SS1_CTI_INEN4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_CTI_INEN5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN5_MSS_R5SS1_CTI_INEN5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN5_MSS_R5SS1_CTI_INEN5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN5_MSS_R5SS1_CTI_INEN5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN5_MSS_R5SS1_CTI_INEN5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_CTI_INEN6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN6_MSS_R5SS1_CTI_INEN6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN6_MSS_R5SS1_CTI_INEN6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN6_MSS_R5SS1_CTI_INEN6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN6_MSS_R5SS1_CTI_INEN6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_CTI_INEN7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN7_MSS_R5SS1_CTI_INEN7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN7_MSS_R5SS1_CTI_INEN7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN7_MSS_R5SS1_CTI_INEN7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN7_MSS_R5SS1_CTI_INEN7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_INEN7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_CTI_OUTEN0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN0_MSS_R5SS1_CTI_OUTEN0_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN0_MSS_R5SS1_CTI_OUTEN0_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN0_MSS_R5SS1_CTI_OUTEN0_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN0_MSS_R5SS1_CTI_OUTEN0_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN0_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_OUTEN1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN1_MSS_R5SS1_CTI_OUTEN1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN1_MSS_R5SS1_CTI_OUTEN1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN1_MSS_R5SS1_CTI_OUTEN1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN1_MSS_R5SS1_CTI_OUTEN1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN1_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_OUTEN2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN2_MSS_R5SS1_CTI_OUTEN2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN2_MSS_R5SS1_CTI_OUTEN2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN2_MSS_R5SS1_CTI_OUTEN2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN2_MSS_R5SS1_CTI_OUTEN2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN2_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_OUTEN3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN3_MSS_R5SS1_CTI_OUTEN3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN3_MSS_R5SS1_CTI_OUTEN3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN3_MSS_R5SS1_CTI_OUTEN3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN3_MSS_R5SS1_CTI_OUTEN3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN3_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_OUTEN4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN4_MSS_R5SS1_CTI_OUTEN4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN4_MSS_R5SS1_CTI_OUTEN4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN4_MSS_R5SS1_CTI_OUTEN4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN4_MSS_R5SS1_CTI_OUTEN4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN4_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_OUTEN5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN5_MSS_R5SS1_CTI_OUTEN5_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN5_MSS_R5SS1_CTI_OUTEN5_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN5_MSS_R5SS1_CTI_OUTEN5_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN5_MSS_R5SS1_CTI_OUTEN5_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN5_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_OUTEN6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN6_MSS_R5SS1_CTI_OUTEN6_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN6_MSS_R5SS1_CTI_OUTEN6_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN6_MSS_R5SS1_CTI_OUTEN6_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN6_MSS_R5SS1_CTI_OUTEN6_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN6_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_OUTEN7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN7_MSS_R5SS1_CTI_OUTEN7_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN7_MSS_R5SS1_CTI_OUTEN7_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN7_MSS_R5SS1_CTI_OUTEN7_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN7_MSS_R5SS1_CTI_OUTEN7_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_OUTEN7_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_TRIGINSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGINSTATUS_MSS_R5SS1_CTI_TRIGINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGINSTATUS_MSS_R5SS1_CTI_TRIGINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGINSTATUS_MSS_R5SS1_CTI_TRIGINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGINSTATUS_MSS_R5SS1_CTI_TRIGINSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGINSTATUS_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_CTI_TRIGOUTSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGOUTSTATUS_MSS_R5SS1_CTI_TRIGOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGOUTSTATUS_MSS_R5SS1_CTI_TRIGOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGOUTSTATUS_MSS_R5SS1_CTI_TRIGOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGOUTSTATUS_MSS_R5SS1_CTI_TRIGOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_TRIGOUTSTATUS_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_CHINSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHINSTATUS_MSS_R5SS1_CTI_CHINSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHINSTATUS_MSS_R5SS1_CTI_CHINSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHINSTATUS_MSS_R5SS1_CTI_CHINSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHINSTATUS_MSS_R5SS1_CTI_CHINSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHINSTATUS_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_CTI_CHOUTSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHOUTSTATUS_MSS_R5SS1_CTI_CHOUTSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHOUTSTATUS_MSS_R5SS1_CTI_CHOUTSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHOUTSTATUS_MSS_R5SS1_CTI_CHOUTSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHOUTSTATUS_MSS_R5SS1_CTI_CHOUTSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CHOUTSTATUS_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_CTI_GATE */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_GATE_MSS_R5SS1_CTI_GATE_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_GATE_MSS_R5SS1_CTI_GATE_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_GATE_MSS_R5SS1_CTI_GATE_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_GATE_MSS_R5SS1_CTI_GATE_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_GATE_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_B_CTI_ASICCTL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ASICCTL_MSS_R5SS1_CTI_ASICCTL_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ASICCTL_MSS_R5SS1_CTI_ASICCTL_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ASICCTL_MSS_R5SS1_CTI_ASICCTL_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ASICCTL_MSS_R5SS1_CTI_ASICCTL_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ASICCTL_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_CTI_ITCHINACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHINACK_MSS_R5SS1_CTI_ITCHINACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHINACK_MSS_R5SS1_CTI_ITCHINACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHINACK_MSS_R5SS1_CTI_ITCHINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHINACK_MSS_R5SS1_CTI_ITCHINACK_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHINACK_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_CTI_ITTRIGINACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGINACK_MSS_R5SS1_CTI_ITTRIGINACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGINACK_MSS_R5SS1_CTI_ITTRIGINACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGINACK_MSS_R5SS1_CTI_ITTRIGINACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGINACK_MSS_R5SS1_CTI_ITTRIGINACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGINACK_RESETVAL                   (0x00000000U)

/* MSS_R5SS1_B_CTI_ITCHOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUT_MSS_R5SS1_CTI_ITCHOUT_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUT_MSS_R5SS1_CTI_ITCHOUT_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUT_MSS_R5SS1_CTI_ITCHOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUT_MSS_R5SS1_CTI_ITCHOUT_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUT_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_CTI_ITTRIGOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUT_MSS_R5SS1_CTI_ITTRIGOUT_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUT_MSS_R5SS1_CTI_ITTRIGOUT_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUT_MSS_R5SS1_CTI_ITTRIGOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUT_MSS_R5SS1_CTI_ITTRIGOUT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUT_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_CTI_ITCHOUTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUTACK_MSS_R5SS1_CTI_ITCHOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUTACK_MSS_R5SS1_CTI_ITCHOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUTACK_MSS_R5SS1_CTI_ITCHOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUTACK_MSS_R5SS1_CTI_ITCHOUTACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHOUTACK_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_CTI_ITTRIGOUTACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUTACK_MSS_R5SS1_CTI_ITTRIGOUTACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUTACK_MSS_R5SS1_CTI_ITTRIGOUTACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUTACK_MSS_R5SS1_CTI_ITTRIGOUTACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUTACK_MSS_R5SS1_CTI_ITTRIGOUTACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGOUTACK_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_CTI_ITCHIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHIN_MSS_R5SS1_CTI_ITCHIN_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHIN_MSS_R5SS1_CTI_ITCHIN_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHIN_MSS_R5SS1_CTI_ITCHIN_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHIN_MSS_R5SS1_CTI_ITCHIN_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCHIN_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_ITTRIGIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGIN_MSS_R5SS1_CTI_ITTRIGIN_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGIN_MSS_R5SS1_CTI_ITTRIGIN_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGIN_MSS_R5SS1_CTI_ITTRIGIN_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGIN_MSS_R5SS1_CTI_ITTRIGIN_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITTRIGIN_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_CTI_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCTRL_MSS_R5SS1_CTI_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCTRL_MSS_R5SS1_CTI_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCTRL_MSS_R5SS1_CTI_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCTRL_MSS_R5SS1_CTI_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_ITCTRL_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_CTI_CLAIM_TAG_SET */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_SET_MSS_R5SS1_CTI_CLAIM_TAG_SET_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_SET_MSS_R5SS1_CTI_CLAIM_TAG_SET_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_SET_MSS_R5SS1_CTI_CLAIM_TAG_SET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_SET_MSS_R5SS1_CTI_CLAIM_TAG_SET_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_SET_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_CLAIM_TAG_CLEAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_CLEAR_MSS_R5SS1_CTI_CLAIM_TAG_CLEAR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_CLEAR_MSS_R5SS1_CTI_CLAIM_TAG_CLEAR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_CLEAR_MSS_R5SS1_CTI_CLAIM_TAG_CLEAR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_CLEAR_MSS_R5SS1_CTI_CLAIM_TAG_CLEAR_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_CLAIM_TAG_CLEAR_RESETVAL               (0x00000000U)

/* MSS_R5SS1_B_CTI_LOCK_ACCESS_REGISTER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS1_CTI_LOCK_ACCESS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS1_CTI_LOCK_ACCESS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS1_CTI_LOCK_ACCESS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_ACCESS_REGISTER_MSS_R5SS1_CTI_LOCK_ACCESS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_ACCESS_REGISTER_RESETVAL          (0x00000000U)

/* MSS_R5SS1_B_CTI_LOCK_STATUS_REGISTER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_STATUS_REGISTER_MSS_R5SS1_CTI_LOCK_STATUS_REGISTER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_STATUS_REGISTER_MSS_R5SS1_CTI_LOCK_STATUS_REGISTER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_STATUS_REGISTER_MSS_R5SS1_CTI_LOCK_STATUS_REGISTER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_STATUS_REGISTER_MSS_R5SS1_CTI_LOCK_STATUS_REGISTER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_LOCK_STATUS_REGISTER_RESETVAL          (0x00000000U)

/* MSS_R5SS1_B_CTI_AUTHENTICATION_STATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_AUTHENTICATION_STATUS_MSS_R5SS1_CTI_AUTHENTICATION_STATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_AUTHENTICATION_STATUS_MSS_R5SS1_CTI_AUTHENTICATION_STATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_AUTHENTICATION_STATUS_MSS_R5SS1_CTI_AUTHENTICATION_STATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_AUTHENTICATION_STATUS_MSS_R5SS1_CTI_AUTHENTICATION_STATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_AUTHENTICATION_STATUS_RESETVAL         (0x00000000U)

/* MSS_R5SS1_B_CTI_DEVICE_ID */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_ID_MSS_R5SS1_CTI_DEVICE_ID_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_ID_MSS_R5SS1_CTI_DEVICE_ID_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_ID_MSS_R5SS1_CTI_DEVICE_ID_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_ID_MSS_R5SS1_CTI_DEVICE_ID_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_ID_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_CTI_DEVICE_TYPE_IDENTIFIER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS1_CTI_DEVICE_TYPE_IDENTIFIER_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS1_CTI_DEVICE_TYPE_IDENTIFIER_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS1_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_TYPE_IDENTIFIER_MSS_R5SS1_CTI_DEVICE_TYPE_IDENTIFIER_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_DEVICE_TYPE_IDENTIFIER_RESETVAL        (0x00000000U)

/* MSS_R5SS1_B_CTI_PERIPHERALID4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID4_MSS_R5SS1_CTI_PERIPHERALID4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID4_MSS_R5SS1_CTI_PERIPHERALID4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID4_MSS_R5SS1_CTI_PERIPHERALID4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID4_MSS_R5SS1_CTI_PERIPHERALID4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID4_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_PERIPHERALID5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID5_MSS_R5SS1_CTI_PERIPHERALID5_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID5_MSS_R5SS1_CTI_PERIPHERALID5_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID5_MSS_R5SS1_CTI_PERIPHERALID5_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID5_MSS_R5SS1_CTI_PERIPHERALID5_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID5_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_PERIPHERALID6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID6_MSS_R5SS1_CTI_PERIPHERALID6_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID6_MSS_R5SS1_CTI_PERIPHERALID6_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID6_MSS_R5SS1_CTI_PERIPHERALID6_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID6_MSS_R5SS1_CTI_PERIPHERALID6_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID6_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_PERIPHERALID7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID7_MSS_R5SS1_CTI_PERIPHERALID7_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID7_MSS_R5SS1_CTI_PERIPHERALID7_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID7_MSS_R5SS1_CTI_PERIPHERALID7_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID7_MSS_R5SS1_CTI_PERIPHERALID7_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID7_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_PERIPHERALID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID0_MSS_R5SS1_CTI_PERIPHERALID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID0_MSS_R5SS1_CTI_PERIPHERALID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID0_MSS_R5SS1_CTI_PERIPHERALID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID0_MSS_R5SS1_CTI_PERIPHERALID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID0_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_PERIPHERALID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID1_MSS_R5SS1_CTI_PERIPHERALID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID1_MSS_R5SS1_CTI_PERIPHERALID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID1_MSS_R5SS1_CTI_PERIPHERALID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID1_MSS_R5SS1_CTI_PERIPHERALID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID1_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_PERIPHERALID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID2_MSS_R5SS1_CTI_PERIPHERALID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID2_MSS_R5SS1_CTI_PERIPHERALID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID2_MSS_R5SS1_CTI_PERIPHERALID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID2_MSS_R5SS1_CTI_PERIPHERALID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID2_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_PERIPHERALID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID3_MSS_R5SS1_CTI_PERIPHERALID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID3_MSS_R5SS1_CTI_PERIPHERALID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID3_MSS_R5SS1_CTI_PERIPHERALID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID3_MSS_R5SS1_CTI_PERIPHERALID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_PERIPHERALID3_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_COMPONENT_ID0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID0_MSS_R5SS1_CTI_COMPONENT_ID0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID0_MSS_R5SS1_CTI_COMPONENT_ID0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID0_MSS_R5SS1_CTI_COMPONENT_ID0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID0_MSS_R5SS1_CTI_COMPONENT_ID0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID0_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_COMPONENT_ID1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID1_MSS_R5SS1_CTI_COMPONENT_ID1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID1_MSS_R5SS1_CTI_COMPONENT_ID1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID1_MSS_R5SS1_CTI_COMPONENT_ID1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID1_MSS_R5SS1_CTI_COMPONENT_ID1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID1_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_COMPONENT_ID2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID2_MSS_R5SS1_CTI_COMPONENT_ID2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID2_MSS_R5SS1_CTI_COMPONENT_ID2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID2_MSS_R5SS1_CTI_COMPONENT_ID2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID2_MSS_R5SS1_CTI_COMPONENT_ID2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID2_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_B_CTI_COMPONENT_ID3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID3_MSS_R5SS1_CTI_COMPONENT_ID3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID3_MSS_R5SS1_CTI_COMPONENT_ID3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID3_MSS_R5SS1_CTI_COMPONENT_ID3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID3_MSS_R5SS1_CTI_COMPONENT_ID3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_CTI_COMPONENT_ID3_RESETVAL                 (0x00000000U)

/* MSS_R5SS1_A_ETM_CR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CR_MSS_R5SS1_ETM_CR_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CR_MSS_R5SS1_ETM_CR_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CR_MSS_R5SS1_ETM_CR_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CR_MSS_R5SS1_ETM_CR_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CR_RESETVAL                            (0x00000000U)

/* MSS_R5SS1_A_ETM_CCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCR_MSS_R5SS1_ETM_CCR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCR_MSS_R5SS1_ETM_CCR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCR_MSS_R5SS1_ETM_CCR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCR_MSS_R5SS1_ETM_CCR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_A_ETM_TRIGGER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRIGGER_MSS_R5SS1_ETM_TRIGGER_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRIGGER_MSS_R5SS1_ETM_TRIGGER_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRIGGER_MSS_R5SS1_ETM_TRIGGER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRIGGER_MSS_R5SS1_ETM_TRIGGER_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRIGGER_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_ASICCTLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ASICCTLR_MSS_R5SS1_ETM_ASICCTLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ASICCTLR_MSS_R5SS1_ETM_ASICCTLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ASICCTLR_MSS_R5SS1_ETM_ASICCTLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ASICCTLR_MSS_R5SS1_ETM_ASICCTLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ASICCTLR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_ETM_SR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SR_MSS_R5SS1_ETM_SR_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SR_MSS_R5SS1_ETM_SR_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SR_MSS_R5SS1_ETM_SR_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SR_MSS_R5SS1_ETM_SR_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SR_RESETVAL                            (0x00000000U)

/* MSS_R5SS1_A_ETM_SCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SCR_MSS_R5SS1_ETM_SCR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SCR_MSS_R5SS1_ETM_SCR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SCR_MSS_R5SS1_ETM_SCR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SCR_MSS_R5SS1_ETM_SCR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SCR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_A_ETM_TSSCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TSSCR_MSS_R5SS1_ETM_TSSCR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TSSCR_MSS_R5SS1_ETM_TSSCR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TSSCR_MSS_R5SS1_ETM_TSSCR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TSSCR_MSS_R5SS1_ETM_TSSCR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TSSCR_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_TECR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR2_MSS_R5SS1_ETM_TECR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR2_MSS_R5SS1_ETM_TECR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR2_MSS_R5SS1_ETM_TECR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR2_MSS_R5SS1_ETM_TECR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_TEEVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TEEVR_MSS_R5SS1_ETM_TEEVR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TEEVR_MSS_R5SS1_ETM_TEEVR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TEEVR_MSS_R5SS1_ETM_TEEVR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TEEVR_MSS_R5SS1_ETM_TEEVR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TEEVR_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_TECR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR1_MSS_R5SS1_ETM_TECR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR1_MSS_R5SS1_ETM_TECR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR1_MSS_R5SS1_ETM_TECR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR1_MSS_R5SS1_ETM_TECR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TECR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_FFRR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFRR_MSS_R5SS1_ETM_FFRR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFRR_MSS_R5SS1_ETM_FFRR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFRR_MSS_R5SS1_ETM_FFRR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFRR_MSS_R5SS1_ETM_FFRR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFRR_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_A_ETM_FFLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFLR_MSS_R5SS1_ETM_FFLR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFLR_MSS_R5SS1_ETM_FFLR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFLR_MSS_R5SS1_ETM_FFLR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFLR_MSS_R5SS1_ETM_FFLR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_FFLR_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_A_ETM_VDEVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDEVR_MSS_R5SS1_ETM_VDEVR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDEVR_MSS_R5SS1_ETM_VDEVR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDEVR_MSS_R5SS1_ETM_VDEVR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDEVR_MSS_R5SS1_ETM_VDEVR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDEVR_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_VDCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR1_MSS_R5SS1_ETM_VDCR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR1_MSS_R5SS1_ETM_VDCR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR1_MSS_R5SS1_ETM_VDCR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR1_MSS_R5SS1_ETM_VDCR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_VDCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR2_MSS_R5SS1_ETM_VDCR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR2_MSS_R5SS1_ETM_VDCR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR2_MSS_R5SS1_ETM_VDCR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR2_MSS_R5SS1_ETM_VDCR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_VDCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR3_MSS_R5SS1_ETM_VDCR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR3_MSS_R5SS1_ETM_VDCR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR3_MSS_R5SS1_ETM_VDCR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR3_MSS_R5SS1_ETM_VDCR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_VDCR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR1_MSS_R5SS1_ETM_ACVR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR1_MSS_R5SS1_ETM_ACVR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR1_MSS_R5SS1_ETM_ACVR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR1_MSS_R5SS1_ETM_ACVR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR2_MSS_R5SS1_ETM_ACVR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR2_MSS_R5SS1_ETM_ACVR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR2_MSS_R5SS1_ETM_ACVR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR2_MSS_R5SS1_ETM_ACVR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR3_MSS_R5SS1_ETM_ACVR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR3_MSS_R5SS1_ETM_ACVR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR3_MSS_R5SS1_ETM_ACVR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR3_MSS_R5SS1_ETM_ACVR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR4_MSS_R5SS1_ETM_ACVR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR4_MSS_R5SS1_ETM_ACVR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR4_MSS_R5SS1_ETM_ACVR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR4_MSS_R5SS1_ETM_ACVR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR5_MSS_R5SS1_ETM_ACVR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR5_MSS_R5SS1_ETM_ACVR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR5_MSS_R5SS1_ETM_ACVR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR5_MSS_R5SS1_ETM_ACVR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR6_MSS_R5SS1_ETM_ACVR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR6_MSS_R5SS1_ETM_ACVR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR6_MSS_R5SS1_ETM_ACVR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR6_MSS_R5SS1_ETM_ACVR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR7_MSS_R5SS1_ETM_ACVR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR7_MSS_R5SS1_ETM_ACVR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR7_MSS_R5SS1_ETM_ACVR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR7_MSS_R5SS1_ETM_ACVR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR8_MSS_R5SS1_ETM_ACVR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR8_MSS_R5SS1_ETM_ACVR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR8_MSS_R5SS1_ETM_ACVR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR8_MSS_R5SS1_ETM_ACVR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR9 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR9_MSS_R5SS1_ETM_ACVR9_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR9_MSS_R5SS1_ETM_ACVR9_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR9_MSS_R5SS1_ETM_ACVR9_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR9_MSS_R5SS1_ETM_ACVR9_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR9_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR10 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR10_MSS_R5SS1_ETM_ACVR10_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR10_MSS_R5SS1_ETM_ACVR10_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR10_MSS_R5SS1_ETM_ACVR10_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR10_MSS_R5SS1_ETM_ACVR10_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR10_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR11 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR11_MSS_R5SS1_ETM_ACVR11_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR11_MSS_R5SS1_ETM_ACVR11_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR11_MSS_R5SS1_ETM_ACVR11_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR11_MSS_R5SS1_ETM_ACVR11_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR11_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR12 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR12_MSS_R5SS1_ETM_ACVR12_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR12_MSS_R5SS1_ETM_ACVR12_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR12_MSS_R5SS1_ETM_ACVR12_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR12_MSS_R5SS1_ETM_ACVR12_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR12_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR13 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR13_MSS_R5SS1_ETM_ACVR13_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR13_MSS_R5SS1_ETM_ACVR13_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR13_MSS_R5SS1_ETM_ACVR13_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR13_MSS_R5SS1_ETM_ACVR13_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR13_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR14 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR14_MSS_R5SS1_ETM_ACVR14_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR14_MSS_R5SS1_ETM_ACVR14_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR14_MSS_R5SS1_ETM_ACVR14_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR14_MSS_R5SS1_ETM_ACVR14_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR14_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR15 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR15_MSS_R5SS1_ETM_ACVR15_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR15_MSS_R5SS1_ETM_ACVR15_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR15_MSS_R5SS1_ETM_ACVR15_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR15_MSS_R5SS1_ETM_ACVR15_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR15_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACVR16 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR16_MSS_R5SS1_ETM_ACVR16_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR16_MSS_R5SS1_ETM_ACVR16_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR16_MSS_R5SS1_ETM_ACVR16_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR16_MSS_R5SS1_ETM_ACVR16_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACVR16_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR1_MSS_R5SS1_ETM_ACTR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR1_MSS_R5SS1_ETM_ACTR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR1_MSS_R5SS1_ETM_ACTR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR1_MSS_R5SS1_ETM_ACTR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR2_MSS_R5SS1_ETM_ACTR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR2_MSS_R5SS1_ETM_ACTR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR2_MSS_R5SS1_ETM_ACTR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR2_MSS_R5SS1_ETM_ACTR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR3_MSS_R5SS1_ETM_ACTR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR3_MSS_R5SS1_ETM_ACTR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR3_MSS_R5SS1_ETM_ACTR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR3_MSS_R5SS1_ETM_ACTR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR4_MSS_R5SS1_ETM_ACTR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR4_MSS_R5SS1_ETM_ACTR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR4_MSS_R5SS1_ETM_ACTR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR4_MSS_R5SS1_ETM_ACTR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR5_MSS_R5SS1_ETM_ACTR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR5_MSS_R5SS1_ETM_ACTR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR5_MSS_R5SS1_ETM_ACTR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR5_MSS_R5SS1_ETM_ACTR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR6_MSS_R5SS1_ETM_ACTR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR6_MSS_R5SS1_ETM_ACTR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR6_MSS_R5SS1_ETM_ACTR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR6_MSS_R5SS1_ETM_ACTR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR7_MSS_R5SS1_ETM_ACTR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR7_MSS_R5SS1_ETM_ACTR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR7_MSS_R5SS1_ETM_ACTR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR7_MSS_R5SS1_ETM_ACTR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR8_MSS_R5SS1_ETM_ACTR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR8_MSS_R5SS1_ETM_ACTR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR8_MSS_R5SS1_ETM_ACTR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR8_MSS_R5SS1_ETM_ACTR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR9 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR9_MSS_R5SS1_ETM_ACTR9_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR9_MSS_R5SS1_ETM_ACTR9_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR9_MSS_R5SS1_ETM_ACTR9_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR9_MSS_R5SS1_ETM_ACTR9_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR9_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR10 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR10_MSS_R5SS1_ETM_ACTR10_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR10_MSS_R5SS1_ETM_ACTR10_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR10_MSS_R5SS1_ETM_ACTR10_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR10_MSS_R5SS1_ETM_ACTR10_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR10_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR11 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR11_MSS_R5SS1_ETM_ACTR11_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR11_MSS_R5SS1_ETM_ACTR11_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR11_MSS_R5SS1_ETM_ACTR11_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR11_MSS_R5SS1_ETM_ACTR11_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR11_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR12 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR12_MSS_R5SS1_ETM_ACTR12_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR12_MSS_R5SS1_ETM_ACTR12_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR12_MSS_R5SS1_ETM_ACTR12_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR12_MSS_R5SS1_ETM_ACTR12_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR12_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR13 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR13_MSS_R5SS1_ETM_ACTR13_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR13_MSS_R5SS1_ETM_ACTR13_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR13_MSS_R5SS1_ETM_ACTR13_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR13_MSS_R5SS1_ETM_ACTR13_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR13_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR14 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR14_MSS_R5SS1_ETM_ACTR14_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR14_MSS_R5SS1_ETM_ACTR14_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR14_MSS_R5SS1_ETM_ACTR14_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR14_MSS_R5SS1_ETM_ACTR14_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR14_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR15 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR15_MSS_R5SS1_ETM_ACTR15_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR15_MSS_R5SS1_ETM_ACTR15_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR15_MSS_R5SS1_ETM_ACTR15_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR15_MSS_R5SS1_ETM_ACTR15_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR15_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_ACTR16 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR16_MSS_R5SS1_ETM_ACTR16_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR16_MSS_R5SS1_ETM_ACTR16_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR16_MSS_R5SS1_ETM_ACTR16_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR16_MSS_R5SS1_ETM_ACTR16_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ACTR16_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_DCVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR1_MSS_R5SS1_ETM_DCVR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR1_MSS_R5SS1_ETM_DCVR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR1_MSS_R5SS1_ETM_DCVR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR1_MSS_R5SS1_ETM_DCVR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR2_MSS_R5SS1_ETM_DCVR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR2_MSS_R5SS1_ETM_DCVR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR2_MSS_R5SS1_ETM_DCVR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR2_MSS_R5SS1_ETM_DCVR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR3_MSS_R5SS1_ETM_DCVR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR3_MSS_R5SS1_ETM_DCVR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR3_MSS_R5SS1_ETM_DCVR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR3_MSS_R5SS1_ETM_DCVR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR4_MSS_R5SS1_ETM_DCVR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR4_MSS_R5SS1_ETM_DCVR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR4_MSS_R5SS1_ETM_DCVR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR4_MSS_R5SS1_ETM_DCVR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR5_MSS_R5SS1_ETM_DCVR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR5_MSS_R5SS1_ETM_DCVR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR5_MSS_R5SS1_ETM_DCVR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR5_MSS_R5SS1_ETM_DCVR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR6_MSS_R5SS1_ETM_DCVR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR6_MSS_R5SS1_ETM_DCVR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR6_MSS_R5SS1_ETM_DCVR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR6_MSS_R5SS1_ETM_DCVR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR7_MSS_R5SS1_ETM_DCVR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR7_MSS_R5SS1_ETM_DCVR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR7_MSS_R5SS1_ETM_DCVR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR7_MSS_R5SS1_ETM_DCVR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCVR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR8_MSS_R5SS1_ETM_DCVR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR8_MSS_R5SS1_ETM_DCVR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR8_MSS_R5SS1_ETM_DCVR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR8_MSS_R5SS1_ETM_DCVR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCVR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCMR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR1_MSS_R5SS1_ETM_DCMR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR1_MSS_R5SS1_ETM_DCMR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR1_MSS_R5SS1_ETM_DCMR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR1_MSS_R5SS1_ETM_DCMR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCMR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR2_MSS_R5SS1_ETM_DCMR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR2_MSS_R5SS1_ETM_DCMR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR2_MSS_R5SS1_ETM_DCMR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR2_MSS_R5SS1_ETM_DCMR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCMR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR3_MSS_R5SS1_ETM_DCMR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR3_MSS_R5SS1_ETM_DCMR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR3_MSS_R5SS1_ETM_DCMR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR3_MSS_R5SS1_ETM_DCMR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCMR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR4_MSS_R5SS1_ETM_DCMR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR4_MSS_R5SS1_ETM_DCMR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR4_MSS_R5SS1_ETM_DCMR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR4_MSS_R5SS1_ETM_DCMR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCMR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR5_MSS_R5SS1_ETM_DCMR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR5_MSS_R5SS1_ETM_DCMR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR5_MSS_R5SS1_ETM_DCMR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR5_MSS_R5SS1_ETM_DCMR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCMR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR6_MSS_R5SS1_ETM_DCMR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR6_MSS_R5SS1_ETM_DCMR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR6_MSS_R5SS1_ETM_DCMR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR6_MSS_R5SS1_ETM_DCMR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCMR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR7_MSS_R5SS1_ETM_DCMR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR7_MSS_R5SS1_ETM_DCMR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR7_MSS_R5SS1_ETM_DCMR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR7_MSS_R5SS1_ETM_DCMR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DCMR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR8_MSS_R5SS1_ETM_DCMR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR8_MSS_R5SS1_ETM_DCMR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR8_MSS_R5SS1_ETM_DCMR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR8_MSS_R5SS1_ETM_DCMR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DCMR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTRLDVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR1_MSS_R5SS1_ETM_CNTRLDVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR1_MSS_R5SS1_ETM_CNTRLDVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR1_MSS_R5SS1_ETM_CNTRLDVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR1_MSS_R5SS1_ETM_CNTRLDVR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR1_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTRLDVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR2_MSS_R5SS1_ETM_CNTRLDVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR2_MSS_R5SS1_ETM_CNTRLDVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR2_MSS_R5SS1_ETM_CNTRLDVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR2_MSS_R5SS1_ETM_CNTRLDVR2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR2_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTRLDVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR3_MSS_R5SS1_ETM_CNTRLDVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR3_MSS_R5SS1_ETM_CNTRLDVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR3_MSS_R5SS1_ETM_CNTRLDVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR3_MSS_R5SS1_ETM_CNTRLDVR3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR3_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTRLDVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR4_MSS_R5SS1_ETM_CNTRLDVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR4_MSS_R5SS1_ETM_CNTRLDVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR4_MSS_R5SS1_ETM_CNTRLDVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR4_MSS_R5SS1_ETM_CNTRLDVR4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDVR4_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTENR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR1_MSS_R5SS1_ETM_CNTENR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR1_MSS_R5SS1_ETM_CNTENR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR1_MSS_R5SS1_ETM_CNTENR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR1_MSS_R5SS1_ETM_CNTENR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTENR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR2_MSS_R5SS1_ETM_CNTENR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR2_MSS_R5SS1_ETM_CNTENR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR2_MSS_R5SS1_ETM_CNTENR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR2_MSS_R5SS1_ETM_CNTENR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTENR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR3_MSS_R5SS1_ETM_CNTENR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR3_MSS_R5SS1_ETM_CNTENR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR3_MSS_R5SS1_ETM_CNTENR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR3_MSS_R5SS1_ETM_CNTENR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTENR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR4_MSS_R5SS1_ETM_CNTENR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR4_MSS_R5SS1_ETM_CNTENR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR4_MSS_R5SS1_ETM_CNTENR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR4_MSS_R5SS1_ETM_CNTENR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTENR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTRLDEVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR1_MSS_R5SS1_ETM_CNTRLDEVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR1_MSS_R5SS1_ETM_CNTRLDEVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR1_MSS_R5SS1_ETM_CNTRLDEVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR1_MSS_R5SS1_ETM_CNTRLDEVR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR1_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTRLDEVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR2_MSS_R5SS1_ETM_CNTRLDEVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR2_MSS_R5SS1_ETM_CNTRLDEVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR2_MSS_R5SS1_ETM_CNTRLDEVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR2_MSS_R5SS1_ETM_CNTRLDEVR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR2_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTRLDEVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR3_MSS_R5SS1_ETM_CNTRLDEVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR3_MSS_R5SS1_ETM_CNTRLDEVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR3_MSS_R5SS1_ETM_CNTRLDEVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR3_MSS_R5SS1_ETM_CNTRLDEVR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR3_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTRLDEVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR4_MSS_R5SS1_ETM_CNTRLDEVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR4_MSS_R5SS1_ETM_CNTRLDEVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR4_MSS_R5SS1_ETM_CNTRLDEVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR4_MSS_R5SS1_ETM_CNTRLDEVR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTRLDEVR4_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR1_MSS_R5SS1_ETM_CNTVR1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR1_MSS_R5SS1_ETM_CNTVR1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR1_MSS_R5SS1_ETM_CNTVR1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR1_MSS_R5SS1_ETM_CNTVR1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR1_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR2_MSS_R5SS1_ETM_CNTVR2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR2_MSS_R5SS1_ETM_CNTVR2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR2_MSS_R5SS1_ETM_CNTVR2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR2_MSS_R5SS1_ETM_CNTVR2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR2_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR3_MSS_R5SS1_ETM_CNTVR3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR3_MSS_R5SS1_ETM_CNTVR3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR3_MSS_R5SS1_ETM_CNTVR3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR3_MSS_R5SS1_ETM_CNTVR3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR3_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_CNTVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR4_MSS_R5SS1_ETM_CNTVR4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR4_MSS_R5SS1_ETM_CNTVR4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR4_MSS_R5SS1_ETM_CNTVR4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR4_MSS_R5SS1_ETM_CNTVR4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CNTVR4_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_SQ12EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ12EVR_MSS_R5SS1_ETM_SQ12EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ12EVR_MSS_R5SS1_ETM_SQ12EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ12EVR_MSS_R5SS1_ETM_SQ12EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ12EVR_MSS_R5SS1_ETM_SQ12EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ12EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_SQ21EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ21EVR_MSS_R5SS1_ETM_SQ21EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ21EVR_MSS_R5SS1_ETM_SQ21EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ21EVR_MSS_R5SS1_ETM_SQ21EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ21EVR_MSS_R5SS1_ETM_SQ21EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ21EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_SQ23EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ23EVR_MSS_R5SS1_ETM_SQ23EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ23EVR_MSS_R5SS1_ETM_SQ23EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ23EVR_MSS_R5SS1_ETM_SQ23EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ23EVR_MSS_R5SS1_ETM_SQ23EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ23EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_SQ31EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ31EVR_MSS_R5SS1_ETM_SQ31EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ31EVR_MSS_R5SS1_ETM_SQ31EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ31EVR_MSS_R5SS1_ETM_SQ31EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ31EVR_MSS_R5SS1_ETM_SQ31EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ31EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_SQ32EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ32EVR_MSS_R5SS1_ETM_SQ32EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ32EVR_MSS_R5SS1_ETM_SQ32EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ32EVR_MSS_R5SS1_ETM_SQ32EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ32EVR_MSS_R5SS1_ETM_SQ32EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ32EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_SQ13EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ13EVR_MSS_R5SS1_ETM_SQ13EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ13EVR_MSS_R5SS1_ETM_SQ13EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ13EVR_MSS_R5SS1_ETM_SQ13EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ13EVR_MSS_R5SS1_ETM_SQ13EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQ13EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_SQR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQR_MSS_R5SS1_ETM_SQR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQR_MSS_R5SS1_ETM_SQR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQR_MSS_R5SS1_ETM_SQR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQR_MSS_R5SS1_ETM_SQR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SQR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_A_ETM_EXTOUTEVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR1_MSS_R5SS1_ETM_EXTOUTEVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR1_MSS_R5SS1_ETM_EXTOUTEVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR1_MSS_R5SS1_ETM_EXTOUTEVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR1_MSS_R5SS1_ETM_EXTOUTEVR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR1_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_EXTOUTEVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR2_MSS_R5SS1_ETM_EXTOUTEVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR2_MSS_R5SS1_ETM_EXTOUTEVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR2_MSS_R5SS1_ETM_EXTOUTEVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR2_MSS_R5SS1_ETM_EXTOUTEVR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR2_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_EXTOUTEVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR3_MSS_R5SS1_ETM_EXTOUTEVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR3_MSS_R5SS1_ETM_EXTOUTEVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR3_MSS_R5SS1_ETM_EXTOUTEVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR3_MSS_R5SS1_ETM_EXTOUTEVR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR3_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_EXTOUTEVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR4_MSS_R5SS1_ETM_EXTOUTEVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR4_MSS_R5SS1_ETM_EXTOUTEVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR4_MSS_R5SS1_ETM_EXTOUTEVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR4_MSS_R5SS1_ETM_EXTOUTEVR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTOUTEVR4_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_CIDCVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR1_MSS_R5SS1_ETM_CIDCVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR1_MSS_R5SS1_ETM_CIDCVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR1_MSS_R5SS1_ETM_CIDCVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR1_MSS_R5SS1_ETM_CIDCVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_CIDCVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR2_MSS_R5SS1_ETM_CIDCVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR2_MSS_R5SS1_ETM_CIDCVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR2_MSS_R5SS1_ETM_CIDCVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR2_MSS_R5SS1_ETM_CIDCVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_CIDCVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR3_MSS_R5SS1_ETM_CIDCVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR3_MSS_R5SS1_ETM_CIDCVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR3_MSS_R5SS1_ETM_CIDCVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR3_MSS_R5SS1_ETM_CIDCVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_CIDCMR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCMR_MSS_R5SS1_ETM_CIDCMR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCMR_MSS_R5SS1_ETM_CIDCMR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCMR_MSS_R5SS1_ETM_CIDCMR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCMR_MSS_R5SS1_ETM_CIDCMR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDCMR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_SYNCFR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SYNCFR_MSS_R5SS1_ETM_SYNCFR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SYNCFR_MSS_R5SS1_ETM_SYNCFR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SYNCFR_MSS_R5SS1_ETM_SYNCFR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SYNCFR_MSS_R5SS1_ETM_SYNCFR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_SYNCFR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_IDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_IDR_MSS_R5SS1_ETM_IDR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_IDR_MSS_R5SS1_ETM_IDR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_IDR_MSS_R5SS1_ETM_IDR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_IDR_MSS_R5SS1_ETM_IDR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_IDR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_A_ETM_CCER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCER_MSS_R5SS1_ETM_CCER_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCER_MSS_R5SS1_ETM_CCER_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCER_MSS_R5SS1_ETM_CCER_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCER_MSS_R5SS1_ETM_CCER_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CCER_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_A_ETM_EXTINSELR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTINSELR_MSS_R5SS1_ETM_EXTINSELR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTINSELR_MSS_R5SS1_ETM_EXTINSELR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTINSELR_MSS_R5SS1_ETM_EXTINSELR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTINSELR_MSS_R5SS1_ETM_EXTINSELR_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_EXTINSELR_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_TRACEIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRACEIDR_MSS_R5SS1_ETM_TRACEIDR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRACEIDR_MSS_R5SS1_ETM_TRACEIDR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRACEIDR_MSS_R5SS1_ETM_TRACEIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRACEIDR_MSS_R5SS1_ETM_TRACEIDR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_TRACEIDR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_ETM_PDSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PDSR_MSS_R5SS1_ETM_PDSR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PDSR_MSS_R5SS1_ETM_PDSR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PDSR_MSS_R5SS1_ETM_PDSR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PDSR_MSS_R5SS1_ETM_PDSR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PDSR_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_A_ETM_ITETMIF */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITETMIF_MSS_R5SS1_ETM_ITETMIF_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITETMIF_MSS_R5SS1_ETM_ITETMIF_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITETMIF_MSS_R5SS1_ETM_ITETMIF_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITETMIF_MSS_R5SS1_ETM_ITETMIF_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITETMIF_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_ITMISCOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCOUT_MSS_R5SS1_ETM_ITMISCOUT_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCOUT_MSS_R5SS1_ETM_ITMISCOUT_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCOUT_MSS_R5SS1_ETM_ITMISCOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCOUT_MSS_R5SS1_ETM_ITMISCOUT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCOUT_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_ITMISCIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCIN_MSS_R5SS1_ETM_ITMISCIN_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCIN_MSS_R5SS1_ETM_ITMISCIN_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCIN_MSS_R5SS1_ETM_ITMISCIN_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCIN_MSS_R5SS1_ETM_ITMISCIN_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITMISCIN_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_ETM_ITTRIGGERACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERACK_MSS_R5SS1_ETM_ITTRIGGERACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERACK_MSS_R5SS1_ETM_ITTRIGGERACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERACK_MSS_R5SS1_ETM_ITTRIGGERACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERACK_MSS_R5SS1_ETM_ITTRIGGERACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERACK_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_ETM_ITTRIGGERREQ */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERREQ_MSS_R5SS1_ETM_ITTRIGGERREQ_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERREQ_MSS_R5SS1_ETM_ITTRIGGERREQ_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERREQ_MSS_R5SS1_ETM_ITTRIGGERREQ_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERREQ_MSS_R5SS1_ETM_ITTRIGGERREQ_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITTRIGGERREQ_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_A_ETM_ITATBDATA0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBDATA0_MSS_R5SS1_ETM_ITATBDATA0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBDATA0_MSS_R5SS1_ETM_ITATBDATA0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBDATA0_MSS_R5SS1_ETM_ITATBDATA0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBDATA0_MSS_R5SS1_ETM_ITATBDATA0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBDATA0_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_ITATBCTR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR2_MSS_R5SS1_ETM_ITATBCTR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR2_MSS_R5SS1_ETM_ITATBCTR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR2_MSS_R5SS1_ETM_ITATBCTR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR2_MSS_R5SS1_ETM_ITATBCTR2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR2_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_ITATBCTR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR1_MSS_R5SS1_ETM_ITATBCTR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR1_MSS_R5SS1_ETM_ITATBCTR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR1_MSS_R5SS1_ETM_ITATBCTR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR1_MSS_R5SS1_ETM_ITATBCTR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR1_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_ITATBCTR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR0_MSS_R5SS1_ETM_ITATBCTR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR0_MSS_R5SS1_ETM_ITATBCTR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR0_MSS_R5SS1_ETM_ITATBCTR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR0_MSS_R5SS1_ETM_ITATBCTR0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITATBCTR0_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_A_ETM_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITCTRL_MSS_R5SS1_ETM_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITCTRL_MSS_R5SS1_ETM_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITCTRL_MSS_R5SS1_ETM_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITCTRL_MSS_R5SS1_ETM_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_ITCTRL_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_A_ETM_CLAIMSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMSET_MSS_R5SS1_ETM_CLAIMSET_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMSET_MSS_R5SS1_ETM_CLAIMSET_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMSET_MSS_R5SS1_ETM_CLAIMSET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMSET_MSS_R5SS1_ETM_CLAIMSET_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMSET_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_ETM_CLAIMCLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMCLR_MSS_R5SS1_ETM_CLAIMCLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMCLR_MSS_R5SS1_ETM_CLAIMCLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMCLR_MSS_R5SS1_ETM_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMCLR_MSS_R5SS1_ETM_CLAIMCLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CLAIMCLR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_A_ETM_LAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LAR_MSS_R5SS1_ETM_LAR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LAR_MSS_R5SS1_ETM_LAR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LAR_MSS_R5SS1_ETM_LAR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LAR_MSS_R5SS1_ETM_LAR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LAR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_A_ETM_LSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LSR_MSS_R5SS1_ETM_LSR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LSR_MSS_R5SS1_ETM_LSR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LSR_MSS_R5SS1_ETM_LSR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LSR_MSS_R5SS1_ETM_LSR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_LSR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_A_ETM_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_AUTHSTATUS_MSS_R5SS1_ETM_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_AUTHSTATUS_MSS_R5SS1_ETM_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_AUTHSTATUS_MSS_R5SS1_ETM_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_AUTHSTATUS_MSS_R5SS1_ETM_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_AUTHSTATUS_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_A_ETM_DEVID */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVID_MSS_R5SS1_ETM_DEVID_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVID_MSS_R5SS1_ETM_DEVID_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVID_MSS_R5SS1_ETM_DEVID_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVID_MSS_R5SS1_ETM_DEVID_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVID_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_DEVTYPE */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVTYPE_MSS_R5SS1_ETM_DEVTYPE_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVTYPE_MSS_R5SS1_ETM_DEVTYPE_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVTYPE_MSS_R5SS1_ETM_DEVTYPE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVTYPE_MSS_R5SS1_ETM_DEVTYPE_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_DEVTYPE_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_A_ETM_PIDR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR4_MSS_R5SS1_ETM_PIDR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR4_MSS_R5SS1_ETM_PIDR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR4_MSS_R5SS1_ETM_PIDR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR4_MSS_R5SS1_ETM_PIDR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_PIDR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR5_MSS_R5SS1_ETM_PIDR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR5_MSS_R5SS1_ETM_PIDR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR5_MSS_R5SS1_ETM_PIDR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR5_MSS_R5SS1_ETM_PIDR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_PIDR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR6_MSS_R5SS1_ETM_PIDR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR6_MSS_R5SS1_ETM_PIDR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR6_MSS_R5SS1_ETM_PIDR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR6_MSS_R5SS1_ETM_PIDR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_PIDR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR7_MSS_R5SS1_ETM_PIDR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR7_MSS_R5SS1_ETM_PIDR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR7_MSS_R5SS1_ETM_PIDR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR7_MSS_R5SS1_ETM_PIDR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_PIDR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR0_MSS_R5SS1_ETM_PIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR0_MSS_R5SS1_ETM_PIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR0_MSS_R5SS1_ETM_PIDR0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR0_MSS_R5SS1_ETM_PIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR0_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_PIDR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR1_MSS_R5SS1_ETM_PIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR1_MSS_R5SS1_ETM_PIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR1_MSS_R5SS1_ETM_PIDR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR1_MSS_R5SS1_ETM_PIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_PIDR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR2_MSS_R5SS1_ETM_PIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR2_MSS_R5SS1_ETM_PIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR2_MSS_R5SS1_ETM_PIDR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR2_MSS_R5SS1_ETM_PIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_PIDR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR3_MSS_R5SS1_ETM_PIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR3_MSS_R5SS1_ETM_PIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR3_MSS_R5SS1_ETM_PIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR3_MSS_R5SS1_ETM_PIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_PIDR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_CIDR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR0_MSS_R5SS1_ETM_CIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR0_MSS_R5SS1_ETM_CIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR0_MSS_R5SS1_ETM_CIDR0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR0_MSS_R5SS1_ETM_CIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR0_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_CIDR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR1_MSS_R5SS1_ETM_CIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR1_MSS_R5SS1_ETM_CIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR1_MSS_R5SS1_ETM_CIDR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR1_MSS_R5SS1_ETM_CIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_CIDR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR2_MSS_R5SS1_ETM_CIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR2_MSS_R5SS1_ETM_CIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR2_MSS_R5SS1_ETM_CIDR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR2_MSS_R5SS1_ETM_CIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_A_ETM_CIDR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR3_MSS_R5SS1_ETM_CIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR3_MSS_R5SS1_ETM_CIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR3_MSS_R5SS1_ETM_CIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR3_MSS_R5SS1_ETM_CIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_A_ETM_CIDR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_CR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CR_MSS_R5SS1_ETM_CR_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CR_MSS_R5SS1_ETM_CR_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CR_MSS_R5SS1_ETM_CR_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CR_MSS_R5SS1_ETM_CR_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CR_RESETVAL                            (0x00000000U)

/* MSS_R5SS1_B_ETM_CCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCR_MSS_R5SS1_ETM_CCR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCR_MSS_R5SS1_ETM_CCR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCR_MSS_R5SS1_ETM_CCR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCR_MSS_R5SS1_ETM_CCR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_B_ETM_TRIGGER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRIGGER_MSS_R5SS1_ETM_TRIGGER_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRIGGER_MSS_R5SS1_ETM_TRIGGER_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRIGGER_MSS_R5SS1_ETM_TRIGGER_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRIGGER_MSS_R5SS1_ETM_TRIGGER_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRIGGER_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_ASICCTLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ASICCTLR_MSS_R5SS1_ETM_ASICCTLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ASICCTLR_MSS_R5SS1_ETM_ASICCTLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ASICCTLR_MSS_R5SS1_ETM_ASICCTLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ASICCTLR_MSS_R5SS1_ETM_ASICCTLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ASICCTLR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_ETM_SR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SR_MSS_R5SS1_ETM_SR_MASK               (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SR_MSS_R5SS1_ETM_SR_SHIFT              (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SR_MSS_R5SS1_ETM_SR_RESETVAL           (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SR_MSS_R5SS1_ETM_SR_MAX                (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SR_RESETVAL                            (0x00000000U)

/* MSS_R5SS1_B_ETM_SCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SCR_MSS_R5SS1_ETM_SCR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SCR_MSS_R5SS1_ETM_SCR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SCR_MSS_R5SS1_ETM_SCR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SCR_MSS_R5SS1_ETM_SCR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SCR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_B_ETM_TSSCR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TSSCR_MSS_R5SS1_ETM_TSSCR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TSSCR_MSS_R5SS1_ETM_TSSCR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TSSCR_MSS_R5SS1_ETM_TSSCR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TSSCR_MSS_R5SS1_ETM_TSSCR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TSSCR_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_TECR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR2_MSS_R5SS1_ETM_TECR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR2_MSS_R5SS1_ETM_TECR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR2_MSS_R5SS1_ETM_TECR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR2_MSS_R5SS1_ETM_TECR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_TEEVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TEEVR_MSS_R5SS1_ETM_TEEVR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TEEVR_MSS_R5SS1_ETM_TEEVR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TEEVR_MSS_R5SS1_ETM_TEEVR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TEEVR_MSS_R5SS1_ETM_TEEVR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TEEVR_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_TECR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR1_MSS_R5SS1_ETM_TECR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR1_MSS_R5SS1_ETM_TECR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR1_MSS_R5SS1_ETM_TECR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR1_MSS_R5SS1_ETM_TECR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TECR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_FFRR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFRR_MSS_R5SS1_ETM_FFRR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFRR_MSS_R5SS1_ETM_FFRR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFRR_MSS_R5SS1_ETM_FFRR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFRR_MSS_R5SS1_ETM_FFRR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFRR_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_B_ETM_FFLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFLR_MSS_R5SS1_ETM_FFLR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFLR_MSS_R5SS1_ETM_FFLR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFLR_MSS_R5SS1_ETM_FFLR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFLR_MSS_R5SS1_ETM_FFLR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_FFLR_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_B_ETM_VDEVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDEVR_MSS_R5SS1_ETM_VDEVR_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDEVR_MSS_R5SS1_ETM_VDEVR_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDEVR_MSS_R5SS1_ETM_VDEVR_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDEVR_MSS_R5SS1_ETM_VDEVR_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDEVR_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_VDCR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR1_MSS_R5SS1_ETM_VDCR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR1_MSS_R5SS1_ETM_VDCR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR1_MSS_R5SS1_ETM_VDCR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR1_MSS_R5SS1_ETM_VDCR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_VDCR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR2_MSS_R5SS1_ETM_VDCR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR2_MSS_R5SS1_ETM_VDCR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR2_MSS_R5SS1_ETM_VDCR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR2_MSS_R5SS1_ETM_VDCR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_VDCR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR3_MSS_R5SS1_ETM_VDCR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR3_MSS_R5SS1_ETM_VDCR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR3_MSS_R5SS1_ETM_VDCR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR3_MSS_R5SS1_ETM_VDCR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_VDCR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR1_MSS_R5SS1_ETM_ACVR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR1_MSS_R5SS1_ETM_ACVR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR1_MSS_R5SS1_ETM_ACVR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR1_MSS_R5SS1_ETM_ACVR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR2_MSS_R5SS1_ETM_ACVR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR2_MSS_R5SS1_ETM_ACVR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR2_MSS_R5SS1_ETM_ACVR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR2_MSS_R5SS1_ETM_ACVR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR3_MSS_R5SS1_ETM_ACVR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR3_MSS_R5SS1_ETM_ACVR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR3_MSS_R5SS1_ETM_ACVR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR3_MSS_R5SS1_ETM_ACVR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR4_MSS_R5SS1_ETM_ACVR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR4_MSS_R5SS1_ETM_ACVR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR4_MSS_R5SS1_ETM_ACVR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR4_MSS_R5SS1_ETM_ACVR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR5_MSS_R5SS1_ETM_ACVR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR5_MSS_R5SS1_ETM_ACVR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR5_MSS_R5SS1_ETM_ACVR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR5_MSS_R5SS1_ETM_ACVR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR6_MSS_R5SS1_ETM_ACVR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR6_MSS_R5SS1_ETM_ACVR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR6_MSS_R5SS1_ETM_ACVR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR6_MSS_R5SS1_ETM_ACVR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR7_MSS_R5SS1_ETM_ACVR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR7_MSS_R5SS1_ETM_ACVR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR7_MSS_R5SS1_ETM_ACVR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR7_MSS_R5SS1_ETM_ACVR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR8_MSS_R5SS1_ETM_ACVR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR8_MSS_R5SS1_ETM_ACVR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR8_MSS_R5SS1_ETM_ACVR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR8_MSS_R5SS1_ETM_ACVR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR9 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR9_MSS_R5SS1_ETM_ACVR9_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR9_MSS_R5SS1_ETM_ACVR9_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR9_MSS_R5SS1_ETM_ACVR9_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR9_MSS_R5SS1_ETM_ACVR9_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR9_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR10 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR10_MSS_R5SS1_ETM_ACVR10_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR10_MSS_R5SS1_ETM_ACVR10_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR10_MSS_R5SS1_ETM_ACVR10_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR10_MSS_R5SS1_ETM_ACVR10_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR10_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR11 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR11_MSS_R5SS1_ETM_ACVR11_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR11_MSS_R5SS1_ETM_ACVR11_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR11_MSS_R5SS1_ETM_ACVR11_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR11_MSS_R5SS1_ETM_ACVR11_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR11_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR12 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR12_MSS_R5SS1_ETM_ACVR12_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR12_MSS_R5SS1_ETM_ACVR12_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR12_MSS_R5SS1_ETM_ACVR12_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR12_MSS_R5SS1_ETM_ACVR12_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR12_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR13 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR13_MSS_R5SS1_ETM_ACVR13_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR13_MSS_R5SS1_ETM_ACVR13_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR13_MSS_R5SS1_ETM_ACVR13_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR13_MSS_R5SS1_ETM_ACVR13_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR13_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR14 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR14_MSS_R5SS1_ETM_ACVR14_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR14_MSS_R5SS1_ETM_ACVR14_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR14_MSS_R5SS1_ETM_ACVR14_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR14_MSS_R5SS1_ETM_ACVR14_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR14_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR15 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR15_MSS_R5SS1_ETM_ACVR15_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR15_MSS_R5SS1_ETM_ACVR15_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR15_MSS_R5SS1_ETM_ACVR15_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR15_MSS_R5SS1_ETM_ACVR15_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR15_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACVR16 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR16_MSS_R5SS1_ETM_ACVR16_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR16_MSS_R5SS1_ETM_ACVR16_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR16_MSS_R5SS1_ETM_ACVR16_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR16_MSS_R5SS1_ETM_ACVR16_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACVR16_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR1_MSS_R5SS1_ETM_ACTR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR1_MSS_R5SS1_ETM_ACTR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR1_MSS_R5SS1_ETM_ACTR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR1_MSS_R5SS1_ETM_ACTR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR2_MSS_R5SS1_ETM_ACTR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR2_MSS_R5SS1_ETM_ACTR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR2_MSS_R5SS1_ETM_ACTR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR2_MSS_R5SS1_ETM_ACTR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR3_MSS_R5SS1_ETM_ACTR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR3_MSS_R5SS1_ETM_ACTR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR3_MSS_R5SS1_ETM_ACTR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR3_MSS_R5SS1_ETM_ACTR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR4_MSS_R5SS1_ETM_ACTR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR4_MSS_R5SS1_ETM_ACTR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR4_MSS_R5SS1_ETM_ACTR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR4_MSS_R5SS1_ETM_ACTR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR5_MSS_R5SS1_ETM_ACTR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR5_MSS_R5SS1_ETM_ACTR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR5_MSS_R5SS1_ETM_ACTR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR5_MSS_R5SS1_ETM_ACTR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR6_MSS_R5SS1_ETM_ACTR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR6_MSS_R5SS1_ETM_ACTR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR6_MSS_R5SS1_ETM_ACTR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR6_MSS_R5SS1_ETM_ACTR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR7_MSS_R5SS1_ETM_ACTR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR7_MSS_R5SS1_ETM_ACTR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR7_MSS_R5SS1_ETM_ACTR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR7_MSS_R5SS1_ETM_ACTR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR8_MSS_R5SS1_ETM_ACTR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR8_MSS_R5SS1_ETM_ACTR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR8_MSS_R5SS1_ETM_ACTR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR8_MSS_R5SS1_ETM_ACTR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR9 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR9_MSS_R5SS1_ETM_ACTR9_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR9_MSS_R5SS1_ETM_ACTR9_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR9_MSS_R5SS1_ETM_ACTR9_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR9_MSS_R5SS1_ETM_ACTR9_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR9_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR10 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR10_MSS_R5SS1_ETM_ACTR10_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR10_MSS_R5SS1_ETM_ACTR10_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR10_MSS_R5SS1_ETM_ACTR10_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR10_MSS_R5SS1_ETM_ACTR10_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR10_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR11 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR11_MSS_R5SS1_ETM_ACTR11_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR11_MSS_R5SS1_ETM_ACTR11_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR11_MSS_R5SS1_ETM_ACTR11_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR11_MSS_R5SS1_ETM_ACTR11_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR11_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR12 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR12_MSS_R5SS1_ETM_ACTR12_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR12_MSS_R5SS1_ETM_ACTR12_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR12_MSS_R5SS1_ETM_ACTR12_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR12_MSS_R5SS1_ETM_ACTR12_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR12_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR13 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR13_MSS_R5SS1_ETM_ACTR13_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR13_MSS_R5SS1_ETM_ACTR13_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR13_MSS_R5SS1_ETM_ACTR13_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR13_MSS_R5SS1_ETM_ACTR13_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR13_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR14 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR14_MSS_R5SS1_ETM_ACTR14_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR14_MSS_R5SS1_ETM_ACTR14_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR14_MSS_R5SS1_ETM_ACTR14_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR14_MSS_R5SS1_ETM_ACTR14_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR14_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR15 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR15_MSS_R5SS1_ETM_ACTR15_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR15_MSS_R5SS1_ETM_ACTR15_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR15_MSS_R5SS1_ETM_ACTR15_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR15_MSS_R5SS1_ETM_ACTR15_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR15_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_ACTR16 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR16_MSS_R5SS1_ETM_ACTR16_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR16_MSS_R5SS1_ETM_ACTR16_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR16_MSS_R5SS1_ETM_ACTR16_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR16_MSS_R5SS1_ETM_ACTR16_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ACTR16_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_DCVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR1_MSS_R5SS1_ETM_DCVR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR1_MSS_R5SS1_ETM_DCVR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR1_MSS_R5SS1_ETM_DCVR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR1_MSS_R5SS1_ETM_DCVR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR2_MSS_R5SS1_ETM_DCVR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR2_MSS_R5SS1_ETM_DCVR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR2_MSS_R5SS1_ETM_DCVR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR2_MSS_R5SS1_ETM_DCVR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR3_MSS_R5SS1_ETM_DCVR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR3_MSS_R5SS1_ETM_DCVR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR3_MSS_R5SS1_ETM_DCVR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR3_MSS_R5SS1_ETM_DCVR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR4_MSS_R5SS1_ETM_DCVR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR4_MSS_R5SS1_ETM_DCVR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR4_MSS_R5SS1_ETM_DCVR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR4_MSS_R5SS1_ETM_DCVR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCVR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR5_MSS_R5SS1_ETM_DCVR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR5_MSS_R5SS1_ETM_DCVR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR5_MSS_R5SS1_ETM_DCVR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR5_MSS_R5SS1_ETM_DCVR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCVR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR6_MSS_R5SS1_ETM_DCVR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR6_MSS_R5SS1_ETM_DCVR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR6_MSS_R5SS1_ETM_DCVR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR6_MSS_R5SS1_ETM_DCVR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCVR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR7_MSS_R5SS1_ETM_DCVR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR7_MSS_R5SS1_ETM_DCVR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR7_MSS_R5SS1_ETM_DCVR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR7_MSS_R5SS1_ETM_DCVR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCVR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR8_MSS_R5SS1_ETM_DCVR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR8_MSS_R5SS1_ETM_DCVR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR8_MSS_R5SS1_ETM_DCVR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR8_MSS_R5SS1_ETM_DCVR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCVR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCMR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR1_MSS_R5SS1_ETM_DCMR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR1_MSS_R5SS1_ETM_DCMR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR1_MSS_R5SS1_ETM_DCMR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR1_MSS_R5SS1_ETM_DCMR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCMR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR2_MSS_R5SS1_ETM_DCMR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR2_MSS_R5SS1_ETM_DCMR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR2_MSS_R5SS1_ETM_DCMR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR2_MSS_R5SS1_ETM_DCMR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCMR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR3_MSS_R5SS1_ETM_DCMR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR3_MSS_R5SS1_ETM_DCMR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR3_MSS_R5SS1_ETM_DCMR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR3_MSS_R5SS1_ETM_DCMR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCMR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR4_MSS_R5SS1_ETM_DCMR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR4_MSS_R5SS1_ETM_DCMR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR4_MSS_R5SS1_ETM_DCMR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR4_MSS_R5SS1_ETM_DCMR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCMR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR5_MSS_R5SS1_ETM_DCMR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR5_MSS_R5SS1_ETM_DCMR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR5_MSS_R5SS1_ETM_DCMR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR5_MSS_R5SS1_ETM_DCMR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCMR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR6_MSS_R5SS1_ETM_DCMR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR6_MSS_R5SS1_ETM_DCMR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR6_MSS_R5SS1_ETM_DCMR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR6_MSS_R5SS1_ETM_DCMR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCMR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR7_MSS_R5SS1_ETM_DCMR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR7_MSS_R5SS1_ETM_DCMR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR7_MSS_R5SS1_ETM_DCMR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR7_MSS_R5SS1_ETM_DCMR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DCMR8 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR8_MSS_R5SS1_ETM_DCMR8_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR8_MSS_R5SS1_ETM_DCMR8_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR8_MSS_R5SS1_ETM_DCMR8_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR8_MSS_R5SS1_ETM_DCMR8_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DCMR8_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTRLDVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR1_MSS_R5SS1_ETM_CNTRLDVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR1_MSS_R5SS1_ETM_CNTRLDVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR1_MSS_R5SS1_ETM_CNTRLDVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR1_MSS_R5SS1_ETM_CNTRLDVR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR1_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTRLDVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR2_MSS_R5SS1_ETM_CNTRLDVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR2_MSS_R5SS1_ETM_CNTRLDVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR2_MSS_R5SS1_ETM_CNTRLDVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR2_MSS_R5SS1_ETM_CNTRLDVR2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR2_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTRLDVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR3_MSS_R5SS1_ETM_CNTRLDVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR3_MSS_R5SS1_ETM_CNTRLDVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR3_MSS_R5SS1_ETM_CNTRLDVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR3_MSS_R5SS1_ETM_CNTRLDVR3_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR3_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTRLDVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR4_MSS_R5SS1_ETM_CNTRLDVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR4_MSS_R5SS1_ETM_CNTRLDVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR4_MSS_R5SS1_ETM_CNTRLDVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR4_MSS_R5SS1_ETM_CNTRLDVR4_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDVR4_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTENR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR1_MSS_R5SS1_ETM_CNTENR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR1_MSS_R5SS1_ETM_CNTENR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR1_MSS_R5SS1_ETM_CNTENR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR1_MSS_R5SS1_ETM_CNTENR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTENR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR2_MSS_R5SS1_ETM_CNTENR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR2_MSS_R5SS1_ETM_CNTENR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR2_MSS_R5SS1_ETM_CNTENR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR2_MSS_R5SS1_ETM_CNTENR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTENR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR3_MSS_R5SS1_ETM_CNTENR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR3_MSS_R5SS1_ETM_CNTENR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR3_MSS_R5SS1_ETM_CNTENR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR3_MSS_R5SS1_ETM_CNTENR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTENR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR4_MSS_R5SS1_ETM_CNTENR4_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR4_MSS_R5SS1_ETM_CNTENR4_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR4_MSS_R5SS1_ETM_CNTENR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR4_MSS_R5SS1_ETM_CNTENR4_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTENR4_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTRLDEVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR1_MSS_R5SS1_ETM_CNTRLDEVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR1_MSS_R5SS1_ETM_CNTRLDEVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR1_MSS_R5SS1_ETM_CNTRLDEVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR1_MSS_R5SS1_ETM_CNTRLDEVR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR1_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTRLDEVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR2_MSS_R5SS1_ETM_CNTRLDEVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR2_MSS_R5SS1_ETM_CNTRLDEVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR2_MSS_R5SS1_ETM_CNTRLDEVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR2_MSS_R5SS1_ETM_CNTRLDEVR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR2_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTRLDEVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR3_MSS_R5SS1_ETM_CNTRLDEVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR3_MSS_R5SS1_ETM_CNTRLDEVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR3_MSS_R5SS1_ETM_CNTRLDEVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR3_MSS_R5SS1_ETM_CNTRLDEVR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR3_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTRLDEVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR4_MSS_R5SS1_ETM_CNTRLDEVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR4_MSS_R5SS1_ETM_CNTRLDEVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR4_MSS_R5SS1_ETM_CNTRLDEVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR4_MSS_R5SS1_ETM_CNTRLDEVR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTRLDEVR4_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR1_MSS_R5SS1_ETM_CNTVR1_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR1_MSS_R5SS1_ETM_CNTVR1_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR1_MSS_R5SS1_ETM_CNTVR1_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR1_MSS_R5SS1_ETM_CNTVR1_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR1_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR2_MSS_R5SS1_ETM_CNTVR2_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR2_MSS_R5SS1_ETM_CNTVR2_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR2_MSS_R5SS1_ETM_CNTVR2_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR2_MSS_R5SS1_ETM_CNTVR2_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR2_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR3_MSS_R5SS1_ETM_CNTVR3_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR3_MSS_R5SS1_ETM_CNTVR3_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR3_MSS_R5SS1_ETM_CNTVR3_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR3_MSS_R5SS1_ETM_CNTVR3_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR3_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_CNTVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR4_MSS_R5SS1_ETM_CNTVR4_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR4_MSS_R5SS1_ETM_CNTVR4_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR4_MSS_R5SS1_ETM_CNTVR4_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR4_MSS_R5SS1_ETM_CNTVR4_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CNTVR4_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_SQ12EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ12EVR_MSS_R5SS1_ETM_SQ12EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ12EVR_MSS_R5SS1_ETM_SQ12EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ12EVR_MSS_R5SS1_ETM_SQ12EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ12EVR_MSS_R5SS1_ETM_SQ12EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ12EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_SQ21EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ21EVR_MSS_R5SS1_ETM_SQ21EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ21EVR_MSS_R5SS1_ETM_SQ21EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ21EVR_MSS_R5SS1_ETM_SQ21EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ21EVR_MSS_R5SS1_ETM_SQ21EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ21EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_SQ23EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ23EVR_MSS_R5SS1_ETM_SQ23EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ23EVR_MSS_R5SS1_ETM_SQ23EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ23EVR_MSS_R5SS1_ETM_SQ23EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ23EVR_MSS_R5SS1_ETM_SQ23EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ23EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_SQ31EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ31EVR_MSS_R5SS1_ETM_SQ31EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ31EVR_MSS_R5SS1_ETM_SQ31EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ31EVR_MSS_R5SS1_ETM_SQ31EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ31EVR_MSS_R5SS1_ETM_SQ31EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ31EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_SQ32EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ32EVR_MSS_R5SS1_ETM_SQ32EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ32EVR_MSS_R5SS1_ETM_SQ32EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ32EVR_MSS_R5SS1_ETM_SQ32EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ32EVR_MSS_R5SS1_ETM_SQ32EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ32EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_SQ13EVR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ13EVR_MSS_R5SS1_ETM_SQ13EVR_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ13EVR_MSS_R5SS1_ETM_SQ13EVR_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ13EVR_MSS_R5SS1_ETM_SQ13EVR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ13EVR_MSS_R5SS1_ETM_SQ13EVR_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQ13EVR_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_SQR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQR_MSS_R5SS1_ETM_SQR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQR_MSS_R5SS1_ETM_SQR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQR_MSS_R5SS1_ETM_SQR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQR_MSS_R5SS1_ETM_SQR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SQR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_B_ETM_EXTOUTEVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR1_MSS_R5SS1_ETM_EXTOUTEVR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR1_MSS_R5SS1_ETM_EXTOUTEVR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR1_MSS_R5SS1_ETM_EXTOUTEVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR1_MSS_R5SS1_ETM_EXTOUTEVR1_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR1_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_EXTOUTEVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR2_MSS_R5SS1_ETM_EXTOUTEVR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR2_MSS_R5SS1_ETM_EXTOUTEVR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR2_MSS_R5SS1_ETM_EXTOUTEVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR2_MSS_R5SS1_ETM_EXTOUTEVR2_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR2_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_EXTOUTEVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR3_MSS_R5SS1_ETM_EXTOUTEVR3_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR3_MSS_R5SS1_ETM_EXTOUTEVR3_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR3_MSS_R5SS1_ETM_EXTOUTEVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR3_MSS_R5SS1_ETM_EXTOUTEVR3_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR3_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_EXTOUTEVR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR4_MSS_R5SS1_ETM_EXTOUTEVR4_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR4_MSS_R5SS1_ETM_EXTOUTEVR4_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR4_MSS_R5SS1_ETM_EXTOUTEVR4_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR4_MSS_R5SS1_ETM_EXTOUTEVR4_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTOUTEVR4_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_CIDCVR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR1_MSS_R5SS1_ETM_CIDCVR1_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR1_MSS_R5SS1_ETM_CIDCVR1_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR1_MSS_R5SS1_ETM_CIDCVR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR1_MSS_R5SS1_ETM_CIDCVR1_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR1_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_CIDCVR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR2_MSS_R5SS1_ETM_CIDCVR2_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR2_MSS_R5SS1_ETM_CIDCVR2_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR2_MSS_R5SS1_ETM_CIDCVR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR2_MSS_R5SS1_ETM_CIDCVR2_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR2_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_CIDCVR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR3_MSS_R5SS1_ETM_CIDCVR3_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR3_MSS_R5SS1_ETM_CIDCVR3_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR3_MSS_R5SS1_ETM_CIDCVR3_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR3_MSS_R5SS1_ETM_CIDCVR3_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCVR3_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_CIDCMR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCMR_MSS_R5SS1_ETM_CIDCMR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCMR_MSS_R5SS1_ETM_CIDCMR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCMR_MSS_R5SS1_ETM_CIDCMR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCMR_MSS_R5SS1_ETM_CIDCMR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDCMR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_SYNCFR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SYNCFR_MSS_R5SS1_ETM_SYNCFR_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SYNCFR_MSS_R5SS1_ETM_SYNCFR_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SYNCFR_MSS_R5SS1_ETM_SYNCFR_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SYNCFR_MSS_R5SS1_ETM_SYNCFR_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_SYNCFR_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_IDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_IDR_MSS_R5SS1_ETM_IDR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_IDR_MSS_R5SS1_ETM_IDR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_IDR_MSS_R5SS1_ETM_IDR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_IDR_MSS_R5SS1_ETM_IDR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_IDR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_B_ETM_CCER */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCER_MSS_R5SS1_ETM_CCER_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCER_MSS_R5SS1_ETM_CCER_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCER_MSS_R5SS1_ETM_CCER_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCER_MSS_R5SS1_ETM_CCER_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CCER_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_B_ETM_EXTINSELR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTINSELR_MSS_R5SS1_ETM_EXTINSELR_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTINSELR_MSS_R5SS1_ETM_EXTINSELR_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTINSELR_MSS_R5SS1_ETM_EXTINSELR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTINSELR_MSS_R5SS1_ETM_EXTINSELR_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_EXTINSELR_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_TRACEIDR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRACEIDR_MSS_R5SS1_ETM_TRACEIDR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRACEIDR_MSS_R5SS1_ETM_TRACEIDR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRACEIDR_MSS_R5SS1_ETM_TRACEIDR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRACEIDR_MSS_R5SS1_ETM_TRACEIDR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_TRACEIDR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_ETM_PDSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PDSR_MSS_R5SS1_ETM_PDSR_MASK           (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PDSR_MSS_R5SS1_ETM_PDSR_SHIFT          (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PDSR_MSS_R5SS1_ETM_PDSR_RESETVAL       (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PDSR_MSS_R5SS1_ETM_PDSR_MAX            (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PDSR_RESETVAL                          (0x00000000U)

/* MSS_R5SS1_B_ETM_ITETMIF */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITETMIF_MSS_R5SS1_ETM_ITETMIF_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITETMIF_MSS_R5SS1_ETM_ITETMIF_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITETMIF_MSS_R5SS1_ETM_ITETMIF_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITETMIF_MSS_R5SS1_ETM_ITETMIF_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITETMIF_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_ITMISCOUT */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCOUT_MSS_R5SS1_ETM_ITMISCOUT_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCOUT_MSS_R5SS1_ETM_ITMISCOUT_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCOUT_MSS_R5SS1_ETM_ITMISCOUT_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCOUT_MSS_R5SS1_ETM_ITMISCOUT_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCOUT_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_ITMISCIN */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCIN_MSS_R5SS1_ETM_ITMISCIN_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCIN_MSS_R5SS1_ETM_ITMISCIN_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCIN_MSS_R5SS1_ETM_ITMISCIN_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCIN_MSS_R5SS1_ETM_ITMISCIN_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITMISCIN_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_ETM_ITTRIGGERACK */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERACK_MSS_R5SS1_ETM_ITTRIGGERACK_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERACK_MSS_R5SS1_ETM_ITTRIGGERACK_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERACK_MSS_R5SS1_ETM_ITTRIGGERACK_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERACK_MSS_R5SS1_ETM_ITTRIGGERACK_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERACK_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_ETM_ITTRIGGERREQ */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERREQ_MSS_R5SS1_ETM_ITTRIGGERREQ_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERREQ_MSS_R5SS1_ETM_ITTRIGGERREQ_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERREQ_MSS_R5SS1_ETM_ITTRIGGERREQ_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERREQ_MSS_R5SS1_ETM_ITTRIGGERREQ_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITTRIGGERREQ_RESETVAL                  (0x00000000U)

/* MSS_R5SS1_B_ETM_ITATBDATA0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBDATA0_MSS_R5SS1_ETM_ITATBDATA0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBDATA0_MSS_R5SS1_ETM_ITATBDATA0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBDATA0_MSS_R5SS1_ETM_ITATBDATA0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBDATA0_MSS_R5SS1_ETM_ITATBDATA0_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBDATA0_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_ITATBCTR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR2_MSS_R5SS1_ETM_ITATBCTR2_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR2_MSS_R5SS1_ETM_ITATBCTR2_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR2_MSS_R5SS1_ETM_ITATBCTR2_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR2_MSS_R5SS1_ETM_ITATBCTR2_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR2_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_ITATBCTR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR1_MSS_R5SS1_ETM_ITATBCTR1_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR1_MSS_R5SS1_ETM_ITATBCTR1_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR1_MSS_R5SS1_ETM_ITATBCTR1_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR1_MSS_R5SS1_ETM_ITATBCTR1_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR1_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_ITATBCTR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR0_MSS_R5SS1_ETM_ITATBCTR0_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR0_MSS_R5SS1_ETM_ITATBCTR0_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR0_MSS_R5SS1_ETM_ITATBCTR0_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR0_MSS_R5SS1_ETM_ITATBCTR0_MAX  (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITATBCTR0_RESETVAL                     (0x00000000U)

/* MSS_R5SS1_B_ETM_ITCTRL */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITCTRL_MSS_R5SS1_ETM_ITCTRL_MASK       (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITCTRL_MSS_R5SS1_ETM_ITCTRL_SHIFT      (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITCTRL_MSS_R5SS1_ETM_ITCTRL_RESETVAL   (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITCTRL_MSS_R5SS1_ETM_ITCTRL_MAX        (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_ITCTRL_RESETVAL                        (0x00000000U)

/* MSS_R5SS1_B_ETM_CLAIMSET */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMSET_MSS_R5SS1_ETM_CLAIMSET_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMSET_MSS_R5SS1_ETM_CLAIMSET_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMSET_MSS_R5SS1_ETM_CLAIMSET_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMSET_MSS_R5SS1_ETM_CLAIMSET_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMSET_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_ETM_CLAIMCLR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMCLR_MSS_R5SS1_ETM_CLAIMCLR_MASK   (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMCLR_MSS_R5SS1_ETM_CLAIMCLR_SHIFT  (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMCLR_MSS_R5SS1_ETM_CLAIMCLR_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMCLR_MSS_R5SS1_ETM_CLAIMCLR_MAX    (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CLAIMCLR_RESETVAL                      (0x00000000U)

/* MSS_R5SS1_B_ETM_LAR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LAR_MSS_R5SS1_ETM_LAR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LAR_MSS_R5SS1_ETM_LAR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LAR_MSS_R5SS1_ETM_LAR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LAR_MSS_R5SS1_ETM_LAR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LAR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_B_ETM_LSR */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LSR_MSS_R5SS1_ETM_LSR_MASK             (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LSR_MSS_R5SS1_ETM_LSR_SHIFT            (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LSR_MSS_R5SS1_ETM_LSR_RESETVAL         (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LSR_MSS_R5SS1_ETM_LSR_MAX              (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_LSR_RESETVAL                           (0x00000000U)

/* MSS_R5SS1_B_ETM_AUTHSTATUS */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_AUTHSTATUS_MSS_R5SS1_ETM_AUTHSTATUS_MASK (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_AUTHSTATUS_MSS_R5SS1_ETM_AUTHSTATUS_SHIFT (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_AUTHSTATUS_MSS_R5SS1_ETM_AUTHSTATUS_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_AUTHSTATUS_MSS_R5SS1_ETM_AUTHSTATUS_MAX (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_AUTHSTATUS_RESETVAL                    (0x00000000U)

/* MSS_R5SS1_B_ETM_DEVID */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVID_MSS_R5SS1_ETM_DEVID_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVID_MSS_R5SS1_ETM_DEVID_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVID_MSS_R5SS1_ETM_DEVID_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVID_MSS_R5SS1_ETM_DEVID_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVID_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_DEVTYPE */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVTYPE_MSS_R5SS1_ETM_DEVTYPE_MASK     (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVTYPE_MSS_R5SS1_ETM_DEVTYPE_SHIFT    (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVTYPE_MSS_R5SS1_ETM_DEVTYPE_RESETVAL (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVTYPE_MSS_R5SS1_ETM_DEVTYPE_MAX      (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_DEVTYPE_RESETVAL                       (0x00000000U)

/* MSS_R5SS1_B_ETM_PIDR4 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR4_MSS_R5SS1_ETM_PIDR4_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR4_MSS_R5SS1_ETM_PIDR4_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR4_MSS_R5SS1_ETM_PIDR4_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR4_MSS_R5SS1_ETM_PIDR4_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR4_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_PIDR5 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR5_MSS_R5SS1_ETM_PIDR5_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR5_MSS_R5SS1_ETM_PIDR5_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR5_MSS_R5SS1_ETM_PIDR5_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR5_MSS_R5SS1_ETM_PIDR5_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR5_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_PIDR6 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR6_MSS_R5SS1_ETM_PIDR6_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR6_MSS_R5SS1_ETM_PIDR6_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR6_MSS_R5SS1_ETM_PIDR6_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR6_MSS_R5SS1_ETM_PIDR6_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR6_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_PIDR7 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR7_MSS_R5SS1_ETM_PIDR7_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR7_MSS_R5SS1_ETM_PIDR7_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR7_MSS_R5SS1_ETM_PIDR7_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR7_MSS_R5SS1_ETM_PIDR7_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR7_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_PIDR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR0_MSS_R5SS1_ETM_PIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR0_MSS_R5SS1_ETM_PIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR0_MSS_R5SS1_ETM_PIDR0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR0_MSS_R5SS1_ETM_PIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR0_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_PIDR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR1_MSS_R5SS1_ETM_PIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR1_MSS_R5SS1_ETM_PIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR1_MSS_R5SS1_ETM_PIDR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR1_MSS_R5SS1_ETM_PIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_PIDR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR2_MSS_R5SS1_ETM_PIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR2_MSS_R5SS1_ETM_PIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR2_MSS_R5SS1_ETM_PIDR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR2_MSS_R5SS1_ETM_PIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_PIDR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR3_MSS_R5SS1_ETM_PIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR3_MSS_R5SS1_ETM_PIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR3_MSS_R5SS1_ETM_PIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR3_MSS_R5SS1_ETM_PIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_PIDR3_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_CIDR0 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR0_MSS_R5SS1_ETM_CIDR0_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR0_MSS_R5SS1_ETM_CIDR0_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR0_MSS_R5SS1_ETM_CIDR0_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR0_MSS_R5SS1_ETM_CIDR0_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR0_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_CIDR1 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR1_MSS_R5SS1_ETM_CIDR1_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR1_MSS_R5SS1_ETM_CIDR1_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR1_MSS_R5SS1_ETM_CIDR1_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR1_MSS_R5SS1_ETM_CIDR1_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR1_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_CIDR2 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR2_MSS_R5SS1_ETM_CIDR2_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR2_MSS_R5SS1_ETM_CIDR2_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR2_MSS_R5SS1_ETM_CIDR2_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR2_MSS_R5SS1_ETM_CIDR2_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR2_RESETVAL                         (0x00000000U)

/* MSS_R5SS1_B_ETM_CIDR3 */

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR3_MSS_R5SS1_ETM_CIDR3_MASK         (0xFFFFFFFFU)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR3_MSS_R5SS1_ETM_CIDR3_SHIFT        (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR3_MSS_R5SS1_ETM_CIDR3_RESETVAL     (0x00000000U)
#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR3_MSS_R5SS1_ETM_CIDR3_MAX          (0xFFFFFFFFU)

#define CSL_MSS_DEBUGSS_MSS_R5SS1_B_ETM_CIDR3_RESETVAL                         (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
