/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdbool.h>
#include <string.h>
#include "test_icss_emac_loopback_utils.h"
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <unity.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ICSS_EMAC_TEST_ETHER_TYPE 0x0806U
#define ICSS_EMAC_TEST_VLAN_ETH_TYPE 0x8100U
#define ICSS_EMAC_TEST_ETH_ALEN    0x6U
#define ICSS_EMAC_TEST_BYTESWAP16(x)    ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))

#define ICSS_EMAC_TEST_ARP_PKT_SIZE_PORT1        60
#define ICSS_EMAC_TEST_ARP_PKT_SIZE_PORT2        60
#define ICSS_EMAC_TEST_UDP_PKT_SIZE_PORT1        1500
#define ICSS_EMAC_TEST_UDP_PKT_SIZE_PORT2        1500

#define ICSS_EMAC_TEST_PKT_TX_COUNT 10
#define ICSS_EMAC_TEST_PKT_SIZE       42
#define ICSS_EMAC_TEST_QUEUE_OVERFLOW_PASS_CRITERIA_COUNT 14U
#define ICSS_EMAC_TEST_QUEUE_OVERFLOW_COUNT       110U

/*    TTS Macros    */
#define ICSS_EMAC_TEST_TTS_PERIOD_MARGIN             50
#define ICSS_EMAC_TEST_TTS_MAX_ACYC_PKT_COUNT        100
#define ICSS_EMAC_TEST_TTS_MAX_CYC_PKT_COUNT            10000

/* Number of VLAN test configurations */
#define ICSS_EMAC_TEST_MAX_NUM_VLAN_TEST_CFGS               (sizeof (testVlanFilterCfg)/sizeof (testVlanCfg_t))
#define ICSS_EMAC_TEST_MAX_VLAN_CMDS                        ( 4 )

/* VLAN filtering test macro defines for weight */
#define ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TAGGED_WEIGHT       10
#define ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_UNTAGGED_WEIGHT     20
#define ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PRIORITY_WEIGHT     40
#define ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_NONE                 0
#define ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TPU                 (ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TAGGED_WEIGHT + ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_UNTAGGED_WEIGHT + ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PRIORITY_WEIGHT)
#define ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TU                  (ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TAGGED_WEIGHT + ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_UNTAGGED_WEIGHT )
#define ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TP                  (ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TAGGED_WEIGHT + ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PRIORITY_WEIGHT )
#define ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PU                  (ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_UNTAGGED_WEIGHT + ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PRIORITY_WEIGHT )


/* VLAN filtering test macro defines for features */
#define ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_DISABLE            (0)
#define ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE             (1)

#define ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_UNTAGGED_HOST_RX     (2)
#define ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_UNTAGGED_HOST_RX (3)

#define ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_PRIO_HOST_RX         (4)
#define ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_PRIO_HOST_RX     (5)

#define ICSS_EMAC_VLAN_FILTER_TEST_VID_NOT_CONFIGURED         ( 6 )
#define ICSS_EMAC_VLAN_FILTER_TEST_VID_ADD_VLAN_ID            ( 7 )
#define ICSS_EMAC_VLAN_FILTER_TEST_VID_REMOVE_VLAN_ID         ( 8 )
#define ICSS_EMAC_VLAN_FILTER_TEST_VID                        ( 1 )

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* Test structures for VLAN filtering tests */
typedef struct testVlanCfg_s
{
    int32_t  vlanFilterEnable;
    int32_t  vlanAddRemoveCfg;
    int32_t  vid;
    int32_t  prioPktCfg;
    int32_t  unTagPktCfg;
    int32_t  expectedRxPktWeight;
}testVlanCfg_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void test_ICSS_EMAC_verify(void *args);

int32_t ICSS_EMAC_testGetPruStats(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle);

bool ICSS_EMAC_testMulticastFiltering_EmacPort1(ICSS_EMAC_Handle icss_emacHandle);

bool ICSS_EMAC_testMulticastFiltering_EmacPort2(ICSS_EMAC_Handle icss_emacHandle);

bool ICSS_EMAC_testVlanFiltering_On_EmacPort(PRUICSS_Handle *pruIcssHandlePtr, uint8_t portNum);

static bool ICSS_EMAC_vlan_filter_feature_config(ICSS_EMAC_Handle *icssEmacHandlePtr, PRUICSS_Handle *pruHandlePtr, uint8_t portNum, testVlanCfg_t *vlanCfg);

static bool ICSS_EMAC_firmware_feature_ctrl(ICSS_EMAC_Handle *eHandle, PRUICSS_Handle *hdl, uint32_t ioctlCmd, uint8_t portNum, ICSS_EMAC_IoctlCmd *ioctlParams);

static bool ICSS_EMAC_SendPktChkRx(ICSS_EMAC_Handle *icssEmacHandlePtr, PRUICSS_Handle *pruHandlePtr, uint32_t portNum, uint8_t *testVlanpkt, uint32_t pktSize);

bool ICSS_EMAC_testRxPktParser(ICSS_EMAC_Handle handle, void *pIcssRxPktInfo);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern ETHPHY_Handle gEthPhyHandle[2];

//static uint32_t ICSS_EMAC_testPruInstance2Done = 0;

/* Handle's to pru-icss instances */
extern PRUICSS_Handle ICSS_EMAC_testPruIcssHandle1;
extern PRUICSS_Handle ICSS_EMAC_testPruIcssHandle2;

/* Handle's to icss-emac  instances */
ICSS_EMAC_Handle ICSS_EMAC_testHandle = NULL;
ICSS_EMAC_Handle ICSS_EMAC_testHandle1 = NULL;
ICSS_EMAC_Handle ICSS_EMAC_testHandle2 = NULL;
ICSS_EMAC_Handle ICSS_EMAC_testHandle3 = NULL;

uint8_t ICSS_EMAC_testLclMac0[6] = {0x01, 0xb2, 0xc1, 0xd4, 0xe4, 0xff};
uint8_t ICSS_EMAC_testLclMac1[6]= {0x02, 0xb3, 0xc2, 0xd4, 0xe4, 0xff};;
uint8_t ICSS_EMAC_testLclMac2[6] = {0x02, 0xb4, 0xc3, 0xdd, 0xee, 0xff};
uint8_t ICSS_EMAC_testLclMac3[6] = {0x02, 0xb5, 0xc4, 0xd4, 0xe4, 0xff};

uint32_t ICSS_EMAC_testLinkIsrPru1Eth0 = 0;
uint32_t ICSS_EMAC_testLinkIsrPru1Eth1 = 0;
uint32_t ICSS_EMAC_testLinkIsrPru2Eth0 = 0;
uint32_t ICSS_EMAC_testLinkIsrPru2Eth1 = 0;

/*    TTS Global Variables    */
uint8_t    ICSS_EMAC_testTtsModePort1 = 0;
uint8_t    ICSS_EMAC_testTtsModePort2 = 0;
uint8_t    ICSS_EMAC_testTtsStartPort1 = 0;
uint8_t    ICSS_EMAC_testTtsStartPort2 = 0;

uint32_t ICSS_EMAC_testPacketRcvdPort0 = 0;
uint32_t ICSS_EMAC_testPacketRcvdPort1 = 0;
uint32_t ICSS_EMAC_testPacketRcvdPort2 = 0;
uint32_t ICSS_EMAC_testPacketRcvdPort3 = 0;

uint32_t ICSS_EMAC_testRecv_Q1PktCntPort1 = 0;
uint32_t ICSS_EMAC_testRecv_Q2PktCntPort1 = 0;
uint32_t ICSS_EMAC_testRecv_Q1PktCntPort2 = 0;
uint32_t ICSS_EMAC_testRecv_Q2PktCntPort2 = 0;

/* number of links which actually came up to test the interface */
static uint32_t  ICSS_EMAC_testLinkUpCount = 0;

uint32_t promisEnableFlag;

/* total number of packets received on all interfaces being tested, cummulative account */
uint32_t ICSS_EMAC_testTotalPktRcvd = 0;

SemaphoreP_Object    ICSS_EMAC_testTtsP1TxSem;
SemaphoreP_Object    ICSS_EMAC_testTtsP2TxSem;
SemaphoreP_Object    ICSS_EMAC_testTtsP1TimeSem;
SemaphoreP_Object    ICSS_EMAC_testTtsP2TimeSem;
SemaphoreP_Object    ICSS_EMAC_testTtsP1ResultSem;
SemaphoreP_Object    ICSS_EMAC_testTtsP2ResultSem;

uint8_t ICSS_EMAC_testTtsRecvPktPort1[ICSS_EMAC_MAXMTU] = {0};
uint8_t ICSS_EMAC_testTtsRecvPktPort2[ICSS_EMAC_MAXMTU] = {0};

uint8_t ICSS_EMAC_testPacketArrayInstance1[256] = {0};
uint8_t ICSS_EMAC_testPacketArrayInstance1_1[256] = {0};
uint8_t ICSS_EMAC_testPacketArrayInstance2[256] = {0};
uint8_t ICSS_EMAC_testPacketArrayInstance2_1[ICSS_EMAC_MAXMTU] = {0};

uint32_t ICSS_EMAC_testPacketTxCompletePort0 = 0;
uint32_t ICSS_EMAC_testPacketTxCompletePort1 = 0;
uint32_t ICSS_EMAC_testPacketTxCompletePort2 = 0;
uint32_t ICSS_EMAC_testPacketTxCompletePort3 = 0;

/* Flag to enable queue overflow check */
uint32_t enableQOverFlowCheck = 0;

/* DO NOT CHANGE ICSS_EMAC_testPkt UNLESS ICSS_EMAC_ICSS_EMAC_TEST_PKT_SIZE IS UPDATED */
uint8_t ICSS_EMAC_testPkt[] = {
    0x02, 0xb2, 0xc1, 0xd4, 0xe4, 0xff, /* broadcast mac */
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00,0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8,0x01, 0x02
};
uint8_t ICSS_EMAC_testPkt1[] = {
    0x02, 0xb3, 0xc2, 0xd4, 0xe4, 0xff, /* broadcast mac */
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00,0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8,0x01, 0x02
};
uint8_t ICSS_EMAC_testPkt2[] = {
    0x02, 0xb4, 0xc3, 0xdd, 0xee, 0xff, /* broadcast mac */
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00,0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8,0x01, 0x02
};
uint8_t ICSS_EMAC_testPkt3[] = {
    0x02, 0xb5, 0xc4, 0xd4, 0xe4, 0xff, /* broadcast mac */
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00,0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8,0x01, 0x02
};
uint8_t ICSS_EMAC_testPktPromiscuous[] = {
    0x02, 0xb0, 0xc3, 0xdd, 0xee, 0xff, /* broadcast mac */
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00,0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8,0x01, 0x02,
    0x0,0x1,0x2,0x3,
    0x4,0x5,0x6,0x7,
    0x8,0x9,0xa,0xb,
    0xc,0xd,0xe,0xf,
    0x10,0x11,0x12,0x13,
    0x14,0x15,0x16,0x17,
    0x18,0x19,0x1a,0x1b,
    0x1c,0x1d,0x1e,0x1f,
    0x20,0x21,0x22,0x23,
    0x24,0x25,0x26,0x27,
    0x28,0x29,0x2a,0x2b,
    0x2c,0x2d,0x2e,0x2f,
    0x30,0x31,0x32,0x33,
    0x34,0x35,0x36,0x37,
    0x38,0x39,0x3a,0x3b,
    0x3c,0x3d,0x3e,0x3f,
    0x40,0x41,0x42,0x43,
    0x44,0x45,0x46,0x47,
    0x48,0x49,0x4a,0x4b,
    0x4c,0x4d,0x4e,0x4f,
    0x50,0x51,0x52,0x53,
    0x54,0x55,0x56,0x57,
    0x58,0x59,0x5a,0x5b,
    0x5c,0x5d,0x5e,0x5f,
    0x60,0x61,0x62,0x63,
    0x64,0x65,0x66,0x67,
    0x68,0x69,0x6a,0x6b,
    0x6c,0x6d,0x6e,0x6f,
    0x70,0x71,0x72,0x73,
    0x74,0x75,0x76,0x77,
    0x78,0x79,0x7a,0x7b,
    0x7c,0x7d,0x7e,0x7f,
    0x80,0x81,0x82,0x83,
    0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x8b,
    0x8c,0x8d,0x8e,0x8f,
    0x90,0x91,0x92,0x93,
    0x94,0x95,0x96,0x97,
    0x98,0x99,0x9a,0x9b,
    0x9c,0x9d,0x9e,0x9f,
    0xa0,0xa1,0xa2,0xa3,
    0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xab,
    0xac,0xad,0xae,0xaf,
    0xb0,0xb1,0xb2,0xb3,
    0xb4,0xb5,0xb6,0xb7,
    0xb8,0xb9,0xba,0xbb,
    0xbc,0xbd,0xbe,0xbf,
    0xc0,0xc1,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,
    0xc8,0xc9,0xca,0xcb,
    0xcc,0xcd,0xce,0xcf,
    0xd0,0xd1,0xd2,0xd3,
    0xd4,0xd5,0xd6,0xd7,
    0xd8,0xd9,0xda,0xdb,
    0xdc,0xdd,0xde,0xdf,
    0xe0,0xe1,0xe2,0xe3,
    0xe4,0xe5,0xe6,0xe7,
    0xe8,0xe9,0xea,0xeb,
    0xec,0xed,0xee,0xef,
    0xf0,0xf1,0xf2,0xf3,
    0xf4,0xf5,0xf6,0xf7,
    0xf8,0xf9,0xfa,0xfb,
    0xfc,0xfd,0xfe,0xff,
    0x0,0x1,0x2,0x3,
    0x4,0x5,0x6,0x7,
    0x8,0x9,0xa,0xb,
    0xc,0xd,0xe,0xf,
    0x10,0x11,0x12,0x13,
    0x14,0x15,0x16,0x17,
    0x18,0x19,0x1a,0x1b,
    0x1c,0x1d,0x1e,0x1f,
    0x20,0x21,0x22,0x23,
    0x24,0x25,0x26,0x27,
    0x28,0x29,0x2a,0x2b,
    0x2c,0x2d,0x2e,0x2f,
    0x30,0x31,0x32,0x33,
    0x34,0x35,0x36,0x37,
    0x38,0x39,0x3a,0x3b,
    0x3c,0x3d,0x3e,0x3f,
    0x40,0x41,0x42,0x43,
    0x44,0x45,0x46,0x47,
    0x48,0x49,0x4a,0x4b,
    0x4c,0x4d,0x4e,0x4f,
    0x50,0x51,0x52,0x53,
    0x54,0x55,0x56,0x57,
    0x58,0x59,0x5a,0x5b,
    0x5c,0x5d,0x5e,0x5f,
    0x60,0x61,0x62,0x63,
    0x64,0x65,0x66,0x67,
    0x68,0x69,0x6a,0x6b,
    0x6c,0x6d,0x6e,0x6f,
    0x70,0x71,0x72,0x73,
    0x74,0x75,0x76,0x77,
    0x78,0x79,0x7a,0x7b,
    0x7c,0x7d,0x7e,0x7f,
    0x80,0x81,0x82,0x83,
    0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x8b,
    0x8c,0x8d,0x8e,0x8f,
    0x90,0x91,0x92,0x93,
    0x94,0x95,0x96,0x97,
    0x98,0x99,0x9a,0x9b,
    0x9c,0x9d,0x9e,0x9f,
    0xa0,0xa1,0xa2,0xa3,
    0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xab,
    0xac,0xad,0xae,0xaf,
    0xb0,0xb1,0xb2,0xb3,
    0xb4,0xb5,0xb6,0xb7,
    0xb8,0xb9,0xba,0xbb,
    0xbc,0xbd,0xbe,0xbf,
    0xc0,0xc1,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,
    0xc8,0xc9,0xca,0xcb,
    0xcc,0xcd,0xce,0xcf,
    0xd0,0xd1,0xd2,0xd3,
    0xd4,0xd5,0xd6,0xd7,
    0xd8,0xd9,0xda,0xdb,
    0xdc,0xdd,0xde,0xdf,
    0xe0,0xe1,0xe2,0xe3,
    0xe4,0xe5,0xe6,0xe7,
    0xe8,0xe9,0xea,0xeb,
    0xec,0xed,0xee,0xef,
    0xf0,0xf1,0xf2,0xf3,
    0xf4,0xf5,0xf6,0xf7,
    0xf8,0xf9,0xfa,0xfb,
    0xfc,0xfd,0xfe,0xff,
    0x0,0x1,0x2,0x3,
    0x4,0x5,0x6,0x7,
    0x8,0x9,0xa,0xb,
    0xc,0xd,0xe,0xf,
    0x10,0x11,0x12,0x13,
    0x14,0x15,0x16,0x17,
    0x18,0x19,0x1a,0x1b,
    0x1c,0x1d,0x1e,0x1f,
    0x20,0x21,0x22,0x23,
    0x24,0x25,0x26,0x27,
    0x28,0x29,0x2a,0x2b,
    0x2c,0x2d,0x2e,0x2f,
    0x30,0x31,0x32,0x33,
    0x34,0x35,0x36,0x37,
    0x38,0x39,0x3a,0x3b,
    0x3c,0x3d,0x3e,0x3f,
    0x40,0x41,0x42,0x43,
    0x44,0x45,0x46,0x47,
    0x48,0x49,0x4a,0x4b,
    0x4c,0x4d,0x4e,0x4f,
    0x50,0x51,0x52,0x53,
    0x54,0x55,0x56,0x57,
    0x58,0x59,0x5a,0x5b,
    0x5c,0x5d,0x5e,0x5f,
    0x60,0x61,0x62,0x63,
    0x64,0x65,0x66,0x67,
    0x68,0x69,0x6a,0x6b,
    0x6c,0x6d,0x6e,0x6f,
    0x70,0x71,0x72,0x73,
    0x74,0x75,0x76,0x77,
    0x78,0x79,0x7a,0x7b,
    0x7c,0x7d,0x7e,0x7f,
    0x80,0x81,0x82,0x83,
    0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x8b,
    0x8c,0x8d,0x8e,0x8f,
    0x90,0x91,0x92,0x93,
    0x94,0x95,0x96,0x97,
    0x98,0x99,0x9a,0x9b,
    0x9c,0x9d,0x9e,0x9f,
    0xa0,0xa1,0xa2,0xa3,
    0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xab,
    0xac,0xad,0xae,0xaf,
    0xb0,0xb1,0xb2,0xb3,
    0xb4,0xb5,0xb6,0xb7,
    0xb8,0xb9,0xba,0xbb,
    0xbc,0xbd,0xbe,0xbf,
    0xc0,0xc1,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,
    0xc8,0xc9,0xca,0xcb,
    0xcc,0xcd,0xce,0xcf,
    0xd0,0xd1,0xd2,0xd3,
    0xd4,0xd5,0xd6,0xd7,
    0xd8,0xd9,0xda,0xdb,
    0xdc,0xdd,0xde,0xdf,
    0xe0,0xe1,0xe2,0xe3,
    0xe4,0xe5,0xe6,0xe7,
    0xe8,0xe9,0xea,0xeb,
    0xec,0xed,0xee,0xef,
    0xf0,0xf1,0xf2,0xf3,
    0xf4,0xf5,0xf6,0xf7,
    0xf8,0xf9,0xfa,0xfb,
    0xfc,0xfd,0xfe,0xff,
    0x0,0x1,0x2,0x3,
    0x4,0x5,0x6,0x7,
    0x8,0x9,0xa,0xb,
    0xc,0xd,0xe,0xf,
    0x10,0x11,0x12,0x13,
    0x14,0x15,0x16,0x17,
    0x18,0x19,0x1a,0x1b,
    0x1c,0x1d,0x1e,0x1f,
    0x20,0x21,0x22,0x23,
    0x24,0x25,0x26,0x27,
    0x28,0x29,0x2a,0x2b,
    0x2c,0x2d,0x2e,0x2f,
    0x30,0x31,0x32,0x33,
    0x34,0x35,0x36,0x37,
    0x38,0x39,0x3a,0x3b,
    0x3c,0x3d,0x3e,0x3f,
    0x40,0x41,0x42,0x43,
    0x44,0x45,0x46,0x47,
    0x48,0x49,0x4a,0x4b,
    0x4c,0x4d,0x4e,0x4f,
    0x50,0x51,0x52,0x53,
    0x54,0x55,0x56,0x57,
    0x58,0x59,0x5a,0x5b,
    0x5c,0x5d,0x5e,0x5f,
    0x60,0x61,0x62,0x63,
    0x64,0x65,0x66,0x67,
    0x68,0x69,0x6a,0x6b,
    0x6c,0x6d,0x6e,0x6f,
    0x70,0x71,0x72,0x73,
    0x74,0x75,0x76,0x77,
    0x78,0x79,0x7a,0x7b,
    0x7c,0x7d,0x7e,0x7f,
    0x80,0x81,0x82,0x83,
    0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x8b,
    0x8c,0x8d,0x8e,0x8f,
    0x90,0x91,0x92,0x93,
    0x94,0x95,0x96,0x97,
    0x98,0x99,0x9a,0x9b,
    0x9c,0x9d,0x9e,0x9f,
    0xa0,0xa1,0xa2,0xa3,
    0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xab,
    0xac,0xad,0xae,0xaf,
    0xb0,0xb1,0xb2,0xb3,
    0xb4,0xb5,0xb6,0xb7,
    0xb8,0xb9,0xba,0xbb,
    0xbc,0xbd,0xbe,0xbf,
    0xc0,0xc1,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,
    0xc8,0xc9,0xca,0xcb,
    0xcc,0xcd,0xce,0xcf,
    0xd0,0xd1,0xd2,0xd3,
    0xd4,0xd5,0xd6,0xd7,
    0xd8,0xd9,0xda,0xdb,
    0xdc,0xdd,0xde,0xdf,
    0xe0,0xe1,0xe2,0xe3,
    0xe4,0xe5,0xe6,0xe7,
    0xe8,0xe9,0xea,0xeb,
    0xec,0xed,0xee,0xef,
    0xf0,0xf1,0xf2,0xf3,
    0xf4,0xf5,0xf6,0xf7,
    0xf8,0xf9,0xfa,0xfb,
    0xfc,0xfd,0xfe,0xff,
    0x0,0x1,0x2,0x3,
    0x4,0x5,0x6,0x7,
    0x8,0x9,0xa,0xb,
    0xc,0xd,0xe,0xf,
    0x10,0x11,0x12,0x13,
    0x14,0x15,0x16,0x17,
    0x18,0x19,0x1a,0x1b,
    0x1c,0x1d,0x1e,0x1f,
    0x20,0x21,0x22,0x23,
    0x24,0x25,0x26,0x27,
    0x28,0x29,0x2a,0x2b,
    0x2c,0x2d,0x2e,0x2f,
    0x30,0x31,0x32,0x33,
    0x34,0x35,0x36,0x37,
    0x38,0x39,0x3a,0x3b,
    0x3c,0x3d,0x3e,0x3f,
    0x40,0x41,0x42,0x43,
    0x44,0x45,0x46,0x47,
    0x48,0x49,0x4a,0x4b,
    0x4c,0x4d,0x4e,0x4f,
    0x50,0x51,0x52,0x53,
    0x54,0x55,0x56,0x57,
    0x58,0x59,0x5a,0x5b,
    0x5c,0x5d,0x5e,0x5f,
    0x60,0x61,0x62,0x63,
    0x64,0x65,0x66,0x67,
    0x68,0x69,0x6a,0x6b,
    0x6c,0x6d,0x6e,0x6f,
    0x70,0x71,0x72,0x73,
    0x74,0x75,0x76,0x77,
    0x78,0x79,0x7a,0x7b,
    0x7c,0x7d,0x7e,0x7f,
    0x80,0x81,0x82,0x83,
    0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x8b,
    0x8c,0x8d,0x8e,0x8f,
    0x90,0x91,0x92,0x93,
    0x94,0x95,0x96,0x97,
    0x98,0x99,0x9a,0x9b,
    0x9c,0x9d,0x9e,0x9f,
    0xa0,0xa1,0xa2,0xa3,
    0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xab,
    0xac,0xad,0xae,0xaf,
    0xb0,0xb1,0xb2,0xb3,
    0xb4,0xb5,0xb6,0xb7,
    0xb8,0xb9,0xba,0xbb,
    0xbc,0xbd,0xbe,0xbf,
    0xc0,0xc1,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,
    0xc8,0xc9,0xca,0xcb,
    0xcc,0xcd,0xce,0xcf,
    0xd0,0xd1,0xd2,0xd3,
    0xd4,0xd5,0xd6,0xd7,
    0xd8,0xd9,0xda,0xdb,
    0xdc,0xdd,0xde,0xdf,
    0xe0,0xe1,0xe2,0xe3,
    0xe4,0xe5,0xe6,0xe7,
    0xe8,0xe9,0xea,0xeb,
    0xec,0xed,0xee,0xef,
    0xf0,0xf1,0xf2,0xf3,
    0xf4,0xf5,0xf6,0xf7,
    0xf8,0xf9,0xfa,0xfb,
    0xfc,0xfd,0xfe,0xff,
    0x0,0x1,0x2,0x3,
    0x4,0x5,0x6,0x7,
    0x8,0x9,0xa,0xb,
    0xc,0xd,0xe,0xf,
    0x10,0x11,0x12,0x13,
    0x14,0x15,0x16,0x17,
    0x18,0x19,0x1a,0x1b,
    0x1c,0x1d,0x1e,0x1f,
    0x20,0x21,0x22,0x23,
    0x24,0x25,0x26,0x27,
    0x28,0x29,0x2a,0x2b,
    0x2c,0x2d,0x2e,0x2f,
    0x30,0x31,0x32,0x33,
    0x34,0x35,0x36,0x37,
    0x38,0x39,0x3a,0x3b,
    0x3c,0x3d,0x3e,0x3f,
    0x40,0x41,0x42,0x43,
    0x44,0x45,0x46,0x47,
    0x48,0x49,0x4a,0x4b,
    0x4c,0x4d,0x4e,0x4f,
    0x50,0x51,0x52,0x53,
    0x54,0x55,0x56,0x57,
    0x58,0x59,0x5a,0x5b,
    0x5c,0x5d,0x5e,0x5f,
    0x60,0x61,0x62,0x63,
    0x64,0x65,0x66,0x67,
    0x68,0x69,0x6a,0x6b,
    0x6c,0x6d,0x6e,0x6f,
    0x70,0x71,0x72,0x73,
    0x74,0x75,0x76,0x77,
    0x78,0x79,0x7a,0x7b,
    0x7c,0x7d,0x7e,0x7f,
    0x80,0x81,0x82,0x83,
    0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x8b,
    0x8c,0x8d,0x8e,0x8f,
    0x90,0x91,0x92,0x93,
    0x94,0x95,0x96,0x97,
    0x98,0x99,0x9a,0x9b,
    0x9c,0x9d,0x9e,0x9f,
    0xa0,0xa1,0xa2,0xa3,
    0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xab,
    0xac,0xad,0xae,0xaf,
    0xb0,0xb1,0xb2,0xb3,
    0xb4,0xb5,0xb6,0xb7,
    0xb8,0xb9,0xba,0xbb,
    0xbc,0xbd,0xbe,0xbf,
    0xc0,0xc1,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,
    0xc8,0xc9,0xca,0xcb,
    0xcc,0xcd,0xce,0xcf,
    0xd0,0xd1,0xd2,0xd3,
    0xd4,0xd5,0xd6,0xd7,
    0xd8,0xd9,0xda,0xdb,
};
/* DO NOT CHANGE ICSS_EMAC_testArpPktPort1 size UNLESS ICSS_EMAC_TEST_ARP_PKT_SIZE_PORT1 IS UPDATED */
uint8_t ICSS_EMAC_testArpPktPort1[ICSS_EMAC_TEST_ARP_PKT_SIZE_PORT1] = {
    0xc4, 0xbe, 0x84, 0xcc, 0xff, 0x1f,
    0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb,
    0x81, 0x00, 0xe0, 0x00,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* DO NOT CHANGE ICSS_EMAC_testArpPktPort2 size UNLESS ICSS_EMAC_TEST_ARP_PKT_SIZE_PORT2 IS UPDATED */
uint8_t ICSS_EMAC_testArpPktPort2[ICSS_EMAC_TEST_ARP_PKT_SIZE_PORT2] = {
    0xc4, 0xbe, 0x84, 0xcc, 0xff, 0x1f,
    0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb,
    0x81, 0x00, 0xe0, 0x00,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
/* DO NOT CHANGE ICSS_EMAC_testUdpPktPort1 size UNLESS ICSS_EMAC_TEST_UDP_PKT_SIZE_PORT1 IS UPDATED */
uint8_t ICSS_EMAC_testUdpPktPort1[ICSS_EMAC_TEST_UDP_PKT_SIZE_PORT1] = {
    0xc4, 0xbe, 0x84, 0xcc, 0xff, 0x1f,
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
    0x81, 0x00, 0x20, 0x00,
    0x08, 0x00, 0x45, 0x00,
    0x00, 0x2E, 0x00, 0x00, 0x40, 0x00,
    0x40, 0x11, 0xB7, 0x56, 0xC0, 0xA8,
    0x01, 0x16, 0xC0, 0xA8,
    0x01, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

/* DO NOT CHANGE ICSS_EMAC_testUdpPktPort2 size UNLESS ICSS_EMAC_TEST_UDP_PKT_SIZE_PORT2 IS UPDATED */
uint8_t ICSS_EMAC_testUdpPktPort2[ICSS_EMAC_TEST_UDP_PKT_SIZE_PORT2] = {
    0xc4, 0xbe, 0x84, 0xcc, 0xff, 0x1f,
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
    0x81, 0x00, 0x20, 0x00,
    0x08, 0x00, 0x45, 0x00,
    0x00, 0x2E, 0x00, 0x00, 0x40, 0x00,
    0x40, 0x11, 0xB7, 0x56, 0xC0, 0xA8,
    0x01, 0x16, 0xC0, 0xA8,
    0x01, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

uint8_t ICSS_EMAC_testPkt1Multicast[] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, /* multicast destination mac */
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00,0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8,0x01, 0x02
};

uint8_t ICSS_EMAC_testPkt2Multicast[] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x07, /* multicast destination mac */
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00,0x01,
    0x01, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0xc0, 0xa8, 0x01, 0x16,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc0, 0xa8,0x01, 0x02
};

/* Canned untagged packet for VLAN filtering test */
uint8_t ICSS_EMAC_testPkt1VlanUntagged[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xd4, 0xbe, 0xd9, 0x3d, 0xb6, 0xc1,
    0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xd4, 0xbe,
    0xd9, 0x3d, 0xb6, 0xc1, 0xc0, 0xa8, 0x03, 0x01,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xa8,
    0x03, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* Canned tagged packet for VLAN filtering test with VID = 1 */
uint8_t ICSS_EMAC_testPkt1VlanTagged[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xd4, 0xbe, 0xd9, 0x3d, 0xb6, 0xc1,
    0x81, 0x00, 0xa0, 0x01,
    0x08, 0x06, 0x00, 0x01, 0x08, 0x00, 0x06, 0x04,
    0x00, 0x01, 0xd4, 0xbe, 0xd9, 0x3d, 0xb6, 0xc1,
    0xc0, 0xa8, 0x03, 0x01, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xc0, 0xa8, 0x03, 0x0a, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

/* Canned priority packet for VLAN filtering test with VID = 0 */
uint8_t ICSS_EMAC_testPkt1VlanPriority[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xd4, 0xbe, 0xd9, 0x3d, 0xb6, 0xc1,
    0x81, 0x00, 0xa0, 0x00,
    0x08, 0x06, 0x00, 0x01, 0x08, 0x00, 0x06, 0x04,
    0x00, 0x01, 0xd4, 0xbe, 0xd9, 0x3d, 0xb6, 0xc1,
    0xc0, 0xa8, 0x03, 0x01, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xc0, 0xa8, 0x03, 0x0a, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

/*
Vlan filter Testing:
T : tagged frames
P : priority frames (VID=0)
U : untagged frames
Table:

==============================+===========+=================================+================================+=================+
ICSS_EMAC_FW_VLAN_FILTER_CTRL |           |  ICSS_EMAC_FW_VLAN_FILTER_      |  ICSS_EMAC_FW_VLAN_FILTER_     |     Result      |
_ENABLE_BIT                   |  VIDs (T) | PRIOTAG_HOST_RCV_ALLOW_CTRL_BIT |  UNTAG_HOST_RCV_ALLOW_CTRL_BIT |                 |
==============================+===========+=================================+================================+=================+
DISABLE                       | dont care |   dont care                     |     dont care                  | TPU             |
                              |           |                                 |                                | (All Rx by Host)|
==============================+===========+=================================+================================+=================+
ENABLE                        | NOT       |  VLAN_FLTR_PRIOTAG_HOST_RCV_NAL |   VLAN_FLTR_UNTAG_HOST_RCV_NAL | T'P'U'          |
                              | CONFIGURED|                                 |                                | (None Rx)       |
==============================+===========+=================================+================================+=================+
ENABLE                        | NOT       |  VLAN_FLTR_PRIOTAG_HOST_RCV_NAL |   VLAN_FLTR_UNTAG_HOST_RCV_ALL | T'P'U           |
                              | CONFIGURED|                                 |                                | (Only U Rx)     |
==============================+===========+=================================+================================+=================+
ENABLE                        |NOT        |  VLAN_FLTR_PRIOTAG_HOST_RCV_ALL |   VLAN_FLTR_UNTAG_HOST_RCV_NAL | T'U'P           |
                              |CONFIGURED |                                 |                                | (Only P Rx)     |
==============================+===========+=================================+================================+=================+
ENABLE                        | NOT       |  VLAN_FLTR_PRIOTAG_HOST_RCV_ALL |   VLAN_FLTR_UNTAG_HOST_RCV_ALL | T'PU            |
                              | CONFIGURED|                                 |                                | (Only P,U Rx)   |
==============================+===========+=================================+================================+=================+
ENABLE                        | CONFIGURED|  VLAN_FLTR_PRIOTAG_HOST_RCV_NAL |   VLAN_FLTR_UNTAG_HOST_RCV_NAL | P'U'T           |
                              |           |                                 |                                | (Only T Rx)     |
==============================+===========+=================================+================================+=================+
ENABLE                        | CONFIGURED|  VLAN_FLTR_PRIOTAG_HOST_RCV_NAL |   VLAN_FLTR_UNTAG_HOST_RCV_ALL | P'TU            |
                              |           |                                 |                                | (OnlyT, U Rx)   |
==============================+===========+=================================+================================+=================+
ENABLE                        |CONFIGURED |  VLAN_FLTR_PRIOTAG_HOST_RCV_ALL |   VLAN_FLTR_UNTAG_HOST_RCV_NAL | U'TP            |
                              |           |                                 |                                | (Only T, P Rx)  |
==============================+===========+=================================+================================+=================+
ENABLE                        |CONFIGURED |  VLAN_FLTR_PRIOTAG_HOST_RCV_ALL |  VLAN_FLTR_UNTAG_HOST_RCV_ALL  |  TPU            |
..............................|           |                                 |                                | (All Rx by Host)|
==============================+===========+=================================+================================+=================+
(DEFAULT)                     |NOT        |  VLAN_FLTR_PRIOTAG_HOST_RCV_ALL |   VLAN_FLTR_UNTAG_HOST_RCV_ALL |   TPU           |
DISABLE                       |CONFIGURED |                                 |                                | (All Rx by host)|
==============================+===========+=================================+================================+=================+
*/

static testVlanCfg_t testVlanFilterCfg[] =
{

    /* Test case configuration 1 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_DISABLE,                /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_NOT_CONFIGURED,             /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                             /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_PRIO_HOST_RX,         /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_UNTAGGED_HOST_RX,     /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TPU                      /* Expected result */
    },

    /* Test case configuration 2 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_NOT_CONFIGURED,             /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_PRIO_HOST_RX,         /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_UNTAGGED_HOST_RX,     /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_NONE                     /* Expected result */
    },

    /* Test case configuration 3 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_NOT_CONFIGURED,             /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_PRIO_HOST_RX,         /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_UNTAGGED_HOST_RX,         /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_UNTAGGED_WEIGHT          /* Expected result */
    },

    /* Test case configuration 4 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_NOT_CONFIGURED,             /* VID Config cmd*/
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_PRIO_HOST_RX,             /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_UNTAGGED_HOST_RX,     /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PRIORITY_WEIGHT          /* Expected result */
    },

    /* Test case configuration 5 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_NOT_CONFIGURED,             /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_PRIO_HOST_RX,             /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_UNTAGGED_HOST_RX,         /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PU                       /* Expected result */
    },

    /* Test case configuration 6 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_ADD_VLAN_ID,                 /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_PRIO_HOST_RX,         /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_UNTAGGED_HOST_RX,     /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TAGGED_WEIGHT            /* Expected result */
    },

    /* Test case configuration 7 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_ADD_VLAN_ID,                 /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_PRIO_HOST_RX,         /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_UNTAGGED_HOST_RX,         /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TU                       /* Expected result */
    },

    /* Test case configuration 8 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_ADD_VLAN_ID,                 /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_PRIO_HOST_RX,             /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_UNTAGGED_HOST_RX,     /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TP                       /* Expected result */
    },

    /* Test case configuration 9 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_ADD_VLAN_ID,                 /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_PRIO_HOST_RX,             /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_UNTAGGED_HOST_RX,         /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TPU                      /* Expected result */
    },
    /* Test case configuration 10 */
    {
      ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE,                 /* Vlan filter feature */
      ICSS_EMAC_VLAN_FILTER_TEST_VID_REMOVE_VLAN_ID,             /* VID Config cmd */
      ICSS_EMAC_VLAN_FILTER_TEST_VID,                            /* Test VID */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_PRIO_HOST_RX,             /* Priority tag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_UNTAGGED_HOST_RX,         /* Untag host rx control */
      ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PU                       /* Expected result */
     }

};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void setUp(void)
{
}

void tearDown(void)
{
}

void ICSS_EMAC_testTask(void *arg)
{
    uint32_t testCaseId = ICSS_EMAC_getTestCaseId();

    UNITY_BEGIN();

    RUN_TEST(test_ICSS_EMAC_verify, testCaseId, NULL);

    UNITY_END();
}

void test_ICSS_EMAC_verify(void *args)
{
    bool                    retVal = false;
    uint32_t                count = 0;
    uint32_t                fail_count= 0;
    ICSS_EMAC_IoctlCmd      ioctlParams;
    ICSS_EMAC_TxArgument    txArgs;
    ICSS_EMAC_RxArgument    rxArgs;
    int32_t                 linkStatus;
    uint32_t                txPacketSize;
    uint32_t                result_flag = 0;
    uint32_t                pru0FwPtr = 0;
    uint32_t                pru0FwLength = 0;
    uint32_t                pru1FwPtr = 0;
    uint32_t                pru1FwLength = 0;
    uint32_t                gFail_count = 0;

    memset(&txArgs, 0, sizeof(ICSS_EMAC_TxArgument));
    memset(&rxArgs, 0, sizeof(ICSS_EMAC_RxArgument));

    PRUICSS_disableCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_1-1);
    PRUICSS_disableCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_2-1);

    ICSS_EMAC_testGetPruFwPtr(&pru0FwPtr, &pru0FwLength, &pru1FwPtr, &pru1FwLength);

    result_flag = PRUICSS_writeMemory(ICSS_EMAC_testPruIcssHandle2, PRUICSS_IRAM_PRU(0), 0, (uint32_t *)pru0FwPtr, pru0FwLength);
    if(result_flag)
    {
        DebugP_log("load to PRU0 passed\r\n");
        retVal = true;
    }
    else
    {
        DebugP_log("load to PRU0 failed\r\n");
    }
    result_flag = PRUICSS_writeMemory(ICSS_EMAC_testPruIcssHandle2, PRUICSS_IRAM_PRU(1), 0, (uint32_t *)pru1FwPtr, pru1FwLength);
    if(result_flag)
    {
        DebugP_log("load to PRU1 passed\r\n");
        retVal = true;
    }
    else
    {
        DebugP_log("load to PRU0 failed\r\n");
    }

    if( retVal)
    {
        PRUICSS_enableCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_1-1);
        PRUICSS_enableCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_2-1);


        while ((!ICSS_EMAC_testLinkIsrPru2Eth0) && (!ICSS_EMAC_testLinkIsrPru2Eth1))
        {
            DebugP_log("ICSS_EMAC_testTaskPruss2: LINK IS DOWN, plug in loopback cable for PRU2 ETH0 and ETH1\r\n");
            ClockP_usleep(10000);
        }
        ICSS_EMAC_testLinkUpCount++;
        linkStatus = ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS;
        DebugP_log("ICSS_EMAC_testTaskPruss2: PRU2 ETH0: LINK IS UP, eth0 state: %d, link up count: %d\r\n", linkStatus, ICSS_EMAC_testLinkUpCount);
        linkStatus = ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS;
        DebugP_log("ICSS_EMAC_testTaskPruss2: PRU2 ETH1: LINK IS UP, eth0 state: %d, link up count: %d\r\n", linkStatus, ICSS_EMAC_testLinkUpCount);

        /* Creating Unicast packets in Non-Promiscuous Mode.*/
        for (count=0;count < 6;count++)
        {
            ICSS_EMAC_testPkt2[count] = ICSS_EMAC_testLclMac2[count];
        }

        /* Enabling Promiscuous Mode. Hence, disable -> reset -> change mmap -> enable */
        PRUICSS_disableCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_1-1);
        PRUICSS_disableCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_2-1);
        PRUICSS_resetCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_1-1);
        PRUICSS_resetCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_2-1);

        promisEnableFlag = 1;           /* enabling promiscuous mode */
        ioctlParams.command = 0;
        ioctlParams.ioctlVal = (void *)(&promisEnableFlag);
        retVal = ICSS_EMAC_ioctl(ICSS_EMAC_testHandle2, ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL, ICSS_EMAC_PORT_1, &ioctlParams);
        retVal = ICSS_EMAC_ioctl(ICSS_EMAC_testHandle2, ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL, ICSS_EMAC_PORT_2, &ioctlParams);

        PRUICSS_enableCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_1-1);
        PRUICSS_enableCore(ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_PORT_2-1);

        /* Sending Unicast packets in Promiscuous Mode. The eth should be able to recieve packets as destination address != eth own address*/
        DebugP_log("\n\n\n======================================================================");
        DebugP_log("\n ICSS_EMAC_testTaskPruss2: Testing Promiscuous Mode, multiple packet sizes");
        DebugP_log("\n sending Unicast packets as destination address != eth own address");
        DebugP_log("\n==========================================================================\n\r\n");

        for(txPacketSize = 64; txPacketSize <= ICSS_EMAC_MAXMTU; txPacketSize += 100)
        {
            DebugP_log("Testing packet size = %u\r\n", txPacketSize);

            txArgs.icssEmacHandle = ICSS_EMAC_testHandle2;
            txArgs.lengthOfPacket = txPacketSize;
            txArgs.portNumber = 1;
            txArgs.queuePriority = 3;
            txArgs.srcAddress = &ICSS_EMAC_testPktPromiscuous[0];

            /*TODO: Check if this is needed */
    //        ICSS_EMAC_setPortLinkStatus(ICSS_EMAC_testHandle2, 0, 1);
            ICSS_EMAC_testPacketRcvdPort3 = 1;
            for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
            {
                {
                    if(ICSS_EMAC_testPacketRcvdPort3 )
                    {
                        ICSS_EMAC_testPacketRcvdPort3 = 0;
                        ICSS_EMAC_txPacket(&txArgs, NULL);
                        fail_count = 0;
                        while(!ICSS_EMAC_testPacketRcvdPort3)
                        {
                            /*TODO: Review this*/
                            ClockP_usleep(10000);
                            fail_count++;
                        }
                    }
                }
            }

            if(txPacketSize == 64)
                txPacketSize = 0;

            if(txPacketSize == 1500)
                txPacketSize = 1418;
        }
        if (gFail_count == 0)
            //&& (ICSS_EMAC_testTotalPktRcvd == (ICSS_EMAC_TEST_PKT_TX_COUNT*ICSS_EMAC_testLinkUpCount)))
        {
            DebugP_log("All tests have passed\r\n");
            TEST_PASS();
        }
        else
        {
            DebugP_log("Few/All tests have failed\r\n");
            TEST_FAIL();
        }
    }
    else
    {
        DebugP_log("ICSS_EMAC_testTaskPruss2: firmware load failure\r\n");
        TEST_FAIL();
    }
}

/*
 * ---ICSS_EMAC Link Interrupt Service routine callback ---
 */
int32_t ICSS_EMAC_testLinkIsrCb(void *icssEmacHandleVoidPtr, void *linkStatus, void *userArg)
{
    uint32_t status = (uint32_t)linkStatus;
    uint32_t pruNum = (uint32_t)userArg;

    if (pruNum > ICSS_EMAC_TEST_PRU2ETH1)
    {
         return -1;
    }
    if ((status  == 1) && (pruNum == ICSS_EMAC_TEST_PRU1ETH0))
    {
        ICSS_EMAC_testLinkIsrPru1Eth0++;
    }
    else if ((status  == 1) && (pruNum == ICSS_EMAC_TEST_PRU1ETH1))
    {
        ICSS_EMAC_testLinkIsrPru1Eth1++;
    }
    else if ((status  == 1) && (pruNum == ICSS_EMAC_TEST_PRU2ETH0))
    {
        ICSS_EMAC_testLinkIsrPru2Eth0++;
    }
    else if ((status  == 1) && (pruNum == ICSS_EMAC_TEST_PRU2ETH1))
    {
        ICSS_EMAC_testLinkIsrPru2Eth1++;
    }
    else
    {
        return -1;
    }
    return 0;
}

int32_t ICSS_EMAC_testCallbackRxPacket2(void *icssEmacHandleVoidPtr, void *queueNum, void *userArg)
{
    uint8_t         j;
    uint32_t        tmp;
    int32_t         packetLength;
    ICSS_EMAC_RxArgument rxArgs;
//    ICSS_EMAC_PktInfo rxPktInfo;
    bool validEtherType;
    ICSS_EMAC_Handle icssEmacHandle = (ICSS_EMAC_Handle)icssEmacHandleVoidPtr;
    int32_t                 portNumber;
    int32_t                 queueNumber;

    rxArgs.icssEmacHandle = ICSS_EMAC_testHandle2;
    rxArgs.queueNumber = ((uint32_t)(queueNum));
    rxArgs.more = 0;
    rxArgs.port = 0;

    if(ICSS_EMAC_testTtsModePort1)
    {
        rxArgs.destAddress =  (uint32_t)(&ICSS_EMAC_testTtsRecvPktPort1[0]);
        memset(ICSS_EMAC_testTtsRecvPktPort1, 0, ICSS_EMAC_MAXMTU);
        if(icssEmacHandle == ICSS_EMAC_testHandle2)
        {
                packetLength = ICSS_EMAC_rxPktGet(&rxArgs, NULL);
                DebugP_assert((packetLength != -1) && (packetLength != 0));
                /*    Extract packet number    */
                tmp = 0;
                for(j=0;j<4;j++)
                {
                    tmp = (tmp | (uint32_t)(ICSS_EMAC_testTtsRecvPktPort1[packetLength - j -1] << ((3-j)*8)));
                }

                if((*((uint32_t *)(queueNum))) == ICSS_EMAC_QUEUE1)
                {
                    if((tmp - ICSS_EMAC_testRecv_Q1PktCntPort1) != 1)
                    {
                         DebugP_log("\nTTS Port 1: Error in received cyclic pkt sequence.\nPrevious seq no.: %u\nNew seq no.:%u", ICSS_EMAC_testRecv_Q1PktCntPort1, tmp);
                    }
                    ICSS_EMAC_testRecv_Q1PktCntPort1 = tmp;
                    if (0 != (memcmp(&ICSS_EMAC_testTtsRecvPktPort1[0], &ICSS_EMAC_testArpPktPort1[0],ICSS_EMAC_TEST_ARP_PKT_SIZE_PORT1)))
                    {
                         DebugP_log("\nTTS Port 1: Error in received cyclic pkt content. Packet mismatch.");
                    }
                    /*    If interrupt is for cyclic packet completion, post semaphore    */
                    SemaphoreP_post(&ICSS_EMAC_testTtsP1TimeSem);
                }
                else if((*((uint32_t *)(queueNum))) == ICSS_EMAC_QUEUE2)
                {
                    if((tmp - ICSS_EMAC_testRecv_Q2PktCntPort1) != 1)
                    {
                         DebugP_log("\nTTS Port 1: Error in received acyclic pkt sequence.\nPrevious seq no.: %u\nNew seq no.:%u", ICSS_EMAC_testRecv_Q2PktCntPort1, tmp);
                    }
                    ICSS_EMAC_testRecv_Q2PktCntPort1 = tmp;
                    if (0 != (memcmp(&ICSS_EMAC_testTtsRecvPktPort1[0], &ICSS_EMAC_testUdpPktPort1[0],ICSS_EMAC_TEST_UDP_PKT_SIZE_PORT1)))
                    {
                         DebugP_log("\nTTS Port 1: Error in received acyclic pkt content. Packet mismatch.");
                    }
                }

                /*    Check if mac cyclic and acyclic packet count is reached    */
                /*    This means test is over and we can DebugP_log results    */
                if((ICSS_EMAC_testRecv_Q1PktCntPort1 == ICSS_EMAC_TEST_TTS_MAX_CYC_PKT_COUNT) && (ICSS_EMAC_testRecv_Q2PktCntPort1 == ICSS_EMAC_TEST_TTS_MAX_ACYC_PKT_COUNT))
                {
                    /*    Post semaphore    */
                    SemaphoreP_post(&ICSS_EMAC_testTtsP1ResultSem);
                }
        }
    }
    else
    {
        rxArgs.destAddress =  (uint32_t)(&ICSS_EMAC_testPacketArrayInstance2[0]);

        if(icssEmacHandle == ICSS_EMAC_testHandle2)
        {
            for (tmp = 1; tmp; )
            {
                /*TODO: Review this*/
                // memset(&rxPktInfo,0, sizeof(ICSS_EMAC_PktInfo));
                // ICSS_EMAC_RxPktInfo2(icssEmacHandle, &rxPktInfo);
                // validEtherType = ICSS_EMAC_testRxPktParser(icssEmacHandle, (void*)rxPktInfo.rdBufferL3Addr);
                ICSS_EMAC_rxPktInfo(ICSS_EMAC_testHandle2, &portNumber, &queueNumber);

                if(enableQOverFlowCheck != 1)
                {
                    /*TODO: Review this*/
//                    if (validEtherType != true)
//                    {
//#if !defined (STP_SWITCH)
//                        DebugP_log("parser returned incorect values: rdBufferL3Addr 0x%x, port: 0x%x, queue: 0x%x\r\n",rxPktInfo.rdBufferL3Addr, rxPktInfo.portNumber, rxPktInfo.queueNumber);
//                        return 0;
//#endif /* STP_SWITCH */
//                    }
                    packetLength = ICSS_EMAC_rxPktGet(&rxArgs, NULL);

                    validEtherType = ICSS_EMAC_testRxPktParser(icssEmacHandle, (void*)(rxArgs.destAddress));
                    if (validEtherType != true)
                    {
    #if !defined (STP_SWITCH)
                        DebugP_log("parser returned incorect values: destAddress 0x%x, port: 0x%x, queue: 0x%x\r\n",
                                    rxArgs.destAddress, rxArgs.port, rxArgs.queueNumber);
                        return 0;
    #endif /* STP_SWITCH */
                    }
                }
                else
                {
                    ICSS_EMAC_testPacketRcvdPort2++; /* PRU2 ETH 0 */
                    packetLength = 0;
                    tmp = 0;
                }
                if(packetLength)
                {
#if defined (STP_SWITCH)
          stp_switch_test(icssEmacHandle, rxPktInfo);
#else
                    if(!(memcmp(&ICSS_EMAC_testPacketArrayInstance2[0], &ICSS_EMAC_testPkt2[0],ICSS_EMAC_TEST_PKT_SIZE)))
                    {
                        DebugP_log("ICSS_EMAC_testTaskPruss2(PRU2 ETH0): received pkt: %d\r\n", ICSS_EMAC_testTotalPktRcvd);
                        ICSS_EMAC_testTotalPktRcvd++;
                        ICSS_EMAC_testPacketRcvdPort2++; /* PRU2 ETH 0 */
                    }
                    else
                    {
                        if(1U == promisEnableFlag)
                        {
                            ICSS_EMAC_testPacketRcvdPort2++;
                            /* DebugP_log("Unit Test Passed, Promiscuous mode packet mismatch occured\r\n"); */
                        }
                        else
                        {
                            DebugP_log("Unit Test Failure, packet mismatch occured\r\n");
                        }
                    }
#endif /* STP_SWITCH */
                }
                if(rxArgs.more== 0)
                    tmp = 0;    // exit if there are no more packets
            }
        }
    }
    return 0;
}

int32_t ICSS_EMAC_testCallbackRxPacket3(void *icssEmacHandleVoidPtr, void *queueNum, void *userArg)
{
    uint8_t         j;
    uint32_t        tmp;
    int32_t         packetLength;
    ICSS_EMAC_RxArgument rxArgs;
//    ICSS_EMAC_PktInfo rxPktInfo;
    bool etherType;
    ICSS_EMAC_Handle icssEmacHandle = (ICSS_EMAC_Handle)icssEmacHandleVoidPtr;

    rxArgs.icssEmacHandle = ICSS_EMAC_testHandle3;
    rxArgs.queueNumber = ((uint32_t)(queueNum));
    rxArgs.more = 0;
    rxArgs.port = 0;

    if(ICSS_EMAC_testTtsModePort2)
    {
        rxArgs.destAddress =  (uint32_t)(&ICSS_EMAC_testTtsRecvPktPort2[0]);
        memset(ICSS_EMAC_testTtsRecvPktPort2, 0, ICSS_EMAC_MAXMTU);
        if(icssEmacHandle == ICSS_EMAC_testHandle3)
        {
                packetLength = ICSS_EMAC_rxPktGet(&rxArgs, NULL);
                DebugP_assert((packetLength != -1) && (packetLength != 0));
                /*    Extract packet number    */
                tmp = 0;
                for(j=0;j<4;j++)
                {
                    tmp = (tmp | (uint32_t)(ICSS_EMAC_testTtsRecvPktPort2[packetLength - j -1] << ((3-j)*8)));
                }

                if((*((uint32_t *)(queueNum))) == ICSS_EMAC_QUEUE3)
                {
                    if((tmp - ICSS_EMAC_testRecv_Q1PktCntPort2) != 1)
                    {
                         DebugP_log("\nTTS Port 2: Error in received cyclic pkt sequence.\nPrevious seq no.: %u\nNew seq no.:%u", ICSS_EMAC_testRecv_Q1PktCntPort2, tmp);
                    }
                    ICSS_EMAC_testRecv_Q1PktCntPort2 = tmp;
                    if (0 != (memcmp(&ICSS_EMAC_testTtsRecvPktPort2[0], &ICSS_EMAC_testArpPktPort2[0],ICSS_EMAC_TEST_ARP_PKT_SIZE_PORT2)))
                    {
                         DebugP_log("\nTTS Port 2: Error in received cyclic pkt content. Packet mismatch.");
                    }
                    /*    If interrupt is for cyclic packet completion, post semaphore    */
                    SemaphoreP_post(&ICSS_EMAC_testTtsP2TimeSem);
                }
                else if((*((uint32_t *)(queueNum))) == ICSS_EMAC_QUEUE4)
                {
                    if((tmp - ICSS_EMAC_testRecv_Q2PktCntPort2) != 1)
                    {
                         DebugP_log("\nTTS Port 2: Error in received acyclic pkt sequence.\nPrevious seq no.: %u\nNew seq no.:%u", ICSS_EMAC_testRecv_Q2PktCntPort2, tmp);
                    }
                    ICSS_EMAC_testRecv_Q2PktCntPort2 = tmp;
                    if (0 != (memcmp(&ICSS_EMAC_testTtsRecvPktPort2[0], &ICSS_EMAC_testUdpPktPort2[0],ICSS_EMAC_TEST_UDP_PKT_SIZE_PORT2)))
                    {
                         DebugP_log("\nTTS Port 2: Error in received acyclic pkt content. Packet mismatch.");
                    }
                }

                /*    Check if mac cyclic and acyclic packet count is reached    */
                /*    This means test is over and we can DebugP_log results    */
                if((ICSS_EMAC_testRecv_Q1PktCntPort2 == ICSS_EMAC_TEST_TTS_MAX_CYC_PKT_COUNT) && (ICSS_EMAC_testRecv_Q2PktCntPort2 == ICSS_EMAC_TEST_TTS_MAX_ACYC_PKT_COUNT))
                {
                    /*    Post semaphore    */
                    SemaphoreP_post(&ICSS_EMAC_testTtsP2ResultSem);
                }
        }
    }
    else
    {
        rxArgs.destAddress =  (uint32_t)(&ICSS_EMAC_testPacketArrayInstance2_1[0]);

        if(icssEmacHandle == ICSS_EMAC_testHandle3)
        {
            for (tmp = 1; tmp; )
            {
                packetLength = ICSS_EMAC_rxPktGet(&rxArgs, NULL);

                etherType = ICSS_EMAC_testRxPktParser(icssEmacHandle, (void*)(rxArgs.destAddress));
                if (etherType != true)
                {
                    DebugP_log("parser returned incorect values: destAddress 0x%x, port: 0x%x, queue: 0x%x\r\n",
                                rxArgs.destAddress, rxArgs.port, rxArgs.queueNumber);
                    return 0;
                }

                if(packetLength)
                {
                    // if (!(memcmp(&ICSS_EMAC_testPacketArrayInstance2_1[0], &ICSS_EMAC_testPkt3[0], ICSS_EMAC_TEST_PKT_SIZE)))
                    if (!(memcmp(&ICSS_EMAC_testPacketArrayInstance2_1[0], &ICSS_EMAC_testPkt2[0], packetLength)))
                    {
                        DebugP_log("ICSS_EMAC_testTaskPruss2(PRU2 ETH1): received pkt: %d\r\n", ICSS_EMAC_testTotalPktRcvd);
                        ICSS_EMAC_testTotalPktRcvd++;
                        ICSS_EMAC_testPacketRcvdPort3++; /* PRU2 ETH 0 */
                    }
                    else
                    {
                        if(1U == promisEnableFlag)
                        {
                            ICSS_EMAC_testPacketRcvdPort3++;
                            if (!(memcmp(&ICSS_EMAC_testPacketArrayInstance2_1[0], &ICSS_EMAC_testPktPromiscuous[0], packetLength)))
                            {
                                DebugP_log("ICSS_EMAC_testTaskPruss2(PRU2 ETH1): received pkt in promiscuous mode: %d\r\n", ICSS_EMAC_testTotalPktRcvd);
                            }
                            else
                            {
                                DebugP_log("ICSS_EMAC_testTaskPruss2(PRU2 ETH1): DATA MISMATCH in received pkt in promiscuous mode: %d !\r\n", ICSS_EMAC_testTotalPktRcvd);
                            }
                            /* DebugP_log("Unit Test Passed, Promiscuous mode packet mismatch occured\r\n"); */
                        }
                        else
                        {
                            DebugP_log("Unit Test Failure, packet mismatch occured\r\n");
                        }
                    }
                }
                if(rxArgs.more== 0)
                        tmp = 0;    // exit if there are no more packets
            }
        }
    }
    return 0;
}

int32_t ICSS_EMAC_testCallbackTxComplete(void *icssEmacHandleVoidPtr, void *arg, void *userArg)
{
    if(!(ICSS_EMAC_testTtsModePort1 | ICSS_EMAC_testTtsModePort2))
    {
        if((ICSS_EMAC_Handle)icssEmacHandleVoidPtr ==  ICSS_EMAC_testHandle)
        {
            ICSS_EMAC_testPacketTxCompletePort0++;
        }
        else if((ICSS_EMAC_Handle)icssEmacHandleVoidPtr == ICSS_EMAC_testHandle1)
        {
            ICSS_EMAC_testPacketTxCompletePort1++;
        }
        else if((ICSS_EMAC_Handle)icssEmacHandleVoidPtr == ICSS_EMAC_testHandle2)
        {
            ICSS_EMAC_testPacketTxCompletePort2++;
        }
        else if((ICSS_EMAC_Handle)icssEmacHandleVoidPtr == ICSS_EMAC_testHandle3)
        {
            ICSS_EMAC_testPacketTxCompletePort3++;
        }
        else
        {
            /* Error DebugP_log */
            DebugP_log("packet transmission not complete for packet(ICSS_EMAC_TEST_PRU1ETH0): invalid Emac Sub System Handle passed  \r\n");
        }
    }
    return 0;
}

int32_t ICSS_EMAC_testGetPruStats(uint8_t portNum, ICSS_EMAC_Handle icssEmacHandle)
{
    ICSS_EMAC_PruStatistics pruStats;

    if (icssEmacHandle == ICSS_EMAC_testHandle)
    {
        DebugP_log("\nPRU-ICSS STATS for PRU1ETH0, port: %d\r\n", portNum);
    }
    else if (icssEmacHandle == ICSS_EMAC_testHandle1)
    {
        DebugP_log("\nPRU-ICSS STATS for PRU1ETH1, port: %d\r\n", portNum);
    }
    else if (icssEmacHandle == ICSS_EMAC_testHandle2)
    {
        DebugP_log("\nPRU-ICSS STATS for PRU2ETH0, port: %d\r\n", portNum);
    }
    else if (icssEmacHandle == ICSS_EMAC_testHandle3)
    {
        DebugP_log("\nPRU-ICSS STATS for PRU2ETH1, port: %d\r\n", portNum);
    }
    else
    {
        DebugP_log("ICSS_EMAC_testGetPruStats: Invalid ICSS_EMAC_Handle, returning without stats\r\n");
        return -1;
    }

    memset((void *)&pruStats, 0, sizeof(ICSS_EMAC_PruStatistics));

    ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STAT_CTRL_GET, portNum, (void *)(&pruStats));

    DebugP_log("txBcast\t:\t0x%x\r\n", pruStats.txBcast);
    DebugP_log("txMcast\t:\t0x%x\r\n", pruStats.txMcast);
    DebugP_log("txUcast\t:\t0x%x\r\n", pruStats.txUcast);
    DebugP_log("txOctets\t:\t0x%x\r\n", pruStats.txOctets);

    DebugP_log("rxBcast\t:\t0x%x\r\n", pruStats.rxBcast);
    DebugP_log("rxMcast\t:\t0x%x\r\n", pruStats.rxMcast);
    DebugP_log("rxUcast\t:\t0x%x\r\n", pruStats.rxUcast);
    DebugP_log("rxOctets\t:\t0x%x\r\n", pruStats.rxOctets);

    DebugP_log("tx64byte\t:\t0x%x\r\n", pruStats.tx64byte);
    DebugP_log("tx65_127byte\t:\t0x%x\r\n", pruStats.tx65_127byte);
    DebugP_log("tx128_255byte\t:\t0x%x\r\n", pruStats.tx128_255byte);
    DebugP_log("tx512_1023byte\t:\t0x%x\r\n", pruStats.tx512_1023byte);
    DebugP_log("tx1024byte\t:\t0x%x\r\n", pruStats.tx1024byte);

    DebugP_log("rx64byte\t:\t0x%x\r\n", pruStats.rx64byte);
    DebugP_log("rx65_127byte\t:\t0x%x\r\n", pruStats.rx65_127byte);
    DebugP_log("rx128_255byte\t:\t0x%x\r\n", pruStats.rx128_255byte);
    DebugP_log("rx512_1023byte\t:\t0x%x\r\n", pruStats.rx512_1023byte);
    DebugP_log("rx1024byte\t:\t0x%x\r\n", pruStats.rx1024byte);

    DebugP_log("lateColl\t:\t0x%x\r\n", pruStats.lateColl);
    DebugP_log("singleColl\t:\t0x%x\r\n", pruStats.singleColl);
    DebugP_log("multiColl\t:\t0x%x\r\n", pruStats.excessColl);
    DebugP_log("excessColl\t:\t0x%x\r\n", pruStats.excessColl);

    DebugP_log("rxMisAlignmentFrames\t:\t0x%x\r\n", pruStats.rxMisAlignmentFrames);
    DebugP_log("stormPrevCounter\t:\t0x%x\r\n", pruStats.stormPrevCounter);
    DebugP_log("macRxError\t:\t0x%x\r\n", pruStats.macRxError);
    DebugP_log("SFDError\t:\t0x%x\r\n", pruStats.SFDError);
    DebugP_log("defTx\t:\t0x%x\r\n", pruStats.defTx);
    DebugP_log("macTxError\t:\t0x%x\r\n", pruStats.macTxError);
    DebugP_log("rxOverSizedFrames\t:\t0x%x\r\n", pruStats.rxOverSizedFrames);
    DebugP_log("rxUnderSizedFrames\t:\t0x%x\r\n", pruStats.rxUnderSizedFrames);
    DebugP_log("rxCRCFrames\t:\t0x%x\r\n", pruStats.rxCRCFrames);
    DebugP_log("droppedPackets\t:\t0x%x\r\n", pruStats.droppedPackets);

/* Debug variables, these are not part of standard MIB. Useful for debugging
    Reserved for future Use
 */
    DebugP_log("txOverFlow\t:\t0x%x\r\n", pruStats.txOverFlow);
    DebugP_log("txUnderFlow\t:\t0x%x\r\n", pruStats.txUnderFlow);
    DebugP_log("sqeTestError\t:\t0x%x\r\n", pruStats.sqeTestError);
    DebugP_log("TXqueueLevel\t:\t0x%x\r\n", pruStats.TXqueueLevel);
    DebugP_log("CSError\t:\t0x%x\n\r\n", pruStats.CSError);

    return 0;
}

/* Multicast filtering Test details :
 *-------------+------------------------------------------------------+--------------
 * MCF         |  Destination address                                 |   Response
 *-------------+------------------------------------------------------+--------------
 * DISABLED    |   XX                                                 |   RECEIVE
 *-------------+------------------------------------------------------+--------------
 * DISABLED    |   with above config : 01:02:03:04:05:07              |   RECEIVE
 *             |   with above config : 01:02:03:04:05:06              |   RECEIVE
 *-------------+----------------------------------------------------- +--------------
 * ENABLED     |  MCT adds MC MAC ID (Lets say 01:02:03:04:05:06)     |   RECEIVE
 *             |  overlaps for 01:02:03:05:04:06 as well              |   RECEIVE
 *-------------+------------------------------------------------------+--------------
 * ENABLED     |  DST MAC ID (Lets say 01:02:03:04:05:07)             |   NOT RECEIVE
 *-------------+------------------------------------------------------+--------------
 * ENABLED     |  Remove 01:02:03:04:05:06 & ping                     |   NOT RECEIVE
               |  Ping with 01:02:03:04:05:07                         |   NOT RECEIVE
 *-------------+------------------------------------------------------+---------------
 */
bool ICSS_EMAC_testMulticastFiltering_EmacPort1(ICSS_EMAC_Handle icss_emacHandle)
{
    bool retVal = false;
    uint32_t count = 0;
    int32_t    portNum = ICSS_EMAC_PORT_1;
    uint32_t   fail_count, gFail_count;
    ICSS_EMAC_IoctlCmd ioctlParams;
    uint8_t    mc_macId[6] = { 01, 02, 03, 04, 05, 06};
    uint8_t    mask[6] = {255, 255, 255, 255, 255, 255};

    ICSS_EMAC_TxArgument txArgs;
    memset(&txArgs, 0, sizeof(ICSS_EMAC_TxArgument));

    /* Enable the promiscuous mode for the ports to get multicast packets*/
    {
        promisEnableFlag = 1;            /* enable promiscuous mode */
        ioctlParams.command = 0;
        ioctlParams.ioctlVal = (void *)(&promisEnableFlag);
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            return(false);
        }
    }

    /* Now enable MCF with MC MAC ID (Lets say 01:02:03:04:05:06) */
    ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ENABLE;
    retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
    if (retVal == false)
    {
        return(false);
    }

    ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ADD_MACID;
    ioctlParams.ioctlVal = (void *) &mc_macId[0] ;
    retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
    if (retVal == false)
    {
        return(false);
    }

    ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_OVERRIDE_HASHMASK;
    ioctlParams.ioctlVal = (void *) &mask[0] ;
    retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, ICSS_EMAC_PORT_1, &ioctlParams);
    if (retVal == false)
    {
        return(false);
    }

    /*
     * MCT with 01:02:03:04:05:06 shoud be received
     */

    ICSS_EMAC_testPacketRcvdPort2 = 1;
    txArgs.icssEmacHandle = ICSS_EMAC_testHandle2;
    txArgs.lengthOfPacket = sizeof(ICSS_EMAC_testPkt1Multicast);
    txArgs.portNumber = portNum;
    txArgs.queuePriority = 3;
    txArgs.srcAddress = &ICSS_EMAC_testPkt1Multicast[0];

    for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
    {
        if(ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)
        {
            if(ICSS_EMAC_testPacketRcvdPort2 )
            {
                ICSS_EMAC_testPacketRcvdPort2 = 0;
                ICSS_EMAC_txPacket(&txArgs, NULL);
                while(!ICSS_EMAC_testPacketRcvdPort2)
                {
                    ClockP_usleep(10000); // TODO: Review this
                }
            }
        }
    }

    /* Keep the multicast filter enabled
     * send with above config : 01:02:03:04:05:07 (should not be received)
     */

    ICSS_EMAC_testPacketRcvdPort2 = 1;
    txArgs.icssEmacHandle = ICSS_EMAC_testHandle2;
    txArgs.lengthOfPacket = sizeof(ICSS_EMAC_testPkt2Multicast);
    txArgs.portNumber = portNum;
    txArgs.queuePriority = 3;
    txArgs.srcAddress = &ICSS_EMAC_testPkt2Multicast[0];

    gFail_count = 0;
    for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
    {
        if(ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)
        {
            if(ICSS_EMAC_testPacketRcvdPort2 )
            {
                ICSS_EMAC_testPacketRcvdPort2 = 0;
                fail_count = 0;
                ICSS_EMAC_txPacket(&txArgs, NULL);
                while(!ICSS_EMAC_testPacketRcvdPort2)
                {
                    ClockP_usleep(10000); // TODO: Review this
                    fail_count++;
                    if (fail_count > 5)
                    {
                        gFail_count ++;
                        ICSS_EMAC_testPacketRcvdPort2 = 1;
                        break;
                    }
                }
            }
        }
    }

    if (gFail_count == ICSS_EMAC_TEST_PKT_TX_COUNT)
    {
        ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_REMOVE_MACID;
        ioctlParams.ioctlVal = (void *) &mc_macId[0] ;
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            goto testComplete;
        }

        /* Now disable MCF with MC MAC ID (Lets say 01:02:03:04:05:06) */
        ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_DISABLE;
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            goto testComplete;
        }

        retVal = true;
    }
    else
    {
        /* Disable MCF feature */
        ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_REMOVE_MACID;
        ioctlParams.ioctlVal = (void *) &mc_macId[0] ;
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            goto testComplete;
        }

        /* Now disable MCF with MC MAC ID (Lets say 01:02:03:04:05:06) */
        ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_DISABLE;
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            goto testComplete;
        }

        retVal = false;
    }

testComplete:

    if (retVal == true)
    {
        /* Now run the MC mac-ID checks and both should make the host packet get the packets */
        /* Keep the multicast filter (disabled)
         * send with above config : 01:02:03:04:05:06
         */
        ICSS_EMAC_testPacketRcvdPort2 = 1;
        txArgs.icssEmacHandle = ICSS_EMAC_testHandle2;
        txArgs.lengthOfPacket = sizeof(ICSS_EMAC_testPkt1Multicast);
        txArgs.portNumber = portNum;
        txArgs.queuePriority = 3;
        txArgs.srcAddress = &ICSS_EMAC_testPkt1Multicast[0];

        for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
        {
            if(ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)
            {
                if(ICSS_EMAC_testPacketRcvdPort2 )
                {
                    ICSS_EMAC_testPacketRcvdPort2 = 0;
                    ICSS_EMAC_txPacket(&txArgs, NULL);
                    while(!ICSS_EMAC_testPacketRcvdPort2)
                    {
                        ClockP_usleep(10000); // TODO: Review this
                    }
                }
            }
        }

        /* Keep the multicast filter default (disabled)
         * send with above config : 01:02:03:04:05:07
         */

        ICSS_EMAC_testPacketRcvdPort2 = 1;
        txArgs.icssEmacHandle = ICSS_EMAC_testHandle2;
        txArgs.lengthOfPacket = sizeof(ICSS_EMAC_testPkt2Multicast);
        txArgs.portNumber = portNum;
        txArgs.queuePriority = 3;
        txArgs.srcAddress = &ICSS_EMAC_testPkt2Multicast[0];

        for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
        {
            if(ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)
            {
                if(ICSS_EMAC_testPacketRcvdPort2 )
                {
                    ICSS_EMAC_testPacketRcvdPort2 = 0;
                    ICSS_EMAC_txPacket(&txArgs, NULL);
                    while(!ICSS_EMAC_testPacketRcvdPort2)
                    {
                        ClockP_usleep(10000); // TODO: Review this
                    }
                }
            }
        }
    }

    /* Disable the promiscuous mode for all ports to after multicast tests*/
    {
        promisEnableFlag = 0;            /* enable promiscuous mode */
        ioctlParams.command = 0;
        ioctlParams.ioctlVal = (void *)(&promisEnableFlag);
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle2, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            return(false);
        }
    }

    return (retVal);
}

/* Multicast filtering Test details :
 *-------------+------------------------------------------------------+--------------
 * MCF         |  Destination address                                 |   Response
 *-------------+------------------------------------------------------+--------------
 * DISABLED    |   XX                                                 |   RECEIVE
 *-------------+------------------------------------------------------+--------------
 * DISABLED    |   with above config : 01:02:03:04:05:07              |   RECEIVE
 *             |   with above config : 01:02:03:04:05:06              |   RECEIVE
 *-------------+----------------------------------------------------- +--------------
 * ENABLED     |  MCT adds MC MAC ID (Lets say 01:02:03:04:05:06)     |   RECEIVE
 *             |  overlaps for 01:02:03:05:04:06 as well              |   RECEIVE
 *-------------+------------------------------------------------------+--------------
 * ENABLED     |  DST MAC ID (Lets say 01:02:03:04:05:07)             |   NOT RECEIVE
 *-------------+------------------------------------------------------+--------------
 * ENABLED     |  Remove 01:02:03:04:05:06 & ping                     |   NOT RECEIVE
               |  Ping with 01:02:03:04:05:07                         |   NOT RECEIVE
 *-------------+------------------------------------------------------+---------------
 */
bool ICSS_EMAC_testMulticastFiltering_EmacPort2(ICSS_EMAC_Handle icss_emacHandle)
{
    bool retVal = false;
    uint32_t count = 0;
    int32_t    portNum = ICSS_EMAC_PORT_2;
    uint32_t   fail_count, gFail_count;
    ICSS_EMAC_IoctlCmd ioctlParams;
    uint8_t    mc_macId[6] = { 01, 02, 03, 04, 05, 06};
    uint8_t    mask[6] = {255, 255, 255, 255, 255, 255};
    ICSS_EMAC_TxArgument txArgs;
    memset(&txArgs, 0, sizeof(ICSS_EMAC_TxArgument));

    /* Enable the promiscuous mode for the ports to get multicast packets*/
    {
        promisEnableFlag = 1;    /* enable promiscuous mode */
        ioctlParams.command = 0;
        ioctlParams.ioctlVal = (void *)(&promisEnableFlag);
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            return(false);
        }
    }

    /* Now enable MCF with MC MAC ID (Lets say 01:02:03:04:05:06) */
    ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ENABLE;
    retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
    if (retVal == false)
    {
        return(false);
    }

    ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_ADD_MACID;
    ioctlParams.ioctlVal = (void *) &mc_macId[0] ;
    retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
    if (retVal == false)
    {
        return(false);
    }

    ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_OVERRIDE_HASHMASK;
    ioctlParams.ioctlVal = (void *) &mask[0] ;
    retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, ICSS_EMAC_PORT_1, &ioctlParams);
    if (retVal == false)
    {
        return(false);
    }

    /*
     * MCT with 01:02:03:04:05:06 shoud be received
     */

    ICSS_EMAC_testPacketRcvdPort3 = 1;
    txArgs.icssEmacHandle = icss_emacHandle;
    txArgs.lengthOfPacket = sizeof(ICSS_EMAC_testPkt1Multicast);
    txArgs.portNumber = portNum;
    txArgs.queuePriority = 3;
    txArgs.srcAddress = &ICSS_EMAC_testPkt1Multicast[0];

    for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
    {
        if(ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)
        {
            if(ICSS_EMAC_testPacketRcvdPort3 )
            {
                ICSS_EMAC_testPacketRcvdPort3 = 0;
                ICSS_EMAC_txPacket(&txArgs, NULL);
                while(!ICSS_EMAC_testPacketRcvdPort3)
                {
                    ClockP_usleep(10000); // TODO: Review this
                }
            }
        }
    }

    /* Keep the multicast filter enabled
     * send with above config : 01:02:03:04:05:07 (should not be received)
     */

    ICSS_EMAC_testPacketRcvdPort3 = 1;
    txArgs.icssEmacHandle = ICSS_EMAC_testHandle3;
    txArgs.lengthOfPacket = sizeof(ICSS_EMAC_testPkt2Multicast);
    txArgs.portNumber = portNum;
    txArgs.queuePriority = 3;
    txArgs.srcAddress = &ICSS_EMAC_testPkt2Multicast[0];

    gFail_count = 0;
    for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
    {
        if(ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)
        {
            if(ICSS_EMAC_testPacketRcvdPort3 )
            {
                ICSS_EMAC_testPacketRcvdPort3 = 0;
                fail_count = 0;
                ICSS_EMAC_txPacket(&txArgs, NULL);
                while(!ICSS_EMAC_testPacketRcvdPort3)
                {
                    ClockP_usleep(10000); // TODO: Review this
                    fail_count++;
                    if (fail_count > 5)
                    {
                        gFail_count ++;
                        ICSS_EMAC_testPacketRcvdPort3 = 1;
                        break;
                    }
                }
            }
        }
    }

    if (gFail_count == ICSS_EMAC_TEST_PKT_TX_COUNT)
    {
        ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_REMOVE_MACID;
        ioctlParams.ioctlVal = (void *) &mc_macId[0] ;
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            goto testComplete;
        }

        /* Now disable MCF with MC MAC ID (Lets say 01:02:03:04:05:06) */
        ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_DISABLE;
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            goto testComplete;
        }

        retVal = true;
    }
    else
    {
        /* Disable MCF feature */
        ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_REMOVE_MACID;
        ioctlParams.ioctlVal = (void *) &mc_macId[0] ;
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            goto testComplete;
        }

        /* Now disable MCF with MC MAC ID (Lets say 01:02:03:04:05:06) */
        ioctlParams.command = ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL_DISABLE;
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_MULTICAST_FILTER_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            goto testComplete;
        }

        retVal = false;
    }

testComplete:

    if (retVal == true)
    {
        /* Now run the MC mac-ID checks and both should make the host packet get the packets */
        /* Keep the multicast filter (disabled)
         * send with above config : 01:02:03:04:05:06
         */
        ICSS_EMAC_testPacketRcvdPort3 = 1;
        txArgs.icssEmacHandle = ICSS_EMAC_testHandle3;
        txArgs.lengthOfPacket = sizeof(ICSS_EMAC_testPkt1Multicast);
        txArgs.portNumber = portNum;
        txArgs.queuePriority = 3;
        txArgs.srcAddress = &ICSS_EMAC_testPkt1Multicast[0];

        for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
        {
            if(ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)
            {
                if(ICSS_EMAC_testPacketRcvdPort3 )
                {
                    ICSS_EMAC_testPacketRcvdPort3 = 0;
                    ICSS_EMAC_txPacket(&txArgs, NULL);
                    while(!ICSS_EMAC_testPacketRcvdPort3)
                    {
                        ClockP_usleep(10000); // TODO: Review this
                    }
                }
            }
        }

        /* Keep the multicast filter default (disabled)
         * send with above config : 01:02:03:04:05:07
         */

        ICSS_EMAC_testPacketRcvdPort3 = 1;
        txArgs.icssEmacHandle = ICSS_EMAC_testHandle3;
        txArgs.lengthOfPacket = sizeof(ICSS_EMAC_testPkt2Multicast);
        txArgs.portNumber = portNum;
        txArgs.queuePriority = 3;
        txArgs.srcAddress = &ICSS_EMAC_testPkt2Multicast[0];

        for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
        {
            if(ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)
            {
                if(ICSS_EMAC_testPacketRcvdPort3 )
                {
                    ICSS_EMAC_testPacketRcvdPort3 = 0;
                    ICSS_EMAC_txPacket(&txArgs, NULL);
                    while(!ICSS_EMAC_testPacketRcvdPort3)
                    {
                        ClockP_usleep(10000); // TODO: Review this
                    }
                }
            }
        }
    }

    /* Disable the promiscuous mode for all ports to after multicast tests*/
    {
        promisEnableFlag = 0;            /* enable promiscuous mode */
        ioctlParams.command = 0;
        ioctlParams.ioctlVal = (void *)(&promisEnableFlag);
        retVal = ICSS_EMAC_firmware_feature_ctrl(&ICSS_EMAC_testHandle3, &ICSS_EMAC_testPruIcssHandle2, ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL, portNum, &ioctlParams);
        if (retVal == false)
        {
            return(false);
        }
    }

    return (retVal);
}


bool ICSS_EMAC_testVlanFiltering_On_EmacPort(PRUICSS_Handle *pruIcssHandlePtr, uint8_t portNum)
{
    bool                 retVal = false;
    uint32_t             weight;
    uint32_t             testCfgIndex;
    ICSS_EMAC_Handle     *icssEmacHandlePtr = NULL;
    ICSS_EMAC_IoctlCmd    ioctlParams;

    /* Test VLAN for given ports */
    if (*pruIcssHandlePtr ==  ICSS_EMAC_testPruIcssHandle2)
    {
        if (portNum == ICSS_EMAC_PORT_1)
        {
            icssEmacHandlePtr = &ICSS_EMAC_testHandle2;
        }
        else
        {
            icssEmacHandlePtr = &ICSS_EMAC_testHandle3;
        }
    }

    if (*pruIcssHandlePtr ==  ICSS_EMAC_testPruIcssHandle1)
    {
        if (portNum == ICSS_EMAC_PORT_1)
        {
            icssEmacHandlePtr = &ICSS_EMAC_testHandle;
        }
        else
        {
            icssEmacHandlePtr = &ICSS_EMAC_testHandle1;
        }
    }

    if(icssEmacHandlePtr != NULL)
    {
        /* Enable the promiscuous mode for the ports to get VLAN packets*/
        promisEnableFlag = 1;    /* enable promiscuous mode */
        ioctlParams.command = 0;
        ioctlParams.ioctlVal = (void *)(&promisEnableFlag);
        retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruIcssHandlePtr, ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL, portNum, &ioctlParams);

        /* Test the configurations */
        for (testCfgIndex = 0, weight = 0; ((testCfgIndex < ICSS_EMAC_TEST_MAX_NUM_VLAN_TEST_CFGS) && (retVal == true)); testCfgIndex++)
        {
            /* Configure the VLAN filter test parameters */
            retVal = ICSS_EMAC_vlan_filter_feature_config(icssEmacHandlePtr, pruIcssHandlePtr, portNum, &testVlanFilterCfg[testCfgIndex]);
            if (retVal == false)
            {
                break;
            }

            /* Get the weight of the received packet after sending and receive the packet */
            weight = ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_NONE;

            retVal = ICSS_EMAC_SendPktChkRx(icssEmacHandlePtr, pruIcssHandlePtr, portNum, &ICSS_EMAC_testPkt1VlanUntagged[0], sizeof(ICSS_EMAC_testPkt1VlanUntagged));
            if (retVal == true)
            {
                weight += ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_UNTAGGED_WEIGHT;
            }

            retVal = ICSS_EMAC_SendPktChkRx(icssEmacHandlePtr, pruIcssHandlePtr, portNum, &ICSS_EMAC_testPkt1VlanTagged[0], sizeof(ICSS_EMAC_testPkt1VlanTagged));
            if (retVal == true)
            {
                weight += ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_TAGGED_WEIGHT;
            }

            retVal = ICSS_EMAC_SendPktChkRx(icssEmacHandlePtr, pruIcssHandlePtr, portNum, &ICSS_EMAC_testPkt1VlanPriority[0], sizeof(ICSS_EMAC_testPkt1VlanPriority));
            if (retVal == true)
            {
                weight += ICSS_EMAC_VLAN_FILTER_TEST_RX_PKT_PRIORITY_WEIGHT;
            }

            if (weight != testVlanFilterCfg[testCfgIndex].expectedRxPktWeight)
            {
                retVal = false;
                DebugP_log("    VLAN filtering test Failed for Configuration %d (weight mismatch - expected %d, obtained %d) \r\n", testCfgIndex, testVlanFilterCfg[testCfgIndex].expectedRxPktWeight, weight );
                break;
            }
            else
            {
               /* Test passed with expected results, continue for next test configuration */
               retVal = true;
               DebugP_log("    VLAN filtering test Passed for Configuration %d \r\n", testCfgIndex);
            }
        }

        /* Disable the promiscuous mode for the ports to get back to original state */
        if (retVal == true)
        {
            promisEnableFlag = 0;    /* enable promiscuous mode */
            ioctlParams.command = 0;
            ioctlParams.ioctlVal = (void *)(&promisEnableFlag);
            retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruIcssHandlePtr, ICSS_EMAC_IOCTL_PROMISCUOUS_CTRL, portNum, &ioctlParams);
        }
    }

    /* Return overall test result */
    return (retVal);
}

static bool ICSS_EMAC_vlan_filter_feature_config(ICSS_EMAC_Handle *icssEmacHandlePtr, PRUICSS_Handle *pruHandlePtr, uint8_t portNum, testVlanCfg_t *vlanCfg)
{
    uint32_t          i;
    int32_t           vlanFilterCmd[ICSS_EMAC_TEST_MAX_VLAN_CMDS];
    ICSS_EMAC_IoctlCmd ioctlParams;
    bool              retVal = true;


    /* Get the commands */
    vlanFilterCmd[0] = vlanCfg->vlanFilterEnable;
    vlanFilterCmd[1] = vlanCfg->vlanAddRemoveCfg;
    vlanFilterCmd[2] = vlanCfg->unTagPktCfg;
    vlanFilterCmd[3] = vlanCfg->prioPktCfg;
    for (i = 0; i < ICSS_EMAC_TEST_MAX_VLAN_CMDS; i++)
    {
        switch(vlanFilterCmd[i])
        {
            case ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_ENABLE:
                ioctlParams.command = ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_ENABLE_CMD;
                retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruHandlePtr, ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL, portNum, &ioctlParams);
                break;

            case ICSS_EMAC_VLAN_FILTER_TEST_FEATURE_DISABLE:
                ioctlParams.command = ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_DISABLE_CMD;
                retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruHandlePtr, ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL, portNum, &ioctlParams);
                break;

            case ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_PRIO_HOST_RX:
                ioctlParams.command = ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_PRIOTAG_HOST_RCV_ALL_CMD;
                retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruHandlePtr, ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL, portNum, &ioctlParams);
                break;

            case ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_PRIO_HOST_RX:
                ioctlParams.command = ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_PRIOTAG_HOST_RCV_NAL_CMD;
                retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruHandlePtr, ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL, portNum, &ioctlParams);
                break;

            case ICSS_EMAC_VLAN_FILTER_TEST_ALLOW_UNTAGGED_HOST_RX:
                ioctlParams.command = ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_UNTAG_HOST_RCV_ALL_CMD;
                retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruHandlePtr, ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL, portNum, &ioctlParams);
                break;

            case ICSS_EMAC_VLAN_FILTER_TEST_NOT_ALLOW_UNTAGGED_HOST_RX:
                ioctlParams.command = ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_UNTAG_HOST_RCV_NAL_CMD;
                retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruHandlePtr, ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL, portNum, &ioctlParams);
                break;

            case ICSS_EMAC_VLAN_FILTER_TEST_VID_ADD_VLAN_ID:
                ioctlParams.command  = ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_ADD_VID_CMD;
                ioctlParams.ioctlVal = (void *) &vlanCfg->vid;
                retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruHandlePtr, ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL, portNum, &ioctlParams);
                break;

            case ICSS_EMAC_VLAN_FILTER_TEST_VID_REMOVE_VLAN_ID:
                ioctlParams.command = ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL_REMOVE_VID_CMD;
                ioctlParams.ioctlVal = (void *) &vlanCfg->vid;
                retVal = ICSS_EMAC_firmware_feature_ctrl(icssEmacHandlePtr, pruHandlePtr, ICSS_EMAC_IOCTL_VLAN_FILTER_CTRL, portNum, &ioctlParams);
                break;

            case ICSS_EMAC_VLAN_FILTER_TEST_VID_NOT_CONFIGURED:
                /* No action */
            default:
                break;
        }
    }

    return(retVal);

}

static bool ICSS_EMAC_firmware_feature_ctrl(ICSS_EMAC_Handle *eHandle, PRUICSS_Handle *hdl, uint32_t ioctlCmd, uint8_t portNum, ICSS_EMAC_IoctlCmd *ioctlParams)
{
    bool   retVal;
    PRUICSS_Handle handle = *hdl;
    ICSS_EMAC_Handle icssEmacHandle = *eHandle;
#ifdef SWITCH_EMAC
    retVal = false;
#else
    int8_t ret_Val = 0;

    PRUICSS_disableCore(handle, ICSS_EMAC_PORT_1-1);
    PRUICSS_disableCore(handle, ICSS_EMAC_PORT_2-1);

    PRUICSS_resetCore(handle, ICSS_EMAC_PORT_1-1);
    PRUICSS_resetCore(handle, ICSS_EMAC_PORT_2-1);

    ret_Val = ICSS_EMAC_ioctl(icssEmacHandle, ioctlCmd, portNum, ioctlParams);
    if (ret_Val != 0)
    {
        retVal = false;
    }
    else
    {
        retVal = true;
    }

    PRUICSS_enableCore(handle, ICSS_EMAC_PORT_1-1);
    PRUICSS_enableCore(handle, ICSS_EMAC_PORT_2-1);
#endif
    return (retVal);
}

static bool ICSS_EMAC_SendPktChkRx(ICSS_EMAC_Handle *icssEmacHandlePtr, PRUICSS_Handle *pruHandlePtr, uint32_t portNum, uint8_t *testVlanpkt, uint32_t pktSize)
{
    ICSS_EMAC_TxArgument  txArgs;
    uint32_t             count, fail_count, gFail_count = 0;
    uint32_t             *rxPktCnt;
    bool                 retVal = true;
    ICSS_EMAC_Handle      icssEmacHandle = *icssEmacHandlePtr;

    /* Clear the arguments */
    memset(&txArgs, 0, sizeof(ICSS_EMAC_TxArgument));

    if (portNum == ICSS_EMAC_PORT_1)
    {
        rxPktCnt = &ICSS_EMAC_testPacketRcvdPort2;
    }
    else
    {
        rxPktCnt = &ICSS_EMAC_testPacketRcvdPort3;
    }

    *rxPktCnt = 1;
    txArgs.icssEmacHandle = icssEmacHandle;
    txArgs.lengthOfPacket = pktSize;
    txArgs.portNumber = portNum;
    txArgs.queuePriority = 3;
    txArgs.srcAddress = testVlanpkt;

    for (count=0;count < ICSS_EMAC_TEST_PKT_TX_COUNT;count++)
    {

        if(((portNum == ICSS_EMAC_PORT_1) && (ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)) ||
           ((portNum == ICSS_EMAC_PORT_2) && (ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_GET_LINK_STATUS, NULL, 0) == SystemP_SUCCESS)))
        {
            if(*rxPktCnt )
            {
                *rxPktCnt = 0;
                ICSS_EMAC_txPacket(&txArgs, NULL);
                fail_count = 0;
                while(*rxPktCnt == 0U)
                {
                    /*TODO: Review this*/
                    ClockP_usleep(10000);
                    fail_count ++;
                    if (fail_count > 2)
                    {
                        gFail_count ++;
                        *rxPktCnt = 1;
                        break;
                    }
                }
            }
        }
    }

    if (gFail_count == ICSS_EMAC_TEST_PKT_TX_COUNT)
    {
        retVal = false;
    }
    else
    {
        retVal = true;
    }

    return (retVal);

}

bool ICSS_EMAC_testRxPktParser(ICSS_EMAC_Handle handle, void *pIcssRxPktInfo)
{
    char *pTemp = (char*) pIcssRxPktInfo;

    uint16_t ethType;
    ethType = *(uint16_t *)(pTemp  +  2 * ICSS_EMAC_TEST_ETH_ALEN);
    ethType = ICSS_EMAC_TEST_BYTESWAP16(ethType);

    /* to support VLAN filtering */
    if (ethType == ICSS_EMAC_TEST_VLAN_ETH_TYPE)
    {
        /* read the actual ethtype at an offset of 4 */
        ethType = *(uint16_t *)(pTemp  +  (2 * ICSS_EMAC_TEST_ETH_ALEN) + 4);
        ethType = ICSS_EMAC_TEST_BYTESWAP16(ethType);
    }

    if (ethType == ICSS_EMAC_TEST_ETHER_TYPE)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }

}
