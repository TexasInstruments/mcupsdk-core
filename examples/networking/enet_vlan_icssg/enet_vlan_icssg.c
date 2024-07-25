/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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

/*!
 * \file  enet_unit_test_icssg.c
 *
 * \brief This file contains the implementation of ICSSG unit test application
 * to verify UNH-IOL VLAN conformance and FDB conformance.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/sciclient.h>
#include <drivers/udma/udma_priv.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/icssg.h>
#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>
#include <enet_appboardutils.h>
/* SDK includes */
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Max number of ports supported per context */
#define ENETMP_PORT_MAX                          (2U)

/* Max number of hardware RX channels. Note that this is different than Enet LLD's
 * RX channel concept which maps to UDMA hardware RX flows */
#define ENETMP_HW_RXCH_MAX                       (2U)

/* Local flag to disable a peripheral from test list */
#define ENETMP_DISABLED                          (0U)

/* Max number of ports that can be tested with this app */
#define ENETMP_ICSSG_INSTANCE_MAX                (2U)
#define ENETMP_PER_MAX                           (ENETMP_ICSSG_INSTANCE_MAX)

/* Task stack size */
#define ENETMP_TASK_STACK_SZ                     (10U * 1024U)

/* 100-ms periodic tick */
#define ENETMP_PERIODIC_TICK_MS                  (100U)

/*Counting Semaphore count*/
#define COUNTING_SEM_COUNT                       (10U)

/* VLAN id for testing purposes */
#define ENETMP_VLAN_ID                           (100U)

/* The maximum no. of test cases that the test application supports in automated test execution mode */
#define MAX_NO_OF_TEST_CASES                     (5U)

/* The maximum no. of VLAN test cases */
#define MAX_NO_OF_VLAN_TEST_CASES                (21)
#define MAX_NO_OF_FDB_TEST_CASES                 (25)
#define MAX_NO_OF_UTILS_CASES                    (9)

#define DEFAULT_FDB_VID                          (1)
#define DEFAULT_FDB_PCP                          (1)

#define TEST_SUCCESS                             (0)
#define TEST_FAILURE                             (-1)

#define NRT_FID                                  (0)

/* Bit#0 Indicates host ownership (indicates Host egress) */
#define FDB_ENTRY_HOST_BIT                       (0)

/* Bit#1 Indicates that MAC ID is connected to Physical Port 1 */
#define FDB_ENTRY_PORT1_BIT                      (1)

/* Bit#2 Indicates that MAC ID is connected to Physical Port 2 */
#define FDB_ENTRY_PORT2_BIT                      (2)

/* Bit#3 This is set to 1 for all learnt entries. 0 for static entries. */
#define FDB_ENTRY_LEARNT_ENTRY_BIT               (3)

/* Bit#4 If set for SA then packet is dropped (can be used to implement a blacklist).
 * If set for DA then packet is determined to be a special packet */
#define FDB_ENTRY_BLOCK_BIT                      (4)

/* Bit#5 If set for DA then the SA from the packet is not learnt */

/* Bit#6 if set, it means packet has been seen recently with source address +
 * FID matching MAC address/FID of entry. */
#define FDB_ENTRY_TOUCH_BIT                      (6)

/* Bit#7 set if entry is valid */
#define FDB_ENTRY_VALID_BIT                      (7)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Test Functions Array.
 * Describes the array of test functions which are called in sequence based on user input */
typedef int32_t (*test_func)(void);

typedef enum testFunctionCategory
{
    /** Unit test case functions */
    TEST_CASE = 0x1U,
    /** Unit test configuration functions */
    TEST_CONFIG = 0x2U,
    /** Utility functions */
    TEST_UTILS = 0x3U,
} testFunctionCategory_t;

typedef struct
{
    test_func   test_function;
    char        test_name[64];
    uint16_t    test_category;
} unitTestFunction_s;

typedef struct
{
    test_func testfunc;
    char TestStr[100];
} FDBTestFunc_t;

typedef struct
{
    test_func test_function;
    char test_name[100];
} UTILSfunc_t;

/** @brief Struct of test functions with name and category*/
typedef struct
{
    test_func testfunc;
    char TestStr[100];
} VLANTestFunc_t;

/* Test parameters for each port in the multiport test */
typedef struct EnetMp_TestParams_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort[ENETMP_PORT_MAX];

    /* Number of MAC ports in macPorts array */
    uint32_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;
} EnetMp_TestParams;

/* Context of a peripheral/port */
typedef struct EnetMp_PerCtxt_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort[ENETMP_PORT_MAX];

    /* Number of MAC ports in macPorts array */
    uint32_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;

    /* ICSSG configuration */
    Icssg_Cfg icssgCfg;

    /* Number of valid MAC address entries present in macAddr variable below*/
    uint8_t numValidMacAddress;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* UDMA driver configuration */
    EnetUdma_Cfg dmaCfg;

    /* TX channel number */
    uint32_t txChNum;

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;

    /* Start flow index */
    uint32_t rxStartFlowIdx[ENETMP_HW_RXCH_MAX];

    /* Flow index */
    uint32_t rxFlowIdx[ENETMP_HW_RXCH_MAX];

    /* RX channel handle */
    EnetDma_RxChHandle hRxCh[ENETMP_HW_RXCH_MAX];

    /* Number of RX channels in hardware. This value is 1 for all peripherals,
     * except for ICSSG Switch where there are two UDMA RX channels */
    uint32_t numHwRxCh;

    /* RX task handle - receives packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* Semaphore posted from RX callback when packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Semaphore posted from event callback upon asynchronous IOCTL completion */
    SemaphoreP_Object ayncIoctlSemObj;

    /* Timestamp of the last received packets */
    uint64_t rxTs[ENET_SYSCFG_TOTAL_NUM_RX_PKT];

    /* Timestamp of the last transmitted packets */
    uint64_t txTs[ENET_SYSCFG_TOTAL_NUM_RX_PKT];

    /* Sequence number used as cookie for timestamp events. This value is passed
     * to the DMA packet when submitting a packet for transmission. Driver will
     * pass this same value when timestamp for that packet is ready. */
    uint32_t txTsSeqId;

    /* Semaphore posted from event callback upon asynchronous IOCTL completion */
    SemaphoreP_Object txTsSemObj;

    /* Enet and Udma handle info for the peripheral */
    EnetApp_HandleInfo handleInfo;

   /* Core attach info for the peripheral */
    EnetPer_AttachCoreOutArgs attachInfo;

} EnetMp_PerCtxt;

typedef struct EnetMp_Obj_s
{
    /* Flag which indicates if test shall run */
    volatile bool run;

    /* This core's id */
    uint32_t coreId;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Array of all peripheral/port contexts used in the test */
    EnetMp_PerCtxt perCtxt[ENETMP_PER_MAX];

    /* Number of active contexts being used */
    uint32_t numPerCtxts;

    /* Whether promiscuous mode is enabled or not */
    bool promisc;

    /* Whether timestamp are enabled or not */
    bool enableTs;
} EnetMp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetMp_mainTask(void *args);

static void EnetMp_init(void);

static int32_t EnetMp_open(EnetMp_PerCtxt *perCtxts,
                           uint32_t numPerCtxts);

static void EnetMp_close(EnetMp_PerCtxt *perCtxts,
                         uint32_t numPerCtxts);

static int32_t EnetMp_openPort(EnetMp_PerCtxt *perCtxt);

static void EnetMp_togglePromisc(EnetMp_PerCtxt *perCtxts,
                                 uint32_t numPerCtxts);

static void EnetMp_printStats(EnetMp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts);

static void EnetMp_resetStats(EnetMp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts);

static void EnetMp_showMacAddrs(EnetMp_PerCtxt *perCtxts,
                                uint32_t numPerCtxts);

static int32_t EnetMp_waitForLinkUp(EnetMp_PerCtxt *perCtxt);

static void EnetMp_macMode2MacMii(emac_mode macMode,
                                  EnetMacPort_Interface *mii);

static int32_t EnetMp_openDma(EnetMp_PerCtxt *perCtxt,uint32_t  perCtxtIndex);

static void EnetMp_closeDma(EnetMp_PerCtxt *perCtxt,uint32_t  perCtxtIndex);

static void EnetMp_initTxFreePktQ(void);

static void EnetMp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

static uint32_t EnetMp_retrieveFreeTxPkts(EnetMp_PerCtxt *perCtxt);

static void EnetMp_createRxTask(EnetMp_PerCtxt *perCtxt,
                                uint8_t *taskStack,
                                uint32_t taskStackSize);

static void EnetMp_destroyRxTask(EnetMp_PerCtxt *perCtxt);

static void EnetMp_rxTask(void *args);

extern uint32_t Board_getPhyAddr(void);

void runTest(void);
int32_t TC_VLAN_testing(void);
int32_t TC_FDB_testing(void);
int32_t UTILS_testing(void);

int32_t TC_VLAN_1_1(void);
int32_t TC_VLAN_1_2_A(void);
int32_t TC_VLAN_1_2_B(void);
int32_t TC_VLAN_1_2_C(void);
int32_t TC_VLAN_1_2_D(void);
int32_t TC_VLAN_1_3_C(void);
int32_t TC_VLAN_1_4_A(void);
int32_t TC_VLAN_1_4_B(void);
int32_t TC_VLAN_1_4_C(void);
int32_t TC_VLAN_1_4_D(void);
int32_t TC_VLAN_1_5_B(void);
int32_t TC_VLAN_2_1_A(void);
int32_t TC_VLAN_2_1_B(void);
int32_t TC_VLAN_2_1_C(void);
int32_t TC_VLAN_2_1_D(void);
int32_t TC_VLAN_2_2_A(void);
int32_t TC_VLAN_2_2_B(void);
int32_t TC_VLAN_2_3(void);
int32_t TC_VLAN_2_4(void);
int32_t TC_VLAN_2_5(void);
int32_t TC_VLAN_3_4(void);
int32_t TC_undef(void);

int32_t TC_FDB_1_1_A(void);
int32_t TC_FDB_1_1_B(void);
int32_t TC_FDB_1_4(void);
int32_t TC_FDB_2_1_A(void);
int32_t TC_FDB_2_1_B(void);
int32_t TC_FDB_2_1_C(void);
int32_t TC_FDB_2_1_D(void);
int32_t TC_FDB_2_2_A(void);
//int32_t TC_FDB_2_2_B(void);
//int32_t TC_FDB_2_2_C(void);

int32_t TC_FDB_2_4_A(void);
int32_t TC_FDB_2_6_A(void);
int32_t TC_FDB_2_6_B(void);
int32_t TC_FDB_2_6_C(void);
int32_t TC_FDB_2_6_D(void);
int32_t TC_FDB_2_8(void);
int32_t TC_FDB_3_1(void);
int32_t TC_FDB_3_2_A(void);
int32_t TC_FDB_3_2_B(void);
int32_t TC_FDB_3_2_C(void);
int32_t TC_FDB_3_2_D(void);
int32_t TC_FDB_4_2_B(void);
int32_t TC_FDB_4_3_A(void);
int32_t Setting_fdb_ageing_timeout(void);
int32_t add_mac_fdb_entry(Icssg_MacAddr mac, int16_t vlanId, uint8_t fdbEntry_port);
int32_t del_mac_fdb_entry(Icssg_MacAddr mac, int16_t vlanId);
int32_t set_ageing_timout(uint64_t timeout);

int32_t UTILS_display_icssg_hw_consolidated_statistics(void);
int32_t UTILS_clear_icssg_hw_consolidated_statistics(void);
int32_t UTILS_NRT_cofig_port_state(void);
void UTILS_icssg_hw_consolidated_statistics(uint32_t portNum);
void UTILS_icssg_hw_reset_consolidated_statistics(uint32_t portNum);
int32_t TC_configure_vlan_aware(void);

int32_t add_default_port_vid(uint8_t port_num, uint8_t pcp, uint16_t vlan_id);
int32_t add_default_host_vid(uint8_t pcp, uint16_t vlan_id);
int32_t add_fid_vid_entry(uint16_t vlan_id, uint8_t fid, uint8_t hostMember, uint8_t memberP1, uint8_t memberP2, uint8_t host_tagged, uint8_t taggedP1, uint8_t taggedP2, uint8_t streamVid, uint8_t floodToHost);
int32_t set_acceptable_frame_type(uint8_t port_num, uint32_t acceptableFrameType, Icssg_AcceptFrameCheck configparam);
int32_t vlan_default_config(void);
int32_t set_priority_reg_mapping(uint8_t port_num, uint32_t *prioRegenMap);
int32_t set_priority_mapping(uint8_t port_num, uint32_t *prioMap);
int32_t fdb_default_config(void);
int32_t set_port_state(uint8_t port_num, Icssg_PortState port_state);
int32_t UTILS_NRT_add_del_mac_fdb_entry(void);
int32_t UTILS_NRT_add_fid_vid_entry(void);
int32_t setup_unit_test_default_settings(void);
int32_t UTILS_config_get_rx_pkt_count(void);
int32_t UTILS_config_clear_rx_pkt_count(void);
int32_t UTILS_config_PVID(void);
int32_t UTILS_Transmit_UC_packets(void);
void UTILS_packetTx(uint8_t port_num);

/*!
 * @brief To enable/disable vlan aware mode
 * @details To enable/disable vlan aware mode
 * @param[in] mode - Vlan aware disable/ enable
 * @retval @ref TEST_FAILURE if the operation not completed successfully
 * @retval @ref TEST_SUCCESS if the operation is completed successfully.
 */
int32_t set_vlan_aware_mode(uint8_t port_num, int32_t mode);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** @brief Array of test functions which are executed in sequence based on user input*/
unitTestFunction_s enet_icssg_ut[MAX_NO_OF_TEST_CASES];

/** @brief Array of test functions which are executed in sequence based on user input*/
unitTestFunction_s enet_icssg_ut[MAX_NO_OF_TEST_CASES] =
{
    {TC_VLAN_testing, "TC_VLAN_testing", TEST_CASE},
    {TC_FDB_testing, "TC_FDB_testing", TEST_CASE},
    {UTILS_testing, "UTILS_testing", TEST_CASE},
    {TC_configure_vlan_aware, "TC_configure_vlan_aware_unaware", TEST_CASE},
    {setup_unit_test_default_settings, "setup_unit_test_default_settings", TEST_CASE}
};

/** @brief Array of VLAN test functions which are executed in sequence based on user input*/
VLANTestFunc_t VLANTestFunc[MAX_NO_OF_VLAN_TEST_CASES + 1] =
{
    {TC_VLAN_1_1, "VLAN Classification "},
    {TC_VLAN_1_2_A, "Acceptable Frame Types Parameter: Part A: Default Behavior"},
    {TC_VLAN_1_2_B, "Acceptable Frame Types Parameter: Part B: Admit All Frames"},
    {TC_VLAN_1_2_C, "Acceptable Frame Types Parameter: Part C: Admit only VLAN-Tagged Frames"},
    {TC_VLAN_1_2_D, "Acceptable Frame Types Parameter: Part D: Admit only Untagged and Priority-Tagged Frames"},
    {TC_VLAN_1_3_C, "Enable Ingress Filtering: Part C"},
    {TC_VLAN_1_4_A, "PVID Configured through Management: Part A: setting the untagged member set and PVID to invalid VID"},
    {TC_VLAN_1_4_B, "PVID Configured through Management: Part B: Setting the Tagged Member Set to Invalid VIDs"},
    {TC_VLAN_1_4_C, "PVID Configured through Management: Part C: Setting the Untagged Member Set and PVID to Valid VIDs"},
    {TC_VLAN_1_4_D, "PVID Configured through Management: Part D: Setting the Tagged Member Set to Valid VIDs"},
    {TC_VLAN_1_5_B, "PVID Assigned to a Port in no VLAN Member Set: Part B"},
    {TC_VLAN_2_1_A, "Minimum Frame Size: Part A: Minimum Untagged Size"},
    {TC_VLAN_2_1_B, "Minimum Frame Size: Part B: Less Than Minimum Untagged Size"},
    {TC_VLAN_2_1_C, "Minimum Frame Size: Part C: Minimum Tagged Size"},
    {TC_VLAN_2_1_D, "Minimum Frame Size: Part D: Less Than Minimum Tagged Size"},
    {TC_VLAN_2_2_A, "Maximum Tagged Size: Part A: Maximum Tagged Size"},
    {TC_VLAN_2_2_B, "Maximum Tagged Size: Part B: Maximum Tagged Size Exceeded"},
    {TC_VLAN_2_3, "Untagging Minimum-Sized Tagged Frames"},
    {TC_VLAN_2_4, "Regenerating User Priority"},
    {TC_VLAN_2_5, "Bad FCS Received"},
    {TC_VLAN_3_4, "Recalculating FCS"},
    {TC_undef, "Test Case undefined"},
};
/** @brief Array of FDB test functions which are executed in sequence based on user input*/
FDBTestFunc_t FDBTestFunc[MAX_NO_OF_FDB_TEST_CASES + 1] =
{
    {TC_FDB_1_1_A, "Entry for Default PVID: Part A"},
    {TC_FDB_1_1_B, "Entry for Default PVID: Part B"},
    {TC_FDB_1_4, "Entry Indicating Forward All Groups"},
    {TC_FDB_2_1_A, "Learning Based on Port State: Part A"},
    {TC_FDB_2_1_B, "Learning Based on Port State: Part B"},
    {TC_FDB_2_1_C, "Learning Based on Port State: Part C"},
    {TC_FDB_2_1_D, "Learning Based on Port State: Part D"},
    {TC_FDB_2_4_A, "Learning Based on VID: Part A"},
    {TC_FDB_2_6_A, "Learning from Frames Discarded on Ingress: Part A"},
    {TC_FDB_2_6_B, "Learning from Frames Discarded on Ingress: Part B"},
    {TC_FDB_2_6_C, "Learning from Frames Discarded on Ingress: Part C"},
    {TC_FDB_2_6_D, "Learning from Frames Discarded on Ingress: Part D"},
    {TC_FDB_2_8, "New Entry Prevented due to Static Filtering Entry"},
    {TC_FDB_3_1, "Duplicate Static Filtering Entries"},
    {TC_FDB_3_2_A, "Basic Filtering Services: Part A"},
    {TC_FDB_3_2_B, "Basic Filtering Services: Part B"},
    {TC_FDB_3_2_C, "Basic Filtering Services: Part C"},
    {TC_FDB_3_2_D, "Basic Filtering Services: Part D"},
    {TC_FDB_4_2_B, "Static Filtering Entries Not Aged Out: Part B"},
    {TC_FDB_4_3_A, "Static VLAN Registration Entries Not Aged Out: Part A"},
    {Setting_fdb_ageing_timeout, "Setting_fdb_ageing_timeout"},
    {TC_undef, "Test Case undefined"},
};

UTILSfunc_t UtilsTestFunc[MAX_NO_OF_UTILS_CASES + 1] =
{
     {UTILS_display_icssg_hw_consolidated_statistics, "UTILS_display_icssg_hw_consolidated_statistics"},
     {UTILS_clear_icssg_hw_consolidated_statistics, "UTILS_clear_icssg_hw_consolidated_statistics"},
     {UTILS_NRT_add_del_mac_fdb_entry, "UTILS_NRT_add_del_mac_fdb_entry"},
     {UTILS_NRT_cofig_port_state, "UTILS_NRT_cofig_port_state"},
     {UTILS_NRT_add_fid_vid_entry, "UTILS_NRT_add_fid_vid_entry for PORT1 and PORT2"},
     {UTILS_config_PVID, "UTILS_config_PVID"},
     {UTILS_config_get_rx_pkt_count, "UTILS_get_receive_packet_count"},
     {UTILS_config_clear_rx_pkt_count, "UTILS_clear_receive_packet_count"},
     {UTILS_Transmit_UC_packets, "UTILS_Transmit_UC_packets"},
     {TC_undef, "TC_undef"},
};

int32_t uartScanTemp = 0;
int32_t uartScanTemp1 = 0;
uint16_t new_common_vlan_id = 100;
uint32_t totalRxCnt = 0U;
uint8_t txDstMac[ENET_MAC_ADDR_LEN] = { 0x00, 0x02, 0x03, 0x04, 0x05, 0x06 };
uint16_t txEthType = 0x0800;
uint8_t ENET_testPkt1[] =
{
    0x45, 0x00, 0x00, 0x6a, 0x00, 0x54,
    0x00, 0x00, 0xff, 0xfd, 0x38, 0xea,
    0xc0, 0x55, 0x01, 0x02, 0xc0, 0x00,
    0x00, 0x01, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd,
    0xdd, 0xdd, 0xab, 0x44, 0x50, 0x3e,
    0x03, 0x1c, 0x43, 0x59, 0xb0, 0xf3,
    0xd1, 0x49, 0x7d, 0x44, 0xa1, 0x33,
    0xe9, 0xa5, 0x33, 0xac, 0xb1, 0x31,
    0x49, 0x7b
};

/* Enet multiport test object */
EnetMp_Obj gEnetMp;

/* Statistics */
IcssgStats_MacPort gEnetMp_icssgStats;
IcssgStats_Pa gEnetMp_icssgPaStats;

/* Test application stack */
static uint8_t gEnetMpTaskStackRx[ENETMP_PER_MAX][ENETMP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

// #define DUAL_MAC_MODE  /* TODO: Need to allocate TX channels as 2 in enet_cfg.h file */
/* Use this array to select the ports that will be used in the test */
static EnetMp_TestParams testParams[] =
{
#if defined(DUAL_MAC_MODE)
    { ENET_ICSSG_DUALMAC, 2U, { ENET_MAC_PORT_1 }, 1U, "icssg1-p1", },
    { ENET_ICSSG_DUALMAC, 3U, { ENET_MAC_PORT_1 }, 1U, "icssg1-p2", },
#else
    { ENET_ICSSG_SWITCH, 1U, { ENET_MAC_PORT_1, ENET_MAC_PORT_2 }, 2U, "icssg1", },
#endif
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetMp_mainTask(void *args)
{
    uint32_t i, j;
    int32_t status = ENET_SOK;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("================================\r\n");
    DebugP_log("      ENET ICSSG UNIT TEST      \r\n");
    DebugP_log("================================\r\n");

    /* Initialize test config */
    memset(&gEnetMp, 0, sizeof(gEnetMp));
    gEnetMp.run = true;
    gEnetMp.promisc = false;
    gEnetMp.enableTs = false;

    gEnetMp.numPerCtxts = ENET_ARRAYSIZE(testParams);

    for (i = 0U; i < gEnetMp.numPerCtxts; i++)
    {
        gEnetMp.perCtxt[i].enetType = testParams[i].enetType;
        gEnetMp.perCtxt[i].instId   = testParams[i].instId;
        gEnetMp.perCtxt[i].name     = testParams[i].name; /* shallow copy */

        gEnetMp.perCtxt[i].macPortNum = testParams[i].macPortNum;
        for (j = 0; j < gEnetMp.perCtxt[i].macPortNum; j++)
        {
            gEnetMp.perCtxt[i].macPort[j]  = testParams[i].macPort[j];
        }
    }

    /* Init driver */
    EnetMp_init();

    status = EnetMp_open(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
    if (status != ENET_SOK)
    {
        DebugP_log("Failed to open peripherals: %d\r\n", status);
    }

    if (status == ENET_SOK)
    {
        /* Start periodic tick timer */


        /* Call UT's main function */
        runTest();

        /* Print statistics */
        EnetMp_printStats(gEnetMp.perCtxt, gEnetMp.numPerCtxts);

        /* Wait until RX tasks have exited */
        for (i = 0U; i < gEnetMp.numPerCtxts; i++)
        {
            DebugP_log("Waiting for RX task %u to exit\r\n", i+1);
            SemaphoreP_post(&gEnetMp.perCtxt[i].rxSemObj);
            SemaphoreP_pend(&gEnetMp.perCtxt[i].rxDoneSemObj, SystemP_WAIT_FOREVER);
        }

        DebugP_log("All RX tasks have exited\r\n");
    }

    /* Close all peripherals */
    EnetMp_close(gEnetMp.perCtxt, gEnetMp.numPerCtxts);
}

static void EnetMp_init(void)
{
    gEnetMp.coreId = EnetSoc_getCoreId();

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetMp.txFreePktInfoQ);
}

static void EnetMp_asyncIoctlCb(Enet_Event evt,
                                uint32_t evtNum,
                                void *evtCbArgs,
                                void *arg1,
                                void *arg2)
{
    EnetMp_PerCtxt *perCtxt = (EnetMp_PerCtxt *)evtCbArgs;

    DebugP_log("%s: Async IOCTL completed\r\n", perCtxt->name);
    SemaphoreP_post(&perCtxt->ayncIoctlSemObj);
}

static void EnetMp_txTsCb(Enet_Event evt,
                          uint32_t evtNum,
                          void *evtCbArgs,
                          void *arg1,
                          void *arg2)
{
    EnetMp_PerCtxt *perCtxt = (EnetMp_PerCtxt *)evtCbArgs;
    Icssg_TxTsEvtCbInfo *txTsInfo = (Icssg_TxTsEvtCbInfo *)arg1;
    Enet_MacPort macPort = *(Enet_MacPort *)arg2;
    uint32_t tsId = txTsInfo->txTsId;
    uint64_t txTs = txTsInfo->ts;
    uint64_t rxTs = perCtxt->rxTs[tsId % ENET_SYSCFG_TOTAL_NUM_RX_PKT];
    uint64_t prevTs;
    int64_t dt;
    bool status = true;

    dt = txTs - rxTs;

    DebugP_log("%s: Port %u: RX-to-TX timestamp delta = %10lld (RX=%llu, TX=%llu)\r\n",
                       perCtxt->name, ENET_MACPORT_ID(macPort), dt, rxTs, txTs);

    /* Check correct timestamp delta */
    if (dt < 0)
    {
        DebugP_log("%s: Port %u: ERROR: RX timestamp > TX timestamp: %llu > %llu\r\n",
                           perCtxt->name, ENET_MACPORT_ID(macPort), rxTs, txTs);
            status = false;
    }

    /* Check monotonicity of the TX and RX timestamps */
    if (txTsInfo->txTsId > 0U)
    {
        prevTs = perCtxt->rxTs[(tsId - 1) % ENET_SYSCFG_TOTAL_NUM_RX_PKT];
        if (prevTs > rxTs)
        {
            DebugP_log("%s: Port %u: ERROR: Non monotonic RX timestamp: %llu -> %llu\r\n",
                               perCtxt->name, ENET_MACPORT_ID(macPort), prevTs, rxTs);
            status = false;
        }

        prevTs = perCtxt->txTs[(tsId - 1) % ENET_SYSCFG_TOTAL_NUM_RX_PKT];
        if (prevTs > txTs)
        {
            DebugP_log("%s: Port %u: ERROR: Non monotonic TX timestamp: %llu -> %llu\r\n",
                               perCtxt->name, ENET_MACPORT_ID(macPort), prevTs, txTs);
            status = false;
        }
    }

    if (!status)
    {
        DebugP_log("\r\n");
    }

    /* Save current timestamp for future monotonicity checks */
     perCtxt->txTs[txTsInfo->txTsId % ENET_SYSCFG_TOTAL_NUM_RX_PKT] = txTs;

    SemaphoreP_post(&perCtxt->txTsSemObj);
}

static void EnetMp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                          bool isLinkUp,
                                          void *appArg)
{
    DebugP_log("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

int32_t EnetMp_getPerIdx(Enet_Type enetType, uint32_t instId, uint32_t *perIdx)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Initialize async IOCTL and TX timestamp semaphores */
    for (i = 0U; i < gEnetMp.numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &(gEnetMp.perCtxt[i]);
        if ((perCtxt->enetType == enetType) && (perCtxt->instId == instId))
        {
            break;
        }
    }
    if (i < gEnetMp.numPerCtxts)
    {
        *perIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_ENOTFOUND;
    }
    return status;
}


int32_t EnetMp_mapPerCtxt2Idx(EnetMp_PerCtxt *perCtxt, uint32_t *perIdx)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    for (i = 0; i < gEnetMp.numPerCtxts;i++)
    {
        if (&gEnetMp.perCtxt[i] == perCtxt)
        {
            break;
        }
    }
    if (i < gEnetMp.numPerCtxts)
    {
        *perIdx = i;
    }
    else
    {
        status = ENET_EFAIL;
    }
    return status;
}

void EnetApp_updateIcssgInitCfg(Enet_Type enetType, uint32_t instId, Icssg_Cfg *icssgCfg)
{
    EnetRm_ResCfg *resCfg;
    uint32_t i;
    uint32_t perIdx;
    int32_t status;

    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit  configs EnetType:%u, InstId :%u\r\n", enetType, instId);
    EnetAppUtils_print("----------------------------------------------\r\n");

    resCfg = &icssgCfg->resCfg;

    /* We use software MAC address pool from apputils, but it will give same MAC address.
        * Add port index to make them unique */
    status = EnetMp_getPerIdx(enetType, instId, &perIdx);
    EnetAppUtils_assert(status == ENET_SOK);
    for (i = 0U; i < ENETMP_PORT_MAX; i++)
    {
        resCfg->macList.macAddress[i][ENET_MAC_ADDR_LEN - 1] += (perIdx * ENETMP_PORT_MAX) + i;
    }
    resCfg->macList.numMacAddress = ENETMP_PORT_MAX;
}
static int32_t EnetMp_open(EnetMp_PerCtxt *perCtxts,
                           uint32_t numPerCtxts)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Initialize async IOCTL and TX timestamp semaphores */
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        status = SemaphoreP_constructBinary(&perCtxt->ayncIoctlSemObj, 0);
        DebugP_assert(SystemP_SUCCESS == status);

        status = SemaphoreP_constructBinary(&perCtxt->txTsSemObj, 0);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* Do peripheral dependent initalization */
    DebugP_log("\nInit all peripheral clocks\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_enableClocks(perCtxt->enetType, perCtxt->instId);
    }

    /* Prepare init configuration for all peripherals */
    DebugP_log("\nInit all configs\r\n");
    DebugP_log("----------------------------------------------\r\n");

    EnetApp_driverInit();
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        status = EnetApp_driverOpen(perCtxt->enetType, perCtxt->instId);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        }
        EnetApp_acquireHandleInfo(perCtxt->enetType, perCtxt->instId, &(perCtxt->handleInfo));
    }

    /* Open Enet driver for all peripherals */
    DebugP_log("\nOpen all peripherals\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        if (Enet_isIcssFamily(perCtxt->enetType))
        {
            DebugP_log("%s: Register async IOCTL callback\r\n", perCtxt->name);
            Enet_registerEventCb(perCtxt->handleInfo.hEnet,
                                 ENET_EVT_ASYNC_CMD_RESP,
                                 0U,
                                 EnetMp_asyncIoctlCb,
                                 (void *)perCtxt);

            DebugP_log("%s: Register TX timestamp callback\r\n", perCtxt->name);
            perCtxt->txTsSeqId = 0U;
            Enet_registerEventCb(perCtxt->handleInfo.hEnet,
                                 ENET_EVT_TIMESTAMP_TX,
                                 0U,
                                 EnetMp_txTsCb,
                                 (void *)perCtxt);
        }
    }

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        DebugP_log("\nAttach core id %u on all peripherals\r\n", gEnetMp.coreId);
        DebugP_log("----------------------------------------------\r\n");
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetMp_PerCtxt *perCtxt = &perCtxts[i];

            DebugP_log("%s: Attach core\r\n", perCtxt->name);

            EnetApp_coreAttach(perCtxt->enetType, perCtxt->instId, gEnetMp.coreId, &perCtxt->attachInfo);
        }
    }

    /* Create RX tasks for each peripheral */
    if (status == ENET_SOK)
    {
        DebugP_log("\nCreate RX tasks\r\n");
        DebugP_log("----------------------------------------------\r\n");
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetMp_PerCtxt *perCtxt = &perCtxts[i];

            DebugP_log("%s: Create RX task\r\n", perCtxt->name);

            EnetMp_createRxTask(perCtxt, &gEnetMpTaskStackRx[i][0U], sizeof(gEnetMpTaskStackRx[i]));
        }
    }

    return status;
}

static void EnetMp_close(EnetMp_PerCtxt *perCtxts,
                         uint32_t numPerCtxts)
{
    uint32_t i;
    DebugP_log("\nClose DMA for all peripherals\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        DebugP_log("%s: Close DMA\r\n", perCtxt->name);

        EnetMp_closeDma(perCtxt, i);
    }

    /* Delete RX tasks created for all peripherals */
    DebugP_log("\nDelete RX tasks\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_destroyRxTask(&perCtxts[i]);
    }

    /* Detach core */
    DebugP_log("\nDetach core from all peripherals\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        DebugP_log("%s: Detach core\r\n", perCtxt->name);
        EnetApp_coreDetach(perCtxt->enetType, perCtxt->instId, gEnetMp.coreId, perCtxt->attachInfo.coreKey);
    }

    /* Close opened Enet drivers if any peripheral failed */
    DebugP_log("\nClose all peripherals\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];
        DebugP_log("%s: Close enet\r\n", perCtxt->name);

        if (Enet_isIcssFamily(perCtxt->enetType))
        {
            DebugP_log("%s: Unregister async IOCTL callback\r\n", perCtxt->name);
            Enet_unregisterEventCb(perCtxt->handleInfo.hEnet,
                                   ENET_EVT_ASYNC_CMD_RESP,
                                   0U);

            DebugP_log("%s: Unregister TX timestamp callback\r\n", perCtxt->name);
            Enet_unregisterEventCb(perCtxt->handleInfo.hEnet,
                                   ENET_EVT_TIMESTAMP_TX,
                                   0U);
        }

        EnetApp_releaseHandleInfo(perCtxt->enetType, perCtxt->instId);
        perCtxt->handleInfo.hEnet = NULL;
    }

    EnetApp_driverDeInit();

    /* Do peripheral dependent initalization */
    DebugP_log("\nDeinit all peripheral clocks\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_disableClocks(perCtxt->enetType, perCtxt->instId);
    }
    /* Destroy async IOCTL semaphore */
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &perCtxts[i];

        SemaphoreP_destruct(&perCtxt->ayncIoctlSemObj);
    }
}

static void EnetMp_togglePromisc(EnetMp_PerCtxt *perCtxts,
                                 uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    uint32_t j;
    int32_t status;

    gEnetMp.promisc = !gEnetMp.promisc;
    DebugP_log("\n%s promiscuous mode\r\n", gEnetMp.promisc ? "Enable" : "Disable");

    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];

        /* Promiscuous test in this app not implemented for CPSW, only for ICSSG */
        if (Enet_isIcssFamily(perCtxt->enetType))
        {

            for (j = 0U; j < perCtxt->macPortNum; j++)
            {
                macPort = perCtxt->macPort[j];
                ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
                if(gEnetMp.promisc == 1)
                {
                    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_MACPORT_IOCTL_ENABLE_PROMISC_MODE, &prms, status);
                }
                else
                {
                    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_MACPORT_IOCTL_DISABLE_PROMISC_MODE, &prms, status);
                }

                if (status != ENET_SOK)
                {
                    DebugP_log("%s: Failed to set promisc mode: %d\r\n",
                                       perCtxt->name, status);
                    continue;
                }
            }
        }
    }
}

static void EnetMp_printStats(EnetMp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    uint32_t j;
    int32_t status;

    DebugP_log("\nPrint statistics\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];

        if (Enet_isIcssFamily(perCtxt->enetType))
        {
            DebugP_log("\n %s - PA statistics\r\n", perCtxt->name);
            DebugP_log("--------------------------------\r\n");
            ENET_IOCTL_SET_OUT_ARGS(&prms, &gEnetMp_icssgPaStats);
            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                DebugP_log("%s: Failed to get PA stats\r\n", perCtxt->name);
            }

            EnetAppUtils_printIcssgPaStats(&gEnetMp_icssgPaStats);
            DebugP_log("\r\n");
        }

        for (j = 0U; j < perCtxt->macPortNum; j++)
        {
            macPort = perCtxt->macPort[j];

            DebugP_log("\n %s - Port %u statistics\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
            DebugP_log("--------------------------------\r\n");

            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &gEnetMp_icssgStats);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                DebugP_log("%s: Failed to get port %u stats\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
                continue;
            }

            EnetAppUtils_printIcssgMacPortStats(&gEnetMp_icssgStats, false);

            DebugP_log("\r\n");
        }
    }
}

static void EnetMp_resetStats(EnetMp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    uint32_t j;
    int32_t status;

    DebugP_log("\nReset statistics\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];

        DebugP_log("%s: Reset statistics\r\n", perCtxt->name);

        ENET_IOCTL_SET_NO_ARGS(&prms);
        ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_RESET_HOSTPORT_STATS, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to reset  host port stats\r\n", perCtxt->name);
            continue;
        }
        for (j = 0U; j < perCtxt->macPortNum; j++)
        {
            macPort = perCtxt->macPort[j];

            ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                DebugP_log("%s: Failed to reset port %u stats\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
                continue;
            }
        }
    }
}

static void EnetMp_showMacAddrs(EnetMp_PerCtxt *perCtxts,
                                uint32_t numPerCtxts)
{
    uint32_t i;

    DebugP_log("\nAllocated MAC addresses\r\n");
    DebugP_log("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[i];

        DebugP_log("%s: \t", perCtxt->name);
        EnetAppUtils_printMacAddr(&perCtxt->macAddr[0U]);
    }
}

static int32_t EnetMp_waitForLinkUp(EnetMp_PerCtxt *perCtxt)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    IcssgMacPort_SetPortStateInArgs setPortStateInArgs;
    bool linked;
    uint32_t i;
    int32_t status = ENET_SOK;

    DebugP_log("%s: Waiting for link up...\r\n", perCtxt->name);

    for (i = 0U; i < perCtxt->macPortNum; i++)
    {
        macPort = perCtxt->macPort[i];
        linked = false;

        while (gEnetMp.run && !linked)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                DebugP_log("%s: Failed to get port %u link status: %d\r\n",
                                   perCtxt->name, ENET_MACPORT_ID(macPort), status);
                linked = false;
                break;
            }

            if (!linked)
            {
                ClockP_usleep(1000);
            }
        }

        if (gEnetMp.run)
        {
            DebugP_log("%s: Port %u link is %s\r\n",
                               perCtxt->name, ENET_MACPORT_ID(macPort), linked ? "up" : "down");

            /* Set port to 'Forward' state */
            if (status == ENET_SOK)
            {
                DebugP_log("%s: Set port state to 'Forward'\r\n", perCtxt->name);

                setPortStateInArgs.macPort   = macPort;
                setPortStateInArgs.portState = ICSSG_PORT_STATE_FORWARD;
                ENET_IOCTL_SET_IN_ARGS(&prms, &setPortStateInArgs);

                ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_PER_IOCTL_SET_PORT_STATE, &prms, status);
                if (status == ENET_SINPROGRESS)
                {
                    /* Wait for asyc ioctl to complete */
                    do
                    {
                        Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
                        status = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, SystemP_WAIT_FOREVER);
                        if (SystemP_SUCCESS == status)
                        {
                            break;
                        }
                    } while (1);

                    status = ENET_SOK;
                }
                else
                {
                    DebugP_log("%s: Failed to set port state: %d\n", perCtxt->name, status);
                }
            }
        }
    }

    return status;
}

static void EnetMp_macMode2MacMii(emac_mode macMode,
                                  EnetMacPort_Interface *mii)
{
    switch (macMode)
    {
        case RMII:
            mii->layerType    = ENET_MAC_LAYER_MII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType  = ENET_MAC_VARIANT_NONE;
            break;

        case RGMII:
            mii->layerType    = ENET_MAC_LAYER_GMII;
            mii->sublayerType = ENET_MAC_SUBLAYER_REDUCED;
            mii->variantType  = ENET_MAC_VARIANT_FORCED;
            break;
        default:
            DebugP_log("Invalid MAC mode: %u\r\n", macMode);
            EnetAppUtils_assert(false);
            break;
    }
}


static void EnetMp_rxIsrFxn(void *appData)
{
    EnetMp_PerCtxt *perCtxt = (EnetMp_PerCtxt *)appData;

    SemaphoreP_post(&perCtxt->rxSemObj);
}

#define ENETAPP_NUM_TX_CH_PER_PERCTXT     (ENET_SYSCFG_TX_CHANNELS_NUM/gEnetMp.numPerCtxts)
#define ENETAPP_NUM_RX_FLOW_PER_PERCTXT   (ENET_SYSCFG_RX_FLOWS_NUM/gEnetMp.numPerCtxts)

static int32_t EnetMp_openDma(EnetMp_PerCtxt *perCtxt,uint32_t  perCtxtIndex)
{
    uint32_t i, flowIdx;
    int32_t status = ENET_SOK;
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;

    /* Open the TX channel */
    txInArgs.cbArg    = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH0 + (perCtxtIndex * ENETAPP_NUM_TX_CH_PER_PERCTXT)),
                           &txInArgs,
                           &txChInfo);

    perCtxt->txChNum = txChInfo.txChNum;
    perCtxt->hTxCh   = txChInfo.hTxCh;

    EnetAppUtils_assert(txChInfo.useGlobalEvt == true);
    EnetAppUtils_assert(txChInfo.maxNumTxPkts >= (ENET_SYSCFG_TOTAL_NUM_TX_PKT/2U));

    if (perCtxt->hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(perCtxt->handleInfo.hEnet,
                              perCtxt->attachInfo.coreKey,
                              gEnetMp.coreId,
                              gEnetMp.txChNum);
#endif
        DebugP_log("EnetMp_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(perCtxt->hTxCh != NULL);
    }

    /* Allocate TX packets and keep them locally enqueued */
    if (status == ENET_SOK)
    {
        EnetMp_initTxFreePktQ();
    }

    /* Open the RX flow */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;
        perCtxt->numHwRxCh = ENET_SYSCFG_RX_CHANNELS_NUM;

        for (i = 0U; i < perCtxt->numHwRxCh; i++)
        {
            for (flowIdx = 0U; flowIdx < ENET_SYSCFG_RX_FLOWS_NUM; flowIdx++)
            {
                rxInArgs.notifyCb = EnetMp_rxIsrFxn;
                rxInArgs.cbArg    = perCtxt;
                EnetApp_getRxDmaHandle((ENET_DMA_RX_CH0 + i) + (perCtxtIndex * ENETAPP_NUM_RX_FLOW_PER_PERCTXT),
                                       &rxInArgs,
                                       &rxChInfo);
                EnetAppUtils_assert(rxChInfo.useDefaultFlow == true);
                EnetAppUtils_assert(rxChInfo.chIdx == i);
                EnetAppUtils_assert(rxChInfo.useGlobalEvt == true);
                EnetAppUtils_assert(rxChInfo.sizeThreshEn == 0U);
                EnetAppUtils_assert(rxChInfo.maxNumRxPkts >= (ENET_SYSCFG_TOTAL_NUM_RX_PKT/2U));

                perCtxt->rxStartFlowIdx[i]    = rxChInfo.rxFlowStartIdx;
                perCtxt->rxFlowIdx[i]         = rxChInfo.rxFlowIdx;
                perCtxt->hRxCh[i]             = rxChInfo.hRxCh;
                perCtxt->numValidMacAddress  += rxChInfo.numValidMacAddress;
                if (rxChInfo.numValidMacAddress > 0)
                {
                    EnetUtils_copyMacAddr(perCtxt->macAddr, &rxChInfo.macAddr[0][0]);
                }
                if (perCtxt->hRxCh[i] == NULL)
                {
                    DebugP_log("EnetMp_openRxCh() failed to open RX flow\r\n");
                    status = ENET_EFAIL;
                    EnetAppUtils_assert(perCtxt->hRxCh[i] != NULL);
                }
            }
        }
    }

    /* Submit all ready RX buffers to DMA */
    if (status == ENET_SOK)
    {
        for (i = 0U; i < perCtxt->numHwRxCh; i++)
        {
            EnetMp_initRxReadyPktQ(perCtxt->hRxCh[i]);
        }
    }

     return status;
}

static void EnetMp_closeDma(EnetMp_PerCtxt *perCtxt, uint32_t perCtxtIndex)
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;
    uint32_t i;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close RX channel */
    for (i = 0U; i < perCtxt->numHwRxCh; i++)
    {
        EnetApp_closeRxDma((ENET_DMA_RX_CH0 + i) + (perCtxtIndex * ENETAPP_NUM_RX_FLOW_PER_PERCTXT),
                           perCtxt->handleInfo.hEnet,
                           perCtxt->attachInfo.coreKey,
                           gEnetMp.coreId,
                           &fqPktInfoQ,
                           &cqPktInfoQ);

        EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
        EnetAppUtils_freePktInfoQ(&cqPktInfoQ);
    }

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetMp_retrieveFreeTxPkts(perCtxt);

    EnetApp_closeTxDma((ENET_DMA_TX_CH0 + (perCtxtIndex * ENETAPP_NUM_TX_CH_PER_PERCTXT)),
                       perCtxt->handleInfo.hEnet,
                       perCtxt->attachInfo.coreKey,
                       gEnetMp.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetMp.txFreePktInfoQ);
}

static void EnetMp_initTxFreePktQ(void)
{
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    /* Initialize TX EthPkts and queue them to txFreePktInfoQ */
    for (i = 0U; i < (ENET_SYSCFG_TOTAL_NUM_TX_PKT/2); i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetMp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);
        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&gEnetMp.txFreePktInfoQ, &pPktInfo->node);
    }

    DebugP_log("initQs() txFreePktInfoQ initialized with %d pkts\r\n",
                       EnetQueue_getQCount(&gEnetMp.txFreePktInfoQ));
}

static void EnetMp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] =
    {
       ENET_MEM_LARGE_POOL_PKT_SIZE,
    };

    EnetQueue_initQ(&rxFreeQ);

    for (i = 0U; i < (ENET_SYSCFG_TOTAL_NUM_RX_PKT/2); i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetMp,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

static uint32_t EnetMp_retrieveFreeTxPkts(EnetMp_PerCtxt *perCtxt)
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(perCtxt->hTxCh, &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&gEnetMp.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        DebugP_log("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

static void EnetMp_createRxTask(EnetMp_PerCtxt *perCtxt,
                                uint8_t *taskStack,
                                uint32_t taskStackSize)
{
    TaskP_Params taskParams;
    int32_t status;
    status = SemaphoreP_constructBinary(&perCtxt->rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructCounting(&perCtxt->rxDoneSemObj, 0, COUNTING_SEM_COUNT);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = taskStack;
    taskParams.stackSize      = taskStackSize;
    taskParams.args           = (void*)perCtxt;
    taskParams.name           = "Rx Task";
    taskParams.taskMain           = &EnetMp_rxTask;

    status = TaskP_construct(&perCtxt->rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

static void EnetMp_destroyRxTask(EnetMp_PerCtxt *perCtxt)
{
    SemaphoreP_destruct(&perCtxt->rxSemObj);
    SemaphoreP_destruct(&perCtxt->rxDoneSemObj);
    TaskP_destruct(&perCtxt->rxTaskObj);
}

static void EnetMp_rxTask(void *args)
{
    EnetMp_PerCtxt *perCtxt = (EnetMp_PerCtxt *)args;
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *rxPktInfo;
    EnetDma_Pkt *txPktInfo;
    EthFrame *rxFrame;
    EthFrame *txFrame;
    Enet_IoctlPrms prms;
    bool semStatus;
    uint32_t reqTs;
    // uint32_t totalRxCnt = 0U;
    uint32_t i;
    int32_t status = ENET_SOK;

    status = EnetMp_waitForLinkUp(perCtxt);
    if (status != ENET_SOK)
    {
        DebugP_log("%s: Failed to wait for link up: %d\n", perCtxt->name, status);
    }

    /* Open DMA for peripheral/port */
    if (status == ENET_SOK)
    {
        uint32_t perCtxtIndex;

        DebugP_log("%s: Open DMA\r\n", perCtxt->name);

        status = EnetMp_mapPerCtxt2Idx(perCtxt, &perCtxtIndex);
        EnetAppUtils_assert(status == ENET_SOK);
        status = EnetMp_openDma(perCtxt, perCtxtIndex);
        if (status != ENET_SOK)
        {
            DebugP_log("%s: failed to open DMA: %d\r\n", perCtxt->name, status);
        }
    }

    /* Add port MAC entry */
    if ((status == ENET_SOK) && (Enet_isIcssFamily(perCtxt->enetType)))
    {
        DebugP_log("%s: Set MAC addr: ", perCtxt->name);
        EnetAppUtils_printMacAddr(&perCtxt->macAddr[0U]);

        if (perCtxt->enetType == ENET_ICSSG_DUALMAC)
        {
            IcssgMacPort_SetMacAddressInArgs inArgs;

            memset(&inArgs, 0, sizeof(inArgs));
            inArgs.macPort = perCtxt->macPort[0U];
            EnetUtils_copyMacAddr(&inArgs.macAddr[0U], &perCtxt->macAddr[0U]);
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_MACPORT_IOCTL_SET_MACADDR, &prms, status);
        }
        else
        {
            Icssg_MacAddr addr; // FIXME Icssg_MacAddr type

            /* Set host port's MAC address */
            EnetUtils_copyMacAddr(&addr.macAddr[0U], &perCtxt->macAddr[0U]);
            ENET_IOCTL_SET_IN_ARGS(&prms, &addr);

            ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_HOSTPORT_IOCTL_SET_MACADDR, &prms, status);
        }

        if (status != ENET_SOK)
        {
                DebugP_log("%s: Failed to set MAC address entry: %d\n", perCtxt->name, status);
        }
    }

    DebugP_log("%s: MAC port addr: ", perCtxt->name);
    EnetAppUtils_printMacAddr(&perCtxt->macAddr[0U]);

    while ((ENET_SOK == status) && (gEnetMp.run))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&perCtxt->rxSemObj, SystemP_WAIT_FOREVER);

        /* All peripherals have single hardware RX channel, so we only need to retrieve
         * packets from a single flow.  But ICSSG Switch has two hardware channels, so
         * we need to retrieve packets from two flows, one flow per channel */
        for (i = 0U; i < perCtxt->numHwRxCh; i++)
        {
            EnetQueue_initQ(&rxReadyQ);
            EnetQueue_initQ(&rxFreeQ);
            EnetQueue_initQ(&txSubmitQ);

            /* Get the packets received so far */
            status = EnetDma_retrieveRxPktQ(perCtxt->hRxCh[i], &rxReadyQ);
            if (status != ENET_SOK)
            {
                /* Should we bail out here? */
                DebugP_log("Failed to retrieve RX pkt queue: %d\r\n", status);
                continue;
            }
#if DEBUG
            DebugP_log("%s: Received %u packets\r\n", perCtxt->name, EnetQueue_getQCount(&rxReadyQ));
#endif
            totalRxCnt += EnetQueue_getQCount(&rxReadyQ);
            reqTs = 0U;

            /* Consume the received packets and send them back */
            rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
            while (rxPktInfo != NULL)
            {
                rxFrame = (EthFrame *)rxPktInfo->sgList.list[0].bufPtr;
                EnetDma_checkPktState(&rxPktInfo->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_DRIVER,
                                      ENET_PKTSTATE_APP_WITH_READYQ);

                /* Retrieve TX packets from driver and recycle them */
                EnetMp_retrieveFreeTxPkts(perCtxt);

                /* Dequeue one free TX Eth packet */
                txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetMp.txFreePktInfoQ);
                if (txPktInfo != NULL)
                {
                    /* Fill the TX Eth frame with test content */
                    txFrame = (EthFrame *)txPktInfo->sgList.list[0].bufPtr;
                    memcpy(txFrame->hdr.dstMac, rxFrame->hdr.srcMac, ENET_MAC_ADDR_LEN);
                    memcpy(txFrame->hdr.srcMac, &perCtxt->macAddr[0U], ENET_MAC_ADDR_LEN);
                    txFrame->hdr.etherType = rxFrame->hdr.etherType;

                    memcpy(&txFrame->payload[0U],
                           &rxFrame->payload[0U],
                           rxPktInfo->sgList.list[0].segmentFilledLen - sizeof(EthFrameHeader));

                    txPktInfo->sgList.list[0].segmentFilledLen = rxPktInfo->sgList.list[0].segmentFilledLen;
                    txPktInfo->sgList.numScatterSegments = 1;
                    txPktInfo->chkSumInfo = 0U;
                    txPktInfo->appPriv = &gEnetMp;

                    /* Set timestamp info in DMA packet.
                     * Packet timestamp currently enabled only for ICSSG. */
                    if (gEnetMp.enableTs &&
                        Enet_isIcssFamily(perCtxt->enetType))
                    {
                        /* Save the timestamp of received packet that we are about to send back,
                         * so we can calculate the RX-to-TX time diffence in TX timestamp callback */
                        perCtxt->rxTs[perCtxt->txTsSeqId % ENET_SYSCFG_TOTAL_NUM_RX_PKT] = rxPktInfo->tsInfo.rxPktTs;

                        txPktInfo->tsInfo.enableHostTxTs = true;
                        txPktInfo->tsInfo.txPktSeqId     = perCtxt->txTsSeqId++;
                        txPktInfo->tsInfo.txPktMsgType   = 0U; /* Don't care for ICSSG */
                        txPktInfo->tsInfo.txPktDomain    = 0U; /* Don't care for ICSSG */
                        reqTs++;
                    }
                    else
                    {
                        txPktInfo->tsInfo.enableHostTxTs = false;
                    }

                    EnetDma_checkPktState(&txPktInfo->pktState,
                                          ENET_PKTSTATE_MODULE_APP,
                                          ENET_PKTSTATE_APP_WITH_FREEQ,
                                          ENET_PKTSTATE_APP_WITH_DRIVER);

                    /* Enqueue the packet for later transmission */
                    EnetQueue_enq(&txSubmitQ, &txPktInfo->node);
                }
                else
                {
                    DebugP_log("%s: Drop due to TX pkt not available\r\n", perCtxt->name);
                }

                EnetDma_checkPktState(&rxPktInfo->pktState,
                                      ENET_PKTSTATE_MODULE_APP,
                                      ENET_PKTSTATE_APP_WITH_READYQ,
                                      ENET_PKTSTATE_APP_WITH_FREEQ);

                /* Release the received packet */
                EnetQueue_enq(&rxFreeQ, &rxPktInfo->node);
                rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
            }

            /* Transmit all enqueued packets */
            status = EnetDma_submitTxPktQ(perCtxt->hTxCh, &txSubmitQ);
            if (status != ENET_SOK)
            {
                DebugP_log("%s: Failed to submit TX pkt queue: %d\r\n", perCtxt->name, status);
            }

            EnetAppUtils_validatePacketState(&rxFreeQ,
                                             ENET_PKTSTATE_APP_WITH_FREEQ,
                                             ENET_PKTSTATE_APP_WITH_DRIVER);

            /* Wait for TX timestamp */
            while (gEnetMp.run && (reqTs != 0U))
            {
                Enet_MacPort macPort = ENET_MACPORT_DENORM(i);

                Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_TIMESTAMP_TX, &macPort, sizeof(macPort));
                semStatus = SemaphoreP_pend(&perCtxt->txTsSemObj, SystemP_WAIT_FOREVER);
                if (semStatus == SystemP_SUCCESS)
                {
                    reqTs--;
                }
            }

            /* Submit now processed buffers */
            EnetDma_submitRxPktQ(perCtxt->hRxCh[i], &rxFreeQ);
            if (status != ENET_SOK)
            {
                DebugP_log("%s: Failed to submit RX pkt queue: %d\r\n", perCtxt->name, status);
            }
        }
    }

#if DEBUG
    DebugP_log("%s: Received %u packets\r\n", perCtxt->name, totalRxCnt);
#endif


    SemaphoreP_post(&perCtxt->rxDoneSemObj);
    TaskP_exit();
}

void runTest(void)
{
    uint16_t printloop;
    int32_t ret_val = 0;

    ClockP_usleep(1000);
    do
    {
        DebugP_log("\n\r***************************************************\n\r");
        DebugP_log("\t\tUNIT TEST MENU\n\r");
        DebugP_log("***************************************************\n\r");

        for (printloop = 0; printloop < MAX_NO_OF_TEST_CASES; printloop++)
        {
            DebugP_log("%d.  %s\n\r", printloop + 1, enet_icssg_ut[printloop].test_name);
        }

        DebugP_log("\n\rTest case id: ");
        DebugP_scanf("%d", &uartScanTemp1);

        if (uartScanTemp1 >=1 && uartScanTemp1 <= MAX_NO_OF_TEST_CASES)
        {
            ret_val = enet_icssg_ut[(uartScanTemp1-1)].test_function();
            if (ret_val == TEST_SUCCESS)
            {
                DebugP_log("\nTest %s run successfully from DUT side...\n\r",
                                   enet_icssg_ut[(uartScanTemp1-1)].test_name);
            }

            else
            {
                DebugP_log("\nTest %s not run successfully from DUT side...\n\r",
                                   enet_icssg_ut[(uartScanTemp1-1)].test_name);
            }
        }
        else
        {
            DebugP_log("\n Invalid test case please select valid test case\n\r");
        }

        TaskP_yield();
    }
    while (true);
}

int32_t TC_VLAN_1_1(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id = 1;
    uint8_t pcp;

    DebugP_log("Selected Test case: VLAN Classification \n\r");

    //Set DUT.TS1 PVID to be 3.
    vlan_id = 0x3;
    pcp = 0x3;

    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);

    //Set DUT.TS1 to be part of the untagged member set of VLAN 3.
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 0, 0, 0, 0, 0, 0);

    //Set DUT.TS2 to be part of the tagged member set of VLAN 2.
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 1, 0, 0);

    return retVal;
}

int32_t TC_VLAN_1_2_A(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    DebugP_log("Selected Test case is TC_VLAN_1_2_A \n\r");

    //Make default configuration with PVID =1 for both the ports
    retVal = vlan_default_config();
    if (retVal != ENET_SOK)
    {
        DebugP_log("ERROR: making default configuration \n\r");
    }

    //Set DUT.TS1 to be in the tagged member set of VLAN 2.
    //Set DUT.TS2 to be in the tagged member set of VLAN 2
    /* Update table for default entry */
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);

    //Set DUT.TS1 to be in untagged member set of VLAN 3.
    vlan_id = 0x3;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 0, 0, 0, 0, 0, 0);

    /* Update table for default entry for priority tagged frames*/
    vlan_id = 0;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 0, 0, 0);

    return retVal;
}

int32_t TC_VLAN_1_2_B(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    DebugP_log("Selected Test case is TC_VLAN_1_2_B \n\r");

    //PORT1
    //Send the IOCTL command to set to accept all types frames
    retVal = set_acceptable_frame_type(ENET_MAC_PORT_1, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, ICSSG_ACCEPT_ALL);

    //For PORT2
    retVal = set_acceptable_frame_type(ENET_MAC_PORT_2, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, ICSSG_ACCEPT_ALL);

    //Make default configuration with PVID =1 for both the ports
    retVal = vlan_default_config();
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: making default configuration \n\r");
    }

    //Set DUT.TS1 to be in the tagged member set of VLAN 2.
    //Set DUT.TS2 to be in the tagged member set of VLAN 2
    /* Update table for default entry */
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    //Set DUT.TS1 to be in untagged member set of VLAN 3.
    vlan_id = 0x3;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 0, 0, 0, 0, 0, 0);

    /* Update table for default entry for priority tagged frames*/
    vlan_id = 0;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 0, 0, 0);

    return retVal;
}

int32_t TC_VLAN_1_2_C(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    DebugP_log("Selected Test case is TC_VLAN_1_2_C \n\r");

    //Send the IOCTL command to set to accept only VLAN-tagged frames
    retVal = set_acceptable_frame_type(ENET_MAC_PORT_1, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, ICSSG_ACCEPT_ONLY_VLAN_TAGGED);

    //For PORT2
    retVal = set_acceptable_frame_type(ENET_MAC_PORT_2, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, ICSSG_ACCEPT_ONLY_VLAN_TAGGED);

    //Make default configuration with PVID =1 for both the ports
    retVal = vlan_default_config();
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: making default configuration \n\r");
    }

    //Set DUT.TS1 to be in the tagged member set of VLAN 2.
    //Set DUT.TS2 to be in the tagged member set of VLAN 2
    /* Update table for default entry */
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    //Set DUT.TS1 to be in untagged member set of VLAN 3.
    vlan_id = 0x3;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 0, 0, 0, 0, 0, 0);

    return retVal;
}

int32_t TC_VLAN_1_2_D(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    DebugP_log("Selected Test case is TC_VLAN_1_2_D \n\r");

    //Send the IOCTL command to set to accept only Untgged and priority tagged frames
    retVal = set_acceptable_frame_type(ENET_MAC_PORT_1, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, ICSSG_ACCEPT_ONLY_UNTAGGED_PRIO_TAGGED);

    //For PORT2
    retVal = set_acceptable_frame_type(ENET_MAC_PORT_2, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, ICSSG_ACCEPT_ONLY_UNTAGGED_PRIO_TAGGED);

    //Make default configuration with PVID =1 for both the ports
    retVal = vlan_default_config();
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: making default configuration \n\r");
    }

    //Set DUT.TS1 to be in the tagged member set of VLAN 2.
    //Set DUT.TS2 to be in the tagged member set of VLAN 2
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);

    //Set DUT.TS1 to be in untagged member set of VLAN 3.
    vlan_id = 0x3;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 0, 0, 0, 0, 0, 0);

    /* Update table for default entry for priority tagged frames*/
    vlan_id = 0;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 0, 0, 0);

    return retVal;
}

int32_t TC_VLAN_1_3_C(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    DebugP_log("Selected Test case is TC_VLAN_1_3_C \n\r");

    retVal = vlan_default_config();
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: making default configuration \n\r");
    }

    //Set DUT.TS1 to be in the untagged member set of VLAN 2.
    /* Update table for default entry */
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 0, 0, 0, 0, 0, 0);

    //Set DUT.TS2 to be in tagged member set of VLAN 3.
    vlan_id = 0x3;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 0, 0, 0, 0, 1, 0, 0);

    return retVal;
}

int32_t TC_VLAN_1_4_A(void)
{
    int32_t retVal = TEST_FAILURE;
    uint8_t pcp;
    uint16_t vlan_id = 0x0;

    DebugP_log("Selected Test case is TC_VLAN_1_4_A \n\r");

    //Attempt to add DUT.TS1 as an untagged member of VID 0x0000.
    pcp = 0x1;

    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0 for port1: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0 for port1: FAILED\n\r");
    }

    //Set DUT.TS2 PVID to be 0.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_2, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0 for port2: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0 for port2: FAILED\n\r");
    }

    //Repeat steps 1 and 2 using a VID of 0x0FFF and 0xFFFE
    vlan_id = 0xFFFE;

    //Attempt to add DUT.TS1 as an untagged member of VID 0xFFFE.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0xFFFE for port1: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0xFFFE for port1: FAILED\n\r");
    }

    //Set DUT.TS2 PVID to be 0xFFFE.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_2, pcp, vlan_id);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0xFFFE for port2: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0xFFFE for port2: FAILED\n\r");
    }

    //Repeat steps 1 and 2 using a VID of 0x0FFF and 0xFFFE
    vlan_id = 0x0FFF;

    //Attempt to add DUT.TS1 as an untagged member of VID 0x0FFF.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0xFFFF for port1: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0xFFFF for port1: FAILED\n\r");
    }

    //Set DUT.TS2 PVID to be 0x0FFF.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_2, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0xFFFF for port2: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("\nTest for Setting default VLAN ID = 0xFFFF for port2: FAILED\n\r");
    }

    return retVal;
}

int32_t TC_VLAN_1_4_B(void)
{
    uint16_t vlan_id = 0x0;
    int32_t retVal = TEST_FAILURE;

    DebugP_log("Selected Test case is TC_VLAN_1_4_B \n\r");

    //Attempt to add DUT.TS1  and DUT.TS2 as a tagged member of VID 0x0000.
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0 for port1 and port2: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("Test for Setting VLAN ID = 0 for port1 and port2: FAILED\n\r");
    }

    //Attempt to add DUT.TS1 as a tagged member of VID 0x0FFF.
    vlan_id = 0x0FFF;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0xFFFF for port1 and port2: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("Test for Setting VLAN ID = 0xFFFF for port1 and port2: FAILED\n\r");
    }

    //Attempt to add DUT.TS1 as a tagged member of VID 0xFFFE
    vlan_id = 0xFFFE;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0xFFFE for port1 and port2: PASSED\n\r");
        retVal = TEST_SUCCESS;
    }
    else
    {
        DebugP_log("Test for Setting VLAN ID = 0xFFFE for port1 and port2: FAILED\n\r");
    }

    return retVal;
}

int32_t TC_VLAN_1_4_C(void)
{
    int32_t retVal = TEST_FAILURE;
    uint8_t pcp;
    uint16_t vlan_id = 0x02;

    DebugP_log("Selected Test case is TC_VLAN_1_4_C \n\r");

    // Attempt to set the PVID of DUT.TS1 to 0x0002.
    pcp = 0x1;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 0, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0x0002 for port1 and port2: FAILED\n\r");
    }

    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting default VLAN ID = 0x0002 for port1: FAILED\n\r");
    }

    //Set DUT.TS2 PVID to be 0x0002.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_2, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting default VLAN ID = 0x0002 for port2: FAILED\n\r");
    }

    //Repeat steps 1 and 2 using a VID of 0x0080 and 0x0800.
    vlan_id = 0x0080;

    //Attempt to add DUT.TS1 and DUT.TS2 as a tagged member of VID 0x0080.
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 0, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0x0080 for port1 and port2: FAILED\n\r");
    }

    //Attempt to set the PVID of DUT.TS1 to 0x0080.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting default VLAN ID = 0x0080 for port1: FAILED\n\r");
    }

    //Set DUT.TS2 PVID to be 0x0080.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_2, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting default VLAN ID = 0x0080 for port2: FAILED\n\r");
    }

    //Repeat steps 1 and 2 using a VID of 0x0080 and 0x0800
    vlan_id = 0x0800;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 0, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0x0800 for port1 and port2: FAILED\n\r");
    }

    //Attempt to set the PVID of DUT.TS1 to 0x0800.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting default VLAN ID = 0x0800 for port1: FAILED\n\r");
    }

    //Set DUT.TS2 PVID to be 0x0800.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_2, pcp, vlan_id);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting default VLAN ID = 0x0800 for port2: FAILED\n\r");
    }

    return retVal;
}

int32_t TC_VLAN_1_4_D(void)
{
    uint16_t vlan_id = 0x02;
    int32_t retVal = TEST_FAILURE;

    DebugP_log("Selected Test case is TC_VLAN_1_4_D \n\r");
    //Attempt to add DUT.TS1  and DUT.TS2 as a tagged member of VID 0x0002.
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0x0002 for port1 and port2: FAILED\n\r");
    }

    //Attempt to add DUT.TS1 and DUT.TS2 as a tagged member of VID 0x0080.
    vlan_id = 0x0080;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0x0080 for port1 and port2: FAILED\n\r");
    }

    //Attempt to add DUT.TS1 and DUT.TS2 as a tagged member of VID 0x0800
    vlan_id = 0x0800;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("Test for Setting VLAN ID = 0x0800 for port1 and port2: FAILED\n\r");
    }

    return retVal;

}

int32_t TC_VLAN_1_5_B(void)
{
    //int32_t retVal = TEST_FAILURE;

    DebugP_log("Selected Test case is TC_VLAN_1_5_B \n\r");

    return TEST_SUCCESS;
}

int32_t TC_VLAN_2_1_A(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    DebugP_log("Selected Test case is TC_VLAN_2_1_A \n\r");

    retVal = vlan_default_config();
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: making default configuration \n\r");
    }

    //Set DUT.TS1 to be in the tagged member set of VLAN 2.
    //Set DUT.TS2 to be in the tagged member set of VLAN 2
    /* Update table for default entry */
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    return retVal;
}

int32_t TC_VLAN_2_1_B(void)
{
    DebugP_log("Selected Test case is TC_VLAN_2_1_B \n\r");
    return TC_VLAN_2_1_A();
}

int32_t TC_VLAN_2_1_C(void)
{
    DebugP_log("Selected Test case is TC_VLAN_2_1_C \n\r");
    return TC_VLAN_2_1_A();
}

int32_t TC_VLAN_2_1_D(void)
{
    return TC_VLAN_2_1_A();
}

int32_t TC_VLAN_2_2_A(void)
{
    int32_t retVal = TEST_FAILURE;

    retVal = TC_VLAN_2_1_A();

    return retVal;
}

int32_t TC_VLAN_2_2_B(void)
{
    int32_t retVal = TEST_FAILURE;

    retVal = TC_VLAN_2_1_A();

    return retVal;
}

int32_t TC_VLAN_2_3(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    DebugP_log("Selected Test case is TC_VLAN_2_3 \n\r");

    retVal = vlan_default_config();
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: making default configuration \n\r");
    }

    //Set DUT.TS1 to be in the tagged member set of VLAN 2.
    //Set DUT.TS2 to be in the untagged member set of VLAN 2
    /* Update table for default entry */
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 0, 0, 0);

    return retVal;
}

int32_t TC_VLAN_2_4(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;
    uint8_t pcp;
    uint32_t prioRegenMap[ENET_PRI_NUM];
    uint32_t  i = 0;
    uint32_t newPrio = 1;

    DebugP_log("Selected Test case: VLAN Classification \n\r");

    //Set DUT.TS1 PVID to be 3.
    vlan_id = 0x3;
    pcp = 0x3;

    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 0, 0, 0, 0, 0, 0);

    //Set DUT.TS1 to be part of the tagged member set of VLAN 2.
    //Set DUT.TS2 to be part of the tagged member set of VLAN 2.
    vlan_id = 0x2;
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 1, 1, 0, 0);

    for (i = 0; i < ENET_PRI_NUM; i++)
    {
      prioRegenMap[i] = (uint32_t)newPrio;
    }

    //Set IOCTL command to make PORT1 as boundary port and PORT2 as accept all type of packets
    retVal = set_priority_reg_mapping(ENET_MAC_PORT_1, prioRegenMap);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: priority regeneration using IOCTL for port1 \n\r");
        return TEST_FAILURE;
    }

    //PORT2
    retVal = set_priority_reg_mapping(ENET_MAC_PORT_2, prioRegenMap);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: priority regeneration using IOCTL for port2 \n\r");
        return TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_VLAN_2_5(void)
{
    int32_t retVal = TEST_FAILURE;

    retVal = TC_VLAN_2_1_A();

    return retVal;
}

int32_t TC_VLAN_3_4(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id = 0x03;

    retVal = add_fid_vid_entry( vlan_id, NRT_FID, 1, 1, 1, 1, 0, 1, 0, 0);

    return retVal;
}

int32_t TC_undef(void)
{
    return TEST_SUCCESS;
}

int32_t TC_VLAN_testing(void)
{
    //uint32_t uartScanTemp = 0;
    int32_t ret_value = 0;
    int32_t printloop;

    DebugP_log("In VLAN part of ENET@UT \n\r");
    DebugP_log("Please select Test no to choose particular test case to execute \n\r");

    for (printloop = 0; printloop < MAX_NO_OF_VLAN_TEST_CASES &&
    VLANTestFunc[printloop].testfunc != TC_undef; printloop++)
    {
        DebugP_log("%d.  %s\n\r", printloop + 1, VLANTestFunc[printloop].TestStr);
    }

    DebugP_scanf("%d", &uartScanTemp);

    if (uartScanTemp > 0 & uartScanTemp <= (MAX_NO_OF_VLAN_TEST_CASES + 1))
    {
        ret_value = VLANTestFunc[uartScanTemp - 1].testfunc();

        if (ret_value == TEST_SUCCESS)
        {
            DebugP_log("Test %s run successfully from DUT side...\n\r",
                               VLANTestFunc[uartScanTemp - 1].TestStr);
        }
        else
        {
            DebugP_log("Test %s FAILED from DUT side...\n\r",
                               VLANTestFunc[uartScanTemp - 1].TestStr);
        }
    }
    else
    {
        DebugP_log("\nInput is invalid!\n\r");
    }

    return ret_value;
}

int32_t TC_FDB_testing(void)
{

    //uint32_t uartScanTemp = 0;
    int32_t ret_value = 0;
    int32_t printloop;

    DebugP_log("In FDB part of ENET@UT \n\r");
    DebugP_log("Please select Test no to choose particular test case to execute \n\r");

    fdb_default_config();

    for (printloop = 0; printloop < MAX_NO_OF_FDB_TEST_CASES && FDBTestFunc[printloop].testfunc != TC_undef; printloop++)
    {
        DebugP_log("%d.  %s\n\r", printloop + 1, FDBTestFunc[printloop].TestStr);
    }

    DebugP_scanf("%d", &uartScanTemp);

    if (uartScanTemp > 0 & uartScanTemp <= (MAX_NO_OF_FDB_TEST_CASES + 1))
    {
        ret_value = FDBTestFunc[uartScanTemp - 1].testfunc();

        if (ret_value == TEST_SUCCESS)
        {
            DebugP_log("Test %s run successfully from DUT side...\n\r",
                               FDBTestFunc[uartScanTemp-1].TestStr);
        }
        else
        {
            DebugP_log("Test %s not run successfully from DUT side...\n\r",
                               FDBTestFunc[uartScanTemp-1].TestStr);
        }
    }
    else
    {
        DebugP_log("\nInput is invalid!\n\r");
    }

    return ret_value;
}

int32_t vlan_default_config(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;
    uint8_t pcp = 0x1;

    //Set DUT.TS1 PVID to be 1.
    vlan_id = 0x1;
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);

    //Set DUT.TS2 PVID to be 1.
    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_2, pcp, vlan_id);

    /* Update table entry for default vid */
    //add_fid_vid_entry(uint16_t vlan_id, uint8_t fid, uint8_t hostMember, uint8_t memberP1, uint8_t memberP2, uint8_t host_tagged, uint8_t taggedP1, uint8_t taggedP2, uint8_t streamVid, uint8_t floodToHost)
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 0, 0, 0);

    return retVal;

}

int32_t add_default_port_vid(uint8_t port_num, uint8_t pcp, uint16_t vlan_id)
{
    int32_t retVal = TEST_SUCCESS;
    Enet_IoctlPrms prms;
    Icssg_MacPortDfltVlanCfgInArgs vlanDefaultEntry;
#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[port_num];
    vlanDefaultEntry.macPort = perCtxt->macPort[ENET_MAC_PORT_1];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    vlanDefaultEntry.macPort = perCtxt->macPort[port_num];
#endif

    vlanDefaultEntry.vlanCfg.portVID = vlan_id;
    vlanDefaultEntry.vlanCfg.portPri = pcp;

    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanDefaultEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_PER_IOCTL_VLAN_SET_MACPORT_DFLT_VID, &prms, retVal);
    if (retVal != ENET_SOK)
    {
        DebugP_log("Failed to set default VID");
    }

    return retVal;
}
int32_t add_default_host_vid(uint8_t pcp, uint16_t vlan_id)
{
    int32_t retVal = TEST_SUCCESS;
    Enet_IoctlPrms prms;
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    EnetPort_VlanCfg vlanDefaultEntry;

    vlanDefaultEntry.portVID = vlan_id;
    vlanDefaultEntry.portPri = pcp;

    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanDefaultEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_PER_IOCTL_VLAN_SET_HOSTPORT_DFLT_VID, &prms, retVal);
    if (retVal != ENET_SOK)
    {
        DebugP_log("Failed to set default VID");
    }

    return retVal;
}

int32_t add_fid_vid_entry(uint16_t vlan_id,
                          uint8_t fid,
                          uint8_t hostMember,
                          uint8_t memberP1,
                          uint8_t memberP2,
                          uint8_t host_tagged,
                          uint8_t taggedP1,
                          uint8_t taggedP2,
                          uint8_t streamVid,
                          uint8_t floodToHost)
{
    //EMAC_IOCTL_PARAMS params;
    Enet_IoctlPrms prms;
    int32_t retVal = TEST_FAILURE;
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    Icssg_VlanFidEntry vlanEntry;
    Icssg_VlanFidParams vlanParams = {
        .fid         = fid,
        .hostMember  = (uint8_t)hostMember,
        .p1Member    = (uint8_t)memberP1,
        .p2Member    = (uint8_t)memberP2,
        .hostTagged  = (uint8_t)host_tagged,
        .p1Tagged    = (uint8_t)taggedP1,
        .p2Tagged    = (uint8_t)taggedP2,
        .streamVid   = (uint8_t)streamVid,
        .floodToHost = (uint8_t)floodToHost
    };

    vlanEntry.vlanFidParams = vlanParams;

    vlanEntry.vlanId= vlan_id;
    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_PER_IOCTL_VLAN_SET_ENTRY, &prms, retVal);
    if (retVal != ENET_SOK)
    {
        DebugP_log("FID entry for VLAN = %u : FAILED\n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    return retVal;
}

int32_t set_acceptable_frame_type(uint8_t port_num, uint32_t acceptableFrameType, Icssg_AcceptFrameCheck configparam)
{
    int32_t retVal = TEST_SUCCESS;
    bool semStatus;
    Enet_IoctlPrms prms;
    Icssg_SetAcceptFrameCheckInArgs params;
#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[port_num];
    params.macPort = perCtxt->macPort[ENET_MAC_PORT_1];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    params.macPort = perCtxt->macPort[port_num];
#endif

    params.acceptFrameCheck = configparam;
    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    //asynchronous IOCTL
    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, &prms, retVal);
    if (retVal == ENET_SINPROGRESS)
    {
        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, SystemP_WAIT_FOREVER);
        } while (gEnetMp.run && (semStatus != SystemP_SUCCESS));

        retVal = ENET_SOK;
    }
    else
    {
        DebugP_log("ERROR: %s: %d: IOCTL command sent for port = %u \n\r", __FUNCTION__, __LINE__, port_num);
        retVal = TEST_FAILURE;
    }
    return retVal;

}
int32_t set_priority_reg_mapping(uint8_t port_num, uint32_t *prioRegenMap)
{
    int32_t retVal = TEST_SUCCESS;
    int32_t i;
    EnetMacPort_SetPriorityRegenMapInArgs params;
    Enet_IoctlPrms prms;
#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[port_num];
    params.macPort = perCtxt->macPort[ENET_MAC_PORT_1];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    params.macPort = perCtxt->macPort[port_num];
#endif

    for (i = 0; i < ENET_PRI_NUM; i++)
    {
        params.priorityRegenMap.priorityMap[i] = prioRegenMap[i];
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP, &prms, retVal);
    if (retVal != ENET_SOK)
    {
        DebugP_log("ERROR: IOCTL command for priority regeneration for PORT = %u\n\r", port_num);
        retVal = TEST_FAILURE;
    }

    return retVal;
}

int32_t set_priority_mapping(uint8_t port_num, uint32_t *prioMap)
{
    int32_t retVal = TEST_SUCCESS;
    int32_t i;
    EnetMacPort_SetEgressPriorityMapInArgs params;
    Enet_IoctlPrms prms;
#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[port_num];
    params.macPort = perCtxt->macPort[ENET_MAC_PORT_1];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    params.macPort = perCtxt->macPort[port_num];
#endif

    for (i = 0; i < ENET_PRI_NUM; i++)
    {
        params.priorityMap.priorityMap[i] = prioMap[i];
    }
    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, retVal);
    if (retVal != ENET_SOK)
    {
        DebugP_log("ERROR: IOCTL command for priority mapping for PORT = %u\n\r", port_num);
        retVal = TEST_FAILURE;
    }

    return retVal;
}

int32_t set_port_state(uint8_t port_num, Icssg_PortState port_state)
{
    int32_t retVal = TEST_FAILURE;
    bool semStatus;
    IcssgMacPort_SetPortStateInArgs params;
    Enet_IoctlPrms prms;
#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[port_num];
    params.macPort = perCtxt->macPort[ENET_MAC_PORT_1];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    params.macPort = perCtxt->macPort[port_num];
#endif

    params.portState = port_state;
    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_PER_IOCTL_SET_PORT_STATE, &prms, retVal);
    if (retVal == ENET_SINPROGRESS)
    {
        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, SystemP_WAIT_FOREVER);
        } while (gEnetMp.run && (semStatus != SystemP_SUCCESS));

        retVal = ENET_SOK;
    }
    else
    {
        DebugP_log("ERROR: %s: %d: IOCTL command sent for port = %u \n\r", __FUNCTION__, __LINE__, port_num);
        retVal = TEST_FAILURE;
    }
    return retVal;
}

int32_t fdb_default_config(void)
{
    int32_t retVal = TEST_FAILURE;
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    Icssg_FdbEntry params;
    Enet_IoctlPrms prms;
    uint8_t pcp;
    uint16_t vlan_id;
    bool semStatus;

    //Clear all Dynamic Entries in the Filtering Database
    ENET_IOCTL_SET_IN_ARGS(&prms, &params);
    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_FDB_IOCTL_REMOVE_AGEABLE_ENTRIES, &prms, retVal);
    if (retVal == ENET_SINPROGRESS)
    {
        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, SystemP_WAIT_FOREVER);
        } while (gEnetMp.run && (semStatus != SystemP_SUCCESS));

        retVal = ENET_SOK;
    }
    else
    {
        DebugP_log("fdbUnitTest: Ageable entries are not cleared \n\r");
        retVal = TEST_FAILURE;
    }

    //Set DUT.TS1 PVID to be 1.
    vlan_id = DEFAULT_FDB_VID;
    pcp = DEFAULT_FDB_PCP;

#if !defined(DUAL_MAC_MODE)
    //ToDo: Identify what is port0 in dual-emac case of Enet LLD
    /* Set default VLAN ID for the SW Host port */
    retVal = add_default_host_vid(pcp, vlan_id);
#endif

    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_1, pcp, vlan_id);

    //Set DUT.TS2 PVID to be 1.
    vlan_id = DEFAULT_FDB_VID;
    pcp = DEFAULT_FDB_PCP;

    /* Set default VLAN ID for the SW port */
    retVal = add_default_port_vid(ENET_MAC_PORT_2, pcp, vlan_id);

    /* Update table for default entry */
    vlan_id = DEFAULT_FDB_VID;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 0, 0, 0, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_1_1_A(void)
{
    //default settings will be used for this test case
    return TEST_SUCCESS;
}

int32_t TC_FDB_1_1_B(void)
{
    //default settings will be used for this test case
    return TEST_SUCCESS;
}

int32_t TC_FDB_1_4(void)
{
    return TEST_SUCCESS;
}

int32_t TC_FDB_2_1_A(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    /* Update table for fid-vid entry */
    vlan_id = 0x2;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    retVal = set_port_state(ENET_MAC_PORT_2, ICSSG_PORT_STATE_DISABLED);

    return retVal;
}

int32_t TC_FDB_2_1_B(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    /* Update table for fid-vid entry */
    vlan_id = 0x2;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    retVal = set_port_state(ENET_MAC_PORT_2, ICSSG_PORT_STATE_BLOCKING);

    return TEST_SUCCESS;
}

int32_t TC_FDB_2_1_C(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    /* Update table for fid-vid entry */
    vlan_id = 0x2;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    retVal = set_port_state(ENET_MAC_PORT_2, ICSSG_PORT_STATE_FORWARD);

    return TEST_SUCCESS;
}

int32_t TC_FDB_2_1_D(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id;

    /* Update table for fid-vid entry */
    vlan_id = 0x2;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    retVal = set_port_state(ENET_MAC_PORT_2, ICSSG_PORT_STATE_FORWARD_WO_LEARNING);

    return TEST_SUCCESS;
}

int32_t TC_FDB_2_2_A(void)
{
    return TEST_SUCCESS;
}

int32_t TC_FDB_2_4_A(void)
{
    int32_t retVal = TEST_FAILURE;
    uint16_t vlan_id = 64;

    //add fid-vid entry for vid = 64
    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_2_6_A(void)
{
    int32_t retVal = TEST_FAILURE;

    //For PORT2
    retVal = set_acceptable_frame_type(ENET_MAC_PORT_2, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, ICSSG_ACCEPT_ONLY_VLAN_TAGGED);

    return retVal;
}

int32_t TC_FDB_2_6_B(void)
{
    int32_t retVal = TEST_FAILURE;

    //For PORT2
    retVal = set_acceptable_frame_type(ENET_MAC_PORT_2, ICSSG_MACPORT_IOCTL_SET_ACCEPT_FRAME_CHECK, ICSSG_ACCEPT_ONLY_UNTAGGED_PRIO_TAGGED);

    return retVal;
}

int32_t TC_FDB_2_6_C(void)
{
    int32_t retVal = TEST_FAILURE;
    //For PORT2
    uint32_t vlan_id = 64;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 0, 0, 0);

    return retVal;
}

int32_t TC_FDB_2_6_D(void)
{
    int32_t retVal = TEST_FAILURE;
    //For PORT2
    uint32_t vlan_id = 64;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);

    return retVal;
}

int32_t TC_FDB_2_8(void)
{
    int32_t retVal = TEST_FAILURE;
    Icssg_MacAddr Mac;
    uint8_t fdbEntry_port = 0x00;
    uint16_t vlan_id;

    Mac.macAddr[0] = 0x00;
    Mac.macAddr[1] = 0x02;
    Mac.macAddr[2] = 0x88;
    Mac.macAddr[3] = 0xAA;
    Mac.macAddr[4] = 0x02;
    Mac.macAddr[5] = 0x22;

    /* Update table for fid-vid entry */
    vlan_id = 100;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    //Create a Static Filtering Entry for 00-02-88-AA-02-22 that has a control element specifying forwarding for
    //DUT.TS2.
    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_PORT2_BIT));

    retVal = add_mac_fdb_entry(Mac, vlan_id, fdbEntry_port);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: Static MAC entry for 00-02-88-AA-02-22 failed\n\r");
        return TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_3_1(void)
{
    int32_t retVal = TEST_FAILURE;
    Icssg_MacAddr Mac;
    uint16_t vlan_id;
    uint8_t fdbEntry_port = 0x00;

    /* Update table for fid-vid entry */
    vlan_id = 100;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    Mac.macAddr[0] = 0x01;
    Mac.macAddr[1] = 0x03;
    Mac.macAddr[2] = 0x01;
    Mac.macAddr[3] = 0xAA;
    Mac.macAddr[4] = 0x02;
    Mac.macAddr[5] = 0x11;

    //Create a Static Filtering Entry for 01-03-01-AA-02-11 that has a control element specifying forwarding for
    //DUT.TS1.
    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_PORT1_BIT));

    retVal = add_mac_fdb_entry(Mac, vlan_id, fdbEntry_port);
    //retVal = add_mac_fdb_entry(Mac, vlan_id, FDB_ENTRY_PORT1_ONLY,  FDB_ENTRY_PORT1_ONLY);

    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: Static MAC entry for 01-03-01-AA-02-11 failed\n\r");
        return TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_3_2_A(void)
{
    int32_t retVal = TEST_FAILURE;
    Icssg_MacAddr Mac;
    uint16_t vlan_id;
    uint8_t fdbEntry_port = 0x00;

    /* Update table for fid-vid entry */
    vlan_id = 100;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    Mac.macAddr[0] = 0x00;
    Mac.macAddr[1] = 0x03;
    Mac.macAddr[2] = 0x02;
    Mac.macAddr[3] = 0xAA;
    Mac.macAddr[4] = 0x02;
    Mac.macAddr[5] = 0x22;

    //Create a Static Filtering Entry for 00-03-02-AA-02-22 that has a control element specifying forwarding for
    //DUT.TS2.
    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_PORT2_BIT));

    retVal = add_mac_fdb_entry(Mac, vlan_id, fdbEntry_port);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: Static MAC entry for 00-03-02-AA-02-22 failed\n\r");
        return TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_3_2_B(void)
{
    int32_t retVal = TEST_FAILURE;
    Icssg_MacAddr Mac;
    uint16_t vlan_id;
    uint8_t fdbEntry_port = 0x00;

    /* Update table for fid-vid entry */
    vlan_id = 100;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    Mac.macAddr[0] = 0x00;
    Mac.macAddr[1] = 0x03;
    Mac.macAddr[2] = 0x02;
    Mac.macAddr[3] = 0xBB;
    Mac.macAddr[4] = 0x02;
    Mac.macAddr[5] = 0x22;

    //Create a Static Filtering Entry for 00-03-02-BB-02-22 with a control element specifying filtering for DUT.TS2 and DUT.TS3.
    //Means forward to Port1 only
    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_PORT1_BIT));

    retVal = add_mac_fdb_entry(Mac, vlan_id, fdbEntry_port);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: Static MAC entry for 00-03-02-BB-02-22 failed\n\r");
        return TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_3_2_C(void)
{
    int32_t retVal = TEST_FAILURE;
    Icssg_MacAddr Mac;
    uint16_t vlan_id;
    uint8_t fdbEntry_port = 0x00;

    /* Update table for fid-vid entry */
    vlan_id = 100;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    Mac.macAddr[0] = 0x01;
    Mac.macAddr[1] = 0x03;
    Mac.macAddr[2] = 0x02;
    Mac.macAddr[3] = 0xCC;
    Mac.macAddr[4] = 0x02;
    Mac.macAddr[5] = 0x22;

    //Create a Static Filtering Entry for 01-03-02-CC-02-22 with a control element specifying forwarding for
    // DUT.TS2 and filtering for DUT.TS3.
    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_PORT2_BIT));

    retVal = add_mac_fdb_entry(Mac, vlan_id, fdbEntry_port);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: Static MAC entry for 01-03-02-CC-02-22 failed\n\r");
        return TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_3_2_D(void)
{
    int32_t retVal = TEST_FAILURE;
    Icssg_MacAddr Mac;
    uint16_t vlan_id;
    uint8_t fdbEntry_port = 0x00;

    /* Update table for fid-vid entry */
    vlan_id = 100;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    Mac.macAddr[0] = 0x01;
    Mac.macAddr[1] = 0x03;
    Mac.macAddr[2] = 0x02;
    Mac.macAddr[3] = 0xDD;
    Mac.macAddr[4] = 0x02;
    Mac.macAddr[5] = 0x22;

    //Create a Static Filtering Entry for 01-03-02-DD-02-22 with a control element specifying filtering for DUT.TS2
    //and DUT.TS3.
    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_PORT1_BIT));

    retVal = add_mac_fdb_entry(Mac, vlan_id, fdbEntry_port);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: Static MAC entry for 01-03-02-DD-02-22 failed\n\r");
        return TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_4_2_B(void)
{
    int32_t retVal = TEST_FAILURE;
    Icssg_MacAddr Mac;
    uint16_t vlan_id;
    uint8_t fdbEntry_port = 0x00;

    /* Update table for fid-vid entry */
    vlan_id = 100;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("fdbUnitTest: fid-vid entry = %u is not done \n\r", vlan_id);
        retVal = TEST_FAILURE;
    }

    Mac.macAddr[0] = 0x00;
    Mac.macAddr[1] = 0x04;
    Mac.macAddr[2] = 0x02;
    Mac.macAddr[3] = 0xBB;
    Mac.macAddr[4] = 0x02;
    Mac.macAddr[5] = 0x22;

    //Create a Static Filtering Entry for 00-04-02-BB-02-22 that has a control element specifying forwarding for
    // DUT.TS2 and filtering for DUT.TS3.
    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_PORT2_BIT));

    retVal = add_mac_fdb_entry(Mac, vlan_id, fdbEntry_port);
    if (retVal != TEST_SUCCESS)
    {
        DebugP_log("ERROR: Static MAC entry for 01-03-02-DD-02-22 failed\n\r");
        return TEST_FAILURE;
    }

    return retVal;
}

int32_t TC_FDB_4_3_A(void)
{
    int32_t retVal = TEST_FAILURE;
    //For PORT2
    uint32_t vlan_id = 2;

    retVal = add_fid_vid_entry(vlan_id, NRT_FID, 1, 1, 1, 1, 1, 1, 0, 0);

    return retVal;

}

int32_t Setting_fdb_ageing_timeout(void)
{
    int32_t retVal = TEST_FAILURE;
    uint32_t uartScanTemp_local = 0;
    uint64_t temp;

    DebugP_log("\n Enter the FDB ageing timeout (in sec)\n\r");
    DebugP_scanf("%u", &uartScanTemp_local);
    temp = (uint64_t)uartScanTemp_local*1000*1000*1000;

    retVal = set_ageing_timout(temp);

    return retVal;

}

int32_t del_mac_fdb_entry(Icssg_MacAddr mac, int16_t vlanId)
{
    int32_t retVal = TEST_FAILURE;
    bool semStatus;
    Enet_IoctlPrms prms;
    Icssg_FdbEntry params;
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    int i = 0;

    //Now make an entry in FDB for the HOST MAC address using Asynchronous IOCTL
    for (i = 0; i < ENET_MAC_ADDR_LEN; i++)
    {
        params.macAddr[i] = mac.macAddr[i];
    }

    params.vlanId = vlanId;

    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_FDB_IOCTL_REMOVE_ENTRY, &prms, retVal);
    if (retVal == ENET_SINPROGRESS)
    {
        DebugP_log("Success: IOCTL command sent for making MAC delete in FDB \n\r");

        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, SystemP_WAIT_FOREVER);
        } while (gEnetMp.run && (semStatus != SystemP_SUCCESS));

        retVal = ENET_SOK;
    }
    else
    {
        DebugP_log("ERROR: IOCTL command sent for making MAC delete in FDB failed as vlanId out of range \n\r");
        DebugP_log("ERROR: IOCTL command sent for making MAC delete in FDB \n\r");
    }
    return retVal;
}

int32_t add_mac_fdb_entry(Icssg_MacAddr mac, int16_t vlanId, uint8_t fdbEntry_port)
{
    int32_t retVal = TEST_FAILURE;
    bool semStatus;
    Enet_IoctlPrms prms;
    Icssg_FdbEntry params;
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    int i = 0;

    //Now make an entry in FDB for the HOST MAC address using Asynchronous IOCTL
    for (i = 0; i < ENET_MAC_ADDR_LEN; i++)
    {
        params.macAddr[i] = mac.macAddr[i];
    }

    params.vlanId = vlanId;
    params.fdbEntry[0] = fdbEntry_port;
    params.fdbEntry[1] = fdbEntry_port;

    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_FDB_IOCTL_ADD_ENTRY, &prms, retVal);
    if (retVal == ENET_SINPROGRESS)
    {
        DebugP_log("Success: IOCTL command sent for making MAC entry in FDB \n\r");

        /* Wait for asyc ioctl to complete */
        do
        {
            Enet_poll(perCtxt->handleInfo.hEnet, ENET_EVT_ASYNC_CMD_RESP, NULL, 0U);
            semStatus = SemaphoreP_pend(&perCtxt->ayncIoctlSemObj, SystemP_WAIT_FOREVER);
        } while (gEnetMp.run && (semStatus != SystemP_SUCCESS));

        retVal = ENET_SOK;
    }
    else
    {
        DebugP_log("ERROR: IOCTL command sent for making MAC entry in FDB failed as vlanId out of range \n\r");
        DebugP_log("ERROR: IOCTL command sent for making MAC entry in FDB \n\r");
    }

    return retVal;
}

int32_t set_ageing_timout(uint64_t timeout)
{
    int32_t ret_val = TEST_SUCCESS;
    Enet_IoctlPrms prms;
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];

    //EMAC_IOCTL_FDB_AGEING_TIMEOUT_CTRL is a synchronous IOCTL
    prms.inArgs = (void *)&timeout;

    //Set IOCTL command to configure ageing timeout
    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_FDB_IOCTL_SET_AGING_PERIOD, &prms, ret_val);
    if (ret_val != ENET_SOK)
    {
        DebugP_log("ERROR: IOCTL command to configure ageing timeout = %u\n\r", timeout);
        return TEST_FAILURE;
    }

    return ret_val;
}

int32_t UTILS_testing(void)
{
    //uint32_t uartScanTemp = 0;
    int32_t ret_value = 0;
    int32_t printloop;
    int32_t ret_val = TEST_SUCCESS;

    DebugP_log("In UTILS related testing... \n\r");
    DebugP_log("Please select Test no to choose particular test case to execute \n\r");

    for (printloop = 0; printloop < MAX_NO_OF_UTILS_CASES &&
            UtilsTestFunc[printloop].test_function != TC_undef; printloop++)
    {
        DebugP_log("%d.  %s\n\r", printloop + 1, UtilsTestFunc[printloop].test_name);
    }

    DebugP_scanf("%d", &uartScanTemp);
    if (uartScanTemp > 0 & uartScanTemp <= (MAX_NO_OF_UTILS_CASES + 1))
    {
        ret_val = UtilsTestFunc[uartScanTemp-1].test_function();

        if (ret_val == TEST_SUCCESS)
        {
            DebugP_log("\nTest %s run successfully from DUT side...\n\r", UtilsTestFunc[uartScanTemp-1].test_name);
        }
        else
        {
            DebugP_log("\nTest %s not run successfully from DUT side...\n\r", UtilsTestFunc[uartScanTemp-1].test_name);
        }
    }
    else
    {
        DebugP_log("\nInput is invalid!\n\r");
    }

    //Individual Test summary
    DebugP_log("\n\r*****************************************************\n\r");
    DebugP_log("\tTEST UTILITY EXECUTION SUMMARY\n\r");
    DebugP_log("*****************************************************\n\r");

    return ret_value;
}

int32_t UTILS_display_icssg_hw_consolidated_statistics(void)
{
    uint32_t portNum;

    DebugP_log("*************************************************************************\n\r");
    DebugP_log("HW STAT PARAM\t\t\tPORT1\t\tPORT2\n\r");
    DebugP_log("*************************************************************************\n\r");
    portNum = ENET_MAC_PORT_1;
    UTILS_icssg_hw_consolidated_statistics(portNum);

    portNum = ENET_MAC_PORT_2;
    UTILS_icssg_hw_consolidated_statistics(portNum);

    return TEST_SUCCESS;
}

int32_t UTILS_clear_icssg_hw_consolidated_statistics(void)
{
    uint32_t portNum;

    DebugP_log("*************************************************************************\n\r");
    DebugP_log("HW STAT PARAMs RESET\t\t\tPORT1\t\tPORT2\n\r");
    DebugP_log("*************************************************************************\n\r");
    portNum = ENET_MAC_PORT_1;
    UTILS_icssg_hw_reset_consolidated_statistics(portNum);

    portNum = ENET_MAC_PORT_2;
    UTILS_icssg_hw_reset_consolidated_statistics(portNum);

    return TEST_SUCCESS;
}

void UTILS_icssg_hw_consolidated_statistics(uint32_t portNum)
{
    Enet_IoctlPrms prms;
    IcssgStats_MacPort stats;
    int32_t status;
#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[portNum];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
#endif

    DebugP_log("\nPrint statistics\n\r");
    DebugP_log("----------------------------------------------\n\r");

    DebugP_log("\n %s statistics\n\r", perCtxt->name);
    DebugP_log("--------------------------------\n\r");

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &perCtxt->macPort[portNum], &stats);
    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        DebugP_log("%s: Failed to get stats\n\r", perCtxt->name);
    }

    EnetAppUtils_printIcssgMacPortStats(&stats, false);
    DebugP_log("\n\r");
}

void UTILS_icssg_hw_reset_consolidated_statistics(uint32_t portNum)
{
    Enet_IoctlPrms prms;
    int32_t status;
#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[portNum];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
#endif

    DebugP_log("\nReset statistics\n\r");
    DebugP_log("----------------------------------------------\n\r");

    DebugP_log("%s: Reset statistics\n\r", perCtxt->name);

    ENET_IOCTL_SET_IN_ARGS(&prms, &perCtxt->macPort[portNum]);
    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        DebugP_log("%s: Failed to reset stats\n\r", perCtxt->name);
    }
}

int32_t UTILS_NRT_cofig_port_state(void)
{
    int32_t ret_val = TEST_FAILURE;
    uint32_t portState = 0;
    uint32_t selectPort = 0;

    DebugP_log("Selected Test case is Support for change in port states \n\r");
    DebugP_log("Enter 1 to Enable the PORTS\n\r");
    DebugP_log("Enter 2 to Disable the PORTS\n\r");
    DebugP_log("Enter 3 to Block the PORTS\n\r");
    DebugP_log("Enter 4 to enable forwarding with learning\n\r");
    DebugP_log("Enter 5 to enable forwarding without learning\n\r");
    DebugP_scanf("%u", &portState);

    switch (portState)
    {
        case 2:
            DebugP_log("Perform settings for the PORT1: (YES = 1) or (NO = 0) \n\r");
            DebugP_scanf("%u", &selectPort);

            if (selectPort)
            {
                ret_val = set_port_state(ENET_MAC_PORT_1, ICSSG_PORT_STATE_DISABLED);
                if (ret_val != TEST_SUCCESS)
                {
                    DebugP_log("ERROR: IOCTL command sent to disable PORT1 \n\r");
                    return TEST_FAILURE;
                }
            }

            DebugP_log("Perform settings for the PORT2: (YES = 1) or (NO = 0) \n\r");
            DebugP_scanf("%u", &selectPort);

            if (selectPort)
            {
                //PORT2
                ret_val = set_port_state(ENET_MAC_PORT_2, ICSSG_PORT_STATE_DISABLED);
                if (ret_val != TEST_SUCCESS)
                {
                    DebugP_log("ERROR: IOCTL command sent to disable PORT2 \n\r");
                    return TEST_FAILURE;
                }
            }

            break;

        default:
        case 4:
            DebugP_log("Perform settings for the PORT1: (YES = 1) or (NO = 0) \n\r");
            DebugP_scanf("%u", &selectPort);

            if (selectPort)
            {
                ret_val = set_port_state(ENET_MAC_PORT_1, ICSSG_PORT_STATE_FORWARD);
                if (ret_val != TEST_SUCCESS)
                {
                    DebugP_log("ERROR: IOCTL command sent to enable forwarding on PORT1 \n\r");
                    return TEST_FAILURE;
                }
            }

            DebugP_log("Perform settings for the PORT2: (YES = 1) or (NO = 0) \n\r");
            DebugP_scanf("%u", &selectPort);

            if (selectPort)
            {
                //PORT2
                ret_val = set_port_state(ENET_MAC_PORT_2, ICSSG_PORT_STATE_FORWARD);
                if (ret_val != TEST_SUCCESS)
                {
                    DebugP_log("ERROR: IOCTL command sent to enable forwarding on PORT2 \n\r");
                    return TEST_FAILURE;
                }
            }

            break;

        case 3:
            DebugP_log("Perform settings for the PORT1: (YES = 1) or (NO = 0) \n\r");
            DebugP_scanf("%u", &selectPort);

            if (selectPort)
            {
                ret_val = set_port_state(ENET_MAC_PORT_1, ICSSG_PORT_STATE_BLOCKING);
                if (ret_val != TEST_SUCCESS)
                {
                    DebugP_log("ERROR: IOCTL command sent to block PORT1 \n\r");
                    return TEST_FAILURE;
                }
            }

            DebugP_log("Perform settings for the PORT2: (YES = 1) or (NO = 0) \n\r");
            DebugP_scanf("%u", &selectPort);

            if (selectPort)
            {
                //PORT2
                ret_val = set_port_state(ENET_MAC_PORT_2, ICSSG_PORT_STATE_BLOCKING);
                if (ret_val != TEST_SUCCESS)
                {
                    DebugP_log("ERROR: IOCTL command sent to block PORT2 \n\r");
                    return TEST_FAILURE;
                }
            }

            //Now send sync or special packet from PORT2 using external packet generator
            //and PORT1 should receive that packet

            break;

        case 5:
            DebugP_log("Perform settings for the PORT1: (YES = 1) or (NO = 0) \n\r");
            DebugP_scanf("%u", &selectPort);

            if (selectPort)
            {
                ret_val = set_port_state(ENET_MAC_PORT_1, ICSSG_PORT_STATE_FORWARD_WO_LEARNING);
                if (ret_val != TEST_SUCCESS)
                {
                    DebugP_log("ERROR: IOCTL command sent to fowarding without learning PORT1 \n\r");
                    return TEST_FAILURE;
                }
            }

            DebugP_log("Perform settings for the PORT2: (YES = 1) or (NO = 0) \n\r");
            DebugP_scanf("%u", &selectPort);

            if (selectPort)
            {
                //PORT2
                ret_val = set_port_state(ENET_MAC_PORT_2, ICSSG_PORT_STATE_FORWARD_WO_LEARNING);
                if (ret_val != TEST_SUCCESS)
                {
                    DebugP_log("ERROR: IOCTL command sent to fowarding without learning PORT2 \n\r");
                    return TEST_FAILURE;
                }
            }

            break;
    }

    return ret_val;

}

int32_t TC_configure_vlan_aware(void)
{

    int32_t vlan_aware_mode = 0;
    int32_t ret_val = TEST_SUCCESS;

    DebugP_log("Configure the mode  0 - Vlan aware Disable, 1 - Vlan aware Enable\n\r");
    DebugP_scanf("%d", &vlan_aware_mode);

#if defined(DUAL_MAC_MODE)
    ret_val = set_vlan_aware_mode (ENET_MAC_PORT_1, vlan_aware_mode);
    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("ERROR: configuring vlan aware mode \n\r");
        return TEST_FAILURE;
    }
    ret_val = set_vlan_aware_mode (ENET_MAC_PORT_2, vlan_aware_mode);
    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("ERROR: configuring vlan aware mode \n\r");
        return TEST_FAILURE;
    }
#else
    ret_val = set_vlan_aware_mode (ENET_MAC_PORT_1, vlan_aware_mode);
    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("ERROR: configuring vlan aware mode \n\r");
        return TEST_FAILURE;
    }
#endif

    return ret_val;
}

int32_t set_vlan_aware_mode(uint8_t port_num, int32_t mode)
{
    int32_t ret_val = TEST_SUCCESS;
    Enet_IoctlPrms prms;
#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[port_num];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
#endif
    ENET_IOCTL_SET_NO_ARGS(&prms);

    if (mode == 1)
    {
        ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_PER_IOCTL_SET_VLAN_AWARE, &prms, ret_val);
    }
    else
    {
        ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ENET_PER_IOCTL_SET_VLAN_UNAWARE, &prms, ret_val);
    }


    if (ret_val != ENET_SOK)
    {
        DebugP_log("ERROR: %s: %d: IOCTL command sent \n\r", __FUNCTION__, __LINE__);
        ret_val = TEST_FAILURE;
    }

    return ret_val;
}

int32_t UTILS_NRT_add_del_mac_fdb_entry(void)
{
    int i = 0;
    uint32_t temp;
    Icssg_MacAddr Mac;
    uint8_t fdbEntry_p = 0x80;
    int16_t vid;
    int32_t ret_val = TEST_FAILURE;

    DebugP_log("To add/delete MAC-FDB entry \n\r");
    DebugP_log("Enter MAC address to make an entry (in hex) \n\r");

    for (i = 0; i < ENET_MAC_ADDR_LEN; i++)
    {
        DebugP_log("\n Please Enter Byte[%u] in hex", i);
        temp = EnetAppUtils_getHex();
        Mac.macAddr[i] = temp;
    }

    DebugP_log("\n\r Enter VLAN-Id to map this MAC-FDB entry");
    DebugP_scanf("%hi", &vid);

    DebugP_log("\n\r want to add(Enter 1) or delete(Enter 0) this entry\n\r ");
    DebugP_scanf("%u", &temp);

    if (temp)
    {
        DebugP_log("\n Enter fdb_entry bytes for Ports (in decimal)\n\r");
        DebugP_scanf("%u", &temp);

        fdbEntry_p = (uint8_t)temp;

        ret_val = add_mac_fdb_entry(Mac, vid, fdbEntry_p);
    }
    else
    {
        ret_val = del_mac_fdb_entry(Mac, vid);

    }

    return ret_val;
}

int32_t UTILS_NRT_add_fid_vid_entry(void)
{
    int32_t ret_val = TEST_FAILURE;
    uint32_t vlan_id;
    uint32_t taggedP1 = 0;
    uint32_t taggedP2 = 0;
    uint32_t memberP0 = 0;
    uint32_t memberP1 = 0;
    uint32_t memberP2 = 0;

    DebugP_log("Selected Test case is Support to make FID-VID entry for PORT1 and PORT2\n\r");
    DebugP_log("Enter the VLAN-ID to make an entry in FID-table .. \n\r");
    DebugP_scanf("%u", &vlan_id);

    DebugP_log("Do you want HOST to be a member of this VID (Enter 1) or (Enter 0) ?\n\r");
    DebugP_scanf("%u", &memberP0);

    DebugP_log("Do you want PORT1 to be a member of this VID (Enter 1) or (Enter 0) ?\n\r");
    DebugP_scanf("%u", &memberP1);

    if (memberP1)
    {
        DebugP_log("Do you want PORT1 as Tagged (Enter 1) or UnTagged( Enter 0) member for this VID ?\n\r");
        DebugP_scanf("%u", &taggedP1);
    }

    DebugP_log("Do you want PORT2 to be a member of this VID (Enter 1) or (Enter 0) ?\n\r");
    DebugP_scanf("%u", &memberP2);

    if (memberP2)
    {
        DebugP_log("Do you want PORT2 as Tagged (Enter 1) or UnTagged( Enter 0) member for this VID ?\n\r");
        DebugP_scanf("%u", &taggedP2);
    }

    /* Update fid-table with new entry */
    ret_val = add_fid_vid_entry((uint16_t)vlan_id, NRT_FID,
                                (uint8_t)memberP0,
                                (uint8_t)memberP1,
                                (uint8_t)memberP2,
                                0,
                                (uint8_t)taggedP1,
                                (uint8_t)taggedP2,
                                0,
                                0);
    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("ERROR: fid-table with new entry \n\r");
        return TEST_FAILURE;
    }

    return ret_val;
}

int32_t UTILS_config_PVID(void)
{
    uint16_t pVID = 1;
    uint8_t taggedP1 = 0;
    uint8_t taggedP2 = 0;
    int32_t ret_val = TEST_FAILURE;
    uint8_t memberP1 = 0;
    uint8_t memberP2 = 0;
    int32_t temp;

    DebugP_log("Selected Test case is Support change in PVID \n\r");
    DebugP_log("PVID will be assigned to HOST \n\r");

    DebugP_log("Enter the PVID you want to set for DUT.... \n\r");
    DebugP_scanf("%hu", &pVID);

    DebugP_log("Do you want PORT1 to be a member of this PVID (Enter 1) or (Enter 0) ?\n\r");
    DebugP_scanf("%d", &temp);
    memberP1 = (uint8_t)temp;

    if (memberP1)
    {
        DebugP_log("Do you want PORT1 as Tagged (Enter 1) or UnTagged( Enter 0) member for this PVID ?\n\r");
        DebugP_scanf("%d", &temp);
        taggedP1 = (uint8_t)temp;
    }

    DebugP_log("Do you want PORT2 to be a member of this PVID (Enter 1) or (Enter 0) ?\n\r");
    DebugP_scanf("%d", &temp);
    memberP2 = (uint8_t)temp;

    if(memberP2)
    {
        DebugP_log("Do you want PORT2 as Tagged (Enter 1) or UnTagged( Enter 0) member for this PVID ?\n\r");
        DebugP_scanf("%d", &temp);
        taggedP2 = (uint8_t)temp;
    }

        //-----------------Make a default entry for Host port-----------------
    ret_val = add_default_host_vid(0, pVID);

    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("\n ERROR: In updating default VLAN for Host : %d\n\r", ret_val);
    }

    //-----------------Make a default entry for P1 port-----------------
    ret_val = add_default_port_vid(ENET_MAC_PORT_1, 1, pVID);

    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("\n ERROR: In updating default VLAN for P1 \n\r");
    }

    //-----------------Make a default entry for P2 port-----------------
    ret_val = add_default_port_vid(ENET_MAC_PORT_2, 2, pVID);

    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("\n ERROR: In updating default VLAN for P2 \n\r");
    }

    ret_val = add_fid_vid_entry(pVID, 0, 1, memberP1, memberP2, 0, taggedP1, taggedP2, 0, 0);

    return ret_val;
}

int32_t UTILS_config_get_rx_pkt_count(void)
{
    DebugP_log("Receive Count: %d: \n\r", totalRxCnt);
    return TEST_SUCCESS;
}

int32_t UTILS_config_clear_rx_pkt_count(void)
{
    totalRxCnt=0;
    DebugP_log("Cleared Receive count \n\r");
    return TEST_SUCCESS;
}

int32_t UTILS_Transmit_UC_packets(void)
{
    uint8_t portNum;
    uint32_t numPackets, i, temp;
    DebugP_log("Enter port number(PORT1 = 1 and PORT2 = 2) to start transmitting the packets:\n\r");
    DebugP_scanf("%u", &temp);
    portNum = (uint8_t)temp;
    portNum = portNum - 1;
    DebugP_log("Enter number of packets to transmit:\n\r");
    DebugP_scanf("%u", &numPackets);

    for (i = 0; i < numPackets; i++)
    {
        UTILS_packetTx(portNum);
    }
    return TEST_SUCCESS;
}

int32_t setup_unit_test_default_settings(void)
{
    int32_t ret_val = TEST_SUCCESS;
    uint16_t vlan_id;
    Enet_IoctlPrms prms;
    uint32_t i;
    uint32_t prioRegenMap[ENET_PRI_NUM];
    uint32_t prioMap[ENET_PRI_NUM];
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
    Icssg_MacAddr Mac, special_mac;
    Icssg_VlanFidEntry vlanEntry;
    Icssg_VlanFidParams vlanParams = {
        .fid         = new_common_vlan_id,
        .hostMember  = 1,
        .p1Member    = 1,
        .p2Member    = 1,
        .hostTagged  = 0,
        .p1Tagged    = 0,
        .p2Tagged    = 0,
        .streamVid   = 0,
        .floodToHost = 0
    };
    Icssg_VlanFidParams vlanParamsForPrioTag = {
        .fid         = 0,
        .hostMember  = 1,
        .p1Member    = 1,
        .p2Member    = 1,
        .hostTagged  = 0,
        .p1Tagged    = 1,
        .p2Tagged    = 1,
        .streamVid   = 0,
        .floodToHost = 0
    };

    for (i = 0; i < ENET_PRI_NUM; i++)
    {
        prioRegenMap[i] = (uint32_t)(ENET_PRI_NUM-1-i);
        prioMap[i] = 0U;
    }

    //Set IOCTL command to make PORT1 as boundary port and PORT2 as accept all type of packets
    ret_val = set_priority_reg_mapping(ENET_MAC_PORT_1, prioRegenMap);
    ret_val = set_priority_reg_mapping(ENET_MAC_PORT_2, prioRegenMap);

    ret_val = set_priority_mapping(ENET_MAC_PORT_1, prioMap);
    ret_val = set_priority_mapping(ENET_MAC_PORT_2, prioMap);

    // Make an entry for common vlan id
    /* Update table for default entry */
    vlanEntry.vlanFidParams = vlanParams;
    vlanEntry.vlanId = (uint16_t)new_common_vlan_id;
    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_PER_IOCTL_VLAN_SET_ENTRY, &prms, ret_val);
    if (ret_val != ENET_SOK)
    {
        DebugP_log("FID VLAN entry for HOST is FAILED = %u : FAILED\n\r", ret_val);
    }

    //-----------------Make an entry for Priority tag-----------------

    vlanEntry.vlanFidParams = vlanParamsForPrioTag;
    vlanEntry.vlanId = (uint16_t)0;
    ENET_IOCTL_SET_IN_ARGS(&prms, &vlanEntry);

    ENET_IOCTL(perCtxt->handleInfo.hEnet, gEnetMp.coreId, ICSSG_PER_IOCTL_VLAN_SET_ENTRY, &prms, ret_val);
    if (ret_val != ENET_SOK)
    {
        DebugP_log("FID VLAN entry for VID = 0: FAILED = %u : FAILED\n\r", ret_val);
    }

    //-----------------Make a default entry for Host port-----------------
    ret_val = add_default_host_vid(0, new_common_vlan_id);
    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("\n ERROR: In updating default VLAN for Host : %d\n\r", ret_val);
    }

    //-----------------Make a default entry for P1 port-----------------
    ret_val = add_default_port_vid(ENET_MAC_PORT_1, 1, new_common_vlan_id);
    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("\n ERROR: In updating default VLAN for P1 \n\r");
    }

    //-----------------Make a default entry for P2 port-----------------
    ret_val = add_default_port_vid(ENET_MAC_PORT_2, 2, new_common_vlan_id);
    if (ret_val != TEST_SUCCESS)
    {
        DebugP_log("\n ERROR: In updating default VLAN for P2 \n\r");
    }

    //change DA with MAC = 0x01 0x02 0x03 0x04 0x05 0x06 for loopback packet
    Mac.macAddr[0] = 0x01;
    Mac.macAddr[1] = 0x02;
    Mac.macAddr[2] = 0x03;
    Mac.macAddr[3] = 0x04;
    Mac.macAddr[4] = 0x05;
    Mac.macAddr[5] = 0x06;

    uint8_t fdbEntry_port = 0x00;

    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_HOST_BIT));

    ret_val = add_mac_fdb_entry(Mac, new_common_vlan_id, fdbEntry_port);

    //A multi cast address for packet Rx on port1
    special_mac.macAddr[0] = 0x01;
    special_mac.macAddr[1] = 0x11;
    special_mac.macAddr[2] = 0x22;
    special_mac.macAddr[3] = 0x33;
    special_mac.macAddr[4] = 0x44;
    special_mac.macAddr[5] = 0x55;

    vlan_id = 100;
    fdbEntry_port = 0x00;
    fdbEntry_port = (fdbEntry_port |
                     (1 << FDB_ENTRY_VALID_BIT) |
                     (1 << FDB_ENTRY_BLOCK_BIT) |
                     (1 << FDB_ENTRY_PORT2_BIT) |
                     (1 << FDB_ENTRY_PORT1_BIT) |
                     (1 << FDB_ENTRY_HOST_BIT));

     ret_val = add_mac_fdb_entry(special_mac, vlan_id, fdbEntry_port);
     if (ret_val != TEST_SUCCESS)
     {
         DebugP_log("ERROR: adding MAC-FDB entry for port1\n\r");
         return TEST_FAILURE;
     }

    return ret_val;
}

void UTILS_packetTx(uint8_t port_num)
{

#if defined(DUAL_MAC_MODE)
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[port_num];
#else
    EnetMp_PerCtxt *perCtxt = &gEnetMp.perCtxt[0];
#endif
    EnetDma_PktQ txSubmitQ;
    EnetDma_Pkt *txPktInfo;
    EthVlanFrame *txFrame;
    int32_t status;

    /* Retrieve TX packets from driver and recycle them */
    EnetMp_retrieveFreeTxPkts(perCtxt);
    EnetQueue_initQ(&txSubmitQ);
    /* Dequeue one free TX Eth packet */
    txPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&gEnetMp.txFreePktInfoQ);
    if (txPktInfo != NULL)
    {
        /* Fill the TX Eth frame with test content */
        txFrame = (EthVlanFrame *)txPktInfo->sgList.list[0].bufPtr;
        memcpy(txFrame->hdr.dstMac, &txDstMac[0U], ENET_MAC_ADDR_LEN);
        memcpy(txFrame->hdr.srcMac, &perCtxt->macAddr[0U], ENET_MAC_ADDR_LEN);

        txFrame->hdr.tpid = 0x8100;
        txFrame->hdr.tci  = 0x0064;
        txFrame->hdr.etherType = txEthType;

        memcpy(&txFrame->payload[0U],
               &ENET_testPkt1[0U],
               sizeof(ENET_testPkt1));

        txPktInfo->sgList.list[0].segmentFilledLen = 128U;
        txPktInfo->sgList.numScatterSegments = 1;
        txPktInfo->chkSumInfo = 0U;
        txPktInfo->appPriv = &gEnetMp;
        txPktInfo->tsInfo.enableHostTxTs = false;
        txPktInfo->txPortNum = perCtxt->macPort[port_num];
        EnetDma_checkPktState(&txPktInfo->pktState,
                                ENET_PKTSTATE_MODULE_APP,
                                ENET_PKTSTATE_APP_WITH_FREEQ,
                                ENET_PKTSTATE_APP_WITH_DRIVER);

        /* Enqueue the packet for later transmission */
        EnetQueue_enq(&txSubmitQ, &txPktInfo->node);
    }
    else
    {
        DebugP_log("%s: Drop due to TX pkt not available\n\r", perCtxt->name);
    }

    status = EnetDma_submitTxPktQ(perCtxt->hTxCh, &txSubmitQ);
    if (status != ENET_SOK)
    {
        DebugP_log("%s: Failed to submit TX pkt queue: %d\n\r", perCtxt->name, status);
    }
}
