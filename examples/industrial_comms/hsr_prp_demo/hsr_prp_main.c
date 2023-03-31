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

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "hsr_prp_menu.h"
#include "tiswitch_pruss_intc_mapping.h"
#include <hsr_prp_soc.h>

#include <networking/icss_emac/icss_emac.h>
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"

#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_firmwareOffsets.h>

#ifdef ICSS_PROTOCOL_RED
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_nodeTable.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_snmp.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_statistics.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_handle.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_multicastTable.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_vlanTable.h>
#endif /* ICSS_PROTOCOL_RED */

#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_config.h>
#include <industrial_comms/hsr_prp/icss_fwhal/firmware/icss_emac_mmap.h>
#include <networking/icss_emac/lwipif/inc/lwip2icss_emac.h>
#include <drivers/mdio.h>

/*--------------------------Stack Related Includes----------------------------*/

#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/autoip.h"
#include "default_netif.h"
/* include the port-dependent configuration */
#include "hsr_prp_lwipcfg.h"
#include "examples/lwiperf/lwiperf_example.h"
#include "udp_iperf.h"

/*SNMP related */
#include "hsr_prp_prvmib.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define RED_PRINTF_UART

#define RED_REM_NODE_TYPE_STR_DANP     "DANP   "
#define RED_REM_NODE_TYPE_STR_VDANP    "VDANP  "
#define RED_REM_NODE_TYPE_STR_REDBOXP  "REDBOXP"
#define RED_REM_NODE_TYPE_STR_DANH     "DANH   "
#define RED_REM_NODE_TYPE_STR_VDANH    "VDANH  "
#define RED_REM_NODE_TYPE_STR_REDBOXH  "REDBOXH"
#define RED_REM_NODE_TYPE_STR_SANA     "SAN A  "
#define RED_REM_NODE_TYPE_STR_SANB     "SAN B  "
#define RED_REM_NODE_TYPE_STR_SANAB    "SAN AB "
#define RED_REM_NODE_TYPE_STR_UNKNOWN  "UNKNOWN"

#define RED_HSR_LRE_MODE_H             "MODE H"
#define RED_HSR_LRE_MODE_N             "MODE N"
#define RED_HSR_LRE_MODE_T             "MODE T"
#define RED_HSR_LRE_MODE_U             "MODE U"
#define RED_HSR_LRE_MODE_M             "MODE M"
#define RED_HSR_LRE_MODE_UNKNOWN       "UNKNOWN"

#define PCP_2_QUEUE_MAP_SIZE    4

/** application version */
#define APP_VERSION "2.0.0.0"

#define LEDTASK1_TASK_PRIORITY            (5)
#define LEDTASK1_TASK_STACK_SIZE          (0x4000)

#define LEDTASK2_TASK_PRIORITY            (5)
#define LEDTASK2_TASK_STACK_SIZE          (0x4000)

#define LWIPINIT_TASK_PRIORITY            (8)
#define LWIPINIT_TASK_STACK_SIZE          (0x4000)

#define UARTMENU_TASK_PRIORITY            (2)
#define UARTMENU_TASK_STACK_SIZE          (0x4000)

#define PTPSTATUS_TASK_PRIORITY            (6)
#define PTPSTATUS_TASK_STACK_SIZE          (0x4000)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern uint8_t hsrPrpTestFrame[HSR_PRP_TEST_FRAME_SIZE];
extern uint8_t check_ip;
extern char IPString[20];

TaskP_Object taskLedBlink2Object;
TaskP_Object taskLedBlinkObject;
TaskP_Object taskLwipInitObject;
#ifdef ICSS_PROTOCOL_RED
TaskP_Object taskUartMenuObject;
uint32_t gtaskUartMenuStack[UARTMENU_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
#endif
#ifdef PTP_TESTING
TaskP_Object monitorPTPStatus_TaskObject;
uint32_t gmonitorPTPStatus_TaskStack[PTPSTATUS_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
#endif

uint32_t gtaskLedBlink2Stack[LEDTASK1_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
uint32_t gtaskLedBlinkStack[LEDTASK2_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
uint32_t gtaskLwipInitStack[LWIPINIT_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

/**
 * Default PCP to Q mapping. See description of function hsrprp_initializePCPtoQueueMap below
 */
uint8_t default_pcp_2_queue_map[PCP_2_QUEUE_MAP_SIZE] = {ICSS_EMAC_QUEUE4, ICSS_EMAC_QUEUE3, ICSS_EMAC_QUEUE2, ICSS_EMAC_QUEUE1};

/** \brief PRU-ICSS LLD handle */
PRUICSS_Handle prusshandle;
/** \brief ICSSEmac LLD handle */
ICSS_EMAC_Handle emachandle;
/** \brief Time Sync handle */
TimeSync_ParamsHandle_t timeSyncHandle;
/** \brief hsrprp handle */
hsrPrpHandle *hsrPrphandle;
/** \brief LwIP Interface layer handle */
Lwip2Emac_Handle Lwipif_handle;
uint8_t lclMac[6];
uint32_t ipAddress;
int collision_pkt_dropped;
int num_of_collision_occured;
uint8_t hsr_prp_testPkt[HSR_PRP_TEST_FRAME_SIZE] = {
    0x6a, 0x3e, 0x26, 0x7a, 0xb2, 0x7d,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
    0x00,0x45, 0x00, 0x00, 0x2E, 0x00,
    0x00, 0x40, 0x00, 0x40, 0x00, 0x3A,
    0xD1
};

uint8_t hsr_prp_testPacketArray[1500] = {0};
/*-----------------------Stack Global Variable--------------------*/
struct dhcp netif_dhcp;
struct autoip netif_autoip;
/*-----------------------PTP global variables---------------------*/

/**When clock goes into sync for very timeset this*/
uint8_t deviceInSync = 0;
/**If clock goes out of sync and dut was in sync then set this*/
uint8_t ptpError = 0;

/**PTP MAC ID for comparison*/
uint8_t timeSyncMAC[6] = {0x1, 0x1b, 0x19, 0x0, 0x0, 0x0};
uint8_t linkLocalMAC[6] = {0x1, 0x80, 0xc2, 0x0, 0x0, 0xE};

#ifdef PTP_TESTING

/*Min/max PTP offsets used for tracking offset from master*/
extern int32_t min_offset;
extern int32_t max_offset;

#endif /*PTP_TESTING*/

/**Temporary placeholder to copy packets*/
uint8_t  tempFrame[ICSS_EMAC_MAXMTU]__attribute__((aligned(32)));
/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 * @brief   Initialize the PCP to Queue Map to default value
 *          i.e. Queue 4 or lowest priority queue
 *
 * @param   emachandle Pointer to EMAC handle. Used to fetch shared RAM base address
 *
 * @return  none
 */
void hsrprp_initializePCPtoQueueMap(ICSS_EMAC_Handle emachandle);

/**
 * @brief   Load the PCP to Queue Map to shared RAM.
 *          The firmware uses this to
 *
 * @param   emachandle Pointer to EMAC handle. Used to fetch shared RAM base address
 * @param   pcp_2_queue_map pointer to PCP to Q mapping.
 *
 * The mapping is very simple. The memory is an 4 bytes char array
 * with each byte containing the queue. The index of the byte corresponds to the
 * PCP value. Here lower value is higher priority queue, see default_pcp_2_queue_map for example
 * So we have as an example (see default_pcp_2_queue_map)
 * PCP         Queue       Mem Offset (QUEUE_2_PCP_MAP_OFFSET is base)
 * 0           Q4          0
 * 1           Q4          0
 * 2           Q3          1
 * ...
 * ...
 * 7           Q1          3
 *
 * @return  none
 */
void hsrprp_loadPCPtoQueueMap(ICSS_EMAC_Handle emachandle, uint8_t *pcp_2_queue_map);

/*This is kept in application for now*/
/*RT frames are processed here including PTP frames*/
void hsrprp_processHighPrioFrames(void *icssEmacHandle, void *queue_number, void *userArg);

/**
 *  @brief  Function to initialize the PTP driver handle
 *
 *          Does Memory allocation, EDMA param configuration
 *
 *
 *  @param   timeSyncHandle   [in]  PTP driver handle
 *  @param   emachandle  [in]  EMAC LLD handle
 *
 *  @retval  Error status
 *
 */
int8_t hsrprp_initICSSPtpHandle(TimeSync_ParamsHandle_t timeSyncHandle,
                                ICSS_EMAC_Handle emachandle);

/**
 *  @brief  Function to initialize the HSR/PRP driver handle
 *
 *          Implemented to facilitate multi-instance capability.
 *          Moving all global variables across the driver files to this handle in hsrPrp_handle.h file
 *
 *  @param   hsrPrpHandle   [in]  HSR driver handle
 *
 *  @retval  none
 *
 */
void hsrPrp_handleInit(hsrPrpHandle *hsrPrphandle);

/**
 *  @brief  Function to register the Port 1 LinkReset callback
 *
 *  @param   linkStatus   [in]  link status
 *  @param   arg2         [in]  pointer to a handle
 *
 *  @retval  none
 *
 */
void hsrPrp_Port1linkResetCallBack(void *icssEmacHandle, void *linkStatus, void *userArg);

/**
 *  @brief  Function to register the Port 2 LinkReset callback
 *
 *  @param   linkStatus   [in]  link status
 *  @param   arg2         [in]  pointer to a handle
 *
 *  @retval  none
 *
 */
void hsrPrp_Port2linkResetCallBack(void *icssEmacHandle, void *linkStatus, void *userArg);
/**
 *  @brief  Function to get ICSS EMAC handle info
 *
 *          Implemented to provide LwIP interface layer info about the EMAC Handle from application
 *
 *  @param   hLwip2emac   [in]  Lwip2emac Handle
 *
 *  @retval  SystemP_SUCCESS
 *              SystemP_FALIURE
 *
 */
int32_t app_getEmacHandle(Lwip2Emac_Handle hLwip2Emac);
/**
 *  @brief   This function initializes this lwIP test. When NO_SYS=1, this is done in
 *           the main_loop context (there is no other one), when NO_SYS=0, this is done
 *           in the tcpip_thread context
 *
 *  @param   none None
 *
 *  @retval none
 *
 */
void hsrprp_LwipInitStack();
static void hsrprp_LwipTest_init(void * arg);
static void hsrprp_LwipTest_netif_init(void);
static void hsrprp_LwipApps_init(void);
static void hsrprp_LwipStatus_callback(struct netif *state_netif);
static void hsrprp_LwipLink_callback(struct netif *state_netif);
extern void Lwip2Emac_getHandle(Lwip2Emac_Handle *AppLwipHandle);
static int32_t hsrprp_LoadFirmware(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  @brief  Function which calls an API in the event of a synchronization loss
 *
 *  @retval  None
 */
void hsrprp_syncLossCallback()
{
    /*This indicates a loss of time sync
      Call your function here which handles sync loss*/

    return;
}

/**
 *  @brief  Function which resets TAS in the event of a synchronization loss
 *
 *  @retval  None
 */
uint8_t hsrprp_AppGPTPSynchronized()
{

    if(timeSyncHandle->tsRunTimeVar->stateMachine & TS_STATE_MACHINE_DEVICE_IN_SYNC)
    {
        return TRUE;
    }

    else
    {
        return FALSE;
    }
}

/**
 * @brief main task to initialize  the stack
 *
 * @param args not used
 *
 * @return none
 */
void taskLwip(void *args)
{
    hsrprp_LwipInitStack();

    TaskP_destruct(&taskLwipInitObject);

}

#ifdef ICSS_PROTOCOL_RED

/*
 *  ---UART Menu task---
 */
void taskUartMenu(void * args)
{
    ICSS_EMAC_Handle emachandle ;
    char rxByte = 0;

    /* Wait for PRU load complete */
    ClockP_usleep(ClockP_ticksToUsec(3000));
    emachandle = (ICSS_EMAC_Handle)args;

    while(1)
    {
        DebugP_log("\n\r\n\r");
        printConfig(emachandle);

        DebugP_scanf("%c", &rxByte);

        if((rxByte == 'S') || (rxByte == 's'))
        {
            printStatistics(emachandle);
            DebugP_log("\n\rShow Statistics...completed\n\r");

        }

        else if((rxByte == 'L') || (rxByte == 'l'))
        {
            printLreStatistics(emachandle);
            DebugP_log("\n\rShow LRE Statistics...completed\n\r");

        }

        else if((rxByte == 'N') || (rxByte == 'n'))
        {
            printNodeTable(hsrPrphandle);
            DebugP_log("\n\rShow Ring members/Node Table...completed\n\r");
        }

        else if((rxByte == 'M') || (rxByte == 'm'))
        {
            printMulticastFilterSubMenu();
            DebugP_scanf("%c", &rxByte);

            if((rxByte == 'D') || (rxByte == 'd'))
            {
                hsrPrp_multicast_filter_config(prusshandle, MULTICAST_FILTER_DISABLED);
                DebugP_log("\n\rMULTICAST_FILTER_DISABLED...completed\n\r");
            }


            else if((rxByte == 'E') || (rxByte == 'e'))
            {
                hsrPrp_multicast_filter_config(prusshandle, MULTICAST_FILTER_ENABLED);
                DebugP_log("\n\rMULTICAST_FILTER_ENABLED...completed\n\r");
            }

            else
            {
                char bkupMacidString[30];

                DebugP_readLine(&bkupMacidString[0], 30);
                DebugP_log("%s\n\r", bkupMacidString);
#if DEBUG_MC_FLT
                DebugP_log(bkupMacidString);
#endif

                if(isValidMacid(bkupMacidString))
                {
                    if((rxByte == 'M') || (rxByte == 'm'))
                    {
                        hsrPrp_multicast_filter_override_hashmask(prusshandle, (uint8_t *)bkupMacidString);
                        DebugP_log("\n\rSuccessfully configured new mask");
                    }

                    else if((rxByte == 'A') || (rxByte == 'a'))
                    {
                        hsrPrp_multicast_filter_update_macid(prusshandle, (uint8_t *)bkupMacidString, ADD_MULTICAST_MAC_ID);
                        DebugP_log("\n\rSuccessfully ADD_MULTICAST_MAC_ID");
                    }

                    else if((rxByte == 'R') || (rxByte == 'r'))
                    {
                        hsrPrp_multicast_filter_update_macid(prusshandle, (uint8_t *)bkupMacidString, REMOVE_MULTICAST_MAC_ID);
                        DebugP_log("\n\rSuccessfully REMOVE_MULTICAST_MAC_ID");
                    }
                }
            }
        }

        else if((rxByte == 'V') || (rxByte == 'v'))
        {
            printVlanFilterSubMenu();
            DebugP_scanf("%c", &rxByte);

            if((rxByte == 'D') || (rxByte == 'd'))
            {
                vlan_filter_config(prusshandle, VLAN_FLTR_DIS);
                DebugP_log("\n\rVLAN_FILTER_DISABLE...completed\n\r");
            }

            else if((rxByte == 'E') || (rxByte == 'e'))
            {
                vlan_filter_config(prusshandle, VLAN_FLTR_ENA);
                DebugP_log("\n\rVLAN_FILTER_ENABLE...completed\n\r");
            }

            else if((rxByte == 'A') || (rxByte == 'a'))
            {
                DebugP_log("\n\rEnter VLAN-Id in integer format...press enter\n\r");
                char bkupMacidString[30];
                DebugP_readLine(bkupMacidString, 30);

                unsigned int vid;
                sscanf(bkupMacidString, "%u", &vid);

                if(vid <= VLAN_VID_MAX)  /*No need to check for vid >= VLAN_VID_MIN as CWARN.NOEFFECT.UCMP.GE: Comparison of unsigned value against 0 is always true*/
                {
                    vlan_filter_update_vid(prusshandle, (uint16_t)vid, ADD_VLAN_VID);
                }
                DebugP_log("\n\rADD_VLAN_VID...completed\n\r");
            }

            else if((rxByte == 'R') || (rxByte == 'r'))
            {
                DebugP_log("\n\rENter VLAN-Id in integer format...press enter\n\r");
                char bkupMacidString[30];
                DebugP_readLine(&bkupMacidString[0], 30);

                unsigned int vid;
                sscanf(bkupMacidString, "%u", &vid);

                if(vid <= VLAN_VID_MAX)  /*No need to check for vid >= VLAN_VID_MIN as CWARN.NOEFFECT.UCMP.GE: Comparison of unsigned value against 0 is always true*/
                {
                    vlan_filter_update_vid(prusshandle, (uint16_t)vid, REM_VLAN_VID);
                }

                DebugP_log("\n\rREM_VLAN_VID...completed\n\r");
            }

            else if((rxByte == 'H') || (rxByte == 'h'))
            {
                vlan_untagged_frames_config(prusshandle,
                                            VLAN_FLTR_UNTAG_HOST_RCV_ALL);
                DebugP_log("\n\rUntagged frames host receive enable...completed\n\r");
            }

            else if((rxByte == 'N') || (rxByte == 'n'))
            {
                vlan_untagged_frames_config(prusshandle,
                                            VLAN_FLTR_UNTAG_HOST_RCV_NAL);
                DebugP_log("\n\rUntagged frames host receive disable...completed\n\r");
            }

            else if((rxByte == 'P') || (rxByte == 'p'))
            {
                vlan_priotag_frames_config(prusshandle,
                                           VLAN_FLTR_PRIOTAG_HOST_RCV_ALL);
                DebugP_log("\n\rPriority frames host receive enable...completed\n\r");
            }

            else if((rxByte == 'Q') || (rxByte == 'q'))
            {
                vlan_priotag_frames_config(prusshandle,
                                           VLAN_FLTR_PRIOTAG_HOST_RCV_NAL);
                DebugP_log("\n\rPriority frames host receive disable...completed\n\r");
            }

            else if((rxByte == 'S') || (rxByte == 's'))
            {
                vlan_sv_frames_config(prusshandle,
                                      VLAN_FLTR_SV_VLAN_FLOW_SKIP);
                DebugP_log("\n\rSupervision frames to skip VLAN filter flow...completed\n\r");
            }

            else if((rxByte == 'F') || (rxByte == 'f'))
            {
                vlan_sv_frames_config(prusshandle,
                                      VLAN_FLTR_SV_VLAN_FLOW_TAKE);
                DebugP_log("\n\rSupervision frames to enter VLAN filter flow...completed\n\r");
            }
        }

        else if((rxByte == 'E') || (rxByte == 'e'))
        {
            void print_cpu_load();
            print_cpu_load();
        }

        else if((rxByte == 'F') || (rxByte == 'f'))
        {
            char bkupMacidString[30];
            DebugP_readLine(&bkupMacidString[0], 30);

            unsigned int intrPacStatus;
            sscanf(bkupMacidString, "%u", &intrPacStatus);
            intr_pac_config(emachandle, intrPacStatus);
            DebugP_log("\n\rHost Rx Interrupt Config...completed\n\r");
        }

        else if((rxByte == 'I') || (rxByte == 'i'))
        {
            DebugP_log("\n\rType IP Address and hit Enter:\t");

            if(readAndAssignIPAddress())
            {
                DebugP_log("\n\rSuccessfully assigned new IP address");
            }
            DebugP_log("\n\rAssign IP address...completed\n\r");
        }

        else if((rxByte == 'P') || (rxByte == 'p'))
        {
            printPTPSubMenu();
            DebugP_scanf("%c", &rxByte);

            if((rxByte == 's') || (rxByte == 'S'))
            {
                printPTPStatus(timeSyncHandle);
            }

            else if((rxByte == 'r') || (rxByte == 'R'))
            {
#ifdef PTP_TESTING
                min_offset = PTP_MIN_OFFSET_INIT_VAL;
                max_offset = PTP_MAX_OFFSET_INIT_VAL;
#endif /*PTP_TESTING*/
                ptpError = 0;
                deviceInSync = 0;
            }

            else
            {
                DebugP_log("\n\r\n\rSorry didnt catch that. Unknown input\n\r");
                DebugP_log("\n\r");
            }


        }

        else if((rxByte == 'R') || (rxByte == 'r'))
        {
            DebugP_log("\n\rRun Rx/Tx test....started");
            runRxTxTest(emachandle);
            DebugP_log("\n\rRun Rx/Tx test....completed");

        }

        else if((rxByte == 'C') || (rxByte == 'c'))
        {
            printConfiguration(emachandle);

        }

        else if((rxByte == 'H') || (rxByte == 'h'))
        {
            printHelpMenu();

        }
        else if((rxByte == 'X') || (rxByte == 'x'))
        {
            uint8_t type = 0;/* BC = 1; MC = 2; UC = 3 */
            uint8_t status = 0;/* enable = 1; disable = 0; reset = 2 */
            stormPreventionSettingsMenu();

            DebugP_scanf("%c", &rxByte);
            if((rxByte == 'B') || (rxByte == 'b'))
            {
                type = 1;
                enableDisablestormPrevention();
                DebugP_scanf("%c", &rxByte);
                if((rxByte == 'E') || (rxByte == 'e'))
                {
                    status = 1;
                }
                else if((rxByte == 'D') || (rxByte == 'd'))
                {
                    status = 0;
                }
                else
                {
                    status = 2;
                }

                stormPrevention(emachandle, type, status);
            }
            else if((rxByte == 'M') || (rxByte == 'm'))
            {
                type = 2;
                enableDisablestormPrevention();
                DebugP_scanf("%c", &rxByte);
                if((rxByte == 'E') || (rxByte == 'e'))
                {
                    status = 1;
                }
                else if((rxByte == 'D') || (rxByte == 'd'))
                {
                    status = 0;
                }
                else
                {
                    status = 2;
                }
                stormPrevention(emachandle, type, status);

            }
            else if((rxByte == 'U') || (rxByte == 'u'))
            {
                type = 3;
                enableDisablestormPrevention();
                DebugP_scanf("%c", &rxByte);

                if((rxByte == 'E') || (rxByte == 'e'))
                {
                    status = 1;
                }
                else if((rxByte == 'D') || (rxByte == 'd'))
                {
                    status = 0;
                }
                else
                {
                    status = 2;
                }

             stormPrevention(emachandle, type, status);

            }
            else
            {
                DebugP_log("\n\r\n\rSorry, did not catch that. Unknown input\n\r");
                DebugP_log("\n\r");

            }

        }

        else
        {
            DebugP_log("\n\r\n\rSorry, did not catch that. Unknown input\n\r");
            DebugP_log("\n\r");
        }

    }

}
#endif /* ICSS_PROTOCOL_RED */
/**
 * @brief alive task - indicates running host processor by blinkind LED
 * also counts states and other conditions signaling
 *
 * @param args not used
 *
 */
void taskLedBlink(void *args)
{

    uint8_t macId[6];
    char macAdd[32];
    uint8_t dutInSync = 0;
    hsrprp_socgetMACAddress(macId);

#ifdef ICSS_PROTOCOL_HSR
    DebugP_log("\n\rHSR Sample application running ");
#endif

#ifdef ICSS_PROTOCOL_PRP
    DebugP_log("\n\rPRP Sample application running ");
#endif

    /* Print the IP address information only if one is present. */
    while(!check_ip)
    {
        if(netif_ip4_addr(Lwipif_handle->netif))
        {
            /* Yes the device was configured and we got the IP address/Mask */
            ip4addr_ntoa_r(netif_ip4_addr(Lwipif_handle->netif), IPString, 20);
            DebugP_log("\n\rAssigned IP \t: ");
            DebugP_log(IPString);
            check_ip = 1;
        }
    }

    sprintf(macAdd, "%x:%x:%x:%x:%x:%x", (char)macId[0], (char)macId[1],
            (char)macId[2], (char)macId[3], (char)macId[4], (char)macId[5]);
    DebugP_log("\n\rMac Id \t\t: ");
    DebugP_log(macAdd);

    /*Add mac address to HSR test frame*/
    addMACID(hsr_prp_testPkt + 6, macId);

    /*Add IP address*/
    ip4addr_aton(IPString, (ip4_addr_t *)&ipAddress); /*To change IP address to numeric format*/
    memcpy(hsrPrpTestFrame + START_OF_IP_ADDRESS, &ipAddress, 4);

    /*Add marker to payload*/
    hsrPrpTestFrame[START_OF_UDP_HEADER] = 0xAB;
    hsrPrpTestFrame[START_OF_UDP_HEADER + 1] = 0xAC;

    do
    {
        dutInSync = hsrprp_AppGPTPSynchronized();

        if((TRUE == dutInSync) && (deviceInSync == 0))
        {
            deviceInSync = 1;
        }

        if((deviceInSync == 1) && (FALSE == dutInSync))
        {
            ptpError = 1;
        }

        if(dutInSync)
        {
            if(ptpError)
            {
                ClockP_usleep(ClockP_ticksToUsec(1000));
            }

            else
            {
                ClockP_usleep(ClockP_ticksToUsec(1000));
            }
        }

        else if(timeSyncHandle->tsRunTimeVar->stateMachine &
                TS_STATE_MACHINE_FIRST_ADJUSTMENT_DONE)
        {
            if(ptpError)
            {
                ClockP_usleep(ClockP_ticksToUsec(200));
                ClockP_usleep(ClockP_ticksToUsec(200));
            }

            else
            {
                ClockP_usleep(ClockP_ticksToUsec(200));
                ClockP_usleep(ClockP_ticksToUsec(200));
            }

        }

        else
        {
            if(ptpError)
            {
                ClockP_usleep(ClockP_ticksToUsec(800));
                ClockP_usleep(ClockP_ticksToUsec(200));
            }

            else
            {
                ClockP_usleep(ClockP_ticksToUsec(800));
                ClockP_usleep(ClockP_ticksToUsec(200));
            }
        }
    }
    while(1);

}

/**
 * @brief LED task for second LED
 *
 * @param args not used
 */
void taskLedBlink2(void *args)
{
    /*Give an initial delay to make sure LED's are initialized*/
    ClockP_usleep(ClockP_ticksToUsec(1000));
    /**
     * Sequence for Device Status LED
     * Blink LED on and off
     */

    while(1)
    {
        LED_on(gLedHandle[CONFIG_LED1], 0);
        ClockP_usleep(ClockP_ticksToUsec(800));
        LED_off(gLedHandle[CONFIG_LED1], 0);
    }
}

/**
 * \brief the application entry point - initializes low level hardware & creates tasks
 *
 */
int hsr_prp_main()
{
    TaskP_Params            taskParams;
    uint32_t                status = SystemP_FAILURE;
    PRUICSS_IntcInitData    pruss_intc_initdata = PRUSS_INTC_INITDATA;
    ICSS_EMAC_Params        icssEmacParams;

    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

    prusshandle = PRUICSS_open(CONFIG_PRU_ICSS1);
    DebugP_assert(prusshandle != NULL);

#ifdef MDIO_MANUAL_MODE_ENABLED
    mdioManualModeSetup();
#endif

    /*Creating hsrPrp Handle*/
    hsrPrphandle = (hsrPrpHandle *)malloc(sizeof(hsrPrpHandle));
    DebugP_assert(hsrPrphandle != NULL);
    /*Getting Lwip2EmacHandle for interface layer*/
    Lwip2Emac_getHandle(&Lwipif_handle);

#ifdef HSR_PRP_RGMII
    hsrprp_rgmii_init();
    DebugP_log("Mode: RGMII\n");
#else
    hsrprp_mii_init();
    DebugP_log("Mode: MII\n");
#endif

    ICSS_EMAC_Params_init(&icssEmacParams);
    icssEmacParams.pruicssIntcInitData = &pruss_intc_initdata;
    icssEmacParams.fwStaticMMap = &(icss_emacFwStaticCfgLocal[1]);
    icssEmacParams.fwDynamicMMap = &icss_emacFwDynamicCfgLocal;
    icssEmacParams.fwVlanFilterParams = &icss_emacFwVlanFilterCfg;
    icssEmacParams.fwMulticastFilterParams = &icss_emacFwMulticastFilterCfg;
    icssEmacParams.pruicssHandle = prusshandle;
    icssEmacParams.callBackObject.port0LinkCallBack.callBack = (ICSS_EMAC_CallBack)hsrPrp_Port1linkResetCallBack;
    icssEmacParams.callBackObject.port0LinkCallBack.userArg = (void*)(timeSyncHandle);
    icssEmacParams.callBackObject.port1LinkCallBack.callBack = (ICSS_EMAC_CallBack)hsrPrp_Port2linkResetCallBack;
    icssEmacParams.callBackObject.port1LinkCallBack.userArg = (void*)(timeSyncHandle);
#ifdef ICSS_PROTOCOL_RED
    /*Packet processing callback*/
    icssEmacParams.callBackObject.rxRTCallBack.callBack = (ICSS_EMAC_CallBack)hsrprp_processHighPrioFrames;
    icssEmacParams.callBackObject.rxRTCallBack.userArg = (void*)(emachandle);
    icssEmacParams.callBackObject.rxNRTCallBack.callBack = (ICSS_EMAC_CallBack)Lwip2Emac_serviceRx;
    icssEmacParams.callBackObject.rxNRTCallBack.userArg = (void*)(Lwipif_handle);
    icssEmacParams.callBackObject.customRxCallBack.callBack = (ICSS_EMAC_CallBack)RedRxPktGet;
    icssEmacParams.callBackObject.customRxCallBack.userArg = (void*)(hsrPrphandle);
    icssEmacParams.callBackObject.customTxCallBack.callBack = (ICSS_EMAC_CallBack)RedTxPacket;
    icssEmacParams.callBackObject.customTxCallBack.userArg = (void*)(hsrPrphandle);
#endif
    icssEmacParams.ethphyHandle[0] = gEthPhyHandle[CONFIG_ETHPHY0];
    icssEmacParams.ethphyHandle[1] = gEthPhyHandle[CONFIG_ETHPHY1];
    hsrprp_socgetMACAddress(lclMac);
    memcpy(&(icssEmacParams.macId[0]), &(lclMac[0]), 6);

    /*Creating timeSync Handle*/
    timeSyncHandle = (TimeSync_ParamsHandle_t)malloc(sizeof(TimeSync_ParamsHandle));
    DebugP_assert(timeSyncHandle != NULL);
    memset(timeSyncHandle, 0, sizeof(TimeSync_ParamsHandle));

    emachandle = ICSS_EMAC_open(CONFIG_ICSS_EMAC0, &icssEmacParams);
    DebugP_assert(emachandle != NULL);

    DebugP_assert(hsrprp_initICSSPtpHandle(timeSyncHandle, emachandle) == TIME_SYNC_OK);

    /*Initilaising timeSync Driver emac handle as emachandle is avilable now*/
    timeSyncHandle->emacHandle = emachandle;
    timeSyncHandle->pruicssHandle = prusshandle;

    /*Configure interrupts*/
    hsrprp_configureInterrupts(emachandle);

    hsrPrp_handleInit(hsrPrphandle);  /*  Initializing HSR handle */

#ifdef ICSS_PROTOCOL_RED
    RedProtocolInit();
    RedProtocolConfigure(prusshandle);
#endif /* ICSS_PROTOCOL_RED */

#ifdef ICSS_PROTOCOL_RED
    RedInit(prusshandle);
#endif /* ICSS_PROTOCOL_RED */

    status = hsrprp_LoadFirmware(); /*  Load Firmware   */
    if(status == SystemP_SUCCESS)
    {
        RedProtocolStart(hsrPrphandle, prusshandle);
        /*Load the default PCP to Queue mapping*/
        hsrprp_loadPCPtoQueueMap(emachandle, default_pcp_2_queue_map);
#ifdef ICSS_PROTOCOL_RED

        TimeSync_drvInit(timeSyncHandle);

        //ptpDrvStackInit(timeSyncHandle);

        /*Start PTP Engine*/
        TimeSync_drvEnable(timeSyncHandle);
        /*Override default setting. Not recommended, use only for testing*/
        //timeSyncHandle->timeSyncConfig.ll_has_hsrTag = 1;
#endif  /*ICSS_PROTOCOL_RED*/
    }
    TaskP_Params_init(&taskParams);
    taskParams.stackSize = LEDTASK1_TASK_STACK_SIZE;
    taskParams.priority = LEDTASK1_TASK_PRIORITY;
    taskParams.stack = (uint8_t *)gtaskLedBlinkStack;
    taskParams.name = "LEDTask1";
    taskParams.taskMain = (TaskP_FxnMain)taskLedBlink;
    status = TaskP_construct(&taskLedBlinkObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("LEDTask Creation failed\r\n");
    }


    TaskP_Params_init(&taskParams);
    taskParams.stackSize = LEDTASK2_TASK_STACK_SIZE;
    taskParams.priority = LEDTASK2_TASK_PRIORITY;
    taskParams.stack = (uint8_t *)gtaskLedBlink2Stack;
    taskParams.name = "LEDTask2";
    taskParams.taskMain = (TaskP_FxnMain)taskLedBlink2;
    status = TaskP_construct(&taskLedBlink2Object, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("LEDTask Creation failed\r\n");
    }

    TaskP_Params_init(&taskParams);
    taskParams.priority = LWIPINIT_TASK_PRIORITY;       /* we need stack prio higher than LED I/O*/
    taskParams.stackSize = LWIPINIT_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gtaskLwipInitStack;
    taskParams.name = "LwipInitTask";
    taskParams.taskMain = (TaskP_FxnMain)taskLwip;
    status = TaskP_construct(&taskLwipInitObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("LwipInitTask Creation failed\r\n");
    }

#ifdef ICSS_PROTOCOL_RED

    TaskP_Params_init(&taskParams);
    taskParams.priority = UARTMENU_TASK_PRIORITY;
    taskParams.name = "UARTMenuTask";
    taskParams.stackSize = UARTMENU_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *) gtaskUartMenuStack;
    taskParams.args = emachandle;
    taskParams.taskMain = (TaskP_FxnMain)taskUartMenu;
    status = TaskP_construct(&taskUartMenuObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("UARTMenuTask Creation failed\r\n");
    }

#endif /* ICSS_PROTOCOL_RED */

#ifdef PTP_TESTING
    TaskP_Params_init(&taskParams);
    taskParams.priority = PTPSTATUS_TASK_PRIORITY;
    taskParams.stackSize = PTPSTATUS_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *) gmonitorPTPStatus_TaskStack;
    taskParams.args = timeSyncHandle;
    taskParams.name = "PTPStatusTask";
    taskParams.taskMain = (TaskP_FxnMain)hsrprp_monitorPTPStatusTask;
    status = TaskP_construct(&monitorPTPStatus_TaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("PTPStatusTask Creation failed\r\n");
    }
#endif /*PTP_TESTING*/

    return status;
}


void hsrprp_processHighPrioFrames(void *icssEmacHandle, void *queue_number, void *userArg)
{

    int16_t size = 0;
    ICSS_EMAC_RxArgument rxArg;
    uint8_t *dstMacId;

    ICSS_EMAC_Handle hsrIcssEmacHandle = timeSyncHandle->emacHandle;

    rxArg.icssEmacHandle = hsrIcssEmacHandle;
    rxArg.destAddress = (uint32_t)(tempFrame);
    rxArg.more = 0;
    rxArg.queueNumber = (uint32_t)queue_number;
    rxArg.port = 0;

    size = (uint16_t)ICSS_EMAC_rxPktGet(&rxArg, &userArg);

    dstMacId = tempFrame;

    if(memcmp(dstMacId, timeSyncMAC, 6U) == 0)
    {
        TimeSync_processPTPFrame(timeSyncHandle, tempFrame, rxArg.port, size, 0);
    }

    else if(memcmp(dstMacId, linkLocalMAC, 6U) == 0)
    {
        TimeSync_processPTPFrame(timeSyncHandle, tempFrame, rxArg.port, size, 1);
    }

    else
    {
        parseAndCheckHSRPRPTestFrame(tempFrame);
    }
}

/**
 *  @brief  Function to initialize the PTP driver handle
 *
 *          Does Memory allocation, EDMA param configuration
 *
 *
 *  @param   timeSyncHandle   [in]  PTP driver handle
 *  @param   emachandle  [in]  EMAC LLD handle
 *
 *  @retval  Error status
 *
 */
int8_t hsrprp_initICSSPtpHandle(TimeSync_ParamsHandle_t timeSyncHandle,
                                ICSS_EMAC_Handle emachandle)
{
    int8_t returnVal = TIME_SYNC_OK;
    /*Configure PTP. These variables must be configured before doing anything else*/
    timeSyncHandle->timeSyncConfig.config = BOTH;
    timeSyncHandle->timeSyncConfig.type = P2P;
    timeSyncHandle->timeSyncConfig.protocol = IEEE_802_3;
    timeSyncHandle->timeSyncConfig.tickPeriod = 1000;
    timeSyncHandle->txprotocol = 0;

    timeSyncHandle->tsSyntInfo = (timeSync_SyntInfo_t *)malloc(sizeof(
                                     timeSync_SyntInfo_t));

    if(timeSyncHandle->tsSyntInfo == NULL)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    timeSyncHandle->tsNrrInfo[0] = (timeSync_NrrInfo_t *)malloc(sizeof(
                                       timeSync_NrrInfo_t));

    if(timeSyncHandle->tsNrrInfo[0] == NULL)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    timeSyncHandle->tsNrrInfo[1] = (timeSync_NrrInfo_t *)malloc(sizeof(
                                       timeSync_NrrInfo_t));

    if(timeSyncHandle->tsNrrInfo[1] == NULL)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    timeSyncHandle->syncParam[0] = (syncParam_t *)malloc(sizeof(syncParam_t));

    if(timeSyncHandle->syncParam[0] == NULL)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    timeSyncHandle->syncParam[1] = (syncParam_t *)malloc(sizeof(syncParam_t));

    if(timeSyncHandle->syncParam[1] == NULL)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    timeSyncHandle->tsRunTimeVar = (timeSync_RuntimeVar_t *)malloc(sizeof(
                                       timeSync_RuntimeVar_t));

    if(timeSyncHandle->tsRunTimeVar == NULL)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    timeSyncHandle->delayParams = (delayReqRespParams_t *)malloc(sizeof(
                                      delayReqRespParams_t));

    if(timeSyncHandle->delayParams == NULL)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    timeSyncHandle->offsetAlgo  = (timeSync_Offset_Stable_Algo_t *)malloc(sizeof(
                                      timeSync_Offset_Stable_Algo_t));

    if(timeSyncHandle->offsetAlgo == NULL)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    /*Allocate Rx and Tx packet buffers*/
    returnVal = TimeSync_alloc_PktBuffer(timeSyncHandle);

    if(returnVal != TIME_SYNC_OK)
    {
        return TIME_SYNC_UNABLE_TO_ALLOC_MEM;
    }

    /*Set only if a custom Tx LLD API is used*/
    timeSyncHandle->timeSyncConfig.custom_tx_api = 0;
    /*Set to 1 if Rx timestamps are coming from shared RAM, Set to 0 if Rx timestamps are coming from packet itself*/
    timeSyncHandle->timeSyncConfig.timestamp_from_shared_ram = 0;

#ifdef ICSS_PROTOCOL_HSR
    /*Enable only for HSR*/
    timeSyncHandle->timeSyncConfig.hsrEnabled = 1;
    /*If there is no forwarding between ports then set this*/
    timeSyncHandle->timeSyncConfig.emac_mode = 0;
#endif /*HSR*/

#ifdef ICSS_PROTOCOL_PRP
    timeSyncHandle->timeSyncConfig.hsrEnabled = 0;
    timeSyncHandle->timeSyncConfig.emac_mode = 1;
#endif /*PRP*/

    timeSyncHandle->rxTimestamp_gPTP = (timeStamp *)malloc(sizeof(timeStamp));

    timeSyncHandle->timeSyncConfig.domainNumber[0] = 0;

    timeSyncHandle->timeSyncConfig.logAnnounceRcptTimeoutInterval = DEFAULT_ANNOUNCE_TIMEOUT_LOG_INTERVAL;
    timeSyncHandle->timeSyncConfig.logAnnounceSendInterval = DEFAULT_ANNOUNCE_SEND_LOG_INTERVAL;
    timeSyncHandle->timeSyncConfig.logPDelReqPktInterval = DEFAULT_PDELAY_REQ_LOG_INTERVAL;
    timeSyncHandle->timeSyncConfig.logSyncInterval = DEFAULT_SYNC_SEND_LOG_INTERVAL;
    /*No asymmetry*/
    timeSyncHandle->timeSyncConfig.asymmetryCorrection[0] = 0;
    timeSyncHandle->timeSyncConfig.asymmetryCorrection[1] = 0;
    timeSyncHandle->timeSyncConfig.pdelayBurstNumPkts = 3;      /*3 frames sent in a burst*/
    timeSyncHandle->timeSyncConfig.pdelayBurstInterval = 200;   /*gap between each frame is 100ms*/
    timeSyncHandle->timeSyncConfig.sync0_interval = 1000000;      /*1 milisec value*/
    /*Register callback*/
    timeSyncHandle->timeSyncConfig.timeSyncSyncLossCallBackfn = (TimeSync_SyncLossCallBack_t)hsrprp_syncLossCallback;

    /*Configure Master params*/
    timeSyncHandle->timeSyncConfig.masterParams.priority1 = TIMESYNC_DEFAULT_PRIO_1;
    timeSyncHandle->timeSyncConfig.masterParams.priority2 = TIMESYNC_DEFAULT_PRIO_2;
    timeSyncHandle->timeSyncConfig.masterParams.clockAccuracy = TIMESYNC_DEFAULT_CLOCK_ACCURACY; /*greater than 10s */
    timeSyncHandle->timeSyncConfig.masterParams.clockClass = TIMESYNC_DEFAULT_CLOCK_CLASS;
    timeSyncHandle->timeSyncConfig.masterParams.clockVariance = TIMESYNC_DEFAULT_CLOCK_VARIANCE;
    timeSyncHandle->timeSyncConfig.masterParams.stepRemoved = TIMESYNC_DEFAULT_STEPS_REMOVED;
    timeSyncHandle->timeSyncConfig.masterParams.UTCOffset = TIMESYNC_UTC_OFFSET;
    timeSyncHandle->timeSyncConfig.masterParams.timeSource = TIMESYNC_DEFAULT_TIME_SOURCE; /*Internal oscillator*/

    timeSyncHandle->timeSyncConfig.masterParams.ptp_flags[TS_PTP_TIMESCALE_INDEX] = 1;
    timeSyncHandle->timeSyncConfig.masterParams.ptp_flags[TS_PTP_TWO_STEP_INDEX] = 1;

#ifdef HSR_PRP_RGMII
    timeSyncHandle->timeSyncConfig.rxPhyLatency = 534;
    timeSyncHandle->timeSyncConfig.txPhyLatency = 408;
#else
    timeSyncHandle->timeSyncConfig.rxPhyLatency = 220;
    timeSyncHandle->timeSyncConfig.txPhyLatency = 64;
#endif

    timeSyncHandle->timeSyncConfig.delayReqSendTaskPriority = 10;
    timeSyncHandle->timeSyncConfig.txTsTaskPriority = 10;
    timeSyncHandle->timeSyncConfig.nrtTaskPriority = 8;
    timeSyncHandle->timeSyncConfig.backgroundTaskPriority = 7;

    return TIME_SYNC_OK;
}

/*Initialize the PCP 2 Q mapping to 0 before starting firmware*/
void hsrprp_initializePCPtoQueueMap(ICSS_EMAC_Handle emachandle)
{
    uint8_t *bytePtr = (uint8_t *)((((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase) + QUEUE_2_PCP_MAP_OFFSET);
    memset(bytePtr, ICSS_EMAC_QUEUE4, PCP_2_QUEUE_MAP_SIZE);
}

/*Initialize the PCP 2 Q mapping to the required values*/
void hsrprp_loadPCPtoQueueMap(ICSS_EMAC_Handle emachandle, uint8_t *pcp_2_queue_map)
{
    uint8_t *bytePtr = (uint8_t *)((((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase) + QUEUE_2_PCP_MAP_OFFSET);
    memcpy(bytePtr, pcp_2_queue_map, PCP_2_QUEUE_MAP_SIZE);
}

void hsrPrp_handleInit(hsrPrpHandle *hsrPrphandle)
{
    hsrPrphandle->redSeqNr = 0;
    hsrPrphandle->supSeqNr = 0;
    hsrPrphandle->collision_pkt_dropped = &collision_pkt_dropped;
    hsrPrphandle->num_of_collision_occured = &num_of_collision_occured;
    hsrPrphandle->redPruCheckTimerHostTableFlag = 0;
    hsrPrphandle->redSupFrame = NULL;
    hsrPrphandle->icssEmacHandle = emachandle;

    hsrPrphandle->indexArrayBase = (RED_INDEX_ARRAY_ENTRY *)((((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase) + INDEX_ARRAY_NT);

    hsrPrphandle->binArrayBase = (RED_BIN_ARRAY_ENTRY *)((((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase) + BIN_ARRAY);

    hsrPrphandle->nodeTableBase = (RED_NODE_TABLE_ENTRY *)((((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase) + NODE_TABLE_NT);
}

void hsrPrp_Port1linkResetCallBack(void *icssEmacHandle, void *linkStatus, void *userArg)
{
    /*
    Here arg2 already points to timeSyncHandle because of the registered callBack
    Yet, explicit usage of timeSyncHandle keeping futuristic approach wherein we might want to add other callbacks and access to handles other than timeSyncHandle
    */
    /*TODO: Review this*/
    if(timeSyncHandle->enabled)
    {
        TimeSync_Port1linkResetCallBack((uint32_t)linkStatus, (void *)(timeSyncHandle));
    }
    else
    {
        DebugP_log("TimeSyncHandle not enabled.Port1 link call back execution failed");
    }
}

void hsrPrp_Port2linkResetCallBack(void *icssEmacHandle, void *linkStatus, void *userArg)
{
    /*
    Here arg2 already points to timeSyncHandle because of the registered callBack
    Yet, explicit usage of timeSyncHandle keeping futuristic approach wherein we might want to add other callbacks and access to handles other than timeSyncHandle
    */
    /*TODO: Review this*/
    if(timeSyncHandle->enabled)
    {
        TimeSync_Port2linkResetCallBack((uint32_t)linkStatus, (void *)(timeSyncHandle));
    }
    else
    {
         DebugP_log("TimeSyncHandle not enabled.Port2 link call back execution failed");
    }
}


int32_t app_getEmacHandle(Lwip2Emac_Handle hLwip2Emac)
{
    int32_t ret_val = SystemP_FAILURE;
    if(hLwip2Emac != NULL)
    {
        hLwip2Emac->emacHandle = emachandle;
        ret_val = SystemP_SUCCESS;
    }

    return (ret_val);
}

void hsrprp_LwipInitStack()
{
    err_t err;
    sys_sem_t init_sem;

    /* initialize lwIP stack, network interfaces and applications */
    err = sys_sem_new(&init_sem, 0);
    LWIP_ASSERT("failed to create init_sem", err == ERR_OK);
    LWIP_UNUSED_ARG(err);
    tcpip_init(hsrprp_LwipTest_init, &init_sem);
    /* we have to wait for initialization to finish before calling update_adapter() */
    sys_sem_wait(&init_sem);
    sys_sem_free(&init_sem);
}


static void hsrprp_LwipTest_init(void * arg)
{
    sys_sem_t *init_sem;
    LWIP_ASSERT("arg != NULL", arg != NULL);
    init_sem = (sys_sem_t*)arg;

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    /* init network interfaces */
    hsrprp_LwipTest_netif_init();

    /* init apps */
    hsrprp_LwipApps_init();

    sys_sem_signal(init_sem);
}

/* This function initializes applications */
static void hsrprp_LwipApps_init(void)
{
    /*Initialise SNMP example*/
    snmp_example_init();
    /*Initialise for Iperf Test*/
    lwiperf_example_init();
    print_app_header();
    sys_thread_new("UDP Iperf", start_application, NULL, DEFAULT_THREAD_STACKSIZE,
                             DEFAULT_THREAD_PRIO);

}

/* This function initializes all network interfaces */
static void hsrprp_LwipTest_netif_init(void)
{
    /*Variables to store ipAddress, Netmask & Gateway*/
    ip4_addr_t ipaddr, netmask, gw;
    /*Initialise network parameters to zero*/
    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    /*Assign the IP read from EEPROM*/
    EEPROM_read(gEepromHandle[CONFIG_EEPROM0], SPI_EEPROM_DEVICEIP_OFFSET, (uint8_t *)&ipAddress, 4);
    changeIPEndianness(&ipAddress);
    /*Converting Ip from EEPROM to ASCII format & assign to network parametr ipaddr*/
    if(ip4addr_aton(ip4addr_ntoa((ip4_addr_t *)&ipAddress), &ipaddr) && isValidIP(ip4addr_ntoa((ip4_addr_t *)&ipAddress)))
    {
        DebugP_log("Starting lwIP, local interface IP is %s\r\n", ip4addr_ntoa(&ipaddr));
    }
    else
    {
        /*Initialise default IP address : Can be changed in cfg file */
        LWIP_PORT_INIT_IPADDR(&ipaddr);
        DebugP_log("Starting lwIP, local interface IP is %s\r\n", ip4addr_ntoa(&ipaddr));
    }

    /*Initialise default gateway address : Can be changed in cfg file*/
    LWIP_PORT_INIT_GW(&gw);
    /*Initialise default Subnet mask : Can be changed in cfg file*/
    LWIP_PORT_INIT_NETMASK(&netmask);

    /*Initialise ICSS EMAC based netif as default netif with above parameters*/
    init_default_netif(&ipaddr, &netmask, &gw);

    /*Inform application about netif status*/
    netif_set_status_callback(netif_default, hsrprp_LwipStatus_callback);
    /*Inform application about link status*/
    netif_set_link_callback(netif_default, hsrprp_LwipLink_callback);

#if USE_DHCP
    err_t err;
    autoip_set_struct(netif_default, &netif_autoip);
    dhcp_set_struct(netif_default, &netif_dhcp);
#endif
    netif_set_up(netif_default);
#if USE_DHCP
    err = dhcp_start(netif_default);
    LWIP_ASSERT("dhcp_start failed", err == ERR_OK);
#elif USE_AUTOIP
    err = autoip_start(netif_default);
    LWIP_ASSERT("autoip_start failed", err == ERR_OK);
#endif
}

static void hsrprp_LwipStatus_callback(struct netif *state_netif)
{
    if (netif_is_up(state_netif))
    {
#if LWIP_IPV4
        DebugP_log("status_callback==UP, local interface IP is %s\r\n", ip4addr_ntoa(netif_ip4_addr(state_netif)));
#endif
    }
    else
    {
        DebugP_log("status_callback==DOWN\r\n");
    }
}

static void hsrprp_LwipLink_callback(struct netif *state_netif)
{
    if (netif_is_link_up(state_netif))
    {
        DebugP_log("link_callback==UP\r\n");
    }
    else
    {
        DebugP_log("link_callback==DOWN\r\n");
    }
}

void print_cpu_load()
{
    uint32_t cpu_load = TaskP_loadGetTotalCpuLoad();
    DebugP_log("CPU load = %3d.%2d %%\r\n", cpu_load/100, cpu_load%100);
    TaskP_loadResetAll();
}

static int32_t hsrprp_LoadFirmware()
{
    uint8_t firmware_Load = SystemP_FAILURE;

    /*Initialize the PCP 2 Q map*/
    hsrprp_initializePCPtoQueueMap(emachandle);
    /*Load Firmware*/
    firmware_Load = RedLoadFirmware(prusshandle);
    return firmware_Load;
}
