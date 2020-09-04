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

#include "hsr_prp_menu.h"
#include "hsr_prp_soc.h"
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_firmwareOffsets.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_config.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_statistics.h>
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_snmp.h>
#include <networking/icss_emac/source/icss_emac_local.h>
#include <networking/icss_emac/lwipif/inc/lwip2icss_emac.h>
#include "ti_board_open_close.h"
#include "ti_dpl_config.h"
#include "ti_drivers_open_close.h"
#include "lwip/netif.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
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

/**Interface macid index*/
#define INTERFACE_MAC 0
/**Port1 macid index*/
#define PORT1_MAC     1
/**Port2 macid index*/
#define PORT2_MAC     2

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern ICSS_EMAC_Handle         emachandle;
extern PRUICSS_Handle           prusshandle;
extern TimeSync_ParamsHandle_t  timeSyncHandle;
extern Lwip2Emac_Handle Lwipif_handle;

/*Indicates PTP error*/
extern uint8_t ptpError;
extern uint8_t deviceInSync;

#ifdef PTP_TESTING

/*Used to track min/max offset from master
 * Useful for testing*/
int32_t min_offset = PTP_MIN_OFFSET_INIT_VAL;
int32_t max_offset = PTP_MAX_OFFSET_INIT_VAL;

#endif /*PTP_TESTING*/

/*IP address stored here*/
char IPString[20];

/*Set to 1 after IP address is assigned*/
uint8_t check_ip = 0;

/**An HSR/PRP example frame with the header*/
uint8_t hsrPrpTestFrame[HSR_PRP_TEST_FRAME_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
                                                    0x45, 0x00, 0x00, 0x2E, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00, 0x3A, 0xD1
                                                   };

uint8_t vlanGooseTestFrame[HSR_PRP_TEST_FRAME_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                        0x81,   0x00,   0x80,   0x00,   0x88,   0xB8,   0x00,   0x00,    0x00,    0x50,    0x00,    0x00,    0x00,    0x00,    0x61,    0x46,
                                                        0x80,   0x07,   0x67,   0x6F,   0x43,   0x62,   0x52,   0x65,    0x31,    0x81,    0x02,    0x0B,    0xB8,    0x82,    0x06,    0x64,
                                                        0x61,   0x74,   0x53,   0x65,   0x74,   0x83,   0x04,   0x67,    0x6F,    0x49,    0x64,    0x84,    0x08,    0x48,    0x88,    0x90,
                                                        0xC1,   0x24,   0x00,   0x00,   0x27,   0x85,   0x01,   0x01,    0x86,    0x02,    0x00,    0xCD,    0x87,    0x01,    0x00,    0x88,
                                                        0x01,   0x78,   0x89,   0x01,   0x00,   0x8A,   0x01,   0x03,    0xAB,    0x0C,    0xA2,    0x80,    0x84,    0x03,    0x00,    0x80,
                                                        0x00,   0x83,   0x01,   0x00,   0x00,   0x00,
                                                        };

uint8_t GooseTestFrame[HSR_PRP_TEST_FRAME_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                    0x88,   0xB8,   0x00,   0x00,   0x00,   0x50,   0x00,   0x00,    0x00,    0x00,    0x61,    0x46,
                                                    0x80,   0x07,   0x67,   0x6F,   0x43,   0x62,   0x52,   0x65,    0x31,    0x81,    0x02,    0x0B,    0xB8,    0x82,    0x06,    0x64,
                                                    0x61,   0x74,   0x53,   0x65,   0x74,   0x83,   0x04,   0x67,    0x6F,    0x49,    0x64,    0x84,    0x08,    0x48,    0x88,    0x90,
                                                    0xC1,   0x24,   0x00,   0x00,   0x27,   0x85,   0x01,   0x01,    0x86,    0x02,    0x00,    0xCD,    0x87,    0x01,    0x00,    0x88,
                                                    0x01,   0x78,   0x89,   0x01,   0x00,   0x8A,   0x01,   0x03,    0xAB,    0x0C,    0xA2,    0x80,    0x84,    0x03,    0x00,    0x80,
                                                    0x00,   0x83,   0x01,   0x00,   0x00,   0x00,
                                                        };

                                                   /* board flash handle */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#ifdef ICSS_PROTOCOL_HSR
char *getHsrLreModePrintStr(ICSS_EMAC_Handle emachandle)
{
    uint16_t mode = *((uint16_t *)(((prusshandle->hwAttrs)->pru0DramBase) + LRE_HSR_MODE));

    switch(mode)
    {
        case MODEH:
            return RED_HSR_LRE_MODE_H;

        case MODEN:
            return RED_HSR_LRE_MODE_N;

        case MODET:
            return RED_HSR_LRE_MODE_T;

        case MODEU:
            return RED_HSR_LRE_MODE_U;

        case MODEM:
            return RED_HSR_LRE_MODE_M;

        default:
            break;
    }

    return RED_HSR_LRE_MODE_UNKNOWN;
}
#endif /* ICSS_PROTOCOL_HSR */
void printHelpMenu()
{
    DebugP_log("\n\r******************HSR/PRP Application Help Menu**********************\n\r");
    DebugP_log("\n\rNode Table:\n Every HSR/PRP device on the network broadcasts Supervision frames.\n\r");
    DebugP_log("The Node table implementation keeps track of it. It has a maximum of 256 entries.\n\r");
    DebugP_log("The node table view presents a consolidated view of the node table.\n\r");
    DebugP_log("\n\r");
    DebugP_log("\n\rStatistics:\n The implementation monitors incoming and outgoing frames.\n\r");
    DebugP_log("The statistics view shows a consolidated view of the same.\n\r");
    DebugP_log("\n\r");
    //DebugP_log("\n\rPTP/1588:\n TI HSR switch supports PTP/1588 Ordinary and Transparent Clock.\n\r");
    DebugP_log("The view shows certain key parameters critical for monitoring.\n\r");
    DebugP_log("\n\r");
    DebugP_log("\n\rRx/Tx Test:\n Select this option to run a simple redundancy test.\n");
    DebugP_log("\n\rUpon selecting the menu option a sub-menu option appears asking to fix a device\n\r");
    DebugP_log("as Receiver or Transmitter. If a device is chosen as a receiver it waits for 120 frames\n\r");
    DebugP_log("to be received or a 3 minute timeout (whichever is earlier) and then exits\n\r");
    DebugP_log("If a device is configured as Transmitter then it starts transmitting 1 frame per second on both ports\n\r");
    DebugP_log("\n\r\n\r");
    DebugP_log("\n\r************************End of Help Menu***************************\n\r");
}

void printConfig(ICSS_EMAC_Handle emachandle)
{
#ifdef ICSS_PROTOCOL_HSR
    char pBuf[100];
#endif

    if(check_ip)
    {
        /*Print IP address*/
        DebugP_log("\n\rIP Address \t: ");
        DebugP_log(IPString);

        /*Print MAC address*/
    }

#ifdef ICSS_PROTOCOL_HSR
    char *mode = getHsrLreModePrintStr(emachandle);
    sprintf(pBuf,
            "\n\r"
            "Device config\t: HSR %s\n\r",
            mode);
    DebugP_log(pBuf);
#endif /* ICSS_PROTOCOL_HSR */

#ifdef ICSS_PROTOCOL_PRP
    DebugP_log("\n\rDevice config\t: PRP \n\r");
#endif /* ICSS_PROTOCOL_HSR */
    DebugP_log("\n\rHSR/PRP Application Menu Options. Press key (Upper/Lower)\n");
    DebugP_log("*******************************************\n\r");
    DebugP_log("S : Show Statistics\n\r");
    DebugP_log("L : Show LRE Statistics\n\r");
    DebugP_log("C : Show HSR/PRP Configuration\n\r");
    DebugP_log("N : Show Ring members/Node Table\n\r");
    DebugP_log("M : Multicast filter handling\n\r");
    DebugP_log("V : VLAN filter handling\n\r");
    DebugP_log("E : Print CPU Load\n\r");
    DebugP_log("I : Assign IP address\n\r");
    DebugP_log("P : Show PTP/1588 status\n\r");
    DebugP_log("R : Run Rx/Tx test\n\r");
    DebugP_log("X : To perform DUT side settings\n\r");
    DebugP_log("H : Help menu. Shows details on all the options\n\r");
    DebugP_log("********************************************\n\r");
}

void printPTPSubMenu(void)
{
    DebugP_log("********************************************\n\r");
    DebugP_log("\n\rPTP Sub Menu Options. Press key (Upper/Lower)\n");
    DebugP_log("S : Show PTP/1588 status\n\r");
    DebugP_log("R : Reset min max values\n\r");
    DebugP_log("********************************************\n\r");
}

void printVlanFilterSubMenu(void)
{
    DebugP_log("********************************************\n\r");
    DebugP_log("\n\rVLAN filter Sub Menu Options. Press key (Upper/Lower)\n");
    DebugP_log("D : Disable VLAN filtering\n\r");
    DebugP_log("E : Enable VLAN filtering\n\r");
    DebugP_log("A : Add VID\n\r");
    DebugP_log("R : Remove VID\n\r");
    DebugP_log("H : Untagged frames host receive enable\n\r");
    DebugP_log("N : Untagged frames host receive disable\n\r");
    DebugP_log("P : Priority frames host receive enable\n\r");
    DebugP_log("Q : Priority frames host receive disable\n\r");
    DebugP_log("S : Supervision frames to skip VLAN filter flow\n\r");
    DebugP_log("F : Supervision frames to enter VLAN filter flow\n\r");
    DebugP_log("********************************************\n\r");
}

void printMulticastFilterSubMenu(void)
{
    DebugP_log("********************************************\n\r");
    DebugP_log("\n\rMulticast filter Sub Menu Options. Press key (Upper/Lower)\n");
    DebugP_log("D : Disable multicast filtering\n\r");
    DebugP_log("E : Enable multicast filtering\n\r");
    DebugP_log("M : Configure mask\n\r");
    DebugP_log("A : Add multicast MAC ID\n\r");
    DebugP_log("R : Remove multicast MAC ID\n\r");
    DebugP_log("********************************************\n\r");
}

void stormPreventionSettingsMenu(void)
{
    DebugP_log("********************************************\n\r");
    DebugP_log("\n\rStorm prevention/ IOCTL implementation. Press key (Upper/Lower)\n");
    DebugP_log("B : enable/disable BC Storm prevention \n\r");
    DebugP_log("M : enable/disable MC Storm prevention \n\r");
    DebugP_log("U : enable/disable UC Storm prevention \n\r");
    DebugP_log("R : Reset Storm prevention \n\r");
    DebugP_log("********************************************\n\r");
}

void enableDisablestormPrevention(void)
{
    DebugP_log("********************************************\n\r");
    DebugP_log("\n\rEnable Disable Storm prevention. Press key (Upper/Lower)\n");
    DebugP_log("E : To Enable \n\r");
    DebugP_log("D : To disable \n\r");
    DebugP_log("R : To reset \n\r");
    DebugP_log("********************************************\n\r");
}
/*reset storm frame prevention for both BC and MC frames*/
void stormPrevention(ICSS_EMAC_Handle icssEmacHandle, uint8_t type, uint8_t status)
{
    uint8_t index = 0;
    ICSS_EMAC_IoctlCmd ioctlParams;
    int8_t ioctlRetVal, uartScanTemp;
    uint8_t port[] = {ICSS_EMAC_PORT_0, ICSS_EMAC_PORT_1, ICSS_EMAC_PORT_2};
    //stormPrevention_t* stormPrevPtr;
    uint16_t credit = 2000;


    DebugP_log("\n\rEnter the packet count limit for storm prevention\n");
    DebugP_scanf("%d", &uartScanTemp);
    credit = (uint16_t)uartScanTemp;

    ioctlParams.ioctlVal = (void*)(&credit);

    switch(type)
    {
    /*BC case*/
    case 1:
        /*enable BC storm prevention*/
        if(status == TRUE)
        {
            for(index = 1 ; index < 3 ; index++)
            {

                ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_ENABLE_BC;
                ioctlRetVal = ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, port[index], (void*)&ioctlParams);
                if(ioctlRetVal != 0)
                {
                    DebugP_log("\n\rIOCTL(ICSS_EMAC_STORM_PREV_CTRL_ENABLE_BC) test for SWITCH failed\n");

                }
            }

        }
        /*disable*/
        else if(status == FALSE)
        {
            for(index = 1 ; index < 3 ; index++)
            {
                ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_DISABLE_BC;
                ioctlRetVal = ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, port[index], (void*)&ioctlParams);
                if(ioctlRetVal != 0)
                {
                    DebugP_log("\n\rIOCTL(ICSS_EMAC_STORM_PREV_CTRL_DISABLE_BC) test for SWITCH failed\n");

                }
            }
        }
        /*reset*/
        else
        {
            ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_RESET_BC;
            ioctlRetVal = ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, port[index], (void*)&ioctlParams);
            if(ioctlRetVal != 0)
            {
                DebugP_log("\n\rIOCTL(ICSS_EMAC_STORM_PREV_CTRL_RESET_BC) test for SWITCH failed\n");

            }
        }

        break;
    /*MC*/
    case 2:
        /*enable MC storm prevention*/
        if(status == TRUE)
        {
            for(index = 1 ; index < 3 ; index++)
            {

                ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_ENABLE_MC;
                ioctlRetVal = ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, port[index], (void*)&ioctlParams);
                if(ioctlRetVal != 0)
                {
                    DebugP_log("\n\rIOCTL(ICSS_EMAC_STORM_PREV_CTRL_ENABLE_MC) test for SWITCH failed\n");

                }
            }
        }
        /*disable MC storm prevention*/
        else if (status == FALSE)
        {
            for(index = 1 ; index < 3 ; index++)
            {

                ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_DISABLE_MC;
                ioctlRetVal = ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, port[index], (void*)&ioctlParams);
                if(ioctlRetVal != 0)
                {
                    DebugP_log("\n\rIOCTL(ICSS_EMAC_STORM_PREV_CTRL_DISABLE_MC) test for SWITCH failed\n");

                }
            }

        }
        /*Reset MC strom prevention*/
        else
        {
            for(index = 1 ; index < 3 ; index++)
            {
                ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_RESET_MC;
                ioctlRetVal = ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, port[index], (void*)&ioctlParams);
                if(ioctlRetVal != 0)
                {
                    DebugP_log("\n\rIOCTL(ICSS_EMAC_STORM_PREV_CTRL_RESET_MC) test for SWITCH failed\n");

                }
            }

        }
        break;
    /*UC*/
    case 3:
        /*enable UC storm prevention*/
        if(status == TRUE)
        {
            for(index = 1 ; index < 3 ; index++)
            {

                ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_ENABLE_UC;
                ioctlRetVal = ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, port[index], (void*)&ioctlParams);
                if(ioctlRetVal != 0)
                {
                    DebugP_log("\n\rIOCTL(ICSS_EMAC_STORM_PREV_CTRL_ENABLE_UC) test for SWITCH failed\n");

                }
             }
        }
        /*disable UC storm prevention*/
        else
        {
            for(index = 1 ; index < 3 ; index++)
            {
                ioctlParams.command = ICSS_EMAC_STORM_PREV_CTRL_DISABLE_UC;
                ioctlRetVal = ICSS_EMAC_ioctl(icssEmacHandle, ICSS_EMAC_IOCTL_STORM_PREV_CTRL, port[index], (void*)&ioctlParams);
                if(ioctlRetVal != 0)
                {
                    DebugP_log("\n\rIOCTL(ICSS_EMAC_STORM_PREV_CTRL_DISABLE_UC) test for SWITCH failed\n");

                }
            }
        }
        /*Reset UC storm prevention*/
        break;

    default:
        DebugP_log("\n\r\n\rSorry did not catch that. Unknown input\n\r");
        DebugP_log("\n\r");

        break;
    }
}


void printPTPStatus(TimeSync_ParamsHandle_t timeSyncHandle)
{
    char pBuf[1024];
    uint64_t seconds = 0;
    uint32_t nanoseconds = 0;

    if(!TimeSync_isEnabled(timeSyncHandle))
    {
        DebugP_log("\n\r PTP is disabled. Forgot to call ptpEnable() ?");
    }

    else
    {

        sprintf(pBuf, "\n\rPeer Delay on P1 :\t\t\t%d ns",
                (int)timeSyncHandle->tsRunTimeVar->pathDelay[0]);
        DebugP_log(pBuf);
        sprintf(pBuf, "\n\rPeer Delay on P2 :\t\t\t%d ns",
                (int)timeSyncHandle->tsRunTimeVar->pathDelay[1]);
        DebugP_log(pBuf);

        DebugP_log("\n\r\n\r--------DUT configured as Slave---------");

        TimeSync_getCurrentTime(timeSyncHandle, &nanoseconds, &seconds);

        DebugP_log("\n\r*********PTP/1588 Params********");
        sprintf(pBuf, "\n\rClock Drift :\t\t\t\t%d ns",
                (int)timeSyncHandle->tsRunTimeVar->clockDrift);
        DebugP_log(pBuf);
        sprintf(pBuf, "\n\rCurr offset :\t\t\t\t%d ns",
                (int)timeSyncHandle->tsRunTimeVar->currOffset);
        DebugP_log(pBuf);
#ifdef PTP_TESTING
        sprintf(pBuf, "\n\rMin offset :\t\t\t\t%d ns",
                (int)min_offset);
        DebugP_log(pBuf);
        sprintf(pBuf, "\n\rMax offset :\t\t\t\t%d ns",
                (int)max_offset);
        DebugP_log(pBuf);
        sprintf(pBuf, "\n\rNum Sync Missed :\t\t\t%d",
                (int)(int)timeSyncHandle->numSyncMissed);
        DebugP_log(pBuf);
#endif /*PTP_TESTING*/
        sprintf(pBuf, "\n\rUTC Offset (Seconds field) :\t\t%ld seconds",
                (long int)seconds);
        DebugP_log(pBuf);
        sprintf(pBuf, "\n\rMaster connected on Port :\t\t%d",
                (int)timeSyncHandle->tsRunTimeVar->syncPortNum);
        DebugP_log(pBuf);
        DebugP_log("\n\r*******************************");

    }

}

void printConfiguration(ICSS_EMAC_Handle emachandle)
{
    char pMsg[1024];
    RED_CONFIG cfg;

    memcpy(&cfg, 0, sizeof(RED_CONFIG));

    sprintf(pMsg,
            "\n\r"
            "C O N F I G U R A T I O N\n\r"
            "\n\r");
    DebugP_log(pMsg);

    if(RedGetConfiguration(&cfg, prusshandle) == RED_OK)
    {
        sprintf(pMsg,
                "    sup address: %08x%04x\n\r"
                "\n\r"
                "                           Node Table  Duplicate Host Table  Duplicate Port Table\n\r"
                "                 --------------------  --------------------  --------------------\n\r"
                "           size:             %08x              %08x              %08x\n\r"
                "    arbitration:             %08x              %08x\n\r"
                " check interval:             %08x              %08x              %08x\n\r"
                "    forget time:             %08x              %08x\n\r"
                "        counter:             %08x              %08x  %08x    %08x\n\r",
                (unsigned int)OS_NetToHost32(cfg.supAddressHi),
                (unsigned int)OS_NetToHost16((uint16_t)(cfg.supAddressLow & 0x0000FFFF)),
                (unsigned int)cfg.nodeTableSize, (unsigned int)cfg.duplicateHostTableSize,
                (unsigned int)cfg.duplicatePortTableSize,
                (unsigned int)cfg.nodeTableArbitration,
                (unsigned int)cfg.hostDuplicateArbitration,
                (unsigned int)cfg.nodeTableCheckInterval,
                (unsigned int)cfg.duplicateHostCheckInterval,
                (unsigned int)cfg.duplicatePortCheckInterval,
                (unsigned int)cfg.nodeForgetTime, (unsigned int)cfg.duplicateForgetTime,
                (unsigned int)cfg.nodeTableCounter, (unsigned int)cfg.duplicateHostCounter,
                (unsigned int)cfg.duplicatePort0Counter,
                (unsigned int)cfg.duplicatePort1Counter);
        DebugP_log(pMsg);
    }

    else
    {
        sprintf(pMsg, "redGetConfiguration failed!\n\r");
        DebugP_log(pMsg);
    }
}

void printStatistics(ICSS_EMAC_Handle emachandle)
{
    char pMsg[1024];
    RED_STATISTICS stats;

#ifdef ICSS_PROTOCOL_HSR
    char *mode = getHsrLreModePrintStr(emachandle);
    sprintf(pMsg,
            "\n\r"
            "HSR %s\n\r",
            mode);
    DebugP_log(pMsg);
#endif /* ICSS_PROTOCOL_HSR */

    sprintf(pMsg,
            "\n\r"
            "S T A T I S T I C S\n\r"
            "\n\r");
    DebugP_log(pMsg);

    if(RedGetStatistics(&stats, emachandle) != RED_OK)
    {
        sprintf(pMsg, "hsrGetIndexArrayEntry failed!\n\r");
        DebugP_log(pMsg);
        return;
    }

    sprintf(pMsg,
            "      Tx  WrongLan        Rx    Errors     Nodes     Proxy    Unique   Duplic.     Multi     OwnRx\n\r"
            "--------  --------  --------  --------  --------  --------  --------  --------  --------  --------\n\r"
            "%08d  %08d  %08d  %08d  %08d  %08d  %08d  %08d  %08d  %08d\n\r"
            "%08d  %08d  %08d  %08d                      %08d  %08d  %08d  %08d\n\r"
            "%08d  %08d  %08d  %08d                      %08d  %08d  %08d\n\r",
            (int)stats.cntTxA, (int)stats.cntErrWrongLanA, (int)stats.cntRxA,
            (int)stats.cntErrorsA,
            (int)stats.cntNodes, (int)stats.cntProxyNodes,
            (int)stats.cntUniqueA, (int)stats.cntDuplicateA, (int)stats.cntMultiA,
            (int)stats.cntOwnRxA,
            (int)stats.cntTxB, (int)stats.cntErrWrongLanB, (int)stats.cntRxB,
            (int)stats.cntErrorsB,
            (int)stats.cntUniqueB, (int)stats.cntDuplicateB, (int)stats.cntMultiB,
            (int)stats.cntOwnRxB,
            (int)stats.cntTxC, (int)stats.cntErrWrongLanC, (int)stats.cntRxC,
            (int)stats.cntErrorsC,
            (int)stats.cntUniqueC, (int)stats.cntDuplicateC, (int)stats.cntMultiC);
    DebugP_log(pMsg);

#ifdef RED_STATS_DBG
    sprintf(pMsg,
            " H_OFlow   F_OFlow  H_F_Acqu  H_F_Acqu    debug1     debu2    debug3   debug4\n\r"
            "--------  --------  --------  --------  --------  --------  --------  --------\n\r"
            "%08d  %08d  %08d  %08d  %08d  %08d  %08d  %08d\n\r"
            "%08d  %08d  %08d  %08d\n\r",
            stats.RXA_OVERFLOW, stats.RXA_FWD_OVERFLOW, stats.RXA_FAILACQU_QUEUE,
            stats.RXA_FWD_FAILACQU_QUEUE , stats.DEBUG_1, stats.DEBUG_2, stats.DEBUG_3,
            stats.DEBUG_4,
            stats.RXB_OVERFLOW, stats.RXB_FWD_OVERFLOW, stats.RXB_FAILACQU_QUEUE,
            stats.RXB_FWD_FAILACQU_QUEUE);
    DebugP_log(pMsg);
#endif /* RED_STATS_DBG */
}

void printLreStatistics(ICSS_EMAC_Handle emachandle)
{
    uint16_t i;

    char lreStats[][50] = {"LRE_START", "LRE_CNT_TX_A", "LRE_CNT_TX_B", "LRE_CNT_TX_C", "LRE_CNT_ERRWRONGLAN_A",
                           "LRE_CNT_ERRWRONGLAN_B", "LRE_CNT_ERRWRONGLAN_C", "LRE_CNT_RX_A", "LRE_CNT_RX_B",
                           "LRE_CNT_RX_C", "LRE_CNT_ERRORS_A", "LRE_CNT_ERRORS_B", "LRE_CNT_ERRORS_C",
                           "LRE_CNT_NODES", "LRE_CNT_PROXY_NODES", "LRE_CNT_UNIQUE_RX_A", "LRE_CNT_UNIQUE_RX_B",
                           "LRE_CNT_UNIQUE_RX_C", "LRE_CNT_DUPLICATE_RX_A", "LRE_CNT_DUPLICATE_RX_B",
                           "LRE_CNT_DUPLICATE_RX_C", "LRE_CNT_MULTIPLE_RX_A", "LRE_CNT_MULTIPLE_RX_B",
                           "LRE_CNT_MULTIPLE_RX_C", "LRE_CNT_OWN_RX_A", "LRE_CNT_OWN_RX_B", "LRE_DUPLICATE_DISCARD",
                           "LRE_TRANSPARENT_RECEPTION", "LRE_NODE_TABLE_FULL", "LRE_MULTICAST_DROPPED", "LRE_VLAN_DROPPED", "LRE_INTR_TMR_EXP"
                          };

    for(i = 0; i < LRE_STATS_DMEM_SIZE / 4; i++)
    {
        DebugP_log("%s %x\n\r", lreStats[i], *((uint32_t *)(((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase + LRE_START +
                                              i * 4)));
    }
}

char *getNodeTypePrintStr(uint8_t nodeEntryStatus)
{
    uint8_t remNodeType = (nodeEntryStatus & NT_REM_NODE_TYPE_MASK) >>
                          NT_REM_NODE_TYPE_SHIFT;
    uint8_t hsrBitSet   = (nodeEntryStatus & NT_REM_NODE_HSR_BIT);

    switch(remNodeType)
    {
        case NT_REM_NODE_TYPE_SANA:
            return RED_REM_NODE_TYPE_STR_SANA;

        case NT_REM_NODE_TYPE_SANB:
            return RED_REM_NODE_TYPE_STR_SANB;

        case NT_REM_NODE_TYPE_SANAB:
            return RED_REM_NODE_TYPE_STR_SANAB;

        case NT_REM_NODE_TYPE_DAN:
            if(hsrBitSet)
            {
                return RED_REM_NODE_TYPE_STR_DANH;
            }

            else
            {
                return RED_REM_NODE_TYPE_STR_DANP;
            }

        case NT_REM_NODE_TYPE_REDBOX:
            if(hsrBitSet)
            {
                return RED_REM_NODE_TYPE_STR_REDBOXH;
            }

            else
            {
                return RED_REM_NODE_TYPE_STR_REDBOXP;
            }

        case NT_REM_NODE_TYPE_VDAN:
            if(hsrBitSet)
            {
                return RED_REM_NODE_TYPE_STR_VDANH;
            }

            else
            {
                return RED_REM_NODE_TYPE_STR_VDANP;
            }

        default:
            break;
    }

    return RED_REM_NODE_TYPE_STR_UNKNOWN;
}

uint8_t getDuplicateDiscardPrintVal(uint8_t nodeEntryStatus)
{
    return ((nodeEntryStatus & NT_REM_NODE_DUP_MASK) >> NT_REM_NODE_DUP_SHIFT);
}

void printSnmpNodeTable(hsrPrpHandle *hsrPrphandle)
{
    Int32 index = 0;
    Int32 nodes = 0;
    char *nodeType = NULL;
    uint8_t dupDiscard = 1;
    char pMsg[1024];
    RED_NODE_TABLE nodeTable;
    RED_NODE_TABLE_ENTRY *entry;

    sprintf(pMsg,
            "\n\r"
            "N O D E   T A B L E (SNMP API)\n\r"
            "\n\r");
    DebugP_log(pMsg);

    nodeTable.max = getLreNodeTableSize(prusshandle);

    nodeTable.cnt = 0;
    nodeTable.entries = malloc(sizeof(RED_NODE_TABLE_ENTRY) * nodeTable.max);

    if(nodeTable.entries == NULL)
    {
        sprintf(pMsg, "malloc failed!\n\r");
        DebugP_log(pMsg);
        return;
    }

    nodes = getLreNodeTable(&nodeTable, hsrPrphandle);

    if(nodes <= 0)
    {
        if(nodes == 0)
        {
            sprintf(pMsg, "Node Table is empty!\n\r");
            DebugP_log(pMsg);
        }

        else
        {
            sprintf(pMsg, "getLreNodeTable failed! Err: %d\n\r", (int)nodes);
            DebugP_log(pMsg);
        }
    }

    else
    {
        sprintf(pMsg,
                "Idx         MacAddress     Type  DD        Rx   LineErr     RxSup  TlsSup     Tls\n\r"
                "---  -----------------  -------  --  --------  --------  --------  ------  ------\n\r");
        DebugP_log(pMsg);

        for(index = 0; index < nodes; index++)
        {
            entry = &nodeTable.entries[index];
            nodeType = getNodeTypePrintStr(entry->status);
            dupDiscard = getDuplicateDiscardPrintVal(entry->status);
            sprintf(pMsg,
                    "%03d  %02x-%02x-%02x-%02x-%02x-%02x  %s  %02d  %08d  %08d  %08d  %06d  %06d\n\r"
                    "                                     %08d  %08d  %08d          %06d\n\r",
                    (int)index, entry->src[0], entry->src[1], entry->src[2], entry->src[3],
                    entry->src[4], entry->src[5],
                    nodeType, (int)dupDiscard, (int)entry->cntRxA, (int)entry->errRxA,
                    (int)entry->cntRxSupA,
                    (int)entry->timeLasSeenS, (int)entry->timeLasSeenA,
                    (int)entry->cntRxB, (int)entry->errRxB, (int)entry->cntRxSupB,
                    (int)entry->timeLasSeenB);
            DebugP_log(pMsg);
        }
    }

    free(nodeTable.entries);
}

void printNodeTable(hsrPrpHandle *hsrPrphandle)
{
    RED_BIN_ARRAY_ENTRY *binArrayBase = hsrPrphandle->binArrayBase;
    RED_BIN_ARRAY_ENTRY *pbinArrayNode;

    RED_NODE_TABLE_ENTRY *nodeTableBase = hsrPrphandle->nodeTableBase;
    RED_NODE_TABLE_ENTRY *pnodeTableNode;

    uint16_t i, j;

#if DEBUG_HSR
    RED_INDEX_ARRAY_ENTRY *indexArrayBase = (RED_INDEX_ARRAY_ENTRY *)((((
            ICSS_EMAC_HwAttrs *)
                                            ICSS_EMAC_Handle->hwAttrs)->emacBaseAddrCfg)->sharedDataRamBaseAddr
                                            + INDEX_ARRAY_NT);

    for(i = 0; i < INDEX_TABLE_MAX_ENTRIES; i++)
        if((indexArrayBase + i)->binNoEntries >= 1)
        {
            DebugP_log("%d %d\n", i, (indexArrayBase + i)->binNoEntries);
        }

#endif

    char pMsg[1024];
    sprintf(pMsg,
            "\n\r"
            "N O D E   T A B L E   N E W\n\r"
            "\n\r"
            "Idx         MacAddress     Type  DD        Rx   LineErr     RxSup  TlsSup     Tls\n\r"
            "---  -----------------  -------  --  --------  --------  --------  ------  ------\n\r");
    DebugP_log(pMsg);

    for(i = 0, j = 0; i < BIN_ARRAY_MAX_ENTRIES; i++)
    {
        pbinArrayNode = binArrayBase + i;

        if(pbinArrayNode->nodetable_offset != CONST_NODETABLE_INVALID)
        {
            pnodeTableNode = nodeTableBase + pbinArrayNode->nodetable_offset;

            char *nodeType = NULL;
            uint8_t dupDiscard = 1;
            nodeType = getNodeTypePrintStr(pnodeTableNode->status);
            dupDiscard = getDuplicateDiscardPrintVal(pnodeTableNode->status);
            sprintf(pMsg,
                    "%03d  %02x-%02x-%02x-%02x-%02x-%02x  %s  %02d  %08d  %08d  %08d  %06d  %06d\n\r"
                    "                                     %08d  %08d  %08d          %06d\n\r",
                    ++j,
                    pbinArrayNode->MacId[3],
                    pbinArrayNode->MacId[2],
                    pbinArrayNode->MacId[1],
                    pbinArrayNode->MacId[0],
                    pbinArrayNode->MacId[5],
                    pbinArrayNode->MacId[4],
                    nodeType,
                    (int)dupDiscard,
                    (int)pnodeTableNode->cntRxA,
                    (int)pnodeTableNode->errRxA,
                    (int)pnodeTableNode->cntRxSupA,
                    (int)pnodeTableNode->timeLasSeenS,
                    (int)pnodeTableNode->timeLasSeenA,
                    (int)pnodeTableNode->cntRxB,
                    (int)pnodeTableNode->errRxB,
                    (int)pnodeTableNode->cntRxSupB,
                    (int)pnodeTableNode->timeLasSeenB);
            DebugP_log(pMsg);
        }
    }
}

void runRxTxTest(ICSS_EMAC_Handle emachandle)
{

    char byte;
    uint8_t originalPrioQueue;
    DebugP_log("\n\rTo configure device as Transmitter enter T or t. To configure as receiver enter R or r : \t");
    DebugP_scanf("%c", &byte);
    originalPrioQueue = ((ICSS_EMAC_Attrs *)emachandle->attrs)->ethPrioQueue;

    if((byte == 'R') || (byte == 'r'))
    {
        DebugP_log("\n\rDevice configured as receiver, will wait for 3 minutes to receive packets");
        ((ICSS_EMAC_Attrs *)emachandle->attrs)->ethPrioQueue = ICSS_EMAC_QUEUE3;
        /*Wait 3 minutes*/
        ClockP_usleep(ClockP_ticksToUsec(3 * 60000));
        ((ICSS_EMAC_Attrs *)emachandle->attrs)->ethPrioQueue = originalPrioQueue;

    }

    else if((byte == 'T') || (byte == 't'))
    {
        DebugP_log("\n\rDevice configured as transmitter, sending packets now");
        txHSRPRPTestFrame(emachandle);
        DebugP_log("\n\rDevice configured as transmitter...completed");
    }

    else
    {
        DebugP_log("\n\rDidn't catch that, going back to main menu");
    }

}

void txHSRPRPTestFrame(ICSS_EMAC_Handle emachandle)
{
    /*sequence id*/
    uint8_t seq_num = 1;
    uint16_t tx_count;
    uint32_t status = 0;
    ICSS_EMAC_TxArgument txArg;

    /*Add sequence id to bytes stream*/
    hsrPrpTestFrame[START_OF_UDP_HEADER + 2] = seq_num;

    tx_count = 0;
    char byte;

    DebugP_log("\n\rTo send normal HSRPRP packet : Enter N; To send GOOSE packet : Enter G; To send VLAN+GOOSE packet : Enter V\t");
    DebugP_scanf("%c", &byte);

    if((byte == 'N') || (byte == 'n'))
    {
        while(1)
        {
            if(tx_count < NUM_HSR_TEST_FRAMES)
            {
                txArg.icssEmacHandle = emachandle;
                txArg.srcAddress = hsrPrpTestFrame;
                txArg.lengthOfPacket = (int32_t)HSR_PRP_TEST_FRAME_SIZE;
                txArg.queuePriority = ICSS_EMAC_QUEUE1;
                /*Port number is irrelevant in HSR/PRP since packet is duplicated*/
                txArg.portNumber = 0;

                status = ICSS_EMAC_txPacket(&txArg, NULL);

                if(status != 0)
                {
                    DebugP_log("\n\rFailed to transmit for tx_count = %u", tx_count);
                }
                else
                {
                    DebugP_log("\n\rDevice transmitted hsrPrpTestFrame packet for tx_count = %u", tx_count);
                }

                seq_num = (1 << (tx_count % 5)) - 1; /*TO DO: 5 Hard coded to use 4 leds*/
                hsrPrpTestFrame[START_OF_UDP_HEADER + 2] = seq_num;
                tx_count++;

                ClockP_usleep(ClockP_ticksToUsec(500));
            }

            if(tx_count >= NUM_HSR_TEST_FRAMES)
            {
                DebugP_log("\n\rDevice transmitted hsrPrpTestFrame packets...completed");
                break;
            }

        }
    }
    else if((byte == 'G') || (byte == 'g'))
    {
        while(1)
        {
            if(tx_count < NUM_HSR_TEST_FRAMES)
            {
                txArg.icssEmacHandle = emachandle;
                txArg.srcAddress = GooseTestFrame;
                txArg.lengthOfPacket = (int32_t)HSR_PRP_TEST_FRAME_SIZE;
                txArg.queuePriority = ICSS_EMAC_QUEUE1;
                /*Port number is irrelevant in HSR/PRP since packet is duplicated*/
                txArg.portNumber = 0;

                status = ICSS_EMAC_txPacket(&txArg, NULL);

                if(status != 0)
                {
                    DebugP_log("\n\rFailed to transmit GooseTestFrame for tx_count = %u", tx_count);
                }
                else
                {
                    DebugP_log("\n\rDevice transmitted GooseTestFrame packet for tx_count = %u", tx_count);
                }


                //seq_num = (1 << (tx_count % 9)) - 1;
                //hsrPrpTestFrame[START_OF_UDP_HEADER + 2] = seq_num;
                tx_count++;

                ClockP_usleep(ClockP_ticksToUsec(500));
            }

            if(tx_count >= NUM_HSR_TEST_FRAMES)
            {
                DebugP_log("\n\rDevice transmitted GooseTestFrame packets...completed");
                break;
            }

        }
    }

    else if((byte == 'V') || (byte == 'v'))
    {
        while(1)
        {
            if(tx_count < NUM_HSR_TEST_FRAMES)
            {
                txArg.icssEmacHandle = emachandle;
                txArg.srcAddress = vlanGooseTestFrame;
                txArg.lengthOfPacket = (int32_t)HSR_PRP_TEST_FRAME_SIZE;
                txArg.queuePriority = ICSS_EMAC_QUEUE1;
                /*Port number is irrelevant in HSR/PRP since packet is duplicated*/
                txArg.portNumber = 0;

                status = ICSS_EMAC_txPacket(&txArg, NULL);

                if(status != 0)
                {
                    DebugP_log("\n\rFailed to transmit vlanGooseTestFrame for tx_count = %u", tx_count);
                }
                else
                {
                    DebugP_log("\n\rDevice transmitted vlanGooseTestFrame packet for tx_count = %u", tx_count);
                }
                //seq_num = (1 << (tx_count % 9)) - 1;
                //hsrPrpTestFrame[START_OF_UDP_HEADER + 2] = seq_num;
                tx_count++;

                ClockP_usleep(ClockP_ticksToUsec(500));
            }

            if(tx_count >= NUM_HSR_TEST_FRAMES)
            {
                DebugP_log("\n\rDevice transmitted vlanGooseTestFrame packets...completed");
                break;
            }
        }

    }
}

void parseAndCheckHSRPRPTestFrame(uint8_t *tempFrame)
{

    /*check for marker and toggle LED's*/
    if((tempFrame[START_OF_UDP_HEADER] == 0xAB)
            && (tempFrame[START_OF_UDP_HEADER + 1] == 0xAC))
    {
        /*Set DGIO status as per data received from transmitter*/
        //Board_setDigOutput(tempFrame[START_OF_UDP_HEADER + 2]);
        LED_setMask(gLedHandle[CONFIG_LED2], (uint32_t)(0xFFFF & tempFrame[START_OF_UDP_HEADER + 2]));
    }

}

uint8_t readAndAssignIPAddress()
{
    uint32_t ipAddr = 0;
    uint8_t returnVal = FALSE;
    char bkupIPString[20];

    memset(bkupIPString, 0x0, sizeof(bkupIPString));

    while(1)
    {
        DebugP_readLine(&bkupIPString[0], 20);

        if(isValidIP(bkupIPString))
        {
            /*Found IP address*/
            returnVal = TRUE;
            break;
        }

        else
        {
            DebugP_log("\n\rIllegal value entered");
            returnVal =  FALSE;
            break;
        }
    }

    if(FALSE == returnVal)
    {
        return returnVal;
    }

    memcpy(IPString, bkupIPString, 20);
    ip4addr_aton(IPString, (ip4_addr_t *)&ipAddr); /*To change IP address to stack format*/

    if(addIPAddress(ipAddr))
    {
        DebugP_log("\n\rAssigned IP \t: ");
        DebugP_log(IPString);
    }

    memcpy((hsrPrpTestFrame + START_OF_IP_ADDRESS), &ipAddr, 4);

    changeIPEndianness(&ipAddr);

    EEPROM_write(gEepromHandle[CONFIG_EEPROM0], SPI_EEPROM_DEVICEIP_OFFSET, (uint8_t *)&ipAddr, 4);

    return returnVal;

}

void removeIPAddress()
{
    //check for this function

}

uint8_t addIPAddress(uint32_t dwIPAddress)
{
    sys_lock_tcpip_core();
    netif_set_ipaddr(Lwipif_handle->netif, (ip4_addr_t *)&dwIPAddress);
    sys_unlock_tcpip_core();
    return 0;
}

int isValidIP(const char *ip_str)
{
    unsigned int n1, n2, n3, n4;

    if(sscanf(ip_str, "%u.%u.%u.%u", &n1, &n2, &n3, &n4) != 4)
    {
        return FALSE;
    }

    if((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255)
            && (n4 <= 255))
    {
        char buf[64];
        sprintf(buf, "%u.%u.%u.%u", n1, n2, n3, n4);

        if(strcmp(buf, ip_str))
        {
            return 0;
        }

        return TRUE;
    }

    return FALSE;
}

#define BUF_SIZE 100
int isValidMacid(char *macid_str)
{
    unsigned int n1, n2, n3, n4, n5, n6;

    if(sscanf(macid_str, "%x %x %x %x %x %x", &n1, &n2, &n3, &n4, &n5, &n6) != 6)
    {
        return FALSE;
    }

    if((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255)
            && (n4 <= 255) && (n5 <= 255) && (n6 <= 255))
    {
#if DEBUG_MC_FLT
        DebugP_log("buf:%s macid_str:%s",
                    buf, macid_str);
#endif
        return TRUE;
    }

    return FALSE;
}

void changeIPEndianness(uint32_t *dwIPAddress)
{
    uint32_t dwTempIP;

    dwTempIP = * dwIPAddress << 24;
    dwTempIP |= (* dwIPAddress & 0xFF00) << 8;
    dwTempIP |= (* dwIPAddress & 0xFF0000) >> 8;
    dwTempIP |= (* dwIPAddress & 0xFF000000) >> 24;
    * dwIPAddress = dwTempIP;
}

#ifdef PTP_TESTING
void hsrprp_monitorPTPStatusTask(void *args)
{
    TimeSync_ParamsHandle_t timeSyncHandle = NULL;
    timeSyncHandle = (TimeSync_ParamsHandle_t)args;

    while(1)
    {
        if(timeSyncHandle->tsRunTimeVar->stateMachine &
                TS_STATE_MACHINE_DEVICE_IN_SYNC)
        {
            if(timeSyncHandle->tsRunTimeVar->currOffset < min_offset)
            {
                min_offset = timeSyncHandle->tsRunTimeVar->currOffset;
            }

            if(timeSyncHandle->tsRunTimeVar->currOffset > max_offset)
            {
                max_offset = timeSyncHandle->tsRunTimeVar->currOffset;
            }
        }

        ClockP_usleep(ClockP_ticksToUsec(200));
    }
}
#endif /*PTP_TESTING*/

void intr_pac_config(ICSS_EMAC_Handle emachandle, uint32_t intrPacStatus)
{
    uint8_t *intrPacing = ((uint8_t *)(((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase) +
                                      INTR_PAC_STATUS_OFFSET);

    uint32_t *timeOutValuePru0 = ((uint32_t *)(((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase) +
                                 INTR_PAC_TMR_EXP_OFFSET_PRU0);
    uint32_t *timeOutValuePru1 = ((uint32_t *)(((PRUICSS_HwAttrs const *)(prusshandle->hwAttrs))->sharedDramBase) +
                                 INTR_PAC_TMR_EXP_OFFSET_PRU1);

    *intrPacing = intrPacStatus;

    if(intrPacStatus == INTR_PAC_DIS_ADP_LGC_DIS)
    {
        DebugP_log("Pacing mode : Interrupt pacing disabled\n");
    }

    else if(intrPacStatus == INTR_PAC_ENA_ADP_LGC_DIS)
    {
        DebugP_log("Pacing mode : Interrupt pacing enabled with adaptive logic disabled\n");

        DebugP_log("Enter the timerExpiration value:");
        char bkupMacidString[30];
        DebugP_readLine(&bkupMacidString[0], 30);

        unsigned int timerExpiration;
        sscanf(bkupMacidString, "%u", &timerExpiration);

        *timeOutValuePru0 = timerExpiration;
        *timeOutValuePru1 = timerExpiration;
    }

    else if(intrPacStatus == INTR_PAC_ENA_ADP_LGC_ENA)
    {
        DebugP_log("Pacing mode : Interrupt pacing enabled with adaptive logic enabled\n");
    }
}

void addMACID(uint8_t *src, uint8_t *macID)
{
    *(src) = *(macID);
    *(src + 1) = *(macID + 1);
    *(src + 2) = *(macID + 2);
    *(src + 3) = *(macID + 3);
    *(src + 4) = *(macID + 4);
    *(src + 5) = *(macID + 5);
}
