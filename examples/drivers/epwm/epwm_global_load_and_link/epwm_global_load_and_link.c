
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

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <linker_defines.h>
/*
 * Global Load Feature :
 *      Many Applications will have EPWMs with runtime updates to their 
 * CMP/TBPRD/Action Qualifier/ DeadBand Registers etc, that affect 
 * the EPWM Outputs. Care needs to be taken while updating these registers 
 * so that there wouldn't be any incorrect/unexpected/corrupted waveforms 
 * while these updates happen. To tackle with such issues, the EPWMs have 
 * been equipped with a Global Load feature. 
 *      Along with the Shadow to Active load strobe for individual register sets, 
 * there is optional Global load strobe, if enabled, overwrites the local load 
 * strobe. The global load strobe can be used for activating all shadow 
 * to active transfers, this will result in retaining the older EPWM outputs 
 * untill all the shadow registers are updated, resulting in no currupted 
 * waveforms in between. 
 * 
 * Link Feature :
 *      Link Feature helps link the EPWMs to have simultaneous updates to 
 * the link enabled registers like TBPRD/CMP(A/B/C/D)/DBFED/DBRED/GLDCTL2, when
 * these registers in the linked EPWMs are written to.
 * 
 * Example Description :
 *      This Example aims to showcase the Global Load and Link features usage.
 * 4 EPWMs, 
 *  - CONFIG_EPWM_NO_GLD            : no Global Load is configured 
 *  - CONFIG_EPWM1_GLD              : Global Load is configured.
 *  - CONFIG_EPWM2_GLD_LINK_SELF    : Global Load is configured. 
 *                                    CMP(A/B)/ GLD Controls Linked to self 
 *  - CONFIG_EPWM3_GLD_LINK_EPWM2   : Global Load is configured. 
 *                                    CMP(A/B)/ GLD Controls Linked to 
 *                                    CONFIG_EPWM2_GLD_LINK_SELF.
 * 
 * Apart from the Global load or Link configurations, all EPWMs are configured 
 * similarly. Check the Syscfg for the Global Load Configurations and differences.
 * 
 * Note : 
 *      Shadow registers are used for CMP, TBPRD regs, while global load is only 
 * used on the CMP regs and Link is used for CMP and GLD controls. 
 * 
 * External Connections.
 *      Probe the EPWM outputs for the above EPWMs and Status pin.
 * 
 *  AM263Px or AM263x Control Card with HSEC 180 Pin out
 *   CONFIG_EPWM_NO_GLD          : EPWM 0A/0B on HSEC pins 49/51 
 *   CONFIG_EPWM1_GLD            : EPWM 1A/1B on HSEC pins 53/55 
 *   CONFIG_EPWM2_GLD_LINK_SELF  : EPWM 2A/2B on HSEC pins 57/59 
 *   CONFIG_EPWM3_GLD_LINK_EPWM2 : EPWM 3A/3B on HSEC pins 61/63
 *   Status Pin                  : GPIO on HSEC pin 50 
 *  
 *  AM263Px or AM263x Launch Pad
 *   CONFIG_EPWM_NO_GLD          : EPWM 0A/0B on BoosterPack J4 11 / J8 59 
 *   CONFIG_EPWM1_GLD            : EPWM 1A/1B on BoosterPack J2 37 / J2 38 
 *   CONFIG_EPWM2_GLD_LINK_SELF  : EPWM 2A/2B on BoosterPack J2 39 / J2 40 
 *   CONFIG_EPWM3_GLD_LINK_EPWM2 : EPWM 3A/3B on BoosterPack J8 77 / J8 78
 *   Status Pin                  : GPIO on                   J8 52
 * 
 *  AM261x Launch Pad
 *   CONFIG_EPWM_NO_GLD          : EPWM 2A/2B on BoosterPack J2 40 / J2 39 
 *   CONFIG_EPWM1_GLD            : EPWM 3A/3B on BoosterPack J2 38 / J2 37 
 *   CONFIG_EPWM2_GLD_LINK_SELF  : EPWM 4A/4B on BoosterPack J2 36 / J2 35 
 *   CONFIG_EPWM3_GLD_LINK_EPWM2 : EPWM 5A/5B on BoosterPack J6 80 / J6 79
 *   Status Pin                  : GPIO on HSEC pin 50       J6 78
 */

#define APP_CMPA_MIN (1000U)
#define APP_CMPA_MAX (10000U)

#define APP_CMPB_MIN (10000U)
#define APP_CMPB_MAX (19000U)

#define APP_CMPA_STEP (1500U)
#define APP_CMPB_STEP (1700U)

uint16_t gApp_cmpa, gApp_cmpb;


void epwm_global_load_and_link_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Check the syscfg for configurations */
    
    DebugP_log("EPWM Global Load and Link Test Started ...\r\n");
    
    /* We will update the Compare Values every 75 us  */
    gApp_cmpa = APP_CMPA_MAX;
    gApp_cmpb = APP_CMPB_MAX;

    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncDisableMask, TRUE);

    while(gApp_cmpb > APP_CMPB_MIN){
        // DebugP_log("CMPA : %d\tCMPB : %d\r\n",gApp_cmpa, gApp_cmpb);

        EPWM_setCounterCompareValue(CONFIG_EPWM_NO_GLD_BASE_ADDR, EPWM_COUNTER_COMPARE_A, gApp_cmpa);
        EPWM_setCounterCompareValue(CONFIG_EPWM1_GLD_BASE_ADDR, EPWM_COUNTER_COMPARE_A, gApp_cmpa);
        EPWM_setCounterCompareValue(CONFIG_EPWM2_GLD_LINK_SELF_BASE_ADDR, EPWM_COUNTER_COMPARE_A, gApp_cmpa);
        /* add code here..!*/
        ClockP_usleep(150);
        EPWM_setCounterCompareValue(CONFIG_EPWM_NO_GLD_BASE_ADDR, EPWM_COUNTER_COMPARE_B, gApp_cmpb);
        EPWM_setCounterCompareValue(CONFIG_EPWM1_GLD_BASE_ADDR, EPWM_COUNTER_COMPARE_B, gApp_cmpb);
        EPWM_setCounterCompareValue(CONFIG_EPWM2_GLD_LINK_SELF_BASE_ADDR, EPWM_COUNTER_COMPARE_B, gApp_cmpb);

        /* the following are not required as the cmp regs are linked as well! */
        // EPWM_setCounterCompareValue(CONFIG_EPWM3_GLD_LINK_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_A, gApp_cmpa);
        // EPWM_setCounterCompareValue(CONFIG_EPWM3_GLD_LINK_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_B, gApp_cmpb);
        GPIO_pinWriteHigh(STATUS_BASE_ADDR, STATUS_PIN);
	    
        EPWM_setGlobalLoadOneShotLatch(CONFIG_EPWM1_GLD_BASE_ADDR);

        /* The following may be used to force the Global Load Strobe after Latch is set
        In the example, we are using an Event to pass through as global load strobe after the latch is set */
	    // EPWM_forceGlobalLoadOneShotEvent(CONFIG_EPWM1_GLD_BASE_ADDR);
	    EPWM_setGlobalLoadOneShotLatch(CONFIG_EPWM2_GLD_LINK_SELF_BASE_ADDR);
        
        /* The following may be used to force the Global Load Strobe after Latch is set 
        In the example, we are using an Event to pass through as global load strobe after the latch is set */
	    // EPWM_forceGlobalLoadOneShotEvent(CONFIG_EPWM2_GLD_LINK_SELF_BASE_ADDR);
        
        GPIO_pinWriteLow(STATUS_BASE_ADDR, STATUS_PIN);
        /* the following is not required as the Global Control 2 reg is linked as well! */
        // EPWM_setGlobalLoadOneShotLatch(CONFIG_EPWM3_GLD_LINK_EPWM2_BASE_ADDR);
        
        ClockP_usleep(1500);
        gApp_cmpa -= APP_CMPA_STEP;
        gApp_cmpb -= APP_CMPB_STEP;
    }
    
    SOC_setMultipleEpwmTbClk(gEpwmTbClkSyncDisableMask, FALSE);

    DebugP_log("EPWM Global Load and Link gTest Passed!!\r\n");
    DebugP_log("All Tests have Passed!!");

    Board_driversClose();
    Drivers_close();
}
