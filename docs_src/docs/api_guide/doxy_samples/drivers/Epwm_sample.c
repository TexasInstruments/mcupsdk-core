
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_soc.h>
#include <stdint.h>
//! [include]
#include <drivers/epwm.h>
//! [include]

uint32_t ePWMBaseAddr = CSL_EPWM0_EPWM_BASE;

void checkExternalSync(void)
{
//! [check_ext_sync]
    uint16_t tbStatus = EPWM_tbGetStatus(ePWMBaseAddr, EPWM_TB_STS_SYNCI);

    if(0 == tbStatus)
    {
        DebugP_log("External Synchronization event has occured\r\n");
    }

    else
    {
        DebugP_log("No external Synchronization event has occured\r\n");
    }
//! [check_ext_sync]
}

void getTbCounterDirection(void)
{
//! [get_timebase_direction]
    uint16_t tbStatus = EPWM_tbGetStatus(ePWMBaseAddr, EPWM_TB_STS_CTR_DIR);

    if(0 == tbStatus)
    {
        DebugP_log("Time base counter is currently counting down\r\n");
    }

    else
    {
        DebugP_log("Time base counter is currently counting up\r\n");
    }
//! [get_timebase_direction]
}

void CountercompareConfig(void)
{
//! [config_counter_compare]
    uint32_t counterCmpVal = 0x100;
    uint32_t status;

    status =EPWM_counterComparatorCfg(ePWMBaseAddr, EPWM_CC_CMP_A, counterCmpVal, EPWM_SHADOW_REG_CTRL_ENABLE, EPWM_CC_CMP_LOAD_MODE_CNT_EQ_PRD, FALSE);

    if(TRUE == status)
    {
        DebugP_log("Comparator value was written successfully\r\n");
    }

    else
    {
        DebugP_log("Comparator value write failed\r\n");
    }
//! [config_counter_compare]
}
