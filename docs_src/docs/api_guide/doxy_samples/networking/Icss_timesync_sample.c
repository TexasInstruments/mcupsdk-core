#include <stddef.h>
#include <string.h>

//! [icss_timesync_include]
#include <networking/icss_timesync/icss_timeSync_init.h>
//! [icss_timesync_include]

PRUICSS_Handle      pruicssHandle;
ICSS_EMAC_Handle    emacHandle;

void icss_timesync_open(void)
{
//! [icss_timesync_open]
    int8_t returnVal = TIME_SYNC_OK;
    TimeSync_ParamsHandle_t timeSyncHandle;

    /* Allocate memory for the handle */
    timeSyncHandle = (TimeSync_ParamsHandle_t)malloc(sizeof(TimeSync_ParamsHandle));
    DebugP_assert(timeSyncHandle != NULL);

    /*Configure PTP. These variables must be configured before doing anything else*/
    timeSyncHandle->emacHandle = emacHandle;
    timeSyncHandle->pruicssHandle = pruicssHandle;

    timeSyncHandle->timeSyncConfig.config      = BOTH;
    timeSyncHandle->timeSyncConfig.type        = E2E;
    timeSyncHandle->timeSyncConfig.protocol    = UDP_IPV4;
    timeSyncHandle->timeSyncConfig.tickPeriod  = 500;
    timeSyncHandle->txprotocol                 = 0;
    /* ... */
    /* ... */

    /* Allocate memory for some structures inside handle */
    timeSyncHandle->tsRunTimeVar = (timeSync_RuntimeVar_t *)malloc(sizeof(timeSync_RuntimeVar_t));
    DebugP_assert(timeSyncHandle->tsRunTimeVar != NULL);
    /* ... */
    /* ... */

    /* Allocate Rx and Tx packet buffers */
    returnVal = TimeSync_alloc_PktBuffer(timeSyncHandle);
    DebugP_assert(returnVal == TIME_SYNC_OK);

    /* Call the initialization API */
    returnVal = TimeSync_drvInit(timeSyncHandle);
    DebugP_assert(returnVal == TIME_SYNC_OK);
//! [icss_timesync_open]
}