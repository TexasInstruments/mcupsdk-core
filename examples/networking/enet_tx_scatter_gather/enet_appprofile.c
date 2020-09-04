#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>


#include "FreeRTOSConfig.h"


#include "csl_arm_r5_pmu.h"

#include "enet_appprofile.h"

#define PMU_EVENT_COUNTER_1 (CSL_ARM_R5_PMU_EVENT_TYPE_CYCLE_CNT)
#define PMU_EVENT_COUNTER_2 (CSL_ARM_R5_PMU_EVENT_TYPE_I_X)
#define PMU_EVENT_COUNTER_3 (CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_STALL)

#define APP_MAX_PROFILE_FXNS                 (15)
/* Task stack size */
#define ENETAPP_PROFILE_TASK_STACK_SZ                     (10U * 1024U)


typedef struct fxnProfile_s
{
    uint32_t startTime[3];
    uint32_t deltaTotalTime[3];
    uint32_t count;
    uint32_t key;
}fxnProfile;

typedef struct appProfileFxnLoad_s {
    uint32_t deltaTime[3];
    uint32_t count;
    uint32_t profileIndex;
    uint32_t numWindows;
} appProfileFxnLoad;

typedef struct appProfileFxnLoadInfo_s {
    uint32_t numFunctions;
    appProfileFxnLoad fxn[APP_MAX_PROFILE_FXNS];
} appProfileFxnLoadInfo;

static uint8_t gCpuLoadTaskStack[ENETAPP_PROFILE_TASK_STACK_SZ] __attribute__ ((aligned(32)));

fxnProfile FXNPROFILE[APP_MAX_PROFILE_FXNS];

#define UTILS_ARRAYSIZE(x) sizeof(x)/sizeof (x[0U])
#define UTILS_ALIGN(x,align)  ((((x) + ((align) - 1))/(align)) * (align))


static void config_pmu(void)
{

    CSL_armR5PmuCfg(0, 0 ,1);
    CSL_armR5PmuEnableAllCntrs(1);
    int num_cnt = CSL_armR5PmuGetNumCntrs();

    DebugP_assert(num_cnt == 3);
    CSL_armR5PmuCfgCntr(0, PMU_EVENT_COUNTER_1);
    CSL_armR5PmuCfgCntr(1, PMU_EVENT_COUNTER_2);
    CSL_armR5PmuCfgCntr(2, PMU_EVENT_COUNTER_3);

    CSL_armR5PmuEnableCntrOverflowIntr(0, 0);
    CSL_armR5PmuEnableCntrOverflowIntr(1, 0);
    CSL_armR5PmuEnableCntrOverflowIntr(2, 0);
    CSL_armR5PmuResetCntrs();
    CSL_armR5PmuEnableCntr(0, 1);
    CSL_armR5PmuEnableCntr(1, 1);
    CSL_armR5PmuEnableCntr(2, 1);
}

static void loadPrintFxnProfile(appProfileFxnLoadInfo *avgFxnLoad)
{
    uint32_t i;

    for (i = 0; i < APP_MAX_PROFILE_FXNS; i++)
    {
        if (avgFxnLoad->fxn[i].count)
        {
            EnetAppUtils_print(":%u:,%u,%u,%u,%u \r\n",
                               avgFxnLoad->fxn[i].profileIndex, avgFxnLoad->fxn[i].count,avgFxnLoad->fxn[i].deltaTime[0],avgFxnLoad->fxn[i].deltaTime[1],avgFxnLoad->fxn[i].deltaTime[2]);
        }
    }
}

static void app_fxnLoadReset(void)
{

    //memset (&apploadEntryTbl.record[0], 0, sizeof (apploadEntryTbl.record));
    memset((void *)&FXNPROFILE[0],0,sizeof(FXNPROFILE));
}

void app_profileAccumulateFxnLoad(struct appProfileFxnLoad_s *avgFxnLoad, fxnProfile *fxnProfile, uint32_t *numActiveFxns)
{
    uint32_t i, j;

    *numActiveFxns = 0;
    for (i = 0; i < APP_MAX_PROFILE_FXNS; i++)
    {
        if (fxnProfile[i].count)
        {
            avgFxnLoad[*numActiveFxns].count = fxnProfile[i].count;
            avgFxnLoad[*numActiveFxns].profileIndex = i;
            for (j = 0; j < UTILS_ARRAYSIZE(avgFxnLoad->deltaTime); j++)
            {
                avgFxnLoad[*numActiveFxns].deltaTime[j] = fxnProfile[i].deltaTotalTime[j] / fxnProfile[i].count;
            }
            *numActiveFxns = *numActiveFxns + 1;
        }
    }
}

void app_profileGetAvgFxnLoad(appProfileFxnLoadInfo *avgFxnLoad, appProfileFxnLoadInfo *accumulatedFxnLoad , uint32_t numWindows)
{
    uint32_t i, j, k;

    memset(avgFxnLoad, 0, sizeof(*avgFxnLoad));
    for (i = 0; i < numWindows; i++)
    {
        if(avgFxnLoad->numFunctions < accumulatedFxnLoad[i].numFunctions)
        {
            avgFxnLoad->numFunctions = accumulatedFxnLoad[i].numFunctions;
        }
        for (j = 0; j < APP_MAX_PROFILE_FXNS; j++)
        {
            if (accumulatedFxnLoad[i].fxn[j].count)
            {
                avgFxnLoad->fxn[accumulatedFxnLoad[i].fxn[j].profileIndex].profileIndex = accumulatedFxnLoad[i].fxn[j].profileIndex;
                avgFxnLoad->fxn[accumulatedFxnLoad[i].fxn[j].profileIndex].count += accumulatedFxnLoad[i].fxn[j].count;
                avgFxnLoad->fxn[accumulatedFxnLoad[i].fxn[j].profileIndex].numWindows++;
                for (k = 0; k < UTILS_ARRAYSIZE(avgFxnLoad->fxn[0].deltaTime); k++)
                {
                    avgFxnLoad->fxn[accumulatedFxnLoad[i].fxn[j].profileIndex].deltaTime[k] += accumulatedFxnLoad[i].fxn[j].deltaTime[k];
                }
            }
        }
    }
    for (j = 0; j < APP_MAX_PROFILE_FXNS; j++)
    {
        if ((avgFxnLoad->fxn[j].numWindows) && (avgFxnLoad->fxn[j].count))
        {
            avgFxnLoad->fxn[j].count /= avgFxnLoad->fxn[j].numWindows;
            for (k = 0; k < UTILS_ARRAYSIZE(avgFxnLoad[0].fxn[0].deltaTime); k++)
            {
                avgFxnLoad->fxn[j].deltaTime[k] /= avgFxnLoad->fxn[j].numWindows;
            }
        }
    }
}


void app_fxnEntry(uint32_t profileIndex)
{
    FXNPROFILE[profileIndex].key = EnetOsal_disableAllIntr();
    FXNPROFILE[profileIndex].startTime[0]  = CSL_armR5PmuReadCntr(0);
    FXNPROFILE[profileIndex].startTime[1]  = CSL_armR5PmuReadCntr(1);
    FXNPROFILE[profileIndex].startTime[2]  = CSL_armR5PmuReadCntr(2);
}


void app_fxnExit(uint32_t profileIndex)
{
    FXNPROFILE[profileIndex].deltaTotalTime[0]  += CSL_armR5PmuReadCntr(0)  - FXNPROFILE[profileIndex].startTime[0] ;
    FXNPROFILE[profileIndex].deltaTotalTime[1]  += CSL_armR5PmuReadCntr(1)  - FXNPROFILE[profileIndex].startTime[1] ;
    FXNPROFILE[profileIndex].deltaTotalTime[2]  += CSL_armR5PmuReadCntr(2)  - FXNPROFILE[profileIndex].startTime[2] ;
    FXNPROFILE[profileIndex].count++;
    EnetOsal_restoreAllIntr(FXNPROFILE[profileIndex].key);
}


#define LOAD_REPORT_MS            (10*1000)


uint32_t  loadList[64] = {0};
uint32_t  rxIsrCount[64] = {0};
uint32_t  txIsrCount[64] = {0};
uint32_t  rxPktCount[64] = {0};
uint32_t  txPktCount[64] = {0};
uint32_t  rxBytesCount[64] = {0};
uint32_t  txBytesCount[64] = {0};
appProfileFxnLoadInfo fxnLoadInfo[64] = {0};
appProfileConfig gAppProfileConfig;
#define APP_PROFILE_LOAD_WINDOW_IN_MS   (500U)


static void EnetApp_cpuLoadTask(void *a0)
{
    volatile uint32_t enableLoad = 1U;
    uint32_t  timeWindowElapsed;
    uint32_t  prevRxIsrCount;
    uint32_t  prevTxIsrCount;
    uint32_t  curRxIsrCount;
    uint32_t  curTxIsrCount;
    uint32_t  prevRxPktCount;
    uint32_t  prevTxPktCount;
    uint32_t  curRxPktCount;
    uint32_t  curTxPktCount;
    uint32_t  prevRxBytesCount;
    uint32_t  prevTxBytesCount;
    uint32_t  curRxBytesCount;
    uint32_t  curTxBytesCount;
    appProfileFxnLoadInfo avgFxnLoadInfo;
    appProfilePktCount_t pktCount;

    timeWindowElapsed = 0;
    config_pmu();
    while (enableLoad)
    {
        uint32_t key;

        key = HwiP_disable();
        if (timeWindowElapsed >= sizeof(loadList)/sizeof(loadList[0]))
        {
            timeWindowElapsed = 0;
        }
        memset(&fxnLoadInfo[timeWindowElapsed].fxn[0], 0, sizeof(fxnLoadInfo[timeWindowElapsed].fxn));
        gAppProfileConfig.getPacketCountFxn(&pktCount);
        prevRxIsrCount = pktCount.rxIsrCount;
        prevTxIsrCount = pktCount.txIsrCount;
        prevRxPktCount = pktCount.rxPktCount;
        prevTxPktCount = pktCount.txPktCount;
        prevRxBytesCount = pktCount.rxBytesCount;
        prevTxBytesCount = pktCount.txBytesCount;
        app_fxnLoadReset();
        HwiP_restore(key);
        ClockP_usleep(APP_PROFILE_LOAD_WINDOW_IN_MS  * 1000);
        key = HwiP_disable();
        gAppProfileConfig.getPacketCountFxn(&pktCount);
        curRxIsrCount = pktCount.rxIsrCount;
        curTxIsrCount = pktCount.txIsrCount;
        curRxPktCount = pktCount.rxPktCount;
        curTxPktCount = pktCount.txPktCount;
        curRxBytesCount = pktCount.rxBytesCount;
        curTxBytesCount = pktCount.txBytesCount;
        app_profileAccumulateFxnLoad(&fxnLoadInfo[timeWindowElapsed].fxn[0], FXNPROFILE, &fxnLoadInfo[timeWindowElapsed].numFunctions);
        HwiP_restore(key);

        loadList[timeWindowElapsed] = TaskP_loadGetTotalCpuLoad();
        rxIsrCount[timeWindowElapsed] = curRxIsrCount - prevRxIsrCount;
        txIsrCount[timeWindowElapsed] = curTxIsrCount - prevTxIsrCount;
        rxPktCount[timeWindowElapsed] = curRxPktCount - prevRxPktCount;
        txPktCount[timeWindowElapsed] = curTxPktCount - prevTxPktCount;
        rxBytesCount[timeWindowElapsed] = curRxBytesCount - prevRxBytesCount;
        txBytesCount[timeWindowElapsed] = curTxBytesCount - prevTxBytesCount;
        timeWindowElapsed++;
        if ((timeWindowElapsed * APP_PROFILE_LOAD_WINDOW_IN_MS) >= LOAD_REPORT_MS)
        {
            uint32_t i;
            uint32_t totalCPULoad = 0;
            uint32_t  totalRxIsrCount = 0;
            uint32_t  totalTxIsrCount = 0;
            uint32_t  totalRxPktCount = 0;
            uint32_t  totalTxPktCount = 0;
            uint32_t  totalRxBytesCount = 0;
            uint32_t  totalTxBytesCount = 0;

            for (i = 0 ; i < timeWindowElapsed;i++)
            {
                totalCPULoad += loadList[i];
                totalRxIsrCount += rxIsrCount[i];
                totalTxIsrCount += txIsrCount[i];
                totalRxPktCount += rxPktCount[i];
                totalTxPktCount += txPktCount[i];
                totalRxBytesCount += rxBytesCount[i];
                totalTxBytesCount += txBytesCount[i];
            }
            totalTxBytesCount = (totalTxBytesCount >> 17);
            totalRxBytesCount = (totalRxBytesCount >> 17);
            app_profileGetAvgFxnLoad(&avgFxnLoadInfo, &fxnLoadInfo[0], timeWindowElapsed);
            EnetAppUtils_print("CPU Load: %u%% RxISR Count:%u TxISR Count:%u \r\n", ((totalCPULoad / timeWindowElapsed)/100), (totalRxIsrCount / timeWindowElapsed), (totalTxIsrCount / timeWindowElapsed));
            EnetAppUtils_print("Tx Pkt count: %u @ %u mbps \r\n", totalTxPktCount, (totalTxBytesCount*1000)/LOAD_REPORT_MS);
            EnetAppUtils_print("Rx Pkt count: %u @ %u mbps \r\n", totalRxPktCount, (totalRxBytesCount*1000)/LOAD_REPORT_MS);
            EnetAppUtils_print("Total Tx Pkt count: %u, Total Rx Pkt count:%u\r\n", pktCount.txPktCount, pktCount.rxPktCount);
            loadPrintFxnProfile(&avgFxnLoadInfo);
            TaskP_loadResetAll();
            timeWindowElapsed = 0;
        }
    }
}


int32_t EnetApp_initCpuLoadTask(appProfileConfig *profileCfg, TaskP_Object * cpuLoadTask)
{
    TaskP_Params params;
    int32_t status;

    gAppProfileConfig = *profileCfg;
    /* Create task to print CPU load periodically */
    TaskP_Params_init(&params);
    params.name      = "CPU_LOAD";
    params.priority  = 15U;
    params.stack     = gCpuLoadTaskStack;
    params.stackSize = sizeof(gCpuLoadTaskStack);
    params.taskMain = EnetApp_cpuLoadTask;
    params.args     = &gAppProfileConfig;

    status = TaskP_construct(cpuLoadTask, &params);
    if (status != SystemP_SUCCESS)
    {
        EnetAppUtils_print("Failed to create CPU load task\r\n");
        EnetAppUtils_assert(false);
    }
    return status;
}

void EnetApp_deinitCpuLoadTask(TaskP_Object *hCpuLoadTask)
{
    TaskP_destruct(hCpuLoadTask);
}
