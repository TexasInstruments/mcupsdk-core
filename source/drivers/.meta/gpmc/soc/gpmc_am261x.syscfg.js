let common = system.getScript("/common");

let gpmc_input_clk_freq = 100000000;

const gpmc_config_r5fss = [
    {
        name            : "GPMC",
        baseAddr        : "CSL_GPMC0_CFG_U_BASE",
        dataBaseAddr    : "CSL_GPMC0_MEM_U_BASE",
        elmBaseAddr     : "CSL_ELM0_BASE",
        inputClkFreq    : gpmc_input_clk_freq,
        clockRateDiv    : 1,
        intrNum         : "CSLR_R5FSS0_CORE0_GPMC_SINTR",
        dmaLocalEventID : 29,
        clockIds        : [ "SOC_RcmPeripheralId_GPMC" ],
        clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_GPMC",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : gpmc_input_clk_freq,
            },
        ],
    },
];

const gpmc_dma_restrict_regions = [
    { start : "CSL_R5FSS0_ATCM_BASE"    , size : "CSL_R5FSS0_ATCM_SIZE" },
    { start : "CSL_R5FSS0_BTCM_BASE"    , size : "CSL_R5FSS0_BTCM_SIZE" },
    { start : "CSL_R5FSS1_ATCM_BASE"    , size : "CSL_R5FSS1_ATCM_SIZE" },
    { start : "CSL_R5FSS1_BTCM_BASE"    , size : "CSL_R5FSS1_BTCM_SIZE" },
    { start : "CSL_MCU_M4FSS0_IRAM_BASE", size : "CSL_MCU_M4FSS0_IRAM_SIZE" },
    { start : "CSL_MCU_M4FSS0_DRAM_BASE", size : "CSL_MCU_M4FSS0_DRAM_SIZE" },
];

const gpmc_config_norlike_device = {
    waitPinPol         :   "CSL_GPMC_CONFIG_WAIT0PINPOLARITY_W0ACTIVEL",
    addrDataMux        :   "CSL_GPMC_CONFIG1_MUXADDDATA_NONMUX",
    timeLatency        :   "CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_X1",//"CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_X2",
    timingParams       :
        {
            csOnTime               :   3,
            csRdOffTime            :   21,
            csWrOffTime            :   21,
            advOnTime              :   1,
            advRdOffTime           :   3,
            advWrOffTime           :   3,
            advAadMuxOnTime        :   1,
            advAadMuxRdOffTime     :   2,
            advAadMuxWrOffTime     :   2,
            weOnTtime              :   10,
            weOffTime              :   16,
            oeOnTime               :   10,
            oeOffTime              :   1,
            oeAadMuxOnTime         :   1,
            oeAadMuxOffTime        :   15,
            pageBurstAccess        :   3,
            rdAccessTime           :   16,
            wrAcessTime            :   8,
            rdCycleTime            :   23,
            wrCycleTime            :   23,
            wrDataOnMuxBusTime     :   0,
            cycle2CycleDelay       :   0,
            cycleDelaySameChipSel  :   "CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_NOC2CDELAY",
            cycleDelayDiffChipSel  :   "CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_NOC2CDELAY",
            busTurnAroundTime      :   1,
        },
    eccAlgo            :   "GPMC_PSRAM_ECC_ALGO_BCH_8BIT",
    readType           :   "CSL_GPMC_CONFIG1_READTYPE_RDASYNC",
    csExDelay          :   "CSL_GPMC_CONFIG2_CSEXTRADELAY_NOTDELAYED",
    accessType         :   "CSL_GPMC_CONFIG1_READMULTIPLE_RDSINGLE",
}

function getDefaultConfig()
{
    return gpmc_config_r5fss[0];
}

function getConfigArr() {

    return gpmc_config_r5fss;
}

function getDmaRestrictedRegions() {

    return gpmc_dma_restrict_regions;
}

function getNorlikeGpmcConfig ()
{
    return gpmc_config_norlike_device;
}

exports = {
    getDefaultConfig,
    getConfigArr,
    getNorlikeGpmcConfig,
    getDmaRestrictedRegions,
};

