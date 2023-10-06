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
            csOnTime               :   "GPMC_PSRAM_CS_ON_TIME",
            csRdOffTime            :   "GPMC_PSRAM_CS_RD_OFF_TIME",
            csWrOffTime            :   "GPMC_PSRAM_CS_WR_OFF_TIME",
            advOnTime              :   "GPMC_PSRAM_ADV_ON_TIME",
            advRdOffTime           :   "GPMC_PSRAM_ADV_RD_OFF_TIME",
            advWrOffTime           :   "GPMC_PSRAM_ADV_WR_OFF_TIME",
            advAadMuxOnTime        :   "GPMC_PSRAM_ADV_AADMUX_ON_TIME",
            advAadMuxRdOffTime     :   "GPMC_PSRAM_ADV_AADMUX_RD_OFF_TIME",
            advAadMuxWrOffTime     :   "GPMC_PSRAM_ADV_AADMUX_WR_OFF_TIME",
            weOnTtime              :   "GPMC_PSRAM_WE_ON_TIME",
            weOffTime              :   "GPMC_PSRAM_WE_OFF_TIME",
            oeOnTime               :   "GPMC_PSRAM_OE_ON_TIME",
            oeOffTime              :   "GPMC_PSRAM_OE_OFF_TIME",
            oeAadMuxOnTime         :   "GPMC_PSRAM_OE_AADMUX_ON_TIME",
            oeAadMuxOffTime        :   "GPMC_PSRAM_OE_AADMUX_OFF_TIME",
            pageBurstAccess        :   "GPMC_PSRAM_PAGEBURST_ACCESS_TIME",
            rdAccessTime           :   "GPMC_PSRAM_RD_ACCESS_TIME",
            wrAcessTime            :   "GPMC_PSRAM_WR_ACCESS_TIME",
            rdCycleTime            :   "GPMC_PSRAM_RD_CYCLE_TIME",
            wrCycleTime            :   "GPMC_PSRAM_WR_CYCLE_TIME",
            wrDataOnMuxBusTime     :   "GPMC_PSRAM_ADMUX_DATA_VALID",
            cycle2CycleDelay       :   "GPMC_PSRAM_C2C_DELAY",
            cycleDelaySameChipSel  :   "CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_NOC2CDELAY",//"CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_C2CDELAY",
            cycleDelayDiffChipSel  :   "CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_NOC2CDELAY",
            busTurnAroundTime      :   "GPMC_PSRAM_BRST_TAROUND_TIME",
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

