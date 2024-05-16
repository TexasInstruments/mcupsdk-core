let common = system.getScript("/common");

let gpmc_input_clk_freq = 133333333;

const gpmc_config_r5fss = [
    {
        name            : "GPMC0",
        baseAddr        : "CSL_GPMC0_CFG_BASE",
        dataBaseAddr    : "CSL_GPMC0_DATA_BASE",
        dataBaseAddrVal : 0x50000000,
        elmBaseAddr     : "CSL_ELM0_BASE",
        inputClkFreq    : gpmc_input_clk_freq,
        clockRateDiv    : 1,
        intrNum         : "CSLR_R5FSS0_CORE0_INTR_GPMC0_GPMC_SINTERRUPT_0",
        dmaLocalEventID : 29,
        clockIds        : [ "TISCI_DEV_GPMC0", "TISCI_DEV_ELM0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_GPMC0",
                clkId   : "TISCI_DEV_GPMC0_FUNC_CLK",
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

const gpmc_config_nandlike_device = {
    waitPinPol         :   "CSL_GPMC_CONFIG_WAIT0PINPOLARITY_W0ACTIVEL",
    addrDataMux        :   "NONMUX",
    timeLatency        :   "X1",
    timingParams       :
        {
            csOnTime               :   0,
            csRdOffTime            :   6,
            csWrOffTime            :   6,
            advOnTime              :   0,
            advRdOffTime           :   4,
            advWrOffTime           :   4,
            advAadMuxOnTime        :   0,
            advAadMuxRdOffTime     :   0,
            advAadMuxWrOffTime     :   0,
            weOnTtime              :   0,
            weOffTime              :   3,
            oeOnTime               :   1,
            oeOffTime              :   4,
            oeAadMuxOnTime         :   0,
            oeAadMuxOffTime        :   0,
            pageBurstAccess        :   0,
            rdAccessTime           :   4,
            wrAcessTime            :   6,
            rdCycleTime            :   6,
            wrCycleTime            :   6,
            wrDataOnMuxBusTime     :   0,
            cycle2CycleDelay       :   0,
            cycleDelaySameChipSel  :   "NOC2CDELAY",
            cycleDelayDiffChipSel  :   "NOC2CDELAY",
            busTurnAroundTime      :   0,
        },
    eccAlgo            :   "GPMC_NAND_ECC_ALGO_BCH_8BIT",
    readType           :   "CSL_GPMC_CONFIG1_READTYPE_RDASYNC",
    csExDelay          :   "CSL_GPMC_CONFIG2_CSEXTRADELAY_NOTDELAYED",
    accessType         :   "CSL_GPMC_CONFIG1_READMULTIPLE_RDSINGLE",
    optimisedAccess    :   "DISABLED",
    cycleOptimisation  :   0,
}

const gpmc_config_psram_device = {
    waitPinPol         :   "CSL_GPMC_CONFIG_WAIT0PINPOLARITY_W0ACTIVEL",
    addrDataMux        :   "NONMUX",
    timeLatency        :   "X1",
    timingParams       :
        {
            csOnTime               :   1,
            csRdOffTime            :   0,
            csWrOffTime            :   1,
            advOnTime              :   4,
            advRdOffTime           :   5,
            advWrOffTime           :   6,
            advAadMuxOnTime        :   1,
            advAadMuxRdOffTime     :   2,
            advAadMuxWrOffTime     :   2,
            weOnTtime              :   5,
            weOffTime              :   3,
            oeOnTime               :   6,
            oeOffTime              :   0,
            oeAadMuxOnTime         :   1,
            oeAadMuxOffTime        :   7,
            pageBurstAccess        :   1,
            rdAccessTime           :   15,
            wrAcessTime            :   15,
            rdCycleTime            :   17,
            wrCycleTime            :   17,
            wrDataOnMuxBusTime     :   0,
            cycle2CycleDelay       :   0,
            cycleDelaySameChipSel  :   "NOC2CDELAY",
            cycleDelayDiffChipSel  :   "NOC2CDELAY",
            busTurnAroundTime      :   0,
        },
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

function getNandlikeGpmcConfig ()
{
    return gpmc_config_nandlike_device;
}

function getPsramGpmcConfig ()
{
    return gpmc_config_psram_device;
}

exports = {
    getDefaultConfig,
    getConfigArr,
    getNandlikeGpmcConfig,
    getPsramGpmcConfig,
    getDmaRestrictedRegions,
};


