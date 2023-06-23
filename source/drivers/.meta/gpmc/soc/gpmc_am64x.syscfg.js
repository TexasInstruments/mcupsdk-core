let common = system.getScript("/common");

let gpmc_input_clk_freq = 133333333;

const gpmc_config_r5fss = [
    {
        name            : "GPMC0",
        baseAddr        : "CSL_GPMC0_CFG_BASE",
        dataBaseAddr    : "CSL_GPMC0_DATA_BASE",
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
    addrDataMux        :   "CSL_GPMC_CONFIG1_MUXADDDATA_NONMUX",
    timeLatency        :   "CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_X1",//"CSL_GPMC_CONFIG1_TIMEPARAGRANULARITY_X2",
    timingParams       :
        {
            csOnTime               :   "GPMC_NAND_CS_ON_TIME",
            csRdOffTime            :   "GPMC_NAND_CS_RD_OFF_TIME",
            csWrOffTime            :   "GPMC_NAND_CS_WR_OFF_TIME",
            advOnTime              :   "GPMC_NAND_ADV_ON_TIME",
            advRdOffTime           :   "GPMC_NAND_ADV_RD_OFF_TIME",
            advWrOffTime           :   "GPMC_NAND_ADV_WR_OFF_TIME",
            advAadMuxOnTime        :   "GPMC_NAND_ADV_AADMUX_ON_TIME",
            advAadMuxRdOffTime     :   "GPMC_NAND_ADV_AADMUX_RD_OFF_TIME",
            advAadMuxWrOffTime     :   "GPMC_NAND_ADV_AADMUX_WR_OFF_TIME",
            weOnTtime              :   "GPMC_NAND_WE_ON_TIME",
            weOffTime              :   "GPMC_NAND_WE_OFF_TIME",
            oeOnTime               :   "GPMC_NAND_OE_ON_TIME",
            oeOffTime              :   "GPMC_NAND_OE_OFF_TIME",
            oeAadMuxOnTime         :   "GPMC_NAND_OE_AADMUX_ON_TIME",
            oeAadMuxOffTime        :   "GPMC_NAND_OE_AADMUX_OFF_TIME",
            pageBurstAccess        :   "GPMC_NAND_PAGEBURST_ACCESS_TIME",
            rdAccessTime           :   "GPMC_NAND_RD_ACCESS_TIME",
            wrAcessTime            :   "GPMC_NAND_WR_ACCESS_TIME",
            rdCycleTime            :   "GPMC_NAND_RD_CYCLE_TIME",
            wrCycleTime            :   "GPMC_NAND_WR_CYCLE_TIME",
            wrDataOnMuxBusTime     :   "GPMC_NAND_ADMUX_DATA_VALID",
            cycle2CycleDelay       :   "GPMC_NAND_C2C_DELAY",
            cycleDelaySameChipSel  :   "CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_NOC2CDELAY",//"CSL_GPMC_CONFIG6_CYCLE2CYCLESAMECSEN_C2CDELAY",
            cycleDelayDiffChipSel  :   "CSL_GPMC_CONFIG6_CYCLE2CYCLEDIFFCSEN_NOC2CDELAY",
            busTurnAroundTime      :   "GPMC_NAND_BRST_TAROUND_TIME",
        },
    eccAlgo            :   "GPMC_NAND_ECC_ALGO_BCH_8BIT",
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

function getNandlikeGpmcConfig ()
{
    return gpmc_config_nandlike_device;
}

exports = {
    getDefaultConfig,
    getConfigArr,
    getNandlikeGpmcConfig,
    getDmaRestrictedRegions,
};


