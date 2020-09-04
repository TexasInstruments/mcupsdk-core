let common = system.getScript("/common");

let mmcsd_input_clk_freq = 192000000;

const mmcsd_config_r5fss = [
	{
		name              : "MMC",
		baseAddr          : "CSL_MMC0_U_BASE",
        instNum           : 0,
        intrNum           : 83,
		inputClkFreq      : mmcsd_input_clk_freq,
        outputClk         : 400000,
        iodelayFxn        : "NULL",
        switchVoltageFxn  : "NULL",
        inputClockControl : "NULL",
		busWidth          : "MMCSD_BUS_WIDTH_8BIT",
		modes             : "MMCSD_SUPPORT_MMC_DS | MMCSD_SUPPORT_MMC_HS_SDR",
		clockIds          : [ "SOC_RcmPeripheralId_MMC0" ],
		clockFrequencies  : [
			{
				moduleId  : "SOC_RcmPeripheralId_MMC0",
				clkId     : "SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1",
				clkRate   : mmcsd_input_clk_freq,
			},
		],
	},
];

function getDefaultConfig() {
	return mmcsd_config_r5fss[0];
}

function getConfigArr() {
	return mmcsd_config_r5fss;
}

exports = {
	getDefaultConfig,
	getConfigArr,
};