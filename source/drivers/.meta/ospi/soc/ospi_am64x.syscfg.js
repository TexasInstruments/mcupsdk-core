let common = system.getScript("/common");

let ospi_input_clk_freq = 133333333;

const ospi_config_r5fss = [
    {
        name            : "OSPI0",
        baseAddr        : "CSL_FSS0_OSPI0_CTRL_BASE",
        dataBaseAddr    : "CSL_FSS0_DAT_REG1_BASE",
        inputClkFreq    : ospi_input_clk_freq,
        dacEnable       : false,
        baudRateDiv     : 4,
        intrNum         : 171,
        clockIds        : [ "TISCI_DEV_FSS0", "TISCI_DEV_FSS0_FSAS_0", "TISCI_DEV_FSS0_OSPI_0" ],
        clockFrequencies: [
            {
                moduleId: "TISCI_DEV_FSS0_OSPI_0",
                clkId   : "TISCI_DEV_FSS0_OSPI_0_OSPI_RCLK_CLK",
                clkRate : ospi_input_clk_freq,
            },
        ],
    },
];

const ospi_dma_restrict_regions = [
    { start : "CSL_R5FSS0_ATCM_BASE"    , size : "CSL_R5FSS0_ATCM_SIZE" },
    { start : "CSL_MCU_M4FSS0_IRAM_BASE", size : "CSL_MCU_M4FSS0_IRAM_SIZE" },
    { start : "CSL_MCU_M4FSS0_DRAM_BASE", size : "CSL_MCU_M4FSS0_DRAM_SIZE" },
];

function getDefaultConfig()
{
    return ospi_config_r5fss[0];
}

function getConfigArr() {

    return ospi_config_r5fss;
}

function getDmaRestrictedRegions() {

    return ospi_dma_restrict_regions;
}

function getSupportedDataLines() {
    return 8;
}

function addModuleInstances(instance) {
    let modInstances = new Array();

    if(instance.dmaEnable == true) {
        modInstances.push({
            name: "udmaDriver",
            displayName: "UDMA Configuration",
            moduleName: "/drivers/udma/udma",
        });
    }

    return modInstances;
}

let ospi_module_name = "/drivers/ospi/ospi";

function getTemplates()
{
    return {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/ospi/templates/ospi_config.c.xdt",
            driver_init: "/drivers/ospi/templates/ospi_init.c.xdt",
            driver_deinit: "/drivers/ospi/templates/ospi_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/ospi/templates/ospi.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/ospi/templates/ospi_open_close_config.c.xdt",
            driver_open: "/drivers/ospi/templates/ospi_open.c.xdt",
            driver_close: "/drivers/ospi/templates/ospi_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/ospi/templates/ospi_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: ospi_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: ospi_module_name,
        },
    };
}

exports = {
    getDefaultConfig,
    getConfigArr,
    getDmaRestrictedRegions,
    getSupportedDataLines,
    addModuleInstances,
    getTemplates
};


