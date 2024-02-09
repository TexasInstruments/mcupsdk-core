let common = system.getScript("/common");
let srcclkfreq = 200000000;
let clockSourcesInfo;
let soc_name = common.getSocName();

if (soc_name == "awr294x")
{
    srcclkfreq = 150000000;
}

if( soc_name == "am263x" || soc_name == "am263px" || soc_name == "am273x" || soc_name == "awr294x")
{
    clockSourcesInfo = system.getScript(`/drivers/watchdog/soc/watchdog_${common.getSocName()}`).SOC_RcmClkSrcInfo;
}

function getConfigArr() {
    return system.getScript(`/drivers/watchdog/soc/watchdog_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function getClockFrequencies(inst) {
    let instConfig = getInstanceConfig(inst);
    if( soc_name == "am263x" || soc_name == "am263px" || soc_name == "am273x" || soc_name == "awr294x")
    {
        instConfig.clockFrequencies[0].clkRate = inst["wdt_func_clk"];
        instConfig.clockFrequencies[0].clkId = inst["wdt_clk_src"];
    }
    return instConfig.clockFrequencies;
}

function getInterfaceName(inst) {
    return "WDT";
}

function utilfunction(inst, ui)
{
    for (let args of clockSourcesInfo)
    {
        if(args.name == inst.wdt_clk_src)
        {
            srcclkfreq = args.freq;
            break;
        }
    }
}

function validatePair(instance, report)
{
    let mult_pair = (instance.expirationTime * instance.wdt_func_clk)/1000;
    let mult_pair_init = (instance.initializationTime * instance.wdt_func_clk)/1000;
    /*
        texp = (2^13(1+PRD)) /freq
        => PRD = (texp * freq)/2^13 -1
        0 <= PRD <= (2^12) -1  (RTIDWDPRLD[DWDPRLD] is a 12-bit field)
        0<= ( (texp * freq) /2^13) -1 <= 2^12 -1
        2^13 < = (texp * freq) <= 2^25
    */
    if( mult_pair_init < 8192 || mult_pair_init > 33554432)
    report.logError(`(Initialization time * WDT input clock frequency) = ${mult_pair_init} has to be within 8192 to 33554432`, instance, "initializationTime");
    if( mult_pair < 8192 || mult_pair > 33554432)
    report.logError(`(Expiration time * WDT input clock frequency) = ${mult_pair} has to be within 8192 to 33554432`, instance, "expirationTime");
}

function validateInputClkFreq(instance, report)
{

    let mod = srcclkfreq % (instance.wdt_func_clk);
    let div = srcclkfreq / (instance.wdt_func_clk);

    if(instance["wdt_clk_src"] == "SOC_RcmPeripheralClockSource_XTALCLK" && div>=5)
        report.logError(`For XTALCLK, you can only choose values above 5 MHz ${div}`, instance, "wdt_func_clk");

    if( mod != 0 || div < 0 || div > 16)
        report.logError(`The WDT input clock frequency has to be INTEGRAL divisor of Clock source ${srcclkfreq} Hz
        and divisor = ${div} should be between 1 to 16`, instance, "wdt_func_clk");

}

function validate(instance, report) {
    common.validate.checkSameInstanceName(instance, report);
    common.validate.checkNumberRange(instance, report, "expirationTime", 0, 60000, "dec");
    common.validate.checkNumberRange(instance, report, "initializationTime", 0, 60000, "dec");
    if( soc_name == "am263x" || soc_name == "am263px" || soc_name == "am273x" || soc_name == "awr294x")
    {
        validatePair(instance, report);
        validateInputClkFreq(instance, report);
    }
}

let clock_sources = []

if( soc_name == "am263x" || soc_name == "am263px" || soc_name == "am273x" || soc_name == "awr294x")
{
    for (let arg of clockSourcesInfo)
    {
        let list = {name: arg.name, displayName: arg.displayName}

        clock_sources =clock_sources.concat(list);
    }
}

let watchdog_module_name = "/drivers/watchdog/watchdog";
let gconfig = [];
gconfig = gconfig.concat([
    common.ui.makeInstanceConfig(getConfigArr()),
    {
        name: "resetMode",
        displayName: "WDT Reset Mode",
        default: "Watchdog_RESET_ON",
        options: [
            {
                name: "Watchdog_RESET_ON",
                displayName: "trigger warm reset"
            },
            {
                name: "Watchdog_RESET_OFF",
                displayName: "trigger WDT interrupt"
            },
        ],
        description: "Reaction to select on WDT expiry, currently supporting only to trigger warm reset",
    },
    {
        name: "windowSize",
        displayName: "Digital WDT Window Size",
        default: "Watchdog_WINDOW_100_PERCENT",
        options: [
            {
                name: "Watchdog_WINDOW_100_PERCENT",
                displayName: "100 Percent"
            },
            {
                name: "Watchdog_WINDOW_50_PERCENT",
                displayName: "50 Percent"
            },
            {
                name: "Watchdog_WINDOW_25_PERCENT",
                displayName: "25 Percent"
            },
            {
                name: "Watchdog_WINDOW_12_5_PERCENT",
                displayName: "12.5 Percent"
            },
            {
                name: "Watchdog_WINDOW_6_25_PERCENT",
                displayName: "6.25 Percent"
            },
            {
                name: "Watchdog_WINDOW_3_125_PERCENT",
                displayName: "3.125 Percent"
            },
        ],
        description: "WDT Window size",
    },
])

if( soc_name == "am263x" || soc_name == "am263px" || soc_name == "am273x" || soc_name == "awr294x")
{
    gconfig = gconfig.concat([
    {
        name: "initializationTime",
        displayName: "WDT Initialization Timeout (ms)",
        default: 165,
        description: "Initialization Timeout in millisecond (ms)",
        longDescription: "Watchdog is started with this timeout at the beginning of Drivers_open before all other peripheral drivers open.",
    },
    {
        name: "expirationTime",
        displayName: "WDT Runtime Timeout (ms)",
        default: 165,
        description: "Runtime Timeout in millisecond (ms)",
        longDescription: "Watchdog is reinitialized with this timeout for the application runtime duration.",
    },
    {
        name: "wdt_clk_src",
        displayName: "WDT Clock Source",
        default: clock_sources[1].name,
        description: "WDT Clock Source",
        options : clock_sources,
        onChange: utilfunction
        },
        {
            name: "wdt_func_clk",
            displayName: "WDT Input Clock Frequency (Hz)",
            hidden: false,
            default: 200000000,
            description: "WDT Input Clock frequency (Hz)",
        },
    ])
}
else{
    gconfig = gconfig.concat([
    {
        name: "initializationTime",
        displayName: "WDT Initialization Timeout (ms)",
        default: 1000,
        description: "Initialization Timeout in millisecond (ms)",
        longDescription: "Watchdog is started with this timeout at the beginning of Drivers_open before all other peripheral drivers open.",
    },
    {
        name: "expirationTime",
        displayName: "WDT Runtime Timeout (ms)",
        default: 1000,
        description: "Runtime Timeout in millisecond (ms)",
        longDescription: "Watchdog is reinitialized with this timeout for the application runtime duration.",
    },
   ])
}
let watchdog_module = {
    displayName: "WDT",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/watchdog/templates/watchdog_config.c.xdt",
            driver_init: "/drivers/watchdog/templates/watchdog_init.c.xdt",
            driver_deinit: "/drivers/watchdog/templates/watchdog_deinit.c.xdt",
			moduleName: watchdog_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/watchdog/templates/watchdog.h.xdt",
			moduleName: watchdog_module_name,
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/watchdog/templates/watchdog_open_close_config.c.xdt",
            driver_open: "/drivers/watchdog/templates/watchdog_open.c.xdt",
            driver_runtime_open: "/drivers/watchdog/templates/watchdog_open_runtime.c.xdt",
            driver_close: "/drivers/watchdog/templates/watchdog_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/watchdog/templates/watchdog_open_close.h.xdt",
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: watchdog_module_name,
        },
    },
    defaultInstanceName: "CONFIG_WDT",
    config : gconfig,
    validate: validate,
    maxInstances: getConfigArr().length,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
    getInterfaceName,
    getClockEnableIds,
    getClockFrequencies,
};

exports = watchdog_module;
