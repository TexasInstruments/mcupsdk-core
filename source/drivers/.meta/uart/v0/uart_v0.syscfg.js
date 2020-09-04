
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let hwi = system.getScript("/kernel/dpl/hwi.js");
let soc = system.getScript(`/drivers/uart/soc/uart_${common.getSocName()}`);

function getConfigArr() {
    return system.getScript(`/drivers/uart/soc/uart_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === solution.peripheralName);

    config.clockFrequencies[0].clkRate = moduleInstance.clkFreq;

    return {
        ...config,
        ...moduleInstance,
    };
};

function getInterfaceName(inst) {

    if(inst.useMcuDomainPeripherals)
        return "MCU_UART"

    return "UART";
}

function getPeripheralPinNames(inst) {
    return [ "RXD", "TXD", "RTSn", "CTSn" ];
}

function pinmuxRequirements(inst) {
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "RXD", "UART RX Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "TXD", "UART TX Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);
    if (inst.hwFlowControl)
    {
        pinResource = pinmux.getPinRequirements(interfaceName, "RTSn", "UART RTSn Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", true );
        resources.push( pinResource);
        pinResource = pinmux.getPinRequirements(interfaceName, "CTSn", "UART CTSn Pin");
        pinmux.setConfigurableDefault( pinResource, "rx", false );
        resources.push( pinResource);
    }

    let peripheral = {
        name: interfaceName,
        displayName: "UART Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

function getClockEnableIds(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}


let uart_module_name = "/drivers/uart/uart";
let uart_driver_config_file = "/drivers/uart/templates/uart_config_am64x_am243x.c.xdt";
let uart_driver_open_close_config = "/drivers/uart/templates/uart_open_close_config_am64x_am243x.c.xdt";
if (common.getSocName() == "am62x")
{
    uart_driver_config_file = "/drivers/uart/templates/uart_config_am62x.c.xdt";
    uart_driver_open_close_config = "/drivers/uart/templates/uart_open_close_config_am62x.c.xdt";
}

let uart_module = {
    displayName: "UART",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: uart_driver_config_file,
            driver_init: "/drivers/uart/templates/uart_init.c.xdt",
            driver_deinit: "/drivers/uart/templates/uart_deinit.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/uart/templates/uart.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: uart_driver_open_close_config,
            driver_open: "/drivers/uart/templates/uart_open.c.xdt",
            driver_close: "/drivers/uart/templates/uart_close.c.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/uart/templates/uart_open_close.h.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: uart_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: uart_module_name,
        },
    },
    defaultInstanceName: "CONFIG_UART",
    config: getConfigurables(),
    validate: validate,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    sharedModuleInstances: addModuleInstances,
    pinmuxRequirements,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
};

function addModuleInstances(instance) {
    let modInstances = new Array();

    if(instance.intrEnable == "DMA") {
        modInstances.push({
            name: "udmaDriver",
            displayName: "UDMA Configuration",
            moduleName: "/drivers/udma/udma",
        });
    }

    return modInstances;
}

function getConfigurables()
{
    let config = [];

    config.push(
        {
            name: "operMode",
            displayName: "Operational Mode",
            default: "16X",
            options: [
                {
                    name: "16X",
                    displayName: "16x",
                    description: "Configure 16X mode to support baudrate upto 230.4kbps and 3Mbps"
                },
                {
                    name: "16X_AUTO_BAUD",
                    displayName: "16x Auto Baud",
                    description: "Automatic Baudrate/Length/Parity detection from the received data ranging from 1.2kbps upto 115.2kbps"
                },
                {
                    name: "13X",
                    displayName: "13x",
                    description: "Configure 13X mode to support baudrate greater than 230.4kbps upto 3.6884Mbps"
                },
            ],
            description: "Operational Mode",
        },
        {
            name: "baudRate",
            displayName: "Baudrate",
            default: 115200,
            description: "UART Baudrate",
            displayFormat: "dec",
        },
        {
            name: "clkFreq",
            displayName: "Clock Freq",
            default: soc.getDefaultClkRate(),
            description: "Source Clock Frequency",
            displayFormat: "dec",
        },
        {
            name: "dataLength",
            displayName: "Data Length",
            default: "8",
            options: [
                {
                    name: "5",
                    displayName: "5-bit"
                },
                {
                    name: "6",
                    displayName: "6-bit"
                },
                {
                    name: "7",
                    displayName: "7-bit"
                },
                {
                    name: "8",
                    displayName: "8-bit"
                },
            ],
            description: "Data Length",
        },
        {
            name: "stopBits",
            displayName: "Stop Bit",
            default: "1",
            options: [
                {
                    name: "1",
                    displayName: "1-bit"
                },
                {
                    name: "2",
                    displayName: "2-bit"
                },
            ],
            description: "Stop Bit",
        },
        {
            name: "parityType",
            displayName: "Parity Type",
            default: "NONE",
            options: [
                {
                    name: "NONE",
                    displayName: "None"
                },
                {
                    name: "ODD",
                    displayName: "Odd"
                },
                {
                    name: "EVEN",
                    displayName: "Even"
                },
                {
                    name: "FORCED0",
                    displayName: "Forced to 0"
                },
                {
                    name: "FORCED1",
                    displayName: "Forced to 1"
                },
            ],
            description: "Parity Type",
        },
        {
            name: "hwFlowControl",
            displayName: "Enable Hardware Flow Control",
            default: false,
            onChange: function (inst, ui) {
            let hideConfigs = true;
            if(inst.hwFlowControl == true) {
                hideConfigs = false;
                }
                ui.hwFlowControlThr.hidden = hideConfigs;
            }
        },
        {
            name: "hwFlowControlThr",
            displayName: "Hardware Flow Control Threshold",
            default: "16",
            hidden: true,
            options: [
                {
                    name: "1",
                },
                {
                    name: "8",
                },
                {
                    name: "16",
                },
                {
                    name: "56",
                },
                {
                    name: "60",
                },
            ],
            description: "UART HW Flow Control Threshold, should be greater than or equal to Rx Trigger level",
        },
        /* Advanced parameters */
        {
            name: "intrEnable",
            displayName: "Transfer Mode",
            default: "ENABLE",
            hidden: false,
            options: [
                {
                    name: "DISABLE",
                    displayName: "Polled Mode"
                },
                {
                    name: "ENABLE",
                    displayName: "Interrupt Mode"
                },
                {
                    name: "USER_INTR",
                    displayName: "User Managed Interrupt"
                },
                {
                    name: "DMA",
                    displayName: "DMA Mode"
                },
            ],
            onChange: function (inst, ui) {
                if(inst.intrEnable == "DISABLE") {
                    ui.intrPriority.hidden = true;
                    ui.readMode.hidden = true;
                    ui.writeMode.hidden = true;
                    ui.readCallbackFxn.hidden = true;
                    ui.writeCallbackFxn.hidden = true;
                    ui.rxTrigLvl.hidden = true;
                    ui.txTrigLvl.hidden = true;
                    ui.readReturnMode.hidden = true;
                }
                if(inst.intrEnable == "ENABLE") {
                    ui.intrPriority.hidden = false;
                    ui.readMode.hidden = false;
                    ui.writeMode.hidden = false;
                    ui.readCallbackFxn.hidden = false;
                    ui.writeCallbackFxn.hidden = false;
                    ui.rxTrigLvl.hidden = false;
                    ui.txTrigLvl.hidden = false;
                    ui.readReturnMode.hidden = false;
                }
                if(inst.intrEnable == "DMA") {
                    ui.intrPriority.hidden = true;
                    ui.readMode.hidden = false;
                    ui.writeMode.hidden = false;
                    ui.readCallbackFxn.hidden = false;
                    ui.writeCallbackFxn.hidden = false;
                    ui.rxTrigLvl.hidden = true;
                    ui.txTrigLvl.hidden = true;
                    ui.readReturnMode.hidden = true;
                }
                if(inst.intrEnable == "USER_INTR") {
                    ui.intrPriority.hidden = false;
                    ui.readMode.hidden = true;
                    ui.writeMode.hidden = true;
                    ui.readCallbackFxn.hidden = true;
                    ui.writeCallbackFxn.hidden = true;
                    ui.rxTrigLvl.hidden = false;
                    ui.txTrigLvl.hidden = false;
                    ui.readReturnMode.hidden = true;
                }
            },
            description: "Transfer Mode",
            longDescription:`
- **Polled Mode:** Driver blocks on the UART status registers for operation completion
- **Interrupt Mode:** Driver registers the UART interrupts and Read/Write operations executed in the ISR
- **User Managed Interrupt:** driver interrupt registration is skipped and user need to manage the interrupt
- **DMA Mode:** Driver uses the EDMA fot the UART Read/Write operations`,
        },
        /* Advance Instance attributes */
        {
            name: "intrPriority",
            displayName: "Interrupt Priority",
            default: 4,
            hidden: false,
            description: `Interrupt Priority: 0 (highest) to ${hwi.getHwiMaxPriority()} (lowest)`,
        },
        {
            name: "rxTrigLvl",
            displayName: "RX Trigger Level",
            default: "8",
            hidden: false,
            options: [
                {
                    name: "1",
                },
                {
                    name: "8",
                },
                {
                    name: "16",
                },
                {
                    name: "56",
                },
                {
                    name: "60",
                },
            ],
            description: "RX trigger level",
        },
        {
            name: "txTrigLvl",
            displayName: "TX Trigger Level",
            default: "32",
            hidden: false,
            options: [
                {
                    name: "1",
                },
                {
                    name: "8",
                },
                {
                    name: "16",
                },
                {
                    name: "32",
                },
                {
                    name: "56",
                },
            ],
            description: "TX trigger level",
        },
        /* Open attributes */
        {
            name: "readMode",
            displayName: "Read Transfer Mode",
            default: "BLOCKING",
            hidden: false,
            options: [
                {
                    name: "BLOCKING",
                    displayName: "Blocking"
                },
                {
                    name: "CALLBACK",
                    displayName: "Callback"
                },
            ],
            description: "This determines whether the driver operates synchronously or asynchronously",
        },
        {
            name: "readCallbackFxn",
            displayName: "Read Transfer Callback",
            default: "NULL",
            hidden: false,
            description: "Read Transfer callback function when callback mode is selected",
        },
        {
            name: "writeMode",
            displayName: "Write Transfer Mode",
            default: "BLOCKING",
            hidden: false,
            options: [
                {
                    name: "BLOCKING",
                    displayName: "Blocking"
                },
                {
                    name: "CALLBACK",
                    displayName: "Callback"
                },
            ],
            description: "This determines whether the driver operates synchronously or asynchronously",
        },
        {
            name: "writeCallbackFxn",
            displayName: "Write Transfer Callback",
            default: "NULL",
            hidden: false,
            description: "Write transfer callback function when callback mode is selected",
        },
        {
            name: "readReturnMode",
            displayName: "Read Return Mode",
            default: "FULL",
            hidden: false,
            options: [
                {
                    name: "FULL",
                    displayName: "Full"
                },
                {
                    name: "PARTIAL",
                    displayName: "Partial"
                },
            ],
            description: "FULL unblocks or performs a callback when read buffer filled with the total num of bytes passed to UART_read().PARTIAL does whenever a few bytes passed/read timeout error occurs.",
        },
    )

    if(common.isMcuDomainSupported())
    {
        config.push(common.getUseMcuDomainPeripheralsConfig());
    }

    return config;
}

function validate(inst, report) {
    common.validate.checkValidCName(inst, report, "readCallbackFxn");
    common.validate.checkValidCName(inst, report, "writeCallbackFxn");
    if((inst.readMode == "CALLBACK") &&
        ((inst.readCallbackFxn == "NULL") ||
            (inst.readCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for callback transfer mode", inst, "readCallbackFxn");
    }
    if((inst.writeMode == "CALLBACK") &&
        ((inst.writeCallbackFxn == "NULL") ||
            (inst.writeCallbackFxn == ""))) {
        report.logError("Callback function MUST be provided for callback transfer mode", inst, "writeCallbackFxn");
    }
    if((inst.hwFlowControl == true) && (Number(inst.rxTrigLvl) > Number(inst.hwFlowControlThr))) {
        report.logError("HW flow control threshold should be greater than or equal to Rx trigger level", inst, "hwFlowControlThr");
    }
    if ((inst.useMcuDomainPeripherals) &&
        (inst.intrEnable == "DMA")) {
        report.logError("DMA is not supported in MCU Domain", inst, "useMcuDomainPeripherals");
    }
    if ((inst.intrEnable == "DMA") && (inst.readReturnMode == "PARTIAL")) {
        report.logError("Partial Mode is not supported in DMA", inst, "readReturnMode");
    }
    switch(inst.operMode) {
      case "16X":
    if((inst.baudRate > Number(230400)) && (inst.baudRate != Number(3000000))) {
        report.logError("Operating mode should be 13X for the configured baudrate", inst, "operMode");
        }
        break;
      case "16X_AUTO_BAUD":
    if((inst.baudRate < Number(1200)) || (inst.baudRate > Number(115200))) {
        report.logError("Configured baudrate range should be 1200 to 115200 for 16X_AUTO_BAUD mode", inst, "operMode");
        }
        break;
      case "13X":
    if(inst.baudRate <= Number(230400)) {
        report.logError("Operating mode should be 16X for the configured baudrate", inst, "operMode");
        }
    if(inst.baudRate == Number(3000000)) {
        report.logError("Operating mode should be 16X for the configured baudrate", inst, "operMode");
        }
        break;
    }
    common.validate.checkNumberRange(inst, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");
    common.validate.checkNumberRange(inst, report, "baudRate", 300, 3692300, "dec");
}

exports = uart_module;
