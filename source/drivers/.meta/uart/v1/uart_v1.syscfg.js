
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let hwi = system.getScript("/kernel/dpl/hwi.js");
let soc = system.getScript(`/drivers/uart/soc/uart_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find(o => o.name === solution.peripheralName);

    return {
        ...config,
        ...moduleInstance,
    };
};

function getInterfaceName(inst) {
    return soc.getInterfaceName(inst);
}

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

function getPeripheralPinNames(inst) {
    return [ "RX", "TX" ];
}

function pinmuxRequirements(inst) {
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    resources.push( pinmux.getPinRequirements(interfaceName, "RX", "UART RX Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "TX", "UART TX Pin"));

    let peripheral = {
        name: interfaceName,
        displayName: "UART Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

let uart_module_name = "/drivers/uart/uart";

let uart_driver_config_file = "/drivers/uart/templates/uart_config.c.xdt";
if ((common.getSocName() == "am273x") || (common.getSocName() == "awr294x"))
{
    uart_driver_config_file = "/drivers/uart/templates/uart_config_v1.c.xdt";
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
            driver_config: "/drivers/uart/templates/uart_v1.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/uart/templates/uart_sci_open_close_config.c.xdt",
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
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_UART",
    config: [
        {
            name: "baudRate",
            displayName: "Baudrate",
            default: 115200,
            description: "UART Baudrate",
            displayFormat: "dec",
        },
        {
            name: "dataLength",
            displayName: "Data Length",
            default: "8",
            options: [
                {
                    name: "1",
                    displayName: "1-bit"
                },
                {
                    name: "2",
                    displayName: "2-bit"
                },
                {
                    name: "3",
                    displayName: "3-bit"
                },
                {
                    name: "4",
                    displayName: "4-bit"
                },
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
            ],
            description: "Parity Type",
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
                }
                if(inst.intrEnable == "ENABLE") {
                    ui.intrPriority.hidden = false;
                    ui.readMode.hidden = false;
                    ui.writeMode.hidden = false;
                }
                if(inst.intrEnable == "DMA") {
                    ui.intrPriority.hidden = true;
                    ui.readMode.hidden = false;
                    ui.writeMode.hidden = false;
                }
                if(inst.intrEnable == "USER_INTR") {
                    ui.intrPriority.hidden = false;
                    ui.readMode.hidden = true;
                    ui.writeMode.hidden = true;
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
            onChange: function (inst, ui) {
                if(inst.readMode == "CALLBACK") {
                    ui.readCallbackFxn.hidden = false;
                }
                if(inst.readMode == "BLOCKING") {
                    ui.readCallbackFxn.hidden = true;
                }
            },
            description: "This determines whether the UART_Read operates synchronously or asynchronously",
            longDescription:`
- **BLOCKING:** UART_Read call block till completion
- **CALLBACK:** UART_Read call returns immediatey, Callback function called after completion`,
        },
        {
            name: "readCallbackFxn",
            displayName: "Read Transfer Callback",
            default: "NULL",
            hidden: true,
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
            onChange: function (inst, ui) {
                if(inst.writeMode == "CALLBACK") {
                    ui.writeCallbackFxn.hidden = false;
                }
                if(inst.writeMode == "BLOCKING") {
                    ui.writeCallbackFxn.hidden = true;
                }

            },
            description: "This determines whether the UART_Write operates synchronously or asynchronously",
            longDescription:`
- **BLOCKING:** UART_Write call block till completion
- **CALLBACK:** UART_Write call returns immediatey, Callback function called after completion`,
        },
        {
            name: "writeCallbackFxn",
            displayName: "Write Transfer Callback",
            default: "NULL",
            hidden: true,
            description: "Write transfer callback function when callback mode is selected",
        },
    ],
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
    common.validate.checkNumberRange(inst, report, "intrPriority", 0, hwi.getHwiMaxPriority(), "dec");
}

function addModuleInstances(inst) {
    let modInstances = new Array();

        modInstances.push({
            name: "edmaConfig",
            displayName: "EDMA",
            moduleName: "/drivers/edma/edma",
        });

    return modInstances;
}

exports = uart_module;
