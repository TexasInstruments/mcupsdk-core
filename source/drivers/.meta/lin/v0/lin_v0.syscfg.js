
let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/lin/soc/lin_${common.getSocName()}`);

let LIN_InterruptFlags = [
    {name: " LIN_INT_WAKEUP", displayName : " Wakeup "},
    {name: " LIN_INT_TO", displayName : " Time out "},
    {name: " LIN_INT_TOAWUS", displayName : " Time out after wakeup signal "},
    {name: " LIN_INT_TOA3WUS", displayName : " Time out after 3 wakeup signals "},
    {name: " LIN_INT_TX", displayName : " Transmit buffer ready "},
    {name: " LIN_INT_RX", displayName : " Receive buffer ready "},
    {name: " LIN_INT_ID", displayName : " Received matching identifier "},
    {name: " LIN_INT_PE", displayName : " Parity error "},
    {name: " LIN_INT_OE", displayName : " Overrun error "},
    {name: " LIN_INT_FE", displayName : " Framing error "},
    {name: " LIN_INT_NRE", displayName : " No response error "},
    {name: " LIN_INT_ISFE", displayName : " Inconsistent sync field error "},
    {name: " LIN_INT_CE", displayName : " Checksum error "},
    {name: " LIN_INT_PBE", displayName : " Physical bus error "},
    {name: " LIN_INT_BE", displayName : " Bit error "},
    {name: " LIN_INT_ALL", displayName : " All interrupts "},
];

function getConfigArr() {
    return system.getScript(`/drivers/lin/soc/lin_${common.getSocName()}`).getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === solution.peripheralName);

    return {
        ...config,
        ...moduleInstance
    }
}

function getPeripheralPinNames(inst) {
    return [ "RXD", "TXD" ];
}

function pinmuxRequirements(inst) {
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    let pinResource = {};

    pinResource = pinmux.getPinRequirements(interfaceName, "RXD", "LIN RX Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", true );
    resources.push( pinResource);
    pinResource = pinmux.getPinRequirements(interfaceName, "TXD", "LIN TX Pin");
    pinmux.setConfigurableDefault( pinResource, "rx", false );
    resources.push( pinResource);

    let peripheral = {
        name: interfaceName,
        displayName: "LIN Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

function getInterfaceName(instance) {
    return soc.getInterfaceName(instance);
}

function getClockEnableIds(instance) {
    let instConfig = getInstanceConfig(instance);
    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

function validate(instance, report) {
    /* None. Verified by SYSCFG based on selected pin */
}

function EnableInterruptChanged(inst, ui) {
    if(inst.enableInterrupt == false){
        ui.interruptFlags.hidden = true
        ui.interruptLine.hidden = true
        ui.interruptPriorityLine0.hidden = true
        ui.interruptPriorityLine1.hidden = true
    }
    else{
        ui.interruptFlags.hidden = false
        ui.interruptLine.hidden = false
        ui.interruptPriorityLine0.hidden = false
        ui.interruptPriorityLine1.hidden = false
    }
}

function LoopBackTypeChanged(inst, ui) {
    if (inst.loopbackType == "LIN_LOOPBACK_DIGITAL"){
        ui.loopbackPath.hidden = true
    }
    else{
        ui.loopbackPath.hidden = false
    }
}

function LoopBackModeChanged(inst, ui) {
    if (inst.loopbackMode == "LIN_TEST_INTERNAL"){
        ui.loopbackType.hidden = true
        ui.loopbackPath.hidden = true
        inst.loopbackType = "LIN_LOOPBACK_DIGITAL"
    }
    else{
        ui.loopbackType.hidden = false
    }
}

function OverrideEnableLoopback(inst, ui) {
    if (inst.enableLoopback == false){
        ui.loopbackMode.hidden = true
        ui.loopbackType.hidden = true
        ui.loopbackPath.hidden = true
        inst.loopbackMode = "LIN_TEST_INTERNAL"
        inst.loopbackType = "LIN_LOOPBACK_DIGITAL"
    }
    else{
        ui.loopbackMode.hidden = false
    }
}

function OverrideDefaultConfiguration(inst, ui) {
    if (inst.useDefault == false){
        ui.linMode.hidden = false
        ui.linCommMode.hidden = false
        ui.linDebugMode.hidden = false
        ui.linChecksum.hidden = false
        ui.linMsgFilter.hidden = false
        ui.enableParityCheck.hidden = false
    }
    else{
        ui.linMode.hidden = true
        ui.linCommMode.hidden = true
        ui.linDebugMode.hidden = true
        ui.linChecksum.hidden = true
        ui.linMsgFilter.hidden = true
        ui.enableParityCheck.hidden = true
    }
}

let config = [
    {
        name        : "enableLoopback",
        displayName : "Enable Loopback",
        onChange    : OverrideEnableLoopback,
        description : 'Whether to Enable the Loopback or not',
        hidden      : false,
        default     : false
    },
    {
        name        : "loopbackMode",
        displayName : "Loopback Mode",
        onChange    : LoopBackModeChanged,
        description : 'Whether Loopback is internal or external',
        hidden      : true,
        default     : "LIN_TEST_INTERNAL",
        options     :
        [
            {name: "LIN_TEST_INTERNAL", displayName : "Internal Loopback Mode"},
            {name: "LIN_TEST_EXTERNAL", displayName : "External Loopback Mode"},
        ]
    },
    {
        name        : "loopbackType",
        displayName : "Type of Loopback",
        onChange    : LoopBackTypeChanged,
        description : 'Whether Loopback is analog or digital',
        hidden      : true,
        default     : "LIN_LOOPBACK_DIGITAL",
        options     :
        [
            { name: "LIN_LOOPBACK_DIGITAL", displayName: "Digital Loopback Mode" },
            { name: "LIN_LOOPBACK_ANALOG", displayName: "Analog Loopback Mode" },
        ]
    },
    {
        name        : "loopbackPath",
        displayName : "Loopback Path In Analog Loopback",
        onChange    : LoopBackTypeChanged,
        description : 'Loopback path in Analog loopback',
        hidden      : true,
        default     : "LIN_ANALOG_LOOP_NONE",
        options     :
        [
            { name: "LIN_ANALOG_LOOP_NONE", displayName: "Default path for digital loopback mode" },
            { name: "LIN_ANALOG_LOOP_TX", displayName: "Analog loopback through transmit pin" },
            { name: "LIN_ANALOG_LOOP_RX", displayName: "Analog loopback through receive pin" },
        ]
    },
    {
        name        : "enableInterrupt",
        displayName : "Enable Interrupt",
        onChange    : EnableInterruptChanged,
        description : 'Enable Interrupt',
        hidden      : false,
        default     : false
    },
    {
        name        : "interruptFlags",
        displayName : "Interrupt Flags",
        description : 'Which Interrupts to enable',
        hidden      : true,
        minSelections : 0,
        default     : [],
        options     : LIN_InterruptFlags,

    },
    {
        name        : "interruptLine",
        displayName : "Interrupt Line",
        description : 'Which Interrupt Lines to enable',
        hidden      : true,
        minSelections : 0,
        default     : [],
        options     :
        [
            { name: "LIN_INTERRUPT_LINE0", displayName: "Interrupt line 0" },
            { name: "LIN_INTERRUPT_LINE1", displayName: "Interrupt line 1" },
        ]
    },
    {
        name        : "interruptPriorityLine0",
        displayName : "Interrupt Line 0 Triggers",
        description : 'Interrupt Line 0 triggers',
        hidden      : true,
        minSelections : 0,
        default     : [],
        options     : LIN_InterruptFlags,
    },
    {
        name        : "interruptPriorityLine1",
        displayName : "Interrupt Line 1 Triggers",
        description : 'Interrupt Line 1 triggers',
        hidden      : true,
        minSelections : 0,
        default     : [],
        options     : LIN_InterruptFlags,
    },
    {
        name        : "useDefault",
        displayName : "Use Default Configuration",
        onChange    : OverrideDefaultConfiguration,
        description : 'Whether to use default initialization or custom',
        hidden      : false,
        default     : true
    },
    {
        name        : "linMode",
        displayName : "LIN Mode",
        description : 'Whether to use LIN as a Commander or a Responder',
        hidden      : true,
        default     : "LIN_MODE_LIN_COMMANDER",
        options     :
        [
            { name: "LIN_MODE_LIN_COMMANDER", displayName: "The node is in commander mode" },
            { name: "LIN_MODE_LIN_RESPONDER", displayName: "The node is in responder mode" },
        ]
    },
    {
        name        : "linCommMode",
        displayName : "LIN Communication Mode",
        description : 'Length to be used from SCIFORMAT or ID4 and ID5 bit fields',
        hidden      : true,
        default     : "LIN_COMM_LIN_USELENGTHVAL",
        options     :
        [
            { name: "LIN_COMM_LIN_USELENGTHVAL", displayName: "Use the length indicated in the LENGTH field of the SCIFORMAT register" },
            { name: "LIN_COMM_LIN_ID4ID5LENCTL", displayName: "Use ID4 and ID5 to convey the length" },
        ]
    },
    {
        name        : "linDebugMode",
        displayName : "LIN Debug Mode",
        description : 'To determine how to freeze the Debug mode',
        hidden      : true,
        default     : "LIN_DEBUG_FROZEN",
        options     :
        [
            { name: "LIN_DEBUG_FROZEN", displayName: "Freeze module during debug" },
            { name: "LIN_DEBUG_COMPLETE", displayName: "Complete Tx/Rx before Freezing" },
        ]
    },
    {
        name        : "linChecksum",
        displayName : "LIN Checksum Type",
        description : 'To determine what type of checksum to be used',
        hidden      : true,
        default     : "LIN_CHECKSUM_CLASSIC",
        options     :
        [
            { name: "LIN_CHECKSUM_CLASSIC", displayName: "Checksum Classic" },
            { name: "LIN_CHECKSUM_ENHANCED", displayName: "Checksum Enhanced" },
        ]
    },
    {
        name        : "linMsgFilter",
        displayName : "LIN Message Filter",
        description : 'To determine what type of message filter to be used',
        hidden      : true,
        default     : "LIN_MSG_FILTER_IDBYTE",
        options     :
        [
            { name: "LIN_MSG_FILTER_IDBYTE", displayName: "LIN Message ID Byte Filtering" },
            { name: "LIN_MSG_FILTER_IDRESPONDER", displayName: "Responder Task ID Byte Filtering" },
        ]
    },
    {
        name        : "enableParityCheck",
        displayName : "Enable Parity Check",
        description : 'Enable Parity Check on the Messages',
        hidden      : true,
        default     : true
    },
];

let lin_module_name = "/drivers/lin/lin";

let lin_module = {
    displayName: "LIN",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_init: "/drivers/lin/templates/lin.c.xdt",
            moduleName: lin_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/lin/templates/lin.h.xdt",
            moduleName: lin_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: lin_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: lin_module_name,
        },
    },
    defaultInstanceName: "CONFIG_LIN",
    validate: validate,
    modules: function(instance) {
        return [{
            name: "system_common",
            moduleName: "/system_common",
        }]
    },
    getInstanceConfig,
    pinmuxRequirements,
    getInterfaceName,
    getPeripheralPinNames,
    getClockEnableIds,
    getClockFrequencies,
    config: config,
    onMigrate,
};

function onMigrate(newInst, oldInst, oldSystem) {
    let pins = getPeripheralPinNames(oldInst)
    let interfaceName = getInterfaceName(oldInst)
    common.onMigrate(newInst, oldInst, oldSystem, pins, interfaceName)
}

exports = lin_module;
