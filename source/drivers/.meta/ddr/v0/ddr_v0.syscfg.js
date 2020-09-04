
let common = system.getScript("/common");
let soc = system.getScript(`/drivers/ddr/soc/ddr_${common.getSocName()}`);

let ddrSize = 0x80000000;

function getInstanceConfig(moduleInstance) {

    return {
        ...moduleInstance,
    };
};

function getClockEnableIds(instance) {
    let staticConfig = soc.getStaticConfig();
    return staticConfig.clockIds;
}

let ddr_module = {
    displayName: "DDR",

    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/drivers/ddr/templates/ddr_config.c.xdt",
            driver_init: "/drivers/ddr/templates/ddr_init.c.xdt",
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/ddr/templates/ddr.h.xdt",
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: "/drivers/ddr/ddr",
        },
    },
    maxInstances: 1,
    defaultInstanceName: "CONFIG_DDR",
    config: [
        {
            name: "ddrConfigIncludeFileName",
            displayName: "DDR Configuration File",
            skipTests: [ "configLongDescription" ], /* skipping due to intermittent 502 errors when checking the weblink in long description */
            longDescription:
`
The file specified here is generated using the DDR SUBSYSTEM REGISTER CONFIGURATION tool.

IMPORTANT NOTES,
- A default DDR config file for TI EVM is pre-generated and used as default value.
- You can override this with your own EVM specific file.

To generate a DDR config file for your EVM
- Goto https://dev.ti.com/sysconfig
- Select product as "DDR SUBSYSTEM REGISTER CONFIGURATION"
- Select the device of interest ( ${common.getSocName()} )
- Configure the DDR parameters using the tool
- Save the generated board_ddrReginit.h in your workspace
- Specify the path to this file including the filename in this text box
- Make sure to use forward slash "/" in the file path so that this will work with linux as well as windows build
- Make sure that path to this is file set in your application include path, as needed.
`,
            default: soc.getDefaultDdrConfigFileName(),
        },
        {
            name: "eccEnableFlag",
            displayName: "Enable Inline ECC",
            longDescription:
`
IMPORTANT NOTES
- 3 Regions can be specified to enable inline ECC.
- The address (ECC start and end) is specified as an offset from the DDR region start
- If start address is greater than end address ECC in this range will be disabled
- 1/9 the size of memory region for which ECC is enabled will be unusable at the end of DDR
`,
            default: false,
            onChange: function (inst, ui) {
                let hideEccAddr = true;
                if (inst.eccEnableFlag == true)
                {
                    hideEccAddr = false;
                }
                ui.eccStart0.hidden = hideEccAddr;
                ui.eccEnd0.hidden = hideEccAddr;
                ui.eccStart1.hidden = hideEccAddr;
                ui.eccEnd1.hidden = hideEccAddr;
                ui.eccStart2.hidden = hideEccAddr;
                ui.eccEnd2.hidden = hideEccAddr;
            }
        },
        {
            name: "eccStart0",
            displayName: "ECC Region 0 Start Address",
            default: 0x00000000,
            hidden: true,
            displayFormat: "hex",
        },
        {
            name: "eccEnd0",
            displayName: "ECC Region 0 End Address",
            default: 0x00000000,
            hidden: true,
            displayFormat: "hex",
        },
        {
            name: "eccStart1",
            displayName: "ECC Region 1 Start Address",
            default: 0x00000000,
            hidden: true,
            displayFormat: "hex",
        },
        {
            name: "eccEnd1",
            displayName: "ECC Region 1 End Address",
            default: 0x00000000,
            hidden: true,
            displayFormat: "hex",
        },
        {
            name: "eccStart2",
            displayName: "ECC Region 2 Start Address",
            default: 0x00000000,
            hidden: true,
            displayFormat: "hex",
        },
        {
            name: "eccEnd2",
            displayName: "ECC Region 2 End Address",
            default: 0x00000000,
            hidden: true,
            displayFormat: "hex",
        },
    ],
    validate: validate,
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    getInstanceConfig,
    getClockEnableIds,
};

function validate(instance, report) {

    let granule_size = 0x10000;

    common.validate.checkNumberRange (instance, report, "eccStart0", 0, ddrSize, "hex");
    common.validate.checkNumberRange (instance, report, "eccEnd0", 0, ddrSize, "hex");

    common.validate.checkNumberRange (instance, report, "eccStart1", 0, ddrSize, "hex");
    common.validate.checkNumberRange (instance, report, "eccEnd1", 0, ddrSize, "hex");

    common.validate.checkNumberRange (instance, report, "eccStart2", 0, ddrSize, "hex");
    common.validate.checkNumberRange (instance, report, "eccEnd2", 0, ddrSize, "hex");

    if ((instance.eccStart0 % granule_size) != 0)
    {
        report.logError( `ECC address start and end must aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
            instance, "eccStart0");
    }

    if ((instance.eccStart1 % granule_size) != 0)
    {
        report.logError( `ECC address start and end must aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
            instance, "eccStart1");
    }

    if ((instance.eccStart2 % granule_size) != 0)
    {
        report.logError( `ECC address start and end must aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
            instance, "eccStart2");
    }

    if ((instance.eccEnd0 % granule_size) != 0)
    {
        report.logError( `ECC address start and end must aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
            instance, "eccEnd0");
    }

    if ((instance.eccEnd1 % granule_size) != 0)
    {
        report.logError( `ECC address start and end must aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
            instance, "eccEnd1");
    }

    if ((instance.eccEnd2 % granule_size) != 0)
    {
        report.logError( `ECC address start and end must aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
            instance, "eccEnd2");
    }
}


exports = ddr_module;
