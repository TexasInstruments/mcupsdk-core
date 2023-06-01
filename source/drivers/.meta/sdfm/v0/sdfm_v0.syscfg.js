let common   = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}`);
let numberOfSDFMs = system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}`).numberOfSDFMs;
let device_peripheral = system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}.syscfg.js`);
let sd_clk_config = system.getScript("/drivers/sdfm/v0/sdfm_clockConfiguration.syscfg.js");
let sd_validate = system.getScript("/drivers/sdfm/v0/sdfm_validation.syscfg.js");

function getStaticConfigArr() {
    return system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}`).getStaticConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let solution = moduleInstance[getInterfaceName(moduleInstance)].$solution;
    let staticConfigArr = getStaticConfigArr();
    let staticConfig = staticConfigArr.find( o => o.name === solution.peripheralName);

    return {
        ...staticConfig,
        ...moduleInstance
    }
}

/* Array of SDFM configurables that are common across device families */
let config = [];

for(let channel = 1; channel <= 4; channel++)
{
config = config.concat
		(
		   [
		     {
				name: "Use_FilterChannel_" + channel.toString(),
				displayName : "Use Filter Channel " + channel.toString(),
				description : 'Enable / Disable Filter Channel',
				hidden      : false,
				default		: false,
				onChange	: onChangeUseFilterChannel,
			 },
		   ]
		)
}

// config = config.concat
// 	(
// 	  [
// 	    {
// 			name: "GROUP_CLOCK",
// 			displayName: "SDCLK Source Configuration",
// 			description: "Select SDCLK for respective filter channels",
// 			collapsed	: false,
// 			longDescription: "",
// 			config: sd_clk_config.config,
// 		},
// 	  ]
// 	)

function onChangeUseFilterChannel(inst, ui)
{
	for (let channel = 1; channel <= 4; channel++)
	{
		let status = inst["Use_FilterChannel_" + channel.toString()];

		ui["Ch" + channel.toString() + "_SDCLKSEL"].hidden = !status;
		ui["Ch" + channel.toString() + "_Mode"].hidden = !status;
		ui["Ch" + channel.toString() + "_ComparatorEnable"].hidden = !status;
		ui["Ch" + channel.toString() + "_DataFilterEnable"].hidden = !status;
		ui["Ch" + channel.toString() + "_SD_modulatorFrequency"].hidden = !status;
		ui["Ch" + channel.toString() + "_Vclipping"].hidden = !status;
		ui["Ch" + channel.toString() + "_DC_Input"].hidden = !status;
		ui["Ch" + channel.toString() + "_bitstream_1s_density"].hidden = !status;
		ui["Ch" + channel.toString() + "_Theoritical_DataFilterOutput"].hidden = !status;
		ui["Ch" + channel.toString() + "_Theoritical_ComparatorFilterOutput"].hidden = !status;
	}
}

/*
 *  ======== filterHardware ========
 *  Control RX, TX Pin usage by the user specified dataDirection.
 *
 *  param component - hardware object describing signals and
 *                     resources they're attached to
 *
 *  returns Boolean indicating whether or not to allow the component to
 *           be assigned to an instance's $hardware config
 */
function filterHardware(component)
{
    return (common.typeMatches(component.type, ["SDFM"]));
}

function getInterfaceName(inst)
{
    return soc.getInterfaceName(inst);
}

function getPeripheralPinNames(inst)
{
    return [ "CLK0", "CLK1", "CLK2", "CLK3", "D0", "D1", "D2", "D3" ];
}

function getClockEnableIds(inst) {
    let instConfig = getInstanceConfig(inst);
    return instConfig.clockIds;
}

function getClockFrequencies(inst) {

    let instConfig = getInstanceConfig(inst);

    return instConfig.clockFrequencies;
}

function pinmuxRequirements(inst)
{
   let interfaceName = getInterfaceName(inst);

    let resources = [];
    resources.push( pinmux.getPinRequirements(interfaceName, "CLK0", "SDFM CLK Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "CLK1", "SDFM CLK Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "CLK2", "SDFM CLK Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "CLK3", "SDFM CLK Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "D0", "SDFM Data Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "D1", "SDFM Data Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "D2", "SDFM Data Pin"));
    resources.push( pinmux.getPinRequirements(interfaceName, "D3", "SDFM Data Pin"));

    let peripheral = {
        name: interfaceName,
        displayName: "SDFM Instance",
        interfaceName: interfaceName,
        resources: resources,
    };

    return [peripheral];
}

let submodulesComponents = [
    {
        moduleName: "/drivers/sdfm/v0/sdfm_clockConfiguration.syscfg.js",
        name: "sdfmClockConfiguration",
        displayName:"SDFM Clock Configuration",
        description:"SDFM Clock Configuration",
    },
    {
        moduleName: "/drivers/sdfm/v0/sdfm_filterConfiguration.syscfg.js",
        name: "SDFM_FilterConfig",
        displayName:"SDFM Filter Configuration",
        description:"SDFM Filter Configuration",
    },
    {
        moduleName: "/drivers/sdfm/v0/sdfm_interrupt.syscfg.js",
        name: "SDFM_InterruptConfig",
        displayName:"SDFM Interrupt Configurations",
        description:"Select Interrupt source",
    }
]

for (let submoduleComponent of submodulesComponents)
{
    let submodule = system.getScript(submoduleComponent.moduleName)
    config = config.concat([
        {
            name: "GROUP_" + submodule.defaultInstanceName,
            displayName: submodule.displayName,
            longDescription: submodule.description,
            description: "",
            config: submodule.config
        },
    ])
}

let sd_module_name = "/drivers/sdfm/sdfm";

let sdModule = {
    peripheralName: "SDFM",
    displayName: "SDFM",
    defaultInstanceName: "CONFIG_SDFM",
    maxInstances: numberOfSDFMs,
    description: "Sigma Delta Filter Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/drivers/sdfm/templates/sdfm.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/drivers/sdfm/templates/sdfm_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/drivers/sdfm/templates/sdfm_open_close_config.c.xdt",
            driver_open: "/drivers/sdfm/templates/sdfm_open.c.xdt",
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: sd_module_name,
        },
        "/drivers/system/power_clock_config.c.xdt": {
            moduleName: sd_module_name,
        },
    },
    validate : sd_validate.onValidate,
    getInstanceConfig,
    getInterfaceName,
    getPeripheralPinNames,
    pinmuxRequirements,
    getClockEnableIds,
    getClockFrequencies,
};

exports = sdModule;