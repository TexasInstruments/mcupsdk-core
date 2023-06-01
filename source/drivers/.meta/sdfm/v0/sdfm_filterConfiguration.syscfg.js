let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}.syscfg.js`);
let useComparator = system.getScript("/drivers/sdfm/v0/sdfm_comparatorFilter.syscfg.js");
let useDatafilter = system.getScript("/drivers/sdfm/v0/sdfm_dataFilter.syscfg.js");
let sd_definition = system.getScript("/drivers/sdfm/v0/sdfm_math.syscfg.js");

// SDModulator settings //
function SDmodulatorSettings(channel)
{
	let Settings = [];

	density_1s = (inst) => {
                let temp= (inst["Ch" + channel.toString() + "_DC_Input"] + inst["Ch" + channel.toString() + "_Vclipping"]) / (2 * inst["Ch" + channel.toString() + "_Vclipping"]);
                return temp;}

	Theoretical_Data_Filter = (inst) => {
				let temp = Math.floor((Math.abs(inst["Ch" + channel.toString() + "_DC_Input"]) / inst["Ch" + channel.toString() + "_Vclipping"])*sd_definition.getShiftvalue(inst["Ch" + channel.toString() + "_FilterType_D"], inst["Ch" + channel.toString() + "_DOSR"], 0 , inst["Ch" + channel.toString() + "_DataFilter_Representation"]));
				if(inst["Ch" + channel.toString() + "_DC_Input"] < 0)
				{
					temp = -1 * temp;
				}
				return temp;}

	Theoretical_Comparator_Filter = (inst) => {
				let temp = (inst["Ch" + channel.toString() + "_DC_Input"] + inst["Ch" + channel.toString() + "_Vclipping"]) / (2 * inst["Ch" + channel.toString() + "_Vclipping"]) * sd_definition.getShiftvalue(inst["Ch" + channel.toString() + "_FilterType"], inst["Ch" + channel.toString() + "_COSR"], 0 , "SDFM_DATA_FORMAT_32_BIT");
				return temp;}

	Settings =
	[
		{
			name: "Ch" + channel.toString() + "_SD_modulatorFrequency",
			displayName : "SD Modulator Frequency (MHz)",
			description : 'Select SD Modulator Frequency in MHz',
			hidden      : true,
			default     : 20,
		},
		{
			name        : "Ch" + channel.toString() + "_Vclipping",
			displayName : "Differential Clipping Voltage (V)",
			description : 'What is the full scale range of SD modulator',
			hidden      : true,
			default     : 0.32,
		},
		{
			name        : "Ch" + channel.toString() + "_DC_Input",
			displayName : "DC Input To SD-modulator (V)",
			description : 'What is the DC input to SD-modulator',
			hidden      : true,
			default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_bitstream_1s_density",
			displayName : "Bitstream 1's Density",
			description : 'Density of 1s in SD modulated bitstream',
			hidden      : true,
 		    getValue    : density_1s,
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Theoritical_DataFilterOutput",
			displayName : "Theoritical Data Filter Output",
			description : 'Theoritical Data filter Output',
			hidden      : true,
			getValue    : Theoretical_Data_Filter,
		    default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Theoritical_ComparatorFilterOutput",
			displayName : "Theoritical Comparator Filter Output",
			description : 'Theoritical Comparator filter Output',
			hidden      : true,
			getValue    : Theoretical_Comparator_Filter,
		    default     : 0,
		},

	]
	return(Settings);
}

function fill_filter_array(channel)
{
	let config =
		[
			{
				name: "Ch" + channel.toString() + "_Mode",
				displayName : "SD Modulator Mode",
				description : 'SD Modulator mode',
				hidden      : true,
				default     : device_peripheral.SDFM_ModulatorClockMode[0].name,
				options     : device_peripheral.SDFM_ModulatorClockMode,
			},
			{
				name: "Ch" + channel.toString() + "_DataFilterEnable",
				displayName : "Enable Data Filter",
				description : 'Enable / Disable Data Filter',
				hidden      : true,
				default     : false,
				onChange	: useDatafilter.onChangeDataFilterEnable
			},
            {
                name: "Ch" + channel.toString() + "_ComparatorEnable",
                displayName : "Enable Comparator",
                description : 'Enable Comparator',
                hidden      : true,
                default     : false,
                onChange    : useComparator.onChangeComparatorEnable
            },
			{
				name        : "GROUP_ComparatorFilter",
				displayName : "ComparatorFilter Settings",
				config      : useComparator.comparatorSettings(channel)
			},
			{
				name        : "GROUP_DataFilter",
				displayName : "DataFilter Settings",
				config      : useDatafilter.datafilterSettings(channel)
			},
			{
				name        : "GROUP_SDMod",
				displayName : "SD_modulator_Settings",
				collapsed	: false,
				config      : SDmodulatorSettings(channel)
			},

        ]
	return(config);
}

let filterConfigs = [];
let density_1s = 0;
let Theoretical_Data_Filter = 0;
let Theoretical_Comparator_Filter = 0;

for (let channel = 1; channel <= 4; channel++)
{
	filterConfigs = filterConfigs.concat
	(
	  [
	   {
        name: "GROUP_FILTER" + channel.toString(),
        displayName: "Filter" + channel.toString(),
        description: "Configure Filter" + channel.toString(),
		collapsed	: true,
        longDescription: "",
        config: fill_filter_array(channel),
	   }
	  ]
	);
}
useComparator.onChangeComparatorEnable

let sdfmFilterConfig = {
    displayName: "SDFM Filter Configuration",
    defaultInstanceName: "SDFM_FilterConfig",
    description: "Sigma Delta Filter Modulator Filter Configuration",
    config: filterConfigs,
    templates: {
        boardc : "",
        boardh : ""
    },
};

exports = sdfmFilterConfig