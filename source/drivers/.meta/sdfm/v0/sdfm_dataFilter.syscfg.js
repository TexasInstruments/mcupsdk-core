let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}.syscfg.js`);
let sd_definition = system.getScript("/drivers/sdfm/v0/sdfm_math.syscfg.js");

let FIFO_LVL =
 [
	{name: "0",   displayName: "FIFO_EMPTY"},
	{name: "1",   displayName: "FIFO_LVL_1"},
	{name: "2",   displayName: "FIFO_LVL_2"},
	{name: "3",   displayName: "FIFO_LVL_3"},
	{name: "4",   displayName: "FIFO_LVL_4"},
	{name: "5",   displayName: "FIFO_LVL_5"},
	{name: "6",   displayName: "FIFO_LVL_6"},
	{name: "7",   displayName: "FIFO_LVL_7"},
	{name: "8",   displayName: "FIFO_LVL_8"},
	{name: "9",   displayName: "FIFO_LVL_9"},
	{name: "10",   displayName: "FIFO_LVL_10"},
	{name: "11",   displayName: "FIFO_LVL_11"},
	{name: "12",   displayName: "FIFO_LVL_12"},
	{name: "13",   displayName: "FIFO_LVL_13"},
	{name: "14",   displayName: "FIFO_LVL_14"},
	{name: "15",   displayName: "FIFO_LVL_15"},
	{name: "16",   displayName: "FIFO_LVL_16"},
 ];

 function onChangeDataFilterEnable(inst, ui)
 {
	for (let channel = 1; channel <= 4; channel++)
	{
		let status = inst["Ch" + channel.toString() + "_DataFilterEnable"];

		ui["Ch" + channel.toString() + "_FilterType_D"].hidden = !status;
		ui["Ch" + channel.toString() + "_DOSR"].hidden = !status;
		ui["Ch" + channel.toString() + "_Min_FilterOutput"].hidden = !status;
		ui["Ch" + channel.toString() + "_Max_FilterOutput"].hidden = !status;

		ui["Ch" + channel.toString() + "_Datarate_DF"].hidden = !status;
		ui["Ch" + channel.toString() + "_Latency_DF"].hidden = !status;

		ui["Ch" + channel.toString() + "_DataFilter_Representation"].hidden = !status;
		ui["Ch" + channel.toString() + "_ShiftControl"].hidden = !status;

		ui["Ch" + channel.toString() + "_SDSYNC_Enable"].hidden = !status;

		ui["Ch" + channel.toString() + "_FIFO_Enable"].hidden = !status;

	}
 }


 function onChangeUseFIFOSettings(inst, ui)
 {
	for (let channel = 1; channel <= 4; channel++)
	{

        let status_FIFO = inst["Ch" + channel.toString() + "_FIFO_Enable"];

        ui["Ch" + channel.toString() + "_SDFFIL"].hidden = !status_FIFO;
        ui["Ch" + channel.toString() + "_FFSYNCCLREN"].hidden = !status_FIFO;

        let status_SyncEn = inst["Ch" + channel.toString() + "_SDSYNC_Enable"];

        ui["Ch" + channel.toString() + "_SDSYNC_source"].hidden = !status_SyncEn;
        ui["Ch" + channel.toString() + "_WTSYNCEN"].hidden = true;
        ui["Ch" + channel.toString() + "_WTSCLREN"].hidden = !status_SyncEn;

	}

	let status_use_int = inst.useInterrupts
	if(inst.useInterrupts)
	{
		let status_FIFOEn = (inst.Ch1_FIFO_Enable || inst.Ch2_FIFO_Enable || inst.Ch3_FIFO_Enable || inst.Ch4_FIFO_Enable);
		ui.SDFFINT.hidden = ((!status_FIFOEn) || (!status_use_int));
		ui.SDFFOVF.hidden = ((!status_FIFOEn) || (!status_use_int));
		ui.AE.hidden = !status_use_int;
	}
 }



function iterate(item)
{
  let settings = [];
  for(let i = 0; i< item.length; i++)
  {
  	settings =  settings.concat((({ name, displayName }) => ({ name, displayName }))(item[i]));
  	//console.log((({ name, displayName }) => ({ name, displayName }))(item[i]));
  }
  	return settings;
}

let rr_temp = iterate(device_peripheral.SDFM_FilterType);

// Data Filter settings //
function datafilterSettings(channel)
{
	let Settings = [];
	let order = 0;

	Settings =
	[
		{
			name: "Ch" + channel.toString() + "_FilterType_D",
			displayName : "Filter Type",
			description : 'Select FilterType for Data Filter',
			hidden      : true,
			default     : iterate(device_peripheral.SDFM_FilterType)[3].name,
			options     : iterate(device_peripheral.SDFM_FilterType),
		},
		{
			name: "Ch" + channel.toString() + "_DOSR",
			displayName : "DOSR",
			description : 'Select OSR for Data Filter (1 to 256)',
			hidden      : true,
			default     : 256,
		},
		{
			name        : "Ch" + channel.toString() + "_Datarate_DF",
			displayName : "Data Rate (us)",
			description : 'Data rate (Data filter) for given DOSR setting and SDmodulator rate',
			hidden      : true,
		    getValue    : (inst) => {
                let datarate = inst["Ch" + channel.toString() + "_DOSR"] / inst["Ch" + channel.toString() + "_SD_modulatorFrequency"];
                return datarate;
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Latency_DF",
			displayName : "Latency (us)",
			description : 'Latency (or) settling time (Data filter) for given DOSR setting and SDmodulator rate',
			hidden      : true,
		    getValue    : (inst) => {
				if(inst["Ch" + channel.toString() + "_FilterType_D"] == "SDFM_FILTER_SINC_FAST" || inst["Ch" + channel.toString() + "_FilterType_D"] == "SDFM_FILTER_SINC_3")
				{
					order = 3;
				}
				if(inst["Ch" + channel.toString() + "_FilterType_D"] == "SDFM_FILTER_SINC_2")
				{
					order = 2;
				}
				if(inst["Ch" + channel.toString() + "_FilterType_D"] == "SDFM_FILTER_SINC_1")
				{
					order = 1;
				}
                let latency = order * inst["Ch" + channel.toString() + "_DOSR"] / inst["Ch" + channel.toString() + "_SD_modulatorFrequency"];
                return latency;
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Min_FilterOutput",
			displayName : "Min (Data Filter Output)",
			description : 'Minimum Data Filter output for given DOSR setting',
			hidden      : true,
		    getValue    : (inst) => {
                let min_value = -1*sd_definition.getShiftvalue(inst["Ch" + channel.toString() + "_FilterType_D"], inst["Ch" + channel.toString() + "_DOSR"], 0 , inst["Ch" + channel.toString() + "_DataFilter_Representation"]);
                return min_value;
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Max_FilterOutput",
			displayName : "Max (Data Filter Output)",
			description : 'Maximum Data Filter output for given DOSR setting',
			hidden      : true,
		    getValue    : (inst) => {
                let max_value = sd_definition.getShiftvalue(inst["Ch" + channel.toString() + "_FilterType_D"], inst["Ch" + channel.toString() + "_DOSR"], 0 , inst["Ch" + channel.toString() + "_DataFilter_Representation"]);
				return max_value;
            },
            default     : 0,
		},
		{
			name: "Ch" + channel.toString() + "_DataFilter_Representation",
			displayName : "Data Filter Output Representation",
			description : '16-bit (or) 32-bit representation',
			hidden		: true,
			default     : device_peripheral.SDFM_OutputDataFormat[1].name,
			options		: device_peripheral.SDFM_OutputDataFormat,
		},
		{
			name: "Ch" + channel.toString() + "_ShiftControl",
			displayName : "Shift 32-bit Filter Output By X Bits",
			description : 'Shift 32-bit filter output by x bits',
			hidden		: true,
			getValue    : (inst) => {
                let shiftvalue = sd_definition.getShiftvalue(inst["Ch" + channel.toString() + "_FilterType_D"], inst["Ch" + channel.toString() + "_DOSR"], 1 , inst["Ch" + channel.toString() + "_DataFilter_Representation"]);
				return shiftvalue;
            },
            default     : 0,
		},
		{
			name: "Ch" + channel.toString() + "_SDSYNC_Enable",
			displayName : "Use PWM Synchronization",
			description : 'Use PWM synchronization',
			hidden		: true,
			default     : false,
			onChange	: onChangeUseFIFOSettings,
		},
	]


Settings = Settings.concat
(
    [
        {
            name        : "GROUP_SDSYNC_settings" + channel.toString() +"",
            displayName : "SDSYNC Feature Settings",
            collapsed	: false,
            config      :
            [
                {
                name: "Ch" + channel.toString() + "_SDSYNC_source",
                displayName : "Source Of SDSYNC Event",
                description : 'Choose PWM source for SDSYNC event',
                hidden		: true,
                default     : device_peripheral.SDFM_PWMSyncSource[0].name,
                options		: device_peripheral.SDFM_PWMSyncSource
                },
                {
                name: "Ch" + channel.toString() + "_WTSYNCEN",
                displayName : "Enable Wait For Sync Feature",
                description : 'FIFO gets written on every data ready event (or) SDSYNC event',
                hidden      : true,
                default     : false,
                },
                {
                name: "Ch" + channel.toString() + "_WTSCLREN",
                displayName : "Clear Wait For Sync Flag Manually",
                description : 'When enabled, wait for sync flag is manually cleared',
                hidden      : true,
                default		: false,
                },
            ]
        },
        {
            name: "Ch" + channel.toString() + "_FIFO_Enable",
            displayName : "Use FIFO",
            description : 'Use FIFO',
            hidden		: true,
            default     : false,
            onChange	: onChangeUseFIFOSettings,
        },
        {
            name        : "GROUP_FIFO_settings" + channel.toString() +"",
            displayName : "FIFO Settings",
            collapsed	: false,
            config      :
            [
                {
                name: "Ch" + channel.toString() + "_SDFFIL",
                displayName : "SDFIFO Interrupt Level (SDFFIL)",
                description : 'FIFO will generate interrupt when FIFO has more (or) equal contents than SDFFIL',
                hidden      : true,
                default		: FIFO_LVL[0].name,
                options		: FIFO_LVL,
                },
                {
                name: "Ch" + channel.toString() + "_FFSYNCCLREN",
                displayName : "Clear FIFO On SDSYNC Event",
                description : 'When enabled, SDSYNC event (from PWM) can be clear the FIFO',
                hidden      : true,
                default		: false,
                },
            ]
        },
    ]
)


	return(Settings);
}



// Data Filter settings //
exports =
{
    datafilterSettings : datafilterSettings,
	onChangeDataFilterEnable : onChangeDataFilterEnable,
}
