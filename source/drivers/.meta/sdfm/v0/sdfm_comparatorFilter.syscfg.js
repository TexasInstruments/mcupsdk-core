let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}.syscfg.js`);

function onChangeComparatorEnable(inst, ui)
{
	for (let channel = 1; channel <= 4; channel++)
	{
		let status;
		status = inst["Ch" + channel.toString() + "_ComparatorEnable"];

		ui["Ch" + channel.toString() + "_FilterType"].hidden = !status;
		ui["Ch" + channel.toString() + "_COSR"].hidden = !status;
		ui["Ch" + channel.toString() + "_Datarate_CF"].hidden = !status;
		ui["Ch" + channel.toString() + "_Latency_CF"].hidden  = !status;

        ui["Ch" + channel.toString() + "_HLT1"].hidden = !status;
        ui["Ch" + channel.toString() + "_HLT2"].hidden = !status;
        ui["Ch" + channel.toString() + "_HLTZ_Enable"].hidden = !status;
        ui["Ch" + channel.toString() + "_HLTZ"].hidden = !status;
        ui["Ch" + channel.toString() + "_LLT1"].hidden = !status;
        ui["Ch" + channel.toString() + "_LLT2"].hidden = !status;

        for(let event = 1; event<=2 ;event++)
        {
            ui["Ch" + channel.toString() + "_CEVT" + event.toString() + "_SourceSelect"].hidden = !status;
            ui["Ch" + channel.toString() + "_Use_CEVT" + event.toString() + "_Digital_Filter"].hidden = !status;
        }

	}

    let status_compEN;
	let status_use_int = inst.useInterrupts;
    status_compEN = (inst.Ch1_ComparatorEnable || inst.Ch2_ComparatorEnable || inst.Ch3_ComparatorEnable || inst.Ch4_ComparatorEnable);
    ui.CEVT1.hidden = ((!status_use_int) || (!status_compEN));
	ui.CEVT2.hidden = ((!status_use_int) || (!status_compEN));

}

function onChangeCEVT_Digital_filterSettings(inst, ui)
{
   for (let channel = 1; channel <= 4; channel++)
   {
     for(let event = 1; event <= 2; event++)
     {
       let status = inst["Ch" + channel.toString() + "_Use_CEVT" + event.toString() + "_Digital_Filter"];

       ui["Ch" + channel.toString() + "_InitFilterCEVT" + event.toString() +""].hidden = !status;
       ui["Ch" + channel.toString() + "_SamplePrescale_CEVT" + event.toString() +""].hidden = !status;
       ui["Ch" + channel.toString() + "_Threshold_CEVT" + event.toString() +""].hidden = !status;
       ui["Ch" + channel.toString() + "_SampleWindow_CEVT" + event.toString() +""].hidden = !status;
     }
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

let Settings = [];

// Comparator Filter settings //
function comparatorSettings(channel)
{
let order = 0;
	Settings =
	[
		{
			name: "Ch" + channel.toString() + "_FilterType",
			displayName : "Filter Type",
			description : 'Select FilterType for Comparator Filter',
			hidden      : true,
			default     : iterate(device_peripheral.SDFM_FilterType)[3].name,
			options     : iterate(device_peripheral.SDFM_FilterType),
		},
		{
			name: "Ch" + channel.toString() + "_COSR",
			displayName : "COSR",
			description : 'Select OSR for Comparator Filter (1 to 32)',
			hidden      : true,
			default     : 32,
		},
		{
			name        : "Ch" + channel.toString() + "_Datarate_CF",
			displayName : "Data Rate (us)",
			description : 'Data rate (Data filter) for given DOSR setting and SDmodulator rate',
			hidden      : true,
		    getValue    : (inst) => {
                let datarate = inst["Ch" + channel.toString() + "_COSR"] / inst["Ch" + channel.toString() + "_SD_modulatorFrequency"];
                return datarate;
            },
            default     : 0,
		},
		{
			name        : "Ch" + channel.toString() + "_Latency_CF",
			displayName : "Latency (us)",
			description : 'Latency (or) settling time (Data filter) for given COSR setting and SDmodulator rate',
			hidden      : true,
		    getValue    : (inst) => {
				if(inst["Ch" + channel.toString() + "_FilterType"] == "SDFM_FILTER_SINC_FAST" || inst["Ch" + channel.toString() + "_FilterType"] == "SDFM_FILTER_SINC_3")
				{
					order = 3;
				}
				if(inst["Ch" + channel.toString() + "_FilterType"] == "SDFM_FILTER_SINC_2")
				{
					order = 2;
				}
				if(inst["Ch" + channel.toString() + "_FilterType"] == "SDFM_FILTER_SINC_1")
				{
					order = 1;
				}
                let latency = order * inst["Ch" + channel.toString() + "_COSR"] / inst["Ch" + channel.toString() + "_SD_modulatorFrequency"];
                return latency;
            },
            default     : 0,
		},
        {
            name: "Ch" + channel.toString() + "_HLT1",
            displayName : "High Level Threshold 1",
            description : 'High Threshold Level 1 (0 to 32767)',
            hidden      : true,
            default     : "32767",
        },
        {
            name: "Ch" + channel.toString() + "_HLT2",
            displayName : "High Level Threshold 2",
            description : 'High Threshold Level 2 (0 to 32767)',
            hidden      : true,
            default     : "32767",
        },
        {
            name: "Ch" + channel.toString() + "_HLTZ_Enable",
            displayName : "Use High Threshold (Z)",
            description : 'High Threshold Level Z (0 to 32767)',
            hidden      : true,
            default		: false,
        },
        {
            name: "Ch" + channel.toString() + "_HLTZ",
            displayName : "High Level Threshold (Z)",
            description : 'High Threshold Level (Z) (0 to 32767)',
            hidden      : true,
            default     : "32767",
        },
        {
            name: "Ch" + channel.toString() + "_LLT1",
            displayName : "Low Level Threshold 1",
            description : 'Low Threshold Level 1  (0 to 32767)',
            hidden      : true,
            default     : "0",
        },
        {
            name: "Ch" + channel.toString() + "_LLT2",
            displayName : "Low Level Threshold 2",
            description : 'Low Threshold Level 2  (0 to 32767)',
            hidden      : true,
            default     : "0",
        },
	]

    for(let event = 1; event <= 2; event++)
    {
        let eventSource;

        if(event == 1)
        {
            eventSource = [device_peripheral.SDFM_CompEventSource[0],
                           device_peripheral.SDFM_CompEventSource[1],
                           device_peripheral.SDFM_CompEventSource[2],
                           device_peripheral.SDFM_CompEventSource[3]
                        ];
        }
        if(event == 2)
        {
            eventSource = [device_peripheral.SDFM_CompEventSource[4],
                           device_peripheral.SDFM_CompEventSource[1],
                           device_peripheral.SDFM_CompEventSource[2],
                           device_peripheral.SDFM_CompEventSource[5]
         ];
        }

        Settings = Settings.concat
        (
            [
            {
            name: "Ch" + channel.toString() + "_CEVT" + event.toString() + "_SourceSelect",
            displayName : "Comparator Event" + event.toString() + " SourceSelect",
            description : "Select Comparator Event" + event.toString(),
            hidden      : true,
            default     : eventSource[0].name,
            options		: eventSource,
            },
            {
            name: "Ch" + channel.toString() + "_Use_CEVT" + event.toString() + "_Digital_Filter",
            displayName : "Use CEVT" + event.toString() + " Digital Filter",
            description : "Use CEVT" + event.toString() + " Digital Filter",
            hidden      : true,
            default     : false,
            onChange	: onChangeCEVT_Digital_filterSettings,
            },
            {
            name        : "GROUP" + event.toString() + "_CEVT_Digital Filter" + channel.toString() +"",
            displayName : "CEVT" + event.toString() + " Digital Filter settings",
            collapsed	: false,
            config      :
            [
            {
            name        : "Ch" + channel.toString() + "_InitFilterCEVT" + event.toString() + "",
            displayName : "Initialize Digital Filter (CEVT" + event.toString() + ")",
            description : "Initialize Digital Filter (CEVT" + event.toString() + ")",
            hidden      : true,
            default     : true
            },
            {
            name        : "Ch" + channel.toString() + "_SamplePrescale_CEVT" + event.toString() + "",
            displayName : "CEVT" + event.toString() + " Prescale",
            description : 'The number of system clock cycles between samples.',
            hidden      : true,
            default     : 0
            },
            {
            name        : "Ch" + channel.toString() + "_Threshold_CEVT" + event.toString() + "",
            displayName : "CEVT" + event.toString() + " Threshold",
            description : 'The number of FIFO samples to monitor.',
            hidden      : true,
            default     : 0
            },
            {
            name        : "Ch" + channel.toString() + "_SampleWindow_CEVT" + event.toString() + "",
            displayName : "CEVT" + event.toString() + " Sample Window",
            description : 'The number of FIFO samples to monitor.',
            hidden      : true,
            default     : 0
            },
            ]
            },
            ]
        )
    }


	return(Settings);
}

exports =
{
	comparatorSettings : comparatorSettings,
	onChangeComparatorEnable : onChangeComparatorEnable,
}