let common   = system.getScript("/common");

let useComparator = system.getScript("/drivers/sdfm/v0/sdfm_comparatorFilter.syscfg.js");
let useDatafilter = system.getScript("/drivers/sdfm/v0/sdfm_dataFilter.syscfg.js");
let device_peripheral = system.getScript(`/drivers/sdfm/soc/sdfm_${common.getSocName()}.syscfg.js`);

let CompEvent =
[
	{ name: "CEVT1"   , displayName: "Comparator Event 1 Interrupt" },
	{ name: "CEVT2"   , displayName: "Comparator Event 2 Interrupt" },
]

let IEL_IEHEvent =
[
	{ name: "IEL"   , displayName: "Low Threshold Interrupt" },
	{ name: "IEH"   , displayName: "High Threshold Interrupt" },
]


let temp1 	 = CompEvent[0];
let temp2    = CompEvent[1];



let FilterNumber = [];

function onChangeUseInterrupts(inst, ui)
{
	let status_use_int = inst.useInterrupts

	let status_FIFO_EN = (inst.Ch1_FIFO_Enable || inst.Ch2_FIFO_Enable || inst.Ch3_FIFO_Enable || inst.Ch4_FIFO_Enable);

	let status_compEN;

	status_compEN = (inst.Ch1_ComparatorEnable || inst.Ch2_ComparatorEnable || inst.Ch3_ComparatorEnable || inst.Ch4_ComparatorEnable);

	ui.MFIE.hidden = !status_use_int;
	ui.AE.hidden = !status_use_int;


    ui.CEVT1.hidden = ((!status_use_int) || (!status_compEN));
    ui.CEVT2.hidden = ((!status_use_int) || (!status_compEN));
    ui.SDFFINT.hidden = ((!status_FIFO_EN) || (!status_use_int));
    ui.SDFFOVF.hidden = ((!status_FIFO_EN) || (!status_use_int));

}

let interruptSettings_Type0 = [];

let intSettings = [];

let interruptSettings_SDFM_INT = [];
let interruptSettings_SDFM_DR_INT = [];

interruptSettings_SDFM_INT = interruptSettings_SDFM_INT.concat([
    {
    name        : "MFIE",
    displayName : "Modulator Clock Failure",
    description : 'Which interrupts to enabled.',
    hidden      : true,
    default     : [],
    minSelections: 0,
    options     : device_peripheral.SDFM_FilterNumber,
    },
    {
    name        : temp1.name,
    displayName : temp1.displayName,
    description : 'Which Interrupts To Enabled.',
    hidden      : true,
    default     : [],
    minSelections: 0,
    options     : device_peripheral.SDFM_FilterNumber,
    },
    {
    name        : temp2.name,
    displayName : temp2.displayName,
    description : 'Which Interrupts To Enabled.',
    hidden      : true,
    default     : [],
    minSelections: 0,
    options     : device_peripheral.SDFM_FilterNumber,
    },
    {
    name        : "SDFFOVF",
    displayName : "FIFO Overflow Error",
    description : 'Which interrupts to enabled.',
    hidden      : true,
    default     : [],
    minSelections: 0,
    options     : device_peripheral.SDFM_FilterNumber,
    },
])
    intSettings = interruptSettings_SDFM_INT;

    interruptSettings_SDFM_DR_INT = interruptSettings_SDFM_DR_INT.concat([
    {
    name        : "AE",
    displayName : "Data Acknowledge",
    description : 'Which interrupts to enabled.',
    hidden      : true,
    default     : [],
    minSelections: 0,
    options     : device_peripheral.SDFM_FilterNumber,
    },
    {
    name        : "SDFFINT",
    displayName : "SDFM FIFO Interrupt",
    description : 'Which interrupts to enabled.',
    hidden      : true,
    default     : [],
    minSelections: 0,
    options     : device_peripheral.SDFM_FilterNumber,
    },
    ])


let interruptSettings = [];

	interruptSettings =
	[
		{
		name		: "useInterrupts",
		displayName : "Use SDFM Interrupts",
		description : 'Whether or not to use Interrupt mode.',
        hidden      : false,
		default     : false,
		onChange    : onChangeUseInterrupts,
		},
		{
        name: "GROUP_SDFM_INT",
        displayName: "SDFM Interrupt",
        description: "Configure SDFM interrupt",
		collapsed	: false,
        longDescription: "",
        config: intSettings,
		},
		{
        name: "GROUP_SDFM_DR_INT",
        displayName: "SDFM Data Ready Interrupt",
        description: "Configure SDFM data ready interrupt",
		collapsed	: false,
        longDescription: "",
        config: interruptSettings_SDFM_DR_INT,
		},
	]


let sdInterruptConfigSubmodule =
{
    displayName: "SDFM Interrupt Configurations",
    defaultInstanceName: "SDFM_InterruptConfig",
	description: "Select Interrupt source",
    config: interruptSettings,
    templates:
	{
        boardc : "",
        boardh : "",
    },
};

exports = sdInterruptConfigSubmodule







