let config = [
    {
        name: "start_symbol",
        displayName: "Start Symbol",
        description: "This field is optional. Add start symbol.",
        default:"",
        hidden: true,
    },
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
    },
    {
        name: "end_symbol",
        displayName: "End Symbol",
        description: "This field is optional. Add end symbol.",
        default:"",
        hidden: true,
    },
    {
        name: "additional_data",
        displayName: "Additional Data",
        description: "This field is optional. Add text to be written in the linker.cmd. Press enter for next line.",
        multiline: true,
        default:"",
    }
]

exports = {
    defaultInstanceName: "CONFIG_INPUT_SECTION",
	displayName: "INPUT SECTION",
	config: config,
}