let config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
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