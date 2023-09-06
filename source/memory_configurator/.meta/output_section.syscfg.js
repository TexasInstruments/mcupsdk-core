let config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
    },
    {
        name: "output_sections_start",
        //multiline: true,
        displayName: "Start Section",
        placeholder: "__IRQ_STACK_START",
        default:"",
        description:'',
    },
    {
        name: "output_sections_end",
        //multiline: true,
        displayName: "End Section",
        placeholder: "__IRQ_STACK_END",
        default:"",
        description:'',
    },
    {
        name: "alignment",
        displayName: "Alignment",
        default: 8,
        description:'',
    },
]

function validate(inst, report) {

    if(inst.$name.length == 0) {
        report.logError("Output section name can't be kept empty", inst, "$name")
    }
}

exports = {
    defaultInstanceName: "CONFIG_OUTPUT_SECTION",
	displayName: "OUTPUT SECTION",
    moduleInstances: addModuleInstances,
	config: config,
    validate: validate
}

function addModuleInstances() {
    let modInstances = new Array();

    modInstances.push({
        name: "input_section",
        displayName: "Input Sections",
        moduleName: "memory_configurator/input_section",
        useArray: true,
        minInstanceCount: 0,
        collapsed: false,
    });

    return modInstances;
}