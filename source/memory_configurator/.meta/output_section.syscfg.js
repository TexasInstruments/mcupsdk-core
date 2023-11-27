let config = [
    {
        name: "$name",
        hidden: false,
        isCIdentifier: false
    },
    {
        name: "include_defines",
        displayName: "Define Macro?",
        default: false,
        onChange: (inst) => {
            inst.$uiState.macro_name.hidden = !inst.include_defines
            let suggestedMacroName = inst.$name.replace(/[^\w\s]/gi, '').toUpperCase()+"_SECTION"
            inst.macro_name = suggestedMacroName
        }
    },
    {
        name: "macro_name",
        displayName:"Macro Name",
        default:"",
        hidden: true
    },
    {
        name: "output_sections_start",
        //multiline: true,
        displayName: "Start Section",
        placeholder: "__IRQ_STACK_START",
        default:"",
        description:'This field is optional',
    },
    {
        name: "output_sections_end",
        //multiline: true,
        displayName: "End Section",
        placeholder: "__IRQ_STACK_END",
        default:"",
        description:'This field is optional',
    },
    {
        name: "alignment",
        displayName: "Alignment",
        default: 8,
        longDescription:'Output section falls on n-byte boundary where n is a power of 2.',
    },
    {
        name: "palignment",
        displayName: "Alignment With Padding",
        default: false,
        longDescription:'In addition to alignment, ensures that size of this section is muliple of\
         its placement alignment restriction, padding the section size up to such a boundary, as needed.',
    },
    {
        name: "fill",
        displayName: "Fill",
        default: 0x0,
        longDescription:'The padded space is filled with this value. By default it is 0.',
    },
]

function validate(inst, report) {

    if(inst.$name.length == 0) {
        report.logError("Output section name can't be kept empty", inst, "$name")
    }

    if(inst.include_defines && inst.macro_name.trim().length == 0){
        report.logError("Macro name can't be kept empty", inst, "macro_name")
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