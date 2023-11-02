// This module shows the basic configurable types that are supported for ADC.
//
let common = system.getScript("/common");

const baseDirName = "/pru_io/adc/"

/*
    ADCs List with reference to their implementation syscfg files
*/
const adcList = {
    "ADS8598H": {
        "AM64xAdapterBoard": baseDirName + "ads85x8/adc_phi_pru_evm_adapter",
        "TandMAdapterBoard": baseDirName + "ads85x8/t_and_m_adapter",
    },
    "ADS8598S": {
        "AM64xAdapterBoard": baseDirName + "ads85x8/adc_phi_pru_evm_adapter",
        "TandMAdapterBoard": baseDirName + "ads85x8/t_and_m_adapter",
    },
    "ADS8588H": {
        "AM64xAdapterBoard": baseDirName + "ads85x8/adc_phi_pru_evm_adapter",
        "TandMAdapterBoard": baseDirName + "ads85x8/t_and_m_adapter",
    },
    "ADS8588S": {
        "AM64xAdapterBoard": baseDirName + "ads85x8/adc_phi_pru_evm_adapter",
        "TandMAdapterBoard": baseDirName + "ads85x8/t_and_m_adapter",
    },
    "ADS127L11": {
        "AM64xAdapterBoard": baseDirName + "ads127/adc_phi_pru_evm_adapter",
    },
    "ADS131M08": {
        "AM64xAdapterBoard": baseDirName + "ads131/adc_phi_pru_evm_adapter",
    },
}

function modifyAdcConfigOptions(inst) {
    for (let adc of Object.keys(adcList)) {
        for (option of Object.keys(adcList[inst.adcIC])) {
            inst.adaptor.readOnly = true;
        };
    };
}

function getInstanceConfig(moduleInstance) {
    return {
        ...moduleInstance
    }
}

function onValidate(inst, report) {
    /* None. Verified by SYSCFG based on selected pin */
}

function getSubmodulePath(instance) {
    return adcList[`${instance.adcIC}`][`${instance.adaptor}`];
}

function getAdcOptions() {
    let options = [];
    for (let adc of Object.keys(adcList)){
        let option = { name: adc };
        options.push(option);
    }
    return options;
}

function getDisabledAdapterOptions(instance) {
    return [];
}

function moduleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "adcConfig",
        displayName: "ADC Configuration",
        moduleName: getSubmodulePath(instance),
        useArray: true,
        minInstanceCount : 1,
        defaultInstanceCount: 1,
        maxInstanceCount : 1,
        requiredArgs: {
            icssInstance: instance.icssInstance,
        },
        collapsed: false,
    });

    return (modInstances);
}

function getAdcPruCore(instance) {
    return  instance.adcConfig.PRU_ICSSG0_PRU?.$solution.peripheralName.substring(11) ||
            instance.adcConfig.PRU_ICSSG1_PRU?.$solution.peripheralName.substring(11);
}

let adc_top_module_name = "/pru_io/adc/adc";

let adc_top_module = {
    displayName: "ADC",

    templates: {
        "/pru_io/common/pru_io_config.inc.xdt": {
            pru_io_config: "/pru_io/adc/templates/adc_config.inc.xdt",
            moduleName: adc_top_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/pru_io/adc/templates/pru_adc.h.xdt",
            moduleName: adc_top_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ADC",
    config: [
        {
            name: "adcIC",
            displayName: "ADC IC",
            options: getAdcOptions(),
            default: "ADS8598H",
            onChange: (inst, ui) => {
                if(inst.adcIC === "ADS127L11" || inst.adcIC === "ADS131M08"){
                    inst.adaptor = "AM64xAdapterBoard";
                    ui.adaptor.readOnly = true;
                } else {
                    ui.adaptor.readOnly = false;
                }
            },
        },
        {
            name: "adaptor",    // change it to "adapter"
            displayName: "Adapter Card",
            options: [{
                name: "AM64xAdapterBoard",
                displayName: "ADC-PHI-PRU-EVM Adapter Board",
            },
            {
                name: "TandMAdapterBoard",
                displayName: "T&M SEM Adapter Board (Obsolete)",
                description: "Adapter Board with both ADC and DAC interface options",
            }],
            default: "AM64xAdapterBoard",
            getDisabledOptions: getDisabledAdapterOptions,
                // TODO: As we add more adcs, remove TandMAdapterBoard option
        },
        {
            name: "icssInstance",
            displayName: "ICSSG Instance",
            default: "ICSSG0",
            options: [{
                name: "ICSSG0",
            },
            {
                name: "ICSSG1",
            }],
            getDisabledOptions: () => {
                return [{
                    name: "ICSSG1",
                    reason: "The Adapter Board only supports ICSSG0"
                }]
            },
        },
    ],
    validate: onValidate,
    moduleInstances,
    getInstanceConfig,
};

exports = adc_top_module;