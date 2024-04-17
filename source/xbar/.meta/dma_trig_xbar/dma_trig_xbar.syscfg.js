
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/dma_trig_xbar/soc/dma_trig_xbar_${common.getSocName()}`);

function getConfigArr() {
    return soc.getConfigArr();
}

function getInstanceConfig(moduleInstance) {
    let configArr = getConfigArr();
    let config = configArr.find( o => o.name === moduleInstance.instance);

    return {
        ...config,
        ...moduleInstance,
    };
};

let DMA_XBAR_LIST = soc.getOptionList("INTERNAL");
let DMA_XBAR = JSON.parse(JSON.stringify(DMA_XBAR_LIST));
DMA_XBAR.map(function(item) {
    delete item.path;
    return item;
});

let dma_trig_xbar_module = {
    displayName: "DMA TRIGGER XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/dma_trig_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/dma_trig_xbar/templates/dma_trig_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/dma_trig_xbar/templates/dma_trig_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/dma_trig_xbar/templates/dma_trig_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/dma_trig_xbar/templates/dma_trig_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_DMA_TRIG_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: DMA_XBAR[0].name,
            options: DMA_XBAR,
            description: "This determines the output of the xbar",
            hidden: false,
        },
        {
            name: "parentName",
            displayName: "Owned By",
            hidden: true,
            default: "NONE",
            description: "This determines the parent module of the xbar",
            onChange: function (inst,ui){
                if(inst.parentName != "NONE")
                {
                    ui.xbarOutput.hidden = true,
                    ui.parentName.hidden = false
                }
                else
                {
                    ui.xbarOutput.hidden = false,
                    ui.parentName.hidden = true
                }
            },
        },
        common.ui.makeInstanceConfig(getConfigArr()),
    ],
    moduleStatic: {
        modules: function(instance) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    moduleInstances: moduleInstances,
    validate: validate,
    getInstanceConfig,
};

function validate(instance, report) {
    common.validate.checkSameInstanceNameOnAllCores('/xbar/dma_trig_xbar/dma_trig_xbar', instance, report);
    let error_status = false;
    let rangeConfig = [];
    let module = system.modules['/drivers/edma/edma'];
    if(module)
    {
        let edmaInst = module.$instances[0];
        if (edmaInst.enableOwnDmaChannelConfig == true) {
            let rm_instances = edmaInst.edmaRmDmaCh;
            error_status = true;
            for(let rm = 0; rm < rm_instances.length; rm++)
            {
                let rm_instance = rm_instances[rm];
                let instanceName = getInstanceConfig(instance).name;
                let currentChannel = parseInt(instanceName.replace("DMA_TRIG_XBAR_EDMA_MODULE_",""));
                if( currentChannel >= rm_instance.startIndex && currentChannel <= rm_instance.endIndex)
                {
                    error_status = false;
                    break;
                }
                else
                {
                    rangeConfig.push(system.getReference(rm_instance,"startIndex"));
                }
            }
        }
    }

    if(error_status == true)
    {
        report.logError(`Channel number does not fall in Resource Range `+rangeConfig.join(","), instance, "instance");
    }
}

function moduleInstances(instance) {
        return soc.supportXbarConfig(DMA_XBAR_LIST.find(o => o.name == instance.xbarOutput), instance);
}

exports = dma_trig_xbar_module;
