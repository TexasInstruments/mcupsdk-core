
let common = system.getScript("/common");
let soc = system.getScript(`/xbar/dma_xbar/soc/dma_xbar_${common.getSocName()}`);

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
    delete item.group;
    return item;
});

let dma_xbar_module = {
    displayName: "DMA XBAR",
    longDescription : '![](../source/xbar/.meta/images/'+common.getSocName()+'/dma_xbar.png)',
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/xbar/dma_xbar/templates/dma_xbar.h.xdt",
        },
        "/drivers/system/drivers_open_close.h.xdt": {
            driver_open_close_config: "/xbar/dma_xbar/templates/dma_xbar_open_close_config.h.xdt",
        },
        "/drivers/system/drivers_open_close.c.xdt": {
            driver_open_close_config: "/xbar/dma_xbar/templates/dma_xbar_open_close_config.c.xdt",
            driver_open: "/xbar/dma_xbar/templates/dma_xbar_open.c.xdt",
        },
    },
    maxInstances: getConfigArr().length,
    defaultInstanceName: "CONFIG_DMA_XBAR",
    config: [
        {
            name: "xbarOutput",
            displayName: "XBAR Output",
            default: DMA_XBAR[0].name,
            options: DMA_XBAR,
            description: "This determines the output of the xbar",
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
    validate: validate,
    getInstanceConfig,
};

function validate(instance, report) {
    common.validate.checkSameInstanceNameOnAllCores('/xbar/dma_xbar/dma_xbar', instance, report);
}

exports = dma_xbar_module;
