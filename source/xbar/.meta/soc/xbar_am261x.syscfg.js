
let common = system.getScript("/common");

const topModules_main = [
    "/xbar/gpio_int_xbar/gpio_int_xbar",
    "/xbar/dma_trig_xbar/dma_trig_xbar",
    "/xbar/dma_xbar/dma_xbar",
    "/xbar/epwm_syncout_xbar/epwm_syncout_xbar",
    "/xbar/epwm_xbar/epwm_xbar",
    "/xbar/icl_xbar/icl_xbar",
    "/xbar/icss_xbar/icss_xbar",
    "/xbar/input_xbar/input_xbar",
    "/xbar/int_xbar/int_xbar",
    "/xbar/mdl_xbar/mdl_xbar",
    "/xbar/output_xbar/output_xbar",
    "/xbar/soc_timesync_xbar0/soc_timesync_xbar0",
    "/xbar/soc_timesync_xbar1/soc_timesync_xbar1",
];

function getOptionListSoc(calledBy, xbarProperties, internal_list) {
    var externalOptions = [];

    if(calledBy == "EXTERNAL")
    {
        if(internal_list[0].hasOwnProperty("group") == true)
        {
            let internal_list_updated = JSON.parse(JSON.stringify(internal_list));
            //property "group" has to be removed
            internal_list_updated.map(function(item) {
                delete item.group;
                return item;
            });
            internal_list = internal_list_updated;
        }
    }

    for(var masterXbarIndex in xbarProperties.masterXbarList)
    {
        let moduleName = xbarProperties.masterXbarList[masterXbarIndex];
        let xbarModule = system.getScript(`/xbar/${moduleName}/soc/${moduleName}_${common.getSocName()}`);
        externalOptions = externalOptions.concat(JSON.parse(JSON.stringify(xbarModule.getOptionList("EXTERNAL"))));
    }

    if(xbarProperties.duplicatesPresent == true)
    {
        //check for duplicates
        let duplicateCount = 0;
        for(let i = 0; i < externalOptions.length; i++)
        {
            duplicateCount = 0;
            for(let j = i+1; j < externalOptions.length; j++)
            {
                if(externalOptions[i].name == externalOptions[j].name)
                {
                    externalOptions[j].name = externalOptions[j].name + "###" + duplicateCount;
                    duplicateCount++;
                }
            }
        }
    }

    // add path to name
    if(calledBy == "INTERNAL")
    {
        for(let i = 0; i < externalOptions.length; i++)
        {
            externalOptions[i].displayName = externalOptions[i].displayName + " ( '''' " + externalOptions[i].path + " '''' ) " ;
        }
    }

    for (var optionsIndex in externalOptions)
    {
        var path = externalOptions[optionsIndex].path;
        externalOptions[optionsIndex].path = xbarProperties.moduleString +'<-' + path;
    }

    return internal_list.concat(externalOptions);
}

function supportXbarConfigSoc(outputSelected, instance, xbarProperties) {
    let modInstances = new Array();
    let path = outputSelected.path;
    if(path.search("<-") != -1)
    {
        let xbarName = path.substring(xbarProperties.moduleString.length + 2, path.length);
        let xbarPos = xbarName.search("<-");

        if(xbarPos != -1)
        {
            xbarName = xbarName.substring(0, xbarPos).toLowerCase();
        }

        let xbarOutput = outputSelected.name;
        if(xbarProperties.duplicatesPresent == true)
        {
            if(xbarOutput.search("###") != -1)
            {
                xbarOutput = xbarOutput.substring(0, xbarOutput.length-4);
            }
        }
        modInstances.push({
            name: "xbarConfig",
            displayName: xbarName.toUpperCase(),
            moduleName: `/xbar/${xbarName}/${xbarName}`,
            requiredArgs: {
                xbarOutput: xbarOutput,
            }
        });
    }
    return modInstances;
}

function getXbarInstanceConfig(xbarProperties) {
    let xbar_config = [];
    let instCount = 0;
    for( let instanceInfoCount in xbarProperties.outputInstanceList)
    {
        let instanceInfo = xbarProperties.outputInstanceList[instanceInfoCount];
        for(let i = 0; i < instanceInfo.count; i++)
        {
            xbar_config[instCount] = { name: instanceInfo.name +"_" + i}
            instCount++;
        }
    }
    return xbar_config;
}

exports = {
    getTopModules: function() {

        let topModules = topModules_main;
        return common.getSelfSysCfgCoreName().includes("r5f")?topModules_main:[];
        return topModules;
    },
    getOptionListSoc,
    supportXbarConfigSoc,
    getXbarInstanceConfig,
};
