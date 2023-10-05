let common = system.getScript("/common");

function coreList() {
    let coreNames = common.getSysCfgCoreNames();
    // coreNames = coreNames.filter(function (coreName) {
    //     return coreName.includes("r5f");
    // });

    let selfCoreName = common.getSelfSysCfgCoreName();

    let tmp_list = coreNames.filter(function (coreName) {
        return coreName !== selfCoreName;
    });

    let options = [];

    for (let core of tmp_list)
    {
        let ele = {name: core, displayName: core}
        options.push(ele)
    }
	return options;
}

let config = [
    {
    name: "shared_cores",
    displayName: "Share with cores",
    description: "",
    longDescription: "",
    default: [], //computeCoreNum(),
    options: coreList,
},
]

let shared_module = {
    displayName: "Shared",
    defaultInstanceName: "CONFIG_SHARED",
    config : config,
}

exports = shared_module;