let common = system.getScript("/common");
let soc = system.getScript(`/drivers/gpio/soc/gpio_${common.getSocName()}`);
let pinmux = system.getScript(`/drivers/pinmux/pinmux_${common.getSocName()}`);

function addModuleInstances(inst) {
    let modInstances = new Array();

    modInstances.push({
        name: "gpioPin",
        displayName: "GPIO GRANDCHILD",
        moduleName: "/drivers/gpio/v0/gpio_grand_child",
        requiredArgs: {$name: inst.$name + "_grandchild"}
    })

    return modInstances;
}

exports = {
    name: "GPIO",
    displayName: "GPIO CHILD",
    config:[
        {
            name: "$assign",
            deprecated: true,
            default: "",
        },
        {
            name: "$suggestSolution",
            deprecated: true,
            default: "",
        },
        {
            name: "$assignWithConflicts",
            deprecated: true,
            default: "",
        },
    ],
    moduleInstances: addModuleInstances,
}