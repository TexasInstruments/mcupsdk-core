let common = system.getScript("/common");
let soc = system.getScript(`/drivers/gpio/soc/gpio_${common.getSocName()}`);

function migrate(inst){

    if( inst.$assign.length > 0 ){
        if (inst.$ownedBy == "GPIO")
            inst.$ownedBy.$ownedBy.GPIO_n.$assign = inst.$assign
        // else if (inst.$ownedBy == "MCU_GPIO")
        //     inst.$ownedBy.$ownedBy.MCU_GPIO_n.$assign = inst.$assign
    }

    if( inst.$suggestSolution.length > 0 ){
        if (inst.$ownedBy == "GPIO")
            inst.$ownedBy.$ownedBy.GPIO_n.$suggestSolution = inst.$suggestSolution
        // else if (inst.$ownedBy == "MCU_GPIO")
        //     inst.$ownedBy.$ownedBy.MCU_GPIO_n.$suggestSolution = inst.$suggestSolution
    }

}

exports = {
    name: "gpioPin",
    displayName: "GPIO GRAND CHILD",
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
    migrateLegacyConfiguration: migrate,
}