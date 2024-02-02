let common = system.getScript("/common");
let soc = system.getScript(`/drivers/gpio/soc/gpio_${common.getSocName()}`);

function migrate(inst){

    let props = [
        "$assign", "$suggestSolution", "$assignAllowConflicts", "pu_pd", "slewRate", "inv", "qualSel"
    ]

    _.each(props, ele => {
            if( inst[ele] !== undefined && inst[ele] !== ""){
            if (inst.$ownedBy.$module.name === "GPIO"){

                if([ "$assign", "$suggestSolution", "$assignAllowConflicts"].includes(ele)){
                    inst.$ownedBy.$ownedBy.GPIO_n[ele] = inst[ele]
                }
                else {
                    if(inst[ele] !== "")
                        inst.$ownedBy.$ownedBy[ele] = inst[ele]
                }
            }
        }
    })
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
        {
            name: "pu_pd",
            deprecated: true,
            default: "",
        },
        {
            name: "slewRate",
            deprecated: true,
            default: "",
        },
        {
            name: "inv",
            deprecated: true,
            default: "",
        },
        {
            name: "qualSel",
            deprecated: true,
            default: "",
        },
    ],
    migrateLegacyConfiguration: migrate,
}