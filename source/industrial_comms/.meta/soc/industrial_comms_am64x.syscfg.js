
let common = system.getScript("/common");

const topModules_main = [
    "/industrial_comms/ethercat/ethercat",
    "/industrial_comms/ethernetip/ethernetip",
    "/industrial_comms/hsr_prp/hsr_prp",
    "/industrial_comms/profinet/profinet",
];

const topModules_mcu = [

];

const topModules_a53 = [

];

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        if(common.getSelfSysCfgCoreName().includes("m4f")) {
            topModules = topModules_mcu;
        }
        if (common.getSelfSysCfgCoreName().match(/a53*/))
        {
            topModules = topModules_a53;
        }

        return topModules;
    },
};
