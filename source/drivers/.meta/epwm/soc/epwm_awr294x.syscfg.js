
let common = system.getScript("/common");

let epwm_func_clk = 200 * 1000 * 1000;


const staticConfig = [
    {
        name: "EPWMA",
        baseAddr: "CSL_MSS_ETPWMA_U_BASE",
        intrNum: 38,
        tripIntrNum: 39,
        isSyncoPresent: true,
        isSynciPresent: true,
		funcClk: epwm_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_MSS_RTIA" ],
		clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_RTIA",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : epwm_func_clk,
            },
        ],
       
    },
    {
        name: "EPWMB",
        baseAddr: "CSL_MSS_ETPWMB_U_BASE",
        intrNum: 40,
        tripIntrNum: 41,
        isSyncoPresent: true,
        isSynciPresent: true,
		funcClk: epwm_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_MSS_RTIB" ],
		clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_RTIB",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : epwm_func_clk,
            },
        ],
        
    },
    {
        name: "EPWMC",
        baseAddr: "CSL_MSS_ETPWMC_U_BASE",
        intrNum: 42,
        tripIntrNum: 43,
        isSyncoPresent: true,
        isSynciPresent: true,
		funcClk: epwm_func_clk,
        clockIds: [ "SOC_RcmPeripheralId_MSS_RTIC" ],
		clockFrequencies: [
            {
                moduleId: "SOC_RcmPeripheralId_MSS_RTIC",
                clkId   : "SOC_RcmPeripheralClockSource_SYS_CLK",
                clkRate : epwm_func_clk,
            },
        ],
        
    },  
	 
];
function getInterfaceName(inst) {

    let interfaceName;
   
        interfaceName = "EPWM";
         return interfaceName;

}

function getStaticConfigArr() {
    return staticConfig;
}



let soc = {
    getStaticConfigArr,
	getInterfaceName,
};

exports = soc;
