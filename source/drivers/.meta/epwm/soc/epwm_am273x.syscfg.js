
let common = system.getScript("/common");

let epwm_func_clk = 200 * 1000 * 1000;


const staticConfig = [
    {
        name: "MSS_EPWMA",
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
        name: "MSS_EPWMB",
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
        name: "MSS_EPWMC",
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
   
        interfaceName = "MSS_EPWM";
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
