let common = system.getScript("/common");

const esm_config_r5fss = [
    {
        name                : "MSS_ESM",
        regBaseAddr         : "CSL_MSS_ESM_U_BASE",
        ctrlBaseAddr        : "CSL_MSS_CTRL_U_BASE",
        numGroupErr         : "ESM_NUM_INTR_PER_GROUP",              
        highPrioIntNum      : "CSL_MSS_INTR_MSS_ESM_HI",
        lowPrioIntNum       : "CSL_MSS_INTR_MSS_ESM_LO",
        intHighPriority     : 0xF,
        intLowPriority      : 0x8,
    },
];

const esm_config_c66ss = [
   {
        name                : "DSS_ESM",
        regBaseAddr         : "CSL_DSS_ESM_U_BASE",
        ctrlBaseAddr        : "CSL_DSS_CTRL_U_BASE",
        numGroupErr         : "ESM_NUM_INTR_PER_GROUP", 
        highPrioIntNum      : "CSL_DSS_INTR_DSS_ESM_HI",    
        lowPrioIntNum       : "CSL_DSS_INTR_DSS_ESM_LO",
        intHighPriority     : 0xF,
        intLowPriority      : 0x1,
    },
];

function getConfigArr() {
    let esm_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        esm_config = esm_config_c66ss;
    }
    else {
        esm_config = esm_config_r5fss;
    }

    return esm_config;
}

function getMaxNotifier() {
   /* Max number of notifer ESM instance can support is 4 */
    return 4;
}

exports = {
    getConfigArr,
    getMaxNotifier,
};