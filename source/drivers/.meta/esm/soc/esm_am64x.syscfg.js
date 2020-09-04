let common = system.getScript("/common");

const esm_config_main = [
    {
        name                : "MAIN_ESM",
        regBaseAddr         : "CSL_ESM0_CFG_BASE",
        highPrioIntNum      : "CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_HI_LVL_0",
        lowPrioIntNum       : "CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_LOW_LVL_0",
        intHighPriority     : 0xF,
        intLowPriority      : 0x8,
    },
];

function getConfigArr() {
    let esm_config;

    if(common.getSelfSysCfgCoreName().includes("r5f") ||
            common.getSelfSysCfgCoreName().includes("a53") )
    {
        esm_config = esm_config_main;
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
