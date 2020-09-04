let common = system.getScript("/common");

const hwa_config_r5fss = [
    {
        name                        : "DSS_HWA0",
        ctrlBaseAddr                : "CSL_DSS_HWA_CFG_U_BASE",
        paramBaseAddr               : "CSL_DSS_HWA_PARAM_U_BASE",
        ramBaseAddr                 : "CSL_DSS_HWA_WINDOW_RAM_U_BASE",
        dssBaseAddr                 : "CSL_DSS_CTRL_U_BASE",
        numHwaParamSets             : "SOC_HWA_NUM_PARAM_SETS",
        intNum1ParamSet             : "CSL_MSS_INTR_DSS_HWA_PARAM_DONE_INTR1",
        intNum2ParamSet             : "CSL_MSS_INTR_DSS_HWA_PARAM_DONE_INTR2",
        intNumDone                  : "CSL_MSS_INTR_DSS_HWA_LOOP_INTR1",
        intNumDoneALT               : "CSL_MSS_INTR_DSS_HWA_LOOP_INTR2",
        intNumLocalRamErr           : "CSL_MSS_INTR_DSS_HWA_LOCAL_RAM_ERR",
        numDmaChannels              : "SOC_HWA_NUM_DMA_CHANNEL",
        accelMemBaseAddr            : "CSL_DSS_HWA_DMA0_U_BASE",
        accelMemSize                : "SOC_HWA_MEM_SIZE",
        isConcurrentAccessAllowed   : "true",
    },
];

const hwa_config_c66ss = [
    {
        name                        : "DSS_HWA0",
        ctrlBaseAddr                : "CSL_DSS_HWA_CFG_U_BASE",
        paramBaseAddr               : "CSL_DSS_HWA_PARAM_U_BASE",
        ramBaseAddr                 : "CSL_DSS_HWA_WINDOW_RAM_U_BASE",
        dssBaseAddr                 : "CSL_DSS_CTRL_U_BASE",
        numHwaParamSets             : "SOC_HWA_NUM_PARAM_SETS",
        intNum1ParamSet             : "CSL_DSS_INTR_DSS_HWA_PARAM_DONE_INTR1",
        intNum2ParamSet             : "CSL_DSS_INTR_DSS_HWA_PARAM_DONE_INTR2",
        intNumDone                  : "CSL_DSS_INTR_DSS_HWA_LOOP_INTR1",
        intNumDoneALT               : "CSL_DSS_INTR_DSS_HWA_LOOP_INTR2",
        intNumLocalRamErr           : "CSL_DSS_INTR_DSS_HWA_LOCAL_RAM_ERR",
        numDmaChannels              : "SOC_HWA_NUM_DMA_CHANNEL",
        accelMemBaseAddr            : "CSL_DSS_HWA_DMA0_U_BASE",
        accelMemSize                : "SOC_HWA_MEM_SIZE",
        isConcurrentAccessAllowed   : "true",
    },
];

function getConfigArr() {
    let hwa_config;

    if(common.getSelfSysCfgCoreName().includes("c66")) {
        hwa_config = hwa_config_c66ss;
    }
    else {
        hwa_config = hwa_config_r5fss;
    }

    return hwa_config;
}

exports = {
    getConfigArr,
};
