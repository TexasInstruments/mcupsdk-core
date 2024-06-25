let common = system.getScript("/common");
let xbarSoc = system.getScript(`/xbar/soc/xbar_${common.getSocName()}`);

const internal_list = [
    {   name: "SOC_TIMESYNC_XBAR0_CPTS_COMP", displayName: "CPTS_COMP", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_CPTS_GENF0", displayName: "CPTS_GENF0", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_CPTS_GENF1", displayName: "CPTS_GENF1", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_CPTS_SYNC", displayName: "CPTS_SYNC", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_EDC0_SYNC_OUT_0", displayName: "PR1_EDC0_SYNC_OUT_0", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_EDC0_SYNC_OUT_1", displayName: "PR1_EDC0_SYNC_OUT_1", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_0", displayName: "PR1_IEP0_CMP_INTR_REQ_0", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_1", displayName: "PR1_IEP0_CMP_INTR_REQ_1", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_2", displayName: "PR1_IEP0_CMP_INTR_REQ_2", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_3", displayName: "PR1_IEP0_CMP_INTR_REQ_3", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_4", displayName: "PR1_IEP0_CMP_INTR_REQ_4", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_5", displayName: "PR1_IEP0_CMP_INTR_REQ_5", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_6", displayName: "PR1_IEP0_CMP_INTR_REQ_6", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_7", displayName: "PR1_IEP0_CMP_INTR_REQ_7", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_8", displayName: "PR1_IEP0_CMP_INTR_REQ_8", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_9", displayName: "PR1_IEP0_CMP_INTR_REQ_9", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_10", displayName: "PR1_IEP0_CMP_INTR_REQ_10", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_11", displayName: "PR1_IEP0_CMP_INTR_REQ_11", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_12", displayName: "PR1_IEP0_CMP_INTR_REQ_12", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_13", displayName: "PR1_IEP0_CMP_INTR_REQ_13", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_14", displayName: "PR1_IEP0_CMP_INTR_REQ_14", path: "soc_timesync_xbar0" },
    {   name: "SOC_TIMESYNC_XBAR0_PR1_IEP0_CMP_INTR_REQ_15", displayName: "PR1_IEP0_CMP_INTR_REQ_15", path: "soc_timesync_xbar0" },
];

let xbarProperties = {
    masterXbarList: [],
    outputInstanceList: [
        { name: "SOC_TIMESYNC_XBAR0_EPWM_MODULE",  count: 2},
        { name: "SOC_TIMESYNC_XBAR0_RTI_MODULE",  count: 8},
        { name: "SOC_TIMESYNC_XBAR0_DMA_TRIG_XBAR",   count: 2},
    ],
    duplicatesPresent: false,
    moduleString: "soc_timesync_xbar0",
}

function getOptionList(calledBy) {
    return xbarSoc.getOptionListSoc(calledBy, xbarProperties, internal_list);
}

function getConfigArr() {
    return xbarSoc.getXbarInstanceConfig(xbarProperties);
}

function supportXbarConfig(outputSelected, instance) {
    return xbarSoc.supportXbarConfigSoc(outputSelected, instance, xbarProperties);
}

exports = {
    getConfigArr,
    getOptionList,
    supportXbarConfig,
};
