let common = system.getScript("/common");
let xbarSoc = system.getScript(`/xbar/soc/xbar_${common.getSocName()}`);

const internal_list = [
    {   name: "SOC_TIMESYNC_XBAR1_CPTS_COMP", displayName: "CPTS_COMP", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_CPTS_GENF0", displayName: "CPTS_GENF0", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_CPTS_GENF1", displayName: "CPTS_GENF1", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_CPTS_SYNC", displayName: "CPTS_SYNC", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_PR1_EDC0_SYNC_OUT_0", displayName: "PR1_EDC0_SYNC_OUT_0", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_PR1_EDC0_SYNC_OUT_1", displayName: "PR1_EDC0_SYNC_OUT_1", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_PWM_SYNCOUT_XBAR_OUT_0", displayName: "PWM_SYNCOUT_XBAR_OUT_0", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_PWM_SYNCOUT_XBAR_OUT_1", displayName: "PWM_SYNCOUT_XBAR_OUT_1", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_PWM_SYNCOUT_XBAR_OUT_2", displayName: "PWM_SYNCOUT_XBAR_OUT_2", path: "soc_timesync_xbar1" },
    {   name: "SOC_TIMESYNC_XBAR1_PWM_SYNCOUT_XBAR_OUT_3", displayName: "PWM_SYNCOUT_XBAR_OUT_3", path: "soc_timesync_xbar1" },
];

let xbarProperties = {
    masterXbarList: ["gpio_int_xbar"],
    outputInstanceList: [
        { name: "SOC_TIMESYNC_XBAR1_DMA_TRIG_XBAR",   count: 2},
        { name: "SOC_TIMESYNC_XBAR1_VIM_MODULE0",  count: 4},
        { name: "SOC_TIMESYNC_XBAR1_VIM_MODULE1",  count: 4},
        { name: "SOC_TIMESYNC_XBAR1_ICSS_MODULE", count: 8},
        { name: "SOC_TIMESYNC_XBAR1_CPSW_MODULE", count: 8},
        { name: "SOC_TIMESYNC_XBAR1_VIM_MODULE2",  count: 4},
        { name: "SOC_TIMESYNC_XBAR1_VIM_MODULE3",  count: 4},
    ],
    duplicatesPresent: false,
    moduleString: "soc_timesync_xbar1",
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
