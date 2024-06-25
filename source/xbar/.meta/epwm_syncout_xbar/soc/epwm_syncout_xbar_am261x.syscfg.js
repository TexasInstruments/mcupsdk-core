let common = system.getScript("/common");
let xbarSoc = system.getScript(`/xbar/soc/xbar_${common.getSocName()}`);

const internal_list = [
    {  name: "EPWM0_SYNCOUT", displayName: "EPWM0_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM1_SYNCOUT", displayName: "EPWM1_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM2_SYNCOUT", displayName: "EPWM2_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM3_SYNCOUT", displayName: "EPWM3_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM4_SYNCOUT", displayName: "EPWM4_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM5_SYNCOUT", displayName: "EPWM5_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM6_SYNCOUT", displayName: "EPWM6_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM7_SYNCOUT", displayName: "EPWM7_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM8_SYNCOUT", displayName: "EPWM8_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM9_SYNCOUT", displayName: "EPWM9_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM10_SYNCOUT", displayName: "EPWM10_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM11_SYNCOUT", displayName: "EPWM11_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM12_SYNCOUT", displayName: "EPWM12_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM13_SYNCOUT", displayName: "EPWM13_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM14_SYNCOUT", displayName: "EPWM14_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM15_SYNCOUT", displayName: "EPWM15_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM16_SYNCOUT", displayName: "EPWM16_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM17_SYNCOUT", displayName: "EPWM17_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM18_SYNCOUT", displayName: "EPWM18_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM19_SYNCOUT", displayName: "EPWM19_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM20_SYNCOUT", displayName: "EPWM20_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM21_SYNCOUT", displayName: "EPWM21_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM22_SYNCOUT", displayName: "EPWM22_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM23_SYNCOUT", displayName: "EPWM23_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM24_SYNCOUT", displayName: "EPWM24_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM25_SYNCOUT", displayName: "EPWM25_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM26_SYNCOUT", displayName: "EPWM26_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM27_SYNCOUT", displayName: "EPWM27_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM28_SYNCOUT", displayName: "EPWM28_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM29_SYNCOUT", displayName: "EPWM29_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM30_SYNCOUT", displayName: "EPWM30_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "EPWM31_SYNCOUT", displayName: "EPWM31_SYNCOUT", path: "epwm_syncout_xbar", group: 0 },
    {  name: "ECAP0_SYNCOUT", displayName: "ECAP0_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP1_SYNCOUT", displayName: "ECAP1_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP2_SYNCOUT", displayName: "ECAP2_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP3_SYNCOUT", displayName: "ECAP3_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP4_SYNCOUT", displayName: "ECAP4_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP5_SYNCOUT", displayName: "ECAP5_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP6_SYNCOUT", displayName: "ECAP6_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP7_SYNCOUT", displayName: "ECAP7_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP8_SYNCOUT", displayName: "ECAP8_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP9_SYNCOUT", displayName: "ECAP9_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP10_SYNCOUT", displayName: "ECAP10_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP11_SYNCOUT", displayName: "ECAP11_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP12_SYNCOUT", displayName: "ECAP12_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP13_SYNCOUT", displayName: "ECAP13_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP14_SYNCOUT", displayName: "ECAP14_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
    {  name: "ECAP15_SYNCOUT", displayName: "ECAP15_SYNCOUT", path: "epwm_syncout_xbar", group: 1 },
];

let xbarProperties = {
    masterXbarList: [],
    outputInstanceList: [
        { name: "EPWM_SYNCOUT_XBAR", count: 4},
    ],
    duplicatesPresent: false,
    moduleString: "epwm_syncout_xbar",
}

function getOptionList(calledBy) {
    return xbarSoc.getOptionListSoc(calledBy, xbarProperties, internal_list);
}

function getConfigArr() {
    return xbarSoc.getXbarInstanceConfig(xbarProperties);
}

exports = {
    getConfigArr,
    getOptionList,
};
