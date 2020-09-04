let common = system.getScript("/common");

const csirx_config = [
    {
        name            : "CSIRX A",
    },
];

function getStaticConfigArr() {
    return csirx_config;
}

function getInstanceHwAttrs(inst) {
    let hwAttrsName = "csirxA";

    if(common.getSelfSysCfgCoreName().match("r5fss")) {
        hwAttrsName += "_r5f";
    } else {
        hwAttrsName += "_c66";
    }
    return hwAttrsName;
}

exports = {
    getStaticConfigArr,
    getInstanceHwAttrs,
};
