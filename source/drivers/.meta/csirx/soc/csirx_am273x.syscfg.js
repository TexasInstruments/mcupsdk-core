let common = system.getScript("/common");

const csirx_config = [
    {
        name            : "CSIRX A",
    },
    {
        name            : "CSIRX B",
    },
];

function getStaticConfigArr() {
    return csirx_config;
}

function getInstanceHwAttrs(inst) {

    let hwAttrsName = "csirxA";

    if(inst.instance=="CSIRX B") {
        hwAttrsName = "csirxB";
    }

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
