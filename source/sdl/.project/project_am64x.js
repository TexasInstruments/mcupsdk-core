let path = require('path');

let device = "am64x";

const files = {
    common: [
        "sdl_dpl.c",
        "sdl_mcrc.c",
        "sdl_ip_mcrc.c",
        "sdl_mcrc_soc.c",
        "sdl_esm.c",
        "sdl_ip_esm.c",
        "sdl_esm_core.c",
        "sdl_esm_priv.c",
        "sdl_dcc.c",
        "sdl_soc_dcc.c",
        "sdl_ip_tog.c",
        "sdl_tog.c",
        "sdl_soc_tog.c",
        "sdl_ip_vtm.c",
		"sdl_vtm_pvt_sensor.c",
		"sdl_vtm.c",
		"sdl_soc_vtm.c",
    ],
};

const filedirs = {
    common: [
        "dpl",
        "mcrc",
        "mcrc/v0",
        "mcrc/v0/soc/am64x",
        "esm",
        "esm/soc",
        "esm/soc/am64x",
        "esm/v0",
        "esm/v0/v0_0",
        "dcc",
        "dcc/v0",
        "dcc/v0/soc",
        "dcc/v0/soc/am64x",
        "stog/v0",
        "stog/v0/soc",
        "stog/v0/soc/am64x",
        "vtm",
        "vtm/v0",
        "vtm/v0/soc/am64x",
    ],
};

const r5_macro = {
    common: [
        "R5F_CORE",
    ],

};

const m4_macro = {
    common: [
        "M4F_CORE",
    ],

};

const buildOptionCombos = [
    { device: device, cpu: "m4f", cgt: "ti-arm-clang"},
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "sdl";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.filedirs = filedirs;
    build_property.files = files;
    if(buildOption.cpu.match(/m4f*/)) {
        build_property.defines = m4_macro;
    }
    if(buildOption.cpu.match(/r5f*/)) {
        build_property.defines = r5_macro;
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};